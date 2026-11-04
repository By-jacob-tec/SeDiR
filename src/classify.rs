//! Recognising the kind of a signal: a short stretch of IQ around it is measured
//! (occupied band, carrier, one-sidedness, envelope, keying) and the measurements,
//! with the band plan as a tie-breaker, give the mode and where to tune.

use std::sync::mpsc::{self, Receiver, RecvTimeoutError};
use std::thread;
use std::time::{Duration, Instant};

use num_complex::Complex;

use crate::bandplan;
use crate::demod::{FirDecimator, Mixer, Mode, halfband, lowpass, taps_for};
use crate::dsp::Spectrum;
use crate::source::Block;

/// Signal analysed (seconds): enough speech for the SSB pitch search.
const ANALYSIS_SECONDS: f64 = 1.2;
/// Spectra averaged over that time.
const SPECTRA: f64 = 8.0;

/// Why a mode was chosen; the UI names these in its own language.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reason {
    /// Wider than a narrow-band channel.
    Wide,
    /// The amplitude stays constant: frequency modulation.
    ConstantEnvelope,
    /// A carrier with sidebands on both sides.
    CarrierAndSidebands,
    /// Energy on one side only, strongest near the low edge.
    VoiceAboveCarrier,
    /// Energy on one side only, strongest near the high edge.
    VoiceBelowCarrier,
    /// A single narrow carrier.
    NarrowCarrier,
    /// The carrier is switched on and off.
    Keyed,
    /// The band plan decided between close candidates.
    BandPlan,
    /// The SSB carrier was placed from the harmonics of the speaker's voice.
    VoicePitch,
}

/// Measurements of a signal (frequencies relative to the analysis centre, Hz).
#[derive(Clone, Copy, Debug, Default)]
pub struct Features {
    /// Width of everything 6 dB over the noise (tells wide signals apart).
    pub outer_width_hz: f64,
    /// Edges of the band holding 99.9 % of the power.
    pub lo_hz: f64,
    pub hi_hz: f64,
    pub peak_hz: f64,
    pub snr_db: f32,
    /// Share of the signal's power in its strongest line.
    pub carrier_fraction: f32,
    /// Power above the strongest line divided by power below it.
    pub balance: f32,
    /// Power centroid as a fraction of the occupied band (0 = low edge).
    pub centroid: f32,
    /// Standard deviation of the amplitude over its mean.
    pub envelope_cv: f32,
    /// Share of time the (smoothed) amplitude is under a third of its peak.
    pub off_fraction: f32,
}

impl Features {
    pub fn width(&self) -> f64 {
        self.hi_hz - self.lo_hz
    }
}

/// The verdict.
#[derive(Clone, Debug, PartialEq)]
pub struct Classification {
    pub mode: Mode,
    /// 0..1.
    pub confidence: f32,
    /// Where to listen (Hz, absolute): the carrier for AM, CW and SSB, the channel for FM.
    pub frequency_hz: f64,
    pub bandwidth_hz: f64,
    pub reasons: Vec<Reason>,
}

/// Measures `samples` (complex baseband at `rate`, the signal near 0 Hz). The signal is
/// looked for only within `focus_hz` of the centre: the analysis band also holds the
/// neighbours, and a stronger one (a beacon 18 kHz away) must not be taken instead.
pub fn measure(samples: &[Complex<f32>], rate: f64, focus_hz: f64) -> Option<Features> {
    let n = ((rate * ANALYSIS_SECONDS / SPECTRA) as usize).next_power_of_two().clamp(512, 1 << 16);
    if samples.len() < 2 * n {
        return None;
    }
    let mut spectrum = Spectrum::new(n);
    spectrum.push(samples);
    let db = spectrum.take()?;
    let bin = rate / n as f64;
    let freq = |i: usize| (i as f64 - n as f64 / 2.0) * bin;
    let power: Vec<f64> = db.iter().map(|&d| 10f64.powf(d as f64 / 10.0)).collect();
    let mut sorted = power.clone();
    sorted.sort_by(f64::total_cmp);
    let noise = sorted[sorted.len() / 5].max(1e-30);

    // The occupied band grows from the strongest bin while bins stand 6 dB over the noise,
    // across gaps of up to 1 % of the analysis band.
    let reach = ((focus_hz / bin) as usize).clamp(2, n / 2 - 1);
    let peak = (n / 2 - reach..=n / 2 + reach).max_by(|&a, &b| power[a].total_cmp(&power[b]))?;
    let above = |i: usize| power[i] > noise * 4.0;
    let gap = (n / 100).max(3);
    let grow = |step: isize| {
        let (mut edge, mut i, mut missed) = (peak as isize, peak as isize, 0usize);
        loop {
            i += step;
            if i < 0 || i >= n as isize {
                break edge as usize;
            }
            if above(i as usize) {
                edge = i;
                missed = 0;
            } else {
                missed += 1;
                if missed > gap {
                    break edge as usize;
                }
            }
        }
    };
    let (lo, hi) = (grow(-1), grow(1));
    let outer_width = (hi - lo + 1) as f64 * bin;
    let excess = |i: usize| (power[i] - noise).max(0.0);
    // Within that, the 99.9 % power band: skirts (window leakage, key clicks) of strong
    // signals carry little power but would stretch the edges, while AM sidebands next to
    // a strong carrier (a few % of the power) are kept.
    let total: f64 = (lo..=hi).map(excess).sum::<f64>().max(1e-30);
    let mut running = 0.0;
    let (mut lo99, mut hi99) = (lo, hi);
    let mut found_lo = false;
    for i in lo..=hi {
        running += excess(i);
        if !found_lo && running >= total * 0.0005 {
            lo99 = i;
            found_lo = true;
        }
        if running >= total * 0.9995 {
            hi99 = i;
            break;
        }
    }
    let (lo, hi) = (lo99.min(peak), hi99.max(peak));
    let total: f64 = (lo..=hi).map(excess).sum::<f64>().max(1e-30);
    let line: f64 = (peak.saturating_sub(1)..=(peak + 1).min(n - 1)).map(excess).sum();
    let below: f64 = (lo..peak.saturating_sub(2)).map(excess).sum();
    let above_sum: f64 = ((peak + 3).min(hi + 1)..=hi).map(excess).sum();
    let centroid_bin = (lo..=hi).map(|i| excess(i) * i as f64).sum::<f64>() / total;
    let width_bins = (hi - lo).max(1) as f64;

    // Envelope of the occupied band alone.
    let center = (freq(lo) + freq(hi)) / 2.0;
    let half = (freq(hi) - freq(lo)) / 2.0 + bin;
    let mut shifted = samples.to_vec();
    Mixer::new(-center, rate).mix(&mut shifted);
    let mut band = Vec::new();
    FirDecimator::new(&lowpass((half / rate).clamp(0.001, 0.45), taps_for(rate, (half * 0.3).max(30.0), 2001)), 1)
        .process(&shifted, &mut band);
    let band = &band[band.len().min(2001)..];
    let env: Vec<f32> = band.iter().map(|z| z.norm()).collect();
    let mean = env.iter().sum::<f32>() / env.len().max(1) as f32;
    let var = env.iter().map(|e| (e - mean) * (e - mean)).sum::<f32>() / env.len().max(1) as f32;
    // Keying shows in the amplitude smoothed over ~10 ms.
    let smooth = ((rate * 0.01) as usize).max(1);
    let slow: Vec<f32> = env.chunks(smooth).map(|c| c.iter().sum::<f32>() / c.len() as f32).collect();
    let top = slow.iter().copied().fold(0.0f32, f32::max);
    let off = slow.iter().filter(|&&v| v < top / 3.0).count() as f32 / slow.len().max(1) as f32;

    Some(Features {
        outer_width_hz: outer_width,
        lo_hz: freq(lo) - bin / 2.0,
        hi_hz: freq(hi) + bin / 2.0,
        peak_hz: freq(peak),
        snr_db: (10.0 * (power[peak] / noise).log10()) as f32,
        carrier_fraction: (line / total) as f32,
        balance: ((above_sum + 1e-30) / (below + 1e-30)) as f32,
        centroid: ((centroid_bin - lo as f64) / width_bins) as f32,
        envelope_cv: if mean > 0.0 { var.sqrt() / mean } else { 1.0 },
        off_fraction: off,
    })
}

/// Decides the mode. `center_hz` is the absolute frequency of the analysis centre.
pub fn decide(f: &Features, center_hz: f64) -> Classification {
    let hint = bandplan::hint(center_hz + f.peak_hz);
    let width = f.width();
    let abs = |rel: f64| center_hz + rel;
    let snr = 10f32.powf(f.snr_db / 10.0).max(1.0);
    // An FM signal's envelope varies only by the noise riding on it.
    let constant_envelope = f.envelope_cv < 0.12 + 0.8 / snr.sqrt();
    let one_sided = f.balance > 4.0 || f.balance < 0.25;
    let mut reasons = Vec::new();
    let pick = |mode: Mode| (mode, hint == Some(mode));
    // Switched on and off: CW keying (AM and FM transmitters stay on while they speak).
    let keyed = f.off_fraction > 0.1 && f.off_fraction < 0.9;
    // AM is expected only where the band plan has it, or where it says nothing.
    let am_plausible = matches!(hint, None | Some(Mode::Am));

    // Broadcast FM is narrow during quiet passages; its outer width and the band plan
    // still give it away.
    let wide = f.outer_width_hz.max(width);
    let (mode, confidence, frequency, bandwidth) = if wide >= 60e3 || (hint == Some(Mode::Wfm) && constant_envelope && width >= 15e3) {
        reasons.push(Reason::Wide);
        if constant_envelope {
            reasons.push(Reason::ConstantEnvelope);
        }
        let mode = if wide >= 100e3 || hint == Some(Mode::Wfm) { Mode::Wfm } else { Mode::Nfm };
        let center = abs((f.lo_hz + f.hi_hz) / 2.0);
        let bw = if mode == Mode::Wfm { 180e3 } else { 25e3 };
        (mode, if constant_envelope { 0.9 } else { 0.7 }, mode.round_to_channel(center), bw)
    } else if width < 400.0
        || (f.carrier_fraction > 0.75 && width < 1.5e3)
        // A keyed carrier is CW, whatever the converter's phase noise spreads around it
        // (on QO-100 that looked like AM sidebands).
        || (keyed && f.carrier_fraction > 0.3 && width < 6e3)
        // A carrier-dominated signal where AM is not used (amateur SSB/CW segments,
        // QO-100) is CW or an unmodulated carrier, not AM.
        || (f.carrier_fraction > 0.8 && !am_plausible && width < 6e3)
    {
        // A bare carrier: CW, or an AM/FM transmitter without modulation.
        reasons.push(Reason::NarrowCarrier);
        if keyed {
            reasons.push(Reason::Keyed);
        }
        let (mode, by_plan) = match hint {
            Some(m @ (Mode::Am | Mode::Nfm)) if !keyed => (m, true),
            _ => pick(Mode::Cw),
        };
        if by_plan && mode != Mode::Cw {
            reasons.push(Reason::BandPlan);
        }
        let frequency = mode.round_to_channel(abs(f.peak_hz));
        (mode, if keyed || by_plan { 0.85 } else { 0.6 }, frequency, mode.default_bandwidth())
    } else if am_plausible && (f.carrier_fraction > 0.5 || (f.carrier_fraction > 0.2 && !constant_envelope)) && !one_sided {
        // A modulated FM signal rarely keeps half its power in one line; AM always does
        // (the carrier carries at least two thirds of it).
        reasons.push(Reason::CarrierAndSidebands);
        let half = (f.peak_hz - f.lo_hz).max(f.hi_hz - f.peak_hz);
        (Mode::Am, 0.8, Mode::Am.round_to_channel(abs(f.peak_hz)), (2.0 * half + 500.0).clamp(3e3, 12e3))
    } else if constant_envelope && width >= 3e3 {
        reasons.push(Reason::ConstantEnvelope);
        let center = abs((f.lo_hz + f.hi_hz) / 2.0);
        (Mode::Nfm, 0.8, Mode::Nfm.round_to_channel(center), (width * 1.3).clamp(8e3, 25e3))
    } else if width <= 4.5e3 || (matches!(hint, Some(Mode::Usb | Mode::Lsb)) && width <= 8e3) {
        // Single sideband (splatter or a neighbour can make it look wider where the band
        // plan expects it). The sideband is a convention of the band (LSB below 10 MHz on
        // the amateur bands, USB above and on QO-100), which the band plan knows; the
        // spectrum alone is a weak guide, as compressed or pre-emphasised speech is not
        // reliably strongest beside the carrier (on QO-100 it pointed the wrong way). It
        // is used only where the band plan says nothing, and only when clear.
        let upper = match hint {
            Some(Mode::Usb) => {
                reasons.push(Reason::BandPlan);
                true
            }
            Some(Mode::Lsb) => {
                reasons.push(Reason::BandPlan);
                false
            }
            _ if f.centroid < 0.3 => {
                reasons.push(Reason::VoiceAboveCarrier);
                true
            }
            _ if f.centroid > 0.7 => {
                reasons.push(Reason::VoiceBelowCarrier);
                false
            }
            _ => {
                reasons.push(Reason::BandPlan);
                center_hz >= 10e6
            }
        };
        let bw = (width + 150.0).clamp(1.8e3, 3.6e3);
        if upper {
            (Mode::Usb, 0.7, abs(f.lo_hz) - 250.0, bw)
        } else {
            (Mode::Lsb, 0.7, abs(f.hi_hz) + 250.0, bw)
        }
    } else {
        // Between narrow FM and AM without a clear carrier: the band plan decides.
        reasons.push(Reason::BandPlan);
        let mode = hint.filter(|m| matches!(m, Mode::Am | Mode::Nfm)).unwrap_or(Mode::Nfm);
        let center = abs((f.lo_hz + f.hi_hz) / 2.0);
        (mode, 0.5, mode.round_to_channel(center), (width * 1.2).clamp(6e3, 25e3))
    };
    let confidence = if hint == Some(mode) { (confidence + 0.1f32).min(0.95) } else { confidence };
    Classification { mode, confidence, frequency_hz: frequency.round(), bandwidth_hz: bandwidth, reasons }
}

/// Correction (Hz) of an SSB dial frequency from the speaker's voice. Voiced speech is a
/// series of harmonics k·f0; only with the dial on the suppressed carrier do they sit at
/// exactly k·f0 in the audio, a mistuning δ puts them at k·f0 + δ. One frame cannot tell
/// δ from δ + f0 (the comb just shifts by one harmonic), but f0 moves with intonation,
/// and only the true δ lines up in every frame: the δ with the best summed fit over
/// all voiced frames is taken. `dial_hz` is the rough dial relative to the samples'
/// centre; the result is to be added to it (USB) or subtracted from it (LSB).
pub fn ssb_pitch_offset(samples: &[Complex<f32>], rate: f64, dial_hz: f64, upper: bool) -> Option<f64> {
    use rustfft::FftPlanner;
    let mut x = samples.to_vec();
    Mixer::new(-dial_hz, rate).mix(&mut x);
    if !upper {
        x.iter_mut().for_each(|z| *z = z.conj());
    }
    // ~40 ms frames, half overlapping: short enough that the pitch barely moves within
    // one (longer frames smear the upper harmonics), long enough to separate harmonics
    // of a 75 Hz voice.
    let n = ((rate * 0.03) as usize).next_power_of_two();
    let bin = rate / n as f64;
    let top_bin = (3400.0 / bin) as usize;
    let fft = FftPlanner::new().plan_fft_forward(n);
    let window: Vec<f32> = (0..n).map(|i| 0.5 - 0.5 * (2.0 * std::f32::consts::PI * i as f32 / n as f32).cos()).collect();
    let mut frames: Vec<(f32, Vec<f32>)> = Vec::new();
    for start in (0..x.len().saturating_sub(n)).step_by(n / 2) {
        let mut buf: Vec<Complex<f32>> = x[start..start + n].iter().zip(&window).map(|(z, w)| z * w).collect();
        fft.process(&mut buf);
        let mag: Vec<f32> = buf[..top_bin].iter().map(|z| z.norm()).collect();
        let energy: f32 = mag[(150.0 / bin) as usize..].iter().map(|m| m * m).sum();
        // Log magnitude over the frame's median, minus its own median: noise scores about
        // zero, so a comb gains only from the harmonics it hits, not from how many
        // positions it samples.
        let mut sorted = mag.clone();
        sorted.sort_by(f32::total_cmp);
        let floor = sorted[sorted.len() / 2].max(1e-12);
        let mut s: Vec<f32> = mag.iter().map(|m| (1.0 + m / floor).ln()).collect();
        let mut sorted_s = s.clone();
        sorted_s.sort_by(f32::total_cmp);
        let mid = sorted_s[sorted_s.len() / 2];
        s.iter_mut().for_each(|v| *v -= mid);
        frames.push((energy, s));
    }
    // Voiced, loud frames only.
    let mut energies: Vec<f32> = frames.iter().map(|f| f.0).collect();
    energies.sort_by(f32::total_cmp);
    let cut = *energies.get(energies.len() * 2 / 5)?;
    let voiced: Vec<&Vec<f32>> = frames.iter().filter(|f| f.0 >= cut && f.0 > 0.0).map(|f| &f.1).collect();
    if voiced.len() < 4 {
        return None;
    }
    // Harmonic peaks of each voiced frame, placed between bins (parabola on the log
    // values): a few Hz precise. Only below 1.6 kHz, where a moving pitch smears least.
    let peaks: Vec<Vec<f64>> = voiced
        .iter()
        .map(|s| {
            let lo = (150.0 / bin) as usize;
            let hi = ((1600.0 / bin) as usize).min(s.len() - 2);
            (lo.max(1)..hi)
                .filter(|&i| s[i] > 1.0 && s[i] > s[i - 1] && s[i] >= s[i + 1])
                .map(|i| {
                    let (a, b, c) = (s[i - 1], s[i], s[i + 1]);
                    let d = a - 2.0 * b + c;
                    let off = if d.abs() > 1e-9 { (0.5 * (a - c) / d).clamp(-0.5, 0.5) } else { 0.0 };
                    (i as f64 + off as f64) * bin
                })
                .collect()
        })
        .filter(|p: &Vec<f64>| p.len() >= 3)
        .collect();
    if peaks.len() < 4 {
        return None;
    }
    // Squared distance of the peaks, less δ, from the nearest harmonic of the frame's
    // best f0; capped, so a stray peak costs a fixed amount.
    const CAP: f64 = 20.0 * 20.0;
    let misfit = |p: &Vec<f64>, delta: f64| {
        (0..300)
            .map(|j| {
                let f0 = 75.0 + j as f64 * 0.75;
                p.iter()
                    .map(|&h| {
                        let x = h - delta;
                        let k = (x / f0).round();
                        if k < 1.0 { CAP } else { (x - k * f0).powi(2).min(CAP) }
                    })
                    .sum::<f64>()
                    / p.len() as f64
            })
            .fold(f64::INFINITY, f64::min)
    };
    let deltas: Vec<f64> = (-70..=70).map(|i| i as f64 * 5.0).collect();
    // Higher is better: the negative mean misfit over the frames.
    let totals: Vec<f32> = deltas.iter().map(|&d| -(peaks.iter().map(|p| misfit(p, d)).sum::<f64>() / peaks.len() as f64) as f32).collect();
    let best = (0..deltas.len()).max_by(|&a, &b| totals[a].total_cmp(&totals[b]))?;
    // Only a clear winner: the best offset more than 40 Hz away must fit clearly worse
    // (a steady pitch cannot tell δ from δ ± f0), and the winner not at the search edge.
    let rival = (0..deltas.len()).filter(|&i| (deltas[i] - deltas[best]).abs() >= 40.0).map(|i| totals[i]).fold(f32::NEG_INFINITY, f32::max);
    let clear = totals[best] > rival + 15.0 && totals[best] > -120.0 && best > 0 && best < deltas.len() - 1;
    #[cfg(test)]
    {
        let mut order: Vec<usize> = (0..deltas.len()).collect();
        order.sort_by(|&a, &b| totals[b].total_cmp(&totals[a]));
        let top: Vec<String> = order.iter().take(6).map(|&i| format!("{:+.0}:{:.1}", deltas[i], totals[i])).collect();
        println!("pitch: {} voiced frames, top {}, rival {rival:.1}, clear {clear}", voiced.len(), top.join(" "));
    }
    clear.then_some(deltas[best])
}

/// Collects the signal around `hz` (receiver frequency) from source blocks and classifies
/// it. `width_hz` is the rough width seen on the spectrum; `lo_hz` the converter offset,
/// added to give radio frequencies (for the band plan and the result). Returns None if
/// the band moved away meanwhile.
pub fn spawn_job(blocks: Receiver<Block>, hz: f64, width_hz: f64, lo_hz: f64) -> Receiver<Option<Classification>> {
    let (tx, rx) = mpsc::sync_channel(1);
    thread::Builder::new()
        .name("classify".into())
        .spawn(move || {
            let verdict = collect(blocks, hz, width_hz).and_then(|(samples, rate)| {
                let c = classify_samples(&samples, rate, hz + lo_hz, width_hz);
                save_analysis(&samples, rate, hz + lo_hz, width_hz, c.as_ref());
                c
            });
            let _ = tx.send(verdict);
        })
        .expect("classify thread");
    rx
}

/// Recent analyses kept on disk.
const KEEP_ANALYSES: usize = 20;

/// Folder where the samples of recent analyses are kept, to study wrong verdicts
/// (Windows: %LOCALAPPDATA%\SeDiR\analysis, elsewhere ~/.cache/sedir/analysis).
pub fn analysis_dir() -> Option<std::path::PathBuf> {
    let base = if cfg!(windows) {
        std::path::PathBuf::from(std::env::var_os("LOCALAPPDATA")?).join("SeDiR")
    } else {
        std::path::PathBuf::from(std::env::var_os("HOME")?).join(".cache").join("sedir")
    };
    Some(base.join("analysis"))
}

/// Writes the analysed samples (cf32_le) and a description of them; the oldest beyond
/// `KEEP_ANALYSES` are removed. Failures are ignored: this only serves diagnosis.
fn save_analysis(samples: &[Complex<f32>], rate: f64, center_hz: f64, width_hz: f64, verdict: Option<&Classification>) {
    let Some(dir) = analysis_dir() else { return };
    if std::fs::create_dir_all(&dir).is_err() {
        return;
    }
    let stamp = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_millis());
    let name = format!("{stamp}_{:.0}Hz", center_hz);
    let bytes: Vec<u8> = samples.iter().flat_map(|z| [z.re.to_le_bytes(), z.im.to_le_bytes()].concat()).collect();
    let _ = std::fs::write(dir.join(format!("{name}.cf32")), bytes);
    let text = format!("rate={rate}\ncenter_hz={center_hz}\nwidth_hz={width_hz}\nverdict={verdict:?}\n");
    let _ = std::fs::write(dir.join(format!("{name}.txt")), text);
    if let Ok(entries) = std::fs::read_dir(&dir) {
        let mut files: Vec<_> = entries.filter_map(|e| e.ok()).map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x == "cf32")).collect();
        files.sort();
        for old in files.iter().take(files.len().saturating_sub(KEEP_ANALYSES)) {
            let _ = std::fs::remove_file(old);
            let _ = std::fs::remove_file(old.with_extension("txt"));
        }
    }
}

/// Reads an analysis saved by `save_analysis`: (samples, rate, center Hz, width Hz).
pub fn load_analysis(cf32: &std::path::Path) -> Option<(Vec<Complex<f32>>, f64, f64, f64)> {
    let bytes = std::fs::read(cf32).ok()?;
    let samples = bytes.chunks_exact(8).map(|c| Complex::new(f32::from_le_bytes(c[0..4].try_into().unwrap()), f32::from_le_bytes(c[4..8].try_into().unwrap()))).collect();
    let text = std::fs::read_to_string(cf32.with_extension("txt")).ok()?;
    let field = |key: &str| text.lines().find_map(|l| l.strip_prefix(key)?.strip_prefix('=')?.parse::<f64>().ok());
    Some((samples, field("rate")?, field("center_hz")?, field("width_hz")?))
}

/// Measures and decides, then for SSB places the dial from the voice. `center_hz` is the
/// radio frequency of the samples' centre.
pub fn classify_samples(samples: &[Complex<f32>], rate: f64, center_hz: f64, width_hz: f64) -> Option<Classification> {
    let focus = (width_hz / 2.0 + 300.0).max(500.0);
    let mut c = decide(&measure(samples, rate, focus)?, center_hz);
    if matches!(c.mode, Mode::Usb | Mode::Lsb) {
        let upper = c.mode == Mode::Usb;
        let dial = c.frequency_hz - center_hz;
        if let Some(delta) = ssb_pitch_offset(samples, rate, dial, upper) {
            c.frequency_hz = (c.frequency_hz + if upper { delta } else { -delta }).round();
            c.reasons.push(Reason::VoicePitch);
            c.confidence = (c.confidence + 0.1).min(0.95);
        }
    }
    Some(c)
}

/// Brings the signal at `hz` down to a rate suited to its width and collects
/// `ANALYSIS_SECONDS` of it; None if the band moved away.
fn collect(blocks: Receiver<Block>, hz: f64, width_hz: f64) -> Option<(Vec<Complex<f32>>, f64)> {
    // Analysis rate: room for the signal and some noise either side, at least 24 kHz.
    let target = (width_hz * 1.6).max(24e3);
    let started = Instant::now();
    let mut chain: Option<(f64, f64, Mixer, Vec<FirDecimator<Complex<f32>>>, f64)> = None;
    let mut collected: Vec<Complex<f32>> = Vec::new();
    let mut a = Vec::new();
    let mut b = Vec::new();
    loop {
        let block = match blocks.recv_timeout(Duration::from_millis(500)) {
            Ok(b) => b,
            Err(RecvTimeoutError::Timeout) if started.elapsed() < Duration::from_secs(5) => continue,
            _ => return None,
        };
        if (hz - block.center_hz).abs() > block.sample_rate / 2.0 {
            return None;
        }
        if chain.as_ref().is_none_or(|c| c.0 != block.center_hz || c.1 != block.sample_rate) {
            if chain.is_some() {
                // Retuned meanwhile: start over with the new band.
                collected.clear();
            }
            let mut rate = block.sample_rate;
            let mut halfbands = Vec::new();
            while rate / 2.0 >= target {
                halfbands.push(halfband());
                rate /= 2.0;
            }
            chain = Some((block.center_hz, block.sample_rate, Mixer::new(block.center_hz - hz, block.sample_rate), halfbands, rate));
        }
        let (_, _, mixer, halfbands, rate) = chain.as_mut().expect("chain");
        a.clear();
        a.extend_from_slice(&block.samples);
        mixer.mix(&mut a);
        for hb in halfbands.iter_mut() {
            b.clear();
            hb.process(&a, &mut b);
            std::mem::swap(&mut a, &mut b);
        }
        collected.extend_from_slice(&a);
        if collected.len() as f64 >= *rate * ANALYSIS_SECONDS {
            return Some((collected, *rate));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const RATE: f64 = 48e3;

    fn noise(n: usize, level: f32) -> Vec<Complex<f32>> {
        let mut seed = 99u32;
        let mut next = move || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5
        };
        (0..n).map(|_| Complex::new(next(), next()) * level).collect()
    }

    fn classify(signal: impl Fn(f64) -> Complex<f32>, center: f64) -> Classification {
        let n = (RATE * ANALYSIS_SECONDS) as usize;
        let samples: Vec<Complex<f32>> =
            noise(n, 0.01).into_iter().enumerate().map(|(i, w)| w + signal(i as f64 / RATE)).collect();
        let f = measure(&samples, RATE, 1.8e3).expect("features");
        let c = classify_samples(&samples, RATE, center, 3e3).expect("classification");
        println!("{f:?}\n→ {c:?}");
        c
    }

    /// Voiced speech in USB above `dial` (relative Hz): harmonics of a pitch that moves
    /// like intonation, shaped by two formants.
    fn voiced_usb(dial: f64) -> impl Fn(f64) -> Complex<f32> {
        move |t: f64| {
            // Phase of the fundamental: integral of f0(t) = 130 + 35 sin(2π·1.7t).
            let phase0 = 2.0 * PI * (130.0 * t - 35.0 / (2.0 * PI * 1.7) * (2.0 * PI * 1.7 * t).cos());
            let f0 = 130.0 + 35.0 * (2.0 * PI * 1.7 * t).sin();
            (1..=20)
                .map(|k| {
                    let f = k as f64 * f0;
                    let formant = 1.0 + 3.0 * (-((f - 600.0) / 250.0).powi(2)).exp() + 2.0 * (-((f - 1800.0) / 400.0).powi(2)).exp();
                    let amp = if (200.0..2800.0).contains(&f) { 0.03 * formant / (k as f64).sqrt() } else { 0.0 };
                    Complex::from_polar(amp as f32, (k as f64 * phase0 + 2.0 * PI * dial * t + k as f64) as f32)
                })
                .sum()
        }
    }

    #[test]
    fn voice_pitch_finds_the_ssb_carrier() {
        let n = (RATE * ANALYSIS_SECONDS) as usize;
        let signal = voiced_usb(0.0);
        let samples: Vec<Complex<f32>> = noise(n, 0.005).into_iter().enumerate().map(|(i, w)| w + signal(i as f64 / RATE)).collect();
        let _ = ssb_pitch_offset(&samples, RATE, 0.0, true);
        for rough in [130.0, -90.0, 40.0] {
            let delta = ssb_pitch_offset(&samples, RATE, rough, true).expect("pitch");
            println!("rough {rough:+} → correction {delta:+}");
            assert!((rough + delta).abs() <= 10.0, "rough {rough}, correction {delta}");
        }
        // LSB: the same voice mirrored below the carrier.
        let mirrored: Vec<Complex<f32>> = samples.iter().map(|z| z.conj()).collect();
        let delta = ssb_pitch_offset(&mirrored, RATE, -70.0, false).expect("pitch");
        assert!((-70.0 - delta).abs() <= 10.0, "LSB correction {delta}");
    }

    #[test]
    fn qo100_voice_is_tuned_to_its_carrier() {
        // Carrier 400 Hz above the analysis centre (a click on the voice, not the carrier).
        let c = classify(voiced_usb(-400.0), 10489.7604e6);
        assert_eq!(c.mode, Mode::Usb);
        assert!(c.reasons.contains(&Reason::VoicePitch), "{c:?}");
        assert!((c.frequency_hz - 10489.760e6).abs() <= 15.0, "dial {}", c.frequency_hz);
    }

    /// Speech-like audio: tones from 300 Hz to 2.6 kHz, strongest at the low end.
    fn voice(t: f64) -> f64 {
        (0..12).map(|k| {
            let f = 300.0 + k as f64 * 200.0;
            (1.0 / (1.0 + k as f64)) * (2.0 * PI * f * t + k as f64).sin() * (1.0 + 0.5 * (2.0 * PI * 3.0 * t).sin())
        })
        .sum::<f64>()
            * 0.15
    }

    #[test]
    fn recognises_am() {
        let c = classify(|t| Complex::new((0.3 * (1.0 + 0.6 * voice(t).clamp(-1.0, 1.0))) as f32, 0.0), 127.5e6);
        assert_eq!(c.mode, Mode::Am);
        assert!((c.frequency_hz - 127.5e6).abs() < 50.0);
    }

    #[test]
    fn recognises_nfm() {
        let c = classify(
            |t| Complex::from_polar(0.3, (2.5e3 / 1e3 * (2.0 * PI * 1e3 * t).sin() + 1.2 * (2.0 * PI * 700.0 * t).cos()) as f32),
            433.5e6,
        );
        assert_eq!(c.mode, Mode::Nfm);
    }

    #[test]
    fn recognises_usb_and_its_carrier() {
        // Dial frequency 14.200 MHz at the analysis centre; voice above it.
        let c = classify(
            |t| {
                (0..12)
                    .map(|k| {
                        let f = 300.0 + k as f64 * 200.0;
                        Complex::from_polar((0.1 / (1.0 + k as f64)) as f32, (2.0 * PI * f * t + k as f64) as f32)
                    })
                    .sum()
            },
            14.2e6,
        );
        assert_eq!(c.mode, Mode::Usb);
        assert!((c.frequency_hz - 14.2e6).abs() < 200.0, "dial {}", c.frequency_hz);
    }

    #[test]
    fn qo100_ssb_is_usb_even_with_bright_speech() {
        // Compressed, pre-emphasised speech: stronger towards the treble, which made the
        // spectrum look like LSB on the satellite.
        let c = classify(
            |t| {
                (0..12)
                    .map(|k| {
                        let f = 300.0 + k as f64 * 200.0;
                        Complex::from_polar((0.02 * (1.0 + k as f64)) as f32, (2.0 * PI * f * t + k as f64) as f32)
                    })
                    .sum()
            },
            10489.75e6,
        );
        assert_eq!(c.mode, Mode::Usb);
        assert!((c.frequency_hz - 10489.75e6).abs() < 200.0, "dial {}", c.frequency_hz);
    }

    #[test]
    fn recognises_lsb_below_10_mhz() {
        let c = classify(
            |t| {
                (0..12)
                    .map(|k| {
                        let f = -(300.0 + k as f64 * 200.0);
                        Complex::from_polar((0.1 / (1.0 + k as f64)) as f32, (2.0 * PI * f * t + k as f64) as f32)
                    })
                    .sum()
            },
            7.1e6,
        );
        assert_eq!(c.mode, Mode::Lsb);
        assert!((c.frequency_hz - 7.1e6).abs() < 200.0, "dial {}", c.frequency_hz);
    }

    #[test]
    fn recognises_keyed_cw() {
        // 12 wpm-ish keying of a carrier 500 Hz above centre.
        let c = classify(
            |t| {
                let on = ((t * 10.0) as u32) % 3 != 2;
                if on { Complex::from_polar(0.3, (2.0 * PI * 500.0 * t) as f32) } else { Complex::default() }
            },
            14.02e6,
        );
        assert_eq!(c.mode, Mode::Cw);
        assert!(c.reasons.contains(&Reason::Keyed));
        assert!((c.frequency_hz - 14.0205e6).abs() < 30.0);
    }

    /// On a real receiver: `SEDIR_SOURCE=ip:192.168.1.47 SEDIR_CENTER=100e6
    /// SEDIR_CLASSIFY=96.6,100.3,102.2 cargo test --lib live_classify -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn live_classify() {
        use crate::device::{DeviceInfo, DeviceKind};
        use crate::radio::Tuning;
        use crate::source::{self, Command, View};
        let (Ok(args), Ok(list)) = (std::env::var("SEDIR_SOURCE"), std::env::var("SEDIR_CLASSIFY")) else { return };
        let center: f64 = std::env::var("SEDIR_CENTER").ok().and_then(|v| v.parse().ok()).unwrap_or(100e6);
        let kind = if args.starts_with("ip:") { DeviceKind::LibreSdr } else { DeviceKind::RtlSdr };
        let rate = kind.limits().max_rate;
        let device = DeviceInfo { label: args.clone(), args, kind };
        let handle = source::spawn(device, Tuning { center_hz: center, sample_rate: rate, gain: Some(40.0) }, View::full(rate), || {});
        for (i, mhz) in list.split(',').filter_map(|v| v.trim().parse::<f64>().ok()).enumerate() {
            let (tx, rx) = std::sync::mpsc::sync_channel(32);
            handle.commands.send(Command::Subscribe(10 + i as u32, tx)).unwrap();
            let result = spawn_job(rx, mhz * 1e6, 150e3, 0.0).recv().unwrap();
            let _ = Reason::VoicePitch;
            println!("{mhz} MHz → {result:?}");
        }
    }

    /// Re-analyses the saved analyses with details:
    /// `cargo test --lib saved_analyses -- --ignored --nocapture` (SEDIR_ANALYSIS=folder to override).
    #[test]
    #[ignore]
    fn saved_analyses() {
        let dir = std::env::var_os("SEDIR_ANALYSIS").map(std::path::PathBuf::from).or_else(analysis_dir).unwrap();
        let mut files: Vec<_> = std::fs::read_dir(&dir).unwrap().filter_map(|e| e.ok()).map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x == "cf32")).collect();
        files.sort();
        for path in files {
            let (samples, rate, center, width) = load_analysis(&path).unwrap();
            let focus = (width / 2.0 + 300.0).max(500.0);
            let f = measure(&samples, rate, focus);
            println!("\n{} · {:.3} kHz rate · center {:.0} Hz · width {width:.0}", path.file_name().unwrap().to_string_lossy(), rate / 1e3, center);
            println!("  {f:?}");
            if let Some(f) = f {
                let d = decide(&f, center);
                println!("  rough: {:?} {:.0} Hz ({:+.0} from centre)", d.mode, d.frequency_hz, d.frequency_hz - center);
                if matches!(d.mode, Mode::Usb | Mode::Lsb) {
                    let r = ssb_pitch_offset(&samples, rate, d.frequency_hz - center, d.mode == Mode::Usb);
                    println!("  pitch correction: {r:?}");
                }
            }
            println!("  final: {:?}", classify_samples(&samples, rate, center, width));
        }
    }

    #[test]
    fn keyed_cw_with_phase_noise_on_qo100_is_cw() {
        // A converter's phase noise spreads the carrier into skirts that looked like AM
        // sidebands; the keying and the band plan settle it.
        let mut phase = 0.0f64;
        let mut seed = 5u32;
        let mut jitter = move || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            ((seed >> 8) as f64 / (1u64 << 24) as f64 - 0.5) * 0.12
        };
        let n = (RATE * ANALYSIS_SECONDS) as usize;
        let carrier: Vec<f64> = (0..n)
            .map(|_| {
                phase += 2.0 * PI * 300.0 / RATE + jitter();
                phase
            })
            .collect();
        let c = classify(
            |t| {
                let i = ((t * RATE) as usize).min(n - 1);
                let on = ((t * 8.0) as u32) % 3 != 2;
                if on { Complex::from_polar(0.3, carrier[i] as f32) } else { Complex::default() }
            },
            10489.6e6,
        );
        assert_eq!(c.mode, Mode::Cw, "{c:?}");
    }

    #[test]
    fn recognises_wfm() {
        let rate = 480e3;
        let n = (rate * ANALYSIS_SECONDS) as usize;
        let mut phase = 0.0;
        let samples: Vec<Complex<f32>> = noise(n, 0.01)
            .into_iter()
            .enumerate()
            .map(|(i, w)| {
                let t = i as f64 / rate;
                let mpx = 0.6 * voice(t) + 0.1 * (2.0 * PI * 19e3 * t).sin() + 0.3 * (2.0 * PI * 38e3 * t).sin() * voice(t + 0.3);
                phase += 2.0 * PI * 75e3 * mpx / rate;
                w + Complex::from_polar(0.3, phase as f32)
            })
            .collect();
        let f = measure(&samples, rate, 100e3).unwrap();
        let c = decide(&f, 100.3e6);
        println!("{f:?} → {c:?}");
        assert_eq!(c.mode, Mode::Wfm);
        assert_eq!(c.frequency_hz, 100.3e6);
    }
}
