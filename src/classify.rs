//! Recognising the kind of a signal: a short stretch of IQ around it is measured
//! (occupied band, carrier, one-sidedness, envelope, keying) and the measurements,
//! with the band plan as a tie-breaker, give the mode and where to tune.

use std::sync::mpsc::{self, Receiver, RecvTimeoutError, SyncSender};
use std::thread;
use std::time::{Duration, Instant};

use num_complex::Complex;

use crate::bandplan;
use crate::demod::{FirDecimator, Mixer, Mode, halfband, lowpass, taps_for};
use crate::dsp::Spectrum;
use crate::source::Block;

/// Signal analysed (seconds).
const ANALYSIS_SECONDS: f64 = 0.7;
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

/// Measures `samples` (complex baseband at `rate`, the signal near 0 Hz).
pub fn measure(samples: &[Complex<f32>], rate: f64) -> Option<Features> {
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
    let peak = (0..n).max_by(|&a, &b| power[a].total_cmp(&power[b]))?;
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
    } else if width < 400.0 || (f.carrier_fraction > 0.75 && width < 1.5e3) {
        // A bare carrier: CW, or an AM/FM transmitter without modulation.
        reasons.push(Reason::NarrowCarrier);
        let keyed = f.off_fraction > 0.1 && f.off_fraction < 0.9;
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
    } else if (f.carrier_fraction > 0.5 || (f.carrier_fraction > 0.2 && !constant_envelope)) && !one_sided {
        // A modulated FM signal rarely keeps half its power in one line; AM always does
        // (the carrier carries at least two thirds of it).
        reasons.push(Reason::CarrierAndSidebands);
        let half = (f.peak_hz - f.lo_hz).max(f.hi_hz - f.peak_hz);
        (Mode::Am, 0.8, Mode::Am.round_to_channel(abs(f.peak_hz)), (2.0 * half + 500.0).clamp(3e3, 12e3))
    } else if constant_envelope && width >= 3e3 {
        reasons.push(Reason::ConstantEnvelope);
        let center = abs((f.lo_hz + f.hi_hz) / 2.0);
        (Mode::Nfm, 0.8, Mode::Nfm.round_to_channel(center), (width * 1.3).clamp(8e3, 25e3))
    } else if width <= 4.5e3 {
        // Single sideband. The sideband is a convention of the band (LSB below 10 MHz on
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

/// Collects the signal around `hz` (receiver frequency) from source blocks and classifies
/// it. `width_hz` is the rough width seen on the spectrum; `lo_hz` the converter offset,
/// added to give radio frequencies (for the band plan and the result). Returns None if
/// the band moved away meanwhile.
pub fn spawn_job(blocks: Receiver<Block>, hz: f64, width_hz: f64, lo_hz: f64) -> Receiver<Option<Classification>> {
    let (tx, rx) = mpsc::sync_channel(1);
    thread::Builder::new().name("classify".into()).spawn(move || job(blocks, hz, width_hz, lo_hz, tx)).expect("classify thread");
    rx
}

fn job(blocks: Receiver<Block>, hz: f64, width_hz: f64, lo_hz: f64, result: SyncSender<Option<Classification>>) {
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
            _ => {
                let _ = result.send(None);
                return;
            }
        };
        if (hz - block.center_hz).abs() > block.sample_rate / 2.0 {
            let _ = result.send(None);
            return;
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
            let verdict = measure(&collected, *rate).map(|f| decide(&f, hz + lo_hz));
            let _ = result.send(verdict);
            return;
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
        let f = measure(&samples, RATE).expect("features");
        let c = decide(&f, center);
        println!("{f:?}\n→ {c:?}");
        c
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
            println!("{mhz} MHz → {result:?}");
        }
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
        let f = measure(&samples, rate).unwrap();
        let c = decide(&f, 100.3e6);
        println!("{f:?} → {c:?}");
        assert_eq!(c.mode, Mode::Wfm);
        assert_eq!(c.frequency_hz, 100.3e6);
    }
}
