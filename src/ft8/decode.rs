//! FT8 demodulation and decoding of one 15 s slot.
//!
//! A spectrogram with half-tone (3.125 Hz) bins and quarter-symbol (40 ms) steps is
//! searched for the three Costas sync blocks; for each candidate the 58 data symbols
//! give soft bits (log-max over the Gray-coded tones), which the LDPC decoder corrects;
//! the CRC confirms a message.

use num_complex::Complex;
use rustfft::FftPlanner;

use super::ldpc::{self, Decoder};
use super::message::{self, CallBook};
use super::{COSTAS, GRAY, START_SECONDS, SYMBOLS, data_symbol, tones};

/// Sample rate of the baseband given to the decoder: one symbol is 1024 samples.
pub const RATE: f64 = 6400.0;
pub const SPS: usize = 1024;
const STEP: usize = SPS / 4;
/// Zero-padded FFT: bins half a tone apart.
const NFFT: usize = 2 * SPS;
pub const BIN_HZ: f64 = RATE / NFFT as f64;
/// Sync candidates tried per slot.
const MAX_CANDIDATES: usize = 300;
/// Sync score (expected tone power over the mean of the 8 tones) needed to try.
const MIN_SYNC: f32 = 2.0;
/// The samples given begin this long before the slot: senders whose clocks run ahead
/// start early (one on QO-100 was 0.9 s ahead), and FT8 programs accept DT down to −2.5 s.
pub const EARLY_S: f64 = 2.0;
/// Latest start of a transmission considered (seconds after the slot start): DT +2.1 s.
const LATEST_START: f64 = 2.6;
/// Noise bandwidth of one bin (rectangular window of one symbol) and the reference
/// bandwidth of FT8 reports: SNR is quoted in 2500 Hz.
const BIN_NOISE_HZ: f64 = RATE / SPS as f64;
const SNR_BANDWIDTH_HZ: f64 = 2500.0;

/// A decoded message.
#[derive(Clone, Debug, PartialEq)]
pub struct Decoded {
    pub text: String,
    /// Signal to noise ratio in 2500 Hz (dB), as FT8 programs report it.
    pub snr_db: f32,
    /// Start time relative to the nominal 0.5 s (seconds).
    pub dt_s: f32,
    /// Frequency of tone 0 relative to the centre of the given baseband (Hz).
    pub freq_hz: f64,
}

/// Power spectra of the slot over the frequency range searched.
struct Spectrogram {
    /// Rows of `width` bins, one row per quarter symbol.
    power: Vec<f32>,
    width: usize,
    rows: usize,
    /// Frequency (relative to the baseband centre) of bin 0 of a row.
    first_hz: f64,
}

impl Spectrogram {
    fn new(samples: &[Complex<f32>], lo_hz: f64, hi_hz: f64) -> Self {
        let fft = FftPlanner::new().plan_fft_forward(NFFT);
        let first = ((lo_hz / BIN_HZ).floor() as isize + NFFT as isize / 2).max(0) as usize;
        let last = (((hi_hz / BIN_HZ).ceil() as isize + NFFT as isize / 2) as usize + 16).min(NFFT);
        let width = last - first;
        let rows = samples.len().saturating_sub(SPS) / STEP + 1;
        let mut power = Vec::with_capacity(rows * width);
        let mut buf = vec![Complex::default(); NFFT];
        for r in 0..rows {
            buf.fill(Complex::default());
            buf[..SPS].copy_from_slice(&samples[r * STEP..r * STEP + SPS]);
            fft.process(&mut buf);
            // Bin k of the centred spectrum is FFT bin k - NFFT/2 (mod NFFT).
            power.extend((first..last).map(|k| buf[(k + NFFT / 2) % NFFT].norm_sqr()));
        }
        Self { power, width, rows, first_hz: (first as f64 - NFFT as f64 / 2.0) * BIN_HZ }
    }

    fn at(&self, row: usize, bin: usize) -> f32 {
        self.power[row * self.width + bin]
    }

    /// The 8 tone powers of symbol `s` of a transmission starting at row `t0`, tone 0
    /// at bin `f`; None past the end of the slot.
    fn symbol(&self, t0: usize, f: usize, s: usize) -> Option<[f32; 8]> {
        let row = t0 + 4 * s;
        if row >= self.rows || f + 14 >= self.width {
            return None;
        }
        let mut p = [0f32; 8];
        for (j, v) in p.iter_mut().enumerate() {
            *v = self.at(row, f + 2 * j);
        }
        Some(p)
    }

    /// How strongly the Costas blocks stand out at (t0, f).
    fn sync(&self, t0: usize, f: usize) -> f32 {
        let (mut on, mut all) = (0f32, 0f32);
        for block in [0usize, 36, 72] {
            for (k, &tone) in COSTAS.iter().enumerate() {
                if let Some(p) = self.symbol(t0, f, block + k) {
                    on += p[tone];
                    all += p.iter().sum::<f32>() / 8.0;
                }
            }
        }
        if all > 0.0 { on / all } else { 0.0 }
    }
}

/// Soft bits of the 58 data symbols read from the coarse spectrogram (the decoder uses
/// `refine` instead; kept for the diagnostic tests).
#[cfg(test)]
fn soft_bits(spec: &Spectrogram, t0: usize, f: usize) -> Option<[f32; ldpc::N]> {
    let mut inverse = [0usize; 8];
    for (index, &tone) in GRAY.iter().enumerate() {
        inverse[tone] = index;
    }
    let mut llr = [0f32; ldpc::N];
    for j in 0..58 {
        let p = spec.symbol(t0, f, data_symbol(j))?;
        let s: Vec<f32> = p.iter().map(|v| (v + 1e-12).ln()).collect();
        for bit in 0..3 {
            let (mut one, mut zero) = (f32::NEG_INFINITY, f32::NEG_INFINITY);
            for tone in 0..8 {
                if (inverse[tone] >> (2 - bit)) & 1 == 1 {
                    one = one.max(s[tone]);
                } else {
                    zero = zero.max(s[tone]);
                }
            }
            llr[3 * j + bit] = one - zero;
        }
    }
    let var = llr.iter().map(|v| v * v).sum::<f32>() / ldpc::N as f32;
    let scale = (24.0 / var.max(1e-9)).sqrt();
    llr.iter_mut().for_each(|v| *v *= scale);
    Some(llr)
}

/// Rate of a candidate brought down on its own: 32 samples per symbol.
const FINE_RATE: f64 = 200.0;
const FINE_SPS: usize = 32;
const FINE_DECIMATION: usize = SPS / FINE_SPS;

/// One candidate, brought down to `FINE_RATE` with its tones centred on 0 Hz, and the
/// tone powers of its 79 symbols at the best time and frequency.
struct Fine {
    /// Tone powers, [symbol][tone].
    power: Vec<[f32; 8]>,
    /// Start of symbol 0 (seconds from the slot start) and tone 0 (Hz from the
    /// baseband centre), as refined.
    start_s: f64,
    freq_hz: f64,
}

/// Refines a coarse candidate (spectrogram row `t0`, tone 0 at `freq_hz`): the band
/// around it is mixed down and decimated, then the time (5 ms steps) and frequency
/// (0.25 Hz steps) where the Costas blocks fit best are searched, and every symbol's
/// tones are measured there by direct DFT. The coarse grid alone (40 ms, 3.125 Hz) put
/// signals on the air up to a fifth of a symbol and 2 Hz off, spilling energy into the
/// neighbouring symbols and tones.
fn refine(samples: &[Complex<f32>], t0: usize, freq_hz: f64, taps: &[f32]) -> Option<Fine> {
    use crate::demod::{FirDecimator, Mixer};
    // Centre of the 8 tones at 0 Hz.
    let centre = freq_hz + 3.5 * 6.25;
    // From one symbol before the coarse start to one after the end.
    let first = (t0 * STEP).saturating_sub(SPS);
    let last = (t0 * STEP + (SYMBOLS + 1) * SPS + taps.len()).min(samples.len());
    let mut x = samples[first..last].to_vec();
    Mixer::new(-centre, RATE).mix(&mut x);
    let mut y = Vec::with_capacity(x.len() / FINE_DECIMATION + 1);
    FirDecimator::new(taps, FINE_DECIMATION).process(&x, &mut y);
    // Output k of the decimating filter is centred on input k·D − (taps−1)/2 (its history
    // starts with taps−1 zeros), so input i is output (i + delay)/D.
    let delay = (taps.len() - 1) as f64 / 2.0;
    // Sample (at FINE_RATE) of the coarse start within y.
    let coarse = ((t0 * STEP - first) as f64 + delay) / FINE_DECIMATION as f64;

    // DFT of one symbol at the 8 tone frequencies plus `df`.
    let tone_power = |start: isize, df: f64| -> Option<[f32; 8]> {
        if start < 0 || start as usize + FINE_SPS > y.len() {
            return None;
        }
        let seg = &y[start as usize..start as usize + FINE_SPS];
        let mut p = [0f32; 8];
        for (j, v) in p.iter_mut().enumerate() {
            let f = (j as f64 - 3.5) * 6.25 + df;
            let w = Complex::from_polar(1.0f64, -2.0 * std::f64::consts::PI * f / FINE_RATE);
            let mut rot = Complex::new(1.0f64, 0.0);
            let mut acc = Complex::new(0.0f64, 0.0);
            for z in seg {
                acc += Complex::new(z.re as f64, z.im as f64) * rot;
                rot *= w;
            }
            *v = acc.norm_sqr() as f32;
        }
        Some(p)
    };
    let sync_at = |offset: isize, df: f64| -> f32 {
        let (mut on, mut all) = (0f32, 0f32);
        for block in [0usize, 36, 72] {
            for (k, &tone) in COSTAS.iter().enumerate() {
                if let Some(p) = tone_power(offset + ((block + k) * FINE_SPS) as isize, df) {
                    on += p[tone];
                    all += p.iter().sum::<f32>();
                }
            }
        }
        if all > 0.0 { on / all } else { 0.0 }
    };
    // Coarse then fine: ±8 samples (±40 ms) in 2s and ±2 Hz in 0.5 Hz, then around the
    // best in single samples and 0.25 Hz.
    let base = coarse.round() as isize;
    let mut best = (f32::MIN, base, 0.0f64);
    for dt in (-8..=8).step_by(2) {
        for k in -4..=4 {
            let df = k as f64 * 0.5;
            let s = sync_at(base + dt, df);
            if s > best.0 {
                best = (s, base + dt, df);
            }
        }
    }
    let (_, t_mid, f_mid) = best;
    for dt in -1..=1 {
        for k in -1..=1 {
            let df = f_mid + k as f64 * 0.25;
            let s = sync_at(t_mid + dt, df);
            if s > best.0 {
                best = (s, t_mid + dt, df);
            }
        }
    }
    let (_, offset, df) = best;
    let power: Option<Vec<[f32; 8]>> = (0..SYMBOLS).map(|s| tone_power(offset + (s * FINE_SPS) as isize, df)).collect();
    let start_s = (first as f64 + offset as f64 * FINE_DECIMATION as f64 - delay) / RATE;
    Some(Fine { power: power?, start_s, freq_hz: freq_hz + df })
}

/// Soft bits from refined tone powers (log-max over the Gray-coded tones), scaled to a
/// fixed spread.
fn fine_soft_bits(power: &[[f32; 8]]) -> [f32; ldpc::N] {
    let mut inverse = [0usize; 8];
    for (index, &tone) in GRAY.iter().enumerate() {
        inverse[tone] = index;
    }
    let mut llr = [0f32; ldpc::N];
    for j in 0..58 {
        let p = &power[data_symbol(j)];
        let s: Vec<f32> = p.iter().map(|v| (v + 1e-12).ln()).collect();
        for bit in 0..3 {
            let (mut one, mut zero) = (f32::NEG_INFINITY, f32::NEG_INFINITY);
            for tone in 0..8 {
                if (inverse[tone] >> (2 - bit)) & 1 == 1 {
                    one = one.max(s[tone]);
                } else {
                    zero = zero.max(s[tone]);
                }
            }
            llr[3 * j + bit] = one - zero;
        }
    }
    let var = llr.iter().map(|v| v * v).sum::<f32>() / ldpc::N as f32;
    let scale = (24.0 / var.max(1e-9)).sqrt();
    llr.iter_mut().for_each(|v| *v *= scale);
    llr
}

/// Decodes the FT8 transmissions of one slot. `samples` are the baseband at `RATE`,
/// sample 0 `EARLY_S` before the slot start; transmissions with tone 0 between `lo_hz`
/// and `hi_hz` (relative to the baseband centre) are looked for. Call signs go to `book`.
pub fn decode_slot(samples: &[Complex<f32>], lo_hz: f64, hi_hz: f64, book: &mut CallBook) -> Vec<Decoded> {
    if samples.len() < SPS * SYMBOLS {
        return Vec::new();
    }
    let spec = Spectrogram::new(samples, lo_hz, hi_hz);
    let latest = (((EARLY_S + LATEST_START) * RATE) as usize / STEP).min(spec.rows.saturating_sub(1));
    let max_f = spec.width.saturating_sub(16);

    // Sync scores and their local maxima.
    let mut scores = vec![0f32; (latest + 1) * max_f];
    for t0 in 0..=latest {
        for f in 0..max_f {
            scores[t0 * max_f + f] = spec.sync(t0, f);
        }
    }
    let mut candidates: Vec<(f32, usize, usize)> = Vec::new();
    for t0 in 0..=latest {
        for f in 0..max_f {
            let s = scores[t0 * max_f + f];
            if s < MIN_SYNC {
                continue;
            }
            let mut best = true;
            'n: for dt in -2isize..=2 {
                for df in -2isize..=2 {
                    let (t, ff) = (t0 as isize + dt, f as isize + df);
                    if (dt, df) != (0, 0) && t >= 0 && ff >= 0 && (t as usize) <= latest && (ff as usize) < max_f {
                        let o = scores[t as usize * max_f + ff as usize];
                        if o > s || (o == s && (dt, df) < (0, 0)) {
                            best = false;
                            break 'n;
                        }
                    }
                }
            }
            if best {
                candidates.push((s, t0, f));
            }
        }
    }
    candidates.sort_by(|a, b| b.0.total_cmp(&a.0));
    candidates.truncate(MAX_CANDIDATES);

    // Noise per bin: the median power over the spectrogram.
    let mut sample: Vec<f32> = spec.power.iter().step_by(7).copied().collect();
    sample.sort_by(f32::total_cmp);
    let noise = sample[sample.len() / 2].max(1e-20);

    let _ = noise;
    let decoder = Decoder::default();
    let taps = crate::demod::lowpass(40.0 / RATE, 513);
    let mut found: Vec<Decoded> = Vec::new();
    let mut used: Vec<(f64, f64)> = Vec::new();
    for (_, t0, f) in candidates {
        let coarse_hz = spec.first_hz + f as f64 * BIN_HZ;
        let coarse_s = (t0 * STEP) as f64 / RATE;
        // A decoded transmission also leaves sync peaks next to itself.
        if used.iter().any(|&(s, hz)| (s - coarse_s).abs() < 0.2 && (hz - coarse_hz).abs() < 10.0) {
            continue;
        }
        let Some(fine) = refine(samples, t0, coarse_hz, &taps) else { continue };
        let llr = fine_soft_bits(&fine.power);
        let Some(word) = decoder.decode(&llr, 30) else { continue };
        let msg = &word[..77];
        if msg.iter().all(|&b| b == 0) {
            continue;
        }
        let crc = word[77..91].iter().fold(0u16, |v, &b| (v << 1) | b as u16);
        if crc != ldpc::crc14(msg) {
            continue;
        }
        let Some(text) = message::unpack(msg, book) else { continue };
        used.push((fine.start_s, fine.freq_hz));
        if found.iter().any(|d| d.text == text) {
            continue;
        }
        // SNR: power in the sent tone against the others (each a 6.25 Hz noise band).
        let sent = tones(&word);
        let mut signal = 0f32;
        let mut others: Vec<f32> = Vec::with_capacity(SYMBOLS * 7);
        for (s, p) in fine.power.iter().enumerate() {
            signal += p[sent[s]];
            others.extend(p.iter().enumerate().filter(|(j, _)| *j != sent[s]).map(|(_, v)| *v));
        }
        signal /= SYMBOLS as f32;
        others.sort_by(f32::total_cmp);
        let noise_bin = others[others.len() / 2].max(1e-20);
        let snr = 10.0 * ((signal / noise_bin - 1.0).max(1e-3) as f64 * BIN_NOISE_HZ / SNR_BANDWIDTH_HZ).log10();
        found.push(Decoded { text, snr_db: snr as f32, dt_s: (fine.start_s - EARLY_S - START_SECONDS) as f32, freq_hz: fine.freq_hz });
    }
    found
}

#[cfg(test)]
pub mod tests {
    use super::*;
    use crate::ft8::message::pack_standard;
    use std::f64::consts::PI;

    /// An FT8 transmission (continuous-phase FSK) added to `out` at `RATE`.
    /// `start_s` is relative to the slot start; the buffer begins `EARLY_S` before it.
    pub fn add_signal(out: &mut [Complex<f32>], text: &str, freq_hz: f64, start_s: f64, amplitude: f32) {
        let bits = pack_standard(text).or_else(|| crate::ft8::message::pack_text(text)).expect(text);
        let word = ldpc::encode(&bits);
        let t = tones(&word);
        let start = ((start_s + EARLY_S) * RATE) as usize;
        let mut phase = 0.0f64;
        for (s, &tone) in t.iter().enumerate() {
            let f = freq_hz + tone as f64 * 6.25;
            for i in 0..SPS {
                let n = start + s * SPS + i;
                if n < out.len() {
                    out[n] += Complex::from_polar(amplitude, phase as f32);
                }
                phase += 2.0 * PI * f / RATE;
            }
        }
    }

    pub fn noise(n: usize, sigma: f32, seed: u32) -> Vec<Complex<f32>> {
        let mut s = seed;
        let mut gauss = move || {
            // Sum of uniforms: close enough to Gaussian for tests.
            (0..12)
                .map(|_| {
                    s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                    (s >> 8) as f32 / (1u32 << 24) as f32
                })
                .sum::<f32>()
                - 6.0
        };
        (0..n).map(|_| Complex::new(gauss(), gauss()) * (sigma / 2f32.sqrt())).collect()
    }

    /// Amplitude giving `snr_db` in 2500 Hz against complex noise of total power 1 at RATE.
    pub fn amplitude(snr_db: f64) -> f32 {
        (10f64.powf(snr_db / 10.0) * SNR_BANDWIDTH_HZ / RATE).sqrt() as f32
    }

    /// Samples of one slot as the worker gives them.
    pub fn slot_len() -> usize {
        ((EARLY_S + 14.0) * RATE) as usize
    }

    #[test]
    fn decodes_several_signals_in_noise() {
        let mut x = noise(slot_len(), 1.0, 42);
        let signals = [
            ("CQ TA4SO KM46", -600.0, 0.5, -10.0),
            ("K1ABC W9XYZ -11", -200.0, 0.8, -15.0),
            ("W9XYZ K1ABC R-09", 333.0, 0.3, -5.0),
            ("DL1ABC TA4SO RR73", 900.0, 1.2, -12.0),
            // A sender whose clock runs 1.4 s ahead (seen on QO-100).
            ("CQ R4FDW LO23", 0.0, -0.9, -8.0),
        ];
        for (text, f, t, snr) in signals {
            add_signal(&mut x, text, f, t, amplitude(snr));
        }
        let mut book = CallBook::default();
        let found = decode_slot(&x, -1400.0, 1400.0, &mut book);
        for d in &found {
            println!("{:>6.1} dB {:+.2} s {:>7.1} Hz  {}", d.snr_db, d.dt_s, d.freq_hz, d.text);
        }
        for (text, f, t, snr) in signals {
            let d = found.iter().find(|d| d.text == text).unwrap_or_else(|| panic!("{text} not decoded"));
            assert!((d.freq_hz - f).abs() < 4.0, "{text}: {} Hz", d.freq_hz);
            assert!((d.dt_s as f64 - (t - START_SECONDS)).abs() < 0.05, "{text}: dt {}", d.dt_s);
            assert!((d.snr_db as f64 - snr).abs() < 3.0, "{text}: {} dB", d.snr_db);
        }
    }

    /// Studies saved slots: `cargo test --lib ft8_slots -- --ignored --nocapture`
    /// (SEDIR_FT8_DIR overrides the folder). For the strongest sync candidates: score,
    /// place, whether LDPC decodes, and where the strongest tone sits symbol by symbol
    /// (to see drift).
    #[test]
    #[ignore]
    fn ft8_slots() {
        let dir = std::env::var_os("SEDIR_FT8_DIR").map(std::path::PathBuf::from).or_else(crate::ft8::worker::slot_dir).unwrap();
        let mut files: Vec<_> = std::fs::read_dir(&dir).unwrap().filter_map(|e| e.ok()).map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x == "cf32")).collect();
        files.sort();
        for path in files {
            let samples = crate::ft8::worker::load_slot(&path).unwrap();
            let spec = Spectrogram::new(&samples, -1500.0, 1500.0);
            let latest = (((EARLY_S + LATEST_START) * RATE) as usize / STEP).min(spec.rows - 1);
            let max_f = spec.width - 16;
            let mut cands: Vec<(f32, usize, usize)> = Vec::new();
            for t0 in 0..=latest {
                for f in 0..max_f {
                    cands.push((spec.sync(t0, f), t0, f));
                }
            }
            cands.sort_by(|a, b| b.0.total_cmp(&a.0));
            let mut shown: Vec<(usize, usize)> = Vec::new();
            println!("\n{}", path.file_name().unwrap().to_string_lossy());
            let decoder = Decoder::default();
            for (s, t0, f) in cands {
                if shown.len() >= 6 {
                    break;
                }
                if shown.iter().any(|&(t, ff)| t.abs_diff(t0) <= 6 && ff.abs_diff(f) <= 6) {
                    continue;
                }
                shown.push((t0, f));
                let hz = spec.first_hz + f as f64 * BIN_HZ + 1600.0;
                let word = soft_bits(&spec, t0, f).and_then(|l| decoder.decode(&l, 30));
                let verdict = match &word {
                    None => "LDPC fails".to_string(),
                    Some(w) => {
                        let crc = w[77..91].iter().fold(0u16, |v, &b| (v << 1) | b as u16);
                        let crc_ok = crc == ldpc::crc14(&w[..77]);
                        let i3n3: String = w[71..77].iter().map(|b| b.to_string()).collect();
                        format!("LDPC ok, CRC {}, n3|i3 bits {i3n3}, text {:?}", if crc_ok { "ok" } else { "BAD" }, message::unpack(&w[..77], &mut CallBook::default()))
                    }
                };
                println!("  sync {s:.2} at {hz:.0} Hz audio, start {:.2} s, {verdict}", t0 as f64 * STEP as f64 / RATE);
                // Strongest bin of each sync symbol within the signal's own tones (half
                // tones from where Costas puts it): all 0 = clean, a trend = drift.
                for block in [0usize, 36, 72] {
                    let peaks: Vec<String> = (0..7)
                        .map(|k| {
                            let row = t0 + 4 * (block + k);
                            if row >= spec.rows {
                                return "--".into();
                            }
                            let expect = f + 2 * COSTAS[k];
                            let (lo, hi) = (expect.saturating_sub(4), (expect + 5).min(spec.width));
                            let best = (lo..hi).max_by(|&a, &b| spec.at(row, a).total_cmp(&spec.at(row, b))).unwrap();
                            format!("{:+}", best as isize - expect as isize)
                        })
                        .collect();
                    println!("    costas {block:>2}: offsets {}", peaks.join(" "));
                }
            }
        }
    }

    /// Writes a saved slot's spectrogram as an 8-bit grey image (audio 100–3100 Hz
    /// across, time down): `SEDIR_FILE=slot.cf32 SEDIR_OUT=out.gray cargo test --lib
    /// ft8_picture -- --ignored --nocapture`; prints its size.
    #[test]
    #[ignore]
    fn ft8_picture() {
        let (Some(file), Some(out)) = (std::env::var_os("SEDIR_FILE"), std::env::var_os("SEDIR_OUT")) else { return };
        let samples = crate::ft8::worker::load_slot(std::path::Path::new(&file)).unwrap();
        let spec = Spectrogram::new(&samples, -1500.0, 1500.0);
        let mut db: Vec<f32> = spec.power.iter().map(|p| 10.0 * (p + 1e-20).log10()).collect();
        let mut sorted = db.clone();
        sorted.sort_by(f32::total_cmp);
        let (lo, hi) = (sorted[sorted.len() / 2], sorted[sorted.len() * 999 / 1000]);
        db.iter_mut().for_each(|v| *v = ((*v - lo) / (hi - lo)).clamp(0.0, 1.0) * 255.0);
        std::fs::write(&out, db.iter().map(|v| *v as u8).collect::<Vec<u8>>()).unwrap();
        println!("size {}x{}", spec.width, spec.rows);
    }

    /// Decodes every saved slot: `cargo test --lib ft8_saved_decode -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn ft8_saved_decode() {
        let dir = std::env::var_os("SEDIR_FT8_DIR").map(std::path::PathBuf::from).or_else(crate::ft8::worker::slot_dir).unwrap();
        let mut files: Vec<_> = std::fs::read_dir(&dir).unwrap().filter_map(|e| e.ok()).map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x == "cf32")).collect();
        files.sort();
        let mut total = 0;
        let mut book = CallBook::default();
        for path in &files {
            let samples = crate::ft8::worker::load_slot(path).unwrap();
            let started = std::time::Instant::now();
            let found = decode_slot(&samples, -1500.0, 1500.0, &mut book);
            println!("{} · {} messages in {:.2} s", path.file_name().unwrap().to_string_lossy(), found.len(), started.elapsed().as_secs_f64());
            for d in &found {
                println!("   {:+5.1} dB {:+.2} s {:6.0} Hz  {}", d.snr_db, d.dt_s, d.freq_hz + 1600.0, d.text);
            }
            total += found.len();
        }
        println!("total {total} messages in {} slots", files.len());
    }

    /// For each message decoded in the saved slots: where each of its 79 symbols really
    /// is, against where the decoder read it (time in 40 ms steps, frequency in half
    /// tones), averaged per 8 symbols. `cargo test --lib ft8_alignment -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn ft8_alignment() {
        let dir = std::env::var_os("SEDIR_FT8_DIR").map(std::path::PathBuf::from).or_else(crate::ft8::worker::slot_dir).unwrap();
        let mut files: Vec<_> = std::fs::read_dir(&dir).unwrap().filter_map(|e| e.ok()).map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x == "cf32")).collect();
        files.sort();
        let decoder = Decoder::default();
        for path in files {
            let samples = crate::ft8::worker::load_slot(&path).unwrap();
            let spec = Spectrogram::new(&samples, -1500.0, 1500.0);
            let latest = (((EARLY_S + LATEST_START) * RATE) as usize / STEP).min(spec.rows - 1);
            for t0 in 0..=latest {
                for f in 0..spec.width - 16 {
                    if spec.sync(t0, f) < 4.0 {
                        continue;
                    }
                    let Some(w) = soft_bits(&spec, t0, f).and_then(|l| decoder.decode(&l, 30)) else { continue };
                    if w[..77].iter().all(|&b| b == 0) {
                        continue;
                    }
                    let tones = tones(&w);
                    println!("\n{} at {:.0} Hz, start row {t0}", path.file_name().unwrap().to_string_lossy(), spec.first_hz + f as f64 * BIN_HZ + 1600.0);
                    for group in 0..10 {
                        let (mut sdt, mut sdf, mut n) = (0i32, 0i32, 0);
                        for s in group * 8..((group + 1) * 8).min(SYMBOLS) {
                            let mut best = (f32::MIN, 0i32, 0i32);
                            for dt in -2i32..=2 {
                                for df in -3i32..=3 {
                                    let row = (t0 + 4 * s) as i32 + dt;
                                    let bin = (f + 2 * tones[s]) as i32 + df;
                                    if row < 0 || row as usize >= spec.rows || bin < 0 || bin as usize >= spec.width {
                                        continue;
                                    }
                                    let p = spec.at(row as usize, bin as usize);
                                    if p > best.0 {
                                        best = (p, dt, df);
                                    }
                                }
                            }
                            sdt += best.1;
                            sdf += best.2;
                            n += 1;
                        }
                        println!("  symbols {:>2}-{:>2}: time {:+.2} steps, frequency {:+.2} half tones", group * 8, group * 8 + n - 1, sdt as f32 / n as f32, sdf as f32 / n as f32);
                    }
                    return;
                }
            }
        }
    }

    #[test]
    fn noise_alone_gives_nothing() {
        let x = noise(slot_len(), 1.0, 7);
        assert!(decode_slot(&x, -1400.0, 1400.0, &mut CallBook::default()).is_empty());
    }
}
