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
/// Latest start of a transmission considered (seconds after the slot start).
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

/// Soft bits of the 58 data symbols: ln P(1)/P(0) by the log-max rule over the tones,
/// scaled to a fixed spread for the LDPC decoder.
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

/// Decodes the FT8 transmissions of one slot. `samples` are the baseband at `RATE`,
/// sample 0 at the slot start; transmissions with tone 0 between `lo_hz` and `hi_hz`
/// (relative to the baseband centre) are looked for. Call signs go to `book`.
pub fn decode_slot(samples: &[Complex<f32>], lo_hz: f64, hi_hz: f64, book: &mut CallBook) -> Vec<Decoded> {
    if samples.len() < SPS * SYMBOLS {
        return Vec::new();
    }
    let spec = Spectrogram::new(samples, lo_hz, hi_hz);
    let latest = ((LATEST_START * RATE) as usize / STEP).min(spec.rows.saturating_sub(1));
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

    let decoder = Decoder::default();
    let mut found: Vec<Decoded> = Vec::new();
    let mut used: Vec<(usize, usize)> = Vec::new();
    for (_, t0, f) in candidates {
        // A decoded transmission also leaves sync peaks next to itself.
        if used.iter().any(|&(t, ff)| t.abs_diff(t0) <= 4 && ff.abs_diff(f) <= 3) {
            continue;
        }
        let tries = [(0isize, 0isize), (0, -1), (0, 1), (-1, 0), (1, 0)];
        for (dt, df) in tries {
            let (t, ff) = (t0 as isize + dt, f as isize + df);
            if t < 0 || ff < 0 {
                continue;
            }
            let (t, ff) = (t as usize, ff as usize);
            let Some(llr) = soft_bits(&spec, t, ff) else { continue };
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
            used.push((t, ff));
            if found.iter().any(|d| d.text == text) {
                break;
            }
            let sent = tones(&word);
            let signal: f32 = (0..SYMBOLS).filter_map(|s| spec.symbol(t, ff, s).map(|p| p[sent[s]])).sum::<f32>() / SYMBOLS as f32;
            let snr = 10.0 * ((signal / noise - 1.0).max(1e-3) as f64 * BIN_NOISE_HZ / SNR_BANDWIDTH_HZ).log10();
            found.push(Decoded {
                text,
                snr_db: snr as f32,
                dt_s: (t as f64 * STEP as f64 / RATE - START_SECONDS) as f32,
                freq_hz: spec.first_hz + ff as f64 * BIN_HZ,
            });
            break;
        }
    }
    found
}

#[cfg(test)]
pub mod tests {
    use super::*;
    use crate::ft8::message::pack_standard;
    use std::f64::consts::PI;

    /// An FT8 transmission (continuous-phase FSK) added to `out` at `RATE`.
    pub fn add_signal(out: &mut [Complex<f32>], text: &str, freq_hz: f64, start_s: f64, amplitude: f32) {
        let bits = pack_standard(text).or_else(|| crate::ft8::message::pack_text(text)).expect(text);
        let word = ldpc::encode(&bits);
        let t = tones(&word);
        let start = (start_s * RATE) as usize;
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

    #[test]
    fn decodes_several_signals_in_noise() {
        let n = (14.0 * RATE) as usize;
        let mut x = noise(n, 1.0, 42);
        let signals = [
            ("CQ TA4SO KM46", -600.0, 0.5, -10.0),
            ("K1ABC W9XYZ -11", -200.0, 0.8, -15.0),
            ("W9XYZ K1ABC R-09", 333.0, 0.3, -5.0),
            ("DL1ABC TA4SO RR73", 900.0, 1.2, -12.0),
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

    #[test]
    fn noise_alone_gives_nothing() {
        let n = (14.0 * RATE) as usize;
        let x = noise(n, 1.0, 7);
        assert!(decode_slot(&x, -1400.0, 1400.0, &mut CallBook::default()).is_empty());
    }
}
