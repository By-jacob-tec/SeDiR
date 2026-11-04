//! FT8/FT4 demodulation and decoding of one slot, for either protocol.
//!
//! A spectrogram with half-tone bins and quarter-symbol steps is searched for the
//! Costas sync blocks; each candidate is refined on its own (time and frequency, DFT of
//! every symbol), its data symbols give soft bits (log-max over the Gray-coded tones),
//! the LDPC decoder corrects them and the CRC confirms a message. A second pass lets
//! the sync blocks sit at different frequencies, for signals that drift or step.

use num_complex::Complex;
use rustfft::FftPlanner;

use super::Protocol;
use super::ldpc::{self, Decoder};
use super::message::{self, CallBook};

/// Sync candidates tried per pass.
const MAX_CANDIDATES: usize = 300;
/// FT8 reports quote SNR in 2500 Hz.
const SNR_BANDWIDTH_HZ: f64 = 2500.0;

/// A decoded message.
#[derive(Clone, Debug, PartialEq)]
pub struct Decoded {
    pub text: String,
    /// Signal to noise ratio in 2500 Hz (dB), as FT8 programs report it.
    pub snr_db: f32,
    /// Start time relative to the protocol's nominal start (seconds).
    pub dt_s: f32,
    /// Frequency of tone 0 relative to the centre of the given baseband (Hz).
    pub freq_hz: f64,
}

/// Power spectra of the slot over the frequency range searched.
struct Spectrogram {
    /// Rows of `width` bins, one row per quarter symbol, bins half a tone apart.
    power: Vec<f32>,
    width: usize,
    rows: usize,
    /// Frequency (relative to the baseband centre) of bin 0 of a row.
    first_hz: f64,
    bin_hz: f64,
    step: usize,
    tones: usize,
}

impl Spectrogram {
    fn new(p: &Protocol, samples: &[Complex<f32>], lo_hz: f64, hi_hz: f64) -> Self {
        let (sps, nfft) = (p.sps, 2 * p.sps);
        let step = sps / 4;
        let bin_hz = p.rate / nfft as f64;
        let fft = FftPlanner::new().plan_fft_forward(nfft);
        let first = ((lo_hz / bin_hz).floor() as isize + nfft as isize / 2).max(0) as usize;
        let last = (((hi_hz / bin_hz).ceil() as isize + nfft as isize / 2) as usize + 2 * p.tones).min(nfft);
        let width = last - first;
        let rows = samples.len().saturating_sub(sps) / step + 1;
        let mut power = Vec::with_capacity(rows * width);
        let mut buf = vec![Complex::default(); nfft];
        for r in 0..rows {
            buf.fill(Complex::default());
            buf[..sps].copy_from_slice(&samples[r * step..r * step + sps]);
            fft.process(&mut buf);
            // Bin k of the centred spectrum is FFT bin k - nfft/2 (mod nfft).
            power.extend((first..last).map(|k| buf[(k + nfft / 2) % nfft].norm_sqr()));
        }
        Self { power, width, rows, first_hz: (first as f64 - nfft as f64 / 2.0) * bin_hz, bin_hz, step, tones: p.tones }
    }

    fn at(&self, row: usize, bin: usize) -> f32 {
        self.power[row * self.width + bin]
    }

    /// Tone powers of symbol `s` of a transmission starting at row `t0`, tone 0 at bin
    /// `f`; None past the end of the slot.
    fn symbol(&self, t0: usize, f: usize, s: usize) -> Option<[f32; 8]> {
        let row = t0 + 4 * s;
        if row >= self.rows || f + 2 * (self.tones - 1) >= self.width {
            return None;
        }
        let mut p = [0f32; 8];
        for (j, v) in p.iter_mut().take(self.tones).enumerate() {
            *v = self.at(row, f + 2 * j);
        }
        Some(p)
    }

    /// One sync block's expected-tone power and mean tone power at (t0, f).
    fn block(&self, t0: usize, f: usize, start: usize, pattern: &[usize]) -> (f32, f32) {
        let (mut on, mut all) = (0f32, 0f32);
        for (k, &tone) in pattern.iter().enumerate() {
            if let Some(p) = self.symbol(t0, f, start + k) {
                on += p[tone];
                all += p[..self.tones].iter().sum::<f32>() / self.tones as f32;
            }
        }
        (on, all)
    }

    /// How strongly the sync blocks stand out at (t0, f).
    fn sync(&self, p: &Protocol, t0: usize, f: usize) -> f32 {
        let (mut on, mut all) = (0f32, 0f32);
        for &(start, pattern) in p.sync {
            let (o, a) = self.block(t0, f, start, pattern);
            on += o;
            all += a;
        }
        if all > 0.0 { on / all } else { 0.0 }
    }

    /// Sync where each block may sit up to `reach` bins (half tones) from `f` on its own:
    /// a signal that drifts or steps during its transmission keeps its score.
    fn sync_drifting(&self, p: &Protocol, t0: usize, f: usize, reach: usize) -> f32 {
        let (mut on, mut all) = (0f32, 0f32);
        for &(start, pattern) in p.sync {
            let best = (f.saturating_sub(reach)..=f + reach)
                .map(|ff| self.block(t0, ff, start, pattern))
                .max_by(|a, b| (a.0 / a.1.max(1e-20)).total_cmp(&(b.0 / b.1.max(1e-20))))
                .unwrap_or((0.0, 0.0));
            on += best.0;
            all += best.1;
        }
        if all > 0.0 { on / all } else { 0.0 }
    }
}

/// One candidate's tone powers after refinement.
struct Fine {
    /// Tone powers, [symbol][tone], at one frequency for the whole transmission.
    power: Vec<[f32; 8]>,
    /// The same with the frequency measured on each sync block separately and carried
    /// between them: follows a signal that drifts or steps during the transmission (an
    /// unsteady LNB, a sender's drifting oscillator).
    tracked: Vec<[f32; 8]>,
    /// Start of sync symbol 0 (seconds from the start of the samples) and tone 0 (Hz from
    /// the baseband centre), as refined.
    start_s: f64,
    freq_hz: f64,
}

/// Refines a coarse candidate (spectrogram row `t0`, tone 0 at `freq_hz`): the band
/// around it is mixed down and decimated, then the time (fine samples) and frequency
/// (hundredths of a tone) where the sync blocks fit best are searched, and every
/// symbol's tones are measured there by direct DFT. The coarse grid alone put signals
/// on the air up to a fifth of a symbol and a third of a tone off.
fn refine(p: &Protocol, samples: &[Complex<f32>], t0: usize, freq_hz: f64, taps: &[f32], wide: bool) -> Option<Fine> {
    use crate::demod::{FirDecimator, Mixer};
    let spacing = p.tone_spacing();
    let decimation = p.sps / p.fine_sps;
    let fine_rate = p.rate / decimation as f64;
    let step = p.sps / 4;
    let half = (p.tones as f64 - 1.0) / 2.0;
    // Centre of the tones at 0 Hz.
    let centre = freq_hz + half * spacing;
    // From one symbol before the coarse start to one after the end.
    let first = (t0 * step).saturating_sub(p.sps);
    let last = (t0 * step + (p.symbols + 1) * p.sps + taps.len()).min(samples.len());
    let mut x = samples[first..last].to_vec();
    Mixer::new(-centre, p.rate).mix(&mut x);
    let mut y = Vec::with_capacity(x.len() / decimation + 1);
    FirDecimator::new(taps, decimation).process(&x, &mut y);
    // Output k of the decimating filter is centred on input k·D − (taps−1)/2 (its history
    // starts with taps−1 zeros), so input i is output (i + delay)/D.
    let delay = (taps.len() - 1) as f64 / 2.0;
    let coarse = ((t0 * step - first) as f64 + delay) / decimation as f64;

    // DFT of one symbol at the tone frequencies plus `df`.
    let tone_power = |start: isize, df: f64| -> Option<[f32; 8]> {
        if start < 0 || start as usize + p.fine_sps > y.len() {
            return None;
        }
        let seg = &y[start as usize..start as usize + p.fine_sps];
        let mut out = [0f32; 8];
        for (j, v) in out.iter_mut().take(p.tones).enumerate() {
            let f = (j as f64 - half) * spacing + df;
            let w = Complex::from_polar(1.0f64, -2.0 * std::f64::consts::PI * f / fine_rate);
            let mut rot = Complex::new(1.0f64, 0.0);
            let mut acc = Complex::new(0.0f64, 0.0);
            for z in seg {
                acc += Complex::new(z.re as f64, z.im as f64) * rot;
                rot *= w;
            }
            *v = acc.norm_sqr() as f32;
        }
        Some(out)
    };
    let block_fit = |offset: isize, df: f64, start: usize, pattern: &[usize]| -> (f32, f32) {
        let (mut on, mut all) = (0f32, 0f32);
        for (k, &tone) in pattern.iter().enumerate() {
            if let Some(pw) = tone_power(offset + ((start + k) * p.fine_sps) as isize, df) {
                on += pw[tone];
                all += pw[..p.tones].iter().sum::<f32>();
            }
        }
        (on, all)
    };
    let sync_at = |offset: isize, df: f64| -> f32 {
        let (mut on, mut all) = (0f32, 0f32);
        for &(start, pattern) in p.sync {
            let (o, a) = block_fit(offset, df, start, pattern);
            on += o;
            all += a;
        }
        if all > 0.0 { on / all } else { 0.0 }
    };
    // Coarse then fine: ±8 fine samples in 2s and ±0.32 tone (±1.6 tones when `wide`) in
    // 0.08 tone, then around the best in single samples and 0.04 tone. (For FT8: ±40 ms,
    // ±2 Hz or ±10 Hz in 0.5 Hz, then 0.25 Hz.)
    let base = coarse.round() as isize;
    let mut best = (f32::MIN, base, 0.0f64);
    let reach = if wide { 20 } else { 4 };
    for dt in (-8..=8).step_by(2) {
        for k in -reach..=reach {
            let df = k as f64 * 0.08 * spacing;
            let s = sync_at(base + dt, df);
            if s > best.0 {
                best = (s, base + dt, df);
            }
        }
    }
    let (_, t_mid, f_mid) = best;
    for dt in -1..=1 {
        for k in -1..=1 {
            let df = f_mid + k as f64 * 0.04 * spacing;
            let s = sync_at(t_mid + dt, df);
            if s > best.0 {
                best = (s, t_mid + dt, df);
            }
        }
    }
    let (_, offset, df) = best;
    let power: Option<Vec<[f32; 8]>> = (0..p.symbols).map(|s| tone_power(offset + (s * p.fine_sps) as isize, df)).collect();

    // Each sync block's own frequency (±1.28 tones around the common one, ±2.56 in the
    // wide pass, where the common one can sit at one end of a long drift), then a
    // straight line between the block centres for the symbols in between.
    let span = if wide { 64 } else { 32 };
    let blocks: Vec<(f64, f64)> = p
        .sync
        .iter()
        .map(|&(start, pattern)| {
            let fit = |d: f64| {
                let (o, a) = block_fit(offset, d, start, pattern);
                o / a.max(1e-20)
            };
            let d = (-span..=span).map(|i| df + i as f64 * 0.04 * spacing).max_by(|&a, &b| fit(a).total_cmp(&fit(b))).unwrap_or(df);
            (start as f64 + pattern.len() as f64 / 2.0 - 0.5, d)
        })
        .collect();
    let df_at = |s: usize| -> f64 {
        let s = s as f64;
        if s <= blocks[0].0 {
            return blocks[0].1;
        }
        for w in blocks.windows(2) {
            if s <= w[1].0 {
                return w[0].1 + (w[1].1 - w[0].1) * (s - w[0].0) / (w[1].0 - w[0].0);
            }
        }
        blocks[blocks.len() - 1].1
    };
    let tracked: Option<Vec<[f32; 8]>> = (0..p.symbols).map(|s| tone_power(offset + (s * p.fine_sps) as isize, df_at(s))).collect();
    let start_s = (first as f64 + offset as f64 * decimation as f64 - delay) / p.rate;
    Some(Fine { power: power?, tracked: tracked?, start_s, freq_hz: freq_hz + df })
}

/// Soft bits from tone powers (log-max over the Gray-coded tones), scaled to a fixed
/// spread.
fn soft_bits(p: &Protocol, power: &[[f32; 8]]) -> [f32; ldpc::N] {
    let mut inverse = [0usize; 8];
    for (index, &tone) in p.gray.iter().enumerate() {
        inverse[tone] = index;
    }
    let b = p.bits_per_symbol;
    let mut llr = [0f32; ldpc::N];
    for j in 0..p.data_symbols() {
        let pw = &power[(p.data_symbol)(j)];
        let s: Vec<f32> = pw[..p.tones].iter().map(|v| (v + 1e-12).ln()).collect();
        for bit in 0..b {
            let (mut one, mut zero) = (f32::NEG_INFINITY, f32::NEG_INFINITY);
            for tone in 0..p.tones {
                if (inverse[tone] >> (b - 1 - bit)) & 1 == 1 {
                    one = one.max(s[tone]);
                } else {
                    zero = zero.max(s[tone]);
                }
            }
            llr[b * j + bit] = one - zero;
        }
    }
    let var = llr.iter().map(|v| v * v).sum::<f32>() / ldpc::N as f32;
    let scale = (24.0 / var.max(1e-9)).sqrt();
    llr.iter_mut().for_each(|v| *v *= scale);
    llr
}

/// Decodes the transmissions of one slot. `samples` are the baseband at the protocol's
/// rate, sample 0 `early_s` before the slot start; transmissions with tone 0 between
/// `lo_hz` and `hi_hz` (relative to the baseband centre) are looked for. Call signs go to
/// `book`.
pub fn decode_slot(p: &Protocol, samples: &[Complex<f32>], lo_hz: f64, hi_hz: f64, book: &mut CallBook) -> Vec<Decoded> {
    if samples.len() < p.sps * p.symbols {
        return Vec::new();
    }
    let spec = Spectrogram::new(p, samples, lo_hz, hi_hz);
    let latest = (((p.early_s + p.latest_start_s) * p.rate) as usize / spec.step).min(spec.rows.saturating_sub(1));
    let max_f = spec.width.saturating_sub(2 * p.tones);
    let decoder = Decoder::default();
    // Low-pass for refinement: the tones and a tone either side.
    let taps = crate::demod::lowpass((p.tones as f64 / 2.0 + 1.5) * p.tone_spacing() / p.rate, 513);
    let mut found: Vec<Decoded> = Vec::new();
    // Where decoded transmissions sit (start s, Hz): their sync peaks are not tried again.
    let mut used: Vec<(f64, f64)> = Vec::new();

    // First pass: the sync blocks at one frequency, as a steady signal has them.
    let steady = local_maxima(p, latest, max_f, |t0, f| spec.sync(p, t0, f));
    try_candidates(p, &spec, samples, &steady, false, &decoder, &taps, book, &mut found, &mut used);
    // Second pass, for what is left: each block may sit ±3 half tones from the others,
    // and the frequency is searched widely (a drifting or stepping signal). (Tests can
    // leave it out to measure what it adds.)
    #[cfg(test)]
    let second = std::env::var_os("SEDIR_FT8_ONE_PASS").is_none();
    #[cfg(not(test))]
    let second = true;
    if second {
        let drifting = local_maxima(p, latest, max_f, |t0, f| spec.sync_drifting(p, t0, f, 3));
        try_candidates(p, &spec, samples, &drifting, true, &decoder, &taps, book, &mut found, &mut used);
    }
    found
}

/// Local maxima of a sync score over (row, bin), strongest first.
fn local_maxima(p: &Protocol, latest: usize, max_f: usize, score: impl Fn(usize, usize) -> f32) -> Vec<(f32, usize, usize)> {
    // Random tones score about 1; 8 tones need 2, 4 tones 1.5.
    let min_sync = 1.0 + p.tones as f32 / 8.0;
    let mut scores = vec![0f32; (latest + 1) * max_f];
    for t0 in 0..=latest {
        for f in 0..max_f {
            scores[t0 * max_f + f] = score(t0, f);
        }
    }
    let mut candidates: Vec<(f32, usize, usize)> = Vec::new();
    for t0 in 0..=latest {
        for f in 0..max_f {
            let s = scores[t0 * max_f + f];
            if s < min_sync {
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
    candidates
}

/// Refines and decodes candidates, adding new messages to `found`.
#[allow(clippy::too_many_arguments)]
fn try_candidates(
    p: &Protocol,
    spec: &Spectrogram,
    samples: &[Complex<f32>],
    candidates: &[(f32, usize, usize)],
    wide: bool,
    decoder: &Decoder,
    taps: &[f32],
    book: &mut CallBook,
    found: &mut Vec<Decoded>,
    used: &mut Vec<(f64, f64)>,
) {
    let spacing = p.tone_spacing();
    for &(_, t0, f) in candidates {
        let coarse_hz = spec.first_hz + f as f64 * spec.bin_hz;
        let coarse_s = (t0 * spec.step) as f64 / p.rate;
        // A decoded transmission also leaves sync peaks next to itself.
        let near = spacing * if wide { 3.2 } else { 1.6 };
        if used.iter().any(|&(s, hz)| (s - coarse_s).abs() < 1.25 * p.sps as f64 / p.rate && (hz - coarse_hz).abs() < near) {
            continue;
        }
        let Some(fine) = refine(p, samples, t0, coarse_hz, taps, wide) else { continue };
        // One frequency first; if that fails, the frequency followed block by block.
        let Some((word, power)) = [&fine.power, &fine.tracked]
            .into_iter()
            .find_map(|pw| decoder.decode(&soft_bits(p, pw), 30).map(|w| (w, pw)))
        else {
            continue;
        };
        let msg = &word[..77];
        if msg.iter().all(|&b| b == 0) {
            continue;
        }
        let crc = word[77..91].iter().fold(0u16, |v, &b| (v << 1) | b as u16);
        if crc != ldpc::crc14(msg) {
            continue;
        }
        // FT4 scrambles the message (the CRC covers the scrambled bits).
        let bits: Vec<u8> = match p.scramble {
            Some(r) => msg.iter().zip(r.iter()).map(|(a, b)| a ^ b).collect(),
            None => msg.to_vec(),
        };
        let Some(text) = message::unpack(&bits, book) else { continue };
        used.push((fine.start_s, fine.freq_hz));
        if found.iter().any(|d| d.text == text) {
            continue;
        }
        // SNR: power in the sent tone against the others (each a one-tone noise band).
        let sent = p.tones_of(&word);
        let mut signal = 0f32;
        let mut others: Vec<f32> = Vec::with_capacity(p.symbols * p.tones);
        for (s, pw) in power.iter().enumerate() {
            signal += pw[sent[s]];
            others.extend(pw[..p.tones].iter().enumerate().filter(|(j, _)| *j != sent[s]).map(|(_, v)| *v));
        }
        signal /= p.symbols as f32;
        others.sort_by(f32::total_cmp);
        let noise_bin = others[others.len() / 2].max(1e-20);
        let snr = 10.0 * ((signal / noise_bin - 1.0).max(1e-3) as f64 * spacing / SNR_BANDWIDTH_HZ).log10();
        found.push(Decoded {
            text,
            snr_db: snr as f32,
            dt_s: (fine.start_s - p.early_s - p.start_s) as f32,
            freq_hz: fine.freq_hz,
        });
    }
}

#[cfg(test)]
pub mod tests {
    use super::*;
    use crate::ft8::message::{pack_standard, pack_text};
    use crate::ft8::{FT4, FT8};
    use std::f64::consts::PI;

    /// A transmission (continuous-phase FSK) added to `out` at the protocol's rate, its
    /// frequency moving by `drift_hz` over it. `start_s` (of sync symbol 0) is relative
    /// to the slot start; the buffer begins `early_s` before it.
    pub fn add_drifting(p: &Protocol, out: &mut [Complex<f32>], text: &str, freq_hz: f64, start_s: f64, amplitude: f32, drift_hz: f64) {
        let mut bits = pack_standard(text).or_else(|| pack_text(text)).expect(text);
        if let Some(r) = p.scramble {
            bits.iter_mut().zip(r.iter()).for_each(|(b, r)| *b ^= r);
        }
        let word = ldpc::encode(&bits);
        let t = p.tones_of(&word);
        let start = ((start_s + p.early_s) * p.rate) as usize;
        let mut phase = 0.0f64;
        for (s, &tone) in t.iter().enumerate() {
            let f = freq_hz + tone as f64 * p.tone_spacing() + drift_hz * s as f64 / p.symbols as f64;
            for i in 0..p.sps {
                let n = start + s * p.sps + i;
                if n < out.len() {
                    out[n] += Complex::from_polar(amplitude, phase as f32);
                }
                phase += 2.0 * PI * f / p.rate;
            }
        }
    }

    /// An FT8 transmission (for the worker's tests).
    pub fn add_signal(out: &mut [Complex<f32>], text: &str, freq_hz: f64, start_s: f64, amplitude: f32) {
        add_drifting(&FT8, out, text, freq_hz, start_s, amplitude, 0.0);
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

    /// Amplitude giving `snr_db` in 2500 Hz against complex noise of total power 1.
    pub fn amplitude_at(p: &Protocol, snr_db: f64) -> f32 {
        (10f64.powf(snr_db / 10.0) * SNR_BANDWIDTH_HZ / p.rate).sqrt() as f32
    }

    pub fn amplitude(snr_db: f64) -> f32 {
        amplitude_at(&FT8, snr_db)
    }

    /// Samples of one slot as the worker gives them.
    pub fn slot_samples(p: &Protocol) -> usize {
        ((p.early_s + p.decode_at_s) * p.rate) as usize
    }

    pub fn slot_len() -> usize {
        slot_samples(&FT8)
    }

    fn check(p: &Protocol, signals: &[(&str, f64, f64, f64)], drift: f64, seed: u32) {
        let mut x = noise(slot_samples(p), 1.0, seed);
        for &(text, f, t, snr) in signals {
            add_drifting(p, &mut x, text, f, t, amplitude_at(p, snr), drift);
        }
        let found = decode_slot(p, &x, -1400.0, 1400.0, &mut CallBook::default());
        for d in &found {
            println!("{} {:>6.1} dB {:+.2} s {:>7.1} Hz  {}", p.name, d.snr_db, d.dt_s, d.freq_hz, d.text);
        }
        for &(text, f, t, snr) in signals {
            let d = found.iter().find(|d| d.text == text).unwrap_or_else(|| panic!("{}: {text} not decoded", p.name));
            if drift == 0.0 {
                assert!((d.freq_hz - f).abs() < 0.6 * p.tone_spacing(), "{text}: {} Hz", d.freq_hz);
                assert!((d.dt_s as f64 - (t - p.start_s)).abs() < 0.05, "{text}: dt {}", d.dt_s);
                assert!((d.snr_db as f64 - snr).abs() < 3.0, "{text}: {} dB", d.snr_db);
            }
        }
    }

    #[test]
    fn ft8_decodes_several_signals_in_noise() {
        check(
            &FT8,
            &[
                ("CQ TA4SO KM46", -600.0, 0.5, -10.0),
                ("K1ABC W9XYZ -11", -200.0, 0.8, -15.0),
                ("W9XYZ K1ABC R-09", 333.0, 0.3, -5.0),
                ("DL1ABC TA4SO RR73", 900.0, 1.2, -12.0),
                // A sender whose clock runs 1.4 s ahead (seen on QO-100).
                ("CQ R4FDW LO23", 0.0, -0.9, -8.0),
            ],
            0.0,
            42,
        );
    }

    #[test]
    fn ft8_decodes_a_drifting_signal() {
        // 15 Hz of drift over the transmission: well over two tone spacings.
        check(&FT8, &[("CQ TA4SO KM46", 200.0, 0.5, -8.0)], 15.0, 11);
    }

    #[test]
    fn ft4_decodes_several_signals_in_noise() {
        check(
            &FT4,
            &[
                ("CQ TA4SO KM46", -600.0, 0.548, -8.0),
                ("K1ABC W9XYZ -11", -150.0, 0.7, -12.0),
                ("W9XYZ K1ABC RR73", 400.0, 0.3, -4.0),
                ("CQ DL1ABC JO31", 900.0, -0.4, -10.0),
            ],
            0.0,
            5,
        );
    }

    #[test]
    fn noise_alone_gives_nothing() {
        for p in [&FT8, &FT4] {
            let x = noise(slot_samples(p), 1.0, 7);
            assert!(decode_slot(p, &x, -1400.0, 1400.0, &mut CallBook::default()).is_empty(), "{}", p.name);
        }
    }

    /// Decodes every saved slot: `cargo test --lib ft8_saved_decode -- --ignored --nocapture`
    /// (SEDIR_FT8_DIR overrides the folder; SEDIR_FT8_ONE_PASS leaves out the second pass).
    #[test]
    #[ignore]
    fn ft8_saved_decode() {
        let dir = std::env::var_os("SEDIR_FT8_DIR").map(std::path::PathBuf::from).or_else(|| crate::ft8::worker::slot_dir(&FT8)).unwrap();
        let mut files: Vec<_> = std::fs::read_dir(&dir).unwrap().filter_map(|e| e.ok()).map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x == "cf32")).collect();
        files.sort();
        let mut total = 0;
        let mut book = CallBook::default();
        for path in &files {
            let samples = crate::ft8::worker::load_slot(path).unwrap();
            let started = std::time::Instant::now();
            let found = decode_slot(&FT8, &samples, -1500.0, 1500.0, &mut book);
            println!("{} · {} messages in {:.2} s", path.file_name().unwrap().to_string_lossy(), found.len(), started.elapsed().as_secs_f64());
            for d in &found {
                println!("   {:+5.1} dB {:+.2} s {:6.0} Hz  {}", d.snr_db, d.dt_s, d.freq_hz + 1600.0, d.text);
            }
            total += found.len();
        }
        println!("total {total} messages in {} slots", files.len());
    }

    /// Writes a saved FT8 slot's spectrogram as an 8-bit grey image (audio 100–3100 Hz
    /// across, time down): `SEDIR_FILE=slot.cf32 SEDIR_OUT=out.gray cargo test --lib
    /// ft8_picture -- --ignored --nocapture`; prints its size.
    #[test]
    #[ignore]
    fn ft8_picture() {
        let (Some(file), Some(out)) = (std::env::var_os("SEDIR_FILE"), std::env::var_os("SEDIR_OUT")) else { return };
        let samples = crate::ft8::worker::load_slot(std::path::Path::new(&file)).unwrap();
        let spec = Spectrogram::new(&FT8, &samples, -1500.0, 1500.0);
        let mut db: Vec<f32> = spec.power.iter().map(|p| 10.0 * (p + 1e-20).log10()).collect();
        let mut sorted = db.clone();
        sorted.sort_by(f32::total_cmp);
        let (lo, hi) = (sorted[sorted.len() / 2], sorted[sorted.len() * 999 / 1000]);
        db.iter_mut().for_each(|v| *v = ((*v - lo) / (hi - lo)).clamp(0.0, 1.0) * 255.0);
        std::fs::write(&out, db.iter().map(|v| *v as u8).collect::<Vec<u8>>()).unwrap();
        println!("size {}x{}", spec.width, spec.rows);
    }
}
