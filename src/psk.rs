//! PSK31 to text (G3PLX): differential BPSK at 31.25 baud with cosine-shaped phase
//! reversals; a 0 bit is a reversal, a 1 none. Characters are Varicode, separated by
//! two zeros. The channel is brought down to ~1 kHz; an averaged spectrum finds the
//! signal, a squaring loop on the symbol differences keeps the carrier, and the symbol
//! clock sits where the one-symbol correlation is strongest (16 phases per symbol).

use std::sync::Arc;

use num_complex::Complex;
use rustfft::{Fft, FftPlanner};

use crate::demod::{FirDecimator, TextDecoder, lowpass, taps_for};

const WORK_RATE: f64 = 1000.0;
pub const BAUD: f64 = 31.25;
/// ~1 Hz bins over ~1 s.
const FFT_SIZE: usize = 1024;
const AFC_EVERY: usize = 250;
const MAX_TEXT: usize = 400;
/// Clock phases per symbol.
const PHASES: usize = 16;
/// Signal power (±20 Hz around it, per bin) over the median of the band.
const SNR_ON: f32 = 4.0;
const SNR_OFF: f32 = 2.0;
/// The signal's spectrum is within this of its carrier (Hz).
const SIGNAL_HALF_HZ: f64 = 20.0;

/// PSK31 Varicode for ASCII 0–127 (G3PLX).
const VARICODE: [&str; 128] = [
    "1010101011", "1011011011", "1011101101", "1101110111", "1011101011", "1101011111", "1011101111", "1011111101",
    "1011111111", "11101111", "11101", "1101101111", "1011011101", "11111", "1101110101", "1110101011",
    "1011110111", "1011110101", "1110101101", "1110101111", "1101011011", "1101101011", "1101101101", "1101010111",
    "1101111011", "1101111101", "1110110111", "1101010101", "1101011101", "1110111011", "1011111011", "1101111111",
    "1", "111111111", "101011111", "111110101", "111011011", "1011010101", "1010111011", "101111111",
    "11111011", "11110111", "101101111", "111011111", "1110101", "110101", "1010111", "110101111",
    "10110111", "10111101", "11101101", "11111111", "101110111", "101011011", "101101011", "110101101",
    "110101011", "110110111", "11110101", "110111101", "111101101", "1010101", "111010111", "1010101111",
    "1010111101", "1111101", "11101011", "10101101", "10110101", "1110111", "11011011", "11111101",
    "101010101", "1111111", "111111101", "101111101", "11010111", "10111011", "11011101", "10101011",
    "11010101", "111011101", "10101111", "1101111", "1101101", "101010111", "110110101", "101011101",
    "101110101", "101111011", "1010101101", "111110111", "111101111", "111111011", "1010111111", "101101101",
    "1011011111", "1011", "1011111", "101111", "101101", "11", "111101", "1011011",
    "101011", "1101", "111101011", "10111111", "11011", "111011", "1111", "111",
    "111111", "110111111", "10101", "10111", "101", "110111", "1111011", "1101011",
    "11011111", "1011101", "111010101", "1010110111", "110111011", "1010110101", "1011010111", "1110110101",
];

fn code_value(s: &str) -> u32 {
    s.bytes().fold(0, |v, b| (v << 1) | u32::from(b == b'1'))
}

/// Character of a Varicode (bits as a number, without the separating zeros).
fn lookup(code: u32) -> Option<char> {
    VARICODE.iter().position(|s| code_value(s) == code).map(|i| i as u8 as char)
}

pub struct PskDecoder {
    decimator: FirDecimator<Complex<f32>>,
    rate: f64,
    scratch: Vec<Complex<f32>>,
    ring: Vec<Complex<f32>>,
    ring_pos: usize,
    since_afc: usize,
    fft: Arc<dyn Fft<f32>>,
    window: Vec<f32>,
    spectrum: Vec<f32>,
    limit_hz: f64,
    /// Carrier from the channel centre (Hz) and the oscillator following it.
    freq_hz: f64,
    phase: f64,
    /// One-symbol moving correlation.
    taps: Vec<Complex<f64>>,
    tap_pos: usize,
    sum: Complex<f64>,
    sum_count: u32,
    spb: f64,
    clock: f64,
    energy: [f32; PHASES],
    best_phase: usize,
    last_phase: usize,
    previous: Complex<f64>,
    /// Recent symbol magnitude, to weigh the carrier loop.
    level: f64,
    code: u32,
    text: String,
    pub signal: bool,
}

impl PskDecoder {
    /// `rate`: channel rate (complex baseband centred on the listening frequency);
    /// `half_band_hz`: half the channel filter's width.
    pub fn new(rate: f64, half_band_hz: f64) -> Self {
        let factor = (rate / WORK_RATE).round().max(1.0) as usize;
        let work = rate / factor as f64;
        let cut = (half_band_hz + 30.0).min(0.45 * work);
        let decimator = FirDecimator::new(&lowpass(cut / rate, taps_for(rate, 100.0, 601)), factor);
        let window = (0..FFT_SIZE).map(|i| (0.5 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / FFT_SIZE as f64).cos()) as f32).collect();
        let spb = work / BAUD;
        Self {
            decimator,
            rate: work,
            scratch: Vec::new(),
            ring: vec![Complex::default(); FFT_SIZE],
            ring_pos: 0,
            since_afc: 0,
            fft: FftPlanner::new().plan_fft_forward(FFT_SIZE),
            window,
            spectrum: vec![0.0; FFT_SIZE],
            limit_hz: half_band_hz.min(0.45 * work),
            freq_hz: 0.0,
            phase: 0.0,
            taps: vec![Complex::default(); spb.round() as usize],
            tap_pos: 0,
            sum: Complex::default(),
            sum_count: 0,
            spb,
            clock: 0.0,
            energy: [0.0; PHASES],
            best_phase: 0,
            last_phase: 0,
            previous: Complex::default(),
            level: 0.0,
            code: 0,
            text: String::new(),
            signal: false,
        }
    }

    /// Carrier from the listening frequency (Hz).
    pub fn offset_hz(&self) -> f64 {
        self.freq_hz
    }

    fn afc(&mut self) {
        let mut buf: Vec<Complex<f32>> =
            (0..FFT_SIZE).map(|i| self.ring[(self.ring_pos + i) % FFT_SIZE] * self.window[i]).collect();
        self.fft.process(&mut buf);
        for (s, z) in self.spectrum.iter_mut().zip(&buf) {
            *s = *s * 0.6 + z.norm_sqr() * 0.4;
        }
        let bin_hz = self.rate / FFT_SIZE as f64;
        let n = FFT_SIZE as i64;
        let at = |k: i64| self.spectrum[k.rem_euclid(n) as usize];
        let limit = (self.limit_hz / bin_hz) as i64;
        let mut band: Vec<f32> = (-limit..=limit).map(at).collect();
        if band.is_empty() {
            return;
        }
        let mid = band.len() / 2;
        let noise = *band.select_nth_unstable_by(mid, f32::total_cmp).1 + 1e-30;
        // Strongest ±20 Hz stretch inside the band, then its centroid.
        let half = (SIGNAL_HALF_HZ / bin_hz).round() as i64;
        let reach = (limit - half).max(0);
        let (mut best, mut best_sum) = (0i64, -1.0f32);
        for c in -reach..=reach {
            let s: f32 = (c - half..=c + half).map(at).sum();
            if s > best_sum {
                (best, best_sum) = (c, s);
            }
        }
        let snr = best_sum / (2 * half + 1) as f32 / noise;
        let on = snr > if self.signal { SNR_OFF } else { SNR_ON };
        if on {
            let (mut w, mut m) = (0.0f64, 0.0f64);
            for k in best - half..=best + half {
                let p = (at(k) - noise).max(0.0) as f64;
                w += p;
                m += p * k as f64;
            }
            let centroid = if w > 0.0 { m / w * bin_hz } else { best as f64 * bin_hz };
            // The loop below holds the carrier closely; only a clear miss moves it here.
            if !self.signal || (centroid - self.freq_hz).abs() > 6.0 {
                self.freq_hz = centroid;
            }
        }
        self.signal = on;
    }

    fn sample(&mut self, z: Complex<f32>) {
        self.ring[self.ring_pos] = z;
        self.ring_pos = (self.ring_pos + 1) % FFT_SIZE;
        self.since_afc += 1;
        if self.since_afc == AFC_EVERY {
            self.since_afc = 0;
            self.afc();
        }
        let tau = 2.0 * std::f64::consts::PI;
        let z = Complex::new(z.re as f64, z.im as f64) * Complex::from_polar(1.0, -self.phase);
        self.phase = (self.phase + tau * self.freq_hz / self.rate) % tau;
        self.sum += z - self.taps[self.tap_pos];
        self.taps[self.tap_pos] = z;
        self.tap_pos = (self.tap_pos + 1) % self.taps.len();
        self.sum_count += 1;
        if self.sum_count == 20_000 {
            self.sum_count = 0;
            self.sum = self.taps.iter().sum();
        }

        // Symbol clock: energy of the correlation at each of 16 phases.
        self.clock += 1.0;
        if self.clock >= self.spb {
            self.clock -= self.spb;
            self.best_phase = (0..PHASES).max_by(|&a, &b| self.energy[a].total_cmp(&self.energy[b])).unwrap_or(0);
        }
        let phase = ((self.clock / self.spb * PHASES as f64) as usize).min(PHASES - 1);
        let e = &mut self.energy[phase];
        *e = *e * 0.99 + self.sum.norm_sqr() as f32 * 0.01;
        if phase != self.last_phase && phase == self.best_phase {
            self.symbol(self.sum);
        }
        self.last_phase = phase;
    }

    fn symbol(&mut self, s: Complex<f64>) {
        let d = s * self.previous.conj();
        self.previous = s;
        let mag = s.norm();
        self.level = self.level * 0.95 + mag * 0.05;
        if self.signal && mag > 0.3 * self.level && d.norm() > 0.0 {
            // Squaring removes the modulation; what is left is the carrier's turn per
            // symbol.
            let turn = (d * d).arg() / 2.0;
            self.freq_hz += 0.08 * turn * BAUD / (2.0 * std::f64::consts::PI);
            self.freq_hz = self.freq_hz.clamp(-self.limit_hz, self.limit_hz);
        }
        let bit = u32::from(d.re > 0.0);
        self.code = (self.code << 1) | bit;
        if self.code & 3 == 0 {
            let c = self.code >> 2;
            self.code = 0;
            if c != 0 && self.signal {
                self.character(lookup(c));
            }
        } else if self.code >= 1 << 14 {
            // No character is this long: noise.
            self.code &= 3;
        }
    }

    fn character(&mut self, c: Option<char>) {
        match c {
            Some('\n' | '\r' | '\t' | ' ') => {
                if !self.text.is_empty() && !self.text.ends_with(' ') {
                    self.text.push(' ');
                }
            }
            Some(c) if !c.is_control() => self.text.push(c),
            _ => {}
        }
        if self.text.len() > MAX_TEXT + 100 {
            let cut = self.text.char_indices().nth(self.text.chars().count() - MAX_TEXT).map_or(0, |(i, _)| i);
            self.text.drain(..cut);
        }
    }
}

impl TextDecoder for PskDecoder {
    fn push(&mut self, channel: &[Complex<f32>]) {
        let mut out = std::mem::take(&mut self.scratch);
        out.clear();
        self.decimator.process(channel, &mut out);
        for &z in &out {
            self.sample(z);
        }
        self.scratch = out;
    }

    fn text(&self) -> &str {
        &self.text
    }

    fn detail(&self) -> String {
        if self.signal { format!("{:+.1} Hz", self.freq_hz) } else { String::new() }
    }

    fn signal(&self) -> bool {
        self.signal
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rtty::tests::add_noise;

    #[test]
    fn varicode_is_well_formed() {
        let mut seen = std::collections::HashSet::new();
        for (i, s) in VARICODE.iter().enumerate() {
            assert!(s.starts_with('1') && s.ends_with('1') && !s.contains("00"), "{i}: {s}");
            assert!(seen.insert(*s), "{i}: {s} twice");
        }
        assert_eq!(lookup(code_value("11")), Some('e'));
        assert_eq!(lookup(code_value("1")), Some(' '));
        assert_eq!(lookup(code_value("1111101")), Some('A'));
    }

    /// PSK31 at `rate`: idle reversals, the text, idle again; carrier at `freq`.
    fn psk31(text: &str, rate: f64, freq: f64, amplitude: f32) -> Vec<Complex<f32>> {
        let mut bits = vec![0u8; 64];
        for c in text.bytes() {
            bits.extend(VARICODE[c as usize].bytes().map(|b| u8::from(b == b'1')));
            bits.extend([0, 0]);
        }
        bits.extend(vec![0u8; 32]);
        let spb = rate / BAUD;
        let mut out = Vec::new();
        let (mut previous, mut polarity) = (1.0f64, 1.0f64);
        let mut t = 0.0;
        let mut n = 0usize;
        for &b in &bits {
            if b == 0 {
                polarity = -polarity;
            }
            while t < spb {
                // Cosine change from the old polarity to the new over the symbol.
                let x = t / spb;
                let v = previous * (1.0 + (std::f64::consts::PI * x).cos()) / 2.0 + polarity * (1.0 - (std::f64::consts::PI * x).cos()) / 2.0;
                let ph = 2.0 * std::f64::consts::PI * freq * n as f64 / rate;
                out.push(Complex::from_polar(amplitude * v as f32, ph as f32));
                t += 1.0;
                n += 1;
            }
            t -= spb;
            previous = polarity;
        }
        out
    }

    const MESSAGE: &str = "CQ CQ CQ de TA4SO TA4SO pse k - Mugla 73!";

    #[test]
    fn decodes_psk31_off_frequency() {
        let rate = 16_000.0;
        let mut x = psk31(MESSAGE, rate, 9.0, 0.1);
        // About 10 dB in 31 Hz.
        add_noise(&mut x, 0.1 * (rate as f32 / 2.0 / 31.0 / 10.0).sqrt(), 5);
        let mut d = PskDecoder::new(rate, 100.0);
        for chunk in x.chunks(1600) {
            d.push(chunk);
        }
        println!("{:?} · {}", d.text(), d.detail());
        assert!(d.text().contains("de TA4SO TA4SO pse k - Mugla 73!"), "{:?}", d.text());
        assert!((d.offset_hz() - 9.0).abs() < 1.0, "{}", d.offset_hz());
    }

    #[test]
    fn noise_alone_gives_nothing() {
        let rate = 16_000.0;
        let mut x = vec![Complex::default(); 16_000 * 10];
        add_noise(&mut x, 0.05, 13);
        let mut d = PskDecoder::new(rate, 100.0);
        for chunk in x.chunks(1600) {
            d.push(chunk);
        }
        assert!(d.text().len() < 5, "{:?}", d.text());
    }
}
