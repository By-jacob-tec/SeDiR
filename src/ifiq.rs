//! Turns the 12 kHz IF that Icom transceivers send over USB or the network (instead of
//! audio, when "AF/IF Output Select" is set to IF) into complex samples centred on the
//! transceiver's frequency.
//!
//! The IF is a real signal: the dial frequency sits at 12 kHz, upper sideband above it,
//! lower sideband below. Shifting it down by 12 kHz, low-pass filtering and keeping every
//! second sample gives complex samples at half the audio rate (24 kS/s from 48 kHz).
//!
//! When the radio sends demodulated audio instead (upper sideband), the audio is made
//! complex with a filter that keeps only positive frequencies: the dial frequency is then
//! at 0 Hz and the audio above it.

use num_complex::Complex;

/// Where the dial frequency sits in the IF (Hz).
pub const IF_HZ: f64 = 12_000.0;
/// Low-pass edge: the IF is at most ~15 kHz wide (±7.5 kHz).
const CUTOFF_HZ: f64 = 9_000.0;
const TAPS: usize = 95;
/// Audio: the pass band 0.5 – 12.5 kHz, as a 6 kHz low-pass moved up to 6.5 kHz.
const AUDIO_CENTRE_HZ: f64 = 6_500.0;
const AUDIO_HALF_WIDTH_HZ: f64 = 6_000.0;
const AUDIO_TAPS: usize = 127;

pub struct IfToIq {
    input_rate: f64,
    phase: f64,
    step: f64,
    /// Filter taps, newest-sample tap last.
    taps: Vec<Complex<f32>>,
    history: Vec<Complex<f32>>,
    /// Next history slot written.
    pos: usize,
    /// Input samples until the next output.
    skip: usize,
    /// The radio's IF is inverted (lower sideband above 12 kHz).
    invert: bool,
}

impl IfToIq {
    pub fn new(input_rate: f64, invert: bool) -> Self {
        let taps = lowpass(CUTOFF_HZ / input_rate, TAPS).into_iter().map(|t| Complex::new(t, 0.0)).collect();
        Self::with(input_rate, -std::f64::consts::TAU * IF_HZ / input_rate, taps, invert)
    }

    /// For demodulated (upper sideband) audio: the dial at 0 Hz, the audio above it.
    pub fn audio(input_rate: f64) -> Self {
        let lp = lowpass(AUDIO_HALF_WIDTH_HZ / input_rate, AUDIO_TAPS);
        let w = std::f64::consts::TAU * AUDIO_CENTRE_HZ / input_rate;
        // Reversed: the oldest sample meets the first tap.
        let n = lp.len();
        let taps = (0..n)
            .map(|k| {
                let j = n - 1 - k;
                let (s, c) = (w * j as f64).sin_cos();
                Complex::new(lp[j] * c as f32, lp[j] * s as f32)
            })
            .collect();
        Self::with(input_rate, 0.0, taps, false)
    }

    fn with(input_rate: f64, step: f64, taps: Vec<Complex<f32>>, invert: bool) -> Self {
        Self { input_rate, phase: 0.0, step, history: vec![Complex::new(0.0, 0.0); taps.len()], taps, pos: 0, skip: 0, invert }
    }

    /// Complex sample rate produced.
    pub fn output_rate(&self) -> f64 {
        self.input_rate / 2.0
    }

    /// Converts real IF samples (−1…1) to complex samples appended to `out`.
    pub fn process(&mut self, input: &[f32], out: &mut Vec<Complex<f32>>) {
        let n = self.taps.len();
        for &x in input {
            let (s, c) = self.phase.sin_cos();
            self.phase += self.step;
            if self.phase < -std::f64::consts::PI {
                self.phase += std::f64::consts::TAU;
            }
            // ×2: a real tone of amplitude a becomes a complex tone of amplitude a/2.
            self.history[self.pos] = Complex::new(2.0 * x * c as f32, 2.0 * x * s as f32);
            self.pos = (self.pos + 1) % n;
            if self.skip > 0 {
                self.skip -= 1;
                continue;
            }
            self.skip = 1;
            let (mut re, mut im) = (0.0f32, 0.0f32);
            // Oldest sample first: the slot about to be overwritten.
            for (k, t) in self.taps.iter().enumerate() {
                let h = self.history[(self.pos + k) % n];
                re += t.re * h.re - t.im * h.im;
                im += t.re * h.im + t.im * h.re;
            }
            out.push(Complex::new(re, if self.invert { -im } else { im }));
        }
    }
}

/// Windowed-sinc low-pass (Blackman), `cutoff` as a fraction of the sample rate.
fn lowpass(cutoff: f64, n: usize) -> Vec<f32> {
    let m = (n - 1) as f64;
    let mut taps: Vec<f64> = (0..n)
        .map(|i| {
            let x = i as f64 - m / 2.0;
            let sinc = if x == 0.0 { 2.0 * cutoff } else { (std::f64::consts::TAU * cutoff * x).sin() / (std::f64::consts::PI * x) };
            let w = 0.42 - 0.5 * (std::f64::consts::TAU * i as f64 / m).cos() + 0.08 * (2.0 * std::f64::consts::TAU * i as f64 / m).cos();
            sinc * w
        })
        .collect();
    let sum: f64 = taps.iter().sum();
    taps.iter_mut().for_each(|t| *t /= sum);
    taps.into_iter().map(|t| t as f32).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Power and frequency of the strongest tone, by a plain DFT at a few probe frequencies.
    fn tone_power(samples: &[Complex<f32>], rate: f64, hz: f64) -> f64 {
        let (mut re, mut im) = (0.0f64, 0.0f64);
        for (i, s) in samples.iter().enumerate() {
            let p = -std::f64::consts::TAU * hz * i as f64 / rate;
            let (sn, cs) = p.sin_cos();
            re += s.re as f64 * cs - s.im as f64 * sn;
            im += s.re as f64 * sn + s.im as f64 * cs;
        }
        let n = samples.len() as f64;
        (re * re + im * im) / (n * n)
    }

    fn convert(tone_hz: f64, invert: bool) -> Vec<Complex<f32>> {
        let rate = 48_000.0;
        let input: Vec<f32> = (0..48_000).map(|i| (0.5 * (std::f64::consts::TAU * tone_hz * i as f64 / rate).cos()) as f32).collect();
        let mut c = IfToIq::new(rate, invert);
        let mut out = Vec::new();
        // In two parts, to check that state carries over.
        c.process(&input[..10_001], &mut out);
        c.process(&input[10_001..], &mut out);
        assert_eq!(c.output_rate(), 24_000.0);
        assert_eq!(out.len(), 24_000);
        out[200..].to_vec()
    }

    #[test]
    fn upper_sideband_lands_above_the_centre() {
        // 1.5 kHz above the dial: 13.5 kHz in the IF.
        let out = convert(13_500.0, false);
        let at = tone_power(&out, 24_000.0, 1_500.0);
        let mirror = tone_power(&out, 24_000.0, -1_500.0);
        assert!((at.sqrt() - 0.5).abs() < 0.02, "amplitude {}", at.sqrt());
        assert!(10.0 * (at / mirror).log10() > 60.0, "image only {:.1} dB down", 10.0 * (at / mirror).log10());
        let inverted = convert(13_500.0, true);
        assert!(tone_power(&inverted, 24_000.0, -1_500.0) > 1e3 * tone_power(&inverted, 24_000.0, 1_500.0));
    }

    #[test]
    fn lower_sideband_lands_below_the_centre() {
        let out = convert(9_000.0, false);
        assert!(tone_power(&out, 24_000.0, -3_000.0) > 1e5 * tone_power(&out, 24_000.0, 3_000.0));
    }

    #[test]
    fn audio_lands_above_the_centre() {
        let rate = 48_000.0;
        let input: Vec<f32> = (0..48_000).map(|i| (0.5 * (std::f64::consts::TAU * 1_500.0 * i as f64 / rate).cos()) as f32).collect();
        let mut c = IfToIq::audio(rate);
        let mut out = Vec::new();
        c.process(&input, &mut out);
        let out = &out[200..];
        let at = tone_power(out, 24_000.0, 1_500.0).sqrt();
        let mirror = tone_power(out, 24_000.0, -1_500.0).sqrt();
        assert!((at - 0.5).abs() < 0.03, "amplitude {at}");
        assert!(mirror < 0.005, "mirror {mirror}");
    }

    #[test]
    fn mirror_of_a_far_tone_is_suppressed() {
        // 23 kHz in the IF: its mirror lands at 13 kHz before decimation and would fold to
        // −11 kHz without the filter.
        let out = convert(23_000.0, false);
        let leak = tone_power(&out, 24_000.0, -11_000.0);
        assert!(leak < 1e-6, "leak {leak}");
    }
}
