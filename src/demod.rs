//! Analog demodulation: from the wideband IQ stream to audio at the sound card's rate.
//!
//! Chain: shift the listening frequency to zero → half-band decimators down to a rate
//! that suits the mode → channel filter → demodulator → audio filter → resampler.
//! Every filter only computes the samples it keeps, so even 20 MS/s fits on one core.

use std::f64::consts::PI;
use std::ops::{Add, Mul};

use num_complex::Complex;

/// Values a filter can work on.
pub trait Sample: Copy + Default + Add<Output = Self> + Mul<f32, Output = Self> {}
impl Sample for f32 {}
impl Sample for Complex<f32> {}

/// Windowed-sinc low-pass filter (Blackman window, unity gain at DC). `cutoff` is in
/// cycles per sample (0..0.5); `taps` is made odd.
pub fn lowpass(cutoff: f64, taps: usize) -> Vec<f32> {
    let n = taps | 1;
    let mid = (n / 2) as f64;
    let mut h: Vec<f64> = (0..n)
        .map(|i| {
            let k = i as f64 - mid;
            let sinc = if k == 0.0 { 2.0 * cutoff } else { (2.0 * PI * cutoff * k).sin() / (PI * k) };
            // Window over n + 2 points without its zero ends, so no tap is wasted.
            let x = 2.0 * PI * (i + 1) as f64 / (n + 1) as f64;
            sinc * (0.42 - 0.5 * x.cos() + 0.08 * (2.0 * x).cos())
        })
        .collect();
    let sum: f64 = h.iter().sum();
    h.iter_mut().for_each(|v| *v /= sum);
    h.into_iter().map(|v| v as f32).collect()
}

/// Number of taps for a Blackman low-pass with the given transition width (Hz).
pub fn taps_for(rate: f64, transition: f64, max: usize) -> usize {
    ((5.5 * rate / transition.max(1.0)) as usize).clamp(15, max) | 1
}

/// FIR filter that keeps one output in `factor`. Zero taps (half-band filters have every
/// other one zero) are skipped.
pub struct FirDecimator<T: Sample> {
    taps: Vec<(usize, f32)>,
    len: usize,
    factor: usize,
    buf: Vec<T>,
    /// Start of the next output's window within `buf`.
    next: usize,
}

impl<T: Sample> FirDecimator<T> {
    pub fn new(taps: &[f32], factor: usize) -> Self {
        let sparse = taps.iter().copied().enumerate().filter(|(_, t)| t.abs() > 1e-9).collect();
        Self { taps: sparse, len: taps.len(), factor: factor.max(1), buf: vec![T::default(); taps.len() - 1], next: 0 }
    }

    pub fn process(&mut self, input: &[T], out: &mut Vec<T>) {
        self.buf.extend_from_slice(input);
        let mut i = self.next;
        while i + self.len <= self.buf.len() {
            let window = &self.buf[i..i + self.len];
            let mut acc = T::default();
            for &(k, t) in &self.taps {
                acc = acc + window[k] * t;
            }
            out.push(acc);
            i += self.factor;
        }
        let drop = i.min(self.buf.len());
        self.buf.drain(..drop);
        self.next = i - drop;
    }
}

/// Decimate-by-2 half-band filter: passes up to ~0.26 of the output rate cleanly.
fn halfband<T: Sample>() -> FirDecimator<T> {
    FirDecimator::new(&lowpass(0.25, 23), 2)
}

/// Complex oscillator that shifts a signal by a frequency.
pub struct Mixer {
    phase: Complex<f64>,
    step: Complex<f64>,
    hz: f64,
    rate: f64,
    count: u32,
}

impl Mixer {
    pub fn new(hz: f64, rate: f64) -> Self {
        let mut m = Self { phase: Complex::new(1.0, 0.0), step: Complex::new(1.0, 0.0), hz: f64::NAN, rate, count: 0 };
        m.set(hz);
        m
    }

    /// Changes the shift keeping the phase continuous.
    pub fn set(&mut self, hz: f64) {
        if hz != self.hz {
            self.hz = hz;
            self.step = Complex::from_polar(1.0, 2.0 * PI * hz / self.rate);
        }
    }

    pub fn next(&mut self) -> Complex<f32> {
        let p = self.phase;
        self.phase *= self.step;
        self.count += 1;
        if self.count == 4096 {
            self.count = 0;
            self.phase /= self.phase.norm();
        }
        Complex::new(p.re as f32, p.im as f32)
    }

    pub fn mix(&mut self, samples: &mut [Complex<f32>]) {
        for s in samples {
            *s *= self.next();
        }
    }
}

/// Demodulation modes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Mode {
    Am,
    Nfm,
    Wfm,
    Usb,
    Lsb,
    Cw,
}

/// Pitch of a CW signal tuned exactly (Hz).
pub const CW_TONE: f64 = 700.0;
/// Lowest audio frequency passed in SSB (Hz).
const SSB_LOW: f64 = 300.0;

impl Mode {
    pub const ALL: [Mode; 6] = [Mode::Am, Mode::Nfm, Mode::Wfm, Mode::Usb, Mode::Lsb, Mode::Cw];

    pub fn name(self) -> &'static str {
        match self {
            Mode::Am => "AM",
            Mode::Nfm => "NFM",
            Mode::Wfm => "WFM",
            Mode::Usb => "USB",
            Mode::Lsb => "LSB",
            Mode::Cw => "CW",
        }
    }

    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|m| m.name().eq_ignore_ascii_case(name.trim()))
    }

    /// Lowest working rate of the channel: the half-band chain stops between this and
    /// twice this, keeping the passband inside the half-bands' clean region.
    fn channel_rate(self) -> f64 {
        match self {
            Mode::Wfm => 480e3,
            Mode::Nfm => 64e3,
            Mode::Am => 32e3,
            Mode::Usb | Mode::Lsb | Mode::Cw => 16e3,
        }
    }

    /// Default filter width (Hz).
    pub fn default_bandwidth(self) -> f64 {
        match self {
            Mode::Wfm => 180e3,
            Mode::Nfm => 12.5e3,
            Mode::Am => 9e3,
            Mode::Usb | Mode::Lsb => 2.4e3,
            Mode::Cw => 500.0,
        }
    }

    pub fn bandwidth_range(self) -> (f64, f64) {
        match self {
            Mode::Wfm => (80e3, 220e3),
            Mode::Nfm => (5e3, 25e3),
            Mode::Am => (2e3, 12e3),
            Mode::Usb | Mode::Lsb => (1e3, 3.6e3),
            Mode::Cw => (100.0, 1.5e3),
        }
    }

    /// Mouse wheel step suited to the mode (Hz).
    pub fn default_step(self) -> f64 {
        match self {
            Mode::Wfm => 100e3,
            Mode::Nfm => 12.5e3,
            Mode::Am => 5e3,
            Mode::Usb | Mode::Lsb => 100.0,
            Mode::Cw => 50.0,
        }
    }

    /// Received band relative to the listening frequency (Hz): (low, high).
    pub fn passband(self, bandwidth: f64) -> (f64, f64) {
        match self {
            Mode::Usb => (SSB_LOW, SSB_LOW + bandwidth),
            Mode::Lsb => (-SSB_LOW - bandwidth, -SSB_LOW),
            _ => (-bandwidth / 2.0, bandwidth / 2.0),
        }
    }

    /// Whether a click on a found signal should tune to its centre (carrier-less modes
    /// are tuned to the edge, which the spectrum cannot tell exactly).
    pub fn snaps_to_center(self) -> bool {
        matches!(self, Mode::Am | Mode::Nfm | Mode::Wfm)
    }
}

/// Slow automatic gain for AM, SSB and CW: fast attack, ~0.4 s decay.
struct Agc {
    peak: f32,
    attack: f32,
    decay: f32,
}

impl Agc {
    fn new(rate: f64) -> Self {
        Self { peak: 1e-3, attack: (1.0 - (-1.0 / (0.002 * rate)).exp()) as f32, decay: 0.5f64.powf(1.0 / (0.4 * rate)) as f32 }
    }

    fn apply(&mut self, x: f32) -> f32 {
        let env = x.abs();
        if env > self.peak {
            self.peak += (env - self.peak) * self.attack;
        } else {
            self.peak = (self.peak * self.decay).max(1e-7);
        }
        (x * 0.3 / self.peak).clamp(-1.0, 1.0)
    }
}

/// Linear-interpolating resampler; the ratio may be nudged while running (to follow the
/// sound card's clock).
struct Resampler {
    /// Position of the next output between `prev` (0) and the current input (1).
    pos: f64,
    prev: f32,
}

impl Resampler {
    fn process(&mut self, input: &[f32], step: f64, out: &mut Vec<f32>) {
        for &x in input {
            while self.pos < 1.0 {
                out.push(self.prev + (x - self.prev) * self.pos as f32);
                self.pos += step;
            }
            self.pos -= 1.0;
            self.prev = x;
        }
    }
}

/// Settings a chain is built for; any change rebuilds it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChainConfig {
    pub mode: Mode,
    pub bandwidth: f64,
    pub input_rate: f64,
    pub audio_rate: f64,
}

/// One listening channel.
pub struct Chain {
    pub config: ChainConfig,
    mixer: Mixer,
    halfbands: Vec<FirDecimator<Complex<f32>>>,
    /// Shift that centres a one-sided passband (SSB, CW) before the channel filter.
    side_shift: Option<(Mixer, Mixer)>,
    channel: FirDecimator<Complex<f32>>,
    channel_rate: f64,
    last: Complex<f32>,
    deemphasis: Option<(f32, f32)>,
    dc: f32,
    agc: Option<Agc>,
    audio_halfbands: Vec<FirDecimator<f32>>,
    audio_filter: Option<FirDecimator<f32>>,
    audio_in_rate: f64,
    resampler: Resampler,
    /// Mean channel power of the last call (dBFS).
    pub level_db: f32,
    scratch: Vec<Complex<f32>>,
    scratch2: Vec<Complex<f32>>,
}

impl Chain {
    pub fn new(config: ChainConfig) -> Self {
        let ChainConfig { mode, bandwidth, input_rate, .. } = config;
        let mut rate = input_rate;
        let mut halfbands = Vec::new();
        while rate / 2.0 >= mode.channel_rate() {
            halfbands.push(halfband());
            rate /= 2.0;
        }
        let (lo, hi) = mode.passband(bandwidth);
        let center = (lo + hi) / 2.0;
        let half = (hi - lo) / 2.0;
        let side_shift = matches!(mode, Mode::Usb | Mode::Lsb | Mode::Cw).then(|| {
            let back = if mode == Mode::Cw { CW_TONE } else { center };
            (Mixer::new(-center, rate), Mixer::new(back, rate))
        });
        // WFM gets a wide transition: its filter runs at the highest rate.
        let transition = if mode == Mode::Wfm { 40e3 } else { (half * 0.25).clamp(80.0, 5e3) };
        let channel = FirDecimator::new(&lowpass(half / rate, taps_for(rate, transition, 1201)), 1);

        // Audio side: WFM is demodulated at a high rate and brought down before filtering;
        // the rate stays above ~80 kHz so the stereo subcarrier (23–53 kHz) cannot fold
        // into the audio band.
        let mut audio_rate_in = rate;
        let mut audio_halfbands = Vec::new();
        while audio_rate_in / 2.0 >= 80e3 && mode == Mode::Wfm {
            audio_halfbands.push(halfband());
            audio_rate_in /= 2.0;
        }
        let audio_cut = match mode {
            Mode::Wfm => Some(15e3),
            Mode::Nfm => Some(3.5e3),
            Mode::Am => Some((bandwidth / 2.0).min(5e3)),
            _ => None,
        };
        let audio_filter =
            audio_cut.map(|c| FirDecimator::new(&lowpass(c / audio_rate_in, taps_for(audio_rate_in, 1.5e3, 401)), 1));
        let deemphasis = match mode {
            // 50 µs (Europe, Turkey).
            Mode::Wfm => Some(((1.0 - (-1.0 / (rate * 50e-6)).exp()) as f32, 0.0)),
            _ => None,
        };
        let agc = matches!(mode, Mode::Am | Mode::Usb | Mode::Lsb | Mode::Cw).then(|| Agc::new(audio_rate_in));
        Self {
            config,
            mixer: Mixer::new(0.0, input_rate),
            halfbands,
            side_shift,
            channel,
            channel_rate: rate,
            last: Complex::default(),
            deemphasis,
            dc: 0.0,
            agc,
            audio_halfbands,
            audio_filter,
            audio_in_rate: audio_rate_in,
            resampler: Resampler { pos: 0.0, prev: 0.0 },
            level_db: -150.0,
            scratch: Vec::new(),
            scratch2: Vec::new(),
        }
    }

    /// Demodulates `input` (at the configured input rate), listening `offset_hz` from the
    /// centre of the band. Audio is appended to `out`; `speed` slightly above 1 produces
    /// fewer samples (used to keep the sound card's buffer level). With `squelch_db` set,
    /// the output is silent while the channel power is below it.
    pub fn process(&mut self, input: &[Complex<f32>], offset_hz: f64, speed: f64, squelch_db: Option<f32>, out: &mut Vec<f32>) {
        let mode = self.config.mode;
        self.mixer.set(-offset_hz);
        let mut a = std::mem::take(&mut self.scratch);
        let mut b = std::mem::take(&mut self.scratch2);
        a.clear();
        a.extend_from_slice(input);
        self.mixer.mix(&mut a);
        for hb in &mut self.halfbands {
            b.clear();
            hb.process(&a, &mut b);
            std::mem::swap(&mut a, &mut b);
        }
        if let Some((down, _)) = &mut self.side_shift {
            down.mix(&mut a);
        }
        b.clear();
        self.channel.process(&a, &mut b);
        let channel = &mut b;
        let power = channel.iter().map(|z| z.norm_sqr()).sum::<f32>() / channel.len().max(1) as f32;
        self.level_db = 10.0 * (power + 1e-20).log10();

        let rate = self.channel_rate as f32;
        let mut audio: Vec<f32> = Vec::with_capacity(channel.len());
        match mode {
            Mode::Wfm | Mode::Nfm => {
                let deviation = if mode == Mode::Wfm { 75e3 } else { (self.config.bandwidth * 0.2) as f32 };
                let scale = rate / (2.0 * std::f32::consts::PI * deviation);
                for &z in channel.iter() {
                    let d = (z * self.last.conj()).arg() * scale;
                    self.last = z;
                    let v = match &mut self.deemphasis {
                        Some((alpha, state)) => {
                            *state += *alpha * (d - *state);
                            *state
                        }
                        None => d,
                    };
                    audio.push(v * 0.5);
                }
            }
            Mode::Am => {
                for &z in channel.iter() {
                    let env = z.norm();
                    self.dc += (env - self.dc) * 1e-3;
                    audio.push(env - self.dc);
                }
            }
            Mode::Usb | Mode::Lsb | Mode::Cw => {
                let (_, up) = self.side_shift.as_mut().expect("side shift");
                for &z in channel.iter() {
                    audio.push((z * up.next()).re);
                }
            }
        }
        self.scratch = a;
        self.scratch2 = b;

        for hb in &mut self.audio_halfbands {
            let mut next = Vec::with_capacity(audio.len() / 2 + 1);
            hb.process(&audio, &mut next);
            audio = next;
        }
        if let Some(f) = &mut self.audio_filter {
            let mut next = Vec::with_capacity(audio.len());
            f.process(&audio, &mut next);
            audio = next;
        }
        if let Some(agc) = &mut self.agc {
            audio.iter_mut().for_each(|x| *x = agc.apply(*x));
        }
        if squelch_db.is_some_and(|s| self.level_db < s) {
            audio.fill(0.0);
        }
        let step = self.audio_in_rate / self.config.audio_rate * speed;
        self.resampler.process(&audio, step, out);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tone(hz: f64, rate: f64, n: usize, amp: f32) -> Vec<Complex<f32>> {
        (0..n).map(|i| Complex::from_polar(amp, (2.0 * PI * hz * i as f64 / rate) as f32)).collect()
    }

    /// Power of `freq` in real audio (single-bin DFT), relative to full scale.
    fn bin_power(x: &[f32], freq: f64, rate: f64) -> f64 {
        let (mut re, mut im) = (0.0, 0.0);
        for (i, &v) in x.iter().enumerate() {
            let p = 2.0 * PI * freq * i as f64 / rate;
            re += v as f64 * p.cos();
            im += v as f64 * p.sin();
        }
        (re * re + im * im).sqrt() * 2.0 / x.len() as f64
    }

    #[test]
    fn decimator_matches_full_convolution() {
        let taps = lowpass(0.1, 31);
        let input: Vec<f32> = (0..500).map(|i| ((i * 37) % 11) as f32 - 5.0).collect();
        let mut d = FirDecimator::new(&taps, 3);
        let mut out = Vec::new();
        // Uneven pieces must give the same result as one call.
        for piece in input.chunks(7) {
            d.process(piece, &mut out);
        }
        let mut whole = Vec::new();
        FirDecimator::new(&taps, 3).process(&input, &mut whole);
        assert_eq!(out.len(), whole.len());
        assert!(out.iter().zip(&whole).all(|(a, b)| (a - b).abs() < 1e-4));
        // First full window starts after the zero history: output k uses input[3k - 30 ..= 3k].
        let k = 20;
        let direct: f32 = (0..31).map(|j| taps[j] * input[3 * k + j - 30]).sum();
        assert!((whole[k] - direct).abs() < 1e-4);
    }

    #[test]
    fn halfband_has_zero_taps() {
        let taps = lowpass(0.25, 23);
        assert_eq!(taps.iter().filter(|t| t.abs() < 1e-9).count(), 10);
    }

    #[test]
    fn fm_tone_comes_out_at_its_pitch() {
        // NFM carrier 30 kHz from the centre, 1 kHz tone at 2.5 kHz deviation, 2.4 MS/s.
        let (rate, offset, audio_rate) = (2.4e6, 30e3, 48e3);
        let n = 2_400_000;
        let mut phase = 0.0f64;
        let input: Vec<Complex<f32>> = (0..n)
            .map(|i| {
                let t = i as f64 / rate;
                phase += 2.0 * PI * (offset + 2.5e3 * (2.0 * PI * 1e3 * t).sin()) / rate;
                Complex::from_polar(0.5, phase as f32)
            })
            .collect();
        let config = ChainConfig { mode: Mode::Nfm, bandwidth: 12.5e3, input_rate: rate, audio_rate };
        let mut chain = Chain::new(config);
        let mut out = Vec::new();
        for block in input.chunks(48_000) {
            chain.process(block, offset, 1.0, None, &mut out);
        }
        assert!((out.len() as f64 - audio_rate).abs() < 200.0, "{} samples", out.len());
        let tail = &out[out.len() / 2..];
        let wanted = bin_power(tail, 1e3, audio_rate);
        let other = bin_power(tail, 1.7e3, audio_rate);
        assert!(wanted > 0.1 && wanted > 20.0 * other, "1 kHz {wanted:.3}, 1.7 kHz {other:.4}");
    }

    #[test]
    fn usb_rejects_the_other_sideband() {
        // Dial 100 kHz from centre; a tone 1 kHz above it (USB) and one 1.5 kHz below (LSB).
        let (rate, dial, audio_rate) = (2.4e6, 100e3, 48e3);
        let n = 1_200_000;
        let upper = tone(dial + 1e3, rate, n, 0.1);
        let lower = tone(dial - 1.5e3, rate, n, 0.1);
        let input: Vec<Complex<f32>> = upper.iter().zip(&lower).map(|(a, b)| a + b).collect();
        let mut chain = Chain::new(ChainConfig { mode: Mode::Usb, bandwidth: 2.4e3, input_rate: rate, audio_rate });
        let mut out = Vec::new();
        for block in input.chunks(24_000) {
            chain.process(block, dial, 1.0, None, &mut out);
        }
        let tail = &out[out.len() / 2..];
        let wanted = bin_power(tail, 1e3, audio_rate);
        let image = bin_power(tail, 1.5e3, audio_rate);
        assert!(wanted > 30.0 * image, "USB {wanted:.3}, LSB leak {image:.5}");
    }

    #[test]
    fn cw_carrier_gives_the_tone() {
        let (rate, carrier, audio_rate) = (2.4e6, -200e3, 48e3);
        let input = tone(carrier, rate, 1_200_000, 0.05);
        let mut chain = Chain::new(ChainConfig { mode: Mode::Cw, bandwidth: 500.0, input_rate: rate, audio_rate });
        let mut out = Vec::new();
        for block in input.chunks(24_000) {
            chain.process(block, carrier, 1.0, None, &mut out);
        }
        let tail = &out[out.len() / 2..];
        assert!(bin_power(tail, CW_TONE, audio_rate) > 0.2);
    }

    #[test]
    fn squelch_silences_noise() {
        let (rate, audio_rate) = (2.4e6, 48e3);
        let input = vec![Complex::new(1e-4, 0.0); 240_000];
        let mut chain = Chain::new(ChainConfig { mode: Mode::Am, bandwidth: 9e3, input_rate: rate, audio_rate });
        let mut out = Vec::new();
        chain.process(&input, 50e3, 1.0, Some(-40.0), &mut out);
        assert!(out.iter().all(|&v| v == 0.0));
        assert!(chain.level_db < -40.0);
    }

    /// Speed check: `cargo test --release --lib chain_speed -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn chain_speed() {
        let rate = 12e6;
        let input = tone(1e6, rate, 1_200_000, 0.3);
        for mode in Mode::ALL {
            let mut chain = Chain::new(ChainConfig { mode, bandwidth: mode.default_bandwidth(), input_rate: rate, audio_rate: 48e3 });
            let mut out = Vec::new();
            let t = std::time::Instant::now();
            for _ in 0..5 {
                for block in input.chunks(240 * 1024) {
                    chain.process(block, 1e6, 1.0, None, &mut out);
                }
            }
            let secs = t.elapsed().as_secs_f64();
            println!("{}: {:.2}× real time at 12 MS/s", mode.name(), 0.5 / secs);
        }
        let t = std::time::Instant::now();
        let mut m = Mixer::new(1e5, rate);
        let mut x = input.clone();
        m.mix(&mut x);
        println!("mixer: {:.1} ms per 0.1 s", t.elapsed().as_secs_f64() * 1e3);
        let t = std::time::Instant::now();
        let mut hb = halfband::<Complex<f32>>();
        let mut y = Vec::new();
        hb.process(&x, &mut y);
        println!("first half-band: {:.1} ms per 0.1 s", t.elapsed().as_secs_f64() * 1e3);
    }

    #[test]
    fn wfm_chain_rates() {
        let chain = Chain::new(ChainConfig { mode: Mode::Wfm, bandwidth: 180e3, input_rate: 12e6, audio_rate: 48e3 });
        assert_eq!(chain.channel_rate, 750e3);
        assert_eq!(chain.audio_in_rate, 93_750.0);
    }
}
