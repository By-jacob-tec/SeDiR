//! RTTY to text: two-tone FSK carrying 5-bit Baudot (ITA2, US figures) in start-stop
//! frames. The channel is brought down to ~2 kHz; a spectrum averaged over half-second
//! windows finds the tone pair (shift 170, 200, 425 or 850 Hz) and follows its centre.
//! Each tone is correlated over one bit; the normalised difference of their energies
//! drives start-stop receivers for 45.45, 50 and 75 baud, each in both polarities. The
//! receiver whose stop bits keep turning up where expected is the one shown.

use std::sync::Arc;

use num_complex::Complex;
use rustfft::{Fft, FftPlanner};

use crate::demod::{FirDecimator, TextDecoder, lowpass, taps_for};

/// Rate the decoder works at (about; an integer fraction of the channel rate).
const WORK_RATE: f64 = 2000.0;
pub const SHIFTS: [f64; 4] = [170.0, 200.0, 425.0, 850.0];
pub const BAUDS: [f64; 3] = [45.45, 50.0, 75.0];
/// Spectrum for finding the tones: ~2 Hz bins over ~0.5 s.
const FFT_SIZE: usize = 1024;
/// Samples between spectrum updates (~0.13 s).
const AFC_EVERY: usize = 256;
const MAX_TEXT: usize = 400;
/// Weaker tone over the median of the band (power) to call it a signal, and to keep it.
const SNR_ON: f32 = 6.0;
const SNR_OFF: f32 = 3.0;
/// Share of good stop bits (recent average) a receiver needs before its text is shown.
const MIN_SCORE: f32 = 0.8;

/// Baudot (ITA2) letters and US figures; '\0' for shifts and unused codes.
const LTRS: [char; 32] = [
    '\0', 'E', '\n', 'A', ' ', 'S', 'I', 'U', '\r', 'D', 'R', 'J', 'N', 'F', 'C', 'K', 'T', 'Z', 'L', 'W', 'H', 'Y', 'P',
    'Q', 'O', 'B', 'G', '\0', 'M', 'X', 'V', '\0',
];
const FIGS: [char; 32] = [
    '\0', '3', '\n', '-', ' ', '\0', '8', '7', '\r', '$', '4', '\'', ',', '!', ':', '(', '5', '"', ')', '2', '#', '6',
    '0', '1', '9', '?', '&', '\0', '.', '/', ';', '\0',
];
const CODE_FIGS: u8 = 27;
const CODE_LTRS: u8 = 31;

/// Moving correlation of the two tones over one bit of a given rate.
struct ToneFilter {
    mark: Vec<Complex<f64>>,
    space: Vec<Complex<f64>>,
    pos: usize,
    mark_sum: Complex<f64>,
    space_sum: Complex<f64>,
    count: u32,
}

impl ToneFilter {
    fn new(len: usize) -> Self {
        let len = len.max(2);
        Self {
            mark: vec![Complex::default(); len],
            space: vec![Complex::default(); len],
            pos: 0,
            mark_sum: Complex::default(),
            space_sum: Complex::default(),
            count: 0,
        }
    }

    /// Mark minus space energy, over their sum (−1 … 1).
    fn push(&mut self, m: Complex<f64>, s: Complex<f64>) -> f32 {
        self.mark_sum += m - self.mark[self.pos];
        self.space_sum += s - self.space[self.pos];
        self.mark[self.pos] = m;
        self.space[self.pos] = s;
        self.pos = (self.pos + 1) % self.mark.len();
        self.count += 1;
        if self.count == 50_000 {
            // Rounding builds up in the running sums.
            self.count = 0;
            self.mark_sum = self.mark.iter().sum();
            self.space_sum = self.space.iter().sum();
        }
        let (me, se) = (self.mark_sum.norm_sqr(), self.space_sum.norm_sqr());
        ((me - se) / (me + se + 1e-30)) as f32
    }
}

#[derive(Clone, Copy, PartialEq)]
enum State {
    Idle,
    Start,
    Data,
    Stop,
}

/// Start-stop receiver for one rate and polarity.
struct Uart {
    baud: usize,
    inverted: bool,
    spb: f64,
    state: State,
    t: f64,
    bits: u8,
    count: u32,
    saw_mark: bool,
    figs: bool,
    text: String,
    /// Recent share of frames with a good stop bit.
    score: f32,
}

impl Uart {
    fn new(baud: usize, inverted: bool, rate: f64) -> Self {
        Self {
            baud,
            inverted,
            spb: rate / BAUDS[baud],
            state: State::Idle,
            t: 0.0,
            bits: 0,
            count: 0,
            saw_mark: false,
            figs: false,
            text: String::new(),
            score: 0.5,
        }
    }

    /// One sample of the tone difference; `emit`: whether a signal is present. The
    /// correlation runs over one bit, so it crosses zero half a bit after the true edge
    /// and is best half a bit after each bit centre; times below are from that crossing.
    fn push(&mut self, d: f32, emit: bool) {
        let mark = (d > 0.0) != self.inverted;
        match self.state {
            State::Idle => {
                if mark {
                    self.saw_mark = true;
                } else if self.saw_mark {
                    self.state = State::Start;
                    self.t = 0.0;
                }
            }
            State::Start => {
                self.t += 1.0;
                if mark {
                    // A noise dip, not the start edge: wait for the real one, so the
                    // frame is not timed from too early.
                    self.state = State::Idle;
                } else if self.t >= 0.5 * self.spb {
                    self.state = State::Data;
                    self.bits = 0;
                    self.count = 0;
                }
            }
            State::Data => {
                self.t += 1.0;
                if self.t >= (1.5 + self.count as f64) * self.spb {
                    if mark {
                        self.bits |= 1 << self.count;
                    }
                    self.count += 1;
                    if self.count == 5 {
                        self.state = State::Stop;
                    }
                }
            }
            State::Stop => {
                self.t += 1.0;
                if self.t >= 6.5 * self.spb {
                    self.state = State::Idle;
                    self.saw_mark = mark;
                    if emit {
                        self.score = self.score * 0.9 + if mark { 0.1 } else { 0.0 };
                    }
                    if mark && emit {
                        self.character(self.bits);
                    }
                }
            }
        }
    }

    fn character(&mut self, code: u8) {
        match code {
            CODE_FIGS => self.figs = true,
            CODE_LTRS => self.figs = false,
            _ => {
                let c = if self.figs { FIGS[code as usize] } else { LTRS[code as usize] };
                match c {
                    '\0' | '\r' => {}
                    ' ' | '\n' => {
                        // Unshift on space, as most senders expect.
                        self.figs = false;
                        if !self.text.ends_with(' ') && !self.text.is_empty() {
                            self.text.push(' ');
                        }
                    }
                    c => self.text.push(c),
                }
                if self.text.len() > MAX_TEXT + 100 {
                    let cut = self.text.char_indices().nth(self.text.chars().count() - MAX_TEXT).map_or(0, |(i, _)| i);
                    self.text.drain(..cut);
                }
            }
        }
    }
}

pub struct RttyDecoder {
    decimator: FirDecimator<Complex<f32>>,
    rate: f64,
    scratch: Vec<Complex<f32>>,
    /// Last FFT_SIZE samples for the spectrum.
    ring: Vec<Complex<f32>>,
    ring_pos: usize,
    since_afc: usize,
    fft: Arc<dyn Fft<f32>>,
    window: Vec<f32>,
    spectrum: Vec<f32>,
    /// Tones are searched within ± this of the channel centre (Hz).
    limit_hz: f64,
    /// Centre of the tone pair and their spacing (Hz).
    center_hz: f64,
    shift_hz: f64,
    /// Spectrum updates in a row in which another tone pair was clearly stronger.
    pending: u32,
    mark_phase: f64,
    space_phase: f64,
    filters: Vec<ToneFilter>,
    uarts: Vec<Uart>,
    best: Option<usize>,
    pub signal: bool,
}

impl RttyDecoder {
    /// `rate`: channel rate (complex baseband centred on the listening frequency);
    /// `half_band_hz`: half the channel filter's width.
    pub fn new(rate: f64, half_band_hz: f64) -> Self {
        let factor = (rate / WORK_RATE).round().max(1.0) as usize;
        let work = rate / factor as f64;
        let cut = (half_band_hz + 50.0).min(0.45 * work);
        let decimator = FirDecimator::new(&lowpass(cut / rate, taps_for(rate, 150.0, 401)), factor);
        let window = (0..FFT_SIZE).map(|i| (0.5 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / FFT_SIZE as f64).cos()) as f32).collect();
        let filters = BAUDS.iter().map(|b| ToneFilter::new((work / b).round() as usize)).collect();
        let uarts = (0..BAUDS.len()).flat_map(|b| [Uart::new(b, false, work), Uart::new(b, true, work)]).collect();
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
            center_hz: 0.0,
            shift_hz: SHIFTS[0],
            pending: 0,
            mark_phase: 0.0,
            space_phase: 0.0,
            filters,
            uarts,
            best: None,
            signal: false,
        }
    }

    /// Tone spacing (Hz), rate (baud) and whether the higher tone is space ("reversed").
    pub fn format(&self) -> Option<(f64, f64, bool)> {
        self.best.map(|i| (self.shift_hz, BAUDS[self.uarts[i].baud], self.uarts[i].inverted))
    }

    /// Centre of the tone pair from the listening frequency (Hz).
    pub fn offset_hz(&self) -> f64 {
        self.center_hz
    }

    fn afc(&mut self) {
        let mut buf: Vec<Complex<f32>> =
            (0..FFT_SIZE).map(|i| self.ring[(self.ring_pos + i) % FFT_SIZE] * self.window[i]).collect();
        self.fft.process(&mut buf);
        for (s, z) in self.spectrum.iter_mut().zip(&buf) {
            *s = *s * 0.7 + z.norm_sqr() * 0.3;
        }
        let bin_hz = self.rate / FFT_SIZE as f64;
        let bin = |f: f64| -> usize { ((f / bin_hz).round() as isize).rem_euclid(FFT_SIZE as isize) as usize };
        let peak = |f: f64| -> f32 { (-1..=1).map(|d| self.spectrum[(bin(f) as isize + d).rem_euclid(FFT_SIZE as isize) as usize]).fold(0.0, f32::max) };
        let mut band: Vec<f32> = (0..FFT_SIZE)
            .filter(|&k| {
                let f = if k < FFT_SIZE / 2 { k as f64 } else { k as f64 - FFT_SIZE as f64 } * bin_hz;
                f.abs() <= self.limit_hz
            })
            .map(|k| self.spectrum[k])
            .collect();
        if band.is_empty() {
            return;
        }
        let mid = band.len() / 2;
        let noise = *band.select_nth_unstable_by(mid, f32::total_cmp).1 + 1e-30;
        let mut best = (0.0f32, 0.0, SHIFTS[0]);
        for &shift in &SHIFTS {
            let reach = self.limit_hz - shift / 2.0;
            if reach < 0.0 {
                continue;
            }
            let steps = (reach / bin_hz) as i64;
            for k in -steps..=steps {
                let c = k as f64 * bin_hz;
                let score = peak(c - shift / 2.0).min(peak(c + shift / 2.0));
                if score > best.0 {
                    best = (score, c, shift);
                }
            }
        }
        if !self.signal {
            if best.0 / noise > SNR_ON {
                (self.center_hz, self.shift_hz, self.signal, self.pending) = (best.1, best.2, true, 0);
            }
            return;
        }
        // Following: the current pair near where it was, unless another one has been
        // clearly better several times running (a new signal, or a different shift).
        let shift = self.shift_hz;
        let near = (-10..=10)
            .map(|k| self.center_hz + k as f64 * bin_hz)
            .map(|c| (peak(c - shift / 2.0).min(peak(c + shift / 2.0)), c))
            .fold((0.0f32, self.center_hz), |a, b| if b.0 > a.0 { b } else { a });
        let elsewhere = best.2 != shift || (best.1 - self.center_hz).abs() > 20.0;
        if elsewhere && best.0 > 2.0 * near.0 {
            self.pending += 1;
            if self.pending >= 4 {
                (self.center_hz, self.shift_hz, self.pending) = (best.1, best.2, 0);
            }
        } else {
            self.pending = 0;
            self.center_hz += 0.2 * (near.1 - self.center_hz);
        }
        self.signal = near.0 / noise > SNR_OFF;
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
        let z = Complex::new(z.re as f64, z.im as f64);
        let m = z * Complex::from_polar(1.0, -self.mark_phase);
        let s = z * Complex::from_polar(1.0, -self.space_phase);
        self.mark_phase = (self.mark_phase + tau * (self.center_hz + self.shift_hz / 2.0) / self.rate) % tau;
        self.space_phase = (self.space_phase + tau * (self.center_hz - self.shift_hz / 2.0) / self.rate) % tau;
        let emit = self.signal;
        let mut d = [0.0f32; BAUDS.len()];
        for (d, f) in d.iter_mut().zip(&mut self.filters) {
            *d = f.push(m, s);
        }
        for u in &mut self.uarts {
            u.push(d[u.baud], emit);
        }
        let (i, top) = self.uarts.iter().enumerate().map(|(i, u)| (i, u.score)).fold((0, f32::MIN), |a, b| if b.1 > a.1 { b } else { a });
        if top >= MIN_SCORE {
            self.best = Some(i);
        }
    }
}

impl TextDecoder for RttyDecoder {
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
        self.best.map_or("", |i| self.uarts[i].text.as_str())
    }

    fn detail(&self) -> String {
        match self.format() {
            Some((shift, baud, reversed)) => {
                format!("{shift:.0} Hz · {baud} Bd{} · {:+.0} Hz", if reversed { " · REV" } else { "" }, self.center_hz)
            }
            None if self.signal => format!("{:.0} Hz · {:+.0} Hz", self.shift_hz, self.center_hz),
            None => String::new(),
        }
    }

    fn signal(&self) -> bool {
        self.signal
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// Baudot codes for a text (US figures), with shifts as needed; LTRS first.
    pub fn encode(text: &str) -> Vec<u8> {
        let mut out = vec![CODE_LTRS];
        let mut figs = false;
        for c in text.chars() {
            if let Some(k) = LTRS.iter().position(|&l| l == c && c != '\0') {
                if figs && c != ' ' {
                    out.push(CODE_LTRS);
                    figs = false;
                }
                out.push(k as u8);
                if c == ' ' {
                    figs = false;
                }
            } else if let Some(k) = FIGS.iter().position(|&l| l == c && c != '\0') {
                if !figs {
                    out.push(CODE_FIGS);
                    figs = true;
                }
                out.push(k as u8);
            }
        }
        out
    }

    /// Continuous-phase FSK at `rate`: idle mark, then the frames (1.5 stop bits).
    /// Mark is the higher tone unless `reversed`.
    pub fn fsk(codes: &[u8], rate: f64, center: f64, shift: f64, baud: f64, reversed: bool, amplitude: f32) -> Vec<Complex<f32>> {
        let mut bits: Vec<bool> = vec![true; (baud * 1.5) as usize];
        for &c in codes {
            bits.push(false);
            bits.extend((0..5).map(|k| c >> k & 1 == 1));
            bits.extend([true, true]);
        }
        bits.extend(vec![true; 20]);
        // 1.5 stop bits: the second stop bit of each frame lasts half a bit.
        let mut out = Vec::new();
        let mut phase = 0.0f64;
        let mut t = 0.0f64;
        let spb = rate / baud;
        let mut i = 0;
        let frame_start = (baud * 1.5) as usize;
        while i < bits.len() {
            let in_frame = i >= frame_start && i < frame_start + codes.len() * 8;
            let len = if in_frame && (i - frame_start) % 8 == 7 { spb * 0.5 } else { spb };
            let mark = bits[i] != reversed;
            let f = center + if mark { shift / 2.0 } else { -shift / 2.0 };
            while t < len {
                out.push(Complex::from_polar(amplitude, phase as f32));
                phase = (phase + 2.0 * std::f64::consts::PI * f / rate) % (2.0 * std::f64::consts::PI);
                t += 1.0;
            }
            t -= len;
            i += 1;
        }
        out
    }

    pub fn add_noise(x: &mut [Complex<f32>], sigma: f32, seed: u64) {
        let mut s = seed;
        let mut next = || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            (s >> 11) as f64 / (1u64 << 53) as f64
        };
        for z in x {
            let (u, v) = (next().max(1e-12), next());
            let r = (-2.0 * u.ln()).sqrt() as f32 * sigma;
            let a = (2.0 * std::f64::consts::PI * v) as f32;
            *z += Complex::new(r * a.cos(), r * a.sin());
        }
    }

    fn decode(signal: &[Complex<f32>], rate: f64) -> RttyDecoder {
        let mut d = RttyDecoder::new(rate, 300.0);
        for chunk in signal.chunks(1600) {
            d.push(chunk);
        }
        d
    }

    const MESSAGE: &str = "CQ CQ CQ DE TA4SO TA4SO PSE K RST 599 73";

    #[test]
    fn decodes_45_baud_170_hz() {
        let rate = 16_000.0;
        let mut x = fsk(&encode(MESSAGE), rate, 37.0, 170.0, 45.45, false, 0.1);
        // Eb/N0 13 dB (about 2 % of the characters are wrong at 12 dB, 10 % at 10 dB).
        add_noise(&mut x, 0.1 * (rate as f32 / 2.0 / 45.45 / 20.0).sqrt(), 7);
        let d = decode(&x, rate);
        println!("{:?} · {}", d.text(), d.detail());
        assert!(d.text().contains("DE TA4SO TA4SO PSE K RST 599 73"), "{:?}", d.text());
        let (shift, baud, reversed) = d.format().unwrap();
        assert_eq!((shift, baud, reversed), (170.0, 45.45, false));
        assert!((d.offset_hz() - 37.0).abs() < 4.0, "{}", d.offset_hz());
    }

    /// Character errors against Eb/N0: `cargo test --lib rtty_sweep -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn rtty_sweep() {
        let rate = 16_000.0;
        let long = MESSAGE.repeat(4);
        for db in [6.0f32, 8.0, 10.0, 12.0, 14.0, 20.0] {
            let mut x = fsk(&encode(&long), rate, 37.0, 170.0, 45.45, false, 0.1);
            let ratio = 10f32.powf(db / 10.0);
            add_noise(&mut x, 0.1 * (rate as f32 / 2.0 / 45.45 / ratio).sqrt(), 7);
            let d = decode(&x, rate);
            let wrong = d.text().chars().zip(long.chars().skip(long.len() - d.text().len())).filter(|(a, b)| a != b).count();
            println!("{db} dB: {} chars, {wrong} differ · {:?}", d.text().len(), &d.text()[d.text().len().saturating_sub(60)..]);
        }
    }

    #[test]
    fn finds_reversed_75_baud_850_hz() {
        let rate = 16_000.0;
        let mut x = fsk(&encode(MESSAGE), rate, -20.0, 850.0, 75.0, true, 0.1);
        add_noise(&mut x, 0.002, 3);
        let mut d = RttyDecoder::new(rate, 600.0);
        for chunk in x.chunks(1600) {
            d.push(chunk);
        }
        println!("{:?} · {}", d.text(), d.detail());
        assert!(d.text().contains("TA4SO PSE K RST 599 73"), "{:?}", d.text());
        assert_eq!(d.format().unwrap(), (850.0, 75.0, true));
    }

    #[test]
    fn noise_alone_gives_nothing() {
        let rate = 16_000.0;
        let mut x = vec![Complex::default(); 16_000 * 10];
        add_noise(&mut x, 0.05, 11);
        let d = decode(&x, rate);
        assert!(d.text().len() < 5, "{:?}", d.text());
    }
}
