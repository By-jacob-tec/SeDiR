//! Feeds the FT8 or FT4 decoder: brings the 3 kHz above a dial frequency down to the
//! decoder's baseband, keeps it with UTC time stamps from the computer's clock (which
//! the user keeps right, e.g. with GPS or NTP), and decodes each slot.

use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, RecvTimeoutError};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use num_complex::Complex;

use super::Protocol;
use super::decode;
use super::message::CallBook;
use crate::demod::{FirDecimator, Mixer, halfband, lowpass, taps_for};
use crate::source::Block;

/// The baseband is centred this far above the dial frequency: FT8 uses about
/// 100–3100 Hz of audio, i.e. ±1500 Hz around it.
pub const CENTER_ABOVE_DIAL_HZ: f64 = 1600.0;
/// Audio range searched (Hz above the dial).
const AUDIO_RANGE_HZ: (f64, f64) = (100.0, 3100.0);
/// Audio kept (seconds).
const KEEP_S: f64 = 32.0;
/// Messages kept for display.
const KEEP_MESSAGES: usize = 500;
/// Dial changes up to this size are followed without starting over (LNB correction).
const MAX_FOLLOW_HZ: f64 = 2_000.0;
/// A difference this large between the counted and the clock time starts over.
const MAX_TIME_JUMP_S: f64 = 2.0;
/// Weight of each block's clock reading in the buffer's start time.
const TIME_FOLLOW: f64 = 0.01;
/// Limits of the learned delay of the signals (satellite path, receiver buffering, and
/// the computer clock's own error: on QO-100 most senders showed up 1 s early, likely
/// a clock set from GPS sentences without the pulse).
const DELAY_RANGE_S: (f64, f64) = (-2.0, 2.0);
/// Decodes the delay is the median of (senders' own clock errors of ±1 s average out
/// only over many), and how many are needed before it is used.
const DELAY_FROM: usize = 40;
const DELAY_AFTER: usize = 5;

/// One decoded message.
#[derive(Clone, Debug, PartialEq)]
pub struct Ft8Message {
    /// "FT8" or "FT4".
    pub mode: &'static str,
    /// Start of its slot (Unix seconds, UTC).
    pub slot: u64,
    pub snr_db: f32,
    pub dt_s: f32,
    /// Audio frequency of tone 0 above the dial (Hz).
    pub audio_hz: f64,
    pub text: String,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Ft8Status {
    /// Slot of the last decode and how many messages it gave.
    pub last_slot: Option<u64>,
    pub last_count: usize,
    /// Seconds of the slot that arrived (gaps in the stream shorten it).
    pub coverage_s: f64,
    /// Learned delay of the transmissions behind the computer's clock (seconds): the
    /// satellite path (~0.25 s on QO-100) plus the receiver's buffering. Slots are taken
    /// this much later, so whole transmissions fall inside them, and DT is shown
    /// relative to it.
    pub delay_s: f64,
}

/// Shared between the UI and the worker of one protocol.
#[derive(Clone)]
pub struct Ft8 {
    pub protocol: &'static Protocol,
    /// Dial frequency in the receiver's own frequencies (Hz); None switches decoding off.
    dial: Arc<Mutex<Option<f64>>>,
    messages: Arc<Mutex<Vec<Ft8Message>>>,
    status: Arc<Mutex<Ft8Status>>,
    /// Start times of recent decodes against the computer's clock (DT plus the delay
    /// in use then), for learning the delay.
    starts: Arc<Mutex<VecDeque<f64>>>,
}

fn now_utc() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map_or(0.0, |d| d.as_secs_f64())
}

impl Ft8 {
    pub fn new(protocol: &'static Protocol) -> Self {
        Self {
            protocol,
            dial: Arc::default(),
            messages: Arc::default(),
            status: Arc::default(),
            starts: Arc::default(),
        }
    }

    pub fn set_dial(&self, dial: Option<f64>) {
        *self.dial.lock().unwrap() = dial;
    }

    pub fn messages(&self) -> Vec<Ft8Message> {
        self.messages.lock().unwrap().clone()
    }

    pub fn clear(&self) {
        self.messages.lock().unwrap().clear();
    }

    pub fn status(&self) -> Ft8Status {
        *self.status.lock().unwrap()
    }

    /// Runs until the source stops.
    pub fn spawn(&self, blocks: Receiver<Block>) {
        let me = self.clone();
        thread::Builder::new().name(self.protocol.name.to_lowercase()).spawn(move || run(blocks, me)).expect("ft8/ft4 thread");
    }
}

/// The stream brought down to `out_rate` around one centre frequency.
struct Downconverter {
    center: f64,
    input_rate: f64,
    out_rate: f64,
    mixer: Mixer,
    halfbands: Vec<FirDecimator<Complex<f32>>>,
    filter: FirDecimator<Complex<f32>>,
    rate: f64,
    /// Resampler position and previous sample.
    pos: f64,
    prev: Complex<f32>,
}

impl Downconverter {
    fn new(center: f64, block: &Block, out_rate: f64) -> Self {
        let mut rate = block.sample_rate;
        let mut halfbands = Vec::new();
        while rate / 2.0 >= 2.0 * out_rate {
            halfbands.push(halfband());
            rate /= 2.0;
        }
        let filter = FirDecimator::new(&lowpass(1600.0 / rate, taps_for(rate, 300.0, 801)), 1);
        Self {
            center,
            input_rate: block.sample_rate,
            out_rate,
            mixer: Mixer::new(block.center_hz - center, block.sample_rate),
            halfbands,
            filter,
            rate,
            pos: 0.0,
            prev: Complex::default(),
        }
    }

    /// Follows a small change of the centre (the LNB correction moving it) without
    /// restarting: the mixer keeps its phase, the samples stay continuous.
    fn retarget(&mut self, center: f64) {
        self.center = center;
    }

    fn process(&mut self, block: &Block, out: &mut Vec<Complex<f32>>) {
        self.mixer.set(block.center_hz - self.center);
        let mut a = block.samples.to_vec();
        self.mixer.mix(&mut a);
        for hb in &mut self.halfbands {
            let mut b = Vec::with_capacity(a.len() / 2 + 1);
            hb.process(&a, &mut b);
            a = b;
        }
        let mut b = Vec::with_capacity(a.len());
        self.filter.process(&a, &mut b);
        // Linear interpolation to exactly the output rate (the band is filtered well
        // inside it).
        let step = self.rate / self.out_rate;
        for &x in &b {
            while self.pos < 1.0 {
                out.push(self.prev + (x - self.prev) * self.pos as f32);
                self.pos += step;
            }
            self.pos -= 1.0;
            self.prev = x;
        }
    }
}

fn run(blocks: Receiver<Block>, me: Ft8) {
    let p = me.protocol;
    let rate = p.rate;
    // Length of a transmission: a slot holding less of it is not decoded.
    let duration = p.symbols as f64 * p.sps as f64 / rate;
    let book = Arc::new(Mutex::new(CallBook::default()));
    let mut down: Option<Downconverter> = None;
    let mut buffer: VecDeque<Complex<f32>> = VecDeque::new();
    // UTC time of buffer[0].
    let mut buffer_start = 0.0f64;
    let mut decoded_slot: Option<f64> = None;
    let mut chunk = Vec::new();
    loop {
        let block = match blocks.recv_timeout(Duration::from_millis(500)) {
            Ok(b) => b,
            Err(RecvTimeoutError::Timeout) => continue,
            Err(RecvTimeoutError::Disconnected) => return,
        };
        let arrived = now_utc();
        let Some(dial) = *me.dial.lock().unwrap() else {
            down = None;
            buffer.clear();
            continue;
        };
        let center = dial + CENTER_ABOVE_DIAL_HZ;
        if (center - block.center_hz).abs() > block.sample_rate / 2.0 - 2e3 {
            down = None;
            buffer.clear();
            continue;
        }
        // The beacon lock moves the dial by some Hz every few seconds: that is followed
        // in place (it also takes the LNB's drift out of the tones during a slot). Only
        // a real retune or a new sample rate starts over.
        match down.as_mut() {
            Some(d) if d.input_rate == block.sample_rate && (d.center - center).abs() < MAX_FOLLOW_HZ => d.retarget(center),
            _ => {
                down = Some(Downconverter::new(center, &block, rate));
                buffer.clear();
            }
        }
        let d = down.as_mut().expect("downconverter");
        chunk.clear();
        d.process(&block, &mut chunk);

        // Time stamps: the newest sample arrived about now. The buffer's start time
        // follows that estimate slowly, which averages the arrival jitter and lets the
        // SDR's and the computer's clocks differ (the stream ran at 99.8 % of nominal);
        // only a jump of seconds (a stall, a clock change) starts over.
        buffer.extend(chunk.iter().copied());
        let estimate = arrived - buffer.len() as f64 / rate;
        if buffer.len() == chunk.len() || (estimate - buffer_start).abs() > MAX_TIME_JUMP_S {
            buffer.clear();
            buffer.extend(chunk.iter().copied());
            buffer_start = estimate;
        } else {
            buffer_start += (estimate - buffer_start) * TIME_FOLLOW;
        }
        let excess = buffer.len().saturating_sub((KEEP_S * rate) as usize);
        if excess > 0 {
            buffer.drain(..excess);
            buffer_start += excess as f64 / rate;
        }

        // Decode a slot once DECODE_AT_S of it has arrived, counting from where the
        // transmissions actually begin (the learned delay later than the clock's slot).
        let delay = me.status.lock().unwrap().delay_s;
        let latest = buffer_start + buffer.len() as f64 / rate;
        let slot = ((latest - delay - p.decode_at_s) / p.slot_s).floor() * p.slot_s;
        // A learned delay that grows can make the same slot (or an earlier one) come round
        // again: only later slots are taken.
        if slot <= 0.0 || decoded_slot.is_some_and(|d| slot <= d) || latest < slot + delay + p.decode_at_s {
            continue;
        }
        decoded_slot = Some(slot);
        // From early_s before the slot, for senders whose clocks run ahead.
        let first = ((slot + delay - p.early_s - buffer_start) * rate).round() as isize;
        let count = ((p.early_s + p.decode_at_s) * rate) as usize;
        // Slot samples; missing ones (before the buffer began) are zero.
        let samples: Vec<Complex<f32>> = (0..count as isize)
            .map(|i| {
                let k = first + i;
                if k >= 0 && (k as usize) < buffer.len() { buffer[k as usize] } else { Complex::default() }
            })
            .collect();
        // Seconds of the slot itself (from its start) that arrived.
        let missing = ((-first).max(0) as f64 / rate - p.early_s).max(0.0);
        let coverage = p.decode_at_s - missing;
        if coverage < p.start_s + duration {
            // Not enough of the slot to hold a whole transmission.
            continue;
        }
        let (me2, book2) = (me.clone(), book.clone());
        // Whole seconds, as WSJT-X shows them (an FT4 slot at :07.5 is :07).
        let slot_secs = (slot + 1e-6).floor() as u64;
        thread::spawn(move || {
            save_slot(p, &samples, slot_secs);
            let lo = AUDIO_RANGE_HZ.0 - CENTER_ABOVE_DIAL_HZ;
            let hi = AUDIO_RANGE_HZ.1 - CENTER_ABOVE_DIAL_HZ;
            let found = decode::decode_slot(p, &samples, lo, hi, &mut book2.lock().unwrap());
            let mut messages = me2.messages.lock().unwrap();
            for d in &found {
                messages.push(Ft8Message {
                    mode: p.name,
                    slot: slot_secs,
                    snr_db: d.snr_db,
                    dt_s: d.dt_s,
                    audio_hz: d.freq_hz + CENTER_ABOVE_DIAL_HZ,
                    text: d.text.clone(),
                });
            }
            let excess = messages.len().saturating_sub(KEEP_MESSAGES);
            messages.drain(..excess);
            drop(messages);
            // Learn the delay from where the decoded transmissions began, against the
            // clock: the median over many decodes (one sender's clock error must not
            // move it).
            let mut starts = me2.starts.lock().unwrap();
            starts.extend(found.iter().map(|d| d.dt_s as f64 + delay));
            while starts.len() > DELAY_FROM {
                starts.pop_front();
            }
            let learned = (starts.len() >= DELAY_AFTER).then(|| {
                let mut sorted: Vec<f64> = starts.iter().copied().collect();
                sorted.sort_by(f64::total_cmp);
                sorted[sorted.len() / 2].clamp(DELAY_RANGE_S.0, DELAY_RANGE_S.1)
            });
            drop(starts);
            let mut status = me2.status.lock().unwrap();
            if let Some(d) = learned {
                status.delay_s = d;
            }
            status.last_slot = Some(slot_secs);
            status.last_count = found.len();
            status.coverage_s = coverage;
        });
    }
}

/// Slots kept on disk for diagnosis.
const KEEP_SLOTS: usize = 10;

/// Folder of a protocol's saved slots (Windows: %LOCALAPPDATA%\SeDiR\ft8 or \ft4).
pub fn slot_dir(p: &Protocol) -> Option<std::path::PathBuf> {
    crate::classify::analysis_dir().and_then(|d| d.parent().map(|dir| dir.join(p.name.to_lowercase())))
}

/// Slots with a disturbance kept apart, for longer.
const KEEP_EVENTS: usize = 20;

/// Whether the noise jumps somewhere in the slot: a 40 ms stretch with more than twice
/// (3 dB) the median power, as when the LNB's PLL briefly loses lock.
fn has_burst(samples: &[Complex<f32>]) -> bool {
    let power: Vec<f32> = samples.chunks(256).map(|c| c.iter().map(|z| z.norm_sqr()).sum::<f32>() / c.len() as f32).collect();
    let mut sorted = power.clone();
    sorted.sort_by(f32::total_cmp);
    let median = sorted[sorted.len() / 2];
    median > 0.0 && power.iter().any(|&p| p > median * 2.0)
}

fn write_slot(dir: &std::path::Path, samples: &[Complex<f32>], slot: u64, keep: usize) {
    if std::fs::create_dir_all(dir).is_err() {
        return;
    }
    let bytes: Vec<u8> = samples.iter().flat_map(|z| [z.re.to_le_bytes(), z.im.to_le_bytes()].concat()).collect();
    let _ = std::fs::write(dir.join(format!("{slot}.cf32")), bytes);
    if let Ok(entries) = std::fs::read_dir(dir) {
        let mut files: Vec<_> = entries.filter_map(|e| e.ok()).map(|e| e.path()).filter(|p| p.extension().is_some_and(|x| x == "cf32")).collect();
        files.sort();
        for old in files.iter().take(files.len().saturating_sub(keep)) {
            let _ = std::fs::remove_file(old);
        }
    }
}

/// Writes a slot's baseband (cf32 at the protocol's rate, the dial 1600 Hz below its
/// centre, from `early_s` before the slot) so that missed decodes can be studied; the last
/// `KEEP_SLOTS` are kept, and slots with a noise burst also go to `events`.
fn save_slot(p: &Protocol, samples: &[Complex<f32>], slot: u64) {
    let Some(dir) = slot_dir(p) else { return };
    write_slot(&dir, samples, slot, KEEP_SLOTS);
    if has_burst(samples) {
        write_slot(&dir.join("events"), samples, slot, KEEP_EVENTS);
    }
}

/// Reads a slot saved by `save_slot`.
pub fn load_slot(path: &std::path::Path) -> Option<Vec<Complex<f32>>> {
    let bytes = std::fs::read(path).ok()?;
    Some(bytes.chunks_exact(8).map(|c| Complex::new(f32::from_le_bytes(c[0..4].try_into().unwrap()), f32::from_le_bytes(c[4..8].try_into().unwrap()))).collect())
}

/// Usual FT8 dial frequencies (Hz).
pub const DIALS: &[(f64, &str)] = &[
    (10_489.540e6, "QO-100"),
    (1.840e6, "160 m"),
    (3.573e6, "80 m"),
    (5.357e6, "60 m"),
    (7.074e6, "40 m"),
    (10.136e6, "30 m"),
    (14.074e6, "20 m"),
    (18.100e6, "17 m"),
    (21.074e6, "15 m"),
    (24.915e6, "12 m"),
    (28.074e6, "10 m"),
    (50.313e6, "6 m"),
    (144.174e6, "2 m"),
    (432.174e6, "70 cm"),
];

/// Usual FT4 dial frequencies (Hz), as WSJT-X lists them. On QO-100 FT4 shares FT8's
/// dial: both are found in the same 3 kHz and can be decoded together.
pub const FT4_DIALS: &[(f64, &str)] = &[
    (10_489.540e6, "QO-100"),
    (3.575e6, "80 m"),
    (7.0475e6, "40 m"),
    (10.140e6, "30 m"),
    (14.080e6, "20 m"),
    (18.104e6, "17 m"),
    (21.140e6, "15 m"),
    (24.919e6, "12 m"),
    (28.180e6, "10 m"),
    (50.318e6, "6 m"),
    (144.170e6, "2 m"),
];

/// Dial list of a protocol.
pub fn dials(p: &Protocol) -> &'static [(f64, &'static str)] {
    if p.name == "FT4" { FT4_DIALS } else { DIALS }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ft8::decode::tests::{add_signal, amplitude, noise};

    /// On QO-100: `SEDIR_SOURCE=ip:192.168.1.47 cargo test --lib live_ft8 -- --ignored --nocapture`
    /// (locks to the beacon first, then decodes two slots).
    #[test]
    #[ignore]
    fn live_ft8() {
        use crate::beacon::{self, QO100_BEACON_HZ};
        use crate::device::{DeviceInfo, DeviceKind};
        use crate::radio::Tuning;
        use crate::source::{self, Command, View};
        let Ok(args) = std::env::var("SEDIR_SOURCE") else { return };
        let (lo, rate) = (9_750e6, 2.4e6);
        let device = DeviceInfo { label: args.clone(), args, kind: DeviceKind::LibreSdr };
        let handle = source::spawn(device, Tuning { center_hz: 10_489.75e6 - lo, sample_rate: rate, gain: Some(40.0) }, View::full(rate), || {});
        let (tx, rx) = std::sync::mpsc::sync_channel(32);
        handle.commands.send(Command::Subscribe(9, tx)).unwrap();
        let state = beacon::spawn(rx, QO100_BEACON_HZ, lo, 0.0);
        let mut correction = None;
        for _ in 0..20 {
            thread::sleep(Duration::from_secs(1));
            correction = state.lock().unwrap().correction_hz;
            if correction.is_some() {
                break;
            }
        }
        let correction = correction.expect("beacon lock");
        println!("LNB correction {correction:+.0} Hz");
        let ft8 = Ft8::new(&crate::ft8::FT8);
        let (tx, rx) = std::sync::mpsc::sync_channel(32);
        handle.commands.send(Command::Subscribe(10, tx)).unwrap();
        ft8.spawn(rx);
        // As the window does: the dial follows the beacon correction every second.
        let seconds: u64 = std::env::var("SEDIR_SECONDS").ok().and_then(|v| v.parse().ok()).unwrap_or(40);
        let mut last_slot = None;
        for _ in 0..seconds * 10 {
            let b = *state.lock().unwrap();
            let c = b.correction_at(std::time::Instant::now()).unwrap_or(correction);
            ft8.set_dial(Some(10_489.540e6 - lo - c));
            thread::sleep(Duration::from_millis(100));
            let s = ft8.status();
            if s.last_slot != last_slot {
                last_slot = s.last_slot;
                println!(
                    "slot {:?}: {} messages, {:.1} s arrived, delay {:+.2} s, LNB {c:+.0} Hz drifting {:+.2} Hz/s",
                    s.last_slot.map(|v| v % 86_400),
                    s.last_count,
                    s.coverage_s,
                    s.delay_s,
                    b.rate_hz_per_s
                );
            }
        }
        println!("{:?}", ft8.status());
        for e in handle.events.try_iter() {
            if let crate::source::Event::Stats(s) = e {
                println!("{s:?}");
            }
        }
        for m in ft8.messages() {
            println!("{} {:+3.0} {:+.1} {:4.0}  {}", m.slot % 86_400, m.snr_db, m.dt_s, m.audio_hz, m.text);
        }
    }

    #[test]
    fn decodes_through_the_downconverter() {
        // A 2.4 MS/s band with the dial 300 kHz above its centre; one signal at 1500 Hz audio.
        let rate = 2.4e6;
        let center_rf = 100e6;
        let dial = center_rf + 300e3;
        // Build the FT8 baseband at its rate, then put it up into the wide band.
        let ft8_rate = crate::ft8::FT8.rate;
        let n = crate::ft8::decode::tests::slot_len();
        let mut base = noise(n, 0.05, 3);
        add_signal(&mut base, "CQ TA4SO KM46", 1500.0 - CENTER_ABOVE_DIAL_HZ, 0.5, amplitude(0.0) * 0.05);
        let up = (rate / ft8_rate) as usize;
        let offset = dial + CENTER_ABOVE_DIAL_HZ - center_rf;
        let mut wide = Vec::with_capacity(n * up);
        let mut phase = 0.0f64;
        for z in &base {
            for _ in 0..up {
                wide.push(z * Complex::from_polar(1.0, phase as f32));
                phase += 2.0 * std::f64::consts::PI * offset / rate;
            }
        }
        let mut d = Downconverter::new(dial + CENTER_ABOVE_DIAL_HZ, &Block { samples: Arc::new(Vec::new()), center_hz: center_rf, sample_rate: rate }, ft8_rate);
        let mut out = Vec::new();
        for c in wide.chunks(48_000) {
            d.process(&Block { samples: Arc::new(c.to_vec()), center_hz: center_rf, sample_rate: rate }, &mut out);
        }
        assert!((out.len() as f64 - n as f64).abs() < 50.0, "{} samples", out.len());
        // The filters delay the stream by a few ms; the decoder searches the start anyway.
        let found = decode::decode_slot(&crate::ft8::FT8, &out, -1500.0, 1500.0, &mut CallBook::default());
        println!("{found:?}");
        let d = found.iter().find(|d| d.text == "CQ TA4SO KM46").expect("decoded");
        assert!((d.freq_hz + CENTER_ABOVE_DIAL_HZ - 1500.0).abs() < 4.0);
    }
}
