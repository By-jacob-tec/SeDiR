//! Feeds the FT8 decoder: brings the 3 kHz above an FT8 dial frequency down to the
//! decoder's 6400 S/s baseband, keeps it with UTC time stamps from the computer's clock
//! (which the user keeps right, e.g. with GPS or NTP), and decodes each 15 s slot.

use std::collections::VecDeque;
use std::sync::mpsc::{Receiver, RecvTimeoutError};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use num_complex::Complex;

use super::decode::{self, RATE};
use super::message::CallBook;
use super::{SLOT_SECONDS, START_SECONDS};
use crate::demod::{FirDecimator, Mixer, halfband, lowpass, taps_for};
use crate::source::Block;

/// The baseband is centred this far above the dial frequency: FT8 uses about
/// 100–3100 Hz of audio, i.e. ±1500 Hz around it.
pub const CENTER_ABOVE_DIAL_HZ: f64 = 1600.0;
/// Audio range searched (Hz above the dial).
const AUDIO_RANGE_HZ: (f64, f64) = (100.0, 3100.0);
/// A slot is decoded when this much of it has arrived (transmissions end at 13.14 s).
const DECODE_AT_S: f64 = 14.0;
/// Audio kept (seconds).
const KEEP_S: f64 = 32.0;
/// Messages kept for display.
const KEEP_MESSAGES: usize = 500;

/// One decoded message.
#[derive(Clone, Debug, PartialEq)]
pub struct Ft8Message {
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
}

/// Shared between the UI and the worker.
#[derive(Clone, Default)]
pub struct Ft8 {
    /// Dial frequency in the receiver's own frequencies (Hz); None switches decoding off.
    dial: Arc<Mutex<Option<f64>>>,
    messages: Arc<Mutex<Vec<Ft8Message>>>,
    status: Arc<Mutex<Ft8Status>>,
}

fn now_utc() -> f64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map_or(0.0, |d| d.as_secs_f64())
}

impl Ft8 {
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
        thread::Builder::new().name("ft8".into()).spawn(move || run(blocks, me)).expect("ft8 thread");
    }
}

/// The stream brought down to RATE around one centre frequency.
struct Downconverter {
    center: f64,
    input_rate: f64,
    mixer: Mixer,
    halfbands: Vec<FirDecimator<Complex<f32>>>,
    filter: FirDecimator<Complex<f32>>,
    rate: f64,
    /// Resampler position and previous sample.
    pos: f64,
    prev: Complex<f32>,
}

impl Downconverter {
    fn new(center: f64, block: &Block) -> Self {
        let mut rate = block.sample_rate;
        let mut halfbands = Vec::new();
        while rate / 2.0 >= 2.0 * RATE {
            halfbands.push(halfband());
            rate /= 2.0;
        }
        let filter = FirDecimator::new(&lowpass(1600.0 / rate, taps_for(rate, 300.0, 801)), 1);
        Self {
            center,
            input_rate: block.sample_rate,
            mixer: Mixer::new(block.center_hz - center, block.sample_rate),
            halfbands,
            filter,
            rate,
            pos: 0.0,
            prev: Complex::default(),
        }
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
        // Linear interpolation to exactly RATE (the band is filtered well inside it).
        let step = self.rate / RATE;
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
    let book = Arc::new(Mutex::new(CallBook::default()));
    let mut down: Option<Downconverter> = None;
    let mut buffer: VecDeque<Complex<f32>> = VecDeque::new();
    // UTC time of buffer[0].
    let mut buffer_start = 0.0f64;
    let mut decoded_slot: Option<u64> = None;
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
        if down.as_ref().is_none_or(|d| d.center != center || d.input_rate != block.sample_rate) {
            down = Some(Downconverter::new(center, &block));
            buffer.clear();
        }
        let d = down.as_mut().expect("downconverter");
        chunk.clear();
        d.process(&block, &mut chunk);

        // Time stamps: the block ended about when it arrived. While the stream runs
        // without gaps, samples are counted on from the first; a gap or a jump of the
        // clock starts the count again.
        let block_start = arrived - block.samples.len() as f64 / block.sample_rate;
        let buffer_end = buffer_start + buffer.len() as f64 / RATE;
        if buffer.is_empty() || (block_start - buffer_end).abs() > 0.25 {
            buffer.clear();
            buffer_start = block_start;
        }
        buffer.extend(chunk.iter().copied());
        let excess = buffer.len().saturating_sub((KEEP_S * RATE) as usize);
        if excess > 0 {
            buffer.drain(..excess);
            buffer_start += excess as f64 / RATE;
        }

        // Decode a slot once DECODE_AT_S of it has arrived.
        let latest = buffer_start + buffer.len() as f64 / RATE;
        let slot = ((latest - DECODE_AT_S) / SLOT_SECONDS).floor() * SLOT_SECONDS;
        if slot <= 0.0 || decoded_slot == Some(slot as u64) || latest < slot + DECODE_AT_S {
            continue;
        }
        decoded_slot = Some(slot as u64);
        let first = ((slot - buffer_start) * RATE).round() as isize;
        let count = (DECODE_AT_S * RATE) as usize;
        // Slot samples; missing ones (before the buffer began) are zero.
        let samples: Vec<Complex<f32>> = (0..count as isize)
            .map(|i| {
                let k = first + i;
                if k >= 0 && (k as usize) < buffer.len() { buffer[k as usize] } else { Complex::default() }
            })
            .collect();
        let coverage = (count as isize - (-first).max(0)).max(0) as f64 / RATE;
        if coverage < START_SECONDS + 12.0 {
            // Not enough of the slot to hold a whole transmission.
            continue;
        }
        let (me2, book2) = (me.clone(), book.clone());
        let slot_secs = slot as u64;
        thread::spawn(move || {
            let lo = AUDIO_RANGE_HZ.0 - CENTER_ABOVE_DIAL_HZ;
            let hi = AUDIO_RANGE_HZ.1 - CENTER_ABOVE_DIAL_HZ;
            let found = decode::decode_slot(&samples, lo, hi, &mut book2.lock().unwrap());
            let mut messages = me2.messages.lock().unwrap();
            for d in &found {
                messages.push(Ft8Message {
                    slot: slot_secs,
                    snr_db: d.snr_db,
                    dt_s: d.dt_s,
                    audio_hz: d.freq_hz + CENTER_ABOVE_DIAL_HZ,
                    text: d.text.clone(),
                });
            }
            let excess = messages.len().saturating_sub(KEEP_MESSAGES);
            messages.drain(..excess);
            *me2.status.lock().unwrap() = Ft8Status { last_slot: Some(slot_secs), last_count: found.len(), coverage_s: coverage };
        });
    }
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
        let ft8 = Ft8::default();
        let (tx, rx) = std::sync::mpsc::sync_channel(32);
        handle.commands.send(Command::Subscribe(10, tx)).unwrap();
        ft8.spawn(rx);
        ft8.set_dial(Some(10_489.540e6 - lo - correction));
        for _ in 0..40 {
            thread::sleep(Duration::from_secs(1));
        }
        println!("{:?}", ft8.status());
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
        // Build the FT8 baseband at RATE, then put it up into the wide band.
        let n = (14.0 * RATE) as usize;
        let mut base = noise(n, 0.05, 3);
        add_signal(&mut base, "CQ TA4SO KM46", 1500.0 - CENTER_ABOVE_DIAL_HZ, 0.5, amplitude(0.0) * 0.05);
        let up = (rate / RATE) as usize;
        let offset = dial + CENTER_ABOVE_DIAL_HZ - center_rf;
        let mut wide = Vec::with_capacity(n * up);
        let mut phase = 0.0f64;
        for z in &base {
            for _ in 0..up {
                wide.push(z * Complex::from_polar(1.0, phase as f32));
                phase += 2.0 * std::f64::consts::PI * offset / rate;
            }
        }
        let mut d = Downconverter::new(dial + CENTER_ABOVE_DIAL_HZ, &Block { samples: Arc::new(Vec::new()), center_hz: center_rf, sample_rate: rate });
        let mut out = Vec::new();
        for c in wide.chunks(48_000) {
            d.process(&Block { samples: Arc::new(c.to_vec()), center_hz: center_rf, sample_rate: rate }, &mut out);
        }
        assert!((out.len() as f64 - n as f64).abs() < 50.0, "{} samples", out.len());
        // The filters delay the stream by a few ms; the decoder searches the start anyway.
        let found = decode::decode_slot(&out, -1500.0, 1500.0, &mut CallBook::default());
        println!("{found:?}");
        let d = found.iter().find(|d| d.text == "CQ TA4SO KM46").expect("decoded");
        assert!((d.freq_hz + CENTER_ABOVE_DIAL_HZ - 1500.0).abs() < 4.0);
    }
}
