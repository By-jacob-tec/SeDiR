//! Sample source: one thread reads the device, another computes the spectrum.
//!
//! The reader never waits for its consumers: every block of samples is offered to each
//! subscriber (the spectrum, later the demodulators) and dropped for those whose queue is
//! full. Dropped blocks and device overflows are counted for the status bar.

use std::collections::HashMap;
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender, SyncSender, TryRecvError, TrySendError};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use num_complex::Complex;

use crate::device::DeviceInfo;
use crate::dsp::{DcBlocker, Spectrum, fft_size_for, max_pool};
use crate::radio::{DriverInfo, Radio, Tuning};

/// Spectrum frames per second.
const FRAMES_PER_SECOND: f64 = 25.0;
/// Most values sent per spectrum frame (about the width of a large screen).
pub const FRAME_POINTS: usize = 4096;
/// Blocks a subscriber may have queued before new ones are dropped for it.
const QUEUE_BLOCKS: usize = 16;
/// Subscriber id of the spectrum worker.
const SPECTRUM_ID: u32 = 0;

/// A block of samples with the settings it was received with.
#[derive(Clone)]
pub struct Block {
    pub samples: Arc<Vec<Complex<f32>>>,
    pub center_hz: f64,
    pub sample_rate: f64,
}

/// The part of the received band shown on the spectrum.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct View {
    /// View centre relative to the tuned centre (Hz).
    pub offset_hz: f64,
    pub span_hz: f64,
}

impl View {
    pub fn full(sample_rate: f64) -> Self {
        Self { offset_hz: 0.0, span_hz: sample_rate }
    }

    /// Keeps the view inside the received band.
    pub fn clamped(self, sample_rate: f64) -> Self {
        let span = self.span_hz.clamp(1_000.0, sample_rate);
        let room = (sample_rate - span) / 2.0;
        Self { offset_hz: self.offset_hz.clamp(-room, room), span_hz: span }
    }
}

pub enum Command {
    Tune(f64),
    SampleRate(f64),
    Gain(Option<f64>),
    View(View),
    /// Sends every block to this channel from now on (for demodulators).
    Subscribe(u32, SyncSender<Block>),
    Unsubscribe(u32),
}

pub struct Frame {
    pub center_hz: f64,
    pub sample_rate: f64,
    /// Absolute frequencies of the first and last value's outer edges (Hz).
    pub lo_hz: f64,
    pub hi_hz: f64,
    /// Power in dBFS, evenly spaced from `lo_hz` to `hi_hz`.
    pub db: Vec<f32>,
    /// Width of one FFT bin (Hz).
    pub bin_hz: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Stats {
    /// Samples per second actually arriving from the device.
    pub measured_rate: f64,
    /// Times the device reported lost samples.
    pub overflows: u64,
    /// Blocks the spectrum could not keep up with.
    pub dropped_blocks: u64,
    /// Blocks the other subscribers (demodulators) could not keep up with.
    pub dropped_listen: u64,
}

pub enum Event {
    Opened(DriverInfo),
    /// The receiver was tuned on the device itself (an Icom's dial) to this frequency (Hz).
    Tuned(f64),
    Frame(Frame),
    Stats(Stats),
    Error(String),
}

/// Handle to the running source. Dropping it stops both threads and closes the device.
pub struct Handle {
    pub commands: Sender<Command>,
    pub events: Receiver<Event>,
}

/// Opens the device and starts reading. `notify` is called on every new event
/// (to wake the UI).
pub fn spawn(device: DeviceInfo, tuning: Tuning, view: View, notify: impl Fn() + Send + Sync + 'static) -> Handle {
    let (cmd_tx, cmd_rx) = mpsc::channel();
    let (ev_tx, ev_rx) = mpsc::channel();
    let notify: Arc<dyn Fn() + Send + Sync> = Arc::new(notify);
    thread::Builder::new()
        .name("sdr-reader".into())
        .spawn(move || {
            if let Err(e) = run(&device, tuning, view, &cmd_rx, &ev_tx, notify.clone()) {
                let _ = ev_tx.send(Event::Error(e));
                notify();
            }
        })
        .expect("reader thread");
    Handle { commands: cmd_tx, events: ev_rx }
}

/// Samples read per block: about 20 ms, so commands take effect quickly.
fn block_len(rate: f64) -> usize {
    // Narrow sources (an Icom's IF, 24 kS/s) would otherwise wait most of a second per block.
    if rate < 100e3 {
        return ((rate / 50.0) as usize).max(256);
    }
    ((rate / 50.0) as usize / 1024).max(16) * 1024
}

fn run(
    device: &DeviceInfo,
    mut tuning: Tuning,
    view: View,
    commands: &Receiver<Command>,
    events: &Sender<Event>,
    notify: Arc<dyn Fn() + Send + Sync>,
) -> Result<(), String> {
    let (mut radio, driver) = Radio::open(device, &tuning)?;
    let _ = events.send(Event::Opened(driver));
    notify();

    let shared_view = Arc::new(Mutex::new(view.clamped(tuning.sample_rate)));
    let mut subscribers: HashMap<u32, SyncSender<Block>> = HashMap::new();
    {
        let (tx, rx) = mpsc::sync_channel(QUEUE_BLOCKS);
        subscribers.insert(SPECTRUM_ID, tx);
        let (events, view, notify) = (events.clone(), shared_view.clone(), notify.clone());
        thread::Builder::new()
            .name("spectrum".into())
            .spawn(move || spectrum_worker(rx, view, events, notify))
            .expect("spectrum thread");
    }

    let mut dc = DcBlocker::new(1e-5);
    let mut stats = Stats::default();
    let mut window_start = Instant::now();
    let mut window_samples = 0u64;
    loop {
        loop {
            let result = match commands.try_recv() {
                Ok(Command::Tune(hz)) => {
                    tuning.center_hz = hz;
                    radio.set_frequency(hz)
                }
                Ok(Command::Gain(g)) => {
                    tuning.gain = g;
                    radio.set_gain(g)
                }
                Ok(Command::SampleRate(rate)) => {
                    tuning.sample_rate = rate;
                    let mut v = shared_view.lock().unwrap();
                    *v = v.clamped(rate);
                    drop(v);
                    radio.set_sample_rate(rate)
                }
                Ok(Command::View(v)) => {
                    *shared_view.lock().unwrap() = v.clamped(tuning.sample_rate);
                    Ok(())
                }
                Ok(Command::Subscribe(id, tx)) => {
                    subscribers.insert(id, tx);
                    Ok(())
                }
                Ok(Command::Unsubscribe(id)) => {
                    subscribers.remove(&id);
                    Ok(())
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => return Ok(()),
            };
            // Some devices reject settings while streaming (e.g. the RTL-SDR over USB/IP);
            // opening again with all settings works.
            if result.is_err() {
                drop(radio);
                radio = Radio::open(device, &tuning)?.0;
            }
        }

        let mut samples = vec![Complex::default(); block_len(tuning.sample_rate)];
        let mut filled = 0;
        while filled < samples.len() {
            match radio.read(&mut samples[filled..])? {
                Some(n) => filled += n,
                None => stats.overflows += 1,
            }
        }
        if let Some(hz) = radio.take_tuned() {
            if (hz - tuning.center_hz).abs() >= 1.0 {
                tuning.center_hz = hz;
                let _ = events.send(Event::Tuned(hz));
                notify();
            }
        }
        dc.process(&mut samples);
        window_samples += samples.len() as u64;

        let block = Block { samples: Arc::new(samples), center_hz: tuning.center_hz, sample_rate: tuning.sample_rate };
        subscribers.retain(|&id, tx| match tx.try_send(block.clone()) {
            Ok(()) => true,
            Err(TrySendError::Full(_)) => {
                if id == SPECTRUM_ID {
                    stats.dropped_blocks += 1;
                } else {
                    stats.dropped_listen += 1;
                }
                true
            }
            Err(TrySendError::Disconnected(_)) => false,
        });

        let elapsed = window_start.elapsed();
        if elapsed >= Duration::from_secs(1) {
            stats.measured_rate = window_samples as f64 / elapsed.as_secs_f64();
            window_start = Instant::now();
            window_samples = 0;
            if events.send(Event::Stats(stats)).is_err() {
                return Ok(());
            }
            notify();
        }
    }
}

fn spectrum_worker(blocks: Receiver<Block>, view: Arc<Mutex<View>>, events: Sender<Event>, notify: Arc<dyn Fn() + Send + Sync>) {
    let mut spectrum = Spectrum::new(2048);
    // Settings the accumulated blocks were received with; a change starts over.
    let mut current: Option<(f64, f64, usize)> = None;
    let mut since_frame = 0usize;
    loop {
        let block = match blocks.recv_timeout(Duration::from_millis(500)) {
            Ok(b) => b,
            Err(RecvTimeoutError::Timeout) => continue,
            Err(RecvTimeoutError::Disconnected) => return,
        };
        let v = *view.lock().unwrap();
        let size = fft_size_for(block.sample_rate, v.span_hz, FRAME_POINTS);
        let settings = (block.center_hz, block.sample_rate, size);
        if current != Some(settings) {
            if current.is_none_or(|c| c.2 != size) {
                spectrum = Spectrum::new(size);
            } else {
                let _ = spectrum.take();
            }
            current = Some(settings);
            since_frame = 0;
        }
        spectrum.push(&block.samples);
        since_frame += block.samples.len();
        if (since_frame as f64) < block.sample_rate / FRAMES_PER_SECOND {
            continue;
        }
        let Some(db) = spectrum.take() else { continue };
        since_frame = 0;
        let n = db.len() as f64;
        let bin_hz = block.sample_rate / n;
        let index = |offset: f64| offset / bin_hz + n / 2.0;
        let lo = index(v.offset_hz - v.span_hz / 2.0).floor().clamp(0.0, n) as usize;
        let hi = index(v.offset_hz + v.span_hz / 2.0).ceil().clamp(0.0, n) as usize;
        let edge = |i: usize| block.center_hz + (i as f64 - n / 2.0 - 0.5) * bin_hz;
        let frame = Frame {
            center_hz: block.center_hz,
            sample_rate: block.sample_rate,
            lo_hz: edge(lo),
            hi_hz: edge(hi),
            db: max_pool(&db, lo, hi, FRAME_POINTS),
            bin_hz,
        };
        if events.send(Event::Frame(frame)).is_err() {
            return;
        }
        notify();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn view_stays_inside_the_band() {
        let v = View { offset_hz: 2e6, span_hz: 1e6 }.clamped(2.4e6);
        assert_eq!(v.offset_hz, 0.7e6);
        let v = View { offset_hz: 0.5e6, span_hz: 5e6 }.clamped(2.4e6);
        assert_eq!(v, View::full(2.4e6));
    }

    /// Live chain on a real receiver, e.g. the FM broadcast band:
    /// `SEDIR_SOURCE=ip:192.168.1.47 SEDIR_RATE=12e6 SEDIR_FREQ=98e6 cargo test --lib live_source -- --ignored --nocapture`
    /// (SEDIR_SOURCE may also be SoapySDR arguments such as "driver=rtlsdr").
    #[test]
    #[ignore]
    fn live_source() {
        use crate::device::DeviceKind;
        let Ok(args) = std::env::var("SEDIR_SOURCE") else { return };
        let env = |k: &str, d: f64| std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d);
        let kind = if args.starts_with("ip:") || args.starts_with("usb:") {
            DeviceKind::LibreSdr
        } else if args.contains("hackrf") {
            DeviceKind::HackRf
        } else {
            DeviceKind::RtlSdr
        };
        let (rate, center) = (env("SEDIR_RATE", kind.limits().max_rate), env("SEDIR_FREQ", 98e6));
        let device = DeviceInfo { label: args.clone(), args, kind };
        let tuning = Tuning { center_hz: center, sample_rate: rate, gain: Some(env("SEDIR_GAIN", 30.0)) };
        let handle = spawn(device, tuning, View::full(rate), || {});
        let started = Instant::now();
        let (mut frames, mut last, mut stats) = (0, None, Stats::default());
        while started.elapsed() < Duration::from_secs(4) {
            match handle.events.recv_timeout(Duration::from_secs(5)) {
                Ok(Event::Opened(d)) => println!("opened: {d:?}"),
                Ok(Event::Tuned(hz)) => println!("tuned on the device: {hz}"),
                Ok(Event::Frame(f)) => {
                    frames += 1;
                    last = Some(f);
                }
                Ok(Event::Stats(s)) => stats = s,
                Ok(Event::Error(e)) => panic!("{e}"),
                Err(e) => panic!("no events: {e}"),
            }
        }
        println!("{frames} frames in 4 s, {stats:?}");
        let f = last.expect("no frame");
        println!("frame {} points, {:.0}–{:.0} Hz, bin {:.0} Hz", f.db.len(), f.lo_hz, f.hi_hz, f.bin_hz);
        for s in crate::detect::find(&f.db, f.lo_hz, f.hi_hz, 10.0) {
            println!("  {:.4} MHz  {:>7.1} kHz  {:5.1} dB", s.center_hz / 1e6, s.bandwidth_hz / 1e3, s.snr_db);
        }
        assert!(stats.measured_rate > rate * 0.95, "stream {:.0} of {rate}", stats.measured_rate);
    }

    #[test]
    fn blocks_are_about_20_ms() {
        assert_eq!(block_len(2.4e6), 46 * 1024);
        assert_eq!(block_len(20e6), 390 * 1024);
        assert_eq!(block_len(250e3), 16 * 1024);
        assert_eq!(block_len(24e3), 480);
    }
}
