//! The listening thread: takes blocks from the source, demodulates the selected
//! frequency and keeps the sound card's queue at a steady level.

use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use crate::audio::AudioBuffer;
use crate::demod::{Chain, ChainConfig, Mode};
use crate::rds::{RdsDecoder, RdsInfo};
use crate::source::Block;
use crate::stations::Stations;

/// Audio kept queued for the sound card (seconds): enough to ride out scheduling jitter,
/// short enough that tuning feels immediate.
const TARGET_DELAY: f64 = 0.12;
/// Above this the oldest audio is dropped (seconds).
const MAX_DELAY: f64 = 0.5;
/// Largest speed correction used to hold the queue level. The clocks of the SDR and the
/// sound card differ by some ppm, but network receivers can deliver up to ~1 % fewer
/// samples than nominal; 3 % (half a semitone at most) covers that.
const MAX_SPEED_CORRECTION: f64 = 0.03;
/// Speed correction per unit of relative queue error.
const SPEED_GAIN: f64 = 0.03;

/// What to listen to; the UI writes it, the thread reads it for every block.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ListenSettings {
    /// Absolute listening frequency (Hz).
    pub frequency_hz: f64,
    pub mode: Mode,
    pub bandwidth_hz: f64,
    /// Channel power (dBFS) below which the output is silenced; None for always open.
    pub squelch_db: Option<f32>,
    pub audio_rate: f64,
}

/// Shared between the UI and the thread.
#[derive(Clone)]
pub struct Listener {
    pub settings: Arc<Mutex<ListenSettings>>,
    level: Arc<AtomicU32>,
    rds: Arc<Mutex<Option<RdsInfo>>>,
}

impl Listener {
    pub fn new(settings: ListenSettings) -> Self {
        Self {
            settings: Arc::new(Mutex::new(settings)),
            level: Arc::new(AtomicU32::new((-150.0f32).to_bits())),
            rds: Arc::new(Mutex::new(None)),
        }
    }

    /// RDS of the station listened to (WFM), None in other modes.
    pub fn rds(&self) -> Option<RdsInfo> {
        self.rds.lock().unwrap().clone()
    }

    pub fn update(&self, f: impl FnOnce(&mut ListenSettings)) {
        f(&mut self.settings.lock().unwrap());
    }

    /// Channel power of the listened signal (dBFS).
    pub fn level_db(&self) -> f32 {
        f32::from_bits(self.level.load(Ordering::Relaxed))
    }

    /// Runs until the source stops (its block channel closes). Station names read from
    /// RDS go to `stations`.
    pub fn spawn(&self, blocks: Receiver<Block>, audio: AudioBuffer, stations: Stations) {
        let me = self.clone();
        thread::Builder::new().name("listen".into()).spawn(move || run(blocks, me, audio, stations)).expect("listen thread");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Live listening without a sound card; writes a WAV file:
    /// `SEDIR_SOURCE=ip:192.168.1.47 SEDIR_FREQ=102.2e6 SEDIR_MODE=WFM SEDIR_WAV=out.wav
    ///  cargo test --lib live_listen -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn live_listen() {
        use crate::device::{DeviceInfo, DeviceKind};
        use crate::radio::Tuning;
        use crate::source::{self, Command, View};
        use std::time::Instant;
        let Ok(args) = std::env::var("SEDIR_SOURCE") else { return };
        let env = |k: &str, d: f64| std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d);
        let kind = if args.starts_with("ip:") { DeviceKind::LibreSdr } else if args.contains("hackrf") { DeviceKind::HackRf } else { DeviceKind::RtlSdr };
        let rate = env("SEDIR_RATE", kind.limits().max_rate);
        let freq = env("SEDIR_FREQ", 102.2e6);
        let mode = std::env::var("SEDIR_MODE").ok().and_then(|m| Mode::from_name(&m)).unwrap_or(Mode::Wfm);
        // Listen beside the centre, as when clicking a station in the spectrum.
        let center = freq - rate * 0.2;
        let device = DeviceInfo { label: args.clone(), args, kind };
        let handle = source::spawn(device, Tuning { center_hz: center, sample_rate: rate, gain: Some(env("SEDIR_GAIN", 30.0)) }, View::full(rate), || {});
        let audio_rate = 48_000.0;
        let listener = Listener::new(ListenSettings { frequency_hz: freq, mode, bandwidth_hz: mode.default_bandwidth(), squelch_db: None, audio_rate });
        let (tx, rx) = std::sync::mpsc::sync_channel(32);
        handle.commands.send(Command::Subscribe(1, tx)).unwrap();
        let audio = AudioBuffer::default();
        listener.spawn(rx, audio.clone(), Stations::new(false));
        // Stand-in for the sound card: take 48 000 samples per second.
        let started = Instant::now();
        let mut taken: Vec<f32> = Vec::new();
        let seconds = 6.0;
        let mut second = (usize::MAX, 0usize, 0usize);
        while started.elapsed().as_secs_f64() < seconds {
            std::thread::sleep(Duration::from_millis(20));
            let due = (started.elapsed().as_secs_f64() * audio_rate) as usize;
            let mut chunk = vec![0.0; due.saturating_sub(taken.len())];
            let queued = audio.len();
            second = (second.0.min(queued), second.1.max(queued), second.2 + chunk.iter().len());
            audio.pop_into(&mut chunk);
            second.2 -= chunk.iter().filter(|v| **v != 0.0).count();
            taken.extend(chunk);
            if taken.len() / 48_000 != (taken.len() - 960) / 48_000 {
                println!("queue min {} max {}, zero samples {}", second.0, second.1, second.2);
                second = (usize::MAX, 0, 0);
            }
        }
        println!("RDS: {:?}", listener.rds());
        for e in handle.events.try_iter() {
            if let crate::source::Event::Stats(s) = e {
                println!("{s:?}");
            }
        }
        let tail = &taken[taken.len() / 3..];
        let rms = (tail.iter().map(|v| v * v).sum::<f32>() / tail.len() as f32).sqrt();
        let silent = tail.iter().filter(|v| **v == 0.0).count();
        println!("level {:.1} dBFS, audio RMS {rms:.3}, {} samples, exact zeros in tail {silent}, queued {}", listener.level_db(), taken.len(), audio.len());
        if let Some(path) = std::env::var_os("SEDIR_WAV") {
            let mut wav = Vec::new();
            let data_len = (taken.len() * 2) as u32;
            wav.extend(b"RIFF");
            wav.extend((36 + data_len).to_le_bytes());
            wav.extend(b"WAVEfmt ");
            wav.extend(16u32.to_le_bytes());
            wav.extend(1u16.to_le_bytes());
            wav.extend(1u16.to_le_bytes());
            wav.extend(48_000u32.to_le_bytes());
            wav.extend(96_000u32.to_le_bytes());
            wav.extend(2u16.to_le_bytes());
            wav.extend(16u16.to_le_bytes());
            wav.extend(b"data");
            wav.extend(data_len.to_le_bytes());
            for v in &taken {
                wav.extend(((v.clamp(-1.0, 1.0) * 32_000.0) as i16).to_le_bytes());
            }
            std::fs::write(path, wav).unwrap();
        }
        assert!(rms > 0.02, "no audio");
        assert!(silent < tail.len() / 20, "gaps in the audio: {silent} zero samples");
    }
}

fn run(blocks: Receiver<Block>, me: Listener, audio: AudioBuffer, stations: Stations) {
    let (settings, level) = (me.settings, me.level);
    let mut chain: Option<Chain> = None;
    // Listening frequency the RDS decoder has been following.
    let mut rds_frequency = f64::NAN;
    let mut out = Vec::new();
    loop {
        let block = match blocks.recv_timeout(Duration::from_millis(500)) {
            Ok(b) => b,
            Err(RecvTimeoutError::Timeout) => continue,
            Err(RecvTimeoutError::Disconnected) => return,
        };
        let s = *settings.lock().unwrap();
        let config = ChainConfig { mode: s.mode, bandwidth: s.bandwidth_hz, input_rate: block.sample_rate, audio_rate: s.audio_rate };
        if chain.as_ref().is_none_or(|c| c.config != config) {
            chain = Some(Chain::new(config));
            audio.clear();
            audio.set_prime((s.audio_rate * TARGET_DELAY) as usize);
        }
        let chain = chain.as_mut().expect("chain");
        // Moved to another station: its RDS starts from scratch.
        if (s.frequency_hz - rds_frequency).abs() >= crate::stations::SAME_STATION_HZ / 2.0 {
            rds_frequency = s.frequency_hz;
            if chain.rds.is_some() {
                chain.rds = Some(RdsDecoder::new(chain.channel_rate()));
            }
        }
        let offset = s.frequency_hz - block.center_hz;
        if offset.abs() > block.sample_rate / 2.0 {
            // Listening outside the received band: nothing to hear.
            level.store((-150.0f32).to_bits(), Ordering::Relaxed);
            continue;
        }

        // Produce slightly fewer samples while the queue is above its target, more below.
        let queued = audio.len() as f64 / s.audio_rate;
        if queued > MAX_DELAY {
            audio.trim_to((s.audio_rate * TARGET_DELAY) as usize);
        }
        let error = (queued - TARGET_DELAY) / TARGET_DELAY;
        let speed = 1.0 + (error * SPEED_GAIN).clamp(-MAX_SPEED_CORRECTION, MAX_SPEED_CORRECTION);

        out.clear();
        chain.process(&block.samples, offset, speed, s.squelch_db, &mut out);
        level.store(chain.level_db.to_bits(), Ordering::Relaxed);
        audio.push(&out);
        let info = chain.rds.as_ref().map(|r| r.info().clone());
        if let Some(name) = info.as_ref().and_then(|i| i.stable_ps.as_ref()) {
            stations.record(s.frequency_hz, name);
        }
        *me.rds.lock().unwrap() = info;
    }
}
