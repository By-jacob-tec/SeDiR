//! Station names: a background scanner reads RDS from the FM stations in the received
//! band (several at a time) and keeps the names it finds, for the spectrum labels.

use std::sync::mpsc::{Receiver, RecvTimeoutError};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use crate::demod::MpxChannel;
use crate::rds::RdsDecoder;
use crate::source::Block;

/// Stations decoded at the same time.
const SLOTS: usize = 3;
/// Time given to a station to send its whole name (seconds of signal).
const TRY_SECONDS: f64 = 6.0;
/// A station that gave no name is tried again after this long.
const RETRY_AFTER: Duration = Duration::from_secs(45);
/// Frequencies closer than this belong to the same station (FM channels are 100 kHz apart).
pub const SAME_STATION_HZ: f64 = 60e3;

#[derive(Default)]
struct Inner {
    names: Vec<(f64, String)>,
    targets: Vec<f64>,
    enabled: bool,
}

/// Shared between the UI, the listening thread and the scanner.
#[derive(Clone, Default)]
pub struct Stations(Arc<Mutex<Inner>>);

impl Stations {
    pub fn new(enabled: bool) -> Self {
        let s = Self::default();
        s.0.lock().unwrap().enabled = enabled;
        s
    }

    pub fn set_enabled(&self, on: bool) {
        self.0.lock().unwrap().enabled = on;
    }

    /// Frequencies (Hz) of the FM stations currently seen on the spectrum.
    pub fn set_targets(&self, targets: Vec<f64>) {
        self.0.lock().unwrap().targets = targets;
    }

    pub fn record(&self, hz: f64, name: &str) {
        let name = name.trim();
        if name.is_empty() {
            return;
        }
        let mut inner = self.0.lock().unwrap();
        match inner.names.iter_mut().find(|(f, _)| (f - hz).abs() < SAME_STATION_HZ) {
            Some(entry) => *entry = (hz, name.to_string()),
            None => inner.names.push((hz, name.to_string())),
        }
    }

    /// Name of the station nearest to `hz`, if one is known within the same channel.
    pub fn name_near(&self, hz: f64) -> Option<String> {
        let inner = self.0.lock().unwrap();
        inner
            .names
            .iter()
            .filter(|(f, _)| (f - hz).abs() < SAME_STATION_HZ)
            .min_by(|a, b| (a.0 - hz).abs().total_cmp(&(b.0 - hz).abs()))
            .map(|(_, n)| n.clone())
    }

    /// Reads names in the background until the source stops.
    pub fn spawn_scanner(&self, blocks: Receiver<Block>) {
        let stations = self.clone();
        thread::Builder::new().name("rds-scan".into()).spawn(move || scan(blocks, stations)).expect("scanner thread");
    }
}

struct Slot {
    hz: f64,
    rate: f64,
    channel: MpxChannel,
    rds: RdsDecoder,
    seconds: f64,
}

fn scan(blocks: Receiver<Block>, stations: Stations) {
    let mut slots: Vec<Slot> = Vec::new();
    let mut attempts: Vec<(f64, Instant)> = Vec::new();
    let mut mpx = Vec::new();
    loop {
        let block = match blocks.recv_timeout(Duration::from_millis(500)) {
            Ok(b) => b,
            Err(RecvTimeoutError::Timeout) => continue,
            Err(RecvTimeoutError::Disconnected) => return,
        };
        let (targets, enabled) = {
            let inner = stations.0.lock().unwrap();
            (inner.targets.clone(), inner.enabled)
        };
        if !enabled {
            slots.clear();
            continue;
        }
        // A station must sit inside the band with room for its channel filter.
        let inside = |hz: f64| (hz - block.center_hz).abs() < block.sample_rate / 2.0 - 120e3;
        slots.retain(|s| s.rate == block.sample_rate && inside(s.hz) && s.seconds < TRY_SECONDS);
        attempts.retain(|(_, t)| t.elapsed() < RETRY_AFTER);
        for &hz in &targets {
            if slots.len() >= SLOTS {
                break;
            }
            let near = |f: f64| (f - hz).abs() < SAME_STATION_HZ;
            if !inside(hz)
                || stations.name_near(hz).is_some()
                || slots.iter().any(|s| near(s.hz))
                || attempts.iter().any(|(f, _)| near(*f))
            {
                continue;
            }
            attempts.push((hz, Instant::now()));
            let channel = MpxChannel::new(block.sample_rate);
            let rds = RdsDecoder::new(channel.rate);
            slots.push(Slot { hz, rate: block.sample_rate, channel, rds, seconds: 0.0 });
        }
        slots.retain_mut(|s| {
            mpx.clear();
            s.channel.process(&block.samples, s.hz - block.center_hz, &mut mpx);
            s.rds.process(&mpx);
            s.seconds += block.samples.len() as f64 / block.sample_rate;
            match &s.rds.info().stable_ps {
                Some(name) => {
                    stations.record(s.hz, name);
                    false
                }
                None => true,
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn names_are_matched_by_channel() {
        let s = Stations::new(true);
        s.record(96.6e6, "RADYO 1 ");
        s.record(102.2e6, "KRAL FM");
        assert_eq!(s.name_near(96.62e6).as_deref(), Some("RADYO 1"));
        assert_eq!(s.name_near(96.7e6), None);
        s.record(102.21e6, "KRAL POP");
        assert_eq!(s.name_near(102.2e6).as_deref(), Some("KRAL POP"));
    }
}
