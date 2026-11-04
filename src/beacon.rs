//! Frequency correction from a beacon of known frequency: the error of a converter's
//! oscillator (an LNB is off by kHz and drifts with temperature) is measured on the
//! beacon and followed continuously.

use std::sync::mpsc::{Receiver, RecvTimeoutError};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use num_complex::Complex;

use crate::demod::{FirDecimator, Mixer, halfband};
use crate::dsp::Spectrum;
use crate::source::Block;

/// QO-100 narrow-band transponder, lower (CW) beacon.
pub const QO100_BEACON_HZ: f64 = 10_489.500e6;
/// How far from its expected place the beacon is looked for before the first lock (Hz).
const SEARCH_HZ: f64 = 15e3;
/// After a lock, only this close to the last position (Hz): CW users sit a few kHz above.
const TRACK_HZ: f64 = 1.5e3;
/// The beacon must stand this far over the noise.
const MIN_SNR_DB: f32 = 15.0;
/// Measurement length (seconds of signal): long enough for both keying tones to show.
const MEASURE_SECONDS: f64 = 2.0;
/// Measurements whose median gives the correction.
const MEDIAN_OF: usize = 5;
/// FFT size at the ~40 kHz analysis rate: ~5 Hz bins.
const FFT_SIZE: usize = 8192;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum BeaconStatus {
    #[default]
    Searching,
    Locked {
        snr_db: f32,
    },
    /// No carrier strong enough where the beacon should be.
    NotFound,
    /// The beacon's frequency is outside the received band.
    OutsideBand,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BeaconState {
    /// Correction to add to the converter LO (Hz); None until the first lock.
    pub correction_hz: Option<f64>,
    pub status: BeaconStatus,
    pub updated: Option<Instant>,
}

pub type SharedBeacon = Arc<Mutex<BeaconState>>;

/// Follows the beacon at `beacon_hz` (radio frequency) through a converter with local
/// oscillator `lo_hz`, starting from `correction_hz`. Runs until the block channel closes.
pub fn spawn(blocks: Receiver<Block>, beacon_hz: f64, lo_hz: f64, correction_hz: f64) -> SharedBeacon {
    let state: SharedBeacon = Arc::new(Mutex::new(BeaconState::default()));
    let shared = state.clone();
    thread::Builder::new()
        .name("beacon".into())
        .spawn(move || run(blocks, beacon_hz, lo_hz, correction_hz, shared))
        .expect("beacon thread");
    state
}

/// Position of a peak between bins, from the dB values around it (parabola through three).
fn interpolate(a: f32, b: f32, c: f32) -> f64 {
    let d = a - 2.0 * b + c;
    if d.abs() < 1e-9 { 0.0 } else { (0.5 * (a - c) / d).clamp(-0.5, 0.5) as f64 }
}

/// Spacing of the beacon's two keying tones (Hz): ~395 Hz measured on the satellite.
const PAIR_SPACING_HZ: (f64, f64) = (330.0, 470.0);
/// The two tones' levels differ by less than this (dB).
const PAIR_LEVEL_DB: f32 = 15.0;

/// The beacon's offset (Hz) from the analysis centre in an averaged spectrum, and its SNR.
/// The QO-100 CW beacon keys by shifting between two tones ~395 Hz apart; which is the
/// stronger changes from second to second, and either can be missing from a short
/// measurement. The beacon is therefore recognised as a pair of carriers that far apart
/// at similar levels, and its lower tone is reported, so the result does not jump
/// between the tones; a stronger CW signal nearby is a single carrier and not taken.
pub fn find_beacon(db: &[f32], rate: f64, window_hz: f64) -> Option<(f64, f32)> {
    let n = db.len();
    let bin = rate / n as f64;
    let mut sorted = db.to_vec();
    sorted.sort_by(f32::total_cmp);
    let noise = sorted[n / 2];
    let reach = ((window_hz / bin) as usize).min(n / 2 - 2);
    // Carriers: local maxima over the threshold, the strongest within ~30 Hz kept.
    let mut carriers: Vec<(f64, f32)> = Vec::new();
    for i in n / 2 - reach..=n / 2 + reach {
        if !(db[i] >= db[i - 1] && db[i] >= db[i + 1] && db[i] - noise >= MIN_SNR_DB) {
            continue;
        }
        let at = (i as f64 - n as f64 / 2.0 + interpolate(db[i - 1], db[i], db[i + 1])) * bin;
        match carriers.last_mut() {
            Some(last) if at - last.0 < 30.0 => {
                if db[i] > last.1 {
                    *last = (at, db[i]);
                }
            }
            _ => carriers.push((at, db[i])),
        }
    }
    let mut best: Option<(f64, f32)> = None;
    for (k, &(f1, l1)) in carriers.iter().enumerate() {
        for &(f2, l2) in &carriers[k + 1..] {
            let spacing = f2 - f1;
            if spacing > PAIR_SPACING_HZ.1 {
                break;
            }
            if spacing >= PAIR_SPACING_HZ.0 && (l1 - l2).abs() <= PAIR_LEVEL_DB {
                let level = l1.min(l2);
                if best.is_none_or(|b| level > b.1) {
                    best = Some((f1, level));
                }
            }
        }
    }
    best.map(|(f, level)| (f, level - noise))
}

fn run(blocks: Receiver<Block>, beacon_hz: f64, lo_hz: f64, mut correction: f64, state: SharedBeacon) {
    let mut locked = false;
    let mut recent: Vec<f64> = Vec::new();
    // Receiver frequency where the beacon is expected.
    let expected = |correction: f64| beacon_hz - lo_hz - correction;
    let mut chain: Option<(f64, f64, f64, Mixer, Vec<FirDecimator<Complex<f32>>>, f64)> = None;
    let mut spectrum = Spectrum::new(FFT_SIZE);
    let mut collected = 0.0;
    let (mut a, mut b) = (Vec::new(), Vec::new());
    loop {
        let block = match blocks.recv_timeout(Duration::from_millis(500)) {
            Ok(b) => b,
            Err(RecvTimeoutError::Timeout) => continue,
            Err(RecvTimeoutError::Disconnected) => return,
        };
        let target = expected(correction);
        if (target - block.center_hz).abs() > block.sample_rate / 2.0 - 25e3 {
            state.lock().unwrap().status = BeaconStatus::OutsideBand;
            chain = None;
            continue;
        }
        if chain.as_ref().is_none_or(|c| c.0 != block.center_hz || c.1 != block.sample_rate || c.2 != target) {
            let mut rate = block.sample_rate;
            let mut halfbands = Vec::new();
            while rate / 2.0 >= 32e3 {
                halfbands.push(halfband());
                rate /= 2.0;
            }
            chain = Some((block.center_hz, block.sample_rate, target, Mixer::new(block.center_hz - target, block.sample_rate), halfbands, rate));
            let _ = spectrum.take();
            collected = 0.0;
        }
        let (_, _, _, mixer, halfbands, rate) = chain.as_mut().expect("chain");
        a.clear();
        a.extend_from_slice(&block.samples);
        mixer.mix(&mut a);
        for hb in halfbands.iter_mut() {
            b.clear();
            hb.process(&a, &mut b);
            std::mem::swap(&mut a, &mut b);
        }
        spectrum.push(&a);
        collected += a.len() as f64 / *rate;
        if collected < MEASURE_SECONDS {
            continue;
        }
        collected = 0.0;
        let Some(db) = spectrum.take() else { continue };
        let window = if locked { TRACK_HZ } else { SEARCH_HZ };
        let mut s = state.lock().unwrap();
        match find_beacon(&db, *rate, window) {
            Some((offset, snr)) => {
                // The beacon sits `offset` above where it was expected: the LO is that
                // much lower than assumed, so the correction falls by it. The median of
                // the last measurements is used, so one bad measurement cannot pull it.
                recent.push(correction - offset);
                if recent.len() > MEDIAN_OF {
                    recent.remove(0);
                }
                let mut sorted = recent.clone();
                sorted.sort_by(f64::total_cmp);
                correction = sorted[sorted.len() / 2];
                locked = true;
                *s = BeaconState { correction_hz: Some(correction), status: BeaconStatus::Locked { snr_db: snr }, updated: Some(Instant::now()) };
            }
            None => {
                // A measurement without the pair keeps the lock for a while.
                if s.updated.is_none_or(|t| t.elapsed() > Duration::from_secs(10)) {
                    s.status = BeaconStatus::NotFound;
                    locked = false;
                    recent.clear();
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;
    use std::sync::mpsc;

    /// Receiver blocks of a band with a beacon `offset_hz` from where a correct LO would
    /// put it, a stronger CW signal 8 kHz above it, and noise.
    fn blocks(center_if: f64, rate: f64, beacon_if: f64, seconds: f64) -> Vec<Block> {
        let mut seed = 7u32;
        let mut noise = move || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5
        };
        let n = (rate * seconds) as usize;
        let samples: Vec<Complex<f32>> = (0..n)
            .map(|i| {
                let t = i as f64 / rate;
                // Two-tone keying like the satellite's beacon: the upper tone half the time.
                let shift = if (t * 4.0) as u32 % 2 == 0 { 0.0 } else { 393.0 };
                let beacon = Complex::from_polar(0.05f32, (2.0 * PI * (beacon_if - center_if) * t + 2.0 * PI * shift * t) as f32);
                let cw = Complex::from_polar(0.2f32, (2.0 * PI * (beacon_if + 8e3 - center_if) * t) as f32);
                beacon + cw + Complex::new(noise(), noise()) * 0.02
            })
            .collect();
        samples
            .chunks(48 * 1024)
            .map(|c| Block { samples: Arc::new(c.to_vec()), center_hz: center_if, sample_rate: rate })
            .collect()
    }

    #[test]
    fn locks_on_the_beacon_not_the_stronger_cw() {
        let lo = 9_750e6;
        // The LNB is 3.2 kHz high: the beacon shows 3.2 kHz low.
        let true_lo = lo + 3_200.0;
        let beacon_if = QO100_BEACON_HZ - true_lo;
        let (tx, rx) = mpsc::sync_channel(1000);
        for b in blocks(739.75e6, 2.4e6, beacon_if, 6.5) {
            tx.send(b).unwrap();
        }
        drop(tx);
        let state = spawn(rx, QO100_BEACON_HZ, lo, 0.0);
        let started = Instant::now();
        while started.elapsed() < Duration::from_secs(20) {
            let s = *state.lock().unwrap();
            if s.correction_hz.is_some() && Arc::strong_count(&state) == 1 {
                break;
            }
            thread::sleep(Duration::from_millis(50));
        }
        let s = *state.lock().unwrap();
        println!("{s:?}");
        assert!(matches!(s.status, BeaconStatus::Locked { .. }));
        let c = s.correction_hz.unwrap();
        assert!((c - 3_200.0).abs() < 3.0, "correction {c}");
    }

    /// On the satellite: `SEDIR_SOURCE=ip:192.168.1.47 cargo test --lib live_beacon -- --ignored --nocapture`
    #[test]
    #[ignore]
    fn live_beacon() {
        use crate::device::{DeviceInfo, DeviceKind};
        use crate::radio::Tuning;
        use crate::source::{self, Command, View};
        let Ok(args) = std::env::var("SEDIR_SOURCE") else { return };
        let (lo, rate) = (9_750e6, 2.4e6);
        let device = DeviceInfo { label: args.clone(), args, kind: DeviceKind::LibreSdr };
        let handle = source::spawn(device, Tuning { center_hz: 10_489.75e6 - lo, sample_rate: rate, gain: Some(40.0) }, View::full(rate), || {});
        let (tx, rx) = mpsc::sync_channel(32);
        handle.commands.send(Command::Subscribe(9, tx)).unwrap();
        // Carriers around the expected beacon in two 1 s spectra.
        let (tx2, rx2) = mpsc::sync_channel(64);
        handle.commands.send(Command::Subscribe(8, tx2)).unwrap();
        for _ in 0..2 {
            let target = QO100_BEACON_HZ - lo;
            let mut mixer = Mixer::new(10_489.75e6 - lo - target, rate);
            let mut hbs: Vec<FirDecimator<Complex<f32>>> = (0..6).map(|_| halfband()).collect();
            let mut spectrum = Spectrum::new(FFT_SIZE);
            let mut got = 0.0;
            while got < 1.0 {
                let block = rx2.recv().unwrap();
                let mut a = block.samples.to_vec();
                mixer.mix(&mut a);
                for hb in hbs.iter_mut() {
                    let mut b = Vec::new();
                    hb.process(&a, &mut b);
                    a = b;
                }
                spectrum.push(&a);
                got += a.len() as f64 / (rate / 64.0);
            }
            let db = spectrum.take().unwrap();
            let n = db.len();
            let bin = rate / 64.0 / n as f64;
            let mut sorted = db.clone();
            sorted.sort_by(f32::total_cmp);
            let noise = sorted[n / 2];
            let peaks: Vec<String> = (1..n - 1)
                .filter(|&i| db[i] > db[i - 1] && db[i] >= db[i + 1] && db[i] - noise > 12.0)
                .filter(|&i| ((i as f64 - n as f64 / 2.0) * bin).abs() < 15e3)
                .map(|i| format!("{:+.0} Hz {:.0} dB", (i as f64 - n as f64 / 2.0) * bin, db[i] - noise))
                .collect();
            println!("peaks: {}", peaks.join(", "));
        }
        handle.commands.send(Command::Unsubscribe(8)).unwrap();
        let state = spawn(rx, QO100_BEACON_HZ, lo, 0.0);
        for _ in 0..8 {
            thread::sleep(Duration::from_secs(2));
            println!("{:?}", *state.lock().unwrap());
        }
    }

    #[test]
    fn peak_interpolation() {
        assert_eq!(interpolate(-10.0, 0.0, -10.0), 0.0);
        assert!(interpolate(-4.0, 0.0, -10.0) < 0.0);
    }
}
