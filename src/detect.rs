//! Finding signals on the spectrum.
//!
//! The noise floor is estimated piecewise (receivers' filters make it uneven across the
//! band), then every run of points standing clear of it becomes a signal.

/// A signal seen on the spectrum.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Signal {
    /// Power-weighted centre (Hz, absolute).
    pub center_hz: f64,
    /// Width of the part above the detection threshold (Hz).
    pub bandwidth_hz: f64,
    pub peak_db: f32,
    /// Peak above the local noise floor (dB).
    pub snr_db: f32,
}

/// Number of pieces the noise floor is estimated in.
const FLOOR_SEGMENTS: usize = 32;
/// Fraction of the points in a piece taken as its floor (most points are noise).
const FLOOR_PERCENTILE: f32 = 0.25;
/// Gaps of up to this many points inside a signal do not split it.
const MAX_GAP: usize = 2;
/// At most this many signals are reported, the strongest first chosen.
const MAX_SIGNALS: usize = 48;

/// Noise floor for every point, interpolated between pieces.
pub fn noise_floor(db: &[f32]) -> Vec<f32> {
    let n = db.len();
    if n == 0 {
        return Vec::new();
    }
    let segments = FLOOR_SEGMENTS.min(n.max(1));
    let levels: Vec<(f32, f32)> = (0..segments)
        .map(|s| {
            let (a, b) = (s * n / segments, ((s + 1) * n / segments).max(s * n / segments + 1));
            let mut piece = db[a..b].to_vec();
            piece.sort_by(f32::total_cmp);
            let level = piece[((piece.len() - 1) as f32 * FLOOR_PERCENTILE) as usize];
            ((a + b) as f32 / 2.0, level)
        })
        .collect();
    (0..n)
        .map(|i| {
            let x = i as f32 + 0.5;
            match levels.iter().position(|&(c, _)| c >= x) {
                Some(0) => levels[0].1,
                None => levels[levels.len() - 1].1,
                Some(k) => {
                    let ((x0, y0), (x1, y1)) = (levels[k - 1], levels[k]);
                    y0 + (y1 - y0) * (x - x0) / (x1 - x0)
                }
            }
        })
        .collect()
}

/// Signals in `db`, whose values are evenly spaced from `lo_hz` to `hi_hz` (outer edges).
/// `threshold_db` is how far above the floor a point must be to count.
pub fn find(db: &[f32], lo_hz: f64, hi_hz: f64, threshold_db: f32) -> Vec<Signal> {
    let n = db.len();
    if n < 8 {
        return Vec::new();
    }
    let floor = noise_floor(db);
    let step = (hi_hz - lo_hz) / n as f64;
    let freq = |i: f64| lo_hz + (i + 0.5) * step;

    let mut found = Vec::new();
    let mut i = 0;
    while i < n {
        if db[i] - floor[i] < threshold_db {
            i += 1;
            continue;
        }
        // Extend the run, allowing short dips.
        let start = i;
        let mut end = i;
        let mut j = i + 1;
        while j < n && j <= end + MAX_GAP + 1 {
            if db[j] - floor[j] >= threshold_db {
                end = j;
            }
            j += 1;
        }
        let (mut weight, mut sum) = (0.0f64, 0.0f64);
        let (mut peak, mut snr) = (f32::NEG_INFINITY, f32::NEG_INFINITY);
        for k in start..=end {
            let p = 10f64.powf((db[k] - floor[k]) as f64 / 10.0);
            weight += p;
            sum += p * k as f64;
            peak = peak.max(db[k]);
            snr = snr.max(db[k] - floor[k]);
        }
        found.push(Signal {
            center_hz: freq(sum / weight),
            bandwidth_hz: (end - start + 1) as f64 * step,
            peak_db: peak,
            snr_db: snr,
        });
        i = end + 1;
    }
    if found.len() > MAX_SIGNALS {
        found.sort_by(|a, b| b.snr_db.total_cmp(&a.snr_db));
        found.truncate(MAX_SIGNALS);
        found.sort_by(|a, b| a.center_hz.total_cmp(&b.center_hz));
    }
    found
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tilted noise floor with two signals on it.
    fn test_spectrum() -> Vec<f32> {
        (0..1000)
            .map(|i| {
                let base = -90.0 + 10.0 * (i as f32 / 1000.0) + if i % 7 == 0 { 1.5 } else { 0.0 };
                match i {
                    200..=209 => -60.0,
                    700 => -50.0,
                    _ => base,
                }
            })
            .collect()
    }

    #[test]
    fn finds_wide_and_narrow_signals() {
        let db = test_spectrum();
        // 1000 points over 1 MHz: 1 kHz per point.
        let found = find(&db, 100e6, 101e6, 6.0);
        assert_eq!(found.len(), 2, "{found:?}");
        assert!((found[0].center_hz - 100.205e6).abs() < 1e3);
        assert!((found[0].bandwidth_hz - 10e3).abs() < 1.0);
        assert!((found[1].center_hz - 100.7005e6).abs() < 1e3);
        assert!(found[1].snr_db > 30.0);
    }

    #[test]
    fn floor_follows_tilt() {
        let floor = noise_floor(&test_spectrum());
        assert!((floor[50] + 89.5).abs() < 1.0);
        assert!((floor[950] + 80.5).abs() < 1.0);
    }
}
