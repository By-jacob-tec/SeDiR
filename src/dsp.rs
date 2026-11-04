//! Signal processing building blocks: spectrum and DC removal.

use std::f32::consts::PI;
use std::sync::Arc;

use num_complex::Complex;
use rustfft::{Fft, FftPlanner};

/// Averaged power spectrum with a Blackman-Harris window (sidelobes below -92 dB, so a
/// weak signal next to a strong one stays visible). Samples may arrive in blocks of any
/// size; leftovers are kept for the next call.
pub struct Spectrum {
    fft: Arc<dyn Fft<f32>>,
    window: Vec<f32>,
    buf: Vec<Complex<f32>>,
    filled: usize,
    acc: Vec<f32>,
    count: usize,
    /// 1 / (sum of the window)²: a full-scale tone reads 0 dBFS.
    scale: f32,
}

impl Spectrum {
    pub fn new(size: usize) -> Self {
        let window: Vec<f32> = (0..size)
            .map(|i| {
                let x = 2.0 * PI * i as f32 / size as f32;
                0.35875 - 0.48829 * x.cos() + 0.14128 * (2.0 * x).cos() - 0.01168 * (3.0 * x).cos()
            })
            .collect();
        let sum: f32 = window.iter().sum();
        Self {
            fft: FftPlanner::new().plan_fft_forward(size),
            window,
            buf: vec![Complex::default(); size],
            filled: 0,
            acc: vec![0.0; size],
            count: 0,
            scale: 1.0 / (sum * sum),
        }
    }

    pub fn size(&self) -> usize {
        self.window.len()
    }

    /// Adds samples; every complete FFT block is accumulated.
    pub fn push(&mut self, mut samples: &[Complex<f32>]) {
        let n = self.size();
        while !samples.is_empty() {
            let take = (n - self.filled).min(samples.len());
            for (k, s) in samples[..take].iter().enumerate() {
                let i = self.filled + k;
                self.buf[i] = s * self.window[i];
            }
            self.filled += take;
            samples = &samples[take..];
            if self.filled == n {
                self.fft.process(&mut self.buf);
                for (a, x) in self.acc.iter_mut().zip(&self.buf) {
                    *a += x.norm_sqr();
                }
                self.count += 1;
                self.filled = 0;
            }
        }
    }

    /// Number of FFT blocks accumulated since the last `take`.
    pub fn count(&self) -> usize {
        self.count
    }

    /// Mean power in dBFS since the last call, centred (first element near -fs/2,
    /// element n/2 at the centre frequency). None if no block is complete yet.
    pub fn take(&mut self) -> Option<Vec<f32>> {
        if self.count == 0 {
            return None;
        }
        let n = self.size();
        let scale = self.scale / self.count as f32;
        let mut db: Vec<f32> = self.acc.iter().map(|p| 10.0 * (p * scale + 1e-20).log10()).collect();
        db.rotate_left(n / 2);
        self.acc.fill(0.0);
        self.count = 0;
        Some(db)
    }
}

/// Reduces `db[range]` to at most `points` values, each the highest of the bins it
/// covers (so a narrow carrier never disappears between pixels).
pub fn max_pool(db: &[f32], lo: usize, hi: usize, points: usize) -> Vec<f32> {
    let hi = hi.min(db.len());
    let lo = lo.min(hi);
    let bins = hi - lo;
    if bins <= points {
        return db[lo..hi].to_vec();
    }
    (0..points)
        .map(|p| {
            let a = lo + p * bins / points;
            let b = (lo + (p + 1) * bins / points).max(a + 1);
            db[a..b].iter().copied().fold(f32::NEG_INFINITY, f32::max)
        })
        .collect()
}

/// Removes the DC offset (the spike at the centre of most direct-conversion receivers)
/// with a slow single-pole high-pass filter.
pub struct DcBlocker {
    mean: Complex<f32>,
    alpha: f32,
}

impl DcBlocker {
    /// `alpha` is the update weight per sample; 1e-5 settles in ~100 000 samples.
    pub fn new(alpha: f32) -> Self {
        Self { mean: Complex::default(), alpha }
    }

    pub fn process(&mut self, samples: &mut [Complex<f32>]) {
        for s in samples {
            self.mean += (*s - self.mean) * self.alpha;
            *s -= self.mean;
        }
    }
}

/// FFT size that gives about `points` bins across the visible `span` (power of two).
pub fn fft_size_for(sample_rate: f64, span: f64, points: usize) -> usize {
    let wanted = (sample_rate / span.max(1.0) * points as f64).max(1024.0);
    (wanted as usize).next_power_of_two().clamp(1024, 1 << 18)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tone_lands_in_expected_bin_at_0_dbfs() {
        let n = 256;
        // Complex tone at fs/4: lands in bin 3n/4 of the centred spectrum.
        let samples: Vec<Complex<f32>> = (0..n * 4).map(|i| Complex::from_polar(1.0, PI / 2.0 * i as f32)).collect();
        let mut s = Spectrum::new(n);
        // Uneven pieces: leftovers carry over.
        s.push(&samples[..100]);
        s.push(&samples[100..]);
        assert_eq!(s.count(), 4);
        let db = s.take().unwrap();
        let peak = (0..n).max_by(|&a, &b| db[a].total_cmp(&db[b])).unwrap();
        assert_eq!(peak, 3 * n / 4);
        assert!(db[peak].abs() < 0.1, "peak {} dBFS", db[peak]);
        assert!(s.take().is_none());
    }

    #[test]
    fn max_pool_keeps_narrow_peaks() {
        let mut db = vec![-100.0; 1000];
        db[517] = -10.0;
        let pooled = max_pool(&db, 0, 1000, 100);
        assert_eq!(pooled.len(), 100);
        assert_eq!(pooled.iter().copied().fold(f32::MIN, f32::max), -10.0);
        assert_eq!(max_pool(&db, 10, 20, 100).len(), 10);
    }

    #[test]
    fn dc_blocker_removes_offset() {
        let mut x = vec![Complex::new(0.3, -0.2); 400_000];
        DcBlocker::new(1e-4).process(&mut x);
        assert!(x.last().unwrap().norm() < 1e-3);
    }

    #[test]
    fn fft_size_follows_zoom() {
        assert_eq!(fft_size_for(2.4e6, 2.4e6, 2048), 2048);
        assert_eq!(fft_size_for(2.4e6, 0.6e6, 2048), 8192);
        assert_eq!(fft_size_for(20e6, 1e3, 2048), 1 << 18);
    }
}
