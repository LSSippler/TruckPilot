//! Moving-median bias estimator for `lane.center_offset`.
//!
//! ETS2 Cockpit-View places the driver left of centre, which introduces a
//! constant positive offset in the raw lane measurement (~0.20 empirically).
//! `BiasEstimator` accumulates the last `window_size` raw values and returns
//! their median as the bias.  During the first `warmup_frames` frames the bias
//! is reported as 0.0 so that the ring-buffer has time to fill before
//! correction kicks in.

use std::collections::VecDeque;

/// Minimum configurable window size (prevents division-by-zero edge cases).
const MIN_WINDOW: usize = 1;

pub struct BiasEstimator {
    buf: VecDeque<f64>,
    pub window_size: usize,
    pub warmup_frames: usize,
    frames_seen: usize,
}

impl BiasEstimator {
    pub fn new(window_size: usize, warmup_frames: usize) -> Self {
        let w = window_size.max(MIN_WINDOW);
        Self {
            buf: VecDeque::with_capacity(w),
            window_size: w,
            warmup_frames,
            frames_seen: 0,
        }
    }

    /// Push a new raw offset sample into the ring-buffer.
    pub fn push(&mut self, raw: f64) {
        if self.buf.len() == self.window_size {
            self.buf.pop_front();
        }
        self.buf.push_back(raw);
        self.frames_seen += 1;
    }

    /// Current bias estimate.  Returns `0.0` during warm-up or if the buffer
    /// is empty.
    pub fn bias(&self) -> f64 {
        if self.frames_seen < self.warmup_frames || self.buf.is_empty() {
            return 0.0;
        }
        median_of(&self.buf)
    }

    /// Bias-corrected offset: `raw - bias()`.
    pub fn corrected(&self, raw: f64) -> f64 {
        raw - self.bias()
    }
}

fn median_of(buf: &VecDeque<f64>) -> f64 {
    let mut v: Vec<f64> = buf.iter().copied().collect();
    v.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let n = v.len();
    if n % 2 == 1 {
        v[n / 2]
    } else {
        (v[n / 2 - 1] + v[n / 2]) / 2.0
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn feed(est: &mut BiasEstimator, value: f64, count: usize) {
        for _ in 0..count {
            est.push(value);
        }
    }

    // ── Warm-up ────────────────────────────────────────────────────────────

    #[test]
    fn bias_zero_during_warmup() {
        let mut est = BiasEstimator::new(100, 50);
        // Push 49 samples (one short of warmup)
        feed(&mut est, 0.20, 49);
        assert_eq!(est.bias(), 0.0, "bias must be 0.0 before warmup completes");
    }

    #[test]
    fn bias_activates_at_warmup_boundary() {
        let mut est = BiasEstimator::new(100, 50);
        feed(&mut est, 0.20, 50); // exactly warmup_frames
        let b = est.bias();
        assert!(
            (b - 0.20).abs() < 1e-9,
            "bias should be ~0.20 at warmup boundary, got {b}"
        );
    }

    // ── Constant bias removal ──────────────────────────────────────────────

    #[test]
    fn constant_bias_removed_after_warmup() {
        let mut est = BiasEstimator::new(100, 50);
        feed(&mut est, 0.198, 100);
        let corrected = est.corrected(0.198);
        assert!(
            corrected.abs() < 1e-9,
            "corrected should be ~0, got {corrected}"
        );
    }

    #[test]
    fn signal_centred_after_bias_removal() {
        // Raw signal oscillates around 0.198 bias.
        let mut est = BiasEstimator::new(100, 50);
        for i in 0..100usize {
            let raw = 0.198 + if i % 2 == 0 { 0.05 } else { -0.05 };
            est.push(raw);
        }
        // Median of alternating [0.248, 0.148, ...] = 0.198
        let b = est.bias();
        assert!((b - 0.198).abs() < 0.01, "bias should be ~0.198, got {b}");
        let corrected = est.corrected(0.248);
        assert!(
            (corrected - 0.05).abs() < 0.01,
            "corrected=+0.05 expected, got {corrected}"
        );
    }

    // ── Spike robustness (lane-change simulation) ──────────────────────────

    #[test]
    fn spike_does_not_corrupt_bias() {
        let mut est = BiasEstimator::new(100, 50);
        // Fill with stable signal
        feed(&mut est, 0.20, 80);
        // Inject 5 large spikes (lane change)
        feed(&mut est, 0.80, 5);
        // Fill rest back to stable
        feed(&mut est, 0.20, 15);
        let b = est.bias();
        // With 80/100 samples at 0.20 and only 5 spikes, median stays at 0.20
        assert!(
            (b - 0.20).abs() < 0.02,
            "median bias should be ~0.20 after spikes, got {b}"
        );
    }

    #[test]
    fn mean_would_fail_but_median_holds() {
        let mut est = BiasEstimator::new(10, 5);
        // 8 samples at 0.20, 2 outliers at 5.0
        let samples = [0.20, 0.20, 0.20, 0.20, 5.0, 0.20, 0.20, 5.0, 0.20, 0.20];
        for &s in &samples {
            est.push(s);
        }
        let b = est.bias();
        assert!(
            (b - 0.20).abs() < 0.01,
            "median={b}, want 0.20; mean would be ~1.16"
        );
    }

    // ── Ring-buffer capacity ───────────────────────────────────────────────

    #[test]
    fn old_samples_evicted_when_full() {
        let mut est = BiasEstimator::new(4, 2);
        // Fill with 0.10 x4
        feed(&mut est, 0.10, 4);
        let b1 = est.bias();
        assert!((b1 - 0.10).abs() < 1e-9);
        // Now push 4 samples at 0.50 — old ones are evicted
        feed(&mut est, 0.50, 4);
        let b2 = est.bias();
        assert!(
            (b2 - 0.50).abs() < 1e-9,
            "bias should adapt to new signal, got {b2}"
        );
    }

    // ── Edge cases ─────────────────────────────────────────────────────────

    #[test]
    fn empty_buffer_returns_zero() {
        let est = BiasEstimator::new(100, 0);
        // warmup=0 but buffer empty
        assert_eq!(est.bias(), 0.0);
    }

    #[test]
    fn window_size_one_works() {
        let mut est = BiasEstimator::new(1, 1);
        est.push(0.33);
        let b = est.bias();
        assert!((b - 0.33).abs() < 1e-9, "window=1 bias={b}");
    }
}
