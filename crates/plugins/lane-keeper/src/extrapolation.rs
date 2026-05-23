//! Single-lane width estimation and center extrapolation for Level-1 fallback.

/// Lane width decay starts at this many ticks of continuous single-lane operation.
const DECAY_START_TICKS: u64 = 150;
/// Width estimate halves every this many ticks beyond DECAY_START_TICKS.
const DECAY_HALF_LIFE_TICKS: u64 = 300;
/// Below this normalised width the estimate is discarded (too uncertain).
const MIN_VALID_WIDTH: f64 = 0.10;
/// Min/max allowed lane width in normalised frame coordinates.
const WIDTH_MIN: f64 = 0.15;
const WIDTH_MAX: f64 = 0.80;
/// Exponential-moving-average weight for new width samples.
const WIDTH_EMA_ALPHA: f64 = 0.70;
/// Confidence penalty applied to single-lane extrapolated output.
const SINGLE_LANE_CONF_FACTOR: f64 = 0.60;

/// Tracks the estimated lane width derived from frames where both lanes are visible.
#[derive(Default)]
pub struct LaneWidthState {
    /// Smoothed lane width in normalised frame coordinates, or None if never set.
    estimated_width: Option<f64>,
    /// Tick at which the last both-lanes measurement was captured.
    captured_at_tick: u64,
    /// Monotonic tick counter (advances every time `advance_tick` is called).
    pub tick_count: u64,
}

impl LaneWidthState {
    pub fn new() -> Self {
        Self::default()
    }

    /// Advance the internal tick counter by one.
    pub fn advance_tick(&mut self) {
        self.tick_count += 1;
    }

    /// Update the width estimate using a new both-lanes measurement.
    /// `new_width` must be in normalised frame coordinates [-1, +1] span.
    pub fn update_lane_width(&mut self, new_width: f64) {
        if !(WIDTH_MIN..=WIDTH_MAX).contains(&new_width) {
            return;
        }
        self.estimated_width = Some(match self.estimated_width {
            None => new_width,
            Some(prev) => WIDTH_EMA_ALPHA * new_width + (1.0 - WIDTH_EMA_ALPHA) * prev,
        });
        self.captured_at_tick = self.tick_count;
    }

    /// Return the current width estimate, applying exponential decay if only one lane
    /// has been visible for a while. Returns `None` if estimate is too uncertain.
    pub fn lane_width_estimate(&self) -> Option<f64> {
        let w = self.estimated_width?;
        let ticks_since = self.tick_count.saturating_sub(self.captured_at_tick);
        if ticks_since <= DECAY_START_TICKS {
            return Some(w);
        }
        let extra = (ticks_since - DECAY_START_TICKS) as f64;
        let decay = (extra / DECAY_HALF_LIFE_TICKS as f64).exp2().recip(); // 2^(-extra/half_life)
        let decayed = w * decay;
        if decayed < MIN_VALID_WIDTH { None } else { Some(decayed) }
    }
}

/// Extrapolate the lane center when only one bounding lane is visible.
///
/// Returns `(center_frame_coord, effective_confidence)` where `center_frame_coord`
/// is in the same [-1, +1] frame-coordinate space as `lane.left_x` / `lane.right_x`
/// (0 = frame centre, negative = left of centre, positive = right of centre).
/// The returned value is fed directly to the PID as the signed error signal:
///   positive → truck left of lane centre → steer right.
///
/// `left_x` / `right_x` are `Option<f64>` carrying `f64::NAN` as absent sentinel
/// (parsed from Blackboard strings). Callers must convert NaN to `None` before here.
///
/// When both lanes are visible this function also updates the stored width estimate
/// and returns the direct-centre-based error (same as Level-0 path does externally),
/// with confidence unchanged.
///
/// When neither lane is visible, returns `(0.0, 0.0)`.
pub fn extrapolate_center(
    left_x: Option<f64>,
    right_x: Option<f64>,
    raw_confidence: f64,
    width_state: &mut LaneWidthState,
) -> (f64, f64) {
    match (left_x, right_x) {
        (Some(lx), Some(rx)) => {
            let width = (rx - lx).abs();
            width_state.update_lane_width(width);
            let center = (lx + rx) / 2.0;
            // center is in frame coords; error = -center (positive → steer right when truck left)
            (-center, raw_confidence)
        }
        (Some(lx), None) => {
            if let Some(w) = width_state.lane_width_estimate() {
                let center = lx + w / 2.0;
                (-center, raw_confidence * SINGLE_LANE_CONF_FACTOR)
            } else {
                (0.0, 0.0)
            }
        }
        (None, Some(rx)) => {
            if let Some(w) = width_state.lane_width_estimate() {
                let center = rx - w / 2.0;
                (-center, raw_confidence * SINGLE_LANE_CONF_FACTOR)
            } else {
                (0.0, 0.0)
            }
        }
        (None, None) => (0.0, 0.0),
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_state() -> LaneWidthState {
        LaneWidthState::new()
    }

    #[test]
    fn extrapolate_center_from_left_lane() {
        let mut s = make_state();
        // Seed a width estimate (both lanes visible first).
        s.update_lane_width(0.40);

        // Now only left lane at −0.30.
        // Expected center = −0.30 + 0.40/2 = −0.30 + 0.20 = −0.10 (frame coords)
        // Error = −(−0.10) = +0.10 (positive → steer right, truck left of centre)
        let (err, conf) = extrapolate_center(Some(-0.30), None, 0.80, &mut s);
        let expected_err = 0.10_f64;
        assert!(
            (err - expected_err).abs() < 0.01,
            "expected err≈{expected_err}, got {err}"
        );
        assert!((conf - 0.80 * 0.60).abs() < 1e-9, "conf={conf}");
    }

    #[test]
    fn extrapolate_center_from_right_lane() {
        let mut s = make_state();
        s.update_lane_width(0.40);

        // Only right lane at +0.30.
        // center = 0.30 − 0.40/2 = 0.30 − 0.20 = +0.10 (frame coords)
        // error  = −0.10 (negative → steer left, truck right of centre)
        let (err, conf) = extrapolate_center(None, Some(0.30), 0.80, &mut s);
        let expected_err = -0.10_f64;
        assert!(
            (err - expected_err).abs() < 0.01,
            "expected err≈{expected_err}, got {err}"
        );
        assert!((conf - 0.48).abs() < 1e-9, "conf={conf}");
    }

    #[test]
    fn both_lanes_updates_width_and_returns_direct_error() {
        let mut s = make_state();
        // left=−0.20, right=+0.20 → width=0.40, center=0.0, error=0.0
        let (err, conf) = extrapolate_center(Some(-0.20), Some(0.20), 0.90, &mut s);
        assert!(err.abs() < 1e-9, "err={err}");
        assert!((conf - 0.90).abs() < 1e-9, "conf={conf}");
        // Width should have been recorded.
        assert!(s.lane_width_estimate().is_some());
    }

    #[test]
    fn lane_width_decays_after_decay_start() {
        let mut s = make_state();
        s.update_lane_width(0.40);
        // Advance past DECAY_START but less than a half-life extra.
        s.tick_count = DECAY_START_TICKS + DECAY_HALF_LIFE_TICKS; // captured_at=0 so extra=300
        let w = s.lane_width_estimate().expect("should still have estimate");
        // After one half-life the estimate is halved: 0.40/2 = 0.20
        assert!((w - 0.20).abs() < 0.01, "w={w}");
    }

    #[test]
    fn lane_width_discarded_after_long_decay() {
        let mut s = make_state();
        s.update_lane_width(0.12); // just above MIN_VALID_WIDTH
        // Advance far enough that decay collapses below MIN_VALID_WIDTH.
        s.tick_count = DECAY_START_TICKS + DECAY_HALF_LIFE_TICKS * 4;
        // 0.12 / 2^4 = 0.12/16 = 0.0075 < 0.10 → None
        assert!(s.lane_width_estimate().is_none());
    }

    #[test]
    fn no_estimate_returns_zero_when_single_lane() {
        let mut s = make_state();
        // No width seeded; only right lane visible → should return (0.0, 0.0)
        let (err, conf) = extrapolate_center(None, Some(0.30), 0.75, &mut s);
        assert!(err.abs() < 1e-9, "err={err}");
        assert!(conf.abs() < 1e-9, "conf={conf}");
    }

    #[test]
    fn width_within_valid_bounds_accepted() {
        let mut s = make_state();
        s.update_lane_width(WIDTH_MIN);
        assert!(s.lane_width_estimate().is_some());
        let mut s2 = make_state();
        s2.update_lane_width(WIDTH_MAX);
        assert!(s2.lane_width_estimate().is_some());
    }

    #[test]
    fn width_out_of_bounds_rejected() {
        let mut s = make_state();
        s.update_lane_width(0.05); // below WIDTH_MIN
        assert!(s.lane_width_estimate().is_none());
        s.update_lane_width(0.90); // above WIDTH_MAX
        assert!(s.lane_width_estimate().is_none());
    }

    #[test]
    fn width_ema_blending() {
        let mut s = make_state();
        s.update_lane_width(0.40); // first sample → estimate = 0.40
        s.update_lane_width(0.60); // second: 0.7*0.60 + 0.3*0.40 = 0.42+0.12 = 0.54
        let w = s.lane_width_estimate().unwrap();
        assert!((w - 0.54).abs() < 1e-9, "w={w}");
    }
}
