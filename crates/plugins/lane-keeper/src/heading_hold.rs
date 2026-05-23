//! Level-3 heading-hold: PID on a captured inertial heading when lane vision is lost.

/// Number of ticks over which to blend from the entry steering to the heading-hold PID output.
const BLEND_TICKS: u64 = 15;
/// Gain factor applied to the heading-error PID output (per DS1 spec §3 L3).
const HEADING_HOLD_GAIN: f64 = 0.50;

/// State for the Level-3 heading-hold sub-controller.
pub struct HeadingHoldState {
    /// Heading captured when Level-3 was entered (radians, inertial frame).
    pub hold_heading: f64,
    /// Tick at which Level-3 was entered.
    hold_entered_at_tick: u64,
    /// Steering value at the moment Level-3 was entered (used for blending).
    hold_entry_steering: f64,
    /// Whether the heading-hold controller is currently active.
    pub active: bool,
}

impl Default for HeadingHoldState {
    fn default() -> Self {
        Self {
            hold_heading: 0.0,
            hold_entered_at_tick: 0,
            hold_entry_steering: 0.0,
            active: false,
        }
    }
}

impl HeadingHoldState {
    pub fn new() -> Self {
        Self::default()
    }

    /// Activate heading-hold, capturing the reference heading and entry steering.
    pub fn enter(&mut self, heading: f64, tick_count: u64, entry_steering: f64) {
        self.hold_heading = heading;
        self.hold_entered_at_tick = tick_count;
        self.hold_entry_steering = entry_steering;
        self.active = true;
    }

    /// Deactivate heading-hold (called when transitioning away from Level-3).
    pub fn exit(&mut self) {
        self.active = false;
    }

    /// Compute the Level-3 steering output.
    ///
    /// `current_heading` — current truck heading in radians (inertial frame).
    /// `tick_count`      — current monotonic tick index.
    /// `pid_fn`          — closure that accepts `(error, dt)` and returns raw PID output;
    ///                     callers pass a closure wrapping their `Pid::update()`.
    /// `dt`              — time delta in seconds (typically 1/50 = 0.02 s).
    ///
    /// Returns the final clamped steering value in [-1, +1].
    pub fn compute_steering(
        &self,
        current_heading: f64,
        tick_count: u64,
        pid_fn: &mut impl FnMut(f64, f64) -> f64,
        dt: f64,
    ) -> f64 {
        debug_assert!(self.active, "compute_steering called while not active");

        // Wrap heading error to (−π, +π].
        let raw_err = self.hold_heading - current_heading;
        let heading_err = wrap_angle(raw_err);

        let raw_pid = pid_fn(heading_err, dt);
        let pid_output = (raw_pid * HEADING_HOLD_GAIN).clamp(-1.0, 1.0);

        // Linear blend from entry steering over BLEND_TICKS.
        let elapsed = tick_count.saturating_sub(self.hold_entered_at_tick);
        if elapsed < BLEND_TICKS {
            let t = elapsed as f64 / BLEND_TICKS as f64; // 0.0 → 1.0
            lerp(self.hold_entry_steering, pid_output, t)
        } else {
            pid_output
        }
    }

    /// Returns how long the heading-hold has been active, in ticks.
    pub fn ticks_active(&self, tick_count: u64) -> u64 {
        tick_count.saturating_sub(self.hold_entered_at_tick)
    }

    /// Drift between the currently held heading and the current heading (absolute, radians).
    pub fn heading_drift(&self, current_heading: f64) -> f64 {
        wrap_angle(self.hold_heading - current_heading).abs()
    }
}

/// Wrap an angle in radians to (−π, +π].
#[inline]
pub fn wrap_angle(mut a: f64) -> f64 {
    use std::f64::consts::PI;
    while a > PI { a -= 2.0 * PI; }
    while a <= -PI { a += 2.0 * PI; }
    a
}

/// Linear interpolation.
#[inline]
fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn dummy_pid(gain: f64) -> impl FnMut(f64, f64) -> f64 {
        move |err, _dt| err * gain
    }

    #[test]
    fn enter_activates_hold() {
        let mut h = HeadingHoldState::new();
        assert!(!h.active);
        h.enter(1.0, 100, 0.3);
        assert!(h.active);
        assert!((h.hold_heading - 1.0).abs() < 1e-9);
    }

    #[test]
    fn exit_deactivates() {
        let mut h = HeadingHoldState::new();
        h.enter(1.0, 0, 0.0);
        h.exit();
        assert!(!h.active);
    }

    #[test]
    fn no_drift_when_on_heading() {
        let mut h = HeadingHoldState::new();
        h.enter(1.0, 200, 0.0);
        // After blend period, error=0 → pid=0 → steering=0
        let s = h.compute_steering(1.0, 200 + BLEND_TICKS, &mut dummy_pid(1.0), 0.02);
        assert!(s.abs() < 1e-9, "s={s}");
    }

    #[test]
    fn heading_error_produces_steering() {
        let mut h = HeadingHoldState::new();
        h.enter(0.0, 0, 0.0); // hold heading 0 rad
        // After blend period; current heading = +0.5 rad (truck drifted right)
        // heading_err = 0.0 - 0.5 = -0.5 → pid raw = -0.5 * gain
        // steering = clamp(-0.5 * 1.0 * 0.5, -1, 1) = -0.25 (steer left)
        let s = h.compute_steering(0.5, BLEND_TICKS, &mut dummy_pid(1.0), 0.02);
        assert!(s < 0.0, "expected steer left, got {s}");
        assert!((s - (-0.25)).abs() < 1e-9, "s={s}");
    }

    #[test]
    fn blending_at_entry() {
        let mut h = HeadingHoldState::new();
        h.enter(0.0, 0, 0.6); // entry steering = 0.6
        // At tick 0 (elapsed=0) → pure entry steering
        let s = h.compute_steering(0.0, 0, &mut dummy_pid(0.0), 0.02);
        assert!((s - 0.6).abs() < 1e-9, "s={s}");
    }

    #[test]
    fn blending_at_midpoint() {
        let mut h = HeadingHoldState::new();
        // entry steering 1.0, pid target 0.0 → midpoint should be 0.5
        h.enter(0.0, 0, 1.0);
        let mid = BLEND_TICKS / 2;
        let s = h.compute_steering(0.0, mid, &mut dummy_pid(0.0), 0.02);
        assert!((s - 0.5).abs() < 0.05, "s={s}"); // t ≈ 0.5 → lerp(1,0,0.5)=0.5
    }

    #[test]
    fn wrap_angle_normalises() {
        use std::f64::consts::PI;
        assert!((wrap_angle(PI + 0.01) - (-PI + 0.01)).abs() < 1e-9);
        assert!((wrap_angle(-PI - 0.01) - (PI - 0.01)).abs() < 1e-9);
        assert!((wrap_angle(0.0)).abs() < 1e-9);
    }

    #[test]
    fn heading_drift_absolute() {
        let mut h = HeadingHoldState::new();
        h.enter(1.0, 0, 0.0);
        assert!((h.heading_drift(1.5) - 0.5).abs() < 1e-9);
        assert!((h.heading_drift(0.5) - 0.5).abs() < 1e-9);
    }

    #[test]
    fn ticks_active_counts_correctly() {
        let mut h = HeadingHoldState::new();
        h.enter(0.0, 50, 0.0);
        assert_eq!(h.ticks_active(80), 30);
    }
}
