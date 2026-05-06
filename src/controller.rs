//! PID controllers for speed and steering regulation.

// ---------------------------------------------------------------------------
// Generic PID controller
// ---------------------------------------------------------------------------

/// Proportional-Integral-Derivative controller with anti-windup.
///
/// Output is clamped to `[-output_limit, output_limit]`. The caller
/// interprets positive/negative output as appropriate (throttle vs brake,
/// or steer-left vs steer-right).
#[derive(Debug, Clone)]
pub struct Pid {
    kp: f64,
    ki: f64,
    kd: f64,
    integral_limit: f64,
    output_limit: f64,
    integral: f64,
    prev_error: f64,
    first_update: bool,
}

impl Pid {
    /// Construct a new PID controller with the given gains and limits.
    ///
    /// `integral_limit` clamps the running integral to avoid windup,
    /// `output_limit` clamps the final control signal.
    pub fn new(kp: f64, ki: f64, kd: f64, integral_limit: f64, output_limit: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral_limit,
            output_limit,
            integral: 0.0,
            prev_error: 0.0,
            first_update: true,
        }
    }

    /// Compute the control signal for a given error and time step.
    pub fn update(&mut self, error: f64, dt: f64) -> f64 {
        // Proportional.
        let p = self.kp * error;

        // Integral with anti-windup.
        self.integral += error * dt;
        self.integral = self
            .integral
            .clamp(-self.integral_limit, self.integral_limit);
        let i = self.ki * self.integral;

        // Derivative (clamp dt to avoid spike on first call).
        let safe_dt = dt.max(0.001);
        let derivative = if self.first_update {
            self.first_update = false;
            0.0
        } else {
            (error - self.prev_error) / safe_dt
        };
        self.prev_error = error;
        let d = self.kd * derivative;

        let output = p + i + d;
        output.clamp(-self.output_limit, self.output_limit)
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.first_update = true;
    }

    /// Access the current integral term (for testing).
    pub fn integral(&self) -> f64 {
        self.integral
    }
}

// ---------------------------------------------------------------------------
// Speed controller — wraps Pid with throttle/brake splitting
// ---------------------------------------------------------------------------

/// Adaptive speed controller using PID regulation.
///
/// Maintains a target speed by computing throttle and brake outputs
/// based on the difference between current and desired speed.
///
/// This is now a thin wrapper around `Pid` that splits the output
/// into positive (throttle) and negative (brake) ranges.
pub struct SpeedController {
    pid: Pid,
}

impl SpeedController {
    /// Create a speed controller with the given PID gains.
    pub fn new(kp: f64, ki: f64, kd: f64, integral_limit: f64) -> Self {
        Self {
            pid: Pid::new(kp, ki, kd, integral_limit, f64::MAX),
        }
    }

    /// Create a default controller tuned for ETS2 speed regulation.
    pub fn default_tuning() -> Self {
        Self::new(0.5, 0.1, 0.05, 10.0)
    }

    /// Create from config values.
    pub fn from_config(kp: f64, ki: f64, kd: f64, integral_limit: f64) -> Self {
        Self::new(kp, ki, kd, integral_limit)
    }

    /// Update with current and target speed (m/s).
    /// Returns `(throttle, brake)` each in [0.0, 1.0].
    pub fn update(&mut self, target: f64, current: f64, dt: f64) -> (f64, f64) {
        let error = target - current;
        let output = self.pid.update(error, dt);
        let throttle = output.clamp(0.0, 1.0);
        let brake = (-output).clamp(0.0, 1.0);
        (throttle, brake)
    }

    /// Reset internal PID state.
    pub fn reset(&mut self) {
        self.pid.reset();
    }
}

// ---------------------------------------------------------------------------
// Steering controller — wraps Pid with angular error normalization
// ---------------------------------------------------------------------------

/// Steering controller using PID regulation.
///
/// The error is an angular difference in radians, normalized to [-PI, PI].
/// Output is the desired steering signal in [-1.0, 1.0].
pub struct SteerController {
    pid: Pid,
}

impl SteerController {
    /// Create a steering controller with the given PID gains.
    pub fn new(kp: f64, ki: f64, kd: f64, integral_limit: f64) -> Self {
        Self {
            pid: Pid::new(kp, ki, kd, integral_limit, 1.0),
        }
    }

    /// Create from config values.
    pub fn from_config(kp: f64, ki: f64, kd: f64, integral_limit: f64) -> Self {
        Self::new(kp, ki, kd, integral_limit)
    }

    /// Default tuning for ETS2 steering.
    pub fn default_tuning() -> Self {
        Self::new(1.5, 0.08, 0.3, 2.0)
    }

    /// Compute steering signal from a heading error (radians).
    /// The caller must normalize the error to [-PI, PI] before calling.
    pub fn update(&mut self, heading_error: f64, dt: f64) -> f64 {
        self.pid.update(heading_error, dt)
    }

    /// Reset internal PID state.
    pub fn reset(&mut self) {
        self.pid.reset();
    }

    /// Access the current integral term (for testing).
    pub fn integral(&self) -> f64 {
        self.pid.integral()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- Pid ---

    #[test]
    fn test_pid_p_only() {
        let mut pid = Pid::new(2.0, 0.0, 0.0, 0.0, 10.0);
        assert_eq!(pid.update(3.0, 0.1), 6.0);
    }

    #[test]
    fn test_pid_integral_accumulates() {
        let mut pid = Pid::new(0.0, 1.0, 0.0, 100.0, 10.0);
        let _ = pid.update(2.0, 0.1); // integral += 0.2
        let out = pid.update(2.0, 0.1);
        assert!((out - 0.4).abs() < 0.01); // 0 + 1.0*0.4
    }

    #[test]
    fn test_pid_integral_clamped() {
        let mut pid = Pid::new(0.0, 1.0, 0.0, 0.5, 10.0);
        for _ in 0..100 {
            pid.update(10.0, 0.1);
        }
        assert!(pid.integral().abs() <= 0.5);
    }

    #[test]
    fn test_pid_first_update_has_no_derivative_kick() {
        let mut pid = Pid::new(0.0, 0.0, 1.0, 10.0, 10.0);

        // First update: derivative must be forced to 0.
        let first = pid.update(5.0, 0.1);
        assert!(
            (first - 0.0).abs() < 1e-9,
            "expected no derivative kick on first update, got {first}"
        );

        // Second update with changed error: derivative should now react.
        let second = pid.update(6.0, 0.1);
        assert!(
            second.abs() > 0.0,
            "expected derivative response after first update"
        );

        // After reset: first update again should have no derivative kick.
        pid.reset();
        let after_reset = pid.update(3.0, 0.1);
        assert!(
            (after_reset - 0.0).abs() < 1e-9,
            "expected no derivative kick after reset, got {after_reset}"
        );
    }

    // --- SpeedController ---

    #[test]
    fn test_speed_below_target_accelerates() {
        let mut sc = SpeedController::new(1.0, 0.0, 0.0, 0.0);
        let (throttle, brake) = sc.update(30.0, 20.0, 0.1);
        assert!(throttle > 0.0);
        assert_eq!(brake, 0.0);
    }

    #[test]
    fn test_speed_above_target_brakes() {
        let mut sc = SpeedController::new(1.0, 0.0, 0.0, 0.0);
        let (throttle, brake) = sc.update(20.0, 30.0, 0.1);
        assert!(brake > 0.0);
        assert_eq!(throttle, 0.0);
    }

    #[test]
    fn test_speed_clamped_output() {
        let mut sc = SpeedController::new(10.0, 0.0, 0.0, 0.0);
        let (throttle, _) = sc.update(100.0, 0.0, 0.1);
        assert!(throttle <= 1.0);
        assert!(throttle > 0.0);
    }

    #[test]
    fn test_speed_anti_windup() {
        let mut sc = SpeedController::new(0.0, 1.0, 0.0, 5.0);
        for _ in 0..100 {
            sc.update(50.0, 0.0, 0.1);
        }
        assert!(sc.pid.integral().abs() <= 5.0);
    }

    #[test]
    fn test_speed_reset() {
        let mut sc = SpeedController::new(0.0, 1.0, 0.0, 10.0);
        sc.update(50.0, 0.0, 0.1);
        assert!(sc.pid.integral().abs() > 0.0);
        sc.reset();
        assert_eq!(sc.pid.integral(), 0.0);
    }

    #[test]
    fn test_speed_at_target() {
        let mut sc = SpeedController::new(1.0, 0.1, 0.05, 10.0);
        let (throttle, brake) = sc.update(30.0, 30.0, 0.1);
        assert!(throttle.abs() < 1e-9);
        assert!(brake.abs() < 1e-9);
    }

    #[test]
    fn test_speed_dt_zero() {
        let mut sc = SpeedController::new(1.0, 0.0, 0.05, 10.0);
        let (throttle, brake) = sc.update(30.0, 20.0, 0.0);
        assert!(throttle.is_finite());
        assert!(brake.is_finite());
        assert!((0.0..=1.0).contains(&throttle));
        assert!((0.0..=1.0).contains(&brake));
    }

    // --- SteerController ---

    #[test]
    fn test_steer_zero_error() {
        let mut sc = SteerController::default_tuning();
        let s = sc.update(0.0, 0.02);
        assert!((s - 0.0).abs() < 0.001);
    }

    #[test]
    fn test_steer_positive_error() {
        let mut sc = SteerController::new(1.0, 0.0, 0.0, 0.0);
        let s = sc.update(0.5, 0.02);
        assert!(s > 0.0);
    }

    #[test]
    fn test_steer_clamped() {
        let mut sc = SteerController::new(5.0, 0.0, 0.0, 0.0);
        let s = sc.update(std::f64::consts::PI, 0.02);
        assert!(s <= 1.0);
        assert!(s >= -1.0);
    }
}
