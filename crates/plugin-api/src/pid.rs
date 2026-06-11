//! Shared PID controller — used by lane-keeper, speed-controller, acc.

/// Proportional-Integral-Derivative controller with anti-windup.
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

    // ── Diagnostics (read-only; recorded each `update`, no control effect) ──
    /// Last proportional term `kp * error`.
    last_p: f64,
    /// Last integral term `ki * integral`.
    last_i: f64,
    /// Last derivative term `kd * d(error)/dt`.
    last_d: f64,
    /// Last summed output BEFORE the `output_limit` clamp (`p + i + d`).
    last_unclamped: f64,
}

impl Pid {
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
            last_p: 0.0,
            last_i: 0.0,
            last_d: 0.0,
            last_unclamped: 0.0,
        }
    }

    pub fn update(&mut self, error: f64, dt: f64) -> f64 {
        let p = self.kp * error;

        self.integral =
            (self.integral + error * dt).clamp(-self.integral_limit, self.integral_limit);
        let i = self.ki * self.integral;

        let safe_dt = dt.max(0.001);
        let d = if self.first_update {
            self.first_update = false;
            0.0
        } else {
            self.kd * (error - self.prev_error) / safe_dt
        };
        self.prev_error = error;

        let unclamped = p + i + d;
        // Record terms for read-only diagnostics — does not affect the returned value.
        self.last_p = p;
        self.last_i = i;
        self.last_d = d;
        self.last_unclamped = unclamped;

        unclamped.clamp(-self.output_limit, self.output_limit)
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.first_update = true;
    }

    // ── Read-only diagnostic accessors (last `update` term breakdown) ──
    /// Proportional term from the most recent `update`.
    pub fn last_p(&self) -> f64 {
        self.last_p
    }
    /// Integral term from the most recent `update`.
    pub fn last_i(&self) -> f64 {
        self.last_i
    }
    /// Derivative term from the most recent `update`.
    pub fn last_d(&self) -> f64 {
        self.last_d
    }
    /// Summed output before the `output_limit` clamp (`p + i + d`).
    pub fn last_unclamped(&self) -> f64 {
        self.last_unclamped
    }
    /// True if the most recent `update` hit the `output_limit` clamp.
    pub fn last_output_clamped(&self) -> bool {
        self.last_unclamped.abs() > self.output_limit + 1e-12
    }

    pub fn set_kp(&mut self, kp: f64) {
        self.kp = kp;
    }
    pub fn set_ki(&mut self, ki: f64) {
        self.ki = ki;
    }
    pub fn set_kd(&mut self, kd: f64) {
        self.kd = kd;
    }
}
