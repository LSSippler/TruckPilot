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

        (p + i + d).clamp(-self.output_limit, self.output_limit)
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.first_update = true;
    }
}
