//! Adaptive cruise control (ACC) distance-based limiter.

use crate::controller::Pid;

/// Distance-based adaptive cruise controller.
///
/// Wraps a [`Pid`] and turns a measured following-distance into a speed
/// cap (km/h). When the lead vehicle is far away or absent, the cap is
/// `f32::MAX` (i.e. "no cap").
pub struct AccController {
    /// Target following distance in meters.
    pub target_distance_m: f32,
    pid: Pid,
}

impl AccController {
    /// Construct a new ACC controller.
    ///
    /// `target_distance_m` is clamped to `>= 1.0 m`. The PID is
    /// preconfigured with sensible integral/output limits for typical
    /// truck dynamics.
    pub fn new(target_distance_m: f32, kp: f64, ki: f64, kd: f64) -> Self {
        let safe_target = if target_distance_m.is_finite() {
            target_distance_m.max(1.0)
        } else {
            1.0
        };
        Self {
            target_distance_m: safe_target,
            pid: Pid::new(kp, ki, kd, 100.0, 120.0),
        }
    }

    /// Returns a speed cap in km/h. `f32::MAX` means "no ACC cap".
    pub fn update(
        &mut self,
        current_distance: Option<f32>,
        current_speed_kmh: f32,
        dt: f32,
    ) -> f32 {
        let Some(distance) = current_distance.filter(|d| d.is_finite()) else {
            self.pid.reset();
            return f32::MAX;
        };

        if distance >= self.target_distance_m {
            self.pid.reset();
            return f32::MAX;
        }

        if !current_speed_kmh.is_finite() {
            self.pid.reset();
            return 0.0;
        }

        let error = f64::from(distance - self.target_distance_m);
        let correction = self.pid.update(error, f64::from(dt.max(0.001))) as f32;
        (current_speed_kmh + correction).clamp(0.0, current_speed_kmh.max(0.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_acc_controller() {
        let mut acc = AccController::new(50.0, 0.8, 0.0, 0.0);

        let no_limit_far = acc.update(Some(100.0), 80.0, 0.1);
        assert_eq!(no_limit_far, f32::MAX);

        let limit_close = acc.update(Some(30.0), 80.0, 0.1);
        assert!(
            limit_close < 80.0,
            "expected ACC limit below current speed, got {limit_close}"
        );

        let no_lead = acc.update(None, 80.0, 0.1);
        assert_eq!(no_lead, f32::MAX);
    }
}
