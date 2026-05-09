//! ACC plugin — Adaptive Cruise Control (distance-based speed cap).
//!
//! Reads lead-vehicle distance from telemetry, computes a speed cap via PID,
//! and writes it to `acc.speed_cap_kmh` on the blackboard.
//! Speed-controller reads this cap.
//!
//! ACC does not produce a `ControlRequest` — it influences throttle/brake
//! indirectly via the blackboard, which the speed-controller consults
//! before submitting *its* request to the arbitrator. The default
//! `tick_request` (returns `None`) is therefore correct as-is.

use truckpilot_plugin_api::{pid::Pid, ControlOutput, Plugin, PluginContext, Telemetry};

const DEFAULT_TARGET_DIST_M: f32 = 50.0;

pub struct AccPlugin {
    pid: Pid,
    target_distance_m: f32,
}

impl Default for AccPlugin {
    fn default() -> Self {
        Self {
            pid: Pid::new(0.8, 0.02, 0.2, 100.0, 120.0),
            target_distance_m: DEFAULT_TARGET_DIST_M,
        }
    }
}

impl Plugin for AccPlugin {
    fn name(&self) -> &str {
        "acc"
    }
    fn version(&self) -> &str {
        "0.1.0"
    }
    fn settings_schema(&self) -> &str {
        r#"{"type":"object","properties":{"target_distance_m":{"type":"number","minimum":5},"kp":{"type":"number"},"ki":{"type":"number"},"kd":{"type":"number"}}}"#
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        if let Some(d) = ctx.blackboard.get_f64("acc.target_distance_m") {
            self.target_distance_m = d as f32;
        }
        tracing::info!("[acc] loaded — target_dist={:.0}m", self.target_distance_m);
        ctx.blackboard.remove("acc.speed_cap_kmh");
    }

    fn on_unload(&mut self) {
        tracing::info!("[acc] unloaded");
    }

    fn tick(
        &mut self,
        telemetry: Option<&Telemetry>,
        _output: &mut ControlOutput,
        ctx: &PluginContext,
    ) {
        let Some(t) = telemetry else {
            ctx.blackboard.remove("acc.speed_cap_kmh");
            return;
        };

        let distance = if t.lead_vehicle_distance_m >= 0.0 {
            Some(t.lead_vehicle_distance_m)
        } else {
            // Proxy from longitudinal deceleration
            proxy_distance(t.accel_longitudinal, self.target_distance_m)
        };

        let Some(dist) = distance else {
            // No lead vehicle — remove cap
            ctx.blackboard.remove("acc.speed_cap_kmh");
            self.pid.reset();
            return;
        };

        if dist >= self.target_distance_m {
            ctx.blackboard.remove("acc.speed_cap_kmh");
            self.pid.reset();
            return;
        }

        let current_speed_kmh = (t.speed_ms * 3.6) as f32;
        let error = f64::from(dist - self.target_distance_m);
        let correction = self.pid.update(error, 0.02) as f32;
        let cap = (current_speed_kmh + correction).clamp(0.0, current_speed_kmh.max(0.0));

        ctx.blackboard.set("acc.speed_cap_kmh", cap.to_string());
        tracing::debug!("[acc] dist={dist:.1}m cap={cap:.1}km/h");
    }
}

fn proxy_distance(accel: f32, target: f32) -> Option<f32> {
    if accel < -0.2 {
        let projected = 0.5 * accel * 2.0_f32 * 2.0_f32;
        Some((target + projected).max(5.0))
    } else {
        None
    }
}

truckpilot_plugin_api::export_plugin!(AccPlugin);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn proxy_distance_negative_accel() {
        let d = proxy_distance(-1.0, 50.0);
        assert!(d.is_some());
        assert!(d.unwrap() < 50.0);
    }

    #[test]
    fn proxy_distance_positive_accel_returns_none() {
        assert!(proxy_distance(0.5, 50.0).is_none());
    }
}
