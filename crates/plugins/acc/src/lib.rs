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
    tick_seq: u64,
}

impl Default for AccPlugin {
    fn default() -> Self {
        Self {
            pid: Pid::new(0.8, 0.02, 0.2, 100.0, 120.0),
            target_distance_m: DEFAULT_TARGET_DIST_M,
            tick_seq: 0,
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
        self.tick_seq = self.tick_seq.wrapping_add(1);
        ctx.blackboard
            .set("acc.tick_seq", self.tick_seq.to_string());

        let Some(t) = telemetry else {
            ctx.blackboard.remove("acc.speed_cap_kmh");
            publish_acc_diag(ctx, false, None, false, false, 0.0, self.target_distance_m);
            return;
        };

        let (distance, from_telemetry, from_proxy) =
            if t.lead_vehicle_distance_m >= 0.0 {
                (Some(t.lead_vehicle_distance_m), true, false)
            } else {
                match proxy_distance(t.accel_longitudinal, self.target_distance_m) {
                    Some(d) => (Some(d), false, true),
                    None => (None, false, false),
                }
            };

        let Some(dist) = distance else {
            // No lead vehicle — remove cap
            ctx.blackboard.remove("acc.speed_cap_kmh");
            self.pid.reset();
            publish_acc_diag(ctx, false, None, false, false, 0.0, self.target_distance_m);
            return;
        };

        if dist >= self.target_distance_m {
            ctx.blackboard.remove("acc.speed_cap_kmh");
            self.pid.reset();
            publish_acc_diag(ctx, false, Some(dist), from_telemetry, from_proxy, 0.0, self.target_distance_m);
            return;
        }

        let current_speed_kmh = (t.speed_ms * 3.6) as f32;
        let error = f64::from(dist - self.target_distance_m);
        let correction = self.pid.update(error, ctx.dt_s) as f32;
        let cap = (current_speed_kmh + correction).clamp(0.0, current_speed_kmh.max(0.0));

        ctx.blackboard.set("acc.speed_cap_kmh", cap.to_string());
        publish_acc_diag(ctx, true, Some(dist), from_telemetry, from_proxy, cap, self.target_distance_m);
        tracing::debug!("[acc] dist={dist:.1}m cap={cap:.1}km/h");
    }
}

fn publish_acc_diag(
    ctx: &PluginContext,
    active: bool,
    lead_dist_m: Option<f32>,
    from_telemetry: bool,
    from_proxy: bool,
    cap_kmh: f32,
    target_distance_m: f32,
) {
    ctx.blackboard.set("acc.active", active.to_string());
    ctx.blackboard.set("acc.brake_cmd", "0.000");
    ctx.blackboard.set(
        "acc.throttle_cap_kmh",
        if active {
            format!("{cap_kmh:.1}")
        } else {
            String::new()
        },
    );
    ctx.blackboard.set(
        "acc.target_speed_ms",
        if active {
            format!("{:.3}", f64::from(cap_kmh) / 3.6)
        } else {
            String::new()
        },
    );
    ctx.blackboard.set(
        "acc.lead_vehicle_detected",
        lead_dist_m.is_some().to_string(),
    );
    ctx.blackboard.set(
        "acc.lead_vehicle_distance_m",
        lead_dist_m
            .map(|d| format!("{d:.1}"))
            .unwrap_or_else(|| "-1".to_string()),
    );
    ctx.blackboard.set(
        "acc.using_accel_proxy",
        from_proxy.to_string(),
    );
    ctx.blackboard.set(
        "acc.lead_from_telemetry",
        from_telemetry.to_string(),
    );
    ctx.blackboard.set(
        "acc.target_distance_m",
        format!("{target_distance_m:.0}"),
    );
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
