//! Speed-Controller plugin — PID-based throttle/brake regulation.
//!
//! Target speed = min(cruise_control, nav_speed_limit, acc.speed_cap_kmh, sign.speed_limit_kmh)
//! Writes `output.throttle` and `output.brake`.

use truckpilot_plugin_api::{
    pid::Pid, ControlOutput, ControlRequest, Plugin, PluginContext, Telemetry,
};

/// Arbitration priority for the speed-controller. Ordinary autopilot.
const PRIORITY_NORMAL: i32 = 50;

const FALLBACK_SPEED_KMH: f64 = 80.0;

pub struct SpeedControllerPlugin {
    pid: Pid,
}

impl Default for SpeedControllerPlugin {
    fn default() -> Self {
        Self {
            pid: Pid::new(0.5, 0.1, 0.05, 10.0, f64::MAX),
        }
    }
}

impl Plugin for SpeedControllerPlugin {
    fn name(&self) -> &str {
        "speed-controller"
    }
    fn version(&self) -> &str {
        "0.1.0"
    }
    fn settings_schema(&self) -> &str {
        r#"{"type":"object","properties":{"kp":{"type":"number"},"ki":{"type":"number"},"kd":{"type":"number"},"fallback_speed_kmh":{"type":"number"}}}"#
    }

    fn on_load(&mut self, _ctx: &PluginContext) {
        tracing::info!("[speed-controller] loaded");
    }

    fn on_unload(&mut self) {
        tracing::info!("[speed-controller] unloaded");
    }

    fn tick(
        &mut self,
        _telemetry: Option<&Telemetry>,
        _output: &mut ControlOutput,
        _ctx: &PluginContext,
    ) {
        // Migrated to `tick_request` (Fix 4): throttle/brake are
        // contributed via the arbitrator, not written into the
        // shared `ControlOutput` last-writer-wins style.
    }

    fn tick_request(
        &mut self,
        telemetry: Option<&Telemetry>,
        ctx: &PluginContext,
    ) -> Option<ControlRequest> {
        let t = telemetry?;

        if t.engine_rpm < 100.0 || t.cruise_control_kmh <= 0.0 {
            self.pid.reset();
            return Some(ControlRequest {
                throttle: Some(0.0),
                brake: Some(0.0),
                priority: PRIORITY_NORMAL,
                ..Default::default()
            });
        }

        let target_kmh = compute_target_speed(t, ctx);
        let current_ms = t.speed_ms;
        let target_ms = target_kmh / 3.6;

        let raw = self.pid.update(target_ms - current_ms, 0.02);
        let throttle = raw.clamp(0.0, 1.0);
        let brake = (-raw).clamp(0.0, 1.0);

        tracing::debug!(
            "[speed-ctrl] target={:.1} current={:.1} thr={:.2} brk={:.2}",
            target_kmh,
            current_ms * 3.6,
            throttle,
            brake
        );

        Some(ControlRequest {
            throttle: Some(throttle),
            brake: Some(brake),
            priority: PRIORITY_NORMAL,
            ..Default::default()
        })
    }
}

fn compute_target_speed(t: &Telemetry, ctx: &PluginContext) -> f64 {
    let mut target = t.cruise_control_kmh;

    // Navigation speed limit
    if t.nav_speed_limit_kmh > 0.0 {
        target = target.min(t.nav_speed_limit_kmh);
    } else {
        target = target.min(FALLBACK_SPEED_KMH);
    }

    // ACC cap
    if let Some(acc_cap) = ctx.blackboard.get_f64("acc.speed_cap_kmh") {
        target = target.min(acc_cap);
    }

    // Sign speed limit
    if let Some(sign_limit) = ctx.blackboard.get_f64("sign.speed_limit_kmh") {
        target = target.min(sign_limit);
    }

    target.max(0.0)
}

truckpilot_plugin_api::export_plugin!(SpeedControllerPlugin);

#[cfg(test)]
mod tests {
    use super::*;
    use truckpilot_plugin_api::SharedBlackboard;

    fn make_telemetry(speed_ms: f64, cruise_kmh: f64, nav_limit: f64) -> Telemetry {
        Telemetry {
            position: [0.0; 3],
            heading: 0.0,
            pitch: 0.0,
            roll: 0.0,
            speed_ms,
            engine_rpm: 1200.0,
            cruise_control_kmh: cruise_kmh,
            nav_speed_limit_kmh: nav_limit,
            lead_vehicle_distance_m: -1.0,
            accel_longitudinal: -1.0,
        }
    }

    #[test]
    fn target_limited_by_nav_limit() {
        let t = make_telemetry(20.0, 100.0, 80.0);
        let bb = SharedBlackboard::new();
        let ctx = PluginContext::new("test", bb);
        let target = compute_target_speed(&t, &ctx);
        assert!((target - 80.0).abs() < 0.01);
    }

    #[test]
    fn acc_cap_further_limits_speed() {
        let t = make_telemetry(20.0, 100.0, 80.0);
        let bb = SharedBlackboard::new();
        bb.set("acc.speed_cap_kmh", "50.0");
        let ctx = PluginContext::new("test", bb);
        let target = compute_target_speed(&t, &ctx);
        assert!((target - 50.0).abs() < 0.01);
    }

    #[test]
    fn no_nav_limit_uses_fallback() {
        let t = make_telemetry(20.0, 120.0, -1.0);
        let bb = SharedBlackboard::new();
        let ctx = PluginContext::new("test", bb);
        let target = compute_target_speed(&t, &ctx);
        assert!((target - FALLBACK_SPEED_KMH).abs() < 0.01);
    }
}
