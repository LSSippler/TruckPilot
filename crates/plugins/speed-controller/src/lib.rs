//! Speed-Controller plugin — PID-based throttle/brake regulation.
//!
//! Target speed is the minimum of six sources (whichever are present):
//!   1. `cruise.target_kmh`        — UI cruise setpoint
//!   2. `t.nav_speed_limit_kmh`    — in-game nav speed limit
//!   3. `sign.speed_limit_kmh`     — map sign-reader
//!   4. `sign_vision.speed_limit_kmh` — vision fallback
//!   5. `acc.speed_cap_kmh`        — ACC follow-distance cap (conditional)
//!   6. `FALLBACK_SPEED_KMH` (80)  — when everything else is absent
//!
//! Writes `output.throttle` and `output.brake` via `tick_request`.
//!
//! Phase 6.2e:
//! - State-gated via `ctx.is_active()` (with PID-reset on exit).
//! - Dead-band ±1 km/h around target → coast (no oscillation).
//! - Bergab safety-override: `error_kmh <= -10` → `brake = 1.0`, bypass PID.
//! - PID gains tunable at runtime via `plugin.speed_controller.{kp,ki,kd}`.

use truckpilot_plugin_api::{
    pid::Pid, ControlOutput, ControlRequest, Plugin, PluginContext, Telemetry,
};

const PRIORITY_NORMAL: i32 = 50;
const FALLBACK_SPEED_KMH: f64 = 80.0;

const DEFAULT_KP: f64 = 0.25;
const DEFAULT_KI: f64 = 0.08;
const DEFAULT_KD: f64 = 0.06;
const INTEGRAL_LIMIT: f64 = 0.8;
const OUTPUT_LIMIT: f64 = 1.0;

const DEAD_BAND_KMH: f64 = 1.0;
const BERGAB_BRAKE_KMH: f64 = -10.0;

pub struct SpeedControllerPlugin {
    pid: Pid,
    last_gains: (f64, f64, f64),
}

impl Default for SpeedControllerPlugin {
    fn default() -> Self {
        Self {
            pid: Pid::new(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, INTEGRAL_LIMIT, OUTPUT_LIMIT),
            last_gains: (DEFAULT_KP, DEFAULT_KI, DEFAULT_KD),
        }
    }
}

impl SpeedControllerPlugin {
    fn apply_gain_overrides(&mut self, ctx: &PluginContext) {
        let kp = ctx
            .blackboard
            .get_f64("plugin.speed_controller.kp")
            .unwrap_or(DEFAULT_KP);
        let ki = ctx
            .blackboard
            .get_f64("plugin.speed_controller.ki")
            .unwrap_or(DEFAULT_KI);
        let kd = ctx
            .blackboard
            .get_f64("plugin.speed_controller.kd")
            .unwrap_or(DEFAULT_KD);
        let next = (kp, ki, kd);
        if next != self.last_gains {
            self.pid.set_kp(kp);
            self.pid.set_ki(ki);
            self.pid.set_kd(kd);
            self.last_gains = next;
            tracing::info!("[speed-ctrl] gains updated kp={kp} ki={ki} kd={kd}");
            ctx.blackboard.set("pid_tuning.speed_controller.kp", kp.to_string());
            ctx.blackboard.set("pid_tuning.speed_controller.ki", ki.to_string());
            ctx.blackboard.set("pid_tuning.speed_controller.kd", kd.to_string());
        }
    }
}

impl Plugin for SpeedControllerPlugin {
    fn name(&self) -> &str { "speed-controller" }
    fn version(&self) -> &str { "0.2.0" }
    fn settings_schema(&self) -> &str {
        r#"{"type":"object","properties":{"kp":{"type":"number"},"ki":{"type":"number"},"kd":{"type":"number"}}}"#
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
        // Throttle/brake contributed via tick_request.
    }

    fn tick_request(
        &mut self,
        telemetry: Option<&Telemetry>,
        ctx: &PluginContext,
    ) -> Option<ControlRequest> {
        if !ctx.is_active() {
            self.pid.reset();
            return None;
        }

        self.apply_gain_overrides(ctx);

        let Some(t) = telemetry else {
            self.pid.reset();
            return None;
        };

        if t.engine_rpm < 100.0 {
            self.pid.reset();
            return Some(ControlRequest {
                throttle: Some(0.0),
                brake: Some(0.0),
                priority: PRIORITY_NORMAL,
                ..Default::default()
            });
        }

        let target_kmh = compute_target_speed(t, ctx);
        let current_kmh = t.speed_ms * 3.6;
        let error_kmh = target_kmh - current_kmh;

        // Bergab safety-override: strongly above target → full brake.
        if error_kmh <= BERGAB_BRAKE_KMH {
            self.pid.reset();
            return Some(ControlRequest {
                throttle: Some(0.0),
                brake: Some(1.0),
                priority: PRIORITY_NORMAL,
                ..Default::default()
            });
        }

        // Dead-band: coast inside ±1 km/h.
        if error_kmh.abs() < DEAD_BAND_KMH {
            return Some(ControlRequest {
                throttle: Some(0.0),
                brake: Some(0.0),
                priority: PRIORITY_NORMAL,
                ..Default::default()
            });
        }

        let target_ms = target_kmh / 3.6;
        let error_ms = target_ms - t.speed_ms;
        let dt = ctx.dt_s.min(0.1);
        let raw = self.pid.update(error_ms, dt);

        let (throttle, brake) = if raw > 0.0 {
            (raw.min(1.0), 0.0)
        } else {
            (0.0, (-raw).min(1.0))
        };

        tracing::debug!(
            "[speed-ctrl] target={target_kmh:.1} current={current_kmh:.1} err={error_kmh:+.2} thr={throttle:.2} brk={brake:.2}"
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
    let mut target = f64::INFINITY;
    let mut had_source = false;

    if let Some(v) = ctx.blackboard.get_f64("cruise.target_kmh") {
        if v > 0.0 {
            target = target.min(v);
            had_source = true;
        }
    } else if t.cruise_control_kmh > 0.0 {
        target = target.min(t.cruise_control_kmh);
        had_source = true;
    }

    if t.nav_speed_limit_kmh > 0.0 {
        target = target.min(t.nav_speed_limit_kmh);
        had_source = true;
    }

    if let Some(v) = ctx.blackboard.get_f64("sign.speed_limit_kmh") {
        if v > 0.0 {
            target = target.min(v);
            had_source = true;
        }
    }

    if let Some(v) = ctx.blackboard.get_f64("sign_vision.speed_limit_kmh") {
        if v > 0.0 {
            target = target.min(v);
            had_source = true;
        }
    }

    if let Some(v) = ctx.blackboard.get_f64("acc.speed_cap_kmh") {
        if v >= 0.0 {
            target = target.min(v);
            had_source = true;
        }
    }

    if !had_source {
        target = FALLBACK_SPEED_KMH;
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
            fuel_liters: -1.0,
            odometer_km: -1.0,
        }
    }

    fn active_ctx(bb: SharedBlackboard) -> PluginContext {
        bb.set("autopilot.state", "Active");
        PluginContext::new("speed-controller", bb)
    }

    fn off_ctx() -> PluginContext {
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", "Off");
        PluginContext::new("speed-controller", bb)
    }

    #[test]
    fn test_cruise_target_stable() {
        let t = make_telemetry(20.0, 90.0, -1.0);
        let ctx = active_ctx(SharedBlackboard::new());
        assert!((compute_target_speed(&t, &ctx) - 90.0).abs() < 0.01);
    }

    #[test]
    fn test_sign_limit_reduces_target() {
        let t = make_telemetry(20.0, 100.0, -1.0);
        let bb = SharedBlackboard::new();
        bb.set("sign.speed_limit_kmh", "60.0");
        let ctx = active_ctx(bb);
        assert!((compute_target_speed(&t, &ctx) - 60.0).abs() < 0.01);
    }

    #[test]
    fn test_acc_cap_when_present() {
        let t = make_telemetry(20.0, 100.0, 80.0);
        let bb = SharedBlackboard::new();
        bb.set("acc.speed_cap_kmh", "50.0");
        let ctx = active_ctx(bb);
        assert!((compute_target_speed(&t, &ctx) - 50.0).abs() < 0.01);
    }

    #[test]
    fn test_acc_cap_absent_path_a() {
        let t = make_telemetry(20.0, 100.0, 80.0);
        let ctx = active_ctx(SharedBlackboard::new());
        assert!((compute_target_speed(&t, &ctx) - 80.0).abs() < 0.01);
    }

    #[test]
    fn test_vision_fallback_when_map_absent() {
        let t = make_telemetry(20.0, 120.0, -1.0);
        let bb = SharedBlackboard::new();
        bb.set("sign_vision.speed_limit_kmh", "70.0");
        let ctx = active_ctx(bb);
        assert!((compute_target_speed(&t, &ctx) - 70.0).abs() < 0.01);
    }

    #[test]
    fn test_all_sources_absent_uses_fallback_80() {
        let t = Telemetry {
            position: [0.0; 3],
            heading: 0.0,
            pitch: 0.0,
            roll: 0.0,
            speed_ms: 0.0,
            engine_rpm: 1200.0,
            cruise_control_kmh: 0.0,
            nav_speed_limit_kmh: -1.0,
            lead_vehicle_distance_m: -1.0,
            accel_longitudinal: -1.0,
            fuel_liters: -1.0,
            odometer_km: -1.0,
        };
        let ctx = active_ctx(SharedBlackboard::new());
        assert!((compute_target_speed(&t, &ctx) - FALLBACK_SPEED_KMH).abs() < 0.01);
    }

    #[test]
    fn test_state_gate_returns_none_when_off() {
        let mut p = SpeedControllerPlugin::default();
        let t = make_telemetry(20.0, 80.0, -1.0);
        assert!(p.tick_request(Some(&t), &off_ctx()).is_none());
    }

    #[test]
    fn test_dead_band_no_oscillation() {
        let mut p = SpeedControllerPlugin::default();
        let t = make_telemetry(80.0 / 3.6, 80.0, -1.0);
        let ctx = active_ctx(SharedBlackboard::new());
        let req = p.tick_request(Some(&t), &ctx).unwrap();
        assert_eq!(req.throttle, Some(0.0));
        assert_eq!(req.brake, Some(0.0));
    }

    #[test]
    fn test_bergab_override_at_minus_10() {
        let mut p = SpeedControllerPlugin::default();
        let t = make_telemetry(100.0 / 3.6, 80.0, -1.0);
        let ctx = active_ctx(SharedBlackboard::new());
        let req = p.tick_request(Some(&t), &ctx).unwrap();
        assert_eq!(req.brake, Some(1.0));
        assert_eq!(req.throttle, Some(0.0));
    }

    #[test]
    fn engine_off_zeros_outputs() {
        let mut p = SpeedControllerPlugin::default();
        let mut t = make_telemetry(0.0, 80.0, -1.0);
        t.engine_rpm = 0.0;
        let ctx = active_ctx(SharedBlackboard::new());
        let req = p.tick_request(Some(&t), &ctx).unwrap();
        assert_eq!(req.throttle, Some(0.0));
        assert_eq!(req.brake, Some(0.0));
    }

    #[test]
    fn pid_throttles_up_when_below_target() {
        let mut p = SpeedControllerPlugin::default();
        let t = make_telemetry(50.0 / 3.6, 80.0, -1.0);
        let ctx = active_ctx(SharedBlackboard::new());
        let req = p.tick_request(Some(&t), &ctx).unwrap();
        assert!(req.throttle.unwrap() > 0.0);
        assert_eq!(req.brake, Some(0.0));
    }

    #[test]
    fn gain_overrides_take_effect() {
        let mut p = SpeedControllerPlugin::default();
        let bb = SharedBlackboard::new();
        bb.set("plugin.speed_controller.kp", "0.5");
        bb.set("plugin.speed_controller.ki", "0.0");
        bb.set("plugin.speed_controller.kd", "0.0");
        let ctx = active_ctx(bb);
        let t = make_telemetry(50.0 / 3.6, 80.0, -1.0);
        let _ = p.tick_request(Some(&t), &ctx);
        assert!((p.last_gains.0 - 0.5).abs() < 1e-9);
    }
}
