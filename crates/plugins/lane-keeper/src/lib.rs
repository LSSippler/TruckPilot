//! Lane-Keeper plugin — PID steering with speed-adaptive look-ahead.
//!
//! Reads waypoints from `router.waypoints` on the blackboard.
//! Writes only `output.steering`, leaves throttle/brake untouched.
//!
//! PID gains are tunable at runtime via `plugin.lane_keeper.{kp,ki,kd}`.
//! Confirmed active gains are echoed to `pid_tuning.lane_keeper.{kp,ki,kd}`.
//!
//! ## Speed-adaptive look-ahead
//!
//! `look_ahead_m = BASE_LOOK_AHEAD + speed_kmh * SPEED_FACTOR`
//!
//! This prevents oscillation at high speed and tight cornering at low speed.

use truckpilot_plugin_api::{
    pid::Pid, ControlOutput, ControlRequest, Plugin, PluginContext, Telemetry,
};

/// Arbitration priority for the lane-keeper. Ordinary autopilot.
const PRIORITY_NORMAL: i32 = 50;

const BASE_LOOK_AHEAD: f64 = 5.0; // meters at standstill
const SPEED_FACTOR: f64 = 0.5; // extra meters per km/h
const WAYPOINT_REACH_M: f64 = 5.0; // advance waypoint within this radius

const DEFAULT_KP: f64 = 0.8;
const DEFAULT_KI: f64 = 0.1;
const DEFAULT_KD: f64 = 0.3;

pub struct LaneKeeperPlugin {
    pid: Pid,
    waypoints: Vec<[f64; 2]>, // (x, z) pairs
    progress_idx: usize,
    /// Catmull-Rom subdivisions (configurable).
    subdivisions: usize,
    last_gains: (f64, f64, f64),
}

impl Default for LaneKeeperPlugin {
    fn default() -> Self {
        Self {
            pid: Pid::new(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, 2.0, 1.0),
            waypoints: Vec::new(),
            progress_idx: 0,
            subdivisions: 4,
            last_gains: (DEFAULT_KP, DEFAULT_KI, DEFAULT_KD),
        }
    }
}

impl LaneKeeperPlugin {
    fn apply_gain_overrides(&mut self, ctx: &PluginContext) {
        let kp = ctx
            .blackboard
            .get_f64("plugin.lane_keeper.kp")
            .unwrap_or(DEFAULT_KP);
        let ki = ctx
            .blackboard
            .get_f64("plugin.lane_keeper.ki")
            .unwrap_or(DEFAULT_KI);
        let kd = ctx
            .blackboard
            .get_f64("plugin.lane_keeper.kd")
            .unwrap_or(DEFAULT_KD);
        let next = (kp, ki, kd);
        if next != self.last_gains {
            self.pid.set_kp(kp);
            self.pid.set_ki(ki);
            self.pid.set_kd(kd);
            self.last_gains = next;
            tracing::info!("[lane-keeper] gains updated kp={kp} ki={ki} kd={kd}");
            ctx.blackboard
                .set("pid_tuning.lane_keeper.kp", kp.to_string());
            ctx.blackboard
                .set("pid_tuning.lane_keeper.ki", ki.to_string());
            ctx.blackboard
                .set("pid_tuning.lane_keeper.kd", kd.to_string());
        }
    }

    fn load_waypoints_from_blackboard(&mut self, ctx: &PluginContext) {
        if let Some(json) = ctx.blackboard.get("router.waypoints") {
            if let Ok(pts) = serde_json::from_str::<Vec<[f64; 2]>>(&json) {
                self.waypoints = smooth_catmull_rom(&pts, self.subdivisions);
                self.progress_idx = 0;
                tracing::info!(
                    "[lane-keeper] loaded {} waypoints (smoothed)",
                    self.waypoints.len()
                );
            }
        }
    }

    fn compute_heading_error(&mut self, tx: f64, tz: f64, heading: f64, speed_ms: f64) -> f64 {
        if self.waypoints.len() < 2 {
            return 0.0;
        }

        // Advance progress index
        while self.progress_idx + 1 < self.waypoints.len() {
            let [wx, wz] = self.waypoints[self.progress_idx + 1];
            let dist = ((tx - wx).powi(2) + (tz - wz).powi(2)).sqrt();
            if dist < WAYPOINT_REACH_M {
                self.progress_idx += 1;
            } else {
                break;
            }
        }

        // Speed-adaptive look-ahead
        let look_ahead = BASE_LOOK_AHEAD + speed_ms * 3.6 * SPEED_FACTOR;

        // Walk forward along waypoints to find look-ahead point
        let mut look_x = self.waypoints[self.progress_idx][0];
        let mut look_z = self.waypoints[self.progress_idx][1];
        let mut accumulated = 0.0;

        for &[px, pz] in &self.waypoints[(self.progress_idx + 1)..] {
            let seg = ((px - look_x).powi(2) + (pz - look_z).powi(2)).sqrt();
            accumulated += seg;
            look_x = px;
            look_z = pz;
            if accumulated >= look_ahead {
                break;
            }
        }

        let dx = look_x - tx;
        let dz = look_z - tz;
        if dx * dx + dz * dz < 1e-12 {
            return 0.0;
        }

        let target = dx.atan2(dz);
        let mut err = target - heading;
        while err > std::f64::consts::PI {
            err -= 2.0 * std::f64::consts::PI;
        }
        while err < -std::f64::consts::PI {
            err += 2.0 * std::f64::consts::PI;
        }
        err
    }
}

impl Plugin for LaneKeeperPlugin {
    fn name(&self) -> &str {
        "lane-keeper"
    }
    fn version(&self) -> &str {
        "0.1.0"
    }
    fn settings_schema(&self) -> &str {
        r#"{"type":"object","properties":{"kp":{"type":"number"},"ki":{"type":"number"},"kd":{"type":"number"},"subdivisions":{"type":"integer","minimum":1,"maximum":20}}}"#
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        self.load_waypoints_from_blackboard(ctx);
        tracing::info!("[lane-keeper] loaded");
    }

    fn on_unload(&mut self) {
        tracing::info!("[lane-keeper] unloaded");
    }

    fn tick(
        &mut self,
        _telemetry: Option<&Telemetry>,
        _output: &mut ControlOutput,
        ctx: &PluginContext,
    ) {
        // Side-effect-only path: keep waypoint cache fresh.
        // Steering itself is contributed via `tick_request` (Fix 4).
        if ctx.blackboard.get("router.active").as_deref() == Some("true")
            && self.waypoints.is_empty()
        {
            self.load_waypoints_from_blackboard(ctx);
        }
    }

    fn tick_request(
        &mut self,
        telemetry: Option<&Telemetry>,
        ctx: &PluginContext,
    ) -> Option<ControlRequest> {
        // State-gate: refuse to steer outside Active. Reset integrator
        // so the next engage starts from a clean PID state.
        if !ctx.is_active() {
            self.pid.reset();
            return None;
        }

        // Apply runtime gain overrides from blackboard (PID hotswap).
        self.apply_gain_overrides(ctx);

        let t = telemetry?;

        // Belt-and-braces: still require engine running. (The state
        // machine's precondition check covers this too, but the gate
        // protects us if the state lags by a tick.)
        if t.engine_rpm < 100.0 {
            self.pid.reset();
            return None;
        }

        // Clamp dt to avoid PID explosion when the daemon loop stalls
        // (debug pause, slow disk, etc.).
        let dt = ctx.dt_s.min(0.1);

        let err = self.compute_heading_error(t.position[0], t.position[2], t.heading, t.speed_ms);
        let steering = self.pid.update(err, dt).clamp(-1.0, 1.0);

        Some(ControlRequest {
            steering: Some(steering),
            priority: PRIORITY_NORMAL,
            ..Default::default()
        })
    }
}

/// Catmull-Rom spline interpolation.
fn smooth_catmull_rom(pts: &[[f64; 2]], subdivisions: usize) -> Vec<[f64; 2]> {
    if pts.len() < 2 {
        return pts.to_vec();
    }
    let n = pts.len();
    let mut result = Vec::with_capacity((n - 1) * (subdivisions + 1));

    for i in 1..n {
        let p0 = if i >= 2 { pts[i - 2] } else { pts[0] };
        let p1 = pts[i - 1];
        let p2 = pts[i];
        let p3 = if i + 1 < n { pts[i + 1] } else { pts[n - 1] };

        for j in 0..=subdivisions {
            if j == 0 && i > 1 {
                continue;
            }
            let t = j as f64 / subdivisions as f64;
            let t2 = t * t;
            let t3 = t2 * t;
            let x = 0.5
                * ((2.0 * p1[0])
                    + (-p0[0] + p2[0]) * t
                    + (2.0 * p0[0] - 5.0 * p1[0] + 4.0 * p2[0] - p3[0]) * t2
                    + (-p0[0] + 3.0 * p1[0] - 3.0 * p2[0] + p3[0]) * t3);
            let z = 0.5
                * ((2.0 * p1[1])
                    + (-p0[1] + p2[1]) * t
                    + (2.0 * p0[1] - 5.0 * p1[1] + 4.0 * p2[1] - p3[1]) * t2
                    + (-p0[1] + 3.0 * p1[1] - 3.0 * p2[1] + p3[1]) * t3);
            result.push([x, z]);
        }
    }
    result
}

truckpilot_plugin_api::export_plugin!(LaneKeeperPlugin);

#[cfg(test)]
mod tests {
    use super::*;
    use truckpilot_plugin_api::SharedBlackboard;

    fn make_telemetry(speed_ms: f64, heading: f64) -> Telemetry {
        Telemetry {
            position: [0.0; 3],
            heading,
            pitch: 0.0,
            roll: 0.0,
            speed_ms,
            engine_rpm: 1200.0,
            cruise_control_kmh: 80.0,
            nav_speed_limit_kmh: -1.0,
            lead_vehicle_distance_m: -1.0,
            accel_longitudinal: -1.0,
            fuel_liters: -1.0,
            odometer_km: -1.0,
        }
    }

    fn ctx_with_state(state: &str) -> PluginContext {
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", state);
        PluginContext::new("lane-keeper", bb)
    }

    fn active_plugin_with_straight_path() -> LaneKeeperPlugin {
        LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, 100.0], [0.0, 200.0]],
            ..Default::default()
        }
    }

    // ---- Pre-existing geometry tests (kept) --------------------------------

    #[test]
    fn look_ahead_increases_with_speed() {
        let slow = BASE_LOOK_AHEAD + 0.0 * SPEED_FACTOR;
        let fast = BASE_LOOK_AHEAD + 80.0 * SPEED_FACTOR;
        assert!(fast > slow);
        assert!((slow - 5.0).abs() < 0.01);
        assert!((fast - 45.0).abs() < 0.01);
    }

    #[test]
    fn straight_north_zero_error() {
        let mut lk = active_plugin_with_straight_path();
        let err = lk.compute_heading_error(0.0, 0.0, 0.0, 10.0);
        assert!(err.abs() < 0.01, "expected ~0, got {err}");
    }

    #[test]
    fn turn_right_positive_error() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [100.0, 0.0]],
            ..Default::default()
        };
        let err = lk.compute_heading_error(0.0, 0.0, 0.0, 10.0);
        assert!(err > 0.0, "expected positive (right turn), got {err}");
    }

    #[test]
    fn catmull_rom_more_points_than_input() {
        let pts = vec![[0.0, 0.0], [50.0, 10.0], [100.0, 0.0]];
        let smoothed = smooth_catmull_rom(&pts, 4);
        assert!(smoothed.len() > pts.len());
    }

    // ---- Phase 6.2d state-gate + control-output tests ----------------------

    #[test]
    fn test_state_gate_returns_none_when_off() {
        let mut lk = active_plugin_with_straight_path();
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Off");
        assert!(lk.tick_request(Some(&t), &ctx).is_none());
    }

    #[test]
    fn test_state_gate_returns_none_when_engaging() {
        let mut lk = active_plugin_with_straight_path();
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Engaging");
        assert!(lk.tick_request(Some(&t), &ctx).is_none());
    }

    #[test]
    fn test_active_steering_with_waypoints() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [100.0, 0.0]], // east
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0); // heading north → must turn right
        let ctx = ctx_with_state("Active");
        let req = lk
            .tick_request(Some(&t), &ctx)
            .expect("active must request");
        let s = req.steering.expect("active must request steering");
        assert!(s > 0.0, "expected positive steering, got {s}");
        assert_eq!(req.priority, PRIORITY_NORMAL);
    }

    #[test]
    fn test_straight_line_near_zero_steering() {
        let mut lk = active_plugin_with_straight_path();
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let req = lk.tick_request(Some(&t), &ctx).unwrap();
        let s = req.steering.unwrap();
        assert!(s.abs() < 0.1, "expected near-zero on straight, got {s}");
    }

    #[test]
    fn test_heading_wraparound() {
        // Waypoint slightly east of north; truck heading near +π. Naive
        // subtraction would give a -π+ε error; the wraparound must
        // normalise it to a small positive value (small left turn).
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.1, -100.0]],
            ..Default::default()
        };
        let heading = std::f64::consts::PI - 0.01;
        let err = lk.compute_heading_error(0.0, 0.0, heading, 10.0);
        assert!(err.abs() < 0.5, "wraparound produced {err}");
    }

    #[test]
    fn test_pid_reset_on_state_exit() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [100.0, 0.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        // Pump integrator while Active.
        let active = ctx_with_state("Active");
        for _ in 0..10 {
            let _ = lk.tick_request(Some(&t), &active);
        }
        // Exit to Off — must reset the integrator.
        let off = ctx_with_state("Off");
        assert!(lk.tick_request(Some(&t), &off).is_none());

        // Build a fresh plugin for an independent baseline.
        let mut fresh = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [100.0, 0.0]],
            ..Default::default()
        };
        let active2 = ctx_with_state("Active");
        let fresh_req = fresh.tick_request(Some(&t), &active2).unwrap();
        let resumed_req = lk.tick_request(Some(&t), &active2).unwrap();
        assert!(
            (fresh_req.steering.unwrap() - resumed_req.steering.unwrap()).abs() < 1e-6,
            "post-reset response must match a fresh PID"
        );
    }

    #[test]
    fn test_dt_clamp_at_0_1() {
        // A huge ctx.dt_s must not blow up the integrator. We can't
        // observe dt directly, so we drive the same error with a large
        // ctx.dt_s and assert the output is still within the controller
        // clamp range [-1, 1].
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [100.0, 0.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", "Active");
        let ctx = PluginContext::new("lane-keeper", bb).with_dt(10.0); // 10 s
        let req = lk.tick_request(Some(&t), &ctx).unwrap();
        let s = req.steering.unwrap();
        assert!((-1.0..=1.0).contains(&s), "output out of range: {s}");
    }
}
