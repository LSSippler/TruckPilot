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
    last_waypoints_hash: u64,
}

impl Default for LaneKeeperPlugin {
    fn default() -> Self {
        Self {
            pid: Pid::new(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, 2.0, 1.0),
            waypoints: Vec::new(),
            progress_idx: 0,
            subdivisions: 4,
            last_gains: (DEFAULT_KP, DEFAULT_KI, DEFAULT_KD),
            last_waypoints_hash: 0,
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
                let new_hash = hash_str(&json);
                self.waypoints = smooth_catmull_rom(&pts, self.subdivisions);
                self.progress_idx = 0;
                self.last_waypoints_hash = new_hash;
                if !self.waypoints.is_empty() {
                    tracing::info!(
                        "[lane-keeper] first 3 spline pts: [{:.1},{:.1}] [{:.1},{:.1}] [{:.1},{:.1}]",
                        self.waypoints[0][0], self.waypoints[0][1],
                        self.waypoints.get(1).map(|p| p[0]).unwrap_or(0.0),
                        self.waypoints.get(1).map(|p| p[1]).unwrap_or(0.0),
                        self.waypoints.get(2).map(|p| p[0]).unwrap_or(0.0),
                        self.waypoints.get(2).map(|p| p[1]).unwrap_or(0.0),
                    );
                }
                tracing::info!(
                    "[lane-keeper] loaded {} waypoints (smoothed, hash={:x})",
                    self.waypoints.len(),
                    new_hash
                );
            }
        }
    }

    fn compute_heading_error(
        &mut self,
        tx: f64,
        tz: f64,
        heading: f64,
        speed_ms: f64,
        ctx: &PluginContext,
    ) -> f64 {
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

        // Route-End-Guard: at last waypoint, no look-ahead possible
        if self.progress_idx + 1 >= self.waypoints.len() {
            ctx.blackboard.set("lane_keeper.skip_reason", "route_end");
            ctx.blackboard.set("lane_keeper.error_rad", "0.0");
            ctx.blackboard.set("lane_keeper.progress_idx_after_advance", self.progress_idx.to_string());
            ctx.blackboard.set("lane_keeper.waypoints_remaining", "0");
            ctx.blackboard.set("lane_keeper.advance_check_dist", "0.00");
            ctx.blackboard.set("lane_keeper.walk_iterations", "0");
            ctx.blackboard.set("lane_keeper.walk_accumulated_m", "0.00");
            return 0.0;
        }

        // Distanz Truck → nächster WP (nach Guard garantiert in-bounds)
        let [nx, nz] = self.waypoints[self.progress_idx + 1];
        let advance_check_dist = ((tx - nx).powi(2) + (tz - nz).powi(2)).sqrt();

        // Speed-adaptive look-ahead
        let look_ahead = BASE_LOOK_AHEAD + speed_ms * 3.6 * SPEED_FACTOR;

        // Walk forward along waypoints to find look-ahead point.
        // Start from truck position so accumulated distance matches actual look-ahead.
        let mut look_x = tx;
        let mut look_z = tz;
        let mut accumulated = 0.0;
        let mut walk_iterations: usize = 0;

        for &[px, pz] in &self.waypoints[(self.progress_idx + 1)..] {
            let seg = ((px - look_x).powi(2) + (pz - look_z).powi(2)).sqrt();
            accumulated += seg;
            walk_iterations += 1;
            look_x = px;
            look_z = pz;
            if accumulated >= look_ahead {
                break;
            }
        }

        // Phase 6.5g: write lookahead diagnostics before atan2
        ctx.blackboard
            .set("lane_keeper.lookahead_m", format!("{look_ahead:.2}"));
        ctx.blackboard
            .set("lane_keeper.look_x", format!("{look_x:.2}"));
        ctx.blackboard
            .set("lane_keeper.look_z", format!("{look_z:.2}"));
        ctx.blackboard
            .set("lane_keeper.dx", format!("{:.2}", look_x - tx));
        ctx.blackboard
            .set("lane_keeper.dz", format!("{:.2}", look_z - tz));
        ctx.blackboard
            .set("lane_keeper.walk_iterations", walk_iterations.to_string());
        ctx.blackboard
            .set("lane_keeper.walk_accumulated_m", format!("{accumulated:.2}"));
        ctx.blackboard
            .set("lane_keeper.advance_check_dist", format!("{advance_check_dist:.2}"));
        ctx.blackboard
            .set("lane_keeper.progress_idx_after_advance", self.progress_idx.to_string());
        ctx.blackboard.set(
            "lane_keeper.waypoints_remaining",
            self.waypoints.len().saturating_sub(self.progress_idx + 1).to_string(),
        );

        let dx = look_x - tx;
        let dz = look_z - tz;
        if dx * dx + dz * dz < 1e-12 {
            return 0.0;
        }

        // ETS2-Konvention: heading=0 zeigt Richtung -Z (Nord).
        // Damit target_heading konsistent mit telemetry.heading ist, muss dz negiert werden.
        let target = dx.atan2(-dz);
        let mut err = target - heading;
        while err > std::f64::consts::PI {
            err -= 2.0 * std::f64::consts::PI;
        }
        while err < -std::f64::consts::PI {
            err += 2.0 * std::f64::consts::PI;
        }

        ctx.blackboard
            .set("lane_keeper.target_heading", format!("{target:.6}"));

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
        if ctx.blackboard.get("router.active").as_deref() == Some("true") {
            let current_hash = ctx
                .blackboard
                .get("router.waypoints")
                .map(|j| hash_str(&j))
                .unwrap_or(0);
            if self.waypoints.is_empty() || current_hash != self.last_waypoints_hash {
                self.load_waypoints_from_blackboard(ctx);
            }
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
            if !self.waypoints.is_empty() {
                self.waypoints.clear();
                self.last_waypoints_hash = 0;
                self.progress_idx = 0;
                tracing::info!("[lane-keeper] state=Off, cleared waypoint cache");
            }
            // Phase 6.5g: mark inactive
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard
                .set("lane_keeper.skip_reason", "state_not_active");
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
            // Phase 6.5g: mark inactive
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard
                .set("lane_keeper.skip_reason", "engine_off");
            return None;
        }

        // Clamp dt to avoid PID explosion when the daemon loop stalls
        // (debug pause, slow disk, etc.).
        let dt = ctx.dt_s.min(0.1);

        let err = self.compute_heading_error(
            t.position[0],
            t.position[2],
            t.heading,
            t.speed_ms,
            ctx,
        );
        let steering = self.pid.update(err, dt).clamp(-1.0, 1.0);

        // Phase 6.5g: Diagnose-Blackboard-Writes fuer Bug-Hunting
        ctx.blackboard.set("lane_keeper.active", "true");
        ctx.blackboard
            .set("lane_keeper.error_rad", format!("{err:.6}"));
        ctx.blackboard
            .set("lane_keeper.steering_out", format!("{steering:.6}"));
        ctx.blackboard.set(
            "lane_keeper.waypoints_loaded",
            self.waypoints.len().to_string(),
        );
        ctx.blackboard
            .set("lane_keeper.progress_idx", self.progress_idx.to_string());
        ctx.blackboard.set("lane_keeper.dt_s", format!("{dt:.6}"));
        ctx.blackboard
            .set("lane_keeper.truck_x", format!("{:.2}", t.position[0]));
        ctx.blackboard
            .set("lane_keeper.truck_z", format!("{:.2}", t.position[2]));
        ctx.blackboard
            .set("lane_keeper.truck_heading", format!("{:.6}", t.heading));
        ctx.blackboard
            .set("lane_keeper.truck_speed_ms", format!("{:.2}", t.speed_ms));

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

fn hash_str(s: &str) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    s.hash(&mut h);
    h.finish()
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

    fn fresh_ctx() -> PluginContext {
        let bb = SharedBlackboard::new();
        PluginContext::new("lane-keeper", bb)
    }

    fn active_plugin_with_straight_path() -> LaneKeeperPlugin {
        // ETS2: North = -Z. Waypoints in -Z direction, heading=0 → near-zero error.
        LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, -100.0], [0.0, -200.0]],
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
        let ctx = fresh_ctx();
        let err = lk.compute_heading_error(0.0, 0.0, 0.0, 10.0, &ctx);
        assert!(err.abs() < 0.01, "expected ~0, got {err}");
    }

    #[test]
    fn turn_right_positive_error() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [100.0, 0.0]],
            ..Default::default()
        };
        let ctx = fresh_ctx();
        let err = lk.compute_heading_error(0.0, 0.0, 0.0, 10.0, &ctx);
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

    // ---- Phase 6.5l Heading-Konvention-Tests (ETS2: heading=0 → -Z / Nord) ----

    #[test]
    fn heading_convention_north_is_zero() {
        // Look-Ahead direkt vor Truck (Nord = -Z). Truck-heading = 0. error ~ 0.
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, -100.0]],
            ..Default::default()
        };
        let ctx = fresh_ctx();
        let err = plugin.compute_heading_error(0.0, 0.0, 0.0, 13.88, &ctx);
        assert!(err.abs() < 0.01, "expected ~0, got {err}");
    }

    #[test]
    fn heading_convention_east_is_half_pi() {
        // Look-Ahead direkt rechts (Osten = +X). Truck-heading = π/2. error ~ 0.
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [100.0, 0.0]],
            ..Default::default()
        };
        let ctx = fresh_ctx();
        let err = plugin.compute_heading_error(
            0.0,
            0.0,
            std::f64::consts::FRAC_PI_2,
            13.88,
            &ctx,
        );
        assert!(err.abs() < 0.01, "expected ~0, got {err}");
    }

    #[test]
    fn heading_convention_punkt_vor_rechts_kleiner_positiver_error() {
        // Reproduziert den Live-Fall: Truck-heading 0.353, Look-Ahead bei dx=145, dz=-156.
        // Erwartet: error ~ +0.4 rad (positiv, klein, kein Vollanschlag).
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [144.89, -156.22]],
            ..Default::default()
        };
        let ctx = fresh_ctx();
        let err = plugin.compute_heading_error(0.0, 0.0, 0.353, 13.88, &ctx);
        assert!(err > 0.0 && err < 0.6, "expected ~0.4 positive, got {err}");
    }

    #[test]
    fn test_heading_wraparound() {
        // ETS2: heading=π → South (+Z). Truck nearly south, waypoint slightly
        // west of south [-0.1, 100]. target ≈ -π+ε (third quadrant, atan2(-0.1,100)
        // with negated dz). Naive subtraction would give ~-2π; the wraparound
        // normalises to a small value near 0.
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [-0.1, 100.0]],
            ..Default::default()
        };
        let heading = std::f64::consts::PI - 0.01;
        let ctx = fresh_ctx();
        let err = lk.compute_heading_error(0.0, 0.0, heading, 10.0, &ctx);
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

        // Off-state now clears the waypoint cache (Phase 6.5k). Restore
        // waypoints explicitly so the re-engage check has a path to steer.
        lk.waypoints = vec![[0.0, 0.0], [100.0, 0.0]];

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
    fn waypoints_reload_on_route_change() {
        let mut plugin = LaneKeeperPlugin::default();

        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", "Active");
        bb.set("router.active", "true");
        bb.set("router.waypoints", "[[0.0,0.0],[10.0,0.0],[20.0,0.0]]");
        let ctx = PluginContext::new("lane-keeper", bb.clone());

        plugin.tick(None, &mut ControlOutput::default(), &ctx);
        let first_len = plugin.waypoints.len();
        let first_hash = plugin.last_waypoints_hash;
        assert!(first_len > 0);
        assert!(first_hash != 0);

        // Different route — hash must differ, waypoints must reload.
        bb.set(
            "router.waypoints",
            "[[100.0,100.0],[110.0,100.0],[120.0,100.0]]",
        );
        plugin.tick(None, &mut ControlOutput::default(), &ctx);

        assert_ne!(plugin.last_waypoints_hash, first_hash);
        assert!((plugin.waypoints[0][0] - 100.0).abs() < 1e-6);
        assert_eq!(plugin.progress_idx, 0);
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

    // ---- Phase 6.5m: Route-End-Guard + Walk-from-Truck-Position tests --------

    /// Route-End-Guard: when progress_idx is at the last waypoint,
    /// compute_heading_error must return 0.0 and set skip_reason = "route_end".
    #[test]
    fn route_end_returns_zero_error() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [10.0, 0.0]],
            progress_idx: 1, // last waypoint
            ..Default::default()
        };
        let ctx = fresh_ctx();
        // Truck near last waypoint but not at it
        let err = plugin.compute_heading_error(9.0, 0.0, 0.0, 0.0, &ctx);
        assert_eq!(err, 0.0, "Route-End-Guard must return 0.0, got {err}");
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("route_end"),
            "skip_reason must be 'route_end'"
        );
    }

    /// Progress advances when truck is within WAYPOINT_REACH_M of next waypoint,
    /// and stops when the following waypoint is out of range.
    #[test]
    fn progress_advances_when_truck_near_next_waypoint() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [10.0, 0.0], [20.0, 0.0]],
            progress_idx: 0,
            ..Default::default()
        };
        let ctx = fresh_ctx();
        // Truck at (6, 0): dist to WP[1]=(10,0) is 4.0 < 5.0 → advance to 1.
        // dist to WP[2]=(20,0) is 14.0 > 5.0 → stop advancing.
        plugin.compute_heading_error(6.0, 0.0, 0.0, 0.0, &ctx);
        assert_eq!(
            plugin.progress_idx, 1,
            "progress_idx must be 1 after advancing past WP[1], got {}",
            plugin.progress_idx
        );
    }

    /// Look-ahead walk starts from the truck's position (progress_idx waypoint),
    /// not from progress_idx+1. Verifies the look point for a simple straight route.
    #[test]
    fn lookahead_starts_from_truck_position() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, 0.0], [40.0, 0.0]],
            progress_idx: 0,
            ..Default::default()
        };
        let ctx = fresh_ctx();
        // Truck at (0,0). Walk over waypoints[0..]:
        //   iter 1: (0,0)→WP[0](0,0) = 0m, accumulated=0, look=(0,0)
        //   iter 2: (0,0)→WP[1](20,0) = 20m, accumulated=20 >= 5 → break, look=(20,0)
        plugin.compute_heading_error(0.0, 0.0, 0.0, 0.0, &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.look_x").as_deref(),
            Some("20.00"),
            "look_x must be 20.00"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.look_z").as_deref(),
            Some("0.00"),
            "look_z must be 0.00"
        );
    }

    /// Walk accumulates distance across multiple short waypoint segments until
    /// the look-ahead distance is reached.
    #[test]
    fn lookahead_walks_through_multiple_waypoints() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![
                [0.0, 0.0],
                [3.0, 0.0],
                [6.0, 0.0],
                [9.0, 0.0],
                [12.0, 0.0],
            ],
            progress_idx: 0,
            ..Default::default()
        };
        let ctx = fresh_ctx();
        // Truck at (0,0). The advance loop first runs: dist to WP[1]=(3,0) is 3m < 5m
        // → progress_idx advances to 1. dist to WP[2]=(6,0) is 6m > 5m → stops.
        // Walk iterates waypoints[(1+1)..] = waypoints[2..], starting from truck (0,0):
        //   iter 1: (0,0)→WP[2](6,0) = 6m, accumulated=6 >= 5 → break, look=(6,0)
        plugin.compute_heading_error(0.0, 0.0, 0.0, 0.0, &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.walk_iterations").as_deref(),
            Some("1"),
            "walk_iterations must be 1 (truck→WP[2] = 6m satisfies look_ahead=5m immediately)"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.look_x").as_deref(),
            Some("6.00"),
            "look_x must be 6.00"
        );
    }
}
