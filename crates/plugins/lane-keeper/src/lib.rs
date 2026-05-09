//! Lane-Keeper plugin — PID steering with speed-adaptive look-ahead.
//!
//! Reads waypoints from `router.waypoints` on the blackboard.
//! Writes only `output.steering`, leaves throttle/brake untouched.
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

pub struct LaneKeeperPlugin {
    pid: Pid,
    waypoints: Vec<[f64; 2]>, // (x, z) pairs
    progress_idx: usize,
    /// Catmull-Rom subdivisions (configurable).
    subdivisions: usize,
}

impl Default for LaneKeeperPlugin {
    fn default() -> Self {
        Self {
            pid: Pid::new(0.8, 0.1, 0.3, 2.0, 1.0),
            waypoints: Vec::new(),
            progress_idx: 0,
            subdivisions: 4,
        }
    }
}

impl LaneKeeperPlugin {
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
        _ctx: &PluginContext,
    ) -> Option<ControlRequest> {
        let t = telemetry?;

        // Only steer when engine on and cruise active.
        if t.engine_rpm < 100.0 || t.cruise_control_kmh <= 0.0 {
            return None;
        }

        let dt = 0.02; // 20 ms fixed step
        let err = self.compute_heading_error(t.position[0], t.position[2], t.heading, t.speed_ms);
        let steering = self.pid.update(err, dt);

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

    #[test]
    fn look_ahead_increases_with_speed() {
        // At 0 km/h: BASE_LOOK_AHEAD = 5m
        // At 80 km/h: 5 + 80 * 0.5 = 45m
        let slow = BASE_LOOK_AHEAD + 0.0 * SPEED_FACTOR;
        let fast = BASE_LOOK_AHEAD + 80.0 * SPEED_FACTOR;
        assert!(fast > slow);
        assert!((slow - 5.0).abs() < 0.01);
        assert!((fast - 45.0).abs() < 0.01);
    }

    #[test]
    fn straight_north_zero_error() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, 100.0], [0.0, 200.0]],
            ..Default::default()
        };
        // Truck at origin, heading North (0.0), target is North
        let err = lk.compute_heading_error(0.0, 0.0, 0.0, 10.0);
        assert!(err.abs() < 0.01, "expected ~0, got {err}");
    }

    #[test]
    fn turn_right_positive_error() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [100.0, 0.0]], // East
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
}
