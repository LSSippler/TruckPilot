//! Live autopilot control loop.
//!
//! When telemetry is enabled, this module runs a blocking loop that:
//! 1. Fetches live telemetry from the ETS2 server.
//! 2. Advances the waypoint along the planned route.
//! 3. Computes heading error with look-ahead and feeds it into a PID controller.
//! 4. Regulates speed using the adaptive `SpeedController`.
//! 5. Outputs control commands via `println!`.
//!
//! ## Heading convention
//!
//! ETS2 uses navigation convention: 0 = North (+Z), positive = clockwise (East).
//! `atan2(dx, dz)` produces angles in this same convention, so steering error
//! is computed directly as `target_heading - current_heading`.

use std::thread;
use std::time::{Duration, Instant};

use crate::acc_controller::AccController;
use crate::config::TruckPilotConfig;
use crate::controller::{SpeedController, SteerController};
use crate::route_smoothing::smooth_route;
use crate::shm_telemetry;
use crate::telemetry::{fetch_telemetry, TelemetryData};
use crate::vjoy::ControlOutput;

/// Default telemetry server URL (Funbit ETS2 Telemetry Server).
pub const DEFAULT_TELEMETRY_URL: &str = "http://localhost:25555/api/ets2/telemetry";

/// Cycle interval for the control loop (20 ms ≈ 50 Hz).
const LOOP_INTERVAL_MS: u64 = 20;

/// Look-ahead distance for steering in meters.
/// Instead of steering directly toward the next waypoint, we look ahead
/// along the route by this distance to avoid zigzagging.
const LOOK_AHEAD_DIST: f64 = 50.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TelemetrySource {
    Shm,
    Http,
}

/// Run the telemetry-driven autopilot loop.
///
/// # Arguments
/// * `route_path` — ordered list of node UIDs defining the planned route.
/// * `node_positions` — lookup of UIDs to (x, z) world coordinates.
/// * `telemetry_url` — URL of the ETS2 telemetry REST endpoint.
///
/// This function blocks indefinitely (until the process is killed).
/// Control commands are printed to stdout.
pub fn run_autopilot_loop(
    route_path: Vec<u64>,
    node_positions: std::collections::HashMap<u64, (f64, f64)>,
    telemetry_url: &str,
    mut output: Box<dyn ControlOutput>,
    config: &TruckPilotConfig,
) {
    let steering_path = build_steering_path(
        &route_path,
        &node_positions,
        config.routing.smooth_route,
        config.routing.subdivisions,
    );

    if config.routing.smooth_route {
        eprintln!(
            "Route smoothing enabled: raw={} -> smoothed={} points",
            route_path.len(),
            steering_path.len()
        );
    }

    let mut speed_controller = SpeedController::from_config(
        config.speed.kp,
        config.speed.ki,
        config.speed.kd,
        config.speed.integral_limit,
    );
    let mut steer_controller = SteerController::from_config(
        config.steering.kp,
        config.steering.ki,
        config.steering.kd,
        config.steering.integral_limit,
    );
    let mut steering_progress_idx: usize = 0;
    let mut acc_controller = config.acc.enabled.then(|| {
        AccController::new(
            config.acc.target_distance_m,
            config.acc.kp,
            config.acc.ki,
            config.acc.kd,
        )
    });
    let mut last_source: Option<TelemetrySource> = None;
    let mut current_waypoint_idx: usize = 0;
    let mut prev_time = Instant::now();

    println!(
        "Autopilot loop started. Route: {} waypoints.",
        route_path.len()
    );

    loop {
        let now = Instant::now();
        let dt = now.duration_since(prev_time).as_secs_f64();
        prev_time = now;

        // 1. Fetch telemetry — SHM first, then HTTP.
        let (telemetry, source) = match resolve_telemetry(shm_telemetry::read_shm_telemetry, || {
            fetch_telemetry(telemetry_url)
        }) {
            Some(t) => t,
            None => {
                // Send neutral controls before sleeping — prevents stuck inputs.
                output.set_steering(0.0);
                output.set_throttle(0.0);
                output.set_brake(0.0);
                output.flush();
                thread::sleep(Duration::from_millis(LOOP_INTERVAL_MS));
                continue;
            }
        };

        if last_source != Some(source) {
            eprintln!("Telemetry source switched to: {:?}", source);
            last_source = Some(source);
        }

        // 2. Check if autopilot should be active: engine on + cruise control set.
        let engine_on = telemetry.truck_float_values.engine_rpm >= 100.0;
        let cruise_speed = telemetry.truck_float_values.cruise_control_speed;
        let cruise_active = cruise_speed > 0.0;

        if !engine_on || !cruise_active {
            if !cruise_active {
                eprintln!(
                    "[vJoy] waiting... (cruise={cruise_speed:.0} km/h) — press C in ETS2 to engage"
                );
            }
            output.set_steering(0.0);
            output.set_throttle(0.0);
            output.set_brake(0.0);
            output.flush();
            thread::sleep(Duration::from_millis(LOOP_INTERVAL_MS));
            continue;
        }

        // 3. Advance waypoint.
        current_waypoint_idx = advance_waypoint(
            &route_path,
            current_waypoint_idx,
            &telemetry,
            &node_positions,
        );

        // 3. Compute heading error and feed into PID steering controller.
        let heading_error = compute_heading_error_from_points(
            &steering_path,
            &telemetry,
            &mut steering_progress_idx,
            config.steering.look_ahead_distance,
        );
        let steer = steer_controller.update(heading_error, dt);

        // 4. Speed control (nav-limit + optional ACC cap).
        let (target_speed_ms, acc_debug) =
            compute_target_speed_ms(&telemetry, config, acc_controller.as_mut(), dt as f32);

        let current_speed_ms = telemetry.truck_float_values.speed;

        if let Some((distance_m, acc_limit_kmh)) = acc_debug {
            eprintln!("ACC: distance={distance_m:.1}m, limit={acc_limit_kmh:.1} km/h");
        }

        let (throttle, brake) = speed_controller.update(target_speed_ms, current_speed_ms, dt);

        // 5. Output control commands.
        if !output.is_available() {
            eprintln!("Output backend unavailable, attempting re-acquire...");
            if !output.try_reacquire() {
                eprintln!("Output re-acquire failed, switching to ConsoleOutput fallback.");
                output = Box::new(crate::vjoy::ConsoleOutput);
            }
        }

        output.set_steering(steer);
        output.set_throttle(throttle);
        output.set_brake(brake);
        output.flush();

        let target_uid = route_path[current_waypoint_idx.min(route_path.len() - 1)];
        let (tx, tz) = node_positions
            .get(&target_uid)
            .copied()
            .unwrap_or((0.0, 0.0));

        eprintln!(
            "[vJoy] steer={:+.3} thr={:.3} brk={:.3} | speed={:.1} km/h | pos=({:.0},{:.0}) -> node {} ({:.0},{:.0})",
            steer,
            throttle,
            brake,
            telemetry.truck_float_values.speed * 3.6,
            telemetry.truck_placement.x,
            telemetry.truck_placement.z,
            target_uid,
            tx,
            tz,
        );

        // Maintain 20 ms cycle.
        let elapsed = now.elapsed();
        if elapsed < Duration::from_millis(LOOP_INTERVAL_MS) {
            thread::sleep(Duration::from_millis(LOOP_INTERVAL_MS) - elapsed);
        }
    }
}

fn compute_target_speed_ms(
    telemetry: &TelemetryData,
    config: &TruckPilotConfig,
    mut acc_controller: Option<&mut AccController>,
    dt: f32,
) -> (f64, Option<(f32, f32)>) {
    let nav_limit_kmh = telemetry
        .navigation_speed_limit
        .unwrap_or(config.speed.fallback_speed_kmh);

    let mut acc_debug = None;
    let acc_limit_kmh = if let Some(acc) = acc_controller.as_mut() {
        let distance_input = telemetry.lead_vehicle_distance_m.or_else(|| {
            proxy_distance_from_accel(
                telemetry.local_acceleration_longitudinal,
                acc.target_distance_m,
            )
        });
        let current_speed_kmh = (telemetry.truck_float_values.speed * 3.6) as f32;
        let limit = acc.update(distance_input, current_speed_kmh, dt);
        if let Some(d) = distance_input {
            acc_debug = Some((d, limit));
        }
        limit
    } else {
        f32::MAX
    };

    let target_speed_kmh = nav_limit_kmh.min(f64::from(acc_limit_kmh));
    (target_speed_kmh / 3.6, acc_debug)
}

fn proxy_distance_from_accel(local_accel: Option<f32>, target_distance: f32) -> Option<f32> {
    let accel = local_accel?;
    if accel < -0.2 {
        // Empirical fallback: project braking acceleration over 2s horizon.
        let reaction_horizon_s = 2.0_f32;
        let projected_delta = 0.5 * accel * reaction_horizon_s * reaction_horizon_s;
        Some((target_distance + projected_delta).max(5.0))
    } else {
        None
    }
}

fn resolve_telemetry<FShm, FHttp, E>(
    read_shm: FShm,
    read_http: FHttp,
) -> Option<(TelemetryData, TelemetrySource)>
where
    FShm: FnOnce() -> Option<TelemetryData>,
    FHttp: FnOnce() -> Result<TelemetryData, E>,
    E: std::fmt::Display,
{
    if let Some(data) = read_shm() {
        return Some((data, TelemetrySource::Shm));
    }

    match read_http() {
        Ok(data) => Some((data, TelemetrySource::Http)),
        Err(e) => {
            eprintln!("HTTP telemetry failed ({e}), using neutral controls");
            None
        }
    }
}

fn build_steering_path(
    route_path: &[u64],
    positions: &std::collections::HashMap<u64, (f64, f64)>,
    smooth: bool,
    subdivisions: usize,
) -> Vec<(f64, f64)> {
    let mut raw_points: Vec<(f64, f64)> = Vec::with_capacity(route_path.len());
    for uid in route_path {
        let Some(&pt) = positions.get(uid) else {
            eprintln!(
                "Route contains missing node UID {uid} in positions map; steering path disabled."
            );
            return Vec::new();
        };
        raw_points.push(pt);
    }

    if smooth {
        smooth_route(&raw_points, subdivisions.max(1))
    } else {
        raw_points
    }
}

fn compute_heading_error_from_points(
    path_points: &[(f64, f64)],
    telemetry: &TelemetryData,
    progress_idx: &mut usize,
    look_ahead_distance: f64,
) -> f64 {
    if path_points.len() < 2 {
        return 0.0;
    }

    let tx = telemetry.truck_placement.x;
    let tz = telemetry.truck_placement.z;

    if *progress_idx >= path_points.len() {
        *progress_idx = path_points.len() - 1;
    }

    // Find closest point near current progress to avoid jumping backwards.
    let search_start = progress_idx.saturating_sub(5);
    let search_end = (*progress_idx + 200).min(path_points.len() - 1);

    let mut nearest_idx = *progress_idx;
    let mut best_d2 = f64::MAX;
    for (i, &(px, pz)) in path_points
        .iter()
        .enumerate()
        .skip(search_start)
        .take(search_end - search_start + 1)
    {
        let dx = px - tx;
        let dz = pz - tz;
        let d2 = dx * dx + dz * dz;
        if d2 < best_d2 {
            best_d2 = d2;
            nearest_idx = i;
        }
    }
    if nearest_idx > *progress_idx {
        *progress_idx = nearest_idx;
    }

    let mut look_x = path_points[*progress_idx].0;
    let mut look_z = path_points[*progress_idx].1;
    let mut accumulated = 0.0;

    for &(px, pz) in &path_points[(*progress_idx + 1)..] {
        let seg_dx = px - look_x;
        let seg_dz = pz - look_z;
        let seg_dist = (seg_dx * seg_dx + seg_dz * seg_dz).sqrt();
        accumulated += seg_dist;
        look_x = px;
        look_z = pz;
        if accumulated >= look_ahead_distance {
            break;
        }
    }

    let dx = look_x - tx;
    let dz = look_z - tz;
    let dist2 = dx * dx + dz * dz;
    if dist2 < 1e-12 {
        return 0.0;
    }

    let target_heading = dx.atan2(dz);
    let current_heading = telemetry.truck_placement.heading;
    let mut error = target_heading - current_heading;
    while error > std::f64::consts::PI {
        error -= 2.0 * std::f64::consts::PI;
    }
    while error < -std::f64::consts::PI {
        error += 2.0 * std::f64::consts::PI;
    }

    error
}

/// Advance the current waypoint index based on proximity to the next node.
fn advance_waypoint(
    route_path: &[u64],
    current_idx: usize,
    telemetry: &TelemetryData,
    positions: &std::collections::HashMap<u64, (f64, f64)>,
) -> usize {
    if current_idx + 1 >= route_path.len() {
        return current_idx; // Reached end of route.
    }

    let next_uid = route_path[current_idx + 1];
    if let Some(&(wx, wz)) = positions.get(&next_uid) {
        let tx = telemetry.truck_placement.x;
        let tz = telemetry.truck_placement.z;
        let dx = tx - wx;
        let dz = tz - wz;
        let dist = (dx * dx + dz * dz).sqrt();

        if dist < 5.0 {
            // Within 5 meters → advance.
            return current_idx + 1;
        }
    }

    current_idx
}

/// Compute the heading error (radians) to the look-ahead point.
///
/// Walks along the route until `LOOK_AHEAD_DIST` meters are accumulated,
/// then computes the angular difference between the truck's current heading
/// and the direction to that point. The caller feeds this into a PID controller.
///
/// Heading convention: ETS2 uses 0 = North (+Z), positive = clockwise (East).
#[allow(dead_code)]
fn compute_heading_error(
    route_path: &[u64],
    current_idx: usize,
    telemetry: &TelemetryData,
    positions: &std::collections::HashMap<u64, (f64, f64)>,
) -> f64 {
    // At the final waypoint, hold current heading.
    if current_idx + 1 >= route_path.len() {
        return 0.0;
    }

    let tx = telemetry.truck_placement.x;
    let tz = telemetry.truck_placement.z;

    // Walk along the route to find a look-ahead point.
    let mut look_x = tx;
    let mut look_z = tz;
    let mut accumulated = 0.0;
    let mut found_valid_point = false;

    // Start from the next waypoint, walking forward.
    for &uid in &route_path[(current_idx + 1)..] {
        let Some(&(wx, wz)) = positions.get(&uid) else {
            continue;
        };
        let seg_dx = wx - look_x;
        let seg_dz = wz - look_z;
        let seg_dist = (seg_dx * seg_dx + seg_dz * seg_dz).sqrt();
        accumulated += seg_dist;
        look_x = wx;
        look_z = wz;
        found_valid_point = true;
        if accumulated >= LOOK_AHEAD_DIST {
            break;
        }
    }

    if !found_valid_point {
        return 0.0;
    }

    let dx = look_x - tx;
    let dz = look_z - tz;

    let dist2 = dx * dx + dz * dz;
    if dist2 < 1e-12 {
        return 0.0;
    }

    // Use atan2(dx, dz) for ETS2 navigation convention (0=North, CW).
    let target_heading = dx.atan2(dz);

    let current_heading = telemetry.truck_placement.heading;

    // Angular error in [-PI, PI].
    let mut error = target_heading - current_heading;
    while error > std::f64::consts::PI {
        error -= 2.0 * std::f64::consts::PI;
    }
    while error < -std::f64::consts::PI {
        error += 2.0 * std::f64::consts::PI;
    }

    error
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::{TruckFloatValues, TruckPlacement};

    fn make_telemetry(x: f64, z: f64, heading: f64, speed: f64) -> TelemetryData {
        TelemetryData {
            truck_placement: TruckPlacement {
                x,
                y: 0.0,
                z,
                heading,
                pitch: 0.0,
                roll: 0.0,
            },
            truck_float_values: TruckFloatValues {
                speed,
                engine_rpm: 0.0,
                fuel: 0.0,
                odometer: 0.0,
                cruise_control_speed: 0.0,
            },
            navigation_speed_limit: None,
            lead_vehicle_distance_m: None,
            local_acceleration_longitudinal: None,
        }
    }

    #[test]
    fn test_advance_waypoint_not_reached() {
        let path = vec![1, 2, 3];
        let mut pos = std::collections::HashMap::new();
        pos.insert(2, (100.0, 0.0));
        let telemetry = make_telemetry(0.0, 0.0, 0.0, 10.0);

        let idx = advance_waypoint(&path, 0, &telemetry, &pos);
        assert_eq!(idx, 0); // too far
    }

    #[test]
    fn test_advance_waypoint_reached() {
        let path = vec![1, 2, 3];
        let mut pos = std::collections::HashMap::new();
        pos.insert(2, (3.0, 0.0));
        let telemetry = make_telemetry(0.0, 0.0, 0.0, 10.0);

        let idx = advance_waypoint(&path, 0, &telemetry, &pos);
        assert_eq!(idx, 1); // within 5m
    }

    #[test]
    fn test_compute_heading_error_straight_north() {
        // ETS2 heading 0 = North (+Z). Target directly North → error = 0.
        let path = vec![1, 2];
        let mut pos = std::collections::HashMap::new();
        pos.insert(1, (0.0, 0.0));
        pos.insert(2, (0.0, 100.0)); // target is North (+Z)
        let telemetry = make_telemetry(0.0, 0.0, 0.0, 10.0);

        let error = compute_heading_error(&path, 0, &telemetry, &pos);
        assert!((error - 0.0).abs() < 0.01, "expected straight, got {error}");
    }

    #[test]
    fn test_compute_heading_error_turn_right_east() {
        // Truck heading North (0), target is East (+X).
        // ETS2 clockwise: East = PI/2. Error = PI/2 → steer > 0 (left/clockwise).
        let path = vec![1, 2];
        let mut pos = std::collections::HashMap::new();
        pos.insert(1, (0.0, 0.0));
        pos.insert(2, (100.0, 0.0)); // target is East (+X)
        let telemetry = make_telemetry(0.0, 0.0, 0.0, 10.0);

        let error = compute_heading_error(&path, 0, &telemetry, &pos);
        assert!(
            error > 0.0,
            "expected clockwise turn (positive error), got {error}"
        );
    }

    #[test]
    fn test_compute_heading_error_turn_left_west() {
        // Truck heading North (0), target is West (-X).
        // ETS2: West = -PI/2. Error = -PI/2 → steer < 0 (counter-clockwise).
        let path = vec![1, 2];
        let mut pos = std::collections::HashMap::new();
        pos.insert(1, (0.0, 0.0));
        pos.insert(2, (-100.0, 0.0)); // target is West (-X)
        let telemetry = make_telemetry(0.0, 0.0, 0.0, 10.0);

        let error = compute_heading_error(&path, 0, &telemetry, &pos);
        assert!(
            error < 0.0,
            "expected counter-clockwise turn (negative error), got {error}"
        );
    }

    #[test]
    fn test_compute_heading_error_lookahead_uses_far_point() {
        // Route with 4 waypoints. Look-ahead should skip the first and target a
        // point beyond 50m. With total distance < LOOK_AHEAD_DIST it targets the
        // last waypoint.
        let path: Vec<u64> = vec![10, 20, 30, 40];
        let mut pos = std::collections::HashMap::new();
        pos.insert(10, (0.0, 0.0));
        pos.insert(20, (0.0, 19.0)); // 19m
        pos.insert(30, (0.0, 40.0)); // +21m = 40m total
        pos.insert(40, (0.0, 62.0)); // +22m = 62m total → exceeds LOOK_AHEAD_DIST(50)
                                     // Truck at (0,0) heading North. Look-ahead reaches waypoint 40 (62m > 50m).
        let telemetry = make_telemetry(0.0, 0.0, 0.0, 10.0);

        let error = compute_heading_error(&path, 0, &telemetry, &pos);
        // Target is (0, 62) straight North → error should be 0.
        assert!(
            (error - 0.0).abs() < 0.01,
            "lookahead should still point straight, got {error}"
        );
    }

    #[test]
    fn test_compute_heading_error_missing_waypoints_holds_course() {
        let path: Vec<u64> = vec![10, 20];
        let pos = std::collections::HashMap::new();
        let telemetry = make_telemetry(100.0, -20.0, 1.2, 10.0);

        let error = compute_heading_error(&path, 0, &telemetry, &pos);
        assert!(
            (error - 0.0).abs() < 0.0001,
            "expected hold-course on missing waypoints, got {error}"
        );
    }

    #[test]
    fn test_compute_heading_error_zero_distance_lookahead_holds_course() {
        let path: Vec<u64> = vec![10, 20];
        let mut pos = std::collections::HashMap::new();
        // Next waypoint is exactly at truck position -> dx=dz=0 edge case.
        pos.insert(20, (100.0, -20.0));
        let telemetry = make_telemetry(100.0, -20.0, 1.2, 10.0);

        let error = compute_heading_error(&path, 0, &telemetry, &pos);
        assert!(
            (error - 0.0).abs() < 0.0001,
            "expected hold-course for zero-distance lookahead, got {error}"
        );
    }

    #[test]
    fn test_pid_steering_in_loop() {
        let src = include_str!("autopilot_loop.rs");
        assert!(
            src.contains("steer_controller.update(heading_error, dt)"),
            "expected SteerController::update call in loop"
        );
    }

    #[test]
    fn test_smoothing_in_loop() {
        let route = vec![1_u64, 2_u64, 3_u64];
        let mut pos = std::collections::HashMap::new();
        pos.insert(1, (0.0, 0.0));
        pos.insert(2, (50.0, 10.0));
        pos.insert(3, (100.0, 0.0));

        let raw = build_steering_path(&route, &pos, false, 4);
        let smoothed = build_steering_path(&route, &pos, true, 4);

        assert!(
            smoothed.len() > raw.len(),
            "smoothed route should contain more points: raw={}, smoothed={}",
            raw.len(),
            smoothed.len()
        );
    }

    #[test]
    fn test_telemetry_priority_shm_first() {
        let shm_data = make_telemetry(10.0, 20.0, 0.4, 12.0);
        let (result, source) = resolve_telemetry(
            || Some(shm_data.clone()),
            || -> Result<TelemetryData, &'static str> {
                panic!("HTTP must not be called when SHM is available")
            },
        )
        .expect("expected telemetry data");

        assert_eq!(source, TelemetrySource::Shm);

        assert!((result.truck_placement.x - 10.0).abs() < 0.0001);
        assert!((result.truck_placement.z - 20.0).abs() < 0.0001);
        assert!((result.truck_float_values.speed - 12.0).abs() < 0.0001);
    }

    #[test]
    fn test_acc_integration() {
        let mut telemetry = make_telemetry(0.0, 0.0, 0.0, 20.0);
        telemetry.navigation_speed_limit = Some(80.0);
        telemetry.lead_vehicle_distance_m = Some(30.0);

        let mut cfg = TruckPilotConfig::default();
        cfg.acc.enabled = true;
        cfg.acc.target_distance_m = 50.0;
        cfg.acc.kp = 1.0;
        cfg.acc.ki = 0.0;
        cfg.acc.kd = 0.0;

        let mut acc = AccController::new(
            cfg.acc.target_distance_m,
            cfg.acc.kp,
            cfg.acc.ki,
            cfg.acc.kd,
        );

        let (target_speed_ms, _) = compute_target_speed_ms(&telemetry, &cfg, Some(&mut acc), 0.1);
        let target_speed_kmh = target_speed_ms * 3.6;
        assert!(
            target_speed_kmh < 80.0,
            "ACC should cap below nav-limit, got {target_speed_kmh}"
        );
    }
}
