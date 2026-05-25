//! Lane-Keeper plugin — dual-mode: route-following (Catmull-Rom) or vision-based.
//!
//! ## Mode selection
//! Set `plugin.lane_keeper.mode` on the Blackboard to `"vision"` or `"route_following"`.
//! Default (if key absent): `RouteFollowing` — preserves all existing behaviour.
//!
//! ## Vision mode — 5-Level Fallback Cascade (DS1 spec)
//! Level 0: Normal vision PID.  Level 1: Single-lane extrapolation.
//! Level 2: Confidence-drop (EMA).  Level 3: Heading-hold.  Level 4: Disengage.
//!
//! ## Route-following mode
//! Reads waypoints from `router.waypoints`.  Speed-adaptive look-ahead.
//! `look_ahead_m = BASE_LOOK_AHEAD + speed_kmh * SPEED_FACTOR`

mod extrapolation;
mod fallback;
mod heading_hold;

use extrapolation::{extrapolate_center, LaneWidthState};
use fallback::FallbackState;
use heading_hold::{wrap_angle, HeadingHoldState};
use truckpilot_plugin_api::{
    pid::Pid, ControlOutput, ControlRequest, Plugin, PluginContext, Telemetry,
};

// ── Priorities ────────────────────────────────────────────────────────────────
const PRIORITY_NORMAL: i32 = 50;
const PRIORITY_LEVEL4: i32 = 200;

// ── Route-following constants ─────────────────────────────────────────────────
const BASE_LOOK_AHEAD: f64 = 5.0;
const SPEED_FACTOR: f64 = 0.5;
const WAYPOINT_REACH_M: f64 = 5.0;

// ── PID defaults ──────────────────────────────────────────────────────────────
const DEFAULT_KP: f64 = 0.8;
const DEFAULT_KI: f64 = 0.1;
const DEFAULT_KD: f64 = 0.3;

/// Block-2: max heading error (radians) before lane-keeper suspends steering.
/// ~80°: covers normal curves/lane-changes (≤45°) but blocks clear mismatch cases.
pub(crate) const HEADING_MISMATCH_THRESHOLD_RAD: f64 = 1.4;

/// Block-2: max steering change per tick.
const STEERING_MAX_DELTA_PER_TICK: f64 = 0.1;

/// Ticks before Level-4 brake is lifted.
const L4_BRAKE_TICKS: u64 = 50;

// ── Mode enum ─────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum LaneKeeperMode {
    #[default]
    RouteFollowing,
    Vision,
    Off,
}

// ── Plugin struct ─────────────────────────────────────────────────────────────

pub struct LaneKeeperPlugin {
    pid: Pid,

    // ── Route-following fields ──────────────────────────────────────────────
    waypoints: Vec<[f64; 2]>,
    progress_idx: usize,
    subdivisions: usize,
    last_gains: (f64, f64, f64),
    last_waypoints_hash: u64,
    previous_steering_out: f64,
    heading_stage: Option<String>,
    previous_heading_stage: Option<String>,

    // ── Vision-mode fields ─────────────────────────────────────────────────
    mode: LaneKeeperMode,
    fallback: FallbackState,
    extrapolator: LaneWidthState,
    heading_hold: HeadingHoldState,
    /// Heading captured at Active-session start (for Block-2 guard in vision mode).
    engagement_heading: Option<f64>,
    /// Tick at which Level-4 was entered (None = not in L4).
    level_4_entered_at_tick: Option<u64>,
    /// Monotonic counter across all tick_request calls.
    tick_count: u64,
    /// Whether the last vision tick was in the Active state (for transition detection).
    was_active: bool,
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
            previous_steering_out: 0.0,
            heading_stage: None,
            previous_heading_stage: None,
            mode: LaneKeeperMode::default(),
            fallback: FallbackState::new(),
            extrapolator: LaneWidthState::new(),
            heading_hold: HeadingHoldState::new(),
            engagement_heading: None,
            level_4_entered_at_tick: None,
            tick_count: 0,
            was_active: false,
        }
    }
}

// ── Shared helpers ────────────────────────────────────────────────────────────

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

    fn update_mode_from_blackboard(&mut self, ctx: &PluginContext) {
        let raw = ctx.blackboard.get("plugin.lane_keeper.mode");
        let next = match raw.as_deref() {
            Some("vision") => LaneKeeperMode::Vision,
            Some("off") => LaneKeeperMode::Off,
            _ => LaneKeeperMode::RouteFollowing,
        };
        if next != self.mode {
            tracing::info!("[lane-keeper] mode switch {:?} → {:?}", self.mode, next);
            self.mode = next;
            self.pid.reset();
            self.fallback.reset();
            self.heading_hold.exit();
            self.engagement_heading = None;
            self.level_4_entered_at_tick = None;
            self.previous_steering_out = 0.0;
        }
    }

    /// Apply the rate-limiter and return the clamped steering value.
    fn rate_limit(&mut self, target: f64, ctx: &PluginContext) -> f64 {
        let delta = target - self.previous_steering_out;
        let clamped = delta.clamp(-STEERING_MAX_DELTA_PER_TICK, STEERING_MAX_DELTA_PER_TICK);
        let output = self.previous_steering_out + clamped;
        let was_limited = (clamped - delta).abs() > 1e-9;
        ctx.blackboard
            .set("lane_keeper.steering_rate_limited", was_limited.to_string());
        ctx.blackboard.set(
            "lane_keeper.steering_delta_clamped",
            format!("{:.4}", delta - clamped),
        );
        self.previous_steering_out = output;
        output
    }
}

// ── Route-following implementation ────────────────────────────────────────────

impl LaneKeeperPlugin {
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
                self.previous_steering_out = 0.0;
                self.pid.reset();
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

        while self.progress_idx + 1 < self.waypoints.len() {
            let [wx, wz] = self.waypoints[self.progress_idx + 1];
            let dist = ((tx - wx).powi(2) + (tz - wz).powi(2)).sqrt();
            if dist < WAYPOINT_REACH_M {
                self.progress_idx += 1;
            } else {
                break;
            }
        }

        if self.progress_idx + 1 >= self.waypoints.len() {
            ctx.blackboard.set("lane_keeper.skip_reason", "route_end");
            ctx.blackboard.set("lane_keeper.error_rad", "0.0");
            ctx.blackboard.set(
                "lane_keeper.progress_idx_after_advance",
                self.progress_idx.to_string(),
            );
            ctx.blackboard.set("lane_keeper.waypoints_remaining", "0");
            ctx.blackboard.set("lane_keeper.advance_check_dist", "0.00");
            ctx.blackboard.set("lane_keeper.walk_iterations", "0");
            ctx.blackboard.set("lane_keeper.walk_accumulated_m", "0.00");
            return 0.0;
        }

        let [nx, nz] = self.waypoints[self.progress_idx + 1];
        let advance_check_dist = ((tx - nx).powi(2) + (tz - nz).powi(2)).sqrt();

        let look_ahead = BASE_LOOK_AHEAD + speed_ms * 3.6 * SPEED_FACTOR;

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
        ctx.blackboard.set(
            "lane_keeper.walk_accumulated_m",
            format!("{accumulated:.2}"),
        );
        ctx.blackboard.set(
            "lane_keeper.advance_check_dist",
            format!("{advance_check_dist:.2}"),
        );
        ctx.blackboard.set(
            "lane_keeper.progress_idx_after_advance",
            self.progress_idx.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.waypoints_remaining",
            self.waypoints
                .len()
                .saturating_sub(self.progress_idx + 1)
                .to_string(),
        );

        let dx = look_x - tx;
        let dz = look_z - tz;
        if dx * dx + dz * dz < 1e-12 {
            return 0.0;
        }

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

    fn tick_request_route_following(
        &mut self,
        telemetry: Option<&Telemetry>,
        ctx: &PluginContext,
    ) -> Option<ControlRequest> {
        // heading_stage is set by tick() before tick_request(). In tests that call
        // tick_request() directly, the field is pre-set via struct literal.
        // Do NOT re-read from blackboard here — that would overwrite the pre-set value.

        if !ctx.is_active() {
            self.pid.reset();
            self.previous_steering_out = 0.0;
            if !self.waypoints.is_empty() {
                self.waypoints.clear();
                self.last_waypoints_hash = 0;
                self.progress_idx = 0;
                tracing::info!("[lane-keeper] state=Off, cleared waypoint cache");
            }
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard
                .set("lane_keeper.skip_reason", "state_not_active");
            return None;
        }

        self.apply_gain_overrides(ctx);

        let t = telemetry?;

        if t.engine_rpm < 100.0 {
            self.pid.reset();
            self.previous_steering_out = 0.0;
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard.set("lane_keeper.skip_reason", "engine_off");
            return None;
        }

        if self.waypoints.is_empty() {
            ctx.blackboard.set("lane_keeper.skip_reason", "no_waypoints");
            ctx.blackboard.set("lane_keeper.active", "false");
            return None;
        }

        let dt = ctx.dt_s.min(0.1);

        let err =
            self.compute_heading_error(t.position[0], t.position[2], t.heading, t.speed_ms, ctx);

        if err.abs() > HEADING_MISMATCH_THRESHOLD_RAD {
            ctx.blackboard
                .set("lane_keeper.skip_reason", "heading_mismatch");
            ctx.blackboard.set("lane_keeper.heading_mismatch", "true");
            ctx.blackboard
                .set("lane_keeper.error_rad", format!("{err:.6}"));
            ctx.blackboard.set("lane_keeper.steering_out", "0.000000");
            ctx.blackboard.set("lane_keeper.active", "true");
            ctx.blackboard
                .set("lane_keeper.steering_rate_limited", "false");
            ctx.blackboard
                .set("lane_keeper.steering_delta_clamped", "0.0000");
            self.previous_steering_out = 0.0;
            self.pid.reset();
            return None;
        }
        ctx.blackboard.set("lane_keeper.heading_mismatch", "false");

        let stage = self.heading_stage.as_deref().unwrap_or("Normal");

        if matches!(stage, "AutoReplan" | "Disengaging") {
            ctx.blackboard
                .set("lane_keeper.skip_reason", "heading_stage");
            ctx.blackboard.set("lane_keeper.heading_mismatch", "true");
            ctx.blackboard
                .set("lane_keeper.error_rad", format!("{err:.6}"));
            ctx.blackboard.set("lane_keeper.steering_out", "0.000000");
            ctx.blackboard.set("lane_keeper.active", "true");
            ctx.blackboard
                .set("lane_keeper.steering_rate_limited", "false");
            ctx.blackboard
                .set("lane_keeper.steering_delta_clamped", "0.0000");
            self.previous_steering_out = 0.0;
            self.pid.reset();
            return None;
        }

        let stage_changed = self.previous_heading_stage != self.heading_stage;
        let transitional = stage_changed
            && (self.heading_stage.as_deref() == Some("SoftLaneKeep")
                || self.previous_heading_stage.as_deref() == Some("SoftLaneKeep"));
        if transitional {
            self.pid.reset();
        }
        self.previous_heading_stage = self.heading_stage.clone();

        let effective_err = if self.heading_stage.as_deref() == Some("SoftLaneKeep") {
            err * 0.3
        } else {
            err
        };

        let raw = self.pid.update(effective_err, dt).clamp(-1.0, 1.0);

        let delta_raw = raw - self.previous_steering_out;
        let delta_clamped =
            delta_raw.clamp(-STEERING_MAX_DELTA_PER_TICK, STEERING_MAX_DELTA_PER_TICK);
        let steering = self.previous_steering_out + delta_clamped;
        self.previous_steering_out = steering;

        let was_rate_limited = (delta_clamped - delta_raw).abs() > 1e-9;
        ctx.blackboard.set(
            "lane_keeper.steering_rate_limited",
            was_rate_limited.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.steering_delta_clamped",
            format!("{:.4}", delta_raw - delta_clamped),
        );

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

// ── Vision-mode implementation ────────────────────────────────────────────────

impl LaneKeeperPlugin {
    fn tick_request_vision(
        &mut self,
        telemetry: Option<&Telemetry>,
        ctx: &PluginContext,
    ) -> Option<ControlRequest> {
        let is_active = ctx.is_active();
        let was_active = self.was_active;
        self.was_active = is_active;

        // Active→Off: reset steering state but keep fallback accumulating so
        // engage_allowed can be re-evaluated while disengaged.
        if !is_active && was_active {
            self.pid.reset();
            self.previous_steering_out = 0.0;
            self.heading_hold.exit();
            self.engagement_heading = None;
            self.level_4_entered_at_tick = None;
            ctx.blackboard.set("lane_keeper.active", "false");
        }

        // Off→Active: fresh PID start; engagement_heading captured below on first tick.
        if is_active && !was_active {
            self.pid.reset();
            self.previous_steering_out = 0.0;
        }

        self.apply_gain_overrides(ctx);

        // Engine gate — applies in both Active and Off states.
        let t = telemetry?;
        if t.engine_rpm < 100.0 {
            self.pid.reset();
            self.previous_steering_out = 0.0;
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard.set("lane_keeper.engage_allowed", "false");
            ctx.blackboard.set("lane_keeper.skip_reason", "engine_off");
            return None;
        }

        let dt = ctx.dt_s.min(0.1);

        // Read lane perception (no telemetry dependency — blackboard only).
        let center_offset = ctx.blackboard.get_f64("lane.center_offset").unwrap_or(0.0);
        let confidence = ctx.blackboard.get_f64("lane.confidence").unwrap_or(0.0);
        let left_vis = ctx.blackboard.get("lane.left_visible").as_deref() == Some("true");
        let right_vis = ctx.blackboard.get("lane.right_visible").as_deref() == Some("true");
        // NaN → None (absent lane)
        let left_x = ctx
            .blackboard
            .get_f64("lane.left_x")
            .filter(|x| x.is_finite());
        let right_x = ctx
            .blackboard
            .get_f64("lane.right_x")
            .filter(|x| x.is_finite());

        // Update fallback cascade — runs always so engage_allowed reflects real
        // lane quality even while the autopilot is still in Off state.
        self.fallback.push_confidence(confidence);
        self.fallback.push_offset(center_offset);
        self.extrapolator.advance_tick();
        let avg_conf = self.fallback.rolling_avg_confidence();
        let level = self.fallback.update(avg_conf, left_vis, right_vis);

        // Publish engage_allowed always — breaks the Off-state chicken-and-egg.
        let engage_allowed = level <= 1;
        ctx.blackboard.set(
            "lane_keeper.engage_allowed",
            if engage_allowed { "true" } else { "false" },
        );
        ctx.blackboard
            .set("lane_keeper.fallback_level", level.to_string());

        if !is_active {
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard
                .set("lane_keeper.skip_reason", "computing_engage_allowed");
            return None;
        }

        // ── Active-only path ────────────────────────────────────────────────────

        // Capture engagement heading on the first tick of each Active session.
        if self.engagement_heading.is_none() {
            self.engagement_heading = Some(t.heading);
            tracing::info!("[lane-keeper] vision engage heading={:.4}", t.heading);
        }

        // Block-2 guard: heading drift since engagement.
        let eng_heading = self.engagement_heading.unwrap_or(t.heading);
        let heading_drift = wrap_angle(t.heading - eng_heading).abs();
        if heading_drift > HEADING_MISMATCH_THRESHOLD_RAD {
            ctx.blackboard
                .set("lane_keeper.skip_reason", "heading_mismatch");
            ctx.blackboard.set("lane_keeper.heading_mismatch", "true");
            ctx.blackboard.set("lane_keeper.active", "true");
            self.previous_steering_out = 0.0;
            self.pid.reset();
            return None;
        }
        ctx.blackboard.set("lane_keeper.heading_mismatch", "false");

        // Heading stage gate.
        self.heading_stage = ctx.blackboard.get("state.heading_stage");
        let stage = self.heading_stage.as_deref().unwrap_or("Normal");
        if matches!(stage, "AutoReplan" | "Disengaging") {
            ctx.blackboard
                .set("lane_keeper.skip_reason", "heading_stage");
            ctx.blackboard.set("lane_keeper.active", "true");
            self.previous_steering_out = 0.0;
            self.pid.reset();
            return None;
        }

        // Heading-hold transitions.
        if level == 3 && !self.heading_hold.active {
            self.heading_hold
                .enter(t.heading, self.tick_count, self.previous_steering_out);
            ctx.blackboard.set(
                "lane_keeper.level_3_entered_at_tick",
                self.fallback.level_entered_at_tick.to_string(),
            );
            tracing::warn!("[lane-keeper] entering L3 heading-hold h={:.4}", t.heading);
        } else if level != 3 && self.heading_hold.active {
            self.heading_hold.exit();
        }

        // Level-4 disengage.
        if level == 4 {
            let l4_start = *self.level_4_entered_at_tick.get_or_insert(self.tick_count);
            let ticks_since = self.tick_count.saturating_sub(l4_start);

            if ticks_since == 0 {
                let event = format!(
                    r#"{{"tick":{},"reason":"{}","confidence":{:.4},"blind_ticks":{}}}"#,
                    self.tick_count,
                    self.fallback.fallback_reason,
                    confidence,
                    self.fallback.blind_tick_count,
                );
                ctx.blackboard.set("lane_keeper.level_4_event_json", event);
                tracing::error!("[lane-keeper] LEVEL-4 DISENGAGE tick={}", self.tick_count);
            }

            let brake = if ticks_since < L4_BRAKE_TICKS {
                Some(0.30)
            } else {
                None
            };
            ctx.blackboard.set("lane_keeper.fallback_level", "4");
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard.set("lane_keeper.steering_source", "none_l4");
            self.publish_vision_diagnostics(ctx, level, avg_conf, 0.0);

            return Some(ControlRequest {
                steering: None,
                brake,
                priority: PRIORITY_LEVEL4,
                ..Default::default()
            });
        }
        self.level_4_entered_at_tick = None;

        // Speed-adaptive gain.
        let speed_kmh = t.speed_ms * 3.6;
        let speed_gain = if speed_kmh < 60.0 {
            1.2
        } else if speed_kmh <= 100.0 {
            1.0
        } else {
            0.8
        };

        // Per-level steering computation.
        let (vision_error, gain_factor, steering_source) = match level {
            0 => {
                // Normal vision: error = -center_offset
                (-center_offset, 1.0 * speed_gain, "vision_l0")
            }
            1 => {
                // Single-lane extrapolation
                let (err, _) =
                    extrapolate_center(left_x, right_x, avg_conf, &mut self.extrapolator);
                (err, 0.6 * speed_gain, "extrapolation_l1")
            }
            2 => {
                // Confidence-drop: EMA-weighted, same error source as L0
                (-center_offset, 0.35 * speed_gain, "vision_l2")
            }
            3 => {
                // Heading-hold: separate PID path, return early
                let pid_ref = &mut self.pid;
                let steering_l3 = self.heading_hold.compute_steering(
                    t.heading,
                    self.tick_count,
                    &mut |err, dt_val| pid_ref.update(err, dt_val),
                    dt,
                );

                // SoftLaneKeep scaling
                let scaled = if self.heading_stage.as_deref() == Some("SoftLaneKeep") {
                    steering_l3 * 0.3
                } else {
                    steering_l3
                };

                let output = self.rate_limit(scaled, ctx);
                ctx.blackboard.set("lane_keeper.active", "true");
                ctx.blackboard.set("lane_keeper.fallback_level", "3");
                ctx.blackboard
                    .set("lane_keeper.heading_hold_active", "true");
                ctx.blackboard.set(
                    "lane_keeper.hold_heading_rad",
                    format!("{:.6}", self.heading_hold.hold_heading),
                );
                ctx.blackboard.set(
                    "lane_keeper.heading_drift_rad",
                    format!("{:.6}", self.heading_hold.heading_drift(t.heading)),
                );
                ctx.blackboard.set(
                    "lane_keeper.heading_hold_ticks",
                    self.heading_hold.ticks_active(self.tick_count).to_string(),
                );
                ctx.blackboard
                    .set("lane_keeper.steering_source", "heading_hold_l3");
                self.publish_vision_diagnostics(ctx, level, avg_conf, output);

                return Some(ControlRequest {
                    steering: Some(output),
                    priority: PRIORITY_NORMAL,
                    ..Default::default()
                });
            }
            _ => (-center_offset, 0.0, "unknown"),
        };

        // PID update (levels 0, 1, 2).
        let raw_pid = self.pid.update(vision_error, dt).clamp(-1.0, 1.0);
        let scaled = (raw_pid * gain_factor).clamp(-1.0, 1.0);

        // SoftLaneKeep scaling (levels 0, 1, 2).
        let effective = if self.heading_stage.as_deref() == Some("SoftLaneKeep") {
            scaled * 0.3
        } else {
            scaled
        };

        // Rate limiter (Block-2).
        let output = self.rate_limit(effective, ctx);

        ctx.blackboard.set("lane_keeper.active", "true");
        ctx.blackboard
            .set("lane_keeper.fallback_level", level.to_string());
        ctx.blackboard
            .set("lane_keeper.heading_hold_active", "false");
        ctx.blackboard
            .set("lane_keeper.steering_source", steering_source);
        self.publish_vision_diagnostics(ctx, level, avg_conf, output);

        Some(ControlRequest {
            steering: Some(output),
            priority: PRIORITY_NORMAL,
            ..Default::default()
        })
    }

    fn publish_vision_diagnostics(
        &self,
        ctx: &PluginContext,
        level: u8,
        avg_conf: f64,
        steering: f64,
    ) {
        ctx.blackboard.set(
            "lane_keeper.fallback_reason",
            self.fallback.fallback_reason.clone(),
        );
        ctx.blackboard.set(
            "lane_keeper.confidence_trend",
            format!("{:.4}", self.fallback.compute_confidence_trend()),
        );
        ctx.blackboard.set(
            "lane_keeper.detection_stability",
            format!("{:.4}", self.fallback.compute_detection_stability()),
        );
        ctx.blackboard.set(
            "lane_keeper.blind_ticks",
            self.fallback.blind_tick_count.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.blind_duration_ms",
            format!("{:.0}", self.fallback.blind_tick_count as f64 * 20.0),
        );
        ctx.blackboard.set("lane_keeper.single_lane_side", {
            let lv = ctx.blackboard.get("lane.left_visible").as_deref() == Some("true");
            let rv = ctx.blackboard.get("lane.right_visible").as_deref() == Some("true");
            match (lv, rv) {
                (true, false) => "left_only",
                (false, true) => "right_only",
                (true, true) => "both",
                (false, false) => "none",
            }
        });
        ctx.blackboard.set(
            "lane_keeper.lane_width_estimate_px",
            self.extrapolator
                .lane_width_estimate()
                .map(|w| format!("{w:.4}"))
                .unwrap_or_else(|| "NaN".to_string()),
        );
        ctx.blackboard
            .set("lane_keeper.steering_out", format!("{steering:.6}"));
        let _ = (level, avg_conf); // included in other keys already
    }
}

// ── Plugin trait impl ─────────────────────────────────────────────────────────

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
        // Pre-populate engage_allowed=false so the state machine never sees an absent key.
        ctx.blackboard.set("lane_keeper.engage_allowed", "false");
        tracing::info!("[lane-keeper] loaded, engage_allowed=false (pre-populated)");
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
        self.heading_stage = ctx.blackboard.get("state.heading_stage");

        if self.mode == LaneKeeperMode::RouteFollowing
            && ctx.blackboard.get("router.active").as_deref() == Some("true")
        {
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
        self.tick_count += 1;

        // Re-check mode every 50 ticks (and on first tick).
        if self.tick_count == 1 || self.tick_count.is_multiple_of(50) {
            self.update_mode_from_blackboard(ctx);
        }

        match self.mode {
            LaneKeeperMode::RouteFollowing => self.tick_request_route_following(telemetry, ctx),
            LaneKeeperMode::Vision => self.tick_request_vision(telemetry, ctx),
            LaneKeeperMode::Off => None,
        }
    }
}

// ── Route-following utilities ─────────────────────────────────────────────────

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

// ── Tests ─────────────────────────────────────────────────────────────────────

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
            nav_distance_m: -1.0,
            nav_time_s: -1.0,
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
        LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, -100.0], [0.0, -200.0]],
            ..Default::default()
        }
    }

    // ── Vision-mode helpers ───────────────────────────────────────────────────

    fn vision_bb(
        state: &str,
        center_offset: f64,
        confidence: f64,
        left: bool,
        right: bool,
    ) -> PluginContext {
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", state);
        bb.set("plugin.lane_keeper.mode", "vision");
        bb.set("lane.center_offset", center_offset.to_string());
        bb.set("lane.confidence", confidence.to_string());
        bb.set("lane.left_visible", left.to_string());
        bb.set("lane.right_visible", right.to_string());
        bb.set("lane.left_x", "NaN");
        bb.set("lane.right_x", "NaN");
        PluginContext::new("lane-keeper", bb)
    }

    fn make_vision_plugin() -> LaneKeeperPlugin {
        LaneKeeperPlugin {
            mode: LaneKeeperMode::Vision,
            ..Default::default()
        }
    }

    // ── Pre-existing route-following geometry tests ───────────────────────────

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
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
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
    fn heading_convention_north_is_zero() {
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
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [100.0, 0.0]],
            ..Default::default()
        };
        let ctx = fresh_ctx();
        let err = plugin.compute_heading_error(0.0, 0.0, std::f64::consts::FRAC_PI_2, 13.88, &ctx);
        assert!(err.abs() < 0.01, "expected ~0, got {err}");
    }

    #[test]
    fn heading_convention_punkt_vor_rechts_kleiner_positiver_error() {
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
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let active = ctx_with_state("Active");
        for _ in 0..10 {
            let _ = lk.tick_request(Some(&t), &active);
        }
        let off = ctx_with_state("Off");
        assert!(lk.tick_request(Some(&t), &off).is_none());

        lk.waypoints = vec![[0.0, 0.0], [20.0, -100.0]];

        let mut fresh = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
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
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", "Active");
        let ctx = PluginContext::new("lane-keeper", bb).with_dt(10.0);
        let req = lk.tick_request(Some(&t), &ctx).unwrap();
        let s = req.steering.unwrap();
        assert!((-1.0..=1.0).contains(&s), "output out of range: {s}");
    }

    #[test]
    fn route_end_returns_zero_error() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [10.0, 0.0]],
            progress_idx: 1,
            ..Default::default()
        };
        let ctx = fresh_ctx();
        let err = plugin.compute_heading_error(9.0, 0.0, 0.0, 0.0, &ctx);
        assert_eq!(err, 0.0, "Route-End-Guard must return 0.0, got {err}");
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("route_end"),
        );
    }

    #[test]
    fn progress_advances_when_truck_near_next_waypoint() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [10.0, 0.0], [20.0, 0.0]],
            progress_idx: 0,
            ..Default::default()
        };
        let ctx = fresh_ctx();
        plugin.compute_heading_error(6.0, 0.0, 0.0, 0.0, &ctx);
        assert_eq!(plugin.progress_idx, 1);
    }

    #[test]
    fn lookahead_starts_from_truck_position() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, 0.0], [40.0, 0.0]],
            progress_idx: 0,
            ..Default::default()
        };
        let ctx = fresh_ctx();
        plugin.compute_heading_error(0.0, 0.0, 0.0, 0.0, &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.look_x").as_deref(),
            Some("20.00")
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.look_z").as_deref(),
            Some("0.00")
        );
    }

    #[test]
    fn lookahead_walks_through_multiple_waypoints() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [3.0, 0.0], [6.0, 0.0], [9.0, 0.0], [12.0, 0.0]],
            progress_idx: 0,
            ..Default::default()
        };
        let ctx = fresh_ctx();
        plugin.compute_heading_error(0.0, 0.0, 0.0, 0.0, &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.walk_iterations").as_deref(),
            Some("1")
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.look_x").as_deref(),
            Some("6.00")
        );
    }

    #[test]
    fn heading_mismatch_above_threshold_returns_none() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, 100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        assert!(lk.tick_request(Some(&t), &ctx).is_none());
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("heading_mismatch")
        );
    }

    #[test]
    fn heading_mismatch_at_threshold_is_allowed() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        assert!(lk.tick_request(Some(&t), &ctx).is_some());
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.heading_mismatch")
                .as_deref(),
            Some("false")
        );
    }

    #[test]
    fn steering_rate_limit_clamps_large_delta() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let req = lk.tick_request(Some(&t), &ctx).unwrap();
        let s = req.steering.unwrap();
        assert!(s.abs() <= STEERING_MAX_DELTA_PER_TICK + 1e-9);
    }

    #[test]
    fn steering_rate_limit_passes_small_delta() {
        let mut lk = active_plugin_with_straight_path();
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let req = lk.tick_request(Some(&t), &ctx).unwrap();
        let s = req.steering.unwrap();
        assert!(s.abs() < STEERING_MAX_DELTA_PER_TICK);
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.steering_rate_limited")
                .as_deref(),
            Some("false")
        );
    }

    #[test]
    fn previous_steering_resets_on_disengage() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let active = ctx_with_state("Active");
        for _ in 0..5 {
            let _ = lk.tick_request(Some(&t), &active);
        }
        assert!(lk.previous_steering_out.abs() > 0.0);
        let off = ctx_with_state("Off");
        lk.tick_request(Some(&t), &off);
        assert_eq!(lk.previous_steering_out, 0.0);
    }

    #[test]
    fn previous_steering_resets_on_heading_mismatch() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let active = ctx_with_state("Active");
        for _ in 0..5 {
            let _ = lk.tick_request(Some(&t), &active);
        }
        assert!(lk.previous_steering_out.abs() > 0.0);
        lk.waypoints = vec![[0.0, 0.0], [0.0, 100.0]];
        let _ = lk.tick_request(Some(&t), &active);
        assert_eq!(lk.previous_steering_out, 0.0);
    }

    #[test]
    fn heading_convention_north_regression_unaffected_by_mismatch() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let req = lk.tick_request(Some(&t), &ctx);
        assert!(req.is_some());
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.heading_mismatch")
                .as_deref(),
            Some("false")
        );
    }

    #[test]
    fn auto_replan_stage_returns_none() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            heading_stage: Some("AutoReplan".to_string()),
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        assert!(lk.tick_request(Some(&t), &ctx).is_none());
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("heading_stage")
        );
    }

    #[test]
    fn disengaging_stage_returns_none() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            heading_stage: Some("Disengaging".to_string()),
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        assert!(lk.tick_request(Some(&t), &ctx).is_none());
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("heading_stage")
        );
    }

    #[test]
    fn stage_change_soft_to_normal_resets_pid() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            heading_stage: Some("SoftLaneKeep".to_string()),
            previous_heading_stage: Some("Normal".to_string()),
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let _ = lk.tick_request(Some(&t), &ctx);
        assert_eq!(lk.previous_heading_stage, Some("SoftLaneKeep".to_string()));
    }

    #[test]
    fn normal_stage_produces_steering() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            heading_stage: Some("Normal".to_string()),
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let result = lk.tick_request(Some(&t), &ctx);
        assert!(result.is_some());
        assert!(result.unwrap().steering.is_some());
    }

    #[test]
    fn phase_6_5p_guard_takes_priority_over_heading_stage() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, 100.0]],
            heading_stage: Some("Normal".to_string()),
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let result = lk.tick_request(Some(&t), &ctx);
        assert!(result.is_none());
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("heading_mismatch")
        );
    }

    #[test]
    fn route_following_no_waypoints_yields() {
        let mut lk = LaneKeeperPlugin::default();
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        assert!(lk.tick_request(Some(&t), &ctx).is_none());
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("no_waypoints")
        );
    }

    // ── Vision-mode tests ─────────────────────────────────────────────────────

    #[test]
    fn vision_level0_produces_steering() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        let ctx = vision_bb("Active", 0.1, 0.85, true, true);
        let req = plugin
            .tick_request(Some(&t), &ctx)
            .expect("L0 must produce steering");
        assert!(req.steering.is_some());
        assert_eq!(req.priority, PRIORITY_NORMAL);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_level").as_deref(),
            Some("0")
        );
    }

    #[test]
    fn vision_state_gate_returns_none_when_off() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        let ctx = vision_bb("Off", 0.0, 0.9, true, true);
        assert!(plugin.tick_request(Some(&t), &ctx).is_none());
        assert_eq!(
            ctx.blackboard.get("lane_keeper.active").as_deref(),
            Some("false")
        );
    }

    #[test]
    fn vision_engine_off_returns_none() {
        let mut plugin = make_vision_plugin();
        let mut t = make_telemetry(20.0, 0.0);
        t.engine_rpm = 0.0;
        let ctx = vision_bb("Active", 0.0, 0.9, true, true);
        assert!(plugin.tick_request(Some(&t), &ctx).is_none());
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("engine_off")
        );
    }

    #[test]
    fn vision_engage_allowed_true_at_level0() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // Pump 5 ticks to build rolling avg > 0.70
        for _ in 0..5 {
            let ctx = vision_bb("Active", 0.0, 0.85, true, true);
            let _ = plugin.tick_request(Some(&t), &ctx);
        }
        let ctx = vision_bb("Active", 0.0, 0.85, true, true);
        let _ = plugin.tick_request(Some(&t), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.engage_allowed").as_deref(),
            Some("true")
        );
    }

    #[test]
    fn vision_engage_allowed_false_at_level2() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // Low confidence → level 2
        for _ in 0..5 {
            let ctx = vision_bb("Active", 0.0, 0.20, true, true);
            let _ = plugin.tick_request(Some(&t), &ctx);
        }
        let ctx = vision_bb("Active", 0.0, 0.20, true, true);
        let _ = plugin.tick_request(Some(&t), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.engage_allowed").as_deref(),
            Some("false")
        );
    }

    #[test]
    fn vision_level4_produces_high_priority_request() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // 105 blind ticks → L4
        for _ in 0..105 {
            let ctx = vision_bb("Active", 0.0, 0.0, false, false);
            let _ = plugin.tick_request(Some(&t), &ctx);
        }
        let ctx = vision_bb("Active", 0.0, 0.0, false, false);
        let req = plugin
            .tick_request(Some(&t), &ctx)
            .expect("L4 must produce ControlRequest");
        assert_eq!(req.priority, PRIORITY_LEVEL4);
        assert!(req.steering.is_none(), "L4 must not steer");
        assert_eq!(
            ctx.blackboard.get("lane_keeper.active").as_deref(),
            Some("false")
        );
    }

    #[test]
    fn vision_level4_brake_first_50_ticks() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // Get to L4
        for _ in 0..105 {
            let ctx = vision_bb("Active", 0.0, 0.0, false, false);
            let _ = plugin.tick_request(Some(&t), &ctx);
        }
        // First L4 tick → brake = 0.30
        let ctx = vision_bb("Active", 0.0, 0.0, false, false);
        let req = plugin.tick_request(Some(&t), &ctx).unwrap();
        assert_eq!(req.brake, Some(0.30), "first L4 tick must brake at 0.30");
    }

    #[test]
    fn vision_center_offset_steers_toward_lane_center() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // Positive center_offset = truck to the right → steer left (negative)
        // vision_error = -center_offset = -0.3 → negative PID → negative steering
        // Pump a few ticks so rolling avg stabilises before the final assertion.
        for _ in 0..5 {
            let c = vision_bb("Active", 0.3, 0.85, true, true);
            let _ = plugin.tick_request(Some(&t), &c);
        }
        let ctx = vision_bb("Active", 0.3, 0.85, true, true);
        let req = plugin.tick_request(Some(&t), &ctx).unwrap();
        let s = req.steering.unwrap();
        assert!(s < 0.0, "positive offset → steer left (negative), got {s}");
    }

    #[test]
    fn vision_rate_limiter_clamps_first_tick() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // Large offset → PID would want large output, rate-limiter clamps it
        let ctx = vision_bb("Active", 1.0, 0.95, true, true);
        let req = plugin.tick_request(Some(&t), &ctx).unwrap();
        if let Some(s) = req.steering {
            assert!(
                s.abs() <= STEERING_MAX_DELTA_PER_TICK + 1e-9,
                "rate limiter must clamp first-tick output, got {s}"
            );
        }
    }

    #[test]
    fn vision_reset_on_off_clears_engagement_heading() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        let ctx = vision_bb("Active", 0.0, 0.85, true, true);
        let _ = plugin.tick_request(Some(&t), &ctx);
        assert!(plugin.engagement_heading.is_some());

        let ctx_off = vision_bb("Off", 0.0, 0.0, false, false);
        let _ = plugin.tick_request(Some(&t), &ctx_off);
        assert!(plugin.engagement_heading.is_none());
    }

    #[test]
    fn vision_on_load_publishes_engage_allowed_false() {
        let mut plugin = make_vision_plugin();
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", "Off");
        let ctx = PluginContext::new("lane-keeper", bb);
        plugin.on_load(&ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.engage_allowed").as_deref(),
            Some("false")
        );
    }

    // ── Off-state engage_allowed tests (chicken-and-egg fix) ──────────────────

    #[test]
    fn vision_off_state_publishes_engage_allowed_with_good_lane_detection() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // Pump ticks in Off state with valid lane data — engage_allowed must become true.
        for _ in 0..5 {
            let ctx = vision_bb("Off", 0.0, 0.85, true, true);
            let result = plugin.tick_request(Some(&t), &ctx);
            assert!(result.is_none(), "Off state must produce no steering");
        }
        let ctx = vision_bb("Off", 0.0, 0.85, true, true);
        let result = plugin.tick_request(Some(&t), &ctx);
        assert!(result.is_none(), "Off state must produce no steering");
        assert_eq!(
            ctx.blackboard.get("lane_keeper.engage_allowed").as_deref(),
            Some("true")
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_level").as_deref(),
            Some("0")
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.active").as_deref(),
            Some("false")
        );
    }

    #[test]
    fn vision_off_state_produces_no_steering_output() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        let ctx = vision_bb("Off", 0.0, 0.85, true, true);
        assert!(plugin.tick_request(Some(&t), &ctx).is_none());
    }

    #[test]
    fn vision_active_state_produces_steering_output() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // Warm up fallback in Active state so rolling avg is stable.
        for _ in 0..5 {
            let ctx = vision_bb("Active", 0.0, 0.85, true, true);
            let _ = plugin.tick_request(Some(&t), &ctx);
        }
        let ctx = vision_bb("Active", 0.0, 0.85, true, true);
        let req = plugin
            .tick_request(Some(&t), &ctx)
            .expect("Active must produce ControlRequest");
        assert!(req.steering.is_some(), "Active must produce steering");
    }
}
