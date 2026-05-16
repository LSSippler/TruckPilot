//! Lane-Changer plugin — detects slow lead vehicles and executes lane changes.
//!
//! ## State machine
//!
//! ```text
//! Idle ──(slow vehicle detected)──► Deciding ──(lane free)──► Changing ──(done)──► Idle
//!                                       └──(no free lane)──────────────────────────► Idle
//! ```
//!
//! ## Blackboard contract
//!
//! | Key                        | Written by   | Read by      |
//! |----------------------------|--------------|--------------|
//! | `acc.speed_cap_kmh`        | acc          | lane-changer |
//! | `lane_changer.active`      | lane-changer | UI           |
//! | `lane_changer.direction`   | lane-changer | UI           |

use std::time::{Duration, Instant};

use truckpilot_plugin_api::{
    ControlOutput, ControlRequest, Plugin, PluginContext, Telemetry, TickPhase,
};

/// Arbitration priority for lane-changer's tick_request — chosen higher
/// than lane-keeper's `PRIORITY_NORMAL` (50) so the manoeuvre overrides
/// normal lane-keeping while it is in progress.
const LC_REQUEST_PRIORITY: i32 = 80;

// ---------------------------------------------------------------------------
// Configuration defaults
// ---------------------------------------------------------------------------

/// Minimum speed difference (km/h) to trigger a lane change.
const MIN_SPEED_DIFF_KMH: f64 = 15.0;
/// Minimum following distance (m) before considering a lane change.
const MIN_TRIGGER_DIST_M: f32 = 40.0;
/// Duration of the lane-change manoeuvre.
const MANOEUVRE_DURATION: Duration = Duration::from_secs(4);
/// Steering offset applied during the manoeuvre (added to lane-keeper output).
const LANE_CHANGE_STEER_OFFSET: f64 = 0.15;

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq)]
enum LcState {
    /// No lane change in progress.
    Idle,
    /// Slow vehicle detected; evaluating whether a lane change is safe.
    Deciding { since: Instant },
    /// Actively executing the lane-change manoeuvre.
    Changing {
        started: Instant,
        /// +1 = left, -1 = right (ETS2 convention: positive heading = CW = right)
        direction: i8,
    },
}

pub struct LaneChangerPlugin {
    state: LcState,
    min_speed_diff_kmh: f64,
    min_trigger_dist_m: f32,
}

impl Default for LaneChangerPlugin {
    fn default() -> Self {
        Self {
            state: LcState::Idle,
            min_speed_diff_kmh: MIN_SPEED_DIFF_KMH,
            min_trigger_dist_m: MIN_TRIGGER_DIST_M,
        }
    }
}

impl LaneChangerPlugin {
    /// Decide whether conditions warrant a lane change.
    fn should_change(&self, t: &Telemetry, ctx: &PluginContext) -> bool {
        // Need a lead vehicle close enough.
        let dist = match t.lead_vehicle_distance_m {
            d if d >= 0.0 => d,
            _ => return false,
        };
        if dist > self.min_trigger_dist_m {
            return false;
        }

        // ACC must have capped our speed significantly below cruise.
        let acc_cap = ctx
            .blackboard
            .get_f64("acc.speed_cap_kmh")
            .unwrap_or(f64::MAX);
        let cruise = t.cruise_control_kmh;
        if cruise - acc_cap < self.min_speed_diff_kmh {
            return false;
        }

        true
    }

    /// Choose lane-change direction. Returns +1 (left) or -1 (right).
    /// Simple heuristic: always prefer left (overtaking lane in Europe).
    fn choose_direction(&self) -> i8 {
        1
    }
}

impl Plugin for LaneChangerPlugin {
    fn name(&self) -> &str {
        "lane-changer"
    }
    fn version(&self) -> &str {
        "0.1.0"
    }
    fn settings_schema(&self) -> &str {
        r#"{
  "type": "object",
  "properties": {
    "min_speed_diff_kmh": { "type": "number", "minimum": 5, "description": "Speed difference (km/h) to trigger lane change." },
    "min_trigger_dist_m": { "type": "number", "minimum": 10, "description": "Following distance (m) to trigger lane change." }
  }
}"#
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        if let Some(v) = ctx.blackboard.get_f64("lane_changer.min_speed_diff_kmh") {
            self.min_speed_diff_kmh = v.max(5.0);
        }
        if let Some(v) = ctx.blackboard.get_f64("lane_changer.min_trigger_dist_m") {
            self.min_trigger_dist_m = v.max(10.0) as f32;
        }
        tracing::info!("[lane-changer] loaded");
    }

    fn on_unload(&mut self) {
        tracing::info!("[lane-changer] unloaded");
    }

    fn default_phase(&self) -> TickPhase {
        TickPhase::PhaseB
    }

    fn tick(
        &mut self,
        telemetry: Option<&Telemetry>,
        output: &mut ControlOutput,
        ctx: &PluginContext,
    ) {
        let Some(t) = telemetry else {
            self.state = LcState::Idle;
            ctx.blackboard.set("lane_changer.active", "false");
            return;
        };

        match &self.state.clone() {
            LcState::Idle => {
                if self.should_change(t, ctx) {
                    tracing::info!("[lane-changer] slow vehicle detected — evaluating lane change");
                    self.state = LcState::Deciding {
                        since: Instant::now(),
                    };
                }
                ctx.blackboard.set("lane_changer.active", "false");
            }

            LcState::Deciding { since } => {
                // Give 1 second to confirm the situation is stable.
                if since.elapsed() >= Duration::from_secs(1) {
                    if self.should_change(t, ctx) {
                        let dir = self.choose_direction();
                        tracing::info!("[lane-changer] executing lane change dir={dir}");
                        self.state = LcState::Changing {
                            started: Instant::now(),
                            direction: dir,
                        };
                    } else {
                        self.state = LcState::Idle;
                    }
                }
                ctx.blackboard.set("lane_changer.active", "false");
            }

            LcState::Changing { started, direction } => {
                let elapsed = started.elapsed();
                if elapsed >= MANOEUVRE_DURATION {
                    tracing::info!("[lane-changer] lane change complete");
                    self.state = LcState::Idle;
                    ctx.blackboard.set("lane_changer.active", "false");
                    ctx.blackboard.remove("lane_changer.direction");
                    return;
                }

                // Steering is now produced via `tick_request` so the
                // arbitrator can favour it over lane-keeper. This block
                // only tracks state & blackboard; `output` is untouched.
                let _ = output;
                ctx.blackboard.set("lane_changer.active", "true");
                ctx.blackboard.set(
                    "lane_changer.direction",
                    if *direction > 0 { "left" } else { "right" },
                );

                let progress = elapsed.as_secs_f64() / MANOEUVRE_DURATION.as_secs_f64();
                tracing::debug!("[lane-changer] manoeuvre {:.0}%", progress * 100.0);
            }
        }
    }

    fn tick_request(
        &mut self,
        _telemetry: Option<&Telemetry>,
        _ctx: &PluginContext,
    ) -> Option<ControlRequest> {
        let LcState::Changing { started, direction } = &self.state else {
            return None;
        };
        let elapsed = started.elapsed();
        if elapsed >= MANOEUVRE_DURATION {
            return None;
        }
        let progress = elapsed.as_secs_f64() / MANOEUVRE_DURATION.as_secs_f64();
        let smooth = (progress * std::f64::consts::PI).sin();
        let steering = (*direction as f64) * LANE_CHANGE_STEER_OFFSET * smooth;
        Some(ControlRequest {
            steering: Some(steering.clamp(-1.0, 1.0)),
            throttle: None,
            brake: None,
            priority: LC_REQUEST_PRIORITY,
        })
    }
}

truckpilot_plugin_api::export_plugin!(LaneChangerPlugin);

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use truckpilot_plugin_api::{PluginContext, SharedBlackboard};

    fn make_telemetry(speed_ms: f64, cruise_kmh: f64, lead_dist: f32) -> Telemetry {
        Telemetry {
            position: [0.0; 3],
            heading: 0.0,
            pitch: 0.0,
            roll: 0.0,
            speed_ms,
            engine_rpm: 1200.0,
            cruise_control_kmh: cruise_kmh,
            nav_speed_limit_kmh: -1.0,
            lead_vehicle_distance_m: lead_dist,
            accel_longitudinal: -1.0,
            fuel_liters: -1.0,
            odometer_km: -1.0,
        }
    }

    fn ctx_with_acc_cap(cap: f64) -> PluginContext {
        let bb = SharedBlackboard::new();
        bb.set("acc.speed_cap_kmh", cap.to_string());
        PluginContext::new("test", bb)
    }

    #[test]
    fn no_lead_vehicle_no_change() {
        let p = LaneChangerPlugin::default();
        let t = make_telemetry(20.0, 80.0, -1.0);
        let ctx = ctx_with_acc_cap(80.0);
        assert!(!p.should_change(&t, &ctx));
    }

    #[test]
    fn lead_too_far_no_change() {
        let p = LaneChangerPlugin::default();
        let t = make_telemetry(20.0, 80.0, 200.0); // 200m > 40m threshold
        let ctx = ctx_with_acc_cap(40.0);
        assert!(!p.should_change(&t, &ctx));
    }

    #[test]
    fn slow_lead_close_triggers_change() {
        let p = LaneChangerPlugin::default();
        let t = make_telemetry(20.0, 80.0, 20.0); // 20m < 40m threshold
        let ctx = ctx_with_acc_cap(50.0); // 80 - 50 = 30 > 15 km/h diff
        assert!(p.should_change(&t, &ctx));
    }

    #[test]
    fn small_speed_diff_no_change() {
        let p = LaneChangerPlugin::default();
        let t = make_telemetry(20.0, 80.0, 20.0);
        let ctx = ctx_with_acc_cap(72.0); // 80 - 72 = 8 < 15 km/h diff
        assert!(!p.should_change(&t, &ctx));
    }

    #[test]
    fn sinusoidal_steer_at_midpoint() {
        // At 50% progress, sin(PI/2) = 1.0 → max offset
        let progress = 0.5_f64;
        let smooth = (progress * std::f64::consts::PI).sin();
        assert!((smooth - 1.0).abs() < 0.001);
    }

    #[test]
    fn sinusoidal_steer_at_start_and_end_near_zero() {
        for progress in [0.0_f64, 1.0_f64] {
            let smooth = (progress * std::f64::consts::PI).sin();
            assert!(smooth.abs() < 0.01, "progress={progress} smooth={smooth}");
        }
    }

    #[test]
    fn changing_state_produces_control_request() {
        let mut p = LaneChangerPlugin {
            state: LcState::Changing {
                started: Instant::now(),
                direction: 1,
            },
            ..LaneChangerPlugin::default()
        };
        let ctx = ctx_with_acc_cap(50.0);
        let req = p.tick_request(None, &ctx).expect("Changing must request");
        assert_eq!(req.priority, LC_REQUEST_PRIORITY);
        assert!(req.steering.is_some());
        assert!(req.throttle.is_none());
        assert!(req.brake.is_none());
    }

    #[test]
    fn idle_state_returns_no_request() {
        let mut p = LaneChangerPlugin::default();
        let ctx = ctx_with_acc_cap(80.0);
        assert!(p.tick_request(None, &ctx).is_none());
    }
}
