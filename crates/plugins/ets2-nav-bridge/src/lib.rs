//! ETS2 Nav-Bridge — translates GPS telemetry fields into blackboard events.
//!
//! Runs at PhaseA (1 Hz). Reads `nav_distance_m` and `nav_time_s` from the
//! telemetry snapshot and publishes a set of derived keys that other plugins
//! can subscribe to without touching raw telemetry.
//!
//! ## Blackboard contract
//!
//! | Key                            | Direction | Format               |
//! |--------------------------------|-----------|----------------------|
//! | `nav.gps_route_set`            | written   | "true"/"false"       |
//! | `nav.distance_to_turn`         | written   | f32 as string (m); absent when no route |
//! | `nav.time_to_turn`             | written   | f32 as string (s); absent when no route |
//! | `nav.approaching_junction`     | written   | "true"/"false"       |
//! | `nav.waypoint_passed`          | written   | "true" for 1 tick, then "false" |
//! | `nav.last_waypoint_passed_ts`  | written   | i64 ms timestamp     |
//!
//! ## Hysteresis thresholds
//!
//! `approaching_junction` turns **on** when `distance_to_turn < 150 m`,
//! turns **off** when `distance_to_turn > 200 m`. Between 150 m and 200 m
//! the state is held (prevents rapid toggling near the threshold).

use std::time::{SystemTime, UNIX_EPOCH};

use truckpilot_plugin_api::{ControlOutput, Plugin, PluginContext, Telemetry, TickPhase};

const APPROACH_ON_M: f32 = 150.0;
const APPROACH_OFF_M: f32 = 200.0;
const WAYPOINT_JUMP_M: f32 = 100.0;

#[derive(Default)]
pub struct Ets2NavBridgePlugin {
    prev_distance: Option<f32>,
    approaching: bool,
    waypoint_set_last_tick: bool,
}

impl Plugin for Ets2NavBridgePlugin {
    fn name(&self) -> &str {
        "ets2-nav-bridge"
    }
    fn version(&self) -> &str {
        "0.1.0"
    }
    fn settings_schema(&self) -> &str {
        r#"{"type":"object","properties":{}}"#
    }
    fn default_phase(&self) -> TickPhase {
        TickPhase::PhaseA
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        ctx.blackboard.set("nav.gps_route_set", "false");
        ctx.blackboard.set("nav.approaching_junction", "false");
        ctx.blackboard.set("nav.waypoint_passed", "false");
        ctx.blackboard.remove("nav.distance_to_turn");
        ctx.blackboard.remove("nav.time_to_turn");
        tracing::info!("[ets2-nav-bridge] loaded");
    }

    fn on_unload(&mut self) {
        tracing::info!("[ets2-nav-bridge] unloaded");
    }

    fn tick(
        &mut self,
        telemetry: Option<&Telemetry>,
        _output: &mut ControlOutput,
        ctx: &PluginContext,
    ) {
        // Clear one-shot flag from previous tick.
        if self.waypoint_set_last_tick {
            ctx.blackboard.set("nav.waypoint_passed", "false");
            self.waypoint_set_last_tick = false;
        }

        let (dist, time) = match telemetry {
            Some(t) => (t.nav_distance_m, t.nav_time_s),
            None => {
                ctx.blackboard.set("nav.gps_route_set", "false");
                ctx.blackboard.remove("nav.distance_to_turn");
                ctx.blackboard.remove("nav.time_to_turn");
                return;
            }
        };

        // Sentinel (-1.0) or zero means no active route.
        let route_active = dist > 0.0 && (dist - (-1.0_f32)).abs() > 0.01;
        ctx.blackboard.set(
            "nav.gps_route_set",
            if route_active { "true" } else { "false" },
        );

        if !route_active {
            ctx.blackboard.remove("nav.distance_to_turn");
            ctx.blackboard.remove("nav.time_to_turn");
            if self.approaching {
                self.approaching = false;
                ctx.blackboard.set("nav.approaching_junction", "false");
            }
            self.prev_distance = None;
            return;
        }

        ctx.blackboard.set("nav.distance_to_turn", dist.to_string());
        ctx.blackboard.set("nav.time_to_turn", time.to_string());

        // Hysteresis: on below APPROACH_ON_M, off above APPROACH_OFF_M.
        if !self.approaching && dist < APPROACH_ON_M {
            self.approaching = true;
            ctx.blackboard.set("nav.approaching_junction", "true");
            tracing::debug!("[ets2-nav-bridge] approaching junction at {dist:.0} m");
        } else if self.approaching && dist > APPROACH_OFF_M {
            self.approaching = false;
            ctx.blackboard.set("nav.approaching_junction", "false");
        }

        // Waypoint-passed: distance jumped up by > WAYPOINT_JUMP_M (re-route or new leg).
        if let Some(prev) = self.prev_distance {
            if dist > prev + WAYPOINT_JUMP_M {
                let ts_ms = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_millis() as i64;
                ctx.blackboard.set("nav.waypoint_passed", "true");
                ctx.blackboard
                    .set("nav.last_waypoint_passed_ts", ts_ms.to_string());
                self.waypoint_set_last_tick = true;
                tracing::info!("[ets2-nav-bridge] waypoint_passed: {prev:.0} m → {dist:.0} m");
            }
        }

        self.prev_distance = Some(dist);
    }
}

truckpilot_plugin_api::export_plugin!(Ets2NavBridgePlugin);

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use truckpilot_plugin_api::{ControlOutput, PluginContext, Telemetry, TickPhase};

    use super::*;

    fn mock_tel(nav_distance_m: f32, nav_time_s: f32) -> Telemetry {
        Telemetry {
            position: [0.0, 0.0, 0.0],
            heading: 0.0,
            pitch: 0.0,
            roll: 0.0,
            speed_ms: 0.0,
            engine_rpm: 0.0,
            engine_gear: 0,
            cruise_control_kmh: 0.0,
            nav_speed_limit_kmh: -1.0,
            lead_vehicle_distance_m: -1.0,
            accel_longitudinal: -1.0,
            fuel_liters: -1.0,
            odometer_km: -1.0,
            nav_distance_m,
            nav_time_s,
        }
    }

    fn tick(plugin: &mut Ets2NavBridgePlugin, t: Option<&Telemetry>, ctx: &PluginContext) {
        plugin.tick(t, &mut ControlOutput::default(), ctx);
    }

    #[test]
    fn default_phase_is_phase_a() {
        assert_eq!(
            Ets2NavBridgePlugin::default().default_phase(),
            TickPhase::PhaseA
        );
    }

    #[test]
    fn on_load_initializes_keys() {
        let mut plugin = Ets2NavBridgePlugin::default();
        let ctx = PluginContext::test();
        plugin.on_load(&ctx);
        assert_eq!(
            ctx.blackboard.get("nav.gps_route_set").as_deref(),
            Some("false")
        );
        assert_eq!(
            ctx.blackboard.get("nav.approaching_junction").as_deref(),
            Some("false")
        );
        assert_eq!(
            ctx.blackboard.get("nav.waypoint_passed").as_deref(),
            Some("false")
        );
        assert!(ctx.blackboard.get("nav.distance_to_turn").is_none());
        assert!(ctx.blackboard.get("nav.time_to_turn").is_none());
    }

    #[test]
    fn no_telemetry_sets_route_inactive() {
        let mut plugin = Ets2NavBridgePlugin::default();
        let ctx = PluginContext::test();
        tick(&mut plugin, None, &ctx);
        assert_eq!(
            ctx.blackboard.get("nav.gps_route_set").as_deref(),
            Some("false")
        );
        assert!(ctx.blackboard.get("nav.distance_to_turn").is_none());
    }

    #[test]
    fn sentinel_distance_sets_route_inactive() {
        let mut plugin = Ets2NavBridgePlugin::default();
        let ctx = PluginContext::test();
        tick(&mut plugin, Some(&mock_tel(-1.0, -1.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("nav.gps_route_set").as_deref(),
            Some("false")
        );
        assert!(ctx.blackboard.get("nav.distance_to_turn").is_none());
    }

    #[test]
    fn zero_distance_sets_route_inactive() {
        let mut plugin = Ets2NavBridgePlugin::default();
        let ctx = PluginContext::test();
        tick(&mut plugin, Some(&mock_tel(0.0, 0.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("nav.gps_route_set").as_deref(),
            Some("false")
        );
    }

    #[test]
    fn valid_distance_publishes_all_keys() {
        let mut plugin = Ets2NavBridgePlugin::default();
        let ctx = PluginContext::test();
        tick(&mut plugin, Some(&mock_tel(500.0, 60.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("nav.gps_route_set").as_deref(),
            Some("true")
        );
        assert!(ctx.blackboard.get("nav.distance_to_turn").is_some());
        assert!(ctx.blackboard.get("nav.time_to_turn").is_some());
    }

    #[test]
    fn approaching_junction_triggers_below_threshold() {
        let mut plugin = Ets2NavBridgePlugin::default();
        let ctx = PluginContext::test();
        tick(&mut plugin, Some(&mock_tel(100.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("nav.approaching_junction").as_deref(),
            Some("true")
        );
    }

    #[test]
    fn approaching_junction_hysteresis() {
        let mut plugin = Ets2NavBridgePlugin::default();
        let ctx = PluginContext::test();

        // Cross the on-threshold (< 150 m)
        tick(&mut plugin, Some(&mock_tel(100.0, 10.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("nav.approaching_junction").as_deref(),
            Some("true")
        );

        // Between thresholds — stays on
        tick(&mut plugin, Some(&mock_tel(175.0, 20.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("nav.approaching_junction").as_deref(),
            Some("true")
        );

        // Cross the off-threshold (> 200 m)
        tick(&mut plugin, Some(&mock_tel(210.0, 25.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("nav.approaching_junction").as_deref(),
            Some("false")
        );

        // Between thresholds from off-side — stays off
        tick(&mut plugin, Some(&mock_tel(180.0, 22.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("nav.approaching_junction").as_deref(),
            Some("false")
        );

        // Re-enters below lower threshold
        tick(&mut plugin, Some(&mock_tel(130.0, 14.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("nav.approaching_junction").as_deref(),
            Some("true")
        );
    }

    #[test]
    fn waypoint_passed_on_distance_jump() {
        let mut plugin = Ets2NavBridgePlugin::default();
        let ctx = PluginContext::test();
        plugin.on_load(&ctx);

        tick(&mut plugin, Some(&mock_tel(200.0, 25.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("nav.waypoint_passed").as_deref(),
            Some("false")
        );

        // Jump of > 100 m
        tick(&mut plugin, Some(&mock_tel(350.0, 45.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("nav.waypoint_passed").as_deref(),
            Some("true")
        );
        assert!(ctx.blackboard.get("nav.last_waypoint_passed_ts").is_some());
    }

    #[test]
    fn waypoint_passed_clears_on_next_tick() {
        let mut plugin = Ets2NavBridgePlugin::default();
        let ctx = PluginContext::test();

        tick(&mut plugin, Some(&mock_tel(200.0, 25.0)), &ctx);
        tick(&mut plugin, Some(&mock_tel(350.0, 45.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("nav.waypoint_passed").as_deref(),
            Some("true")
        );

        tick(&mut plugin, Some(&mock_tel(340.0, 42.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("nav.waypoint_passed").as_deref(),
            Some("false")
        );
    }

    #[test]
    fn small_jump_does_not_trigger_waypoint_passed() {
        let mut plugin = Ets2NavBridgePlugin::default();
        let ctx = PluginContext::test();
        plugin.on_load(&ctx);

        tick(&mut plugin, Some(&mock_tel(200.0, 25.0)), &ctx);
        // 99 m jump — below threshold
        tick(&mut plugin, Some(&mock_tel(299.0, 35.0)), &ctx);
        assert_eq!(
            ctx.blackboard.get("nav.waypoint_passed").as_deref(),
            Some("false")
        );
    }
}
