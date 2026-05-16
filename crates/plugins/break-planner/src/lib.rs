//! Break-Planner plugin — driver fatigue monitoring and EU driving-time rules.
//!
//! ## EU Regulation (EC) No 561/2006 (simplified)
//!
//! - Max continuous driving: **4.5 hours** → mandatory 45-minute break.
//! - Break can be split: 15 min + 30 min (in that order).
//! - After a break the 4.5-hour counter resets.
//!
//! The plugin tracks driving time using wall-clock elapsed time while the
//! truck is moving (speed > 1 m/s). It does NOT track real-world time —
//! only in-game driving time.
//!
//! ## Blackboard contract
//!
//! | Key                          | Written by    | Read by |
//! |------------------------------|---------------|---------|
//! | `break.needed`               | break-planner | UI      |
//! | `break.driving_time_s`       | break-planner | UI      |
//! | `break.remaining_s`          | break-planner | UI      |
//! | `break.in_break`             | break-planner | UI      |

use std::time::Instant;

use truckpilot_plugin_api::{ControlOutput, Plugin, PluginContext, Telemetry, TickPhase};

/// EU max continuous driving time before a mandatory break.
const EU_MAX_DRIVE_SECS: f64 = 4.5 * 3600.0; // 4h 30min
/// EU mandatory break duration.
const EU_BREAK_SECS: f64 = 45.0 * 60.0; // 45 min
/// Speed threshold below which the truck is considered stopped.
const MOVING_THRESHOLD_MS: f64 = 1.0;
/// Warn when this many seconds remain before mandatory break.
const WARN_BEFORE_SECS: f64 = 15.0 * 60.0; // 15 min warning

pub struct BreakPlannerPlugin {
    /// Accumulated driving time in seconds since last break.
    driving_time_s: f64,
    /// Accumulated break time in seconds during current break.
    break_time_s: f64,
    /// Whether the driver is currently on a break.
    in_break: bool,
    /// Whether EU rules are enforced.
    eu_rules_enabled: bool,
    /// Fatigue threshold from game telemetry (0..1).
    fatigue_threshold: f32,
    last_tick: Option<Instant>,
}

impl Default for BreakPlannerPlugin {
    fn default() -> Self {
        Self {
            driving_time_s: 0.0,
            break_time_s: 0.0,
            in_break: false,
            eu_rules_enabled: true,
            fatigue_threshold: 0.85,
            last_tick: None,
        }
    }
}

impl BreakPlannerPlugin {
    fn remaining_drive_secs(&self) -> f64 {
        (EU_MAX_DRIVE_SECS - self.driving_time_s).max(0.0)
    }

    fn break_needed(&self, fatigue: f32) -> bool {
        if self.in_break {
            return false;
        }
        if fatigue >= self.fatigue_threshold {
            return true;
        }
        if self.eu_rules_enabled && self.driving_time_s >= EU_MAX_DRIVE_SECS {
            return true;
        }
        false
    }
}

impl Plugin for BreakPlannerPlugin {
    fn name(&self) -> &str {
        "break-planner"
    }
    fn version(&self) -> &str {
        "0.1.0"
    }
    fn settings_schema(&self) -> &str {
        r#"{
  "type": "object",
  "properties": {
    "eu_rules_enabled": {
      "type": "boolean",
      "description": "Enforce EU 561/2006 driving-time rules."
    },
    "fatigue_threshold": {
      "type": "number", "minimum": 0.5, "maximum": 1.0,
      "description": "Game fatigue level (0-1) at which a break is forced."
    }
  }
}"#
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        if let Some(v) = ctx.blackboard.get("break_planner.eu_rules_enabled") {
            self.eu_rules_enabled = v == "true";
        }
        if let Some(v) = ctx.blackboard.get_f64("break_planner.fatigue_threshold") {
            self.fatigue_threshold = v.clamp(0.5, 1.0) as f32;
        }
        tracing::info!(
            "[break-planner] loaded — eu_rules={} fatigue_threshold={:.2}",
            self.eu_rules_enabled,
            self.fatigue_threshold
        );
    }

    fn on_unload(&mut self) {
        tracing::info!("[break-planner] unloaded");
    }

    fn default_phase(&self) -> TickPhase {
        TickPhase::PhaseA
    }

    fn tick(
        &mut self,
        telemetry: Option<&Telemetry>,
        _output: &mut ControlOutput,
        ctx: &PluginContext,
    ) {
        let now = Instant::now();
        let dt = self
            .last_tick
            .map(|t| now.duration_since(t).as_secs_f64())
            .unwrap_or(0.02);
        self.last_tick = Some(now);

        let Some(t) = telemetry else {
            ctx.blackboard.set("break.needed", "false");
            return;
        };

        let moving = t.speed_ms > MOVING_THRESHOLD_MS;
        // Fatigue from blackboard (SHM plugin writes it) or default 0.
        let fatigue = ctx.blackboard.get_f64("telemetry.fatigue").unwrap_or(0.0) as f32;

        if self.in_break {
            // Accumulate break time while stopped.
            if !moving {
                self.break_time_s += dt;
            }
            // Break complete when 45 min accumulated.
            if self.break_time_s >= EU_BREAK_SECS {
                tracing::info!("[break-planner] break complete — resetting driving timer");
                self.driving_time_s = 0.0;
                self.break_time_s = 0.0;
                self.in_break = false;
            }
        } else {
            // Accumulate driving time while moving.
            if moving {
                self.driving_time_s += dt;
            }

            if self.break_needed(fatigue) {
                tracing::warn!(
                    "[break-planner] break needed — driven {:.0}min, fatigue={:.2}",
                    self.driving_time_s / 60.0,
                    fatigue
                );
                self.in_break = true;
                self.break_time_s = 0.0;
            } else if self.eu_rules_enabled {
                let remaining = self.remaining_drive_secs();
                if remaining <= WARN_BEFORE_SECS {
                    tracing::warn!(
                        "[break-planner] {:.0} min until mandatory break",
                        remaining / 60.0
                    );
                }
            }
        }

        ctx.blackboard
            .set("break.needed", if self.in_break { "true" } else { "false" });
        ctx.blackboard
            .set("break.driving_time_s", self.driving_time_s.to_string());
        ctx.blackboard
            .set("break.remaining_s", self.remaining_drive_secs().to_string());
        ctx.blackboard.set(
            "break.in_break",
            if self.in_break { "true" } else { "false" },
        );
    }
}

truckpilot_plugin_api::export_plugin!(BreakPlannerPlugin);

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_break_needed_when_fresh() {
        let p = BreakPlannerPlugin::default();
        assert!(!p.break_needed(0.0));
    }

    #[test]
    fn break_needed_after_eu_limit() {
        let p = BreakPlannerPlugin {
            driving_time_s: EU_MAX_DRIVE_SECS + 1.0,
            ..Default::default()
        };
        assert!(p.break_needed(0.0));
    }

    #[test]
    fn break_needed_on_high_fatigue() {
        let p = BreakPlannerPlugin {
            driving_time_s: 0.0,
            ..Default::default()
        };
        assert!(p.break_needed(0.9)); // above 0.85 threshold
    }

    #[test]
    fn no_break_needed_when_already_in_break() {
        let p = BreakPlannerPlugin {
            in_break: true,
            driving_time_s: EU_MAX_DRIVE_SECS + 1.0,
            ..Default::default()
        };
        assert!(!p.break_needed(0.9));
    }

    #[test]
    fn remaining_drive_time_decreases() {
        let p = BreakPlannerPlugin {
            driving_time_s: 3600.0, // 1 hour driven
            ..Default::default()
        };
        let remaining = p.remaining_drive_secs();
        assert!((remaining - (EU_MAX_DRIVE_SECS - 3600.0)).abs() < 0.01);
    }

    #[test]
    fn remaining_never_negative() {
        let p = BreakPlannerPlugin {
            driving_time_s: EU_MAX_DRIVE_SECS * 2.0,
            ..Default::default()
        };
        assert_eq!(p.remaining_drive_secs(), 0.0);
    }

    #[test]
    fn eu_rules_disabled_no_time_break() {
        let p = BreakPlannerPlugin {
            eu_rules_enabled: false,
            driving_time_s: EU_MAX_DRIVE_SECS + 1.0,
            ..Default::default()
        };
        assert!(!p.break_needed(0.0)); // no fatigue, rules disabled
    }
}
