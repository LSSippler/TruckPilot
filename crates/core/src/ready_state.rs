//! Daemon readiness contract — coarse blackboard flags for startup gating.
//!
//! These keys exist so future lazy-load paths (SplineIndex background build,
//! deferred plugin warmups) can signal incomplete state without any plugin
//! steering on half-loaded graph/spline/plugin data.
//!
//! **Semantics (important):**
//! - `lane_detection_ready` — ONNX/plugin *load* readiness (`lane.diag.load_ok`),
//!   **not** current lane validity, centerline, or offset quality.
//! - `truckpilot_system_ready` — daemon subsystem startup complete (AND of the
//!   component flags below). **Not** an Engage/autopilot clearance.
//!
//! ## `truckpilot_system_ready` is NOT a drive / engage authorization
//!
//! `truckpilot_system_ready` means **only** that the subsystems finished
//! loading/initialising (graph, spline index, plugins, lane-detection model).
//! It must **never** be read as permission to steer or to engage the autopilot.
//!
//! Engaging requires *additional runtime checks* that this contract
//! deliberately does not cover, evaluated later in the state machine / engage
//! path, e.g.:
//!
//! - **telemetry fresh** — a recent, sane telemetry frame is arriving
//! - **route_valid** — a planned route exists for the current position
//! - **lane_model_valid** — a *current* lane detection (live confidence /
//!   fresh frame), not merely a loaded model
//! - **resolver safe** — the spline/route resolver is in a safe state
//! - **input allowed** — the output sink / input path is permitted to act
//!
//! In short: `system_ready` is "subsystems up"; engage is "safe to act now".
//!
//! **v1 scope:** Core publishes flags during daemon startup and refreshes
//! `lane_detection_ready` / `truckpilot_system_ready` each tick. No engage or
//! steering behaviour changes yet.

use truckpilot_plugin_api::SharedBlackboard;

pub const GRAPH_READY: &str = "graph_ready";
pub const SPLINE_INDEX_READY: &str = "spline_index_ready";
pub const PLUGINS_READY: &str = "plugins_ready";
/// lane-detection plugin loaded and `lane.diag.load_ok == true` (or plugin absent).
pub const LANE_DETECTION_READY: &str = "lane_detection_ready";
/// All component readiness flags true; system startup gate only — not Engage.
pub const SYSTEM_READY: &str = "truckpilot_system_ready";

const READY_KEYS: &[&str] = &[
    GRAPH_READY,
    SPLINE_INDEX_READY,
    PLUGINS_READY,
    LANE_DETECTION_READY,
    SYSTEM_READY,
];

/// Seed all contract keys to `"false"` before any subsystem completes.
pub fn seed_all_false(bb: &SharedBlackboard) {
    for key in READY_KEYS {
        bb.set(*key, "false");
    }
}

pub fn set_graph_ready(bb: &SharedBlackboard, ready: bool) {
    bb.set(GRAPH_READY, ready.to_string());
    refresh_system_ready(bb);
}

pub fn set_spline_index_ready(bb: &SharedBlackboard, ready: bool) {
    bb.set(SPLINE_INDEX_READY, ready.to_string());
    refresh_system_ready(bb);
}

pub fn set_plugins_ready(bb: &SharedBlackboard, ready: bool) {
    bb.set(PLUGINS_READY, ready.to_string());
    refresh_system_ready(bb);
}

/// Re-read plugin-owned diag keys and recompute `lane_detection_ready` +
/// `truckpilot_system_ready`. Call after `load_all` and once per daemon tick.
pub fn refresh_dynamic(bb: &SharedBlackboard) {
    let lane_detection = compute_lane_detection_ready(bb);
    bb.set(LANE_DETECTION_READY, lane_detection.to_string());
    refresh_system_ready(bb);
}

fn refresh_system_ready(bb: &SharedBlackboard) {
    let ready = is_true(bb, GRAPH_READY)
        && is_true(bb, SPLINE_INDEX_READY)
        && is_true(bb, PLUGINS_READY)
        && is_true(bb, LANE_DETECTION_READY);
    bb.set(SYSTEM_READY, ready.to_string());
}

fn is_true(bb: &SharedBlackboard, key: &str) -> bool {
    bb.get(key).as_deref() == Some("true")
}

/// lane-detection *load* readiness from `lane.diag.load_ok` when the plugin is loaded.
fn compute_lane_detection_ready(bb: &SharedBlackboard) -> bool {
    let loaded = bb.get("plugins.loaded").unwrap_or_default();
    let names: Vec<&str> = loaded.split(',').filter(|s| !s.is_empty()).collect();

    if names.iter().any(|&n| n == "lane-detection") {
        if bb.get("lane.diag.load_ok").as_deref() != Some("true") {
            return false;
        }
    }

    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use truckpilot_plugin_api::SharedBlackboard;

    fn bb_with_plugins(list: &str) -> SharedBlackboard {
        let bb = SharedBlackboard::new();
        bb.set("plugins.loaded", list.to_string());
        bb
    }

    #[test]
    fn seed_all_false_sets_contract_keys() {
        let bb = SharedBlackboard::new();
        seed_all_false(&bb);
        for key in READY_KEYS {
            assert_eq!(bb.get(key).as_deref(), Some("false"));
        }
    }

    #[test]
    fn system_ready_requires_all_components() {
        let bb = SharedBlackboard::new();
        seed_all_false(&bb);
        set_graph_ready(&bb, true);
        assert_eq!(bb.get(SYSTEM_READY).as_deref(), Some("false"));
        set_spline_index_ready(&bb, true);
        set_plugins_ready(&bb, true);
        bb.set(LANE_DETECTION_READY, "true");
        refresh_system_ready(&bb);
        assert_eq!(bb.get(SYSTEM_READY).as_deref(), Some("true"));
    }

    #[test]
    fn system_ready_false_while_any_single_flag_false() {
        // With every other component flag true, flipping exactly one to false
        // must drop the aggregate to false — the AND has no shortcuts.
        let components = [
            GRAPH_READY,
            SPLINE_INDEX_READY,
            PLUGINS_READY,
            LANE_DETECTION_READY,
        ];
        for missing in components {
            let bb = SharedBlackboard::new();
            for key in components {
                bb.set(key, if key == missing { "false" } else { "true" });
            }
            refresh_system_ready(&bb);
            assert_eq!(
                bb.get(SYSTEM_READY).as_deref(),
                Some("false"),
                "system_ready must be false when {missing} is false"
            );
        }
    }

    #[test]
    fn lane_detection_requires_load_ok_when_plugin_loaded() {
        let bb = bb_with_plugins("lane-detection,lane-keeper");
        bb.set("lane.diag.load_ok", "false");
        assert!(!compute_lane_detection_ready(&bb));
        bb.set("lane.diag.load_ok", "true");
        assert!(compute_lane_detection_ready(&bb));
    }

    #[test]
    fn lane_detection_false_when_load_ok_key_missing() {
        // lane-detection loaded but the diag key was never published yet →
        // detector is not ready (missing is treated like not-ok).
        let bb = bb_with_plugins("lane-detection,router");
        assert!(bb.get("lane.diag.load_ok").is_none());
        assert!(!compute_lane_detection_ready(&bb));
    }

    #[test]
    fn lane_detection_ok_without_lane_detection_plugin() {
        let bb = bb_with_plugins("lane-keeper,router");
        assert!(compute_lane_detection_ready(&bb));
    }

    #[test]
    fn refresh_dynamic_updates_lane_detection_and_system_ready() {
        let bb = bb_with_plugins("lane-detection");
        seed_all_false(&bb);
        set_graph_ready(&bb, true);
        set_spline_index_ready(&bb, true);
        set_plugins_ready(&bb, true);
        bb.set("lane.diag.load_ok", "true");
        refresh_dynamic(&bb);
        assert_eq!(bb.get(LANE_DETECTION_READY).as_deref(), Some("true"));
        assert_eq!(bb.get(SYSTEM_READY).as_deref(), Some("true"));
    }
}
