//! Build read-only [`PlannedPathData`] for overlay / CLI from existing SHM status.
//!
//! v1 geometry comes from the embedded offline map-graph fixture
//! (`source = offline_graph`) and mirrors preflight safety from
//! [`StatusReport`] — no resolver activation, no ETS2 memory reads.

use truckpilot_plugin_api::planned_path::{
    build_offline_fixture_v1, with_safety, PlannedPathData, PlannedPathSafety, PlannedPathSource,
};

use crate::lane_debug::LaneDebugSnapshot;
use crate::preflight::{evaluate_preflight, PreflightDisplay};
use crate::status_report::StatusReport;

/// Result of a read-only planned-path build for overlay snapshots.
#[derive(Debug, Clone, PartialEq)]
pub struct PlannedPathOverlayBuild {
    /// Built path when geometry is valid; absent when skipped.
    pub data: Option<PlannedPathData>,
    /// `attached` when geometry is present; `skipped` when absent.
    pub status: &'static str,
    /// Producer label when attached (`offline_graph`).
    pub source: Option<&'static str>,
    /// Human-readable skip reason when `data` is absent.
    pub skip_reason: Option<&'static str>,
}

fn planned_path_has_geometry(data: &PlannedPathData) -> bool {
    !data.items.is_empty() && data.items.iter().any(|item| !item.points.is_empty())
}

/// Map preflight display fields into [`PlannedPathSafety`].
pub fn safety_from_preflight(p: &PreflightDisplay) -> PlannedPathSafety {
    PlannedPathSafety {
        route_valid: p.route_valid == Some(true),
        lane_model_valid: p.lane_model_valid == Some(true),
        resolver_safe: p.resolver_safe == Some(true),
        telemetry_fresh: p.telemetry_fresh == Some(true),
        input_allowed: p.input_allowed == Some(true),
        drive_allowed_display_only: p.drive_allowed_display,
        reasons: p.reasons.clone(),
    }
}

/// Try to build overlay planned path: offline-graph fixture geometry + live safety mirror.
///
/// Never panics — returns `None` when embedded fixture geometry is empty.
pub fn try_build_planned_path_overlay(
    status: &StatusReport,
    lane: &LaneDebugSnapshot,
) -> PlannedPathOverlayBuild {
    let preflight = evaluate_preflight(status);
    let mut safety = safety_from_preflight(&preflight);
    if preflight.lane_model_valid.is_none() {
        safety.lane_model_valid = lane.lane_model_valid;
        if !lane.lane_model_valid && !safety.reasons.iter().any(|r| r.contains("lane model")) {
            safety.reasons.push("lane model invalid".into());
            safety.drive_allowed_display_only = false;
        }
    }
    let data = with_safety(build_offline_fixture_v1(), safety);
    if !planned_path_has_geometry(&data) {
        return PlannedPathOverlayBuild {
            data: None,
            status: "skipped",
            source: None,
            skip_reason: Some("offline fixture produced empty geometry"),
        };
    }
    if data.source != PlannedPathSource::OfflineGraph {
        return PlannedPathOverlayBuild {
            data: None,
            status: "skipped",
            source: None,
            skip_reason: Some("planned path source is not offline_graph"),
        };
    }
    PlannedPathOverlayBuild {
        data: Some(data),
        status: "attached",
        source: Some("offline_graph"),
        skip_reason: None,
    }
}

/// Build overlay planned path: offline-graph fixture geometry + live safety mirror.
pub fn build_planned_path_overlay(
    status: &StatusReport,
    lane: &LaneDebugSnapshot,
) -> PlannedPathData {
    try_build_planned_path_overlay(status, lane)
        .data
        .expect("offline_graph fixture must always produce geometry")
}

#[cfg(test)]
#[cfg(windows)]
mod tests {
    use super::*;
    use crate::dll_perf::{DllPerfSnapshot, DLL_PERF_MAGIC, DLL_PERF_VERSION};
    use crate::lane_debug::build_lane_debug;
    use crate::nav_route::{RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE, ROUTE_BB_STATUS_DLL_ACTIVE, RouteSnapshot};
    use crate::overlay_snapshot::{format_overlay_json, OverlaySnapshot, PlannedPathProducerStatus};
    use crate::status_report::{evaluate_status, RawStatusInputs};
    use crate::lane_debug::lane_keeper_allowed;
    use truckpilot_plugin_api::planned_path::{
        planned_path_to_json, PlannedPathItemKind, PlannedPathSource,
    };

    fn safe_off_status() -> crate::status_report::StatusReport {
        evaluate_status(&RawStatusInputs {
            perf: Some(DllPerfSnapshot {
                magic: DLL_PERF_MAGIC,
                version: DLL_PERF_VERSION,
                diag_level_code: 9,
                frame_cb_count: 100,
                ..Default::default()
            }),
            route: Some(RouteSnapshot {
                bb_status: ROUTE_BB_STATUS_DLL_ACTIVE,
                resolve_status: RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE,
                valid: false,
                ..Default::default()
            }),
            telemetry_shm_present: true,
        })
    }

    #[test]
    fn overlay_planned_path_uses_offline_graph_source() {
        let status = safe_off_status();
        let lane = build_lane_debug(None, &status);
        let ppd = build_planned_path_overlay(&status, &lane);
        assert!(ppd.valid);
        assert_eq!(ppd.source, PlannedPathSource::OfflineGraph);
        assert_ne!(ppd.source, PlannedPathSource::Mock);
        assert!(
            ppd.items
                .iter()
                .any(|i| i.kind == PlannedPathItemKind::LaneChange)
        );
        assert!(
            ppd.items
                .iter()
                .any(|i| i.kind == PlannedPathItemKind::Junction)
        );
    }

    #[test]
    fn overlay_json_includes_planned_path() {
        let status = safe_off_status();
        let lane = build_lane_debug(None, &status);
        let snap = OverlaySnapshot {
            status: status.clone(),
            lane: lane.clone(),
            planned_path: Some(build_planned_path_overlay(&status, &lane)),
            planned_path_producer: PlannedPathProducerStatus {
                status: "attached".into(),
                source: Some("offline_graph".into()),
                reason: None,
            },
            lane_keeper_allowed: lane_keeper_allowed(&status, &lane),
            verdict: status.verdict,
        };
        let json = format_overlay_json(&snap);
        let parsed: serde_json::Value = serde_json::from_str(&json).expect("valid json");
        assert_eq!(parsed["planned_path"]["valid"], true);
        assert_eq!(parsed["planned_path"]["source"], "offline_graph");
        assert_eq!(
            parsed["planned_path"]["safety"]["drive_allowed_display_only"],
            false
        );
        assert!(parsed["planned_path"]["items"].as_array().unwrap().len() > 0);
        let point_count: usize = parsed["planned_path"]["items"]
            .as_array()
            .unwrap()
            .iter()
            .map(|item| item["points"].as_array().map(|a| a.len()).unwrap_or(0))
            .sum();
        assert!(point_count > 0);
        assert_eq!(parsed["planned_path_producer"]["status"], "attached");
        assert_eq!(parsed["status"]["resolver_attempts"], 0);
    }

    #[test]
    fn overlay_json_without_planned_path_still_valid() {
        let status = safe_off_status();
        let lane = build_lane_debug(None, &status);
        let snap = OverlaySnapshot {
            status,
            lane,
            planned_path: None,
            planned_path_producer: PlannedPathProducerStatus {
                status: "skipped".into(),
                source: None,
                reason: Some("test skip".into()),
            },
            lane_keeper_allowed: false,
            verdict: crate::status_report::StatusVerdict::SafeCold,
        };
        let json = format_overlay_json(&snap);
        let parsed: serde_json::Value = serde_json::from_str(&json).expect("valid json");
        assert!(parsed.get("planned_path").is_none());
    }

    #[test]
    fn fixture_json_export_roundtrip() {
        let status = safe_off_status();
        let lane = build_lane_debug(None, &status);
        let ppd = build_planned_path_overlay(&status, &lane);
        let json = planned_path_to_json(&ppd);
        let parsed: PlannedPathData = serde_json::from_str(&json).expect("roundtrip");
        assert_eq!(parsed.route_id, ppd.route_id);
        assert!(!parsed.safety.drive_allowed_display_only);
    }

    /// Run with `cargo test write_planned_path_fixture -- --ignored` to refresh
    /// `outputs/YYYY-MM-DD/planned_path_fixture.json`.
    #[test]
    #[ignore]
    fn write_planned_path_fixture() {
        let status = safe_off_status();
        let lane = build_lane_debug(None, &status);
        let ppd = build_planned_path_overlay(&status, &lane);
        let json = planned_path_to_json(&ppd);
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../outputs/2026-06-22/planned_path_fixture.json");
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).expect("create outputs dir");
        }
        std::fs::write(&path, json).expect("write fixture json");
    }
}
