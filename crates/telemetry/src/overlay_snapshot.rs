//! Combined read-only overlay JSON: DLL status + lane debug model.
//!
//! No steering, no daemon, no new SHM layouts — uses existing readers only.

use std::io;
use std::path::{Path, PathBuf};

use crate::lane_debug::{build_lane_debug, lane_keeper_allowed, LaneDebugSnapshot};
use crate::planned_path_overlay::try_build_planned_path_overlay;
use crate::status_report::{RawStatusInputs, StatusReport, StatusVerdict};
use truckpilot_plugin_api::planned_path::PlannedPathData;

/// Read-only status of the optional planned-path producer (display/debug only).
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
pub struct PlannedPathProducerStatus {
    /// `attached` or `skipped`.
    pub status: String,
    /// Producer label when attached (e.g. `offline_graph`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub source: Option<String>,
    /// Why geometry was omitted when `status = skipped`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

/// Full read-only overlay backend payload.
#[derive(Debug, Clone, serde::Serialize)]
pub struct OverlaySnapshot {
    /// DLL health and safety verdict derived from SHM.
    pub status: StatusReport,
    /// Lane geometry model built from the route blackboard or mock fixtures.
    pub lane: LaneDebugSnapshot,
    /// Read-only planned path (offline-graph fixture + live safety mirror in v1).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub planned_path: Option<PlannedPathData>,
    /// Whether planned_path was attached or skipped (debug for live feed consumers).
    pub planned_path_producer: PlannedPathProducerStatus,
    /// Lane keeper must remain disabled unless all safety gates pass.
    pub lane_keeper_allowed: bool,
    /// Duplicate of `status.verdict` for quick UI consumption.
    pub verdict: StatusVerdict,
}

/// Build overlay snapshot from raw SHM inputs (testable without live SHM).
pub fn build_overlay_snapshot(raw: &RawStatusInputs) -> OverlaySnapshot {
    let status = crate::status_report::evaluate_status(raw);
    let lane = build_lane_debug(raw.route.as_ref(), &status);
    let planned = try_build_planned_path_overlay(&status, &lane);
    let planned_path_producer = PlannedPathProducerStatus {
        status: planned.status.to_string(),
        source: planned.source.map(str::to_string),
        reason: planned.skip_reason.map(str::to_string),
    };
    let lane_keeper_allowed = lane_keeper_allowed(&status, &lane);
    let verdict = status.verdict;
    OverlaySnapshot {
        status,
        lane,
        planned_path: planned.data,
        planned_path_producer,
        lane_keeper_allowed,
        verdict,
    }
}

/// Read live SHM and build the overlay snapshot.
pub fn read_overlay_snapshot() -> OverlaySnapshot {
    build_overlay_snapshot(&crate::status_report::read_raw_inputs())
}

/// Default file path for `--overlay-loop` when `--overlay-out` is omitted.
pub fn default_overlay_loop_path() -> PathBuf {
    #[cfg(windows)]
    if let Ok(local) = std::env::var("LOCALAPPDATA") {
        return PathBuf::from(local)
            .join("TruckPilot")
            .join("overlay_snapshot.json");
    }
    #[cfg(unix)]
    if let Ok(home) = std::env::var("HOME") {
        return PathBuf::from(home)
            .join(".local/share/TruckPilot/overlay_snapshot.json");
    }
    PathBuf::from("overlay_snapshot.json")
}

/// Atomically write overlay JSON (tmp + rename) for live feed consumers.
pub fn write_overlay_snapshot_atomic(path: &Path, snap: &OverlaySnapshot) -> io::Result<()> {
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent)?;
        }
    }
    let json = format_overlay_json(snap);
    let tmp = path.with_extension("json.tmp");
    std::fs::write(&tmp, json)?;
    std::fs::rename(tmp, path)?;
    Ok(())
}

/// Pretty JSON for `--overlay`.
pub fn format_overlay_json(snap: &OverlaySnapshot) -> String {
    serde_json::to_string_pretty(snap).unwrap_or_else(|e| format!("{{\"json_error\":\"{e}\"}}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(windows)]
    use crate::dll_perf::{DllPerfSnapshot, DLL_PERF_MAGIC, DLL_PERF_VERSION};
    use crate::lane_debug::LaneDataSource;
    use crate::nav_route::{RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE, ROUTE_BB_STATUS_DLL_ACTIVE, RouteSnapshot};
    use crate::planned_path_overlay::build_planned_path_overlay;
    use crate::status_report::{evaluate_status, RawStatusInputs};
    use truckpilot_plugin_api::planned_path::PlannedPathSource;

    #[test]
    fn overlay_snapshot_includes_offline_planned_path_without_shm() {
        let snap = build_overlay_snapshot(&RawStatusInputs::default());
        let pp = snap.planned_path.as_ref().expect("planned_path");
        assert!(pp.valid);
        assert_eq!(pp.source, PlannedPathSource::OfflineGraph);
        assert!(!pp.items.is_empty());
        assert!(pp.items.iter().any(|i| !i.points.is_empty()));
        assert!(!pp.safety.drive_allowed_display_only);
        assert_eq!(snap.planned_path_producer.status, "attached");
        assert_eq!(
            snap.planned_path_producer.source.as_deref(),
            Some("offline_graph")
        );
        let json = format_overlay_json(&snap);
        let parsed: serde_json::Value = serde_json::from_str(&json).expect("valid json");
        assert_eq!(parsed["planned_path"]["source"], "offline_graph");
        assert_eq!(
            parsed["planned_path_producer"]["status"],
            "attached"
        );
    }

    #[test]
    fn overlay_loop_default_path_is_non_empty() {
        assert!(!default_overlay_loop_path().as_os_str().is_empty());
    }

    #[test]
    fn write_overlay_snapshot_atomic_roundtrip() {
        let snap = build_overlay_snapshot(&RawStatusInputs::default());
        let dir = std::env::temp_dir().join("truckpilot_overlay_test");
        let path = dir.join("overlay_snapshot.json");
        let _ = std::fs::remove_dir_all(&dir);
        write_overlay_snapshot_atomic(&path, &snap).expect("write");
        let raw = std::fs::read_to_string(&path).expect("read");
        let parsed: serde_json::Value = serde_json::from_str(&raw).expect("json");
        assert_eq!(parsed["planned_path"]["source"], "offline_graph");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[cfg(windows)]
    #[test]
    fn overlay_json_includes_status_and_lane() {
        let perf = DllPerfSnapshot {
            magic: DLL_PERF_MAGIC,
            version: DLL_PERF_VERSION,
            diag_level_code: 9,
            ..Default::default()
        };
        let route = RouteSnapshot {
            bb_status: ROUTE_BB_STATUS_DLL_ACTIVE,
            resolve_status: RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE,
            valid: false,
            ..Default::default()
        };
        let status = evaluate_status(&RawStatusInputs {
            perf: Some(perf),
            route: Some(route),
            telemetry_shm_present: true,
        });
        let lane = build_lane_debug(None, &status);
        let snap = OverlaySnapshot {
            lane_keeper_allowed: lane_keeper_allowed(&status, &lane),
            verdict: status.verdict,
            status: status.clone(),
            lane: lane.clone(),
            planned_path: Some(build_planned_path_overlay(&status, &lane)),
            planned_path_producer: PlannedPathProducerStatus {
                status: "attached".into(),
                source: Some("offline_graph".into()),
                reason: None,
            },
        };
        let json = format_overlay_json(&snap);
        let parsed: serde_json::Value = serde_json::from_str(&json).expect("valid json");
        assert_eq!(parsed["verdict"], "safe_cold");
        assert_eq!(parsed["lane"]["source"], "mock");
        assert_eq!(parsed["lane_keeper_allowed"], false);
        assert!(parsed["status"]["dll_active"].as_bool().unwrap());
        assert_eq!(parsed["status"]["core_readiness"]["available"], false);
        assert_eq!(parsed["status"]["preflight"]["drive_allowed_display"], false);
    }

    #[cfg(windows)]
    #[test]
    fn overlay_safe_off_lane_is_mock_not_error() {
        let perf = DllPerfSnapshot {
            magic: DLL_PERF_MAGIC,
            version: DLL_PERF_VERSION,
            diag_level_code: 0,
            ..Default::default()
        };
        let route = RouteSnapshot {
            bb_status: ROUTE_BB_STATUS_DLL_ACTIVE,
            resolve_status: RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE,
            valid: false,
            ..Default::default()
        };
        let status = evaluate_status(&RawStatusInputs {
            perf: Some(perf),
            route: Some(route),
            telemetry_shm_present: false,
        });
        let lane = build_lane_debug(None, &status);
        assert!(!lane.lane_model_valid);
        assert_eq!(lane.source, LaneDataSource::Mock);
    }
}
