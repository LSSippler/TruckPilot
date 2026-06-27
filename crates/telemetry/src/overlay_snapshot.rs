//! Combined read-only overlay JSON: DLL status + lane debug model.
//!
//! No steering, no daemon, no new SHM layouts — uses existing readers only.

use crate::lane_debug::{build_lane_debug, lane_keeper_allowed, LaneDebugSnapshot};
use crate::status_report::{StatusReport, StatusVerdict};

/// Full read-only overlay backend payload.
#[derive(Debug, Clone, serde::Serialize)]
pub struct OverlaySnapshot {
    /// DLL health and safety verdict derived from SHM.
    pub status: StatusReport,
    /// Lane geometry model built from the route blackboard or mock fixtures.
    pub lane: LaneDebugSnapshot,
    /// Lane keeper must remain disabled unless all safety gates pass.
    pub lane_keeper_allowed: bool,
    /// Duplicate of `status.verdict` for quick UI consumption.
    pub verdict: StatusVerdict,
}

/// Read live SHM and build the overlay snapshot.
pub fn read_overlay_snapshot() -> OverlaySnapshot {
    let raw = crate::status_report::read_raw_inputs();
    let status = crate::status_report::evaluate_status(&raw);
    let lane = build_lane_debug(raw.route.as_ref(), &status);
    let lane_keeper_allowed = lane_keeper_allowed(&status, &lane);
    let verdict = status.verdict;
    OverlaySnapshot {
        status,
        lane,
        lane_keeper_allowed,
        verdict,
    }
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
    use crate::status_report::{evaluate_status, RawStatusInputs};

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
            status,
            lane,
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
