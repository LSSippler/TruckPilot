//! Build read-only [`PlannedPathData`] for overlay / CLI from existing SHM status.
//!
//! Tries route-blackboard geometry when safe gates pass; otherwise falls back to
//! the embedded offline map-graph fixture. No resolver activation, no ETS2 memory
//! reads beyond existing SHM readers.

use truckpilot_plugin_api::planned_path::{
    build_offline_fixture_v1, planned_path_from_route_blackboard, with_safety,
    PlannedPathData, PlannedPathSafety, PlannedPathSource, RouteBlackboardWaypoint,
};

use crate::lane_debug::LaneDebugSnapshot;
use crate::nav_route::{RouteSnapshot, ROUTE_WP_FLAG_HAS_POSITION};
use crate::preflight::{evaluate_preflight, PreflightDisplay};
use crate::shm::ShmReader;
use crate::status_report::{StatusReport, StatusVerdict};

/// Producer status labels exposed in overlay JSON (`planned_path_producer.status`).
pub mod producer_status {
    /// Embedded offline-graph fixture attached.
    pub const OFFLINE_FIXTURE: &str = "offline_fixture";
    /// Live route gates failed or geometry build failed; offline fallback used.
    pub const LIVE_ROUTE_UNAVAILABLE: &str = "live_route_unavailable";
    /// Route UIDs present but insufficient positioned geometry.
    pub const LIVE_ROUTE_CANDIDATE: &str = "live_route_candidate";
    /// Live route blackboard polyline attached.
    pub const LIVE_ROUTE_ATTACHED: &str = "live_route_attached";
    /// No planned-path geometry produced.
    pub const SKIPPED: &str = "skipped";
}

/// Producer source labels (`planned_path_producer.source`).
pub mod producer_source {
    /// Embedded offline map-graph fixture.
    pub const OFFLINE_GRAPH: &str = "offline_graph";
    /// ETS2 route blackboard SHM waypoint polyline.
    pub const ROUTE_BLACKBOARD: &str = "route_blackboard";
}

/// Result of a read-only planned-path build for overlay snapshots.
#[derive(Debug, Clone, PartialEq)]
pub struct PlannedPathOverlayBuild {
    /// Built path when geometry is valid; absent when skipped.
    pub data: Option<PlannedPathData>,
    /// Producer mode (see [`producer_status`]).
    pub status: &'static str,
    /// Producer label (`offline_graph` or `route_blackboard`).
    pub source: Option<&'static str>,
    /// Human-readable skip / fallback reason when live route is not attached.
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

fn merge_lane_safety(safety: &mut PlannedPathSafety, lane: &LaneDebugSnapshot) {
    if safety.lane_model_valid {
        return;
    }
    safety.lane_model_valid = lane.lane_model_valid;
    if !lane.lane_model_valid && !safety.reasons.iter().any(|r| r.contains("lane model")) {
        safety.reasons.push("lane model invalid".into());
        safety.drive_allowed_display_only = false;
    }
}

fn truck_xz_from_shm() -> Option<(f64, f64)> {
    ShmReader::open()
        .ok()
        .and_then(|mut r| r.read())
        .map(|t| (t.position[0], t.position[2]))
}

fn positioned_route_waypoints(route: &RouteSnapshot) -> Vec<RouteBlackboardWaypoint> {
    route
        .waypoints
        .iter()
        .filter(|wp| wp.flags & ROUTE_WP_FLAG_HAS_POSITION != 0)
        .map(|wp| RouteBlackboardWaypoint {
            uid: wp.uid,
            x: wp.x as f64,
            y: wp.y as f64,
            z: wp.z as f64,
        })
        .collect()
}

fn live_route_gate_failure(
    route: Option<&RouteSnapshot>,
    status: &StatusReport,
) -> Option<&'static str> {
    if !status.route_bb_available {
        return Some("route blackboard SHM not available");
    }
    let route = route?;
    if status.resolver_off {
        return Some("resolver safe-off mode; live route not consumed");
    }
    if status.verdict != StatusVerdict::SafeCold {
        return Some("status verdict is not safe_cold");
    }
    if !status.route_valid || !route.valid {
        return Some("route not valid in SHM");
    }
    if route.waypoints.is_empty() {
        return Some("route has no waypoints");
    }
    None
}

fn try_build_live_route_path(
    route: &RouteSnapshot,
    safety: PlannedPathSafety,
) -> Option<PlannedPathData> {
    let positioned = positioned_route_waypoints(route);
    if positioned.len() < 2 {
        return None;
    }
    let mut data =
        planned_path_from_route_blackboard(&positioned, route.route_hash, truck_xz_from_shm())?;
    data.safety = safety;
    if data.source != PlannedPathSource::RouteBlackboard {
        return None;
    }
    if !planned_path_has_geometry(&data) {
        return None;
    }
    Some(data)
}

fn build_offline_fixture_path(safety: PlannedPathSafety) -> PlannedPathOverlayBuild {
    let data = with_safety(build_offline_fixture_v1(), safety);
    if !planned_path_has_geometry(&data) {
        return PlannedPathOverlayBuild {
            data: None,
            status: producer_status::SKIPPED,
            source: None,
            skip_reason: Some("offline fixture produced empty geometry"),
        };
    }
    if data.source != PlannedPathSource::OfflineGraph {
        return PlannedPathOverlayBuild {
            data: None,
            status: producer_status::SKIPPED,
            source: None,
            skip_reason: Some("planned path source is not offline_graph"),
        };
    }
    PlannedPathOverlayBuild {
        data: Some(data),
        status: producer_status::OFFLINE_FIXTURE,
        source: Some(producer_source::OFFLINE_GRAPH),
        skip_reason: None,
    }
}

/// Try to build overlay planned path: live route blackboard when safe, else offline fixture.
///
/// Never panics — returns `skipped` when no geometry can be produced.
pub fn try_build_planned_path_overlay(
    status: &StatusReport,
    lane: &LaneDebugSnapshot,
    route: Option<&RouteSnapshot>,
) -> PlannedPathOverlayBuild {
    let preflight = evaluate_preflight(status);
    let mut safety = safety_from_preflight(&preflight);
    merge_lane_safety(&mut safety, lane);

    if let Some(route) = route {
        if live_route_gate_failure(Some(route), status).is_none() {
            if let Some(data) = try_build_live_route_path(route, safety.clone()) {
                return PlannedPathOverlayBuild {
                    data: Some(data),
                    status: producer_status::LIVE_ROUTE_ATTACHED,
                    source: Some(producer_source::ROUTE_BLACKBOARD),
                    skip_reason: None,
                };
            }
            let positioned = positioned_route_waypoints(route).len();
            if positioned < 2 && route.waypoints.len() >= 2 {
                let mut build = build_offline_fixture_path(safety);
                build.status = producer_status::LIVE_ROUTE_CANDIDATE;
                build.skip_reason = Some(
                    "route has UIDs but fewer than 2 positioned waypoints for geometry",
                );
                return build;
            }
            let mut build = build_offline_fixture_path(safety);
            build.status = producer_status::LIVE_ROUTE_UNAVAILABLE;
            build.skip_reason = Some("live route gates passed but geometry build failed");
            return build;
        }
    }

    let gate_reason = live_route_gate_failure(route, status);
    let mut build = build_offline_fixture_path(safety);
    if gate_reason == Some("resolver safe-off mode; live route not consumed") {
        build.status = producer_status::OFFLINE_FIXTURE;
        build.skip_reason = None;
    } else if gate_reason == Some("route blackboard SHM not available") {
        build.status = producer_status::OFFLINE_FIXTURE;
        build.skip_reason = None;
    } else if let Some(reason) = gate_reason {
        build.status = producer_status::LIVE_ROUTE_UNAVAILABLE;
        build.skip_reason = Some(reason);
    }
    build
}

/// Build overlay planned path: offline-graph fixture geometry + live safety mirror.
pub fn build_planned_path_overlay(
    status: &StatusReport,
    lane: &LaneDebugSnapshot,
    route: Option<&RouteSnapshot>,
) -> PlannedPathData {
    try_build_planned_path_overlay(status, lane, route)
        .data
        .expect("offline_graph fixture must always produce geometry when route is absent")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dll_perf::{DllPerfSnapshot, DLL_PERF_MAGIC, DLL_PERF_VERSION};
    use crate::lane_debug::build_lane_debug;
    use crate::nav_route::{
        RouteSnapshot, RouteWaypoint, RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE,
        RESOLVE_WAYPOINTS_COLLECTED, ROUTE_BB_STATUS_DLL_ACTIVE, ROUTE_WP_FLAG_HAS_POSITION,
    };
    use crate::overlay_snapshot::{format_overlay_json, OverlaySnapshot, PlannedPathProducerStatus};
    use crate::status_report::{evaluate_status, RawStatusInputs};
    use crate::lane_debug::lane_keeper_allowed;
    use truckpilot_plugin_api::planned_path::{
        planned_path_to_json, PlannedPathItemKind, PlannedPathSource,
    };

    fn safe_off_inputs() -> RawStatusInputs {
        RawStatusInputs {
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
        }
    }

    fn live_route_snapshot() -> RouteSnapshot {
        RouteSnapshot {
            valid: true,
            bb_status: ROUTE_BB_STATUS_DLL_ACTIVE,
            resolve_status: RESOLVE_WAYPOINTS_COLLECTED,
            route_hash: 0xABCD,
            waypoints: vec![
                RouteWaypoint {
                    uid: 1001,
                    x: 0.0,
                    z: 0.0,
                    flags: ROUTE_WP_FLAG_HAS_POSITION,
                    ..Default::default()
                },
                RouteWaypoint {
                    uid: 1002,
                    x: 0.0,
                    z: 40.0,
                    flags: ROUTE_WP_FLAG_HAS_POSITION,
                    ..Default::default()
                },
                RouteWaypoint {
                    uid: 1003,
                    x: 10.0,
                    z: 80.0,
                    flags: ROUTE_WP_FLAG_HAS_POSITION,
                    ..Default::default()
                },
            ],
            ..Default::default()
        }
    }

    fn live_status_with_route(route: RouteSnapshot) -> crate::status_report::StatusReport {
        evaluate_status(&RawStatusInputs {
            perf: Some(DllPerfSnapshot {
                magic: DLL_PERF_MAGIC,
                version: DLL_PERF_VERSION,
                diag_level_code: 9,
                frame_cb_count: 100,
                ..Default::default()
            }),
            route: Some(route),
            telemetry_shm_present: true,
        })
    }

    #[test]
    fn resolver_off_uses_offline_fixture_producer() {
        let inputs = safe_off_inputs();
        let status = evaluate_status(&inputs);
        let lane = build_lane_debug(inputs.route.as_ref(), &status);
        let build = try_build_planned_path_overlay(&status, &lane, inputs.route.as_ref());
        assert_eq!(build.status, producer_status::OFFLINE_FIXTURE);
        assert_eq!(build.source, Some(producer_source::OFFLINE_GRAPH));
        assert_eq!(
            build.data.as_ref().unwrap().source,
            PlannedPathSource::OfflineGraph
        );
        assert!(build.skip_reason.is_none());
    }

    #[test]
    fn live_route_attached_when_route_blackboard_valid() {
        let route = live_route_snapshot();
        let status = live_status_with_route(route.clone());
        let lane = build_lane_debug(Some(&route), &status);
        let build = try_build_planned_path_overlay(&status, &lane, Some(&route));
        assert_eq!(build.status, producer_status::LIVE_ROUTE_ATTACHED);
        assert_eq!(build.source, Some(producer_source::ROUTE_BLACKBOARD));
        let data = build.data.expect("live path");
        assert_eq!(data.source, PlannedPathSource::RouteBlackboard);
        assert_eq!(data.items.len(), 2);
        assert!(data.items.iter().all(|i| i.kind == PlannedPathItemKind::RoadEdge));
        assert!(!data.safety.drive_allowed_display_only);
    }

    #[test]
    fn live_route_candidate_when_uids_without_positions() {
        let route = RouteSnapshot {
            valid: true,
            bb_status: ROUTE_BB_STATUS_DLL_ACTIVE,
            resolve_status: RESOLVE_WAYPOINTS_COLLECTED,
            waypoints: vec![
                RouteWaypoint {
                    uid: 1,
                    flags: 0,
                    ..Default::default()
                },
                RouteWaypoint {
                    uid: 2,
                    flags: 0,
                    ..Default::default()
                },
            ],
            ..Default::default()
        };
        let status = live_status_with_route(route.clone());
        let lane = build_lane_debug(Some(&route), &status);
        let build = try_build_planned_path_overlay(&status, &lane, Some(&route));
        assert_eq!(build.status, producer_status::LIVE_ROUTE_CANDIDATE);
        assert_eq!(build.source, Some(producer_source::OFFLINE_GRAPH));
        assert_eq!(
            build.data.as_ref().unwrap().source,
            PlannedPathSource::OfflineGraph
        );
        assert!(build.skip_reason.is_some());
    }

    #[test]
    fn overlay_planned_path_uses_offline_graph_source_when_resolver_off() {
        let inputs = safe_off_inputs();
        let status = evaluate_status(&inputs);
        let lane = build_lane_debug(inputs.route.as_ref(), &status);
        let ppd = build_planned_path_overlay(&status, &lane, inputs.route.as_ref());
        assert!(ppd.valid);
        assert_eq!(ppd.source, PlannedPathSource::OfflineGraph);
        assert_ne!(ppd.source, PlannedPathSource::Mock);
        assert!(
            ppd.items
                .iter()
                .any(|i| i.kind == PlannedPathItemKind::LaneChange)
        );
    }

    #[test]
    fn overlay_json_includes_planned_path_producer_metadata() {
        let inputs = safe_off_inputs();
        let status = evaluate_status(&inputs);
        let lane = build_lane_debug(inputs.route.as_ref(), &status);
        let planned = try_build_planned_path_overlay(&status, &lane, inputs.route.as_ref());
        let snap = OverlaySnapshot {
            status: status.clone(),
            lane: lane.clone(),
            planned_path: planned.data,
            planned_path_producer: PlannedPathProducerStatus {
                status: planned.status.to_string(),
                source: planned.source.map(str::to_string),
                reason: planned.skip_reason.map(str::to_string),
            },
            lane_keeper_allowed: lane_keeper_allowed(&status, &lane),
            verdict: status.verdict,
        };
        let json = format_overlay_json(&snap);
        let parsed: serde_json::Value = serde_json::from_str(&json).expect("valid json");
        assert_eq!(parsed["planned_path"]["source"], "offline_graph");
        assert_eq!(
            parsed["planned_path_producer"]["status"],
            producer_status::OFFLINE_FIXTURE
        );
        assert_eq!(
            parsed["planned_path_producer"]["source"],
            producer_source::OFFLINE_GRAPH
        );
        assert_eq!(parsed["status"]["resolver_attempts"], 0);
    }

    #[test]
    fn overlay_json_without_planned_path_still_valid() {
        let inputs = safe_off_inputs();
        let status = evaluate_status(&inputs);
        let lane = build_lane_debug(inputs.route.as_ref(), &status);
        let snap = OverlaySnapshot {
            status,
            lane,
            planned_path: None,
            planned_path_producer: PlannedPathProducerStatus {
                status: producer_status::SKIPPED.into(),
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
        let inputs = safe_off_inputs();
        let status = evaluate_status(&inputs);
        let lane = build_lane_debug(inputs.route.as_ref(), &status);
        let ppd = build_planned_path_overlay(&status, &lane, inputs.route.as_ref());
        let json = planned_path_to_json(&ppd);
        let parsed: PlannedPathData = serde_json::from_str(&json).expect("roundtrip");
        assert_eq!(parsed.route_id, ppd.route_id);
        assert!(!parsed.safety.drive_allowed_display_only);
    }
}
