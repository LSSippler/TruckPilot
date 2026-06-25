//! ETS2 route SHM → shared cache → graph match → blackboard diagnostics (Phase 5a+5i).

use std::sync::{Arc, RwLock};

use tracing::info;
use truckpilot_plugin_api::ets2_route::{
    match_ets2_route_uids, match_invalid_ets2_route, match_unavailable_ets2_route,
    Ets2RouteMatchResult, Ets2RouteSharedState, Ets2RouteSnapshot, Ets2RouteWaypoint,
};
use truckpilot_plugin_api::graph::RouterGraph;
use truckpilot_plugin_api::SharedBlackboard;
use truckpilot_telemetry::nav_route::{
    diagnose_route_coords, diagnose_route_distances, format_position_triple, RouteBlackboardReader,
    RouteSnapshot,
};

const ETS2_ROUTE_RAW_UIDS_LIMIT: usize = 64;

const ETS2_ROUTE_SNAPSHOT_KEYS: &[&str] = &[
    "navigation.ets2_route.valid",
    "navigation.ets2_route.sequence",
    "navigation.ets2_route.hash",
    "navigation.ets2_route.waypoint_count",
    "navigation.ets2_route.source",
    "navigation.ets2_route.raw_uids",
];

const ETS2_ROUTE_COORD_KEYS: &[&str] = &[
    "navigation.ets2_route.position_count",
    "navigation.ets2_route.position_ratio",
    "navigation.ets2_route.distance_count",
    "navigation.ets2_route.time_count",
    "navigation.ets2_route.first_position",
    "navigation.ets2_route.last_position",
    "navigation.ets2_route.coord_source",
    "navigation.ets2_route.coord_status",
    "navigation.ets2_route.distance_untrusted_count",
    "navigation.ets2_route.distance_first_m",
    "navigation.ets2_route.distance_last_m",
    "navigation.ets2_route.distance_min_m",
    "navigation.ets2_route.distance_max_m",
    "navigation.ets2_route.distance_monotonic_status",
    "navigation.ets2_route.distance_increase_count",
    "navigation.ets2_route.distance_drop_max_m",
    "navigation.ets2_route.distance_step_avg_m",
];

const ETS2_ROUTE_MATCH_KEYS: &[&str] = &[
    "navigation.ets2_route.match_status",
    "navigation.ets2_route.matched_count",
    "navigation.ets2_route.missing_count",
    "navigation.ets2_route.match_ratio",
    "navigation.ets2_route.first_missing_uid",
    "navigation.ets2_route.usable",
    "navigation.ets2_route.import_error",
];

fn uids_from_snapshot(snap: &RouteSnapshot) -> Vec<u64> {
    snap.waypoints
        .iter()
        .map(|wp| wp.uid as u64)
        .collect()
}

fn waypoints_from_snapshot(snap: &RouteSnapshot) -> Vec<Ets2RouteWaypoint> {
    snap.waypoints
        .iter()
        .map(|wp| Ets2RouteWaypoint {
            uid: wp.uid as u64,
            x: wp.x,
            y: wp.y,
            z: wp.z,
            distance: wp.distance,
            time: wp.time,
            flags: wp.flags,
        })
        .collect()
}

fn snapshot_from_route(snap: &RouteSnapshot) -> Ets2RouteSnapshot {
    Ets2RouteSnapshot {
        sequence: snap.sequence,
        route_hash: snap.route_hash,
        valid: snap.valid,
        uids: uids_from_snapshot(snap),
    }
}

/// Poll route SHM, update shared cache, publish snapshot + match diagnostics.
pub fn poll_ets2_route(
    reader: Option<&mut RouteBlackboardReader>,
    graph: &Arc<RouterGraph>,
    cache: &Arc<RwLock<Ets2RouteSharedState>>,
    bb: &SharedBlackboard,
) {
    let Some(reader) = reader else {
        clear_ets2_route(cache, bb);
        return;
    };

    let (snap, route_changed) = if let Some(snap) = reader.read_if_changed() {
        info!(
            "ETS2 route snapshot updated: valid={} waypoints={} hash={:#x} seq={}",
            snap.valid,
            snap.waypoints.len(),
            snap.route_hash,
            snap.sequence,
        );
        (snap, true)
    } else {
        let Some(snap) = reader.read() else {
            clear_ets2_route(cache, bb);
            return;
        };
        (snap, false)
    };

    publish_snapshot_keys(&snap, bb);
    publish_coord_keys(&snap, bb);

    if !route_changed {
        return;
    }

    let ets2_snap = snapshot_from_route(&snap);
    let shm_waypoints = waypoints_from_snapshot(&snap);
    let match_result = if !snap.valid || ets2_snap.uids.is_empty() {
        match_invalid_ets2_route(if !snap.valid {
            "snapshot marked invalid"
        } else {
            "empty waypoint list"
        })
    } else {
        match_ets2_route_uids(graph, &ets2_snap.uids)
    };

    if let Ok(mut guard) = cache.write() {
        guard.snapshot = Some(ets2_snap);
        guard.match_result = Some(match_result.clone());
        guard.waypoints = shm_waypoints;
    }

    publish_match_keys(&match_result, bb);

    info!(
        "ETS2 route match: status={} matched={} missing={} ratio={:.1}% usable={} hash={:#x}",
        match_result.status.as_str(),
        match_result.matched_count,
        match_result.missing_count,
        match_result.match_ratio * 100.0,
        match_result.is_usable,
        snap.route_hash,
    );
}

fn clear_ets2_route(cache: &Arc<RwLock<Ets2RouteSharedState>>, bb: &SharedBlackboard) {
    if let Ok(mut guard) = cache.write() {
        *guard = Ets2RouteSharedState::default();
    }
    publish_unavailable_keys(bb);
    for key in ETS2_ROUTE_MATCH_KEYS {
        bb.remove(key);
    }
    for key in ETS2_ROUTE_COORD_KEYS {
        bb.remove(key);
    }
    publish_match_keys(&match_unavailable_ets2_route(), bb);
}

fn publish_unavailable_keys(bb: &SharedBlackboard) {
    bb.set("navigation.ets2_route.available", "false");
    for key in ETS2_ROUTE_SNAPSHOT_KEYS {
        bb.remove(key);
    }
    for key in ETS2_ROUTE_COORD_KEYS {
        bb.remove(key);
    }
}

fn publish_snapshot_keys(snap: &RouteSnapshot, bb: &SharedBlackboard) {
    bb.set("navigation.ets2_route.available", "true");
    bb.set(
        "navigation.ets2_route.valid",
        if snap.valid { "true" } else { "false" },
    );
    bb.set("navigation.ets2_route.sequence", snap.sequence.to_string());
    bb.set("navigation.ets2_route.hash", snap.route_hash.to_string());
    bb.set(
        "navigation.ets2_route.waypoint_count",
        snap.waypoints.len().to_string(),
    );
    bb.set("navigation.ets2_route.source", "ets2_shm");

    if snap.valid && snap.waypoints.len() <= ETS2_ROUTE_RAW_UIDS_LIMIT {
        let uids: Vec<i64> = snap.waypoints.iter().map(|wp| wp.uid).collect();
        if let Ok(json) = serde_json::to_string(&uids) {
            bb.set("navigation.ets2_route.raw_uids", json);
        }
    } else {
        bb.remove("navigation.ets2_route.raw_uids");
    }
}

fn publish_coord_keys(snap: &RouteSnapshot, bb: &SharedBlackboard) {
    let diag = diagnose_route_coords(snap.valid, &snap.waypoints);
    bb.set(
        "navigation.ets2_route.position_count",
        diag.position_count.to_string(),
    );
    bb.set(
        "navigation.ets2_route.position_ratio",
        format!("{:.4}", diag.position_ratio),
    );
    bb.set(
        "navigation.ets2_route.distance_count",
        diag.distance_count.to_string(),
    );
    bb.set(
        "navigation.ets2_route.time_count",
        diag.time_count.to_string(),
    );
    let first = format_position_triple(diag.first_position);
    if first.is_empty() {
        bb.remove("navigation.ets2_route.first_position");
    } else {
        bb.set("navigation.ets2_route.first_position", &first);
    }
    let last = format_position_triple(diag.last_position);
    if last.is_empty() {
        bb.remove("navigation.ets2_route.last_position");
    } else {
        bb.set("navigation.ets2_route.last_position", &last);
    }
    bb.set(
        "navigation.ets2_route.coord_source",
        diag.coord_source.as_str(),
    );
    bb.set(
        "navigation.ets2_route.coord_status",
        diag.coord_status.as_str(),
    );

    let dist = diagnose_route_distances(&snap.waypoints);
    bb.set(
        "navigation.ets2_route.distance_untrusted_count",
        dist.distance_untrusted_count.to_string(),
    );
    publish_optional_f32_key(
        bb,
        "navigation.ets2_route.distance_first_m",
        dist.distance_first_m,
    );
    publish_optional_f32_key(
        bb,
        "navigation.ets2_route.distance_last_m",
        dist.distance_last_m,
    );
    publish_optional_f32_key(bb, "navigation.ets2_route.distance_min_m", dist.distance_min_m);
    publish_optional_f32_key(bb, "navigation.ets2_route.distance_max_m", dist.distance_max_m);
    bb.set(
        "navigation.ets2_route.distance_monotonic_status",
        dist.distance_monotonic_status.as_str(),
    );
    bb.set(
        "navigation.ets2_route.distance_increase_count",
        dist.distance_increase_count.to_string(),
    );
    bb.set(
        "navigation.ets2_route.distance_drop_max_m",
        format!("{:.1}", dist.distance_drop_max_m),
    );
    if let Some(step) = dist.distance_step_avg_m {
        bb.set(
            "navigation.ets2_route.distance_step_avg_m",
            format!("{:.1}", step),
        );
    } else {
        bb.remove("navigation.ets2_route.distance_step_avg_m");
    }
}

fn publish_optional_f32_key(bb: &SharedBlackboard, key: &str, value: Option<f32>) {
    if let Some(v) = value {
        bb.set(key, format!("{v:.1}"));
    } else {
        bb.remove(key);
    }
}

fn publish_match_keys(result: &Ets2RouteMatchResult, bb: &SharedBlackboard) {
    bb.set(
        "navigation.ets2_route.match_status",
        result.status.as_str(),
    );
    bb.set(
        "navigation.ets2_route.matched_count",
        result.matched_count.to_string(),
    );
    bb.set(
        "navigation.ets2_route.missing_count",
        result.missing_count.to_string(),
    );
    bb.set(
        "navigation.ets2_route.match_ratio",
        format!("{:.4}", result.match_ratio),
    );
    bb.set(
        "navigation.ets2_route.usable",
        if result.is_usable { "true" } else { "false" },
    );
    if let Some(uid) = result.first_missing_uid {
        bb.set("navigation.ets2_route.first_missing_uid", uid.to_string());
    } else {
        bb.remove("navigation.ets2_route.first_missing_uid");
    }
    if let Some(ref err) = result.import_error {
        bb.set("navigation.ets2_route.import_error", err.clone());
    } else {
        bb.remove("navigation.ets2_route.import_error");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use truckpilot_plugin_api::ets2_route::Ets2RouteMatchStatus;
    use truckpilot_telemetry::nav_route::{
        RouteCoordSource, RouteCoordStatus, RouteDistanceMonotonicStatus, RouteWaypoint,
        ROUTE_WP_FLAG_HAS_DISTANCE, ROUTE_WP_FLAG_HAS_POSITION, ROUTE_WP_FLAG_UNTRUSTED,
    };

    fn tiny_graph() -> Arc<RouterGraph> {
        Arc::new(RouterGraph::new(
            vec![(1, 0.0, 0.0), (2, 10.0, 0.0), (3, 20.0, 0.0), (4, 30.0, 0.0)],
            vec![(1, 2, 10.0), (2, 3, 10.0), (3, 4, 10.0)],
        ))
    }

    fn snap_with_waypoints(valid: bool, waypoints: Vec<RouteWaypoint>) -> RouteSnapshot {
        RouteSnapshot {
            sequence: 1,
            route_hash: 42,
            valid,
            flags: 0,
            bb_status: 0,
            waypoints,
            ..Default::default()
        }
    }

    #[test]
    fn publish_match_keys_sets_usable() {
        let bb = SharedBlackboard::new();
        let result = match_ets2_route_uids(&tiny_graph(), &[1, 2, 3, 4]);
        publish_match_keys(&result, &bb);
        assert_eq!(
            bb.get("navigation.ets2_route.match_status").as_deref(),
            Some("matched")
        );
        assert_eq!(
            bb.get("navigation.ets2_route.usable").as_deref(),
            Some("true")
        );
    }

    #[test]
    fn invalid_snapshot_produces_invalid_status() {
        let bb = SharedBlackboard::new();
        let result = match_invalid_ets2_route("snapshot marked invalid");
        publish_match_keys(&result, &bb);
        assert_eq!(
            bb.get("navigation.ets2_route.match_status").as_deref(),
            Some(Ets2RouteMatchStatus::Invalid.as_str())
        );
    }

    #[test]
    fn coord_keys_unavailable_without_position_flags() {
        let bb = SharedBlackboard::new();
        let snap = snap_with_waypoints(
            true,
            vec![
                RouteWaypoint {
                    uid: 1,
                    ..Default::default()
                },
                RouteWaypoint {
                    uid: 2,
                    ..Default::default()
                },
            ],
        );
        publish_coord_keys(&snap, &bb);
        assert_eq!(
            bb.get("navigation.ets2_route.position_count").as_deref(),
            Some("0")
        );
        assert_eq!(
            bb.get("navigation.ets2_route.coord_status").as_deref(),
            Some(RouteCoordStatus::Unavailable.as_str())
        );
        assert_eq!(
            bb.get("navigation.ets2_route.coord_source").as_deref(),
            Some(RouteCoordSource::GraphOnly.as_str())
        );
        assert!(bb.get("navigation.ets2_route.first_position").is_none());
    }

    #[test]
    fn coord_keys_partial_positions() {
        let bb = SharedBlackboard::new();
        let snap = snap_with_waypoints(
            true,
            vec![
                RouteWaypoint {
                    uid: 1,
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                    flags: ROUTE_WP_FLAG_HAS_POSITION,
                    ..Default::default()
                },
                RouteWaypoint {
                    uid: 2,
                    ..Default::default()
                },
            ],
        );
        publish_coord_keys(&snap, &bb);
        assert_eq!(
            bb.get("navigation.ets2_route.position_count").as_deref(),
            Some("1")
        );
        assert_eq!(
            bb.get("navigation.ets2_route.coord_status").as_deref(),
            Some(RouteCoordStatus::Partial.as_str())
        );
        assert_eq!(
            bb.get("navigation.ets2_route.first_position").as_deref(),
            Some("1.0,2.0,3.0")
        );
    }

    #[test]
    fn coord_keys_distance_only_untrusted() {
        let bb = SharedBlackboard::new();
        let snap = snap_with_waypoints(
            true,
            vec![RouteWaypoint {
                uid: 1,
                distance: 500.0,
                flags: ROUTE_WP_FLAG_HAS_DISTANCE | ROUTE_WP_FLAG_UNTRUSTED,
                ..Default::default()
            }],
        );
        publish_coord_keys(&snap, &bb);
        assert_eq!(
            bb.get("navigation.ets2_route.distance_count").as_deref(),
            Some("1")
        );
        assert_eq!(
            bb.get("navigation.ets2_route.coord_status").as_deref(),
            Some(RouteCoordStatus::Untrusted.as_str())
        );
    }

    #[test]
    fn distance_keys_monotonic_falling() {
        let bb = SharedBlackboard::new();
        let snap = snap_with_waypoints(
            true,
            vec![
                RouteWaypoint {
                    uid: 1,
                    distance: 3000.0,
                    flags: ROUTE_WP_FLAG_HAS_DISTANCE | ROUTE_WP_FLAG_UNTRUSTED,
                    ..Default::default()
                },
                RouteWaypoint {
                    uid: 2,
                    distance: 2000.0,
                    flags: ROUTE_WP_FLAG_HAS_DISTANCE | ROUTE_WP_FLAG_UNTRUSTED,
                    ..Default::default()
                },
                RouteWaypoint {
                    uid: 3,
                    distance: 1000.0,
                    flags: ROUTE_WP_FLAG_HAS_DISTANCE | ROUTE_WP_FLAG_UNTRUSTED,
                    ..Default::default()
                },
            ],
        );
        publish_coord_keys(&snap, &bb);
        assert_eq!(
            bb.get("navigation.ets2_route.distance_monotonic_status").as_deref(),
            Some(RouteDistanceMonotonicStatus::Ok.as_str())
        );
        assert_eq!(
            bb.get("navigation.ets2_route.distance_untrusted_count").as_deref(),
            Some("3")
        );
        assert_eq!(
            bb.get("navigation.ets2_route.distance_first_m").as_deref(),
            Some("3000.0")
        );
    }

    #[test]
    fn distance_keys_empty_monotonic_none() {
        let bb = SharedBlackboard::new();
        let snap = snap_with_waypoints(true, vec![RouteWaypoint { uid: 1, ..Default::default() }]);
        publish_coord_keys(&snap, &bb);
        assert_eq!(
            bb.get("navigation.ets2_route.distance_monotonic_status").as_deref(),
            Some(RouteDistanceMonotonicStatus::None.as_str())
        );
        assert!(bb.get("navigation.ets2_route.distance_first_m").is_none());
    }
}
