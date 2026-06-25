//! Read-only lane geometry debug model for future overlay / lane-keeper work.
//!
//! No steering, no resolver activation, no ETS2 memory reads beyond existing SHM.

use crate::nav_route::{RouteSnapshot, ROUTE_WP_FLAG_HAS_POSITION};
use crate::status_report::{StatusReport, StatusVerdict};
use crate::shm::ShmReader;

/// Horizontal map point (ETS2 world X/Z metres).
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct MapPoint2D {
    /// ETS2 world X (metres).
    pub x: f64,
    /// ETS2 world Z (metres).
    pub z: f64,
}

/// One spline segment between two centerline indices.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct SplineSegment {
    /// Index into the parent centerline point array where this segment starts.
    pub start_idx: usize,
    /// Index into the parent centerline point array where this segment ends.
    pub end_idx: usize,
    /// Euclidean length of the segment in metres.
    pub length_m: f64,
}

/// Where lane geometry came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum LaneDataSource {
    /// Synthesised mock data — no live SHM available or route invalid.
    Mock,
    /// Built from live route waypoints in the route blackboard SHM.
    RouteBlackboard,
}

/// Read-only lane debug payload for overlay visualization.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct LaneDebugSnapshot {
    /// `true` when a real lane model was built from live route data.
    pub lane_model_valid: bool,
    /// Signed lateral offset of the truck from the centerline in metres (positive = left).
    pub ego_offset_m: f64,
    /// Ordered centerline waypoints in ETS2 world coordinates.
    pub centerline_points: Vec<MapPoint2D>,
    /// Left lane-edge boundary points (parallel to centerline).
    pub left_lane_points: Vec<MapPoint2D>,
    /// Right lane-edge boundary points (parallel to centerline).
    pub right_lane_points: Vec<MapPoint2D>,
    /// Estimated centerline curvature in 1/m (three-point approximation).
    pub curvature: f64,
    /// Lookahead horizon used for curvature sampling in metres.
    pub lookahead_m: f64,
    /// Graph node UIDs corresponding to each centerline point.
    pub node_ids: Vec<i64>,
    /// Pre-computed Euclidean lengths for each centerline segment.
    pub spline_segments: Vec<SplineSegment>,
    /// Origin of this snapshot (live route blackboard or mock fixture).
    pub source: LaneDataSource,
    /// 0..1 confidence in the lane model (diagnostic only).
    pub confidence: f64,
}

const MOCK_HALF_WIDTH_M: f64 = 1.85;
const MOCK_LOOKAHEAD_M: f64 = 80.0;

fn mock_lane_snapshot() -> LaneDebugSnapshot {
    let centerline_points = vec![
        MapPoint2D { x: 0.0, z: 0.0 },
        MapPoint2D { x: 0.0, z: 20.0 },
        MapPoint2D { x: 5.0, z: 40.0 },
        MapPoint2D { x: 15.0, z: 60.0 },
        MapPoint2D { x: 30.0, z: 80.0 },
    ];
    let left_lane_points = centerline_points
        .iter()
        .map(|p| MapPoint2D {
            x: p.x - MOCK_HALF_WIDTH_M,
            z: p.z,
        })
        .collect();
    let right_lane_points = centerline_points
        .iter()
        .map(|p| MapPoint2D {
            x: p.x + MOCK_HALF_WIDTH_M,
            z: p.z,
        })
        .collect();
    let spline_segments = (0..centerline_points.len().saturating_sub(1))
        .map(|i| SplineSegment {
            start_idx: i,
            end_idx: i + 1,
            length_m: segment_length(centerline_points[i], centerline_points[i + 1]),
        })
        .collect();

    LaneDebugSnapshot {
        lane_model_valid: false,
        ego_offset_m: 0.0,
        centerline_points,
        left_lane_points,
        right_lane_points,
        curvature: 0.002,
        lookahead_m: MOCK_LOOKAHEAD_M,
        node_ids: vec![101, 102, 103, 104, 105],
        spline_segments,
        source: LaneDataSource::Mock,
        confidence: 0.0,
    }
}

fn segment_length(a: MapPoint2D, b: MapPoint2D) -> f64 {
    let dx = b.x - a.x;
    let dz = b.z - a.z;
    (dx * dx + dz * dz).sqrt()
}

fn estimate_curvature(points: &[MapPoint2D]) -> f64 {
    if points.len() < 3 {
        return 0.0;
    }
    let a = points[0];
    let b = points[points.len() / 2];
    let c = points[points.len() - 1];
    let ab = segment_length(a, b);
    let bc = segment_length(b, c);
    let ac = segment_length(a, c);
    let denom = (ab * bc * ac).max(1e-6);
    let area2 = ((b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x)).abs();
    2.0 * area2 / denom
}

fn lateral_offset_to_polyline(truck: MapPoint2D, points: &[MapPoint2D]) -> f64 {
    if points.len() < 2 {
        return 0.0;
    }
    let mut best = f64::INFINITY;
    for w in points.windows(2) {
        let a = w[0];
        let b = w[1];
        let abx = b.x - a.x;
        let abz = b.z - a.z;
        let len2 = abx * abx + abz * abz;
        if len2 <= 1e-9 {
            continue;
        }
        let t = ((truck.x - a.x) * abx + (truck.z - a.z) * abz) / len2;
        let t = t.clamp(0.0, 1.0);
        let px = a.x + abx * t;
        let pz = a.z + abz * t;
        let dx = truck.x - px;
        let dz = truck.z - pz;
        let dist = (dx * dx + dz * dz).sqrt();
        let cross = abx * (truck.z - a.z) - abz * (truck.x - a.x);
        let signed = if cross >= 0.0 { dist } else { -dist };
        if dist < best.abs() || best.is_infinite() {
            best = signed;
        }
    }
    if best.is_infinite() { 0.0 } else { best }
}

fn from_route_blackboard(route: &RouteSnapshot, truck: Option<MapPoint2D>) -> Option<LaneDebugSnapshot> {
    let positioned: Vec<_> = route
        .waypoints
        .iter()
        .filter(|wp| wp.flags & ROUTE_WP_FLAG_HAS_POSITION != 0)
        .collect();
    if positioned.len() < 2 {
        return None;
    }

    let centerline_points: Vec<MapPoint2D> = positioned
        .iter()
        .map(|wp| MapPoint2D {
            x: wp.x as f64,
            z: wp.z as f64,
        })
        .collect();
    let node_ids: Vec<i64> = positioned.iter().map(|wp| wp.uid).collect();
    let left_lane_points: Vec<MapPoint2D> = centerline_points
        .iter()
        .map(|p| MapPoint2D {
            x: p.x - MOCK_HALF_WIDTH_M,
            z: p.z,
        })
        .collect();
    let right_lane_points: Vec<MapPoint2D> = centerline_points
        .iter()
        .map(|p| MapPoint2D {
            x: p.x + MOCK_HALF_WIDTH_M,
            z: p.z,
        })
        .collect();
    let spline_segments = (0..centerline_points.len().saturating_sub(1))
        .map(|i| SplineSegment {
            start_idx: i,
            end_idx: i + 1,
            length_m: segment_length(centerline_points[i], centerline_points[i + 1]),
        })
        .collect();
    let ego_offset_m = truck
        .map(|t| lateral_offset_to_polyline(t, &centerline_points))
        .unwrap_or(0.0);

    Some(LaneDebugSnapshot {
        lane_model_valid: true,
        ego_offset_m,
        curvature: estimate_curvature(&centerline_points),
        lookahead_m: MOCK_LOOKAHEAD_M,
        centerline_points,
        left_lane_points,
        right_lane_points,
        node_ids,
        spline_segments,
        source: LaneDataSource::RouteBlackboard,
        confidence: 0.75,
    })
}

/// Build lane debug geometry from route blackboard data or mock fixtures.
pub fn build_lane_debug(
    route: Option<&RouteSnapshot>,
    status: &StatusReport,
) -> LaneDebugSnapshot {
    let truck = ShmReader::open()
        .ok()
        .and_then(|mut r| r.read())
        .map(|t| MapPoint2D {
            x: t.position[0],
            z: t.position[2],
        });

    let can_use_route = status.verdict == StatusVerdict::SafeCold
        && status.route_valid
        && !status.resolver_off;

    if can_use_route {
        if let Some(route) = route {
            if let Some(model) = from_route_blackboard(route, truck) {
                return model;
            }
        }
    }

    let mut mock = mock_lane_snapshot();
    if status.resolver_off || !status.route_valid {
        mock.lane_model_valid = false;
        mock.confidence = 0.0;
    }
    mock
}

/// Lane keeper must stay off unless all safety gates pass.
pub fn lane_keeper_allowed(status: &StatusReport, lane: &LaneDebugSnapshot) -> bool {
    status.verdict == StatusVerdict::SafeCold
        && status.route_valid
        && lane.lane_model_valid
        && !status.resolver_off
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nav_route::{RouteSnapshot, RouteWaypoint, ROUTE_WP_FLAG_HAS_POSITION};
    use crate::status_report::{evaluate_status, RawStatusInputs, StatusVerdict};
    #[cfg(windows)]
    use crate::dll_perf::{DllPerfSnapshot, DLL_PERF_MAGIC, DLL_PERF_VERSION};
    use crate::nav_route::{RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE, ROUTE_BB_STATUS_DLL_ACTIVE};

    fn route_with_positions() -> RouteSnapshot {
        RouteSnapshot {
            valid: true,
            bb_status: ROUTE_BB_STATUS_DLL_ACTIVE,
            resolve_status: RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE + 1, // not off
            waypoints: vec![
                RouteWaypoint {
                    uid: 1,
                    x: 0.0,
                    z: 0.0,
                    flags: ROUTE_WP_FLAG_HAS_POSITION,
                    ..Default::default()
                },
                RouteWaypoint {
                    uid: 2,
                    x: 0.0,
                    z: 50.0,
                    flags: ROUTE_WP_FLAG_HAS_POSITION,
                    ..Default::default()
                },
            ],
            ..Default::default()
        }
    }

    #[cfg(windows)]
    #[test]
    fn resolver_off_yields_invalid_mock_lane() {
        let mut perf = DllPerfSnapshot {
            magic: DLL_PERF_MAGIC,
            version: DLL_PERF_VERSION,
            ..Default::default()
        };
        perf.diag_level_code = 9;
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
        assert!(!lane.lane_model_valid);
        assert_eq!(lane.source, LaneDataSource::Mock);
        assert!(!lane_keeper_allowed(&status, &lane));
    }

    #[cfg(windows)]
    #[test]
    fn route_blackboard_builds_centerline_when_valid() {
        let mut perf = DllPerfSnapshot {
            magic: DLL_PERF_MAGIC,
            version: DLL_PERF_VERSION,
            resolver_attempts: 0,
            pattern_scan_count: 0,
            worker_walk_count: 0,
            worker_wake_set_event_count: 0,
            ..Default::default()
        };
        perf.diag_level_code = 9;
        let mut route = route_with_positions();
        route.resolve_status = 7; // waypoints_collected — not safe-off
        let status = evaluate_status(&RawStatusInputs {
            perf: Some(perf),
            route: Some(route.clone()),
            telemetry_shm_present: true,
        });
        assert_eq!(status.verdict, StatusVerdict::SafeCold);
        assert!(status.route_valid);
        let lane = build_lane_debug(Some(&route), &status);
        assert!(lane.lane_model_valid);
        assert_eq!(lane.source, LaneDataSource::RouteBlackboard);
        assert_eq!(lane.centerline_points.len(), 2);
        assert_eq!(lane.node_ids, vec![1, 2]);
        assert!(lane_keeper_allowed(&status, &lane));
    }

    #[test]
    fn mock_fixture_has_segments_and_node_ids() {
        let mock = mock_lane_snapshot();
        assert_eq!(mock.source, LaneDataSource::Mock);
        assert_eq!(mock.node_ids.len(), mock.centerline_points.len());
        assert_eq!(mock.spline_segments.len(), mock.centerline_points.len() - 1);
    }
}
