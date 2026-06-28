//! Read-only planned path model (PlannedPathData v1).
//!
//! Inspired by ETS2LA concepts (PrefabPath, NavCurve, Hermite splines, lateral
//! offset, path curvature, semaphores) without copying ETS2LA code. v1 is
//! display-only — no steering, engage, or resolver activation.

use std::collections::HashMap;

use serde::{Deserialize, Serialize};
use truckpilot_map_parser::graph::{GraphEdge, GraphNode, MapGraph, PrefabAiPath};

/// Where planned path geometry was sourced from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PlannedPathSource {
    /// Synthesised fixture for debug / overlay.
    Mock,
    /// Offline map graph (future).
    OfflineGraph,
    /// ETS2 route blackboard SHM (future live path).
    RouteBlackboard,
    /// Prefab AI path from map graph.
    PrefabAiPath,
    /// NavCurve segment from map graph.
    NavCurve,
    /// Source not classified.
    Unknown,
}

/// Kind of one planned-path segment.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PlannedPathItemKind {
    /// Road graph edge.
    RoadEdge,
    /// Prefab internal path.
    PrefabPath,
    /// NavCurve spline segment.
    NavCurve,
    /// Lane-change manoeuvre segment.
    LaneChange,
    /// Junction / merge decision segment.
    Junction,
    /// Unclassified segment.
    Unknown,
}

/// One point along a planned path in ETS2 world space.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PathPoint {
    /// World X (metres).
    pub x: f64,
    /// World Y (metres).
    pub y: f64,
    /// World Z (metres).
    pub z: f64,
    /// Arc length from the start of the parent item (metres).
    pub distance_m: f64,
    /// Heading in radians (optional).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub heading_rad: Option<f64>,
    /// Signed lateral offset from segment centreline (metres).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lane_offset_m: Option<f64>,
    /// Local curvature in 1/m (optional).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub curvature_1pm: Option<f64>,
}

/// One contiguous segment of the planned path.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlannedPathItem {
    /// Stable item id within the path.
    pub id: u32,
    /// Segment kind.
    pub kind: PlannedPathItemKind,
    /// Start graph node UID (optional).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub node_uid_start: Option<u64>,
    /// End graph node UID (optional).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub node_uid_end: Option<u64>,
    /// Prefab UID when applicable.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub prefab_uid: Option<u64>,
    /// NavCurve / spline index when applicable.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub curve_index: Option<u32>,
    /// Total segment length (metres).
    pub length_m: f64,
    /// Active lane index (0 = rightmost driving lane convention).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lane_index: Option<u32>,
    /// Number of lanes on this segment.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lane_count: Option<u32>,
    /// Lane width (metres).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lane_width_m: Option<f64>,
    /// Lateral offset from centreline (metres).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lateral_offset_m: Option<f64>,
    /// Representative curvature (1/m).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub curvature_1pm: Option<f64>,
    /// Advisory speed (km/h).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub speed_hint_kmh: Option<f64>,
    /// Traffic-light / priority hint label (display only).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub semaphore_hint: Option<String>,
    /// Sampled geometry along the segment.
    pub points: Vec<PathPoint>,
}

/// Nearest projection of ego onto the planned path (display only).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct NearestPathPoint {
    /// Parent [`PlannedPathItem::id`].
    pub item_id: u32,
    /// Distance along the item from its start (metres).
    pub distance_along_m: f64,
    /// Signed crosstrack error (metres; positive = left of path).
    pub crosstrack_m: f64,
    /// Heading error vs path tangent (radians).
    pub heading_error_rad: f64,
    /// 0..1 confidence in the projection.
    pub confidence: f64,
}

/// Read-only safety / preflight mirror bundled with the path.
///
/// `drive_allowed_display_only` is **never** an engage or steering gate in v1.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlannedPathSafety {
    /// Route validity (from status / preflight).
    pub route_valid: bool,
    /// Live lane model validity.
    pub lane_model_valid: bool,
    /// Resolver parked / safe-off indicator.
    pub resolver_safe: bool,
    /// Telemetry recently arriving.
    pub telemetry_fresh: bool,
    /// Output path permitted to act.
    pub input_allowed: bool,
    /// Display-only aggregate — not engage authorization.
    pub drive_allowed_display_only: bool,
    /// Human-readable blockers (includes unknown qualifiers).
    pub reasons: Vec<String>,
}

/// Central read-only planned path payload (v1).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlannedPathData {
    /// `true` when path geometry/items are structurally usable.
    pub valid: bool,
    /// Provenance of geometry.
    pub source: PlannedPathSource,
    /// Optional route id or hash string.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub route_id: Option<String>,
    /// Index into `items` for the current ego segment.
    pub current_index: u32,
    /// Lookahead horizon (metres).
    pub lookahead_m: f64,
    /// Ordered path segments.
    pub items: Vec<PlannedPathItem>,
    /// Ego projection onto the path (optional).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nearest: Option<NearestPathPoint>,
    /// Bundled safety / preflight mirror.
    pub safety: PlannedPathSafety,
}

/// Pretty JSON for CLI / fixture export.
pub fn planned_path_to_json(data: &PlannedPathData) -> String {
    serde_json::to_string_pretty(data).unwrap_or_else(|e| format!("{{\"json_error\":\"{e}\"}}"))
}

fn straight_points(
    x0: f64,
    z0: f64,
    x1: f64,
    z1: f64,
    steps: usize,
    lane_offset_m: f64,
) -> Vec<PathPoint> {
    let mut out = Vec::with_capacity(steps + 1);
    for i in 0..=steps {
        let t = i as f64 / steps as f64;
        let x = x0 + (x1 - x0) * t;
        let z = z0 + (z1 - z0) * t;
        let dist = (x - x0).hypot(z - z0);
        let heading = (x1 - x0).atan2(z1 - z0);
        out.push(PathPoint {
            x,
            y: 0.0,
            z,
            distance_m: dist,
            heading_rad: Some(heading),
            lane_offset_m: Some(lane_offset_m),
            curvature_1pm: Some(0.0),
        });
    }
    out
}

fn curved_points(
    cx: f64,
    cz: f64,
    radius: f64,
    a0: f64,
    a1: f64,
    steps: usize,
    lane_offset_m: f64,
) -> Vec<PathPoint> {
    let mut out = Vec::with_capacity(steps + 1);
    let curvature = if radius.abs() > 1e-6 {
        1.0 / radius.abs()
    } else {
        0.0
    };
    let mut dist = 0.0;
    let mut prev: Option<(f64, f64)> = None;
    for i in 0..=steps {
        let t = i as f64 / steps as f64;
        let a = a0 + (a1 - a0) * t;
        let x = cx + radius * a.sin();
        let z = cz + radius * a.cos();
        if let Some((px, pz)) = prev {
            dist += (x - px).hypot(z - pz);
        }
        prev = Some((x, z));
        let heading = a + std::f64::consts::FRAC_PI_2;
        out.push(PathPoint {
            x,
            y: 0.0,
            z,
            distance_m: dist,
            heading_rad: Some(heading),
            lane_offset_m: Some(lane_offset_m),
            curvature_1pm: Some(curvature),
        });
    }
    out
}

/// Build a synthetic v1 fixture with road, curve, prefab junction, lane change,
/// and nav-curve segments. Geometry only — attach safety via [`with_safety`].
pub fn build_mock_fixture_v1() -> PlannedPathData {
    let item0_points = straight_points(0.0, 0.0, 0.0, 120.0, 6, 0.0);
    let item1_points = curved_points(0.0, 120.0, 45.0, 0.0, std::f64::consts::FRAC_PI_2, 8, 0.0);
    let item2_points = straight_points(45.0, 165.0, 95.0, 165.0, 5, 0.0);
    let item3_points = straight_points(95.0, 165.0, 115.0, 185.0, 4, 1.85);
    let item4_points = straight_points(115.0, 185.0, 115.0, 260.0, 5, -1.85);

    let items = vec![
        PlannedPathItem {
            id: 1,
            kind: PlannedPathItemKind::RoadEdge,
            node_uid_start: Some(10_001),
            node_uid_end: Some(10_002),
            prefab_uid: None,
            curve_index: None,
            length_m: 120.0,
            lane_index: Some(1),
            lane_count: Some(2),
            lane_width_m: Some(3.7),
            lateral_offset_m: Some(0.0),
            curvature_1pm: Some(0.0),
            speed_hint_kmh: Some(80.0),
            semaphore_hint: None,
            points: item0_points,
        },
        PlannedPathItem {
            id: 2,
            kind: PlannedPathItemKind::RoadEdge,
            node_uid_start: Some(10_002),
            node_uid_end: Some(10_003),
            prefab_uid: None,
            curve_index: Some(42),
            length_m: 70.7,
            lane_index: Some(1),
            lane_count: Some(2),
            lane_width_m: Some(3.7),
            lateral_offset_m: Some(0.0),
            curvature_1pm: Some(1.0 / 45.0),
            speed_hint_kmh: Some(50.0),
            semaphore_hint: None,
            points: item1_points,
        },
        PlannedPathItem {
            id: 3,
            kind: PlannedPathItemKind::Junction,
            node_uid_start: Some(10_003),
            node_uid_end: Some(10_004),
            prefab_uid: Some(9001),
            curve_index: None,
            length_m: 50.0,
            lane_index: Some(0),
            lane_count: Some(1),
            lane_width_m: Some(3.5),
            lateral_offset_m: Some(0.0),
            curvature_1pm: Some(0.0),
            speed_hint_kmh: Some(30.0),
            semaphore_hint: Some("priority_merge".into()),
            points: item2_points,
        },
        PlannedPathItem {
            id: 4,
            kind: PlannedPathItemKind::LaneChange,
            node_uid_start: Some(10_004),
            node_uid_end: Some(10_005),
            prefab_uid: None,
            curve_index: None,
            length_m: 28.3,
            lane_index: Some(2),
            lane_count: Some(3),
            lane_width_m: Some(3.7),
            lateral_offset_m: Some(1.85),
            curvature_1pm: Some(0.002),
            speed_hint_kmh: Some(60.0),
            semaphore_hint: None,
            points: item3_points,
        },
        PlannedPathItem {
            id: 5,
            kind: PlannedPathItemKind::NavCurve,
            node_uid_start: Some(10_005),
            node_uid_end: Some(10_006),
            prefab_uid: None,
            curve_index: Some(128),
            length_m: 75.0,
            lane_index: Some(1),
            lane_count: Some(3),
            lane_width_m: Some(3.7),
            lateral_offset_m: Some(-1.85),
            curvature_1pm: Some(0.001),
            speed_hint_kmh: Some(70.0),
            semaphore_hint: Some("red_hold".into()),
            points: item4_points,
        },
    ];

    let nearest = NearestPathPoint {
        item_id: 2,
        distance_along_m: 18.5,
        crosstrack_m: -0.12,
        heading_error_rad: 0.04,
        confidence: 0.92,
    };

    PlannedPathData {
        valid: true,
        source: PlannedPathSource::Mock,
        route_id: Some("mock-fixture-v1".into()),
        current_index: 1,
        lookahead_m: 80.0,
        items,
        nearest: Some(nearest),
        safety: PlannedPathSafety {
            route_valid: false,
            lane_model_valid: false,
            resolver_safe: true,
            telemetry_fresh: true,
            input_allowed: false,
            drive_allowed_display_only: false,
            reasons: vec![
                "route invalid".into(),
                "lane model invalid".into(),
                "input disabled".into(),
            ],
        },
    }
}

/// Replace safety block on an existing path (display-only mirror).
pub fn with_safety(mut data: PlannedPathData, safety: PlannedPathSafety) -> PlannedPathData {
    data.safety = safety;
    data
}

/// One positioned route-blackboard waypoint for read-only path building.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RouteBlackboardWaypoint {
    /// Graph node UID from ETS2 route blackboard SHM.
    pub uid: i64,
    /// World X (metres).
    pub x: f64,
    /// World Y (metres).
    pub y: f64,
    /// World Z (metres).
    pub z: f64,
}

/// Build read-only [`PlannedPathData`] from route-blackboard waypoint geometry.
///
/// Each hop between consecutive waypoints becomes a [`PlannedPathItemKind::RoadEdge`]
/// segment. No map graph, resolver, or steering — display-only polyline.
pub fn planned_path_from_route_blackboard(
    waypoints: &[RouteBlackboardWaypoint],
    route_hash: u64,
    truck_xz: Option<(f64, f64)>,
) -> Option<PlannedPathData> {
    if waypoints.len() < 2 {
        return None;
    }

    let mut items = Vec::with_capacity(waypoints.len() - 1);
    for (idx, pair) in waypoints.windows(2).enumerate() {
        let a = &pair[0];
        let b = &pair[1];
        let id = (idx + 1) as u32;
        let length_m = dist2((a.x, a.z), (b.x, b.z));
        let heading = (b.x - a.x).atan2(b.z - a.z);
        let curvature = if idx > 0 {
            let prev = &waypoints[idx - 1];
            menger_curvature((prev.x, prev.z), (a.x, a.z), (b.x, b.z))
        } else {
            0.0
        };
        let steps = 4usize;
        let mut points = Vec::with_capacity(steps + 1);
        for step in 0..=steps {
            let t = step as f64 / steps as f64;
            let x = a.x + (b.x - a.x) * t;
            let y = a.y + (b.y - a.y) * t;
            let z = a.z + (b.z - a.z) * t;
            points.push(PathPoint {
                x,
                y,
                z,
                distance_m: length_m * t,
                heading_rad: Some(heading),
                lane_offset_m: Some(0.0),
                curvature_1pm: Some(curvature),
            });
        }
        items.push(PlannedPathItem {
            id,
            kind: PlannedPathItemKind::RoadEdge,
            node_uid_start: Some(a.uid as u64),
            node_uid_end: Some(b.uid as u64),
            prefab_uid: None,
            curve_index: None,
            length_m,
            lane_index: Some(0),
            lane_count: Some(1),
            lane_width_m: Some(DEFAULT_LANE_WIDTH_M),
            lateral_offset_m: Some(0.0),
            curvature_1pm: Some(curvature),
            speed_hint_kmh: None,
            semaphore_hint: None,
            points,
        });
    }

    let nearest = truck_xz.and_then(|truck| nearest_on_items(&items, truck));

    Some(PlannedPathData {
        valid: true,
        source: PlannedPathSource::RouteBlackboard,
        route_id: Some(format!("route-bb-{route_hash:016x}")),
        current_index: nearest
            .as_ref()
            .and_then(|n| items.iter().position(|i| i.id == n.item_id))
            .unwrap_or(0) as u32,
        lookahead_m: 80.0,
        items,
        nearest,
        safety: PlannedPathSafety {
            route_valid: true,
            lane_model_valid: false,
            resolver_safe: true,
            telemetry_fresh: truck_xz.is_some(),
            input_allowed: false,
            drive_allowed_display_only: false,
            reasons: vec!["route blackboard polyline (display only)".into()],
        },
    })
}

fn nearest_on_items(items: &[PlannedPathItem], truck: (f64, f64)) -> Option<NearestPathPoint> {
    let (tx, tz) = truck;
    let mut best_item = None;
    let mut best_dist = f64::INFINITY;
    let mut best_along = 0.0;
    let mut best_cross = 0.0;
    let mut best_heading_err = 0.0;

    for item in items {
        let pts = &item.points;
        if pts.len() < 2 {
            continue;
        }
        for w in pts.windows(2) {
            let a = &w[0];
            let b = &w[1];
            let abx = b.x - a.x;
            let abz = b.z - a.z;
            let len2 = abx * abx + abz * abz;
            if len2 <= 1e-9 {
                continue;
            }
            let t = ((tx - a.x) * abx + (tz - a.z) * abz) / len2;
            let t = t.clamp(0.0, 1.0);
            let px = a.x + abx * t;
            let pz = a.z + abz * t;
            let dx = tx - px;
            let dz = tz - pz;
            let dist = (dx * dx + dz * dz).sqrt();
            if dist >= best_dist {
                continue;
            }
            let cross = abx * (tz - a.z) - abz * (tx - a.x);
            let signed = if cross >= 0.0 { dist } else { -dist };
            let seg_heading = abx.atan2(abz);
            best_dist = dist;
            best_item = Some(item.id);
            best_along = a.distance_m + (b.distance_m - a.distance_m) * t;
            best_cross = signed;
            best_heading_err = seg_heading;
        }
    }

    best_item.map(|item_id| NearestPathPoint {
        item_id,
        distance_along_m: best_along,
        crosstrack_m: best_cross,
        heading_error_rad: best_heading_err,
        confidence: 0.6,
    })
}

// ---------------------------------------------------------------------------
// Offline map-graph → PlannedPathData (read-only, display only)
// ---------------------------------------------------------------------------

const DEFAULT_LANE_WIDTH_M: f64 = 3.7;

fn dist2(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

/// Menger curvature (1/m) through three XZ points; `0.0` if degenerate.
fn menger_curvature(p0: (f64, f64), p1: (f64, f64), p2: (f64, f64)) -> f64 {
    let denom = dist2(p0, p1) * dist2(p1, p2) * dist2(p2, p0);
    if denom < 1e-9 {
        return 0.0;
    }
    // 2 * signed triangle area (cross product of the two edge vectors).
    let cross = (p1.0 - p0.0) * (p2.1 - p0.1) - (p2.0 - p0.0) * (p1.1 - p0.1);
    (2.0 * cross.abs()) / denom
}

/// Per-point curvature along an XZ polyline (endpoints copy their neighbour).
fn polyline_curvatures(pts: &[(f64, f64)]) -> Vec<f64> {
    let n = pts.len();
    let mut out = vec![0.0; n];
    for i in 1..n.saturating_sub(1) {
        out[i] = menger_curvature(pts[i - 1], pts[i], pts[i + 1]);
    }
    if n >= 2 {
        out[0] = out[1];
        out[n - 1] = out[n - 2];
    }
    out
}

/// Sample a prefab AI-path spline into [`PathPoint`]s, lerping the lane offset
/// from `start_off` to `end_off`. Returns the points plus the max curvature.
fn points_from_spline(
    spline: &[[f32; 3]],
    start_off: f64,
    end_off: f64,
) -> (Vec<PathPoint>, f64) {
    let xz: Vec<(f64, f64)> = spline.iter().map(|p| (p[0] as f64, p[2] as f64)).collect();
    let curv = polyline_curvatures(&xz);
    let mut out = Vec::with_capacity(spline.len());
    let mut dist = 0.0;
    let mut prev: Option<(f64, f64)> = None;
    let last = spline.len().saturating_sub(1).max(1);
    for (i, p) in spline.iter().enumerate() {
        let (x, z) = (p[0] as f64, p[2] as f64);
        if let Some(pv) = prev {
            dist += dist2(pv, (x, z));
        }
        let heading = prev.map(|pv| (x - pv.0).atan2(z - pv.1));
        prev = Some((x, z));
        let t = i as f64 / last as f64;
        out.push(PathPoint {
            x,
            y: p[1] as f64,
            z,
            distance_m: dist,
            heading_rad: heading,
            lane_offset_m: Some(start_off + (end_off - start_off) * t),
            curvature_1pm: Some(curv[i]),
        });
    }
    let max_c = curv.iter().copied().fold(0.0_f64, f64::max);
    (out, max_c)
}

/// Straight road-edge geometry between two graph nodes at constant curvature.
fn points_from_edge(a: &GraphNode, b: &GraphNode, steps: usize, lane_off: f64, curvature: f64) -> Vec<PathPoint> {
    let heading = (b.x - a.x).atan2(b.z - a.z);
    (0..=steps)
        .map(|i| {
            let t = i as f64 / steps as f64;
            let x = a.x + (b.x - a.x) * t;
            let z = a.z + (b.z - a.z) * t;
            PathPoint {
                x,
                y: a.y + (b.y - a.y) * t,
                z,
                distance_m: (x - a.x).hypot(z - a.z),
                heading_rad: Some(heading),
                lane_offset_m: Some(lane_off),
                curvature_1pm: Some(curvature),
            }
        })
        .collect()
}

fn item_from_prefab_ai_path(
    id: u32,
    a: u64,
    b: u64,
    pap: &PrefabAiPath,
    graph: &MapGraph,
) -> PlannedPathItem {
    let kind = if pap.start_lane_idx != pap.end_lane_idx {
        PlannedPathItemKind::LaneChange
    } else if !pap.curve_indices.is_empty() {
        PlannedPathItemKind::NavCurve
    } else if pap.semaphore_id.is_some() {
        PlannedPathItemKind::Junction
    } else {
        PlannedPathItemKind::PrefabPath
    };
    let start_off = pap.start_lane_idx as f64 * DEFAULT_LANE_WIDTH_M;
    let end_off = pap.end_lane_idx as f64 * DEFAULT_LANE_WIDTH_M;
    let (points, curvature) = points_from_spline(&pap.spline_points, start_off, end_off);
    let prefab_uid = graph
        .prefabs
        .iter()
        .find(|p| p.connected_node_uids.contains(&a) && p.connected_node_uids.contains(&b))
        .map(|p| p.uid);

    PlannedPathItem {
        id,
        kind,
        node_uid_start: Some(a),
        node_uid_end: Some(b),
        prefab_uid,
        curve_index: pap.curve_indices.first().map(|c| *c as u32),
        length_m: pap.length_m as f64,
        lane_index: Some(pap.end_lane_idx as u32),
        lane_count: None,
        lane_width_m: Some(DEFAULT_LANE_WIDTH_M),
        lateral_offset_m: Some(end_off),
        curvature_1pm: Some(curvature),
        speed_hint_kmh: pap.speed_kmh.map(|s| s as f64),
        semaphore_hint: pap.semaphore_id.map(|id| format!("semaphore #{id}")),
        points,
    }
}

fn item_from_edge(id: u32, edge: &GraphEdge, a: &GraphNode, b: &GraphNode, curvature: f64) -> PlannedPathItem {
    let lane_off = edge.road_offset_m as f64;
    PlannedPathItem {
        id,
        kind: PlannedPathItemKind::RoadEdge,
        node_uid_start: Some(edge.from),
        node_uid_end: Some(edge.to),
        prefab_uid: None,
        curve_index: None,
        length_m: edge.distance_m,
        lane_index: Some(0),
        lane_count: Some(edge.lanes as u32),
        lane_width_m: Some(edge.lane_width_m as f64),
        lateral_offset_m: Some(lane_off),
        curvature_1pm: Some(curvature),
        speed_hint_kmh: edge.speed_limit_kmh,
        semaphore_hint: None,
        points: points_from_edge(a, b, 6, lane_off, curvature),
    }
}

/// Build read-only [`PlannedPathData`] from a routing [`MapGraph`] along an
/// ordered list of node UIDs. Prefab AI-paths take precedence over plain road
/// edges for a hop; missing hops are skipped. `source = OfflineGraph`.
///
/// Geometry only — the caller attaches a display-only [`PlannedPathSafety`].
pub fn planned_path_from_map_graph(graph: &MapGraph, route_node_uids: &[u64]) -> PlannedPathData {
    let node_by_uid: HashMap<u64, &GraphNode> = graph.nodes.iter().map(|n| (n.uid, n)).collect();
    let mut items = Vec::new();
    let mut id = 0u32;

    for (idx, w) in route_node_uids.windows(2).enumerate() {
        let (a, b) = (w[0], w[1]);
        if let Some(pap) = graph
            .prefab_ai_paths
            .iter()
            .find(|p| p.from_node_uid == a && p.to_node_uid == b)
        {
            id += 1;
            items.push(item_from_prefab_ai_path(id, a, b, pap, graph));
        } else if let (Some(edge), Some(na), Some(nb)) = (
            graph.edges.iter().find(|e| e.from == a && e.to == b),
            node_by_uid.get(&a),
            node_by_uid.get(&b),
        ) {
            // Road-edge curvature = bend at the start node (prev, a, b).
            let curvature = idx
                .checked_sub(1)
                .and_then(|i| node_by_uid.get(&route_node_uids[i]))
                .map(|prev| menger_curvature((prev.x, prev.z), (na.x, na.z), (nb.x, nb.z)))
                .unwrap_or(0.0);
            id += 1;
            items.push(item_from_edge(id, edge, na, nb, curvature));
        }
    }

    PlannedPathData {
        valid: !items.is_empty(),
        source: PlannedPathSource::OfflineGraph,
        route_id: None,
        current_index: 0,
        lookahead_m: 80.0,
        items,
        nearest: None,
        // Display-only: no live gates here → drive display stays false.
        safety: PlannedPathSafety {
            route_valid: false,
            lane_model_valid: false,
            resolver_safe: true,
            telemetry_fresh: false,
            input_allowed: false,
            drive_allowed_display_only: false,
            reasons: vec!["offline graph fixture (display only)".into()],
        },
    }
}

/// Build a v1 path from the embedded minimal offline-graph fixture
/// (`tests/fixtures/offline_graph_mini.json`). `source = OfflineGraph`.
pub fn build_offline_fixture_v1() -> PlannedPathData {
    const FIXTURE: &str = include_str!("../tests/fixtures/offline_graph_mini.json");
    let graph: MapGraph =
        serde_json::from_str(FIXTURE).expect("embedded offline_graph_mini fixture must parse");
    let route = [10001u64, 10002, 10003, 10004, 10005, 10006];
    let mut data = planned_path_from_map_graph(&graph, &route);
    data.route_id = Some("offline-graph-mini-v1".into());
    data.current_index = 1;
    data.nearest = Some(NearestPathPoint {
        item_id: 2,
        distance_along_m: 12.0,
        crosstrack_m: -0.08,
        heading_error_rad: 0.03,
        confidence: 0.9,
    });
    data.safety = PlannedPathSafety {
        route_valid: false,
        lane_model_valid: false,
        resolver_safe: true,
        telemetry_fresh: true,
        input_allowed: false,
        drive_allowed_display_only: false,
        reasons: vec![
            "route invalid".into(),
            "lane model invalid".into(),
            "input disabled".into(),
        ],
    };
    data
}

/// Aggregate stats for overlay debug panels.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PlannedPathDebugStats {
    /// Number of items in the path.
    pub item_count: usize,
    /// Current item id (from `current_index`).
    pub current_item_id: Option<u32>,
    /// Current item kind label.
    pub current_item_kind: Option<PlannedPathItemKind>,
    /// Nearest crosstrack (metres) when projection present.
    pub nearest_crosstrack_m: Option<f64>,
    /// Min item curvature (1/m) when any populated.
    pub curvature_min_1pm: Option<f64>,
    /// Max item curvature (1/m) when any populated.
    pub curvature_max_1pm: Option<f64>,
    /// Count of junction + prefab_path items.
    pub junction_prefab_count: usize,
    /// Count of items with a semaphore hint.
    pub semaphore_hint_count: usize,
}

/// Compute read-only debug stats from a planned path.
pub fn planned_path_debug_stats(data: &PlannedPathData) -> PlannedPathDebugStats {
    let cur = data.items.get(data.current_index as usize);
    let mut curvatures: Vec<f64> = data
        .items
        .iter()
        .filter_map(|i| i.curvature_1pm)
        .collect();
    curvatures.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    PlannedPathDebugStats {
        item_count: data.items.len(),
        current_item_id: cur.map(|i| i.id),
        current_item_kind: cur.map(|i| i.kind),
        nearest_crosstrack_m: data.nearest.map(|n| n.crosstrack_m),
        curvature_min_1pm: curvatures.first().copied(),
        curvature_max_1pm: curvatures.last().copied(),
        junction_prefab_count: data
            .items
            .iter()
            .filter(|i| {
                matches!(
                    i.kind,
                    PlannedPathItemKind::Junction | PlannedPathItemKind::PrefabPath
                )
            })
            .count(),
        semaphore_hint_count: data
            .items
            .iter()
            .filter(|i| i.semaphore_hint.is_some())
            .count(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn json_roundtrip_preserves_fixture() {
        let original = build_mock_fixture_v1();
        let json = planned_path_to_json(&original);
        let parsed: PlannedPathData = serde_json::from_str(&json).expect("valid json");
        assert_eq!(parsed.valid, original.valid);
        assert_eq!(parsed.source, original.source);
        assert_eq!(parsed.route_id, original.route_id);
        assert_eq!(parsed.items.len(), original.items.len());
        assert_eq!(parsed.items[3].kind, PlannedPathItemKind::LaneChange);
        assert_eq!(parsed.nearest, original.nearest);
        assert_eq!(parsed.safety, original.safety);
        // Re-serialize to ensure stable JSON output.
        let json2 = planned_path_to_json(&parsed);
        let reparsed: PlannedPathData = serde_json::from_str(&json2).expect("reparse");
        assert_eq!(reparsed.items.len(), 5);
    }

    #[test]
    fn fixture_valid_true_but_drive_display_false_with_reasons() {
        let data = build_mock_fixture_v1();
        assert!(data.valid);
        assert!(!data.safety.drive_allowed_display_only);
        assert!(!data.safety.reasons.is_empty());
    }

    #[test]
    fn fixture_has_lane_change_item() {
        let data = build_mock_fixture_v1();
        assert!(
            data.items
                .iter()
                .any(|i| i.kind == PlannedPathItemKind::LaneChange)
        );
    }

    #[test]
    fn fixture_has_junction_item() {
        let data = build_mock_fixture_v1();
        assert!(
            data.items
                .iter()
                .any(|i| i.kind == PlannedPathItemKind::Junction)
        );
    }

    #[test]
    fn fixture_curvature_populated() {
        let data = build_mock_fixture_v1();
        let curved: Vec<_> = data
            .items
            .iter()
            .filter_map(|i| i.curvature_1pm)
            .collect();
        assert!(curved.len() >= 3);
        assert!(curved.iter().any(|c| *c > 0.0));
    }

    #[test]
    fn debug_stats_counts_semaphores_and_junctions() {
        let data = build_mock_fixture_v1();
        let stats = planned_path_debug_stats(&data);
        assert_eq!(stats.item_count, 5);
        assert_eq!(stats.junction_prefab_count, 1);
        assert_eq!(stats.semaphore_hint_count, 2);
        assert!(stats.curvature_max_1pm.unwrap_or(0.0) > 0.0);
    }

    #[test]
    fn drive_allowed_display_only_is_not_used_for_control_gates() {
        // Document v1 contract: safety bool is display-only; no host reads it for engage.
        let mut data = build_mock_fixture_v1();
        data.safety.drive_allowed_display_only = true;
        assert!(data.valid);
        // Changing display flag must not imply path items changed.
        assert_eq!(data.items.len(), 5);
    }

    // ---- route blackboard polyline --------------------------------------

    #[test]
    fn route_blackboard_polyline_builds_road_edge_items() {
        let wps = [
            RouteBlackboardWaypoint {
                uid: 1,
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            RouteBlackboardWaypoint {
                uid: 2,
                x: 0.0,
                y: 0.0,
                z: 50.0,
            },
            RouteBlackboardWaypoint {
                uid: 3,
                x: 10.0,
                y: 0.0,
                z: 100.0,
            },
        ];
        let data = planned_path_from_route_blackboard(&wps, 0x42, None).expect("path");
        assert_eq!(data.source, PlannedPathSource::RouteBlackboard);
        assert_eq!(data.items.len(), 2);
        assert!(data.items.iter().all(|i| !i.points.is_empty()));
        assert!(!data.safety.drive_allowed_display_only);
    }

    // ---- offline-graph fixture ------------------------------------------

    #[test]
    fn offline_fixture_source_is_offline_graph_not_mock() {
        let data = build_offline_fixture_v1();
        assert_eq!(data.source, PlannedPathSource::OfflineGraph);
        assert_ne!(data.source, PlannedPathSource::Mock);
        assert!(data.valid);
        assert_eq!(data.route_id.as_deref(), Some("offline-graph-mini-v1"));
    }

    #[test]
    fn offline_fixture_covers_road_junction_lanechange_navcurve() {
        let data = build_offline_fixture_v1();
        let kinds: Vec<_> = data.items.iter().map(|i| i.kind).collect();
        // 2 road edges + junction + lane change + nav curve.
        assert_eq!(data.items.len(), 5);
        assert!(kinds.contains(&PlannedPathItemKind::RoadEdge));
        assert!(kinds.contains(&PlannedPathItemKind::Junction));
        assert!(kinds.contains(&PlannedPathItemKind::LaneChange));
        assert!(kinds.contains(&PlannedPathItemKind::NavCurve));
    }

    #[test]
    fn offline_fixture_carries_real_node_uids_and_prefab() {
        let data = build_offline_fixture_v1();
        // Node UIDs threaded from the graph fixture.
        assert_eq!(data.items[0].node_uid_start, Some(10_001));
        assert_eq!(data.items[4].node_uid_end, Some(10_006));
        // Junction item resolves its prefab uid from the graph.
        let junction = data
            .items
            .iter()
            .find(|i| i.kind == PlannedPathItemKind::Junction)
            .expect("junction item");
        assert_eq!(junction.prefab_uid, Some(9001));
    }

    #[test]
    fn offline_fixture_curvature_and_counts_visible() {
        let data = build_offline_fixture_v1();
        let stats = planned_path_debug_stats(&data);
        // Curvature populated and a curved segment present.
        assert!(stats.curvature_max_1pm.unwrap_or(0.0) > 0.0);
        // Junction/prefab and semaphore counts surfaced.
        assert_eq!(stats.junction_prefab_count, 1);
        assert_eq!(stats.semaphore_hint_count, 2);
    }

    #[test]
    fn offline_fixture_drive_display_false_when_gates_missing() {
        let data = build_offline_fixture_v1();
        assert!(!data.safety.drive_allowed_display_only);
        assert!(!data.safety.reasons.is_empty());
    }

    #[test]
    fn offline_fixture_json_roundtrip_keeps_source_and_curvature() {
        let data = build_offline_fixture_v1();
        let json = planned_path_to_json(&data);
        // Source is the snake_case offline_graph in JSON.
        assert!(json.contains("\"source\": \"offline_graph\""));
        let parsed: PlannedPathData = serde_json::from_str(&json).expect("roundtrip");
        assert_eq!(parsed.source, PlannedPathSource::OfflineGraph);
        assert_eq!(parsed.items.len(), data.items.len());
        assert!(parsed
            .items
            .iter()
            .filter_map(|i| i.curvature_1pm)
            .any(|c| c > 0.0));
    }
}
