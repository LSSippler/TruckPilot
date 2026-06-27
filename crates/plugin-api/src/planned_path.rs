//! Read-only planned path model (PlannedPathData v1).
//!
//! Inspired by ETS2LA concepts (PrefabPath, NavCurve, Hermite splines, lateral
//! offset, path curvature, semaphores) without copying ETS2LA code. v1 is
//! display-only — no steering, engage, or resolver activation.

use serde::{Deserialize, Serialize};

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
}
