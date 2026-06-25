//! ETS2 in-game route validation against a [`RouterGraph`].
//!
//! Phase 5a: conservative UID presence check — missing UIDs are skipped.
//! Phase 5g: repairable gaps between matched anchors are closed via `RouterGraph::plan`.

use crate::graph::RouterGraph;

/// Minimum matched nodes for an imported route to be considered usable.
pub const ETS2_ROUTE_MIN_MATCHED_NODES: usize = 2;
/// Minimum fraction of input UIDs that must exist in the graph.
pub const ETS2_ROUTE_MIN_MATCH_RATIO: f64 = 0.80;
/// Max distance from truck to a route segment for progress snap (metres).
pub const ETS2_ROUTE_SNAP_MAX_DIST_M: f64 = 80.0;
/// Max angle between truck heading and route segment direction (degrees).
pub const ETS2_ROUTE_SNAP_MAX_HEADING_DEG: f64 = 120.0;
/// Hard off-route distance — immediate ETS2 release (metres).
pub const ETS2_ROUTE_OFFROUTE_HARD_DIST_M: f64 = 120.0;
/// Sustained `too_far` snap before ETS2 release (seconds).
pub const ETS2_ROUTE_OFFROUTE_RELEASE_SECS: f64 = 3.0;
/// Minimum interval between ETS2 progress re-publishes (~5 Hz).
pub const ETS2_ROUTE_PROGRESS_MIN_INTERVAL_MS: u64 = 200;
/// Max missing UIDs in one gap for A* repair attempt.
pub const ETS2_ROUTE_REPAIR_MAX_MISSING_UIDS_PER_GAP: usize = 8;
/// Max nodes in a single A* repair path (inclusive of anchors).
pub const ETS2_ROUTE_REPAIR_MAX_PATH_NODES: usize = 80;
/// Max total intermediate nodes inserted across all gap repairs.
pub const ETS2_ROUTE_REPAIR_MAX_TOTAL_INSERTED_NODES: usize = 300;
/// Max missing UIDs in a failed gap that still allows partial import without repair.
pub const ETS2_ROUTE_REPAIR_MAX_PARTIAL_MISSING: usize = 2;
/// Max unrepaired gaps allowed under partial import policy.
pub const ETS2_ROUTE_REPAIR_MAX_PARTIAL_UNREPAIRED_GAPS: usize = 1;

/// High-level outcome of validating an ETS2 UID list against the routing graph.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ets2RouteMatchStatus {
    /// No SHM snapshot / reader not connected.
    Unavailable,
    /// Snapshot present but marked invalid or empty.
    Invalid,
    /// Every input UID matched in the graph and usability thresholds met.
    Matched,
    /// Some UIDs missing but usability thresholds still met.
    Partial,
    /// Not enough coverage to use the imported route.
    Failed,
}

impl Ets2RouteMatchStatus {
    /// Wire string for blackboard `navigation.ets2_route.match_status`.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Unavailable => "unavailable",
            Self::Invalid => "invalid",
            Self::Matched => "matched",
            Self::Partial => "partial",
            Self::Failed => "failed",
        }
    }
}

/// Cached ETS2 route metadata shared between Core and plugins (read-only for plugins).
#[derive(Debug, Clone, Default)]
pub struct Ets2RouteSnapshot {
    pub sequence: u32,
    pub route_hash: u64,
    pub valid: bool,
    pub uids: Vec<u64>,
}

/// Full waypoint metadata from RouteBlackboard SHM (Phase 5i).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Ets2RouteWaypoint {
    pub uid: u64,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub distance: f32,
    pub time: f32,
    pub flags: u32,
}

/// Mirrors telemetry `ROUTE_WP_FLAG_HAS_POSITION`.
pub const ETS2_WP_FLAG_HAS_POSITION: u32 = 1 << 0;
/// Mirrors telemetry `ROUTE_WP_FLAG_HAS_DISTANCE`.
pub const ETS2_WP_FLAG_HAS_DISTANCE: u32 = 1 << 1;
/// Mirrors telemetry `ROUTE_WP_FLAG_UNTRUSTED`.
pub const ETS2_WP_FLAG_UNTRUSTED: u32 = 1 << 3;

/// Relative tolerance for ETS2-vs-graph distance comparison (diagnostic).
pub const ETS2_GRAPH_DISTANCE_RATIO_TOLERANCE: f64 = 0.20;

/// Outcome of comparing ETS2 remaining-distance to graph route length (Phase 5j).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ets2GraphDistanceStatus {
    None,
    Ok,
    Mismatch,
    Partial,
    Untrusted,
}

impl Ets2GraphDistanceStatus {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::Ok => "ok",
            Self::Mismatch => "mismatch",
            Self::Partial => "partial",
            Self::Untrusted => "untrusted",
        }
    }
}

/// ETS2 remaining-distance vs graph geometry (diagnostic only).
#[derive(Debug, Clone, PartialEq)]
pub struct Ets2GraphDistanceCompare {
    pub graph_total_m: f64,
    pub first_vs_graph_delta_m: f64,
    pub graph_ratio: f64,
    pub status: Ets2GraphDistanceStatus,
    pub segment_compare_count: usize,
    pub segment_mismatch_count: usize,
}

impl Default for Ets2GraphDistanceCompare {
    fn default() -> Self {
        Self {
            graph_total_m: 0.0,
            first_vs_graph_delta_m: 0.0,
            graph_ratio: 0.0,
            status: Ets2GraphDistanceStatus::None,
            segment_compare_count: 0,
            segment_mismatch_count: 0,
        }
    }
}

/// Sum of straight-line segment lengths between consecutive graph nodes on the route.
pub fn graph_route_length_m(graph: &RouterGraph, route_node_ids: &[u64]) -> f64 {
    if route_node_ids.len() < 2 {
        return 0.0;
    }
    route_node_ids
        .windows(2)
        .map(|w| {
            match (graph.node_position(w[0]), graph.node_position(w[1])) {
                (Some((ax, az)), Some((bx, bz))) => {
                    let dx = bx - ax;
                    let dz = bz - az;
                    (dx * dx + dz * dz).sqrt()
                }
                _ => 0.0,
            }
        })
        .sum()
}

/// Compare ETS2 distance fields against graph route geometry (no import impact).
pub fn compare_ets2_graph_distance(
    graph: &RouterGraph,
    route_node_ids: &[u64],
    waypoints: &[Ets2RouteWaypoint],
) -> Ets2GraphDistanceCompare {
    let dist_samples: Vec<(u64, f32, u32)> = waypoints
        .iter()
        .filter(|wp| wp.flags & ETS2_WP_FLAG_HAS_DISTANCE != 0)
        .map(|wp| (wp.uid, wp.distance, wp.flags))
        .collect();

    if dist_samples.is_empty() || route_node_ids.len() < 2 {
        return Ets2GraphDistanceCompare {
            status: Ets2GraphDistanceStatus::None,
            ..Default::default()
        };
    }

    let any_untrusted = dist_samples
        .iter()
        .any(|(_, _, flags)| flags & ETS2_WP_FLAG_UNTRUSTED != 0);
    let coverage_partial =
        dist_samples.len() < waypoints.len() || dist_samples.len() < route_node_ids.len();

    let graph_total_m = graph_route_length_m(graph, route_node_ids);
    let ets2_first_m = f64::from(dist_samples[0].1);
    let first_vs_graph_delta_m = (ets2_first_m - graph_total_m).abs();
    let graph_ratio = if graph_total_m > 1.0 {
        ets2_first_m / graph_total_m
    } else {
        0.0
    };

    let uid_dist: std::collections::HashMap<u64, f32> = dist_samples
        .iter()
        .map(|(uid, dist, _)| (*uid, *dist))
        .collect();

    let seg_tol = 0.5_f64;
    let mut segment_compare_count = 0usize;
    let mut segment_mismatch_count = 0usize;

    for w in route_node_ids.windows(2) {
        let Some(d0) = uid_dist.get(&w[0]) else {
            continue;
        };
        let Some(d1) = uid_dist.get(&w[1]) else {
            continue;
        };
        let graph_seg = match (graph.node_position(w[0]), graph.node_position(w[1])) {
            (Some((ax, az)), Some((bx, bz))) => {
                let dx = bx - ax;
                let dz = bz - az;
                (dx * dx + dz * dz).sqrt()
            }
            _ => continue,
        };
        let ets2_step = f64::from(d0 - d1);
        if ets2_step < -seg_tol {
            continue;
        }
        segment_compare_count += 1;
        let delta = (ets2_step - graph_seg).abs();
        let rel = if graph_seg > 1.0 {
            delta / graph_seg
        } else {
            delta
        };
        if delta > seg_tol && rel > ETS2_GRAPH_DISTANCE_RATIO_TOLERANCE {
            segment_mismatch_count += 1;
        }
    }

    let ratio_ok = graph_total_m > 1.0
        && (graph_ratio - 1.0).abs() <= ETS2_GRAPH_DISTANCE_RATIO_TOLERANCE;
    let delta_ok = graph_total_m <= 1.0
        || first_vs_graph_delta_m / graph_total_m <= ETS2_GRAPH_DISTANCE_RATIO_TOLERANCE;
    let segments_ok = segment_compare_count == 0
        || segment_mismatch_count * 2 <= segment_compare_count;

    let status = if any_untrusted {
        Ets2GraphDistanceStatus::Untrusted
    } else if coverage_partial || graph_total_m <= 0.0 {
        Ets2GraphDistanceStatus::Partial
    } else if ratio_ok && delta_ok && segments_ok {
        Ets2GraphDistanceStatus::Ok
    } else {
        Ets2GraphDistanceStatus::Mismatch
    };

    Ets2GraphDistanceCompare {
        graph_total_m,
        first_vs_graph_delta_m,
        graph_ratio,
        status,
        segment_compare_count,
        segment_mismatch_count,
    }
}

/// Graph vs ETS2 horizontal position delta summary (diagnostic).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Ets2GraphCoordDelta {
    pub avg_m: f64,
    pub max_m: f64,
    pub count: usize,
}

/// Compare ETS2 waypoint x/z (when flagged) against graph node positions for matched UIDs.
pub fn compare_ets2_graph_coord_delta(
    graph: &RouterGraph,
    waypoints: &[Ets2RouteWaypoint],
) -> Ets2GraphCoordDelta {
    let mut sum = 0.0_f64;
    let mut max = 0.0_f64;
    let mut count = 0usize;
    for wp in waypoints {
        if wp.flags & ETS2_WP_FLAG_HAS_POSITION == 0 {
            continue;
        }
        let Some((gx, gz)) = graph.node_position(wp.uid) else {
            continue;
        };
        let dx = f64::from(wp.x) - gx;
        let dz = f64::from(wp.z) - gz;
        let d = (dx * dx + dz * dz).sqrt();
        sum += d;
        max = max.max(d);
        count += 1;
    }
    Ets2GraphCoordDelta {
        avg_m: if count > 0 { sum / count as f64 } else { 0.0 },
        max_m: max,
        count,
    }
}

/// Result of matching ETS2 route UIDs to graph node IDs.
#[derive(Debug, Clone, PartialEq)]
pub struct Ets2RouteMatchResult {
    pub status: Ets2RouteMatchStatus,
    /// Graph node UIDs that matched, in travel order (gaps omitted).
    pub route_node_ids: Vec<u64>,
    pub matched_count: usize,
    pub missing_count: usize,
    pub first_missing_uid: Option<u64>,
    pub match_ratio: f64,
    pub is_usable: bool,
    pub import_error: Option<String>,
}

impl Default for Ets2RouteMatchResult {
    fn default() -> Self {
        Self {
            status: Ets2RouteMatchStatus::Unavailable,
            route_node_ids: Vec::new(),
            matched_count: 0,
            missing_count: 0,
            first_missing_uid: None,
            match_ratio: 0.0,
            is_usable: false,
            import_error: None,
        }
    }
}

/// Shared read-only cache updated by Core when the ETS2 route changes.
#[derive(Debug, Clone, Default)]
pub struct Ets2RouteSharedState {
    pub snapshot: Option<Ets2RouteSnapshot>,
    pub match_result: Option<Ets2RouteMatchResult>,
    /// Full SHM waypoint rows (UID + optional ETS2 coords/dist/time).
    pub waypoints: Vec<Ets2RouteWaypoint>,
}

/// Validate ETS2 route UIDs against `graph`. Missing UIDs are skipped; order preserved.
pub fn match_ets2_route_uids(graph: &RouterGraph, uids: &[u64]) -> Ets2RouteMatchResult {
    if uids.is_empty() {
        return Ets2RouteMatchResult {
            status: Ets2RouteMatchStatus::Invalid,
            import_error: Some("empty route".into()),
            ..Default::default()
        };
    }

    let mut route_node_ids = Vec::with_capacity(uids.len());
    let mut missing_count = 0usize;
    let mut first_missing_uid = None;

    for &uid in uids {
        if graph.has_node(uid) {
            route_node_ids.push(uid);
        } else {
            missing_count += 1;
            if first_missing_uid.is_none() {
                first_missing_uid = Some(uid);
            }
        }
    }

    let total = uids.len();
    let matched_count = route_node_ids.len();
    let match_ratio = matched_count as f64 / total as f64;
    let is_usable = matched_count >= ETS2_ROUTE_MIN_MATCHED_NODES
        && match_ratio >= ETS2_ROUTE_MIN_MATCH_RATIO
        && !route_node_ids.is_empty();

    let status = if missing_count == 0 && is_usable {
        Ets2RouteMatchStatus::Matched
    } else if is_usable {
        Ets2RouteMatchStatus::Partial
    } else {
        Ets2RouteMatchStatus::Failed
    };

    let import_error = if is_usable {
        None
    } else if matched_count < ETS2_ROUTE_MIN_MATCHED_NODES {
        Some(format!(
            "only {matched_count} matched nodes (need >= {ETS2_ROUTE_MIN_MATCHED_NODES})"
        ))
    } else if match_ratio < ETS2_ROUTE_MIN_MATCH_RATIO {
        Some(format!(
            "match ratio {:.1}% below {:.0}% threshold",
            match_ratio * 100.0,
            ETS2_ROUTE_MIN_MATCH_RATIO * 100.0
        ))
    } else {
        Some("route not usable".into())
    };

    Ets2RouteMatchResult {
        status,
        route_node_ids,
        matched_count,
        missing_count,
        first_missing_uid,
        match_ratio,
        is_usable,
        import_error,
    }
}

/// Match an invalid (empty) snapshot without touching the graph.
pub fn match_invalid_ets2_route(reason: &str) -> Ets2RouteMatchResult {
    Ets2RouteMatchResult {
        status: Ets2RouteMatchStatus::Invalid,
        import_error: Some(reason.into()),
        ..Default::default()
    }
}

/// Match result when no ETS2 route SHM is available.
pub fn match_unavailable_ets2_route() -> Ets2RouteMatchResult {
    Ets2RouteMatchResult {
        status: Ets2RouteMatchStatus::Unavailable,
        import_error: Some("ets2 route shm unavailable".into()),
        ..Default::default()
    }
}

/// A gap between two matched ETS2 anchors with missing UIDs in between.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Ets2RouteGap {
    pub from_uid: u64,
    pub to_uid: u64,
    pub missing_count: usize,
    pub first_missing_uid: Option<u64>,
}

/// Outcome of A* gap repair for blackboard `navigation.ets2_route.repair_status`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ets2RouteRepairStatus {
    None,
    NotNeeded,
    Repaired,
    PartialUnrepaired,
    Failed,
    Disabled,
}

impl Ets2RouteRepairStatus {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::NotNeeded => "not_needed",
            Self::Repaired => "repaired",
            Self::PartialUnrepaired => "partial_unrepaired",
            Self::Failed => "failed",
            Self::Disabled => "disabled",
        }
    }
}

/// Result of attempting to close gaps in a matched ETS2 route via A*.
#[derive(Debug, Clone, PartialEq)]
pub struct Ets2RouteRepairResult {
    pub status: Ets2RouteRepairStatus,
    pub route_node_ids: Vec<u64>,
    pub gap_count: usize,
    pub success_count: usize,
    pub failed_count: usize,
    pub inserted_node_count: usize,
    pub first_failed_gap: Option<String>,
    pub error: Option<String>,
    pub import_allowed: bool,
}

impl Default for Ets2RouteRepairResult {
    fn default() -> Self {
        Self {
            status: Ets2RouteRepairStatus::None,
            route_node_ids: Vec::new(),
            gap_count: 0,
            success_count: 0,
            failed_count: 0,
            inserted_node_count: 0,
            first_failed_gap: None,
            error: None,
            import_allowed: false,
        }
    }
}

/// Detect repairable gaps: missing UID runs between two graph-matched anchors.
pub fn detect_ets2_route_gaps(uids: &[u64], graph: &RouterGraph) -> Vec<Ets2RouteGap> {
    let mut gaps = Vec::new();
    let mut last_matched: Option<u64> = None;
    let mut missing_count = 0usize;
    let mut first_missing_uid = None;

    for &uid in uids {
        if graph.has_node(uid) {
            if missing_count > 0 {
                if let Some(from_uid) = last_matched {
                    gaps.push(Ets2RouteGap {
                        from_uid,
                        to_uid: uid,
                        missing_count,
                        first_missing_uid,
                    });
                }
                missing_count = 0;
                first_missing_uid = None;
            }
            last_matched = Some(uid);
        } else {
            missing_count += 1;
            if first_missing_uid.is_none() {
                first_missing_uid = Some(uid);
            }
        }
    }

    gaps
}

fn gap_label(from_uid: u64, to_uid: u64) -> String {
    format!("{from_uid}->{to_uid}")
}

/// Close repairable gaps in a matched ETS2 route using `RouterGraph::plan` (A*).
///
/// Policy (conservative):
/// - No gaps → `not_needed`, original matched route.
/// - All gaps repaired within limits → `repaired`.
/// - One small failed gap (≤2 missing) → `partial_unrepaired`, original matched route.
/// - Gap too large, multiple failures, no A* path, or insert budget exceeded → `failed`, import rejected.
pub fn repair_ets2_route_gaps(
    graph: &RouterGraph,
    uids: &[u64],
    matched_route_node_ids: &[u64],
) -> Ets2RouteRepairResult {
    if matched_route_node_ids.len() < ETS2_ROUTE_MIN_MATCHED_NODES {
        return Ets2RouteRepairResult {
            status: Ets2RouteRepairStatus::Failed,
            route_node_ids: matched_route_node_ids.to_vec(),
            error: Some("matched route too short for repair".into()),
            ..Default::default()
        };
    }

    let gaps = detect_ets2_route_gaps(uids, graph);
    if gaps.is_empty() {
        return Ets2RouteRepairResult {
            status: Ets2RouteRepairStatus::NotNeeded,
            route_node_ids: matched_route_node_ids.to_vec(),
            import_allowed: true,
            ..Default::default()
        };
    }

    let gap_count = gaps.len();
    let mut repaired_route = Vec::with_capacity(matched_route_node_ids.len() + gap_count * 4);
    repaired_route.push(matched_route_node_ids[0]);

    let mut success_count = 0usize;
    let mut failed_count = 0usize;
    let mut inserted_node_count = 0usize;
    let mut first_failed_gap = None;
    let mut errors = Vec::new();
    let mut oversize_gap = false;
    let mut failed_missing_counts = Vec::new();

    for window in matched_route_node_ids.windows(2) {
        let from_uid = window[0];
        let to_uid = window[1];
        let Some(gap) = gaps
            .iter()
            .find(|g| g.from_uid == from_uid && g.to_uid == to_uid)
        else {
            repaired_route.push(to_uid);
            continue;
        };

        if gap.missing_count > ETS2_ROUTE_REPAIR_MAX_MISSING_UIDS_PER_GAP {
            oversize_gap = true;
            failed_count += 1;
            failed_missing_counts.push(gap.missing_count);
            first_failed_gap.get_or_insert_with(|| gap_label(from_uid, to_uid));
            errors.push(format!(
                "gap {from_uid}->{to_uid} missing={} exceeds max {}",
                gap.missing_count, ETS2_ROUTE_REPAIR_MAX_MISSING_UIDS_PER_GAP
            ));
            continue;
        }

        match graph.plan(from_uid, to_uid) {
            None => {
                failed_count += 1;
                failed_missing_counts.push(gap.missing_count);
                first_failed_gap.get_or_insert_with(|| gap_label(from_uid, to_uid));
                errors.push(format!("no A* path {from_uid}->{to_uid}"));
            }
            Some((path, _dist)) if path.len() < 2 => {
                failed_count += 1;
                failed_missing_counts.push(gap.missing_count);
                first_failed_gap.get_or_insert_with(|| gap_label(from_uid, to_uid));
                errors.push(format!("degenerate A* path {from_uid}->{to_uid}"));
            }
            Some((path, _dist)) if path.len() > ETS2_ROUTE_REPAIR_MAX_PATH_NODES => {
                failed_count += 1;
                failed_missing_counts.push(gap.missing_count);
                first_failed_gap.get_or_insert_with(|| gap_label(from_uid, to_uid));
                errors.push(format!(
                    "A* path {from_uid}->{to_uid} too long: {} nodes",
                    path.len()
                ));
            }
            Some((path, _dist)) => {
                let insert_count = path.len().saturating_sub(2);
                if inserted_node_count + insert_count > ETS2_ROUTE_REPAIR_MAX_TOTAL_INSERTED_NODES
                {
                    failed_count += 1;
                    failed_missing_counts.push(gap.missing_count);
                    first_failed_gap.get_or_insert_with(|| gap_label(from_uid, to_uid));
                    errors.push(format!(
                        "insert budget exceeded at {from_uid}->{to_uid} (+{insert_count})"
                    ));
                    continue;
                }
                inserted_node_count += insert_count;
                repaired_route.extend_from_slice(&path[1..]);
                success_count += 1;
            }
        }
    }

    let error = if errors.is_empty() {
        None
    } else {
        Some(errors.join("; "))
    };

    if oversize_gap {
        return Ets2RouteRepairResult {
            status: Ets2RouteRepairStatus::Failed,
            route_node_ids: matched_route_node_ids.to_vec(),
            gap_count,
            success_count,
            failed_count,
            inserted_node_count,
            first_failed_gap,
            error,
            import_allowed: false,
        };
    }

    if failed_count == 0 && success_count > 0 {
        return Ets2RouteRepairResult {
            status: Ets2RouteRepairStatus::Repaired,
            route_node_ids: repaired_route,
            gap_count,
            success_count,
            failed_count,
            inserted_node_count,
            first_failed_gap: None,
            error: None,
            import_allowed: true,
        };
    }

    if success_count > 0 && failed_count > 0 {
        return Ets2RouteRepairResult {
            status: Ets2RouteRepairStatus::Failed,
            route_node_ids: matched_route_node_ids.to_vec(),
            gap_count,
            success_count,
            failed_count,
            inserted_node_count,
            first_failed_gap,
            error: Some(format!(
                "mixed repair not allowed ({success_count} ok, {failed_count} failed)"
            )),
            import_allowed: false,
        };
    }

    if failed_count == 0 && success_count == 0 {
        return Ets2RouteRepairResult {
            status: Ets2RouteRepairStatus::Failed,
            route_node_ids: matched_route_node_ids.to_vec(),
            gap_count,
            success_count,
            failed_count,
            inserted_node_count,
            first_failed_gap,
            error: Some("gaps present but none repaired".into()),
            import_allowed: false,
        };
    }

    let partial_allowed = failed_count <= ETS2_ROUTE_REPAIR_MAX_PARTIAL_UNREPAIRED_GAPS
        && failed_missing_counts
            .iter()
            .all(|&c| c <= ETS2_ROUTE_REPAIR_MAX_PARTIAL_MISSING);

    if partial_allowed {
        return Ets2RouteRepairResult {
            status: Ets2RouteRepairStatus::PartialUnrepaired,
            route_node_ids: matched_route_node_ids.to_vec(),
            gap_count,
            success_count,
            failed_count,
            inserted_node_count,
            first_failed_gap,
            error,
            import_allowed: true,
        };
    }

    Ets2RouteRepairResult {
        status: Ets2RouteRepairStatus::Failed,
        route_node_ids: matched_route_node_ids.to_vec(),
        gap_count,
        success_count,
        failed_count,
        inserted_node_count,
        first_failed_gap,
        error,
        import_allowed: false,
    }
}

/// Router-ready output built from matched ETS2 node UIDs.
#[derive(Debug, Clone, PartialEq)]
pub struct Ets2RouterOutput {
    pub waypoints: Vec<[f64; 2]>,
    pub route_node_ids: Vec<u64>,
    pub distance_m: f64,
}

/// Build `router.waypoints` / `router.route_node_ids` payload from matched node UIDs.
///
/// Returns an error when positions are missing, the route is too short, or degenerate.
pub fn build_router_output_from_node_ids(
    graph: &RouterGraph,
    route_node_ids: &[u64],
) -> Result<Ets2RouterOutput, String> {
    if route_node_ids.len() < ETS2_ROUTE_MIN_MATCHED_NODES {
        return Err(format!(
            "need at least {} route nodes, got {}",
            ETS2_ROUTE_MIN_MATCHED_NODES,
            route_node_ids.len()
        ));
    }

    let mut waypoints = Vec::with_capacity(route_node_ids.len());
    for &uid in route_node_ids {
        let (x, z) = graph
            .node_position(uid)
            .ok_or_else(|| format!("node {uid} has no graph position"))?;
        waypoints.push([x, z]);
    }

    if waypoints.len() < ETS2_ROUTE_MIN_MATCHED_NODES {
        return Err("too few waypoints after position lookup".into());
    }

    if is_degenerate_waypoints(&waypoints) {
        return Err("degenerate route: all waypoints coincide".into());
    }

    let distance_m = path_length_m(&waypoints);
    if distance_m <= 0.0 {
        return Err("degenerate route: zero path length".into());
    }

    Ok(Ets2RouterOutput {
        waypoints,
        route_node_ids: route_node_ids.to_vec(),
        distance_m,
    })
}

/// Outcome of snapping truck progress onto an imported ETS2 route.
#[derive(Debug, Clone, PartialEq)]
pub struct Ets2RouteTrimResult {
    pub start_index: usize,
    pub snap_dist_m: f64,
    pub snap_status: Ets2RouteSnapStatus,
    pub snap_heading_delta_deg: Option<f64>,
    pub original_node_count: usize,
    pub trimmed_node_count: usize,
    pub trimmed: bool,
}

/// Snap/trim diagnostic status for blackboard `navigation.ets2_route.snap_status`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ets2RouteSnapStatus {
    Ok,
    TooFar,
    NoPosition,
    TooShort,
    FallbackZero,
}

impl Ets2RouteSnapStatus {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Ok => "ok",
            Self::TooFar => "too_far",
            Self::NoPosition => "no_position",
            Self::TooShort => "too_short",
            Self::FallbackZero => "fallback_zero",
        }
    }
}

/// Find the route node index where the trimmed route should begin for the truck.
///
/// `heading` uses ETS2 convention: `[0..1]` CCW from North (same as `telemetry.heading`).
pub fn find_route_start_index_for_truck(
    graph: &RouterGraph,
    route_node_ids: &[u64],
    truck_x: f64,
    truck_z: f64,
    heading: Option<f64>,
) -> Ets2RouteTrimResult {
    let original_node_count = route_node_ids.len();
    let max_start = original_node_count.saturating_sub(ETS2_ROUTE_MIN_MATCHED_NODES);

    if original_node_count < ETS2_ROUTE_MIN_MATCHED_NODES {
        return Ets2RouteTrimResult {
            start_index: 0,
            snap_dist_m: f64::INFINITY,
            snap_status: Ets2RouteSnapStatus::TooShort,
            snap_heading_delta_deg: None,
            original_node_count,
            trimmed_node_count: original_node_count,
            trimmed: false,
        };
    }

    let min_heading_dot = (ETS2_ROUTE_SNAP_MAX_HEADING_DEG.to_radians()).cos();

    let mut best = scan_route_segments_for_truck(
        graph,
        route_node_ids,
        truck_x,
        truck_z,
        heading,
        min_heading_dot,
        true,
    );

    if heading.is_some() && !best.found {
        best = scan_route_segments_for_truck(
            graph,
            route_node_ids,
            truck_x,
            truck_z,
            heading,
            min_heading_dot,
            false,
        );
    }

    let (snap_status, start_index) = if !best.found {
        (Ets2RouteSnapStatus::FallbackZero, 0usize)
    } else if best.dist_m > ETS2_ROUTE_SNAP_MAX_DIST_M {
        (Ets2RouteSnapStatus::TooFar, 0usize)
    } else {
        (Ets2RouteSnapStatus::Ok, best.start_index.min(max_start))
    };

    let trimmed = start_index > 0;
    let trimmed_node_count = original_node_count.saturating_sub(start_index);

    Ets2RouteTrimResult {
        start_index,
        snap_dist_m: if best.found { best.dist_m } else { f64::INFINITY },
        snap_status,
        snap_heading_delta_deg: best.heading_delta_deg,
        original_node_count,
        trimmed_node_count,
        trimmed,
    }
}

struct RouteSegmentSnap {
    found: bool,
    start_index: usize,
    dist_m: f64,
    heading_delta_deg: Option<f64>,
}

fn scan_route_segments_for_truck(
    graph: &RouterGraph,
    route_node_ids: &[u64],
    truck_x: f64,
    truck_z: f64,
    heading: Option<f64>,
    min_heading_dot: f64,
    apply_heading: bool,
) -> RouteSegmentSnap {
    let mut best_dist = f64::INFINITY;
    let mut best_start = 0usize;
    let mut best_heading_delta_deg = None;
    let mut found = false;

    for i in 0..route_node_ids.len().saturating_sub(1) {
        let Some((ax, az)) = graph.node_position(route_node_ids[i]) else {
            continue;
        };
        let Some((bx, bz)) = graph.node_position(route_node_ids[i + 1]) else {
            continue;
        };
        let (t, dist) = project_point_on_segment(truck_x, truck_z, ax, az, bx, bz);
        let mut heading_delta_deg = None;

        if apply_heading {
            if let Some(h) = heading {
                let (hx, hz) = truck_forward_vector(h);
                let abx = bx - ax;
                let abz = bz - az;
                let seg_len = (abx * abx + abz * abz).sqrt();
                if seg_len >= 1e-9 {
                    let sx = abx / seg_len;
                    let sz = abz / seg_len;
                    let dot = (hx * sx + hz * sz).clamp(-1.0, 1.0);
                    heading_delta_deg = Some(dot.acos().to_degrees());
                    if dot < min_heading_dot {
                        continue;
                    }
                }
            }
        }

        if dist < best_dist {
            best_dist = dist;
            best_start = if t > 0.5 { i + 1 } else { i };
            best_heading_delta_deg = heading_delta_deg;
            found = true;
        }
    }

    RouteSegmentSnap {
        found,
        start_index: best_start,
        dist_m: best_dist,
        heading_delta_deg: best_heading_delta_deg,
    }
}

/// Trim an ETS2 route to truck progress and build router output.
///
/// When truck position is unavailable, imports the full route with `snap_status=no_position`.
pub fn build_trimmed_ets2_router_output(
    graph: &RouterGraph,
    route_node_ids: &[u64],
    truck_x: Option<f64>,
    truck_z: Option<f64>,
    heading: Option<f64>,
) -> Result<(Ets2RouterOutput, Ets2RouteTrimResult), (Ets2RouteTrimResult, String)> {
    let original_node_count = route_node_ids.len();

    let trim = match (truck_x, truck_z) {
        (Some(x), Some(z)) => find_route_start_index_for_truck(graph, route_node_ids, x, z, heading),
        _ => Ets2RouteTrimResult {
            start_index: 0,
            snap_dist_m: f64::INFINITY,
            snap_status: Ets2RouteSnapStatus::NoPosition,
            snap_heading_delta_deg: None,
            original_node_count,
            trimmed_node_count: original_node_count,
            trimmed: false,
        },
    };

    if trim.trimmed_node_count < ETS2_ROUTE_MIN_MATCHED_NODES {
        return Err((
            Ets2RouteTrimResult {
                snap_status: Ets2RouteSnapStatus::TooShort,
                ..trim
            },
            format!(
                "trim leaves {} nodes (need >= {})",
                trim.trimmed_node_count, ETS2_ROUTE_MIN_MATCHED_NODES
            ),
        ));
    }

    let trimmed_ids = &route_node_ids[trim.start_index..];
    match build_router_output_from_node_ids(graph, trimmed_ids) {
        Ok(output) => Ok((output, trim)),
        Err(e) => Err((trim, e)),
    }
}

/// Live progress status for blackboard `navigation.ets2_route.progress_status`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ets2RouteProgressStatus {
    Ok,
    Unchanged,
    Advanced,
    RegressionIgnored,
    SnapBad,
    Released,
}

impl Ets2RouteProgressStatus {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Ok => "ok",
            Self::Unchanged => "unchanged",
            Self::Advanced => "advanced",
            Self::RegressionIgnored => "regression_ignored",
            Self::SnapBad => "snap_bad",
            Self::Released => "released",
        }
    }
}

/// Outcome of evaluating whether active ETS2 progress should advance or release.
#[derive(Debug, Clone, PartialEq)]
pub struct Ets2RouteProgressDecision {
    pub should_republish: bool,
    pub should_release: bool,
    pub release_reason: Option<&'static str>,
    pub effective_start_index: usize,
    pub progress_status: Ets2RouteProgressStatus,
    /// When true, caller should start/continue off-route accumulation timer.
    pub counts_as_offroute: bool,
}

/// Whether snap quality blocks progress advancement (keeps last published route).
pub fn snap_blocks_progress(status: Ets2RouteSnapStatus) -> bool {
    matches!(
        status,
        Ets2RouteSnapStatus::TooFar
            | Ets2RouteSnapStatus::NoPosition
            | Ets2RouteSnapStatus::FallbackZero
    )
}

/// Decide live ETS2 progress along a full imported route.
pub fn decide_ets2_route_progress(
    computed_start_index: usize,
    last_published_start_index: usize,
    full_route_len: usize,
    trim: &Ets2RouteTrimResult,
    offroute_secs: f64,
    now_ms: u64,
    last_republish_at_ms: u64,
) -> Ets2RouteProgressDecision {
    let max_start = full_route_len.saturating_sub(ETS2_ROUTE_MIN_MATCHED_NODES);
    let clamped = computed_start_index.min(max_start);
    let remaining = full_route_len.saturating_sub(clamped);

    if remaining < ETS2_ROUTE_MIN_MATCHED_NODES {
        return Ets2RouteProgressDecision {
            should_republish: false,
            should_release: true,
            release_reason: Some("too_short"),
            effective_start_index: last_published_start_index,
            progress_status: Ets2RouteProgressStatus::Released,
            counts_as_offroute: false,
        };
    }

    if trim.snap_dist_m > ETS2_ROUTE_OFFROUTE_HARD_DIST_M
        && trim.snap_status == Ets2RouteSnapStatus::TooFar
    {
        return Ets2RouteProgressDecision {
            should_republish: false,
            should_release: true,
            release_reason: Some("ets2_off_route"),
            effective_start_index: last_published_start_index,
            progress_status: Ets2RouteProgressStatus::Released,
            counts_as_offroute: true,
        };
    }

    if trim.snap_status == Ets2RouteSnapStatus::TooFar
        && offroute_secs >= ETS2_ROUTE_OFFROUTE_RELEASE_SECS
    {
        return Ets2RouteProgressDecision {
            should_republish: false,
            should_release: true,
            release_reason: Some("ets2_off_route"),
            effective_start_index: last_published_start_index,
            progress_status: Ets2RouteProgressStatus::Released,
            counts_as_offroute: true,
        };
    }

    if snap_blocks_progress(trim.snap_status) {
        return Ets2RouteProgressDecision {
            should_republish: false,
            should_release: false,
            release_reason: None,
            effective_start_index: last_published_start_index,
            progress_status: Ets2RouteProgressStatus::SnapBad,
            counts_as_offroute: trim.snap_status == Ets2RouteSnapStatus::TooFar,
        };
    }

    if clamped < last_published_start_index {
        return Ets2RouteProgressDecision {
            should_republish: false,
            should_release: false,
            release_reason: None,
            effective_start_index: last_published_start_index,
            progress_status: Ets2RouteProgressStatus::RegressionIgnored,
            counts_as_offroute: false,
        };
    }

    if clamped > last_published_start_index {
        let rate_ok =
            now_ms.saturating_sub(last_republish_at_ms) >= ETS2_ROUTE_PROGRESS_MIN_INTERVAL_MS;
        return Ets2RouteProgressDecision {
            should_republish: rate_ok,
            should_release: false,
            release_reason: None,
            effective_start_index: clamped,
            progress_status: if rate_ok {
                Ets2RouteProgressStatus::Advanced
            } else {
                Ets2RouteProgressStatus::Unchanged
            },
            counts_as_offroute: false,
        };
    }

    Ets2RouteProgressDecision {
        should_republish: false,
        should_release: false,
        release_reason: None,
        effective_start_index: last_published_start_index,
        progress_status: Ets2RouteProgressStatus::Unchanged,
        counts_as_offroute: false,
    }
}

/// Build trim metadata for a chosen start index on the full imported route.
pub fn trim_result_for_start_index(
    full_route_node_ids: &[u64],
    start_index: usize,
    snap_dist_m: f64,
    snap_status: Ets2RouteSnapStatus,
    snap_heading_delta_deg: Option<f64>,
) -> Ets2RouteTrimResult {
    let original_node_count = full_route_node_ids.len();
    let trimmed_node_count = original_node_count.saturating_sub(start_index);
    Ets2RouteTrimResult {
        start_index,
        snap_dist_m,
        snap_status,
        snap_heading_delta_deg,
        original_node_count,
        trimmed_node_count,
        trimmed: start_index > 0,
    }
}

fn truck_forward_vector(heading: f64) -> (f64, f64) {
    let heading_rad = -heading * std::f64::consts::TAU;
    (heading_rad.sin(), -heading_rad.cos())
}

fn project_point_on_segment(
    px: f64,
    pz: f64,
    ax: f64,
    az: f64,
    bx: f64,
    bz: f64,
) -> (f64, f64) {
    let abx = bx - ax;
    let abz = bz - az;
    let len_sq = abx * abx + abz * abz;
    if len_sq < 1e-18 {
        let dx = px - ax;
        let dz = pz - az;
        return (0.0, (dx * dx + dz * dz).sqrt());
    }
    let t_raw = ((px - ax) * abx + (pz - az) * abz) / len_sq;
    let t = t_raw.clamp(0.0, 1.0);
    let proj_x = ax + t * abx;
    let proj_z = az + t * abz;
    let dx = px - proj_x;
    let dz = pz - proj_z;
    (t, (dx * dx + dz * dz).sqrt())
}

fn path_length_m(waypoints: &[[f64; 2]]) -> f64 {
    waypoints
        .windows(2)
        .map(|w| {
            let dx = w[1][0] - w[0][0];
            let dz = w[1][1] - w[0][1];
            (dx * dx + dz * dz).sqrt()
        })
        .sum()
}

fn is_degenerate_waypoints(waypoints: &[[f64; 2]]) -> bool {
    if waypoints.len() < 2 {
        return true;
    }
    let [x0, z0] = waypoints[0];
    waypoints
        .iter()
        .all(|&[x, z]| (x - x0).abs() < 1e-6 && (z - z0).abs() < 1e-6)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::RouterGraph;

    fn graph_with_nodes(uids: &[u64]) -> RouterGraph {
        let nodes: Vec<(u64, f64, f64)> = uids
            .iter()
            .enumerate()
            .map(|(i, &uid)| (uid, i as f64 * 10.0, i as f64))
            .collect();
        let edges: Vec<(u64, u64, f64)> = uids
            .windows(2)
            .map(|w| (w[0], w[1], 10.0))
            .collect();
        RouterGraph::new(nodes, edges)
    }

    #[test]
    fn all_uids_match() {
        let graph = graph_with_nodes(&[1, 2, 3, 4]);
        let result = match_ets2_route_uids(&graph, &[1, 2, 3, 4]);
        assert_eq!(result.status, Ets2RouteMatchStatus::Matched);
        assert_eq!(result.route_node_ids, vec![1, 2, 3, 4]);
        assert_eq!(result.matched_count, 4);
        assert_eq!(result.missing_count, 0);
        assert!(result.first_missing_uid.is_none());
        assert!((result.match_ratio - 1.0).abs() < f64::EPSILON);
        assert!(result.is_usable);
    }

    #[test]
    fn partial_missing_but_usable() {
        let graph = graph_with_nodes(&[1, 2, 3, 4, 5]);
        // 4/5 = 80% exactly at threshold
        let result = match_ets2_route_uids(&graph, &[1, 2, 99, 4, 5]);
        assert_eq!(result.status, Ets2RouteMatchStatus::Partial);
        assert_eq!(result.route_node_ids, vec![1, 2, 4, 5]);
        assert_eq!(result.matched_count, 4);
        assert_eq!(result.missing_count, 1);
        assert_eq!(result.first_missing_uid, Some(99));
        assert!((result.match_ratio - 0.8).abs() < f64::EPSILON);
        assert!(result.is_usable);
    }

    #[test]
    fn partial_missing_unusable_below_ratio() {
        let graph = graph_with_nodes(&[1, 2, 3]);
        let result = match_ets2_route_uids(&graph, &[1, 99, 3]);
        assert_eq!(result.status, Ets2RouteMatchStatus::Failed);
        assert_eq!(result.matched_count, 2);
        assert_eq!(result.missing_count, 1);
        assert!(!result.is_usable);
    }

    #[test]
    fn empty_route_is_invalid() {
        let graph = graph_with_nodes(&[1, 2]);
        let result = match_ets2_route_uids(&graph, &[]);
        assert_eq!(result.status, Ets2RouteMatchStatus::Invalid);
        assert!(!result.is_usable);
    }

    #[test]
    fn only_one_node_is_not_usable() {
        let graph = graph_with_nodes(&[1, 2, 3]);
        let result = match_ets2_route_uids(&graph, &[2]);
        assert_eq!(result.matched_count, 1);
        assert_eq!(result.status, Ets2RouteMatchStatus::Failed);
        assert!(!result.is_usable);
    }

    #[test]
    fn ratio_under_threshold_is_unusable() {
        let graph = graph_with_nodes(&[1, 2, 3, 4, 5]);
        // 3/5 = 60%
        let result = match_ets2_route_uids(&graph, &[1, 2, 99, 99, 5]);
        assert_eq!(result.matched_count, 3);
        assert!((result.match_ratio - 0.6).abs() < f64::EPSILON);
        assert_eq!(result.status, Ets2RouteMatchStatus::Failed);
        assert!(!result.is_usable);
    }

    #[test]
    fn ratio_over_threshold_is_usable() {
        let graph = graph_with_nodes(&[10, 20, 30, 40, 50]);
        // 4/5 = 80%
        let result = match_ets2_route_uids(&graph, &[10, 20, 999, 40, 50]);
        assert!(result.is_usable);
        assert_eq!(result.status, Ets2RouteMatchStatus::Partial);
    }

    #[test]
    fn build_output_from_matched_nodes() {
        let graph = graph_with_nodes(&[1, 2, 3, 4]);
        let out = build_router_output_from_node_ids(&graph, &[1, 2, 3, 4]).unwrap();
        assert_eq!(out.route_node_ids, vec![1, 2, 3, 4]);
        assert_eq!(out.waypoints.len(), 4);
        assert!(out.distance_m > 0.0);
        let json = serde_json::to_string(&out.waypoints).unwrap();
        assert!(json.starts_with("[[") && json.contains(","));
    }

    #[test]
    fn build_output_rejects_missing_position() {
        let graph = graph_with_nodes(&[1, 2]);
        let err = build_router_output_from_node_ids(&graph, &[1, 99]).unwrap_err();
        assert!(err.contains("99"));
    }

    #[test]
    fn build_output_rejects_single_node() {
        let graph = graph_with_nodes(&[1, 2]);
        assert!(build_router_output_from_node_ids(&graph, &[1]).is_err());
    }

    #[test]
    fn build_output_rejects_degenerate_route() {
        let graph = RouterGraph::new(
            vec![(1, 0.0, 0.0), (2, 0.0, 0.0)],
            vec![(1, 2, 0.0)],
        );
        assert!(build_router_output_from_node_ids(&graph, &[1, 2]).is_err());
    }

    fn straight_route_graph() -> (RouterGraph, Vec<u64>) {
        let uids = vec![1_u64, 2, 3, 4];
        let nodes = vec![
            (1, 0.0, 0.0),
            (2, 100.0, 0.0),
            (3, 200.0, 0.0),
            (4, 300.0, 0.0),
        ];
        let edges = vec![(1, 2, 100.0), (2, 3, 100.0), (3, 4, 100.0)];
        (RouterGraph::new(nodes, edges), uids)
    }

    #[test]
    fn trim_truck_near_route_start() {
        let (graph, uids) = straight_route_graph();
        let trim = find_route_start_index_for_truck(&graph, &uids, 5.0, 0.0, Some(0.75));
        assert_eq!(trim.start_index, 0);
        assert_eq!(trim.snap_status, Ets2RouteSnapStatus::Ok);
        assert!(!trim.trimmed);
    }

    #[test]
    fn trim_truck_near_middle_segment() {
        let (graph, uids) = straight_route_graph();
        let trim = find_route_start_index_for_truck(&graph, &uids, 151.0, 2.0, Some(0.75));
        assert!(trim.start_index > 0);
        assert_eq!(trim.snap_status, Ets2RouteSnapStatus::Ok);
        assert!(trim.trimmed);
        assert_eq!(trim.trimmed_node_count, uids.len() - trim.start_index);
    }

    #[test]
    fn trim_truck_near_end_keeps_two_nodes() {
        let (graph, uids) = straight_route_graph();
        let trim = find_route_start_index_for_truck(&graph, &uids, 290.0, 0.0, Some(0.75));
        assert_eq!(trim.start_index, 2);
        assert_eq!(trim.trimmed_node_count, 2);
        assert_eq!(trim.snap_status, Ets2RouteSnapStatus::Ok);
    }

    #[test]
    fn trim_truck_too_far_falls_back_to_zero() {
        let (graph, uids) = straight_route_graph();
        let trim = find_route_start_index_for_truck(&graph, &uids, 1000.0, 1000.0, Some(0.75));
        assert_eq!(trim.start_index, 0);
        assert_eq!(trim.snap_status, Ets2RouteSnapStatus::TooFar);
        assert!(!trim.trimmed);
    }

    #[test]
    fn trim_degenerate_identical_points_no_panic() {
        let graph = RouterGraph::new(
            vec![(1, 0.0, 0.0), (2, 0.0, 0.0), (3, 0.0, 0.0)],
            vec![(1, 2, 0.0), (2, 3, 0.0)],
        );
        let trim = find_route_start_index_for_truck(&graph, &[1, 2, 3], 0.0, 0.0, None);
        assert_eq!(trim.start_index, 0);
        assert!(!trim.snap_status.as_str().is_empty());
    }

    #[test]
    fn build_trimmed_publishes_fewer_nodes() {
        let (graph, uids) = straight_route_graph();
        let (out, trim) = build_trimmed_ets2_router_output(&graph, &uids, Some(151.0), Some(0.0), Some(0.75))
            .unwrap();
        assert!(trim.trimmed);
        assert!(out.route_node_ids.len() < uids.len());
        assert_eq!(out.route_node_ids.len(), trim.trimmed_node_count);
    }

    #[test]
    fn build_trimmed_rejects_too_short_after_trim() {
        let graph = RouterGraph::new(
            vec![(1, 0.0, 0.0), (2, 100.0, 0.0)],
            vec![(1, 2, 100.0)],
        );
        // Force start_index=1 manually via find — truck past node 2 would only leave 1 node if not clamped
        let trim = find_route_start_index_for_truck(&graph, &[1, 2], 200.0, 0.0, None);
        assert_eq!(trim.start_index, 0, "two-node route must clamp to keep both nodes");
        let err = build_trimmed_ets2_router_output(&graph, &[1], Some(0.0), Some(0.0), None);
        assert!(err.is_err());
    }

    #[test]
    fn build_trimmed_no_position_uses_full_route() {
        let (graph, uids) = straight_route_graph();
        let (out, trim) =
            build_trimmed_ets2_router_output(&graph, &uids, None, None, None).unwrap();
        assert_eq!(trim.snap_status, Ets2RouteSnapStatus::NoPosition);
        assert_eq!(out.route_node_ids, uids);
        assert!(!trim.trimmed);
    }

    fn ok_trim_at(start: usize, dist: f64) -> Ets2RouteTrimResult {
        trim_result_for_start_index(&[1, 2, 3, 4], start, dist, Ets2RouteSnapStatus::Ok, None)
    }

    #[test]
    fn progress_advances_when_start_index_increases() {
        let trim = ok_trim_at(2, 5.0);
        let d = decide_ets2_route_progress(2, 0, 4, &trim, 0.0, 1000, 0);
        assert_eq!(d.progress_status, Ets2RouteProgressStatus::Advanced);
        assert!(d.should_republish);
        assert_eq!(d.effective_start_index, 2);
    }

    #[test]
    fn progress_unchanged_on_same_segment() {
        let trim = ok_trim_at(0, 3.0);
        let d = decide_ets2_route_progress(0, 0, 4, &trim, 0.0, 1000, 0);
        assert_eq!(d.progress_status, Ets2RouteProgressStatus::Unchanged);
        assert!(!d.should_republish);
    }

    #[test]
    fn progress_regression_ignored() {
        let trim = ok_trim_at(0, 3.0);
        let d = decide_ets2_route_progress(0, 2, 4, &trim, 0.0, 1000, 0);
        assert_eq!(d.progress_status, Ets2RouteProgressStatus::RegressionIgnored);
        assert_eq!(d.effective_start_index, 2);
        assert!(!d.should_republish);
    }

    #[test]
    fn progress_single_bad_snap_keeps_import() {
        let trim = trim_result_for_start_index(
            &[1, 2, 3, 4],
            0,
            90.0,
            Ets2RouteSnapStatus::TooFar,
            None,
        );
        let d = decide_ets2_route_progress(0, 0, 4, &trim, 1.0, 1000, 0);
        assert!(!d.should_release);
        assert_eq!(d.progress_status, Ets2RouteProgressStatus::SnapBad);
        assert!(d.counts_as_offroute);
    }

    #[test]
    fn progress_offroute_secs_triggers_release() {
        let trim = trim_result_for_start_index(
            &[1, 2, 3, 4],
            0,
            90.0,
            Ets2RouteSnapStatus::TooFar,
            None,
        );
        let d = decide_ets2_route_progress(0, 0, 4, &trim, 3.0, 1000, 0);
        assert!(d.should_release);
        assert_eq!(d.release_reason, Some("ets2_off_route"));
    }

    #[test]
    fn progress_hard_dist_triggers_release() {
        let trim = trim_result_for_start_index(
            &[1, 2, 3, 4],
            0,
            150.0,
            Ets2RouteSnapStatus::TooFar,
            None,
        );
        let d = decide_ets2_route_progress(0, 0, 4, &trim, 0.0, 1000, 0);
        assert!(d.should_release);
    }

    #[test]
    fn progress_rate_limit_blocks_republish() {
        let trim = ok_trim_at(2, 5.0);
        let d = decide_ets2_route_progress(2, 0, 4, &trim, 0.0, 150, 100);
        assert!(!d.should_republish);
        assert_eq!(d.progress_status, Ets2RouteProgressStatus::Unchanged);
    }

    #[test]
    fn progress_too_short_triggers_release() {
        let trim = trim_result_for_start_index(&[1], 0, 1.0, Ets2RouteSnapStatus::Ok, None);
        let d = decide_ets2_route_progress(0, 0, 1, &trim, 0.0, 1000, 0);
        assert!(d.should_release);
        assert_eq!(d.release_reason, Some("too_short"));
    }

    #[test]
    fn waypoint_json_matches_router_format() {
        let graph = graph_with_nodes(&[1, 2, 3]);
        let out = build_router_output_from_node_ids(&graph, &[1, 2, 3]).unwrap();
        let parsed: Vec<[f64; 2]> = serde_json::from_str(&serde_json::to_string(&out.waypoints).unwrap())
            .unwrap();
        assert_eq!(parsed, out.waypoints);
    }

    fn chain_graph(count: u64) -> RouterGraph {
        let nodes: Vec<(u64, f64, f64)> = (1..=count)
            .map(|i| (i, (i - 1) as f64 * 100.0, 0.0))
            .collect();
        let edges: Vec<(u64, u64, f64)> = (1..count)
            .map(|i| (i, i + 1, 100.0))
            .collect();
        RouterGraph::new(nodes, edges)
    }

    #[test]
    fn repair_not_needed_without_gaps() {
        let graph = chain_graph(4);
        let uids = vec![1_u64, 2, 3, 4];
        let matched = vec![1_u64, 2, 3, 4];
        let repair = repair_ets2_route_gaps(&graph, &uids, &matched);
        assert_eq!(repair.status, Ets2RouteRepairStatus::NotNeeded);
        assert_eq!(repair.route_node_ids, matched);
        assert!(repair.import_allowed);
    }

    #[test]
    fn repair_small_gap_inserts_astar_nodes() {
        let graph = chain_graph(6);
        let uids = vec![1_u64, 2, 999, 998, 5, 6];
        let matched = vec![1_u64, 2, 5, 6];
        let repair = repair_ets2_route_gaps(&graph, &uids, &matched);
        assert_eq!(repair.status, Ets2RouteRepairStatus::Repaired);
        assert_eq!(repair.route_node_ids, vec![1, 2, 3, 4, 5, 6]);
        assert_eq!(repair.inserted_node_count, 2);
        assert_eq!(repair.success_count, 1);
    }

    #[test]
    fn repair_path_has_no_duplicate_anchors() {
        let graph = chain_graph(6);
        let uids = vec![1_u64, 2, 999, 5, 6];
        let matched = vec![1_u64, 2, 5, 6];
        let repair = repair_ets2_route_gaps(&graph, &uids, &matched);
        let ids = repair.route_node_ids;
        assert_eq!(ids.windows(2).filter(|w| w[0] == w[1]).count(), 0);
    }

    #[test]
    fn repair_leading_gap_not_repaired() {
        let graph = chain_graph(4);
        let uids = vec![999_u64, 998, 1, 2, 3];
        let matched = vec![1_u64, 2, 3];
        let gaps = detect_ets2_route_gaps(&uids, &graph);
        assert!(gaps.is_empty());
        let repair = repair_ets2_route_gaps(&graph, &uids, &matched);
        assert_eq!(repair.status, Ets2RouteRepairStatus::NotNeeded);
    }

    #[test]
    fn repair_trailing_gap_not_repaired() {
        let graph = chain_graph(4);
        let uids = vec![1_u64, 2, 3, 999, 998];
        let matched = vec![1_u64, 2, 3];
        let gaps = detect_ets2_route_gaps(&uids, &graph);
        assert!(gaps.is_empty());
        let repair = repair_ets2_route_gaps(&graph, &uids, &matched);
        assert_eq!(repair.status, Ets2RouteRepairStatus::NotNeeded);
    }

    #[test]
    fn repair_oversize_gap_rejects_import() {
        let graph = chain_graph(10);
        let uids = vec![
            1_u64, 2,
            901, 902, 903, 904, 905, 906, 907, 908, 909,
            10,
        ];
        let matched = vec![1_u64, 2, 10];
        let repair = repair_ets2_route_gaps(&graph, &uids, &matched);
        assert_eq!(repair.status, Ets2RouteRepairStatus::Failed);
        assert!(!repair.import_allowed);
    }

    #[test]
    fn repair_no_path_partial_or_failed() {
        let graph = RouterGraph::new(
            vec![(1, 0.0, 0.0), (2, 100.0, 0.0), (5, 500.0, 0.0), (6, 600.0, 0.0)],
            vec![(1, 2, 100.0), (5, 6, 100.0)],
        );
        let uids = vec![1_u64, 2, 999, 5, 6];
        let matched = vec![1_u64, 2, 5, 6];
        let repair = repair_ets2_route_gaps(&graph, &uids, &matched);
        assert_eq!(repair.status, Ets2RouteRepairStatus::PartialUnrepaired);
        assert_eq!(repair.route_node_ids, matched);
        assert!(repair.import_allowed);
    }

    #[test]
    fn repair_multiple_small_gaps() {
        let graph = chain_graph(10);
        let uids = vec![1_u64, 2, 991, 5, 992, 8, 9, 10];
        let matched = vec![1_u64, 2, 5, 8, 9, 10];
        let repair = repair_ets2_route_gaps(&graph, &uids, &matched);
        assert_eq!(repair.status, Ets2RouteRepairStatus::Repaired);
        assert_eq!(repair.success_count, 2);
        assert_eq!(repair.route_node_ids, vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);
        assert_eq!(repair.inserted_node_count, 4);
    }

    #[test]
    fn repair_then_trim_build_works() {
        let graph = chain_graph(6);
        let uids = vec![1_u64, 2, 999, 998, 5, 6];
        let matched = vec![1_u64, 2, 5, 6];
        let repair = repair_ets2_route_gaps(&graph, &uids, &matched);
        let (out, _trim) =
            build_trimmed_ets2_router_output(&graph, &repair.route_node_ids, Some(0.0), Some(0.0), None)
                .unwrap();
        assert_eq!(out.route_node_ids.len(), 6);
    }

    #[test]
    fn repair_insert_budget_exceeded_fails() {
        let graph = chain_graph(400);
        let uids = vec![
            1_u64,
            991,
            80,
            992,
            160,
            993,
            240,
            994,
            320,
        ];
        let matched = vec![1_u64, 80, 160, 240, 320];
        let repair = repair_ets2_route_gaps(&graph, &uids, &matched);
        assert_eq!(repair.status, Ets2RouteRepairStatus::Failed);
        assert!(!repair.import_allowed);
    }

    #[test]
    fn compare_graph_coord_delta_matches_identical_positions() {
        let graph = chain_graph(3);
        let waypoints = vec![
            Ets2RouteWaypoint {
                uid: 1,
                x: 0.0,
                z: 0.0,
                flags: ETS2_WP_FLAG_HAS_POSITION,
                ..Default::default()
            },
            Ets2RouteWaypoint {
                uid: 2,
                x: 100.0,
                z: 0.0,
                flags: ETS2_WP_FLAG_HAS_POSITION,
                ..Default::default()
            },
        ];
        let delta = compare_ets2_graph_coord_delta(&graph, &waypoints);
        assert_eq!(delta.count, 2);
        assert!(delta.max_m < 0.01);
    }

    #[test]
    fn compare_graph_coord_delta_skips_missing_position_flag() {
        let graph = chain_graph(2);
        let waypoints = vec![Ets2RouteWaypoint {
            uid: 1,
            x: 999.0,
            z: 999.0,
            flags: 0,
            ..Default::default()
        }];
        let delta = compare_ets2_graph_coord_delta(&graph, &waypoints);
        assert_eq!(delta.count, 0);
    }

    fn dist_wp(uid: u64, distance: f32) -> Ets2RouteWaypoint {
        Ets2RouteWaypoint {
            uid,
            distance,
            flags: ETS2_WP_FLAG_HAS_DISTANCE | ETS2_WP_FLAG_UNTRUSTED,
            ..Default::default()
        }
    }

    #[test]
    fn graph_distance_compare_similar_is_untrusted() {
        let graph = chain_graph(3);
        let route = vec![1_u64, 2, 3];
        let waypoints = vec![dist_wp(1, 200.0), dist_wp(2, 100.0), dist_wp(3, 0.0)];
        let cmp = compare_ets2_graph_distance(&graph, &route, &waypoints);
        assert!((cmp.graph_total_m - 200.0).abs() < 0.01);
        assert_eq!(cmp.status, Ets2GraphDistanceStatus::Untrusted);
        assert!((cmp.graph_ratio - 1.0).abs() < 0.01);
    }

    #[test]
    fn graph_distance_compare_mismatch_still_untrusted() {
        let graph = chain_graph(3);
        let route = vec![1_u64, 2, 3];
        let waypoints = vec![dist_wp(1, 500.0), dist_wp(2, 400.0), dist_wp(3, 0.0)];
        let cmp = compare_ets2_graph_distance(&graph, &route, &waypoints);
        assert_eq!(cmp.status, Ets2GraphDistanceStatus::Untrusted);
        assert!(cmp.first_vs_graph_delta_m > 100.0);
    }

    #[test]
    fn graph_distance_compare_none_without_distance() {
        let graph = chain_graph(3);
        let cmp = compare_ets2_graph_distance(&graph, &[1, 2, 3], &[]);
        assert_eq!(cmp.status, Ets2GraphDistanceStatus::None);
    }
}
