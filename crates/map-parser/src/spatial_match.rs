//! Phase 5.23 — Spatial-Matching Infrastructure for Cross-Sector Edges.
//!
//! Provides the data structures and primitives used by [`crate::graph::GraphBuilder`]
//! to bridge orphan road endpoints across sector boundaries via a uniform-grid
//! spatial index plus multi-pass filter pipeline.
//!
//! Sub-Phase 5.23a delivers infrastructure only: index build + circle query.
//! Filter pipeline, multi-pass driver and edge generation land in 5.23b–e.
//! See `outputs/position_matching_spec.md` for the full algorithm.

use std::collections::HashMap;

use crate::graph::GraphNode;

/// Globally unique sector index (0-based, assigned during merge).
pub type SectorId = u32;

/// Marker for "no known sector" — populated when a node UID is missing from
/// the `node_to_sector` map. The spatial index never panics on unknown UIDs.
pub const SECTOR_ID_UNKNOWN: SectorId = u32::MAX;

/// One endpoint of a road that failed to resolve in the global node lookup.
/// We know the OTHER endpoint's position (`resolved_pos`) and need to find
/// the MISSING endpoint in a neighbouring sector.
#[derive(Debug, Clone)]
pub struct OrphanEndpoint {
    pub road_uid: u64,
    /// The endpoint UID that WAS found in `node_lookup`.
    pub resolved_uid: u64,
    /// World position of the resolved endpoint (XYZ, world units).
    pub resolved_pos: [f64; 3],
    /// The endpoint UID NOT found (logging only).
    pub missing_uid: u64,
    /// Sector of the resolved node — used by the SECTOR-FILTER to reject
    /// same-sector candidates.
    pub sector_id: SectorId,
    /// Unit vector along the road, computable when at least one neighbour
    /// road at `resolved_uid` has both endpoints resolved. `None` => the
    /// HEADING-FILTER is skipped for this orphan (5.23d).
    pub road_dir_hint: Option<[f64; 3]>,
}

/// Lightweight reference into the [`SpatialIndex`] grid.
#[derive(Debug, Clone, Copy)]
pub struct NodeRef {
    pub uid: u64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub sector_id: SectorId,
}

/// Match-confidence — assigned per-pass, downgraded on ambiguity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfidenceLevel {
    High,
    Medium,
    Low,
}

impl ConfidenceLevel {
    /// Drop one level (High -> Medium -> Low -> Low).
    pub fn downgrade(self) -> Self {
        match self {
            ConfidenceLevel::High => ConfidenceLevel::Medium,
            ConfidenceLevel::Medium => ConfidenceLevel::Low,
            ConfidenceLevel::Low => ConfidenceLevel::Low,
        }
    }

    /// Edge-direction tag written into [`crate::graph::GraphEdge`].
    pub fn to_direction_string(self) -> &'static str {
        match self {
            ConfidenceLevel::High => "cross_sector_spatial_high",
            ConfidenceLevel::Medium => "cross_sector_spatial",
            ConfidenceLevel::Low => "cross_sector_spatial_low",
        }
    }
}

/// Per-pass filter parameters. Loaded by the multi-pass driver in 5.23c.
#[derive(Debug, Clone, Copy)]
pub struct PassConfig {
    pub max_dist: f64,
    pub z_tol: f64,
    pub heading_threshold: f64,
    pub require_heading: bool,
    pub level: ConfidenceLevel,
}

/// Default cell size in world units (metres). 250 m yields ~260 nodes/cell
/// on average for `base_map.scs` and bounds query cost at ~9–25 cells.
pub const DEFAULT_CELL_SIZE: f64 = 250.0;

/// 2D-distance threshold above which the Z-tolerance is doubled (long road
/// segments through hilly terrain may cross significant elevation deltas).
pub const Z_RELAX_2D_THRESHOLD: f64 = 100.0;

/// Ambiguity margin (metres) between the best and second-best candidate.
pub const AMBIGUITY_MARGIN: f64 = 10.0;

/// Uniform-grid spatial index keyed by `(ix, iz) = (floor(x/cs), floor(z/cs))`.
/// Y is stored on the [`NodeRef`] but excluded from the grid: ETS2 maps are
/// XZ-ground with Y as elevation, so the grid is naturally 2D.
#[derive(Debug, Default)]
pub struct SpatialIndex {
    pub cells: HashMap<(i32, i32), Vec<NodeRef>>,
    pub cell_size: f64,
}

impl SpatialIndex {
    pub fn total_nodes(&self) -> usize {
        self.cells.values().map(|c| c.len()).sum()
    }

    pub fn cell_count(&self) -> usize {
        self.cells.len()
    }
}

/// Build a [`SpatialIndex`] over `nodes`, attaching `sector_id` per node via
/// `node_to_sector` (missing UIDs become [`SECTOR_ID_UNKNOWN`]).
pub fn build_spatial_index(
    nodes: &[GraphNode],
    node_to_sector: &HashMap<u64, SectorId>,
    cell_size: f64,
) -> SpatialIndex {
    let mut cells: HashMap<(i32, i32), Vec<NodeRef>> = HashMap::new();
    for node in nodes {
        let sid = node_to_sector
            .get(&node.uid)
            .copied()
            .unwrap_or(SECTOR_ID_UNKNOWN);
        let ix = (node.x / cell_size).floor() as i32;
        let iz = (node.z / cell_size).floor() as i32;
        cells.entry((ix, iz)).or_default().push(NodeRef {
            uid: node.uid,
            x: node.x,
            y: node.y,
            z: node.z,
            sector_id: sid,
        });
    }
    SpatialIndex { cells, cell_size }
}

/// 2D circle query in the XZ plane. Returns every [`NodeRef`] whose
/// `(x, z)` lies inside `radius` of `center`. Caller is responsible for the
/// Y / sector / heading / distance-3D filters (Phase D in the spec).
pub fn query_circle<'a>(
    index: &'a SpatialIndex,
    center: &[f64; 3],
    radius: f64,
) -> Vec<&'a NodeRef> {
    let cell_radius = (radius / index.cell_size).ceil() as i32 + 1;
    let cx = (center[0] / index.cell_size).floor() as i32;
    let cz = (center[2] / index.cell_size).floor() as i32;
    let r2 = radius * radius;

    let mut results: Vec<&NodeRef> = Vec::new();
    for dix in -cell_radius..=cell_radius {
        for diz in -cell_radius..=cell_radius {
            let key = (cx + dix, cz + diz);
            if let Some(cell) = index.cells.get(&key) {
                for node in cell {
                    let dx = node.x - center[0];
                    let dz = node.z - center[2];
                    if dx * dx + dz * dz <= r2 {
                        results.push(node);
                    }
                }
            }
        }
    }
    results
}

#[cfg(test)]
mod tests {
    use super::*;

    fn n(uid: u64, x: f64, y: f64, z: f64) -> GraphNode {
        GraphNode { uid, x, y, z }
    }

    fn sectors(pairs: &[(u64, SectorId)]) -> HashMap<u64, SectorId> {
        pairs.iter().copied().collect()
    }

    #[test]
    fn confidence_downgrade_terminates_at_low() {
        assert_eq!(ConfidenceLevel::High.downgrade(), ConfidenceLevel::Medium);
        assert_eq!(ConfidenceLevel::Medium.downgrade(), ConfidenceLevel::Low);
        assert_eq!(ConfidenceLevel::Low.downgrade(), ConfidenceLevel::Low);
    }

    #[test]
    fn direction_strings_are_distinct() {
        let h = ConfidenceLevel::High.to_direction_string();
        let m = ConfidenceLevel::Medium.to_direction_string();
        let l = ConfidenceLevel::Low.to_direction_string();
        assert_ne!(h, m);
        assert_ne!(m, l);
        assert!(h.starts_with("cross_sector"));
    }

    #[test]
    fn empty_index_has_no_results() {
        let idx = build_spatial_index(&[], &HashMap::new(), DEFAULT_CELL_SIZE);
        assert_eq!(idx.total_nodes(), 0);
        assert_eq!(idx.cell_count(), 0);
        let hits = query_circle(&idx, &[0.0, 0.0, 0.0], 1000.0);
        assert!(hits.is_empty());
    }

    #[test]
    fn build_index_assigns_sectors_and_buckets_by_cell() {
        let nodes = vec![
            n(1, 100.0, 0.0, 100.0),  // cell (0,0)
            n(2, 200.0, 0.0, 50.0),   // cell (0,0)
            n(3, 300.0, 0.0, 100.0),  // cell (1,0)
            n(4, 50.0, 0.0, 300.0),   // cell (0,1)
            n(5, -50.0, 0.0, -100.0), // cell (-1,-1) — sector mapping omitted
        ];
        let s = sectors(&[(1, 0), (2, 0), (3, 1), (4, 0)]);
        let idx = build_spatial_index(&nodes, &s, 250.0);
        assert_eq!(idx.total_nodes(), 5);
        assert_eq!(idx.cells.get(&(0, 0)).map(|c| c.len()), Some(2));
        assert_eq!(idx.cells.get(&(1, 0)).map(|c| c.len()), Some(1));
        assert_eq!(idx.cells.get(&(0, 1)).map(|c| c.len()), Some(1));
        assert_eq!(idx.cells.get(&(-1, -1)).map(|c| c.len()), Some(1));

        let unknown_cell = idx.cells.get(&(-1, -1)).unwrap();
        assert_eq!(unknown_cell[0].sector_id, SECTOR_ID_UNKNOWN);
    }

    #[test]
    fn query_circle_returns_only_nodes_within_2d_radius() {
        let nodes = vec![
            n(1, 0.0, 0.0, 0.0),
            n(2, 30.0, 0.0, 0.0),
            n(3, 0.0, 0.0, 60.0),
            n(4, 100.0, 0.0, 100.0),
            n(5, 0.0, 1000.0, 0.0), // far in Y but at origin XZ — must hit (2D query)
        ];
        let idx = build_spatial_index(&nodes, &HashMap::new(), 50.0);
        let hits = query_circle(&idx, &[0.0, 0.0, 0.0], 50.0);
        let mut uids: Vec<u64> = hits.iter().map(|n| n.uid).collect();
        uids.sort();
        // 1 (dist 0), 2 (dist 30), 5 (dist 0 XZ) inside r=50
        // 3 (dist 60), 4 (dist ~141) outside
        assert_eq!(uids, vec![1, 2, 5]);
    }

    #[test]
    fn query_circle_spans_neighbour_cells() {
        // Cell-size 25 is small enough that the query origin sits in (0,0)
        // but the relevant nodes live in neighbour cells.
        let nodes = vec![
            n(10, -5.0, 0.0, -5.0),    // cell (-1,-1), 2D dist ≈ sqrt(17² + 17²) ≈ 24
            n(11, 30.0, 0.0, 30.0),    // cell (1,1), 2D dist ≈ sqrt(18² + 18²) ≈ 25.5
            n(12, 600.0, 0.0, 600.0),  // far away
        ];
        let idx = build_spatial_index(&nodes, &HashMap::new(), 25.0);
        let hits = query_circle(&idx, &[12.0, 0.0, 12.0], 30.0);
        let mut uids: Vec<u64> = hits.iter().map(|n| n.uid).collect();
        uids.sort();
        assert_eq!(uids, vec![10, 11]);
    }
}
