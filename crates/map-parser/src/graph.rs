//! Graph builder — converts parsed sector data into a routing graph.
//!
//! Merges nodes and roads from all loaded sectors, deduplicates by UID,
//! and emits directed edges with speed limits and lane counts.

use std::collections::HashMap;
use std::time::Instant;

use serde::{Deserialize, Serialize};
use tracing::{debug, info, instrument, warn};

use crate::ppd::{self, NavCurve, PrefabDescriptor};
use crate::road_look::RoadLookEntry;
use crate::sector::{ParsedSector, RawBuilding, RawFerry, RawNode, RawPrefab, RawRoad};
use crate::signs::TrafficSign;
use crate::spatial_match::{
    apply_filters, build_spatial_index, pass1_strict_config, pass2_config, query_circle,
    select_best_match, stitch_cross_sector_boundary, OrphanEndpoint, SectorId, DEFAULT_CELL_SIZE,
    SECTOR_ID_UNKNOWN,
};
use std::collections::HashSet;

use crate::spline::{quat_rotate_vec, Vec3 as SplineVec3, FORWARD};

// ---------------------------------------------------------------------------
// Graph types
// ---------------------------------------------------------------------------

fn default_quat() -> [f32; 4] {
    [0.0; 4]
}

fn default_lane_width() -> f32 {
    3.75
}

/// A node in the routing graph.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, bincode::Encode, bincode::Decode)]
pub struct GraphNode {
    pub uid: u64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    /// Rotation quaternion [qw, qx, qy, qz] from the binary node record.
    /// `[0.0; 4]` means not set (sized-format nodes or missing data).
    #[serde(default = "default_quat")]
    pub rotation: [f32; 4],
}

/// A directed edge in the routing graph.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, bincode::Encode, bincode::Decode)]
pub struct GraphEdge {
    pub uid: u64,
    pub from: u64,
    pub to: u64,
    /// Length in meters.
    pub distance_m: f64,
    /// Speed limit in km/h. `None` = unknown.
    pub speed_limit_kmh: Option<f64>,
    /// Number of lanes in this direction.
    pub lanes: u8,
    /// Direction tag: "forward", "backward", "prefab".
    pub direction: String,
    /// DLC-guard byte propagated from the source road (0 = no DLC required).
    /// Prefab-derived edges default to 0.
    pub dlc_guard: u8,
    /// `true` when the underlying road is hidden from the in-game UI map.
    /// Prefab-derived edges default to `false`.
    pub is_hidden: bool,
    /// `true` when the underlying road is flagged "GPS-avoid".
    /// Prefab-derived edges default to `false`.
    pub gps_avoid: bool,
    /// Road-look token64 for this direction (0 for non-road edges).
    #[serde(default)]
    pub road_look_token: u64,
    /// Lane count in the opposite direction on the same road (0 for non-road edges).
    #[serde(default)]
    pub lanes_opposite: u8,
    /// Lane width in metres derived from the road-look type (default 3.75 for unknown).
    #[serde(default = "default_lane_width")]
    pub lane_width_m: f32,
    /// Lateral median shift in metres from the road-look (`road_offset`, default 0.0).
    #[serde(default)]
    pub road_offset_m: f32,
}

/// A prefab (junction/intersection) in the graph.
#[derive(Debug, Clone, Serialize, Deserialize, bincode::Encode, bincode::Decode, PartialEq)]
pub struct Prefab {
    pub uid: u64,
    pub template_token: u64,
    pub connected_node_uids: Vec<u64>,
}

/// A single AI nav path through a prefab: one input ControlNode -> one output
/// ControlNode, optionally lane-resolved, with pre-sampled spline points in
/// WORLD coordinates.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, bincode::Encode, bincode::Decode)]
pub struct PrefabAiPath {
    pub from_node_uid: u64,
    pub to_node_uid: u64,
    pub start_lane_idx: u8,
    pub end_lane_idx: u8,
    pub spline_points: Vec<[f32; 3]>,
    pub length_m: f32,
    pub speed_kmh: Option<u16>,
    pub blinker: Option<ppd::Blinker>,
    pub curve_indices: Vec<u16>,
    pub semaphore_id: Option<i32>,
    /// Rotation of the first NavCurve at path entry [qw, qx, qy, qz] (WXYZ).
    #[serde(default = "default_quat")]
    pub start_rotation: [f32; 4],
    /// Rotation of the last NavCurve at path exit [qw, qx, qy, qz] (WXYZ).
    #[serde(default = "default_quat")]
    pub end_rotation: [f32; 4],
}

/// A single prefab placement in the world, referencing a shared descriptor.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, bincode::Encode, bincode::Decode)]
pub struct PrefabInstance {
    pub uid: u64,
    pub token: u64,
    pub origin_pos: [f32; 3],
    pub origin_rot: [f32; 4],
    pub node_uids: Vec<u64>,
}

/// The complete routing graph produced from one or more sectors.
#[derive(Debug, Clone, Serialize, Deserialize, Default, bincode::Encode, bincode::Decode)]
pub struct MapGraph {
    pub nodes: Vec<GraphNode>,
    pub edges: Vec<GraphEdge>,
    pub signs: Vec<TrafficSign>,
    pub prefabs: Vec<Prefab>,
    pub stats: BuildStats,
    pub prefab_ai_paths: Vec<PrefabAiPath>,
    pub prefab_instances: Vec<PrefabInstance>,
    pub prefab_descriptors: HashMap<u64, PrefabDescriptor>,
}

/// Result of [`GraphBuilder::analyze_roads_for_audit`] — road classification
/// before graph build, used by the road-drop-audit binary.
pub struct RoadAuditResult {
    /// Roads where BOTH endpoints are absent from the merged node map.
    pub both_unresolved: Vec<crate::sector::RawRoad>,
    /// Roads where exactly ONE endpoint resolves.
    /// Tuple: (road, resolved_node_uid, resolved_pos \[x, y, z\] in metres).
    pub one_unresolved: Vec<(crate::sector::RawRoad, u64, [f64; 3])>,
}

/// Timing and count statistics from a graph build.
#[derive(Debug, Clone, Serialize, Deserialize, Default, bincode::Encode, bincode::Decode)]
pub struct BuildStats {
    pub node_count: usize,
    pub edge_count: usize,
    pub sign_count: usize,
    pub prefab_count: usize,
    pub sectors_merged: usize,
    pub build_time_ms: f64,
    #[serde(default)]
    pub ppd_files_attempted: usize,
    #[serde(default)]
    pub ppd_files_loaded: usize,
    #[serde(default)]
    pub ppd_files_failed: usize,
    #[serde(default)]
    pub ppd_total_nav_curves: usize,
    #[serde(default)]
    pub ppd_failed_token_miss: usize,
    #[serde(default)]
    pub ppd_failed_archive_miss: usize,
    #[serde(default)]
    pub ppd_failed_parse_err: usize,
    #[serde(default)]
    pub ppd_chain_broken: usize,
}

// ---------------------------------------------------------------------------
// Builder
// ---------------------------------------------------------------------------

/// Accumulates sectors and builds the final graph.
#[derive(Default)]
pub struct GraphBuilder {
    nodes: HashMap<u64, RawNode>,
    roads: Vec<RawRoad>,
    raw_prefabs: Vec<RawPrefab>,
    raw_signs: Vec<crate::sector::RawSign>,
    ferries: Vec<RawFerry>,
    buildings: Vec<RawBuilding>,
    /// Phase 5.23a: maps node UID -> sector index assigned during
    /// [`GraphBuilder::merge_sector`]. Used by the SECTOR-FILTER in the
    /// spatial-match pipeline to reject same-sector candidates.
    node_to_sector: HashMap<u64, SectorId>,
    sectors_merged: usize,
    /// Phase 5.28-C: road-look token → lane counts, loaded from
    /// `/def/road_look.sii`.  Used in [`GraphBuilder::build`] to assign
    /// `lanes_forward` / `lanes_backward` for legacy-format roads that carry
    /// zero in those fields.
    road_look: HashMap<u64, RoadLookEntry>,
    /// PPD descriptor cache: template_token -> PrefabDescriptor.
    ppd_descriptors: HashMap<u64, PrefabDescriptor>,
    ppd_files_attempted: usize,
    ppd_files_loaded: usize,
    ppd_files_failed: usize,
    ppd_total_nav_curves: usize,
    ppd_failed_token_miss: usize,
    ppd_failed_archive_miss: usize,
    ppd_failed_parse_err: usize,
    ppd_chain_broken: usize,
}

impl GraphBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Attach road-look lane-count data loaded from `road_look.sii`.
    /// Must be called before [`GraphBuilder::build`].
    pub fn set_road_look(&mut self, map: HashMap<u64, RoadLookEntry>) {
        self.road_look = map;
    }

    /// Attach PPD descriptors loaded from archive PPD files.
    /// Must be called before [`GraphBuilder::build`].
    pub fn set_ppd_descriptors(&mut self, map: HashMap<u64, PrefabDescriptor>) {
        self.ppd_descriptors = map;
    }

    /// Record PPD load statistics for inclusion in [`BuildStats`].
    pub fn set_ppd_stats(
        &mut self,
        attempted: usize,
        loaded: usize,
        failed: usize,
        nav_curves: usize,
        token_miss: usize,
        archive_miss: usize,
        parse_err: usize,
    ) {
        self.ppd_files_attempted = attempted;
        self.ppd_files_loaded = loaded;
        self.ppd_files_failed = failed;
        self.ppd_total_nav_curves = nav_curves;
        self.ppd_failed_token_miss = token_miss;
        self.ppd_failed_archive_miss = archive_miss;
        self.ppd_failed_parse_err = parse_err;
    }

    /// Merge one parsed sector into the builder.
    /// Duplicate node UIDs are silently overwritten (last writer wins).
    #[instrument(skip(self, sector), fields(nodes = sector.nodes.len(), roads = sector.roads.len()))]
    pub fn merge_sector(&mut self, sector: ParsedSector) {
        let sid = self.sectors_merged as SectorId;
        for node in sector.nodes {
            self.node_to_sector.insert(node.uid, sid);
            self.nodes.insert(node.uid, node);
        }
        self.roads.extend(sector.roads);
        self.raw_prefabs.extend(sector.prefabs);
        self.raw_signs.extend(sector.signs);
        self.ferries.extend(sector.ferries);
        self.buildings.extend(sector.buildings);
        self.sectors_merged += 1;
    }

    /// Apply road-look lane counts to legacy roads (those with lanes == 0).
    ///
    /// Tries an exact lookup of `road_type_token` (= `road_type` token) in
    /// the road-look map.  On miss, falls back to bidirectional (1 lane each
    /// way) for any road that has at least one look token set, so that graph
    /// connectivity is preserved.  Sized-format roads that already carry lane
    /// data are left unchanged.
    fn apply_road_look(&mut self) {
        let mut hits = 0usize;
        let mut heuristic = 0usize;
        let mut unchanged = 0usize;
        for road in &mut self.roads {
            if road.lanes_forward != 0 || road.lanes_backward != 0 {
                // Already has lane data (sized-road format) — leave untouched.
                unchanged += 1;
                continue;
            }
            // Try exact road_look map lookup first.
            if road.road_type_token != 0 {
                if let Some(entry) = self.road_look.get(&road.road_type_token) {
                    // Phase 2e: real lane counts drive the lane OFFSET (forward =
                    // lanes_right). `.max(1)` keeps a counter-direction edge alive
                    // even for one-way carriageways (lanes_left==0) so routing
                    // connectivity matches the prior bidirectional baseline —
                    // Phase 2e changes offsets, not topology. Correct one-way
                    // enforcement is deferred to a junction/cross-sector
                    // connectivity phase.
                    road.lanes_forward = entry.lanes_right.max(1);
                    road.lanes_backward = entry.lanes_left.max(1);
                    hits += 1;
                    continue;
                }
            }
            // Fallback: road has look tokens but no matching road_look definition.
            // Treat as bidirectional (1 lane each way) — one-way enforcement requires
            // verified lane counts from road_look SII files; without them, defaulting
            // to one-way creates sink/source nodes that break graph connectivity.
            if road.road_type_token != 0 || road.look_token != 0 {
                road.lanes_forward = 1;
                road.lanes_backward = 1;
                heuristic += 1;
            } else {
                unchanged += 1;
            }
        }
        let total = self.roads.len();
        info!(
            hits,
            heuristic,
            unchanged,
            total,
            "road_look apply: {hits} exact, {heuristic} heuristic, {unchanged} unchanged"
        );
    }

    /// Build the final `MapGraph` from all merged sectors.
    #[instrument(skip(self))]
    pub fn build(mut self) -> MapGraph {
        self.apply_road_look();

        let t0 = Instant::now();

        let mut nodes: Vec<GraphNode> = self
            .nodes
            .values()
            .map(|n| GraphNode {
                uid: n.uid,
                x: n.x as f64,
                y: n.y as f64,
                z: n.z as f64,
                rotation: n.rotation,
            })
            .collect();
        nodes.sort_by_key(|n| n.uid);

        let zero_quat = nodes
            .iter()
            .filter(|n| {
                let q = n.rotation;
                q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] < 1e-6
            })
            .count();
        debug!(
            "rotation propagation: {}/{} nodes have valid quaternion",
            nodes.len() - zero_quat,
            nodes.len()
        );

        let node_lookup: HashMap<u64, &GraphNode> = nodes.iter().map(|n| (n.uid, n)).collect();

        let mut edges: Vec<GraphEdge> = Vec::new();
        let mut edge_uid: u64 = 1;

        // Phase 0d.2b: node → resolved-neighbor positions for road_dir_hint.
        // Maps each node UID to the world positions of all nodes it shares a
        // fully-resolved road with. Used below when building OrphanEndpoints.
        let mut node_neighbors: HashMap<u64, Vec<[f64; 3]>> = HashMap::new();
        for road in &self.roads {
            if let (Some(a_node), Some(b_node)) =
                (node_lookup.get(&road.node_a), node_lookup.get(&road.node_b))
            {
                node_neighbors
                    .entry(road.node_a)
                    .or_default()
                    .push([b_node.x, b_node.y, b_node.z]);
                node_neighbors
                    .entry(road.node_b)
                    .or_default()
                    .push([a_node.x, a_node.y, a_node.z]);
            }
        }

        // Phase 5.23b: orphan endpoints collected during road-edge generation.
        // An "orphan" is a road whose ONE endpoint resolves in node_lookup
        // and the other does not. Both-unresolved roads are dropped with
        // a warning (cannot match a position we don't know).
        let mut orphans: Vec<OrphanEndpoint> = Vec::new();
        let mut both_unresolved = 0usize;

        for road in &self.roads {
            let a_node = node_lookup.get(&road.node_a).copied();
            let b_node = node_lookup.get(&road.node_b).copied();
            let (a, b) = match (a_node, b_node) {
                (Some(a), Some(b)) => (a, b),
                (Some(a), None) => {
                    let pos = [a.x, a.y, a.z];
                    orphans.push(OrphanEndpoint {
                        road_uid: road.uid,
                        resolved_uid: road.node_a,
                        resolved_pos: pos,
                        missing_uid: road.node_b,
                        sector_id: self
                            .node_to_sector
                            .get(&road.node_a)
                            .copied()
                            .unwrap_or(SECTOR_ID_UNKNOWN),
                        road_dir_hint: compute_dir_hint(
                            &pos,
                            node_neighbors
                                .get(&road.node_a)
                                .map(Vec::as_slice)
                                .unwrap_or(&[]),
                        ),
                    });
                    continue;
                }
                (None, Some(b)) => {
                    let pos = [b.x, b.y, b.z];
                    orphans.push(OrphanEndpoint {
                        road_uid: road.uid,
                        resolved_uid: road.node_b,
                        resolved_pos: pos,
                        missing_uid: road.node_a,
                        sector_id: self
                            .node_to_sector
                            .get(&road.node_b)
                            .copied()
                            .unwrap_or(SECTOR_ID_UNKNOWN),
                        road_dir_hint: compute_dir_hint(
                            &pos,
                            node_neighbors
                                .get(&road.node_b)
                                .map(Vec::as_slice)
                                .unwrap_or(&[]),
                        ),
                    });
                    continue;
                }
                (None, None) => {
                    both_unresolved += 1;
                    continue;
                }
            };

            let dist = euclidean_3d(a, b);
            let speed = if road.speed_limit_kmh > 0 {
                Some(road.speed_limit_kmh as f64)
            } else {
                None
            };

            // DS8: resolve lane width from road_look map.
            let lane_width = if road.road_type_token != 0 {
                self.road_look
                    .get(&road.road_type_token)
                    .map(|e| e.lane_width_m)
                    .unwrap_or(3.75)
            } else if road.look_token != 0 {
                self.road_look
                    .get(&road.look_token)
                    .map(|e| e.lane_width_m)
                    .unwrap_or(3.75)
            } else {
                3.75_f32
            };

            // Phase 2h: resolve road_offset (median shift) from the same road_look entry.
            let road_offset_m = if road.road_type_token != 0 {
                self.road_look
                    .get(&road.road_type_token)
                    .map(|e| e.road_offset_m)
                    .unwrap_or(0.0)
            } else if road.look_token != 0 {
                self.road_look
                    .get(&road.look_token)
                    .map(|e| e.road_offset_m)
                    .unwrap_or(0.0)
            } else {
                0.0_f32
            };

            if road.lanes_forward > 0 {
                edges.push(GraphEdge {
                    uid: edge_uid,
                    from: road.node_a,
                    to: road.node_b,
                    distance_m: dist,
                    speed_limit_kmh: speed,
                    lanes: road.lanes_forward,
                    direction: "forward".into(),
                    dlc_guard: road.dlc_guard,
                    is_hidden: road.is_hidden,
                    gps_avoid: road.gps_avoid,
                    road_look_token: road.road_type_token,
                    lanes_opposite: road.lanes_backward,
                    lane_width_m: lane_width,
                    road_offset_m,
                });
                edge_uid += 1;
            }

            if road.lanes_backward > 0 {
                edges.push(GraphEdge {
                    uid: edge_uid,
                    from: road.node_b,
                    to: road.node_a,
                    distance_m: dist,
                    speed_limit_kmh: speed,
                    lanes: road.lanes_backward,
                    direction: "backward".into(),
                    dlc_guard: road.dlc_guard,
                    is_hidden: road.is_hidden,
                    gps_avoid: road.gps_avoid,
                    road_look_token: road.road_type_token,
                    lanes_opposite: road.lanes_forward,
                    lane_width_m: lane_width,
                    road_offset_m,
                });
                edge_uid += 1;
            }

            // Bidirectional unknown
            if road.lanes_forward == 0 && road.lanes_backward == 0 {
                for (from, to, dir) in [
                    (road.node_a, road.node_b, "bidirectional_unknown"),
                    (road.node_b, road.node_a, "bidirectional_unknown"),
                ] {
                    edges.push(GraphEdge {
                        uid: edge_uid,
                        from,
                        to,
                        distance_m: dist,
                        speed_limit_kmh: speed,
                        lanes: 1,
                        direction: dir.into(),
                        dlc_guard: road.dlc_guard,
                        is_hidden: road.is_hidden,
                        gps_avoid: road.gps_avoid,
                        road_look_token: road.road_type_token,
                        lanes_opposite: 1,
                        lane_width_m: lane_width,
                        road_offset_m,
                    });
                    edge_uid += 1;
                }
            }
        }

        // Phase 5.25a — Building-derived edges.
        //
        // Buildings (Type 2) carry a `Node` and a `ForwardNode` that
        // delineate the building strip along a road. Phase 5.24 audit
        // showed ~9% of 574k singleton nodes are referenced by ignored
        // item types in base_map; buildings are one of the largest
        // contributors. We generate a bidirectional `direction="building"`
        // edge for each pair where both endpoints resolve, recovering
        // connectivity that the road parser alone misses.
        let mut buildings_total = 0usize;
        let mut buildings_zero_uid = 0usize;
        let mut buildings_self_loop = 0usize;
        let mut buildings_one_unresolved = 0usize;
        let mut buildings_both_unresolved = 0usize;
        let mut building_edges_count = 0usize;
        for b in &self.buildings {
            buildings_total += 1;
            if b.node_uid == 0 || b.forward_node_uid == 0 {
                buildings_zero_uid += 1;
                continue;
            }
            if b.node_uid == b.forward_node_uid {
                buildings_self_loop += 1;
                continue;
            }
            let a_node = node_lookup.get(&b.node_uid).copied();
            let f_node = node_lookup.get(&b.forward_node_uid).copied();
            let (a, f) = match (a_node, f_node) {
                (Some(a), Some(f)) => (a, f),
                (Some(_), None) | (None, Some(_)) => {
                    buildings_one_unresolved += 1;
                    continue;
                }
                (None, None) => {
                    buildings_both_unresolved += 1;
                    continue;
                }
            };
            let dist = euclidean_3d(a, f);
            for (from, to) in [
                (b.node_uid, b.forward_node_uid),
                (b.forward_node_uid, b.node_uid),
            ] {
                edges.push(GraphEdge {
                    uid: edge_uid,
                    from,
                    to,
                    distance_m: dist,
                    speed_limit_kmh: None,
                    lanes: 1,
                    direction: "building".into(),
                    dlc_guard: 0,
                    is_hidden: false,
                    gps_avoid: false,
                    road_look_token: 0,
                    lanes_opposite: 0,
                    lane_width_m: 3.75,
                    road_offset_m: 0.0,
                });
                edge_uid += 1;
                building_edges_count += 1;
            }
        }
        info!(
            "Building edges: {} from {} buildings ({} both-resolved); skipped: {} zero-uid, {} self-loop, {} one-unresolved, {} both-unresolved",
            building_edges_count,
            buildings_total,
            building_edges_count / 2,
            buildings_zero_uid,
            buildings_self_loop,
            buildings_one_unresolved,
            buildings_both_unresolved
        );

        // Phase 5.10' — Prefab-derived edges.
        //
        // In ETS2, prefabs (intersections, junctions, ramps) are the glue
        // between roads: each prefab carries a list of `connected_node_uids`
        // that the surrounding roads attach to.  Without an explicit edge
        // between those nodes, two roads meeting at the same junction end up
        // in different connected components — which is exactly what was
        // happening before this commit (largest CC = 217 nodes, 79 %
        // isolated).
        //
        // We treat each prefab as a fully-connected clique of its valid
        // node UIDs (those that resolve in `node_lookup`).  Each unordered
        // pair becomes two directed edges (`from→to` and `to→from`) so the
        // graph stays directional like the road edges.  Distance uses the
        // 3-D Euclidean of the two nodes — prefab geometries are small
        // enough that this is a reasonable proxy for in-prefab travel.
        for raw in &self.raw_prefabs {
            let valid: Vec<u64> = raw
                .nodes
                .iter()
                .copied()
                .filter(|uid| node_lookup.contains_key(uid))
                .collect();
            for i in 0..valid.len() {
                let Some(a) = node_lookup.get(&valid[i]) else {
                    continue;
                };
                for j in (i + 1)..valid.len() {
                    let Some(b) = node_lookup.get(&valid[j]) else {
                        continue;
                    };
                    let dist = euclidean_3d(a, b);
                    for (from, to) in [(valid[i], valid[j]), (valid[j], valid[i])] {
                        edges.push(GraphEdge {
                            uid: edge_uid,
                            from,
                            to,
                            distance_m: dist,
                            speed_limit_kmh: None,
                            lanes: 1,
                            direction: "prefab".into(),
                            dlc_guard: 0,
                            is_hidden: false,
                            gps_avoid: false,
                            road_look_token: 0,
                            lanes_opposite: 0,
                            lane_width_m: 3.75,
                            road_offset_m: 0.0,
                        });
                        edge_uid += 1;
                    }
                }
            }
        }

        // Phase 5.22 — Ferry-derived clique edges.
        //
        // Ferry items (Type 19) carry a `port_token` (hash of the port unit
        // name from `/def/ferry.sii`). All ferries sharing the same
        // `port_token` belong to the same route — bidirectional edges form
        // a fully-connected clique between their nodes. This is the only
        // *natural* cross-sector connectivity in v907 maps.
        let mut ferry_groups: HashMap<u64, Vec<u64>> = HashMap::new();
        let mut ferries_skipped_zero = 0usize;
        let mut ferries_skipped_missing_node = 0usize;
        for ferry in &self.ferries {
            if ferry.port_token == 0 || ferry.node_uid == 0 {
                ferries_skipped_zero += 1;
                continue;
            }
            if !node_lookup.contains_key(&ferry.node_uid) {
                warn!(
                    "Ferry {} references node {} not in lookup",
                    ferry.uid, ferry.node_uid
                );
                ferries_skipped_missing_node += 1;
                continue;
            }
            ferry_groups
                .entry(ferry.port_token)
                .or_default()
                .push(ferry.node_uid);
        }
        let ferry_group_count = ferry_groups.len();
        let mut ferry_edges_count = 0usize;
        let mut ferry_groups_with_edges = 0usize;
        for nodes in ferry_groups.values() {
            if nodes.len() < 2 {
                continue;
            }
            ferry_groups_with_edges += 1;
            for i in 0..nodes.len() {
                let Some(a) = node_lookup.get(&nodes[i]) else {
                    continue;
                };
                for j in (i + 1)..nodes.len() {
                    let Some(b) = node_lookup.get(&nodes[j]) else {
                        continue;
                    };
                    let dist = euclidean_3d(a, b);
                    for (from, to) in [(nodes[i], nodes[j]), (nodes[j], nodes[i])] {
                        edges.push(GraphEdge {
                            uid: edge_uid,
                            from,
                            to,
                            distance_m: dist,
                            speed_limit_kmh: None,
                            lanes: 1,
                            direction: "ferry".into(),
                            dlc_guard: 0,
                            is_hidden: false,
                            gps_avoid: false,
                            road_look_token: 0,
                            lanes_opposite: 0,
                            lane_width_m: 3.75,
                            road_offset_m: 0.0,
                        });
                        edge_uid += 1;
                        ferry_edges_count += 1;
                    }
                }
            }
        }
        info!(
            "Ferry edges: {} from {} groups ({} with >=2 nodes); ferries skipped: {} zero, {} missing-node",
            ferry_edges_count,
            ferry_group_count,
            ferry_groups_with_edges,
            ferries_skipped_zero,
            ferries_skipped_missing_node
        );

        // Phase 5.23b — Cross-sector spatial matching (Pass 1 STRICT).
        //
        // For each orphan endpoint (road where exactly one endpoint resolved
        // in node_lookup), search a 50 m XZ radius for a node in a DIFFERENT
        // sector that lies within 5 m vertical tolerance and within 50 m 3D.
        // Bidirectional cross_sector_spatial_high edges are generated for
        // each match. Multi-pass + heading filter land in 5.23c/5.23d.
        let total_road_endpoints = self.roads.len() * 2;
        info!(
            "Spatial match input: {} orphan endpoints ({:.1}% of {} road endpoints), {} roads with both endpoints unresolved (dropped)",
            orphans.len(),
            if total_road_endpoints > 0 {
                100.0 * orphans.len() as f64 / total_road_endpoints as f64
            } else {
                0.0
            },
            total_road_endpoints,
            both_unresolved
        );

        let spatial_index = build_spatial_index(&nodes, &self.node_to_sector, DEFAULT_CELL_SIZE);
        info!(
            "Spatial index: {} nodes in {} cells (cell_size={}m)",
            spatial_index.total_nodes(),
            spatial_index.cell_count(),
            DEFAULT_CELL_SIZE
        );

        let pass1 = pass1_strict_config();
        let mut matched_pairs: HashSet<(u64, u64)> = HashSet::new();
        let mut pass1_matches = 0usize;
        let mut pass1_high = 0usize;
        let mut pass1_medium = 0usize;
        let mut pass1_low = 0usize;
        let mut pass1_edges = 0usize;

        for orphan in &orphans {
            let raw_candidates = query_circle(&spatial_index, &orphan.resolved_pos, pass1.max_dist);
            let mut filtered: Vec<(&_, f64)> = Vec::new();
            for cand in raw_candidates {
                if let Some(d) = apply_filters(orphan, cand, &pass1) {
                    filtered.push((cand, d));
                }
            }
            let Some((best, dist, conf)) = select_best_match(&mut filtered, pass1.level) else {
                continue;
            };
            // Self-match guard (orphan's own resolved node).
            if best.uid == orphan.resolved_uid {
                continue;
            }
            let from = orphan.resolved_uid;
            let to = best.uid;
            let pair = if from < to { (from, to) } else { (to, from) };
            if !matched_pairs.insert(pair) {
                continue;
            }
            pass1_matches += 1;
            match conf {
                crate::spatial_match::ConfidenceLevel::High => pass1_high += 1,
                crate::spatial_match::ConfidenceLevel::Medium => pass1_medium += 1,
                crate::spatial_match::ConfidenceLevel::Low => pass1_low += 1,
            }
            for (f, t) in [(from, to), (to, from)] {
                edges.push(GraphEdge {
                    uid: edge_uid,
                    from: f,
                    to: t,
                    distance_m: dist,
                    speed_limit_kmh: None,
                    lanes: 1,
                    direction: conf.to_direction_string().into(),
                    dlc_guard: 0,
                    is_hidden: false,
                    gps_avoid: false,
                    road_look_token: 0,
                    lanes_opposite: 0,
                    lane_width_m: 3.75,
                    road_offset_m: 0.0,
                });
                edge_uid += 1;
                pass1_edges += 1;
            }
        }
        info!(
            "Pass 1 (STRICT 50m/5m): {} unique matches ({} edges) from {} orphans — confidence: high={} medium={} low={}",
            pass1_matches, pass1_edges, orphans.len(), pass1_high, pass1_medium, pass1_low
        );

        // Phase 0d.2b — Cross-sector spatial matching (Pass 2 WIDE).
        //
        // 200m XZ radius, 15m Y-tolerance, virtual-sector adjacency guard
        // (prevents cross-continent false matches from isolated DLC islands).
        // Runs over ALL orphans; the `matched_pairs` dedup set already
        // contains Pass-1 pairs, so no orphan that was matched in Pass 1 can
        // produce a duplicate edge here.
        let pass2 = pass2_config();
        let mut pass2_matches = 0usize;
        let mut pass2_edges = 0usize;

        for orphan in &orphans {
            let raw_candidates = query_circle(&spatial_index, &orphan.resolved_pos, pass2.max_dist);
            let mut filtered: Vec<(&_, f64)> = Vec::new();
            for cand in raw_candidates {
                if let Some(d) = apply_filters(orphan, cand, &pass2) {
                    filtered.push((cand, d));
                }
            }
            let Some((best, dist, _conf)) = select_best_match(&mut filtered, pass2.level) else {
                continue;
            };
            if best.uid == orphan.resolved_uid {
                continue;
            }
            let from = orphan.resolved_uid;
            let to = best.uid;
            let pair = if from < to { (from, to) } else { (to, from) };
            if !matched_pairs.insert(pair) {
                continue;
            }
            pass2_matches += 1;
            for (f, t) in [(from, to), (to, from)] {
                edges.push(GraphEdge {
                    uid: edge_uid,
                    from: f,
                    to: t,
                    distance_m: dist,
                    speed_limit_kmh: None,
                    lanes: 1,
                    direction: "cross_sector_pass2".into(),
                    dlc_guard: 0,
                    is_hidden: false,
                    gps_avoid: false,
                    road_look_token: 0,
                    lanes_opposite: 0,
                    road_offset_m: 0.0,
                    lane_width_m: 3.75,
                });
                edge_uid += 1;
                pass2_edges += 1;
            }
        }
        info!(
            "Pass 2 (WIDE 200m/15m + adj-sector): {} unique matches ({} edges) from {} orphans",
            pass2_matches,
            pass2_edges,
            orphans.len()
        );

        // Phase 6.3 — Cross-sector boundary stitch (road-endpoint nodes, not orphan-driven).
        let road_endpoint_uids: HashSet<u64> = self
            .roads
            .iter()
            .flat_map(|r| {
                let mut uids = Vec::new();
                if node_lookup.contains_key(&r.node_a) {
                    uids.push(r.node_a);
                }
                if node_lookup.contains_key(&r.node_b) {
                    uids.push(r.node_b);
                }
                uids
            })
            .collect();

        let mut edge_pairs: HashSet<(u64, u64)> = HashSet::new();
        for e in &edges {
            let pair = if e.from < e.to {
                (e.from, e.to)
            } else {
                (e.to, e.from)
            };
            edge_pairs.insert(pair);
        }

        let (boundary_edges, _edge_uid, boundary_stats) = stitch_cross_sector_boundary(
            &self.roads,
            &node_lookup,
            &self.node_to_sector,
            &road_endpoint_uids,
            &spatial_index,
            &edge_pairs,
            edge_uid,
        );
        edges.extend(boundary_edges);

        info!(
            "Boundary stitch: {} unique matches ({} edges) from {} candidates — rejected: {} heading, {} distance, {} same-sector, {} already-connected, {} no-heading",
            boundary_stats.matches,
            boundary_stats.edges,
            boundary_stats.candidates,
            boundary_stats.rejected_heading,
            boundary_stats.rejected_distance,
            boundary_stats.rejected_same_sector,
            boundary_stats.rejected_already_connected,
            boundary_stats.rejected_no_heading,
        );

        // Process prefabs and generate PrefabAiPaths
        let mut prefabs: Vec<Prefab> = Vec::with_capacity(self.raw_prefabs.len());
        let mut prefab_instances: Vec<PrefabInstance> = Vec::with_capacity(self.raw_prefabs.len());
        let mut prefab_ai_paths: Vec<PrefabAiPath> = Vec::new();
        let mut total_chain_broken = 0usize;
        let mut used_descriptors: HashMap<u64, PrefabDescriptor> = HashMap::new();

        // Build a quick node position lookup for origin derivation
        let node_pos_map: HashMap<u64, [f32; 3]> = nodes
            .iter()
            .map(|n| (n.uid, [n.x as f32, n.y as f32, n.z as f32]))
            .collect();

        for raw in &self.raw_prefabs {
            let valid_nodes: Vec<u64> = raw
                .nodes
                .iter()
                .copied()
                .filter(|uid| node_lookup.contains_key(uid))
                .collect();

            let origin_pos = if let Some(first) = valid_nodes.first() {
                if let Some(n) = node_lookup.get(first) {
                    [n.x as f32, n.y as f32, n.z as f32]
                } else {
                    [0.0f32; 3]
                }
            } else {
                [0.0f32; 3]
            };

            let origin_rot = [0.0f32, 0.0f32, 0.0f32, 1.0f32];

            prefab_instances.push(PrefabInstance {
                uid: raw.uid,
                token: raw.template_token,
                origin_pos,
                origin_rot,
                node_uids: valid_nodes.clone(),
            });

            // Try to look up the PPD descriptor
            if let Some(desc) = self.ppd_descriptors.get(&raw.template_token) {
                used_descriptors
                    .entry(raw.template_token)
                    .or_insert_with(|| desc.clone());

                // Generate AI paths — pass full raw.nodes + origin so
                // build_prefab_ai_paths can apply ETS2LA-style rotation.
                let (paths, broken) = build_prefab_ai_paths(
                    desc,
                    &origin_pos,
                    &origin_rot,
                    &raw.nodes,
                    raw.origin_node_index as usize,
                    &node_pos_map,
                );
                prefab_ai_paths.extend(paths);
                total_chain_broken += broken;
            }

            prefabs.push(Prefab {
                uid: raw.uid,
                template_token: raw.template_token,
                connected_node_uids: raw.nodes.clone(),
            });
        }

        let ai_path_count = prefab_ai_paths.len();
        let descriptor_count = used_descriptors.len();
        if descriptor_count > 0 {
            info!(
                "Generated {} AI paths from {} PPD descriptors for {} prefab instances ({} chain-breaks)",
                ai_path_count,
                descriptor_count,
                prefab_instances.len(),
                total_chain_broken,
            );
        }
        self.ppd_chain_broken = total_chain_broken;

        // Attach signs to nearest nodes
        let signs = crate::signs::attach_signs_to_nodes(&self.raw_signs, &nodes);

        let elapsed_ms = t0.elapsed().as_secs_f64() * 1000.0;
        info!(
            "Graph built: {} nodes, {} edges, {} signs, {} prefabs in {:.1} ms",
            nodes.len(),
            edges.len(),
            signs.len(),
            prefabs.len(),
            elapsed_ms
        );

        MapGraph {
            stats: BuildStats {
                node_count: nodes.len(),
                edge_count: edges.len(),
                sign_count: signs.len(),
                prefab_count: prefabs.len(),
                sectors_merged: self.sectors_merged,
                build_time_ms: elapsed_ms,
                ppd_files_attempted: self.ppd_files_attempted,
                ppd_files_loaded: self.ppd_files_loaded,
                ppd_files_failed: self.ppd_files_failed,
                ppd_total_nav_curves: self.ppd_total_nav_curves,
                ppd_failed_token_miss: self.ppd_failed_token_miss,
                ppd_failed_archive_miss: self.ppd_failed_archive_miss,
                ppd_failed_parse_err: self.ppd_failed_parse_err,
                ppd_chain_broken: self.ppd_chain_broken,
            },
            nodes,
            edges,
            signs,
            prefabs,
            prefab_ai_paths,
            prefab_instances,
            prefab_descriptors: used_descriptors,
        }
    }

    // -----------------------------------------------------------------------
    // Audit helpers — read-only, must be called BEFORE build() consumes self.
    // -----------------------------------------------------------------------

    /// All roads accumulated so far (before graph build).
    pub fn roads(&self) -> &[RawRoad] {
        &self.roads
    }

    /// All raw nodes accumulated so far.
    pub fn raw_nodes(&self) -> &HashMap<u64, RawNode> {
        &self.nodes
    }

    /// All raw prefabs accumulated so far (before graph build).
    pub fn raw_prefabs(&self) -> &[RawPrefab] {
        &self.raw_prefabs
    }

    /// Find the nearest node to `(x, z)` in XZ-plane within `max_dist` metres.
    /// Returns `(uid, distance_m)` or `None` if nothing is within range.
    pub fn find_nearest_node(&self, x: f32, z: f32, max_dist: f32) -> Option<(u64, f32)> {
        let max_dist_sq = (max_dist as f64).powi(2);
        let mut best: Option<(u64, f64)> = None;
        for node in self.nodes.values() {
            let dx = node.x as f64 - x as f64;
            let dz = node.z as f64 - z as f64;
            let d2 = dx * dx + dz * dz;
            if d2 <= max_dist_sq && best.is_none_or(|(_, bd)| d2 < bd) {
                best = Some((node.uid, d2));
            }
        }
        best.map(|(uid, d2)| (uid, d2.sqrt() as f32))
    }

    /// Count accumulated roads that reference `uid` as either endpoint.
    pub fn roads_referencing_node(&self, uid: u64) -> usize {
        self.roads
            .iter()
            .filter(|r| r.node_a == uid || r.node_b == uid)
            .count()
    }

    /// Classify all accumulated roads by graph-level node-resolution status.
    ///
    /// This is a pure read — it does NOT modify `self` and can be called
    /// immediately before `build()` (which moves `self`).
    pub fn analyze_roads_for_audit(&self) -> RoadAuditResult {
        let mut both_unresolved = Vec::new();
        let mut one_unresolved = Vec::new();
        for road in &self.roads {
            let a = self.nodes.get(&road.node_a);
            let b = self.nodes.get(&road.node_b);
            match (a, b) {
                (None, None) => both_unresolved.push(road.clone()),
                (Some(n), None) => one_unresolved.push((
                    road.clone(),
                    road.node_a,
                    [n.x as f64, n.y as f64, n.z as f64],
                )),
                (None, Some(n)) => one_unresolved.push((
                    road.clone(),
                    road.node_b,
                    [n.x as f64, n.y as f64, n.z as f64],
                )),
                (Some(_), Some(_)) => {}
            }
        }
        RoadAuditResult {
            both_unresolved,
            one_unresolved,
        }
    }
}

/// Average unit-vector from `resolved_pos` toward each `neighbors` position.
/// Returns `None` when there are no neighbors or all are degenerate (same pos).
/// Used to populate `OrphanEndpoint::road_dir_hint` for the optional heading
/// filter in Pass 2 and later passes.
fn compute_dir_hint(resolved_pos: &[f64; 3], neighbors: &[[f64; 3]]) -> Option<[f64; 3]> {
    let mut sx = 0.0_f64;
    let mut sy = 0.0_f64;
    let mut sz = 0.0_f64;
    let mut count = 0usize;
    for nb in neighbors {
        let dx = nb[0] - resolved_pos[0];
        let dy = nb[1] - resolved_pos[1];
        let dz = nb[2] - resolved_pos[2];
        let len = (dx * dx + dy * dy + dz * dz).sqrt();
        if len > 0.001 {
            sx += dx / len;
            sy += dy / len;
            sz += dz / len;
            count += 1;
        }
    }
    if count == 0 {
        return None;
    }
    let mag = (sx * sx + sy * sy + sz * sz).sqrt();
    if mag < 0.001 {
        return None;
    }
    Some([sx / mag, sy / mag, sz / mag])
}

fn euclidean_3d(a: &GraphNode, b: &GraphNode) -> f64 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    let dz = a.z - b.z;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

// ---------------------------------------------------------------------------
// Prefab AI path generation
// ---------------------------------------------------------------------------

/// Sample a NavCurve into `n` world-space points using Hermite interpolation.
fn sample_nav_curve(
    nc: &NavCurve,
    origin: &[f32; 3],
    _origin_rot: &[f32; 4],
    n: usize,
) -> Vec<[f32; 3]> {
    let ox = origin[0];
    let oy = origin[1];
    let oz = origin[2];

    let p0: [f32; 3] = [
        nc.start_position[0] + ox,
        nc.start_position[1] + oy,
        nc.start_position[2] + oz,
    ];
    let p1: [f32; 3] = [
        nc.end_position[0] + ox,
        nc.end_position[1] + oy,
        nc.end_position[2] + oz,
    ];

    let len = nc.length.max(0.001);

    // Tangent from rotation quaternion: rotate FORWARD by q, scaled by len.
    let m0: SplineVec3 = quat_rotate_vec(nc.start_rotation, FORWARD * len);
    let m1: SplineVec3 = quat_rotate_vec(nc.end_rotation, FORWARD * len);

    let nf = n as f32;
    let mut pts = Vec::with_capacity(n);

    for i in 0..n {
        let t = i as f32 / nf.max(1.0);
        let t2 = t * t;
        let t3 = t2 * t;

        let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
        let h10 = t3 - 2.0 * t2 + t;
        let h01 = -2.0 * t3 + 3.0 * t2;
        let h11 = t3 - t2;

        let x = h00 * p0[0] + h10 * m0.x + h01 * p1[0] + h11 * m1.x;
        let y = h00 * p0[1] + h10 * m0.y + h01 * p1[1] + h11 * m1.y;
        let z = h00 * p0[2] + h10 * m0.z + h01 * p1[2] + h11 * m1.z;

        pts.push([x, y, z]);
    }

    if n > 1 {
        // Snap first and last to endpoints
        pts[0] = p0;
        pts[n - 1] = p1;
    }

    pts
}

/// Build PrefabAiPath records by walking NavCurves within a PrefabDescriptor,
/// mapping control nodes to world-space GraphNode UIDs.
///
/// `all_node_uids` is the **full**, unfiltered `RawPrefab.nodes` list.
/// `origin_node_index` rotates it into PPD-local ordering:
///   world_uid_for(control_nodes[i]) = all_node_uids[(origin + i) % n]
/// This mirrors ETS2LA's `rotate_right(node_uids, origin_node_index)`.
/// `node_lookup` is used to skip UIDs not yet resolved (cross-sector nodes).
fn build_prefab_ai_paths(
    desc: &PrefabDescriptor,
    origin_pos: &[f32; 3],
    _origin_rot: &[f32; 4],
    all_node_uids: &[u64],
    origin_node_index: usize,
    node_lookup: &HashMap<u64, [f32; 3]>,
) -> (Vec<PrefabAiPath>, usize) {
    let mut paths = Vec::new();
    let mut chain_broken = 0usize;

    let n = all_node_uids.len();
    if n == 0 {
        return (paths, chain_broken);
    }

    // For each ControlNode in the PPD, map its output lines to paths.
    // Each output line (NavCurve index) from a ControlNode represents a path
    // starting at that control node.
    for (cn_idx, cn) in desc.control_nodes.iter().enumerate() {
        // ETS2LA-style: rotate all_node_uids by origin_node_index
        // so that control_nodes[cn_idx] ↔ all_node_uids[(origin + cn_idx) % n]
        let rotated_from = (origin_node_index + cn_idx) % n;
        let from_node_uid = all_node_uids[rotated_from];
        if !node_lookup.contains_key(&from_node_uid) {
            continue; // cross-sector node not yet resolved
        }

        // Walk each output line (NavCurve) from this ControlNode
        for &curve_idx_raw in &cn.output_lines {
            if curve_idx_raw < 0 {
                continue;
            }
            let start_curve_idx = curve_idx_raw as usize;
            if start_curve_idx >= desc.nav_curves.len() {
                continue;
            }

            // Follow the curve chain to find the terminating node
            let (to_node_idx, curve_indices) = trace_curve_chain_to_node(
                &desc.control_nodes,
                &desc.nav_curves,
                cn_idx as u32,
                start_curve_idx,
                64, // max depth to prevent infinite loops
            );

            if curve_indices.is_empty() {
                chain_broken += 1;
            }

            if to_node_idx as usize >= desc.control_nodes.len() {
                continue;
            }

            // Apply same rotation for the destination control node
            let rotated_to = (origin_node_index + to_node_idx as usize) % n;
            let to_node_uid = all_node_uids[rotated_to];
            if !node_lookup.contains_key(&to_node_uid) {
                continue; // cross-sector destination node
            }

            // Don't create self-loops
            if from_node_uid == to_node_uid {
                continue;
            }

            // Sample all curves in the chain into world-space points
            let mut spline_points: Vec<[f32; 3]> = Vec::new();
            let mut total_length = 0.0f32;

            for (i, &ci) in curve_indices.iter().enumerate() {
                if ci >= desc.nav_curves.len() {
                    continue;
                }
                let nc = &desc.nav_curves[ci];
                let n_pts = 16usize;
                let pts = sample_nav_curve(nc, origin_pos, &[0.0, 0.0, 0.0, 1.0], n_pts);

                for (j, pt) in pts.iter().enumerate() {
                    if i > 0 && j == 0 {
                        continue; // skip duplicate at curve junction
                    }
                    spline_points.push(*pt);
                }
                total_length += nc.length;
            }

            if spline_points.is_empty() {
                continue;
            }

            // Get meta from first curve
            let first_curve = &desc.nav_curves[start_curve_idx];
            let last_curve_idx = curve_indices.last().copied().unwrap_or(start_curve_idx);
            let last_curve =
                &desc.nav_curves[last_curve_idx.min(desc.nav_curves.len().saturating_sub(1))];

            paths.push(PrefabAiPath {
                from_node_uid,
                to_node_uid,
                start_lane_idx: first_curve.leads_to.start_lane,
                end_lane_idx: first_curve.leads_to.end_lane,
                spline_points,
                length_m: total_length,
                speed_kmh: None,
                blinker: Some(ppd::Blinker::from_nav_curve_flags(first_curve.flags)),
                curve_indices: curve_indices.iter().map(|&i| i as u16).collect(),
                semaphore_id: if first_curve.semaphore_id >= 0 {
                    Some(first_curve.semaphore_id)
                } else {
                    None
                },
                start_rotation: first_curve.start_rotation,
                end_rotation: last_curve.end_rotation,
            });
        }
    }

    (paths, chain_broken)
}

/// Follow a NavCurve chain from a start curve to the next ControlNode.
/// Returns (end_control_node_index, curve_indices_visited).
fn trace_curve_chain_to_node(
    control_nodes: &[ppd::ControlNode],
    nav_curves: &[NavCurve],
    start_node: u32,
    start_curve: usize,
    max_depth: usize,
) -> (u32, Vec<usize>) {
    let mut visited: HashSet<usize> = HashSet::new();
    let mut curve_indices: Vec<usize> = Vec::new();

    let mut current_curve = start_curve;

    for _depth in 0..max_depth {
        if current_curve >= nav_curves.len() {
            break;
        }
        if !visited.insert(current_curve) {
            break; // cycle detected
        }
        curve_indices.push(current_curve);

        let nc = &nav_curves[current_curve];

        // Check if this curve terminates at a different ControlNode.
        // Skip same-node termination here — it will be caught on dead-end below,
        // allowing the chain to continue through intermediate same-node entries.
        let end_node = nc.leads_to.end_node as u32;
        if end_node as usize != start_node as usize && end_node < control_nodes.len() as u32 {
            return (end_node, curve_indices);
        }

        // Follow next_lines to continue the chain
        let mut found_next = false;
        for &next_raw in &nc.next_lines {
            if next_raw < 0 {
                continue;
            }
            let next_idx = next_raw as usize;
            if next_idx < nav_curves.len() && !visited.contains(&next_idx) {
                current_curve = next_idx;
                found_next = true;
                break;
            }
        }
        if !found_next {
            // Chain dead-ends here. If the last curve has a valid end_node
            // (including same-node for roundabout exits), use it.
            // Self-loops are already filtered in build_prefab_ai_paths.
            if end_node < control_nodes.len() as u32 {
                return (end_node, curve_indices);
            }
            break;
        }
    }

    (start_node, Vec::new())
}

// ---------------------------------------------------------------------------
// SplineIndex helpers
// ---------------------------------------------------------------------------

impl MapGraph {
    /// Generate HermiteSegments from all PrefabAiPaths for SplineIndex
    /// consumption. Each path's spline_points are decomposed into consecutive
    /// segments usable by the lane-follower's nearest/heading queries.
    /// Build HermiteSegments and per-segment DS7 metadata from all PrefabAiPath NavCurves.
    ///
    /// Each segment gets `SegmentMetadata { is_prefab: true, lane_offset_right_m: 0.0, … }`
    /// so the lane-follower applies no lateral offset on NavCurve segments (they already
    /// sit at lane-centre).
    pub fn prefab_hermite_segments_with_metadata(
        &self,
    ) -> (
        Vec<crate::spline::HermiteSegment>,
        Vec<Option<crate::spline::SegmentMetadata>>,
    ) {
        let mut segments = Vec::new();
        let mut metadata = Vec::new();
        for path in &self.prefab_ai_paths {
            let pts = &path.spline_points;
            if pts.len() < 2 {
                continue;
            }
            let p0 = crate::spline::Vec3::new(pts[0][0], pts[0][1], pts[0][2]);
            let p1 = crate::spline::Vec3::new(
                pts[pts.len() - 1][0],
                pts[pts.len() - 1][1],
                pts[pts.len() - 1][2],
            );
            let chord_len = (p1 - p0).length();
            if chord_len < 1e-4 {
                continue;
            }
            let m0 = crate::spline::quat_rotate_vec(path.start_rotation, crate::spline::FORWARD)
                * chord_len;
            let m1 = crate::spline::quat_rotate_vec(path.end_rotation, crate::spline::FORWARD)
                * chord_len;
            segments.push(crate::spline::HermiteSegment {
                p0,
                p1,
                m0,
                m1,
                length_m: path.length_m,
                from_uid: path.from_node_uid,
                to_uid: path.to_node_uid,
                edge_uid: 0,
            });
            metadata.push(Some(crate::spline::SegmentMetadata {
                lanes_in_direction: 1,
                lanes_opposite: 0,
                lanes_total: 1,
                lane_width_m: 3.75,
                lane_offset_right_m: 0.0,
                road_offset_m: 0.0,
                road_look_token: 0,
                is_prefab: true,
            }));
        }
        (segments, metadata)
    }

    /// Convenience wrapper — returns segments only (drops metadata).
    pub fn prefab_hermite_segments(&self) -> Vec<crate::spline::HermiteSegment> {
        self.prefab_hermite_segments_with_metadata().0
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sector::{ParsedSector, RawNode, RawRoad};

    fn make_sector(
        nodes: Vec<RawNode>,
        roads: Vec<RawRoad>,
        prefabs: Vec<RawPrefab>,
    ) -> ParsedSector {
        ParsedSector {
            nodes,
            roads,
            prefabs,
            signs: vec![],
            ferries: vec![],
            buildings: vec![],
            recovered_nodes_count: 0,
        }
    }

    #[test]
    fn empty_build_yields_empty_graph() {
        let g = GraphBuilder::new().build();
        assert!(g.nodes.is_empty());
        assert!(g.edges.is_empty());
    }

    #[test]
    fn two_nodes_one_bidirectional_road() {
        let mut b = GraphBuilder::new();
        b.merge_sector(make_sector(
            vec![
                RawNode {
                    uid: 1,
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    rotation: [0.0; 4],
                    forward_item_uid: 0,
                    backward_item_uid: 0,
                },
                RawNode {
                    uid: 2,
                    x: 100.0,
                    y: 0.0,
                    z: 0.0,
                    rotation: [0.0; 4],
                    forward_item_uid: 0,
                    backward_item_uid: 0,
                },
            ],
            vec![RawRoad {
                uid: 10,
                node_a: 1,
                node_b: 2,
                speed_limit_kmh: 80,
                lanes_forward: 1,
                lanes_backward: 1,
                look_token: 0,
                dlc_guard: 0,
                is_hidden: false,
                gps_avoid: false,
                road_type_token: 0,
            }],
            vec![],
        ));
        let g = b.build();
        assert_eq!(g.nodes.len(), 2);
        assert_eq!(g.edges.len(), 2); // forward + backward
        assert!(g.edges.iter().any(|e| e.direction == "forward"));
        assert!(g.edges.iter().any(|e| e.direction == "backward"));
        assert!((g.edges[0].distance_m - 100.0).abs() < 0.01);
    }

    #[test]
    fn duplicate_nodes_across_sectors_deduped() {
        let mut b = GraphBuilder::new();
        b.merge_sector(make_sector(
            vec![RawNode {
                uid: 1,
                x: 0.0,
                y: 0.0,
                z: 0.0,
                rotation: [0.0; 4],
                forward_item_uid: 0,
                backward_item_uid: 0,
            }],
            vec![],
            vec![],
        ));
        b.merge_sector(make_sector(
            vec![RawNode {
                uid: 1,
                x: 5.0,
                y: 0.0,
                z: 0.0,
                rotation: [0.0; 4],
                forward_item_uid: 0,
                backward_item_uid: 0,
            }], // same UID, different pos
            vec![],
            vec![],
        ));
        let g = b.build();
        assert_eq!(g.nodes.len(), 1);
        // Last writer wins
        assert_eq!(g.nodes[0].x, 5.0);
    }

    #[test]
    fn road_with_missing_node_is_skipped() {
        let mut b = GraphBuilder::new();
        b.merge_sector(make_sector(
            vec![RawNode {
                uid: 1,
                x: 0.0,
                y: 0.0,
                z: 0.0,
                rotation: [0.0; 4],
                forward_item_uid: 0,
                backward_item_uid: 0,
            }],
            vec![RawRoad {
                uid: 10,
                node_a: 1,
                node_b: 999, // 999 missing
                speed_limit_kmh: 50,
                lanes_forward: 1,
                lanes_backward: 0,
                look_token: 0,
                dlc_guard: 0,
                is_hidden: false,
                gps_avoid: false,
                road_type_token: 0,
            }],
            vec![],
        ));
        let g = b.build();
        assert!(g.edges.is_empty());
    }

    #[test]
    fn unknown_speed_limit_zero_becomes_none() {
        let mut b = GraphBuilder::new();
        b.merge_sector(make_sector(
            vec![
                RawNode {
                    uid: 1,
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    rotation: [0.0; 4],
                    forward_item_uid: 0,
                    backward_item_uid: 0,
                },
                RawNode {
                    uid: 2,
                    x: 10.0,
                    y: 0.0,
                    z: 0.0,
                    rotation: [0.0; 4],
                    forward_item_uid: 0,
                    backward_item_uid: 0,
                },
            ],
            vec![RawRoad {
                uid: 10,
                node_a: 1,
                node_b: 2,
                speed_limit_kmh: 0,
                lanes_forward: 1,
                lanes_backward: 0,
                look_token: 0,
                dlc_guard: 0,
                is_hidden: false,
                gps_avoid: false,
                road_type_token: 0,
            }],
            vec![],
        ));
        let g = b.build();
        assert_eq!(g.edges[0].speed_limit_kmh, None);
    }

    #[test]
    fn building_pair_yields_bidirectional_edges() {
        let mut b = GraphBuilder::new();
        let mut s = ParsedSector {
            nodes: vec![
                RawNode {
                    uid: 1,
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    rotation: [0.0; 4],
                    forward_item_uid: 0,
                    backward_item_uid: 0,
                },
                RawNode {
                    uid: 2,
                    x: 30.0,
                    y: 0.0,
                    z: 40.0,
                    rotation: [0.0; 4],
                    forward_item_uid: 0,
                    backward_item_uid: 0,
                },
            ],
            roads: vec![],
            prefabs: vec![],
            signs: vec![],
            ferries: vec![],
            buildings: vec![RawBuilding {
                uid: 500,
                node_uid: 1,
                forward_node_uid: 2,
            }],
            recovered_nodes_count: 0,
        };
        // Also test skip cases
        s.buildings.push(RawBuilding {
            uid: 501,
            node_uid: 0,
            forward_node_uid: 2,
        }); // zero-uid
        s.buildings.push(RawBuilding {
            uid: 502,
            node_uid: 7,
            forward_node_uid: 2,
        }); // one unresolved
        s.buildings.push(RawBuilding {
            uid: 503,
            node_uid: 1,
            forward_node_uid: 1,
        }); // self-loop
        b.merge_sector(s);
        let g = b.build();
        let building_edges: Vec<_> = g
            .edges
            .iter()
            .filter(|e| e.direction == "building")
            .collect();
        assert_eq!(
            building_edges.len(),
            2,
            "exactly one bidirectional pair survives"
        );
        assert!(building_edges.iter().any(|e| e.from == 1 && e.to == 2));
        assert!(building_edges.iter().any(|e| e.from == 2 && e.to == 1));
        // 3-4-5 triangle => distance 50
        assert!((building_edges[0].distance_m - 50.0).abs() < 0.01);
    }

    #[test]
    fn prefabs_are_processed() {
        let mut b = GraphBuilder::new();
        b.merge_sector(make_sector(
            vec![],
            vec![],
            vec![RawPrefab {
                uid: 100,
                template_token: 0xABCD,
                node_count: 2,
                nodes: vec![1, 2],
            }],
        ));
        let g = b.build();
        assert_eq!(g.prefabs.len(), 1);
        assert_eq!(g.stats.prefab_count, 1);
        assert_eq!(g.prefabs[0].uid, 100);
        assert_eq!(g.prefabs[0].template_token, 0xABCD);
        assert_eq!(g.prefabs[0].connected_node_uids, vec![1, 2]);
    }

    #[test]
    fn test_navcurve_tangent_from_rotation() {
        // Identity quaternion [1,0,0,0] WXYZ → FORWARD direction unchanged: (0,0,-1)
        let identity = [1.0f32, 0.0, 0.0, 0.0];
        let t = crate::spline::quat_rotate_vec(identity, crate::spline::FORWARD);
        assert!(t.x.abs() < 1e-4, "x≈0, got {}", t.x);
        assert!(t.y.abs() < 1e-4, "y≈0, got {}", t.y);
        assert!((t.z + 1.0).abs() < 1e-4, "z≈-1 (North), got {}", t.z);

        // East quaternion [√2/2, 0, -√2/2, 0] WXYZ → FORWARD rotates to East (+X)
        let s = (2.0f32).sqrt() / 2.0;
        let east_rot = [s, 0.0, -s, 0.0];
        let t2 = crate::spline::quat_rotate_vec(east_rot, crate::spline::FORWARD);
        assert!((t2.x - 1.0).abs() < 1e-3, "x≈1 (East), got {}", t2.x);
        assert!(t2.y.abs() < 1e-4, "y≈0, got {}", t2.y);
        assert!(t2.z.abs() < 1e-3, "z≈0, got {}", t2.z);
    }
}
