//! Graph builder — converts parsed sector data into a routing graph.
//!
//! Merges nodes and roads from all loaded sectors, deduplicates by UID,
//! and emits directed edges with speed limits and lane counts.

use std::collections::HashMap;
use std::time::Instant;

use serde::{Deserialize, Serialize};
use tracing::{info, instrument, warn};

use crate::road_look::RoadLookEntry;
use crate::sector::{ParsedSector, RawBuilding, RawFerry, RawNode, RawPrefab, RawRoad};
use crate::signs::TrafficSign;
use crate::spatial_match::{
    apply_filters, build_spatial_index, pass1_strict_config, pass2_config, query_circle,
    select_best_match, OrphanEndpoint, SectorId, DEFAULT_CELL_SIZE, SECTOR_ID_UNKNOWN,
};
use std::collections::HashSet;

// ---------------------------------------------------------------------------
// Graph types
// ---------------------------------------------------------------------------

/// A node in the routing graph.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, bincode::Encode, bincode::Decode)]
pub struct GraphNode {
    pub uid: u64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
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
}

/// A prefab (junction/intersection) in the graph.
#[derive(Debug, Clone, Serialize, Deserialize, bincode::Encode, bincode::Decode, PartialEq)]
pub struct Prefab {
    pub uid: u64,
    pub template_token: u32,
    pub connected_node_uids: Vec<u64>,
}

/// The complete routing graph produced from one or more sectors.
#[derive(Debug, Clone, Serialize, Deserialize, Default, bincode::Encode, bincode::Decode)]
pub struct MapGraph {
    pub nodes: Vec<GraphNode>,
    pub edges: Vec<GraphEdge>,
    pub signs: Vec<TrafficSign>,
    pub prefabs: Vec<Prefab>,
    pub stats: BuildStats,
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
    /// Tries an exact lookup of `road_type_token` (= `right_look` token) in
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
                    road.lanes_forward = entry.lanes_right;
                    road.lanes_backward = entry.lanes_left;
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
            })
            .collect();
        nodes.sort_by_key(|n| n.uid);

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
            let raw_candidates =
                query_circle(&spatial_index, &orphan.resolved_pos, pass2.max_dist);
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
                });
                edge_uid += 1;
                pass2_edges += 1;
            }
        }
        info!(
            "Pass 2 (WIDE 200m/15m + adj-sector): {} unique matches ({} edges) from {} orphans",
            pass2_matches, pass2_edges, orphans.len()
        );

        // Process prefabs
        let prefabs: Vec<Prefab> = self
            .raw_prefabs
            .into_iter()
            .map(|p| Prefab {
                uid: p.uid,
                template_token: p.template_token,
                connected_node_uids: p.nodes,
            })
            .collect();

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
            },
            nodes,
            edges,
            signs,
            prefabs,
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
}
