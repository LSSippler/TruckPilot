//! Graph builder — converts parsed sector data into a routing graph.
//!
//! Merges nodes and roads from all loaded sectors, deduplicates by UID,
//! and emits directed edges with speed limits and lane counts.

use std::collections::HashMap;
use std::time::Instant;

use serde::{Deserialize, Serialize};
use tracing::{info, instrument, warn};

use crate::sector::{ParsedSector, RawFerry, RawNode, RawPrefab, RawRoad};
use crate::signs::TrafficSign;

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
    sectors_merged: usize,
}

impl GraphBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Merge one parsed sector into the builder.
    /// Duplicate node UIDs are silently overwritten (last writer wins).
    #[instrument(skip(self, sector), fields(nodes = sector.nodes.len(), roads = sector.roads.len()))]
    pub fn merge_sector(&mut self, sector: ParsedSector) {
        for node in sector.nodes {
            self.nodes.insert(node.uid, node);
        }
        self.roads.extend(sector.roads);
        self.raw_prefabs.extend(sector.prefabs);
        self.raw_signs.extend(sector.signs);
        self.ferries.extend(sector.ferries);
        self.sectors_merged += 1;
    }

    /// Build the final `MapGraph` from all merged sectors.
    #[instrument(skip(self))]
    pub fn build(self) -> MapGraph {
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

        for road in &self.roads {
            let (Some(a), Some(b)) = (node_lookup.get(&road.node_a), node_lookup.get(&road.node_b))
            else {
                warn!(
                    "Road {} references missing node(s) {} / {}",
                    road.uid, road.node_a, road.node_b
                );
                continue;
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
                let Some(a) = node_lookup.get(&valid[i]) else { continue };
                for j in (i + 1)..valid.len() {
                    let Some(b) = node_lookup.get(&valid[j]) else { continue };
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
                let Some(a) = node_lookup.get(&nodes[i]) else { continue };
                for j in (i + 1)..nodes.len() {
                    let Some(b) = node_lookup.get(&nodes[j]) else { continue };
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

        // Process prefabs
        let prefabs: Vec<Prefab> = self.raw_prefabs.into_iter().map(|p| Prefab {
            uid: p.uid,
            template_token: p.template_token,
            connected_node_uids: p.nodes,
        }).collect();

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

    fn make_sector(nodes: Vec<RawNode>, roads: Vec<RawRoad>, prefabs: Vec<RawPrefab>) -> ParsedSector {
        ParsedSector {
            nodes,
            roads,
            prefabs,
            signs: vec![],
            ferries: vec![],
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
                },
                RawNode {
                    uid: 2,
                    x: 100.0,
                    y: 0.0,
                    z: 0.0,
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
                RawNode { uid: 1, x: 0.0, y: 0.0, z: 0.0 },
                RawNode { uid: 2, x: 10.0, y: 0.0, z: 0.0 },
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
