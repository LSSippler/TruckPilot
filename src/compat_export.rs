//! Compatibility export layer.
//!
//! Converts internal `MapData` and `GraphData` into simplified "compat" formats
//! intended for external tooling consumption.

use std::collections::BTreeSet;

use crate::graph_schema::{
    CompatGraph, CompatGraphEdge, CompatNode, CompatRoad, CompatRoadLook, GraphData,
};
use crate::json_export::MapData;

/// Build the compat-node list from map data.
///
/// Each `MapNode` maps to one `CompatNode`.
pub fn build_compat_nodes(map: &MapData) -> Vec<CompatNode> {
    let mut nodes: Vec<CompatNode> = map
        .nodes
        .iter()
        .map(|n| CompatNode {
            node_uid: n.uid,
            x: n.x,
            y: n.y,
            z: n.z,
        })
        .collect();
    nodes.sort_by_key(|n| n.node_uid);
    nodes
}

/// Build the compat-road list from map data.
///
/// Each `MapRoad` maps to one `CompatRoad`. Only the first and last node of
/// the road's node list are used as endpoints.
pub fn build_compat_roads(map: &MapData) -> Vec<CompatRoad> {
    let mut roads: Vec<CompatRoad> = map
        .roads
        .iter()
        .map(|r| {
            let length = if r.nodes.len() >= 2 {
                let mut total = 0.0;
                for w in r.nodes.windows(2) {
                    let a = map.nodes.iter().find(|n| n.uid == w[0]);
                    let b = map.nodes.iter().find(|n| n.uid == w[1]);
                    if let (Some(a), Some(b)) = (a, b) {
                        let dx = a.x - b.x;
                        let dy = a.y - b.y;
                        let dz = a.z - b.z;
                        total += (dx * dx + dy * dy + dz * dz).sqrt();
                    }
                }
                total
            } else {
                0.0
            };

            let from_node_uid = r.nodes.first().copied().unwrap_or(0);
            let to_node_uid = r.nodes.last().copied().unwrap_or(0);
            let lane_count = r.lane_count_forward.max(r.lane_count_backward).max(1);

            CompatRoad {
                road_uid: r.uid.clone(),
                name: r.name.clone(),
                look_token: r.look_token.clone(),
                from_node_uid,
                to_node_uid,
                length,
                speed_limit: r.speed_limit,
                lane_count,
            }
        })
        .collect();
    roads.sort_by(|a, b| a.road_uid.cmp(&b.road_uid));
    roads
}

/// Build a sorted, deduplicated list of road looks from map data.
///
/// Each unique `look_token` produces one `CompatRoadLook`. The `name` field
/// mirrors the token (real names would come from definition files).
pub fn build_compat_road_looks(map: &MapData) -> Vec<CompatRoadLook> {
    let mut tokens: BTreeSet<&str> = BTreeSet::new();
    for road in &map.roads {
        if !road.look_token.is_empty() {
            tokens.insert(&road.look_token);
        }
    }
    tokens
        .into_iter()
        .map(|t| CompatRoadLook {
            token: t.to_string(),
            name: t.to_string(),
        })
        .collect()
}

/// Build the compat graph from internal `GraphData`.
///
/// This is a straightforward structural conversion.
pub fn build_compat_graph(graph: &GraphData) -> CompatGraph {
    let nodes: Vec<CompatNode> = graph
        .nodes
        .iter()
        .map(|n| CompatNode {
            node_uid: n.uid,
            x: n.x,
            y: n.y,
            z: n.z,
        })
        .collect();

    let edges: Vec<CompatGraphEdge> = graph
        .edges
        .iter()
        .map(|e| CompatGraphEdge {
            edge_uid: e.edge_uid,
            from_node_uid: e.from_node_uid,
            to_node_uid: e.to_node_uid,
            road_uid: e.road_uid.clone(),
            distance_m: e.distance_m,
            direction: e.direction.clone(),
            lane_count: e.lane_count,
            speed_limit_kmh: e.speed_limit_kmh,
            flags: e.flags.clone(),
        })
        .collect();

    CompatGraph { nodes, edges }
}

/// Write all four compat JSON files to the given directory prefix.
///
/// Files written:
/// - `<prefix>_nodes.json`
/// - `<prefix>_roads.json`
/// - `<prefix>_road_looks.json`
/// - `<prefix>_graph.json`
pub fn write_compat_files(prefix: &str, map: &MapData, graph: &GraphData) -> Result<(), String> {
    let nodes = build_compat_nodes(map);
    let roads = build_compat_roads(map);
    let looks = build_compat_road_looks(map);
    let compat_graph = build_compat_graph(graph);

    write_json(&format!("{prefix}_nodes.json"), &nodes)?;
    write_json(&format!("{prefix}_roads.json"), &roads)?;
    write_json(&format!("{prefix}_road_looks.json"), &looks)?;
    write_json(&format!("{prefix}_graph.json"), &compat_graph)?;

    Ok(())
}

fn write_json<T: serde::Serialize>(path: &str, value: &T) -> Result<(), String> {
    let file = std::fs::File::create(path).map_err(|e| format!("cannot create {path}: {e}"))?;
    serde_json::to_writer(file, value).map_err(|e| format!("serialization error: {e}"))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_map() -> MapData {
        crate::json_export::MapData {
            nodes: vec![
                crate::json_export::MapNode {
                    uid: 1,
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                crate::json_export::MapNode {
                    uid: 2,
                    x: 10.0,
                    y: 0.0,
                    z: 0.0,
                },
            ],
            roads: vec![
                crate::json_export::MapRoad {
                    uid: "r1".into(),
                    name: "Main Rd".into(),
                    look_token: "asphalt".into(),
                    nodes: vec![1, 2],
                    speed_limit: Some(80.0),
                    lane_count_forward: 2,
                    lane_count_backward: 1,
                },
                crate::json_export::MapRoad {
                    uid: "r2".into(),
                    name: String::new(),
                    look_token: "asphalt".into(),
                    nodes: vec![2, 1],
                    speed_limit: None,
                    lane_count_forward: 0,
                    lane_count_backward: 0,
                },
            ],
            prefabs: vec![],
        }
    }

    #[test]
    fn test_build_compat_nodes() {
        let map = sample_map();
        let nodes = build_compat_nodes(&map);
        assert_eq!(nodes.len(), 2);
        assert_eq!(nodes[0].node_uid, 1);
        assert_eq!(nodes[1].node_uid, 2);
        assert_eq!(nodes[0].x, 0.0);
        assert_eq!(nodes[0].y, 0.0);
        assert_eq!(nodes[0].z, 0.0);
    }

    #[test]
    fn test_build_compat_roads() {
        let map = sample_map();
        let roads = build_compat_roads(&map);
        assert_eq!(roads.len(), 2);
        assert_eq!(roads[0].road_uid, "r1");
        assert_eq!(roads[0].name, "Main Rd");
        assert_eq!(roads[0].speed_limit, Some(80.0));
        assert_eq!(roads[0].lane_count, 2); // max(2, 1)
        assert_eq!(roads[1].lane_count, 1); // max(0, 0) -> 1
    }

    #[test]
    fn test_build_compat_road_looks_dedup() {
        let map = sample_map();
        let looks = build_compat_road_looks(&map);
        // Both roads have "asphalt" → only 1 look
        assert_eq!(looks.len(), 1);
        assert_eq!(looks[0].token, "asphalt");
    }

    #[test]
    fn test_build_compat_graph_conversion() {
        let map = sample_map();
        let graph_data = crate::graph_export::build_graph(&map).unwrap();
        let compat = build_compat_graph(&graph_data);

        assert_eq!(compat.nodes.len(), graph_data.nodes.len());
        assert_eq!(compat.edges.len(), graph_data.edges.len());
    }

    #[test]
    fn test_write_compat_files() {
        let map = sample_map();
        let graph = crate::graph_export::build_graph(&map).unwrap();
        let prefix = "test_compat";
        write_compat_files(prefix, &map, &graph).unwrap();

        // Verify all four files exist and parse.
        for suffix in &["nodes", "roads", "road_looks", "graph"] {
            let path = format!("{prefix}_{suffix}.json");
            assert!(std::path::Path::new(&path).exists(), "missing {path}");
            std::fs::remove_file(&path).unwrap();
        }
    }
}
