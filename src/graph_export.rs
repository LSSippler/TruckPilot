//! Graph construction from parsed ETS2 map data.
//!
//! Converts raw `MapData` (nodes, roads, prefabs) into the topological
//! `GraphData` representation used by the autopilot.

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::hash::Hasher;

use siphasher::sip::SipHasher24;

use crate::graph_schema::{
    GraphData, GraphEdge, GraphMetrics, GraphNode, QualityMeta, QualityReport,
};
// MapPrefab, MapRoad are used in test constructors below.
#[allow(unused_imports)]
use crate::json_export::{MapData, MapNode, MapPrefab, MapRoad};

/// Fixed keys for deterministic SipHash-based edge UID generation.
const SIP_KEY_A: u64 = 0x1234_5678;
const SIP_KEY_B: u64 = 0x9ABC_DEF0;

/// Build a graph from raw map data.
///
/// # Algorithm
/// 1. Convert all `MapNode` → `GraphNode`.
/// 2. For each road, walk consecutive node pairs and emit directed edges
///    based on forward/backward lane counts.
/// 3. For each prefab with >1 nodes, connect every unordered pair in both
///    directions (unless a parallel edge already exists).
/// 4. Validate referential integrity (no dangling node/road references).
/// 5. Sort nodes and edges for deterministic output.
/// 6. Generate `edge_uid` deterministically via SipHash.
pub fn build_graph(map: &MapData) -> Result<GraphData, String> {
    let node_lookup: BTreeMap<u64, &MapNode> = map.nodes.iter().map(|n| (n.uid, n)).collect();
    let road_set: BTreeSet<&str> = map.roads.iter().map(|r| r.uid.as_str()).collect();

    // 1) Determine lane capacity per base node.
    let mut node_lane_capacity: BTreeMap<u64, u32> =
        node_lookup.keys().map(|&uid| (uid, 1_u32)).collect();
    for road in &map.roads {
        for &nuid in &road.nodes {
            if !node_lookup.contains_key(&nuid) {
                return Err(format!("dangling node_uid {} in road {}", nuid, road.uid));
            }
            let road_lane_max = road.lane_count_forward.max(road.lane_count_backward).max(1);
            let cap = node_lane_capacity.entry(nuid).or_insert(1);
            *cap = (*cap).max(road_lane_max);
        }
    }

    // 2) Build lane sub-nodes.
    let mut used_uids: BTreeSet<u64> = node_lookup.keys().copied().collect();
    let mut lane_node_uid: BTreeMap<(u64, u32), u64> = BTreeMap::new();
    let mut lane_node_str: BTreeMap<(u64, u32), String> = BTreeMap::new();
    let mut nodes: Vec<GraphNode> = Vec::new();

    for (&base_uid, base_node) in &node_lookup {
        let lane_cap = *node_lane_capacity.get(&base_uid).unwrap_or(&1);
        for lane in 0..lane_cap {
            let lane_uid_num = if lane == 0 {
                base_uid
            } else {
                unique_lane_uid(base_uid, lane, &mut used_uids)
            };
            let lane_uid_text = format!("0x{:X}_lane_{}", base_uid, lane);

            lane_node_uid.insert((base_uid, lane), lane_uid_num);
            lane_node_str.insert((base_uid, lane), lane_uid_text.clone());
            nodes.push(GraphNode {
                uid: lane_uid_num,
                lane_uid: Some(lane_uid_text),
                base_uid: Some(base_uid),
                lane_index: Some(lane),
                x: base_node.x,
                y: base_node.y,
                z: base_node.z,
            });
        }
    }

    // 3) Build lane edges from roads.
    let mut edges: Vec<GraphEdge> = Vec::new();
    for road in &map.roads {
        let road_uid = Some(road.uid.clone());
        for w in road.nodes.windows(2) {
            let from_base = w[0];
            let to_base = w[1];
            let (from_node, to_node) =
                match (node_lookup.get(&from_base), node_lookup.get(&to_base)) {
                    (Some(f), Some(t)) => (*f, *t),
                    _ => continue,
                };
            let distance_m = euclidean_3d(from_node, to_node);

            let fwd = road.lane_count_forward > 0;
            let bwd = road.lane_count_backward > 0;

            if fwd {
                for lane in 0..road.lane_count_forward {
                    let from_uid = *lane_node_uid.get(&(from_base, lane)).ok_or_else(|| {
                        format!("missing lane node for {} lane {}", from_base, lane)
                    })?;
                    let to_uid = *lane_node_uid.get(&(to_base, lane)).ok_or_else(|| {
                        format!("missing lane node for {} lane {}", to_base, lane)
                    })?;
                    edges.push(make_edge(EdgeParams {
                        from_uid,
                        to_uid,
                        from_lane_uid: lane_node_str.get(&(from_base, lane)).cloned(),
                        to_lane_uid: lane_node_str.get(&(to_base, lane)).cloned(),
                        road_uid: road_uid.clone(),
                        distance_m,
                        direction: "forward",
                        lane_count: road.lane_count_forward,
                        speed_limit_kmh: road.speed_limit,
                        lane: Some(lane),
                        from_lane: None,
                        to_lane: None,
                        flags: vec![format!("lane={}", lane)],
                    }));
                }
            }

            if bwd {
                for lane in 0..road.lane_count_backward {
                    let from_uid = *lane_node_uid.get(&(to_base, lane)).ok_or_else(|| {
                        format!("missing lane node for {} lane {}", to_base, lane)
                    })?;
                    let to_uid = *lane_node_uid.get(&(from_base, lane)).ok_or_else(|| {
                        format!("missing lane node for {} lane {}", from_base, lane)
                    })?;
                    edges.push(make_edge(EdgeParams {
                        from_uid,
                        to_uid,
                        from_lane_uid: lane_node_str.get(&(to_base, lane)).cloned(),
                        to_lane_uid: lane_node_str.get(&(from_base, lane)).cloned(),
                        road_uid: road_uid.clone(),
                        distance_m,
                        direction: "backward",
                        lane_count: road.lane_count_backward,
                        speed_limit_kmh: road.speed_limit,
                        lane: Some(lane),
                        from_lane: None,
                        to_lane: None,
                        flags: vec![format!("lane={}", lane)],
                    }));
                }
            }

            if !fwd && !bwd {
                let from_uid = *lane_node_uid
                    .get(&(from_base, 0))
                    .ok_or_else(|| format!("missing lane node for {} lane 0", from_base))?;
                let to_uid = *lane_node_uid
                    .get(&(to_base, 0))
                    .ok_or_else(|| format!("missing lane node for {} lane 0", to_base))?;
                edges.push(make_edge(EdgeParams {
                    from_uid,
                    to_uid,
                    from_lane_uid: lane_node_str.get(&(from_base, 0)).cloned(),
                    to_lane_uid: lane_node_str.get(&(to_base, 0)).cloned(),
                    road_uid: road_uid.clone(),
                    distance_m,
                    direction: "bidirectional_unknown",
                    lane_count: 1,
                    speed_limit_kmh: road.speed_limit,
                    lane: Some(0),
                    from_lane: None,
                    to_lane: None,
                    flags: vec!["no_lanes_unknown".to_string(), "lane=0".to_string()],
                }));
                edges.push(make_edge(EdgeParams {
                    from_uid: to_uid,
                    to_uid: from_uid,
                    from_lane_uid: lane_node_str.get(&(to_base, 0)).cloned(),
                    to_lane_uid: lane_node_str.get(&(from_base, 0)).cloned(),
                    road_uid: road_uid.clone(),
                    distance_m,
                    direction: "bidirectional_unknown",
                    lane_count: 1,
                    speed_limit_kmh: road.speed_limit,
                    lane: Some(0),
                    from_lane: None,
                    to_lane: None,
                    flags: vec!["no_lanes_unknown".to_string(), "lane=0".to_string()],
                }));
            }
        }
    }

    // 4) Lane-change edges between adjacent lanes at same base node.
    for (&base_uid, &lane_cap) in &node_lane_capacity {
        if lane_cap <= 1 {
            continue;
        }
        for lane in 0..(lane_cap - 1) {
            let from_uid = *lane_node_uid
                .get(&(base_uid, lane))
                .ok_or_else(|| format!("missing lane node for {} lane {}", base_uid, lane))?;
            let to_uid = *lane_node_uid
                .get(&(base_uid, lane + 1))
                .ok_or_else(|| format!("missing lane node for {} lane {}", base_uid, lane + 1))?;
            edges.push(make_edge(EdgeParams {
                from_uid,
                to_uid,
                from_lane_uid: lane_node_str.get(&(base_uid, lane)).cloned(),
                to_lane_uid: lane_node_str.get(&(base_uid, lane + 1)).cloned(),
                road_uid: None,
                distance_m: 1.0,
                direction: "lane_change",
                lane_count: 1,
                speed_limit_kmh: None,
                lane: None,
                from_lane: Some(lane),
                to_lane: Some(lane + 1),
                flags: vec![
                    "lane_change".to_string(),
                    format!("from_lane={}", lane),
                    format!("to_lane={}", lane + 1),
                ],
            }));

            edges.push(make_edge(EdgeParams {
                from_uid: to_uid,
                to_uid: from_uid,
                from_lane_uid: lane_node_str.get(&(base_uid, lane + 1)).cloned(),
                to_lane_uid: lane_node_str.get(&(base_uid, lane)).cloned(),
                road_uid: None,
                distance_m: 1.0,
                direction: "lane_change",
                lane_count: 1,
                speed_limit_kmh: None,
                lane: None,
                from_lane: Some(lane + 1),
                to_lane: Some(lane),
                flags: vec![
                    "lane_change".to_string(),
                    format!("from_lane={}", lane + 1),
                    format!("to_lane={}", lane),
                ],
            }));
        }
    }

    // 5. Prefab interconnections use lane_0 as connector lane.
    // Validate all prefab nodes exist before processing.
    let mut connected_pairs: HashSet<(u64, u64)> = edges
        .iter()
        .map(|e| {
            if e.from_node_uid <= e.to_node_uid {
                (e.from_node_uid, e.to_node_uid)
            } else {
                (e.to_node_uid, e.from_node_uid)
            }
        })
        .collect();

    for prefab in &map.prefabs {
        for &nuid in &prefab.nodes {
            if !node_lookup.contains_key(&nuid) {
                return Err(format!(
                    "dangling node_uid {} in prefab {}",
                    nuid, prefab.uid
                ));
            }
        }
    }

    for prefab in &map.prefabs {
        if prefab.nodes.len() <= 1 {
            continue;
        }
        let prefab_road: Option<String> = None;
        for i in 0..prefab.nodes.len() {
            for j in (i + 1)..prefab.nodes.len() {
                let u = prefab.nodes[i];
                let v = prefab.nodes[j];

                // Nodes already validated above — expect is safe.
                let u_node = map
                    .nodes
                    .iter()
                    .find(|n| n.uid == u)
                    .expect("validated prefab node");
                let v_node = map
                    .nodes
                    .iter()
                    .find(|n| n.uid == v)
                    .expect("validated prefab node");
                let dist = euclidean_3d(u_node, v_node);

                let u_lane0 = *lane_node_uid
                    .get(&(u, 0))
                    .ok_or_else(|| format!("missing lane_0 for node {}", u))?;
                let v_lane0 = *lane_node_uid
                    .get(&(v, 0))
                    .ok_or_else(|| format!("missing lane_0 for node {}", v))?;

                let e1 = make_edge(EdgeParams {
                    from_uid: u_lane0,
                    to_uid: v_lane0,
                    from_lane_uid: lane_node_str.get(&(u, 0)).cloned(),
                    to_lane_uid: lane_node_str.get(&(v, 0)).cloned(),
                    road_uid: prefab_road.clone(),
                    distance_m: dist,
                    direction: "prefab_interconnect",
                    lane_count: 1,
                    speed_limit_kmh: None,
                    lane: Some(0),
                    from_lane: None,
                    to_lane: None,
                    flags: Vec::new(),
                });
                let e2 = make_edge(EdgeParams {
                    from_uid: v_lane0,
                    to_uid: u_lane0,
                    from_lane_uid: lane_node_str.get(&(v, 0)).cloned(),
                    to_lane_uid: lane_node_str.get(&(u, 0)).cloned(),
                    road_uid: prefab_road.clone(),
                    distance_m: dist,
                    direction: "prefab_interconnect",
                    lane_count: 1,
                    speed_limit_kmh: None,
                    lane: Some(0),
                    from_lane: None,
                    to_lane: None,
                    flags: Vec::new(),
                });

                // Skip if any edge between the same pair already exists (regardless of direction).
                let pair = if u_lane0 <= v_lane0 {
                    (u_lane0, v_lane0)
                } else {
                    (v_lane0, u_lane0)
                };
                let already_exists = connected_pairs.contains(&pair);
                if !already_exists {
                    edges.push(e1);
                    edges.push(e2);
                    connected_pairs.insert(pair);
                }
            }
        }
    }

    // 6. Dangling-reference check.
    let node_set: BTreeSet<u64> = nodes.iter().map(|n| n.uid).collect();
    for edge in &edges {
        if !node_set.contains(&edge.from_node_uid) {
            return Err(format!(
                "dangling from_node_uid {} in edge {}",
                edge.from_node_uid, edge.edge_uid
            ));
        }
        if !node_set.contains(&edge.to_node_uid) {
            return Err(format!(
                "dangling to_node_uid {} in edge {}",
                edge.to_node_uid, edge.edge_uid
            ));
        }
        if let Some(ref ruid) = edge.road_uid {
            if !road_set.contains(ruid.as_str()) {
                return Err(format!(
                    "dangling road_uid {} in edge {}",
                    ruid, edge.edge_uid
                ));
            }
        }
    }

    // 7. Stable sort — include all key fields for determinism.
    nodes.sort_by_key(|n| n.uid);
    edges.sort_by(|a, b| {
        a.from_node_uid
            .cmp(&b.from_node_uid)
            .then_with(|| a.to_node_uid.cmp(&b.to_node_uid))
            .then_with(|| a.from_lane_uid.cmp(&b.from_lane_uid))
            .then_with(|| a.to_lane_uid.cmp(&b.to_lane_uid))
            .then_with(|| a.road_uid.cmp(&b.road_uid))
            .then_with(|| a.direction.cmp(&b.direction))
            .then_with(|| a.edge_uid.cmp(&b.edge_uid))
    });

    Ok(GraphData {
        meta: QualityMeta {
            schema_version: "1.0.0".into(),
            map_name: "unknown".into(),
            generated_at: String::new(),
        },
        nodes,
        edges,
    })
}

/// Parameters for creating a graph edge.
struct EdgeParams<'a> {
    from_uid: u64,
    to_uid: u64,
    from_lane_uid: Option<String>,
    to_lane_uid: Option<String>,
    road_uid: Option<String>,
    distance_m: f64,
    direction: &'a str,
    lane_count: u32,
    speed_limit_kmh: Option<f64>,
    lane: Option<u32>,
    from_lane: Option<u32>,
    to_lane: Option<u32>,
    flags: Vec<String>,
}

/// Helper: create a `GraphEdge` with deterministic `edge_uid`.
fn make_edge(params: EdgeParams) -> GraphEdge {
    let EdgeParams {
        from_uid,
        to_uid,
        from_lane_uid,
        to_lane_uid,
        road_uid,
        distance_m,
        direction,
        lane_count,
        speed_limit_kmh,
        lane,
        from_lane,
        to_lane,
        flags,
    } = params;
    let edge_uid = hash_edge_uid(
        from_uid, to_uid, &road_uid, direction, lane, from_lane, to_lane,
    );
    GraphEdge {
        edge_uid,
        from_node_uid: from_uid,
        to_node_uid: to_uid,
        from_lane_uid,
        to_lane_uid,
        road_uid,
        distance_m,
        direction: direction.to_string(),
        lane_count,
        speed_limit_kmh,
        flags,
    }
}

fn unique_lane_uid(base_uid: u64, lane: u32, used_uids: &mut BTreeSet<u64>) -> u64 {
    let mut salt = 0_u64;
    loop {
        let mut hasher =
            SipHasher24::new_with_keys(SIP_KEY_A ^ 0xA5A5_5A5A, SIP_KEY_B ^ 0x5A5A_A5A5);
        hasher.write(format!("lane|{}|{}|{}", base_uid, lane, salt).as_bytes());
        let candidate = hasher.finish();
        if !used_uids.contains(&candidate) {
            used_uids.insert(candidate);
            return candidate;
        }
        salt = salt.wrapping_add(1);
    }
}

/// Compute a deterministic edge UID using SipHash-2-4 with fixed keys.
///
/// The input string is constructed as
/// `<from>|<to>|<road>|<direction>[|<lane>][|fl:<from_lane>|tl:<to_lane>]`.
fn hash_edge_uid(
    from: u64,
    to: u64,
    road_uid: &Option<String>,
    direction: &str,
    lane: Option<u32>,
    from_lane: Option<u32>,
    to_lane: Option<u32>,
) -> u64 {
    let mut hasher = SipHasher24::new_with_keys(SIP_KEY_A, SIP_KEY_B);
    hasher.write(format!("{}|{}", from, to).as_bytes());
    if let Some(ref ruid) = road_uid {
        hasher.write(b"|");
        hasher.write(ruid.as_bytes());
    } else {
        hasher.write(b"|");
    }
    hasher.write(b"|");
    hasher.write(direction.as_bytes());
    if let Some(l) = lane {
        hasher.write(b"|");
        hasher.write(l.to_string().as_bytes());
    }
    if let Some(fl) = from_lane {
        hasher.write(b"|fl:");
        hasher.write(fl.to_string().as_bytes());
    }
    if let Some(tl) = to_lane {
        hasher.write(b"|tl:");
        hasher.write(tl.to_string().as_bytes());
    }
    hasher.finish()
}

fn euclidean_3d(a: &MapNode, b: &MapNode) -> f64 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    let dz = a.z - b.z;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Write the graph to a JSON file with byte-stable output.
pub fn write_graph_file(path: &str, graph: &GraphData) -> Result<(), String> {
    let file = std::fs::File::create(path).map_err(|e| format!("cannot create {}: {}", path, e))?;
    serde_json::to_writer(file, graph).map_err(|e| format!("serialization error: {}", e))
}

// ----- Phase 6 placeholders (filled in later) -----

/// Build a graph and return it together with the elapsed wall-clock time.
pub fn build_graph_timed(map: &MapData) -> Result<(GraphData, std::time::Duration), String> {
    let start = std::time::Instant::now();
    let graph = build_graph(map)?;
    let elapsed = start.elapsed();
    Ok((graph, elapsed))
}

/// Compute aggregate metrics for a given graph.
pub fn compute_graph_metrics(graph: &GraphData, build_time: std::time::Duration) -> GraphMetrics {
    let nodes_total = graph.nodes.len();
    let edges_total = graph.edges.len();

    let density = if nodes_total > 1 {
        edges_total as f64 / nodes_total as f64
    } else {
        0.0
    };

    let directed_count = graph
        .edges
        .iter()
        .filter(|e| e.direction == "forward" || e.direction == "backward")
        .count();
    let unknown_count = graph
        .edges
        .iter()
        .filter(|e| e.direction == "bidirectional_unknown")
        .count();
    let with_speed = graph
        .edges
        .iter()
        .filter(|e| e.speed_limit_kmh.is_some())
        .count();

    let pct_directed = if edges_total > 0 {
        directed_count as f64 / edges_total as f64 * 100.0
    } else {
        0.0
    };
    let pct_unknown = if edges_total > 0 {
        unknown_count as f64 / edges_total as f64 * 100.0
    } else {
        0.0
    };
    let pct_with_speed_limit = if edges_total > 0 {
        with_speed as f64 / edges_total as f64 * 100.0
    } else {
        0.0
    };

    // Largest component ratio: simple BFS-based estimation.
    let largest_component_ratio = largest_component_fraction(graph);

    GraphMetrics {
        nodes_total,
        edges_total,
        density,
        largest_component_ratio,
        pct_directed,
        pct_unknown,
        pct_with_speed_limit,
        build_time_ms: build_time.as_secs_f64() * 1000.0,
    }
}

fn largest_component_fraction(graph: &GraphData) -> f64 {
    if graph.nodes.is_empty() {
        return 0.0;
    }

    let mut adj: HashMap<u64, Vec<u64>> = HashMap::new();
    for edge in &graph.edges {
        adj.entry(edge.from_node_uid)
            .or_default()
            .push(edge.to_node_uid);
        adj.entry(edge.to_node_uid)
            .or_default()
            .push(edge.from_node_uid);
    }

    let mut visited: HashSet<u64> = HashSet::new();
    let mut max_size = 0usize;

    for node in &graph.nodes {
        if visited.contains(&node.uid) {
            continue;
        }
        let mut stack = vec![node.uid];
        visited.insert(node.uid);
        let mut comp_size = 1;
        while let Some(current) = stack.pop() {
            if let Some(neighbors) = adj.get(&current) {
                for &neighbor in neighbors {
                    if visited.insert(neighbor) {
                        stack.push(neighbor);
                        comp_size += 1;
                    }
                }
            }
        }
        if comp_size > max_size {
            max_size = comp_size;
        }
    }

    max_size as f64 / graph.nodes.len() as f64
}

/// Log graph metrics to stdout (only when `verbose` is true).
pub fn log_graph_metrics(metrics: &GraphMetrics, verbose: bool) {
    if !verbose {
        return;
    }
    println!("Graph Metrics:");
    println!("  nodes:             {}", metrics.nodes_total);
    println!("  edges:             {}", metrics.edges_total);
    println!("  density:           {:.3}", metrics.density);
    println!(
        "  largest component: {:.1}%",
        metrics.largest_component_ratio * 100.0
    );
    println!("  directed:          {:.1}%", metrics.pct_directed);
    println!("  unknown:           {:.1}%", metrics.pct_unknown);
    println!("  with speed limit:  {:.1}%", metrics.pct_with_speed_limit);
    println!("  build time:        {:.2} ms", metrics.build_time_ms);
}

/// Write a quality report to the given file path.
pub fn write_quality_report(path: &str, report: &QualityReport) -> Result<(), String> {
    let file = std::fs::File::create(path).map_err(|e| format!("cannot create {}: {}", path, e))?;
    serde_json::to_writer(file, report).map_err(|e| format!("serialization error: {}", e))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;

    fn sample_map() -> MapData {
        MapData {
            nodes: vec![
                MapNode {
                    uid: 1,
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                MapNode {
                    uid: 2,
                    x: 10.0,
                    y: 0.0,
                    z: 0.0,
                },
                MapNode {
                    uid: 3,
                    x: 20.0,
                    y: 0.0,
                    z: 0.0,
                },
                MapNode {
                    uid: 4,
                    x: 20.0,
                    y: 0.0,
                    z: 10.0,
                },
            ],
            roads: vec![
                MapRoad {
                    uid: "road_1".into(),
                    name: "Main St".into(),
                    look_token: "asphalt".into(),
                    nodes: vec![1, 2, 3],
                    speed_limit: Some(50.0),
                    lane_count_forward: 2,
                    lane_count_backward: 2,
                },
                MapRoad {
                    uid: "road_2".into(),
                    name: "Side Rd".into(),
                    look_token: "asphalt".into(),
                    nodes: vec![3, 4],
                    speed_limit: Some(30.0),
                    lane_count_forward: 1,
                    lane_count_backward: 0,
                },
            ],
            prefabs: vec![MapPrefab {
                uid: "prefab_1".into(),
                nodes: vec![1, 4],
            }],
        }
    }

    fn no_lanes_map() -> MapData {
        MapData {
            nodes: vec![
                MapNode {
                    uid: 10,
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                MapNode {
                    uid: 20,
                    x: 5.0,
                    y: 0.0,
                    z: 0.0,
                },
            ],
            roads: vec![MapRoad {
                uid: "unknown_lanes".into(),
                name: String::new(),
                look_token: "dirt".into(),
                nodes: vec![10, 20],
                speed_limit: None,
                lane_count_forward: 0,
                lane_count_backward: 0,
            }],
            prefabs: vec![],
        }
    }

    #[test]
    fn test_build_graph_basic() {
        let map = sample_map();
        let graph = build_graph(&map).unwrap();

        // Lane subnodes are added for lane>0.
        assert_eq!(graph.nodes.len(), 7);
        assert!(graph.edges.len() >= 14, "got {} edges", graph.edges.len());

        // All base nodes still exist as lane_0.
        let base_uids: HashSet<u64> = graph
            .nodes
            .iter()
            .filter_map(|n| (n.lane_index == Some(0)).then_some(n.uid))
            .collect();
        assert!(base_uids.contains(&1));
        assert!(base_uids.contains(&2));
        assert!(base_uids.contains(&3));
        assert!(base_uids.contains(&4));
    }

    #[test]
    fn test_direction_forward() {
        let map = sample_map();
        let graph = build_graph(&map).unwrap();

        let fwd_edges: Vec<&GraphEdge> = graph
            .edges
            .iter()
            .filter(|e| e.direction == "forward")
            .collect();
        assert_eq!(fwd_edges.len(), 5); // 2 lanes × 2 segments (1→2,2→3) + 1 lane (3→4)

        let bwd_edges: Vec<&GraphEdge> = graph
            .edges
            .iter()
            .filter(|e| e.direction == "backward")
            .collect();
        assert_eq!(bwd_edges.len(), 4); // 2 lanes × 2 segments (2→1, 3→2)
    }

    #[test]
    fn test_no_lanes_unknown() {
        let map = no_lanes_map();
        let graph = build_graph(&map).unwrap();

        assert_eq!(graph.edges.len(), 2);
        for edge in &graph.edges {
            assert_eq!(edge.direction, "bidirectional_unknown");
            assert!(edge.flags.contains(&"no_lanes_unknown".to_string()));
            assert_eq!(edge.lane_count, 1);
        }
    }

    #[test]
    fn test_prefab_interconnect() {
        let map = sample_map();
        let graph = build_graph(&map).unwrap();

        let prefab_edges: Vec<&GraphEdge> = graph
            .edges
            .iter()
            .filter(|e| e.direction == "prefab_interconnect")
            .collect();
        assert_eq!(prefab_edges.len(), 2); // 1→4, 4→1
        assert!(prefab_edges.iter().all(|e| e.road_uid.is_none()));
    }

    #[test]
    fn test_lane_graph_generation() {
        let map = MapData {
            nodes: vec![
                MapNode {
                    uid: 10,
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                MapNode {
                    uid: 20,
                    x: 100.0,
                    y: 0.0,
                    z: 0.0,
                },
            ],
            roads: vec![MapRoad {
                uid: "lane_test_road".into(),
                name: String::new(),
                look_token: "asphalt".into(),
                nodes: vec![10, 20],
                speed_limit: Some(80.0),
                lane_count_forward: 2,
                lane_count_backward: 1,
            }],
            prefabs: vec![],
        };

        let graph = build_graph(&map).unwrap();
        let forward: Vec<&GraphEdge> = graph
            .edges
            .iter()
            .filter(|e| e.direction == "forward")
            .collect();
        let backward: Vec<&GraphEdge> = graph
            .edges
            .iter()
            .filter(|e| e.direction == "backward")
            .collect();
        let lane_change: Vec<&GraphEdge> = graph
            .edges
            .iter()
            .filter(|e| e.direction == "lane_change")
            .collect();

        assert_eq!(forward.len(), 2);
        assert_eq!(backward.len(), 1);
        assert_eq!(lane_change.len(), 4); // bidirectional lane-change edges at both nodes

        assert!(forward
            .iter()
            .any(|e| e.flags.contains(&"lane=0".to_string())));
        assert!(forward
            .iter()
            .any(|e| e.flags.contains(&"lane=1".to_string())));
    }

    #[test]
    fn test_prefab_skip_existing_edge() {
        // Road and prefab between same nodes → only road edge should survive
        let map = MapData {
            nodes: vec![
                MapNode {
                    uid: 100,
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                MapNode {
                    uid: 200,
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
            ],
            roads: vec![MapRoad {
                uid: "r".into(),
                name: String::new(),
                look_token: "a".into(),
                nodes: vec![100, 200],
                speed_limit: None,
                lane_count_forward: 1,
                lane_count_backward: 0,
            }],
            prefabs: vec![MapPrefab {
                uid: "p".into(),
                nodes: vec![100, 200],
            }],
        };
        let graph = build_graph(&map).unwrap();
        // Only the forward road edge, prefab skipped.
        assert_eq!(graph.edges.len(), 1);
        assert_eq!(graph.edges[0].direction, "forward");
    }

    #[test]
    fn test_dangling_node_uid() {
        let map = MapData {
            nodes: vec![MapNode {
                uid: 1,
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }],
            roads: vec![MapRoad {
                uid: "r".into(),
                name: String::new(),
                look_token: "a".into(),
                nodes: vec![1, 999], // 999 does not exist
                speed_limit: None,
                lane_count_forward: 1,
                lane_count_backward: 0,
            }],
            prefabs: vec![],
        };
        let result = build_graph(&map);
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("dangling"));
    }

    #[test]
    fn test_determinism() {
        let map = sample_map();
        let g1 = build_graph(&map).unwrap();
        let g2 = build_graph(&map).unwrap();

        // Nodes should match
        assert_eq!(g1.nodes, g2.nodes);
        // Edges should match exactly
        assert_eq!(g1.edges.len(), g2.edges.len());
        for (e1, e2) in g1.edges.iter().zip(g2.edges.iter()) {
            assert_eq!(e1.edge_uid, e2.edge_uid);
            assert_eq!(e1.from_node_uid, e2.from_node_uid);
            assert_eq!(e1.to_node_uid, e2.to_node_uid);
            assert_eq!(e1.direction, e2.direction);
        }
    }

    #[test]
    fn test_edge_uid_deterministic() {
        let uid1 = hash_edge_uid(1, 2, &Some("road_a".into()), "forward", None, None, None);
        let uid2 = hash_edge_uid(1, 2, &Some("road_a".into()), "forward", None, None, None);
        assert_eq!(uid1, uid2);

        // Different direction gives different UID
        let uid3 = hash_edge_uid(1, 2, &Some("road_a".into()), "backward", None, None, None);
        assert_ne!(uid1, uid3);

        // Different road gives different UID
        let uid4 = hash_edge_uid(1, 2, &Some("road_b".into()), "forward", None, None, None);
        assert_ne!(uid1, uid4);

        // Different main lane gives different UIDs (normal edges)
        let uid5 = hash_edge_uid(1, 2, &Some("road_a".into()), "forward", Some(0), None, None);
        let uid6 = hash_edge_uid(1, 2, &Some("road_a".into()), "forward", Some(1), None, None);
        assert_ne!(uid5, uid6);

        // Different from/to lanes give different UIDs (lane-change edges)
        let uid7 = hash_edge_uid(
            1,
            2,
            &Some("road_a".into()),
            "lane_change",
            None,
            Some(0),
            Some(1),
        );
        let uid8 = hash_edge_uid(
            1,
            2,
            &Some("road_a".into()),
            "lane_change",
            None,
            Some(1),
            Some(2),
        );
        assert_ne!(uid7, uid8);

        // from_lane=Some(0),to_lane=None vs from_lane=None,to_lane=Some(0) → different
        let uid9 = hash_edge_uid(
            1,
            2,
            &Some("road_a".into()),
            "lane_change",
            None,
            Some(0),
            None,
        );
        let uid10 = hash_edge_uid(
            1,
            2,
            &Some("road_a".into()),
            "lane_change",
            None,
            None,
            Some(0),
        );
        assert_ne!(uid9, uid10);
    }

    #[test]
    fn test_compute_metrics() {
        let map = sample_map();
        let graph = build_graph(&map).unwrap();
        let metrics = compute_graph_metrics(&graph, std::time::Duration::from_millis(5));

        assert_eq!(metrics.nodes_total, 7);
        assert!(metrics.edges_total >= 14);
        assert!(metrics.density > 0.0);
        assert_eq!(metrics.build_time_ms, 5.0);
        assert!(metrics.pct_with_speed_limit > 0.0);
    }

    #[test]
    fn test_write_graph_file() {
        let map = sample_map();
        let graph = build_graph(&map).unwrap();
        let path = "test_graph_output.json";
        write_graph_file(path, &graph).unwrap();

        let data = std::fs::read_to_string(path).unwrap();
        let parsed: GraphData = serde_json::from_str(&data).unwrap();
        assert_eq!(parsed.nodes.len(), 7);
        assert!(parsed.edges.len() >= 14);

        // Cleanup
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn test_write_quality_report() {
        let map = sample_map();
        let graph = build_graph(&map).unwrap();
        let metrics = compute_graph_metrics(&graph, std::time::Duration::ZERO);
        let report = QualityReport {
            meta: graph.meta.clone(),
            metrics,
        };
        let path = "test_quality_report.json";
        write_quality_report(path, &report).unwrap();

        let data = std::fs::read_to_string(path).unwrap();
        let parsed: QualityReport = serde_json::from_str(&data).unwrap();
        assert_eq!(parsed.metrics.nodes_total, 7);

        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn test_build_graph_timed() {
        let map = sample_map();
        let (_graph, _duration) = build_graph_timed(&map).unwrap();
        assert_eq!(_graph.nodes.len(), 7);
    }

    #[test]
    fn test_prefab_dangling_node_uid() {
        let map = MapData {
            nodes: vec![MapNode {
                uid: 1,
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }],
            roads: vec![],
            prefabs: vec![MapPrefab {
                uid: "prefab_1".into(),
                nodes: vec![1, 999], // 999 does not exist
            }],
        };
        let result = build_graph(&map);
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(
            err.contains("dangling node_uid"),
            "expected dangling node error, got: {}",
            err
        );
    }
}
