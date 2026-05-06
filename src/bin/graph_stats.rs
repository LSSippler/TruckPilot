//! `graph_stats` — print descriptive statistics for a `graph.json` file.
//!
//! Usage:
//!     cargo run --bin graph_stats -- output/graph.json

use std::collections::HashMap;
use std::fs::File;
use std::io::BufReader;
use std::path::PathBuf;

use truckpilot::graph_schema::GraphData;

#[derive(Debug, Default, Clone)]
struct Stats {
    nodes_total: usize,
    edges_total: usize,
    top_out_degree: Vec<(u64, usize)>,
    longest_edge: Option<(u64, u64, f64)>,
    shortest_edge: Option<(u64, u64, f64)>,
    avg_distance_m: f64,
    isolated_nodes: usize,
    self_loops: usize,
    direction_counts: HashMap<String, usize>,
    edges_with_speed: usize,
    edges_without_speed: usize,
    speed_min_kmh: Option<f64>,
    speed_max_kmh: Option<f64>,
    speed_avg_kmh: f64,
}

fn compute_stats(graph: &GraphData) -> Stats {
    let mut stats = Stats {
        nodes_total: graph.nodes.len(),
        edges_total: graph.edges.len(),
        ..Stats::default()
    };

    let mut out_degree: HashMap<u64, usize> = HashMap::with_capacity(graph.nodes.len());
    let mut connected: std::collections::HashSet<u64> =
        std::collections::HashSet::with_capacity(graph.nodes.len() * 2);

    let mut total_dist = 0.0_f64;
    let mut speed_total = 0.0_f64;

    for edge in &graph.edges {
        *out_degree.entry(edge.from_node_uid).or_insert(0) += 1;
        connected.insert(edge.from_node_uid);
        connected.insert(edge.to_node_uid);

        total_dist += edge.distance_m;

        if edge.from_node_uid == edge.to_node_uid {
            stats.self_loops += 1;
        }

        match stats.longest_edge {
            Some((_, _, d)) if d >= edge.distance_m => {}
            _ => stats.longest_edge = Some((edge.from_node_uid, edge.to_node_uid, edge.distance_m)),
        }
        match stats.shortest_edge {
            Some((_, _, d)) if d <= edge.distance_m => {}
            _ => {
                stats.shortest_edge = Some((edge.from_node_uid, edge.to_node_uid, edge.distance_m))
            }
        }

        *stats
            .direction_counts
            .entry(edge.direction.clone())
            .or_insert(0) += 1;

        if let Some(s) = edge.speed_limit_kmh {
            stats.edges_with_speed += 1;
            speed_total += s;
            stats.speed_min_kmh = Some(stats.speed_min_kmh.map_or(s, |m| m.min(s)));
            stats.speed_max_kmh = Some(stats.speed_max_kmh.map_or(s, |m| m.max(s)));
        } else {
            stats.edges_without_speed += 1;
        }
    }

    stats.avg_distance_m = if stats.edges_total > 0 {
        total_dist / stats.edges_total as f64
    } else {
        0.0
    };
    stats.speed_avg_kmh = if stats.edges_with_speed > 0 {
        speed_total / stats.edges_with_speed as f64
    } else {
        0.0
    };

    let mut deg_vec: Vec<(u64, usize)> = out_degree.into_iter().collect();
    // Stable secondary sort by uid so output is deterministic.
    deg_vec.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
    stats.top_out_degree = deg_vec.into_iter().take(5).collect();

    stats.isolated_nodes = graph
        .nodes
        .iter()
        .filter(|n| !connected.contains(&n.uid))
        .count();

    stats
}

fn print_stats(stats: &Stats) {
    println!("Graph statistics");
    println!("================");
    println!("Nodes total : {}", stats.nodes_total);
    println!("Edges total : {}", stats.edges_total);
    println!();

    println!("Top 5 nodes by outgoing edges");
    println!("-----------------------------");
    if stats.top_out_degree.is_empty() {
        println!("  (no edges)");
    } else {
        for (uid, deg) in &stats.top_out_degree {
            println!("  uid=0x{uid:X}  out_degree={deg}");
        }
    }
    println!();

    println!("Edge length");
    println!("-----------");
    match stats.longest_edge {
        Some((from, to, d)) => println!("  longest  : {d:.2} m  (0x{from:X} -> 0x{to:X})"),
        None => println!("  longest  : n/a"),
    }
    match stats.shortest_edge {
        Some((from, to, d)) => println!("  shortest : {d:.2} m  (0x{from:X} -> 0x{to:X})"),
        None => println!("  shortest : n/a"),
    }
    println!("  average  : {:.2} m", stats.avg_distance_m);
    println!();

    println!("Topology");
    println!("--------");
    println!("  isolated nodes : {}", stats.isolated_nodes);
    println!("  self-loops     : {}", stats.self_loops);
    println!();

    println!("Edge direction distribution");
    println!("---------------------------");
    let mut dirs: Vec<(&String, &usize)> = stats.direction_counts.iter().collect();
    dirs.sort_by(|a, b| b.1.cmp(a.1).then(a.0.cmp(b.0)));
    if dirs.is_empty() {
        println!("  (no edges)");
    } else {
        for (kind, count) in dirs {
            println!("  {kind:<28} : {count}");
        }
    }
    println!();

    println!("Speed limit distribution");
    println!("------------------------");
    println!("  with limit    : {}", stats.edges_with_speed);
    println!("  without limit : {}", stats.edges_without_speed);
    if stats.edges_with_speed > 0 {
        let min = stats.speed_min_kmh.unwrap_or(0.0);
        let max = stats.speed_max_kmh.unwrap_or(0.0);
        println!("  min speed     : {min:.1} km/h");
        println!("  max speed     : {max:.1} km/h");
        println!("  avg speed     : {:.1} km/h", stats.speed_avg_kmh);
    }
}

fn load_graph(path: &PathBuf) -> Result<GraphData, String> {
    let file = File::open(path).map_err(|e| format!("failed to open {}: {e}", path.display()))?;
    let reader = BufReader::new(file);
    serde_json::from_reader(reader).map_err(|e| format!("failed to parse {}: {e}", path.display()))
}

fn main() {
    let mut args = std::env::args().skip(1);
    let Some(path) = args.next() else {
        eprintln!("Usage: cargo run --bin graph_stats -- <graph.json>");
        std::process::exit(1);
    };

    let path = PathBuf::from(path);
    let graph = match load_graph(&path) {
        Ok(g) => g,
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(1);
        }
    };

    let stats = compute_stats(&graph);
    print_stats(&stats);
}

#[cfg(test)]
mod tests {
    use super::*;
    use truckpilot::graph_schema::{GraphEdge, GraphNode, QualityMeta};

    fn sample_graph() -> GraphData {
        GraphData {
            meta: QualityMeta {
                schema_version: "1.0.0".to_string(),
                map_name: "unit_test".to_string(),
                generated_at: String::new(),
            },
            nodes: vec![
                GraphNode {
                    uid: 1,
                    lane_uid: None,
                    base_uid: None,
                    lane_index: None,
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                GraphNode {
                    uid: 2,
                    lane_uid: None,
                    base_uid: None,
                    lane_index: None,
                    x: 100.0,
                    y: 0.0,
                    z: 0.0,
                },
                GraphNode {
                    uid: 3,
                    lane_uid: None,
                    base_uid: None,
                    lane_index: None,
                    x: 250.0,
                    y: 0.0,
                    z: 0.0,
                },
                // Isolated node:
                GraphNode {
                    uid: 99,
                    lane_uid: None,
                    base_uid: None,
                    lane_index: None,
                    x: 1000.0,
                    y: 0.0,
                    z: 0.0,
                },
            ],
            edges: vec![
                GraphEdge {
                    edge_uid: 1001,
                    from_node_uid: 1,
                    to_node_uid: 2,
                    from_lane_uid: None,
                    to_lane_uid: None,
                    road_uid: None,
                    distance_m: 100.0,
                    direction: "forward".to_string(),
                    lane_count: 1,
                    speed_limit_kmh: Some(80.0),
                    flags: vec![],
                },
                GraphEdge {
                    edge_uid: 1002,
                    from_node_uid: 2,
                    to_node_uid: 3,
                    from_lane_uid: None,
                    to_lane_uid: None,
                    road_uid: None,
                    distance_m: 150.0,
                    direction: "forward".to_string(),
                    lane_count: 1,
                    speed_limit_kmh: None,
                    flags: vec![],
                },
                GraphEdge {
                    edge_uid: 1003,
                    from_node_uid: 1,
                    to_node_uid: 1,
                    from_lane_uid: None,
                    to_lane_uid: None,
                    road_uid: None,
                    distance_m: 0.0,
                    direction: "bidirectional_unknown".to_string(),
                    lane_count: 1,
                    speed_limit_kmh: Some(50.0),
                    flags: vec![],
                },
            ],
        }
    }

    #[test]
    fn computes_basic_counts() {
        let stats = compute_stats(&sample_graph());
        assert_eq!(stats.nodes_total, 4);
        assert_eq!(stats.edges_total, 3);
        assert_eq!(stats.isolated_nodes, 1);
        assert_eq!(stats.self_loops, 1);
    }

    #[test]
    fn computes_distance_extrema() {
        let stats = compute_stats(&sample_graph());
        assert_eq!(stats.shortest_edge.unwrap().2, 0.0);
        assert_eq!(stats.longest_edge.unwrap().2, 150.0);
        assert!((stats.avg_distance_m - (250.0 / 3.0)).abs() < 1e-9);
    }

    #[test]
    fn computes_speed_distribution() {
        let stats = compute_stats(&sample_graph());
        assert_eq!(stats.edges_with_speed, 2);
        assert_eq!(stats.edges_without_speed, 1);
        assert_eq!(stats.speed_min_kmh, Some(50.0));
        assert_eq!(stats.speed_max_kmh, Some(80.0));
    }

    #[test]
    fn top_out_degree_is_deterministic() {
        let stats = compute_stats(&sample_graph());
        // Node 1 has 2 outgoing edges (to 2 and self-loop), node 2 has 1.
        assert_eq!(stats.top_out_degree.first(), Some(&(1u64, 2usize)));
    }
}
