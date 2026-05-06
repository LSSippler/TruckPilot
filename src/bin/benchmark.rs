use std::fs::File;
use std::io::BufReader;
use std::path::PathBuf;
use std::time::Instant;

use truckpilot::autopilot::{plan_route_on_graph, CostMode, RouteConfig};
use truckpilot::graph_schema::GraphData;

const RUNS: usize = 100;

#[derive(Debug, Clone)]
struct BenchStats {
    runs: usize,
    successful_routes: usize,
    min_ms: f64,
    max_ms: f64,
    avg_ms: f64,
}

fn lcg_next(state: &mut u64) -> u64 {
    *state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
    *state
}

fn random_index(state: &mut u64, upper: usize) -> usize {
    if upper <= 1 {
        return 0;
    }
    (lcg_next(state) as usize) % upper
}

fn run_benchmark(graph: &GraphData, runs: usize, mut seed: u64) -> BenchStats {
    let nodes: Vec<u64> = graph.nodes.iter().map(|n| n.uid).collect();
    if nodes.is_empty() {
        return BenchStats {
            runs: 0,
            successful_routes: 0,
            min_ms: 0.0,
            max_ms: 0.0,
            avg_ms: 0.0,
        };
    }

    let config = RouteConfig {
        prefer_speed: false,
        cost_mode: CostMode::Distance,
    };

    let mut successful_routes = 0usize;
    let mut total_ms = 0.0;
    let mut min_ms = f64::INFINITY;
    let mut max_ms: f64 = 0.0;

    let mut completed = 0usize;
    let mut safety_counter = 0usize;
    while completed < runs && safety_counter < runs.saturating_mul(10).max(1) {
        safety_counter += 1;

        let start_uid = nodes[random_index(&mut seed, nodes.len())];
        let goal_uid = nodes[random_index(&mut seed, nodes.len())];
        if start_uid == goal_uid {
            continue;
        }

        let start = Instant::now();
        let result = plan_route_on_graph(graph, start_uid, goal_uid, &config);
        let elapsed_ms = start.elapsed().as_secs_f64() * 1000.0;

        if result.is_some() {
            successful_routes += 1;
            min_ms = min_ms.min(elapsed_ms);
            max_ms = max_ms.max(elapsed_ms);
            total_ms += elapsed_ms;
        }

        completed += 1;
    }

    let avg_ms = if successful_routes == 0 {
        0.0
    } else {
        total_ms / successful_routes as f64
    };
    if successful_routes == 0 {
        min_ms = 0.0;
    }

    BenchStats {
        runs: completed,
        successful_routes,
        min_ms,
        max_ms,
        avg_ms,
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
        eprintln!("Usage: cargo run --bin benchmark -- <graph.json>");
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

    if graph.nodes.is_empty() {
        eprintln!("graph has no nodes");
        std::process::exit(1);
    }

    let seed = 0xD15EA5E5u64;
    let stats = run_benchmark(&graph, RUNS, seed);

    println!("Benchmark completed: {} routes", stats.runs);
    println!("Average planning time: {:.4} ms", stats.avg_ms);
    println!("Min planning time: {:.4} ms", stats.min_ms);
    println!("Max planning time: {:.4} ms", stats.max_ms);
    println!("Successful routes: {}", stats.successful_routes);
}

#[cfg(test)]
mod tests {
    use super::*;
    use truckpilot::graph_schema::{GraphEdge, GraphNode, QualityMeta};

    fn mini_graph() -> GraphData {
        GraphData {
            meta: QualityMeta {
                schema_version: "1.0.0".to_string(),
                map_name: "mini".to_string(),
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
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
            ],
            edges: vec![GraphEdge {
                edge_uid: 100,
                from_node_uid: 1,
                to_node_uid: 2,
                from_lane_uid: None,
                to_lane_uid: None,
                road_uid: Some("0x01".to_string()),
                distance_m: 1.0,
                direction: "forward".to_string(),
                lane_count: 1,
                speed_limit_kmh: Some(50.0),
                flags: Vec::new(),
            }],
        }
    }

    #[test]
    fn test_benchmark_runs() {
        let graph = mini_graph();
        let stats = run_benchmark(&graph, 100, 123456);
        assert_eq!(stats.runs, 100);
        assert!(stats.successful_routes < 100);
        assert!(stats.max_ms >= stats.min_ms);
    }
}
