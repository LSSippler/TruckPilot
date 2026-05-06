//! `route_stress` — pick 100 long-distance start/goal pairs and stress-test
//! the A* router.
//!
//! Usage:
//!     cargo run --bin route_stress -- output/graph.json
//!
//! Each candidate pair must be at least `MIN_DISTANCE_M` meters apart by
//! straight-line distance. The tool reports min/max/avg planning time,
//! min/max/avg expanded nodes, success rate and the 10 toughest routes
//! (sorted by planning time, ties broken by nodes expanded).

use std::fs::File;
use std::io::BufReader;
use std::path::PathBuf;
use std::time::Instant;

use truckpilot::autopilot::{plan_route_on_graph, CostMode, RouteConfig};
use truckpilot::graph_schema::GraphData;

const RUNS: usize = 100;
const MIN_DISTANCE_M: f64 = 50_000.0;
const MAX_ATTEMPTS_FACTOR: usize = 200;

#[derive(Debug, Clone)]
struct AttemptResult {
    start_uid: u64,
    goal_uid: u64,
    straight_km: f64,
    planning_ms: f64,
    nodes_expanded: u64,
    edges_examined: u64,
    success: bool,
}

#[derive(Debug, Default, Clone)]
struct StressStats {
    attempts: usize,
    pairs_evaluated: usize,
    successful: usize,
    min_ms: f64,
    max_ms: f64,
    avg_ms: f64,
    min_expanded: u64,
    max_expanded: u64,
    avg_expanded: f64,
}

fn lcg_next(state: &mut u64) -> u64 {
    *state = state
        .wrapping_mul(6_364_136_223_846_793_005)
        .wrapping_add(1);
    *state
}

fn random_index(state: &mut u64, upper: usize) -> usize {
    if upper <= 1 {
        return 0;
    }
    (lcg_next(state) as usize) % upper
}

fn euclid_2d(a: (f64, f64, f64), b: (f64, f64, f64)) -> f64 {
    let dx = a.0 - b.0;
    let dz = a.2 - b.2;
    (dx * dx + dz * dz).sqrt()
}

fn run_stress(graph: &GraphData, runs: usize, mut seed: u64) -> (StressStats, Vec<AttemptResult>) {
    let mut stats = StressStats::default();
    let mut attempts: Vec<AttemptResult> = Vec::with_capacity(runs);

    if graph.nodes.is_empty() {
        return (stats, attempts);
    }

    let positions: Vec<(u64, (f64, f64, f64))> = graph
        .nodes
        .iter()
        .map(|n| (n.uid, (n.x, n.y, n.z)))
        .collect();

    let config = RouteConfig {
        prefer_speed: false,
        cost_mode: CostMode::Distance,
    };

    let max_attempts = runs.saturating_mul(MAX_ATTEMPTS_FACTOR).max(runs);
    let mut total_ms = 0.0_f64;
    let mut total_expanded: u64 = 0;
    let mut min_ms = f64::INFINITY;
    let mut max_ms = 0.0_f64;
    let mut min_expanded = u64::MAX;
    let mut max_expanded: u64 = 0;
    let small_graph = positions.len() < 4;

    while stats.pairs_evaluated < runs && stats.attempts < max_attempts {
        stats.attempts += 1;

        let i = random_index(&mut seed, positions.len());
        let j = random_index(&mut seed, positions.len());
        if i == j {
            continue;
        }
        let (start_uid, sp) = positions[i];
        let (goal_uid, gp) = positions[j];

        let dist = euclid_2d(sp, gp);
        if !small_graph && dist < MIN_DISTANCE_M {
            continue;
        }

        let start = Instant::now();
        let result = plan_route_on_graph(graph, start_uid, goal_uid, &config);
        let planning_ms = start.elapsed().as_secs_f64() * 1000.0;

        let (success, nodes_expanded, edges_examined) = match result {
            Some(r) => (true, r.nodes_expanded, r.edges_examined),
            None => (false, 0, 0),
        };

        attempts.push(AttemptResult {
            start_uid,
            goal_uid,
            straight_km: dist / 1000.0,
            planning_ms,
            nodes_expanded,
            edges_examined,
            success,
        });

        stats.pairs_evaluated += 1;

        if success {
            stats.successful += 1;
            total_ms += planning_ms;
            min_ms = min_ms.min(planning_ms);
            max_ms = max_ms.max(planning_ms);
            total_expanded += nodes_expanded;
            min_expanded = min_expanded.min(nodes_expanded);
            max_expanded = max_expanded.max(nodes_expanded);
        }
    }

    if stats.successful == 0 {
        stats.min_ms = 0.0;
        stats.max_ms = 0.0;
        stats.avg_ms = 0.0;
        stats.min_expanded = 0;
        stats.max_expanded = 0;
        stats.avg_expanded = 0.0;
    } else {
        stats.min_ms = min_ms;
        stats.max_ms = max_ms;
        stats.avg_ms = total_ms / stats.successful as f64;
        stats.min_expanded = min_expanded;
        stats.max_expanded = max_expanded;
        stats.avg_expanded = total_expanded as f64 / stats.successful as f64;
    }

    (stats, attempts)
}

fn print_report(stats: &StressStats, attempts: &[AttemptResult]) {
    println!("Route stress test");
    println!("=================");
    println!(
        "Target runs            : {RUNS}  (min straight-line distance {:.0} km)",
        MIN_DISTANCE_M / 1000.0
    );
    println!("Attempts (incl. skips) : {}", stats.attempts);
    println!("Pairs evaluated        : {}", stats.pairs_evaluated);
    println!("Successful routes      : {}", stats.successful);
    let success_rate = if stats.pairs_evaluated == 0 {
        0.0
    } else {
        stats.successful as f64 * 100.0 / stats.pairs_evaluated as f64
    };
    println!("Success rate           : {success_rate:.1} %");
    println!();
    println!("Planning time (ms)");
    println!("  min : {:.4}", stats.min_ms);
    println!("  max : {:.4}", stats.max_ms);
    println!("  avg : {:.4}", stats.avg_ms);
    println!();
    println!("Nodes expanded");
    println!("  min : {}", stats.min_expanded);
    println!("  max : {}", stats.max_expanded);
    println!("  avg : {:.1}", stats.avg_expanded);
    println!();

    let mut sorted: Vec<&AttemptResult> = attempts.iter().filter(|a| a.success).collect();
    sorted.sort_by(|a, b| {
        b.planning_ms
            .partial_cmp(&a.planning_ms)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(b.nodes_expanded.cmp(&a.nodes_expanded))
    });

    println!("Top 10 hardest routes (planning time desc)");
    println!("------------------------------------------");
    if sorted.is_empty() {
        println!("  (no successful routes)");
    } else {
        println!(
            "  {:>4}  {:>10}  {:>10}  {:>10}  start -> goal (km)",
            "#", "ms", "expanded", "examined"
        );
        for (i, a) in sorted.iter().take(10).enumerate() {
            println!(
                "  {:>4}  {:>10.4}  {:>10}  {:>10}  0x{:X} -> 0x{:X}  ({:.1} km)",
                i + 1,
                a.planning_ms,
                a.nodes_expanded,
                a.edges_examined,
                a.start_uid,
                a.goal_uid,
                a.straight_km
            );
        }
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
        eprintln!("Usage: cargo run --bin route_stress -- <graph.json>");
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

    let seed = 0xC0FFEE_u64;
    let (stats, attempts) = run_stress(&graph, RUNS, seed);
    print_report(&stats, &attempts);
}

#[cfg(test)]
mod tests {
    use super::*;
    use truckpilot::graph_schema::{GraphEdge, GraphNode, QualityMeta};

    fn chain_graph(n: usize, step_m: f64) -> GraphData {
        let mut nodes = Vec::with_capacity(n);
        let mut edges = Vec::with_capacity(n.saturating_sub(1));
        for i in 0..n {
            nodes.push(GraphNode {
                uid: (i as u64) + 1,
                lane_uid: None,
                base_uid: None,
                lane_index: None,
                x: i as f64 * step_m,
                y: 0.0,
                z: 0.0,
            });
        }
        for i in 0..n.saturating_sub(1) {
            edges.push(GraphEdge {
                edge_uid: 1000 + i as u64,
                from_node_uid: (i as u64) + 1,
                to_node_uid: (i as u64) + 2,
                from_lane_uid: None,
                to_lane_uid: None,
                road_uid: None,
                distance_m: step_m,
                direction: "forward".to_string(),
                lane_count: 1,
                speed_limit_kmh: Some(80.0),
                flags: vec![],
            });
        }
        GraphData {
            meta: QualityMeta {
                schema_version: "1.0.0".to_string(),
                map_name: "stress_test".to_string(),
                generated_at: String::new(),
            },
            nodes,
            edges,
        }
    }

    #[test]
    fn deterministic_with_fixed_seed() {
        let graph = chain_graph(80, 1000.0);
        let (a, _) = run_stress(&graph, 10, 42);
        let (b, _) = run_stress(&graph, 10, 42);
        assert_eq!(a.pairs_evaluated, b.pairs_evaluated);
        assert_eq!(a.successful, b.successful);
    }

    #[test]
    fn small_graph_falls_back_to_any_distance() {
        // Tiny graph cannot satisfy the 50 km filter, but the small-graph
        // fallback ensures we still evaluate pairs.
        let graph = chain_graph(3, 100.0);
        let (stats, _attempts) = run_stress(&graph, 5, 7);
        assert!(stats.pairs_evaluated > 0);
    }

    #[test]
    fn empty_graph_yields_zero_stats() {
        let graph = GraphData {
            meta: QualityMeta {
                schema_version: "1.0.0".to_string(),
                map_name: "empty".to_string(),
                generated_at: String::new(),
            },
            nodes: vec![],
            edges: vec![],
        };
        let (stats, attempts) = run_stress(&graph, 10, 1);
        assert_eq!(stats.pairs_evaluated, 0);
        assert_eq!(attempts.len(), 0);
    }
}
