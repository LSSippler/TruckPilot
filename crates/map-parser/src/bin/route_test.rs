//! `truckpilot-route-test` — Phase 5.9 routing smoke test
//!
//! Loads a `MapGraph` (JSON) and a `cities.toml` file, snaps each city to its
//! nearest graph node (within 5 km), then runs A* between every pair of
//! cities and reports per-pair distance / hop count / pass-or-fail and an
//! overall success-rate summary.
//!
//! Usage:
//!
//! ```powershell
//! cargo run --release --bin truckpilot-route-test -- `
//!   --graph graph.json `
//!   --cities crates\map-parser\tests\fixtures\test_cities.toml
//! ```

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};
use std::path::PathBuf;
use std::time::Instant;

use truckpilot_map_parser::graph::MapGraph;

const NEAREST_NODE_RADIUS_M: f64 = 5_000.0;

#[derive(Debug)]
struct Args {
    graph: PathBuf,
    cities: PathBuf,
}

fn parse_args() -> Args {
    let mut graph = PathBuf::from("graph.json");
    let mut cities = PathBuf::from("crates/map-parser/tests/fixtures/test_cities.toml");
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--graph" => {
                graph = PathBuf::from(argv.get(i + 1).expect("--graph needs value"));
                i += 2;
            }
            "--cities" => {
                cities = PathBuf::from(argv.get(i + 1).expect("--cities needs value"));
                i += 2;
            }
            "-h" | "--help" => {
                eprintln!("usage: truckpilot-route-test [--graph <PATH>] [--cities <PATH>]");
                std::process::exit(0);
            }
            other => {
                eprintln!("unknown argument: {other}");
                std::process::exit(2);
            }
        }
    }
    Args { graph, cities }
}

#[derive(Debug, Clone)]
struct City {
    name: String,
    x: f64,
    z: f64,
}

/// Tiny line-based TOML reader limited to the `[[city]] name=… x=… z=…`
/// shape our fixture uses — keeps the binary dependency-free.
fn read_cities(path: &PathBuf) -> Vec<City> {
    let text =
        std::fs::read_to_string(path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));

    let mut cities: Vec<City> = Vec::new();
    let mut cur_name: Option<String> = None;
    let mut cur_x: Option<f64> = None;
    let mut cur_z: Option<f64> = None;

    fn flush(
        list: &mut Vec<City>,
        name: &mut Option<String>,
        x: &mut Option<f64>,
        z: &mut Option<f64>,
    ) {
        if let (Some(n), Some(xv), Some(zv)) = (name.take(), x.take(), z.take()) {
            list.push(City {
                name: n,
                x: xv,
                z: zv,
            });
        }
    }

    for raw in text.lines() {
        let line = raw.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        if line == "[[city]]" {
            flush(&mut cities, &mut cur_name, &mut cur_x, &mut cur_z);
            continue;
        }
        if let Some((key, val)) = line.split_once('=') {
            let key = key.trim();
            let val = val.trim();
            match key {
                "name" => {
                    cur_name = Some(val.trim_matches('"').to_string());
                }
                "x" => {
                    cur_x = val.parse::<f64>().ok();
                }
                "z" => {
                    cur_z = val.parse::<f64>().ok();
                }
                _ => {}
            }
        }
    }
    flush(&mut cities, &mut cur_name, &mut cur_x, &mut cur_z);
    cities
}

fn main() {
    let args = parse_args();

    eprintln!("loading {} …", args.graph.display());
    let bytes =
        std::fs::read(&args.graph).unwrap_or_else(|e| panic!("read {}: {e}", args.graph.display()));
    let graph: MapGraph = serde_json::from_slice(&bytes)
        .unwrap_or_else(|e| panic!("parse {}: {e}", args.graph.display()));
    eprintln!(
        "graph: {} nodes / {} edges",
        graph.nodes.len(),
        graph.edges.len()
    );

    let cities = read_cities(&args.cities);
    eprintln!(
        "loaded {} cities from {}",
        cities.len(),
        args.cities.display()
    );
    if cities.is_empty() {
        eprintln!("ERROR: no cities parsed");
        std::process::exit(1);
    }

    // Snap each city to its nearest graph node within 5 km.
    let mut city_nodes: Vec<(City, Option<u64>, f64)> = Vec::with_capacity(cities.len());
    for city in &cities {
        let mut best_uid: Option<u64> = None;
        let mut best_d2 = f64::INFINITY;
        for n in &graph.nodes {
            let dx = n.x - city.x;
            let dz = n.z - city.z;
            let d2 = dx * dx + dz * dz;
            if d2 < best_d2 {
                best_d2 = d2;
                best_uid = Some(n.uid);
            }
        }
        let dist = best_d2.sqrt();
        let snapped = if dist <= NEAREST_NODE_RADIUS_M {
            best_uid
        } else {
            None
        };
        city_nodes.push((city.clone(), snapped, dist));
    }

    println!("=== CITY → NEAREST NODE ===");
    for (city, uid, dist) in &city_nodes {
        match uid {
            Some(u) => println!("  {:<10} → 0x{:016X}  ({:.0} m)", city.name, u, dist),
            None => println!(
                "  {:<10} → NO NODE within {:.0} m radius (closest {:.0} m)",
                city.name, NEAREST_NODE_RADIUS_M, dist
            ),
        }
    }
    println!();

    // Adjacency from graph.edges, treated as undirected so we measure
    // weakly-connected reachability between cities.
    let adj = build_adjacency(&graph);

    let mut pair_results: Vec<PairResult> = Vec::new();
    for (i, (from_city, from_uid, _)) in city_nodes.iter().enumerate() {
        for (j, (to_city, to_uid, _)) in city_nodes.iter().enumerate() {
            if i == j {
                continue;
            }
            let res = match (from_uid, to_uid) {
                (Some(a), Some(b)) => {
                    let t0 = Instant::now();
                    let path = a_star(&graph, &adj, *a, *b);
                    let elapsed_ms = t0.elapsed().as_secs_f64() * 1000.0;
                    PairResult {
                        from: from_city.name.clone(),
                        to: to_city.name.clone(),
                        path,
                        elapsed_ms,
                    }
                }
                _ => PairResult {
                    from: from_city.name.clone(),
                    to: to_city.name.clone(),
                    path: None,
                    elapsed_ms: 0.0,
                },
            };
            pair_results.push(res);
        }
    }

    print_pair_results(&pair_results);
}

#[derive(Debug, Clone)]
struct PairResult {
    from: String,
    to: String,
    path: Option<PathInfo>,
    elapsed_ms: f64,
}

#[derive(Debug, Clone)]
struct PathInfo {
    distance_m: f64,
    hops: usize,
}

fn print_pair_results(results: &[PairResult]) {
    let total = results.len();
    let success = results.iter().filter(|r| r.path.is_some()).count();
    let success_rate = if total == 0 {
        0.0
    } else {
        100.0 * success as f64 / total as f64
    };

    println!("=== ROUTING RESULTS ({success}/{total} pairs, {success_rate:.1}%) ===");
    println!();
    for r in results {
        match &r.path {
            Some(p) => println!(
                "  {:<10} → {:<10}  {:>9.1} km  {:>5} hops  {:>6.1} ms",
                r.from,
                r.to,
                p.distance_m / 1000.0,
                p.hops,
                r.elapsed_ms
            ),
            None => println!(
                "  {:<10} → {:<10}  NO PATH                            {:>6.1} ms",
                r.from, r.to, r.elapsed_ms
            ),
        }
    }
    println!();

    let succ_paths: Vec<&PathInfo> = results.iter().filter_map(|r| r.path.as_ref()).collect();
    if !succ_paths.is_empty() {
        let avg_dist: f64 =
            succ_paths.iter().map(|p| p.distance_m).sum::<f64>() / succ_paths.len() as f64;
        let avg_hops: f64 =
            succ_paths.iter().map(|p| p.hops as f64).sum::<f64>() / succ_paths.len() as f64;
        let max_dist = succ_paths
            .iter()
            .map(|p| p.distance_m)
            .fold(0.0_f64, f64::max);
        let min_dist = succ_paths
            .iter()
            .map(|p| p.distance_m)
            .fold(f64::INFINITY, f64::min);
        println!("=== SUMMARY ===");
        println!("Success rate : {success_rate:.1}%  ({success}/{total})");
        println!("Avg distance : {:.1} km", avg_dist / 1000.0);
        println!("Avg hops     : {avg_hops:.1}");
        println!(
            "Min / Max    : {:.1} km / {:.1} km",
            min_dist / 1000.0,
            max_dist / 1000.0
        );

        let mut sorted: Vec<&PairResult> = results.iter().filter(|r| r.path.is_some()).collect();
        sorted.sort_by(|a, b| {
            b.path
                .as_ref()
                .unwrap()
                .distance_m
                .partial_cmp(&a.path.as_ref().unwrap().distance_m)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        println!();
        println!("Top 5 longest successful routes:");
        for r in sorted.iter().take(5) {
            let p = r.path.as_ref().unwrap();
            println!(
                "  {:<10} → {:<10}  {:>9.1} km  {:>5} hops",
                r.from,
                r.to,
                p.distance_m / 1000.0,
                p.hops
            );
        }
    }

    let failures: Vec<&PairResult> = results.iter().filter(|r| r.path.is_none()).collect();
    if !failures.is_empty() {
        println!();
        println!("Failures ({}/{}):", failures.len(), total);
        for r in failures.iter().take(10) {
            println!("  {:<10} → {:<10}", r.from, r.to);
        }
        if failures.len() > 10 {
            println!("  … and {} more", failures.len() - 10);
        }
    }
}

fn build_adjacency(graph: &MapGraph) -> HashMap<u64, Vec<(u64, f64)>> {
    let mut adj: HashMap<u64, Vec<(u64, f64)>> = HashMap::with_capacity(graph.edges.len());
    for e in &graph.edges {
        adj.entry(e.from).or_default().push((e.to, e.distance_m));
        adj.entry(e.to).or_default().push((e.from, e.distance_m));
    }
    adj
}

fn a_star(
    graph: &MapGraph,
    adj: &HashMap<u64, Vec<(u64, f64)>>,
    start: u64,
    goal: u64,
) -> Option<PathInfo> {
    if start == goal {
        return Some(PathInfo {
            distance_m: 0.0,
            hops: 0,
        });
    }

    let positions: HashMap<u64, (f64, f64)> =
        graph.nodes.iter().map(|n| (n.uid, (n.x, n.z))).collect();
    let &(gx, gz) = positions.get(&goal)?;

    let mut g_score: HashMap<u64, f64> = HashMap::new();
    let mut came_from: HashMap<u64, u64> = HashMap::new();
    let mut open: BinaryHeap<Reverse<(u64, u64)>> = BinaryHeap::new();
    g_score.insert(start, 0.0);
    open.push(Reverse((0, start)));

    let cap = (graph.nodes.len() / 4).max(1024);
    let mut steps = 0usize;
    while let Some(Reverse((_, cur))) = open.pop() {
        steps += 1;
        if steps > cap {
            return None;
        }
        if cur == goal {
            let mut hops = 0usize;
            let mut node = goal;
            while let Some(&prev) = came_from.get(&node) {
                hops += 1;
                node = prev;
                if node == start {
                    break;
                }
            }
            let total = g_score.get(&goal).copied().unwrap_or(f64::INFINITY);
            return Some(PathInfo {
                distance_m: total,
                hops,
            });
        }

        let cur_g = g_score.get(&cur).copied().unwrap_or(f64::INFINITY);
        let Some(neighbours) = adj.get(&cur) else {
            continue;
        };
        for &(next, edge_w) in neighbours {
            let tentative = cur_g + edge_w;
            if tentative < g_score.get(&next).copied().unwrap_or(f64::INFINITY) {
                g_score.insert(next, tentative);
                came_from.insert(next, cur);
                let h = if let Some(&(nx, nz)) = positions.get(&next) {
                    let dx = nx - gx;
                    let dz = nz - gz;
                    (dx * dx + dz * dz).sqrt()
                } else {
                    0.0
                };
                let f = (tentative + h) as u64;
                open.push(Reverse((f, next)));
            }
        }
    }
    None
}
