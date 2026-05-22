//! `truckpilot-graph-stats` — load a `MapGraph` from `graph.json` and print
//! connectivity / degree / bounding-box / edge-length statistics.
//!
//! Usage:
//!
//! ```powershell
//! cargo run --release --bin truckpilot-graph-stats -- --graph graph.json
//! ```

use std::collections::HashMap;
use std::path::PathBuf;

use truckpilot_map_parser::graph::MapGraph;

#[derive(Debug)]
struct Args {
    graph: PathBuf,
}

fn parse_args() -> Args {
    let mut graph = PathBuf::from("graph.json");
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--graph" => {
                graph = PathBuf::from(argv.get(i + 1).expect("--graph needs value"));
                i += 2;
            }
            "-h" | "--help" => {
                eprintln!("usage: truckpilot-graph-stats [--graph <PATH>]");
                std::process::exit(0);
            }
            other => {
                eprintln!("unknown argument: {other}");
                std::process::exit(2);
            }
        }
    }
    Args { graph }
}

fn main() {
    let args = parse_args();
    eprintln!("loading {} …", args.graph.display());
    let bytes =
        std::fs::read(&args.graph).unwrap_or_else(|e| panic!("read {}: {e}", args.graph.display()));
    let graph: MapGraph = serde_json::from_slice(&bytes)
        .unwrap_or_else(|e| panic!("parse {}: {e}", args.graph.display()));
    eprintln!(
        "loaded {} nodes, {} edges, {} prefabs, {} signs",
        graph.nodes.len(),
        graph.edges.len(),
        graph.prefabs.len(),
        graph.signs.len()
    );

    print_basic_stats(&graph);
    print_bounding_box(&graph);
    print_edge_lengths(&graph);
    let degrees = compute_degrees(&graph);
    print_degree_stats(&graph, &degrees);
    print_connectivity(&graph);
}

fn print_basic_stats(graph: &MapGraph) {
    println!("=== BASIC ===");
    println!("Nodes   : {}", graph.nodes.len());
    println!("Edges   : {}", graph.edges.len());
    println!("Prefabs : {}", graph.prefabs.len());
    println!("Signs   : {}", graph.signs.len());
    println!();
}

fn print_bounding_box(graph: &MapGraph) {
    if graph.nodes.is_empty() {
        return;
    }
    let mut x_min = f64::INFINITY;
    let mut x_max = f64::NEG_INFINITY;
    let mut y_min = f64::INFINITY;
    let mut y_max = f64::NEG_INFINITY;
    let mut z_min = f64::INFINITY;
    let mut z_max = f64::NEG_INFINITY;
    for n in &graph.nodes {
        if n.x < x_min {
            x_min = n.x;
        }
        if n.x > x_max {
            x_max = n.x;
        }
        if n.y < y_min {
            y_min = n.y;
        }
        if n.y > y_max {
            y_max = n.y;
        }
        if n.z < z_min {
            z_min = n.z;
        }
        if n.z > z_max {
            z_max = n.z;
        }
    }
    println!("=== BOUNDING BOX ===");
    println!(
        "x ∈ [{:.0}, {:.0}]  span {:.0}",
        x_min,
        x_max,
        x_max - x_min
    );
    println!(
        "y ∈ [{:.0}, {:.0}]  span {:.0}",
        y_min,
        y_max,
        y_max - y_min
    );
    println!(
        "z ∈ [{:.0}, {:.0}]  span {:.0}",
        z_min,
        z_max,
        z_max - z_min
    );
    println!();
}

fn print_edge_lengths(graph: &MapGraph) {
    if graph.edges.is_empty() {
        return;
    }
    let mut lengths: Vec<f64> = graph.edges.iter().map(|e| e.distance_m).collect();
    lengths.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let sum: f64 = lengths.iter().sum();
    let avg = sum / lengths.len() as f64;
    let median = lengths[lengths.len() / 2];
    let max = *lengths.last().unwrap();
    let p99 = lengths[(lengths.len() as f64 * 0.99) as usize];
    println!("=== EDGE LENGTHS (m) ===");
    println!("avg   : {avg:>10.1}");
    println!("median: {median:>10.1}");
    println!("p99   : {p99:>10.1}");
    println!("max   : {max:>10.1}");
    println!();
}

fn compute_degrees(graph: &MapGraph) -> HashMap<u64, u32> {
    let mut deg: HashMap<u64, u32> = HashMap::with_capacity(graph.nodes.len());
    for n in &graph.nodes {
        deg.insert(n.uid, 0);
    }
    for e in &graph.edges {
        if let Some(d) = deg.get_mut(&e.from) {
            *d += 1;
        }
        if let Some(d) = deg.get_mut(&e.to) {
            *d += 1;
        }
    }
    deg
}

fn print_degree_stats(graph: &MapGraph, degrees: &HashMap<u64, u32>) {
    if graph.nodes.is_empty() {
        return;
    }
    let mut all: Vec<u32> = degrees.values().copied().collect();
    all.sort_unstable();
    let isolated = all.iter().filter(|&&d| d == 0).count();
    let max = *all.last().unwrap();
    let median = all[all.len() / 2];
    let sum: u64 = all.iter().map(|&d| d as u64).sum();
    let avg = sum as f64 / all.len() as f64;
    println!("=== DEGREE ===");
    println!("avg     : {avg:>8.2}");
    println!("median  : {median}");
    println!("max     : {max}");
    println!(
        "isolated: {isolated} ({:.1}%)",
        100.0 * isolated as f64 / graph.nodes.len() as f64
    );
    println!();

    // Show a sample of isolated node UIDs — run `node-inspect --uid <UID>` for details.
    let mut isolated_uids: Vec<u64> = degrees
        .iter()
        .filter(|(_, &d)| d == 0)
        .map(|(&uid, _)| uid)
        .collect();
    isolated_uids.sort_unstable();
    let sample_n = isolated_uids.len().min(20);
    if sample_n > 0 {
        println!("=== ISOLATED NODE SAMPLE (first {sample_n} of {isolated}) ===");
        println!("  (run `truckpilot-node-inspect --uid <UID> --ets2-dir <DIR>` for details)");
        for uid in &isolated_uids[..sample_n] {
            println!("  uid={uid:#018x}  ({uid})");
        }
        println!();
    }
}

fn print_connectivity(graph: &MapGraph) {
    println!("=== CONNECTIVITY ===");

    let node_count = graph.nodes.len();
    if node_count == 0 {
        println!("(empty graph)");
        return;
    }

    let uid_to_idx: HashMap<u64, usize> = graph
        .nodes
        .iter()
        .enumerate()
        .map(|(i, n)| (n.uid, i))
        .collect();

    // Undirected adjacency from edges.
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); node_count];
    for e in &graph.edges {
        if let (Some(&a), Some(&b)) = (uid_to_idx.get(&e.from), uid_to_idx.get(&e.to)) {
            adj[a].push(b);
            adj[b].push(a);
        }
    }

    let mut component_of = vec![usize::MAX; node_count];
    let mut sizes: Vec<usize> = Vec::new();
    let mut queue: Vec<usize> = Vec::with_capacity(1024);

    for start in 0..node_count {
        if component_of[start] != usize::MAX {
            continue;
        }
        let cid = sizes.len();
        component_of[start] = cid;
        queue.clear();
        queue.push(start);
        let mut head = 0usize;
        let mut size = 0usize;
        while head < queue.len() {
            let cur = queue[head];
            head += 1;
            size += 1;
            for &nx in &adj[cur] {
                if component_of[nx] == usize::MAX {
                    component_of[nx] = cid;
                    queue.push(nx);
                }
            }
        }
        sizes.push(size);
    }

    sizes.sort_unstable_by(|a, b| b.cmp(a));
    println!("Components total       : {}", sizes.len());
    println!(
        "Largest component size : {}",
        sizes.first().copied().unwrap_or(0)
    );
    if !sizes.is_empty() {
        println!(
            "Largest CC coverage    : {:.1}%",
            100.0 * sizes[0] as f64 / node_count as f64
        );
    }
    println!(
        "Top 10 component sizes : {:?}",
        &sizes[..sizes.len().min(10)]
    );
    let singletons = sizes.iter().filter(|&&s| s == 1).count();
    println!("Singleton components   : {singletons}");
    println!();
}
