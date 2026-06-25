//! `graph-load-check` — diagnose `graph.json` loading without starting the daemon.
//!
//! ```powershell
//! cargo run -p truckpilot-core --bin graph-load-check -- --graph graph.json
//! ```

use std::path::PathBuf;
use std::process;
use std::time::Instant;

use truckpilot_map_parser::{
    build_index_with_metadata, build_splines_ex, load_map_graph_from_path, log_graph_load_stage,
    MapGraph,
};
use truckpilot_plugin_api::graph::RouterGraph;

#[derive(Debug)]
struct Args {
    graph: PathBuf,
    skip_spline: bool,
    skip_plan: bool,
}

fn parse_args() -> Args {
    let mut graph = PathBuf::from("graph.json");
    let mut skip_spline = false;
    let mut skip_plan = false;
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--graph" => {
                graph = PathBuf::from(
                    argv.get(i + 1)
                        .unwrap_or_else(|| {
                            eprintln!("--graph requires a path");
                            process::exit(2);
                        })
                        .clone(),
                );
                i += 2;
            }
            "--skip-spline" => {
                skip_spline = true;
                i += 1;
            }
            "--skip-plan" => {
                skip_plan = true;
                i += 1;
            }
            "-h" | "--help" => {
                eprintln!(
                    "usage: graph-load-check [--graph PATH] [--skip-spline] [--skip-plan]"
                );
                process::exit(0);
            }
            other => {
                eprintln!("unknown argument: {other}");
                process::exit(2);
            }
        }
    }
    Args {
        graph,
        skip_spline,
        skip_plan,
    }
}

fn build_router_graph(map_graph: &MapGraph, log_prefix: &str) -> RouterGraph {
    log_graph_load_stage(log_prefix, "build RouterGraph start");
    let nodes: Vec<(u64, f64, f64)> = map_graph.nodes.iter().map(|n| (n.uid, n.x, n.z)).collect();
    let edges: Vec<(u64, u64, f64)> = map_graph
        .edges
        .iter()
        .map(|e| (e.from, e.to, e.distance_m))
        .collect();
    log_graph_load_stage(
        log_prefix,
        &format!(
            "RouterGraph input: {} nodes, {} edges",
            nodes.len(),
            edges.len()
        ),
    );
    let router = RouterGraph::new(nodes, edges);
    log_graph_load_stage(log_prefix, "build RouterGraph done");
    router
}

fn main() {
    let args = parse_args();
    let log_prefix = "graph-load-check";
    let t0 = Instant::now();

    let map_graph = match load_map_graph_from_path(&args.graph, log_prefix) {
        Ok(g) => g,
        Err(e) => {
            eprintln!("ERROR: {e}");
            process::exit(1);
        }
    };

    let router = build_router_graph(&map_graph, log_prefix);

    if let Some(&(uid, x, z)) = router.nodes.first() {
        println!("first node uid={uid} pos=({x:.1}, {z:.1})");
        println!("has_node(first)={}", router.has_node(uid));
        println!("node_position(first)={:?}", router.node_position(uid));
    } else {
        println!("first node: (none)");
    }

    if let Some(&(from, to, dist)) = router.edges.first() {
        println!("first edge: {from} -> {to} ({dist:.1} m)");
        if !args.skip_plan {
            match router.plan(from, to) {
                Some((path, cost)) => {
                    println!("plan({from}, {to}): {} hops, cost={cost:.1} m", path.len());
                }
                None => {
                    println!("plan({from}, {to}): no route");
                }
            }
        }
    } else {
        println!("first edge: (none)");
    }

    if !args.skip_spline {
        log_graph_load_stage(log_prefix, "build SplineIndex start");
        let (mut segments, mut metadata, _stats) = build_splines_ex(&map_graph);
        let road_seg_count = segments.len();
        let (prefab_segs, prefab_meta) = map_graph.prefab_hermite_segments_with_metadata();
        segments.extend(prefab_segs);
        metadata.extend(prefab_meta);
        log_graph_load_stage(log_prefix, "indexes start");
        let index = build_index_with_metadata(segments, metadata);
        log_graph_load_stage(log_prefix, "indexes done");
        println!(
            "SplineIndex: {} road + {} prefab = {} total segments",
            road_seg_count,
            index.segments.len().saturating_sub(road_seg_count),
            index.segments.len()
        );
    }

    log_graph_load_stage(log_prefix, "done");
    println!(
        "OK: loaded {} in {:.1}s",
        args.graph.display(),
        t0.elapsed().as_secs_f64()
    );
}
