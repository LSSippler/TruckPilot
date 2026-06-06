//! `seg-inspect` — Segment + Nearest-Node Diagnostic (Phase 1a Blocker)
//!
//! Usage:
//!   seg-inspect --seg 556533 --truck-x -3186.641 --truck-z -1965.427 [--graph graph.json] [--top-n 10]
//!   seg-inspect --nearest-only --truck-x -3186.641 --truck-z -1965.427

use std::collections::HashMap;
use std::path::PathBuf;

use truckpilot_map_parser::graph::MapGraph;
use truckpilot_map_parser::spline::{build_splines, HermiteSegment};

// ---------------------------------------------------------------------------
// Args
// ---------------------------------------------------------------------------

struct Args {
    seg_idx: Option<usize>,
    truck_x: f64,
    truck_z: f64,
    graph: PathBuf,
    top_n: usize,
    radius_m: f64,
}

fn print_usage() {
    eprintln!("usage: seg-inspect [--seg IDX] [--truck-x X] [--truck-z Z] [--graph PATH] [--top-n N] [--radius M]");
    eprintln!("  --seg IDX       Segment index to inspect (optional)");
    eprintln!("  --truck-x X     Truck X coordinate (ETS2, meters)");
    eprintln!("  --truck-z Z     Truck Z coordinate (ETS2, meters)");
    eprintln!("  --graph PATH    Path to graph.json (default: graph.json)");
    eprintln!("  --top-n N       How many nearest nodes to print (default: 10)");
    eprintln!("  --radius M      Edge coverage radius in meters (default: 50.0)");
}

fn parse_args() -> Args {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    if argv.iter().any(|a| a == "--help" || a == "-h") {
        print_usage();
        std::process::exit(0);
    }

    let mut seg_idx: Option<usize> = None;
    let mut truck_x: Option<f64> = None;
    let mut truck_z: Option<f64> = None;
    let mut graph = PathBuf::from("graph.json");
    let mut top_n = 10usize;
    let mut radius_m = 50.0f64;

    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--seg" if i + 1 < argv.len() => {
                seg_idx = argv[i + 1].parse().ok();
                i += 2;
            }
            "--truck-x" if i + 1 < argv.len() => {
                truck_x = argv[i + 1].parse().ok();
                i += 2;
            }
            "--truck-z" if i + 1 < argv.len() => {
                truck_z = argv[i + 1].parse().ok();
                i += 2;
            }
            "--graph" if i + 1 < argv.len() => {
                graph = PathBuf::from(&argv[i + 1]);
                i += 2;
            }
            "--top-n" if i + 1 < argv.len() => {
                top_n = argv[i + 1].parse().unwrap_or(10);
                i += 2;
            }
            "--radius" if i + 1 < argv.len() => {
                radius_m = argv[i + 1].parse().unwrap_or(50.0);
                i += 2;
            }
            other => {
                eprintln!("unknown arg: {other}");
                print_usage();
                std::process::exit(2);
            }
        }
    }

    Args {
        seg_idx,
        truck_x: truck_x.unwrap_or(0.0),
        truck_z: truck_z.unwrap_or(0.0),
        graph,
        top_n,
        radius_m,
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn dist_xz(ax: f64, az: f64, bx: f64, bz: f64) -> f64 {
    let dx = ax - bx;
    let dz = az - bz;
    (dx * dx + dz * dz).sqrt()
}

fn seg_p0_xz(seg: &HermiteSegment) -> (f64, f64) {
    (seg.p0.x as f64, seg.p0.z as f64)
}

fn seg_p1_xz(seg: &HermiteSegment) -> (f64, f64) {
    (seg.p1.x as f64, seg.p1.z as f64)
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() {
    let args = parse_args();

    // Load graph.
    eprintln!("Loading {}…", args.graph.display());
    let bytes = match std::fs::read(&args.graph) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("ERROR: cannot read {}: {e}", args.graph.display());
            std::process::exit(2);
        }
    };
    let graph: MapGraph = match serde_json::from_slice(&bytes) {
        Ok(g) => g,
        Err(e) => {
            eprintln!("ERROR: cannot parse graph.json: {e}");
            std::process::exit(2);
        }
    };
    eprintln!(
        "Graph: {} nodes, {} edges",
        graph.nodes.len(),
        graph.edges.len()
    );

    // Build node position index.
    let node_pos: HashMap<u64, (f64, f64, f64)> = graph
        .nodes
        .iter()
        .map(|n| (n.uid, (n.x, n.y, n.z)))
        .collect();

    // Build edge maps (from/to → edge).
    let mut edges_by_from: HashMap<u64, Vec<usize>> = HashMap::new();
    let mut edges_by_to: HashMap<u64, Vec<usize>> = HashMap::new();
    for (i, e) in graph.edges.iter().enumerate() {
        edges_by_from.entry(e.from).or_default().push(i);
        edges_by_to.entry(e.to).or_default().push(i);
    }

    // Build splines.
    eprintln!("Building splines…");
    let (segments, stats) = build_splines(&graph);
    eprintln!(
        "Splines: {} segments built ({} skipped — missing nodes)",
        stats.total_segments, stats.skipped_missing_node
    );

    let tx = args.truck_x;
    let tz = args.truck_z;

    println!();
    println!("=== Truck Position ===");
    println!("  truck_x = {tx:.3}");
    println!("  truck_z = {tz:.3}");

    // -----------------------------------------------------------------------
    // Task 1: Segment Inspection
    // -----------------------------------------------------------------------

    if let Some(idx) = args.seg_idx {
        println!();
        println!("=== Task 1: Segment {} Inspection ===", idx);

        if idx >= segments.len() {
            println!(
                "ERROR: idx {} out of range (total={})!",
                idx,
                segments.len()
            );
        } else {
            let seg = &segments[idx];
            println!("  from_uid  : {}", seg.from_uid);
            println!("  to_uid    : {}", seg.to_uid);
            println!("  edge_uid  : {}", seg.edge_uid);
            println!("  length_m  : {:.3}", seg.length_m);
            println!(
                "  P0        : ({:.3}, {:.3}, {:.3})",
                seg.p0.x, seg.p0.y, seg.p0.z
            );
            println!(
                "  P1        : ({:.3}, {:.3}, {:.3})",
                seg.p1.x, seg.p1.y, seg.p1.z
            );
            println!(
                "  M0 (tang) : ({:.4}, {:.4}, {:.4})",
                seg.m0.x, seg.m0.y, seg.m0.z
            );
            println!(
                "  M1 (tang) : ({:.4}, {:.4}, {:.4})",
                seg.m1.x, seg.m1.y, seg.m1.z
            );

            // Task 2: Compare spline P0/P1 with graph node positions.
            println!();
            println!("=== Task 2: Spline P0/P1 vs Graph Node Positions ===");

            if let Some(&(nx, ny, nz)) = node_pos.get(&seg.from_uid) {
                let dx = seg.p0.x as f64 - nx;
                let dy = seg.p0.y as f64 - ny;
                let dz = seg.p0.z as f64 - nz;
                let dist = (dx * dx + dy * dy + dz * dz).sqrt();
                println!(
                    "  from_uid {} node pos = ({:.3}, {:.3}, {:.3})",
                    seg.from_uid, nx, ny, nz
                );
                println!(
                    "  seg.P0               = ({:.3}, {:.3}, {:.3})",
                    seg.p0.x, seg.p0.y, seg.p0.z
                );
                println!(
                    "  P0 delta             = ({:.4}, {:.4}, {:.4}) => dist={:.4}m",
                    dx, dy, dz, dist
                );
                if dist < 0.01 {
                    println!("  MATCH: P0 == node position (OK)");
                } else {
                    println!("  MISMATCH: P0 differs from node by {:.4}m !", dist);
                }
            } else {
                println!("  from_uid {} NOT FOUND in graph nodes!", seg.from_uid);
            }

            if let Some(&(nx, ny, nz)) = node_pos.get(&seg.to_uid) {
                let dx = seg.p1.x as f64 - nx;
                let dy = seg.p1.y as f64 - ny;
                let dz = seg.p1.z as f64 - nz;
                let dist = (dx * dx + dy * dy + dz * dz).sqrt();
                println!(
                    "  to_uid {} node pos   = ({:.3}, {:.3}, {:.3})",
                    seg.to_uid, nx, ny, nz
                );
                println!(
                    "  seg.P1               = ({:.3}, {:.3}, {:.3})",
                    seg.p1.x, seg.p1.y, seg.p1.z
                );
                println!(
                    "  P1 delta             = ({:.4}, {:.4}, {:.4}) => dist={:.4}m",
                    dx, dy, dz, dist
                );
                if dist < 0.01 {
                    println!("  MATCH: P1 == node position (OK)");
                } else {
                    println!("  MISMATCH: P1 differs from node by {:.4}m !", dist);
                }
            } else {
                println!("  to_uid {} NOT FOUND in graph nodes!", seg.to_uid);
            }

            // Edge info from graph.
            if let Some(edge) = graph.edges.iter().find(|e| e.uid == seg.edge_uid) {
                println!();
                println!("=== Edge {} in Graph ===", seg.edge_uid);
                println!("  from={} to={}", edge.from, edge.to);
                println!(
                    "  distance_m={:.2}  speed_kmh={:?}  lanes={}",
                    edge.distance_m, edge.speed_limit_kmh, edge.lanes
                );
                println!(
                    "  direction={}  dlc_guard={}  gps_avoid={}",
                    edge.direction, edge.dlc_guard, edge.gps_avoid
                );
            } else {
                println!("  Edge uid {} not found in graph.edges!", seg.edge_uid);
            }

            // Distance from P0/P1 to truck.
            let (p0x, p0z) = seg_p0_xz(seg);
            let (p1x, p1z) = seg_p1_xz(seg);
            let d0 = dist_xz(tx, tz, p0x, p0z);
            let d1 = dist_xz(tx, tz, p1x, p1z);
            println!();
            println!("=== Segment {} Distance to Truck ===", idx);
            println!(
                "  dist(truck, P0) = {:.2}m  [P0=({:.1},{:.1})]",
                d0, p0x, p0z
            );
            println!(
                "  dist(truck, P1) = {:.2}m  [P1=({:.1},{:.1})]",
                d1, p1x, p1z
            );
            println!("  Truck is {}m from nearest endpoint", d0.min(d1));
        }
    }

    // -----------------------------------------------------------------------
    // Task 3: Nearest Nodes (brute-force XZ)
    // -----------------------------------------------------------------------

    println!();
    println!(
        "=== Task 3: Top {} Nearest Graph Nodes to Truck ===",
        args.top_n
    );
    println!("  Query: ({:.3}, {:.3})", tx, tz);

    let mut node_dists: Vec<(f64, u64, f64, f64, f64)> = graph
        .nodes
        .iter()
        .map(|n| {
            let d = dist_xz(tx, tz, n.x, n.z);
            (d, n.uid, n.x, n.y, n.z)
        })
        .collect();
    node_dists.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

    for (rank, &(d, uid, nx, ny, nz)) in node_dists.iter().take(args.top_n).enumerate() {
        let out = edges_by_from.get(&uid).map_or(0, |v| v.len());
        let inc = edges_by_to.get(&uid).map_or(0, |v| v.len());
        println!(
            "  #{:02}  dist={:7.2}m  uid={:>22}  pos=({:.1}, {:.1}, {:.1})  edges_out={} in={}",
            rank + 1,
            d,
            uid,
            nx,
            ny,
            nz,
            out,
            inc
        );
    }

    // -----------------------------------------------------------------------
    // Task 3b: Edges within radius
    // -----------------------------------------------------------------------

    println!();
    println!(
        "=== Task 3b: Edges within {:.0}m of Truck ===",
        args.radius_m
    );

    let mut near_edges: Vec<(f64, usize)> = Vec::new();
    for (i, seg) in segments.iter().enumerate() {
        let (p0x, p0z) = seg_p0_xz(seg);
        let (p1x, p1z) = seg_p1_xz(seg);
        let mid_x = (p0x + p1x) / 2.0;
        let mid_z = (p0z + p1z) / 2.0;
        let d_p0 = dist_xz(tx, tz, p0x, p0z);
        let d_p1 = dist_xz(tx, tz, p1x, p1z);
        let d_mid = dist_xz(tx, tz, mid_x, mid_z);
        let d_min = d_p0.min(d_p1).min(d_mid);
        if d_min <= args.radius_m {
            near_edges.push((d_min, i));
        }
    }
    near_edges.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

    if near_edges.is_empty() {
        println!(
            "  NO segments within {:.0}m — truck is in gap area!",
            args.radius_m
        );
    } else {
        println!(
            "  {} segments found within {:.0}m:",
            near_edges.len(),
            args.radius_m
        );
        for &(d, i) in near_edges.iter().take(20) {
            let seg = &segments[i];
            let (p0x, p0z) = seg_p0_xz(seg);
            let (p1x, p1z) = seg_p1_xz(seg);
            println!(
                "    seg {:>7}  d_min={:6.2}m  from={} to={}  P0=({:.0},{:.0}) P1=({:.0},{:.0})",
                i, d, seg.from_uid, seg.to_uid, p0x, p0z, p1x, p1z
            );
        }
        if near_edges.len() > 20 {
            println!("    … {} more", near_edges.len() - 20);
        }
    }

    // -----------------------------------------------------------------------
    // Task 4: Sector info for truck position
    // -----------------------------------------------------------------------

    println!();
    println!("=== Task 4: Sector Hint for Truck Position ===");
    // ETS2 sector size = 4096 units. Sector origin at (0,0) = sector (0,0).
    // sector_x = floor(x / 4096) + 256 (offset varies), typically:
    // sector index = floor((x + 32768) / 4096) etc. But simpler:
    let sector_x = (tx / 4096.0).floor() as i32;
    let sector_z = (tz / 4096.0).floor() as i32;
    println!(
        "  ETS2 sector grid (4096m cells): sector_x={}, sector_z={}",
        sector_x, sector_z
    );
    // Also check nearest node UIDs for sector hints (upper 32 bits).
    if let Some(&(_, uid, _, _, _)) = node_dists.first() {
        let high32 = (uid >> 32) as u32;
        println!("  Nearest node uid={}  high32=0x{:08X}", uid, high32);
        // ETS2 UID: bits[47:32] = sector_col (X), bits[31:16] = sector_row (Z).
        let sc = (high32 >> 16) as i32;
        let sr = (high32 & 0xFFFF) as i32;
        println!("  UID sector hint: col≈{}, row≈{}", sc, sr);
    }

    println!();
    println!("=== Summary ===");
    if args.seg_idx.is_none() {
        println!("  (No --seg specified; skipped segment inspection)");
    }
    if node_dists.first().map_or(999.0, |v| v.0) > args.radius_m {
        println!(
            "  FINDING: No graph node within {:.0}m. Truck is in unmapped gap.",
            args.radius_m
        );
    } else {
        println!(
            "  Nearest graph node is {:.2}m away from truck.",
            node_dists.first().map_or(0.0, |v| v.0)
        );
    }
    if near_edges.is_empty() {
        println!(
            "  FINDING: No spline segment within {:.0}m. SplineIndex gap confirmed.",
            args.radius_m
        );
    } else {
        println!(
            "  {} spline segments within {:.0}m.",
            near_edges.len(),
            args.radius_m
        );
    }
}
