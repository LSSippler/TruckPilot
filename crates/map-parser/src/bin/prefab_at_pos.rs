//! `truckpilot-prefab-at-pos` — Diagnostic: list Prefab NavCurve segments near a world position.
//!
//! Loads the full SplineIndex (road + prefab) and queries within_radius_with_idx
//! at the given (x, z) to show which prefab segments (is_prefab=true) are present.
//!
//! Usage:
//!
//! ```powershell
//! cargo run --release --bin truckpilot-prefab-at-pos -- --graph graph.json --x 9106 --z -10001 --radius 30
//! ```

use std::path::PathBuf;
use truckpilot_map_parser::{
    build_index_with_metadata,
    build_splines_ex,
    MapGraph,
    spline::evaluate_heading_deg,
};

struct Args {
    graph: PathBuf,
    x: f32,
    z: f32,
    radius: f32,
}

fn parse_args() -> Args {
    let mut graph = PathBuf::from("graph.json");
    let mut x: Option<f32> = None;
    let mut z: Option<f32> = None;
    let mut radius = 30.0f32;
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--graph" => {
                graph = PathBuf::from(argv.get(i + 1).expect("--graph needs value"));
                i += 2;
            }
            "--x" => {
                x = Some(argv.get(i + 1).expect("--x needs value").parse().expect("--x must be f32"));
                i += 2;
            }
            "--z" => {
                z = Some(argv.get(i + 1).expect("--z needs value").parse().expect("--z must be f32"));
                i += 2;
            }
            "--radius" => {
                radius = argv.get(i + 1).expect("--radius needs value").parse().expect("--radius must be f32");
                i += 2;
            }
            "-h" | "--help" => {
                eprintln!("usage: truckpilot-prefab-at-pos --x <f32> --z <f32> [--radius <f32>] [--graph <PATH>]");
                std::process::exit(0);
            }
            other => {
                eprintln!("unknown argument: {other}");
                std::process::exit(2);
            }
        }
    }
    Args {
        graph,
        x: x.expect("--x is required"),
        z: z.expect("--z is required"),
        radius,
    }
}

/// Approximate distance from query (qx, qz) to nearest point on the segment chord,
/// sampled at 5 evenly-spaced t values (t=0, 0.25, 0.5, 0.75, 1.0).
fn approx_dist_to_seg(
    p0: (f32, f32),
    p1: (f32, f32),
    qx: f32,
    qz: f32,
) -> f32 {
    (0..=4)
        .map(|i| {
            let t = i as f32 / 4.0;
            let px = p0.0 + t * (p1.0 - p0.0);
            let pz = p0.1 + t * (p1.1 - p0.1);
            ((px - qx).powi(2) + (pz - qz).powi(2)).sqrt()
        })
        .fold(f32::INFINITY, f32::min)
}

/// Check whether a prefab_instance's AABB (approximated from origin_pos) covers the query.
fn instance_dist(origin: &[f32; 3], qx: f32, qz: f32) -> f32 {
    ((origin[0] - qx).powi(2) + (origin[2] - qz).powi(2)).sqrt()
}

fn main() {
    let args = parse_args();

    // -----------------------------------------------------------------------
    // 1. Load graph
    // -----------------------------------------------------------------------
    eprintln!("loading {} …", args.graph.display());
    let bytes = std::fs::read(&args.graph)
        .unwrap_or_else(|e| panic!("cannot read {}: {e}", args.graph.display()));
    let graph: MapGraph = serde_json::from_slice(&bytes)
        .unwrap_or_else(|e| panic!("cannot parse {}: {e}", args.graph.display()));
    eprintln!(
        "graph: {} nodes  {} edges  {} prefab_instances  {} prefab_ai_paths",
        graph.nodes.len(),
        graph.edges.len(),
        graph.prefab_instances.len(),
        graph.prefab_ai_paths.len(),
    );

    // -----------------------------------------------------------------------
    // 2. Build SplineIndex (road + prefab)
    // -----------------------------------------------------------------------
    let (mut segs, mut meta, _stats) = build_splines_ex(&graph);
    let road_seg_count = segs.len();

    let (prefab_segs, prefab_meta) = graph.prefab_hermite_segments_with_metadata();
    let prefab_seg_count = prefab_segs.len();
    segs.extend(prefab_segs);
    meta.extend(prefab_meta);

    eprintln!(
        "SplineIndex: {} road + {} prefab = {} total segments",
        road_seg_count, prefab_seg_count, segs.len()
    );

    let index = build_index_with_metadata(segs, meta);

    // -----------------------------------------------------------------------
    // 3. Radius query
    // -----------------------------------------------------------------------
    let query = truckpilot_map_parser::spline::Vec3::new(args.x, 0.0, args.z);
    let hits = index.within_radius_with_idx(query, args.radius);

    let prefab_hits: Vec<_> = hits
        .iter()
        .filter(|(_, _, m)| m.map(|mm| mm.is_prefab).unwrap_or(false))
        .collect();
    let road_hits: Vec<_> = hits
        .iter()
        .filter(|(_, _, m)| !m.map(|mm| mm.is_prefab).unwrap_or(false))
        .collect();

    // -----------------------------------------------------------------------
    // 4. Summary
    // -----------------------------------------------------------------------
    println!();
    println!(
        "=== PREFAB-AT-POS  x={:.0}  z={:.0}  r={:.0}m ===",
        args.x, args.z, args.radius
    );
    println!("Total segments in radius : {}", hits.len());
    println!("  is_prefab=true  (NavCurves) : {}", prefab_hits.len());
    println!("  is_prefab=false (Road segs) : {}", road_hits.len());
    println!();

    // -----------------------------------------------------------------------
    // 5. Prefab segments detail
    // -----------------------------------------------------------------------
    if prefab_hits.is_empty() {
        println!(">>> NO PREFAB NAVICURVES IN RADIUS <<<");
        println!();
    } else {
        // Sort by approx distance to query
        let mut rows: Vec<(usize, f32, f32)> = prefab_hits
            .iter()
            .map(|(idx, seg, _)| {
                let dist = approx_dist_to_seg(
                    (seg.p0.x, seg.p0.z),
                    (seg.p1.x, seg.p1.z),
                    args.x,
                    args.z,
                );
                let heading = evaluate_heading_deg(seg, 0.5);
                (*idx, dist, heading)
            })
            .collect();
        rows.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        println!("--- Prefab NavCurve segments (sorted by distance) ---");
        println!(
            "{:>7}  {:>20}  {:>20}  {:>9}  {:>9}  {:>9}  {:>9}  {:>10}  {:>8}",
            "idx", "from_uid", "to_uid",
            "start_x", "start_z", "end_x", "end_z",
            "dist_m", "hdg_deg"
        );
        for (idx, dist, heading) in &rows {
            let (_, seg, _) = prefab_hits
                .iter()
                .find(|(i, _, _)| i == idx)
                .unwrap();
            println!(
                "{:>7}  {:>20}  {:>20}  {:>9.1}  {:>9.1}  {:>9.1}  {:>9.1}  {:>10.2}  {:>8.1}",
                idx,
                seg.from_uid,
                seg.to_uid,
                seg.p0.x, seg.p0.z,
                seg.p1.x, seg.p1.z,
                dist,
                heading
            );
        }
        println!();
    }

    // -----------------------------------------------------------------------
    // 6. Road segments (brief)
    // -----------------------------------------------------------------------
    let show_road = road_hits.len().min(10);
    println!(
        "--- Road segments (first {show_road} of {}) ---",
        road_hits.len()
    );
    if road_hits.is_empty() {
        println!("  (none)");
    } else {
        println!(
            "{:>7}  {:>9}  {:>9}  {:>9}  {:>9}  {:>10}  {:>8}",
            "idx", "start_x", "start_z", "end_x", "end_z", "dist_m", "hdg_deg"
        );
        let mut road_rows: Vec<(usize, f32, f32)> = road_hits
            .iter()
            .map(|(idx, seg, _)| {
                let dist = approx_dist_to_seg(
                    (seg.p0.x, seg.p0.z),
                    (seg.p1.x, seg.p1.z),
                    args.x,
                    args.z,
                );
                let heading = evaluate_heading_deg(seg, 0.5);
                (*idx, dist, heading)
            })
            .collect();
        road_rows.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        for (idx, dist, heading) in road_rows.iter().take(10) {
            let (_, seg, _) = road_hits
                .iter()
                .find(|(i, _, _)| i == idx)
                .unwrap();
            println!(
                "{:>7}  {:>9.1}  {:>9.1}  {:>9.1}  {:>9.1}  {:>10.2}  {:>8.1}",
                idx,
                seg.p0.x, seg.p0.z,
                seg.p1.x, seg.p1.z,
                dist,
                heading
            );
        }
    }
    println!();

    // -----------------------------------------------------------------------
    // 7. Coverage analysis: prefab instances near query (always shown)
    // -----------------------------------------------------------------------
    let search_r = args.radius * 3.0;
    let nearby_instances: Vec<_> = graph
        .prefab_instances
        .iter()
        .filter(|inst| instance_dist(&inst.origin_pos, args.x, args.z) < search_r)
        .collect();

    println!(
        "=== COVERAGE ANALYSIS (prefab instances within {:.0}m) ===",
        search_r
    );
    println!("graph.prefab_instances total : {}", graph.prefab_instances.len());
    println!("graph.prefab_ai_paths  total : {}", graph.prefab_ai_paths.len());
    println!("Instances within {:.0}m        : {}", search_r, nearby_instances.len());
    println!();

    if nearby_instances.is_empty() {
        println!("VERDICT: No prefab_instance found → likely a Road-Road crossing (no PPD).");
    } else {
        println!(
            "{:>20}  {:>20}  {:>9}  {:>9}  {:>8}  {:>10}",
            "uid", "token", "origin_x", "origin_z", "dist_m", "node_count"
        );
        for inst in &nearby_instances {
            let dist = instance_dist(&inst.origin_pos, args.x, args.z);
            println!(
                "{:>20}  {:>20}  {:>9.1}  {:>9.1}  {:>8.1}  {:>10}",
                inst.uid,
                inst.token,
                inst.origin_pos[0],
                inst.origin_pos[2],
                dist,
                inst.node_uids.len(),
            );
        }
        println!();

        // Count how many ai_paths connect through these instance nodes
        let instance_node_uids: std::collections::HashSet<u64> = nearby_instances
            .iter()
            .flat_map(|inst| inst.node_uids.iter().copied())
            .collect();

        let paths_via_instance: Vec<_> = graph
            .prefab_ai_paths
            .iter()
            .filter(|p| {
                instance_node_uids.contains(&p.from_node_uid)
                    || instance_node_uids.contains(&p.to_node_uid)
            })
            .collect();

        println!(
            "PrefabAiPaths referencing these instance nodes: {}",
            paths_via_instance.len()
        );

        if paths_via_instance.is_empty() {
            println!("VERDICT: Prefab instance exists BUT zero NavCurves generated → PPD parsing gap.");
            println!("         (descriptor missing / PPD load failed for token)");
        } else {
            // Check if those paths produce segments within radius
            println!(
                "VERDICT: {} NavCurve paths exist for nearby instance(s), but none ended up within {:.0}m.",
                paths_via_instance.len(), args.radius
            );
            println!("         NavCurve positions (first 5):");
            for path in paths_via_instance.iter().take(5) {
                if let Some(pt) = path.spline_points.first() {
                    let d = ((pt[0] - args.x).powi(2) + (pt[2] - args.z).powi(2)).sqrt();
                    println!(
                        "           from={} to={}  start=({:.1},{:.1})  dist_to_query={:.1}m",
                        path.from_node_uid, path.to_node_uid, pt[0], pt[2], d
                    );
                }
            }
        }
    }
}
