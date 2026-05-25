//! `uid-lookup` — Phase 6.5c Task 3
//!
//! Generic UID diagnostic: look up any UID in graph.json and report its
//! position, connected edges, and sector. If the UID is not found, reports
//! the nearest known UID.
//!
//! Usage:
//!   uid-lookup <UID> [--graph PATH]
//!   uid-lookup 6526933291294064640
//!   uid-lookup 0x5A94533F4C410000 --graph /path/to/graph.json
//!
//! UID formats accepted: decimal, 0x-prefixed hex.

use std::collections::HashMap;
use std::path::PathBuf;

use truckpilot_map_parser::graph::MapGraph;

struct Args {
    uid_raw: String,
    graph: PathBuf,
}

fn parse_args() -> Args {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    if argv.is_empty() || argv.iter().any(|a| a == "--help" || a == "-h") {
        eprintln!("usage: uid-lookup <UID> [--graph PATH]");
        eprintln!("  UID: decimal or 0x-hex");
        std::process::exit(0);
    }

    let mut uid_raw = argv[0].clone();
    let mut graph = PathBuf::from("graph.json");
    let mut i = 1;
    while i < argv.len() {
        match argv[i].as_str() {
            "--graph" if i + 1 < argv.len() => {
                graph = PathBuf::from(&argv[i + 1]);
                i += 2;
            }
            other if uid_raw == argv[0] && i > 0 => {
                // Fallback: treat as UID if no UID-looking arg given first
                uid_raw = other.to_string();
                i += 1;
            }
            other => {
                eprintln!("unknown argument: {other}");
                std::process::exit(2);
            }
        }
    }
    Args { uid_raw, graph }
}

fn parse_uid(raw: &str) -> Option<u64> {
    let s = raw.trim();
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u64::from_str_radix(hex, 16).ok()
    } else {
        s.parse::<u64>().ok()
    }
}

fn main() {
    let args = parse_args();

    // Parse UID.
    let uid = match parse_uid(&args.uid_raw) {
        Some(u) => u,
        None => {
            eprintln!(
                "ERROR: cannot parse '{}' as u64 (expected decimal or 0x-hex)",
                args.uid_raw
            );
            std::process::exit(2);
        }
    };

    println!("UID parsed as : {} (decimal) / 0x{:X} (hex)", uid, uid);

    // Load graph.
    eprintln!("Loading {}…", args.graph.display());
    let bytes = match std::fs::read(&args.graph) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("ERROR: cannot read {}: {e}", args.graph.display());
            eprintln!("Run the map-build first to generate graph.json.");
            std::process::exit(2);
        }
    };
    let graph: MapGraph = match serde_json::from_slice(&bytes) {
        Ok(g) => g,
        Err(e) => {
            eprintln!("ERROR: cannot parse {}: {e}", args.graph.display());
            std::process::exit(2);
        }
    };
    eprintln!(
        "Graph: {} nodes / {} edges",
        graph.nodes.len(),
        graph.edges.len()
    );

    // Build lookup structures.
    let positions: HashMap<u64, (f64, f64, f64)> = graph
        .nodes
        .iter()
        .map(|n| (n.uid, (n.x, n.y, n.z)))
        .collect();
    let mut outgoing: HashMap<u64, usize> = HashMap::new();
    let mut incoming: HashMap<u64, usize> = HashMap::new();
    for e in &graph.edges {
        *outgoing.entry(e.from).or_insert(0) += 1;
        *incoming.entry(e.to).or_insert(0) += 1;
    }

    // Look up the requested UID.
    if let Some(&(x, y, z)) = positions.get(&uid) {
        let out_edges = outgoing.get(&uid).copied().unwrap_or(0);
        let in_edges = incoming.get(&uid).copied().unwrap_or(0);
        println!("Found in graph : yes");
        println!("Position       : (x={x:.2}, y={y:.2}, z={z:.2})");
        println!("Outgoing edges : {out_edges}");
        println!("Incoming edges : {in_edges}");
        println!("Total edges    : {}", out_edges + in_edges);
        std::process::exit(0);
    }

    // Not found — find nearest.
    println!("Found in graph : NO");
    println!();
    println!("Searching for nearest UID…");

    let target_x = 0.0f64; // No position known for unknown UID.
    let target_z = 0.0f64;
    // We can't do a meaningful spatial search without a position for the
    // missing UID. Instead report the 5 numerically closest UIDs (by value).
    let mut all_uids: Vec<u64> = positions.keys().copied().collect();
    all_uids.sort();

    // Binary search for insertion point.
    let pos = all_uids.partition_point(|&u| u < uid);
    let start = pos.saturating_sub(3);
    let end = (pos + 3).min(all_uids.len());
    let nearby = &all_uids[start..end];

    println!(
        "Numerically adjacent UIDs (graph has {} nodes):",
        graph.nodes.len()
    );
    for &n_uid in nearby {
        let marker = if n_uid == uid {
            " ← REQUESTED (missing)"
        } else {
            ""
        };
        let (nx, _ny, nz) = positions[&n_uid];
        let out = outgoing.get(&n_uid).copied().unwrap_or(0);
        println!("  uid={n_uid:>22}  pos=({nx:.0}, {nz:.0})  edges_out={out}{marker}");
    }

    // Also find the nearest node by Euclidean distance if we know the target
    // position (we don't for an arbitrary UID, but report a placeholder).
    println!();
    println!("Note: spatial nearest-node lookup requires a reference position.");
    println!("      Use validate-cities to find the nearest UID by x/z coordinate.");

    // Report whether this UID looks like it has a sector tag in its high bits.
    // ETS2 UIDs encode sector in the upper 32 bits.
    let sector_x = (uid >> 32) as i32 >> 16;
    let sector_z = (uid >> 32) as i32 & 0xFFFF;
    println!();
    println!(
        "UID sector hint: high32=0x{:08X} (sector row≈{}, col≈{})",
        uid >> 32,
        sector_x,
        sector_z
    );

    let _ = target_x;
    let _ = target_z;

    std::process::exit(1);
}
