//! `truckpilot-node-inspect` — Phase 5.28-D Node-Diagnose CLI.
//!
//! Loads all `.base` sectors from `base_map.scs`, finds the node with the
//! requested UID, and prints all fields including `forward_item_uid` and
//! `backward_item_uid`.  Optionally cross-references a `graph.json` to
//! show edge degree and connectivity.
//!
//! Usage:
//!
//! ```powershell
//! cargo run --release --bin truckpilot-node-inspect -- `
//!   --ets2-dir "C:\...\Euro Truck Simulator 2" `
//!   --uid 0x001234567890ABCD
//!
//! # with optional graph.json cross-reference:
//! cargo run --release --bin truckpilot-node-inspect -- `
//!   --ets2-dir "C:\...\Euro Truck Simulator 2" `
//!   --graph graph.json `
//!   --uid 0x001234567890ABCD
//!
//! # inspect multiple UIDs (space-separated):
//! cargo run --release --bin truckpilot-node-inspect -- `
//!   --ets2-dir "C:\...\Euro Truck Simulator 2" `
//!   --uid 0x1111 0x2222 0x3333
//! ```

use std::collections::{HashMap, HashSet};
use std::path::PathBuf;

use truckpilot_map_parser::sector::{parse_sector, RawNode};
use truckpilot_map_parser::{Archive, HashFsArchive};

#[derive(Debug)]
struct Args {
    ets2_dir: PathBuf,
    graph: Option<PathBuf>,
    uids: Vec<u64>,
}

fn parse_uid(s: &str) -> u64 {
    let s = s.trim();
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u64::from_str_radix(hex, 16).unwrap_or_else(|_| panic!("invalid hex UID: {s}"))
    } else {
        s.parse::<u64>()
            .unwrap_or_else(|_| panic!("invalid UID: {s}"))
    }
}

fn parse_args() -> Args {
    let mut ets2_dir: Option<PathBuf> = None;
    let mut graph: Option<PathBuf> = None;
    let mut uids: Vec<u64> = Vec::new();

    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--ets2-dir" => {
                ets2_dir = Some(PathBuf::from(
                    argv.get(i + 1).expect("--ets2-dir needs value"),
                ));
                i += 2;
            }
            "--graph" => {
                graph = Some(PathBuf::from(argv.get(i + 1).expect("--graph needs value")));
                i += 2;
            }
            "--uid" => {
                i += 1;
                while i < argv.len() && !argv[i].starts_with('-') {
                    uids.push(parse_uid(&argv[i]));
                    i += 1;
                }
            }
            "-h" | "--help" => {
                eprintln!(
                    "usage: truckpilot-node-inspect --ets2-dir <DIR> --uid <UID...> [--graph <graph.json>]"
                );
                std::process::exit(0);
            }
            other => {
                eprintln!("unknown argument: {other}");
                std::process::exit(2);
            }
        }
    }
    if uids.is_empty() {
        eprintln!("ERROR: --uid <UID> is required");
        std::process::exit(2);
    }
    Args {
        ets2_dir: ets2_dir.unwrap_or_else(|| {
            eprintln!("ERROR: --ets2-dir is required");
            std::process::exit(2);
        }),
        graph,
        uids,
    }
}

// Minimal graph.json deserialization — only fields we need.
#[derive(serde::Deserialize)]
struct GraphFile {
    nodes: Vec<GraphNodeJson>,
    edges: Vec<GraphEdgeJson>,
    prefabs: Vec<GraphPrefabJson>,
}
#[derive(serde::Deserialize)]
struct GraphNodeJson {
    uid: u64,
}
#[derive(serde::Deserialize)]
struct GraphEdgeJson {
    uid: u64,
    from: u64,
    to: u64,
    distance_m: f64,
    direction: String,
}
#[derive(serde::Deserialize)]
struct GraphPrefabJson {
    connected_node_uids: Vec<u64>,
}

fn main() {
    let args = parse_args();
    let target_uids: HashSet<u64> = args.uids.iter().copied().collect();

    // --- Load sectors, build node map ---
    let base_map = args.ets2_dir.join("base_map.scs");
    eprintln!("opening {} ...", base_map.display());
    let mut archive =
        HashFsArchive::open(&base_map).unwrap_or_else(|e| panic!("open base_map.scs: {e}"));

    let sector_paths: Vec<String> = archive
        .probe_sector_paths()
        .into_iter()
        .filter(|p| p.ends_with(".base"))
        .collect();
    eprintln!(
        "scanning {} sectors for {} UID(s) ...",
        sector_paths.len(),
        target_uids.len()
    );

    let mut node_map: HashMap<u64, (RawNode, String)> = HashMap::new();

    for path in &sector_paths {
        let Ok(data) = archive.read_path(path) else {
            continue;
        };
        let Ok(sector) = parse_sector(&data) else {
            continue;
        };
        for node in sector.nodes {
            if target_uids.contains(&node.uid) {
                node_map.insert(node.uid, (node, path.clone()));
            }
        }
        if node_map.len() == target_uids.len() {
            break; // all found
        }
    }

    // --- Load graph.json if provided ---
    let graph_data: Option<GraphFile> = args.graph.as_ref().map(|p| {
        eprintln!("loading graph from {} ...", p.display());
        let bytes = std::fs::read(p).unwrap_or_else(|e| panic!("read {}: {e}", p.display()));
        serde_json::from_slice(&bytes).unwrap_or_else(|e| panic!("parse graph.json: {e}"))
    });

    let mut degrees: HashMap<u64, u32> = HashMap::new();
    let mut prefab_members: HashSet<u64> = HashSet::new();
    let mut edge_index: HashMap<u64, Vec<&GraphEdgeJson>> = HashMap::new();
    if let Some(ref g) = graph_data {
        for n in &g.nodes {
            degrees.insert(n.uid, 0);
        }
        for e in &g.edges {
            *degrees.entry(e.from).or_insert(0) += 1;
            *degrees.entry(e.to).or_insert(0) += 1;
            edge_index.entry(e.from).or_default().push(e);
            edge_index.entry(e.to).or_default().push(e);
        }
        for p in &g.prefabs {
            for &u in &p.connected_node_uids {
                prefab_members.insert(u);
            }
        }
    }

    // --- Print results ---
    println!();
    for uid in &args.uids {
        println!("══════════════════════════════════════════════════════");
        println!("NODE  uid = {uid:#018x}  ({uid})");
        println!("══════════════════════════════════════════════════════");

        if let Some((node, sector_path)) = node_map.get(uid) {
            println!("  sector           : {sector_path}");
            println!(
                "  position         : x={:.3}  y={:.3}  z={:.3}  (meters)",
                node.x, node.y, node.z
            );
            if node.forward_item_uid == 0 {
                println!("  forward_item_uid : 0  (none / sized-format)");
            } else {
                println!(
                    "  forward_item_uid : {:#018x}  ({})",
                    node.forward_item_uid, node.forward_item_uid
                );
            }
            if node.backward_item_uid == 0 {
                println!("  backward_item_uid: 0  (none / sized-format)");
            } else {
                println!(
                    "  backward_item_uid: {:#018x}  ({})",
                    node.backward_item_uid, node.backward_item_uid
                );
            }

            // Isolation diagnosis
            let has_forward = node.forward_item_uid != 0;
            let has_backward = node.backward_item_uid != 0;
            println!();
            match (has_forward, has_backward) {
                (false, false) => println!("  [DIAG] No item UIDs — likely decorative node or sized-format sector."),
                (true, false) => println!(
                    "  [DIAG] forward_item_uid set, backward=0 — endpoint node (road/prefab start).  \
                    If isolated in graph, the item {:#018x} may be from an unparsed type.",
                    node.forward_item_uid
                ),
                (false, true) => println!(
                    "  [DIAG] backward_item_uid set, forward=0 — endpoint node (road/prefab end).  \
                    If isolated in graph, the item {:#018x} may be from an unparsed type.",
                    node.backward_item_uid
                ),
                (true, true) => println!(
                    "  [DIAG] Both UIDs set — interior/junction node between items \
                    {:#018x} (fwd) and {:#018x} (bwd).",
                    node.forward_item_uid, node.backward_item_uid
                ),
            }
        } else {
            println!("  [NOT FOUND] UID not present in any parsed sector.");
            println!("  Possible reasons: sized-format sector, UID is an item not a node,");
            println!("  or the sector was not fully parsed (abort due to unknown item type).");
        }

        // Graph connectivity info
        if graph_data.is_some() {
            println!();
            let degree = degrees.get(uid).copied();
            let in_prefab = prefab_members.contains(uid);
            match degree {
                None => println!("  [GRAPH] Not in graph.json nodes list."),
                Some(d) => {
                    let prefab_tag = if in_prefab { " [prefab member]" } else { "" };
                    println!("  [GRAPH] degree={d}{prefab_tag}");
                    if d == 0 && !in_prefab {
                        println!("  [GRAPH] → ISOLATED (degree=0, not in any prefab clique)");
                    }
                    if let Some(edges) = edge_index.get(uid) {
                        println!("  [GRAPH] edges ({}):", edges.len());
                        for e in edges.iter().take(10) {
                            let other = if e.from == *uid { e.to } else { e.from };
                            println!(
                                "    edge uid={:#010x}  {} → {:#010x}  dist={:.1}m  dir={}",
                                e.uid, uid, other, e.distance_m, e.direction
                            );
                        }
                        if edges.len() > 10 {
                            println!("    ... and {} more", edges.len() - 10);
                        }
                    }
                }
            }
        }
        println!();
    }
}
