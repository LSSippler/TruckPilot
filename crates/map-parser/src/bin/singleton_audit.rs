//! `truckpilot-singleton-audit` — Phase 5.24 Singleton-Item-Source-Audit.
//!
//! Loads the production graph.json, finds singleton nodes (degree 0 +
//! not in any prefab clique), then brute-force scans every `.base` sector
//! body for 8-byte aligned u64 LE patterns matching singleton UIDs.
//! Reports hit-count distribution + sample.
//!
//! Each singleton UID is DEFINED at least once (in its sector's trailing
//! node section). A hit-count of:
//! - 0: defined nowhere (anomalous)
//! - 1: defined, NOT referenced -> truly orphan
//! - >=2: defined + referenced N-1 times by item-body bytes
//!
//! Output: `outputs/singleton_audit.md`.

use std::collections::{HashMap, HashSet};
use std::fmt::Write as _;
use std::path::PathBuf;

use truckpilot_map_parser::{Archive, HashFsArchive};

#[derive(Debug)]
struct Args {
    ets2_dir: PathBuf,
    graph: PathBuf,
    output: PathBuf,
    sample_count: usize,
}

fn parse_args() -> Args {
    let mut ets2_dir: Option<PathBuf> = None;
    let mut graph = PathBuf::from("graph.json");
    let mut output = PathBuf::from("outputs/singleton_audit.md");
    let mut sample_count = 100usize;
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
                graph = PathBuf::from(argv.get(i + 1).expect("--graph needs value"));
                i += 2;
            }
            "--output" => {
                output = PathBuf::from(argv.get(i + 1).expect("--output needs value"));
                i += 2;
            }
            "--samples" => {
                sample_count = argv.get(i + 1).and_then(|s| s.parse().ok()).unwrap_or(100);
                i += 2;
            }
            _ => i += 1,
        }
    }
    Args {
        ets2_dir: ets2_dir.expect("--ets2-dir required"),
        graph,
        output,
        sample_count,
    }
}

#[derive(serde::Deserialize)]
struct GraphFile {
    nodes: Vec<NodeJson>,
    edges: Vec<EdgeJson>,
    prefabs: Vec<PrefabJson>,
}
#[derive(serde::Deserialize)]
struct NodeJson {
    uid: u64,
}
#[derive(serde::Deserialize)]
struct EdgeJson {
    from: u64,
    to: u64,
}
#[derive(serde::Deserialize)]
struct PrefabJson {
    connected_node_uids: Vec<u64>,
}

fn main() {
    let args = parse_args();
    eprintln!("loading graph from {} ...", args.graph.display());
    let graph_bytes = std::fs::read(&args.graph).expect("read graph.json");
    let graph: GraphFile = serde_json::from_slice(&graph_bytes).expect("parse graph.json");
    eprintln!(
        "  {} nodes, {} edges, {} prefabs",
        graph.nodes.len(),
        graph.edges.len(),
        graph.prefabs.len()
    );

    let mut degree: HashMap<u64, u32> = HashMap::with_capacity(graph.nodes.len());
    for n in &graph.nodes {
        degree.insert(n.uid, 0);
    }
    for e in &graph.edges {
        *degree.entry(e.from).or_insert(0) += 1;
        *degree.entry(e.to).or_insert(0) += 1;
    }
    let mut prefab_attached: HashSet<u64> = HashSet::new();
    for p in &graph.prefabs {
        for u in &p.connected_node_uids {
            prefab_attached.insert(*u);
        }
    }
    let singletons: HashSet<u64> = degree
        .iter()
        .filter(|(_, d)| **d == 0)
        .filter(|(uid, _)| !prefab_attached.contains(uid))
        .map(|(uid, _)| *uid)
        .collect();
    eprintln!(
        "singleton nodes (degree=0, not in prefab clique): {}",
        singletons.len()
    );

    let base_map = args.ets2_dir.join("base_map.scs");
    let mut archive = HashFsArchive::open(&base_map).expect("open base_map.scs");
    let mut sector_paths: Vec<String> = archive
        .probe_sector_paths()
        .into_iter()
        .filter(|p| p.ends_with(".base"))
        .collect();
    sector_paths.sort();
    eprintln!("scanning {} `.base` sectors ...", sector_paths.len());

    let mut hit_counts: HashMap<u64, u32> = HashMap::with_capacity(singletons.len());
    for uid in &singletons {
        hit_counts.insert(*uid, 0);
    }

    let mut total_bytes_scanned = 0usize;
    let mut total_aligned_positions = 0usize;
    for (i, path) in sector_paths.iter().enumerate() {
        let Ok(data) = archive.read_path(path) else {
            continue;
        };
        total_bytes_scanned += data.len();
        // Byte-level scan: ETS2 item bodies pack u64s at non-8-aligned
        // offsets (kdop_item is 53B, breaking alignment). 8-aligned scan
        // misses most refs.
        let mut pos = 0usize;
        let len = data.len();
        while pos + 8 <= len {
            let v = u64::from_le_bytes(data[pos..pos + 8].try_into().unwrap());
            if let Some(c) = hit_counts.get_mut(&v) {
                *c = c.saturating_add(1);
            }
            pos += 1;
        }
        total_aligned_positions += data.len().saturating_sub(7);
        if i % 50 == 0 {
            eprintln!("  scanned {}/{} sectors", i + 1, sector_paths.len());
        }
    }

    let mut by_count: HashMap<u32, usize> = HashMap::new();
    for c in hit_counts.values() {
        *by_count.entry(*c).or_insert(0) += 1;
    }
    let mut by_count_vec: Vec<(u32, usize)> = by_count.into_iter().collect();
    by_count_vec.sort_by_key(|a| a.0);

    let total_singletons = singletons.len();
    let zero_hits = hit_counts.values().filter(|c| **c == 0).count();
    let one_hit = hit_counts.values().filter(|c| **c == 1).count();
    let multi_hits = hit_counts.values().filter(|c| **c >= 2).count();

    let mut sample: Vec<(u64, u32)> = hit_counts
        .iter()
        .take(args.sample_count)
        .map(|(u, c)| (*u, *c))
        .collect();
    sample.sort_by_key(|a| std::cmp::Reverse(a.1));

    let mut out = String::new();
    let _ = writeln!(out, "# Phase 5.24 — Singleton-Audit (Task 2)");
    let _ = writeln!(out);
    let _ = writeln!(out, "## Methodology");
    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "Byte-level brute-force scan of every u64 LE position across all"
    );
    let _ = writeln!(
        out,
        "`.base` sectors (kdop_item is 53 bytes, so item-body u64s are NOT"
    );
    let _ = writeln!(
        out,
        "8-aligned; 8-aligned scan misses most refs). Each singleton UID is"
    );
    let _ = writeln!(
        out,
        "DEFINED at least once (trailing node section of its home sector)."
    );
    let _ = writeln!(
        out,
        "Hit-count 1 = defined-only; >=2 = also referenced by item-body bytes."
    );
    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "False-positive rate at byte alignment is still bounded: scanned"
    );
    let _ = writeln!(
        out,
        "positions * singletons / 2^64 < 0.01 expected FP per singleton."
    );
    let _ = writeln!(out);
    let _ = writeln!(out, "## Aggregates");
    let _ = writeln!(out);
    let _ = writeln!(out, "| Metric | Value |");
    let _ = writeln!(out, "| --- | ---: |");
    let _ = writeln!(out, "| Total singletons | {} |", total_singletons);
    let _ = writeln!(out, "| Sectors scanned | {} |", sector_paths.len());
    let _ = writeln!(out, "| Total bytes scanned | {} |", total_bytes_scanned);
    let _ = writeln!(
        out,
        "| Byte-aligned positions | {} |",
        total_aligned_positions
    );
    let _ = writeln!(out);
    let _ = writeln!(out, "## Hit-count distribution");
    let _ = writeln!(out);
    let _ = writeln!(out, "| Hit count | Singletons | % |");
    let _ = writeln!(out, "| ---: | ---: | ---: |");
    for (k, v) in &by_count_vec {
        let pct = 100.0 * *v as f64 / total_singletons as f64;
        let label = match *k {
            0 => "0 (not found — anomalous)".to_string(),
            1 => "1 (defined, NOT referenced — orphan)".to_string(),
            n => format!("{n} (defined + referenced {}x)", n - 1),
        };
        let _ = writeln!(out, "| {label} | {v} | {pct:.2}% |");
    }
    let _ = writeln!(out);
    let _ = writeln!(out, "## Headline");
    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "- {} ({:.1}%) singletons hit-count 1 — truly orphan",
        one_hit,
        100.0 * one_hit as f64 / total_singletons as f64
    );
    let _ = writeln!(
        out,
        "- {} ({:.1}%) singletons hit-count >=2 — referenced by item-body",
        multi_hits,
        100.0 * multi_hits as f64 / total_singletons as f64
    );
    let _ = writeln!(
        out,
        "- {} ({:.1}%) singletons hit-count 0 — anomalous",
        zero_hits,
        100.0 * zero_hits as f64 / total_singletons as f64
    );
    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "## Sample ({} singletons by hit-count desc)",
        sample.len()
    );
    let _ = writeln!(out);
    let _ = writeln!(out, "| UID (hex) | Hit count | Refs in items |");
    let _ = writeln!(out, "| --- | ---: | ---: |");
    for (uid, c) in &sample {
        let refs = c.saturating_sub(1);
        let _ = writeln!(out, "| 0x{uid:016x} | {c} | {refs} |");
    }
    let _ = writeln!(out);
    let _ = writeln!(out, "## Interpretation");
    let _ = writeln!(out);
    let multi_pct = 100.0 * multi_hits as f64 / total_singletons as f64;
    if multi_pct > 50.0 {
        let _ = writeln!(out, "**>50% der Singletons sind referenziert** — Inventory-Hypothese bestaetigt: ungenutzte Node-Refs in Sign/Model/FarModel/MapOverlay/BusStop/u.a. erklaeren die Singletons.");
    } else if multi_pct > 10.0 {
        let _ = writeln!(out, "**{multi_pct:.1}% referenziert** — Inventory-Hypothese stimmt teilweise; ein erheblicher Teil sind echte Tail-Recovery-Verwaiste.");
    } else {
        let _ = writeln!(out, "**<10% referenziert** ({multi_pct:.1}%) — Inventory-Hypothese FALSCH. Die meisten Singletons sind echte Verwaiste die nirgendwo im base_map.scs als Item-Body-Bytes auftauchen. Cross-Reference muss in DLCs oder /def/ liegen.");
    }

    if let Some(parent) = args.output.parent() {
        std::fs::create_dir_all(parent).ok();
    }
    std::fs::write(&args.output, &out).expect("write output");
    eprintln!("-> {} ({} bytes)", args.output.display(), out.len());
}
