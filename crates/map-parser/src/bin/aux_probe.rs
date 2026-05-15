//! `truckpilot-aux-probe` — Phase 5.14 (C1) hypothesis test.
//!
//! Phase 5.13 ruled out the trailing `vis_uids` block as cross-sector
//! references (0.0 % cross of 954 uids — they are not node UIDs at all).
//! Cross-sector edges are still at ~1.4 % in the graph, so the data must
//! live elsewhere. Candidate: the `.aux` companion file SCS writes next to
//! every `.base`. We don't currently parse `.aux` at all (`mod_loader.rs`
//! filters them out explicitly).
//!
//! This binary tests whether `.aux` files contain node UIDs that match
//! the v907 fingerprint `high u16 == 0x0029`, and whether those UIDs
//! reference nodes from *other* sectors (the "cross-sector" tell).
//!
//! Workflow:
//!
//! 1. Probe every `.base` and `.aux` path in `base_map.scs`.
//! 2. Parse every `.base` to build the global `node_to_sector` map +
//!    a per-sector `local_nodes` set.
//! 3. Pick the first N (=5) `.aux` files whose `.base` companion parses.
//! 4. For each chosen `.aux`:
//!    a. Dump the first 64 and last 64 bytes as hex (head/tail layout).
//!    b. Walk every byte offset, read u64-LE, filter by fingerprint,
//!    deduplicate.
//!    c. Classify each unique fingerprint-matching uid against
//!    `local_nodes` (same-sector) and `node_to_sector`
//!    (cross-sector if not local but globally known) — the rest
//!    are "unresolved".
//! 5. Verdict line: `.aux` carries cross-sector node refs if
//!    cross-share is high.
//!
//! Statistical sanity: a 16-bit fingerprint gives ~1/65536 false
//! positives per offset. A typical 50 KiB `.aux` has ~50 K offsets, so
//! ~0.8 chance hits per file. Anything substantially above that is
//! signal.
//!
//! Usage:
//!
//! ```powershell
//! cargo run --release --bin truckpilot-aux-probe -- `
//!   --ets2-dir "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2"
//! ```

use std::collections::{HashMap, HashSet};
use std::fmt::Write as _;
use std::path::PathBuf;

use truckpilot_map_parser::sector::parse_sector;
use truckpilot_map_parser::{Archive, HashFsArchive};

/// CLI arguments.
#[derive(Debug)]
struct Args {
    ets2_dir: PathBuf,
    output: PathBuf,
    sample_count: usize,
}

/// Per-`.aux` classification report.
struct AuxReport {
    aux_path: String,
    base_path: String,
    aux_size: usize,
    base_node_count: usize,
    base_item_count: usize,
    head_hex: String,
    tail_hex: String,
    raw_offsets_scanned: usize,
    unique_fingerprint_uids: usize,
    same_node: Vec<u64>,
    same_item: Vec<u64>,
    cross_node: Vec<(u64, u32)>,
    cross_item: Vec<(u64, u32)>,
    unresolved: Vec<u64>,
}

fn parse_args() -> Args {
    let mut ets2_dir: Option<PathBuf> = None;
    let mut output = PathBuf::from("outputs/aux_inspection.txt");
    let mut sample_count = 5usize;

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
            "--output" => {
                output = PathBuf::from(argv.get(i + 1).expect("--output needs value"));
                i += 2;
            }
            "--samples" => {
                sample_count = argv
                    .get(i + 1)
                    .and_then(|s| s.parse().ok())
                    .expect("--samples needs positive integer");
                i += 2;
            }
            "-h" | "--help" => {
                eprintln!(
                    "usage: truckpilot-aux-probe --ets2-dir <PATH> [--output <FILE>] [--samples N]"
                );
                std::process::exit(0);
            }
            other => {
                eprintln!("unknown argument: {other}");
                std::process::exit(2);
            }
        }
    }

    Args {
        ets2_dir: ets2_dir.unwrap_or_else(|| {
            eprintln!("ERROR: --ets2-dir is required");
            std::process::exit(2);
        }),
        output,
        sample_count,
    }
}

/// Strip `.base` / `.aux` extension, leaving `…/sec+0001+0001` so
/// companion files can be paired by stem.
fn stem_of(path: &str) -> &str {
    if let Some(p) = path.strip_suffix(".base") {
        return p;
    }
    if let Some(p) = path.strip_suffix(".aux") {
        return p;
    }
    path
}

/// `high u16 == 0x0029` matches the v907 node-UID prefix established in
/// the Phase 5.11–5.13 diagnostics.
fn looks_like_node_uid(uid: u64) -> bool {
    (uid >> 48) as u16 == 0x0029
}

/// Hex-dump the first `n` (or all if shorter) bytes of `data`,
/// formatted `xx xx xx xx  xx xx xx xx  | …` per 16-byte row.
fn hex_block(data: &[u8], n: usize) -> String {
    let take = data.len().min(n);
    let mut out = String::with_capacity(take * 4);
    for (i, chunk) in data[..take].chunks(16).enumerate() {
        let _ = write!(out, "      {:04x}: ", i * 16);
        for (j, b) in chunk.iter().enumerate() {
            let _ = write!(out, "{b:02x}");
            if j == 7 {
                let _ = write!(out, "  ");
            } else if j < chunk.len() - 1 {
                let _ = write!(out, " ");
            }
        }
        let pad = 16 - chunk.len();
        for _ in 0..pad {
            let _ = write!(out, "   ");
        }
        let _ = write!(out, "  | ");
        for &b in chunk {
            let c = if (0x20..=0x7e).contains(&b) {
                b as char
            } else {
                '.'
            };
            out.push(c);
        }
        out.push('\n');
    }
    out
}

/// Hex-dump the last `n` bytes (mirror of `hex_block`).
fn hex_tail(data: &[u8], n: usize) -> String {
    if data.len() <= n {
        return hex_block(data, data.len());
    }
    let start = data.len() - n;
    let mut out = String::with_capacity(n * 4);
    for (i, chunk) in data[start..].chunks(16).enumerate() {
        let _ = write!(out, "      {:04x}: ", start + i * 16);
        for (j, b) in chunk.iter().enumerate() {
            let _ = write!(out, "{b:02x}");
            if j == 7 {
                let _ = write!(out, "  ");
            } else if j < chunk.len() - 1 {
                let _ = write!(out, " ");
            }
        }
        let pad = 16 - chunk.len();
        for _ in 0..pad {
            let _ = write!(out, "   ");
        }
        let _ = write!(out, "  | ");
        for &b in chunk {
            let c = if (0x20..=0x7e).contains(&b) {
                b as char
            } else {
                '.'
            };
            out.push(c);
        }
        out.push('\n');
    }
    out
}

fn main() {
    let args = parse_args();
    let base_map = args.ets2_dir.join("base_map.scs");
    if !base_map.exists() {
        eprintln!(
            "ERROR: {} not found — wrong --ets2-dir?",
            base_map.display()
        );
        std::process::exit(1);
    }

    eprintln!("opening {} …", base_map.display());
    let mut archive = HashFsArchive::open(&base_map).expect("open base_map.scs");

    let all_paths = archive.probe_sector_paths();
    let base_paths: Vec<String> = all_paths
        .iter()
        .filter(|p| p.ends_with(".base"))
        .cloned()
        .collect();
    let aux_paths: HashSet<String> = all_paths
        .iter()
        .filter(|p| p.ends_with(".aux"))
        .cloned()
        .collect();
    eprintln!(
        "probed {} `.base`, {} `.aux` paths",
        base_paths.len(),
        aux_paths.len()
    );

    // ----- Pass 1: parse every .base -----------------------------------
    //
    // Build two global lookups:
    //   * `node_to_sector`: only RawNode.uid — for "this is a node" tagging.
    //   * `world_uid_to_sector`: nodes ∪ road UIDs ∪ road endpoints ∪
    //     prefab UIDs ∪ prefab node refs ∪ sign UIDs — for "this UID is
    //     known to live somewhere in .base" tagging.
    //
    // Per-sector local mirrors so we can split same-sector vs cross-sector.

    let mut node_to_sector: HashMap<u64, u32> = HashMap::with_capacity(200_000);
    let mut world_uid_to_sector: HashMap<u64, u32> = HashMap::with_capacity(2_000_000);
    let mut local_nodes: Vec<HashSet<u64>> = Vec::with_capacity(base_paths.len());
    let mut local_world: Vec<HashSet<u64>> = Vec::with_capacity(base_paths.len());
    let mut parsed_ok: Vec<bool> = Vec::with_capacity(base_paths.len());

    for (sid, path) in base_paths.iter().enumerate() {
        let Ok(data) = archive.read_path(path) else {
            local_nodes.push(HashSet::new());
            local_world.push(HashSet::new());
            parsed_ok.push(false);
            continue;
        };
        match parse_sector(&data) {
            Ok(parsed) => {
                let mut node_set = HashSet::with_capacity(parsed.nodes.len());
                let mut world_set = HashSet::with_capacity(parsed.nodes.len() * 4);
                let sid32 = sid as u32;
                for n in &parsed.nodes {
                    node_to_sector.insert(n.uid, sid32);
                    world_uid_to_sector.insert(n.uid, sid32);
                    node_set.insert(n.uid);
                    world_set.insert(n.uid);
                }
                for r in &parsed.roads {
                    world_uid_to_sector.insert(r.uid, sid32);
                    world_uid_to_sector.insert(r.node_a, sid32);
                    world_uid_to_sector.insert(r.node_b, sid32);
                    world_set.insert(r.uid);
                    world_set.insert(r.node_a);
                    world_set.insert(r.node_b);
                }
                for p in &parsed.prefabs {
                    world_uid_to_sector.insert(p.uid, sid32);
                    world_set.insert(p.uid);
                    for &nu in &p.nodes {
                        world_uid_to_sector.insert(nu, sid32);
                        world_set.insert(nu);
                    }
                }
                for s in &parsed.signs {
                    world_uid_to_sector.insert(s.uid, sid32);
                    world_set.insert(s.uid);
                }
                local_nodes.push(node_set);
                local_world.push(world_set);
                parsed_ok.push(true);
            }
            Err(_) => {
                local_nodes.push(HashSet::new());
                local_world.push(HashSet::new());
                parsed_ok.push(false);
            }
        }
    }
    eprintln!(
        "parsed {}/{} sectors, node pool = {}, full world-uid pool = {}",
        parsed_ok.iter().filter(|b| **b).count(),
        base_paths.len(),
        node_to_sector.len(),
        world_uid_to_sector.len()
    );

    // ----- Pass 2: pick N samples whose .base parsed -------------------

    let mut samples: Vec<(usize, String, String)> = Vec::new();
    for (sid, base) in base_paths.iter().enumerate() {
        if samples.len() >= args.sample_count {
            break;
        }
        if !parsed_ok[sid] {
            continue;
        }
        let aux = format!("{}.aux", stem_of(base));
        if aux_paths.contains(&aux) {
            samples.push((sid, base.clone(), aux));
        }
    }
    eprintln!("selected {} sample .aux files", samples.len());
    if samples.is_empty() {
        eprintln!("ERROR: no .aux companions found for any parsed .base sector");
        std::process::exit(1);
    }

    // ----- Pass 3: per-sample classify ---------------------------------

    let mut reports: Vec<AuxReport> = Vec::with_capacity(samples.len());
    for (sid, base, aux) in &samples {
        eprintln!("probing {} …", aux);
        let Ok(data) = archive.read_path(aux) else {
            eprintln!("  read failed, skipping");
            continue;
        };
        let head_hex = hex_block(&data, 64);
        let tail_hex = hex_tail(&data, 64);

        let mut seen: HashSet<u64> = HashSet::new();
        let mut offsets_scanned = 0usize;
        if data.len() >= 8 {
            for off in 0..=(data.len() - 8) {
                offsets_scanned += 1;
                let uid = u64::from_le_bytes(data[off..off + 8].try_into().unwrap());
                if looks_like_node_uid(uid) {
                    seen.insert(uid);
                }
            }
        }

        let local_node = &local_nodes[*sid];
        let local_w = &local_world[*sid];
        let mut same_node = Vec::new();
        let mut same_item = Vec::new();
        let mut cross_node = Vec::new();
        let mut cross_item = Vec::new();
        let mut unresolved = Vec::new();
        for &uid in &seen {
            if local_node.contains(&uid) {
                same_node.push(uid);
            } else if local_w.contains(&uid) {
                same_item.push(uid);
            } else if let Some(other_sid) = node_to_sector.get(&uid) {
                cross_node.push((uid, *other_sid));
            } else if let Some(other_sid) = world_uid_to_sector.get(&uid) {
                cross_item.push((uid, *other_sid));
            } else {
                unresolved.push(uid);
            }
        }
        same_node.sort_unstable();
        same_item.sort_unstable();
        cross_node.sort_unstable();
        cross_item.sort_unstable();
        unresolved.sort_unstable();

        reports.push(AuxReport {
            aux_path: aux.clone(),
            base_path: base.clone(),
            aux_size: data.len(),
            base_node_count: local_node.len(),
            base_item_count: local_w.len() - local_node.len(),
            head_hex,
            tail_hex,
            raw_offsets_scanned: offsets_scanned,
            unique_fingerprint_uids: seen.len(),
            same_node,
            same_item,
            cross_node,
            cross_item,
            unresolved,
        });
    }

    // ----- Pass 4: render report ---------------------------------------

    let mut out = String::new();
    let _ = writeln!(out, "# Phase 5.14 (C1) — `.aux` cross-sector probe");
    let _ = writeln!(out);
    let _ = writeln!(out, "Archive:           {}", base_map.display());
    let _ = writeln!(out, "Total `.base`:     {}", base_paths.len());
    let _ = writeln!(
        out,
        "Parsed `.base`:    {}",
        parsed_ok.iter().filter(|b| **b).count()
    );
    let _ = writeln!(out, "Total `.aux`:      {}", aux_paths.len());
    let _ = writeln!(out, "Global node pool:  {}", node_to_sector.len());
    let _ = writeln!(
        out,
        "Global world pool: {} (nodes ∪ road/prefab/sign UIDs ∪ road/prefab node refs)",
        world_uid_to_sector.len()
    );
    let _ = writeln!(out, "Samples requested: {}", args.sample_count);
    let _ = writeln!(out, "Samples produced:  {}", reports.len());
    let _ = writeln!(
        out,
        "UID fingerprint:   high u16 == 0x0029 (v907 world-uid prefix)"
    );
    let _ = writeln!(out);

    let mut total_unique = 0usize;
    let mut total_same_node = 0usize;
    let mut total_same_item = 0usize;
    let mut total_cross_node = 0usize;
    let mut total_cross_item = 0usize;
    let mut total_unresolved = 0usize;

    for r in &reports {
        let _ = writeln!(
            out,
            "------------------------------------------------------------"
        );
        let _ = writeln!(out, "## {}", r.aux_path);
        let _ = writeln!(out, "    companion .base        : {}", r.base_path);
        let _ = writeln!(out, "    .aux size              : {} bytes", r.aux_size);
        let _ = writeln!(out, "    .base node count       : {}", r.base_node_count);
        let _ = writeln!(out, "    .base item count (other): {}", r.base_item_count);
        let _ = writeln!(
            out,
            "    byte offsets scanned   : {}",
            r.raw_offsets_scanned
        );
        let _ = writeln!(
            out,
            "    fingerprint matches    : {} unique u64 values",
            r.unique_fingerprint_uids
        );
        let _ = writeln!(out, "    classification:");
        let _ = writeln!(out, "      same-sector node     : {}", r.same_node.len());
        let _ = writeln!(out, "      same-sector item     : {}", r.same_item.len());
        let _ = writeln!(out, "      cross-sector node    : {}", r.cross_node.len());
        let _ = writeln!(out, "      cross-sector item    : {}", r.cross_item.len());
        let _ = writeln!(out, "      unresolved           : {}", r.unresolved.len());
        let _ = writeln!(out);
        let _ = writeln!(out, "    head (first 64 bytes):");
        out.push_str(&r.head_hex);
        let _ = writeln!(out);
        let _ = writeln!(out, "    tail (last 64 bytes):");
        out.push_str(&r.tail_hex);
        let _ = writeln!(out);
        if !r.cross_node.is_empty() {
            let _ = writeln!(out, "    cross-sector node samples (uid -> other sector):");
            for (uid, sid) in r.cross_node.iter().take(10) {
                let other = base_paths
                    .get(*sid as usize)
                    .map(String::as_str)
                    .unwrap_or("?");
                let _ = writeln!(out, "      {uid:#018x} -> sid {sid} ({other})");
            }
            let _ = writeln!(out);
        }
        if !r.cross_item.is_empty() {
            let _ = writeln!(out, "    cross-sector item samples (uid -> other sector):");
            for (uid, sid) in r.cross_item.iter().take(10) {
                let other = base_paths
                    .get(*sid as usize)
                    .map(String::as_str)
                    .unwrap_or("?");
                let _ = writeln!(out, "      {uid:#018x} -> sid {sid} ({other})");
            }
            let _ = writeln!(out);
        }
        if !r.same_item.is_empty() {
            let _ = writeln!(out, "    same-sector item samples (first 5):");
            for uid in r.same_item.iter().take(5) {
                let _ = writeln!(out, "      {uid:#018x}");
            }
            let _ = writeln!(out);
        }
        if !r.unresolved.is_empty() {
            let _ = writeln!(out, "    unresolved samples (first 5):");
            for uid in r.unresolved.iter().take(5) {
                let _ = writeln!(out, "      {uid:#018x}");
            }
            let _ = writeln!(out);
        }

        total_unique += r.unique_fingerprint_uids;
        total_same_node += r.same_node.len();
        total_same_item += r.same_item.len();
        total_cross_node += r.cross_node.len();
        total_cross_item += r.cross_item.len();
        total_unresolved += r.unresolved.len();
    }

    let _ = writeln!(
        out,
        "============================================================"
    );
    let _ = writeln!(out, "## Aggregate ({} samples)", reports.len());
    let _ = writeln!(out, "    unique fingerprint uids  : {total_unique}");
    let _ = writeln!(out, "    same-sector node         : {total_same_node}");
    let _ = writeln!(out, "    same-sector item         : {total_same_item}");
    let _ = writeln!(out, "    cross-sector node        : {total_cross_node}");
    let _ = writeln!(out, "    cross-sector item        : {total_cross_item}");
    let _ = writeln!(out, "    unresolved               : {total_unresolved}");
    let _ = writeln!(out);

    let total_cross = total_cross_node + total_cross_item;
    let total_resolved = total_same_node + total_same_item + total_cross;
    let resolved_share = if total_unique > 0 {
        100.0 * total_resolved as f64 / total_unique as f64
    } else {
        0.0
    };
    let unresolved_share = if total_unique > 0 {
        100.0 * total_unresolved as f64 / total_unique as f64
    } else {
        0.0
    };
    let cross_node_share = if total_unique > 0 {
        100.0 * total_cross_node as f64 / total_unique as f64
    } else {
        0.0
    };

    let _ = writeln!(out, "## Verdict");
    if total_cross_node >= 5 && cross_node_share >= 10.0 {
        let _ = writeln!(
            out,
            "→ STRONGLY SUPPORTED — {total_cross_node} cross-sector NODE hits ({cross_node_share:.1}%)."
        );
        let _ = writeln!(
            out,
            "→ `.aux` carries cross-sector node references. Build a real `.aux` parser → routing fix."
        );
    } else if total_cross_item >= 50 && resolved_share >= 20.0 {
        let _ = writeln!(
            out,
            "→ ITEM-LAYER ({total_cross_item} cross-sector item hits, {resolved_share:.1}% resolved overall)."
        );
        let _ = writeln!(
            out,
            "→ `.aux` references items (signs/roads/prefabs) in OTHER sectors — visibility/LOD layer, not direct routing topology."
        );
        let _ = writeln!(
            out,
            "→ Routing-relevant only via prefab indirection. Recommend (C3) prefab `.ppd` next."
        );
    } else if total_resolved >= 50 && resolved_share >= 20.0 {
        let _ = writeln!(
            out,
            "→ LOCAL-ONLY ({total_same_node} same-node + {total_same_item} same-item, {resolved_share:.1}% resolved)."
        );
        let _ = writeln!(
            out,
            "→ `.aux` references its OWN sector's items — visibility/LOD or item-extension data. Not a cross-sector layer."
        );
        let _ = writeln!(out, "→ Move on: (C2) road handler or (C3) prefab `.ppd`.");
    } else if total_unresolved > 50 && unresolved_share >= 90.0 {
        let _ = writeln!(
            out,
            "→ EXTERNAL UID FAMILY — {total_unique} fingerprint matches but {unresolved_share:.1}% unresolved against full world pool."
        );
        let _ = writeln!(
            out,
            "→ `.aux` UIDs reference a fourth source (`.data`/`.desc` companions, def files, model tokens). Routing-relevant only via prefab `.ppd` (recommend C3)."
        );
    } else {
        let _ = writeln!(
            out,
            "→ REJECTED — {total_unique} fingerprint matches, near noise floor."
        );
        let _ = writeln!(out, "→ Move on to (C2) road handler or (C3) prefab `.ppd`.");
    }

    if let Some(parent) = args.output.parent() {
        std::fs::create_dir_all(parent).expect("create output dir");
    }
    std::fs::write(&args.output, &out).expect("write output");
    eprintln!("→ {} ({} bytes)", args.output.display(), out.len());
}
