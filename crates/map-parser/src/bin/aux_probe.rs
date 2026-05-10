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
    head_hex: String,
    tail_hex: String,
    raw_offsets_scanned: usize,
    unique_fingerprint_uids: usize,
    same_sector: Vec<u64>,
    cross_sector: Vec<(u64, u32)>,
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

    let mut node_to_sector: HashMap<u64, u32> = HashMap::with_capacity(1_000_000);
    let mut local_nodes: Vec<HashSet<u64>> = Vec::with_capacity(base_paths.len());
    let mut parsed_ok: Vec<bool> = Vec::with_capacity(base_paths.len());

    for (sid, path) in base_paths.iter().enumerate() {
        let Ok(data) = archive.read_path(path) else {
            local_nodes.push(HashSet::new());
            parsed_ok.push(false);
            continue;
        };
        match parse_sector(&data) {
            Ok(parsed) => {
                let mut set = HashSet::with_capacity(parsed.nodes.len());
                for n in &parsed.nodes {
                    node_to_sector.insert(n.uid, sid as u32);
                    set.insert(n.uid);
                }
                local_nodes.push(set);
                parsed_ok.push(true);
            }
            Err(_) => {
                local_nodes.push(HashSet::new());
                parsed_ok.push(false);
            }
        }
    }
    eprintln!(
        "parsed {}/{} sectors, global node-uid pool = {}",
        parsed_ok.iter().filter(|b| **b).count(),
        base_paths.len(),
        node_to_sector.len()
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

        let local = &local_nodes[*sid];
        let mut same_sector = Vec::new();
        let mut cross_sector = Vec::new();
        let mut unresolved = Vec::new();
        for &uid in &seen {
            if local.contains(&uid) {
                same_sector.push(uid);
            } else if let Some(other_sid) = node_to_sector.get(&uid) {
                cross_sector.push((uid, *other_sid));
            } else {
                unresolved.push(uid);
            }
        }
        same_sector.sort_unstable();
        cross_sector.sort_unstable();
        unresolved.sort_unstable();

        reports.push(AuxReport {
            aux_path: aux.clone(),
            base_path: base.clone(),
            aux_size: data.len(),
            base_node_count: local.len(),
            head_hex,
            tail_hex,
            raw_offsets_scanned: offsets_scanned,
            unique_fingerprint_uids: seen.len(),
            same_sector,
            cross_sector,
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
    let _ = writeln!(out, "Samples requested: {}", args.sample_count);
    let _ = writeln!(out, "Samples produced:  {}", reports.len());
    let _ = writeln!(out, "UID fingerprint:   high u16 == 0x0029 (v907 node-uid)");
    let _ = writeln!(out);

    let mut total_unique = 0usize;
    let mut total_same = 0usize;
    let mut total_cross = 0usize;
    let mut total_unresolved = 0usize;

    for r in &reports {
        let _ = writeln!(out, "------------------------------------------------------------");
        let _ = writeln!(out, "## {}", r.aux_path);
        let _ = writeln!(out, "    companion .base       : {}", r.base_path);
        let _ = writeln!(out, "    .aux size             : {} bytes", r.aux_size);
        let _ = writeln!(out, "    .base node count      : {}", r.base_node_count);
        let _ = writeln!(out, "    byte offsets scanned  : {}", r.raw_offsets_scanned);
        let _ = writeln!(
            out,
            "    fingerprint matches   : {} unique u64 values",
            r.unique_fingerprint_uids
        );
        let _ = writeln!(out, "    classification:");
        let _ = writeln!(out, "      same-sector  : {}", r.same_sector.len());
        let _ = writeln!(out, "      cross-sector : {}", r.cross_sector.len());
        let _ = writeln!(out, "      unresolved   : {}", r.unresolved.len());
        let _ = writeln!(out);
        let _ = writeln!(out, "    head (first 64 bytes):");
        out.push_str(&r.head_hex);
        let _ = writeln!(out);
        let _ = writeln!(out, "    tail (last 64 bytes):");
        out.push_str(&r.tail_hex);
        let _ = writeln!(out);
        if !r.cross_sector.is_empty() {
            let _ = writeln!(out, "    cross-sector samples (uid -> other sector index):");
            for (uid, sid) in r.cross_sector.iter().take(10) {
                let other = base_paths
                    .get(*sid as usize)
                    .map(String::as_str)
                    .unwrap_or("?");
                let _ = writeln!(out, "      {uid:#018x} -> sid {sid} ({other})");
            }
            let _ = writeln!(out);
        }
        if !r.same_sector.is_empty() {
            let _ = writeln!(out, "    same-sector samples (first 5):");
            for uid in r.same_sector.iter().take(5) {
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
        total_same += r.same_sector.len();
        total_cross += r.cross_sector.len();
        total_unresolved += r.unresolved.len();
    }

    let _ = writeln!(out, "============================================================");
    let _ = writeln!(out, "## Aggregate ({} samples)", reports.len());
    let _ = writeln!(out, "    unique fingerprint uids : {total_unique}");
    let _ = writeln!(out, "    same-sector             : {total_same}");
    let _ = writeln!(out, "    cross-sector            : {total_cross}");
    let _ = writeln!(out, "    unresolved              : {total_unresolved}");
    let _ = writeln!(out);

    let cross_share = if total_unique > 0 {
        100.0 * total_cross as f64 / total_unique as f64
    } else {
        0.0
    };
    let unresolved_share = if total_unique > 0 {
        100.0 * total_unresolved as f64 / total_unique as f64
    } else {
        0.0
    };

    let _ = writeln!(out, "## Verdict");
    if total_cross >= 5 && cross_share >= 20.0 {
        let _ = writeln!(
            out,
            "→ STRONGLY SUPPORTED ({total_cross} cross-sector hits, {cross_share:.1}% of fingerprint uids)."
        );
        let _ = writeln!(
            out,
            "→ `.aux` likely carries cross-sector node references. Build a real `.aux` parser next."
        );
    } else if total_cross > 0 {
        let _ = writeln!(
            out,
            "→ PARTIAL ({total_cross} cross hits, {cross_share:.1}% — above the ~0.8/file noise floor but below the 20% threshold)."
        );
        let _ = writeln!(
            out,
            "→ Worth a closer look but might be incidental matches. Consider a wider sample (--samples 30) before committing to a parser."
        );
    } else if total_unresolved > 50 {
        let _ = writeln!(
            out,
            "→ INDETERMINATE — many fingerprint matches ({total_unique}, {unresolved_share:.1}% unresolved) but none resolve to known nodes."
        );
        let _ = writeln!(
            out,
            "→ Could be (a) random false positives, or (b) UIDs of cross-sector nodes whose .base failed to parse. Compare against the {} unparsed-sector list.",
            base_paths.len() - parsed_ok.iter().filter(|b| **b).count()
        );
    } else {
        let _ = writeln!(
            out,
            "→ REJECTED ({total_cross} cross, {total_unique} fingerprint matches total — close to the noise floor)."
        );
        let _ = writeln!(
            out,
            "→ `.aux` does not appear to carry node UIDs. Move on to (C2) road handler or (C3) prefab `.ppd`."
        );
    }

    if let Some(parent) = args.output.parent() {
        std::fs::create_dir_all(parent).expect("create output dir");
    }
    std::fs::write(&args.output, &out).expect("write output");
    eprintln!("→ {} ({} bytes)", args.output.display(), out.len());
}
