//! `truckpilot-road-dump` — empirical diagnostic tool for Phase 5.7
//!
//! Reads `base_map.scs` from a real ETS2 install, finds sectors that contain
//! exactly one Road item (no Prefab, no other items), parses the 265-byte
//! fixed header, and writes every byte that follows in an annotated hex dump
//! so we can manually reverse-engineer the v907 DataPayload format.
//!
//! Usage:
//!
//! ```powershell
//! cargo run --release --bin truckpilot-road-dump -- `
//!   --ets2-dir "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2" `
//!   --output road_dump.txt `
//!   --max-roads 10
//! ```
//!
//! Flags:
//! * `--ets2-dir <PATH>`     ETS2 install root containing `base_map.scs`.
//! * `--output <PATH>`       Where to write the annotated dump.
//! * `--max-roads N`         How many candidate sectors to dump (default 1).
//! * `--include-multi-item`  Allow sectors with >1 item (still requires the
//!   first item to be a Road).
//!
//! Read-only — never modifies the archive.

use std::fmt::Write as _;
use std::io::Cursor;
use std::path::PathBuf;

use binrw::BinRead;

use truckpilot_map_parser::road_full::RoadFixedHeader;
use truckpilot_map_parser::sector::{audit_sector, AuditReport};
use truckpilot_map_parser::{Archive, HashFsArchive};

/// Item-type code for a Road in the legacy sector format.
const ITEM_TYPE_ROAD: u32 = 3;
/// Sector header (16) + item_count (4) + item_type (4) before the road body.
const ROAD_BODY_START: usize = 16 + 4 + 4;
/// Fixed-header size for a Road body — same value the parser uses (`0x109`).
const ROAD_FIXED_HEADER_LEN: usize = 0x109;

#[derive(Debug)]
struct Args {
    ets2_dir: PathBuf,
    output: PathBuf,
    max_roads: usize,
    include_multi_item: bool,
    audit_skips: bool,
    max_sectors: usize,
}

fn parse_args() -> Args {
    let mut ets2_dir: Option<PathBuf> = None;
    let mut output: Option<PathBuf> = None;
    let mut max_roads = 1usize;
    let mut include_multi_item = false;
    let mut audit_skips = false;
    let mut max_sectors = 50usize;

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
                output = Some(PathBuf::from(
                    argv.get(i + 1).expect("--output needs value"),
                ));
                i += 2;
            }
            "--max-roads" => {
                max_roads = argv
                    .get(i + 1)
                    .expect("--max-roads needs value")
                    .parse()
                    .expect("--max-roads must be a positive integer");
                i += 2;
            }
            "--include-multi-item" => {
                include_multi_item = true;
                i += 1;
            }
            "--audit-skips" => {
                audit_skips = true;
                i += 1;
            }
            "--max-sectors" => {
                max_sectors = argv
                    .get(i + 1)
                    .expect("--max-sectors needs value")
                    .parse()
                    .expect("--max-sectors must be a positive integer");
                i += 2;
            }
            "-h" | "--help" => {
                print_usage();
                std::process::exit(0);
            }
            other => {
                eprintln!("unknown argument: {other}");
                print_usage();
                std::process::exit(2);
            }
        }
    }

    Args {
        ets2_dir: ets2_dir.unwrap_or_else(|| {
            eprintln!("ERROR: --ets2-dir is required");
            print_usage();
            std::process::exit(2);
        }),
        output: output.unwrap_or_else(|| PathBuf::from("road_dump.txt")),
        max_roads,
        include_multi_item,
        audit_skips,
        max_sectors,
    }
}

fn print_usage() {
    eprintln!(
        "usage: truckpilot-road-dump --ets2-dir <PATH> [--output <FILE>] \
         [--max-roads N] [--include-multi-item] \
         [--audit-skips [--max-sectors N]]"
    );
}

#[derive(Clone)]
struct Candidate {
    path: String,
    data: Vec<u8>,
    item_count: u32,
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

    let mut sector_paths: Vec<String> = archive
        .probe_sector_paths()
        .into_iter()
        .filter(|p| p.ends_with(".base"))
        .collect();
    sector_paths.sort();
    eprintln!("probed {} `.base` sector paths", sector_paths.len());

    if args.audit_skips {
        run_audit(&args, &mut archive, &sector_paths);
        return;
    }

    let mut single_item: Vec<Candidate> = Vec::new();
    let mut multi_item: Vec<Candidate> = Vec::new();

    for path in &sector_paths {
        let Ok(data) = archive.read_path(path) else {
            continue;
        };
        if data.len() < ROAD_BODY_START + ROAD_FIXED_HEADER_LEN {
            continue;
        }
        let item_count = u32::from_le_bytes([data[16], data[17], data[18], data[19]]);
        let first_item_type = u32::from_le_bytes([data[20], data[21], data[22], data[23]]);
        if first_item_type != ITEM_TYPE_ROAD {
            continue;
        }

        let cand = Candidate {
            path: path.clone(),
            data,
            item_count,
        };
        if item_count == 1 {
            single_item.push(cand);
            if single_item.len() >= args.max_roads && !args.include_multi_item {
                break;
            }
        } else if args.include_multi_item {
            multi_item.push(cand);
        }
    }

    // Prefer single-item sectors first, then fall back to multi if requested.
    single_item.sort_by_key(|c| c.data.len());
    multi_item.sort_by_key(|c| (c.item_count, c.data.len()));

    let mut chosen: Vec<Candidate> = single_item.into_iter().take(args.max_roads).collect();
    if args.include_multi_item && chosen.len() < args.max_roads {
        let needed = args.max_roads - chosen.len();
        chosen.extend(multi_item.into_iter().take(needed));
    }

    if chosen.is_empty() {
        eprintln!(
            "ERROR: no sectors with first_item_type == 3 found{}",
            if args.include_multi_item {
                " (even with --include-multi-item)"
            } else {
                " — try --include-multi-item"
            }
        );
        std::process::exit(1);
    }

    let mut out = String::new();
    for (idx, cand) in chosen.iter().enumerate() {
        write_sector_section(&mut out, idx + 1, cand);
    }

    std::fs::write(&args.output, out.as_bytes())
        .unwrap_or_else(|e| panic!("write {}: {e}", args.output.display()));
    eprintln!(
        "wrote {} sector dump(s) to {}",
        chosen.len(),
        args.output.display()
    );
}

fn write_sector_section(out: &mut String, index: usize, cand: &Candidate) {
    let post_start = ROAD_BODY_START + ROAD_FIXED_HEADER_LEN;
    let post_bytes = &cand.data[post_start..];

    // Try to parse the fixed header so we can show every recovered field.
    let mut cur = Cursor::new(&cand.data[ROAD_BODY_START..]);
    let header = match RoadFixedHeader::read(&mut cur) {
        Ok(h) => h,
        Err(e) => {
            writeln!(
                out,
                "########## SECTOR #{index}: {} — FIXED HEADER PARSE FAILED ({e:?})\n",
                cand.path
            )
            .ok();
            return;
        }
    };

    writeln!(out, "########## SECTOR #{index} ##########").ok();
    writeln!(out, "=== SECTOR INFO ===").ok();
    writeln!(out, "Path: {}", cand.path).ok();
    writeln!(out, "Total size: {} bytes", cand.data.len()).ok();
    writeln!(out, "Item count: {}", cand.item_count).ok();
    writeln!(out, "First item type: 3 (Road)").ok();
    writeln!(out).ok();

    writeln!(
        out,
        "=== FIXED HEADER ({ROAD_FIXED_HEADER_LEN} bytes, parsed) ==="
    )
    .ok();
    writeln!(out, "UID                       : 0x{:016X}", header.uid).ok();
    writeln!(
        out,
        "StartNode                 : 0x{:016X}",
        header.start_node_uid
    )
    .ok();
    writeln!(
        out,
        "EndNode                   : 0x{:016X}",
        header.end_node_uid
    )
    .ok();
    writeln!(
        out,
        "RoadType                  : 0x{:016X}",
        header.road_type
    )
    .ok();
    writeln!(out, "Length                    : {}", header.length).ok();
    writeln!(out, "DlcGuard                  : {}", header.dlc_guard).ok();
    writeln!(
        out,
        "ViewDistance              : {} m",
        header.view_distance_meters()
    )
    .ok();
    writeln!(out, "Hidden                    : {}", header.is_hidden()).ok();
    writeln!(out, "Secret                    : {}", header.is_secret()).ok();
    writeln!(out, "GpsAvoid                  : {}", header.gps_avoid()).ok();
    writeln!(out, "HighPoly                  : {}", header.is_high_poly()).ok();
    writeln!(out, "Superfine                 : {}", header.is_superfine()).ok();
    writeln!(
        out,
        "LeftHandTraffic           : {}",
        header.is_left_hand_traffic()
    )
    .ok();
    writeln!(
        out,
        "kflags(1..4)              : {:02X} {:02X} {:02X} {:02X}",
        header.kflag1, header.kflag2, header.kflag3, header.kflag4
    )
    .ok();
    writeln!(
        out,
        "rflags(1, 3, 4)           : {:02X} {:02X} {:02X}",
        header.rflag1, header.rflag3, header.rflag4
    )
    .ok();
    writeln!(out, "kdop mins                 : {:?}", header.kdop.mins).ok();
    writeln!(out, "kdop maxs                 : {:?}", header.kdop.maxs).ok();
    writeln!(
        out,
        "RightTrafficRule          : 0x{:016X}",
        header.right_traffic_rule
    )
    .ok();
    writeln!(
        out,
        "LeftTrafficRule           : 0x{:016X}",
        header.left_traffic_rule
    )
    .ok();
    writeln!(
        out,
        "RightVariant              : 0x{:016X}",
        header.right_variant
    )
    .ok();
    writeln!(
        out,
        "LeftVariant               : 0x{:016X}",
        header.left_variant
    )
    .ok();
    writeln!(
        out,
        "RightRightEdge            : 0x{:016X}",
        header.right_right_edge
    )
    .ok();
    writeln!(
        out,
        "RightLeftEdge             : 0x{:016X}",
        header.right_left_edge
    )
    .ok();
    writeln!(
        out,
        "LeftRightEdge             : 0x{:016X}",
        header.left_right_edge
    )
    .ok();
    writeln!(
        out,
        "LeftLeftEdge              : 0x{:016X}",
        header.left_left_edge
    )
    .ok();
    writeln!(
        out,
        "RightTerrainProfile       : 0x{:016X} (coef {})",
        header.right_terrain_profile, header.right_terrain_coefficient
    )
    .ok();
    writeln!(
        out,
        "LeftTerrainProfile        : 0x{:016X} (coef {})",
        header.left_terrain_profile, header.left_terrain_coefficient
    )
    .ok();
    writeln!(
        out,
        "RightLook                 : 0x{:016X}",
        header.right_look
    )
    .ok();
    writeln!(
        out,
        "LeftLook                  : 0x{:016X}",
        header.left_look
    )
    .ok();
    writeln!(
        out,
        "Material                  : 0x{:016X}",
        header.material
    )
    .ok();
    for (i, rail) in header.railings.iter().enumerate() {
        writeln!(
            out,
            "Railing[{i}]                : R=0x{:016X} ofs={:+5} | L=0x{:016X} ofs={:+5}",
            rail.right_model, rail.right_offset, rail.left_model, rail.left_offset
        )
        .ok();
    }
    writeln!(
        out,
        "RightHeightOffset         : {}",
        header.right_height_offset
    )
    .ok();
    writeln!(
        out,
        "LeftHeightOffset          : {}",
        header.left_height_offset
    )
    .ok();
    writeln!(out).ok();

    writeln!(
        out,
        "=== POST-HEADER BYTES ({} bytes, NOT parsed) ===",
        post_bytes.len()
    )
    .ok();
    writeln!(
        out,
        "Offset  | Hex                                              | ASCII            | u32 LE     | u64 LE"
    )
    .ok();
    writeln!(
        out,
        "--------+--------------------------------------------------+------------------+------------+--------------------"
    )
    .ok();
    for (row_idx, chunk) in post_bytes.chunks(16).enumerate() {
        write_hex_row(out, row_idx * 16, chunk);
    }
    writeln!(out).ok();

    write_analysis(out, &header, post_bytes);

    writeln!(out, "=== SUMMARY ===").ok();
    writeln!(out, "Bytes after fixed header: {}", post_bytes.len()).ok();
    if post_bytes.len() >= 16 {
        let last16 = &post_bytes[post_bytes.len() - 16..];
        let hex: String = last16
            .iter()
            .map(|b| format!("{b:02X}"))
            .collect::<Vec<_>>()
            .join(" ");
        writeln!(out, "Last 16 bytes (likely tail markers): {hex}").ok();
    }
    writeln!(out).ok();
    writeln!(out, "{}", "=".repeat(80)).ok();
    writeln!(out).ok();
}

fn write_hex_row(out: &mut String, offset: usize, chunk: &[u8]) {
    let mut hex = String::with_capacity(48);
    for (i, b) in chunk.iter().enumerate() {
        if i > 0 {
            hex.push(' ');
        }
        write!(hex, "{b:02x}").ok();
    }
    let hex_padded = format!("{hex:<47}");

    let mut ascii = String::with_capacity(16);
    for b in chunk {
        let c = if (32..127).contains(b) {
            *b as char
        } else {
            '.'
        };
        ascii.push(c);
    }
    let ascii_padded = format!("{ascii:<16}");

    let u32_str = if chunk.len() >= 4 {
        let v = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
        format!("{v:>10}")
    } else {
        format!("{:>10}", "")
    };
    let u64_str = if chunk.len() >= 8 {
        let mut buf = [0u8; 8];
        buf.copy_from_slice(&chunk[..8]);
        format!("0x{:016X}", u64::from_le_bytes(buf))
    } else {
        format!("{:>18}", "")
    };

    writeln!(
        out,
        "0x{offset:04X}  | {hex_padded} | {ascii_padded} | {u32_str} | {u64_str}"
    )
    .ok();
}

/// Best-effort heuristics that highlight likely structural boundaries inside
/// the post-header bytes.  Looks for:
///   * known node UIDs (start / end) — those usually appear inside the
///     trailing node section.
///   * candidate `(node_count u32, vis_count u32)` boundaries that satisfy
///     the equation `4 + N*56 + 4 + M*8 == bytes_after_split`.
///   * the leading u32 (Phase 5.7's smoking-gun count prefix).
fn write_analysis(out: &mut String, header: &RoadFixedHeader, post: &[u8]) {
    writeln!(out, "=== HEURISTIC ANALYSIS ===").ok();

    if post.len() >= 4 {
        let leading_u32 = u32::from_le_bytes([post[0], post[1], post[2], post[3]]);
        let leading_u64 = if post.len() >= 8 {
            let mut buf = [0u8; 8];
            buf.copy_from_slice(&post[..8]);
            u64::from_le_bytes(buf)
        } else {
            0
        };
        writeln!(
            out,
            "Leading 4 bytes as u32 LE : {leading_u32}  (= 0x{leading_u32:08X})"
        )
        .ok();
        writeln!(out, "Leading 8 bytes as u64 LE : 0x{leading_u64:016X}").ok();
        writeln!(
            out,
            "  → if u32 is a small count (≤ 32), payload likely begins with a count-prefixed list."
        )
        .ok();
    }

    // Find the start_node and end_node UIDs anywhere in the post-header bytes.
    let scan_for = [
        ("StartNode", header.start_node_uid),
        ("EndNode", header.end_node_uid),
    ];
    for (label, uid) in scan_for {
        let needle = uid.to_le_bytes();
        let mut hits = vec![];
        let mut off = 0;
        while off + 8 <= post.len() {
            if post[off..off + 8] == needle {
                hits.push(off);
            }
            off += 1;
        }
        if hits.is_empty() {
            writeln!(
                out,
                "{label} UID 0x{uid:016X} : not present in post-header bytes"
            )
            .ok();
        } else {
            writeln!(
                out,
                "{label} UID 0x{uid:016X} : found at offsets {:?}",
                hits.iter()
                    .map(|o| format!("0x{o:04X}"))
                    .collect::<Vec<_>>()
            )
            .ok();
        }
    }

    // Try to locate a `node_count u32 + nodes(56*N) [+ vis_count u32 + vis_uids(8*M)]`
    // tail.  We sweep candidate split-points from the start of the post-header
    // bytes and report the first few matches.
    let mut tail_candidates: Vec<(usize, usize, Option<usize>, usize)> = Vec::new();
    let limit = post.len();
    if limit >= 4 {
        for split in (0..=limit.saturating_sub(4)).step_by(4) {
            let n_bytes = &post[split..split + 4];
            let n = u32::from_le_bytes([n_bytes[0], n_bytes[1], n_bytes[2], n_bytes[3]]) as usize;
            if n > 4096 {
                continue;
            }
            let after_nodes = split + 4 + n * 56;
            if after_nodes > limit {
                continue;
            }
            let remaining = limit - after_nodes;
            // Variant A: nothing after the nodes.
            if remaining == 0 {
                tail_candidates.push((split, n, None, 0));
                continue;
            }
            // Variant B: vis_count + vis_uids[M].
            if remaining >= 4 {
                let m_bytes = &post[after_nodes..after_nodes + 4];
                let m =
                    u32::from_le_bytes([m_bytes[0], m_bytes[1], m_bytes[2], m_bytes[3]]) as usize;
                if m <= 4096 && (4 + m * 8) == remaining {
                    tail_candidates.push((split, n, Some(after_nodes), m));
                }
            }
            if tail_candidates.len() >= 5 {
                break;
            }
        }
    }
    if tail_candidates.is_empty() {
        writeln!(
            out,
            "No tail (node_count + nodes [+ vis_count + uids]) layout matched."
        )
        .ok();
    } else {
        writeln!(
            out,
            "Possible tail layouts (first {} match[es]):",
            tail_candidates.len()
        )
        .ok();
        for (split, n, vis_at, m) in tail_candidates {
            match vis_at {
                None => writeln!(
                    out,
                    "  • split @ 0x{split:04X}  →  node_count={n} (no vis section)"
                )
                .ok(),
                Some(vis) => writeln!(
                    out,
                    "  • split @ 0x{split:04X}  →  node_count={n}, vis_count={m} (vis @ 0x{vis:04X})"
                )
                .ok(),
            };
        }
        writeln!(
            out,
            "  → bytes 0x0000..first-split therefore belong to the road's variable payload."
        )
        .ok();
    }
    writeln!(out).ok();
}

// ---------------------------------------------------------------------------
// Phase 5.8 — `--audit-skips` mode
// ---------------------------------------------------------------------------

fn run_audit(args: &Args, archive: &mut HashFsArchive, sector_paths: &[String]) {
    use std::collections::HashMap;

    let mut total = 0usize;
    let mut clean = 0usize;
    let mut failures: Vec<(String, Vec<u8>, AuditReport)> = Vec::new();

    for path in sector_paths {
        let Ok(data) = archive.read_path(path) else {
            continue;
        };
        total += 1;
        let report = audit_sector(&data);
        if report.failure.is_some() {
            failures.push((path.clone(), data, report));
        } else {
            clean += 1;
        }
    }

    // Group failures by previous-handler kind.
    let mut by_prev_kind: HashMap<&'static str, usize> = HashMap::new();
    let mut by_raw_type: HashMap<u32, usize> = HashMap::new();
    for (_p, _d, rep) in &failures {
        let prev_kind = rep
            .items
            .last()
            .map(|it| it.kind_name)
            .unwrap_or("<no_items>");
        *by_prev_kind.entry(prev_kind).or_insert(0) += 1;
        if let Some(f) = &rep.failure {
            *by_raw_type.entry(f.raw_type).or_insert(0) += 1;
        }
    }

    let mut out = String::new();
    writeln!(out, "=== AUDIT REPORT ===").ok();
    writeln!(out, "Sectors analyzed     : {total}").ok();
    writeln!(out, "Sectors clean        : {clean}").ok();
    writeln!(out, "Sectors failed       : {}", failures.len()).ok();
    writeln!(out).ok();

    writeln!(out, "=== FAILURES BY PREVIOUS HANDLER ===").ok();
    let mut prev_sorted: Vec<_> = by_prev_kind.iter().collect();
    prev_sorted.sort_by(|a, b| b.1.cmp(a.1));
    for (kind, count) in &prev_sorted {
        writeln!(out, "  {count:5}  {kind}").ok();
    }
    writeln!(out).ok();

    writeln!(out, "=== FAILURES BY RAW TYPE READ ===").ok();
    let mut raw_sorted: Vec<_> = by_raw_type.iter().collect();
    raw_sorted.sort_by(|a, b| b.1.cmp(a.1));
    for (raw, count) in raw_sorted.iter().take(15) {
        writeln!(out, "  {count:5}  type 0x{raw:08X} ({raw})").ok();
    }
    writeln!(out).ok();

    // Per-sector detail: limited to args.max_sectors. Take a stratified sample —
    // first N from each prev-kind bucket so all dominant kinds are represented.
    let sample = stratified_sample(&failures, args.max_sectors);
    writeln!(
        out,
        "=== PER-SECTOR DETAIL (sample of {}/{}) ===",
        sample.len(),
        failures.len()
    )
    .ok();
    writeln!(out).ok();
    for (idx, &fi) in sample.iter().enumerate() {
        let (path, data, rep) = &failures[fi];
        write_failure_detail(&mut out, idx + 1, path, data, rep);
    }

    std::fs::write(&args.output, out.as_bytes())
        .unwrap_or_else(|e| panic!("write {}: {e}", args.output.display()));
    eprintln!(
        "audit complete — {} clean / {} failed (of {} analyzed). Report: {}",
        clean,
        failures.len(),
        total,
        args.output.display()
    );
}

fn stratified_sample(failures: &[(String, Vec<u8>, AuditReport)], cap: usize) -> Vec<usize> {
    use std::collections::HashMap;
    let mut buckets: HashMap<&'static str, Vec<usize>> = HashMap::new();
    for (i, (_, _, rep)) in failures.iter().enumerate() {
        let prev_kind = rep
            .items
            .last()
            .map(|it| it.kind_name)
            .unwrap_or("<no_items>");
        buckets.entry(prev_kind).or_default().push(i);
    }
    let bucket_count = buckets.len().max(1);
    let per_bucket = (cap / bucket_count).max(1);
    let mut chosen: Vec<usize> = Vec::with_capacity(cap);
    for indices in buckets.values() {
        for &i in indices.iter().take(per_bucket) {
            if chosen.len() >= cap {
                break;
            }
            chosen.push(i);
        }
        if chosen.len() >= cap {
            break;
        }
    }
    chosen.sort();
    chosen
}

fn write_failure_detail(
    out: &mut String,
    sample_idx: usize,
    path: &str,
    data: &[u8],
    rep: &AuditReport,
) {
    let failure = rep.failure.as_ref().expect("failure must be present here");
    let prev = rep.items.last();
    writeln!(out, "--- #{sample_idx}: {path} ---").ok();
    writeln!(
        out,
        "Sector size: {} B | item_count: {} | parsed before failure: {}",
        data.len(),
        rep.item_count,
        rep.items.len()
    )
    .ok();

    if rep.items.len() >= 3 {
        let tail = &rep.items[rep.items.len() - 3..];
        writeln!(out, "Last 3 successful items:").ok();
        for it in tail {
            writeln!(
                out,
                "  #{:>3} type={:>3} ({}) @ 0x{:04X}..0x{:04X}  ({} bytes)",
                it.index,
                it.item_type,
                it.kind_name,
                it.start_offset,
                it.end_offset,
                it.end_offset - it.start_offset
            )
            .ok();
        }
    } else if !rep.items.is_empty() {
        writeln!(out, "Successful items:").ok();
        for it in &rep.items {
            writeln!(
                out,
                "  #{:>3} type={:>3} ({}) @ 0x{:04X}..0x{:04X}  ({} bytes)",
                it.index,
                it.item_type,
                it.kind_name,
                it.start_offset,
                it.end_offset,
                it.end_offset - it.start_offset
            )
            .ok();
        }
    } else {
        writeln!(out, "(no successful items before failure)").ok();
    }

    writeln!(
        out,
        "Failure: item #{} read u32 = 0x{:08X} ({}) at offset 0x{:04X} — \"{}\"",
        failure.item_index,
        failure.raw_type,
        failure.raw_type,
        failure.error_offset,
        failure.error_msg
    )
    .ok();

    // Hex preview around the failure point — 16 bytes back, 64 bytes forward.
    let here = failure.error_offset;
    let lo = here.saturating_sub(16);
    let hi = (here + 64).min(data.len());
    writeln!(out).ok();
    writeln!(out, "Hex around failure (slice 0x{lo:04X}..0x{hi:04X}):").ok();
    write_audit_hex(out, &data[lo..hi], lo, here);

    // Plausibility scan: at which negative/positive offsets does a u32 value
    // 1..=48 (a real item type) sit?  That hints how many bytes the previous
    // handler over-/under-read.
    writeln!(
        out,
        "Plausible item-type offsets near failure (read u32 LE):"
    )
    .ok();
    for delta in [-12i32, -8, -4, 0, 4, 8, 12] {
        let abs = here as i64 + delta as i64;
        if abs < 0 || (abs as usize + 4) > data.len() {
            continue;
        }
        let off = abs as usize;
        let v = u32::from_le_bytes([data[off], data[off + 1], data[off + 2], data[off + 3]]);
        let plausible = (1..=48).contains(&v);
        let marker = if plausible { " ← plausible" } else { "" };
        writeln!(
            out,
            "  delta {delta:+3}  offset 0x{off:04X}  u32 = {v:>10} (0x{v:08X}){marker}"
        )
        .ok();
    }
    if let Some(p) = prev {
        writeln!(
            out,
            "Hint: previous handler \"{}\" ended at 0x{:04X}. \
             If it should have advanced N more bytes, the next item_type would sit at 0x{:04X} (+N).",
            p.kind_name, p.end_offset, p.end_offset
        )
        .ok();
    }
    writeln!(out).ok();
    writeln!(out, "{}", "-".repeat(80)).ok();
    writeln!(out).ok();
}

fn write_audit_hex(out: &mut String, bytes: &[u8], start_offset: usize, marker_offset: usize) {
    for (i, chunk) in bytes.chunks(16).enumerate() {
        let row_off = start_offset + i * 16;
        let mut hex = String::with_capacity(48);
        for (j, b) in chunk.iter().enumerate() {
            if j > 0 {
                hex.push(' ');
            }
            write!(hex, "{b:02x}").ok();
        }
        let hex_padded = format!("{hex:<47}");
        let mut ascii = String::with_capacity(16);
        for b in chunk {
            let c = if (32..127).contains(b) {
                *b as char
            } else {
                '.'
            };
            ascii.push(c);
        }
        let here_marker = if (row_off..row_off + chunk.len()).contains(&marker_offset) {
            "  ← failure"
        } else {
            ""
        };
        writeln!(
            out,
            "  0x{row_off:04X}  | {hex_padded} | {ascii:<16} |{here_marker}"
        )
        .ok();
    }
}
