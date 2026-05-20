//! `node-parser-hex-audit` — Phase 6.2b-Fix-4
//!
//! Hex-level forensic audit of a single sector. Runs `audit_sector` to
//! find item boundaries, then brute-force scans for a target Node-UID and
//! reports where it is relative to the node section.
//!
//! Gate guards:
//!   - Read-only. No parser changes.
//!   - Output goes to stdout + outputs/YYYY-MM-DD/node_parser_skip_audit.md

use std::io::Write as IoWrite;
use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;
use truckpilot_map_parser::{HashFsArchive, ModLoadOrder, ZipArchive};
use truckpilot_map_parser::sector::{audit_sector, AuditReport};

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

#[derive(Parser)]
#[command(
    name = "node-parser-hex-audit",
    about = "Phase 6.2b-Fix-4: hex-dump a sector and locate a target Node-UID relative to the node section"
)]
struct Args {
    /// ETS2 install directory.
    #[arg(long)]
    ets2_dir: PathBuf,

    /// Optional mods dir.
    #[arg(long)]
    mods_dir: Option<PathBuf>,

    /// Sector path inside archives, e.g. map/europe/sec+0008+0011.base
    #[arg(long)]
    sector_path: String,

    /// Target node UID (decimal or 0x hex).
    #[arg(long, value_parser = parse_u64)]
    target_uid: u64,

    /// Context window in bytes around each UID hit.
    #[arg(long, default_value_t = 256)]
    window: usize,

    /// Output directory.
    #[arg(long, default_value = "outputs/2026-05-17")]
    output_dir: PathBuf,
}

fn parse_u64(s: &str) -> Result<u64, String> {
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u64::from_str_radix(hex, 16).map_err(|e| e.to_string())
    } else {
        s.parse::<u64>().map_err(|e| e.to_string())
    }
}

fn default_mods_dir() -> PathBuf {
    dirs::document_dir()
        .or_else(dirs::home_dir)
        .unwrap_or_default()
        .join("Euro Truck Simulator 2/mod")
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn hex_line(data: &[u8], base_offset: usize) -> String {
    let hex: Vec<String> = data.iter().map(|b| format!("{b:02X}")).collect();
    let ascii: String = data
        .iter()
        .map(|&b| if b.is_ascii_graphic() || b == b' ' { b as char } else { '.' })
        .collect();
    format!("{base_offset:08X}  {:<48}  |{ascii}|", hex.join(" "))
}

fn hex_dump(data: &[u8], base_offset: usize) -> Vec<String> {
    let mut lines = Vec::new();
    let mut i = 0;
    while i < data.len() {
        let end = (i + 16).min(data.len());
        lines.push(hex_line(&data[i..end], base_offset + i));
        i += 16;
    }
    lines
}

fn le_u32(data: &[u8], off: usize) -> Option<u32> {
    data.get(off..off + 4).and_then(|b| b.try_into().ok()).map(u32::from_le_bytes)
}

fn le_i32(data: &[u8], off: usize) -> Option<i32> {
    data.get(off..off + 4).and_then(|b| b.try_into().ok()).map(i32::from_le_bytes)
}

fn le_u64(data: &[u8], off: usize) -> Option<u64> {
    data.get(off..off + 8).and_then(|b| b.try_into().ok()).map(u64::from_le_bytes)
}

fn le_f64(data: &[u8], off: usize) -> Option<f64> {
    data.get(off..off + 8).and_then(|b| b.try_into().ok()).map(f64::from_le_bytes)
}

// ---------------------------------------------------------------------------
// Node-region parsers (read-only)
// ---------------------------------------------------------------------------

/// Attempt to read a legacy 56-byte node record from `data` at `offset`.
/// Returns (uid, x, y, z) where xyz are in metres.
fn read_legacy_node(data: &[u8], offset: usize) -> Option<(u64, f32, f32, f32)> {
    let uid = le_u64(data, offset)?;
    let x = le_i32(data, offset + 8)? as f32 / 256.0;
    let y = le_i32(data, offset + 12)? as f32 / 256.0;
    let z = le_i32(data, offset + 16)? as f32 / 256.0;
    Some((uid, x, y, z))
}

/// Attempt to read a sized 36-byte node record from `data` at `offset`.
fn read_sized_node(data: &[u8], offset: usize) -> Option<(u64, f64, f64, f64)> {
    let uid = le_u64(data, offset)?;
    let x = le_f64(data, offset + 8)?;
    let y = le_f64(data, offset + 16)?;
    let z = le_f64(data, offset + 24)?;
    Some((uid, x, y, z))
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() -> Result<()> {
    let args = Args::parse();
    let mods_dir = args.mods_dir.clone().unwrap_or_else(default_mods_dir);
    std::fs::create_dir_all(&args.output_dir)?;

    let out_path = args.output_dir.join("node_parser_skip_audit.md");

    // ── Load archives ──────────────────────────────────────────────────────
    eprintln!("Opening archives ...");
    let order = ModLoadOrder::from_directories(&args.ets2_dir, &mods_dir)
        .context("build mod load order")?;
    let mut archives = Vec::new();
    for entry in &order.entries {
        let arc: Box<dyn truckpilot_map_parser::Archive> = match HashFsArchive::open(&entry.path) {
            Ok(a) => Box::new(a),
            Err(_) => match ZipArchive::open(&entry.path) {
                Ok(a) => Box::new(a),
                Err(_) => continue,
            },
        };
        archives.push(arc);
    }
    eprintln!("  {} archives open.", archives.len());

    // ── Read sector ────────────────────────────────────────────────────────
    eprintln!("Reading sector {} ...", args.sector_path);
    let data: Vec<u8> = archives
        .iter_mut()
        .rev()
        .find_map(|arc| arc.read_path(&args.sector_path).ok())
        .with_context(|| format!("sector not found in any archive: {}", args.sector_path))?;
    eprintln!("  {} bytes", data.len());

    // ── Run audit_sector ───────────────────────────────────────────────────
    eprintln!("Running audit_sector ...");
    let report: AuditReport = audit_sector(&data);

    let items_end_offset: usize = if let Some(last) = report.items.last() {
        last.end_offset
    } else {
        20 // at minimum skip header
    };

    // The node section starts right after the items (+ 4 bytes for item_count in legacy, but
    // audit_sector already consumed header+item_count, so last.end_offset is the cursor position
    // after the last parsed item, which is exactly where node_count should be).
    let node_section_start = items_end_offset;

    // ── Brute-force scan for target UID ────────────────────────────────────
    let uid_bytes = args.target_uid.to_le_bytes();
    let mut hit_offsets: Vec<usize> = Vec::new();
    if data.len() >= 8 {
        for i in 0..=(data.len() - 8) {
            if data[i..i + 8] == uid_bytes {
                hit_offsets.push(i);
            }
        }
    }

    // ── Classify hits ──────────────────────────────────────────────────────
    // A hit is "in node section" if offset >= node_section_start (or >= failure offset).
    let classify_hit = |off: usize| -> &'static str {
        if off >= node_section_start {
            "NODE-SECTION"
        } else {
            "ITEM-BODY"
        }
    };

    // ── Build report ──────────────────────────────────────────────────────
    let mut lines: Vec<String> = Vec::new();

    lines.push("# Node-Parser Hex-Audit".into());
    lines.push(format!("> Phase 6.2b-Fix-4 | sector `{}` | UID `0x{:016X}` ({})",
        args.sector_path, args.target_uid, args.target_uid));
    lines.push(String::new());

    // ── 1. Sector overview ─────────────────────────────────────────────────
    lines.push("## 1. Sector Overview".into());
    lines.push(String::new());
    lines.push("| Field | Value |".to_string());
    lines.push("|---|---|".to_string());
    lines.push(format!("| File size | {} bytes | ", data.len()));
    lines.push(format!("| Items in audit_sector | {} |", report.items.len()));
    lines.push(format!("| item_count field | {} |", report.item_count));
    lines.push(format!("| Last item end offset | 0x{:X} ({}) |",
        items_end_offset, items_end_offset));
    lines.push(format!("| Expected node_section_start | 0x{:X} ({}) |",
        node_section_start, node_section_start));
    if let Some(ref f) = report.failure {
        lines.push(format!("| **Audit failure** | item #{}, raw_type=0x{:X}, offset=0x{:X}: `{}` |",
            f.item_index, f.raw_type, f.error_offset, f.error_msg));
    } else {
        lines.push("| Audit failure | none |".into());
    }
    lines.push(String::new());

    // ── 2. Sector header hex ───────────────────────────────────────────────
    lines.push("## 2. Sector Header (first 64 bytes)".into());
    lines.push(String::new());
    lines.push("```".into());
    // Legacy header layout annotation
    lines.push("// offset | hex                                             | ascii".into());
    lines.push("// Legacy: CoreMapVersion(4) + GameId(8) + GameMapVersion(4) = 16B header".into());
    let header_len = 64.min(data.len());
    for l in hex_dump(&data[..header_len], 0) {
        lines.push(l);
    }
    lines.push("```".into());
    lines.push(String::new());

    // Decode header fields
    if data.len() >= 20 {
        let core_ver = le_u32(&data, 0).unwrap_or(0);
        let game_id = le_u64(&data, 4).unwrap_or(0);
        let game_map_ver_legacy = le_u32(&data, 12).unwrap_or(0);
        let item_count_legacy = le_u32(&data, 16).unwrap_or(0);
        // Sized header: version(4)+game_id(8)+map_ver(8)+item_count(4) = 24B
        let map_ver_sized_hi = le_u32(&data, 12).unwrap_or(0);
        let map_ver_sized_lo = le_u32(&data, 16).unwrap_or(0);
        let item_count_sized = le_u32(&data, 20).unwrap_or(0);
        lines.push(format!("**Legacy interpretation:** CoreMapVersion={core_ver}, GameId=0x{game_id:016X}, GameMapVersion={game_map_ver_legacy}, item_count={item_count_legacy}"));
        lines.push(String::new());
        lines.push(format!("**Sized interpretation:** version={core_ver}, game_id=0x{game_id:016X}, map_version=0x{map_ver_sized_hi:08X}{map_ver_sized_lo:08X}, item_count={item_count_sized}"));
        lines.push(String::new());
    }

    // ── 3. Item summary ────────────────────────────────────────────────────
    lines.push("## 3. Item Walk (audit_sector)".into());
    lines.push(String::new());
    if report.items.is_empty() {
        lines.push("No items walked successfully.".into());
    } else {
        lines.push("| # | type | kind | start_offset | end_offset | size |".to_string());
        lines.push("|---|---|---|---|---|---|".to_string());
        // Show last 5 items before failure + first 3
        let show_first = 3usize;
        let show_last = 5usize;
        let n = report.items.len();
        let mut shown = std::collections::HashSet::new();
        let indices: Vec<usize> = (0..show_first.min(n))
            .chain(n.saturating_sub(show_last)..n)
            .collect();
        for &i in &indices {
            if shown.contains(&i) {
                continue;
            }
            shown.insert(i);
            let it = &report.items[i];
            let size = it.end_offset.saturating_sub(it.start_offset);
            lines.push(format!("| {} | 0x{:02X} | {} | 0x{:X} | 0x{:X} | {} |",
                it.index, it.item_type, it.kind_name,
                it.start_offset, it.end_offset, size));
        }
        if n > show_first + show_last {
            lines.push(format!("| … | ({} more items) | … | … | … | … |", n - show_first - show_last));
        }
    }
    if let Some(ref f) = report.failure {
        lines.push(String::new());
        lines.push(format!("**Failure at item #{}:** raw_type=0x{:08X} (`{}`), error_offset=0x{:X}",
            f.item_index, f.raw_type, f.error_msg, f.error_offset));
        // Show 32B before the failure
        let pre_start = f.error_offset.saturating_sub(32);
        let pre_end = (f.error_offset + 16).min(data.len());
        if pre_start < pre_end {
            lines.push(String::new());
            lines.push("Bytes around failure point (−32..+16):".into());
            lines.push("```".into());
            for l in hex_dump(&data[pre_start..pre_end], pre_start) {
                lines.push(l);
            }
            // Annotate the failure offset
            lines.push(format!("// ↑ failure at 0x{:X} — read this u32 as item_type=0x{:08X}", f.error_offset, f.raw_type));
            lines.push("```".into());
        }
        // What item type is this raw value?
        let known_types = [
            (1u32, "road"), (2, "buildings"), (3, "curve"), (4, "model"),
            (5, "company"), (6, "service"), (7, "cut_plane"), (8, "city"),
            (9, "map_overlay"), (10, "ferry"), (11, "garage"), (12, "trigger"),
            (13, "fuel_pump"), (14, "sign"), (15, "bus_stop"), (16, "traffic_area"),
            (17, "bezier_patch"), (18, "trajectory"), (19, "map_area"),
            (20, "far_model"), (21, "curve"), (25, "cutscene"), (34, "visibility_area"),
        ];
        let raw = f.raw_type;
        let known = known_types.iter().find(|(t, _)| *t == raw).map(|(_, n)| *n);
        if let Some(k) = known {
            lines.push(format!("raw_type 0x{raw:X} = known item type `{k}` (cursor is ALIGNED, handler returned error)"));
        } else {
            lines.push(format!("raw_type 0x{raw:X} does NOT match any known item type → **cursor was desynced by a preceding handler**"));
        }
    }
    lines.push(String::new());

    // ── 4. UID hit locations ───────────────────────────────────────────────
    lines.push("## 4. Target UID Hit Locations".into());
    lines.push(String::new());
    lines.push(format!("Target UID: `0x{:016X}` ({})", args.target_uid, args.target_uid));
    lines.push(format!("Hits found: **{}**", hit_offsets.len()));
    lines.push(String::new());
    if hit_offsets.is_empty() {
        lines.push("⚠ UID not found in sector binary. Wrong sector?".into());
    } else {
        lines.push("| Hit # | File offset | Region | Notes |".to_string());
        lines.push("|---|---|---|---|".to_string());
        for (i, &off) in hit_offsets.iter().enumerate() {
            let region = classify_hit(off);
            let note = if off >= node_section_start {
                // Try to decode as legacy node
                let from_ns = off - node_section_start;
                let ns_bytes = &data[node_section_start..];
                let node_count_at_ns = le_u32(ns_bytes, 0).unwrap_or(0);
                // Offset within node array (after 4-byte count)
                if from_ns >= 4 {
                    let idx = (from_ns - 4) / 56;
                    let rem = (from_ns - 4) % 56;
                    format!("ns+{from_ns} (node_count={node_count_at_ns}, array[{idx}]+{rem})")
                } else {
                    format!("ns+{from_ns} (within node_count field, node_count={node_count_at_ns})")
                }
            } else {
                // Find which item contains this offset
                let item = report.items.iter().find(|it| it.start_offset <= off && off < it.end_offset);
                if let Some(it) = item {
                    let rel = off - it.start_offset;
                    format!("inside item #{} ({}, rel+{})", it.index, it.kind_name, rel)
                } else if report.failure.as_ref().is_some_and(|f| off >= f.error_offset) {
                    "after failure point (unwalked items region)".into()
                } else {
                    "between items or in header".into()
                }
            };
            lines.push(format!("| {} | 0x{:X} | **{}** | {} |", i + 1, off, region, note));
        }
        lines.push(String::new());

        // ── 5. 256B window around each hit ──────────────────────────────────
        lines.push("## 5. Hex Window Around UID Hits (±{} bytes)".replace("{}", &args.window.to_string()));
        lines.push(String::new());
        for (i, &off) in hit_offsets.iter().enumerate().take(4) {
            let win_start = off.saturating_sub(args.window / 2);
            let win_end = (off + args.window / 2 + 8).min(data.len());
            lines.push(format!("### Hit {} — offset 0x{:X} ({})", i + 1, off, classify_hit(off)));
            lines.push(String::new());
            lines.push("```".into());
            for l in hex_dump(&data[win_start..win_end], win_start) {
                // Mark the UID position
                if off >= win_start && off < win_end {
                    let rel = off - win_start;
                    let row = (rel / 16) * 16 + win_start;
                    if row == (off / 16) * 16 {
                        // already in hex_dump
                    }
                }
                lines.push(l);
            }
            lines.push(format!("// ↑ Target UID 0x{:016X} at offset 0x{:X}", args.target_uid, off));
            // Interpret the window as possible node records
            lines.push(String::new());
            // Try legacy node interpretation at the hit
            if let Some((uid, x, y, z)) = read_legacy_node(&data, off) {
                lines.push(format!("// Legacy node @ 0x{:X}: uid=0x{uid:016X}, x={x:.1}m, y={y:.1}m, z={z:.1}m", off));
            }
            if let Some((uid, x, y, z)) = read_sized_node(&data, off) {
                lines.push(format!("// Sized node @ 0x{:X}: uid=0x{uid:016X}, x={x:.1}m, y={y:.1}m, z={z:.1}m", off));
            }
            lines.push("```".into());
            lines.push(String::new());
        }
    }

    // ── 6. Node section dump ───────────────────────────────────────────────
    lines.push("## 6. Node Section Dump".into());
    lines.push(String::new());
    let ns = &data[node_section_start..];
    if ns.len() < 4 {
        lines.push(format!("Node section at 0x{node_section_start:X}: < 4 bytes remaining ({} bytes). Empty sector tail.",
            ns.len()));
    } else {
        let node_count = le_u32(ns, 0).unwrap_or(0);
        let remaining_after_count = ns.len().saturating_sub(4);
        lines.push(format!("Node section starts at offset 0x{node_section_start:X}"));
        lines.push(format!("node_count field (u32 LE): **{node_count}**"));
        lines.push(format!("Bytes remaining after count: {remaining_after_count}"));
        lines.push(format!("If legacy (56B/node): {remaining_after_count}/56 = {} nodes, expected {node_count}",
            remaining_after_count / 56));
        lines.push(format!("If sized  (36B/node): {remaining_after_count}/36 = {} nodes, expected {node_count}",
            remaining_after_count / 36));
        lines.push(String::new());

        // Check plausibility for recover_nodes_from_tail
        let legacy_fits = remaining_after_count == node_count as usize * 56;
        let sized_fits  = remaining_after_count == node_count as usize * 36;
        lines.push(format!("Legacy layout exact: {legacy_fits}"));
        lines.push(format!("Sized layout exact:  {sized_fits}"));
        lines.push(String::new());

        // Hex dump first 256B of node section
        let dump_len = 256.min(ns.len());
        lines.push("First 256 bytes of node section:".into());
        lines.push("```".into());
        for l in hex_dump(&ns[..dump_len], node_section_start) {
            lines.push(l);
        }
        lines.push("```".into());
        lines.push(String::new());

        // Decode first 3 node records (both formats)
        lines.push("**Legacy node decode (first 5 records, 56B each):**".into());
        lines.push(String::new());
        lines.push("| # | file_offset | uid | x_raw | x_m | y_m | z_m |".into());
        lines.push("|---|---|---|---|---|---|---|".into());
        for i in 0..5usize {
            let off = node_section_start + 4 + i * 56;
            if let Some((uid, x, y, z)) = read_legacy_node(&data, off) {
                let x_raw = le_i32(&data, off + 8).unwrap_or(0);
                lines.push(format!("| {i} | 0x{off:X} | 0x{uid:016X} | {x_raw} | {x:.1} | {y:.1} | {z:.1} |"));
            } else {
                break;
            }
        }
        lines.push(String::new());

        lines.push("**Sized node decode (first 5 records, 36B each):**".into());
        lines.push(String::new());
        lines.push("| # | file_offset | uid | x_m | y_m | z_m |".into());
        lines.push("|---|---|---|---|---|---|".into());
        for i in 0..5usize {
            let off = node_section_start + 4 + i * 36;
            if let Some((uid, x, y, z)) = read_sized_node(&data, off) {
                lines.push(format!("| {i} | 0x{off:X} | 0x{uid:016X} | {x:.1} | {y:.1} | {z:.1} |"));
            } else {
                break;
            }
        }
        lines.push(String::new());
    }

    // ── 7. recover_nodes_from_tail analysis ───────────────────────────────
    lines.push("## 7. recover_nodes_from_tail Analysis".into());
    lines.push(String::new());
    if report.failure.is_some() {
        lines.push("audit_sector found a failure → `all_items_parsed = false` → `recover_nodes_from_tail` would be called.".to_string());
        lines.push(String::new());
        // Simulate recover_nodes_from_tail by looking at the sector tail
        let total = data.len();
        // Check for vis_count = 0 at the end (m=0 case)
        let vis0_pos = total.saturating_sub(4);
        let vis0 = le_u32(&data, vis0_pos).unwrap_or(0xFFFF);
        lines.push(format!("Tail check (m=0): vis_count at offset 0x{vis0_pos:X} = {vis0}"));
        if vis0 == 0 {
            // Look for node_count just before
            let nodes_end = vis0_pos;
            // Try a few n values
            lines.push("vis_count=0 found. Checking node_count candidates:".into());
            for n in [0usize, 1, 2, 3, 5, 10, 50, 100] {
                let block = 4 + n * 56;
                if block > nodes_end {
                    break;
                }
                let count_pos = nodes_end - block;
                if count_pos < 16 {
                    break;
                }
                let count_at = le_u32(&data, count_pos).unwrap_or(0xFFFF) as usize;
                let matches = count_at == n;
                if matches {
                    lines.push(format!("  ✅ n={n}: count at 0x{count_pos:X} = {count_at} (MATCH! Legacy 56B layout would be accepted)"));
                } else if n < 5 {
                    lines.push(format!("  n={n}: count at 0x{count_pos:X} = {count_at} (no match)"));
                }
            }
        } else {
            lines.push(format!("vis_count ≠ 0 for m=0 (got {vis0}). recover_nodes_from_tail will sweep further."));
            lines.push(String::new());
            // Check m=1,2,3 quickly
            for m in 1usize..=3 {
                let vis_block = 4 + m * 8;
                if vis_block > total { break; }
                let vis_count_pos = total - vis_block;
                if vis_count_pos < 20 { break; }
                let vc = le_u32(&data, vis_count_pos).unwrap_or(0xFFFF) as usize;
                if vc == m {
                    lines.push(format!("  vis_count=m={m} matched at 0x{vis_count_pos:X}. Would then search for node_count."));
                }
            }
            // Test the actual node_count value at the computed node_section_start
            if node_section_start + 4 <= data.len() {
                let nc_at_expected = le_u32(&data, node_section_start).unwrap_or(0);
                let expected_legacy_end = node_section_start + 4 + nc_at_expected as usize * 56;
                lines.push(format!("  Expected node_section_start=0x{node_section_start:X}: node_count={nc_at_expected}, legacy_end=0x{expected_legacy_end:X}, file_end=0x{total:X}"));
                if expected_legacy_end <= total {
                    let gap = total - expected_legacy_end;
                    lines.push(format!("  Gap after legacy nodes: {gap} bytes (= vis_block bytes?)"));
                    // Check if vis_count at expected_legacy_end is plausible
                    if gap >= 4 {
                        let vc_at_gap = le_u32(&data, expected_legacy_end).unwrap_or(0xFFFF);
                        let expected_vis_bytes = gap.saturating_sub(4);
                        let vis_fits = expected_vis_bytes.is_multiple_of(8) && vc_at_gap as usize == expected_vis_bytes / 8;
                        lines.push(format!("  vis_count field at 0x{expected_legacy_end:X} = {vc_at_gap}, fits={vis_fits}"));
                    }
                }
            }
        }
    } else {
        lines.push("audit_sector succeeded (no failure) → normal node path would be used.".into());
        lines.push("⚠ BothUnresolved events for this sector are from a DIFFERENT root cause.".into());
        lines.push(String::new());
        lines.push("Possible causes:".into());
        lines.push("- `try_parse_sized_sector` accepted the sector but parsed nodes at wrong stride (36B instead of 56B)".into());
        lines.push("- The node UIDs found by brute-force scan are from ROAD ITEM BODIES (node_a/node_b refs), not actual node definitions".into());
        lines.push("- A version/format mismatch makes node UIDs compute to different values".into());
    }
    lines.push(String::new());

    // ── 8. Hypothesis ─────────────────────────────────────────────────────
    lines.push("## 8. Hypothesis".into());
    lines.push(String::new());
    if report.failure.is_some() {
        let is_desync = report.failure.as_ref().is_some_and(|f| {
            !matches!(f.raw_type, 1|2|3|4|5|6|7|8|9|10|11|12|13|14|15|16|17|18|19|20|21|25|34)
        });
        if is_desync {
            lines.push("**Hypothesis C: Cursor-Desync → recover_nodes_from_tail Failure**".into());
            lines.push(String::new());
            let raw_type_hex = report.failure.as_ref().map_or(0, |f| f.raw_type);
            lines.push(format!("A preceding item handler consumed the wrong number of bytes. The cursor drifted into item body data, and `read_u32` for the next item_type returned garbage (0x{raw_type_hex:08X}) — not a valid ETS2 item type."));
            lines.push(String::new());
            lines.push("Chain:".into());
            lines.push("1. Item handler reads N bytes instead of M → cursor at wrong position".into());
            lines.push("2. `read_u32` reads garbage as item_type → `UnknownItemType` → `all_items_parsed = false`".into());
            lines.push("3. `recover_nodes_from_tail` is called as fallback".into());
            lines.push("4. If it fails to find the `(vis_count, node_count)` layout → zero nodes extracted".into());
            lines.push("5. Graph builder: road endpoints can't be resolved → `both_unresolved`".into());
            lines.push(String::new());
            let last_ok = report.items.last();
            if let Some(it) = last_ok {
                lines.push(format!("Last successfully parsed item: #{} ({}, ended at 0x{:X}). **This handler or the one immediately after is the likely desync point.**",
                    it.index, it.kind_name, it.end_offset));
            }
        } else {
            lines.push("**Hypothesis: known item_type but handler failed** (rare)".into());
        }
    } else {
        lines.push("**Hypothesis D: Sized-Format False-Positive or Road-Body False Hit**".into());
        lines.push(String::new());
        lines.push("The sector parsed without any cursor desync. The BothUnresolved events suggest:".into());
        lines.push("- Either `try_parse_sized_sector` was accepted but nodes were parsed at wrong stride".into());
        lines.push("- Or the UID hits from brute-force are in road item bodies (false positives)".into());
        lines.push("- Needs further investigation: check if UID hits are in item bodies or node section".into());
    }
    lines.push(String::new());

    // ── Write output ───────────────────────────────────────────────────────
    let content = lines.join("\n");
    println!("{content}");
    let mut f = std::fs::File::create(&out_path)
        .with_context(|| format!("create {:?}", out_path))?;
    f.write_all(content.as_bytes())?;
    eprintln!("\nWrote {}", out_path.display());

    Ok(())
}
