//! `multi-sector-audit` — Phase 6.2b-Fix-5b Step 1
//!
//! Runs `audit_sector` on multiple sectors and reports:
//!   - Last successfully parsed item before failure
//!   - Failure point (offset, raw_type, surrounding hex bytes)
//!   - Second-to-last item (for frequency-bias check)
//!
//! Gate guards:
//!   - Read-only. No parser changes.
//!   - Output goes to stdout (Markdown table).

use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;
use truckpilot_map_parser::sector::audit_sector;
use truckpilot_map_parser::{Archive, HashFsArchive, ModLoadOrder, ZipArchive};

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

#[derive(Parser)]
#[command(
    name = "multi-sector-audit",
    about = "Phase 6.2b-Fix-5b: multi-sector item-handler failure audit"
)]
struct Args {
    /// ETS2 install directory.
    #[arg(long)]
    ets2_dir: PathBuf,

    /// Optional mods dir.
    #[arg(long)]
    mods_dir: Option<PathBuf>,

    /// Sector paths to audit, e.g. map/europe/sec-0001-0007.base
    #[arg(long, required = true, num_args = 1..)]
    sector: Vec<String>,

    /// Context bytes to hex-dump around the failure point.
    #[arg(long, default_value_t = 32)]
    context_bytes: usize,
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

fn hex_dump_line(data: &[u8], base_offset: usize) -> String {
    let hex: Vec<String> = data.iter().map(|b| format!("{b:02X}")).collect();
    let ascii: String = data
        .iter()
        .map(|&b| {
            if b.is_ascii_graphic() || b == b' ' {
                b as char
            } else {
                '.'
            }
        })
        .collect();
    format!("{base_offset:08X}  {:<48}  |{ascii}|", hex.join(" "))
}

fn hex_dump(data: &[u8], base_offset: usize, context: usize) -> String {
    if data.is_empty() {
        return "(empty)".to_string();
    }
    let start = base_offset.saturating_sub(context);
    let actual_start = start.min(data.len());
    let end = (base_offset + context + 16).min(data.len());

    let mut lines = Vec::new();
    let mut i = actual_start;
    while i < end {
        let line_end = (i + 16).min(data.len());
        lines.push(hex_dump_line(&data[i..line_end], i));
        i += 16;
    }
    lines.join("\n")
}

// ---------------------------------------------------------------------------
// Per-sector audit
// ---------------------------------------------------------------------------

struct SectorAuditResult {
    sector_path: String,
    data_len: usize,
    item_count: u32,
    items_parsed: usize,
    last_ok_index: isize,
    last_ok_type: u32,
    last_ok_kind: String,
    #[allow(dead_code)]
    last_ok_start: usize,
    last_ok_end: usize,
    second_last_type: u32,
    second_last_kind: String,
    failure_raw_type: u32,
    failure_offset: usize,
    failure_msg: String,
    sector_data: Vec<u8>,
}

fn audit_one_sector(data: &[u8], sector_path: &str, _context: usize) -> SectorAuditResult {
    let report = audit_sector(data);

    let items_parsed = report.items.len();
    let (last_ok_index, last_ok_type, last_ok_kind, last_ok_start, last_ok_end) =
        if let Some(last) = report.items.last() {
            (
                last.index as isize,
                last.item_type,
                last.kind_name.to_string(),
                last.start_offset,
                last.end_offset,
            )
        } else {
            (-1, 0, "-".to_string(), 0, 0)
        };

    let (second_last_type, second_last_kind) = if report.items.len() >= 2 {
        let sl = &report.items[report.items.len() - 2];
        (sl.item_type, sl.kind_name.to_string())
    } else {
        (0, "-".to_string())
    };

    let (failure_raw_type, failure_offset, failure_msg) = match &report.failure {
        Some(f) => (f.raw_type, f.error_offset, f.error_msg.clone()),
        None => (0, 0, "no failure".to_string()),
    };

    SectorAuditResult {
        sector_path: sector_path.to_string(),
        data_len: report.data_len,
        item_count: report.item_count,
        items_parsed,
        last_ok_index,
        last_ok_type,
        last_ok_kind,
        last_ok_start,
        last_ok_end,
        second_last_type,
        second_last_kind,
        failure_raw_type,
        failure_offset,
        failure_msg,
        sector_data: data.to_vec(),
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() -> Result<()> {
    let args = Args::parse();
    let mods_dir = args.mods_dir.unwrap_or_else(default_mods_dir);
    let context = args.context_bytes;

    // ── Load archives ──────────────────────────────────────────────────────
    eprintln!("Opening archives ...");
    let order = ModLoadOrder::from_directories(&args.ets2_dir, &mods_dir)
        .context("build mod load order")?;
    let mut archives: Vec<Box<dyn Archive>> = Vec::new();
    for entry in &order.entries {
        let arc: Box<dyn Archive> = match HashFsArchive::open(&entry.path) {
            Ok(a) => Box::new(a),
            Err(_) => match ZipArchive::open(&entry.path) {
                Ok(a) => Box::new(a),
                Err(_) => continue,
            },
        };
        archives.push(arc);
    }
    eprintln!("  {} archives open.", archives.len());

    // ── Audit each sector ──────────────────────────────────────────────────
    let mut results: Vec<SectorAuditResult> = Vec::new();
    for sp in &args.sector {
        eprintln!("Auditing {sp} ...");
        let data = archives
            .iter_mut()
            .rev()
            .find_map(|arc| arc.read_path(sp).ok());
        let Some(data) = data else {
            eprintln!("  SKIP: sector data not found in archives");
            continue;
        };
        let r = audit_one_sector(&data, sp, context);
        eprintln!(
            "  {} items parsed, last OK: #{} (type=0x{:X} {})",
            r.items_parsed, r.last_ok_index, r.last_ok_type, r.last_ok_kind
        );
        if r.failure_offset > 0 {
            eprintln!(
                "  FAILURE at 0x{:X}: raw_type=0x{:X}",
                r.failure_offset, r.failure_raw_type
            );
        }
        results.push(r);
    }

    // ── Print Markdown report to stdout ────────────────────────────────────

    // Section A: Per-sector table
    println!("# Multi-Sector Hex-Audit — Phase 6.2b-Fix-5b Step 1\n");
    println!("| Sector | Size | item_count | Items Parsed | LastOK # | LastOK Type | LastOK Kind | LastOK End | Failure Offset | raw_type |");
    println!("|---|---|---|---|---|---|---|---|---|---|");
    for r in &results {
        println!(
            "| `{}` | {} | {} | {} | {} | 0x{:X} | {} | 0x{:X} | 0x{:X} | 0x{:X} |",
            r.sector_path,
            r.data_len,
            r.item_count,
            r.items_parsed,
            r.last_ok_index,
            r.last_ok_type,
            r.last_ok_kind,
            r.last_ok_end,
            r.failure_offset,
            r.failure_raw_type,
        );
    }

    // Section B: Predecessor type distribution
    println!("\n## Last-OK Handler Distribution\n");
    println!("| LastOK Type | LastOK Kind | Count (as last) |");
    println!("|---|---|---|");

    use std::collections::HashMap;
    let mut last_ok_counts: HashMap<String, usize> = HashMap::new();
    for r in &results {
        let key = format!("0x{:X} ({})", r.last_ok_type, r.last_ok_kind);
        *last_ok_counts.entry(key).or_default() += 1;
    }
    // Sort by count desc
    let mut sorted: Vec<_> = last_ok_counts.into_iter().collect();
    sorted.sort_by_key(|b| std::cmp::Reverse(b.1));
    for (key, count) in &sorted {
        println!("| {} | {} |", key, count);
    }

    // Section C: Second-to-last type for frequency bias check
    println!("\n## Second-to-Last Handler (frequency-bias check)\n");
    println!("| Sector | 2nd-last Type | 2nd-last Kind | LastOK Type | LastOK Kind |");
    println!("|---|---|---|---|---|");
    for r in &results {
        println!(
            "| `{}` | 0x{:X} | {} | 0x{:X} | {} |",
            r.sector_path, r.second_last_type, r.second_last_kind, r.last_ok_type, r.last_ok_kind
        );
    }

    // Section D: Failure bytes hex dump per sector
    println!("\n## Failure-Point Hex Dump (context={} bytes)\n", context);
    for r in &results {
        println!("### `{}`\n", r.sector_path);
        println!(
            "Failure at offset 0x{:X}, raw_type=0x{:X}, error: {}",
            r.failure_offset, r.failure_raw_type, r.failure_msg
        );
        println!();
        if r.failure_offset > 0 && r.failure_offset < 1_000_000 && !r.sector_data.is_empty() {
            let hd = hex_dump(&r.sector_data, r.failure_offset, context);
            println!("```");
            println!("{hd}");
            println!("```");
        }
        println!();
    }

    // Section E: Item-type frequency inventory
    println!("\n## Item-Type Inventory (all items across all audited sectors)\n");
    println!("| Type | Kind | Total Occurrences | Sectors Present |");
    println!("|---|---|---|---|");

    let mut type_inventory: HashMap<(u32, String), (usize, usize)> = HashMap::new();
    for r in &results {
        let report = audit_sector(&r.sector_data);
        let mut sector_types: std::collections::HashSet<(u32, String)> =
            std::collections::HashSet::new();
        for item in &report.items {
            let key = (item.item_type, item.kind_name.to_string());
            type_inventory.entry(key.clone()).or_default().0 += 1;
            sector_types.insert(key);
        }
        for st in sector_types {
            type_inventory.entry(st).or_default().1 += 1;
        }
    }
    let mut inv_sorted: Vec<_> = type_inventory.into_iter().collect();
    inv_sorted.sort_by_key(|b| std::cmp::Reverse(b.1 .0));
    for ((it, kind), (total, sectors)) in &inv_sorted {
        println!("| 0x{:X} | {} | {} | {} |", it, kind, total, sectors);
    }

    // Section F: Diagnosis
    println!("\n## Diagnosis\n");

    // Check if bezier_patch is the dominant last-OK type
    let bezier_as_last = results.iter().filter(|r| r.last_ok_type == 39).count();
    let road_as_last = results.iter().filter(|r| r.last_ok_type == 3).count();
    let prefab_as_last = results.iter().filter(|r| r.last_ok_type == 4).count();
    let total = results.len();

    println!(
        "- bezier_patch (0x27) as last-OK: {}/{}",
        bezier_as_last, total
    );
    println!("- road (0x03) as last-OK: {}/{}", road_as_last, total);
    println!("- prefab (0x04) as last-OK: {}/{}", prefab_as_last, total);
    println!();

    if bezier_as_last >= 4 {
        println!("**Diagnosis: Szenario 1 — bezier_patch dominant.**");
        println!("bezier_patch (0x27) is the last successfully parsed item in {bezier_as_last}/{total} sectors.");
        println!("Recommendation: Fix-5b Step 2 — audit and fix bezier_patch handler.");
    } else if bezier_as_last >= 2 {
        println!("**Diagnosis: Szenario 2 — Multiple handler types are 'last'.**");
        println!("bezier_patch is last in {bezier_as_last}/{total}, road in {road_as_last}/{total}, prefab in {prefab_as_last}/{total}.");
        println!("Recommendation: Inventory all variable-length handlers, fix each.");
    } else if road_as_last + prefab_as_last >= 3 {
        println!("**Diagnosis: Szenario 3 — Fixed-size handler as last-OK.**");
        println!("Road/prefab (fixed-size) as last-OK suggests an EARLIER handler already desynced the cursor.");
        println!("Recommendation: Backward-walk to find the desyncing handler (check second-to-last types).");
    } else {
        println!("**Diagnosis: Szenario 4 — No clear pattern.**");
        println!("Could not identify a dominant last-OK handler type.");
        println!("Recommendation: STOP, present data to Philipp for decision.");
    }

    eprintln!("Done.");
    Ok(())
}
