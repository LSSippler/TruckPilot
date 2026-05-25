//! `bezier-size-scan` — Phase 6.2b-Fix-5b Step 2 hex-empirie
//!
//! For each sector: finds the bezier_patch item, then brute-force scans
//! skip sizes 50..=2000 bytes to find the smallest skip that allows all
//! remaining sector items to parse cleanly (items_parsed == item_count).
//!
//! This gives the EMPIRICAL bezier_patch body size for each sector.
//!
//! Gate: read-only, no parser changes.

use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;
use truckpilot_map_parser::sector::audit_sector;
use truckpilot_map_parser::{Archive, HashFsArchive, ModLoadOrder, ZipArchive};

#[derive(Parser)]
#[command(name = "bezier-size-scan")]
struct Args {
    #[arg(long)]
    ets2_dir: PathBuf,
    #[arg(long)]
    mods_dir: Option<PathBuf>,
    #[arg(long, required = true, num_args = 1..)]
    sector: Vec<String>,
    /// Max body bytes to try
    #[arg(long, default_value_t = 2000)]
    max_bytes: usize,
    /// Context bytes to hex-dump around the bezier body
    #[arg(long, default_value_t = 512)]
    dump_bytes: usize,
}

fn default_mods_dir() -> PathBuf {
    dirs::document_dir()
        .or_else(dirs::home_dir)
        .unwrap_or_default()
        .join("Euro Truck Simulator 2/mod")
}

// ── Attempt to parse the sector items starting from `cursor_pos` ─────────────
// Returns (items_parsed, items_needed) using audit_sector on a patched sector
// copy where bytes [bezier_start..cursor_pos] are zero-padded so the real
// data is intact and the audit skips from the right offset.
//
// Simpler approach: just run audit_sector on the original data but with a fake
// bezier_patch that reads exactly `body_size` bytes by temporarily patching.
//
// Even simpler: we know the audit will fail at bezier_patch. We need to check
// if skipping `body_size` bytes and resuming produces a clean parse.
// We do this by building a synthetic continuation sector that starts with the
// remaining item count and items from the current cursor position.

fn try_skip_and_parse(
    data: &[u8],
    _sector_header: &[u8], // 20 bytes: version(4) + game_id(8) + map_version(4) + item_count(4)
    items_before_bezier: usize,
    bezier_item_start: usize, // offset of bezier item_type u32 in data
    body_size: usize,         // how many bytes to skip after item_type u32
    item_count: u32,
) -> usize {
    // After item_type u32 (4 bytes) + body_size bytes, cursor should be here:
    let resume_offset = bezier_item_start + 4 + body_size;
    if resume_offset >= data.len() {
        return 0;
    }

    // Build a synthetic mini-sector:
    // header: version=1 u32 + game_id=0 u64 + map_version=0 u32 + remaining_count u32
    // body: data[resume_offset..]
    let remaining_items = item_count as usize - items_before_bezier - 1;
    if remaining_items == 0 {
        // All items before bezier already done, bezier is last — skip is trivially "valid"
        return item_count as usize;
    }

    let remaining_count = remaining_items as u32;
    let mut mini = Vec::with_capacity(20 + data.len() - resume_offset);
    mini.extend_from_slice(&1u32.to_le_bytes()); // version
    mini.extend_from_slice(&0u64.to_le_bytes()); // game_id
    mini.extend_from_slice(&0u32.to_le_bytes()); // map_version
    mini.extend_from_slice(&remaining_count.to_le_bytes()); // item_count
    mini.extend_from_slice(&data[resume_offset..]);

    let report = audit_sector(&mini);
    // items_parsed includes only those in the mini sector
    // items_before_bezier + 1 (bezier) + report.items.len() = total parsed
    items_before_bezier + 1 + report.items.len()
}

fn hex_dump_range(data: &[u8], start: usize, len: usize) -> String {
    let end = (start + len).min(data.len());
    let slice = &data[start..end];
    let mut lines = Vec::new();
    let mut i = 0;
    while i < slice.len() {
        let line_end = (i + 16).min(slice.len());
        let hex: Vec<String> = slice[i..line_end]
            .iter()
            .map(|b| format!("{b:02X}"))
            .collect();
        let asc: String = slice[i..line_end]
            .iter()
            .map(|&b| {
                if b.is_ascii_graphic() || b == b' ' {
                    b as char
                } else {
                    '.'
                }
            })
            .collect();
        lines.push(format!("{:08X}  {:<48}  |{asc}|", start + i, hex.join(" ")));
        i += 16;
    }
    lines.join("\n")
}

fn analyze_sector(data: &[u8], _sp: &str, max_bytes: usize, dump_bytes: usize) {
    let report = audit_sector(data);
    let item_count = report.item_count;

    // Find the last bezier_patch item
    let bezier = report.items.iter().rev().find(|it| it.item_type == 39);
    let Some(bezier) = bezier else {
        println!(
            "  No bezier_patch found — sector parsed {}/{} items cleanly.",
            report.items.len(),
            item_count
        );
        if let Some(f) = &report.failure {
            println!(
                "  failure at item #{}: offset=0x{:X}, raw_type=0x{:08X}",
                f.item_index, f.error_offset, f.raw_type
            );
            println!("  error: {}", f.error_msg);
            // Show 64 bytes around failure
            let ctx_start = f.error_offset.saturating_sub(16);
            println!("  --- 64 bytes around failure offset ---");
            println!("{}", hex_dump_range(data, ctx_start, 80));
            // If this is a bezier_patch (type 0x27=39), dump full raw body so we can analyze
            if f.raw_type == 0x27 {
                let body_start = f.error_offset + 4;
                println!("\n  --- Raw bezier body dump 600 bytes from 0x{body_start:X} ---");
                println!("{}", hex_dump_range(data, body_start, 600));
            }
        }
        return;
    };

    let bezier_start = bezier.start_offset;
    let items_before = bezier.index;
    // current handler end = bezier.end_offset; body_size = bezier.end_offset - bezier_start - 4
    let current_body = bezier.end_offset - bezier_start - 4;

    println!("  bezier_patch item #{items_before}: start=0x{bezier_start:X}, current_body={current_body} bytes");
    println!(
        "  item_count={item_count}, items_parsed={} (gap={})",
        report.items.len(),
        item_count as usize - report.items.len()
    );
    if let Some(f) = &report.failure {
        println!(
            "  failure: offset=0x{:X}, raw_type=0x{:08X}, msg={}",
            f.error_offset, f.raw_type, f.error_msg
        );
    }

    // Hex dump: from bezier body start, dump_bytes ahead
    let body_start = bezier_start + 4;
    println!(
        "\n  --- Hex dump from bezier body start (first {} bytes) ---",
        dump_bytes
    );
    println!("{}", hex_dump_range(data, body_start, dump_bytes));
    println!();

    // Brute-force scan
    println!("  --- Brute-force body-size scan (min items_parsed wins) ---");
    let mut best_size: Option<usize> = None;
    let mut best_parsed = items_before + 1; // at minimum we've parsed up to bezier

    let scan_min = current_body;
    let scan_max = max_bytes.min(data.len().saturating_sub(bezier_start + 4));

    // Scan every 1 byte (fine-grained for small range, coarse for large)
    // For speed: coarse first (step 4), then refine around the winner
    let candidates: Vec<usize> = (scan_min..=scan_max).collect();

    for &body_size in &candidates {
        let parsed =
            try_skip_and_parse(data, &[], items_before, bezier_start, body_size, item_count);
        if parsed > best_parsed || (parsed == best_parsed && best_size.is_none()) {
            best_parsed = parsed;
            best_size = Some(body_size);
        }
        if parsed >= item_count as usize {
            // Perfect — found the exact size
            break;
        }
    }

    match best_size {
        Some(sz) => {
            let delta = sz as isize - current_body as isize;
            println!(
                "  RESULT: best body_size={sz} bytes (current={current_body}, delta=+{delta})"
            );
            println!(
                "          items parsed with best_size: {}/{}",
                best_parsed, item_count
            );
            if best_parsed >= item_count as usize {
                println!("  STATUS: PERFECT — all items parse cleanly with body_size={sz}");
            } else {
                println!(
                    "  STATUS: PARTIAL — best guess only, still {} items unparsed",
                    item_count as usize - best_parsed
                );
            }
            // Hex dump around the correct end
            let correct_end = bezier_start + 4 + sz;
            println!("\n  --- 64 bytes at correct bezier end (offset 0x{correct_end:X}) ---");
            println!(
                "{}",
                hex_dump_range(data, correct_end.saturating_sub(32), 96)
            );
        }
        None => {
            println!("  RESULT: No improvement found in range {scan_min}..={scan_max}");
        }
    }
}

fn main() -> Result<()> {
    let args = Args::parse();
    let mods_dir = args.mods_dir.unwrap_or_else(default_mods_dir);

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

    for sp in &args.sector {
        println!("\n## Sector: `{sp}`\n");
        let data = archives
            .iter_mut()
            .rev()
            .find_map(|arc| arc.read_path(sp).ok());
        let Some(data) = data else {
            println!("  SKIP: not found in archives");
            continue;
        };
        analyze_sector(&data, sp, args.max_bytes, args.dump_bytes);
    }

    eprintln!("Done.");
    Ok(())
}
