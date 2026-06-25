//! bezier_len_scan — forward-scan each failing bezier body from body+261
//! looking for valid item_type u32 values. The first hit past 261 bytes is
//! the candidate body_len.

use std::collections::HashSet;
use std::fmt::Write as _;
use std::path::PathBuf;

use truckpilot_map_parser::sector::audit_sector;
use truckpilot_map_parser::{Archive, HashFsArchive};

const VALID_ITEM_TYPES: &[u32] = &[
    1, 2, 3, 4, 5, 6, 7, 8, 12, 18, 19, 22, 34, 35, 36, 37, 38, 39, 41, 42, 43, 44, 46, 48,
];
// Scan window: from body+261 to body+261+SCAN_LEN in 4-byte steps
const SCAN_START: usize = 261;
const SCAN_LEN: usize = 2048;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut ets2_dir: Option<PathBuf> = None;
    let mut output = PathBuf::from("outputs/bezier_len_scan.txt");
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--ets2-dir" => { ets2_dir = Some(PathBuf::from(&args[i + 1])); i += 2; }
            "--output"   => { output   = PathBuf::from(&args[i + 1]); i += 2; }
            _ => { i += 1; }
        }
    }
    let ets2_dir = ets2_dir.unwrap_or_else(|| { eprintln!("need --ets2-dir"); std::process::exit(1); });

    let valid: HashSet<u32> = VALID_ITEM_TYPES.iter().copied().collect();
    let base_map = ets2_dir.join("base_map.scs");
    let mut archive = HashFsArchive::open(&base_map).expect("open base_map.scs");

    let mut sector_paths: Vec<String> = archive
        .probe_sector_paths()
        .into_iter()
        .filter(|p| p.ends_with(".base"))
        .collect();
    sector_paths.sort();
    eprintln!("probed {} .base paths", sector_paths.len());

    let mut out = String::new();
    let _ = writeln!(out, "# bezier_len_scan — forward scan for next valid item_type");
    let _ = writeln!(out);

    let mut found = 0usize;

    for path in &sector_paths {
        let Ok(data) = archive.read_path(path) else { continue; };
        let report = audit_sector(&data);
        let Some(failure) = &report.failure else { continue; };
        // Only process bezier failures
        if failure.raw_type != 39 {
            if let Some(last) = report.items.last() {
                if last.kind_name != "bezier_patch" { continue; }
            } else {
                continue;
            }
        }

        let bezier_body_start = failure.error_offset + 4; // past the failing item_type u32
        if bezier_body_start + SCAN_START >= data.len() { continue; }

        let _ = writeln!(out, "------------------------------------------------------------");
        let _ = writeln!(out, "## {path}");
        let _ = writeln!(out, "    body_start={bezier_body_start}  failure={}", failure.error_msg);

        // Dump 48 bytes at body+261 (= BEZIER_DIAG zone, after veg+sphere_count)
        let diag_pos = bezier_body_start + SCAN_START;
        let diag_end = (diag_pos + 96).min(data.len());
        let _ = write!(out, "    body+261 hex: ");
        for b in &data[diag_pos..diag_end] {
            let _ = write!(out, "{b:02x} ");
        }
        let _ = writeln!(out);

        // Forward scan
        let scan_end = (bezier_body_start + SCAN_START + SCAN_LEN).min(data.len().saturating_sub(4));
        let _ = writeln!(out, "    scan from body+{SCAN_START} to body+{}:", SCAN_START + SCAN_LEN);
        let mut hits: Vec<(usize, u32)> = Vec::new();
        let mut pos = bezier_body_start + SCAN_START;
        while pos + 4 <= scan_end {
            let u = u32::from_le_bytes(data[pos..pos + 4].try_into().unwrap());
            if valid.contains(&u) {
                let body_len = pos - bezier_body_start;
                hits.push((body_len, u));
            }
            pos += 4;
        }
        if hits.is_empty() {
            let _ = writeln!(out, "      (no valid item_type found in scan window)");
        } else {
            for (body_len, item_type) in &hits {
                let _ = writeln!(out, "      body_len={body_len:<6} → item_type={item_type} (next={:?})",
                    hits.iter().find(|(l,_)| *l > *body_len).map(|(l,t)| (*l,*t)));
            }
        }
        let _ = writeln!(out);
        found += 1;
    }

    let _ = writeln!(out, "============================================================");
    let _ = writeln!(out, "Total failing bezier sectors scanned: {found}");

    if let Some(p) = output.parent() { let _ = std::fs::create_dir_all(p); }
    std::fs::write(&output, &out).expect("write");
    eprintln!("→ {}", output.display());
}
