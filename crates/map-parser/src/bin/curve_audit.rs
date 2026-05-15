//! `truckpilot-curve-audit` — Phase 5.17 (Curve-handler) diagnostic.
//!
//! Walks every `.base` sector via [`audit_sector`], picks the first N sectors
//! where the **failure happened INSIDE skip_curve** (i.e. `failure.raw_type
//! == 44`). For each such sector it dumps:
//!
//! * sector path, item count, failing item-index, error offset (where the
//!   `item_type u32 = 44` was read), error message (which read inside
//!   skip_curve overflowed).
//! * hex preview of 192 bytes starting at `failure.error_offset + 4` (=
//!   curve body, after the type tag), so we can compare against the
//!   TruckLib `CurveSerializer` field layout.
//! * highlight bytes at `body + 53` (immediately after the 53-byte
//!   `kdop_item`) — discriminates "u64+u64 like terrain" vs "token-like
//!   fields like TruckLib Curve".
//!
//! Read-only. Output: `outputs/curve_audit.txt`.

use std::collections::HashMap;
use std::fmt::Write as _;
use std::path::PathBuf;

use truckpilot_map_parser::sector::audit_sector;
use truckpilot_map_parser::{Archive, HashFsArchive};

/// Type 44 = ITEM_TYPE_CURVE.
const CURVE_TYPE: u32 = 44;
const KDOP_ITEM_LEN: usize = 53;

#[derive(Debug)]
struct Args {
    ets2_dir: PathBuf,
    output: PathBuf,
    sample_count: usize,
}

fn parse_args() -> Args {
    let mut ets2_dir: Option<PathBuf> = None;
    let mut output = PathBuf::from("outputs/curve_audit.txt");
    let mut sample_count = 20usize;

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
                    "usage: truckpilot-curve-audit --ets2-dir <PATH> [--output <FILE>] [--samples N]"
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

fn hex_block(data: &[u8], start: usize, len: usize) -> String {
    let end = (start + len).min(data.len());
    let mut out = String::with_capacity(len * 4);
    let mut i = start;
    while i < end {
        let _ = write!(out, "      {i:08x}: ");
        let row_end = (i + 16).min(end);
        let row = &data[i..row_end];
        for (k, b) in row.iter().enumerate() {
            let _ = write!(out, "{b:02x}");
            if k == 7 {
                let _ = write!(out, "  ");
            } else if k < row.len() - 1 {
                let _ = write!(out, " ");
            }
        }
        let pad = 16 - row.len();
        for _ in 0..pad {
            let _ = write!(out, "   ");
        }
        let _ = write!(out, "  | ");
        for &b in row {
            let c = if (0x20..=0x7e).contains(&b) {
                b as char
            } else {
                '.'
            };
            out.push(c);
        }
        out.push('\n');
        i = row_end;
    }
    out
}

fn classify_field(bytes: &[u8]) -> &'static str {
    if bytes.len() < 8 {
        return "too-short";
    }
    let v = u64::from_le_bytes(bytes[0..8].try_into().unwrap());
    if v == 0 {
        return "zero-token";
    }
    let printable = bytes[0..8]
        .iter()
        .filter(|&&b| (0x20..=0x7e).contains(&b))
        .count();
    let zeros = bytes[0..8].iter().filter(|&&b| b == 0).count();
    if printable >= 4 && zeros >= 2 {
        "token-like"
    } else if zeros == 0 {
        "uid-like (high-entropy)"
    } else {
        "mixed"
    }
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
    eprintln!("probed {} `.base` paths", sector_paths.len());

    type Sample = (
        String,
        Vec<u8>,
        truckpilot_map_parser::sector::AuditReport,
    );
    let mut samples: Vec<Sample> = Vec::new();
    let mut total_curve_failures = 0usize;
    let mut error_msg_hist: HashMap<String, usize> = HashMap::new();

    for path in &sector_paths {
        let Ok(data) = archive.read_path(path) else {
            continue;
        };
        let report = audit_sector(&data);
        let Some(failure) = &report.failure else {
            continue;
        };
        if failure.raw_type != CURVE_TYPE {
            continue;
        }
        total_curve_failures += 1;
        let bucket = failure
            .error_msg
            .chars()
            .take(60)
            .collect::<String>();
        *error_msg_hist.entry(bucket).or_insert(0) += 1;

        if samples.len() < args.sample_count {
            samples.push((path.clone(), data, report));
        }
    }
    eprintln!(
        "found {} sectors where skip_curve crashed; dumping first {}",
        total_curve_failures,
        samples.len()
    );

    let mut out = String::new();
    let _ = writeln!(out, "# Phase 5.17 — Curve-handler audit");
    let _ = writeln!(out);
    let _ = writeln!(out, "Archive:                 {}", base_map.display());
    let _ = writeln!(out, "Total sectors:           {}", sector_paths.len());
    let _ = writeln!(
        out,
        "Sectors failing in skip_curve (raw_type=44): {total_curve_failures}"
    );
    let _ = writeln!(out, "Samples dumped:          {}", samples.len());
    let _ = writeln!(out);

    let _ = writeln!(out, "## Error-message histogram (first 60 chars)");
    let mut by_msg: Vec<(String, usize)> = error_msg_hist.into_iter().collect();
    by_msg.sort_by_key(|b| std::cmp::Reverse(b.1));
    for (m, n) in &by_msg {
        let pct = 100.0 * (*n as f64) / (total_curve_failures.max(1) as f64);
        let _ = writeln!(out, "    {n:>3}  ({pct:5.1}%)  {m}");
    }
    let _ = writeln!(out);

    let _ = writeln!(
        out,
        "## Field classification at body+{KDOP_ITEM_LEN} (post-kdop_item, first 8B)"
    );
    let _ = writeln!(out, "    (terrain has u64 node-uid here = uid-like; TruckLib curve has token-like)");
    let mut class_hist: HashMap<&'static str, usize> = HashMap::new();
    for (_, data, report) in &samples {
        let Some(failure) = report.failure.as_ref() else { continue; };
        let body = failure.error_offset + 4;
        let post_kdop = body + KDOP_ITEM_LEN;
        if post_kdop + 8 > data.len() {
            *class_hist.entry("oob").or_insert(0) += 1;
            continue;
        }
        let cls = classify_field(&data[post_kdop..post_kdop + 8]);
        *class_hist.entry(cls).or_insert(0) += 1;
    }
    let mut class_sorted: Vec<(&&str, &usize)> = class_hist.iter().collect();
    class_sorted.sort_by_key(|b| std::cmp::Reverse(*b.1));
    for (k, n) in &class_sorted {
        let _ = writeln!(out, "    {n:>3}  {k}");
    }
    let _ = writeln!(out);

    for (path, data, report) in &samples {
        let failure = report.failure.as_ref().expect("checked above");
        let body = failure.error_offset + 4;
        let post_kdop = body + KDOP_ITEM_LEN;

        let _ = writeln!(out, "------------------------------------------------------------");
        let _ = writeln!(out, "## {path}");
        let _ = writeln!(out, "    sector size           : {} bytes", data.len());
        let _ = writeln!(out, "    item_count            : {}", report.item_count);
        let _ = writeln!(
            out,
            "    items consumed before failure: {}",
            report.items.len()
        );
        if let Some(last) = report.items.last() {
            let _ = writeln!(
                out,
                "    last successful kind  : {} (idx {})",
                last.kind_name, last.index
            );
        }
        let _ = writeln!(out, "    failure item_index    : {}", failure.item_index);
        let _ = writeln!(
            out,
            "    failure raw_type      : 0x{:08x} ({})",
            failure.raw_type, failure.raw_type
        );
        let _ = writeln!(out, "    failure error_offset  : {}", failure.error_offset);
        let _ = writeln!(out, "    body offset           : {body}");
        let _ = writeln!(out, "    body+kdop ({KDOP_ITEM_LEN}) offset: {post_kdop}");
        let _ = writeln!(out, "    failure msg           : {}", failure.error_msg);
        if post_kdop + 8 <= data.len() {
            let cls = classify_field(&data[post_kdop..post_kdop + 8]);
            let v = u64::from_le_bytes(data[post_kdop..post_kdop + 8].try_into().unwrap());
            let _ = writeln!(out, "    body+53 classifier    : {cls}  (raw u64 = 0x{v:016x})");
        }
        let _ = writeln!(out);

        let _ = writeln!(out, "    hex preview — 192 bytes from curve body (skip type-tag):");
        out.push_str(&hex_block(data, body, 192));
        let _ = writeln!(out);
    }

    if let Some(parent) = args.output.parent() {
        std::fs::create_dir_all(parent).expect("create output dir");
    }
    std::fs::write(&args.output, &out).expect("write output");
    eprintln!("→ {} ({} bytes)", args.output.display(), out.len());
}
