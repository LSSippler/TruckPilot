//! `truckpilot-bezier-audit` — Phase 5.20 (BezierPatch-handler) diagnostic.
//!
//! Walks every `.base` sector via [`audit_sector`], picks all sectors where
//! the **last successfully audited item is a BezierPatch** AND the audit
//! failed immediately afterwards. For each such sector dumps:
//!
//! - sector path, item index, body offsets
//! - hex preview (256 bytes from body start) — wide window because bezier
//!   patches are likely variable-length (vertex grid)
//! - first 16 u32 starting at end_offset (each as hex/dec/f32)
//! - plausibility scan over Delta in [-32..+128] in steps of 4
//!
//! Read-only. Output: `outputs/bezier_audit.txt`.

use std::collections::HashSet;
use std::fmt::Write as _;
use std::path::PathBuf;

use truckpilot_map_parser::sector::audit_sector;
use truckpilot_map_parser::{Archive, HashFsArchive};

const VALID_ITEM_TYPES: &[u32] = &[
    1, 2, 3, 4, 5, 6, 7, 8, 12, 18, 19, 22, 34, 35, 36, 37, 38, 39, 41, 42, 43, 44, 46, 48,
];

#[derive(Debug)]
struct Args {
    ets2_dir: PathBuf,
    output: PathBuf,
    sample_count: usize,
}

fn parse_args() -> Args {
    let mut ets2_dir: Option<PathBuf> = None;
    let mut output = PathBuf::from("outputs/bezier_audit.txt");
    let mut sample_count = 12usize;

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
                    "usage: truckpilot-bezier-audit --ets2-dir <PATH> [--output <FILE>] [--samples N]"
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

fn read_u32_le(data: &[u8], pos: usize) -> Option<u32> {
    if pos + 4 > data.len() {
        return None;
    }
    Some(u32::from_le_bytes(data[pos..pos + 4].try_into().unwrap()))
}

fn read_f32_le(data: &[u8], pos: usize) -> Option<f32> {
    read_u32_le(data, pos).map(f32::from_bits)
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
    let valid: HashSet<u32> = VALID_ITEM_TYPES.iter().copied().collect();

    let probe_deltas: Vec<i32> = (-32..=128).step_by(4).collect();

    eprintln!("opening {} …", base_map.display());
    let mut archive = HashFsArchive::open(&base_map).expect("open base_map.scs");
    let mut sector_paths: Vec<String> = archive
        .probe_sector_paths()
        .into_iter()
        .filter(|p| p.ends_with(".base"))
        .collect();
    sector_paths.sort();
    eprintln!("probed {} `.base` paths", sector_paths.len());

    type Sample = (String, Vec<u8>, truckpilot_map_parser::sector::AuditReport);
    let mut samples: Vec<Sample> = Vec::new();
    let mut total_bezier_last = 0usize;
    let mut total_failures = 0usize;
    let mut failure_raw_type: std::collections::HashMap<u32, usize> =
        std::collections::HashMap::new();
    let mut last_success_kind: std::collections::HashMap<&'static str, usize> =
        std::collections::HashMap::new();
    for path in &sector_paths {
        let Ok(data) = archive.read_path(path) else {
            continue;
        };
        let report = audit_sector(&data);
        let Some(failure) = &report.failure else {
            continue;
        };
        total_failures += 1;
        *failure_raw_type.entry(failure.raw_type).or_insert(0) += 1;
        if let Some(last) = report.items.last() {
            *last_success_kind.entry(last.kind_name).or_insert(0) += 1;
            if last.kind_name == "bezier_patch" || failure.raw_type == 39 {
                total_bezier_last += 1;
                if samples.len() < args.sample_count {
                    samples.push((path.clone(), data, report));
                }
            }
        }
    }
    eprintln!(
        "found {} sectors with bezier_patch-as-last-successful, dumping first {}",
        total_bezier_last,
        samples.len()
    );

    let mut delta_hits: std::collections::HashMap<i32, usize> = std::collections::HashMap::new();
    let mut min_delta_per_sample: Vec<Option<i32>> = Vec::with_capacity(samples.len());
    let mut tail_advance_dist: std::collections::HashMap<usize, usize> =
        std::collections::HashMap::new();

    let mut out = String::new();
    let _ = writeln!(out, "# Phase 5.20 — BezierPatch-handler audit");
    let _ = writeln!(out);
    let _ = writeln!(out, "Archive:       {}", base_map.display());
    let _ = writeln!(out, "Total sectors: {}", sector_paths.len());
    let _ = writeln!(
        out,
        "Sectors with audit-failure AND last-success=bezier_patch: {total_bezier_last}"
    );
    let _ = writeln!(out, "Samples dumped: {}", samples.len());
    let _ = writeln!(
        out,
        "Probe Delta range: {:?}..={:?} step 4",
        probe_deltas.first(),
        probe_deltas.last()
    );
    let _ = writeln!(out, "Valid item_type set: {:?}", VALID_ITEM_TYPES);
    let _ = writeln!(out);

    for (path, data, report) in &samples {
        let last = report.items.last().expect("checked above");
        let failure = report.failure.as_ref().expect("checked above");
        let advance = last.end_offset.saturating_sub(last.start_offset);
        let body_start = last.start_offset + 4;
        let body_len = last.end_offset.saturating_sub(body_start);

        let _ = writeln!(
            out,
            "------------------------------------------------------------"
        );
        let _ = writeln!(out, "## {path}");
        let _ = writeln!(out, "    sector size              : {} bytes", data.len());
        let _ = writeln!(out, "    item_count               : {}", report.item_count);
        let _ = writeln!(
            out,
            "    items consumed before fail: {}",
            report.items.len()
        );
        let _ = writeln!(out, "    bezier item index        : {}", last.index);
        let _ = writeln!(out, "    bezier start_offset      : {}", last.start_offset);
        let _ = writeln!(out, "    bezier end_offset        : {}", last.end_offset);
        let _ = writeln!(out, "    bezier body bytes        : {body_len}");
        let _ = writeln!(out, "    bytes advance (incl tag) : {advance}");
        let _ = writeln!(out, "    failure item_index       : {}", failure.item_index);
        let _ = writeln!(
            out,
            "    failure raw_type         : 0x{:08x} ({})",
            failure.raw_type, failure.raw_type
        );
        let _ = writeln!(
            out,
            "    failure error_offset     : {}",
            failure.error_offset
        );
        let _ = writeln!(out, "    failure msg              : {}", failure.error_msg);
        let _ = writeln!(out);

        let _ = writeln!(out, "    hex preview — 256 bytes from bezier body:");
        out.push_str(&hex_block(data, body_start, 256));
        let _ = writeln!(out);

        let _ = writeln!(
            out,
            "    first 16 u32 starting at end_offset (hex / dec / f32):"
        );
        for k in 0..16 {
            let pos = last.end_offset + k * 4;
            let Some(u) = read_u32_le(data, pos) else {
                break;
            };
            let f = read_f32_le(data, pos).unwrap_or(0.0);
            let valid_marker = if valid.contains(&u) {
                "  <-- VALID type"
            } else {
                ""
            };
            let _ = writeln!(
                out,
                "        +{:3}  pos {pos:7}  hex=0x{u:08x}  dec={u:>10}  f32={f:>+13.4}{valid_marker}",
                k * 4
            );
        }
        let _ = writeln!(out);

        let _ = writeln!(
            out,
            "    plausibility scan (Delta where u32 is a valid item_type):"
        );
        let mut hits_this_sample: Vec<i32> = Vec::new();
        for &delta in &probe_deltas {
            let probe_pos_i64 = last.end_offset as i64 + delta as i64;
            if probe_pos_i64 < 0 {
                continue;
            }
            let probe_pos = probe_pos_i64 as usize;
            if let Some(v) = read_u32_le(data, probe_pos) {
                if valid.contains(&v) {
                    let _ = writeln!(
                        out,
                        "      Delta = {delta:+5}  pos {probe_pos:7}  u32 = {v:>3}    VALID"
                    );
                    *delta_hits.entry(delta).or_insert(0) += 1;
                    hits_this_sample.push(delta);
                }
            }
        }
        let smallest_nonneg = hits_this_sample.iter().copied().filter(|d| *d >= 0).min();
        min_delta_per_sample.push(smallest_nonneg);
        if let Some(d) = smallest_nonneg {
            *tail_advance_dist.entry(d as usize).or_insert(0) += 1;
            let _ = writeln!(
                out,
                "    smallest non-negative Delta with valid item_type: {d:+}"
            );
        } else {
            let _ = writeln!(
                out,
                "    no valid item_type in scanned window — drift > 128 bytes"
            );
        }
        let _ = writeln!(out);
    }

    let _ = writeln!(
        out,
        "============================================================"
    );
    let _ = writeln!(
        out,
        "## Aggregate Delta-distribution ({} samples)",
        samples.len()
    );
    let mut deltas: Vec<(i32, usize)> = delta_hits.into_iter().collect();
    deltas.sort_by_key(|(d, _)| *d);
    for (d, n) in &deltas {
        let pct = if !samples.is_empty() {
            100.0 * (*n as f64) / (samples.len() as f64)
        } else {
            0.0
        };
        let _ = writeln!(out, "    Delta = {d:+5}  ->  {n:3} samples ({pct:.1}%)");
    }
    let _ = writeln!(out);

    let mut min_delta_dist: std::collections::HashMap<Option<i32>, usize> =
        std::collections::HashMap::new();
    for d in &min_delta_per_sample {
        *min_delta_dist.entry(*d).or_insert(0) += 1;
    }
    let _ = writeln!(out, "## Smallest non-negative valid Delta per sample:");
    let mut min_keys: Vec<(Option<i32>, usize)> = min_delta_dist.into_iter().collect();
    min_keys.sort_by_key(|a| a.0);
    for (d, n) in &min_keys {
        match d {
            Some(d) => {
                let _ = writeln!(out, "    Delta = {d:+5}  ->  {n:3} samples");
            }
            None => {
                let _ = writeln!(out, "    Delta = (none) ->  {n:3} samples");
            }
        }
    }
    let _ = writeln!(out);

    let _ = writeln!(
        out,
        "## Tail-advance distribution (extra bytes the handler should consume):"
    );
    let mut tail_keys: Vec<(usize, usize)> = tail_advance_dist.into_iter().collect();
    tail_keys.sort_by_key(|a| a.0);
    for (b, n) in &tail_keys {
        let _ = writeln!(out, "    +{b:>4} bytes ->  {n:3} samples");
    }
    let _ = writeln!(out);

    let _ = writeln!(
        out,
        "============================================================"
    );
    let _ = writeln!(
        out,
        "## Global failing-handler distribution (all {total_failures} failing sectors)"
    );
    let _ = writeln!(out);
    let mut by_type: Vec<(u32, usize)> = failure_raw_type.into_iter().collect();
    by_type.sort_by_key(|b| std::cmp::Reverse(b.1));
    let total_fail = total_failures.max(1);
    let _ = writeln!(out, "### Failure raw_type (culprit OR garbage)");
    for (t, n) in &by_type {
        let pct = 100.0 * (*n as f64) / (total_fail as f64);
        let _ = writeln!(out, "    raw_type 0x{t:08x}  {n:>3}  ({pct:5.1}%)");
    }
    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "### Last SUCCESSFUL handler before failure (predecessor)"
    );
    let mut by_pred: Vec<(&str, usize)> = last_success_kind.iter().map(|(k, v)| (*k, *v)).collect();
    by_pred.sort_by_key(|b| std::cmp::Reverse(b.1));
    for (k, n) in &by_pred {
        let pct = 100.0 * (*n as f64) / (total_fail as f64);
        let _ = writeln!(out, "    {k:<16}        {n:>3}  ({pct:5.1}%)");
    }

    if let Some(parent) = args.output.parent() {
        std::fs::create_dir_all(parent).expect("create output dir");
    }
    std::fs::write(&args.output, &out).expect("write output");
    eprintln!("→ {} ({} bytes)", args.output.display(), out.len());
}
