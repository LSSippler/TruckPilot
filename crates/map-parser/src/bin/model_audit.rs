//! `truckpilot-model-audit` — Phase 5.18 (Model-handler) diagnostic.
//!
//! Walks every `.base` sector via [`audit_sector`], picks the first N sectors
//! where the **last successfully audited item is a Model** AND the audit
//! failed immediately afterwards (i.e. `skip_model` returned but consumed too
//! few bytes, leaving the cursor mid-body so the next item-type read sees
//! garbage). For each such sector it dumps:
//!
//! * sector path, item count, model item-index, model `start_offset` (where
//!   the `item_type u32 = 5` was read), model `end_offset` (where
//!   `skip_model` claimed the body ended).
//! * bytes-advance = `end_offset - start_offset` (should be `4 + body_len`;
//!   the current `skip_model` consumes 4 + 86 = 90 bytes per item).
//! * hex preview of 128 bytes starting at `start_offset + 4` (= model body).
//! * plausibility scan: at offsets `end_offset + Δ` for Δ in a small probe
//!   set, read a u32 and check whether it is a valid `item_type` (1..=48
//!   set). The smallest non-negative Δ at which a valid type appears is the
//!   most likely off-by-N for the missing tail of `skip_model`.
//!
//! Read-only. Output: `outputs/model_audit.txt`.
//!
//! Usage:
//!
//! ```powershell
//! cargo run --release --bin truckpilot-model-audit -- `
//!   --ets2-dir "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2"
//! ```

use std::collections::HashSet;
use std::fmt::Write as _;
use std::path::PathBuf;

use truckpilot_map_parser::sector::audit_sector;
use truckpilot_map_parser::{Archive, HashFsArchive};

/// Set of valid `item_type` values used in v907 sectors. Anything outside is
/// treated as garbage by the plausibility scan.
const VALID_ITEM_TYPES: &[u32] = &[
    1, 2, 3, 4, 5, 6, 7, 8, 12, 18, 19, 22, 34, 35, 36, 37, 38, 39, 41, 42, 43, 44, 46, 48,
];

/// Δ probe offsets (signed). Negative means earlier-than-claimed end, positive
/// means past-the-claimed end.
const PROBE_DELTAS: &[i32] = &[-16, -12, -8, -4, 0, 4, 8, 12, 16, 20, 24, 32];

#[derive(Debug)]
struct Args {
    ets2_dir: PathBuf,
    output: PathBuf,
    sample_count: usize,
}

fn parse_args() -> Args {
    let mut ets2_dir: Option<PathBuf> = None;
    let mut output = PathBuf::from("outputs/model_audit.txt");
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
                    "usage: truckpilot-model-audit --ets2-dir <PATH> [--output <FILE>] [--samples N]"
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
    let mut total_model_last = 0usize;
    // Global failure distribution across ALL failing sectors — gives the true
    // by-handler prioritisation (the model-as-last subset is biased by
    // model-item frequency in the item stream).
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
            if last.kind_name == "model" {
                total_model_last += 1;
                if samples.len() < args.sample_count {
                    samples.push((path.clone(), data, report));
                }
            }
        }
    }
    eprintln!(
        "found {} sectors with model-as-last-successful, dumping first {}",
        total_model_last,
        samples.len()
    );

    let mut delta_hits: std::collections::HashMap<i32, usize> = std::collections::HashMap::new();
    let mut min_delta_per_sample: Vec<Option<i32>> = Vec::with_capacity(samples.len());

    let mut out = String::new();
    let _ = writeln!(out, "# Phase 5.18 — Model-handler audit");
    let _ = writeln!(out);
    let _ = writeln!(out, "Archive:       {}", base_map.display());
    let _ = writeln!(out, "Total sectors: {}", sector_paths.len());
    let _ = writeln!(
        out,
        "Sectors with audit-failure AND last-success=model: {total_model_last}"
    );
    let _ = writeln!(out, "Samples dumped: {}", samples.len());
    let _ = writeln!(
        out,
        "Probe Δ offsets (relative to handler-claimed end): {:?}",
        PROBE_DELTAS
    );
    let _ = writeln!(out, "Valid item_type set: {:?}", VALID_ITEM_TYPES);
    let _ = writeln!(out);

    for (path, data, report) in &samples {
        let last = report.items.last().expect("checked above");
        let failure = report.failure.as_ref().expect("checked above");
        let advance = last.end_offset.saturating_sub(last.start_offset);
        let model_body = last.start_offset + 4;

        let _ = writeln!(
            out,
            "------------------------------------------------------------"
        );
        let _ = writeln!(out, "## {path}");
        let _ = writeln!(out, "    sector size           : {} bytes", data.len());
        let _ = writeln!(out, "    item_count            : {}", report.item_count);
        let _ = writeln!(
            out,
            "    items consumed before failure: {}",
            report.items.len()
        );
        let _ = writeln!(out, "    model item index      : {}", last.index);
        let _ = writeln!(out, "    model start_offset    : {}", last.start_offset);
        let _ = writeln!(out, "    model end_offset      : {}", last.end_offset);
        let _ = writeln!(out, "    bytes advance         : {advance}");
        let _ = writeln!(out, "    failure item_index    : {}", failure.item_index);
        let _ = writeln!(
            out,
            "    failure raw_type      : 0x{:08x} ({})",
            failure.raw_type, failure.raw_type
        );
        let _ = writeln!(out, "    failure error_offset  : {}", failure.error_offset);
        let _ = writeln!(out, "    failure msg           : {}", failure.error_msg);
        let _ = writeln!(out);

        let _ = writeln!(
            out,
            "    hex preview — 128 bytes from model body (skip type-tag):"
        );
        out.push_str(&hex_block(data, model_body, 128));
        let _ = writeln!(out);

        let _ = writeln!(
            out,
            "    plausibility scan around handler-claimed end ({}):",
            last.end_offset
        );
        let mut hits_this_sample: Vec<i32> = Vec::new();
        for &delta in PROBE_DELTAS {
            let probe_pos_i64 = last.end_offset as i64 + delta as i64;
            if probe_pos_i64 < 0 {
                let _ = writeln!(out, "      Δ = {delta:+5}  → out of range (before sector)");
                continue;
            }
            let probe_pos = probe_pos_i64 as usize;
            match read_u32_le(data, probe_pos) {
                Some(v) if valid.contains(&v) => {
                    let _ = writeln!(
                        out,
                        "      Δ = {delta:+5}  pos {probe_pos:7}  u32 = {v:>3}    VALID item_type"
                    );
                    *delta_hits.entry(delta).or_insert(0) += 1;
                    hits_this_sample.push(delta);
                }
                Some(v) => {
                    let _ = writeln!(
                        out,
                        "      Δ = {delta:+5}  pos {probe_pos:7}  u32 = 0x{v:08x} ({v})"
                    );
                }
                None => {
                    let _ = writeln!(out, "      Δ = {delta:+5}  pos {probe_pos:7}  out of range");
                }
            }
        }
        let smallest_nonneg = hits_this_sample.iter().copied().filter(|d| *d >= 0).min();
        min_delta_per_sample.push(smallest_nonneg);
        if let Some(d) = smallest_nonneg {
            let _ = writeln!(
                out,
                "    smallest non-negative Δ with valid item_type: {d:+}"
            );
        } else {
            let _ = writeln!(
                out,
                "    no valid item_type in scanned window — likely larger off-by-N or variable suffix"
            );
        }
        let _ = writeln!(out);
    }

    // ----- Aggregate -----
    let _ = writeln!(
        out,
        "============================================================"
    );
    let _ = writeln!(
        out,
        "## Aggregate Δ-distribution ({} samples)",
        samples.len()
    );
    let _ = writeln!(
        out,
        "Number of samples whose VALID item_type appears at each Δ:"
    );
    let mut deltas: Vec<(i32, usize)> = delta_hits.into_iter().collect();
    deltas.sort_by_key(|(d, _)| *d);
    for (d, n) in &deltas {
        let pct = if !samples.is_empty() {
            100.0 * (*n as f64) / (samples.len() as f64)
        } else {
            0.0
        };
        let _ = writeln!(out, "    Δ = {d:+5}  →  {n:3} samples ({pct:.1}%)");
    }
    let _ = writeln!(out);

    let mut min_delta_dist: std::collections::HashMap<Option<i32>, usize> =
        std::collections::HashMap::new();
    for d in &min_delta_per_sample {
        *min_delta_dist.entry(*d).or_insert(0) += 1;
    }
    let _ = writeln!(
        out,
        "## Smallest non-negative valid Δ per sample (the off-by-N candidate):"
    );
    let mut min_keys: Vec<(Option<i32>, usize)> = min_delta_dist.into_iter().collect();
    min_keys.sort_by_key(|a| a.0);
    for (d, n) in &min_keys {
        match d {
            Some(d) => {
                let _ = writeln!(out, "    Δ = {d:+5}  →  {n:3} samples");
            }
            None => {
                let _ = writeln!(out, "    Δ = (none) →  {n:3} samples");
            }
        }
    }
    let _ = writeln!(out);

    let total = samples.len().max(1);
    let dominant = min_keys
        .iter()
        .filter_map(|(d, n)| d.map(|dd| (dd, *n)))
        .max_by_key(|(_, n)| *n);

    // ----- Global failing-handler distribution -----
    let _ = writeln!(
        out,
        "============================================================"
    );
    let _ = writeln!(
        out,
        "## Global failing-handler distribution (all {total_failures} failing sectors)"
    );
    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "### Failure happened INSIDE handler (failure.raw_type — the actual culprit)"
    );
    let mut by_type: Vec<(u32, usize)> = failure_raw_type.into_iter().collect();
    by_type.sort_by_key(|b| std::cmp::Reverse(b.1));
    let total_fail = total_failures.max(1);
    for (t, n) in &by_type {
        let pct = 100.0 * (*n as f64) / (total_fail as f64);
        let name = match *t {
            1 => "terrain",
            2 => "buildings",
            3 => "road",
            4 => "prefab",
            5 => "model",
            6 => "company",
            7 => "service",
            8 => "cut_plane",
            12 => "city",
            18 => "map_overlay",
            19 => "ferry",
            22 => "garage",
            34 => "trigger",
            35 => "fuel_pump",
            36 => "sign",
            37 => "bus_stop",
            38 => "traffic_area",
            39 => "bezier_patch",
            41 => "trajectory",
            42 => "map_area",
            43 => "far_model",
            44 => "curve",
            46 => "cutscene",
            48 => "visibility_area",
            _ => "?",
        };
        let _ = writeln!(out, "    {name:<16} ({t:>3})  {n:>3}  ({pct:5.1}%)");
    }
    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "### Last SUCCESSFUL handler before failure (predecessor — biased by item frequency)"
    );
    let mut by_pred: Vec<(&str, usize)> = last_success_kind.iter().map(|(k, v)| (*k, *v)).collect();
    by_pred.sort_by_key(|b| std::cmp::Reverse(b.1));
    for (k, n) in &by_pred {
        let pct = 100.0 * (*n as f64) / (total_fail as f64);
        let _ = writeln!(out, "    {k:<16}        {n:>3}  ({pct:5.1}%)");
    }
    let _ = writeln!(out);
    let _ = writeln!(out, "## Verdict");
    match dominant {
        Some((d, n)) if (n as f64) / (total as f64) >= 0.8 => {
            let _ = writeln!(
                out,
                "→ CONSTANT OFF-BY-N likely: Δ = {:+} matches {}/{} samples ({:.1}%).",
                d,
                n,
                total,
                100.0 * (n as f64) / (total as f64)
            );
            let _ = writeln!(
                out,
                "→ Fix candidate: extend model handler to consume {d} extra bytes after the current 86-byte body."
            );
        }
        Some((d, n)) if n > 0 => {
            let _ = writeln!(
                out,
                "→ PARTIAL: Δ = {:+} dominates ({}/{} samples, {:.1}%) but not unanimous.",
                d,
                n,
                total,
                100.0 * (n as f64) / (total as f64)
            );
            let _ = writeln!(
                out,
                "→ Likely flag-conditional or count-driven suffix block. Inspect divergent-sample hex for the discriminator."
            );
        }
        _ => {
            let _ = writeln!(
                out,
                "→ NO CLEAR Δ — variable-length suffix in window > 32 bytes, or per-model dependent."
            );
            let _ = writeln!(
                out,
                "→ Re-run with wider Δ probe range, or extend skip_model field-by-field per TruckLib ModelSerializer and re-audit."
            );
        }
    }

    if let Some(parent) = args.output.parent() {
        std::fs::create_dir_all(parent).expect("create output dir");
    }
    std::fs::write(&args.output, &out).expect("write output");
    eprintln!("→ {} ({} bytes)", args.output.display(), out.len());
}
