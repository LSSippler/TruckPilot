//! `truckpilot-sector-audit` — Phase 5.11 (B1 vs B2) decider.
//!
//! Walks every `.base` sector in `base_map.scs`, calls
//! `audit_sector(data)` on each, and aggregates two histograms:
//!
//! * **Last successful handler** — the `kind_name` of the item right
//!   before the dispatch loop failed. If 80 %+ of failing sectors all
//!   stop at the same handler, we have B1: that handler is desyncing
//!   the cursor and dropping every cross-sector road that comes after.
//! * **Garbage `item_type` after failure** — the u32 the walker read
//!   when it expected the next `item_type` field. Whatever this u32
//!   is points at the body bytes of the *previous* (broken) handler,
//!   which is a strong fingerprint for which handler is wrong.
//!
//! On top of the two 1-D histograms we also print a small heat-map of
//! `(last_kind → garbage_type)` so we can see whether one handler
//! always desyncs into the same garbage offset (true bug) or whether
//! the failure pattern is broad (B2 — bug isn't in our skip handlers,
//! cross-sector data must live elsewhere, e.g. `.aux` files).
//!
//! Usage:
//!
//! ```powershell
//! cargo run --release --bin truckpilot-sector-audit -- `
//!   --ets2-dir "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2"
//! ```

use std::collections::HashMap;
use std::fmt::Write as _;
use std::path::PathBuf;

use truckpilot_map_parser::sector::{audit_sector, AuditReport};
use truckpilot_map_parser::{Archive, HashFsArchive};

#[derive(Debug)]
struct Args {
    ets2_dir: PathBuf,
    output: PathBuf,
}

fn parse_args() -> Args {
    let mut ets2_dir: Option<PathBuf> = None;
    let mut output = PathBuf::from("outputs/sector_audit.txt");

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
            "-h" | "--help" => {
                eprintln!("usage: truckpilot-sector-audit --ets2-dir <PATH> [--output <FILE>]");
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
    eprintln!("probed {} `.base` sector paths", sector_paths.len());

    let mut clean_sectors = 0usize;
    let mut failing_sectors = 0usize;
    let mut unread_sectors = 0usize;

    let mut last_kind_hist: HashMap<&'static str, u64> = HashMap::new();
    let mut garbage_type_hist: HashMap<u32, u64> = HashMap::new();
    let mut joint_hist: HashMap<(&'static str, u32), u64> = HashMap::new();
    let mut item0_failures = 0u64;

    let mut consumed_hist: HashMap<usize, u64> = HashMap::new();

    for path in &sector_paths {
        let Ok(data) = archive.read_path(path) else {
            unread_sectors += 1;
            continue;
        };
        let report: AuditReport = audit_sector(&data);

        let last_kind = report.items.last().map(|it| it.kind_name);
        let garbage_type = report.failure.as_ref().map(|f| f.raw_type);

        if report.failure.is_none() {
            clean_sectors += 1;
        } else {
            failing_sectors += 1;
            match last_kind {
                Some(k) => *last_kind_hist.entry(k).or_default() += 1,
                None => item0_failures += 1,
            }
            if let Some(g) = garbage_type {
                *garbage_type_hist.entry(g).or_default() += 1;
                if let Some(k) = last_kind {
                    *joint_hist.entry((k, g)).or_default() += 1;
                }
            }
        }

        *consumed_hist.entry(report.items.len()).or_default() += 1;
    }

    let total = clean_sectors + failing_sectors;

    // --- Render ---------------------------------------------------------

    let mut out = String::with_capacity(8 * 1024);
    let _ = writeln!(out, "=== SECTOR AUDIT — Phase 5.11 ===");
    let _ = writeln!(out, "archive       : {}", base_map.display());
    let _ = writeln!(out, "sectors total : {}", sector_paths.len());
    let _ = writeln!(out, "  unread      : {}", unread_sectors);
    let _ = writeln!(
        out,
        "  parsed clean: {}  ({:.1} %)",
        clean_sectors,
        pct(clean_sectors as u64, total as u64)
    );
    let _ = writeln!(
        out,
        "  failed      : {}  ({:.1} %)",
        failing_sectors,
        pct(failing_sectors as u64, total as u64)
    );
    if item0_failures > 0 {
        let _ = writeln!(
            out,
            "    of which {} failed at item #0 (no last-kind)",
            item0_failures
        );
    }
    let _ = writeln!(out);

    // 1-D: last successful handler
    let _ = writeln!(out, "── LAST SUCCESSFUL HANDLER (before failure) ──");
    let mut sorted: Vec<(&&'static str, &u64)> = last_kind_hist.iter().collect();
    sorted.sort_by(|a, b| b.1.cmp(a.1));
    let kind_total: u64 = last_kind_hist.values().sum();
    for (kind, count) in &sorted {
        let _ = writeln!(
            out,
            "  {:<28} {:>4}  ({:>5.1} % of failing sectors)",
            kind,
            count,
            pct(**count, failing_sectors as u64)
        );
    }
    if kind_total == 0 {
        let _ = writeln!(out, "  (no failing sectors had a successful predecessor)");
    }
    let _ = writeln!(out);

    // 1-D: garbage type
    let _ = writeln!(out, "── GARBAGE u32 (read where next item_type was expected) ──");
    let mut g_sorted: Vec<(&u32, &u64)> = garbage_type_hist.iter().collect();
    g_sorted.sort_by(|a, b| b.1.cmp(a.1));
    for (g, count) in g_sorted.iter().take(20) {
        let _ = writeln!(
            out,
            "  0x{:08X}   ({:>10})   {:>4}  ({:>5.1} %)",
            *g,
            *g,
            count,
            pct(**count, failing_sectors as u64)
        );
    }
    if garbage_type_hist.is_empty() {
        let _ = writeln!(out, "  (no failing sectors)");
    } else if garbage_type_hist.len() > 20 {
        let _ = writeln!(
            out,
            "  ... ({} more values not shown)",
            garbage_type_hist.len() - 20
        );
    }
    let _ = writeln!(out);

    // 2-D: heat-map of (last_kind, garbage)
    let _ = writeln!(out, "── (LAST_KIND → GARBAGE) — top 20 buckets ──");
    let mut joint_sorted: Vec<(&(&'static str, u32), &u64)> = joint_hist.iter().collect();
    joint_sorted.sort_by(|a, b| b.1.cmp(a.1));
    for ((kind, garbage), count) in joint_sorted.iter().take(20) {
        let _ = writeln!(
            out,
            "  {:<28} → 0x{:08X}  {:>4}",
            kind, garbage, count
        );
    }
    let _ = writeln!(out);

    // Item-count distribution — quick "did we get past item 0/1/2?" stats.
    let mut c_sorted: Vec<(usize, u64)> =
        consumed_hist.iter().map(|(k, v)| (*k, *v)).collect();
    c_sorted.sort_by_key(|(k, _)| *k);
    let _ = writeln!(out, "── ITEMS CONSUMED PER SECTOR (distribution) ──");
    for (k, v) in c_sorted.iter().take(15) {
        let _ = writeln!(
            out,
            "  {:>4} items   {:>4}  ({:>5.1} %)",
            k,
            v,
            pct(*v, total as u64)
        );
    }
    if c_sorted.len() > 15 {
        let _ = writeln!(
            out,
            "  ... {} more buckets up to {} items",
            c_sorted.len() - 15,
            c_sorted.last().map(|(k, _)| *k).unwrap_or(0)
        );
    }
    let _ = writeln!(out);

    // Verdict
    let verdict_threshold = 0.80;
    let top_share = sorted
        .first()
        .map(|(_, c)| **c as f64 / failing_sectors.max(1) as f64)
        .unwrap_or(0.0);
    let _ = writeln!(out, "── VERDICT ──");
    if failing_sectors == 0 {
        let _ = writeln!(
            out,
            "all sectors parsed cleanly — neither B1 nor B2; problem is elsewhere"
        );
    } else if top_share >= verdict_threshold {
        let kind = sorted.first().map(|(k, _)| **k).unwrap_or("<none>");
        let _ = writeln!(
            out,
            "B1 CONFIRMED — {:.1} % of failing sectors stop right after `{}`",
            top_share * 100.0,
            kind
        );
        let _ = writeln!(out, "→ fix the {} skip handler; it desyncs the cursor.", kind);
    } else {
        let _ = writeln!(
            out,
            "B2 INDICATED — top handler `{}` only causes {:.1} % of failures",
            sorted.first().map(|(k, _)| **k).unwrap_or("<none>"),
            top_share * 100.0
        );
        let _ = writeln!(
            out,
            "→ failure is broadly distributed; cross-sector data likely lives outside the items section"
        );
        let _ = writeln!(
            out,
            "  (next: inspect .aux companion files and the trailing vis_uids block)"
        );
    }

    if let Some(parent) = args.output.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    std::fs::write(&args.output, &out)
        .unwrap_or_else(|e| panic!("write {}: {e}", args.output.display()));
    eprintln!("wrote {}", args.output.display());
    print!("{}", out);
}

fn pct(num: u64, denom: u64) -> f64 {
    if denom == 0 {
        0.0
    } else {
        100.0 * num as f64 / denom as f64
    }
}
