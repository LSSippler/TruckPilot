//! `bothunresolved-forensik` — Phase 6.2b-Diag-5
//!
//! Read-only forensik tool. For each of the ~5940 BothUnresolved roads in
//! `road_drop_audit.json`, determines whether the node_a / node_b UIDs
//! exist anywhere in the raw .base sector data. Outputs bucket
//! distribution (BothMissing / OnlyAFound / OnlyBFound / BothFound) and a
//! root-cause hypothesis (COVERAGE / MERGE / CROSS_SECTOR_MERGE / etc.).
//!
//! Gate Guards:
//!   - Read-only. No parser / merge / graph changes.
//!   - Scan must stay under 60s for default run.

use std::collections::HashSet;
use std::io::Write as IoWrite;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Instant;

use anyhow::{Context, Result};
use clap::Parser;
use rustc_hash::{FxHashMap, FxHashSet};
use serde::Deserialize;
use serde_json::json;
use truckpilot_map_parser::{Archive, HashFsArchive, ModLoadOrder, ZipArchive};

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

#[derive(Parser)]
#[command(
    name = "bothunresolved-forensik",
    about = "Phase 6.2b-Diag-5: forensic analysis of BothUnresolved road drops"
)]
struct Args {
    /// ETS2 install directory.
    #[arg(long)]
    ets2_dir: PathBuf,

    /// Optional mods dir.
    #[arg(long)]
    mods_dir: Option<PathBuf>,

    /// Path to graph.json (output of MapGraph::build()).
    #[arg(long, default_value = "graph.json")]
    graph_path: PathBuf,

    /// Path to road_drop_audit.json.
    #[arg(long, default_value = "outputs/2026-05-17/prefix/road_drop_audit.json")]
    audit_path: PathBuf,

    /// Output directory.
    #[arg(long, default_value = "outputs/2026-05-17")]
    output_dir: PathBuf,

    /// Optional focus city (Berlin, Hamburg, Wien, …).
    #[arg(long)]
    focus_city: Option<String>,

    /// Optional focus UID — emit a detailed trace for one specific node UID.
    #[arg(long)]
    focus_uid: Option<u64>,

    /// Scan method: parse | bruteforce | both
    #[arg(long, default_value = "both")]
    scan_method: String,

    /// Max number of sample events to include per bucket in the MD report.
    #[arg(long, default_value_t = 50)]
    sample_limit: usize,

    /// Disable the plausibility filter on brute-force hits (more recall,
    /// more false positives).
    #[arg(long)]
    no_filter: bool,
}

fn default_mods_dir() -> PathBuf {
    dirs::document_dir()
        .or_else(dirs::home_dir)
        .unwrap_or_default()
        .join("Euro Truck Simulator 2/mod")
}

fn known_cities() -> Vec<(&'static str, f32, f32)> {
    vec![
        ("Berlin", -16400.0, -3200.0),
        ("Hamburg", -22300.0, -7200.0),
        ("Wien", -8400.0, 3500.0),
        ("Paris", -29000.0, -500.0),
        ("Amsterdam", -25400.0, -9900.0),
        ("Köln", -23600.0, -5000.0),
        ("Frankfurt", -20700.0, -2200.0),
        ("München", -16800.0, 2000.0),
        ("Prag", -12600.0, 400.0),
        ("Warschau", -4200.0, -5800.0),
    ]
}

// ---------------------------------------------------------------------------
// Input types
// ---------------------------------------------------------------------------

#[derive(Deserialize, Clone, Debug)]
struct AuditEvent {
    category: String,
    #[serde(default)]
    #[allow(dead_code)]
    item_type: u32,
    #[serde(default)]
    item_uid: Option<u64>,
    #[serde(default)]
    node_a: Option<u64>,
    #[serde(default)]
    node_b: Option<u64>,
}

#[derive(Deserialize)]
struct GraphPartial {
    nodes: Vec<GraphNode>,
}

#[derive(Deserialize)]
struct GraphNode {
    uid: u64,
}

// ---------------------------------------------------------------------------
// Forensic classification
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Bucket {
    BothMissing,
    OnlyAFound,
    OnlyBFound,
    BothFound,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(clippy::enum_variant_names)]
enum SubBucket {
    SameSector,
    CrossSector,
    RoadPlusBothSameSector,
}

#[derive(Debug, Clone)]
struct ForensicResult {
    road_uid: Option<u64>,
    node_a: u64,
    node_b: u64,
    bucket: Bucket,
    sub_bucket: Option<SubBucket>,
    node_a_sectors: Vec<String>,
    node_b_sectors: Vec<String>,
    road_origin_sectors: Vec<String>,
}

fn bucket_name(b: Bucket) -> &'static str {
    match b {
        Bucket::BothMissing => "both_missing",
        Bucket::OnlyAFound => "only_a_found",
        Bucket::OnlyBFound => "only_b_found",
        Bucket::BothFound => "both_found",
    }
}

fn sub_bucket_name(s: SubBucket) -> &'static str {
    match s {
        SubBucket::SameSector => "same_sector",
        SubBucket::CrossSector => "cross_sector",
        SubBucket::RoadPlusBothSameSector => "road_plus_both_same_sector",
    }
}

// ---------------------------------------------------------------------------
// Brute-force scan
// ---------------------------------------------------------------------------

/// Phase-0-style filter from the spec: confirm a hit looks like a real Node
/// definition (UID followed by plausible XYZ floats) or a RoadFixedHeader
/// (UID at +0, node_a at +245, node_b at +253).
fn is_plausible_node_definition(data: &[u8], hit: usize) -> bool {
    let n = data.len();
    // Need at least 24 bytes after the UID for the XYZ check; otherwise
    // accept (end-of-sector).
    if hit + 8 + 24 > n {
        return true;
    }

    let x = f32::from_le_bytes(data[hit + 8..hit + 12].try_into().unwrap());
    let y = f32::from_le_bytes(data[hit + 12..hit + 16].try_into().unwrap());
    let z = f32::from_le_bytes(data[hit + 16..hit + 20].try_into().unwrap());
    if x.is_finite()
        && y.is_finite()
        && z.is_finite()
        && x.abs() < 250_000.0
        && y.abs() < 10_000.0
        && z.abs() < 250_000.0
    {
        return true;
    }

    // RoadFixedHeader check: uid@0, node_a@245, node_b@253.
    if hit + 261 <= n {
        let na = u64::from_le_bytes(data[hit + 245..hit + 253].try_into().unwrap());
        let nb = u64::from_le_bytes(data[hit + 253..hit + 261].try_into().unwrap());
        if na != 0 && nb != 0 {
            return true;
        }
    }

    false
}

/// Single-pass byte-level scan. Returns the subset of `target` that was
/// found in `data` at a plausible position.
fn scan_sector(data: &[u8], target: &FxHashSet<u64>, apply_filter: bool) -> FxHashSet<u64> {
    let mut found = FxHashSet::default();
    let len = data.len();
    if len < 8 {
        return found;
    }
    let last = len - 8;
    let mut pos = 0usize;
    while pos <= last {
        let candidate = u64::from_le_bytes(data[pos..pos + 8].try_into().unwrap());
        if target.contains(&candidate) && (!apply_filter || is_plausible_node_definition(data, pos))
        {
            found.insert(candidate);
        }
        pos += 1;
    }
    found
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::WARN)
        .with_writer(std::io::stderr)
        .init();

    let args = Args::parse();
    let mods_dir = args.mods_dir.clone().unwrap_or_else(default_mods_dir);
    std::fs::create_dir_all(&args.output_dir)?;

    let t_total = Instant::now();

    // ── Step 1: load audit + graph (Phase 0 input-consistency) ─────────────
    eprintln!("[1/6] Loading audit + graph.json ...");
    let t = Instant::now();
    let audit_raw: Vec<AuditEvent> = serde_json::from_reader(std::io::BufReader::new(
        std::fs::File::open(&args.audit_path)
            .with_context(|| format!("open audit_path {:?}", args.audit_path))?,
    ))
    .context("parse road_drop_audit.json")?;
    let audit_events_bu: Vec<AuditEvent> = audit_raw
        .into_iter()
        .filter(|e| e.category == "both_unresolved")
        .collect();
    eprintln!(
        "      {} BothUnresolved events loaded ({:.1}s)",
        audit_events_bu.len(),
        t.elapsed().as_secs_f32()
    );

    let t = Instant::now();
    eprintln!("      loading graph.json (partial schema) ...");
    let graph: GraphPartial = serde_json::from_reader(std::io::BufReader::new(
        std::fs::File::open(&args.graph_path)
            .with_context(|| format!("open graph_path {:?}", args.graph_path))?,
    ))
    .context("parse graph.json")?;
    let global_uids: FxHashSet<u64> = graph.nodes.iter().map(|n| n.uid).collect();
    eprintln!(
        "      {} graph nodes loaded ({:.1}s)",
        global_uids.len(),
        t.elapsed().as_secs_f32()
    );

    // ── Phase 0: input consistency ────────────────────────────────────────
    eprintln!("[2/6] Phase 0: input-consistency check ...");
    let mut consistent: Vec<AuditEvent> = Vec::new();
    let mut inconsistent: Vec<(AuditEvent, bool, bool)> = Vec::new();
    for e in audit_events_bu.iter() {
        let na = e.node_a.unwrap_or(0);
        let nb = e.node_b.unwrap_or(0);
        let a_in = global_uids.contains(&na);
        let b_in = global_uids.contains(&nb);
        if a_in || b_in {
            inconsistent.push((e.clone(), a_in, b_in));
        } else {
            consistent.push(e.clone());
        }
    }
    eprintln!(
        "      consistent={}, inconsistent={}",
        consistent.len(),
        inconsistent.len()
    );

    // ── Step 2: open archives ─────────────────────────────────────────────
    eprintln!("[3/6] Opening archives ...");
    let t = Instant::now();
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
    eprintln!(
        "      {} archives ready ({:.1}s)",
        archives.len(),
        t.elapsed().as_secs_f32()
    );

    // ── Step 3: discover sectors ──────────────────────────────────────────
    eprintln!("[4/6] Discovering sectors (range ±50) ...");
    let t = Instant::now();
    let mut discovered: Vec<String> = Vec::new();
    for x in -50..=50 {
        for z in -50..=50 {
            let path = format!("map/europe/sec{:+05}{:+05}.base", x, z);
            if archives.iter().any(|a| a.contains(&path)) {
                discovered.push(path);
            }
        }
    }
    discovered.sort();
    discovered.dedup();
    eprintln!(
        "      {} sectors discovered ({:.1}s)",
        discovered.len(),
        t.elapsed().as_secs_f32()
    );

    // ── Step 4: build target UID sets ─────────────────────────────────────
    eprintln!("[5/6] Building target UID sets + brute-force scan ...");
    let t_scan = Instant::now();
    let mut node_target: FxHashSet<u64> = FxHashSet::default();
    let mut road_target: FxHashSet<u64> = FxHashSet::default();
    for e in &consistent {
        if let Some(na) = e.node_a {
            node_target.insert(na);
        }
        if let Some(nb) = e.node_b {
            node_target.insert(nb);
        }
        if let Some(ru) = e.item_uid {
            road_target.insert(ru);
        }
    }
    // Combined target set (one scan finds both kinds of hits).
    let mut combined_target: FxHashSet<u64> = FxHashSet::default();
    combined_target.extend(node_target.iter().copied());
    combined_target.extend(road_target.iter().copied());
    eprintln!(
        "      node_target={}, road_target={}, combined={}",
        node_target.len(),
        road_target.len(),
        combined_target.len()
    );

    let apply_filter = !args.no_filter && args.scan_method != "parse-only-disabled";
    let _ = args.scan_method; // present-as-info; both methods use the same scan

    // Pre-read all sectors and scan. Parallelise with std::thread::scope.
    let uid_to_sectors: Arc<Mutex<FxHashMap<u64, Vec<String>>>> =
        Arc::new(Mutex::new(FxHashMap::default()));

    // Read all sector data sequentially (archives are not Sync). Spawn scan
    // threads from the read loop.
    let combined_arc = Arc::new(combined_target);
    let num_workers = std::thread::available_parallelism()
        .map(|n| n.get().max(1))
        .unwrap_or(4);
    eprintln!(
        "      scanning {} sectors with {} workers ...",
        discovered.len(),
        num_workers
    );

    // Use a job queue: producer reads, workers scan.
    let (tx, rx) = std::sync::mpsc::sync_channel::<(String, Vec<u8>)>(num_workers * 2);
    let rx = Arc::new(Mutex::new(rx));
    let mut handles = Vec::new();
    for _ in 0..num_workers {
        let rx = Arc::clone(&rx);
        let target = Arc::clone(&combined_arc);
        let out = Arc::clone(&uid_to_sectors);
        let h = std::thread::spawn(move || loop {
            let job = {
                let guard = rx.lock().unwrap();
                guard.recv()
            };
            let Ok((path, data)) = job else { break };
            let hits = scan_sector(&data, &target, apply_filter);
            if !hits.is_empty() {
                let mut map = out.lock().unwrap();
                for uid in hits {
                    map.entry(uid).or_default().push(path.clone());
                }
            }
        });
        handles.push(h);
    }

    let mut sectors_read = 0usize;
    for path in &discovered {
        let data_opt: Option<Vec<u8>> = archives
            .iter_mut()
            .rev()
            .find_map(|arc| arc.read_path(path).ok());
        if let Some(data) = data_opt {
            sectors_read += 1;
            tx.send((path.clone(), data)).expect("send job");
        }
        if sectors_read.is_multiple_of(200) {
            eprintln!(
                "      ... {}/{} sectors read ({:.1}s elapsed)",
                sectors_read,
                discovered.len(),
                t_scan.elapsed().as_secs_f32()
            );
        }
    }
    drop(tx);
    for h in handles {
        h.join().expect("scan worker");
    }

    let uid_to_sectors = Arc::try_unwrap(uid_to_sectors)
        .map_err(|_| anyhow::anyhow!("Arc still has references"))?
        .into_inner()
        .map_err(|_| anyhow::anyhow!("Mutex poisoned"))?;
    eprintln!(
        "      scan done: {} sectors read, {} unique UIDs hit ({:.1}s)",
        sectors_read,
        uid_to_sectors.len(),
        t_scan.elapsed().as_secs_f32()
    );

    // Split the combined hits back into node-index vs road-origin-index.
    let mut node_index: FxHashMap<u64, Vec<String>> = FxHashMap::default();
    let mut road_origin: FxHashMap<u64, Vec<String>> = FxHashMap::default();
    for (uid, sectors) in uid_to_sectors {
        let in_nodes = node_target.contains(&uid);
        let in_roads = road_target.contains(&uid);
        if in_nodes {
            node_index.insert(uid, sectors.clone());
        }
        if in_roads {
            road_origin.insert(uid, sectors);
        }
    }
    eprintln!(
        "      node_index entries={}, road_origin entries={}",
        node_index.len(),
        road_origin.len()
    );

    // ── Step 5: classify ──────────────────────────────────────────────────
    eprintln!("[6/6] Classifying events + writing report ...");
    let t = Instant::now();
    let empty: Vec<String> = Vec::new();
    let mut results: Vec<ForensicResult> = Vec::with_capacity(consistent.len());
    for e in &consistent {
        let na = e.node_a.unwrap_or(0);
        let nb = e.node_b.unwrap_or(0);
        let a_sectors = node_index.get(&na).unwrap_or(&empty).clone();
        let b_sectors = node_index.get(&nb).unwrap_or(&empty).clone();
        let road_sectors = e
            .item_uid
            .and_then(|ru| road_origin.get(&ru).cloned())
            .unwrap_or_default();

        let bucket = match (!a_sectors.is_empty(), !b_sectors.is_empty()) {
            (false, false) => Bucket::BothMissing,
            (true, false) => Bucket::OnlyAFound,
            (false, true) => Bucket::OnlyBFound,
            (true, true) => Bucket::BothFound,
        };

        let sub = if bucket == Bucket::BothFound {
            let a_set: HashSet<&str> = a_sectors.iter().map(|s| s.as_str()).collect();
            let b_set: HashSet<&str> = b_sectors.iter().map(|s| s.as_str()).collect();
            let road_set: HashSet<&str> = road_sectors.iter().map(|s| s.as_str()).collect();
            let shared: Vec<&str> = a_set.intersection(&b_set).copied().collect();
            let road_plus_both = shared.iter().any(|p| road_set.contains(p));
            if road_plus_both {
                Some(SubBucket::RoadPlusBothSameSector)
            } else if !shared.is_empty() {
                Some(SubBucket::SameSector)
            } else {
                Some(SubBucket::CrossSector)
            }
        } else {
            None
        };

        results.push(ForensicResult {
            road_uid: e.item_uid,
            node_a: na,
            node_b: nb,
            bucket,
            sub_bucket: sub,
            node_a_sectors: a_sectors,
            node_b_sectors: b_sectors,
            road_origin_sectors: road_sectors,
        });
    }
    eprintln!(
        "      classified {} events ({:.1}s)",
        results.len(),
        t.elapsed().as_secs_f32()
    );

    // ── Step 6: write outputs ─────────────────────────────────────────────
    let scan_duration_ms = t_scan.elapsed().as_millis() as u64;
    let total_duration_ms = t_total.elapsed().as_millis() as u64;

    let md_name = match &args.focus_city {
        Some(city) => format!("bothunresolved_forensik_{}.md", city),
        None => "bothunresolved_forensik.md".to_string(),
    };
    let json_name = "bothunresolved_forensik.json".to_string();

    write_markdown(
        &args.output_dir.join(&md_name),
        &args,
        &results,
        &inconsistent,
        &discovered,
        &global_uids,
        scan_duration_ms,
        total_duration_ms,
    )?;
    eprintln!("      wrote {}", args.output_dir.join(&md_name).display());

    // JSON only on default run (without focus-city), so a city run does not
    // overwrite the full bucket dump.
    if args.focus_city.is_none() {
        write_json(
            &args.output_dir.join(&json_name),
            &args,
            &results,
            &inconsistent,
            &discovered,
            &global_uids,
            scan_duration_ms,
            total_duration_ms,
        )?;
        eprintln!("      wrote {}", args.output_dir.join(&json_name).display());
    }

    eprintln!("Done. ({:.1}s total)", t_total.elapsed().as_secs_f32());
    Ok(())
}

// ---------------------------------------------------------------------------
// Output: Markdown
// ---------------------------------------------------------------------------

#[allow(clippy::too_many_arguments)]
fn write_markdown(
    path: &std::path::Path,
    args: &Args,
    results: &[ForensicResult],
    inconsistent: &[(AuditEvent, bool, bool)],
    discovered: &[String],
    global_uids: &FxHashSet<u64>,
    scan_duration_ms: u64,
    total_duration_ms: u64,
) -> Result<()> {
    let mut f = std::fs::File::create(path)?;
    macro_rules! w {
        ($($arg:tt)*) => { writeln!(f, $($arg)*)?; }
    }

    let total_classified = results.len();
    let mut counts = [0usize; 4];
    let mut sub_counts = [0usize; 3];
    for r in results {
        counts[r.bucket as usize] += 1;
        if let Some(sb) = r.sub_bucket {
            sub_counts[sb as usize] += 1;
        }
    }
    let pct = |n: usize, d: usize| {
        if d == 0 {
            0.0
        } else {
            n as f32 * 100.0 / d as f32
        }
    };

    w!("# BothUnresolved-UID-Forensik");
    w!();
    w!("> Generated by `bothunresolved-forensik` (Phase 6.2b-Diag-5)");
    w!(
        "> Scan method: `{}` (filter={})",
        args.scan_method,
        if args.no_filter { "off" } else { "on" }
    );
    w!();

    // ── 1. Summary ────────────────────────────────────────────────────────
    w!("## 1. Summary");
    w!();
    w!("| Metric | Value |");
    w!("|---|---|");
    w!(
        "| BothUnresolved events (input) | {} |",
        inconsistent.len() + total_classified
    );
    w!(
        "| InputInconsistency (UID in graph.json) | {} |",
        inconsistent.len()
    );
    w!("| Classifiable events | {} |", total_classified);
    w!("| Sectors discovered | {} |", discovered.len());
    w!("| Global node-map size | {} |", global_uids.len());
    w!(
        "| Scan duration | {:.1}s |",
        scan_duration_ms as f32 / 1000.0
    );
    w!(
        "| Total duration | {:.1}s |",
        total_duration_ms as f32 / 1000.0
    );
    w!();

    // ── 2. Input data quality ─────────────────────────────────────────────
    w!("## 2. Input Data Quality");
    w!();
    if inconsistent.is_empty() {
        w!(
            "✅ All {} BothUnresolved events are consistent with `graph.json`.",
            total_classified
        );
        w!();
    } else {
        w!(
            "⚠ **{} events excluded as InputInconsistency** — these BothUnresolved",
            inconsistent.len()
        );
        w!("events have a `node_a` or `node_b` UID that is present in `graph.json`.");
        w!("Possible cause: `road_drop_audit.json` and `graph.json` from different");
        w!("parse runs, or non-atomic JSON writes.");
        w!();
        w!("| Road UID | node_a | a_in_global | node_b | b_in_global |");
        w!("|---|---|---|---|---|");
        for (e, a, b) in inconsistent.iter().take(20) {
            w!(
                "| {:?} | {:?} | {} | {:?} | {} |",
                e.item_uid,
                e.node_a,
                a,
                e.node_b,
                b
            );
        }
        if inconsistent.len() > 20 {
            w!("| ... {} more ... | | | | |", inconsistent.len() - 20);
        }
        w!();
    }

    // ── 3. Bucket distribution ────────────────────────────────────────────
    w!("## 3. Bucket Distribution (n={})", total_classified);
    w!();
    w!("| Bucket | Count | % | Interpretation |");
    w!("|---|---|---|---|");
    w!(
        "| BothMissing | {} | {:.1}% | Neither UID found in any sector |",
        counts[Bucket::BothMissing as usize],
        pct(counts[Bucket::BothMissing as usize], total_classified)
    );
    w!(
        "| OnlyAFound | {} | {:.1}% | Only node_a found in sector(s) |",
        counts[Bucket::OnlyAFound as usize],
        pct(counts[Bucket::OnlyAFound as usize], total_classified)
    );
    w!(
        "| OnlyBFound | {} | {:.1}% | Only node_b found in sector(s) |",
        counts[Bucket::OnlyBFound as usize],
        pct(counts[Bucket::OnlyBFound as usize], total_classified)
    );
    w!(
        "| BothFound | {} | {:.1}% | Both UIDs found in sector(s) (merge issue) |",
        counts[Bucket::BothFound as usize],
        pct(counts[Bucket::BothFound as usize], total_classified)
    );
    w!();

    // ── 4. BothFound sub-buckets ──────────────────────────────────────────
    let both_found = counts[Bucket::BothFound as usize];
    if both_found > 0 {
        w!("## 4. BothFound Sub-Buckets (n={})", both_found);
        w!();
        w!("| Sub-Bucket | Count | % of BothFound |");
        w!("|---|---|---|");
        w!(
            "| SameSector (both nodes in same sector) | {} | {:.1}% |",
            sub_counts[SubBucket::SameSector as usize],
            pct(sub_counts[SubBucket::SameSector as usize], both_found)
        );
        w!(
            "| CrossSector (nodes in different sectors) | {} | {:.1}% |",
            sub_counts[SubBucket::CrossSector as usize],
            pct(sub_counts[SubBucket::CrossSector as usize], both_found)
        );
        w!(
            "| RoadPlusBothSameSector (road + both nodes same sector) | {} | {:.1}% |",
            sub_counts[SubBucket::RoadPlusBothSameSector as usize],
            pct(
                sub_counts[SubBucket::RoadPlusBothSameSector as usize],
                both_found
            )
        );
        w!();
    }

    // ── 5. Top sectors per bucket ─────────────────────────────────────────
    w!("## 5. Top-10 Sectors per Bucket (by event count where road originates)");
    w!();
    for b in [
        Bucket::BothMissing,
        Bucket::OnlyAFound,
        Bucket::OnlyBFound,
        Bucket::BothFound,
    ] {
        let mut secs: FxHashMap<String, usize> = FxHashMap::default();
        for r in results.iter().filter(|r| r.bucket == b) {
            for s in &r.road_origin_sectors {
                *secs.entry(s.clone()).or_insert(0) += 1;
            }
        }
        if secs.is_empty() {
            continue;
        }
        let mut top: Vec<(String, usize)> = secs.into_iter().collect();
        top.sort_by_key(|(_, c)| std::cmp::Reverse(*c));
        top.truncate(10);
        w!("**{:?}**", b);
        w!();
        w!("| Sector | Events |");
        w!("|---|---|");
        for (s, c) in top {
            w!("| `{}` | {} |", s, c);
        }
        w!();
    }

    // ── 6. Sample events per bucket ───────────────────────────────────────
    w!(
        "## 6. Sample Events per Bucket (up to {})",
        args.sample_limit
    );
    w!();
    for b in [
        Bucket::BothMissing,
        Bucket::OnlyAFound,
        Bucket::OnlyBFound,
        Bucket::BothFound,
    ] {
        let samples: Vec<&ForensicResult> = results
            .iter()
            .filter(|r| r.bucket == b)
            .take(args.sample_limit)
            .collect();
        if samples.is_empty() {
            continue;
        }
        w!(
            "### {:?} (showing {} of {})",
            b,
            samples.len(),
            counts[b as usize]
        );
        w!();
        w!("| Road UID | node_a | node_b | a_sectors | b_sectors | road_sectors |");
        w!("|---|---|---|---|---|---|");
        for r in samples {
            let a_sec = if r.node_a_sectors.is_empty() {
                "—".to_string()
            } else {
                r.node_a_sectors
                    .iter()
                    .take(2)
                    .map(|s| short_sec(s))
                    .collect::<Vec<_>>()
                    .join(", ")
            };
            let b_sec = if r.node_b_sectors.is_empty() {
                "—".to_string()
            } else {
                r.node_b_sectors
                    .iter()
                    .take(2)
                    .map(|s| short_sec(s))
                    .collect::<Vec<_>>()
                    .join(", ")
            };
            let r_sec = if r.road_origin_sectors.is_empty() {
                "—".to_string()
            } else {
                r.road_origin_sectors
                    .iter()
                    .take(2)
                    .map(|s| short_sec(s))
                    .collect::<Vec<_>>()
                    .join(", ")
            };
            w!(
                "| {:?} | {} | {} | {} | {} | {} |",
                r.road_uid,
                r.node_a,
                r.node_b,
                a_sec,
                b_sec,
                r_sec
            );
        }
        w!();
    }

    // ── 7. City correlation ───────────────────────────────────────────────
    w!("## 7. City Correlation");
    w!();
    w!("Road-origin sector center estimated from sector grid (x = +0001 → ~4096m).");
    w!("Distance is computed from the city XZ to the sector's approximate center.");
    w!();
    let cities = known_cities();
    w!("| City | BothMissing | OnlyA | OnlyB | BothFound | Total in 10km radius |");
    w!("|---|---|---|---|---|---|");
    for (name, cx, cz) in &cities {
        let mut bm = 0;
        let mut oa = 0;
        let mut ob = 0;
        let mut bf = 0;
        for r in results {
            let in_radius = r.road_origin_sectors.iter().any(|s| {
                if let Some((sx, sz)) = parse_sector_xz(s) {
                    let dx = sx - cx;
                    let dz = sz - cz;
                    (dx * dx + dz * dz).sqrt() < 10_000.0
                } else {
                    false
                }
            });
            if in_radius {
                match r.bucket {
                    Bucket::BothMissing => bm += 1,
                    Bucket::OnlyAFound => oa += 1,
                    Bucket::OnlyBFound => ob += 1,
                    Bucket::BothFound => bf += 1,
                }
            }
        }
        w!(
            "| {} | {} | {} | {} | {} | {} |",
            name,
            bm,
            oa,
            ob,
            bf,
            bm + oa + ob + bf
        );
    }
    w!();

    // ── 8. Focus city ─────────────────────────────────────────────────────
    if let Some(city) = &args.focus_city {
        write_focus_city(&mut f, city, results, &cities, args.sample_limit)?;
    }

    // ── 9. Focus UID ──────────────────────────────────────────────────────
    if let Some(uid) = args.focus_uid {
        w!("## Focus UID: {}", uid);
        w!();
        let hits: Vec<&ForensicResult> = results
            .iter()
            .filter(|r| r.node_a == uid || r.node_b == uid)
            .collect();
        w!("Events with this UID as node_a or node_b: {}", hits.len());
        w!();
        if !hits.is_empty() {
            w!("| Road UID | node_a | node_b | bucket | sub | a_secs | b_secs | road_secs |");
            w!("|---|---|---|---|---|---|---|---|");
            for r in hits.iter().take(50) {
                w!(
                    "| {:?} | {} | {} | {:?} | {:?} | {} | {} | {} |",
                    r.road_uid,
                    r.node_a,
                    r.node_b,
                    r.bucket,
                    r.sub_bucket,
                    r.node_a_sectors.len(),
                    r.node_b_sectors.len(),
                    r.road_origin_sectors.len()
                );
            }
            w!();
        }
    }

    // ── 10. Auto-diagnosis ────────────────────────────────────────────────
    w!("## 8. Auto-Diagnosis");
    w!();
    write_diagnosis(
        &mut f,
        total_classified,
        inconsistent.len(),
        &counts,
        &sub_counts,
    )?;

    Ok(())
}

fn write_focus_city(
    f: &mut std::fs::File,
    city_name: &str,
    results: &[ForensicResult],
    cities: &[(&'static str, f32, f32)],
    sample_limit: usize,
) -> Result<()> {
    macro_rules! w {
        ($($arg:tt)*) => { writeln!(f, $($arg)*)?; }
    }
    let Some(&(_, cx, cz)) = cities
        .iter()
        .find(|(n, _, _)| n.eq_ignore_ascii_case(city_name))
    else {
        w!("## Focus City: `{}` — NOT in known_cities()", city_name);
        w!();
        return Ok(());
    };
    w!("## Focus City: {}", city_name);
    w!();
    w!("- City XZ: ({:.0}, {:.0})", cx, cz);
    w!();

    let in_radius: Vec<&ForensicResult> = results
        .iter()
        .filter(|r| {
            r.road_origin_sectors.iter().any(|s| {
                parse_sector_xz(s)
                    .map(|(sx, sz)| ((sx - cx).powi(2) + (sz - cz).powi(2)).sqrt() < 10_000.0)
                    .unwrap_or(false)
            })
        })
        .collect();
    w!(
        "- BothUnresolved roads with origin sector in 10km radius: **{}**",
        in_radius.len()
    );

    let mut bcount = [0usize; 4];
    for r in &in_radius {
        bcount[r.bucket as usize] += 1;
    }
    w!("  - BothMissing: {}", bcount[0]);
    w!("  - OnlyAFound: {}", bcount[1]);
    w!("  - OnlyBFound: {}", bcount[2]);
    w!("  - BothFound: {}", bcount[3]);
    w!();

    // Sector breakdown
    let mut sec_count: FxHashMap<String, [usize; 4]> = FxHashMap::default();
    for r in &in_radius {
        for s in &r.road_origin_sectors {
            let e = sec_count.entry(s.clone()).or_insert([0; 4]);
            e[r.bucket as usize] += 1;
        }
    }
    let mut sec_vec: Vec<(String, [usize; 4])> = sec_count.into_iter().collect();
    sec_vec.sort_by_key(|(_, c)| std::cmp::Reverse(c.iter().sum::<usize>()));
    w!("### Sector breakdown");
    w!();
    w!("| Sector | BothMissing | OnlyA | OnlyB | BothFound | Total |");
    w!("|---|---|---|---|---|---|");
    for (s, c) in sec_vec.iter().take(30) {
        let total: usize = c.iter().sum();
        w!(
            "| `{}` | {} | {} | {} | {} | {} |",
            short_sec(s),
            c[0],
            c[1],
            c[2],
            c[3],
            total
        );
    }
    w!();

    // Show up to N detailed events
    w!("### Sample events in 10km radius (up to {})", sample_limit);
    w!();
    w!("| Road UID | node_a | node_b | bucket | sub | road_sectors | a_sectors | b_sectors |");
    w!("|---|---|---|---|---|---|---|---|");
    for r in in_radius.iter().take(sample_limit) {
        w!(
            "| {:?} | {} | {} | {:?} | {:?} | {} | {} | {} |",
            r.road_uid,
            r.node_a,
            r.node_b,
            r.bucket,
            r.sub_bucket,
            r.road_origin_sectors
                .iter()
                .take(3)
                .map(|s| short_sec(s))
                .collect::<Vec<_>>()
                .join(", "),
            r.node_a_sectors
                .iter()
                .take(3)
                .map(|s| short_sec(s))
                .collect::<Vec<_>>()
                .join(", "),
            r.node_b_sectors
                .iter()
                .take(3)
                .map(|s| short_sec(s))
                .collect::<Vec<_>>()
                .join(", "),
        );
    }
    w!();
    Ok(())
}

fn write_diagnosis(
    f: &mut std::fs::File,
    total: usize,
    inconsistent: usize,
    counts: &[usize; 4],
    sub_counts: &[usize; 3],
) -> Result<()> {
    macro_rules! w {
        ($($arg:tt)*) => { writeln!(f, $($arg)*)?; }
    }
    if total == 0 {
        w!("**ROOT=NONE: no classifiable BothUnresolved events.**");
        w!();
        return Ok(());
    }
    let bm = counts[Bucket::BothMissing as usize] as f32 * 100.0 / total as f32;
    let oa = counts[Bucket::OnlyAFound as usize] as f32 * 100.0 / total as f32;
    let ob = counts[Bucket::OnlyBFound as usize] as f32 * 100.0 / total as f32;
    let bf = counts[Bucket::BothFound as usize] as f32 * 100.0 / total as f32;
    let bf_total = counts[Bucket::BothFound as usize] as f32;
    let same = if bf_total > 0.0 {
        sub_counts[SubBucket::SameSector as usize] as f32 * 100.0 / bf_total
    } else {
        0.0
    };

    if inconsistent > 0 {
        w!(
            "⚠ **InputInconsistency: {} events excluded.** `road_drop_audit.json` and",
            inconsistent
        );
        w!("`graph.json` may come from different parse runs.");
        w!();
    }

    if bm > 60.0 {
        w!("**ROOT=COVERAGE (HIGH confidence)**");
        w!();
        w!(
            "{:.1}% of classifiable BothUnresolved roads have endpoint UIDs that are",
            bm
        );
        w!("in NO parsed sector. Likely causes:");
        w!("- Unparsed sectors (DLCs not installed/loaded)");
        w!("- Cursor desync in item stream → node section unreached");
        w!("- Item-type skip bug → sector aborted before node section");
        w!("- DLC reference to unloaded map extension");
        w!();
        w!("**Recommended next phase**: Parser-Coverage-Audit.");
    } else if bf > 40.0 {
        if same > 50.0 {
            w!("**ROOT=MERGE (HIGH confidence)**");
            w!();
            w!(
                "{:.1}% of BothUnresolved roads have both endpoint UIDs in sector data",
                bf
            );
            w!(
                "({:.1}% in the SAME sector). Parser extracts the nodes correctly,",
                same
            );
            w!("but the merge logic discards them. Likely causes:");
            w!("- HashMap collision (u64 duplicates)");
            w!("- Sector order bug (DLC/Mod overrides Base nodes)");
            w!("- DLC-Guard filter discards nodes");
            w!("- Building-item filter (buildings not merged)");
            w!();
            w!("**Recommended next phase**: Merge-Logic-Audit.");
        } else {
            w!("**ROOT=CROSS_SECTOR_MERGE**");
            w!();
            w!(
                "{:.1}% BothFound, but only {:.1}% in same sector — cross-sector",
                bf,
                same
            );
            w!("references are not resolved.");
            w!();
            w!("**Recommended next phase**: Cross-Sector-Merge-Phase.");
        }
    } else if oa + ob > 30.0 {
        w!("**ROOT=PARTIAL_COVERAGE**");
        w!();
        w!(
            "{:.1}% OnlyAFound + {:.1}% OnlyBFound — one endpoint UID per road",
            oa,
            ob
        );
        w!("is in sector data, the other isn't. One sector defines the node,");
        w!("the other doesn't (or is missing/unparseable).");
        w!();
        w!("**Recommended next phase**: Sector-Pair-Audit.");
    } else {
        w!("**ROOT=MIXED**");
        w!();
        w!(
            "No dominant bucket: BothMissing={:.1}%, BothFound={:.1}%, OnlyOne={:.1}%.",
            bm,
            bf,
            oa + ob
        );
        w!("Spatial cluster analysis recommended.");
    }
    w!();
    Ok(())
}

// ---------------------------------------------------------------------------
// Output: JSON
// ---------------------------------------------------------------------------

#[allow(clippy::too_many_arguments)]
fn write_json(
    path: &std::path::Path,
    args: &Args,
    results: &[ForensicResult],
    inconsistent: &[(AuditEvent, bool, bool)],
    discovered: &[String],
    global_uids: &FxHashSet<u64>,
    scan_duration_ms: u64,
    total_duration_ms: u64,
) -> Result<()> {
    let total_classified = results.len();
    let total_input = inconsistent.len() + total_classified;
    let mut counts = [0usize; 4];
    let mut sub_counts = [0usize; 3];
    for r in results {
        counts[r.bucket as usize] += 1;
        if let Some(sb) = r.sub_bucket {
            sub_counts[sb as usize] += 1;
        }
    }
    let pct = |n: usize, d: usize| {
        if d == 0 {
            0.0f64
        } else {
            n as f64 * 100.0 / d as f64
        }
    };

    let mut bucket_events: [Vec<serde_json::Value>; 4] = Default::default();
    for r in results {
        let v = json!({
            "road_uid": r.road_uid,
            "node_a": r.node_a,
            "node_b": r.node_b,
            "bucket": bucket_name(r.bucket),
            "sub_bucket": r.sub_bucket.map(sub_bucket_name),
            "node_a_sectors": r.node_a_sectors,
            "node_b_sectors": r.node_b_sectors,
            "road_origin_sectors": r.road_origin_sectors,
        });
        bucket_events[r.bucket as usize].push(v);
    }

    let out = json!({
        "meta": {
            "tool": "bothunresolved-forensik",
            "version": "1.0",
            "scan_method": args.scan_method,
            "filter": !args.no_filter,
            "input_audit_path": args.audit_path,
            "input_graph_path": args.graph_path,
            "total_input_events": total_input,
            "total_classified": total_classified,
            "sectors_scanned": discovered.len(),
            "global_node_map_size": global_uids.len(),
            "scan_duration_ms": scan_duration_ms,
            "total_duration_ms": total_duration_ms,
        },
        "input_consistency": {
            "total_input_events": total_input,
            "inconsistent_events": inconsistent.len(),
            "classified_events": total_classified,
        },
        "buckets": {
            "both_missing": {
                "count": counts[Bucket::BothMissing as usize],
                "pct": pct(counts[Bucket::BothMissing as usize], total_classified),
                "events": bucket_events[Bucket::BothMissing as usize],
            },
            "only_a_found": {
                "count": counts[Bucket::OnlyAFound as usize],
                "pct": pct(counts[Bucket::OnlyAFound as usize], total_classified),
                "events": bucket_events[Bucket::OnlyAFound as usize],
            },
            "only_b_found": {
                "count": counts[Bucket::OnlyBFound as usize],
                "pct": pct(counts[Bucket::OnlyBFound as usize], total_classified),
                "events": bucket_events[Bucket::OnlyBFound as usize],
            },
            "both_found": {
                "count": counts[Bucket::BothFound as usize],
                "pct": pct(counts[Bucket::BothFound as usize], total_classified),
                "sub": {
                    "same_sector": {
                        "count": sub_counts[SubBucket::SameSector as usize],
                        "pct_of_bucket": pct(sub_counts[SubBucket::SameSector as usize], counts[Bucket::BothFound as usize]),
                    },
                    "cross_sector": {
                        "count": sub_counts[SubBucket::CrossSector as usize],
                        "pct_of_bucket": pct(sub_counts[SubBucket::CrossSector as usize], counts[Bucket::BothFound as usize]),
                    },
                    "road_plus_both_same_sector": {
                        "count": sub_counts[SubBucket::RoadPlusBothSameSector as usize],
                        "pct_of_bucket": pct(sub_counts[SubBucket::RoadPlusBothSameSector as usize], counts[Bucket::BothFound as usize]),
                    },
                },
                "events": bucket_events[Bucket::BothFound as usize],
            },
        },
    });
    let s = serde_json::to_string_pretty(&out)?;
    std::fs::write(path, s)?;
    Ok(())
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn short_sec(path: &str) -> String {
    path.rsplit('/')
        .next()
        .unwrap_or(path)
        .trim_end_matches(".base")
        .to_string()
}

/// Parse a sector path like `map/europe/sec+0001-0003.base` into world XZ
/// (sector center, very approximate — each sector = 4096m × 4096m).
fn parse_sector_xz(path: &str) -> Option<(f32, f32)> {
    let name = path.rsplit('/').next()?;
    let name = name.strip_suffix(".base")?;
    let name = name.strip_prefix("sec")?;
    if name.len() < 10 {
        return None;
    }
    let x_str = &name[0..5];
    let z_str = &name[5..10];
    let x: i32 = x_str.parse().ok()?;
    let z: i32 = z_str.parse().ok()?;
    // ETS2 sector grid: 4096m per step. Sector center = grid origin + half.
    Some((x as f32 * 4096.0 + 2048.0, z as f32 * 4096.0 + 2048.0))
}
