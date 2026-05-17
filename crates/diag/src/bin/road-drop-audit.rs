//! `road-drop-audit` — Phase 6.2b-Diag-3: Road-Drop-Audit-CLI
//!
//! Instruments the full parse pipeline to trace every point where a road item
//! is silently discarded before it can contribute edges to the routing graph.
//!
//! Two drop layers are probed:
//!   1. Sector-level: parse_sector_with_tracer records RoadParseFailed,
//!      SectorHandlerError, UnknownItemType events.
//!   2. Graph-level: analyze_roads_for_audit classifies roads whose node
//!      UIDs are absent from the global node map → BothUnresolved or
//!      OneUnresolved.
//!
//! Hard Gate Guards (MUST NOT be violated):
//!   - No parser behaviour changes (this binary does NOT affect production).
//!   - No new item types implemented.
//!   - No DLC-guard changes.
//!   - No Pass-2/3 spatial matching.
//!   - No commit until output has been reviewed.
//!
//! Usage:
//!   cargo run --release -p truckpilot-diag --bin road-drop-audit -- \
//!     --ets2-dir "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2" \
//!     --output-dir outputs/2026-05-16 --focus-city Berlin --hex-dump-limit 64

use std::collections::HashMap;
use std::io::Write as IoWrite;
use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;
use truckpilot_map_parser::{
    drop_tracer::{DropCategory, DropEvent},
    DropTracer, HashFsArchive, ModLoadOrder, ZipArchive,
    Archive,
    parse_sectors_with_drop_tracer,
};

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

#[derive(Parser)]
#[command(
    name = "road-drop-audit",
    about = "Phase 6.2b-Diag-3: instrument parse pipeline to trace silently-dropped roads"
)]
struct Args {
    /// ETS2 install directory (contains base_map.scs etc.)
    #[arg(long)]
    ets2_dir: PathBuf,

    /// Optional mods directory (defaults to ~/Documents/Euro Truck Simulator 2/mod)
    #[arg(long)]
    mods_dir: Option<PathBuf>,

    /// Output directory for reports (will be created if absent)
    #[arg(long, default_value = "outputs/2026-05-16")]
    output_dir: PathBuf,

    /// City to produce a detailed trace for (e.g. Berlin, Hamburg, Wien)
    #[arg(long)]
    focus_city: Option<String>,

    /// Sector filename substring filter for detailed traces (e.g. sec+0000+0000)
    #[arg(long)]
    focus_sector: Option<String>,

    /// Maximum bytes of raw header captured per drop event
    #[arg(long, default_value_t = 64)]
    hex_dump_limit: usize,
}

// ---------------------------------------------------------------------------
// Known cities — XZ world coordinates (ETS2 engine units, same as node coords)
// ---------------------------------------------------------------------------

fn known_cities() -> Vec<(&'static str, f32, f32)> {
    vec![
        ("Berlin",   -16400.0, -3200.0),
        ("Hamburg",  -22300.0, -7200.0),
        ("Wien",      -8400.0,  3500.0),
        ("Paris",    -29000.0,  -500.0),
        ("Amsterdam",-25400.0, -9900.0),
        ("Köln",     -23600.0, -5000.0),
        ("Frankfurt",-20700.0, -2200.0),
        ("München",  -16800.0,  2000.0),
        ("Prag",     -12600.0,   400.0),
        ("Warschau",  -4200.0, -5800.0),
    ]
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn default_mods_dir() -> PathBuf {
    dirs::document_dir()
        .or_else(dirs::home_dir)
        .unwrap_or_default()
        .join("Euro Truck Simulator 2/mod")
}

fn category_label(cat: &DropCategory) -> &'static str {
    match cat {
        DropCategory::RoadParseFailed       => "RoadParseFailed",
        DropCategory::SectorHandlerError    => "SectorHandlerError",
        DropCategory::UnknownItemType       => "UnknownItemType",
        DropCategory::BothUnresolved        => "BothUnresolved",
        DropCategory::OneUnresolved         => "OneUnresolved",
        DropCategory::SizedRoadParseFailed  => "SizedRoadParseFailed",
    }
}

fn hex_dump(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(" ")
}

/// XZ distance (metres) between two points.
fn dist_xz(ax: f32, az: f32, bx: f32, bz: f32) -> f32 {
    let dx = ax - bx;
    let dz = az - bz;
    (dx * dx + dz * dz).sqrt()
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
    let mods_dir = args.mods_dir.unwrap_or_else(default_mods_dir);

    std::fs::create_dir_all(&args.output_dir)
        .with_context(|| format!("create output dir {:?}", args.output_dir))?;

    // ── 1. Open archives ────────────────────────────────────────────────────
    eprintln!("Opening archives from {} ...", args.ets2_dir.display());
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
    eprintln!("Opened {} archives.", archives.len());

    // ── 2. Parse with drop tracer (sector-level instrumentation) ────────────
    eprintln!("Parsing sectors with drop tracer (hex_limit={}) ...", args.hex_dump_limit);
    let tracer = DropTracer::new(args.hex_dump_limit);
    let (builder, _parsed_paths) =
        parse_sectors_with_drop_tracer(&mut archives, &tracer)
            .context("parse_sectors_with_drop_tracer")?;

    let sector_events = tracer.take_events();
    eprintln!(
        "Sector-level drops recorded: {}",
        sector_events.len()
    );

    // ── 3. Graph-level drops ─────────────────────────────────────────────────
    eprintln!(
        "Builder has {} roads, {} nodes — classifying graph-level drops ...",
        builder.roads().len(),
        builder.raw_nodes().len()
    );
    let audit = builder.analyze_roads_for_audit();
    eprintln!(
        "Graph-level: {} BothUnresolved, {} OneUnresolved",
        audit.both_unresolved.len(),
        audit.one_unresolved.len()
    );

    // Synthesise DropEvents for graph-level drops so the JSON output is uniform.
    let mut graph_events: Vec<DropEvent> = Vec::new();
    for road in &audit.both_unresolved {
        graph_events.push(DropEvent {
            category: DropCategory::BothUnresolved,
            sector_path: "graph_builder".to_string(),
            item_type: 3,
            item_uid: Some(road.uid),
            node_a: Some(road.node_a),
            node_b: Some(road.node_b),
            node_a_resolved: Some(false),
            node_b_resolved: Some(false),
            x: None,
            z: None,
            raw_hex: Vec::new(),
        });
    }
    for (road, resolved_uid, resolved_pos) in &audit.one_unresolved {
        let (a_ok, b_ok) = (*resolved_uid == road.node_a, *resolved_uid == road.node_b);
        graph_events.push(DropEvent {
            category: DropCategory::OneUnresolved,
            sector_path: "graph_builder".to_string(),
            item_type: 3,
            item_uid: Some(road.uid),
            node_a: Some(road.node_a),
            node_b: Some(road.node_b),
            node_a_resolved: Some(a_ok),
            node_b_resolved: Some(b_ok),
            x: Some(resolved_pos[0] as f32),
            z: Some(resolved_pos[2] as f32),
            raw_hex: Vec::new(),
        });
    }

    let all_events: Vec<&DropEvent> = sector_events.iter().chain(graph_events.iter()).collect();

    // ── 4. Aggregate statistics ──────────────────────────────────────────────
    let mut by_category: HashMap<String, usize> = HashMap::new();
    let mut by_sector: HashMap<&str, usize> = HashMap::new();
    let mut by_item_type: HashMap<u32, usize> = HashMap::new();
    for ev in &all_events {
        *by_category.entry(category_label(&ev.category).to_string()).or_default() += 1;
        *by_sector.entry(ev.sector_path.as_str()).or_default() += 1;
        if matches!(
            ev.category,
            DropCategory::SectorHandlerError | DropCategory::UnknownItemType
        ) {
            *by_item_type.entry(ev.item_type).or_default() += 1;
        }
    }

    let road_parse_failed   = *by_category.get("RoadParseFailed").unwrap_or(&0);
    let handler_error       = *by_category.get("SectorHandlerError").unwrap_or(&0);
    let unknown_type        = *by_category.get("UnknownItemType").unwrap_or(&0);
    let both_unresolved     = audit.both_unresolved.len();
    let one_unresolved      = audit.one_unresolved.len();
    let total_roads         = builder.roads().len();
    let total_nodes         = builder.raw_nodes().len();
    let fully_resolved      = total_roads - both_unresolved - one_unresolved;

    // ── 5. Focus-city analysis ───────────────────────────────────────────────
    let cities = known_cities();
    let focus_result: Option<FocusResult> = args.focus_city.as_ref().and_then(|name| {
        let city = cities.iter().find(|(n, _, _)| n.eq_ignore_ascii_case(name))?;
        compute_focus(city.0, city.1, city.2, &builder, &sector_events, &graph_events)
    });

    // ── 6. Write JSON (all events) ───────────────────────────────────────────
    let json_path = args.output_dir.join("road_drop_audit.json");
    {
        let all_for_json: Vec<&DropEvent> = all_events.to_vec();
        let json = serde_json::to_string_pretty(&all_for_json)?;
        std::fs::write(&json_path, &json)?;
        eprintln!("Wrote {}", json_path.display());
    }

    // ── 7. Write Markdown report ─────────────────────────────────────────────
    let md_path = args.output_dir.join("road_drop_audit.md");
    write_markdown(
        &md_path,
        &by_category,
        &by_sector,
        &by_item_type,
        road_parse_failed,
        handler_error,
        unknown_type,
        both_unresolved,
        one_unresolved,
        fully_resolved,
        total_roads,
        total_nodes,
        &sector_events,
        &focus_result,
        args.focus_sector.as_deref(),
    )?;
    eprintln!("Wrote {}", md_path.display());

    // ── 8. Per-city trace ────────────────────────────────────────────────────
    if let (Some(city_name), Some(fr)) = (&args.focus_city, &focus_result) {
        let trace_name = format!("road_drop_{city_name}_trace.md");
        let trace_path = args.output_dir.join(&trace_name);
        write_city_trace(&trace_path, city_name, fr, &sector_events, &graph_events)?;
        eprintln!("Wrote {}", trace_path.display());
    }

    eprintln!("Done.");
    Ok(())
}

// ---------------------------------------------------------------------------
// Focus-city analysis
// ---------------------------------------------------------------------------

struct FocusResult {
    city_name: &'static str,
    city_x: f32,
    city_z: f32,
    snap_uid: Option<u64>,
    snap_dist_m: Option<f32>,
    roads_referencing_snap: usize,
    sector_drops_within_5km: usize,
    graph_drops_within_5km: usize,
    diagnosis: String,
}

fn compute_focus(
    city_name: &'static str,
    city_x: f32,
    city_z: f32,
    builder: &truckpilot_map_parser::GraphBuilder,
    sector_events: &[DropEvent],
    graph_events: &[DropEvent],
) -> Option<FocusResult> {
    let snap_radius = 5000.0_f32;

    let (snap_uid, snap_dist_m) = match builder.find_nearest_node(city_x, city_z, snap_radius) {
        Some((uid, d)) => (Some(uid), Some(d)),
        None => (None, None),
    };

    let roads_referencing_snap = snap_uid.map_or(0, |uid| builder.roads_referencing_node(uid));

    // Count drop events within 5 km of city centre
    let sector_drops_within_5km = sector_events
        .iter()
        .filter(|ev| {
            if let (Some(x), Some(z)) = (ev.x, ev.z) {
                dist_xz(x, z, city_x, city_z) < snap_radius
            } else {
                false
            }
        })
        .count();

    let graph_drops_within_5km = graph_events
        .iter()
        .filter(|ev| {
            if let (Some(x), Some(z)) = (ev.x, ev.z) {
                dist_xz(x, z, city_x, city_z) < snap_radius
            } else {
                false
            }
        })
        .count();

    let diagnosis = if let Some(uid) = snap_uid {
        let dist = snap_dist_m.unwrap_or(0.0);
        if roads_referencing_snap == 0 {
            format!(
                "Snap-node {uid} found at {dist:.1}m but has ZERO road references in the parsed graph. \
                 Roads are being dropped UPSTREAM — either at sector parse time (RoadParseFailed / \
                 SectorHandlerError) or the node belongs to a sector whose roads reference a different node set."
            )
        } else {
            format!(
                "Snap-node {uid} found at {dist:.1}m with {roads_referencing_snap} road references in parsed graph. \
                 Drops are at GRAPH level (BothUnresolved / OneUnresolved) — the roads parsed OK \
                 but their endpoint nodes are not in the merged node map. \
                 Likely cause: cross-sector node UIDs reference nodes from neighbouring sectors \
                 that loaded under a different UID or were not included in the parse."
            )
        }
    } else {
        format!(
            "NO snap-node found within {snap_radius}m of {city_name} ({city_x}, {city_z}). \
             This city region may not be covered by any parsed sector, or all nearby sectors failed to parse."
        )
    };

    Some(FocusResult {
        city_name,
        city_x,
        city_z,
        snap_uid,
        snap_dist_m,
        roads_referencing_snap,
        sector_drops_within_5km,
        graph_drops_within_5km,
        diagnosis,
    })
}

// ---------------------------------------------------------------------------
// Markdown writers
// ---------------------------------------------------------------------------

#[allow(clippy::too_many_arguments)]
fn write_markdown(
    path: &std::path::Path,
    _by_category: &HashMap<String, usize>,
    by_sector: &HashMap<&str, usize>,
    by_item_type: &HashMap<u32, usize>,
    road_parse_failed: usize,
    handler_error: usize,
    unknown_type: usize,
    both_unresolved: usize,
    one_unresolved: usize,
    fully_resolved: usize,
    total_roads: usize,
    total_nodes: usize,
    sector_events: &[DropEvent],
    focus: &Option<FocusResult>,
    focus_sector: Option<&str>,
) -> Result<()> {
    let mut f = std::fs::File::create(path)?;
    macro_rules! w {
        ($($arg:tt)*) => { writeln!(f, $($arg)*)?; }
    }

    w!("# Road-Drop-Audit — 2026-05-16");
    w!();
    w!("## Overview");
    w!();
    w!("| Metric | Value |");
    w!("|---|---|");
    w!("| Total roads in parsed graph | {} |", total_roads);
    w!("| Total nodes in merged node map | {} |", total_nodes);
    w!("| Fully resolved roads (both endpoints in node map) | {} ({:.1}%) |",
        fully_resolved,
        pct(fully_resolved, total_roads));
    w!();
    w!("## Drop Summary by Category");
    w!();
    w!("| Category | Layer | Count | % of roads |");
    w!("|---|---|---|---|");
    w!("| RoadParseFailed | Sector | {} | {:.2}% |",
        road_parse_failed, pct(road_parse_failed, total_roads));
    w!("| SectorHandlerError | Sector | {} | — |", handler_error);
    w!("| UnknownItemType | Sector | {} | — |", unknown_type);
    w!("| BothUnresolved | Graph | {} | {:.2}% |",
        both_unresolved, pct(both_unresolved, total_roads));
    w!("| OneUnresolved | Graph | {} | {:.2}% |",
        one_unresolved, pct(one_unresolved, total_roads));
    w!();
    w!("> **Note:** SectorHandlerError/UnknownItemType drop *all subsequent items* in that sector,");
    w!("> not just roads. Their count is sectors aborted, not roads dropped directly.");
    w!();

    // Item-type histogram for non-road drops
    if !by_item_type.is_empty() {
        w!("## Item-Type Histogram (SectorHandlerError + UnknownItemType)");
        w!();
        w!("| item_type (decimal) | Count |");
        w!("|---|---|");
        let mut sorted: Vec<_> = by_item_type.iter().collect();
        sorted.sort_by_key(|(_, &c)| std::cmp::Reverse(c));
        for (t, c) in &sorted {
            w!("| {} | {} |", t, c);
        }
        w!();
    }

    // Top-20 sectors by drop count
    {
        w!("## Top-20 Sectors by Drop Count");
        w!();
        w!("| Sector | Drops |");
        w!("|---|---|");
        let mut sorted: Vec<_> = by_sector.iter().collect();
        sorted.sort_by_key(|(_, &c)| std::cmp::Reverse(c));
        for (sec, c) in sorted.iter().take(20) {
            w!("| `{}` | {} |", sec, c);
        }
        w!();
    }

    // RoadParseFailed hex-dump sample (first 10)
    let road_fails: Vec<&DropEvent> = sector_events
        .iter()
        .filter(|e| e.category == DropCategory::RoadParseFailed)
        .take(10)
        .collect();
    if !road_fails.is_empty() {
        w!("## RoadParseFailed — First {} Samples", road_fails.len());
        w!();
        for (i, ev) in road_fails.iter().enumerate() {
            w!("### Sample {}", i + 1);
            w!("- **Sector:** `{}`", ev.sector_path);
            if let Some(uid) = ev.item_uid { w!("- **Road UID:** {uid:#018x}"); }
            if let Some(a) = ev.node_a { w!("- **node_a:** {a:#018x}"); }
            if let Some(b) = ev.node_b { w!("- **node_b:** {b:#018x}"); }
            if !ev.raw_hex.is_empty() {
                w!("- **Raw bytes ({}B):** `{}`", ev.raw_hex.len(), hex_dump(&ev.raw_hex));
            }
            w!();
        }
    }

    // Focus-sector detail
    if let Some(fsec) = focus_sector {
        let fsec_events: Vec<&DropEvent> = sector_events
            .iter()
            .filter(|e| e.sector_path.contains(fsec))
            .collect();
        if !fsec_events.is_empty() {
            w!("## Focus-Sector `{}` — {} drop events", fsec, fsec_events.len());
            w!();
            for ev in &fsec_events {
                w!("- `{}` item_type={} uid={:?} node_a={:?} node_b={:?}",
                    category_label(&ev.category),
                    ev.item_type,
                    ev.item_uid.map(|u| format!("{u:#018x}")),
                    ev.node_a.map(|u| format!("{u:#018x}")),
                    ev.node_b.map(|u| format!("{u:#018x}")));
            }
            w!();
        }
    }

    // Focus-city summary
    if let Some(fr) = focus {
        w!("## Focus-City: {}", fr.city_name);
        w!();
        w!("| Field | Value |");
        w!("|---|---|");
        w!("| City XZ | ({:.0}, {:.0}) |", fr.city_x, fr.city_z);
        match (fr.snap_uid, fr.snap_dist_m) {
            (Some(uid), Some(d)) => {
                w!("| Snap node UID | {uid:#018x} |");
                w!("| Snap distance | {d:.1} m |");
            }
            _ => { w!("| Snap node | NOT FOUND within 5000 m |"); }
        }
        w!("| Roads referencing snap node | {} |", fr.roads_referencing_snap);
        w!("| Sector-level drops within 5 km | {} |", fr.sector_drops_within_5km);
        w!("| Graph-level drops within 5 km | {} |", fr.graph_drops_within_5km);
        w!();
        w!("### Positive-Check Result");
        w!();
        w!("{}", fr.diagnosis);
        w!();
    }

    // Auto-diagnosis
    w!("## Auto-Diagnosis");
    w!();
    write_auto_diagnosis(&mut f, road_parse_failed, handler_error, unknown_type, both_unresolved, one_unresolved, total_roads)?;

    Ok(())
}

fn write_auto_diagnosis(
    f: &mut std::fs::File,
    road_parse_failed: usize,
    handler_error: usize,
    unknown_type: usize,
    both_unresolved: usize,
    one_unresolved: usize,
    total_roads: usize,
) -> Result<()> {
    macro_rules! w {
        ($($arg:tt)*) => { writeln!(f, $($arg)*)?; }
    }

    if both_unresolved == 0 && one_unresolved == 0 && road_parse_failed == 0 {
        w!("All roads fully resolved — graph connectivity is intact at the road-parser level.");
        w!("If routing still fails, the issue lies in edge direction, DLC guards, or prefab connectivity.");
        return Ok(());
    }

    if both_unresolved > 0 {
        let pct = pct(both_unresolved, total_roads);
        if pct > 30.0 {
            w!("**[HIGH]** {:.1}% of roads have BOTH endpoints unresolved. This is the dominant drop source.", pct);
            w!("Root cause hypothesis: roads in sector A reference nodes from sector B,");
            w!("but sector B's nodes are loaded into the map under different UIDs (or sector B failed to parse).");
            w!("Next step: check whether the missing node UIDs appear in any loaded sector at all.");
        } else {
            w!("**[MEDIUM]** {:.1}% of roads are BothUnresolved.", pct);
        }
        w!();
    }

    if one_unresolved > 0 {
        w!("**[INFO]** {} OneUnresolved roads (one node found, one missing).", one_unresolved);
        w!("These are candidates for spatial matching — if nodes cluster within 50 m they");
        w!("should resolve via Pass-1 strict spatial match in the production graph builder.");
        w!();
    }

    if road_parse_failed > 0 {
        w!("**[WARN]** {} RoadParseFailed events — roads dropped at binrw parse time.", road_parse_failed);
        w!("Check the RoadParseFailed sample section above for raw bytes and cursor position.");
        w!("Possible causes: wrong RoadFixedHeader layout assumption, v907 variation, partial sector.");
        w!();
    }

    if handler_error > 0 || unknown_type > 0 {
        w!("**[INFO]** {handler_error} SectorHandlerError + {unknown_type} UnknownItemType events.");
        w!("Each aborts the entire item stream for that sector — all subsequent road items in");
        w!("the sector are lost. Check the Item-Type Histogram to identify the triggering type.");
        w!();
    }

    Ok(())
}

fn write_city_trace(
    path: &std::path::Path,
    city_name: &str,
    fr: &FocusResult,
    sector_events: &[DropEvent],
    graph_events: &[DropEvent],
) -> Result<()> {
    let mut f = std::fs::File::create(path)?;
    macro_rules! w {
        ($($arg:tt)*) => { writeln!(f, $($arg)*)?; }
    }

    w!("# Road-Drop Trace — {city_name}");
    w!();
    w!("City XZ: ({:.0}, {:.0})", fr.city_x, fr.city_z);
    w!();
    w!("## Snap-Node Summary");
    w!();
    match (fr.snap_uid, fr.snap_dist_m) {
        (Some(uid), Some(d)) => {
            w!("- Snap node UID: `{uid:#018x}`");
            w!("- Snap distance: {d:.1} m");
            w!("- Roads referencing snap node in parsed graph: **{}**", fr.roads_referencing_snap);
        }
        _ => {
            w!("- **No snap-node found** within 5000 m of city centre.");
        }
    }
    w!();
    w!("## Positive-Check Diagnosis");
    w!();
    w!("{}", fr.diagnosis);
    w!();

    // Sector-level events near city (within 5 km)
    let nearby_sector: Vec<&DropEvent> = sector_events
        .iter()
        .filter(|ev| {
            if let (Some(x), Some(z)) = (ev.x, ev.z) {
                dist_xz(x, z, fr.city_x, fr.city_z) < 5000.0
            } else {
                false
            }
        })
        .collect();

    if nearby_sector.is_empty() {
        w!("## Sector-Level Events Near City");
        w!();
        w!("No sector-level drop events with position data within 5 km of city centre.");
        w!("(Events without position data — e.g. SectorHandlerError from cursor desync — are not");
        w!("included here because their XZ is unknown.)");
    } else {
        w!("## Sector-Level Events Near City ({} events within 5 km)", nearby_sector.len());
        w!();
        w!("| Category | Sector | UID | node_a | node_b |");
        w!("|---|---|---|---|---|");
        for ev in nearby_sector.iter().take(50) {
            w!("| {} | `{}` | {} | {} | {} |",
                category_label(&ev.category),
                short_sector(&ev.sector_path),
                opt_hex(ev.item_uid),
                opt_hex(ev.node_a),
                opt_hex(ev.node_b));
        }
        if nearby_sector.len() > 50 {
            w!();
            w!("*(showing first 50 of {})*", nearby_sector.len());
        }
    }
    w!();

    // Graph-level events near city
    let nearby_graph: Vec<&DropEvent> = graph_events
        .iter()
        .filter(|ev| {
            if let (Some(x), Some(z)) = (ev.x, ev.z) {
                dist_xz(x, z, fr.city_x, fr.city_z) < 5000.0
            } else {
                false
            }
        })
        .collect();

    if nearby_graph.is_empty() {
        w!("## Graph-Level Events Near City");
        w!();
        w!("No OneUnresolved events with position data within 5 km.");
        w!("BothUnresolved events have no position (both nodes missing) and are not shown here.");
    } else {
        w!("## Graph-Level Events Near City ({} OneUnresolved within 5 km)", nearby_graph.len());
        w!();
        w!("| Category | Road UID | node_a | node_b | a_resolved | b_resolved | x | z |");
        w!("|---|---|---|---|---|---|---|---|");
        for ev in nearby_graph.iter().take(50) {
            w!("| {} | {} | {} | {} | {} | {} | {:.0} | {:.0} |",
                category_label(&ev.category),
                opt_hex(ev.item_uid),
                opt_hex(ev.node_a),
                opt_hex(ev.node_b),
                ev.node_a_resolved.map_or("-", |v| if v { "yes" } else { "no" }),
                ev.node_b_resolved.map_or("-", |v| if v { "yes" } else { "no" }),
                ev.x.unwrap_or(0.0),
                ev.z.unwrap_or(0.0));
        }
        if nearby_graph.len() > 50 {
            w!();
            w!("*(showing first 50 of {})*", nearby_graph.len());
        }
    }
    w!();

    Ok(())
}

// ---------------------------------------------------------------------------
// Formatting helpers
// ---------------------------------------------------------------------------

fn pct(n: usize, total: usize) -> f64 {
    if total == 0 { 0.0 } else { 100.0 * n as f64 / total as f64 }
}

fn opt_hex(v: Option<u64>) -> String {
    v.map_or("-".to_string(), |u| format!("{u:#018x}"))
}

fn short_sector(path: &str) -> &str {
    path.rsplit('/').next().unwrap_or(path)
}

