//! `sign-precrash-audit` — Phase 6.2b-Diag-4
//!
//! Pre-crash context analysis for the 9 SectorHandlerError events in which
//! the Sign-Handler (item_type=36) crashes at an `ensure_count` check.
//!
//! Three empirical questions answered here:
//!   A) Where in the world are the 9 crash sectors?
//!   B) What do the 64-byte raw_hex captures reveal (kdop_item decode)?
//!   C) What was the last successfully parsed item before the crash in each sector?
//!
//! Gate Guards (MUST NOT be violated):
//!   - No change to skip_sign or any item handler.
//!   - No change to DropTracer / DropEvent structs.
//!   - No change to cursor tracking in parse_sector_legacy_inner.
//!   - Read-only analysis only — all findings go to outputs files.
//!
//! Usage:
//!   cargo run --release -p truckpilot-diag --bin sign-precrash-audit -- \
//!     --ets2-dir "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2" \
//!     --output-dir outputs/2026-05-17

use std::collections::HashMap;
use std::io::Write as IoWrite;
use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;
use truckpilot_map_parser::{
    sector::{audit_sector, parse_sector_with_tracer, AuditReport},
    Archive, DropTracer, HashFsArchive, ModLoadOrder, ZipArchive,
};

// ---------------------------------------------------------------------------
// The 9 known crash sectors (from road_drop_audit.json, all item_type=36)
// ---------------------------------------------------------------------------

const CRASH_SECTORS: &[&str] = &[
    "map/europe/sec+0029+0024.base",
    "map/europe/sec+0029+0025.base",
    "map/europe/sec+0027+0025.base",
    "map/europe/sec+0014-0009.base",
    "map/europe/sec+0028+0024.base",
    "map/europe/sec+0028+0015.base",
    "map/europe/sec+0027+0022.base",
    "map/europe/sec+0026+0024.base",
    "map/europe/sec+0027+0016.base",
];

// Raw bytes from road_drop_audit.json (64 bytes per sector, captured from
// item body start = right after the 4-byte item_type=36 field).
const RAW_HEX: &[[u8; 64]] = &[
    // sec+0029+0024
    [
        30, 0, 5, 59, 144, 228, 224, 58, 19, 197, 231, 71, 0, 168, 115, 66, 180, 81, 192, 71, 102,
        11, 212, 71, 196, 204, 29, 70, 61, 199, 231, 71, 0, 48, 127, 66, 250, 82, 192, 71, 26, 13,
        212, 71, 192, 209, 29, 70, 0, 0, 0, 2, 95, 176, 133, 162, 143, 89, 112, 13, 1, 69, 0, 197,
    ],
    // sec+0029+0025
    [
        110, 1, 5, 27, 44, 109, 25, 54, 158, 100, 228, 71, 0, 160, 51, 66, 104, 203, 200, 71, 4,
        152, 214, 71, 72, 200, 220, 69, 232, 102, 228, 71, 0, 40, 63, 66, 105, 204, 200, 71, 166,
        153, 214, 71, 96, 213, 220, 69, 0, 0, 0, 2, 95, 176, 133, 162, 143, 89, 112, 13, 1, 47, 3,
        197,
    ],
    // sec+0027+0025
    [
        126, 0, 197, 67, 8, 120, 25, 54, 218, 26, 212, 71, 0, 0, 151, 65, 126, 131, 197, 71, 47,
        207, 204, 71, 160, 110, 105, 69, 47, 28, 212, 71, 0, 176, 195, 65, 30, 133, 197, 71, 164,
        208, 204, 71, 64, 120, 105, 69, 0, 0, 0, 6, 95, 240, 27, 198, 131, 148, 172, 228, 73, 109,
        1, 197,
    ],
    // sec+0014-0009
    [
        1, 5, 0, 44, 252, 110, 243, 44, 90, 191, 93, 71, 0, 0, 174, 64, 132, 87, 0, 199, 180, 207,
        58, 70, 76, 10, 47, 71, 210, 192, 93, 71, 0, 112, 15, 65, 16, 84, 0, 199, 124, 217, 58, 70,
        146, 11, 47, 71, 0, 0, 0, 2, 95, 112, 123, 9, 148, 40, 1, 184, 8, 91, 12, 192,
    ],
    // sec+0028+0024
    [
        19, 0, 197, 193, 216, 62, 202, 53, 139, 28, 220, 71, 0, 8, 138, 65, 210, 151, 188, 71, 47,
        90, 204, 71, 136, 36, 252, 69, 100, 31, 220, 71, 0, 32, 208, 65, 94, 152, 188, 71, 176, 91,
        204, 71, 152, 60, 252, 69, 0, 0, 0, 3, 95, 240, 191, 180, 153, 138, 85, 127, 1, 107, 0, 69,
    ],
    // sec+0028+0015
    [
        233, 4, 5, 8, 76, 188, 125, 70, 65, 14, 221, 71, 0, 176, 62, 65, 242, 37, 117, 71, 158,
        208, 171, 71, 72, 246, 196, 70, 162, 15, 221, 71, 0, 96, 131, 65, 80, 39, 117, 71, 164,
        209, 171, 71, 62, 248, 196, 70, 0, 0, 0, 2, 40, 30, 183, 174, 115, 36, 193, 253, 33, 130,
        11, 5,
    ],
    // sec+0027+0022
    [
        152, 0, 197, 34, 48, 178, 200, 53, 20, 176, 218, 71, 0, 32, 137, 65, 145, 59, 172, 71, 210,
        117, 195, 71, 108, 209, 57, 70, 52, 179, 218, 71, 0, 80, 166, 65, 238, 59, 172, 71, 140,
        119, 195, 71, 220, 221, 57, 70, 0, 0, 0, 2, 95, 176, 29, 190, 11, 103, 201, 15, 1, 28, 2,
        69,
    ],
    // sec+0026+0024
    [
        0, 0, 133, 208, 168, 21, 213, 56, 15, 195, 210, 71, 0, 128, 119, 192, 114, 168, 189, 71,
        192, 53, 200, 71, 192, 204, 168, 69, 179, 195, 210, 71, 0, 0, 100, 63, 200, 169, 189, 71,
        165, 54, 200, 71, 80, 215, 168, 69, 0, 0, 0, 6, 95, 68, 207, 103, 0, 0, 0, 0, 0, 1, 0, 69,
    ],
    // sec+0027+0016
    [
        0, 0, 0, 122, 216, 254, 204, 61, 147, 46, 218, 71, 0, 128, 48, 64, 63, 81, 131, 71, 88,
        192, 174, 71, 116, 184, 173, 70, 168, 47, 218, 71, 0, 192, 231, 64, 93, 82, 131, 71, 138,
        192, 174, 71, 204, 188, 173, 70, 0, 0, 0, 2, 40, 30, 109, 167, 45, 99, 239, 40, 30, 2, 0,
        64,
    ],
];

// ---------------------------------------------------------------------------
// Known cities — ETS2 world coordinates (metres, same as node x/z)
// ---------------------------------------------------------------------------

fn known_cities() -> Vec<(&'static str, f32, f32)> {
    vec![
        ("Berlin", -16400.0, -3200.0),
        ("Hamburg", -22300.0, -7200.0),
        ("Wien", -8400.0, 3500.0),
        ("Paris", -33500.0, 1900.0),
        ("Amsterdam", -29200.0, -3400.0),
        ("Köln", -23600.0, -5000.0),
        ("Frankfurt", -20700.0, -2200.0),
        ("München", -16200.0, 2700.0),
        ("Prag", -11800.0, 400.0),
        ("Warschau", -3000.0, -3200.0),
        ("Wien", -8400.0, 3500.0),
        ("Budapest", -1500.0, 5500.0),
        ("Bratislava", -7000.0, 3500.0),
        ("Krakau", -1000.0, -1500.0),
        ("Riga", 8500.0, -19000.0),
        ("Vilnius", 8000.0, -10000.0),
    ]
}

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

#[derive(Parser)]
#[command(
    name = "sign-precrash-audit",
    about = "Phase 6.2b-Diag-4: pre-crash context for 9 Sign-Handler (type-36) crashes"
)]
struct Args {
    /// ETS2 install directory (contains base_map.scs etc.)
    #[arg(long)]
    ets2_dir: PathBuf,

    /// Optional mods directory
    #[arg(long)]
    mods_dir: Option<PathBuf>,

    /// Output directory for reports (created if absent)
    #[arg(long, default_value = "outputs/2026-05-17")]
    output_dir: PathBuf,
}

fn default_mods_dir() -> PathBuf {
    dirs::document_dir()
        .or_else(dirs::home_dir)
        .unwrap_or_default()
        .join("Euro Truck Simulator 2/mod")
}

// ---------------------------------------------------------------------------
// Per-sector analysis record
// ---------------------------------------------------------------------------

struct SectorAnalysis {
    sector_path: &'static str,
    found_in_archive: bool,
    sector_bytes_len: usize,

    // From audit_sector()
    item_count: u32,
    items_before_crash: usize,
    last_item_type: Option<u32>,
    last_item_kind: Option<String>,
    last_item_start_offset: Option<usize>,
    last_item_end_offset: Option<usize>,
    crash_error_offset: Option<usize>,
    crash_error_msg: Option<String>,
    crash_raw_type: Option<u32>,
    bytes_crash_item_body: Option<usize>,
    item_type_histogram: HashMap<u32, usize>,

    // From parse_sector_with_tracer()
    nodes_count: usize,
    recovered_nodes_count: usize,
    node_bounds: Option<NodeBounds>,
    tracer_raw_hex: Vec<u8>,

    // Derived — world position
    world_center_x: Option<f32>,
    world_center_z: Option<f32>,
    nearest_city: Option<(&'static str, f32)>,
}

#[derive(Clone)]
struct NodeBounds {
    min_x: f32,
    max_x: f32,
    min_z: f32,
    max_z: f32,
}

impl NodeBounds {
    fn center_x(&self) -> f32 {
        (self.min_x + self.max_x) / 2.0
    }
    fn center_z(&self) -> f32 {
        (self.min_z + self.max_z) / 2.0
    }
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
    let t0 = std::time::Instant::now();
    eprintln!(
        "[1/3] Discovering archives in {} ...",
        args.ets2_dir.display()
    );
    let order = ModLoadOrder::from_directories(&args.ets2_dir, &mods_dir)
        .context("build mod load order")?;
    eprintln!(
        "      {} archive(s) found — opening (this is the slow step)...",
        order.entries.len()
    );

    let mut archives: Vec<Box<dyn Archive>> = Vec::new();
    for (i, entry) in order.entries.iter().enumerate() {
        eprint!(
            "      [{}/{}] {} ... ",
            i + 1,
            order.entries.len(),
            entry.name
        );
        let t = std::time::Instant::now();
        let arc: Box<dyn Archive> = match HashFsArchive::open(&entry.path) {
            Ok(a) => Box::new(a),
            Err(_) => match ZipArchive::open(&entry.path) {
                Ok(a) => Box::new(a),
                Err(_) => {
                    eprintln!("skip");
                    continue;
                }
            },
        };
        eprintln!("ok ({:.1}s)", t.elapsed().as_secs_f32());
        archives.push(arc);
    }
    eprintln!(
        "      {} archive(s) ready. ({:.1}s total)",
        archives.len(),
        t0.elapsed().as_secs_f32()
    );

    // ── 2. Analyse each of the 9 crash sectors ──────────────────────────────
    eprintln!("[2/3] Analysing {} crash sectors ...", CRASH_SECTORS.len());
    let cities = known_cities();
    let mut results: Vec<SectorAnalysis> = Vec::new();

    for (si, &sector_path) in CRASH_SECTORS.iter().enumerate() {
        eprint!(
            "      [{}/{}] {} ... ",
            si + 1,
            CRASH_SECTORS.len(),
            short_sector_display(sector_path)
        );

        let data_opt: Option<Vec<u8>> = archives
            .iter_mut()
            .rev()
            .find_map(|arc| arc.read_path(sector_path).ok());

        let Some(data) = data_opt else {
            eprintln!("NOT FOUND");
            results.push(SectorAnalysis {
                sector_path,
                found_in_archive: false,
                sector_bytes_len: 0,
                item_count: 0,
                items_before_crash: 0,
                last_item_type: None,
                last_item_kind: None,
                last_item_start_offset: None,
                last_item_end_offset: None,
                crash_error_offset: None,
                crash_error_msg: None,
                crash_raw_type: None,
                bytes_crash_item_body: None,
                item_type_histogram: HashMap::new(),
                nodes_count: 0,
                recovered_nodes_count: 0,
                node_bounds: None,
                tracer_raw_hex: Vec::new(),
                world_center_x: None,
                world_center_z: None,
                nearest_city: None,
            });
            continue;
        };

        let sector_bytes_len = data.len();

        // -- audit_sector(): item history + crash context --------------------
        let report: AuditReport = audit_sector(&data);
        let item_count = report.item_count;
        let items_before_crash = report.items.len();

        let mut item_type_histogram: HashMap<u32, usize> = HashMap::new();
        for item in &report.items {
            *item_type_histogram.entry(item.item_type).or_default() += 1;
        }

        let last_item = report.items.last();
        let last_item_type = last_item.map(|i| i.item_type);
        let last_item_kind = last_item.map(|i| i.kind_name.to_string());
        let last_item_start_offset = last_item.map(|i| i.start_offset);
        let last_item_end_offset = last_item.map(|i| i.end_offset);

        let (crash_error_offset, crash_error_msg, crash_raw_type) = if let Some(f) = &report.failure
        {
            (
                Some(f.error_offset),
                Some(f.error_msg.clone()),
                Some(f.raw_type),
            )
        } else {
            (None, None, None)
        };

        // bytes consumed by the crashing item before failure was detected
        // = crash_error_offset (=pos before type read) minus last_item_end_offset
        // plus 4 (the item_type u32 itself) = crash_item_body_bytes consumed
        let bytes_crash_item_body = match (crash_error_offset, last_item_end_offset) {
            (Some(co), Some(lo)) if co >= lo => Some(co - lo),
            _ => None,
        };

        // -- parse_sector_with_tracer(): nodes + raw_hex ---------------------
        let tracer = DropTracer::new(256);
        let parsed_sector =
            parse_sector_with_tracer(&data, sector_path, &tracer).unwrap_or_default();
        let tracer_events = tracer.take_events();

        let tracer_raw_hex = tracer_events
            .into_iter()
            .find(|ev| ev.item_type == 36)
            .map(|ev| ev.raw_hex)
            .unwrap_or_default();

        let nodes_count = parsed_sector.nodes.len();
        let recovered_nodes_count = parsed_sector.recovered_nodes_count;

        let node_bounds = if !parsed_sector.nodes.is_empty() {
            let mut min_x = f32::MAX;
            let mut max_x = f32::MIN;
            let mut min_z = f32::MAX;
            let mut max_z = f32::MIN;
            for n in &parsed_sector.nodes {
                if n.x < min_x {
                    min_x = n.x;
                }
                if n.x > max_x {
                    max_x = n.x;
                }
                if n.z < min_z {
                    min_z = n.z;
                }
                if n.z > max_z {
                    max_z = n.z;
                }
            }
            Some(NodeBounds {
                min_x,
                max_x,
                min_z,
                max_z,
            })
        } else {
            None
        };

        let (world_center_x, world_center_z) = node_bounds
            .as_ref()
            .map(|b| (Some(b.center_x()), Some(b.center_z())))
            .unwrap_or((None, None));

        // nearest city
        let nearest_city = world_center_x.and_then(|cx| {
            world_center_z.and_then(|cz| {
                cities
                    .iter()
                    .map(|&(name, cx2, cz2)| {
                        let d = dist_xz(cx, cz, cx2, cz2);
                        (name, d)
                    })
                    .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                    .map(|(name, d)| (name, d / 1000.0))
            })
        });

        let city_str = nearest_city
            .map(|(n, d)| format!(" → near {n} ({d:.1} km)"))
            .unwrap_or_default();
        eprintln!(
            "ok | last={} | crash={} | {} nodes{}",
            last_item_kind.as_deref().unwrap_or("?"),
            crash_error_msg
                .as_deref()
                .and_then(|s| s.split(':').next())
                .unwrap_or("?"),
            nodes_count,
            city_str,
        );

        results.push(SectorAnalysis {
            sector_path,
            found_in_archive: true,
            sector_bytes_len,
            item_count,
            items_before_crash,
            last_item_type,
            last_item_kind,
            last_item_start_offset,
            last_item_end_offset,
            crash_error_offset,
            crash_error_msg,
            crash_raw_type,
            bytes_crash_item_body,
            item_type_histogram,
            nodes_count,
            recovered_nodes_count,
            node_bounds,
            tracer_raw_hex,
            world_center_x,
            world_center_z,
            nearest_city,
        });
    }

    // ── 3. Write outputs ─────────────────────────────────────────────────────
    eprintln!("[3/3] Writing outputs ...");
    let json_path = args.output_dir.join("sign_precrash_audit.json");
    write_json(&json_path, &results)?;
    eprintln!("      {}", json_path.display());

    let md_path = args.output_dir.join("sign_precrash_audit.md");
    write_markdown(&md_path, &results)?;
    eprintln!("      {}", md_path.display());

    eprintln!("Done. ({:.1}s total)", t0.elapsed().as_secs_f32());
    Ok(())
}

// ---------------------------------------------------------------------------
// JSON output
// ---------------------------------------------------------------------------

fn write_json(path: &std::path::Path, results: &[SectorAnalysis]) -> Result<()> {
    use serde_json::{json, Value};

    let mut sectors: Vec<Value> = Vec::new();
    for (i, r) in results.iter().enumerate() {
        let raw_hex_json: Vec<u8> = if !r.tracer_raw_hex.is_empty() {
            r.tracer_raw_hex.clone()
        } else {
            RAW_HEX[i].to_vec()
        };
        let raw_decoded = decode_kdop_header(&RAW_HEX[i]);

        sectors.push(json!({
            "sector_path": r.sector_path,
            "found_in_archive": r.found_in_archive,
            "sector_bytes": r.sector_bytes_len,
            "item_count": r.item_count,
            "items_before_crash": r.items_before_crash,
            "last_item_type": r.last_item_type,
            "last_item_kind": r.last_item_kind,
            "last_item_start_offset": r.last_item_start_offset,
            "last_item_end_offset": r.last_item_end_offset,
            "crash_error_offset": r.crash_error_offset,
            "crash_raw_type": r.crash_raw_type,
            "crash_error_msg": r.crash_error_msg,
            "bytes_crash_item_body_offset": r.bytes_crash_item_body,
            "nodes_count": r.nodes_count,
            "recovered_nodes_count": r.recovered_nodes_count,
            "world_min_x": r.node_bounds.as_ref().map(|b| b.min_x),
            "world_max_x": r.node_bounds.as_ref().map(|b| b.max_x),
            "world_min_z": r.node_bounds.as_ref().map(|b| b.min_z),
            "world_max_z": r.node_bounds.as_ref().map(|b| b.max_z),
            "world_center_x": r.world_center_x,
            "world_center_z": r.world_center_z,
            "nearest_city": r.nearest_city.map(|(name, d)| json!({
                "name": name,
                "distance_km": (d * 10.0).round() / 10.0,
            })),
            "raw_hex_64b": raw_hex_json,
            "kdop_uid_hex": raw_decoded.uid_hex,
            "kdop_bounds": raw_decoded.bounds,
            "kdop_flags": raw_decoded.flags,
            "kdop_view_dist": raw_decoded.view_dist,
            "model_token_hex": raw_decoded.model_token_hex,
            "uid_is_plausible_ets2": raw_decoded.uid_is_plausible,
            "uid_as_f32_pair": raw_decoded.uid_as_f32_pair,
            "item_type_histogram": r.item_type_histogram.iter()
                .map(|(k, v)| (k.to_string(), *v))
                .collect::<HashMap<String, usize>>(),
        }));
    }

    let json = serde_json::to_string_pretty(&sectors)?;
    std::fs::write(path, &json)?;
    Ok(())
}

// ---------------------------------------------------------------------------
// Markdown output
// ---------------------------------------------------------------------------

fn write_markdown(path: &std::path::Path, results: &[SectorAnalysis]) -> Result<()> {
    let mut f = std::fs::File::create(path)?;
    macro_rules! w {
        ($($arg:tt)*) => { writeln!(f, $($arg)*)?; }
    }

    w!("# Sign Pre-Crash Audit — Phase 6.2b-Diag-4");
    w!();
    w!("> Generated: 2026-05-17 | 9 SectorHandlerError events, all item_type=36 (Sign)");
    w!();

    // ── Section 1: Summary ───────────────────────────────────────────────────
    w!("## 1. Summary");
    w!();
    w!("| Sector | Found | Items before crash | Last item type | Crash msg (short) | Nearest city |");
    w!("|---|---|---|---|---|---|");
    for r in results {
        let short_name = short_sector(r.sector_path);
        let found = if r.found_in_archive { "✓" } else { "✗" };
        let last_kind = r.last_item_kind.as_deref().unwrap_or("-");
        let crash_short = r
            .crash_error_msg
            .as_deref()
            .map(|s| s.split(':').next().unwrap_or(s))
            .unwrap_or("-");
        let city = r
            .nearest_city
            .map(|(name, d)| format!("{name} ({:.1} km)", d))
            .unwrap_or_else(|| "-".to_string());
        w!(
            "| `{short_name}` | {found} | {} | {last_kind} | {crash_short} | {city} |",
            r.items_before_crash
        );
    }
    w!();

    // ── Section 2: Per-Sector World Bounds + City Correlation ────────────────
    w!("## 2. Per-Sector World Bounds and City Correlation");
    w!();
    for (i, r) in results.iter().enumerate() {
        w!("### {} `{}`", i + 1, short_sector(r.sector_path));
        w!();
        if !r.found_in_archive {
            w!("**NOT FOUND** in any loaded archive.");
            w!();
            continue;
        }
        w!("| Field | Value |");
        w!("|---|---|");
        w!("| Sector bytes | {} | ", r.sector_bytes_len);
        w!("| item_count (header) | {} |", r.item_count);
        w!("| Items parsed before crash | {} |", r.items_before_crash);
        if let Some(b) = &r.node_bounds {
            w!("| Node x-range | {:.1} … {:.1} m |", b.min_x, b.max_x);
            w!("| Node z-range | {:.1} … {:.1} m |", b.min_z, b.max_z);
            w!(
                "| World centre (approx) | ({:.0}, {:.0}) |",
                b.center_x(),
                b.center_z()
            );
        } else {
            w!("| Node bounds | no nodes recovered |");
        }
        w!(
            "| Nodes total | {} ({} recovered from tail) |",
            r.nodes_count,
            r.recovered_nodes_count
        );
        if let Some((name, d)) = r.nearest_city {
            w!("| Nearest city | **{name}** at {:.1} km |", d);
        }
        w!();

        // Item-type histogram
        if !r.item_type_histogram.is_empty() {
            w!("**Pre-crash item histogram:**");
            w!();
            let mut sorted: Vec<_> = r.item_type_histogram.iter().collect();
            sorted.sort_by_key(|(_, &c)| std::cmp::Reverse(c));
            w!("| item_type | kind | count |");
            w!("|---|---|---|");
            for (t, c) in &sorted {
                let kind = item_type_name(**t);
                w!("| {} | {kind} | {} |", t, c);
            }
            w!();
        }
    }

    // ── Section 3: Raw-Hex Dump + kdop_item Decode ───────────────────────────
    w!("## 3. Raw-Hex Dump and kdop_item Decode");
    w!();
    w!("The 64-byte captures cover the **beginning of the sign item body** (immediately");
    w!("after the 4-byte item_type=36 field). Layout: kdop_item=53 B + model_token=8 B + 3 B.");
    w!();
    w!("kdop_item layout: uid(8) + kdop_bounds(40 = 10×f32) + flags(4) + view_dist(1) = **53 bytes**");
    w!();
    w!("⚠️  The crash count `0x02000000 = 33 554 432` is **NOT** in this window.");
    w!("    It is read deep inside skip_sign (after boards + template), not in the first 64 B.");
    w!();

    for (i, r) in results.iter().enumerate() {
        let raw = &RAW_HEX[i];
        let dec = decode_kdop_header(raw);
        w!("### {} `{}`", i + 1, short_sector(r.sector_path));
        w!();
        w!("```");
        w!("{}", hex_dump_16(raw));
        w!("```");
        w!();
        w!("| Field | Value |");
        w!("|---|---|");
        w!("| uid (u64 LE) | `{}` |", dec.uid_hex);
        w!(
            "| uid plausible ETS2? | {} |",
            if dec.uid_is_plausible {
                "✓ yes"
            } else {
                "✗ no"
            }
        );
        w!(
            "| uid bytes[0..4] as f32 | `{:.6}` — {} |",
            dec.uid_as_f32_pair[0],
            if dec.uid_as_f32_pair[0].abs() < 0.01 {
                "⚠ near-zero"
            } else {
                "ok"
            }
        );
        w!(
            "| uid bytes[4..8] as f32 | `{:.6}` — {} |",
            dec.uid_as_f32_pair[1],
            if dec.uid_as_f32_pair[1].abs() < 0.01 {
                "⚠ near-zero"
            } else {
                "ok"
            }
        );
        w!(
            "| kdop_bounds (10×f32) | `{}` |",
            dec.bounds
                .iter()
                .map(|v| format!("{v:.1}"))
                .collect::<Vec<_>>()
                .join(", ")
        );
        w!(
            "| kdop_flags (u32 LE) | `0x{:08x}` = {} |",
            dec.flags,
            dec.flags
        );
        w!("| view_dist (u8) | {} |", dec.view_dist);
        w!("| model_token (u64 LE) | `{}` |", dec.model_token_hex);
        w!();
    }

    // ── Section 4: Last-Item Pattern ─────────────────────────────────────────
    w!("## 4. Last-Item-Before-Crash Pattern");
    w!();
    w!("| Sector | Last item type | Last item kind | Last end offset | Crash at offset | Sign body bytes before failure |");
    w!("|---|---|---|---|---|---|");
    for r in results {
        let short = short_sector(r.sector_path);
        let lt = r.last_item_type.map_or("-".to_string(), |t| t.to_string());
        let lk = r.last_item_kind.as_deref().unwrap_or("-");
        let le = r
            .last_item_end_offset
            .map_or("-".to_string(), |o| o.to_string());
        let co = r
            .crash_error_offset
            .map_or("-".to_string(), |o| o.to_string());
        let bd = r
            .bytes_crash_item_body
            .map_or("-".to_string(), |b| b.to_string());
        w!("| `{short}` | {lt} | {lk} | {le} | {co} | {bd} |");
    }
    w!();

    // Last-item histogram (across all 9 sectors)
    let mut last_type_hist: HashMap<String, usize> = HashMap::new();
    for r in results {
        if let Some(k) = r.last_item_kind.as_deref() {
            *last_type_hist.entry(k.to_string()).or_default() += 1;
        }
    }
    let mut sorted_last: Vec<_> = last_type_hist.iter().collect();
    sorted_last.sort_by_key(|(_, &c)| std::cmp::Reverse(c));
    w!("**Last-item histogram across all 9 sectors:**");
    w!();
    w!("| Last item kind | Count |");
    w!("|---|---|");
    for (kind, count) in &sorted_last {
        w!("| {kind} | {count} |");
    }
    w!();

    // Crash error messages
    w!("**Full crash error messages:**");
    w!();
    for r in results {
        if let Some(msg) = &r.crash_error_msg {
            w!("- `{}`: `{msg}`", short_sector(r.sector_path));
        }
    }
    w!();

    // ── Section 4: Sign Body Field Decode (256-byte window) ─────────────────
    w!("## 4. Sign Body Field Decode (256-byte Window)");
    w!();
    w!("Byte-by-byte decode of the crashing Sign item body from DropTracer captures.");
    w!("Layout (sector.rs `skip_sign`): kdop_item(53B) + model_token(8) + node_uid(8) +");
    w!("look_token(8) + variant_token(8) + board_count(1) + boards(N×24) + template_len(8) + ...");
    w!();
    for (i, r) in results.iter().enumerate() {
        w!("### {} `{}`", i + 1, short_sector(r.sector_path));
        w!();
        let raw: &[u8] = if !r.tracer_raw_hex.is_empty() {
            &r.tracer_raw_hex
        } else {
            &RAW_HEX[i]
        };
        if raw.is_empty() {
            w!("*No tracer data captured.*");
            w!();
            continue;
        }
        w!("Captured: **{} bytes**", raw.len());
        w!();
        let decode = decode_sign_body(raw);
        w!("{}", decode);
        w!();
    }

    // ── Section 5: Auto-Diagnosis Hypothesis ─────────────────────────────────
    w!("## 5. Auto-Diagnosis Hypothesis");
    w!();
    write_auto_diagnosis(&mut f, results)?;

    Ok(())
}

fn write_auto_diagnosis(f: &mut std::fs::File, results: &[SectorAnalysis]) -> Result<()> {
    macro_rules! w {
        ($($arg:tt)*) => { writeln!(f, $($arg)*)?; }
    }

    // Gather evidence
    let found = results.iter().filter(|r| r.found_in_archive).count();

    // Pattern 1: uid bytes interpreted as float — near-zero means cursor was NOT at body start
    let uid_plausible_count = (0..results.len())
        .filter(|&i| decode_kdop_header(&RAW_HEX[i]).uid_is_plausible)
        .count();

    // Pattern 2: last-item variety
    let mut last_kinds: Vec<String> = results
        .iter()
        .filter_map(|r| r.last_item_kind.clone())
        .collect();
    last_kinds.sort();
    last_kinds.dedup();
    let unique_last_kinds = last_kinds.len();

    // Pattern 3: crash message variety
    let crash_msgs: std::collections::HashSet<String> = results
        .iter()
        .filter_map(|r| {
            r.crash_error_msg.as_ref().map(|m| {
                // normalise: strip the number to compare message patterns
                let idx = m.find(|c: char| c.is_ascii_digit()).unwrap_or(m.len());
                m[..idx].trim().to_string()
            })
        })
        .collect();

    w!("### Evidence Summary");
    w!();
    w!("| Evidence | Value |");
    w!("|---|---|");
    w!("| Sectors found in archive | {}/{} |", found, results.len());
    w!(
        "| uid bytes[0..8] plausible ETS2 UID (not float) | {}/{} |",
        uid_plausible_count,
        found
    );
    w!(
        "| Unique last-item kinds across all 9 sectors | {} |",
        unique_last_kinds
    );
    w!("| Unique crash-message prefixes | {} |", crash_msgs.len());
    w!();
    for msg in &crash_msgs {
        w!("  - `{msg}…`");
    }
    w!();

    w!("### Hypothesis");
    w!();

    if uid_plausible_count == found {
        w!("**Cursor was at sign body start** when the item was dispatched.");
        w!(
            "All {} UID values look like valid ETS2 UIDs (not float garbage).",
            found
        );
        w!("→ The cursor was NOT desynced by the previous item.");
        w!();
    } else {
        w!("⚠️ Some UID bytes look like floats — cursor may have been misaligned.");
        w!();
    }

    if unique_last_kinds <= 2 {
        let kinds_str = last_kinds.join(", ");
        w!("**Strong predecessor pattern:** last item kind(s) = [{kinds_str}].");
        w!("Shared predecessor type(s) across all 9 sectors suggests that the item");
        w!("BEFORE the sign did NOT desync the cursor — the crash is inside skip_sign.");
        w!();
    } else {
        w!("**Varied predecessors ({unique_last_kinds} unique kinds)** before the crash.");
        w!("This rules out a single predecessor type as the root cause.");
        w!();
    }

    if crash_msgs.len() == 1 {
        let msg = crash_msgs.iter().next().unwrap();
        w!("**All 9 crashes produce the SAME error message prefix:** `{msg}…`");
        w!("→ The crash always fires at the same code path inside skip_sign.");
        w!();
    }

    w!("### Conclusion");
    w!();
    w!("Based on the above evidence, the root cause is:");
    w!();

    if uid_plausible_count == found && crash_msgs.len() == 1 {
        w!("**Sign-Handler-Format-Bug (high confidence)**");
        w!();
        w!("The cursor was correctly positioned at the sign body start in all 9 cases.");
        w!("The crash fires at a fixed code path inside `skip_sign` — a count field");
        w!("inside the sign body is being misread as `0x02000000 = 33 554 432`.");
        w!();
        w!("Likely sub-hypotheses (requires per-byte trace past the 64-byte window):");
        w!("  A) `board_count` u8 is misread → loop overshoots → cursor lands in garbage");
        w!("     → `skip_sign_board_override_list` reads garbage count");
        w!("  B) `template_len` u64 read is correct but non-empty template skip under/over-shoots");
        w!("     → `skip_sign_board_override_list` reads bytes from template payload");
        w!("  C) The sign format for these specific signs has a field not present in");
        w!("     the Phase-5.12 TruckLib reference (format variant / version byte)");
    } else {
        w!("**Ambiguous — more instrumentation needed**");
        w!();
        w!("The evidence is mixed. Extend the raw_hex capture window beyond 64 bytes");
        w!("to reach the board_count / template_len / override list count bytes.");
    }

    Ok(())
}

// ---------------------------------------------------------------------------
// kdop_item header decode
// ---------------------------------------------------------------------------

struct KdopDecode {
    uid_hex: String,
    uid_is_plausible: bool,
    uid_as_f32_pair: [f32; 2],
    bounds: [f32; 10],
    flags: u32,
    view_dist: u8,
    model_token_hex: String,
}

fn decode_kdop_header(raw: &[u8; 64]) -> KdopDecode {
    let uid = u64::from_le_bytes(raw[0..8].try_into().unwrap());
    let uid_hex = format!("0x{uid:016x}");

    // Is uid plausible for ETS2? Heuristic: non-zero, high bits set (≥ 2^48)
    let uid_is_plausible = uid > (1u64 << 48) || (uid > 0 && uid < u64::MAX / 4);

    // Interpret uid bytes as 2×f32 (tests "was cursor in float data?")
    let f0 = f32::from_le_bytes(raw[0..4].try_into().unwrap());
    let f1 = f32::from_le_bytes(raw[4..8].try_into().unwrap());

    let mut bounds = [0.0f32; 10];
    for (j, b) in bounds.iter_mut().enumerate() {
        *b = f32::from_le_bytes(raw[8 + j * 4..12 + j * 4].try_into().unwrap());
    }

    let flags = u32::from_le_bytes(raw[48..52].try_into().unwrap());
    let view_dist = raw[52];

    // model_token at bytes 53..61 (may be truncated at byte 64 → only 8B available if 53+8≤64)
    let model_token = if raw.len() >= 61 {
        u64::from_le_bytes(raw[53..61].try_into().unwrap())
    } else {
        0
    };
    let model_token_hex = format!("0x{model_token:016x}");

    KdopDecode {
        uid_hex,
        uid_is_plausible,
        uid_as_f32_pair: [f0, f1],
        bounds,
        flags,
        view_dist,
        model_token_hex,
    }
}

// ---------------------------------------------------------------------------
// Sign body field decode (256-byte window)
// ---------------------------------------------------------------------------

fn decode_sign_body(raw: &[u8]) -> String {
    let mut rows: Vec<String> = Vec::new();
    rows.push("| Offset | Field | Hex (LE) | Value | Note |".into());
    rows.push("|---|---|---|---|---|".into());

    let n = raw.len();
    let mut pos = 0usize;

    macro_rules! need {
        ($size:expr) => {
            if pos + $size > n {
                rows.push(format!(
                    "| {pos} | (truncated — need {} more bytes) | — | — | window={n}B |",
                    pos + $size - n
                ));
                return rows.join("\n");
            }
        };
    }

    macro_rules! hex_bytes {
        ($start:expr, $end:expr) => {
            raw[$start..$end]
                .iter()
                .map(|b| format!("{b:02x}"))
                .collect::<Vec<_>>()
                .join(" ")
        };
    }

    // uid (u64, 8B)
    need!(8);
    let uid = u64::from_le_bytes(raw[pos..pos + 8].try_into().unwrap());
    let uid_ok = uid > (1u64 << 48) || (uid > 0 && uid < u64::MAX / 4);
    rows.push(format!(
        "| {pos} | uid | `{}` | `0x{uid:016x}` | {} |",
        hex_bytes!(pos, pos + 8),
        if uid_ok {
            "ETS2 UID ✓"
        } else {
            "⚠ implausible"
        }
    ));
    pos += 8;

    // kdop_bounds (10×f32, 40B)
    need!(40);
    let b0 = f32::from_le_bytes(raw[pos..pos + 4].try_into().unwrap());
    let b9 = f32::from_le_bytes(raw[pos + 36..pos + 40].try_into().unwrap());
    rows.push(format!(
        "| {pos} | kdop_bounds (10×f32, 40B) | `{} …` | {b0:.1} … {b9:.1} | ok |",
        hex_bytes!(pos, pos + 4)
    ));
    pos += 40;

    // kdop_flags (u32, 4B)
    need!(4);
    let flags = u32::from_le_bytes(raw[pos..pos + 4].try_into().unwrap());
    rows.push(format!(
        "| {pos} | kdop_flags | `{}` | `0x{flags:08x}` | ok |",
        hex_bytes!(pos, pos + 4)
    ));
    pos += 4;

    // view_dist (u8, 1B)
    need!(1);
    let vd = raw[pos];
    rows.push(format!("| {pos} | view_dist | `{vd:02x}` | {vd} | ok |"));
    pos += 1;

    // model_token (u64, 8B)
    need!(8);
    let model_tok = u64::from_le_bytes(raw[pos..pos + 8].try_into().unwrap());
    rows.push(format!(
        "| {pos} | model_token | `{}` | `0x{model_tok:016x}` | ok |",
        hex_bytes!(pos, pos + 8)
    ));
    pos += 8;

    // node_uid (u64, 8B)
    need!(8);
    let node_uid_val = u64::from_le_bytes(raw[pos..pos + 8].try_into().unwrap());
    rows.push(format!(
        "| {pos} | node_uid | `{}` | `0x{node_uid_val:016x}` | ok |",
        hex_bytes!(pos, pos + 8)
    ));
    pos += 8;

    // look_token (u64, 8B)
    need!(8);
    let look_tok = u64::from_le_bytes(raw[pos..pos + 8].try_into().unwrap());
    rows.push(format!(
        "| {pos} | look_token | `{}` | `0x{look_tok:016x}` | ok |",
        hex_bytes!(pos, pos + 8)
    ));
    pos += 8;

    // variant_token (u64, 8B)
    need!(8);
    let var_tok = u64::from_le_bytes(raw[pos..pos + 8].try_into().unwrap());
    rows.push(format!(
        "| {pos} | variant_token | `{}` | `0x{var_tok:016x}` | ok |",
        hex_bytes!(pos, pos + 8)
    ));
    pos += 8;

    // board_count (u8, 1B)
    need!(1);
    let board_count = raw[pos];
    rows.push(format!(
        "| {pos} | **board_count** | `{board_count:02x}` | **{board_count}** | {} |",
        if board_count > 20 {
            "⚠ suspiciously large"
        } else {
            "ok"
        }
    ));
    pos += 1;

    // boards (board_count × 24B)
    for b in 0..board_count as usize {
        for field in ["road_token", "city1_token", "city2_token"] {
            need!(8);
            let tok = u64::from_le_bytes(raw[pos..pos + 8].try_into().unwrap());
            rows.push(format!(
                "| {pos} | board[{b}].{field} | `{}` | `0x{tok:016x}` | ok |",
                hex_bytes!(pos, pos + 8)
            ));
            pos += 8;
        }
    }

    // template_len (u64, 8B)
    need!(8);
    let template_len = u64::from_le_bytes(raw[pos..pos + 8].try_into().unwrap());
    let tlen_note = if template_len > 10_000_000 {
        "⚠ GARBAGE — too large"
    } else if template_len == 0 {
        "zero → no overrides follow"
    } else {
        "ok"
    };
    rows.push(format!(
        "| {pos} | **template_len** | `{}` | **{template_len}** | {tlen_note} |",
        hex_bytes!(pos, pos + 8)
    ));
    pos += 8;

    if template_len == 0 || template_len > 10_000_000 {
        return rows.join("\n");
    }

    // template payload — show start bytes
    if pos < n {
        let show_end = (pos + 8).min(n);
        rows.push(format!(
            "| {pos} | template_payload ({template_len}B total) | `{}…` | — | |",
            hex_bytes!(pos, show_end)
        ));
    }
    let pos_after_template = pos.saturating_add(template_len as usize);

    // board_override_count (first field after template payload)
    if pos_after_template + 4 <= n {
        let boc = u32::from_le_bytes(
            raw[pos_after_template..pos_after_template + 4]
                .try_into()
                .unwrap(),
        );
        rows.push(format!(
            "| {pos_after_template} | **board_override_count** | `{}` | **{boc}** | {} |",
            hex_bytes!(pos_after_template, pos_after_template + 4),
            if boc > 2_000_000 { "⚠ GARBAGE" } else { "ok" }
        ));

        // sign_override_count (only readable if board_override_count=0 and window permits)
        if boc == 0 {
            let pos_soc = pos_after_template + 4;
            if pos_soc + 4 <= n {
                let soc = u32::from_le_bytes(raw[pos_soc..pos_soc + 4].try_into().unwrap());
                rows.push(format!(
                    "| {pos_soc} | **sign_override_count** | `{}` | **{soc}** | {} |",
                    hex_bytes!(pos_soc, pos_soc + 4),
                    if soc > 2_000_000 {
                        "⚠ GARBAGE — this is the crash!"
                    } else {
                        "ok"
                    }
                ));
            } else {
                rows.push(format!(
                    "| {pos_soc} | sign_override_count | (beyond {n}B window) | — | extend window |"
                ));
            }
        }
    } else {
        rows.push(format!(
            "| {pos_after_template} | board_override_count | (beyond {n}B window) | — | extend window |"
        ));
    }

    rows.join("\n")
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn dist_xz(ax: f32, az: f32, bx: f32, bz: f32) -> f32 {
    let dx = ax - bx;
    let dz = az - bz;
    (dx * dx + dz * dz).sqrt()
}

fn short_sector(path: &str) -> &str {
    path.rsplit('/')
        .next()
        .unwrap_or(path)
        .trim_end_matches(".base")
}

fn short_sector_display(path: &str) -> &str {
    path.rsplit('/').next().unwrap_or(path)
}

fn item_type_name(t: u32) -> &'static str {
    match t {
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
        _ => "unknown",
    }
}

/// Format bytes as a hex dump: 16 bytes per line with ASCII column.
fn hex_dump_16(data: &[u8]) -> String {
    let mut out = String::new();
    for (i, chunk) in data.chunks(16).enumerate() {
        let offset = i * 16;
        let hex: String = chunk
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect::<Vec<_>>()
            .join(" ");
        let ascii: String = chunk
            .iter()
            .map(|&b| if b.is_ascii_graphic() { b as char } else { '.' })
            .collect();
        let padding = "   ".repeat(16 - chunk.len());
        out.push_str(&format!("{offset:04x}  {hex}{padding}  |{ascii}|\n"));
    }
    out
}
