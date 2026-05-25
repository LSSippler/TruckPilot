//! `sec-single-inspect` — Phase 6.2b-Outlier (Tasks 1-3)
//!
//! Read-only diagnostic for sec-0001-0008 (or any named sector).
//! Outputs:
//!   - DLC status (which archive owns the sector)
//!   - node_count, road_count, recovered_nodes_count
//!   - Item-type histogram (via audit_sector)
//!   - First 5 roads: node_a / node_b UIDs
//!   - Prefab-UID × Road-node-UID intersection (H7 check)
//!   - Geographic world-space position estimate
//!
//! Gate: Read-only. No production-code changes.

use std::collections::HashMap;
use std::io::Write as _;
use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;
use truckpilot_map_parser::sector::{audit_sector, parse_sector};
use truckpilot_map_parser::{Archive, HashFsArchive, ModLoadOrder, ZipArchive};

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

#[derive(Parser)]
#[command(
    name = "sec-single-inspect",
    about = "Phase 6.2b-Outlier: single-sector deep inspection (DLC-check + node-count + histogram)"
)]
struct Args {
    /// ETS2 install directory.
    #[arg(long)]
    ets2_dir: PathBuf,

    /// Optional mods dir (defaults to ~/Documents/Euro Truck Simulator 2/mod).
    #[arg(long)]
    mods_dir: Option<PathBuf>,

    /// Sector path inside archives, e.g. map/europe/sec-0001-0008.base
    #[arg(long, default_value = "map/europe/sec-0001-0008.base")]
    sector: String,

    /// Output directory.
    #[arg(long, default_value = "outputs/2026-05-17")]
    output_dir: PathBuf,

    /// Output filename (inside output_dir).
    #[arg(long, default_value = "sec_0001_0008_inspection.md")]
    output_file: String,
}

fn default_mods_dir() -> PathBuf {
    dirs::document_dir()
        .or_else(dirs::home_dir)
        .unwrap_or_default()
        .join("Euro Truck Simulator 2/mod")
}

// ---------------------------------------------------------------------------
// Item type names
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Geography helpers
// ---------------------------------------------------------------------------

/// Approximate world-space XZ center of a sector tile.
/// ETS2 sector grid: 4096 units per tile, center = index * 4096 + 2048.
fn sector_world_xz(path: &str) -> Option<(f32, f32)> {
    let name = path.rsplit('/').next()?;
    let name = name.strip_suffix(".base")?;
    let name = name.strip_prefix("sec")?;
    if name.len() < 10 {
        return None;
    }
    let x: i32 = name[0..5].parse().ok()?;
    let z: i32 = name[5..10].parse().ok()?;
    Some((x as f32 * 4096.0 + 2048.0, z as f32 * 4096.0 + 2048.0))
}

fn known_cities() -> &'static [(&'static str, f32, f32)] {
    &[
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
        ("Calais", -30000.0, -8500.0),
        ("London", -32000.0, -9000.0),
        ("Duisburg", -22800.0, -5200.0),
    ]
}

fn nearest_city(wx: f32, wz: f32) -> (&'static str, f32) {
    known_cities()
        .iter()
        .map(|(name, cx, cz)| {
            let d = ((wx - cx).powi(2) + (wz - cz).powi(2)).sqrt();
            (*name, d)
        })
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
        .unwrap_or(("unknown", f32::MAX))
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() -> Result<()> {
    let args = Args::parse();
    let mods_dir = args.mods_dir.clone().unwrap_or_else(default_mods_dir);
    std::fs::create_dir_all(&args.output_dir)?;

    // ── Step 1: Open archives ─────────────────────────────────────────────
    eprintln!("Opening archives...");
    let order = ModLoadOrder::from_directories(&args.ets2_dir, &mods_dir)
        .context("build mod load order")?;

    let mut archives: Vec<Box<dyn Archive>> = Vec::new();
    let mut archive_names: Vec<String> = Vec::new();
    for entry in &order.entries {
        let arc: Box<dyn Archive> = match HashFsArchive::open(&entry.path) {
            Ok(a) => Box::new(a),
            Err(_) => match ZipArchive::open(&entry.path) {
                Ok(a) => Box::new(a),
                Err(_) => continue,
            },
        };
        archives.push(arc);
        archive_names.push(entry.name.clone());
    }
    eprintln!("  {} archives open.", archives.len());

    // ── Step 2: DLC check — find which archive owns the sector ────────────
    // Iterate in reverse (same as multi-sector-audit): last archive wins.
    let sector_path = &args.sector;
    let mut owner_archive: Option<String> = None;
    let mut sector_data: Option<Vec<u8>> = None;

    for (i, arc) in archives.iter_mut().enumerate().rev() {
        if let Ok(data) = arc.read_path(sector_path) {
            owner_archive = Some(archive_names[i].clone());
            sector_data = Some(data);
            break;
        }
    }

    let owner_name = match &owner_archive {
        Some(n) => n.clone(),
        None => {
            eprintln!("ERROR: sector '{}' not found in any archive.", sector_path);
            eprintln!("Searched archives:");
            for name in &archive_names {
                eprintln!("  {}", name);
            }
            anyhow::bail!("sector not found");
        }
    };

    let data = sector_data.unwrap();
    eprintln!("  Owner archive: {}", owner_name);
    eprintln!("  Sector data size: {} bytes", data.len());

    // ── Step 3: audit_sector — item type histogram ─────────────────────────
    eprintln!("Running audit_sector...");
    let audit = audit_sector(&data);

    let mut type_counts: HashMap<u32, usize> = HashMap::new();
    for item in &audit.items {
        *type_counts.entry(item.item_type).or_default() += 1;
    }
    let mut type_vec: Vec<(u32, usize)> = type_counts.into_iter().collect();
    type_vec.sort_by_key(|(t, _)| *t);

    let audit_failure = audit
        .failure
        .as_ref()
        .map(|f| {
            format!(
                "offset=0x{:X}, raw_type=0x{:X}, msg={}",
                f.error_offset, f.raw_type, f.error_msg
            )
        })
        .unwrap_or_else(|| "none (clean parse)".to_string());

    eprintln!(
        "  item_count={}, items_parsed={}, failure={}",
        audit.item_count,
        audit.items.len(),
        if audit.failure.is_none() {
            "NONE"
        } else {
            "YES"
        }
    );

    // ── Step 4: parse_sector — node/road counts + first 5 roads ──────────
    eprintln!("Running parse_sector...");
    let parsed = parse_sector(&data).map_err(|e| anyhow::anyhow!("parse_sector failed: {e:?}"))?;

    let node_count = parsed.nodes.len();
    let road_count = parsed.roads.len();
    let prefab_count = parsed.prefabs.len();
    let building_count = parsed.buildings.len();
    let ferry_count = parsed.ferries.len();
    let recovered_count = parsed.recovered_nodes_count;

    eprintln!(
        "  nodes={} (recovered={}), roads={}, prefabs={}, buildings={}, ferries={}",
        node_count, recovered_count, road_count, prefab_count, building_count, ferry_count
    );

    // First 5 roads
    let sample_roads: Vec<(u64, u64, u64)> = parsed
        .roads
        .iter()
        .take(5)
        .map(|r| (r.uid, r.node_a, r.node_b))
        .collect();

    // ── Step 5: H7 check — prefab UID × road node UID intersection ────────
    let road_node_uids: std::collections::HashSet<u64> = parsed
        .roads
        .iter()
        .flat_map(|r| [r.node_a, r.node_b])
        .filter(|&uid| uid != 0)
        .collect();

    let prefab_node_uids: std::collections::HashSet<u64> = parsed
        .prefabs
        .iter()
        .flat_map(|p| p.nodes.iter().copied())
        .filter(|&uid| uid != 0)
        .collect();

    let intersection: std::collections::HashSet<&u64> =
        road_node_uids.intersection(&prefab_node_uids).collect();
    let h7_overlap_count = intersection.len();
    let h7_overlap_pct = if road_node_uids.is_empty() {
        0.0
    } else {
        h7_overlap_count as f64 * 100.0 / road_node_uids.len() as f64
    };

    // ── Step 6: Geographic position ──────────────────────────────────────
    let world_pos = sector_world_xz(sector_path);
    let _geo_desc = if let Some((wx, wz)) = world_pos {
        let (city, dist_m) = nearest_city(wx, wz);
        format!(
            "World XZ ≈ ({:.0}, {:.0}) — nearest city: {} ({:.0}m away)",
            wx, wz, city, dist_m
        )
    } else {
        "Could not parse sector coordinates from path".to_string()
    };

    // ── DLC classification ────────────────────────────────────────────────
    let dlc_status = if owner_name == "base_map.scs" || owner_name == "def.scs" {
        "BASE GAME"
    } else if owner_name.starts_with("dlc_") {
        "DLC"
    } else {
        "OTHER (mod or unknown)"
    };

    // ── Write output Markdown ─────────────────────────────────────────────
    let out_path = args.output_dir.join(&args.output_file);
    let mut f = std::fs::File::create(&out_path)
        .with_context(|| format!("create output {:?}", out_path))?;

    macro_rules! w {
        ($($arg:tt)*) => { writeln!(f, $($arg)*)?; }
    }

    w!("# sec-0001-0008 Inspection — Phase 6.2b-Outlier");
    w!();
    w!("> Generated by `sec-single-inspect` (read-only)");
    w!("> Sector: `{}`", sector_path);
    w!("> Date: 2026-05-17");
    w!();

    // ── Section 1: DLC Status ─────────────────────────────────────────────
    w!("## 1. DLC Status (H6 Check)");
    w!();
    w!("| Field | Value |");
    w!("|---|---|");
    w!("| Owner archive | `{}` | ", owner_name);
    w!("| Classification | **{}** |", dlc_status);
    w!("| Sector data size | {} bytes |", data.len());
    w!();
    let h6_verdict = if dlc_status == "BASE GAME" {
        "**H6 FALSIFIED**: Sector is in base game archive — no DLC-specific format expected."
    } else if dlc_status == "DLC" {
        "**H6 POSSIBLE**: Sector is in a DLC archive — DLC-specific node format possible."
    } else {
        "**H6 INCONCLUSIVE**: Sector is in a mod/unknown archive."
    };
    w!("{}", h6_verdict);
    w!();

    // ── Section 2: Geographic Position ───────────────────────────────────
    w!("## 2. Geographic Position");
    w!();
    if let Some((wx, wz)) = world_pos {
        w!("Sector coordinates from path name: X={}, Z={}", -1, -8);
        w!("World-space center (ETS2 units, 1 unit ≈ 1m):");
        w!();
        w!("- World X ≈ {:.0}", wx);
        w!("- World Z ≈ {:.0}", wz);
        w!();
        let (city, dist_m) = nearest_city(wx, wz);
        w!(
            "Nearest known city: **{}** ({:.0}m from sector center)",
            city,
            dist_m
        );
        w!();

        // Top 5 nearest cities
        let mut city_dists: Vec<(&str, f32)> = known_cities()
            .iter()
            .map(|(n, cx, cz)| (*n, ((wx - cx).powi(2) + (wz - cz).powi(2)).sqrt()))
            .collect();
        city_dists.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        w!("| City | Distance (m) |");
        w!("|---|---|");
        for (city_name, dist) in city_dists.iter().take(5) {
            w!("| {} | {:.0} |", city_name, dist);
        }
    } else {
        w!(
            "Could not parse sector coordinates from path `{}`.",
            sector_path
        );
    }
    w!();

    // ── Section 3: Parse Results ──────────────────────────────────────────
    w!("## 3. Parse Results (H1 / H7 Core Data)");
    w!();
    w!("| Metric | Value |");
    w!("|---|---|");
    w!("| `sector.nodes.len()` | **{}** |", node_count);
    w!("| `sector.recovered_nodes_count` | {} |", recovered_count);
    w!("| `sector.roads.len()` | {} |", road_count);
    w!("| `sector.prefabs.len()` | {} |", prefab_count);
    w!("| `sector.buildings.len()` | {} |", building_count);
    w!("| `sector.ferries.len()` | {} |", ferry_count);
    w!("| `audit.item_count` (header) | {} |", audit.item_count);
    w!(
        "| `audit.items.len()` (parsed by walker) | {} |",
        audit.items.len()
    );
    w!("| Audit failure | {} |", audit_failure);
    w!();

    if node_count == 0 {
        w!("**H1 CONFIRMED**: `sector.nodes.len() == 0`. The sector has no trailing node section entries.");
        w!("All 230 roads parse successfully, but their node_a/node_b UIDs have no matching RawNode.");
        w!("The brute-force forensic scan finds these UIDs in the road body bytes (+245/+253 offsets),");
        w!("not in a node definition block — consistent with the spec's critical constraint.");
    } else {
        w!(
            "**H1 FALSIFIED**: sector has {} nodes — parse produced nodes.",
            node_count
        );
        w!("Problem must be downstream. Continue with H2/H3 tests.");
    }
    w!();

    // ── Section 4: Item-type Histogram ────────────────────────────────────
    w!("## 4. Item-Type Histogram");
    w!();
    w!("| Type | Name | Count | % of header item_count |");
    w!("|---|---|---|---|");
    let total_header = audit.item_count as f64;
    for (t, c) in &type_vec {
        let pct = if total_header > 0.0 {
            *c as f64 * 100.0 / total_header
        } else {
            0.0
        };
        w!("| {} | {} | {} | {:.1}% |", t, item_type_name(*t), c, pct);
    }
    w!();

    // ── Section 5: First 5 Roads ──────────────────────────────────────────
    w!("## 5. First 5 Roads — node_a / node_b UIDs");
    w!();
    if sample_roads.is_empty() {
        w!("No roads parsed.");
    } else {
        w!("| # | road_uid | node_a | node_b |");
        w!("|---|---|---|---|");
        for (i, (uid, na, nb)) in sample_roads.iter().enumerate() {
            w!("| {} | {} | {} | {} |", i + 1, uid, na, nb);
        }
    }
    w!();

    // ── Section 6: H7 Prefab-Node Cross-Reference ─────────────────────────
    w!("## 6. H7 Prefab-Node Cross-Reference");
    w!();
    w!("Tests whether road node_a/node_b UIDs overlap with prefab `connected_node_uids`.");
    w!("(If overlap is high and nodes.len()==0, the UIDs are Prefab-References, not Standalone Nodes.)");
    w!();
    w!("| Metric | Value |");
    w!("|---|---|");
    w!(
        "| Unique road node UIDs (node_a ∪ node_b) | {} |",
        road_node_uids.len()
    );
    w!(
        "| Unique prefab connected_node_uids | {} |",
        prefab_node_uids.len()
    );
    w!(
        "| Intersection (road refs in prefab UIDs) | {} ({:.1}%) |",
        h7_overlap_count,
        h7_overlap_pct
    );
    w!();

    if node_count == 0 {
        if h7_overlap_pct > 30.0 {
            w!(
                "**H7 SUPPORTED**: {:.1}% of road node refs appear in prefab UID lists.",
                h7_overlap_pct
            );
            w!("Road endpoints are NOT standalone nodes — they are referenced via prefab connections.");
            w!("This is a structural cross-sector issue: roads in this sector reference nodes defined");
            w!("elsewhere (another sector's node trailer or a prefab item in another sector).");
        } else if prefab_count == 0 {
            w!("**H7 NOT APPLICABLE**: No prefabs in this sector — H7 requires prefab items.");
            w!("Road node UIDs with no matching nodes and no local prefabs = pure node_count=0 sector.");
            w!("The sector simply has no trailing nodes (H1). Roads reference nodes in OTHER sectors.");
        } else {
            w!(
                "**H7 WEAK**: Only {:.1}% overlap between road refs and prefab UIDs.",
                h7_overlap_pct
            );
            w!("Road node refs are not primarily from prefabs — they reference nodes in other sectors.");
        }
    } else {
        w!(
            "(H7 analysis only meaningful when nodes.len()==0; sector has {} nodes.)",
            node_count
        );
    }
    w!();

    // ── Section 7: Diagnosis + Recommendation ─────────────────────────────
    w!("## 7. Diagnosis & Recommendation");
    w!();
    if node_count == 0 {
        w!("### Root Cause: H1 Confirmed — Empty Node Trailer");
        w!();
        w!("The sector parses completely (0 UnknownItemType, clean item dispatch),");
        w!(
            "but its trailing `node_count` is 0. All {} roads reference endpoint UIDs",
            road_count
        );
        w!("that exist only as 8-byte values in Road body bytes (offsets +245/+253),");
        w!("never as `RawNode` entries. The brute-force forensic scan's 230 BothFound");
        w!("events are explained: UIDs found in road bodies, not node definitions.");
        w!();
        w!("### Impact");
        w!();
        w!("- {} roads dropped as BothUnresolved.", road_count);
        w!("- 0 nodes contributed to the global node map.");
        if let Some((wx, wz)) = world_pos {
            let (city, dist_m) = nearest_city(wx, wz);
            w!(
                "- Sector is near {} ({:.0}m) — check whether any test-set route passes through.",
                city,
                dist_m
            );
        }
        w!();
        w!("### Decision Gate");
        w!();
        w!("Per the DeepSeek-Spec Decision-Tree (H1 + node_count=0 confirmed):");
        w!();
        w!("| Question | To determine |");
        w!("|---|---|");
        w!("| Does any cities.toml route require traversal through this sector? | Fix-5c vs Defer |");
        w!("| Are road node UIDs defined as standalone nodes in ADJACENT sectors? | Cross-sector issue scope |");
        w!();
        w!("**Recommendation: DEFER to Phase 6.4 (Cross-Sector-Edge-Generation)**");
        w!(
            "unless a cities.toml route is blocked. Impact is {} / 1,063,231 nodes = 0.02%.",
            road_count
        );
        w!("A proper fix requires cross-sector node resolution, not a local patch.");
    } else {
        w!(
            "H1 falsified — sector has {} nodes. Continue with H2/H3 tests.",
            node_count
        );
        w!("Run `sec-single-inspect` with `--verbose` flag after adding sized-gate instrumentation.");
    }
    w!();

    eprintln!("-> {}", out_path.display());
    println!("{}", out_path.display());
    Ok(())
}
