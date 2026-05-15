//! `truckpilot-item-census` — Phase 5.25 pre-check.
//!
//! Quantifies Two-Node-Item volume in `base_map.scs` BEFORE implementing
//! 5.25b/5.25c edge-generation. Produces total / resolved / redundant /
//! unique-new / cross-sector breakdown per item type plus GO/SKIP advice.
//!
//! Diagnose-only — no production-dispatch changes, no ParsedSector
//! mutation. Light "parser" is a byte-offset slice on the raw item body
//! (item_type u32 + kdop_item 53 B is fixed for all v907 items, and the
//! Node + ForwardNode pair sits at known constant offsets for the three
//! types of interest). Item byte ranges come from the public
//! `audit_sector` walker; node-to-sector mapping from `parse_sector`.

use std::collections::{HashMap, HashSet};
use std::fmt::Write as _;
use std::path::PathBuf;

use truckpilot_map_parser::sector::{audit_sector, parse_sector};
use truckpilot_map_parser::{Archive, HashFsArchive, ModLoadOrder, ZipArchive};

const ITEM_TYPE_TERRAIN: u32 = 1;
const ITEM_TYPE_BUILDINGS: u32 = 2;
const ITEM_TYPE_CURVE: u32 = 44;

#[derive(serde::Deserialize)]
struct GraphFile {
    nodes: Vec<NodeJson>,
    edges: Vec<EdgeJson>,
}
#[derive(serde::Deserialize)]
struct NodeJson {
    uid: u64,
}
#[derive(serde::Deserialize)]
struct EdgeJson {
    from: u64,
    to: u64,
    direction: String,
}

#[derive(Default)]
struct TypeStats {
    total: usize,
    parse_failed: usize,
    zero_uid: usize,
    self_loop: usize,
    both_resolved: usize,
    already_road: usize,
    already_building: usize,
    unique_new: usize,
    cross_sector: usize,
    cs_already_road: usize,
    cs_unique_new: usize,
    locator_nonzero_any: usize,
    locator_nonzero_count: usize,
}

fn pair_key(a: u64, b: u64) -> (u64, u64) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}

fn census_pair(
    s: &mut TypeStats,
    n_uid: u64,
    f_uid: u64,
    known_uids: &HashSet<u64>,
    road_pairs: &HashSet<(u64, u64)>,
    building_pairs: &HashSet<(u64, u64)>,
    node_to_sector: &HashMap<u64, u32>,
) {
    if n_uid == 0 || f_uid == 0 {
        s.zero_uid += 1;
        return;
    }
    if n_uid == f_uid {
        s.self_loop += 1;
        return;
    }
    if !known_uids.contains(&n_uid) || !known_uids.contains(&f_uid) {
        return;
    }
    s.both_resolved += 1;
    let pair = pair_key(n_uid, f_uid);
    let in_road = road_pairs.contains(&pair);
    let in_bldg = building_pairs.contains(&pair);
    if in_road {
        s.already_road += 1;
    }
    if in_bldg {
        s.already_building += 1;
    }
    if !in_road && !in_bldg {
        s.unique_new += 1;
    }
    if let (Some(a), Some(b)) = (node_to_sector.get(&n_uid), node_to_sector.get(&f_uid)) {
        if a != b {
            s.cross_sector += 1;
            if in_road {
                s.cs_already_road += 1;
            }
            if !in_road && !in_bldg {
                s.cs_unique_new += 1;
            }
        }
    }
}

fn read_u64_le(slice: &[u8], off: usize) -> Option<u64> {
    let end = off.checked_add(8)?;
    if end > slice.len() {
        return None;
    }
    Some(u64::from_le_bytes(slice[off..end].try_into().unwrap()))
}

/// Returns `(node, forward_node, locator_0, locator_1)`. Locators are
/// only valid for curve. `item_bytes` starts at the item_type u32, body
/// kdop_item starts at +4 and is 53 B fixed -> first post-kdop field at +57.
fn extract_two_node(
    item_bytes: &[u8],
    item_type: u32,
) -> Option<(u64, u64, Option<u64>, Option<u64>)> {
    let kdop_end = 4 + 53;
    let (n_off, f_off, loc0, loc1) = match item_type {
        ITEM_TYPE_TERRAIN => (kdop_end, kdop_end + 8, None, None),
        ITEM_TYPE_BUILDINGS => (kdop_end + 16, kdop_end + 24, None, None),
        ITEM_TYPE_CURVE => (
            kdop_end,
            kdop_end + 8,
            Some(kdop_end + 16),
            Some(kdop_end + 24),
        ),
        _ => return None,
    };
    let n = read_u64_le(item_bytes, n_off)?;
    let f = read_u64_le(item_bytes, f_off)?;
    let l0 = loc0.and_then(|o| read_u64_le(item_bytes, o));
    let l1 = loc1.and_then(|o| read_u64_le(item_bytes, o));
    Some((n, f, l0, l1))
}

fn parse_args() -> (PathBuf, Option<PathBuf>, PathBuf, PathBuf) {
    let mut ets2_dir: Option<PathBuf> = None;
    let mut mods_dir: Option<PathBuf> = None;
    let mut graph = PathBuf::from("graph.json");
    let mut output = PathBuf::from("outputs/item_census.txt");
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--ets2-dir" => {
                ets2_dir = Some(PathBuf::from(&argv[i + 1]));
                i += 2;
            }
            "--mods-dir" => {
                mods_dir = Some(PathBuf::from(&argv[i + 1]));
                i += 2;
            }
            "--graph" => {
                graph = PathBuf::from(&argv[i + 1]);
                i += 2;
            }
            "--output" => {
                output = PathBuf::from(&argv[i + 1]);
                i += 2;
            }
            _ => i += 1,
        }
    }
    (
        ets2_dir.expect("--ets2-dir required"),
        mods_dir,
        graph,
        output,
    )
}

fn default_mods_dir() -> PathBuf {
    dirs::document_dir()
        .or_else(dirs::home_dir)
        .unwrap_or_default()
        .join("Euro Truck Simulator 2/mod")
}

fn recommend(label: &str, s: &TypeStats) -> String {
    let unique_pct = if s.both_resolved > 0 {
        100.0 * s.unique_new as f64 / s.both_resolved as f64
    } else {
        0.0
    };
    if s.total < 5000 {
        format!(
            "Type {label} : SKIP   (total {} < 5000, Buildings-Pattern: zu wenig Volumen)",
            s.total
        )
    } else if s.cs_unique_new > 2000 {
        format!(
            "Type {label} : GO     (cross-sec-unique {} > 2000)",
            s.cs_unique_new
        )
    } else if unique_pct > 20.0 {
        format!(
            "Type {label} : GO     (unique-rate {:.1}% > 20%, {} new edges expected)",
            unique_pct, s.unique_new
        )
    } else if unique_pct < 10.0 {
        format!(
            "Type {label} : SKIP   (unique-rate {:.1}% < 10%)",
            unique_pct
        )
    } else {
        format!(
            "Type {label} : REVIEW (unique-rate {:.1}% in [10%, 20%], {} new edges)",
            unique_pct, s.unique_new
        )
    }
}

fn main() {
    let (ets2_dir, mods_dir_opt, graph_path, output_path) = parse_args();
    let mods_dir = mods_dir_opt.unwrap_or_else(default_mods_dir);
    eprintln!("ets2_dir = {}", ets2_dir.display());
    eprintln!(
        "mods_dir = {} (exists: {})",
        mods_dir.display(),
        mods_dir.exists()
    );
    eprintln!("loading graph from {} ...", graph_path.display());
    let bytes = std::fs::read(&graph_path).expect("read graph.json");
    let graph: GraphFile = serde_json::from_slice(&bytes).expect("parse graph.json");
    eprintln!("  {} nodes, {} edges", graph.nodes.len(), graph.edges.len());

    let known_uids: HashSet<u64> = graph.nodes.iter().map(|n| n.uid).collect();
    let mut road_pairs: HashSet<(u64, u64)> = HashSet::new();
    let mut building_pairs: HashSet<(u64, u64)> = HashSet::new();
    for e in &graph.edges {
        let p = pair_key(e.from, e.to);
        match e.direction.as_str() {
            "building" => {
                building_pairs.insert(p);
            }
            "forward" | "backward" | "bidirectional_unknown" | "prefab" => {
                road_pairs.insert(p);
            }
            _ => {}
        }
    }
    eprintln!(
        "  {} road-pairs, {} building-pairs",
        road_pairs.len(),
        building_pairs.len()
    );

    // Use the same ModLoadOrder as production (base + workshop mods,
    // alphabetical for mods, last-wins). Open each entry as HashFS first,
    // ZIP fallback.
    let order = ModLoadOrder::from_directories(&ets2_dir, &mods_dir).expect("build mod load order");
    let mut archives: Vec<Box<dyn Archive>> = Vec::new();
    for entry in &order.entries {
        let arc: Box<dyn Archive> = match HashFsArchive::open(&entry.path) {
            Ok(a) => Box::new(a),
            Err(_) => match ZipArchive::open(&entry.path) {
                Ok(a) => Box::new(a),
                Err(e) => {
                    eprintln!("  skip {}: {e}", entry.name);
                    continue;
                }
            },
        };
        archives.push(arc);
    }
    eprintln!(
        "opened {} archives (production-equivalent load order)",
        archives.len()
    );

    // Mod-loader pattern: gather all .base paths, last-archive-wins.
    let mut all_paths: HashSet<String> = HashSet::new();
    for arc in &archives {
        let mut files = arc.list_files();
        if files.is_empty() {
            if let Some(h) = arc.as_any().downcast_ref::<HashFsArchive>() {
                files = h.probe_sector_paths();
            }
        }
        for f in files {
            if f.ends_with(".base") {
                all_paths.insert(f);
            }
        }
    }
    let mut sector_paths: Vec<String> = all_paths.into_iter().collect();
    sector_paths.sort();
    eprintln!(
        "scanning {} `.base` sectors across all archives ...",
        sector_paths.len()
    );

    let read_sector = |path: &str, archives: &mut [Box<dyn Archive>]| -> Option<Vec<u8>> {
        archives
            .iter_mut()
            .rev()
            .find_map(|a| a.read_path(path).ok())
    };

    // Pass 1: build node_to_sector.
    let mut node_to_sector: HashMap<u64, u32> = HashMap::with_capacity(1_200_000);
    for (sid, path) in sector_paths.iter().enumerate() {
        let Some(data) = read_sector(path, &mut archives) else {
            continue;
        };
        if let Ok(parsed) = parse_sector(&data) {
            for n in &parsed.nodes {
                node_to_sector.insert(n.uid, sid as u32);
            }
        }
    }
    eprintln!("  node_to_sector: {} entries", node_to_sector.len());

    // Pass 2 — combined walk:
    //   * parse_sector gives the production-equivalent Buildings list
    //     (matches Phase 5.25a's 606 figure, includes sized-format sectors).
    //   * audit_sector gives raw item byte ranges so we can extract Terrain
    //     and Curve Node + ForwardNode pairs (those types aren't captured
    //     in ParsedSector). audit_sector only handles legacy-format
    //     sectors; on sized sectors it would yield garbage, so we
    //     per-sector sanity-gate: only trust audit when its Buildings
    //     count agrees with parse_sector's. Buildings counter itself
    //     always comes from parse_sector (canonical source).
    let mut stats: HashMap<u32, TypeStats> = HashMap::new();
    let mut hex_dumps: Vec<String> = Vec::new();
    let mut sectors_audit_unsafe = 0usize;
    for path in &sector_paths {
        let Some(data) = read_sector(path, &mut archives) else {
            continue;
        };
        let parsed = match parse_sector(&data) {
            Ok(p) => p,
            Err(_) => continue,
        };

        // Buildings — counted from parse_sector for production parity.
        for b in &parsed.buildings {
            let s = stats.entry(ITEM_TYPE_BUILDINGS).or_default();
            s.total += 1;
            census_pair(
                s,
                b.node_uid,
                b.forward_node_uid,
                &known_uids,
                &road_pairs,
                &building_pairs,
                &node_to_sector,
            );
        }

        // Terrain/Curve — via audit_sector. Sanity-gate per sector.
        let report = audit_sector(&data);
        let audit_bldg_count = report
            .items
            .iter()
            .filter(|i| i.item_type == ITEM_TYPE_BUILDINGS)
            .count();
        if audit_bldg_count != parsed.buildings.len() {
            sectors_audit_unsafe += 1;
            continue;
        }
        for it in &report.items {
            if !matches!(it.item_type, ITEM_TYPE_TERRAIN | ITEM_TYPE_CURVE) {
                continue;
            }
            let s = stats.entry(it.item_type).or_default();
            s.total += 1;
            let item_bytes = &data[it.start_offset..it.end_offset];
            let Some((n_uid, f_uid, l0, l1)) = extract_two_node(item_bytes, it.item_type) else {
                s.parse_failed += 1;
                if hex_dumps.len() < 3 {
                    let preview_end = it.start_offset.saturating_add(200).min(data.len());
                    hex_dumps.push(format!(
                        "  type={} sector={} item#={} bytes(0..{}): {:02x?}",
                        it.item_type,
                        path,
                        it.index,
                        preview_end - it.start_offset,
                        &data[it.start_offset..preview_end]
                    ));
                }
                continue;
            };
            if it.item_type == ITEM_TYPE_CURVE {
                let l0 = l0.unwrap_or(0);
                let l1 = l1.unwrap_or(0);
                if l0 != 0 || l1 != 0 {
                    s.locator_nonzero_any += 1;
                }
                if l0 != 0 {
                    s.locator_nonzero_count += 1;
                }
                if l1 != 0 {
                    s.locator_nonzero_count += 1;
                }
            }
            census_pair(
                s,
                n_uid,
                f_uid,
                &known_uids,
                &road_pairs,
                &building_pairs,
                &node_to_sector,
            );
        }
    }
    eprintln!(
        "  sectors with audit/parse Buildings count mismatch (terrain/curve skipped): {}",
        sectors_audit_unsafe
    );

    let terrain = std::mem::take(stats.entry(ITEM_TYPE_TERRAIN).or_default());
    let bldg = std::mem::take(stats.entry(ITEM_TYPE_BUILDINGS).or_default());
    let curve = std::mem::take(stats.entry(ITEM_TYPE_CURVE).or_default());

    let bldg_sanity_ok = bldg.total == 606 && bldg.both_resolved == 483;

    let mut out = String::new();
    let _ = writeln!(out, "===========================================");
    let _ = writeln!(out, "TRUCKPILOT ITEM CENSUS — Phase 5.25 Pre-Check");
    let _ = writeln!(
        out,
        "Source: production-equivalent load order ({} archives, {} .base sectors)",
        archives.len(),
        sector_paths.len()
    );
    let _ = writeln!(out, "===========================================");
    let _ = writeln!(out);
    let _ = writeln!(out, "ITEM-TYPE-COUNTS");
    let _ = writeln!(out, "----------------");
    let _ = writeln!(out, "Type 1  (Terrain)    :  {:>9} items", terrain.total);
    let _ = writeln!(
        out,
        "Type 2  (Buildings)  :  {:>9} items   [Sanity: should be 606 -> {}]",
        bldg.total,
        if bldg_sanity_ok { "OK" } else { "MISMATCH" }
    );
    let _ = writeln!(out, "Type 44 (Curve)      :  {:>9} items", curve.total);
    let _ = writeln!(out);
    let _ = writeln!(out, "RESOLVE-RATES");
    let _ = writeln!(out, "-------------");
    let _ = writeln!(
        out,
        "Type    | Total    | Both-Resolved | Resolve-% | ZeroUID | SelfLoop | ParseFail"
    );
    for (label, s) in [
        ("Terrain", &terrain),
        ("Bldg   ", &bldg),
        ("Curve  ", &curve),
    ] {
        let pct = if s.total > 0 {
            100.0 * s.both_resolved as f64 / s.total as f64
        } else {
            0.0
        };
        let _ = writeln!(
            out,
            "{} | {:>8} | {:>13} | {:>7.2}% | {:>7} | {:>8} | {:>9}",
            label, s.total, s.both_resolved, pct, s.zero_uid, s.self_loop, s.parse_failed
        );
    }
    let _ = writeln!(out);
    let _ = writeln!(out, "REDUNDANZ-CHECK (of resolved items)");
    let _ = writeln!(out, "-----------------------------------");
    let _ = writeln!(
        out,
        "Type    | Already-Road | Already-Bldg | Unique-New | Unique-%"
    );
    for (label, s) in [
        ("Terrain", &terrain),
        ("Bldg   ", &bldg),
        ("Curve  ", &curve),
    ] {
        let pct = if s.both_resolved > 0 {
            100.0 * s.unique_new as f64 / s.both_resolved as f64
        } else {
            0.0
        };
        let _ = writeln!(
            out,
            "{} | {:>12} | {:>12} | {:>10} | {:>6.2}%",
            label, s.already_road, s.already_building, s.unique_new, pct
        );
    }
    let _ = writeln!(out);
    let _ = writeln!(out, "CROSS-SECTOR");
    let _ = writeln!(out, "------------");
    let _ = writeln!(out, "Type    | Cross-Sec | Already-Road-CS | Unique-CS-New");
    for (label, s) in [
        ("Terrain", &terrain),
        ("Bldg   ", &bldg),
        ("Curve  ", &curve),
    ] {
        let _ = writeln!(
            out,
            "{} | {:>9} | {:>15} | {:>13}",
            label, s.cross_sector, s.cs_already_road, s.cs_unique_new
        );
    }
    let _ = writeln!(out);
    let _ = writeln!(out, "CURVE LOCATOR SUB-CHECK (DeepSeek 1 H1 verification)");
    let _ = writeln!(out, "----------------------------------------------------");
    let curve_loc_pct = if curve.total > 0 {
        100.0 * curve.locator_nonzero_any as f64 / curve.total as f64
    } else {
        0.0
    };
    let h1_verdict = if curve_loc_pct < 20.0 {
        "H1 BESTAETIGT (locators sparse)"
    } else if curve_loc_pct > 80.0 {
        "DEEP-DIVE NOETIG (locators dominant)"
    } else {
        "REVIEW (in [20%, 80%], judgement call)"
    };
    let _ = writeln!(
        out,
        "Curve items with any non-zero locator : {} / {} ({:.2}%)",
        curve.locator_nonzero_any, curve.total, curve_loc_pct
    );
    let _ = writeln!(
        out,
        "Total non-zero locator slots          : {}",
        curve.locator_nonzero_count
    );
    let _ = writeln!(
        out,
        "Verdict                               : {}",
        h1_verdict
    );
    let _ = writeln!(out);
    let _ = writeln!(out, "EMPFEHLUNGEN");
    let _ = writeln!(out, "------------");
    let _ = writeln!(out, "{}", recommend("1  (Terrain)  ", &terrain));
    let _ = writeln!(out, "{}", recommend("2  (Buildings)", &bldg));
    let _ = writeln!(out, "{}", recommend("44 (Curve)    ", &curve));
    let _ = writeln!(out);
    if !hex_dumps.is_empty() {
        let _ = writeln!(
            out,
            "PARSE-FAILURES (first 3 hex dumps for manual inspection)"
        );
        let _ = writeln!(
            out,
            "--------------------------------------------------------"
        );
        for d in &hex_dumps {
            let _ = writeln!(out, "{d}");
        }
    }

    if let Some(parent) = output_path.parent() {
        std::fs::create_dir_all(parent).ok();
    }
    std::fs::write(&output_path, &out).expect("write census");
    let claude_copy = PathBuf::from("outputs/claude/item_census.txt");
    if let Some(parent) = claude_copy.parent() {
        std::fs::create_dir_all(parent).ok();
    }
    std::fs::write(&claude_copy, &out).expect("write claude copy");
    eprintln!("-> {} ({} bytes)", output_path.display(), out.len());
    eprintln!("-> {} (copy)", claude_copy.display());

    if !bldg_sanity_ok {
        eprintln!(
            "WARNING: Buildings sanity check failed (got {}/{}, expected 606/483)",
            bldg.total, bldg.both_resolved
        );
        std::process::exit(2);
    }
}
