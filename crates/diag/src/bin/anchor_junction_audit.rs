//! `truckpilot-anchor-junction-audit` — Phase 5.26d (H4b probe).
//!
//! Counts how many Single-Anchor-Items (Sign, Model, FarModel,
//! MapOverlay, VisibilityArea, BusStop) reference the same Node-UID.
//! If multiple ignored anchor items share a Node, that node is an
//! IMPLICIT JUNCTION — currently invisible to the graph builder.
//!
//! Diagnose-only. Uses audit_sector() to walk all items at known byte
//! offsets, then direct-slices Node-UIDs from the raw bytes per type.
//! No cursor, no production-code changes, no ParsedSector mutation.
//!
//! Anchor offsets (from AuditedItem.start_offset; item_type u32 +
//! kdop_item 53 B = first post-kdop byte at +57):
//!   Sign (36)         : +65            (token Model, then Node)
//!   FarModel (43)     : +81            (3 tokens, then Node)
//!   MapOverlay (18)   : +65            (1 token, then Node)
//!   VisibilityArea(48): +57            (right after kdop)
//!   BusStop (37)      : +57, +65, +73  (3 Nodes)
//!   Model (5)         : +85 + 8*N      (3 tokens + count u32 at +81 + N*u64)

use std::collections::{HashMap, HashSet};
use std::fmt::Write as _;
use std::path::PathBuf;

use truckpilot_map_parser::sector::{audit_sector, parse_sector};
use truckpilot_map_parser::{Archive, HashFsArchive, ModLoadOrder, ZipArchive};

const T_MODEL: u32 = 5;
const T_MAP_OVERLAY: u32 = 18;
const T_SIGN: u32 = 36;
const T_BUS_STOP: u32 = 37;
const T_FAR_MODEL: u32 = 43;
const T_VIS_AREA: u32 = 48;

fn type_name(t: u32) -> &'static str {
    match t {
        T_MODEL => "Model",
        T_MAP_OVERLAY => "MapOverlay",
        T_SIGN => "Sign",
        T_BUS_STOP => "BusStop",
        T_FAR_MODEL => "FarModel",
        T_VIS_AREA => "VisibilityArea",
        _ => "?",
    }
}

#[derive(serde::Deserialize)]
struct GraphFile { nodes: Vec<NodeJson>, edges: Vec<EdgeJson>, prefabs: Vec<PrefabJson> }
#[derive(serde::Deserialize)]
struct NodeJson { uid: u64 }
#[derive(serde::Deserialize)]
struct EdgeJson { from: u64, to: u64 }
#[derive(serde::Deserialize)]
struct PrefabJson { connected_node_uids: Vec<u64> }

fn read_u64(slice: &[u8], off: usize) -> Option<u64> {
    let end = off.checked_add(8)?;
    if end > slice.len() { return None; }
    Some(u64::from_le_bytes(slice[off..end].try_into().unwrap()))
}

fn read_u32(slice: &[u8], off: usize) -> Option<u32> {
    let end = off.checked_add(4)?;
    if end > slice.len() { return None; }
    Some(u32::from_le_bytes(slice[off..end].try_into().unwrap()))
}

fn extract_anchors(item_bytes: &[u8], item_type: u32) -> Vec<u64> {
    const KDOP_END: usize = 4 + 53;
    let mut uids = Vec::new();
    match item_type {
        T_VIS_AREA => {
            if let Some(u) = read_u64(item_bytes, KDOP_END) { uids.push(u); }
        }
        T_SIGN | T_MAP_OVERLAY => {
            if let Some(u) = read_u64(item_bytes, KDOP_END + 8) { uids.push(u); }
        }
        T_FAR_MODEL => {
            if let Some(u) = read_u64(item_bytes, KDOP_END + 24) { uids.push(u); }
        }
        T_BUS_STOP => {
            for off in [KDOP_END, KDOP_END + 8, KDOP_END + 16] {
                if let Some(u) = read_u64(item_bytes, off) { uids.push(u); }
            }
        }
        T_MODEL => {
            if let Some(n) = read_u32(item_bytes, KDOP_END + 24) {
                let node_off = KDOP_END + 28 + 8 * (n as usize);
                if let Some(u) = read_u64(item_bytes, node_off) { uids.push(u); }
            }
        }
        _ => {}
    }
    uids
}

fn parse_args() -> (PathBuf, Option<PathBuf>, PathBuf, PathBuf) {
    let mut ets2_dir: Option<PathBuf> = None;
    let mut mods_dir: Option<PathBuf> = None;
    let mut graph = PathBuf::from("graph.json");
    let mut output = PathBuf::from("outputs/h4b_anchor_junction_audit.txt");
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--ets2-dir" => { ets2_dir = Some(PathBuf::from(&argv[i + 1])); i += 2; }
            "--mods-dir" => { mods_dir = Some(PathBuf::from(&argv[i + 1])); i += 2; }
            "--graph" => { graph = PathBuf::from(&argv[i + 1]); i += 2; }
            "--output" => { output = PathBuf::from(&argv[i + 1]); i += 2; }
            _ => i += 1,
        }
    }
    (ets2_dir.expect("--ets2-dir required"), mods_dir, graph, output)
}

fn default_mods_dir() -> PathBuf {
    dirs::document_dir().or_else(dirs::home_dir).unwrap_or_default()
        .join("Euro Truck Simulator 2/mod")
}

fn pct(num: u64, denom: u64) -> f64 {
    if denom == 0 { 0.0 } else { 100.0 * num as f64 / denom as f64 }
}

fn main() {
    let (ets2_dir, mods_dir_opt, graph_path, output_path) = parse_args();
    let mods_dir = mods_dir_opt.unwrap_or_else(default_mods_dir);

    eprintln!("loading graph from {} ...", graph_path.display());
    let bytes = std::fs::read(&graph_path).expect("read graph.json");
    let graph: GraphFile = serde_json::from_slice(&bytes).expect("parse graph.json");
    eprintln!("  {} nodes, {} edges, {} prefabs",
        graph.nodes.len(), graph.edges.len(), graph.prefabs.len());

    let mut degree: HashMap<u64, u32> = HashMap::with_capacity(graph.nodes.len());
    for n in &graph.nodes { degree.insert(n.uid, 0); }
    for e in &graph.edges {
        *degree.entry(e.from).or_insert(0) += 1;
        *degree.entry(e.to).or_insert(0) += 1;
    }
    let mut prefab_attached: HashSet<u64> = HashSet::new();
    for p in &graph.prefabs {
        for u in &p.connected_node_uids { prefab_attached.insert(*u); }
    }

    let order = ModLoadOrder::from_directories(&ets2_dir, &mods_dir).expect("mod load order");
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
    eprintln!("opened {} archives", archives.len());

    let mut path_to_arch: HashMap<String, usize> = HashMap::new();
    for (idx, arc) in archives.iter().enumerate() {
        let mut files = arc.list_files();
        if files.is_empty() {
            if let Some(h) = arc.as_any().downcast_ref::<HashFsArchive>() {
                files = h.probe_sector_paths();
            }
        }
        for f in files {
            if f.ends_with(".base") { path_to_arch.insert(f, idx); }
        }
    }
    let mut sector_paths: Vec<(String, usize)> = path_to_arch.into_iter().collect();
    sector_paths.sort();
    eprintln!("scanning {} `.base` sectors ...", sector_paths.len());

    let target = [T_MODEL, T_MAP_OVERLAY, T_SIGN, T_BUS_STOP, T_FAR_MODEL, T_VIS_AREA];

    let mut node_items: HashMap<u64, Vec<u32>> = HashMap::with_capacity(2_000_000);
    let mut item_counts: HashMap<u32, usize> = HashMap::new();
    let mut sectors_audited = 0usize;

    for (path, idx) in &sector_paths {
        let Ok(data) = archives[*idx].read_path(path) else { continue };
        if parse_sector(&data).is_err() { continue; }
        sectors_audited += 1;
        let report = audit_sector(&data);
        for it in &report.items {
            if !target.contains(&it.item_type) { continue; }
            let item_bytes = &data[it.start_offset..it.end_offset];
            let uids = extract_anchors(item_bytes, it.item_type);
            *item_counts.entry(it.item_type).or_insert(0) += 1;
            for u in uids {
                if u == 0 { continue; }
                node_items.entry(u).or_default().push(it.item_type);
            }
        }
    }

    eprintln!("  sectors audited: {}", sectors_audited);
    eprintln!("  unique anchor nodes: {}", node_items.len());

    let total_items: usize = item_counts.values().sum();
    let mut nodes_with_1 = 0usize;
    let mut nodes_with_2 = 0usize;
    let mut nodes_with_3plus = 0usize;
    let mut combo_counts: HashMap<Vec<u32>, usize> = HashMap::new();
    let mut junction_singletons = 0usize;
    let mut junction_non_singletons = 0usize;
    let mut junction_known_in_graph = 0usize;
    let mut junction_unknown_in_graph = 0usize;

    for (uid, types) in &node_items {
        let mut combo: Vec<u32> = types.clone();
        combo.sort();
        combo.dedup();
        match types.len() {
            1 => nodes_with_1 += 1,
            2 => nodes_with_2 += 1,
            _ => nodes_with_3plus += 1,
        }
        *combo_counts.entry(combo).or_insert(0) += 1;
        if types.len() >= 2 {
            let known = degree.contains_key(uid);
            let in_prefab = prefab_attached.contains(uid);
            let deg = degree.get(uid).copied().unwrap_or(0);
            if known {
                junction_known_in_graph += 1;
                if deg == 0 && !in_prefab { junction_singletons += 1; }
                else { junction_non_singletons += 1; }
            } else {
                junction_unknown_in_graph += 1;
            }
        }
    }

    let total_unique = node_items.len();
    let junction_total = nodes_with_2 + nodes_with_3plus;

    let mut out = String::new();
    let _ = writeln!(out, "============================================");
    let _ = writeln!(out, "TRUCKPILOT ANCHOR-JUNCTION AUDIT — Phase 5.26d (H4b)");
    let _ = writeln!(out, "Source: production load order ({} archives, {} sectors)",
        archives.len(), sectors_audited);
    let _ = writeln!(out, "============================================");
    let _ = writeln!(out);
    let _ = writeln!(out, "SECTION 1: ITEM-COUNTS PER TYPE");
    let _ = writeln!(out, "-------------------------------");
    let _ = writeln!(out, "Item-Type             |        Count");
    let _ = writeln!(out, "----------------------|-------------");
    for &t in &target {
        let c = item_counts.get(&t).copied().unwrap_or(0);
        let _ = writeln!(out, "{:<14} ({:>2})    | {:>12}", type_name(t), t, c);
    }
    let _ = writeln!(out, "----------------------|-------------");
    let _ = writeln!(out, "{:<22}| {:>12}", "TOTAL", total_items);
    let _ = writeln!(out);

    let _ = writeln!(out, "SECTION 2: ANCHOR-DISTRIBUTION (Nodes by # referencing items)");
    let _ = writeln!(out, "-------------------------------------------------------------");
    let _ = writeln!(out, "Items per Node   |        Count |    %");
    let _ = writeln!(out, "-----------------|--------------|------");
    let _ = writeln!(out, "1                | {:>12} | {:>5.2}%", nodes_with_1, pct(nodes_with_1 as u64, total_unique as u64));
    let _ = writeln!(out, "2                | {:>12} | {:>5.2}%", nodes_with_2, pct(nodes_with_2 as u64, total_unique as u64));
    let _ = writeln!(out, "3+               | {:>12} | {:>5.2}%", nodes_with_3plus, pct(nodes_with_3plus as u64, total_unique as u64));
    let _ = writeln!(out, "-----------------|--------------|------");
    let _ = writeln!(out, "{:<16} | {:>12} | 100.00%", "Total unique", total_unique);
    let _ = writeln!(out);

    let _ = writeln!(out, "SECTION 3: ITEM-TYPE-COMBO HISTOGRAM (Top 20, only >=2-item junctions)");
    let _ = writeln!(out, "----------------------------------------------------------------------");
    let mut combos: Vec<(&Vec<u32>, &usize)> = combo_counts.iter()
        .filter(|(c, _)| c.len() >= 2)
        .collect();
    combos.sort_by_key(|(_, c)| std::cmp::Reverse(**c));
    let _ = writeln!(out, "Combination                        |   Count");
    let _ = writeln!(out, "-----------------------------------|---------");
    for (combo, count) in combos.iter().take(20) {
        let names: Vec<&str> = combo.iter().map(|t| type_name(*t)).collect();
        let label = names.join(" + ");
        let _ = writeln!(out, "{:<35}| {:>8}", label, count);
    }
    let _ = writeln!(out);

    let _ = writeln!(out, "SECTION 4: SINGLETON-REDUCTION HYPOTHESIS");
    let _ = writeln!(out, "-----------------------------------------");
    let _ = writeln!(out, "Junction nodes total (>=2 items)         : {:>10}", junction_total);
    let _ = writeln!(out, "  known in graph.json                    : {:>10}", junction_known_in_graph);
    let _ = writeln!(out, "    of which singletons (deg=0, no prefab): {:>10}", junction_singletons);
    let _ = writeln!(out, "    of which already connected           : {:>10}", junction_non_singletons);
    let _ = writeln!(out, "  unknown in graph.json (no node defined): {:>10}", junction_unknown_in_graph);
    let total_singletons_today = degree.iter()
        .filter(|(uid, d)| **d == 0 && !prefab_attached.contains(*uid))
        .count();
    let _ = writeln!(out, "Total singletons today                   : {:>10}", total_singletons_today);
    let _ = writeln!(out, "% of singletons covered by junctions     : {:>9.2}%",
        pct(junction_singletons as u64, total_singletons_today as u64));
    let _ = writeln!(out);

    let _ = writeln!(out, "SECTION 5: GO/SKIP RECOMMENDATION");
    let _ = writeln!(out, "---------------------------------");
    let _ = writeln!(out, "Junction-Pool size (nodes with >=2 items): {}", junction_total);
    let _ = writeln!(out, "Junction-Singletons reachable            : {}", junction_singletons);
    let verdict = if junction_total > 50_000 && junction_singletons > 20_000 {
        "GO — significant junction pool AND significant singleton coverage."
    } else if junction_total > 50_000 {
        "REVIEW — large pool but few singletons gained; phantom-edge risk."
    } else if junction_total < 10_000 || junction_singletons < 5_000 {
        "SKIP — too few junctions for ROI (Buildings-Pattern <50k threshold)."
    } else {
        "REVIEW — borderline; manual evaluation needed."
    };
    let _ = writeln!(out, "Verdict: {}", verdict);
    let _ = writeln!(out);
    let _ = writeln!(out, "PHANTOM-ROUTING-RISK NOTE");
    let _ = writeln!(out, "-------------------------");
    let _ = writeln!(out, "Junction edges between Sign/Model/MapOverlay/etc. would NOT be");
    let _ = writeln!(out, "drivable roads. They are visual-anchor co-references. Implementing");
    let _ = writeln!(out, "them as routing edges risks A* finding shortcuts the truck cannot");
    let _ = writeln!(out, "physically traverse. Top-3 combos in Section 3 above show which");
    let _ = writeln!(out, "type pairs dominate; combos like Sign+Model are most suspect, while");
    let _ = writeln!(out, "BusStop+Sign or BusStop+Model are more plausible (BusStop carries");
    let _ = writeln!(out, "3 nodes that already span a road segment).");

    if let Some(parent) = output_path.parent() { std::fs::create_dir_all(parent).ok(); }
    std::fs::write(&output_path, &out).expect("write");
    let claude = PathBuf::from("outputs/claude/h4b_anchor_junction_audit.txt");
    if let Some(parent) = claude.parent() { std::fs::create_dir_all(parent).ok(); }
    std::fs::write(&claude, &out).expect("write claude copy");
    eprintln!("-> {} ({} bytes)", output_path.display(), out.len());
    eprintln!("-> {} (copy)", claude.display());
}
