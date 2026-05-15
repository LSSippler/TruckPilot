//! `truckpilot-archive-audit` — Phase 5.26a Reframe-B.
//!
//! Per-archive diagnostic that ranks H1/H2/H3 hypotheses for the
//! DLC-edge-extraction bottleneck:
//!   H1: DLC sectors fail to parse more often than base_map.
//!   H2: DLC roads parse but reference cross-archive nodes that don't
//!       resolve in node_lookup.
//!   H3: DLC prefab counts are normal but their cliques don't generate
//!       edges (singleton rate stays high regardless).
//!
//! Diagnose-only. No production-code changes. Reuses parse_sector +
//! audit_sector + mod_loader::from_directories.

use std::collections::{HashMap, HashSet};
use std::fmt::Write as _;
use std::path::PathBuf;

use truckpilot_map_parser::sector::{audit_sector, parse_sector};
use truckpilot_map_parser::{Archive, HashFsArchive, ModLoadOrder, ZipArchive};

const ITEM_TYPE_NAMES: &[(u32, &str)] = &[
    (1, "terrain"),
    (2, "buildings"),
    (3, "road"),
    (4, "prefab"),
    (5, "model"),
    (6, "company"),
    (7, "service"),
    (8, "cut_plane"),
    (12, "city"),
    (18, "map_overlay"),
    (19, "ferry"),
    (22, "garage"),
    (34, "trigger"),
    (35, "fuel_pump"),
    (36, "sign"),
    (37, "bus_stop"),
    (38, "traffic_area"),
    (39, "bezier_patch"),
    (41, "trajectory"),
    (42, "map_area"),
    (43, "far_model"),
    (44, "curve"),
    (46, "cutscene"),
    (48, "visibility_area"),
];

fn item_type_name(t: u32) -> String {
    ITEM_TYPE_NAMES
        .iter()
        .find(|(k, _)| *k == t)
        .map(|(_, n)| n.to_string())
        .unwrap_or_else(|| format!("type_{t}"))
}

#[derive(Default)]
struct ArchiveStats {
    name: String,
    sectors_total: usize,
    sectors_parsed_ok: usize,
    sectors_audit_failed: usize,
    sectors_likely_sized: usize,
    failure_handlers: HashMap<u32, usize>,
    items_road: usize,
    items_prefab: usize,
    items_building: usize,
    items_curve: usize,
    items_terrain: usize,
    nodes_defined: usize,
    nodes_singletons: usize,
    roads: Vec<(u64, u64)>,
    roads_both_resolved: usize,
    roads_one_unresolved: usize,
    roads_unresolved: usize,
    cross_archive_roads: usize,
    cross_archive_resolved: usize,
}

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
}

fn parse_args() -> (PathBuf, Option<PathBuf>, PathBuf, PathBuf) {
    let mut ets2_dir: Option<PathBuf> = None;
    let mut mods_dir: Option<PathBuf> = None;
    let mut graph = PathBuf::from("graph.json");
    let mut output = PathBuf::from("outputs/archive_audit.txt");
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

fn main() {
    let (ets2_dir, mods_dir_opt, graph_path, output_path) = parse_args();
    let mods_dir = mods_dir_opt.unwrap_or_else(default_mods_dir);

    eprintln!("loading graph from {} ...", graph_path.display());
    let bytes = std::fs::read(&graph_path).expect("read graph.json");
    let graph: GraphFile = serde_json::from_slice(&bytes).expect("parse graph.json");
    eprintln!("  {} nodes, {} edges", graph.nodes.len(), graph.edges.len());

    let known_uids: HashSet<u64> = graph.nodes.iter().map(|n| n.uid).collect();
    let mut degree: HashMap<u64, u32> = HashMap::with_capacity(graph.nodes.len());
    for n in &graph.nodes {
        degree.insert(n.uid, 0);
    }
    for e in &graph.edges {
        *degree.entry(e.from).or_insert(0) += 1;
        *degree.entry(e.to).or_insert(0) += 1;
    }

    let order = ModLoadOrder::from_directories(&ets2_dir, &mods_dir).expect("mod load order");
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
            if f.ends_with(".base") {
                path_to_arch.insert(f, idx);
            }
        }
    }
    let mut sector_paths: Vec<(String, usize)> = path_to_arch.into_iter().collect();
    sector_paths.sort();
    eprintln!("scanning {} `.base` sectors ...", sector_paths.len());

    let mut stats: HashMap<usize, ArchiveStats> = HashMap::new();
    for (_, idx) in &sector_paths {
        stats.entry(*idx).or_insert_with(|| ArchiveStats {
            name: archive_names[*idx].clone(),
            ..Default::default()
        });
    }

    let mut node_to_arch: HashMap<u64, usize> = HashMap::with_capacity(1_200_000);

    for (path, idx) in &sector_paths {
        let Ok(data) = archives[*idx].read_path(path) else {
            continue;
        };
        let s = stats.get_mut(idx).unwrap();
        s.sectors_total += 1;

        let parsed = match parse_sector(&data) {
            Ok(p) => p,
            Err(_) => continue,
        };
        s.sectors_parsed_ok += 1;

        s.items_road += parsed.roads.len();
        s.items_prefab += parsed.prefabs.len();
        s.items_building += parsed.buildings.len();
        for n in &parsed.nodes {
            s.nodes_defined += 1;
            node_to_arch.entry(n.uid).or_insert(*idx);
        }
        for r in &parsed.roads {
            s.roads.push((r.node_a, r.node_b));
        }

        let report = audit_sector(&data);
        let audit_b = report.items.iter().filter(|i| i.item_type == 2).count();
        let audit_c = report.items.iter().filter(|i| i.item_type == 44).count();
        let audit_t = report.items.iter().filter(|i| i.item_type == 1).count();
        let audit_consistent = audit_b == parsed.buildings.len();
        if audit_consistent {
            s.items_curve += audit_c;
            s.items_terrain += audit_t;
        }
        match (&report.failure, audit_consistent) {
            (Some(f), true) => {
                s.sectors_audit_failed += 1;
                *s.failure_handlers.entry(f.raw_type).or_insert(0) += 1;
            }
            (Some(_), false) => {
                s.sectors_likely_sized += 1;
            }
            _ => {}
        }
    }

    // Pass 2: classify roads + count singletons per archive.
    let stat_indices: Vec<usize> = stats.keys().copied().collect();
    for idx in stat_indices {
        let mut s = stats.remove(&idx).unwrap();
        for (a, b) in &s.roads {
            let a_known = known_uids.contains(a);
            let b_known = known_uids.contains(b);
            match (a_known, b_known) {
                (true, true) => s.roads_both_resolved += 1,
                (false, false) => s.roads_unresolved += 1,
                _ => s.roads_one_unresolved += 1,
            }
            if let (Some(aa), Some(bb)) = (node_to_arch.get(a), node_to_arch.get(b)) {
                if aa != bb {
                    s.cross_archive_roads += 1;
                    if a_known && b_known {
                        s.cross_archive_resolved += 1;
                    }
                }
            }
        }
        for (uid, owner) in &node_to_arch {
            if *owner == idx && degree.get(uid).copied().unwrap_or(0) == 0 {
                s.nodes_singletons += 1;
            }
        }
        stats.insert(idx, s);
    }

    let mut entries: Vec<&ArchiveStats> = stats.values().collect();
    entries.sort_by_key(|s| std::cmp::Reverse(s.sectors_total));

    let base_idx = entries.iter().position(|s| s.name == "base_map.scs");
    let base_fail_rate = base_idx
        .map(|i| {
            let s = entries[i];
            if s.sectors_total > 0 {
                100.0 * s.sectors_audit_failed as f64 / s.sectors_total as f64
            } else {
                0.0
            }
        })
        .unwrap_or(0.0);
    let dlcs: Vec<&&ArchiveStats> = entries
        .iter()
        .filter(|s| s.name != "base_map.scs")
        .collect();
    let dlc_avg_fail = {
        let mut t = 0usize;
        let mut f = 0usize;
        for s in &dlcs {
            t += s.sectors_total;
            f += s.sectors_audit_failed;
        }
        if t > 0 {
            100.0 * f as f64 / t as f64
        } else {
            0.0
        }
    };

    let cross_resolve_rate = |s: &ArchiveStats| {
        if s.cross_archive_roads > 0 {
            100.0 * s.cross_archive_resolved as f64 / s.cross_archive_roads as f64
        } else {
            -1.0
        }
    };

    let mut out = String::new();
    let _ = writeln!(out, "============================================");
    let _ = writeln!(out, "TRUCKPILOT ARCHIVE AUDIT — Phase 5.26a Reframe-B");
    let _ = writeln!(
        out,
        "Source: production load order ({} archives, {} owning .base sectors)",
        archives.len(),
        entries.len()
    );
    let _ = writeln!(out, "============================================");
    let _ = writeln!(out);
    let _ = writeln!(out, "PER-ARCHIVE PARSE-SUCCESS (H1 evidence)");
    let _ = writeln!(out, "---------------------------------------");
    let _ = writeln!(out, "Archive                    | Sectors |  OK   | AudFail | Sized? | Fail-% | Dominant Failing Type");
    for s in &entries {
        let dom = s
            .failure_handlers
            .iter()
            .max_by_key(|(_, c)| *c)
            .map(|(t, c)| format!("{} ({}) x{}", item_type_name(*t), t, c))
            .unwrap_or_else(|| "-".into());
        let pct = if s.sectors_total > 0 {
            100.0 * s.sectors_audit_failed as f64 / s.sectors_total as f64
        } else {
            0.0
        };
        let _ = writeln!(
            out,
            "{:<26} | {:>7} | {:>5} | {:>7} | {:>6} | {:>5.1}% | {}",
            s.name,
            s.sectors_total,
            s.sectors_parsed_ok,
            s.sectors_audit_failed,
            s.sectors_likely_sized,
            pct,
            dom
        );
    }
    let _ = writeln!(out);
    let _ = writeln!(out, "PER-ARCHIVE ITEM-COUNTS");
    let _ = writeln!(out, "-----------------------");
    let _ = writeln!(
        out,
        "Archive                    |   Roads | Prefabs |    Bldg | Curve | Terrain"
    );
    for s in &entries {
        let _ = writeln!(
            out,
            "{:<26} | {:>7} | {:>7} | {:>7} | {:>5} | {:>7}",
            s.name, s.items_road, s.items_prefab, s.items_building, s.items_curve, s.items_terrain
        );
    }
    let _ = writeln!(out);
    let _ = writeln!(out, "PER-ARCHIVE NODE-CONNECTIVITY (H3 evidence)");
    let _ = writeln!(out, "-------------------------------------------");
    let _ = writeln!(
        out,
        "Archive                    | Defined | Singletons | Sgl-%"
    );
    for s in &entries {
        let pct = if s.nodes_defined > 0 {
            100.0 * s.nodes_singletons as f64 / s.nodes_defined as f64
        } else {
            0.0
        };
        let _ = writeln!(
            out,
            "{:<26} | {:>7} | {:>10} | {:>5.1}%",
            s.name, s.nodes_defined, s.nodes_singletons, pct
        );
    }
    let _ = writeln!(out);
    let _ = writeln!(out, "CROSS-ARCHIVE ROAD RESOLUTION (H2 evidence)");
    let _ = writeln!(out, "-------------------------------------------");
    let _ = writeln!(out, "Archive                    |   Roads | Both-Res | One-Unr | No-Res | CrossArch | CrossRes-%");
    for s in &entries {
        let cr = cross_resolve_rate(s);
        let cr_str = if cr >= 0.0 {
            format!("{:>8.1}%", cr)
        } else {
            "        -".into()
        };
        let _ = writeln!(
            out,
            "{:<26} | {:>7} | {:>8} | {:>7} | {:>6} | {:>9} | {}",
            s.name,
            s.items_road,
            s.roads_both_resolved,
            s.roads_one_unresolved,
            s.roads_unresolved,
            s.cross_archive_roads,
            cr_str
        );
    }
    let _ = writeln!(out);

    let h1_strength = if dlc_avg_fail > 5.0 * base_fail_rate.max(0.01) {
        "DECISIVE"
    } else if dlc_avg_fail > 2.0 * base_fail_rate.max(0.01) {
        "STRONG"
    } else {
        "WEAK"
    };

    let dlcs_with_low_cross = dlcs
        .iter()
        .filter(|s| {
            let r = cross_resolve_rate(s);
            (0.0..50.0).contains(&r)
        })
        .count();
    let base_cross = base_idx
        .map(|i| cross_resolve_rate(entries[i]))
        .unwrap_or(-1.0);
    let majority_dlcs_low_cross = !dlcs.is_empty() && dlcs_with_low_cross * 2 > dlcs.len();
    let h2_strength = if majority_dlcs_low_cross
        && base_cross > 90.0
        && dlc_avg_fail < 2.0 * base_fail_rate.max(0.01)
    {
        if dlcs.iter().any(|s| {
            let r = cross_resolve_rate(s);
            (0.0..20.0).contains(&r)
        }) {
            "DECISIVE"
        } else {
            "STRONG"
        }
    } else if majority_dlcs_low_cross {
        "STRONG"
    } else {
        "WEAK"
    };

    let dlc_avg_singleton = {
        let mut tot = 0usize;
        let mut sgl = 0usize;
        for s in &dlcs {
            tot += s.nodes_defined;
            sgl += s.nodes_singletons;
        }
        if tot > 0 {
            100.0 * sgl as f64 / tot as f64
        } else {
            0.0
        }
    };
    let h3_strength = if dlc_avg_singleton > 60.0 {
        "STRONG"
    } else {
        "WEAK"
    };

    let _ = writeln!(out, "HYPOTHESIS RANKING");
    let _ = writeln!(out, "------------------");
    let _ = writeln!(out, "H1 (parse-failures higher in DLCs) : {}", h1_strength);
    let _ = writeln!(
        out,
        "  base fail-rate: {:.2}%, DLC-avg fail-rate: {:.2}%, ratio {:.1}x",
        base_fail_rate,
        dlc_avg_fail,
        if base_fail_rate > 0.01 {
            dlc_avg_fail / base_fail_rate
        } else {
            0.0
        }
    );
    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "H2 (cross-arch UID resolution broken) : {}",
        h2_strength
    );
    let _ = writeln!(
        out,
        "  base cross-resolve: {:.1}%, DLCs with <50% cross-resolve: {}/{}",
        base_cross,
        dlcs_with_low_cross,
        dlcs.len()
    );
    let _ = writeln!(out);
    let _ = writeln!(out, "H3 (prefab clique semantics differ) : {}", h3_strength);
    let _ = writeln!(out, "  DLC-avg singleton-rate: {:.1}%", dlc_avg_singleton);
    let _ = writeln!(out);

    let dominant = [
        (h1_strength, "H1"),
        (h2_strength, "H2"),
        (h3_strength, "H3"),
    ]
    .iter()
    .max_by_key(|(s, _)| match *s {
        "DECISIVE" => 3,
        "STRONG" => 2,
        _ => 1,
    })
    .map(|(_, n)| *n)
    .unwrap_or("-");

    let _ = writeln!(out, "RECOMMENDATION");
    let _ = writeln!(out, "--------------");
    match dominant {
        "H1" => {
            let _ = writeln!(
                out,
                "Strongest signal: H1. Hex-dump 10 failing sectors from worst DLC,"
            );
            let _ = writeln!(
                out,
                "identify the failing item-type per Phase 5.20 BezierPatch playbook."
            );
            let worst = dlcs
                .iter()
                .filter(|s| s.sectors_total >= 10)
                .max_by(|a, b| {
                    let ra = a.sectors_audit_failed as f64 / a.sectors_total.max(1) as f64;
                    let rb = b.sectors_audit_failed as f64 / b.sectors_total.max(1) as f64;
                    ra.partial_cmp(&rb).unwrap_or(std::cmp::Ordering::Equal)
                });
            if let Some(w) = worst {
                let _ = writeln!(
                    out,
                    "Worst DLC: {} ({}/{} failed = {:.1}%).",
                    w.name,
                    w.sectors_audit_failed,
                    w.sectors_total,
                    100.0 * w.sectors_audit_failed as f64 / w.sectors_total as f64
                );
            }
        }
        "H2" => {
            let _ = writeln!(
                out,
                "Strongest signal: H2. DLC roads' cross-archive nodes don't resolve."
            );
            let _ = writeln!(
                out,
                "Likely cause: trailing-node blocks of DLC sectors are not parsed"
            );
            let _ = writeln!(
                out,
                "(sector failed before reaching tail) so their nodes never enter"
            );
            let _ = writeln!(out, "node_lookup. Fix upstream item-handler.");
        }
        "H3" => {
            let _ = writeln!(
                out,
                "Strongest signal: H3. DLC items parse and prefabs count normally"
            );
            let _ = writeln!(
                out,
                "but cliques don't generate edges. Sample DLC prefab vs base prefab"
            );
            let _ = writeln!(
                out,
                "and inspect connected_node_uids semantics + prefab descriptor lookup."
            );
        }
        _ => {
            let _ = writeln!(
                out,
                "All three hypotheses weak — fourth hypothesis required."
            );
            let _ = writeln!(
                out,
                "Candidates: prefab-template-token resolution (no .ppd parse yet);"
            );
            let _ = writeln!(
                out,
                "vis_uids cross-sector edges (Phase 5.13 was rejected, may reopen)."
            );
        }
    }

    if let Some(parent) = output_path.parent() {
        std::fs::create_dir_all(parent).ok();
    }
    std::fs::write(&output_path, &out).expect("write archive_audit.txt");
    let claude = PathBuf::from("outputs/claude/archive_audit.txt");
    if let Some(parent) = claude.parent() {
        std::fs::create_dir_all(parent).ok();
    }
    std::fs::write(&claude, &out).expect("write claude copy");
    eprintln!("-> {} ({} bytes)", output_path.display(), out.len());
    eprintln!("-> {} (copy)", claude.display());
}
