//! `truckpilot-vis-uid-audit` — Phase 5.26a Reframe-C (H4a probe).
//!
//! Re-runs the Phase 5.13 vis_uids hypothesis test against the FULL
//! production load order (1500 sectors across 124 archives) instead of
//! the original base_map-only pool of 282 sectors. Phase 5.13 rejected
//! the hypothesis decisively: parsed Node-UIDs live in `0x0029XXXX...`,
//! vis_uids in `0x45-0x5DXXXX...` -- disjoint UID namespaces, so
//! vis_uids are visibility/LOD asset hashes, not node references. This
//! audit verifies the conclusion holds at scale and quantifies the
//! hypothetical edge-generation gain (should be ~0).

use std::collections::{HashMap, HashSet};
use std::fmt::Write as _;
use std::path::PathBuf;

use truckpilot_map_parser::sector::parse_sector;
use truckpilot_map_parser::{Archive, HashFsArchive, ModLoadOrder, ZipArchive};

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

/// Sweeps the trailing layout `... | node_count u32 | nodes(56*N) |
/// vis_count u32 | vis_uids(8*M)` and returns `(N, vis_uids)` if a
/// plausible layout fits the data exactly.
fn sweep_vis_layout(data: &[u8]) -> Option<(u32, Vec<u64>)> {
    const NODE_BYTES: usize = 56;
    const MAX_N: usize = 4096;
    const MAX_M: usize = 4096;
    let total = data.len();
    for m in 0..=MAX_M {
        let vis_block = 4 + m * 8;
        if vis_block > total {
            break;
        }
        let vis_count_pos = total - vis_block;
        if vis_count_pos < 4 + 16 {
            continue;
        }
        let vis_count =
            u32::from_le_bytes(data[vis_count_pos..vis_count_pos + 4].try_into().unwrap());
        if vis_count as usize != m {
            continue;
        }
        let nodes_end = vis_count_pos;
        for n in 0..=MAX_N {
            let block = 4 + n * NODE_BYTES;
            if block > nodes_end {
                break;
            }
            let count_pos = nodes_end - block;
            if count_pos < 16 {
                break;
            }
            let count_at = u32::from_le_bytes(data[count_pos..count_pos + 4].try_into().unwrap());
            if count_at as usize != n {
                continue;
            }
            if n > 0 {
                let first_uid =
                    u64::from_le_bytes(data[count_pos + 4..count_pos + 12].try_into().unwrap());
                if first_uid == 0 {
                    continue;
                }
            }
            let mut vis_uids = Vec::with_capacity(m);
            for k in 0..m {
                let pos = vis_count_pos + 4 + k * 8;
                vis_uids.push(u64::from_le_bytes(data[pos..pos + 8].try_into().unwrap()));
            }
            return Some((n as u32, vis_uids));
        }
    }
    None
}

fn parse_args() -> (PathBuf, Option<PathBuf>, PathBuf, PathBuf) {
    let mut ets2_dir: Option<PathBuf> = None;
    let mut mods_dir: Option<PathBuf> = None;
    let mut graph = PathBuf::from("graph.json");
    let mut output = PathBuf::from("outputs/vis_uid_audit.txt");
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

fn pct(num: u64, denom: u64) -> f64 {
    if denom == 0 {
        0.0
    } else {
        100.0 * num as f64 / denom as f64
    }
}

#[derive(Default)]
struct ArchiveVis {
    name: String,
    sectors: usize,
    total_vis: usize,
    max: usize,
    sum_for_mean: usize,
    counts: Vec<usize>,
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

    // Pass 1: build node_to_sector + node_to_arch.
    let mut node_to_sector: HashMap<u64, u32> = HashMap::with_capacity(1_200_000);
    let mut node_to_arch: HashMap<u64, usize> = HashMap::with_capacity(1_200_000);
    for (sid, (path, idx)) in sector_paths.iter().enumerate() {
        let Ok(data) = archives[*idx].read_path(path) else {
            continue;
        };
        if let Ok(parsed) = parse_sector(&data) {
            for n in &parsed.nodes {
                node_to_sector.entry(n.uid).or_insert(sid as u32);
                node_to_arch.entry(n.uid).or_insert(*idx);
            }
        }
    }

    // Pass 2: sweep vis_uids per sector, classify.
    let mut sector_vis: Vec<Vec<u64>> = Vec::with_capacity(sector_paths.len());
    let mut arch_stats: HashMap<usize, ArchiveVis> = HashMap::new();
    let mut total_vis: u64 = 0;
    let mut zero_uids: u64 = 0;
    let mut same: u64 = 0;
    let mut cross_sector: u64 = 0;
    let mut cross_archive: u64 = 0;
    let mut unresolved: u64 = 0;
    let mut max_m: usize = 0;
    let mut sectors_zero_m: usize = 0;
    let mut sectors_with_vis: usize = 0;
    let mut sectors_high_vis: usize = 0;
    let mut high_byte_hist: HashMap<u8, u64> = HashMap::new();

    for (sid, (path, idx)) in sector_paths.iter().enumerate() {
        let s = arch_stats.entry(*idx).or_insert_with(|| ArchiveVis {
            name: archive_names[*idx].clone(),
            ..Default::default()
        });
        s.sectors += 1;
        let Ok(data) = archives[*idx].read_path(path) else {
            sector_vis.push(Vec::new());
            continue;
        };
        let uids = match sweep_vis_layout(&data) {
            Some((_, u)) => u,
            None => {
                sector_vis.push(Vec::new());
                sectors_zero_m += 1;
                continue;
            }
        };
        let m = uids.len();
        if m == 0 {
            sectors_zero_m += 1;
        } else {
            sectors_with_vis += 1;
        }
        if m > 1000 {
            sectors_high_vis += 1;
        }
        if m > max_m {
            max_m = m;
        }
        s.total_vis += m;
        s.sum_for_mean += m;
        s.counts.push(m);
        if m > s.max {
            s.max = m;
        }

        for &uid in &uids {
            total_vis += 1;
            if uid == 0 {
                zero_uids += 1;
                unresolved += 1;
                continue;
            }
            let top = (uid >> 56) as u8;
            *high_byte_hist.entry(top).or_insert(0) += 1;
            match (node_to_sector.get(&uid), node_to_arch.get(&uid)) {
                (Some(&owner_sid), Some(&owner_arch)) => {
                    if owner_sid == sid as u32 {
                        same += 1;
                    } else {
                        cross_sector += 1;
                    }
                    if owner_arch != *idx {
                        cross_archive += 1;
                    }
                }
                _ => unresolved += 1,
            }
        }
        sector_vis.push(uids);
    }

    // Hypothetical edge-generation probe: clique first-node x vis_uids per sector.
    let singleton_set: HashSet<u64> = degree
        .iter()
        .filter(|(_, d)| **d == 0)
        .map(|(u, _)| *u)
        .collect();
    let singletons_today: u64 = singleton_set.len() as u64;

    let mut hypothetical_edges = 0usize;
    let mut hypothetical_cross_sector_edges = 0usize;
    let mut singletons_gaining_edge: HashSet<u64> = HashSet::new();

    for (sid, _) in sector_paths.iter().enumerate() {
        let uids = &sector_vis[sid];
        if uids.is_empty() {
            continue;
        }
        let anchor = node_to_sector
            .iter()
            .find(|(_, s)| **s == sid as u32)
            .map(|(uid, _)| *uid);
        let Some(anchor) = anchor else { continue };
        for &uid in uids {
            if uid == 0 || uid == anchor {
                continue;
            }
            if !known_uids.contains(&uid) {
                continue;
            }
            hypothetical_edges += 1;
            if let (Some(&sa), Some(&sb)) = (node_to_sector.get(&anchor), node_to_sector.get(&uid))
            {
                if sa != sb {
                    hypothetical_cross_sector_edges += 1;
                }
            }
            if singleton_set.contains(&uid) {
                singletons_gaining_edge.insert(uid);
            }
            if singleton_set.contains(&anchor) {
                singletons_gaining_edge.insert(anchor);
            }
        }
    }

    let mut out = String::new();
    let _ = writeln!(out, "============================================");
    let _ = writeln!(out, "TRUCKPILOT VIS-UID AUDIT — Phase 5.26a Reframe-C");
    let _ = writeln!(
        out,
        "Source: production load order ({} archives, {} sectors)",
        archives.len(),
        sector_paths.len()
    );
    let _ = writeln!(out, "============================================");
    let _ = writeln!(out);
    let _ = writeln!(out, "PHASE 5.13 CONTEXT");
    let _ = writeln!(out, "------------------");
    let _ = writeln!(
        out,
        "Phase 5.13 (commit 0eddb8f8) ran on base_map.scs only (282 sectors)."
    );
    let _ = writeln!(
        out,
        "Result: 954 vis_uids total, 100.0%% unresolved, 0%% same-sector,"
    );
    let _ = writeln!(
        out,
        "0%% cross-sector. Decisive argument: parsed Node-UIDs live in"
    );
    let _ = writeln!(
        out,
        "`0x0029XXXX...`, vis_uids in `0x45-0x5DXXXX...` — disjoint UID"
    );
    let _ = writeln!(
        out,
        "namespaces. vis_uids are visibility/LOD asset hashes, not node"
    );
    let _ = writeln!(
        out,
        "references. Reframe-C re-checks at multi-archive scale."
    );
    let _ = writeln!(out);

    let _ = writeln!(out, "GLOBAL VIS-UID STATISTICS");
    let _ = writeln!(out, "-------------------------");
    let _ = writeln!(out, "Total sectors                : {}", sector_paths.len());
    let _ = writeln!(
        out,
        "Sectors with M = 0           : {} ({:.1}%)",
        sectors_zero_m,
        pct(sectors_zero_m as u64, sector_paths.len() as u64)
    );
    let _ = writeln!(
        out,
        "Sectors with M > 1000        : {} ({:.1}%)",
        sectors_high_vis,
        pct(sectors_high_vis as u64, sector_paths.len() as u64)
    );
    let _ = writeln!(out, "Total vis_uids               : {}", total_vis);
    let _ = writeln!(
        out,
        "Mean per non-empty sector    : {:.1}",
        if sectors_with_vis > 0 {
            total_vis as f64 / sectors_with_vis as f64
        } else {
            0.0
        }
    );
    let _ = writeln!(out, "Max per sector               : {}", max_m);
    let _ = writeln!(out);

    let _ = writeln!(out, "PER-ARCHIVE BREAKDOWN");
    let _ = writeln!(out, "---------------------");
    let _ = writeln!(
        out,
        "Archive                    | Sectors |  Total VisUIDs | Mean | Median |   Max"
    );
    let mut entries: Vec<&ArchiveVis> = arch_stats.values().collect();
    entries.sort_by_key(|s| std::cmp::Reverse(s.total_vis));
    for s in &entries {
        let mut counts = s.counts.clone();
        counts.sort_unstable();
        let median = if counts.is_empty() {
            0
        } else {
            counts[counts.len() / 2]
        };
        let mean = if s.sectors > 0 {
            s.sum_for_mean as f64 / s.sectors as f64
        } else {
            0.0
        };
        let _ = writeln!(
            out,
            "{:<26} | {:>7} | {:>14} | {:>4.0} | {:>6} | {:>5}",
            s.name, s.sectors, s.total_vis, mean, median, s.max
        );
    }
    let _ = writeln!(out);

    let _ = writeln!(out, "RESOLVE RATES (vs node_lookup over all archives)");
    let _ = writeln!(out, "------------------------------------------------");
    let _ = writeln!(out, "Total vis_uids               : {}", total_vis);
    let _ = writeln!(
        out,
        "  Zero (0x00)                : {} ({:.2}%)",
        zero_uids,
        pct(zero_uids, total_vis)
    );
    let _ = writeln!(
        out,
        "  Same-sector resolved       : {} ({:.2}%)",
        same,
        pct(same, total_vis)
    );
    let _ = writeln!(
        out,
        "  Cross-sector resolved      : {} ({:.2}%)",
        cross_sector,
        pct(cross_sector, total_vis)
    );
    let _ = writeln!(
        out,
        "    of which cross-archive   : {} ({:.2}%)",
        cross_archive,
        pct(cross_archive, total_vis)
    );
    let _ = writeln!(
        out,
        "  Unresolved                 : {} ({:.2}%)",
        unresolved,
        pct(unresolved, total_vis)
    );
    let _ = writeln!(out);

    let _ = writeln!(out, "BYTE-PATTERN DISCRIMINATOR (top u8 of UID)");
    let _ = writeln!(out, "------------------------------------------");
    let _ = writeln!(
        out,
        "Phase 5.13 found Node-UIDs at 0x0029... and vis_uids at 0x45-0x5D..."
    );
    let _ = writeln!(
        out,
        "If vis_uids share Node namespace, top byte should be 0x00 dominantly."
    );
    let _ = writeln!(out, "Top-byte histogram (vis_uids only, top 10):");
    let mut hist: Vec<(u8, u64)> = high_byte_hist.into_iter().collect();
    hist.sort_by_key(|x| std::cmp::Reverse(x.1));
    for (b, c) in hist.iter().take(10) {
        let _ = writeln!(
            out,
            "  0x{:02x}XXXXXXXXXXXXXX  : {:>10} ({:.2}%)",
            b,
            c,
            pct(*c, total_vis)
        );
    }
    let _ = writeln!(out);

    let _ = writeln!(out, "HYPOTHETICAL EDGE GENERATION (H4a Probe)");
    let _ = writeln!(out, "----------------------------------------");
    let _ = writeln!(
        out,
        "Naive clique: per sector connect first-node anchor -> each vis_uid"
    );
    let _ = writeln!(out, "that resolves in node_lookup.");
    let _ = writeln!(out, "  Singletons today            : {}", singletons_today);
    let _ = writeln!(
        out,
        "  Singletons that would gain edge: {} ({:.2}% of singletons)",
        singletons_gaining_edge.len(),
        pct(singletons_gaining_edge.len() as u64, singletons_today)
    );
    let _ = writeln!(
        out,
        "  Hypothetical new edges      : {}",
        hypothetical_edges
    );
    let _ = writeln!(
        out,
        "    of which cross-sector     : {}",
        hypothetical_cross_sector_edges
    );
    let _ = writeln!(out);

    let resolved_share = pct(same + cross_sector, total_vis);
    let singleton_reduction = pct(singletons_gaining_edge.len() as u64, singletons_today);
    let _ = writeln!(out, "RECOMMENDATION");
    let _ = writeln!(out, "--------------");
    if singleton_reduction > 50.0 {
        let _ = writeln!(
            out,
            "H4a GO — vis_uid edges would reduce singletons by {:.1}%, exceeds",
            singleton_reduction
        );
        let _ = writeln!(
            out,
            "the 50% GO threshold. Proceed with edge-generation prototype."
        );
    } else if singleton_reduction > 10.0 {
        let _ = writeln!(
            out,
            "H4a REVIEW — singleton reduction {:.1}% is in the [10%, 50%] band.",
            singleton_reduction
        );
        let _ = writeln!(
            out,
            "Manual call: implementation effort vs Phase 5.13 namespace caveat."
        );
    } else if resolved_share < 5.0 {
        let _ = writeln!(
            out,
            "H4a SKIP — vis_uids do NOT resolve in node_lookup at scale"
        );
        let _ = writeln!(
            out,
            "({:.2}% resolve, {:.2}% unresolved). Singleton reduction {:.2}%.",
            resolved_share,
            pct(unresolved, total_vis),
            singleton_reduction
        );
        let _ = writeln!(
            out,
            "Phase 5.13 verdict confirmed at multi-archive scale: vis_uids are"
        );
        let _ = writeln!(
            out,
            "a disjoint UID namespace (visibility/LOD asset hashes), not node"
        );
        let _ = writeln!(
            out,
            "references. Pivot to H4b (Single-Anchor-Junction-Detection) or"
        );
        let _ = writeln!(out, "H4c (Prefab-PPD).");
    } else {
        let _ = writeln!(
            out,
            "H4a SKIP — singleton reduction only {:.2}% (resolve {:.2}%).",
            singleton_reduction, resolved_share
        );
        let _ = writeln!(
            out,
            "Below 10% threshold; implementation cost not justified."
        );
    }

    if let Some(parent) = output_path.parent() {
        std::fs::create_dir_all(parent).ok();
    }
    std::fs::write(&output_path, &out).expect("write");
    let claude = PathBuf::from("outputs/claude/vis_uid_audit.txt");
    if let Some(parent) = claude.parent() {
        std::fs::create_dir_all(parent).ok();
    }
    std::fs::write(&claude, &out).expect("write claude copy");
    eprintln!("-> {} ({} bytes)", output_path.display(), out.len());
    eprintln!("-> {} (copy)", claude.display());
}
