//! `truckpilot-uid-resolution` — Phase 5.11 UID-mismatch diagnostic.
//!
//! Walks `base_map.scs` sector-by-sector, tags every node / road / prefab
//! with the sector path it came from, and answers four questions:
//!
//! 1. **Road references** — what fraction of `start_node_uid` /
//!    `end_node_uid` resolves at all, and of those, how many point at the
//!    same sector vs. a different sector.
//! 2. **Prefab references** — same classification for
//!    `connected_node_uids`.
//! 3. **Recovered-node integration** — how many nodes come from the
//!    Phase-5.8 tail-rebuild path, and whether anyone actually references
//!    them (or whether the heuristic is producing dead UIDs).
//! 4. **Cross-sector edges** — how many of the generated road / prefab
//!    edges bridge two sectors. Required for inter-region routing —
//!    if we only have intra-sector edges, every region is its own island.
//!
//! Output: a single fixed-width text table in `outputs/uid_resolution.txt`
//! (plus a copy on stdout).
//!
//! Usage:
//!
//! ```powershell
//! cargo run --release --bin truckpilot-uid-resolution -- `
//!   --ets2-dir "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2"
//! ```

use std::collections::{HashMap, HashSet};
use std::fmt::Write as _;
use std::path::PathBuf;

use truckpilot_map_parser::sector::parse_sector;
use truckpilot_map_parser::{Archive, HashFsArchive};

#[derive(Debug)]
struct Args {
    ets2_dir: PathBuf,
    output: PathBuf,
}

fn parse_args() -> Args {
    let mut ets2_dir: Option<PathBuf> = None;
    let mut output = PathBuf::from("outputs/uid_resolution.txt");

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
                eprintln!(
                    "usage: truckpilot-uid-resolution --ets2-dir <PATH> [--output <FILE>]"
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
    }
}

/// `SectorId` indexes into `sector_paths` — keeps the per-node lookup
/// table compact (u32 instead of full path strings).
type SectorId = u32;

/// One node attributed to its origin sector.
struct NodeOrigin {
    sector: SectorId,
    recovered: bool,
}

/// One road kept for the cross-sector pass.
struct RoadRef {
    sector: SectorId,
    node_a: u64,
    node_b: u64,
    lanes_forward: u8,
    lanes_backward: u8,
}

/// One prefab kept for the cross-sector pass.
struct PrefabRef {
    sector: SectorId,
    nodes: Vec<u64>,
}

/// One referenced UID, classified relative to the referencing item's sector.
#[derive(Default)]
struct RefStats {
    /// UID resolves and lives in the **same** sector as the referencer.
    same: u64,
    /// UID resolves but lives in a **different** sector.
    cross: u64,
    /// UID does not resolve in any sector at all.
    unresolved: u64,
}

impl RefStats {
    fn total(&self) -> u64 {
        self.same + self.cross + self.unresolved
    }
    fn pct_same(&self) -> f64 {
        pct(self.same, self.total())
    }
    fn pct_cross(&self) -> f64 {
        pct(self.cross, self.total())
    }
    fn pct_unresolved(&self) -> f64 {
        pct(self.unresolved, self.total())
    }
}

/// Per-road classification: each road has exactly two endpoint refs.
#[derive(Default)]
struct PerRoadStats {
    both_same: u64,
    both_cross: u64,
    one_missing: u64,
    both_missing: u64,
}

/// Per-prefab-pair classification (each clique pair contributes one bucket).
#[derive(Default)]
struct PerPairStats {
    both_same: u64,
    both_cross: u64,
    at_least_one_missing: u64,
}

/// Edges as the graph builder would emit them — counted by sector relation.
#[derive(Default)]
struct EdgeStats {
    same_sector: u64,
    cross_sector: u64,
}

#[derive(Copy, Clone)]
enum Class {
    Same,
    Cross,
    Unresolved,
}

fn pct(num: u64, denom: u64) -> f64 {
    if denom == 0 {
        0.0
    } else {
        100.0 * num as f64 / denom as f64
    }
}

fn classify_ref(
    node_to_sector: &HashMap<u64, NodeOrigin>,
    referencer_sector: SectorId,
    uid: u64,
) -> Class {
    match node_to_sector.get(&uid) {
        Some(o) if o.sector == referencer_sector => Class::Same,
        Some(_) => Class::Cross,
        None => Class::Unresolved,
    }
}

fn bump(s: &mut RefStats, c: Class) {
    match c {
        Class::Same => s.same += 1,
        Class::Cross => s.cross += 1,
        Class::Unresolved => s.unresolved += 1,
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

    // --- Pass 1: parse every sector, attribute UIDs to sector ids. -------
    //
    // Last-writer-wins on duplicate node UIDs mirrors `GraphBuilder::merge_sector`.

    let mut sectors_parsed = 0usize;
    let mut sectors_failed = 0usize;
    let mut total_normal_nodes: u64 = 0;
    let mut total_recovered_nodes: u64 = 0;

    let mut node_to_sector: HashMap<u64, NodeOrigin> = HashMap::with_capacity(1_000_000);
    let mut roads: Vec<RoadRef> = Vec::with_capacity(200_000);
    let mut prefabs: Vec<PrefabRef> = Vec::with_capacity(60_000);

    for (sid, path) in sector_paths.iter().enumerate() {
        if sid > 0 && sid % 500 == 0 {
            eprintln!("  …{}/{} sectors parsed", sid, sector_paths.len());
        }
        let Ok(data) = archive.read_path(path) else {
            sectors_failed += 1;
            continue;
        };
        let parsed = match parse_sector(&data) {
            Ok(s) => s,
            Err(_) => {
                sectors_failed += 1;
                continue;
            }
        };

        let sector_id = sid as SectorId;
        sectors_parsed += 1;
        let recovered_count = parsed.recovered_nodes_count;
        let total_count = parsed.nodes.len();
        total_recovered_nodes += recovered_count as u64;
        total_normal_nodes += total_count.saturating_sub(recovered_count) as u64;

        // The recovery path appends `recovered_nodes_count` nodes at the
        // tail of `parsed.nodes`, so the last `recovered_count` entries
        // are the recovered ones.
        let split = total_count.saturating_sub(recovered_count);
        for (idx, n) in parsed.nodes.into_iter().enumerate() {
            let recovered = idx >= split;
            node_to_sector.insert(
                n.uid,
                NodeOrigin {
                    sector: sector_id,
                    recovered,
                },
            );
        }
        for r in parsed.roads {
            roads.push(RoadRef {
                sector: sector_id,
                node_a: r.node_a,
                node_b: r.node_b,
                lanes_forward: r.lanes_forward,
                lanes_backward: r.lanes_backward,
            });
        }
        for p in parsed.prefabs {
            prefabs.push(PrefabRef {
                sector: sector_id,
                nodes: p.nodes,
            });
        }
    }

    eprintln!(
        "parsed {} sectors ({} failed), {} nodes ({} recovered), {} roads, {} prefabs",
        sectors_parsed,
        sectors_failed,
        node_to_sector.len(),
        total_recovered_nodes,
        roads.len(),
        prefabs.len()
    );

    // --- Pass 2: classify references ------------------------------------

    let mut road_refs = RefStats::default();
    let mut road_pairs = PerRoadStats::default();
    let mut road_edges = EdgeStats::default();

    for r in &roads {
        let class_a = classify_ref(&node_to_sector, r.sector, r.node_a);
        let class_b = classify_ref(&node_to_sector, r.sector, r.node_b);
        bump(&mut road_refs, class_a);
        bump(&mut road_refs, class_b);

        match (class_a, class_b) {
            (Class::Same, Class::Same) => road_pairs.both_same += 1,
            (Class::Cross, Class::Cross)
            | (Class::Same, Class::Cross)
            | (Class::Cross, Class::Same) => road_pairs.both_cross += 1,
            (Class::Unresolved, Class::Unresolved) => road_pairs.both_missing += 1,
            _ => road_pairs.one_missing += 1,
        }

        // Mirror GraphBuilder edge emission: forward + backward + bidir-fallback.
        let edges_for_this_road = if r.lanes_forward == 0 && r.lanes_backward == 0 {
            2
        } else {
            (if r.lanes_forward > 0 { 1u64 } else { 0 })
                + (if r.lanes_backward > 0 { 1u64 } else { 0 })
        };
        if edges_for_this_road > 0 {
            match (class_a, class_b) {
                (Class::Same, Class::Same) => {
                    road_edges.same_sector += edges_for_this_road;
                }
                (Class::Same, Class::Cross)
                | (Class::Cross, Class::Same)
                | (Class::Cross, Class::Cross) => {
                    road_edges.cross_sector += edges_for_this_road;
                }
                _ => { /* unresolved roads emit no edge in graph.rs */ }
            }
        }
    }

    let mut prefab_refs = RefStats::default();
    let mut prefab_pairs = PerPairStats::default();
    let mut prefab_edges = EdgeStats::default();

    for p in &prefabs {
        let mut classes: Vec<Class> = Vec::with_capacity(p.nodes.len());
        for uid in &p.nodes {
            let c = classify_ref(&node_to_sector, p.sector, *uid);
            bump(&mut prefab_refs, c);
            classes.push(c);
        }
        // Each clique pair contributes 2 directed edges (graph.rs).
        for i in 0..classes.len() {
            for j in (i + 1)..classes.len() {
                let (ca, cb) = (classes[i], classes[j]);
                match (ca, cb) {
                    (Class::Same, Class::Same) => {
                        prefab_pairs.both_same += 1;
                        prefab_edges.same_sector += 2;
                    }
                    (Class::Cross, Class::Cross)
                    | (Class::Same, Class::Cross)
                    | (Class::Cross, Class::Same) => {
                        prefab_pairs.both_cross += 1;
                        prefab_edges.cross_sector += 2;
                    }
                    _ => {
                        prefab_pairs.at_least_one_missing += 1;
                    }
                }
            }
        }
    }

    // --- Pass 3: recovered-node integration -----------------------------

    let mut referenced_uids: HashSet<u64> =
        HashSet::with_capacity(roads.len() * 2 + prefabs.len() * 4);
    for r in &roads {
        referenced_uids.insert(r.node_a);
        referenced_uids.insert(r.node_b);
    }
    for p in &prefabs {
        for u in &p.nodes {
            referenced_uids.insert(*u);
        }
    }

    let mut recovered_referenced: u64 = 0;
    let mut recovered_isolated: u64 = 0;
    let mut normal_referenced: u64 = 0;
    let mut normal_isolated: u64 = 0;
    for (uid, origin) in &node_to_sector {
        let is_ref = referenced_uids.contains(uid);
        match (origin.recovered, is_ref) {
            (true, true) => recovered_referenced += 1,
            (true, false) => recovered_isolated += 1,
            (false, true) => normal_referenced += 1,
            (false, false) => normal_isolated += 1,
        }
    }

    // --- Render ----------------------------------------------------------

    let mut out = String::with_capacity(8 * 1024);
    let _ = writeln!(out, "=== UID RESOLUTION DIAGNOSE — Phase 5.11 ===");
    let _ = writeln!(out, "archive            : {}", base_map.display());
    let _ = writeln!(
        out,
        "sectors            : {} parsed, {} failed",
        sectors_parsed, sectors_failed
    );
    let _ = writeln!(out);

    // 1. NODES
    let total_nodes = node_to_sector.len() as u64;
    let _ = writeln!(out, "── NODES ──");
    let _ = writeln!(out, "total                : {:>10}", total_nodes);
    let _ = writeln!(
        out,
        "parsed normally      : {:>10}  ({:>5.1} %)",
        total_normal_nodes,
        pct(total_normal_nodes, total_nodes)
    );
    let _ = writeln!(
        out,
        "recovered from tail  : {:>10}  ({:>5.1} %)",
        total_recovered_nodes,
        pct(total_recovered_nodes, total_nodes)
    );
    let _ = writeln!(out);

    // 2. ROAD REFS
    let _ = writeln!(out, "── ROAD REFERENCES (start/end node UIDs) ──");
    let _ = writeln!(
        out,
        "total refs           : {:>10}  (= 2 × {} roads)",
        road_refs.total(),
        roads.len()
    );
    let _ = writeln!(
        out,
        "  same sector        : {:>10}  ({:>5.1} %)",
        road_refs.same,
        road_refs.pct_same()
    );
    let _ = writeln!(
        out,
        "  cross sector       : {:>10}  ({:>5.1} %)",
        road_refs.cross,
        road_refs.pct_cross()
    );
    let _ = writeln!(
        out,
        "  unresolved         : {:>10}  ({:>5.1} %)",
        road_refs.unresolved,
        road_refs.pct_unresolved()
    );
    let _ = writeln!(out);
    let road_total: u64 = roads.len() as u64;
    let _ = writeln!(out, "per road             : ({} roads total)", road_total);
    let _ = writeln!(
        out,
        "  both same sector   : {:>10}  ({:>5.1} %)",
        road_pairs.both_same,
        pct(road_pairs.both_same, road_total)
    );
    let _ = writeln!(
        out,
        "  cross sector       : {:>10}  ({:>5.1} %)",
        road_pairs.both_cross,
        pct(road_pairs.both_cross, road_total)
    );
    let _ = writeln!(
        out,
        "  one node missing   : {:>10}  ({:>5.1} %)",
        road_pairs.one_missing,
        pct(road_pairs.one_missing, road_total)
    );
    let _ = writeln!(
        out,
        "  both nodes missing : {:>10}  ({:>5.1} %)",
        road_pairs.both_missing,
        pct(road_pairs.both_missing, road_total)
    );
    let _ = writeln!(out);

    // 3. PREFAB REFS
    let _ = writeln!(out, "── PREFAB REFERENCES (connected_node_uids) ──");
    let _ = writeln!(
        out,
        "total refs           : {:>10}  ({} prefabs)",
        prefab_refs.total(),
        prefabs.len()
    );
    let _ = writeln!(
        out,
        "  same sector        : {:>10}  ({:>5.1} %)",
        prefab_refs.same,
        prefab_refs.pct_same()
    );
    let _ = writeln!(
        out,
        "  cross sector       : {:>10}  ({:>5.1} %)",
        prefab_refs.cross,
        prefab_refs.pct_cross()
    );
    let _ = writeln!(
        out,
        "  unresolved         : {:>10}  ({:>5.1} %)",
        prefab_refs.unresolved,
        prefab_refs.pct_unresolved()
    );
    let _ = writeln!(out);
    let pair_total =
        prefab_pairs.both_same + prefab_pairs.both_cross + prefab_pairs.at_least_one_missing;
    let _ = writeln!(out, "per clique pair      : ({} pairs total)", pair_total);
    let _ = writeln!(
        out,
        "  both same sector   : {:>10}  ({:>5.1} %)",
        prefab_pairs.both_same,
        pct(prefab_pairs.both_same, pair_total)
    );
    let _ = writeln!(
        out,
        "  cross sector       : {:>10}  ({:>5.1} %)",
        prefab_pairs.both_cross,
        pct(prefab_pairs.both_cross, pair_total)
    );
    let _ = writeln!(
        out,
        "  ≥ 1 missing        : {:>10}  ({:>5.1} %)",
        prefab_pairs.at_least_one_missing,
        pct(prefab_pairs.at_least_one_missing, pair_total)
    );
    let _ = writeln!(out);

    // 4. EDGES
    let road_edge_total = road_edges.same_sector + road_edges.cross_sector;
    let prefab_edge_total = prefab_edges.same_sector + prefab_edges.cross_sector;
    let total_edges = road_edge_total + prefab_edge_total;
    let _ = writeln!(out, "── GENERATED EDGES (mirrors GraphBuilder::build) ──");
    let _ = writeln!(
        out,
        "road edges           : {:>10}  (same {:>5.1} % / cross {:>5.1} %)",
        road_edge_total,
        pct(road_edges.same_sector, road_edge_total),
        pct(road_edges.cross_sector, road_edge_total),
    );
    let _ = writeln!(
        out,
        "prefab clique edges  : {:>10}  (same {:>5.1} % / cross {:>5.1} %)",
        prefab_edge_total,
        pct(prefab_edges.same_sector, prefab_edge_total),
        pct(prefab_edges.cross_sector, prefab_edge_total),
    );
    let _ = writeln!(
        out,
        "all edges            : {:>10}  (same {:>5.1} % / cross {:>5.1} %)",
        total_edges,
        pct(
            road_edges.same_sector + prefab_edges.same_sector,
            total_edges
        ),
        pct(
            road_edges.cross_sector + prefab_edges.cross_sector,
            total_edges
        ),
    );
    let _ = writeln!(out);

    // 5. RECOVERED INTEGRATION
    let _ = writeln!(out, "── RECOVERED-NODE INTEGRATION ──");
    let _ = writeln!(
        out,
        "normal nodes         : {:>10}  (referenced {}, isolated {})",
        normal_referenced + normal_isolated,
        normal_referenced,
        normal_isolated
    );
    let _ = writeln!(
        out,
        "recovered nodes      : {:>10}  (referenced {}, isolated {})",
        recovered_referenced + recovered_isolated,
        recovered_referenced,
        recovered_isolated
    );
    let recovered_pop = recovered_referenced + recovered_isolated;
    let _ = writeln!(
        out,
        "  recovered & ref'd  : {:>5.1} %  (= are recovered UIDs ever used?)",
        pct(recovered_referenced, recovered_pop)
    );
    let _ = writeln!(
        out,
        "  recovered isolated : {:>5.1} %  (= dead UIDs from heuristic?)",
        pct(recovered_isolated, recovered_pop)
    );

    // Write file + stdout
    if let Some(parent) = args.output.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    std::fs::write(&args.output, &out)
        .unwrap_or_else(|e| panic!("write {}: {e}", args.output.display()));
    eprintln!("wrote {}", args.output.display());
    print!("{}", out);
}
