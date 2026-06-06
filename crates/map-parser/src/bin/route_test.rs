//! `truckpilot-route-test` — Phase 5.9 routing smoke test
//!
//! Loads a `MapGraph` (JSON) and a `cities.toml` file, snaps each city to its
//! nearest graph node, runs A* between every pair of cities and reports
//! per-pair distance / hop count / pass-or-fail and an overall success-rate
//! summary.
//!
//! ## city.sii integration
//!
//! When `--scs-dir` is supplied, the tool opens all `.scs` archives in that
//! directory, loads `/def/city.sii` from them (last-wins / mod-override order),
//! and uses the authoritative coordinates from city.sii instead of the manual
//! estimates in `test_cities.toml`.
//!
//! Snap radii:
//! - city.sii source : 5 km primary, 50 km fallback (no global)
//! - toml fallback   : 20 km primary, global nearest (existing behaviour)
//!
//! STOP condition: if `|city.sii_pos − toml_pos| ≥ 5000 m` the entry is
//! kept at toml coordinates and flagged `STOP` in the output.
//!
//! Usage:
//!
//! ```powershell
//! # baseline (no city.sii)
//! cargo run --release --bin truckpilot-route-test -- `
//!   --graph graph.json `
//!   --cities crates\map-parser\tests\fixtures\test_cities.toml
//!
//! # with city.sii (authoritative coordinates)
//! cargo run --release --bin truckpilot-route-test -- `
//!   --graph graph.json `
//!   --cities crates\map-parser\tests\fixtures\test_cities.toml `
//!   --scs-dir "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2"
//! ```

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};
use std::path::{Path, PathBuf};
use std::time::Instant;

use truckpilot_map_parser::archive::Archive;
use truckpilot_map_parser::city_sii::{
    build_display_name_index, load_city_sii, normalize_name, CityEntry,
};
use truckpilot_map_parser::graph::MapGraph;
use truckpilot_map_parser::{HashFsArchive, ZipArchive};

// Snap radii (metres).
const SNAP_RADIUS_SII_PRIMARY_M: f64 = 5_000.0;
const SNAP_RADIUS_SII_FALLBACK_M: f64 = 50_000.0;
const SNAP_RADIUS_TOML_M: f64 = 20_000.0; // primary for toml-only coords

// STOP threshold: delta between city.sii and toml that blocks auto-overwrite.
const STOP_DELTA_M: f64 = 5_000.0;

// ---------------------------------------------------------------------------
// Args
// ---------------------------------------------------------------------------

#[derive(Debug)]
struct Args {
    graph: PathBuf,
    cities: PathBuf,
    /// If set: directory containing ETS2 .scs archives. Enables city.sii loading.
    scs_dir: Option<PathBuf>,
}

fn parse_args() -> Args {
    let mut graph = PathBuf::from("graph.json");
    let mut cities = PathBuf::from("crates/map-parser/tests/fixtures/test_cities.toml");
    let mut scs_dir: Option<PathBuf> = None;
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--graph" => {
                graph = PathBuf::from(argv.get(i + 1).expect("--graph needs value"));
                i += 2;
            }
            "--cities" => {
                cities = PathBuf::from(argv.get(i + 1).expect("--cities needs value"));
                i += 2;
            }
            "--scs-dir" => {
                scs_dir = Some(PathBuf::from(
                    argv.get(i + 1).expect("--scs-dir needs value"),
                ));
                i += 2;
            }
            "-h" | "--help" => {
                eprintln!(
                    "usage: truckpilot-route-test [--graph PATH] [--cities PATH] [--scs-dir PATH]"
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
        graph,
        cities,
        scs_dir,
    }
}

// ---------------------------------------------------------------------------
// City from test_cities.toml
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
struct TomlCity {
    name: String,
    x: f64,
    z: f64,
}

fn read_cities(path: &PathBuf) -> Vec<TomlCity> {
    let text =
        std::fs::read_to_string(path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));

    let mut cities: Vec<TomlCity> = Vec::new();
    let mut cur_name: Option<String> = None;
    let mut cur_x: Option<f64> = None;
    let mut cur_z: Option<f64> = None;

    fn flush(
        list: &mut Vec<TomlCity>,
        name: &mut Option<String>,
        x: &mut Option<f64>,
        z: &mut Option<f64>,
    ) {
        if let (Some(n), Some(xv), Some(zv)) = (name.take(), x.take(), z.take()) {
            list.push(TomlCity {
                name: n,
                x: xv,
                z: zv,
            });
        }
    }

    for raw in text.lines() {
        let line = raw.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        if line == "[[city]]" {
            flush(&mut cities, &mut cur_name, &mut cur_x, &mut cur_z);
            continue;
        }
        if let Some((key, val)) = line.split_once('=') {
            let key = key.trim();
            let val = val.trim();
            match key {
                "name" => cur_name = Some(val.trim_matches('"').to_string()),
                "x" => cur_x = val.parse::<f64>().ok(),
                "z" => cur_z = val.parse::<f64>().ok(),
                _ => {}
            }
        }
    }
    flush(&mut cities, &mut cur_name, &mut cur_x, &mut cur_z);
    cities
}

// ---------------------------------------------------------------------------
// City.sii archive loading
// ---------------------------------------------------------------------------

/// Open all `.scs` archives found in `scs_dir` (HashFS or ZIP).
fn open_archives_from_dir(scs_dir: &Path) -> Vec<Box<dyn Archive>> {
    let entries = match std::fs::read_dir(scs_dir) {
        Ok(e) => e,
        Err(e) => {
            eprintln!("WARN: cannot read scs-dir {:?}: {e}", scs_dir);
            return Vec::new();
        }
    };

    let mut archives: Vec<Box<dyn Archive>> = Vec::new();
    let mut scs_paths: Vec<PathBuf> = entries
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().and_then(|s| s.to_str()) == Some("scs"))
        .collect();
    scs_paths.sort(); // deterministic load order

    for path in &scs_paths {
        let arc: Box<dyn Archive> = match HashFsArchive::open(path) {
            Ok(a) => Box::new(a),
            Err(_) => match ZipArchive::open(path) {
                Ok(a) => Box::new(a),
                Err(e) => {
                    eprintln!(
                        "WARN: skipping {}: {e}",
                        path.file_name().unwrap_or_default().to_string_lossy()
                    );
                    continue;
                }
            },
        };
        archives.push(arc);
    }

    eprintln!(
        "city.sii: opened {} archive(s) from {:?}",
        archives.len(),
        scs_dir
    );
    archives
}

// ---------------------------------------------------------------------------
// City resolution — merges toml + city.sii
// ---------------------------------------------------------------------------

/// Source of the authoritative coordinates used for snapping.
#[derive(Debug, Clone, Copy, PartialEq)]
enum CoordSource {
    /// Coordinates come from city.sii (authoritative).
    CitySii,
    /// city.sii coords exceed STOP_DELTA_M from toml — kept at toml coords.
    SiiStop,
    /// city not found in city.sii — using toml coords as fallback.
    TomlFallback,
    /// No city.sii was loaded at all — toml coords only.
    TomlOnly,
}

#[derive(Debug, Clone)]
struct ResolvedCity {
    name: String,
    /// Coordinates actually used for snapping (authoritative).
    x: f64,
    z: f64,
    source: CoordSource,
    /// Original toml coordinates (kept for A/B comparison output).
    toml_x: f64,
    toml_z: f64,
    /// city.sii coordinates when found (for delta reporting).
    sii_x: Option<f64>,
    sii_z: Option<f64>,
}

/// Find the best city.sii entry for a given toml city name.
///
/// Lookup order:
/// 1. Exact unit_name match (lowercase of toml name, e.g. "berlin" → "berlin")
/// 2. Normalized display-name match via `display_idx`
fn find_city_entry<'a>(
    toml_name: &str,
    sii_map: &'a HashMap<String, CityEntry>,
    display_idx: &HashMap<String, String>,
) -> Option<&'a CityEntry> {
    // 1. Exact unit_name
    let key = toml_name.to_lowercase();
    if let Some(e) = sii_map.get(&key) {
        return Some(e);
    }
    // 2. Normalized display name
    let norm = normalize_name(toml_name);
    if let Some(unit) = display_idx.get(&norm) {
        return sii_map.get(unit.as_str());
    }
    None
}

fn resolve_cities(
    toml_cities: &[TomlCity],
    sii_map: Option<&HashMap<String, CityEntry>>,
) -> Vec<ResolvedCity> {
    // Build display-name index once (O(N)), not once per city (O(N²)).
    let display_idx_owned: Option<HashMap<String, String>> = sii_map.map(build_display_name_index);
    let display_idx_ref = display_idx_owned.as_ref();

    toml_cities
        .iter()
        .map(|city| {
            let (toml_x, toml_z) = (city.x, city.z);

            let (Some(sii_map), Some(display_idx)) = (sii_map, display_idx_ref) else {
                return ResolvedCity {
                    name: city.name.clone(),
                    x: toml_x,
                    z: toml_z,
                    source: CoordSource::TomlOnly,
                    toml_x,
                    toml_z,
                    sii_x: None,
                    sii_z: None,
                };
            };

            match find_city_entry(&city.name, sii_map, display_idx) {
                None => ResolvedCity {
                    name: city.name.clone(),
                    x: toml_x,
                    z: toml_z,
                    source: CoordSource::TomlFallback,
                    toml_x,
                    toml_z,
                    sii_x: None,
                    sii_z: None,
                },
                Some(entry) => {
                    let (sx, sz) = (entry.x, entry.z);
                    let dx = sx - toml_x;
                    let dz = sz - toml_z;
                    let delta_m = (dx * dx + dz * dz).sqrt();

                    if delta_m >= STOP_DELTA_M {
                        // STOP condition: delta too large — keep toml coords.
                        ResolvedCity {
                            name: city.name.clone(),
                            x: toml_x,
                            z: toml_z,
                            source: CoordSource::SiiStop,
                            toml_x,
                            toml_z,
                            sii_x: Some(sx),
                            sii_z: Some(sz),
                        }
                    } else {
                        // Auto-overwrite with city.sii coords.
                        ResolvedCity {
                            name: city.name.clone(),
                            x: sx,
                            z: sz,
                            source: CoordSource::CitySii,
                            toml_x,
                            toml_z,
                            sii_x: Some(sx),
                            sii_z: Some(sz),
                        }
                    }
                }
            }
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Snapping
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
struct SnapResult {
    uid: Option<u64>,
    dist_m: f64,
    tier: &'static str,
}

fn snap_city(city: &ResolvedCity, graph: &MapGraph) -> SnapResult {
    let (primary_r, fallback_r) = match city.source {
        CoordSource::CitySii => (SNAP_RADIUS_SII_PRIMARY_M, Some(SNAP_RADIUS_SII_FALLBACK_M)),
        _ => (SNAP_RADIUS_TOML_M, None), // global fallback for toml
    };

    let mut best_uid: Option<u64> = None;
    let mut best_d2 = f64::INFINITY;
    for n in &graph.nodes {
        let dx = n.x - city.x;
        let dz = n.z - city.z;
        let d2 = dx * dx + dz * dz;
        if d2 < best_d2 {
            best_d2 = d2;
            best_uid = Some(n.uid);
        }
    }
    let dist = best_d2.sqrt();

    if dist <= primary_r {
        SnapResult {
            uid: best_uid,
            dist_m: dist,
            tier: "primary",
        }
    } else if let Some(fb) = fallback_r {
        if dist <= fb {
            SnapResult {
                uid: best_uid,
                dist_m: dist,
                tier: "fallback",
            }
        } else {
            SnapResult {
                uid: None,
                dist_m: dist,
                tier: "miss",
            }
        }
    } else {
        // Global fallback (toml-only behaviour).
        SnapResult {
            uid: best_uid,
            dist_m: dist,
            tier: "global",
        }
    }
}

/// Snap using toml coords + 20km primary + global fallback (for A/B pre column).
fn snap_toml(toml_x: f64, toml_z: f64, graph: &MapGraph) -> SnapResult {
    let mut best_uid: Option<u64> = None;
    let mut best_d2 = f64::INFINITY;
    for n in &graph.nodes {
        let dx = n.x - toml_x;
        let dz = n.z - toml_z;
        let d2 = dx * dx + dz * dz;
        if d2 < best_d2 {
            best_d2 = d2;
            best_uid = Some(n.uid);
        }
    }
    let dist = best_d2.sqrt();
    let tier = if dist <= SNAP_RADIUS_TOML_M {
        "20km"
    } else {
        "global"
    };
    SnapResult {
        uid: best_uid,
        dist_m: dist,
        tier,
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() {
    let args = parse_args();

    eprintln!("loading {} …", args.graph.display());
    let bytes =
        std::fs::read(&args.graph).unwrap_or_else(|e| panic!("read {}: {e}", args.graph.display()));
    let graph: MapGraph = serde_json::from_slice(&bytes)
        .unwrap_or_else(|e| panic!("parse {}: {e}", args.graph.display()));
    eprintln!(
        "graph: {} nodes / {} edges",
        graph.nodes.len(),
        graph.edges.len()
    );

    let toml_cities = read_cities(&args.cities);
    eprintln!(
        "loaded {} cities from {}",
        toml_cities.len(),
        args.cities.display()
    );
    if toml_cities.is_empty() {
        eprintln!("ERROR: no cities parsed");
        std::process::exit(1);
    }

    // Load city.sii if --scs-dir was provided.
    let sii_map_owned: Option<HashMap<String, CityEntry>> = args.scs_dir.as_ref().map(|dir| {
        let mut archives = open_archives_from_dir(dir);
        if archives.is_empty() {
            eprintln!(
                "WARN: no archives opened from {:?} — falling back to toml coords",
                dir
            );
            return HashMap::new();
        }
        load_city_sii(&mut archives)
    });
    let sii_map = sii_map_owned.as_ref();

    if let Some(m) = sii_map {
        eprintln!("city.sii: {} entries loaded", m.len());
    }

    // Resolve authoritative coordinates for each city.
    let resolved = resolve_cities(&toml_cities, sii_map);

    // Compute pre-snap (toml) AND post-snap (resolved) for each city.
    // Pre is shown in the A/B comparison; post drives routing.
    let snapped: Vec<(ResolvedCity, SnapResult, SnapResult)> = resolved
        .into_iter()
        .map(|city| {
            let pre = snap_toml(city.toml_x, city.toml_z, &graph);
            let post = snap_city(&city, &graph);
            (city, pre, post)
        })
        .collect();

    // Print city source section (A/B comparison including snap UIDs).
    print_city_source(&snapped, sii_map.is_some());

    // Build shared routing structures once (not per A* call).
    let adj = build_adjacency(&graph);
    let positions: HashMap<u64, (f64, f64)> =
        graph.nodes.iter().map(|n| (n.uid, (n.x, n.z))).collect();
    let node_count = graph.nodes.len();
    let mut pair_results: Vec<PairResult> = Vec::new();

    for (i, (from_city, _, from_post)) in snapped.iter().enumerate() {
        for (j, (to_city, _, to_post)) in snapped.iter().enumerate() {
            if i == j {
                continue;
            }
            let res = match (from_post.uid, to_post.uid) {
                (Some(a), Some(b)) => {
                    let t0 = Instant::now();
                    let path = a_star(&positions, &adj, node_count, a, b);
                    let elapsed_ms = t0.elapsed().as_secs_f64() * 1000.0;
                    PairResult {
                        from: from_city.name.clone(),
                        to: to_city.name.clone(),
                        path,
                        elapsed_ms,
                    }
                }
                _ => PairResult {
                    from: from_city.name.clone(),
                    to: to_city.name.clone(),
                    path: None,
                    elapsed_ms: 0.0,
                },
            };
            pair_results.push(res);
        }
    }

    print_pair_results(&pair_results);
}

// ---------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------

fn print_city_source(snapped: &[(ResolvedCity, SnapResult, SnapResult)], sii_loaded: bool) {
    if !sii_loaded {
        println!("=== CITY COORDINATES (toml-only, no --scs-dir) ===");
        println!(
            "  {:<12}  {:>22}  {:>18}  {:>6}",
            "name", "toml (x, z)", "snap uid", "dist_m"
        );
        println!("  {}", "-".repeat(70));
        for (c, _pre, post) in snapped {
            let uid_str = post
                .uid
                .map(|u| format!("0x{u:016X}[{}]", post.tier))
                .unwrap_or_else(|| "MISS".to_string());
            let dist_str = if post.uid.is_some() {
                format!("{:>6.0}", post.dist_m)
            } else {
                " MISS".to_string()
            };
            println!(
                "  {:<12}  ({:>9.0},{:>9.0})  {}  {}",
                c.name, c.toml_x, c.toml_z, uid_str, dist_str
            );
        }
        println!();
        return;
    }

    println!("=== CITY SOURCE (city.sii A/B) ===");
    println!(
        "  {:<12}  {:<8}  {:>22}  {:>22}  {:>8}  {:>18}  {:>18}",
        "name", "source", "toml (x, z)", "sii  (x, z)", "delta_m", "pre-snap uid", "post-snap uid",
    );
    println!("  {}", "-".repeat(130));

    for (c, pre, post) in snapped {
        let source_str = match c.source {
            CoordSource::CitySii => "city.sii",
            CoordSource::SiiStop => "STOP!   ",
            CoordSource::TomlFallback => "toml-fb ",
            CoordSource::TomlOnly => "toml    ",
        };
        let toml_str = format!("({:>9.0},{:>9.0})", c.toml_x, c.toml_z);
        let sii_str = match (c.sii_x, c.sii_z) {
            (Some(sx), Some(sz)) => format!("({:>9.0},{:>9.0})", sx, sz),
            _ => format!("{:>22}", "n/a"),
        };
        let delta_str = match (c.sii_x, c.sii_z) {
            (Some(sx), Some(sz)) => {
                let d = ((sx - c.toml_x).powi(2) + (sz - c.toml_z).powi(2)).sqrt();
                format!("{:>7.0} m", d)
            }
            _ => format!("{:>8}", "n/a"),
        };
        let pre_uid = pre
            .uid
            .map(|u| format!("0x{u:016X}[{}]", pre.tier))
            .unwrap_or_else(|| "MISS".to_string());
        let post_uid = post
            .uid
            .map(|u| format!("0x{u:016X}[{}]", post.tier))
            .unwrap_or_else(|| "MISS".to_string());
        println!(
            "  {:<12}  {}  {}  {}  {}  {}  {}",
            c.name, source_str, toml_str, sii_str, delta_str, pre_uid, post_uid
        );
        if c.source == CoordSource::SiiStop {
            println!(
                "  !! STOP: delta ≥ {:.0} m — kept toml coords; verify manually before overwriting.",
                STOP_DELTA_M
            );
        }
    }
    println!();

    let n_sii = snapped
        .iter()
        .filter(|(c, _, _)| c.source == CoordSource::CitySii)
        .count();
    let n_stop = snapped
        .iter()
        .filter(|(c, _, _)| c.source == CoordSource::SiiStop)
        .count();
    let n_fb = snapped
        .iter()
        .filter(|(c, _, _)| c.source == CoordSource::TomlFallback)
        .count();
    println!(
        "  city.sii: {n_sii} matched | {n_fb} not-found (toml fallback) | {n_stop} STOP (delta ≥ {:.0} m)",
        STOP_DELTA_M
    );
    println!();
}

// ---------------------------------------------------------------------------
// Routing
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
struct PairResult {
    from: String,
    to: String,
    path: Option<PathInfo>,
    elapsed_ms: f64,
}

#[derive(Debug, Clone)]
struct PathInfo {
    distance_m: f64,
    hops: usize,
}

fn build_adjacency(graph: &MapGraph) -> HashMap<u64, Vec<(u64, f64)>> {
    let mut adj: HashMap<u64, Vec<(u64, f64)>> = HashMap::with_capacity(graph.edges.len());
    for e in &graph.edges {
        adj.entry(e.from).or_default().push((e.to, e.distance_m));
        adj.entry(e.to).or_default().push((e.from, e.distance_m));
    }
    adj
}

fn a_star(
    positions: &HashMap<u64, (f64, f64)>,
    adj: &HashMap<u64, Vec<(u64, f64)>>,
    node_count: usize,
    start: u64,
    goal: u64,
) -> Option<PathInfo> {
    if start == goal {
        return Some(PathInfo {
            distance_m: 0.0,
            hops: 0,
        });
    }

    let &(gx, gz) = positions.get(&goal)?;

    let mut g_score: HashMap<u64, f64> = HashMap::new();
    let mut came_from: HashMap<u64, u64> = HashMap::new();
    let mut open: BinaryHeap<Reverse<(u64, u64)>> = BinaryHeap::new();
    g_score.insert(start, 0.0);
    open.push(Reverse((0, start)));

    let cap = (node_count / 4).max(1024);
    let mut steps = 0usize;
    while let Some(Reverse((_, cur))) = open.pop() {
        steps += 1;
        if steps > cap {
            return None;
        }
        if cur == goal {
            let mut hops = 0usize;
            let mut node = goal;
            while let Some(&prev) = came_from.get(&node) {
                hops += 1;
                node = prev;
                if node == start {
                    break;
                }
            }
            let total = g_score.get(&goal).copied().unwrap_or(f64::INFINITY);
            return Some(PathInfo {
                distance_m: total,
                hops,
            });
        }

        let cur_g = g_score.get(&cur).copied().unwrap_or(f64::INFINITY);
        let Some(neighbours) = adj.get(&cur) else {
            continue;
        };
        for &(next, edge_w) in neighbours {
            let tentative = cur_g + edge_w;
            if tentative < g_score.get(&next).copied().unwrap_or(f64::INFINITY) {
                g_score.insert(next, tentative);
                came_from.insert(next, cur);
                let h = if let Some(&(nx, nz)) = positions.get(&next) {
                    let dx = nx - gx;
                    let dz = nz - gz;
                    (dx * dx + dz * dz).sqrt()
                } else {
                    0.0
                };
                let f = (tentative + h) as u64;
                open.push(Reverse((f, next)));
            }
        }
    }
    None
}

fn print_pair_results(results: &[PairResult]) {
    let total = results.len();
    let success = results.iter().filter(|r| r.path.is_some()).count();
    let success_rate = if total == 0 {
        0.0
    } else {
        100.0 * success as f64 / total as f64
    };

    println!("=== ROUTING RESULTS ({success}/{total} pairs, {success_rate:.1}%) ===");
    println!();
    for r in results {
        match &r.path {
            Some(p) => println!(
                "  {:<12} → {:<12}  {:>9.1} km  {:>5} hops  {:>6.1} ms",
                r.from,
                r.to,
                p.distance_m / 1000.0,
                p.hops,
                r.elapsed_ms
            ),
            None => println!(
                "  {:<12} → {:<12}  NO PATH                              {:>6.1} ms",
                r.from, r.to, r.elapsed_ms
            ),
        }
    }
    println!();

    let succ_paths: Vec<&PathInfo> = results.iter().filter_map(|r| r.path.as_ref()).collect();
    if !succ_paths.is_empty() {
        let avg_dist =
            succ_paths.iter().map(|p| p.distance_m).sum::<f64>() / succ_paths.len() as f64;
        let avg_hops =
            succ_paths.iter().map(|p| p.hops as f64).sum::<f64>() / succ_paths.len() as f64;
        let max_dist = succ_paths
            .iter()
            .map(|p| p.distance_m)
            .fold(0.0_f64, f64::max);
        let min_dist = succ_paths
            .iter()
            .map(|p| p.distance_m)
            .fold(f64::INFINITY, f64::min);

        println!("=== SUMMARY ===");
        println!("Success rate : {success_rate:.1}%  ({success}/{total})");
        println!("Avg distance : {:.1} km", avg_dist / 1000.0);
        println!("Avg hops     : {avg_hops:.1}");
        println!(
            "Min / Max    : {:.1} km / {:.1} km",
            min_dist / 1000.0,
            max_dist / 1000.0
        );

        let mut sorted: Vec<&PairResult> = results.iter().filter(|r| r.path.is_some()).collect();
        sorted.sort_by(|a, b| {
            b.path
                .as_ref()
                .unwrap()
                .distance_m
                .partial_cmp(&a.path.as_ref().unwrap().distance_m)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        println!();
        println!("Top 5 longest successful routes:");
        for r in sorted.iter().take(5) {
            let p = r.path.as_ref().unwrap();
            println!(
                "  {:<12} → {:<12}  {:>9.1} km  {:>5} hops",
                r.from,
                r.to,
                p.distance_m / 1000.0,
                p.hops
            );
        }
    }

    let failures: Vec<&PairResult> = results.iter().filter(|r| r.path.is_none()).collect();
    if !failures.is_empty() {
        println!();
        println!("Failures ({}/{}):", failures.len(), total);
        for r in failures.iter().take(10) {
            println!("  {:<12} → {:<12}", r.from, r.to);
        }
        if failures.len() > 10 {
            println!("  … and {} more", failures.len() - 10);
        }
    }
}
