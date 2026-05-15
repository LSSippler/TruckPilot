//! `truckpilot-router-perf-audit` — Phase 6.2b pre-audit
//!
//! Measures A* routing latency on the Big-8 cluster of cities (28 pairs).
//! Reports P50/P95/P99 percentiles broken down by:
//!   - nearest_node lookup
//!   - A* search (faithfully mirrors `RouterPlugin::plan()`)
//!   - waypoint materialization
//!
//! Output: `outputs/router_perf_audit.txt`
//!
//! Usage:
//!   cargo run --release -p truckpilot-diag --bin truckpilot-router-perf-audit -- \
//!     --graph graph.json \
//!     --cities crates/map-parser/tests/fixtures/test_cities.toml
//!
//! See CLAUDE.md / docs/vault for context; this binary makes no production
//! code changes.

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::path::PathBuf;
use std::time::Instant;

use truckpilot_map_parser::graph::MapGraph;

const BIG8: [&str; 8] = [
    "Muenchen", "Prag", "Warschau", "Amsterdam",
    "Mailand", "Sevilla", "Sofia", "Istanbul",
];
const RUNS_PER_PAIR: usize = 3;

#[derive(Debug)]
struct Args {
    graph: PathBuf,
    cities: PathBuf,
    output: PathBuf,
}

fn parse_args() -> Args {
    let mut graph = PathBuf::from("graph.json");
    let mut cities = PathBuf::from("crates/map-parser/tests/fixtures/test_cities.toml");
    let mut output = PathBuf::from("outputs/router_perf_audit.txt");
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--graph" => { graph = PathBuf::from(&argv[i + 1]); i += 2; }
            "--cities" => { cities = PathBuf::from(&argv[i + 1]); i += 2; }
            "--output" => { output = PathBuf::from(&argv[i + 1]); i += 2; }
            "-h" | "--help" => {
                eprintln!("usage: truckpilot-router-perf-audit [--graph PATH] [--cities PATH] [--output PATH]");
                std::process::exit(0);
            }
            other => { eprintln!("unknown argument: {other}"); std::process::exit(2); }
        }
    }
    Args { graph, cities, output }
}

#[derive(Debug, Clone)]
struct City { name: String, x: f64, z: f64 }

fn read_cities(path: &PathBuf) -> Vec<City> {
    let text = std::fs::read_to_string(path)
        .unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    let mut cities: Vec<City> = Vec::new();
    let (mut cur_name, mut cur_x, mut cur_z): (Option<String>, Option<f64>, Option<f64>) = (None, None, None);
    let flush = |list: &mut Vec<City>, name: &mut Option<String>, x: &mut Option<f64>, z: &mut Option<f64>| {
        if let (Some(n), Some(xv), Some(zv)) = (name.take(), x.take(), z.take()) {
            list.push(City { name: n, x: xv, z: zv });
        }
    };
    for raw in text.lines() {
        let line = raw.split('#').next().unwrap_or("").trim();
        if line.is_empty() { continue; }
        if line == "[[city]]" {
            flush(&mut cities, &mut cur_name, &mut cur_x, &mut cur_z);
            continue;
        }
        if let Some((key, val)) = line.split_once('=') {
            let (key, val) = (key.trim(), val.trim());
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
// A* — mirrors `crates/plugins/router/src/lib.rs` plan() byte-for-byte
// (positions + adjacency rebuilt per call; Euclidean heuristic; f64).
// Returns (path, nodes_expanded) for stats.
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct HeapEntry { uid: u64, f: f64 }
impl PartialEq for HeapEntry { fn eq(&self, o: &Self) -> bool { self.f.total_cmp(&o.f).is_eq() && self.uid == o.uid } }
impl Eq for HeapEntry {}
impl PartialOrd for HeapEntry { fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> { Some(self.cmp(o)) } }
impl Ord for HeapEntry { fn cmp(&self, o: &Self) -> std::cmp::Ordering { self.f.total_cmp(&o.f).then_with(|| self.uid.cmp(&o.uid)) } }

fn heuristic(p: (f64, f64), g: (f64, f64)) -> f64 {
    let dx = p.0 - g.0; let dz = p.1 - g.1; (dx * dx + dz * dz).sqrt()
}

fn plan(
    nodes: &[(u64, f64, f64)],
    edges: &[(u64, u64, f64)],
    start: u64,
    goal: u64,
) -> (Option<Vec<u64>>, usize) {
    let positions: HashMap<u64, (f64, f64)> =
        nodes.iter().map(|&(uid, x, z)| (uid, (x, z))).collect();
    let mut adj: HashMap<u64, Vec<(u64, f64)>> = HashMap::new();
    for &(from, to, dist) in edges {
        adj.entry(from).or_default().push((to, dist));
    }
    let Some(&goal_pos) = positions.get(&goal) else { return (None, 0); };
    let Some(&start_pos) = positions.get(&start) else { return (None, 0); };
    let mut open: BinaryHeap<Reverse<HeapEntry>> = BinaryHeap::new();
    let mut g: HashMap<u64, f64> = HashMap::new();
    let mut came_from: HashMap<u64, u64> = HashMap::new();
    let mut closed: HashSet<u64> = HashSet::new();
    let mut expanded = 0usize;
    g.insert(start, 0.0);
    open.push(Reverse(HeapEntry { uid: start, f: heuristic(start_pos, goal_pos) }));
    while let Some(Reverse(entry)) = open.pop() {
        if entry.uid == goal {
            return (Some(reconstruct(&came_from, start, goal)), expanded);
        }
        if !closed.insert(entry.uid) { continue; }
        expanded += 1;
        for &(nb, cost) in adj.get(&entry.uid).into_iter().flatten() {
            if closed.contains(&nb) { continue; }
            let tg = g[&entry.uid] + cost;
            if tg < *g.get(&nb).unwrap_or(&f64::MAX) {
                came_from.insert(nb, entry.uid);
                g.insert(nb, tg);
                let Some(&np) = positions.get(&nb) else { continue; };
                let h = heuristic(np, goal_pos);
                open.push(Reverse(HeapEntry { uid: nb, f: tg + h }));
            }
        }
    }
    (None, expanded)
}

fn reconstruct(came_from: &HashMap<u64, u64>, start: u64, goal: u64) -> Vec<u64> {
    let mut path = vec![goal];
    let mut cur = goal;
    while cur != start {
        if let Some(&prev) = came_from.get(&cur) { path.push(prev); cur = prev; } else { break; }
    }
    path.reverse();
    path
}

fn find_nearest(nodes: &[(u64, f64, f64)], x: f64, z: f64) -> Option<u64> {
    nodes.iter()
        .map(|&(uid, nx, nz)| { let dx = nx - x; let dz = nz - z; (uid, dx * dx + dz * dz) })
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(uid, _)| uid)
}

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Default)]
struct PairTimings {
    nearest_ms: f64,
    astar_ms: f64,
    waypoint_ms: f64,
    total_ms: f64,
    nodes_expanded: usize,
    path_len: usize,
    reachable: bool,
}

fn percentile(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() { return 0.0; }
    let idx = ((sorted.len() as f64 - 1.0) * p).round() as usize;
    sorted[idx.min(sorted.len() - 1)]
}

fn summarize(values: &[f64]) -> (f64, f64, f64, f64) {
    let mut v = values.to_vec();
    v.sort_by(|a, b| a.total_cmp(b));
    (percentile(&v, 0.50), percentile(&v, 0.95), percentile(&v, 0.99),
     v.last().copied().unwrap_or(0.0))
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() {
    let args = parse_args();

    eprintln!("loading {} …", args.graph.display());
    let bytes = match std::fs::read(&args.graph) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("ERROR: cannot read {}: {e}", args.graph.display());
            eprintln!("(run the map-build first to generate graph.json)");
            std::process::exit(2);
        }
    };
    let graph: MapGraph = match serde_json::from_slice(&bytes) {
        Ok(g) => g,
        Err(e) => {
            eprintln!("ERROR: cannot parse {}: {e}", args.graph.display());
            std::process::exit(2);
        }
    };
    eprintln!("graph: {} nodes / {} edges", graph.nodes.len(), graph.edges.len());

    // Flatten to plugin-router shape.
    let nodes: Vec<(u64, f64, f64)> = graph.nodes.iter().map(|n| (n.uid, n.x, n.z)).collect();
    let edges: Vec<(u64, u64, f64)> = graph.edges.iter().map(|e| (e.from, e.to, e.distance_m)).collect();
    let positions: HashMap<u64, (f64, f64)> = nodes.iter().map(|&(u, x, z)| (u, (x, z))).collect();

    let all_cities = read_cities(&args.cities);
    let big8: Vec<City> = BIG8.iter()
        .filter_map(|name| all_cities.iter().find(|c| c.name == *name).cloned())
        .collect();
    if big8.len() != BIG8.len() {
        eprintln!("ERROR: expected {} Big-8 cities, found {} in {}",
            BIG8.len(), big8.len(), args.cities.display());
        std::process::exit(2);
    }
    eprintln!("Big-8 cities loaded: {}", big8.iter().map(|c| c.name.as_str()).collect::<Vec<_>>().join(", "));

    // 28 unordered pairs.
    let mut pairs: Vec<(City, City)> = Vec::new();
    for i in 0..big8.len() {
        for j in (i + 1)..big8.len() {
            pairs.push((big8[i].clone(), big8[j].clone()));
        }
    }
    eprintln!("pair count: {}", pairs.len());

    let mut results: Vec<(String, String, PairTimings)> = Vec::with_capacity(pairs.len());

    for (from, to) in &pairs {
        let mut acc = PairTimings::default();
        for run in 0..RUNS_PER_PAIR {
            let t_total = Instant::now();

            let t0 = Instant::now();
            let from_uid = find_nearest(&nodes, from.x, from.z);
            let to_uid = find_nearest(&nodes, to.x, to.z);
            let nearest_ms = t0.elapsed().as_secs_f64() * 1000.0;

            let (path, expanded, astar_ms) = match (from_uid, to_uid) {
                (Some(a), Some(b)) => {
                    let t1 = Instant::now();
                    let (p, e) = plan(&nodes, &edges, a, b);
                    let ms = t1.elapsed().as_secs_f64() * 1000.0;
                    (p, e, ms)
                }
                _ => (None, 0, 0.0),
            };

            let t2 = Instant::now();
            let path_len = path.as_ref().map(|p| {
                p.iter().filter_map(|uid| positions.get(uid)).count()
            }).unwrap_or(0);
            let waypoint_ms = t2.elapsed().as_secs_f64() * 1000.0;

            let total_ms = t_total.elapsed().as_secs_f64() * 1000.0;

            // Average across runs (last run wins for path metadata; timings averaged).
            acc.nearest_ms += nearest_ms;
            acc.astar_ms += astar_ms;
            acc.waypoint_ms += waypoint_ms;
            acc.total_ms += total_ms;
            if run == RUNS_PER_PAIR - 1 {
                acc.nearest_ms /= RUNS_PER_PAIR as f64;
                acc.astar_ms /= RUNS_PER_PAIR as f64;
                acc.waypoint_ms /= RUNS_PER_PAIR as f64;
                acc.total_ms /= RUNS_PER_PAIR as f64;
                acc.nodes_expanded = expanded;
                acc.path_len = path_len;
                acc.reachable = path.is_some();
            }
        }
        eprintln!("  {:<10} -> {:<10}  total {:>7.1} ms  astar {:>7.1} ms  expanded {:>7}  reachable={}",
            from.name, to.name, acc.total_ms, acc.astar_ms, acc.nodes_expanded, acc.reachable);
        results.push((from.name.clone(), to.name.clone(), acc));
    }

    // ---- Build output ----------------------------------------------------
    use std::fmt::Write as _;
    let mut out = String::new();

    let ts = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    writeln!(out, "============================================").unwrap();
    writeln!(out, "A* ROUTER PERFORMANCE AUDIT").unwrap();
    writeln!(out, "Generated: unix_ts={ts}").unwrap();
    writeln!(out, "Graph: {} nodes, {} edges", graph.nodes.len(), graph.edges.len()).unwrap();
    writeln!(out, "Runs per pair: {RUNS_PER_PAIR} (warm-cache avg)").unwrap();
    writeln!(out, "============================================").unwrap();
    writeln!(out).unwrap();

    writeln!(out, "PER-PAIR LATENCY").unwrap();
    writeln!(out, "----------------").unwrap();
    writeln!(out, "{:<30} | {:>10} | {:>9} | {:>10} | {:>14} | status",
        "Pair", "nearest_ms", "astar_ms", "waypt_ms", "nodes_expanded").unwrap();
    for (a, b, r) in &results {
        let status = if r.reachable { format!("OK ({} wps)", r.path_len) } else { "NO PATH".to_string() };
        writeln!(out, "{:<30} | {:>10.2} | {:>9.2} | {:>10.2} | {:>14} | {}",
            format!("{} -> {}", a, b),
            r.nearest_ms, r.astar_ms, r.waypoint_ms, r.nodes_expanded, status).unwrap();
    }
    writeln!(out).unwrap();

    let totals: Vec<f64> = results.iter().map(|(_, _, r)| r.total_ms).collect();
    let nearests: Vec<f64> = results.iter().map(|(_, _, r)| r.nearest_ms).collect();
    let astars: Vec<f64> = results.iter().map(|(_, _, r)| r.astar_ms).collect();
    let wpts: Vec<f64> = results.iter().map(|(_, _, r)| r.waypoint_ms).collect();
    let (p50, p95, p99, max) = summarize(&totals);
    let (n_p50, n_p95, _, _) = summarize(&nearests);
    let (a_p50, a_p95, _, _) = summarize(&astars);
    let (w_p50, w_p95, _, _) = summarize(&wpts);

    writeln!(out, "PERCENTILES ({} pairs, total ms)", results.len()).unwrap();
    writeln!(out, "----------------------").unwrap();
    writeln!(out, "P50:  {p50:>7.2} ms").unwrap();
    writeln!(out, "P95:  {p95:>7.2} ms").unwrap();
    writeln!(out, "P99:  {p99:>7.2} ms").unwrap();
    writeln!(out, "Max:  {max:>7.2} ms").unwrap();
    writeln!(out).unwrap();

    writeln!(out, "BREAKDOWN").unwrap();
    writeln!(out, "---------").unwrap();
    writeln!(out, "nearest_node:  P50 {n_p50:>7.2} ms, P95 {n_p95:>7.2} ms").unwrap();
    writeln!(out, "A* search:     P50 {a_p50:>7.2} ms, P95 {a_p95:>7.2} ms").unwrap();
    writeln!(out, "waypoint mat:  P50 {w_p50:>7.2} ms, P95 {w_p95:>7.2} ms").unwrap();
    writeln!(out).unwrap();

    // ---- Verdict ---------------------------------------------------------
    let budget_ms = 200.0;
    let verdict = if p95 <= budget_ms * 0.5 { "PASS" }
                  else if p95 <= budget_ms   { "WARN" }
                  else                       { "FAIL" };
    let bottleneck = {
        let candidates = [("nearest_node", n_p95), ("A* search", a_p95), ("waypoint mat", w_p95)];
        candidates.iter().max_by(|x, y| x.1.total_cmp(&y.1)).map(|(n, _)| *n).unwrap_or("?")
    };
    writeln!(out, "VERDICT").unwrap();
    writeln!(out, "-------").unwrap();
    writeln!(out, "1 Hz Router (200ms budget): {verdict}  (P95 = {p95:.1} ms)").unwrap();
    writeln!(out, "Bottleneck (highest P95):   {bottleneck}").unwrap();
    writeln!(out).unwrap();

    // ---- Follow-up optimizations ----------------------------------------
    writeln!(out, "FOLLOW-UP OPTIMIZATIONS").unwrap();
    writeln!(out, "-----------------------").unwrap();
    if a_p95 > 100.0 {
        writeln!(out, "[A* P95 = {a_p95:.1} ms > 100 ms]").unwrap();
        writeln!(out, "  - Lift positions+adjacency build OUT of plan(): currently rebuilt every").unwrap();
        writeln!(out, "    call (HashMap of 1M+ nodes/edges). Cache them on RouterPlugin and reuse.").unwrap();
        writeln!(out, "  - Heuristic: Euclidean is admissible but loose for grid-like networks; try").unwrap();
        writeln!(out, "    Octile or a tightened scaling factor.").unwrap();
        writeln!(out, "  - Adjacency: replace HashMap<u64, Vec<...>> with Vec<Vec<...>> + uid->index").unwrap();
        writeln!(out, "    map (built once); cuts hash overhead on the hot edge-iteration path.").unwrap();
        writeln!(out, "  - PriorityQueue: BinaryHeap re-inserts (decrease-key emulation) blow up the").unwrap();
        writeln!(out, "    heap; consider `indexmap`-backed PQ or fibonacci heap for dense graphs.").unwrap();
    }
    if n_p95 > 50.0 {
        writeln!(out, "[nearest_node P95 = {n_p95:.1} ms > 50 ms]").unwrap();
        writeln!(out, "  - Linear scan over {} nodes is O(N). Build a spatial index once at load:", graph.nodes.len()).unwrap();
        writeln!(out, "    R-Tree (`rstar` crate) or KD-Tree (`kdtree` / `kiddo` crate) gives O(log N).").unwrap();
        writeln!(out, "  - Alternative: 2D grid bucketing (cell-size ~ROI radius); cheap, no deps.").unwrap();
    }
    if w_p95 > 20.0 {
        writeln!(out, "[waypoint mat P95 = {w_p95:.1} ms > 20 ms]").unwrap();
        writeln!(out, "  - positions HashMap lookups dominate; with index-based adjacency the path").unwrap();
        writeln!(out, "    is a Vec<usize> directly indexing into a Vec<(x,z)> — O(1) per waypoint.").unwrap();
    }
    if a_p95 <= 100.0 && n_p95 <= 50.0 && w_p95 <= 20.0 {
        writeln!(out, "(no thresholds breached — current implementation fits the 1 Hz budget)").unwrap();
    }
    writeln!(out).unwrap();

    // Write output (mkdir -p).
    if let Some(parent) = args.output.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    match std::fs::write(&args.output, &out) {
        Ok(_) => eprintln!("wrote {}", args.output.display()),
        Err(e) => {
            eprintln!("ERROR: cannot write {}: {e}", args.output.display());
            std::process::exit(2);
        }
    }
    // Also echo verdict.
    eprintln!();
    eprintln!("VERDICT: {verdict} (P95={p95:.1} ms; bottleneck: {bottleneck})");
}
