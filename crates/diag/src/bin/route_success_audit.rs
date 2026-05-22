//! `route-success-audit` — A* success/failure audit over all city pairs
//!
//! Usage:
//!   cargo run --release -p truckpilot-diag --bin route-success-audit -- \
//!     --graph graph.json \
//!     --cities crates/map-parser/tests/fixtures/test_cities.toml \
//!     --snap-radius 5000.0 \
//!     --out-dir outputs/2026-05-22/diag

use std::collections::HashMap;
use std::io::Write as IoWrite;
use std::path::PathBuf;

use anyhow::{bail, Context, Result};
use clap::Parser;

#[derive(Parser)]
#[command(name = "route-success-audit", about = "Route success/failure audit over all city pairs")]
struct Args {
    #[arg(long, default_value = "graph.json")]
    graph: PathBuf,
    #[arg(long, default_value = "crates/map-parser/tests/fixtures/test_cities.toml")]
    cities: PathBuf,
    #[arg(long, default_value_t = 5000.0)]
    snap_radius: f64,
    #[arg(long, default_value = "outputs/2026-05-22/diag")]
    out_dir: PathBuf,
}

#[derive(Debug, Clone)]
struct City {
    name: String,
    x: f64,
    z: f64,
}

fn read_cities(path: &PathBuf) -> Result<Vec<City>> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("read cities: {}", path.display()))?;
    let mut cities: Vec<City> = Vec::new();
    let (mut cur_name, mut cur_x, mut cur_z): (Option<String>, Option<f64>, Option<f64>) =
        (None, None, None);

    let flush = |list: &mut Vec<City>,
                 name: &mut Option<String>,
                 x: &mut Option<f64>,
                 z: &mut Option<f64>| {
        if let (Some(n), Some(xv), Some(zv)) = (name.take(), x.take(), z.take()) {
            list.push(City { name: n, x: xv, z: zv });
        }
    };

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
    Ok(cities)
}

// Snap to nearest node within radius; returns (node_idx, dist_m)
fn snap_nearest(
    node_x: &[f64],
    node_z: &[f64],
    x: f64,
    z: f64,
    max_dist: f64,
) -> Option<(usize, f64)> {
    let max_d2 = max_dist * max_dist;
    let mut best_idx = None;
    let mut best_d2 = max_d2;
    for i in 0..node_x.len() {
        let d2 = (node_x[i] - x).powi(2) + (node_z[i] - z).powi(2);
        if d2 < best_d2 {
            best_d2 = d2;
            best_idx = Some(i);
        }
    }
    best_idx.map(|i| (i, best_d2.sqrt()))
}

// A* over index-based adjacency. Returns Some(dist_m) or None.
// g_scores and visited_gen are pre-allocated and reset via generation counter.
fn astar(
    adj: &[Vec<(usize, f32)>],
    positions: &[(f64, f64)], // (x, z) per node index
    start: usize,
    goal: usize,
    g_score: &mut [f32],
    gen: &mut [u32],
    current_gen: u32,
) -> Option<f64> {
    use std::cmp::Reverse;
    use std::collections::BinaryHeap;

    if start == goal {
        return Some(0.0);
    }

    // Reset only what we need via generation counter
    // We'll mark nodes with current_gen; anything != current_gen is "unvisited"

    let start_g = 0.0f32;
    gen[start] = current_gen;
    g_score[start] = start_g;

    // Heap entries: (priority_as_neg_f32_bits, node_idx)
    // Use (OrderedU32, usize) where OrderedU32 encodes f32 cost for min-heap.
    // We store cost as (cost * 1000) as u64 for ordering.
    let h = |idx: usize| -> u64 {
        let (gx, gz) = positions[goal];
        let (nx, nz) = positions[idx];
        let dx = gx - nx;
        let dz = gz - nz;
        ((dx * dx + dz * dz).sqrt() as u64).saturating_mul(1000)
    };

    let mut heap: BinaryHeap<Reverse<(u64, usize)>> = BinaryHeap::new();
    heap.push(Reverse((h(start), start)));

    while let Some(Reverse((_, u))) = heap.pop() {
        if u == goal {
            return Some(g_score[goal] as f64);
        }

        let gu = g_score[u];

        for &(v, w) in &adj[u] {
            let tentative = gu + w;
            if gen[v] != current_gen || tentative < g_score[v] {
                gen[v] = current_gen;
                g_score[v] = tentative;
                let fv = (tentative as u64).saturating_mul(1000) + h(v);
                heap.push(Reverse((fv, v)));
            }
        }
    }

    None
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum RouteResult {
    SnapBoth,
    SnapSource,
    SnapDest,
    NoPath,
    Success,
}

impl RouteResult {
    fn as_str(self) -> &'static str {
        match self {
            RouteResult::SnapBoth => "CAT1_SNAP_BOTH",
            RouteResult::SnapSource => "CAT1_SNAP_SOURCE",
            RouteResult::SnapDest => "CAT1_SNAP_DEST",
            RouteResult::NoPath => "CAT3_NO_PATH",
            RouteResult::Success => "CAT4_SUCCESS",
        }
    }
}

struct PairResult {
    source: String,
    target: String,
    result: RouteResult,
    snap_dist_source: Option<f64>,
    snap_dist_dest: Option<f64>,
    route_dist_m: Option<f64>,
    duration_ms: u64,
}

fn main() -> Result<()> {
    let args = Args::parse();

    eprintln!("[route-success-audit] loading graph: {}", args.graph.display());
    let t0 = std::time::Instant::now();

    let graph_bytes = std::fs::read(&args.graph)
        .with_context(|| format!("open graph: {}", args.graph.display()))?;
    let graph: serde_json::Value =
        serde_json::from_slice(&graph_bytes).context("parse graph.json")?;
    drop(graph_bytes);

    let node_arr = graph["nodes"].as_array().context("graph.nodes missing")?;
    let edge_arr = graph["edges"].as_array().context("graph.edges missing")?;

    eprintln!(
        "[route-success-audit] {} nodes, {} edges — loaded in {:.1}s",
        node_arr.len(),
        edge_arr.len(),
        t0.elapsed().as_secs_f64()
    );

    // Build index + position arrays
    let t_build = std::time::Instant::now();
    let n = node_arr.len();

    let mut uid_to_idx: HashMap<u64, usize> = HashMap::with_capacity(n);
    let mut node_x: Vec<f64> = Vec::with_capacity(n);
    let mut node_z: Vec<f64> = Vec::with_capacity(n);

    for (i, nd) in node_arr.iter().enumerate() {
        let uid = nd["uid"].as_u64().unwrap_or(0);
        uid_to_idx.insert(uid, i);
        node_x.push(nd["x"].as_f64().unwrap_or(0.0));
        node_z.push(nd["z"].as_f64().unwrap_or(0.0));
    }

    // adjacency list: Vec<Vec<(to_idx, weight_f32)>>
    let mut adj: Vec<Vec<(usize, f32)>> = vec![Vec::new(); n];

    for e in edge_arr.iter() {
        let from_uid = e["from"].as_u64().unwrap_or(0);
        let to_uid = e["to"].as_u64().unwrap_or(0);
        let dist = e["distance_m"].as_f64().unwrap_or(0.0) as f32;
        if let (Some(&fi), Some(&ti)) = (uid_to_idx.get(&from_uid), uid_to_idx.get(&to_uid)) {
            adj[fi].push((ti, dist));
        }
    }

    let build_elapsed = t_build.elapsed().as_secs_f64();
    if build_elapsed > 60.0 {
        bail!(
            "[route-success-audit] adjacency build took {build_elapsed:.1}s > 60s limit — aborting"
        );
    }
    eprintln!("[route-success-audit] adjacency built in {build_elapsed:.1}s");

    // Positions for heuristic
    let positions: Vec<(f64, f64)> = node_x.iter().zip(node_z.iter()).map(|(&x, &z)| (x, z)).collect();

    // Pre-allocate A* buffers
    let mut g_score: Vec<f32> = vec![f32::MAX; n];
    let mut gen: Vec<u32> = vec![0u32; n];
    let mut current_gen: u32 = 0;

    // Load cities
    let cities = read_cities(&args.cities)?;
    eprintln!("[route-success-audit] {} cities loaded", cities.len());

    // Snap all cities once
    let city_snaps: Vec<Option<(usize, f64)>> = cities
        .iter()
        .map(|c| snap_nearest(&node_x, &node_z, c.x, c.z, args.snap_radius))
        .collect();

    let total_pairs = cities.len() * (cities.len() - 1);
    eprintln!("[route-success-audit] running {total_pairs} directed pairs ...");

    let mut results: Vec<PairResult> = Vec::with_capacity(total_pairs);
    let mut done = 0usize;

    for (si, src) in cities.iter().enumerate() {
        for (di, dst) in cities.iter().enumerate() {
            if si == di {
                continue;
            }

            let tp = std::time::Instant::now();

            let snap_src = city_snaps[si];
            let snap_dst = city_snaps[di];

            let (result, snap_dist_source, snap_dist_dest, route_dist_m) = match (snap_src, snap_dst) {
                (None, None) => (RouteResult::SnapBoth, None, None, None),
                (None, Some((_, dd))) => (RouteResult::SnapSource, None, Some(dd), None),
                (Some((_, sd)), None) => (RouteResult::SnapDest, Some(sd), None, None),
                (Some((s_idx, sd)), Some((d_idx, dd))) => {
                    current_gen = current_gen.wrapping_add(1);
                    let route = astar(
                        &adj,
                        &positions,
                        s_idx,
                        d_idx,
                        &mut g_score,
                        &mut gen,
                        current_gen,
                    );
                    match route {
                        None => (RouteResult::NoPath, Some(sd), Some(dd), None),
                        Some(dist) => (RouteResult::Success, Some(sd), Some(dd), Some(dist)),
                    }
                }
            };

            let duration_ms = tp.elapsed().as_millis() as u64;

            results.push(PairResult {
                source: src.name.clone(),
                target: dst.name.clone(),
                result,
                snap_dist_source,
                snap_dist_dest,
                route_dist_m,
                duration_ms,
            });

            done += 1;
            if done.is_multiple_of(100) {
                eprintln!("[route-success-audit] {}/{} pairs done ...", done, total_pairs);
            }
        }
    }

    eprintln!("[route-success-audit] all {total_pairs} pairs done in {:.1}s", t0.elapsed().as_secs_f64());

    // Write outputs
    std::fs::create_dir_all(&args.out_dir)
        .with_context(|| format!("create out-dir: {}", args.out_dir.display()))?;

    // --- CSV ---
    let csv_path = args.out_dir.join("route_audit.csv");
    let mut csv_file = std::fs::File::create(&csv_path)
        .with_context(|| format!("create {}", csv_path.display()))?;

    writeln!(
        csv_file,
        "source_city,target_city,result,snap_dist_source_m,snap_dist_dest_m,route_dist_m,duration_ms"
    )?;

    for r in &results {
        let sds = r.snap_dist_source.map(|v| format!("{v:.1}")).unwrap_or_default();
        let sdd = r.snap_dist_dest.map(|v| format!("{v:.1}")).unwrap_or_default();
        let rd = r.route_dist_m.map(|v| format!("{v:.1}")).unwrap_or_default();
        writeln!(
            csv_file,
            "{},{},{},{},{},{},{}",
            r.source,
            r.target,
            r.result.as_str(),
            sds,
            sdd,
            rd,
            r.duration_ms,
        )?;
    }
    eprintln!("[route-success-audit] wrote {}", csv_path.display());

    // --- Markdown summary ---
    let md_path = args.out_dir.join("route_audit_summary.md");
    let mut md = std::fs::File::create(&md_path)
        .with_context(|| format!("create {}", md_path.display()))?;

    let success_count = results.iter().filter(|r| r.result == RouteResult::Success).count();
    let success_rate = success_count as f64 / results.len() as f64 * 100.0;

    writeln!(md, "# Route Success Audit")?;
    writeln!(md)?;
    writeln!(md, "Generated: {}", chrono_lite())?;
    writeln!(md)?;
    writeln!(md, "## Summary")?;
    writeln!(md)?;
    writeln!(md, "- Total pairs: {}", results.len())?;
    writeln!(md, "- Successful (CAT4): {} ({success_rate:.1}%)", success_count)?;
    writeln!(md)?;

    // Counts per category
    let snap_both = results.iter().filter(|r| r.result == RouteResult::SnapBoth).count();
    let snap_src = results.iter().filter(|r| r.result == RouteResult::SnapSource).count();
    let snap_dst = results.iter().filter(|r| r.result == RouteResult::SnapDest).count();
    let no_path = results.iter().filter(|r| r.result == RouteResult::NoPath).count();
    let cat1_total = snap_both + snap_src + snap_dst;

    writeln!(md, "| Category | Count | % |")?;
    writeln!(md, "|----------|-------|---|")?;
    writeln!(md, "| CAT4_SUCCESS | {success_count} | {success_rate:.1}% |")?;
    writeln!(md, "| CAT3_NO_PATH | {no_path} | {:.1}% |", no_path as f64 / results.len() as f64 * 100.0)?;
    writeln!(md, "| CAT1_SNAP_SOURCE | {snap_src} | {:.1}% |", snap_src as f64 / results.len() as f64 * 100.0)?;
    writeln!(md, "| CAT1_SNAP_DEST | {snap_dst} | {:.1}% |", snap_dst as f64 / results.len() as f64 * 100.0)?;
    writeln!(md, "| CAT1_SNAP_BOTH | {snap_both} | {:.1}% |", snap_both as f64 / results.len() as f64 * 100.0)?;
    writeln!(md)?;
    writeln!(md, "**CAT1 total** (snap failures): {cat1_total}")?;
    writeln!(md, "**CAT3 total** (A* failures): {no_path}")?;
    writeln!(md)?;

    // Per-city success table (as source)
    writeln!(md, "## Per-City Success Rate (as Source)")?;
    writeln!(md)?;
    writeln!(md, "| City | Success | Total | Rate |")?;
    writeln!(md, "|------|---------|-------|------|")?;
    for city in &cities {
        let city_results: Vec<&PairResult> = results.iter().filter(|r| r.source == city.name).collect();
        let city_success = city_results.iter().filter(|r| r.result == RouteResult::Success).count();
        let city_total = city_results.len();
        let rate = if city_total > 0 { city_success as f64 / city_total as f64 * 100.0 } else { 0.0 };
        writeln!(md, "| {} | {city_success} | {city_total} | {rate:.1}% |", city.name)?;
    }
    writeln!(md)?;

    // Failure pairs by category
    if cat1_total + no_path > 0 {
        writeln!(md, "## Failure Pairs")?;
        writeln!(md)?;

        let cat1_pairs: Vec<&PairResult> = results
            .iter()
            .filter(|r| {
                matches!(r.result, RouteResult::SnapBoth | RouteResult::SnapSource | RouteResult::SnapDest)
            })
            .collect();

        if !cat1_pairs.is_empty() {
            writeln!(md, "### CAT1 — Snap Failures")?;
            writeln!(md)?;
            writeln!(md, "| Source | Target | Category |")?;
            writeln!(md, "|--------|--------|----------|")?;
            for r in &cat1_pairs {
                writeln!(md, "| {} | {} | {} |", r.source, r.target, r.result.as_str())?;
            }
            writeln!(md)?;
        }

        let cat3_pairs: Vec<&PairResult> = results.iter().filter(|r| r.result == RouteResult::NoPath).collect();
        if !cat3_pairs.is_empty() {
            writeln!(md, "### CAT3 — No Path Found")?;
            writeln!(md)?;
            writeln!(md, "| Source | Target | Snap-Src (m) | Snap-Dst (m) |")?;
            writeln!(md, "|--------|--------|-------------|-------------|")?;
            for r in &cat3_pairs {
                let ss = r.snap_dist_source.map(|v| format!("{v:.0}")).unwrap_or_default();
                let sd = r.snap_dist_dest.map(|v| format!("{v:.0}")).unwrap_or_default();
                writeln!(md, "| {} | {} | {ss} | {sd} |", r.source, r.target)?;
            }
            writeln!(md)?;
        }
    }

    writeln!(md, "## CAT1 vs CAT3 Split")?;
    writeln!(md)?;
    writeln!(md, "- H4 hypothesis (missing snap coverage = CAT1): {cat1_total} pairs")?;
    writeln!(md, "- H5 hypothesis (graph connectivity = CAT3): {no_path} pairs")?;

    eprintln!("[route-success-audit] wrote {}", md_path.display());

    Ok(())
}

fn chrono_lite() -> String {
    use std::time::{SystemTime, UNIX_EPOCH};
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();
    let s = secs % 60;
    let m = (secs / 60) % 60;
    let h = (secs / 3600) % 24;
    let days = secs / 86400;
    let year = 1970 + days / 365;
    let day_of_year = days % 365;
    let month = day_of_year / 30 + 1;
    let day = day_of_year % 30 + 1;
    format!("{year}-{month:02}-{day:02} {h:02}:{m:02}:{s:02} UTC")
}
