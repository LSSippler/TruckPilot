//! `cross-sector-audit` — Phase 6.2b-Diag-2: Cross-Sector-Edge-Audit
//!
//! Analyses graph.json to answer:
//! "Why are Berlin, Hamburg etc. singleton SCCs despite being parsed?"
//!
//! Approach (no ETS2 re-parse needed):
//!   1. Classify every node by connectivity degree
//!   2. For each isolated node (degree=0): find nearest connected-node distance
//!   3. Bucket: <50m / 50-200m / 200-500m / >500m
//!   4. Count existing cross-sector edges in the graph
//!   5. Virtual-sector analysis (4096-unit grid, ETS2 standard)
//!   6. Focus-city detail trace (reads routing_islands.json for city→UID map)
//!
//! Usage:
//!   cargo run --release -p truckpilot-diag --bin cross-sector-audit -- \
//!     --graph graph.json --out-dir outputs/diag
//!
//!   cargo run --release -p truckpilot-diag --bin cross-sector-audit -- \
//!     --graph graph.json --out-dir outputs/diag --focus-city Berlin

use std::collections::HashMap;
use std::io::Write as IoWrite;
use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;
use serde::Serialize;
use serde_json::Value;

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

#[derive(Parser)]
#[command(name = "cross-sector-audit", about = "Cross-Sector-Edge-Audit")]
struct Args {
    #[arg(long, default_value = "graph.json")]
    graph: PathBuf,
    #[arg(long, default_value = "outputs/diag")]
    out_dir: PathBuf,
    #[arg(long, default_value = "outputs/diag/routing_islands.json")]
    islands_json: PathBuf,
    #[arg(long)]
    focus_city: Option<String>,
}

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const SECTOR_STEP: f64 = 4096.0;
const STUB_MARGIN: f64 = 500.0; // distance to sector boundary → "stub"
const BUCKET_1: f64 = 50.0;
const BUCKET_2: f64 = 200.0;
const BUCKET_3: f64 = 500.0;
const CELL_SIZE: f64 = 100.0; // spatial grid cell for nearest-neighbour queries

// ---------------------------------------------------------------------------
// Data
// ---------------------------------------------------------------------------

struct NodeInfo {
    uid: u64,
    x: f64,
    z: f64,
    out_degree: usize,
    in_degree: usize,
}

#[derive(Debug, Clone, Serialize)]
struct SectorStats {
    sx: i32,
    sz: i32,
    total_nodes: usize,
    isolated_nodes: usize,
    connected_nodes: usize,
    cross_sector_edges_out: usize,
    orphan_stubs: usize, // isolated nodes near boundary
}

#[derive(Debug, Clone, Serialize)]
struct CrossSectorEdgeInfo {
    direction: String,
    dist_m: f64,
    from_uid: u64,
    to_uid: u64,
    same_vsector: bool, // sanity: should be false for real cross-sector edges
}

#[derive(Debug, Clone, Serialize)]
struct CityTrace {
    name: String,
    node_uid: u64,
    x: f64,
    z: f64,
    in_degree: usize,
    out_degree: usize,
    virtual_sector: (i32, i32),
    near_boundary: bool,
    nearest_connected_dist_m: Option<f64>,
    nearest_connected_uid: Option<u64>,
    nearest_any_dist_m: Option<f64>,
    bucket: String,
    neighbor_sectors_loaded: Vec<(i32, i32)>,
    neighbor_sectors_with_connected: Vec<(i32, i32)>,
}

#[derive(Debug, Serialize)]
struct DistBuckets {
    lt_50m: usize,
    m50_200m: usize,
    m200_500m: usize,
    gt_500m: usize,
    no_neighbor_at_all: usize,
}

#[derive(Debug, Serialize)]
struct ReportJson {
    graph_path: String,
    node_count: usize,
    edge_count: usize,
    isolated_nodes: usize,
    dead_end_source: usize,
    dead_end_sink: usize,
    connected: usize,
    cross_sector_edge_count: usize,
    cross_sector_directions: HashMap<String, usize>,
    isolated_to_connected_buckets: DistBuckets,
    isolated_to_any_buckets: DistBuckets,
    virtual_sectors_total: usize,
    virtual_sectors_with_connected: usize,
    virtual_sectors_isolated_only: usize,
    virtual_sectors_empty: usize,
    top_orphan_sectors: Vec<SectorStats>,
    city_traces: Vec<CityTrace>,
}

// ---------------------------------------------------------------------------
// Spatial grid (100m cells, index over connected nodes for nearest-conn query)
// ---------------------------------------------------------------------------

type GridCell = Vec<(u64, f64, f64)>;

struct Grid {
    cells: HashMap<(i32, i32), GridCell>, // uid, x, z
    cell_size: f64,
}

impl Grid {
    fn new(cell_size: f64) -> Self {
        Self {
            cells: HashMap::new(),
            cell_size,
        }
    }

    fn insert(&mut self, uid: u64, x: f64, z: f64) {
        let key = self.key(x, z);
        self.cells.entry(key).or_default().push((uid, x, z));
    }

    fn key(&self, x: f64, z: f64) -> (i32, i32) {
        (
            (x / self.cell_size).floor() as i32,
            (z / self.cell_size).floor() as i32,
        )
    }

    /// Nearest node within `radius`. Returns (uid, dist).
    fn nearest(&self, x: f64, z: f64, radius: f64) -> Option<(u64, f64)> {
        let cr = (radius / self.cell_size).ceil() as i32 + 1;
        let (cx, cz) = self.key(x, z);
        let r2 = radius * radius;
        let mut best: Option<(u64, f64)> = None;
        for dix in -cr..=cr {
            for diz in -cr..=cr {
                if let Some(cell) = self.cells.get(&(cx + dix, cz + diz)) {
                    for &(uid, nx, nz) in cell {
                        let d2 = (nx - x).powi(2) + (nz - z).powi(2);
                        if d2 <= r2 {
                            let d = d2.sqrt();
                            if best.is_none() || d < best.unwrap().1 {
                                best = Some((uid, d));
                            }
                        }
                    }
                }
            }
        }
        best
    }
}

// ---------------------------------------------------------------------------
// Virtual sector helpers
// ---------------------------------------------------------------------------

fn vsector(x: f64, z: f64) -> (i32, i32) {
    (
        (x / SECTOR_STEP).floor() as i32,
        (z / SECTOR_STEP).floor() as i32,
    )
}

fn near_boundary(x: f64, z: f64) -> bool {
    let lx = x.rem_euclid(SECTOR_STEP);
    let lz = z.rem_euclid(SECTOR_STEP);
    let dx = lx.min(SECTOR_STEP - lx);
    let dz = lz.min(SECTOR_STEP - lz);
    dx < STUB_MARGIN || dz < STUB_MARGIN
}

fn neighbor_sectors(sx: i32, sz: i32) -> [(i32, i32); 8] {
    [
        (sx - 1, sz - 1),
        (sx, sz - 1),
        (sx + 1, sz - 1),
        (sx - 1, sz),
        (sx + 1, sz),
        (sx - 1, sz + 1),
        (sx, sz + 1),
        (sx + 1, sz + 1),
    ]
}

fn bucket_label(dist: Option<f64>) -> &'static str {
    match dist {
        None => ">500m or no-neighbor",
        Some(d) if d < BUCKET_1 => "<50m",
        Some(d) if d < BUCKET_2 => "50-200m",
        Some(d) if d < BUCKET_3 => "200-500m",
        _ => ">500m",
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() -> Result<()> {
    let args = Args::parse();
    let t0 = std::time::Instant::now();

    eprintln!(
        "[cross-sector-audit] loading graph: {}",
        args.graph.display()
    );
    let graph_bytes =
        std::fs::read(&args.graph).with_context(|| format!("open {}", args.graph.display()))?;
    let graph: Value = serde_json::from_slice(&graph_bytes).context("parse graph.json")?;
    drop(graph_bytes);

    let node_arr = graph["nodes"].as_array().context("graph.nodes")?;
    let edge_arr = graph["edges"].as_array().context("graph.edges")?;
    eprintln!(
        "[cross-sector-audit] {} nodes, {} edges — loaded in {:.1}s",
        node_arr.len(),
        edge_arr.len(),
        t0.elapsed().as_secs_f64()
    );

    // ── 1. Parse nodes ──────────────────────────────────────────────────────
    let t1 = std::time::Instant::now();
    let mut uid_to_idx: HashMap<u64, usize> = HashMap::with_capacity(node_arr.len());
    let mut nodes: Vec<NodeInfo> = Vec::with_capacity(node_arr.len());

    for (i, n) in node_arr.iter().enumerate() {
        let uid = n["uid"].as_u64().context("node uid")?;
        let x = n["x"].as_f64().unwrap_or(0.0);
        let z = n["z"].as_f64().unwrap_or(0.0);
        uid_to_idx.insert(uid, i);
        nodes.push(NodeInfo {
            uid,
            x,
            z,
            out_degree: 0,
            in_degree: 0,
        });
    }

    // ── 2. Parse edges, compute degrees, classify cross-sector ─────────────
    let mut cross_sector_edges: Vec<CrossSectorEdgeInfo> = Vec::new();
    let mut cross_dir_counts: HashMap<String, usize> = HashMap::new();

    for e in edge_arr.iter() {
        let from_uid = e["from"].as_u64().context("edge.from")?;
        let to_uid = e["to"].as_u64().context("edge.to")?;
        let dir = e["direction"].as_str().unwrap_or("unknown").to_string();
        let dist = e["distance_m"].as_f64().unwrap_or(0.0);

        if let Some(&fi) = uid_to_idx.get(&from_uid) {
            nodes[fi].out_degree += 1;
        }
        if let Some(&ti) = uid_to_idx.get(&to_uid) {
            nodes[ti].in_degree += 1;
        }

        if dir.contains("cross_sector") {
            *cross_dir_counts.entry(dir.clone()).or_insert(0) += 1;
            let fi = uid_to_idx.get(&from_uid).copied();
            let ti = uid_to_idx.get(&to_uid).copied();
            let same_vsector = match (fi, ti) {
                (Some(fi), Some(ti)) => {
                    vsector(nodes[fi].x, nodes[fi].z) == vsector(nodes[ti].x, nodes[ti].z)
                }
                _ => false,
            };
            cross_sector_edges.push(CrossSectorEdgeInfo {
                direction: dir,
                dist_m: dist,
                from_uid,
                to_uid,
                same_vsector,
            });
        }
    }

    eprintln!(
        "[cross-sector-audit] degrees + cross-sector edges ({}) computed in {:.1}s",
        cross_sector_edges.len(),
        t1.elapsed().as_secs_f64()
    );

    // ── 3. Classify nodes ───────────────────────────────────────────────────
    let mut isolated = 0usize;
    let mut dead_end_src = 0usize;
    let mut dead_end_snk = 0usize;
    let mut connected_count = 0usize;

    for n in &nodes {
        match (n.in_degree, n.out_degree) {
            (0, 0) => isolated += 1,
            (0, _) => dead_end_src += 1,
            (_, 0) => dead_end_snk += 1,
            _ => connected_count += 1,
        }
    }
    eprintln!(
        "[cross-sector-audit] isolated={isolated} dead_end_src={dead_end_src} dead_end_snk={dead_end_snk} connected={connected_count}"
    );

    // ── 4. Build spatial grids ──────────────────────────────────────────────
    let t2 = std::time::Instant::now();
    let mut connected_grid = Grid::new(CELL_SIZE);
    let mut all_grid = Grid::new(CELL_SIZE);

    for n in &nodes {
        all_grid.insert(n.uid, n.x, n.z);
        if n.in_degree > 0 && n.out_degree > 0 {
            connected_grid.insert(n.uid, n.x, n.z);
        }
    }
    eprintln!(
        "[cross-sector-audit] grids built in {:.1}s",
        t2.elapsed().as_secs_f64()
    );

    // ── 5. Virtual sector analysis ──────────────────────────────────────────
    let t3 = std::time::Instant::now();
    // sector → (total, isolated, connected, stubs_near_boundary)
    let mut sector_map: HashMap<(i32, i32), (usize, usize, usize, usize)> = HashMap::new();
    // cross_sector edges count per from-sector
    let mut sector_cross_out: HashMap<(i32, i32), usize> = HashMap::new();

    for n in &nodes {
        let vs = vsector(n.x, n.z);
        let e = sector_map.entry(vs).or_insert((0, 0, 0, 0));
        e.0 += 1;
        if n.in_degree == 0 && n.out_degree == 0 {
            e.1 += 1;
            if near_boundary(n.x, n.z) {
                e.3 += 1;
            }
        } else {
            e.2 += 1;
        }
    }
    for ce in &cross_sector_edges {
        if let Some(&fi) = uid_to_idx.get(&ce.from_uid) {
            let vs = vsector(nodes[fi].x, nodes[fi].z);
            *sector_cross_out.entry(vs).or_insert(0) += 1;
        }
    }

    let vs_total = sector_map.len();
    let vs_with_connected = sector_map.values().filter(|s| s.2 > 0).count();
    let vs_isolated_only = sector_map.values().filter(|s| s.0 > 0 && s.2 == 0).count();

    // Virtual sectors that exist in node range but have 0 nodes = "not loaded"
    // We don't know the set of "all possible sectors", only ones with nodes.
    // "Empty" in our context = sectors with 0 connected nodes (no road coverage).

    eprintln!(
        "[cross-sector-audit] {vs_total} virtual sectors ({vs_with_connected} with connected, {vs_isolated_only} isolated-only) — {:.1}s",
        t3.elapsed().as_secs_f64()
    );

    // ── 6. Distance buckets for isolated nodes ──────────────────────────────
    let t4 = std::time::Instant::now();
    let mut conn_buckets = DistBuckets {
        lt_50m: 0,
        m50_200m: 0,
        m200_500m: 0,
        gt_500m: 0,
        no_neighbor_at_all: 0,
    };
    let mut any_buckets = DistBuckets {
        lt_50m: 0,
        m50_200m: 0,
        m200_500m: 0,
        gt_500m: 0,
        no_neighbor_at_all: 0,
    };

    let isolated_nodes: Vec<&NodeInfo> = nodes
        .iter()
        .filter(|n| n.in_degree == 0 && n.out_degree == 0)
        .collect();
    let isolated_count = isolated_nodes.len();

    for n in &isolated_nodes {
        let cd = connected_grid.nearest(n.x, n.z, BUCKET_3).map(|(_, d)| d);
        let ad = all_grid.nearest(n.x, n.z, BUCKET_3).map(|(_, d)| d);

        // The grid returns self (uid matches) — subtract self from all_grid
        // by using a threshold: any_dist should ignore distance-0 (self match).
        // Since all nodes are unique positions? Not guaranteed. Use uid check.
        // Workaround: query with radius and skip uid == self.
        let ad = if ad == Some(0.0) {
            // Re-query without self: find second closest
            let (cx, cz) = all_grid.key(n.x, n.z);
            let cr = (BUCKET_3 / CELL_SIZE).ceil() as i32 + 1;
            let mut best = None;
            for dix in -cr..=cr {
                for diz in -cr..=cr {
                    if let Some(cell) = all_grid.cells.get(&(cx + dix, cz + diz)) {
                        for &(uid2, nx, nz) in cell {
                            if uid2 == n.uid {
                                continue;
                            }
                            let d2 = (nx - n.x).powi(2) + (nz - n.z).powi(2);
                            if d2 <= BUCKET_3 * BUCKET_3 {
                                let d = d2.sqrt();
                                if best.is_none() || d < best.unwrap() {
                                    best = Some(d);
                                }
                            }
                        }
                    }
                }
            }
            best
        } else {
            ad
        };

        increment_bucket(&mut conn_buckets, cd);
        increment_bucket(&mut any_buckets, ad);
    }
    eprintln!(
        "[cross-sector-audit] distance buckets ({isolated_count} isolated nodes) — {:.1}s",
        t4.elapsed().as_secs_f64()
    );

    // ── 7. Top orphan sectors ───────────────────────────────────────────────
    let mut sector_stats: Vec<SectorStats> = sector_map
        .iter()
        .filter(|(_, v)| v.1 > 0) // has at least one isolated node
        .map(|(&(sx, sz), &(total, iso, conn, stubs))| SectorStats {
            sx,
            sz,
            total_nodes: total,
            isolated_nodes: iso,
            connected_nodes: conn,
            cross_sector_edges_out: *sector_cross_out.get(&(sx, sz)).unwrap_or(&0),
            orphan_stubs: stubs,
        })
        .collect();
    sector_stats.sort_by_key(|s| std::cmp::Reverse(s.isolated_nodes));
    let top_orphan_sectors = sector_stats.into_iter().take(20).collect::<Vec<_>>();

    // ── 8. City focus traces ────────────────────────────────────────────────
    let city_traces = build_city_traces(
        &args,
        &nodes,
        &uid_to_idx,
        &connected_grid,
        &all_grid,
        &sector_map,
        &sector_cross_out,
    );

    // ── 9. Write outputs ────────────────────────────────────────────────────
    std::fs::create_dir_all(&args.out_dir)
        .with_context(|| format!("create {}", args.out_dir.display()))?;

    let report = ReportJson {
        graph_path: args.graph.display().to_string(),
        node_count: nodes.len(),
        edge_count: edge_arr.len(),
        isolated_nodes: isolated,
        dead_end_source: dead_end_src,
        dead_end_sink: dead_end_snk,
        connected: connected_count,
        cross_sector_edge_count: cross_sector_edges.len(),
        cross_sector_directions: cross_dir_counts.clone(),
        isolated_to_connected_buckets: conn_buckets,
        isolated_to_any_buckets: any_buckets,
        virtual_sectors_total: vs_total,
        virtual_sectors_with_connected: vs_with_connected,
        virtual_sectors_isolated_only: vs_isolated_only,
        virtual_sectors_empty: 0, // can't determine without full probe list
        top_orphan_sectors: top_orphan_sectors.clone(),
        city_traces: city_traces.clone(),
    };

    let json_path = args.out_dir.join("cross_sector_audit.json");
    let json_bytes = serde_json::to_vec_pretty(&report).context("serialize")?;
    std::fs::write(&json_path, &json_bytes)
        .with_context(|| format!("write {}", json_path.display()))?;
    eprintln!("[cross-sector-audit] wrote {}", json_path.display());

    let md_path = args.out_dir.join("cross_sector_audit.md");
    write_markdown(&md_path, &report, &cross_sector_edges, &city_traces)?;
    eprintln!("[cross-sector-audit] wrote {}", md_path.display());

    // Focus-city trace file
    if let Some(city_name) = &args.focus_city {
        let focus: Vec<&CityTrace> = city_traces
            .iter()
            .filter(|c| c.name.to_lowercase() == city_name.to_lowercase())
            .collect();
        if focus.is_empty() {
            eprintln!("[cross-sector-audit] WARNING: focus city '{city_name}' not found in routing_islands.json");
        } else {
            let stub_path = args
                .out_dir
                .join(format!("{}_stub_trace.md", city_name.to_lowercase()));
            write_city_trace_md(&stub_path, focus[0])?;
            eprintln!("[cross-sector-audit] wrote {}", stub_path.display());
        }
    }

    eprintln!(
        "[cross-sector-audit] total time: {:.1}s",
        t0.elapsed().as_secs_f64()
    );
    Ok(())
}

// ---------------------------------------------------------------------------
// Build city traces from routing_islands.json
// ---------------------------------------------------------------------------

fn build_city_traces(
    args: &Args,
    nodes: &[NodeInfo],
    uid_to_idx: &HashMap<u64, usize>,
    connected_grid: &Grid,
    all_grid: &Grid,
    sector_map: &HashMap<(i32, i32), (usize, usize, usize, usize)>,
    sector_cross_out: &HashMap<(i32, i32), usize>,
) -> Vec<CityTrace> {
    let Ok(bytes) = std::fs::read(&args.islands_json) else {
        eprintln!("[cross-sector-audit] routing_islands.json not found, skipping city traces");
        return Vec::new();
    };
    let Ok(json) = serde_json::from_slice::<Value>(&bytes) else {
        eprintln!("[cross-sector-audit] could not parse routing_islands.json");
        return Vec::new();
    };

    let mut traces = Vec::new();

    // city_audit array: each entry has city, node_uid, x, z, component_id
    if let Some(audit) = json["city_audit"].as_array() {
        for entry in audit {
            let city_name = entry["city"].as_str().unwrap_or("").to_string();
            let node_uid = match entry["node_uid"].as_u64() {
                Some(u) => u,
                None => continue, // no-snap city
            };

            let Some(&ni) = uid_to_idx.get(&node_uid) else {
                continue;
            };
            let n = &nodes[ni];
            let vs = vsector(n.x, n.z);
            let is_near = near_boundary(n.x, n.z);

            let conn_result = connected_grid.nearest(n.x, n.z, BUCKET_3);
            // Skip self in all_grid query
            let any_result = {
                let (cx, cz) = all_grid.key(n.x, n.z);
                let cr = (BUCKET_3 / CELL_SIZE).ceil() as i32 + 1;
                let mut best: Option<(u64, f64)> = None;
                for dix in -cr..=cr {
                    for diz in -cr..=cr {
                        if let Some(cell) = all_grid.cells.get(&(cx + dix, cz + diz)) {
                            for &(uid2, nx, nz) in cell {
                                if uid2 == node_uid {
                                    continue;
                                }
                                let d2 = (nx - n.x).powi(2) + (nz - n.z).powi(2);
                                if d2 <= BUCKET_3 * BUCKET_3 {
                                    let d = d2.sqrt();
                                    if best.is_none() || d < best.unwrap().1 {
                                        best = Some((uid2, d));
                                    }
                                }
                            }
                        }
                    }
                }
                best
            };

            // Neighbor sectors: which ones are loaded? which have connected nodes?
            let neighbors = neighbor_sectors(vs.0, vs.1);
            let neighbor_sectors_loaded: Vec<(i32, i32)> = neighbors
                .iter()
                .filter(|&&s| sector_map.contains_key(&s))
                .copied()
                .collect();
            let neighbor_sectors_with_connected: Vec<(i32, i32)> = neighbors
                .iter()
                .filter(|&&s| sector_map.get(&s).map(|v| v.2 > 0).unwrap_or(false))
                .copied()
                .collect();

            let _ = sector_cross_out; // suppress unused warning

            traces.push(CityTrace {
                name: city_name,
                node_uid,
                x: n.x,
                z: n.z,
                in_degree: n.in_degree,
                out_degree: n.out_degree,
                virtual_sector: vs,
                near_boundary: is_near,
                nearest_connected_dist_m: conn_result.map(|(_, d)| d),
                nearest_connected_uid: conn_result.map(|(u, _)| u),
                nearest_any_dist_m: any_result.map(|(_, d)| d),
                bucket: bucket_label(conn_result.map(|(_, d)| d)).to_string(),
                neighbor_sectors_loaded,
                neighbor_sectors_with_connected,
            });
        }
    }
    traces
}

// ---------------------------------------------------------------------------
// Markdown report
// ---------------------------------------------------------------------------

fn write_markdown(
    path: &PathBuf,
    r: &ReportJson,
    cross_edges: &[CrossSectorEdgeInfo],
    city_traces: &[CityTrace],
) -> Result<()> {
    let mut f =
        std::fs::File::create(path).with_context(|| format!("create {}", path.display()))?;

    let now = simple_timestamp();
    writeln!(f, "# Cross-Sector-Edge-Audit")?;
    writeln!(f)?;
    writeln!(f, "Generated: {now}")?;
    writeln!(
        f,
        "Graph: `{}`, {} nodes, {} edges",
        r.graph_path, r.node_count, r.edge_count
    )?;
    writeln!(f)?;

    writeln!(f, "## Node Connectivity Summary")?;
    writeln!(f)?;
    writeln!(f, "| Class | Count | % |")?;
    writeln!(f, "|-------|-------|---|")?;
    let n = r.node_count as f64;
    writeln!(
        f,
        "| Isolated (in=0, out=0) | {} | {:.1}% |",
        r.isolated_nodes,
        r.isolated_nodes as f64 / n * 100.0
    )?;
    writeln!(
        f,
        "| Dead-end source (in=0, out>0) | {} | {:.1}% |",
        r.dead_end_source,
        r.dead_end_source as f64 / n * 100.0
    )?;
    writeln!(
        f,
        "| Dead-end sink (in>0, out=0) | {} | {:.1}% |",
        r.dead_end_sink,
        r.dead_end_sink as f64 / n * 100.0
    )?;
    writeln!(
        f,
        "| Connected (in>0, out>0) | {} | {:.1}% |",
        r.connected,
        r.connected as f64 / n * 100.0
    )?;
    writeln!(f)?;

    writeln!(f, "## Cross-Sector Edges (already in graph)")?;
    writeln!(f)?;
    writeln!(
        f,
        "Total cross-sector edges: **{}**",
        r.cross_sector_edge_count
    )?;
    writeln!(f)?;
    writeln!(f, "| Direction type | Count |")?;
    writeln!(f, "|----------------|-------|")?;
    let mut dirs: Vec<_> = r.cross_sector_directions.iter().collect();
    dirs.sort_by_key(|(_, &v)| std::cmp::Reverse(v));
    for (dir, count) in &dirs {
        writeln!(f, "| `{dir}` | {count} |")?;
    }

    // Cross-sector edge distance histogram
    let mut ce_lt50 = 0usize;
    let mut ce_50_200 = 0usize;
    let mut ce_200_500 = 0usize;
    let mut ce_gt500 = 0usize;
    for ce in cross_edges {
        if ce.dist_m < 50.0 {
            ce_lt50 += 1;
        } else if ce.dist_m < 200.0 {
            ce_50_200 += 1;
        } else if ce.dist_m < 500.0 {
            ce_200_500 += 1;
        } else {
            ce_gt500 += 1;
        }
    }
    writeln!(f)?;
    writeln!(f, "Cross-sector edge distance histogram:")?;
    writeln!(f)?;
    writeln!(f, "| Distance | Count |")?;
    writeln!(f, "|----------|-------|")?;
    writeln!(f, "| <50m | {ce_lt50} |")?;
    writeln!(f, "| 50-200m | {ce_50_200} |")?;
    writeln!(f, "| 200-500m | {ce_200_500} |")?;
    writeln!(f, "| >500m | {ce_gt500} |")?;
    writeln!(f)?;

    writeln!(f, "## Virtual-Sector Coverage (4096-unit grid)")?;
    writeln!(f)?;
    writeln!(f, "| Metric | Value |")?;
    writeln!(f, "|--------|-------|")?;
    writeln!(
        f,
        "| Total virtual sectors with any nodes | {} |",
        r.virtual_sectors_total
    )?;
    writeln!(
        f,
        "| Virtual sectors with connected nodes | {} |",
        r.virtual_sectors_with_connected
    )?;
    writeln!(
        f,
        "| Virtual sectors with isolated nodes only | {} |",
        r.virtual_sectors_isolated_only
    )?;
    writeln!(f)?;

    writeln!(f, "## Isolated-Node Nearest-Neighbour Distance Buckets")?;
    writeln!(f)?;
    writeln!(
        f,
        "**To nearest connected node** (in=0, out=0 → nearest node with in>0 AND out>0):"
    )?;
    writeln!(f)?;
    write_bucket_table(&mut f, &r.isolated_to_connected_buckets)?;
    writeln!(f)?;
    writeln!(f, "**To nearest any node** (excluding self):")?;
    writeln!(f)?;
    write_bucket_table(&mut f, &r.isolated_to_any_buckets)?;
    writeln!(f)?;

    writeln!(f, "## Top-20 Virtual Sectors by Isolated-Node Count")?;
    writeln!(f)?;
    writeln!(
        f,
        "| Sector (sx,sz) | Total | Isolated | Connected | Cross-Sect-Out | Stubs@boundary |"
    )?;
    writeln!(
        f,
        "|---------------|-------|----------|-----------|----------------|----------------|"
    )?;
    for s in &r.top_orphan_sectors {
        writeln!(
            f,
            "| ({},{}) | {} | {} | {} | {} | {} |",
            s.sx,
            s.sz,
            s.total_nodes,
            s.isolated_nodes,
            s.connected_nodes,
            s.cross_sector_edges_out,
            s.orphan_stubs
        )?;
    }
    writeln!(f)?;

    writeln!(f, "## Cities Detail")?;
    writeln!(f)?;
    if city_traces.is_empty() {
        writeln!(f, "*(routing_islands.json not found or no city data)*")?;
    } else {
        writeln!(f, "| City | Node-UID | in | out | VirtualSector | Near-Boundary | Nearest-Connected | Bucket | Neighbors-Loaded | Neighbors-w-Connected |")?;
        writeln!(f, "|------|----------|----|-----|---------------|---------------|-------------------|--------|------------------|-----------------------|")?;
        for ct in city_traces {
            writeln!(
                f,
                "| {} | {} | {} | {} | ({},{}) | {} | {} | {} | {} | {} |",
                ct.name,
                ct.node_uid,
                ct.in_degree,
                ct.out_degree,
                ct.virtual_sector.0,
                ct.virtual_sector.1,
                ct.near_boundary,
                ct.nearest_connected_dist_m
                    .map(|d| format!("{d:.0}m"))
                    .unwrap_or("-".to_string()),
                ct.bucket,
                ct.neighbor_sectors_loaded.len(),
                ct.neighbor_sectors_with_connected.len(),
            )?;
        }
    }
    writeln!(f)?;

    // Auto-diagnosis
    writeln!(f, "## Diagnose-Hypothesen (auto-generiert)")?;
    writeln!(f)?;

    let conn_b = &r.isolated_to_connected_buckets;
    let total_iso = (conn_b.lt_50m
        + conn_b.m50_200m
        + conn_b.m200_500m
        + conn_b.gt_500m
        + conn_b.no_neighbor_at_all) as f64;

    if conn_b.lt_50m > 0 {
        let pct = conn_b.lt_50m as f64 / total_iso * 100.0;
        writeln!(f, "- **{} isolated nodes ({:.1}%) have a connected neighbour <50m** — Pass 1 (50m) SHOULD have matched these. Either:", conn_b.lt_50m, pct)?;
        writeln!(f, "  - These are NOT orphan endpoints (no road referenced them) → Pure ghost nodes, not fixable by spatial matching.")?;
        writeln!(f, "  - OR there's a bug in Pass 1 filtering.")?;
        writeln!(f)?;
    }
    if conn_b.m50_200m > 0 {
        let pct = conn_b.m50_200m as f64 / total_iso * 100.0;
        writeln!(f, "- **{} isolated nodes ({:.1}%) have a connected neighbour 50-200m away** — Pass 2 (200m radius) would potentially connect these.", conn_b.m50_200m, pct)?;
        writeln!(f)?;
    }
    if conn_b.m200_500m > 0 {
        let pct = conn_b.m200_500m as f64 / total_iso * 100.0;
        writeln!(f, "- **{} isolated nodes ({:.1}%) have a connected neighbour 200-500m away** — Pass 3 (500m radius) would potentially connect these.", conn_b.m200_500m, pct)?;
        writeln!(f)?;
    }
    if conn_b.gt_500m + conn_b.no_neighbor_at_all > 0 {
        let count = conn_b.gt_500m + conn_b.no_neighbor_at_all;
        let pct = count as f64 / total_iso * 100.0;
        writeln!(f, "- **{count} isolated nodes ({pct:.1}%) have no connected neighbour within 500m** — These are in truly isolated sub-regions. Likely explanation: DLC sectors parsed but DLC road network not fully included, OR sectors with only node data but no actual roads.")?;
        writeln!(f)?;
    }

    if r.virtual_sectors_isolated_only > 0 {
        writeln!(f, "- **{} virtual sectors have nodes but ZERO connected nodes** — These sectors were parsed (nodes extracted) but the road parser produced no edges for them. Possibly: DLC-guard filtered all roads out, or road format not yet fully supported.", r.virtual_sectors_isolated_only)?;
        writeln!(f)?;
    }

    Ok(())
}

fn write_bucket_table(f: &mut std::fs::File, b: &DistBuckets) -> Result<()> {
    let total = b.lt_50m + b.m50_200m + b.m200_500m + b.gt_500m + b.no_neighbor_at_all;
    let pct = |n: usize| {
        if total > 0 {
            n as f64 / total as f64 * 100.0
        } else {
            0.0
        }
    };
    writeln!(f, "| Distance bucket | Count | % |")?;
    writeln!(f, "|-----------------|-------|---|")?;
    writeln!(f, "| <50m | {} | {:.1}% |", b.lt_50m, pct(b.lt_50m))?;
    writeln!(f, "| 50-200m | {} | {:.1}% |", b.m50_200m, pct(b.m50_200m))?;
    writeln!(
        f,
        "| 200-500m | {} | {:.1}% |",
        b.m200_500m,
        pct(b.m200_500m)
    )?;
    writeln!(f, "| >500m | {} | {:.1}% |", b.gt_500m, pct(b.gt_500m))?;
    writeln!(
        f,
        "| no neighbour at all | {} | {:.1}% |",
        b.no_neighbor_at_all,
        pct(b.no_neighbor_at_all)
    )?;
    Ok(())
}

// ---------------------------------------------------------------------------
// City-specific trace markdown
// ---------------------------------------------------------------------------

fn write_city_trace_md(path: &PathBuf, ct: &CityTrace) -> Result<()> {
    let mut f =
        std::fs::File::create(path).with_context(|| format!("create {}", path.display()))?;
    let now = simple_timestamp();
    writeln!(f, "# Stub-Trace: {}", ct.name)?;
    writeln!(f)?;
    writeln!(f, "Generated: {now}")?;
    writeln!(f)?;
    writeln!(f, "## City Snap-Node")?;
    writeln!(f)?;
    writeln!(f, "| Field | Value |")?;
    writeln!(f, "|-------|-------|")?;
    writeln!(f, "| Node UID | {} |", ct.node_uid)?;
    writeln!(f, "| Position | ({:.1}, {:.1}) |", ct.x, ct.z)?;
    writeln!(f, "| in_degree | {} |", ct.in_degree)?;
    writeln!(f, "| out_degree | {} |", ct.out_degree)?;
    writeln!(
        f,
        "| Virtual sector | ({}, {}) |",
        ct.virtual_sector.0, ct.virtual_sector.1
    )?;
    writeln!(f, "| Near sector boundary (<500m) | {} |", ct.near_boundary)?;
    writeln!(f)?;
    writeln!(f, "## Connectivity")?;
    writeln!(f)?;
    writeln!(f, "| Metric | Value |")?;
    writeln!(f, "|--------|-------|")?;
    writeln!(
        f,
        "| Nearest connected node dist | {} |",
        ct.nearest_connected_dist_m
            .map(|d| format!("{d:.1}m"))
            .unwrap_or(">500m".to_string())
    )?;
    writeln!(
        f,
        "| Nearest connected node UID | {} |",
        ct.nearest_connected_uid
            .map(|u| u.to_string())
            .unwrap_or("-".to_string())
    )?;
    writeln!(
        f,
        "| Nearest any node dist | {} |",
        ct.nearest_any_dist_m
            .map(|d| format!("{d:.1}m"))
            .unwrap_or(">500m".to_string())
    )?;
    writeln!(f, "| Distance bucket | {} |", ct.bucket)?;
    writeln!(f)?;
    writeln!(f, "## Neighbour Sector Analysis")?;
    writeln!(f)?;
    writeln!(f, "| Sector | Loaded (has nodes) | Has connected nodes |")?;
    writeln!(f, "|--------|-------------------|---------------------|")?;
    let (sx, sz) = ct.virtual_sector;
    for &(nsx, nsz) in &neighbor_sectors(sx, sz) {
        let loaded = ct.neighbor_sectors_loaded.contains(&(nsx, nsz));
        let has_conn = ct.neighbor_sectors_with_connected.contains(&(nsx, nsz));
        writeln!(f, "| ({nsx},{nsz}) | {loaded} | {has_conn} |")?;
    }
    writeln!(f)?;
    writeln!(f, "## Diagnosis")?;
    writeln!(f)?;
    if ct.in_degree == 0 && ct.out_degree == 0 {
        writeln!(
            f,
            "Node `{}` is **completely isolated** (degree=0).",
            ct.name
        )?;
        writeln!(f)?;
        match ct.nearest_connected_dist_m {
            None => writeln!(f, "No connected node within 500m. This node is in a sector with no road coverage — likely a DLC region or unparsed road type.")?,
            Some(d) if d < 50.0 => writeln!(f, "Connected node at {d:.1}m (<50m). Pass 1 should have found this. Possible causes: (a) no road with this node as endpoint existed in the sector data; (b) Z-tolerance filter rejected the candidate; (c) the road had both endpoints missing (both_unresolved, dropped silently).")?,
            Some(d) if d < 200.0 => writeln!(f, "Connected node at {d:.1}m (50-200m). Pass 1 (50m) missed this. **Pass 2 at 200m would connect this city.**")?,
            Some(d) if d < 500.0 => writeln!(f, "Connected node at {d:.1}m (200-500m). Pass 1+2 missed this. **Pass 3 at 500m would potentially connect this city.**")?,
            Some(d) => writeln!(f, "Nearest connected node at {d:.1}m (>500m). Spatial matching at any reasonable radius won't help. The sector has roads but they're far away.")?,
        }
    } else {
        writeln!(f, "Node has edges (in={}, out={}). It's not fully isolated — it's in a tiny SCC that can't reach the main network.", ct.in_degree, ct.out_degree)?;
        writeln!(
            f,
            "This is a directional connectivity issue, not a missing-node issue."
        )?;
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn increment_bucket(b: &mut DistBuckets, dist: Option<f64>) {
    match dist {
        None => b.no_neighbor_at_all += 1,
        Some(d) if d < BUCKET_1 => b.lt_50m += 1,
        Some(d) if d < BUCKET_2 => b.m50_200m += 1,
        Some(d) if d < BUCKET_3 => b.m200_500m += 1,
        _ => b.gt_500m += 1,
    }
}

fn simple_timestamp() -> String {
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
    let doy = days % 365;
    let mon = doy / 30 + 1;
    let day = doy % 30 + 1;
    format!("{year}-{mon:02}-{day:02} {h:02}:{m:02}:{s:02} UTC")
}
