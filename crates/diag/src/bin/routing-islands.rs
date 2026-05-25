//! `routing-islands` — Phase 6.2b-Diag: Routing-Islands-Audit
//!
//! Loads graph.json, computes Strongly Connected Components (Tarjan),
//! maps test-cities to components, and writes a Markdown + JSON report.
//!
//! Usage:
//!   cargo run --release -p truckpilot-diag --bin routing-islands -- \
//!     --graph graph.json \
//!     --cities crates/map-parser/tests/fixtures/test_cities.toml \
//!     --out-dir outputs/diag \
//!     --top-n 20

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
#[command(name = "routing-islands", about = "Routing-Islands SCC Audit")]
struct Args {
    #[arg(long, default_value = "graph.json")]
    graph: PathBuf,
    #[arg(
        long,
        default_value = "crates/map-parser/tests/fixtures/test_cities.toml"
    )]
    cities: PathBuf,
    #[arg(long, default_value = "outputs/diag")]
    out_dir: PathBuf,
    #[arg(long, default_value_t = 20)]
    top_n: usize,
}

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
struct City {
    name: String,
    x: f64,
    z: f64,
}

#[derive(Debug, Clone, Serialize)]
struct ComponentInfo {
    id: usize,
    size: usize,
    edge_count: usize,
    cities: Vec<CityMapping>,
    x_min: f64,
    x_max: f64,
    z_min: f64,
    z_max: f64,
    x_center: f64,
    z_center: f64,
}

#[derive(Debug, Clone, Serialize)]
struct CityMapping {
    name: String,
    snap_dist_m: f64,
    node_uid: u64,
}

#[derive(Debug, Clone, Serialize)]
struct CityAuditRow {
    city: String,
    component_id: Option<usize>,
    component_size: Option<usize>,
    snap_dist_m: Option<f64>,
    node_uid: Option<u64>,
    note: String,
}

#[derive(Debug, Clone, Serialize)]
struct CrossEdge {
    from_comp: usize,
    to_comp: usize,
    count: usize,
}

#[derive(Debug, Serialize)]
struct ReportJson {
    graph_path: String,
    node_count: usize,
    edge_count: usize,
    total_sccs: usize,
    singleton_sccs: usize,
    cities_in_largest: usize,
    city_total: usize,
    top_components: Vec<ComponentInfo>,
    city_audit: Vec<CityAuditRow>,
    cross_edges: Vec<CrossEdge>,
}

// ---------------------------------------------------------------------------
// Tarjan SCC — O(V + E), iterative to avoid stack overflow on 1M nodes
// ---------------------------------------------------------------------------

struct TarjanState {
    index_counter: usize,
    index: Vec<Option<usize>>,
    lowlink: Vec<usize>,
    on_stack: Vec<bool>,
    stack: Vec<usize>,
    sccs: Vec<Vec<usize>>,
}

fn tarjan_scc(adj: &[Vec<usize>]) -> Vec<Vec<usize>> {
    let n = adj.len();
    let mut state = TarjanState {
        index_counter: 0,
        index: vec![None; n],
        lowlink: vec![0; n],
        on_stack: vec![false; n],
        stack: Vec::new(),
        sccs: Vec::new(),
    };

    for v in 0..n {
        if state.index[v].is_none() {
            strongconnect_iterative(v, adj, &mut state);
        }
    }

    state.sccs
}

fn strongconnect_iterative(start: usize, adj: &[Vec<usize>], st: &mut TarjanState) {
    // Each frame: (node, edge_iterator_position)
    let mut call_stack: Vec<(usize, usize)> = vec![(start, 0)];

    // Assign index/lowlink/stack for start
    st.index[start] = Some(st.index_counter);
    st.lowlink[start] = st.index_counter;
    st.index_counter += 1;
    st.stack.push(start);
    st.on_stack[start] = true;

    while let Some((v, ei)) = call_stack.last_mut() {
        let v = *v;
        if *ei < adj[v].len() {
            let w = adj[v][*ei];
            *ei += 1;
            if st.index[w].is_none() {
                st.index[w] = Some(st.index_counter);
                st.lowlink[w] = st.index_counter;
                st.index_counter += 1;
                st.stack.push(w);
                st.on_stack[w] = true;
                call_stack.push((w, 0));
            } else if st.on_stack[w] {
                let wl = st.index[w].unwrap();
                if wl < st.lowlink[v] {
                    st.lowlink[v] = wl;
                }
            }
        } else {
            // Done with v — pop frame
            call_stack.pop();
            if let Some(&(parent, _)) = call_stack.last() {
                if st.lowlink[v] < st.lowlink[parent] {
                    st.lowlink[parent] = st.lowlink[v];
                }
                // root check for v
                if st.lowlink[v] == st.index[v].unwrap() {
                    let mut scc = Vec::new();
                    while let Some(w) = st.stack.pop() {
                        st.on_stack[w] = false;
                        scc.push(w);
                        if w == v {
                            break;
                        }
                    }
                    st.sccs.push(scc);
                }
            } else {
                // v is the root of its SCC (call_stack empty after pop)
                if st.lowlink[v] == st.index[v].unwrap() {
                    let mut scc = Vec::new();
                    while let Some(w) = st.stack.pop() {
                        st.on_stack[w] = false;
                        scc.push(w);
                        if w == v {
                            break;
                        }
                    }
                    st.sccs.push(scc);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// City TOML parser (hand-rolled, no toml dep needed)
// ---------------------------------------------------------------------------

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
            list.push(City {
                name: n,
                x: xv,
                z: zv,
            });
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

// ---------------------------------------------------------------------------
// Nearest node lookup
// ---------------------------------------------------------------------------

fn nearest_node(nodes: &[(u64, f64, f64)], x: f64, z: f64) -> Option<(u64, f64)> {
    let max_dist = 5_000.0_f64; // 5 km snap radius
    let mut best_uid = None;
    let mut best_dist = max_dist * max_dist;
    for &(uid, nx, nz) in nodes {
        let d2 = (nx - x).powi(2) + (nz - z).powi(2);
        if d2 < best_dist {
            best_dist = d2;
            best_uid = Some(uid);
        }
    }
    best_uid.map(|uid| (uid, best_dist.sqrt()))
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() -> Result<()> {
    let args = Args::parse();

    eprintln!("[routing-islands] loading graph: {}", args.graph.display());
    let t0 = std::time::Instant::now();

    // Load graph.json — parse as serde_json::Value first to avoid deserializing
    // the full MapGraph type (avoids version skew issues with bincode derive).
    let graph_bytes = std::fs::read(&args.graph)
        .with_context(|| format!("open graph: {}", args.graph.display()))?;
    let graph: Value = serde_json::from_slice(&graph_bytes).context("parse graph.json")?;
    drop(graph_bytes); // free the raw bytes

    let node_arr = graph["nodes"].as_array().context("graph.nodes missing")?;
    let edge_arr = graph["edges"].as_array().context("graph.edges missing")?;

    eprintln!(
        "[routing-islands] {} nodes, {} edges — loaded in {:.1}s",
        node_arr.len(),
        edge_arr.len(),
        t0.elapsed().as_secs_f64()
    );

    // Build index: uid -> array index
    let t1 = std::time::Instant::now();
    let mut uid_to_idx: HashMap<u64, usize> = HashMap::with_capacity(node_arr.len());
    // (uid, x, z) tuples for snap lookups
    let mut node_coords: Vec<(u64, f64, f64)> = Vec::with_capacity(node_arr.len());
    // x/z per index (for bounding box)
    let mut node_x: Vec<f64> = Vec::with_capacity(node_arr.len());
    let mut node_z: Vec<f64> = Vec::with_capacity(node_arr.len());

    for (i, n) in node_arr.iter().enumerate() {
        let uid = n["uid"].as_u64().context("node uid")?;
        let x = n["x"].as_f64().unwrap_or(0.0);
        let z = n["z"].as_f64().unwrap_or(0.0);
        uid_to_idx.insert(uid, i);
        node_coords.push((uid, x, z));
        node_x.push(x);
        node_z.push(z);
    }

    // Build adjacency list (index-based)
    let n = node_arr.len();
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n];
    // For cross-edge counting we also need: node_idx -> comp_idx (filled later)
    // Store edges as (from_idx, to_idx) for cross-component analysis
    let mut raw_edges: Vec<(usize, usize)> = Vec::with_capacity(edge_arr.len());

    for e in edge_arr.iter() {
        let from_uid = e["from"].as_u64().context("edge.from")?;
        let to_uid = e["to"].as_u64().context("edge.to")?;
        if let (Some(&fi), Some(&ti)) = (uid_to_idx.get(&from_uid), uid_to_idx.get(&to_uid)) {
            adj[fi].push(ti);
            raw_edges.push((fi, ti));
        }
    }

    eprintln!(
        "[routing-islands] index + adjacency built in {:.1}s",
        t1.elapsed().as_secs_f64()
    );

    // ---------------------------------------------------------------------------
    // Tarjan SCC
    // ---------------------------------------------------------------------------
    let t2 = std::time::Instant::now();
    eprintln!("[routing-islands] running Tarjan SCC on {} nodes ...", n);
    let mut sccs = tarjan_scc(&adj);
    // Sort by size descending
    sccs.sort_by_key(|b| std::cmp::Reverse(b.len()));
    let total_sccs = sccs.len();
    let singleton_sccs = sccs.iter().filter(|c| c.len() == 1).count();
    eprintln!(
        "[routing-islands] SCCs done in {:.1}s — total={total_sccs}, singletons={singleton_sccs}",
        t2.elapsed().as_secs_f64()
    );

    // Map node index -> component id (0 = largest)
    let mut node_comp: Vec<usize> = vec![usize::MAX; n];
    for (cid, scc) in sccs.iter().enumerate() {
        for &ni in scc {
            node_comp[ni] = cid;
        }
    }

    // ---------------------------------------------------------------------------
    // Count intra-component edges per component (top-N)
    // ---------------------------------------------------------------------------
    let top_n = args.top_n.min(total_sccs);
    let mut intra_edge_count: Vec<usize> = vec![0; top_n];
    // Cross-edge matrix for top-10 pairs
    let cross_n = 10.min(top_n);
    let mut cross_matrix: HashMap<(usize, usize), usize> = HashMap::new();

    for &(fi, ti) in &raw_edges {
        let fc = node_comp[fi];
        let tc = node_comp[ti];
        if fc == tc && fc < top_n {
            intra_edge_count[fc] += 1;
        } else if fc < cross_n && tc < cross_n && fc != tc {
            *cross_matrix.entry((fc, tc)).or_insert(0) += 1;
        }
    }

    // ---------------------------------------------------------------------------
    // Build ComponentInfo for top-N
    // ---------------------------------------------------------------------------
    let mut components: Vec<ComponentInfo> = Vec::with_capacity(top_n);
    for (cid, scc) in sccs.iter().enumerate().take(top_n) {
        let mut x_min = f64::MAX;
        let mut x_max = f64::MIN;
        let mut z_min = f64::MAX;
        let mut z_max = f64::MIN;
        let mut x_sum = 0.0_f64;
        let mut z_sum = 0.0_f64;
        for &ni in scc {
            let x = node_x[ni];
            let z = node_z[ni];
            if x < x_min {
                x_min = x;
            }
            if x > x_max {
                x_max = x;
            }
            if z < z_min {
                z_min = z;
            }
            if z > z_max {
                z_max = z;
            }
            x_sum += x;
            z_sum += z;
        }
        let cnt = scc.len() as f64;
        components.push(ComponentInfo {
            id: cid,
            size: scc.len(),
            edge_count: if cid < top_n {
                intra_edge_count[cid]
            } else {
                0
            },
            cities: Vec::new(), // filled below
            x_min,
            x_max,
            z_min,
            z_max,
            x_center: x_sum / cnt,
            z_center: z_sum / cnt,
        });
    }

    // ---------------------------------------------------------------------------
    // City -> Component mapping
    // ---------------------------------------------------------------------------
    let cities = read_cities(&args.cities)?;
    let mut city_audit: Vec<CityAuditRow> = Vec::new();
    let mut cities_in_largest = 0usize;

    for city in &cities {
        match nearest_node(&node_coords, city.x, city.z) {
            None => {
                city_audit.push(CityAuditRow {
                    city: city.name.clone(),
                    component_id: None,
                    component_size: None,
                    snap_dist_m: None,
                    node_uid: None,
                    note: "NO_SNAP (no node within 5km)".to_string(),
                });
            }
            Some((uid, dist)) => {
                let idx = uid_to_idx[&uid];
                let cid = node_comp[idx];
                let (comp_id, comp_size, note) = if cid == usize::MAX {
                    (None, None, "NO_COMPONENT".to_string())
                } else {
                    let size = sccs[cid].len();
                    if cid == 0 {
                        cities_in_largest += 1;
                    }
                    (Some(cid), Some(size), format!("comp-{cid}"))
                };

                // Attach to ComponentInfo if in top-N
                if let Some(cid_val) = comp_id {
                    if let Some(ci) = components.get_mut(cid_val) {
                        ci.cities.push(CityMapping {
                            name: city.name.clone(),
                            snap_dist_m: dist,
                            node_uid: uid,
                        });
                    }
                }

                city_audit.push(CityAuditRow {
                    city: city.name.clone(),
                    component_id: comp_id,
                    component_size: comp_size,
                    snap_dist_m: Some(dist),
                    node_uid: Some(uid),
                    note,
                });
            }
        }
    }

    // ---------------------------------------------------------------------------
    // Cross-component edges list
    // ---------------------------------------------------------------------------
    let mut cross_edges: Vec<CrossEdge> = cross_matrix
        .into_iter()
        .map(|((fc, tc), count)| CrossEdge {
            from_comp: fc,
            to_comp: tc,
            count,
        })
        .collect();
    cross_edges.sort_by_key(|b| std::cmp::Reverse(b.count));

    // ---------------------------------------------------------------------------
    // Write outputs
    // ---------------------------------------------------------------------------
    std::fs::create_dir_all(&args.out_dir)
        .with_context(|| format!("create out-dir: {}", args.out_dir.display()))?;

    // --- JSON ---
    let json_path = args.out_dir.join("routing_islands.json");
    let report_json = ReportJson {
        graph_path: args.graph.display().to_string(),
        node_count: n,
        edge_count: raw_edges.len(),
        total_sccs,
        singleton_sccs,
        cities_in_largest,
        city_total: cities.len(),
        top_components: components.clone(),
        city_audit: city_audit.clone(),
        cross_edges: cross_edges.clone(),
    };
    let json_bytes = serde_json::to_vec_pretty(&report_json).context("serialize json")?;
    std::fs::write(&json_path, &json_bytes)
        .with_context(|| format!("write {}", json_path.display()))?;
    eprintln!("[routing-islands] wrote {}", json_path.display());

    // --- Markdown ---
    let md_path = args.out_dir.join("routing_islands_audit.md");
    let mut md =
        std::fs::File::create(&md_path).with_context(|| format!("create {}", md_path.display()))?;

    let now = chrono_lite();
    let largest_size = sccs.first().map(|c| c.len()).unwrap_or(0);
    let largest_pct = if n > 0 {
        largest_size as f64 / n as f64 * 100.0
    } else {
        0.0
    };

    writeln!(md, "# Routing-Islands-Audit")?;
    writeln!(md)?;
    writeln!(md, "Generated: {now}")?;
    writeln!(
        md,
        "Graph: `{}`, {} nodes, {} edges",
        args.graph.display(),
        n,
        raw_edges.len()
    )?;
    writeln!(
        md,
        "Cities-Fixture: `{}`, {} cities",
        args.cities.display(),
        cities.len()
    )?;
    writeln!(md)?;
    writeln!(md, "## Summary")?;
    writeln!(md)?;
    writeln!(md, "- Total SCCs: {total_sccs}")?;
    writeln!(
        md,
        "- Largest SCC: {} nodes ({:.1}% of graph)",
        largest_size, largest_pct
    )?;
    writeln!(
        md,
        "- Cities mapped to largest SCC: {cities_in_largest}/{}",
        cities.len()
    )?;
    writeln!(md, "- Singleton SCCs: {singleton_sccs}")?;
    writeln!(md)?;

    // Top-N table
    writeln!(md, "## Top-{top_n} Components")?;
    writeln!(md)?;
    writeln!(
        md,
        "| # | Size | Intra-Edges | Cities | BBox (x_min..x_max, z_min..z_max) | Center |"
    )?;
    writeln!(
        md,
        "|---|------|-------------|--------|-----------------------------------|--------|"
    )?;
    for ci in &components {
        let city_names: Vec<&str> = ci.cities.iter().map(|c| c.name.as_str()).collect();
        let cities_str = if city_names.is_empty() {
            "(none)".to_string()
        } else {
            city_names.join(", ")
        };
        writeln!(
            md,
            "| {} | {} | {} | {} | {:.0}..{:.0}, {:.0}..{:.0} | ({:.0}, {:.0}) |",
            ci.id + 1,
            ci.size,
            ci.edge_count,
            cities_str,
            ci.x_min,
            ci.x_max,
            ci.z_min,
            ci.z_max,
            ci.x_center,
            ci.z_center
        )?;
    }
    writeln!(md)?;

    // City -> Component table
    writeln!(md, "## Cities -> Component Mapping")?;
    writeln!(md)?;
    writeln!(
        md,
        "| City | Component-ID | Component-Size | Snap-Dist (m) | Notes |"
    )?;
    writeln!(
        md,
        "|------|--------------|----------------|---------------|-------|"
    )?;
    for row in &city_audit {
        writeln!(
            md,
            "| {} | {} | {} | {} | {} |",
            row.city,
            row.component_id
                .map(|c| (c + 1).to_string())
                .unwrap_or("-".to_string()),
            row.component_size
                .map(|s| s.to_string())
                .unwrap_or("-".to_string()),
            row.snap_dist_m
                .map(|d| format!("{d:.0}"))
                .unwrap_or("-".to_string()),
            row.note
        )?;
    }
    writeln!(md)?;

    // Cross-component edges
    writeln!(
        md,
        "## Cross-Component Connectivity (Top-{cross_n} Components)"
    )?;
    writeln!(md)?;
    if cross_edges.is_empty() {
        writeln!(
            md,
            "No cross-component edges found between top-{cross_n} components."
        )?;
    } else {
        writeln!(md, "| From Comp | To Comp | Edge-Count |")?;
        writeln!(md, "|-----------|---------|------------|")?;
        for ce in &cross_edges {
            writeln!(
                md,
                "| {} | {} | {} |",
                ce.from_comp + 1,
                ce.to_comp + 1,
                ce.count
            )?;
        }
    }
    writeln!(md)?;

    // Auto-diagnosis
    writeln!(md, "## Diagnose-Hypothesen (auto-generiert)")?;
    writeln!(md)?;
    let isolated_cities: Vec<&CityAuditRow> = city_audit
        .iter()
        .filter(|r| r.component_id.map(|c| c > 0).unwrap_or(true))
        .collect();
    let isolated_count = isolated_cities.len();

    if isolated_count == 0 {
        writeln!(
            md,
            "- **Alle Staedte in der groessten Komponente.** Graph ist stark zusammenhaengend \
             fuer alle Test-Staedte. Routing-Problem liegt anderswo (A*-Heuristik, Kosten, etc.)."
        )?;
    } else {
        // Check if isolated cities are spread across many tiny SCCs or a second big one
        let isolated_comp_ids: std::collections::HashSet<usize> = city_audit
            .iter()
            .filter(|r| r.component_id.map(|c| c > 0).unwrap_or(false))
            .filter_map(|r| r.component_id)
            .collect();

        let max_isolated_comp_size = isolated_comp_ids
            .iter()
            .filter_map(|&cid| sccs.get(cid).map(|s| s.len()))
            .max()
            .unwrap_or(0);

        writeln!(
            md,
            "- **{isolated_count} Staedte NICHT in der groessten Komponente.**"
        )?;
        writeln!(
            md,
            "  Isolierte Staedte: {}",
            isolated_cities
                .iter()
                .map(|r| r.city.as_str())
                .collect::<Vec<_>>()
                .join(", ")
        )?;
        writeln!(md)?;

        if max_isolated_comp_size > 10_000 {
            writeln!(
                md,
                "- **Zweite grosse Komponente** ({max_isolated_comp_size} Nodes) enthaelt \
                 isolierte Staedte. Hypothese: **zwei getrennte Hauptnetze** (z.B. \
                 Hauptnetz West-Europa vs. Netz Ost-Europa). Suche nach Bridge-Edges \
                 die fehlen."
            )?;
        } else {
            writeln!(
                md,
                "- Isolierte Staedte sind in **winzigen SCCs** (max {max_isolated_comp_size} \
                 Nodes). Hypothese: **Graph-Builder verbindet sie nicht** mit dem Hauptnetz. \
                 Wahrscheinliche Ursachen: \
                 (a) fehlende Cross-Sector-Edges, \
                 (b) Prefab-Connections ohne Reverse-Edge, \
                 (c) DLC-Guard filtert die Verbindungsstrassen raus."
            )?;
        }

        if cross_edges.is_empty() {
            writeln!(
                md,
                "- **Keine Cross-Component-Edges** zwischen den Top-{cross_n} Komponenten. \
                 Die Subgraphen sind **komplett disjunkt** — keine Edges die nur \
                 falsch herum sind."
            )?;
        } else {
            writeln!(
                md,
                "- Es gibt {} Cross-Component-Edges. Manche Verbindungen existieren \
                 aber sind moeglicherweise **unidirektional** (A→B aber nicht B→A), \
                 was SCC-Trennung erklaert.",
                cross_edges.iter().map(|ce| ce.count).sum::<usize>()
            )?;
        }
    }

    writeln!(md)?;
    eprintln!("[routing-islands] wrote {}", md_path.display());
    eprintln!(
        "[routing-islands] total time: {:.1}s",
        t0.elapsed().as_secs_f64()
    );

    Ok(())
}

fn chrono_lite() -> String {
    // Simple timestamp without chrono dep
    use std::time::{SystemTime, UNIX_EPOCH};
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();
    let s = secs % 60;
    let m = (secs / 60) % 60;
    let h = (secs / 3600) % 24;
    let days = secs / 86400;
    // Approximate date (good enough for a report header)
    let year = 1970 + days / 365;
    let day_of_year = days % 365;
    let month = day_of_year / 30 + 1;
    let day = day_of_year % 30 + 1;
    format!("{year}-{month:02}-{day:02} {h:02}:{m:02}:{s:02} UTC")
}

// ---------------------------------------------------------------------------
// Integration test (mini-graph, 2 disjoint components)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::tarjan_scc;

    #[test]
    fn two_disjoint_components() {
        // 0->1->2->0  (SCC of size 3)
        // 3->4        (two singletons OR one SCC if 4->3 added)
        let adj = vec![
            vec![1], // 0->1
            vec![2], // 1->2
            vec![0], // 2->0
            vec![4], // 3->4
            vec![],  // 4 (no outgoing)
        ];
        let sccs = tarjan_scc(&adj);
        // Should have 3 SCCs: {0,1,2}, {3}, {4} (or {4},{3} order)
        assert_eq!(sccs.len(), 3);
        let mut sizes: Vec<usize> = sccs.iter().map(|s| s.len()).collect();
        sizes.sort_unstable();
        assert_eq!(sizes, vec![1, 1, 3]);
    }

    #[test]
    fn fully_connected_cycle() {
        // 0->1->2->3->0 — one big SCC
        let adj = vec![vec![1], vec![2], vec![3], vec![0]];
        let sccs = tarjan_scc(&adj);
        assert_eq!(sccs.len(), 1);
        assert_eq!(sccs[0].len(), 4);
    }

    #[test]
    fn dag_all_singletons() {
        // 0->1  1->2  (DAG — no back edges)
        let adj = vec![vec![1], vec![2], vec![]];
        let sccs = tarjan_scc(&adj);
        assert_eq!(sccs.len(), 3);
        for scc in &sccs {
            assert_eq!(scc.len(), 1);
        }
    }
}
