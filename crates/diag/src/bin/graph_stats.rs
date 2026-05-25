//! `graph-stats` — structural statistics over graph.json
//!
//! Usage:
//!   cargo run --release -p truckpilot-diag --bin graph-stats -- \
//!     --graph graph.json \
//!     --out-dir outputs/2026-05-22/diag

use std::collections::HashMap;
use std::io::Write as IoWrite;
use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;
use serde::Serialize;
use serde_json::Value;

#[derive(Parser)]
#[command(name = "graph-stats", about = "Graph structural statistics")]
struct Args {
    #[arg(long, default_value = "graph.json")]
    graph: PathBuf,
    #[arg(long, default_value = "outputs/2026-05-22/diag")]
    out_dir: PathBuf,
}

#[derive(Debug, Serialize)]
struct GraphStats {
    total_nodes: usize,
    total_edges: usize,
    total_signs: usize,
    total_prefabs: usize,
    edge_type_distribution: HashMap<String, usize>,
    raw_direction_distribution: HashMap<String, usize>,
    dlc_guard_base_edges: usize,
    dlc_guard_dlc_edges: usize,
    dlc_guard_top5: Vec<(u64, usize)>,
    gps_avoid_edges: usize,
    gps_avoid_pct: f64,
    hidden_edges: usize,
    hidden_pct: f64,
    speed_limit_known_edges: usize,
    speed_limit_unknown_edges: usize,
    isolated_nodes: usize,
    source_only_nodes: usize,
    sink_only_nodes: usize,
    avg_out_degree: f64,
    avg_in_degree: f64,
    prefab_nodes_total: usize,
    prefab_nodes_also_in_edges: usize,
    build_stats: Value,
}

fn main() -> Result<()> {
    let args = Args::parse();

    eprintln!("[graph-stats] loading graph: {}", args.graph.display());
    let t0 = std::time::Instant::now();

    let graph_bytes = std::fs::read(&args.graph)
        .with_context(|| format!("open graph: {}", args.graph.display()))?;
    let graph: Value = serde_json::from_slice(&graph_bytes).context("parse graph.json")?;
    drop(graph_bytes);

    let node_arr = graph["nodes"].as_array().context("graph.nodes missing")?;
    let edge_arr = graph["edges"].as_array().context("graph.edges missing")?;
    let sign_arr = graph["signs"].as_array().context("graph.signs missing")?;
    let prefab_arr = graph["prefabs"]
        .as_array()
        .context("graph.prefabs missing")?;

    eprintln!(
        "[graph-stats] {} nodes, {} edges, {} signs, {} prefabs — loaded in {:.1}s",
        node_arr.len(),
        edge_arr.len(),
        sign_arr.len(),
        prefab_arr.len(),
        t0.elapsed().as_secs_f64()
    );

    let total_nodes = node_arr.len();
    let total_edges = edge_arr.len();
    let total_signs = sign_arr.len();
    let total_prefabs = prefab_arr.len();

    // --- Node-Degree-Analysis (index-based, not HashMap<u64,_>) ---
    let mut uid_to_idx: HashMap<u64, usize> = HashMap::with_capacity(total_nodes);
    for (i, n) in node_arr.iter().enumerate() {
        let uid = n["uid"].as_u64().unwrap_or(0);
        uid_to_idx.insert(uid, i);
    }

    let mut out_degree: Vec<u32> = vec![0u32; total_nodes];
    let mut in_degree: Vec<u32> = vec![0u32; total_nodes];

    // --- Edge iteration (single pass) ---
    let mut edge_type_distribution: HashMap<String, usize> = HashMap::new();
    let mut raw_direction_distribution: HashMap<String, usize> = HashMap::new();
    let mut dlc_guard_counts: HashMap<u64, usize> = HashMap::new();
    let mut gps_avoid_edges = 0usize;
    let mut hidden_edges = 0usize;
    let mut speed_limit_known = 0usize;
    let mut speed_limit_unknown = 0usize;

    for e in edge_arr.iter() {
        let from_uid = e["from"].as_u64().unwrap_or(0);
        let to_uid = e["to"].as_u64().unwrap_or(0);

        if let (Some(&fi), Some(&ti)) = (uid_to_idx.get(&from_uid), uid_to_idx.get(&to_uid)) {
            out_degree[fi] = out_degree[fi].saturating_add(1);
            in_degree[ti] = in_degree[ti].saturating_add(1);
        }

        let dir = e["direction"].as_str().unwrap_or("").to_ascii_lowercase();
        *raw_direction_distribution.entry(dir.clone()).or_insert(0) += 1;

        // bucket into canonical types
        let bucket = match dir.as_str() {
            "forward" => "forward",
            "backward" => "backward",
            d if d.contains("prefab") => "prefab",
            d if d.contains("building") => "building",
            d if d.contains("ferry") => "ferry",
            _ => "other",
        };
        *edge_type_distribution
            .entry(bucket.to_string())
            .or_insert(0) += 1;

        let dlc_guard = e["dlc_guard"].as_u64().unwrap_or(0);
        *dlc_guard_counts.entry(dlc_guard).or_insert(0) += 1;

        if e["gps_avoid"].as_bool().unwrap_or(false) {
            gps_avoid_edges += 1;
        }
        if e["is_hidden"].as_bool().unwrap_or(false) {
            hidden_edges += 1;
        }

        if e["speed_limit_kmh"].is_null() || !e["speed_limit_kmh"].is_number() {
            speed_limit_unknown += 1;
        } else {
            speed_limit_known += 1;
        }
    }

    let dlc_guard_base_edges = dlc_guard_counts.get(&0).copied().unwrap_or(0);
    let dlc_guard_dlc_edges = total_edges - dlc_guard_base_edges;

    // Top-5 DLC guard values (excluding 0 = base game)
    let mut dlc_entries: Vec<(u64, usize)> = dlc_guard_counts
        .iter()
        .filter(|(&k, _)| k != 0)
        .map(|(&k, &v)| (k, v))
        .collect();
    dlc_entries.sort_by_key(|&(_, count)| std::cmp::Reverse(count));
    dlc_entries.truncate(5);

    // --- Degree summary ---
    let mut isolated = 0usize;
    let mut source_only = 0usize;
    let mut sink_only = 0usize;
    let mut out_sum = 0u64;
    let mut in_sum = 0u64;

    for i in 0..total_nodes {
        let od = out_degree[i];
        let id = in_degree[i];
        out_sum += od as u64;
        in_sum += id as u64;
        match (od, id) {
            (0, 0) => isolated += 1,
            (od, 0) if od > 0 => source_only += 1,
            (0, id) if id > 0 => sink_only += 1,
            _ => {}
        }
    }

    let avg_out = if total_nodes > 0 {
        out_sum as f64 / total_nodes as f64
    } else {
        0.0
    };
    let avg_in = if total_nodes > 0 {
        in_sum as f64 / total_nodes as f64
    } else {
        0.0
    };

    // --- Prefab connectivity ---
    // Collect all node UIDs that appear in edges (as from or to)
    let mut edge_node_set: std::collections::HashSet<u64> =
        std::collections::HashSet::with_capacity(total_nodes);
    for e in edge_arr.iter() {
        if let Some(f) = e["from"].as_u64() {
            edge_node_set.insert(f);
        }
        if let Some(t) = e["to"].as_u64() {
            edge_node_set.insert(t);
        }
    }

    let mut prefab_nodes_total = 0usize;
    let mut prefab_nodes_in_edges = 0usize;
    for p in prefab_arr.iter() {
        if let Some(arr) = p["connected_node_uids"].as_array() {
            for uid_val in arr {
                if let Some(uid) = uid_val.as_u64() {
                    prefab_nodes_total += 1;
                    if edge_node_set.contains(&uid) {
                        prefab_nodes_in_edges += 1;
                    }
                }
            }
        }
    }

    let build_stats = graph["stats"].clone();

    let gps_avoid_pct = if total_edges > 0 {
        gps_avoid_edges as f64 / total_edges as f64 * 100.0
    } else {
        0.0
    };
    let hidden_pct = if total_edges > 0 {
        hidden_edges as f64 / total_edges as f64 * 100.0
    } else {
        0.0
    };

    let stats = GraphStats {
        total_nodes,
        total_edges,
        total_signs,
        total_prefabs,
        edge_type_distribution,
        raw_direction_distribution,
        dlc_guard_base_edges,
        dlc_guard_dlc_edges,
        dlc_guard_top5: dlc_entries,
        gps_avoid_edges,
        gps_avoid_pct,
        hidden_edges,
        hidden_pct,
        speed_limit_known_edges: speed_limit_known,
        speed_limit_unknown_edges: speed_limit_unknown,
        isolated_nodes: isolated,
        source_only_nodes: source_only,
        sink_only_nodes: sink_only,
        avg_out_degree: avg_out,
        avg_in_degree: avg_in,
        prefab_nodes_total,
        prefab_nodes_also_in_edges: prefab_nodes_in_edges,
        build_stats,
    };

    std::fs::create_dir_all(&args.out_dir)
        .with_context(|| format!("create out-dir: {}", args.out_dir.display()))?;

    // --- JSON output ---
    let json_path = args.out_dir.join("graph_stats.json");
    let json_bytes = serde_json::to_vec_pretty(&stats).context("serialize json")?;
    std::fs::write(&json_path, &json_bytes)
        .with_context(|| format!("write {}", json_path.display()))?;
    eprintln!("[graph-stats] wrote {}", json_path.display());

    // --- Markdown output ---
    let md_path = args.out_dir.join("graph_stats.md");
    let mut md =
        std::fs::File::create(&md_path).with_context(|| format!("create {}", md_path.display()))?;

    let now = chrono_lite();
    writeln!(md, "# Graph Stats")?;
    writeln!(md)?;
    writeln!(md, "Generated: {now}")?;
    writeln!(md, "Graph: `{}`", args.graph.display())?;
    writeln!(md)?;
    writeln!(md, "## Counts")?;
    writeln!(md)?;
    writeln!(md, "| Metric | Value |")?;
    writeln!(md, "|--------|-------|")?;
    writeln!(md, "| total_nodes | {} |", stats.total_nodes)?;
    writeln!(md, "| total_edges | {} |", stats.total_edges)?;
    writeln!(md, "| total_signs | {} |", stats.total_signs)?;
    writeln!(md, "| total_prefabs | {} |", stats.total_prefabs)?;
    writeln!(md)?;

    writeln!(md, "## Edge Type Distribution")?;
    writeln!(md)?;
    writeln!(md, "| Direction Bucket | Count | % |")?;
    writeln!(md, "|-----------------|-------|---|")?;
    let mut buckets: Vec<(&String, &usize)> = stats.edge_type_distribution.iter().collect();
    buckets.sort_by_key(|(k, _)| k.as_str());
    for (k, v) in &buckets {
        let pct = **v as f64 / stats.total_edges as f64 * 100.0;
        writeln!(md, "| {k} | {v} | {pct:.2}% |")?;
    }
    writeln!(md)?;

    writeln!(md, "## DLC Guard Distribution")?;
    writeln!(md)?;
    writeln!(md, "| Category | Count | % |")?;
    writeln!(md, "|----------|-------|---|")?;
    let base_pct = stats.dlc_guard_base_edges as f64 / stats.total_edges as f64 * 100.0;
    let dlc_pct = stats.dlc_guard_dlc_edges as f64 / stats.total_edges as f64 * 100.0;
    writeln!(
        md,
        "| base (guard=0) | {} | {base_pct:.2}% |",
        stats.dlc_guard_base_edges
    )?;
    writeln!(
        md,
        "| dlc (guard>0) | {} | {dlc_pct:.2}% |",
        stats.dlc_guard_dlc_edges
    )?;
    writeln!(md)?;
    writeln!(md, "Top-5 DLC guard values:")?;
    writeln!(md)?;
    writeln!(md, "| dlc_guard | Count |")?;
    writeln!(md, "|-----------|-------|")?;
    for (guard, count) in &stats.dlc_guard_top5 {
        writeln!(md, "| {guard} | {count} |")?;
    }
    writeln!(md)?;

    writeln!(md, "## Edge Flags")?;
    writeln!(md)?;
    writeln!(md, "| Flag | Count | % |")?;
    writeln!(md, "|------|-------|---|")?;
    writeln!(
        md,
        "| gps_avoid | {} | {:.2}% |",
        stats.gps_avoid_edges, stats.gps_avoid_pct
    )?;
    writeln!(
        md,
        "| is_hidden | {} | {:.2}% |",
        stats.hidden_edges, stats.hidden_pct
    )?;
    let spd_known_pct = stats.speed_limit_known_edges as f64 / stats.total_edges as f64 * 100.0;
    let spd_unk_pct = stats.speed_limit_unknown_edges as f64 / stats.total_edges as f64 * 100.0;
    writeln!(
        md,
        "| speed_limit_known | {} | {spd_known_pct:.2}% |",
        stats.speed_limit_known_edges
    )?;
    writeln!(
        md,
        "| speed_limit_unknown | {} | {spd_unk_pct:.2}% |",
        stats.speed_limit_unknown_edges
    )?;
    writeln!(md)?;

    writeln!(md, "## Node Degree Analysis")?;
    writeln!(md)?;
    writeln!(md, "| Metric | Value |")?;
    writeln!(md, "|--------|-------|")?;
    writeln!(
        md,
        "| isolated_nodes (in=0, out=0) | {} |",
        stats.isolated_nodes
    )?;
    writeln!(
        md,
        "| source_only_nodes (out>0, in=0) | {} |",
        stats.source_only_nodes
    )?;
    writeln!(
        md,
        "| sink_only_nodes (out=0, in>0) | {} |",
        stats.sink_only_nodes
    )?;
    writeln!(md, "| avg_out_degree | {:.3} |", stats.avg_out_degree)?;
    writeln!(md, "| avg_in_degree | {:.3} |", stats.avg_in_degree)?;
    writeln!(md)?;

    writeln!(md, "## Prefab Connectivity")?;
    writeln!(md)?;
    writeln!(md, "| Metric | Value |")?;
    writeln!(md, "|--------|-------|")?;
    writeln!(
        md,
        "| prefab_nodes_total (all connected_node_uids) | {} |",
        stats.prefab_nodes_total
    )?;
    writeln!(
        md,
        "| prefab_nodes_also_in_edges | {} |",
        stats.prefab_nodes_also_in_edges
    )?;
    if stats.prefab_nodes_total > 0 {
        let pct = stats.prefab_nodes_also_in_edges as f64 / stats.prefab_nodes_total as f64 * 100.0;
        writeln!(md, "| coverage_pct | {pct:.2}% |")?;
    }
    writeln!(md)?;

    if !stats.build_stats.is_null() {
        writeln!(md, "## BuildStats")?;
        writeln!(md)?;
        writeln!(md, "```json")?;
        writeln!(md, "{}", serde_json::to_string_pretty(&stats.build_stats)?)?;
        writeln!(md, "```")?;
        writeln!(md)?;
    }

    eprintln!("[graph-stats] wrote {}", md_path.display());
    eprintln!(
        "[graph-stats] total time: {:.1}s",
        t0.elapsed().as_secs_f64()
    );

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
