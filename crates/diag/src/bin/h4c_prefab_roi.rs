//! H4c B1 — Prefab connectivity ROI quick-check.
use std::collections::BTreeMap;
use std::fs::{self, File};
use std::io::Write as _;

#[derive(serde::Deserialize)]
struct GraphFile {
    prefabs: Vec<PrefabJson>,
}
#[derive(serde::Deserialize)]
struct PrefabJson {
    connected_node_uids: Vec<u64>,
}

fn main() {
    let graph_path = std::env::args()
        .skip_while(|a| a != "--graph")
        .nth(1)
        .unwrap_or_else(|| "graph.json".into());
    let bytes = std::fs::read(&graph_path).expect("read graph.json");
    let g: GraphFile = serde_json::from_slice(&bytes).expect("parse graph.json");
    let mut hist: BTreeMap<usize, usize> = BTreeMap::new();
    let mut clique_edges = 0u64;
    let mut lane_edges_est = 0u64;
    for p in &g.prefabs {
        let n = p.connected_node_uids.len();
        *hist.entry(n).or_default() += 1;
        let n64 = n as u64;
        clique_edges += n64.saturating_mul(n64.saturating_sub(1));
        lane_edges_est += if n >= 2 {
            (2 * n64).min(n64 * (n64 - 1))
        } else {
            0
        };
    }
    let total: usize = g.prefabs.len();
    let buckets: [(usize, usize); 5] = [(1, 1), (2, 2), (3, 3), (4, 4), (5, usize::MAX)];
    let mut buf = String::new();
    buf.push_str("============================================\nH4c B1 PREFAB CONNECTIVITY ROI\n============================================\n\n");
    buf.push_str("CONNECTED-NODE-UIDS HISTOGRAM\n-----------------------------\n");
    buf.push_str(&format!("{:<6} | {:>9} | {:>6}\n", "Count", "Prefabs", "%"));
    for (lo, hi) in buckets {
        let label = if hi == usize::MAX {
            format!("{lo}+")
        } else {
            format!("{lo}")
        };
        let c: usize = hist.range(lo..=hi).map(|(_, v)| *v).sum();
        let pct = if total > 0 {
            100.0 * c as f64 / total as f64
        } else {
            0.0
        };
        buf.push_str(&format!("{label:<6} | {c:>9} | {pct:>5.1}%\n"));
    }
    buf.push_str(&format!(
        "\nMax connected_node_uids: {}\n",
        hist.keys().last().copied().unwrap_or(0)
    ));
    let n3plus: usize = hist.range(3..).map(|(_, v)| *v).sum();
    let pct3 = if total > 0 {
        100.0 * n3plus as f64 / total as f64
    } else {
        0.0
    };
    let edges3plus: u64 = g
        .prefabs
        .iter()
        .filter(|p| p.connected_node_uids.len() >= 3)
        .map(|p| {
            let n = p.connected_node_uids.len() as u64;
            n * (n - 1)
        })
        .sum();
    let phantom_saved = clique_edges.saturating_sub(lane_edges_est);
    buf.push_str("\nEDGE-IMPACT\n-----------\n");
    buf.push_str(&format!("Total prefabs:                   {total}\n"));
    buf.push_str(&format!(
        "Total edges (clique today):      {clique_edges}\n"
    ));
    buf.push_str(&format!("Edges from 3+ node prefabs:      {edges3plus}\n"));
    buf.push_str(&format!(
        "Lane-edge estimate (2*N heuristic): {lane_edges_est}\n"
    ));
    buf.push_str(&format!(
        "Phantom-edge reduction estimate: {phantom_saved}\n"
    ));
    let two_share = hist.get(&2).copied().unwrap_or(0) as f64 / total.max(1) as f64;
    let verdict = if pct3 < 10.0 && two_share > 0.9 {
        "NONE (>90% prefabs have 2 nodes — H4c quality-only, SKIP)"
    } else if phantom_saved > 100_000 && pct3 > 10.0 {
        "HIGH"
    } else if phantom_saved > 20_000 {
        "MEDIUM"
    } else {
        "LOW"
    };
    buf.push_str(&format!(
        "\nRECOMMENDATION\n--------------\nH4c ROI: {verdict}\n3+-node prefab share: {pct3:.1}%\n"
    ));
    fs::create_dir_all("outputs/claude").ok();
    File::create("outputs/h4c_b1_prefab_roi.txt")
        .unwrap()
        .write_all(buf.as_bytes())
        .unwrap();
    File::create("outputs/claude/h4c_b1_prefab_roi.txt")
        .unwrap()
        .write_all(buf.as_bytes())
        .unwrap();
    print!("{buf}");
}
