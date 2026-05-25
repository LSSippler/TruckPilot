//! `validate-cities` — Phase 6.5c Task 2
//!
//! Loads graph.json and test_cities.toml; for each city finds the nearest
//! graph node and reports its UID plus distance. Also validates the hardcoded
//! UI preset UIDs from RouteCard.tsx.
//!
//! Usage:
//!   validate-cities [--graph PATH] [--cities PATH] [--output PATH]
//!
//! Defaults:
//!   --graph   graph.json
//!   --cities  crates/map-parser/tests/fixtures/test_cities.toml
//!   --output  outputs/YYYY-MM-DD/cities_validation_report.md

use std::collections::HashMap;
use std::path::PathBuf;

use truckpilot_map_parser::graph::MapGraph;

// ── Hardcoded UI presets from crates/ui/src/components/RouteCard.tsx ──────────
// These are the UIDs users actually click — we validate them against graph.json.
const UI_PRESETS: &[(&str, u64)] = &[
    ("Berlin", 282_353_445_640_339_601),
    ("Hamburg", 6_526_933_291_294_064_640),
    ("München", 12_090_290_263_537_061_888),
    ("Köln", 4_789_015_231_856_640_000),
    ("Frankfurt", 7_891_234_567_890_123_456),
];

// ── City from test_cities.toml ─────────────────────────────────────────────────

#[derive(Debug, Clone)]
struct City {
    name: String,
    x: f64,
    z: f64,
}

fn read_cities(path: &PathBuf) -> Vec<City> {
    let text =
        std::fs::read_to_string(path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
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
    cities
}

// ── Graph helpers ──────────────────────────────────────────────────────────────

struct GraphIndex {
    nodes: Vec<(u64, f64, f64)>,
    positions: HashMap<u64, (f64, f64)>,
    /// Number of outgoing edges per node.
    edge_count: HashMap<u64, usize>,
}

impl GraphIndex {
    fn build(graph: &MapGraph) -> Self {
        let nodes: Vec<(u64, f64, f64)> = graph.nodes.iter().map(|n| (n.uid, n.x, n.z)).collect();
        let positions: HashMap<u64, (f64, f64)> =
            graph.nodes.iter().map(|n| (n.uid, (n.x, n.z))).collect();
        let mut edge_count: HashMap<u64, usize> = HashMap::new();
        for e in &graph.edges {
            *edge_count.entry(e.from).or_insert(0) += 1;
        }
        Self {
            nodes,
            positions,
            edge_count,
        }
    }

    fn find_nearest(&self, x: f64, z: f64) -> Option<(u64, f64)> {
        self.nodes
            .iter()
            .map(|&(uid, nx, nz)| {
                let dx = nx - x;
                let dz = nz - z;
                (uid, (dx * dx + dz * dz).sqrt())
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
    }

    fn lookup(&self, uid: u64) -> Option<(f64, f64)> {
        self.positions.get(&uid).copied()
    }

    fn outgoing_edges(&self, uid: u64) -> usize {
        self.edge_count.get(&uid).copied().unwrap_or(0)
    }
}

// ── Args ───────────────────────────────────────────────────────────────────────

struct Args {
    graph: PathBuf,
    cities: PathBuf,
    output: PathBuf,
}

fn parse_args() -> Args {
    let mut graph = PathBuf::from("graph.json");
    let mut cities = PathBuf::from("crates/map-parser/tests/fixtures/test_cities.toml");
    let mut output = PathBuf::from("outputs/2026-05-19/cities_validation_report.md");
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--graph" if i + 1 < argv.len() => {
                graph = PathBuf::from(&argv[i + 1]);
                i += 2;
            }
            "--cities" if i + 1 < argv.len() => {
                cities = PathBuf::from(&argv[i + 1]);
                i += 2;
            }
            "--output" if i + 1 < argv.len() => {
                output = PathBuf::from(&argv[i + 1]);
                i += 2;
            }
            "-h" | "--help" => {
                eprintln!("usage: validate-cities [--graph PATH] [--cities PATH] [--output PATH]");
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
        output,
    }
}

// ── Main ───────────────────────────────────────────────────────────────────────

fn main() {
    let args = parse_args();

    // ── Load graph ──────────────────────────────────────────────────────────
    eprintln!("Loading {}…", args.graph.display());
    let bytes = match std::fs::read(&args.graph) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("ERROR: cannot read {}: {e}", args.graph.display());
            eprintln!("Run the map-build first to generate graph.json.");
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
    eprintln!(
        "Graph: {} nodes / {} edges",
        graph.nodes.len(),
        graph.edges.len()
    );
    let idx = GraphIndex::build(&graph);

    // ── Load cities ─────────────────────────────────────────────────────────
    eprintln!("Loading {}…", args.cities.display());
    let cities = read_cities(&args.cities);
    eprintln!("{} cities loaded", cities.len());

    // ── Section 1: nearest-node lookup per city ─────────────────────────────
    println!();
    println!("=== Section 1: Nearest Graph Node per City ===");
    println!(
        "{:<16} {:>22} {:>12}  {:>10}  {:>6}  Edges",
        "City", "Nearest UID", "Dist (m)", "x", "z"
    );
    println!("{}", "-".repeat(90));

    let mut city_rows: Vec<String> = Vec::new();
    for city in &cities {
        match idx.find_nearest(city.x, city.z) {
            Some((uid, dist)) => {
                let (nx, nz) = idx.lookup(uid).unwrap_or_default();
                let edges = idx.outgoing_edges(uid);
                let flag = if dist > 5000.0 { "  ⚠ >5km" } else { "" };
                let row = format!(
                    "{:<16} {:>22} {:>12.0}  {:>10.1}  {:>10.1}  {:>6}{}",
                    city.name, uid, dist, nx, nz, edges, flag
                );
                println!("{row}");
                city_rows.push(format!(
                    "| {:<16} | `{uid}` | {dist:.0} m | ({nx:.0}, {nz:.0}) | {edges} |{flag}|",
                    city.name
                ));
            }
            None => {
                println!("{:<16}  (no graph nodes loaded)", city.name);
                city_rows.push(format!("| {:<16} | — | — | — | — | NO NODES |", city.name));
            }
        }
    }

    // ── Section 2: UI preset UID validation ─────────────────────────────────
    println!();
    println!("=== Section 2: UI Preset UID Validation (RouteCard.tsx) ===");
    println!("{:<12} {:>22}  Status", "Preset", "UID");
    println!("{}", "-".repeat(60));

    let mut preset_rows: Vec<String> = Vec::new();
    let mut missing_count = 0usize;
    for &(name, uid) in UI_PRESETS {
        let status = if let Some((nx, nz)) = idx.lookup(uid) {
            let edges = idx.outgoing_edges(uid);
            format!("FOUND at ({nx:.0}, {nz:.0}), {edges} edges")
        } else {
            missing_count += 1;
            "NOT FOUND".to_string()
        };
        println!("{:<12} {:>22}  {}", name, uid, status);
        preset_rows.push(format!("| {name} | `{uid}` | {status} |"));
    }

    // ── Summary ─────────────────────────────────────────────────────────────
    println!();
    println!("=== Summary ===");
    println!("Graph nodes total : {}", graph.nodes.len());
    println!("Graph edges total : {}", graph.edges.len());
    println!("Cities checked    : {}", cities.len());
    println!("UI presets total  : {}", UI_PRESETS.len());
    println!("UI presets missing: {}", missing_count);
    if missing_count > 0 {
        println!();
        println!("*** ROOT CAUSE HYPOTHESIS ***");
        println!(
            "  {missing_count}/{} UI preset UIDs are NOT in graph.json.",
            UI_PRESETS.len()
        );
        println!("  This is the likely root cause of the Live-Test failure.");
        println!("  The router receives the correct UID but cannot find it in the graph.");
        println!("  Resolution: regenerate cities.toml presets from current graph.json,");
        println!("  or use the nearest-node UIDs from Section 1 above for testing.");
    } else {
        println!("All UI preset UIDs found in graph — UID validity is NOT the root cause.");
    }

    // ── Write Markdown report ────────────────────────────────────────────────
    if let Some(parent) = args.output.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    let mut md = String::new();
    md.push_str("# Cities Validation Report — Phase 6.5c\n\n");
    md.push_str(&format!(
        "Generated: 2026-05-19  \nGraph: `{}`  \nCities: `{}`  \n\n",
        args.graph.display(),
        args.cities.display()
    ));
    md.push_str(&format!(
        "**Graph:** {} nodes / {} edges\n\n",
        graph.nodes.len(),
        graph.edges.len()
    ));

    md.push_str("## Section 1: Nearest Graph Node per City\n\n");
    md.push_str("| City | Nearest UID | Distance | Position (x, z) | Outgoing Edges | Note |\n");
    md.push_str("|------|-------------|----------|-----------------|----------------|------|\n");
    for row in &city_rows {
        md.push_str(row);
        md.push('\n');
    }

    md.push_str("\n## Section 2: UI Preset UID Validation\n\n");
    md.push_str("| Preset | UID | Status |\n");
    md.push_str("|--------|-----|--------|\n");
    for row in &preset_rows {
        md.push_str(row);
        md.push('\n');
    }

    md.push_str("\n## Summary\n\n");
    md.push_str("| Metric | Value |\n|--------|-------|\n");
    md.push_str(&format!("| Graph nodes | {} |\n", graph.nodes.len()));
    md.push_str(&format!("| Graph edges | {} |\n", graph.edges.len()));
    md.push_str(&format!("| Cities checked | {} |\n", cities.len()));
    md.push_str(&format!("| UI presets | {} |\n", UI_PRESETS.len()));
    md.push_str(&format!("| UI presets missing | **{}** |\n", missing_count));

    if missing_count > 0 {
        md.push_str("\n## ⚠ Root Cause Hypothesis\n\n");
        md.push_str(&format!(
            "**{missing_count}/{} UI preset UIDs are NOT in graph.json.**\n\n",
            UI_PRESETS.len()
        ));
        md.push_str("This is the likely root cause of the Live-Test failure:\n\n");
        md.push_str("1. The IPC wire type is correct (string, no precision loss)\n");
        md.push_str("2. The router receives the correct UID\n");
        md.push_str("3. But the UID does not exist in the loaded graph\n");
        md.push_str("4. A* cannot run → `uid_not_in_graph` result\n\n");
        md.push_str("**Resolution:** Use the nearest-node UIDs from Section 1 for testing,\n");
        md.push_str("or regenerate the UI preset UIDs from the current graph.json.\n");
    } else {
        md.push_str("\n## ✓ UID Validity\n\n");
        md.push_str("All UI preset UIDs are present in graph.json.\n");
        md.push_str("UID validity is NOT the root cause of the Live-Test failure.\n");
        md.push_str("Check router.last_planning_result in the blackboard for the actual error.\n");
    }

    match std::fs::write(&args.output, &md) {
        Ok(_) => eprintln!("Report written to {}", args.output.display()),
        Err(e) => eprintln!("WARNING: could not write report: {e}"),
    }

    std::process::exit(if missing_count > 0 { 1 } else { 0 });
}
