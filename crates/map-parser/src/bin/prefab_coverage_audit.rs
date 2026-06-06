//! `truckpilot-prefab-coverage-audit` — Coverage-Audit über alle PrefabInstanzen.
//!
//! Berechnet für jede PrefabInstance: wie viele ihrer Approaches (node_uids)
//! haben mindestens eine NavCurve (from_node_uid / to_node_uid in PrefabAiPaths)?
//!
//! Usage:
//!
//! ```powershell
//! # Ganzer Map
//! cargo run --release --bin truckpilot-prefab-coverage-audit -- --graph graph.json
//!
//! # Berlin-Region
//! cargo run --release --bin truckpilot-prefab-coverage-audit -- --graph graph.json \
//!   --region-x 9750 --region-z -10000 --region-radius 1000
//! ```

use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::PathBuf;
use truckpilot_map_parser::graph::MapGraph;

// ---------------------------------------------------------------------------
// Args
// ---------------------------------------------------------------------------

struct Args {
    graph: PathBuf,
    region: Option<RegionFilter>,
}

struct RegionFilter {
    cx: f32,
    cz: f32,
    radius: f32,
}

fn parse_args() -> Args {
    let mut graph = PathBuf::from("graph.json");
    let mut region_x: Option<f32> = None;
    let mut region_z: Option<f32> = None;
    let mut region_radius: Option<f32> = None;

    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--graph" => {
                graph = PathBuf::from(argv.get(i + 1).expect("--graph needs value"));
                i += 2;
            }
            "--region-x" => {
                region_x = Some(
                    argv.get(i + 1)
                        .expect("--region-x needs value")
                        .parse()
                        .expect("--region-x must be f32"),
                );
                i += 2;
            }
            "--region-z" => {
                region_z = Some(
                    argv.get(i + 1)
                        .expect("--region-z needs value")
                        .parse()
                        .expect("--region-z must be f32"),
                );
                i += 2;
            }
            "--region-radius" => {
                region_radius = Some(
                    argv.get(i + 1)
                        .expect("--region-radius needs value")
                        .parse()
                        .expect("--region-radius must be f32"),
                );
                i += 2;
            }
            "-h" | "--help" => {
                eprintln!(
                    "usage: truckpilot-prefab-coverage-audit \
                     [--graph <PATH>] \
                     [--region-x <f32> --region-z <f32> --region-radius <f32>]"
                );
                std::process::exit(0);
            }
            other => {
                eprintln!("unknown argument: {other}");
                std::process::exit(2);
            }
        }
    }

    let region = match (region_x, region_z, region_radius) {
        (Some(cx), Some(cz), Some(radius)) => Some(RegionFilter { cx, cz, radius }),
        (None, None, None) => None,
        _ => {
            eprintln!("--region-x, --region-z and --region-radius must all be given together");
            std::process::exit(2);
        }
    };

    Args { graph, region }
}

// ---------------------------------------------------------------------------
// Per-instance result
// ---------------------------------------------------------------------------

struct InstanceResult {
    token: u64,
    total_approaches: usize,
    covered_approaches: usize,
}

impl InstanceResult {
    fn coverage_ratio(&self) -> f32 {
        if self.total_approaches == 0 {
            1.0 // no approaches → vacuously fully covered
        } else {
            self.covered_approaches as f32 / self.total_approaches as f32
        }
    }

    fn bucket(&self) -> CoverageBucket {
        CoverageBucket::from_ratio(self.coverage_ratio())
    }
}

// ---------------------------------------------------------------------------
// Coverage buckets
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
enum CoverageBucket {
    Zero,    // 0%
    Low,     // 1–24%
    Quarter, // 25–49%
    Half,    // 50–74%
    High,    // 75–99%
    Full,    // 100%
}

impl CoverageBucket {
    fn from_ratio(r: f32) -> Self {
        if r <= 0.0 {
            Self::Zero
        } else if r < 0.25 {
            Self::Low
        } else if r < 0.50 {
            Self::Quarter
        } else if r < 0.75 {
            Self::Half
        } else if r < 1.0 {
            Self::High
        } else {
            Self::Full
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::Zero => "  0%        (no NavCurves)",
            Self::Low => "  1–24%     (minimal coverage)",
            Self::Quarter => " 25–49%    (quarter coverage)",
            Self::Half => " 50–74%    (half coverage)",
            Self::High => " 75–99%    (mostly covered)",
            Self::Full => "100%        (full coverage)",
        }
    }
}

// ---------------------------------------------------------------------------
// Token stats
// ---------------------------------------------------------------------------

struct TokenStats {
    token: u64,
    instance_count: usize,
    ratios: Vec<f32>,
}

impl TokenStats {
    fn avg(&self) -> f32 {
        if self.ratios.is_empty() {
            return 1.0;
        }
        self.ratios.iter().sum::<f32>() / self.ratios.len() as f32
    }

    fn min(&self) -> f32 {
        self.ratios.iter().cloned().fold(f32::INFINITY, f32::min)
    }

    fn max(&self) -> f32 {
        self.ratios
            .iter()
            .cloned()
            .fold(f32::NEG_INFINITY, f32::max)
    }

    /// True if the token consistently shows partial coverage (not all instances full).
    fn is_systematic_partial(&self) -> bool {
        // Systematic: avg < 0.95 AND max < 1.0 (never fully covered)
        self.avg() < 0.95 && self.max() < 1.0
    }
}

// ---------------------------------------------------------------------------
// Computation
// ---------------------------------------------------------------------------

fn compute(graph: &MapGraph, region: Option<&RegionFilter>) -> Vec<InstanceResult> {
    // All node_uids that appear in any PrefabAiPath (from OR to).
    let mut covered_nodes: HashSet<u64> = HashSet::with_capacity(graph.prefab_ai_paths.len() * 2);
    for path in &graph.prefab_ai_paths {
        covered_nodes.insert(path.from_node_uid);
        covered_nodes.insert(path.to_node_uid);
    }

    let mut results = Vec::with_capacity(graph.prefab_instances.len());

    for inst in &graph.prefab_instances {
        // Optional region filter on origin_pos
        if let Some(r) = region {
            let dx = inst.origin_pos[0] - r.cx;
            let dz = inst.origin_pos[2] - r.cz;
            if (dx * dx + dz * dz).sqrt() > r.radius {
                continue;
            }
        }

        let total = inst.node_uids.len();
        let covered = inst
            .node_uids
            .iter()
            .filter(|uid| covered_nodes.contains(uid))
            .count();

        results.push(InstanceResult {
            token: inst.token,
            total_approaches: total,
            covered_approaches: covered,
        });
    }

    results
}

// ---------------------------------------------------------------------------
// Report helpers
// ---------------------------------------------------------------------------

fn print_histogram(results: &[InstanceResult], label: &str) {
    let total = results.len();
    if total == 0 {
        println!("{label}: (keine Instanzen)");
        return;
    }

    let mut counts: BTreeMap<CoverageBucket, usize> = BTreeMap::new();
    for r in results {
        *counts.entry(r.bucket()).or_insert(0) += 1;
    }

    let weighted_sum: f32 = results.iter().map(|r| r.coverage_ratio()).sum::<f32>();
    let avg_ratio = weighted_sum / total as f32;

    let partial = results
        .iter()
        .filter(|r| r.coverage_ratio() < 1.0 && r.coverage_ratio() > 0.0)
        .count();
    let full = counts.get(&CoverageBucket::Full).copied().unwrap_or(0);
    let zero = counts.get(&CoverageBucket::Zero).copied().unwrap_or(0);

    println!("{label}");
    println!("  Instanzen total          : {total}");
    println!(
        "  Voll abgedeckt (100%)    : {full:>7}  ({:.1}%)",
        100.0 * full as f32 / total as f32
    );
    println!(
        "  Teil-Coverage (1-99%)    : {partial:>7}  ({:.1}%)",
        100.0 * partial as f32 / total as f32
    );
    println!(
        "  Keine NavCurves (0%)     : {zero:>7}  ({:.1}%)",
        100.0 * zero as f32 / total as f32
    );
    println!(
        "  Avg coverage_ratio       : {:.3}  ({:.1}%)",
        avg_ratio,
        avg_ratio * 100.0
    );
    println!();
    println!("  Histogramm:");

    // Print in descending order (Full first)
    for bucket in [
        CoverageBucket::Full,
        CoverageBucket::High,
        CoverageBucket::Half,
        CoverageBucket::Quarter,
        CoverageBucket::Low,
        CoverageBucket::Zero,
    ] {
        let n = counts.get(&bucket).copied().unwrap_or(0);
        let bar_len = (40 * n / total.max(1)).min(40);
        let bar: String = "#".repeat(bar_len);
        println!(
            "    {:28}  {:>6}  ({:5.1}%)  |{bar}",
            bucket.label(),
            n,
            100.0 * n as f32 / total as f32
        );
    }
    println!();
}

fn token_analysis(results: &[InstanceResult]) -> Vec<TokenStats> {
    let mut by_token: HashMap<u64, Vec<f32>> = HashMap::new();
    for r in results {
        by_token
            .entry(r.token)
            .or_default()
            .push(r.coverage_ratio());
    }

    let mut stats: Vec<TokenStats> = by_token
        .into_iter()
        .map(|(token, ratios)| {
            let instance_count = ratios.len();
            TokenStats {
                token,
                instance_count,
                ratios,
            }
        })
        .collect();

    // Sort by instance count descending
    stats.sort_by(|a, b| b.instance_count.cmp(&a.instance_count));
    stats
}

fn print_token_table(stats: &[TokenStats], n: usize, label: &str) {
    println!("{label} (Top {n} by Häufigkeit)");
    println!(
        "  {:>20}  {:>9}  {:>8}  {:>8}  {:>8}  {:>12}",
        "token", "instances", "avg%", "min%", "max%", "systematic?"
    );
    for ts in stats.iter().take(n) {
        let systematic = if ts.is_systematic_partial() {
            "PARTIAL"
        } else {
            "ok"
        };
        println!(
            "  {:>20}  {:>9}  {:>8.1}  {:>8.1}  {:>8.1}  {:>12}",
            ts.token,
            ts.instance_count,
            ts.avg() * 100.0,
            ts.min() * 100.0,
            ts.max() * 100.0,
            systematic
        );
    }
    println!();
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() {
    let args = parse_args();

    eprintln!("loading {} …", args.graph.display());
    let bytes = std::fs::read(&args.graph)
        .unwrap_or_else(|e| panic!("cannot read {}: {e}", args.graph.display()));
    let graph: MapGraph = serde_json::from_slice(&bytes)
        .unwrap_or_else(|e| panic!("cannot parse {}: {e}", args.graph.display()));
    eprintln!(
        "graph: {} prefab_instances  {} prefab_ai_paths",
        graph.prefab_instances.len(),
        graph.prefab_ai_paths.len(),
    );

    // -----------------------------------------------------------------------
    // TASK 1: Global (or region-filtered) histogram
    // -----------------------------------------------------------------------
    let region_label = if let Some(r) = &args.region {
        format!("Region ({:.0},{:.0}) ±{:.0}m", r.cx, r.cz, r.radius)
    } else {
        "Global (alle Instanzen)".to_string()
    };
    let global_results = compute(&graph, args.region.as_ref());
    println!("=== PREFAB COVERAGE AUDIT — {region_label} ===");
    println!();
    print_histogram(&global_results, &region_label);

    // -----------------------------------------------------------------------
    // TASK 2: Token-level analysis (global)
    // -----------------------------------------------------------------------
    let token_stats = token_analysis(&global_results);
    let systematic_tokens: Vec<&TokenStats> = token_stats
        .iter()
        .filter(|t| t.is_systematic_partial())
        .collect();

    println!("=== TOKEN-LEVEL ANALYSE (global) ===");
    println!();
    print_token_table(&token_stats, 30, "Top-30 Tokens");

    println!(
        "Tokens mit systematischer Teil-Coverage (avg<95% und max<100%): {}",
        systematic_tokens.len()
    );
    let sys_instances: usize = systematic_tokens.iter().map(|t| t.instance_count).sum();
    println!(
        "Davon betroffene Instanzen: {} ({:.1}% aller Instanzen)",
        sys_instances,
        100.0 * sys_instances as f32 / global_results.len().max(1) as f32
    );
    println!();

    // Check token 1960238657931 specifically
    let problem_token: u64 = 1960238657931;
    if let Some(ts) = token_stats.iter().find(|t| t.token == problem_token) {
        println!("--- Token 1960238657931 (bekanntes Problem-Junction) ---");
        println!("  Instanzen  : {}", ts.instance_count);
        println!("  avg coverage : {:.1}%", ts.avg() * 100.0);
        println!("  min coverage : {:.1}%", ts.min() * 100.0);
        println!("  max coverage : {:.1}%", ts.max() * 100.0);
        println!(
            "  Systematisch : {}",
            if ts.is_systematic_partial() {
                "JA (Template-Bug)"
            } else {
                "nein"
            }
        );
        println!();
    } else {
        println!("Token 1960238657931 nicht in globalen Ergebnissen gefunden.");
        println!();
    }

    // -----------------------------------------------------------------------
    // TASK 3: Berlin-Region
    // -----------------------------------------------------------------------
    // Berlin approx center and radius to cover x=9000-10500, z=-9500 to -10500
    let berlin_cx = 9750.0f32;
    let berlin_cz = -10000.0f32;
    let berlin_radius = 1000.0f32;

    let berlin_filter = RegionFilter {
        cx: berlin_cx,
        cz: berlin_cz,
        radius: berlin_radius,
    };
    let berlin_results = compute(&graph, Some(&berlin_filter));

    println!(
        "=== BERLIN-REGION ({:.0},{:.0}) ±{:.0}m ===",
        berlin_cx, berlin_cz, berlin_radius
    );
    println!();
    print_histogram(&berlin_results, "Berlin (gefiltert)");

    let berlin_token_stats = token_analysis(&berlin_results);
    print_token_table(&berlin_token_stats, 20, "Berlin Top-20 Tokens");

    // -----------------------------------------------------------------------
    // Extra: zero-node-uid instances (suspicious)
    // -----------------------------------------------------------------------
    let zero_node = global_results
        .iter()
        .filter(|r| r.total_approaches == 0)
        .count();
    if zero_node > 0 {
        println!(
            "HINWEIS: {} Instanzen mit 0 node_uids (keine Approaches registriert)",
            zero_node
        );
        println!();
    }

    // -----------------------------------------------------------------------
    // Summary verdict (to stderr for easy capture)
    // -----------------------------------------------------------------------
    let total = global_results.len();
    let full = global_results
        .iter()
        .filter(|r| r.coverage_ratio() >= 1.0)
        .count();
    let partial = global_results
        .iter()
        .filter(|r| r.coverage_ratio() > 0.0 && r.coverage_ratio() < 1.0)
        .count();
    let zero = global_results
        .iter()
        .filter(|r| r.coverage_ratio() <= 0.0)
        .count();
    let avg_global = global_results
        .iter()
        .map(|r| r.coverage_ratio())
        .sum::<f32>()
        / total.max(1) as f32;

    eprintln!();
    eprintln!("=== VERDICT ===");
    eprintln!("Total Instanzen : {total}");
    eprintln!(
        "Voll (100%)     : {full}  ({:.1}%)",
        100.0 * full as f32 / total as f32
    );
    eprintln!(
        "Teil (1-99%)    : {partial}  ({:.1}%)",
        100.0 * partial as f32 / total as f32
    );
    eprintln!(
        "Null (0%)       : {zero}  ({:.1}%)",
        100.0 * zero as f32 / total as f32
    );
    eprintln!("Avg ratio       : {:.1}%", avg_global * 100.0);
    eprintln!(
        "Systematische Teil-Coverage Tokens: {}",
        systematic_tokens.len()
    );
}
