//! Pipeline orchestration: ties together graph building, compat export,
//! quality reporting, and route planning.

use crate::autopilot::{self, CostMode, RouteConfig};
use crate::compat_export;
use crate::graph_export::{
    self, compute_graph_metrics, log_graph_metrics, write_graph_file, write_quality_report,
};
use crate::graph_schema::{GraphMetrics, QualityMeta, QualityReport};
use crate::json_export::MapData;

/// Aggregated CLI options passed into the pipeline.
#[derive(Debug, Clone)]
pub struct CliOptions {
    /// Path to an unpacked ETS2 installation directory, if any.
    pub ets2_dir: Option<String>,
    /// Path to a directory of pre-extracted HashFS sector files.
    pub hashfs_sectors: Option<String>,
    /// Path to an `scs_packer.exe` helper binary, if used.
    pub scs_packer: Option<String>,
    /// Write `graph.json` to disk after the build.
    pub write_graph: bool,
    /// Also write `compat_*.json` files for legacy tooling.
    pub compat_export: bool,
    /// Write a `quality_report.json` summarising graph metrics.
    pub quality_report: bool,
    /// Run a side-by-side comparison of graph- vs. roads-based routing.
    pub performance_compare: bool,
    /// Routing back-end selector ("self_route" by default).
    pub routing_mode: String,
    /// Bias the planner towards higher speed limits when costs are equal.
    pub prefer_speed: bool,
    /// Cost-mode key — currently `"distance"` or `"eta"`.
    pub cost_mode: String,
    /// Emit verbose progress output.
    pub verbose: bool,
    /// Optional start node UID for an immediate route plan.
    pub start_uid: Option<u64>,
    /// Optional goal node UID for an immediate route plan.
    pub goal_uid: Option<u64>,
}

impl Default for CliOptions {
    fn default() -> Self {
        Self {
            ets2_dir: None,
            hashfs_sectors: None,
            scs_packer: None,
            write_graph: true,
            compat_export: false,
            quality_report: false,
            performance_compare: false,
            routing_mode: "self_route".into(),
            prefer_speed: false,
            cost_mode: "distance".into(),
            verbose: false,
            start_uid: None,
            goal_uid: None,
        }
    }
}

/// Run the full pipeline with the given map data and options.
pub fn run_pipeline(map: &MapData, opts: &CliOptions) -> Result<(), String> {
    // Build graph with timing.
    let (graph, build_duration) = graph_export::build_graph_timed(map)?;

    // Compute and optionally log metrics.
    let metrics = compute_graph_metrics(&graph, build_duration);
    if opts.verbose {
        log_graph_metrics(&metrics, true);
    }

    // Write graph JSON.
    if opts.write_graph {
        write_graph_file("graph.json", &graph)?;
        if opts.verbose {
            println!(
                "Wrote graph.json ({} nodes, {} edges)",
                metrics.nodes_total, metrics.edges_total
            );
        }
    }

    // Compat export.
    if opts.compat_export {
        compat_export::write_compat_files("compat", map, &graph)?;
        if opts.verbose {
            println!("Wrote compat_*.json files");
        }
    }

    // Quality report.
    if opts.quality_report {
        // Zero build_time_ms for byte-stable output.
        let stable_metrics = GraphMetrics {
            build_time_ms: 0.0,
            ..metrics
        };
        let report = QualityReport {
            meta: QualityMeta {
                schema_version: "1.0.0".into(),
                map_name: opts
                    .ets2_dir
                    .as_deref()
                    .or(opts.hashfs_sectors.as_deref())
                    .unwrap_or("unknown")
                    .into(),
                generated_at: String::new(),
            },
            metrics: stable_metrics,
        };
        write_quality_report("quality_report.json", &report)?;
        if opts.verbose {
            println!("Wrote quality_report.json");
        }
    }

    // Route planning.
    if let (Some(start), Some(goal)) = (opts.start_uid, opts.goal_uid) {
        let cost_mode = match opts.cost_mode.as_str() {
            "eta" => CostMode::Eta,
            _ => CostMode::Distance,
        };
        let route_config = RouteConfig {
            prefer_speed: opts.prefer_speed,
            cost_mode,
        };

        let result = autopilot::plan_route(map, Some(&graph), start, goal, &route_config);

        match result {
            Some(route) => {
                println!(
                    "Route found: {} nodes, cost={:.2}, validated={}, time={:.2}ms",
                    route.path.len(),
                    route.total_cost,
                    route.validated,
                    route.planning_time_ms
                );
                if opts.verbose {
                    println!("  Path: {:?}", route.path);
                    println!("  Edges examined: {}", route.edges_examined);
                    println!("  Nodes expanded: {}", route.nodes_expanded);
                }
            }
            None => {
                println!("No route found from {} to {}", start, goal);
            }
        }

        // Performance comparison: graph vs roads routing.
        if opts.performance_compare {
            println!();
            println!("--- Performance Comparison ---");
            let graph_result = autopilot::plan_route_on_graph(&graph, start, goal, &route_config);
            let roads_result =
                autopilot::plan_route_on_roads(map, None, start, goal, &route_config);

            match (graph_result, roads_result) {
                (Some(g), Some(r)) => {
                    println!(
                        "Graph routing:  {:.3} ms (cost={:.2}, expanded={})",
                        g.planning_time_ms, g.total_cost, g.nodes_expanded
                    );
                    println!(
                        "Roads routing:  {:.3} ms (cost={:.2}, expanded={})",
                        r.planning_time_ms, r.total_cost, r.nodes_expanded
                    );
                    let speedup = r.planning_time_ms / g.planning_time_ms.max(0.001);
                    println!("Graph speedup:  {:.1}x", speedup);
                }
                (Some(g), None) => {
                    println!(
                        "Graph routing:  {:.3} ms (cost={:.2})",
                        g.planning_time_ms, g.total_cost
                    );
                    println!("Roads routing:  FAILED");
                }
                (None, Some(r)) => {
                    println!("Graph routing:  FAILED");
                    println!(
                        "Roads routing:  {:.3} ms (cost={:.2})",
                        r.planning_time_ms, r.total_cost
                    );
                }
                (None, None) => {
                    println!("Both routing methods failed.");
                }
            }
        }
    }

    Ok(())
}

/// Build a simple test fixture `MapData` for use when no real map is available.
pub fn build_test_map() -> MapData {
    use crate::json_export::{MapNode, MapRoad};

    MapData {
        nodes: vec![
            MapNode {
                uid: 1,
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            MapNode {
                uid: 2,
                x: 100.0,
                y: 0.0,
                z: 0.0,
            },
            MapNode {
                uid: 3,
                x: 200.0,
                y: 0.0,
                z: 0.0,
            },
            MapNode {
                uid: 4,
                x: 200.0,
                y: 0.0,
                z: 100.0,
            },
            MapNode {
                uid: 5,
                x: 100.0,
                y: 0.0,
                z: 100.0,
            },
        ],
        roads: vec![
            MapRoad {
                uid: "road1".into(),
                name: "Highway A".into(),
                look_token: "asphalt".into(),
                nodes: vec![1, 2, 3],
                speed_limit: Some(80.0),
                lane_count_forward: 2,
                lane_count_backward: 2,
            },
            MapRoad {
                uid: "road2".into(),
                name: "Cross St".into(),
                look_token: "asphalt".into(),
                nodes: vec![3, 4],
                speed_limit: Some(50.0),
                lane_count_forward: 1,
                lane_count_backward: 1,
            },
            MapRoad {
                uid: "road3".into(),
                name: "Back Rd".into(),
                look_token: "dirt".into(),
                nodes: vec![4, 5, 2],
                speed_limit: None,
                lane_count_forward: 0,
                lane_count_backward: 0,
            },
        ],
        prefabs: vec![],
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pipeline_full_flow() {
        let map = build_test_map();
        let opts = CliOptions {
            write_graph: true,
            compat_export: true,
            quality_report: true,
            verbose: false,
            start_uid: Some(1),
            goal_uid: Some(4),
            ..Default::default()
        };

        run_pipeline(&map, &opts).unwrap();

        // Cleanup generated files.
        for path in &[
            "graph.json",
            "compat_nodes.json",
            "compat_roads.json",
            "compat_road_looks.json",
            "compat_graph.json",
            "quality_report.json",
        ] {
            if std::path::Path::new(path).exists() {
                std::fs::remove_file(path).unwrap();
            }
        }
    }

    #[test]
    fn test_pipeline_no_route() {
        let map = build_test_map();
        let opts = CliOptions {
            write_graph: false,
            compat_export: false,
            quality_report: false,
            start_uid: Some(1),
            goal_uid: Some(99),
            ..Default::default()
        };

        run_pipeline(&map, &opts).unwrap();
    }

    #[test]
    fn test_cli_options_default() {
        let opts = CliOptions::default();
        assert!(opts.write_graph);
        assert!(!opts.compat_export);
        assert_eq!(opts.cost_mode, "distance");
    }
}
