//! End-to-end integration tests for the TruckPilot pipeline.
//!
//! Tests the full flow: MapData → GraphData → Compat Export → Route Planning.

use truckpilot::autopilot::{self, CostMode, RouteConfig};
use truckpilot::compat_export;
use truckpilot::graph_export::{self, compute_graph_metrics};
use truckpilot::json_export::{MapData, MapNode, MapPrefab, MapRoad};
use truckpilot::pipeline::{self, CliOptions};

fn build_test_map() -> MapData {
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
                x: 300.0,
                y: 0.0,
                z: 0.0,
            },
        ],
        roads: vec![MapRoad {
            uid: "r1".into(),
            name: "Main Rd".into(),
            look_token: "asphalt".into(),
            nodes: vec![1, 2, 3, 4],
            speed_limit: Some(80.0),
            lane_count_forward: 2,
            lane_count_backward: 2,
        }],
        prefabs: vec![],
    }
}

#[test]
fn test_full_pipeline_graph_build() {
    let map = build_test_map();
    let (graph, duration) = graph_export::build_graph_timed(&map).unwrap();

    assert_eq!(graph.nodes.len(), 8);
    assert!(graph.edges.len() >= 12, "got {} edges", graph.edges.len());
    assert!(duration.as_nanos() > 0);

    // Verify base lane_0 nodes are still present.
    let base_lane0: std::collections::HashSet<u64> = graph
        .nodes
        .iter()
        .filter_map(|n| (n.lane_index == Some(0)).then_some(n.uid))
        .collect();
    assert!(base_lane0.contains(&1));
    assert!(base_lane0.contains(&2));
    assert!(base_lane0.contains(&3));
    assert!(base_lane0.contains(&4));
}

#[test]
fn test_full_pipeline_compat_export() {
    let map = build_test_map();
    let graph = graph_export::build_graph(&map).unwrap();
    let prefix = "test_compat_integration";

    compat_export::write_compat_files(prefix, &map, &graph).unwrap();

    for suffix in &["nodes", "roads", "road_looks", "graph"] {
        let path = format!("{prefix}_{suffix}.json");
        assert!(std::path::Path::new(&path).exists());
        let content = std::fs::read_to_string(&path).unwrap();
        assert!(!content.is_empty());
        std::fs::remove_file(&path).unwrap();
    }
}

#[test]
fn test_full_pipeline_routing() {
    let map = build_test_map();
    let graph = graph_export::build_graph(&map).unwrap();
    let config = RouteConfig {
        cost_mode: CostMode::Distance,
        ..Default::default()
    };

    // Route from node 1 to node 4.
    let result = autopilot::plan_route_on_graph(&graph, 1, 4, &config).unwrap();
    assert_eq!(result.path, vec![1, 2, 3, 4]);
    assert!(result.validated);
    assert!(result.total_cost > 0.0);

    // Route from 4 to 1 (should work since bidirectional).
    let rev = autopilot::plan_route_on_graph(&graph, 4, 1, &config).unwrap();
    assert_eq!(rev.path, vec![4, 3, 2, 1]);
}

#[test]
fn test_full_pipeline_determinism() {
    let map = build_test_map();
    let graph1 = graph_export::build_graph(&map).unwrap();
    let graph2 = graph_export::build_graph(&map).unwrap();

    assert_eq!(graph1.nodes, graph2.nodes);
    assert_eq!(graph1.edges, graph2.edges);

    let metrics1 = compute_graph_metrics(&graph1, std::time::Duration::ZERO);
    let metrics2 = compute_graph_metrics(&graph2, std::time::Duration::ZERO);
    assert_eq!(metrics1.nodes_total, metrics2.nodes_total);
    assert_eq!(metrics1.edges_total, metrics2.edges_total);
}

#[test]
fn test_full_pipeline_with_prefabs() {
    let map = MapData {
        nodes: vec![
            MapNode {
                uid: 1,
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            MapNode {
                uid: 2,
                x: 10.0,
                y: 0.0,
                z: 0.0,
            },
            MapNode {
                uid: 3,
                x: 20.0,
                y: 0.0,
                z: 0.0,
            },
        ],
        roads: vec![MapRoad {
            uid: "r1".into(),
            name: "".into(),
            look_token: "a".into(),
            nodes: vec![1, 2],
            speed_limit: None,
            lane_count_forward: 1,
            lane_count_backward: 0,
        }],
        prefabs: vec![MapPrefab {
            uid: "p1".into(),
            nodes: vec![1, 3],
        }],
    };

    let graph = graph_export::build_graph(&map).unwrap();
    // Forward edge: 1→2, prefab interconnect: 1→3 and 3→1
    assert_eq!(graph.edges.len(), 3);

    // Route from 2 to 3 should work via 2→1 (backward? no, 1→2 is forward only)
    // Actually 2→1 has no edge since it's forward-only.
    // So route 2→3 is unreachable.
    let config = RouteConfig::default();
    let result = autopilot::plan_route_on_graph(&graph, 2, 3, &config);
    assert!(result.is_none());

    // But 1→3 should work via prefab.
    let result = autopilot::plan_route_on_graph(&graph, 1, 3, &config);
    assert!(result.is_some());
}

#[test]
fn test_full_pipeline_cli_run() {
    let map = pipeline::build_test_map();
    let opts = CliOptions {
        write_graph: true,
        compat_export: true,
        quality_report: true,
        verbose: false,
        start_uid: Some(1),
        goal_uid: Some(5),
        ..Default::default()
    };

    let result = pipeline::run_pipeline(&map, &opts);
    assert!(result.is_ok());

    // Cleanup.
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

/// Large-scale test simulating a realistic ETS2 sector with ~50 nodes,
/// multiple roads, varied lane configurations, and prefab intersections.
#[test]
fn test_realistic_medium_map() {
    let mut nodes = Vec::new();
    let mut roads = Vec::new();
    let mut prefabs = Vec::new();

    // Generate a grid of nodes: 0..6 in X, 0..6 in Z (49 nodes).
    for ix in 0..7u64 {
        for iz in 0..7u64 {
            let uid = ix * 100 + iz;
            nodes.push(MapNode {
                uid,
                x: ix as f64 * 100.0,
                y: 0.0,
                z: iz as f64 * 100.0,
            });
        }
    }

    // Create horizontal roads (east-west) with bidirectional lanes.
    for iz in 0..7 {
        let mut road_nodes = Vec::new();
        for ix in 0..7 {
            road_nodes.push(ix * 100 + iz);
        }
        roads.push(MapRoad {
            uid: format!("h{}", iz),
            name: format!("H-{}", iz),
            look_token: "asphalt".into(),
            nodes: road_nodes,
            speed_limit: Some(if iz == 3 { 80.0 } else { 50.0 }),
            lane_count_forward: 2,
            lane_count_backward: 2,
        });
    }

    // Create vertical roads (north-south), some one-way.
    for ix in 0..7 {
        let mut road_nodes = Vec::new();
        for iz in 0..7 {
            road_nodes.push(ix * 100 + iz);
        }
        roads.push(MapRoad {
            uid: format!("v{}", ix),
            name: format!("V-{}", ix),
            look_token: if ix % 2 == 0 {
                "asphalt".into()
            } else {
                "dirt".into()
            },
            nodes: road_nodes,
            speed_limit: None,
            lane_count_forward: 1,
            lane_count_backward: if ix == 0 { 0 } else { 1 },
        });
    }

    // Add a few prefabs connecting distant nodes.
    prefabs.push(MapPrefab {
        uid: "p_center".into(),
        nodes: vec![303, 304, 403, 404],
    });
    prefabs.push(MapPrefab {
        uid: "p_corner".into(),
        nodes: vec![0, 100, 600, 606],
    });

    let map = MapData {
        nodes,
        roads,
        prefabs,
    };

    // Build graph.
    let (graph, duration) = graph_export::build_graph_timed(&map).unwrap();
    assert_eq!(graph.nodes.len(), 98);
    assert!(
        graph.edges.len() > 50,
        "expected >50 edges, got {}",
        graph.edges.len()
    );

    // Metrics.
    let metrics = compute_graph_metrics(&graph, duration);
    assert_eq!(metrics.nodes_total, 98);
    assert!(
        metrics.largest_component_ratio > 0.9,
        "expected connected graph"
    );

    // Route: corner to corner (0,0) → (6,6) = uid 0 → uid 606.
    let config = RouteConfig {
        cost_mode: CostMode::Distance,
        ..Default::default()
    };
    let result = autopilot::plan_route_on_graph(&graph, 0, 606, &config);
    assert!(result.is_some(), "expected route from 0 to 606");
    let route = result.unwrap();
    assert!(route.path.len() >= 2);
    assert!(route.validated);
    assert!(route.edges_examined > 0);

    // Determinism: rebuild and compare.
    let graph2 = graph_export::build_graph(&map).unwrap();
    assert_eq!(graph.edges.len(), graph2.edges.len());

    // Compat export should succeed.
    let prefix = "test_realistic_compat";
    compat_export::write_compat_files(prefix, &map, &graph).unwrap();
    for suffix in &["nodes", "roads", "road_looks", "graph"] {
        let path = format!("{prefix}_{suffix}.json");
        assert!(std::path::Path::new(&path).exists());
        std::fs::remove_file(&path).unwrap();
    }
}

/// End-to-end: telemetry_diag binary exits with code 1 and a readable message
/// when the shared-memory segment is not present.
#[test]
fn test_telemetry_diag_missing_shm() {
    // Make sure the Linux SHM path does not exist.
    let shm_path = "/dev/shm/truckpilot_telemetry";
    if std::path::Path::new(shm_path).exists() {
        std::fs::remove_file(shm_path).unwrap();
    }

    let exe_path = env!("CARGO_BIN_EXE_telemetry_diag");
    let output = std::process::Command::new(exe_path)
        .args(["--once"])
        .env_remove("TRUCKPILOT_SHM_PATH")
        .output()
        .expect("failed to execute telemetry_diag");

    let stderr = String::from_utf8_lossy(&output.stderr);
    let stdout = String::from_utf8_lossy(&output.stdout);

    assert!(
        !output.status.success(),
        "expected non-zero exit code, got stdout={stdout} stderr={stderr}"
    );
    assert_eq!(
        output.status.code(),
        Some(1),
        "expected exit code 1, got {:?}. stdout={stdout} stderr={stderr}",
        output.status.code()
    );
    assert!(
        stderr.to_lowercase().contains("shared memory")
            || stdout.to_lowercase().contains("shared memory"),
        "expected readable 'shared memory' message in output, got stdout={stdout} stderr={stderr}"
    );
}
