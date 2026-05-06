//! Determinism tests: verify that two graph builds produce byte-identical output.

use truckpilot::graph_export;
use truckpilot::json_export::{MapData, MapNode, MapPrefab, MapRoad};

fn complex_map() -> MapData {
    MapData {
        nodes: vec![
            MapNode {
                uid: 10,
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            MapNode {
                uid: 20,
                x: 5.0,
                y: 0.0,
                z: 0.0,
            },
            MapNode {
                uid: 30,
                x: 10.0,
                y: 0.0,
                z: 0.0,
            },
            MapNode {
                uid: 40,
                x: 5.0,
                y: 0.0,
                z: 5.0,
            },
        ],
        roads: vec![
            MapRoad {
                uid: "rA".into(),
                name: "Alpha".into(),
                look_token: "asphalt".into(),
                nodes: vec![10, 20, 30],
                speed_limit: Some(50.0),
                lane_count_forward: 2,
                lane_count_backward: 1,
            },
            MapRoad {
                uid: "rB".into(),
                name: "Beta".into(),
                look_token: "dirt".into(),
                nodes: vec![20, 40],
                speed_limit: None,
                lane_count_forward: 0,
                lane_count_backward: 0,
            },
        ],
        prefabs: vec![MapPrefab {
            uid: "p1".into(),
            nodes: vec![10, 40, 30],
        }],
    }
}

#[test]
fn test_deterministic_graph_json() {
    let map = complex_map();
    let graph = graph_export::build_graph(&map).unwrap();

    // Serialize to JSON.
    let json1 = serde_json::to_string(&graph).unwrap();

    // Build again, serialize again.
    let graph2 = graph_export::build_graph(&map).unwrap();
    let json2 = serde_json::to_string(&graph2).unwrap();

    assert_eq!(
        json1, json2,
        "Graph JSON must be byte-identical across runs"
    );
}

#[test]
fn test_deterministic_edge_ordering() {
    let map = complex_map();
    let g1 = graph_export::build_graph(&map).unwrap();
    let g2 = graph_export::build_graph(&map).unwrap();

    for (a, b) in g1.edges.iter().zip(g2.edges.iter()) {
        assert_eq!(a.edge_uid, b.edge_uid);
        assert_eq!(a.from_node_uid, b.from_node_uid);
        assert_eq!(a.to_node_uid, b.to_node_uid);
        assert_eq!(a.direction, b.direction);
        assert_eq!(a.road_uid, b.road_uid);
        assert!((a.distance_m - b.distance_m).abs() < 1e-9);
    }
}

#[test]
fn test_deterministic_routing() {
    use truckpilot::autopilot::{self, CostMode, RouteConfig};

    let map = complex_map();
    let graph = graph_export::build_graph(&map).unwrap();
    let config = RouteConfig {
        cost_mode: CostMode::Distance,
        ..Default::default()
    };

    let r1 = autopilot::plan_route_on_graph(&graph, 10, 30, &config).unwrap();
    let r2 = autopilot::plan_route_on_graph(&graph, 10, 30, &config).unwrap();

    assert_eq!(r1.path, r2.path);
    assert!((r1.total_cost - r2.total_cost).abs() < 1e-9);
    assert_eq!(r1.edges_examined, r2.edges_examined);
    assert_eq!(r1.nodes_expanded, r2.nodes_expanded);
}

#[test]
fn test_deterministic_compat_json() {
    use truckpilot::compat_export;

    let map = complex_map();
    let graph = graph_export::build_graph(&map).unwrap();

    let nodes1 = compat_export::build_compat_nodes(&map);
    let nodes2 = compat_export::build_compat_nodes(&map);
    let json1 = serde_json::to_string(&nodes1).unwrap();
    let json2 = serde_json::to_string(&nodes2).unwrap();
    assert_eq!(json1, json2);

    let roads1 = compat_export::build_compat_roads(&map);
    let roads2 = compat_export::build_compat_roads(&map);
    let json1 = serde_json::to_string(&roads1).unwrap();
    let json2 = serde_json::to_string(&roads2).unwrap();
    assert_eq!(json1, json2);

    let looks1 = compat_export::build_compat_road_looks(&map);
    let looks2 = compat_export::build_compat_road_looks(&map);
    let json1 = serde_json::to_string(&looks1).unwrap();
    let json2 = serde_json::to_string(&looks2).unwrap();
    assert_eq!(json1, json2);

    let cg1 = compat_export::build_compat_graph(&graph);
    let cg2 = compat_export::build_compat_graph(&graph);
    let json1 = serde_json::to_string(&cg1).unwrap();
    let json2 = serde_json::to_string(&cg2).unwrap();
    assert_eq!(json1, json2);
}

#[test]
fn test_lane_change_edge_uids_deterministic() {
    let map = complex_map();
    let g1 = graph_export::build_graph(&map).unwrap();
    let g2 = graph_export::build_graph(&map).unwrap();

    let lc1: Vec<_> = g1
        .edges
        .iter()
        .filter(|e| e.direction == "lane_change")
        .map(|e| e.edge_uid)
        .collect();
    let lc2: Vec<_> = g2
        .edges
        .iter()
        .filter(|e| e.direction == "lane_change")
        .map(|e| e.edge_uid)
        .collect();

    assert!(!lc1.is_empty(), "Expected lane_change edges in graph");
    assert_eq!(
        lc1, lc2,
        "lane_change edge_uids must be deterministic across builds"
    );
}

#[test]
fn test_lane_change_edge_uids_unique_within_graph() {
    let map = complex_map();
    let g1 = graph_export::build_graph(&map).unwrap();
    let g2 = graph_export::build_graph(&map).unwrap();

    let lc1: Vec<u64> = g1
        .edges
        .iter()
        .filter(|e| e.direction == "lane_change")
        .map(|e| e.edge_uid)
        .collect();
    let lc2: Vec<u64> = g2
        .edges
        .iter()
        .filter(|e| e.direction == "lane_change")
        .map(|e| e.edge_uid)
        .collect();

    assert!(!lc1.is_empty(), "Expected lane_change edges in graph");

    // No duplicates within each graph build
    let mut seen1 = std::collections::HashSet::new();
    for uid in &lc1 {
        assert!(
            seen1.insert(*uid),
            "Duplicate lane_change edge_uid {} in first graph",
            uid
        );
    }
    let mut seen2 = std::collections::HashSet::new();
    for uid in &lc2 {
        assert!(
            seen2.insert(*uid),
            "Duplicate lane_change edge_uid {} in second graph",
            uid
        );
    }

    // Identical lists between independent builds
    assert_eq!(
        lc1, lc2,
        "lane_change edge_uids must be identical across builds"
    );
}

#[test]
fn test_road_lane_edge_uids_unique() {
    let map = complex_map();
    let graph = graph_export::build_graph(&map).unwrap();

    // Collect all normal road lane edges (forward / backward)
    let road_uids: Vec<u64> = graph
        .edges
        .iter()
        .filter(|e| e.direction == "forward" || e.direction == "backward")
        .map(|e| e.edge_uid)
        .collect();

    let mut seen = std::collections::HashSet::new();
    for uid in &road_uids {
        assert!(
            seen.insert(*uid),
            "Duplicate road lane edge_uid {} — different lanes must produce different UIDs",
            uid
        );
    }
}

#[test]
fn test_all_edge_uids_unique() {
    let map = complex_map();
    let graph = graph_export::build_graph(&map).unwrap();

    let mut seen = std::collections::HashSet::new();
    for edge in &graph.edges {
        assert!(
            seen.insert(edge.edge_uid),
            "Duplicate edge_uid {} across the whole graph (direction={}, road_uid={:?})",
            edge.edge_uid,
            edge.direction,
            edge.road_uid
        );
    }
}

#[test]
fn test_graph_sort_key_stable_json() {
    let map = complex_map();
    let g1 = graph_export::build_graph(&map).unwrap();
    let g2 = graph_export::build_graph(&map).unwrap();

    let json1 = serde_json::to_string_pretty(&g1).unwrap();
    let json2 = serde_json::to_string_pretty(&g2).unwrap();

    assert_eq!(
        json1, json2,
        "Graph JSON must be byte-identical across runs (stable sort key)"
    );
}
