//! Autopilot route planning on the road network graph.
//!
//! Provides A*-based routing on `GraphData` with configurable cost models
//! (distance vs. estimated time) and a fallback road-level planner.

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};

use crate::graph_schema::GraphData;
use crate::json_export::MapData;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Cost model for route planning.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CostMode {
    /// Minimize total distance in meters.
    Distance,
    /// Minimize estimated travel time.
    Eta,
}

/// Configuration for a single route-planning invocation.
#[derive(Debug, Clone)]
pub struct RouteConfig {
    /// Prefer higher speed limits when selecting edges.
    pub prefer_speed: bool,
    /// Cost model to optimize.
    pub cost_mode: CostMode,
}

impl Default for RouteConfig {
    fn default() -> Self {
        Self {
            prefer_speed: false,
            cost_mode: CostMode::Distance,
        }
    }
}

/// Result of a route-planning operation.
#[derive(Debug, Clone)]
pub struct RouteResult {
    /// Ordered list of node UIDs from start to goal.
    pub path: Vec<u64>,
    /// Total cost of the path.
    pub total_cost: f64,
    /// Number of edges examined during search.
    pub edges_examined: u64,
    /// Number of nodes expanded (popped from open set).
    pub nodes_expanded: u64,
    /// Whether the path was validated.
    pub validated: bool,
    /// Wall-clock planning time in milliseconds (filled in Phase 6).
    pub planning_time_ms: f64,
}

// ---------------------------------------------------------------------------
// A* internals
// ---------------------------------------------------------------------------

/// Maximum assumed speed in m/s for eta-mode heuristics (~90 km/h).
const MAX_SPEED_MS: f64 = 50.0;

/// Default effective speed when no speed limit is specified (~80 km/h).
const DEFAULT_SPEED_MS: f64 = 22.222;

/// State tracked for A*.
#[derive(Clone)]
struct HeapEntry {
    uid: u64,
    f_score: f64,
}

impl PartialEq for HeapEntry {
    fn eq(&self, other: &Self) -> bool {
        self.f_score.total_cmp(&other.f_score) == std::cmp::Ordering::Equal && self.uid == other.uid
    }
}

impl Eq for HeapEntry {}

impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for HeapEntry {
    /// Lower f_score = higher priority. Tie-break by UID for determinism.
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.f_score
            .total_cmp(&other.f_score)
            .then_with(|| self.uid.cmp(&other.uid))
    }
}

/// Euclidean distance between two nodes (2D horizontal only for heuristic).
fn euclidean_2d(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dx = a.0 - b.0;
    let dz = a.1 - b.1;
    (dx * dx + dz * dz).sqrt()
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Plan a route on the graph using A*.
///
/// Returns `None` if the goal is unreachable or start/goal do not exist.
pub fn plan_route_on_graph(
    graph: &GraphData,
    start_uid: u64,
    goal_uid: u64,
    config: &RouteConfig,
) -> Option<RouteResult> {
    let start_time = std::time::Instant::now();

    // Build adjacency list (sorted for determinism).
    let mut adj: HashMap<u64, Vec<&crate::graph_schema::GraphEdge>> = HashMap::new();
    for edge in &graph.edges {
        adj.entry(edge.from_node_uid).or_default().push(edge);
    }
    for list in adj.values_mut() {
        list.sort_by(|a, b| {
            a.to_node_uid
                .cmp(&b.to_node_uid)
                .then_with(|| a.edge_uid.cmp(&b.edge_uid))
        });
    }

    // Build position lookup.
    let positions: HashMap<u64, (f64, f64)> =
        graph.nodes.iter().map(|n| (n.uid, (n.x, n.z))).collect();

    let goal_pos = positions.get(&goal_uid)?;
    let start_pos = positions.get(&start_uid)?;

    let mut open_set: BinaryHeap<Reverse<HeapEntry>> = BinaryHeap::new();
    let mut g_scores: HashMap<u64, f64> = HashMap::new();
    let mut came_from: HashMap<u64, u64> = HashMap::new();
    let mut closed: HashSet<u64> = HashSet::new();
    let mut edges_examined: u64 = 0;
    let mut nodes_expanded: u64 = 0;

    let h_start = heuristic(start_pos, goal_pos, config.cost_mode);
    g_scores.insert(start_uid, 0.0);
    open_set.push(Reverse(HeapEntry {
        uid: start_uid,
        f_score: h_start,
    }));

    while let Some(Reverse(entry)) = open_set.pop() {
        let current = entry.uid;

        if current == goal_uid {
            let path = reconstruct_path(&came_from, start_uid, goal_uid);
            let validated = validate_path(&path, &adj);
            let elapsed = start_time.elapsed();

            return Some(RouteResult {
                total_cost: g_scores[&goal_uid],
                path,
                edges_examined,
                nodes_expanded,
                validated,
                planning_time_ms: elapsed.as_secs_f64() * 1000.0,
            });
        }

        if !closed.insert(current) {
            continue;
        }
        nodes_expanded += 1;

        if let Some(neighbors) = adj.get(&current) {
            for edge in neighbors {
                edges_examined += 1;
                let neighbor = edge.to_node_uid;

                if closed.contains(&neighbor) {
                    continue;
                }

                let edge_cost = edge_cost(edge, config);
                let tentative_g = g_scores.get(&current).copied().unwrap_or(f64::MAX) + edge_cost;

                if tentative_g < g_scores.get(&neighbor).copied().unwrap_or(f64::MAX) {
                    came_from.insert(neighbor, current);
                    g_scores.insert(neighbor, tentative_g);

                    let neighbor_pos = positions.get(&neighbor).copied()?;
                    let h = heuristic(&neighbor_pos, goal_pos, config.cost_mode);
                    let f = tentative_g + h;

                    open_set.push(Reverse(HeapEntry {
                        uid: neighbor,
                        f_score: f,
                    }));
                }
            }
        }
    }

    None
}

/// Plan a route using lane-subnode string identifiers (`<base>_lane_<idx>`).
///
/// Returns `None` if either lane UID is unknown or route is unreachable.
pub fn plan_route_on_graph_lane_uids(
    graph: &GraphData,
    start_lane_uid: &str,
    goal_lane_uid: &str,
    config: &RouteConfig,
) -> Option<RouteResult> {
    let lane_uid_to_num: HashMap<&str, u64> = graph
        .nodes
        .iter()
        .filter_map(|n| n.lane_uid.as_deref().map(|s| (s, n.uid)))
        .collect();
    let start_uid = *lane_uid_to_num.get(start_lane_uid)?;
    let goal_uid = *lane_uid_to_num.get(goal_lane_uid)?;
    plan_route_on_graph(graph, start_uid, goal_uid, config)
}

/// Fallback route planning using the road network (simplified).
///
/// This is a placeholder that delegates to graph-based routing if a graph is available.
/// In the original codebase this used road-level heuristics.
pub fn plan_route_on_roads(
    _map: &MapData,
    graph_opt: Option<&GraphData>,
    start_uid: u64,
    goal_uid: u64,
    config: &RouteConfig,
) -> Option<RouteResult> {
    match graph_opt {
        Some(graph) => plan_route_on_graph(graph, start_uid, goal_uid, config),
        None => None,
    }
}

/// Central route-planning dispatcher.
///
/// Prefers graph-based routing when a graph is provided, falls back to
/// road-level routing otherwise.
pub fn plan_route(
    map: &MapData,
    graph_opt: Option<&GraphData>,
    start_uid: u64,
    goal_uid: u64,
    config: &RouteConfig,
) -> Option<RouteResult> {
    match graph_opt {
        Some(graph) => plan_route_on_graph(graph, start_uid, goal_uid, config),
        None => plan_route_on_roads(map, None, start_uid, goal_uid, config),
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Compute the heuristic estimate from `pos` to `goal`.
fn heuristic(pos: &(f64, f64), goal: &(f64, f64), mode: CostMode) -> f64 {
    let dist = euclidean_2d(*pos, *goal);
    match mode {
        CostMode::Distance => dist,
        CostMode::Eta => dist / MAX_SPEED_MS,
    }
}

/// Compute the traversal cost of a single edge.
fn edge_cost(edge: &crate::graph_schema::GraphEdge, config: &RouteConfig) -> f64 {
    let speed = if config.prefer_speed {
        edge.speed_limit_kmh.unwrap_or(80.0) / 3.6
    } else {
        DEFAULT_SPEED_MS
    };

    let mut cost = match config.cost_mode {
        CostMode::Distance => edge.distance_m,
        CostMode::Eta => edge.distance_m / speed.max(1.0),
    };

    // Penalty for flagged edges (e.g. unknown direction).
    if edge.flags.iter().any(|f| f == "no_lanes_unknown") {
        cost *= 1.5;
    }

    // Penalty for lane-change edges — discourage unnecessary lane switching.
    if edge.direction == "lane_change" {
        cost *= 2.0;
    }

    cost
}

/// Reconstruct the path by walking backward through `came_from`.
fn reconstruct_path(came_from: &HashMap<u64, u64>, start: u64, goal: u64) -> Vec<u64> {
    let mut path = Vec::new();
    let mut current = goal;
    path.push(current);
    while current != start {
        if let Some(&prev) = came_from.get(&current) {
            path.push(prev);
            current = prev;
        } else {
            break;
        }
    }
    path.reverse();
    path
}

/// Verify that consecutive nodes in the path are connected by an edge.
fn validate_path(path: &[u64], adj: &HashMap<u64, Vec<&crate::graph_schema::GraphEdge>>) -> bool {
    for w in path.windows(2) {
        let from = w[0];
        let to = w[1];
        if let Some(neighbors) = adj.get(&from) {
            if !neighbors.iter().any(|e| e.to_node_uid == to) {
                return false;
            }
        } else {
            return false;
        }
    }
    true
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::json_export::{MapNode, MapRoad};

    /// Create a simple four-node chain: 1 → 2 → 3 → 4, all bidirectional.
    fn build_chain_graph() -> (MapData, GraphData) {
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
                uid: "chain".into(),
                name: String::new(),
                look_token: "a".into(),
                nodes: vec![1, 2, 3, 4],
                speed_limit: Some(50.0),
                lane_count_forward: 1,
                lane_count_backward: 1,
            }],
            prefabs: vec![],
        };
        let graph = crate::graph_export::build_graph(&map).unwrap();
        (map, graph)
    }

    #[test]
    fn test_simple_route() {
        let (_map, graph) = build_chain_graph();
        let config = RouteConfig::default();
        let result = plan_route_on_graph(&graph, 1, 4, &config).unwrap();

        assert_eq!(result.path, vec![1, 2, 3, 4]);
        assert!(result.total_cost > 0.0);
        assert!(result.validated);
        assert!(result.edges_examined > 0);
        assert!(result.nodes_expanded > 0);
    }

    #[test]
    fn test_unreachable_goal() {
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
                    x: 100.0,
                    y: 0.0,
                    z: 0.0,
                },
            ],
            roads: vec![], // no connections
            prefabs: vec![],
        };
        let graph = crate::graph_export::build_graph(&map).unwrap();
        let config = RouteConfig::default();
        let result = plan_route_on_graph(&graph, 1, 2, &config);
        assert!(result.is_none());
    }

    #[test]
    fn test_eta_mode() {
        let (_map, graph) = build_chain_graph();
        let config = RouteConfig {
            cost_mode: CostMode::Eta,
            ..Default::default()
        };
        let result = plan_route_on_graph(&graph, 1, 4, &config).unwrap();
        // ETA cost should be smaller than distance cost for the same path.
        let dist_config = RouteConfig {
            cost_mode: CostMode::Distance,
            ..Default::default()
        };
        let dist_result = plan_route_on_graph(&graph, 1, 4, &dist_config).unwrap();
        assert!(result.total_cost < dist_result.total_cost);
        assert_eq!(result.path, dist_result.path);
    }

    #[test]
    fn test_determinism() {
        let (_map, graph) = build_chain_graph();
        let config = RouteConfig::default();
        let r1 = plan_route_on_graph(&graph, 1, 4, &config).unwrap();
        let r2 = plan_route_on_graph(&graph, 1, 4, &config).unwrap();
        assert_eq!(r1.path, r2.path);
        assert!((r1.total_cost - r2.total_cost).abs() < 1e-9);
        assert_eq!(r1.edges_examined, r2.edges_examined);
        assert_eq!(r1.nodes_expanded, r2.nodes_expanded);
    }

    #[test]
    fn test_plan_route_dispatcher_with_graph() {
        let (map, graph) = build_chain_graph();
        let config = RouteConfig::default();
        let result = plan_route(&map, Some(&graph), 1, 4, &config).unwrap();
        assert_eq!(result.path, vec![1, 2, 3, 4]);
    }

    #[test]
    fn test_plan_route_dispatcher_without_graph() {
        let map = MapData {
            nodes: vec![MapNode {
                uid: 1,
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }],
            roads: vec![],
            prefabs: vec![],
        };
        let config = RouteConfig::default();
        // Without graph, roads planner returns None for missing graph
        let result = plan_route(&map, None, 1, 2, &config);
        assert!(result.is_none());
    }

    #[test]
    fn test_edge_cost_penalty() {
        use crate::graph_schema::GraphEdge;

        let normal_edge = GraphEdge {
            edge_uid: 1,
            from_node_uid: 1,
            to_node_uid: 2,
            from_lane_uid: None,
            to_lane_uid: None,
            road_uid: None,
            distance_m: 100.0,
            direction: "forward".into(),
            lane_count: 1,
            speed_limit_kmh: Some(80.0),
            flags: vec![],
        };
        let unknown_edge = GraphEdge {
            edge_uid: 2,
            from_node_uid: 1,
            to_node_uid: 2,
            from_lane_uid: None,
            to_lane_uid: None,
            road_uid: None,
            distance_m: 100.0,
            direction: "bidirectional_unknown".into(),
            lane_count: 1,
            speed_limit_kmh: Some(80.0),
            flags: vec!["no_lanes_unknown".into()],
        };

        let config = RouteConfig::default();
        let normal_cost = edge_cost(&normal_edge, &config);
        let unknown_cost = edge_cost(&unknown_edge, &config);
        assert!(unknown_cost > normal_cost);
        assert!((unknown_cost - normal_cost * 1.5).abs() < 1e-9);
    }

    #[test]
    fn test_planning_time_measured() {
        let (_map, graph) = build_chain_graph();
        let config = RouteConfig::default();
        let result = plan_route_on_graph(&graph, 1, 4, &config).unwrap();
        assert!(result.planning_time_ms >= 0.0);
    }

    #[test]
    fn test_lane_routing() {
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
            ],
            roads: vec![
                MapRoad {
                    uid: "r1".into(),
                    name: String::new(),
                    look_token: "asphalt".into(),
                    nodes: vec![1, 2],
                    speed_limit: Some(80.0),
                    lane_count_forward: 1,
                    lane_count_backward: 0,
                },
                MapRoad {
                    uid: "r2".into(),
                    name: String::new(),
                    look_token: "asphalt".into(),
                    nodes: vec![2, 3],
                    speed_limit: Some(80.0),
                    lane_count_forward: 2,
                    lane_count_backward: 0,
                },
            ],
            prefabs: vec![],
        };

        let graph = crate::graph_export::build_graph(&map).unwrap();
        let config = RouteConfig::default();
        let route = plan_route_on_graph_lane_uids(&graph, "0x1_lane_0", "0x3_lane_1", &config)
            .expect("lane route should exist");

        let mut has_lane_change = false;
        for pair in route.path.windows(2) {
            if graph.edges.iter().any(|e| {
                e.from_node_uid == pair[0]
                    && e.to_node_uid == pair[1]
                    && e.direction == "lane_change"
            }) {
                has_lane_change = true;
                break;
            }
        }
        assert!(
            has_lane_change,
            "expected route with lane_change edge, path={:?}",
            route.path
        );
    }
}
