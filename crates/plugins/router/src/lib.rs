//! Router plugin — A* route planning on the MapGraph.
//!
//! On load, parses `graph.json` (path from blackboard `router.graph_path`
//! or `graph.json` in CWD) into flat node/edge lists. Every 50 ticks
//! (~1 Hz at 50 Hz daemon cadence) the router finds the nearest node to
//! the truck's current position and runs A* to `goal_uid`. The resulting
//! waypoint list is published as JSON to the blackboard for lane-keeper.

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::path::PathBuf;

use truckpilot_plugin_api::{ControlOutput, Plugin, PluginContext, Telemetry};

const REPLAN_INTERVAL_TICKS: u64 = 50;
const DEFAULT_GRAPH_PATH: &str = "graph.json";

#[derive(Clone)]
struct HeapEntry {
    uid: u64,
    f: f64,
}
impl PartialEq for HeapEntry {
    fn eq(&self, o: &Self) -> bool { self.f.total_cmp(&o.f).is_eq() && self.uid == o.uid }
}
impl Eq for HeapEntry {}
impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> { Some(self.cmp(o)) }
}
impl Ord for HeapEntry {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        self.f.total_cmp(&o.f).then_with(|| self.uid.cmp(&o.uid))
    }
}

#[derive(serde::Deserialize)]
struct GraphFile {
    nodes: Vec<NodeJson>,
    edges: Vec<EdgeJson>,
}
#[derive(serde::Deserialize)]
struct NodeJson { uid: u64, x: f64, z: f64 }
#[derive(serde::Deserialize)]
struct EdgeJson { from: u64, to: u64, distance_m: f64 }

#[derive(Default)]
pub struct RouterPlugin {
    start_uid: u64,
    goal_uid: u64,
    graph_path: PathBuf,
    nodes: Vec<(u64, f64, f64)>,
    edges: Vec<(u64, u64, f64)>,
    positions: HashMap<u64, (f64, f64)>,
    active: bool,
    tick_count: u64,
}

impl RouterPlugin {
    fn plan(
        nodes: &[(u64, f64, f64)],
        edges: &[(u64, u64, f64)],
        start: u64,
        goal: u64,
    ) -> Option<Vec<u64>> {
        let positions: HashMap<u64, (f64, f64)> =
            nodes.iter().map(|&(uid, x, z)| (uid, (x, z))).collect();
        let mut adj: HashMap<u64, Vec<(u64, f64)>> = HashMap::new();
        for &(from, to, dist) in edges {
            adj.entry(from).or_default().push((to, dist));
        }
        let goal_pos = *positions.get(&goal)?;
        let mut open: BinaryHeap<Reverse<HeapEntry>> = BinaryHeap::new();
        let mut g: HashMap<u64, f64> = HashMap::new();
        let mut came_from: HashMap<u64, u64> = HashMap::new();
        let mut closed: HashSet<u64> = HashSet::new();
        g.insert(start, 0.0);
        open.push(Reverse(HeapEntry { uid: start, f: heuristic(positions.get(&start)?, &goal_pos) }));
        while let Some(Reverse(entry)) = open.pop() {
            if entry.uid == goal { return Some(reconstruct(&came_from, start, goal)); }
            if !closed.insert(entry.uid) { continue; }
            for &(nb, cost) in adj.get(&entry.uid).into_iter().flatten() {
                if closed.contains(&nb) { continue; }
                let tg = g[&entry.uid] + cost;
                if tg < *g.get(&nb).unwrap_or(&f64::MAX) {
                    came_from.insert(nb, entry.uid);
                    g.insert(nb, tg);
                    let h = heuristic(positions.get(&nb)?, &goal_pos);
                    open.push(Reverse(HeapEntry { uid: nb, f: tg + h }));
                }
            }
        }
        None
    }

    fn find_nearest(&self, x: f64, z: f64) -> Option<u64> {
        self.nodes
            .iter()
            .map(|&(uid, nx, nz)| { let dx = nx - x; let dz = nz - z; (uid, dx * dx + dz * dz) })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(uid, _)| uid)
    }
}

fn heuristic(pos: &(f64, f64), goal: &(f64, f64)) -> f64 {
    let dx = pos.0 - goal.0;
    let dz = pos.1 - goal.1;
    (dx * dx + dz * dz).sqrt()
}

fn reconstruct(came_from: &HashMap<u64, u64>, start: u64, goal: u64) -> Vec<u64> {
    let mut path = vec![goal];
    let mut cur = goal;
    while cur != start {
        if let Some(&prev) = came_from.get(&cur) { path.push(prev); cur = prev; } else { break; }
    }
    path.reverse();
    path
}

impl Plugin for RouterPlugin {
    fn name(&self) -> &str { "router" }
    fn version(&self) -> &str { "0.2.0" }
    fn settings_schema(&self) -> &str {
        r#"{"type":"object","properties":{"start_uid":{"type":"integer"},"goal_uid":{"type":"integer"},"graph_path":{"type":"string"}}}"#
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        if let Some(s) = ctx.blackboard.get_f64("router.start_uid") { self.start_uid = s as u64; }
        if let Some(g) = ctx.blackboard.get_f64("router.goal_uid") { self.goal_uid = g as u64; }
        self.graph_path = ctx.blackboard.get("router.graph_path")
            .map(PathBuf::from).unwrap_or_else(|| PathBuf::from(DEFAULT_GRAPH_PATH));

        match std::fs::read_to_string(&self.graph_path) {
            Ok(data) => match serde_json::from_str::<GraphFile>(&data) {
                Ok(g) => {
                    self.nodes = g.nodes.iter().map(|n| (n.uid, n.x, n.z)).collect();
                    self.edges = g.edges.iter().map(|e| (e.from, e.to, e.distance_m)).collect();
                    self.positions = g.nodes.iter().map(|n| (n.uid, (n.x, n.z))).collect();
                    tracing::info!("[router] loaded {} nodes / {} edges from {:?}",
                        self.nodes.len(), self.edges.len(), self.graph_path);
                }
                Err(e) => tracing::warn!("[router] cannot parse {:?}: {e}", self.graph_path),
            },
            Err(e) => tracing::warn!("[router] cannot read {:?}: {e}", self.graph_path),
        }
        ctx.blackboard.set("router.active", "false");
    }

    fn on_unload(&mut self) { tracing::info!("[router] unloaded"); }

    fn tick(&mut self, telemetry: Option<&Telemetry>, _output: &mut ControlOutput, ctx: &PluginContext) {
        self.tick_count = self.tick_count.wrapping_add(1);

        if self.tick_count.is_multiple_of(REPLAN_INTERVAL_TICKS) && self.goal_uid != 0 && !self.nodes.is_empty() {
            let (px, pz) = telemetry
                .map(|t| (t.position[0], t.position[2]))
                .unwrap_or((0.0, 0.0));
            self.active = if let Some(nearest) = self.find_nearest(px, pz) {
                if let Some(path) = Self::plan(&self.nodes, &self.edges, nearest, self.goal_uid) {
                    let waypoints: Vec<[f64; 2]> = path.iter()
                        .filter_map(|uid| self.positions.get(uid).copied().map(|(x, z)| [x, z]))
                        .collect();
                    match serde_json::to_string(&waypoints) {
                        Ok(json) => { ctx.blackboard.set("router.waypoints", &json); true }
                        Err(_) => false,
                    }
                } else { false }
            } else { false };
        }

        ctx.blackboard.set("router.active", if self.active { "true" } else { "false" });
    }
}

truckpilot_plugin_api::export_plugin!(RouterPlugin);

#[cfg(test)]
mod tests {
    use super::*;
    use truckpilot_plugin_api::SharedBlackboard;

    type NodeList = Vec<(u64, f64, f64)>;
    type EdgeList = Vec<(u64, u64, f64)>;

    fn simple_graph() -> (NodeList, EdgeList) {
        (vec![(1, 0.0, 0.0), (2, 100.0, 0.0), (3, 200.0, 0.0)],
         vec![(1, 2, 100.0), (2, 3, 100.0)])
    }

    #[test]
    fn finds_direct_route() {
        let (n, e) = simple_graph();
        assert_eq!(RouterPlugin::plan(&n, &e, 1, 3).unwrap(), vec![1, 2, 3]);
    }

    #[test]
    fn unreachable_returns_none() {
        let n: NodeList = vec![(1, 0.0, 0.0), (2, 100.0, 0.0)];
        let e: EdgeList = vec![];
        assert!(RouterPlugin::plan(&n, &e, 1, 2).is_none());
    }

    #[test]
    fn start_equals_goal() {
        let (n, e) = simple_graph();
        assert_eq!(RouterPlugin::plan(&n, &e, 1, 1).unwrap(), vec![1]);
    }

    #[test]
    fn tick_produces_waypoints() {
        let (n, e) = simple_graph();
        let mut p = RouterPlugin {
            goal_uid: 3,
            nodes: n.clone(),
            edges: e,
            positions: n.iter().map(|&(u, x, z)| (u, (x, z))).collect(),
            tick_count: REPLAN_INTERVAL_TICKS - 1,
            ..Default::default()
        };
        let bb = SharedBlackboard::new();
        let ctx = PluginContext::new("test", bb.clone());
        let mut out = ControlOutput::default();
        let t = Telemetry {
            position: [0.0, 0.0, 0.0], heading: 0.0, pitch: 0.0, roll: 0.0,
            speed_ms: 0.0, engine_rpm: 0.0, cruise_control_kmh: 0.0,
            nav_speed_limit_kmh: -1.0, lead_vehicle_distance_m: -1.0,
            accel_longitudinal: -1.0, fuel_liters: -1.0, odometer_km: -1.0,
        };
        p.tick(Some(&t), &mut out, &ctx);
        assert_eq!(bb.get("router.active").as_deref(), Some("true"));
        let wp = bb.get("router.waypoints").expect("waypoints set");
        assert!(wp.contains("200"));
    }
}
