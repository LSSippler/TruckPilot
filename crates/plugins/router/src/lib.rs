//! Router plugin — A* route planning on the MapGraph.
//!
//! Reads start/goal from config, plans a route once on load, then publishes
//! waypoints to the blackboard for lane-keeper to follow.
//!
//! Blackboard writes:
//!   `router.active`    = "true" / "false"
//!   `router.waypoints` = JSON array of [x, z] pairs

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};

use truckpilot_plugin_api::{ControlOutput, Plugin, PluginContext, Telemetry};

// ---------------------------------------------------------------------------
// A* types
// ---------------------------------------------------------------------------

#[allow(dead_code)]
#[derive(Clone)]
struct HeapEntry {
    uid: u64,
    f: f64,
}

impl PartialEq for HeapEntry {
    fn eq(&self, o: &Self) -> bool {
        self.f.total_cmp(&o.f).is_eq() && self.uid == o.uid
    }
}
impl Eq for HeapEntry {}
impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for HeapEntry {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        self.f.total_cmp(&o.f).then_with(|| self.uid.cmp(&o.uid))
    }
}

// ---------------------------------------------------------------------------
// Plugin
// ---------------------------------------------------------------------------

#[allow(dead_code)]
#[derive(Default)]
pub struct RouterPlugin {
    /// Start node UID (set via settings).
    start_uid: u64,
    /// Goal node UID (set via settings).
    goal_uid: u64,
    /// Planned waypoints as (x, z) pairs.
    waypoints: Vec<[f64; 2]>,
    /// Whether a valid route is active.
    active: bool,
}

impl RouterPlugin {
    /// Plan a route using A* on a flat node/edge list.
    /// Nodes: Vec<(uid, x, z)>, Edges: Vec<(from, to, dist)>
    #[allow(dead_code)]
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
        open.push(Reverse(HeapEntry {
            uid: start,
            f: heuristic(positions.get(&start)?, &goal_pos),
        }));

        while let Some(Reverse(entry)) = open.pop() {
            if entry.uid == goal {
                return Some(reconstruct(&came_from, start, goal));
            }
            if !closed.insert(entry.uid) {
                continue;
            }

            for &(nb, cost) in adj.get(&entry.uid).into_iter().flatten() {
                if closed.contains(&nb) {
                    continue;
                }
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
}

#[allow(dead_code)]
fn heuristic(pos: &(f64, f64), goal: &(f64, f64)) -> f64 {
    let dx = pos.0 - goal.0;
    let dz = pos.1 - goal.1;
    (dx * dx + dz * dz).sqrt()
}

#[allow(dead_code)]
fn reconstruct(came_from: &HashMap<u64, u64>, start: u64, goal: u64) -> Vec<u64> {
    let mut path = vec![goal];
    let mut cur = goal;
    while cur != start {
        if let Some(&prev) = came_from.get(&cur) {
            path.push(prev);
            cur = prev;
        } else {
            break;
        }
    }
    path.reverse();
    path
}

impl Plugin for RouterPlugin {
    fn name(&self) -> &str {
        "router"
    }
    fn version(&self) -> &str {
        "0.1.0"
    }
    fn settings_schema(&self) -> &str {
        r#"{"type":"object","properties":{"start_uid":{"type":"integer"},"goal_uid":{"type":"integer"}}}"#
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        // Read start/goal from blackboard (set by UI or CLI before load)
        if let Some(s) = ctx.blackboard.get_f64("router.start_uid") {
            self.start_uid = s as u64;
        }
        if let Some(g) = ctx.blackboard.get_f64("router.goal_uid") {
            self.goal_uid = g as u64;
        }
        tracing::info!(
            "[router] loaded — start={} goal={}",
            self.start_uid,
            self.goal_uid
        );
        ctx.blackboard.set("router.active", "false");
    }

    fn on_unload(&mut self) {
        tracing::info!("[router] unloaded");
    }

    fn tick(
        &mut self,
        _telemetry: Option<&Telemetry>,
        _output: &mut ControlOutput,
        ctx: &PluginContext,
    ) {
        // Publish active state every tick so other plugins can check
        ctx.blackboard
            .set("router.active", if self.active { "true" } else { "false" });
    }
}

truckpilot_plugin_api::export_plugin!(RouterPlugin);

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    type NodeList = Vec<(u64, f64, f64)>;
    type EdgeList = Vec<(u64, u64, f64)>;

    fn simple_graph() -> (NodeList, EdgeList) {
        let nodes = vec![(1, 0.0, 0.0), (2, 100.0, 0.0), (3, 200.0, 0.0)];
        let edges = vec![(1, 2, 100.0), (2, 3, 100.0)];
        (nodes, edges)
    }

    #[test]
    fn finds_direct_route() {
        let (nodes, edges) = simple_graph();
        let path = RouterPlugin::plan(&nodes, &edges, 1, 3).unwrap();
        assert_eq!(path, vec![1, 2, 3]);
    }

    #[test]
    fn unreachable_returns_none() {
        let nodes = vec![(1, 0.0, 0.0), (2, 100.0, 0.0)];
        let edges = vec![];
        assert!(RouterPlugin::plan(&nodes, &edges, 1, 2).is_none());
    }

    #[test]
    fn start_equals_goal() {
        let (nodes, edges) = simple_graph();
        let path = RouterPlugin::plan(&nodes, &edges, 1, 1).unwrap();
        assert_eq!(path, vec![1]);
    }
}
