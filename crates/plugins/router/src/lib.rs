//! Router plugin — A* route planning on the MapGraph.
//!
//! On load, parses `graph.json` (path from blackboard `router.graph_path`
//! or `graph.json` in CWD) into flat node/edge lists. Every 50 ticks
//! (~1 Hz at 50 Hz daemon cadence) the router finds the nearest node to
//! the truck's current position and runs A* to `goal_uid`. The resulting
//! waypoint list is published as JSON to the blackboard for lane-keeper.
//!
//! ## Diagnostic Blackboard Keys (Phase 6.5c)
//!
//! | Key                              | Type   | Notes                              |
//! |----------------------------------|--------|------------------------------------|
//! | router.last_goal_uid_received    | string | Exact UID string from blackboard   |
//! | router.last_goal_received_at     | u64    | Epoch ms when goal changed         |
//! | router.current_goal_uid          | string | Current parsed goal UID (or "")    |
//! | router.last_planning_attempt_at  | u64    | Epoch ms when A* started           |
//! | router.last_planning_result      | string | "ok" / "uid_not_in_graph" / ...    |
//! | router.last_planning_duration_ms | u64    | Duration of last A* run            |
//! | router.last_planning_error_detail| string | Human-readable failure reason      |
//! | router.waypoint_count            | u32    | Waypoints in current plan (0=none) |
//! | router.path_total_distance_m     | f64    | Total route distance in metres     |

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::path::PathBuf;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use truckpilot_plugin_api::{ControlOutput, Plugin, PluginContext, Telemetry, TickPhase};

const DEFAULT_GRAPH_PATH: &str = "graph.json";

fn epoch_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
}

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

#[derive(serde::Deserialize)]
struct GraphFile {
    nodes: Vec<NodeJson>,
    edges: Vec<EdgeJson>,
}
#[derive(serde::Deserialize)]
struct NodeJson {
    uid: u64,
    x: f64,
    z: f64,
}
#[derive(serde::Deserialize)]
struct EdgeJson {
    from: u64,
    to: u64,
    distance_m: f64,
}

pub struct RouterPlugin {
    start_uid: u64,
    goal_uid: u64,
    /// Last value of `router.goal_uid` we read from the blackboard — used to
    /// detect new goals without re-triggering on every tick.
    last_seen_goal_str: String,
    graph_path: PathBuf,
    nodes: Vec<(u64, f64, f64)>,
    edges: Vec<(u64, u64, f64)>,
    positions: HashMap<u64, (f64, f64)>,
    active: bool,
    // ---- Diagnostic state (Phase 6.5c) ----
    last_planning_result: String,
    last_planning_error_detail: String,
    last_planning_duration_ms: u64,
    waypoint_count: u32,
    path_total_distance_m: f64,
}

impl Default for RouterPlugin {
    fn default() -> Self {
        Self {
            start_uid: 0,
            goal_uid: 0,
            last_seen_goal_str: String::new(),
            graph_path: PathBuf::new(),
            nodes: Vec::new(),
            edges: Vec::new(),
            positions: HashMap::new(),
            active: false,
            last_planning_result: String::new(),
            last_planning_error_detail: String::new(),
            last_planning_duration_ms: 0,
            waypoint_count: 0,
            path_total_distance_m: 0.0,
        }
    }
}

impl RouterPlugin {
    /// A* from `start` to `goal`. Returns `(path, total_distance_m)` on
    /// success, `None` if unreachable. Distance is the sum of edge weights
    /// along the chosen path (same as g[goal] in the A* cost map).
    fn plan(
        nodes: &[(u64, f64, f64)],
        edges: &[(u64, u64, f64)],
        start: u64,
        goal: u64,
    ) -> Option<(Vec<u64>, f64)> {
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
                let total_dist = *g.get(&goal).unwrap_or(&0.0);
                return Some((reconstruct(&came_from, start, goal), total_dist));
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

    fn find_nearest(&self, x: f64, z: f64) -> Option<u64> {
        self.nodes
            .iter()
            .map(|&(uid, nx, nz)| {
                let dx = nx - x;
                let dz = nz - z;
                (uid, dx * dx + dz * dz)
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(uid, _)| uid)
    }

    /// Execute one planning attempt and update all diagnostic keys on the
    /// blackboard. Called both on the scheduled replan tick and immediately
    /// when a new goal arrives (for fast feedback).
    fn do_replan(&mut self, telemetry: Option<&Telemetry>, ctx: &PluginContext) {
        if self.goal_uid == 0 || self.nodes.is_empty() {
            return;
        }

        let now_ms = epoch_ms();
        ctx.blackboard
            .set("router.last_planning_attempt_at", now_ms.to_string());

        let (px, pz) = telemetry
            .map(|t| (t.position[0], t.position[2]))
            .unwrap_or((0.0, 0.0));

        let t_start = Instant::now();

        // Guard: goal UID must exist in the graph.
        if !self.positions.contains_key(&self.goal_uid) {
            let duration_ms = t_start.elapsed().as_millis() as u64;
            self.last_planning_result = "uid_not_in_graph".to_string();
            self.last_planning_error_detail = format!(
                "Goal UID {} not found in graph ({} nodes loaded)",
                self.goal_uid,
                self.nodes.len()
            );
            self.last_planning_duration_ms = duration_ms;
            self.active = false;
            self.waypoint_count = 0;
            self.path_total_distance_m = 0.0;
            tracing::warn!("[router] {}", self.last_planning_error_detail);
            self.publish_planning_diag(ctx);
            return;
        }

        // Guard: truck must be on the graph.
        let Some(nearest) = self.find_nearest(px, pz) else {
            let duration_ms = t_start.elapsed().as_millis() as u64;
            self.last_planning_result = "start_node_unknown".to_string();
            self.last_planning_error_detail = format!(
                "Start position ({:.1}, {:.1}) has no nearby graph node",
                px, pz
            );
            self.last_planning_duration_ms = duration_ms;
            self.active = false;
            self.waypoint_count = 0;
            self.path_total_distance_m = 0.0;
            tracing::warn!("[router] {}", self.last_planning_error_detail);
            self.publish_planning_diag(ctx);
            return;
        };

        tracing::info!(
            "[router] A* start: start_uid={}, goal_uid={}",
            nearest,
            self.goal_uid
        );

        match Self::plan(&self.nodes, &self.edges, nearest, self.goal_uid) {
            Some((path, total_dist)) => {
                let duration_ms = t_start.elapsed().as_millis() as u64;
                let waypoints: Vec<[f64; 2]> = path
                    .iter()
                    .filter_map(|uid| self.positions.get(uid).copied().map(|(x, z)| [x, z]))
                    .collect();
                match serde_json::to_string(&waypoints) {
                    Ok(json) => {
                        ctx.blackboard.set("router.waypoints", &json);
                        self.last_planning_result = "ok".to_string();
                        self.last_planning_error_detail = String::new();
                        self.last_planning_duration_ms = duration_ms;
                        self.waypoint_count = waypoints.len() as u32;
                        self.path_total_distance_m = total_dist;
                        self.active = true;
                        tracing::info!(
                            "[router] A* success: waypoints={}, distance={:.0}m, duration={}ms",
                            self.waypoint_count,
                            total_dist,
                            duration_ms
                        );
                    }
                    Err(e) => {
                        let duration_ms = t_start.elapsed().as_millis() as u64;
                        self.last_planning_result = "no_path_found".to_string();
                        self.last_planning_error_detail =
                            format!("Failed to serialise waypoints: {e}");
                        self.last_planning_duration_ms = duration_ms;
                        self.active = false;
                        self.waypoint_count = 0;
                        self.path_total_distance_m = 0.0;
                        tracing::warn!("[router] {}", self.last_planning_error_detail);
                    }
                }
            }
            None => {
                let duration_ms = t_start.elapsed().as_millis() as u64;
                self.last_planning_result = "no_path_found".to_string();
                self.last_planning_error_detail = format!(
                    "No path from start UID {} to goal UID {} after graph search",
                    nearest, self.goal_uid
                );
                self.last_planning_duration_ms = duration_ms;
                self.active = false;
                self.waypoint_count = 0;
                self.path_total_distance_m = 0.0;
                tracing::warn!("[router] {}", self.last_planning_error_detail);
            }
        }

        self.publish_planning_diag(ctx);
    }

    fn publish_planning_diag(&self, ctx: &PluginContext) {
        ctx.blackboard
            .set("router.last_planning_result", &self.last_planning_result);
        ctx.blackboard.set(
            "router.last_planning_duration_ms",
            self.last_planning_duration_ms.to_string(),
        );
        ctx.blackboard.set(
            "router.last_planning_error_detail",
            &self.last_planning_error_detail,
        );
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
        "0.3.0"
    }
    fn settings_schema(&self) -> &str {
        r#"{"type":"object","properties":{"start_uid":{"type":"integer"},"goal_uid":{"type":"integer"},"graph_path":{"type":"string"}}}"#
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        // Use string parse for u64 to avoid f64 precision loss on high UIDs.
        if let Some(s) = ctx
            .blackboard
            .get("router.start_uid")
            .and_then(|v| v.trim().parse::<u64>().ok())
        {
            self.start_uid = s;
        }
        if let Some(raw) = ctx.blackboard.get("router.goal_uid") {
            if let Ok(uid) = raw.trim().parse::<u64>() {
                self.goal_uid = uid;
            }
            // Initialise last_seen so we don't re-fire "new goal" on tick 0.
            self.last_seen_goal_str = raw;
        }
        self.graph_path = ctx
            .blackboard
            .get("router.graph_path")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from(DEFAULT_GRAPH_PATH));

        match std::fs::read_to_string(&self.graph_path) {
            Ok(data) => match serde_json::from_str::<GraphFile>(&data) {
                Ok(g) => {
                    self.nodes = g.nodes.iter().map(|n| (n.uid, n.x, n.z)).collect();
                    self.edges = g
                        .edges
                        .iter()
                        .map(|e| (e.from, e.to, e.distance_m))
                        .collect();
                    self.positions = g.nodes.iter().map(|n| (n.uid, (n.x, n.z))).collect();
                    tracing::info!(
                        "[router] loaded {} nodes / {} edges from {:?}",
                        self.nodes.len(),
                        self.edges.len(),
                        self.graph_path
                    );
                }
                Err(e) => tracing::warn!("[router] cannot parse {:?}: {e}", self.graph_path),
            },
            Err(e) => tracing::warn!("[router] cannot read {:?}: {e}", self.graph_path),
        }

        // Initialise diagnostic keys so the UI has something to display.
        ctx.blackboard.set("router.active", "false");
        ctx.blackboard.set("router.waypoint_count", "0");
        ctx.blackboard.set("router.path_total_distance_m", "0");
        ctx.blackboard.set("router.last_planning_result", "");
        ctx.blackboard.set("router.last_planning_error_detail", "");
        if !self.last_seen_goal_str.is_empty() {
            ctx.blackboard
                .set("router.current_goal_uid", &self.last_seen_goal_str);
        } else {
            ctx.blackboard.set("router.current_goal_uid", "");
        }
    }

    fn on_unload(&mut self) {
        tracing::info!("[router] unloaded");
    }

    fn default_phase(&self) -> TickPhase {
        TickPhase::PhaseA
    }

    fn tick(
        &mut self,
        telemetry: Option<&Telemetry>,
        _output: &mut ControlOutput,
        ctx: &PluginContext,
    ) {
        // ── 1. Detect goal changes ────────────────────────────────────────────
        let goal_str = ctx.blackboard.get("router.goal_uid").unwrap_or_default();
        if goal_str != self.last_seen_goal_str {
            self.last_seen_goal_str = goal_str.clone();
            let now_ms = epoch_ms();
            ctx.blackboard
                .set("router.last_goal_uid_received", &goal_str);
            ctx.blackboard
                .set("router.last_goal_received_at", now_ms.to_string());

            if goal_str.is_empty() {
                // Goal cleared.
                self.goal_uid = 0;
                ctx.blackboard.set("router.current_goal_uid", "");
                tracing::info!("[router] goal cleared");
            } else {
                match goal_str.trim().parse::<u64>() {
                    Ok(uid) => {
                        self.goal_uid = uid;
                        ctx.blackboard.set("router.current_goal_uid", &goal_str);
                        tracing::info!("[router] new goal received: uid={}", uid);
                        // Trigger an immediate plan so the user sees feedback
                        // without waiting for the next scheduled replan tick.
                        self.do_replan(telemetry, ctx);
                    }
                    Err(_) => {
                        self.goal_uid = 0;
                        self.last_planning_result = "uid_parse_error".to_string();
                        self.last_planning_error_detail = format!(
                            "UID parse error: could not parse '{}' as u64",
                            goal_str
                        );
                        ctx.blackboard.set("router.current_goal_uid", "");
                        ctx.blackboard
                            .set("router.last_planning_result", "uid_parse_error");
                        ctx.blackboard.set(
                            "router.last_planning_error_detail",
                            &self.last_planning_error_detail,
                        );
                        tracing::warn!("[router] {}", self.last_planning_error_detail);
                    }
                }
            }
        }

        // ── 2. Scheduled replan ───────────────────────────────────────────────
        if ctx.is_replan_tick() {
            self.do_replan(telemetry, ctx);
        }

        // ── 3. Publish current state (every tick) ─────────────────────────────
        ctx.blackboard
            .set("router.active", if self.active { "true" } else { "false" });
        ctx.blackboard
            .set("router.waypoint_count", self.waypoint_count.to_string());
        ctx.blackboard.set(
            "router.path_total_distance_m",
            format!("{:.1}", self.path_total_distance_m),
        );
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
        (
            vec![(1, 0.0, 0.0), (2, 100.0, 0.0), (3, 200.0, 0.0)],
            vec![(1, 2, 100.0), (2, 3, 100.0)],
        )
    }

    #[test]
    fn finds_direct_route() {
        let (n, e) = simple_graph();
        let (path, dist) = RouterPlugin::plan(&n, &e, 1, 3).unwrap();
        assert_eq!(path, vec![1, 2, 3]);
        assert!((dist - 200.0).abs() < 0.001, "dist={dist}");
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
        let (path, dist) = RouterPlugin::plan(&n, &e, 1, 1).unwrap();
        assert_eq!(path, vec![1]);
        assert!(dist.abs() < 0.001, "dist={dist}");
    }

    #[test]
    fn tick_produces_waypoints_on_replan_tick() {
        let (n, e) = simple_graph();
        let mut p = RouterPlugin {
            goal_uid: 3,
            nodes: n.clone(),
            edges: e,
            positions: n.iter().map(|&(u, x, z)| (u, (x, z))).collect(),
            ..Default::default()
        };
        let bb = SharedBlackboard::new();
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseA)
            .with_tick_count(50);
        let mut out = ControlOutput::default();
        let t = Telemetry {
            position: [0.0, 0.0, 0.0],
            heading: 0.0,
            pitch: 0.0,
            roll: 0.0,
            speed_ms: 0.0,
            engine_rpm: 0.0,
            cruise_control_kmh: 0.0,
            nav_speed_limit_kmh: -1.0,
            lead_vehicle_distance_m: -1.0,
            accel_longitudinal: -1.0,
            fuel_liters: -1.0,
            odometer_km: -1.0,
        };
        p.tick(Some(&t), &mut out, &ctx);
        assert_eq!(bb.get("router.active").as_deref(), Some("true"));
        let wp = bb.get("router.waypoints").expect("waypoints set");
        assert!(wp.contains("200"));
        assert_eq!(bb.get("router.last_planning_result").as_deref(), Some("ok"));
        assert_eq!(bb.get("router.waypoint_count").as_deref(), Some("3"));
    }

    #[test]
    fn tick_skips_when_not_replan_tick() {
        let (n, e) = simple_graph();
        let mut p = RouterPlugin {
            goal_uid: 3,
            nodes: n.clone(),
            edges: e,
            positions: n.iter().map(|&(u, x, z)| (u, (x, z))).collect(),
            ..Default::default()
        };
        let bb = SharedBlackboard::new();
        // PhaseC (wrong phase) — is_replan_tick() returns false → no replan.
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseC)
            .with_tick_count(50);
        let mut out = ControlOutput::default();
        p.tick(None, &mut out, &ctx);
        assert_eq!(bb.get("router.active").as_deref(), Some("false"));
        assert!(bb.get("router.waypoints").is_none());
    }

    #[test]
    fn new_goal_via_blackboard_triggers_immediate_plan() {
        let (n, e) = simple_graph();
        let mut p = RouterPlugin {
            nodes: n.clone(),
            edges: e,
            positions: n.iter().map(|&(u, x, z)| (u, (x, z))).collect(),
            ..Default::default()
        };
        let bb = SharedBlackboard::new();
        // Set goal via blackboard (simulates IPC SetRouterGoal).
        bb.set("router.goal_uid", "3");
        // Non-replan tick — but new goal should trigger immediate plan.
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseC)
            .with_tick_count(7);
        let mut out = ControlOutput::default();
        let t = Telemetry {
            position: [0.0, 0.0, 0.0],
            heading: 0.0,
            pitch: 0.0,
            roll: 0.0,
            speed_ms: 0.0,
            engine_rpm: 0.0,
            cruise_control_kmh: 0.0,
            nav_speed_limit_kmh: -1.0,
            lead_vehicle_distance_m: -1.0,
            accel_longitudinal: -1.0,
            fuel_liters: -1.0,
            odometer_km: -1.0,
        };
        p.tick(Some(&t), &mut out, &ctx);
        assert_eq!(bb.get("router.active").as_deref(), Some("true"));
        assert_eq!(bb.get("router.last_planning_result").as_deref(), Some("ok"));
        assert_eq!(bb.get("router.current_goal_uid").as_deref(), Some("3"));
    }

    #[test]
    fn uid_not_in_graph_sets_diagnostic_key() {
        let (n, e) = simple_graph();
        let mut p = RouterPlugin {
            nodes: n.clone(),
            edges: e,
            positions: n.iter().map(|&(u, x, z)| (u, (x, z))).collect(),
            ..Default::default()
        };
        let bb = SharedBlackboard::new();
        // UID 999 does not exist in the simple graph.
        bb.set("router.goal_uid", "999");
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseC)
            .with_tick_count(7);
        let mut out = ControlOutput::default();
        p.tick(None, &mut out, &ctx);
        assert_eq!(
            bb.get("router.last_planning_result").as_deref(),
            Some("uid_not_in_graph")
        );
        assert_eq!(bb.get("router.active").as_deref(), Some("false"));
        let detail = bb
            .get("router.last_planning_error_detail")
            .unwrap_or_default();
        assert!(detail.contains("999"), "detail={detail}");
    }

    #[test]
    fn uid_parse_error_sets_diagnostic_key() {
        let (n, e) = simple_graph();
        let mut p = RouterPlugin {
            nodes: n.clone(),
            edges: e,
            positions: n.iter().map(|&(u, x, z)| (u, (x, z))).collect(),
            ..Default::default()
        };
        let bb = SharedBlackboard::new();
        bb.set("router.goal_uid", "not_a_number");
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseC)
            .with_tick_count(7);
        let mut out = ControlOutput::default();
        p.tick(None, &mut out, &ctx);
        assert_eq!(
            bb.get("router.last_planning_result").as_deref(),
            Some("uid_parse_error")
        );
    }

    #[test]
    fn high_u64_uid_does_not_lose_precision() {
        // Hamburg UID exceeds 2^53 — must not be corrupted by f64 cast.
        const HAMBURG: u64 = 6_526_933_291_294_064_640;
        let hamburg_str = HAMBURG.to_string();

        let (n, e) = simple_graph();
        let mut p = RouterPlugin {
            nodes: n.clone(),
            edges: e,
            positions: n.iter().map(|&(u, x, z)| (u, (x, z))).collect(),
            ..Default::default()
        };
        let bb = SharedBlackboard::new();
        bb.set("router.goal_uid", &hamburg_str);
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseC)
            .with_tick_count(7);
        let mut out = ControlOutput::default();
        p.tick(None, &mut out, &ctx);
        // UID not in the tiny simple graph, but must have been parsed correctly.
        assert_eq!(p.goal_uid, HAMBURG, "goal_uid corrupted by f64 cast");
        assert_eq!(
            bb.get("router.last_planning_result").as_deref(),
            Some("uid_not_in_graph")
        );
    }
}
