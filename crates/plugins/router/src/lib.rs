//! Router plugin — A* route planning on the MapGraph.
//!
//! On load, parses `graph.json` (path from blackboard `router.graph_path`
//! or `graph.json` in CWD) into flat node/edge lists. The graph data is
//! wrapped in an `Arc` and shared with a dedicated worker thread.
//!
//! The plugin tick is non-blocking (<5 ms):
//!   1. Poll worker result via `try_recv` (zero-copy if no result ready)
//!   2. Detect goal changes; submit a new request immediately
//!   3. Trigger a periodic replan on PhaseA cadence if not pending
//!   4. Publish current state to the blackboard
//!
//! Heavy A* work (find_nearest + A*) runs on the `router-worker` thread.
//!
//! ## Diagnostic Blackboard Keys (Phase 6.5c)
//!
//! | Key                              | Type   | Notes                              |
//! |----------------------------------|--------|------------------------------------|
//! | router.last_goal_uid_received    | string | Exact UID string from blackboard   |
//! | router.last_goal_received_at     | u64    | Epoch ms when goal changed         |
//! | router.current_goal_uid          | string | Current parsed goal UID (or "")    |
//! | router.last_planning_attempt_at  | u64    | Epoch ms when A* was submitted     |
//! | router.last_planning_result      | string | "ok" / "uid_not_in_graph" / ...    |
//! | router.last_planning_duration_ms | u64    | Duration of last A* run            |
//! | router.last_planning_error_detail| string | Human-readable failure reason      |
//! | router.waypoint_count            | u32    | Waypoints in current plan (0=none) |
//! | router.path_total_distance_m     | f64    | Total route distance in metres     |

use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::path::PathBuf;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::Arc;
use std::thread::JoinHandle;
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

// ---------------------------------------------------------------------------
// Graph data shared between plugin tick and worker thread (immutable after load)
// ---------------------------------------------------------------------------

struct RouterGraph {
    nodes: Vec<(u64, f64, f64)>,
    edges: Vec<(u64, u64, f64)>,
    positions: HashMap<u64, (f64, f64)>,
}

impl RouterGraph {
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
}

// ---------------------------------------------------------------------------
// Worker communication types
// ---------------------------------------------------------------------------

struct RouteRequest {
    truck_x: f64,
    truck_z: f64,
    goal_uid: u64,
}

struct RouteResult {
    /// Echoed goal_uid for stale-result detection.
    goal_uid: u64,
    success: bool,
    waypoints: Vec<[f64; 2]>,
    waypoint_count: usize,
    distance_m: f64,
    plan_ms: u64,
    result_kind: String,
    error_detail: String,
}

// ---------------------------------------------------------------------------
// Plugin struct
// ---------------------------------------------------------------------------

pub struct RouterPlugin {
    goal_uid: u64,
    last_seen_goal_str: String,
    graph_path: PathBuf,
    /// Shared with the worker thread via Arc clone.
    graph: Option<Arc<RouterGraph>>,
    active: bool,
    /// True while a route request is in flight (worker hasn't responded yet).
    pending_request: bool,
    request_tx: Option<Sender<RouteRequest>>,
    /// Wrapped in Mutex so RouterPlugin satisfies Plugin: Sync.
    /// Only ever accessed from the single plugin-tick thread.
    result_rx: Option<std::sync::Mutex<Receiver<RouteResult>>>,
    worker_handle: Option<JoinHandle<()>>,
    // ---- Diagnostic state ----
    last_planning_result: String,
    last_planning_error_detail: String,
    last_planning_duration_ms: u64,
    waypoint_count: u32,
    path_total_distance_m: f64,
}

impl Default for RouterPlugin {
    fn default() -> Self {
        Self {
            goal_uid: 0,
            last_seen_goal_str: String::new(),
            graph_path: PathBuf::new(),
            graph: None,
            active: false,
            pending_request: false,
            request_tx: None,
            result_rx: None,
            worker_handle: None,
            last_planning_result: String::new(),
            last_planning_error_detail: String::new(),
            last_planning_duration_ms: 0,
            waypoint_count: 0,
            path_total_distance_m: 0.0,
        }
    }
}

impl Drop for RouterPlugin {
    fn drop(&mut self) {
        // Dropping the sender closes the channel → worker exits its recv loop.
        self.request_tx = None;
        if let Some(h) = self.worker_handle.take() {
            let _ = h.join();
        }
    }
}

impl RouterPlugin {
    /// A* from `start` to `goal`. Returns `(path, total_distance_m)` on
    /// success, `None` if unreachable. Static so tests can call it directly.
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

    fn spawn_worker(&mut self, graph: Arc<RouterGraph>) {
        let (req_tx, req_rx) = channel::<RouteRequest>();
        let (res_tx, res_rx) = channel::<RouteResult>();
        let graph_clone = Arc::clone(&graph);
        let handle = std::thread::Builder::new()
            .name("router-worker".to_string())
            .spawn(move || router_worker_loop(req_rx, res_tx, graph_clone))
            .expect("spawn router-worker thread");
        self.graph = Some(graph);
        self.request_tx = Some(req_tx);
        self.result_rx = Some(std::sync::Mutex::new(res_rx));
        self.worker_handle = Some(handle);
    }

    fn send_route_request(&mut self, pos_x: f64, pos_z: f64, ctx: &PluginContext) {
        if let Some(chan) = &self.request_tx {
            ctx.blackboard
                .set("router.last_planning_attempt_at", epoch_ms().to_string());
            let _ = chan.send(RouteRequest {
                truck_x: pos_x,
                truck_z: pos_z,
                goal_uid: self.goal_uid,
            });
            self.pending_request = true;
        }
    }
}

// ---------------------------------------------------------------------------
// Worker thread
// ---------------------------------------------------------------------------

fn router_worker_loop(
    req_rx: Receiver<RouteRequest>,
    res_tx: Sender<RouteResult>,
    graph: Arc<RouterGraph>,
) {
    while let Ok(req) = req_rx.recv() {
        let t_start = Instant::now();

        // Validate goal exists before the expensive A*.
        if !graph.positions.contains_key(&req.goal_uid) {
            let _ = res_tx.send(RouteResult {
                goal_uid: req.goal_uid,
                success: false,
                waypoints: vec![],
                waypoint_count: 0,
                distance_m: 0.0,
                plan_ms: t_start.elapsed().as_millis() as u64,
                result_kind: "uid_not_in_graph".to_string(),
                error_detail: format!(
                    "Goal UID {} not found in graph ({} nodes loaded)",
                    req.goal_uid,
                    graph.nodes.len()
                ),
            });
            continue;
        }

        // Find nearest graph node to the truck's current position.
        let Some(start_uid) = graph.find_nearest(req.truck_x, req.truck_z) else {
            let _ = res_tx.send(RouteResult {
                goal_uid: req.goal_uid,
                success: false,
                waypoints: vec![],
                waypoint_count: 0,
                distance_m: 0.0,
                plan_ms: t_start.elapsed().as_millis() as u64,
                result_kind: "start_node_unknown".to_string(),
                error_detail: format!(
                    "Start position ({:.1}, {:.1}) has no nearby graph node",
                    req.truck_x, req.truck_z,
                ),
            });
            continue;
        };

        tracing::info!(
            "[router-worker] A* start={} goal={}",
            start_uid,
            req.goal_uid
        );

        match RouterPlugin::plan(&graph.nodes, &graph.edges, start_uid, req.goal_uid) {
            Some((path, total_dist)) => {
                let waypoints: Vec<[f64; 2]> = path
                    .iter()
                    .filter_map(|uid| graph.positions.get(uid).copied().map(|(x, z)| [x, z]))
                    .collect();
                let wpc = waypoints.len();
                let plan_ms = t_start.elapsed().as_millis() as u64;
                tracing::info!(
                    "[router-worker] A* ok: {} waypoints, {:.0}m, {}ms",
                    wpc,
                    total_dist,
                    plan_ms
                );
                let _ = res_tx.send(RouteResult {
                    goal_uid: req.goal_uid,
                    success: true,
                    waypoints,
                    waypoint_count: wpc,
                    distance_m: total_dist,
                    plan_ms,
                    result_kind: "ok".to_string(),
                    error_detail: String::new(),
                });
            }
            None => {
                let _ = res_tx.send(RouteResult {
                    goal_uid: req.goal_uid,
                    success: false,
                    waypoints: vec![],
                    waypoint_count: 0,
                    distance_m: 0.0,
                    plan_ms: t_start.elapsed().as_millis() as u64,
                    result_kind: "no_path_found".to_string(),
                    error_detail: format!(
                        "No path from start UID {} to goal UID {} after graph search",
                        start_uid, req.goal_uid
                    ),
                });
            }
        }
    }
    tracing::info!("[router-worker] thread exiting");
}

// ---------------------------------------------------------------------------
// Free functions (unchanged)
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Plugin impl
// ---------------------------------------------------------------------------

impl Plugin for RouterPlugin {
    fn name(&self) -> &str {
        "router"
    }
    fn version(&self) -> &str {
        "0.4.0"
    }
    fn settings_schema(&self) -> &str {
        r#"{"type":"object","properties":{"goal_uid":{"type":"integer"},"graph_path":{"type":"string"}}}"#
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        if let Some(raw) = ctx.blackboard.get("router.goal_uid") {
            if let Ok(uid) = raw.trim().parse::<u64>() {
                self.goal_uid = uid;
            }
            self.last_seen_goal_str = raw;
        }
        self.graph_path = ctx
            .blackboard
            .get("router.graph_path")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from(DEFAULT_GRAPH_PATH));

        let graph = match std::fs::read_to_string(&self.graph_path) {
            Ok(data) => match serde_json::from_str::<GraphFile>(&data) {
                Ok(g) => {
                    let node_count = g.nodes.len();
                    let edge_count = g.edges.len();
                    let nodes = g.nodes.iter().map(|n| (n.uid, n.x, n.z)).collect();
                    let edges = g
                        .edges
                        .iter()
                        .map(|e| (e.from, e.to, e.distance_m))
                        .collect();
                    let positions = g.nodes.iter().map(|n| (n.uid, (n.x, n.z))).collect();
                    tracing::info!(
                        "[router] loaded {} nodes / {} edges from {:?}",
                        node_count,
                        edge_count,
                        self.graph_path
                    );
                    Some(Arc::new(RouterGraph {
                        nodes,
                        edges,
                        positions,
                    }))
                }
                Err(e) => {
                    tracing::warn!("[router] cannot parse {:?}: {e}", self.graph_path);
                    None
                }
            },
            Err(e) => {
                tracing::warn!("[router] cannot read {:?}: {e}", self.graph_path);
                None
            }
        };

        if let Some(g) = graph {
            self.spawn_worker(g);
            tracing::info!("[router] worker thread spawned");
        }

        ctx.blackboard.set("router.active", "false");
        ctx.blackboard.set("router.waypoint_count", "0");
        ctx.blackboard.set("router.path_total_distance_m", "0");
        ctx.blackboard.set("router.last_planning_result", "");
        ctx.blackboard.set("router.last_planning_error_detail", "");
        ctx.blackboard.set(
            "router.current_goal_uid",
            if self.last_seen_goal_str.is_empty() {
                ""
            } else {
                &self.last_seen_goal_str
            },
        );
    }

    fn on_unload(&mut self) {
        // Drop sender first so the worker sees the channel close and exits.
        self.request_tx = None;
        if let Some(h) = self.worker_handle.take() {
            let _ = h.join();
        }
        tracing::info!("[router] worker thread shutdown");
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
        // ── 1. Poll worker result (non-blocking, always first) ────────────────
        if let Some(rx) = &self.result_rx {
            if let Ok(result) = rx.lock().unwrap().try_recv() {
                self.pending_request = false;

                if result.goal_uid == self.goal_uid {
                    self.last_planning_result = result.result_kind.clone();
                    self.last_planning_error_detail = result.error_detail.clone();
                    self.last_planning_duration_ms = result.plan_ms;

                    if result.success {
                        match serde_json::to_string(&result.waypoints) {
                            Ok(json) => {
                                ctx.blackboard.set("router.waypoints", &json);
                                self.waypoint_count = result.waypoint_count as u32;
                                self.path_total_distance_m = result.distance_m;
                                self.active = true;
                                tracing::info!(
                                    "[router] route ready: {} waypoints, {:.1}km, {}ms",
                                    result.waypoint_count,
                                    result.distance_m / 1000.0,
                                    result.plan_ms,
                                );
                            }
                            Err(e) => {
                                self.last_planning_result = "serialise_error".to_string();
                                self.last_planning_error_detail =
                                    format!("Failed to serialise waypoints: {e}");
                                self.active = false;
                                self.waypoint_count = 0;
                                self.path_total_distance_m = 0.0;
                            }
                        }
                    } else {
                        self.active = false;
                        self.waypoint_count = 0;
                        self.path_total_distance_m = 0.0;
                        tracing::warn!("[router] route failed: {}", result.error_detail);
                    }

                    self.publish_planning_diag(ctx);
                } else {
                    tracing::debug!(
                        "[router] discarding stale result (result_goal={} current={})",
                        result.goal_uid,
                        self.goal_uid
                    );
                }
            }
        }

        // ── 2. Detect goal changes ────────────────────────────────────────────
        let goal_str = ctx.blackboard.get("router.goal_uid").unwrap_or_default();
        if goal_str != self.last_seen_goal_str {
            self.last_seen_goal_str = goal_str.clone();
            let now_ms = epoch_ms();
            ctx.blackboard
                .set("router.last_goal_uid_received", &goal_str);
            ctx.blackboard
                .set("router.last_goal_received_at", now_ms.to_string());

            if goal_str.is_empty() {
                self.goal_uid = 0;
                ctx.blackboard.set("router.current_goal_uid", "");
                tracing::info!("[router] goal cleared");
            } else {
                match goal_str.trim().parse::<u64>() {
                    Ok(uid) => {
                        self.goal_uid = uid;
                        ctx.blackboard.set("router.current_goal_uid", &goal_str);
                        tracing::info!("[router] new goal received: uid={}", uid);
                        // Submit immediately for fast feedback.
                        let (pos_x, pos_z) = telemetry
                            .map(|t| (t.position[0], t.position[2]))
                            .unwrap_or((0.0, 0.0));
                        self.send_route_request(pos_x, pos_z, ctx);
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

        // ── 3. Periodic replan (PhaseA cadence, skip if request in flight) ────
        if ctx.is_replan_tick() && !self.pending_request && self.goal_uid != 0 {
            let (pos_x, pos_z) = telemetry
                .map(|t| (t.position[0], t.position[2]))
                .unwrap_or((0.0, 0.0));
            self.send_route_request(pos_x, pos_z, ctx);
        }

        // ── 4. Publish current state (every tick) ─────────────────────────────
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

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;
    use truckpilot_plugin_api::SharedBlackboard;

    type NodeList = Vec<(u64, f64, f64)>;
    type EdgeList = Vec<(u64, u64, f64)>;

    fn simple_graph() -> (NodeList, EdgeList) {
        (
            vec![(1, 0.0, 0.0), (2, 100.0, 0.0), (3, 200.0, 0.0)],
            vec![(1, 2, 100.0), (2, 3, 100.0)],
        )
    }

    fn fake_telemetry_at(x: f64, z: f64) -> Telemetry {
        Telemetry {
            position: [x, 0.0, z],
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
        }
    }

    /// Build a RouterPlugin with a pre-built graph and a live worker thread.
    /// Used by tests that exercise the async routing path.
    fn plugin_with_worker(nodes: NodeList, edges: EdgeList) -> RouterPlugin {
        let positions = nodes.iter().map(|&(u, x, z)| (u, (x, z))).collect();
        let graph = Arc::new(RouterGraph {
            nodes,
            edges,
            positions,
        });
        let mut p = RouterPlugin::default();
        p.spawn_worker(graph);
        p
    }

    // ---- A* unit tests (call plan() directly, no thread) -------------------

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

    // ---- Tick tests (async: submit → sleep → poll) -------------------------

    #[test]
    fn tick_produces_waypoints_on_replan_tick() {
        let (n, e) = simple_graph();
        let mut p = plugin_with_worker(n, e);
        p.goal_uid = 3;

        let bb = SharedBlackboard::new();
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseA)
            .with_tick_count(50);
        let mut out = ControlOutput::default();
        let t = fake_telemetry_at(0.0, 0.0);

        // First tick: submits route request to worker.
        p.tick(Some(&t), &mut out, &ctx);
        assert!(p.pending_request, "request should be pending after first tick");

        // Let the worker complete A*.
        std::thread::sleep(Duration::from_millis(300));

        // Second tick: polls result and publishes to blackboard.
        p.tick(Some(&t), &mut out, &ctx);

        assert_eq!(bb.get("router.active").as_deref(), Some("true"));
        let wp = bb.get("router.waypoints").expect("waypoints set");
        assert!(wp.contains("200"), "wp={wp}");
        assert_eq!(bb.get("router.last_planning_result").as_deref(), Some("ok"));
        assert_eq!(bb.get("router.waypoint_count").as_deref(), Some("3"));
    }

    #[test]
    fn tick_skips_when_not_replan_tick() {
        let (n, e) = simple_graph();
        let mut p = plugin_with_worker(n, e);
        p.goal_uid = 3;

        let bb = SharedBlackboard::new();
        // PhaseC (wrong phase) — is_replan_tick() returns false → no replan.
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseC)
            .with_tick_count(50);
        let mut out = ControlOutput::default();
        p.tick(None, &mut out, &ctx);
        assert_eq!(bb.get("router.active").as_deref(), Some("false"));
        assert!(bb.get("router.waypoints").is_none());
        assert!(!p.pending_request, "no request should be pending");
    }

    #[test]
    fn new_goal_via_blackboard_triggers_immediate_plan() {
        let (n, e) = simple_graph();
        let mut p = plugin_with_worker(n, e);

        let bb = SharedBlackboard::new();
        bb.set("router.goal_uid", "3");
        // Non-replan tick — new goal triggers immediate submit.
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseC)
            .with_tick_count(7);
        let mut out = ControlOutput::default();
        let t = fake_telemetry_at(0.0, 0.0);

        // First tick: detects new goal, submits immediately.
        p.tick(Some(&t), &mut out, &ctx);
        assert!(p.pending_request, "should be pending after new goal");
        assert_eq!(bb.get("router.current_goal_uid").as_deref(), Some("3"));

        // Let the worker complete.
        std::thread::sleep(Duration::from_millis(300));

        // Second tick: polls result.
        p.tick(Some(&t), &mut out, &ctx);

        assert_eq!(bb.get("router.active").as_deref(), Some("true"));
        assert_eq!(bb.get("router.last_planning_result").as_deref(), Some("ok"));
        assert_eq!(bb.get("router.current_goal_uid").as_deref(), Some("3"));
    }

    #[test]
    fn uid_not_in_graph_sets_diagnostic_key() {
        let (n, e) = simple_graph();
        let mut p = plugin_with_worker(n, e);

        let bb = SharedBlackboard::new();
        bb.set("router.goal_uid", "999");
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseC)
            .with_tick_count(7);
        let mut out = ControlOutput::default();

        // First tick: detects new goal, submits.
        p.tick(None, &mut out, &ctx);
        assert!(p.pending_request);

        // Worker quickly finds uid_not_in_graph.
        std::thread::sleep(Duration::from_millis(200));

        // Second tick: receives error result.
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
        let mut p = plugin_with_worker(n, e);

        let bb = SharedBlackboard::new();
        bb.set("router.goal_uid", "not_a_number");
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseC)
            .with_tick_count(7);
        let mut out = ControlOutput::default();
        // Parse error is caught in tick() before reaching the worker.
        p.tick(None, &mut out, &ctx);
        assert_eq!(
            bb.get("router.last_planning_result").as_deref(),
            Some("uid_parse_error")
        );
        assert!(!p.pending_request, "parse error must not submit a request");
    }

    #[test]
    fn high_u64_uid_does_not_lose_precision() {
        // Hamburg UID exceeds 2^53 — must not be corrupted by f64 cast.
        const HAMBURG: u64 = 6_526_933_291_294_064_640;
        let hamburg_str = HAMBURG.to_string();

        let (n, e) = simple_graph();
        let mut p = plugin_with_worker(n, e);

        let bb = SharedBlackboard::new();
        bb.set("router.goal_uid", &hamburg_str);
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseC)
            .with_tick_count(7);
        let mut out = ControlOutput::default();

        // First tick: parses HAMBURG uid and submits.
        p.tick(None, &mut out, &ctx);
        assert_eq!(p.goal_uid, HAMBURG, "goal_uid corrupted by f64 cast");
        assert!(p.pending_request);

        // Worker finds uid_not_in_graph.
        std::thread::sleep(Duration::from_millis(200));

        // Second tick: receives result.
        p.tick(None, &mut out, &ctx);

        assert_eq!(
            bb.get("router.last_planning_result").as_deref(),
            Some("uid_not_in_graph")
        );
    }
}
