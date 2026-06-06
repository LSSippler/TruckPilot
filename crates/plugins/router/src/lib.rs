//! Router plugin — A* route planning on the MapGraph.
//!
//! On load, parses `graph.json` (path from blackboard `router.graph_path`
//! or `graph.json` in CWD) into flat node/edge lists. The graph data is
//! wrapped in an `Arc` and shared with a dedicated worker thread.
//!
//! The plugin tick is non-blocking (<5 ms):
//!   1. Poll worker result via `try_recv` (zero-copy if no result ready)
//!   2. Detect goal changes; submit a new request immediately
//!      2.5 Off-route auto-replan check (Phase 6.5q)
//!   3. Trigger a periodic replan on PhaseA cadence if not pending
//!   4. Publish current state to the blackboard
//!
//! Heavy A* work (heading-aware snap + A*) runs on the `router-worker` thread.
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
//! | router.last_snap_dist            | f64    | Distance to snapped start node (m) |
//! | router.last_snap_heading_filter_applied | bool | Whether heading filter was used |
//! | router.snap_method               | string | "edge" / "node" / "" — snap strategy used for last plan |
//! | router.auto_replan_count         | u32    | Number of auto-replans triggered   |
//! | router.auto_replan_triggered_at  | u64    | Epoch ms of last auto-replan       |
//! | router.last_replan_reason        | string | "off_route" or ""                  |

use std::collections::{HashMap, HashSet, VecDeque};
use std::path::PathBuf;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use truckpilot_plugin_api::graph::RouterGraph;
use truckpilot_plugin_api::{ControlOutput, Plugin, PluginContext, Telemetry, TickPhase};

const DEFAULT_GRAPH_PATH: &str = "graph.json";
/// Node-snap radius: fallback when edge-snap finds nothing.
const SNAP_RADIUS_M: f64 = 20.0;
/// Edge-snap radius: project truck position onto the nearest road edge within this distance.
/// Highway segments can span 50–200 m between nodes; 100 m covers all practical cases.
const EDGE_SNAP_RADIUS_M: f64 = 100.0;
/// Wider radius for off-route detection vs the snap limit.
const OFF_ROUTE_DETECT_RADIUS_M: f64 = 50.0;

/// Sliding-window size for snap stabilisation (Phase 6.5t).
/// At 1 Hz (PhaseA), 5 frames = 5 s history.
const SNAP_WINDOW_SIZE: usize = 5;
const SNAP_MAJORITY_THRESHOLD: usize = 3;
const SNAP_HYSTERESIS_THRESHOLD: usize = 4;

// ---------------------------------------------------------------------------
// SnapWindow — sliding-window majority vote for snap stabilisation (Phase 6.5t)
// ---------------------------------------------------------------------------

struct SnapWindow {
    buffer: VecDeque<Option<u64>>,
    capacity: usize,
}

impl SnapWindow {
    fn new(capacity: usize) -> Self {
        Self {
            buffer: VecDeque::with_capacity(capacity),
            capacity,
        }
    }

    fn clear(&mut self) {
        self.buffer.clear();
    }

    fn push_snap(&mut self, snap: Option<u64>) {
        self.buffer.push_back(snap);
        if self.buffer.len() > self.capacity {
            self.buffer.pop_front();
        }
    }

    /// Majority vote with hysteresis.
    /// Returns `(stable_edge, vote_count_for_stable, distinct_edges_in_window)`.
    ///
    /// `stable_edge` is `None` if no edge reaches `majority`.
    /// Once a `stable_edge` is set, it holds until a different edge reaches
    /// `hysteresis` votes. If the current stable edge disappears from the
    /// window (0 votes), hysteresis is relaxed — any edge with `>= majority`
    /// takes over immediately.
    fn vote(
        &self,
        current_stable: Option<u64>,
        majority: usize,
        hysteresis: usize,
    ) -> (Option<u64>, u8, u8) {
        let mut counts: HashMap<u64, u8> = HashMap::new();
        for uid in self.buffer.iter().flatten() {
            *counts.entry(*uid).or_default() += 1;
        }
        let unique_count = counts.len() as u8;

        if counts.is_empty() {
            return (None, 0, 0);
        }

        let (&top_edge, &top_count) = counts.iter().max_by_key(|(_, c)| *c).unwrap();

        let cur_count = current_stable
            .and_then(|s| counts.get(&s).copied())
            .unwrap_or(0);

        if let Some(cur) = current_stable {
            if cur_count == 0 {
                // Old stable gone — fall back to simple majority.
                if top_count >= majority as u8 {
                    return (Some(top_edge), top_count, unique_count);
                }
                return (None, top_count, unique_count);
            }
            if top_edge == cur {
                if top_count >= majority as u8 {
                    return (Some(cur), top_count, unique_count);
                }
                return (None, top_count, unique_count);
            }
            // Challenger must meet hysteresis threshold to unseat the incumbent.
            if top_count >= hysteresis as u8 {
                return (Some(top_edge), top_count, unique_count);
            }
            // Incumbent holds regardless of own vote count (as long as >0).
            return (Some(cur), cur_count, unique_count);
        }

        if top_count >= majority as u8 {
            (Some(top_edge), top_count, unique_count)
        } else {
            (None, top_count, unique_count)
        }
    }
}

fn epoch_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64
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
// Worker communication types
// ---------------------------------------------------------------------------

struct RouteRequest {
    truck_x: f64,
    truck_z: f64,
    truck_heading: f64,
    goal_uid: u64,
}

struct RouteResult {
    goal_uid: u64,
    success: bool,
    waypoints: Vec<[f64; 2]>,
    waypoint_count: usize,
    distance_m: f64,
    plan_ms: u64,
    result_kind: String,
    error_detail: String,
    route_node_ids: Vec<u64>,
    snap_dist_m: f64,
    heading_filter_applied: bool,
    snap_method: String,
}

// ---------------------------------------------------------------------------
// Plugin struct
// ---------------------------------------------------------------------------

pub struct RouterPlugin {
    goal_uid: u64,
    last_seen_goal_str: String,
    graph_path: PathBuf,
    graph: Option<Arc<RouterGraph>>,
    active: bool,
    pending_request: bool,
    request_tx: Option<Sender<RouteRequest>>,
    result_rx: Option<std::sync::Mutex<Receiver<RouteResult>>>,
    worker_handle: Option<JoinHandle<()>>,
    // ---- Diagnostic state ----
    last_planning_result: String,
    last_planning_error_detail: String,
    last_planning_duration_ms: u64,
    waypoint_count: u32,
    path_total_distance_m: f64,
    // ---- Phase 6.5q: Snap diagnostic ----
    last_snap_dist_m: f64,
    last_snap_heading_filter_applied: bool,
    last_snap_method: String,
    // ---- Phase 6.5q: Off-route auto-replan ----
    current_route_node_ids: HashSet<u64>,
    auto_replan_count: u32,
    last_auto_replan_at_ms: u64,
    last_replan_reason: String,
    last_replan_snap_pos: Option<(f64, f64)>,
    // ---- Phase 6.5t: Snap sliding-window stabilitisation ----
    snap_window: SnapWindow,
    stable_snap_edge_id: Option<u64>,
    snap_stability: u8,
    snap_window_unique_edges: u8,
    snap_last_change_at_ms: u64,
    last_autopilot_state: String,
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
            last_snap_dist_m: 0.0,
            last_snap_method: String::new(),
            last_snap_heading_filter_applied: false,
            current_route_node_ids: HashSet::new(),
            auto_replan_count: 0,
            last_auto_replan_at_ms: 0,
            last_replan_reason: String::new(),
            last_replan_snap_pos: None,
            snap_window: SnapWindow::new(SNAP_WINDOW_SIZE),
            stable_snap_edge_id: None,
            snap_stability: 0,
            snap_window_unique_edges: 0,
            snap_last_change_at_ms: 0,
            last_autopilot_state: String::new(),
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

    fn reset_snap_window(&mut self) {
        self.snap_window.clear();
        self.stable_snap_edge_id = None;
        self.snap_stability = 0;
        self.snap_window_unique_edges = 0;
    }

    fn send_route_request(
        &mut self,
        pos_x: f64,
        pos_z: f64,
        truck_heading: f64,
        ctx: &PluginContext,
    ) {
        if let Some(chan) = &self.request_tx {
            ctx.blackboard
                .set("router.last_planning_attempt_at", epoch_ms().to_string());
            let _ = chan.send(RouteRequest {
                truck_x: pos_x,
                truck_z: pos_z,
                truck_heading,
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
                route_node_ids: vec![],
                snap_dist_m: 0.0,
                heading_filter_applied: false,
                snap_method: "none".to_string(),
            });
            continue;
        }

        // Snap truck position to the nearest graph node.
        // First try edge-snap (projects truck onto nearest road segment, then picks
        // the direction-aligned endpoint); fall back to node-snap for sparse areas.
        let edge_snap = graph.find_nearest_on_edge(
            req.truck_x,
            req.truck_z,
            req.truck_heading,
            EDGE_SNAP_RADIUS_M,
        );
        let (snap_result, snap_method) = match edge_snap {
            Some(r) => (Some(r), "edge"),
            None => (
                graph.find_nearest_with_heading(
                    req.truck_x,
                    req.truck_z,
                    req.truck_heading,
                    SNAP_RADIUS_M,
                ),
                "node",
            ),
        };
        let (start_uid, snap_dist_m, heading_filter_applied) = match snap_result {
            Some(r) => r,
            None => {
                let _ = res_tx.send(RouteResult {
                    goal_uid: req.goal_uid,
                    success: false,
                    waypoints: vec![],
                    waypoint_count: 0,
                    distance_m: 0.0,
                    plan_ms: t_start.elapsed().as_millis() as u64,
                    result_kind: "start_node_unknown".to_string(),
                    error_detail: format!(
                        "Start position ({:.1}, {:.1}) has no nearby graph edge within {}m or node within {}m",
                        req.truck_x, req.truck_z, EDGE_SNAP_RADIUS_M, SNAP_RADIUS_M,
                    ),
                    route_node_ids: vec![],
                    snap_dist_m: 0.0,
                    heading_filter_applied: false,
                    snap_method: "none".to_string(),
                });
                continue;
            }
        };

        tracing::info!(
            "[router-worker] A* start={} goal={}",
            start_uid,
            req.goal_uid
        );

        match graph.plan(start_uid, req.goal_uid) {
            Some((path, total_dist)) => {
                let route_node_ids = path.clone();
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
                    route_node_ids,
                    snap_dist_m,
                    heading_filter_applied,
                    snap_method: snap_method.to_string(),
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
                    route_node_ids: vec![],
                    snap_dist_m: 0.0,
                    heading_filter_applied: false,
                    snap_method: snap_method.to_string(),
                });
            }
        }
    }
    tracing::info!("[router-worker] thread exiting");
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

        let graph = if let Some(shared) = &ctx.graph {
            tracing::info!(
                "[router] using shared graph: {} nodes / {} edges",
                shared.nodes.len(),
                shared.edges.len()
            );
            ctx.blackboard
                .set("router.graph_node_count", shared.nodes.len().to_string());
            Some(Arc::clone(shared))
        } else {
            self.graph_path = ctx
                .blackboard
                .get("router.graph_path")
                .map(PathBuf::from)
                .unwrap_or_else(|| PathBuf::from(DEFAULT_GRAPH_PATH));

            match std::fs::read_to_string(&self.graph_path) {
                Ok(data) => match serde_json::from_str::<GraphFile>(&data) {
                    Ok(g) => {
                        let nodes = g.nodes.iter().map(|n| (n.uid, n.x, n.z)).collect();
                        let edges = g
                            .edges
                            .iter()
                            .map(|e| (e.from, e.to, e.distance_m))
                            .collect();
                        tracing::info!(
                            "[router] loaded {} nodes / {} edges from {:?}",
                            g.nodes.len(),
                            g.edges.len(),
                            self.graph_path
                        );
                        ctx.blackboard
                            .set("router.graph_node_count", g.nodes.len().to_string());
                        Some(Arc::new(RouterGraph::new(nodes, edges)))
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
        ctx.blackboard.set("router.last_snap_dist", "0");
        ctx.blackboard
            .set("router.last_snap_heading_filter_applied", "false");
        ctx.blackboard.set("router.snap_method", "");
        ctx.blackboard.set("router.auto_replan_count", "0");
        ctx.blackboard.set("router.auto_replan_triggered_at", "");
        ctx.blackboard.set("router.last_replan_reason", "");
        ctx.blackboard.set("router.snap_stable_edge_id", "");
        ctx.blackboard.set("router.snap_stability", "0");
        ctx.blackboard.set("router.snap_window_unique_edges", "0");
        ctx.blackboard.set("router.snap_last_change_at", "");
        self.reset_snap_window();
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
        // ── 0. Phase 6.5q.1: consume synchronous replan from state machine ──
        if ctx.blackboard.get("router.sync_replan_done").as_deref() == Some("true") {
            ctx.blackboard.remove("router.sync_replan_done");
            self.pending_request = false;
            if let Some(ref lock) = ctx.route_node_ids {
                self.current_route_node_ids = lock.read().unwrap().clone();
            }
            // The engage-time synchronous replan (state_machine.rs) just published
            // `router.route_node_ids` for the truck's snapped start position. Drain
            // any in-flight worker result so the Step-1 poll below cannot overwrite
            // that authoritative route in the same tick with a path snapped from a
            // different start (same goal_uid passes the guard). Stays consistent.
            if let Some(rx) = &self.result_rx {
                while rx.lock().unwrap().try_recv().is_ok() {}
            }
        }

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
                                self.last_snap_dist_m = result.snap_dist_m;
                                self.last_snap_heading_filter_applied =
                                    result.heading_filter_applied;
                                self.last_snap_method = result.snap_method.clone();
                                self.current_route_node_ids =
                                    result.route_node_ids.iter().copied().collect();
                                if let Some(ref lock) = ctx.route_node_ids {
                                    *lock.write().unwrap() = self.current_route_node_ids.clone();
                                }
                                match serde_json::to_string(&result.route_node_ids) {
                                    Ok(route_json) => {
                                        ctx.blackboard.set("router.route_node_ids", &route_json);
                                    }
                                    Err(e) => {
                                        tracing::warn!(
                                            "[router] failed to serialize route_node_ids: {e}"
                                        );
                                    }
                                }
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
                        ctx.blackboard.remove("router.route_node_ids");
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
                self.current_route_node_ids.clear();
                if let Some(ref lock) = ctx.route_node_ids {
                    lock.write().unwrap().clear();
                }
                ctx.blackboard.remove("router.route_node_ids");
                self.auto_replan_count = 0;
                self.last_replan_snap_pos = None;
                self.last_auto_replan_at_ms = 0;
                self.last_replan_reason = String::new();
                self.reset_snap_window();
                ctx.blackboard.set("router.current_goal_uid", "");
                tracing::info!("[router] goal cleared");
            } else {
                match goal_str.trim().parse::<u64>() {
                    Ok(uid) => {
                        self.goal_uid = uid;
                        self.auto_replan_count = 0;
                        self.last_replan_snap_pos = None;
                        self.last_auto_replan_at_ms = 0;
                        self.last_replan_reason = String::new();
                        self.reset_snap_window();
                        ctx.blackboard.set("router.current_goal_uid", &goal_str);
                        tracing::info!("[router] new goal received: uid={}", uid);
                        // Submit immediately for fast feedback.
                        let (pos_x, pos_z) = telemetry
                            .map(|t| (t.position[0], t.position[2]))
                            .unwrap_or((0.0, 0.0));
                        let truck_heading = telemetry.map(|t| t.heading).unwrap_or(0.0);
                        self.send_route_request(pos_x, pos_z, truck_heading, ctx);
                    }
                    Err(_) => {
                        self.goal_uid = 0;
                        self.last_planning_result = "uid_parse_error".to_string();
                        self.last_planning_error_detail =
                            format!("UID parse error: could not parse '{}' as u64", goal_str);
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

        // ── 2.3. Per-tick snap → sliding-window vote (Phase 6.5t) ──────────────
        if let (Some(tel), Some(graph)) = (telemetry, self.graph.as_ref()) {
            let snap = graph
                .find_nearest_with_heading(
                    tel.position[0],
                    tel.position[2],
                    tel.heading,
                    OFF_ROUTE_DETECT_RADIUS_M,
                )
                .map(|(uid, _, _)| uid);
            self.snap_window.push_snap(snap);
        }

        let now_ms = epoch_ms();
        let (stable_edge, stability, unique) = self.snap_window.vote(
            self.stable_snap_edge_id,
            SNAP_MAJORITY_THRESHOLD,
            SNAP_HYSTERESIS_THRESHOLD,
        );
        if stable_edge != self.stable_snap_edge_id {
            self.stable_snap_edge_id = stable_edge;
            self.snap_last_change_at_ms = now_ms;
        }
        self.snap_stability = stability;
        self.snap_window_unique_edges = unique;

        // ── 2.4. Autopilot state-change → reset snap window ───────────────────
        let ap_state = ctx.blackboard.get("autopilot.state").unwrap_or_default();
        if ap_state != self.last_autopilot_state {
            self.last_autopilot_state = ap_state;
            self.reset_snap_window();
        }

        // ── 2.5. Off-route auto-replan check (Phase 6.5q + 6.5s) ───────────────
        if self.goal_uid != 0 && !self.pending_request && !self.current_route_node_ids.is_empty() {
            if let Some(tel) = telemetry {
                let pos_x = tel.position[0];
                let pos_z = tel.position[2];
                let truck_on_route = self
                    .graph
                    .as_ref()
                    .and_then(|g| g.find_nearest_geometric(pos_x, pos_z, OFF_ROUTE_DETECT_RADIUS_M))
                    .map(|(uid, _)| self.current_route_node_ids.contains(&uid))
                    .unwrap_or(true);

                let heading_stage = ctx
                    .blackboard
                    .get("state.heading_stage")
                    .unwrap_or_default();
                let heading_replan = heading_stage == "AutoReplan";

                let (trigger, reason) = if !truck_on_route {
                    (true, "off_route")
                } else if heading_replan {
                    (true, "heading_stage")
                } else {
                    (false, "")
                };

                if trigger {
                    let now_ms = epoch_ms();
                    let rate_ok = now_ms.saturating_sub(self.last_auto_replan_at_ms) > 5_000;
                    let hysteresis_ok = match self.last_replan_snap_pos {
                        None => true,
                        Some((lx, lz)) => {
                            let dx = pos_x - lx;
                            let dz = pos_z - lz;
                            dx * dx + dz * dz > 50.0 * 50.0
                        }
                    };

                    if self.auto_replan_count < 3 && rate_ok && hysteresis_ok {
                        tracing::info!(
                            "[router] auto-replanning (count={}, reason={})",
                            self.auto_replan_count + 1,
                            reason
                        );
                        self.auto_replan_count += 1;
                        self.last_auto_replan_at_ms = now_ms;
                        self.last_replan_snap_pos = Some((pos_x, pos_z));
                        self.last_replan_reason = reason.to_string();
                        self.send_route_request(pos_x, pos_z, tel.heading, ctx);
                        ctx.blackboard
                            .set("router.auto_replan_triggered_at", now_ms.to_string());
                        ctx.blackboard.set(
                            "router.auto_replan_count",
                            self.auto_replan_count.to_string(),
                        );
                        ctx.blackboard.set("router.last_replan_reason", reason);
                    } else if self.auto_replan_count >= 3 {
                        ctx.blackboard.set("state.precondition_route_ok", "false");
                        tracing::warn!(
                            "[router] max replans ({}) reached, reason={}",
                            self.auto_replan_count,
                            reason
                        );
                    }
                }
            }
        }

        // ── 3. Periodic replan (PhaseA cadence, skip if request in flight) ────
        if ctx.is_replan_tick() && !self.pending_request && self.goal_uid != 0 {
            let (pos_x, pos_z) = telemetry
                .map(|t| (t.position[0], t.position[2]))
                .unwrap_or((0.0, 0.0));
            let truck_heading = telemetry.map(|t| t.heading).unwrap_or(0.0);
            self.send_route_request(pos_x, pos_z, truck_heading, ctx);
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
        ctx.blackboard.set(
            "router.last_snap_dist",
            format!("{:.1}", self.last_snap_dist_m),
        );
        ctx.blackboard.set(
            "router.last_snap_heading_filter_applied",
            self.last_snap_heading_filter_applied.to_string(),
        );
        ctx.blackboard
            .set("router.snap_method", &self.last_snap_method);
        ctx.blackboard.set(
            "router.auto_replan_count",
            self.auto_replan_count.to_string(),
        );
        ctx.blackboard
            .set("router.last_replan_reason", &self.last_replan_reason);
        // Phase 6.5t: Snap sliding-window diagnostics
        ctx.blackboard.set(
            "router.snap_stable_edge_id",
            self.stable_snap_edge_id
                .map_or(String::new(), |uid| uid.to_string()),
        );
        ctx.blackboard
            .set("router.snap_stability", self.snap_stability.to_string());
        ctx.blackboard.set(
            "router.snap_window_unique_edges",
            self.snap_window_unique_edges.to_string(),
        );
        if self.snap_last_change_at_ms > 0 {
            ctx.blackboard.set(
                "router.snap_last_change_at",
                self.snap_last_change_at_ms.to_string(),
            );
        } else {
            ctx.blackboard.set("router.snap_last_change_at", "");
        }
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
            nav_distance_m: -1.0,
            nav_time_s: -1.0,
        }
    }

    /// Build a RouterPlugin with a pre-built graph and a live worker thread.
    /// Used by tests that exercise the async routing path.
    fn plugin_with_worker(nodes: NodeList, edges: EdgeList) -> RouterPlugin {
        let graph = Arc::new(RouterGraph::new(nodes, edges));
        let mut p = RouterPlugin::default();
        p.spawn_worker(graph);
        p
    }

    // ---- A* unit tests (call plan() directly, no thread) -------------------

    #[test]
    fn finds_direct_route() {
        let (n, e) = simple_graph();
        let g = RouterGraph::new(n, e);
        let (path, dist) = g.plan(1, 3).unwrap();
        assert_eq!(path, vec![1, 2, 3]);
        assert!((dist - 200.0).abs() < 0.001, "dist={dist}");
    }

    #[test]
    fn unreachable_returns_none() {
        let n: NodeList = vec![(1, 0.0, 0.0), (2, 100.0, 0.0)];
        let e: EdgeList = vec![];
        let g = RouterGraph::new(n, e);
        assert!(g.plan(1, 2).is_none());
    }

    #[test]
    fn start_equals_goal() {
        let (n, e) = simple_graph();
        let g = RouterGraph::new(n, e);
        let (path, dist) = g.plan(1, 1).unwrap();
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
        assert!(
            p.pending_request,
            "request should be pending after first tick"
        );

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

    // ── router.route_node_ids publishing tests ──────────────────────────────────

    /// Build a graph where the truck starts at node 1 (0, 0) with heading 0.5
    /// (ETS2 South = +z direction). The edge 1→2 runs along the +z axis so
    /// edge-snap picks node 1 as start. Path 1→2→3 = three nodes.
    fn south_graph() -> (Vec<(u64, f64, f64)>, Vec<(u64, u64, f64)>) {
        (
            vec![(1, 0.0, 0.0), (2, 0.0, 100.0), (3, 0.0, 200.0)],
            vec![(1, 2, 100.0), (2, 3, 100.0)],
        )
    }

    /// After a successful plan the blackboard key `router.route_node_ids` must:
    /// 1. Be present (non-None) — the fix publishes it in the tick() success block.
    /// 2. Deserialise to a Vec<u64>.
    /// 3. Contain the goal UID as the last element (direction Start→Goal).
    #[test]
    fn route_node_ids_published_after_successful_plan() {
        let (n, e) = south_graph();
        let mut p = plugin_with_worker(n, e);
        p.goal_uid = 3;

        let bb = SharedBlackboard::new();
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseA)
            .with_tick_count(50); // replan tick
        let mut out = ControlOutput::default();
        let t = fake_telemetry_at(0.0, 0.0); // truck at node 1 position

        // First tick: submits route request.
        p.tick(Some(&t), &mut out, &ctx);
        assert!(
            p.pending_request,
            "request should be pending after first tick"
        );

        // Let the worker complete A*.
        std::thread::sleep(Duration::from_millis(300));

        // Second tick: polls result, publishes to blackboard.
        p.tick(Some(&t), &mut out, &ctx);

        assert_eq!(
            bb.get("router.active").as_deref(),
            Some("true"),
            "route must be active after successful plan"
        );

        // Key must be present.
        let raw = bb
            .get("router.route_node_ids")
            .expect("router.route_node_ids must be set after successful plan");

        // Must parse as a JSON array of u64.
        let ids: Vec<u64> =
            serde_json::from_str(&raw).expect("route_node_ids must be valid JSON Vec<u64>");

        assert!(
            !ids.is_empty(),
            "route_node_ids must not be empty after a successful plan"
        );

        // The final element must be the goal UID — this verifies Start→Goal order.
        assert_eq!(
            *ids.last().unwrap(),
            3u64,
            "last element of route_node_ids must be goal UID 3 (Start→Goal order), got {:?}",
            ids
        );

        // The first element must NOT be the goal — path must not be reversed.
        assert_ne!(
            ids[0], 3u64,
            "first element must not be the goal; route would be reversed: {:?}",
            ids
        );

        // Consecutive pairs must be monotonically increasing in z (north→south graph).
        // This verifies that the order is travel-direction (Start→Goal), not reversed.
        for window in ids.windows(2) {
            let a = window[0];
            let b = window[1];
            // Higher-UID nodes are further south in this graph, and all edges go south.
            assert!(
                a < b,
                "node IDs must be in Start→Goal order (a={a} < b={b}): {:?}",
                ids
            );
        }
    }

    /// Edge case: when the worker returns success=false, the key must be
    /// absent (removed) so the lane-keeper cannot read stale node IDs.
    #[test]
    fn route_node_ids_absent_after_failed_plan() {
        let (n, e) = south_graph();
        let mut p = plugin_with_worker(n, e);

        let bb = SharedBlackboard::new();
        // Pre-populate with stale data to prove the remove() actually fires.
        bb.set("router.route_node_ids", "[1,2,3]");

        bb.set("router.goal_uid", "999"); // UID not in graph → failure
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseC)
            .with_tick_count(7);
        let mut out = ControlOutput::default();

        // First tick: detects new goal 999, submits.
        p.tick(None, &mut out, &ctx);
        assert!(p.pending_request);

        // Worker quickly responds uid_not_in_graph.
        std::thread::sleep(Duration::from_millis(200));

        // Second tick: receives failure result.
        p.tick(None, &mut out, &ctx);

        assert_eq!(bb.get("router.active").as_deref(), Some("false"));
        assert_eq!(
            bb.get("router.last_planning_result").as_deref(),
            Some("uid_not_in_graph")
        );
        assert!(
            bb.get("router.route_node_ids").is_none(),
            "router.route_node_ids must be removed (not stale) after a failed plan"
        );
    }

    /// Edge case: when the goal is cleared (empty string), the key must be
    /// removed so downstream consumers see no stale route.
    #[test]
    fn route_node_ids_absent_after_goal_cleared() {
        let (n, e) = south_graph();
        let mut p = plugin_with_worker(n, e);

        let bb = SharedBlackboard::new();
        // Pre-load a route so there is something to clear.
        bb.set("router.route_node_ids", "[1,2,3]");
        // Simulate plugin having accepted goal 3 previously.
        p.goal_uid = 3;
        p.last_seen_goal_str = "3".to_string();

        // Now clear the goal.
        bb.set("router.goal_uid", "");
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseC)
            .with_tick_count(1);
        let mut out = ControlOutput::default();

        p.tick(None, &mut out, &ctx);

        assert_eq!(
            bb.get("router.route_node_ids"),
            None,
            "router.route_node_ids must be removed when goal is cleared"
        );
        assert_eq!(p.goal_uid, 0, "goal_uid must be reset to 0");
    }

    /// Serialisation invariant (no thread needed): serde_json::to_string on a
    /// known Vec<u64> must produce exactly the expected JSON, matching what
    /// tick() writes to the blackboard.
    #[test]
    fn route_node_ids_serialisation_preserves_order_and_precision() {
        let ids: Vec<u64> = vec![10, 20, 30];
        let json = serde_json::to_string(&ids).unwrap();
        let round_tripped: Vec<u64> = serde_json::from_str(&json).unwrap();
        assert_eq!(
            round_tripped,
            vec![10u64, 20u64, 30u64],
            "round-trip must preserve exact order and values"
        );
        // Large ETS2 UIDs above 2^53 must not lose precision.
        const HAMBURG: u64 = 6_526_933_291_294_064_640;
        let large_ids: Vec<u64> = vec![HAMBURG, HAMBURG + 1];
        let large_json = serde_json::to_string(&large_ids).unwrap();
        let large_rt: Vec<u64> = serde_json::from_str(&large_json).unwrap();
        assert_eq!(
            large_rt[0], HAMBURG,
            "large UID must survive JSON round-trip without precision loss"
        );
        assert_eq!(large_rt[1], HAMBURG + 1);
    }

    // ── Phase 6.5q: Snap und Auto-Replan Tests ──────────────────────────────────

    /// Graph with two parallel anti-parallel lanes for heading filter tests.
    fn dual_lane_graph() -> RouterGraph {
        let nodes: Vec<(u64, f64, f64)> = vec![
            (10, 0.0, 0.0),   // forward start
            (11, 100.0, 0.0), // forward end
            (20, 100.0, 5.0), // backward start (close to forward end)
            (21, 0.0, 5.0),   // backward end
        ];
        let edges: Vec<(u64, u64, f64)> = vec![
            (10, 11, 100.0), // forward: east (+X)
            (20, 21, 100.0), // backward: west (-X)
        ];
        RouterGraph::new(nodes, edges)
    }

    #[test]
    fn heading_filter_accepts_forward_edge() {
        let graph = dual_lane_graph();
        // Truck at (2,0), heading 0.75 (ETS2 East): hx=1,hz=0
        // Node 10 at (0,0) dist=2: edge 10→11 dir=(1,0), dot=1.0 ≥ 0.5 → accept
        let result = graph.find_nearest_with_heading(2.0, 0.0, 0.75, 20.0);
        assert!(result.is_some());
        let (uid, _, filter_used) = result.unwrap();
        assert_eq!(uid, 10, "should snap to forward node 10, got {uid}");
        assert!(filter_used, "heading filter should have found a candidate");
    }

    #[test]
    fn heading_filter_falls_back_when_no_compatible_candidate() {
        let graph = dual_lane_graph();
        // Truck at (98,5), heading 0.75 (ETS2 East): only node 20 in range (dist≈2)
        // Node 20's edge 20→21 dir=(-1,0), dot((-1,0),(1,0))=-1 < 0.5 → filter fails → fallback
        let result = graph.find_nearest_with_heading(98.0, 5.0, 0.75, 20.0);
        assert!(
            result.is_some(),
            "fallback should return a result when filter has no candidates"
        );
        let (_, _, filter_used) = result.unwrap();
        assert!(
            !filter_used,
            "fallback path should report heading_filter_applied=false"
        );
    }

    #[test]
    fn heading_filter_boundary_dot_at_threshold() {
        // Edge at exactly 60° from truck heading: dot = cos(60°) = 0.5 → accepted
        let nodes: Vec<(u64, f64, f64)> = vec![(1, 0.0, 0.0), (2, 50.0, 86.6)];
        let edges: Vec<(u64, u64, f64)> = vec![(1, 2, 100.0)];
        let graph = RouterGraph::new(nodes, edges);
        // heading 0.75 (ETS2 East), edge dir ≈ (0.5, 0.866) normalized, dot with (1,0) ≈ 0.5
        let result = graph.find_nearest_with_heading(0.0, 0.0, 0.75, 20.0);
        assert!(result.is_some());
        let (uid, _, filter_used) = result.unwrap();
        assert_eq!(uid, 1, "boundary dot≈0.5 should be accepted");
        assert!(
            filter_used,
            "boundary case should be accepted by heading filter, not fallback"
        );
    }

    #[test]
    fn geometric_snap_returns_none_beyond_distance_limit() {
        let nodes: Vec<(u64, f64, f64)> = vec![(1, 0.0, 0.0), (2, 200.0, 0.0)];
        let edges: Vec<(u64, u64, f64)> = vec![(1, 2, 200.0)];
        let graph = RouterGraph::new(nodes, edges);
        // Truck 25m from node 1, max_dist=20m → None
        assert!(graph.find_nearest_geometric(25.0, 0.0, 20.0).is_none());
    }

    #[test]
    fn geometric_snap_returns_node_within_limit() {
        let nodes: Vec<(u64, f64, f64)> = vec![(1, 0.0, 0.0), (2, 200.0, 0.0)];
        let edges: Vec<(u64, u64, f64)> = vec![(1, 2, 200.0)];
        let graph = RouterGraph::new(nodes, edges);
        let result = graph.find_nearest_geometric(10.0, 0.0, 20.0);
        assert!(result.is_some());
        let (uid, dist) = result.unwrap();
        assert_eq!(uid, 1);
        assert!((dist - 10.0).abs() < 0.01, "dist={dist}");
    }

    #[test]
    fn auto_replan_triggers_when_truck_off_route() {
        let (n, e) = simple_graph();
        let mut p = plugin_with_worker(n, e);
        p.goal_uid = 3;
        // Simulate route only covering nodes 1 and 2
        p.current_route_node_ids = vec![1u64, 2u64].into_iter().collect();

        let bb = SharedBlackboard::new();
        // PhaseC so periodic replan doesn't fire
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseC)
            .with_tick_count(1);
        let mut out = ControlOutput::default();
        // Truck at node 3 (200,0) — not in route {1,2}
        let t = fake_telemetry_at(200.0, 0.0);

        p.tick(Some(&t), &mut out, &ctx);
        assert!(p.pending_request, "off-route should trigger auto-replan");
        assert_eq!(p.auto_replan_count, 1);
        assert_eq!(p.last_replan_reason, "off_route");
    }

    #[test]
    fn auto_replan_does_not_trigger_when_on_route() {
        let (n, e) = simple_graph();
        let mut p = plugin_with_worker(n, e);
        p.goal_uid = 3;
        // Truck at node 1 (0,0), route includes node 1 → on-route
        p.current_route_node_ids = vec![1u64, 2u64, 3u64].into_iter().collect();

        let bb = SharedBlackboard::new();
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseC)
            .with_tick_count(1);
        let mut out = ControlOutput::default();
        let t = fake_telemetry_at(0.0, 0.0); // at node 1, dist=0 < 20m

        p.tick(Some(&t), &mut out, &ctx);
        assert!(!p.pending_request, "on-route should not trigger replan");
        assert_eq!(p.auto_replan_count, 0);
    }

    #[test]
    fn auto_replan_rate_limited() {
        let (n, e) = simple_graph();
        let mut p = plugin_with_worker(n, e);
        p.goal_uid = 3;
        p.current_route_node_ids = vec![1u64].into_iter().collect();
        // Simulate last replan just happened
        p.last_auto_replan_at_ms = epoch_ms();
        p.auto_replan_count = 1;

        let bb = SharedBlackboard::new();
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseC)
            .with_tick_count(1);
        let mut out = ControlOutput::default();
        // Truck at node 2 (100,0) — not in route {1}
        let t = fake_telemetry_at(100.0, 0.0);

        p.tick(Some(&t), &mut out, &ctx);
        assert!(!p.pending_request, "rate limit should block rapid replan");
        assert_eq!(p.auto_replan_count, 1, "count must not increase");
    }

    #[test]
    fn auto_replan_stops_at_max_three_and_sets_precondition() {
        let (n, e) = simple_graph();
        let mut p = plugin_with_worker(n, e);
        p.goal_uid = 3;
        p.current_route_node_ids = vec![1u64].into_iter().collect();
        p.auto_replan_count = 3;
        p.last_auto_replan_at_ms = 0; // rate limit expired

        let bb = SharedBlackboard::new();
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseC)
            .with_tick_count(1);
        let mut out = ControlOutput::default();
        let t = fake_telemetry_at(200.0, 0.0); // off-route

        p.tick(Some(&t), &mut out, &ctx);
        assert!(!p.pending_request, "no 4th replan");
        assert_eq!(p.auto_replan_count, 3);
        assert_eq!(
            bb.get("state.precondition_route_ok").as_deref(),
            Some("false"),
            "must set state.precondition_route_ok=false after max replans"
        );
    }

    // ── Phase 6.5s: heading-stage-triggered auto-replan tests ────────────────

    #[test]
    fn heading_stage_triggers_replan() {
        let (n, e) = simple_graph();
        let mut p = plugin_with_worker(n, e);
        p.goal_uid = 3;
        p.current_route_node_ids = vec![1u64, 2u64, 3u64].into_iter().collect();

        let bb = SharedBlackboard::new();
        bb.set("state.heading_stage", "AutoReplan");
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseC)
            .with_tick_count(1);
        let mut out = ControlOutput::default();
        let t = fake_telemetry_at(0.0, 0.0);

        p.tick(Some(&t), &mut out, &ctx);
        assert!(
            p.pending_request,
            "heading stage AutoReplan should trigger replan"
        );
        assert_eq!(p.auto_replan_count, 1);
        assert_eq!(p.last_replan_reason, "heading_stage");
    }

    #[test]
    fn heading_replan_respects_rate_limit() {
        let (n, e) = simple_graph();
        let mut p = plugin_with_worker(n, e);
        p.goal_uid = 3;
        p.current_route_node_ids = vec![1u64, 2u64, 3u64].into_iter().collect();
        p.last_auto_replan_at_ms = epoch_ms();
        p.auto_replan_count = 1;

        let bb = SharedBlackboard::new();
        bb.set("state.heading_stage", "AutoReplan");
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseC)
            .with_tick_count(1);
        let mut out = ControlOutput::default();
        let t = fake_telemetry_at(0.0, 0.0);

        p.tick(Some(&t), &mut out, &ctx);
        assert!(!p.pending_request, "rate limit should block heading replan");
        assert_eq!(p.auto_replan_count, 1);
    }

    #[test]
    fn heading_replan_stops_at_max_three() {
        let (n, e) = simple_graph();
        let mut p = plugin_with_worker(n, e);
        p.goal_uid = 3;
        p.current_route_node_ids = vec![1u64, 2u64, 3u64].into_iter().collect();
        p.auto_replan_count = 3;
        p.last_auto_replan_at_ms = 0;

        let bb = SharedBlackboard::new();
        bb.set("state.heading_stage", "AutoReplan");
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseC)
            .with_tick_count(1);
        let mut out = ControlOutput::default();
        let t = fake_telemetry_at(0.0, 0.0);

        p.tick(Some(&t), &mut out, &ctx);
        assert!(!p.pending_request);
        assert_eq!(p.auto_replan_count, 3);
        assert_eq!(
            bb.get("state.precondition_route_ok").as_deref(),
            Some("false")
        );
    }

    // ── Phase 6.5t: SnapWindow sliding-window tests ───────────────────────

    #[test]
    fn snap_window_empty_returns_no_stable() {
        let w = SnapWindow::new(5);
        let (stable, stability, unique) = w.vote(None, 3, 4);
        assert_eq!(stable, None);
        assert_eq!(stability, 0);
        assert_eq!(unique, 0);
    }

    #[test]
    fn snap_window_all_same_edge_becomes_stable() {
        let mut w = SnapWindow::new(5);
        for _ in 0..5 {
            w.push_snap(Some(42));
        }
        let (stable, stability, unique) = w.vote(None, 3, 4);
        assert_eq!(stable, Some(42));
        assert_eq!(stability, 5);
        assert_eq!(unique, 1);
    }

    #[test]
    fn snap_window_majority_3_of_5() {
        let mut w = SnapWindow::new(5);
        w.push_snap(Some(10));
        w.push_snap(Some(10));
        w.push_snap(Some(10));
        w.push_snap(Some(20));
        w.push_snap(Some(20));
        let (stable, stability, unique) = w.vote(None, 3, 4);
        assert_eq!(stable, Some(10));
        assert_eq!(stability, 3);
        assert_eq!(unique, 2);
    }

    #[test]
    fn snap_window_below_majority_no_stable() {
        let mut w = SnapWindow::new(5);
        w.push_snap(Some(10));
        w.push_snap(Some(10));
        w.push_snap(Some(20));
        w.push_snap(Some(20));
        w.push_snap(Some(30));
        // 2-2-1 split, no edge reaches 3
        let (stable, stability, _unique) = w.vote(None, 3, 4);
        assert_eq!(stable, None);
        assert!(stability <= 2, "top count should be 2, got {stability}");
    }

    #[test]
    fn snap_window_hysteresis_holds_stable() {
        let mut w = SnapWindow::new(5);
        // Establish stable edge A (5/5)
        for _ in 0..5 {
            w.push_snap(Some(10));
        }
        let (stable, _, _) = w.vote(None, 3, 4);
        assert_eq!(stable, Some(10), "first establish stable edge A");

        // Now shift: 3 for B, 2 for A. B=3 < hysteresis=4, A=2 >= majority=3 → hold A
        w.push_snap(Some(20));
        w.push_snap(Some(20));
        w.push_snap(Some(20));
        w.push_snap(Some(10));
        w.push_snap(Some(10));
        // window: [B,B,B,A,A]
        let (stable, stability, _) = w.vote(Some(10), 3, 4);
        assert_eq!(
            stable,
            Some(10),
            "A should hold: B=3 < hysteresis=4, incumbent holds as long as >0 votes"
        );
        assert_eq!(stability, 2, "2 votes for A");
    }

    #[test]
    fn snap_window_hysteresis_switch_when_challenger_reaches_threshold() {
        let mut w = SnapWindow::new(5);
        // Establish stable edge A
        for _ in 0..5 {
            w.push_snap(Some(10));
        }
        let (stable, _, _) = w.vote(None, 3, 4);
        assert_eq!(stable, Some(10));

        // Shift: 4 for B, 1 for A. B=4 >= hysteresis=4 → switch
        w.push_snap(Some(20));
        w.push_snap(Some(20));
        w.push_snap(Some(20));
        w.push_snap(Some(20));
        w.push_snap(Some(10));
        // window: [B,B,B,B,A]
        let (stable, stability, _) = w.vote(Some(10), 3, 4);
        assert_eq!(stable, Some(20), "B should take over: B=4 >= hysteresis=4");
        assert_eq!(stability, 4);
    }

    #[test]
    fn snap_window_old_stable_gone_immediate_switch() {
        let mut w = SnapWindow::new(5);
        // Old stable was A, but all frames are now B
        for _ in 0..5 {
            w.push_snap(Some(20));
        }
        // current_stable=10 is no longer in the window
        let (stable, stability, _) = w.vote(Some(10), 3, 4);
        assert_eq!(
            stable,
            Some(20),
            "old stable gone, new majority B=5 should take over"
        );
        assert_eq!(stability, 5);
    }

    #[test]
    fn snap_window_clear_resets() {
        let mut w = SnapWindow::new(5);
        for _ in 0..5 {
            w.push_snap(Some(42));
        }
        w.clear();
        let (stable, stability, unique) = w.vote(Some(42), 3, 4);
        assert_eq!(stable, None);
        assert_eq!(stability, 0);
        assert_eq!(unique, 0);
    }

    #[test]
    fn snap_window_partial_fill_works() {
        let mut w = SnapWindow::new(5);
        // Only 3 frames yet
        w.push_snap(Some(10));
        w.push_snap(Some(10));
        w.push_snap(Some(20));
        let (stable, stability, unique) = w.vote(None, 3, 4);
        assert_eq!(stable, None, "2/3 < majority=3/5");
        assert_eq!(stability, 2);
        assert_eq!(unique, 2);
    }

    #[test]
    fn snap_window_overflow_keeps_last_n() {
        let mut w = SnapWindow::new(5);
        for uid in 1..=10 {
            w.push_snap(Some(uid));
        }
        // Only last 5 entries remain: 6,7,8,9,10
        let (stable, _stability, unique) = w.vote(None, 3, 4);
        assert_eq!(stable, None, "each uid appears once, cannot reach 3");
        assert_eq!(unique, 5);
    }

    #[test]
    fn snap_window_none_entries_not_counted() {
        let mut w = SnapWindow::new(5);
        w.push_snap(Some(10));
        w.push_snap(Some(10));
        w.push_snap(Some(10));
        w.push_snap(None);
        w.push_snap(None);
        let (stable, stability, unique) = w.vote(None, 3, 4);
        assert_eq!(stable, Some(10), "3 out of 3 non-None = unanimous");
        assert_eq!(stability, 3);
        assert_eq!(unique, 1, "only edge 10 appears");
    }

    #[test]
    fn snap_window_hysteresis_incumbent_holds_weak() {
        let mut w = SnapWindow::new(5);
        // Establish A
        for _ in 0..5 {
            w.push_snap(Some(10));
        }
        w.vote(None, 3, 4);

        // Shift: A=2, B=3. B=3 < hysteresis=4, so incumbent A holds.
        w.push_snap(Some(20));
        w.push_snap(Some(20));
        w.push_snap(Some(20));
        w.push_snap(Some(10));
        w.push_snap(Some(10));
        let (stable, stability, _) = w.vote(Some(10), 3, 4);
        assert_eq!(
            stable,
            Some(10),
            "incumbent A holds with only 2 votes: B=3 < hysteresis=4"
        );
        assert_eq!(stability, 2);
    }

    #[test]
    fn snap_window_integration_100_ticks_no_flapping() {
        let mut w = SnapWindow::new(5);
        let mut current_stable: Option<u64> = None;
        let mut change_count = 0u32;
        let mut prev_stable = None;

        // Simulate truck moving along a road with some noise
        let frames: Vec<Option<u64>> = (0..100)
            .map(|i| {
                if i % 10 == 0 || i % 10 == 1 || i % 10 == 2 {
                    Some(10) // 3 frames = majority candidate
                } else if i % 10 == 4 {
                    // Occasional noise
                    if i < 50 {
                        Some(20)
                    } else {
                        Some(30)
                    }
                } else {
                    Some(10)
                }
            })
            .collect();

        for snap in frames {
            w.push_snap(snap);
            let (edge, _, _) = w.vote(current_stable, 3, 4);
            current_stable = edge;
            if edge != prev_stable {
                prev_stable = edge;
                change_count += 1;
            }
        }

        // With hysteresis, the stable edge should flip at most 3 times
        // (None→10, 10→20 noise, 20→30 noise with hysteresis delay)
        assert!(
            change_count <= 5,
            "stable edge changed {} times, should be stable with hysteresis",
            change_count
        );
    }
}
