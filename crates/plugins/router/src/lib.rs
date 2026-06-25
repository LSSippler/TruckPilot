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
use truckpilot_plugin_api::ets2_route::{
    build_router_output_from_node_ids, build_trimmed_ets2_router_output,
    compare_ets2_graph_coord_delta, compare_ets2_graph_distance, decide_ets2_route_progress,
    find_route_start_index_for_truck, repair_ets2_route_gaps, trim_result_for_start_index,
    Ets2RouteProgressDecision, Ets2RouteProgressStatus, Ets2RouteRepairResult,
    Ets2RouteTrimResult,
};
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
    snap_rejected_by_heading: u32,
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
    last_snap_rejected_by_heading: u32,
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
    // ---- Phase 5b: ETS2 in-game route import ----
    ets2_import_active: bool,
    last_ets2_imported_hash: Option<u64>,
    last_ets2_imported_sequence: Option<u32>,
    /// Full matched ETS2 node list (pre-trim); used for live progress re-trim.
    full_imported_route_node_ids: Vec<u64>,
    last_published_start_index: usize,
    ets2_offroute_since_ms: Option<u64>,
    last_ets2_progress_republish_at_ms: u64,
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
            last_snap_rejected_by_heading: 0,
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
            ets2_import_active: false,
            last_ets2_imported_hash: None,
            last_ets2_imported_sequence: None,
            full_imported_route_node_ids: Vec::new(),
            last_published_start_index: 0,
            ets2_offroute_since_ms: None,
            last_ets2_progress_republish_at_ms: 0,
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
        if self.ets2_import_active {
            return;
        }
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

    fn drain_worker_results(&mut self) {
        if let Some(rx) = &self.result_rx {
            while rx.lock().unwrap().try_recv().is_ok() {}
            self.pending_request = false;
        }
    }

    /// Heavy per-tick graph snap / off-route work — skip when idle (Off, no goal, no ETS2 import).
    fn needs_heavy_router_work(&self, ctx: &PluginContext) -> bool {
        self.goal_uid != 0
            || self.ets2_import_active
            || self.pending_request
            || ctx.is_engaged()
            || !self.current_route_node_ids.is_empty()
    }

    fn set_ets2_not_imported(&self, ctx: &PluginContext) {
        ctx.blackboard
            .set("navigation.ets2_route.imported", "false");
        ctx.blackboard.remove("navigation.ets2_route.imported_hash");
        ctx.blackboard.remove("navigation.ets2_route.imported_node_count");
    }

    fn clear_ets2_repair_keys(&self, ctx: &PluginContext) {
        ctx.blackboard.remove("navigation.ets2_route.repair_status");
        ctx.blackboard.remove("navigation.ets2_route.repair_gap_count");
        ctx.blackboard.remove("navigation.ets2_route.repair_success_count");
        ctx.blackboard.remove("navigation.ets2_route.repair_failed_count");
        ctx.blackboard
            .remove("navigation.ets2_route.repair_inserted_node_count");
        ctx.blackboard
            .remove("navigation.ets2_route.repair_first_failed_gap");
        ctx.blackboard.remove("navigation.ets2_route.repair_error");
    }

    fn clear_ets2_coord_delta_keys(&self, ctx: &PluginContext) {
        ctx.blackboard
            .remove("navigation.ets2_route.coord_graph_delta_avg_m");
        ctx.blackboard
            .remove("navigation.ets2_route.coord_graph_delta_max_m");
        ctx.blackboard
            .remove("navigation.ets2_route.coord_graph_delta_count");
    }

    fn publish_ets2_coord_delta_keys(&self, ctx: &PluginContext) {
        let Some(ets2_lock) = ctx.ets2_route.as_ref() else {
            self.clear_ets2_coord_delta_keys(ctx);
            return;
        };
        let Some(graph) = self.graph.as_ref() else {
            self.clear_ets2_coord_delta_keys(ctx);
            return;
        };
        let waypoints = ets2_lock
            .read()
            .ok()
            .map(|g| g.waypoints.clone())
            .unwrap_or_default();
        let delta = compare_ets2_graph_coord_delta(graph, &waypoints);
        if delta.count == 0 {
            self.clear_ets2_coord_delta_keys(ctx);
            return;
        }
        ctx.blackboard.set(
            "navigation.ets2_route.coord_graph_delta_avg_m",
            format!("{:.2}", delta.avg_m),
        );
        ctx.blackboard.set(
            "navigation.ets2_route.coord_graph_delta_max_m",
            format!("{:.2}", delta.max_m),
        );
        ctx.blackboard.set(
            "navigation.ets2_route.coord_graph_delta_count",
            delta.count.to_string(),
        );
    }

    fn clear_ets2_distance_graph_keys(&self, ctx: &PluginContext) {
        ctx.blackboard
            .remove("navigation.ets2_route.distance_graph_total_m");
        ctx.blackboard
            .remove("navigation.ets2_route.distance_first_vs_graph_delta_m");
        ctx.blackboard
            .remove("navigation.ets2_route.distance_graph_ratio");
        ctx.blackboard
            .remove("navigation.ets2_route.distance_graph_status");
    }

    fn publish_ets2_distance_graph_keys(
        &self,
        ctx: &PluginContext,
        route_node_ids: &[u64],
    ) {
        let Some(ets2_lock) = ctx.ets2_route.as_ref() else {
            self.clear_ets2_distance_graph_keys(ctx);
            return;
        };
        let Some(graph) = self.graph.as_ref() else {
            self.clear_ets2_distance_graph_keys(ctx);
            return;
        };
        let waypoints = ets2_lock
            .read()
            .ok()
            .map(|g| g.waypoints.clone())
            .unwrap_or_default();
        let cmp = compare_ets2_graph_distance(graph, route_node_ids, &waypoints);
        if cmp.status.as_str() == "none" {
            self.clear_ets2_distance_graph_keys(ctx);
            return;
        }
        ctx.blackboard.set(
            "navigation.ets2_route.distance_graph_total_m",
            format!("{:.1}", cmp.graph_total_m),
        );
        ctx.blackboard.set(
            "navigation.ets2_route.distance_first_vs_graph_delta_m",
            format!("{:.1}", cmp.first_vs_graph_delta_m),
        );
        ctx.blackboard.set(
            "navigation.ets2_route.distance_graph_ratio",
            format!("{:.4}", cmp.graph_ratio),
        );
        ctx.blackboard.set(
            "navigation.ets2_route.distance_graph_status",
            cmp.status.as_str(),
        );
    }

    fn publish_ets2_repair_keys(&self, ctx: &PluginContext, repair: &Ets2RouteRepairResult) {
        ctx.blackboard.set(
            "navigation.ets2_route.repair_status",
            repair.status.as_str(),
        );
        ctx.blackboard.set(
            "navigation.ets2_route.repair_gap_count",
            repair.gap_count.to_string(),
        );
        ctx.blackboard.set(
            "navigation.ets2_route.repair_success_count",
            repair.success_count.to_string(),
        );
        ctx.blackboard.set(
            "navigation.ets2_route.repair_failed_count",
            repair.failed_count.to_string(),
        );
        ctx.blackboard.set(
            "navigation.ets2_route.repair_inserted_node_count",
            repair.inserted_node_count.to_string(),
        );
        if let Some(ref gap) = repair.first_failed_gap {
            ctx.blackboard.set(
                "navigation.ets2_route.repair_first_failed_gap",
                gap.as_str(),
            );
        } else {
            ctx.blackboard
                .remove("navigation.ets2_route.repair_first_failed_gap");
        }
        if let Some(ref err) = repair.error {
            ctx.blackboard.set("navigation.ets2_route.repair_error", err.as_str());
        } else {
            ctx.blackboard.remove("navigation.ets2_route.repair_error");
        }
    }

    fn clear_ets2_trim_keys(&self, ctx: &PluginContext) {
        ctx.blackboard.remove("navigation.ets2_route.trimmed");
        ctx.blackboard.remove("navigation.ets2_route.trim_start_index");
        ctx.blackboard
            .remove("navigation.ets2_route.trim_original_node_count");
        ctx.blackboard.remove("navigation.ets2_route.trimmed_node_count");
        ctx.blackboard.remove("navigation.ets2_route.snap_dist_m");
        ctx.blackboard.remove("navigation.ets2_route.snap_status");
        ctx.blackboard
            .remove("navigation.ets2_route.snap_heading_delta_deg");
    }

    fn publish_ets2_trim_keys(&self, ctx: &PluginContext, trim: &Ets2RouteTrimResult) {
        ctx.blackboard.set(
            "navigation.ets2_route.trimmed",
            if trim.trimmed { "true" } else { "false" },
        );
        ctx.blackboard.set(
            "navigation.ets2_route.trim_start_index",
            trim.start_index.to_string(),
        );
        ctx.blackboard.set(
            "navigation.ets2_route.trim_original_node_count",
            trim.original_node_count.to_string(),
        );
        ctx.blackboard.set(
            "navigation.ets2_route.trimmed_node_count",
            trim.trimmed_node_count.to_string(),
        );
        ctx.blackboard.set(
            "navigation.ets2_route.snap_dist_m",
            format!("{:.2}", trim.snap_dist_m),
        );
        ctx.blackboard.set(
            "navigation.ets2_route.snap_status",
            trim.snap_status.as_str(),
        );
        if let Some(deg) = trim.snap_heading_delta_deg {
            ctx.blackboard.set(
                "navigation.ets2_route.snap_heading_delta_deg",
                format!("{:.1}", deg),
            );
        } else {
            ctx.blackboard
                .remove("navigation.ets2_route.snap_heading_delta_deg");
        }
    }

    fn clear_ets2_progress_keys(&self, ctx: &PluginContext) {
        ctx.blackboard.remove("navigation.ets2_route.progress_start_index");
        ctx.blackboard
            .remove("navigation.ets2_route.progress_original_node_count");
        ctx.blackboard
            .remove("navigation.ets2_route.progress_remaining_node_count");
        ctx.blackboard.remove("navigation.ets2_route.progress_republished");
        ctx.blackboard.remove("navigation.ets2_route.progress_status");
        ctx.blackboard.remove("navigation.ets2_route.offroute_secs");
    }

    fn clear_ets2_import_progress_state(&mut self) {
        self.full_imported_route_node_ids.clear();
        self.last_published_start_index = 0;
        self.ets2_offroute_since_ms = None;
        self.last_ets2_progress_republish_at_ms = 0;
    }

    fn publish_ets2_progress_keys(
        &self,
        ctx: &PluginContext,
        trim: &Ets2RouteTrimResult,
        decision: &Ets2RouteProgressDecision,
        offroute_secs: f64,
        republished: bool,
    ) {
        let remaining = self
            .full_imported_route_node_ids
            .len()
            .saturating_sub(self.last_published_start_index);
        ctx.blackboard.set(
            "navigation.ets2_route.progress_start_index",
            self.last_published_start_index.to_string(),
        );
        ctx.blackboard.set(
            "navigation.ets2_route.progress_original_node_count",
            self.full_imported_route_node_ids.len().to_string(),
        );
        ctx.blackboard.set(
            "navigation.ets2_route.progress_remaining_node_count",
            remaining.to_string(),
        );
        ctx.blackboard.set(
            "navigation.ets2_route.progress_republished",
            if republished { "true" } else { "false" },
        );
        ctx.blackboard.set(
            "navigation.ets2_route.progress_status",
            decision.progress_status.as_str(),
        );
        ctx.blackboard.set(
            "navigation.ets2_route.offroute_secs",
            format!("{:.2}", offroute_secs),
        );
        self.publish_ets2_trim_keys(ctx, trim);
    }

    fn publish_ets2_router_output(
        &mut self,
        ctx: &PluginContext,
        output: truckpilot_plugin_api::ets2_route::Ets2RouterOutput,
    ) {
        self.waypoint_count = output.waypoints.len() as u32;
        self.path_total_distance_m = output.distance_m;
        self.current_route_node_ids = output.route_node_ids.iter().copied().collect();

        if let Ok(json) = serde_json::to_string(&output.waypoints) {
            ctx.blackboard.set("router.waypoints", &json);
        }
        if let Ok(route_json) = serde_json::to_string(&output.route_node_ids) {
            ctx.blackboard.set("router.route_node_ids", &route_json);
        }
        if let Some(ref lock) = ctx.route_node_ids {
            *lock.write().unwrap() = self.current_route_node_ids.clone();
        }
        ctx.blackboard.set(
            "navigation.ets2_route.imported_node_count",
            output.route_node_ids.len().to_string(),
        );
    }

    fn update_ets2_live_progress(
        &mut self,
        telemetry: Option<&Telemetry>,
        ctx: &PluginContext,
    ) {
        if !self.ets2_import_active || self.full_imported_route_node_ids.is_empty() {
            return;
        }
        let Some(graph) = self.graph.as_ref() else {
            return;
        };

        let (truck_x, truck_z, heading) = Self::truck_pose_for_ets2_trim(telemetry, ctx);
        let now_ms = epoch_ms();

        let trim = match (truck_x, truck_z) {
            (Some(x), Some(z)) => find_route_start_index_for_truck(
                graph,
                &self.full_imported_route_node_ids,
                x,
                z,
                heading,
            ),
            _ => trim_result_for_start_index(
                &self.full_imported_route_node_ids,
                self.last_published_start_index,
                0.0,
                truckpilot_plugin_api::ets2_route::Ets2RouteSnapStatus::NoPosition,
                None,
            ),
        };

        if trim.snap_status == truckpilot_plugin_api::ets2_route::Ets2RouteSnapStatus::TooFar {
            if self.ets2_offroute_since_ms.is_none() {
                self.ets2_offroute_since_ms = Some(now_ms);
            }
        } else {
            self.ets2_offroute_since_ms = None;
        }

        let offroute_secs = self
            .ets2_offroute_since_ms
            .map(|s| now_ms.saturating_sub(s) as f64 / 1000.0)
            .unwrap_or(0.0);

        let decision = decide_ets2_route_progress(
            trim.start_index,
            self.last_published_start_index,
            self.full_imported_route_node_ids.len(),
            &trim,
            offroute_secs,
            now_ms,
            self.last_ets2_progress_republish_at_ms,
        );

        if decision.should_release {
            let release_reason = decision.release_reason.unwrap_or("ets2_off_route");
            let fallback = if release_reason == "too_short" {
                "build_error"
            } else {
                "ets2_off_route"
            };
            self.release_ets2_import(
                telemetry,
                ctx,
                "fallback_astar",
                fallback,
                release_reason,
                &format!("ets2 progress release: {release_reason}"),
            );
            return;
        }

        if decision.should_republish {
            let trimmed = &self.full_imported_route_node_ids[decision.effective_start_index..];
            match build_router_output_from_node_ids(graph, trimmed) {
                Ok(output) => {
                    self.last_published_start_index = decision.effective_start_index;
                    self.last_ets2_progress_republish_at_ms = now_ms;
                    self.publish_ets2_router_output(ctx, output);
                    let publish_trim = trim_result_for_start_index(
                        &self.full_imported_route_node_ids,
                        decision.effective_start_index,
                        trim.snap_dist_m,
                        trim.snap_status,
                        trim.snap_heading_delta_deg,
                    );
                    self.publish_ets2_progress_keys(
                        ctx,
                        &publish_trim,
                        &decision,
                        offroute_secs,
                        true,
                    );
                    tracing::debug!(
                        "[router] ETS2 progress advanced: start={} remaining={}",
                        decision.effective_start_index,
                        self.full_imported_route_node_ids.len()
                            - decision.effective_start_index,
                    );
                }
                Err(e) => {
                    self.release_ets2_import(
                        telemetry,
                        ctx,
                        "fallback_astar",
                        "build_error",
                        "too_short",
                        &e,
                    );
                }
            }
        } else {
            let publish_trim = trim_result_for_start_index(
                &self.full_imported_route_node_ids,
                self.last_published_start_index,
                trim.snap_dist_m,
                trim.snap_status,
                trim.snap_heading_delta_deg,
            );
            self.publish_ets2_progress_keys(
                ctx,
                &publish_trim,
                &decision,
                offroute_secs,
                false,
            );
        }
    }

    fn truck_pose_for_ets2_trim(
        telemetry: Option<&Telemetry>,
        ctx: &PluginContext,
    ) -> (Option<f64>, Option<f64>, Option<f64>) {
        if let Some(tel) = telemetry {
            return (
                Some(tel.position[0]),
                Some(tel.position[2]),
                Some(tel.heading),
            );
        }
        (
            ctx.blackboard.get_f64("telemetry.position_x"),
            ctx.blackboard.get_f64("telemetry.position_z"),
            ctx.blackboard.get_f64("telemetry.heading"),
        )
    }

    fn publish_ets2_lifecycle_idle(&self, ctx: &PluginContext) {
        ctx.blackboard
            .set("navigation.ets2_route.import_state", "inactive");
        ctx.blackboard.remove("navigation.ets2_route.fallback_reason");
    }

    fn publish_ets2_lifecycle_active(&self, ctx: &PluginContext) {
        ctx.blackboard
            .set("navigation.ets2_route.import_state", "active");
        ctx.blackboard.remove("navigation.ets2_route.fallback_reason");
        ctx.blackboard.remove("navigation.ets2_route.import_error");
    }

    fn clear_ets2_router_output(&mut self, ctx: &PluginContext) {
        self.active = false;
        self.waypoint_count = 0;
        self.path_total_distance_m = 0.0;
        ctx.blackboard.remove("router.waypoints");
        self.clear_ets2_trim_keys(ctx);
        self.clear_ets2_progress_keys(ctx);
        self.clear_ets2_repair_keys(ctx);
        ctx.blackboard.remove("router.route_node_ids");
        self.current_route_node_ids.clear();
        if let Some(ref lock) = ctx.route_node_ids {
            lock.write().unwrap().clear();
        }
    }

    /// Drop ETS2-import mode and allow A* again. Clears stale router output if import was active.
    fn release_ets2_import(
        &mut self,
        telemetry: Option<&Telemetry>,
        ctx: &PluginContext,
        import_state: &str,
        fallback_reason: &str,
        release_reason: &str,
        import_error: &str,
    ) {
        let was_active = self.ets2_import_active;
        self.ets2_import_active = false;
        self.clear_ets2_import_progress_state();
        self.set_ets2_not_imported(ctx);
        ctx.blackboard
            .set("navigation.ets2_route.import_state", import_state);
        ctx.blackboard
            .set("navigation.ets2_route.fallback_reason", fallback_reason);
        ctx.blackboard
            .set("navigation.ets2_route.release_reason", release_reason);
        ctx.blackboard
            .set("navigation.ets2_route.import_error", import_error);
        ctx.blackboard.set(
            "navigation.ets2_route.progress_status",
            Ets2RouteProgressStatus::Released.as_str(),
        );
        ctx.blackboard
            .set("navigation.ets2_route.progress_republished", "false");
        if was_active {
            self.clear_ets2_router_output(ctx);
            tracing::info!(
                "[router] ETS2 import released: state={import_state} reason={fallback_reason} release={release_reason} ({import_error})"
            );
        }
        if was_active && self.goal_uid != 0 {
            let (pos_x, pos_z, truck_heading) = match telemetry {
                Some(t) => (t.position[0], t.position[2], t.heading),
                None => (
                    ctx.blackboard.get_f64("telemetry.position_x").unwrap_or(0.0),
                    ctx.blackboard.get_f64("telemetry.position_z").unwrap_or(0.0),
                    ctx.blackboard.get_f64("telemetry.heading").unwrap_or(0.0),
                ),
            };
            self.last_replan_reason = "ets2_release_replan".to_string();
            ctx.blackboard
                .set("router.last_replan_reason", &self.last_replan_reason);
            self.send_route_request(pos_x, pos_z, truck_heading, ctx);
            tracing::info!(
                "[router] ETS2 release → immediate A* replan to goal_uid={}",
                self.goal_uid
            );
        }
    }

    fn mark_astar_fallback_active(&self, ctx: &PluginContext) {
        if self.last_ets2_imported_hash.is_some() && !self.ets2_import_active {
            ctx.blackboard
                .set("navigation.ets2_route.import_state", "fallback_astar");
        }
    }

    fn apply_ets2_import(
        &mut self,
        ctx: &PluginContext,
        route_hash: u64,
        sequence: u32,
        full_route_node_ids: Vec<u64>,
        output: truckpilot_plugin_api::ets2_route::Ets2RouterOutput,
        trim: &Ets2RouteTrimResult,
        repair: &Ets2RouteRepairResult,
    ) {
        self.drain_worker_results();
        self.ets2_import_active = true;
        self.last_ets2_imported_hash = Some(route_hash);
        self.last_ets2_imported_sequence = Some(sequence);
        self.full_imported_route_node_ids = full_route_node_ids;
        self.last_published_start_index = trim.start_index;
        self.ets2_offroute_since_ms = None;
        self.last_ets2_progress_republish_at_ms = 0;

        self.active = true;
        self.last_planning_result = "ok_ets2_import".to_string();
        self.last_planning_error_detail = String::new();
        self.last_planning_duration_ms = 0;
        self.last_replan_reason = "ets2_route_import".to_string();
        let imported_route_ids = output.route_node_ids.clone();
        self.publish_ets2_router_output(ctx, output);

        ctx.blackboard.set("navigation.ets2_route.imported", "true");
        ctx.blackboard
            .set("navigation.ets2_route.imported_hash", route_hash.to_string());
        ctx.blackboard
            .set("navigation.ets2_route.release_reason", "none");
        self.publish_ets2_trim_keys(ctx, trim);
        let initial_progress = Ets2RouteProgressDecision {
            should_republish: false,
            should_release: false,
            release_reason: None,
            effective_start_index: trim.start_index,
            progress_status: Ets2RouteProgressStatus::Ok,
            counts_as_offroute: false,
        };
        self.publish_ets2_progress_keys(ctx, trim, &initial_progress, 0.0, false);
        self.publish_ets2_repair_keys(ctx, repair);
        self.publish_ets2_coord_delta_keys(ctx);
        self.publish_ets2_distance_graph_keys(ctx, &imported_route_ids);
        ctx.blackboard
            .set("navigation.ets2_route.last_imported_hash", route_hash.to_string());
        ctx.blackboard
            .set("navigation.ets2_route.last_imported_sequence", sequence.to_string());
        self.publish_ets2_lifecycle_active(ctx);

        self.publish_planning_diag(ctx);

        tracing::info!(
            "[router] ETS2 route imported: {} nodes (trimmed from {}), {} waypoints, {:.0}m, snap={} start={}, repair={}, hash={:#x} seq={}",
            trim.trimmed_node_count,
            trim.original_node_count,
            self.waypoint_count,
            self.path_total_distance_m,
            trim.snap_status.as_str(),
            trim.start_index,
            repair.status.as_str(),
            route_hash,
            sequence,
        );
        if repair.gap_count > 0 {
            tracing::info!(
                "[router] ETS2 route repaired: gaps={} repaired={} failed={} inserted={} hash={:#x}",
                repair.gap_count,
                repair.success_count,
                repair.failed_count,
                repair.inserted_node_count,
                route_hash,
            );
        }
    }

    /// Returns `true` when an ETS2 route is actively driving router output.
    fn try_import_ets2_route(
        &mut self,
        telemetry: Option<&Telemetry>,
        ctx: &PluginContext,
    ) -> bool {
        use truckpilot_plugin_api::ets2_route::Ets2RouteMatchStatus;

        let was_active = self.ets2_import_active;

        let Some(ets2_lock) = ctx.ets2_route.as_ref() else {
            if was_active {
                self.release_ets2_import(
                    telemetry,
                    ctx,
                    "lost",
                    "no_snapshot",
                    "route_lost",
                    "ets2_route_lost",
                );
            } else {
                self.publish_ets2_lifecycle_idle(ctx);
            }
            return false;
        };

        let guard = match ets2_lock.read() {
            Ok(g) => g,
            Err(_) => {
                if was_active {
                    self.release_ets2_import(
                        telemetry,
                        ctx,
                        "lost",
                        "no_snapshot",
                        "route_lost",
                        "ets2_route_lost",
                    );
                }
                return false;
            }
        };

        let Some(snapshot) = guard.snapshot.as_ref() else {
            if was_active {
                self.release_ets2_import(
                    telemetry,
                    ctx,
                    "lost",
                    "no_snapshot",
                    "route_lost",
                    "ets2_route_lost",
                );
            } else {
                self.publish_ets2_lifecycle_idle(ctx);
            }
            return false;
        };

        if !snapshot.valid {
            if was_active {
                self.release_ets2_import(
                    telemetry,
                    ctx,
                    "invalid",
                    "invalid_snapshot",
                    "invalid",
                    "ets2_route_invalid",
                );
            } else {
                self.set_ets2_not_imported(ctx);
                ctx.blackboard
                    .set("navigation.ets2_route.import_state", "invalid");
                ctx.blackboard
                    .set("navigation.ets2_route.fallback_reason", "invalid_snapshot");
                ctx.blackboard
                    .set("navigation.ets2_route.import_error", "ets2_route_invalid");
                ctx.blackboard
                    .set("navigation.ets2_route.release_reason", "invalid");
            }
            return false;
        }

        let Some(match_result) = guard.match_result.as_ref() else {
            if was_active {
                self.release_ets2_import(
                    telemetry,
                    ctx,
                    "lost",
                    "no_snapshot",
                    "route_lost",
                    "ets2_route_lost",
                );
            } else {
                self.publish_ets2_lifecycle_idle(ctx);
            }
            return false;
        };

        if !match_result.is_usable {
            let import_state = match match_result.status {
                Ets2RouteMatchStatus::Invalid => "invalid",
                Ets2RouteMatchStatus::Unavailable => "lost",
                _ => "unusable",
            };
            let import_error = match match_result.status {
                Ets2RouteMatchStatus::Invalid => "ets2_route_invalid",
                Ets2RouteMatchStatus::Unavailable => "ets2_route_lost",
                _ => "ets2_route_unusable",
            };
            let release_reason = match match_result.status {
                Ets2RouteMatchStatus::Invalid => "invalid",
                Ets2RouteMatchStatus::Unavailable => "route_lost",
                _ => "unusable",
            };
            if was_active {
                self.release_ets2_import(
                    telemetry,
                    ctx,
                    import_state,
                    "unusable_match",
                    release_reason,
                    import_error,
                );
            } else {
                self.set_ets2_not_imported(ctx);
                ctx.blackboard
                    .set("navigation.ets2_route.import_state", import_state);
                ctx.blackboard
                    .set("navigation.ets2_route.fallback_reason", "unusable_match");
                ctx.blackboard
                    .set("navigation.ets2_route.import_error", import_error);
                ctx.blackboard
                    .set("navigation.ets2_route.release_reason", release_reason);
            }
            return false;
        }

        let route_hash = snapshot.route_hash;
        let sequence = snapshot.sequence;
        let changed = self.last_ets2_imported_hash != Some(route_hash)
            || self.last_ets2_imported_sequence != Some(sequence);
        let recover_inactive = !self.active;

        if !changed && !recover_inactive && self.ets2_import_active {
            return true;
        }

        let Some(graph) = self.graph.as_ref() else {
            if was_active {
                self.release_ets2_import(
                    telemetry,
                    ctx,
                    "unusable",
                    "build_error",
                    "build_error",
                    "router graph not loaded",
                );
            } else {
                ctx.blackboard.set(
                    "navigation.ets2_route.import_error",
                    "router graph not loaded",
                );
                self.set_ets2_not_imported(ctx);
                ctx.blackboard
                    .set("navigation.ets2_route.import_state", "unusable");
                ctx.blackboard
                    .set("navigation.ets2_route.fallback_reason", "build_error");
                ctx.blackboard
                    .set("navigation.ets2_route.release_reason", "build_error");
            }
            self.ets2_import_active = false;
            return false;
        };

        let (truck_x, truck_z, heading) = Self::truck_pose_for_ets2_trim(telemetry, ctx);

        let repair = repair_ets2_route_gaps(
            graph,
            &snapshot.uids,
            &match_result.route_node_ids,
        );
        self.publish_ets2_repair_keys(ctx, &repair);

        if !repair.import_allowed {
            let err = repair
                .error
                .clone()
                .unwrap_or_else(|| "ets2 route gap repair failed".into());
            tracing::warn!("[router] ETS2 gap repair failed: {err}");
            if was_active {
                self.release_ets2_import(
                    telemetry,
                    ctx,
                    "unusable",
                    "build_error",
                    "build_error",
                    &err,
                );
            } else {
                ctx.blackboard.set("navigation.ets2_route.import_error", err.clone());
                self.set_ets2_not_imported(ctx);
                ctx.blackboard
                    .set("navigation.ets2_route.import_state", "unusable");
                ctx.blackboard
                    .set("navigation.ets2_route.fallback_reason", "build_error");
                ctx.blackboard
                    .set("navigation.ets2_route.release_reason", "build_error");
            }
            self.ets2_import_active = false;
            return false;
        }

        let route_for_import = repair.route_node_ids.clone();

        match build_trimmed_ets2_router_output(
            graph,
            &route_for_import,
            truck_x,
            truck_z,
            heading,
        ) {
            Ok((output, trim)) => {
                self.apply_ets2_import(
                    ctx,
                    route_hash,
                    sequence,
                    route_for_import,
                    output,
                    &trim,
                    &repair,
                );
                true
            }
            Err((trim, e)) => {
                self.publish_ets2_trim_keys(ctx, &trim);
                tracing::warn!("[router] ETS2 import failed: {e}");
                if was_active {
                    self.release_ets2_import(
                        telemetry,
                        ctx,
                        "unusable",
                        "build_error",
                        "build_error",
                        &e,
                    );
                } else {
                    ctx.blackboard
                        .set("navigation.ets2_route.import_error", e.clone());
                    self.set_ets2_not_imported(ctx);
                    ctx.blackboard
                        .set("navigation.ets2_route.import_state", "unusable");
                    ctx.blackboard
                        .set("navigation.ets2_route.fallback_reason", "build_error");
                    ctx.blackboard
                        .set("navigation.ets2_route.release_reason", "build_error");
                }
                self.ets2_import_active = false;
                false
            }
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
                snap_rejected_by_heading: 0,
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
        let (start_uid, snap_dist_m, heading_filter_applied, snap_rejected_by_heading, snap_method) =
            if let Some((uid, dist, hf, rejected)) = edge_snap {
                (uid, dist, hf, rejected, "edge")
            } else if let Some((uid, dist, hf)) = graph.find_nearest_with_heading(
                req.truck_x,
                req.truck_z,
                req.truck_heading,
                SNAP_RADIUS_M,
            ) {
                (uid, dist, hf, 0, "node")
            } else {
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
                    snap_rejected_by_heading: 0,
                    snap_method: "none".to_string(),
                });
                continue;
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
                    snap_rejected_by_heading,
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
                    snap_dist_m,
                    heading_filter_applied,
                    snap_rejected_by_heading,
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
        ctx.blackboard
            .set("router.last_snap_rejected_by_heading", "0");
        ctx.blackboard.set("router.snap_method", "");
        ctx.blackboard.set("router.auto_replan_count", "0");
        ctx.blackboard.set("router.auto_replan_triggered_at", "");
        ctx.blackboard.set("router.last_replan_reason", "");
        ctx.blackboard.set("router.snap_stable_edge_id", "");
        ctx.blackboard.set("router.snap_stability", "0");
        ctx.blackboard.set("router.snap_window_unique_edges", "0");
        ctx.blackboard.set("router.snap_last_change_at", "");
        ctx.blackboard
            .set("navigation.ets2_route.imported", "false");
        ctx.blackboard
            .set("navigation.ets2_route.import_state", "inactive");
        ctx.blackboard
            .set("navigation.ets2_route.release_reason", "none");
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
        let tick_started = Instant::now();
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

        // ── 1. ETS2 in-game route import (Phase 5b, preferred over A*) ───────
        self.try_import_ets2_route(telemetry, ctx);
        if self.ets2_import_active {
            self.update_ets2_live_progress(telemetry, ctx);
        }

        // ── 2. Poll worker result (non-blocking; skipped when ETS2 active) ─
        if self.ets2_import_active {
            self.drain_worker_results();
        } else if let Some(rx) = &self.result_rx {
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
                                self.last_snap_rejected_by_heading =
                                    result.snap_rejected_by_heading;
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
                                self.mark_astar_fallback_active(ctx);
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

        // ── 3. Detect goal changes ────────────────────────────────────────────
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
                        ctx.blackboard.set(
                            "navigation.ets2_route.fallback_reason",
                            "manual_goal_astar",
                        );
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

        // ── 3.1. Per-tick snap → sliding-window vote (Phase 6.5t) ──────────────
        if self.needs_heavy_router_work(ctx) {
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
        }

        // ── 3.2. Autopilot state-change → reset snap window ───────────────────
        let ap_state = ctx.blackboard.get("autopilot.state").unwrap_or_default();
        if ap_state != self.last_autopilot_state {
            self.last_autopilot_state = ap_state;
            self.reset_snap_window();
        }

        // ── 3.3. Off-route auto-replan check (Phase 6.5q + 6.5s) ───────────────
        if self.goal_uid != 0 && !self.pending_request && !self.current_route_node_ids.is_empty() {
            if let Some(tel) = telemetry {
                let pos_x = tel.position[0];
                let pos_z = tel.position[2];
                // Node-proximity check: is the nearest route node within radius?
                // find_nearest_geometric is intentionally heading-blind here —
                // the "truck counterflow but still near route nodes" case is
                // handled by the heading_stage == "AutoReplan" trigger below.
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

        // ── 4. Periodic replan (PhaseA cadence, skip if request in flight) ────
        if ctx.is_replan_tick() && !self.pending_request && self.goal_uid != 0 {
            let (pos_x, pos_z) = telemetry
                .map(|t| (t.position[0], t.position[2]))
                .unwrap_or((0.0, 0.0));
            let truck_heading = telemetry.map(|t| t.heading).unwrap_or(0.0);
            self.send_route_request(pos_x, pos_z, truck_heading, ctx);
        }

        // ── 5. Publish current state (every tick) ─────────────────────────────
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
        ctx.blackboard.set(
            "router.last_snap_rejected_by_heading",
            self.last_snap_rejected_by_heading.to_string(),
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

        let tick_ms = tick_started.elapsed().as_millis();
        ctx.blackboard
            .set("router.last_tick_ms", tick_ms.to_string());
        if tick_ms > 100 {
            let reason = if self.ets2_import_active {
                "ets2_import"
            } else if self.needs_heavy_router_work(ctx) {
                "snap_or_replan"
            } else {
                "idle_publish"
            };
            ctx.blackboard
                .set("router.slow_tick_reason", reason);
        } else {
            ctx.blackboard.remove("router.slow_tick_reason");
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

    fn long_graph() -> (NodeList, EdgeList) {
        (
            vec![
                (1, 0.0, 0.0),
                (2, 100.0, 0.0),
                (3, 200.0, 0.0),
                (4, 300.0, 0.0),
            ],
            vec![(1, 2, 100.0), (2, 3, 100.0), (3, 4, 100.0)],
        )
    }

    fn fake_telemetry_at(x: f64, z: f64) -> Telemetry {
        Telemetry {
            position: [x, 0.0, z],
            heading: 0.0,
            pitch: 0.0,
            roll: 0.0,
            speed_ms: 0.0,
            engine_gear: 0,
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
        // south_graph: edge 1→2 runs along +z, aligned with the truck's heading
        // (fake_telemetry_at uses heading 0.0). simple_graph's East edges are
        // perpendicular to that heading (dot≈0), which makes edge-snap skip
        // node 1 to node 2 — a geometry artefact, not what this test means to
        // assert. Aligned geometry snaps to node 1 → full 1→2→3 path.
        let (n, e) = south_graph();
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
    fn idle_off_state_skips_heavy_snap_and_sets_last_tick_ms() {
        let (n, e) = simple_graph();
        let mut p = plugin_with_worker(n, e);

        let bb = SharedBlackboard::new();
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseA)
            .with_tick_count(1);
        assert!(!p.needs_heavy_router_work(&ctx));

        let mut out = ControlOutput::default();
        let t = fake_telemetry_at(50.0, 0.0);
        p.tick(Some(&t), &mut out, &ctx);

        assert_eq!(p.snap_stability, 0);
        assert_eq!(p.snap_window_unique_edges, 0);
        assert!(bb.get("router.last_tick_ms").is_some());
        assert!(bb.get("router.slow_tick_reason").is_none());
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
    fn south_graph() -> (NodeList, EdgeList) {
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

    // ── Phase 5b: ETS2 route import tests ─────────────────────────────────────

    use std::collections::HashSet;
    use std::sync::{Arc, RwLock};
    use truckpilot_plugin_api::ets2_route::{
        match_ets2_route_uids, Ets2RouteMatchResult,
        Ets2RouteMatchStatus, Ets2RouteSharedState, Ets2RouteSnapshot, Ets2RouteWaypoint,
        ETS2_WP_FLAG_HAS_DISTANCE, ETS2_WP_FLAG_HAS_POSITION, ETS2_WP_FLAG_UNTRUSTED,
        Ets2GraphDistanceStatus,
    };

    fn ctx_with_ets2(
        bb: SharedBlackboard,
        graph: Arc<RouterGraph>,
        state: Arc<RwLock<Ets2RouteSharedState>>,
    ) -> PluginContext {
        let mut ctx = PluginContext::new("test", bb)
            .with_phase(TickPhase::PhaseC)
            .with_tick_count(1);
        ctx.graph = Some(graph);
        ctx.ets2_route = Some(state);
        ctx.route_node_ids = Some(Arc::new(RwLock::new(HashSet::new())));
        ctx
    }

    fn ctx_with_ets2_state(
        bb: SharedBlackboard,
        graph: Arc<RouterGraph>,
        state: Ets2RouteSharedState,
    ) -> PluginContext {
        ctx_with_ets2(bb, graph, Arc::new(RwLock::new(state)))
    }

    fn usable_ets2_state(
        graph: &RouterGraph,
        uids: &[u64],
        hash: u64,
        seq: u32,
    ) -> Ets2RouteSharedState {
        Ets2RouteSharedState {
            snapshot: Some(Ets2RouteSnapshot {
                sequence: seq,
                route_hash: hash,
                valid: true,
                uids: uids.to_vec(),
            }),
            match_result: Some(match_ets2_route_uids(graph, uids)),
            waypoints: Vec::new(),
        }
    }

    #[test]
    fn ets2_import_uses_graph_not_ets2_positions_for_router_waypoints() {
        let (n, e) = simple_graph();
        let graph = Arc::new(RouterGraph::new(n, e));
        let uids = vec![1_u64, 2, 3];
        let mut state = usable_ets2_state(&graph, &uids, 0xDEAD, 10);
        state.waypoints = vec![
            Ets2RouteWaypoint {
                uid: 1,
                x: 999.0,
                z: 999.0,
                flags: ETS2_WP_FLAG_HAS_POSITION,
                ..Default::default()
            },
            Ets2RouteWaypoint {
                uid: 2,
                x: 999.0,
                z: 999.0,
                flags: ETS2_WP_FLAG_HAS_POSITION,
                ..Default::default()
            },
            Ets2RouteWaypoint {
                uid: 3,
                x: 999.0,
                z: 999.0,
                flags: ETS2_WP_FLAG_HAS_POSITION,
                ..Default::default()
            },
        ];
        let bb = SharedBlackboard::new();
        let ctx = ctx_with_ets2_state(bb.clone(), Arc::clone(&graph), state);
        let mut p = plugin_with_worker(graph.nodes.clone(), graph.edges.clone());
        let mut out = ControlOutput::default();
        p.tick(None, &mut out, &ctx);
        let wp: Vec<[f64; 2]> =
            serde_json::from_str(&bb.get("router.waypoints").unwrap()).unwrap();
        assert!((wp[0][0] - 0.0).abs() < 0.01);
        assert!((wp[2][0] - 200.0).abs() < 0.01);
        assert_eq!(
            bb.get("navigation.ets2_route.coord_graph_delta_count").as_deref(),
            Some("3")
        );
        let max_m: f64 = bb
            .get("navigation.ets2_route.coord_graph_delta_max_m")
            .unwrap()
            .parse()
            .unwrap();
        assert!(max_m > 100.0);
    }

    #[test]
    fn ets2_import_publishes_distance_graph_diagnosis() {
        let (n, e) = simple_graph();
        let graph = Arc::new(RouterGraph::new(n, e));
        let uids = vec![1_u64, 2, 3];
        let mut state = usable_ets2_state(&graph, &uids, 0xBEEF, 11);
        state.waypoints = vec![
            Ets2RouteWaypoint {
                uid: 1,
                distance: 200.0,
                flags: ETS2_WP_FLAG_HAS_DISTANCE | ETS2_WP_FLAG_UNTRUSTED,
                ..Default::default()
            },
            Ets2RouteWaypoint {
                uid: 2,
                distance: 100.0,
                flags: ETS2_WP_FLAG_HAS_DISTANCE | ETS2_WP_FLAG_UNTRUSTED,
                ..Default::default()
            },
            Ets2RouteWaypoint {
                uid: 3,
                distance: 0.0,
                flags: ETS2_WP_FLAG_HAS_DISTANCE | ETS2_WP_FLAG_UNTRUSTED,
                ..Default::default()
            },
        ];
        let bb = SharedBlackboard::new();
        let ctx = ctx_with_ets2_state(bb.clone(), Arc::clone(&graph), state);
        let mut p = plugin_with_worker(graph.nodes.clone(), graph.edges.clone());
        let mut out = ControlOutput::default();
        p.tick(None, &mut out, &ctx);
        assert_eq!(
            bb.get("navigation.ets2_route.distance_graph_status").as_deref(),
            Some(Ets2GraphDistanceStatus::Untrusted.as_str())
        );
        assert_eq!(
            bb.get("navigation.ets2_route.distance_graph_total_m").as_deref(),
            Some("200.0")
        );
    }

    fn chain_graph(n: u64) -> (NodeList, EdgeList) {
        let nodes: NodeList = (1..=n)
            .map(|i| (i, (i - 1) as f64 * 100.0, 0.0))
            .collect();
        let edges: EdgeList = (1..n).map(|i| (i, i + 1, 100.0)).collect();
        (nodes, edges)
    }

    #[test]
    fn ets2_gap_repair_imports_astar_nodes() {
        let (n, e) = chain_graph(6);
        let graph = Arc::new(RouterGraph::new(n, e));
        let uids = vec![1_u64, 2, 999, 998, 5, 6];
        let state = Ets2RouteSharedState {
            snapshot: Some(Ets2RouteSnapshot {
                sequence: 60,
                route_hash: 0x6001,
                valid: true,
                uids: uids.clone(),
            }),
            match_result: Some(Ets2RouteMatchResult {
                status: Ets2RouteMatchStatus::Partial,
                route_node_ids: vec![1, 2, 5, 6],
                matched_count: 4,
                missing_count: 2,
                first_missing_uid: Some(999),
                match_ratio: 1.0,
                is_usable: true,
                import_error: None,
            }),
            ..Default::default()
        };

        let bb = SharedBlackboard::new();
        let ctx = ctx_with_ets2_state(bb.clone(), Arc::clone(&graph), state);
        let mut p = plugin_with_worker(
            graph.nodes.clone(),
            graph.edges.clone(),
        );
        let mut out = ControlOutput::default();

        p.tick(
            Some(&fake_telemetry_at(0.0, 0.0)),
            &mut out,
            &ctx,
        );

        assert_eq!(
            bb.get("navigation.ets2_route.repair_status").as_deref(),
            Some("repaired")
        );
        assert_eq!(
            bb.get("navigation.ets2_route.repair_inserted_node_count").as_deref(),
            Some("2")
        );
        let route_ids: Vec<u64> =
            serde_json::from_str(&bb.get("router.route_node_ids").unwrap()).unwrap();
        assert_eq!(route_ids, vec![1, 2, 3, 4, 5, 6]);
    }

    #[test]
    fn ets2_gap_repair_failure_rejects_import() {
        let (n, e) = chain_graph(10);
        let graph = Arc::new(RouterGraph::new(n, e));
        let uids = vec![
            1_u64, 2,
            901, 902, 903, 904, 905, 906, 907, 908, 909,
            10,
        ];
        let state = Ets2RouteSharedState {
            snapshot: Some(Ets2RouteSnapshot {
                sequence: 61,
                route_hash: 0x6002,
                valid: true,
                uids: uids.clone(),
            }),
            match_result: Some(Ets2RouteMatchResult {
                status: Ets2RouteMatchStatus::Partial,
                route_node_ids: vec![1, 2, 10],
                matched_count: 3,
                missing_count: 9,
                first_missing_uid: Some(901),
                match_ratio: 1.0,
                is_usable: true,
                import_error: None,
            }),
            ..Default::default()
        };
        let bb = SharedBlackboard::new();
        let ctx = ctx_with_ets2_state(bb.clone(), graph, state);
        let mut p = plugin_with_worker(chain_graph(10).0, chain_graph(10).1);
        let mut out = ControlOutput::default();

        p.tick(
            Some(&fake_telemetry_at(0.0, 0.0)),
            &mut out,
            &ctx,
        );

        assert_eq!(
            bb.get("navigation.ets2_route.imported").as_deref(),
            Some("false")
        );
        assert_eq!(
            bb.get("navigation.ets2_route.repair_status").as_deref(),
            Some("failed")
        );
        assert!(bb.get("router.waypoints").is_none());
    }

    #[test]
    fn ets2_import_publishes_router_contract() {
        let (n, e) = simple_graph();
        let graph = Arc::new(RouterGraph::new(n, e));
        let uids = vec![1_u64, 2, 3];
        let state = usable_ets2_state(&graph, &uids, 0xDEAD, 10);
        let bb = SharedBlackboard::new();
        let ctx = ctx_with_ets2_state(bb.clone(), Arc::clone(&graph), state);
        let mut p = plugin_with_worker(
            graph.nodes.clone(),
            graph.edges.clone(),
        );
        let mut out = ControlOutput::default();

        p.tick(None, &mut out, &ctx);

        assert_eq!(bb.get("router.active").as_deref(), Some("true"));
        assert_eq!(
            bb.get("router.last_planning_result").as_deref(),
            Some("ok_ets2_import")
        );
        assert_eq!(
            bb.get("navigation.ets2_route.imported").as_deref(),
            Some("true")
        );
        assert_eq!(
            bb.get("navigation.ets2_route.imported_hash").as_deref(),
            Some("57005")
        );
        let wp: Vec<[f64; 2]> =
            serde_json::from_str(&bb.get("router.waypoints").unwrap()).unwrap();
        assert_eq!(wp.len(), 3);
        assert!((wp[2][0] - 200.0).abs() < 0.01);
        let route_ids: Vec<u64> =
            serde_json::from_str(&bb.get("router.route_node_ids").unwrap()).unwrap();
        assert_eq!(route_ids, uids);
        assert!(!p.pending_request, "ETS2 import must not leave A* pending");
        assert_eq!(
            bb.get("navigation.ets2_route.snap_status").as_deref(),
            Some("no_position")
        );
        assert_eq!(
            bb.get("navigation.ets2_route.trimmed").as_deref(),
            Some("false")
        );
    }

    #[test]
    fn ets2_trim_snaps_to_truck_progress() {
        let (n, e) = simple_graph();
        let graph = Arc::new(RouterGraph::new(n, e));
        let uids = vec![1_u64, 2, 3];
        let state = usable_ets2_state(&graph, &uids, 0xFEED, 12);
        let bb = SharedBlackboard::new();
        let ctx = ctx_with_ets2_state(bb.clone(), Arc::clone(&graph), state);
        let mut p = plugin_with_worker(
            graph.nodes.clone(),
            graph.edges.clone(),
        );
        let mut out = ControlOutput::default();

        p.tick(
            Some(&fake_telemetry_at(151.0, 0.0)),
            &mut out,
            &ctx,
        );

        let route_ids: Vec<u64> =
            serde_json::from_str(&bb.get("router.route_node_ids").unwrap()).unwrap();
        assert_eq!(route_ids, vec![2, 3]);
        assert_eq!(
            bb.get("navigation.ets2_route.trimmed").as_deref(),
            Some("true")
        );
        assert_eq!(
            bb.get("navigation.ets2_route.trim_start_index").as_deref(),
            Some("1")
        );
        assert_eq!(
            bb.get("navigation.ets2_route.snap_status").as_deref(),
            Some("ok")
        );
        assert_eq!(
            bb.get("navigation.ets2_route.imported_node_count").as_deref(),
            Some("2")
        );
    }

    #[test]
    fn ets2_trim_too_short_rejects_import() {
        let (graph, uids) = {
            let nodes = vec![(1, 0.0, 0.0), (2, 100.0, 0.0)];
            let edges = vec![(1, 2, 100.0)];
            (
                Arc::new(RouterGraph::new(nodes, edges)),
                vec![1_u64, 2],
            )
        };
        let state = Ets2RouteSharedState {
            snapshot: Some(Ets2RouteSnapshot {
                sequence: 5,
                route_hash: 5,
                valid: true,
                uids: uids.clone(),
            }),
            match_result: Some(Ets2RouteMatchResult {
                status: Ets2RouteMatchStatus::Matched,
                route_node_ids: vec![1],
                matched_count: 1,
                missing_count: 0,
                first_missing_uid: None,
                match_ratio: 1.0,
                is_usable: true,
                import_error: None,
            }),
            ..Default::default()
        };
        let bb = SharedBlackboard::new();
        let ctx = ctx_with_ets2_state(bb.clone(), graph, state);
        let mut p = plugin_with_worker(simple_graph().0, simple_graph().1);
        let mut out = ControlOutput::default();

        p.tick(
            Some(&fake_telemetry_at(0.0, 0.0)),
            &mut out,
            &ctx,
        );

        assert_eq!(
            bb.get("navigation.ets2_route.imported").as_deref(),
            Some("false")
        );
        assert_eq!(
            bb.get("navigation.ets2_route.repair_status").as_deref(),
            Some("failed")
        );
        assert!(bb.get("router.waypoints").is_none());
    }

    // ── Phase 5e: ETS2 live progress tests ──────────────────────────────────

    #[test]
    fn ets2_live_progress_advances_router_output() {
        let (n, e) = long_graph();
        let graph = Arc::new(RouterGraph::new(n, e));
        let uids = vec![1_u64, 2, 3, 4];
        let state = usable_ets2_state(&graph, &uids, 0x5001, 50);
        let bb = SharedBlackboard::new();
        let ctx = ctx_with_ets2_state(bb.clone(), Arc::clone(&graph), state);
        let mut p = plugin_with_worker(
            graph.nodes.clone(),
            graph.edges.clone(),
        );
        let mut out = ControlOutput::default();

        p.tick(
            Some(&fake_telemetry_at(5.0, 0.0)),
            &mut out,
            &ctx,
        );
        let initial: Vec<u64> =
            serde_json::from_str(&bb.get("router.route_node_ids").unwrap()).unwrap();
        assert_eq!(initial.len(), 4);

        p.tick(
            Some(&fake_telemetry_at(151.0, 0.0)),
            &mut out,
            &ctx,
        );
        let advanced: Vec<u64> =
            serde_json::from_str(&bb.get("router.route_node_ids").unwrap()).unwrap();
        assert!(advanced.len() < initial.len());
        assert_eq!(
            bb.get("navigation.ets2_route.progress_status").as_deref(),
            Some("advanced")
        );
        assert_eq!(
            bb.get("navigation.ets2_route.progress_republished").as_deref(),
            Some("true")
        );
    }

    #[test]
    fn ets2_live_progress_unchanged_without_movement() {
        let (n, e) = long_graph();
        let graph = Arc::new(RouterGraph::new(n, e));
        let state = usable_ets2_state(&graph, &[1, 2, 3, 4], 0x5002, 51);
        let bb = SharedBlackboard::new();
        let ctx = ctx_with_ets2_state(bb.clone(), graph, state);
        let mut p = plugin_with_worker(long_graph().0, long_graph().1);
        let mut out = ControlOutput::default();
        let tel = fake_telemetry_at(5.0, 0.0);

        p.tick(Some(&tel), &mut out, &ctx);
        let first: Vec<u64> =
            serde_json::from_str(&bb.get("router.route_node_ids").unwrap()).unwrap();

        p.tick(Some(&tel), &mut out, &ctx);
        let second: Vec<u64> =
            serde_json::from_str(&bb.get("router.route_node_ids").unwrap()).unwrap();
        assert_eq!(first, second);
        assert_eq!(
            bb.get("navigation.ets2_route.progress_status").as_deref(),
            Some("unchanged")
        );
        assert_eq!(
            bb.get("navigation.ets2_route.progress_republished").as_deref(),
            Some("false")
        );
    }

    #[test]
    fn ets2_live_progress_regression_ignored() {
        let (n, e) = long_graph();
        let graph = Arc::new(RouterGraph::new(n, e));
        let state = usable_ets2_state(&graph, &[1, 2, 3, 4], 0x5003, 52);
        let bb = SharedBlackboard::new();
        let ctx = ctx_with_ets2_state(bb.clone(), graph, state);
        let mut p = plugin_with_worker(long_graph().0, long_graph().1);
        let mut out = ControlOutput::default();

        p.tick(
            Some(&fake_telemetry_at(151.0, 0.0)),
            &mut out,
            &ctx,
        );
        let mid: Vec<u64> =
            serde_json::from_str(&bb.get("router.route_node_ids").unwrap()).unwrap();

        p.tick(
            Some(&fake_telemetry_at(5.0, 0.0)),
            &mut out,
            &ctx,
        );
        let after_back: Vec<u64> =
            serde_json::from_str(&bb.get("router.route_node_ids").unwrap()).unwrap();
        assert_eq!(mid, after_back);
        assert_eq!(
            bb.get("navigation.ets2_route.progress_status").as_deref(),
            Some("regression_ignored")
        );
    }

    #[test]
    fn ets2_single_offroute_tick_keeps_import() {
        let (n, e) = long_graph();
        let graph = Arc::new(RouterGraph::new(n, e));
        let state = usable_ets2_state(&graph, &[1, 2, 3, 4], 0x5004, 53);
        let bb = SharedBlackboard::new();
        let ctx = ctx_with_ets2_state(bb.clone(), graph, state);
        let mut p = plugin_with_worker(long_graph().0, long_graph().1);
        let mut out = ControlOutput::default();

        p.tick(
            Some(&fake_telemetry_at(5.0, 0.0)),
            &mut out,
            &ctx,
        );
        p.tick(
            Some(&fake_telemetry_at(0.0, 90.0)),
            &mut out,
            &ctx,
        );

        assert!(p.ets2_import_active);
        assert_eq!(
            bb.get("navigation.ets2_route.progress_status").as_deref(),
            Some("snap_bad")
        );
        assert!(bb.get("router.waypoints").is_some());
    }

    #[test]
    fn ets2_release_triggers_immediate_astar_replan() {
        let (n, e) = long_graph();
        let graph = Arc::new(RouterGraph::new(n, e));
        let shared = Arc::new(RwLock::new(usable_ets2_state(
            &graph,
            &[1, 2, 3, 4],
            0x5005,
            54,
        )));
        let bb = SharedBlackboard::new();
        bb.set("router.goal_uid", "4");
        let ctx = ctx_with_ets2(bb.clone(), Arc::clone(&graph), Arc::clone(&shared));
        let mut p = plugin_with_worker(
            graph.nodes.clone(),
            graph.edges.clone(),
        );
        p.goal_uid = 4;
        let mut out = ControlOutput::default();

        p.tick(
            Some(&fake_telemetry_at(5.0, 0.0)),
            &mut out,
            &ctx,
        );
        assert!(p.ets2_import_active);

        *shared.write().unwrap() = Ets2RouteSharedState::default();
        p.tick(
            Some(&fake_telemetry_at(5.0, 0.0)),
            &mut out,
            &ctx,
        );

        assert!(!p.ets2_import_active);
        assert!(
            p.pending_request,
            "A* replan must be requested immediately after ETS2 release when goal is set"
        );
        assert_eq!(
            bb.get("router.last_replan_reason").as_deref(),
            Some("ets2_release_replan")
        );
    }

    #[test]
    fn ets2_unusable_does_not_import() {
        let (n, e) = simple_graph();
        let graph = Arc::new(RouterGraph::new(n, e));
        let state = Ets2RouteSharedState {
            snapshot: Some(Ets2RouteSnapshot {
                sequence: 1,
                route_hash: 1,
                valid: true,
                uids: vec![1, 99, 3],
            }),
            match_result: Some(match_ets2_route_uids(&graph, &[1, 99, 3])),
            ..Default::default()
        };
        assert!(!state.match_result.as_ref().unwrap().is_usable);

        let bb = SharedBlackboard::new();
        let ctx = ctx_with_ets2_state(bb.clone(), graph, state);
        let mut p = plugin_with_worker(simple_graph().0, simple_graph().1);
        let mut out = ControlOutput::default();

        p.tick(None, &mut out, &ctx);

        assert_eq!(
            bb.get("navigation.ets2_route.imported").as_deref(),
            Some("false")
        );
        assert_eq!(bb.get("router.active").as_deref(), Some("false"));
        assert!(bb.get("router.waypoints").is_none());
    }

    #[test]
    fn ets2_same_hash_is_not_reimported() {
        let (n, e) = simple_graph();
        let graph = Arc::new(RouterGraph::new(n, e));
        let uids = vec![1_u64, 2, 3];
        let state = usable_ets2_state(&graph, &uids, 0xBEEF, 20);
        let bb = SharedBlackboard::new();
        let ctx = ctx_with_ets2_state(bb.clone(), Arc::clone(&graph), state);
        let mut p = plugin_with_worker(
            graph.nodes.clone(),
            graph.edges.clone(),
        );
        let mut out = ControlOutput::default();

        p.tick(None, &mut out, &ctx);
        assert_eq!(
            bb.get("router.last_planning_result").as_deref(),
            Some("ok_ets2_import")
        );

        p.last_planning_result = "tampered".to_string();
        p.tick(None, &mut out, &ctx);
        assert_eq!(
            p.last_planning_result, "tampered",
            "same ETS2 hash must not re-run import"
        );
        assert_eq!(p.last_ets2_imported_hash, Some(0xBEEF));
    }

    #[test]
    fn ets2_missing_position_prevents_import() {
        let (n, e) = simple_graph();
        let graph = Arc::new(RouterGraph::new(n, e));
        let state = Ets2RouteSharedState {
            snapshot: Some(Ets2RouteSnapshot {
                sequence: 3,
                route_hash: 3,
                valid: true,
                uids: vec![1, 2, 99],
            }),
            match_result: Some(Ets2RouteMatchResult {
                status: Ets2RouteMatchStatus::Partial,
                route_node_ids: vec![1, 2, 99],
                matched_count: 3,
                missing_count: 0,
                first_missing_uid: None,
                match_ratio: 1.0,
                is_usable: true,
                import_error: None,
            }),
            ..Default::default()
        };
        let bb = SharedBlackboard::new();
        let ctx = ctx_with_ets2_state(bb.clone(), graph, state);
        let mut p = plugin_with_worker(simple_graph().0, simple_graph().1);
        let mut out = ControlOutput::default();

        p.tick(None, &mut out, &ctx);

        assert_eq!(
            bb.get("navigation.ets2_route.imported").as_deref(),
            Some("false")
        );
        assert!(bb
            .get("navigation.ets2_route.import_error")
            .unwrap()
            .contains("99"));
    }

    // ── Phase 5c: ETS2 lifecycle / fallback tests ───────────────────────────

    #[test]
    fn ets2_lost_snapshot_releases_astar() {
        let (n, e) = simple_graph();
        let graph = Arc::new(RouterGraph::new(n, e));
        let uids = vec![1_u64, 2, 3];
        let shared = Arc::new(RwLock::new(usable_ets2_state(
            &graph,
            &uids,
            0xAA,
            1,
        )));
        let bb = SharedBlackboard::new();
        let ctx = ctx_with_ets2(bb.clone(), Arc::clone(&graph), Arc::clone(&shared));
        let mut p = plugin_with_worker(
            graph.nodes.clone(),
            graph.edges.clone(),
        );
        let mut out = ControlOutput::default();

        p.tick(None, &mut out, &ctx);
        assert!(p.ets2_import_active);

        *shared.write().unwrap() = Ets2RouteSharedState::default();
        p.tick(None, &mut out, &ctx);

        assert!(!p.ets2_import_active);
        assert_eq!(
            bb.get("navigation.ets2_route.import_state").as_deref(),
            Some("lost")
        );
        assert_eq!(
            bb.get("navigation.ets2_route.fallback_reason").as_deref(),
            Some("no_snapshot")
        );
        assert!(bb.get("router.waypoints").is_none());

        p.goal_uid = 3;
        bb.set("router.goal_uid", "3");
        let mut ctx_astar = ctx_with_ets2(bb.clone(), graph, shared);
        ctx_astar.tick_phase = TickPhase::PhaseA;
        ctx_astar.tick_count = 50;
        p.tick(
            Some(&fake_telemetry_at(0.0, 0.0)),
            &mut out,
            &ctx_astar,
        );
        assert!(
            p.pending_request,
            "A* must be allowed after ETS2 route loss"
        );
    }

    #[test]
    fn ets2_unusable_after_active_releases_import() {
        let (n, e) = simple_graph();
        let graph = Arc::new(RouterGraph::new(n, e));
        let shared = Arc::new(RwLock::new(usable_ets2_state(
            &graph,
            &[1, 2, 3],
            0xBB,
            2,
        )));
        let bb = SharedBlackboard::new();
        let ctx = ctx_with_ets2(bb.clone(), Arc::clone(&graph), Arc::clone(&shared));
        let mut p = plugin_with_worker(
            graph.nodes.clone(),
            graph.edges.clone(),
        );
        let mut out = ControlOutput::default();

        p.tick(None, &mut out, &ctx);
        assert!(p.ets2_import_active);

        *shared.write().unwrap() = Ets2RouteSharedState {
            snapshot: Some(Ets2RouteSnapshot {
                sequence: 3,
                route_hash: 0xCC,
                valid: true,
                uids: vec![1, 99, 3],
            }),
            match_result: Some(match_ets2_route_uids(&graph, &[1, 99, 3])),
            ..Default::default()
        };
        p.tick(None, &mut out, &ctx);

        assert!(!p.ets2_import_active);
        assert_eq!(
            bb.get("navigation.ets2_route.import_state").as_deref(),
            Some("unusable")
        );
        assert_eq!(
            bb.get("navigation.ets2_route.imported").as_deref(),
            Some("false")
        );
    }

    #[test]
    fn ets2_reimports_on_new_usable_hash() {
        let (n, e) = simple_graph();
        let graph = Arc::new(RouterGraph::new(n, e));
        let shared = Arc::new(RwLock::new(usable_ets2_state(
            &graph,
            &[1, 2, 3],
            0xD1,
            10,
        )));
        let bb = SharedBlackboard::new();
        let ctx = ctx_with_ets2(bb.clone(), Arc::clone(&graph), Arc::clone(&shared));
        let mut p = plugin_with_worker(
            graph.nodes.clone(),
            graph.edges.clone(),
        );
        let mut out = ControlOutput::default();

        p.tick(None, &mut out, &ctx);
        assert_eq!(
            bb.get("navigation.ets2_route.imported_hash").as_deref(),
            Some("209")
        );

        *shared.write().unwrap() = usable_ets2_state(&graph, &[1, 2], 0xD2, 11);
        p.tick(None, &mut out, &ctx);

        assert!(p.ets2_import_active);
        assert_eq!(
            bb.get("navigation.ets2_route.imported_hash").as_deref(),
            Some("210")
        );
        assert_eq!(
            bb.get("navigation.ets2_route.last_imported_sequence").as_deref(),
            Some("11")
        );
    }

    #[test]
    fn astar_works_without_ets2_channel() {
        let (n, e) = simple_graph();
        let mut p = plugin_with_worker(n, e);
        p.goal_uid = 3;

        let bb = SharedBlackboard::new();
        bb.set("router.goal_uid", "3");
        let ctx = PluginContext::new("test", bb.clone())
            .with_phase(TickPhase::PhaseC)
            .with_tick_count(7);
        let mut out = ControlOutput::default();

        p.tick(
            Some(&fake_telemetry_at(0.0, 0.0)),
            &mut out,
            &ctx,
        );
        assert!(p.pending_request);
        assert!(!p.ets2_import_active);
        assert_eq!(
            bb.get("navigation.ets2_route.import_state").as_deref(),
            Some("inactive")
        );
    }

    #[test]
    fn ets2_import_blocks_astar_replan() {
        let (n, e) = simple_graph();
        let graph = Arc::new(RouterGraph::new(n, e));
        let state = usable_ets2_state(&graph, &[1, 2, 3], 0xCAFE, 4);
        let bb = SharedBlackboard::new();
        bb.set("router.goal_uid", "3");
        let ctx = ctx_with_ets2_state(bb.clone(), graph, state)
            .with_phase(TickPhase::PhaseA)
            .with_tick_count(50);
        let mut p = plugin_with_worker(simple_graph().0, simple_graph().1);
        p.goal_uid = 3;
        let mut out = ControlOutput::default();

        p.tick(Some(&fake_telemetry_at(0.0, 0.0)), &mut out, &ctx);

        assert!(p.ets2_import_active);
        assert!(!p.pending_request, "A* must not run while ETS2 route is active");
        assert_eq!(
            bb.get("router.last_planning_result").as_deref(),
            Some("ok_ets2_import")
        );
    }
}
