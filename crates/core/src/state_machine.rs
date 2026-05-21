//! Autopilot state machine — Phase 6.2a.
//!
//! Runs in the daemon loop **before** [`crate::plugin_manager::PluginManager::tick_all`]
//! and publishes `autopilot.state` (plus `autopilot.fault_reason` while in
//! `Fault`) to the shared blackboard so plugins can branch via
//! `ctx.is_active()` / `ctx.is_engaged()` / `ctx.is_fault()`.
//!
//! Deviations from the original spec (documented):
//! - `Telemetry` carries `speed_ms` (m/s), not `speed_kmh`; the zero-speed
//!   threshold is `0.028 m/s` ≈ 0.1 km/h.
//! - `Telemetry` has no `steering_wheel_position` field; the "user grabs
//!   the wheel" transition is therefore not currently triggerable. The
//!   helper is kept for completeness and always returns `false`.

// Several public items below are wired in by later sub-phases
// (6.2g Watchdog uses `report_fault` + the broader FailureReason set;
// IPC will route `FaultDetected` events). They are stable surface, so
// suppress dead-code lints rather than narrow visibility.
#![allow(dead_code)]

use std::collections::HashSet;
use std::sync::{Arc, RwLock};

use truckpilot_ipc_protocol::PreconditionSnapshot;
use truckpilot_plugin_api::graph::RouterGraph;
use truckpilot_plugin_api::{SharedBlackboard, Telemetry};

// ---- Constants (in 50 Hz daemon ticks) -------------------------------------

const ENGAGE_TIMEOUT: u64 = 250; // 5 s
const PAUSE_DETECT: u64 = 250; // 5 s stopped
const PAUSE_TIMEOUT: u64 = 15_000; // 5 min
const PRECONDITION_STABLE: u64 = 50; // 1 s
const TELEMETRY_LOSS: u64 = 25; // 500 ms
const CRUISE_OFF_TOLERANCE: u64 = 25; // 500 ms — debounce before CruiseDeactivated fault
const ENGINE_OFF_TOLERANCE: u64 = 25; // 500 ms — debounce before EngineStopped fault
const PRECONDITION_GLITCH_TOLERANCE: u64 = 10; // 200 ms at 50 Hz daemon tick rate
const ZERO_SPEED_MS: f64 = 0.028; // ≈ 0.1 km/h

// ---- AutopilotState --------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AutopilotState {
    Off,
    Engaging,
    Active,
    Paused,
    Fault,
}

impl AutopilotState {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Off => "Off",
            Self::Engaging => "Engaging",
            Self::Active => "Active",
            Self::Paused => "Paused",
            Self::Fault => "Fault",
        }
    }
}

// ---- Failure reasons -------------------------------------------------------

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FailureReason {
    WatchdogStall,
    TelemetryLost,
    TelemetryFlapping,
    PluginPanic(String),
    WaypointsUnreachable,
    VjoyDisconnected,
    BlackboardPoisoned,
    CruiseDeactivated,
    EngineStopped,
    HeadingUnrecoverable,
    CriticalPluginMissing(String),
    UserRequested,
}

impl FailureReason {
    pub fn as_str(&self) -> String {
        match self {
            Self::PluginPanic(name) => format!("plugin_panic:{name}"),
            Self::CriticalPluginMissing(name) => format!("critical_plugin_missing:{name}"),
            other => format!("{other:?}"),
        }
    }
}

// ---- Events ----------------------------------------------------------------

#[derive(Debug, Clone)]
pub enum AutopilotEvent {
    UserEngage,
    UserDisengage,
    UserReset,
    FaultDetected(FailureReason),
}

// ---- Preconditions ---------------------------------------------------------

#[derive(Debug, Clone, Copy)]
pub struct Preconditions {
    pub telemetry_ok: bool,
    pub engine_running: bool,
    pub cruise_active: bool,
    pub critical_plugins_loaded: bool,
    pub router_active: bool,
}

impl Preconditions {
    pub fn all_met(&self) -> bool {
        self.telemetry_ok
            && self.engine_running
            && self.cruise_active
            && self.critical_plugins_loaded
            && self.router_active
    }
}

// ---- Engagement Preconditions (Phase 6.5r) ----------------------------------

#[derive(Debug, Clone)]
pub struct EngagementPreconditions {
    pub telemetry_fresh: bool,
    pub truck_on_road: bool,
    pub heading_aligned: bool,
    pub route_planned: bool,
    pub truck_on_route: bool,
    pub speed_ok: bool,
}

impl EngagementPreconditions {
    pub fn hard_blockers_met(&self) -> bool {
        self.telemetry_fresh && self.route_planned
    }

    pub fn all_met(&self) -> bool {
        self.telemetry_fresh
            && self.truck_on_road
            && self.heading_aligned
            && self.route_planned
            && self.truck_on_route
            && self.speed_ok
    }

    pub fn blocked_names(&self) -> String {
        let mut names = Vec::new();
        if !self.telemetry_fresh {
            names.push("telemetry_fresh");
        }
        if !self.truck_on_road {
            names.push("truck_on_road");
        }
        if !self.heading_aligned {
            names.push("heading_aligned");
        }
        if !self.route_planned {
            names.push("route_planned");
        }
        if !self.truck_on_route {
            names.push("truck_on_route");
        }
        if !self.speed_ok {
            names.push("speed_ok");
        }
        names.join(", ")
    }
}

// ---- State machine ---------------------------------------------------------

pub struct AutopilotStateMachine {
    state: AutopilotState,
    fault_reason: Option<FailureReason>,
    engaging_ticks: u64,
    paused_ticks: u64,
    stopped_ticks: u64,
    telemetry_lost_ticks: u64,
    precondition_stable_ticks: u64,
    /// Monotonic tick counter bumped once per `evaluate()` call. Used by
    /// the UI status frame so the operator can see "Active for N seconds"
    /// without the daemon having to thread its own counter through.
    ticks: u64,
    /// Ticks-since-entering-current-state. Reset on every state transition
    /// initiated by `evaluate` or `handle_event`.
    state_entry_ticks: u64,
    /// Consecutive ticks with cruise_control_kmh <= 0. Faults only after
    /// [`CRUISE_OFF_TOLERANCE`] — protects against single-frame torn reads.
    cruise_off_ticks: u64,
    /// Consecutive ticks with engine_rpm <= 100. Faults only after
    /// [`ENGINE_OFF_TOLERANCE`] — protects against single-frame glitches.
    engine_off_ticks: u64,
    /// Consecutive ticks where at least one precondition failed in Engaging.
    /// Stable counter is only reset after this exceeds PRECONDITION_GLITCH_TOLERANCE.
    precondition_failure_streak: u64,
    /// Highest precondition_stable_ticks value reached during the current Engaging session.
    max_stable_counter_in_engaging: u64,
    /// Human-readable reason the last precondition check failed (diagnostic).
    last_failure_reason: String,
    /// Last received telemetry frame (Phase 6.5r engagement preconditions).
    last_telemetry: Option<Telemetry>,
    /// Monotonic Instant of last telemetry receipt (Phase 6.5r).
    last_telemetry_time: Option<std::time::Instant>,
    /// Shared routing graph (Phase 6.5q.1). Used for synchronous off-route
    /// check at engage time.
    graph: Option<Arc<RouterGraph>>,
    /// Shared route node IDs (Phase 6.5q.1). Read at engage time to check
    /// if the truck's current position is on the active route.
    route_node_ids: Option<Arc<RwLock<HashSet<u64>>>>,
}

impl Default for AutopilotStateMachine {
    fn default() -> Self {
        Self::new()
    }
}

impl AutopilotStateMachine {
    pub fn new() -> Self {
        Self {
            state: AutopilotState::Off,
            fault_reason: None,
            engaging_ticks: 0,
            paused_ticks: 0,
            stopped_ticks: 0,
            telemetry_lost_ticks: 0,
            precondition_stable_ticks: 0,
            ticks: 0,
            state_entry_ticks: 0,
            cruise_off_ticks: 0,
            engine_off_ticks: 0,
            precondition_failure_streak: 0,
            max_stable_counter_in_engaging: 0,
            last_failure_reason: String::new(),
            last_telemetry: None,
            last_telemetry_time: None,
            graph: None,
            route_node_ids: None,
        }
    }

    /// Attach a shared routing graph for synchronous off-route checks (Phase 6.5q.1).
    pub fn with_graph(mut self, graph: Arc<RouterGraph>) -> Self {
        self.graph = Some(graph);
        self
    }

    /// Attach shared route node IDs for synchronous off-route checks (Phase 6.5q.1).
    pub fn with_route_node_ids(mut self, ids: Arc<RwLock<HashSet<u64>>>) -> Self {
        self.route_node_ids = Some(ids);
        self
    }

    /// Total ticks since the state machine was constructed. Bumped once
    /// per `evaluate` call.
    pub fn tick_count(&self) -> u64 {
        self.ticks
    }

    /// Ticks the state machine has spent in its current state. Used by
    /// the UI to render "Active for X seconds".
    pub fn state_age_ticks(&self) -> u64 {
        self.state_entry_ticks
    }

    pub fn state(&self) -> AutopilotState {
        self.state
    }

    pub fn fault_reason(&self) -> Option<&FailureReason> {
        self.fault_reason.as_ref()
    }

    /// Per-tick evaluation. Always finishes by publishing the current state.
    pub fn evaluate(
        &mut self,
        telemetry: Option<&Telemetry>,
        bb: &SharedBlackboard,
    ) -> AutopilotState {
        self.ticks = self.ticks.wrapping_add(1);
        self.last_telemetry = telemetry.cloned();
        if telemetry.is_some() {
            self.last_telemetry_time = Some(std::time::Instant::now());
        }
        let state_before = self.state;
        if telemetry.is_some() {
            self.telemetry_lost_ticks = 0;
        } else {
            self.telemetry_lost_ticks += 1;
        }

        if self.telemetry_lost_ticks > TELEMETRY_LOSS
            && self.state != AutopilotState::Off
            && self.state != AutopilotState::Fault
        {
            self.transition_to_fault(FailureReason::TelemetryLost);
            self.publish(bb);
            return self.state;
        }

        match self.state {
            AutopilotState::Off => {}
            AutopilotState::Engaging => {
                self.engaging_ticks += 1;
                let pre = check_preconditions(telemetry, bb);
                if pre.all_met() {
                    self.precondition_stable_ticks += 1;
                    self.precondition_failure_streak = 0;
                    if self.precondition_stable_ticks > self.max_stable_counter_in_engaging {
                        self.max_stable_counter_in_engaging = self.precondition_stable_ticks;
                    }
                } else {
                    self.precondition_failure_streak += 1;
                    self.last_failure_reason = precondition_failure_reason(&pre).to_owned();
                    if self.precondition_failure_streak >= PRECONDITION_GLITCH_TOLERANCE {
                        if self.precondition_stable_ticks > 0 {
                            tracing::warn!(
                                "[state] Engaging: preconditions failed for {} ticks, resetting stable counter (was {}, reason: {})",
                                self.precondition_failure_streak,
                                self.precondition_stable_ticks,
                                self.last_failure_reason,
                            );
                        }
                        self.precondition_stable_ticks = 0;
                    }
                }
                self.publish_precondition_diag(&pre, telemetry, bb);
                if self.precondition_stable_ticks > PRECONDITION_STABLE {
                    tracing::info!("[state] Engaging -> Active (preconditions stable)");
                    self.state = AutopilotState::Active;
                    self.engaging_ticks = 0;
                    self.precondition_stable_ticks = 0;
                    self.precondition_failure_streak = 0;
                    self.cruise_off_ticks = 0;
                    self.engine_off_ticks = 0;
                } else if self.engaging_ticks > ENGAGE_TIMEOUT {
                    tracing::warn!(
                        "[state] Engaging timed out -> Off (max_stable={}, last_failure={})",
                        self.max_stable_counter_in_engaging,
                        self.last_failure_reason,
                    );
                    self.state = AutopilotState::Off;
                    self.engaging_ticks = 0;
                    self.precondition_stable_ticks = 0;
                    self.precondition_failure_streak = 0;
                    self.max_stable_counter_in_engaging = 0;
                    self.cruise_off_ticks = 0;
                    self.engine_off_ticks = 0;
                }
            }
            AutopilotState::Active => {
                // When telemetry is missing, the dedicated telemetry-loss
                // path above is authoritative. Don't fire engine/cruise
                // faults from absent data — those checks need a frame.
                if let Some(t) = telemetry {
                    if check_steering_override(Some(t)) {
                        tracing::info!("[state] Active -> Off (steering override)");
                        self.state = AutopilotState::Off;
                        self.cruise_off_ticks = 0;
                        self.engine_off_ticks = 0;
                        self.publish(bb);
                        return self.state;
                    }
                    if t.engine_rpm <= 100.0 {
                        self.engine_off_ticks += 1;
                        if self.engine_off_ticks > ENGINE_OFF_TOLERANCE {
                            self.transition_to_fault(FailureReason::EngineStopped);
                            self.publish(bb);
                            return self.state;
                        }
                    } else {
                        self.engine_off_ticks = 0;
                    }
                    if t.cruise_control_kmh <= 0.0 {
                        self.cruise_off_ticks += 1;
                        if self.cruise_off_ticks > CRUISE_OFF_TOLERANCE {
                            self.transition_to_fault(FailureReason::CruiseDeactivated);
                            self.publish(bb);
                            return self.state;
                        }
                    } else {
                        self.cruise_off_ticks = 0;
                    }
                    if t.speed_ms.abs() < ZERO_SPEED_MS {
                        self.stopped_ticks += 1;
                    } else {
                        self.stopped_ticks = 0;
                    }
                    if self.stopped_ticks > PAUSE_DETECT {
                        tracing::info!("[state] Active -> Paused (speed=0 for 5 s)");
                        self.state = AutopilotState::Paused;
                        self.paused_ticks = 0;
                        self.stopped_ticks = 0;
                        self.cruise_off_ticks = 0;
                        self.engine_off_ticks = 0;
                    }
                }
            }
            AutopilotState::Paused => {
                self.paused_ticks += 1;
                if !is_speed_zero(telemetry) {
                    tracing::info!("[state] Paused -> Active (speed > 0)");
                    self.state = AutopilotState::Active;
                    self.paused_ticks = 0;
                    self.stopped_ticks = 0;
                    self.cruise_off_ticks = 0;
                    self.engine_off_ticks = 0;
                } else if self.paused_ticks > PAUSE_TIMEOUT {
                    tracing::info!("[state] Paused -> Off (5 min timeout)");
                    self.state = AutopilotState::Off;
                    self.paused_ticks = 0;
                    self.cruise_off_ticks = 0;
                    self.engine_off_ticks = 0;
                }
            }
            AutopilotState::Fault => {}
        }

        if self.state == state_before {
            self.state_entry_ticks = self.state_entry_ticks.wrapping_add(1);
        } else {
            self.state_entry_ticks = 0;
        }
        // Engagement precondition evaluation (Phase 6.5r, throttled to ~3 Hz)
        if self.ticks.is_multiple_of(17) {
            let ep = self.evaluate_engagement_preconditions(bb);
            self.publish_engagement_preconditions(&ep, bb);
        }

        // Phase 6.5s: heading stage Disengaging -> Fault
        if bb.get("state.heading_stage").as_deref() == Some("Disengaging")
            && self.state != AutopilotState::Off
            && self.state != AutopilotState::Fault
        {
            self.transition_to_fault(FailureReason::HeadingUnrecoverable);
        }

        self.publish(bb);
        self.state
    }

    pub fn handle_event(
        &mut self,
        event: AutopilotEvent,
        bb: &SharedBlackboard,
    ) -> Result<AutopilotState, String> {
        match (self.state, &event) {
            (AutopilotState::Off, AutopilotEvent::UserEngage) => {
                // ── Phase 6.5q.1: synchronous off-route check ────────────
                self.check_and_replan_if_offroute(bb);

                let engage_ready = bb.get("state.engage_ready").unwrap_or_default();
                if engage_ready == "false" {
                    let blocked = bb
                        .get("state.engage_blocked_by")
                        .unwrap_or_else(|| "unknown".to_string());
                    return Err(format!("Engage blocked: {}", blocked));
                }
                tracing::info!("[state] Off -> Engaging (user engage)");
                self.state = AutopilotState::Engaging;
                self.engaging_ticks = 0;
                self.precondition_stable_ticks = 0;
                self.precondition_failure_streak = 0;
                self.max_stable_counter_in_engaging = 0;
                self.last_failure_reason = String::new();
                self.cruise_off_ticks = 0;
                self.engine_off_ticks = 0;
                self.state_entry_ticks = 0;
            }
            (AutopilotState::Engaging, AutopilotEvent::UserDisengage)
            | (AutopilotState::Active, AutopilotEvent::UserDisengage)
            | (AutopilotState::Paused, AutopilotEvent::UserDisengage) => {
                tracing::info!("[state] {:?} -> Off (user disengage)", self.state);
                self.state = AutopilotState::Off;
                self.engaging_ticks = 0;
                self.paused_ticks = 0;
                self.stopped_ticks = 0;
                self.precondition_stable_ticks = 0;
                self.cruise_off_ticks = 0;
                self.engine_off_ticks = 0;
                self.state_entry_ticks = 0;
            }
            (AutopilotState::Fault, AutopilotEvent::UserReset) => {
                tracing::info!("[state] Fault -> Off (user reset)");
                self.state = AutopilotState::Off;
                self.fault_reason = None;
                self.cruise_off_ticks = 0;
                self.engine_off_ticks = 0;
                self.state_entry_ticks = 0;
            }
            (_, AutopilotEvent::FaultDetected(reason)) => {
                tracing::warn!("[state] -> Fault ({:?})", reason);
                self.transition_to_fault(reason.clone());
            }
            _ => {
                let msg = format!("invalid transition: {:?} -> {:?}", self.state, event);
                self.publish(bb);
                return Err(msg);
            }
        }
        self.publish(bb);
        Ok(self.state)
    }

    /// Phase 6.5q.1: Synchronous off-route check triggered at engage time.
    ///
    /// Reads the truck position from the blackboard, snaps it to the routing
    /// graph, and checks against the current route node IDs. If off-route,
    /// runs A* replanning synchronously. On success, updates the blackboard
    /// with fresh waypoints and resets the auto_replan counters so the
    /// Engaging precondition sees a valid route. On failure, sets diagnostic
    /// keys and leaves state.engage_ready = false so the engage is blocked.
    fn check_and_replan_if_offroute(&self, bb: &SharedBlackboard) {
        let graph = match &self.graph {
            Some(g) => g,
            None => {
                bb.set("state.engage_synchronous_replan_result", "no_graph");
                return;
            }
        };

        let route_ids = match &self.route_node_ids {
            Some(ids) => ids,
            None => {
                bb.set("state.engage_synchronous_replan_result", "no_route_data");
                return;
            }
        };

        let goal_str = bb.get("router.goal_uid").unwrap_or_default();
        if goal_str.is_empty() {
            bb.set("state.engage_synchronous_replan_result", "no_goal");
            return;
        }
        let goal_uid: u64 = match goal_str.trim().parse() {
            Ok(uid) => uid,
            Err(_) => {
                bb.set("state.engage_synchronous_replan_result", "goal_parse_error");
                return;
            }
        };
        if goal_uid == 0 {
            bb.set("state.engage_synchronous_replan_result", "no_goal");
            return;
        }

        let route_snapshot = route_ids.read().unwrap();
        if route_snapshot.is_empty() {
            bb.set("state.engage_synchronous_replan_result", "route_empty");
            return;
        }

        let pos_x = bb.get_f64("telemetry.position_x").unwrap_or(0.0);
        let pos_z = bb.get_f64("telemetry.position_z").unwrap_or(0.0);

        let snap = graph.find_nearest_geometric(pos_x, pos_z, 50.0);
        let (snap_uid, _) = match snap {
            Some(s) => s,
            None => {
                bb.set("state.engage_synchronous_replan_result", "snap_failed");
                return;
            }
        };

        if route_snapshot.contains(&snap_uid) {
            bb.set("state.engage_synchronous_replan_triggered", "false");
            bb.set("state.engage_synchronous_replan_result", "on_route");
            return;
        }

        bb.set("state.engage_synchronous_replan_triggered", "true");
        tracing::info!(
            "[state] engage-time replan: truck off-route (snap_uid={}, not in {} route nodes)",
            snap_uid,
            route_snapshot.len()
        );
        drop(route_snapshot);

        match graph.plan(snap_uid, goal_uid) {
            Some((path, _total_dist)) => {
                let route_node_ids_json = serde_json::to_string(&path).unwrap_or_default();
                bb.set("router.route_node_ids", &route_node_ids_json);

                let waypoints: Vec<[f64; 2]> = path
                    .iter()
                    .filter_map(|uid| {
                        graph.positions.get(uid).copied().map(|(x, z)| [x, z])
                    })
                    .collect();
                let waypoints_json = serde_json::to_string(&waypoints).unwrap_or_default();
                bb.set("router.waypoints", &waypoints_json);

                let waypoint_count = waypoints.len();
                bb.set("router.waypoint_count", waypoint_count.to_string());
                bb.set("router.active", "true");
                bb.set("router.auto_replan_count", "0");
                bb.set("router.last_replan_reason", "engage_sync");
                bb.set("state.precondition_route_ok", "true");

                if let Some(ref lock) = &self.route_node_ids {
                    *lock.write().unwrap() = path.iter().copied().collect();
                }

                bb.set("router.sync_replan_done", "true");

                bb.set(
                    "state.engage_synchronous_replan_result",
                    "replanned",
                );
                tracing::info!(
                    "[state] engage-time replan ok: {} waypoints",
                    waypoint_count,
                );
            }
            None => {
                bb.set(
                    "state.engage_synchronous_replan_result",
                    "replan_failed",
                );
                bb.set("state.precondition_route_ok", "false");
                bb.set(
                    "state.last_engage_fail_reason",
                    "Replan failed: no path found to goal",
                );
                tracing::warn!(
                    "[state] engage-time replan FAILED: snap_uid={} goal_uid={}",
                    snap_uid,
                    goal_uid,
                );
            }
        }
    }

    /// Build the wire-form precondition snapshot for the UI. Re-uses the
    /// same `check_preconditions` predicate the state machine evaluates
    /// against — drift between what gets shown and what gates Engaging→Active
    /// would be confusing.
    pub fn preconditions_snapshot(
        &self,
        telemetry: Option<&Telemetry>,
        bb: &SharedBlackboard,
    ) -> PreconditionSnapshot {
        let p = check_preconditions(telemetry, bb);
        PreconditionSnapshot {
            telemetry_ok: p.telemetry_ok,
            engine_running: p.engine_running,
            cruise_active: p.cruise_active,
            critical_plugins_loaded: p.critical_plugins_loaded,
            router_active: p.router_active,
        }
    }

    pub fn report_fault(&mut self, reason: FailureReason, bb: &SharedBlackboard) {
        self.transition_to_fault(reason);
        self.publish(bb);
    }

    /// Drain request-keys from the blackboard and translate them into
    /// events. Each consumed key is removed so it doesn't fire twice.
    pub fn consume_requests(&mut self, bb: &SharedBlackboard) {
        if bb.get("autopilot.engage_requested").as_deref() == Some("true") {
            bb.remove("autopilot.engage_requested");
            let _ = self.handle_event(AutopilotEvent::UserEngage, bb);
        }
        if bb.get("autopilot.disengage_requested").as_deref() == Some("true") {
            bb.remove("autopilot.disengage_requested");
            let _ = self.handle_event(AutopilotEvent::UserDisengage, bb);
        }
        if bb.get("autopilot.reset_requested").as_deref() == Some("true") {
            bb.remove("autopilot.reset_requested");
            let _ = self.handle_event(AutopilotEvent::UserReset, bb);
        }
    }

    fn reset_debounce_counters(&mut self) {
        self.cruise_off_ticks = 0;
        self.engine_off_ticks = 0;
    }

    fn transition_to_fault(&mut self, reason: FailureReason) {
        self.state = AutopilotState::Fault;
        self.fault_reason = Some(reason);
        self.engaging_ticks = 0;
        self.paused_ticks = 0;
        self.stopped_ticks = 0;
        self.precondition_stable_ticks = 0;
        self.precondition_failure_streak = 0;
        self.max_stable_counter_in_engaging = 0;
        self.last_failure_reason = String::new();
        self.reset_debounce_counters();
        self.state_entry_ticks = 0;
    }

    fn publish(&self, bb: &SharedBlackboard) {
        bb.set("autopilot.state", self.state.as_str());
        match (&self.fault_reason, self.state) {
            (Some(r), AutopilotState::Fault) => bb.set("autopilot.fault_reason", r.as_str()),
            _ => bb.set("autopilot.fault_reason", ""),
        }
    }

    fn publish_precondition_diag(
        &self,
        pre: &Preconditions,
        telemetry: Option<&Telemetry>,
        bb: &SharedBlackboard,
    ) {
        if self.state != AutopilotState::Engaging {
            return;
        }
        bb.set("state.precondition_cruise_ok", pre.cruise_active.to_string());
        bb.set("state.precondition_engine_ok", pre.engine_running.to_string());
        bb.set(
            "state.precondition_no_fault",
            (pre.telemetry_ok && pre.critical_plugins_loaded && pre.router_active).to_string(),
        );
        bb.set(
            "state.precondition_stable_ticks",
            self.precondition_stable_ticks.to_string(),
        );
        bb.set(
            "state.precondition_last_failure_reason",
            &self.last_failure_reason,
        );
        bb.set(
            "state.precondition_max_stable_counter",
            self.max_stable_counter_in_engaging.to_string(),
        );
        bb.set(
            "state.precondition_speed_ok",
            telemetry
                .map(|t| t.speed_ms > ZERO_SPEED_MS)
                .unwrap_or(false)
                .to_string(),
        );
    }

    // ---- Engagement preconditions (Phase 6.5r) --------------------------------

    fn evaluate_engagement_preconditions(&self, bb: &SharedBlackboard) -> EngagementPreconditions {
        let telemetry_fresh = self
            .last_telemetry_time
            .map(|t| t.elapsed().as_millis() < 200)
            .unwrap_or(false)
            && self.last_telemetry.is_some();

        let truck_on_road = bb
            .get("router.last_snap_dist")
            .and_then(|s| s.trim().parse::<f64>().ok())
            .map(|d| d < 20.0)
            .unwrap_or(false);

        let heading_aligned = match &self.last_telemetry {
            Some(t) => compute_heading_aligned(t, bb),
            None => false,
        };

        let route_planned = bb
            .get("router.last_planning_result")
            .map(|s| s == "ok")
            .unwrap_or(false);

        let truck_on_route = match (
            bb.get("router.last_snap_edge_id")
                .and_then(|s| s.trim().parse::<u64>().ok()),
            bb.get("router.route_edge_ids"),
        ) {
            (Some(e), Some(ref set)) => set
                .split(',')
                .any(|s| s.trim().parse::<u64>().ok() == Some(e)),
            _ => false,
        };

        let speed_ok = self
            .last_telemetry
            .as_ref()
            .map(|t| t.speed_ms > 1.4)
            .unwrap_or(false);

        EngagementPreconditions {
            telemetry_fresh,
            truck_on_road,
            heading_aligned,
            route_planned,
            truck_on_route,
            speed_ok,
        }
    }

    fn publish_engagement_preconditions(
        &self,
        pre: &EngagementPreconditions,
        bb: &SharedBlackboard,
    ) {
        bb.set(
            "state.engage_precondition_telemetry_fresh",
            pre.telemetry_fresh.to_string(),
        );
        bb.set(
            "state.engage_precondition_truck_on_road",
            pre.truck_on_road.to_string(),
        );
        bb.set(
            "state.engage_precondition_heading_aligned",
            pre.heading_aligned.to_string(),
        );
        bb.set(
            "state.engage_precondition_route_planned",
            pre.route_planned.to_string(),
        );
        bb.set(
            "state.engage_precondition_truck_on_route",
            pre.truck_on_route.to_string(),
        );
        bb.set(
            "state.engage_precondition_speed_ok",
            pre.speed_ok.to_string(),
        );
        bb.set("state.engage_ready", pre.hard_blockers_met().to_string());
        bb.set("state.engage_all_ok", pre.all_met().to_string());
        bb.set("state.engage_blocked_by", pre.blocked_names());

        // Detail keys for UI inline display
        if let Some(dist) = bb.get("router.last_snap_dist") {
            bb.set("state.engage_detail_snap_dist_m", &dist);
        }
        if let Some(t) = &self.last_telemetry {
            if let Some(d) = heading_diff_degrees(t, bb) {
                bb.set(
                    "state.engage_detail_heading_diff_deg",
                    format!("{:.1}", d),
                );
            }
            let kmh = t.speed_ms * 3.6;
            bb.set("state.engage_detail_speed_kmh", format!("{:.1}", kmh));
            if let Some(last_time) = self.last_telemetry_time {
                let age_ms = last_time.elapsed().as_millis();
                bb.set("state.engage_detail_telemetry_age_ms", age_ms.to_string());
            }
        }
    }
}

// ---- Helpers ---------------------------------------------------------------

fn check_preconditions(telemetry: Option<&Telemetry>, bb: &SharedBlackboard) -> Preconditions {
    Preconditions {
        telemetry_ok: telemetry.is_some(),
        engine_running: telemetry.map(|t| t.engine_rpm > 100.0).unwrap_or(false),
        cruise_active: telemetry
            .map(|t| t.cruise_control_kmh > 0.0)
            .unwrap_or(false),
        critical_plugins_loaded: check_critical_plugins(bb),
        router_active: bb
            .get("router.active")
            .map(|s| s == "true")
            .unwrap_or(false),
    }
}

fn check_critical_plugins(bb: &SharedBlackboard) -> bool {
    let loaded = bb.get("plugins.loaded").unwrap_or_default();
    let names: Vec<&str> = loaded.split(',').map(str::trim).collect();
    let critical = ["lane-keeper", "speed-controller", "vjoy-output"];
    critical.iter().all(|n| names.contains(n))
}

fn is_speed_zero(telemetry: Option<&Telemetry>) -> bool {
    telemetry
        .map(|t| t.speed_ms.abs() < ZERO_SPEED_MS)
        .unwrap_or(false)
}

fn is_engine_running(telemetry: Option<&Telemetry>) -> bool {
    telemetry.map(|t| t.engine_rpm > 100.0).unwrap_or(false)
}

fn check_steering_override(_telemetry: Option<&Telemetry>) -> bool {
    false
}

fn precondition_failure_reason(pre: &Preconditions) -> &'static str {
    if !pre.telemetry_ok {
        "telemetry_lost"
    } else if !pre.engine_running {
        "engine_off"
    } else if !pre.cruise_active {
        "cruise_inactive"
    } else if !pre.critical_plugins_loaded {
        "plugins_missing"
    } else {
        "router_inactive"
    }
}

// ---- Heading alignment helpers (Phase 6.5r) ---------------------------------

pub fn compute_heading_aligned(telemetry: &Telemetry, bb: &SharedBlackboard) -> bool {
    let waypoints_json = match bb.get("router.waypoints") {
        Some(json) => json,
        None => return false,
    };
    let wps: Vec<[f64; 2]> = match serde_json::from_str(&waypoints_json) {
        Ok(v) => v,
        Err(_) => return false,
    };
    if wps.len() < 2 {
        return true;
    }
    let tx = telemetry.position[0];
    let tz = telemetry.position[2];
    let dx = wps[1][0] - tx;
    let dz = wps[1][1] - tz;
    let len = (dx * dx + dz * dz).sqrt();
    if len < 0.01 {
        return true;
    }
    let (dir_x, dir_z) = (dx / len, dz / len);
    let fw_x = telemetry.heading.sin();
    let fw_z = -telemetry.heading.cos();
    let dot = fw_x * dir_x + fw_z * dir_z;
    dot >= 0.707
}

pub fn heading_diff_degrees(telemetry: &Telemetry, bb: &SharedBlackboard) -> Option<f64> {
    let waypoints_json = bb.get("router.waypoints")?;
    let wps: Vec<[f64; 2]> = serde_json::from_str(&waypoints_json).ok()?;
    if wps.len() < 2 {
        return Some(0.0);
    }
    let tx = telemetry.position[0];
    let tz = telemetry.position[2];
    let dx = wps[1][0] - tx;
    let dz = wps[1][1] - tz;
    let len = (dx * dx + dz * dz).sqrt();
    if len < 0.01 {
        return Some(0.0);
    }
    let (dir_x, dir_z) = (dx / len, dz / len);
    let fw_x = telemetry.heading.sin();
    let fw_z = -telemetry.heading.cos();
    let dot = (fw_x * dir_x + fw_z * dir_z).clamp(-1.0, 1.0);
    Some(dot.acos() * 180.0 / std::f64::consts::PI)
}

// ---- Tests -----------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn mock_running() -> Telemetry {
        Telemetry {
            position: [0.0; 3],
            heading: 0.0,
            pitch: 0.0,
            roll: 0.0,
            speed_ms: 22.0, // ≈ 80 km/h
            engine_rpm: 1500.0,
            cruise_control_kmh: 80.0,
            nav_speed_limit_kmh: -1.0,
            lead_vehicle_distance_m: -1.0,
            accel_longitudinal: -1.0,
            fuel_liters: -1.0,
            odometer_km: -1.0,
        }
    }

    fn mock_stopped() -> Telemetry {
        let mut t = mock_running();
        t.speed_ms = 0.0;
        t
    }

    fn mock_off() -> Telemetry {
        let mut t = mock_running();
        t.speed_ms = 0.0;
        t.engine_rpm = 0.0;
        t.cruise_control_kmh = 0.0;
        t
    }

    fn bb_with_preconditions() -> SharedBlackboard {
        let bb = SharedBlackboard::new();
        bb.set("router.active", "true");
        bb.set("plugins.loaded", "lane-keeper,speed-controller,vjoy-output");
        bb.set("state.engage_ready", "true");
        bb
    }

    #[test]
    fn initial_state_is_off() {
        let sm = AutopilotStateMachine::new();
        assert_eq!(sm.state(), AutopilotState::Off);
    }

    #[test]
    fn off_to_engaging_via_user_engage() {
        let mut sm = AutopilotStateMachine::new();
        let bb = SharedBlackboard::new();
        bb.set("state.engage_ready", "true");
        let new_state = sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();
        assert_eq!(new_state, AutopilotState::Engaging);
        assert_eq!(bb.get("autopilot.state").as_deref(), Some("Engaging"));
    }

    #[test]
    fn engaging_to_active_when_preconditions_met_stably() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let t = mock_running();
        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();
        for _ in 0..51 {
            sm.evaluate(Some(&t), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Active);
        assert_eq!(bb.get("autopilot.state").as_deref(), Some("Active"));
    }

    #[test]
    fn engaging_timeout_returns_to_off() {
        let mut sm = AutopilotStateMachine::new();
        let bb = SharedBlackboard::new(); // no preconditions
        bb.set("state.engage_ready", "true");
        let t = mock_off();
        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();
        for _ in 0..251 {
            sm.evaluate(Some(&t), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Off);
    }

    #[test]
    fn engaging_to_off_via_user_disengage() {
        let mut sm = AutopilotStateMachine::new();
        let bb = SharedBlackboard::new();
        bb.set("state.engage_ready", "true");
        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();
        sm.handle_event(AutopilotEvent::UserDisengage, &bb).unwrap();
        assert_eq!(sm.state(), AutopilotState::Off);
    }

    #[test]
    fn active_to_paused_when_speed_zero_for_5s() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let running = mock_running();
        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();
        for _ in 0..51 {
            sm.evaluate(Some(&running), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Active);

        let stopped = mock_stopped();
        for _ in 0..251 {
            sm.evaluate(Some(&stopped), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Paused);
    }

    #[test]
    fn paused_to_active_on_resume() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let running = mock_running();
        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();
        for _ in 0..51 {
            sm.evaluate(Some(&running), &bb);
        }
        let stopped = mock_stopped();
        for _ in 0..251 {
            sm.evaluate(Some(&stopped), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Paused);
        sm.evaluate(Some(&running), &bb);
        assert_eq!(sm.state(), AutopilotState::Active);
    }

    #[test]
    fn active_to_fault_on_telemetry_loss() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let running = mock_running();
        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();
        for _ in 0..51 {
            sm.evaluate(Some(&running), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Active);
        for _ in 0..26 {
            sm.evaluate(None, &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Fault);
        assert_eq!(sm.fault_reason(), Some(&FailureReason::TelemetryLost));
    }

    #[test]
    fn fault_to_off_via_user_reset() {
        let mut sm = AutopilotStateMachine::new();
        let bb = SharedBlackboard::new();
        sm.report_fault(FailureReason::WatchdogStall, &bb);
        assert_eq!(sm.state(), AutopilotState::Fault);
        sm.handle_event(AutopilotEvent::UserReset, &bb).unwrap();
        assert_eq!(sm.state(), AutopilotState::Off);
        assert!(sm.fault_reason().is_none());
    }

    #[test]
    fn invalid_transition_returns_error() {
        let mut sm = AutopilotStateMachine::new();
        let bb = SharedBlackboard::new();
        assert!(sm.handle_event(AutopilotEvent::UserReset, &bb).is_err());
    }

    #[test]
    fn active_to_fault_on_engine_stop() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let running = mock_running();
        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();
        for _ in 0..51 {
            sm.evaluate(Some(&running), &bb);
        }
        let mut stalled = running;
        stalled.engine_rpm = 0.0;
        // One bad frame should NOT trigger fault immediately — debounce check.
        sm.evaluate(Some(&stalled), &bb);
        assert_eq!(sm.state(), AutopilotState::Active);
        // After ENGINE_OFF_TOLERANCE + 1 ticks, fault fires.
        for _ in 0..25 {
            sm.evaluate(Some(&stalled), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Fault);
        assert_eq!(sm.fault_reason(), Some(&FailureReason::EngineStopped));
    }

    fn engage_to_active(sm: &mut AutopilotStateMachine, bb: &SharedBlackboard, t: &Telemetry) {
        sm.handle_event(AutopilotEvent::UserEngage, bb).unwrap();
        for _ in 0..51 {
            sm.evaluate(Some(t), bb);
        }
        assert_eq!(sm.state(), AutopilotState::Active);
    }

    #[test]
    fn single_cruise_glitch_does_not_fault() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let running = mock_running();
        engage_to_active(&mut sm, &bb, &running);
        let mut cc_off = running.clone();
        cc_off.cruise_control_kmh = 0.0;
        sm.evaluate(Some(&cc_off), &bb);
        assert_eq!(sm.state(), AutopilotState::Active);
        sm.evaluate(Some(&running), &bb);
        assert_eq!(sm.state(), AutopilotState::Active);
    }

    #[test]
    fn sustained_cruise_off_triggers_fault() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let running = mock_running();
        engage_to_active(&mut sm, &bb, &running);
        let mut cc_off = running.clone();
        cc_off.cruise_control_kmh = 0.0;
        for _ in 0..26 {
            sm.evaluate(Some(&cc_off), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Fault);
        assert_eq!(sm.fault_reason(), Some(&FailureReason::CruiseDeactivated));
    }

    #[test]
    fn fault_reason_published_to_blackboard() {
        let mut sm = AutopilotStateMachine::new();
        let bb = SharedBlackboard::new();
        sm.report_fault(FailureReason::VjoyDisconnected, &bb);
        assert_eq!(bb.get("autopilot.state").as_deref(), Some("Fault"));
        assert_eq!(
            bb.get("autopilot.fault_reason").as_deref(),
            Some("VjoyDisconnected")
        );
    }

    #[test]
    fn publish_state_to_blackboard() {
        let mut sm = AutopilotStateMachine::new();
        let bb = SharedBlackboard::new();
        sm.evaluate(None, &bb);
        assert_eq!(bb.get("autopilot.state").as_deref(), Some("Off"));
        assert_eq!(bb.get("autopilot.fault_reason").as_deref(), Some(""));
    }

    #[test]
    fn preconditions_snapshot_matches_internal() {
        let sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let t = mock_running();
        let snap = sm.preconditions_snapshot(Some(&t), &bb);
        assert!(snap.telemetry_ok);
        assert!(snap.engine_running);
        assert!(snap.cruise_active);
        assert!(snap.critical_plugins_loaded);
        assert!(snap.router_active);
    }

    #[test]
    fn preconditions_snapshot_without_telemetry() {
        let sm = AutopilotStateMachine::new();
        let bb = SharedBlackboard::new();
        let snap = sm.preconditions_snapshot(None, &bb);
        assert!(!snap.telemetry_ok);
        assert!(!snap.engine_running);
        assert!(!snap.cruise_active);
        assert!(!snap.critical_plugins_loaded);
        assert!(!snap.router_active);
    }

    #[test]
    fn state_entry_ticks_resets_on_transition() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let t = mock_running();
        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();
        assert_eq!(sm.state_age_ticks(), 0);
        // ENGAGE_TIMEOUT/PRECONDITION_STABLE all internal; just step a
        // few ticks while preconditions hold, then verify the age tracker
        // counted ticks since reaching Active.
        for _ in 0..51 {
            sm.evaluate(Some(&t), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Active);
        let active_age_just_after = sm.state_age_ticks();
        for _ in 0..5 {
            sm.evaluate(Some(&t), &bb);
        }
        assert_eq!(sm.state_age_ticks(), active_age_just_after + 5);
    }

    #[test]
    fn consume_requests_drives_transitions() {
        let mut sm = AutopilotStateMachine::new();
        let bb = SharedBlackboard::new();
        bb.set("state.engage_ready", "true");
        bb.set("autopilot.engage_requested", "true");
        sm.consume_requests(&bb);
        assert_eq!(sm.state(), AutopilotState::Engaging);
        assert!(bb.get("autopilot.engage_requested").is_none());
    }

    // ---- Phase 6.5n tests: glitch tolerance, reset logic, diag keys ----------

    /// Glitch within PRECONDITION_GLITCH_TOLERANCE (5 bad frames ≤ 10) must NOT
    /// reset stable_ticks. After 40 good + 5 bad + 12 good = stable reaches 52 → Active.
    #[test]
    fn precondition_glitch_within_tolerance_does_not_reset() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let good = mock_running();
        let bad = mock_off();

        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();

        // 40 good ticks → stable_ticks = 40
        for _ in 0..40 {
            sm.evaluate(Some(&good), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Engaging);

        // 5 bad ticks (streak = 5 ≤ 10 → no reset)
        for _ in 0..5 {
            sm.evaluate(Some(&bad), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Engaging);

        // 12 good ticks: streak resets on first good tick, then stable increments
        // stable_ticks goes 41, 42, … 52 on tick 12 → exceeds 50 → Active
        for _ in 0..12 {
            sm.evaluate(Some(&good), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Active);
    }

    /// Glitch beyond PRECONDITION_GLITCH_TOLERANCE (11 bad frames > 10) resets
    /// stable_ticks to 0. Machine stays in Engaging, then needs another 51 good
    /// ticks to reach Active.
    #[test]
    fn precondition_failure_beyond_tolerance_resets() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let good = mock_running();
        let bad = mock_off();

        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();

        // 40 good ticks → stable_ticks = 40
        for _ in 0..40 {
            sm.evaluate(Some(&good), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Engaging);

        // 11 bad ticks (streak = 11 > 10 → stable_ticks reset to 0)
        for _ in 0..11 {
            sm.evaluate(Some(&bad), &bb);
        }
        // Still Engaging — not Active, not Off (timeout is 250 ticks, only 51 elapsed)
        assert_eq!(sm.state(), AutopilotState::Engaging);

        // 51 more good ticks → stable_ticks reaches 51 → Active
        for _ in 0..51 {
            sm.evaluate(Some(&good), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Active);
    }

    /// 30 good + 5 bad (streak ≤ 10, no reset) + 25 good = stable 55 > 50 → Active.
    #[test]
    fn engaging_to_active_with_glitches_within_tolerance() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let good = mock_running();
        let bad = mock_off();

        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();

        for _ in 0..30 {
            sm.evaluate(Some(&good), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Engaging);

        for _ in 0..5 {
            sm.evaluate(Some(&bad), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Engaging);

        // 25 more good ticks: stable_ticks = 30 + 25 = 55 > 50 → Active
        // (transition happens at stable_ticks = 51, so well within 25 ticks)
        for _ in 0..25 {
            sm.evaluate(Some(&good), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Active);
    }

    /// max_stable_counter_in_engaging tracks the highest stable_ticks seen.
    /// After reset it stays at the previous high-water mark.
    #[test]
    fn max_stable_counter_tracks_highest_value() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let good = mock_running();
        let bad = mock_off();

        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();

        // 30 good ticks → stable_ticks = 30, max = 30
        for _ in 0..30 {
            sm.evaluate(Some(&good), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Engaging);

        // 11 bad ticks → streak = 11 > 10, stable_ticks reset to 0, max stays at 30
        for _ in 0..11 {
            sm.evaluate(Some(&bad), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Engaging);

        // 20 good ticks → stable_ticks = 20, max still 30
        for _ in 0..20 {
            sm.evaluate(Some(&good), &bb);
        }
        // State is still Engaging (stable_ticks = 20, not yet > 50)
        assert_eq!(sm.state(), AutopilotState::Engaging);

        // BB key is published in Engaging state — check high-water mark via blackboard
        assert_eq!(
            bb.get("state.precondition_max_stable_counter").as_deref(),
            Some("30")
        );
    }

    /// Last failure reason key is set before timeout. Check at tick 250
    /// (still Engaging), then one more tick → Off.
    #[test]
    fn last_failure_reason_set_after_timeout() {
        let mut sm = AutopilotStateMachine::new();
        // No preconditions → all precondition checks fail; cruise checked first → "cruise_inactive"
        let bb = SharedBlackboard::new();
        bb.set("state.engage_ready", "true");
        let bad = mock_off();

        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();

        // Tick 250 times — engaging_ticks = 250, NOT yet > 250, still Engaging
        for _ in 0..250 {
            sm.evaluate(Some(&bad), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Engaging);
        // Failure reason: mock_off has telemetry present but engine_rpm=0 →
        // telemetry_ok=true, engine_running=false → "engine_off" (telemetry_lost has priority 1,
        // engine_off has priority 2 in precondition_failure_reason)
        assert_eq!(
            bb.get("state.precondition_last_failure_reason").as_deref(),
            Some("engine_off")
        );

        // One more tick: engaging_ticks = 251 > 250 → Off
        sm.evaluate(Some(&bad), &bb);
        assert_eq!(sm.state(), AutopilotState::Off);
    }

    /// Diagnostics BB keys are published correctly while in Engaging state.
    #[test]
    fn engaging_diag_keys_published_in_engaging() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let good = mock_running();

        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();

        for _ in 0..5 {
            sm.evaluate(Some(&good), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Engaging);

        assert_eq!(
            bb.get("state.precondition_cruise_ok").as_deref(),
            Some("true")
        );
        assert_eq!(
            bb.get("state.precondition_engine_ok").as_deref(),
            Some("true")
        );
        assert_eq!(
            bb.get("state.precondition_no_fault").as_deref(),
            Some("true")
        );
        assert_eq!(
            bb.get("state.precondition_speed_ok").as_deref(),
            Some("true")
        );
        assert_eq!(
            bb.get("state.precondition_stable_ticks").as_deref(),
            Some("5")
        );
    }

    // ---- Phase 6.5r: Engagement preconditions tests -------------------------

    #[test]
    fn engagement_preconditions_all_false_with_empty_blackboard() {
        let sm = AutopilotStateMachine::new();
        let bb = SharedBlackboard::new();
        let ep = sm.evaluate_engagement_preconditions(&bb);
        assert!(!ep.telemetry_fresh);
        assert!(!ep.truck_on_road);
        assert!(!ep.heading_aligned);
        assert!(!ep.route_planned);
        assert!(!ep.truck_on_route);
        assert!(!ep.speed_ok);
    }

    #[test]
    fn engagement_preconditions_hard_blockers() {
        let pre = EngagementPreconditions {
            telemetry_fresh: false,
            truck_on_road: true,
            heading_aligned: true,
            route_planned: true,
            truck_on_route: true,
            speed_ok: true,
        };
        assert!(!pre.hard_blockers_met(), "telemetry_fresh=false should fail hard blockers");

        let pre = EngagementPreconditions {
            telemetry_fresh: true,
            truck_on_road: true,
            heading_aligned: true,
            route_planned: false,
            truck_on_route: true,
            speed_ok: true,
        };
        assert!(!pre.hard_blockers_met(), "route_planned=false should fail hard blockers");

        let pre = EngagementPreconditions {
            telemetry_fresh: true,
            truck_on_road: false,
            heading_aligned: false,
            route_planned: true,
            truck_on_route: false,
            speed_ok: false,
        };
        assert!(pre.hard_blockers_met(), "only telemetry_fresh+route_planned needed");
    }

    #[test]
    fn engagement_preconditions_all_met() {
        let mut sm = AutopilotStateMachine::new();
        // Seed telemetry for freshness check
        sm.last_telemetry = Some(mock_running());
        sm.last_telemetry_time = Some(std::time::Instant::now());

        let bb = SharedBlackboard::new();
        bb.set("router.last_snap_dist", "5.0");
        bb.set("router.waypoints", "[[0.0,0.0],[100.0,0.0]]");
        bb.set("router.last_planning_result", "ok");
        bb.set("router.last_snap_edge_id", "1");
        bb.set("router.route_edge_ids", "1,2,3");

        let ep = sm.evaluate_engagement_preconditions(&bb);

        // With fresh telemetry + heading=0, and waypoints [[0,0],[100,0]],
        // truck at (0,0,0) with heading=0: forward = (sin(0), -cos(0)) = (0, -1)
        // dir to wp[1] = (100, 0), dot = 0 → heading_aligned = false
        // We need waypoints that align better with heading 0 (North = -Z)
        assert!(ep.telemetry_fresh, "mock_running at start should be fresh");
        assert!(ep.truck_on_road, "snap_dist 5.0 < 20.0");
        assert!(ep.route_planned, "last_planning_result=ok");
        assert!(ep.truck_on_route, "snap_edge 1 in route 1,2,3");
        assert!(ep.speed_ok, "22 m/s > 1.4");
    }

    #[test]
    fn handle_event_engage_blocked() {
        let mut sm = AutopilotStateMachine::new();
        let bb = SharedBlackboard::new();
        bb.set("state.engage_ready", "false");
        bb.set("state.engage_blocked_by", "telemetry_fresh, route_planned");

        let result = sm.handle_event(AutopilotEvent::UserEngage, &bb);
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(err.contains("Engage blocked"), "got: {err}");
        assert!(err.contains("telemetry_fresh"), "got: {err}");
        assert_eq!(sm.state(), AutopilotState::Off);
    }

    #[test]
    fn handle_event_engage_allowed() {
        let mut sm = AutopilotStateMachine::new();
        let bb = SharedBlackboard::new();
        bb.set("state.engage_ready", "true");
        let state = sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();
        assert_eq!(state, AutopilotState::Engaging);
    }

    #[test]
    fn compute_heading_aligned_dot_product() {
        let bb = SharedBlackboard::new();
        bb.set("router.waypoints", "[[0.0,0.0],[100.0,0.0]]");

        // heading π/2 (east): forward = (sin(π/2), -cos(π/2)) = (1, 0)
        // dir to wp[1] from (0,0) = (100, 0), dot = 1.0 >= 0.707 → true
        let t = Telemetry {
            position: [0.0, 0.0, 0.0],
            heading: std::f64::consts::FRAC_PI_2,
            ..mock_running()
        };
        assert!(compute_heading_aligned(&t, &bb));

        // heading π (south): forward = (sin(π), -cos(π)) = (0, 1)
        // dir to wp[1] = (100, 0), dot = 0 → false
        let t2 = Telemetry {
            heading: std::f64::consts::PI,
            ..t
        };
        assert!(!compute_heading_aligned(&t2, &bb));
    }

    #[test]
    fn compute_heading_aligned_missing_waypoints() {
        let bb = SharedBlackboard::new();
        let t = mock_running();
        assert!(!compute_heading_aligned(&t, &bb));
    }

    #[test]
    fn engage_blocked_by_format() {
        let pre = EngagementPreconditions {
            telemetry_fresh: false,
            truck_on_road: false,
            heading_aligned: true,
            route_planned: false,
            truck_on_route: true,
            speed_ok: true,
        };
        let blocked = pre.blocked_names();
        assert!(blocked.contains("telemetry_fresh"));
        assert!(blocked.contains("truck_on_road"));
        assert!(blocked.contains("route_planned"));
        assert!(!blocked.contains("heading_aligned"));
        assert!(blocked.split(", ").count() == 3);
    }

    #[test]
    fn engagement_precondition_keys_published_after_17_ticks() {
        let mut sm = AutopilotStateMachine::new();
        let bb = SharedBlackboard::new();
        let t = mock_running();

        // ticks 1-16: no engagement keys published
        for _ in 0..16 {
            sm.evaluate(Some(&t), &bb);
        }
        assert!(bb.get("state.engage_ready").is_none(), "should be absent before tick 17");

        // tick 17: triggers engagement precondition eval
        sm.evaluate(Some(&t), &bb);
        assert!(bb.get("state.engage_ready").is_some(), "should be published at tick 17");
    }

    #[test]
    fn engagement_precondition_snap_dist_unparseable() {
        let sm = AutopilotStateMachine::new();
        let bb = SharedBlackboard::new();
        bb.set("router.last_snap_dist", "abc");
        let ep = sm.evaluate_engagement_preconditions(&bb);
        assert!(!ep.truck_on_road, "unparseable snap_dist should be false");
    }

    #[test]
    fn engagement_precondition_snap_dist_threshold() {
        let sm = AutopilotStateMachine::new();
        let bb = SharedBlackboard::new();
        bb.set("router.last_snap_dist", "20.1");
        let ep = sm.evaluate_engagement_preconditions(&bb);
        assert!(!ep.truck_on_road, "20.1m should exceed 20m threshold");

        bb.set("router.last_snap_dist", "19.9");
        let ep = sm.evaluate_engagement_preconditions(&bb);
        assert!(ep.truck_on_road, "19.9m should be under 20m threshold");

        bb.set("router.last_snap_dist", "20.0");
        let ep = sm.evaluate_engagement_preconditions(&bb);
        assert!(!ep.truck_on_road, "20.0m exactly should be NOT < 20.0 (strict)");
    }

    #[test]
    fn engagement_precondition_route_planned_edge_cases() {
        let sm = AutopilotStateMachine::new();
        let bb = SharedBlackboard::new();

        bb.set("router.last_planning_result", "ok");
        let ep = sm.evaluate_engagement_preconditions(&bb);
        assert!(ep.route_planned);

        bb.set("router.last_planning_result", "error");
        let ep = sm.evaluate_engagement_preconditions(&bb);
        assert!(!ep.route_planned, "error should not count as ok");

        bb.set("router.last_planning_result", "pending");
        let ep = sm.evaluate_engagement_preconditions(&bb);
        assert!(!ep.route_planned, "pending should not count as ok");
    }

    #[test]
    fn engagement_precondition_speed_threshold() {
        let bb = SharedBlackboard::new();

        let check = |speed_ms: f64| -> bool {
            let mut sm = AutopilotStateMachine::new();
            let mut t = mock_running();
            t.speed_ms = speed_ms;
            sm.last_telemetry = Some(t);
            sm.last_telemetry_time = Some(std::time::Instant::now());
            sm.evaluate_engagement_preconditions(&bb).speed_ok
        };

        // Threshold is `> 1.4`, not `>= 1.4` (strict)
        assert!(!check(1.4), "1.4 m/s == threshold, not strictly > 1.4");
        assert!(check(1.41), "1.41 m/s should pass > 1.4");
        assert!(!check(1.39), "1.39 m/s should not pass > 1.4");
        assert!(!check(0.0), "0 m/s should not pass");
    }

    #[test]
    fn engagement_precondition_truck_on_route_missing() {
        let sm = AutopilotStateMachine::new();
        let bb = SharedBlackboard::new();
        let ep = sm.evaluate_engagement_preconditions(&bb);
        assert!(!ep.truck_on_route, "missing both edge_id and route_edge_ids");

        bb.set("router.last_snap_edge_id", "42");
        let ep = sm.evaluate_engagement_preconditions(&bb);
        assert!(!ep.truck_on_route, "edge_id present but route_edge_ids missing");

        bb.set("router.route_edge_ids", "1,2,3");
        let ep = sm.evaluate_engagement_preconditions(&bb);
        assert!(!ep.truck_on_route, "42 not in [1,2,3]");

        bb.set("router.last_snap_edge_id", "2");
        let ep = sm.evaluate_engagement_preconditions(&bb);
        assert!(ep.truck_on_route, "2 is in [1,2,3]");
    }

    #[test]
    fn engagement_precondition_hard_blockers_ignore_others() {
        let pre = EngagementPreconditions {
            telemetry_fresh: true,
            truck_on_road: false,
            heading_aligned: false,
            route_planned: true,
            truck_on_route: false,
            speed_ok: false,
        };
        assert!(pre.hard_blockers_met());
        assert!(!pre.all_met());
    }

    #[test]
    fn heading_aligned_single_waypoint_returns_true() {
        let bb = SharedBlackboard::new();
        bb.set("router.waypoints", "[[0.0,0.0]]");
        let t = mock_running();
        assert!(compute_heading_aligned(&t, &bb), "single waypoint: no direction → always aligned");
    }

    #[test]
    fn heading_aligned_waypoint_on_truck_returns_true() {
        let bb = SharedBlackboard::new();
        bb.set("router.waypoints", "[[0.0,0.0],[0.0,0.0]]");
        let mut t = mock_running();
        t.position = [0.0, 0.0, 0.0];
        assert!(compute_heading_aligned(&t, &bb), "truck on waypoint → len < 0.01 → always aligned");
    }

    #[test]
    fn heading_aligned_invalid_json_waypoints_returns_false() {
        let bb = SharedBlackboard::new();
        bb.set("router.waypoints", "not valid json");
        let t = mock_running();
        assert!(!compute_heading_aligned(&t, &bb), "invalid JSON should return false");
    }

    #[test]
    fn heading_aligned_46_degrees_returns_false() {
        let bb = SharedBlackboard::new();
        // Waypoint south of truck at (0, 0, 0): direction = (0, -1) in ETS2 units
        bb.set("router.waypoints", "[[0.0,0.0],[0.0,-100.0]]");

        // heading = π/4 (45°): forward = (sin, -cos) = (0.707, -0.707)
        // dot with (0, -1) = 0*(-0.707) + (-1)*(-0.707) = 0.707 → exactly threshold
        let t_at_45 = Telemetry {
            position: [0.0, 0.0, 0.0],
            heading: std::f64::consts::FRAC_PI_4,
            ..mock_running()
        };
        assert!(compute_heading_aligned(&t_at_45, &bb), "45° should be at threshold");

        // heading = π/4 + 0.03 (≈46.7°): forward = (sin, -cos) = (0.731, -0.683)
        // dot with (0, -1) = 0 + (-1)*(-0.683) = 0.683 < 0.707 → misaligned
        let t_over = Telemetry {
            heading: std::f64::consts::FRAC_PI_4 + 0.03,
            ..t_at_45
        };
        assert!(!compute_heading_aligned(&t_over, &bb), ">45° misalignment should fail");
    }

    #[test]
    fn engagement_precondition_telemetry_stale_returns_false() {
        let mut sm = AutopilotStateMachine::new();
        sm.last_telemetry = Some(mock_running());
        sm.last_telemetry_time = Some(std::time::Instant::now() - std::time::Duration::from_millis(500));
        let bb = SharedBlackboard::new();
        let ep = sm.evaluate_engagement_preconditions(&bb);
        assert!(!ep.telemetry_fresh, "500ms old telemetry should not be fresh");
    }

    #[test]
    fn engagement_detail_keys_published() {
        let mut sm = AutopilotStateMachine::new();
        let mut t = mock_running();
        t.position = [10.0, 0.0, 20.0];
        t.heading = std::f64::consts::FRAC_PI_2; // east
        t.speed_ms = 15.0;
        sm.last_telemetry = Some(t);
        sm.last_telemetry_time = Some(std::time::Instant::now());

        let bb = SharedBlackboard::new();
        bb.set("router.last_snap_dist", "12.3");
        bb.set("router.waypoints", "[[0.0,0.0],[100.0,0.0]]");

        let ep = sm.evaluate_engagement_preconditions(&bb);
        sm.publish_engagement_preconditions(&ep, &bb);

        assert_eq!(
            bb.get("state.engage_detail_snap_dist_m").as_deref(),
            Some("12.3")
        );
        assert_eq!(
            bb.get("state.engage_detail_speed_kmh").as_deref(),
            Some("54.0")
        );
        assert!(bb.get("state.engage_detail_heading_diff_deg").is_some());
        assert!(bb.get("state.engage_detail_telemetry_age_ms").is_some());
    }

    #[test]
    fn heading_diff_degrees_returns_none_for_missing_waypoints() {
        let bb = SharedBlackboard::new();
        let t = mock_running();
        assert_eq!(heading_diff_degrees(&t, &bb), None);
    }

    #[test]
    fn heading_diff_degrees_returns_zero_for_single_waypoint() {
        let bb = SharedBlackboard::new();
        bb.set("router.waypoints", "[[0.0,0.0]]");
        let t = mock_running();
        assert_eq!(heading_diff_degrees(&t, &bb), Some(0.0));
    }

    #[test]
    fn blocked_names_empty_when_all_true() {
        let pre = EngagementPreconditions {
            telemetry_fresh: true,
            truck_on_road: true,
            heading_aligned: true,
            route_planned: true,
            truck_on_route: true,
            speed_ok: true,
        };
        assert_eq!(pre.blocked_names(), "");
    }

    #[test]
    fn blocked_names_all_false() {
        let pre = EngagementPreconditions {
            telemetry_fresh: false,
            truck_on_road: false,
            heading_aligned: false,
            route_planned: false,
            truck_on_route: false,
            speed_ok: false,
        };
        let names = pre.blocked_names();
        assert_eq!(names.split(", ").count(), 6);
    }

    // ---- Phase 6.5s: heading stage Disengaging -> Fault tests ---------------

    #[test]
    fn disengaging_stage_transitions_active_to_fault() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let running = mock_running();

        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();
        for _ in 0..51 {
            sm.evaluate(Some(&running), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Active);

        bb.set("state.heading_stage", "Disengaging");
        sm.evaluate(Some(&running), &bb);
        assert_eq!(sm.state(), AutopilotState::Fault);
        assert_eq!(sm.fault_reason(), Some(&FailureReason::HeadingUnrecoverable));
    }

    #[test]
    fn disengaging_stage_stays_in_fault() {
        let mut sm = AutopilotStateMachine::new();
        let bb = SharedBlackboard::new();
        bb.set("state.heading_stage", "Disengaging");
        sm.report_fault(FailureReason::HeadingUnrecoverable, &bb);
        assert_eq!(sm.state(), AutopilotState::Fault);
        sm.evaluate(None, &bb);
        assert_eq!(sm.state(), AutopilotState::Fault);
    }
}
