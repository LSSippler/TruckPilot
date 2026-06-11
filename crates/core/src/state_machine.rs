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
const ENGINE_OFF_TOLERANCE: u64 = 25; // 500 ms — debounce before EngineStopped fault
const PRECONDITION_GLITCH_TOLERANCE: u64 = 10; // 200 ms at 50 Hz daemon tick rate
const ZERO_SPEED_MS: f64 = 0.028; // ≈ 0.1 km/h
/// Lane-only Paused-Deadlock-Guard: ab diesem Gas-Wunsch (`speed_controller.throttle_cmd`)
/// gilt der Längsregler als „will aktiv beschleunigen". Solange das im Lane-Only-Modus bei
/// Stillstand der Fall ist, wird der `Active → Paused`-Latch unterdrückt — sonst würde der
/// (an `is_active()` gekoppelte) Speed-Controller abgeschaltet und der Truck käme nie wieder
/// ins Rollen (selbsthaltender Stillstand). Route-Modus ist davon unberührt.
const PAUSE_THROTTLE_EPS: f64 = 0.05;
const ROUTE_TO_LANE_TICKS: u64 = 150; // 3s at 50 Hz
const LANE_TO_ROUTE_TICKS: u64 = 100; // 2s at 50 Hz
const TO_DEGRADED_TICKS: u64 = 250; // 5s at 50 Hz
const DEGRADED_RECOVERY_TICKS: u64 = 100; // 2s at 50 Hz
const DEGRADED_TIMEOUT_TICKS: u64 = 1500; // 30s at 50 Hz

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
    EngineStopped,
    CriticalPluginMissing(String),
    UserRequested,
    VisionLostLongBlackout,
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

// ---- EngageMode ------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EngageMode {
    Route,
    Lane,
    Degraded,
}

impl EngageMode {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Route => "route",
            Self::Lane => "lane",
            Self::Degraded => "degraded",
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
    pub critical_plugins_loaded: bool,
    pub router_active: bool,
    /// Lane-only mode: truck must be within 20m of a known road node.
    /// Always `true` in route/vision mode — only gated for lane_only.
    pub truck_on_road: bool,
}

impl Preconditions {
    pub fn all_met(&self) -> bool {
        self.telemetry_ok
            && self.engine_running
            && self.critical_plugins_loaded
            && self.router_active
            && self.truck_on_road
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
    /// Hard-block: heading diff > 60° — truck faces wrong way, replan won't help.
    pub heading_ok_for_engage: bool,
    /// Vision mode: lane_keeper plugin reports no active fallback (engage_allowed=true).
    pub lane_keeper_engage_allowed: bool,
    /// "vision" | "route_following" (default). Drives which preconditions apply.
    pub mode: String,
}

impl Default for EngagementPreconditions {
    fn default() -> Self {
        Self {
            telemetry_fresh: false,
            truck_on_road: false,
            heading_aligned: false,
            route_planned: false,
            truck_on_route: false,
            speed_ok: false,
            heading_ok_for_engage: false,
            lane_keeper_engage_allowed: false,
            mode: "route_following".into(),
        }
    }
}

impl EngagementPreconditions {
    pub fn hard_blockers_met(&self) -> bool {
        if self.mode == "vision" {
            // Route-related hard blocks are irrelevant; absolute heading guard stays.
            self.telemetry_fresh && self.heading_ok_for_engage
        } else {
            self.telemetry_fresh && self.route_planned && self.heading_ok_for_engage
        }
    }

    pub fn all_met(&self) -> bool {
        if self.mode == "vision" {
            self.telemetry_fresh
                && self.truck_on_road
                && self.heading_ok_for_engage
                && self.speed_ok
                && self.lane_keeper_engage_allowed
        } else {
            self.telemetry_fresh
                && self.truck_on_road
                && self.heading_aligned
                && self.route_planned
                && self.truck_on_route
                && self.speed_ok
                && self.heading_ok_for_engage
        }
    }

    pub fn blocked_names(&self) -> String {
        let mut names = Vec::new();
        if !self.telemetry_fresh {
            names.push("telemetry_fresh");
        }
        if !self.truck_on_road {
            names.push("truck_on_road");
        }
        if self.mode != "vision" {
            if !self.heading_aligned {
                names.push("heading_aligned");
            }
            if !self.route_planned {
                names.push("route_planned");
            }
            if !self.truck_on_route {
                names.push("truck_on_route");
            }
        }
        if !self.speed_ok {
            names.push("speed_ok");
        }
        if !self.heading_ok_for_engage {
            names.push("heading_ok_for_engage");
        }
        if self.mode == "vision" && !self.lane_keeper_engage_allowed {
            names.push("lane_keeper_engage_allowed");
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
    /// Lane-keeper mode captured at engage time ("vision" | "route_following").
    /// Clean Off disengage fires if mode changes while Active.
    mode_at_engage: String,
    /// Set when the user engages via `--lane-only`. Bypasses route preconditions
    /// and locks engage_mode to Lane for the session.
    lane_only_engage: bool,
    engage_mode: EngageMode,
    route_to_lane_ticks: u64,
    lane_to_route_ticks: u64,
    to_degraded_ticks: u64,
    degraded_recovery_ticks: u64,
    degraded_timeout_ticks: u64,
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
            engine_off_ticks: 0,
            precondition_failure_streak: 0,
            max_stable_counter_in_engaging: 0,
            last_failure_reason: String::new(),
            last_telemetry: None,
            last_telemetry_time: None,
            graph: None,
            route_node_ids: None,
            mode_at_engage: String::new(),
            lane_only_engage: false,
            engage_mode: EngageMode::Route,
            route_to_lane_ticks: 0,
            lane_to_route_ticks: 0,
            to_degraded_ticks: 0,
            degraded_recovery_ticks: 0,
            degraded_timeout_ticks: 0,
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
                let pre = check_preconditions(telemetry, bb, self.lane_only_engage);
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
                    // Vision mode Level-4 disengage signal from lane-keeper plugin.
                    if bb.get("lane_keeper.fallback_level").as_deref() == Some("4")
                        && bb.get("lane_keeper.active").as_deref() == Some("false")
                    {
                        self.transition_to_fault(FailureReason::VisionLostLongBlackout);
                        self.publish(bb);
                        return self.state;
                    }

                    // Mode-change while Active → clean Off disengage.
                    if !self.mode_at_engage.is_empty() {
                        let current_mode = bb
                            .get("plugin.lane_keeper.mode")
                            .unwrap_or_else(|| "route_following".to_string());
                        if current_mode != self.mode_at_engage {
                            tracing::info!(
                                "[state] Active -> Off (mode changed: {} -> {})",
                                self.mode_at_engage,
                                current_mode,
                            );
                            self.state = AutopilotState::Off;
                            self.engine_off_ticks = 0;
                            self.publish(bb);
                            return self.state;
                        }
                    }

                    // Lane-only Paused-Deadlock-Guard: den Active→Paused-Latch NICHT
                    // ziehen, solange der Speed-Controller bei Stillstand aktiv Gas
                    // anfordert (Truck steht, will aber losfahren). Der Längsregler
                    // hängt an is_active() — ginge der State nach Paused, würde sein
                    // Gas gekappt und der Truck käme nie wieder ins Rollen. Im Route-
                    // Modus (lane_only_engage == false) ist das Verhalten unverändert:
                    // throttle_demanded bleibt false, der Latch greift wie zuvor.
                    let throttle_demanded = self.lane_only_engage
                        && bb
                            .get_f64("speed_controller.throttle_cmd")
                            .map(|thr| thr > PAUSE_THROTTLE_EPS)
                            .unwrap_or(false);
                    if t.speed_ms.abs() < ZERO_SPEED_MS && !throttle_demanded {
                        self.stopped_ticks += 1;
                    } else {
                        self.stopped_ticks = 0;
                    }
                    bb.set(
                        "autopilot.pause_suppressed",
                        if throttle_demanded && t.speed_ms.abs() < ZERO_SPEED_MS {
                            "lane_only_throttle_demand"
                        } else {
                            ""
                        },
                    );
                    if self.stopped_ticks > PAUSE_DETECT {
                        tracing::info!("[state] Active -> Paused (speed=0 for 5 s)");
                        bb.set("autopilot.paused_reason", "speed_zero_5s");
                        self.state = AutopilotState::Paused;
                        self.paused_ticks = 0;
                        self.stopped_ticks = 0;
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
                    self.engine_off_ticks = 0;
                } else if self.paused_ticks > PAUSE_TIMEOUT {
                    tracing::info!("[state] Paused -> Off (5 min timeout)");
                    self.state = AutopilotState::Off;
                    self.paused_ticks = 0;
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

        self.evaluate_engage_mode(bb);

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
                // Lane-only engage: bypass route/cruise preconditions.
                let lane_only = bb.get("autopilot.requested_mode").as_deref() == Some("lane_only");
                bb.remove("autopilot.requested_mode");

                if lane_only {
                    if self.last_telemetry.is_none() {
                        return Err("Engage blocked: telemetry_lost".into());
                    }
                    // Auto-Moduswechsel: lane_only ⇒ NearestSpline (routerloses Folgen).
                    // Der Lane-Keeper schaltet daraufhin um und publiziert
                    // lane_keeper.engage_allowed aus seinem eigenen Nearest-Hit (60°/20m),
                    // der das truck_on_road-Gate für die Engaging→Active-Transition stellt.
                    // Zurückgesetzt auf route_following beim Disengage (s.u.).
                    bb.set("plugin.lane_keeper.mode", "nearest_spline");
                } else {
                    // Stale NearestSpline-Auto-Mode aus einem vorherigen lane_only-Engage
                    // für einen normalen (Route-)Engage aufräumen — egal wie der vorige
                    // Engage endete (Disengage, Timeout, Fault). Vision (extern gesetzt) bleibt
                    // unberührt. nearest_spline wird AUSSCHLIESSLICH vom lane_only-Auto-Switch
                    // gesetzt, ist hier also immer ein Überbleibsel.
                    if bb.get("plugin.lane_keeper.mode").as_deref() == Some("nearest_spline") {
                        bb.set("plugin.lane_keeper.mode", "route_following");
                    }
                    // ── Phase 6.5q.1: synchronous off-route check ────────────
                    self.check_and_replan_if_offroute(bb);

                    // Vision mode: block engage when lane_keeper reports fallback level >= 2.
                    if bb.get("plugin.lane_keeper.mode").as_deref() == Some("vision")
                        && bb.get("lane_keeper.engage_allowed").as_deref() != Some("true")
                    {
                        let level = bb.get("lane_keeper.fallback_level").unwrap_or_default();
                        tracing::warn!(
                            "[state] Engage blocked: vision fallback_level={} (engage_allowed != true)",
                            level,
                        );
                        return Err(format!(
                            "Engage blocked: lane_keeper.engage_allowed=false (vision level {})",
                            level
                        ));
                    }

                    let engage_ready = bb.get("state.engage_ready").unwrap_or_default();
                    if engage_ready == "false" {
                        let blocked = bb
                            .get("state.engage_blocked_by")
                            .unwrap_or_else(|| "unknown".to_string());
                        return Err(format!("Engage blocked: {}", blocked));
                    }
                }

                tracing::info!("[state] Off -> Engaging (user engage, lane_only={lane_only})");
                self.lane_only_engage = lane_only;
                self.mode_at_engage = bb
                    .get("plugin.lane_keeper.mode")
                    .unwrap_or_else(|| "route_following".to_string());
                self.state = AutopilotState::Engaging;
                self.engaging_ticks = 0;
                self.precondition_stable_ticks = 0;
                self.precondition_failure_streak = 0;
                self.max_stable_counter_in_engaging = 0;
                self.last_failure_reason = String::new();
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
                self.engine_off_ticks = 0;
                self.state_entry_ticks = 0;
                self.lane_only_engage = false;
                // NearestSpline-Auto-Mode beim Disengage zurücknehmen (Vision extern → unberührt).
                if bb.get("plugin.lane_keeper.mode").as_deref() == Some("nearest_spline") {
                    bb.set("plugin.lane_keeper.mode", "route_following");
                }
            }
            (AutopilotState::Fault, AutopilotEvent::UserReset) => {
                tracing::info!("[state] Fault -> Off (user reset)");
                self.state = AutopilotState::Off;
                self.fault_reason = None;
                self.engine_off_ticks = 0;
                self.state_entry_ticks = 0;
                self.lane_only_engage = false;
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

        let in_route = route_snapshot.contains(&snap_uid);
        let heading = self
            .last_telemetry
            .as_ref()
            .map(|t| t.heading)
            .unwrap_or(0.0);
        let ahead = waypoint_ahead_of_truck(bb, pos_x, pos_z, heading);

        if in_route && ahead {
            bb.set("state.engage_synchronous_replan_triggered", "false");
            bb.set("state.engage_synchronous_replan_result", "on_route");
            return;
        }

        if in_route {
            tracing::info!(
                "[state] engage-time: snap_uid={} in route but heading reversed — replanning",
                snap_uid
            );
        } else {
            tracing::info!(
                "[state] engage-time replan: truck off-route (snap_uid={}, not in {} route nodes)",
                snap_uid,
                route_snapshot.len()
            );
        }
        bb.set("state.engage_synchronous_replan_triggered", "true");
        drop(route_snapshot);

        match graph.plan(snap_uid, goal_uid) {
            Some((path, _total_dist)) => {
                let route_node_ids_json = serde_json::to_string(&path).unwrap_or_default();
                bb.set("router.route_node_ids", &route_node_ids_json);

                let waypoints: Vec<[f64; 2]> = path
                    .iter()
                    .filter_map(|uid| graph.positions.get(uid).copied().map(|(x, z)| [x, z]))
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

                bb.set("state.engage_synchronous_replan_result", "replanned");
                tracing::info!(
                    "[state] engage-time replan ok: {} waypoints",
                    waypoint_count,
                );
            }
            None => {
                bb.set("state.engage_synchronous_replan_result", "replan_failed");
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
        let p = check_preconditions(telemetry, bb, self.lane_only_engage);
        PreconditionSnapshot {
            telemetry_ok: p.telemetry_ok,
            engine_running: p.engine_running,
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
        self.engine_off_ticks = 0;
    }

    fn reset_engage_mode_timers(&mut self) {
        self.route_to_lane_ticks = 0;
        self.lane_to_route_ticks = 0;
        self.to_degraded_ticks = 0;
        self.degraded_recovery_ticks = 0;
        self.degraded_timeout_ticks = 0;
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
        self.lane_only_engage = false;
    }

    fn publish(&self, bb: &SharedBlackboard) {
        bb.set("autopilot.state", self.state.as_str());
        match (&self.fault_reason, self.state) {
            (Some(r), AutopilotState::Fault) => bb.set("autopilot.fault_reason", r.as_str()),
            _ => bb.set("autopilot.fault_reason", ""),
        }
        // `paused_reason` ist nur im Paused-State gültig. Bei jedem anderen State
        // (inkl. Disengage→Off, Reset, Fault, 5-min-Timeout) leeren, damit keine
        // veraltete Begründung stehen bleibt. Single source of truth.
        if self.state != AutopilotState::Paused {
            bb.set("autopilot.paused_reason", "");
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
        bb.set(
            "state.precondition_engine_ok",
            pre.engine_running.to_string(),
        );
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

        // Hard-block engage if heading deviation exceeds 60°. None (no waypoints) → allow.
        let heading_ok_for_engage = match &self.last_telemetry {
            Some(t) => heading_diff_degrees(t, bb).is_none_or(|d| d < 60.0),
            None => true,
        };

        let mode = bb
            .get("plugin.lane_keeper.mode")
            .unwrap_or_else(|| "route_following".to_string());
        let lane_keeper_engage_allowed = bb
            .get("lane_keeper.engage_allowed")
            .map(|s| s == "true")
            .unwrap_or(false);

        let mut ep = EngagementPreconditions {
            telemetry_fresh,
            truck_on_road,
            heading_aligned,
            route_planned,
            truck_on_route,
            speed_ok,
            heading_ok_for_engage,
            lane_keeper_engage_allowed,
            mode,
        };
        // Lane-only engage bypasses route/heading conditions — these aren't relevant
        // without a planned route. Safety conditions (truck_on_road, telemetry_fresh,
        // speed_ok) are intentionally NOT bypassed.
        if self.lane_only_engage {
            ep.heading_aligned = true;
            ep.route_planned = true;
            ep.truck_on_route = true;
            // Heading/Distanz-Schutz an den NearestSpline-Engage-Gate des Lane-Keepers
            // binden (statt blind true). Spiegelt das tatsächliche Engaging→Active-Gate
            // (check_preconditions: truck_on_road = lane_keeper.engage_allowed) in die
            // UI-Diagnose + Advisory. route_planned bleibt bypassed (keine Route nötig).
            ep.heading_ok_for_engage = lane_keeper_engage_allowed;
            // ep.lane_keeper_engage_allowed bleibt der echte BB-Wert (kein Force).
        }
        ep
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
        bb.set(
            "state.engage_precondition_heading_ok_for_engage",
            pre.heading_ok_for_engage.to_string(),
        );
        bb.set(
            "state.engage_precondition_lane_keeper_engage_allowed",
            pre.lane_keeper_engage_allowed.to_string(),
        );
        // In vision mode the route/heading_aligned conditions don't apply — publish
        // as "true" so UI rows that aren't hidden yet don't show as blockers.
        if pre.mode == "vision" {
            bb.set("state.engage_precondition_route_planned", "true");
            bb.set("state.engage_precondition_truck_on_route", "true");
            bb.set("state.engage_precondition_heading_aligned", "true");
        }
        // Lane-only engage: route/heading conditions are irrelevant — force them to
        // "true" in the diagnostic output so preflight checks don't block the engage.
        if self.lane_only_engage {
            bb.set("state.engage_precondition_route_planned", "true");
            bb.set("state.engage_precondition_truck_on_route", "true");
            bb.set("state.engage_precondition_heading_aligned", "true");
            // heading_ok_for_engage + lane_keeper_engage_allowed werden NICHT mehr auf
            // "true" geforced — sie spiegeln jetzt den echten NearestSpline-Gate
            // (lane_keeper.engage_allowed), publiziert vom allgemeinen Block oben.
        }
        bb.set("state.engage_ready", pre.hard_blockers_met().to_string());
        bb.set("state.engage_all_ok", pre.all_met().to_string());
        bb.set("state.engage_blocked_by", pre.blocked_names());

        // Advisory message — highest-priority reason the truck can't engage.
        // Route-related advisories are suppressed in vision mode.
        let advisory = if pre.mode != "vision"
            && bb.get("router.last_planning_result").as_deref() == Some("start_node_unknown")
        {
            "Truck-Position nicht im Routing-Graph. Fahre auf eine Hauptstrasse."
        } else if !pre.heading_ok_for_engage {
            "Drehe Truck in Stra\u{00df}en-Richtung (Abweichung >60\u{00b0})"
        } else if pre.mode == "vision" && !pre.lane_keeper_engage_allowed {
            "Lane-Keeper nicht bereit (engage_allowed=false). Pr\u{00fc}fe Kamera-Signal."
        } else {
            ""
        };
        bb.set("state.engage_advisory", advisory);

        // Detail keys for UI inline display
        if let Some(dist) = bb.get("router.last_snap_dist") {
            bb.set("state.engage_detail_snap_dist_m", &dist);
        }
        if let Some(t) = &self.last_telemetry {
            if let Some(d) = heading_diff_degrees(t, bb) {
                bb.set("state.engage_detail_heading_diff_deg", format!("{:.1}", d));
            }
            let kmh = t.speed_ms * 3.6;
            bb.set("state.engage_detail_speed_kmh", format!("{:.1}", kmh));
            if let Some(last_time) = self.last_telemetry_time {
                let age_ms = last_time.elapsed().as_millis();
                bb.set("state.engage_detail_telemetry_age_ms", age_ms.to_string());
            }
        }
    }

    fn transition_engage_mode(&mut self, new_mode: EngageMode, bb: &SharedBlackboard) {
        tracing::info!("[engage_mode] {:?} -> {:?}", self.engage_mode, new_mode);
        self.engage_mode = new_mode;
        self.reset_engage_mode_timers();
        if matches!(new_mode, EngageMode::Lane | EngageMode::Route) {
            bb.set("state.heading_stage_reset_requested", "true");
        }
    }

    fn truck_is_on_route(bb: &SharedBlackboard) -> bool {
        bb.get("router.active").as_deref() == Some("true")
            && bb
                .get("router.waypoint_count")
                .and_then(|v| v.trim().parse::<u32>().ok())
                .unwrap_or(0)
                > 0
    }

    fn evaluate_engage_mode(&mut self, bb: &SharedBlackboard) {
        if self.state != AutopilotState::Active {
            self.reset_engage_mode_timers();
            if self.lane_only_engage {
                // Latch "lane" immediately on engage-request, before reaching Active.
                self.engage_mode = EngageMode::Lane;
                bb.set("autopilot.engage_mode", EngageMode::Lane.as_str());
            } else {
                self.engage_mode = EngageMode::Route;
                bb.set("autopilot.engage_mode", "route");
            }
            bb.set("autopilot.advisory_reason", "");
            return;
        }

        // Vision mode: transitions don't apply, always Lane
        if self.mode_at_engage == "vision" {
            bb.set("autopilot.engage_mode", EngageMode::Lane.as_str());
            bb.set("autopilot.advisory_reason", "");
            return;
        }

        // Lane-only engage: locked to Lane mode for the session
        if self.lane_only_engage {
            self.engage_mode = EngageMode::Lane;
            bb.set("autopilot.engage_mode", EngageMode::Lane.as_str());
            bb.set("autopilot.advisory_reason", "");
            return;
        }

        let heading_diff = bb
            .get("lane_follower.heading_diff_deg")
            .and_then(|v| v.trim().parse::<f64>().ok())
            .unwrap_or(0.0);
        let dist_m = bb
            .get("lane_follower.nearest_seg_dist_m")
            .and_then(|v| v.trim().parse::<f64>().ok())
            .unwrap_or(0.0);
        let router_active = bb.get("router.active").as_deref() == Some("true");
        let on_route = Self::truck_is_on_route(bb);

        match self.engage_mode {
            EngageMode::Route => {
                // ROUTE -> LANE: heading > 90 deg sustained 3s OR route lost
                if heading_diff > 90.0 || !router_active {
                    self.route_to_lane_ticks += 1;
                } else {
                    self.route_to_lane_ticks = 0;
                }
                if self.route_to_lane_ticks >= ROUTE_TO_LANE_TICKS {
                    self.transition_engage_mode(EngageMode::Lane, bb);
                }

                // ROUTE -> DEGRADED: dist > 50m sustained 5s (only if still Route)
                if self.engage_mode == EngageMode::Route {
                    if dist_m > 50.0 {
                        self.to_degraded_ticks += 1;
                    } else {
                        self.to_degraded_ticks = 0;
                    }
                    if self.to_degraded_ticks >= TO_DEGRADED_TICKS {
                        self.transition_engage_mode(EngageMode::Degraded, bb);
                    }
                }

                self.lane_to_route_ticks = 0;
                self.degraded_recovery_ticks = 0;
                self.degraded_timeout_ticks = 0;
            }
            EngageMode::Lane => {
                // LANE -> ROUTE: on_route AND heading < 30 deg sustained 2s
                if on_route && heading_diff < 30.0 {
                    self.lane_to_route_ticks += 1;
                } else {
                    self.lane_to_route_ticks = 0;
                }
                if self.lane_to_route_ticks >= LANE_TO_ROUTE_TICKS {
                    self.transition_engage_mode(EngageMode::Route, bb);
                }

                // LANE -> DEGRADED: dist > 50m sustained 5s (only if still Lane)
                if self.engage_mode == EngageMode::Lane {
                    if dist_m > 50.0 {
                        self.to_degraded_ticks += 1;
                    } else {
                        self.to_degraded_ticks = 0;
                    }
                    if self.to_degraded_ticks >= TO_DEGRADED_TICKS {
                        self.transition_engage_mode(EngageMode::Degraded, bb);
                    }
                }

                self.route_to_lane_ticks = 0;
                self.degraded_recovery_ticks = 0;
                self.degraded_timeout_ticks = 0;
            }
            EngageMode::Degraded => {
                // DEGRADED -> LANE: dist < 10m sustained 2s
                if dist_m < 10.0 {
                    self.degraded_recovery_ticks += 1;
                } else {
                    self.degraded_recovery_ticks = 0;
                }
                if self.degraded_recovery_ticks >= DEGRADED_RECOVERY_TICKS {
                    self.transition_engage_mode(EngageMode::Lane, bb);
                }

                // DEGRADED -> Off: 30s no recovery
                if self.engage_mode == EngageMode::Degraded {
                    self.degraded_timeout_ticks += 1;
                    if self.degraded_timeout_ticks >= DEGRADED_TIMEOUT_TICKS {
                        tracing::info!("[state] Active -> Off (degraded 30s timeout)");
                        self.state = AutopilotState::Off;
                        self.state_entry_ticks = 0;
                        self.reset_engage_mode_timers();
                        self.engage_mode = EngageMode::Route;
                        bb.set("autopilot.engage_mode", "route");
                        bb.set("autopilot.advisory_reason", "");
                        return;
                    }
                    bb.set("autopilot.advisory_reason", "truck_off_road");
                }

                self.route_to_lane_ticks = 0;
                self.lane_to_route_ticks = 0;
                self.to_degraded_ticks = 0;
            }
        }

        bb.set("autopilot.engage_mode", self.engage_mode.as_str());
        if self.engage_mode != EngageMode::Degraded {
            bb.set("autopilot.advisory_reason", "");
        }
    }
}

// ---- Helpers ---------------------------------------------------------------

fn check_preconditions(
    telemetry: Option<&Telemetry>,
    bb: &SharedBlackboard,
    lane_only: bool,
) -> Preconditions {
    let mode = bb
        .get("plugin.lane_keeper.mode")
        .unwrap_or_else(|| "route_following".to_string());
    // Lane-only: no route required. Vision mode: gate on engage_allowed instead.
    let router_active = if lane_only {
        true
    } else if mode == "vision" {
        bb.get("lane_keeper.engage_allowed")
            .map(|s| s == "true")
            .unwrap_or(false)
    } else {
        bb.get("router.active")
            .map(|s| s == "true")
            .unwrap_or(false)
    };
    // Lane-only: gate on the lane-keeper's own NearestSpline engage gate
    // (lane_keeper.engage_allowed = nearest_hit.heading_diff<60° && dist<20m). This is
    // the reliable, router-independent proximity+heading check. router.last_snap_dist is
    // stale/"0" without a planned route (it reflects the last *planning* snap), so it must
    // not gate lane_only. Mirrors the vision-mode router_active = engage_allowed pattern.
    // Route/vision: not gated here — router guarantees on-graph position.
    let truck_on_road = if lane_only {
        bb.get("lane_keeper.engage_allowed")
            .map(|s| s == "true")
            .unwrap_or(false)
    } else {
        true
    };
    Preconditions {
        telemetry_ok: telemetry.is_some(),
        engine_running: telemetry.map(|t| t.engine_rpm > 100.0).unwrap_or(false),
        critical_plugins_loaded: check_critical_plugins(bb),
        router_active,
        truck_on_road,
    }
}

fn check_critical_plugins(bb: &SharedBlackboard) -> bool {
    let loaded = bb.get("plugins.loaded").unwrap_or_default();
    let names: Vec<&str> = loaded.split(',').map(str::trim).collect();
    // Base plugins always required.
    let base = ["lane-keeper", "speed-controller"];
    if !base.iter().all(|n| names.contains(n)) {
        return false;
    }
    // At least one output plugin must be active.
    let output_plugins = ["vjoy-output", "scs-sdk-output"];
    output_plugins.iter().any(|n| names.contains(n))
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
    // SCS SDK Telemetry has no raw steering-input field (heading/position only).
    // Human override: use `engage-cli disengage` or a dedicated key mapping until
    // a DirectInput-parallel solution is implemented outside the SCS SDK path.
    false
}

fn precondition_failure_reason(pre: &Preconditions) -> &'static str {
    if !pre.telemetry_ok {
        "telemetry_lost"
    } else if !pre.engine_running {
        "engine_off"
    } else if !pre.critical_plugins_loaded {
        "plugins_missing"
    } else if !pre.truck_on_road {
        "truck_off_road"
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
    let heading_rad = -telemetry.heading * std::f64::consts::TAU;
    let fw_x = heading_rad.sin();
    let fw_z = -heading_rad.cos();
    let dot = fw_x * dir_x + fw_z * dir_z;
    dot >= 0.707
}

/// Returns `true` if the next meaningful waypoint is in the truck's forward hemisphere.
/// Used at engage time to detect a heading-reversed truck that is geometrically on-route
/// but facing the wrong way. Falls back to `true` (don't block) when waypoints are absent
/// or all waypoints are within 5 m of the truck.
fn waypoint_ahead_of_truck(bb: &SharedBlackboard, pos_x: f64, pos_z: f64, heading: f64) -> bool {
    let json = match bb.get("router.waypoints") {
        Some(j) => j,
        None => return true,
    };
    let wps: Vec<[f64; 2]> = match serde_json::from_str(&json) {
        Ok(v) => v,
        Err(_) => return true,
    };
    // Skip waypoints that are too close to give a reliable direction.
    let target = wps.iter().find(|wp| {
        let dx = wp[0] - pos_x;
        let dz = wp[1] - pos_z;
        dx * dx + dz * dz > 25.0 // > 5 m
    });
    let wp = match target {
        Some(w) => w,
        None => return true, // all waypoints within 5 m — can't determine direction
    };
    let dx = wp[0] - pos_x;
    let dz = wp[1] - pos_z;
    let len = (dx * dx + dz * dz).sqrt();
    let heading_rad = -heading * std::f64::consts::TAU;
    let fw_x = heading_rad.sin();
    let fw_z = -heading_rad.cos();
    (fw_x * dx + fw_z * dz) / len >= 0.0
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
    let heading_rad = -telemetry.heading * std::f64::consts::TAU;
    let fw_x = heading_rad.sin();
    let fw_z = -heading_rad.cos();
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
            nav_distance_m: -1.0,
            nav_time_s: -1.0,
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

    /// Drive a fresh state machine into `Active` via a **lane-only** engage.
    /// Lane-only gates `truck_on_road` on `lane_keeper.engage_allowed`, so that
    /// must be set; `last_telemetry` must be populated first (one `evaluate`)
    /// or the lane-only engage is rejected as `telemetry_lost`.
    fn drive_to_active_lane_only(bb: &SharedBlackboard) -> AutopilotStateMachine {
        let mut sm = AutopilotStateMachine::new();
        let running = mock_running();
        sm.evaluate(Some(&running), bb); // populate last_telemetry (state stays Off)
        bb.set("lane_keeper.engage_allowed", "true");
        bb.set("autopilot.requested_mode", "lane_only");
        sm.handle_event(AutopilotEvent::UserEngage, bb).unwrap();
        for _ in 0..51 {
            sm.evaluate(Some(&running), bb);
        }
        assert_eq!(sm.state(), AutopilotState::Active);
        sm
    }

    #[test]
    fn lane_only_throttle_demand_prevents_paused_deadlock() {
        let bb = bb_with_preconditions();
        let mut sm = drive_to_active_lane_only(&bb);
        // Truck steht (speed≈0), aber der Speed-Controller fordert Gas an.
        bb.set("speed_controller.throttle_cmd", "1.000");
        let stopped = mock_stopped();
        for _ in 0..400 {
            sm.evaluate(Some(&stopped), &bb);
        }
        // Kein selbsthaltender Stillstand: der Truck bleibt Active, der an
        // is_active() gekoppelte Längsregler bleibt scharf und kann anfahren.
        assert_eq!(sm.state(), AutopilotState::Active);
        assert_eq!(
            bb.get("autopilot.pause_suppressed").as_deref(),
            Some("lane_only_throttle_demand")
        );
    }

    #[test]
    fn lane_only_paused_still_triggers_without_throttle() {
        let bb = bb_with_preconditions();
        let mut sm = drive_to_active_lane_only(&bb);
        // Truck steht und der Regler fordert KEIN Gas (z.B. Sollgeschwindigkeit 0 /
        // Coast) → legitimes Stehenbleiben → Paused greift weiterhin.
        bb.set("speed_controller.throttle_cmd", "0.000");
        let stopped = mock_stopped();
        for _ in 0..251 {
            sm.evaluate(Some(&stopped), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Paused);
        assert_eq!(
            bb.get("autopilot.paused_reason").as_deref(),
            Some("speed_zero_5s")
        );
    }

    #[test]
    fn route_paused_unaffected_by_throttle_cmd() {
        // Route-Modus: der Deadlock-Guard ist auf lane_only beschränkt. Selbst bei
        // anliegendem Gas-Wunsch muss der Paused-Latch unverändert greifen.
        let bb = bb_with_preconditions();
        let running = mock_running();
        let mut sm = AutopilotStateMachine::new();
        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();
        for _ in 0..51 {
            sm.evaluate(Some(&running), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Active);
        bb.set("speed_controller.throttle_cmd", "1.000");
        let stopped = mock_stopped();
        for _ in 0..251 {
            sm.evaluate(Some(&stopped), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Paused);
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
    fn cruise_off_does_not_fault() {
        // ETS2 cruise control is irrelevant — autopilot runs its own throttle/brake.
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let running = mock_running();
        engage_to_active(&mut sm, &bb, &running);
        let mut cc_off = running.clone();
        cc_off.cruise_control_kmh = 0.0;
        for _ in 0..700 {
            sm.evaluate(Some(&cc_off), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Active);
    }

    /// Build a blackboard that satisfies all vision-mode preconditions,
    /// including cruise_control_kmh = 0 (the bug scenario).
    fn bb_vision_preconditions() -> SharedBlackboard {
        let bb = SharedBlackboard::new();
        bb.set("plugin.lane_keeper.mode", "vision");
        bb.set("lane_keeper.engage_allowed", "true");
        bb.set(
            "plugins.loaded",
            "lane-keeper,speed-controller,scs-sdk-output",
        );
        bb.set("state.engage_ready", "true");
        bb
    }

    fn mock_running_no_cruise() -> Telemetry {
        let mut t = mock_running();
        t.cruise_control_kmh = 0.0;
        t
    }

    #[test]
    fn vision_mode_engages_without_cruise() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_vision_preconditions();
        let t = mock_running_no_cruise(); // cruise_control_kmh == 0
        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();
        for _ in 0..51 {
            sm.evaluate(Some(&t), &bb);
        }
        assert_eq!(
            sm.state(),
            AutopilotState::Active,
            "must reach Active without ETS2 cruise active"
        );
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
            heading_ok_for_engage: true,
            ..Default::default()
        };
        assert!(
            !pre.hard_blockers_met(),
            "telemetry_fresh=false should fail hard blockers"
        );

        let pre = EngagementPreconditions {
            telemetry_fresh: true,
            truck_on_road: true,
            heading_aligned: true,
            route_planned: false,
            truck_on_route: true,
            speed_ok: true,
            heading_ok_for_engage: true,
            ..Default::default()
        };
        assert!(
            !pre.hard_blockers_met(),
            "route_planned=false should fail hard blockers"
        );

        let pre = EngagementPreconditions {
            telemetry_fresh: true,
            truck_on_road: false,
            heading_aligned: false,
            route_planned: true,
            truck_on_route: false,
            speed_ok: false,
            heading_ok_for_engage: true,
            ..Default::default()
        };
        assert!(
            pre.hard_blockers_met(),
            "telemetry_fresh+route_planned+heading_ok_for_engage needed"
        );

        let pre = EngagementPreconditions {
            telemetry_fresh: true,
            truck_on_road: true,
            heading_aligned: true,
            route_planned: true,
            truck_on_route: true,
            speed_ok: true,
            heading_ok_for_engage: false,
            ..Default::default()
        };
        assert!(
            !pre.hard_blockers_met(),
            "heading_ok_for_engage=false should fail hard blockers"
        );
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

        // heading 0.75 (East, ETS2 0..1): fw=(1,0), dot=1.0 >= 0.707 → true
        let t = Telemetry {
            position: [0.0, 0.0, 0.0],
            heading: 0.75,
            ..mock_running()
        };
        assert!(compute_heading_aligned(&t, &bb));

        // heading 0.5 (South, ETS2 0..1): fw=(0,1), dot=0 → false
        let t2 = Telemetry { heading: 0.5, ..t };
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
            heading_ok_for_engage: true,
            ..Default::default()
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
        assert!(
            bb.get("state.engage_ready").is_none(),
            "should be absent before tick 17"
        );

        // tick 17: triggers engagement precondition eval
        sm.evaluate(Some(&t), &bb);
        assert!(
            bb.get("state.engage_ready").is_some(),
            "should be published at tick 17"
        );
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
        assert!(
            !ep.truck_on_road,
            "20.0m exactly should be NOT < 20.0 (strict)"
        );
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
        assert!(
            !ep.truck_on_route,
            "missing both edge_id and route_edge_ids"
        );

        bb.set("router.last_snap_edge_id", "42");
        let ep = sm.evaluate_engagement_preconditions(&bb);
        assert!(
            !ep.truck_on_route,
            "edge_id present but route_edge_ids missing"
        );

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
            heading_ok_for_engage: true,
            ..Default::default()
        };
        assert!(pre.hard_blockers_met());
        assert!(!pre.all_met());
    }

    #[test]
    fn heading_aligned_single_waypoint_returns_true() {
        let bb = SharedBlackboard::new();
        bb.set("router.waypoints", "[[0.0,0.0]]");
        let t = mock_running();
        assert!(
            compute_heading_aligned(&t, &bb),
            "single waypoint: no direction → always aligned"
        );
    }

    #[test]
    fn heading_aligned_waypoint_on_truck_returns_true() {
        let bb = SharedBlackboard::new();
        bb.set("router.waypoints", "[[0.0,0.0],[0.0,0.0]]");
        let mut t = mock_running();
        t.position = [0.0, 0.0, 0.0];
        assert!(
            compute_heading_aligned(&t, &bb),
            "truck on waypoint → len < 0.01 → always aligned"
        );
    }

    #[test]
    fn heading_aligned_invalid_json_waypoints_returns_false() {
        let bb = SharedBlackboard::new();
        bb.set("router.waypoints", "not valid json");
        let t = mock_running();
        assert!(
            !compute_heading_aligned(&t, &bb),
            "invalid JSON should return false"
        );
    }

    #[test]
    fn heading_aligned_46_degrees_returns_false() {
        let bb = SharedBlackboard::new();
        // Waypoint south of truck at (0, 0, 0): direction = (0, -1) in ETS2 units
        bb.set("router.waypoints", "[[0.0,0.0],[0.0,-100.0]]");

        // heading 0.125 (NW, 45° CCW from N in ETS2): fw=(-0.707,-0.707)
        // dot with (0,-1) = 0.707 → exactly threshold → aligned
        let t_at_45 = Telemetry {
            position: [0.0, 0.0, 0.0],
            heading: 0.125,
            ..mock_running()
        };
        assert!(
            compute_heading_aligned(&t_at_45, &bb),
            "45° should be at threshold"
        );

        // heading 0.130 (≈47° CCW from N in ETS2): dot < 0.707 → misaligned
        let t_over = Telemetry {
            heading: 0.130,
            ..t_at_45
        };
        assert!(
            !compute_heading_aligned(&t_over, &bb),
            ">45° misalignment should fail"
        );
    }

    #[test]
    fn engagement_precondition_telemetry_stale_returns_false() {
        let mut sm = AutopilotStateMachine::new();
        sm.last_telemetry = Some(mock_running());
        sm.last_telemetry_time =
            Some(std::time::Instant::now() - std::time::Duration::from_millis(500));
        let bb = SharedBlackboard::new();
        let ep = sm.evaluate_engagement_preconditions(&bb);
        assert!(
            !ep.telemetry_fresh,
            "500ms old telemetry should not be fresh"
        );
    }

    #[test]
    fn engagement_detail_keys_published() {
        let mut sm = AutopilotStateMachine::new();
        let mut t = mock_running();
        t.position = [10.0, 0.0, 20.0];
        t.heading = 0.75; // east (ETS2 0..1)
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
            heading_ok_for_engage: true,
            ..Default::default()
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
            heading_ok_for_engage: false,
            ..Default::default() // mode="route_following", lane_keeper_engage_allowed=false (irrelevant in route mode)
        };
        let names = pre.blocked_names();
        assert_eq!(names.split(", ").count(), 7);
    }

    // ---- Phase 6.5q.2: Fix 1 — waypoint_ahead_of_truck ----------------------

    #[test]
    fn waypoint_ahead_of_truck_aligned_heading() {
        let bb = SharedBlackboard::new();
        // heading 0.75 (East, ETS2 0..1): fw=(1,0). Waypoint east at (100,0) → dot=1 → ahead
        bb.set("router.waypoints", "[[0.0,0.0],[100.0,0.0]]");
        assert!(waypoint_ahead_of_truck(&bb, 0.0, 0.0, 0.75));
    }

    #[test]
    fn waypoint_behind_truck_reversed() {
        let bb = SharedBlackboard::new();
        // heading 0.75 (East, ETS2 0..1): fw=(1,0). Waypoint west at (-100,0) → dot=-1 → behind
        bb.set("router.waypoints", "[[0.0,0.0],[-100.0,0.0]]");
        assert!(!waypoint_ahead_of_truck(&bb, 0.0, 0.0, 0.75));
    }

    #[test]
    fn waypoint_ahead_no_waypoints_returns_true() {
        let bb = SharedBlackboard::new();
        assert!(
            waypoint_ahead_of_truck(&bb, 0.0, 0.0, 0.0),
            "no waypoints → assume ahead"
        );
    }

    #[test]
    fn waypoint_ahead_too_close_returns_true() {
        let bb = SharedBlackboard::new();
        // Waypoint only 3 m away → skip (< 5 m threshold) → assume ahead
        bb.set("router.waypoints", "[[0.0,0.0],[3.0,0.0]]");
        assert!(waypoint_ahead_of_truck(&bb, 0.0, 0.0, 0.0));
    }

    #[test]
    fn waypoint_ahead_perpendicular_is_ahead() {
        let bb = SharedBlackboard::new();
        // heading 0 (north): fw=(0,-1). Waypoint due east at (100,0) → dot=0 → exactly perpendicular
        // ">= 0.0" → returns true (perpendicular is not behind)
        bb.set("router.waypoints", "[[0.0,0.0],[100.0,0.0]]");
        assert!(waypoint_ahead_of_truck(&bb, 0.0, 0.0, 0.0));
    }

    // ---- Phase 6.5q.2: Fix 2 — heading_ok_for_engage 60° hard block ---------

    #[test]
    fn heading_ok_for_engage_aligned_passes() {
        let mut sm = AutopilotStateMachine::new();
        let mut t = mock_running();
        t.position = [0.0, 0.0, 0.0];
        t.heading = 0.0; // north: fw=(0,-1)
        sm.last_telemetry = Some(t);
        sm.last_telemetry_time = Some(std::time::Instant::now());
        let bb = SharedBlackboard::new();
        // Waypoints going north (−Z): diff ≈ 0° < 60° → ok
        bb.set("router.waypoints", "[[0.0,0.0],[0.0,-100.0]]");
        let ep = sm.evaluate_engagement_preconditions(&bb);
        assert!(
            ep.heading_ok_for_engage,
            "0° diff should pass 60° threshold"
        );
    }

    #[test]
    fn heading_ok_for_engage_reversed_fails() {
        let mut sm = AutopilotStateMachine::new();
        let mut t = mock_running();
        t.position = [0.0, 0.0, 0.0];
        t.heading = 0.5; // south (ETS2 0..1): fw=(0,1)
        sm.last_telemetry = Some(t);
        sm.last_telemetry_time = Some(std::time::Instant::now());
        let bb = SharedBlackboard::new();
        // Waypoints going north (−Z): diff = 180° > 60° → fail
        bb.set("router.waypoints", "[[0.0,0.0],[0.0,-100.0]]");
        let ep = sm.evaluate_engagement_preconditions(&bb);
        assert!(
            !ep.heading_ok_for_engage,
            "180° diff should fail 60° threshold"
        );
        assert!(
            !ep.hard_blockers_met(),
            "hard_blockers_met must fail when heading_ok_for_engage=false"
        );
    }

    #[test]
    fn heading_ok_for_engage_no_waypoints_allows_engage() {
        let mut sm = AutopilotStateMachine::new();
        sm.last_telemetry = Some(mock_running());
        sm.last_telemetry_time = Some(std::time::Instant::now());
        let bb = SharedBlackboard::new(); // no waypoints
        let ep = sm.evaluate_engagement_preconditions(&bb);
        assert!(
            ep.heading_ok_for_engage,
            "no waypoints → cannot measure → allow"
        );
    }

    #[test]
    fn heading_hard_block_sets_engage_ready_false() {
        let mut sm = AutopilotStateMachine::new();
        let mut t = mock_running();
        t.position = [0.0, 0.0, 0.0];
        t.heading = 0.5; // south/reversed (ETS2 0..1)
        sm.last_telemetry = Some(t);
        sm.last_telemetry_time = Some(std::time::Instant::now());
        let bb = SharedBlackboard::new();
        bb.set("router.waypoints", "[[0.0,0.0],[0.0,-100.0]]");
        bb.set("router.last_planning_result", "ok");
        let ep = sm.evaluate_engagement_preconditions(&bb);
        sm.publish_engagement_preconditions(&ep, &bb);
        assert_eq!(
            bb.get("state.engage_ready").as_deref(),
            Some("false"),
            "engage_ready must be false when heading_ok_for_engage=false"
        );
        assert!(bb
            .get("state.engage_blocked_by")
            .unwrap_or_default()
            .contains("heading_ok_for_engage"));
    }

    // ---- ETS2 heading convention (0..1 CCW from North) regression tests ------

    #[test]
    fn compute_heading_aligned_ets2_convention() {
        let bb = SharedBlackboard::new();
        bb.set("router.waypoints", "[[0.0,0.0],[100.0,0.0]]");

        // East (0.75) facing east waypoint → aligned
        let t_east = Telemetry {
            position: [0.0, 0.0, 0.0],
            heading: 0.75,
            ..mock_running()
        };
        assert!(
            compute_heading_aligned(&t_east, &bb),
            "east heading, east waypoint → aligned"
        );

        // South (0.5) facing east waypoint → dot=0 → not aligned
        let t_south = Telemetry {
            heading: 0.5,
            ..t_east
        };
        assert!(
            !compute_heading_aligned(&t_south, &bb),
            "south heading, east waypoint → not aligned"
        );
    }

    #[test]
    fn waypoint_ahead_of_truck_ets2_south_ahead() {
        let bb = SharedBlackboard::new();
        // South (0.5): fw=(0,1). Waypoint south at (0,100) → ahead
        bb.set("router.waypoints", "[[0.0,0.0],[0.0,100.0]]");
        assert!(
            waypoint_ahead_of_truck(&bb, 0.0, 0.0, 0.5),
            "south heading, south waypoint → ahead"
        );

        let bb2 = SharedBlackboard::new();
        // North (0.0): fw=(0,-1). Waypoint south at (0,100) → behind
        bb2.set("router.waypoints", "[[0.0,0.0],[0.0,100.0]]");
        assert!(
            !waypoint_ahead_of_truck(&bb2, 0.0, 0.0, 0.0),
            "north heading, south waypoint → behind"
        );
    }

    #[test]
    fn heading_diff_degrees_ets2_south_reversed() {
        let bb = SharedBlackboard::new();
        // South (0.5): fw=(0,1). Waypoint north at (0,-100) → diff ≈ 180°
        bb.set("router.waypoints", "[[0.0,0.0],[0.0,-100.0]]");
        let t = Telemetry {
            position: [0.0, 0.0, 0.0],
            heading: 0.5,
            ..mock_running()
        };
        let diff = heading_diff_degrees(&t, &bb).expect("should return Some");
        assert!(
            (diff - 180.0).abs() < 0.01,
            "south heading vs north waypoint → 180° diff, got {diff}"
        );
    }

    // ---- Phase 6.5q.2: Fix 3 — start_node_unknown advisory ------------------

    #[test]
    fn start_node_unknown_publishes_advisory() {
        let mut sm = AutopilotStateMachine::new();
        sm.last_telemetry = Some(mock_running());
        sm.last_telemetry_time = Some(std::time::Instant::now());
        let bb = SharedBlackboard::new();
        bb.set("router.last_planning_result", "start_node_unknown");
        bb.set("router.waypoints", "[[0.0,0.0],[0.0,-100.0]]");
        let ep = sm.evaluate_engagement_preconditions(&bb);
        sm.publish_engagement_preconditions(&ep, &bb);
        let advisory = bb.get("state.engage_advisory").unwrap_or_default();
        assert!(
            advisory.contains("Routing-Graph"),
            "advisory must mention Routing-Graph: {advisory}"
        );
    }

    #[test]
    fn advisory_cleared_when_planning_ok() {
        let mut sm = AutopilotStateMachine::new();
        sm.last_telemetry = Some(mock_running());
        sm.last_telemetry_time = Some(std::time::Instant::now());
        let bb = SharedBlackboard::new();
        bb.set("router.last_planning_result", "ok");
        bb.set("router.waypoints", "[[0.0,0.0],[0.0,-100.0]]");
        let ep = sm.evaluate_engagement_preconditions(&bb);
        sm.publish_engagement_preconditions(&ep, &bb);
        assert_eq!(
            bb.get("state.engage_advisory").as_deref(),
            Some(""),
            "advisory must be empty when no advisory condition"
        );
    }

    #[test]
    fn heading_hard_block_publishes_advisory() {
        let mut sm = AutopilotStateMachine::new();
        let mut t = mock_running();
        t.heading = 0.5; // south/reversed (ETS2 0..1)
        sm.last_telemetry = Some(t);
        sm.last_telemetry_time = Some(std::time::Instant::now());
        let bb = SharedBlackboard::new();
        bb.set("router.last_planning_result", "ok");
        bb.set("router.waypoints", "[[0.0,0.0],[0.0,-100.0]]");
        let ep = sm.evaluate_engagement_preconditions(&bb);
        sm.publish_engagement_preconditions(&ep, &bb);
        let advisory = bb.get("state.engage_advisory").unwrap_or_default();
        assert!(
            advisory.contains("Richtung") || advisory.contains("60"),
            "heading advisory must mention direction or threshold: {advisory}"
        );
    }

    #[test]
    fn start_node_unknown_takes_priority_over_heading_advisory() {
        let mut sm = AutopilotStateMachine::new();
        let mut t = mock_running();
        t.heading = 0.5; // south/reversed (ETS2 0..1) — would normally trigger heading advisory
        sm.last_telemetry = Some(t);
        sm.last_telemetry_time = Some(std::time::Instant::now());
        let bb = SharedBlackboard::new();
        bb.set("router.last_planning_result", "start_node_unknown");
        bb.set("router.waypoints", "[[0.0,0.0],[0.0,-100.0]]");
        let ep = sm.evaluate_engagement_preconditions(&bb);
        sm.publish_engagement_preconditions(&ep, &bb);
        let advisory = bb.get("state.engage_advisory").unwrap_or_default();
        assert!(
            advisory.contains("Routing-Graph"),
            "start_node_unknown advisory must take priority: {advisory}"
        );
    }

    // ---- CC: Vision-mode engage preconditions --------------------------------

    #[test]
    fn vision_mode_all_ok_no_route_all_met_true() {
        let mut sm = AutopilotStateMachine::new();
        sm.last_telemetry = Some(mock_running());
        sm.last_telemetry_time = Some(std::time::Instant::now());
        let bb = SharedBlackboard::new();
        bb.set("plugin.lane_keeper.mode", "vision");
        bb.set("lane_keeper.engage_allowed", "true");
        bb.set("router.last_snap_dist", "5.0");
        bb.set("router.waypoints", "[[0.0,0.0],[0.0,-100.0]]");
        // No route_planned, no truck_on_route — must not matter in vision mode.
        let ep = sm.evaluate_engagement_preconditions(&bb);
        assert_eq!(ep.mode, "vision");
        assert!(ep.lane_keeper_engage_allowed);
        assert!(ep.telemetry_fresh);
        assert!(ep.truck_on_road);
        assert!(
            ep.all_met(),
            "vision mode: all_met must be true when lane_keeper allows"
        );
        assert!(
            ep.hard_blockers_met(),
            "vision mode: hard_blockers_met without route"
        );
    }

    #[test]
    fn vision_mode_engage_allowed_false_all_met_false() {
        let mut sm = AutopilotStateMachine::new();
        sm.last_telemetry = Some(mock_running());
        sm.last_telemetry_time = Some(std::time::Instant::now());
        let bb = SharedBlackboard::new();
        bb.set("plugin.lane_keeper.mode", "vision");
        bb.set("lane_keeper.engage_allowed", "false");
        bb.set("router.last_snap_dist", "5.0");
        let ep = sm.evaluate_engagement_preconditions(&bb);
        assert!(!ep.lane_keeper_engage_allowed);
        assert!(
            !ep.all_met(),
            "vision mode: all_met must be false when lane_keeper blocked"
        );
        assert!(ep.blocked_names().contains("lane_keeper_engage_allowed"));
    }

    #[test]
    fn vision_mode_blocked_names_no_route_keys() {
        let pre = EngagementPreconditions {
            mode: "vision".into(),
            lane_keeper_engage_allowed: false,
            telemetry_fresh: true,
            truck_on_road: true,
            heading_ok_for_engage: true,
            speed_ok: true,
            ..Default::default()
        };
        let names = pre.blocked_names();
        assert!(names.contains("lane_keeper_engage_allowed"));
        assert!(
            !names.contains("route_planned"),
            "route_planned must not appear in vision mode"
        );
        assert!(
            !names.contains("truck_on_route"),
            "truck_on_route must not appear in vision mode"
        );
    }

    #[test]
    fn route_mode_unchanged_requires_route() {
        let pre = EngagementPreconditions {
            mode: "route_following".into(),
            telemetry_fresh: true,
            truck_on_road: true,
            heading_aligned: true,
            route_planned: false,
            truck_on_route: true,
            speed_ok: true,
            heading_ok_for_engage: true,
            lane_keeper_engage_allowed: false, // irrelevant in route mode
        };
        assert!(
            !pre.all_met(),
            "route mode: route_planned=false must fail all_met"
        );
        assert!(
            !pre.hard_blockers_met(),
            "route mode: route_planned=false must fail hard_blockers"
        );
    }

    #[test]
    fn vision_mode_engage_ready_published_without_route() {
        let mut sm = AutopilotStateMachine::new();
        sm.last_telemetry = Some(mock_running());
        sm.last_telemetry_time = Some(std::time::Instant::now());
        let bb = SharedBlackboard::new();
        bb.set("plugin.lane_keeper.mode", "vision");
        bb.set("lane_keeper.engage_allowed", "true");
        bb.set("router.waypoints", "[[0.0,0.0],[0.0,-100.0]]");
        // Simulate publish pipeline.
        let ep = sm.evaluate_engagement_preconditions(&bb);
        sm.publish_engagement_preconditions(&ep, &bb);
        assert_eq!(
            bb.get("state.engage_ready").as_deref(),
            Some("true"),
            "engage_ready must be true in vision mode when telemetry+heading ok"
        );
        assert_eq!(
            bb.get("state.engage_precondition_route_planned").as_deref(),
            Some("true"),
            "route_planned key must be published as true in vision mode"
        );
    }

    #[test]
    fn mode_switch_mid_active_disengages() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        bb.set("plugin.lane_keeper.mode", "route_following");
        let t = mock_running();
        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();
        for _ in 0..51 {
            sm.evaluate(Some(&t), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Active);
        assert_eq!(sm.mode_at_engage, "route_following");

        // Simulate operator switching mode while Active.
        bb.set("plugin.lane_keeper.mode", "vision");
        sm.evaluate(Some(&t), &bb);
        assert_eq!(
            sm.state(),
            AutopilotState::Off,
            "mode change must trigger clean Off"
        );
    }

    // ---- P0.2: EngageMode transitions ----------------------------------------

    fn activate_sm(sm: &mut AutopilotStateMachine, bb: &SharedBlackboard) {
        let running = mock_running();
        sm.handle_event(AutopilotEvent::UserEngage, bb).unwrap();
        for _ in 0..51 {
            sm.evaluate(Some(&running), bb);
        }
        assert_eq!(sm.state(), AutopilotState::Active);
    }

    #[test]
    fn engage_mode_defaults_to_route_on_activate() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        activate_sm(&mut sm, &bb);
        assert_eq!(bb.get("autopilot.engage_mode").as_deref(), Some("route"));
    }

    #[test]
    fn route_to_lane_after_sustained_heading_diff() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let running = mock_running();
        activate_sm(&mut sm, &bb);

        bb.set("lane_follower.heading_diff_deg", "95.0");
        bb.set("lane_follower.nearest_seg_dist_m", "5.0");
        bb.set("router.active", "true");
        bb.set("router.waypoint_count", "3");
        for _ in 0..149 {
            sm.evaluate(Some(&running), &bb);
        }
        assert_eq!(bb.get("autopilot.engage_mode").as_deref(), Some("route"));

        sm.evaluate(Some(&running), &bb);
        assert_eq!(bb.get("autopilot.engage_mode").as_deref(), Some("lane"));
    }

    #[test]
    fn route_to_lane_no_flap_on_oscillating_heading() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let running = mock_running();
        activate_sm(&mut sm, &bb);

        bb.set("lane_follower.nearest_seg_dist_m", "5.0");
        bb.set("router.active", "true");
        bb.set("router.waypoint_count", "3");

        for i in 0..200u32 {
            if i % 5 == 0 {
                bb.set("lane_follower.heading_diff_deg", "10.0");
            } else {
                bb.set("lane_follower.heading_diff_deg", "95.0");
            }
            sm.evaluate(Some(&running), &bb);
        }
        assert_eq!(
            bb.get("autopilot.engage_mode").as_deref(),
            Some("route"),
            "oscillating heading_diff must not trigger route->lane"
        );
    }

    #[test]
    fn lane_to_route_after_sustained_recovery() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let running = mock_running();
        activate_sm(&mut sm, &bb);

        // Force into Lane
        bb.set("lane_follower.heading_diff_deg", "95.0");
        bb.set("lane_follower.nearest_seg_dist_m", "5.0");
        bb.set("router.active", "true");
        bb.set("router.waypoint_count", "3");
        for _ in 0..150 {
            sm.evaluate(Some(&running), &bb);
        }
        assert_eq!(bb.get("autopilot.engage_mode").as_deref(), Some("lane"));

        // Recover
        bb.set("lane_follower.heading_diff_deg", "15.0");
        for _ in 0..99 {
            sm.evaluate(Some(&running), &bb);
        }
        assert_eq!(
            bb.get("autopilot.engage_mode").as_deref(),
            Some("lane"),
            "must not transition before 100 ticks"
        );
        sm.evaluate(Some(&running), &bb);
        assert_eq!(bb.get("autopilot.engage_mode").as_deref(), Some("route"));
    }

    #[test]
    fn route_to_degraded_after_sustained_dist() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let running = mock_running();
        activate_sm(&mut sm, &bb);

        bb.set("lane_follower.heading_diff_deg", "5.0");
        bb.set("router.active", "true");
        bb.set("router.waypoint_count", "3");
        bb.set("lane_follower.nearest_seg_dist_m", "60.0");
        for _ in 0..249 {
            sm.evaluate(Some(&running), &bb);
        }
        assert_eq!(bb.get("autopilot.engage_mode").as_deref(), Some("route"));
        sm.evaluate(Some(&running), &bb);
        assert_eq!(bb.get("autopilot.engage_mode").as_deref(), Some("degraded"));
    }

    #[test]
    fn degraded_to_lane_after_recovery() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let running = mock_running();
        activate_sm(&mut sm, &bb);

        // Drive to Degraded
        bb.set("lane_follower.heading_diff_deg", "5.0");
        bb.set("router.active", "true");
        bb.set("router.waypoint_count", "3");
        bb.set("lane_follower.nearest_seg_dist_m", "60.0");
        for _ in 0..250 {
            sm.evaluate(Some(&running), &bb);
        }
        assert_eq!(bb.get("autopilot.engage_mode").as_deref(), Some("degraded"));

        // Recover
        bb.set("lane_follower.nearest_seg_dist_m", "5.0");
        for _ in 0..99 {
            sm.evaluate(Some(&running), &bb);
        }
        assert_eq!(
            bb.get("autopilot.engage_mode").as_deref(),
            Some("degraded"),
            "must not transition before 100 recovery ticks"
        );
        sm.evaluate(Some(&running), &bb);
        assert_eq!(bb.get("autopilot.engage_mode").as_deref(), Some("lane"));
    }

    #[test]
    fn degraded_timeout_transitions_to_off() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let running = mock_running();
        activate_sm(&mut sm, &bb);

        bb.set("lane_follower.heading_diff_deg", "5.0");
        bb.set("router.active", "true");
        bb.set("router.waypoint_count", "3");
        bb.set("lane_follower.nearest_seg_dist_m", "60.0");
        for _ in 0..250 {
            sm.evaluate(Some(&running), &bb);
        }
        assert_eq!(bb.get("autopilot.engage_mode").as_deref(), Some("degraded"));

        for _ in 0..1500 {
            sm.evaluate(Some(&running), &bb);
        }
        assert_eq!(
            sm.state(),
            AutopilotState::Off,
            "degraded timeout must disengage"
        );
    }

    #[test]
    fn vision_mode_always_publishes_lane() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        bb.set("plugin.lane_keeper.mode", "vision");
        bb.set("lane_keeper.engage_allowed", "true");
        let running = mock_running();
        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();
        for _ in 0..51 {
            sm.evaluate(Some(&running), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Active);

        bb.set("lane_follower.heading_diff_deg", "175.0");
        bb.set("router.active", "false");
        bb.set("lane_follower.nearest_seg_dist_m", "60.0");
        for _ in 0..200 {
            sm.evaluate(Some(&running), &bb);
        }
        assert_eq!(
            bb.get("autopilot.engage_mode").as_deref(),
            Some("lane"),
            "vision mode must always publish 'lane'"
        );
        assert_eq!(
            sm.state(),
            AutopilotState::Active,
            "vision mode must not disengage via degraded timeout"
        );
    }

    #[test]
    fn heading_stage_reset_requested_on_lane_transition() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let running = mock_running();
        activate_sm(&mut sm, &bb);

        bb.set("lane_follower.heading_diff_deg", "95.0");
        bb.set("lane_follower.nearest_seg_dist_m", "5.0");
        bb.set("router.active", "true");
        bb.set("router.waypoint_count", "3");
        for _ in 0..150 {
            sm.evaluate(Some(&running), &bb);
        }
        assert_eq!(bb.get("autopilot.engage_mode").as_deref(), Some("lane"));
        assert_eq!(
            bb.get("state.heading_stage_reset_requested").as_deref(),
            Some("true"),
            "ROUTE->LANE must set heading_stage_reset_requested=true"
        );
    }

    #[test]
    fn engage_mode_resets_to_route_on_fault() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_with_preconditions();
        let running = mock_running();
        activate_sm(&mut sm, &bb);

        bb.set("lane_follower.heading_diff_deg", "95.0");
        bb.set("lane_follower.nearest_seg_dist_m", "5.0");
        bb.set("router.active", "true");
        bb.set("router.waypoint_count", "3");
        for _ in 0..150 {
            sm.evaluate(Some(&running), &bb);
        }
        assert_eq!(bb.get("autopilot.engage_mode").as_deref(), Some("lane"));

        sm.report_fault(FailureReason::TelemetryLost, &bb);
        sm.evaluate(Some(&running), &bb);
        assert_eq!(
            bb.get("autopilot.engage_mode").as_deref(),
            Some("route"),
            "engage_mode must reset to route on Fault state"
        );
    }

    // ── P0.3: lane_only_engage ─────────────────────────────────────────────

    fn bb_lane_only() -> SharedBlackboard {
        let bb = SharedBlackboard::new();
        // No router.active — lane_only must bypass this
        bb.set("plugins.loaded", "lane-keeper,speed-controller,vjoy-output");
        // Lane-keeper's NearestSpline engage gate is satisfied: truck within 20 m of a
        // heading-compatible lane (<60°). This is the routerless truck_on_road gate now.
        bb.set("lane_keeper.engage_allowed", "true");
        // Snap-dist still set so the (separate) diagnostic truck_on_road indicator is green.
        bb.set("router.last_snap_dist", "5.0");
        bb
    }

    fn activate_sm_lane_only(sm: &mut AutopilotStateMachine, bb: &SharedBlackboard) {
        let running = mock_running();
        // One evaluate tick to populate last_telemetry before engage event
        sm.evaluate(Some(&running), bb);
        bb.set("autopilot.requested_mode", "lane_only");
        sm.handle_event(AutopilotEvent::UserEngage, bb).unwrap();
        for _ in 0..51 {
            sm.evaluate(Some(&running), bb);
        }
        assert_eq!(sm.state(), AutopilotState::Active);
    }

    #[test]
    fn lane_only_engage_bypasses_router_active() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_lane_only(); // no router.active
                                 // Must succeed without router.active
        activate_sm_lane_only(&mut sm, &bb);
        assert_eq!(sm.state(), AutopilotState::Active);
    }

    #[test]
    fn lane_only_engage_publishes_lane_mode() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_lane_only();
        activate_sm_lane_only(&mut sm, &bb);
        assert_eq!(
            bb.get("autopilot.engage_mode").as_deref(),
            Some("lane"),
            "lane_only engage must publish engage_mode=lane"
        );
    }

    #[test]
    fn lane_only_stays_lane_regardless_of_heading_or_route() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_lane_only();
        let running = mock_running();
        activate_sm_lane_only(&mut sm, &bb);

        // Even with conditions that would normally trigger route transitions
        bb.set("lane_follower.heading_diff_deg", "5.0");
        bb.set("lane_follower.nearest_seg_dist_m", "1.0");
        bb.set("router.active", "true");
        bb.set("router.waypoint_count", "10");
        for _ in 0..200 {
            sm.evaluate(Some(&running), &bb);
        }
        assert_eq!(
            bb.get("autopilot.engage_mode").as_deref(),
            Some("lane"),
            "lane_only must stay in Lane mode — no route transitions allowed"
        );
        assert_eq!(sm.state(), AutopilotState::Active);
    }

    #[test]
    fn lane_only_engage_resets_on_disengage() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_lane_only();
        let running = mock_running();
        activate_sm_lane_only(&mut sm, &bb);
        assert_eq!(sm.state(), AutopilotState::Active);

        sm.handle_event(AutopilotEvent::UserDisengage, &bb).unwrap();
        sm.evaluate(Some(&running), &bb);
        assert_eq!(sm.state(), AutopilotState::Off);
        assert_eq!(
            bb.get("autopilot.engage_mode").as_deref(),
            Some("route"),
            "after disengage, engage_mode must reset to route"
        );
    }

    #[test]
    fn lane_only_engage_sets_nearest_spline_mode() {
        // Auto-Moduswechsel: lane_only-Engage muss plugin.lane_keeper.mode=nearest_spline setzen.
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_lane_only();
        let running = mock_running();
        sm.evaluate(Some(&running), &bb);
        bb.set("autopilot.requested_mode", "lane_only");
        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();
        assert_eq!(
            bb.get("plugin.lane_keeper.mode").as_deref(),
            Some("nearest_spline"),
            "lane_only engage must auto-switch the lane-keeper to nearest_spline"
        );
    }

    #[test]
    fn lane_only_disengage_resets_mode_to_route_following() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_lane_only();
        let running = mock_running();
        activate_sm_lane_only(&mut sm, &bb);
        assert_eq!(
            bb.get("plugin.lane_keeper.mode").as_deref(),
            Some("nearest_spline")
        );
        sm.handle_event(AutopilotEvent::UserDisengage, &bb).unwrap();
        sm.evaluate(Some(&running), &bb);
        assert_eq!(
            bb.get("plugin.lane_keeper.mode").as_deref(),
            Some("route_following"),
            "disengage must reset the auto-switched nearest_spline mode to route_following"
        );
    }

    #[test]
    fn engage_ready_without_route() {
        // lane_only + lane_keeper.engage_allowed=true (no router.active, no route) must reach
        // Active — the route precondition is bypassed and the gate is the NearestSpline hit.
        let mut sm = AutopilotStateMachine::new();
        let bb = SharedBlackboard::new();
        bb.set("plugins.loaded", "lane-keeper,speed-controller,vjoy-output");
        bb.set("lane_keeper.engage_allowed", "true");
        // Deliberately NO router.active, NO router.route_node_ids, NO waypoints.
        let running = mock_running();
        sm.evaluate(Some(&running), &bb);
        bb.set("autopilot.requested_mode", "lane_only");
        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();
        for _ in 0..51 {
            sm.evaluate(Some(&running), &bb);
        }
        assert_eq!(
            sm.state(),
            AutopilotState::Active,
            "lane_only must reach Active without any router route when engage_allowed=true"
        );
    }

    #[test]
    fn normal_engage_still_requires_router_active() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_lane_only(); // no router.active
                                 // state.engage_ready not set → should default-allow (engage_ready != "false")
                                 // But router_active=false → preconditions fail → times out in Engaging
        let running = mock_running();
        sm.evaluate(Some(&running), &bb); // populate last_telemetry
                                          // Normal engage (no lane_only key)
        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();
        // Run until engage timeout (> ENGAGE_TIMEOUT ticks)
        for _ in 0..600 {
            sm.evaluate(Some(&running), &bb);
        }
        // Must NOT reach Active — timed out back to Off
        assert_ne!(
            sm.state(),
            AutopilotState::Active,
            "normal engage without router.active must not reach Active"
        );
    }

    // ── P0.3 additional tests ────────────────────────────────────────────────

    #[test]
    fn test_engage_lane_only_with_misaligned_heading_succeeds() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_lane_only();
        // The ROUTE-mode heading check (router.waypoints) is irrelevant for lane_only —
        // heading_aligned stays bypassed to true. The real gate is lane_keeper.engage_allowed
        // (set true in bb_lane_only), so engage still succeeds. A south-facing truck vs
        // north waypoints would fail in route mode but is moot here.
        bb.set("router.waypoints", "[[0.0,0.0],[0.0,-100.0]]");
        let south_truck = Telemetry {
            heading: 0.5,
            ..mock_running()
        };
        sm.evaluate(Some(&south_truck), &bb);
        bb.set("autopilot.requested_mode", "lane_only");
        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();
        for _ in 0..51 {
            sm.evaluate(Some(&south_truck), &bb);
        }
        assert_eq!(sm.state(), AutopilotState::Active);
        assert_eq!(bb.get("autopilot.engage_mode").as_deref(), Some("lane"));
        assert_eq!(
            bb.get("state.engage_precondition_heading_aligned")
                .as_deref(),
            Some("true"),
            "heading_aligned must be bypassed to true for lane_only"
        );
    }

    #[test]
    fn test_engage_lane_only_without_telemetry_fails() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_lane_only();
        bb.set("autopilot.requested_mode", "lane_only");
        // No evaluate() call → last_telemetry is None
        let result = sm.handle_event(AutopilotEvent::UserEngage, &bb);
        assert!(
            result.is_err(),
            "lane_only engage must fail when no telemetry"
        );
        assert_eq!(sm.state(), AutopilotState::Off);
    }

    #[test]
    fn test_engage_lane_only_blocked_when_engage_allowed_false() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_lane_only();
        let running = mock_running();
        // Lane-keeper reports the truck is NOT near a heading-compatible lane
        // (engage_allowed=false). This IS the routerless truck_on_road gate now → must block.
        bb.set("lane_keeper.engage_allowed", "false");
        // Seed last_telemetry
        sm.evaluate(Some(&running), &bb);
        bb.set("autopilot.requested_mode", "lane_only");
        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();
        // Run past ENGAGE_TIMEOUT (250 ticks) → preconditions never stable → Off
        for _ in 0..300 {
            sm.evaluate(Some(&running), &bb);
        }
        assert_eq!(
            sm.state(),
            AutopilotState::Off,
            "lane_only with engage_allowed=false must time out to Off (truck_on_road gate)"
        );
        // Diagnostic must surface the gate as blocked — heading_ok_for_engage is bound to
        // engage_allowed for lane_only.
        let blocked = bb.get("state.engage_blocked_by").unwrap_or_default();
        assert!(
            blocked.contains("heading_ok_for_engage"),
            "blocked_by must contain heading_ok_for_engage when engage_allowed=false: got '{blocked}'"
        );
    }

    #[test]
    fn test_engage_blocked_by_is_empty_when_lane_only_succeeds() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_lane_only();
        let running = mock_running();
        // snap_dist < 20m → truck_on_road=true; mock_running speed=22 m/s → speed_ok=true
        bb.set("router.last_snap_dist", "5.0");
        activate_sm_lane_only(&mut sm, &bb);
        // Trigger fresh diagnostic publish
        for _ in 0..17 {
            sm.evaluate(Some(&running), &bb);
        }
        let blocked = bb.get("state.engage_blocked_by").unwrap_or_default();
        assert!(
            blocked.is_empty(),
            "blocked_by must be empty when all physical conditions ok for lane_only: got '{blocked}'"
        );
        assert_eq!(
            bb.get("state.engage_all_ok").as_deref(),
            Some("true"),
            "engage_all_ok must be true when all conditions met"
        );
    }

    #[test]
    fn test_engage_mode_is_lane_immediately_on_lane_only_engage() {
        let mut sm = AutopilotStateMachine::new();
        let bb = bb_lane_only();
        let running = mock_running();
        // Seed last_telemetry
        sm.evaluate(Some(&running), &bb);
        bb.set("autopilot.requested_mode", "lane_only");
        sm.handle_event(AutopilotEvent::UserEngage, &bb).unwrap();
        assert_eq!(
            sm.state(),
            AutopilotState::Engaging,
            "should be Engaging after UserEngage"
        );
        // Run 1 tick — still Engaging (PRECONDITION_STABLE=50 ticks)
        sm.evaluate(Some(&running), &bb);
        assert_eq!(
            bb.get("autopilot.engage_mode").as_deref(),
            Some("lane"),
            "engage_mode must latch to 'lane' immediately in Engaging for lane_only"
        );
        assert_ne!(
            sm.state(),
            AutopilotState::Active,
            "must still be Engaging after only 1 stable tick"
        );
    }
}
