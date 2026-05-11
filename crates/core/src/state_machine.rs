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

use truckpilot_ipc_protocol::PreconditionSnapshot;
use truckpilot_plugin_api::{SharedBlackboard, Telemetry};

// ---- Constants (in 50 Hz daemon ticks) -------------------------------------

const ENGAGE_TIMEOUT: u64 = 250; // 5 s
const PAUSE_DETECT: u64 = 250; // 5 s stopped
const PAUSE_TIMEOUT: u64 = 15_000; // 5 min
const PRECONDITION_STABLE: u64 = 50; // 1 s
const TELEMETRY_LOSS: u64 = 25; // 500 ms
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
        }
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
                } else {
                    self.precondition_stable_ticks = 0;
                }
                if self.precondition_stable_ticks > PRECONDITION_STABLE {
                    tracing::info!("[state] Engaging -> Active (preconditions stable)");
                    self.state = AutopilotState::Active;
                    self.engaging_ticks = 0;
                    self.precondition_stable_ticks = 0;
                } else if self.engaging_ticks > ENGAGE_TIMEOUT {
                    tracing::warn!("[state] Engaging timed out -> Off");
                    self.state = AutopilotState::Off;
                    self.engaging_ticks = 0;
                    self.precondition_stable_ticks = 0;
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
                        self.publish(bb);
                        return self.state;
                    }
                    if t.engine_rpm <= 100.0 {
                        self.transition_to_fault(FailureReason::EngineStopped);
                        self.publish(bb);
                        return self.state;
                    }
                    if t.cruise_control_kmh <= 0.0 {
                        self.transition_to_fault(FailureReason::CruiseDeactivated);
                        self.publish(bb);
                        return self.state;
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
                } else if self.paused_ticks > PAUSE_TIMEOUT {
                    tracing::info!("[state] Paused -> Off (5 min timeout)");
                    self.state = AutopilotState::Off;
                    self.paused_ticks = 0;
                }
            }
            AutopilotState::Fault => {}
        }

        if self.state == state_before {
            self.state_entry_ticks = self.state_entry_ticks.wrapping_add(1);
        } else {
            self.state_entry_ticks = 0;
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
                tracing::info!("[state] Off -> Engaging (user engage)");
                self.state = AutopilotState::Engaging;
                self.engaging_ticks = 0;
                self.precondition_stable_ticks = 0;
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
                self.state_entry_ticks = 0;
            }
            (AutopilotState::Fault, AutopilotEvent::UserReset) => {
                tracing::info!("[state] Fault -> Off (user reset)");
                self.state = AutopilotState::Off;
                self.fault_reason = None;
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

    fn transition_to_fault(&mut self, reason: FailureReason) {
        self.state = AutopilotState::Fault;
        self.fault_reason = Some(reason);
        self.engaging_ticks = 0;
        self.paused_ticks = 0;
        self.stopped_ticks = 0;
        self.precondition_stable_ticks = 0;
        self.state_entry_ticks = 0;
    }

    fn publish(&self, bb: &SharedBlackboard) {
        bb.set("autopilot.state", self.state.as_str());
        match (&self.fault_reason, self.state) {
            (Some(r), AutopilotState::Fault) => bb.set("autopilot.fault_reason", r.as_str()),
            _ => bb.set("autopilot.fault_reason", ""),
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
        sm.evaluate(Some(&stalled), &bb);
        assert_eq!(sm.state(), AutopilotState::Fault);
        assert_eq!(sm.fault_reason(), Some(&FailureReason::EngineStopped));
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
        bb.set("autopilot.engage_requested", "true");
        sm.consume_requests(&bb);
        assert_eq!(sm.state(), AutopilotState::Engaging);
        assert!(bb.get("autopilot.engage_requested").is_none());
    }
}
