//! Watchdog — Phase 6.2g.2: Heartbeat stall + Telemetry-stale detection.
//!
//! The watchdog runs in its own `tokio::spawn` task and polls at 40 Hz.
//! Two independent checks:
//!
//! 1. **Heartbeat stall** — the daemon control loop bumps an `AtomicU64`
//!    every tick. If the bump age exceeds `heartbeat_stall_ms`, the watchdog
//!    activates the vJoy failsafe (auto-recovers when the heartbeat is fresh
//!    again). This is *not* a `Fault`; the state machine is left intact.
//!
//! 2. **Telemetry stale** — reads `telemetry.available` from the blackboard.
//!    If `false` for longer than `telemetry_stale_ms`, calls
//!    `state_machine.report_fault(TelemetryLost)` and activates failsafe.
//!
//! vJoy wiring stays a stub until Phase 6.2c.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use tokio::sync::Mutex;
use truckpilot_plugin_api::SharedBlackboard;

use crate::state_machine::{AutopilotStateMachine, FailureReason};

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------

pub struct WatchdogConfig {
    pub telemetry_stale_ms: u64,
    pub heartbeat_stall_ms: u64,
    pub watchdog_poll_ms: u64,
    pub failsafe_steering: f64,
    pub failsafe_throttle: f64,
    pub failsafe_brake: f64,
}

impl Default for WatchdogConfig {
    fn default() -> Self {
        Self {
            telemetry_stale_ms: 300,
            heartbeat_stall_ms: 100,
            watchdog_poll_ms: 25,
            failsafe_steering: 0.0,
            failsafe_throttle: 0.0,
            failsafe_brake: 0.3,
        }
    }
}

// ---------------------------------------------------------------------------
// Checks
// ---------------------------------------------------------------------------

/// Returns `true` when the heartbeat is stale and failsafe should be active.
/// Auto-recovers when the heartbeat is fresh again.
///
/// Grace period: if `heartbeat == 0` and the daemon has been running for less
/// than 100 ms, the loop has not yet had a chance to bump the counter — treat
/// as fresh.
pub fn check_heartbeat_stall(
    heartbeat: &Arc<AtomicU64>,
    daemon_start: Instant,
    config: &WatchdogConfig,
) -> bool {
    let last_tick_us = heartbeat.load(Ordering::Relaxed);

    if last_tick_us == 0 && daemon_start.elapsed().as_millis() < 100 {
        return false;
    }

    let now_us = daemon_start.elapsed().as_micros() as u64;
    let age_us = now_us.saturating_sub(last_tick_us);
    let age_ms = age_us / 1_000;

    age_ms > config.heartbeat_stall_ms
}

/// Returns `Some(TelemetryLost)` when `telemetry.available` has been `false`
/// (or absent) for longer than `telemetry_stale_ms`.
///
/// Resets `last_good_frame` whenever telemetry is available so the timer
/// restarts cleanly on recovery.
pub fn check_telemetry_stale(
    bb: &SharedBlackboard,
    last_good_frame: &mut Instant,
    config: &WatchdogConfig,
) -> Option<FailureReason> {
    let available = bb
        .get("telemetry.available")
        .and_then(|v| v.parse::<bool>().ok())
        .unwrap_or(false);

    if available {
        *last_good_frame = Instant::now();
        return None;
    }

    let age_ms = last_good_frame.elapsed().as_millis() as u64;
    if age_ms > config.telemetry_stale_ms {
        Some(FailureReason::TelemetryLost)
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
// Failsafe output (stub)
// ---------------------------------------------------------------------------

/// STUB: logs the intended vJoy output. Real wiring deferred to Phase 6.2c.
pub fn apply_vjoy_failsafe(config: &WatchdogConfig) {
    tracing::warn!(
        "VJOY FAILSAFE ACTIVE: steer={}, throttle={}, brake={}",
        config.failsafe_steering,
        config.failsafe_throttle,
        config.failsafe_brake,
    );
    // TODO Phase 6.2c: wire to actual vjoy_output channel / BB-Key
}

// ---------------------------------------------------------------------------
// Watchdog loop
// ---------------------------------------------------------------------------

pub async fn watchdog_loop(
    heartbeat: Arc<AtomicU64>,
    bb: SharedBlackboard,
    state_machine: Arc<Mutex<AutopilotStateMachine>>,
    daemon_start: Instant,
) {
    let config = WatchdogConfig::default();
    let mut last_good_telemetry = Instant::now();
    let mut failsafe_active = false;

    let mut interval =
        tokio::time::interval(Duration::from_millis(config.watchdog_poll_ms));

    loop {
        interval.tick().await;

        // --- Heartbeat check (failsafe on/off, no Fault) ---
        let heartbeat_stale =
            check_heartbeat_stall(&heartbeat, daemon_start, &config);

        if heartbeat_stale && !failsafe_active {
            tracing::warn!("Heartbeat stall detected, activating failsafe");
            apply_vjoy_failsafe(&config);
            failsafe_active = true;
        } else if !heartbeat_stale && failsafe_active {
            tracing::info!("Heartbeat recovered, deactivating failsafe");
            failsafe_active = false;
        }

        // --- Telemetry-stale check (Fault) ---
        if let Some(reason) =
            check_telemetry_stale(&bb, &mut last_good_telemetry, &config)
        {
            let mut sm = state_machine.lock().await;
            sm.report_fault(reason, &bb);
            drop(sm);
            apply_vjoy_failsafe(&config);
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn t1_heartbeat_stale_detected() {
        let heartbeat = Arc::new(AtomicU64::new(0));
        let config = WatchdogConfig::default();
        // Daemon has been running 500 ms; heartbeat never bumped → stale.
        let daemon_start = Instant::now() - Duration::from_millis(500);
        assert!(check_heartbeat_stall(&heartbeat, daemon_start, &config));
    }

    #[test]
    fn t2_heartbeat_fresh_no_failsafe() {
        let heartbeat = Arc::new(AtomicU64::new(0));
        let daemon_start = Instant::now();
        let config = WatchdogConfig::default();
        // Store "now" as the last beat — age ≈ 0 ms < 100 ms → not stale.
        let now_us = daemon_start.elapsed().as_micros() as u64;
        heartbeat.store(now_us, Ordering::Relaxed);
        assert!(!check_heartbeat_stall(&heartbeat, daemon_start, &config));
    }

    #[test]
    fn t3_heartbeat_zero_skipped() {
        let heartbeat = Arc::new(AtomicU64::new(0));
        let config = WatchdogConfig::default();
        // Daemon just started (<100 ms elapsed), heartbeat=0 → grace period.
        let daemon_start = Instant::now();
        assert!(!check_heartbeat_stall(&heartbeat, daemon_start, &config));
    }

    #[test]
    fn t4_telemetry_stale_over_300ms() {
        let bb = SharedBlackboard::new();
        bb.set("telemetry.available", "false");
        // last_good_frame was 350 ms ago → exceeds 300 ms threshold.
        let mut last_good = Instant::now() - Duration::from_millis(350);
        let config = WatchdogConfig::default();
        let result = check_telemetry_stale(&bb, &mut last_good, &config);
        assert!(matches!(result, Some(FailureReason::TelemetryLost)));
    }

    #[test]
    fn t18_failsafe_output_values_correct() {
        let config = WatchdogConfig::default();
        assert_eq!(config.failsafe_steering, 0.0);
        assert_eq!(config.failsafe_throttle, 0.0);
        assert_eq!(config.failsafe_brake, 0.3);
    }

    #[test]
    fn t19_no_fault_when_telemetry_available() {
        let bb = SharedBlackboard::new();
        bb.set("telemetry.available", "true");
        // Even though last_good is 500 ms ago, available=true resets the timer.
        let mut last_good = Instant::now() - Duration::from_millis(500);
        let config = WatchdogConfig::default();
        let result = check_telemetry_stale(&bb, &mut last_good, &config);
        assert!(result.is_none());
    }
}
