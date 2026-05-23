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
            failsafe_brake: 0.0,
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
// Failsafe output (Phase 6.2b A1: wired via safety.emergency_brake BB-key)
// ---------------------------------------------------------------------------

/// Returns `true` when at least one output plugin (vjoy-output or scs-sdk-output)
/// is present in `plugins.loaded`. Used by the watchdog to decide whether
/// activating the vJoy failsafe has any effect.
///
/// TODO: vjoy_failsafe should also guard scs-sdk-output
fn is_output_plugin_active(bb: &SharedBlackboard) -> bool {
    let loaded = bb.get("plugins.loaded").unwrap_or_default();
    let names: Vec<&str> = loaded.split(',').map(str::trim).collect();
    names.contains(&"vjoy-output") || names.contains(&"scs-sdk-output")
}

/// Activate failsafe by raising the `safety.emergency_brake` blackboard flag.
/// vjoy-output's `tick()` reads this key first and, when `true`, drives the
/// virtual stick to (steer=0, throttle=0, brake=1.0) regardless of the
/// arbitrated ControlOutput. The watchdog clears the flag in
/// [`clear_vjoy_failsafe`] on recovery.
pub fn apply_vjoy_failsafe(bb: &SharedBlackboard, config: &WatchdogConfig, reason: &str) {
    tracing::warn!(
        "[watchdog] FAILSAFE ACTIVE — reason={reason} steer={} throttle={} brake={} (neutral, no auto-brake). User must take over.",
        config.failsafe_steering,
        config.failsafe_throttle,
        config.failsafe_brake,
    );
    let now_ms = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64;
    bb.set("safety.emergency_brake", "true");
    bb.set("safety.failsafe_active", "true");
    bb.set("safety.failsafe_reason", reason);
    bb.set("safety.last_failsafe_at", now_ms.to_string());
}

/// Clear the failsafe flag so vjoy-output resumes normal arbitration.
pub fn clear_vjoy_failsafe(bb: &SharedBlackboard) {
    bb.set("safety.emergency_brake", "false");
    bb.set("safety.failsafe_active", "false");
}

/// Returns `true` when the autopilot is in a state that requires failsafe
/// protection (Engaging, Active, Paused, or Fault). In `Off` state the
/// truck is under manual driver control and a failsafe brake would block it.
pub fn is_autopilot_active(bb: &SharedBlackboard) -> bool {
    matches!(
        bb.get("autopilot.state").as_deref(),
        Some("Engaging" | "Active" | "Paused" | "Fault")
    )
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
    // Track each condition independently so clearing one doesn't mask the other.
    let mut heartbeat_failsafe = false;
    let mut telem_failsafe = false;

    let mut interval = tokio::time::interval(Duration::from_millis(config.watchdog_poll_ms));

    loop {
        interval.tick().await;

        let ap_active = is_autopilot_active(&bb);

        // --- Heartbeat check (failsafe on/off, no Fault) ---
        let heartbeat_stale = check_heartbeat_stall(&heartbeat, daemon_start, &config);
        let output_active = is_output_plugin_active(&bb);

        if heartbeat_stale && ap_active && !heartbeat_failsafe {
            if output_active {
                tracing::warn!("[watchdog] heartbeat stall — autopilot active, activating failsafe");
                heartbeat_failsafe = true;
                apply_vjoy_failsafe(&bb, &config, "heartbeat_stall");
            } else {
                tracing::warn!(
                    "[watchdog] heartbeat stall — autopilot active but no output plugin loaded, \
                     skipping failsafe"
                );
            }
        } else if heartbeat_stale && !ap_active && heartbeat_failsafe {
            // Autopilot disengaged while stall was active — release brake.
            tracing::info!("[watchdog] heartbeat stall but autopilot Off — clearing failsafe, writing neutral");
            clear_vjoy_failsafe(&bb);
            heartbeat_failsafe = false;
        } else if !heartbeat_stale && heartbeat_failsafe {
            tracing::info!("[watchdog] heartbeat recovered — deactivating failsafe");
            clear_vjoy_failsafe(&bb);
            heartbeat_failsafe = false;
        }

        // --- Telemetry-stale check (Fault) ---
        match check_telemetry_stale(&bb, &mut last_good_telemetry, &config) {
            Some(reason) => {
                {
                    let mut sm = state_machine.lock().await;
                    sm.report_fault(reason, &bb);
                }
                if ap_active && !telem_failsafe {
                    if output_active {
                        tracing::warn!("[watchdog] telemetry stale — autopilot active, activating failsafe");
                        telem_failsafe = true;
                        apply_vjoy_failsafe(&bb, &config, "telemetry_stale");
                    } else {
                        tracing::warn!(
                            "[watchdog] telemetry stale — autopilot active but no output plugin \
                             loaded, skipping failsafe"
                        );
                    }
                } else if !ap_active && telem_failsafe {
                    tracing::info!("[watchdog] telemetry stale but autopilot Off — clearing failsafe");
                    clear_vjoy_failsafe(&bb);
                    telem_failsafe = false;
                }
            }
            None => {
                if telem_failsafe {
                    tracing::info!("[watchdog] telemetry recovered — deactivating failsafe");
                    clear_vjoy_failsafe(&bb);
                    telem_failsafe = false;
                }
            }
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
        assert_eq!(config.failsafe_brake, 0.0);
    }

    #[test]
    fn t20_apply_failsafe_sets_emergency_brake_flag() {
        let bb = SharedBlackboard::new();
        let config = WatchdogConfig::default();
        apply_vjoy_failsafe(&bb, &config, "test");
        assert_eq!(bb.get("safety.emergency_brake").as_deref(), Some("true"));
    }

    #[test]
    fn t21_clear_failsafe_resets_emergency_brake_flag() {
        let bb = SharedBlackboard::new();
        let config = WatchdogConfig::default();
        apply_vjoy_failsafe(&bb, &config, "test");
        clear_vjoy_failsafe(&bb);
        assert_eq!(bb.get("safety.emergency_brake").as_deref(), Some("false"));
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

    #[test]
    fn t22_failsafe_not_applied_when_autopilot_off() {
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", "Off");
        assert!(!is_autopilot_active(&bb));
    }

    #[test]
    fn t23_failsafe_applied_when_autopilot_active() {
        for state in ["Engaging", "Active", "Paused", "Fault"] {
            let bb = SharedBlackboard::new();
            bb.set("autopilot.state", state);
            assert!(is_autopilot_active(&bb), "expected active for state={state}");
        }
    }

    #[test]
    fn t24_failsafe_keys_set_on_apply() {
        let bb = SharedBlackboard::new();
        let config = WatchdogConfig::default();
        apply_vjoy_failsafe(&bb, &config, "heartbeat_stall");
        assert_eq!(bb.get("safety.emergency_brake").as_deref(), Some("true"));
        assert_eq!(bb.get("safety.failsafe_active").as_deref(), Some("true"));
        assert_eq!(bb.get("safety.failsafe_reason").as_deref(), Some("heartbeat_stall"));
        assert!(bb.get("safety.last_failsafe_at").is_some());
    }

    #[test]
    fn t25_failsafe_keys_cleared_on_clear() {
        let bb = SharedBlackboard::new();
        let config = WatchdogConfig::default();
        apply_vjoy_failsafe(&bb, &config, "test");
        clear_vjoy_failsafe(&bb);
        assert_eq!(bb.get("safety.emergency_brake").as_deref(), Some("false"));
        assert_eq!(bb.get("safety.failsafe_active").as_deref(), Some("false"));
    }

    #[test]
    fn t26_autopilot_unknown_state_not_active() {
        let bb = SharedBlackboard::new();
        // Missing key → Off / unknown → not active
        assert!(!is_autopilot_active(&bb));
        bb.set("autopilot.state", "Off");
        assert!(!is_autopilot_active(&bb));
    }
}
