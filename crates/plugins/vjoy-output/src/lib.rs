//! vJoy-Output plugin — sends arbitrated ControlOutput to the vJoy virtual
//! joystick. Runs post-arbitration (TickPhase::PostPhase), after all other
//! plugins have had their say. On non-Windows platforms it loads but stays
//! inactive.

mod vjoy_wrapper;

use truckpilot_plugin_api::{ControlOutput, Plugin, PluginContext, Telemetry, TickPhase};

#[cfg(windows)]
use vjoy_wrapper::VJoyHandle;

// ---------------------------------------------------------------------------
// Plugin struct
// ---------------------------------------------------------------------------

pub struct VJoyOutputPlugin {
    device_id: u32,
    #[cfg(windows)]
    vjoy: Option<VJoyHandle>,
    failsafe_timeout_ms: u64,
    last_tick_count: u64,
    inactive: bool,
    tick_count_since_reconnect: u32,
    /// True while the plugin is writing center/neutral values with no active
    /// ControlRequest (game not running or autopilot disengaged).
    idle_centered: bool,
    /// `ctx.tick_count` of the most recent successful `set_axes` call.
    last_write_tick: u64,
}

impl Default for VJoyOutputPlugin {
    fn default() -> Self {
        Self {
            device_id: 1,
            #[cfg(windows)]
            vjoy: None,
            failsafe_timeout_ms: 500,
            last_tick_count: 0,
            inactive: false,
            tick_count_since_reconnect: 0,
            idle_centered: false,
            last_write_tick: 0,
        }
    }
}

// ---------------------------------------------------------------------------
// Pure helper functions (unit-testable, no hardware)
// ---------------------------------------------------------------------------

pub fn should_emergency_brake(ctx: &PluginContext) -> bool {
    ctx.blackboard.get("safety.emergency_brake").as_deref() == Some("true")
}

/// Returns `true` when autopilot is Off (or state is unknown). Used to skip
/// the emergency-brake override and write neutral instead.
pub fn is_autopilot_off(ctx: &PluginContext) -> bool {
    !matches!(
        ctx.blackboard.get("autopilot.state").as_deref(),
        Some("Engaging" | "Active" | "Paused" | "Fault")
    )
}

/// Neutral axis values: steering centered, throttle and brake released.
/// Call this instead of a hard brake whenever autopilot is not engaged.
pub fn write_neutral_axes() -> (f64, f64, f64) {
    (0.0, 0.0, 0.0)
}

/// Returns true if the watchdog has expired: more than `timeout_ms / 20` ticks
/// have passed since `last_tick`. Uses wrapping subtraction for u64 safety.
pub fn is_watchdog_expired(last_tick: u64, current_tick: u64, timeout_ms: u64) -> bool {
    let max_allowed = (timeout_ms / 20).max(1);
    current_tick.wrapping_sub(last_tick) > max_allowed
}

/// Returns true when all three axes are at neutral (steer=center, throttle=0,
/// brake=0). Used to determine whether the plugin is in idle-center state.
pub fn is_idle_output(steer: f64, throttle: f64, brake: f64) -> bool {
    steer == 0.0 && throttle == 0.0 && brake == 0.0
}

// ---------------------------------------------------------------------------
// Plugin impl
// ---------------------------------------------------------------------------

impl Plugin for VJoyOutputPlugin {
    fn name(&self) -> &str {
        "vjoy-output"
    }

    fn version(&self) -> &str {
        "0.2.0"
    }

    fn settings_schema(&self) -> &str {
        r#"{
  "type": "object",
  "properties": {
    "device_id": {
      "type": "integer",
      "minimum": 1,
      "maximum": 16,
      "default": 1,
      "description": "vJoy device ID"
    },
    "enabled": {
      "type": "boolean",
      "default": true,
      "description": "Enable vJoy output"
    },
    "failsafe_timeout_ms": {
      "type": "integer",
      "minimum": 100,
      "maximum": 5000,
      "default": 500,
      "description": "Watchdog timeout in milliseconds"
    }
  }
}"#
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        // Read configuration from blackboard (written by core before plugin load)
        self.device_id = ctx
            .blackboard
            .get_f64("truckpilot.config.vjoy_output.device_id")
            .map(|v| (v as u32).clamp(1, 16))
            .unwrap_or_else(|| {
                ctx.blackboard
                    .get_f64("vjoy.device_id")
                    .map(|v| (v as u32).clamp(1, 16))
                    .unwrap_or(1)
            });

        self.failsafe_timeout_ms = ctx
            .blackboard
            .get_f64("truckpilot.config.vjoy_output.failsafe_timeout_ms")
            .map(|v| (v as u64).clamp(100, 5_000))
            .unwrap_or(500);

        // Announce clean initial state to the blackboard.
        ctx.blackboard.set("vjoy.connected", "false");
        ctx.blackboard
            .set("vjoy.device_id", self.device_id.to_string());
        ctx.blackboard.remove("vjoy.last_error");
        ctx.blackboard.set("vjoy.last_write_tick", "0");
        ctx.blackboard.set("vjoy.idle_centered", "false");
        ctx.blackboard.set("vjoy.last_raw_x", "0");
        ctx.blackboard.set("vjoy.last_raw_sl0", "0");
        ctx.blackboard.set("vjoy.last_raw_sl1", "0");
        ctx.blackboard.set("vjoy.last_raw_source", "none");

        #[cfg(windows)]
        {
            match VJoyHandle::try_acquire(self.device_id) {
                Ok(mut handle) => {
                    // try_acquire already commits a center-write + 100ms
                    // settling sleep before returning. A second explicit
                    // set_axes_verified here surfaces the raw values to the
                    // blackboard so the very first frame the UI shows is
                    // accurate.
                    match handle.set_axes_verified(0.0, 0.0, 0.0) {
                        Ok((rx, rsl0, rsl1)) => {
                            self.idle_centered = true;
                            ctx.blackboard.set("vjoy.idle_centered", "true");
                            ctx.blackboard.set("vjoy.last_raw_x", rx.to_string());
                            ctx.blackboard.set("vjoy.last_raw_sl0", rsl0.to_string());
                            ctx.blackboard.set("vjoy.last_raw_sl1", rsl1.to_string());
                            ctx.blackboard.set("vjoy.last_raw_source", "on_load");
                            tracing::info!(
                                "[vjoy-output] axes centered on acquire \
                                 X={rx} SL0={rsl0} SL1={rsl1}"
                            );
                        }
                        Err(e) => {
                            ctx.blackboard.set("vjoy.last_error", format!("{e}"));
                            tracing::warn!("[vjoy-output] initial center-write failed: {e}");
                        }
                    }
                    self.vjoy = Some(handle);
                    ctx.blackboard.set("vjoy.connected", "true");
                    tracing::info!(
                        "[vjoy-output] vJoy device {} acquired (failsafe={}ms) \
                         HID: X=0x30 (steering), SL0=0x36 (throttle), SL1=0x37 (brake)",
                        self.device_id,
                        self.failsafe_timeout_ms
                    );
                    tracing::info!(
                        "[vjoy-output] failsafe neutral: steer=16384 (center), \
                         throttle=0, brake=0"
                    );
                }
                Err(e) => {
                    self.vjoy = None;
                    self.inactive = true;
                    ctx.blackboard.set("vjoy.last_error", format!("{e}"));
                    tracing::warn!(
                        "[vjoy-output] vJoy init failed: {e} — plugin loaded but inactive"
                    );
                }
            }
        }

        #[cfg(not(windows))]
        {
            self.inactive = true;
            tracing::info!("[vjoy-output] platform not supported — plugin loaded but inactive");
        }
    }

    fn on_unload(&mut self) {
        #[cfg(windows)]
        if let Some(ref mut handle) = self.vjoy {
            handle.center_and_release();
        }
        #[cfg(windows)]
        {
            self.vjoy = None;
        }
        tracing::info!("[vjoy-output] unloaded — axes centered, device released");
    }

    fn default_phase(&self) -> TickPhase {
        TickPhase::PostPhase
    }

    fn tick(
        &mut self,
        _telemetry: Option<&Telemetry>,
        output: &mut ControlOutput,
        ctx: &PluginContext,
    ) {
        if self.inactive {
            return;
        }

        // Emergency brake overrides everything else — but only when autopilot
        // is engaged. If autopilot is Off (manual drive), write neutral instead
        // of hard brake so the driver retains control.
        if should_emergency_brake(ctx) {
            let (s, t, b) = if is_autopilot_off(ctx) {
                tracing::info!(
                    "[vjoy-output] emergency_brake set but autopilot Off — writing neutral"
                );
                write_neutral_axes()
            } else {
                tracing::warn!(
                    "[vjoy-output] FAILSAFE — writing neutral (no auto-brake). User must take over."
                );
                write_neutral_axes()
            };
            #[cfg(windows)]
            if let Some(ref mut handle) = self.vjoy {
                let _ = handle.set_axes(s, t, b);
            }
            return;
        }

        // Watchdog: if too many ticks passed since last send, center and bail.
        // On cold start (last_tick_count=0) this fires after timeout_ms worth
        // of ticks, but tick_windows already runs before then via the normal
        // path. The watchdog's job is to recover from silent ControlOutput gaps.
        if is_watchdog_expired(
            self.last_tick_count,
            ctx.tick_count,
            self.failsafe_timeout_ms,
        ) {
            tracing::warn!(
                "[vjoy-output] watchdog expired (last={} current={} timeout={}ms) — centering",
                self.last_tick_count,
                ctx.tick_count,
                self.failsafe_timeout_ms,
            );
            #[cfg(windows)]
            if let Some(ref mut handle) = self.vjoy {
                match handle.set_axes_verified(0.0, 0.0, 0.0) {
                    Ok((rx, rsl0, rsl1)) => {
                        self.idle_centered = true;
                        self.last_write_tick = ctx.tick_count;
                        ctx.blackboard.set("vjoy.idle_centered", "true");
                        ctx.blackboard
                            .set("vjoy.last_write_tick", ctx.tick_count.to_string());
                        ctx.blackboard.set("vjoy.last_raw_x", rx.to_string());
                        ctx.blackboard.set("vjoy.last_raw_sl0", rsl0.to_string());
                        ctx.blackboard.set("vjoy.last_raw_sl1", rsl1.to_string());
                        ctx.blackboard.set("vjoy.last_raw_source", "watchdog");
                    }
                    Err(e) => {
                        handle.connected = false;
                        ctx.blackboard.set("vjoy.connected", "false");
                        ctx.blackboard.set("vjoy.last_error", format!("{e}"));
                        tracing::error!(
                            "[vjoy-output] watchdog center-write failed: {e} — marking disconnected"
                        );
                    }
                }
            }
            self.last_tick_count = ctx.tick_count;
            return;
        }
        self.last_tick_count = ctx.tick_count;

        #[cfg(windows)]
        self.tick_windows(output, ctx);
    }
}

// ---------------------------------------------------------------------------
// Windows-only tick logic (separated to keep cfg blocks readable)
// ---------------------------------------------------------------------------

#[cfg(windows)]
impl VJoyOutputPlugin {
    fn tick_windows(&mut self, output: &mut ControlOutput, ctx: &PluginContext) {
        let handle = match self.vjoy.as_mut() {
            Some(h) => h,
            None => return,
        };

        // Reconnect attempt every 10 ticks while disconnected
        if !handle.connected {
            self.tick_count_since_reconnect += 1;
            if self.tick_count_since_reconnect >= 10 {
                self.tick_count_since_reconnect = 0;
                if handle.try_reconnect() {
                    ctx.blackboard.set("vjoy.connected", "true");
                    ctx.blackboard.remove("vjoy.last_error");
                    tracing::info!(
                        "[vjoy-output] reconnected to vJoy device {}",
                        self.device_id
                    );
                }
            }
            return;
        }

        match handle.set_axes_verified(output.steering, output.throttle, output.brake) {
            Ok((rx, rsl0, rsl1)) => {
                self.tick_count_since_reconnect = 0;
                let idle = is_idle_output(output.steering, output.throttle, output.brake);
                self.idle_centered = idle;
                self.last_write_tick = ctx.tick_count;
                ctx.blackboard
                    .set("vjoy.idle_centered", if idle { "true" } else { "false" });
                ctx.blackboard
                    .set("vjoy.last_write_tick", ctx.tick_count.to_string());
                ctx.blackboard.set("vjoy.last_raw_x", rx.to_string());
                ctx.blackboard.set("vjoy.last_raw_sl0", rsl0.to_string());
                ctx.blackboard.set("vjoy.last_raw_sl1", rsl1.to_string());
                ctx.blackboard.set("vjoy.last_raw_source", "tick");
            }
            Err(e) => {
                handle.connected = false;
                ctx.blackboard.set("vjoy.connected", "false");
                ctx.blackboard.set("vjoy.last_error", format!("{e}"));
                tracing::error!("[vjoy-output] vJoy send failed: {e} — marking disconnected");
            }
        }
    }
}

truckpilot_plugin_api::export_plugin!(VJoyOutputPlugin);

// ---------------------------------------------------------------------------
// Tests (pure functions, no vJoy hardware required)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- Scaling (via vjoy_wrapper re-export) ---

    #[test]
    fn signed_full_left() {
        assert_eq!(vjoy_wrapper::map_signed_to_raw(-1.0), 0);
    }

    #[test]
    fn signed_center() {
        assert_eq!(vjoy_wrapper::map_signed_to_raw(0.0), 16384);
    }

    #[test]
    fn signed_full_right() {
        assert_eq!(vjoy_wrapper::map_signed_to_raw(1.0), 32767);
    }

    #[test]
    fn signed_clamp_under() {
        assert_eq!(vjoy_wrapper::map_signed_to_raw(-2.5), 0);
    }

    #[test]
    fn signed_clamp_over() {
        assert_eq!(vjoy_wrapper::map_signed_to_raw(2.5), 32767);
    }

    #[test]
    fn unsigned_zero() {
        assert_eq!(vjoy_wrapper::map_unsigned_to_raw(0.0), 0);
    }

    #[test]
    fn unsigned_full() {
        assert_eq!(vjoy_wrapper::map_unsigned_to_raw(1.0), 32767);
    }

    #[test]
    fn unsigned_clamp_under() {
        assert_eq!(vjoy_wrapper::map_unsigned_to_raw(-0.5), 0);
    }

    #[test]
    fn unsigned_clamp_over() {
        assert_eq!(vjoy_wrapper::map_unsigned_to_raw(1.5), 32767);
    }

    // --- Emergency brake ---

    #[test]
    fn emergency_brake_when_key_true() {
        let ctx = PluginContext::test();
        ctx.blackboard.set("safety.emergency_brake", "true");
        assert!(should_emergency_brake(&ctx));
    }

    #[test]
    fn no_emergency_brake_when_key_absent() {
        let ctx = PluginContext::test();
        assert!(!should_emergency_brake(&ctx));
    }

    #[test]
    fn no_emergency_brake_when_key_false() {
        let ctx = PluginContext::test();
        ctx.blackboard.set("safety.emergency_brake", "false");
        assert!(!should_emergency_brake(&ctx));
    }

    #[test]
    fn autopilot_off_when_state_missing() {
        let ctx = PluginContext::test();
        assert!(is_autopilot_off(&ctx));
    }

    #[test]
    fn autopilot_off_when_state_is_off() {
        let ctx = PluginContext::test();
        ctx.blackboard.set("autopilot.state", "Off");
        assert!(is_autopilot_off(&ctx));
    }

    #[test]
    fn autopilot_not_off_when_active() {
        for state in ["Engaging", "Active", "Paused", "Fault"] {
            let ctx = PluginContext::test();
            ctx.blackboard.set("autopilot.state", state);
            assert!(
                !is_autopilot_off(&ctx),
                "expected not-off for state={state}"
            );
        }
    }

    #[test]
    fn write_neutral_axes_returns_zeros() {
        let (s, t, b) = write_neutral_axes();
        assert_eq!((s, t, b), (0.0, 0.0, 0.0));
    }

    // --- Watchdog ---

    #[test]
    fn watchdog_triggers_after_timeout() {
        // 500ms / 20ms = 25 ticks; delta of 26 must trigger
        assert!(is_watchdog_expired(0, 26, 500));
    }

    #[test]
    fn watchdog_ok_within_timeout() {
        // delta of exactly 25 = not expired (> not >=)
        assert!(!is_watchdog_expired(0, 25, 500));
    }

    #[test]
    fn watchdog_handles_wraparound() {
        // last near u64::MAX, current has wrapped to 1 → 32 ticks elapsed (> 25)
        assert!(is_watchdog_expired(u64::MAX - 30, 1, 500));
    }

    #[test]
    fn watchdog_no_spurious_trigger_on_small_wraparound() {
        // last=MAX, current=5 → only 6 ticks elapsed (wrapping) → should NOT fire
        assert!(!is_watchdog_expired(u64::MAX, 5, 500));
    }

    #[test]
    fn watchdog_ok_at_tick_zero() {
        // last=0, current=0 → delta=0 → not expired
        assert!(!is_watchdog_expired(0, 0, 500));
    }

    #[test]
    fn cold_start_watchdog_boundary() {
        // Cold start: last_tick=0. Watchdog must NOT fire before timeout elapses,
        // and must fire exactly one tick after the boundary.
        let timeout_ms = 500u64;
        let boundary = timeout_ms / 20; // = 25
        assert!(!is_watchdog_expired(0, boundary, timeout_ms)); // tick 25: ok
        assert!(is_watchdog_expired(0, boundary + 1, timeout_ms)); // tick 26: fires
    }

    // --- Idle-output detection ---

    /// Idle float inputs (steer=0.0, throttle=0.0, brake=0.0) are correctly
    /// detected by is_idle_output, and steer=0.0 maps to raw 16384 (center),
    /// NOT raw 0. raw 0 = full-left; raw 16384 = hardware center.
    #[test]
    fn idle_output_float_zero_is_detected_and_maps_to_steer_center_raw() {
        assert!(is_idle_output(0.0, 0.0, 0.0));
        // The critical assertion: idle steer float 0.0 → raw 16384, not raw 0.
        assert_eq!(vjoy_wrapper::map_signed_to_raw(0.0), 16384);
        assert_ne!(vjoy_wrapper::map_signed_to_raw(0.0), 0);
        // Throttle and brake idle → raw 0 (zero, correct for unsigned axes).
        assert_eq!(vjoy_wrapper::map_unsigned_to_raw(0.0), 0);
    }

    #[test]
    fn not_idle_when_steering_nonzero() {
        assert!(!is_idle_output(0.1, 0.0, 0.0));
        assert!(!is_idle_output(-0.1, 0.0, 0.0));
    }

    #[test]
    fn not_idle_when_throttle_nonzero() {
        assert!(!is_idle_output(0.0, 0.3, 0.0));
    }

    #[test]
    fn not_idle_when_brake_nonzero() {
        assert!(!is_idle_output(0.0, 0.0, 0.5));
    }

    #[test]
    fn idle_output_center_steer_means_zero_float() {
        // Steering center maps to 0.0 f64 (plugin convention: 0.0 = center).
        // Confirm that map_signed_to_raw(0.0) == 16384 (hardware center).
        assert_eq!(vjoy_wrapper::map_signed_to_raw(0.0), 16384);
        assert!(is_idle_output(0.0, 0.0, 0.0));
    }
}
