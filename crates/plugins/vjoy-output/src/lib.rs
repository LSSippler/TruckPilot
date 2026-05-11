//! vJoy-Output plugin — sends ControlOutput to the vJoy virtual joystick.
//!
//! Must run LAST in the plugin order (letztes Plugin schreibt, gewinnt).
//! On non-Windows platforms falls back to console output.

use truckpilot_plugin_api::{ControlOutput, Plugin, PluginContext, Telemetry};

pub struct VJoyOutputPlugin {
    #[allow(dead_code)]
    device_id: u32,
    #[cfg(windows)]
    #[allow(dead_code)]
    acquired: bool,
}

impl Default for VJoyOutputPlugin {
    fn default() -> Self {
        Self {
            device_id: 1,
            #[cfg(windows)]
            acquired: false,
        }
    }
}

// ---------------------------------------------------------------------------
// Axis scaling (same as TruckPilot 1.0)
// ---------------------------------------------------------------------------

#[allow(dead_code)]
const VJOY_MIN: i32 = 1;
#[allow(dead_code)]
const VJOY_MAX: i32 = 32768;
#[allow(dead_code)]
const VJOY_CENTER: i32 = 16384;

#[cfg_attr(not(windows), allow(dead_code))]
fn scale_steering(v: f64) -> i32 {
    ((v.clamp(-1.0, 1.0) + 1.0) / 2.0 * 32767.0) as i32 + 1
}

#[cfg_attr(not(windows), allow(dead_code))]
fn scale_throttle(v: f64) -> i32 {
    (VJOY_CENTER as f64 + v.clamp(0.0, 1.0) * (VJOY_MAX - VJOY_CENTER) as f64).round() as i32
}

#[cfg_attr(not(windows), allow(dead_code))]
fn scale_brake(v: f64) -> i32 {
    (VJOY_CENTER as f64 - v.clamp(0.0, 1.0) * (VJOY_CENTER - VJOY_MIN) as f64).round() as i32
}

// ---------------------------------------------------------------------------
// Platform output
// ---------------------------------------------------------------------------

#[cfg(windows)]
fn send_to_vjoy(_device_id: u32, output: &ControlOutput) {
    // Dynamic loading of vJoyInterface.dll — same approach as TruckPilot 1.0.
    // Stub: actual DLL loading is handled by the core's vjoy module.
    tracing::debug!(
        "[vjoy-output] steer={} thr={} brk={}",
        scale_steering(output.steering),
        scale_throttle(output.throttle),
        scale_brake(output.brake),
    );
}

#[cfg(not(windows))]
fn send_to_vjoy(_device_id: u32, output: &ControlOutput) {
    tracing::debug!(
        "[vjoy-output] steer={:.3} thr={:.3} brk={:.3}",
        output.steering,
        output.throttle,
        output.brake
    );
}

// ---------------------------------------------------------------------------
// Plugin impl
// ---------------------------------------------------------------------------

impl Plugin for VJoyOutputPlugin {
    fn name(&self) -> &str {
        "vjoy-output"
    }
    fn version(&self) -> &str {
        "0.1.0"
    }
    fn settings_schema(&self) -> &str {
        r#"{"type":"object","properties":{"device_id":{"type":"integer","minimum":1,"maximum":16}}}"#
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        if let Some(id) = ctx.blackboard.get_f64("vjoy.device_id") {
            self.device_id = (id as u32).clamp(1, 16);
        }
        tracing::info!("[vjoy-output] loaded — device={}", self.device_id);
    }

    fn on_unload(&mut self) {
        tracing::info!("[vjoy-output] unloaded");
    }

    fn tick(
        &mut self,
        _telemetry: Option<&Telemetry>,
        output: &mut ControlOutput,
        _ctx: &PluginContext,
    ) {
        send_to_vjoy(self.device_id, output);
    }
}

truckpilot_plugin_api::export_plugin!(VJoyOutputPlugin);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scale_steering_center() {
        assert_eq!(scale_steering(0.0), 16384);
    }

    #[test]
    fn scale_steering_full_left() {
        assert_eq!(scale_steering(-1.0), 1);
    }

    #[test]
    fn scale_steering_full_right() {
        assert_eq!(scale_steering(1.0), 32768);
    }

    #[test]
    fn scale_throttle_zero() {
        assert_eq!(scale_throttle(0.0), 16384);
    }

    #[test]
    fn scale_throttle_full() {
        assert_eq!(scale_throttle(1.0), 32768);
    }

    #[test]
    fn scale_brake_zero() {
        assert_eq!(scale_brake(0.0), 16384);
    }

    #[test]
    fn scale_brake_full() {
        assert_eq!(scale_brake(1.0), 1);
    }
}
