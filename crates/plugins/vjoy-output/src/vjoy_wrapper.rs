//! VJoyHandle — thin Windows-only wrapper around the vjoy v0.4 crate.

use std::fmt;

// ---------------------------------------------------------------------------
// Axis constants (align 1:1 with vjoy_probe.rs)
// ---------------------------------------------------------------------------

const AXIS_MIN: i32 = 0;
const AXIS_MAX: i32 = 32_767;

pub const STEER_AXIS: u32 = 1; // HID_USAGE_X
pub const THROTTLE_AXIS: u32 = 2; // HID_USAGE_Y
pub const BRAKE_AXIS: u32 = 3; // HID_USAGE_Z

// ---------------------------------------------------------------------------
// Error types
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub enum VJoyInitError {
    DllNotFound(String),
    DeviceNotConfigured(u32),
    AxisMissing(u32, u32),
    Other(String),
}

impl fmt::Display for VJoyInitError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DllNotFound(p) => write!(
                f,
                "vJoy driver not found. Install from https://github.com/njz3/vJoy/releases (dll: {p})"
            ),
            Self::DeviceNotConfigured(id) => write!(
                f,
                "Device {id} not configured in vJoyConf (enable Device {id} with X/Y/Z axes), \
                 or owned by another process"
            ),
            Self::AxisMissing(id, axis) => write!(
                f,
                "Device {id} missing axis {axis}. Reconfigure in vJoyConf (enable X, Y, Z axes)."
            ),
            Self::Other(s) => write!(f, "{s}"),
        }
    }
}

#[derive(Debug)]
pub enum VJoySendError {
    AxisError(u32),
    UpdateError(String),
}

impl fmt::Display for VJoySendError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::AxisError(axis) => write!(f, "vJoy set_axis failed on axis {axis}"),
            Self::UpdateError(s) => write!(f, "vJoy update_device_state failed: {s}"),
        }
    }
}

// ---------------------------------------------------------------------------
// Axis scaling (1:1 from vjoy_probe.rs — unit-testable, no hardware needed)
// ---------------------------------------------------------------------------

/// Maps `value` in `[-1.0, 1.0]` to `[AXIS_MIN, AXIS_MAX]` with center at
/// 16384. Out-of-range inputs are clamped.
pub fn map_signed_to_raw(value: f64) -> i32 {
    let clamped = value.clamp(-1.0, 1.0);
    let center = (AXIS_MIN + AXIS_MAX) as f64 / 2.0;
    let half_range = (AXIS_MAX - AXIS_MIN) as f64 / 2.0;
    (center + clamped * half_range).round() as i32
}

/// Maps `value` in `[0.0, 1.0]` to `[AXIS_MIN, AXIS_MAX]`. Clamps.
pub fn map_unsigned_to_raw(value: f64) -> i32 {
    let clamped = value.clamp(0.0, 1.0);
    let span = (AXIS_MAX - AXIS_MIN) as f64;
    (AXIS_MIN as f64 + clamped * span).round() as i32
}

// ---------------------------------------------------------------------------
// VJoyHandle
// ---------------------------------------------------------------------------

#[cfg(windows)]
pub struct VJoyHandle {
    vjoy: vjoy::VJoy,
    device: vjoy::Device,
    pub device_id: u32,
    pub connected: bool,
}

#[cfg(windows)]
impl VJoyHandle {
    /// Acquire vJoy device `device_id`, verify X/Y/Z axes. Returns error if
    /// the driver is missing, the device is unconfigured/busy, or axes absent.
    pub fn try_acquire(device_id: u32) -> Result<Self, VJoyInitError> {
        use vjoy::{Error, FFIError, VJoy};

        let mut vjoy = VJoy::from_default_dll_location().map_err(|e| match e {
            Error::Ffi(FFIError::DynamicLybraryNotFound(p)) => VJoyInitError::DllNotFound(p),
            other => VJoyInitError::Other(format!("{other}")),
        })?;

        let mut device = vjoy
            .get_device_state(device_id)
            .map_err(|_| VJoyInitError::DeviceNotConfigured(device_id))?;

        let center = AXIS_MAX / 2;
        for axis_id in [STEER_AXIS, THROTTLE_AXIS, BRAKE_AXIS] {
            if device.set_axis(axis_id, center).is_err() {
                return Err(VJoyInitError::AxisMissing(device_id, axis_id));
            }
        }
        let _ = vjoy.update_device_state(&device);

        Ok(Self {
            vjoy,
            device,
            device_id,
            connected: true,
        })
    }

    /// Write steering, throttle, brake axes to the device.
    pub fn set_axes(&mut self, steer: f64, throttle: f64, brake: f64) -> Result<(), VJoySendError> {
        self.set_axes_verified(steer, throttle, brake).map(|_| ())
    }

    /// Like `set_axes` but also returns the raw i32 values that were passed
    /// to `device.set_axis` immediately before `update_device_state`.
    /// (steer_raw, throttle_raw, brake_raw) — steer center = 16384, not 0.
    /// If last_raw_x == 0 in the blackboard the mapping path itself is the bug.
    pub fn set_axes_verified(
        &mut self,
        steer: f64,
        throttle: f64,
        brake: f64,
    ) -> Result<(i32, i32, i32), VJoySendError> {
        let steer_raw = map_signed_to_raw(steer);
        let throttle_raw = map_unsigned_to_raw(throttle);
        let brake_raw = map_unsigned_to_raw(brake);

        if self.device.set_axis(STEER_AXIS, steer_raw).is_err() {
            return Err(VJoySendError::AxisError(STEER_AXIS));
        }
        if self.device.set_axis(THROTTLE_AXIS, throttle_raw).is_err() {
            return Err(VJoySendError::AxisError(THROTTLE_AXIS));
        }
        if self.device.set_axis(BRAKE_AXIS, brake_raw).is_err() {
            return Err(VJoySendError::AxisError(BRAKE_AXIS));
        }

        self.vjoy
            .update_device_state(&self.device)
            .map_err(|e| VJoySendError::UpdateError(format!("{e}")))?;

        Ok((steer_raw, throttle_raw, brake_raw))
    }

    /// Set all axes to safe/neutral values and mark disconnected. The vJoy
    /// handle is released when this struct drops.
    pub fn center_and_release(&mut self) {
        let center = AXIS_MAX / 2;
        let _ = self.device.set_axis(STEER_AXIS, center);
        let _ = self.device.set_axis(THROTTLE_AXIS, AXIS_MIN);
        let _ = self.device.set_axis(BRAKE_AXIS, AXIS_MIN);
        let _ = self.vjoy.update_device_state(&self.device);
        self.connected = false;
    }

    /// Attempt to re-acquire the device. Returns `true` on success.
    pub fn try_reconnect(&mut self) -> bool {
        match Self::try_acquire(self.device_id) {
            Ok(new_handle) => {
                *self = new_handle;
                true
            }
            Err(_) => false,
        }
    }
}

// ---------------------------------------------------------------------------
// Tests (scaling functions only — no hardware required)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signed_full_left() {
        assert_eq!(map_signed_to_raw(-1.0), 0);
    }

    #[test]
    fn signed_center() {
        assert_eq!(map_signed_to_raw(0.0), 16384);
    }

    #[test]
    fn signed_full_right() {
        assert_eq!(map_signed_to_raw(1.0), 32767);
    }

    #[test]
    fn signed_clamp_under() {
        assert_eq!(map_signed_to_raw(-2.5), 0);
    }

    #[test]
    fn signed_clamp_over() {
        assert_eq!(map_signed_to_raw(2.5), 32767);
    }

    #[test]
    fn unsigned_zero() {
        assert_eq!(map_unsigned_to_raw(0.0), 0);
    }

    #[test]
    fn unsigned_full() {
        assert_eq!(map_unsigned_to_raw(1.0), 32767);
    }

    #[test]
    fn unsigned_clamp_under() {
        assert_eq!(map_unsigned_to_raw(-0.5), 0);
    }

    #[test]
    fn unsigned_clamp_over() {
        assert_eq!(map_unsigned_to_raw(1.5), 32767);
    }
}
