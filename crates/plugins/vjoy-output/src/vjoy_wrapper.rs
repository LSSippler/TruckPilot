//! VJoyHandle — direct FFI wrapper around `vJoyInterface.dll`.
//!
//! Replaces the `vjoy` 0.4 crate, which buffered axis writes into a
//! `JOYSTICK_POSITION_V2` struct and pushed them via `UpdateVJD()`.
//! That buffered pattern triggered a state-cache bug when acquire and
//! the first `UpdateVJD` were temporally separated (plugin `on_load`
//! followed by a delayed first `tick`): the driver reported
//! "Device Status: 1" (= VJD_STAT_FREE) and the hardware never saw
//! the write, even though `AcquireVJD` had returned success.
//!
//! This wrapper calls `SetAxis()` directly. SetAxis writes immediately
//! to the device — no intermediate buffer, no UpdateVJD, no cache.

use std::fmt;

// ---------------------------------------------------------------------------
// Axis range + HID usage constants
// ---------------------------------------------------------------------------

const AXIS_MIN: i32 = 0;
const AXIS_MAX: i32 = 32_767;

pub const HID_USAGE_X:   u32 = 0x30; // Steering (bipolar, center = 16384)
pub const HID_USAGE_SL0: u32 = 0x36; // Slider       — Throttle (unipolar)
pub const HID_USAGE_SL1: u32 = 0x37; // Dial/Slider2 — Brake    (unipolar)

// VjdStat values returned by GetVJDStatus (vJoy SDK).
const VJD_STAT_OWN: i32 = 0;
const VJD_STAT_FREE: i32 = 1;
const VJD_STAT_BUSY: i32 = 2;
const VJD_STAT_MISS: i32 = 3;

// ---------------------------------------------------------------------------
// Error types
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub enum VJoyInitError {
    DllNotFound(String),
    NotEnabled,
    DeviceBusy(u32),
    DeviceMissing(u32),
    DeviceUnknown(u32, i32),
    AcquireFailed(u32, i32),
    SymbolMissing(String),
}

impl fmt::Display for VJoyInitError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DllNotFound(p) => write!(
                f,
                "vJoyInterface.dll not found ({p}). Install from https://github.com/njz3/vJoy/releases"
            ),
            Self::NotEnabled => write!(
                f,
                "vJoy driver is installed but disabled (vJoyEnabled returned 0). \
                 Open vJoyConf and enable the driver."
            ),
            Self::DeviceBusy(id) => write!(
                f,
                "Device {id} is busy (owned by another process). Close ETS2-LA, \
                 x360ce, joy.cpl test pane, or any previous truckpilot instance."
            ),
            Self::DeviceMissing(id) => write!(
                f,
                "Device {id} is not configured in vJoyConf. Enable Device {id} \
                 with X, Slider and Dial/Slider2 axes."
            ),
            Self::DeviceUnknown(id, status) => write!(
                f,
                "Device {id} has unknown status {status}. Driver may be in an \
                 inconsistent state — try reinstalling vJoy."
            ),
            Self::AcquireFailed(id, status) => write!(
                f,
                "AcquireVJD({id}) failed (post-acquire status={status})."
            ),
            Self::SymbolMissing(name) => write!(
                f,
                "vJoyInterface.dll is missing required symbol: {name}. \
                 DLL version may be too old."
            ),
        }
    }
}

#[derive(Debug)]
pub enum VJoySendError {
    AxisError(u32),
}

impl fmt::Display for VJoySendError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::AxisError(axis) => {
                write!(
                    f,
                    "SetAxis(0x{axis:02x}) returned FALSE — axis missing or device released"
                )
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Axis scaling (pure, unit-testable, no hardware required)
// ---------------------------------------------------------------------------

/// Maps `value` in `[-1.0, 1.0]` to `[AXIS_MIN, AXIS_MAX]` with center at
/// 16384. Out-of-range inputs are clamped. Identical math to the probe binary.
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
// FFI: function-pointer signatures
// ---------------------------------------------------------------------------

// All vJoyInterface exports use the Windows stdcall convention; on x64
// stdcall == fastcall == "system" so `extern "system"` is correct.
type FnVJoyEnabled       = unsafe extern "system" fn() -> i32;
type FnAcquireVJD        = unsafe extern "system" fn(u32) -> i32;
type FnRelinquishVJD     = unsafe extern "system" fn(u32);
type FnGetVJDStatus      = unsafe extern "system" fn(u32) -> i32;
type FnSetAxis           = unsafe extern "system" fn(i32, u32, u32) -> i32;
type FnGetVJDAxisExist   = unsafe extern "system" fn(u32, u32) -> i32;

// ---------------------------------------------------------------------------
// VJoyHandle (Windows only)
// ---------------------------------------------------------------------------

#[cfg(windows)]
use windows::Win32::Foundation::HMODULE;

#[cfg(windows)]
pub struct VJoyHandle {
    /// Module handle from LoadLibraryW. Reference-counted by the loader so
    /// concurrent reconnects (each doing LoadLibraryW+FreeLibrary) net out.
    dll: HMODULE,
    pub device_id: u32,
    pub connected: bool,
    // Cached function pointers — valid as long as `dll` ref-count > 0.
    relinquish_vjd: FnRelinquishVJD,
    set_axis: FnSetAxis,
}

#[cfg(windows)]
impl VJoyHandle {
    /// Acquire vJoy device `device_id`. Loads vJoyInterface.dll, resolves
    /// the required symbols, checks `vJoyEnabled`, queries `GetVJDStatus`
    /// (must be FREE or OWN), calls `AcquireVJD`, re-verifies status is
    /// OWN, then commits a center-write that must reach hardware before
    /// the function returns.
    pub fn try_acquire(device_id: u32) -> Result<Self, VJoyInitError> {
        use windows::core::w;
        use windows::Win32::System::LibraryLoader::LoadLibraryW;

        // Prefer the canonical install path; fall back to PATH lookup so
        // dev machines that put the DLL in PATH still work.
        let dll = unsafe {
            LoadLibraryW(w!("C:\\Program Files\\vJoy\\x64\\vJoyInterface.dll"))
                .or_else(|_| LoadLibraryW(w!("vJoyInterface.dll")))
                .map_err(|e| VJoyInitError::DllNotFound(format!("{e}")))?
        };

        // Resolve all symbols up front so a torn DLL fails cleanly here,
        // not on the first hot-path call.
        let vjoy_enabled: FnVJoyEnabled = unsafe { load_sym(dll, "vJoyEnabled")? };
        let acquire_vjd: FnAcquireVJD = unsafe { load_sym(dll, "AcquireVJD")? };
        let relinquish_vjd: FnRelinquishVJD = unsafe { load_sym(dll, "RelinquishVJD")? };
        let get_vjd_status: FnGetVJDStatus = unsafe { load_sym(dll, "GetVJDStatus")? };
        let set_axis: FnSetAxis = unsafe { load_sym(dll, "SetAxis")? };

        // Helper to roll back DLL load if any check fails below.
        let unload_on_fail = |e: VJoyInitError| -> VJoyInitError {
            unsafe {
                let _ = windows::Win32::Foundation::FreeLibrary(dll);
            }
            e
        };

        if unsafe { vjoy_enabled() } == 0 {
            return Err(unload_on_fail(VJoyInitError::NotEnabled));
        }

        let status = unsafe { get_vjd_status(device_id) };
        match status {
            VJD_STAT_FREE | VJD_STAT_OWN => {}
            VJD_STAT_BUSY => return Err(unload_on_fail(VJoyInitError::DeviceBusy(device_id))),
            VJD_STAT_MISS => return Err(unload_on_fail(VJoyInitError::DeviceMissing(device_id))),
            _ => {
                // VJD_STAT_UNKN and any forward-compatible value.
                return Err(unload_on_fail(VJoyInitError::DeviceUnknown(
                    device_id, status,
                )));
            }
        }

        if unsafe { acquire_vjd(device_id) } == 0 {
            let post_status = unsafe { get_vjd_status(device_id) };
            return Err(unload_on_fail(VJoyInitError::AcquireFailed(
                device_id,
                post_status,
            )));
        }

        // Re-verify post-acquire status is OWN — catches "Acquire-returned-
        // true-but-driver-says-FREE" mysteries from buggy drivers.
        let post_status = unsafe { get_vjd_status(device_id) };
        if post_status != VJD_STAT_OWN {
            unsafe {
                relinquish_vjd(device_id);
            }
            return Err(unload_on_fail(VJoyInitError::AcquireFailed(
                device_id,
                post_status,
            )));
        }

        // Optional axis-existence pre-check: warn if SL0/SL1 are not
        // configured in vJoyConf. Non-fatal — the acquire already succeeded
        // and SetAxis will simply return FALSE on missing axes.
        if let Ok(axis_exists_fn) =
            unsafe { load_sym::<FnGetVJDAxisExist>(dll, "GetVJDAxisExist") }
        {
            for (hid, name) in [
                (HID_USAGE_SL0, "Slider (Throttle)"),
                (HID_USAGE_SL1, "Dial/Slider2 (Brake)"),
            ] {
                if unsafe { axis_exists_fn(device_id, hid) } == 0 {
                    tracing::warn!(
                        "[vjoy-output] vJoy axis 0x{hid:02x} ({name}) is NOT enabled in \
                         vJoyConf — activate it and restart. TruckPilot writes throttle/brake \
                         to Slider and Dial/Slider2, NOT to Y/Z."
                    );
                }
            }
        }

        // Commit the device to a known neutral state, then wait 100 ms
        // before returning. The blocking sleep is part of the fix, not a
        // diagnostic: the caller (plugin `on_load`) runs on a tokio task
        // and would yield to a different OS thread on the next .await.
        // Holding the thread here long enough for the driver to commit
        // the writes stops the cdylib/runtime boundary from racing the
        // first user-visible state.
        let center = map_signed_to_raw(0.0);
        unsafe {
            let _ = set_axis(center, device_id, HID_USAGE_X);
            let _ = set_axis(0, device_id, HID_USAGE_SL0);
            let _ = set_axis(0, device_id, HID_USAGE_SL1);
        }
        std::thread::sleep(std::time::Duration::from_millis(100));
        unsafe {
            let _ = set_axis(center, device_id, HID_USAGE_X);
        }

        // Drop the symbols we won't need on the hot path now that
        // try_acquire is done with them.
        let _ = vjoy_enabled;
        let _ = acquire_vjd;
        let _ = get_vjd_status;

        Ok(Self {
            dll,
            device_id,
            connected: true,
            relinquish_vjd,
            set_axis,
        })
    }

    /// Write steering, throttle, brake axes to the device. Returns the
    /// raw i32 values that were sent to `SetAxis` so the caller can
    /// surface them on the blackboard.
    pub fn set_axes_verified(
        &mut self,
        steer: f64,
        throttle: f64,
        brake: f64,
    ) -> Result<(i32, i32, i32), VJoySendError> {
        let steer_raw = map_signed_to_raw(steer);
        let throttle_raw = map_unsigned_to_raw(throttle);
        let brake_raw = map_unsigned_to_raw(brake);

        unsafe {
            if (self.set_axis)(steer_raw, self.device_id, HID_USAGE_X) == 0 {
                return Err(VJoySendError::AxisError(HID_USAGE_X));
            }
            if (self.set_axis)(throttle_raw, self.device_id, HID_USAGE_SL0) == 0 {
                return Err(VJoySendError::AxisError(HID_USAGE_SL0));
            }
            if (self.set_axis)(brake_raw, self.device_id, HID_USAGE_SL1) == 0 {
                return Err(VJoySendError::AxisError(HID_USAGE_SL1));
            }
        }

        Ok((steer_raw, throttle_raw, brake_raw))
    }

    /// Convenience: discard raw values, return unit error.
    pub fn set_axes(&mut self, steer: f64, throttle: f64, brake: f64) -> Result<(), VJoySendError> {
        self.set_axes_verified(steer, throttle, brake).map(|_| ())
    }

    /// Move all axes to safe/neutral and relinquish the device.
    pub fn center_and_release(&mut self) {
        if self.connected {
            unsafe {
                let _ = (self.set_axis)(map_signed_to_raw(0.0), self.device_id, HID_USAGE_X);
                let _ = (self.set_axis)(0, self.device_id, HID_USAGE_SL0);
                let _ = (self.set_axis)(0, self.device_id, HID_USAGE_SL1);
                (self.relinquish_vjd)(self.device_id);
            }
            self.connected = false;
        }
    }

    /// Drop the current handle and re-acquire from scratch.
    /// Returns `true` on success. Reserved for future error recovery
    /// (e.g. driver-initiated relinquish detected by SetAxis failure).
    #[allow(dead_code)]
    pub fn try_reconnect(&mut self) -> bool {
        let device_id = self.device_id;
        // Release current handle first so we don't acquire on top of an
        // already-owned device (which is valid but wastes a syscall).
        self.center_and_release();
        match Self::try_acquire(device_id) {
            Ok(new_handle) => {
                *self = new_handle;
                true
            }
            Err(_) => false,
        }
    }
}

#[cfg(windows)]
impl Drop for VJoyHandle {
    fn drop(&mut self) {
        if self.connected {
            unsafe {
                (self.relinquish_vjd)(self.device_id);
            }
            self.connected = false;
        }
        // Decrement the DLL's reference count. The loader only actually
        // unmaps when the count reaches zero — safe even if another
        // VJoyHandle is alive concurrently.
        unsafe {
            let _ = windows::Win32::Foundation::FreeLibrary(self.dll);
        }
    }
}

// ---------------------------------------------------------------------------
// Symbol resolution helper
// ---------------------------------------------------------------------------

/// Resolve a vJoy DLL export to a typed function pointer.
///
/// # Safety
/// Caller must guarantee `F` matches the actual export signature.
/// `name` must be the exact ANSI symbol name.
#[cfg(windows)]
unsafe fn load_sym<F: Sized>(dll: HMODULE, name: &str) -> Result<F, VJoyInitError> {
    use windows::core::PCSTR;
    use windows::Win32::System::LibraryLoader::GetProcAddress;

    // GetProcAddress requires a null-terminated ANSI string.
    let mut cstr = String::with_capacity(name.len() + 1);
    cstr.push_str(name);
    cstr.push('\0');

    let proc = GetProcAddress(dll, PCSTR(cstr.as_ptr()));
    match proc {
        Some(addr) => {
            debug_assert_eq!(
                std::mem::size_of::<F>(),
                std::mem::size_of::<unsafe extern "system" fn() -> isize>()
            );
            Ok(std::mem::transmute_copy::<
                unsafe extern "system" fn() -> isize,
                F,
            >(&addr))
        }
        None => Err(VJoyInitError::SymbolMissing(name.to_string())),
    }
}

// ---------------------------------------------------------------------------
// Tests — pure scaling functions only, no hardware required.
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
