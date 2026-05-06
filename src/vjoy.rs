//! vJoy virtual joystick interface.
//!
//! Uses dynamic loading (`LoadLibrary`/`GetProcAddress`) so the binary
//! compiles and runs even without vJoy installed. Falls back to
//! `ConsoleOutput` gracefully.
//!
//! # ETS2 Axis Mapping (vJoy SDK v2.x / API Version 3)
//!
//! | vJoy Field  | HID Usage | ETS2 Input        | Value Range               |
//! |-------------|-----------|-------------------|---------------------------|
//! | `wAxisX`    | 0x30      | Steering          | 1 .. 32768 (center 16384) |
//! | `wAxisY`    | 0x31      | Throttle + Brake  | 1 .. 32768 (center 16384) |
//! | `wAxisZ`    | 0x32      | Clutch (unused)   | 16384 (fixed center)      |
//!
//! Y-axis is split by the vJoy driver internally:
//!   upper half (16384..32768) = throttle, lower half (1..16384) = brake.
//! Brake takes priority over throttle when both are non-zero.
//!
//! # Scaling reference
//!
//! vJoy axis range: 0x0001 (1) .. 0x8000 (32768), center = 0x4000 (16384).

#[cfg(windows)]
use std::ffi::c_void;
#[cfg(windows)]
use std::time::{Duration, Instant};

// ---------------------------------------------------------------------------
// JOYSTICK_POSITION_V3 — exact layout from vJoy SDK public.h
// (USE_JOYSTICK_API_VERSION == 3)
// ---------------------------------------------------------------------------

#[repr(C)]
#[allow(dead_code)]
struct JoystickPositionV3 {
    b_device: u8, // 1-based device index
    // V1 legacy axes (positions 1-12)
    w_throttle: i32,
    w_rudder: i32,
    w_aileron: i32,
    w_axis_x: i32, // HID 0x30 — Steering
    w_axis_y: i32, // HID 0x31 — Throttle + Brake combined
    w_axis_z: i32, // HID 0x32 — Clutch
    w_axis_x_rot: i32,
    w_axis_y_rot: i32,
    w_axis_z_rot: i32,
    w_slider: i32,
    w_dial: i32,
    w_wheel: i32,
    // V3-specific replacement axes (positions 13-16, replace V1 VX/VY/VZ/VBRX)
    w_accelerator: i32,
    w_brake: i32,
    w_clutch: i32,
    w_steering: i32,
    // Remaining V1 fields
    w_axis_vx: i32,
    w_axis_vy: i32,
    // Buttons + hats
    l_buttons: i32,
    b_hats: u32,
    b_hats_ex1: u32,
    b_hats_ex2: u32,
    b_hats_ex3: u32,
    // V2 extension: extra buttons
    l_buttons_ex1: i32,
    l_buttons_ex2: i32,
    l_buttons_ex3: i32,
    // V3: fields moved from V1 positions to the tail
    w_axis_vz: i32,
    w_axis_vbrx: i32,
    w_axis_vbry: i32,
    w_axis_vbrz: i32,
}

#[allow(dead_code)]
const VJOY_AXIS_MIN: i32 = 1;
#[allow(dead_code)]
const VJOY_AXIS_MAX: i32 = 32768;
#[allow(dead_code)]
const VJOY_AXIS_CENTER: i32 = 16384;

// ---------------------------------------------------------------------------
// Scaling helpers (pub(crate) so tests can use them on any platform)
// ---------------------------------------------------------------------------

#[allow(dead_code)]
pub(crate) fn scale_steering(value: f64) -> i32 {
    let v = value.clamp(-1.0, 1.0);
    ((v + 1.0) / 2.0 * 32767.0) as i32 + 1
}

#[allow(dead_code)]
pub(crate) fn scale_throttle(value: f64) -> i32 {
    let v = value.clamp(0.0, 1.0);
    (VJOY_AXIS_CENTER as f64 + v * (VJOY_AXIS_MAX - VJOY_AXIS_CENTER) as f64).round() as i32
}

#[allow(dead_code)]
pub(crate) fn scale_brake(value: f64) -> i32 {
    let v = value.clamp(0.0, 1.0);
    (VJOY_AXIS_CENTER as f64 - v * (VJOY_AXIS_CENTER - VJOY_AXIS_MIN) as f64).round() as i32
}

/// Combine throttle and brake into a single Y-axis value.
/// Brake takes priority when both are non-zero.
#[allow(dead_code)]
pub(crate) fn compute_y_axis(throttle: f64, brake: f64) -> i32 {
    if brake > 0.0 {
        scale_brake(brake)
    } else if throttle > 0.0 {
        scale_throttle(throttle)
    } else {
        VJOY_AXIS_CENTER
    }
}

// ---------------------------------------------------------------------------
// ControlOutput trait
// ---------------------------------------------------------------------------

/// Trait for outputting vehicle control commands.
pub trait ControlOutput {
    /// Set desired steering angle. Range: -1.0 (full left) .. +1.0 (full right).
    fn set_steering(&mut self, value: f64);
    /// Set throttle. Range: 0.0 (idle) .. 1.0 (full).
    fn set_throttle(&mut self, value: f64);
    /// Set brake. Range: 0.0 (no brake) .. 1.0 (full brake).
    fn set_brake(&mut self, value: f64);
    /// Send all accumulated values to the output device.
    /// Called once per control cycle.
    fn flush(&mut self);

    /// Whether the output backend is currently available.
    fn is_available(&self) -> bool {
        true
    }

    /// Try to re-acquire or recover the output backend.
    fn try_reacquire(&mut self) -> bool {
        true
    }
}

// ---------------------------------------------------------------------------
// Console output (fallback)
// ---------------------------------------------------------------------------

/// Console-based control output (prints to stdout).
pub struct ConsoleOutput;

impl ControlOutput for ConsoleOutput {
    fn set_steering(&mut self, value: f64) {
        println!("steer={:.4}", value);
    }
    fn set_throttle(&mut self, value: f64) {
        println!("throttle={:.4}", value);
    }
    fn set_brake(&mut self, value: f64) {
        println!("brake={:.4}", value);
    }
    fn flush(&mut self) {}
}

// ---------------------------------------------------------------------------
// Dynamic vJoy loading (Windows only)
// ---------------------------------------------------------------------------

#[cfg(windows)]
mod ffi {
    use std::ffi::c_void;
    use std::sync::OnceLock;

    #[allow(clippy::upper_case_acronyms)]
    type HANDLE = isize;
    type FARPROC = isize;

    // vJoy function signatures.
    type VJoyEnabledFn = unsafe extern "system" fn() -> i32;
    type IsVJDExistsFn = unsafe extern "system" fn(u32) -> i32;
    type GetVJDStatusFn = unsafe extern "system" fn(u32) -> u32;
    type AcquireVJDFn = unsafe extern "system" fn(u32) -> i32;
    type RelinquishVJDFn = unsafe extern "system" fn(u32);
    type ResetVJDFn = unsafe extern "system" fn(u32);
    type UpdateVJDFn = unsafe extern "system" fn(device_id: u32, data: *mut c_void) -> i32;

    extern "system" {
        fn LoadLibraryA(lpLibFileName: *const u8) -> HANDLE;
        fn GetProcAddress(hModule: HANDLE, lpProcName: *const u8) -> FARPROC;
    }

    pub(crate) struct VJoyApi {
        _module: HANDLE,
        vjoy_enabled: VJoyEnabledFn,
        is_exists: IsVJDExistsFn,
        get_status: GetVJDStatusFn,
        acquire: AcquireVJDFn,
        relinquish: RelinquishVJDFn,
        reset: ResetVJDFn,
        update_vjd: UpdateVJDFn,
    }

    unsafe impl Send for VJoyApi {}
    unsafe impl Sync for VJoyApi {}

    static VJOY: OnceLock<Option<VJoyApi>> = OnceLock::new();

    fn load() -> Option<&'static VJoyApi> {
        VJOY.get_or_init(|| {
            let search_dlls: &[&[u8]] = &[
                b"C:\\Program Files\\vJoy\\x64\\vJoyInterface.dll\0",
                b"C:\\Program Files (x86)\\vJoy\\x64\\vJoyInterface.dll\0",
                b"vJoyInterface.dll\0",
            ];

            let mut h: HANDLE = 0;
            let mut tried: Vec<&str> = Vec::new();
            for name in search_dlls {
                let s = std::str::from_utf8(name).unwrap_or("?");
                tried.push(s.trim_end_matches('\0'));
                h = unsafe { LoadLibraryA(name.as_ptr()) };
                if h != 0 {
                    break;
                }
            }

            if h == 0 {
                eprintln!("vJoy: vJoyInterface.dll not found. Searched: {:?}", tried);
                eprintln!("vJoy: Install vJoy 2.2.1+ from https://github.com/njz3/vJoy");
                return None;
            }

            unsafe fn get<T>(h: HANDLE, name: &[u8]) -> Option<T> {
                let p = unsafe { GetProcAddress(h, name.as_ptr()) };
                if p == 0 {
                    return None;
                }
                Some(unsafe { std::mem::transmute_copy(&p) })
            }

            unsafe {
                Some(VJoyApi {
                    _module: h,
                    vjoy_enabled: get(h, b"vJoyEnabled\0")?,
                    is_exists: get(h, b"isVJDExists\0")?,
                    get_status: get(h, b"GetVJDStatus\0")?,
                    acquire: get(h, b"AcquireVJD\0")?,
                    relinquish: get(h, b"RelinquishVJD\0")?,
                    reset: get(h, b"ResetVJD\0")?,
                    update_vjd: get(h, b"UpdateVJD\0")?,
                })
            }
        })
        .as_ref()
    }

    pub(crate) fn try_load() -> Option<&'static VJoyApi> {
        load()
    }

    /// Check if the vJoy driver is running.
    pub fn vjoy_enabled() -> bool {
        load()
            .map(|a| unsafe { (a.vjoy_enabled)() != 0 })
            .unwrap_or(false)
    }

    /// Get status of a vJoy device: 0=OWN, 1=FREE, 2=BUSY, 3=MISS, 4=UNKN.
    pub fn is_exists(id: u32) -> bool {
        load()
            .map(|a| unsafe { (a.is_exists)(id) != 0 })
            .unwrap_or(false)
    }

    /// Get status of a vJoy device: 0=OWN, 1=FREE, 2=BUSY, 3=MISS, 4=UNKN.
    pub fn get_status(id: u32) -> u32 {
        load().map(|a| unsafe { (a.get_status)(id) }).unwrap_or(3) // MISS
    }

    /// Acquire a vJoy device. Returns true on success.
    pub fn acquire(id: u32) -> bool {
        load()
            .map(|a| unsafe { (a.acquire)(id) != 0 })
            .unwrap_or(false)
    }

    /// Release a vJoy device.
    pub fn relinquish(id: u32) {
        if let Some(a) = load() {
            unsafe { (a.relinquish)(id) }
        }
    }

    /// Reset all axes and buttons to default (center).
    pub fn reset(id: u32) {
        if let Some(a) = load() {
            unsafe { (a.reset)(id) }
        }
    }

    /// Send a full joystick position to the driver.
    pub fn update_vjd(device_id: u32, data: *mut c_void) -> bool {
        load()
            .map(|a| unsafe { (a.update_vjd)(device_id, data) != 0 })
            .unwrap_or(false)
    }
}

#[allow(dead_code)]
fn enumerate_vjoy_devices_impl<FExists, FStatus>(
    is_exists: FExists,
    get_status: FStatus,
) -> Vec<u32>
where
    FExists: Fn(u32) -> bool,
    FStatus: Fn(u32) -> u32,
{
    let mut devices = Vec::new();
    for id in 1..=16 {
        if is_exists(id) && get_status(id) == 1 {
            devices.push(id);
        }
    }
    devices
}

/// Enumerate vJoy devices that exist and are currently free (Windows only).
#[cfg(windows)]
pub fn enumerate_vjoy_devices() -> Vec<u32> {
    enumerate_vjoy_devices_impl(ffi::is_exists, ffi::get_status)
}

/// Enumerate vJoy devices — non-Windows stub, always returns an empty list.
#[cfg(not(windows))]
pub fn enumerate_vjoy_devices() -> Vec<u32> {
    Vec::new()
}

// ---------------------------------------------------------------------------
// VJoyOutput
// ---------------------------------------------------------------------------

/// Real vJoy output (Windows only, dynamically loaded).
///
/// Accumulates steering, throttle, and brake values in memory.
/// `flush()` sends them atomically via `UpdateVJD`.
#[cfg(windows)]
pub struct VJoyOutput {
    device_id: u32,
    acquired: bool,
    steer: f64,
    throttle: f64,
    brake: f64,
    last_update_error_log: Option<Instant>,
}

#[cfg(windows)]
impl VJoyOutput {
    /// Try to acquire the vJoy device.
    ///
    /// Returns `None` if the DLL cannot be loaded, the driver is not running,
    /// or the device is unavailable.
    pub fn try_acquire(device_id: u32) -> Option<Self> {
        // Step 1: Load DLL.
        let _api = ffi::try_load()?; // error already printed in load()

        // Step 2: Check driver is running.
        if !ffi::vjoy_enabled() {
            eprintln!(
                "vJoy: driver not running. Open 'Configure vJoy' and enable device {device_id}."
            );
            return None;
        }

        // Step 3: Check device status (must be FREE = 1).
        let status = ffi::get_status(device_id);
        match status {
            0 => eprintln!("vJoy: device {device_id} is already owned by this process."),
            1 => {} // FREE — ok
            2 => eprintln!("vJoy: device {device_id} is busy (owned by another app)."),
            3 => eprintln!(
                "vJoy: device {device_id} does not exist. Configure it in 'Configure vJoy'."
            ),
            _ => eprintln!("vJoy: device {device_id} status unknown ({status})."),
        }
        if status != 1 {
            return None;
        }

        // Step 4: Acquire device.
        if !ffi::acquire(device_id) {
            eprintln!("vJoy: failed to acquire device {device_id}.");
            return None;
        }

        // Step 5: Reset to center.
        ffi::reset(device_id);

        eprintln!("vJoy: acquired device {device_id}.");

        Some(Self {
            device_id,
            acquired: true,
            steer: 0.0,
            throttle: 0.0,
            brake: 0.0,
            last_update_error_log: None,
        })
    }
}

#[cfg(windows)]
impl crate::vjoy::ControlOutput for VJoyOutput {
    fn set_steering(&mut self, value: f64) {
        self.steer = value;
    }

    fn set_throttle(&mut self, value: f64) {
        self.throttle = value;
    }

    fn set_brake(&mut self, value: f64) {
        self.brake = value;
    }

    fn flush(&mut self) {
        use crate::vjoy::JoystickPositionV3;

        debug_assert!(
            (1..=16).contains(&(self.device_id as i32)),
            "vJoy device ID out of range: {}",
            self.device_id
        );

        let mut pos = JoystickPositionV3 {
            b_device: self.device_id as u8,
            w_axis_x: scale_steering(self.steer),
            w_axis_y: scale_throttle(self.throttle),
            w_axis_z: scale_brake(self.brake),
            // All other axes at center, buttons/hats at 0.
            w_throttle: VJOY_AXIS_CENTER,
            w_rudder: VJOY_AXIS_CENTER,
            w_aileron: VJOY_AXIS_CENTER,
            w_axis_x_rot: VJOY_AXIS_CENTER,
            w_axis_y_rot: VJOY_AXIS_CENTER,
            w_axis_z_rot: VJOY_AXIS_CENTER,
            w_slider: VJOY_AXIS_CENTER,
            w_dial: VJOY_AXIS_CENTER,
            w_wheel: VJOY_AXIS_CENTER,
            w_accelerator: VJOY_AXIS_CENTER,
            w_brake: VJOY_AXIS_CENTER,
            w_clutch: VJOY_AXIS_CENTER,
            w_steering: VJOY_AXIS_CENTER,
            w_axis_vx: VJOY_AXIS_CENTER,
            w_axis_vy: VJOY_AXIS_CENTER,
            w_axis_vz: VJOY_AXIS_CENTER,
            w_axis_vbrx: VJOY_AXIS_CENTER,
            w_axis_vbry: VJOY_AXIS_CENTER,
            w_axis_vbrz: VJOY_AXIS_CENTER,
            l_buttons: 0,
            b_hats: u32::MAX,
            b_hats_ex1: u32::MAX,
            b_hats_ex2: u32::MAX,
            b_hats_ex3: u32::MAX,
            l_buttons_ex1: 0,
            l_buttons_ex2: 0,
            l_buttons_ex3: 0,
        };

        let ptr = &mut pos as *mut JoystickPositionV3 as *mut c_void;
        if !ffi::update_vjd(self.device_id, ptr) {
            let should_log = self
                .last_update_error_log
                .map(|t| t.elapsed() >= Duration::from_secs(1))
                .unwrap_or(true);
            if should_log {
                eprintln!("vJoy: UpdateVJD failed for device {}.", self.device_id);
                self.last_update_error_log = Some(Instant::now());
            }
        } else {
            self.last_update_error_log = None;
        }
    }

    fn is_available(&self) -> bool {
        if !self.acquired {
            return false;
        }
        ffi::get_status(self.device_id) == 0
    }

    fn try_reacquire(&mut self) -> bool {
        if !ffi::vjoy_enabled() {
            return false;
        }

        let status = ffi::get_status(self.device_id);
        if status == 0 {
            self.acquired = true;
            true
        } else if status == 1 {
            if ffi::acquire(self.device_id) {
                self.acquired = true;
                ffi::reset(self.device_id);
                true
            } else {
                false
            }
        } else {
            false
        }
    }
}

#[cfg(windows)]
impl Drop for VJoyOutput {
    fn drop(&mut self) {
        if self.acquired {
            ffi::reset(self.device_id);
            ffi::relinquish(self.device_id);
            eprintln!("vJoy: released device {}.", self.device_id);
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
    fn test_scale_steering() {
        assert_eq!(scale_steering(-1.0), 1);
        assert_eq!(scale_steering(0.0), 16384);
        assert_eq!(scale_steering(1.0), 32768);
        assert_eq!(scale_steering(-2.0), 1);
        assert_eq!(scale_steering(2.0), 32768);
    }

    #[test]
    fn test_scale_throttle() {
        assert_eq!(scale_throttle(0.0), 16384);
        assert_eq!(scale_throttle(1.0), 32768);
        assert_eq!(scale_throttle(-0.5), 16384);
        assert_eq!(scale_throttle(2.0), 32768);
    }

    #[test]
    fn test_scale_brake() {
        assert_eq!(scale_brake(0.0), 16384);
        assert_eq!(scale_brake(1.0), 1);
        assert_eq!(scale_brake(-0.5), 16384);
        assert_eq!(scale_brake(2.0), 1);
    }

    #[test]
    fn test_joystick_position_layout() {
        let size = std::mem::size_of::<JoystickPositionV3>();
        assert_eq!(
            size, 124,
            "V3 struct mismatch: expected 124 bytes, got {size}"
        );
    }

    #[test]
    fn test_y_axis_combined_priority() {
        // When both throttle and brake are non-zero, brake takes priority.
        let y = compute_y_axis(0.5, 0.3);
        let expected_brake = scale_brake(0.3);
        let throttle_val = scale_throttle(0.5);
        assert_eq!(
            y, expected_brake,
            "brake should take priority over throttle"
        );
        assert_ne!(
            y, throttle_val,
            "y-axis should not be throttle value when brake is active"
        );
    }

    #[test]
    fn test_vjoy_enumeration() {
        // Mock: devices 1 and 3 exist, but 3 is busy -> only 1 returned.
        let devices = enumerate_vjoy_devices_impl(
            |id| matches!(id, 1 | 3),
            |id| {
                if id == 1 {
                    1 // FREE
                } else if id == 3 {
                    2 // BUSY
                } else {
                    3 // MISS
                }
            },
        );

        let _: Vec<u32> = devices.clone();
        assert_eq!(devices, vec![1]);
    }
}
