//! TruckPilot native ETS2 telemetry plugin DLL.
//!
//! ## What this does
//! ETS2 loads this DLL from `<ETS2>/bin/win_x64/plugins/truckpilot_telemetry.dll`
//! at startup. The DLL registers SCS SDK channel callbacks, collects truck data
//! every frame, and writes it to a named shared-memory region.
//!
//! ## Shared memory
//! - Windows name: `Local\TruckPilotTelemetry`
//! - Linux path:   `/dev/shm/truckpilot_telemetry` (for testing only)
//! - Layout:       `ShmLayout` (must stay in sync with `crates/telemetry/src/shm.rs`)
//! - Magic:        `0x54504C54` ("TPLT")
//! - Version:      `2`
//!
//! ## Installation
//! 1. Cross-compile for Windows: `cargo build --release --target x86_64-pc-windows-gnu`
//! 2. Copy `truckpilot_telemetry.dll` to `<ETS2>/bin/win_x64/plugins/`
//! 3. Start ETS2 — the DLL is loaded automatically
//! 4. Run `truckpilot-diag` to verify the SHM region is active

// Only compile the real DLL code on Windows.
// On Linux the module compiles as a stub so `cargo test` works.
#![cfg_attr(not(test), cfg(windows))]
#![allow(
    non_camel_case_types,
    non_upper_case_globals,
    clippy::upper_case_acronyms
)]

use std::ffi::c_void;
use std::mem;
use std::ptr;

// ---------------------------------------------------------------------------
// Windows API FFI (kernel32.dll / user32.dll)
// ---------------------------------------------------------------------------

type HANDLE = isize;
type LPVOID = *mut c_void;
type LPCWSTR = *const u16;
type DWORD = u32;
type BOOL = i32;

const INVALID_HANDLE_VALUE: HANDLE = -1isize;
const PAGE_READWRITE: DWORD = 4;
const FILE_MAP_WRITE: DWORD = 2;
const NULL: HANDLE = 0;

extern "system" {
    fn CreateFileMappingW(
        hFile: HANDLE,
        lpAttributes: *const c_void,
        flProtect: DWORD,
        dwMaximumSizeHigh: DWORD,
        dwMaximumSizeLow: DWORD,
        lpName: LPCWSTR,
    ) -> HANDLE;

    fn MapViewOfFile(
        hFileMappingObject: HANDLE,
        dwDesiredAccess: DWORD,
        dwFileOffsetHigh: DWORD,
        dwFileOffsetLow: DWORD,
        dwNumberOfBytesToMap: usize,
    ) -> LPVOID;

    fn UnmapViewOfFile(lpBaseAddress: LPVOID) -> BOOL;
    fn CloseHandle(hObject: HANDLE) -> BOOL;

    fn CreateEventW(
        lpEventAttributes: *const c_void,
        bManualReset: BOOL,
        bInitialState: BOOL,
        lpName: LPCWSTR,
    ) -> HANDLE;

    fn SetEvent(hEvent: HANDLE) -> BOOL;
    fn OutputDebugStringA(lpOutputString: *const u8);
}

// ---------------------------------------------------------------------------
// SCS SDK type aliases
// ---------------------------------------------------------------------------

type scs_u32_t = u32;
type scs_s32_t = i32;
type scs_double_t = f64;
type scs_string_t = *const i8;
type scs_context_t = *mut c_void;
type scs_result_t = scs_s32_t;

const SCS_RESULT_OK: scs_result_t = 0;
const SCS_VALUE_TYPE_bool: scs_u32_t = 1;
const SCS_VALUE_TYPE_float: scs_u32_t = 5;
#[allow(dead_code)]
const SCS_VALUE_TYPE_double: scs_u32_t = 7;
const SCS_VALUE_TYPE_dplacement: scs_u32_t = 11;
const SCS_VALUE_TYPE_fvector: scs_u32_t = 8;
const SCS_CHANNEL_FLAG_none: scs_u32_t = 0;

// SCS frame-end event ID
const SCS_TELEMETRY_EVENT_frame_end: scs_u32_t = 2;

// ---------------------------------------------------------------------------
// Shared memory layout
// MUST stay byte-for-byte identical to `ShmTelemetryLayout` in
// `crates/telemetry/src/shm.rs`.
// ---------------------------------------------------------------------------

/// Magic written at offset 0. Reader checks this before trusting any data.
pub const SHM_MAGIC: u32 = 0x54504C54; // "TPLT"
/// Layout version. Increment when the struct changes.
pub const SHM_VERSION: u32 = 2;
/// Shared memory name (Windows).
const SHM_NAME: &str = "Local\\TruckPilotTelemetry";
/// Ready-event name (Windows) — signalled after every frame write.
const SHM_EVENT_NAME: &str = "Local\\TruckPilotTelemetryReady";

#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct ShmLayout {
    pub magic: u32,
    pub version: u32,
    pub sequence: u32,
    pub _pad: u32,

    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub heading: f64,
    pub pitch: f64,
    pub roll: f64,

    pub speed_ms: f64,
    pub engine_rpm: f64,
    pub nav_speed_limit_kmh: f64,
    pub nav_speed_limit_valid: u32,

    pub fuel_liters: f64,
    pub odometer_km: f64,
    pub cruise_control_speed_kmh: f64,

    pub local_velocity: [f32; 3],
    pub local_acceleration: [f32; 3],
    pub effective_throttle: f32,
    pub distance_to_lead_m: f32,

    // v2 fields
    pub effective_brake: f32,
    pub effective_clutch: f32,
    pub input_steering: f32,
    pub input_throttle: f32,
    pub input_brake: f32,
    pub input_clutch: f32,
    pub engine_gear: i32,
    pub displayed_gear: i32,
    pub hazard_warning: u8,
    pub blinker_left: u8,
    pub blinker_right: u8,
    pub parking_brake: u8,
    pub paused: u8,
    pub _reserved0: [u8; 3],
    pub timestamp_us: u64,
}

// ---------------------------------------------------------------------------
// SCS SDK structs
// ---------------------------------------------------------------------------

#[repr(C)]
#[derive(Clone, Copy)]
struct ScsDPlacement {
    x: scs_double_t,
    y: scs_double_t,
    z: scs_double_t,
    heading: scs_double_t,
    pitch: scs_double_t,
    roll: scs_double_t,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct ScsFVector {
    x: f32,
    y: f32,
    z: f32,
}

#[repr(C)]
union ScsValueUnion {
    value_bool: u8,
    value_float: f32,
    value_double: f64,
    value_dplacement: ScsDPlacement,
    value_fvector: ScsFVector,
}

#[repr(C)]
struct ScsValue {
    value_type: scs_u32_t,
    _padding: scs_u32_t,
    value: ScsValueUnion,
}

type ScsLogFn = unsafe extern "system" fn(scs_s32_t, scs_string_t);
type ScsEventCallback = unsafe extern "system" fn(scs_u32_t, *const c_void, scs_context_t);
type ScsChannelCallback =
    unsafe extern "system" fn(scs_string_t, scs_u32_t, *const ScsValue, scs_context_t);
type ScsRegisterForEventFn =
    unsafe extern "system" fn(scs_u32_t, ScsEventCallback, scs_context_t) -> scs_result_t;
type ScsRegisterForChannelFn = unsafe extern "system" fn(
    scs_string_t,
    scs_u32_t,
    scs_u32_t,
    scs_u32_t,
    ScsChannelCallback,
    scs_context_t,
) -> scs_result_t;

#[repr(C)]
struct ScsSdkInitParamsV100 {
    game_name: scs_string_t,
    game_id: scs_string_t,
    game_version: scs_u32_t,
    _padding: scs_u32_t,
    log: ScsLogFn,
}

#[repr(C)]
pub struct ScsTelemetryInitParamsV100 {
    common: ScsSdkInitParamsV100,
    register_for_event: ScsRegisterForEventFn,
    unregister_from_event: unsafe extern "system" fn(scs_u32_t) -> scs_result_t,
    register_for_channel: ScsRegisterForChannelFn,
    unregister_from_channel: unsafe extern "system" fn(scs_string_t) -> scs_result_t,
}

// ---------------------------------------------------------------------------
// Global frame state (written by callbacks, flushed by frame_end)
// ---------------------------------------------------------------------------

static mut SHM_HANDLE: HANDLE = NULL;
static mut SHM_PTR: *mut ShmLayout = ptr::null_mut();
static mut READY_EVENT: HANDLE = NULL;
static mut SEQUENCE: u32 = 0;

static mut G_X: f64 = 0.0;
static mut G_Y: f64 = 0.0;
static mut G_Z: f64 = 0.0;
static mut G_HEADING: f64 = 0.0;
static mut G_PITCH: f64 = 0.0;
static mut G_ROLL: f64 = 0.0;
static mut G_SPEED: f64 = 0.0;
static mut G_RPM: f64 = 0.0;
static mut G_CRUISE: f64 = 0.0;
static mut G_NAV_LIMIT: f64 = 0.0;
static mut G_NAV_LIMIT_VALID: u32 = 0;
static mut G_FUEL: f64 = 0.0;
static mut G_ODOMETER: f64 = 0.0;
static mut G_VEL_X: f32 = 0.0;
static mut G_VEL_Y: f32 = 0.0;
static mut G_VEL_Z: f32 = 0.0;
static mut G_ACCEL_X: f32 = 0.0;
static mut G_ACCEL_Y: f32 = 0.0;
static mut G_ACCEL_Z: f32 = 0.0;
static mut G_THROTTLE: f32 = 0.0;
static mut G_BRAKE: f32 = 0.0;
static mut G_CLUTCH: f32 = 0.0;
#[allow(dead_code)]
static mut G_STEER: f32 = 0.0;
static mut G_IN_THROTTLE: f32 = 0.0;
static mut G_IN_BRAKE: f32 = 0.0;
static mut G_IN_CLUTCH: f32 = 0.0;
static mut G_IN_STEER: f32 = 0.0;
static mut G_GEAR: i32 = 0;
static mut G_GEAR_DISPLAY: i32 = 0;
static mut G_HAZARD: u8 = 0;
static mut G_BLINKER_L: u8 = 0;
static mut G_BLINKER_R: u8 = 0;
static mut G_PARKING_BRAKE: u8 = 0;
static mut G_PAUSED: u8 = 0;
static mut G_TIMESTAMP_US: u64 = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

unsafe fn debug_log(msg: &str) {
    let s = format!("[TruckPilot] {msg}\0");
    OutputDebugStringA(s.as_ptr());
}

fn wide_str(s: &str) -> Vec<u16> {
    s.encode_utf16().chain(std::iter::once(0)).collect()
}

/// Register one channel callback. Silently ignores registration failures
/// (channel might not exist in this game version).
unsafe fn reg_channel(
    p: &ScsTelemetryInitParamsV100,
    name: &str,
    value_type: scs_u32_t,
    cb: ScsChannelCallback,
) {
    let cname = std::ffi::CString::new(name).unwrap_or_default();
    let _ = (p.register_for_channel)(
        cname.as_ptr(),
        0,
        value_type,
        SCS_CHANNEL_FLAG_none,
        cb,
        ptr::null_mut(),
    );
}

// ---------------------------------------------------------------------------
// DLL entry points (exported to ETS2)
// ---------------------------------------------------------------------------

/// Called by ETS2 when the DLL is loaded. Sets up SHM and registers callbacks.
///
/// # Safety
/// Called by ETS2 with a valid `ScsTelemetryInitParamsV100` pointer. Must not be called from Rust.
#[no_mangle]
pub unsafe extern "system" fn scs_telemetry_init(
    version: scs_u32_t,
    params: *const ScsTelemetryInitParamsV100,
) -> scs_result_t {
    if params.is_null() {
        return -2;
    }
    if version < 0x0001_0000 {
        return -1; // unsupported SDK version
    }

    let p = &*params;
    debug_log("scs_telemetry_init v2 starting");

    // Create the shared memory region.
    let name = wide_str(SHM_NAME);
    let shm_size = mem::size_of::<ShmLayout>();
    SHM_HANDLE = CreateFileMappingW(
        INVALID_HANDLE_VALUE,
        ptr::null(),
        PAGE_READWRITE,
        0,
        shm_size as DWORD,
        name.as_ptr(),
    );
    if SHM_HANDLE == NULL {
        debug_log("CreateFileMappingW failed — SHM unavailable");
        return -7;
    }

    SHM_PTR = MapViewOfFile(SHM_HANDLE, FILE_MAP_WRITE, 0, 0, shm_size) as *mut ShmLayout;
    if SHM_PTR.is_null() {
        debug_log("MapViewOfFile failed");
        CloseHandle(SHM_HANDLE);
        SHM_HANDLE = NULL;
        return -7;
    }

    // Write initial magic so readers know the DLL is alive.
    (*SHM_PTR).magic = SHM_MAGIC;
    (*SHM_PTR).version = SHM_VERSION;
    (*SHM_PTR).sequence = 0;

    // Ready event — signalled after every frame flush.
    let ev_name = wide_str(SHM_EVENT_NAME);
    READY_EVENT = CreateEventW(ptr::null(), 1, 0, ev_name.as_ptr());

    // Register frame-end event → flush all data to SHM.
    let _ = (p.register_for_event)(SCS_TELEMETRY_EVENT_frame_end, frame_end_cb, ptr::null_mut());

    // --- Position & orientation ---
    reg_channel(
        p,
        "truck.world.placement",
        SCS_VALUE_TYPE_dplacement,
        cb_placement,
    );

    // --- Motion ---
    reg_channel(p, "truck.speed", SCS_VALUE_TYPE_float, cb_speed);
    reg_channel(
        p,
        "truck.local.velocity",
        SCS_VALUE_TYPE_fvector,
        cb_local_velocity,
    );
    reg_channel(
        p,
        "truck.local.acceleration",
        SCS_VALUE_TYPE_fvector,
        cb_local_accel,
    );

    // --- Engine ---
    reg_channel(p, "truck.engine.rpm", SCS_VALUE_TYPE_float, cb_rpm);
    reg_channel(p, "truck.engine.gear", SCS_VALUE_TYPE_float, cb_engine_gear);
    reg_channel(
        p,
        "truck.displayed.gear",
        SCS_VALUE_TYPE_float,
        cb_displayed_gear,
    );

    // --- Driver inputs ---
    reg_channel(
        p,
        "truck.effective.throttle",
        SCS_VALUE_TYPE_float,
        cb_eff_throttle,
    );
    reg_channel(
        p,
        "truck.effective.brake",
        SCS_VALUE_TYPE_float,
        cb_eff_brake,
    );
    reg_channel(
        p,
        "truck.effective.clutch",
        SCS_VALUE_TYPE_float,
        cb_eff_clutch,
    );
    reg_channel(p, "truck.input.steering", SCS_VALUE_TYPE_float, cb_in_steer);
    reg_channel(
        p,
        "truck.input.throttle",
        SCS_VALUE_TYPE_float,
        cb_in_throttle,
    );
    reg_channel(p, "truck.input.brake", SCS_VALUE_TYPE_float, cb_in_brake);
    reg_channel(p, "truck.input.clutch", SCS_VALUE_TYPE_float, cb_in_clutch);

    // --- Fuel & navigation ---
    reg_channel(p, "truck.fuel", SCS_VALUE_TYPE_float, cb_fuel);
    reg_channel(p, "truck.odometer", SCS_VALUE_TYPE_float, cb_odometer);
    reg_channel(p, "truck.cruise_control", SCS_VALUE_TYPE_float, cb_cruise);
    reg_channel(
        p,
        "truck.navigation.speed.limit",
        SCS_VALUE_TYPE_float,
        cb_nav_limit,
    );

    // --- Lights & state ---
    reg_channel(
        p,
        "truck.light.blinker.left.active",
        SCS_VALUE_TYPE_bool,
        cb_blinker_l,
    );
    reg_channel(
        p,
        "truck.light.blinker.right.active",
        SCS_VALUE_TYPE_bool,
        cb_blinker_r,
    );
    reg_channel(
        p,
        "truck.light.hazard.warning",
        SCS_VALUE_TYPE_bool,
        cb_hazard,
    );
    reg_channel(
        p,
        "truck.brake.parking",
        SCS_VALUE_TYPE_bool,
        cb_parking_brake,
    );

    debug_log("scs_telemetry_init done — all channels registered");
    SCS_RESULT_OK
}

/// Called by ETS2 when the DLL is unloaded. Releases all resources.
///
/// # Safety
/// Called by ETS2 during shutdown. Must not be called from Rust.
#[no_mangle]
pub unsafe extern "system" fn scs_telemetry_shutdown() {
    debug_log("scs_telemetry_shutdown");
    if !SHM_PTR.is_null() {
        UnmapViewOfFile(SHM_PTR as LPVOID);
        SHM_PTR = ptr::null_mut();
    }
    if SHM_HANDLE != NULL {
        CloseHandle(SHM_HANDLE);
        SHM_HANDLE = NULL;
    }
    if READY_EVENT != NULL {
        CloseHandle(READY_EVENT);
        READY_EVENT = NULL;
    }
}

// ---------------------------------------------------------------------------
// Frame-end callback — atomically flushes all globals to SHM
// ---------------------------------------------------------------------------

unsafe extern "system" fn frame_end_cb(
    _event: scs_u32_t,
    _info: *const c_void,
    _ctx: scs_context_t,
) {
    if SHM_PTR.is_null() {
        return;
    }

    let seq = SEQUENCE.wrapping_add(1);
    SEQUENCE = seq;

    let layout = ShmLayout {
        magic: SHM_MAGIC,
        version: SHM_VERSION,
        sequence: seq,
        _pad: 0,
        x: G_X,
        y: G_Y,
        z: G_Z,
        heading: G_HEADING,
        pitch: G_PITCH,
        roll: G_ROLL,
        speed_ms: G_SPEED,
        engine_rpm: G_RPM,
        nav_speed_limit_kmh: G_NAV_LIMIT,
        nav_speed_limit_valid: G_NAV_LIMIT_VALID,
        fuel_liters: G_FUEL,
        odometer_km: G_ODOMETER,
        cruise_control_speed_kmh: G_CRUISE,
        local_velocity: [G_VEL_X, G_VEL_Y, G_VEL_Z],
        local_acceleration: [G_ACCEL_X, G_ACCEL_Y, G_ACCEL_Z],
        effective_throttle: G_THROTTLE,
        distance_to_lead_m: -1.0, // not available via SCS SDK
        effective_brake: G_BRAKE,
        effective_clutch: G_CLUTCH,
        input_steering: G_IN_STEER,
        input_throttle: G_IN_THROTTLE,
        input_brake: G_IN_BRAKE,
        input_clutch: G_IN_CLUTCH,
        engine_gear: G_GEAR,
        displayed_gear: G_GEAR_DISPLAY,
        hazard_warning: G_HAZARD,
        blinker_left: G_BLINKER_L,
        blinker_right: G_BLINKER_R,
        parking_brake: G_PARKING_BRAKE,
        paused: G_PAUSED,
        _reserved0: [0; 3],
        timestamp_us: G_TIMESTAMP_US,
    };

    // Single atomic write — readers use sequence number to detect torn reads.
    ptr::copy_nonoverlapping(&layout, SHM_PTR, 1);

    if READY_EVENT != NULL {
        SetEvent(READY_EVENT);
    }
}

// ---------------------------------------------------------------------------
// Channel callbacks — one per data field
// ---------------------------------------------------------------------------

macro_rules! float_cb {
    ($name:ident, $global:ident) => {
        unsafe extern "system" fn $name(
            _: scs_string_t,
            _: scs_u32_t,
            value: *const ScsValue,
            _: scs_context_t,
        ) {
            if !value.is_null() && (*value).value_type == SCS_VALUE_TYPE_float {
                $global = (*value).value.value_float;
            }
        }
    };
}

macro_rules! float_cb_f64 {
    ($name:ident, $global:ident) => {
        unsafe extern "system" fn $name(
            _: scs_string_t,
            _: scs_u32_t,
            value: *const ScsValue,
            _: scs_context_t,
        ) {
            if !value.is_null() && (*value).value_type == SCS_VALUE_TYPE_float {
                $global = (*value).value.value_float as f64;
            }
        }
    };
}

macro_rules! bool_cb {
    ($name:ident, $global:ident) => {
        unsafe extern "system" fn $name(
            _: scs_string_t,
            _: scs_u32_t,
            value: *const ScsValue,
            _: scs_context_t,
        ) {
            if !value.is_null() && (*value).value_type == SCS_VALUE_TYPE_bool {
                $global = (*value).value.value_bool;
            }
        }
    };
}

unsafe extern "system" fn cb_placement(
    _: scs_string_t,
    _: scs_u32_t,
    value: *const ScsValue,
    _: scs_context_t,
) {
    if value.is_null() || (*value).value_type != SCS_VALUE_TYPE_dplacement {
        return;
    }
    let dp = (*value).value.value_dplacement;
    G_X = dp.x;
    G_Y = dp.y;
    G_Z = dp.z;
    G_HEADING = dp.heading;
    G_PITCH = dp.pitch;
    G_ROLL = dp.roll;
}

unsafe extern "system" fn cb_local_velocity(
    _: scs_string_t,
    _: scs_u32_t,
    value: *const ScsValue,
    _: scs_context_t,
) {
    if value.is_null() || (*value).value_type != SCS_VALUE_TYPE_fvector {
        return;
    }
    let v = (*value).value.value_fvector;
    G_VEL_X = v.x;
    G_VEL_Y = v.y;
    G_VEL_Z = v.z;
}

unsafe extern "system" fn cb_local_accel(
    _: scs_string_t,
    _: scs_u32_t,
    value: *const ScsValue,
    _: scs_context_t,
) {
    if value.is_null() || (*value).value_type != SCS_VALUE_TYPE_fvector {
        return;
    }
    let v = (*value).value.value_fvector;
    G_ACCEL_X = v.x;
    G_ACCEL_Y = v.y;
    G_ACCEL_Z = v.z;
}

unsafe extern "system" fn cb_nav_limit(
    _: scs_string_t,
    _: scs_u32_t,
    value: *const ScsValue,
    _: scs_context_t,
) {
    if !value.is_null() && (*value).value_type == SCS_VALUE_TYPE_float {
        let kmh = (*value).value.value_float as f64 * 3.6;
        G_NAV_LIMIT = kmh;
        G_NAV_LIMIT_VALID = if kmh > 0.0 { 1 } else { 0 };
    }
}

unsafe extern "system" fn cb_engine_gear(
    _: scs_string_t,
    _: scs_u32_t,
    value: *const ScsValue,
    _: scs_context_t,
) {
    if !value.is_null() && (*value).value_type == SCS_VALUE_TYPE_float {
        G_GEAR = (*value).value.value_float as i32;
    }
}

unsafe extern "system" fn cb_displayed_gear(
    _: scs_string_t,
    _: scs_u32_t,
    value: *const ScsValue,
    _: scs_context_t,
) {
    if !value.is_null() && (*value).value_type == SCS_VALUE_TYPE_float {
        G_GEAR_DISPLAY = (*value).value.value_float as i32;
    }
}

float_cb_f64!(cb_speed, G_SPEED);
float_cb_f64!(cb_rpm, G_RPM);
float_cb_f64!(cb_fuel, G_FUEL);
float_cb_f64!(cb_odometer, G_ODOMETER);
float_cb_f64!(cb_cruise, G_CRUISE);

float_cb!(cb_eff_throttle, G_THROTTLE);
float_cb!(cb_eff_brake, G_BRAKE);
float_cb!(cb_eff_clutch, G_CLUTCH);
float_cb!(cb_in_steer, G_IN_STEER);
float_cb!(cb_in_throttle, G_IN_THROTTLE);
float_cb!(cb_in_brake, G_IN_BRAKE);
float_cb!(cb_in_clutch, G_IN_CLUTCH);

bool_cb!(cb_blinker_l, G_BLINKER_L);
bool_cb!(cb_blinker_r, G_BLINKER_R);
bool_cb!(cb_hazard, G_HAZARD);
bool_cb!(cb_parking_brake, G_PARKING_BRAKE);

// ---------------------------------------------------------------------------
// Tests (run on Linux too — no Win32 calls)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shm_magic_matches_reader() {
        // Must equal SHM_MAGIC in crates/telemetry/src/shm.rs
        assert_eq!(SHM_MAGIC, 0x54504C54);
    }

    #[test]
    fn shm_version_is_2() {
        assert_eq!(SHM_VERSION, 2);
    }

    #[test]
    fn layout_size_is_fixed() {
        // Size must be stable — any change breaks existing SHM readers.
        // If you add fields, update this test AND bump SHM_VERSION.
        let sz = mem::size_of::<ShmLayout>();
        // Exact size check: 4+4+4+4 + 6*8 + 4+8+8+8+8+8 + 3*4+3*4+4+4 + 6*4+2*4 + 4*1+3+8
        // = 16 + 48 + 44 + 24 + 32 + 16 = 180 bytes (packed)
        assert!((160..=256).contains(&sz), "unexpected ShmLayout size: {sz}");
    }

    #[test]
    fn layout_magic_at_offset_zero() {
        assert_eq!(mem::offset_of!(ShmLayout, magic), 0);
    }

    #[test]
    fn layout_version_at_offset_4() {
        assert_eq!(mem::offset_of!(ShmLayout, version), 4);
    }

    #[test]
    fn layout_sequence_at_offset_8() {
        assert_eq!(mem::offset_of!(ShmLayout, sequence), 8);
    }

    #[test]
    fn layout_x_at_offset_16() {
        // After magic(4) + version(4) + sequence(4) + _pad(4) = 16
        assert_eq!(mem::offset_of!(ShmLayout, x), 16);
    }
}
