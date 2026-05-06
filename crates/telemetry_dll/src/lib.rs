//! TruckPilot native ETS2 telemetry plugin DLL.
//!
//! This DLL is loaded by ETS2 as a telemetry plugin. It connects to the
//! official SCS Telemetry SDK, receives live truck data via channel
//! callbacks, and writes it to a named shared memory region.
//!
//! All Windows API calls use direct `extern "system"` FFI — no external
//! crate dependency beyond the standard library.

#![cfg_attr(not(test), cfg(windows))]
#![allow(
    non_camel_case_types,
    non_upper_case_globals,
    unused,
    private_interfaces
)]

use std::ffi::c_void;
use std::mem;
use std::ptr;

// ---------------------------------------------------------------------------
// Windows API FFI (kernel32.dll)
// ---------------------------------------------------------------------------

type HANDLE = isize;
type LPVOID = *mut c_void;
type LPCWSTR = *const u16;
type DWORD = u32;
type BOOL = i32;

const INVALID_HANDLE_VALUE: HANDLE = -1;
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
// C-ABI compatible type aliases matching the SCS SDK.
// ---------------------------------------------------------------------------

type scs_u32_t = u32;
type scs_u64_t = u64;
type scs_s32_t = i32;
type scs_double_t = f64;
type scs_string_t = *const i8;
type scs_context_t = *mut c_void;
type scs_result_t = scs_s32_t;

const SCS_RESULT_OK: scs_result_t = 0;
const SCS_VALUE_TYPE_dplacement: scs_u32_t = 11;
const SCS_VALUE_TYPE_float: scs_u32_t = 5;
const SCS_CHANNEL_FLAG_none: scs_u32_t = 0;

// ---------------------------------------------------------------------------
// Shared memory layout — MUST match ShmTelemetryLayout in shm_telemetry.rs
// ---------------------------------------------------------------------------

const SHM_MAGIC: u32 = 0x54505054;
const SHM_VERSION: u32 = 2;

#[repr(C, packed)]
#[derive(Clone, Copy)]
struct ShmLayout {
    magic: u32,
    version: u32,
    sequence: u32,
    _pad: u32,
    x: f64,
    y: f64,
    z: f64,
    heading: f64,
    pitch: f64,
    roll: f64,
    speed_ms: f64,
    engine_rpm: f64,
    nav_speed_limit_kmh: f64,
    nav_speed_limit_valid: u32,
    fuel_liters: f64,
    odometer_km: f64,
    cruise_control_speed_kmh: f64,
    local_velocity: [f32; 3],
    local_acceleration: [f32; 3],
    effective_throttle: f32,
    distance_to_lead_m: f32,
    effective_brake: f32,
    effective_clutch: f32,
    input_steering: f32,
    input_throttle: f32,
    input_brake: f32,
    input_clutch: f32,
    engine_gear: i32,
    displayed_gear: i32,
    hazard_warning: u8,
    blinker_left: u8,
    blinker_right: u8,
    parking_brake: u8,
    paused: u8,
    reserved0: [u8; 3],
    timestamp_us: u64,
    game_id: [u8; 16],
    game_version: u32,
    reserved1: u32,
    reserved: [u8; 512 - 220],
}

// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------

static mut SHM_HANDLE: HANDLE = NULL;
static mut SHM_PTR: *mut ShmLayout = ptr::null_mut();
static mut READY_EVENT: HANDLE = NULL;
static mut SEQUENCE: u32 = 0;

static mut FRAME_POS_X: f64 = 0.0;
static mut FRAME_POS_Y: f64 = 0.0;
static mut FRAME_POS_Z: f64 = 0.0;
static mut FRAME_HEADING: f64 = 0.0;
static mut FRAME_PITCH: f64 = 0.0;
static mut FRAME_ROLL: f64 = 0.0;
static mut FRAME_SPEED: f64 = 0.0;
static mut FRAME_RPM: f64 = 0.0;
static mut FRAME_CRUISE: f64 = 0.0;

// ---------------------------------------------------------------------------
// SCS SDK types
// ---------------------------------------------------------------------------

type ScsLogFn = unsafe extern "system" fn(scs_s32_t, scs_string_t);

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
union ScsValueUnion {
    value_float: f32,
    value_double: f64,
    value_dplacement: ScsDPlacement,
}

#[repr(C)]
struct ScsValue {
    value_type: scs_u32_t,
    _padding: scs_u32_t,
    value: ScsValueUnion,
}

// Event callback: void(event, event_info, context)
type ScsEventCallback = unsafe extern "system" fn(scs_u32_t, *const c_void, scs_context_t);

// Channel callback: void(name, index, value, context)
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
struct ScsTelemetryInitParamsV100 {
    common: ScsSdkInitParamsV100,
    register_for_event: ScsRegisterForEventFn,
    unregister_from_event: unsafe extern "system" fn(scs_u32_t) -> scs_result_t,
    register_for_channel: ScsRegisterForChannelFn,
    unregister_from_channel: unsafe extern "system" fn(scs_string_t) -> scs_result_t,
}

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

// ---------------------------------------------------------------------------
// DLL Exports
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "system" fn scs_telemetry_init(
    version: scs_u32_t,
    params: *const ScsTelemetryInitParamsV100,
) -> scs_result_t {
    if params.is_null() {
        return -2;
    }
    if version < 0x0001_0000 {
        return -1;
    }

    let p = &*params;
    debug_log("scs_telemetry_init called");

    // Create shared memory.
    let name = wide_str("Local\\TruckPilotTelemetry");
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
        debug_log("CreateFileMappingW failed");
        return -7;
    }

    SHM_PTR = MapViewOfFile(SHM_HANDLE, FILE_MAP_WRITE, 0, 0, shm_size) as *mut ShmLayout;
    if SHM_PTR.is_null() {
        debug_log("MapViewOfFile failed");
        CloseHandle(SHM_HANDLE);
        SHM_HANDLE = NULL;
        return -7;
    }

    (*SHM_PTR).magic = SHM_MAGIC;
    (*SHM_PTR).version = SHM_VERSION;
    (*SHM_PTR).sequence = 0;

    // Ready event.
    let ev_name = wide_str("Local\\TruckPilotTelemetryReady");
    READY_EVENT = CreateEventW(ptr::null(), 1, 0, ev_name.as_ptr());

    // Register for frame_end → flush telemetry.
    let _ = (p.register_for_event)(2, frame_end_callback, ptr::null_mut());

    // Register channel callbacks.
    fn reg(p: &ScsTelemetryInitParamsV100, name: &str, vt: scs_u32_t, cb: ScsChannelCallback) {
        let cname = std::ffi::CString::new(name).unwrap();
        unsafe {
            let _ = (p.register_for_channel)(
                cname.as_ptr(),
                0,
                vt,
                SCS_CHANNEL_FLAG_none,
                cb,
                ptr::null_mut(),
            );
        }
    }
    reg(
        p,
        "truck.world.placement",
        SCS_VALUE_TYPE_dplacement,
        placement_callback,
    );
    reg(p, "truck.speed", SCS_VALUE_TYPE_float, speed_callback);
    reg(p, "truck.engine.rpm", SCS_VALUE_TYPE_float, rpm_callback);
    reg(
        p,
        "truck.cruise_control",
        SCS_VALUE_TYPE_float,
        cruise_callback,
    );

    debug_log("scs_telemetry_init done");
    SCS_RESULT_OK
}

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
// Callbacks
// ---------------------------------------------------------------------------

unsafe extern "system" fn frame_end_callback(
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
        x: FRAME_POS_X,
        y: FRAME_POS_Y,
        z: FRAME_POS_Z,
        heading: FRAME_HEADING,
        pitch: FRAME_PITCH,
        roll: FRAME_ROLL,
        speed_ms: FRAME_SPEED,
        engine_rpm: FRAME_RPM,
        nav_speed_limit_kmh: 0.0,
        nav_speed_limit_valid: 0,
        fuel_liters: 0.0,
        odometer_km: 0.0,
        cruise_control_speed_kmh: FRAME_CRUISE,
        local_velocity: [0.0, 0.0, 0.0],
        local_acceleration: [0.0, 0.0, 0.0],
        effective_throttle: 0.0,
        distance_to_lead_m: -1.0,
        effective_brake: 0.0,
        effective_clutch: 0.0,
        input_steering: 0.0,
        input_throttle: 0.0,
        input_brake: 0.0,
        input_clutch: 0.0,
        engine_gear: 0,
        displayed_gear: 0,
        hazard_warning: 0,
        blinker_left: 0,
        blinker_right: 0,
        parking_brake: 0,
        paused: 0,
        reserved0: [0; 3],
        timestamp_us: 0,
        game_id: [0; 16],
        game_version: 0,
        reserved1: 0,
        reserved: [0; 512 - 220],
    };
    ptr::copy_nonoverlapping(&layout, SHM_PTR, 1);
    if READY_EVENT != NULL {
        SetEvent(READY_EVENT);
    }
}

unsafe extern "system" fn placement_callback(
    _name: scs_string_t,
    _index: scs_u32_t,
    value: *const ScsValue,
    _ctx: scs_context_t,
) {
    if value.is_null() {
        return;
    }
    if (*value).value_type == SCS_VALUE_TYPE_dplacement {
        let dp = &(*value).value.value_dplacement;
        FRAME_POS_X = dp.x;
        FRAME_POS_Y = dp.y;
        FRAME_POS_Z = dp.z;
        FRAME_HEADING = dp.heading;
        FRAME_PITCH = dp.pitch;
        FRAME_ROLL = dp.roll;
    }
}

unsafe extern "system" fn speed_callback(
    _name: scs_string_t,
    _index: scs_u32_t,
    value: *const ScsValue,
    _ctx: scs_context_t,
) {
    if !value.is_null() {
        FRAME_SPEED = (*value).value.value_float as f64;
    }
}

unsafe extern "system" fn rpm_callback(
    _name: scs_string_t,
    _index: scs_u32_t,
    value: *const ScsValue,
    _ctx: scs_context_t,
) {
    if !value.is_null() {
        FRAME_RPM = (*value).value.value_float as f64;
    }
}

unsafe extern "system" fn cruise_callback(
    _name: scs_string_t,
    _index: scs_u32_t,
    value: *const ScsValue,
    _ctx: scs_context_t,
) {
    if !value.is_null() {
        FRAME_CRUISE = (*value).value.value_float as f64 * 3.6;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_layout_matches_reader() {
        let sz = mem::size_of::<ShmLayout>();
        assert!(sz >= 64, "DLL layout too small: {sz}");
        assert!(sz <= 512, "DLL layout too large: {sz}");
        assert_eq!(mem::offset_of!(ShmLayout, magic), 0);
    }

    #[test]
    fn test_shm_magic_constant() {
        assert_eq!(SHM_MAGIC, 0x54505054);
        assert_eq!(SHM_VERSION, 2);
    }

    #[test]
    fn test_layout_size_exact() {
        let sz = mem::size_of::<ShmLayout>();
        assert_eq!(sz, 512, "DLL layout size mismatch: expected 512, got {sz}");
    }
}
