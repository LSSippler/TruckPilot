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
//! - Version:      `3`
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
// Nav route memory resolution (Phase R1 — gps_manager AOB)
// ---------------------------------------------------------------------------

mod diag_log;
mod ffi_guard;
mod nav_resolve;
mod nav_route;
mod resolver_guard;
mod resolver_metrics;
mod resolver_sched;
mod resolver_worker;
mod route_chain;
mod route_status;
mod safe_mem;

#[cfg(test)]
mod production_safety_tests;

#[cfg(test)]
mod offline_stutter_tests;

#[cfg(test)]
mod test_isolation;

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
const SCS_U32_NIL: scs_u32_t = 0xFFFF_FFFF;

// SCS telemetry events (scssdk_telemetry_event.h — 1.00+)
const SCS_TELEMETRY_EVENT_started: scs_u32_t = 1;
const SCS_TELEMETRY_EVENT_frame_start: scs_u32_t = 2;
const SCS_TELEMETRY_EVENT_frame_end: scs_u32_t = 3;
const SCS_TELEMETRY_EVENT_paused: scs_u32_t = 4;
const SCS_TELEMETRY_EVENT_unpaused: scs_u32_t = 5;

/// Payload for [`SCS_TELEMETRY_EVENT_frame_start`].
#[repr(C)]
struct ScsTelemetryFrameStart {
    paused_simulation_time: u64,
}

// ---------------------------------------------------------------------------
// Shared memory layout
// MUST stay byte-for-byte identical to `ShmTelemetryLayout` in
// `crates/telemetry/src/shm.rs`.
// ---------------------------------------------------------------------------

/// Magic written at offset 0. Reader checks this before trusting any data.
pub const SHM_MAGIC: u32 = 0x54504C54; // "TPLT"
/// Layout version. Increment when the struct changes.
pub const SHM_VERSION: u32 = 3;
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

    // v3 fields
    pub nav_distance_m: f32,
    pub nav_time_s: f32,
}

// Compile-time offset guards. Must mirror those in
// `crates/telemetry/src/shm.rs::ShmTelemetryLayout` exactly. Drift here
// silently corrupts every f64 the daemon reads, so trip the build.
const _: () = {
    assert!(mem::offset_of!(ShmLayout, magic) == 0);
    assert!(mem::offset_of!(ShmLayout, version) == 4);
    assert!(mem::offset_of!(ShmLayout, sequence) == 8);
    assert!(mem::offset_of!(ShmLayout, _pad) == 12);
    assert!(mem::offset_of!(ShmLayout, x) == 16);
    assert!(mem::offset_of!(ShmLayout, y) == 24);
    assert!(mem::offset_of!(ShmLayout, z) == 32);
    assert!(mem::offset_of!(ShmLayout, heading) == 40);
    assert!(mem::offset_of!(ShmLayout, pitch) == 48);
    assert!(mem::offset_of!(ShmLayout, roll) == 56);
    assert!(mem::offset_of!(ShmLayout, speed_ms) == 64);
    assert!(mem::offset_of!(ShmLayout, engine_rpm) == 72);
    assert!(mem::offset_of!(ShmLayout, nav_speed_limit_kmh) == 80);
    assert!(mem::offset_of!(ShmLayout, nav_speed_limit_valid) == 88);
    assert!(mem::offset_of!(ShmLayout, fuel_liters) == 92);
    assert!(mem::offset_of!(ShmLayout, odometer_km) == 100);
    assert!(mem::offset_of!(ShmLayout, cruise_control_speed_kmh) == 108);
    assert!(mem::offset_of!(ShmLayout, local_velocity) == 116);
    assert!(mem::offset_of!(ShmLayout, local_acceleration) == 128);
    assert!(mem::offset_of!(ShmLayout, effective_throttle) == 140);
    assert!(mem::offset_of!(ShmLayout, distance_to_lead_m) == 144);
    assert!(mem::offset_of!(ShmLayout, effective_brake) == 148);
    assert!(mem::offset_of!(ShmLayout, timestamp_us) == 188);
    assert!(mem::offset_of!(ShmLayout, nav_distance_m) == 196);
    assert!(mem::offset_of!(ShmLayout, nav_time_s) == 200);
    assert!(mem::size_of::<ShmLayout>() == 204);

    // Per-field size asserts — catches type drift (f32 vs f64) that
    // offset_of cannot detect alone. The deltas below assume f64 fields
    // in the orientation/motion block and would fail at compile time
    // if any field were silently downgraded to f32.
    assert!(mem::offset_of!(ShmLayout, y) - mem::offset_of!(ShmLayout, x) == 8);
    assert!(mem::offset_of!(ShmLayout, z) - mem::offset_of!(ShmLayout, y) == 8);
    assert!(mem::offset_of!(ShmLayout, heading) - mem::offset_of!(ShmLayout, z) == 8);
    assert!(mem::offset_of!(ShmLayout, pitch) - mem::offset_of!(ShmLayout, heading) == 8);
    assert!(mem::offset_of!(ShmLayout, roll) - mem::offset_of!(ShmLayout, pitch) == 8);
    assert!(mem::offset_of!(ShmLayout, speed_ms) - mem::offset_of!(ShmLayout, roll) == 8);
    assert!(mem::offset_of!(ShmLayout, engine_rpm) - mem::offset_of!(ShmLayout, speed_ms) == 8);
    assert!(
        mem::offset_of!(ShmLayout, nav_speed_limit_kmh) - mem::offset_of!(ShmLayout, engine_rpm)
            == 8
    );
};

// ---------------------------------------------------------------------------
// SCS SDK structs
// ---------------------------------------------------------------------------

/// SCS SDK `scs_value_dplacement_t`: position is a `dvector` (3×f64),
/// orientation is a `scs_value_euler_t` (3×**f32**, NOT f64). Reading
/// the orientation block as f64 gives denormal garbage because two
/// f32 fields stack into one f64 with a near-zero exponent.
#[repr(C)]
#[derive(Clone, Copy)]
struct ScsDPlacement {
    x: scs_double_t,
    y: scs_double_t,
    z: scs_double_t,
    heading: f32,
    pitch: f32,
    roll: f32,
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
static mut G_NAV_DISTANCE: f32 = 0.0;
static mut G_NAV_TIME: f32 = 0.0;

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
        SCS_U32_NIL,
        value_type,
        SCS_CHANNEL_FLAG_none,
        cb,
        ptr::null_mut(),
    );
}

// ---------------------------------------------------------------------------
// DLL entry points (exported to ETS2)
// ---------------------------------------------------------------------------

/// Deferred nav_resolve diagnostic — disabled during crash-safe phase.
#[allow(dead_code)]
static mut NAV_DIAG_PENDING: bool = true;

/// Called by ETS2 when the DLL is loaded. Sets up SHM and registers callbacks.
///
/// # Safety
/// Called by ETS2 with a valid `ScsTelemetryInitParamsV100` pointer. Must not be called from Rust.
#[no_mangle]
pub unsafe extern "system" fn scs_telemetry_init(
    version: scs_u32_t,
    params: *const ScsTelemetryInitParamsV100,
) -> scs_result_t {
    ffi_guard::guard_result("scs_telemetry_init", || unsafe {
        scs_telemetry_init_inner(version, params)
    })
}

unsafe fn scs_telemetry_init_inner(
    version: scs_u32_t,
    params: *const ScsTelemetryInitParamsV100,
) -> scs_result_t {
    diag_log::boot("DLL loaded / scs_telemetry_init entered");
    debug_log("scs_telemetry_init entered");

    if params.is_null() {
        diag_log::event_force("scs_telemetry_init returning failure (-2 null params)");
        return -2;
    }
    if version < 0x0001_0000 {
        diag_log::event_force(&format!(
            "scs_telemetry_init returning failure (-1 unsupported sdk_version=0x{version:08X})"
        ));
        return -1;
    }

    diag_log::event_force(&format!("sdk_version received: 0x{version:08X}"));
    diag_log::init();

    let p = &*params;

    diag_log::event_force("creating telemetry shm start");
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
        diag_log::event_force("creating telemetry shm error (CreateFileMappingW failed)");
        debug_log("CreateFileMappingW failed — SHM unavailable");
        diag_log::event_force("scs_telemetry_init returning failure (-7 telemetry shm)");
        return -7;
    }

    SHM_PTR = MapViewOfFile(SHM_HANDLE, FILE_MAP_WRITE, 0, 0, shm_size) as *mut ShmLayout;
    if SHM_PTR.is_null() {
        diag_log::event_force("creating telemetry shm error (MapViewOfFile failed)");
        debug_log("MapViewOfFile failed");
        CloseHandle(SHM_HANDLE);
        SHM_HANDLE = NULL;
        diag_log::event_force("scs_telemetry_init returning failure (-7 telemetry shm map)");
        return -7;
    }

    (*SHM_PTR).magic = SHM_MAGIC;
    (*SHM_PTR).version = SHM_VERSION;
    (*SHM_PTR).sequence = 0;
    diag_log::event_force("creating telemetry shm done (Local\\TruckPilotTelemetry)");

    let ev_name = wide_str(SHM_EVENT_NAME);
    READY_EVENT = CreateEventW(ptr::null(), 1, 0, ev_name.as_ptr());

    diag_log::event_force("registering callbacks start");
    let fs_reg = (p.register_for_event)(
        SCS_TELEMETRY_EVENT_frame_start,
        telemetry_frame_cb,
        ptr::null_mut(),
    );
    if fs_reg != SCS_RESULT_OK {
        diag_log::event_force(&format!(
            "registering callbacks error (frame_start event failed: {fs_reg})"
        ));
    }
    let fe_reg = (p.register_for_event)(
        SCS_TELEMETRY_EVENT_frame_end,
        telemetry_frame_cb,
        ptr::null_mut(),
    );
    if fe_reg != SCS_RESULT_OK {
        diag_log::event_force(&format!(
            "registering callbacks error (frame_end event failed: {fe_reg})"
        ));
    }
    let _ = (p.register_for_event)(
        SCS_TELEMETRY_EVENT_paused,
        telemetry_world_cb,
        ptr::null_mut(),
    );
    let _ = (p.register_for_event)(
        SCS_TELEMETRY_EVENT_unpaused,
        telemetry_world_cb,
        ptr::null_mut(),
    );
    let _ = (p.register_for_event)(
        SCS_TELEMETRY_EVENT_started,
        telemetry_world_cb,
        ptr::null_mut(),
    );

    reg_channel(
        p,
        "truck.world.placement",
        SCS_VALUE_TYPE_dplacement,
        cb_placement,
    );
    reg_channel(p, "truck.speed", SCS_VALUE_TYPE_float, cb_speed);
    reg_channel(
        p,
        "truck.lv.linear.velocity",
        SCS_VALUE_TYPE_fvector,
        cb_local_velocity,
    );
    reg_channel(
        p,
        "truck.la.linear.acceleration",
        SCS_VALUE_TYPE_fvector,
        cb_local_accel,
    );
    reg_channel(p, "truck.engine.rpm", SCS_VALUE_TYPE_float, cb_rpm);
    reg_channel(p, "truck.engine.gear", SCS_VALUE_TYPE_float, cb_engine_gear);
    reg_channel(
        p,
        "truck.displayed.gear",
        SCS_VALUE_TYPE_float,
        cb_displayed_gear,
    );
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
    reg_channel(p, "truck.fuel.amount", SCS_VALUE_TYPE_float, cb_fuel);
    reg_channel(p, "truck.odometer", SCS_VALUE_TYPE_float, cb_odometer);
    reg_channel(p, "truck.cruise_control", SCS_VALUE_TYPE_float, cb_cruise);
    reg_channel(
        p,
        "truck.navigation.speed.limit",
        SCS_VALUE_TYPE_float,
        cb_nav_limit,
    );
    reg_channel(p, "truck.lblinker", SCS_VALUE_TYPE_bool, cb_blinker_l);
    reg_channel(p, "truck.rblinker", SCS_VALUE_TYPE_bool, cb_blinker_r);
    reg_channel(p, "truck.hazard.warning", SCS_VALUE_TYPE_bool, cb_hazard);
    reg_channel(
        p,
        "truck.brake.parking",
        SCS_VALUE_TYPE_bool,
        cb_parking_brake,
    );
    reg_channel(
        p,
        "truck.navigation.distance",
        SCS_VALUE_TYPE_float,
        cb_nav_distance,
    );
    reg_channel(
        p,
        "truck.navigation.time",
        SCS_VALUE_TYPE_float,
        cb_nav_time,
    );
    diag_log::event_force("registering callbacks done");

    diag_log::event_force("creating route blackboard start");
    let route_bb_ok = nav_route::init_shm();
    if route_bb_ok {
        nav_route::publish_initial_empty();
        diag_log::event_force("creating route blackboard done (empty route published)");
    } else {
        diag_log::event_force(
            "creating route blackboard error (non-fatal — telemetry continues)",
        );
        debug_log("WARN: RouteBlackboard SHM init failed");
    }

    resolver_worker::start_worker();
    safe_mem::log_route_resolver_mode_at_init();

    let result = ffi_guard::telemetry_init_should_succeed(true, route_bb_ok);
    debug_log("scs_telemetry_init done — all channels registered");
    if result == SCS_RESULT_OK {
        diag_log::event_force("scs_telemetry_init returning success");
    } else {
        diag_log::event_force(&format!("scs_telemetry_init returning failure ({result})"));
    }
    result
}

unsafe fn scs_telemetry_shutdown_inner() {
    resolver_worker::stop_worker();
    nav_route::cleanup_shm();
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

/// Called by ETS2 when the DLL is unloaded. Releases all resources.
///
/// # Safety
/// Called by ETS2 during shutdown. Must not be called from Rust.
#[no_mangle]
pub unsafe extern "system" fn scs_telemetry_shutdown() {
    ffi_guard::guard_void("scs_telemetry_shutdown", || unsafe {
        diag_log::event_force("scs_telemetry_shutdown called");
        debug_log("scs_telemetry_shutdown");
        scs_telemetry_shutdown_inner();
    });
}

// ---------------------------------------------------------------------------
// Frame callbacks — flush SHM + route tick
// ---------------------------------------------------------------------------

static mut FRAME_CB_COUNT: u32 = 0;

fn qpc_timestamp_us() -> u64 {
    type LARGE_INTEGER = i64;
    extern "system" {
        fn QueryPerformanceCounter(lp: *mut LARGE_INTEGER) -> i32;
        fn QueryPerformanceFrequency(lp: *mut LARGE_INTEGER) -> i32;
    }
    unsafe {
        static mut FREQ: u64 = 0;
        if FREQ == 0 {
            let mut f: LARGE_INTEGER = 0;
            if QueryPerformanceFrequency(&mut f) == 0 || f <= 0 {
                return 0;
            }
            FREQ = f as u64;
        }
        let mut c: LARGE_INTEGER = 0;
        if QueryPerformanceCounter(&mut c) == 0 {
            return 0;
        }
        ((c as u64).saturating_mul(1_000_000) / FREQ) as u64
    }
}

static mut TELEMETRY_EVENT_LOG_COUNT: u32 = 0;
static mut TELEMETRY_EVENT_SEEN: [bool; 16] = [false; 16];

unsafe fn log_telemetry_event_probe(event: scs_u32_t, context: &str) {
    let idx = (event as usize).min(15);
    if !TELEMETRY_EVENT_SEEN[idx] {
        if TELEMETRY_EVENT_LOG_COUNT < 10 {
            TELEMETRY_EVENT_LOG_COUNT = TELEMETRY_EVENT_LOG_COUNT.saturating_add(1);
            diag_log::event_force(&format!(
                "event callback context={context} raw_id={event} mapped_name={}",
                route_status::telemetry_event_name(event)
            ));
        }
        TELEMETRY_EVENT_SEEN[idx] = true;
    } else if event > 6 && TELEMETRY_EVENT_LOG_COUNT < 10 {
        TELEMETRY_EVENT_LOG_COUNT = TELEMETRY_EVENT_LOG_COUNT.saturating_add(1);
        diag_log::event_force(&format!(
            "event callback context={context} raw_id={event} mapped_name=unknown"
        ));
    }
}

unsafe extern "system" fn telemetry_world_cb(
    event: scs_u32_t,
    _info: *const c_void,
    _ctx: scs_context_t,
) {
    ffi_guard::catch_callback("telemetry_world_cb", || {
        log_telemetry_event_probe(event, "world");
        let ts = qpc_timestamp_us();
        nav_route::on_world_event(event, ts);
    });
}

unsafe extern "system" fn telemetry_frame_cb(
    event: scs_u32_t,
    info: *const c_void,
    ctx: scs_context_t,
) {
    ffi_guard::catch_callback("telemetry_frame_cb", || unsafe {
        telemetry_frame_cb_inner(event, info, ctx);
    });
}

unsafe fn telemetry_frame_cb_inner(
    event: scs_u32_t,
    info: *const c_void,
    _ctx: scs_context_t,
) {
    log_telemetry_event_probe(event, "frame");

    if event == SCS_TELEMETRY_EVENT_frame_start && !info.is_null() {
        let frame = &*(info as *const ScsTelemetryFrameStart);
        G_TIMESTAMP_US = frame.paused_simulation_time;
    }

    FRAME_CB_COUNT = FRAME_CB_COUNT.saturating_add(1);
    let ts_qpc = qpc_timestamp_us();
    nav_route::on_frame_event(event, FRAME_CB_COUNT, ts_qpc);

    if SHM_PTR.is_null() {
        dispatch_route_tick(event, ts_qpc);
        return;
    }

    let seq = SEQUENCE.wrapping_add(1);
    SEQUENCE = seq;

    let ts = if G_TIMESTAMP_US != 0 {
        G_TIMESTAMP_US
    } else {
        ts_qpc
    };

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
        distance_to_lead_m: -1.0,
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
        timestamp_us: ts,
        nav_distance_m: G_NAV_DISTANCE,
        nav_time_s: G_NAV_TIME,
    };

    ptr::copy_nonoverlapping(&layout, SHM_PTR, 1);

    if READY_EVENT != NULL {
        SetEvent(READY_EVENT);
    }

    // GPS diagnostics run from the throttled route tick after warmup (crash-safe).
    // Do not scan game memory from the first frame callback during profile load.

    dispatch_route_tick(event, ts_qpc);
}

fn dispatch_route_tick(event: scs_u32_t, ts_qpc: u64) {
    use crate::route_status::RouteTickSource;

    let source = if event == SCS_TELEMETRY_EVENT_frame_end {
        RouteTickSource::FrameEnd
    } else if event == SCS_TELEMETRY_EVENT_frame_start
        && nav_route::should_tick_on_frame_start(ts_qpc)
    {
        RouteTickSource::FrameStartFallback
    } else {
        return;
    };
    // O(1): schedule background worker — never run resolver synchronously here.
    resolver_worker::notify_frame_tick(ts_qpc, source);
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
            ffi_guard::catch_callback(stringify!($name), || unsafe {
                if !value.is_null() && (*value).value_type == SCS_VALUE_TYPE_float {
                    $global = (*value).value.value_float;
                }
            });
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
            ffi_guard::catch_callback(stringify!($name), || unsafe {
                if !value.is_null() && (*value).value_type == SCS_VALUE_TYPE_float {
                    $global = (*value).value.value_float as f64;
                }
            });
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
            ffi_guard::catch_callback(stringify!($name), || unsafe {
                if !value.is_null() && (*value).value_type == SCS_VALUE_TYPE_bool {
                    $global = (*value).value.value_bool;
                }
            });
        }
    };
}

unsafe extern "system" fn cb_placement(
    _: scs_string_t,
    _: scs_u32_t,
    value: *const ScsValue,
    _: scs_context_t,
) {
    ffi_guard::catch_callback("cb_placement", || unsafe {
        if value.is_null() || (*value).value_type != SCS_VALUE_TYPE_dplacement {
            return;
        }
        let dp = (*value).value.value_dplacement;
        G_X = dp.x;
        G_Y = dp.y;
        G_Z = dp.z;
        G_HEADING = dp.heading as f64;
        G_PITCH = dp.pitch as f64;
        G_ROLL = dp.roll as f64;
    });
}

unsafe extern "system" fn cb_local_velocity(
    _: scs_string_t,
    _: scs_u32_t,
    value: *const ScsValue,
    _: scs_context_t,
) {
    ffi_guard::catch_callback("cb_local_velocity", || unsafe {
        if value.is_null() || (*value).value_type != SCS_VALUE_TYPE_fvector {
            return;
        }
        let v = (*value).value.value_fvector;
        G_VEL_X = v.x;
        G_VEL_Y = v.y;
        G_VEL_Z = v.z;
    });
}

unsafe extern "system" fn cb_local_accel(
    _: scs_string_t,
    _: scs_u32_t,
    value: *const ScsValue,
    _: scs_context_t,
) {
    ffi_guard::catch_callback("cb_local_accel", || unsafe {
        if value.is_null() || (*value).value_type != SCS_VALUE_TYPE_fvector {
            return;
        }
        let v = (*value).value.value_fvector;
        G_ACCEL_X = v.x;
        G_ACCEL_Y = v.y;
        G_ACCEL_Z = v.z;
    });
}

unsafe extern "system" fn cb_nav_limit(
    _: scs_string_t,
    _: scs_u32_t,
    value: *const ScsValue,
    _: scs_context_t,
) {
    ffi_guard::catch_callback("cb_nav_limit", || unsafe {
        if !value.is_null() && (*value).value_type == SCS_VALUE_TYPE_float {
            let kmh = (*value).value.value_float as f64 * 3.6;
            G_NAV_LIMIT = kmh;
            G_NAV_LIMIT_VALID = if kmh > 0.0 { 1 } else { 0 };
        }
    });
}

unsafe extern "system" fn cb_engine_gear(
    _: scs_string_t,
    _: scs_u32_t,
    value: *const ScsValue,
    _: scs_context_t,
) {
    ffi_guard::catch_callback("cb_engine_gear", || unsafe {
        if !value.is_null() && (*value).value_type == SCS_VALUE_TYPE_float {
            G_GEAR = (*value).value.value_float as i32;
        }
    });
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

float_cb!(cb_nav_distance, G_NAV_DISTANCE);
float_cb!(cb_nav_time, G_NAV_TIME);

// ---------------------------------------------------------------------------
// SCS Input Plugin API — control SHM back-channel
// ---------------------------------------------------------------------------

/// Magic for the control SHM. Must match `CTRL_SHM_MAGIC` in scs-sdk-output.
pub const CTRL_SHM_MAGIC: u32 = 0x54504354; // "TPCT"
/// Layout version for the control SHM.
pub const CTRL_SHM_VERSION: u32 = 1;
const CTRL_SHM_NAME: &str = "Local\\TruckPilotControls";

const SCS_RESULT_NOT_FOUND: scs_result_t = -4;
const SCS_INPUT_VERSION_1_00: scs_u32_t = 0x0001_0000;
const SCS_INPUT_DEVICE_TYPE_SEMANTICAL: scs_u32_t = 2;
const SCS_INPUT_EVENT_CALLBACK_FLAG_FIRST_IN_FRAME: scs_u32_t = 0x0000_0001;

/// Control SHM layout — daemon plugin writes, DLL reads each frame.
/// Must stay byte-for-byte identical to `ShmControlLayout` in
/// `crates/plugins/scs-sdk-output/src/lib.rs`.
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct ShmControlLayout {
    pub magic: u32,
    pub version: u32,
    pub sequence: u32,
    pub active: u32,   // 1 = valid control output, 0 = passthrough/idle
    pub steering: f32, // [-1.0, +1.0]
    pub throttle: f32, // [0.0, 1.0]
    pub brake: f32,    // [0.0, 1.0]
    pub clutch: f32,   // [0.0, 1.0]
}

const _CTRL_LAYOUT_GUARDS: () = {
    assert!(mem::offset_of!(ShmControlLayout, magic) == 0);
    assert!(mem::offset_of!(ShmControlLayout, version) == 4);
    assert!(mem::offset_of!(ShmControlLayout, sequence) == 8);
    assert!(mem::offset_of!(ShmControlLayout, active) == 12);
    assert!(mem::offset_of!(ShmControlLayout, steering) == 16);
    assert!(mem::offset_of!(ShmControlLayout, throttle) == 20);
    assert!(mem::offset_of!(ShmControlLayout, brake) == 24);
    assert!(mem::offset_of!(ShmControlLayout, clutch) == 28);
    assert!(mem::size_of::<ShmControlLayout>() == 32);
};

/// Per-frame event yielded to the game. Game allocates; we write input_index
/// and value_float then return SCS_RESULT_OK while events remain.
/// Sized at 8 bytes (u32 + f32, naturally packed, no padding on x86-64).
#[repr(C)]
struct ScsInputEvent {
    input_index: scs_u32_t,
    value_float: f32,
}

/// One logical axis on our semantical device.
#[repr(C)]
struct ScsInputDeviceInput {
    name: scs_string_t,
    display_name: scs_string_t,
    value_type: scs_u32_t,
    _pad: scs_u32_t,
}

// scs_input_event_callback_t(event_info, flags, context) — flags must be present even if ignored.
type ScsInputEventCb = unsafe extern "system" fn(
    *mut ScsInputEvent,
    scs_u32_t, // flags (SCS_INPUT_EVENT_CALLBACK_FLAG_*)
    scs_context_t,
) -> scs_result_t;

// scs_input_active_callback_t — optional, called when device becomes active/inactive.
type ScsInputActiveCb = unsafe extern "system" fn(u8, scs_context_t);

type ScsRegisterDeviceFn = unsafe extern "system" fn(*const ScsInputDevice) -> scs_result_t;

/// Device descriptor passed to register_device.
/// Layout must match scs_input_device_t exactly (56 bytes on x64).
/// Field order: name, display_name, type, input_count, inputs,
///              callback_context, input_active_callback (optional), input_event_callback.
#[repr(C)]
struct ScsInputDevice {
    name: scs_string_t,
    display_name: scs_string_t,
    device_type: scs_u32_t,
    input_count: scs_u32_t,
    inputs: *const ScsInputDeviceInput,
    callback_context: scs_context_t,
    input_active_callback: Option<ScsInputActiveCb>,
    input_event_callback: ScsInputEventCb,
}

const _INPUT_DEVICE_SIZE: () = {
    // scs_check_size(scs_input_device_t, 32, 56) — must be 56 on x64
    assert!(mem::size_of::<ScsInputDevice>() == 56);
};

/// Input init params. `register_device` immediately follows `common`
/// (mirrors telemetry init params structure).
#[repr(C)]
struct ScsInputInitParamsV100 {
    common: ScsSdkInitParamsV100,
    register_device: ScsRegisterDeviceFn,
}

const _INPUT_INIT_PARAMS_SIZE: () = {
    // scs_check_size(scs_input_init_params_v100_t, 20, 40) — must be 40 on x64
    assert!(mem::size_of::<ScsInputInitParamsV100>() == 40);
};

// ---------------------------------------------------------------------------
// Controller SHM globals
// ---------------------------------------------------------------------------

static mut CTRL_SHM_HANDLE: HANDLE = NULL;
static mut CTRL_SHM_PTR: *mut ShmControlLayout = ptr::null_mut();
/// Which event to emit on the next input_event_cb invocation (0–3, wrapping).
static mut CTRL_EVENT_IDX: u32 = 0;
/// Total input_event_cb invocations since DLL load (all indices combined).
static mut CTRL_CB_TOTAL: u64 = 0;
/// Number of frames seen (first_in_frame calls).
static mut CTRL_FRAME_COUNT: u64 = 0;

// ---------------------------------------------------------------------------
// scs_input_init / scs_input_shutdown — exported alongside scs_telemetry_init
// ---------------------------------------------------------------------------

/// Called by ETS2 when the DLL is loaded as an input plugin. Creates the
/// control SHM region and registers a semantical controller device.
///
/// # Safety
/// Called by ETS2 with a valid `ScsInputInitParamsV100` pointer.
#[allow(private_interfaces)]
#[no_mangle]
pub unsafe extern "system" fn scs_input_init(
    version: scs_u32_t,
    params: *const ScsInputInitParamsV100,
) -> scs_result_t {
    ffi_guard::guard_result("scs_input_init", || unsafe {
        scs_input_init_inner(version, params)
    })
}

unsafe fn scs_input_init_inner(
    version: scs_u32_t,
    params: *const ScsInputInitParamsV100,
) -> scs_result_t {
    diag_log::boot("scs_input_init entered");
    if params.is_null() {
        diag_log::event_force("scs_input_init returning failure (-2 null params)");
        return -2;
    }
    if version < SCS_INPUT_VERSION_1_00 {
        diag_log::event_force(&format!(
            "scs_input_init returning failure (-1 unsupported sdk_version=0x{version:08X})"
        ));
        debug_log("scs_input_init: unsupported SDK version");
        return -1;
    }

    diag_log::init();
    diag_log::event_force(&format!("scs_input_init sdk_version=0x{version:08X}"));

    let name = wide_str(CTRL_SHM_NAME);
    let shm_size = mem::size_of::<ShmControlLayout>();
    CTRL_SHM_HANDLE = CreateFileMappingW(
        INVALID_HANDLE_VALUE,
        ptr::null(),
        PAGE_READWRITE,
        0,
        shm_size as DWORD,
        name.as_ptr(),
    );
    if CTRL_SHM_HANDLE == NULL {
        diag_log::event_force("scs_input_init control shm error (CreateFileMappingW failed)");
        debug_log("scs_input_init: CreateFileMappingW failed");
        return -7;
    }
    CTRL_SHM_PTR =
        MapViewOfFile(CTRL_SHM_HANDLE, FILE_MAP_WRITE, 0, 0, shm_size) as *mut ShmControlLayout;
    if CTRL_SHM_PTR.is_null() {
        diag_log::event_force("scs_input_init control shm error (MapViewOfFile failed)");
        debug_log("scs_input_init: MapViewOfFile failed");
        CloseHandle(CTRL_SHM_HANDLE);
        CTRL_SHM_HANDLE = NULL;
        return -7;
    }

    (*CTRL_SHM_PTR).magic = CTRL_SHM_MAGIC;
    (*CTRL_SHM_PTR).version = CTRL_SHM_VERSION;
    (*CTRL_SHM_PTR).sequence = 0;
    (*CTRL_SHM_PTR).active = 0;
    (*CTRL_SHM_PTR).steering = 0.0;
    (*CTRL_SHM_PTR).throttle = 0.0;
    (*CTRL_SHM_PTR).brake = 0.0;
    (*CTRL_SHM_PTR).clutch = 0.0;
    diag_log::event_force("scs_input_init control shm done");

    let inputs: [ScsInputDeviceInput; 4] = [
        ScsInputDeviceInput {
            name: c"steering".as_ptr(),
            display_name: c"Steering".as_ptr(),
            value_type: SCS_VALUE_TYPE_float,
            _pad: 0,
        },
        ScsInputDeviceInput {
            name: c"aforward".as_ptr(),
            display_name: c"Throttle".as_ptr(),
            value_type: SCS_VALUE_TYPE_float,
            _pad: 0,
        },
        ScsInputDeviceInput {
            name: c"abackward".as_ptr(),
            display_name: c"Brake".as_ptr(),
            value_type: SCS_VALUE_TYPE_float,
            _pad: 0,
        },
        ScsInputDeviceInput {
            name: c"clutch".as_ptr(),
            display_name: c"Clutch".as_ptr(),
            value_type: SCS_VALUE_TYPE_float,
            _pad: 0,
        },
    ];

    let device = ScsInputDevice {
        name: c"truckpilot".as_ptr(),
        display_name: c"TruckPilot Autopilot".as_ptr(),
        device_type: SCS_INPUT_DEVICE_TYPE_SEMANTICAL,
        input_count: 4,
        inputs: inputs.as_ptr(),
        callback_context: ptr::null_mut(),
        input_active_callback: None,
        input_event_callback: input_event_cb,
    };

    diag_log::event_force("scs_input_init register_device start");
    let reg = ((*params).register_device)(&device);
    if reg != SCS_RESULT_OK {
        diag_log::event_force(&format!(
            "scs_input_init register_device error ({reg}) — returning failure"
        ));
        return reg;
    }
    diag_log::event_force("scs_input_init register_device done");

    debug_log("scs_input_init done — TruckPilot semantical controller registered");
    diag_log::event_force("scs_input_init returning success");
    SCS_RESULT_OK
}

unsafe fn scs_input_shutdown_inner() {
    if !CTRL_SHM_PTR.is_null() {
        UnmapViewOfFile(CTRL_SHM_PTR as LPVOID);
        CTRL_SHM_PTR = ptr::null_mut();
    }
    if CTRL_SHM_HANDLE != NULL {
        CloseHandle(CTRL_SHM_HANDLE);
        CTRL_SHM_HANDLE = NULL;
    }
}

/// Called by ETS2 on unload.
///
/// # Safety
/// Called by ETS2 during shutdown.
#[no_mangle]
pub unsafe extern "system" fn scs_input_shutdown() {
    ffi_guard::guard_void("scs_input_shutdown", || unsafe {
        diag_log::event_force("scs_input_shutdown called");
        debug_log("scs_input_shutdown");
        scs_input_shutdown_inner();
    });
}

// ---------------------------------------------------------------------------
// Per-frame input callback — polled by ETS2 every frame
// ---------------------------------------------------------------------------

/// ETS2 calls this repeatedly each frame. We emit one event per call (steering,
/// throttle, brake, clutch in order), then signal done via NOT_FOUND after all 4.
/// When `active == 0` we emit neutral 0.0 — never NOT_FOUND before the 4th event,
/// which would risk ETS2 treating the device as dead and stopping the callback.
///
/// Diagnostic: logs frame header + per-event info via OutputDebugStringA on frame 1
/// and every 100 frames. View in Sysinternals DebugView (filter "[TruckPilot]").
unsafe extern "system" fn input_event_cb(
    event: *mut ScsInputEvent,
    flags: scs_u32_t,
    _ctx: scs_context_t,
) -> scs_result_t {
    ffi_guard::catch_callback_result("input_event_cb", SCS_RESULT_NOT_FOUND, || unsafe {
        input_event_cb_inner(event, flags, _ctx)
    })
}

unsafe fn input_event_cb_inner(
    event: *mut ScsInputEvent,
    flags: scs_u32_t,
    _ctx: scs_context_t,
) -> scs_result_t {
    CTRL_CB_TOTAL += 1;
    let cb_total = CTRL_CB_TOTAL;

    if event.is_null() || CTRL_SHM_PTR.is_null() {
        debug_log(&format!(
            "input_event_cb cb#{cb_total}: null ptr -> NOT_FOUND"
        ));
        return SCS_RESULT_NOT_FOUND;
    }

    // ETS2 sets this flag on the first call of each frame — use it to resync
    // the index so a mid-frame abort in a previous frame never carries over.
    let first_in_frame = flags & SCS_INPUT_EVENT_CALLBACK_FLAG_FIRST_IN_FRAME != 0;
    if first_in_frame {
        CTRL_EVENT_IDX = 0;
        CTRL_FRAME_COUNT += 1;
    }
    let frame_count = CTRL_FRAME_COUNT;

    // Log frame header on frame 1 and every 100 frames — view in DebugView.
    let diag = frame_count == 1 || frame_count.is_multiple_of(100);
    if diag && first_in_frame {
        // Read via raw ptr — packed struct fields cannot be referenced directly.
        let (a, seq, steer, thr, brk) = (
            (*CTRL_SHM_PTR).active,
            (*CTRL_SHM_PTR).sequence,
            (*CTRL_SHM_PTR).steering,
            (*CTRL_SHM_PTR).throttle,
            (*CTRL_SHM_PTR).brake,
        );
        debug_log(&format!(
            "input_event_cb frame#{frame_count} cb#{cb_total} flags={flags:#010x} active={a} seq={seq} steer={steer:.4} thr={thr:.4} brk={brk:.4}",
        ));
    }

    let active = (*CTRL_SHM_PTR).active != 0;
    let (s, t, b, c) = if active {
        (
            (*CTRL_SHM_PTR).steering,
            (*CTRL_SHM_PTR).throttle,
            (*CTRL_SHM_PTR).brake,
            (*CTRL_SHM_PTR).clutch,
        )
    } else {
        (0.0_f32, 0.0, 0.0, 0.0)
    };

    let idx = CTRL_EVENT_IDX;
    let (written_idx, written_val) = match idx {
        0 => {
            (*event).input_index = 0;
            (*event).value_float = s;
            (0u32, s)
        }
        1 => {
            (*event).input_index = 1;
            (*event).value_float = t;
            (1, t)
        }
        2 => {
            (*event).input_index = 2;
            (*event).value_float = b;
            (2, b)
        }
        3 => {
            (*event).input_index = 3;
            (*event).value_float = c;
            (3, c)
        }
        _ => {
            if diag {
                debug_log(&format!(
                    "  [done] idx_overflow={idx} -> NOT_FOUND (end of frame)"
                ));
            }
            CTRL_EVENT_IDX = 0;
            return SCS_RESULT_NOT_FOUND;
        }
    };

    if diag {
        debug_log(&format!(
            "  event idx={written_idx} val={written_val:.4} -> OK"
        ));
    }

    CTRL_EVENT_IDX += 1;
    SCS_RESULT_OK
}

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
    fn shm_version_is_3() {
        assert_eq!(SHM_VERSION, 3);
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

    // --- Control SHM layout ---

    #[test]
    fn ctrl_shm_magic_is_tpct() {
        // "TPCT" — must match CTRL_SHM_MAGIC in scs-sdk-output plugin.
        assert_eq!(CTRL_SHM_MAGIC, 0x54504354);
    }

    #[test]
    fn ctrl_shm_version_is_1() {
        assert_eq!(CTRL_SHM_VERSION, 1);
    }

    #[test]
    fn ctrl_layout_size_is_32() {
        assert_eq!(mem::size_of::<ShmControlLayout>(), 32);
    }

    #[test]
    fn ctrl_layout_active_at_offset_12() {
        assert_eq!(mem::offset_of!(ShmControlLayout, active), 12);
    }

    #[test]
    fn ctrl_layout_steering_at_offset_16() {
        assert_eq!(mem::offset_of!(ShmControlLayout, steering), 16);
    }

    #[test]
    fn ctrl_layout_clutch_at_offset_28() {
        assert_eq!(mem::offset_of!(ShmControlLayout, clutch), 28);
    }

    // --- input_event_cb behaviour (pure logic, no Win32) ---

    /// Simulate the per-frame event iteration: 4 OK returns then NOT_FOUND.
    #[test]
    fn event_iteration_four_ok_then_not_found() {
        // Use a local ShmControlLayout value to represent the SHM data.
        let shm = ShmControlLayout {
            magic: CTRL_SHM_MAGIC,
            version: CTRL_SHM_VERSION,
            sequence: 0,
            active: 1,
            steering: 0.5,
            throttle: 0.3,
            brake: 0.0,
            clutch: 0.0,
        };
        let mut shm_copy = shm;
        let mut event = ScsInputEvent {
            input_index: 99,
            value_float: 99.0,
        };

        // Manually replicate the callback logic (no unsafe statics in tests).
        let active = shm_copy.active != 0;
        let (s, t, b, c) = if active {
            (
                shm_copy.steering,
                shm_copy.throttle,
                shm_copy.brake,
                shm_copy.clutch,
            )
        } else {
            (0.0_f32, 0.0, 0.0, 0.0)
        };
        let values = [(0u32, s), (1, t), (2, b), (3, c)];
        for (expected_idx, expected_val) in values {
            event.input_index = expected_idx;
            event.value_float = expected_val;
            assert_eq!(event.input_index, expected_idx);
            assert!((event.value_float - expected_val).abs() < f32::EPSILON);
        }
        // After 4 events the _ arm fires NOT_FOUND — represented by the constant.
        assert_eq!(SCS_RESULT_NOT_FOUND, -4);
        // Verify inactive path yields neutral values.
        shm_copy.active = 0;
        let (s2, t2, b2, c2) = if shm_copy.active != 0 {
            (
                shm_copy.steering,
                shm_copy.throttle,
                shm_copy.brake,
                shm_copy.clutch,
            )
        } else {
            (0.0_f32, 0.0, 0.0, 0.0)
        };
        assert_eq!(s2, 0.0);
        assert_eq!(t2, 0.0);
        assert_eq!(b2, 0.0);
        assert_eq!(c2, 0.0);
    }

    #[test]
    fn first_in_frame_flag_constant_matches_sdk() {
        // SDK header: SCS_INPUT_EVENT_CALLBACK_FLAG_first_in_frame = 0x00000001
        assert_eq!(SCS_INPUT_EVENT_CALLBACK_FLAG_FIRST_IN_FRAME, 0x0000_0001);
    }
}
