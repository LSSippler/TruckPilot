//! In-process navigation route UID walk for ETS2 1.59 (Phase R2+).
//!
//! Publishes route UIDs exclusively via seqlock `Local\TruckPilotRouteBlackboard`.
//! Legacy `Local\TruckPilotNavRoute` was removed in Phase 5h.

#![allow(clippy::cast_possible_wrap)]

use crate::nav_resolve::{
    invalidate_session_cache, resolve_game_ctrl_cached, verify_trip_distance, GameCtrlSessionCache,
};
use crate::resolver_sched::ResolverSchedule;
use crate::resolver_worker::{
    self, FRAME_CB_COUNT, FRAME_END_COUNT, FRAME_START_COUNT, LAST_FRAME_END_US,
    RESOLVER_RESET_REQUESTED, ROUTE_TICK_COUNT, SESSION_STARTED_US,
};
use crate::route_chain::{self, LiveMem};
use crate::route_status::{
    self, DebounceGate, MilestoneLog, PauseGateDecision, RateLog, SuppressedSummaryLog,
    WorldEventState, WorldEventAction, PUBLISH_ACTIVE, PUBLISH_EMPTY, PUBLISH_NONE,
    RESOLVE_FIRST_UID_ZERO, RESOLVE_NONE, RESOLVE_PAUSED_NO_ROUTE_WALK,
    RESOLVE_ROUTE_ITEMS_CANDIDATE_NULL, RESOLVE_ROUTE_ITEMS_EMPTY, RESOLVE_ROUTE_TASK_PTR_NULL,
    RESOLVE_WAYPOINTS_COLLECTED,
    RESOLVE_UNSAFE_ROUTE_SCAN_DISABLED, RESOLVE_GPS_RESOLVED_ROUTE_SCAN_DISABLED,
    RESOLVE_GPS_MANAGER_NOT_RESOLVED,
    RESOLVE_GPS_TABLE_ONLY_DONE, RESOLVE_GPS_TABLE_READ_FAILED,
    RESOLVE_GAME_CTRL_TABLE_ONLY_DONE, RESOLVE_GAME_CTRL_TABLE_READ_FAILED,
    RESOLVE_ROUTE_CANDIDATE_TABLE_DONE, RESOLVE_ROUTE_CANDIDATE_TABLE_READ_FAILED,
    RESOLVE_ROUTE_RESOLVER_WORKER_ACTIVE, RESOLVE_ROUTE_RESOLVER_PARKED,
    RESOLVE_ROUTE_RESOLVER_BACKOFF, RESOLVE_ROUTE_RESOLVER_CACHE_HIT,
    RESOLVE_ROUTE_RESOLVER_SCAN_LIMITED,
    RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE,
    evaluate_route_scan_gate, ROUTE_SCAN_MIN_INTERVAL_US,
    SCS_EVENT_FRAME_END, SCS_EVENT_FRAME_START, WORLD_RESET_MIN_INTERVAL_US,
    evaluate_pause_gate, pack_route_tick_meta, pack_waypoint_frame_end, pack_world_reset,
    should_frame_start_fallback, WORLD_RESET_REASON_PAUSED,
};

// ---------------------------------------------------------------------------
// Route blackboard SHM — canonical route channel (must match telemetry crate).
// ---------------------------------------------------------------------------

/// Route blackboard — primary route channel (must match telemetry crate).
pub const MAX_ROUTE_WAYPOINTS: usize = 6000;
pub const ROUTE_MAGIC: u32 = 0x4252_5054; // "TPRB"
pub const ROUTE_VERSION: u32 = 1;
pub const ROUTE_SHM_NAME: &str = "Local\\TruckPilotRouteBlackboard";
pub const ROUTE_FLAG_TRUNCATED: u32 = 1;

/// `reserved[0]` — DLL wrote this mapping (always set after successful init).
pub use route_status::ROUTE_BB_STATUS_DLL_ACTIVE;
/// `reserved[0]` — last route walk resolved `route_task` pointer chain.
pub use route_status::ROUTE_BB_STATUS_ROUTE_TASK_OK;
/// `reserved[0]` — at least one route tick ran after init.
pub use route_status::ROUTE_BB_STATUS_ROUTE_TICK_SEEN;
/// `reserved[0]` — at least one telemetry frame callback ran after init.
pub use route_status::ROUTE_BB_STATUS_FRAME_CB_SEEN;
pub use route_status::ROUTE_BB_STATUS_PAUSE_EVENT_SEEN;
pub use route_status::ROUTE_BB_STATUS_PAUSE_GATE_ACTIVE;
pub use route_status::ROUTE_BB_STATUS_FRAME_END_MISSING;
pub use route_status::ROUTE_BB_STATUS_TICK_SOURCE_FALLBACK;
pub use route_status::RouteTickSource;

/// Per-waypoint bitflags (Phase 5i) — must match `crates/telemetry/src/nav_route.rs`.
/// Some flags are asserted only in tests until coord/time fill is wired in the DLL walk.
#[allow(dead_code)]
pub const ROUTE_WP_FLAG_HAS_POSITION: u32 = 1 << 0;
pub const ROUTE_WP_FLAG_HAS_DISTANCE: u32 = 1 << 1;
#[allow(dead_code)]
pub const ROUTE_WP_FLAG_HAS_TIME: u32 = 1 << 2;
pub const ROUTE_WP_FLAG_UNTRUSTED: u32 = 1 << 3;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct RouteWaypoint {
    pub uid: i64,
    /// ETS2 world X (horizontal, metres).
    pub x: f32,
    /// ETS2 world Y (height / elevation, metres — same axis as telemetry `position[1]`).
    pub y: f32,
    /// ETS2 world Z (horizontal, metres).
    pub z: f32,
    pub distance: f32,
    pub time: f32,
    pub flags: u32,
}

#[repr(C)]
pub struct RouteBlackboard {
    pub magic: u32,
    pub version: u32,
    pub sequence: u32,
    pub valid: u32,
    pub waypoint_count: u32,
    pub flags: u32,
    pub route_hash: u64,
    pub reserved: [u32; 8],
    pub waypoints: [RouteWaypoint; MAX_ROUTE_WAYPOINTS],
}

// Compile-time layout guard.
const _: () = {
    use std::mem;
    assert!(mem::size_of::<RouteWaypoint>() == 32);
    assert!(mem::offset_of!(RouteBlackboard, waypoints) == 64);
    assert!(mem::size_of::<RouteBlackboard>() == 64 + MAX_ROUTE_WAYPOINTS * 32);
};

// --- 1.59 offsets — canonical chain in `route_chain.rs` --------------------
// --- 1.59 item layout (verified UID @ +0x30, active @ +0x0C) ---------------

const ITEM_STRIDE: usize = route_chain::ITEM_STRIDE;
const OFF_ITEM_UID: usize = route_chain::OFF_ITEM_UID;
const OFF_ITEM_ACTIVE: usize = route_chain::OFF_ITEM_ACTIVE;
const OFF_ITEM_DIST_LEFT: usize = route_chain::OFF_ITEM_DIST_LEFT;

const MAX_ROUTE_ITEMS: usize = 4000;
const SIZE_FIELD_MAX: u64 = 6000;

/// UID below this is garbage past the real route tail (sanity cap only, not primary end).
const UID_MIN_PLAUSIBLE: u64 = 5_000_000_000_000_000_000;

const NAV_ROUTE_MIN_INTERVAL_US: u64 = ROUTE_SCAN_MIN_INTERVAL_US;
const GPS_RETRY_INTERVAL_US: u64 = 2_000_000;

// ---------------------------------------------------------------------------
// Pure helpers (unit-testable)
// ---------------------------------------------------------------------------

fn looks_like_heap_ptr(v: u64) -> bool {
    (0x1_0000..0x0007_FFFF_FFFF_FFFF).contains(&v)
}

fn uid_below_sanity_cap(uid: u64) -> bool {
    uid == 0 || uid < UID_MIN_PLAUSIBLE
}

/// Drop trailing items whose `+0x0C` active-dword is zero (padding past route end).
#[cfg(test)]
pub fn trim_trailing_inactive_count(len: usize, is_active_tail: impl Fn(usize) -> bool) -> usize {
    let mut n = len;
    while n > 0 && !is_active_tail(n - 1) {
        n -= 1;
    }
    n
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WalkStop {
    /// Final UID count after uid-cap scan + inactive-tail trim.
    ActiveCount(usize),
    /// Scan stopped at index `i` due to uid sanity cap (`< 5e18`).
    UidSanityCap(usize),
    /// Filled scan bound without uid cap — suspect garbage loop.
    MaxCap,
    /// No items after trim (or index 0 failed uid cap).
    NoItemsAtZero,
    /// `physical_route_items.ptr` missing or invalid.
    NoArrayPtr,
}

/// FNV-1a 64-bit over the UID sequence (change detection).
pub fn uid_sequence_hash(uids: &[u64]) -> u64 {
    let mut h = 0xcbf29ce484222325u64;
    for &uid in uids {
        for byte in uid.to_le_bytes() {
            h ^= u64::from(byte);
            h = h.wrapping_mul(0x100000001b3);
        }
    }
    h
}

pub fn uid_sequence_hash_waypoints(waypoints: &[RouteWaypoint]) -> u64 {
    uid_sequence_hash(
        &waypoints
            .iter()
            .map(|wp| wp.uid as u64)
            .collect::<Vec<_>>(),
    )
}

/// Plausibility gate for `physical_route_item+0x14` distance-left (metres).
pub fn distance_left_plausible(meters: f32) -> bool {
    meters.is_finite() && meters >= 0.0 && meters < 50_000_000.0
}

/// Build a SHM waypoint from UID + optional distance-left read from ETS2 memory.
///
/// Position/time are not extracted in 1.59 without a verified node-pointer chain;
/// x/y/z stay zero and no [`ROUTE_WP_FLAG_HAS_POSITION`] is set.
pub fn waypoint_from_route_item(uid: u64, dist_left: f32) -> RouteWaypoint {
    let mut wp = RouteWaypoint {
        uid: uid as i64,
        ..Default::default()
    };
    if distance_left_plausible(dist_left) {
        wp.distance = dist_left;
        wp.flags |= ROUTE_WP_FLAG_HAS_DISTANCE | ROUTE_WP_FLAG_UNTRUSTED;
    }
    wp
}

/// Scan bound: use `array_dyn.size` only when plausible (usually garbage in 1.59).
pub fn route_item_limit(size_field: u64) -> usize {
    if (1..=SIZE_FIELD_MAX).contains(&size_field) {
        (size_field as usize).min(MAX_ROUTE_ITEMS)
    } else {
        MAX_ROUTE_ITEMS
    }
}

// ---------------------------------------------------------------------------
// In-process memory reads (unsafe, encapsulated)
// ---------------------------------------------------------------------------

#[cfg(windows)]
mod win {
    use super::*;
    use std::ptr;

    extern "system" {
        fn OutputDebugStringA(lpOutputString: *const u8);
        fn CreateFileMappingW(
            hFile: isize,
            lpAttr: *const core::ffi::c_void,
            flProtect: u32,
            dwSizeHigh: u32,
            dwSizeLow: u32,
            lpName: *const u16,
        ) -> isize;
        fn MapViewOfFile(
            hMapping: isize,
            dwAccess: u32,
            offHigh: u32,
            offLow: u32,
            n: usize,
        ) -> *mut core::ffi::c_void;
        fn UnmapViewOfFile(base: *mut core::ffi::c_void) -> i32;
        fn CloseHandle(h: isize) -> i32;
    }

    const INVALID_FILE_HANDLE: isize = -1isize;
    const PAGE_READWRITE: u32 = 0x04;
    const FILE_MAP_WRITE: u32 = 0x02;

    static mut RB_SHM_HANDLE: isize = 0;
    static mut RB_SHM_PTR: *mut RouteBlackboard = ptr::null_mut();
    static mut RB_SEQUENCE: u32 = 0;

    fn wide_nr(s: &str) -> Vec<u16> {
        s.encode_utf16().chain(std::iter::once(0)).collect()
    }

    pub fn init_shm() -> bool {
        init_route_bb_shm()
    }

    fn init_route_bb_shm() -> bool {
        unsafe {
            let name = wide_nr(ROUTE_SHM_NAME);
            let size = std::mem::size_of::<RouteBlackboard>();
            let h = CreateFileMappingW(
                INVALID_FILE_HANDLE,
                ptr::null(),
                PAGE_READWRITE,
                0,
                size as u32,
                name.as_ptr(),
            );
            if h == 0 {
                nav_warn("init_shm: CreateFileMappingW failed (RouteBlackboard)");
                return false;
            }
            let raw = MapViewOfFile(h, FILE_MAP_WRITE, 0, 0, size);
            if raw.is_null() {
                CloseHandle(h);
                nav_warn("init_shm: MapViewOfFile failed (RouteBlackboard)");
                return false;
            }
            let shm = raw as *mut RouteBlackboard;
            (*shm).magic = ROUTE_MAGIC;
            (*shm).version = ROUTE_VERSION;
            (*shm).sequence = 0;
            (*shm).valid = 0;
            (*shm).waypoint_count = 0;
            (*shm).flags = 0;
            (*shm).route_hash = 0;
            (*shm).reserved = [0; 8];
            (*shm).reserved[0] = ROUTE_BB_STATUS_DLL_ACTIVE;
            RB_SHM_HANDLE = h;
            RB_SHM_PTR = shm;
            crate::diag_log::state("route_bb", "mapped");
            nav_log(&format!("RouteBlackboard SHM ready ({size} bytes)"));
            true
        }
    }

    /// Publish an empty invalid route through the seqlock (call after [`init_shm`]).
    pub fn publish_initial_empty() {
        unsafe {
            if RB_SHM_PTR.is_null() {
                return;
            }
            write_route_blackboard_invalid();
            let shm = &mut *RB_SHM_PTR;
            ptr::write_volatile(&mut shm.reserved[0], ROUTE_BB_STATUS_DLL_ACTIVE);
            if crate::safe_mem::route_resolver_mode().is_off() {
                ptr::write_volatile(
                    &mut shm.reserved[1],
                    RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE,
                );
            } else {
                ptr::write_volatile(&mut shm.reserved[1], 0);
            }
            crate::diag_log::state("route", "empty");
            crate::diag_log::state("waypoint_count", "0");
            crate::diag_log::state("route_hash", "0");
        }
    }

    fn set_bb_route_task_ok(ok: bool) {
        unsafe {
            if RB_SHM_PTR.is_null() {
                return;
            }
            let shm = &mut *RB_SHM_PTR;
            let mut status = ptr::read_volatile(&shm.reserved[0]);
            if ok {
                status |= ROUTE_BB_STATUS_ROUTE_TASK_OK;
            } else {
                status &= !ROUTE_BB_STATUS_ROUTE_TASK_OK;
            }
            ptr::write_volatile(&mut shm.reserved[0], status);
        }
    }

    fn walk_stop_to_resolve_status(stop: WalkStop) -> u32 {
        match stop {
            WalkStop::NoArrayPtr => RESOLVE_ROUTE_ITEMS_CANDIDATE_NULL,
            WalkStop::NoItemsAtZero => RESOLVE_ROUTE_ITEMS_EMPTY,
            WalkStop::UidSanityCap(0) => RESOLVE_FIRST_UID_ZERO,
            WalkStop::ActiveCount(0) => RESOLVE_ROUTE_ITEMS_EMPTY,
            WalkStop::ActiveCount(_) | WalkStop::UidSanityCap(_) | WalkStop::MaxCap => {
                RESOLVE_WAYPOINTS_COLLECTED
            }
        }
    }

    /// Update frame-only diagnostic fields — never overwrites resolver status.
    unsafe fn write_bb_frame_only(state: &NavRouteState, bump_sequence: bool) {
        if RB_SHM_PTR.is_null() {
            return;
        }
        let shm = &mut *RB_SHM_PTR;
        if bump_sequence {
            let seq = ptr::read_volatile(&shm.sequence);
            let odd = seq.wrapping_add(1) | 1;
            ptr::write_volatile(&mut shm.sequence, odd);
            RB_SEQUENCE = odd.wrapping_add(1);
            ptr::write_volatile(&mut shm.sequence, RB_SEQUENCE);
        }
        let mut bits = ptr::read_volatile(&shm.reserved[0]);
        bits |= ROUTE_BB_STATUS_DLL_ACTIVE | ROUTE_BB_STATUS_FRAME_CB_SEEN;
        if state.route_tick_count > 0 {
            bits |= ROUTE_BB_STATUS_ROUTE_TICK_SEEN;
        }
        if state.world_events.pause_event_seen() {
            bits |= ROUTE_BB_STATUS_PAUSE_EVENT_SEEN;
        }
        if state.last_pause_gate.pause_gate_active {
            bits |= ROUTE_BB_STATUS_PAUSE_GATE_ACTIVE;
        } else {
            bits &= !ROUTE_BB_STATUS_PAUSE_GATE_ACTIVE;
        }
        if state.frame_end_event_count == 0 && state.frame_start_count > 0 {
            bits |= ROUTE_BB_STATUS_FRAME_END_MISSING;
        } else {
            bits &= !ROUTE_BB_STATUS_FRAME_END_MISSING;
        }
        if state.last_tick_source == RouteTickSource::FrameStartFallback {
            bits |= ROUTE_BB_STATUS_TICK_SOURCE_FALLBACK;
        } else {
            bits &= !ROUTE_BB_STATUS_TICK_SOURCE_FALLBACK;
        }
        ptr::write_volatile(&mut shm.reserved[0], bits);
        ptr::write_volatile(
            &mut shm.reserved[4],
            state.frame_cb_count,
        );
        ptr::write_volatile(
            &mut shm.reserved[5],
            pack_route_tick_meta(state.route_tick_count, state.frame_start_count),
        );
        ptr::write_volatile(
            &mut shm.reserved[6],
            pack_waypoint_frame_end(
                state.last_waypoint_count.min(u32::MAX as usize) as u32,
                state.frame_end_event_count,
            ),
        );
        ptr::write_volatile(
            &mut shm.reserved[7],
            pack_world_reset(
                state.world_reset_count,
                state.last_world_reset_reason,
                state.world_reset_gate.suppressed().min(255),
            ),
        );
    }

    /// Update resolver / publish diagnostic fields (call after each resolve attempt).
    unsafe fn write_bb_resolve_diag(state: &NavRouteState, bump_sequence: bool) {
        if RB_SHM_PTR.is_null() {
            return;
        }
        let shm = &mut *RB_SHM_PTR;
        if bump_sequence {
            let seq = ptr::read_volatile(&shm.sequence);
            let odd = seq.wrapping_add(1) | 1;
            ptr::write_volatile(&mut shm.sequence, odd);
            RB_SEQUENCE = odd.wrapping_add(1);
            ptr::write_volatile(&mut shm.sequence, RB_SEQUENCE);
        }
        let status = route_status::effective_resolve_status(
            state.last_resolve_status,
            state.resolve_attempts,
        );
        ptr::write_volatile(&mut shm.reserved[1], status);
        ptr::write_volatile(&mut shm.reserved[2], state.resolve_attempts);
        ptr::write_volatile(&mut shm.reserved[3], state.last_publish_status);
        ptr::write_volatile(
            &mut shm.reserved[6],
            pack_waypoint_frame_end(
                state.last_waypoint_count.min(u32::MAX as usize) as u32,
                state.frame_end_event_count,
            ),
        );
        ptr::write_volatile(
            &mut shm.reserved[7],
            pack_world_reset(
                state.world_reset_count,
                state.last_world_reset_reason,
                state.world_reset_gate.suppressed().min(255),
            ),
        );
        write_bb_frame_only(state, false);
    }

    fn record_resolve_status(state: &mut NavRouteState, status: u32, timestamp_us: u64) {
        debug_assert_ne!(status, RESOLVE_NONE);
        let changed = state.last_resolve_status != status;
        state.last_resolve_status = status;
        state.rate_log.event(
            "resolve",
            &format!(
                "resolver attempt count={} status={}",
                state.resolve_attempts,
                route_status::resolve_status_str(status)
            ),
            timestamp_us,
            changed || state.resolve_attempts <= 5,
        );
    }

    fn record_publish_status(
        state: &mut NavRouteState,
        status: u32,
        timestamp_us: u64,
        waypoint_count: usize,
        route_hash: u64,
    ) {
        let changed = state.last_publish_status != status;
        state.last_publish_status = status;
        let msg = match status {
            PUBLISH_ACTIVE => format!(
                "publish active waypoint_count={waypoint_count} route_hash={route_hash:#x}"
            ),
            PUBLISH_EMPTY => "publish empty status=valid_false".into(),
            _ => format!("publish status={}", route_status::publish_status_str(status)),
        };
        state.rate_log.event("publish", &msg, timestamp_us, changed);
    }

    struct NavRouteState {
        cached_gps: *const u8,
        cached_game_ctrl: u64,
        last_gps_resolve_us: u64,
        waypoint_buf: Vec<RouteWaypoint>,
        last_hash: u64,
        last_walk_us: u64,
        first_success_logged: bool,
        buffer_ready: bool,
        empty_published: bool,
        last_route_task_ok: bool,
        last_phys_items_offset: usize,
        last_chain_candidate: &'static str,
        last_chain_failed_step: &'static str,
        last_pause_gate: PauseGateDecision,
        last_tick_source: RouteTickSource,
        last_waypoint_count: usize,
        resolve_attempts: u32,
        route_tick_count: u32,
        frame_cb_count: u32,
        frame_start_count: u32,
        frame_end_event_count: u32,
        last_frame_end_event_us: u64,
        last_frame_event: u32,
        fallback_logged: bool,
        last_resolve_status: u32,
        last_publish_status: u32,
        world_reset_count: u32,
        world_reset_suppressed_total: u32,
        last_world_reset_reason: u32,
        world_reset_gate: DebounceGate,
        world_events: WorldEventState,
        rate_log: RateLog,
        frame_milestone: MilestoneLog,
        tick_milestone: MilestoneLog,
        suppressed_log: SuppressedSummaryLog,
        last_chain_fail_status: u32,
        session_started_us: u64,
        scan_policy_logged: bool,
        diagnostic_park_logged: bool,
    }

    impl NavRouteState {
        const fn empty() -> Self {
            Self {
                cached_gps: std::ptr::null(),
                cached_game_ctrl: 0,
                last_gps_resolve_us: 0,
                waypoint_buf: Vec::new(),
                last_hash: 0,
                last_walk_us: 0,
                first_success_logged: false,
                buffer_ready: false,
                empty_published: true,
                last_route_task_ok: false,
                last_phys_items_offset: route_chain::OFF_PHYS_ITEMS,
                last_chain_candidate: "",
                last_chain_failed_step: "",
                last_pause_gate: PauseGateDecision::NONE,
                last_tick_source: RouteTickSource::FrameEnd,
                last_waypoint_count: 0,
                resolve_attempts: 0,
                route_tick_count: 0,
                frame_cb_count: 0,
                frame_start_count: 0,
                frame_end_event_count: 0,
                last_frame_end_event_us: 0,
                last_frame_event: 0,
                fallback_logged: false,
                last_resolve_status: RESOLVE_NONE,
                last_publish_status: PUBLISH_NONE,
                world_reset_count: 0,
                world_reset_suppressed_total: 0,
                last_world_reset_reason: route_status::WORLD_RESET_REASON_NONE,
                world_reset_gate: DebounceGate::new(WORLD_RESET_MIN_INTERVAL_US),
                world_events: WorldEventState::new(),
                rate_log: RateLog::new(),
                frame_milestone: MilestoneLog::new(),
                tick_milestone: MilestoneLog::new(),
                suppressed_log: SuppressedSummaryLog::new(),
                last_chain_fail_status: RESOLVE_NONE,
                session_started_us: 0,
                scan_policy_logged: false,
                diagnostic_park_logged: false,
            }
        }

        fn soft_reset_resolver_cache(&mut self) {
            self.cached_gps = std::ptr::null();
            self.cached_game_ctrl = 0;
            self.last_gps_resolve_us = 0;
            self.last_walk_us = 0;
            self.empty_published = false;
            self.diagnostic_park_logged = false;
        }

        fn ensure_buffer(&mut self) {
            if !self.buffer_ready {
                self.waypoint_buf.reserve(MAX_ROUTE_ITEMS);
                self.buffer_ready = true;
            }
        }
    }

    static mut NAV_ROUTE: NavRouteState = NavRouteState::empty();
    static RESOLVER_SCHEDULE: std::sync::Mutex<ResolverSchedule> =
        std::sync::Mutex::new(ResolverSchedule::new());

    fn sync_atomics_to_state(state: &mut NavRouteState) {
        state.frame_cb_count = FRAME_CB_COUNT.load(std::sync::atomic::Ordering::Relaxed);
        state.frame_start_count = FRAME_START_COUNT.load(std::sync::atomic::Ordering::Relaxed);
        state.frame_end_event_count = FRAME_END_COUNT.load(std::sync::atomic::Ordering::Relaxed);
        state.route_tick_count = ROUTE_TICK_COUNT.load(std::sync::atomic::Ordering::Relaxed);
        state.last_frame_end_event_us = LAST_FRAME_END_US.load(std::sync::atomic::Ordering::Relaxed);
        let started = SESSION_STARTED_US.load(std::sync::atomic::Ordering::Relaxed);
        if started != 0 {
            state.session_started_us = started;
        }
    }

    /// Lightweight RouteBlackboard frame fields — safe from SCS callback thread.
    pub fn write_bb_frame_from_atomics() {
        unsafe {
            if RB_SHM_PTR.is_null() {
                return;
            }
            let state = &*std::ptr::addr_of!(NAV_ROUTE);
            let frame_cb = FRAME_CB_COUNT.load(std::sync::atomic::Ordering::Relaxed);
            let frame_start = FRAME_START_COUNT.load(std::sync::atomic::Ordering::Relaxed);
            let frame_end = FRAME_END_COUNT.load(std::sync::atomic::Ordering::Relaxed);
            let route_tick = ROUTE_TICK_COUNT.load(std::sync::atomic::Ordering::Relaxed);
            let shm = &mut *RB_SHM_PTR;
            let mut bits = ptr::read_volatile(&shm.reserved[0]);
            bits |= ROUTE_BB_STATUS_DLL_ACTIVE | ROUTE_BB_STATUS_FRAME_CB_SEEN;
            if route_tick > 0 {
                bits |= ROUTE_BB_STATUS_ROUTE_TICK_SEEN;
            }
            if state.world_events.pause_event_seen() {
                bits |= ROUTE_BB_STATUS_PAUSE_EVENT_SEEN;
            }
            if state.last_pause_gate.pause_gate_active {
                bits |= ROUTE_BB_STATUS_PAUSE_GATE_ACTIVE;
            } else {
                bits &= !ROUTE_BB_STATUS_PAUSE_GATE_ACTIVE;
            }
            if frame_end == 0 && frame_start > 0 {
                bits |= ROUTE_BB_STATUS_FRAME_END_MISSING;
            } else {
                bits &= !ROUTE_BB_STATUS_FRAME_END_MISSING;
            }
            if state.last_tick_source == RouteTickSource::FrameStartFallback {
                bits |= ROUTE_BB_STATUS_TICK_SOURCE_FALLBACK;
            } else {
                bits &= !ROUTE_BB_STATUS_TICK_SOURCE_FALLBACK;
            }
            ptr::write_volatile(&mut shm.reserved[0], bits);
            ptr::write_volatile(&mut shm.reserved[4], frame_cb);
            ptr::write_volatile(
                &mut shm.reserved[5],
                pack_route_tick_meta(route_tick, frame_start),
            );
            ptr::write_volatile(
                &mut shm.reserved[6],
                pack_waypoint_frame_end(
                    state.last_waypoint_count.min(u32::MAX as usize) as u32,
                    frame_end,
                ),
            );
        }
    }

    unsafe fn handle_resolver_off(state: &mut NavRouteState, timestamp_us: u64) {
        if let Ok(mut sched) = RESOLVER_SCHEDULE.lock() {
            sched.park_resolver_off();
        }
        crate::resolver_metrics::set_resolver_parked(true);
        if state.last_resolve_status != RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE {
            record_resolve_status(state, RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE, timestamp_us);
        }
        write_bb_resolve_diag(state, false);
        publish_empty_if_needed(state, timestamp_us);
    }

    unsafe fn park_diagnostic_after_run(state: &mut NavRouteState, status: u32, timestamp_us: u64) {
        let mode = crate::safe_mem::route_resolver_mode();
        if matches!(
            status,
            RESOLVE_GPS_TABLE_ONLY_DONE
                | RESOLVE_GPS_TABLE_READ_FAILED
                | RESOLVE_GAME_CTRL_TABLE_ONLY_DONE
                | RESOLVE_GAME_CTRL_TABLE_READ_FAILED
                | RESOLVE_ROUTE_CANDIDATE_TABLE_DONE
                | RESOLVE_ROUTE_CANDIDATE_TABLE_READ_FAILED
        ) {
            crate::resolver_metrics::note_diagnostic_table_run();
            if let Ok(mut sched) = RESOLVER_SCHEDULE.lock() {
                sched.park_diagnostic_done();
            }
            if !state.diagnostic_park_logged {
                state.diagnostic_park_logged = true;
                crate::diag_log::event_force(&format!(
                    "diagnostic mode {} completed; parking resolver",
                    mode.sidecar_label()
                ));
                crate::diag_log::event_force("resolver parked after diagnostic done");
            }
        }
        record_resolve_status(state, status, timestamp_us);
        write_bb_resolve_diag(state, false);
        publish_empty_if_needed(state, timestamp_us);
    }

    unsafe fn resolve_game_ctrl_for_walk(
        state: &mut NavRouteState,
        timestamp_us: u64,
        force_scan: bool,
    ) -> Result<GameCtrlSessionCache, u32> {
        let allow_scan = RESOLVER_SCHEDULE
            .lock()
            .map(|s| !s.scan_limited())
            .unwrap_or(false);
        match resolve_game_ctrl_cached(force_scan, allow_scan) {
            Ok((entry, cache_hit)) => {
                state.cached_game_ctrl = entry.game_ctrl;
                state.cached_gps = entry.gps_slot_addr as *const u8;
                state.last_gps_resolve_us = timestamp_us;
                if let Ok(mut sched) = RESOLVER_SCHEDULE.lock() {
                    sched.note_success();
                    if !cache_hit {
                        sched.note_pattern_scan();
                    }
                }
                if cache_hit && state.last_resolve_status != RESOLVE_ROUTE_RESOLVER_CACHE_HIT {
                    record_resolve_status(state, RESOLVE_ROUTE_RESOLVER_CACHE_HIT, timestamp_us);
                }
                Ok(entry)
            }
            Err(err) => {
                state.cached_game_ctrl = 0;
                state.cached_gps = ptr::null();
                let st = if !allow_scan {
                    if let Ok(mut sched) = RESOLVER_SCHEDULE.lock() {
                        sched.note_expensive_failure();
                    }
                    RESOLVE_ROUTE_RESOLVER_SCAN_LIMITED
                } else {
                    if let Ok(mut sched) = RESOLVER_SCHEDULE.lock() {
                        sched.note_pattern_scan();
                        sched.note_expensive_failure();
                    }
                    err.status_code()
                };
                Err(st)
            }
        }
    }

    /// Handler for SCS world events (`started` / `paused` / `unpaused`).
    pub fn on_world_event(event_id: u32, timestamp_us: u64) {
        unsafe {
            let state = &mut *std::ptr::addr_of_mut!(NAV_ROUTE);
            let action = state.world_events.on_event(event_id);
            match action {
                WorldEventAction::Ignore => {}
                WorldEventAction::LogStartedOnce => {
                    state.session_started_us = timestamp_us;
                    SESSION_STARTED_US.store(timestamp_us, std::sync::atomic::Ordering::Relaxed);
                    state.rate_log.event(
                        "started_once",
                        "session started acknowledged (no resolver reset)",
                        timestamp_us,
                        true,
                    );
                }
                WorldEventAction::SoftReset { reason } => {
                    if reason == WORLD_RESET_REASON_PAUSED {
                        state.rate_log.event(
                            "pause_event",
                            "pause event observed raw_id=4",
                            timestamp_us,
                            true,
                        );
                    }
                    if !state.world_reset_gate.try_fire(timestamp_us) {
                        state.world_reset_suppressed_total = state
                            .world_reset_suppressed_total
                            .saturating_add(1);
                        state.suppressed_log.maybe_log(
                            route_status::world_reset_reason_str(reason),
                            state.world_reset_suppressed_total,
                            timestamp_us,
                        );
                        return;
                    }

                    let suppressed = state.world_reset_gate.take_suppressed();
                    state.soft_reset_resolver_cache();
                    invalidate_session_cache();
                    RESOLVER_RESET_REQUESTED.store(true, std::sync::atomic::Ordering::Release);
                    crate::safe_mem::bump_enable_file_generation();
                    if let Ok(mut sched) = RESOLVER_SCHEDULE.lock() {
                        sched.reset_session();
                    }
                    state.world_reset_count = state.world_reset_count.saturating_add(1);
                    state.last_world_reset_reason = reason;
                    state.rate_log.event(
                        "world_reset",
                        &format!(
                            "world event reset reason={} count={}{}",
                            route_status::world_reset_reason_str(reason),
                            state.world_reset_count,
                            if suppressed > 0 {
                                format!(" suppressed_since_last={suppressed}")
                            } else {
                                String::new()
                            }
                        ),
                        timestamp_us,
                        state.world_reset_count <= 3,
                    );
                    write_bb_resolve_diag(state, true);
                }
            }
        }
    }

    /// Track per-frame SCS events (frame_start / frame_end) and callback milestones.
    pub fn on_frame_event(event_id: u32, frame_count: u32, timestamp_us: u64) {
        FRAME_CB_COUNT.store(frame_count, std::sync::atomic::Ordering::Relaxed);
        match event_id {
            SCS_EVENT_FRAME_START => {
                FRAME_START_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            }
            SCS_EVENT_FRAME_END => {
                FRAME_END_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                LAST_FRAME_END_US.store(timestamp_us, std::sync::atomic::Ordering::Relaxed);
            }
            _ => {}
        }
        unsafe {
            let state = &mut *std::ptr::addr_of_mut!(NAV_ROUTE);
            state.frame_cb_count = frame_count;
            state.last_frame_event = event_id;
            if state.frame_milestone.should_log(frame_count) {
                state.rate_log.event(
                    "frame",
                    &format!("frame callback seen count={frame_count}"),
                    timestamp_us,
                    true,
                );
            }
        }
        write_bb_frame_from_atomics();
    }

    /// Legacy wrapper — prefer [`on_frame_event`].
    pub fn on_frame_callback(frame_count: u32, timestamp_us: u64) {
        on_frame_event(SCS_EVENT_FRAME_START, frame_count, timestamp_us);
    }

    pub fn should_tick_on_frame_start(now_us: u64) -> bool {
        should_frame_start_fallback(
            FRAME_END_COUNT.load(std::sync::atomic::Ordering::Relaxed),
            LAST_FRAME_END_US.load(std::sync::atomic::Ordering::Relaxed),
            now_us,
        )
    }

    /// Background resolver walk — only invoked from the worker thread.
    pub fn resolver_walk(timestamp_us: u64) {
        unsafe {
            let state = &mut *std::ptr::addr_of_mut!(NAV_ROUTE);
            sync_atomics_to_state(state);
            state.last_tick_source = resolver_worker::last_tick_source();

            if crate::safe_mem::route_resolver_mode().is_off() {
                handle_resolver_off(state, timestamp_us);
                return;
            }

            if state.last_tick_source == RouteTickSource::FrameStartFallback
                && !state.fallback_logged
            {
                state.fallback_logged = true;
                state.rate_log.event(
                    "tick_fallback",
                    "frame_end missing; enabling frame_start fallback route tick",
                    timestamp_us,
                    true,
                );
            }

            if RESOLVER_RESET_REQUESTED.swap(false, std::sync::atomic::Ordering::AcqRel) {
                invalidate_session_cache();
                state.soft_reset_resolver_cache();
                if let Ok(mut sched) = RESOLVER_SCHEDULE.lock() {
                    sched.reset_session();
                }
                crate::safe_mem::bump_enable_file_generation();
            }

            let mut sched = match RESOLVER_SCHEDULE.lock() {
                Ok(g) => g,
                Err(_) => return,
            };
            sched.note_enable_generation(crate::safe_mem::enable_file_generation());

            match crate::resolver_guard::decide_walk(
                crate::safe_mem::route_resolver_mode(),
                &sched,
                timestamp_us,
            ) {
                crate::resolver_guard::WalkDecision::OffModePark => {
                    drop(sched);
                    handle_resolver_off(state, timestamp_us);
                    return;
                }
                crate::resolver_guard::WalkDecision::ScheduledParked => {
                    if state.last_resolve_status != RESOLVE_ROUTE_RESOLVER_PARKED {
                        record_resolve_status(state, RESOLVE_ROUTE_RESOLVER_PARKED, timestamp_us);
                        write_bb_resolve_diag(state, false);
                    }
                    return;
                }
                crate::resolver_guard::WalkDecision::DiagnosticParked => {
                    write_bb_resolve_diag(state, false);
                    return;
                }
                crate::resolver_guard::WalkDecision::BackoffWait => {
                    if state.last_resolve_status != RESOLVE_ROUTE_RESOLVER_BACKOFF {
                        record_resolve_status(state, RESOLVE_ROUTE_RESOLVER_BACKOFF, timestamp_us);
                        write_bb_resolve_diag(state, false);
                    }
                    return;
                }
                crate::resolver_guard::WalkDecision::Proceed => {}
            }

            sched.note_walk_started(timestamp_us);
            drop(sched);
            crate::resolver_metrics::note_resolver_walk_proceeded();

            if state.tick_milestone.should_log(state.route_tick_count) {
                state.rate_log.event(
                    "tick",
                    &format!(
                        "resolver walk tick_count={} source={} frame_end_count={} frame_start_count={}",
                        state.route_tick_count,
                        state.last_tick_source.as_str(),
                        state.frame_end_event_count,
                        state.frame_start_count,
                    ),
                    timestamp_us,
                    true,
                );
            }

            state.resolve_attempts = state.resolve_attempts.saturating_add(1);
            state.ensure_buffer();
            record_resolve_status(state, RESOLVE_ROUTE_RESOLVER_WORKER_ACTIVE, timestamp_us);

            let gate = evaluate_pause_gate(
                state.world_events.pause_event_seen(),
                state.world_events.pause_state(),
                state.frame_end_event_count,
                state.frame_start_count,
                state.frame_cb_count,
            );
            state.last_pause_gate = gate;

            if gate.block_walk {
                set_bb_route_task_ok(false);
                record_resolve_status(state, RESOLVE_PAUSED_NO_ROUTE_WALK, timestamp_us);
                state.rate_log.event(
                    "paused_skip",
                    &format!(
                        "resolver skipped route walk pause_gate_active=true reason={}",
                        gate.reason
                    ),
                    timestamp_us,
                    true,
                );
                write_bb_resolve_diag(state, true);
                publish_empty_if_needed(state, timestamp_us);
                return;
            }

            if gate.pause_event_seen {
                state.rate_log.event(
                    "pause_ignore",
                    &format!(
                        "pause gate ignored because frames active frame_end_count={} frame_start_count={} reason={}",
                        state.frame_end_event_count,
                        state.frame_start_count,
                        gate.reason
                    ),
                    timestamp_us,
                    state.resolve_attempts <= 3,
                );
                state.rate_log.event(
                    "pause_proceed",
                    "route walk proceeding despite pause event",
                    timestamp_us,
                    state.resolve_attempts <= 3,
                );
            }

            if let Err(block_status) = evaluate_route_scan_gate(
                timestamp_us,
                state.session_started_us,
                state.frame_start_count,
            ) {
                set_bb_route_task_ok(false);
                record_resolve_status(state, block_status, timestamp_us);
                state.rate_log.event(
                    "scan_gate",
                    &format!(
                        "route scan blocked status={}",
                        route_status::resolve_status_str(block_status)
                    ),
                    timestamp_us,
                    state.resolve_attempts <= 5
                        || state.last_resolve_status != block_status,
                );
                write_bb_resolve_diag(state, true);
                publish_empty_if_needed(state, timestamp_us);
                return;
            }

            if !state.scan_policy_logged {
                state.scan_policy_logged = true;
                let mode = crate::safe_mem::route_resolver_mode();
                crate::diag_log::event_force(&format!(
                    "route resolver mode={}",
                    mode.sidecar_label()
                ));
                let policy = crate::safe_mem::route_scan_policy();
                if !mode.is_table_diagnostic() {
                    crate::diag_log::event_force(&format!(
                        "route scan warmup complete deep_route_scan={}",
                        policy.deep_scan
                    ));
                if !policy.deep_scan {
                    crate::diag_log::event_force(
                        "route resolver enabled — deep route scan remains disabled unless full mode",
                    );
                }
                }
            }

            let resolver_mode = crate::safe_mem::route_resolver_mode();

            if matches!(
                resolver_mode,
                crate::safe_mem::RouteResolverMode::GameCtrlTableOnly
                    | crate::safe_mem::RouteResolverMode::RouteCandidateTableOnly
            ) {
                let game_ctrl = match resolve_game_ctrl_for_walk(state, timestamp_us, false) {
                    Ok(entry) => entry.game_ctrl,
                    Err(st) => {
                        set_bb_route_task_ok(false);
                        record_resolve_status(state, st, timestamp_us);
                        write_bb_resolve_diag(state, true);
                        publish_empty_if_needed(state, timestamp_us);
                        return;
                    }
                };
                set_bb_route_task_ok(false);
                let st = if resolver_mode
                    == crate::safe_mem::RouteResolverMode::RouteCandidateTableOnly
                {
                    route_chain::run_route_candidate_table_only_diagnostic(game_ctrl as usize)
                } else {
                    route_chain::run_game_ctrl_table_only_diagnostic(game_ctrl as usize)
                };
                park_diagnostic_after_run(state, st, timestamp_us);
                return;
            }

            let gps = match resolve_game_ctrl_for_walk(state, timestamp_us, false) {
                Ok(entry) => entry.gps_slot_addr as *const u8,
                Err(st) => {
                    state.cached_gps = ptr::null();
                    set_bb_route_task_ok(false);
                    let st = if resolver_mode == crate::safe_mem::RouteResolverMode::GpsTableOnly {
                        RESOLVE_GPS_MANAGER_NOT_RESOLVED
                    } else {
                        st
                    };
                    record_resolve_status(state, st, timestamp_us);
                    write_bb_resolve_diag(state, true);
                    publish_empty_if_needed(state, timestamp_us);
                    return;
                }
            };

            if verify_trip_distance(gps).is_none() {
                state.rate_log.event(
                    "trip_distance",
                    "trip distance low or unavailable (non-blocking)",
                    timestamp_us,
                    state.resolve_attempts <= 3,
                );
            }

            if resolver_mode == crate::safe_mem::RouteResolverMode::GpsTableOnly {
                set_bb_route_task_ok(false);
                let st = route_chain::run_gps_table_only_diagnostic(gps as usize);
                park_diagnostic_after_run(state, st, timestamp_us);
                return;
            }

            let policy = crate::safe_mem::route_scan_policy();
            let mem = LiveMem;
            let chain = if policy.deep_scan {
                route_chain::resolve_route_chain_with_policy(&mem, gps as usize, policy)
            } else {
                route_chain::log_safe_gps_pointer_table(gps as usize);
                Err(route_chain::failure_unsafe_scan_disabled())
            };
            let chain_ok = match chain {
                Ok(ok) => {
                    state.last_chain_candidate = ok.candidate;
                    state.last_chain_failed_step = "";
                    state.last_phys_items_offset = ok.phys_items_offset;
                    ok
                }
                Err(f) => {
                    state.last_chain_candidate = f.candidate;
                    state.last_chain_failed_step = f.failed_step;
                    set_bb_route_task_ok(false);
                    let st = if f.status == RESOLVE_UNSAFE_ROUTE_SCAN_DISABLED {
                        RESOLVE_GPS_RESOLVED_ROUTE_SCAN_DISABLED
                    } else {
                        route_chain::failure_to_status(&f)
                    };
                    record_resolve_status(state, st, timestamp_us);
                    let status_changed = state.last_chain_fail_status != st;
                    state.last_chain_fail_status = st;
                    state.rate_log.event(
                        "chain_fail",
                        &format!(
                            "pointer_chain_failed status={} step={} offset=0x{:X} value=0x{:X} reason={}",
                            route_status::resolve_status_str(st),
                            f.failed_step,
                            f.offset,
                            f.raw_value,
                            f.reason
                        ),
                        timestamp_us,
                        status_changed || state.resolve_attempts <= 3,
                    );
                    write_bb_resolve_diag(state, true);
                    publish_empty_if_needed(state, timestamp_us);
                    return;
                }
            };

            let route_task = chain_ok.route_task as *const u8;
            if route_task.is_null() {
                set_bb_route_task_ok(false);
                record_resolve_status(state, RESOLVE_ROUTE_TASK_PTR_NULL, timestamp_us);
                write_bb_resolve_diag(state, true);
                publish_empty_if_needed(state, timestamp_us);
                return;
            }

            set_bb_route_task_ok(true);
            if !state.last_route_task_ok {
                state.last_route_task_ok = true;
                crate::diag_log::state("route_task", "available");
            }

            let size_field = crate::safe_mem::safe_read_u64(
                route_task as usize + chain_ok.phys_items_offset + 0x08,
            )
            .unwrap_or(0);
            let stop = read_route_waypoints_into(
                route_task,
                chain_ok.phys_items_offset,
                &mut state.waypoint_buf,
            );
            log_walk_warnings(stop, size_field);

            if state.waypoint_buf.is_empty() {
                let st = walk_stop_to_resolve_status(stop);
                record_resolve_status(state, st, timestamp_us);
                write_bb_resolve_diag(state, true);
                publish_empty_if_needed(state, timestamp_us);
                return;
            }

            let first_uid = state.waypoint_buf[0].uid;
            if first_uid == 0 {
                record_resolve_status(state, RESOLVE_FIRST_UID_ZERO, timestamp_us);
                write_bb_resolve_diag(state, true);
                publish_empty_if_needed(state, timestamp_us);
                return;
            }

            record_resolve_status(state, RESOLVE_WAYPOINTS_COLLECTED, timestamp_us);
            state.empty_published = false;
            let hash = uid_sequence_hash_waypoints(&state.waypoint_buf);
            let count = state.waypoint_buf.len();

            if !state.first_success_logged || hash != state.last_hash {
                state.first_success_logged = true;
                state.last_hash = hash;
                state.last_waypoint_count = count;
                crate::diag_log::state("waypoint_count", &count.to_string());
                crate::diag_log::state("route_hash", &format!("{hash:#x}"));
                crate::diag_log::state("route", "active");
                nav_log(&format!(
                    "nav_route: {count} waypoints, first5=[{}], stop={stop:?}, size_field=0x{size_field:X}",
                    format_first5(&state.waypoint_buf),
                ));
                write_route_blackboard(&state.waypoint_buf);
                record_publish_status(state, PUBLISH_ACTIVE, timestamp_us, count, hash);
                write_bb_resolve_diag(state, true);
            } else {
                write_bb_resolve_diag(state, false);
            }
        }
    }

    /// Legacy synchronous entry — unit tests; production uses worker + [`resolver_walk`].
    pub fn tick(timestamp_us: u64, source: RouteTickSource) {
        let _ = source;
        resolver_walk(timestamp_us);
    }

    fn publish_empty_if_needed(state: &mut NavRouteState, timestamp_us: u64) {
        unsafe {
            if !state.empty_published || state.last_hash != 0 {
                state.last_hash = 0;
                state.empty_published = true;
                state.last_waypoint_count = 0;
                write_route_blackboard_invalid();
                record_publish_status(state, PUBLISH_EMPTY, timestamp_us, 0, 0);
                crate::diag_log::state("route", "empty");
                crate::diag_log::state("waypoint_count", "0");
                crate::diag_log::state("route_hash", "0");
            }
        }
    }

    pub fn cleanup_shm() {
        unsafe {
            if !RB_SHM_PTR.is_null() {
                UnmapViewOfFile(RB_SHM_PTR as *mut core::ffi::c_void);
                RB_SHM_PTR = ptr::null_mut();
            }
            if RB_SHM_HANDLE != 0 {
                CloseHandle(RB_SHM_HANDLE);
                RB_SHM_HANDLE = 0;
            }
        }
    }

    /// Seqlock publish into `Local\TruckPilotRouteBlackboard`.
    unsafe fn write_route_blackboard(waypoints: &[RouteWaypoint]) {
        if RB_SHM_PTR.is_null() {
            return;
        }
        let shm = &mut *RB_SHM_PTR;
        let seq = ptr::read_volatile(&shm.sequence);
        let odd = seq.wrapping_add(1) | 1;
        ptr::write_volatile(&mut shm.sequence, odd);
        ptr::write_volatile(&mut shm.valid, 0);

        let truncated = waypoints.len() > MAX_ROUTE_WAYPOINTS;
        let count = waypoints.len().min(MAX_ROUTE_WAYPOINTS);
        let mut flags = 0u32;
        if truncated {
            flags |= ROUTE_FLAG_TRUNCATED;
        }

        for i in count..MAX_ROUTE_WAYPOINTS {
            shm.waypoints[i] = RouteWaypoint::default();
        }
        for (i, wp) in waypoints[..count].iter().enumerate() {
            shm.waypoints[i] = *wp;
        }

        let hash = uid_sequence_hash_waypoints(&waypoints[..count]);
        ptr::write_volatile(&mut shm.waypoint_count, count as u32);
        ptr::write_volatile(&mut shm.flags, flags);
        ptr::write_volatile(&mut shm.route_hash, hash);
        ptr::write_volatile(&mut shm.valid, if count > 0 { 1 } else { 0 });
        RB_SEQUENCE = odd.wrapping_add(1);
        ptr::write_volatile(&mut shm.sequence, RB_SEQUENCE);
    }

    unsafe fn write_route_blackboard_invalid() {
        if RB_SHM_PTR.is_null() {
            return;
        }
        let shm = &mut *RB_SHM_PTR;
        let seq = ptr::read_volatile(&shm.sequence);
        let odd = seq.wrapping_add(1) | 1;
        ptr::write_volatile(&mut shm.sequence, odd);
        ptr::write_volatile(&mut shm.valid, 0);
        ptr::write_volatile(&mut shm.waypoint_count, 0);
        ptr::write_volatile(&mut shm.flags, 0);
        ptr::write_volatile(&mut shm.route_hash, 0);
        RB_SEQUENCE = odd.wrapping_add(1);
        ptr::write_volatile(&mut shm.sequence, RB_SEQUENCE);
    }

    fn nav_log(msg: &str) {
        let s = format!("[TruckPilot] {msg}\0");
        unsafe { OutputDebugStringA(s.as_ptr()) };
    }

    fn nav_warn(msg: &str) {
        nav_log(&format!("WARN nav_route: {msg}"));
    }

    unsafe fn read_u64(addr: *const u8) -> Option<u64> {
        if addr.is_null() {
            return None;
        }
        let v = crate::safe_mem::safe_read_u64(addr as usize).ok()?;
        if looks_like_heap_ptr(v) {
            Some(v)
        } else {
            None
        }
    }

    /// Resolve route_task via multi-candidate chain (see [`crate::route_chain`]).
    pub unsafe fn resolve_route_task(gps: *const u8) -> Option<*const u8> {
        if gps.is_null() {
            return None;
        }
        let mem = LiveMem;
        route_chain::resolve_route_chain(&mem, gps as usize)
            .ok()
            .map(|ok| ok.route_task as *const u8)
    }

    /// Fill `out` with route waypoints (reuses `out` capacity, no fresh allocation).
    pub unsafe fn read_route_waypoints_into(
        route_task: *const u8,
        phys_items_offset: usize,
        out: &mut Vec<RouteWaypoint>,
    ) -> WalkStop {
        out.clear();
        if route_task.is_null() {
            return WalkStop::NoArrayPtr;
        }

        let arr_ptr = match read_u64(route_task.add(phys_items_offset)) {
            Some(p) => p as *const u8,
            None => return WalkStop::NoArrayPtr,
        };

        let size_field =
            std::ptr::read_unaligned(route_task.add(phys_items_offset + 0x08) as *const u64);
        let limit = route_item_limit(size_field);
        let mut scan_stop = WalkStop::MaxCap;

        for i in 0..limit {
            let item = arr_ptr.add(i * ITEM_STRIDE);
            let uid = ptr::read_unaligned(item.add(OFF_ITEM_UID) as *const u64);

            if uid_below_sanity_cap(uid) {
                if i == 0 {
                    return WalkStop::NoItemsAtZero;
                }
                scan_stop = WalkStop::UidSanityCap(i);
                break;
            }

            let dist_left =
                ptr::read_unaligned(item.add(OFF_ITEM_DIST_LEFT) as *const f32);
            out.push(waypoint_from_route_item(uid, dist_left));
        }

        while !out.is_empty() {
            let idx = out.len() - 1;
            let item = arr_ptr.add(idx * ITEM_STRIDE);
            let active = ptr::read_unaligned(item.add(OFF_ITEM_ACTIVE) as *const u32);
            if active != 0 {
                break;
            }
            out.pop();
        }

        if out.is_empty() {
            return WalkStop::NoItemsAtZero;
        }

        match scan_stop {
            WalkStop::MaxCap if out.len() >= limit => WalkStop::MaxCap,
            WalkStop::UidSanityCap(_) | WalkStop::MaxCap => WalkStop::ActiveCount(out.len()),
            other => other,
        }
    }

    fn log_walk_warnings(stop: WalkStop, size_field: u64) {
        match stop {
            WalkStop::MaxCap => nav_warn(&format!(
                "walk hit scan bound ({MAX_ROUTE_ITEMS}) without uid sanity cap; \
                 size_field=0x{size_field:X} — suspect garbage"
            )),
            WalkStop::NoItemsAtZero => nav_warn(&format!(
                "walk empty after trim; size_field=0x{size_field:X} — no readable route items"
            )),
            _ => {}
        }
    }

    fn format_first5(waypoints: &[RouteWaypoint]) -> String {
        waypoints
            .iter()
            .take(5)
            .map(|wp| format!("{}", wp.uid))
            .collect::<Vec<_>>()
            .join(", ")
    }
}

#[cfg(windows)]
#[allow(unused_imports)]
pub use win::{
    cleanup_shm, init_shm, on_frame_callback, on_frame_event, on_world_event,
    publish_initial_empty, resolve_route_task, resolver_walk, should_tick_on_frame_start,
    tick, write_bb_frame_from_atomics,
};

#[cfg(not(windows))]
pub fn publish_initial_empty() {}

#[cfg(not(windows))]
pub fn on_world_event(_event_id: u32, _timestamp_us: u64) {}

#[cfg(not(windows))]
pub fn on_frame_callback(_frame_count: u32, _timestamp_us: u64) {}

#[cfg(not(windows))]
pub fn on_frame_event(_event_id: u32, _frame_count: u32, _timestamp_us: u64) {}

#[cfg(not(windows))]
pub fn should_tick_on_frame_start(_now_us: u64) -> bool {
    false
}

#[cfg(not(windows))]
pub fn tick(_timestamp_us: u64, _source: RouteTickSource) {}

#[cfg(not(windows))]
pub fn resolver_walk(_timestamp_us: u64) {}

#[cfg(not(windows))]
pub fn write_bb_frame_from_atomics() {}

#[cfg(not(windows))]
pub fn init_shm() -> bool {
    false
}

#[cfg(not(windows))]
pub fn cleanup_shm() {}

#[cfg(not(windows))]
pub unsafe fn resolve_route_task(_gps: *const u8) -> Option<*const u8> {
    None
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn route_item_limit_prefers_size_field() {
        assert_eq!(route_item_limit(37), 37);
        assert_eq!(route_item_limit(0), MAX_ROUTE_ITEMS);
        assert_eq!(route_item_limit(99999), MAX_ROUTE_ITEMS);
    }

    #[test]
    fn uid_sanity_cap_matches_python() {
        assert!(uid_below_sanity_cap(0));
        assert!(uid_below_sanity_cap(UID_MIN_PLAUSIBLE - 1));
        assert!(!uid_below_sanity_cap(UID_MIN_PLAUSIBLE));
        assert!(!uid_below_sanity_cap(6_282_842_151_886_729_779));
    }

    #[test]
    fn trim_trailing_inactive_only_from_tail() {
        let active = [true, true, false, true, false, false];
        let n = trim_trailing_inactive_count(active.len(), |i| active[i]);
        assert_eq!(n, 4);
    }

    #[test]
    fn fnv_hash_stable() {
        let a = [1_u64, 2, 3];
        let b = [1_u64, 2, 3];
        assert_eq!(uid_sequence_hash(&a), uid_sequence_hash(&b));
        assert_ne!(uid_sequence_hash(&a), uid_sequence_hash(&[1, 2, 4]));
    }

    #[test]
    fn route_bb_status_constants_match_dll() {
        assert_eq!(ROUTE_BB_STATUS_DLL_ACTIVE, 1);
        assert_eq!(ROUTE_BB_STATUS_ROUTE_TASK_OK, 1 << 1);
    }

    #[test]
    fn route_blackboard_layout_sizes() {
        use std::mem;
        assert_eq!(mem::size_of::<RouteWaypoint>(), 32);
        assert_eq!(mem::offset_of!(RouteBlackboard, waypoints), 64);
        assert_eq!(
            mem::size_of::<RouteBlackboard>(),
            64 + MAX_ROUTE_WAYPOINTS * 32
        );
        assert_eq!(ROUTE_MAGIC, 0x4252_5054);
        assert_eq!(ROUTE_VERSION, 1);
    }

    #[test]
    fn route_hash_changes_with_uids() {
        assert_ne!(
            uid_sequence_hash(&[1, 2, 3]),
            uid_sequence_hash(&[1, 2, 4])
        );
    }

    #[test]
    fn waypoint_from_item_sets_distance_flag() {
        let wp = waypoint_from_route_item(6_282_842_151_886_729_779, 1234.5);
        assert_eq!(wp.uid, 6_282_842_151_886_729_779);
        assert!((wp.distance - 1234.5).abs() < 0.01);
        assert_ne!(wp.flags & ROUTE_WP_FLAG_HAS_DISTANCE, 0);
        assert_ne!(wp.flags & ROUTE_WP_FLAG_UNTRUSTED, 0);
        assert_eq!(wp.flags & ROUTE_WP_FLAG_HAS_POSITION, 0);
    }

    #[test]
    fn waypoint_from_item_rejects_bad_distance() {
        let wp = waypoint_from_route_item(1, f32::NAN);
        assert_eq!(wp.distance, 0.0);
        assert_eq!(wp.flags, 0);
    }

    #[test]
    fn distance_left_plausible_bounds() {
        assert!(distance_left_plausible(0.0));
        assert!(distance_left_plausible(690_170.4));
        assert!(!distance_left_plausible(-1.0));
        assert!(!distance_left_plausible(f32::NAN));
    }

    #[test]
    fn publish_initial_empty_noop_without_shm() {
        publish_initial_empty();
    }

    #[test]
    fn cleanup_shm_idempotent_on_stub() {
        cleanup_shm();
        cleanup_shm();
    }
}
