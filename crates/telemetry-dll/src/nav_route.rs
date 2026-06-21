//! In-process navigation route UID walk for ETS2 1.59 (Phase R2/R3).
//!
//! ## Route end detection (R3 Teil 1, empiric 1.59.1.3s)
//! - **No reliable `array_dyn.size`** at `route_task+0x58` (second pointer, not count).
//! - **Scan cap:** read items while `uid >= 5e18` (sanity — DLC UID ranges may differ).
//! - **True end:** trim trailing slots where `physical_route_item+0x0C == 0` (inactive tail
//!   padding before the sub-threshold garbage UID). Stable vs graph.json gold on long routes.
//! - **Known risk:** `5e18` alone caused 1085/1086 wobble; `+0x0C` tail-trim fixes it.
//!   Mid-array `+0x0C==0` slots with valid UIDs are kept (not tail).

#![allow(clippy::cast_possible_wrap)]

use crate::nav_resolve::{resolve_gps_manager, verify_trip_distance};

// ---------------------------------------------------------------------------
// Nav-route SHM constants — must match `crates/telemetry/src/nav_route.rs`.
// ---------------------------------------------------------------------------

pub const NAV_ROUTE_SHM_MAGIC: u32 = 0x54504E52; // "TPNR"
pub const NAV_ROUTE_SHM_VERSION: u32 = 1;
pub const NAV_ROUTE_SHM_NAME: &str = "Local\\TruckPilotNavRoute";
/// Maximum route items stored in SHM. Covers routes up to ~2000 road segments.
pub const NAV_ROUTE_MAX_ITEMS: usize = 2048;

/// Route SHM layout — DLL writes, Core reads on every route change.
/// Must stay byte-for-byte identical to `NavRouteShmLayout` in
/// `crates/telemetry/src/nav_route.rs`.
#[repr(C)]
pub struct NavRouteShmLayout {
    pub magic: u32,       // NAV_ROUTE_SHM_MAGIC
    pub version: u32,     // NAV_ROUTE_SHM_VERSION
    pub sequence: u32,    // increments on every route change
    pub item_count: u32,  // number of valid UIDs in `items`
    pub items: [u64; NAV_ROUTE_MAX_ITEMS],
}

// Compile-time layout guard.
const _: () = {
    use std::mem;
    assert!(mem::offset_of!(NavRouteShmLayout, magic) == 0);
    assert!(mem::offset_of!(NavRouteShmLayout, version) == 4);
    assert!(mem::offset_of!(NavRouteShmLayout, sequence) == 8);
    assert!(mem::offset_of!(NavRouteShmLayout, item_count) == 12);
    assert!(mem::offset_of!(NavRouteShmLayout, items) == 16);
};

// --- 1.59 offsets (verified, see outputs/nav_offsets_1_59.md) ----------------

const OFF_SIMPLE_ROUTE_SRC: usize = 0x08;
const OFF_SRS_ROUTE_A: usize = 0x58;
const OFF_ROUTE_B_SLOT: usize = 0x2C0;
const OFF_ROUTE_TASK_REF: usize = 0x1A8;
const OFF_ROUTE_TASK_BIAS: usize = 0x18;
const OFF_PHYS_ITEMS: usize = 0x50;
const OFF_PHYS_ITEMS_SIZE: usize = OFF_PHYS_ITEMS + 0x08;
const ITEM_STRIDE: usize = 0x40;
const OFF_ITEM_UID: usize = 0x30;
/// Inactive / padding tail slots: dword at +0x0C is 0 (active slots non-zero).
const OFF_ITEM_ACTIVE: usize = 0x0C;

const MAX_ROUTE_ITEMS: usize = 4000;
const SIZE_FIELD_MAX: u64 = 6000;

/// UID below this is garbage past the real route tail (sanity cap only, not primary end).
const UID_MIN_PLAUSIBLE: u64 = 5_000_000_000_000_000_000;

const NAV_ROUTE_MIN_FRAMES: u32 = 30;
const NAV_ROUTE_MIN_INTERVAL_US: u64 = 500_000;

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

    static mut NR_SHM_HANDLE: isize = 0;
    static mut NR_SHM_PTR: *mut NavRouteShmLayout = ptr::null_mut();
    static mut NR_SEQUENCE: u32 = 0;

    fn wide_nr(s: &str) -> Vec<u16> {
        s.encode_utf16().chain(std::iter::once(0)).collect()
    }

    pub fn init_shm() -> bool {
        unsafe {
            let name = wide_nr(NAV_ROUTE_SHM_NAME);
            let size = std::mem::size_of::<NavRouteShmLayout>();
            let h = CreateFileMappingW(
                INVALID_FILE_HANDLE,
                ptr::null(),
                PAGE_READWRITE,
                0,
                size as u32,
                name.as_ptr(),
            );
            if h == 0 {
                nav_warn("init_shm: CreateFileMappingW failed");
                return false;
            }
            let raw = MapViewOfFile(h, FILE_MAP_WRITE, 0, 0, size);
            if raw.is_null() {
                CloseHandle(h);
                nav_warn("init_shm: MapViewOfFile failed");
                return false;
            }
            let shm = raw as *mut NavRouteShmLayout;
            (*shm).magic = NAV_ROUTE_SHM_MAGIC;
            (*shm).version = NAV_ROUTE_SHM_VERSION;
            (*shm).sequence = 0;
            (*shm).item_count = 0;
            NR_SHM_HANDLE = h;
            NR_SHM_PTR = shm;
            nav_log(&format!("nav_route SHM ready ({size} bytes)"));
            true
        }
    }

    pub fn cleanup_shm() {
        unsafe {
            if !NR_SHM_PTR.is_null() {
                UnmapViewOfFile(NR_SHM_PTR as *mut core::ffi::c_void);
                NR_SHM_PTR = ptr::null_mut();
            }
            if NR_SHM_HANDLE != 0 {
                CloseHandle(NR_SHM_HANDLE);
                NR_SHM_HANDLE = 0;
            }
        }
    }

    /// Write a diagnostic step code into `sequence` when item_count stays 0.
    /// The reader exposes this via `peek_diag_code()`.
    unsafe fn write_diag_to_shm(code: u32) {
        if NR_SHM_PTR.is_null() {
            return;
        }
        let shm = &mut *NR_SHM_PTR;
        // Only overwrite when there is no real route to avoid clobbering real data.
        if ptr::read_volatile(&shm.item_count) == 0 {
            ptr::write_volatile(&mut shm.sequence, code);
        }
    }

    unsafe fn write_route_to_shm(uids: &[u64]) {
        if NR_SHM_PTR.is_null() {
            return;
        }
        let shm = &mut *NR_SHM_PTR;
        let count = uids.len().min(NAV_ROUTE_MAX_ITEMS);
        // Write items first, then publish count + sequence atomically last.
        for (i, &uid) in uids[..count].iter().enumerate() {
            shm.items[i] = uid;
        }
        NR_SEQUENCE = NR_SEQUENCE.wrapping_add(1);
        ptr::write_volatile(&mut shm.item_count, count as u32);
        ptr::write_volatile(&mut shm.sequence, NR_SEQUENCE);
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
        let v = ptr::read_unaligned(addr as *const u64);
        if looks_like_heap_ptr(v) { Some(v) } else { None }
    }

    /// Three-stage chain: gps → srs* → +0x58 → +0x2C0 → +0x1A8 (+0x18).
    pub unsafe fn resolve_route_task(gps: *const u8) -> Option<*const u8> {
        if gps.is_null() {
            return None;
        }
        let srs = read_u64(gps.add(OFF_SIMPLE_ROUTE_SRC))?;
        let a = read_u64((srs as *const u8).add(OFF_SRS_ROUTE_A))?;
        let b = read_u64((a as *const u8).add(OFF_ROUTE_B_SLOT))?;
        let rt_ref = read_u64((b as *const u8).add(OFF_ROUTE_TASK_REF))?;
        Some((rt_ref as *const u8).add(OFF_ROUTE_TASK_BIAS))
    }

    /// Fill `out` with route UIDs (reuses `out` capacity, no fresh allocation).
    pub unsafe fn read_route_uids_into(
        route_task: *const u8,
        out: &mut Vec<u64>,
    ) -> WalkStop {
        out.clear();
        if route_task.is_null() {
            return WalkStop::NoArrayPtr;
        }

        let arr_ptr = match read_u64(route_task.add(OFF_PHYS_ITEMS)) {
            Some(p) => p as *const u8,
            None => return WalkStop::NoArrayPtr,
        };

        let size_field = ptr::read_unaligned(route_task.add(OFF_PHYS_ITEMS_SIZE) as *const u64);
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

            out.push(uid);
        }

        // Trim inactive tail padding (+0x0C == 0); mid-route inactive slots are not tail.
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

    fn format_first5(uids: &[u64]) -> String {
        uids.iter()
            .take(5)
            .map(|u| format!("{u}"))
            .collect::<Vec<_>>()
            .join(", ")
    }

    struct NavRouteState {
        cached_gps: *const u8,
        uid_buf: Vec<u64>,
        last_hash: u64,
        last_walk_us: u64,
        frames_since_walk: u32,
        first_success_logged: bool,
        buffer_ready: bool,
    }

    impl NavRouteState {
        const fn empty() -> Self {
            Self {
                cached_gps: std::ptr::null(),
                uid_buf: Vec::new(),
                last_hash: 0,
                last_walk_us: 0,
                frames_since_walk: 0,
                first_success_logged: false,
                buffer_ready: false,
            }
        }

        fn ensure_buffer(&mut self) {
            if !self.buffer_ready {
                self.uid_buf.reserve(MAX_ROUTE_ITEMS);
                self.buffer_ready = true;
            }
        }
    }

    static mut NAV_ROUTE: NavRouteState = NavRouteState::empty();

    /// Throttled frame-end tick — call **after** SHM flush + `SetEvent`.
    pub fn tick(timestamp_us: u64) {
        unsafe {
            let state = &mut *std::ptr::addr_of_mut!(NAV_ROUTE);
            state.frames_since_walk = state.frames_since_walk.saturating_add(1);

            let elapsed = timestamp_us.saturating_sub(state.last_walk_us);
            if state.frames_since_walk < NAV_ROUTE_MIN_FRAMES
                && elapsed < NAV_ROUTE_MIN_INTERVAL_US
            {
                return;
            }

            state.frames_since_walk = 0;
            state.last_walk_us = timestamp_us;
            state.ensure_buffer();

            let gps = if !state.cached_gps.is_null() {
                state.cached_gps
            } else {
                match resolve_gps_manager() {
                    Some(g) => {
                        state.cached_gps = g;
                        g
                    }
                    None => {
                        write_diag_to_shm(0xD1A6_0001); // step 1 = GPS AOB scan failed
                        return;
                    }
                }
            };

            // Step 2: trip distance check — write diag code if it fails.
            if verify_trip_distance(gps).is_none() {
                write_diag_to_shm(0xD1A6_0002); // step 2 = trip_dist gate
                return;
            }

            let route_task = match resolve_route_task(gps) {
                Some(rt) => rt,
                None => {
                    write_diag_to_shm(0xD1A6_0003); // step 3 = route_task failed
                    return;
                }
            };

            let size_field =
                std::ptr::read_unaligned(route_task.add(OFF_PHYS_ITEMS_SIZE) as *const u64);
            let stop = read_route_uids_into(route_task, &mut state.uid_buf);
            log_walk_warnings(stop, size_field);

            if state.uid_buf.is_empty() {
                write_diag_to_shm(0xD1A6_0004); // step 4 = uid_buf empty after walk
                return;
            }

            let hash = uid_sequence_hash(&state.uid_buf);
            if !state.first_success_logged {
                state.first_success_logged = true;
                state.last_hash = hash;
                nav_log(&format!(
                    "nav_route: {} UIDs, first5=[{}], stop={stop:?}, size_field=0x{size_field:X}",
                    state.uid_buf.len(),
                    format_first5(&state.uid_buf),
                ));
                // Seed SHM with the initial route so Core gets it immediately.
                write_route_to_shm(&state.uid_buf);
                return;
            }

            if hash != state.last_hash {
                state.last_hash = hash;
                nav_log(&format!(
                    "nav_route: route changed {} UIDs, first5=[{}], stop={stop:?}",
                    state.uid_buf.len(),
                    format_first5(&state.uid_buf),
                ));
                write_route_to_shm(&state.uid_buf);
            }
        }
    }
}

#[cfg(windows)]
#[allow(unused_imports)] // public API for R3 / diagnostics
pub use win::{cleanup_shm, init_shm, resolve_route_task, tick};

#[cfg(not(windows))]
pub fn tick(_timestamp_us: u64) {}

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
    fn trip_distance_gate_unchanged() {
        use crate::nav_resolve::trip_distance_plausible;
        assert!(trip_distance_plausible(690_170.4));
        assert!(!trip_distance_plausible(0.0));
    }
}
