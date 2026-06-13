//! In-process navigation route UID walk for ETS2 1.59 (Phase R2).
//!
//! Port of `scripts/truckpilot_nav_read.py` pointer chain + UID scan.
//!
//! ## Known risk: UID termination heuristic
//! Walk stops when `uid == 0` or `uid < 5e18`. That threshold mirrors the Python
//! diagnostic tool and assumes vanilla 1.59 node UID layout. DLC / ProMods may use
//! different UID ranges — we may stop early or read garbage. R3 may replace this
//! with a versioned terminator once more game versions are profiled.

#![allow(clippy::cast_possible_wrap)]

use crate::nav_resolve::{resolve_gps_manager, verify_trip_distance};

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

const MAX_ROUTE_ITEMS: usize = 4000;
const SIZE_FIELD_MAX: u64 = 6000;

/// Python parity: `uid < 5_000_000_000_000_000_000` ends the walk (fragile heuristic).
const UID_MIN_PLAUSIBLE: u64 = 5_000_000_000_000_000_000;

const NAV_ROUTE_MIN_FRAMES: u32 = 30;
const NAV_ROUTE_MIN_INTERVAL_US: u64 = 500_000;

// ---------------------------------------------------------------------------
// Pure helpers (unit-testable)
// ---------------------------------------------------------------------------

fn looks_like_heap_ptr(v: u64) -> bool {
    (0x1_0000..0x0007_FFFF_FFFF_FFFF).contains(&v)
}

fn uid_terminates(uid: u64) -> bool {
    uid == 0 || uid < UID_MIN_PLAUSIBLE
}

/// Primary loop bound: use `array_dyn.size` when plausible, else scan cap.
pub fn route_item_limit(size_field: u64) -> (usize, bool) {
    if (1..=SIZE_FIELD_MAX).contains(&size_field) {
        ((size_field as usize).min(MAX_ROUTE_ITEMS), true)
    } else {
        (MAX_ROUTE_ITEMS, false)
    }
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WalkStop {
    /// `array_dyn.size` was in 1..=6000 and defined the loop bound.
    SizeField(usize),
    /// Stopped at index `i` because uid failed the heuristic (route end).
    UidHeuristic(usize),
    /// Filled `MAX_ROUTE_ITEMS` without a clean terminator — suspect garbage loop.
    MaxCap,
    /// First item already failed uid check — no route items readable.
    NoItemsAtZero,
    /// `physical_route_items.ptr` missing or invalid.
    NoArrayPtr,
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
        let (limit, used_size) = route_item_limit(size_field);

        for i in 0..limit {
            let item = arr_ptr.add(i * ITEM_STRIDE);
            let uid = ptr::read_unaligned(item.add(OFF_ITEM_UID) as *const u64);

            if uid_terminates(uid) {
                if i == 0 {
                    return WalkStop::NoItemsAtZero;
                }
                return WalkStop::UidHeuristic(i);
            }

            out.push(uid);
        }

        if used_size {
            WalkStop::SizeField(limit)
        } else {
            WalkStop::MaxCap
        }
    }

    fn log_walk_warnings(stop: WalkStop, size_field: u64, used_size: bool) {
        match stop {
            WalkStop::MaxCap => nav_warn(&format!(
                "walk hit MAX_ROUTE_ITEMS ({MAX_ROUTE_ITEMS}) without uid terminator; \
                 size_field=0x{size_field:X} used_size={used_size} — suspect garbage"
            )),
            WalkStop::NoItemsAtZero => nav_warn(&format!(
                "walk aborted at item 0 (uid heuristic); size_field=0x{size_field:X} \
                 used_size={used_size} — no readable route items"
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
                    None => return,
                }
            };

            if verify_trip_distance(gps).is_none() {
                return;
            }

            let route_task = match resolve_route_task(gps) {
                Some(rt) => rt,
                None => return,
            };

            let size_field =
                std::ptr::read_unaligned(route_task.add(OFF_PHYS_ITEMS_SIZE) as *const u64);
            let stop = read_route_uids_into(route_task, &mut state.uid_buf);
            log_walk_warnings(stop, size_field, route_item_limit(size_field).1);

            if state.uid_buf.is_empty() {
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
                return;
            }

            if hash != state.last_hash {
                state.last_hash = hash;
                nav_log(&format!(
                    "nav_route: route changed {} UIDs, first5=[{}], stop={stop:?}",
                    state.uid_buf.len(),
                    format_first5(&state.uid_buf),
                ));
            }
        }
    }
}

#[cfg(windows)]
#[allow(unused_imports)] // public API for R3 / diagnostics
pub use win::{resolve_route_task, tick};

#[cfg(not(windows))]
pub fn tick(_timestamp_us: u64) {}

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
        assert_eq!(route_item_limit(37), (37, true));
        assert_eq!(route_item_limit(0), (MAX_ROUTE_ITEMS, false));
        assert_eq!(route_item_limit(99999), (MAX_ROUTE_ITEMS, false));
    }

    #[test]
    fn uid_terminates_matches_python() {
        assert!(uid_terminates(0));
        assert!(uid_terminates(UID_MIN_PLAUSIBLE - 1));
        assert!(!uid_terminates(UID_MIN_PLAUSIBLE));
        assert!(!uid_terminates(6_282_842_151_886_729_779));
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
