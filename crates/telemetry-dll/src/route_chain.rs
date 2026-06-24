//! GPS → route_task → route_items pointer chains (ETS2 1.59+).
//!
//! # Verified item layout (1.59)
//!
//! ```text
//! gps + 0x08  ->  simple_route_src*
//! srs + 0x58  ->  route_a*
//! a   + 0x2C0 ->  route_b*        (often null on short routes / menu)
//! b   + 0x1A8 ->  route_task_ref*
//! route_task = ref + 0x18
//! route_task + 0x50 -> physical_route_items[] (alt +0x58)
//! item + 0x30 -> UID
//! item + 0x0C -> active tail trim
//! item + 0x14 -> distance (untrusted)
//! ```

use crate::nav_resolve::GPS_OFFSET_IN_GAME_CTRL;
use crate::route_status::{
    RateLog, RESOLVE_FIRST_UID_ZERO, RESOLVE_GAME_CTRL_TABLE_ONLY_DONE,
    RESOLVE_GAME_CTRL_TABLE_READ_FAILED, RESOLVE_GPS_OFFSET_PROBE_DONE,
    RESOLVE_GPS_OFFSET_PROBE_READ_FAILED, RESOLVE_GPS_TABLE_ONLY_DONE,
    RESOLVE_GPS_TABLE_READ_FAILED,
    RESOLVE_ROUTE_CANDIDATE_TABLE_DONE, RESOLVE_ROUTE_CANDIDATE_TABLE_READ_FAILED,
    RESOLVE_ROUTE_CHAIN_ALL_FAILED, RESOLVE_ROUTE_CHAIN_CANDIDATE_FAILED,
    RESOLVE_ROUTE_ITEMS_CANDIDATE_NULL, RESOLVE_ROUTE_ITEMS_EMPTY,
    RESOLVE_ROUTE_TASK_CANDIDATE_NULL, RESOLVE_SIMPLE_ROUTE_SRC_NULL,
    RESOLVE_SIMPLE_ROUTE_SRC_OFFSET_UNKNOWN, RESOLVE_SRS_OFFSET_SCAN_FAILED,
    RESOLVE_UNSAFE_ROUTE_SCAN_DISABLED,
};
use crate::safe_mem::{self, RouteScanPolicy, GPS_TABLE_SAFE_END, GPS_TABLE_SLOT_COUNT};

pub const OFF_SIMPLE_ROUTE_SRC: usize = 0x08;
pub const OFF_SRS_ROUTE_A: usize = 0x58;
pub const OFF_ROUTE_B_SLOT: usize = 0x2C0;
pub const OFF_ROUTE_TASK_REF: usize = 0x1A8;
pub const OFF_ROUTE_TASK_BIAS: usize = 0x18;
pub const OFF_PHYS_ITEMS: usize = 0x50;
pub const OFF_PHYS_ITEMS_ALT: usize = 0x58;
pub const OFF_PHYS_ITEMS_ALT2: usize = 0x48;
pub const OFF_PHYS_ITEMS_SIZE: usize = OFF_PHYS_ITEMS + 0x08;
pub const ITEM_STRIDE: usize = 0x40;
pub const OFF_ITEM_UID: usize = 0x30;
pub const OFF_ITEM_ACTIVE: usize = 0x0C;
pub const OFF_ITEM_DIST_LEFT: usize = 0x14;

const UID_MIN_PLAUSIBLE: u64 = 5_000_000_000_000_000_000;
const MAX_PREVIEW_ITEMS: usize = 32;
const A_WINDOW_START: usize = 0x200;
const A_WINDOW_END: usize = 0x400;

/// GPS pointer table dump range for deep-scan diagnostics only.
pub const GPS_PTR_TABLE_END: usize = 0x200;
/// Direct route_task / items scan range (`gps + 0x00 ..= 0x400`).
pub const GPS_DIRECT_SCAN_END: usize = 0x400;
/// Dynamic simple_route_src offset scan range.
pub const SRS_OFFSET_SCAN_END: usize = 0x200;

/// Offsets always printed in the GPS table (including explicit null).
pub const GPS_TABLE_KNOWN_OFFSETS: &[usize] = &[
    0x00, 0x08, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38, 0x40, 0x48, 0x50, 0x58, 0x60, 0x68,
];

/// Route items pointer candidates on a route_task object.
pub const DIRECT_ITEMS_OFFSETS: &[usize] = &[0x50, 0x58, 0x60, 0x68];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HopBase {
    Gps,
    Prev,
}

#[derive(Debug, Clone, Copy)]
pub struct ChainHop {
    pub name: &'static str,
    pub base: HopBase,
    pub offset: usize,
}

#[derive(Debug, Clone, Copy)]
pub struct RouteChainCandidate {
    pub name: &'static str,
    pub hops: &'static [ChainHop],
    pub route_task_bias: usize,
    pub phys_items_offset: usize,
}

/// Static chain candidates — dynamic `a_slot_window` runs after these when `route_a` is known.
pub const CHAIN_CANDIDATES: &[RouteChainCandidate] = &[
    RouteChainCandidate {
        name: "v159_long",
        hops: &[
            ChainHop {
                name: "simple_route_src",
                base: HopBase::Gps,
                offset: OFF_SIMPLE_ROUTE_SRC,
            },
            ChainHop {
                name: "route_a",
                base: HopBase::Prev,
                offset: OFF_SRS_ROUTE_A,
            },
            ChainHop {
                name: "route_b",
                base: HopBase::Prev,
                offset: OFF_ROUTE_B_SLOT,
            },
            ChainHop {
                name: "route_task_ref",
                base: HopBase::Prev,
                offset: OFF_ROUTE_TASK_REF,
            },
        ],
        route_task_bias: OFF_ROUTE_TASK_BIAS,
        phys_items_offset: OFF_PHYS_ITEMS,
    },
    RouteChainCandidate {
        name: "v159_skip_b",
        hops: &[
            ChainHop {
                name: "simple_route_src",
                base: HopBase::Gps,
                offset: OFF_SIMPLE_ROUTE_SRC,
            },
            ChainHop {
                name: "route_a",
                base: HopBase::Prev,
                offset: OFF_SRS_ROUTE_A,
            },
            ChainHop {
                name: "route_task_ref",
                base: HopBase::Prev,
                offset: OFF_ROUTE_TASK_REF,
            },
        ],
        route_task_bias: OFF_ROUTE_TASK_BIAS,
        phys_items_offset: OFF_PHYS_ITEMS,
    },
    RouteChainCandidate {
        name: "v159_long_bias0",
        hops: &[
            ChainHop {
                name: "simple_route_src",
                base: HopBase::Gps,
                offset: OFF_SIMPLE_ROUTE_SRC,
            },
            ChainHop {
                name: "route_a",
                base: HopBase::Prev,
                offset: OFF_SRS_ROUTE_A,
            },
            ChainHop {
                name: "route_b",
                base: HopBase::Prev,
                offset: OFF_ROUTE_B_SLOT,
            },
            ChainHop {
                name: "route_task_ref",
                base: HopBase::Prev,
                offset: OFF_ROUTE_TASK_REF,
            },
        ],
        route_task_bias: 0,
        phys_items_offset: OFF_PHYS_ITEMS,
    },
    RouteChainCandidate {
        name: "v159_srs_rt20",
        hops: &[
            ChainHop {
                name: "simple_route_src",
                base: HopBase::Gps,
                offset: OFF_SIMPLE_ROUTE_SRC,
            },
            ChainHop {
                name: "route_task_direct",
                base: HopBase::Prev,
                offset: 0x20,
            },
        ],
        route_task_bias: 0,
        phys_items_offset: OFF_PHYS_ITEMS,
    },
    RouteChainCandidate {
        name: "v159_srs_rt18",
        hops: &[
            ChainHop {
                name: "simple_route_src",
                base: HopBase::Gps,
                offset: OFF_SIMPLE_ROUTE_SRC,
            },
            ChainHop {
                name: "route_task_direct",
                base: HopBase::Prev,
                offset: 0x18,
            },
        ],
        route_task_bias: 0,
        phys_items_offset: OFF_PHYS_ITEMS,
    },
    RouteChainCandidate {
        name: "v159_long_items58",
        hops: &[
            ChainHop {
                name: "simple_route_src",
                base: HopBase::Gps,
                offset: OFF_SIMPLE_ROUTE_SRC,
            },
            ChainHop {
                name: "route_a",
                base: HopBase::Prev,
                offset: OFF_SRS_ROUTE_A,
            },
            ChainHop {
                name: "route_b",
                base: HopBase::Prev,
                offset: OFF_ROUTE_B_SLOT,
            },
            ChainHop {
                name: "route_task_ref",
                base: HopBase::Prev,
                offset: OFF_ROUTE_TASK_REF,
            },
        ],
        route_task_bias: OFF_ROUTE_TASK_BIAS,
        phys_items_offset: OFF_PHYS_ITEMS_ALT,
    },
];

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChainStepLog {
    pub name: &'static str,
    pub offset: usize,
    pub value: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RouteChainSuccess {
    pub candidate: &'static str,
    pub route_task: usize,
    pub phys_items_offset: usize,
    pub steps: Vec<ChainStepLog>,
    pub first_uid: u64,
    pub item_count: usize,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RouteChainFailure {
    pub status: u32,
    pub candidate: &'static str,
    pub failed_step: &'static str,
    pub offset: usize,
    pub raw_value: u64,
    pub reason: &'static str,
    pub steps: Vec<ChainStepLog>,
}

pub fn looks_like_heap_ptr(v: u64) -> bool {
    (0x1_0000..0x0007_FFFF_FFFF_FFFF).contains(&v)
}

pub fn uid_plausible(uid: u64) -> bool {
    uid != 0 && uid >= UID_MIN_PLAUSIBLE
}

pub trait MemRead {
    fn read_u64(&self, addr: usize) -> Option<u64>;
    fn read_u32(&self, addr: usize) -> Option<u32>;
}

#[cfg(windows)]
pub struct LiveMem;

#[cfg(windows)]
impl MemRead for LiveMem {
    fn read_u64(&self, addr: usize) -> Option<u64> {
        safe_mem::safe_read_u64(addr).ok()
    }

    fn read_u32(&self, addr: usize) -> Option<u32> {
        safe_mem::safe_read_u32(addr).ok()
    }
}

#[derive(Debug, Clone, Default)]
pub struct FakeMem {
    pub words: std::collections::HashMap<usize, u64>,
}

impl FakeMem {
    pub fn set(&mut self, addr: usize, value: u64) {
        self.words.insert(addr, value);
    }
}

impl MemRead for FakeMem {
    fn read_u64(&self, addr: usize) -> Option<u64> {
        self.words.get(&addr).copied()
    }

    fn read_u32(&self, addr: usize) -> Option<u32> {
        self.read_u64(addr).map(|v| v as u32)
    }
}

fn read_ptr<R: MemRead>(mem: &R, addr: usize) -> Option<u64> {
    let v = mem.read_u64(addr)?;
    if looks_like_heap_ptr(v) {
        Some(v)
    } else {
        None
    }
}

fn count_route_items<R: MemRead>(mem: &R, arr_ptr: usize) -> (usize, u64) {
    let mut count = 0usize;
    let mut first_uid = 0u64;
    for i in 0..MAX_PREVIEW_ITEMS {
        let item = arr_ptr + i * ITEM_STRIDE;
        let uid = mem.read_u64(item + OFF_ITEM_UID).unwrap_or(0);
        if !uid_plausible(uid) {
            break;
        }
        if count == 0 {
            first_uid = uid;
        }
        count += 1;
        let active = mem.read_u32(item + OFF_ITEM_ACTIVE).unwrap_or(0);
        if active == 0 && i > 0 {
            break;
        }
    }
    (count, first_uid)
}

fn validate_route_task<R: MemRead>(
    mem: &R,
    route_task: usize,
    primary_phys: usize,
) -> Result<(usize, u64, usize), u32> {
    if route_task == 0 {
        return Err(RESOLVE_ROUTE_TASK_CANDIDATE_NULL);
    }
    for &phys_off in &[primary_phys, OFF_PHYS_ITEMS, OFF_PHYS_ITEMS_ALT, OFF_PHYS_ITEMS_ALT2] {
        let arr_ptr = match read_ptr(mem, route_task + phys_off) {
            Some(p) => p as usize,
            None => continue,
        };
        let (count, first_uid) = count_route_items(mem, arr_ptr);
        let uid0 = mem.read_u64(arr_ptr + OFF_ITEM_UID).unwrap_or(0);
        if uid0 == 0 {
            return Err(RESOLVE_FIRST_UID_ZERO);
        }
        if first_uid == 0 || count == 0 {
            continue;
        }
        return Ok((arr_ptr, first_uid, phys_off));
    }
    Err(RESOLVE_ROUTE_ITEMS_CANDIDATE_NULL)
}

fn walk_hops<R: MemRead>(
    mem: &R,
    gps: usize,
    cand: &RouteChainCandidate,
) -> Result<(usize, Vec<ChainStepLog>), RouteChainFailure> {
    let mut steps = Vec::new();
    let mut cur = gps;
    for hop in cand.hops {
        let addr = match hop.base {
            HopBase::Gps => gps + hop.offset,
            HopBase::Prev => cur + hop.offset,
        };
        let raw = mem.read_u64(addr).unwrap_or(0);
        let next = if hop.name == "route_task_direct" || hop.name == "route_task_ref" {
            if hop.name == "route_task_ref" {
                if !looks_like_heap_ptr(raw) {
                    return Err(RouteChainFailure {
                        status: RESOLVE_ROUTE_CHAIN_CANDIDATE_FAILED,
                        candidate: cand.name,
                        failed_step: hop.name,
                        offset: hop.offset,
                        raw_value: raw,
                        reason: if raw == 0 { "null_ptr" } else { "invalid_ptr" },
                        steps,
                    });
                }
                raw as usize + cand.route_task_bias
            } else {
                if !looks_like_heap_ptr(raw) {
                    return Err(RouteChainFailure {
                        status: RESOLVE_ROUTE_CHAIN_CANDIDATE_FAILED,
                        candidate: cand.name,
                        failed_step: hop.name,
                        offset: hop.offset,
                        raw_value: raw,
                        reason: if raw == 0 { "null_ptr" } else { "invalid_ptr" },
                        steps,
                    });
                }
                raw as usize + cand.route_task_bias
            }
        } else {
            match read_ptr(mem, addr) {
                Some(p) => p as usize,
                None => {
                    return Err(RouteChainFailure {
                        status: RESOLVE_ROUTE_CHAIN_CANDIDATE_FAILED,
                        candidate: cand.name,
                        failed_step: hop.name,
                        offset: hop.offset,
                        raw_value: raw,
                        reason: if raw == 0 { "null_ptr" } else { "invalid_ptr" },
                        steps,
                    });
                }
            }
        };
        steps.push(ChainStepLog {
            name: hop.name,
            offset: hop.offset,
            value: next as u64,
        });
        cur = next;
    }
    Ok((cur, steps))
}

fn try_candidate<R: MemRead>(
    mem: &R,
    gps: usize,
    cand: &RouteChainCandidate,
) -> Result<RouteChainSuccess, RouteChainFailure> {
    let (route_task, steps) = walk_hops(mem, gps, cand)?;
    match validate_route_task(mem, route_task, cand.phys_items_offset) {
        Ok((_arr, _first_uid, phys_off)) => {
            let (count, first_uid) = count_route_items(
                mem,
                read_ptr(mem, route_task + phys_off).unwrap_or(0) as usize,
            );
            if first_uid == 0 {
                return Err(RouteChainFailure {
                    status: RESOLVE_FIRST_UID_ZERO,
                    candidate: cand.name,
                    failed_step: "first_item",
                    offset: OFF_ITEM_UID,
                    raw_value: 0,
                    reason: "uid_zero",
                    steps,
                });
            }
            Ok(RouteChainSuccess {
                candidate: cand.name,
                route_task,
                phys_items_offset: phys_off,
                steps,
                first_uid,
                item_count: count,
            })
        }
        Err(st) => Err(RouteChainFailure {
            status: st,
            candidate: cand.name,
            failed_step: if st == RESOLVE_ROUTE_TASK_CANDIDATE_NULL {
                "route_task"
            } else if st == RESOLVE_ROUTE_ITEMS_EMPTY {
                "items_empty"
            } else {
                "route_items"
            },
            offset: cand.phys_items_offset,
            raw_value: route_task as u64,
            reason: "validation_failed",
            steps,
        }),
    }
}

fn walk_hops_from<R: MemRead>(
    mem: &R,
    start: usize,
    cand: &RouteChainCandidate,
    skip_first: bool,
) -> Result<(usize, Vec<ChainStepLog>), RouteChainFailure> {
    let mut steps = Vec::new();
    let mut cur = start;
    let hops = if skip_first {
        &cand.hops[1..]
    } else {
        cand.hops
    };
    if skip_first {
        steps.push(ChainStepLog {
            name: "simple_route_src",
            offset: 0,
            value: start as u64,
        });
    }
    for hop in hops {
        let addr = if skip_first {
            cur + hop.offset
        } else {
            match hop.base {
                HopBase::Gps => start + hop.offset,
                HopBase::Prev => cur + hop.offset,
            }
        };
        let raw = mem.read_u64(addr).unwrap_or(0);
        let next = if hop.name == "route_task_direct" || hop.name == "route_task_ref" {
            if !looks_like_heap_ptr(raw) {
                return Err(RouteChainFailure {
                    status: RESOLVE_ROUTE_CHAIN_CANDIDATE_FAILED,
                    candidate: cand.name,
                    failed_step: hop.name,
                    offset: hop.offset,
                    raw_value: raw,
                    reason: if raw == 0 { "null_ptr" } else { "invalid_ptr" },
                    steps,
                });
            }
            raw as usize + cand.route_task_bias
        } else {
            match read_ptr(mem, addr) {
                Some(p) => p as usize,
                None => {
                    return Err(RouteChainFailure {
                        status: RESOLVE_ROUTE_CHAIN_CANDIDATE_FAILED,
                        candidate: cand.name,
                        failed_step: hop.name,
                        offset: hop.offset,
                        raw_value: raw,
                        reason: if raw == 0 { "null_ptr" } else { "invalid_ptr" },
                        steps,
                    });
                }
            }
        };
        steps.push(ChainStepLog {
            name: hop.name,
            offset: hop.offset,
            value: next as u64,
        });
        cur = next;
    }
    Ok((cur, steps))
}

fn try_candidate_from_srs<R: MemRead>(
    mem: &R,
    srs: usize,
    cand: &RouteChainCandidate,
    dyn_name: &'static str,
) -> Result<RouteChainSuccess, RouteChainFailure> {
    let (route_task, steps) = walk_hops_from(mem, srs, cand, true)?;
    match validate_route_task(mem, route_task, cand.phys_items_offset) {
        Ok((_arr, _first_uid, phys_off)) => {
            let (count, first_uid) = count_route_items(
                mem,
                read_ptr(mem, route_task + phys_off).unwrap_or(0) as usize,
            );
            if first_uid == 0 {
                return Err(RouteChainFailure {
                    status: RESOLVE_FIRST_UID_ZERO,
                    candidate: dyn_name,
                    failed_step: "first_item",
                    offset: OFF_ITEM_UID,
                    raw_value: 0,
                    reason: "uid_zero",
                    steps,
                });
            }
            Ok(RouteChainSuccess {
                candidate: dyn_name,
                route_task,
                phys_items_offset: phys_off,
                steps,
                first_uid,
                item_count: count,
            })
        }
        Err(st) => Err(RouteChainFailure {
            status: st,
            candidate: dyn_name,
            failed_step: if st == RESOLVE_ROUTE_TASK_CANDIDATE_NULL {
                "route_task"
            } else {
                "route_items"
            },
            offset: cand.phys_items_offset,
            raw_value: route_task as u64,
            reason: "validation_failed",
            steps,
        }),
    }
}

fn try_a_slot_window<R: MemRead>(
    mem: &R,
    gps: usize,
    route_a: usize,
) -> Result<RouteChainSuccess, RouteChainFailure> {
    let mut steps = vec![
        ChainStepLog {
            name: "simple_route_src",
            offset: OFF_SIMPLE_ROUTE_SRC,
            value: read_ptr(mem, gps + OFF_SIMPLE_ROUTE_SRC).unwrap_or(0),
        },
        ChainStepLog {
            name: "route_a",
            offset: OFF_SRS_ROUTE_A,
            value: route_a as u64,
        },
    ];
    for off in (A_WINDOW_START..=A_WINDOW_END).step_by(8) {
        let q = match read_ptr(mem, route_a + off) {
            Some(v) => v as usize,
            None => continue,
        };
        for &(bias, label) in &[(OFF_ROUTE_TASK_BIAS, "ref+0x18"), (0, "ref+0x0")] {
            let ref_raw = mem.read_u64(q + OFF_ROUTE_TASK_REF).unwrap_or(0);
            if !looks_like_heap_ptr(ref_raw) {
                continue;
            }
            let rt = ref_raw as usize + bias;
            if let Ok((_arr, first_uid, phys_off)) =
                validate_route_task(mem, rt, OFF_PHYS_ITEMS)
            {
                steps.push(ChainStepLog {
                    name: "a_slot_window",
                    offset: off,
                    value: q as u64,
                });
                steps.push(ChainStepLog {
                    name: label,
                    offset: OFF_ROUTE_TASK_REF,
                    value: rt as u64,
                });
                let (count, _) = count_route_items(
                    mem,
                    read_ptr(mem, rt + phys_off).unwrap_or(0) as usize,
                );
                return Ok(RouteChainSuccess {
                    candidate: "a_slot_window",
                    route_task: rt,
                    phys_items_offset: phys_off,
                    steps,
                    first_uid,
                    item_count: count,
                });
            }
        }
    }
    Err(RouteChainFailure {
        status: RESOLVE_ROUTE_CHAIN_CANDIDATE_FAILED,
        candidate: "a_slot_window",
        failed_step: "a_slot_scan",
        offset: A_WINDOW_START,
        raw_value: 0,
        reason: "no_valid_slot",
        steps,
    })
}

fn read_route_a<R: MemRead>(mem: &R, gps: usize) -> Option<usize> {
    let srs = read_ptr(mem, gps + OFF_SIMPLE_ROUTE_SRC)?;
    read_ptr(mem, srs as usize + OFF_SRS_ROUTE_A).map(|v| v as usize)
}

fn simple_route_src_null<R: MemRead>(mem: &R, gps: usize) -> bool {
    mem.read_u64(gps + OFF_SIMPLE_ROUTE_SRC).unwrap_or(0) == 0
}

/// Format crash-safe GPS pointer table (`gps + 0x00 ..= 0x100`), no follow derefs.
pub fn format_safe_gps_pointer_table_lines(gps: usize) -> Vec<String> {
    let mut lines = vec![format!("gps pointer table gps=0x{gps:X}")];
    for off in (0..=GPS_TABLE_SAFE_END).step_by(8) {
        let known = GPS_TABLE_KNOWN_OFFSETS.contains(&off);
        match safe_mem::safe_read_u64(gps.saturating_add(off)) {
            Ok(raw) => {
                if looks_like_heap_ptr(raw) || (known && raw == 0) {
                    lines.push(format!("+0x{off:02X} = 0x{raw:X}"));
                }
            }
            Err(e) => {
                if known {
                    lines.push(format!("+0x{off:02X} read_failed={}", e.as_str()));
                }
            }
        }
    }
    lines
}

/// Full GPS pointer table for `gps_table_only` mode — every slot logged, no follow derefs.
pub fn format_gps_table_only_lines(gps: usize) -> Vec<String> {
    let mut lines = vec![format!("gps pointer table gps=0x{gps:X}")];
    for off in (0..=GPS_TABLE_SAFE_END).step_by(8) {
        match safe_mem::safe_read_u64(gps.saturating_add(off)) {
            Ok(raw) => lines.push(format!("+0x{off:02X} = 0x{raw:X}")),
            Err(e) => lines.push(format!("+0x{off:02X} read_failed={}", e.as_str())),
        }
    }
    lines.push(format!("gps pointer table done slots={GPS_TABLE_SLOT_COUNT}"));
    lines
}

/// Test/diagnostic helper: format GPS table via `MemRead` (no VirtualQuery).
pub fn format_gps_table_only_lines_mem<R: MemRead>(mem: &R, gps: usize) -> Vec<String> {
    let mut lines = vec![format!("gps pointer table gps=0x{gps:X}")];
    for off in (0..=GPS_TABLE_SAFE_END).step_by(8) {
        match mem.read_u64(gps.saturating_add(off)) {
            Some(raw) => lines.push(format!("+0x{off:02X} = 0x{raw:X}")),
            None => lines.push(format!("+0x{off:02X} read_failed=missing")),
        }
    }
    lines.push(format!("gps pointer table done slots={GPS_TABLE_SLOT_COUNT}"));
    lines
}

/// Outcome of a crash-safe GPS table read (no pointer derefs).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpsTableDiagnostic {
    pub base_read_ok: bool,
    pub slots_attempted: u32,
}

/// Read `gps+0x00..0x100` via `safe_read_u64` only; never dereference slot values.
pub fn diagnose_gps_pointer_table(gps: usize) -> (GpsTableDiagnostic, Vec<String>) {
    if gps == 0 || !safe_mem::addr_canonical(gps) {
        return (
            GpsTableDiagnostic {
                base_read_ok: false,
                slots_attempted: 0,
            },
            vec![format!("gps pointer table gps=0x{gps:X}")],
        );
    }

    let mut base_read_ok = false;
    let mut slots_attempted = 0u32;
    let mut lines = vec![format!("gps pointer table gps=0x{gps:X}")];
    for off in (0..=GPS_TABLE_SAFE_END).step_by(8) {
        slots_attempted += 1;
        match safe_mem::safe_read_u64(gps.saturating_add(off)) {
            Ok(raw) => {
                if off == 0 {
                    base_read_ok = true;
                }
                lines.push(format!("+0x{off:02X} = 0x{raw:X}"));
            }
            Err(e) => {
                lines.push(format!("+0x{off:02X} read_failed={}", e.as_str()));
            }
        }
    }
    lines.push(format!("gps pointer table done slots={GPS_TABLE_SLOT_COUNT}"));
    (
        GpsTableDiagnostic {
            base_read_ok,
            slots_attempted,
        },
        lines,
    )
}

/// Run `gps_table_only` diagnostic — logs table once per GPS pointer, no chain walk.
pub fn run_gps_table_only_diagnostic(gps: usize) -> u32 {
    if let Some(st) = crate::resolver_guard::block_if_resolver_off() {
        return st;
    }
    let (diag, lines) = diagnose_gps_pointer_table(gps);
    log_gps_table_only_sidecar(gps, &lines);
    if !diag.base_read_ok || diag.slots_attempted != GPS_TABLE_SLOT_COUNT {
        RESOLVE_GPS_TABLE_READ_FAILED
    } else {
        RESOLVE_GPS_TABLE_ONLY_DONE
    }
}

fn log_gps_table_only_sidecar(gps: usize, lines: &[String]) {
    #[cfg(windows)]
    {
        static LOGGED_GPS: std::sync::Mutex<Option<usize>> = std::sync::Mutex::new(None);
        let mut guard = LOGGED_GPS.lock().unwrap();
        if *guard == Some(gps) {
            return;
        }
        *guard = Some(gps);
        drop(guard);
        for line in lines {
            crate::diag_log::event_force(line);
        }
    }
    #[cfg(not(windows))]
    {
        let _ = (gps, lines);
    }
}

/// Max `game_ctrl` table span for crash-safe diagnostics (`game_ctrl + 0x0000 .. 0x5000`).
pub const GAME_CTRL_TABLE_SAFE_END: usize = 0x5000;
/// Number of 8-byte slots (`0x0000..0x5000`, end exclusive).
pub const GAME_CTRL_TABLE_SLOT_COUNT: u32 = (GAME_CTRL_TABLE_SAFE_END / 8) as u32;
const GAME_CTRL_KNOWN_RANGE_START: usize = 0x3E00;
const GAME_CTRL_KNOWN_RANGE_END: usize = 0x3E80;
const GAME_CTRL_EXTRA_ALWAYS_LOG: &[usize] = &[0x3E28, 0x3E38];

pub fn game_ctrl_offset_always_logged(off: usize) -> bool {
    if off % 8 != 0 {
        return false;
    }
    if GAME_CTRL_EXTRA_ALWAYS_LOG.contains(&off) {
        return true;
    }
    (GAME_CTRL_KNOWN_RANGE_START..=GAME_CTRL_KNOWN_RANGE_END).contains(&off)
}

/// Outcome of a crash-safe `game_ctrl` table read (no pointer derefs).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GameCtrlTableDiagnostic {
    pub base_read_ok: bool,
    pub slots_attempted: u32,
    pub nonzero_slots: u32,
    pub read_failures: u32,
}

/// Read `game_ctrl+0x0000..0x5000` via `safe_read_u64` only; never dereference slot values.
pub fn diagnose_game_ctrl_pointer_table(game_ctrl: usize) -> (GameCtrlTableDiagnostic, Vec<String>) {
    if game_ctrl == 0 || !safe_mem::addr_canonical(game_ctrl) {
        return (
            GameCtrlTableDiagnostic {
                base_read_ok: false,
                slots_attempted: 0,
                nonzero_slots: 0,
                read_failures: 0,
            },
            vec![format!("game_ctrl table game_ctrl=0x{game_ctrl:X}")],
        );
    }

    let mut lines = vec![format!("game_ctrl table game_ctrl=0x{game_ctrl:X}")];
    let mut base_read_ok = false;
    let mut slots_attempted = 0u32;
    let mut nonzero_slots = 0u32;
    let mut read_failures = 0u32;

    match safe_mem::safe_read_u64(game_ctrl.saturating_add(GPS_OFFSET_IN_GAME_CTRL)) {
        Ok(raw) => lines.push(format!(
            "known gps offset +0x{GPS_OFFSET_IN_GAME_CTRL:04X} = 0x{raw:X}"
        )),
        Err(e) => lines.push(format!(
            "known gps offset +0x{GPS_OFFSET_IN_GAME_CTRL:04X} read_failed={}",
            e.as_str()
        )),
    }

    for off in (0..GAME_CTRL_TABLE_SAFE_END).step_by(8) {
        slots_attempted += 1;
        let always = game_ctrl_offset_always_logged(off);
        match safe_mem::safe_read_u64(game_ctrl.saturating_add(off)) {
            Ok(raw) => {
                if off == 0 {
                    base_read_ok = true;
                }
                if raw != 0 {
                    nonzero_slots += 1;
                }
                if always || raw != 0 {
                    lines.push(format!("game_ctrl slot +0x{off:04X} = 0x{raw:X}"));
                }
            }
            Err(e) => {
                read_failures += 1;
                if always {
                    lines.push(format!(
                        "game_ctrl slot +0x{off:04X} read_failed={}",
                        e.as_str()
                    ));
                }
            }
        }
    }

    if read_failures > 0 {
        lines.push(format!("game_ctrl table read_failures={read_failures}"));
    }
    lines.push(format!(
        "game_ctrl table done slots={} nonzero={nonzero_slots}",
        GAME_CTRL_TABLE_SLOT_COUNT
    ));

    (
        GameCtrlTableDiagnostic {
            base_read_ok,
            slots_attempted,
            nonzero_slots,
            read_failures,
        },
        lines,
    )
}

/// Test helper: format `game_ctrl` table via `MemRead` (no VirtualQuery).
pub fn format_game_ctrl_table_lines_mem<R: MemRead>(mem: &R, game_ctrl: usize) -> Vec<String> {
    let mut lines = vec![format!("game_ctrl table game_ctrl=0x{game_ctrl:X}")];
    let gps_off = mem
        .read_u64(game_ctrl + GPS_OFFSET_IN_GAME_CTRL)
        .map(|raw| format!(
            "known gps offset +0x{GPS_OFFSET_IN_GAME_CTRL:04X} = 0x{raw:X}"
        ))
        .unwrap_or_else(|| {
            format!(
                "known gps offset +0x{GPS_OFFSET_IN_GAME_CTRL:04X} read_failed=missing"
            )
        });
    lines.push(gps_off);

    let mut nonzero = 0u32;
    for off in (0..GAME_CTRL_TABLE_SAFE_END).step_by(8) {
        let always = game_ctrl_offset_always_logged(off);
        match mem.read_u64(game_ctrl.saturating_add(off)) {
            Some(raw) => {
                if raw != 0 {
                    nonzero += 1;
                }
                if always || raw != 0 {
                    lines.push(format!("game_ctrl slot +0x{off:04X} = 0x{raw:X}"));
                }
            }
            None if always => {
                lines.push(format!("game_ctrl slot +0x{off:04X} read_failed=missing"));
            }
            None => {}
        }
    }
    lines.push(format!(
        "game_ctrl table done slots={} nonzero={nonzero}",
        GAME_CTRL_TABLE_SLOT_COUNT
    ));
    lines
}

/// ETS2 1.60 offline candidate: `lea rsi,[rdi+0x40F8]` after `mov rdi,[rip+game_ctrl]`.
pub const GPS_OFFSET_PROBE_OFFSET: usize = 0x40F8;
/// Offline singleton slot hint from `ets2-bin-analyze` (sidecar context only).
pub const GPS_OFFSET_PROBE_SINGLETON_RVA_HINT: usize = 0x354F398;

/// Outcome of the one-shot `game_ctrl + GPS_OFFSET_PROBE_OFFSET` read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpsOffsetProbeDiagnostic {
    pub read_ok: bool,
    pub value: u64,
}

/// Format probe lines via `MemRead` (unit tests — no VirtualQuery).
pub fn format_gps_offset_probe_lines_mem<R: MemRead>(mem: &R, game_ctrl: usize) -> Vec<String> {
    let offset = GPS_OFFSET_PROBE_OFFSET;
    let mut lines = vec![
        format!(
            "gps offset probe start singleton_rva=0x{GPS_OFFSET_PROBE_SINGLETON_RVA_HINT:X} offset=0x{offset:X}"
        ),
        format!("gps offset probe game_ctrl=0x{game_ctrl:X}"),
    ];
    if game_ctrl == 0 {
        lines.push(format!(
            "gps offset probe read failed game_ctrl=0x{game_ctrl:X} offset=0x{offset:X}"
        ));
        lines.push("gps offset probe done; parking resolver".into());
        return lines;
    }
    match mem.read_u64(game_ctrl.saturating_add(offset)) {
        Some(value) => {
            lines.push(format!(
                "gps offset probe read game_ctrl+0x{offset:X} value=0x{value:X}"
            ));
        }
        None => {
            lines.push(format!(
                "gps offset probe read failed game_ctrl=0x{game_ctrl:X} offset=0x{offset:X}"
            ));
        }
    }
    lines.push("gps offset probe done; parking resolver".into());
    lines
}

/// One `safe_read_u64(game_ctrl + 0x40F8)` — no pointer follow, no tables, no chain.
pub fn diagnose_gps_offset_probe(game_ctrl: usize) -> (GpsOffsetProbeDiagnostic, Vec<String>) {
    if game_ctrl == 0 || !safe_mem::addr_canonical(game_ctrl) {
        let lines = format_gps_offset_probe_lines_mem(&FakeMem::default(), game_ctrl);
        return (
            GpsOffsetProbeDiagnostic {
                read_ok: false,
                value: 0,
            },
            lines,
        );
    }

    let offset = GPS_OFFSET_PROBE_OFFSET;
    let mut lines = vec![
        format!(
            "gps offset probe start singleton_rva=0x{GPS_OFFSET_PROBE_SINGLETON_RVA_HINT:X} offset=0x{offset:X}"
        ),
        format!("gps offset probe game_ctrl=0x{game_ctrl:X}"),
    ];
    match safe_mem::safe_read_u64(game_ctrl.saturating_add(offset)) {
        Ok(value) => {
            lines.push(format!(
                "gps offset probe read game_ctrl+0x{offset:X} value=0x{value:X}"
            ));
            lines.push("gps offset probe done; parking resolver".into());
            (
                GpsOffsetProbeDiagnostic {
                    read_ok: true,
                    value,
                },
                lines,
            )
        }
        Err(_) => {
            lines.push(format!(
                "gps offset probe read failed game_ctrl=0x{game_ctrl:X} offset=0x{offset:X}"
            ));
            lines.push("gps offset probe done; parking resolver".into());
            (
                GpsOffsetProbeDiagnostic {
                    read_ok: false,
                    value: 0,
                },
                lines,
            )
        }
    }
}

fn log_gps_offset_probe_sidecar(game_ctrl: usize, lines: &[String]) {
    #[cfg(windows)]
    {
        static LOGGED_GAME_CTRL: std::sync::Mutex<Option<usize>> = std::sync::Mutex::new(None);
        let mut guard = LOGGED_GAME_CTRL.lock().unwrap();
        if *guard == Some(game_ctrl) {
            return;
        }
        *guard = Some(game_ctrl);
        drop(guard);
        for line in lines {
            crate::diag_log::event_force(line);
        }
    }
    #[cfg(not(windows))]
    {
        let _ = (game_ctrl, lines);
    }
}

/// Run `gps_offset_probe` diagnostic — exactly one pointer-sized read, then park.
pub fn run_gps_offset_probe_diagnostic(game_ctrl: usize) -> u32 {
    if let Some(st) = crate::resolver_guard::block_if_resolver_off() {
        return st;
    }
    let (diag, lines) = diagnose_gps_offset_probe(game_ctrl);
    log_gps_offset_probe_sidecar(game_ctrl, &lines);
    if diag.read_ok {
        RESOLVE_GPS_OFFSET_PROBE_DONE
    } else {
        RESOLVE_GPS_OFFSET_PROBE_READ_FAILED
    }
}

/// Run `game_ctrl_table` diagnostic — logs table once per `game_ctrl` pointer, no chain walk.
pub fn run_game_ctrl_table_only_diagnostic(game_ctrl: usize) -> u32 {
    if let Some(st) = crate::resolver_guard::block_if_resolver_off() {
        return st;
    }
    let (diag, lines) = diagnose_game_ctrl_pointer_table(game_ctrl);
    log_game_ctrl_table_only_sidecar(game_ctrl, &lines);
    if !diag.base_read_ok || diag.slots_attempted != GAME_CTRL_TABLE_SLOT_COUNT {
        RESOLVE_GAME_CTRL_TABLE_READ_FAILED
    } else {
        RESOLVE_GAME_CTRL_TABLE_ONLY_DONE
    }
}

fn log_game_ctrl_table_only_sidecar(game_ctrl: usize, lines: &[String]) {
    #[cfg(windows)]
    {
        static LOGGED_GAME_CTRL: std::sync::Mutex<Option<usize>> = std::sync::Mutex::new(None);
        let mut guard = LOGGED_GAME_CTRL.lock().unwrap();
        if *guard == Some(game_ctrl) {
            return;
        }
        *guard = Some(game_ctrl);
        drop(guard);
        for line in lines {
            crate::diag_log::event_force(line);
        }
    }
    #[cfg(not(windows))]
    {
        let _ = (game_ctrl, lines);
    }
}

/// Fixed `game_ctrl` offsets whose pointer values get a one-level candidate table dump.
pub const ROUTE_CANDIDATE_GAME_CTRL_OFFSETS: &[usize] = &[
    0x3AC0, 0x4038, 0x4130, 0x41B0, 0x4228, 0x4230, 0x4390, 0x4580,
];

const ROUTE_CANDIDATE_IMPORTANT_OFFSETS: &[usize] =
    &[0x00, 0x08, 0x10, 0x18, 0x20, 0x30, 0x50, 0x58, 0x60];

pub fn route_candidate_table_important_offset(off: usize) -> bool {
    ROUTE_CANDIDATE_IMPORTANT_OFFSETS.contains(&off)
}

/// Outcome of route candidate table diagnostic (no second-level derefs).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RouteCandidateTableDiagnostic {
    pub base_read_ok: bool,
    pub sources_attempted: u32,
}

pub fn format_route_candidate_table_lines<R: MemRead>(
    mem: &R,
    game_ctrl: usize,
) -> Vec<String> {
    let mut lines = vec![format!("route candidate table game_ctrl=0x{game_ctrl:X}")];
    for &src_off in ROUTE_CANDIDATE_GAME_CTRL_OFFSETS {
        match mem.read_u64(game_ctrl.saturating_add(src_off)) {
            Some(val) => {
                lines.push(format!(
                    "candidate source game_ctrl+0x{src_off:04X} value=0x{val:X}"
                ));
                if looks_like_heap_ptr(val) {
                    lines.extend(format_route_candidate_child_table_lines_mem(
                        mem,
                        val as usize,
                        src_off,
                    ));
                }
            }
            None => {
                lines.push(format!(
                    "candidate source game_ctrl+0x{src_off:04X} read_failed=missing"
                ));
            }
        }
    }
    lines
}

fn format_route_candidate_child_table_lines_mem<R: MemRead>(
    mem: &R,
    candidate: usize,
    source_off: usize,
) -> Vec<String> {
    let mut lines = vec![format!(
        "candidate table source=+0x{source_off:04X} candidate=0x{candidate:X}"
    )];
    let mut nonzero = 0u32;
    for off in (0..=GPS_TABLE_SAFE_END).step_by(8) {
        match mem.read_u64(candidate.saturating_add(off)) {
            Some(raw) => {
                if raw != 0 {
                    nonzero += 1;
                }
                if raw != 0 || route_candidate_table_important_offset(off) {
                    lines.push(format!("  +0x{off:02X} = 0x{raw:X}"));
                }
            }
            None if route_candidate_table_important_offset(off) => {
                lines.push(format!("  +0x{off:02X} read_failed=missing"));
            }
            None => {}
        }
    }
    lines.push(format!(
        "candidate table done source=+0x{source_off:04X} slots={GPS_TABLE_SLOT_COUNT} nonzero={nonzero}"
    ));
    lines
}

fn format_route_candidate_child_table_lines(candidate: usize, source_off: usize) -> Vec<String> {
    let mut lines = vec![format!(
        "candidate table source=+0x{source_off:04X} candidate=0x{candidate:X}"
    )];
    let mut nonzero = 0u32;
    for off in (0..=GPS_TABLE_SAFE_END).step_by(8) {
        match safe_mem::safe_read_u64(candidate.saturating_add(off)) {
            Ok(raw) => {
                if raw != 0 {
                    nonzero += 1;
                }
                if raw != 0 || route_candidate_table_important_offset(off) {
                    lines.push(format!("  +0x{off:02X} = 0x{raw:X}"));
                }
            }
            Err(e) if route_candidate_table_important_offset(off) => {
                lines.push(format!("  +0x{off:02X} read_failed={}", e.as_str()));
            }
            Err(_) => {}
        }
    }
    lines.push(format!(
        "candidate table done source=+0x{source_off:04X} slots={GPS_TABLE_SLOT_COUNT} nonzero={nonzero}"
    ));
    lines
}

pub fn diagnose_route_candidate_table(game_ctrl: usize) -> (RouteCandidateTableDiagnostic, Vec<String>) {
    if game_ctrl == 0 || !safe_mem::addr_canonical(game_ctrl) {
        return (
            RouteCandidateTableDiagnostic {
                base_read_ok: false,
                sources_attempted: 0,
            },
            vec![format!("route candidate table game_ctrl=0x{game_ctrl:X}")],
        );
    }

    let base_read_ok = safe_mem::safe_read_u64(game_ctrl).is_ok();
    let mut lines = vec![format!("route candidate table game_ctrl=0x{game_ctrl:X}")];
    let mut sources_attempted = 0u32;

    for &src_off in ROUTE_CANDIDATE_GAME_CTRL_OFFSETS {
        sources_attempted += 1;
        match safe_mem::safe_read_u64(game_ctrl.saturating_add(src_off)) {
            Ok(val) => {
                lines.push(format!(
                    "candidate source game_ctrl+0x{src_off:04X} value=0x{val:X}"
                ));
                if looks_like_heap_ptr(val) {
                    lines.extend(format_route_candidate_child_table_lines(
                        val as usize,
                        src_off,
                    ));
                }
            }
            Err(e) => {
                lines.push(format!(
                    "candidate source game_ctrl+0x{src_off:04X} read_failed={}",
                    e.as_str()
                ));
            }
        }
    }

    (
        RouteCandidateTableDiagnostic {
            base_read_ok,
            sources_attempted,
        },
        lines,
    )
}

pub fn run_route_candidate_table_only_diagnostic(game_ctrl: usize) -> u32 {
    if let Some(st) = crate::resolver_guard::block_if_resolver_off() {
        return st;
    }
    let (diag, lines) = diagnose_route_candidate_table(game_ctrl);
    log_route_candidate_table_only_sidecar(game_ctrl, &lines);
    if !diag.base_read_ok
        || diag.sources_attempted != ROUTE_CANDIDATE_GAME_CTRL_OFFSETS.len() as u32
    {
        RESOLVE_ROUTE_CANDIDATE_TABLE_READ_FAILED
    } else {
        RESOLVE_ROUTE_CANDIDATE_TABLE_DONE
    }
}

fn log_route_candidate_table_only_sidecar(game_ctrl: usize, lines: &[String]) {
    #[cfg(windows)]
    {
        static LOGGED_GAME_CTRL: std::sync::Mutex<Option<usize>> = std::sync::Mutex::new(None);
        let mut guard = LOGGED_GAME_CTRL.lock().unwrap();
        if *guard == Some(game_ctrl) {
            return;
        }
        *guard = Some(game_ctrl);
        drop(guard);
        for line in lines {
            crate::diag_log::event_force(line);
        }
    }
    #[cfg(not(windows))]
    {
        let _ = (game_ctrl, lines);
    }
}

/// Format GPS pointer table lines for unit tests / FakeMem (no VirtualQuery).
pub fn format_gps_pointer_table_lines<R: MemRead>(mem: &R, gps: usize) -> Vec<String> {
    let mut lines = vec![format!("gps pointer table gps=0x{gps:X}")];
    for off in (0..=GPS_PTR_TABLE_END).step_by(8) {
        let raw = mem.read_u64(gps + off).unwrap_or(0);
        let known = GPS_TABLE_KNOWN_OFFSETS.contains(&off);
        if looks_like_heap_ptr(raw) || (known && raw == 0) {
            lines.push(format!("+0x{off:02X} = 0x{raw:X}"));
        }
    }
    lines
}

fn scan_srs_offsets_with_meta<R: MemRead>(
    mem: &R,
    gps: usize,
) -> (Option<RouteChainSuccess>, bool) {
    #[cfg(windows)]
    log_srs_scan_start(gps);
    let mut tried_any = false;
    for off in (0..=SRS_OFFSET_SCAN_END).step_by(8) {
        if off == OFF_SIMPLE_ROUTE_SRC {
            continue;
        }
        let raw = mem.read_u64(gps + off).unwrap_or(0);
        if !looks_like_heap_ptr(raw) {
            continue;
        }
        tried_any = true;
        let srs = raw as usize;
        for cand in CHAIN_CANDIDATES {
            match try_candidate_from_srs(mem, srs, cand, "srs_dyn") {
                Ok(ok) => return (Some(ok), tried_any),
                Err(f) => {
                    #[cfg(windows)]
                    log_srs_candidate(gps, off, raw, &f);
                }
            }
        }
    }
    (None, tried_any)
}

fn scan_direct_route<R: MemRead>(
    mem: &R,
    gps: usize,
) -> Option<RouteChainSuccess> {
    for off in (0..=GPS_DIRECT_SCAN_END).step_by(8) {
        let raw = mem.read_u64(gps + off).unwrap_or(0);
        if !looks_like_heap_ptr(raw) {
            continue;
        }
        let ptr = raw as usize;

        for &items_off in DIRECT_ITEMS_OFFSETS {
            if let Ok((_arr, first_uid, phys_off)) = validate_route_task(mem, ptr, items_off) {
                let (count, _) = count_route_items(
                    mem,
                    read_ptr(mem, ptr + phys_off).unwrap_or(0) as usize,
                );
                #[cfg(windows)]
                log_direct_route_task(gps, off, ptr, items_off, "waypoints_collected", first_uid);
                return Some(RouteChainSuccess {
                    candidate: "direct_route_task",
                    route_task: ptr,
                    phys_items_offset: phys_off,
                    steps: vec![ChainStepLog {
                        name: "direct_route_task",
                        offset: off,
                        value: ptr as u64,
                    }],
                    first_uid,
                    item_count: count,
                });
            }
        }

        for &items_off in DIRECT_ITEMS_OFFSETS {
            let Some(arr_ptr) = read_ptr(mem, ptr + items_off) else {
                continue;
            };
            let uid0 = mem.read_u64(arr_ptr as usize + OFF_ITEM_UID).unwrap_or(0);
            #[cfg(windows)]
            log_direct_items(gps, off, ptr, uid0);
            if !uid_plausible(uid0) {
                continue;
            }
            let (count, first_uid) = count_route_items(mem, arr_ptr as usize);
            if first_uid == 0 {
                continue;
            }
            return Some(RouteChainSuccess {
                candidate: "direct_items_owner",
                route_task: ptr,
                phys_items_offset: items_off,
                steps: vec![ChainStepLog {
                    name: "direct_items_owner",
                    offset: off,
                    value: ptr as u64,
                }],
                first_uid,
                item_count: count,
            });
        }
    }
    None
}

fn refine_failure<R: MemRead>(
    mem: &R,
    gps: usize,
    last_fail: Option<RouteChainFailure>,
    srs_scan_ran: bool,
    srs_candidates_tried: bool,
) -> RouteChainFailure {
    if simple_route_src_null(mem, gps) {
        if srs_scan_ran && srs_candidates_tried {
            return RouteChainFailure {
                status: RESOLVE_SRS_OFFSET_SCAN_FAILED,
                candidate: "srs_offset_scan",
                failed_step: "simple_route_src",
                offset: OFF_SIMPLE_ROUTE_SRC,
                raw_value: 0,
                reason: "srs_offset_scan_failed",
                steps: Vec::new(),
            };
        }
        return RouteChainFailure {
            status: RESOLVE_SIMPLE_ROUTE_SRC_OFFSET_UNKNOWN,
            candidate: "simple_route_src",
            failed_step: "simple_route_src",
            offset: OFF_SIMPLE_ROUTE_SRC,
            raw_value: 0,
            reason: "gps+0x08_null",
            steps: Vec::new(),
        };
    }
    if let Some(mut f) = last_fail {
        if f.failed_step == "simple_route_src" && f.raw_value == 0 {
            f.status = RESOLVE_SIMPLE_ROUTE_SRC_NULL;
        }
        return f;
    }
    RouteChainFailure {
        status: RESOLVE_ROUTE_CHAIN_ALL_FAILED,
        candidate: "none",
        failed_step: "simple_route_src",
        offset: OFF_SIMPLE_ROUTE_SRC,
        raw_value: 0,
        reason: "all_candidates_failed",
        steps: Vec::new(),
    }
}

pub fn failure_unsafe_scan_disabled() -> RouteChainFailure {
    RouteChainFailure {
        status: RESOLVE_UNSAFE_ROUTE_SCAN_DISABLED,
        candidate: "policy",
        failed_step: "deep_scan_disabled",
        offset: 0,
        raw_value: 0,
        reason: "unsafe_route_scan_disabled",
        steps: Vec::new(),
    }
}

#[cfg(windows)]
fn log_route_scan_policy(policy: RouteScanPolicy) {
    static LOGGED: std::sync::Once = std::sync::Once::new();
    LOGGED.call_once(|| {
        crate::diag_log::event_force("route scan step=start");
        crate::diag_log::event_force(&format!(
            "route scan step=deep_route_scan enabled={}",
            policy.deep_scan
        ));
        crate::diag_log::event_force(&format!(
            "route scan step=static_chain enabled={}",
            policy.allow_static_chain
        ));
        crate::diag_log::event_force(&format!(
            "route scan step=srs_offset_scan enabled={}",
            policy.allow_srs_offset_scan
        ));
        crate::diag_log::event_force(&format!(
            "route scan step=direct_items_scan enabled={}",
            policy.allow_direct_scan
        ));
    });
}

#[cfg(not(windows))]
fn log_route_scan_policy(_policy: RouteScanPolicy) {}

/// Crash-safe GPS table dump (pointer fields only, `gps+0x00..0x100`).
pub fn log_safe_gps_pointer_table(gps: usize) {
    #[cfg(windows)]
    {
        static LOGGED_GPS: std::sync::Mutex<Option<usize>> = std::sync::Mutex::new(None);
        let mut guard = LOGGED_GPS.lock().unwrap();
        if *guard == Some(gps) {
            return;
        }
        *guard = Some(gps);
        drop(guard);
        crate::diag_log::event_force("route scan step=gps_table safe=true");
        for line in format_safe_gps_pointer_table_lines(gps) {
            crate::diag_log::event_force(&line);
        }
        crate::diag_log::event_force("gps_pointer_table_dumped");
    }
    #[cfg(not(windows))]
    let _ = gps;
}

pub fn resolve_route_chain<R: MemRead>(mem: &R, gps: usize) -> Result<RouteChainSuccess, RouteChainFailure> {
    resolve_route_chain_with_policy(mem, gps, safe_mem::route_scan_policy())
}

pub fn resolve_route_chain_with_policy<R: MemRead>(
    mem: &R,
    gps: usize,
    policy: RouteScanPolicy,
) -> Result<RouteChainSuccess, RouteChainFailure> {
    log_route_scan_policy(policy);

    if !policy.deep_scan {
        log_safe_gps_pointer_table(gps);
        return Err(failure_unsafe_scan_disabled());
    }

    resolve_route_chain_deep(mem, gps, policy)
}

fn resolve_route_chain_deep<R: MemRead>(
    mem: &R,
    gps: usize,
    policy: RouteScanPolicy,
) -> Result<RouteChainSuccess, RouteChainFailure> {
    let srs_null = simple_route_src_null(mem, gps);
    let mut last_fail: Option<RouteChainFailure> = None;

    if policy.allow_static_chain && !srs_null {
        for (i, cand) in CHAIN_CANDIDATES.iter().enumerate() {
            let attempt = try_candidate(mem, gps, cand);
            #[cfg(windows)]
            log_chain_attempt(gps, &attempt, i, false);
            match attempt {
                Ok(ok) => return Ok(ok),
                Err(f) => last_fail = Some(f),
            }
        }
        if let Some(route_a) = read_route_a(mem, gps) {
            let attempt = try_a_slot_window(mem, gps, route_a);
            #[cfg(windows)]
            log_chain_attempt(gps, &attempt, CHAIN_CANDIDATES.len(), false);
            match attempt {
                Ok(ok) => return Ok(ok),
                Err(f) => last_fail = Some(f),
            }
        }
        return Err(refine_failure(mem, gps, last_fail, false, false));
    }

    if srs_null {
        log_safe_gps_pointer_table(gps);
    }

    if policy.allow_static_chain {
        for (i, cand) in CHAIN_CANDIDATES.iter().enumerate() {
            let attempt = try_candidate(mem, gps, cand);
            #[cfg(windows)]
            log_chain_attempt(gps, &attempt, i, srs_null);
            if let Err(f) = &attempt {
                last_fail = Some(f.clone());
            }
        }
    }

    let mut srs_tried = false;
    if policy.allow_srs_offset_scan {
        let (srs_ok, tried) = scan_srs_offsets_with_meta(mem, gps);
        srs_tried = tried;
        if let Some(ok) = srs_ok {
            return Ok(ok);
        }
    }

    if policy.allow_direct_scan {
        if let Some(ok) = scan_direct_route(mem, gps) {
            return Ok(ok);
        }
    }

    Err(refine_failure(
        mem,
        gps,
        last_fail,
        srs_null || srs_tried,
        srs_tried,
    ))
}

pub fn failure_to_status(f: &RouteChainFailure) -> u32 {
    match f.status {
        RESOLVE_FIRST_UID_ZERO => RESOLVE_FIRST_UID_ZERO,
        RESOLVE_ROUTE_ITEMS_EMPTY => RESOLVE_ROUTE_ITEMS_EMPTY,
        RESOLVE_ROUTE_ITEMS_CANDIDATE_NULL => RESOLVE_ROUTE_ITEMS_CANDIDATE_NULL,
        RESOLVE_ROUTE_TASK_CANDIDATE_NULL => RESOLVE_ROUTE_TASK_CANDIDATE_NULL,
        RESOLVE_SIMPLE_ROUTE_SRC_NULL => RESOLVE_SIMPLE_ROUTE_SRC_NULL,
        RESOLVE_SIMPLE_ROUTE_SRC_OFFSET_UNKNOWN => RESOLVE_SIMPLE_ROUTE_SRC_OFFSET_UNKNOWN,
        RESOLVE_SRS_OFFSET_SCAN_FAILED => RESOLVE_SRS_OFFSET_SCAN_FAILED,
        RESOLVE_UNSAFE_ROUTE_SCAN_DISABLED => RESOLVE_UNSAFE_ROUTE_SCAN_DISABLED,
        RESOLVE_ROUTE_CHAIN_CANDIDATE_FAILED => RESOLVE_ROUTE_CHAIN_ALL_FAILED,
        _ => f.status,
    }
}

pub fn format_chain_steps(steps: &[ChainStepLog]) -> String {
    steps
        .iter()
        .enumerate()
        .map(|(i, s)| format!("step[{i}] {} + 0x{:X} => 0x{:X}", s.name, s.offset, s.value))
        .collect::<Vec<_>>()
        .join(" ")
}

#[cfg(windows)]
mod chain_diag_log {
    use super::*;
    use std::sync::Mutex;
    use std::time::{SystemTime, UNIX_EPOCH};

    struct ChainDiagState {
        gps_table_gps: usize,
        gps_table_logged: bool,
        chain_start_logged: bool,
        srs_scan_logged: bool,
        rate: RateLog,
    }

    impl ChainDiagState {
        const fn new() -> Self {
            Self {
                gps_table_gps: 0,
                gps_table_logged: false,
                chain_start_logged: false,
                srs_scan_logged: false,
                rate: RateLog::new(),
            }
        }
    }

    static CHAIN_DIAG: Mutex<ChainDiagState> = Mutex::new(ChainDiagState::new());

    fn now_us() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_micros() as u64)
            .unwrap_or(0)
    }

    pub fn log_gps_pointer_table<R: MemRead>(mem: &R, gps: usize) {
        let mut st = CHAIN_DIAG.lock().unwrap();
        if st.gps_table_logged && st.gps_table_gps == gps {
            return;
        }
        st.gps_table_logged = true;
        st.gps_table_gps = gps;
        drop(st);
        for line in format_gps_pointer_table_lines(mem, gps) {
            crate::diag_log::event_force(&line);
        }
        crate::diag_log::event_force("gps_pointer_table_dumped");
    }

    pub fn log_srs_scan_start(gps: usize) {
        let mut st = CHAIN_DIAG.lock().unwrap();
        if st.srs_scan_logged {
            return;
        }
        st.srs_scan_logged = true;
        drop(st);
        crate::diag_log::event_force(&format!("srs offset scan gps=0x{gps:X}"));
    }

    pub fn log_srs_candidate(gps: usize, off: usize, ptr: u64, f: &RouteChainFailure) {
        let mut st = CHAIN_DIAG.lock().unwrap();
        let now = now_us();
        let key = format!("srs:{gps}:{off}");
        let msg = format!(
            "srs_candidate offset=0x{off:X} ptr=0x{ptr:X} result={}",
            crate::route_status::resolve_status_str(failure_to_status(f))
        );
        if st.rate.should_log(&key, now, false) {
            crate::diag_log::event_force(&msg);
            st.rate.record(&key, now);
        }
    }

    pub fn log_direct_route_task(
        gps: usize,
        off: usize,
        ptr: usize,
        items_off: usize,
        result: &str,
        uid: u64,
    ) {
        let mut st = CHAIN_DIAG.lock().unwrap();
        let now = now_us();
        let key = format!("direct_rt:{gps}:{off}:{items_off}:{result}");
        let msg = format!(
            "direct route_task scan gps+0x{off:X} ptr=0x{ptr:X} items_off=0x{items_off:X} result={result} uid0=0x{uid:X}"
        );
        if st.rate.should_log(&key, now, result == "waypoints_collected") {
            crate::diag_log::event_force(&msg);
            st.rate.record(&key, now);
        }
    }

    pub fn log_direct_items(gps: usize, off: usize, ptr: usize, uid0: u64) {
        let mut st = CHAIN_DIAG.lock().unwrap();
        let now = now_us();
        let key = format!("direct_items:{gps}:{off}:{uid0}");
        let msg = format!("direct items scan gps+0x{off:X} ptr=0x{ptr:X} uid0=0x{uid0:X}");
        if st.rate.should_log(&key, now, uid_plausible(uid0)) {
            crate::diag_log::event_force(&msg);
            st.rate.record(&key, now);
        }
    }

    pub fn log_chain_attempt(
        gps: usize,
        result: &Result<RouteChainSuccess, RouteChainFailure>,
        candidate_index: usize,
        srs_null: bool,
    ) {
        let mut st = CHAIN_DIAG.lock().unwrap();
        let now = now_us();
        if !st.chain_start_logged {
            st.chain_start_logged = true;
            crate::diag_log::event_force(&format!("pointer chain start gps=0x{gps:X}"));
        }
        let status_key = match result {
            Ok(ok) => format!("ok:{}", ok.candidate),
            Err(f) => format!(
                "fail:{}:{}:{}",
                f.candidate, f.failed_step, failure_to_status(f)
            ),
        };
        let key = format!("chain:{candidate_index}:{status_key}");
        let force = matches!(result, Ok(_));
        if !st.rate.should_log(&key, now, force) {
            return;
        }
        match result {
            Ok(ok) => {
                crate::diag_log::event_force(&format!(
                    "route chain candidate[{candidate_index}] name={} result=waypoints_collected count={} uid=0x{:X}",
                    ok.candidate, ok.item_count, ok.first_uid
                ));
                crate::diag_log::event_force(&format_chain_steps(&ok.steps));
            }
            Err(f) => {
                if srs_null && f.failed_step == "simple_route_src" && f.raw_value == 0 {
                    // Expected when gps+0x08 is null — suppress per-candidate spam.
                    if candidate_index == 0 {
                        crate::diag_log::event_force(&format!(
                            "route chain candidate[{candidate_index}] name={} result={} step={} offset=0x{:X} value=0x{:X} reason={}",
                            f.candidate,
                            crate::route_status::resolve_status_str(failure_to_status(f)),
                            f.failed_step,
                            f.offset,
                            f.raw_value,
                            f.reason
                        ));
                    }
                } else {
                    crate::diag_log::event_force(&format!(
                        "route chain candidate[{candidate_index}] name={} result={} step={} offset=0x{:X} value=0x{:X} reason={}",
                        f.candidate,
                        crate::route_status::resolve_status_str(failure_to_status(f)),
                        f.failed_step,
                        f.offset,
                        f.raw_value,
                        f.reason
                    ));
                }
                if !f.steps.is_empty() && candidate_index == 0 {
                    crate::diag_log::event_force(&format_chain_steps(&f.steps));
                }
            }
        }
        st.rate.record(&key, now);
    }
}

#[cfg(windows)]
use chain_diag_log::{
    log_chain_attempt, log_direct_items, log_direct_route_task, log_gps_pointer_table,
    log_srs_candidate, log_srs_scan_start,
};

#[cfg(windows)]
pub unsafe fn log_gps_pointer_table_once(gps: *const u8) {
    if gps.is_null() {
        return;
    }
    log_gps_pointer_table(&LiveMem, gps as usize);
}

#[cfg(not(windows))]
pub unsafe fn log_gps_pointer_table_once(_gps: *const u8) {}

#[cfg(not(windows))]
fn log_gps_pointer_table<R: MemRead>(_mem: &R, _gps: usize) {}

#[cfg(not(windows))]
fn log_srs_scan_start(_gps: usize) {}

#[cfg(not(windows))]
fn log_srs_candidate(_gps: usize, _off: usize, _ptr: u64, _f: &RouteChainFailure) {}

#[cfg(not(windows))]
fn log_direct_route_task(
    _gps: usize,
    _off: usize,
    _ptr: usize,
    _items_off: usize,
    _result: &str,
    _uid: u64,
) {
}

#[cfg(not(windows))]
fn log_direct_items(_gps: usize, _off: usize, _ptr: usize, _uid0: u64) {}

#[cfg(not(windows))]
fn log_chain_attempt(
    _gps: usize,
    _result: &Result<RouteChainSuccess, RouteChainFailure>,
    _i: usize,
    _srs_null: bool,
) {
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::route_status::RESOLVE_WAYPOINTS_COLLECTED;
    use crate::safe_mem::{self, RouteScanPolicy};
    use crate::test_isolation::TestResolverStateGuard;

    fn with_enable_file(enable_name: &str, f: impl FnOnce()) {
        let _guard = TestResolverStateGuard::acquire();
        let dir = std::env::temp_dir().join(format!("tp-rc-{enable_name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::File::create(dir.join(enable_name)).unwrap();
        safe_mem::set_test_enable_dir(Some(dir.clone()));
        f();
        safe_mem::set_test_enable_dir(None);
        let _ = std::fs::remove_dir_all(dir);
    }

    fn full_scan_policy() -> RouteScanPolicy {
        RouteScanPolicy {
            deep_scan: true,
            allow_static_chain: true,
            allow_srs_offset_scan: true,
            allow_direct_scan: true,
        }
    }

    fn build_v159_long(mem: &mut FakeMem, gps: usize) -> usize {
        let srs = 0x20_0000usize;
        let a = 0x30_0000;
        let b = 0x40_0000;
        let rt_ref = 0x50_0000;
        let rt = rt_ref + OFF_ROUTE_TASK_BIAS;
        let items = 0x60_0000;
        mem.set(gps + OFF_SIMPLE_ROUTE_SRC, srs as u64);
        mem.set(srs + OFF_SRS_ROUTE_A, a as u64);
        mem.set(a + OFF_ROUTE_B_SLOT, b as u64);
        mem.set(b + OFF_ROUTE_TASK_REF, rt_ref as u64);
        mem.set(rt + OFF_PHYS_ITEMS, items as u64);
        let uid = UID_MIN_PLAUSIBLE + 42;
        mem.set(items + OFF_ITEM_UID, uid);
        mem.set(items + OFF_ITEM_ACTIVE, 1);
        mem.set(items + ITEM_STRIDE + OFF_ITEM_UID, uid + 1);
        mem.set(items + ITEM_STRIDE + OFF_ITEM_ACTIVE, 1);
        rt
    }

    #[test]
    fn chain_v159_long_success() {
        let mut mem = FakeMem::default();
        let gps = 0x10_0000usize;
        build_v159_long(&mut mem, gps);
        let ok = resolve_route_chain_with_policy(&mem, gps, full_scan_policy()).unwrap();
        assert_eq!(ok.candidate, "v159_long");
        assert!(ok.item_count >= 2);
    }

    #[test]
    fn route_task_null_maps_status() {
        let mem = FakeMem::default();
        let gps = 0x10_0000usize;
        let f = resolve_route_chain(&mem, gps).unwrap_err();
        assert_eq!(
            failure_to_status(&f),
            RESOLVE_UNSAFE_ROUTE_SCAN_DISABLED
        );
    }

    #[test]
    fn deep_scan_disabled_by_default_policy() {
        let mem = FakeMem::default();
        let gps = 0x10_0000usize;
        let f = resolve_route_chain(&mem, gps).unwrap_err();
        assert_eq!(f.status, RESOLVE_UNSAFE_ROUTE_SCAN_DISABLED);
    }

    #[test]
    fn route_candidate_table_only_rejects_null_game_ctrl() {
        with_enable_file("truckpilot_route_resolver.route_candidate_table", || {
            assert_eq!(
                run_route_candidate_table_only_diagnostic(0),
                RESOLVE_ROUTE_CANDIDATE_TABLE_READ_FAILED
            );
        });
    }

    #[test]
    fn route_candidate_table_reads_only_configured_sources() {
        let mut mem = FakeMem::default();
        let game_ctrl = 0x10_0000usize;
        mem.set(game_ctrl, 0x1);
        for &off in ROUTE_CANDIDATE_GAME_CTRL_OFFSETS {
            mem.set(game_ctrl + off, 0);
        }
        let lines = format_route_candidate_table_lines(&mem, game_ctrl);
        assert_eq!(
            lines
                .iter()
                .filter(|l| l.starts_with("candidate source game_ctrl+"))
                .count(),
            ROUTE_CANDIDATE_GAME_CTRL_OFFSETS.len()
        );
        assert!(!lines.iter().any(|l| l.contains("game_ctrl+0x3AC8")));
    }

    #[test]
    fn route_candidate_table_reads_direct_candidate_slots_only() {
        let mut mem = FakeMem::default();
        let game_ctrl = 0x10_0000usize;
        let candidate = 0x20_0000usize;
        mem.set(game_ctrl + 0x3AC0, candidate as u64);
        mem.set(candidate + 0x00, 0);
        mem.set(candidate + 0x50, 0x1234);
        mem.set(candidate + 0x108, 0xDEAD);
        let lines = format_route_candidate_table_lines(&mem, game_ctrl);
        assert!(lines.iter().any(|l| l.contains("candidate table source=+0x3AC0")));
        assert!(lines.iter().any(|l| l.contains("  +0x50 = 0x1234")));
        assert!(lines.iter().any(|l| l.contains("  +0x00 = 0x0")));
        assert!(!lines.iter().any(|l| l.contains("  +0x108 =")));
    }

    #[test]
    fn route_candidate_table_logs_source_read_failed() {
        let mem = FakeMem::default();
        let game_ctrl = 0x10_0000usize;
        let lines = format_route_candidate_table_lines(&mem, game_ctrl);
        assert!(lines
            .iter()
            .any(|l| l.contains("candidate source game_ctrl+0x3AC0 read_failed=missing")));
    }

    #[test]
    fn route_candidate_table_only_does_not_invoke_chain_walk() {
        with_enable_file("truckpilot_route_resolver.route_candidate_table", || {
            let mem = FakeMem::default();
            let game_ctrl = 0x10_0000usize;
            let chain_err =
                resolve_route_chain_with_policy(&mem, game_ctrl, RouteScanPolicy::safe_default());
            assert!(chain_err.is_err());
            let status = run_route_candidate_table_only_diagnostic(game_ctrl);
            assert!(
                status == RESOLVE_ROUTE_CANDIDATE_TABLE_DONE
                    || status == RESOLVE_ROUTE_CANDIDATE_TABLE_READ_FAILED
            );
        });
    }

    #[test]
    fn game_ctrl_table_only_rejects_null_game_ctrl() {
        with_enable_file("truckpilot_route_resolver.game_ctrl_table", || {
            assert_eq!(
                run_game_ctrl_table_only_diagnostic(0),
                RESOLVE_GAME_CTRL_TABLE_READ_FAILED
            );
        });
    }

    #[test]
    fn game_ctrl_table_known_offsets_always_logged_even_if_zero() {
        let mut mem = FakeMem::default();
        let game_ctrl = 0x10_0000usize;
        mem.set(game_ctrl + GPS_OFFSET_IN_GAME_CTRL, 0);
        mem.set(game_ctrl + 0x3E00, 0);
        mem.set(game_ctrl + 0x3E28, 0);
        mem.set(game_ctrl + 0x3E80, 0);
        let lines = format_game_ctrl_table_lines_mem(&mem, game_ctrl);
        assert!(lines.iter().any(|l| l.contains("known gps offset +0x3E30 = 0x0")));
        assert!(lines.iter().any(|l| l.contains("game_ctrl slot +0x3E28 = 0x0")));
        assert!(lines.iter().any(|l| l.contains("game_ctrl slot +0x3E00 = 0x0")));
        assert!(lines.iter().any(|l| l.contains("game_ctrl slot +0x3E80 = 0x0")));
    }

    #[test]
    fn game_ctrl_table_logs_nonzero_slots_outside_known_range() {
        let mut mem = FakeMem::default();
        let game_ctrl = 0x10_0000usize;
        mem.set(game_ctrl + 0x100, 0xDEAD_BEEF);
        mem.set(game_ctrl + GPS_OFFSET_IN_GAME_CTRL, 0);
        let lines = format_game_ctrl_table_lines_mem(&mem, game_ctrl);
        assert!(lines.iter().any(|l| l.contains("game_ctrl slot +0x0100 = 0xDEADBEEF")));
        assert!(!lines.iter().any(|l| l.contains("game_ctrl slot +0x0108 =")));
    }

    #[test]
    fn gps_offset_probe_only_rejects_null_game_ctrl() {
        with_enable_file("truckpilot_route_resolver.gps_offset_probe", || {
            assert_eq!(
                run_gps_offset_probe_diagnostic(0),
                RESOLVE_GPS_OFFSET_PROBE_READ_FAILED
            );
        });
    }

    #[test]
    fn gps_offset_probe_reads_single_offset_via_mem() {
        let mut mem = FakeMem::default();
        let game_ctrl = 0x10_0000usize;
        mem.set(game_ctrl + GPS_OFFSET_PROBE_OFFSET, 0xDEAD_BEEF_CAFE);
        let lines = format_gps_offset_probe_lines_mem(&mem, game_ctrl);
        assert!(lines.iter().any(|l| l.contains("singleton_rva=0x354F398")));
        assert!(lines.iter().any(|l| l.contains("offset=0x40F8")));
        assert!(lines
            .iter()
            .any(|l| l.contains("gps offset probe read game_ctrl+0x40F8 value=0xDEADBEEFCAFE")));
        assert!(lines
            .iter()
            .any(|l| l.contains("gps offset probe done; parking resolver")));
    }

    #[test]
    fn gps_offset_probe_mem_performs_exactly_one_read() {
        struct CountingMem {
            inner: FakeMem,
            reads: std::cell::Cell<u32>,
        }
        impl MemRead for CountingMem {
            fn read_u64(&self, addr: usize) -> Option<u64> {
                self.reads.set(self.reads.get() + 1);
                self.inner.read_u64(addr)
            }

            fn read_u32(&self, addr: usize) -> Option<u32> {
                self.inner.read_u32(addr)
            }
        }
        let mut inner = FakeMem::default();
        let game_ctrl = 0x10_0000usize;
        inner.set(game_ctrl + GPS_OFFSET_PROBE_OFFSET, 0x1234);
        let mem = CountingMem {
            inner,
            reads: std::cell::Cell::new(0),
        };
        let lines = format_gps_offset_probe_lines_mem(&mem, game_ctrl);
        assert_eq!(mem.reads.get(), 1);
        assert!(lines.iter().any(|l| l.contains("value=0x1234")));
    }

    #[test]
    fn gps_offset_probe_only_does_not_invoke_chain_walk() {
        with_enable_file("truckpilot_route_resolver.gps_offset_probe", || {
            let mem = FakeMem::default();
            let game_ctrl = 0x10_0000usize;
            let chain_err =
                resolve_route_chain_with_policy(&mem, game_ctrl, RouteScanPolicy::safe_default());
            assert!(chain_err.is_err());
            let status = run_gps_offset_probe_diagnostic(game_ctrl);
            assert!(
                status == RESOLVE_GPS_OFFSET_PROBE_DONE
                    || status == RESOLVE_GPS_OFFSET_PROBE_READ_FAILED
            );
            assert_ne!(status, RESOLVE_WAYPOINTS_COLLECTED);
        });
    }

    #[test]
    fn game_ctrl_table_slot_count_is_2560() {
        assert_eq!(GAME_CTRL_TABLE_SLOT_COUNT, 2560);
        assert!(game_ctrl_offset_always_logged(0x3E30));
        assert!(game_ctrl_offset_always_logged(0x3E28));
    }

    #[test]
    fn game_ctrl_table_only_does_not_invoke_chain_walk() {
        with_enable_file("truckpilot_route_resolver.game_ctrl_table", || {
            let mem = FakeMem::default();
            let game_ctrl = 0x10_0000usize;
            let chain_err =
                resolve_route_chain_with_policy(&mem, game_ctrl, RouteScanPolicy::safe_default());
            assert!(chain_err.is_err());
            let status = run_game_ctrl_table_only_diagnostic(game_ctrl);
            assert!(
                status == RESOLVE_GAME_CTRL_TABLE_ONLY_DONE
                    || status == RESOLVE_GAME_CTRL_TABLE_READ_FAILED
            );
        });
    }

    #[test]
    fn gps_table_only_rejects_null_gps() {
        with_enable_file("truckpilot_route_resolver.gps_table", || {
            assert_eq!(
                run_gps_table_only_diagnostic(0),
                RESOLVE_GPS_TABLE_READ_FAILED
            );
        });
    }

    #[test]
    fn gps_table_only_format_lists_all_slots_via_mem() {
        let mut mem = FakeMem::default();
        let gps = 0x10_0000usize;
        mem.set(gps, 0xAA);
        mem.set(gps + OFF_SIMPLE_ROUTE_SRC, 0);
        mem.set(gps + 0x10, 0x20_0000);
        let lines = format_gps_table_only_lines_mem(&mem, gps);
        assert!(lines.iter().any(|l| l.contains("gps pointer table gps=0x100000")));
        assert!(lines.iter().any(|l| l.contains("+0x00 = 0xAA")));
        assert!(lines.iter().any(|l| l.contains("+0x08 = 0x0")));
        assert!(lines.iter().any(|l| l.contains("+0x10 = 0x200000")));
        assert!(lines.iter().any(|l| l.contains("gps pointer table done slots=33")));
        assert_eq!(lines.iter().filter(|l| l.starts_with("+0x")).count(), 33);
    }

    #[test]
    fn gps_table_only_mem_reports_read_failed_slots() {
        let mem = FakeMem::default();
        let gps = 0x10_0000usize;
        let lines = format_gps_table_only_lines_mem(&mem, gps);
        assert!(lines.iter().any(|l| l.contains("+0x00 read_failed=")));
        assert!(lines.iter().any(|l| l.contains("done slots=33")));
    }

    #[test]
    fn gps_table_only_does_not_invoke_chain_walk() {
        with_enable_file("truckpilot_route_resolver.gps_table", || {
            let mem = FakeMem::default();
            let gps = 0x10_0000usize;
            let chain_err =
                resolve_route_chain_with_policy(&mem, gps, RouteScanPolicy::safe_default());
            assert!(chain_err.is_err());
            let status = run_gps_table_only_diagnostic(gps);
            assert_ne!(status, RESOLVE_WAYPOINTS_COLLECTED);
            assert!(
                status == RESOLVE_GPS_TABLE_ONLY_DONE || status == RESOLVE_GPS_TABLE_READ_FAILED
            );
        });
    }

    #[test]
    fn gps_pointer_table_contains_known_offsets() {
        let mut mem = FakeMem::default();
        let gps = 0x10_0000usize;
        mem.set(gps + OFF_SIMPLE_ROUTE_SRC, 0);
        mem.set(gps + 0x10, 0x20_0000);
        let lines = format_gps_pointer_table_lines(&mem, gps);
        assert!(lines.iter().any(|l| l.contains("gps pointer table")));
        assert!(lines.iter().any(|l| l.contains("+0x08 = 0x0")));
        assert!(lines.iter().any(|l| l.contains("+0x10 = 0x200000")));
    }

    #[test]
    fn srs_dynamic_offset_scan_finds_srs_at_0x18() {
        let mut mem = FakeMem::default();
        let gps = 0x10_0000usize;
        let srs = 0x20_0000usize;
        mem.set(gps + OFF_SIMPLE_ROUTE_SRC, 0);
        mem.set(gps + 0x18, srs as u64);
        build_v159_long(&mut mem, gps);
        mem.set(gps + OFF_SIMPLE_ROUTE_SRC, 0);
        mem.set(gps + 0x18, srs as u64);
        let ok = resolve_route_chain_with_policy(&mem, gps, full_scan_policy()).unwrap();
        assert!(
            ok.candidate == "srs_dyn" || ok.candidate == "v159_long",
            "candidate={}",
            ok.candidate
        );
        assert!(ok.item_count >= 2);
    }

    #[test]
    fn direct_route_task_scan_finds_items() {
        let mut mem = FakeMem::default();
        let gps = 0x10_0000usize;
        let rt = 0x30_0000usize;
        let items = 0x40_0000usize;
        mem.set(gps + OFF_SIMPLE_ROUTE_SRC, 0);
        mem.set(gps + 0x28, rt as u64);
        mem.set(rt + OFF_PHYS_ITEMS, items as u64);
        let uid = UID_MIN_PLAUSIBLE + 99;
        mem.set(items + OFF_ITEM_UID, uid);
        mem.set(items + OFF_ITEM_ACTIVE, 1);
        mem.set(items + ITEM_STRIDE + OFF_ITEM_UID, uid + 1);
        mem.set(items + ITEM_STRIDE + OFF_ITEM_ACTIVE, 1);
        let ok = resolve_route_chain_with_policy(&mem, gps, full_scan_policy()).unwrap();
        assert!(
            ok.candidate == "direct_route_task" || ok.candidate == "direct_items_owner"
        );
        assert_eq!(ok.first_uid, uid);
    }

    #[test]
    fn rate_log_limits_repeated_candidate_messages() {
        let mut rl = RateLog::new();
        for _ in 0..RateLog::LOG_BURST {
            assert!(rl.should_log("chain:0:fail", 0, false));
            rl.record("chain:0:fail", 0);
        }
        assert!(!rl.should_log("chain:0:fail", RateLog::LOG_INTERVAL_US - 1, false));
        assert!(rl.should_log("chain:0:fail", RateLog::LOG_INTERVAL_US, false));
    }

    #[test]
    fn items_null_maps_status() {
        let mut mem = FakeMem::default();
        let gps = 0x10_0000usize;
        let rt = build_v159_long(&mut mem, gps);
        mem.words.remove(&(rt + OFF_PHYS_ITEMS));
        mem.words.remove(&(rt + OFF_PHYS_ITEMS_ALT));
        let f = resolve_route_chain_with_policy(&mem, gps, full_scan_policy()).unwrap_err();
        assert!(
            f.status == RESOLVE_ROUTE_ITEMS_CANDIDATE_NULL
                || failure_to_status(&f) == RESOLVE_ROUTE_CHAIN_ALL_FAILED
        );
    }

    #[test]
    fn first_uid_zero_maps_status() {
        let mut mem = FakeMem::default();
        let gps = 0x10_0000usize;
        let rt = build_v159_long(&mut mem, gps);
        let items = mem.read_u64(rt + OFF_PHYS_ITEMS).unwrap() as usize;
        mem.set(items + OFF_ITEM_UID, 0);
        let f = try_candidate(&mem, gps, &CHAIN_CANDIDATES[0]).unwrap_err();
        assert_eq!(f.status, RESOLVE_FIRST_UID_ZERO);
    }

    #[test]
    fn skip_b_chain_when_b_null() {
        let mut mem = FakeMem::default();
        let gps = 0x10_0000usize;
        let srs = 0x20_0000usize;
        let a = 0x30_0000;
        let rt_ref = 0x50_0000;
        let rt = rt_ref + OFF_ROUTE_TASK_BIAS;
        let items = 0x60_0000;
        mem.set(gps + OFF_SIMPLE_ROUTE_SRC, srs as u64);
        mem.set(srs + OFF_SRS_ROUTE_A, a as u64);
        mem.set(a + OFF_ROUTE_TASK_REF, rt_ref as u64);
        mem.set(rt + OFF_PHYS_ITEMS, items as u64);
        let uid = UID_MIN_PLAUSIBLE + 1;
        mem.set(items + OFF_ITEM_UID, uid);
        mem.set(items + OFF_ITEM_ACTIVE, 1);
        mem.set(items + ITEM_STRIDE + OFF_ITEM_UID, uid + 1);
        mem.set(items + ITEM_STRIDE + OFF_ITEM_ACTIVE, 1);
        let ok = resolve_route_chain_with_policy(&mem, gps, full_scan_policy()).unwrap();
        assert_eq!(ok.candidate, "v159_skip_b");
    }
}
