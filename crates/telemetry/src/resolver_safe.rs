//! Display-only resolver safety tri-state from existing SHM snapshots.
//!
//! Does not activate the resolver or read ETS2 process memory — only interprets
//! route-blackboard + perf-SHM fields already exposed by `truckpilot-status`.

use crate::nav_route::{
    RESOLVE_GAME_CTRL_TABLE_ONLY_DONE, RESOLVE_GPS_RESOLVED_ROUTE_SCAN_DISABLED,
    RESOLVE_GPS_TABLE_ONLY_DONE, RESOLVE_NONE, RESOLVE_PAUSED_NO_ROUTE_WALK,
    RESOLVE_ROUTE_CANDIDATE_TABLE_DONE, RESOLVE_ROUTE_RESOLVER_BACKOFF,
    RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE, RESOLVE_ROUTE_RESOLVER_PARKED,
    RESOLVE_ROUTE_RESOLVER_SCAN_LIMITED, RESOLVE_ROUTE_RESOLVER_WORKER_ACTIVE,
    RESOLVE_ROUTE_SCAN_WAITING_FOR_STABLE_WORLD, RESOLVE_ROUTE_SCAN_WARMUP,
    RESOLVE_TICK_THROTTLED, RESOLVE_UNSAFE_ROUTE_SCAN_DISABLED,
};

use crate::preflight::TriState;

/// Blackboard key mirrored by core for overlay preflight (display-only).
pub const PREFLIGHT_RESOLVER_SAFE_KEY: &str = "preflight.resolver_safe";

/// Inputs for resolver safety classification (SHM-only).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResolverSafeInputs {
    /// Route blackboard SHM mapped.
    pub route_bb_available: bool,
    /// DLL perf SHM mapped.
    pub perf_shm_available: bool,
    /// Raw `resolve_status` code from route blackboard.
    pub resolve_status: u32,
    /// Cumulative resolver attempts from perf SHM.
    pub resolver_attempts: u32,
    /// Pattern scan counter from perf SHM.
    pub pattern_scan_count: u32,
}

impl ResolverSafeInputs {
    /// Build from a status report (route code must be populated when route BB exists).
    pub fn from_status_report(
        route_bb_available: bool,
        perf_shm_available: bool,
        resolve_status_code: Option<u32>,
        resolver_attempts: u32,
        pattern_scan_count: u32,
    ) -> Self {
        Self {
            route_bb_available,
            perf_shm_available,
            resolve_status: resolve_status_code.unwrap_or(RESOLVE_NONE),
            resolver_attempts,
            pattern_scan_count,
        }
    }
}

/// Classify resolver safety for preflight display.
///
/// `true` = known safe (parked / disabled safe mode, no hot scans).
/// `false` = known unsafe (active worker, scans, or attempts).
/// `None` = no SHM or ambiguous status mapping.
pub fn evaluate_resolver_safe(inp: &ResolverSafeInputs) -> TriState {
    if !inp.route_bb_available && !inp.perf_shm_available {
        return None;
    }

    if inp.pattern_scan_count > 0 || inp.resolver_attempts > 0 {
        return Some(false);
    }

    if inp.route_bb_available {
        return classify_resolve_status(inp.resolve_status);
    }

    // Perf-SHM only: counters quiet → treat as safe/parked.
    Some(true)
}

fn classify_resolve_status(code: u32) -> TriState {
    match code {
        RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE
        | RESOLVE_ROUTE_RESOLVER_PARKED
        | RESOLVE_NONE
        | RESOLVE_PAUSED_NO_ROUTE_WALK
        | RESOLVE_TICK_THROTTLED
        | RESOLVE_UNSAFE_ROUTE_SCAN_DISABLED
        | RESOLVE_GPS_RESOLVED_ROUTE_SCAN_DISABLED
        | RESOLVE_ROUTE_SCAN_WARMUP
        | RESOLVE_ROUTE_SCAN_WAITING_FOR_STABLE_WORLD
        | RESOLVE_GPS_TABLE_ONLY_DONE
        | RESOLVE_GAME_CTRL_TABLE_ONLY_DONE
        | RESOLVE_ROUTE_CANDIDATE_TABLE_DONE => Some(true),

        RESOLVE_ROUTE_RESOLVER_WORKER_ACTIVE
        | RESOLVE_ROUTE_RESOLVER_BACKOFF
        | RESOLVE_ROUTE_RESOLVER_SCAN_LIMITED => Some(false),

        _ => None,
    }
}

/// Human label for resolver safety tri-state (`yes` / `no` / `unknown`).
pub fn resolver_safe_label(v: TriState) -> &'static str {
    match v {
        Some(true) => "yes",
        Some(false) => "no",
        None => "unknown",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nav_route::RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE;

    #[test]
    fn safe_off_disabled_safe_mode_is_yes() {
        let v = evaluate_resolver_safe(&ResolverSafeInputs {
            route_bb_available: true,
            perf_shm_available: true,
            resolve_status: RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE,
            resolver_attempts: 0,
            pattern_scan_count: 0,
        });
        assert_eq!(v, Some(true));
    }

    #[test]
    fn pattern_scan_count_forces_no() {
        let v = evaluate_resolver_safe(&ResolverSafeInputs {
            route_bb_available: true,
            perf_shm_available: true,
            resolve_status: RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE,
            resolver_attempts: 0,
            pattern_scan_count: 1,
        });
        assert_eq!(v, Some(false));
    }

    #[test]
    fn resolver_attempts_forces_no_even_in_safe_mode() {
        let v = evaluate_resolver_safe(&ResolverSafeInputs {
            route_bb_available: true,
            perf_shm_available: true,
            resolve_status: RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE,
            resolver_attempts: 2,
            pattern_scan_count: 0,
        });
        assert_eq!(v, Some(false));
    }

    #[test]
    fn worker_active_is_no_with_quiet_counters() {
        let v = evaluate_resolver_safe(&ResolverSafeInputs {
            route_bb_available: true,
            perf_shm_available: true,
            resolve_status: RESOLVE_ROUTE_RESOLVER_WORKER_ACTIVE,
            resolver_attempts: 0,
            pattern_scan_count: 0,
        });
        assert_eq!(v, Some(false));
    }

    #[test]
    fn no_shm_is_unknown() {
        let v = evaluate_resolver_safe(&ResolverSafeInputs {
            route_bb_available: false,
            perf_shm_available: false,
            resolve_status: RESOLVE_NONE,
            resolver_attempts: 0,
            pattern_scan_count: 0,
        });
        assert_eq!(v, None);
    }
}
