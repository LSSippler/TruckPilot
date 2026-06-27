//! Read-only TruckPilot DLL health summary from existing SHM readers.
//!
//! No daemon, no game-memory reads, no steering, no resolver activation.
//! Used by `truckpilot-status` and the overlay snapshot layer.

#[cfg(windows)]
use crate::dll_perf::{diag_level_name, DllPerfReader, DllPerfSnapshot};
use crate::core_readiness::{format_core_readiness_human, CoreReadiness};
use crate::nav_route::{
    route_resolve_status_name, RouteBlackboardReader, RouteSnapshot,
    RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE, ROUTE_BB_STATUS_DLL_ACTIVE,
};
use crate::shm::ShmReader;

/// Overall safety verdict for the telemetry DLL hotpath.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum StatusVerdict {
    /// DLL present, resolver off, nothing hot — stable diagnostic state.
    SafeCold,
    /// DLL present but worker/scans/stutter detected (see `reasons`).
    Hot,
    /// No TruckPilot SHM found.
    Unavailable,
}

/// Raw inputs gathered from SHM (separated from logic so it is testable offline).
#[derive(Debug, Clone, Default)]
pub struct RawStatusInputs {
    /// Perf SHM snapshot, if mapped (Windows only — `dll_perf` uses `std::os::windows`).
    #[cfg(windows)]
    pub perf: Option<DllPerfSnapshot>,
    /// Route blackboard snapshot, if mapped.
    pub route: Option<RouteSnapshot>,
    /// Whether telemetry SHM could be opened (even if not read this cycle).
    pub telemetry_shm_present: bool,
}

/// Derived, human/JSON-renderable status.
#[derive(Debug, Clone, serde::Serialize)]
pub struct StatusReport {
    /// Whether the TruckPilot DLL is considered active (Perf-SHM present or DLL-active bit set).
    pub dll_active: bool,
    /// Whether the performance SHM segment could be mapped.
    pub perf_shm_available: bool,
    /// Whether the route blackboard SHM segment could be mapped.
    pub route_bb_available: bool,
    /// Whether the main telemetry SHM segment is present (ETS2 running + DLL loaded).
    pub telemetry_shm_present: bool,
    /// Diagnostic level name as reported by the DLL (e.g. `"normal_default_off"`).
    pub diag_level: String,
    /// `true` when the resolver is in safe-off mode and `resolver_attempts == 0`.
    pub resolver_off: bool,
    /// Human-readable resolve status name from the route blackboard.
    pub resolve_status: String,
    /// Cumulative resolver activation count (>0 means hot).
    pub resolver_attempts: u32,
    /// `true` when `input_enabled == false` (game input intercepted by DLL).
    pub input_disabled: bool,
    /// `true` when the DLL has input interception enabled.
    pub input_enabled: bool,
    /// `true` when the worker thread has never woken (walk and wake counts both zero).
    pub worker_asleep: bool,
    /// Number of times the worker thread walked its task list.
    pub worker_walk_count: u32,
    /// Number of times the worker thread was woken via `SetEvent`.
    pub worker_wake_set_event_count: u32,
    /// Number of times the worker skipped a cycle because it was parked.
    pub worker_parked_skip_count: u32,
    /// Total number of pattern scans performed (>0 means hot).
    pub pattern_scan_count: u32,
    /// Total number of D3D12 present callbacks fired.
    pub frame_cb_count: u32,
    /// Maximum frame-callback duration in microseconds.
    pub frame_cb_us_max: u64,
    /// Number of frame callbacks that exceeded 1 000 µs (stutter indicator).
    pub frame_cb_over_1000us: u64,
    /// Whether the route blackboard reports a valid routed path.
    pub route_valid: bool,
    /// Number of waypoints in the current route (0 when route is invalid).
    pub waypoint_count: u32,
    /// Overall safety verdict derived from all inputs.
    pub verdict: StatusVerdict,
    /// Why the verdict is `Hot` (empty otherwise).
    pub reasons: Vec<String>,
    /// Core daemon readiness (blackboard); unavailable when CLI has no daemon connection.
    pub core_readiness: CoreReadiness,
}

/// Map a verdict to the process exit code for CLI tools.
pub fn status_exit_code(v: StatusVerdict) -> i32 {
    match v {
        StatusVerdict::SafeCold => 0,
        StatusVerdict::Unavailable => 1,
        StatusVerdict::Hot => 2,
    }
}

/// Read current SHM inputs without evaluation.
pub fn read_raw_inputs() -> RawStatusInputs {
    RawStatusInputs {
        #[cfg(windows)]
        perf: DllPerfReader::open().ok().and_then(|r| r.read()),
        route: RouteBlackboardReader::open().ok().and_then(|r| r.read()),
        telemetry_shm_present: ShmReader::open().is_ok(),
    }
}

/// Read and evaluate the current DLL status from live SHM.
pub fn read_live_status() -> StatusReport {
    let inp = read_raw_inputs();
    evaluate_status(&inp)
}

/// Pure status evaluation from raw SHM inputs.
pub fn evaluate_status(inp: &RawStatusInputs) -> StatusReport {
    #[cfg(windows)]
    let perf_shm_available = inp.perf.is_some();
    #[cfg(not(windows))]
    let perf_shm_available = false;
    let route_bb_available = inp.route.is_some();

    if !perf_shm_available && !route_bb_available {
        return StatusReport {
            dll_active: false,
            perf_shm_available: false,
            route_bb_available: false,
            telemetry_shm_present: inp.telemetry_shm_present,
            diag_level: "unknown".into(),
            resolver_off: false,
            resolve_status: "unavailable".into(),
            resolver_attempts: 0,
            input_disabled: false,
            input_enabled: false,
            worker_asleep: false,
            worker_walk_count: 0,
            worker_wake_set_event_count: 0,
            worker_parked_skip_count: 0,
            pattern_scan_count: 0,
            frame_cb_count: 0,
            frame_cb_us_max: 0,
            frame_cb_over_1000us: 0,
            route_valid: false,
            waypoint_count: 0,
            verdict: StatusVerdict::Unavailable,
            reasons: vec!["keine TruckPilot-SHM gefunden (ETS2 aus oder DLL nicht geladen)".into()],
            core_readiness: CoreReadiness::unavailable(),
        };
    }

    let route = inp.route.as_ref();

    let dll_active = route
        .map(|r| r.bb_status & ROUTE_BB_STATUS_DLL_ACTIVE != 0)
        .unwrap_or(false)
        || perf_shm_available;

    // Perf-SHM values (Windows only; all default to zero/unknown on other platforms).
    #[cfg(windows)]
    let (diag_level, resolver_attempts_from_perf, pattern_scan_count,
         worker_walk_count, worker_wake_set_event_count, worker_parked_skip_count,
         input_enabled, frame_cb_count_from_perf, frame_cb_us_max, frame_cb_over_1000us) = {
        let p = inp.perf.as_ref();
        (
            p.map(|p| diag_level_name(p.diag_level_code).to_string())
             .unwrap_or_else(|| "unknown".into()),
            p.map(|p| p.resolver_attempts),
            p.map(|p| p.pattern_scan_count).unwrap_or(0),
            p.map(|p| p.worker_walk_count).unwrap_or(0),
            p.map(|p| p.worker_wake_set_event_count).unwrap_or(0),
            p.map(|p| p.worker_parked_skip_count).unwrap_or(0),
            p.map(|p| p.input_enabled != 0).unwrap_or(false),
            p.map(|p| p.frame_cb_count),
            p.map(|p| p.buckets[0].max_us).unwrap_or(0),
            p.map(|p| p.buckets[0].over_1000us).unwrap_or(0),
        )
    };
    #[cfg(not(windows))]
    let (diag_level, resolver_attempts_from_perf, pattern_scan_count,
         worker_walk_count, worker_wake_set_event_count, worker_parked_skip_count,
         input_enabled, frame_cb_count_from_perf, frame_cb_us_max, frame_cb_over_1000us):
        (String, Option<u32>, u32, u32, u32, u32, bool, Option<u32>, u64, u64) =
        ("unknown".to_string(), None, 0, 0, 0, 0, false, None, 0, 0);

    let resolver_attempts = resolver_attempts_from_perf
        .or_else(|| route.map(|r| r.resolve_attempts))
        .unwrap_or(0);
    let worker_asleep = worker_walk_count == 0 && worker_wake_set_event_count == 0;
    let input_disabled = !input_enabled;
    let frame_cb_count = frame_cb_count_from_perf
        .or_else(|| route.map(|r| r.frame_cb_count))
        .unwrap_or(0);

    let resolve_status = route
        .map(|r| route_resolve_status_name(r.resolve_status).to_string())
        .unwrap_or_else(|| "unknown".into());
    let resolver_off = route
        .map(|r| r.resolve_status == RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE)
        .unwrap_or(true)
        && resolver_attempts == 0;
    let route_valid = route.map(|r| r.valid).unwrap_or(false);
    let waypoint_count = route
        .map(|r| {
            if r.waypoints.is_empty() {
                r.last_waypoint_count
            } else {
                r.waypoints.len() as u32
            }
        })
        .unwrap_or(0);

    let mut reasons = Vec::new();
    if resolver_attempts > 0 {
        reasons.push(format!("resolver_attempts={resolver_attempts} (>0)"));
    }
    if pattern_scan_count > 0 {
        reasons.push(format!("pattern_scan_count={pattern_scan_count} (>0)"));
    }
    if worker_walk_count > 0 {
        reasons.push(format!("worker_walk_count={worker_walk_count} (>0)"));
    }
    if frame_cb_over_1000us > 0 {
        reasons.push(format!(
            "frame_cb_over_1000us={frame_cb_over_1000us} (>0, Stutter)"
        ));
    }

    let verdict = if reasons.is_empty() {
        StatusVerdict::SafeCold
    } else {
        StatusVerdict::Hot
    };

    StatusReport {
        dll_active,
        perf_shm_available,
        route_bb_available,
        telemetry_shm_present: inp.telemetry_shm_present,
        diag_level,
        resolver_off,
        resolve_status,
        resolver_attempts,
        input_disabled,
        input_enabled,
        worker_asleep,
        worker_walk_count,
        worker_wake_set_event_count,
        worker_parked_skip_count,
        pattern_scan_count,
        frame_cb_count,
        frame_cb_us_max,
        frame_cb_over_1000us,
        route_valid,
        waypoint_count,
        verdict,
        reasons,
        core_readiness: CoreReadiness::unavailable(),
    }
}

fn yes_no(b: bool) -> &'static str {
    if b {
        "ja"
    } else {
        "nein"
    }
}

/// Human-readable status block.
pub fn format_status_human(r: &StatusReport) -> String {
    if r.verdict == StatusVerdict::Unavailable {
        return format!(
            "TruckPilot Status  (read-only, kein Daemon)\n  \
             keine TruckPilot-SHM gefunden.\n  \
             Ursachen: ETS2 läuft nicht · truckpilot_telemetry.dll nicht geladen · DLL hat SHM nicht erstellt.\n  \
             ────────────────────────────────────────────\n  \
             GESAMT: UNAVAILABLE (exit 1)\n  \
             ────────────────────────────────────────────\n  \
             {}",
            format_core_readiness_human(&r.core_readiness),
        );
    }

    let resolver_line = if r.resolver_off {
        format!(
            "off           ({}, attempts={})",
            r.resolve_status, r.resolver_attempts
        )
    } else {
        format!(
            "AKTIV/HEISS   ({}, attempts={})",
            r.resolve_status, r.resolver_attempts
        )
    };
    let worker_line = if r.worker_asleep {
        format!(
            "schläft       (walk={}, wake={}, parked_skip={})",
            r.worker_walk_count, r.worker_wake_set_event_count, r.worker_parked_skip_count
        )
    } else {
        format!(
            "AKTIV         (walk={}, wake={}, parked_skip={})",
            r.worker_walk_count, r.worker_wake_set_event_count, r.worker_parked_skip_count
        )
    };
    let input_line = if r.input_disabled {
        "disabled      (input_enabled=false)".to_string()
    } else {
        "aktiv         (input_enabled=true)".to_string()
    };
    let frame_line = if r.frame_cb_count == 0 {
        "keine Frames  (ETS2 nicht im Spiel/Menü/aus?)".to_string()
    } else if r.frame_cb_over_1000us > 0 {
        format!(
            "STUTTER       (count={}, us_max={}, over_1000us={})",
            r.frame_cb_count, r.frame_cb_us_max, r.frame_cb_over_1000us
        )
    } else {
        format!(
            "ok            (count={}, us_max={}, over_1000us=0)",
            r.frame_cb_count, r.frame_cb_us_max
        )
    };

    let verdict_line = match r.verdict {
        StatusVerdict::SafeCold => "GESAMT:           SICHER-KALT  (exit 0)".to_string(),
        StatusVerdict::Hot => format!(
            "GESAMT:           HEISS/UNSAFE (exit 2) — {}",
            r.reasons.join(", ")
        ),
        StatusVerdict::Unavailable => "GESAMT:           UNAVAILABLE  (exit 1)".to_string(),
    };

    format!(
        "TruckPilot Status  (read-only, kein Daemon)\n  \
         DLL aktiv:        {}            (Perf-SHM={}, RouteBlackboard={})\n  \
         Diag-Level:       {}\n  \
         Resolver:         {}\n  \
         Worker:           {}\n  \
         Pattern-Scans:    {}\n  \
         Input:            {}\n  \
         Frame-Callback:   {}\n  \
         Route gültig:     {} (waypoints={})\n  \
         Telemetry-SHM:    {}\n  \
         ────────────────────────────────────────────\n  \
         {}\n  \
         ────────────────────────────────────────────\n  \
         {}",
        yes_no(r.dll_active),
        yes_no(r.perf_shm_available),
        yes_no(r.route_bb_available),
        r.diag_level,
        resolver_line,
        worker_line,
        r.pattern_scan_count,
        input_line,
        frame_line,
        yes_no(r.route_valid),
        r.waypoint_count,
        if r.telemetry_shm_present {
            "vorhanden"
        } else {
            "nicht gefunden"
        },
        verdict_line,
        format_core_readiness_human(&r.core_readiness),
    )
}

/// Pretty JSON status.
pub fn format_status_json(r: &StatusReport) -> String {
    serde_json::to_string_pretty(r).unwrap_or_else(|e| format!("{{\"json_error\":\"{e}\"}}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(windows)]
    use crate::dll_perf::{DLL_PERF_MAGIC, DLL_PERF_VERSION};

    #[cfg(windows)]
    fn safe_off_perf() -> DllPerfSnapshot {
        DllPerfSnapshot {
            magic: DLL_PERF_MAGIC,
            version: DLL_PERF_VERSION,
            diag_level_code: 9,
            resolver_attempts: 0,
            pattern_scan_count: 0,
            worker_walk_count: 0,
            worker_wake_set_event_count: 0,
            input_enabled: 0,
            frame_cb_count: 12345,
            ..Default::default()
        }
    }

    fn safe_off_route() -> RouteSnapshot {
        RouteSnapshot {
            bb_status: ROUTE_BB_STATUS_DLL_ACTIVE,
            resolve_status: RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE,
            resolve_attempts: 0,
            valid: false,
            last_waypoint_count: 0,
            ..Default::default()
        }
    }

    #[cfg(windows)]
    #[test]
    fn safe_off_is_cold_and_exit_zero() {
        let r = evaluate_status(&RawStatusInputs {
            perf: Some(safe_off_perf()),
            route: Some(safe_off_route()),
            telemetry_shm_present: true,
        });
        assert_eq!(r.verdict, StatusVerdict::SafeCold);
        assert_eq!(status_exit_code(r.verdict), 0);
        assert_eq!(r.diag_level, "normal_default_off");
        assert!(!r.route_valid);
        assert_eq!(r.waypoint_count, 0);
    }

    #[cfg(windows)]
    #[test]
    fn resolver_attempts_makes_it_hot() {
        let mut perf = safe_off_perf();
        perf.resolver_attempts = 3;
        let r = evaluate_status(&RawStatusInputs {
            perf: Some(perf),
            route: Some(safe_off_route()),
            telemetry_shm_present: true,
        });
        assert_eq!(status_exit_code(r.verdict), 2);
    }

    #[test]
    fn no_shm_is_unavailable_exit_one() {
        let r = evaluate_status(&RawStatusInputs::default());
        assert_eq!(r.verdict, StatusVerdict::Unavailable);
        assert_eq!(status_exit_code(r.verdict), 1);
        assert!(!r.core_readiness.available);
    }
}
