//! `truckpilot-status` — read-only ETS2 DLL health/safety summary.
//!
//! Reads only shared memory (no daemon, no game-memory reads, no resolver, no
//! steering) and reports the stable diagnostic state in plain language:
//! DLL active, resolver off, input disabled, diag level, worker asleep,
//! pattern scans 0, frame callback ok.
//!
//! ```text
//! cargo run -p truckpilot-telemetry --bin truckpilot-status
//! cargo run -p truckpilot-telemetry --bin truckpilot-status -- --json
//! ```
//!
//! Exit codes:
//! - `0` — DLL present and SAFE-COLD (resolver off, no worker/pattern/stutter).
//! - `1` — no SHM found (ETS2 not running or DLL not loaded).
//! - `2` — DLL present but HOT/unsafe: `resolver_attempts > 0`,
//!   `pattern_scan_count > 0`, `worker_walk_count > 0`, or `frame_cb_over_1000us > 0`.
//!
//! `route_valid = false` is NOT an error in the safe-off default state.

use truckpilot_telemetry::dll_perf::{diag_level_name, DllPerfReader, DllPerfSnapshot};
use truckpilot_telemetry::nav_route::{
    route_resolve_status_name, RouteBlackboardReader, RouteSnapshot,
    RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE, ROUTE_BB_STATUS_DLL_ACTIVE,
};
use truckpilot_telemetry::shm::ShmReader;

/// Overall safety verdict.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "snake_case")]
enum Verdict {
    /// DLL present, resolver off, nothing hot — the stable diagnostic state.
    SafeCold,
    /// DLL present but something is running/stuttering (see `reasons`).
    Hot,
    /// No SHM found at all.
    Unavailable,
}

/// Raw inputs gathered from SHM (separated from logic so it is testable offline).
struct RawInputs<'a> {
    perf: Option<&'a DllPerfSnapshot>,
    route: Option<&'a RouteSnapshot>,
    telemetry_shm_present: bool,
}

/// Derived, human/JSON-renderable status.
#[derive(Debug, Clone, serde::Serialize)]
struct StatusReport {
    dll_active: bool,
    perf_shm_available: bool,
    route_bb_available: bool,
    telemetry_shm_present: bool,
    diag_level: String,
    resolver_off: bool,
    resolve_status: String,
    resolver_attempts: u32,
    input_disabled: bool,
    input_enabled: bool,
    worker_asleep: bool,
    worker_walk_count: u32,
    worker_wake_set_event_count: u32,
    pattern_scan_count: u32,
    frame_cb_count: u32,
    frame_cb_us_max: u64,
    frame_cb_over_1000us: u64,
    route_valid: bool,
    verdict: Verdict,
    /// Why the verdict is `Hot` (empty otherwise).
    reasons: Vec<String>,
}

/// Map a verdict to the process exit code.
fn exit_code(v: Verdict) -> i32 {
    match v {
        Verdict::SafeCold => 0,
        Verdict::Unavailable => 1,
        Verdict::Hot => 2,
    }
}

/// Pure status evaluation from raw SHM inputs.
fn evaluate(inp: &RawInputs) -> StatusReport {
    let perf_shm_available = inp.perf.is_some();
    let route_bb_available = inp.route.is_some();

    // Exit-1 condition: no SHM at all.
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
            pattern_scan_count: 0,
            frame_cb_count: 0,
            frame_cb_us_max: 0,
            frame_cb_over_1000us: 0,
            route_valid: false,
            verdict: Verdict::Unavailable,
            reasons: vec!["keine TruckPilot-SHM gefunden (ETS2 aus oder DLL nicht geladen)".into()],
        };
    }

    let perf = inp.perf;
    let route = inp.route;

    let dll_active = route
        .map(|r| r.bb_status & ROUTE_BB_STATUS_DLL_ACTIVE != 0)
        .unwrap_or(false)
        || perf_shm_available;

    let diag_level = perf
        .map(|p| diag_level_name(p.diag_level_code).to_string())
        .unwrap_or_else(|| "unknown".into());

    // Prefer the perf-SHM resolver attempts; fall back to the RouteBlackboard's.
    let resolver_attempts = perf
        .map(|p| p.resolver_attempts)
        .or_else(|| route.map(|r| r.resolve_attempts))
        .unwrap_or(0);
    let pattern_scan_count = perf.map(|p| p.pattern_scan_count).unwrap_or(0);
    let worker_walk_count = perf.map(|p| p.worker_walk_count).unwrap_or(0);
    let worker_wake_set_event_count = perf.map(|p| p.worker_wake_set_event_count).unwrap_or(0);
    let worker_asleep = worker_walk_count == 0 && worker_wake_set_event_count == 0;

    let input_enabled = perf.map(|p| p.input_enabled != 0).unwrap_or(false);
    let input_disabled = !input_enabled;

    let frame_cb_count = perf.map(|p| p.frame_cb_count).unwrap_or(0);
    let frame_cb_us_max = perf.map(|p| p.buckets[0].max_us).unwrap_or(0);
    let frame_cb_over_1000us = perf.map(|p| p.buckets[0].over_1000us).unwrap_or(0);

    let resolve_status = route
        .map(|r| route_resolve_status_name(r.resolve_status).to_string())
        .unwrap_or_else(|| "unknown".into());
    let resolver_off = route
        .map(|r| r.resolve_status == RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE)
        .unwrap_or(true)
        && resolver_attempts == 0;
    let route_valid = route.map(|r| r.valid).unwrap_or(false);

    // HOT/unsafe conditions (route_valid=false is NOT one of them).
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
        reasons.push(format!("frame_cb_over_1000us={frame_cb_over_1000us} (>0, Stutter)"));
    }

    let verdict = if reasons.is_empty() {
        Verdict::SafeCold
    } else {
        Verdict::Hot
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
        pattern_scan_count,
        frame_cb_count,
        frame_cb_us_max,
        frame_cb_over_1000us,
        route_valid,
        verdict,
        reasons,
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
fn format_human(r: &StatusReport) -> String {
    if r.verdict == Verdict::Unavailable {
        return format!(
            "TruckPilot Status  (read-only, kein Daemon)\n  \
             keine TruckPilot-SHM gefunden.\n  \
             Ursachen: ETS2 läuft nicht · truckpilot_telemetry.dll nicht geladen · DLL hat SHM nicht erstellt.\n  \
             ────────────────────────────────────────────\n  \
             GESAMT: UNAVAILABLE (exit 1)"
        );
    }

    let resolver_line = if r.resolver_off {
        format!("off           ({}, attempts={})", r.resolve_status, r.resolver_attempts)
    } else {
        format!(
            "AKTIV/HEISS   ({}, attempts={})",
            r.resolve_status, r.resolver_attempts
        )
    };
    let worker_line = if r.worker_asleep {
        format!(
            "schläft       (worker_walk_count={}, worker_wake={})",
            r.worker_walk_count, r.worker_wake_set_event_count
        )
    } else {
        format!(
            "AKTIV         (worker_walk_count={}, worker_wake={})",
            r.worker_walk_count, r.worker_wake_set_event_count
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
        Verdict::SafeCold => "GESAMT:           SICHER-KALT  (exit 0)".to_string(),
        Verdict::Hot => format!("GESAMT:           HEISS/UNSAFE (exit 2) — {}", r.reasons.join(", ")),
        Verdict::Unavailable => "GESAMT:           UNAVAILABLE  (exit 1)".to_string(),
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
         Route gültig:     {} (im Safe-Off normal & ok)\n  \
         Telemetry-SHM:    {}\n  \
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
        if r.telemetry_shm_present { "vorhanden" } else { "nicht gefunden" },
        verdict_line,
    )
}

/// Pretty JSON status.
fn format_json(r: &StatusReport) -> String {
    serde_json::to_string_pretty(r).unwrap_or_else(|e| format!("{{\"json_error\":\"{e}\"}}"))
}

fn has_flag(flag: &str) -> bool {
    std::env::args().any(|a| a == flag)
}

fn main() {
    let json = has_flag("--json");

    let perf = DllPerfReader::open().ok().and_then(|r| r.read());
    let route = RouteBlackboardReader::open().ok().and_then(|r| r.read());
    let telemetry_shm_present = ShmReader::open().is_ok();

    let report = evaluate(&RawInputs {
        perf: perf.as_ref(),
        route: route.as_ref(),
        telemetry_shm_present,
    });

    if json {
        println!("{}", format_json(&report));
    } else {
        println!("{}", format_human(&report));
    }

    std::process::exit(exit_code(report.verdict));
}

#[cfg(test)]
mod tests {
    use super::*;
    use truckpilot_telemetry::dll_perf::{DLL_PERF_MAGIC, DLL_PERF_VERSION};

    fn safe_off_perf() -> DllPerfSnapshot {
        DllPerfSnapshot {
            magic: DLL_PERF_MAGIC,
            version: DLL_PERF_VERSION,
            diag_level_code: 9, // normal_default_off
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
            valid: false, // safe-off: no active route — NOT an error
            ..Default::default()
        }
    }

    #[test]
    fn safe_off_is_cold_and_exit_zero() {
        let perf = safe_off_perf();
        let route = safe_off_route();
        let r = evaluate(&RawInputs {
            perf: Some(&perf),
            route: Some(&route),
            telemetry_shm_present: true,
        });
        assert_eq!(r.verdict, Verdict::SafeCold);
        assert_eq!(exit_code(r.verdict), 0);
        assert!(r.dll_active);
        assert!(r.resolver_off);
        assert!(r.input_disabled);
        assert!(r.worker_asleep);
        assert_eq!(r.diag_level, "normal_default_off");
        assert!(!r.route_valid, "route invalid in safe-off");
        assert!(r.reasons.is_empty());
    }

    #[test]
    fn route_invalid_alone_stays_safe_cold() {
        // route_valid=false must never push the verdict to Hot.
        let perf = safe_off_perf();
        let mut route = safe_off_route();
        route.valid = false;
        let r = evaluate(&RawInputs {
            perf: Some(&perf),
            route: Some(&route),
            telemetry_shm_present: true,
        });
        assert_eq!(r.verdict, Verdict::SafeCold);
        assert_eq!(exit_code(r.verdict), 0);
    }

    #[test]
    fn resolver_attempts_makes_it_hot() {
        let mut perf = safe_off_perf();
        perf.resolver_attempts = 3;
        let route = safe_off_route();
        let r = evaluate(&RawInputs {
            perf: Some(&perf),
            route: Some(&route),
            telemetry_shm_present: true,
        });
        assert_eq!(r.verdict, Verdict::Hot);
        assert_eq!(exit_code(r.verdict), 2);
        assert!(r.reasons.iter().any(|s| s.contains("resolver_attempts=3")));
        assert!(!r.resolver_off);
    }

    #[test]
    fn pattern_scan_makes_it_hot() {
        let mut perf = safe_off_perf();
        perf.pattern_scan_count = 1;
        let route = safe_off_route();
        let r = evaluate(&RawInputs {
            perf: Some(&perf),
            route: Some(&route),
            telemetry_shm_present: true,
        });
        assert_eq!(exit_code(r.verdict), 2);
        assert!(r.reasons.iter().any(|s| s.contains("pattern_scan_count=1")));
    }

    #[test]
    fn worker_walk_makes_it_hot() {
        let mut perf = safe_off_perf();
        perf.worker_walk_count = 7;
        let route = safe_off_route();
        let r = evaluate(&RawInputs {
            perf: Some(&perf),
            route: Some(&route),
            telemetry_shm_present: true,
        });
        assert_eq!(exit_code(r.verdict), 2);
        assert!(!r.worker_asleep);
        assert!(r.reasons.iter().any(|s| s.contains("worker_walk_count=7")));
    }

    #[test]
    fn frame_stutter_makes_it_hot() {
        let mut perf = safe_off_perf();
        perf.buckets[0].over_1000us = 4;
        perf.buckets[0].max_us = 2500;
        let route = safe_off_route();
        let r = evaluate(&RawInputs {
            perf: Some(&perf),
            route: Some(&route),
            telemetry_shm_present: true,
        });
        assert_eq!(exit_code(r.verdict), 2);
        assert_eq!(r.frame_cb_over_1000us, 4);
        assert!(r.reasons.iter().any(|s| s.contains("over_1000us=4")));
    }

    #[test]
    fn no_shm_is_unavailable_exit_one() {
        let r = evaluate(&RawInputs {
            perf: None,
            route: None,
            telemetry_shm_present: false,
        });
        assert_eq!(r.verdict, Verdict::Unavailable);
        assert_eq!(exit_code(r.verdict), 1);
        assert!(!r.dll_active);
    }

    #[test]
    fn input_enabled_reports_not_disabled() {
        let mut perf = safe_off_perf();
        perf.input_enabled = 1;
        let route = safe_off_route();
        let r = evaluate(&RawInputs {
            perf: Some(&perf),
            route: Some(&route),
            telemetry_shm_present: true,
        });
        assert!(!r.input_disabled);
        assert!(r.input_enabled);
        // input on its own is not a hot/unsafe condition for the verdict.
        assert_eq!(r.verdict, Verdict::SafeCold);
    }

    #[test]
    fn human_format_renders_all_lines() {
        let perf = safe_off_perf();
        let route = safe_off_route();
        let r = evaluate(&RawInputs {
            perf: Some(&perf),
            route: Some(&route),
            telemetry_shm_present: true,
        });
        let out = format_human(&r);
        for key in [
            "DLL aktiv:",
            "Diag-Level:",
            "Resolver:",
            "Worker:",
            "Pattern-Scans:",
            "Input:",
            "Frame-Callback:",
            "SICHER-KALT",
        ] {
            assert!(out.contains(key), "missing line: {key}\n{out}");
        }
    }

    #[test]
    fn json_format_is_valid_and_has_verdict() {
        let perf = safe_off_perf();
        let route = safe_off_route();
        let r = evaluate(&RawInputs {
            perf: Some(&perf),
            route: Some(&route),
            telemetry_shm_present: true,
        });
        let json = format_json(&r);
        let parsed: serde_json::Value = serde_json::from_str(&json).expect("valid json");
        assert_eq!(parsed["verdict"], "safe_cold");
        assert_eq!(parsed["resolver_off"], true);
        assert_eq!(parsed["diag_level"], "normal_default_off");
    }
}
