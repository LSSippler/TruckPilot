//! Pure resolver walk decisions and off-mode guards (offline-testable).

use crate::resolver_sched::ResolverSchedule;
use crate::route_status::RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE;
use crate::safe_mem::{ResolverModeSelection, RouteResolverMode};

/// Outcome of the scheduling gate before any memory work.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WalkDecision {
    /// Default/off — park immediately, no resolver work.
    OffModePark,
    /// Retry/scan limit park.
    ScheduledParked,
    /// Diagnostic one-shot completed.
    DiagnosticParked,
    /// Exponential backoff — wait.
    BackoffWait,
    /// Proceed with resolver work for the active mode.
    Proceed,
}

/// Decide whether a worker walk may perform resolver/memory work.
pub fn decide_walk(mode: RouteResolverMode, sched: &ResolverSchedule, now_us: u64) -> WalkDecision {
    if mode.is_off() {
        return WalkDecision::OffModePark;
    }
    if sched.parked {
        return WalkDecision::ScheduledParked;
    }
    if sched.diagnostic_parked {
        return WalkDecision::DiagnosticParked;
    }
    if !sched.should_run_walk(now_us) {
        return WalkDecision::BackoffWait;
    }
    WalkDecision::Proceed
}

/// Block resolver/diagnostic entry points when mode is off.
pub fn block_if_resolver_off() -> Option<u32> {
    if crate::safe_mem::route_resolver_mode().is_off() {
        crate::resolver_metrics::note_off_mode_blocked_call();
        Some(RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE)
    } else {
        None
    }
}

/// Sidecar lines emitted once at init for a mode selection (offline-testable).
pub fn format_mode_init_log_lines(sel: &ResolverModeSelection) -> Vec<String> {
    let mut lines = vec![format!(
        "route resolver mode={}",
        sel.mode.sidecar_label()
    )];
    if sel.mode.is_off() {
        lines.push("route resolver disabled safe mode".into());
        lines.push("route resolver worker parked".into());
    } else {
        lines.push(format!(
            "route resolver mode selected={} source={}",
            sel.mode.sidecar_label(),
            sel.source_file
        ));
        if !sel.ignored_lower_priority.is_empty() {
            lines.push(format!(
                "ignored lower-priority resolver mode files: {}",
                sel.ignored_lower_priority.join(", ")
            ));
        }
    }
    lines
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::resolver_sched::ResolverSchedule;

    #[test]
    fn off_mode_decision_parks_without_proceed() {
        let sched = ResolverSchedule::new();
        assert_eq!(
            decide_walk(RouteResolverMode::SafeDefault, &sched, 1),
            WalkDecision::OffModePark
        );
    }

    #[test]
    fn diagnostic_parked_blocks_proceed() {
        let mut sched = ResolverSchedule::new();
        sched.park_diagnostic_done();
        assert_eq!(
            decide_walk(RouteResolverMode::RouteCandidateTableOnly, &sched, u64::MAX),
            WalkDecision::DiagnosticParked
        );
    }

    #[test]
    fn off_log_lines_match_live_baseline() {
        let sel = ResolverModeSelection {
            mode: RouteResolverMode::SafeDefault,
            source_file: "none",
            ignored_lower_priority: Vec::new(),
        };
        let lines = format_mode_init_log_lines(&sel);
        assert!(lines.iter().any(|l| l.contains("mode=off")));
        assert!(lines.iter().any(|l| l.contains("disabled safe mode")));
        assert!(lines.iter().any(|l| l.contains("worker parked")));
    }
}
