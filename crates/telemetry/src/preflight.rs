//! Read-only preflight / drive-readiness **display** (not an engage gate).
//!
//! Combines SHM-derived DLL safety fields with optional core readiness.
//! Missing daemon blackboard data stays `null` / unknown — never affects CLI exit codes.

use crate::core_readiness::CoreReadiness;
use crate::status_report::StatusReport;

/// Tri-state bool for JSON (`true` / `false` / `null` = unknown).
pub type TriState = Option<bool>;

/// Display-only preflight snapshot for `truckpilot-status --json`.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
pub struct PreflightDisplay {
    /// Aggregate subsystem startup (`truckpilot_system_ready`); unknown without daemon BB.
    pub system_ready: TriState,
    /// Recent telemetry arriving; SHM proxy when daemon keys unavailable.
    pub telemetry_fresh: TriState,
    /// Valid routed path (route blackboard `valid` bit when SHM mapped).
    pub route_valid: TriState,
    /// Live lane model quality; unknown in SHM-only CLI (daemon blackboard only).
    pub lane_model_valid: TriState,
    /// Resolver parked / safe-off (`resolver_off` && no attempts).
    pub resolver_safe: TriState,
    /// Output path permitted to act; SHM uses DLL `input_enabled` as proxy.
    pub input_allowed: TriState,
    /// Autopilot state machine label when known (overlay/daemon); absent in SHM-only CLI.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub autopilot_state: Option<String>,
    /// **Display only** — not an engage authorization.
    pub drive_allowed_display: bool,
    /// Human-readable blockers (includes `unknown` qualifiers).
    pub reasons: Vec<String>,
}

impl PreflightDisplay {
    /// All-unknown preflight (no SHM, no daemon).
    pub fn unavailable() -> Self {
        Self {
            system_ready: None,
            telemetry_fresh: None,
            route_valid: None,
            lane_model_valid: None,
            resolver_safe: None,
            input_allowed: None,
            autopilot_state: None,
            drive_allowed_display: false,
            reasons: vec!["preflight data unavailable".into()],
        }
    }
}

fn push_tri_reason(reasons: &mut Vec<String>, label: &str, v: TriState, false_msg: &str) {
    match v {
        Some(false) => reasons.push(false_msg.to_string()),
        None => reasons.push(format!("{label} unknown")),
        Some(true) => {}
    }
}

fn compute_drive_allowed(
    system_ready: TriState,
    telemetry_fresh: TriState,
    route_valid: TriState,
    lane_model_valid: TriState,
    resolver_safe: TriState,
    input_allowed: TriState,
    autopilot_state: Option<&str>,
) -> (bool, Vec<String>) {
    let mut reasons = Vec::new();
    push_tri_reason(&mut reasons, "system ready", system_ready, "system not ready");
    push_tri_reason(
        &mut reasons,
        "telemetry fresh",
        telemetry_fresh,
        "telemetry not fresh",
    );
    push_tri_reason(&mut reasons, "route valid", route_valid, "route invalid");
    push_tri_reason(
        &mut reasons,
        "lane model valid",
        lane_model_valid,
        "lane model invalid",
    );
    push_tri_reason(
        &mut reasons,
        "resolver safe",
        resolver_safe,
        "resolver not safe",
    );
    push_tri_reason(
        &mut reasons,
        "input allowed",
        input_allowed,
        "input disabled",
    );
    if matches!(autopilot_state, Some("Fault")) {
        reasons.push("autopilot fault".into());
    }
    let drive_allowed_display = reasons.is_empty();
    (drive_allowed_display, reasons)
}

/// Build display-only preflight from a [`StatusReport`] (SHM + embedded core readiness).
pub fn evaluate_preflight(status: &StatusReport) -> PreflightDisplay {
    let core: &CoreReadiness = &status.core_readiness;

    let system_ready = if core.available {
        core.truckpilot_system_ready
    } else {
        None
    };

    let telemetry_fresh = if status.telemetry_shm_present {
        Some(status.frame_cb_count > 0)
    } else {
        None
    };

    let route_valid = if status.route_bb_available {
        Some(status.route_valid)
    } else {
        None
    };

    // Live lane validity requires daemon blackboard — not available in SHM-only CLI.
    let lane_model_valid = None;

    let resolver_safe = if status.route_bb_available || status.perf_shm_available {
        Some(status.resolver_off && status.resolver_attempts == 0)
    } else {
        None
    };

    let input_allowed = if status.perf_shm_available {
        Some(status.input_enabled)
    } else {
        None
    };

    let (drive_allowed_display, reasons) = compute_drive_allowed(
        system_ready,
        telemetry_fresh,
        route_valid,
        lane_model_valid,
        resolver_safe,
        input_allowed,
        None,
    );

    PreflightDisplay {
        system_ready,
        telemetry_fresh,
        route_valid,
        lane_model_valid,
        resolver_safe,
        input_allowed,
        autopilot_state: None,
        drive_allowed_display,
        reasons,
    }
}

/// Optional one-line human suffix (full block lives in overlay).
pub fn format_preflight_human(p: &PreflightDisplay) -> String {
    let drive = if p.drive_allowed_display {
        "yes (display only)"
    } else {
        "no"
    };
    format!(
        "Preflight drive allowed (display): {drive}\n  Reason: {}",
        if p.reasons.is_empty() {
            "none".to_string()
        } else {
            p.reasons.join(", ")
        }
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::status_report::{evaluate_status, RawStatusInputs, StatusVerdict};

    #[test]
    fn unavailable_preflight_when_no_shm() {
        let status = evaluate_status(&RawStatusInputs::default());
        assert_eq!(status.verdict, StatusVerdict::Unavailable);
        let p = evaluate_preflight(&status);
        assert!(!p.drive_allowed_display);
        assert!(p.system_ready.is_none());
        assert!(p.lane_model_valid.is_none());
    }

    #[cfg(windows)]
    #[test]
    fn route_invalid_lists_reason_without_making_status_hot() {
        use crate::dll_perf::{DllPerfSnapshot, DLL_PERF_MAGIC, DLL_PERF_VERSION};
        use crate::nav_route::{
            RouteSnapshot, RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE, ROUTE_BB_STATUS_DLL_ACTIVE,
        };

        let status = evaluate_status(&RawStatusInputs {
            perf: Some(DllPerfSnapshot {
                magic: DLL_PERF_MAGIC,
                version: DLL_PERF_VERSION,
                diag_level_code: 9,
                frame_cb_count: 100,
                ..Default::default()
            }),
            route: Some(RouteSnapshot {
                bb_status: ROUTE_BB_STATUS_DLL_ACTIVE,
                resolve_status: RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE,
                valid: false,
                ..Default::default()
            }),
            telemetry_shm_present: true,
        });
        assert_eq!(status.verdict, StatusVerdict::SafeCold);
        let p = evaluate_preflight(&status);
        assert_eq!(p.route_valid, Some(false));
        assert!(!p.drive_allowed_display);
        assert!(p.reasons.iter().any(|r| r.contains("route invalid")));
    }

    #[test]
    fn compute_drive_allowed_needs_all_true() {
        let (ok, reasons) = compute_drive_allowed(
            Some(true),
            Some(true),
            Some(true),
            Some(true),
            Some(true),
            Some(true),
            Some("Off"),
        );
        assert!(ok);
        assert!(reasons.is_empty());

        let (ok, reasons) = compute_drive_allowed(
            Some(true),
            Some(true),
            Some(false),
            Some(false),
            Some(true),
            Some(false),
            Some("Fault"),
        );
        assert!(!ok);
        assert!(reasons.iter().any(|r| r.contains("route invalid")));
        assert!(reasons.iter().any(|r| r.contains("autopilot fault")));
    }
}
