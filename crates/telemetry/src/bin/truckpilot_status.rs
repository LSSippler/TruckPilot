//! `truckpilot-status` — read-only ETS2 DLL health/safety summary.
//!
//! Reads shared memory (no daemon auto-start, no game-memory reads, no resolver,
//! no steering). Core readiness keys live on the daemon blackboard — without a
//! separate daemon connection they appear as `unknown` / `available: false`.
//!
//! ```text
//! cargo run -p truckpilot-telemetry --bin truckpilot-status
//! cargo run -p truckpilot-telemetry --bin truckpilot-status -- --json
//! cargo run -p truckpilot-telemetry --bin truckpilot-status -- --overlay
//! ```
//!
//! Exit codes (DLL/SHM safety only — core readiness does not affect exit code):
//! - `0` — DLL present and SAFE-COLD.
//! - `1` — no SHM found.
//! - `2` — DLL present but HOT/unsafe.

use truckpilot_telemetry::overlay_snapshot::{format_overlay_json, read_overlay_snapshot};
use truckpilot_telemetry::status_report::{
    format_status_human, format_status_json, read_live_status, status_exit_code,
};

fn has_flag(flag: &str) -> bool {
    std::env::args().any(|a| a == flag)
}

fn main() {
    if has_flag("--overlay") {
        let snap = read_overlay_snapshot();
        println!("{}", format_overlay_json(&snap));
        std::process::exit(status_exit_code(snap.verdict));
    }

    let json = has_flag("--json");
    let report = read_live_status();

    if json {
        println!("{}", format_status_json(&report));
    } else {
        println!("{}", format_status_human(&report));
    }

    std::process::exit(status_exit_code(report.verdict));
}

#[cfg(test)]
mod tests {
    use truckpilot_telemetry::status_report::{
        evaluate_status, format_status_human, format_status_json, RawStatusInputs, StatusVerdict,
    };

    #[test]
    fn json_includes_core_readiness_unavailable_without_daemon() {
        let r = evaluate_status(&RawStatusInputs::default());
        let json = format_status_json(&r);
        let parsed: serde_json::Value = serde_json::from_str(&json).expect("valid json");
        assert_eq!(parsed["core_readiness"]["available"], false);
        assert!(parsed["core_readiness"]["graph_ready"].is_null());
        assert_eq!(parsed["preflight"]["drive_allowed_display"], false);
        assert_eq!(r.verdict, StatusVerdict::Unavailable);
    }

    #[test]
    fn json_includes_preflight_object() {
        let r = evaluate_status(&RawStatusInputs::default());
        let parsed: serde_json::Value =
            serde_json::from_str(&format_status_json(&r)).expect("valid json");
        assert!(parsed["preflight"]["reasons"].is_array());
    }

    #[test]
    fn human_output_includes_core_readiness_block() {
        let r = evaluate_status(&RawStatusInputs::default());
        let out = format_status_human(&r);
        assert!(out.contains("Core readiness:"));
        assert!(out.contains("graph_ready              unknown"));
        assert!(out.contains("note: system_ready is not engage authorization"));
    }
}
