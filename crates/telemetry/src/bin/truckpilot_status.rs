//! `truckpilot-status` — read-only ETS2 DLL health/safety summary.
//!
//! ```text
//! cargo run -p truckpilot-telemetry --bin truckpilot-status
//! cargo run -p truckpilot-telemetry --bin truckpilot-status -- --json
//! cargo run -p truckpilot-telemetry --bin truckpilot-status -- --overlay
//! ```

use truckpilot_telemetry::overlay_snapshot::{format_overlay_json, read_overlay_snapshot};
use truckpilot_telemetry::status_report::{
    format_status_human, format_status_json, read_live_status, status_exit_code,
};

fn has_flag(flag: &str) -> bool {
    std::env::args().any(|a| a == flag)
}

fn main() {
    let json = has_flag("--json");
    let overlay = has_flag("--overlay");

    if overlay {
        let snap = read_overlay_snapshot();
        println!("{}", format_overlay_json(&snap));
        std::process::exit(status_exit_code(snap.verdict));
    }

    let report = read_live_status();
    if json {
        println!("{}", format_status_json(&report));
    } else {
        println!("{}", format_status_human(&report));
    }
    std::process::exit(status_exit_code(report.verdict));
}
