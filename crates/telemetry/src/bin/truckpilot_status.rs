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
//! cargo run -p truckpilot-telemetry --bin truckpilot-status -- --overlay-loop
//! cargo run -p truckpilot-telemetry --bin truckpilot-status -- --overlay-loop --overlay-interval-ms 2000 --overlay-out overlay_snapshot.json
//! ```
//!
//! `--overlay-loop` continuously writes read-only overlay JSON (including
//! `planned_path` from route blackboard when safe, else offline-graph fixture)
//! to `--overlay-out` or the default `%LOCALAPPDATA%/TruckPilot/overlay_snapshot.json`
//! on Windows.
//!
//! Exit codes (DLL/SHM safety only — core readiness does not affect exit code):
//! - `0` — DLL present and SAFE-COLD.
//! - `1` — no SHM found.
//! - `2` — DLL present but HOT/unsafe.

use std::path::PathBuf;
use std::thread;
use std::time::Duration;

use truckpilot_telemetry::overlay_snapshot::{
    build_overlay_snapshot, default_overlay_loop_path, format_overlay_json,
    read_overlay_snapshot, write_overlay_snapshot_atomic,
};
use truckpilot_telemetry::status_report::{
    format_status_human, format_status_json, read_live_status, status_exit_code,
};

fn has_flag(flag: &str) -> bool {
    std::env::args().any(|a| a == flag)
}

fn arg_value(flag: &str) -> Option<String> {
    let mut args = std::env::args();
    while let Some(a) = args.next() {
        if a == flag {
            return args.next();
        }
        if let Some(rest) = a.strip_prefix(&format!("{flag}=")) {
            return Some(rest.to_string());
        }
    }
    None
}

fn overlay_interval_ms() -> u64 {
    arg_value("--overlay-interval-ms")
        .and_then(|v| v.parse().ok())
        .filter(|&ms| ms >= 250)
        .unwrap_or(2000)
}

fn overlay_out_path() -> PathBuf {
    arg_value("--overlay-out")
        .map(PathBuf::from)
        .unwrap_or_else(default_overlay_loop_path)
}

fn run_overlay_once() {
    let snap = read_overlay_snapshot();
    println!("{}", format_overlay_json(&snap));
    std::process::exit(status_exit_code(snap.verdict));
}

fn run_overlay_loop() {
    let path = overlay_out_path();
    let interval = Duration::from_millis(overlay_interval_ms());
    eprintln!(
        "truckpilot-status: overlay loop → {} every {} ms (Ctrl+C to stop)",
        path.display(),
        interval.as_millis()
    );
    loop {
        let snap = build_overlay_snapshot(&truckpilot_telemetry::status_report::read_raw_inputs());
        if let Err(e) = write_overlay_snapshot_atomic(&path, &snap) {
            eprintln!("truckpilot-status: overlay write failed: {e}");
        }
        thread::sleep(interval);
    }
}

fn main() {
    if has_flag("--overlay-loop") {
        run_overlay_loop();
    }
    if has_flag("--overlay") {
        run_overlay_once();
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
    use truckpilot_telemetry::overlay_snapshot::build_overlay_snapshot;
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

    #[test]
    fn overlay_snapshot_includes_read_only_planned_path() {
        let snap = build_overlay_snapshot(&RawStatusInputs::default());
        let json = truckpilot_telemetry::overlay_snapshot::format_overlay_json(&snap);
        let parsed: serde_json::Value = serde_json::from_str(&json).expect("valid json");
        assert_eq!(parsed["planned_path"]["source"], "offline_graph");
        assert!(parsed["planned_path"]["items"].as_array().unwrap().len() > 0);
        let point_count: usize = parsed["planned_path"]["items"]
            .as_array()
            .unwrap()
            .iter()
            .map(|item| item["points"].as_array().map(|a| a.len()).unwrap_or(0))
            .sum();
        assert!(point_count > 0);
        assert_eq!(
            parsed["planned_path"]["safety"]["drive_allowed_display_only"],
            false
        );
        assert_eq!(parsed["planned_path_producer"]["status"], "offline_fixture");
        assert_eq!(parsed["status"]["resolver_attempts"], 0);
    }
}
