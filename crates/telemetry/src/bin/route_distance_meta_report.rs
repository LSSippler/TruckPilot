//! Aggregate route distance verification reports across multiple CSV recordings (Phase 5l).
//!
//! ```text
//! cargo run -p truckpilot-telemetry --bin route-distance-meta-report -- logs/*.csv
//! cargo run -p truckpilot-telemetry --bin route-distance-meta-report -- --json meta_report.json --min-runs 3 logs/*.csv
//! ```

use std::fs;
use std::time::{SystemTime, UNIX_EPOCH};

use truckpilot_telemetry::route_distance_log::{
    build_distance_meta_report, collect_meta_report_input_paths, format_distance_meta_report_text,
};

fn arg_value(flag: &str) -> Option<String> {
    let args: Vec<String> = std::env::args().collect();
    args.windows(2)
        .find(|w| w[0] == flag)
        .map(|w| w[1].clone())
}

fn parse_usize_arg(flag: &str, default: usize) -> usize {
    arg_value(flag)
        .and_then(|s| s.parse().ok())
        .unwrap_or(default)
        .max(1)
}

fn wall_time_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let min_runs = parse_usize_arg("--min-runs", 3);
    let json_path = arg_value("--json");

    let paths = match collect_meta_report_input_paths(&args) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("ERROR: {e}");
            eprintln!("Usage: route-distance-meta-report [--json out.json] [--min-runs N] <file.csv> ...");
            std::process::exit(1);
        }
    };

    let meta = match build_distance_meta_report(&paths, min_runs, wall_time_ms()) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("ERROR: {e}");
            std::process::exit(1);
        }
    };

    print!("{}", format_distance_meta_report_text(&meta));

    if let Some(json_path) = json_path {
        let json = serde_json::to_string_pretty(&meta).expect("serialize meta report");
        fs::write(&json_path, json).unwrap_or_else(|e| {
            eprintln!("ERROR: write {json_path}: {e}");
            std::process::exit(1);
        });
        eprintln!("JSON meta-report → {json_path}");
    }

    if meta.verdict.as_str() == "suspicious" {
        std::process::exit(2);
    }
}
