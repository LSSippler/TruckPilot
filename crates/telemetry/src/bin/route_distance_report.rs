//! Analyse route distance recorder CSV and print verification report (Phase 5k).
//!
//! ```text
//! cargo run -p truckpilot-telemetry --bin route-distance-report -- logs/route_distance.csv
//! cargo run -p truckpilot-telemetry --bin route-distance-report -- logs/route_distance.csv --json report.json
//! ```

use std::fs;
use std::path::PathBuf;

use truckpilot_telemetry::route_distance_log::{
    analyze_distance_samples, format_distance_report_text, load_sample_csv,
};

fn arg_value(flag: &str) -> Option<String> {
    let args: Vec<String> = std::env::args().collect();
    args.windows(2)
        .find(|w| w[0] == flag)
        .map(|w| w[1].clone())
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let csv_path = args
        .iter()
        .skip(1)
        .find(|a| !a.starts_with('-'))
        .cloned()
        .or_else(|| arg_value("--csv"))
        .unwrap_or_else(|| {
            eprintln!("Usage: route-distance-report [--json out.json] <recording.csv>");
            std::process::exit(1);
        });

    let rows = match load_sample_csv(PathBuf::from(&csv_path).as_path()) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("ERROR: {e}");
            std::process::exit(1);
        }
    };

    let report = analyze_distance_samples(&rows);
    print!("{}", format_distance_report_text(&report));

    if let Some(json_path) = arg_value("--json") {
        let json = serde_json::to_string_pretty(&report).expect("serialize report");
        fs::write(&json_path, json).unwrap_or_else(|e| {
            eprintln!("ERROR: write {json_path}: {e}");
            std::process::exit(1);
        });
        eprintln!("JSON report → {json_path}");
    }

    if report.verdict.as_str() == "suspicious" {
        std::process::exit(2);
    }
}
