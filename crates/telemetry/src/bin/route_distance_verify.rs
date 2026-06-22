//! Promotion verification gate for ETS2 route distance @+0x14 (Phase 5m).
//!
//! Stricter than `route-distance-meta-report`. Does not modify code or flags.
//!
//! ```text
//! cargo run -p truckpilot-telemetry --bin route-distance-verify -- logs/*.csv
//! cargo run -p truckpilot-telemetry --bin route-distance-verify -- \
//!   --min-runs 5 \
//!   --require-scenarios autobahn,stadt,auffahrt,depot \
//!   --json verification.json \
//!   --markdown verification.md \
//!   logs/*.csv
//! ```

use std::fs;
use std::time::{SystemTime, UNIX_EPOCH};

use truckpilot_telemetry::route_distance_log::{
    build_distance_verification_report, collect_tool_input_paths,
    format_distance_verification_markdown, format_distance_verification_text,
    parse_required_scenarios, RouteDistancePromotionVerdict, RouteDistanceVerificationConfig,
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
    let min_runs = parse_usize_arg("--min-runs", 5);
    let json_path = arg_value("--json");
    let markdown_path = arg_value("--markdown");
    let required_scenarios = arg_value("--require-scenarios")
        .map(|s| parse_required_scenarios(&s))
        .unwrap_or_default();

    let config = RouteDistanceVerificationConfig {
        min_runs,
        required_scenarios,
        ..Default::default()
    };

    let paths = match collect_tool_input_paths(&args) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("ERROR: {e}");
            eprintln!(
                "Usage: route-distance-verify [--json out.json] [--markdown out.md] [--min-runs N] [--require-scenarios a,b,c] <file.csv> ..."
            );
            std::process::exit(1);
        }
    };

    let report = match build_distance_verification_report(&paths, &config, wall_time_ms()) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("ERROR: {e}");
            std::process::exit(1);
        }
    };

    print!("{}", format_distance_verification_text(&report));

    if let Some(json_path) = json_path {
        let json = serde_json::to_string_pretty(&report).expect("serialize verification report");
        fs::write(&json_path, json).unwrap_or_else(|e| {
            eprintln!("ERROR: write {json_path}: {e}");
            std::process::exit(1);
        });
        eprintln!("JSON verification → {json_path}");
    }

    if let Some(md_path) = markdown_path {
        let md = format_distance_verification_markdown(&report);
        fs::write(&md_path, md).unwrap_or_else(|e| {
            eprintln!("ERROR: write {md_path}: {e}");
            std::process::exit(1);
        });
        eprintln!("Markdown verification → {md_path}");
    }

    match report.verdict {
        RouteDistancePromotionVerdict::Passed => std::process::exit(0),
        RouteDistancePromotionVerdict::Failed | RouteDistancePromotionVerdict::Inconclusive => {
            std::process::exit(2)
        }
    }
}
