//! Offline analyzer for saved `truckpilot_telemetry.log` sidecar files.

use std::env;
use std::fs;
use std::path::PathBuf;
use std::process;

use truckpilot_telemetry::route_log_analyze::{analyze_log_text, render_report};

fn main() {
    let path = parse_args();
    let text = fs::read_to_string(&path).unwrap_or_else(|e| {
        eprintln!("route-log-analyze: read {}: {e}", path.display());
        process::exit(1);
    });
    let report = analyze_log_text(&text);
    print!("{}", render_report(&report));
}

fn parse_args() -> PathBuf {
    let mut args = env::args().skip(1);
    let Some(path) = args.next() else {
        eprintln!("usage: route-log-analyze <path-to-truckpilot_telemetry.log>");
        process::exit(2);
    };
    PathBuf::from(path)
}

#[cfg(test)]
mod tests {
    use super::parse_args;

    #[test]
    fn usage_requires_path() {
        // compile-time smoke only — CLI tested via lib tests
        let _ = parse_args;
    }
}
