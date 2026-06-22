//! Record ETS2 route distance diagnostics from RouteBlackboard SHM (Phase 5k).
//!
//! ```text
//! cargo run -p truckpilot-telemetry --bin route-distance-recorder -- --out logs/route_distance.csv --hz 2 --duration-sec 300
//! ```

use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::Path;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use truckpilot_telemetry::nav_route::RouteBlackboardReader;
use truckpilot_telemetry::route_distance_log::{
    snapshot_to_sample_row, snapshot_to_waypoint_records, write_sample_csv_header,
    write_sample_csv_row,
};

fn has_flag(flag: &str) -> bool {
    std::env::args().any(|a| a == flag)
}

fn arg_value(flag: &str) -> Option<String> {
    let args: Vec<String> = std::env::args().collect();
    args.windows(2)
        .find(|w| w[0] == flag)
        .map(|w| w[1].clone())
}

fn parse_f64_arg(flag: &str, default: f64) -> f64 {
    arg_value(flag)
        .and_then(|s| s.parse().ok())
        .unwrap_or(default)
        .max(0.1)
}

fn parse_u64_arg(flag: &str) -> Option<u64> {
    arg_value(flag).and_then(|s| s.parse().ok())
}

fn wall_time_ms() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

fn open_csv_writer(path: &Path, append: bool) -> Result<BufWriter<File>, String> {
    let file_exists = path.exists();
    let file = OpenOptions::new()
        .create(true)
        .append(append)
        .write(true)
        .open(path)
        .map_err(|e| format!("open {}: {e}", path.display()))?;
    let mut w = BufWriter::new(file);
    write_sample_csv_header(&mut w, append, file_exists)
        .map_err(|e| format!("write header: {e}"))?;
    Ok(w)
}

fn wait_for_reader(wait: bool) -> Result<RouteBlackboardReader, String> {
    if !wait {
        return RouteBlackboardReader::open();
    }
    eprintln!("Waiting for RouteBlackboard SHM (Ctrl+C to abort)...");
    loop {
        if let Ok(reader) = RouteBlackboardReader::open() {
            eprintln!("RouteBlackboard SHM connected.");
            return Ok(reader);
        }
        thread::sleep(Duration::from_millis(500));
    }
}

fn main() {
    let out_path = match arg_value("--out") {
        Some(p) => p,
        None => {
            eprintln!("ERROR: --out <path> is required");
            eprintln!("Example: route-distance-recorder --out logs/route_distance.csv --hz 2");
            std::process::exit(1);
        }
    };

    let hz = parse_f64_arg("--hz", 2.0);
    let duration_sec = parse_u64_arg("--duration-sec");
    let append = has_flag("--append");
    let wait = has_flag("--wait") || !has_flag("--no-wait");
    let include_waypoints = has_flag("--include-waypoints");
    let jsonl_path = arg_value("--jsonl");

    let interval = Duration::from_secs_f64(1.0 / hz);
    let out = Path::new(&out_path);

    if let Some(parent) = out.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent).unwrap_or_else(|e| {
                eprintln!("ERROR: create dir {}: {e}", parent.display());
                std::process::exit(1);
            });
        }
    }

    let mut csv_writer = match open_csv_writer(out, append) {
        Ok(w) => w,
        Err(e) => {
            eprintln!("ERROR: {e}");
            std::process::exit(1);
        }
    };

    let mut jsonl_writer: Option<BufWriter<File>> = if include_waypoints || jsonl_path.is_some() {
        let jp = jsonl_path.unwrap_or_else(|| {
            format!(
                "{}.waypoints.jsonl",
                out_path.trim_end_matches(".csv")
            )
        });
        let file = OpenOptions::new()
            .create(true)
            .append(append)
            .write(true)
            .open(&jp)
            .unwrap_or_else(|e| {
                eprintln!("ERROR: open jsonl {jp}: {e}");
                std::process::exit(1);
            });
        eprintln!("Waypoint JSONL → {jp}");
        Some(BufWriter::new(file))
    } else {
        None
    };

    let reader = match wait_for_reader(wait) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("ERROR: {e}");
            eprintln!("Hint: start ETS2 with telemetry DLL or route-shm-sim; use --wait (default).");
            std::process::exit(1);
        }
    };

    eprintln!(
        "route-distance-recorder → {} (hz={hz:.2}, append={append}, duration={duration_sec:?})",
        out.display()
    );

    let start = Instant::now();
    let mut samples = 0u64;
    let mut last_sequence: Option<u32> = None;

    loop {
        if let Some(dur) = duration_sec {
            if start.elapsed().as_secs() >= dur {
                break;
            }
        }

        if let Some(snap) = reader.read() {
            let now_ms = wall_time_ms();
            let row = snapshot_to_sample_row(now_ms, &snap);
            if write_sample_csv_row(&mut csv_writer, &row).is_err() {
                eprintln!("ERROR: CSV write failed");
                break;
            }
            csv_writer.flush().ok();

            if let Some(jw) = jsonl_writer.as_mut() {
                let records = snapshot_to_waypoint_records(now_ms, &snap);
                for rec in records {
                    if let Ok(line) = serde_json::to_string(&rec) {
                        let _ = writeln!(jw, "{line}");
                    }
                }
                jw.flush().ok();
            }

            if last_sequence != Some(snap.sequence) {
                eprintln!(
                    "sample #{samples} seq={} hash={:#x} dist_count={} mono={} valid={}",
                    snap.sequence,
                    snap.route_hash,
                    row.distance_count,
                    row.distance_monotonic_status,
                    snap.valid
                );
                last_sequence = Some(snap.sequence);
            }
            samples += 1;
        }

        thread::sleep(interval);
    }

    eprintln!("Done. {samples} samples written to {}", out.display());
}
