//! Offline analyzer for local `eurotrucks2.exe` — PE sections, AOB patterns, strings, heuristic xrefs.
//!
//! File I/O only. No process handles, no live memory, no DLL changes.

use std::env;
use std::path::PathBuf;
use std::process;

use truckpilot_telemetry::ets2_bin_analyze::{
    analyze_file, diff_snapshots, render_diff, render_focus_only, render_snapshot, AnalyzeLimits,
};

fn main() {
    let args = parse_args();
    let limits = AnalyzeLimits::default();

    if let (Some(old), Some(new)) = (&args.old, &args.new) {
        let old_a = analyze_file(old, &limits).unwrap_or_else(|e| die(&e));
        let new_a = analyze_file(new, &limits).unwrap_or_else(|e| die(&e));
        let diff = diff_snapshots(
            &old_a.snapshot.as_diff_snapshot(),
            &new_a.snapshot.as_diff_snapshot(),
        );
        print!("{}", render_diff(&diff));
        return;
    }

    let exe = args
        .exe
        .or(args.new)
        .unwrap_or_else(|| die("usage: ets2-bin-analyze --exe PATH | --old OLD --new NEW"));

    let analysis = analyze_file(&exe, &limits).unwrap_or_else(|e| die(&e));
    if args.focus_gps {
        print!("{}", render_focus_only(&analysis.snapshot, &analysis.pe));
    } else {
        print!("{}", render_snapshot(&analysis.snapshot, &analysis.pe));
    }
}

fn die(msg: &str) -> ! {
    eprintln!("ets2-bin-analyze: {msg}");
    eprintln!("usage: ets2-bin-analyze --exe path\\to\\eurotrucks2.exe [--focus gps]");
    eprintln!("       ets2-bin-analyze --old path\\to\\1.59\\eurotrucks2.exe --new path\\to\\1.60\\eurotrucks2.exe");
    process::exit(2);
}

struct CliArgs {
    exe: Option<PathBuf>,
    old: Option<PathBuf>,
    new: Option<PathBuf>,
    focus_gps: bool,
}

fn parse_args() -> CliArgs {
    let mut exe = None;
    let mut old = None;
    let mut new = None;
    let mut focus_gps = false;
    let mut it = env::args().skip(1);
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--exe" => exe = Some(PathBuf::from(it.next().unwrap_or_else(|| die("missing --exe path")))),
            "--old" => old = Some(PathBuf::from(it.next().unwrap_or_else(|| die("missing --old path")))),
            "--new" => new = Some(PathBuf::from(it.next().unwrap_or_else(|| die("missing --new path")))),
            "--focus" => {
                let mode = it.next().unwrap_or_else(|| die("missing --focus value (gps)"));
                if mode != "gps" {
                    die("--focus only supports 'gps'");
                }
                focus_gps = true;
            }
            "-h" | "--help" => die("see usage above"),
            other => die(&format!("unknown argument: {other}")),
        }
    }
    CliArgs {
        exe,
        old,
        new,
        focus_gps,
    }
}

#[cfg(test)]
mod tests {
    use super::parse_args;

    #[test]
    fn cli_module_compiles() {
        let _ = parse_args;
    }
}
