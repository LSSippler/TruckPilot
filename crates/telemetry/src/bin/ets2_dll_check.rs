//! Check ETS2 telemetry DLL installation and game.log load hints.
//!
//! ```text
//! cargo run -p truckpilot-telemetry --bin ets2-dll-check
//! ```

use std::path::{Path, PathBuf};

const DLL_NAME: &str = "truckpilot_telemetry.dll";

pub fn default_plugins_dir() -> Option<PathBuf> {
    if let Ok(dir) = std::env::var("ETS2_PLUGINS_DIR") {
        return Some(PathBuf::from(dir));
    }
    for base in steam_common_roots() {
        let plugins = base.join("bin").join("win_x64").join("plugins");
        if plugins.is_dir() {
            return Some(plugins);
        }
    }
    None
}

fn steam_common_roots() -> Vec<PathBuf> {
    let mut roots = Vec::new();
    if let Ok(p) = std::env::var("ProgramFiles(x86)") {
        roots.push(
            PathBuf::from(p)
                .join("Steam")
                .join("steamapps")
                .join("common")
                .join("Euro Truck Simulator 2"),
        );
    }
    if let Ok(p) = std::env::var("ProgramFiles") {
        roots.push(
            PathBuf::from(p)
                .join("Steam")
                .join("steamapps")
                .join("common")
                .join("Euro Truck Simulator 2"),
        );
    }
    roots
}

pub fn default_game_log_path() -> Option<PathBuf> {
    if let Ok(profile) = std::env::var("USERPROFILE") {
        let p = PathBuf::from(profile)
            .join("Documents")
            .join("Euro Truck Simulator 2")
            .join("game.log.txt");
        if p.is_file() {
            return Some(p);
        }
    }
    None
}

pub fn format_file_age(path: &Path) -> String {
    match path.metadata().and_then(|m| m.modified()) {
        Ok(t) => match t.elapsed() {
            Ok(d) if d.as_secs() < 120 => format!("{:.0}s ago (fresh)", d.as_secs()),
            Ok(d) if d.as_secs() < 86400 => format!("{:.0}m ago", d.as_secs() / 60),
            Ok(d) => format!("{:.1}d ago", d.as_secs_f64() / 86400.0),
            Err(_) => "unknown age".into(),
        },
        Err(_) => "unknown age".into(),
    }
}

pub fn scan_game_log_for_dll(path: &Path) -> Vec<String> {
    let Ok(text) = std::fs::read_to_string(path) else {
        return Vec::new();
    };
    text.lines()
        .filter(|line| {
            let l = line.to_ascii_lowercase();
            l.contains("truckpilot") || l.contains("truckpilot_telemetry")
        })
        .map(str::to_string)
        .collect()
}

pub fn sidecar_init_status(tail: &str) -> Option<&'static str> {
    if tail.contains("scs_telemetry_init returning success")
        || tail.contains("scs_input_init returning success")
    {
        Some("init reported success")
    } else if tail.contains("PANIC in") {
        Some("PANIC during init — check full log")
    } else if tail.contains("returning failure") {
        Some("init reported failure — check full log")
    } else {
        None
    }
}

pub fn sidecar_shutdown_seen(tail: &str) -> bool {
    tail.contains("scs_telemetry_shutdown called") || tail.contains("scs_input_shutdown called")
}

pub fn dependency_hint() -> &'static str {
    "Expected native deps: KERNEL32.dll, USER32.dll, VCRUNTIME140.dll (MSVC runtime). \
     No ONNX/DirectML — telemetry-dll has zero Cargo dependencies."
}

pub fn dll_probe_message(plugins_dir: &Path, dll_path: &Path) -> String {
    if !dll_path.is_file() {
        return format!(
            "MISSING: {DLL_NAME} not found in {}\n\
             Deploy with: cargo xtask copy-ets2-dll --release \"{}\"",
            plugins_dir.display(),
            plugins_dir.display()
        );
    }
    let size = dll_path.metadata().map(|m| m.len()).unwrap_or(0);
    let age = format_file_age(dll_path);
    format!(
        "OK: {} found\n  path: {}\n  size: {size} bytes\n  modified: {age}",
        DLL_NAME,
        dll_path.display()
    )
}

fn main() {
    println!("ets2-dll-check — TruckPilot telemetry DLL installation probe");
    println!();

    let plugins_dir = match default_plugins_dir() {
        Some(d) => d,
        None => {
            eprintln!("Could not locate ETS2 plugins directory.");
            eprintln!("Set ETS2_PLUGINS_DIR to <ETS2>/bin/win_x64/plugins/");
            std::process::exit(1);
        }
    };

    println!("ETS2 plugins dir: {}", plugins_dir.display());
    let dll_path = plugins_dir.join(DLL_NAME);
    println!("{}", dll_probe_message(&plugins_dir, &dll_path));
    println!("{}", dependency_hint());

    let sidecar_log = plugins_dir.join("truckpilot_telemetry.log");
    if sidecar_log.is_file() {
        println!(
            "Sidecar log: {} ({})",
            sidecar_log.display(),
            format_file_age(&sidecar_log)
        );
        if let Ok(tail) = std::fs::read_to_string(&sidecar_log) {
            if let Some(status) = sidecar_init_status(&tail) {
                println!("  Sidecar status: {status}");
            }
            if sidecar_shutdown_seen(&tail) {
                println!("  Sidecar: shutdown was called (DLL unloaded cleanly or ETS2 exited plugin)");
            }
            for line in tail.lines().rev().take(8).collect::<Vec<_>>().into_iter().rev() {
                println!("  {line}");
            }
        }
    } else {
        println!("Sidecar log: not found ({})", sidecar_log.display());
        println!("  (created after ETS2 loads the DLL)");
    }

    if let Some(game_log) = default_game_log_path() {
        println!();
        println!("game.log: {}", game_log.display());
        let hits = scan_game_log_for_dll(&game_log);
        if hits.is_empty() {
            println!("  No TruckPilot lines found — DLL may not be loaded.");
        } else {
            println!("  TruckPilot-related lines (last matches):");
            for line in hits.iter().rev().take(8).collect::<Vec<_>>().into_iter().rev() {
                println!("    {line}");
            }
        }
    } else {
        println!();
        println!("game.log: not found under Documents/Euro Truck Simulator 2/");
    }

    println!();
    println!("Next: start ETS2, then run:");
    println!("  cargo run -p truckpilot-telemetry --bin route-shm-dump -- --once");
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[test]
    fn dll_probe_missing_reports_deploy_hint() {
        let dir = std::env::temp_dir().join(format!("ets2-dll-check-{}", std::process::id()));
        let _ = std::fs::create_dir_all(&dir);
        let msg = dll_probe_message(&dir, &dir.join(DLL_NAME));
        assert!(msg.contains("MISSING"));
        assert!(msg.contains("copy-ets2-dll"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn sidecar_status_detects_success_and_shutdown() {
        let ok = "[1] scs_telemetry_init returning success\n";
        assert_eq!(sidecar_init_status(ok), Some("init reported success"));
        let down = "[2] scs_telemetry_shutdown called\n";
        assert!(sidecar_shutdown_seen(down));
    }

    #[test]
    fn dependency_hint_mentions_msvc() {
        assert!(dependency_hint().contains("VCRUNTIME140"));
    }

    #[test]
    fn game_log_scan_finds_truckpilot_lines() {
        let dir = std::env::temp_dir().join(format!("ets2-log-scan-{}", std::process::id()));
        let _ = std::fs::create_dir_all(&dir);
        let log = dir.join("game.log.txt");
        {
            let mut f = std::fs::File::create(&log).unwrap();
            writeln!(f, "Loading plugin: truckpilot_telemetry.dll").unwrap();
            writeln!(f, "[TruckPilot] scs_telemetry_init done").unwrap();
        }
        let hits = scan_game_log_for_dll(&log);
        assert_eq!(hits.len(), 2);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
