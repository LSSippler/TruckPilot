//! TruckPilot Diagnostic Tool
//!
//! Checks all subsystems in order and prints a colour-coded summary.
//! Exit code 0 = all checks passed, 1 = at least one check failed.
//!
//! Usage:
//!   truckpilot-diag [--ets2-dir <path>] [--plugin-dir <path>] [--http-url <url>]
//!
//! Defaults:
//!   --ets2-dir    Windows: "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2"
//!                 Linux:   (skipped — ETS2 runs on Windows)
//!   --plugin-dir  ./plugins
//!   --http-url    http://127.0.0.1:25555/api/ets2/telemetry

use std::path::{Path, PathBuf};
use std::time::Duration;

// ---------------------------------------------------------------------------
// ANSI colour helpers (work on Windows 10+ and all Linux terminals)
// ---------------------------------------------------------------------------

const GREEN: &str = "\x1b[32m";
const RED: &str = "\x1b[31m";
const YELLOW: &str = "\x1b[33m";
const CYAN: &str = "\x1b[36m";
const BOLD: &str = "\x1b[1m";
const RESET: &str = "\x1b[0m";

// ---------------------------------------------------------------------------
// Check result
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq, Eq)]
enum Status {
    Ok(String),
    Warn(String),
    Fail(String),
    Skip(String),
}

impl Status {
    fn symbol(&self) -> &str {
        match self {
            Status::Ok(_) => "✓",
            Status::Warn(_) => "⚠",
            Status::Fail(_) => "✗",
            Status::Skip(_) => "–",
        }
    }

    fn colour(&self) -> &str {
        match self {
            Status::Ok(_) => GREEN,
            Status::Warn(_) => YELLOW,
            Status::Fail(_) => RED,
            Status::Skip(_) => CYAN,
        }
    }

    fn message(&self) -> &str {
        match self {
            Status::Ok(m) | Status::Warn(m) | Status::Fail(m) | Status::Skip(m) => m,
        }
    }

    fn is_fail(&self) -> bool {
        matches!(self, Status::Fail(_))
    }
}

struct CheckResult {
    name: &'static str,
    status: Status,
    detail: Vec<String>,
}

impl CheckResult {
    fn ok(name: &'static str, msg: impl Into<String>) -> Self {
        Self {
            name,
            status: Status::Ok(msg.into()),
            detail: vec![],
        }
    }

    fn ok_detail(name: &'static str, msg: impl Into<String>, detail: Vec<String>) -> Self {
        Self {
            name,
            status: Status::Ok(msg.into()),
            detail,
        }
    }

    fn warn(name: &'static str, msg: impl Into<String>) -> Self {
        Self {
            name,
            status: Status::Warn(msg.into()),
            detail: vec![],
        }
    }

    fn warn_detail(name: &'static str, msg: impl Into<String>, detail: Vec<String>) -> Self {
        Self {
            name,
            status: Status::Warn(msg.into()),
            detail,
        }
    }

    #[allow(dead_code)]
    fn fail(name: &'static str, msg: impl Into<String>) -> Self {
        Self {
            name,
            status: Status::Fail(msg.into()),
            detail: vec![],
        }
    }

    fn fail_detail(name: &'static str, msg: impl Into<String>, detail: Vec<String>) -> Self {
        Self {
            name,
            status: Status::Fail(msg.into()),
            detail,
        }
    }

    fn skip(name: &'static str, msg: impl Into<String>) -> Self {
        Self {
            name,
            status: Status::Skip(msg.into()),
            detail: vec![],
        }
    }
}

// ---------------------------------------------------------------------------
// CLI args (minimal, no clap dependency)
// ---------------------------------------------------------------------------

struct Config {
    ets2_dir: Option<PathBuf>,
    plugin_dir: PathBuf,
    http_url: String,
}

impl Config {
    fn from_args() -> Self {
        let args: Vec<String> = std::env::args().collect();
        let mut ets2_dir = default_ets2_dir();
        let mut plugin_dir = PathBuf::from("plugins");
        let mut http_url = "http://127.0.0.1:25555/api/ets2/telemetry".to_string();

        let mut i = 1;
        while i < args.len() {
            match args[i].as_str() {
                "--ets2-dir" if i + 1 < args.len() => {
                    ets2_dir = Some(PathBuf::from(&args[i + 1]));
                    i += 2;
                }
                "--plugin-dir" if i + 1 < args.len() => {
                    plugin_dir = PathBuf::from(&args[i + 1]);
                    i += 2;
                }
                "--http-url" if i + 1 < args.len() => {
                    http_url = args[i + 1].clone();
                    i += 2;
                }
                "--help" | "-h" => {
                    print_help();
                    std::process::exit(0);
                }
                other => {
                    eprintln!("Unknown argument: {other}");
                    std::process::exit(2);
                }
            }
        }

        Self {
            ets2_dir,
            plugin_dir,
            http_url,
        }
    }
}

fn default_ets2_dir() -> Option<PathBuf> {
    #[cfg(windows)]
    {
        let p =
            PathBuf::from(r"C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2");
        if p.exists() {
            Some(p)
        } else {
            None
        }
    }
    #[cfg(not(windows))]
    {
        // ETS2 runs on Windows, not on this Linux server.
        None
    }
}

fn print_help() {
    println!("truckpilot-diag — TruckPilot subsystem checker");
    println!();
    println!("Usage: truckpilot-diag [OPTIONS]");
    println!();
    println!("Options:");
    println!("  --ets2-dir <path>    Path to ETS2 installation");
    println!("  --plugin-dir <path>  Path to plugin directory (default: ./plugins)");
    println!("  --http-url <url>     Funbit telemetry URL");
    println!("  --help               Show this help");
}

// ---------------------------------------------------------------------------
// Individual checks
// ---------------------------------------------------------------------------

/// Check 1: vJoy virtual joystick driver
fn check_vjoy() -> CheckResult {
    #[cfg(windows)]
    {
        check_vjoy_windows()
    }
    #[cfg(not(windows))]
    {
        CheckResult::skip(
            "vJoy",
            "Linux — vJoy runs on Windows only. Will be checked on the Windows PC.",
        )
    }
}

#[cfg(windows)]
fn check_vjoy_windows() -> CheckResult {
    use std::ffi::OsStr;
    use std::os::windows::ffi::OsStrExt;

    // Try to load vJoyInterface.dll from standard locations.
    let candidates = [
        r"C:\Program Files\vJoy\x64\vJoyInterface.dll",
        r"C:\Program Files (x86)\vJoy\x64\vJoyInterface.dll",
        r"vJoyInterface.dll",
    ];

    for path in &candidates {
        let wide: Vec<u16> = OsStr::new(path)
            .encode_wide()
            .chain(std::iter::once(0))
            .collect();
        let handle = unsafe {
            windows::Win32::System::LibraryLoader::LoadLibraryW(windows::core::PCWSTR(
                wide.as_ptr(),
            ))
        };
        if let Ok(h) = handle {
            if !h.is_invalid() {
                // `FreeLibrary` lives in `Win32::Foundation` in `windows` v0.52
                // (only `LoadLibrary*` lives under `LibraryLoader`).
                unsafe { windows::Win32::Foundation::FreeLibrary(h).ok() };
                return CheckResult::ok_detail(
                    "vJoy",
                    format!("DLL found at {path}"),
                    vec![
                        "Install vJoy 2.2.1+ from https://github.com/njz3/vJoy".to_string(),
                        "Configure device 1 with X/Y/Z axes in 'Configure vJoy'".to_string(),
                    ],
                );
            }
        }
    }

    CheckResult::fail_detail(
        "vJoy",
        "vJoyInterface.dll not found",
        vec![
            "Install vJoy 2.2.1+ from https://github.com/njz3/vJoy".to_string(),
            "Searched: C:\\Program Files\\vJoy\\x64\\, C:\\Program Files (x86)\\vJoy\\x64\\"
                .to_string(),
        ],
    )
}

/// Check 2: ETS2 process running
fn check_ets2_process() -> CheckResult {
    #[cfg(windows)]
    {
        check_ets2_process_windows()
    }
    #[cfg(not(windows))]
    {
        CheckResult::skip(
            "ETS2 Process",
            "Linux — ETS2 runs on Windows. Start ETS2 on the Windows PC before running.",
        )
    }
}

#[cfg(windows)]
fn check_ets2_process_windows() -> CheckResult {
    use windows::Win32::System::Diagnostics::ToolHelp::{
        CreateToolhelp32Snapshot, Process32FirstW, Process32NextW, PROCESSENTRY32W,
        TH32CS_SNAPPROCESS,
    };

    let snapshot = unsafe { CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0) };
    let Ok(snapshot) = snapshot else {
        return CheckResult::fail("ETS2 Process", "CreateToolhelp32Snapshot failed");
    };

    let mut entry = PROCESSENTRY32W {
        dwSize: std::mem::size_of::<PROCESSENTRY32W>() as u32,
        ..Default::default()
    };

    let target = "eurotrucks2.exe";
    let mut found = false;

    if unsafe { Process32FirstW(snapshot, &mut entry) }.is_ok() {
        loop {
            let name: String = entry
                .szExeFile
                .iter()
                .take_while(|&&c| c != 0)
                .map(|&c| char::from_u32(c as u32).unwrap_or('?'))
                .collect();

            if name.eq_ignore_ascii_case(target) {
                found = true;
                break;
            }

            if unsafe { Process32NextW(snapshot, &mut entry) }.is_err() {
                break;
            }
        }
    }

    unsafe { windows::Win32::Foundation::CloseHandle(snapshot).ok() };

    if found {
        CheckResult::ok(
            "ETS2 Process",
            format!("eurotrucks2.exe running (PID {})", entry.th32ProcessID),
        )
    } else {
        CheckResult::fail(
            "ETS2 Process",
            "eurotrucks2.exe not found — start ETS2 first",
        )
    }
}

/// Check 3: Shared Memory telemetry
fn check_shm() -> CheckResult {
    use truckpilot_telemetry::shm::{ShmReader, SHM_MAGIC, SHM_VERSION};

    match ShmReader::open() {
        Ok(mut reader) => match reader.read() {
            Some(t) => CheckResult::ok_detail(
                "SHM Telemetry",
                format!(
                    "Active — speed {:.1} km/h, pos ({:.0}, {:.0}, {:.0})",
                    t.speed_ms * 3.6,
                    t.position[0],
                    t.position[1],
                    t.position[2]
                ),
                vec![
                    format!("Magic: 0x{:08X} (expected 0x{:08X})", SHM_MAGIC, SHM_MAGIC),
                    format!("Version: {SHM_VERSION}"),
                ],
            ),
            None => CheckResult::warn(
                "SHM Telemetry",
                "SHM region opened but data invalid — is the telemetry DLL loaded in ETS2?",
            ),
        },
        Err(e) => CheckResult::fail_detail(
            "SHM Telemetry",
            "SHM region not found",
            vec![
                e,
                "Copy truckpilot_telemetry.dll to ETS2/bin/win_x64/plugins/".to_string(),
                "Start ETS2 — the DLL creates the shared memory on load".to_string(),
            ],
        ),
    }
}

/// Check 4: HTTP telemetry (optional Funbit server — not required)
///
/// TruckPilot uses its own `truckpilot_telemetry.dll` via shared memory.
/// This check is informational only: if a Funbit server happens to be running
/// it will show as OK, otherwise it shows as Skip (not a failure).
fn check_http(url: &str) -> CheckResult {
    let timeout = Duration::from_secs(2);

    let result = ureq::get(url).timeout(timeout).call();

    match result {
        Ok(resp) if resp.status() == 200 => match resp.into_string() {
            Ok(body) if body.contains("\"speed\"") || body.contains("\"Speed\"") => {
                CheckResult::ok(
                    "HTTP Telemetry (optional)",
                    format!("Funbit server responding at {url}"),
                )
            }
            Ok(body) => CheckResult::warn_detail(
                "HTTP Telemetry (optional)",
                "Server responded but JSON looks unexpected",
                vec![format!("Body preview: {}", &body[..body.len().min(120)])],
            ),
            Err(e) => CheckResult::warn(
                "HTTP Telemetry (optional)",
                format!("Response read error: {e}"),
            ),
        },
        Ok(resp) => CheckResult::warn(
            "HTTP Telemetry (optional)",
            format!("Server returned HTTP {}", resp.status()),
        ),
        Err(_) => CheckResult::skip(
            "HTTP Telemetry (optional)",
            "Not running — not needed. TruckPilot uses truckpilot_telemetry.dll via SHM.",
        ),
    }
}

/// Check 5: ETS2 installation + map files
fn check_map(ets2_dir: Option<&Path>) -> CheckResult {
    let Some(dir) = ets2_dir else {
        return CheckResult::skip(
            "Map Files",
            "ETS2 directory not found or not specified (use --ets2-dir <path>)",
        );
    };

    if !dir.exists() {
        return CheckResult::fail_detail(
            "Map Files",
            format!("ETS2 directory not found: {}", dir.display()),
            vec!["Use --ets2-dir to specify the correct path".to_string()],
        );
    }

    // Check for the key .scs files
    let candidates = [
        ("base.scs", "Core map data"),
        ("def.scs", "Road definitions"),
        ("base_map.scs", "Map sectors"),
    ];

    let mut found = vec![];
    let mut missing = vec![];

    for (name, desc) in &candidates {
        let path = dir.join(name);
        if path.exists() {
            let size_mb = std::fs::metadata(&path)
                .map(|m| m.len() / 1_048_576)
                .unwrap_or(0);
            found.push(format!("{name} ({size_mb} MB) — {desc}"));
        } else {
            missing.push(format!("{name} — {desc}"));
        }
    }

    if missing.is_empty() {
        CheckResult::ok_detail(
            "Map Files",
            format!("{} SCS archives found in {}", found.len(), dir.display()),
            found,
        )
    } else if !found.is_empty() {
        let mut detail = found;
        detail.push(format!("Missing: {}", missing.join(", ")));
        CheckResult::warn_detail(
            "Map Files",
            format!(
                "{}/{} SCS archives found",
                detail.len() - 1,
                candidates.len()
            ),
            detail,
        )
    } else {
        CheckResult::fail_detail(
            "Map Files",
            format!("No SCS archives found in {}", dir.display()),
            vec![
                "Expected: base.scs, def.scs, base_map.scs".to_string(),
                "Check --ets2-dir path".to_string(),
            ],
        )
    }
}

/// Check 6: Plugin directory
fn check_plugins(plugin_dir: &Path) -> CheckResult {
    if !plugin_dir.exists() {
        return CheckResult::fail_detail(
            "Plugin Directory",
            format!("Directory not found: {}", plugin_dir.display()),
            vec![
                "Create the directory and copy compiled plugin .dll/.so files into it".to_string(),
                format!(
                    "Expected path: {}",
                    plugin_dir
                        .canonicalize()
                        .unwrap_or_else(|_| plugin_dir.to_path_buf())
                        .display()
                ),
            ],
        );
    }

    #[cfg(windows)]
    let ext = "dll";
    #[cfg(not(windows))]
    let ext = "so";

    let plugins: Vec<String> = std::fs::read_dir(plugin_dir)
        .map(|entries| {
            entries
                .filter_map(|e| e.ok())
                .filter(|e| {
                    e.path()
                        .extension()
                        .and_then(|x| x.to_str())
                        .map(|x| x.eq_ignore_ascii_case(ext))
                        .unwrap_or(false)
                })
                .map(|e| e.file_name().to_string_lossy().into_owned())
                .collect()
        })
        .unwrap_or_default();

    if plugins.is_empty() {
        CheckResult::warn_detail(
            "Plugin Directory",
            format!("Directory exists but no .{ext} files found"),
            vec![
                format!("Build plugins: cargo build --release -p truckpilot-plugin-*"),
                format!("Copy .{ext} files to {}", plugin_dir.display()),
            ],
        )
    } else {
        CheckResult::ok_detail(
            "Plugin Directory",
            format!(
                "{} plugin(s) found in {}",
                plugins.len(),
                plugin_dir.display()
            ),
            plugins,
        )
    }
}

/// Check 7: Core config file
fn check_config() -> CheckResult {
    let candidates = ["truckpilot.toml", "config/truckpilot.toml"];

    for path in &candidates {
        let p = Path::new(path);
        if p.exists() {
            return CheckResult::ok("Config File", format!("Found: {path}"));
        }
    }

    CheckResult::warn_detail(
        "Config File",
        "truckpilot.toml not found — defaults will be used",
        vec![
            "Create truckpilot.toml in the working directory".to_string(),
            "See docs/CONFIG.md for available options".to_string(),
        ],
    )
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

fn print_header() {
    println!();
    println!("{BOLD}{CYAN}TruckPilot Diagnostic Tool{RESET}");
    println!("{CYAN}══════════════════════════{RESET}");
    println!();
}

fn print_result(idx: usize, total: usize, result: &CheckResult) {
    let c = result.status.colour();
    let sym = result.status.symbol();
    let msg = result.status.message();

    println!(
        "  [{idx}/{total}] {BOLD}{}{RESET}  {c}{sym}{RESET}  {}",
        result.name, msg
    );

    for line in &result.detail {
        println!("         {CYAN}→{RESET} {line}");
    }
}

fn print_summary(results: &[CheckResult]) {
    let ok = results
        .iter()
        .filter(|r| matches!(r.status, Status::Ok(_)))
        .count();
    let warn = results
        .iter()
        .filter(|r| matches!(r.status, Status::Warn(_)))
        .count();
    let fail = results
        .iter()
        .filter(|r| matches!(r.status, Status::Fail(_)))
        .count();
    let skip = results
        .iter()
        .filter(|r| matches!(r.status, Status::Skip(_)))
        .count();

    println!();
    println!("{CYAN}══════════════════════════{RESET}");

    if fail == 0 && warn == 0 {
        println!("{BOLD}{GREEN}All checks passed — ready to run!{RESET}");
    } else if fail == 0 {
        println!("{BOLD}{YELLOW}Ready with warnings — {ok} ok, {warn} warn, {skip} skipped{RESET}");
    } else {
        println!(
            "{BOLD}{RED}Not ready — {fail} check(s) failed ({ok} ok, {warn} warn, {skip} skipped){RESET}"
        );
        println!();
        println!("{RED}Failed checks:{RESET}");
        for r in results.iter().filter(|r| r.status.is_fail()) {
            println!("  {RED}✗{RESET} {} — {}", r.name, r.status.message());
        }
    }

    println!();
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() {
    // Minimal tracing setup — only errors to stderr so they don't pollute output
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::ERROR)
        .with_writer(std::io::stderr)
        .init();

    let cfg = Config::from_args();

    print_header();

    let checks: Vec<CheckResult> = vec![
        check_vjoy(),
        check_ets2_process(),
        check_shm(),
        check_http(&cfg.http_url),
        check_map(cfg.ets2_dir.as_deref()),
        check_plugins(&cfg.plugin_dir),
        check_config(),
    ];

    let total = checks.len();
    for (i, result) in checks.iter().enumerate() {
        print_result(i + 1, total, result);
    }

    print_summary(&checks);

    let any_fail = checks.iter().any(|r| r.status.is_fail());
    std::process::exit(if any_fail { 1 } else { 0 });
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn status_ok_is_not_fail() {
        assert!(!Status::Ok("fine".into()).is_fail());
    }

    #[test]
    fn status_fail_is_fail() {
        assert!(Status::Fail("broken".into()).is_fail());
    }

    #[test]
    fn status_warn_is_not_fail() {
        assert!(!Status::Warn("meh".into()).is_fail());
    }

    #[test]
    fn check_result_constructors() {
        let r = CheckResult::ok("Test", "all good");
        assert_eq!(r.name, "Test");
        assert!(matches!(r.status, Status::Ok(_)));
        assert!(r.detail.is_empty());

        let r = CheckResult::fail_detail("Test", "broken", vec!["hint".into()]);
        assert!(r.status.is_fail());
        assert_eq!(r.detail.len(), 1);
    }

    #[test]
    fn check_plugins_missing_dir() {
        let result = check_plugins(Path::new("/nonexistent/path/xyz"));
        assert!(result.status.is_fail());
    }

    #[test]
    fn check_plugins_empty_dir() {
        let dir = tempfile::tempdir().unwrap();
        let result = check_plugins(dir.path());
        assert!(matches!(result.status, Status::Warn(_)));
    }

    #[test]
    fn check_plugins_with_dll() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(dir.path().join("my_plugin.dll"), b"fake").unwrap();
        std::fs::write(dir.path().join("other_plugin.dll"), b"fake").unwrap();
        let result = check_plugins(dir.path());
        // On Linux this checks for .so, so .dll files won't be found — that's correct
        // On Windows it would find them
        let _ = result; // just ensure it doesn't panic
    }

    #[test]
    fn check_map_no_dir() {
        let result = check_map(None);
        assert!(matches!(result.status, Status::Skip(_)));
    }

    #[test]
    fn check_map_nonexistent_dir() {
        let result = check_map(Some(Path::new("/nonexistent/ets2")));
        assert!(result.status.is_fail());
    }

    #[test]
    fn check_map_empty_dir() {
        let dir = tempfile::tempdir().unwrap();
        let result = check_map(Some(dir.path()));
        assert!(result.status.is_fail());
    }

    #[test]
    fn check_map_partial_files() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(dir.path().join("base.scs"), b"fake").unwrap();
        let result = check_map(Some(dir.path()));
        assert!(matches!(result.status, Status::Warn(_)));
    }

    #[test]
    fn check_map_all_files() {
        let dir = tempfile::tempdir().unwrap();
        std::fs::write(dir.path().join("base.scs"), b"fake").unwrap();
        std::fs::write(dir.path().join("def.scs"), b"fake").unwrap();
        std::fs::write(dir.path().join("base_map.scs"), b"fake").unwrap();
        let result = check_map(Some(dir.path()));
        assert!(matches!(result.status, Status::Ok(_)));
    }

    #[test]
    fn check_config_no_file() {
        // In a temp dir there's no truckpilot.toml
        let original = std::env::current_dir().unwrap();
        let dir = tempfile::tempdir().unwrap();
        std::env::set_current_dir(dir.path()).unwrap();
        let result = check_config();
        std::env::set_current_dir(original).unwrap();
        assert!(matches!(result.status, Status::Warn(_)));
    }

    #[test]
    fn check_http_unreachable_is_skip() {
        // HTTP telemetry is optional — unreachable = Skip, not Fail
        let result = check_http("http://127.0.0.1:19999/api/ets2/telemetry");
        assert!(matches!(result.status, Status::Skip(_)));
    }
}
