use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::SystemTime;

fn main() {
    let mut args = std::env::args().skip(1);
    match args.next().as_deref() {
        Some("copy-plugins") => copy_plugins(args),
        Some("build-release") => build_release(),
        Some("deploy-ets2-telemetry") => deploy_ets2_telemetry(args),
        Some(cmd) => {
            eprintln!("Unknown command: {cmd}");
            eprintln!();
            print_usage();
            std::process::exit(1);
        }
        None => {
            print_usage();
            std::process::exit(1);
        }
    }
}

fn print_usage() {
    eprintln!("Usage: cargo xtask <command>");
    eprintln!();
    eprintln!("Commands:");
    eprintln!("  copy-plugins [--debug]       Copy plugin DLLs from target/ to plugins/");
    eprintln!(
        "  build-release                cargo build --workspace --release, then copy-plugins"
    );
    eprintln!(
        "  deploy-ets2-telemetry [DIR]  Build (msvc) + copy truckpilot_telemetry.dll to ETS2 plugins dir"
    );
    eprintln!();
    eprintln!("Flags for copy-plugins:");
    eprintln!("  --debug   Copy from target/debug/ instead of target/release/");
    eprintln!();
    eprintln!("deploy-ets2-telemetry DIR: path to ETS2 bin/win_x64/plugins/");
    eprintln!("  Falls back to ETS2_PLUGINS_DIR env var if DIR is omitted.");
}

fn build_release() {
    println!("==> cargo build --workspace --release");
    let status = Command::new("cargo")
        .args(["build", "--workspace", "--release"])
        .status()
        .unwrap_or_else(|e| {
            eprintln!("Failed to spawn cargo: {e}");
            std::process::exit(1);
        });
    if !status.success() {
        std::process::exit(status.code().unwrap_or(1));
    }
    println!();
    println!("==> Deploying plugin DLLs to plugins/");
    copy_plugins_impl("release");
}

/// Standard build target for the telemetry DLL. Explicitly msvc so the build
/// output lands in a deterministic, toolchain-independent directory
/// (`target/x86_64-pc-windows-msvc/release/`). Two reasons this is pinned:
///
/// 1. An explicit `--target` makes cargo write to `target/<triple>/release/`,
///    NOT the host-default `target/release/`. The old deploy read
///    `target/release/` while builds (cross-gnu, or any explicit-target build)
///    landed in `target/<triple>/release/` — so deploy silently shipped a
///    stale DLL every time. Build and deploy now read the SAME directory.
/// 2. msvc needs no external toolchain (gnu's mingw linker went missing before),
///    so this build works on a stock Windows dev box.
const TELEMETRY_TARGET: &str = "x86_64-pc-windows-msvc";
const TELEMETRY_DLL: &str = "truckpilot_telemetry.dll";

/// Directory the telemetry DLL is built into for [`TELEMETRY_TARGET`].
fn telemetry_dll_build_path(root: &Path) -> PathBuf {
    root.join("target")
        .join(TELEMETRY_TARGET)
        .join("release")
        .join(TELEMETRY_DLL)
}

/// Last-modified time of `p`, or `None` if it can't be read.
fn file_mtime(p: &Path) -> Option<SystemTime> {
    fs::metadata(p).and_then(|m| m.modified()).ok()
}

/// Newest mtime among all `*.rs` files under `dir` (recursive). `None` if the
/// directory can't be read or holds no `.rs` files.
fn newest_rs_mtime(dir: &Path) -> Option<SystemTime> {
    let mut newest: Option<SystemTime> = None;
    let mut stack = vec![dir.to_path_buf()];
    while let Some(d) = stack.pop() {
        let Ok(rd) = fs::read_dir(&d) else { continue };
        for entry in rd.flatten() {
            let path = entry.path();
            if path.is_dir() {
                stack.push(path);
            } else if path.extension().and_then(|s| s.to_str()) == Some("rs") {
                if let Ok(m) = entry.metadata().and_then(|md| md.modified()) {
                    newest = Some(newest.map_or(m, |n| n.max(m)));
                }
            }
        }
    }
    newest
}

/// Would deploying this DLL ship stale code? `None` dll_mtime = missing DLL =
/// stale. If we have no source mtime to compare against we trust the DLL.
/// Otherwise stale iff the DLL predates the newest source file.
fn is_stale(dll_mtime: Option<SystemTime>, newest_src_mtime: Option<SystemTime>) -> bool {
    match (dll_mtime, newest_src_mtime) {
        (None, _) => true,
        (Some(_), None) => false,
        (Some(dll), Some(src)) => dll < src,
    }
}

fn deploy_ets2_telemetry(mut args: impl Iterator<Item = String>) {
    let root = workspace_root();

    // 1) Build fresh for the standard msvc target. This is the heart of the
    //    stale-deploy fix: build and deploy now reference the SAME directory,
    //    so a successful deploy can never ship an old DLL.
    println!("==> cargo build --release --target {TELEMETRY_TARGET} -p truckpilot-telemetry-dll");
    let status = Command::new("cargo")
        .args([
            "build",
            "--release",
            "--target",
            TELEMETRY_TARGET,
            "-p",
            "truckpilot-telemetry-dll",
        ])
        .current_dir(&root)
        .status()
        .unwrap_or_else(|e| {
            eprintln!("Failed to spawn cargo: {e}");
            std::process::exit(1);
        });
    if !status.success() {
        eprintln!("Build failed — not deploying.");
        std::process::exit(status.code().unwrap_or(1));
    }

    let src = telemetry_dll_build_path(&root);

    // 2) Stale guard (backstop). The build above should have produced a fresh
    //    DLL; if it is missing or older than the crate sources, abort loudly
    //    instead of silently shipping stale code (the bug this whole change
    //    exists to kill).
    let src_dir = root.join("crates").join("telemetry-dll").join("src");
    let dll_mtime = file_mtime(&src);
    let src_mtime = newest_rs_mtime(&src_dir);
    if is_stale(dll_mtime, src_mtime) {
        if dll_mtime.is_none() {
            eprintln!("DLL not found at {} after build.", src.display());
        } else {
            eprintln!(
                "STALE: DLL at {} is older than sources in {} — refusing to deploy.",
                src.display(),
                src_dir.display()
            );
        }
        eprintln!("Build the telemetry DLL for {TELEMETRY_TARGET} and retry.");
        std::process::exit(1);
    }

    // 3) Destination: explicit arg wins, else ETS2_PLUGINS_DIR.
    let dst_dir = args
        .next()
        .or_else(|| std::env::var("ETS2_PLUGINS_DIR").ok())
        .unwrap_or_else(|| {
            eprintln!("No destination directory provided.");
            eprintln!("Usage: cargo deploy-ets2 <path/to/bin/win_x64/plugins/>");
            eprintln!("  or set ETS2_PLUGINS_DIR env var.");
            std::process::exit(1);
        });
    let dst_dir = PathBuf::from(&dst_dir);
    if !dst_dir.exists() {
        eprintln!("Destination directory does not exist: {}", dst_dir.display());
        std::process::exit(1);
    }

    // 4) Copy and report the deployed file's age. `fs::copy` preserves the
    //    source mtime on Windows, so this shows when the DLL was BUILT — a
    //    few seconds means fresh; minutes/days means something is wrong.
    let dst = dst_dir.join(TELEMETRY_DLL);
    match fs::copy(&src, &dst) {
        Ok(_) => {
            let age = file_mtime(&dst)
                .and_then(|m| m.elapsed().ok())
                .map(|d| format!("{:.1}s ago", d.as_secs_f64()))
                .unwrap_or_else(|| "unknown".into());
            println!("Deployed {TELEMETRY_DLL} -> {}", dst.display());
            println!("  from  : {}", src.display());
            println!("  built : {age}  (build timestamp; stale-guard already confirmed it is newer than sources)");
        }
        Err(e) => {
            eprintln!("Copy failed: {e}");
            eprintln!(
                "The DLL in {} is locked. Close ETS2 (and stop the daemon) first.",
                dst_dir.display()
            );
            std::process::exit(1);
        }
    }
}

/// Returns true if the TruckPilot daemon process is currently running.
/// On Windows, uses `tasklist`; on other platforms, uses `pgrep`.
/// Returns false when detection fails (safe default: allow deploy).
#[cfg(windows)]
fn daemon_is_running() -> bool {
    Command::new("tasklist")
        .args(["/FI", "IMAGENAME eq truckpilot-core.exe", "/NH"])
        .output()
        .map(|o| String::from_utf8_lossy(&o.stdout).contains("truckpilot-core.exe"))
        .unwrap_or(false)
}

#[cfg(not(windows))]
fn daemon_is_running() -> bool {
    Command::new("pgrep")
        .args(["-x", "truckpilot-core"])
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

/// Returns true if `path` cannot be opened for writing (locked by another process).
/// On Windows, os error 32 = ERROR_SHARING_VIOLATION.
fn dll_is_locked(path: &Path) -> bool {
    match std::fs::OpenOptions::new().write(true).create(false).open(path) {
        Err(e) => e.raw_os_error() == Some(32),
        Ok(_) => false,
    }
}

fn copy_plugins(args: impl Iterator<Item = String>) {
    let mut debug = false;
    for arg in args {
        match arg.as_str() {
            "--debug" => debug = true,
            other => {
                eprintln!("Unknown flag: {other}");
                std::process::exit(1);
            }
        }
    }
    let profile = if debug { "debug" } else { "release" };
    copy_plugins_impl(profile);
}

fn copy_plugins_impl(profile: &str) {
    let root = workspace_root();
    let src = root.join("target").join(profile);
    let dst = root.join("plugins");

    if !src.exists() {
        eprintln!("Source directory does not exist: {}", src.display());
        eprintln!("Run `cargo build --workspace --{}` first.", profile);
        std::process::exit(1);
    }

    // Daemon-Guard: abort before any copy attempt if daemon is running.
    // Prevents the Windows FILE_SHARE_DELETE trap where the copy appears to
    // succeed (delete-pending + new file written) but the daemon continues
    // executing the old DLL mapping until it is fully restarted.
    if daemon_is_running() {
        eprintln!("ERROR: TruckPilot daemon (truckpilot-core.exe) is running.");
        eprintln!("Stop the daemon first, then re-run `cargo xtask copy-plugins`.");
        std::process::exit(1);
    }

    // Lock-Guard: catch any other process holding a plugin DLL (e.g. debugger,
    // antivirus with an exclusive handle). Daemon-Guard above handles the common
    // case; this is a defence-in-depth backstop.
    if dst.exists() {
        if let Ok(entries) = fs::read_dir(&dst) {
            for entry in entries.flatten() {
                let path = entry.path();
                if is_plugin_file(&path) && dll_is_locked(&path) {
                    let name = path
                        .file_name()
                        .map(|n| n.to_string_lossy().into_owned())
                        .unwrap_or_else(|| path.display().to_string());
                    eprintln!("ERROR: Plugin DLL `{name}` is locked by another process.");
                    eprintln!(
                        "Stop whatever holds the DLL, then re-run `cargo xtask copy-plugins`."
                    );
                    std::process::exit(1);
                }
            }
        }
    }

    let (copied, errors) = deploy_plugins_to_dir(&src, &dst);
    if errors.is_empty() {
        println!("{} plugin(s) deployed to plugins/", copied);
    } else {
        eprintln!(
            "Deploy FAILED: {} plugin(s) deployed, {} ERROR(s):",
            copied,
            errors.len()
        );
        for err in &errors {
            eprintln!("  - {}", err);
        }
        eprintln!();
        eprintln!("Action: stop the daemon, then run `cargo xtask copy-plugins`");
        std::process::exit(2);
    }
}

fn deploy_plugins_to_dir(src: &Path, dst: &Path) -> (usize, Vec<String>) {
    if let Err(e) = fs::create_dir_all(dst) {
        return (0, vec![format!("Cannot create {:?}: {e}", dst.display())]);
    }

    let mut stale_errors: Vec<String> = Vec::new();
    if let Ok(rd) = fs::read_dir(dst) {
        let stale: Vec<_> = rd
            .flatten()
            .filter(|e| is_plugin_file(e.path().as_path()))
            .collect();
        for entry in &stale {
            let path = entry.path();
            match fs::remove_file(&path) {
                Ok(()) => {
                    println!("  removed  {}", path.file_name().unwrap().to_string_lossy());
                }
                Err(e) => {
                    stale_errors.push(format!("Cannot remove stale {}: {e}", path.display()));
                }
            }
        }
    }

    let mut copied = 0usize;
    let mut errors: Vec<String> = Vec::new();
    if let Ok(entries) = fs::read_dir(src) {
        for entry in entries.flatten() {
            let path = entry.path();
            if !is_plugin_file(path.as_path()) {
                continue;
            }
            let file_name = path.file_name().unwrap();
            let stem = Path::new(file_name).file_stem().unwrap_or_default();
            let stem_str = stem.to_string_lossy();
            if !stem_str.starts_with("truckpilot_plugin_") {
                continue;
            }
            let dst_path = dst.join(file_name);
            match fs::copy(&path, &dst_path) {
                Ok(_) => {
                    println!("  copied   {}", file_name.to_string_lossy());
                    copied += 1;
                }
                Err(e) => {
                    errors.push(format!(
                        "{} ({} → {})",
                        e,
                        file_name.to_string_lossy(),
                        dst_path.display()
                    ));
                }
            }
        }
    }

    errors.extend(stale_errors);
    println!();
    (copied, errors)
}

fn is_plugin_file(path: &Path) -> bool {
    matches!(
        path.extension().and_then(|s| s.to_str()),
        Some("dll" | "so" | "dylib")
    )
}

/// Returns the workspace root by walking up two directories from this
/// crate's manifest (crates/xtask -> crates -> workspace root).
fn workspace_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("crates/")
        .parent()
        .expect("workspace root")
        .to_path_buf()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deploy_plugins_all_success() {
        let tmp = tempfile::tempdir().unwrap();
        let src = tmp.path().join("src");
        let dst = tmp.path().join("dst");
        fs::create_dir_all(&src).unwrap();
        fs::create_dir_all(&dst).unwrap();

        let dll = src.join("truckpilot_plugin_test.dll");
        fs::write(&dll, b"fake dll content").unwrap();

        let (copied, errors) = deploy_plugins_to_dir(&src, &dst);
        assert_eq!(copied, 1);
        assert!(errors.is_empty());
        assert!(dst.join("truckpilot_plugin_test.dll").exists());
    }

    #[test]
    fn deploy_skips_non_plugin_files() {
        let tmp = tempfile::tempdir().unwrap();
        let src = tmp.path().join("src");
        let dst = tmp.path().join("dst");
        fs::create_dir_all(&src).unwrap();
        fs::create_dir_all(&dst).unwrap();

        fs::write(src.join("truckpilot_plugin_test.dll"), b"dll").unwrap();
        fs::write(src.join("truckpilot_telemetry.dll"), b"dll").unwrap();
        fs::write(src.join("other.pdb"), b"pdb").unwrap();

        let (copied, errors) = deploy_plugins_to_dir(&src, &dst);
        assert_eq!(copied, 1);
        assert!(errors.is_empty());
        assert!(dst.join("truckpilot_plugin_test.dll").exists());
        assert!(!dst.join("truckpilot_telemetry.dll").exists());
    }

    #[test]
    fn deploy_empty_source_dir() {
        let tmp = tempfile::tempdir().unwrap();
        let src = tmp.path().join("src");
        let dst = tmp.path().join("dst");
        fs::create_dir_all(&src).unwrap();

        let (copied, errors) = deploy_plugins_to_dir(&src, &dst);
        assert_eq!(copied, 0);
        assert!(errors.is_empty());
    }

    #[test]
    fn deploy_nonexistent_source_dir() {
        let tmp = tempfile::tempdir().unwrap();
        let src = tmp.path().join("nonexistent");
        let dst = tmp.path().join("dst");

        let (copied, errors) = deploy_plugins_to_dir(&src, &dst);
        assert_eq!(copied, 0);
        assert!(errors.is_empty());
    }

    #[test]
    fn deploy_dst_is_file_not_dir() {
        let tmp = tempfile::tempdir().unwrap();
        let src = tmp.path().join("src");
        let dst = tmp.path().join("dst");
        fs::create_dir_all(&src).unwrap();
        fs::write(&dst, b"not a dir").unwrap();
        fs::write(src.join("truckpilot_plugin_test.dll"), b"dll").unwrap();

        let (copied, errors) = deploy_plugins_to_dir(&src, &dst);
        assert_eq!(copied, 0);
        assert_eq!(errors.len(), 1);
        assert!(errors[0].starts_with("Cannot create "));
    }

    #[test]
    fn deploy_stale_cleanup() {
        let tmp = tempfile::tempdir().unwrap();
        let src = tmp.path().join("src");
        let dst = tmp.path().join("dst");
        fs::create_dir_all(&src).unwrap();
        fs::create_dir_all(&dst).unwrap();

        let stale_dll = dst.join("truckpilot_plugin_old.dll");
        fs::write(&stale_dll, b"old").unwrap();
        fs::write(src.join("truckpilot_plugin_new.dll"), b"new").unwrap();

        let (copied, errors) = deploy_plugins_to_dir(&src, &dst);
        assert_eq!(copied, 1);
        assert!(errors.is_empty());
        assert!(dst.join("truckpilot_plugin_new.dll").exists());
        assert!(!dst.join("truckpilot_plugin_old.dll").exists());
    }

    #[test]
    fn deploy_skips_non_dll_extensions() {
        let tmp = tempfile::tempdir().unwrap();
        let src = tmp.path().join("src");
        let dst = tmp.path().join("dst");
        fs::create_dir_all(&src).unwrap();

        fs::write(src.join("truckpilot_plugin_test.dll"), b"dll").unwrap();
        fs::write(src.join("truckpilot_plugin_test.pdb"), b"pdb").unwrap();
        fs::write(src.join("truckpilot_plugin_test.txt"), b"txt").unwrap();

        let (copied, errors) = deploy_plugins_to_dir(&src, &dst);
        assert_eq!(copied, 1);
        assert!(errors.is_empty());
        assert!(dst.join("truckpilot_plugin_test.dll").exists());
        assert!(!dst.join("truckpilot_plugin_test.pdb").exists());
        assert!(!dst.join("truckpilot_plugin_test.txt").exists());
    }

    #[test]
    fn deploy_reports_error_for_lock_simulation_readonly_src() {
        let tmp = tempfile::tempdir().unwrap();
        let src = tmp.path().join("src");
        let dst = tmp.path().join("dst");
        fs::create_dir_all(&src).unwrap();
        fs::create_dir_all(&dst).unwrap();

        let dll = src.join("truckpilot_plugin_test.dll");
        fs::write(&dll, b"fake dll content").unwrap();

        let mut perms = fs::metadata(&dll).unwrap().permissions();
        perms.set_readonly(true);
        fs::set_permissions(&dll, perms).unwrap();

        let (copied, errors) = deploy_plugins_to_dir(&src, &dst);
        assert_eq!(copied, 1);
        assert!(errors.is_empty());
    }

    #[test]
    fn deploy_linux_so_files_copied() {
        let tmp = tempfile::tempdir().unwrap();
        let src = tmp.path().join("src");
        let dst = tmp.path().join("dst");
        fs::create_dir_all(&src).unwrap();

        fs::write(src.join("truckpilot_plugin_test.so"), b"so").unwrap();

        let (copied, errors) = deploy_plugins_to_dir(&src, &dst);
        assert_eq!(copied, 1);
        assert!(errors.is_empty());
        assert!(dst.join("truckpilot_plugin_test.so").exists());
    }

    // --- deploy-ets2-telemetry: path + stale-guard (the stale-deploy fix) ---

    #[test]
    fn build_path_uses_msvc_triple_not_host_default() {
        let root = Path::new("X:").join("repo");
        let p = telemetry_dll_build_path(&root);
        assert!(p.ends_with("truckpilot_telemetry.dll"));
        let s = p.to_string_lossy();
        assert!(s.contains("x86_64-pc-windows-msvc"), "path: {s}");
        assert!(s.contains("release"), "path: {s}");
        // Must NOT be the host-default target/release/ path that silently
        // shipped stale DLLs.
        let host_default = Path::new("target")
            .join("release")
            .join("truckpilot_telemetry.dll");
        assert!(!p.ends_with(&host_default), "must not be host-default: {s}");
    }

    #[test]
    fn is_stale_true_when_dll_missing() {
        assert!(is_stale(None, Some(SystemTime::UNIX_EPOCH)));
        assert!(is_stale(None, None));
    }

    #[test]
    fn is_stale_true_when_dll_older_than_source() {
        let older = SystemTime::UNIX_EPOCH;
        let newer = SystemTime::UNIX_EPOCH + std::time::Duration::from_secs(60);
        assert!(is_stale(Some(older), Some(newer)));
    }

    #[test]
    fn is_stale_false_when_dll_newer_or_equal() {
        let older = SystemTime::UNIX_EPOCH;
        let newer = SystemTime::UNIX_EPOCH + std::time::Duration::from_secs(60);
        assert!(!is_stale(Some(newer), Some(older)));
        assert!(!is_stale(Some(newer), Some(newer))); // equal = not stale
    }

    #[test]
    fn is_stale_false_when_no_source_mtime() {
        // Can't read sources → trust the built DLL rather than block deploy.
        assert!(!is_stale(Some(SystemTime::UNIX_EPOCH), None));
    }

    #[test]
    fn newest_rs_mtime_some_when_rs_present_recursive() {
        let tmp = tempfile::tempdir().unwrap();
        let sub = tmp.path().join("sub");
        fs::create_dir_all(&sub).unwrap();
        fs::write(sub.join("lib.rs"), b"fn x() {}").unwrap();
        assert!(newest_rs_mtime(tmp.path()).is_some());
    }

    #[test]
    fn newest_rs_mtime_none_without_rs() {
        let tmp = tempfile::tempdir().unwrap();
        fs::write(tmp.path().join("notes.txt"), b"x").unwrap();
        fs::write(tmp.path().join("Cargo.toml"), b"x").unwrap();
        assert!(newest_rs_mtime(tmp.path()).is_none());
    }

    #[test]
    fn dll_is_locked_false_for_nonexistent() {
        assert!(!dll_is_locked(Path::new("C:\\nonexistent_dll_path_12345_xtask.dll")));
    }

    #[test]
    fn dll_is_locked_false_for_unlocked_file() {
        let tmp = tempfile::tempdir().unwrap();
        let dll = tmp.path().join("truckpilot_plugin_test.dll");
        fs::write(&dll, b"fake dll content").unwrap();
        assert!(!dll_is_locked(&dll));
    }

    // Manual tests for locked-DLL / error collection path:
    //
    // Windows file-locking semantics differ from file-permission semantics.
    // `set_readonly(true)` on a directory or file does NOT actually block
    // same-user delete/copy on Windows. A real file lock (ERROR_SHARING_VIOLATION)
    // requires the target process to hold an open handle without FILE_SHARE_DELETE,
    // which cannot be simulated from a single-process unit test without platform-
    // specific Win32 API calls (CreateFileW with dwShareMode=0).
    //
    // Manual test workflows:
    //
    // 1. Daemon-Guard (copy_plugins_impl level, NEW):
    //    a. Start the daemon: `.\target\release\truckpilot-core.exe daemon &`
    //    b. Run `cargo xtask copy-plugins`
    //    c. Expected: "ERROR: TruckPilot daemon ... is running", exit code 1
    //    d. Verify: `echo $LASTEXITCODE` → 1  (1 = blocked-before-copy, 2 = copy-failed)
    //    e. Stop daemon, re-run: plugins deployed, exit 0
    //
    // 2. Lock-Guard (copy_plugins_impl level, NEW):
    //    a. Lock one DLL manually in plugins/:
    //       `$f = [System.IO.File]::Open("plugins\truckpilot_plugin_router.dll",
    //              [System.IO.FileMode]::Open, [System.IO.FileAccess]::Write,
    //              [System.IO.FileShare]::None)`
    //    b. Run `cargo xtask copy-plugins`
    //    c. Expected: "ERROR: Plugin DLL ... is locked", exit code 1
    //    d. `$f.Close()`, re-run: exit 0
    //
    // 3. Locked DLL → ERROR in deploy_plugins_to_dir (legacy path, still active):
    //    a. Start the daemon: `.\target\release\truckpilot-core.exe daemon &`
    //    b. Run `cargo build-release`
    //    c. Expected: non-zero exit code (daemon-guard catches it at 1, not 2)
    //    d. Verify: `echo $LASTEXITCODE` → 1
    //
    // 4. After stopping daemon → success:
    //    a. Kill the daemon
    //    b. Run `cargo xtask copy-plugins`
    //    c. Expected: all plugins deployed, exit 0
    //
    // 5. Partial deploy (some DLLs locked, some not):
    //    a. Lock one DLL manually (see case 2 above)
    //    b. Daemon NOT running (so daemon-guard passes)
    //    c. Run `cargo xtask copy-plugins`
    //    d. Expected: lock-guard fires, exit 1, names the locked DLL
}
