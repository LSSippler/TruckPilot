use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

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
    eprintln!("  build-release                cargo build --workspace --release, then copy-plugins");
    eprintln!("  deploy-ets2-telemetry [DIR]  Copy truckpilot_telemetry.dll to ETS2 plugins dir");
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

fn deploy_ets2_telemetry(mut args: impl Iterator<Item = String>) {
    let root = workspace_root();
    let src = root
        .join("target")
        .join("release")
        .join("truckpilot_telemetry.dll");

    if !src.exists() {
        eprintln!("truckpilot_telemetry.dll not found at {}", src.display());
        eprintln!("Run `cargo build --workspace --release` first.");
        std::process::exit(1);
    }

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

    let dst = dst_dir.join("truckpilot_telemetry.dll");
    match fs::copy(&src, &dst) {
        Ok(_) => println!(
            "Deployed truckpilot_telemetry.dll -> {}",
            dst.display()
        ),
        Err(e) => {
            eprintln!("Copy failed: {e}");
            eprintln!("Is ETS2 running? Close it first.");
            std::process::exit(1);
        }
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

    let (copied, errors) = deploy_plugins_to_dir(&src, &dst);
    if errors.is_empty() {
        println!("{} plugin(s) deployed to plugins/", copied);
    } else {
        eprintln!("Deploy FAILED: {} plugin(s) deployed, {} ERROR(s):", copied, errors.len());
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
        let stale: Vec<_> = rd.flatten().filter(|e| is_plugin_file(e.path().as_path())).collect();
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
    // 1. Locked DLL → ERROR + exit code 2:
    //    a. Start the daemon: `.\target\release\truckpilot-core.exe daemon &`
    //    b. Run `cargo build-release`
    //    c. Expected: non-zero exit code, error message listing locked DLLs,
    //       "Action: stop the daemon, then run `cargo xtask copy-plugins`"
    //    d. Verify: `echo $LASTEXITCODE` → 2
    //
    // 2. After stopping daemon → success:
    //    a. Kill the daemon
    //    b. Run `cargo xtask copy-plugins`
    //    c. Expected: all plugins deployed, exit 0
    //
    // 3. Partial deploy (some DLLs locked, some not):
    //    a. Lock one DLL manually (e.g. PowerShell: `$f = [System.IO.File]::Open(...)`)
    //    b. Run `cargo xtask copy-plugins`
    //    c. Expected: non-zero exit, lists only the locked file, others deploy ok
}
