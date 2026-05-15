use std::fs;
use std::path::{Path, PathBuf};

fn main() {
    let mut args = std::env::args().skip(1);
    match args.next().as_deref() {
        Some("copy-plugins") => copy_plugins(args),
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
    eprintln!("  copy-plugins [--debug]  Copy plugin DLLs from target/ to plugins/");
    eprintln!();
    eprintln!("Flags:");
    eprintln!("  --debug   Copy from target/debug/ instead of target/release/");
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
    let root = workspace_root();
    let src = root.join("target").join(profile);
    let dst = root.join("plugins");

    if !src.exists() {
        eprintln!("Source directory does not exist: {}", src.display());
        eprintln!("Run `cargo build --workspace --{}` first.", profile);
        std::process::exit(1);
    }

    fs::create_dir_all(&dst).unwrap_or_else(|e| {
        eprintln!("Cannot create plugins/: {e}");
        std::process::exit(1);
    });

    // Remove stale plugin files from dst to keep it in sync.
    let stale: Vec<_> = fs::read_dir(&dst)
        .unwrap_or_else(|e| {
            eprintln!("Cannot read plugins/: {e}");
            std::process::exit(1);
        })
        .flatten()
        .filter(|e| is_plugin_file(e.path().as_path()))
        .collect();

    for entry in &stale {
        let path = entry.path();
        fs::remove_file(&path).unwrap_or_else(|e| {
            eprintln!("Cannot remove {}: {e}", path.display());
            std::process::exit(1);
        });
        println!("  removed  {}", path.file_name().unwrap().to_string_lossy());
    }

    // Copy matching plugin files from target/<profile>/.
    let mut copied = 0usize;
    let entries = fs::read_dir(&src).unwrap_or_else(|e| {
        eprintln!("Cannot read {}: {e}", src.display());
        std::process::exit(1);
    });

    for entry in entries.flatten() {
        let path = entry.path();
        if !is_plugin_file(path.as_path()) {
            continue;
        }
        let file_name = path.file_name().unwrap();
        // Only copy files whose stem starts with "truckpilot_plugin_".
        let stem = Path::new(file_name).file_stem().unwrap_or_default();
        let stem_str = stem.to_string_lossy();
        if !stem_str.starts_with("truckpilot_plugin_") {
            continue;
        }
        let dst_path = dst.join(file_name);
        fs::copy(&path, &dst_path).unwrap_or_else(|e| {
            eprintln!(
                "Cannot copy {} -> {}: {e}",
                path.display(),
                dst_path.display()
            );
            std::process::exit(1);
        });
        println!("  copied   {}", file_name.to_string_lossy());
        copied += 1;
    }

    println!();
    println!("{} plugin(s) deployed to plugins/", copied);
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
