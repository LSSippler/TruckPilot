//! Smoke test against a real ETS2 base.scs archive.
//!
//! The path is taken from the `TRUCKPILOT_BASE_SCS` environment variable so
//! the test stays plattform-neutral. If the variable is unset or the file
//! does not exist, the test is skipped (it does not fail), which keeps it
//! safe to leave in CI.
//!
//! Example (PowerShell):
//! ```powershell
//! $env:TRUCKPILOT_BASE_SCS = "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2\base.scs"
//! cargo test -p truckpilot-map-parser --test real_archive_test -- --nocapture
//! ```
//!
//! What this verifies:
//!   1. Header parsing succeeds (used to fail with garbled u32 offsets).
//!   2. `entry_table` and `metadata_table` decompress without error
//!      (the original symptom we were tracking).
//!   3. The hash index is non-empty.
//!   4. At least one well-known SII path can be read back.

use std::path::PathBuf;
use std::time::Instant;

use truckpilot_map_parser::{Archive, HashFsArchive};

fn fixture_path() -> Option<PathBuf> {
    let raw = std::env::var("TRUCKPILOT_BASE_SCS").ok()?;
    let path = PathBuf::from(raw);
    if path.is_file() {
        Some(path)
    } else {
        eprintln!(
            "TRUCKPILOT_BASE_SCS points at {:?} which is not a file — skipping",
            path
        );
        None
    }
}

/// Single end-to-end test — the file is huge (the SHA-256 alone takes ~80 s
/// on a 9.9 GB base.scs), so we don't open it twice.
#[test]
fn end_to_end_real_archive() {
    // Surface tracing logs from the parser so build_index diagnostics show
    // up under `cargo test -- --nocapture`. The base subscriber emits at
    // INFO and above by default; build_index uses info!.
    let _ = tracing_subscriber::fmt()
        .with_test_writer()
        .try_init();

    let Some(path) = fixture_path() else {
        eprintln!("TRUCKPILOT_BASE_SCS not set — skipping end_to_end_real_archive");
        return;
    };

    let started = Instant::now();
    let mut archive = HashFsArchive::open(&path)
        .unwrap_or_else(|e| panic!("failed to open {path:?}: {e}"));
    println!(
        "Opened {} in {:?}: {} entries indexed",
        path.display(),
        started.elapsed(),
        archive.entry_count()
    );

    // (1) Header + table-decompression worked if we got here with a
    //     non-empty index. This is the symptom we were tracking.
    assert!(
        archive.entry_count() > 0,
        "expected a non-empty hash index — header or metadata parsing is broken"
    );

    // (2) Diagnostic: log the first eight indexed hashes so the human can
    //     spot-check them against TruckLib's hash table.
    let mut hashes = archive.list_hashes();
    hashes.sort();
    println!("first 8 hashes:");
    for h in hashes.iter().take(8) {
        println!("  0x{h:016X}");
    }

    // (3) End-to-end read: try the root directory first, fall back to a
    //     few well-known paths used by definition vs. map archives.
    let candidates = [
        "",                                  // root directory (CityHash64 = K2)
        "manifest.sii",                      // archive manifest, common
        "version.sii",                       // optional
        "automat",                           // base.scs has this as a dir
        "def",                               // both base / base_map have this
        "map/europe/sec+0000+0000.base",     // base_map.scs only
    ];

    let mut hits = 0usize;
    for p in candidates {
        if archive.contains(p) {
            match archive.read_path(p) {
                Ok(data) => {
                    println!("read {:?}: {} bytes", p, data.len());
                    hits += 1;
                }
                Err(e) => {
                    eprintln!("read {:?}: {e}", p);
                }
            }
        } else {
            eprintln!(
                "{:?} (hash 0x{:016X}) not in archive",
                p,
                truckpilot_map_parser::hashfs::scs_path_hash(archive.salt(), &p.to_lowercase())
            );
        }
    }

    assert!(
        hits > 0,
        "no candidate path resolved — index1/index2/CityHash wiring still suspect"
    );
}
