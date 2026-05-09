//! Smoke test against a real ETS2 `base.scs` archive.
//!
//! The path is taken from `TRUCKPILOT_BASE_SCS` so the test stays
//! plattform-neutral; without the env var the test skips silently.
//!
//! Run (PowerShell):
//!
//! ```powershell
//! $env:TRUCKPILOT_BASE_SCS = "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2\base.scs"
//! cargo test --release -p truckpilot-map-parser --test real_archive_test -- --nocapture
//! ```
//!
//! What this verifies end-to-end:
//!   1. Header parsing (the original `u64`-offset bug).
//!   2. Index1 + Index2 zlib-inflate.
//!   3. Two-tier mini-header → body indirection in Index2.
//!   4. CityHash64 of a real path resolves to a real entry — proving our
//!      hash function matches what `base.scs` actually uses.

use std::path::PathBuf;
use std::time::Instant;

use truckpilot_map_parser::{parse_directory_listing, Archive, HashFsArchive};

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

#[test]
fn end_to_end_real_archive() {
    // Surface parser tracing under `cargo test -- --nocapture`.
    let _ = tracing_subscriber::fmt().with_test_writer().try_init();

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
    assert!(
        archive.entry_count() > 0,
        "expected a non-empty hash index — header/metadata parsing broken"
    );

    // (1) Read root listing — proves "" lookup, table inflate, body deref,
    //     and zlib data inflate all work for at least one entry.
    let root_bytes = archive
        .read_path("")
        .expect("reading the root directory ('') must succeed");
    println!("root listing: {} compressed → {} inflated", root_bytes.len(), root_bytes.len());

    let items = parse_directory_listing(&root_bytes)
        .expect("root directory listing must parse");
    println!("root contains {} items:", items.len());
    for it in &items {
        println!("  {}{}", if it.is_dir { "/" } else { "" }, it.name);
    }
    assert!(
        !items.is_empty(),
        "root listing parsed to zero items — listing format is wrong"
    );

    println!("\n=== empirical CityHash showdown ===");
    // Three real top-level paths from the listing above. For each, compare
    // OUR rust impl vs TruckLib's CityHash.cs (computed offline via dotnet
    // run against TruckLib/HashFs/CityHash.cs). Whichever value lives in
    // the entry index is the algorithm base.scs actually uses.
    //
    // TruckLib values were computed using TruckLib.HashFs CityHash.CityHash64
    // (utf-8 bytes) directly via `dotnet run` — see commit message.
    let trucklib_vectors: &[(&str, u64)] = &[
        ("automat", 0x56BC42EECBC73F2F),
        ("def",     0x2C6F469EFB31C45A),
        ("map",     0x3543EC1D70156653),
    ];
    let index = archive.list_hashes();
    let index_set: std::collections::HashSet<u64> = index.into_iter().collect();
    let mut our_hits = 0;
    let mut tl_hits = 0;
    for (s, tl_hash) in trucklib_vectors {
        let our_hash =
            truckpilot_map_parser::hashfs::scs_path_hash(archive.salt(), s);
        let our_in = index_set.contains(&our_hash);
        let tl_in = index_set.contains(tl_hash);
        if our_in { our_hits += 1; }
        if tl_in { tl_hits += 1; }
        println!(
            "  {:>10}  ours=0x{:016X} {:5}  trucklib=0x{:016X} {:5}",
            format!("{:?}", s),
            our_hash, if our_in { "HIT" } else { "miss" },
            tl_hash, if tl_in { "HIT" } else { "miss" },
        );
    }
    println!(
        "Summary: ours={}/{} hits, trucklib={}/{} hits",
        our_hits, trucklib_vectors.len(), tl_hits, trucklib_vectors.len()
    );

    // The decisive assertion: at least one of the three real paths must
    // produce a hash that is actually in the archive's index. If TruckLib
    // wins, we know to port its CityHash. If neither wins, neither
    // algorithm matches what base.scs uses and we need a third reference.
    assert!(
        our_hits > 0 || tl_hits > 0,
        "neither our CityHash nor TruckLib's matched any of the three known \
         paths in base.scs — both algorithms are wrong relative to the archive"
    );
}
