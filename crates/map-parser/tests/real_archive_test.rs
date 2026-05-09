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

    // (2) Dynamic lookup: pick the first subdirectory from the root
    //     listing and verify the full chain (CityHash → index → inflate
    //     → directory listing parse) works for an arbitrary path. This
    //     stays robust across ETS2 versions: whichever dir comes first
    //     in the archive's own root listing is the test target.
    let first_subdir = items
        .iter()
        .find(|i| i.is_dir)
        .map(|i| i.name.clone())
        .expect("root listing has no subdirectories — unexpected for base.scs");
    println!("\n=== dynamic subdirectory lookup: {:?} ===", first_subdir);
    assert!(
        archive.contains(&first_subdir),
        "subdirectory {:?} from root listing is not in the index — \
         CityHash + lookup chain still broken",
        first_subdir
    );
    let sub_bytes = archive
        .read_path(&first_subdir)
        .unwrap_or_else(|e| panic!("read subdir {:?}: {e}", first_subdir));
    let sub_items = parse_directory_listing(&sub_bytes)
        .expect("subdirectory listing must parse");
    println!(
        "  /{} listing: {} bytes → {} items",
        first_subdir,
        sub_bytes.len(),
        sub_items.len()
    );
    for it in sub_items.iter().take(5) {
        println!("    {}{}", if it.is_dir { "/" } else { "" }, it.name);
    }
    if sub_items.len() > 5 {
        println!("    ... ({} more)", sub_items.len() - 5);
    }
    assert!(
        !sub_items.is_empty(),
        "subdirectory {:?} parsed to zero items — listing format wrong \
         at depth > 0?",
        first_subdir
    );

    // (3) Dynamic file read: walk one level deeper to find a regular file
    //     and read it. Bounded so we don't accidentally scan the whole
    //     archive.
    if let Some(file_path) = first_leaf_file(&mut archive, &first_subdir, &sub_items) {
        let bytes = archive
            .read_path(&file_path)
            .unwrap_or_else(|e| panic!("read leaf file {:?}: {e}", file_path));
        println!("\nleaf file {:?}: {} bytes", file_path, bytes.len());
        assert!(!bytes.is_empty(), "leaf file inflated to zero bytes");
    } else {
        eprintln!(
            "\nno leaf file found within depth budget — diagnostic only, \
             not a failure"
        );
    }

    // (4) Regression sentinel: even though the test is now version-agnostic
    //     for its primary path, keep an empirical anchor so future CityHash
    //     regressions surface immediately. "automat" is present in stock
    //     base.scs across every ETS2 1.x version we know of, with hash
    //     0x56BC42EECBC73F2F.
    const AUTOMAT_HASH: u64 = 0x56BC42EECBC73F2F;
    let our_automat =
        truckpilot_map_parser::hashfs::scs_path_hash(archive.salt(), "automat");
    assert_eq!(
        our_automat, AUTOMAT_HASH,
        "regression: cityhash64(\"automat\") drifted away from TruckLib"
    );
    assert!(
        archive.list_hashes().contains(&AUTOMAT_HASH),
        "regression: \"automat\" no longer found in base.scs — either \
         the archive structure changed or our hash silently broke"
    );
}

/// Walk the directory tree starting at `subdir` and return the path of
/// the first regular (non-directory) item found. Bounded depth so we
/// never iterate the whole archive.
fn first_leaf_file(
    archive: &mut HashFsArchive,
    subdir: &str,
    items: &[truckpilot_map_parser::DirItem],
) -> Option<String> {
    let mut stack: Vec<(String, Vec<truckpilot_map_parser::DirItem>)> =
        vec![(subdir.to_string(), items.to_vec())];
    let mut visited = 0usize;

    while let Some((dir, listing)) = stack.pop() {
        visited += 1;
        if visited > 32 {
            return None;
        }
        if let Some(leaf) = listing.iter().find(|i| !i.is_dir) {
            return Some(format!("{}/{}", dir, leaf.name));
        }
        for item in listing.iter().filter(|i| i.is_dir) {
            let child = format!("{}/{}", dir, item.name);
            if let Ok(bytes) = archive.read_path(&child) {
                if let Ok(child_items) = parse_directory_listing(&bytes) {
                    stack.push((child, child_items));
                }
            }
        }
    }
    None
}
