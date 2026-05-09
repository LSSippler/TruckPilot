//! Sanity tests for the binary fixtures produced by `extract_fixtures.rs`.
//!
//! These tests run end-to-end: they take a real `.base` sector extracted
//! from `base_map.scs` and feed it through `parse_sector`, verifying that
//! Phase 5.6's full Road parser walks the variable-length DataPayload
//! correctly and recovers the expected number of roads / prefabs.
//!
//! Fixtures are NOT committed to the repo — generate them locally:
//!
//! ```powershell
//! $env:ETS2_DIR = "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2"
//! cargo test --release --test extract_fixtures -- --ignored --nocapture
//! ```

use std::path::PathBuf;

use truckpilot_map_parser::sector::parse_sector;

fn fixture_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join(name)
}

/// Read a fixture file or skip the test (with a printed hint) if it is
/// absent. Returning `None` lets the caller `return` early without panicking
/// — which keeps the test suite runnable on machines that have no ETS2
/// install yet.
fn load_fixture(name: &str) -> Option<Vec<u8>> {
    let path = fixture_path(name);
    if !path.exists() {
        eprintln!(
            "fixture {} missing — run `cargo test --test extract_fixtures -- --ignored` first",
            path.display()
        );
        return None;
    }
    Some(std::fs::read(&path).unwrap_or_else(|e| panic!("read {}: {e}", path.display())))
}

#[test]
fn parse_simple_road_sector() {
    let Some(data) = load_fixture("simple_road_sector.bin") else {
        return;
    };
    let sector = parse_sector(&data).expect("simple_road_sector parses");
    assert_eq!(
        sector.roads.len(),
        1,
        "simple_road_sector should contain exactly one road, got {}",
        sector.roads.len()
    );
    assert_eq!(
        sector.prefabs.len(),
        0,
        "simple_road_sector should contain no prefabs, got {}",
        sector.prefabs.len()
    );
}

#[test]
fn parse_multi_road_sector() {
    let Some(data) = load_fixture("multi_road_sector.bin") else {
        return;
    };
    let sector = parse_sector(&data).expect("multi_road_sector parses");
    assert!(
        (2..=3).contains(&sector.roads.len()),
        "multi_road_sector should have 2..=3 roads, got {}",
        sector.roads.len()
    );
    assert_eq!(
        sector.prefabs.len(),
        0,
        "multi_road_sector should have no prefabs, got {}",
        sector.prefabs.len()
    );
}

#[test]
fn parse_road_with_prefab_sector() {
    let Some(data) = load_fixture("road_with_prefab_sector.bin") else {
        return;
    };
    let sector = parse_sector(&data).expect("road_with_prefab_sector parses");
    assert!(
        !sector.roads.is_empty(),
        "road_with_prefab_sector should contain at least one road"
    );
    assert!(
        !sector.prefabs.is_empty(),
        "road_with_prefab_sector should contain at least one prefab"
    );
}
