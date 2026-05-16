//! Extracts representative `.base` sectors from a real ETS2 `base_map.scs`
//! into `tests/fixtures/` so that the unit tests in `sector_fixtures.rs`
//! have known-shape input.
//!
//! Usage:
//!
//! ```powershell
//! $env:ETS2_DIR = "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2"
//! cargo test --release --test extract_fixtures -- --ignored --nocapture
//! ```
//!
//! The test is `#[ignore]`d because it requires a real game install. It
//! produces three fixtures:
//!
//! * `simple_road_sector.bin` — sector with exactly 1 Road and 0 Prefabs
//! * `multi_road_sector.bin`  — sector with 2-3 Roads and 0 Prefabs
//! * `road_with_prefab_sector.bin` — sector with at least 1 Road and 1 Prefab
//!
//! Categorisation uses the *currently parseable* subset of items. Today
//! the Road parser cannot read the variable-length DataPayload trailer
//! that follows every Road, so most sectors fail to parse — we pick from
//! the ones that do, taking the smallest in each category.

use std::path::PathBuf;

use truckpilot_map_parser::sector::parse_sector;
use truckpilot_map_parser::{Archive, HashFsArchive};

#[test]
#[ignore = "needs ETS2_DIR pointing at a real ETS2 install (~9 GB base_map.scs)"]
fn extract_road_sector_fixtures() {
    let ets2_dir: PathBuf = std::env::var("ETS2_DIR")
        .expect("set ETS2_DIR=<path-to-ETS2-install> before running this test")
        .into();
    let base_map_path = ets2_dir.join("base_map.scs");
    assert!(
        base_map_path.exists(),
        "base_map.scs not found under {} — wrong ETS2_DIR?",
        ets2_dir.display()
    );

    let mut archive = HashFsArchive::open(&base_map_path).expect("open base_map.scs");

    // HashFS does not list paths; we have to probe known sector names.
    let probed = archive.probe_sector_paths();
    let base_paths: Vec<String> = probed
        .into_iter()
        .filter(|p| p.ends_with(".base"))
        .collect();
    println!("probed {} .base sector paths", base_paths.len());

    let mut simple_road: Option<Candidate> = None;
    let mut multi_road: Option<Candidate> = None;
    let mut road_with_prefab: Option<Candidate> = None;

    for path in &base_paths {
        let Ok(data) = archive.read_path(path) else {
            continue;
        };
        let Ok(sector) = parse_sector(&data) else {
            continue;
        };

        let roads = sector.roads.len();
        let prefabs = sector.prefabs.len();
        let category = if prefabs == 0 && roads == 1 {
            Category::SimpleRoad
        } else if prefabs == 0 && (2..=3).contains(&roads) {
            Category::MultiRoad
        } else if prefabs >= 1 && roads >= 1 {
            Category::RoadWithPrefab
        } else {
            continue;
        };

        let candidate = Candidate {
            path: path.clone(),
            data,
            roads,
            prefabs,
        };
        let slot = match category {
            Category::SimpleRoad => &mut simple_road,
            Category::MultiRoad => &mut multi_road,
            Category::RoadWithPrefab => &mut road_with_prefab,
        };
        if slot
            .as_ref()
            .is_none_or(|c| candidate.data.len() < c.data.len())
        {
            println!(
                "[{:?}] candidate from {}: {} bytes, {} roads, {} prefabs",
                category,
                candidate.path,
                candidate.data.len(),
                candidate.roads,
                candidate.prefabs
            );
            *slot = Some(candidate);
        }
    }

    let fixtures_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures");
    std::fs::create_dir_all(&fixtures_dir).expect("create fixtures dir");

    let mut wrote = 0usize;
    for (slot, name) in [
        (simple_road.as_ref(), "simple_road_sector.bin"),
        (multi_road.as_ref(), "multi_road_sector.bin"),
        (road_with_prefab.as_ref(), "road_with_prefab_sector.bin"),
    ] {
        match slot {
            Some(c) => {
                let out = fixtures_dir.join(name);
                std::fs::write(&out, &c.data)
                    .unwrap_or_else(|e| panic!("write {}: {e}", out.display()));
                println!(
                    "wrote {} ({} bytes, source: {})",
                    out.display(),
                    c.data.len(),
                    c.path
                );
                wrote += 1;
            }
            None => {
                eprintln!("WARN: no candidate found for {name} — skip writing this fixture");
            }
        }
    }

    assert!(
        wrote > 0,
        "no fixtures could be extracted — the parser produced 0 candidates of any category"
    );
}

struct Candidate {
    path: String,
    data: Vec<u8>,
    roads: usize,
    prefabs: usize,
}

#[derive(Debug)]
enum Category {
    SimpleRoad,
    MultiRoad,
    RoadWithPrefab,
}
