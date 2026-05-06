//! Integration tests for the multi-archive mod-support layer.
//!
//! These tests use the in-memory `InMemorySource` and the public API of
//! `truckpilot::ets2_parser` exclusively — no `.scs` writing is required.

use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

use truckpilot::ets2_parser::binary_parser::SectorData;
use truckpilot::ets2_parser::{
    discover_mods, expand_mod_list, merge_sectors, InMemorySource, ModDescriptor, ModLoadOrder,
    MultiArchiveReader,
};
use truckpilot::json_export::{MapNode, MapPrefab, MapRoad};

fn unique_tmp(prefix: &str) -> PathBuf {
    let nonce = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    let mut p = std::env::temp_dir();
    p.push(format!(
        "tp_modtest_{prefix}_{}_{}",
        std::process::id(),
        nonce
    ));
    std::fs::create_dir_all(&p).unwrap();
    p
}

fn fake_descriptor(name: &str, load_order: u32) -> ModDescriptor {
    ModDescriptor {
        name: name.into(),
        file_path: PathBuf::from(format!("/fake/{name}")),
        load_order,
        is_map_mod: true,
        is_enabled: true,
    }
}

fn node(uid: u64, x: f64, z: f64) -> MapNode {
    MapNode { uid, x, y: 0.0, z }
}

fn road(uid: &str, nodes: Vec<u64>) -> MapRoad {
    MapRoad {
        uid: uid.into(),
        name: String::new(),
        look_token: String::new(),
        nodes,
        speed_limit: None,
        lane_count_forward: 1,
        lane_count_backward: 1,
    }
}

fn prefab(uid: &str, nodes: Vec<u64>) -> MapPrefab {
    MapPrefab {
        uid: uid.into(),
        nodes,
    }
}

// ---------------------------------------------------------------------------
// 1. test_mod_duplicate_nodes
// ---------------------------------------------------------------------------
#[test]
fn test_mod_duplicate_nodes() {
    let s1 = SectorData {
        nodes: vec![node(1, 0.0, 0.0), node(2, 1.0, 1.0)],
        roads: vec![],
        prefabs: vec![],
    };
    let s2 = SectorData {
        // Node 2 is shared.
        nodes: vec![node(2, 99.0, 99.0), node(3, 2.0, 2.0)],
        roads: vec![],
        prefabs: vec![],
    };
    let merged = merge_sectors(vec![s1, s2]);
    assert_eq!(merged.nodes.len(), 3, "shared node must appear only once");
    let uids: Vec<u64> = merged.nodes.iter().map(|n| n.uid).collect();
    assert_eq!(uids, vec![1, 2, 3]);
    // First-occurrence wins: node 2's coordinates come from s1.
    let node2 = merged.nodes.iter().find(|n| n.uid == 2).unwrap();
    assert_eq!(node2.x, 1.0);
}

// ---------------------------------------------------------------------------
// 2. test_mod_sector_override
// ---------------------------------------------------------------------------
#[test]
fn test_mod_sector_override() {
    let mut reader = MultiArchiveReader::new();

    // Base archive carries the original sector with one road.
    let base_sector_bytes = b"base-sector-payload".to_vec();
    reader.push(
        fake_descriptor("base.scs", 0),
        Box::new(InMemorySource::new(vec![(
            "map/europe/sec+0001+0001.base".into(),
            base_sector_bytes.clone(),
        )])),
    );

    // Mod overrides the same sector with a different payload (extra road).
    let mod_sector_bytes = b"mod-sector-payload-with-extra-road".to_vec();
    reader.push(
        fake_descriptor("mymod.scs", 10),
        Box::new(InMemorySource::new(vec![(
            "map/europe/sec+0001+0001.base".into(),
            mod_sector_bytes.clone(),
        )])),
    );

    let resolved = reader
        .get_sector("map/europe/sec+0001+0001.base")
        .expect("sector must resolve");
    assert_eq!(resolved, mod_sector_bytes, "mod must override base");
    assert_ne!(resolved, base_sector_bytes);

    // The base archive still owns the path too — index lookup confirms.
    let providers = reader.sector_providers("map/europe/sec+0001+0001.base");
    assert_eq!(providers, vec![0, 1]);
}

// ---------------------------------------------------------------------------
// 3. test_multi_mod_loading
// ---------------------------------------------------------------------------
#[test]
fn test_multi_mod_loading() {
    let mut reader = MultiArchiveReader::new();

    reader.push(
        fake_descriptor("base.scs", 0),
        Box::new(InMemorySource::new(vec![
            ("map/europe/sec+0000+0000.base".into(), b"base-0".to_vec()),
            ("map/europe/sec+0001+0000.base".into(), b"base-1".to_vec()),
        ])),
    );
    reader.push(
        fake_descriptor("promods.scs", 10),
        Box::new(InMemorySource::new(vec![
            (
                "map/europe/sec+0010+0010.base".into(),
                b"promods-a".to_vec(),
            ),
            (
                "map/europe/sec+0001+0000.base".into(),
                b"promods-b".to_vec(),
            ),
        ])),
    );
    reader.push(
        fake_descriptor("rusmap.scs", 20),
        Box::new(InMemorySource::new(vec![(
            "map/europe/sec+0020+0020.base".into(),
            b"rusmap".to_vec(),
        )])),
    );

    assert_eq!(reader.archive_count(), 3);
    assert_eq!(reader.unique_sector_count(), 4);

    // Highest priority on the contested sector is promods (load_order 10).
    assert_eq!(
        reader.get_sector("map/europe/sec+0001+0000.base"),
        Some(b"promods-b".to_vec())
    );
    // Sectors unique to each archive resolve to their owner.
    assert_eq!(
        reader.get_sector("map/europe/sec+0000+0000.base"),
        Some(b"base-0".to_vec())
    );
    assert_eq!(
        reader.get_sector("map/europe/sec+0010+0010.base"),
        Some(b"promods-a".to_vec())
    );
    assert_eq!(
        reader.get_sector("map/europe/sec+0020+0020.base"),
        Some(b"rusmap".to_vec())
    );
}

// ---------------------------------------------------------------------------
// 4. test_connector_priority
// ---------------------------------------------------------------------------
#[test]
fn test_connector_priority() {
    let mut reader = MultiArchiveReader::new();

    reader.push(
        fake_descriptor("base.scs", 0),
        Box::new(InMemorySource::new(vec![("sec".into(), b"base".to_vec())])),
    );
    reader.push(
        fake_descriptor("promods.scs", 10),
        Box::new(InMemorySource::new(vec![(
            "sec".into(),
            b"promods".to_vec(),
        )])),
    );
    reader.push(
        fake_descriptor("rusmap.scs", 20),
        Box::new(InMemorySource::new(vec![(
            "sec".into(),
            b"rusmap".to_vec(),
        )])),
    );
    reader.push(
        fake_descriptor("connector.scs", 999),
        Box::new(InMemorySource::new(vec![(
            "sec".into(),
            b"connector".to_vec(),
        )])),
    );

    assert_eq!(reader.get_sector("sec"), Some(b"connector".to_vec()));
}

// ---------------------------------------------------------------------------
// 5. test_mod_discovery
// ---------------------------------------------------------------------------
#[test]
fn test_mod_discovery() {
    let dir = unique_tmp("discovery");
    for name in ["zulu.scs", "alpha.scs", "mike.scs", "ignored.txt"] {
        std::fs::write(dir.join(name), b"x").unwrap();
    }
    let mods = expand_mod_list(&dir).unwrap();
    let names: Vec<&str> = mods.iter().map(|m| m.name.as_str()).collect();
    assert_eq!(names, vec!["alpha.scs", "mike.scs", "zulu.scs"]);
    for (i, m) in mods.iter().enumerate() {
        assert_eq!(m.load_order, i as u32);
        assert!(m.is_enabled);
    }
    // discover_mods is the infallible variant.
    let same = discover_mods(&dir);
    assert_eq!(same.len(), mods.len());
    std::fs::remove_dir_all(&dir).ok();
}

// ---------------------------------------------------------------------------
// 6. test_mod_order_json
// ---------------------------------------------------------------------------
#[test]
fn test_mod_order_json() {
    let dir = unique_tmp("json_order");
    std::fs::write(dir.join("a.scs"), b"x").unwrap();
    std::fs::write(dir.join("b.scs"), b"x").unwrap();
    std::fs::write(dir.join("c.scs"), b"x").unwrap();

    let json = r#"{
        "base_game_files": ["base.scs", "def.scs"],
        "mod_descriptors": [
            {"name": "C-mod", "file": "c.scs", "order": 30},
            {"name": "A-mod", "file": "a.scs", "order": 10},
            {"name": "B-mod", "file": "b.scs", "order": 20}
        ]
    }"#;
    let json_path = dir.join("mod_order.json");
    std::fs::write(&json_path, json).unwrap();

    let order = ModLoadOrder::from_json_file(
        &json_path,
        Some(&dir),
        Some(std::path::Path::new("/games/ets2")),
    )
    .unwrap();
    assert_eq!(order.descriptors.len(), 3);
    assert_eq!(order.base_game_paths.len(), 2);
    assert_eq!(
        order.base_game_paths[0],
        PathBuf::from("/games/ets2/base.scs")
    );

    // enabled_sorted gives ascending load_order.
    let sorted = order.enabled_sorted();
    let names: Vec<&str> = sorted.iter().map(|d| d.name.as_str()).collect();
    assert_eq!(names, vec!["A-mod", "B-mod", "C-mod"]);

    std::fs::remove_dir_all(&dir).ok();
}

// ---------------------------------------------------------------------------
// 7. test_combined_map_parsing
// ---------------------------------------------------------------------------
#[test]
fn test_combined_map_parsing() {
    // Build "base" parsed sector data simulating the vanilla map.
    let base = SectorData {
        nodes: vec![node(1, 0.0, 0.0), node(2, 1.0, 0.0)],
        roads: vec![road("r-base", vec![1, 2])],
        prefabs: vec![prefab("p-base", vec![1])],
    };
    let promods = SectorData {
        nodes: vec![node(2, 1.0, 0.0), node(3, 2.0, 0.0)],
        roads: vec![road("r-pm", vec![2, 3])],
        prefabs: vec![prefab("p-pm", vec![3])],
    };
    let rusmap = SectorData {
        nodes: vec![node(4, 5.0, 0.0)],
        roads: vec![road("r-ru", vec![4, 4])],
        prefabs: vec![],
    };
    let connector = SectorData {
        nodes: vec![node(5, 9.0, 0.0)],
        roads: vec![road("r-conn", vec![3, 5])],
        prefabs: vec![],
    };

    let base_only = merge_sectors(vec![base.clone()]);
    let combined = merge_sectors(vec![
        base.clone(),
        promods.clone(),
        rusmap.clone(),
        connector.clone(),
    ]);

    assert!(combined.nodes.len() > base_only.nodes.len());
    assert!(combined.roads.len() > base_only.roads.len());
    assert!(combined.prefabs.len() >= base_only.prefabs.len());

    // No duplicate UIDs.
    let mut uids: Vec<u64> = combined.nodes.iter().map(|n| n.uid).collect();
    uids.sort();
    uids.dedup();
    assert_eq!(uids.len(), combined.nodes.len());

    assert_eq!(combined.nodes.len(), 5);
}

// ---------------------------------------------------------------------------
// 8. test_idempotent_parsing
// ---------------------------------------------------------------------------
#[test]
fn test_idempotent_parsing() {
    let make = || SectorData {
        nodes: vec![node(1, 0.0, 0.0), node(2, 1.0, 0.0), node(3, 2.0, 0.0)],
        roads: vec![road("r1", vec![1, 2]), road("r2", vec![2, 3])],
        prefabs: vec![prefab("p1", vec![2])],
    };

    let a = merge_sectors(vec![make(), make()]);
    let b = merge_sectors(vec![make(), make()]);

    assert_eq!(a.nodes.len(), b.nodes.len());
    assert_eq!(a.nodes, b.nodes);
    assert_eq!(a.roads.len(), b.roads.len());
    assert_eq!(a.prefabs.len(), b.prefabs.len());
}

// ---------------------------------------------------------------------------
// Bonus: end-to-end resolve through MultiArchiveReader keeps consistency.
// ---------------------------------------------------------------------------
#[test]
fn test_resolve_then_get_sector_consistency() {
    let mut reader = MultiArchiveReader::new();
    reader.push(
        fake_descriptor("a", 0),
        Box::new(InMemorySource::new(vec![("s".into(), b"low".to_vec())])),
    );
    reader.push(
        fake_descriptor("b", 5),
        Box::new(InMemorySource::new(vec![("s".into(), b"high".to_vec())])),
    );
    let idx = reader.resolve_sector("s").unwrap();
    let desc = reader.descriptor(idx).unwrap();
    assert_eq!(desc.name, "b");
    assert_eq!(reader.get_sector("s"), Some(b"high".to_vec()));
}
