//! ETS2 map data parser module.
//!
//! Provides the entry points for parsing ETS2 game data from SCS archives
//! or from pre-exported text files. The output is `MapData` ready for
//! graph construction and routing.

use std::collections::HashSet;
use std::fs;
use std::path::{Path, PathBuf};

use miniz_oxide::inflate;

use crate::ets2_parser::binary_parser::SectorData;
use crate::json_export::MapData;

pub mod archive;
pub mod binary_parser;
pub mod error;
pub mod map_parser;
pub mod mod_descriptor;
pub mod multi_archive_reader;
pub mod scs_reader;
pub mod sii_parser;
pub mod zip_reader;

pub use mod_descriptor::{
    discover_mods, expand_mod_list, ModDescriptor, ModLoadOrder, ModOrderEntry, ModOrderFile,
};
pub use multi_archive_reader::{
    open_mod_directory, ArchiveSource, InMemorySource, MultiArchiveReader, SectorSource,
};

/// Read the first 4 magic bytes of a file.
fn read_magic_bytes(path: &Path) -> Result<[u8; 4], String> {
    use std::io::Read;
    let mut f = fs::File::open(path).map_err(|e| format!("cannot open {}: {e}", path.display()))?;
    let mut buf = [0u8; 4];
    f.read_exact(&mut buf)
        .map_err(|e| format!("cannot read magic from {}: {e}", path.display()))?;
    Ok(buf)
}

/// Parse all `.base` sector files from a ZIP archive (e.g. `base_map.scs` as ZIP).
fn parse_zip_map_archive(path: &Path) -> Result<MapData, String> {
    let mut archive = archive::Archive::open(path).map_err(|e| format!("{e}"))?;
    let sector_paths = archive::ArchiveReader::find_files_starting_with(&archive, "map/");
    let base_paths: Vec<String> = sector_paths
        .into_iter()
        .filter(|p| p.ends_with(".base"))
        .collect();

    eprintln!("  {} .base sectors in ZIP base_map.scs", base_paths.len());

    let mut map = MapData {
        nodes: Vec::new(),
        roads: Vec::new(),
        prefabs: Vec::new(),
    };

    for path_str in &base_paths {
        let bytes = match archive::ArchiveReader::read_file(&mut archive, path_str) {
            Ok(b) => b,
            Err(_) => continue,
        };
        let bytes = try_decompress_zlib(bytes);
        if let Ok(s) = binary_parser::parse_binary_sector(&bytes) {
            map.nodes.extend(s.nodes);
            map.roads.extend(s.roads);
            map.prefabs.extend(s.prefabs);
        }
    }

    dedup_map(&mut map);
    Ok(map)
}

fn dedup_map(map: &mut MapData) {
    map.nodes.sort_by_key(|n| n.uid);
    map.nodes.dedup_by_key(|n| n.uid);
    map.roads.sort_by(|a, b| a.uid.cmp(&b.uid));
    map.roads.dedup_by(|a, b| a.uid == b.uid);
    map.prefabs.sort_by(|a, b| a.uid.cmp(&b.uid));
    map.prefabs.dedup_by(|a, b| a.uid == b.uid);
}

fn collect_base_sector_files(dir: &Path, files: &mut Vec<PathBuf>) -> Result<(), String> {
    for entry in fs::read_dir(dir).map_err(|e| format!("cannot read {}: {}", dir.display(), e))? {
        let entry = entry
            .map_err(|e| format!("cannot read directory entry in {}: {}", dir.display(), e))?;
        let path = entry.path();
        let file_type = entry
            .file_type()
            .map_err(|e| format!("cannot read file type in {}: {}", dir.display(), e))?;
        if file_type.is_symlink() {
            continue;
        }

        if file_type.is_dir() {
            collect_base_sector_files(&path, files)?;
        } else if path
            .extension()
            .and_then(|ext| ext.to_str())
            .is_some_and(|ext| ext.eq_ignore_ascii_case("base"))
        {
            files.push(path);
        }
    }
    Ok(())
}

fn try_decompress_zlib(data: Vec<u8>) -> Vec<u8> {
    if data.len() < 2 || (data[0] & 0x0F) != 0x08 {
        return data;
    }

    inflate::decompress_to_vec_zlib(&data).unwrap_or(data)
}

/// Parse extracted HashFS sectors from a directory containing `*.base` files.
pub fn parse_hashfs_sectors_dir(dir: &Path) -> Result<MapData, String> {
    if !dir.exists() {
        return Err(format!(
            "sector directory does not exist: {}",
            dir.display()
        ));
    }
    if !dir.is_dir() {
        return Err(format!("sector path is not a directory: {}", dir.display()));
    }

    let mut sector_files = Vec::new();
    collect_base_sector_files(dir, &mut sector_files)?;
    if sector_files.is_empty() {
        return Err(format!("no .base sector files found in {}", dir.display()));
    }

    let mut map = MapData {
        nodes: Vec::new(),
        roads: Vec::new(),
        prefabs: Vec::new(),
    };

    for path in &sector_files {
        let data = fs::read(path).map_err(|e| format!("cannot read {}: {}", path.display(), e))?;
        let data = try_decompress_zlib(data);
        if data.len() < 8 {
            eprintln!(
                "warning: skipping {}: data too short for sector",
                path.display()
            );
            continue;
        }

        match binary_parser::parse_binary_sector(&data) {
            Ok(sector) => {
                map.nodes.extend(sector.nodes);
                map.roads.extend(sector.roads);
                map.prefabs.extend(sector.prefabs);
            }
            Err(e) => {
                eprintln!("warning: skipping {}: {}", path.display(), e);
            }
        }
    }

    dedup_map(&mut map);

    let node_set: HashSet<u64> = map.nodes.iter().map(|n| n.uid).collect();
    let before_roads = map.roads.len();
    map.roads
        .retain(|r| r.nodes.len() >= 2 && r.nodes.iter().all(|n| node_set.contains(n)));

    let before_prefabs = map.prefabs.len();
    map.prefabs
        .retain(|p| p.nodes.is_empty() || p.nodes.iter().all(|n| node_set.contains(n)));

    let removed_roads = before_roads.saturating_sub(map.roads.len());
    let removed_prefabs = before_prefabs.saturating_sub(map.prefabs.len());
    if removed_roads > 0 || removed_prefabs > 0 {
        eprintln!(
            "warning: filtered {} roads and {} prefabs with dangling node references",
            removed_roads, removed_prefabs
        );
    }

    if map.nodes.is_empty() {
        return Err(format!("no map data found in {}", dir.display()));
    }

    Ok(map)
}

/// Parse ETS2 map data from the game installation directory.
///
/// Opens `base.scs` (and optionally `base_map.scs` and `def.scs`)
/// to extract road network nodes, roads, and prefabs.
///
/// Tries multiple strategies:
/// 0. If `base_map.scs` is a ZIP archive, read all `.base` sectors directly.
/// 1. Look for text-format sector files in `base_map.scs` using known sector paths.
/// 2. Brute-force: decode every entry in the archive as text and attempt parsing.
/// 3. Look for SII definition data in `base.scs`.
pub fn parse_ets2_map(game_path: &Path) -> Result<MapData, String> {
    let base_scs = game_path.join("base.scs");
    let base_map_scs = game_path.join("base_map.scs");
    let def_scs = game_path.join("def.scs");

    if !base_scs.exists() && !base_map_scs.exists() {
        return Err(format!(
            "Neither base.scs nor base_map.scs found in {}",
            game_path.display()
        ));
    }

    // Strategy 0: If base_map.scs is a ZIP archive, read all .base sectors directly.
    // This handles cases where the game ships base_map.scs as a ZIP (older versions
    // or modded setups) and bypasses the encrypted HashFS v2 reader.
    if base_map_scs.exists() {
        if let Ok(magic) = read_magic_bytes(&base_map_scs) {
            if magic == [0x50, 0x4B, 0x03, 0x04] {
                eprintln!("base_map.scs is a ZIP archive — reading sectors directly...");
                if let Ok(map) = parse_zip_map_archive(&base_map_scs) {
                    if !map.nodes.is_empty() {
                        return Ok(map);
                    }
                }
            }
        }
    }

    let mut map = MapData {
        nodes: Vec::new(),
        roads: Vec::new(),
        prefabs: Vec::new(),
    };

    /// Print a progress bar to stderr: `[====>     ] 42% (123/456)`
    fn progress(prefix: &str, done: usize, total: usize) {
        let pct = done
            .checked_mul(100)
            .and_then(|v| v.checked_div(total))
            .unwrap_or(100);
        let bar_width = 40;
        let filled = done * bar_width / total.max(1);
        let bar: String = (0..bar_width)
            .map(|i| {
                if i < filled {
                    '='
                } else if i == filled {
                    '>'
                } else {
                    ' '
                }
            })
            .collect();
        eprint!("\r{prefix} [{bar}] {pct:>3}% ({done}/{total})");
    }

    // Strategy 1: Try base_map.scs with known sector paths.
    if base_map_scs.exists() {
        eprintln!("Opening base_map.scs...");
        if let Ok(mut archive) = scs_reader::ScsArchive::open(&base_map_scs) {
            let num = archive.num_entries();
            let hashes_count = archive.entry_hashes().len();
            eprintln!("  {} entries in directory, {} with data", num, hashes_count);

            let sector_paths = scs_reader::list_map_sector_paths();
            let total_paths = sector_paths.len();
            eprintln!("Scanning {} sector paths...", total_paths);

            for (idx, name) in sector_paths.iter().enumerate() {
                if idx % 1000 == 0 || idx == total_paths - 1 {
                    progress("  Sector paths", idx + 1, total_paths);
                }
                if let Ok(data) = archive.read_file(name) {
                    if let Ok(text) = std::str::from_utf8(&data) {
                        if let Ok(sector) = map_parser::parse_text_sector(text) {
                            map.nodes.extend(sector.nodes);
                            map.roads.extend(sector.roads);
                            map.prefabs.extend(sector.prefabs);
                        }
                    } else {
                        if let Ok(sector) = binary_parser::parse_binary_sector(&data) {
                            map.nodes.extend(sector.nodes);
                            map.roads.extend(sector.roads);
                            map.prefabs.extend(sector.prefabs);
                        }
                    }
                }
            }
            eprintln!(); // newline after progress bar

            // Strategy 2: Brute-force — try every entry.
            if map.nodes.is_empty() {
                let hashes = archive.entry_hashes();
                let total = hashes.len();
                eprintln!("Brute-force scanning {} entries...", total);
                for (idx, hash) in hashes.iter().enumerate() {
                    if idx % 500 == 0 || idx == total - 1 {
                        progress("  Brute-force", idx + 1, total);
                    }
                    if let Ok(data) = archive.read_entry(*hash) {
                        if let Ok(text) = std::str::from_utf8(&data) {
                            if let Ok(sector) = map_parser::parse_text_sector(text) {
                                map.nodes.extend(sector.nodes);
                                map.roads.extend(sector.roads);
                                map.prefabs.extend(sector.prefabs);
                            }
                        } else {
                            if let Ok(sector) = binary_parser::parse_binary_sector(&data) {
                                map.nodes.extend(sector.nodes);
                                map.roads.extend(sector.roads);
                                map.prefabs.extend(sector.prefabs);
                            }
                        }
                    }
                }
                eprintln!();
            }
        }
    }

    // Strategy 3: Try def.scs and base.scs for definition data.
    for scs_path in &[def_scs, base_scs] {
        if !scs_path.exists() {
            continue;
        }
        eprintln!(
            "Opening {}...",
            scs_path.file_name().unwrap_or_default().to_string_lossy()
        );
        if let Ok(mut archive) = scs_reader::ScsArchive::open(scs_path) {
            let known = archive.list_known_files();
            for name in &known {
                if let Ok(data) = archive.read_file(name) {
                    if let Ok(text) = std::str::from_utf8(&data) {
                        let _ = sii_parser::parse_sii(text);
                    }
                }
            }
        }
    }

    // Deduplicate.
    eprintln!("Deduplicating...");
    dedup_map(&mut map);

    if map.nodes.is_empty() {
        return Err("no map data found".into());
    }

    Ok(map)
}

/// Merge several already-parsed [`SectorData`] blobs into a single
/// [`MapData`], deduplicating nodes by UID.
///
/// The first occurrence of each node UID wins (caller controls priority by
/// ordering the input list — the *highest priority* sector should come
/// first). Roads and prefabs are concatenated unchanged here; the caller's
/// later [`dedup_map`] pass collapses any UID collisions.
pub fn merge_sectors(sectors: Vec<SectorData>) -> MapData {
    let mut map = MapData {
        nodes: Vec::new(),
        roads: Vec::new(),
        prefabs: Vec::new(),
    };
    let mut seen_node_uids: HashSet<u64> = HashSet::new();

    for sector in sectors {
        for node in sector.nodes {
            if seen_node_uids.insert(node.uid) {
                map.nodes.push(node);
            }
        }
        for road in sector.roads {
            map.roads.push(road);
        }
        for prefab in sector.prefabs {
            map.prefabs.push(prefab);
        }
    }
    map
}

/// Parse one sector blob (text or binary) into [`SectorData`].
fn parse_sector_bytes(data: &[u8]) -> Result<SectorData, String> {
    let data = try_decompress_zlib(data.to_vec());
    if let Ok(text) = std::str::from_utf8(&data) {
        if let Ok(sector) = map_parser::parse_text_sector(text) {
            return Ok(SectorData {
                nodes: sector.nodes,
                roads: sector.roads,
                prefabs: sector.prefabs,
            });
        }
    }
    binary_parser::parse_binary_sector(&data)
}

/// Parse an ETS2 installation while applying a stack of map mods on top.
///
/// Strategy:
/// 1. Parse the base game with the existing `parse_ets2_map` (handles the
///    encrypted HashFS v2 format via the brute-force / sector-path probe).
/// 2. Open every enabled mod archive (ZIP or HashFS) and collect all
///    `.base` sector paths from them.
/// 3. For each mod sector, parse it and merge it into the base map.
///    Nodes are deduplicated by UID (mod wins on collision because mod
///    sectors are processed after the base game).
///
/// When `order` has no enabled descriptors this is equivalent to
/// [`parse_ets2_map`].
pub fn parse_ets2_map_with_mods(
    game_path: &Path,
    order: &mod_descriptor::ModLoadOrder,
) -> Result<MapData, String> {
    let enabled: Vec<&mod_descriptor::ModDescriptor> =
        order.descriptors.iter().filter(|d| d.is_enabled).collect();

    if enabled.is_empty() {
        return parse_ets2_map(game_path);
    }

    // Step 1: base game (best-effort — may fail for encrypted SCS v2).
    eprintln!("Parsing base game from {}...", game_path.display());
    let base_result = parse_ets2_map(game_path);
    let mut map = match base_result {
        Ok(m) => {
            eprintln!(
                "Base game: {} nodes, {} roads, {} prefabs",
                m.nodes.len(),
                m.roads.len(),
                m.prefabs.len()
            );
            m
        }
        Err(e) => {
            eprintln!("Base game parse failed ({}), continuing with mods only", e);
            MapData {
                nodes: Vec::new(),
                roads: Vec::new(),
                prefabs: Vec::new(),
            }
        }
    };
    let base_nodes = map.nodes.len();
    let base_roads = map.roads.len();
    let base_prefabs = map.prefabs.len();

    // Step 2: open mod archives and collect sector paths, sorted by load_order.
    let mut sorted_mods: Vec<&mod_descriptor::ModDescriptor> = enabled;
    sorted_mods.sort_by_key(|d| d.load_order);

    let mut total_sectors_loaded = 0usize;
    let mut total_overrides = 0usize;

    // Track which node UIDs already exist in the base map so we can count
    // overrides accurately.
    let mut existing_node_uids: HashSet<u64> = map.nodes.iter().map(|n| n.uid).collect();
    let mut existing_road_uids: HashSet<String> = map.roads.iter().map(|r| r.uid.clone()).collect();
    let mut existing_prefab_uids: HashSet<String> =
        map.prefabs.iter().map(|p| p.uid.clone()).collect();

    // Two-pass road extraction for new-format sectors:
    // 1. Pass 1: collect all node UIDs, defer road candidates.
    // 2. After pass 1: filter road candidates by known UIDs and merge.
    let mut deferred_road_candidates: Vec<Vec<crate::json_export::MapRoad>> = Vec::new();

    for desc in &sorted_mods {
        eprintln!("Opening mod: {} ({})", desc.name, desc.file_path.display());

        let mut archive = match archive::Archive::open(&desc.file_path) {
            Ok(a) => a,
            Err(e) => {
                eprintln!("  warning: cannot open {}: {}", desc.file_path.display(), e);
                continue;
            }
        };

        // Collect all .base sector paths from this archive.
        let sector_paths = archive::ArchiveReader::find_files_starting_with(&archive, "map/");
        let base_paths: Vec<String> = sector_paths
            .into_iter()
            .filter(|p| p.ends_with(".base"))
            .collect();

        eprintln!("  {} .base sectors found", base_paths.len());

        for path in &base_paths {
            let bytes = match archive::ArchiveReader::read_file(&mut archive, path) {
                Ok(b) => b,
                Err(e) => {
                    eprintln!("  warning: cannot read {}: {}", path, e);
                    continue;
                }
            };

            let sector = match parse_sector_bytes(&bytes) {
                Ok(s) => s,
                Err(e) => {
                    eprintln!("  warning: skipping {}: {}", path, e);
                    continue;
                }
            };

            total_sectors_loaded += 1;

            // Pass 1: Collect nodes immediately (for UID resolution).
            for node in sector.nodes {
                if existing_node_uids.contains(&node.uid) {
                    if let Some(existing) = map.nodes.iter_mut().find(|n| n.uid == node.uid) {
                        *existing = node;
                        total_overrides += 1;
                    }
                } else {
                    existing_node_uids.insert(node.uid);
                    map.nodes.push(node);
                }
            }

            // Defer road candidates for pass 2 (after all node UIDs are known).
            if !sector.roads.is_empty() {
                deferred_road_candidates.push(sector.roads);
            }

            for prefab in sector.prefabs {
                if !existing_prefab_uids.contains(&prefab.uid) {
                    existing_prefab_uids.insert(prefab.uid.clone());
                    map.prefabs.push(prefab);
                }
            }
        }
    }

    // Pass 2: Filter deferred road candidates against collected node UIDs.
    let mut filtered_roads = 0usize;
    let mut discarded_roads = 0usize;
    for sector_roads in &deferred_road_candidates {
        for road in sector_roads {
            if road.nodes.iter().all(|n| existing_node_uids.contains(n)) && road.nodes.len() >= 2 {
                if !existing_road_uids.contains(&road.uid) {
                    existing_road_uids.insert(road.uid.clone());
                    map.roads.push(road.clone());
                    filtered_roads += 1;
                }
            } else {
                discarded_roads += 1;
            }
        }
    }

    eprintln!(
        "Road filtering: {} accepted, {} discarded (no matching nodes)",
        filtered_roads, discarded_roads
    );

    eprintln!(
        "Mod loading complete: {} sectors loaded, {} node overrides",
        total_sectors_loaded, total_overrides
    );
    eprintln!(
        "Total: {} nodes (+{}), {} roads (+{}), {} prefabs (+{})",
        map.nodes.len(),
        map.nodes.len().saturating_sub(base_nodes),
        map.roads.len(),
        map.roads.len().saturating_sub(base_roads),
        map.prefabs.len(),
        map.prefabs.len().saturating_sub(base_prefabs)
    );

    dedup_map(&mut map);

    if map.nodes.is_empty() {
        return Err("no map data found in base game or mods".into());
    }

    Ok(map)
}

/// Parse a text-format map sector file into `MapData`.
///
/// This reads the `edit_save_text` exported format where map data is
/// stored as human-readable blocks: `node { ... }`, `road { ... }`,
/// `prefab { ... }`.
pub fn parse_text_map_file(path: &Path) -> Result<MapData, String> {
    let content = std::fs::read_to_string(path)
        .map_err(|e| format!("cannot read {}: {}", path.display(), e))?;

    parse_text_map(&content)
}

/// Parse text-format content directly.
pub fn parse_text_map(content: &str) -> Result<MapData, String> {
    let sector = map_parser::parse_text_sector(content)?;
    let mut map = MapData {
        nodes: sector.nodes,
        roads: sector.roads,
        prefabs: sector.prefabs,
    };
    dedup_map(&mut map);
    if map.nodes.is_empty() {
        return Err("no map data found".into());
    }
    Ok(map)
}

#[cfg(test)]
mod tests {
    use super::*;
    use miniz_oxide::deflate;
    use std::fs;
    use std::time::{SystemTime, UNIX_EPOCH};

    fn unique_temp_dir(prefix: &str) -> std::path::PathBuf {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos();
        let mut dir = std::env::temp_dir();
        dir.push(format!(
            "truckpilot_{prefix}_{}_{}",
            std::process::id(),
            nonce
        ));
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn build_test_binary_sector() -> Vec<u8> {
        let mut data = Vec::new();

        // header: version, game id token, map version, item count
        data.extend_from_slice(&906u32.to_le_bytes());
        data.extend_from_slice(&0u64.to_le_bytes());
        data.extend_from_slice(&0u64.to_le_bytes());
        data.extend_from_slice(&1u32.to_le_bytes());

        data.extend_from_slice(&3u32.to_le_bytes()); // road type
        data.extend_from_slice(&44u32.to_le_bytes());
        data.extend_from_slice(&0x0000_0000_0000_1000u64.to_le_bytes());
        data.extend_from_slice(&0x0000_0000_0000_1001u64.to_le_bytes());
        data.extend_from_slice(&120.0f32.to_le_bytes());
        data.extend_from_slice(&1u32.to_le_bytes());
        data.extend_from_slice(&1u32.to_le_bytes());
        data.extend_from_slice(&80.0f32.to_le_bytes());
        data.extend_from_slice(&0u32.to_le_bytes());
        data.extend_from_slice(&0u64.to_le_bytes());

        // node tail
        data.extend_from_slice(&2u32.to_le_bytes());
        data.extend_from_slice(&0x0000_0000_0000_1000u64.to_le_bytes());
        data.extend_from_slice(&0.0f64.to_le_bytes());
        data.extend_from_slice(&0.0f64.to_le_bytes());
        data.extend_from_slice(&0.0f64.to_le_bytes());
        data.extend_from_slice(&0.0f32.to_le_bytes());
        data.extend_from_slice(&0x0000_0000_0000_1001u64.to_le_bytes());
        data.extend_from_slice(&100.0f64.to_le_bytes());
        data.extend_from_slice(&0.0f64.to_le_bytes());
        data.extend_from_slice(&0.0f64.to_le_bytes());
        data.extend_from_slice(&0.0f32.to_le_bytes());

        data
    }

    #[test]
    fn test_parse_text_map_basic() {
        let input = r#"
node { uid: 0x1 position: (0,0,0) }
node { uid: 0x2 position: (10,0,0) }
road { uid: 0xA name: "test" look_token: "a" nodes: (0x1, 0x2) speed_limit: 50.0 lane_count_forward: 1 lane_count_backward: 0 }
"#;
        let map = parse_text_map(input).unwrap();
        assert_eq!(map.nodes.len(), 2);
        assert_eq!(map.roads.len(), 1);
        assert_eq!(map.roads[0].name, "test");
        assert!(map.prefabs.is_empty());
    }

    #[test]
    fn test_parse_text_map_dedup() {
        let input = r#"
node { uid: 0x1 position: (0,0,0) }
node { uid: 0x1 position: (999,999,999) }
"#;
        let map = parse_text_map(input).unwrap();
        assert_eq!(map.nodes.len(), 1);
    }

    #[test]
    fn test_parse_hashfs_sectors_dir_missing() {
        let mut missing = std::env::temp_dir();
        missing.push(format!(
            "truckpilot_missing_{}_{}",
            std::process::id(),
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos()
        ));
        assert!(!missing.exists());

        let err = parse_hashfs_sectors_dir(&missing).unwrap_err();
        assert!(err.contains("does not exist"));
    }

    #[test]
    fn test_parse_hashfs_sectors_dir_empty() {
        let dir = unique_temp_dir("empty_hashfs");
        let err = parse_hashfs_sectors_dir(&dir).unwrap_err();
        assert!(err.contains("no .base sector files found"));
        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn test_parse_hashfs_sectors_dir_valid_base() {
        let dir = unique_temp_dir("valid_hashfs");
        let data = build_test_binary_sector();
        fs::write(dir.join("test.base"), data).unwrap();

        let map = parse_hashfs_sectors_dir(&dir).unwrap();
        assert!(
            map.nodes.len() >= 2,
            "expected >=2 nodes, got {}",
            map.nodes.len()
        );
        assert!(!map.roads.is_empty(), "expected >=1 road");

        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn test_parse_hashfs_sectors_dir_skips_invalid_files() {
        let dir = unique_temp_dir("mixed_hashfs");
        fs::write(dir.join("bad.base"), b"this-is-not-a-sector").unwrap();
        fs::write(dir.join("good.base"), build_test_binary_sector()).unwrap();

        let map = parse_hashfs_sectors_dir(&dir).unwrap();
        assert!(!map.nodes.is_empty());
        assert!(!map.roads.is_empty());

        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn test_parse_hashfs_sectors_dir_zlib_compressed_base() {
        let dir = unique_temp_dir("compressed_hashfs");
        let raw = build_test_binary_sector();
        let compressed = deflate::compress_to_vec_zlib(&raw, 6);
        fs::write(dir.join("compressed.base"), compressed).unwrap();

        let map = parse_hashfs_sectors_dir(&dir).unwrap();
        assert!(map.nodes.len() >= 2);
        assert!(!map.roads.is_empty());

        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    fn test_parse_hashfs_sectors_dir_all_invalid_returns_error() {
        let dir = unique_temp_dir("all_invalid_hashfs");
        fs::write(dir.join("a.base"), b"garbage-one").unwrap();
        fs::write(dir.join("b.base"), b"garbage-two").unwrap();

        let err = parse_hashfs_sectors_dir(&dir).unwrap_err();
        assert!(err.contains("no map data found"));

        let _ = fs::remove_dir_all(dir);
    }

    #[test]
    #[ignore = "requires ETS2 installation at default path"]
    fn test_real_ets2_map_parsing() {
        let path =
            Path::new("C:/Program Files (x86)/Steam/steamapps/common/Euro Truck Simulator 2");
        if !path.exists() {
            return;
        }
        let map = parse_ets2_map(path).unwrap();
        assert!(
            map.nodes.len() > 100,
            "expected >100 nodes, got {}",
            map.nodes.len()
        );
        assert!(
            map.roads.len() > 100,
            "expected >100 roads, got {}",
            map.roads.len()
        );
    }
}
