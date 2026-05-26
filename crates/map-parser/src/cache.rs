//! Graph cache — serialises/deserialises `MapGraph` to disk using bincode v2.
//!
//! Cache key: SHA-256 of all mod file hashes + parser version tag.
//! Format: bincode-encoded `CacheFile` struct.
//!
//! Cache files live in `<cache_dir>/<hex_key>.bin`.

use std::path::{Path, PathBuf};

use bincode::config::Configuration;
use bincode::{decode_from_slice, encode_to_vec};
use sha2::{Digest, Sha256};
use tracing::{debug, info, warn};

use crate::error::ParseError;
use crate::graph::MapGraph;

/// Increment when the `MapGraph` schema changes to invalidate old caches.
const PARSER_VERSION: u32 = 6; // Bumped: PrefabAiPath gained start/end_rotation (DS13a)
const BINCODE_CONFIG: Configuration = bincode::config::standard();

/// On-disk cache envelope.
#[derive(serde::Serialize, serde::Deserialize, bincode::Encode, bincode::Decode)]
struct CacheFile {
    /// Must equal `PARSER_VERSION` to be valid.
    parser_version: u32,
    /// Hex string of the mod-combo hash (for human inspection).
    combo_hash: String,
    /// The serialised graph.
    graph: MapGraph,
}

/// Compute the cache key from a list of per-archive SHA-256 hashes.
///
/// The key is SHA-256( PARSER_VERSION || sorted(archive_hashes) ).
pub fn compute_cache_key(archive_hashes: &[[u8; 32]]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(PARSER_VERSION.to_le_bytes());
    // Sort so that mod order doesn't affect the key (content-addressed).
    let mut sorted = archive_hashes.to_vec();
    sorted.sort();
    for h in &sorted {
        hasher.update(h);
    }
    hex_encode(&hasher.finalize())
}

/// Try to load a cached graph. Returns `None` on any miss or version mismatch.
pub fn load_cache(cache_dir: &Path, key: &str) -> Option<MapGraph> {
    let path = cache_path(cache_dir, key);
    if !path.exists() {
        debug!("Cache miss: {:?}", path);
        return None;
    }

    let data = std::fs::read(&path)
        .map_err(|e| warn!("Cache read error {:?}: {e}", path))
        .ok()?;

    let (file, _len): (CacheFile, usize) = decode_from_slice(&data, BINCODE_CONFIG)
        .map_err(|e| warn!("Cache deserialise error: {e}"))
        .ok()?;

    if file.parser_version != PARSER_VERSION {
        warn!(
            "Cache version mismatch (got {}, want {}), ignoring",
            file.parser_version, PARSER_VERSION
        );
        return None;
    }

    info!(
        "Cache hit: {} nodes, {} edges",
        file.graph.nodes.len(),
        file.graph.edges.len()
    );
    Some(file.graph)
}

/// Write a graph to the cache.
pub fn save_cache(cache_dir: &Path, key: &str, graph: &MapGraph) -> Result<(), ParseError> {
    std::fs::create_dir_all(cache_dir)
        .map_err(|e| ParseError::Io(format!("create cache dir: {e}")))?;

    let file = CacheFile {
        parser_version: PARSER_VERSION,
        combo_hash: key.to_string(),
        graph: graph.clone(),
    };

    let data = encode_to_vec(file, BINCODE_CONFIG)
        .map_err(|e| ParseError::Cache(format!("serialise: {e}")))?;

    let path = cache_path(cache_dir, key);
    std::fs::write(&path, &data)
        .map_err(|e| ParseError::Io(format!("write cache {:?}: {e}", path)))?;

    info!("Cache saved: {:?} ({} bytes)", path, data.len());
    Ok(())
}

/// Delete all cache files in `cache_dir`.
pub fn clear_cache(cache_dir: &Path) -> Result<usize, ParseError> {
    let mut count = 0;
    if !cache_dir.exists() {
        return Ok(0);
    }
    for entry in
        std::fs::read_dir(cache_dir).map_err(|e| ParseError::Io(format!("read cache dir: {e}")))?
    {
        let entry = entry.map_err(|e| ParseError::Io(format!("dir entry: {e}")))?;
        let path = entry.path();
        if path.extension().and_then(|s| s.to_str()) == Some("bin") {
            std::fs::remove_file(&path)
                .map_err(|e| ParseError::Io(format!("remove {:?}: {e}", path)))?;
            count += 1;
        }
    }
    Ok(count)
}

fn cache_path(dir: &Path, key: &str) -> PathBuf {
    dir.join(format!("{key}.bin"))
}

fn hex_encode(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::{BuildStats, MapGraph};
    use std::collections::HashMap;

    fn empty_graph() -> MapGraph {
        MapGraph {
            nodes: vec![],
            edges: vec![],
            signs: vec![],
            prefabs: vec![],
            stats: BuildStats::default(),
            prefab_ai_paths: vec![],
            prefab_instances: vec![],
            prefab_descriptors: HashMap::new(),
        }
    }

    #[test]
    fn cache_key_is_deterministic() {
        let h1 = [1u8; 32];
        let h2 = [2u8; 32];
        let k1 = compute_cache_key(&[h1, h2]);
        let k2 = compute_cache_key(&[h2, h1]); // different order
        assert_eq!(k1, k2, "key must be order-independent");
    }

    #[test]
    fn cache_key_differs_for_different_archives() {
        let k1 = compute_cache_key(&[[1u8; 32]]);
        let k2 = compute_cache_key(&[[2u8; 32]]);
        assert_ne!(k1, k2);
    }

    #[test]
    fn roundtrip_save_load() {
        let dir = std::env::temp_dir().join("truckpilot_cache_test_bincode");
        let _ = std::fs::remove_dir_all(&dir);

        let graph = empty_graph();
        let key = compute_cache_key(&[[42u8; 32]]);

        save_cache(&dir, &key, &graph).unwrap();
        let loaded = load_cache(&dir, &key).unwrap();
        assert_eq!(loaded.nodes.len(), 0);

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn missing_cache_returns_none() {
        let dir = std::env::temp_dir().join("truckpilot_cache_miss_bincode");
        let _ = std::fs::remove_dir_all(&dir);
        assert!(load_cache(&dir, "nonexistent").is_none());
    }

    #[test]
    fn clear_removes_bin_files() {
        let dir = std::env::temp_dir().join("truckpilot_cache_clear_bincode");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("abc.bin"), b"x").unwrap();
        std::fs::write(dir.join("def.bin"), b"y").unwrap();
        std::fs::write(dir.join("keep.txt"), b"z").unwrap();

        let removed = clear_cache(&dir).unwrap();
        assert_eq!(removed, 2);
        assert!(dir.join("keep.txt").exists());

        let _ = std::fs::remove_dir_all(&dir);
    }
}
