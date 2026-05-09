//! Mod loader — discovers and layers base game and mod `.scs` archives.

use std::path::{Path, PathBuf};
use std::time::Instant;

use tracing::{debug, info, instrument, warn};

use crate::archive::Archive;
use crate::cache::{compute_cache_key, load_cache, save_cache};
use crate::error::ParseError;
use crate::graph::{GraphBuilder, MapGraph};
use crate::hashfs::HashFsArchive;
use crate::sector::parse_sector;
use crate::zip_archive::ZipArchive;

/// A single archive file to be loaded.
#[derive(Debug, Clone, Ord, PartialOrd, Eq, PartialEq)]
pub struct ArchiveFile {
    pub name: String,
    pub path: PathBuf,
}

/// The final, ordered list of all archives (base + mods) to be loaded.
#[derive(Debug, Default)]
pub struct ModLoadOrder {
    pub entries: Vec<ArchiveFile>,
}

impl ModLoadOrder {
    /// Discover all base game and mod archives and create the final load order.
    pub fn from_directories(base_dir: &Path, mods_dir: &Path) -> Result<Self, ParseError> {
        // 1. Get base game files (unsorted, order doesn't matter among them)
        let mut base_files = Self::discover_scs(base_dir)?;
        info!("Found {} base game .scs files in {:?}", base_files.len(), base_dir);

        // 2. Get mod files and sort alphabetically (last wins)
        let mut mod_files = if mods_dir.exists() {
            let files = Self::discover_scs(mods_dir)?;
            info!("Found {} mod .scs files in {:?}", files.len(), mods_dir);
            files
        } else {
            warn!("Mods directory not found, skipping: {:?}", mods_dir);
            vec![]
        };
        mod_files.sort(); // Alphabetical sort determines load order

        // 3. Combine them: base files first, then mods
        let mut all_entries = Vec::new();
        all_entries.append(&mut base_files);
        all_entries.append(&mut mod_files);

        info!("Final load order ({} total archives):", all_entries.len());
        for (i, entry) in all_entries.iter().enumerate() {
            info!("  {:>2}. {}", i + 1, entry.name);
        }

        Ok(Self { entries: all_entries })
    }

    /// Helper to find all `.scs` files in a directory.
    fn discover_scs(dir: &Path) -> Result<Vec<ArchiveFile>, ParseError> {
        Ok(std::fs::read_dir(dir)?
            .filter_map(|e| e.ok())
            .filter(|e| e.path().extension().and_then(|s| s.to_str()) == Some("scs"))
            .map(|e| {
                let path = e.path();
                let name = path.file_name().unwrap().to_string_lossy().to_string();
                ArchiveFile { name, path }
            })
            .collect())
    }
}

/// Open an SCS file, trying HashFS first, then falling back to ZIP.
fn open_scs_archive(path: &Path) -> Result<Box<dyn Archive>, ParseError> {
    match HashFsArchive::open(path) {
        Ok(archive) => Ok(Box::new(archive)),
        Err(ParseError::InvalidMagic(_)) => {
            warn!("{:?}: not a HashFS archive, trying ZIP fallback", path.file_name().unwrap());
            let archive = ZipArchive::open(path)?;
            Ok(Box::new(archive))
        }
        Err(e) => Err(e), // Other HashFS error
    }
}

#[instrument(skip(order, cache_dir))]
pub fn load_and_build(order: &ModLoadOrder, cache_dir: Option<&Path>) -> Result<MapGraph, ParseError> {
    let t_total = Instant::now();

    if order.entries.is_empty() {
        warn!("No archives in load order");
        return Ok(MapGraph::default());
    }

    info!("Archive phase: opening {} archive(s)", order.entries.len());
    let t_archives = Instant::now();

    let mut archives: Vec<Box<dyn Archive>> = Vec::new();
    for entry in &order.entries {
        match open_scs_archive(&entry.path) {
            Ok(arc) => {
                if entry.name == "base_map.scs" {
                    if let Some(hashfs) = arc.as_any().downcast_ref::<HashFsArchive>() {
                        info!(
                            "base_map.scs index: {} entries — listing first 20",
                            hashfs.entries().len()
                        );
                        for (i, (hash, dir_entry)) in
                            hashfs.entries().iter().take(20).enumerate()
                        {
                            info!(
                                "  entry {i}: hash={hash:016x} size={}",
                                dir_entry.size
                            );
                        }
                    }
                }
                archives.push(arc);
            }
            Err(e) => warn!("  Skipping {}: {e}", entry.name),
        }
    }

    info!("Archive phase done in {:.1} ms", t_archives.elapsed().as_secs_f64() * 1000.0);

    // ── debug probe: hash known paths and check archive containment ──
    {
        use crate::cityhash::cityhash64;

        let test_paths = [
            "map/europe.mbd",
            "map/europe/sec+0000+0000.base",
            "map/europe/sec-0001-0001.base",
            "map/europe.sii",
            "def/world/road.sii",
            "version.txt",
        ];

        for path in &test_paths {
            let hash = cityhash64(path.as_bytes());
            let mut found_in = vec![];
            for archive in &archives {
                if archive.contains(path) {
                    found_in.push(
                        archive
                            .path()
                            .file_name()
                            .unwrap()
                            .to_string_lossy()
                            .to_string(),
                    );
                }
            }
            info!(
                "test path '{}' hash={:016x} found_in={:?}",
                path, hash, found_in
            );
        }

        // Probe a real read + parse on the first sector that any archive contains.
        for archive in &mut archives {
            if archive.contains("map/europe/sec+0000+0000.base") {
                match archive.read_path("map/europe/sec+0000+0000.base") {
                    Ok(data) => {
                        info!(
                            "read OK from {}: {} bytes",
                            archive.path().file_name().unwrap().to_string_lossy(),
                            data.len()
                        );
                        match crate::sector::parse_sector(&data) {
                            Ok(s) => info!(
                                "  parsed: {} nodes, {} roads, {} prefabs",
                                s.nodes.len(),
                                s.roads.len(),
                                s.prefabs.len()
                            ),
                            Err(e) => info!("  parse FAILED: {:?}", e),
                        }
                        break;
                    }
                    Err(e) => info!("read FAILED: {:?}", e),
                }
            }
        }
    }

    if let Some(cache_dir) = cache_dir {
        let hashes: Vec<[u8; 32]> = archives.iter().map(|a| a.file_hash()).collect();
        let key = compute_cache_key(&hashes);
        if let Some(graph) = load_cache(cache_dir, &key) {
            info!("Cache hit — total load time {:.1} ms", t_total.elapsed().as_secs_f64() * 1000.0);
            return Ok(graph);
        }

        let graph = parse_sectors_from_archives(&mut archives)?;

        if let Err(e) = save_cache(cache_dir, &key, &graph) {
            warn!("Failed to save cache: {e}");
        }

        info!("Total load time: {:.1} ms", t_total.elapsed().as_secs_f64() * 1000.0);
        Ok(graph)
    } else {
        let graph = parse_sectors_from_archives(&mut archives)?;
        info!("Total load time (no cache): {:.1} ms", t_total.elapsed().as_secs_f64() * 1000.0);
        Ok(graph)
    }
}

fn parse_sectors_from_archives(archives: &mut [Box<dyn Archive>]) -> Result<MapGraph, ParseError> {
    let t_sectors = Instant::now();
    let mut builder = GraphBuilder::new();
    let mut sector_count = 0usize;

    // 1. Collect all unique file paths from all archives.
    // For ZIPs, this is a full file list. For HashFS, it's the probed paths.
    let mut all_paths = std::collections::HashSet::new();
    let mut override_count = 0;
    info!("Collecting file lists from {} archives...", archives.len());
    for (i, arc) in archives.iter().enumerate() {
        let mut files = arc.list_files();
        // If list_files is empty (HashFS), fall back to probing.
        if files.is_empty() {
            if let Some(hashfs_arc) = arc.as_any().downcast_ref::<HashFsArchive>() {
                files = hashfs_arc.probe_sector_paths();
            }
        }

        let initial_len = all_paths.len();
        all_paths.extend(files);
        let new_count = all_paths.len() - initial_len;
        if i > 0 && new_count > 0 {
            let overridden = (arc.list_files().len() + hashfs_probe_len(&**arc)) - new_count;
            if overridden > 0 {
                override_count += overridden;
            }
        }
        debug!("  {}: found {} new files", arc.path().file_name().unwrap().to_string_lossy(), new_count);
    }
    
    info!("Found {} total unique files. {} files were overridden by mods.", all_paths.len(), override_count);

    // Sector binary format lives in `*.base`. Other extensions (`.aux`,
    // `.data`, `.desc`) and prefab files use different layouts and would
    // produce spurious empty parses.
    let sector_paths: Vec<String> = all_paths
        .into_iter()
        .filter(|p| p.ends_with(".base"))
        .collect();

    if !sector_paths.is_empty() {
        info!("Parsing {} sector and prefab files...", sector_paths.len());
        for (i, path) in sector_paths.iter().enumerate() {
            if i > 0 && i % 500 == 0 {
                info!("  ...parsed {}/{} files", i, sector_paths.len());
            }
            // Find the last archive that contains this path (highest priority)
            let data = archives.iter_mut().rev().find_map(|arc| arc.read_path(path).ok());
            let Some(data) = data else { continue };
            match parse_sector(&data) {
                Ok(sector) => {
                    builder.merge_sector(sector);
                    sector_count += 1;
                }
                Err(e) => warn!("Failed to parse sector {}: {e}", path),
            }
        }
    } else {
        warn!("No sector files found in any loaded archive.");
    }

    info!(
        "Sector phase done: {} sectors parsed in {:.1} ms",
        sector_count,
        t_sectors.elapsed().as_secs_f64() * 1000.0
    );

    Ok(builder.build())
}

fn hashfs_probe_len(arc: &dyn Archive) -> usize {
    if let Some(hashfs_arc) = arc.as_any().downcast_ref::<HashFsArchive>() {
        hashfs_arc.probe_sector_paths().len()
    } else {
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_load_order_returns_empty_graph() {
        let order = ModLoadOrder::default();
        let g = load_and_build(&order, None).unwrap();
        assert!(g.nodes.is_empty());
    }

    #[test]
    fn invalid_scs_is_skipped_gracefully() {
        let tmp = std::env::temp_dir().join("truckpilot_invalid.scs");
        std::fs::write(&tmp, b"this is not a valid archive").unwrap();
        let order = ModLoadOrder {
            entries: vec![ArchiveFile { name: "invalid".into(), path: tmp.clone() }]
        };
        let g = load_and_build(&order, None).unwrap();
        assert!(g.nodes.is_empty());
        std::fs::remove_file(&tmp).ok();
    }
}
