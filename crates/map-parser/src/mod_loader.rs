//! Mod loader — discovers and layers base game and mod `.scs` archives.

use std::path::{Path, PathBuf};
use std::time::Instant;

use tracing::{debug, info, instrument, warn};

use std::collections::HashMap;

use crate::archive::Archive;
use crate::cache::{compute_cache_key, load_cache, save_cache};
use crate::drop_tracer::DropTracer;
use crate::error::ParseError;
use crate::graph::{GraphBuilder, MapGraph};
use crate::hashfs::{parse_directory_listing, scs_path_hash, HashFsArchive};
use crate::ppd::PrefabDescriptor;
use crate::road_look::{load_road_look, scs_token_hash, RoadLookEntry};
use crate::sector::{parse_sector, parse_sector_with_tracer};
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
        info!(
            "Found {} base game .scs files in {:?}",
            base_files.len(),
            base_dir
        );

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

        Ok(Self {
            entries: all_entries,
        })
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
            warn!(
                "{:?}: not a HashFS archive, trying ZIP fallback",
                path.file_name().unwrap()
            );
            let archive = ZipArchive::open(path)?;
            Ok(Box::new(archive))
        }
        Err(e) => Err(e), // Other HashFS error
    }
}

#[instrument(skip(order, cache_dir))]
pub fn load_and_build(
    order: &ModLoadOrder,
    cache_dir: Option<&Path>,
) -> Result<MapGraph, ParseError> {
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
                        for (i, (hash, dir_entry)) in hashfs.entries().iter().take(20).enumerate() {
                            info!("  entry {i}: hash={hash:016x} size={}", dir_entry.size);
                        }
                    }
                }
                archives.push(arc);
            }
            Err(e) => warn!("  Skipping {}: {e}", entry.name),
        }
    }

    info!(
        "Archive phase done in {:.1} ms",
        t_archives.elapsed().as_secs_f64() * 1000.0
    );

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
            info!(
                "Cache hit — total load time {:.1} ms",
                t_total.elapsed().as_secs_f64() * 1000.0
            );
            return Ok(graph);
        }

        let road_look = load_road_look(&mut archives);
        let graph = parse_sectors_from_archives(&mut archives, road_look)?;

        if let Err(e) = save_cache(cache_dir, &key, &graph) {
            warn!("Failed to save cache: {e}");
        }

        info!(
            "Total load time: {:.1} ms",
            t_total.elapsed().as_secs_f64() * 1000.0
        );
        Ok(graph)
    } else {
        let road_look = load_road_look(&mut archives);
        let graph = parse_sectors_from_archives(&mut archives, road_look)?;
        info!(
            "Total load time (no cache): {:.1} ms",
            t_total.elapsed().as_secs_f64() * 1000.0
        );
        Ok(graph)
    }
}

/// Like [`parse_sectors_from_archives`] but records every sector-level and
/// (optionally) graph-level drop into `tracer`. Returns the `GraphBuilder`
/// (not yet built) and a `Vec<(sector_path, sector_data)>` of every sector
/// that was successfully read, for caller-side road-sector mapping.
///
/// The caller owns the `DropTracer` and calls `tracer.take_events()` after
/// this returns.
pub fn parse_sectors_with_drop_tracer(
    archives: &mut [Box<dyn Archive>],
    tracer: &DropTracer,
) -> Result<(GraphBuilder, Vec<String>), ParseError> {
    let road_look = load_road_look(archives);
    let mut builder = GraphBuilder::new();
    builder.set_road_look(road_look);
    let mut parsed_paths: Vec<String> = Vec::new();
    let mut all_paths = std::collections::HashSet::new();

    for arc in archives.iter() {
        let mut files = arc.list_files();
        if files.is_empty() {
            if let Some(hashfs_arc) = arc.as_any().downcast_ref::<crate::hashfs::HashFsArchive>() {
                files = hashfs_arc.probe_sector_paths();
            }
        }
        all_paths.extend(files);
    }

    let sector_paths: Vec<String> = all_paths
        .into_iter()
        .filter(|p| p.ends_with(".base"))
        .collect();

    for path in &sector_paths {
        let data = archives
            .iter_mut()
            .rev()
            .find_map(|arc| arc.read_path(path).ok());
        let Some(data) = data else { continue };

        match parse_sector_with_tracer(&data, path, tracer) {
            Ok(sector) => {
                builder.merge_sector(sector);
                parsed_paths.push(path.clone());
            }
            Err(e) => warn!("Failed to parse sector {path}: {e}"),
        }
    }

    Ok((builder, parsed_paths))
}

fn parse_sectors_from_archives(
    archives: &mut [Box<dyn Archive>],
    road_look: HashMap<u64, RoadLookEntry>,
) -> Result<MapGraph, ParseError> {
    let t_sectors = Instant::now();
    let mut builder = GraphBuilder::new();
    builder.set_road_look(road_look);
    let mut sector_count = 0usize;
    let mut total_roads = 0usize;
    let mut next_road_milestone = 500usize;

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
        debug!(
            "  {}: found {} new files",
            arc.path().file_name().unwrap().to_string_lossy(),
            new_count
        );
    }

    info!(
        "Found {} total unique files. {} files were overridden by mods.",
        all_paths.len(),
        override_count
    );

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
            if i > 0 && i % 100 == 0 {
                info!("  sector {}/{}: {}", i, sector_paths.len(), path);
            }
            // Find the last archive that contains this path (highest priority)
            let data = archives
                .iter_mut()
                .rev()
                .find_map(|arc| arc.read_path(path).ok());
            let Some(data) = data else { continue };
            match parse_sector(&data) {
                Ok(sector) => {
                    total_roads += sector.roads.len();
                    while total_roads >= next_road_milestone {
                        info!(
                            "  ...{} roads parsed across {} sectors",
                            total_roads,
                            sector_count + 1
                        );
                        next_road_milestone += 500;
                    }
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
        "Sector phase done: {} sectors parsed, {} roads extracted in {:.1} ms",
        sector_count,
        total_roads,
        t_sectors.elapsed().as_secs_f64() * 1000.0
    );

    // Load PPD descriptors for prefabs
    let (ppd_descriptors, ppd_stats) = load_ppd_descriptors(archives, &builder);
    builder.set_ppd_descriptors(ppd_descriptors);
    builder.set_ppd_stats(ppd_stats.0, ppd_stats.1, ppd_stats.2, ppd_stats.3);

    Ok(builder.build())
}

fn hashfs_probe_len(arc: &dyn Archive) -> usize {
    if let Some(hashfs_arc) = arc.as_any().downcast_ref::<HashFsArchive>() {
        hashfs_arc.probe_sector_paths().len()
    } else {
        0
    }
}

/// Walk all `.ppd` files under the `prefab/` and `prefab2/` directory trees
/// of a HashFS archive using directory listings.
fn walk_ppd_paths(arc: &HashFsArchive) -> Vec<String> {
    use std::collections::HashSet;
    let mut hits = Vec::new();
    // Seed from known PPD roots; avoids scanning the whole archive.
    let mut stack: Vec<String> = vec!["prefab2".into(), "prefab".into()];
    let mut seen: HashSet<String> = HashSet::new();
    while let Some(dir) = stack.pop() {
        if !seen.insert(dir.clone()) {
            continue;
        }
        let bytes = match arc.read_hash(scs_path_hash(0, &dir.to_ascii_lowercase())) {
            Ok(b) => b,
            Err(_) => continue,
        };
        let items = match parse_directory_listing(&bytes) {
            Ok(v) => v,
            Err(_) => continue,
        };
        for it in items {
            let child = format!("{dir}/{}", it.name);
            if it.is_dir {
                stack.push(child);
            } else if child.to_ascii_lowercase().ends_with(".ppd") {
                hits.push(child);
            }
        }
    }
    hits
}

/// Load PPD (Prefab Descriptor) files referenced by prefab template tokens.
///
/// Walks the `prefab/` and `prefab2/` trees in HashFS archives via directory
/// listings, builds a `scs_token_hash(stem) → path` map, then reads and
/// parses each PPD whose token appears in the accumulated raw prefabs.
///
/// Returns `(token→descriptor map, (attempted, loaded, failed, total_nav_curves))`.
fn load_ppd_descriptors(
    archives: &mut [Box<dyn Archive>],
    builder: &GraphBuilder,
) -> (HashMap<u64, PrefabDescriptor>, (usize, usize, usize, usize)) {
    use crate::ppd::parse_ppd;
    use std::collections::{HashMap, HashSet};

    let t0 = std::time::Instant::now();

    // Collect unique template tokens from all accumulated raw prefabs
    let tokens: HashSet<u64> = builder
        .raw_prefabs()
        .iter()
        .map(|p| p.template_token)
        .filter(|&t| t != 0)
        .collect();

    if tokens.is_empty() {
        return (HashMap::new(), (0, 0, 0, 0));
    }

    // Build token→path map by walking PPD directories.
    // HashFS: walk directory listings. ZIP: list_files() returns strings.
    let mut token_to_path: HashMap<u64, String> = HashMap::new();

    for arc in archives.iter() {
        if let Some(hashfs_arc) = arc.as_any().downcast_ref::<HashFsArchive>() {
            for path in walk_ppd_paths(hashfs_arc) {
                if let Some(stem) = std::path::Path::new(&path)
                    .file_stem()
                    .and_then(|s| s.to_str())
                {
                    let tok = scs_token_hash(&stem.to_ascii_lowercase());
                    token_to_path.entry(tok).or_insert(path);
                }
            }
        }
        for path in arc.list_files() {
            if path.to_ascii_lowercase().ends_with(".ppd") {
                if let Some(stem) = std::path::Path::new(&path)
                    .file_stem()
                    .and_then(|s| s.to_str())
                {
                    let tok = scs_token_hash(&stem.to_ascii_lowercase());
                    token_to_path.entry(tok).or_insert(path);
                }
            }
        }
    }

    info!(
        "PPD discovery: {} paths indexed for {} unique tokens",
        token_to_path.len(),
        tokens.len()
    );

    let mut descriptors: HashMap<u64, PrefabDescriptor> = HashMap::new();
    let attempted = tokens.len();
    let mut loaded = 0usize;
    let mut failed = 0usize;
    let mut total_nav_curves = 0usize;

    for token in &tokens {
        let data = token_to_path
            .get(token)
            .and_then(|path| archives.iter_mut().rev().find_map(|arc| arc.read_path(path).ok()));

        if let Some(raw) = data {
            match parse_ppd(&raw) {
                Ok(desc) => {
                    total_nav_curves += desc.nav_curves.len();
                    descriptors.insert(*token, desc);
                    loaded += 1;
                }
                Err(e) => {
                    debug!("Failed to parse PPD for token 0x{token:016X}: {e}");
                    failed += 1;
                }
            }
        } else {
            debug!("No PPD path found for token 0x{token:016X}");
            failed += 1;
        }
    }

    let elapsed = t0.elapsed().as_secs_f64() * 1000.0;
    info!(
        "PPD load: {}/{} loaded, {} failed, {} nav_curves in {:.1} ms",
        loaded, attempted, failed, total_nav_curves, elapsed
    );

    (descriptors, (attempted, loaded, failed, total_nav_curves))
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
            entries: vec![ArchiveFile {
                name: "invalid".into(),
                path: tmp.clone(),
            }],
        };
        let g = load_and_build(&order, None).unwrap();
        assert!(g.nodes.is_empty());
        std::fs::remove_file(&tmp).ok();
    }
}
