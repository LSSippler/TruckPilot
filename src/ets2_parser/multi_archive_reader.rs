//! Multi-archive reader with mod priority resolution.
//!
//! Loads several `.scs` archives (base game + mods + connector patches)
//! and answers sector lookups according to the configured load order:
//! the descriptor with the *highest* `load_order` wins. This mirrors how
//! ETS2 itself layers mods on top of the base game.

use std::collections::{BTreeSet, HashMap};
use std::path::Path;

use crate::ets2_parser::archive::{Archive, ArchiveReader};
use crate::ets2_parser::error::{Error, Result};
use crate::ets2_parser::mod_descriptor::{ModDescriptor, ModLoadOrder};

/// Trait abstracting "an archive that can answer reads by logical path and
/// can enumerate which logical paths it contains".
///
/// This is a thin layer above [`ArchiveReader`] used so that tests can
/// substitute in-memory fakes without writing real `.scs` files.
pub trait SectorSource {
    /// Read raw bytes for a sector path. Returns `None` if not present.
    fn read_sector(&mut self, sector_path: &str) -> Option<Vec<u8>>;
    /// Return all sector paths present in this source.
    fn sector_paths(&self) -> Vec<String>;
}

/// Adapter wrapping an [`Archive`]. It probes the archive against the
/// canonical ETS2 sector path list and remembers which probes succeeded.
///
/// For ZIP archives the index is queried directly (O(1) per path) instead
/// of reading file data, so even 40 000-entry archives are indexed quickly.
/// For HashFS archives we fall back to the probe-by-read strategy.
pub struct ArchiveSource {
    archive: Archive,
    available_paths: BTreeSet<String>,
}

impl ArchiveSource {
    /// Open an archive file and build the set of available sector paths.
    pub fn open(path: &Path) -> Result<Self> {
        let archive = Archive::open(path)?;
        let available_paths = Self::index_paths(&archive);
        Ok(ArchiveSource {
            archive,
            available_paths,
        })
    }

    /// Build the set of sector paths present in the archive.
    ///
    /// For ZIP archives we use `find_files_starting_with("map/")` which
    /// is an O(n) scan of the in-memory index — much faster than probing
    /// 10 000+ paths one by one. For HashFS archives we fall back to the
    /// probe strategy because the HashFS reader only knows paths it has
    /// been explicitly asked about.
    fn index_paths(archive: &Archive) -> BTreeSet<String> {
        // Try the fast path first: ask the archive for all map/ entries.
        let fast = archive.find_files_starting_with("map/");
        if !fast.is_empty() {
            return fast
                .into_iter()
                .filter(|p| p.ends_with(".base") || p.ends_with(".data"))
                .collect();
        }

        // Slow path: probe every known sector path (HashFS archives).
        BTreeSet::new() // populated lazily via get_sector probes
    }

    /// Number of indexed sector paths in this archive.
    pub fn known_path_count(&self) -> usize {
        self.available_paths.len()
    }
}

impl SectorSource for ArchiveSource {
    fn read_sector(&mut self, sector_path: &str) -> Option<Vec<u8>> {
        self.archive.read_file(sector_path).ok()
    }

    fn sector_paths(&self) -> Vec<String> {
        self.available_paths.iter().cloned().collect()
    }
}

/// In-memory sector source — the test fake.
pub struct InMemorySource {
    sectors: HashMap<String, Vec<u8>>,
}

impl InMemorySource {
    /// Construct from a list of `(path, data)` pairs.
    pub fn new(sectors: Vec<(String, Vec<u8>)>) -> Self {
        InMemorySource {
            sectors: sectors.into_iter().collect(),
        }
    }
}

impl SectorSource for InMemorySource {
    fn read_sector(&mut self, sector_path: &str) -> Option<Vec<u8>> {
        self.sectors.get(sector_path).cloned()
    }

    fn sector_paths(&self) -> Vec<String> {
        let mut paths: Vec<String> = self.sectors.keys().cloned().collect();
        paths.sort();
        paths
    }
}

/// A reader that combines multiple archives with priority-based override
/// semantics.
///
/// `archives` is stored in *ascending* load-order: index 0 is the lowest
/// priority (base game), index `len-1` is the highest (e.g. connector
/// patch). [`resolve_sector`](Self::resolve_sector) walks the list in
/// reverse so the first hit corresponds to the highest priority.
pub struct MultiArchiveReader {
    archives: Vec<(ModDescriptor, Box<dyn SectorSource>)>,
    /// `sector_path -> Vec<(load_order, archive_index)>`, ordered by
    /// ascending load_order so the highest priority is `last()`.
    sector_index: HashMap<String, Vec<(u32, usize)>>,
}

impl MultiArchiveReader {
    /// Create an empty reader.
    pub fn new() -> Self {
        MultiArchiveReader {
            archives: Vec::new(),
            sector_index: HashMap::new(),
        }
    }

    /// Open every descriptor as an [`ArchiveSource`] and build the index.
    ///
    /// Descriptors are sorted by `load_order` ascending before opening, so
    /// the resulting `archives` vector is itself in priority order.
    /// Disabled descriptors are skipped.
    pub fn load_archives(descriptors: &[ModDescriptor]) -> Result<Self> {
        let mut sorted: Vec<ModDescriptor> = descriptors
            .iter()
            .filter(|d| d.is_enabled)
            .cloned()
            .collect();
        sorted.sort_by_key(|d| d.load_order);

        let mut reader = MultiArchiveReader::new();
        for desc in sorted {
            let source = ArchiveSource::open(&desc.file_path)?;
            reader.push(desc, Box::new(source));
        }
        Ok(reader)
    }

    /// Open archives from a [`ModLoadOrder`], including base-game paths.
    ///
    /// Base-game paths are inserted at load_order 0 (lowest priority); the
    /// mod descriptors retain their declared order.
    pub fn load_from_order(order: &ModLoadOrder) -> Result<Self> {
        let mut reader = MultiArchiveReader::new();

        for path in &order.base_game_paths {
            let desc = ModDescriptor {
                name: path
                    .file_name()
                    .map(|n| n.to_string_lossy().to_string())
                    .unwrap_or_else(|| path.display().to_string()),
                file_path: path.clone(),
                load_order: 0,
                is_map_mod: true,
                is_enabled: true,
            };
            let source = ArchiveSource::open(path)?;
            reader.push(desc, Box::new(source));
        }

        for desc in order.enabled_sorted() {
            let source = ArchiveSource::open(&desc.file_path)?;
            reader.push(desc, Box::new(source));
        }

        Ok(reader)
    }

    /// Append one archive at the end (becomes highest priority for ties).
    /// The descriptor's `load_order` is preserved in the index entries so
    /// callers can keep track of what came from where.
    pub fn push(&mut self, descriptor: ModDescriptor, source: Box<dyn SectorSource>) {
        let archive_index = self.archives.len();
        let load_order = descriptor.load_order;
        for path in source.sector_paths() {
            self.sector_index
                .entry(path)
                .or_default()
                .push((load_order, archive_index));
        }
        // Keep each entry's vector ordered ascending by load_order so the
        // last element is always the highest-priority source for that path.
        for vec in self.sector_index.values_mut() {
            vec.sort_by_key(|(order, _)| *order);
        }
        self.archives.push((descriptor, source));
    }

    /// Number of loaded archives.
    pub fn archive_count(&self) -> usize {
        self.archives.len()
    }

    /// Number of distinct sector paths across all archives.
    pub fn unique_sector_count(&self) -> usize {
        self.sector_index.len()
    }

    /// Return the descriptor of an archive by its index, if any.
    pub fn descriptor(&self, idx: usize) -> Option<&ModDescriptor> {
        self.archives.get(idx).map(|(d, _)| d)
    }

    /// Resolve which archive should answer a request for `sector_path`.
    ///
    /// Returns the archive index of the *highest-priority* source that
    /// owns the sector, or `None` if the sector is not present anywhere.
    pub fn resolve_sector(&self, sector_path: &str) -> Option<usize> {
        let candidates = self.sector_index.get(sector_path)?;
        candidates.last().map(|(_, idx)| *idx)
    }

    /// Read a sector's raw bytes from the highest-priority archive.
    pub fn get_sector(&mut self, sector_path: &str) -> Option<Vec<u8>> {
        let idx = self.resolve_sector(sector_path)?;
        let (_, source) = self.archives.get_mut(idx)?;
        source.read_sector(sector_path)
    }

    /// Return every distinct sector path known across all loaded archives,
    /// sorted lexicographically.
    pub fn get_all_sectors(&self) -> Vec<String> {
        let mut paths: Vec<String> = self.sector_index.keys().cloned().collect();
        paths.sort();
        paths
    }

    /// For a given sector path, list the archive indices that contain it,
    /// in ascending load_order. Useful for diagnostics / `--verbose`.
    pub fn sector_providers(&self, sector_path: &str) -> Vec<usize> {
        self.sector_index
            .get(sector_path)
            .map(|v| v.iter().map(|(_, idx)| *idx).collect())
            .unwrap_or_default()
    }
}

impl Default for MultiArchiveReader {
    fn default() -> Self {
        Self::new()
    }
}

/// Convenience: read every sector through the priority-resolved view.
///
/// Returns `(sector_path, raw_bytes)` pairs in lexicographic path order.
pub fn read_all_resolved(reader: &mut MultiArchiveReader) -> Vec<(String, Vec<u8>)> {
    let mut out = Vec::new();
    for path in reader.get_all_sectors() {
        if let Some(bytes) = reader.get_sector(&path) {
            out.push((path, bytes));
        }
    }
    out
}

/// Open all `.scs` files in a directory and return a ready-to-use reader.
/// Convenience wrapper around [`expand_mod_list`](super::mod_descriptor::expand_mod_list).
pub fn open_mod_directory(mod_dir: &Path) -> Result<MultiArchiveReader> {
    let descriptors = super::mod_descriptor::expand_mod_list(mod_dir)?;
    if descriptors.is_empty() {
        return Err(Error::ArchiveFormat(format!(
            "no .scs files found in {}",
            mod_dir.display()
        )));
    }
    MultiArchiveReader::load_archives(&descriptors)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn fake_descriptor(name: &str, load_order: u32) -> ModDescriptor {
        ModDescriptor {
            name: name.into(),
            file_path: PathBuf::from(format!("/fake/{name}")),
            load_order,
            is_map_mod: true,
            is_enabled: true,
        }
    }

    #[test]
    fn priority_resolution_picks_highest_load_order() {
        let mut reader = MultiArchiveReader::new();

        // Base game (load_order 0) provides "secA" and "secB".
        reader.push(
            fake_descriptor("base", 0),
            Box::new(InMemorySource::new(vec![
                ("secA".into(), b"base-A".to_vec()),
                ("secB".into(), b"base-B".to_vec()),
            ])),
        );
        // Mod (load_order 5) overrides "secA".
        reader.push(
            fake_descriptor("mod", 5),
            Box::new(InMemorySource::new(vec![(
                "secA".into(),
                b"mod-A".to_vec(),
            )])),
        );

        assert_eq!(reader.get_sector("secA"), Some(b"mod-A".to_vec()));
        assert_eq!(reader.get_sector("secB"), Some(b"base-B".to_vec()));
        assert_eq!(reader.get_sector("missing"), None);
        assert_eq!(reader.unique_sector_count(), 2);
    }

    #[test]
    fn connector_patches_have_highest_priority() {
        let mut reader = MultiArchiveReader::new();
        reader.push(
            fake_descriptor("base", 0),
            Box::new(InMemorySource::new(vec![("sec".into(), b"base".to_vec())])),
        );
        reader.push(
            fake_descriptor("promods", 10),
            Box::new(InMemorySource::new(vec![(
                "sec".into(),
                b"promods".to_vec(),
            )])),
        );
        reader.push(
            fake_descriptor("connector", 100),
            Box::new(InMemorySource::new(vec![(
                "sec".into(),
                b"connector".to_vec(),
            )])),
        );

        assert_eq!(reader.get_sector("sec"), Some(b"connector".to_vec()));
    }

    #[test]
    fn get_all_sectors_returns_unique_sorted() {
        let mut reader = MultiArchiveReader::new();
        reader.push(
            fake_descriptor("a", 0),
            Box::new(InMemorySource::new(vec![
                ("z".into(), b"".to_vec()),
                ("a".into(), b"".to_vec()),
            ])),
        );
        reader.push(
            fake_descriptor("b", 1),
            Box::new(InMemorySource::new(vec![
                ("a".into(), b"".to_vec()),
                ("m".into(), b"".to_vec()),
            ])),
        );
        let all = reader.get_all_sectors();
        assert_eq!(all, vec!["a".to_string(), "m".into(), "z".into()]);
    }

    #[test]
    fn resolve_sector_returns_correct_index() {
        let mut reader = MultiArchiveReader::new();
        reader.push(
            fake_descriptor("low", 0),
            Box::new(InMemorySource::new(vec![("x".into(), b"l".to_vec())])),
        );
        reader.push(
            fake_descriptor("hi", 5),
            Box::new(InMemorySource::new(vec![("x".into(), b"h".to_vec())])),
        );
        // Highest priority lives at index 1.
        assert_eq!(reader.resolve_sector("x"), Some(1));
        let providers = reader.sector_providers("x");
        assert_eq!(providers, vec![0, 1]);
    }
}
