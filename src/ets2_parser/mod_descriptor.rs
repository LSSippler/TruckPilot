//! Mod descriptor types and load-order management.
//!
//! Describes a single ETS2 map mod (`*.scs` archive) and the global ordered
//! list used by [`MultiArchiveReader`] when resolving sectors. The order is
//! significant: archives later in the list override earlier ones, mirroring
//! how ETS2 itself layers mods on top of the base game.

use std::ffi::OsStr;
use std::fs;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::ets2_parser::error::{Error, Result};

/// Description of one mod or base-game archive.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct ModDescriptor {
    /// Human-readable name (defaults to the file name).
    pub name: String,
    /// Absolute path to the `.scs` file.
    pub file_path: PathBuf,
    /// Load priority. Higher values override lower ones.
    pub load_order: u32,
    /// `true` if the archive contains map sectors (vs. pure asset packs).
    #[serde(default = "default_true")]
    pub is_map_mod: bool,
    /// Whether the descriptor participates in resolution.
    #[serde(default = "default_true")]
    pub is_enabled: bool,
}

fn default_true() -> bool {
    true
}

/// Ordered set of descriptors plus the base-game archives they layer on top of.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ModLoadOrder {
    /// All mod descriptors (base game + mods + connectors), in load order.
    pub descriptors: Vec<ModDescriptor>,
    /// Base-game archive paths (typically `base.scs`, `def.scs`, `base_map.scs`).
    pub base_game_paths: Vec<PathBuf>,
    /// Default ordering hint by file name (for reproducible discovery).
    pub default_load_order: Vec<String>,
}

/// JSON file format for `mod_order.json`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ModOrderFile {
    /// Base-game files relative to the game directory (e.g. `["base.scs", "def.scs"]`).
    #[serde(default)]
    pub base_game_files: Vec<String>,
    /// Mod descriptor entries from the JSON file.
    #[serde(default)]
    pub mod_descriptors: Vec<ModOrderEntry>,
}

/// One entry in the JSON `mod_descriptors` array.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ModOrderEntry {
    /// Human-readable name.
    pub name: String,
    /// File name (relative to mod directory) or absolute path.
    pub file: String,
    /// Load priority — higher overrides lower.
    pub order: u32,
    /// Optional flag, default `true`.
    #[serde(default = "default_true")]
    pub is_map_mod: bool,
    /// Optional flag, default `true`.
    #[serde(default = "default_true")]
    pub is_enabled: bool,
}

impl ModDescriptor {
    /// Create a descriptor from a file path with the given load order.
    pub fn from_path(path: PathBuf, load_order: u32) -> Self {
        let name = path
            .file_name()
            .map(|n| n.to_string_lossy().to_string())
            .unwrap_or_else(|| path.display().to_string());
        ModDescriptor {
            name,
            file_path: path,
            load_order,
            is_map_mod: true,
            is_enabled: true,
        }
    }
}

impl ModLoadOrder {
    /// Build a load order by scanning a mod directory for `.scs` files.
    ///
    /// Files are sorted alphabetically (case-insensitive) and assigned
    /// ascending load orders starting from 0. This matches how the game
    /// itself sorts mods when no manual order is provided.
    pub fn from_directory(mod_dir: &Path) -> Result<Self> {
        let descriptors = expand_mod_list(mod_dir)?;
        let default_load_order = descriptors.iter().map(|d| d.name.clone()).collect();
        Ok(ModLoadOrder {
            descriptors,
            base_game_paths: Vec::new(),
            default_load_order,
        })
    }

    /// Load a [`ModLoadOrder`] from a JSON file. Paths in the JSON are
    /// resolved relative to `mod_dir` if they are not absolute, and base
    /// game paths relative to `game_dir`.
    pub fn from_json_file(
        json_path: &Path,
        mod_dir: Option<&Path>,
        game_dir: Option<&Path>,
    ) -> Result<Self> {
        let text = fs::read_to_string(json_path).map_err(Error::Io)?;
        let parsed: ModOrderFile = serde_json::from_str(&text)
            .map_err(|e| Error::ArchiveFormat(format!("invalid mod_order.json: {e}")))?;

        let resolve = |raw: &str, base: Option<&Path>| -> PathBuf {
            let candidate = PathBuf::from(raw);
            if candidate.is_absolute() {
                candidate
            } else if let Some(b) = base {
                b.join(raw)
            } else {
                candidate
            }
        };

        let descriptors: Vec<ModDescriptor> = parsed
            .mod_descriptors
            .iter()
            .map(|e| ModDescriptor {
                name: e.name.clone(),
                file_path: resolve(&e.file, mod_dir),
                load_order: e.order,
                is_map_mod: e.is_map_mod,
                is_enabled: e.is_enabled,
            })
            .collect();

        let base_game_paths: Vec<PathBuf> = parsed
            .base_game_files
            .iter()
            .map(|f| resolve(f, game_dir))
            .collect();

        let default_load_order: Vec<String> = descriptors.iter().map(|d| d.name.clone()).collect();

        Ok(ModLoadOrder {
            descriptors,
            base_game_paths,
            default_load_order,
        })
    }

    /// Return descriptors filtered to enabled ones, sorted ascending by
    /// `load_order`. Sort is stable, ties keep their original order.
    pub fn enabled_sorted(&self) -> Vec<ModDescriptor> {
        let mut list: Vec<ModDescriptor> = self
            .descriptors
            .iter()
            .filter(|d| d.is_enabled)
            .cloned()
            .collect();
        list.sort_by_key(|d| d.load_order);
        list
    }

    /// Append a base-game path to the load order. Returns the new descriptor.
    pub fn push_base_game(&mut self, path: PathBuf) {
        self.base_game_paths.push(path);
    }

    /// Total number of registered descriptors (excluding base-game paths).
    pub fn len(&self) -> usize {
        self.descriptors.len()
    }

    /// `true` if no descriptors are registered.
    pub fn is_empty(&self) -> bool {
        self.descriptors.is_empty()
    }
}

/// Discover all `.scs` files in `mod_dir` and produce sorted descriptors.
///
/// The directory is scanned non-recursively (matches ETS2 `mod` folder
/// behaviour). Names are lowercased for the sort key so the output is
/// deterministic across filesystems with case-insensitive ordering.
pub fn expand_mod_list(mod_dir: &Path) -> Result<Vec<ModDescriptor>> {
    if !mod_dir.exists() {
        return Err(Error::ArchiveFormat(format!(
            "mod directory does not exist: {}",
            mod_dir.display()
        )));
    }
    if !mod_dir.is_dir() {
        return Err(Error::ArchiveFormat(format!(
            "mod path is not a directory: {}",
            mod_dir.display()
        )));
    }

    let mut descriptors: Vec<ModDescriptor> = Vec::new();
    for entry in fs::read_dir(mod_dir).map_err(Error::Io)? {
        let entry = entry.map_err(Error::Io)?;
        let path = entry.path();
        if !path.is_file() {
            continue;
        }
        if path.extension() != Some(OsStr::new("scs")) {
            continue;
        }
        descriptors.push(ModDescriptor::from_path(path, 0));
    }

    descriptors.sort_by_key(|d| d.name.to_lowercase());
    for (i, desc) in descriptors.iter_mut().enumerate() {
        desc.load_order = i as u32;
    }
    Ok(descriptors)
}

/// Discover mods in a directory. Convenience wrapper used by the CLI.
pub fn discover_mods(mod_dir: &Path) -> Vec<ModDescriptor> {
    expand_mod_list(mod_dir).unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{SystemTime, UNIX_EPOCH};

    fn unique_tmp() -> PathBuf {
        let nonce = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_nanos();
        let mut p = std::env::temp_dir();
        p.push(format!("tp_mods_{}_{}", std::process::id(), nonce));
        std::fs::create_dir_all(&p).unwrap();
        p
    }

    #[test]
    fn discover_sorts_alphabetically_and_assigns_orders() {
        let dir = unique_tmp();
        for name in ["zeta.scs", "alpha.scs", "mu.scs", "ignored.txt"] {
            std::fs::write(dir.join(name), b"x").unwrap();
        }

        let mods = expand_mod_list(&dir).unwrap();
        let names: Vec<_> = mods.iter().map(|m| m.name.as_str()).collect();
        assert_eq!(names, vec!["alpha.scs", "mu.scs", "zeta.scs"]);
        assert_eq!(mods[0].load_order, 0);
        assert_eq!(mods[1].load_order, 1);
        assert_eq!(mods[2].load_order, 2);
        std::fs::remove_dir_all(dir).ok();
    }

    #[test]
    fn discover_returns_error_for_missing_directory() {
        let bogus = std::env::temp_dir().join(format!("tp_missing_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&bogus);
        assert!(expand_mod_list(&bogus).is_err());
    }

    #[test]
    fn json_load_resolves_relative_paths() {
        let dir = unique_tmp();
        std::fs::write(dir.join("a.scs"), b"x").unwrap();
        std::fs::write(dir.join("b.scs"), b"x").unwrap();
        let json = r#"{
            "base_game_files": ["base.scs"],
            "mod_descriptors": [
                {"name": "A", "file": "a.scs", "order": 1},
                {"name": "B", "file": "b.scs", "order": 2}
            ]
        }"#;
        let json_path = dir.join("mod_order.json");
        std::fs::write(&json_path, json).unwrap();

        let order = ModLoadOrder::from_json_file(&json_path, Some(&dir), Some(Path::new("/games")))
            .unwrap();
        assert_eq!(order.descriptors.len(), 2);
        assert_eq!(order.descriptors[0].file_path, dir.join("a.scs"));
        assert_eq!(order.base_game_paths[0], PathBuf::from("/games/base.scs"));
        std::fs::remove_dir_all(dir).ok();
    }

    #[test]
    fn enabled_sorted_respects_load_order() {
        let order = ModLoadOrder {
            descriptors: vec![
                ModDescriptor {
                    name: "high".into(),
                    file_path: PathBuf::from("/h"),
                    load_order: 99,
                    is_map_mod: true,
                    is_enabled: true,
                },
                ModDescriptor {
                    name: "off".into(),
                    file_path: PathBuf::from("/o"),
                    load_order: 50,
                    is_map_mod: true,
                    is_enabled: false,
                },
                ModDescriptor {
                    name: "low".into(),
                    file_path: PathBuf::from("/l"),
                    load_order: 1,
                    is_map_mod: true,
                    is_enabled: true,
                },
            ],
            base_game_paths: vec![],
            default_load_order: vec![],
        };
        let sorted = order.enabled_sorted();
        assert_eq!(sorted.len(), 2);
        assert_eq!(sorted[0].name, "low");
        assert_eq!(sorted[1].name, "high");
    }
}
