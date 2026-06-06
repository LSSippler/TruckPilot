//! Prefab-model SII loader — maps `trucklib_token(dot_suffix)` to `.ppd` paths.
//!
//! ETS2 sector binary stores the **TruckLib little-endian base-38 token** of the
//! dot-suffix of the SII unit name (e.g. `"prefab.mod_ger_67"` → suffix `"mod_ger_67"`).
//! This module walks `def/world/` in every SCS archive and parses `prefab_model`
//! and `prefab_corner_model` definitions to build the token→path map.
//!
//! ## Supported SII formats
//!
//! **Text style 1 (legacy):**
//! ```text
//! prefab_model : prefab.unit_suffix {
//!     prefab_desc: "prefab2/.../file.ppd"
//! }
//! ```
//! token = `trucklib_token("unit_suffix")`
//!
//! **Text style 2 (modern namespaced):**
//! ```text
//! prefab_model.unit_suffix : .prefab_model_data {
//!     prefab_desc: "prefab2/.../file.ppd"
//! }
//! ```
//! token = `trucklib_token("unit_suffix")`
//!
//! Same two styles apply to `prefab_corner_model` for roundabout/corner prefabs.
//!
//! **Binary BSII / ScsB / 3nK:** logged with magic bytes and skipped.

use std::collections::HashMap;

use tracing::{debug, info, warn};

use crate::archive::Archive;
use crate::hashfs::parse_directory_listing;
use crate::road_look::trucklib_token;

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Walk every archive for `prefab_model` SII definitions and return raw
/// `(unit_name, ppd_path)` pairs — unhashed, for hash-variant testing.
///
/// Later archives override earlier ones (mod-wins semantics).
pub fn load_prefab_sii_pairs(archives: &mut [Box<dyn Archive>]) -> Vec<(String, String)> {
    let mut merged: std::collections::HashMap<String, String> = std::collections::HashMap::new();
    for archive in archives.iter_mut() {
        let paths = discover_prefab_sii_paths(archive);
        for path in &paths {
            if let Ok(bytes) = archive.read_path(path) {
                if detect_binary_sii(&bytes).is_some() {
                    continue;
                }
                for (unit_name, ppd_path) in parse_prefab_sii_text_pairs(&bytes) {
                    merged.insert(unit_name, ppd_path);
                }
            }
        }
    }
    merged.into_iter().collect()
}

/// Walk every archive for `prefab_model` SII definitions and return a
/// `scs_token_hash(unit_name) → ppd_path` map.
///
/// Later archives override earlier ones (mod-wins semantics).
pub fn load_prefab_sii_defs(archives: &mut [Box<dyn Archive>]) -> HashMap<u64, String> {
    let mut merged: HashMap<u64, String> = HashMap::new();
    let mut files_scanned = 0usize;
    let mut files_binary = 0usize;
    let mut first_binary_magic: Option<[u8; 8]> = None;
    let mut include_queue: Vec<String> = Vec::new();

    for (arc_idx, archive) in archives.iter_mut().enumerate() {
        let paths = discover_prefab_sii_paths(archive);
        if !paths.is_empty() {
            debug!(
                "prefab_sii archive[{}]: {} SII paths discovered",
                arc_idx,
                paths.len()
            );
        }
        for path in &paths {
            match archive.read_path(path) {
                Ok(bytes) => {
                    files_scanned += 1;
                    if let Some(label) = detect_binary_sii(&bytes) {
                        files_binary += 1;
                        let mut m = [0u8; 16];
                        let n = m.len().min(bytes.len());
                        m[..n].copy_from_slice(&bytes[..n]);
                        if first_binary_magic.is_none() {
                            first_binary_magic = Some(m[..8].try_into().unwrap());
                        }
                        debug!(
                            "prefab_sii BINARY '{}' ({}) magic {:02X?}",
                            path,
                            label,
                            &m[..n]
                        );
                        continue;
                    }
                    // Collect @include directives before parsing entries
                    for inc in extract_includes(&bytes) {
                        let stripped = inc.trim_start_matches('/').to_string();
                        if !stripped.is_empty() && !include_queue.contains(&stripped) {
                            include_queue.push(stripped);
                        }
                    }
                    let entries = parse_prefab_sii_text(&bytes);
                    debug!("prefab_sii text '{}' → {} entries", path, entries.len());
                    for (token, ppd_path) in entries {
                        merged.insert(token, ppd_path);
                    }
                }
                Err(e) => {
                    warn!("prefab_sii read_path '{}' failed: {e}", path);
                }
            }
        }
    }

    // Follow @include paths — try each from ALL archives (mod-wins: last writer wins)
    for inc_path in &include_queue {
        let mut found = false;
        for archive in archives.iter_mut() {
            if let Ok(bytes) = archive.read_path(inc_path) {
                if detect_binary_sii(&bytes).is_some() {
                    continue;
                }
                let entries = parse_prefab_sii_text(&bytes);
                if !entries.is_empty() {
                    debug!(
                        "prefab_sii @include '{}' → {} entries",
                        inc_path,
                        entries.len()
                    );
                    found = true;
                }
                files_scanned += 1;
                for (token, ppd_path) in entries {
                    merged.insert(token, ppd_path);
                }
            }
        }
        if !found {
            debug!(
                "prefab_sii @include '{}' → not found in any archive",
                inc_path
            );
        }
    }

    if files_scanned == 0 {
        warn!("prefab_sii: no def/world/prefab*.sii files found in any archive");
    } else if files_binary == files_scanned {
        warn!(
            "prefab_sii: all {} scanned files are binary (magic {:02X?}) — 0 text entries",
            files_scanned,
            first_binary_magic.unwrap_or_default()
        );
    } else {
        info!(
            "prefab_sii: {} files scanned ({} binary skipped), {} unique token→path entries",
            files_scanned,
            files_binary,
            merged.len()
        );
    }

    merged
}

// ---------------------------------------------------------------------------
// Discovery
// ---------------------------------------------------------------------------

fn discover_prefab_sii_paths(archive: &mut Box<dyn Archive>) -> Vec<String> {
    let mut paths: Vec<String> = Vec::new();

    // Primary: directory-listing walk (works for HashFS archives with dir entries)
    if let Ok(bytes) = archive.read_path("def/world") {
        if let Ok(items) = parse_directory_listing(&bytes) {
            for item in items {
                if item.is_dir {
                    continue;
                }
                let lc = item.name.to_ascii_lowercase();
                if lc.starts_with("prefab") && (lc.ends_with(".sii") || lc.ends_with(".sui")) {
                    paths.push(format!("def/world/{}", item.name));
                }
            }
        }
    }
    if let Ok(bytes) = archive.read_path("def/world/prefab") {
        if let Ok(items) = parse_directory_listing(&bytes) {
            for item in items {
                if item.is_dir {
                    continue;
                }
                let lc = item.name.to_ascii_lowercase();
                if lc.ends_with(".sii") || lc.ends_with(".sui") {
                    paths.push(format!("def/world/prefab/{}", item.name));
                }
            }
        }
    }

    // Fallback: list_files() scan for archives without directory entries (e.g. ProMods)
    if paths.is_empty() {
        for file_path in archive.list_files() {
            let lc = file_path.to_ascii_lowercase();
            if (lc.starts_with("def/world/prefab") || lc.starts_with("def/world\\prefab"))
                && (lc.ends_with(".sii") || lc.ends_with(".sui"))
            {
                paths.push(file_path);
            }
        }
    }

    paths
}

// ---------------------------------------------------------------------------
// Format detection
// ---------------------------------------------------------------------------

fn detect_binary_sii(data: &[u8]) -> Option<&'static str> {
    if data.starts_with(b"BSII") || data.starts_with(b"\x49\x49\x42\x53") {
        Some("BSII")
    } else if data.starts_with(b"ScsB") {
        Some("ScsB")
    } else if data.len() >= 3 && &data[..3] == b"3nK" {
        Some("3nK-encrypted")
    } else {
        None
    }
}

/// Extract `@include "path"` directives from a text SII file.
fn extract_includes(data: &[u8]) -> Vec<String> {
    let text = match std::str::from_utf8(data) {
        Ok(s) => s,
        Err(_) => return Vec::new(),
    };
    let mut includes = Vec::new();
    for line in text.lines() {
        let line = line.trim();
        if let Some(rest) = line.strip_prefix("@include") {
            let path = rest.trim().trim_matches('"');
            if !path.is_empty() {
                includes.push(path.to_string());
            }
        }
    }
    includes
}

// ---------------------------------------------------------------------------
// Text SII parser
// ---------------------------------------------------------------------------

/// Extract a prefab model token from a block-header line.
///
/// Handles any known model class (`prefab_model`, `prefab_corner_model`, …):
///
/// - Style 2: `<class>.unit_suffix : .class_data {`  → `trucklib_token(unit_suffix)`
/// - Style 1: `<class> : prefab.unit_suffix {`       → `trucklib_token(unit_suffix)`
fn model_token_from_line(line: &str) -> Option<u64> {
    const STYLE2: &[&str] = &["prefab_model.", "prefab_corner_model."];
    const STYLE1: &[&str] = &["prefab_model", "prefab_corner_model"];

    // Style 2: class.unit_suffix : ...
    for prefix in STYLE2 {
        if let Some(rest) = line.strip_prefix(prefix) {
            let end = rest.find([' ', ':']).unwrap_or(rest.len());
            let suffix = rest[..end].trim();
            if !suffix.is_empty() {
                let dot_suffix = suffix.rsplit('.').next().unwrap_or(suffix);
                return Some(trucklib_token(dot_suffix));
            }
        }
    }
    // Style 1: class : unit_name {
    for prefix in STYLE1 {
        if let Some(rest) = line.strip_prefix(prefix) {
            let rest = rest.trim();
            if let Some(rest) = rest.strip_prefix(':') {
                let name_part = rest.trim();
                let end = name_part.find([' ', '{']).unwrap_or(name_part.len());
                let unit_name = name_part[..end].trim();
                if !unit_name.is_empty() {
                    let dot_suffix = unit_name.rsplit('.').next().unwrap_or(unit_name);
                    return Some(trucklib_token(dot_suffix));
                }
            }
        }
    }
    None
}

/// Parse plain-text SII data, extracting `token → ppd_path` entries for all
/// `prefab_model` / `prefab_corner_model` blocks that contain a `prefab_desc` field.
///
/// Token = `trucklib_token(dot_suffix)` where `dot_suffix` is the part after
/// the last dot in the SII unit name (e.g. `"prefab.mod_ger_67"` → `"mod_ger_67"`).
fn parse_prefab_sii_text(data: &[u8]) -> Vec<(u64, String)> {
    let text = match std::str::from_utf8(data) {
        Ok(s) => s,
        Err(_) => return Vec::new(),
    };

    let mut results: Vec<(u64, String)> = Vec::new();
    let mut in_block = false;
    let mut pending_brace = false;
    let mut current_token: u64 = 0;
    let mut current_ppd: Option<String> = None;

    for raw_line in text.lines() {
        let line = raw_line.trim();

        if line.is_empty() || line.starts_with('#') || line.starts_with("//") {
            continue;
        }

        if pending_brace {
            if line == "{" {
                in_block = true;
            } else {
                current_token = 0;
            }
            pending_brace = false;
            continue;
        }

        if !in_block {
            if let Some(tok) = model_token_from_line(line) {
                current_token = tok;
                current_ppd = None;
                if line.contains('{') {
                    in_block = true;
                } else {
                    pending_brace = true;
                }
            }
        } else {
            if line.starts_with('}') {
                if current_token != 0 {
                    if let Some(path) = current_ppd.take() {
                        results.push((current_token, path));
                    }
                }
                in_block = false;
                current_token = 0;
            } else if let Some((key, val)) = line.split_once(':') {
                if key.trim() == "prefab_desc" {
                    let path = val.trim().trim_matches('"');
                    if path.ends_with(".ppd") {
                        current_ppd = Some(path.to_string());
                    }
                }
            }
        }
    }

    results
}

/// Like `parse_prefab_sii_text` but returns raw `(unit_name, ppd_path)` strings
/// instead of pre-computing the token hash. Used for hash-variant exploration.
fn parse_prefab_sii_text_pairs(data: &[u8]) -> Vec<(String, String)> {
    let text = match std::str::from_utf8(data) {
        Ok(s) => s,
        Err(_) => return Vec::new(),
    };

    let mut results: Vec<(String, String)> = Vec::new();
    let mut in_block = false;
    let mut pending_brace = false;
    let mut current_name: String = String::new();
    let mut current_ppd: Option<String> = None;

    for raw_line in text.lines() {
        let line = raw_line.trim();

        if line.is_empty() || line.starts_with('#') || line.starts_with("//") {
            continue;
        }

        if pending_brace {
            if line == "{" {
                in_block = true;
            } else {
                current_name.clear();
            }
            pending_brace = false;
            continue;
        }

        if !in_block {
            // Reuse the same prefix matching as parse_prefab_sii_text;
            // reconstruct the canonical unit name from the matched prefix.
            {
                const STYLE2: &[&str] = &["prefab_model.", "prefab_corner_model."];
                const STYLE1: &[&str] = &["prefab_model", "prefab_corner_model"];
                let mut matched = false;
                for prefix in STYLE2 {
                    if let Some(rest) = line.strip_prefix(prefix) {
                        let end = rest.find([' ', ':']).unwrap_or(rest.len());
                        let suffix = rest[..end].trim();
                        if !suffix.is_empty() {
                            current_name = format!("{}{suffix}", prefix);
                            current_ppd = None;
                            if line.contains('{') {
                                in_block = true;
                            } else {
                                pending_brace = true;
                            }
                            matched = true;
                            break;
                        }
                    }
                }
                if !matched {
                    for prefix in STYLE1 {
                        if let Some(rest) = line.strip_prefix(prefix) {
                            let rest = rest.trim();
                            if let Some(rest) = rest.strip_prefix(':') {
                                let name_part = rest.trim().trim_start_matches('.');
                                let end = name_part.find([' ', '{']).unwrap_or(name_part.len());
                                let unit_name = name_part[..end].trim();
                                if !unit_name.is_empty() {
                                    current_name = unit_name.to_string();
                                    current_ppd = None;
                                    if line.contains('{') {
                                        in_block = true;
                                    } else {
                                        pending_brace = true;
                                    }
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        } else if line.starts_with('}') {
            if !current_name.is_empty() {
                if let Some(path) = current_ppd.take() {
                    results.push((current_name.clone(), path));
                }
            }
            in_block = false;
            current_name.clear();
        } else if let Some((key, val)) = line.split_once(':') {
            if key.trim() == "prefab_desc" {
                let path = val.trim().trim_matches('"');
                if path.ends_with(".ppd") {
                    current_ppd = Some(path.to_string());
                }
            }
        }
    }

    results
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_legacy_style_block() {
        // Style 1: "prefab_model : bus_station_small" → dot_suffix = "bus_station_small" (no dot)
        let sii = concat!(
            "SiiNunit {\n",
            "prefab_model : bus_station_small {\n",
            "    prefab_desc: \"prefab2/bus/ger_bus_small.ppd\"\n",
            "    look: \"look_a\"\n",
            "}\n",
            "}\n",
        );
        let entries = parse_prefab_sii_text(sii.as_bytes());
        assert_eq!(entries.len(), 1);
        let (tok, path) = &entries[0];
        assert_eq!(*tok, trucklib_token("bus_station_small"));
        assert_eq!(path, "prefab2/bus/ger_bus_small.ppd");
    }

    #[test]
    fn parse_legacy_style_dotted_name() {
        // Style 1: "prefab_model : prefab.mod_ger_67" → dot_suffix = "mod_ger_67"
        let sii = concat!(
            "prefab_model : prefab.mod_ger_67 {\n",
            "    prefab_desc: \"prefab2/cross_temp/ger/test.ppd\"\n",
            "}\n",
        );
        let entries = parse_prefab_sii_text(sii.as_bytes());
        assert_eq!(entries.len(), 1);
        assert_eq!(entries[0].0, trucklib_token("mod_ger_67"));
    }

    #[test]
    fn parse_modern_namespaced_block() {
        // Style 2: "prefab_model.r1_t" → suffix = "r1_t" → dot_suffix = "r1_t"
        let sii = concat!(
            "prefab_model.r1_t {\n",
            "    prefab_desc: \"prefab2/roads/r1_t.ppd\"\n",
            "}\n",
        );
        let entries = parse_prefab_sii_text(sii.as_bytes());
        assert_eq!(entries.len(), 1);
        let (tok, path) = &entries[0];
        assert_eq!(*tok, trucklib_token("r1_t"));
        assert_eq!(path, "prefab2/roads/r1_t.ppd");
    }

    #[test]
    fn parse_multiple_blocks() {
        let sii = concat!(
            "SiiNunit {\n",
            "prefab_model : r2_r3_merge {\n",
            "    prefab_desc: \"prefab2/roads/r2_r3_merge.ppd\"\n",
            "}\n",
            "prefab_model : r3_r4_merge {\n",
            "    prefab_desc: \"prefab2/roads/r3_r4_merge.ppd\"\n",
            "}\n",
            "}\n",
        );
        let entries = parse_prefab_sii_text(sii.as_bytes());
        assert_eq!(entries.len(), 2);
    }

    #[test]
    fn block_without_ppd_is_skipped() {
        let sii = concat!(
            "prefab_model : no_ppd_here {\n",
            "    look: \"look_a\"\n",
            "}\n",
            "prefab_model : has_ppd {\n",
            "    prefab_desc: \"prefab2/test.ppd\"\n",
            "}\n",
        );
        let entries = parse_prefab_sii_text(sii.as_bytes());
        assert_eq!(entries.len(), 1);
        assert_eq!(entries[0].0, trucklib_token("has_ppd"));
    }

    #[test]
    fn binary_bsii_detected() {
        assert_eq!(detect_binary_sii(b"BSII\x00\x01"), Some("BSII"));
        assert_eq!(detect_binary_sii(b"\x49\x49\x42\x53"), Some("BSII"));
        assert_eq!(detect_binary_sii(b"ScsB\x00"), Some("ScsB"));
        assert_eq!(detect_binary_sii(b"3nKsomething"), Some("3nK-encrypted"));
        assert_eq!(detect_binary_sii(b"SiiNunit"), None);
    }

    #[test]
    fn non_utf8_returns_empty() {
        let result = parse_prefab_sii_text(b"\xFF\xFE\x00\x00");
        assert!(result.is_empty());
    }

    #[test]
    fn deferred_brace_style() {
        let sii = concat!(
            "prefab_model : delayed_open\n",
            "{\n",
            "    prefab_desc: \"prefab2/roads/delayed_open.ppd\"\n",
            "}\n",
        );
        let entries = parse_prefab_sii_text(sii.as_bytes());
        assert_eq!(entries.len(), 1);
        assert_eq!(entries[0].0, trucklib_token("delayed_open"));
    }
}
