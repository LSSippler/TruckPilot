//! City-definition loader — maps city unit-names to world coordinates.
//!
//! Scans `def/city.*.sii` and `def/city/*.sui` across all SCS archives.
//! The returned map is keyed by **unit_name** (e.g. `"berlin"`, `"munich"`).
//!
//! ## ETS2 format history
//!
//! **ETS2 ≤ 1.44 (old format)** — all cities in one file `def/city.sii`:
//! ```text
//! SiiNunit {
//! city.data : .berlin {
//!     city_name: "Berlin"
//!     country: .country.germany
//!     position: (-16400.0, 0.0, -3200.0)   # (x, y, z) — y ignored
//!     population: 3644826
//! }
//! }
//! ```
//!
//! **ETS2 1.48+ (new format)** — split into per-city `def/city/<name>.sui`:
//! ```text
//! city_data : city.berlin
//! {
//!     city_name: "Berlin"
//!     country: germany
//!     population: 3650000
//!     map_x_offsets[]: ...   # minimap pixel offsets — NOT world coordinates
//!     map_y_offsets[]: ...
//! }
//! ```
//! **Note:** ETS2 1.48+ removed the `position:` field.  `load_city_sii` will
//! return an empty map for modern ETS2 installations; callers should fall back
//! to `test_cities.toml`.
//!
//! ## Archive merge order
//!
//! `load_city_sii` iterates archives **front-to-back** and merges results, so
//! later archives (mods / DLC) override base-game entries (mod-wins semantics).

use std::collections::HashMap;

use tracing::{debug, info, warn};

use crate::archive::Archive;
use crate::hashfs::parse_directory_listing;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// One city entry parsed from `/def/city.sii`.
#[derive(Debug, Clone)]
pub struct CityEntry {
    /// SII unit name — lowercase ASCII, no dots (e.g. `"berlin"`, `"munich"`).
    pub unit_name: String,
    /// Display name from `city_name:` field (e.g. `"Berlin"`, `"Munich"`).
    pub city_name: String,
    /// Country token reference (e.g. `".country.germany"`).
    pub country: String,
    /// ETS2 world X coordinate in metres.
    pub x: f64,
    /// ETS2 world Z coordinate in metres (positive = south).
    pub z: f64,
}

// ---------------------------------------------------------------------------
// SII parser
// ---------------------------------------------------------------------------

/// Parse plain-text `city.sii` bytes into a `unit_name → CityEntry` map.
///
/// Returns an empty map on any failure so that absent or unreadable city
/// data degrades gracefully (snap falls back to `test_cities.toml`).
pub fn parse_city_sii(data: &[u8]) -> HashMap<String, CityEntry> {
    if data.starts_with(b"BSII") || data.starts_with(b"\x49\x49\x42\x53") {
        eprintln!(
            "[city.sii] binary BSII format detected (magic={:?}) — plain-text parse impossible",
            &data[..4.min(data.len())]
        );
        warn!("city.sii is in binary BSII format — city coordinates unavailable");
        return HashMap::new();
    }

    let text = match std::str::from_utf8(data) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("[city.sii] UTF-8 decode failed: {e}");
            warn!("city.sii UTF-8 decode failed ({e}) — city coordinates unavailable");
            return HashMap::new();
        }
    };

    let map = parse_city_text(text);
    map
}

fn parse_city_text(text: &str) -> HashMap<String, CityEntry> {
    let mut map: HashMap<String, CityEntry> = HashMap::new();
    let mut in_block = false;
    // ETS2 1.48+ splits cities into per-city `.sui` files where the block
    // header (`city_data : city.berlin`) appears on one line and the `{`
    // on the next.  This flag is set while we wait for that opening brace.
    let mut pending_brace = false;
    let mut unit_name = String::new();
    let mut city_name = String::new();
    let mut country = String::new();
    let mut cur_x: Option<f64> = None;
    let mut cur_z: Option<f64> = None;

    for raw_line in text.lines() {
        let line = raw_line.trim();

        if line.is_empty() || line.starts_with('#') || line.starts_with("//") {
            continue;
        }

        // ── waiting for opening brace (new-format header already parsed) ──────
        if pending_brace {
            if line == "{" {
                in_block = true;
            } else {
                // unexpected line before '{' — abandon this entry
                unit_name.clear();
            }
            pending_brace = false;
            continue;
        }

        if !in_block {
            // Old format:   city.data : .berlin {
            if let Some(rest) = line.strip_prefix("city.data :") {
                let rest = rest.trim();
                if let Some(rest) = rest.strip_prefix('.') {
                    let end = rest.find([' ', '{']).unwrap_or(rest.len());
                    let name = rest[..end].trim();
                    if !name.is_empty() {
                        unit_name = name.to_string();
                        city_name = String::new();
                        country = String::new();
                        cur_x = None;
                        cur_z = None;
                        in_block = true;
                        debug!(unit = %unit_name, "city.sii block start (old fmt)");
                    }
                }
            }
            // New format (ETS2 1.48+):  city_data : city.berlin
            //   Brace is on the NEXT line.
            else if let Some(rest) = line.strip_prefix("city_data :") {
                let rest = rest.trim();
                // Strip "city." prefix to get the bare unit name.
                let base = rest.strip_prefix("city.").unwrap_or(rest);
                let end = base.find([' ', '{']).unwrap_or(base.len());
                let name = base[..end].trim();
                if !name.is_empty() {
                    unit_name = name.to_string();
                    city_name = String::new();
                    country = String::new();
                    cur_x = None;
                    cur_z = None;
                    if line.contains('{') {
                        in_block = true;
                        debug!(unit = %unit_name, "city.sii block start (new fmt, inline brace)");
                    } else {
                        pending_brace = true;
                        debug!(unit = %unit_name, "city.sii block start (new fmt, deferred brace)");
                    }
                }
            }
        } else {
            if line == "}" {
                if let (Some(x), Some(z)) = (cur_x, cur_z) {
                    debug!(unit = %unit_name, city = %city_name, x, z, "city.sii block end");
                    map.insert(
                        unit_name.clone(),
                        CityEntry { unit_name: unit_name.clone(), city_name: city_name.clone(), country: country.clone(), x, z },
                    );
                } else {
                    warn!("city.sii block '{}' has no position — skipped", unit_name);
                }
                in_block = false;
            } else if let Some((key, val)) = line.split_once(':') {
                // Accept both `key: value` and `key : value` formats.
                let val = val.trim();
                match key.trim() {
                    "city_name" => city_name = val.trim_matches('"').to_string(),
                    "country" => country = val.to_string(),
                    "position" => {
                        if let Some((x, z)) = parse_position_xz(val) {
                            cur_x = Some(x);
                            cur_z = Some(z);
                        } else {
                            warn!("city.sii block '{}': could not parse position '{val}'", unit_name);
                        }
                    }
                    _ => {} // other fields (population, map_x_offsets, …) are ignored
                }
            }
        }
    }

    // Handle an unclosed block at EOF (malformed file — save if coords present).
    if in_block {
        if let (Some(x), Some(z)) = (cur_x, cur_z) {
            map.insert(unit_name.clone(), CityEntry { unit_name, city_name, country, x, z });
        }
    }

    info!("city.sii: {} entries parsed", map.len());
    map
}

/// Parse `(x, y, z)` ETS2 position tuple, discarding y (height). Returns `(x, z)`.
///
/// Accepts `(-16400.0, 0.0, -3200.0)` with optional surrounding whitespace.
fn parse_position_xz(s: &str) -> Option<(f64, f64)> {
    let inner = s.trim().trim_start_matches('(').trim_end_matches(')');
    let mut parts = inner.splitn(3, ',');
    let x: f64 = parts.next()?.trim().parse().ok()?;
    let _y: f64 = parts.next()?.trim().parse().ok()?; // height — discarded
    let z: f64 = parts.next()?.trim().parse().ok()?;
    Some((x, z))
}

// ---------------------------------------------------------------------------
// Archive loader
// ---------------------------------------------------------------------------

/// Discover all parseable city definition paths inside a single archive.
///
/// Two levels of discovery:
/// 1. `def/city.*.sii` — per-DLC namespace files (modern ETS2 uses `@include`
///    inside these to point to individual `.sui` files — we skip them as stubs).
/// 2. `def/city/*.sui` — individual city entries; this is where the actual
///    `city.data : .<name> { position: ... }` blocks live in ETS2 1.48+.
///
/// Falls back to an empty list for ZIP archives or if the directories are absent.
fn discover_city_sii_paths(archive: &mut Box<dyn Archive>) -> Vec<String> {
    let mut paths: Vec<String> = Vec::new();

    // Level 1: def/city.*.sii files (legacy/compat stubs — still try them).
    if let Ok(bytes) = archive.read_path("def") {
        if let Ok(items) = parse_directory_listing(&bytes) {
            for item in items {
                if !item.is_dir
                    && item.name.starts_with("city.")
                    && item.name.ends_with(".sii")
                {
                    paths.push(format!("def/{}", item.name));
                }
            }
        }
    }

    // Level 2: def/city/*.sui — individual city data files (ETS2 1.48+ style).
    if let Ok(bytes) = archive.read_path("def/city") {
        if let Ok(items) = parse_directory_listing(&bytes) {
            for item in items {
                if !item.is_dir && item.name.ends_with(".sui") {
                    paths.push(format!("def/city/{}", item.name));
                }
            }
        }
    }

    paths
}

/// Load city definitions from the given archive slice.
///
/// Iterates all archives front-to-back, discovering every `def/city.*.sii`
/// file via the HashFS directory listing.  Results from later archives
/// override earlier ones (mod-wins semantics, same as ETS2 load order).
/// The stub file `def/city.sii` (an empty namespace marker with no blocks)
/// is parsed but expected to yield 0 entries and is silently skipped.
///
/// Diagnostic lines go to stderr so they are visible in release builds
/// even without a tracing subscriber.
pub fn load_city_sii(archives: &mut [Box<dyn Archive>]) -> HashMap<String, CityEntry> {
    eprintln!("[city.sii] scanning {} archive(s) for city definitions", archives.len());
    let mut merged: HashMap<String, CityEntry> = HashMap::new();
    let mut any_found = false;

    for archive in archives.iter_mut() {
        let arc_label = archive.path().file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default();

        let paths = discover_city_sii_paths(archive);
        for path in &paths {
            match archive.read_path(path) {
                Ok(bytes) => {
                    any_found = true;
                    info!("city.sii: loading '{}' from {} ({} bytes)", path, arc_label, bytes.len());
                    let entries = parse_city_sii(&bytes);
                    if entries.is_empty() {
                        eprintln!("[city.sii] '{}' ({}): 0 entries (stub or BSII) — skipped", path, arc_label);
                    } else {
                        eprintln!("[city.sii] '{}' ({}): {} entries", path, arc_label, entries.len());
                        merged.extend(entries);
                    }
                }
                Err(_) => {}
            }
        }
    }

    if !any_found {
        eprintln!("[city.sii] no def/city.*.sii files found in any archive — city coordinates unavailable");
        warn!("def/city.sii not found in any archive — city coordinates unavailable");
    } else if merged.is_empty() {
        eprintln!("[city.sii] files found but all yielded 0 entries (BSII format?) — city coordinates unavailable");
        warn!("city.sii: all candidate files yielded 0 entries — city coordinates unavailable");
    } else {
        eprintln!("[city.sii] {} total city entries loaded", merged.len());
        info!("city.sii: {} total entries after archive merge", merged.len());
    }

    merged
}

// ---------------------------------------------------------------------------
// Name-matching helpers
// ---------------------------------------------------------------------------

/// Normalize a city name for fuzzy lookup: lowercase + keep only alphanumeric chars.
///
/// Example: `"New York"` → `"newyork"`, `"München"` → `"münchen"`.
pub fn normalize_name(name: &str) -> String {
    name.to_lowercase()
        .chars()
        .filter(|c| c.is_alphanumeric())
        .collect()
}

/// Build a secondary index **normalized display-name → unit_name** for fuzzy matching.
///
/// Enables looking up `"berlin"` (from `city_name: "Berlin"`) even when the
/// caller only has a display name (as in `test_cities.toml`).
pub fn build_display_name_index(map: &HashMap<String, CityEntry>) -> HashMap<String, String> {
    map.values()
        .map(|e| (normalize_name(&e.city_name), e.unit_name.clone()))
        .collect()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_single_city() {
        let sii = concat!(
            "SiiNunit {\n",
            "city.data : .berlin {\n",
            "    city_name: \"Berlin\"\n",
            "    city_name_upper: \"BERLIN\"\n",
            "    country: .country.germany\n",
            "    position: (-16400.0, 0.0, -3200.0)\n",
            "    population: 3644826\n",
            "}\n",
            "}\n",
        );
        let map = parse_city_text(sii);
        assert_eq!(map.len(), 1);
        let e = map.get("berlin").expect("berlin entry must exist");
        assert_eq!(e.unit_name, "berlin");
        assert_eq!(e.city_name, "Berlin");
        assert_eq!(e.country, ".country.germany");
        assert!((e.x - (-16400.0)).abs() < 0.1, "x mismatch: {}", e.x);
        assert!((e.z - (-3200.0)).abs() < 0.1, "z mismatch: {}", e.z);
    }

    #[test]
    fn parse_multiple_cities() {
        let sii = concat!(
            "SiiNunit {\n",
            "city.data : .berlin {\n",
            "    city_name: \"Berlin\"\n",
            "    country: .country.germany\n",
            "    position: (-16400.0, 0.0, -3200.0)\n",
            "}\n",
            "city.data : .hamburg {\n",
            "    city_name: \"Hamburg\"\n",
            "    country: .country.germany\n",
            "    position: (-22300.0, 0.0, -7200.0)\n",
            "}\n",
            "}\n",
        );
        let map = parse_city_text(sii);
        assert_eq!(map.len(), 2);
        let h = map.get("hamburg").expect("hamburg must exist");
        assert!((h.x - (-22300.0)).abs() < 0.1);
        assert!((h.z - (-7200.0)).abs() < 0.1);
    }

    #[test]
    fn position_y_is_ignored() {
        let sii = concat!(
            "SiiNunit {\n",
            "city.data : .test {\n",
            "    city_name: \"Test\"\n",
            "    country: .country.test\n",
            "    position: (1234.5, 99.9, -5678.0)\n",
            "}\n",
            "}\n",
        );
        let map = parse_city_text(sii);
        let e = map.get("test").unwrap();
        assert!((e.x - 1234.5).abs() < 0.1);
        assert!((e.z - (-5678.0)).abs() < 0.1);
    }

    #[test]
    fn comments_and_blanks_skipped() {
        let sii = concat!(
            "# top comment\n",
            "SiiNunit {\n",
            "// another comment\n",
            "city.data : .berlin {\n",
            "    # inline\n",
            "    city_name: \"Berlin\"\n",
            "    country: .country.germany\n",
            "    position: (-16400.0, 0.0, -3200.0)\n",
            "}\n",
            "}\n",
        );
        assert_eq!(parse_city_text(sii).len(), 1);
    }

    #[test]
    fn bsii_magic_returns_empty() {
        assert!(parse_city_sii(b"BSII\x00\x01\x02\x03").is_empty());
    }

    #[test]
    fn non_utf8_returns_empty() {
        assert!(parse_city_sii(b"\xFF\xFE\x00\x00").is_empty());
    }

    #[test]
    fn missing_position_entry_skipped() {
        let sii = concat!(
            "SiiNunit {\n",
            "city.data : .nopos {\n",
            "    city_name: \"NoPos\"\n",
            "    country: .country.test\n",
            "}\n",
            "city.data : .withpos {\n",
            "    city_name: \"WithPos\"\n",
            "    country: .country.test\n",
            "    position: (100.0, 0.0, 200.0)\n",
            "}\n",
            "}\n",
        );
        let map = parse_city_text(sii);
        assert_eq!(map.len(), 1, "only withpos should be present");
        assert!(map.contains_key("withpos"));
    }

    #[test]
    fn normalize_name_lowercases_and_strips_spaces() {
        assert_eq!(normalize_name("New York"), "newyork");
        assert_eq!(normalize_name("Berlin"), "berlin");
        assert_eq!(normalize_name("München"), "münchen");
    }

    #[test]
    fn display_name_index_lookup() {
        let sii = concat!(
            "SiiNunit {\n",
            "city.data : .berlin {\n",
            "    city_name: \"Berlin\"\n",
            "    country: .country.germany\n",
            "    position: (-16400.0, 0.0, -3200.0)\n",
            "}\n",
            "}\n",
        );
        let map = parse_city_text(sii);
        let idx = build_display_name_index(&map);
        assert_eq!(idx.get("berlin").map(|s| s.as_str()), Some("berlin"));
    }
}
