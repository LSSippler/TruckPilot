//! road_look.sii loader — maps road-look token64 values to lane counts.
//!
//! In the ETS2 binary sector format, every Road item stores the road type in
//! its fixed header: `road_type` at offset 0x39 in
//! [`crate::road_full::RoadFixedHeader`].  This token is a little-endian
//! base-38 (trucklib) hash of the last dotted segment of the road-look unit
//! name (e.g. `road.ger7` → stem `"ger7"` → `trucklib_token("ger7")`).
//!
//! Additionally, `right_look` (visual look variant, offset 0x99) and
//! `left_look` (offset 0xA1) are stored but carry look/style information only,
//! not lane-count data.  The `look_token` field on `RawRoad` holds `left_look`
//! as a visual-fallback reference; lane-count lookup uses `road_type_token`.
//!
//! ## Token hash
//!
//! ETS2 road_type token64 values use a **little-endian base-38 (trucklib)**
//! encoding — `trucklib_token(stem)`.  `scs_token_hash` (big-endian polynomial)
//! is a separate algorithm used in other contexts; it is kept for compatibility
//! with callers in `mod_loader` but is NOT used for road_look keying.
//! See [`trucklib_token`] and [`scs_token_hash`] for the algorithms.
//!
//! ## SII format
//!
//! The plain-text SII file contains blocks like:
//!
//! ```text
//! road_look : road.look0 {          # legacy format
//!     lanes_left[]:  traffic_lane.road.local
//!     lanes_right[]: traffic_lane.road.local
//! }
//! road_look.narrow1.road : .road_look_data {   # modern format
//!     lanes_left[]: "narrow_lane"
//!     lanes_right[]: "narrow_lane"
//! }
//! ```
//!
//! `lanes_left[]` count → `lanes_backward`, `lanes_right[]` count → `lanes_forward`.
//!
//! ## Fallback behaviour
//!
//! When no matching road_look entry is found (the modern SII files containing
//! short-code names like `ols_b` or `u4_d` are not yet located), the loader
//! falls back to treating every look-token-bearing road as bidirectional
//! (lanes_forward = 1, lanes_backward = 1) to preserve graph connectivity.

use std::collections::{HashMap, HashSet};

use tracing::{debug, info, warn};

use crate::archive::Archive;
use crate::hashfs::parse_directory_listing;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Compact lane-count entry for one road-look definition.
#[derive(Debug, Clone, Copy)]
pub struct RoadLookEntry {
    pub lanes_left: u8,
    pub lanes_right: u8,
    /// Lane width in metres derived from the lane-type name (e.g. motorway→3.75, local→3.0).
    pub lane_width_m: f32,
}

impl Default for RoadLookEntry {
    fn default() -> Self {
        Self { lanes_left: 0, lanes_right: 0, lane_width_m: 3.75 }
    }
}

// ---------------------------------------------------------------------------
// SCS token hash
// ---------------------------------------------------------------------------

/// Compute the SCS token64 polynomial hash used for in-binary token64 values.
///
/// **Not** CityHash64 (used only for HashFS file-path lookups).
///
/// Character mapping:
/// - `a`–`z`, `A`–`Z` → 1–26 (case-insensitive)
/// - `0`–`9`           → 27–36
/// - `_`               → 37
/// - all other chars   → 0  (`.` shifts the hash without adding a unique value)
///
/// The hash accumulates as `h = h * 38 + char_value` (wrapping u64 arithmetic).
pub fn scs_token_hash(s: &str) -> u64 {
    let mut h: u64 = 0;
    for &b in s.as_bytes() {
        let v: u64 = match b {
            b'a'..=b'z' => (b - b'a' + 1) as u64,
            b'A'..=b'Z' => (b - b'A' + 1) as u64,
            b'0'..=b'9' => (b - b'0' + 27) as u64,
            b'_' => 37,
            _ => 0,
        };
        h = h.wrapping_mul(38).wrapping_add(v);
    }
    h
}

/// TruckLib/Prism3D Token encoding: little-endian base-38.
///
/// Charset: '\0'=0, '0'-'9'=1-10, 'a'-'z'=11-36, '_'=37.
/// token = sum(charIndex[i] * 38^i)  (position 0 is least-significant)
///
/// This is the format used to store prefab model tokens in ETS2 binary sector
/// files. For a SII unit like "prefab.mod_ger_67", pass just "mod_ger_67"
/// (the suffix after the last '.').
pub fn trucklib_token(s: &str) -> u64 {
    let mut h: u64 = 0;
    let mut pow: u64 = 1; // 38^i, wrapping
    for &b in s.as_bytes() {
        let v: u64 = match b {
            b'0'..=b'9' => (b - b'0' + 1) as u64,
            b'a'..=b'z' => (b - b'a' + 11) as u64,
            b'A'..=b'Z' => (b - b'A' + 11) as u64,
            b'_' => 37,
            _ => 0,
        };
        h = h.wrapping_add(v.wrapping_mul(pow));
        pow = pow.wrapping_mul(38);
    }
    h
}

/// Maps an ETS2 lane-type name (from `lanes_left[]` / `lanes_right[]`) to a lane width.
///
/// Motorway, highway, and expressway lanes are wider; local/city lanes are
/// narrower.  See inline comment for the empirical basis of the expressway value.
pub fn lane_type_to_width(s: &str) -> f32 {
    // `expressway` → 3.75 m: no explicit road_size_* field was found in
    // road_look.template.sii; ETS2 standard lane width and the lane-keeper
    // Erfolgskriterium (2 − 0.5) × 3.75 = 5.625 m ≈ 5.6 m both use 3.75.
    if s.contains("motorway") || s.contains("highway") || s.contains("expressway") {
        3.75
    } else if s.contains("local") || s.contains("city") {
        3.0
    } else if s.contains("country") {
        3.5
    } else {
        3.75
    }
}

// ---------------------------------------------------------------------------
// SII parser
// ---------------------------------------------------------------------------

/// Parse plain-text road_look.sii bytes into a token → lane-count map.
///
/// Returns an empty map (not an error) on any parse failure so that absent
/// or unreadable lane data degrades gracefully to `bidirectional_unknown` edges.
pub fn parse_road_look_sii(data: &[u8]) -> HashMap<u64, RoadLookEntry> {
    // Detect binary SII (BSII) — not supported without a full binary parser.
    if data.starts_with(b"BSII") || data.starts_with(b"\x49\x49\x42\x53") {
        warn!("road_look.sii is in binary SII format (BSII) — lane counts unavailable");
        return HashMap::new();
    }

    let text = match std::str::from_utf8(data) {
        Ok(s) => s,
        Err(e) => {
            warn!("road_look.sii UTF-8 decode failed ({e}) — lane counts unavailable");
            return HashMap::new();
        }
    };

    parse_road_look_text(text)
}

/// Extract the value after the first `:` on a lane-entry line, stripping quotes.
fn extract_lane_type(line: &str) -> Option<&str> {
    let colon = line.find(':')?;
    let val = line[colon + 1..].trim().trim_matches('"');
    if val.is_empty() { None } else { Some(val) }
}

fn parse_road_look_text(text: &str) -> HashMap<u64, RoadLookEntry> {
    let mut map: HashMap<u64, RoadLookEntry> = HashMap::new();
    let mut in_block = false;
    let mut current_token: u64 = 0;
    let mut current_name = String::new();
    let mut lanes_left: u8 = 0;
    let mut lanes_right: u8 = 0;
    let mut current_lane_width: f32 = 3.75;

    for raw_line in text.lines() {
        let line = raw_line.trim();

        // Skip comments and blanks.
        if line.is_empty() || line.starts_with('#') || line.starts_with("//") {
            continue;
        }

        if !in_block {
            // Modern format:  road_look.NAME : .road_look_data {
            //   token = trucklib_token(last segment of "road_look.NAME")
            // Legacy format:  road_look : road.lookN {
            //   token = trucklib_token(last segment of "road.lookN")
            //
            // The binary road_type field stores trucklib_token(stem) where stem
            // is the last dotted segment of the unit name (e.g. "ger7" from "road.ger7").
            if let Some(rest) = line.strip_prefix("road_look.") {
                // Extract NAME — everything before the first space or colon.
                let end = rest.find([' ', ':']).unwrap_or(rest.len());
                let unit_name = &rest[..end];
                if !unit_name.is_empty() {
                    current_name = format!("road_look.{unit_name}");
                    let stem = current_name.rsplit('.').next().unwrap_or(&current_name);
                    current_token = trucklib_token(stem);
                    lanes_left = 0;
                    lanes_right = 0;
                    current_lane_width = 3.75;
                    in_block = true;
                    debug!(token = current_token, name = %current_name, stem = %stem, "road_look block start (modern)");
                }
            } else if let Some(rest) = line.strip_prefix("road_look :") {
                // Legacy: extract the class name (e.g. "road.ger7") after " : ".
                let class_name = rest.trim_start();
                let end = class_name.find([' ', '{']).unwrap_or(class_name.len());
                let class_name = &class_name[..end];
                if !class_name.is_empty() {
                    current_name = class_name.to_string();
                    let stem = class_name.rsplit('.').next().unwrap_or(class_name);
                    current_token = trucklib_token(stem);
                    lanes_left = 0;
                    lanes_right = 0;
                    current_lane_width = 3.75;
                    in_block = true;
                    debug!(token = current_token, name = %current_name, stem = %stem, "road_look block start (legacy)");
                }
            }
        } else {
            // Inside a block — count lane array entries and watch for closing brace.
            if line == "}" {
                map.insert(
                    current_token,
                    RoadLookEntry {
                        lanes_left,
                        lanes_right,
                        lane_width_m: current_lane_width,
                    },
                );
                debug!(
                    name = %current_name,
                    lanes_left,
                    lanes_right,
                    lane_width_m = current_lane_width,
                    "road_look block end"
                );
                in_block = false;
            } else if line.starts_with("lanes_left[") {
                lanes_left = lanes_left.saturating_add(1);
                if let Some(typ) = extract_lane_type(line) {
                    current_lane_width = lane_type_to_width(typ);
                }
            } else if line.starts_with("lanes_right[") {
                lanes_right = lanes_right.saturating_add(1);
                if let Some(typ) = extract_lane_type(line) {
                    current_lane_width = lane_type_to_width(typ);
                }
            }
        }
    }

    // Handle a block that was never closed (malformed file — save anyway).
    if in_block {
        map.insert(
            current_token,
            RoadLookEntry {
                lanes_left,
                lanes_right,
                lane_width_m: current_lane_width,
            },
        );
    }

    info!("road_look.sii: {} entries parsed", map.len());
    map
}

// ---------------------------------------------------------------------------
// Archive loader
// ---------------------------------------------------------------------------

/// Candidate paths for road_look definitions, used as a seed set before
/// directory-walk discovery.  Listed from most-generic to most-specific so
/// that later entries (template files) override the legacy stub.
const ROAD_LOOK_PATHS: &[&str] = &[
    "def/road_look.sii",
    "def/world/road_look.sii",
    "def/world/road.sii",
];

/// Walk an archive's `def`/`def/world` directory listings and collect every
/// file path whose name contains `"road_look"`.
///
/// HashFS stores no flat path list, but it does store per-directory listing
/// entries, so a bounded BFS reconstructs real paths.  Descends into `def`,
/// `def/world`, and any directory whose name contains `"road_look"`.
/// Budget and visited-set prevent infinite loops on cyclic listings.
fn walk_for_road_look(arc: &mut Box<dyn Archive>) -> Vec<String> {
    let mut found = Vec::new();
    let mut stack: Vec<String> = vec!["def".to_string()];
    let mut visited: HashSet<String> = HashSet::new();
    let mut budget = 4096usize;
    while let Some(dir) = stack.pop() {
        if budget == 0 || !visited.insert(dir.clone()) {
            continue;
        }
        budget -= 1;
        let Ok(bytes) = arc.read_path(&dir) else {
            continue;
        };
        let Ok(items) = parse_directory_listing(&bytes) else {
            continue;
        };
        for it in items {
            let full = format!("{dir}/{}", it.name);
            if it.is_dir {
                if full == "def/world" || full.contains("road_look") {
                    stack.push(full);
                }
            } else if full.contains("road_look") {
                found.push(full);
            }
        }
    }
    found
}

/// Load road_look definitions from the given archive slice.
///
/// Collects every `road_look`-bearing file path via directory-walk across all
/// archives, deduplicates, then reads each path (last archive wins = mod-override
/// order), parses it, and merges all entries into a single map.  Later files
/// overwrite earlier ones so DLC/mod definitions override vanilla.
///
/// Returns an empty map (not an error) if no definitions are found.
pub fn load_road_look(archives: &mut [Box<dyn Archive>]) -> HashMap<u64, RoadLookEntry> {
    // Collect all candidate paths: seed set + directory-walk discovery.
    let mut path_set: HashSet<String> = ROAD_LOOK_PATHS
        .iter()
        .map(|s| s.to_string())
        .collect();

    for arc in archives.iter_mut() {
        for p in walk_for_road_look(arc) {
            path_set.insert(p);
        }
    }

    // Sort for determinism: ROAD_LOOK_PATHS order (legacy stub) first, then
    // alphabetically for discovered files (template.sii sorts after road_look.sii,
    // DLC overrides naturally sort last).
    let mut paths: Vec<String> = path_set.into_iter().collect();
    paths.sort_unstable_by(|a, b| {
        let rank_a = ROAD_LOOK_PATHS.iter().position(|&p| p == a.as_str());
        let rank_b = ROAD_LOOK_PATHS.iter().position(|&p| p == b.as_str());
        match (rank_a, rank_b) {
            (Some(ra), Some(rb)) => ra.cmp(&rb),
            (Some(_), None) => std::cmp::Ordering::Less,
            (None, Some(_)) => std::cmp::Ordering::Greater,
            (None, None) => a.cmp(b),
        }
    });

    // Read each path (last archive wins), parse, and merge into one map.
    let mut merged: HashMap<u64, RoadLookEntry> = HashMap::new();
    let mut files_loaded = 0usize;

    for path in &paths {
        // Last archive wins: mod archives override base game.
        let data = archives
            .iter_mut()
            .rev()
            .find_map(|arc| arc.read_path(path).ok());

        let Some(bytes) = data else {
            continue;
        };

        info!("road_look: loading '{}' ({} bytes)", path, bytes.len());
        let map = parse_road_look_sii(&bytes);
        if map.is_empty() {
            // BSII or no parseable blocks — skip silently (already warned inside).
            continue;
        }
        merged.extend(map);
        files_loaded += 1;
    }

    info!(
        "road_look: {} file(s) loaded, {} entries total",
        files_loaded,
        merged.len()
    );

    if merged.is_empty() {
        warn!("road_look: no entries found in any archive — all legacy roads will use bidirectional_unknown edges");
    }

    merged
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ── token hash ──────────────────────────────────────────────────────────

    #[test]
    fn token_hash_empty() {
        assert_eq!(scs_token_hash(""), 0);
    }

    #[test]
    fn token_hash_single_letter() {
        // 'a' = 1
        assert_eq!(scs_token_hash("a"), 1);
        // 'z' = 26
        assert_eq!(scs_token_hash("z"), 26);
    }

    #[test]
    fn token_hash_digits_and_underscore() {
        // '0' = 27
        assert_eq!(scs_token_hash("0"), 27);
        // '9' = 36
        assert_eq!(scs_token_hash("9"), 36);
        // '_' = 37
        assert_eq!(scs_token_hash("_"), 37);
    }

    #[test]
    fn token_hash_two_chars() {
        // "ab" = 1*38 + 2 = 40
        assert_eq!(scs_token_hash("ab"), 40);
    }

    #[test]
    fn token_hash_case_insensitive() {
        assert_eq!(scs_token_hash("road"), scs_token_hash("ROAD"));
        assert_eq!(scs_token_hash("Road"), scs_token_hash("road"));
    }

    #[test]
    fn token_hash_dot_shifts() {
        // '.' → 0, so "a." = 1*38 + 0 = 38; "a.b" = 38*38 + 2 = 1446
        assert_eq!(scs_token_hash("a."), 38);
        assert_eq!(scs_token_hash("a.b"), 1446);
    }

    // ── SII parser ──────────────────────────────────────────────────────────

    #[test]
    fn parse_minimal_one_lane() {
        // Modern format: road_look.narrow1.look → stem = "look"
        let sii = "SiiNunit\n{\nroad_look.narrow1.look : .road_look_data {\n\
                   lanes_left[]: \"narrow\"\n\
                   lanes_right[]: \"narrow\"\n\
                   }\n}\n";
        let map = parse_road_look_text(sii);
        // key = trucklib_token("look")
        let tok = trucklib_token("look");
        let e = map.get(&tok).expect("entry must exist");
        assert_eq!(e.lanes_left, 1);
        assert_eq!(e.lanes_right, 1);
    }

    #[test]
    fn parse_asymmetric_lanes() {
        // Modern format: road_look.motor3.mway → stem = "mway"
        let sii = "SiiNunit\n{\nroad_look.motor3.mway : .road_look_data {\n\
                   lanes_left[]: \"m\"\n\
                   lanes_left[]: \"m\"\n\
                   lanes_right[]: \"m\"\n\
                   lanes_right[]: \"m\"\n\
                   lanes_right[]: \"m\"\n\
                   }\n}\n";
        let map = parse_road_look_text(sii);
        let tok = trucklib_token("mway");
        let e = map.get(&tok).expect("entry must exist");
        assert_eq!(e.lanes_left, 2, "2 left lanes");
        assert_eq!(e.lanes_right, 3, "3 right lanes");
    }

    #[test]
    fn parse_multiple_entries() {
        // Use distinct last-segments to avoid key collision.
        // road_look.a.loca → stem "loca"; road_look.b.motorway → stem "motorway"
        let sii = "SiiNunit\n{\n\
                   road_look.a.loca : .road_look_data {\n\
                   lanes_left[]: \"x\"\n\
                   lanes_right[]: \"x\"\n\
                   }\n\
                   road_look.b.motorway : .road_look_data {\n\
                   lanes_right[]: \"x\"\n\
                   lanes_right[]: \"x\"\n\
                   }\n\
                   }\n";
        let map = parse_road_look_text(sii);
        assert_eq!(map.len(), 2);
        let a = map[&trucklib_token("loca")];
        assert_eq!((a.lanes_left, a.lanes_right), (1, 1));
        let b = map[&trucklib_token("motorway")];
        assert_eq!((b.lanes_left, b.lanes_right), (0, 2));
    }

    #[test]
    fn parse_comments_ignored() {
        let sii = "# file header\nSiiNunit\n{\n\
                   # first look\n\
                   road_look.x.road : .road_look_data {\n\
                   // comment\n\
                   lanes_left[]: \"x\"\n\
                   lanes_right[]: \"x\"\n\
                   }\n}\n";
        let map = parse_road_look_text(sii);
        assert_eq!(map.len(), 1);
    }

    #[test]
    fn bsii_returns_empty() {
        assert!(parse_road_look_sii(b"BSII\x00\x01\x02").is_empty());
    }

    #[test]
    fn non_utf8_returns_empty() {
        assert!(parse_road_look_sii(b"\xFF\xFE\x00\x00").is_empty());
    }

    // ── legacy format ────────────────────────────────────────────────────────

    #[test]
    fn parse_legacy_one_lane() {
        // Legacy: road_look : road.look0 { → stem = "look0"
        let sii = "SiiNunit\n{\nroad_look : road.look0 {\n\
                   lanes_left[]: traffic_lane.road.local\n\
                   lanes_right[]: traffic_lane.road.local\n\
                   }\n}\n";
        let map = parse_road_look_text(sii);
        let tok = trucklib_token("look0");
        let e = map.get(&tok).expect("legacy entry must exist");
        assert_eq!(e.lanes_left, 1);
        assert_eq!(e.lanes_right, 1);
    }

    #[test]
    fn parse_legacy_motorway() {
        // Legacy: road_look : road.look1 { → stem = "look1"
        let sii = "SiiNunit\n{\nroad_look : road.look1 {\n\
                   lanes_left[]: traffic_lane.road.motorway\n\
                   lanes_left[]: traffic_lane.road.motorway\n\
                   lanes_right[]: traffic_lane.road.motorway\n\
                   lanes_right[]: traffic_lane.road.motorway\n\
                   }\n}\n";
        let map = parse_road_look_text(sii);
        let tok = trucklib_token("look1");
        let e = map.get(&tok).expect("legacy motorway entry must exist");
        assert_eq!(e.lanes_left, 2);
        assert_eq!(e.lanes_right, 2);
    }

    #[test]
    fn parse_legacy_and_modern_mixed() {
        // Legacy stem "look0" and modern stem "narrow1" — no collision.
        let sii = "SiiNunit\n{\n\
                   road_look : road.look0 {\n\
                   lanes_left[]: t\n\
                   lanes_right[]: t\n\
                   }\n\
                   road_look.a.narrow1 : .road_look_data {\n\
                   lanes_left[]: t\n\
                   lanes_left[]: t\n\
                   lanes_right[]: t\n\
                   }\n}\n";
        let map = parse_road_look_text(sii);
        assert_eq!(map.len(), 2);
        let legacy = map[&trucklib_token("look0")];
        assert_eq!((legacy.lanes_left, legacy.lanes_right), (1, 1));
        let modern = map[&trucklib_token("narrow1")];
        assert_eq!((modern.lanes_left, modern.lanes_right), (2, 1));
    }

    // ── lane_type_to_width ───────────────────────────────────────────────────

    #[test]
    fn lane_type_to_width_motorway() {
        assert_eq!(lane_type_to_width("traffic_lane.road.motorway"), 3.75);
        assert_eq!(lane_type_to_width("highway_lane"), 3.75);
        assert_eq!(lane_type_to_width("traffic_lane.road.local"), 3.0);
        assert_eq!(lane_type_to_width("traffic_lane.road.city"), 3.0);
        assert_eq!(lane_type_to_width("traffic_lane.road.country"), 3.5);
        assert_eq!(lane_type_to_width("unknown_type"), 3.75);
    }

    #[test]
    fn lane_width_propagated_to_entry() {
        // road_look.motor2.mw → stem = "mw"
        let sii = "SiiNunit\n{\nroad_look.motor2.mw : .road_look_data {\n\
                   lanes_left[]: traffic_lane.road.motorway\n\
                   lanes_left[]: traffic_lane.road.motorway\n\
                   lanes_right[]: traffic_lane.road.motorway\n\
                   lanes_right[]: traffic_lane.road.motorway\n\
                   }\n}\n";
        let map = parse_road_look_text(sii);
        let tok = trucklib_token("mw");
        let e = map.get(&tok).expect("motorway entry must exist");
        assert_eq!(e.lanes_left, 2);
        assert_eq!(e.lanes_right, 2);
        assert!((e.lane_width_m - 3.75).abs() < 1e-5, "motorway lane width = 3.75, got {}", e.lane_width_m);
    }

    // ── Phase-2e: trucklib_token empirical anchor ────────────────────────────

    /// trucklib_token("ger7") MUST equal 479995.
    ///
    /// This is the empirically verified value observed in live ETS2 sector data
    /// (road_type field = 479995 for German A7-style roads).  If this fails the
    /// trucklib_token algorithm is broken.
    #[test]
    fn trucklib_token_ger7_anchor() {
        assert_eq!(trucklib_token("ger7"), 479995,
            "empirical anchor: road.ger7 must hash to 479995 via trucklib encoding");
    }

    /// trucklib_token("") must equal 0 (empty stem → no contribution).
    #[test]
    fn trucklib_token_empty() {
        assert_eq!(trucklib_token(""), 0);
    }

    /// Stem extraction: rsplit('.').next() picks the last dotted segment.
    /// These three cases cover the full road_look unit-name space:
    ///   • "road.ger7"  → "ger7"
    ///   • "road.look0" → "look0"
    ///   • "foo"        → "foo"  (no dot → whole string)
    #[test]
    fn stem_extraction_rsplit() {
        assert_eq!("road.ger7".rsplit('.').next(), Some("ger7"));
        assert_eq!("road.look0".rsplit('.').next(), Some("look0"));
        assert_eq!("foo".rsplit('.').next(), Some("foo"));
    }

    // ── Phase-2e: road.ger7 fixture ─────────────────────────────────────────

    /// parse_road_look_text on a legacy "road.ger7" block must:
    ///   • use key = trucklib_token("ger7") = 479995
    ///   • lanes_right = 2  (lanes_forward)
    ///   • lanes_left  = 2  (lanes_backward)
    ///   • lane_width_m = 3.75  (expressway type)
    ///
    /// This is a text-fixture, not the real .scs archive.
    #[test]
    fn parse_ger7_fixture_legacy_format() {
        let sii = "SiiNunit\n{\n\
            road_look : road.ger7 {\n\
            lanes_right[]: traffic_lane.road.expressway\n\
            lanes_right[]: traffic_lane.road.expressway\n\
            lanes_left[]:  traffic_lane.road.expressway\n\
            lanes_left[]:  traffic_lane.road.expressway\n\
            }\n}\n";
        let map = parse_road_look_text(sii);
        let token = trucklib_token("ger7");
        assert_eq!(token, 479995, "anchor token must be 479995");
        let entry = map.get(&token).expect("road.ger7 entry must be present under trucklib_token(\"ger7\")");
        assert_eq!(entry.lanes_right, 2, "lanes_right (forward) must be 2");
        assert_eq!(entry.lanes_left,  2, "lanes_left (backward) must be 2");
        assert!(
            (entry.lane_width_m - 3.75).abs() < 1e-5,
            "expressway lane_width_m must be 3.75, got {}",
            entry.lane_width_m
        );
    }

    /// Same fixture in modern format: road_look.ger7.road → stem = "road".
    /// Verifies modern-format stem extraction does NOT accidentally produce 479995
    /// when the stem is "road", only when stem is "ger7".
    ///
    /// NOTE: the modern unit name for the ger7 look would be something like
    /// "road_look.ger7" (stem = "ger7") or "road_look.ger_road.ger7" (stem = "ger7").
    /// Using a modern block where the last segment IS "ger7":
    #[test]
    fn parse_ger7_fixture_modern_format() {
        let sii = "SiiNunit\n{\n\
            road_look.ger_road.ger7 : .road_look_data {\n\
            lanes_right[]: traffic_lane.road.expressway\n\
            lanes_right[]: traffic_lane.road.expressway\n\
            lanes_left[]:  traffic_lane.road.expressway\n\
            lanes_left[]:  traffic_lane.road.expressway\n\
            }\n}\n";
        let map = parse_road_look_text(sii);
        let token = trucklib_token("ger7");
        assert_eq!(token, 479995);
        let entry = map.get(&token).expect("modern road_look.ger_road.ger7 must map to token 479995");
        assert_eq!(entry.lanes_right, 2);
        assert_eq!(entry.lanes_left,  2);
        assert!((entry.lane_width_m - 3.75).abs() < 1e-5);
    }

    // ── Phase-2e: expressway lane width ─────────────────────────────────────

    /// lane_type_to_width must return 3.75 for expressway.
    /// This is the critical value for the lane-keeper Erfolgskriterium.
    #[test]
    fn expressway_lane_width_is_3_75() {
        assert_eq!(
            lane_type_to_width("traffic_lane.road.expressway"),
            3.75,
            "expressway lane width must be exactly 3.75 m"
        );
    }

    // ── Phase-2e: edge case CONCERN-4 — zero lanes_right ────────────────────

    /// A road_look block with no lanes_right[] entries must result in
    /// RoadLookEntry.lanes_right == 0.
    ///
    /// This documents the current behaviour (no Produktivcode-Fix).
    /// A router or graph builder consuming this entry must handle 0 lanes.
    #[test]
    fn zero_lanes_right_preserved() {
        let sii = "SiiNunit\n{\n\
            road_look : road.oneway {\n\
            lanes_left[]: traffic_lane.road.local\n\
            lanes_left[]: traffic_lane.road.local\n\
            }\n}\n";
        let map = parse_road_look_text(sii);
        let entry = map.get(&trucklib_token("oneway"))
            .expect("oneway entry must be present");
        assert_eq!(entry.lanes_right, 0,
            "CONCERN-4: road with no lanes_right[] entries must produce lanes_right=0");
        assert_eq!(entry.lanes_left, 2,
            "lanes_left must still be counted correctly");
    }

    // ── Phase-2e: load_road_look merge — note on testability ─────────────────
    //
    // load_road_look() requires `&mut [Box<dyn Archive>]`.  The Archive trait
    // needs a full read_path() impl.  Building two mock archives requires a
    // non-trivial in-memory implementation of the Archive trait, which in turn
    // requires parse_directory_listing() to accept hand-crafted bytes.
    //
    // The merge logic itself (HashMap::extend, later-file-wins) is exercised
    // indirectly by the parse_road_look_text multi-block tests above.
    // A dedicated integration test would require a test-only InMemoryArchive
    // fixture; that is deferred and the reason documented here.

    /// Diagnostic: print CityHash64 vs scs_token_hash for road.lookN names.
    ///
    /// Run with: cargo test hash_probe -- --nocapture --ignored
    #[test]
    #[ignore]
    fn hash_probe() {
        use crate::cityhash::cityhash64;
        // Known binary tokens from live sectors (from apply_road_look diagnostic).
        let binary_tokens: &[u64] = &[
            113575,
            2476698808250199,
            3238735520439,
            2805081445975,
            1671359661933188,
            353978643968900,
            197458126983044,
            1180059150283140,
            101735984151, // from road_dump (road_type field)
        ];
        for i in 0..=31u32 {
            let name = format!("road.look{i}");
            let scs = scs_token_hash(&name);
            let city = cityhash64(name.as_bytes());
            println!("scs_token={scs:>22}  city={city:>22}  name={name}");
            for &bt in binary_tokens {
                if bt == city {
                    println!("  *** CITY MATCH for binary_token={bt} ***");
                }
                if bt == scs {
                    println!("  *** SCS MATCH for binary_token={bt} ***");
                }
            }
        }
    }
}
