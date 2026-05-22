//! road_look.sii loader — maps road-look token64 values to lane counts.
//!
//! In the ETS2 binary sector format, every Road item stores a `roadLook`
//! token64 (the first 8-byte field after the flags, corresponding to
//! `header.road_type` in [`crate::road_full::RoadFixedHeader`]).  That token
//! is a reference into `/def/road_look.sii`, which lists the number of lanes
//! per side for each road look.
//!
//! ## Token hash
//!
//! ETS2 token64 values in binary sector items use a **base-38 polynomial hash**
//! — distinct from CityHash64 which is used only for HashFS file-path lookups.
//! See [`scs_token_hash`] for the algorithm.
//!
//! ## SII format
//!
//! The plain-text SII file contains blocks like:
//!
//! ```text
//! road_look.narrow1.road : .road_look_data {
//!     name: "Narrow 1-lane"
//!     lanes_left[]: "narrow_lane"
//!     lanes_right[]: "narrow_lane"
//!     lanes_right[]: "narrow_lane"
//!     ...
//! }
//! ```
//!
//! The token stored in the binary is `scs_token_hash("road_look.narrow1.road")`.
//! `lanes_left[]` count → `lanes_backward`, `lanes_right[]` count → `lanes_forward`.
//!
//! If the file is absent or in binary SII (BSII) format the loader returns an
//! empty map and lane counts remain 0 (falling back to `bidirectional_unknown`
//! edges), matching existing behaviour.

use std::collections::HashMap;

use tracing::{debug, info, warn};

use crate::archive::Archive;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Compact lane-count entry for one road-look definition.
#[derive(Debug, Clone, Copy, Default)]
pub struct RoadLookEntry {
    /// Lane count on the left side — **lanes going backward** (`lanes_backward`).
    pub lanes_left: u8,
    /// Lane count on the right side — **lanes going forward** (`lanes_forward`).
    pub lanes_right: u8,
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

fn parse_road_look_text(text: &str) -> HashMap<u64, RoadLookEntry> {
    let mut map: HashMap<u64, RoadLookEntry> = HashMap::new();
    let mut in_block = false;
    let mut current_token: u64 = 0;
    let mut current_name = String::new();
    let mut lanes_left: u8 = 0;
    let mut lanes_right: u8 = 0;

    for raw_line in text.lines() {
        let line = raw_line.trim();

        // Skip comments and blanks.
        if line.is_empty() || line.starts_with('#') || line.starts_with("//") {
            continue;
        }

        if !in_block {
            // Modern format:  road_look.NAME : .road_look_data {
            //   token = scs_token_hash("road_look.NAME")
            // Legacy format:  road_look : road.lookN {
            //   token = scs_token_hash("road.lookN")
            if let Some(rest) = line.strip_prefix("road_look.") {
                // Extract NAME — everything before the first space or colon.
                let end = rest.find([' ', ':']).unwrap_or(rest.len());
                let unit_name = &rest[..end];
                if !unit_name.is_empty() {
                    current_name = format!("road_look.{unit_name}");
                    current_token = scs_token_hash(&current_name);
                    lanes_left = 0;
                    lanes_right = 0;
                    in_block = true;
                    debug!(token = current_token, name = %current_name, "road_look block start (modern)");
                }
            } else if let Some(rest) = line.strip_prefix("road_look :") {
                // Legacy: extract the class name (e.g. "road.look0") after " : ".
                let class_name = rest.trim_start();
                let end = class_name.find([' ', '{']).unwrap_or(class_name.len());
                let class_name = &class_name[..end];
                if !class_name.is_empty() {
                    current_name = class_name.to_string();
                    current_token = scs_token_hash(class_name);
                    lanes_left = 0;
                    lanes_right = 0;
                    in_block = true;
                    debug!(token = current_token, name = %current_name, "road_look block start (legacy)");
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
                    },
                );
                debug!(
                    name = %current_name,
                    lanes_left,
                    lanes_right,
                    "road_look block end"
                );
                in_block = false;
            } else if line.starts_with("lanes_left[") {
                lanes_left = lanes_left.saturating_add(1);
            } else if line.starts_with("lanes_right[") {
                lanes_right = lanes_right.saturating_add(1);
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
            },
        );
    }

    info!("road_look.sii: {} entries parsed", map.len());
    map
}

// ---------------------------------------------------------------------------
// Archive loader
// ---------------------------------------------------------------------------

/// Candidate paths for road_look.sii, tried in order.
const ROAD_LOOK_PATHS: &[&str] = &[
    "def/road_look.sii",
    "def/world/road_look.sii",
    "def/world/road.sii",
];

/// Load road_look definitions from the given archive slice.
///
/// Iterates the archives from back to front (mod-override order), returns
/// the first non-empty map.  Returns an empty map if the file is not found
/// in any archive.
pub fn load_road_look(archives: &mut [Box<dyn Archive>]) -> HashMap<u64, RoadLookEntry> {
    for &path in ROAD_LOOK_PATHS {
        // Last archive wins (mods override base).
        let data = archives
            .iter_mut()
            .rev()
            .find_map(|arc| arc.read_path(path).ok());

        if let Some(bytes) = data {
            info!("road_look: loading from '{}' ({} bytes)", path, bytes.len());
            let map = parse_road_look_sii(&bytes);
            if !map.is_empty() {
                return map;
            }
            // Dump each zero-entry file with a unique name for inspection.
            let safe_name = path.replace('/', "_").replace('.', "_");
            let dump_path = format!("outputs/2026-05-22/diag/road_look_dump_{safe_name}.sii");
            if let Err(e) = std::fs::write(&dump_path, &bytes) {
                warn!("road_look: failed to write dump to {dump_path}: {e}");
            } else {
                warn!("road_look at '{path}' ({} bytes total) yielded 0 entries — dump written to {dump_path}", bytes.len());
            }
        }
    }

    warn!("road_look.sii not found in any archive — all legacy roads will use bidirectional_unknown edges");
    HashMap::new()
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
        let sii = "SiiNunit\n{\nroad_look.narrow1.road : .road_look_data {\n\
                   lanes_left[]: \"narrow\"\n\
                   lanes_right[]: \"narrow\"\n\
                   }\n}\n";
        let map = parse_road_look_text(sii);
        let tok = scs_token_hash("road_look.narrow1.road");
        let e = map.get(&tok).expect("entry must exist");
        assert_eq!(e.lanes_left, 1);
        assert_eq!(e.lanes_right, 1);
    }

    #[test]
    fn parse_asymmetric_lanes() {
        let sii = "SiiNunit\n{\nroad_look.motor3.road : .road_look_data {\n\
                   lanes_left[]: \"m\"\n\
                   lanes_left[]: \"m\"\n\
                   lanes_right[]: \"m\"\n\
                   lanes_right[]: \"m\"\n\
                   lanes_right[]: \"m\"\n\
                   }\n}\n";
        let map = parse_road_look_text(sii);
        let tok = scs_token_hash("road_look.motor3.road");
        let e = map.get(&tok).expect("entry must exist");
        assert_eq!(e.lanes_left, 2, "2 left lanes");
        assert_eq!(e.lanes_right, 3, "3 right lanes");
    }

    #[test]
    fn parse_multiple_entries() {
        let sii = "SiiNunit\n{\n\
                   road_look.a.road : .road_look_data {\n\
                   lanes_left[]: \"x\"\n\
                   lanes_right[]: \"x\"\n\
                   }\n\
                   road_look.b.road : .road_look_data {\n\
                   lanes_right[]: \"x\"\n\
                   lanes_right[]: \"x\"\n\
                   }\n\
                   }\n";
        let map = parse_road_look_text(sii);
        assert_eq!(map.len(), 2);
        let a = map[&scs_token_hash("road_look.a.road")];
        assert_eq!((a.lanes_left, a.lanes_right), (1, 1));
        let b = map[&scs_token_hash("road_look.b.road")];
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
}
