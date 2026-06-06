//! `truckpilot-sii-probe` — diagnostic probe for prefab SII discovery.
//!
//! Opens ETS2 archives from a given directory, calls `load_prefab_sii_defs`,
//! and reports what SII files were found, whether they are binary or text,
//! and how many token→path entries were loaded.
//!
//! Usage:
//! ```powershell
//! cargo run --release --bin truckpilot-sii-probe -- \
//!   --scs-dir "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2"
//! ```

use std::path::{Path, PathBuf};

use truckpilot_map_parser::archive::Archive;
use truckpilot_map_parser::hashfs::parse_directory_listing;
use truckpilot_map_parser::prefab_sii::{load_prefab_sii_defs, load_prefab_sii_pairs};
use truckpilot_map_parser::road_look::scs_token_hash;
use truckpilot_map_parser::{HashFsArchive, ZipArchive};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let scs_dir = parse_scs_dir(&args);

    eprintln!("[sii_probe] scanning archives in: {}", scs_dir.display());

    let mut scs_files: Vec<PathBuf> = std::fs::read_dir(&scs_dir)
        .unwrap_or_else(|e| panic!("cannot read {:?}: {e}", scs_dir))
        .filter_map(|e| e.ok())
        .filter(|e| {
            e.path()
                .extension()
                .and_then(|s| s.to_str())
                .map(|ext| ext.eq_ignore_ascii_case("scs"))
                .unwrap_or(false)
        })
        .map(|e| e.path())
        .collect();
    scs_files.sort();

    eprintln!("[sii_probe] {} .scs files found", scs_files.len());
    for f in &scs_files {
        eprintln!("[sii_probe]   {}", f.file_name().unwrap().to_string_lossy());
    }

    let mut archives: Vec<Box<dyn Archive>> = Vec::new();
    for path in &scs_files {
        match open_archive(path) {
            Some(arc) => {
                eprintln!(
                    "[sii_probe] opened: {}",
                    path.file_name().unwrap().to_string_lossy()
                );
                archives.push(arc);
            }
            None => {
                eprintln!(
                    "[sii_probe] SKIP (not HashFS or ZIP): {}",
                    path.file_name().unwrap().to_string_lossy()
                );
            }
        }
    }

    // Dump first 80 lines of prefab.sii (the large main file)
    'dump: for arc in &mut archives {
        let target = "def/world/prefab.sii";
        if let Ok(bytes) = arc.read_path(target) {
            if let Ok(text) = std::str::from_utf8(&bytes) {
                eprintln!("[sii_probe] --- RAW content of '{target}' (first 80 lines) ---");
                for (i, line) in text.lines().take(80).enumerate() {
                    eprintln!("[sii_probe] {i:3}: {line}");
                }
                eprintln!("[sii_probe] --- end ---");
                break 'dump;
            }
        }
    }

    eprintln!(
        "[sii_probe] --- calling load_prefab_sii_defs on {} archives ---",
        archives.len()
    );
    let map = load_prefab_sii_defs(&mut archives);
    eprintln!(
        "[sii_probe] RESULT: {} token→path entries loaded",
        map.len()
    );

    // Print a sample of the entries
    eprintln!("[sii_probe] sample entries (first 20):");
    for (i, (tok, path)) in map.iter().take(20).enumerate() {
        eprintln!("[sii_probe]   {i}: 0x{tok:016X} → {path}");
    }

    // -----------------------------------------------------------------------
    // HASH VARIANT BRUTE-FORCE: find which hash input + width produces the
    // observed token values seen in sector binary data:
    //   0x0000000003052735  and  0x00000000862939C5
    //
    // We load raw (unit_name, ppd_path) pairs (re-opening archives), then
    // for each pair compute ~10 variants and check for a match.
    // -----------------------------------------------------------------------

    // field B (variant, @61): 0x0000000003052735, 0x00000000862939C5
    // field A (model, @53):   0x0004D1B6A37B76AD, 0x0004C1E5710EF2AD
    let observed_targets: [u64; 4] = [
        0x0000000003052735u64,
        0x00000000862939C5u64,
        0x0004D1B6A37B76ADu64,
        0x0004C1E5710EF2ADu64,
    ];

    eprintln!("\n[sii_probe] --- hash-variant brute-force ---");
    eprintln!(
        "[sii_probe] targets: {:?}",
        observed_targets.map(|v| format!("0x{v:016X}"))
    );

    // Re-open archives for the pairs call
    let mut archives2: Vec<Box<dyn Archive>> = Vec::new();
    for path in &scs_files {
        if let Some(arc) = open_archive(path) {
            archives2.push(arc);
        }
    }
    let pairs = load_prefab_sii_pairs(&mut archives2);
    eprintln!(
        "[sii_probe] {} raw (unit_name, ppd_path) pairs loaded",
        pairs.len()
    );

    // Build variant map: variant_u64 → (variant_tag, unit_name, ppd_path)
    let mut variant_map: std::collections::HashMap<u64, (&str, &str, &str)> =
        std::collections::HashMap::new();

    for (unit_name, ppd_path) in &pairs {
        // Derive ppd_stem: last path component, strip .ppd
        let ppd_file = ppd_path.rsplit('/').next().unwrap_or(ppd_path);
        let ppd_stem = ppd_file.strip_suffix(".ppd").unwrap_or(ppd_file);

        // Derive suffix after last '.' in unit_name (e.g. "prefab.0" → "0")
        let dot_suffix = unit_name.rsplit('.').next().unwrap_or(unit_name);

        // ppd_stem with hyphens stripped (treated as non-alphabet → 0 in hash)
        // The hash already maps '-' → 0, but let's also try a version with '_' replacing '-'
        let ppd_stem_underscore = ppd_stem.replace('-', "_");

        let variants: &[(&str, &str)] = &[
            ("unit_name:u64", unit_name.as_str()),
            ("dot_suffix:u64", dot_suffix),
            ("ppd_stem:u64", ppd_stem),
            ("ppd_stem_under:u64", ppd_stem_underscore.as_str()),
        ];

        for &(tag, input) in variants {
            let h64 = scs_token_hash(input);
            let h32 = (h64 & 0xFFFF_FFFF) as u64;
            let tag32 = Box::leak(tag.replace(":u64", ":u32").into_boxed_str());
            variant_map
                .entry(h64)
                .or_insert((tag, unit_name.as_str(), ppd_path.as_str()));
            variant_map
                .entry(h32)
                .or_insert((tag32, unit_name.as_str(), ppd_path.as_str()));
        }

        // u32-native variant: same polynomial but with u32 wrapping throughout
        for (tag, input) in &[
            ("unit_name:u32native", unit_name.as_str()),
            ("dot_suffix:u32native", dot_suffix),
            ("ppd_stem:u32native", ppd_stem),
            ("ppd_stem_under:u32native", ppd_stem_underscore.as_str()),
        ] {
            let h = scs_token_hash_u32(input) as u64;
            variant_map
                .entry(h)
                .or_insert((tag, unit_name.as_str(), ppd_path.as_str()));
        }

        // TruckLib little-endian token: sum(charIndex[i] * 38^i)
        // charset: 0-9=1-10, a-z=11-36, _=37
        for (tag, input) in &[
            ("unit_name:trucklib", unit_name.as_str()),
            ("dot_suffix:trucklib", dot_suffix),
            ("ppd_stem:trucklib", ppd_stem),
            ("ppd_stem_under:trucklib", ppd_stem_underscore.as_str()),
        ] {
            let h64 = trucklib_token(input);
            let h32 = h64 & 0xFFFF_FFFF;
            let tag32 = Box::leak(format!("{}_u32", tag).into_boxed_str());
            variant_map
                .entry(h64)
                .or_insert((tag, unit_name.as_str(), ppd_path.as_str()));
            variant_map
                .entry(h32)
                .or_insert((tag32, unit_name.as_str(), ppd_path.as_str()));
        }
    }

    eprintln!(
        "[sii_probe] variant map: {} distinct values",
        variant_map.len()
    );

    let mut any_match = false;
    for &target in &observed_targets {
        if let Some((tag, name, path)) = variant_map.get(&target) {
            eprintln!(
                "[sii_probe] HIT  0x{target:016X}  variant={tag}  unit_name={name:?}  ppd={path:?}"
            );
            any_match = true;
        } else {
            eprintln!("[sii_probe] MISS 0x{target:016X}  — no variant matched");
        }
    }
    if !any_match {
        eprintln!(
            "[sii_probe] CONCLUSION: none of the ~10 hash variants matched either observed token"
        );
        eprintln!("[sii_probe]   → the token is likely computed from a DIFFERENT string (not SII unit_name or ppd_stem)");
        eprintln!("[sii_probe]   → or the SII files are all binary (BSII) and were not parsed");
    }

    // -----------------------------------------------------------------------
    // BINARY SCAN: check BOTH scs_token_hash AND TruckLib tokens vs sector data.
    // -----------------------------------------------------------------------
    // Build TruckLib token set from the same 4145 pairs.
    let mut trucklib_keys: std::collections::HashMap<u64, String> =
        std::collections::HashMap::new();
    for (unit_name, ppd_path) in &pairs {
        let dot_suffix = unit_name.rsplit('.').next().unwrap_or(unit_name.as_str());
        let tok = trucklib_token(dot_suffix);
        trucklib_keys.insert(tok, ppd_path.clone());
    }
    eprintln!(
        "[sii_probe] TruckLib dot-suffix token set: {} keys",
        trucklib_keys.len()
    );

    let sii_keys: std::collections::HashSet<u64> = map.into_keys().collect();
    // Known observed token candidates from parse_prefab debug output:
    let observed_tokens: std::collections::HashSet<u64> = [
        0x0000000003052735u64,
        0x00000000862939C5u64,
        0x0004D1B6A37B76ADu64,
        0x0004C1E5710EF2ADu64,
    ]
    .iter()
    .copied()
    .collect();

    eprintln!(
        "\n[sii_probe] --- binary scan: {} SII keys, up to 50 sectors ---",
        sii_keys.len()
    );
    let mut sectors_scanned = 0usize;
    let mut sectors_with_observed = 0usize;
    let mut total_sii_matches = 0usize;
    let mut total_observed_matches = 0usize;

    'sector_scan: for arc in &mut archives {
        for sector_path_candidate in &["map/europe", "map/europe_r", "map"] {
            if let Ok(dir_bytes) = arc.read_path(sector_path_candidate) {
                if let Ok(items) = parse_directory_listing(&dir_bytes) {
                    for item in &items {
                        if item.is_dir {
                            continue;
                        }
                        if !item.name.to_ascii_lowercase().ends_with(".base") {
                            continue;
                        }
                        let full_path = format!("{}/{}", sector_path_candidate, item.name);
                        let sector_bytes = match arc.read_path(&full_path) {
                            Ok(b) => b,
                            Err(_) => continue,
                        };
                        sectors_scanned += 1;
                        let mut sii_count = 0usize;
                        let mut tl_count = 0usize;
                        let mut obs_count = 0usize;
                        let n = sector_bytes.len().saturating_sub(7);
                        for i in (0..n).step_by(1) {
                            if i + 8 > sector_bytes.len() {
                                break;
                            }
                            let v = u64::from_le_bytes(sector_bytes[i..i + 8].try_into().unwrap());
                            if v != 0 {
                                if sii_keys.contains(&v) {
                                    sii_count += 1;
                                    if total_sii_matches < 10 {
                                        eprintln!("[sii_probe] SII MATCH in {full_path} byte {i}: 0x{v:016X}");
                                    }
                                    total_sii_matches += 1;
                                }
                                if let Some(ppd) = trucklib_keys.get(&v) {
                                    tl_count += 1;
                                    if tl_count <= 3 {
                                        eprintln!("[sii_probe] TL MATCH in {full_path} byte {i}: 0x{v:016X} → {ppd}");
                                    }
                                }
                                if observed_tokens.contains(&v) {
                                    obs_count += 1;
                                }
                            }
                        }
                        if obs_count > 0 || tl_count > 0 {
                            sectors_with_observed += 1;
                            total_observed_matches += obs_count;
                            eprintln!("[sii_probe] sector {full_path} ({} bytes): {sii_count} SII-hash matches, {tl_count} TruckLib-token matches, {obs_count} observed-token matches", sector_bytes.len());
                        }
                        if sectors_scanned >= 50 {
                            break 'sector_scan;
                        }
                    }
                }
            }
        }
    }
    eprintln!("\n[sii_probe] SUMMARY: {sectors_scanned} sectors scanned");
    eprintln!("[sii_probe]   sectors with observed prefab tokens: {sectors_with_observed}");
    eprintln!("[sii_probe]   total observed-token matches: {total_observed_matches}");
    eprintln!("[sii_probe]   total SII map key matches: {total_sii_matches}");
    if total_observed_matches == 0 {
        eprintln!("[sii_probe] WARNING: observed tokens not found in any of the 50 scanned sectors — sectors may have no prefabs");
    } else if total_sii_matches == 0 {
        eprintln!("[sii_probe] CONCLUSION: prefab sectors exist (observed tokens found) but SII map keys never match → SII parser computes WRONG tokens");
    } else {
        eprintln!("[sii_probe] CONCLUSION: SII map keys match sector data!");
    }
}

/// TruckLib Token encoding: little-endian base-38.
/// Charset: '\0'=0, '0'-'9'=1-10, 'a'-'z'=11-36, '_'=37
/// token = sum(charIndex[i] * 38^i)
fn trucklib_token(s: &str) -> u64 {
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

fn scs_token_hash_u32(s: &str) -> u32 {
    let mut h: u32 = 0;
    for &b in s.as_bytes() {
        let v: u32 = match b {
            b'a'..=b'z' => (b - b'a' + 1) as u32,
            b'A'..=b'Z' => (b - b'A' + 1) as u32,
            b'0'..=b'9' => (b - b'0' + 27) as u32,
            b'_' => 37,
            _ => 0,
        };
        h = h.wrapping_mul(38).wrapping_add(v);
    }
    h
}

fn open_archive(path: &Path) -> Option<Box<dyn Archive>> {
    if let Ok(arc) = HashFsArchive::open(path) {
        return Some(Box::new(arc));
    }
    ZipArchive::open(path)
        .ok()
        .map(|a| Box::new(a) as Box<dyn Archive>)
}

fn parse_scs_dir(args: &[String]) -> PathBuf {
    let mut i = 0;
    while i < args.len() {
        if args[i] == "--scs-dir" {
            let val = args.get(i + 1).expect("--scs-dir needs value");
            return PathBuf::from(val);
        }
        i += 1;
    }
    eprintln!("usage: truckpilot-sii-probe --scs-dir <ETS2_DIR>");
    std::process::exit(1);
}
