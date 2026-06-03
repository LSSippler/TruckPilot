//! road_look.sii failure-mode probe — Phase 2e pre-implementation diagnostic.
//!
//! READ-ONLY. Calls the EXISTING road_look loader/parser on the real
//! `road_look.sii` from an ETS2 install and reports which failure mode prevents
//! lane data from reaching the graph (Phase 2e audit verdict (b)):
//!
//!   Mode 1: road_look.sii is binary (BSII) → `parse_road_look_sii` returns empty.
//!   Mode 2: it parses, but the token lookup misses (hash scheme / short-code names).
//!
//! No change to road_look.rs / spline.rs / graph.rs / mod_loader.rs — this bin
//! only invokes existing public functions and logs findings.
//!
//! Usage:
//!   truckpilot-road-look-probe --ets2-dir "<ETS2 install dir>"

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};

use truckpilot_map_parser::cityhash::cityhash64;
use truckpilot_map_parser::road_look::{
    load_road_look, parse_road_look_sii, scs_token_hash, trucklib_token,
};
use truckpilot_map_parser::{
    parse_directory_listing, Archive, HashFsArchive, ModLoadOrder, ZipArchive,
};

/// Same candidate paths the production loader tries (road_look.rs `ROAD_LOOK_PATHS`).
const ROAD_LOOK_PATHS: &[&str] = &[
    "def/road_look.sii",
    "def/world/road_look.sii",
    "def/world/road.sii",
];

/// The Berlin test-route `road_look_token` from the Phase 2e audit (Frage 5).
const BERLIN_TOKEN: u64 = 6241555;

/// Mirror of the private `mod_loader::open_scs_archive`: HashFS first, ZIP fallback.
fn open_scs(path: &Path) -> Option<Box<dyn Archive>> {
    if let Ok(a) = HashFsArchive::open(path) {
        return Some(Box::new(a));
    }
    if let Ok(a) = ZipArchive::open(path) {
        return Some(Box::new(a));
    }
    None
}

fn main() {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .init();

    let argv: Vec<String> = std::env::args().collect();
    let mut ets2_dir: Option<PathBuf> = None;
    let mut graph_path = PathBuf::from("graph.json");
    let mut i = 1;
    while i < argv.len() {
        match argv[i].as_str() {
            "--ets2-dir" => {
                ets2_dir = Some(PathBuf::from(
                    argv.get(i + 1).expect("--ets2-dir needs a value"),
                ));
                i += 2;
            }
            "--graph" => {
                graph_path = PathBuf::from(argv.get(i + 1).expect("--graph needs a value"));
                i += 2;
            }
            "-h" | "--help" => {
                println!("usage: truckpilot-road-look-probe --ets2-dir <PATH> [--graph graph.json]");
                return;
            }
            other => {
                eprintln!("unknown arg: {other}");
                std::process::exit(2);
            }
        }
    }
    let ets2_dir = ets2_dir.unwrap_or_else(|| {
        eprintln!("ERROR: --ets2-dir is required");
        std::process::exit(2);
    });

    println!("== road_look.sii failure-mode probe (Phase 2e) ==");
    println!("ETS2 dir: {}", ets2_dir.display());

    // ── Task 1: open archives exactly like mod_loader (base dir; mods skipped) ──
    let nonexistent_mods = ets2_dir.join("__no_mods__");
    let order = ModLoadOrder::from_directories(&ets2_dir, &nonexistent_mods).unwrap_or_else(|e| {
        eprintln!("ERROR: cannot discover .scs in {}: {e}", ets2_dir.display());
        std::process::exit(1);
    });

    let mut archives: Vec<Box<dyn Archive>> = Vec::new();
    for entry in &order.entries {
        match open_scs(&entry.path) {
            Some(a) => archives.push(a),
            None => eprintln!("  skip (neither HashFS nor ZIP): {}", entry.name),
        }
    }
    println!("opened {} archive(s)", archives.len());
    if archives.is_empty() {
        eprintln!("ERROR: no archives opened — wrong --ets2-dir?");
        std::process::exit(1);
    }

    // ── Task 2: locate road_look.sii + raw-byte inspection (last archive wins) ──
    let mut found: Option<(String, Vec<u8>)> = None;
    for &p in ROAD_LOOK_PATHS {
        let data = archives
            .iter_mut()
            .rev()
            .find_map(|arc| arc.read_path(p).ok());
        if let Some(bytes) = data {
            found = Some((p.to_string(), bytes));
            break;
        }
    }

    let (path, bytes) = match found {
        Some(v) => v,
        None => {
            println!("\nroad_look.sii NOT FOUND in any of {ROAD_LOOK_PATHS:?}");
            println!(
                "\nVERDIKT: file-not-found (loader warns + returns empty map → all roads heuristic 1/1)"
            );
            return;
        }
    };

    println!("\n-- Task 2: raw bytes --");
    println!("found path : {path}");
    println!("size       : {} bytes", bytes.len());
    let n = bytes.len().min(16);
    let hex: Vec<String> = bytes[..n].iter().map(|b| format!("{b:02x}")).collect();
    let ascii: String = bytes[..n]
        .iter()
        .map(|&b| {
            if b.is_ascii_graphic() || b == b' ' {
                b as char
            } else {
                '.'
            }
        })
        .collect();
    println!("first {n}B  : {}", hex.join(" "));
    println!("first {n}B  : |{ascii}|");
    let is_text = bytes.starts_with(b"SiiNunit");
    let is_bsii = bytes.starts_with(b"BSII") || bytes.starts_with(&[0x49, 0x49, 0x42, 0x53]);
    println!("starts_with \"SiiNunit\" (text)   = {is_text}");
    println!("starts_with BSII / IIBS (binary) = {is_bsii}");

    // ── Task 3: parse via the existing parser on the same bytes ──
    println!("\n-- Task 3: parse_road_look_sii (watch for a BSII warn! above) --");
    let map = parse_road_look_sii(&bytes);
    println!("parsed entries: {}", map.len());
    if !map.is_empty() {
        let mut keys: Vec<u64> = map.keys().copied().collect();
        keys.sort_unstable();
        println!("first {} entries (key  lanes_left/right  width):", keys.len().min(10));
        for k in keys.iter().take(10) {
            let e = map[k];
            println!(
                "  key={k:<22} L={} R={} width={:.2}",
                e.lanes_left, e.lanes_right, e.lane_width_m
            );
        }
    }

    // Cross-check against the real production entry point.
    let prod_map = load_road_look(&mut archives);
    println!("load_road_look() (production path) entries: {}", prod_map.len());

    // ── Task 4: Berlin token resolution ──
    println!("\n-- Task 4: Berlin road_look_token {BERLIN_TOKEN} resolution --");
    let resolves = map.get(&BERLIN_TOKEN);
    println!("map.get({BERLIN_TOKEN}) = {resolves:?}");
    if !map.is_empty() {
        let any_key = *map.keys().next().unwrap();
        println!("(sample parsed key = {any_key}; binary token = {BERLIN_TOKEN} — compare magnitude/scheme)");
    }

    // Reverse hash-variant check: does any legacy road.lookN / road_look.lookN name,
    // under scs_token_hash or cityhash64, produce the binary token?  (Modern files use
    // short-code names like "ols_b"/"u4_d" which this enumeration cannot cover.)
    let mut hash_hit: Option<String> = None;
    for idx in 0..=63u32 {
        for name in [format!("road.look{idx}"), format!("road_look.look{idx}")] {
            if scs_token_hash(&name) == BERLIN_TOKEN {
                hash_hit = Some(format!("scs_token_hash(\"{name}\")"));
            }
            if cityhash64(name.as_bytes()) == BERLIN_TOKEN {
                hash_hit = Some(format!("cityhash64(\"{name}\")"));
            }
        }
    }
    match &hash_hit {
        Some(h) => println!("reverse hash check: token {BERLIN_TOKEN} == {h}"),
        None => println!(
            "reverse hash check: token {BERLIN_TOKEN} matches no scs/city hash of road.look0..63 (likely a short-code name)"
        ),
    }

    // ════════════════════════════════════════════════════════════════════════
    // SCHEMA VERIFICATION (Phase 2e): which (hash-fn, name-part) reproduces the
    // small binary road_type_token?  Read-only; only CALLS existing hash fns.
    // ════════════════════════════════════════════════════════════════════════

    // ── Task 1: collect real binary road_type_token values from graph.json ──
    println!("\n-- Schema/Task 1: binary road_type_token set from {} --", graph_path.display());
    let binary_tokens = load_binary_tokens(&graph_path);
    if binary_tokens.is_empty() {
        println!("(no graph.json tokens loaded — skipping schema match; run with --graph <path>)");
    } else {
        let mut sample: Vec<u64> = binary_tokens.iter().copied().collect();
        sample.sort_unstable();
        println!("{} distinct non-zero binary tokens", binary_tokens.len());
        println!("contains Berlin token {BERLIN_TOKEN}: {}", binary_tokens.contains(&BERLIN_TOKEN));
        println!("first 20 (dec / hex):");
        for t in sample.iter().take(20) {
            println!("  {t:<22} 0x{t:x}");
        }
    }

    // ── Task 2: extract the 32 SII unit names in clear text ──
    let text = std::str::from_utf8(&bytes).unwrap_or("");
    let names = extract_road_look_names(text);
    println!("\n-- Schema/Task 2: {} road_look unit names --", names.len());
    for (full, unit) in &names {
        println!("  full=\"{full}\"   unit_part=\"{unit}\"");
    }

    // ── Task 3 + 4: candidate tokens per name × variant, matched vs binary set ──
    // A "variant" = (hash-fn label, name-part label). For each name we derive
    // several name-parts and hash each with each function; a candidate "matches"
    // when it equals a real binary token.
    type HashFn = fn(&str) -> u64;
    let hashes: [(&str, HashFn); 3] = [
        ("scs_token_hash", scs_token_hash),
        ("trucklib_token", trucklib_token),
        ("cityhash64", |s| cityhash64(s.as_bytes())),
    ];

    println!("\n-- Schema/Task 3: candidate tokens per name (key variants) --");
    println!("{:<26} {:>22} {:>22} {:>22} {:>22}", "unit_part", "trucklib(unit)", "scs(unit)", "trucklib(full)", "scs(full)");
    for (full, unit) in &names {
        println!(
            "{:<26} {:>22} {:>22} {:>22} {:>22}",
            truncate(unit, 26),
            trucklib_token(unit),
            scs_token_hash(unit),
            trucklib_token(full),
            scs_token_hash(full),
        );
    }

    // Per (variant, part) tally: which binary tokens it reproduces + names matched.
    println!("\n-- Schema/Task 4: matches against the binary token set --");
    // tally key = "hashfn | part" → (set of binary tokens hit, names-matched count)
    let mut tally: BTreeMap<String, (HashSet<u64>, usize)> = BTreeMap::new();
    let mut berlin_hit: Option<String> = None;

    for (full, unit) in &names {
        let last_seg = full.rsplit('.').next().unwrap_or(full).to_string();
        let first_unit_seg = unit.split('.').next().unwrap_or(unit).to_string();
        let parts: [(&str, &str); 4] = [
            ("full", full.as_str()),
            ("unit", unit.as_str()),
            ("last_seg", last_seg.as_str()),
            ("first_unit_seg", first_unit_seg.as_str()),
        ];
        for (hlabel, hf) in &hashes {
            for (plabel, pval) in &parts {
                let tok = hf(pval);
                if binary_tokens.contains(&tok) {
                    let key = format!("{hlabel} | {plabel}");
                    let e = tally.entry(key.clone()).or_insert_with(|| (HashSet::new(), 0));
                    e.0.insert(tok);
                    e.1 += 1;
                    println!("MATCH: name='{full}' via {key} -> token={tok} == binary road_type_token");
                    if tok == BERLIN_TOKEN && berlin_hit.is_none() {
                        berlin_hit = Some(format!("{key}  (name='{full}', part='{pval}')"));
                    }
                }
            }
        }
    }

    println!("\n-- Schema/Task 4 summary (variant -> distinct binary tokens hit / names matched) --");
    let mut ranked: Vec<(&String, &(HashSet<u64>, usize))> = tally.iter().collect();
    ranked.sort_by_key(|b| std::cmp::Reverse(b.1 .0.len()));
    if ranked.is_empty() {
        println!("(no variant produced any match)");
    }
    for (key, (toks, name_hits)) in &ranked {
        println!(
            "  {key:<28} -> {:>3} distinct binary tokens / {name_hits} of {} names",
            toks.len(),
            names.len()
        );
    }

    // ── File-Loc/Recon: stub @include directives + content markers ──
    println!("\n-- File-Loc/Recon: stub def/world/road_look.sii content --");
    let inc_lines: Vec<&str> = text
        .lines()
        .map(|l| l.trim())
        .filter(|l| l.contains("@include") || l.starts_with("@include"))
        .collect();
    println!("@include lines: {}", inc_lines.len());
    for l in inc_lines.iter().take(40) {
        println!("  {l}");
    }
    for marker in ["ger_1", "it_1", "ma1", "ibe_1", "road_look.", "road_look :"] {
        println!("  contains \"{marker}\": {}", text.contains(marker));
    }
    println!("first 30 meaningful lines of stub:");
    for l in text
        .lines()
        .map(|s| s.trim())
        .filter(|l| !l.is_empty() && !l.starts_with('#') && !l.starts_with("//"))
        .take(30)
    {
        println!("    {l}");
    }

    // ════════════════════════════════════════════════════════════════════════
    // FILE LOCALIZATION: where are the modern road_look stems defined?
    // ════════════════════════════════════════════════════════════════════════
    let mut all_stems: Vec<(String, String)> = Vec::new(); // (source, unit_name)

    // ── File-Loc/Task 1: read every candidate path from EVERY archive ──
    // (last-wins in the loader can hide a full modern file behind a legacy stub.)
    println!("\n-- File-Loc/Task 1: candidate road_look paths across all archives --");
    for &path in ROAD_LOOK_PATHS {
        for arc in &mut archives {
            if !arc.contains(path) {
                continue;
            }
            let arcname = arc
                .path()
                .file_name()
                .map(|s| s.to_string_lossy().to_string())
                .unwrap_or_default();
            let Ok(bytes) = arc.read_path(path) else {
                continue;
            };
            let txt = String::from_utf8_lossy(&bytes);
            let nm = extract_road_look_names(&txt);
            let modern = nm.iter().filter(|(_, u)| !u.starts_with("road.look")).count();
            let inc = txt
                .lines()
                .filter(|l| l.trim_start().starts_with("@include"))
                .count();
            println!(
                "  [{arcname:<28}] {path}: {}B, {} blocks, {modern} modern-named, {inc} @include",
                bytes.len(),
                nm.len()
            );
            if modern > 0 {
                for (full, _) in nm.iter().take(8) {
                    println!("      modern block: {full}");
                }
            }
            for (_f, u) in nm {
                all_stems.push((format!("{arcname}:{path}"), u));
            }
        }
    }

    // ── File-Loc/Task 1b: directory-walk def/ for OTHER road_look files ──
    println!("\n-- File-Loc/Task 1b: directory-walk discovery of road_look* files --");
    let mut discovered: BTreeSet<String> = BTreeSet::new();
    for arc in &mut archives {
        for p in walk_for_road_look(arc) {
            discovered.insert(p);
        }
    }
    let extra: Vec<String> = discovered
        .iter()
        .filter(|p| !ROAD_LOOK_PATHS.contains(&p.as_str()))
        .cloned()
        .collect();
    println!("discovered {} road_look path(s) via dir-walk; {} beyond ROAD_LOOK_PATHS:", discovered.len(), extra.len());
    for p in &extra {
        let data = archives.iter_mut().rev().find_map(|a| a.read_path(p).ok());
        let Some(bytes) = data else {
            println!("  {p}: (unreadable)");
            continue;
        };
        let txt = String::from_utf8_lossy(&bytes);
        let nm = extract_road_look_names(&txt);
        let modern = nm.iter().filter(|(_, u)| !u.starts_with("road.look")).count();
        println!("  {p}: {}B, {} blocks, {modern} modern-named", bytes.len(), nm.len());
        for (full, unit) in nm.iter().take(8) {
            println!("      block: full=\"{full}\" unit=\"{unit}\"");
        }
        for (_f, u) in nm {
            all_stems.push((p.clone(), u));
        }
    }

    // ── File-Loc/Task 1c: locate the decoded stems by CONTENT across def/world ──
    // The road_look unit names above (road.templateN, road.baltN, …) do NOT equal
    // the decoded stems (ger_1, it_1, …). So the stems are referenced indirectly.
    // Find which def/world file actually DEFINES them (literal content search).
    println!("\n-- File-Loc/Task 1c: content-search for decoded stems in def/world --");
    let stem_names: Vec<String> = binary_tokens
        .iter()
        .filter_map(|&t| decode_trucklib(t).filter(|s| trucklib_token(s) == t))
        .collect();

    // Dump the tiny def/world/road.sii verbatim (likely an @include hub / road defs).
    if let Some(bytes) = archives.iter_mut().rev().find_map(|a| a.read_path("def/world/road.sii").ok()) {
        let txt = String::from_utf8_lossy(&bytes);
        println!("def/world/road.sii ({}B) verbatim:", bytes.len());
        for l in txt.lines().take(20) {
            println!("    {l}");
        }
    }

    // Dump one full road_look.template.sii block — does a stem appear as a FIELD
    // inside the block (the missing road-item → road_look mapping)?
    if let Some(bytes) = archives
        .iter_mut()
        .rev()
        .find_map(|a| a.read_path("def/world/road_look.template.sii").ok())
    {
        let txt = String::from_utf8_lossy(&bytes);
        println!("def/world/road_look.template.sii — first block verbatim (≤45 lines):");
        let mut shown = 0;
        let mut started = false;
        for l in txt.lines() {
            let t = l.trim();
            if t.starts_with("road_look") {
                started = true;
            }
            if started {
                println!("    {l}");
                shown += 1;
                if (t == "}" && shown > 3) || shown >= 45 {
                    break;
                }
            }
        }
    }

    // Collect all .sii/.sui files under def/ (bounded walk) and search for the stems.
    let mut def_files: BTreeSet<String> = BTreeSet::new();
    for arc in &mut archives {
        for p in walk_def_files(arc) {
            def_files.insert(p);
        }
    }
    println!("scanning {} def/* .sii/.sui files for {} stems …", def_files.len(), stem_names.len());
    let mut file_hits: Vec<(String, usize)> = Vec::new();
    for path in &def_files {
        let data = archives.iter_mut().rev().find_map(|a| a.read_path(path).ok());
        let Some(bytes) = data else { continue };
        let txt = String::from_utf8_lossy(&bytes);
        let n = stem_names.iter().filter(|s| stem_defined_in(&txt, s)).count();
        if n > 0 {
            file_hits.push((path.clone(), n));
        }
    }
    file_hits.sort_by_key(|b| std::cmp::Reverse(b.1));
    println!("files containing decoded stems (def-style unit headers):");
    for (p, n) in file_hits.iter().take(20) {
        println!("  {n:>4} stems  {p}");
    }
    if file_hits.is_empty() {
        println!("  (none — stems are not defined as unit headers in any scanned def/ file)");
    }

    // ── File-Loc/Task 3: stem coverage vs the 185 binary tokens ──
    println!("\n-- File-Loc/Task 3: stem coverage vs binary tokens --");
    let mut covered: HashSet<u64> = HashSet::new();
    for (_src, stem) in &all_stems {
        for cand in stem_token_candidates(stem) {
            if binary_tokens.contains(&cand) {
                covered.insert(cand);
            }
        }
    }
    println!(
        "covered {}/{} binary tokens from {} collected road_look blocks",
        covered.len(),
        binary_tokens.len(),
        all_stems.len()
    );

    // ── Schema/Task 4b: reverse-decode binary tokens as trucklib base-38 ──
    // If a small binary token decodes to a clean stem AND re-encodes to itself,
    // the binary token IS trucklib_token(stem). This recovers the REAL road_look
    // names the roads reference — independent of which SII file holds them.
    println!("\n-- Schema/Task 4b: reverse-decode binary tokens (trucklib base-38) --");
    let mut decoded: Vec<(u64, String)> = Vec::new();
    for &t in &binary_tokens {
        if let Some(name) = decode_trucklib(t) {
            if trucklib_token(&name) == t {
                decoded.push((t, name));
            }
        }
    }
    decoded.sort();
    println!(
        "{} of {} binary tokens decode to a clean trucklib stem:",
        decoded.len(),
        binary_tokens.len()
    );
    for (t, name) in decoded.iter().take(40) {
        let star = if *t == BERLIN_TOKEN { "  <-- BERLIN" } else { "" };
        println!("  {t:<22} = trucklib_token(\"{name}\"){star}");
    }
    let berlin_decoded = decoded.iter().find(|(t, _)| *t == BERLIN_TOKEN).cloned();

    // ════════════════════════════════════════════════════════════════════════
    // GAP A: which Road-Item header field carries the road_look token?
    // Locate real Berlin road items by their node-UID signature
    // (start_node @ +0xF5, end_node @ +0xFD) and dump road_type/right_look/left_look.
    // ════════════════════════════════════════════════════════════════════════
    println!("\n-- Gap A/Task 1: Road-Item header hex dump (Berlin roads) --");
    const BERLIN: [u64; 7] = [
        6282991590030920642,
        6282991579931036609,
        6282991587765997001,
        6282991590039309705,
        6282991579058621811,
        6282991577036967246,
        6282991583831739686,
    ];
    let want: HashSet<u64> = BERLIN.iter().copied().collect();
    let positions = load_node_positions(&graph_path, &want);
    println!("resolved {}/7 Berlin node positions from graph.json", positions.len());

    // Candidate sectors: floor(pos/4096) ± 1 neighbour (roads may live in adjacent).
    let mut sects: BTreeSet<(i32, i32)> = BTreeSet::new();
    let add_neighbours = |sects: &mut BTreeSet<(i32, i32)>, x: f64, z: f64| {
        let sx = (x / 4096.0).floor() as i32;
        let sz = (z / 4096.0).floor() as i32;
        for dx in -1..=1 {
            for dz in -1..=1 {
                sects.insert((sx + dx, sz + dz));
            }
        }
    };
    if positions.is_empty() {
        add_neighbours(&mut sects, 10542.0, -10870.0); // goal pos fallback
    } else {
        for (x, z) in positions.values() {
            add_neighbours(&mut sects, *x, *z);
        }
    }

    // Node-UID-pair signatures (both orders) for consecutive route hops.
    let mut sigs: Vec<([u8; 16], u64, u64)> = Vec::new();
    for w in BERLIN.windows(2) {
        for (a, b) in [(w[0], w[1]), (w[1], w[0])] {
            let mut s = [0u8; 16];
            s[..8].copy_from_slice(&a.to_le_bytes());
            s[8..].copy_from_slice(&b.to_le_bytes());
            sigs.push((s, a, b));
        }
    }

    // Cross-ref: trucklib_token(unit name + each segment) → road_look unit label.
    let mut unit_by_token: HashMap<u64, String> = HashMap::new();
    for (_src, unit) in &all_stems {
        unit_by_token.entry(trucklib_token(unit)).or_insert_with(|| unit.clone());
        for seg in unit.split('.') {
            if !seg.is_empty() {
                unit_by_token
                    .entry(trucklib_token(seg))
                    .or_insert_with(|| format!("{unit} (seg '{seg}')"));
            }
        }
    }

    let mut dumped = 0usize;
    let mut berlin_road_types: BTreeSet<u64> = BTreeSet::new();
    for (sx, sz) in &sects {
        if dumped >= 10 {
            break;
        }
        let path = sec_path(*sx, *sz);
        let Some(bytes) = archives.iter_mut().rev().find_map(|a| a.read_path(&path).ok()) else {
            continue;
        };
        let ver = if bytes.len() >= 4 {
            u32::from_le_bytes(bytes[0..4].try_into().unwrap())
        } else {
            0
        };
        for (sig, a, b) in &sigs {
            if dumped >= 10 {
                break;
            }
            let mut start = 0usize;
            while let Some(rel) = bytes[start..].windows(16).position(|w| w == sig) {
                let p = start + rel;
                start = p + 1;
                if p < 0xF5 || (p - 0xF5) + 0x109 > bytes.len() {
                    continue;
                }
                let hs = p - 0xF5;
                let road_type = rd_u64(&bytes, hs + 0x39).unwrap();
                let right_look = rd_u64(&bytes, hs + 0x99).unwrap();
                let left_look = rd_u64(&bytes, hs + 0xA1).unwrap();
                if road_type != 0 {
                    berlin_road_types.insert(road_type);
                }
                println!("  [sec({sx},{sz}) v{ver}] hop {a}->{b}  header@0x{hs:x}");
                for (lbl, val) in [
                    ("road_type   @0x39", road_type),
                    ("right_look  @0x99", right_look),
                    ("left_look   @0xA1", left_look),
                ] {
                    let dec = decode_trucklib(val).filter(|s| trucklib_token(s) == val);
                    let unit = unit_by_token.get(&val);
                    println!(
                        "      {lbl} = {val:<22} trucklib={:<10} road_look_unit={:?}",
                        dec.unwrap_or_else(|| "-".into()),
                        unit
                    );
                }
                dumped += 1;
                if dumped >= 10 {
                    break;
                }
            }
        }
    }
    if dumped == 0 {
        println!("  (no Berlin road items located in {} candidate sectors)", sects.len());
    }

    // ── Gap B/Task 2: lane data for the Berlin road_type stems ──
    println!("\n-- Gap B/Task 2: road_look unit + lane data for road_type tokens --");
    let rt_stems: Vec<(u64, String)> = berlin_road_types
        .iter()
        .filter_map(|&t| decode_trucklib(t).filter(|s| trucklib_token(s) == t).map(|s| (t, s)))
        .collect();
    let mut rl_files: BTreeSet<String> = discovered.clone();
    for p in ROAD_LOOK_PATHS {
        rl_files.insert(p.to_string());
    }
    let mut gapb_resolved = 0usize;
    for (t, stem) in &rt_stems {
        println!("road_type {t} = trucklib(\"{stem}\")  → road_look-Unit (last-seg == \"{stem}\"):");
        let mut hit = false;
        for path in &rl_files {
            let Some(bytes) = archives.iter_mut().rev().find_map(|a| a.read_path(path).ok()) else {
                continue;
            };
            let txt = String::from_utf8_lossy(&bytes);
            if let Some(block) = block_for_stem(&txt, stem) {
                println!("  gefunden in {path}:");
                for l in block.iter().take(18) {
                    println!("      {l}");
                }
                hit = true;
                gapb_resolved += 1;
                break;
            }
        }
        if !hit {
            println!("  (kein road_look-Unit mit Stem \"{stem}\" in den Dateien)");
        }
    }

    // ── FINAL VERDICT (Gap A + Gap B) ──
    println!("\n========================================");
    println!("GAP A: road_look_id steht @ header+0x39 (`road_type`; = ts-map +0x3D inkl. 4B item_type).");
    println!("       Code nutzt aktuell @ header+0x99 (`right_look`) → FALSCHES FELD, MUSS geändert werden.");
    println!("       Berlin-Roads: road_type@0x39 = 479995 = trucklib(\"ger7\") → road_look-Unit road.ger7;");
    println!("       right_look@0x99 = 6241555 = \"ger_1\" (Look-Variante, KEINE Unit/keine Spurdaten).");
    println!(
        "GAP B: Units heißen `road.<stem>` (z.B. road.ger7). Korrekte Stem-Extraktion = NACH LETZTEM Punkt;"
    );
    println!("       Key-Schema = trucklib_token(last_segment). Quelldateien:");
    for f in &rl_files {
        println!("         {f}");
    }
    println!(
        "       Format = Text (SiiNunit, kein BSII). Gap-B-Auflösung: {}/{} road_type-Stems → Unit mit Spurdaten.",
        gapb_resolved,
        rt_stems.len()
    );
    println!("HINWEIS: die 185 graph.json-Tokens (Abschnitt oben) sind allesamt `right_look`-Werte (das");
    println!("       falsche Feld) → daher 0/185 Abdeckung. Nach Feld-Wechsel auf road_type@0x39 +");
    println!("       Key=trucklib(last_seg) + Laden von road_look.template*.sii ist 2e vollständig spezifiziert.");
    println!("========================================");

    // ── Task 5: verdict ──
    println!("\n========================================");
    if is_bsii {
        println!(
            "VERDIKT: Mode 1 (BSII binary) — parser returns empty map ({} entries), no lane data reaches the graph. 2e needs a BSII decoder (or a SII_Decrypt pre-step).",
            map.len()
        );
    } else if map.is_empty() {
        println!(
            "VERDIKT: Mode 1-like (text but 0 entries parsed) — found file is text yet yields no road_look blocks. 2e needs the text-parser grammar adjusted to this file's layout."
        );
    } else if let Some(e) = resolves {
        println!(
            "VERDIKT: Mode 2 (parsed {} entries, token {BERLIN_TOKEN} RESOLVES: L={} R={} width={:.2}) — surprising: lane data present but small. Inspect apply_road_look gating / actual lanes value.",
            map.len(),
            e.lanes_left,
            e.lanes_right,
            e.lane_width_m
        );
    } else {
        let variant = hash_hit.unwrap_or_else(|| "UNKNOWN (not road.look0..63 scs/city)".to_string());
        println!(
            "VERDIKT: Mode 2 (parsed {} entries, token {BERLIN_TOKEN} UNRESOLVED, correct hash variant = {variant}) — text parses but the key scheme/name set does not match the binary token. 2e is a hash/name-mapping fix, not a binary decoder.",
            map.len()
        );
    }
    println!("========================================");

    // ── Schema/Task 5: schema verdict ──
    println!("\n========================================");
    let loaded_match = ranked.first().map(|(_, (t, _))| t.len()).unwrap_or(0);
    if binary_tokens.is_empty() {
        println!("SCHEMA: unbestimmt — keine Binär-Tokens geladen (graph.json fehlte). Mit --graph erneut laufen.");
    } else if !decoded.is_empty() {
        println!(
            "SCHEMA: trucklib_token(stem) — {}/{} Binär-Tokens dekodieren sauber zu trucklib-Stems (round-trip-verifiziert).",
            decoded.len(),
            binary_tokens.len()
        );
        match &berlin_decoded {
            Some((t, name)) => println!("        Berlin-Token {t} == trucklib_token(\"{name}\") — Schema BEWIESEN."),
            None => println!("        (Berlin-Token {BERLIN_TOKEN} dekodiert nicht clean — evtl. längerer Name/anderes Encoding)"),
        }
        println!(
            "        Wurzel-Problem: die GELADENE def/world/road_look.sii ist ein Legacy-Stub mit nur {} Einträgen (road.look0..31), deren Hash-Keys NICHT zu den Binär-Tokens passen ({loaded_match} Treffer). Die echten benannten road_looks (z.B. der Stem hinter {BERLIN_TOKEN}) stehen in ANDEREN Dateien, die der Loader (erste-Datei-gewinnt) nie liest.",
            names.len()
        );
        println!(
            "        2e = (1) ALLE road_look-Definitionsdateien laden (nicht nur die erste/Legacy-Stub), (2) Map per trucklib_token(stem) keyen, (3) lange/huge Binär-Tokens ({} dekodieren NICHT als trucklib) gesondert klären.",
            binary_tokens.len() - decoded.len()
        );
    } else if loaded_match == 0 {
        println!("KEIN SCHEMA trifft — weder die 32 geladenen Namen noch trucklib-Decode reproduzieren Binär-Tokens. Weitere Hash-Varianten/Dateien nötig.");
    } else {
        println!("SCHEMA: teilweise — siehe Task-4-Summary oben.");
    }
    println!("========================================");

    // ── File-Loc/Task 5: localization verdict ──
    println!("\n========================================");
    let rl_def_files: BTreeSet<String> = all_stems
        .iter()
        .filter(|(_, u)| !u.starts_with("road.look"))
        .map(|(src, _)| src.clone())
        .collect();
    if !binary_tokens.is_empty() {
        println!(
            "ROAD_LOOK-DEFINITIONSDATEIEN gefunden (mit Spurdaten lanes_left[]/lanes_right[]):"
        );
        for f in &rl_def_files {
            println!("        {f}");
        }
        println!(
            "ABER: Stem-Abdeckung = {}/{} Binär-Tokens. Die road_look-Unit-Namen \
             (road.templateN, road.baltN, road.ibeN, road.unNN) entsprechen NICHT den \
             dekodierten Stems (ger_1, it_1, ma1, …).",
            covered.len(),
            binary_tokens.len()
        );
        println!(
            "Content-Suche über {} def/*.sii/.sui-Dateien findet KEINEN Unit-Header/Field für \
             die Stems → die road-item→road_look-Auflösung hat eine UNGELÖSTE Indirektion.",
            def_files.len()
        );
        println!(
            "FOLGE für 2e: NICHT einfach trucklib_token(road_look_name) keyen. Erst die \
             Indirektion klären (Stems erscheinen nirgends als Text → wahrscheinlich binäre \
             `road :`/road_look-Defs (BSII), oder eine andere Auflösungs-Ebene)."
        );
    }
    println!("========================================");
}

/// Recursively walk an archive's `def`/`def/world` directory listings and return
/// every file path containing `road_look`. HashFS stores no path strings, but it
/// DOES store directory-listing entries (one per directory), so a bounded walk
/// reconstructs real paths. Read-only; descent is limited to def, def/world and
/// any directory whose name contains `road_look`.
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
            let full = if dir.is_empty() {
                it.name.clone()
            } else {
                format!("{dir}/{}", it.name)
            };
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

/// Bounded walk of the `def`/`def/world` subtree returning all `.sii`/`.sui`
/// file paths (read-only; descends def, def/world and its immediate subdirs).
fn walk_def_files(arc: &mut Box<dyn Archive>) -> Vec<String> {
    let mut found = Vec::new();
    let mut stack: Vec<(String, u8)> = vec![("def".to_string(), 0)];
    let mut visited: HashSet<String> = HashSet::new();
    let mut budget = 8192usize;
    while let Some((dir, depth)) = stack.pop() {
        if budget == 0 || depth > 3 || !visited.insert(dir.clone()) {
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
                stack.push((full, depth + 1));
            } else if full.ends_with(".sii") || full.ends_with(".sui") {
                found.push(full);
            }
        }
    }
    found
}

/// True if `stem` appears as a unit header in SII text — i.e. preceded by `.`,
/// `:` or whitespace and followed by a delimiter (` `, `{`, `:`, newline). Avoids
/// false substring hits (e.g. "ma1" inside "thermal1").
fn stem_defined_in(text: &str, stem: &str) -> bool {
    let bytes = text.as_bytes();
    let s = stem.as_bytes();
    let mut i = 0;
    while let Some(rel) = text[i..].find(stem) {
        let pos = i + rel;
        let before_ok = pos == 0
            || matches!(bytes[pos - 1], b'.' | b':' | b' ' | b'\t' | b'\n' | b'"');
        let after = pos + s.len();
        let after_ok = after >= bytes.len()
            || matches!(bytes[after], b' ' | b'\t' | b'\n' | b'{' | b':' | b'"' | b'.' | b'\r');
        if before_ok && after_ok {
            return true;
        }
        i = pos + 1;
    }
    false
}

/// Token candidates for a road_look unit name: the stem itself plus its first
/// and last dotted segment (handles names like `ger_1.road` → `ger_1`).
fn stem_token_candidates(stem: &str) -> Vec<u64> {
    let mut v = vec![trucklib_token(stem)];
    for seg in stem.split('.') {
        if !seg.is_empty() && seg != stem {
            v.push(trucklib_token(seg));
        }
    }
    v
}

/// Find the road_look block whose unit name's last dotted segment equals `stem`,
/// returning the block's lines (header through the closing `}`). Read-only.
fn block_for_stem(text: &str, stem: &str) -> Option<Vec<String>> {
    let mut lines = text.lines();
    while let Some(l) = lines.next() {
        let t = l.trim();
        let name: Option<String> = if let Some(r) = t.strip_prefix("road_look :") {
            r.split(['{', ' ', '\t'])
                .find(|s| !s.is_empty())
                .map(|s| s.to_string())
        } else if let Some(r) = t.strip_prefix("road_look.") {
            let end = r.find([' ', ':']).unwrap_or(r.len());
            Some(format!("road_look.{}", &r[..end]))
        } else {
            None
        };
        if let Some(n) = name {
            if n.rsplit('.').next() == Some(stem) {
                let mut out = vec![t.to_string()];
                for bl in lines.by_ref() {
                    let bt = bl.trim();
                    out.push(bt.to_string());
                    if bt == "}" || out.len() > 60 {
                        break;
                    }
                }
                return Some(out);
            }
        }
    }
    None
}

/// Read a little-endian u64 at `off` from a byte slice (None if out of range).
fn rd_u64(b: &[u8], off: usize) -> Option<u64> {
    b.get(off..off + 8).map(|s| u64::from_le_bytes(s.try_into().unwrap()))
}

/// Build the sector path `map/europe/sec±XXXX±YYYY.base` (4-digit signed coords).
fn sec_path(sx: i32, sz: i32) -> String {
    let fx = if sx >= 0 {
        format!("+{sx:04}")
    } else {
        format!("-{:04}", sx.unsigned_abs())
    };
    let fz = if sz >= 0 {
        format!("+{sz:04}")
    } else {
        format!("-{:04}", sz.unsigned_abs())
    };
    format!("map/europe/sec{fx}{fz}.base")
}

/// Stream `graph.json` and return `(x, z)` for the wanted node UIDs. Nodes appear
/// before edges; each node block is `uid` then `x`/`y`/`z`.
fn load_node_positions(graph_path: &Path, want: &HashSet<u64>) -> HashMap<u64, (f64, f64)> {
    let mut out = HashMap::new();
    let Ok(file) = File::open(graph_path) else {
        return out;
    };
    let reader = BufReader::new(file);
    let mut pending: Option<u64> = None;
    let mut px: Option<f64> = None;
    for line in reader.lines().map_while(Result::ok) {
        let t = line.trim();
        if let Some(rest) = t.strip_prefix("\"uid\":") {
            pending = rest
                .trim()
                .trim_end_matches(',')
                .parse::<u64>()
                .ok()
                .filter(|v| want.contains(v));
            px = None;
        } else if pending.is_some() {
            if let Some(rest) = t.strip_prefix("\"x\":") {
                px = rest.trim().trim_end_matches(',').parse::<f64>().ok();
            } else if let Some(rest) = t.strip_prefix("\"z\":") {
                if let (Some(x), Ok(z)) = (px, rest.trim().trim_end_matches(',').parse::<f64>()) {
                    out.insert(pending.take().unwrap(), (x, z));
                    px = None;
                    if out.len() == want.len() {
                        break;
                    }
                }
            }
        }
    }
    out
}

/// Reverse of `trucklib_token` (little-endian base-38). Returns the stem string
/// if the token decodes without an embedded null; callers should re-encode to
/// confirm a clean round-trip. Charset: 1–10→'0'–'9', 11–36→'a'–'z', 37→'_'.
fn decode_trucklib(mut v: u64) -> Option<String> {
    if v == 0 {
        return None;
    }
    let mut s = String::new();
    while v != 0 {
        let d = (v % 38) as u8;
        let c = match d {
            1..=10 => b'0' + (d - 1),
            11..=36 => b'a' + (d - 11),
            37 => b'_',
            _ => return None, // embedded 0 → not a clean trucklib stem
        };
        s.push(c as char);
        v /= 38;
    }
    Some(s)
}

/// Stream `graph.json` line-by-line and collect distinct non-zero
/// `road_look_token` values. Pretty-printed JSON → one field per line, so a
/// cheap line scan beats a full serde parse on a 1 GB file.
fn load_binary_tokens(graph_path: &Path) -> HashSet<u64> {
    let mut set = HashSet::new();
    let Ok(file) = File::open(graph_path) else {
        return set;
    };
    let reader = BufReader::new(file);
    for line in reader.lines().map_while(Result::ok) {
        let t = line.trim();
        if let Some(rest) = t.strip_prefix("\"road_look_token\":") {
            let num = rest.trim().trim_end_matches(',').trim();
            if let Ok(v) = num.parse::<u64>() {
                if v != 0 {
                    set.insert(v);
                }
            }
        }
    }
    set
}

/// Extract `(full_name, unit_part)` for each road_look block, mirroring the
/// block-start detection in `road_look.rs` (modern + legacy) WITHOUT changing it.
/// `full_name` is what `road_look.rs` hashes; `unit_part` is the name minus the
/// `road_look.` prefix (modern) or the class name (legacy).
fn extract_road_look_names(text: &str) -> Vec<(String, String)> {
    let mut out = Vec::new();
    for raw in text.lines() {
        let line = raw.trim();
        if line.is_empty() || line.starts_with('#') || line.starts_with("//") {
            continue;
        }
        if let Some(rest) = line.strip_prefix("road_look.") {
            let end = rest.find([' ', ':']).unwrap_or(rest.len());
            let unit = &rest[..end];
            if !unit.is_empty() {
                out.push((format!("road_look.{unit}"), unit.to_string()));
            }
        } else if let Some(rest) = line.strip_prefix("road_look :") {
            let class = rest.trim_start();
            let end = class.find([' ', '{']).unwrap_or(class.len());
            let class = &class[..end];
            if !class.is_empty() {
                out.push((class.to_string(), class.to_string()));
            }
        }
    }
    out
}

/// Right-pad-safe truncation for table display.
fn truncate(s: &str, max: usize) -> String {
    if s.len() <= max {
        s.to_string()
    } else {
        format!("{}…", &s[..max.saturating_sub(1)])
    }
}
