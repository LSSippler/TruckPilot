//! `sign-format-compare` — Phase 6.2b-Fix-2 expanded re-audit
//!
//! Read-only audit: dump sign-override-region bytes from ALL cleanly-parsed
//! Type-36 items across all discovered Vanilla sectors, plus the crashing
//! signs in the known crash-sector list. Tier 4 (bo>0 AND so>0) is the
//! prize sample class — Fix-1 missed these entirely because the original
//! 15-sector pool never produced one.
//!
//! Gate Guards:
//!   - Read-only. No change to skip_sign, parser, or DropTracer.
//!   - No code fix.

use std::io::Write as IoWrite;
use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;
use truckpilot_map_parser::{
    sector::{audit_sector, AuditReport},
    Archive, HashFsArchive, ModLoadOrder, ZipArchive,
};

/// Known crash sectors — always included even if dynamic discovery already
/// found them, so they are guaranteed to land in the report regardless of
/// the scan range.
const CRASH_REFERENCE_SECTORS: &[&str] = &[
    "map/europe/sec+0027+0022.base", // Class B: bo=0, so=3, attr_count garbage
    "map/europe/sec+0028+0024.base", // Class A: bo=12, Hebrew Mitzpe Ramon
    "map/europe/sec+0029+0024.base",
    "map/europe/sec+0029+0025.base",
    "map/europe/sec+0027+0025.base",
    "map/europe/sec+0014-0009.base",
    "map/europe/sec+0028+0015.base",
    "map/europe/sec+0026+0024.base",
    "map/europe/sec+0027+0016.base",
    // 4 new crashers from Fix-1 (post-revert these parse OK again):
    "map/europe/sec+0025+0025.base",
    "map/europe/sec+0026+0025.base",
    "map/europe/sec+0027+0015.base",
    "map/europe/sec+0027+0023.base",
];

#[derive(Parser)]
#[command(name = "sign-format-compare")]
struct Args {
    #[arg(long)]
    ets2_dir: PathBuf,

    #[arg(long)]
    mods_dir: Option<PathBuf>,

    #[arg(long, default_value = "outputs/2026-05-17")]
    output_dir: PathBuf,

    /// Output filename inside output_dir.
    #[arg(long, default_value = "sign_format_compare_v3.md")]
    output_name: String,

    /// Per non-tier-4 cap (Tier 1/2/3). Tier 4 is uncapped.
    #[arg(long, default_value_t = 5)]
    per_tier_cap: usize,

    /// Scan range for dynamic sector discovery (both x and z, symmetric).
    #[arg(long, default_value_t = 50)]
    scan_range: i32,
}

fn default_mods_dir() -> PathBuf {
    dirs::document_dir()
        .or_else(dirs::home_dir)
        .unwrap_or_default()
        .join("Euro Truck Simulator 2/mod")
}

/// One captured sign sample.
#[derive(Clone)]
struct SignSample {
    sector_path: String,
    item_index: usize,
    is_crash_item: bool,
    body_start: usize,
    body_end: Option<usize>,
    board_count: u8,
    template_len: u64,
    template_str: String,

    /// Offset of override region start (right after template), relative to
    /// body_start.
    override_start_rel: usize,

    /// First 128 bytes of override region.
    override_bytes: Vec<u8>,

    /// Naive u32 at override_start+0 (current code calls this
    /// board_override_count).
    bo_count_naive: u32,

    /// Replay of skip_sign_board_override_list: cursor offset (relative to
    /// body_start) where the per-item iteration ended. Equal to
    /// override_start_rel + 4 when bo_count_naive==0.
    /// Computed only when the bytes fit in the available window.
    bo_end_rel: Option<usize>,

    /// Naive u32 read at bo_end_rel (current code calls this
    /// sign_override_count). None if bo_end is outside the window.
    so_count_naive: Option<u32>,

    /// First 32 bytes starting at bo_end_rel — covers so_count (4B) and the
    /// first sign-override item.
    boundary_bytes: Vec<u8>,

    /// First 32 bytes of the first sign_override_item (= bo_end_rel + 4).
    first_override_bytes: Vec<u8>,

    /// Tier 0..4 (computed once at sample creation for ranking).
    tier: u8,
}

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::WARN)
        .with_writer(std::io::stderr)
        .init();

    let args = Args::parse();
    let mods_dir = args.mods_dir.clone().unwrap_or_else(default_mods_dir);
    std::fs::create_dir_all(&args.output_dir)?;

    eprintln!("[1/4] Opening archives ...");
    let order = ModLoadOrder::from_directories(&args.ets2_dir, &mods_dir)
        .context("build mod load order")?;
    let mut archives: Vec<Box<dyn Archive>> = Vec::new();
    for entry in &order.entries {
        let arc: Box<dyn Archive> = match HashFsArchive::open(&entry.path) {
            Ok(a) => Box::new(a),
            Err(_) => match ZipArchive::open(&entry.path) {
                Ok(a) => Box::new(a),
                Err(_) => continue,
            },
        };
        archives.push(arc);
    }
    eprintln!("      {} archives ready", archives.len());

    eprintln!(
        "[2/4] Discovering sectors in range x,z ∈ [{}..{}] ...",
        -args.scan_range, args.scan_range
    );
    let mut discovered: Vec<String> = Vec::new();
    let r = args.scan_range;
    for x in -r..=r {
        for z in -r..=r {
            let path = format!("map/europe/sec{:+05}{:+05}.base", x, z);
            if archives.iter().any(|a| a.contains(&path)) {
                discovered.push(path);
            }
        }
    }
    // Make sure crash-reference sectors are always included, dedup.
    for &p in CRASH_REFERENCE_SECTORS {
        if !discovered.iter().any(|d| d == p) {
            discovered.push(p.to_string());
        }
    }
    discovered.sort();
    discovered.dedup();
    eprintln!("      {} sectors discovered", discovered.len());

    eprintln!("[3/4] Capturing sign samples ...");
    let mut all_samples: Vec<SignSample> = Vec::new();
    let total = discovered.len();
    for (idx, sector_path) in discovered.iter().enumerate() {
        let data_opt: Option<Vec<u8>> = archives
            .iter_mut()
            .rev()
            .find_map(|arc| arc.read_path(sector_path).ok());
        let Some(data) = data_opt else { continue };
        let samples = analyze_sector(&data, sector_path, args.per_tier_cap);
        if !samples.is_empty() && (idx % 20 == 0 || samples.iter().any(|s| s.tier == 4)) {
            let t4 = samples.iter().filter(|s| s.tier == 4).count();
            eprintln!(
                "      [{}/{}] {} → {} samples (tier4={})",
                idx + 1,
                total,
                short_sector(sector_path),
                samples.len(),
                t4
            );
        }
        all_samples.extend(samples);
    }

    eprintln!("[4/4] Writing report ...");
    let md_path = args.output_dir.join(&args.output_name);
    write_report(&md_path, &all_samples)?;
    eprintln!("      {}", md_path.display());

    Ok(())
}

fn short_sector(path: &str) -> &str {
    path.rsplit('/').next().unwrap_or(path).trim_end_matches(".base")
}

fn analyze_sector(data: &[u8], sector_path: &str, per_tier_cap: usize) -> Vec<SignSample> {
    let report: AuditReport = audit_sector(data);
    let mut working: Vec<SignSample> = Vec::new();
    let mut crash: Option<SignSample> = None;

    for item in &report.items {
        if item.item_type != 36 {
            continue;
        }
        let body_start = item.start_offset + 4;
        if let Some(s) = decode_sign(
            data,
            body_start,
            Some(item.end_offset),
            sector_path,
            item.index,
            false,
        ) {
            working.push(s);
        }
    }

    if let Some(f) = &report.failure {
        if f.raw_type == 36 {
            crash = decode_sign(data, f.error_offset + 4, None, sector_path, f.item_index, true);
        }
    }

    // Bucket per tier, cap non-tier-4 lists.
    let mut by_tier: [Vec<SignSample>; 5] = Default::default();
    for s in working {
        let t = s.tier as usize;
        by_tier[t].push(s);
    }
    for (t, bucket) in by_tier.iter_mut().enumerate() {
        bucket.sort_by_key(|s| s.item_index);
        if t <= 3 {
            bucket.truncate(per_tier_cap);
        }
    }

    let mut samples = Vec::new();
    if let Some(c) = crash {
        samples.push(c);
    }
    // Order: tier 4 first, then 3, 2, 1, 0
    for bucket in by_tier.iter_mut().rev() {
        samples.append(bucket);
    }
    samples
}

fn decode_sign(
    data: &[u8],
    body_start: usize,
    body_end: Option<usize>,
    sector_path: &str,
    item_index: usize,
    is_crash: bool,
) -> Option<SignSample> {
    // Sign body layout up to board_count: 53 (kdop) + 8 (model) + 8 (node_uid) +
    // 8 (look) + 8 (variant) + 1 (board_count) = 86 bytes
    if body_start + 86 > data.len() {
        return None;
    }
    let board_count = data[body_start + 85];
    if board_count > 30 {
        return None;
    }
    let boards_size = (board_count as usize) * 24;
    let template_len_offset = body_start + 86 + boards_size;
    if template_len_offset + 8 > data.len() {
        return None;
    }
    let template_len = u64::from_le_bytes(
        data[template_len_offset..template_len_offset + 8].try_into().unwrap(),
    );
    if template_len > 256 {
        return None;
    }
    let template_start = template_len_offset + 8;
    let template_end = template_start + template_len as usize;
    if template_end > data.len() {
        return None;
    }
    let template_str = String::from_utf8_lossy(&data[template_start..template_end]).to_string();

    let override_start_abs = template_end;
    let override_start_rel = override_start_abs - body_start;

    // Window we are allowed to look at (capped by body_end for working items,
    // or by data.len() for crash items where end_offset is unknown).
    let window_end = if let Some(be) = body_end { be } else { data.len() };

    // ── First 128 bytes of override region (full snapshot) ─────────────────
    let override_capture_end = (override_start_abs + 128).min(data.len()).min(window_end);
    let override_bytes = if override_capture_end > override_start_abs {
        data[override_start_abs..override_capture_end].to_vec()
    } else {
        Vec::new()
    };

    let bo_count_naive = if override_bytes.len() >= 4 {
        u32::from_le_bytes(override_bytes[0..4].try_into().unwrap())
    } else {
        0
    };

    // ── Replay skip_sign_board_override_list to find the boundary ──────────
    // Per-item layout (current code): skip_token(8B) + flags u8 +
    //   if flags & 0x01: 2B + if flags & 0x02: skip_token(8B).
    let mut bo_end_rel: Option<usize> = None;
    let mut cursor_abs = override_start_abs + 4;
    let mut bo_replay_ok = true;
    if bo_count_naive < 10_000 {
        // sanity: don't iterate insane counts
        for _ in 0..bo_count_naive {
            if cursor_abs + 8 > window_end {
                bo_replay_ok = false;
                break;
            }
            cursor_abs += 8; // skip_token
            if cursor_abs + 1 > window_end {
                bo_replay_ok = false;
                break;
            }
            let flags = data[cursor_abs];
            cursor_abs += 1;
            if flags & 0x01 != 0 {
                if cursor_abs + 2 > window_end {
                    bo_replay_ok = false;
                    break;
                }
                cursor_abs += 2;
            }
            if flags & 0x02 != 0 {
                if cursor_abs + 8 > window_end {
                    bo_replay_ok = false;
                    break;
                }
                cursor_abs += 8;
            }
        }
        if bo_replay_ok {
            bo_end_rel = Some(cursor_abs - body_start);
        }
    }

    // ── Boundary bytes (32B starting at bo_end_rel) ────────────────────────
    let mut boundary_bytes = Vec::new();
    let mut so_count_naive: Option<u32> = None;
    let mut first_override_bytes = Vec::new();
    if let Some(boe) = bo_end_rel {
        let abs = body_start + boe;
        let take_end = (abs + 32).min(data.len()).min(window_end);
        if take_end > abs {
            boundary_bytes = data[abs..take_end].to_vec();
        }
        if boundary_bytes.len() >= 4 {
            so_count_naive =
                Some(u32::from_le_bytes(boundary_bytes[0..4].try_into().unwrap()));
        }
        // First override item starts at abs + 4
        let foi = abs + 4;
        let foi_end = (foi + 32).min(data.len()).min(window_end);
        if foi_end > foi {
            first_override_bytes = data[foi..foi_end].to_vec();
        }
    }

    // ── Tier ranking ───────────────────────────────────────────────────────
    let bo_ok = (1..1000).contains(&bo_count_naive);
    let so_ok = matches!(so_count_naive, Some(v) if (1..1000).contains(&v));
    let has_tmpl = template_len > 0;
    let tier: u8 = if bo_ok && so_ok {
        4
    } else if so_ok {
        3
    } else if bo_ok {
        2
    } else if has_tmpl {
        1
    } else {
        0
    };

    Some(SignSample {
        sector_path: sector_path.to_string(),
        item_index,
        is_crash_item: is_crash,
        body_start,
        body_end,
        board_count,
        template_len,
        template_str,
        override_start_rel,
        override_bytes,
        bo_count_naive,
        bo_end_rel,
        so_count_naive,
        boundary_bytes,
        first_override_bytes,
        tier,
    })
}

fn write_report(path: &std::path::Path, samples: &[SignSample]) -> Result<()> {
    let mut f = std::fs::File::create(path)?;
    macro_rules! w {
        ($($arg:tt)*) => { writeln!(f, $($arg)*)?; }
    }

    let working: Vec<&SignSample> = samples.iter().filter(|s| !s.is_crash_item).collect();
    let crashing: Vec<&SignSample> = samples.iter().filter(|s| s.is_crash_item).collect();

    let mut tier_counts = [0usize; 5];
    for s in &working {
        tier_counts[s.tier as usize] += 1;
    }

    w!("# Sign Format Compare v3 — Phase 6.2b-Fix-2");
    w!();
    w!("Re-audit with dynamic sector discovery. Goal: capture Tier-4 samples");
    w!("(`board_override_count > 0` AND `sign_override_count > 0`) which Fix-1");
    w!("missed entirely.");
    w!();
    w!("## 1. Tier Counts");
    w!();
    w!("| Tier | Definition | Working count |");
    w!("|---|---|---|");
    w!("| 4 | bo>0 AND so>0 (full override section) | **{}** |", tier_counts[4]);
    w!("| 3 | so>0 only (no board overrides) | {} |", tier_counts[3]);
    w!("| 2 | bo>0 only (board overrides but no sign overrides) | {} |", tier_counts[2]);
    w!("| 1 | template_len>0 but no overrides | {} |", tier_counts[1]);
    w!("| 0 | template_len==0 (no override section) | {} |", tier_counts[0]);
    w!();
    w!("**Totals**: {} working / {} crash", working.len(), crashing.len());
    w!();

    // ── Tier 4 — full hex dumps ─────────────────────────────────────────────
    w!("## 2. Tier 4 Working Signs (bo>0 AND so>0) — FULL DUMPS");
    w!();
    if tier_counts[4] == 0 {
        w!("**⚠ No Tier-4 samples found.**");
        w!();
        w!("Implication: in every cleanly-parsed sign across all discovered sectors,");
        w!("either `bo_count_naive==0` (Vanilla pattern, supports the padding hypothesis)");
        w!("or the replay of the alleged `skip_sign_board_override_list` produced an");
        w!("invalid `so_count`. If the field at override_start were a real");
        w!("board_override_count, we would expect non-zero values that ALSO");
        w!("lead to a sane so_count — and we are not seeing that.");
        w!();
    } else {
        let mut shown = 0usize;
        for s in &working {
            if s.tier != 4 {
                continue;
            }
            shown += 1;
            w!("### T4.{}: `{}` Item #{}", shown, short_sector(&s.sector_path), s.item_index);
            w!();
            w!(
                "- board_count={}, template_len={}, template=`{}`",
                s.board_count, s.template_len, s.template_str
            );
            w!(
                "- override_start_rel={}, bo_count={}, bo_end_rel={:?}, so_count={:?}",
                s.override_start_rel, s.bo_count_naive, s.bo_end_rel, s.so_count_naive
            );
            if let Some(be) = s.body_end {
                w!(
                    "- body_end={}, override region size = {} bytes",
                    be, be.saturating_sub(s.body_start + s.override_start_rel)
                );
            }
            w!();
            w!("**Override region (first 128B from override_start):**");
            w!("```");
            w!("{}", hex_dump(&s.override_bytes, s.override_start_rel));
            w!("```");
            if !s.boundary_bytes.is_empty() {
                w!();
                w!("**Boundary (32B at bo_end_rel = {:?}):**", s.bo_end_rel);
                w!("```");
                w!("{}", hex_dump(&s.boundary_bytes, s.bo_end_rel.unwrap_or(0)));
                w!("```");
            }
            if !s.first_override_bytes.is_empty() {
                w!();
                w!("**First sign_override_item (32B at bo_end_rel + 4):**");
                w!("```");
                w!(
                    "{}",
                    hex_dump(&s.first_override_bytes, s.bo_end_rel.unwrap_or(0) + 4)
                );
                w!("```");
            }
            w!();
        }
    }

    // ── Tier 3 — first override item per sample ─────────────────────────────
    w!("## 3. Tier 3 Working Signs (so>0, bo==0) — Per-Override Bytes");
    w!();
    w!(
        "These have `bo_count_naive==0` (cursor reaches `so_count` at override_start+4)");
    w!("but `so_count_naive>0`, so the bytes of the first sign-override item are visible.");
    w!();
    let mut tier3_shown = 0usize;
    for s in &working {
        if s.tier != 3 || tier3_shown >= 12 {
            continue;
        }
        tier3_shown += 1;
        w!(
            "### T3.{}: `{}` Item #{} — so={:?}, bo_end_rel={:?}",
            tier3_shown,
            short_sector(&s.sector_path),
            s.item_index,
            s.so_count_naive,
            s.bo_end_rel
        );
        w!();
        w!("- template=`{}`", s.template_str);
        w!("```");
        w!(
            "{}",
            hex_dump(&s.first_override_bytes, s.bo_end_rel.unwrap_or(0) + 4)
        );
        w!("```");
        w!();
    }

    // ── Tier 2 — boundary bytes (no so, but bo>0) ──────────────────────────
    w!("## 4. Tier 2 Working Signs (bo>0, so==0/garbage) — Boundary Bytes");
    w!();
    w!(
        "These have `bo_count_naive>0` (the alleged board-override section)");
    w!("but the cursor after replay produces `so_count_naive==0/garbage`.");
    w!("Either the per-bo-item layout is wrong (cursor lands at wrong place)");
    w!("or there's an extra field, or the bo field isn't really a count.");
    w!();
    let mut tier2_shown = 0usize;
    for s in &working {
        if s.tier != 2 || tier2_shown >= 12 {
            continue;
        }
        tier2_shown += 1;
        w!(
            "### T2.{}: `{}` Item #{} — bo={}, bo_end_rel={:?}, so_naive={:?}",
            tier2_shown,
            short_sector(&s.sector_path),
            s.item_index,
            s.bo_count_naive,
            s.bo_end_rel,
            s.so_count_naive
        );
        w!();
        w!("- template=`{}`", s.template_str);
        w!();
        w!("**Override region (start):**");
        w!("```");
        w!("{}", hex_dump(&s.override_bytes, s.override_start_rel));
        w!("```");
        w!();
    }

    // ── Crash samples ──────────────────────────────────────────────────────
    w!("## 5. Crash Samples (Type-36 failures)");
    w!();
    w!("| # | Sector | Item idx | bo_count | bo_end_rel | so_count | crash class |");
    w!("|---|---|---|---|---|---|---|");
    for (i, s) in crashing.iter().enumerate() {
        let class = match (s.bo_count_naive, s.so_count_naive) {
            (0, Some(v)) if v < 1000 => "B (attr_count garbage downstream)",
            _ => "A (so_count garbage at boundary)",
        };
        w!(
            "| {} | `{}` | {} | {} | {:?} | {:?} | {} |",
            i + 1,
            short_sector(&s.sector_path),
            s.item_index,
            s.bo_count_naive,
            s.bo_end_rel,
            s.so_count_naive,
            class,
        );
    }
    w!();
    for (i, s) in crashing.iter().enumerate() {
        w!(
            "### C{}: `{}` Item #{} (CRASH)",
            i + 1,
            short_sector(&s.sector_path),
            s.item_index
        );
        w!();
        w!(
            "- board_count={}, template_len={}, template=`{}`",
            s.board_count, s.template_len, s.template_str
        );
        w!(
            "- override_start_rel={}, bo_count={}, bo_end_rel={:?}, so_count={:?}",
            s.override_start_rel, s.bo_count_naive, s.bo_end_rel, s.so_count_naive
        );
        w!();
        w!("**Override region:**");
        w!("```");
        w!("{}", hex_dump(&s.override_bytes, s.override_start_rel));
        w!("```");
        if !s.boundary_bytes.is_empty() {
            w!();
            w!("**Boundary (32B at bo_end_rel):**");
            w!("```");
            w!("{}", hex_dump(&s.boundary_bytes, s.bo_end_rel.unwrap_or(0)));
            w!("```");
        }
        if !s.first_override_bytes.is_empty() {
            w!();
            w!("**First (alleged) sign_override_item (32B):**");
            w!("```");
            w!(
                "{}",
                hex_dump(&s.first_override_bytes, s.bo_end_rel.unwrap_or(0) + 4)
            );
            w!("```");
        }
        w!();
    }

    // ── Side-by-side ───────────────────────────────────────────────────────
    let t4_sample = working.iter().find(|s| s.tier == 4).copied();
    let t3_sample = working.iter().find(|s| s.tier == 3).copied();

    let class_a_crash = crashing.iter().find(|s| {
        s.bo_count_naive > 0 || !matches!(s.so_count_naive, Some(v) if v < 1000)
    }).copied();
    let class_b_crash = crashing
        .iter()
        .find(|s| s.bo_count_naive == 0 && matches!(s.so_count_naive, Some(v) if v < 1000))
        .copied();

    w!("## 6. Side-by-Side Comparisons");
    w!();

    write_side_by_side(&mut f, "Tier 4 working vs Class A crash", t4_sample, class_a_crash)?;
    write_side_by_side(&mut f, "Tier 3 working vs Class B crash", t3_sample, class_b_crash)?;

    // ── Hypotheses ─────────────────────────────────────────────────────────
    w!("## 7. Hypotheses (to discuss after review)");
    w!();
    w!("### Class A (`so_count` garbage at boundary)");
    w!();
    w!("Crash sectors: {}", crashing.iter()
        .filter(|s| s.bo_count_naive > 0 || !matches!(s.so_count_naive, Some(v) if v < 1000))
        .count());
    w!();
    w!("If Tier 4 working samples exist: their `bo_count_naive` is a real count");
    w!("and the per-bo-item layout in `skip_sign_board_override_list` works for");
    w!("them. The crash sectors must have a DIFFERENT per-bo-item layout, OR a");
    w!("per-bo-item that hits an unhandled `flags` bit.");
    w!();
    w!("If Tier 4 working samples are ZERO: the field is NOT a real count in");
    w!("Vanilla — it's reserved/padding (always 0). The crash sectors are then");
    w!("ProMods-format-specific, and Vanilla parser CAN'T handle them. The fix");
    w!("would need to skip the entire override section in ProMods-format signs,");
    w!("or detect the difference.");
    w!();
    w!("### Class B (`attr_count` garbage in per-override iteration)");
    w!();
    w!("Crash sector: sec+0027+0022 #5 (bo=0, so=3).");
    w!();
    w!("Look at Tier 3 first-override hex dumps. Current code reads each");
    w!("sign-override-item as `u32 + skip_token(8B) + u32 attr_count`. If the");
    w!("Tier-3 working samples show a different leading byte pattern (e.g.");
    w!("three f32 floats = position Vec3), the per-override layout is wrong.");
    w!();
    w!("Expected from Phase-6.2b-Fix-1 status report: bytes after `so_count`");
    w!("in sec+0027+0022 look like `52 00 05 23 30 b2 c8 35 36 54 da 47 …` —");
    w!("the `36 54 da 47` reads as f32 ≈ 112000m (world x). If Tier 3 working");
    w!("samples show the same Vec3-shaped pattern, we have the fix.");
    w!();

    Ok(())
}

fn write_side_by_side(
    f: &mut std::fs::File,
    title: &str,
    a: Option<&SignSample>,
    b: Option<&SignSample>,
) -> Result<()> {
    macro_rules! w {
        ($($arg:tt)*) => { writeln!(f, $($arg)*)?; }
    }
    w!("### {}", title);
    w!();
    let (Some(a), Some(b)) = (a, b) else {
        w!("*Not available (one or both sides missing).*");
        w!();
        return Ok(());
    };
    w!("| Field | A: `{}` #{} | B: `{}` #{} |",
        short_sector(&a.sector_path), a.item_index,
        short_sector(&b.sector_path), b.item_index);
    w!("|---|---|---|");
    w!("| board_count | {} | {} |", a.board_count, b.board_count);
    w!("| template_len | {} | {} |", a.template_len, b.template_len);
    w!("| bo_count | {} | {} |", a.bo_count_naive, b.bo_count_naive);
    w!("| bo_end_rel | {:?} | {:?} |", a.bo_end_rel, b.bo_end_rel);
    w!("| so_count | {:?} | {:?} |", a.so_count_naive, b.so_count_naive);
    w!();
    let na = a.first_override_bytes.len().min(32);
    let nb = b.first_override_bytes.len().min(32);
    if na == 0 && nb == 0 {
        w!("*No first-override bytes available.*");
        w!();
        return Ok(());
    }
    w!("**First override-item bytes (32B):**");
    w!();
    w!("```");
    w!("offset  A                                                  B");
    let rows = na.max(nb).div_ceil(8);
    for row in 0..rows {
        let off = row * 8;
        let ah = (off..off + 8)
            .map(|i| if i < na { format!("{:02x}", a.first_override_bytes[i]) } else { "  ".into() })
            .collect::<Vec<_>>()
            .join(" ");
        let bh = (off..off + 8)
            .map(|i| if i < nb { format!("{:02x}", b.first_override_bytes[i]) } else { "  ".into() })
            .collect::<Vec<_>>()
            .join(" ");
        w!("+{:03}    {}    {}", off, ah, bh);
    }
    w!("```");
    w!();
    Ok(())
}

fn hex_dump(data: &[u8], start_offset: usize) -> String {
    let mut out = String::new();
    for (i, chunk) in data.chunks(16).enumerate() {
        let offset = start_offset + i * 16;
        let hex: String = chunk.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(" ");
        let ascii: String = chunk.iter()
            .map(|&b| if (32..127).contains(&b) { b as char } else { '.' })
            .collect();
        let pad = "   ".repeat(16 - chunk.len());
        out.push_str(&format!("+{offset:04}  {hex}{pad}  |{ascii}|\n"));
    }
    out
}
