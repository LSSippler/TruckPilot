//! GPS / navigation focus report for offline 1.60 signature research.

use std::collections::{BTreeMap, BTreeSet};

use super::code_heuristics::{format_hex_window, scan_code_refs, CodeRef, CodeRefKind};
use super::gps_semantic::{
    live_probe_from_semantic, render_gps_semantic_section, render_live_probe_candidate,
    scan_gps_semantic_patterns, GpsSemanticMatch, LiveProbeCandidate, SingletonRankHint,
};
use super::patterns::STATIC_GAME_CTRL_SLOTS;
use super::pe::PeFile;
use super::scan::PatternHit;
use super::strings::StringHit;
use super::xref::{find_heuristic_xrefs, XrefKind};
use super::SnapshotWithXrefs;

/// Legacy live-tested `game_ctrl + GPS` embed offset (1.59 hypothesis).
pub const LEGACY_GPS_OFFSET_HYPOTHESIS: usize = 0x3E30;

/// Substrings that identify high-value GPS/navigation RTTI strings.
pub const GPS_STRING_FOCUS_MARKERS: &[&str] = &[
    "profile_gps_message_t",
    "waypoints_entry_t",
    "avoid_points_entry_t",
    "online_gps_system",
    "gps_waypoint_storage",
    "stored_gps_ahead_waypoints",
    "stored_gps_behind_waypoints",
    "stored_online_gps_ahead_waypoints",
    "stored_online_gps_behind_waypoints",
    "g_gps_navigation",
    "gps_path",
];

const CONTEXT_RADIUS: usize = 0x100;
const MAX_GPS_STRINGS: usize = 20;
const SINGLETON_NEAR_RADIUS: usize = 0x2000;

#[derive(Debug, Clone)]
pub struct GpsFocusReport {
    pub gps_lea_section: Option<GpsLeaFocus>,
    pub gps_semantic: Vec<GpsSemanticMatch>,
    pub live_probe_candidate: Option<LiveProbeCandidate>,
    pub singleton_targets: Vec<SingletonCluster>,
    pub gps_strings: Vec<GpsStringFocus>,
    pub candidate_scores: Vec<CandidateScore>,
}

#[derive(Debug, Clone)]
pub struct GpsLeaFocus {
    pub hits: Vec<GpsLeaHitDetail>,
}

#[derive(Debug, Clone)]
pub struct GpsLeaHitDetail {
    pub hit_rva: usize,
    pub embedded_disp32: i32,
    pub context_before: String,
    pub context_after: String,
    pub nearby_code_refs: Vec<CodeRef>,
    pub nearby_rip_targets: Vec<usize>,
}

#[derive(Debug, Clone)]
pub struct SingletonCluster {
    pub target_rva: usize,
    pub hit_count: usize,
    pub patterns: Vec<String>,
    pub hit_rvas: Vec<usize>,
    pub in_data_section: bool,
    pub distance_from_anchor: Option<usize>,
    pub priority_rank: usize,
}

#[derive(Debug, Clone)]
pub struct GpsStringFocus {
    pub rva: usize,
    pub kind: String,
    pub text: String,
    pub xrefs: Vec<GpsStringXref>,
    pub priority: i32,
}

#[derive(Debug, Clone)]
pub struct GpsStringXref {
    pub text_rva: usize,
    pub kind: XrefKind,
    pub detail: String,
    pub context_bytes: String,
}

#[derive(Debug, Clone)]
pub struct CandidateScore {
    pub candidate: String,
    pub score: i32,
    pub reasons: Vec<String>,
    pub recommendation: String,
}

pub fn build_gps_focus_report(pe: &PeFile, snapshot: &SnapshotWithXrefs) -> GpsFocusReport {
    let anchor = detect_singleton_anchor(&snapshot.pattern_hits);
    let gps_lea_section = build_gps_lea_focus(pe, &snapshot.pattern_hits);
    let singleton_targets =
        cluster_singleton_targets(pe, &snapshot.pattern_hits, anchor);
    let singleton_ranks: Vec<SingletonRankHint> = singleton_targets
        .iter()
        .map(|c| SingletonRankHint {
            target_rva: c.target_rva,
            priority_rank: c.priority_rank,
        })
        .collect();
    let gps_semantic =
        scan_gps_semantic_patterns(pe, &snapshot.pattern_hits, &singleton_ranks);
    let live_probe_candidate = gps_semantic
        .first()
        .filter(|m| m.confidence >= 10)
        .map(live_probe_from_semantic);
    let gps_strings = build_gps_string_focus(pe, &snapshot.string_hits);
    let candidate_scores = score_candidates(
        &gps_lea_section,
        &gps_semantic,
        &singleton_targets,
        &gps_strings,
    );
    GpsFocusReport {
        gps_lea_section,
        gps_semantic,
        live_probe_candidate,
        singleton_targets,
        gps_strings,
        candidate_scores,
    }
}

fn detect_singleton_anchor(hits: &[PatternHit]) -> usize {
    hits.iter()
        .find(|h| h.pattern_name == "game_ctrl_load_lea_wc")
        .and_then(|h| h.rip_target_rva)
        .or_else(|| {
            hits.iter()
                .find(|h| h.pattern_name == "game_ctrl_load_v159")
                .and_then(|h| h.rip_target_rva)
        })
        .unwrap_or(STATIC_GAME_CTRL_SLOTS[0])
}

fn build_gps_lea_focus(pe: &PeFile, hits: &[PatternHit]) -> Option<GpsLeaFocus> {
    let lea_hits: Vec<_> = hits
        .iter()
        .filter(|h| h.pattern_name == "gps_lea_rsi_v158")
        .collect();
    if lea_hits.is_empty() {
        return None;
    }
    let details = lea_hits
        .iter()
        .filter_map(|h| {
            let disp = h.embedded_disp32?;
            let (before, after, refs) = text_context_around(pe, h.hit_rva, 10);
            let rip_targets: BTreeSet<_> = refs
                .iter()
                .filter(|r| {
                    matches!(
                        r.kind,
                        CodeRefKind::RipRelativeLea | CodeRefKind::RipRelativeMov
                    )
                })
                .map(|r| r.target_rva)
                .collect();
            Some(GpsLeaHitDetail {
                hit_rva: h.hit_rva,
                embedded_disp32: disp,
                context_before: before,
                context_after: after,
                nearby_rip_targets: rip_targets.into_iter().collect(),
                nearby_code_refs: refs,
            })
        })
        .collect();
    Some(GpsLeaFocus { hits: details })
}

fn text_context_around(
    pe: &PeFile,
    hit_rva: usize,
    pattern_len: usize,
) -> (String, String, Vec<CodeRef>) {
    let Some((window, start)) = pe.text_window(hit_rva, CONTEXT_RADIUS, CONTEXT_RADIUS + pattern_len)
    else {
        return ("<no .text>".into(), "<no .text>".into(), Vec::new());
    };
    let hit_off = hit_rva.saturating_sub(start);
    let before_end = hit_off.min(window.len());
    let after_start = (hit_off + pattern_len).min(window.len());
    let before = format_hex_window(&window[..before_end], start, 16);
    let after = format_hex_window(&window[after_start..], start + after_start, 16);
    let refs = scan_code_refs(&window, start);
    (before, after, refs)
}

fn cluster_singleton_targets(
    pe: &PeFile,
    hits: &[PatternHit],
    anchor: usize,
) -> Vec<SingletonCluster> {
    let mut map: BTreeMap<usize, SingletonCluster> = BTreeMap::new();
    for h in hits {
        let Some(target) = h.rip_target_rva else {
            continue;
        };
        let entry = map.entry(target).or_insert_with(|| SingletonCluster {
            target_rva: target,
            hit_count: 0,
            patterns: Vec::new(),
            hit_rvas: Vec::new(),
            in_data_section: pe.rva_in_section_name(target, ".data"),
            distance_from_anchor: Some(target.abs_diff(anchor)),
            priority_rank: 0,
        });
        entry.hit_count += 1;
        if !entry.patterns.contains(&h.pattern_name) {
            entry.patterns.push(h.pattern_name.clone());
        }
        entry.hit_rvas.push(h.hit_rva);
    }
    let mut clusters: Vec<_> = map.into_values().collect();
    for c in &mut clusters {
        c.patterns.sort();
        c.hit_rvas.sort();
        c.hit_rvas.dedup();
    }
    clusters.sort_by(|a, b| {
        let score_a = singleton_sort_key(a, anchor);
        let score_b = singleton_sort_key(b, anchor);
        score_b.cmp(&score_a).then_with(|| a.target_rva.cmp(&b.target_rva))
    });
    for (i, c) in clusters.iter_mut().enumerate() {
        c.priority_rank = i + 1;
    }
    clusters
}

fn singleton_sort_key(c: &SingletonCluster, anchor: usize) -> i32 {
    let mut score = 0i32;
    if c.target_rva.abs_diff(anchor) <= SINGLETON_NEAR_RADIUS {
        score += 10;
    }
    score += (c.patterns.len() as i32).saturating_mul(3);
    score += (c.hit_count as i32).min(20);
    if c.in_data_section {
        score += 5;
    }
    score
}

fn build_gps_string_focus(pe: &PeFile, strings: &[StringHit]) -> Vec<GpsStringFocus> {
    let mut focused: Vec<GpsStringFocus> = strings
        .iter()
        .filter_map(|s| {
            if !matches_gps_focus_marker(&s.text) {
                return None;
            }
            let kind = classify_gps_string(&s.text);
            let priority = gps_string_priority(&s.text, &kind);
            let xrefs = xrefs_for_string(pe, s);
            Some(GpsStringFocus {
                rva: s.rva,
                kind,
                text: s.text.clone(),
                xrefs,
                priority,
            })
        })
        .collect();
    focused.sort_by(|a, b| b.priority.cmp(&a.priority).then_with(|| a.rva.cmp(&b.rva)));
    focused.truncate(MAX_GPS_STRINGS);
    focused
}

fn matches_gps_focus_marker(text: &str) -> bool {
    let lower = text.to_ascii_lowercase();
    GPS_STRING_FOCUS_MARKERS
        .iter()
        .any(|m| lower.contains(&m.to_ascii_lowercase()))
}

fn classify_gps_string(text: &str) -> String {
    let lower = text.to_ascii_lowercase();
    if lower.contains("waypoints_entry_t") && lower.contains("profile_gps_message_t") {
        "profile_gps_waypoints".into()
    } else if lower.contains("avoid_points_entry_t") {
        "profile_gps_avoid".into()
    } else if lower.contains("online_gps_system") {
        "online_gps_system".into()
    } else if lower.contains("gps_waypoint_storage") || lower.contains("stored_gps") {
        "gps_waypoint_storage".into()
    } else if lower.contains("g_gps_navigation") {
        "g_gps_navigation".into()
    } else if lower.contains("gps_path") {
        "gps_path".into()
    } else if lower.contains("profile_gps_message_t") {
        "profile_gps_message".into()
    } else {
        "gps_other".into()
    }
}

fn gps_string_priority(text: &str, kind: &str) -> i32 {
    let lower = text.to_ascii_lowercase();
    let mut p = 0;
    if kind == "profile_gps_waypoints" {
        p += 10;
    }
    if lower.contains("profile_gps_message_t") {
        p += 8;
    }
    if lower.contains("online_gps_system") && !lower.contains("online_gps_system_u") {
        p += 6;
    }
    if lower.contains("gps_set") {
        p -= 3;
    }
    if lower.contains("array_local_t") || lower.contains("array_t") {
        p += 2;
    }
    p
}

fn xrefs_for_string(pe: &PeFile, s: &StringHit) -> Vec<GpsStringXref> {
    let mut out = Vec::new();
    for x in find_heuristic_xrefs(pe, &[s.clone()], 8) {
        let ctx = pe
            .read_at_rva(x.text_rva, 16)
            .map(|b| {
                b.iter()
                    .map(|byte| format!("{byte:02X}"))
                    .collect::<Vec<_>>()
                    .join(" ")
            })
            .unwrap_or_else(|| "read_failed".into());
        out.push(GpsStringXref {
            text_rva: x.text_rva,
            kind: x.kind,
            detail: x.detail.clone(),
            context_bytes: ctx,
        });
    }
    out
}

fn score_candidates(
    gps_lea: &Option<GpsLeaFocus>,
    gps_semantic: &[GpsSemanticMatch],
    singletons: &[SingletonCluster],
    gps_strings: &[GpsStringFocus],
) -> Vec<CandidateScore> {
    let mut scores = Vec::new();
    let profile_xref_count = gps_strings
        .iter()
        .filter(|s| s.kind.starts_with("profile_gps") && !s.xrefs.is_empty())
        .count();
    let top_singleton = singletons.first().map(|s| s.target_rva);

    for m in gps_semantic {
        scores.push(CandidateScore {
            candidate: format!(
                "semantic:game_ctrl+0x{:X}@0x{:X}",
                m.gps_candidate.offset, m.hit_rva
            ),
            score: m.confidence + 5,
            reasons: m
                .reasons
                .iter()
                .map(|r| format!("+ semantic: {r}"))
                .chain(std::iter::once("- no live validation yet".into()))
                .collect(),
            recommendation: m.recommendation.clone(),
        });
    }

    if let Some(lea) = gps_lea {
        for hit in &lea.hits {
            let disp = hit.embedded_disp32 as u32 as usize;
            let candidate = format!("gps_lea_rsi_v158+0x{disp:X}");
            let mut score = 0i32;
            let mut reasons = Vec::new();

            if lea.hits.len() == 1 {
                score += 3;
                reasons.push("+ unique gps_lea hit".into());
            }
            if (0x1000..=0x8000).contains(&disp) {
                score += 2;
                reasons.push("+ embedded offset in plausible struct range 0x1000..0x8000".into());
            }
            if disp != LEGACY_GPS_OFFSET_HYPOTHESIS {
                score += 1;
                reasons.push(format!(
                    "+ embedded offset differs from legacy 0x{LEGACY_GPS_OFFSET_HYPOTHESIS:X} hypothesis"
                ));
            } else {
                reasons.push("- matches legacy 0x3E30 hypothesis only".into());
            }
            if profile_xref_count > 0 {
                score += 4;
                reasons.push(format!(
                    "+ {profile_xref_count} profile_gps/waypoints strings with xrefs in focus set"
                ));
            } else {
                score -= 1;
                reasons.push("- no supporting profile_gps xrefs in focus set".into());
            }
            if top_singleton.is_some() && !hit.nearby_rip_targets.is_empty() {
                score += 2;
                reasons.push("+ nearby RIP targets in hit context window".into());
            }
            if hit.nearby_code_refs.is_empty() {
                score -= 1;
                reasons.push("- sparse code refs in ±0x100 context".into());
            }
            reasons.push("- no live validation yet".into());

            scores.push(CandidateScore {
                candidate,
                score,
                reasons,
                recommendation: "offline_review_before_live_probe".into(),
            });
        }
    }

    for s in gps_strings.iter().take(3) {
        if s.kind == "profile_gps_waypoints" || s.kind == "online_gps_system" {
            continue;
        }
        if s.text.contains("gps_set") || s.text.len() < 12 {
            let mut score = -2i32;
            let mut reasons = vec!["- generic or UI-ish GPS string".into()];
            if s.xrefs.is_empty() {
                score -= 1;
                reasons.push("- no supporting xrefs".into());
            }
            scores.push(CandidateScore {
                candidate: format!("gps_string@0x{:X}", s.rva),
                score,
                reasons,
                recommendation: "low_priority_offline_only".into(),
            });
        }
    }

    scores.sort_by(|a, b| b.score.cmp(&a.score).then_with(|| a.candidate.cmp(&b.candidate)));
    scores
}

pub fn render_gps_focus_report(report: &GpsFocusReport) -> String {
    let mut out = String::new();
    out.push_str("[gps_focus]\n");
    if let Some(ref lea) = report.gps_lea_section {
        out.push_str("gps_lea_rsi_v158:\n");
        out.push_str(&format!("  hits={}\n", lea.hits.len()));
        for h in &lea.hits {
            out.push_str(&format!("  hit_rva=0x{:X}\n", h.hit_rva));
            out.push_str(&format!("  embedded_disp32=0x{:X}\n", h.embedded_disp32 as u32));
            out.push_str("  context_before=\n");
            out.push_str(&h.context_before);
            out.push_str("  context_after=\n");
            out.push_str(&h.context_after);
            out.push_str("  nearby_calls_or_leas=\n");
            let (aligned, unaligned): (Vec<_>, Vec<_>) = h
                .nearby_code_refs
                .iter()
                .partition(|r| !r.maybe_unaligned);
            for r in &aligned {
                out.push_str(&format!(
                    "    at=0x{:X} kind={:?} target=0x{:X} aligned=true {}\n",
                    r.at_rva, r.kind, r.target_rva, r.detail
                ));
            }
            if !unaligned.is_empty() {
                out.push_str("  maybe_unaligned_calls_or_jmps=\n");
                for r in &unaligned {
                    out.push_str(&format!(
                        "    at=0x{:X} kind={:?} target=0x{:X} maybe_unaligned=true {}\n",
                        r.at_rva, r.kind, r.target_rva, r.detail
                    ));
                }
            }
            out.push_str("  nearby_rip_targets=\n");
            for t in &h.nearby_rip_targets {
                out.push_str(&format!("    0x{t:X}\n"));
            }
        }
    } else {
        out.push_str("gps_lea_rsi_v158: hits=0\n");
    }

    out.push_str("\n");
    out.push_str(&render_gps_semantic_section(&report.gps_semantic));
    if let Some(ref probe) = report.live_probe_candidate {
        out.push_str("\n");
        out.push_str(&render_live_probe_candidate(probe));
    }

    out.push_str("\n[singleton_targets]\n");
    for c in &report.singleton_targets {
        out.push_str(&format!(
            "target_rva=0x{:X} hit_count={} rank={} in_data={}\n",
            c.target_rva, c.hit_count, c.priority_rank, c.in_data_section
        ));
        out.push_str(&format!("  patterns={}\n", c.patterns.join(",")));
        out.push_str(&format!(
            "  hit_rvas={}\n",
            c.hit_rvas
                .iter()
                .map(|r| format!("0x{r:X}"))
                .collect::<Vec<_>>()
                .join(",")
        ));
        if let Some(d) = c.distance_from_anchor {
            out.push_str(&format!("  distance_from_anchor=0x{d:X}\n"));
        }
    }

    out.push_str("\n[gps_strings_focus]\n");
    for s in &report.gps_strings {
        out.push_str(&format!(
            "rva=0x{:X} kind={} priority={} text=\"{}\"\n",
            s.rva, s.kind, s.priority, s.text
        ));
        out.push_str("xrefs:\n");
        if s.xrefs.is_empty() {
            out.push_str("  (none found)\n");
        }
        for x in &s.xrefs {
            out.push_str(&format!(
                "  text_rva=0x{:X} kind={:?} {} context bytes={}\n",
                x.text_rva, x.kind, x.detail, x.context_bytes
            ));
        }
    }

    out.push_str("\n[candidate_score]\n");
    for c in &report.candidate_scores {
        out.push_str(&format!("candidate={} score={}\n", c.candidate, c.score));
        out.push_str("reasons:\n");
        for r in &c.reasons {
            out.push_str(&format!("  {r}\n"));
        }
        out.push_str(&format!("recommendation={}\n", c.recommendation));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ets2_bin_analyze::patterns::OFFLINE_PATTERNS;
    use crate::ets2_bin_analyze::pe::{build_minimal_pe64, PeFile};
    use crate::ets2_bin_analyze::scan::{scan_one_pattern, scan_patterns, PatternHit, rip_resolve_rva};
    use crate::ets2_bin_analyze::strings::scan_strings;
    use std::path::Path;

    fn gps_lea_pattern() -> &'static crate::ets2_bin_analyze::patterns::OfflinePattern {
        OFFLINE_PATTERNS
            .iter()
            .find(|p| p.name == "gps_lea_rsi_v158")
            .unwrap()
    }

    #[test]
    fn gps_lea_embedded_disp32_40f8() {
        let hit_rva = 0x1050usize;
        let off = hit_rva - 0x1000;
        let mut text = vec![0x90u8; 0x300];
        let pat = gps_lea_pattern();
        text[off..off + pat.pattern.len()].copy_from_slice(pat.pattern);
        text[off + 3..off + 7].copy_from_slice(&0x40F8u32.to_le_bytes());
        let pe = PeFile::from_bytes_for_test(
            Path::new("t.exe"),
            build_minimal_pe64(&text, b"profile_gps_message_t waypoints_entry_t\0"),
        )
        .unwrap();
        let hits = scan_one_pattern(&pe, pat, 4);
        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0].embedded_disp32, Some(0x40F8));
    }

    #[test]
    fn context_scanner_finds_rip_and_call() {
        let hit_rva = 0x1100usize;
        let mut text = vec![0x90u8; 0x400];
        let pat = gps_lea_pattern();
        let off = hit_rva - 0x1000;
        text[off..off + pat.pattern.len()].copy_from_slice(pat.pattern);
        text[off + 3..off + 7].copy_from_slice(&0x40F8u32.to_le_bytes());
        let lea_off = hit_rva + 0x20 - 0x1000;
        let target = 0x2000usize;
        let disp = (target as i64 - (hit_rva as i64 + 0x20 + 7)) as i32;
        text[lea_off..lea_off + 7].copy_from_slice(&[
            0x48,
            0x8D,
            0x0D,
            (disp as u32).to_le_bytes()[0],
            (disp as u32).to_le_bytes()[1],
            (disp as u32).to_le_bytes()[2],
            (disp as u32).to_le_bytes()[3],
        ]);
        let call_off = hit_rva + 0x40 - 0x1000;
        text[call_off] = 0xE8;
        text[call_off + 1..call_off + 5].copy_from_slice(&0x10i32.to_le_bytes());
        let pe = PeFile::from_bytes_for_test(Path::new("t.exe"), build_minimal_pe64(&text, b"x\0")).unwrap();
        let (_, _, refs) = text_context_around(&pe, hit_rva, pat.pattern.len());
        assert!(refs.iter().any(|r| r.kind == CodeRefKind::RipRelativeLea));
        assert!(refs.iter().any(|r| r.kind == CodeRefKind::CallRel32));
    }

    #[test]
    fn singleton_targets_grouped() {
        let mut text = vec![0x90u8; 0x200];
        let targets = [0x354F398usize, 0x354F428usize, 0x2EEDD68usize];
        for (i, &slot_disp) in [0x05u32, 0x15u32, 0x25u32].iter().enumerate() {
            let base = 0x1000 + i * 0x20;
            let off = base - 0x1000;
            text[off..off + 7].copy_from_slice(&[
                0x48,
                0x8B,
                0x0D,
                slot_disp.to_le_bytes()[0],
                slot_disp.to_le_bytes()[1],
                slot_disp.to_le_bytes()[2],
                slot_disp.to_le_bytes()[3],
            ]);
            let _ = targets;
        }
        let pe = PeFile::from_bytes_for_test(Path::new("t.exe"), build_minimal_pe64(&text, b"x\0")).unwrap();
        let mut hits = Vec::new();
        for i in 0..3 {
            let hit_rva = 0x1000 + i * 0x20;
            let disp = [0x05i32, 0x15, 0x25][i];
            hits.push(PatternHit {
                pattern_name: "game_ctrl_load_short".into(),
                hit_rva,
                context_hex: String::new(),
                rip_target_rva: Some(rip_resolve_rva(hit_rva, 7, disp)),
                embedded_disp32: None,
            });
        }
        hits.push(PatternHit {
            pattern_name: "game_ctrl_load_lea_wc".into(),
            hit_rva: 0x1060,
            context_hex: String::new(),
            rip_target_rva: Some(0x354F398),
            embedded_disp32: None,
        });
        let clusters = cluster_singleton_targets(&pe, &hits, 0x354F398);
        assert!(clusters.len() >= 2);
        assert_eq!(clusters[0].target_rva, 0x354F398);
    }

    #[test]
    fn gps_string_focus_finds_profile_gps() {
        let rdata = b"array_local_t<waypoints_entry_t, profile_gps_message_t>\0gps_set\0";
        let pe = PeFile::from_bytes_for_test(
            Path::new("t.exe"),
            build_minimal_pe64(&[0x90; 32], rdata),
        )
        .unwrap();
        let strings = scan_strings(&pe, false, 32);
        let snapshot = SnapshotWithXrefs {
            label: "t".into(),
            pattern_hits: vec![],
            string_hits: strings,
            xrefs: vec![],
        };
        let focused = build_gps_string_focus(&pe, &snapshot.string_hits);
        assert!(focused.iter().any(|s| s.kind == "profile_gps_waypoints"));
        assert!(focused.iter().any(|s| s.text.contains("profile_gps_message_t")));
    }

    #[test]
    fn score_prioritizes_semantic_over_generic() {
        const SINGLETON_RVA: usize = 0x354F398;
        let hit_rva = 0x1000 + 0x1DE;
        let mov_rva = 0x1000 + 0x114;
        let mut text = vec![0x90u8; 0x400];
        let mov_disp = (SINGLETON_RVA as i64 - (mov_rva as i64 + 7)) as i32;
        let mov_off = mov_rva - 0x1000;
        text[mov_off..mov_off + 7].copy_from_slice(&[
            0x48,
            0x8B,
            0x3D,
            mov_disp.to_le_bytes()[0],
            mov_disp.to_le_bytes()[1],
            mov_disp.to_le_bytes()[2],
            mov_disp.to_le_bytes()[3],
        ]);
        let pat = gps_lea_pattern();
        let lea_off = hit_rva - 0x1000;
        text[lea_off..lea_off + pat.pattern.len()].copy_from_slice(pat.pattern);
        text[lea_off + 3..lea_off + 7].copy_from_slice(&0x40F8u32.to_le_bytes());
        let store_off = (hit_rva + 0xD) - 0x1000;
        text[store_off..store_off + 7].copy_from_slice(&[
            0x48, 0x89, 0xB3, 0xB0, 0x00, 0x00, 0x00,
        ]);
        let rdata = b"array_t<waypoints_entry_t, profile_gps_message_t>\0gps_set\0";
        let pe = PeFile::from_bytes_for_test(Path::new("t.exe"), build_minimal_pe64(&text, rdata)).unwrap();
        let mut pattern_hits = scan_patterns(&pe, 8);
        if !pattern_hits.iter().any(|h| h.pattern_name == "game_ctrl_load_lea_wc") {
            pattern_hits.push(PatternHit {
                pattern_name: "game_ctrl_load_lea_wc".into(),
                hit_rva: mov_rva,
                context_hex: String::new(),
                rip_target_rva: Some(SINGLETON_RVA),
                embedded_disp32: None,
            });
        }
        let strings = scan_strings(&pe, false, 32);
        let snapshot = SnapshotWithXrefs {
            label: "t".into(),
            pattern_hits,
            string_hits: strings,
            xrefs: vec![],
        };
        let report = build_gps_focus_report(&pe, &snapshot);
        assert!(!report.candidate_scores.is_empty());
        assert!(
            report.candidate_scores[0]
                .candidate
                .starts_with("semantic:game_ctrl+0x40F8")
        );
        assert!(!report.gps_semantic.is_empty());
        assert!(report.live_probe_candidate.is_some());
        let generic = report
            .candidate_scores
            .iter()
            .find(|c| c.candidate.contains("gps_lea_rsi_v158+0x40F8"));
        if let Some(g) = generic {
            assert!(report.candidate_scores[0].score > g.score);
        }
    }

    #[test]
    fn context_no_false_jmp_from_mov_rbp_rcx() {
        let hit_rva = 0x1100usize;
        let mut text = vec![0x90u8; 0x400];
        let pat = gps_lea_pattern();
        let off = hit_rva - 0x1000;
        text[off..off + pat.pattern.len()].copy_from_slice(pat.pattern);
        text[off + 3..off + 7].copy_from_slice(&0x40F8u32.to_le_bytes());
        let trap_off = hit_rva - 0x17 - 0x1000;
        text[trap_off..trap_off + 3].copy_from_slice(&[0x48, 0x8B, 0xE9]);
        let pe = PeFile::from_bytes_for_test(Path::new("t.exe"), build_minimal_pe64(&text, b"x\0")).unwrap();
        let (_, _, refs) = text_context_around(&pe, hit_rva, pat.pattern.len());
        assert!(
            !refs.iter().any(|r| r.kind == CodeRefKind::JmpRel32 && !r.maybe_unaligned),
            "aligned JmpRel32 from 48 8B E9 must not appear"
        );
    }

    #[test]
    fn context_no_false_call_from_bt_eax() {
        let hit_rva = 0x1100usize;
        let mut text = vec![0x90u8; 0x400];
        let pat = gps_lea_pattern();
        let off = hit_rva - 0x1000;
        text[off..off + pat.pattern.len()].copy_from_slice(pat.pattern);
        let trap_off = hit_rva - 0x30 - 0x1000;
        text[trap_off..trap_off + 4].copy_from_slice(&[0x0F, 0xBA, 0xE8, 0x1F]);
        let pe = PeFile::from_bytes_for_test(Path::new("t.exe"), build_minimal_pe64(&text, b"x\0")).unwrap();
        let (_, _, refs) = text_context_around(&pe, hit_rva, pat.pattern.len());
        assert!(
            !refs.iter().any(|r| r.kind == CodeRefKind::CallRel32 && !r.maybe_unaligned),
            "aligned CallRel32 from 0F BA E8 must not appear"
        );
    }
}
