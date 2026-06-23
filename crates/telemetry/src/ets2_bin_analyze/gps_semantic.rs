//! Semantic GPS pattern recognition around `gps_lea_rsi_v158` hits (offline only).

use super::pe::PeFile;
use super::scan::{PatternHit, rip_resolve_rva};

const LEGACY_GPS_OFFSET_HYPOTHESIS: usize = 0x3E30;

const SEMANTIC_LOOKBACK: usize = 0x120;
const SEMANTIC_LOOKAHEAD: usize = 0x40;

/// Parsed `mov rdi,[rip+disp32] … lea rsi,[rdi+disp32] … mov [rbx+disp32],rsi` chain.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GpsSemanticMatch {
    pub pattern: String,
    pub hit_rva: usize,
    pub singleton_load: SingletonLoad,
    pub gps_candidate: GpsCandidate,
    pub store: Option<GpsStore>,
    pub confidence: i32,
    pub reasons: Vec<String>,
    pub recommendation: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SingletonLoad {
    pub at_rva: usize,
    pub register: String,
    pub singleton_rva: usize,
    pub singleton_rank: Option<usize>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GpsCandidate {
    pub at_rva: usize,
    pub base_register: String,
    pub offset: usize,
    pub expression: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GpsStore {
    pub at_rva: usize,
    pub store_offset: usize,
    pub target: String,
    pub source: String,
}

/// Machine-readable live-probe hint block (report only — no DLL wiring).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LiveProbeCandidate {
    pub name: String,
    pub singleton_rva: usize,
    pub offset: usize,
    pub derived_address_expression: String,
    pub probe_type: String,
    pub allowed_reads: Vec<String>,
    pub not_allowed: Vec<String>,
    pub stop_condition: String,
}

/// Singleton rank hint for semantic confidence scoring.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SingletonRankHint {
    pub target_rva: usize,
    pub priority_rank: usize,
}

pub fn scan_gps_semantic_patterns(
    pe: &PeFile,
    hits: &[PatternHit],
    singleton_ranks: &[SingletonRankHint],
) -> Vec<GpsSemanticMatch> {
    hits.iter()
        .filter(|h| h.pattern_name == "gps_lea_rsi_v158")
        .filter_map(|h| {
            let offset = h.embedded_disp32? as u32 as usize;
            scan_one_semantic(pe, h.hit_rva, offset, singleton_ranks)
        })
        .collect()
}

fn scan_one_semantic(
    pe: &PeFile,
    hit_rva: usize,
    gps_offset: usize,
    singleton_ranks: &[SingletonRankHint],
) -> Option<GpsSemanticMatch> {
    let lookback = hit_rva.saturating_sub(SEMANTIC_LOOKBACK);
    let lookahead_end = hit_rva.saturating_add(SEMANTIC_LOOKAHEAD);
    let window = pe.read_rva_range(lookback, lookahead_end)?;
    let hit_off = hit_rva - lookback;

    let singleton_load = find_singleton_mov_rdi(&window, lookback, hit_off)?;
    let singleton_rank = singleton_ranks
        .iter()
        .find(|c| c.target_rva == singleton_load.singleton_rva)
        .map(|c| c.priority_rank);

    let store = find_mov_rbx_store_rsi(&window, lookback, hit_off + 7);

    let mut confidence = 0i32;
    let mut reasons = Vec::new();

    if singleton_rank == Some(1) {
        confidence += 5;
        reasons.push("+ singleton target is top-ranked game_ctrl candidate".into());
    } else if singleton_rank.is_some() {
        confidence += 2;
        reasons.push("+ singleton target appears in singleton cluster list".into());
    } else {
        confidence -= 1;
        reasons.push("- singleton target not in clustered pattern hits".into());
    }

    if singleton_load.register == "rdi" {
        confidence += 3;
        reasons.push("+ gps candidate uses same register loaded from singleton".into());
    }

    if gps_offset != LEGACY_GPS_OFFSET_HYPOTHESIS {
        confidence += 2;
        reasons.push(format!(
            "+ gps candidate offset differs from legacy 0x{LEGACY_GPS_OFFSET_HYPOTHESIS:X}"
        ));
    } else {
        reasons.push("- offset matches legacy 0x3E30 hypothesis".into());
    }

    if let Some(ref st) = store {
        confidence += 4;
        reasons.push(format!(
            "+ result stored into object field [rbx+0x{:X}]",
            st.store_offset
        ));
    } else {
        confidence -= 2;
        reasons.push("- no mov [rbx+disp32], rsi store found after gps lea".into());
    }

    if (0x1000..=0x8000).contains(&gps_offset) {
        confidence += 1;
        reasons.push("+ gps offset in plausible struct range".into());
    }

    Some(GpsSemanticMatch {
        pattern: "gps_lea_rsi_v158".into(),
        hit_rva,
        singleton_load: SingletonLoad {
            register: singleton_load.register,
            singleton_rva: singleton_load.singleton_rva,
            singleton_rank,
            at_rva: singleton_load.at_rva,
        },
        gps_candidate: GpsCandidate {
            at_rva: hit_rva,
            base_register: "rdi".into(),
            offset: gps_offset,
            expression: format!("[singleton_value + 0x{gps_offset:X}]"),
        },
        store,
        confidence,
        reasons,
        recommendation: if confidence >= 10 {
            "best_offline_candidate_for_one_shot_probe".into()
        } else if confidence >= 6 {
            "promising_offline_pattern_review".into()
        } else {
            "weak_semantic_match".into()
        },
    })
}

struct RawSingletonLoad {
    at_rva: usize,
    register: String,
    singleton_rva: usize,
}

fn find_singleton_mov_rdi(bytes: &[u8], base_rva: usize, before_off: usize) -> Option<RawSingletonLoad> {
    let end = before_off.saturating_sub(3);
    for i in (0..=end).rev() {
        if i + 7 > bytes.len() {
            continue;
        }
        if bytes[i] == 0x48 && bytes[i + 1] == 0x8B && bytes[i + 2] == 0x3D {
            let disp =
                i32::from_le_bytes([bytes[i + 3], bytes[i + 4], bytes[i + 5], bytes[i + 6]]);
            let at = base_rva + i;
            let singleton_rva = rip_resolve_rva(at, 7, disp);
            return Some(RawSingletonLoad {
                at_rva: at,
                register: "rdi".into(),
                singleton_rva,
            });
        }
    }
    None
}

fn find_mov_rbx_store_rsi(bytes: &[u8], base_rva: usize, after_off: usize) -> Option<GpsStore> {
    let limit = bytes.len().saturating_sub(7).min(after_off + SEMANTIC_LOOKAHEAD);
    for i in after_off..=limit {
        if bytes[i] == 0x48 && bytes[i + 1] == 0x89 && bytes[i + 2] == 0xB3 {
            let disp = u32::from_le_bytes([
                bytes[i + 3],
                bytes[i + 4],
                bytes[i + 5],
                bytes[i + 6],
            ]) as usize;
            return Some(GpsStore {
                at_rva: base_rva + i,
                store_offset: disp,
                target: format!("[rbx+0x{disp:X}]"),
                source: "rsi".into(),
            });
        }
    }
    None
}

pub fn live_probe_from_semantic(m: &GpsSemanticMatch) -> LiveProbeCandidate {
    LiveProbeCandidate {
        name: "game_ctrl_plus_40f8".into(),
        singleton_rva: m.singleton_load.singleton_rva,
        offset: m.gps_candidate.offset,
        derived_address_expression: format!("game_ctrl + 0x{:X}", m.gps_candidate.offset),
        probe_type: "one_shot_log_only".into(),
        allowed_reads: vec![
            format!(
                "read_u64(game_ctrl + 0x{:X})",
                m.gps_candidate.offset
            ),
            "optionally log candidate pointer value only".into(),
        ],
        not_allowed: vec![
            "no table walk".into(),
            "no chain deref".into(),
            "no UID probe".into(),
            "no route_task/items".into(),
            "no frame-callback scan".into(),
        ],
        stop_condition: "log once and park".into(),
    }
}

pub fn render_gps_semantic_section(matches: &[GpsSemanticMatch]) -> String {
    let mut out = String::new();
    out.push_str("[gps_semantic]\n");
    if matches.is_empty() {
        out.push_str("(no semantic gps_lea_rsi_v158 chain detected)\n");
        return out;
    }
    for m in matches {
        out.push_str(&format!("pattern={}\n", m.pattern));
        out.push_str(&format!("hit_rva=0x{:X}\n", m.hit_rva));
        out.push_str("singleton_load:\n");
        out.push_str(&format!("  at_rva=0x{:X}\n", m.singleton_load.at_rva));
        out.push_str(&format!("  register={}\n", m.singleton_load.register));
        out.push_str(&format!(
            "  singleton_rva=0x{:X}\n",
            m.singleton_load.singleton_rva
        ));
        if let Some(rank) = m.singleton_load.singleton_rank {
            out.push_str(&format!("  singleton_rank={rank}\n"));
        }
        out.push_str("gps_candidate:\n");
        out.push_str(&format!("  at_rva=0x{:X}\n", m.gps_candidate.at_rva));
        out.push_str(&format!(
            "  base_register={}\n",
            m.gps_candidate.base_register
        ));
        out.push_str(&format!("  offset=0x{:X}\n", m.gps_candidate.offset));
        out.push_str(&format!(
            "  expression={}\n",
            m.gps_candidate.expression
        ));
        if let Some(ref st) = m.store {
            out.push_str("store:\n");
            out.push_str(&format!("  at_rva=0x{:X}\n", st.at_rva));
            out.push_str(&format!("  target={}\n", st.target));
            out.push_str(&format!("  source={}\n", st.source));
        } else {
            out.push_str("store: (not found)\n");
        }
        out.push_str(&format!("confidence={}\n", m.confidence));
        out.push_str("reasons:\n");
        for r in &m.reasons {
            out.push_str(&format!("  {r}\n"));
        }
        out.push_str(&format!("recommendation={}\n", m.recommendation));
        out.push('\n');
    }
    out
}

pub fn render_live_probe_candidate(probe: &LiveProbeCandidate) -> String {
    let mut out = String::new();
    out.push_str("[live_probe_candidate]\n");
    out.push_str(&format!("name={}\n", probe.name));
    out.push_str(&format!("singleton_rva=0x{:X}\n", probe.singleton_rva));
    out.push_str(&format!("offset=0x{:X}\n", probe.offset));
    out.push_str(&format!(
        "derived_address_expression={}\n",
        probe.derived_address_expression
    ));
    out.push_str(&format!("probe_type={}\n", probe.probe_type));
    out.push_str("allowed_reads_if_later_approved:\n");
    for line in &probe.allowed_reads {
        out.push_str(&format!("  - {line}\n"));
    }
    out.push_str("not_allowed:\n");
    for line in &probe.not_allowed {
        out.push_str(&format!("  - {line}\n"));
    }
    out.push_str(&format!("stop_condition:\n  - {}\n", probe.stop_condition));
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ets2_bin_analyze::patterns::OFFLINE_PATTERNS;
    use crate::ets2_bin_analyze::pe::{build_minimal_pe64, PeFile};
    use crate::ets2_bin_analyze::scan::{scan_one_pattern, PatternHit};
    use std::path::Path;

    const TEXT_BASE: usize = 0x1000;
    const SINGLETON_RVA: usize = 0x354F398;

    fn gps_lea_pattern() -> &'static crate::ets2_bin_analyze::patterns::OfflinePattern {
        OFFLINE_PATTERNS
            .iter()
            .find(|p| p.name == "gps_lea_rsi_v158")
            .unwrap()
    }

    fn build_semantic_fixture() -> (PeFile, usize, Vec<SingletonRankHint>) {
        let hit_rva = TEXT_BASE + 0x1DE;
        let mov_rva = TEXT_BASE + 0x114;
        let store_rva = hit_rva + 0xD;
        let mut text = vec![0x90u8; 0x400];

        let mov_disp = (SINGLETON_RVA as i64 - (mov_rva as i64 + 7)) as i32;
        let mov_off = mov_rva - TEXT_BASE;
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
        let lea_off = hit_rva - TEXT_BASE;
        text[lea_off..lea_off + pat.pattern.len()].copy_from_slice(pat.pattern);
        text[lea_off + 3..lea_off + 7].copy_from_slice(&0x40F8u32.to_le_bytes());

        let store_off = store_rva - TEXT_BASE;
        text[store_off..store_off + 7].copy_from_slice(&[
            0x48, 0x89, 0xB3, 0xB0, 0x00, 0x00, 0x00,
        ]);

        let pe = PeFile::from_bytes_for_test(Path::new("t.exe"), build_minimal_pe64(&text, b"x\0")).unwrap();
        let singletons = vec![SingletonRankHint {
            target_rva: SINGLETON_RVA,
            priority_rank: 1,
        }];
        (pe, hit_rva, singletons)
    }

    #[test]
    fn semantic_finds_singleton_rva() {
        let (pe, hit_rva, singletons) = build_semantic_fixture();
        let hits = vec![PatternHit {
            pattern_name: "gps_lea_rsi_v158".into(),
            hit_rva,
            context_hex: String::new(),
            rip_target_rva: None,
            embedded_disp32: Some(0x40F8),
        }];
        let matches = scan_gps_semantic_patterns(&pe, &hits, &singletons);
        assert_eq!(matches.len(), 1);
        assert_eq!(matches[0].singleton_load.singleton_rva, SINGLETON_RVA);
    }

    #[test]
    fn semantic_finds_offset_40f8() {
        let (pe, hit_rva, singletons) = build_semantic_fixture();
        let hits = vec![PatternHit {
            pattern_name: "gps_lea_rsi_v158".into(),
            hit_rva,
            context_hex: String::new(),
            rip_target_rva: None,
            embedded_disp32: Some(0x40F8),
        }];
        let m = scan_gps_semantic_patterns(&pe, &hits, &singletons)[0].clone();
        assert_eq!(m.gps_candidate.offset, 0x40F8);
    }

    #[test]
    fn semantic_finds_store_offset_b0() {
        let (pe, hit_rva, singletons) = build_semantic_fixture();
        let hits = vec![PatternHit {
            pattern_name: "gps_lea_rsi_v158".into(),
            hit_rva,
            context_hex: String::new(),
            rip_target_rva: None,
            embedded_disp32: Some(0x40F8),
        }];
        let m = scan_gps_semantic_patterns(&pe, &hits, &singletons)[0].clone();
        assert_eq!(m.store.as_ref().unwrap().store_offset, 0xB0);
    }

    #[test]
    fn semantic_confidence_beats_generic_score() {
        let (pe, hit_rva, singletons) = build_semantic_fixture();
        let hits = vec![PatternHit {
            pattern_name: "gps_lea_rsi_v158".into(),
            hit_rva,
            context_hex: String::new(),
            rip_target_rva: None,
            embedded_disp32: Some(0x40F8),
        }];
        let m = scan_gps_semantic_patterns(&pe, &hits, &singletons)[0].clone();
        let generic_score = 11i32;
        assert!(
            m.confidence > generic_score,
            "semantic confidence {} should exceed generic {}",
            m.confidence,
            generic_score
        );
    }

    #[test]
    fn live_probe_candidate_from_semantic() {
        let (pe, hit_rva, singletons) = build_semantic_fixture();
        let hits = vec![PatternHit {
            pattern_name: "gps_lea_rsi_v158".into(),
            hit_rva,
            context_hex: String::new(),
            rip_target_rva: None,
            embedded_disp32: Some(0x40F8),
        }];
        let m = scan_gps_semantic_patterns(&pe, &hits, &singletons)[0].clone();
        let probe = live_probe_from_semantic(&m);
        assert_eq!(probe.offset, 0x40F8);
        assert_eq!(probe.singleton_rva, SINGLETON_RVA);
        assert!(probe.derived_address_expression.contains("0x40F8"));
    }

    #[test]
    fn pattern_scan_still_finds_gps_lea_in_fixture() {
        let (pe, hit_rva, _) = build_semantic_fixture();
        let hits = scan_one_pattern(&pe, gps_lea_pattern(), 4);
        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0].hit_rva, hit_rva);
        assert_eq!(hits[0].embedded_disp32, Some(0x40F8));
    }
}
