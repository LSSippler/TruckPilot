//! AOB scanning and RIP-relative target computation (file-only).

use super::patterns::{OfflinePattern, OfflinePatternKind, OFFLINE_PATTERNS, pattern_context_hex};
use super::pe::PeFile;

/// One pattern hit inside the PE file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PatternHit {
    pub pattern_name: String,
    pub hit_rva: usize,
    pub context_hex: String,
    pub rip_target_rva: Option<usize>,
    pub embedded_disp32: Option<i32>,
}

/// Scan all offline patterns in `pe`.
pub fn scan_patterns(pe: &PeFile, max_hits_per_pattern: usize) -> Vec<PatternHit> {
    let mut out = Vec::new();
    for pat in OFFLINE_PATTERNS {
        out.extend(scan_one_pattern(pe, pat, max_hits_per_pattern));
    }
    out
}

pub(crate) fn scan_one_pattern(pe: &PeFile, pat: &OfflinePattern, max_hits: usize) -> Vec<PatternHit> {
    match pat.kind {
        OfflinePatternKind::StaticSlot { slot_rva } => {
            vec![PatternHit {
                pattern_name: pat.name.to_string(),
                hit_rva: slot_rva,
                context_hex: "static_slot".into(),
                rip_target_rva: Some(slot_rva),
                embedded_disp32: None,
            }]
        }
        _ if pat.pattern.is_empty() => Vec::new(),
        _ => {
            let haystacks: Vec<(&str, usize, &[u8])> = if pat.text_only {
                pe.section(".text")
                    .and_then(|s| pe.section_data(".text").map(|d| (".text", s.rva, d)))
                    .into_iter()
                    .collect()
            } else {
                pe.sections
                    .iter()
                    .filter_map(|s| {
                        pe.section_data(&s.name)
                            .map(|d| (s.name.as_str(), s.rva, d))
                    })
                    .collect()
            };
            let mut hits = Vec::new();
            for (_sec, base_rva, data) in haystacks {
                for off in scan_aob(data, pat.pattern, pat.mask) {
                    if hits.len() >= max_hits {
                        break;
                    }
                    let hit_rva = base_rva + off;
                    let ctx = pe
                        .read_at_rva(hit_rva, pat.pattern.len())
                        .map(|b| pattern_context_hex(b, pat.pattern, pat.mask))
                        .unwrap_or_else(|| "read_failed".into());
                    let (rip_target_rva, embedded_disp32) =
                        decode_hit_meta(pe, hit_rva, pat.kind);
                    hits.push(PatternHit {
                        pattern_name: pat.name.to_string(),
                        hit_rva,
                        context_hex: ctx,
                        rip_target_rva,
                        embedded_disp32,
                    });
                }
            }
            hits
        }
    }
}

fn decode_hit_meta(
    pe: &PeFile,
    hit_rva: usize,
    kind: OfflinePatternKind,
) -> (Option<usize>, Option<i32>) {
    match kind {
        OfflinePatternKind::RipRelative {
            instr_len,
            disp_offset,
        } => {
            if let Some(disp) = pe.read_i32_at_rva(hit_rva + disp_offset) {
                (Some(rip_resolve_rva(hit_rva, instr_len, disp)), None)
            } else {
                (None, None)
            }
        }
        OfflinePatternKind::GpsStructDisp { disp_offset } => {
            (None, pe.read_i32_at_rva(hit_rva + disp_offset))
        }
        OfflinePatternKind::StaticSlot { slot_rva } => (Some(slot_rva), None),
        OfflinePatternKind::MatchOnly => (None, None),
    }
}

/// Scan `haystack` for `pattern` using parallel `mask` (`0xFF` = match, `0x00` = wildcard).
pub fn scan_aob(haystack: &[u8], pattern: &[u8], mask: &[u8]) -> Vec<usize> {
    if pattern.is_empty() || pattern.len() != mask.len() || pattern.len() > haystack.len() {
        return Vec::new();
    }
    let n = pattern.len();
    haystack
        .windows(n)
        .enumerate()
        .filter(|(_, window)| mask_match(window, pattern, mask))
        .map(|(i, _)| i)
        .collect()
}

fn mask_match(window: &[u8], pattern: &[u8], mask: &[u8]) -> bool {
    window
        .iter()
        .zip(pattern.iter().zip(mask.iter()))
        .all(|(got, (want, m))| *m == 0 || got == want)
}

/// RIP-relative target RVA: `insn_rva + instr_len + disp32`.
pub fn rip_resolve_rva(insn_rva: usize, instr_len: usize, disp32: i32) -> usize {
    (insn_rva as isize + instr_len as isize + disp32 as isize) as usize
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ets2_bin_analyze::patterns::OFFLINE_PATTERNS;
    use crate::ets2_bin_analyze::pe::{build_minimal_pe64, PeFile};
    use std::path::Path;

    #[test]
    fn scan_aob_respects_wildcards() {
        let hay = [
            0x48, 0x8B, 0x0D, 0x12, 0x34, 0x56, 0x78, 0x48, 0x8D, 0x55, 0xB7, 0x48, 0x8B, 0x01,
        ];
        let pat = OFFLINE_PATTERNS
            .iter()
            .find(|p| p.name == "game_ctrl_load_short")
            .unwrap();
        let hits = scan_aob(&hay, pat.pattern, pat.mask);
        assert_eq!(hits, vec![0]);
    }

    #[test]
    fn rip_resolve_rva_known() {
        assert_eq!(rip_resolve_rva(0x1000, 7, 0x10), 0x1017);
        assert_eq!(rip_resolve_rva(0x1000, 7, -4i32), 0x1003);
    }

    #[test]
    fn pattern_scan_finds_game_ctrl_in_synthetic_pe() {
        let mut text = vec![0x90u8; 64];
        text[16..36].copy_from_slice(&[
            0x48, 0x8B, 0x0D, 0x05, 0x00, 0x00, 0x00, 0x48, 0x8D, 0x55, 0xB7, 0x48, 0x8B, 0x01,
            0xFF, 0x90, 0x70, 0x01, 0x00, 0x00,
        ]);
        let pe_bytes = build_minimal_pe64(&text, b"route_marker\0");
        let pe = PeFile::from_bytes_for_test(Path::new("synthetic.exe"), pe_bytes).unwrap();
        let lea_pat = OFFLINE_PATTERNS
            .iter()
            .find(|p| p.name == "game_ctrl_load_lea_wc")
            .unwrap();
        let hits = scan_one_pattern(&pe, lea_pat, 8);
        assert!(!hits.is_empty());
        assert!(hits[0].rip_target_rva.is_some());
    }
}
