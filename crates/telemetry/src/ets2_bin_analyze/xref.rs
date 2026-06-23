//! Light heuristic cross-reference search (no disassembler).

use super::pe::PeFile;
use super::scan::rip_resolve_rva;
use super::strings::StringHit;

/// Heuristic code reference to a string RVA.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HeuristicXref {
    pub string_rva: usize,
    pub text_rva: usize,
    pub kind: XrefKind,
    pub detail: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum XrefKind {
    RipRelativeLea,
    Imm32Rva,
}

/// Find potential `.text` references to string RVAs.
pub fn find_heuristic_xrefs(pe: &PeFile, strings: &[StringHit], max_xrefs: usize) -> Vec<HeuristicXref> {
    let Some(text_sec) = pe.section(".text") else {
        return Vec::new();
    };
    let Some(text) = pe.section_data(".text") else {
        return Vec::new();
    };
    let text_base = text_sec.rva;
    let mut out = Vec::new();
    'outer: for s in strings {
        for (off, kind, detail) in scan_text_for_rva(text, text_base, s.rva) {
            if out.len() >= max_xrefs {
                break 'outer;
            }
            out.push(HeuristicXref {
                string_rva: s.rva,
                text_rva: off,
                kind,
                detail,
            });
        }
    }
    out
}

fn scan_text_for_rva(text: &[u8], text_base: usize, target_rva: usize) -> Vec<(usize, XrefKind, String)> {
    let mut out = Vec::new();
    let target_u32 = target_rva as u32;
    for i in 0..text.len().saturating_sub(4) {
        let imm = u32::from_le_bytes([text[i], text[i + 1], text[i + 2], text[i + 3]]);
        if imm == target_u32 {
            out.push((
                text_base + i,
                XrefKind::Imm32Rva,
                format!("imm32=0x{target_rva:X}"),
            ));
        }
    }
    for i in 0..text.len().saturating_sub(7) {
        let b0 = text[i];
        let b1 = text[i + 1];
        let b2 = text[i + 2];
        if b0 != 0x48 {
            continue;
        }
        let is_rip_lea = matches!(b1, 0x8D) && matches!(b2, 0x05 | 0x0D | 0x15 | 0x1D | 0x25 | 0x2D | 0x35 | 0x3D);
        let is_rip_mov = matches!(b1, 0x8B) && matches!(b2, 0x05 | 0x0D | 0x15 | 0x1D | 0x25 | 0x2D | 0x35 | 0x3D);
        if !is_rip_lea && !is_rip_mov {
            continue;
        }
        let disp = i32::from_le_bytes([text[i + 3], text[i + 4], text[i + 5], text[i + 6]]);
        let insn_rva = text_base + i;
        let resolved = rip_resolve_rva(insn_rva, 7, disp);
        if resolved == target_rva {
            out.push((
                insn_rva,
                XrefKind::RipRelativeLea,
                format!("rip_rel disp32={disp} -> 0x{resolved:X}"),
            ));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ets2_bin_analyze::pe::{build_minimal_pe64, PeFile};
    use crate::ets2_bin_analyze::strings::{StringHit, StringEncoding};
    use std::path::Path;

    #[test]
    fn finds_rip_relative_xref_heuristic() {
        let string_rva = 0x2000usize;
        let insn_rva = 0x1010usize;
        let disp = (string_rva as i64 - (insn_rva as i64 + 7)) as i32;
        let mut text = vec![0x90u8; 32];
        let off = insn_rva - 0x1000;
        text[off..off + 7].copy_from_slice(&[
            0x48,
            0x8D,
            0x0D,
            (disp as u32).to_le_bytes()[0],
            (disp as u32).to_le_bytes()[1],
            (disp as u32).to_le_bytes()[2],
            (disp as u32).to_le_bytes()[3],
        ]);
        let rdata = b"route_test\0";
        let pe_bytes = build_minimal_pe64(&text, rdata);
        let pe = PeFile::from_bytes_for_test(Path::new("t.exe"), pe_bytes).unwrap();
        let s = StringHit {
            keyword: "route".into(),
            rva: string_rva,
            encoding: StringEncoding::Ascii,
            text: "route_test".into(),
        };
        let xrefs = find_heuristic_xrefs(&pe, &[s], 8);
        assert!(xrefs.iter().any(|x| x.text_rva == insn_rva));
    }
}
