//! x86-64 instruction heuristics for offline PE analysis (no disassembler).

use super::scan::rip_resolve_rva;

/// Kind of statically resolved code reference.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CodeRefKind {
    RipRelativeLea,
    RipRelativeMov,
    CallRel32,
    JmpRel32,
}

/// One heuristic code reference inside a byte window.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CodeRef {
    pub at_rva: usize,
    pub kind: CodeRefKind,
    pub target_rva: usize,
    pub detail: String,
    /// `false` when found at a sequential instruction boundary; `true` when misaligned.
    pub maybe_unaligned: bool,
}

/// Scan a `.text` slice sequentially for RIP-relative LEA/MOV and aligned `call`/`jmp`.
pub fn scan_code_refs(bytes: &[u8], base_rva: usize) -> Vec<CodeRef> {
    let mut out = Vec::new();
    let mut i = 0usize;
    let mut aligned = true;
    while i < bytes.len() {
        if let Some((len, refs)) = decode_at(bytes, i, base_rva, aligned) {
            out.extend(refs);
            i += len;
            aligned = true;
        } else {
            i += 1;
            aligned = false;
        }
    }
    out.sort_by_key(|r| r.at_rva);
    out
}

fn decode_at(
    bytes: &[u8],
    i: usize,
    base_rva: usize,
    aligned: bool,
) -> Option<(usize, Vec<CodeRef>)> {
    if i + 2 > bytes.len() {
        return None;
    }

    if bytes[i] == 0x90 {
        return Some((1, Vec::new()));
    }

    // Longer / specific patterns first (avoid treating inner E8/E9 as call/jmp).
    if i + 9 <= bytes.len()
        && bytes[i] == 0xF2
        && bytes[i + 1] == 0x0F
        && bytes[i + 2] == 0x11
        && bytes[i + 3] == 0x83
    {
        return Some((9, Vec::new()));
    }
    if i + 8 <= bytes.len()
        && bytes[i] == 0xF3
        && bytes[i + 1] == 0x0F
        && bytes[i + 2] == 0x10
        && bytes[i + 3] == 0x0D
    {
        return Some((8, Vec::new()));
    }
    if i + 4 <= bytes.len() && bytes[i] == 0x0F && bytes[i + 1] == 0xBA && bytes[i + 2] == 0xE8 {
        return Some((4, Vec::new()));
    }
    if i + 3 <= bytes.len() && bytes[i] == 0x48 && bytes[i + 1] == 0x8B && bytes[i + 2] == 0xE9 {
        return Some((3, Vec::new()));
    }
    if i + 3 <= bytes.len() && bytes[i] == 0x48 && bytes[i + 1] == 0x8B && bytes[i + 2] == 0xCB {
        return Some((3, Vec::new()));
    }
    if i + 7 <= bytes.len() && bytes[i] == 0x48 {
        let b1 = bytes[i + 1];
        let b2 = bytes[i + 2];
        let is_rip_lea =
            b1 == 0x8D && matches!(b2, 0x05 | 0x0D | 0x15 | 0x1D | 0x25 | 0x2D | 0x35 | 0x3D);
        let is_rip_mov =
            b1 == 0x8B && matches!(b2, 0x05 | 0x0D | 0x15 | 0x1D | 0x25 | 0x2D | 0x35 | 0x3D);
        if is_rip_lea || is_rip_mov {
            let disp =
                i32::from_le_bytes([bytes[i + 3], bytes[i + 4], bytes[i + 5], bytes[i + 6]]);
            let at = base_rva + i;
            let target = rip_resolve_rva(at, 7, disp);
            return Some((
                7,
                vec![CodeRef {
                    at_rva: at,
                    kind: if is_rip_lea {
                        CodeRefKind::RipRelativeLea
                    } else {
                        CodeRefKind::RipRelativeMov
                    },
                    target_rva: target,
                    detail: format!(
                        "{:02X} {:02X} {:02X} disp32={disp} -> 0x{target:X}",
                        bytes[i], bytes[i + 1], bytes[i + 2]
                    ),
                    maybe_unaligned: false,
                }],
            ));
        }
        if b1 == 0x8D && b2 == 0xB7 {
            return Some((7, Vec::new()));
        }
        if b1 == 0x89 && b2 == 0xB3 {
            return Some((7, Vec::new()));
        }
    }
    if i + 2 <= bytes.len() && bytes[i] == 0x0F {
        if let Some(len) = sse_0f_len(bytes, i) {
            return Some((len, Vec::new()));
        }
    }
    if i + 5 <= bytes.len() && (bytes[i] == 0xE8 || bytes[i] == 0xE9) {
        let op = bytes[i];
        let rel = i32::from_le_bytes([bytes[i + 1], bytes[i + 2], bytes[i + 3], bytes[i + 4]]);
        let at = base_rva + i;
        let target = rip_resolve_rva(at, 5, rel);
        return Some((
            5,
            vec![CodeRef {
                at_rva: at,
                kind: if op == 0xE8 {
                    CodeRefKind::CallRel32
                } else {
                    CodeRefKind::JmpRel32
                },
                target_rva: target,
                detail: format!("{op:02X} rel32={rel} -> 0x{target:X}"),
                maybe_unaligned: !aligned,
            }],
        ));
    }
    None
}

fn sse_0f_len(bytes: &[u8], i: usize) -> Option<usize> {
    let op2 = bytes[i + 1];
    if !matches!(op2, 0x28 | 0x14 | 0x57) {
        return None;
    }
    if i + 3 > bytes.len() {
        return Some(2);
    }
    let modrm = bytes[i + 2];
    let mod_field = (modrm >> 6) & 3;
    let rm = modrm & 7;
    match mod_field {
        3 => Some(3),
        0 if rm == 5 => Some(7),
        0 => Some(3),
        1 | 2 => Some(4),
        _ => Some(3),
    }
}

/// Format bytes as `rva=0x.... hex...` lines (16 bytes per line).
pub fn format_hex_window(bytes: &[u8], start_rva: usize, max_lines: usize) -> String {
    let mut out = String::new();
    for (line_idx, chunk) in bytes.chunks(16).enumerate().take(max_lines) {
        if line_idx > 0 && line_idx >= max_lines {
            break;
        }
        let rva = start_rva + line_idx * 16;
        let hex: Vec<String> = chunk.iter().map(|b| format!("{b:02X}")).collect();
        out.push_str(&format!("  0x{rva:X}: {}\n", hex.join(" ")));
    }
    if bytes.len() > max_lines * 16 {
        out.push_str("  ... truncated\n");
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn finds_call_rel32_in_window() {
        let call_at = 0x20usize;
        let rel: i32 = 0x10;
        let mut bytes = vec![0x90u8; 64];
        bytes[call_at] = 0xE8;
        bytes[call_at + 1..call_at + 5].copy_from_slice(&rel.to_le_bytes());
        let refs = scan_code_refs(&bytes, 0x1000);
        let call = refs
            .iter()
            .find(|r| r.kind == CodeRefKind::CallRel32 && r.at_rva == 0x1020)
            .unwrap();
        assert!(!call.maybe_unaligned);
    }

    #[test]
    fn finds_rip_lea_in_window() {
        let mut bytes = vec![0x90u8; 32];
        bytes[8..15].copy_from_slice(&[0x48, 0x8D, 0x0D, 0x05, 0x00, 0x00, 0x00]);
        let refs = scan_code_refs(&bytes, 0x1000);
        assert!(refs.iter().any(|r| r.kind == CodeRefKind::RipRelativeLea));
    }

    #[test]
    fn mov_rbp_rcx_not_reported_as_jmp() {
        let mut bytes = vec![0x90u8; 16];
        bytes[4..7].copy_from_slice(&[0x48, 0x8B, 0xE9]);
        let refs = scan_code_refs(&bytes, 0x1000);
        assert!(
            !refs.iter().any(|r| r.kind == CodeRefKind::JmpRel32),
            "48 8B E9 must not produce JmpRel32"
        );
    }

    #[test]
    fn bt_eax_imm8_not_reported_as_call() {
        let mut bytes = vec![0x90u8; 16];
        bytes[2..6].copy_from_slice(&[0x0F, 0xBA, 0xE8, 0x1F]);
        let refs = scan_code_refs(&bytes, 0x1000);
        assert!(
            !refs.iter().any(|r| r.kind == CodeRefKind::CallRel32),
            "0F BA E8 imm8 must not produce CallRel32"
        );
    }

    #[test]
    fn misaligned_e9_marked_maybe_unaligned() {
        let mut bytes = vec![0u8; 8];
        bytes[1] = 0xE9;
        bytes[2..6].copy_from_slice(&0x10i32.to_le_bytes());
        let refs = scan_code_refs(&bytes, 0x1000);
        let jmp = refs
            .iter()
            .find(|r| r.kind == CodeRefKind::JmpRel32)
            .expect("misaligned E9 may still appear");
        assert!(jmp.maybe_unaligned);
    }
}
