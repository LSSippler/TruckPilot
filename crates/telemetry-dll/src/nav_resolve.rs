//! In-process `gps_manager` resolution for ETS2 1.59+ (Phase R1).
//!
//! # Pointer chain (verified offsets for 1.59 — see `nav_route.rs` walk)
//!
//! ```text
//! eurotrucks2.exe + slot_rva  ->  u64 game_ctrl singleton
//! game_ctrl + 0x3E30          ->  gps_manager (embedded struct, not a pointer)
//! gps + 0x08                  ->  simple_route_src*  (see `route_chain.rs`)
//! ... + 0x58 / 0x2C0 / 0x1A8  ->  route_task (+ 0x18 bias)
//! route_task + 0x50           ->  physical_route_items[]
//! item + 0x30                 ->  UID (verified 1.59)
//! item + 0x0C                 ->  active tail trim
//! ```
//!
//! # RIP-relative resolution
//!
//! Pattern hits contain `mov rcx,[rip+disp32]` (or similar). The signed disp32 at
//! `match_rva + 3` resolves the **singleton slot RVA**:
//!
//! `slot_rva = match_rva + instr_len + disp32`
//!
//! `game_ctrl = *(module_base + slot_rva)` must look like a heap pointer.
//! `gps = game_ctrl + GPS_OFFSET_IN_GAME_CTRL`.

#![allow(clippy::cast_possible_wrap)]

use crate::route_status::{
    RESOLVE_GAME_CTRL_NULL, RESOLVE_MODULE_NOT_FOUND, RESOLVE_MODULE_PE_PARSE_FAILED,
    RESOLVE_PATTERN_MATCH_INVALID, RESOLVE_PATTERN_MULTIPLE_MATCHES, RESOLVE_PATTERN_NOT_FOUND,
};

/// Expected game version — informational only until a real version gate exists.
#[allow(dead_code)]
pub const EXPECTED_GAME_VERSION: &str = "1.59.1.3s";

/// Embedded `gps_manager` within the game controller object (1.59).
pub const GPS_OFFSET_IN_GAME_CTRL: usize = 0x3E30;
const OFF_GPS_TRIP_DIST: usize = 0x21C;
/// Pointer to simple-route source at `gps_manager + 0x08` (1.59).
const OFF_GPS_SIMPLE_ROUTE_SRC: usize = 0x08;

const TRIP_DIST_MIN_M: f32 = 100.0;
const TRIP_DIST_MAX_M: f32 = 50_000_000.0;

/// Target module for in-process resolution (main executable).
pub const TARGET_MODULE_NAME: &str = "eurotrucks2.exe";

/// Known singleton slot RVAs (relative to `eurotrucks2.exe`), newest first.
pub const STATIC_GAME_CTRL_SLOTS: &[usize] = &[0x33C0548];

/// `mov rcx,[rip+disp32]; lea rdx,[rbp-0x49]; mov rax,[rcx]; call [rax+0x170]` (1.59.1.3s)
pub const AOB_GAME_CTRL_LOAD: [u8; 20] = [
    0x48, 0x8B, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x48, 0x8D, 0x55, 0xB7, 0x48, 0x8B, 0x01,
    0xFF, 0x90, 0x70, 0x01, 0x00, 0x00,
];
pub const AOB_GAME_CTRL_MASK: [u8; 20] = [
    0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
];

/// Same as [`AOB_GAME_CTRL_LOAD`] but vtable call offset bytes are wildcards.
const AOB_GAME_CTRL_CALL_WC: [u8; 20] = AOB_GAME_CTRL_LOAD;
const AOB_GAME_CTRL_CALL_WC_MASK: [u8; 20] = [
    0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF,
];

/// Same prefix; `lea rdx,[rbp+disp8]` displacement is wildcard.
const AOB_GAME_CTRL_LEA_WC: [u8; 20] = AOB_GAME_CTRL_LOAD;
const AOB_GAME_CTRL_LEA_WC_MASK: [u8; 20] = [
    0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
];

/// Shorter anchor: `mov rcx,[rip+disp]; lea rdx,[rbp+disp8]; mov rax,[rcx]`.
const AOB_GAME_CTRL_SHORT: [u8; 14] = [
    0x48, 0x8B, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x48, 0x8D, 0x55, 0x00, 0x48, 0x8B, 0x01,
];
const AOB_GAME_CTRL_SHORT_MASK: [u8; 14] = [
    0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF,
];

/// Cluster: `mov rcx,[rip+disp]; lea rdx,[rbp+...]` — many hits, validate via slot.
const AOB_MOV_RCX_LEA55: [u8; 10] = [
    0x48, 0x8B, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x48, 0x8D, 0x55,
];
const AOB_MOV_RCX_LEA55_MASK: [u8; 10] = [
    0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF,
];

/// `movss [rsi+0x21C], xmm*` — trip_distance store (diagnostic / narrow scan).
const AOB_MOVSS_TRIP_21C: [u8; 8] = [0xF3, 0x0F, 0x11, 0x86, 0x1C, 0x02, 0x00, 0x00];
const AOB_MOVSS_TRIP_21C_MASK: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];

/// 1.58 `lea rsi,[rdi+disp32]; xorps xmm1,xmm1` — disp32 = gps offset in older builds.
const AOB_GPS_LEA_RSI: [u8; 10] = [
    0x48, 0x8D, 0xB7, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x57, 0xC9,
];
const AOB_GPS_LEA_RSI_MASK: [u8; 10] = [
    0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF,
];

const MOV_RCX_RIP_LEN: usize = 7;
const RIP_DISP_OFFSET: usize = 3;
const MAX_CLUSTER_SLOT_TRIES: usize = 64;

/// How a pattern candidate resolves the singleton slot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CandidateKind {
    /// `mov reg,[rip+disp32]` — disp32 at [`RIP_DISP_OFFSET`], 7-byte instruction.
    RipRelative { instr_len: usize },
    /// Read `game_ctrl` directly at `module_base + slot_rva` (`.data` singleton).
    StaticSlot { slot_rva: usize },
    /// Diagnostic-only — never selects gps (logs match count).
    DiagnosticOnly,
}

/// One AOB / resolution strategy tried in order.
#[derive(Debug, Clone, Copy)]
pub struct PatternCandidate {
    pub name: &'static str,
    pub pattern: &'static [u8],
    pub mask: &'static [u8],
    pub kind: CandidateKind,
    /// Limit scan to `.text` when true (code patterns); false = full image.
    pub text_only: bool,
}

/// Ordered candidate list — first validated match wins.
pub const PATTERN_CANDIDATES: &[PatternCandidate] = &[
    PatternCandidate {
        name: "game_ctrl_load_v159",
        pattern: &AOB_GAME_CTRL_LOAD,
        mask: &AOB_GAME_CTRL_MASK,
        kind: CandidateKind::RipRelative {
            instr_len: MOV_RCX_RIP_LEN,
        },
        text_only: true,
    },
    PatternCandidate {
        name: "game_ctrl_load_call_wc",
        pattern: &AOB_GAME_CTRL_CALL_WC,
        mask: &AOB_GAME_CTRL_CALL_WC_MASK,
        kind: CandidateKind::RipRelative {
            instr_len: MOV_RCX_RIP_LEN,
        },
        text_only: true,
    },
    PatternCandidate {
        name: "game_ctrl_load_lea_wc",
        pattern: &AOB_GAME_CTRL_LEA_WC,
        mask: &AOB_GAME_CTRL_LEA_WC_MASK,
        kind: CandidateKind::RipRelative {
            instr_len: MOV_RCX_RIP_LEN,
        },
        text_only: true,
    },
    PatternCandidate {
        name: "game_ctrl_load_short",
        pattern: &AOB_GAME_CTRL_SHORT,
        mask: &AOB_GAME_CTRL_SHORT_MASK,
        kind: CandidateKind::RipRelative {
            instr_len: MOV_RCX_RIP_LEN,
        },
        text_only: true,
    },
    PatternCandidate {
        name: "static_slot_33c0548",
        pattern: &[],
        mask: &[],
        kind: CandidateKind::StaticSlot {
            slot_rva: STATIC_GAME_CTRL_SLOTS[0],
        },
        text_only: false,
    },
    PatternCandidate {
        name: "mov_rcx_rip_lea55_cluster",
        pattern: &AOB_MOV_RCX_LEA55,
        mask: &AOB_MOV_RCX_LEA55_MASK,
        kind: CandidateKind::RipRelative {
            instr_len: MOV_RCX_RIP_LEN,
        },
        text_only: true,
    },
    PatternCandidate {
        name: "movss_trip_21c",
        pattern: &AOB_MOVSS_TRIP_21C,
        mask: &AOB_MOVSS_TRIP_21C_MASK,
        kind: CandidateKind::DiagnosticOnly,
        text_only: true,
    },
    PatternCandidate {
        name: "gps_lea_rsi_v158",
        pattern: &AOB_GPS_LEA_RSI,
        mask: &AOB_GPS_LEA_RSI_MASK,
        kind: CandidateKind::DiagnosticOnly,
        text_only: true,
    },
];

/// Outcome of a gps_manager resolution attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpsResolveError {
    ModuleNotFound,
    PeParseFailed,
    PatternNotFound { image_size: usize },
    PatternMultipleMatches { hit_count: usize, slot_count: usize },
    PatternMatchInvalid { hit_count: usize, slot_count: usize },
    GameCtrlNull,
}

impl GpsResolveError {
    pub fn status_code(self) -> u32 {
        match self {
            Self::ModuleNotFound => RESOLVE_MODULE_NOT_FOUND,
            Self::PeParseFailed => RESOLVE_MODULE_PE_PARSE_FAILED,
            Self::PatternNotFound { .. } => RESOLVE_PATTERN_NOT_FOUND,
            Self::PatternMultipleMatches { .. } => RESOLVE_PATTERN_MULTIPLE_MATCHES,
            Self::PatternMatchInvalid { .. } => RESOLVE_PATTERN_MATCH_INVALID,
            Self::GameCtrlNull => RESOLVE_GAME_CTRL_NULL,
        }
    }

    pub fn reason_str(self) -> &'static str {
        match self {
            Self::ModuleNotFound => "module_not_found",
            Self::PeParseFailed => "pe_parse_failed",
            Self::PatternNotFound { .. } => "no_pattern_match",
            Self::PatternMultipleMatches { .. } => "multiple_slot_matches",
            Self::PatternMatchInvalid { .. } => "invalid_slot_or_game_ctrl",
            Self::GameCtrlNull => "game_ctrl_null",
        }
    }
}

/// Per-candidate scan stats for sidecar diagnostics.
#[derive(Debug, Clone, Default)]
pub struct CandidateScanStat {
    pub name: &'static str,
    pub pattern_hex: String,
    pub matches: usize,
    #[allow(dead_code)]
    pub selected: bool,
    pub validation: Option<&'static str>,
}

/// Successful resolution metadata.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpsResolveSuccess {
    pub candidate: &'static str,
    pub slot_rva: usize,
    pub game_ctrl: u64,
    pub gps: usize,
}

/// How strictly module-scan candidates are validated before acceptance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResolveValidateKind {
    /// Require embedded `gps_manager` fields to look plausible.
    GpsEmbedded,
    /// Accept any plausible heap `game_ctrl` pointer (table diagnostics only).
    GameCtrlHeap,
}

/// Sidecar line for a successful module scan (`gps_slot_addr` vs read `gps_slot_value`).
pub fn format_module_scan_success_line(
    ok: &GpsResolveSuccess,
    gps_slot_value: Result<u64, crate::safe_mem::ReadError>,
) -> String {
    let gps_slot_addr = ok.gps;
    let value_str = match gps_slot_value {
        Ok(v) => format!("0x{v:X}"),
        Err(e) => format!("read_failed({})", e.as_str()),
    };
    format!(
        "module scan success candidate={} slot_rva=0x{:X} game_ctrl=0x{:X} gps_slot_addr=0x{gps_slot_addr:X} gps_slot_value={value_str}",
        ok.candidate, ok.slot_rva, ok.game_ctrl
    )
}

/// PE section summary for diagnostics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PeSectionInfo {
    pub name: [u8; 8],
    pub virtual_address: usize,
    pub virtual_size: usize,
}

impl PeSectionInfo {
    pub fn name_str(&self) -> &str {
        std::str::from_utf8(&self.name)
            .unwrap_or("")
            .trim_end_matches('\0')
    }

    pub fn slice<'a>(&self, image: &'a [u8]) -> Option<&'a [u8]> {
        let end = self.virtual_address.checked_add(self.virtual_size)?;
        if end > image.len() {
            return None;
        }
        Some(&image[self.virtual_address..end])
    }
}

/// Format pattern bytes for logs (`??` = wildcard).
pub fn pattern_hex(pattern: &[u8], mask: &[u8]) -> String {
    pattern
        .iter()
        .zip(mask.iter())
        .map(|(b, m)| {
            if *m == 0 {
                "??".to_string()
            } else {
                format!("{b:02X}")
            }
        })
        .collect::<Vec<_>>()
        .join(" ")
}

/// Human-readable legacy pattern (first candidate).
#[allow(dead_code)]
pub fn aob_pattern_hex() -> String {
    pattern_hex(&AOB_GAME_CTRL_LOAD, &AOB_GAME_CTRL_MASK)
}

// ---------------------------------------------------------------------------
// Pure helpers (unit-testable on any target)
// ---------------------------------------------------------------------------

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

/// Read signed disp32 at `match_rva + disp_offset` inside `image`.
pub fn read_disp32_at(image: &[u8], match_rva: usize, disp_offset: usize) -> Option<i32> {
    let start = match_rva.checked_add(disp_offset)?;
    let end = start.checked_add(4)?;
    if end > image.len() {
        return None;
    }
    let b = &image[start..end];
    Some(i32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}

pub fn looks_like_heap_ptr(v: u64) -> bool {
    (0x1_0000..0x0007_FFFF_FFFF_FFFF).contains(&v)
}

/// Parse PE section headers from an in-memory mapped image.
pub fn pe_sections(image: &[u8]) -> Option<Vec<PeSectionInfo>> {
    if image.len() < 0x40 {
        return None;
    }
    let e_lfanew = i32::from_le_bytes(image[0x3C..0x40].try_into().ok()?) as usize;
    if e_lfanew + 0x18 > image.len() {
        return None;
    }
    let pe = &image[e_lfanew..];
    if pe.len() < 24 || &pe[0..4] != b"PE\0\0" {
        return None;
    }
    let num_sections = u16::from_le_bytes(pe[6..8].try_into().ok()?) as usize;
    let opt_size = u16::from_le_bytes(pe[20..22].try_into().ok()?) as usize;
    let sect_off = 24 + opt_size;
    if pe.len() < sect_off + num_sections * 40 {
        return None;
    }
    let mut out = Vec::with_capacity(num_sections);
    for i in 0..num_sections {
        let s = &pe[sect_off + i * 40..sect_off + (i + 1) * 40];
        let mut name = [0u8; 8];
        name.copy_from_slice(&s[0..8]);
        let virtual_size = u32::from_le_bytes(s[8..12].try_into().ok()?) as usize;
        let virtual_address = u32::from_le_bytes(s[12..16].try_into().ok()?) as usize;
        out.push(PeSectionInfo {
            name,
            virtual_address,
            virtual_size,
        });
    }
    Some(out)
}

fn section_slice<'a>(image: &'a [u8], name: &str) -> Option<&'a [u8]> {
    pe_sections(image)?
        .into_iter()
        .find(|s| s.name_str() == name)
        .and_then(|s| s.slice(image))
}

/// Resolve all AOB hits to unique singleton slot RVAs.
#[allow(dead_code)]
pub fn collect_slot_rvas(image: &[u8], hits: &[usize], instr_len: usize) -> Vec<usize> {
    let mut slots: Vec<usize> = hits
        .iter()
        .filter_map(|&match_rva| {
            let disp = read_disp32_at(image, match_rva, RIP_DISP_OFFSET)?;
            Some(rip_resolve_rva(match_rva, instr_len, disp))
        })
        .collect();
    slots.sort_unstable();
    slots.dedup();
    slots
}

/// Rank slot RVAs by hit frequency (descending).
pub fn rank_slot_rvas(image: &[u8], hits: &[usize], instr_len: usize) -> Vec<(usize, usize)> {
    use std::collections::HashMap;
    let mut counts: HashMap<usize, usize> = HashMap::new();
    for &match_rva in hits {
        if let Some(disp) = read_disp32_at(image, match_rva, RIP_DISP_OFFSET) {
            let slot = rip_resolve_rva(match_rva, instr_len, disp);
            *counts.entry(slot).or_insert(0) += 1;
        }
    }
    let mut ranked: Vec<_> = counts.into_iter().collect();
    ranked.sort_by(|a, b| b.1.cmp(&a.1).then_with(|| a.0.cmp(&b.0)));
    ranked
}

fn read_u64_le(image: &[u8], offset: usize) -> Option<u64> {
    let end = offset.checked_add(8)?;
    if end > image.len() {
        return None;
    }
    Some(u64::from_le_bytes(image[offset..end].try_into().ok()?))
}

fn read_f32_le(image: &[u8], offset: usize) -> Option<f32> {
    let end = offset.checked_add(4)?;
    if end > image.len() {
        return None;
    }
    let v = f32::from_le_bytes(image[offset..end].try_into().ok()?);
    if v.is_finite() { Some(v) } else { None }
}

/// Read u64 from an absolute VA mapped inside the PE image (RVA = addr - base).
pub fn read_u64_at_image(image: &[u8], base: usize, addr: usize) -> Option<u64> {
    let off = addr.checked_sub(base)?;
    read_u64_le(image, off)
}

pub fn read_f32_at_image(image: &[u8], base: usize, addr: usize) -> Option<f32> {
    let off = addr.checked_sub(base)?;
    read_f32_le(image, off)
}

/// Validate embedded `gps_manager` using only bytes present in the mapped image.
pub fn validate_gps_ptr_image(image: &[u8], base: usize, game_ctrl: u64) -> bool {
    if !looks_like_heap_ptr(game_ctrl) {
        return false;
    }
    let gps = match (game_ctrl as usize).checked_add(GPS_OFFSET_IN_GAME_CTRL) {
        Some(g) => g,
        None => return false,
    };
    if read_f32_at_image(image, base, gps + OFF_GPS_TRIP_DIST).is_none() {
        return false;
    }
    match read_u64_at_image(image, base, gps + OFF_GPS_SIMPLE_ROUTE_SRC) {
        None => false,
        Some(0) => true,
        Some(p) => looks_like_heap_ptr(p),
    }
}

#[cfg(windows)]
pub(crate) unsafe fn validate_gps_ptr_live(game_ctrl: u64) -> bool {
    if !looks_like_heap_ptr(game_ctrl) {
        return false;
    }
    let gps = match (game_ctrl as usize).checked_add(GPS_OFFSET_IN_GAME_CTRL) {
        Some(g) => g,
        None => return false,
    };
    let trip = match crate::safe_mem::safe_read_f32(gps + OFF_GPS_TRIP_DIST) {
        Ok(v) => v,
        Err(_) => return false,
    };
    if !trip.is_finite() {
        return false;
    }
    let src = crate::safe_mem::safe_read_u64(gps + OFF_GPS_SIMPLE_ROUTE_SRC).unwrap_or(0);
    src == 0 || looks_like_heap_ptr(src)
}

/// Validate only that `game_ctrl` looks like a readable heap object header.
pub fn validate_game_ctrl_heap_image(image: &[u8], base: usize, game_ctrl: u64) -> bool {
    if !looks_like_heap_ptr(game_ctrl) {
        return false;
    }
    read_u64_at_image(image, base, game_ctrl as usize).is_some()
}

#[cfg(windows)]
pub(crate) fn validate_game_ctrl_heap_live(game_ctrl: u64) -> bool {
    if !looks_like_heap_ptr(game_ctrl) {
        return false;
    }
    crate::safe_mem::region_allows_read(game_ctrl as usize, 8).is_ok()
}

#[cfg(not(windows))]
pub(crate) fn validate_game_ctrl_heap_live(game_ctrl: u64) -> bool {
    looks_like_heap_ptr(game_ctrl)
}

fn validate_candidate(
    image: &[u8],
    base: usize,
    game_ctrl: u64,
    live_validate: bool,
    kind: ResolveValidateKind,
) -> bool {
    match kind {
        ResolveValidateKind::GpsEmbedded => {
            if live_validate {
                #[cfg(windows)]
                {
                    unsafe { validate_gps_ptr_live(game_ctrl) }
                }
                #[cfg(not(windows))]
                {
                    validate_gps_ptr_image(image, base, game_ctrl)
                }
            } else {
                validate_gps_ptr_image(image, base, game_ctrl)
            }
        }
        ResolveValidateKind::GameCtrlHeap => {
            if live_validate {
                validate_game_ctrl_heap_live(game_ctrl)
            } else {
                validate_game_ctrl_heap_image(image, base, game_ctrl)
            }
        }
    }
}

fn try_slot_with_validate(
    image: &[u8],
    base: usize,
    slot_rva: usize,
    name: &'static str,
    live_validate: bool,
    validate_kind: ResolveValidateKind,
) -> Result<GpsResolveSuccess, GpsResolveError> {
    if slot_rva + 8 > image.len() {
        return Err(GpsResolveError::PatternMatchInvalid {
            hit_count: 1,
            slot_count: 1,
        });
    }
    let game_ctrl = read_u64_le(image, slot_rva).unwrap_or(0);
    let ok = validate_candidate(image, base, game_ctrl, live_validate, validate_kind);
    if !ok {
        return Err(GpsResolveError::GameCtrlNull);
    }
    Ok(GpsResolveSuccess {
        candidate: name,
        slot_rva,
        game_ctrl,
        gps: game_ctrl as usize + GPS_OFFSET_IN_GAME_CTRL,
    })
}
/// Resolve gps from a validated singleton slot RVA within the mapped image.
/// Offline/test resolver (image-only validation).
#[allow(dead_code)] // used from `#[cfg(test)]` and offline tooling
pub fn gps_from_slot_in_image(
    image: &[u8],
    base: usize,
    slot_rva: usize,
) -> Result<GpsResolveSuccess, GpsResolveError> {
    try_slot_with_validate(
        image,
        base,
        slot_rva,
        "",
        false,
        ResolveValidateKind::GpsEmbedded,
    )
}

fn try_rip_hits(
    image: &[u8],
    base: usize,
    hits: &[usize],
    instr_len: usize,
    candidate_name: &'static str,
    cluster: bool,
    live_validate: bool,
    validate_kind: ResolveValidateKind,
) -> Result<GpsResolveSuccess, GpsResolveError> {
    if hits.is_empty() {
        return Err(GpsResolveError::PatternNotFound {
            image_size: image.len(),
        });
    }

    let ranked = rank_slot_rvas(image, hits, instr_len);
    if ranked.is_empty() {
        return Err(GpsResolveError::PatternMatchInvalid {
            hit_count: hits.len(),
            slot_count: 0,
        });
    }

    let try_limit = if cluster {
        MAX_CLUSTER_SLOT_TRIES.min(ranked.len())
    } else {
        ranked.len()
    };

    let mut valid: Vec<GpsResolveSuccess> = Vec::new();
    for (slot_rva, _) in ranked.iter().take(try_limit) {
        let game_ctrl = match read_u64_le(image, *slot_rva) {
            Some(v) if looks_like_heap_ptr(v) => v,
            _ => continue,
        };
        let ok = validate_candidate(image, base, game_ctrl, live_validate, validate_kind);
        if !ok {
            continue;
        }
        let gps = game_ctrl as usize + GPS_OFFSET_IN_GAME_CTRL;
        valid.push(GpsResolveSuccess {
            candidate: candidate_name,
            slot_rva: *slot_rva,
            game_ctrl,
            gps,
        });
    }

    if valid.is_empty() {
        return Err(GpsResolveError::GameCtrlNull);
    }

    // Prefer the most frequent slot; all valid entries should agree on game_ctrl.
    valid.sort_by_key(|v| v.slot_rva);
    valid.dedup_by_key(|v| v.game_ctrl);
    if valid.len() > 1 && !cluster {
        return Err(GpsResolveError::PatternMultipleMatches {
            hit_count: hits.len(),
            slot_count: valid.len(),
        });
    }
    Ok(valid[0])
}

/// Multi-candidate resolver over a mapped PE image — unit-testable.
/// Scan PE image for game_ctrl slot (image validation only; no live heap reads).
#[allow(dead_code)] // used from `#[cfg(test)]` and offline tooling
pub fn resolve_gps_from_image(
    image: &[u8],
    base: usize,
) -> Result<GpsResolveSuccess, (GpsResolveError, Vec<CandidateScanStat>)> {
    resolve_from_image_inner(image, base, false, ResolveValidateKind::GpsEmbedded)
}

fn resolve_from_image_inner(
    image: &[u8],
    base: usize,
    live_validate: bool,
    validate_kind: ResolveValidateKind,
) -> Result<GpsResolveSuccess, (GpsResolveError, Vec<CandidateScanStat>)> {
    let text = section_slice(image, ".text");
    let mut stats = Vec::with_capacity(PATTERN_CANDIDATES.len());
    let mut last_err = GpsResolveError::PatternNotFound {
        image_size: image.len(),
    };

    for cand in PATTERN_CANDIDATES {
        let pattern_hex = if cand.pattern.is_empty() {
            format!("static_slot_rva=0x{:X}", match cand.kind {
                CandidateKind::StaticSlot { slot_rva } => slot_rva,
                _ => 0,
            })
        } else {
            pattern_hex(cand.pattern, cand.mask)
        };

        let (matches, result) = match cand.kind {
            CandidateKind::DiagnosticOnly => {
                let (hay, rva_bias) = if cand.text_only {
                    text.map(|sl| {
                        pe_sections(image)
                            .and_then(|secs| {
                                secs.into_iter()
                                    .find(|s| s.name_str() == ".text")
                                    .map(|s| (sl, s.virtual_address))
                            })
                            .unwrap_or((sl, 0))
                    })
                    .unwrap_or((image, 0))
                } else {
                    (image, 0)
                };
                let hits: Vec<usize> = scan_aob(hay, cand.pattern, cand.mask)
                    .into_iter()
                    .map(|h| h.saturating_add(rva_bias))
                    .collect();
                (hits.len(), None)
            }
            CandidateKind::StaticSlot { slot_rva } => {
                let res = try_slot_with_validate(
                    image,
                    base,
                    slot_rva,
                    cand.name,
                    live_validate,
                    validate_kind,
                );
                (if res.is_ok() { 1 } else { 0 }, res.ok())
            }
            CandidateKind::RipRelative { instr_len } => {
                let (hay, rva_bias) = if cand.text_only {
                    match pe_sections(image)
                        .and_then(|secs| secs.into_iter().find(|s| s.name_str() == ".text"))
                        .and_then(|s| s.slice(image).map(|sl| (sl, s.virtual_address)))
                    {
                        Some(v) => v,
                        None => (image, 0),
                    }
                } else {
                    (image, 0)
                };
                let hits: Vec<usize> = scan_aob(hay, cand.pattern, cand.mask)
                    .into_iter()
                    .map(|h| h.saturating_add(rva_bias))
                    .collect();
                let cluster = cand.name.contains("cluster");
                let res = try_rip_hits(
                    image,
                    base,
                    &hits,
                    instr_len,
                    cand.name,
                    cluster,
                    live_validate,
                    validate_kind,
                );
                (
                    hits.len(),
                    res.ok().map(|mut ok| {
                        ok.candidate = cand.name;
                        ok
                    }),
                )
            }
        };

        let selected = result.is_some();
        let validation = if selected {
            Some("ok")
        } else if matches > 0 {
            Some("matches_but_invalid")
        } else {
            None
        };

        stats.push(CandidateScanStat {
            name: cand.name,
            pattern_hex,
            matches,
            selected,
            validation,
        });

        if let Some(ok) = result {
            return Ok(ok);
        }

        if matches > 0 {
            last_err = GpsResolveError::GameCtrlNull;
        }
    }

    Err((last_err, stats))
}

/// Trip distance plausibility (active route in world).
pub fn trip_distance_plausible(meters: f32) -> bool {
    meters.is_finite() && meters > TRIP_DIST_MIN_M && meters < TRIP_DIST_MAX_M
}

// ---------------------------------------------------------------------------
// Session-persistent `game_ctrl` cache (avoids repeated `.text` AOB scans)
// ---------------------------------------------------------------------------

use std::sync::Mutex;

/// Cached successful module-scan result for the current session.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GameCtrlSessionCache {
    pub candidate: &'static str,
    pub slot_rva: usize,
    pub game_ctrl: u64,
    pub gps_slot_addr: usize,
    pub gps_slot_value: Option<u64>,
}

static SESSION_CACHE: Mutex<Option<GameCtrlSessionCache>> = Mutex::new(None);
static SCAN_SUCCESS_LOGGED: Mutex<Option<u64>> = Mutex::new(None);

pub fn invalidate_session_cache() {
    if let Ok(mut c) = SESSION_CACHE.lock() {
        *c = None;
    }
    if let Ok(mut l) = SCAN_SUCCESS_LOGGED.lock() {
        *l = None;
    }
}

pub fn session_cache_snapshot() -> Option<GameCtrlSessionCache> {
    SESSION_CACHE.lock().ok().and_then(|g| *g)
}

fn store_session_cache(ok: &GpsResolveSuccess) {
    let gps_slot_addr = ok.gps;
    let gps_slot_value = crate::safe_mem::safe_read_u64(gps_slot_addr).ok();
    if let Ok(mut c) = SESSION_CACHE.lock() {
        *c = Some(GameCtrlSessionCache {
            candidate: ok.candidate,
            slot_rva: ok.slot_rva,
            game_ctrl: ok.game_ctrl,
            gps_slot_addr,
            gps_slot_value,
        });
    }
}

fn cache_still_valid(entry: GameCtrlSessionCache) -> bool {
    validate_game_ctrl_heap_live(entry.game_ctrl)
}

#[cfg(windows)]
pub fn log_scan_success_deduped(ok: &GpsResolveSuccess) {
    let mut guard = match SCAN_SUCCESS_LOGGED.lock() {
        Ok(g) => g,
        Err(_) => return,
    };
    if *guard == Some(ok.game_ctrl) {
        return;
    }
    *guard = Some(ok.game_ctrl);
    crate::diag_log::event_force(&format_module_scan_success_line(
        ok,
        crate::safe_mem::safe_read_u64(ok.gps),
    ));
}

#[cfg(not(windows))]
pub fn log_scan_success_deduped(_ok: &GpsResolveSuccess) {}

/// Resolve `game_ctrl` using session cache when valid; optional full scan when allowed.
#[cfg(windows)]
pub unsafe fn resolve_game_ctrl_cached(
    force_full_scan: bool,
    allow_full_scan: bool,
) -> Result<(GameCtrlSessionCache, bool), GpsResolveError> {
    if crate::safe_mem::route_resolver_mode().is_off() {
        crate::resolver_metrics::note_off_mode_blocked_call();
        return Err(GpsResolveError::GameCtrlNull);
    }
    if !force_full_scan {
        if let Some(entry) = session_cache_snapshot() {
            if cache_still_valid(entry) {
                log_scan_success_deduped(&GpsResolveSuccess {
                    candidate: entry.candidate,
                    slot_rva: entry.slot_rva,
                    game_ctrl: entry.game_ctrl,
                    gps: entry.gps_slot_addr,
                });
                return Ok((entry, true));
            }
            invalidate_session_cache();
        }
    } else {
        invalidate_session_cache();
    }

    if !allow_full_scan {
        return Err(GpsResolveError::GameCtrlNull);
    }

    let ok = win::resolve_game_ctrl_manager_full()?;
    crate::resolver_metrics::note_worker_pattern_scan();
    store_session_cache(&ok);
    log_scan_success_deduped(&ok);
    Ok((
        session_cache_snapshot().expect("cache stored"),
        false,
    ))
}

#[cfg(not(windows))]
pub unsafe fn resolve_game_ctrl_cached(
    _force_full_scan: bool,
    _allow_full_scan: bool,
) -> Result<(GameCtrlSessionCache, bool), GpsResolveError> {
    Err(GpsResolveError::ModuleNotFound)
}

// ---------------------------------------------------------------------------
// Windows in-process resolution
// ---------------------------------------------------------------------------

#[cfg(windows)]
mod win {
    use super::*;
    use std::ptr;

    type HMODULE = isize;

    extern "system" {
        fn GetModuleHandleW(lpModuleName: *const u16) -> HMODULE;
        fn OutputDebugStringA(lpOutputString: *const u8);
    }

    static mut SCAN_DIAG_LOGGED: bool = false;

    fn nav_log(msg: &str) {
        let s = format!("[TruckPilot] {msg}\0");
        unsafe { OutputDebugStringA(s.as_ptr()) };
    }

    fn sidecar_scan(msg: &str) {
        crate::diag_log::event_force(msg);
    }

    fn log_scan_diagnostics(
        base: usize,
        image_size: usize,
        image: &[u8],
        stats: &[CandidateScanStat],
        err: GpsResolveError,
    ) {
        unsafe {
            if SCAN_DIAG_LOGGED {
                return;
            }
            SCAN_DIAG_LOGGED = true;
        }
        sidecar_scan("module scan diagnostics:");
        sidecar_scan(&format!(
            "module name={TARGET_MODULE_NAME} base=0x{base:X} size={image_size}"
        ));
        if let Some(sections) = pe_sections(image) {
            for sec in sections {
                let name = sec.name_str();
                if name == ".text" || name == ".rdata" || name == ".data" {
                    sidecar_scan(&format!(
                        "section {name} range=0x{:X}-0x{:X}",
                        sec.virtual_address,
                        sec.virtual_address.saturating_add(sec.virtual_size)
                    ));
                }
            }
        }
        for (i, st) in stats.iter().enumerate() {
            let val_note = st
                .validation
                .map(|v| format!(" validation={v}"))
                .unwrap_or_default();
            sidecar_scan(&format!(
                "candidate[{i}] name={} pattern={} matches={}{}",
                st.name, st.pattern_hex, st.matches, val_note
            ));
        }
        sidecar_scan(&format!("module scan failed reason={}", err.reason_str()));
    }

    fn log_scan_success(ok: &GpsResolveSuccess) {
        crate::nav_resolve::log_scan_success_deduped(ok);
    }

    unsafe fn main_module_base() -> Option<*const u8> {
        let h = GetModuleHandleW(ptr::null());
        if h == 0 {
            return None;
        }
        Some(h as *const u8)
    }

    unsafe fn pe_size_of_image(base: *const u8) -> Option<usize> {
        if base.is_null() {
            return None;
        }
        let _image = std::slice::from_raw_parts(base, 0x1000.min(isize::MAX as usize));
        let e_lfanew = ptr::read_unaligned(base.add(0x3C) as *const i32);
        if e_lfanew <= 0 || e_lfanew > 0x1000 {
            return None;
        }
        let pe = base.add(e_lfanew as usize);
        let optional = pe.add(4 + 20);
        let magic = ptr::read_unaligned(optional as *const u16);
        if magic != 0x20B {
            return None;
        }
        let size_of_image = ptr::read_unaligned(optional.add(0x38) as *const u32);
        if size_of_image == 0 || size_of_image > 0x2000_0000 {
            return None;
        }
        Some(size_of_image as usize)
    }

    unsafe fn read_f32(addr: *const u8) -> Option<f32> {
        if addr.is_null() {
            return None;
        }
        let v = ptr::read_unaligned(addr as *const f32);
        if v.is_finite() { Some(v) } else { None }
    }

    /// Resolve `gps_manager` via multi-candidate AOB + static slot fallbacks.
    pub unsafe fn resolve_gps_manager() -> Result<*const u8, GpsResolveError> {
        let base_ptr = main_module_base().ok_or(GpsResolveError::ModuleNotFound)?;
        let base = base_ptr as usize;
        let size = pe_size_of_image(base_ptr).ok_or(GpsResolveError::PeParseFailed)?;
        let image = std::slice::from_raw_parts(base_ptr, size);
        match resolve_from_image_inner(image, base, true, ResolveValidateKind::GpsEmbedded) {
            Ok(ok) => {
                log_scan_success(&ok);
                Ok(ok.gps as *const u8)
            }
            Err((err, stats)) => {
                log_scan_diagnostics(base, size, image, &stats, err);
                Err(err)
            }
        }
    }

    /// Resolve `game_ctrl` singleton via module scan (heap pointer only, no gps validation).
    pub unsafe fn resolve_game_ctrl_manager() -> Result<u64, GpsResolveError> {
        resolve_game_ctrl_manager_full().map(|ok| ok.game_ctrl)
    }

    pub(super) unsafe fn resolve_game_ctrl_manager_full() -> Result<GpsResolveSuccess, GpsResolveError> {
        if crate::safe_mem::route_resolver_mode().is_off() {
            crate::resolver_metrics::note_off_mode_blocked_call();
            return Err(GpsResolveError::GameCtrlNull);
        }
        let base_ptr = main_module_base().ok_or(GpsResolveError::ModuleNotFound)?;
        let base = base_ptr as usize;
        let size = pe_size_of_image(base_ptr).ok_or(GpsResolveError::PeParseFailed)?;
        let image = std::slice::from_raw_parts(base_ptr, size);
        match resolve_from_image_inner(image, base, true, ResolveValidateKind::GameCtrlHeap) {
            Ok(ok) => Ok(ok),
            Err((err, stats)) => {
                log_scan_diagnostics(base, size, image, &stats, err);
                Err(err)
            }
        }
    }

    /// Legacy alias used by gps resolve path.
    pub unsafe fn resolve_game_ctrl_manager_scan() -> Result<GpsResolveSuccess, GpsResolveError> {
        resolve_game_ctrl_manager_full()
    }

    pub unsafe fn verify_trip_distance(gps: *const u8) -> Option<f32> {
        if gps.is_null() {
            return None;
        }
        let meters = crate::safe_mem::safe_read_f32(gps as usize + OFF_GPS_TRIP_DIST).ok()?;
        if trip_distance_plausible(meters) {
            Some(meters)
        } else {
            None
        }
    }

    pub fn diagnose_once() {
        unsafe {
            match resolve_gps_manager() {
                Err(e) => {
                    nav_log(&format!(
                        "nav_resolve: gps_manager scan failed ({})",
                        e.reason_str()
                    ));
                }
                Ok(gps) => {
                    let addr = gps as usize;
                    match verify_trip_distance(gps) {
                        Some(trip) => {
                            nav_log(&format!(
                                "nav_resolve: gps_manager=0x{addr:X} trip_distance={trip:.1} m (OK)"
                            ));
                        }
                        None => {
                            nav_log(&format!(
                                "nav_resolve: gps_manager=0x{addr:X} — keine aktive Route oder im Menue"
                            ));
                        }
                    }
                }
            }
        }
    }
}

#[cfg(windows)]
#[allow(unused_imports)]
pub use win::{diagnose_once, resolve_game_ctrl_manager, resolve_gps_manager, verify_trip_distance};

#[cfg(not(windows))]
pub fn diagnose_once() {}

#[cfg(not(windows))]
pub unsafe fn resolve_game_ctrl_manager() -> Result<u64, GpsResolveError> {
    Err(GpsResolveError::ModuleNotFound)
}

#[cfg(not(windows))]
pub unsafe fn resolve_gps_manager() -> Result<*const u8, GpsResolveError> {
    Err(GpsResolveError::ModuleNotFound)
}

#[cfg(not(windows))]
pub unsafe fn verify_trip_distance(_gps: *const u8) -> Option<f32> {
    None
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn setup_image_with_slot(slot_rva: usize, game_ctrl: u64, base: usize) -> Vec<u8> {
        let gps_off = (game_ctrl as usize)
            .saturating_add(GPS_OFFSET_IN_GAME_CTRL)
            .saturating_sub(base);
        let need = slot_rva
            .max(gps_off + OFF_GPS_TRIP_DIST + 4)
            .max(gps_off + OFF_GPS_SIMPLE_ROUTE_SRC + 8)
            + 8;
        let mut image = vec![0u8; need];
        image[slot_rva..slot_rva + 8].copy_from_slice(&game_ctrl.to_le_bytes());
        image[gps_off + OFF_GPS_TRIP_DIST..gps_off + OFF_GPS_TRIP_DIST + 4]
            .copy_from_slice(&0.0f32.to_le_bytes());
        image
    }

    const TEST_BASE: usize = 0;
    fn test_game_ctrl(_base: usize) -> u64 {
        0x0000_0000_0010_0000
    }

    #[test]
    fn mask_allows_real_zero_bytes_in_pattern() {
        let haystack = [0x48, 0x8B, 0x0D, 0xAA, 0xBB, 0xCC, 0xDD, 0x48, 0x8D, 0x55, 0xB7, 0x48, 0x8B, 0x01, 0xFF, 0x90, 0x70, 0x01, 0x00, 0x00];
        let hits = scan_aob(&haystack, &AOB_GAME_CTRL_LOAD, &AOB_GAME_CTRL_MASK);
        assert_eq!(hits, vec![0]);
    }

    #[test]
    fn rip_resolve_matches_python_slot_rva() {
        let disp = i32::from_le_bytes([0xC3, 0xDC, 0xF4, 0x02]);
        let slot = rip_resolve_rva(0x47287E, MOV_RCX_RIP_LEN, disp);
        assert_eq!(slot, 0x33C0548);
    }

    #[test]
    fn static_slot_resolves_gps() {
        let base = TEST_BASE;
        let slot_rva = 0x2000usize;
        let game_ctrl = test_game_ctrl(base);
        let image = setup_image_with_slot(slot_rva, game_ctrl, base);
        let ok = gps_from_slot_in_image(&image, base, slot_rva).unwrap();
        assert_eq!(ok.gps, game_ctrl as usize + GPS_OFFSET_IN_GAME_CTRL);
    }

    #[test]
    fn static_slot_rva_try_validate() {
        let base = TEST_BASE;
        let slot_rva = 0x8000usize;
        let game_ctrl = test_game_ctrl(base);
        let image = setup_image_with_slot(slot_rva, game_ctrl, base);
        let ok = try_slot_with_validate(
            &image,
            base,
            slot_rva,
            "static_slot_33c0548",
            false,
            ResolveValidateKind::GpsEmbedded,
        )
        .unwrap();
        assert_eq!(ok.slot_rva, slot_rva);
    }

    #[test]
    fn pattern_v159_resolves_from_text() {
        let base = TEST_BASE;
        let slot_rva = 0x5000usize;
        let match_rva = 0x1000usize;
        let game_ctrl = test_game_ctrl(base);
        let gps_off = (game_ctrl as usize + GPS_OFFSET_IN_GAME_CTRL).saturating_sub(base);
        let mut image = vec![0u8; gps_off + OFF_GPS_TRIP_DIST + 8];
        // Minimal PE headers so .text slice works
        image[0x3C..0x40].copy_from_slice(&0x80u32.to_le_bytes());
        image[0x80..0x84].copy_from_slice(b"PE\0\0");
        image[0x86..0x88].copy_from_slice(&1u16.to_le_bytes());
        image[0x94..0x96].copy_from_slice(&0xF0u16.to_le_bytes());
        image[0x80 + 24 + 0xF0..0x80 + 24 + 0xF0 + 8].copy_from_slice(b".text\0\0\0");
        image[0x80 + 24 + 0xF0 + 8..0x80 + 24 + 0xF0 + 12]
            .copy_from_slice(&0x9000u32.to_le_bytes());
        image[0x80 + 24 + 0xF0 + 12..0x80 + 24 + 0xF0 + 16]
            .copy_from_slice(&0x1000u32.to_le_bytes());
        image[match_rva..match_rva + 20].copy_from_slice(&AOB_GAME_CTRL_LOAD);
        let disp = (slot_rva as isize - match_rva as isize - MOV_RCX_RIP_LEN as isize) as i32;
        image[match_rva + 3..match_rva + 7].copy_from_slice(&disp.to_le_bytes());
        image[slot_rva..slot_rva + 8].copy_from_slice(&game_ctrl.to_le_bytes());
        let gps_off = (game_ctrl as usize + GPS_OFFSET_IN_GAME_CTRL).saturating_sub(base);
        image[gps_off + OFF_GPS_TRIP_DIST..gps_off + OFF_GPS_TRIP_DIST + 4]
            .copy_from_slice(&100.0f32.to_le_bytes());

        let ok = resolve_gps_from_image(&image, base).unwrap();
        assert_eq!(ok.candidate, "game_ctrl_load_v159");
    }

    #[test]
    fn resolve_from_image_all_fail_pattern_not_found() {
        let image = vec![0u8; 1024];
        let (err, stats) = resolve_gps_from_image(&image, 0x1000_0000).unwrap_err();
        assert!(matches!(err, GpsResolveError::PatternNotFound { .. }));
        assert_eq!(stats.len(), PATTERN_CANDIDATES.len());
    }

    fn embed_trip_distance(image: &mut [u8], base: usize, game_ctrl: u64) {
        let gps_off = (game_ctrl as usize + GPS_OFFSET_IN_GAME_CTRL).saturating_sub(base);
        if gps_off + OFF_GPS_TRIP_DIST + 4 <= image.len() {
            image[gps_off + OFF_GPS_TRIP_DIST..gps_off + OFF_GPS_TRIP_DIST + 4]
                .copy_from_slice(&0.0f32.to_le_bytes());
        }
    }

    #[test]
    fn multiple_slots_rejects_non_cluster() {
        let base = TEST_BASE;
        let mut image = vec![0u8; 0x200000];
        let match_a = 0x2000usize;
        let match_b = 0x3000usize;
        let slot_a = 0x5000usize;
        let slot_b = 0x6000usize;
        let gc_a = test_game_ctrl(base);
        let gc_b = test_game_ctrl(base) + 0x20_000;
        for (m, slot, gc) in [(match_a, slot_a, gc_a), (match_b, slot_b, gc_b)] {
            image[m..m + 20].copy_from_slice(&AOB_GAME_CTRL_LOAD);
            let disp = (slot as isize - m as isize - MOV_RCX_RIP_LEN as isize) as i32;
            image[m + 3..m + 7].copy_from_slice(&disp.to_le_bytes());
            image[slot..slot + 8].copy_from_slice(&gc.to_le_bytes());
            embed_trip_distance(&mut image, base, gc);
        }
        let hits = scan_aob(&image, &AOB_GAME_CTRL_LOAD, &AOB_GAME_CTRL_MASK);
        let err = try_rip_hits(
            &image,
            base,
            &hits,
            MOV_RCX_RIP_LEN,
            "test",
            false,
            false,
            ResolveValidateKind::GpsEmbedded,
        )
        .unwrap_err();
        assert!(matches!(err, GpsResolveError::PatternMultipleMatches { .. }));
    }

    #[test]
    fn rank_slot_rvas_by_frequency() {
        let mut image = vec![0u8; 0x10000];
        let target_slot = 0x33C0548usize;
        for &match_rva in &[0x1000usize, 0x2000usize, 0x3000usize] {
            image[match_rva..match_rva + 20].copy_from_slice(&AOB_GAME_CTRL_LOAD);
            let disp = (target_slot as isize - match_rva as isize - MOV_RCX_RIP_LEN as isize) as i32;
            image[match_rva + 3..match_rva + 7].copy_from_slice(&disp.to_le_bytes());
        }
        let hits = scan_aob(&image, &AOB_GAME_CTRL_LOAD, &AOB_GAME_CTRL_MASK);
        let ranked = rank_slot_rvas(&image, &hits, MOV_RCX_RIP_LEN);
        assert_eq!(ranked[0].0, target_slot);
        assert_eq!(ranked[0].1, 3);
    }

    #[test]
    fn gps_error_maps_to_status_code() {
        assert_eq!(
            GpsResolveError::PatternNotFound { image_size: 100 }.status_code(),
            RESOLVE_PATTERN_NOT_FOUND
        );
    }

    #[test]
    fn trip_distance_plausible_bounds() {
        assert!(!trip_distance_plausible(0.0));
        assert!(trip_distance_plausible(690_170.4));
    }

    #[test]
    fn module_scan_success_line_uses_gps_slot_addr_and_value() {
        let game_ctrl = test_game_ctrl(TEST_BASE);
        let ok = GpsResolveSuccess {
            candidate: "game_ctrl_load_v159",
            slot_rva: 0x33C0548,
            game_ctrl,
            gps: game_ctrl as usize + GPS_OFFSET_IN_GAME_CTRL,
        };
        let line = format_module_scan_success_line(&ok, Ok(0));
        assert!(line.contains("gps_slot_addr=0x"));
        assert!(line.contains("gps_slot_value=0x0"));
        assert!(!line.contains(" gps=0x"));
    }

    #[test]
    fn module_scan_success_line_reports_read_failed_value() {
        let game_ctrl = test_game_ctrl(TEST_BASE);
        let ok = GpsResolveSuccess {
            candidate: "game_ctrl_load_v159",
            slot_rva: 0x33C0548,
            game_ctrl,
            gps: game_ctrl as usize + GPS_OFFSET_IN_GAME_CTRL,
        };
        let line = format_module_scan_success_line(&ok, Err(crate::safe_mem::ReadError::NullAddress));
        assert!(line.contains("gps_slot_value=read_failed(null_address)"));
    }

    #[test]
    fn session_cache_starts_empty() {
        invalidate_session_cache();
        assert!(session_cache_snapshot().is_none());
    }

    #[test]
    fn scan_success_log_dedupes_per_game_ctrl() {
        invalidate_session_cache();
        let ok = GpsResolveSuccess {
            candidate: "game_ctrl_load_v159",
            slot_rva: 0x33C0548,
            game_ctrl: 0xDEAD_BEEF,
            gps: 0xDEAD_BEEF + GPS_OFFSET_IN_GAME_CTRL,
        };
        log_scan_success_deduped(&ok);
        log_scan_success_deduped(&ok);
        let logged = SCAN_SUCCESS_LOGGED.lock().unwrap();
        assert_eq!(logged.as_ref(), Some(&0xDEAD_BEEFu64));
    }

    #[test]
    fn gps_slot_value_zero_is_valid_cache_field() {
        let entry = GameCtrlSessionCache {
            candidate: "test",
            slot_rva: 0x1000,
            game_ctrl: 0x10_0000,
            gps_slot_addr: 0x10_0000 + GPS_OFFSET_IN_GAME_CTRL,
            gps_slot_value: Some(0),
        };
        assert_eq!(entry.gps_slot_value, Some(0));
    }
}
