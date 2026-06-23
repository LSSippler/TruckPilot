//! Offline-safe copies of ETS2 route resolver AOB patterns (from `telemetry-dll` `nav_resolve.rs`).
//!
//! Duplicated here so file analysis never links or executes DLL resolver code paths.

/// `mov rcx,[rip+disp32]; …` instruction length for RIP slot resolution.
pub const MOV_RCX_RIP_LEN: usize = 7;
/// Byte offset of disp32 inside `mov rcx,[rip+disp32]`.
pub const RIP_DISP_OFFSET: usize = 3;

/// Known singleton slot RVAs (relative to `eurotrucks2.exe`), newest first.
pub const STATIC_GAME_CTRL_SLOTS: &[usize] = &[0x33C0548];

const AOB_GAME_CTRL_LOAD: [u8; 20] = [
    0x48, 0x8B, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x48, 0x8D, 0x55, 0xB7, 0x48, 0x8B, 0x01,
    0xFF, 0x90, 0x70, 0x01, 0x00, 0x00,
];
const AOB_GAME_CTRL_MASK: [u8; 20] = [
    0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
];

const AOB_GAME_CTRL_CALL_WC: [u8; 20] = AOB_GAME_CTRL_LOAD;
const AOB_GAME_CTRL_CALL_WC_MASK: [u8; 20] = [
    0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF,
];

const AOB_GAME_CTRL_LEA_WC: [u8; 20] = AOB_GAME_CTRL_LOAD;
const AOB_GAME_CTRL_LEA_WC_MASK: [u8; 20] = [
    0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
];

const AOB_GAME_CTRL_SHORT: [u8; 14] = [
    0x48, 0x8B, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x48, 0x8D, 0x55, 0x00, 0x48, 0x8B, 0x01,
];
const AOB_GAME_CTRL_SHORT_MASK: [u8; 14] = [
    0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF,
];

const AOB_MOV_RCX_LEA55: [u8; 10] = [
    0x48, 0x8B, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x48, 0x8D, 0x55,
];
const AOB_MOV_RCX_LEA55_MASK: [u8; 10] = [
    0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF,
];

const AOB_MOVSS_TRIP_21C: [u8; 8] = [0xF3, 0x0F, 0x11, 0x86, 0x1C, 0x02, 0x00, 0x00];
const AOB_MOVSS_TRIP_21C_MASK: [u8; 8] = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF];

const AOB_GPS_LEA_RSI: [u8; 10] = [
    0x48, 0x8D, 0xB7, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x57, 0xC9,
];
const AOB_GPS_LEA_RSI_MASK: [u8; 10] = [
    0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF,
];

/// How offline analysis interprets a pattern hit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OfflinePatternKind {
    /// `mov reg,[rip+disp32]` — compute singleton slot RVA.
    RipRelative { instr_len: usize, disp_offset: usize },
    /// Fixed `.data` singleton slot (no byte scan).
    StaticSlot { slot_rva: usize },
    /// `lea rsi,[rdi+disp32]` — report disp32 only (not RIP-relative).
    GpsStructDisp { disp_offset: usize },
    /// Match location only (no RIP target).
    MatchOnly,
}

/// One offline AOB entry mirroring DLL `PATTERN_CANDIDATES`.
#[derive(Debug, Clone, Copy)]
pub struct OfflinePattern {
    pub name: &'static str,
    pub pattern: &'static [u8],
    pub mask: &'static [u8],
    pub kind: OfflinePatternKind,
    pub text_only: bool,
}

/// All patterns searched by `ets2-bin-analyze` (same order/names as DLL resolver list).
pub const OFFLINE_PATTERNS: &[OfflinePattern] = &[
    OfflinePattern {
        name: "game_ctrl_load_v159",
        pattern: &AOB_GAME_CTRL_LOAD,
        mask: &AOB_GAME_CTRL_MASK,
        kind: OfflinePatternKind::RipRelative {
            instr_len: MOV_RCX_RIP_LEN,
            disp_offset: RIP_DISP_OFFSET,
        },
        text_only: true,
    },
    OfflinePattern {
        name: "game_ctrl_load_call_wc",
        pattern: &AOB_GAME_CTRL_CALL_WC,
        mask: &AOB_GAME_CTRL_CALL_WC_MASK,
        kind: OfflinePatternKind::RipRelative {
            instr_len: MOV_RCX_RIP_LEN,
            disp_offset: RIP_DISP_OFFSET,
        },
        text_only: true,
    },
    OfflinePattern {
        name: "game_ctrl_load_lea_wc",
        pattern: &AOB_GAME_CTRL_LEA_WC,
        mask: &AOB_GAME_CTRL_LEA_WC_MASK,
        kind: OfflinePatternKind::RipRelative {
            instr_len: MOV_RCX_RIP_LEN,
            disp_offset: RIP_DISP_OFFSET,
        },
        text_only: true,
    },
    OfflinePattern {
        name: "game_ctrl_load_short",
        pattern: &AOB_GAME_CTRL_SHORT,
        mask: &AOB_GAME_CTRL_SHORT_MASK,
        kind: OfflinePatternKind::RipRelative {
            instr_len: MOV_RCX_RIP_LEN,
            disp_offset: RIP_DISP_OFFSET,
        },
        text_only: true,
    },
    OfflinePattern {
        name: "static_slot_33c0548",
        pattern: &[],
        mask: &[],
        kind: OfflinePatternKind::StaticSlot {
            slot_rva: STATIC_GAME_CTRL_SLOTS[0],
        },
        text_only: false,
    },
    OfflinePattern {
        name: "mov_rcx_rip_lea55_cluster",
        pattern: &AOB_MOV_RCX_LEA55,
        mask: &AOB_MOV_RCX_LEA55_MASK,
        kind: OfflinePatternKind::RipRelative {
            instr_len: MOV_RCX_RIP_LEN,
            disp_offset: RIP_DISP_OFFSET,
        },
        text_only: true,
    },
    OfflinePattern {
        name: "movss_trip_21c",
        pattern: &AOB_MOVSS_TRIP_21C,
        mask: &AOB_MOVSS_TRIP_21C_MASK,
        kind: OfflinePatternKind::MatchOnly,
        text_only: true,
    },
    OfflinePattern {
        name: "gps_lea_rsi_v158",
        pattern: &AOB_GPS_LEA_RSI,
        mask: &AOB_GPS_LEA_RSI_MASK,
        kind: OfflinePatternKind::GpsStructDisp { disp_offset: 3 },
        text_only: true,
    },
];

/// Format matched bytes for reports (`??` = wildcard).
pub fn pattern_context_hex(bytes: &[u8], pattern: &[u8], mask: &[u8]) -> String {
    bytes
        .iter()
        .zip(pattern.iter().zip(mask.iter()))
        .map(|(got, (want, m))| {
            if *m == 0 {
                "??".to_string()
            } else if got == want {
                format!("{got:02X}")
            } else {
                format!("{got:02X}!")
            }
        })
        .collect::<Vec<_>>()
        .join(" ")
}
