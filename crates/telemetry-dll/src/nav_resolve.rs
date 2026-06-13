//! In-process gps_manager resolution for ETS2 1.59 (Phase R1).
//!
//! Port of `scripts/resolve_gps.py` — AOB scan in `eurotrucks2.exe`, RIP-relative
//! singleton load, embedded gps at `game_ctrl + 0x3E30`.

#![allow(clippy::cast_possible_wrap)]

/// Expected game version — R3 will gate resolution on a real version check.
#[allow(dead_code)]
pub const EXPECTED_GAME_VERSION: &str = "1.59.1.3s";

const GPS_OFFSET_IN_GAME_CTRL: usize = 0x3E30;
const OFF_GPS_TRIP_DIST: usize = 0x21C;

const TRIP_DIST_MIN_M: f32 = 100.0;
const TRIP_DIST_MAX_M: f32 = 50_000_000.0;

/// `mov rcx,[rip+disp32]; lea rdx,[rbp-0x49]; mov rax,[rcx]; call [rax+0x170]`
const AOB_GAME_CTRL_LOAD: [u8; 20] = [
    0x48, 0x8B, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x48, 0x8D, 0x55, 0xB7, 0x48, 0x8B, 0x01,
    0xFF, 0x90, 0x70, 0x01, 0x00, 0x00,
];
/// `0xFF` = byte must match, `0x00` = wildcard (disp32 only).
const AOB_GAME_CTRL_MASK: [u8; 20] = [
    0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
];

const MOV_RCX_RIP_LEN: usize = 7;

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

/// RIP-relative target RVA: `insn_rva + insn_len + disp32` (signed displacement).
pub fn rip_resolve_rva(insn_rva: usize, insn_len: usize, disp32: i32) -> usize {
    (insn_rva as isize + insn_len as isize + disp32 as isize) as usize
}

/// Read signed disp32 at `match_rva + 3` inside `image`.
pub fn read_disp32_at_match(image: &[u8], match_rva: usize) -> Option<i32> {
    let start = match_rva.checked_add(3)?;
    let end = start.checked_add(4)?;
    if end > image.len() {
        return None;
    }
    let b = &image[start..end];
    Some(i32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}

fn looks_like_heap_ptr(v: u64) -> bool {
    (0x1_0000..0x0007_FFFF_FFFF_FFFF).contains(&v)
}

/// Resolve all AOB hits to unique singleton slot RVAs.
pub fn collect_slot_rvas(image: &[u8], hits: &[usize]) -> Vec<usize> {
    let mut slots: Vec<usize> = hits
        .iter()
        .filter_map(|&match_rva| {
            let disp = read_disp32_at_match(image, match_rva)?;
            Some(rip_resolve_rva(match_rva, MOV_RCX_RIP_LEN, disp))
        })
        .collect();
    slots.sort_unstable();
    slots.dedup();
    slots
}

/// Trip distance plausibility (active route in world).
pub fn trip_distance_plausible(meters: f32) -> bool {
    meters.is_finite() && meters > TRIP_DIST_MIN_M && meters < TRIP_DIST_MAX_M
}

// ---------------------------------------------------------------------------
// Windows in-process resolution
// ---------------------------------------------------------------------------

#[cfg(windows)]
mod win {
    use super::*;
    use std::ptr;

    type HMODULE = isize;
    type LPCWSTR = *const u16;

    extern "system" {
        fn GetModuleHandleW(lpModuleName: LPCWSTR) -> HMODULE;
        fn OutputDebugStringA(lpOutputString: *const u8);
    }

    fn nav_log(msg: &str) {
        let s = format!("[TruckPilot] {msg}\0");
        unsafe { OutputDebugStringA(s.as_ptr()) };
    }

    /// Main executable base (`GetModuleHandleW(NULL)`).
    unsafe fn main_module_base() -> Option<*const u8> {
        let h = GetModuleHandleW(ptr::null());
        if h == 0 {
            return None;
        }
        Some(h as *const u8)
    }

    /// `SizeOfImage` from the PE optional header.
    unsafe fn pe_size_of_image(base: *const u8) -> Option<usize> {
        if base.is_null() {
            return None;
        }
        let e_lfanew = ptr::read_unaligned(base.add(0x3C) as *const i32);
        if e_lfanew <= 0 {
            return None;
        }
        let pe = base.add(e_lfanew as usize);
        // PE signature + COFF file header (24) => optional header.
        let optional = pe.add(4 + 20);
        let magic = ptr::read_unaligned(optional as *const u16);
        if magic != 0x20B {
            return None; // expect PE32+
        }
        let size_of_image = ptr::read_unaligned(optional.add(0x38) as *const u32);
        if size_of_image == 0 {
            return None;
        }
        Some(size_of_image as usize)
    }

    unsafe fn read_u64(addr: *const u8) -> Option<u64> {
        if addr.is_null() {
            return None;
        }
        let v = ptr::read_unaligned(addr as *const u64);
        if looks_like_heap_ptr(v) { Some(v) } else { None }
    }

    unsafe fn read_f32(addr: *const u8) -> Option<f32> {
        if addr.is_null() {
            return None;
        }
        let v = ptr::read_unaligned(addr as *const f32);
        if v.is_finite() { Some(v) } else { None }
    }

    /// Resolve `gps_manager` via in-process AOB + RIP-relative singleton.
    ///
    /// Returns `None` when the pattern is missing or hits resolve to conflicting slots.
    pub unsafe fn resolve_gps_manager() -> Option<*const u8> {
        let base = main_module_base()?;
        let size = pe_size_of_image(base)?;
        let image = std::slice::from_raw_parts(base, size);

        let hits = scan_aob(image, &AOB_GAME_CTRL_LOAD, &AOB_GAME_CTRL_MASK);
        if hits.is_empty() {
            return None;
        }

        let slot_rvas = collect_slot_rvas(image, &hits);
        if slot_rvas.len() != 1 {
            return None;
        }

        let slot_addr = base.add(slot_rvas[0]);
        let game_ctrl = read_u64(slot_addr)?;
        let gps = game_ctrl
            .checked_add(GPS_OFFSET_IN_GAME_CTRL as u64)?
            as *const u8;
        Some(gps)
    }

    /// Read `gps_manager::trip_distance` (+0x21C) if plausible.
    pub unsafe fn verify_trip_distance(gps: *const u8) -> Option<f32> {
        let meters = read_f32(gps.add(OFF_GPS_TRIP_DIST))?;
        if trip_distance_plausible(meters) {
            Some(meters)
        } else {
            None
        }
    }

    /// One-shot diagnostic at plugin init — INFO-level, never treats missing route as AOB failure.
    pub fn diagnose_once() {
        unsafe {
            match resolve_gps_manager() {
                None => {
                    nav_log(
                        "nav_resolve: AOB/Singleton nicht aufgeloest (kein Treffer oder widerspruechliche Slots)",
                    );
                }
                Some(gps) => {
                    let addr = gps as usize;
                    match verify_trip_distance(gps) {
                        Some(trip) => {
                            nav_log(&format!(
                                "nav_resolve: gps_manager=0x{addr:X} trip_distance={trip:.1} m (OK)"
                            ));
                        }
                        None => {
                            nav_log(&format!(
                                "nav_resolve: gps_manager=0x{addr:X} — keine aktive Route oder im Menue (trip_distance unplausibel)"
                            ));
                        }
                    }
                }
            }
        }
    }
}

#[cfg(windows)]
pub use win::diagnose_once;

#[cfg(windows)]
#[allow(unused_imports)] // public API for R2+ / external callers
pub use win::{resolve_gps_manager, verify_trip_distance};

#[cfg(not(windows))]
pub fn diagnose_once() {}

#[cfg(not(windows))]
pub unsafe fn resolve_gps_manager() -> Option<*const u8> {
    None
}

#[cfg(not(windows))]
pub unsafe fn verify_trip_distance(_gps: *const u8) -> Option<f32> {
    None
}

// ---------------------------------------------------------------------------
// Unit tests (run on Linux CI)
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mask_allows_real_zero_bytes_in_pattern() {
        let haystack = [0x48, 0x8B, 0x0D, 0xAA, 0xBB, 0xCC, 0xDD, 0x48, 0x8D, 0x55, 0xB7, 0x48, 0x8B, 0x01, 0xFF, 0x90, 0x70, 0x01, 0x00, 0x00];
        let hits = scan_aob(&haystack, &AOB_GAME_CTRL_LOAD, &AOB_GAME_CTRL_MASK);
        assert_eq!(hits, vec![0]);
    }

    #[test]
    fn mask_rejects_wrong_fixed_byte() {
        let mut haystack = [0u8; 20];
        haystack.copy_from_slice(&AOB_GAME_CTRL_LOAD);
        haystack[17] = 0x02; // corrupt fixed byte before real 0x00
        let hits = scan_aob(&haystack, &AOB_GAME_CTRL_LOAD, &AOB_GAME_CTRL_MASK);
        assert!(hits.is_empty());
    }

    #[test]
    fn rip_resolve_matches_python_slot_rva() {
        // Diagnosed match @ mod+0x47287E, disp32 bytes c3 dc f4 02 => slot mod+0x33C0548
        let disp = i32::from_le_bytes([0xC3, 0xDC, 0xF4, 0x02]);
        let slot = rip_resolve_rva(0x47287E, MOV_RCX_RIP_LEN, disp);
        assert_eq!(slot, 0x33C0548);
    }

    #[test]
    fn rip_resolve_negative_disp() {
        let insn_rva = 0x1000usize;
        let disp: i32 = -8;
        assert_eq!(rip_resolve_rva(insn_rva, 7, disp), 0x1000 + 7 - 8);
    }

    #[test]
    fn collect_slot_rvas_dedupes_identical_slots() {
        let target_slot = 0x33C0548usize;
        let mut image = vec![0u8; 0x800000];
        for &match_rva in &[0x47287Eusize, 0x697910usize] {
            image[match_rva..match_rva + 20].copy_from_slice(&AOB_GAME_CTRL_LOAD);
            let disp = (target_slot as isize - match_rva as isize - MOV_RCX_RIP_LEN as isize) as i32;
            image[match_rva + 3..match_rva + 7].copy_from_slice(&disp.to_le_bytes());
        }
        let hits = scan_aob(&image, &AOB_GAME_CTRL_LOAD, &AOB_GAME_CTRL_MASK);
        assert_eq!(hits.len(), 2);
        let slots = collect_slot_rvas(&image, &hits);
        assert_eq!(slots, vec![target_slot]);
    }

    #[test]
    fn collect_slot_rvas_rejects_conflicting_slots() {
        let mut image = vec![0u8; 0x800000];
        image[0x1000..0x1000 + 20].copy_from_slice(&AOB_GAME_CTRL_LOAD);
        image[0x2000..0x2000 + 20].copy_from_slice(&AOB_GAME_CTRL_LOAD);
        let d1 = i32::from_le_bytes([0x01, 0x00, 0x00, 0x00]).to_le_bytes();
        let d2 = i32::from_le_bytes([0x02, 0x00, 0x00, 0x00]).to_le_bytes();
        image[0x1003..0x1007].copy_from_slice(&d1);
        image[0x2003..0x2007].copy_from_slice(&d2);
        let hits = scan_aob(&image, &AOB_GAME_CTRL_LOAD, &AOB_GAME_CTRL_MASK);
        let slots = collect_slot_rvas(&image, &hits);
        assert_eq!(slots.len(), 2);
    }

    #[test]
    fn trip_distance_plausible_bounds() {
        assert!(!trip_distance_plausible(0.0));
        assert!(!trip_distance_plausible(50.0));
        assert!(trip_distance_plausible(690_170.4));
        assert!(!trip_distance_plausible(60_000_000.0));
    }
}
