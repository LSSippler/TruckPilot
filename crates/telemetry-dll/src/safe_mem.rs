//! VirtualQuery-guarded process memory reads for the telemetry DLL.
//!
//! All live ETS2 heap walks must go through these helpers — raw pointer reads
//! can AV the game process and are not caught by `catch_unwind`.

use std::sync::OnceLock;

/// Why a guarded read was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadError {
    NullAddress,
    NonCanonical,
    VirtualQueryFailed,
    NotCommitted,
    GuardPage,
    NoAccess,
    OutOfRegion,
}

impl ReadError {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::NullAddress => "null_address",
            Self::NonCanonical => "non_canonical",
            Self::VirtualQueryFailed => "virtual_query_failed",
            Self::NotCommitted => "not_committed",
            Self::GuardPage => "guard_page",
            Self::NoAccess => "no_access",
            Self::OutOfRegion => "out_of_region",
        }
    }
}

/// User-mode canonical address range (x64 Windows usermode).
pub fn addr_canonical(addr: usize) -> bool {
    (0x1_0000..0x0007_FFFF_FFFF_FFFF).contains(&addr)
}

fn check_addr(addr: usize, size: usize) -> Result<(), ReadError> {
    if addr == 0 {
        return Err(ReadError::NullAddress);
    }
    if !addr_canonical(addr) {
        return Err(ReadError::NonCanonical);
    }
    addr.checked_add(size)
        .filter(|&end| addr_canonical(end.saturating_sub(1)))
        .ok_or(ReadError::OutOfRegion)?;
    Ok(())
}

#[cfg(windows)]
mod win {
    use super::*;

    type LPVOID = *mut core::ffi::c_void;
    type SIZE_T = usize;

    #[repr(C)]
    struct MemoryBasicInformation {
        base_address: LPVOID,
        allocation_base: LPVOID,
        allocation_protect: u32,
        partition_id: u16,
        _pad: u16,
        region_size: SIZE_T,
        state: u32,
        protect: u32,
        type_: u32,
    }

    extern "system" {
        fn VirtualQuery(
            lpAddress: LPVOID,
            lpBuffer: *mut MemoryBasicInformation,
            dwLength: SIZE_T,
        ) -> SIZE_T;
    }

    const MEM_COMMIT: u32 = 0x1000;
    const PAGE_NOACCESS: u32 = 0x01;
    const PAGE_GUARD: u32 = 0x100;

    pub fn page_protect_allows_read(protect: u32) -> Result<(), ReadError> {
        let prot = protect & 0xFF;
        if prot == PAGE_NOACCESS {
            return Err(ReadError::NoAccess);
        }
        if (protect & PAGE_GUARD) != 0 {
            return Err(ReadError::GuardPage);
        }
        Ok(())
    }

    pub fn region_allows_read(addr: usize, size: usize) -> Result<(), ReadError> {
        check_addr(addr, size)?;
        unsafe {
            let mut mbi: MemoryBasicInformation = std::mem::zeroed();
            let got = VirtualQuery(
                addr as LPVOID,
                &mut mbi,
                std::mem::size_of::<MemoryBasicInformation>(),
            );
            if got == 0 {
                return Err(ReadError::VirtualQueryFailed);
            }
            if mbi.state != MEM_COMMIT {
                return Err(ReadError::NotCommitted);
            }
            page_protect_allows_read(mbi.protect)?;
            let base = mbi.base_address as usize;
            let region_end = base.saturating_add(mbi.region_size);
            let end = addr.saturating_add(size);
            if addr < base || end > region_end {
                return Err(ReadError::OutOfRegion);
            }
        }
        Ok(())
    }
}

#[cfg(not(windows))]
mod win {
    use super::*;
    pub fn page_protect_allows_read(_protect: u32) -> Result<(), ReadError> {
        Ok(())
    }
}

pub fn page_protect_allows_read(protect: u32) -> Result<(), ReadError> {
    win::page_protect_allows_read(protect)
}

#[cfg(windows)]
pub fn region_allows_read(addr: usize, size: usize) -> Result<(), ReadError> {
    win::region_allows_read(addr, size)
}

#[cfg(not(windows))]
pub fn region_allows_read(addr: usize, size: usize) -> Result<(), ReadError> {
    check_addr(addr, size)
}

pub fn safe_read_u64(addr: usize) -> Result<u64, ReadError> {
    region_allows_read(addr, 8)?;
    Ok(unsafe { std::ptr::read_unaligned(addr as *const u64) })
}

pub fn safe_read_u32(addr: usize) -> Result<u32, ReadError> {
    region_allows_read(addr, 4)?;
    Ok(unsafe { std::ptr::read_unaligned(addr as *const u32) })
}

pub fn safe_read_f32(addr: usize) -> Result<f32, ReadError> {
    region_allows_read(addr, 4)?;
    let v = unsafe { std::ptr::read_unaligned(addr as *const f32) };
    if v.is_finite() {
        Ok(v)
    } else {
        Err(ReadError::NoAccess)
    }
}

/// Max GPS pointer-table span for crash-safe diagnostics (`gps + 0x00 ..= end`).
pub const GPS_TABLE_SAFE_END: usize = 0x100;

/// Number of 8-byte slots in the crash-safe GPS pointer table (`0x00..=0x100`).
pub const GPS_TABLE_SLOT_COUNT: u32 = (GPS_TABLE_SAFE_END / 8) as u32 + 1;

/// Route resolver operating mode (file/env gated, cached once per process).
///
/// Priority (highest wins): `full` > `static` > `route_candidate_table` > `game_ctrl_table` > `gps_table` > off.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RouteResolverMode {
    /// Default — resolver disabled; no memory reads or scans.
    SafeDefault,
    /// Diagnostic: log `gps+0x00..0x100` via `safe_read_u64` only (no follow derefs).
    GpsTableOnly,
    /// Diagnostic: log `game_ctrl+0x0000..0x5000` via `safe_read_u64` only.
    GameCtrlTableOnly,
    /// Diagnostic: log fixed route candidate tables from selected `game_ctrl` slots.
    RouteCandidateTableOnly,
    /// Static pointer-chain candidates only (no SRS/direct scan).
    StaticChain,
    /// Full deep scan (SRS offset + direct route_task/items).
    FullDeep,
}

impl RouteResolverMode {
    pub const fn sidecar_label(self) -> &'static str {
        match self {
            Self::SafeDefault => "off",
            Self::GpsTableOnly => "gps_table",
            Self::GameCtrlTableOnly => "game_ctrl_table",
            Self::RouteCandidateTableOnly => "route_candidate_table",
            Self::StaticChain => "static",
            Self::FullDeep => "full",
        }
    }

    /// True when no enable file is present — resolver must not touch game memory.
    pub const fn is_off(self) -> bool {
        matches!(self, Self::SafeDefault)
    }

    /// True when any resolver work (scan, table, chain) is allowed.
    pub const fn enables_resolver_work(self) -> bool {
        !self.is_off()
    }

    pub const fn is_table_diagnostic(self) -> bool {
        matches!(
            self,
            Self::GpsTableOnly | Self::GameCtrlTableOnly | Self::RouteCandidateTableOnly
        )
    }
}

/// Scan policy — cached once per process.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RouteScanPolicy {
    pub deep_scan: bool,
    pub allow_static_chain: bool,
    pub allow_srs_offset_scan: bool,
    pub allow_direct_scan: bool,
}

impl RouteScanPolicy {
    pub const fn safe_default() -> Self {
        Self {
            deep_scan: false,
            allow_static_chain: false,
            allow_srs_offset_scan: false,
            allow_direct_scan: false,
        }
    }

    pub const fn static_chain_only() -> Self {
        Self {
            deep_scan: true,
            allow_static_chain: true,
            allow_srs_offset_scan: false,
            allow_direct_scan: false,
        }
    }

    #[cfg(any(test, feature = "dangerous-route-scan"))]
    #[allow(dead_code)] // used by integration tests when feature enabled
    pub const fn full_deep_for_test() -> Self {
        Self {
            deep_scan: true,
            allow_static_chain: true,
            allow_srs_offset_scan: true,
            allow_direct_scan: true,
        }
    }
}

static SCAN_POLICY: OnceLock<RouteScanPolicy> = OnceLock::new();
static RESOLVER_MODE: OnceLock<RouteResolverMode> = OnceLock::new();
static ENABLE_GENERATION: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);

/// Enable-file names in priority order (highest first).
pub const RESOLVER_ENABLE_FILES: [(&'static str, RouteResolverMode); 5] = [
    ("truckpilot_route_resolver.full", RouteResolverMode::FullDeep),
    ("truckpilot_route_resolver.static", RouteResolverMode::StaticChain),
    (
        "truckpilot_route_resolver.route_candidate_table",
        RouteResolverMode::RouteCandidateTableOnly,
    ),
    (
        "truckpilot_route_resolver.game_ctrl_table",
        RouteResolverMode::GameCtrlTableOnly,
    ),
    ("truckpilot_route_resolver.gps_table", RouteResolverMode::GpsTableOnly),
];

/// Result of scanning the ETS2 plugins directory for resolver enable files.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResolverModeSelection {
    pub mode: RouteResolverMode,
    pub source_file: &'static str,
    pub ignored_lower_priority: Vec<&'static str>,
}

pub fn enable_file_generation() -> u64 {
    ENABLE_GENERATION.load(std::sync::atomic::Ordering::Relaxed)
}

pub fn bump_enable_file_generation() {
    ENABLE_GENERATION.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
}

#[cfg(test)]
std::thread_local! {
    static TEST_ENABLE_DIR: std::cell::RefCell<Option<std::path::PathBuf>> =
        const { std::cell::RefCell::new(None) };
}

/// Override enable-file directory for unit tests (thread-local temp dirs).
#[cfg(test)]
pub fn set_test_enable_dir(dir: Option<std::path::PathBuf>) {
    TEST_ENABLE_DIR.with(|cell| {
        *cell.borrow_mut() = dir;
    });
}

#[cfg(test)]
fn test_enable_dir() -> Option<std::path::PathBuf> {
    TEST_ENABLE_DIR.with(|cell| cell.borrow().clone())
}

fn enable_dir() -> Option<std::path::PathBuf> {
    #[cfg(test)]
    {
        if let Some(dir) = test_enable_dir() {
            return Some(dir);
        }
    }
    crate::diag_log::plugin_dir()
}

fn enable_file_exists(name: &str) -> bool {
    enable_dir()
        .map(|dir| dir.join(name).is_file())
        .unwrap_or(false)
}

/// Detect effective mode from enable files in `dir` (offline-testable).
pub fn detect_resolver_mode_selection_from_dir(dir: &std::path::Path) -> ResolverModeSelection {
    let mut present = Vec::new();
    for (file, mode) in RESOLVER_ENABLE_FILES {
        if dir.join(file).is_file() {
            present.push((file, mode));
        }
    }
    if present.is_empty() {
        return ResolverModeSelection {
            mode: RouteResolverMode::SafeDefault,
            source_file: "none",
            ignored_lower_priority: Vec::new(),
        };
    }
    let (source_file, mode) = present[0];
    let ignored_lower_priority = present[1..].iter().map(|(f, _)| *f).collect();
    ResolverModeSelection {
        mode,
        source_file,
        ignored_lower_priority,
    }
}

/// True when legacy `truckpilot_route_scan.enable` exists (ignored for mode selection).
pub fn legacy_route_scan_enable_present(dir: &std::path::Path) -> bool {
    dir.join("truckpilot_route_scan.enable").is_file()
}

/// Detect effective mode from enable files only (no env overrides).
pub fn detect_resolver_mode_selection() -> ResolverModeSelection {
    enable_dir()
        .map(|dir| detect_resolver_mode_selection_from_dir(&dir))
        .unwrap_or(ResolverModeSelection {
            mode: RouteResolverMode::SafeDefault,
            source_file: "none",
            ignored_lower_priority: Vec::new(),
        })
}

/// Log resolver mode once at DLL init (Sidecar).
pub fn log_route_resolver_mode_at_init() {
    use std::sync::Once;
    static LOGGED: Once = Once::new();
    LOGGED.call_once(|| {
        let sel = detect_resolver_mode_selection();
        for line in crate::resolver_guard::format_mode_init_log_lines(&sel) {
            crate::diag_log::event_force(&line);
        }
        if sel.mode.is_off() {
            crate::resolver_metrics::set_resolver_parked(true);
        }
    });
}

fn env_deep_scan_enabled() -> bool {
    std::env::var("TRUCKPILOT_ENABLE_DEEP_ROUTE_SCAN")
        .ok()
        .is_some_and(|v| v == "1" || v.eq_ignore_ascii_case("true"))
}

#[cfg(windows)]
fn file_deep_scan_enabled() -> bool {
    enable_file_exists("truckpilot_route_resolver.static")
}

#[cfg(not(windows))]
fn file_deep_scan_enabled() -> bool {
    false
}

#[cfg(windows)]
fn file_gps_table_mode_enabled() -> bool {
    enable_file_exists("truckpilot_route_resolver.gps_table")
}

#[cfg(not(windows))]
fn file_gps_table_mode_enabled() -> bool {
    false
}

#[cfg(windows)]
fn file_game_ctrl_table_mode_enabled() -> bool {
    enable_file_exists("truckpilot_route_resolver.game_ctrl_table")
}

#[cfg(not(windows))]
fn file_game_ctrl_table_mode_enabled() -> bool {
    false
}

#[cfg(windows)]
fn file_route_candidate_table_mode_enabled() -> bool {
    enable_file_exists("truckpilot_route_resolver.route_candidate_table")
}

#[cfg(not(windows))]
fn file_route_candidate_table_mode_enabled() -> bool {
    false
}

fn env_full_scan_enabled() -> bool {
    std::env::var("TRUCKPILOT_ENABLE_FULL_ROUTE_SCAN")
        .ok()
        .is_some_and(|v| v == "1" || v.eq_ignore_ascii_case("true"))
        || cfg!(feature = "dangerous-route-scan")
}

#[cfg(windows)]
fn file_full_scan_enabled() -> bool {
    enable_file_exists("truckpilot_route_resolver.full")
}

#[cfg(not(windows))]
fn file_full_scan_enabled() -> bool {
    false
}

fn static_scan_enabled() -> bool {
    file_deep_scan_enabled() || env_deep_scan_enabled() || cfg!(feature = "dangerous-route-scan")
}

fn full_scan_enabled() -> bool {
    file_full_scan_enabled() || env_full_scan_enabled()
}

/// Resolve effective mode from enable files (priority: full > static > route_candidate_table > game_ctrl_table > gps_table > off).
pub fn select_route_resolver_mode(
    gps_table_file: bool,
    game_ctrl_table_file: bool,
    route_candidate_table_file: bool,
    static_file: bool,
    full_file: bool,
) -> RouteResolverMode {
    if full_file {
        RouteResolverMode::FullDeep
    } else if static_file {
        RouteResolverMode::StaticChain
    } else if route_candidate_table_file {
        RouteResolverMode::RouteCandidateTableOnly
    } else if game_ctrl_table_file {
        RouteResolverMode::GameCtrlTableOnly
    } else if gps_table_file {
        RouteResolverMode::GpsTableOnly
    } else {
        RouteResolverMode::SafeDefault
    }
}

/// Effective resolver mode for this process.
pub fn route_resolver_mode() -> RouteResolverMode {
    #[cfg(test)]
    if let Some(ref dir) = test_enable_dir() {
        return detect_resolver_mode_selection_from_dir(dir).mode;
    }
    *RESOLVER_MODE.get_or_init(|| detect_resolver_mode_selection().mode)
}

/// Effective scan policy for this process (default: all deep scans off).
pub fn route_scan_policy() -> RouteScanPolicy {
    *SCAN_POLICY.get_or_init(|| match route_resolver_mode() {
        RouteResolverMode::FullDeep => RouteScanPolicy {
            deep_scan: true,
            allow_static_chain: true,
            allow_srs_offset_scan: true,
            allow_direct_scan: true,
        },
        RouteResolverMode::StaticChain => RouteScanPolicy::static_chain_only(),
        RouteResolverMode::SafeDefault
        | RouteResolverMode::GpsTableOnly
        | RouteResolverMode::GameCtrlTableOnly
        | RouteResolverMode::RouteCandidateTableOnly => RouteScanPolicy::safe_default(),
    })
}

#[cfg(test)]
#[allow(dead_code)] // OnceLock cannot reset; tests use constants directly.
pub fn reset_route_scan_policy_for_test() {
    // OnceLock cannot reset; tests call RouteScanPolicy constants directly.
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn safe_read_rejects_null() {
        assert_eq!(safe_read_u64(0), Err(ReadError::NullAddress));
        assert_eq!(safe_read_u32(0), Err(ReadError::NullAddress));
        assert_eq!(safe_read_f32(0), Err(ReadError::NullAddress));
    }

    #[test]
    fn safe_read_rejects_non_canonical() {
        assert_eq!(safe_read_u64(0xFFFF_8000_0000_0001), Err(ReadError::NonCanonical));
        assert_eq!(safe_read_u64(0x8), Err(ReadError::NonCanonical));
    }

    #[test]
    fn default_policy_is_safe() {
        let p = RouteScanPolicy::safe_default();
        assert!(!p.deep_scan);
        assert!(!p.allow_static_chain);
        assert!(!p.allow_srs_offset_scan);
        assert!(!p.allow_direct_scan);
    }

    #[test]
    fn static_chain_policy_disables_deep_scans() {
        let p = RouteScanPolicy::static_chain_only();
        assert!(p.deep_scan);
        assert!(p.allow_static_chain);
        assert!(!p.allow_srs_offset_scan);
        assert!(!p.allow_direct_scan);
    }

    #[test]
    fn page_protect_rejects_noaccess_and_guard() {
        assert_eq!(page_protect_allows_read(0x01), Err(ReadError::NoAccess));
        assert_eq!(page_protect_allows_read(0x100), Err(ReadError::GuardPage));
        assert!(page_protect_allows_read(0x04).is_ok());
    }

    #[test]
    fn resolver_default_policy_blocks_deep_scan() {
        let p = RouteScanPolicy::safe_default();
        assert!(!p.deep_scan);
    }

    #[test]
    fn gps_table_slot_count_is_33() {
        assert_eq!(GPS_TABLE_SLOT_COUNT, 33);
    }

    #[test]
    fn resolver_mode_gps_table_file_wins_over_default() {
        assert_eq!(
            select_route_resolver_mode(true, false, false, false, false),
            RouteResolverMode::GpsTableOnly
        );
    }

    #[test]
    fn resolver_mode_game_ctrl_table_beats_gps_table() {
        assert_eq!(
            select_route_resolver_mode(true, true, false, false, false),
            RouteResolverMode::GameCtrlTableOnly
        );
    }

    #[test]
    fn resolver_mode_route_candidate_table_beats_game_ctrl_table() {
        assert_eq!(
            select_route_resolver_mode(true, true, true, false, false),
            RouteResolverMode::RouteCandidateTableOnly
        );
    }

    #[test]
    fn resolver_mode_static_beats_route_candidate_table() {
        assert_eq!(
            select_route_resolver_mode(true, true, true, true, false),
            RouteResolverMode::StaticChain
        );
    }

    #[test]
    fn off_mode_sidecar_label() {
        assert_eq!(RouteResolverMode::SafeDefault.sidecar_label(), "off");
        assert!(RouteResolverMode::SafeDefault.is_off());
        assert!(!RouteResolverMode::SafeDefault.enables_resolver_work());
    }

    #[test]
    fn no_enable_files_selects_off() {
        assert_eq!(
            select_route_resolver_mode(false, false, false, false, false),
            RouteResolverMode::SafeDefault
        );
    }

    #[test]
    fn resolver_mode_full_beats_static_and_tables() {
        assert_eq!(
            select_route_resolver_mode(true, true, true, true, true),
            RouteResolverMode::FullDeep
        );
    }

    #[test]
    fn detect_off_when_no_files_present() {
        let sel = detect_resolver_mode_selection();
        assert_eq!(sel.mode, RouteResolverMode::SafeDefault);
        assert_eq!(sel.source_file, "none");
        assert!(sel.ignored_lower_priority.is_empty());
    }

    #[test]
    fn route_candidate_table_mode_sidecar_label() {
        assert_eq!(
            RouteResolverMode::RouteCandidateTableOnly.sidecar_label(),
            "route_candidate_table"
        );
    }

    #[test]
    fn game_ctrl_table_mode_sidecar_label() {
        assert_eq!(
            RouteResolverMode::GameCtrlTableOnly.sidecar_label(),
            "game_ctrl_table"
        );
    }

    #[test]
    fn resolver_mode_static_beats_gps_table() {
        assert_eq!(
            select_route_resolver_mode(true, false, false, true, false),
            RouteResolverMode::StaticChain
        );
    }

    #[test]
    fn resolver_mode_full_beats_gps_table_and_static() {
        assert_eq!(
            select_route_resolver_mode(true, false, false, true, true),
            RouteResolverMode::FullDeep
        );
    }

    #[test]
    fn gps_table_mode_sidecar_label() {
        assert_eq!(
            RouteResolverMode::GpsTableOnly.sidecar_label(),
            "gps_table"
        );
    }
}
