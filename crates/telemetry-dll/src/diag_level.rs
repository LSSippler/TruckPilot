//! Crash-safe ETS2 stutter bisect levels (`truckpilot_diag.*` plugin files).
//!
//! These levels isolate, one hot-path component at a time, where the telemetry
//! DLL starts costing ETS2 frame time relative to running with no DLL at all.
//! Each level is selected by an empty marker file in the ETS2 plugin directory
//! (next to `truckpilot_telemetry.dll`):
//!
//! ```text
//! truckpilot_diag.load_only          DLL loaded + scs_telemetry_init OK, nothing else
//! truckpilot_diag.init_only          + SHM/perf/RouteBlackboard created, no frame callbacks
//! truckpilot_diag.callback_noop      + frame callback: atomic counter only, no publish
//! truckpilot_diag.callback_counter   + per-frame perf snapshot publish
//! truckpilot_diag.callback_qpc       + per-frame QPC frame timing
//! truckpilot_diag.perf_snapshot      + dedicated perf-snapshot component
//! truckpilot_diag.telemetry_shm      + telemetry SHM full-copy per frame
//! truckpilot_diag.ready_event        + READY_EVENT SetEvent per frame
//! truckpilot_diag.route_bb           + RouteBlackboard frame write
//! truckpilot_diag.normal_default_off the current safe default/off behaviour
//! ```
//!
//! ## Priority
//! When several files exist, the **lowest / safest** level wins so diagnosis can
//! never silently escalate into more work than intended. The winner and every
//! ignored higher-priority file are logged once at init.
//!
//! ## Defaults
//! No `truckpilot_diag.*` file present → [`DiagLevel::Normal`], i.e. byte-for-byte
//! the established safe default/off path (resolver off, dispatch hard-off gate,
//! no worker wake, input honouring `truckpilot_input.disable`).
//!
//! ## Relationship to `truckpilot_minimal_telemetry.enable`
//! `truckpilot_minimal_telemetry.enable` keeps its **own legacy behaviour** and is
//! NOT remapped onto a diag level. When any `truckpilot_diag.*` file is present the
//! diag level wins and minimal telemetry is ignored (logged once). Roughly,
//! legacy minimal telemetry sits between [`DiagLevel::CallbackCounter`] and
//! [`DiagLevel::PerfSnapshot`] (counter + per-frame perf publish, no SHM/ready/bb).

use std::sync::OnceLock;

/// One stutter-bisect stage. Ordinal value is the stable `diag_level_code`
/// written to the perf SHM; lower = safer = higher selection priority.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u32)]
pub enum DiagLevel {
    /// DLL load + `scs_telemetry_init` returns OK. No SHM, callbacks, or worker.
    LoadOnly = 0,
    /// Init work (SHM/perf/RouteBlackboard) but no frame callbacks registered.
    InitOnly = 1,
    /// Frame callback registered; per frame only an atomic counter, then return.
    CallbackNoop = 2,
    /// Noop + per-frame perf snapshot publish (counter becomes live-visible).
    CallbackCounter = 3,
    /// Counter + per-frame QPC frame timing.
    CallbackQpc = 4,
    /// Dedicated perf-snapshot component on top of QPC timing.
    PerfSnapshot = 5,
    /// + telemetry SHM full-copy per frame.
    TelemetryShm = 6,
    /// + READY_EVENT SetEvent per frame.
    ReadyEvent = 7,
    /// + RouteBlackboard frame write per frame.
    RouteBb = 8,
    /// Current safe default/off behaviour (also selected when no diag file exists).
    Normal = 9,
}

/// Diag marker files in priority order (lowest / safest first).
pub const DIAG_LEVEL_FILES: [(&str, DiagLevel); 10] = [
    ("truckpilot_diag.load_only", DiagLevel::LoadOnly),
    ("truckpilot_diag.init_only", DiagLevel::InitOnly),
    ("truckpilot_diag.callback_noop", DiagLevel::CallbackNoop),
    ("truckpilot_diag.callback_counter", DiagLevel::CallbackCounter),
    ("truckpilot_diag.callback_qpc", DiagLevel::CallbackQpc),
    ("truckpilot_diag.perf_snapshot", DiagLevel::PerfSnapshot),
    ("truckpilot_diag.telemetry_shm", DiagLevel::TelemetryShm),
    ("truckpilot_diag.ready_event", DiagLevel::ReadyEvent),
    ("truckpilot_diag.route_bb", DiagLevel::RouteBb),
    ("truckpilot_diag.normal_default_off", DiagLevel::Normal),
];

impl DiagLevel {
    /// Stable numeric code published in the perf SHM (`diag_level_code`).
    pub const fn code(self) -> u32 {
        self as u32
    }

    /// Stable lowercase label matching the marker file suffix.
    pub const fn label(self) -> &'static str {
        match self {
            Self::LoadOnly => "load_only",
            Self::InitOnly => "init_only",
            Self::CallbackNoop => "callback_noop",
            Self::CallbackCounter => "callback_counter",
            Self::CallbackQpc => "callback_qpc",
            Self::PerfSnapshot => "perf_snapshot",
            Self::TelemetryShm => "telemetry_shm",
            Self::ReadyEvent => "ready_event",
            Self::RouteBb => "route_bb",
            Self::Normal => "normal_default_off",
        }
    }

    /// Map a perf `diag_level_code` back to a level (reader side / tests).
    pub const fn from_code(code: u32) -> Option<Self> {
        Some(match code {
            0 => Self::LoadOnly,
            1 => Self::InitOnly,
            2 => Self::CallbackNoop,
            3 => Self::CallbackCounter,
            4 => Self::CallbackQpc,
            5 => Self::PerfSnapshot,
            6 => Self::TelemetryShm,
            7 => Self::ReadyEvent,
            8 => Self::RouteBb,
            9 => Self::Normal,
            _ => return None,
        })
    }

    /// True for the established default/off behaviour (explicit file or no file).
    pub const fn is_normal(self) -> bool {
        matches!(self, Self::Normal)
    }

    /// True when SCS frame/channel callbacks are registered at init.
    /// `LoadOnly`/`InitOnly` deliberately register nothing.
    pub const fn registers_callbacks(self) -> bool {
        !matches!(self, Self::LoadOnly | Self::InitOnly)
    }

    /// True when the diag frame hot-path owns the callback (every level that
    /// registers callbacks but is not the full normal path).
    pub const fn intercepts_frame_path(self) -> bool {
        self.registers_callbacks() && !self.is_normal()
    }

    /// True when init creates telemetry SHM, perf SHM and the RouteBlackboard.
    /// Only `LoadOnly` creates nothing.
    pub const fn creates_shm_at_init(self) -> bool {
        !matches!(self, Self::LoadOnly)
    }

    /// True when the input plugin device may register. Only the normal path
    /// registers input (and only if `truckpilot_input.disable` is absent).
    pub const fn input_allowed(self) -> bool {
        self.is_normal()
    }

    /// True when the per-frame telemetry SHM full-copy runs.
    pub const fn telemetry_shm_frame_write(self) -> bool {
        matches!(
            self,
            Self::TelemetryShm | Self::ReadyEvent | Self::RouteBb | Self::Normal
        )
    }

    /// True when the per-frame READY_EVENT `SetEvent` runs.
    pub const fn ready_event_frame_set(self) -> bool {
        matches!(self, Self::ReadyEvent | Self::RouteBb | Self::Normal)
    }

    /// True when the per-frame RouteBlackboard frame write runs.
    pub const fn route_bb_frame_write(self) -> bool {
        matches!(self, Self::RouteBb | Self::Normal)
    }

    /// True when the frame callback dispatches route ticks (worker notify path).
    pub const fn dispatch_enabled(self) -> bool {
        self.is_normal()
    }

    /// True when the background resolver worker thread may start.
    pub const fn worker_enabled(self) -> bool {
        self.is_normal()
    }

    /// True when the resolver must be forced off and route enable files ignored.
    pub const fn resolver_forced_off(self) -> bool {
        !self.is_normal()
    }

    /// True when this level publishes a fresh perf snapshot on every frame.
    /// `CallbackNoop` deliberately does not, to keep the raw-callback baseline pure.
    pub const fn publishes_per_frame(self) -> bool {
        matches!(
            self,
            Self::CallbackCounter
                | Self::CallbackQpc
                | Self::PerfSnapshot
                | Self::TelemetryShm
                | Self::ReadyEvent
                | Self::RouteBb
        )
    }

    /// True when the frame body is wrapped in QPC timing (`frame_cb_us_*`).
    pub const fn qpc_frame_timing(self) -> bool {
        matches!(
            self,
            Self::CallbackQpc
                | Self::PerfSnapshot
                | Self::TelemetryShm
                | Self::ReadyEvent
                | Self::RouteBb
        )
    }
}

/// Result of scanning the plugin directory for diag marker files.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DiagLevelSelection {
    pub level: DiagLevel,
    /// Marker file that selected `level`, or `"none"` when no diag file exists.
    pub source_file: &'static str,
    /// Higher-priority (less safe) marker files present but ignored.
    pub ignored_higher: Vec<&'static str>,
}

/// Detect the effective level from marker files in `dir` (offline-testable).
/// Lowest / safest level wins.
pub fn detect_level_selection_from_dir(dir: &std::path::Path) -> DiagLevelSelection {
    let mut present: Vec<(&'static str, DiagLevel)> = Vec::new();
    for (file, level) in DIAG_LEVEL_FILES {
        if dir.join(file).is_file() {
            present.push((file, level));
        }
    }
    if present.is_empty() {
        return DiagLevelSelection {
            level: DiagLevel::Normal,
            source_file: "none",
            ignored_higher: Vec::new(),
        };
    }
    // DIAG_LEVEL_FILES is already lowest-first; present preserves that order.
    let (source_file, level) = present[0];
    let ignored_higher = present[1..].iter().map(|(f, _)| *f).collect();
    DiagLevelSelection {
        level,
        source_file,
        ignored_higher,
    }
}

/// Detect the effective level selection from the live plugin directory.
pub fn detect_level_selection() -> DiagLevelSelection {
    crate::safe_mem::enable_dir()
        .map(|dir| detect_level_selection_from_dir(&dir))
        .unwrap_or(DiagLevelSelection {
            level: DiagLevel::Normal,
            source_file: "none",
            ignored_higher: Vec::new(),
        })
}

static ACTIVE_LEVEL: OnceLock<DiagLevel> = OnceLock::new();

/// Effective diag level for this process.
///
/// Cached once (production) so the per-frame hot path performs no filesystem
/// access. Under `cfg(test)` it re-detects every call so tests can swap the
/// enable directory between cases.
pub fn active() -> DiagLevel {
    #[cfg(test)]
    {
        detect_level_selection().level
    }
    #[cfg(not(test))]
    {
        *ACTIVE_LEVEL.get_or_init(|| detect_level_selection().level)
    }
}

/// True when the active level forces the resolver off (every non-normal level).
pub fn resolver_forced_off() -> bool {
    active().resolver_forced_off()
}

/// Effective input-registration state for this process.
///
/// Only `normal_default_off` *without* `truckpilot_input.disable` registers the
/// SCS input device. Every diag bisect level below normal — and normal with the
/// disable file present — reports `false`. Single source of truth for both the
/// sidecar init log and the perf SHM (`diag_input_registered`).
pub fn effective_input_registered() -> bool {
    active().input_allowed() && !crate::safe_mem::input_plugin_disabled()
}

/// Sidecar lines emitted once at init for a level selection (offline-testable).
///
/// `input_registered` is the EFFECTIVE state (see [`effective_input_registered`]),
/// so the sidecar log always agrees with the perf SHM's `diag_input_registered`
/// — including `normal_default_off` + `truckpilot_input.disable` → `false`.
pub fn format_level_init_log_lines(
    sel: &DiagLevelSelection,
    input_registered: bool,
    resolver_enable_present: bool,
    minimal_telemetry_present: bool,
) -> Vec<String> {
    let l = sel.level;
    let mut lines = vec![format!("diagnostic level={}", l.label())];
    if sel.source_file != "none" {
        lines.push(format!(
            "diagnostic level selected={} source={}",
            l.label(),
            sel.source_file
        ));
    }
    if !sel.ignored_higher.is_empty() {
        lines.push(format!(
            "diagnostic ignored higher-priority diag level files: {}",
            sel.ignored_higher.join(", ")
        ));
    }
    lines.push(format!(
        "diagnostic callbacks_registered={}",
        l.registers_callbacks()
    ));
    lines.push(format!("diagnostic input_registered={input_registered}"));
    lines.push(format!(
        "diagnostic telemetry_shm={}",
        l.telemetry_shm_frame_write()
    ));
    lines.push(format!(
        "diagnostic ready_event={}",
        l.ready_event_frame_set()
    ));
    lines.push(format!(
        "diagnostic route_bb_frame_write={}",
        l.route_bb_frame_write()
    ));
    lines.push(format!("diagnostic dispatch={}", l.dispatch_enabled()));
    lines.push(format!("diagnostic worker={}", l.worker_enabled()));
    if !l.is_normal() {
        if l == DiagLevel::LoadOnly {
            lines.push("diagnostic load_only: no shm, no callbacks, no worker".into());
        } else if l == DiagLevel::InitOnly {
            lines.push("callbacks disabled by diagnostic level".into());
        }
        if resolver_enable_present {
            lines.push(format!(
                "route resolver enable files ignored because diagnostic level={}",
                l.label()
            ));
        }
        if minimal_telemetry_present {
            lines.push(format!(
                "minimal telemetry ignored because diagnostic level={}",
                l.label()
            ));
        }
    }
    lines
}

/// Push the active level's configuration into the perf SHM counters.
pub fn publish_config(level: DiagLevel, input_will_register: bool) {
    crate::frame_perf::set_diag_config(crate::frame_perf::DiagConfig {
        level_code: level.code(),
        callbacks_registered: level.registers_callbacks(),
        input_registered: input_will_register,
        telemetry_shm_enabled: level.telemetry_shm_frame_write(),
        ready_event_enabled: level.ready_event_frame_set(),
        route_bb_frame_write_enabled: level.route_bb_frame_write(),
        dispatch_enabled: level.dispatch_enabled(),
        worker_enabled: level.worker_enabled(),
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_dir(tag: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("tp-diag-level-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");
        dir
    }

    #[test]
    fn no_file_selects_normal() {
        let dir = temp_dir("none");
        let sel = detect_level_selection_from_dir(&dir);
        assert_eq!(sel.level, DiagLevel::Normal);
        assert_eq!(sel.source_file, "none");
        assert!(sel.ignored_higher.is_empty());
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn each_file_selects_its_level() {
        for (file, level) in DIAG_LEVEL_FILES {
            let dir = temp_dir(level.label());
            std::fs::write(dir.join(file), b"").expect("marker");
            let sel = detect_level_selection_from_dir(&dir);
            assert_eq!(sel.level, level, "file {file}");
            assert_eq!(sel.source_file, file);
            let _ = std::fs::remove_dir_all(&dir);
        }
    }

    #[test]
    fn lowest_safest_level_wins_when_multiple_present() {
        let dir = temp_dir("multi");
        std::fs::write(dir.join("truckpilot_diag.route_bb"), b"").expect("a");
        std::fs::write(dir.join("truckpilot_diag.callback_noop"), b"").expect("b");
        std::fs::write(dir.join("truckpilot_diag.normal_default_off"), b"").expect("c");
        let sel = detect_level_selection_from_dir(&dir);
        assert_eq!(sel.level, DiagLevel::CallbackNoop);
        assert_eq!(sel.source_file, "truckpilot_diag.callback_noop");
        assert!(sel.ignored_higher.contains(&"truckpilot_diag.route_bb"));
        assert!(sel.ignored_higher.contains(&"truckpilot_diag.normal_default_off"));
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn code_round_trips_for_all_levels() {
        for (_, level) in DIAG_LEVEL_FILES {
            assert_eq!(DiagLevel::from_code(level.code()), Some(level));
        }
        assert_eq!(DiagLevel::from_code(99), None);
    }

    #[test]
    fn component_gating_is_monotonic_and_correct() {
        use DiagLevel::*;
        // No frame callbacks below callback_noop.
        assert!(!LoadOnly.registers_callbacks());
        assert!(!InitOnly.registers_callbacks());
        assert!(CallbackNoop.registers_callbacks());
        // Only LoadOnly creates nothing at init.
        assert!(!LoadOnly.creates_shm_at_init());
        assert!(InitOnly.creates_shm_at_init());
        // Diag levels never register input.
        for l in [LoadOnly, InitOnly, CallbackNoop, CallbackQpc, TelemetryShm, ReadyEvent, RouteBb] {
            assert!(!l.input_allowed(), "{l:?}");
            assert!(l.resolver_forced_off(), "{l:?}");
            assert!(!l.dispatch_enabled(), "{l:?}");
            assert!(!l.worker_enabled(), "{l:?}");
        }
        // Telemetry SHM full-copy introduced at TelemetryShm and cumulative upward.
        assert!(!PerfSnapshot.telemetry_shm_frame_write());
        assert!(TelemetryShm.telemetry_shm_frame_write());
        assert!(ReadyEvent.telemetry_shm_frame_write());
        assert!(RouteBb.telemetry_shm_frame_write());
        // Ready event introduced at ReadyEvent, cumulative.
        assert!(!TelemetryShm.ready_event_frame_set());
        assert!(ReadyEvent.ready_event_frame_set());
        assert!(RouteBb.ready_event_frame_set());
        // Route BB frame write introduced at RouteBb.
        assert!(!ReadyEvent.route_bb_frame_write());
        assert!(RouteBb.route_bb_frame_write());
        // Normal enables every component.
        assert!(Normal.telemetry_shm_frame_write());
        assert!(Normal.ready_event_frame_set());
        assert!(Normal.route_bb_frame_write());
        assert!(Normal.dispatch_enabled());
        assert!(Normal.worker_enabled());
        assert!(Normal.input_allowed());
        assert!(!Normal.resolver_forced_off());
        // callback_noop is the only callback level that does not publish per frame.
        assert!(!CallbackNoop.publishes_per_frame());
        assert!(CallbackCounter.publishes_per_frame());
        assert!(!CallbackCounter.qpc_frame_timing());
        assert!(CallbackQpc.qpc_frame_timing());
    }

    #[test]
    fn init_log_lines_describe_components_and_ignored_files() {
        let sel = DiagLevelSelection {
            level: DiagLevel::CallbackQpc,
            source_file: "truckpilot_diag.callback_qpc",
            ignored_higher: vec!["truckpilot_diag.route_bb"],
        };
        // Diag levels never register input -> effective input_registered = false.
        let lines = format_level_init_log_lines(&sel, false, true, true);
        let joined = lines.join("\n");
        assert!(joined.contains("diagnostic level=callback_qpc"));
        assert!(joined.contains("diagnostic callbacks_registered=true"));
        assert!(joined.contains("diagnostic input_registered=false"));
        assert!(joined.contains("diagnostic telemetry_shm=false"));
        assert!(joined.contains("diagnostic dispatch=false"));
        assert!(joined.contains("diagnostic worker=false"));
        assert!(joined.contains("ignored higher-priority diag level files: truckpilot_diag.route_bb"));
        assert!(joined.contains(
            "route resolver enable files ignored because diagnostic level=callback_qpc"
        ));
        assert!(joined.contains("minimal telemetry ignored because diagnostic level=callback_qpc"));
    }

    #[test]
    fn normal_init_log_has_no_ignore_warnings() {
        let sel = DiagLevelSelection {
            level: DiagLevel::Normal,
            source_file: "none",
            ignored_higher: Vec::new(),
        };
        // normal_default_off without input.disable -> input registered.
        let lines = format_level_init_log_lines(&sel, true, true, true);
        let joined = lines.join("\n");
        assert!(joined.contains("diagnostic level=normal_default_off"));
        assert!(joined.contains("diagnostic input_registered=true"));
        assert!(joined.contains("diagnostic dispatch=true"));
        assert!(joined.contains("diagnostic worker=true"));
        assert!(!joined.contains("ignored because diagnostic level"));
    }
}
