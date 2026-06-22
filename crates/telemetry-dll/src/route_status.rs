//! Route resolver / publish status codes and rate-limited sidecar logging.
//!
//! Numeric codes are mirrored in `crates/telemetry/src/nav_route.rs` for readers.

/// `reserved[0]` status bits (low 16 bits used; high bits reserved).
pub const ROUTE_BB_STATUS_DLL_ACTIVE: u32 = 1;
pub const ROUTE_BB_STATUS_ROUTE_TASK_OK: u32 = 1 << 1;
pub const ROUTE_BB_STATUS_ROUTE_TICK_SEEN: u32 = 1 << 2;
pub const ROUTE_BB_STATUS_FRAME_CB_SEEN: u32 = 1 << 3;
pub const ROUTE_BB_STATUS_PAUSE_EVENT_SEEN: u32 = 1 << 4;
pub const ROUTE_BB_STATUS_PAUSE_GATE_ACTIVE: u32 = 1 << 5;
/// `reserved[0]` — no SCS frame_end observed while frame_start callbacks run.
pub const ROUTE_BB_STATUS_FRAME_END_MISSING: u32 = 1 << 6;
/// `reserved[0]` — last route tick used frame_start fallback (not frame_end).
pub const ROUTE_BB_STATUS_TICK_SOURCE_FALLBACK: u32 = 1 << 7;

/// SCS telemetry frame event IDs (for route tick source selection).
pub const SCS_EVENT_FRAME_START: u32 = 2;
pub const SCS_EVENT_FRAME_END: u32 = 3;

/// If no frame_end arrives for this long, route tick falls back to frame_start.
pub const FRAME_END_STALE_US: u64 = 1_000_000;

/// Where the route resolver tick was triggered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RouteTickSource {
    FrameEnd,
    FrameStartFallback,
}

impl RouteTickSource {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::FrameEnd => "frame_end",
            Self::FrameStartFallback => "frame_start_fallback",
        }
    }
}

/// Pack `reserved[5]`: low 16 = route_tick_count, high 16 = frame_start_count.
pub fn pack_route_tick_meta(route_tick_count: u32, frame_start_count: u32) -> u32 {
    (route_tick_count & 0xFFFF) | ((frame_start_count & 0xFFFF) << 16)
}

/// Decode `reserved[5]`: low 16 bits = route tick count, high 16 bits = frame_start count.
pub fn unpack_route_tick_meta(packed: u32) -> (u32, u32) {
    (packed & 0xFFFF, packed >> 16)
}

/// Whether route tick should run on frame_start (frame_end missing or stale).
pub fn should_frame_start_fallback(
    frame_end_event_count: u32,
    last_frame_end_event_us: u64,
    now_us: u64,
) -> bool {
    if frame_end_event_count == 0 {
        return true;
    }
    if last_frame_end_event_us == 0 {
        return true;
    }
    now_us.saturating_sub(last_frame_end_event_us) > FRAME_END_STALE_US
}

/// `reserved[1]` — last resolver outcome.
pub const RESOLVE_NONE: u32 = 0;
/// Generic module/AOB scan failure (legacy aggregate).
pub const RESOLVE_MODULE_SCAN_FAILED: u32 = 1;
pub const RESOLVE_GPS_PTR_NULL: u32 = 2;
pub const RESOLVE_ROUTE_TASK_PTR_NULL: u32 = 3;
pub const RESOLVE_ROUTE_ITEMS_PTR_NULL: u32 = 4;
pub const RESOLVE_ITEMS_EMPTY: u32 = 5;
pub const RESOLVE_FIRST_UID_ZERO: u32 = 6;
pub const RESOLVE_WAYPOINTS_COLLECTED: u32 = 7;
pub const RESOLVE_TICK_THROTTLED: u32 = 8;
pub const RESOLVE_NOT_IN_WORLD: u32 = 9;
pub const RESOLVE_TRIP_DISTANCE_LOW: u32 = 10;
pub const RESOLVE_POINTER_CHAIN_FAILED: u32 = 11;
pub const RESOLVE_VALIDATION_FAILED: u32 = 12;
pub const RESOLVE_PANIC_CAUGHT: u32 = 13;
/// Main executable module handle was null.
pub const RESOLVE_MODULE_NOT_FOUND: u32 = 14;
/// PE optional header could not be parsed.
pub const RESOLVE_MODULE_PE_PARSE_FAILED: u32 = 15;
/// AOB pattern had zero matches in the main module image.
pub const RESOLVE_PATTERN_NOT_FOUND: u32 = 16;
/// AOB pattern matched but resolved to conflicting singleton slots.
pub const RESOLVE_PATTERN_MULTIPLE_MATCHES: u32 = 17;
/// AOB matched but RIP slot or game_ctrl pointer was invalid.
pub const RESOLVE_PATTERN_MATCH_INVALID: u32 = 18;
/// Singleton slot resolved but game_ctrl heap pointer was null/invalid.
pub const RESOLVE_GAME_CTRL_NULL: u32 = 19;
/// Resolver skipped walk because ETS2 is paused (ESC menu).
pub const RESOLVE_PAUSED_NO_ROUTE_WALK: u32 = 20;
/// GPS manager resolved; informational mid-resolve (not a terminal failure).
pub const RESOLVE_GPS_RESOLVED: u32 = 21;
/// All route_task chain candidates yielded null/invalid route_task.
pub const RESOLVE_ROUTE_TASK_CANDIDATE_NULL: u32 = 22;
/// route_task found but physical_route_items pointer invalid on all layouts.
pub const RESOLVE_ROUTE_ITEMS_CANDIDATE_NULL: u32 = 23;
/// Items array present but empty after validation trim.
pub const RESOLVE_ROUTE_ITEMS_EMPTY: u32 = 24;
/// One named chain candidate failed (logged; try next).
pub const RESOLVE_ROUTE_CHAIN_CANDIDATE_FAILED: u32 = 25;
/// Every static + dynamic chain candidate failed.
pub const RESOLVE_ROUTE_CHAIN_ALL_FAILED: u32 = 26;
/// `gps + 0x08` simple_route_src pointer is null.
pub const RESOLVE_SIMPLE_ROUTE_SRC_NULL: u32 = 27;
/// SRS pointer not at +0x08; dynamic offset scan found no route.
pub const RESOLVE_SIMPLE_ROUTE_SRC_OFFSET_UNKNOWN: u32 = 28;
/// Dynamic SRS offset scan exhausted with no UID-bearing items.
pub const RESOLVE_SRS_OFFSET_SCAN_FAILED: u32 = 29;
/// Deep route memory scan disabled (default — crash-safe mode).
pub const RESOLVE_UNSAFE_ROUTE_SCAN_DISABLED: u32 = 30;
/// GPS resolved but route chain walk disabled for stability.
pub const RESOLVE_GPS_RESOLVED_ROUTE_SCAN_DISABLED: u32 = 31;
/// Route scan blocked during post-start warmup window.
pub const RESOLVE_ROUTE_SCAN_WARMUP: u32 = 32;
/// Route scan blocked until telemetry frames are stable.
pub const RESOLVE_ROUTE_SCAN_WAITING_FOR_STABLE_WORLD: u32 = 33;
/// GPS pointer table diagnostic completed (`gps_table_only` mode).
pub const RESOLVE_GPS_TABLE_ONLY_DONE: u32 = 34;
/// GPS manager resolved but `gps+0x00` table base unreadable.
pub const RESOLVE_GPS_TABLE_READ_FAILED: u32 = 35;
/// GPS manager could not be resolved (`gps_table_only` mode).
pub const RESOLVE_GPS_MANAGER_NOT_RESOLVED: u32 = 36;
/// Game-ctrl pointer table diagnostic completed (`game_ctrl_table` mode).
pub const RESOLVE_GAME_CTRL_TABLE_ONLY_DONE: u32 = 37;
/// `game_ctrl` resolved but table base unreadable (`game_ctrl_table` mode).
pub const RESOLVE_GAME_CTRL_TABLE_READ_FAILED: u32 = 38;
/// Route candidate table diagnostic completed (`route_candidate_table` mode).
pub const RESOLVE_ROUTE_CANDIDATE_TABLE_DONE: u32 = 39;
/// `game_ctrl` resolved but route candidate table read failed.
pub const RESOLVE_ROUTE_CANDIDATE_TABLE_READ_FAILED: u32 = 40;
/// Background resolver worker is active (informational).
pub const RESOLVE_ROUTE_RESOLVER_WORKER_ACTIVE: u32 = 41;
/// Resolver parked after retry/scan limit or diagnostic done.
pub const RESOLVE_ROUTE_RESOLVER_PARKED: u32 = 42;
/// Resolver waiting for exponential backoff.
pub const RESOLVE_ROUTE_RESOLVER_BACKOFF: u32 = 43;
/// Session `game_ctrl` cache used (no full pattern scan).
pub const RESOLVE_ROUTE_RESOLVER_CACHE_HIT: u32 = 44;
/// Full pattern scan limit reached for this session.
pub const RESOLVE_ROUTE_RESOLVER_SCAN_LIMITED: u32 = 45;
/// Default crash-safe mode — resolver fully disabled (no memory walks).
pub const RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE: u32 = 46;

/// Minimum time after SCS `started` before any route memory walk.
pub const ROUTE_SCAN_WARMUP_US: u64 = 10_000_000;
/// Minimum frame_start callbacks before route diagnostics run.
pub const ROUTE_SCAN_MIN_FRAME_START: u32 = 60;
/// Minimum interval between route resolver walks (crash-safe default).
pub const ROUTE_SCAN_MIN_INTERVAL_US: u64 = 2_000_000;

/// `reserved[3]` — last publish outcome.
pub const PUBLISH_NONE: u32 = 0;
pub const PUBLISH_EMPTY: u32 = 1;
pub const PUBLISH_ACTIVE: u32 = 2;
pub const PUBLISH_FAILED: u32 = 3;

/// SCS telemetry event IDs (for world-reset reason codes in `reserved[7]`).
pub const WORLD_RESET_REASON_NONE: u32 = 0;
pub const WORLD_RESET_REASON_PAUSED: u32 = 4;
pub const WORLD_RESET_REASON_UNPAUSED: u32 = 5;

pub fn world_reset_reason_str(code: u32) -> &'static str {
    match code {
        WORLD_RESET_REASON_PAUSED => "paused",
        WORLD_RESET_REASON_UNPAUSED => "unpaused",
        _ => "none",
    }
}

/// Pack world-reset count (low 16) + reason (bits 16–23) + suppressed-since-last (bits 24–31).
pub fn pack_world_reset(count: u32, reason: u32, suppressed_since_last: u32) -> u32 {
    (count & 0xFFFF)
        | ((reason & 0xFF) << 16)
        | ((suppressed_since_last & 0xFF) << 24)
}

#[cfg(test)]
pub fn unpack_world_reset(packed: u32) -> (u32, u32, u32) {
    (
        packed & 0xFFFF,
        (packed >> 16) & 0xFF,
        packed >> 24,
    )
}

/// Pack `reserved[6]`: low 16 = last waypoint count, high 16 = frame_end callback count.
pub fn pack_waypoint_frame_end(waypoint_count: u32, frame_end_count: u32) -> u32 {
    (waypoint_count & 0xFFFF) | ((frame_end_count & 0xFFFF) << 16)
}

#[cfg(test)]
pub fn unpack_waypoint_frame_end(packed: u32) -> (u32, u32) {
    (packed & 0xFFFF, packed >> 16)
}

/// Minimum interval between effective world-resolver resets (5 s).
pub const WORLD_RESET_MIN_INTERVAL_US: u64 = 5_000_000;

/// Debounce rapid repeated triggers (e.g. duplicate pause edges).
pub struct DebounceGate {
    last_fire_us: Option<u64>,
    min_interval_us: u64,
    suppressed: u32,
}

impl DebounceGate {
    pub const fn new(min_interval_us: u64) -> Self {
        Self {
            last_fire_us: None,
            min_interval_us,
            suppressed: 0,
        }
    }

    /// Returns `true` when the action should run; otherwise increments `suppressed`.
    pub fn try_fire(&mut self, now_us: u64) -> bool {
        if let Some(last) = self.last_fire_us {
            if now_us.saturating_sub(last) < self.min_interval_us {
                self.suppressed = self.suppressed.saturating_add(1);
                return false;
            }
        }
        self.last_fire_us = Some(now_us);
        true
    }

    pub fn suppressed(&self) -> u32 {
        self.suppressed
    }

    pub fn take_suppressed(&mut self) -> u32 {
        let n = self.suppressed;
        self.suppressed = 0;
        n
    }
}

pub fn telemetry_event_name(id: u32) -> &'static str {
    match id {
        0 => "invalid",
        1 => "started",
        2 => "frame_start",
        3 => "frame_end",
        4 => "paused",
        5 => "unpaused",
        6 => "configuration",
        _ => "unknown",
    }
}

pub fn effective_resolve_status(stored: u32, attempts: u32) -> u32 {
    if stored == RESOLVE_NONE && attempts > 0 {
        RESOLVE_MODULE_SCAN_FAILED
    } else {
        stored
    }
}

pub fn resolve_status_str(code: u32) -> &'static str {
    match code {
        RESOLVE_NONE => "none",
        RESOLVE_MODULE_SCAN_FAILED => "module_scan_failed",
        RESOLVE_GPS_PTR_NULL => "gps_ptr_null",
        RESOLVE_ROUTE_TASK_PTR_NULL => "route_task_ptr_null",
        RESOLVE_ROUTE_ITEMS_PTR_NULL => "route_items_ptr_null",
        RESOLVE_ITEMS_EMPTY => "items_empty",
        RESOLVE_FIRST_UID_ZERO => "first_uid_zero",
        RESOLVE_WAYPOINTS_COLLECTED => "waypoints_collected",
        RESOLVE_TICK_THROTTLED => "tick_throttled",
        RESOLVE_NOT_IN_WORLD => "not_in_world",
        RESOLVE_TRIP_DISTANCE_LOW => "trip_distance_low",
        RESOLVE_POINTER_CHAIN_FAILED => "pointer_chain_failed",
        RESOLVE_VALIDATION_FAILED => "validation_failed",
        RESOLVE_PANIC_CAUGHT => "panic_caught",
        RESOLVE_MODULE_NOT_FOUND => "module_not_found",
        RESOLVE_MODULE_PE_PARSE_FAILED => "module_pe_parse_failed",
        RESOLVE_PATTERN_NOT_FOUND => "pattern_not_found",
        RESOLVE_PATTERN_MULTIPLE_MATCHES => "pattern_multiple_matches",
        RESOLVE_PATTERN_MATCH_INVALID => "pattern_match_invalid",
        RESOLVE_GAME_CTRL_NULL => "game_ctrl_null",
        RESOLVE_PAUSED_NO_ROUTE_WALK => "paused_no_route_walk",
        RESOLVE_GPS_RESOLVED => "gps_resolved",
        RESOLVE_ROUTE_TASK_CANDIDATE_NULL => "route_task_candidate_null",
        RESOLVE_ROUTE_ITEMS_CANDIDATE_NULL => "route_items_candidate_null",
        RESOLVE_ROUTE_ITEMS_EMPTY => "route_items_empty",
        RESOLVE_ROUTE_CHAIN_CANDIDATE_FAILED => "route_chain_candidate_failed",
        RESOLVE_ROUTE_CHAIN_ALL_FAILED => "route_chain_all_failed",
        RESOLVE_SIMPLE_ROUTE_SRC_NULL => "simple_route_src_null",
        RESOLVE_SIMPLE_ROUTE_SRC_OFFSET_UNKNOWN => "simple_route_src_offset_unknown",
        RESOLVE_SRS_OFFSET_SCAN_FAILED => "srs_offset_scan_failed",
        RESOLVE_UNSAFE_ROUTE_SCAN_DISABLED => "unsafe_route_scan_disabled",
        RESOLVE_GPS_RESOLVED_ROUTE_SCAN_DISABLED => "gps_resolved_route_scan_disabled",
        RESOLVE_ROUTE_SCAN_WARMUP => "route_scan_warmup",
        RESOLVE_ROUTE_SCAN_WAITING_FOR_STABLE_WORLD => "route_scan_waiting_for_stable_world",
        RESOLVE_GPS_TABLE_ONLY_DONE => "gps_table_only_done",
        RESOLVE_GPS_TABLE_READ_FAILED => "gps_table_read_failed",
        RESOLVE_GPS_MANAGER_NOT_RESOLVED => "gps_manager_not_resolved",
        RESOLVE_GAME_CTRL_TABLE_ONLY_DONE => "game_ctrl_table_only_done",
        RESOLVE_GAME_CTRL_TABLE_READ_FAILED => "game_ctrl_table_read_failed",
        RESOLVE_ROUTE_CANDIDATE_TABLE_DONE => "route_candidate_table_done",
        RESOLVE_ROUTE_CANDIDATE_TABLE_READ_FAILED => "route_candidate_table_read_failed",
        RESOLVE_ROUTE_RESOLVER_WORKER_ACTIVE => "route_resolver_worker_active",
        RESOLVE_ROUTE_RESOLVER_PARKED => "route_resolver_parked",
        RESOLVE_ROUTE_RESOLVER_BACKOFF => "route_resolver_backoff",
        RESOLVE_ROUTE_RESOLVER_CACHE_HIT => "route_resolver_cache_hit",
        RESOLVE_ROUTE_RESOLVER_SCAN_LIMITED => "route_resolver_scan_limited",
        RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE => "route_resolver_disabled_safe_mode",
        _ => "unknown",
    }
}

pub fn publish_status_str(code: u32) -> &'static str {
    match code {
        PUBLISH_NONE => "none",
        PUBLISH_EMPTY => "publish_empty",
        PUBLISH_ACTIVE => "publish_active",
        PUBLISH_FAILED => "publish_failed",
        _ => "unknown",
    }
}

/// Rate-limited logger: first [`LOG_BURST`] events per key, then every 5 s, always on key change.
pub struct RateLog {
    burst_left: u32,
    last_log_us: u64,
    last_key: Option<String>,
}

impl RateLog {
    pub const LOG_BURST: u32 = 5;
    pub const LOG_INTERVAL_US: u64 = 5_000_000;

    pub const fn new() -> Self {
        Self {
            burst_left: Self::LOG_BURST,
            last_log_us: 0,
            last_key: None,
        }
    }

    pub fn should_log(&mut self, key: &str, now_us: u64, force: bool) -> bool {
        if force {
            return true;
        }
        if self.last_key.as_deref() != Some(key) {
            return true;
        }
        if self.burst_left > 0 {
            return true;
        }
        now_us.saturating_sub(self.last_log_us) >= Self::LOG_INTERVAL_US
    }

    pub fn record(&mut self, key: &str, now_us: u64) {
        if self.last_key.as_deref() != Some(key) {
            self.burst_left = Self::LOG_BURST;
        }
        if self.burst_left > 0 {
            self.burst_left -= 1;
        }
        self.last_key = Some(key.to_string());
        self.last_log_us = now_us;
    }

    pub fn event(&mut self, key: &str, msg: &str, now_us: u64, force: bool) {
        if self.should_log(key, now_us, force) {
            crate::diag_log::event_force(msg);
            self.record(key, now_us);
        }
    }
}

/// Log only at explicit count milestones (1–5, then 1k/5k/10k/…).
pub struct MilestoneLog {
    last_logged: u32,
}

impl MilestoneLog {
    pub const fn new() -> Self {
        Self { last_logged: 0 }
    }

    fn is_milestone(count: u32) -> bool {
        if count <= 5 {
            return true;
        }
        if count == 1_000 || count == 5_000 || count == 10_000 {
            return true;
        }
        count.is_multiple_of(50_000)
    }

    /// Returns `true` when this count should produce a log line.
    pub fn should_log(&mut self, count: u32) -> bool {
        if count <= self.last_logged {
            return false;
        }
        if Self::is_milestone(count) {
            self.last_logged = count;
            true
        } else {
            false
        }
    }
}

/// Aggregate suppressed world-reset events — log at most every 5 s with delta.
pub struct SuppressedSummaryLog {
    last_log_us: u64,
    last_total: u32,
}

impl SuppressedSummaryLog {
    pub const INTERVAL_US: u64 = 5_000_000;

    pub const fn new() -> Self {
        Self {
            last_log_us: 0,
            last_total: 0,
        }
    }

    pub fn maybe_log(&mut self, reason: &str, total_suppressed: u32, now_us: u64) {
        if total_suppressed <= self.last_total {
            return;
        }
        let delta = total_suppressed - self.last_total;
        let due = self.last_log_us == 0
            || now_us.saturating_sub(self.last_log_us) >= Self::INTERVAL_US;
        if !due {
            return;
        }
        crate::diag_log::event_force(&format!(
            "world event reset suppressed reason={reason} suppressed_count={total_suppressed} delta={delta}"
        ));
        self.last_log_us = now_us;
        self.last_total = total_suppressed;
    }
}

/// Pause/running state tracked from SCS pause/unpause events.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PauseState {
    Unknown,
    Running,
    Paused,
}

/// Action to take after an SCS world telemetry event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WorldEventAction {
    /// No side effects.
    Ignore,
    /// Log first `started` once per session — never resets resolver.
    LogStartedOnce,
    /// Debounced soft-reset of GPS cache (pause/unpause edge only).
    SoftReset { reason: u32 },
}

/// Edge-detecting world-event tracker — unit-testable without the DLL loaded.
#[derive(Debug, Clone, Copy)]
pub struct WorldEventState {
    pause_state: PauseState,
    started_logged: bool,
    pause_event_seen: bool,
}

/// Outcome of pause-gate evaluation for route walk.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PauseGateDecision {
    pub block_walk: bool,
    pub pause_event_seen: bool,
    pub pause_gate_active: bool,
    pub reason: &'static str,
}

/// Returns `Ok(())` when route memory diagnostics may run; otherwise the blocking status code.
pub fn evaluate_route_scan_gate(
    now_us: u64,
    session_started_us: u64,
    frame_start_count: u32,
) -> Result<(), u32> {
    if session_started_us == 0 {
        return Err(RESOLVE_ROUTE_SCAN_WAITING_FOR_STABLE_WORLD);
    }
    if now_us.saturating_sub(session_started_us) < ROUTE_SCAN_WARMUP_US {
        return Err(RESOLVE_ROUTE_SCAN_WARMUP);
    }
    if frame_start_count < ROUTE_SCAN_MIN_FRAME_START {
        return Err(RESOLVE_ROUTE_SCAN_WAITING_FOR_STABLE_WORLD);
    }
    Ok(())
}

impl PauseGateDecision {
    pub const NONE: Self = Self {
        block_walk: false,
        pause_event_seen: false,
        pause_gate_active: false,
        reason: "none",
    };
}

/// Decide whether route walk should be blocked due to pause/menu state.
///
/// Frame-end ticks (`frame_end_count > 0`) mean the simulation is advancing;
/// a stale SCS `paused` event must not block the pointer chain in that case.
pub fn evaluate_pause_gate(
    pause_event_seen: bool,
    pause_state: PauseState,
    frame_end_event_count: u32,
    frame_start_count: u32,
    frame_cb_count: u32,
) -> PauseGateDecision {
    let paused = pause_state == PauseState::Paused;
    let observed = pause_event_seen || paused;
    let frames_active = frame_end_event_count > 0 || frame_start_count > 0;

    if frames_active {
        return PauseGateDecision {
            block_walk: false,
            pause_event_seen: observed,
            pause_gate_active: false,
            reason: if observed {
                if frame_end_event_count > 0 {
                    "ignored_pause_event_frames_active"
                } else {
                    "ignored_pause_event_frame_start_active"
                }
            } else {
                "none"
            },
        };
    }

    if paused || pause_event_seen {
        return PauseGateDecision {
            block_walk: true,
            pause_event_seen: observed,
            pause_gate_active: true,
            reason: "explicit_pause_and_no_frame_end",
        };
    }

    if frame_cb_count == 0 {
        return PauseGateDecision {
            block_walk: true,
            pause_event_seen: false,
            pause_gate_active: true,
            reason: "menu_state",
        };
    }

    PauseGateDecision {
        block_walk: false,
        pause_event_seen: observed,
        pause_gate_active: false,
        reason: "recent_pause_event_but_frames_pending",
    }
}

impl WorldEventState {
    pub const fn new() -> Self {
        Self {
            pause_state: PauseState::Unknown,
            started_logged: false,
            pause_event_seen: false,
        }
    }

    pub fn is_paused(&self) -> bool {
        self.pause_state == PauseState::Paused
    }

    pub fn pause_event_seen(&self) -> bool {
        self.pause_event_seen
    }

    pub fn pause_state(&self) -> PauseState {
        self.pause_state
    }

    #[allow(dead_code)]
    pub fn on_event(&mut self, event_id: u32) -> WorldEventAction {
        match event_id {
            1 => {
                if self.started_logged {
                    WorldEventAction::Ignore
                } else {
                    self.started_logged = true;
                    WorldEventAction::LogStartedOnce
                }
            }
            4 => {
                self.pause_event_seen = true;
                if self.pause_state == PauseState::Paused {
                    WorldEventAction::Ignore
                } else {
                    self.pause_state = PauseState::Paused;
                    WorldEventAction::SoftReset {
                        reason: WORLD_RESET_REASON_PAUSED,
                    }
                }
            }
            5 => {
                if self.pause_state == PauseState::Running {
                    WorldEventAction::Ignore
                } else {
                    self.pause_state = PauseState::Running;
                    WorldEventAction::SoftReset {
                        reason: WORLD_RESET_REASON_UNPAUSED,
                    }
                }
            }
            _ => WorldEventAction::Ignore,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn route_scan_gate_blocks_before_started() {
        assert_eq!(
            evaluate_route_scan_gate(10_000_000, 0, 100),
            Err(RESOLVE_ROUTE_SCAN_WAITING_FOR_STABLE_WORLD)
        );
    }

    #[test]
    fn route_scan_gate_warmup() {
        assert_eq!(
            evaluate_route_scan_gate(5_000_000, 1_000_000, 100),
            Err(RESOLVE_ROUTE_SCAN_WARMUP)
        );
    }

    #[test]
    fn route_scan_gate_allows_after_warmup() {
        assert!(evaluate_route_scan_gate(12_000_000, 1_000_000, 100).is_ok());
    }

    #[test]
    fn resolve_status_names_include_safe_modes() {
        assert_eq!(
            resolve_status_str(RESOLVE_UNSAFE_ROUTE_SCAN_DISABLED),
            "unsafe_route_scan_disabled"
        );
        assert_eq!(
            resolve_status_str(RESOLVE_GPS_RESOLVED_ROUTE_SCAN_DISABLED),
            "gps_resolved_route_scan_disabled"
        );
        assert_eq!(
            resolve_status_str(RESOLVE_GPS_TABLE_ONLY_DONE),
            "gps_table_only_done"
        );
        assert_eq!(
            resolve_status_str(RESOLVE_GPS_TABLE_READ_FAILED),
            "gps_table_read_failed"
        );
        assert_eq!(
            resolve_status_str(RESOLVE_GPS_MANAGER_NOT_RESOLVED),
            "gps_manager_not_resolved"
        );
        assert_eq!(
            resolve_status_str(RESOLVE_GAME_CTRL_TABLE_ONLY_DONE),
            "game_ctrl_table_only_done"
        );
        assert_eq!(
            resolve_status_str(RESOLVE_GAME_CTRL_TABLE_READ_FAILED),
            "game_ctrl_table_read_failed"
        );
        assert_eq!(
            resolve_status_str(RESOLVE_ROUTE_CANDIDATE_TABLE_DONE),
            "route_candidate_table_done"
        );
        assert_eq!(
            resolve_status_str(RESOLVE_ROUTE_CANDIDATE_TABLE_READ_FAILED),
            "route_candidate_table_read_failed"
        );
    }

    #[test]
    fn debounce_gate_suppresses_rapid_fire() {
        let mut gate = DebounceGate::new(1_000_000);
        assert!(gate.try_fire(0));
        assert!(!gate.try_fire(500_000));
        assert_eq!(gate.suppressed(), 1);
        assert!(gate.try_fire(1_000_001));
    }

    #[test]
    fn world_reset_pack_roundtrip() {
        let packed = pack_world_reset(42, WORLD_RESET_REASON_UNPAUSED, 7);
        let (count, reason, suppressed) = unpack_world_reset(packed);
        assert_eq!(count, 42);
        assert_eq!(reason, WORLD_RESET_REASON_UNPAUSED);
        assert_eq!(suppressed, 7);
    }

    #[test]
    fn waypoint_frame_end_pack_roundtrip() {
        let packed = pack_waypoint_frame_end(120, 9999);
        let (wp, fe) = unpack_waypoint_frame_end(packed);
        assert_eq!(wp, 120);
        assert_eq!(fe, 9999);
    }

    #[test]
    fn effective_resolve_status_never_none_after_attempts() {
        assert_eq!(
            effective_resolve_status(RESOLVE_NONE, 1),
            RESOLVE_MODULE_SCAN_FAILED
        );
        assert_eq!(
            effective_resolve_status(RESOLVE_PATTERN_NOT_FOUND, 2),
            RESOLVE_PATTERN_NOT_FOUND
        );
        assert_eq!(effective_resolve_status(RESOLVE_NONE, 0), RESOLVE_NONE);
    }

    #[test]
    fn resolve_status_names_stable() {
        assert_eq!(resolve_status_str(RESOLVE_GPS_PTR_NULL), "gps_ptr_null");
        assert_eq!(
            resolve_status_str(RESOLVE_PATTERN_NOT_FOUND),
            "pattern_not_found"
        );
        assert_eq!(
            resolve_status_str(RESOLVE_ROUTE_CHAIN_ALL_FAILED),
            "route_chain_all_failed"
        );
        assert_eq!(
            resolve_status_str(RESOLVE_PAUSED_NO_ROUTE_WALK),
            "paused_no_route_walk"
        );
    }

    #[test]
    fn paused_state_is_detected() {
        let mut w = WorldEventState::new();
        assert!(!w.is_paused());
        assert_eq!(w.on_event(4), WorldEventAction::SoftReset {
            reason: WORLD_RESET_REASON_PAUSED
        });
        assert!(w.is_paused());
        assert!(w.pause_event_seen());
    }

    #[test]
    fn pause_gate_ignored_when_frame_end_active() {
        let gate = evaluate_pause_gate(true, PauseState::Paused, 2, 0, 100);
        assert!(!gate.block_walk);
        assert!(gate.pause_event_seen);
        assert!(!gate.pause_gate_active);
        assert_eq!(gate.reason, "ignored_pause_event_frames_active");
    }

    #[test]
    fn pause_gate_ignored_when_frame_start_active() {
        let gate = evaluate_pause_gate(true, PauseState::Paused, 0, 3, 100);
        assert!(!gate.block_walk);
        assert_eq!(gate.reason, "ignored_pause_event_frame_start_active");
    }

    #[test]
    fn pause_gate_blocks_without_frame_events() {
        let gate = evaluate_pause_gate(true, PauseState::Paused, 0, 0, 0);
        assert!(gate.block_walk);
        assert!(gate.pause_gate_active);
        assert_eq!(gate.reason, "explicit_pause_and_no_frame_end");
    }

    #[test]
    fn frame_start_fallback_when_no_frame_end() {
        assert!(should_frame_start_fallback(0, 0, 1_000_000));
    }

    #[test]
    fn frame_start_fallback_when_frame_end_stale() {
        assert!(should_frame_start_fallback(10, 1_000_000, 3_000_000));
    }

    #[test]
    fn no_frame_start_fallback_when_frame_end_recent() {
        assert!(!should_frame_start_fallback(10, 1_000_000, 1_500_000));
    }

    #[test]
    fn pause_gate_allows_walk_after_pause_event_when_frames_active() {
        let mut ws = WorldEventState::new();
        ws.on_event(4);
        let gate = evaluate_pause_gate(ws.pause_event_seen(), ws.pause_state(), 0, 1, 50);
        assert!(!gate.block_walk);
        assert_eq!(gate.reason, "ignored_pause_event_frame_start_active");
    }

    #[test]
    fn route_tick_meta_pack_roundtrip() {
        let packed = pack_route_tick_meta(42, 100);
        assert_eq!(unpack_route_tick_meta(packed), (42, 100));
    }

    #[test]
    fn rate_log_burst_then_silence() {
        let mut rl = RateLog::new();
        for _ in 0..RateLog::LOG_BURST {
            assert!(rl.should_log("a", 0, false));
            rl.record("a", 0);
        }
        assert!(!rl.should_log("a", RateLog::LOG_INTERVAL_US - 1, false));
        assert!(rl.should_log("a", RateLog::LOG_INTERVAL_US, false));
    }

    #[test]
    fn rate_log_status_change_resets_burst() {
        let mut rl = RateLog::new();
        for _ in 0..RateLog::LOG_BURST {
            rl.record("a", 0);
        }
        assert!(rl.should_log("b", 0, false));
    }

    #[test]
    fn milestone_log_first_five_then_1k() {
        let mut ml = MilestoneLog::new();
        for i in 1..=5 {
            assert!(ml.should_log(i));
        }
        assert!(!ml.should_log(6));
        assert!(!ml.should_log(999));
        assert!(ml.should_log(1000));
    }

    #[test]
    fn suppressed_summary_logs_at_interval() {
        let mut sl = SuppressedSummaryLog::new();
        sl.maybe_log("paused", 10, 0);
        assert_eq!(sl.last_total, 10);
        sl.maybe_log("paused", 20, 1_000_000);
        assert_eq!(sl.last_total, 20);
        sl.maybe_log("paused", 30, 2_000_000);
        assert_eq!(sl.last_total, 20);
        sl.maybe_log("paused", 30, 6_000_000);
        assert_eq!(sl.last_total, 30);
    }

    #[test]
    fn started_is_not_world_reset_reason() {
        assert_eq!(world_reset_reason_str(1), "none");
    }

    #[test]
    fn repeated_started_does_not_reset() {
        let mut ws = WorldEventState::new();
        assert_eq!(ws.on_event(1), WorldEventAction::LogStartedOnce);
        assert_eq!(ws.on_event(1), WorldEventAction::Ignore);
        assert_eq!(ws.on_event(1), WorldEventAction::Ignore);
    }

    #[test]
    fn frame_events_do_not_reset() {
        let mut ws = WorldEventState::new();
        assert_eq!(ws.on_event(2), WorldEventAction::Ignore);
        assert_eq!(ws.on_event(3), WorldEventAction::Ignore);
    }

    #[test]
    fn pause_edge_only_once() {
        let mut ws = WorldEventState::new();
        assert!(matches!(
            ws.on_event(4),
            WorldEventAction::SoftReset { reason: WORLD_RESET_REASON_PAUSED }
        ));
        assert_eq!(ws.on_event(4), WorldEventAction::Ignore);
    }

    #[test]
    fn unpaused_edge_after_pause() {
        let mut ws = WorldEventState::new();
        assert!(matches!(ws.on_event(4), WorldEventAction::SoftReset { .. }));
        assert!(matches!(
            ws.on_event(5),
            WorldEventAction::SoftReset { reason: WORLD_RESET_REASON_UNPAUSED }
        ));
        assert_eq!(ws.on_event(5), WorldEventAction::Ignore);
    }
}
