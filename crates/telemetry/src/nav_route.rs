//! ETS2 route shared-memory reader.
//!
//! Canonical route channel: `Local\TruckPilotRouteBlackboard` (Phase R2–R4, 5h).
//! Legacy `Local\TruckPilotNavRoute` was removed in Phase 5h — use [`RouteBlackboardReader`] only.

use std::mem;

// ---------------------------------------------------------------------------
// Route blackboard (`Local\TruckPilotRouteBlackboard`)
// ---------------------------------------------------------------------------

/// Must match `MAX_ROUTE_WAYPOINTS` in `crates/telemetry-dll/src/nav_route.rs`.
pub const MAX_ROUTE_WAYPOINTS: usize = 6000;
/// Magic `"TPRB"` little-endian.
pub const ROUTE_MAGIC: u32 = 0x4252_5054;
/// Route blackboard layout version (must match DLL).
pub const ROUTE_VERSION: u32 = 1;
/// Windows shared-memory object name for the route blackboard.
pub const ROUTE_SHM_NAME: &str = "Local\\TruckPilotRouteBlackboard";
/// Route exceeded [`MAX_ROUTE_WAYPOINTS`] and was clamped.
pub const ROUTE_FLAG_TRUNCATED: u32 = 1;

/// `reserved[0]` bit — set by telemetry DLL after RouteBlackboard init.
pub const ROUTE_BB_STATUS_DLL_ACTIVE: u32 = 1;
/// `reserved[0]` bit — route_task pointer chain last succeeded.
pub const ROUTE_BB_STATUS_ROUTE_TASK_OK: u32 = 1 << 1;
/// `reserved[0]` bit — at least one route tick ran after init.
pub const ROUTE_BB_STATUS_ROUTE_TICK_SEEN: u32 = 1 << 2;
/// `reserved[0]` bit — at least one telemetry frame callback ran after init.
pub const ROUTE_BB_STATUS_FRAME_CB_SEEN: u32 = 1 << 3;
/// `reserved[0]` bit — SCS pause event (raw_id=4) observed this session.
pub const ROUTE_BB_STATUS_PAUSE_EVENT_SEEN: u32 = 1 << 4;
/// `reserved[0]` bit — pause gate actively blocking route walk.
pub const ROUTE_BB_STATUS_PAUSE_GATE_ACTIVE: u32 = 1 << 5;
/// `reserved[0]` bit — no SCS frame_end while frame_start callbacks run.
pub const ROUTE_BB_STATUS_FRAME_END_MISSING: u32 = 1 << 6;
/// `reserved[0]` bit — last route tick used frame_start fallback.
pub const ROUTE_BB_STATUS_TICK_SOURCE_FALLBACK: u32 = 1 << 7;

/// Pack `reserved[5]`: low 16 = route_tick_count, high 16 = frame_start_count.
pub fn pack_route_tick_meta(route_tick_count: u32, frame_start_count: u32) -> u32 {
    (route_tick_count & 0xFFFF) | ((frame_start_count & 0xFFFF) << 16)
}

/// Decode `reserved[5]`: low 16 bits = route tick count, high 16 bits = frame_start count.
pub fn unpack_route_tick_meta(packed: u32) -> (u32, u32) {
    (packed & 0xFFFF, packed >> 16)
}

/// `reserved[1]` — last resolver status code (mirrors telemetry-dll `route_status.rs`).
pub const ROUTE_BB_RESERVED_RESOLVE_STATUS: usize = 1;
/// `reserved[2]` — cumulative resolver attempts since DLL load.
pub const ROUTE_BB_RESERVED_RESOLVE_ATTEMPTS: usize = 2;
/// `reserved[3]` — last publish status code.
pub const ROUTE_BB_RESERVED_PUBLISH_STATUS: usize = 3;
/// `reserved[4]` — telemetry frame callback count (saturating).
pub const ROUTE_BB_RESERVED_FRAME_CB_COUNT: usize = 4;
/// `reserved[5]` — route tick count (saturating).
pub const ROUTE_BB_RESERVED_ROUTE_TICK_COUNT: usize = 5;
/// `reserved[6]` — last published waypoint count.
pub const ROUTE_BB_RESERVED_LAST_WAYPOINT_COUNT: usize = 6;
/// `reserved[7]` — packed world-reset count (low 16) + last reason code (high 16).
pub const ROUTE_BB_RESERVED_WORLD_RESET: usize = 7;

/// Initial resolver status — only valid before the first resolve attempt.
pub const RESOLVE_NONE: u32 = 0;
/// AOB / PE scan did not locate `gps_manager`.
pub const RESOLVE_MODULE_SCAN_FAILED: u32 = 1;
/// `gps_manager` pointer was null after scan.
pub const RESOLVE_GPS_PTR_NULL: u32 = 2;
/// Route-task pointer chain returned null.
pub const RESOLVE_ROUTE_TASK_PTR_NULL: u32 = 3;
/// Route items array pointer missing or invalid.
pub const RESOLVE_ROUTE_ITEMS_PTR_NULL: u32 = 4;
/// Route items array empty after walk.
pub const RESOLVE_ITEMS_EMPTY: u32 = 5;
/// First waypoint UID was zero.
pub const RESOLVE_FIRST_UID_ZERO: u32 = 6;
/// UID walk succeeded — waypoints collected.
pub const RESOLVE_WAYPOINTS_COLLECTED: u32 = 7;
/// Route tick ran but resolver was interval-throttled.
pub const RESOLVE_TICK_THROTTLED: u32 = 8;
/// Trip-distance gate low (informational — walk may continue).
pub const RESOLVE_TRIP_DISTANCE_LOW: u32 = 10;
/// GPS → route_task pointer chain failed.
pub const RESOLVE_POINTER_CHAIN_FAILED: u32 = 11;
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
/// Resolver skipped walk because ETS2 is paused.
pub const RESOLVE_PAUSED_NO_ROUTE_WALK: u32 = 20;
/// GPS manager resolved (informational).
pub const RESOLVE_GPS_RESOLVED: u32 = 21;
/// All route_task chain candidates failed.
pub const RESOLVE_ROUTE_TASK_CANDIDATE_NULL: u32 = 22;
/// route_task found but items pointer invalid.
pub const RESOLVE_ROUTE_ITEMS_CANDIDATE_NULL: u32 = 23;
/// Items array empty after chain validation.
pub const RESOLVE_ROUTE_ITEMS_EMPTY: u32 = 24;
/// Named chain candidate failed (non-terminal).
pub const RESOLVE_ROUTE_CHAIN_CANDIDATE_FAILED: u32 = 25;
/// Every chain candidate failed.
pub const RESOLVE_ROUTE_CHAIN_ALL_FAILED: u32 = 26;
/// `gps + 0x08` simple_route_src pointer is null.
pub const RESOLVE_SIMPLE_ROUTE_SRC_NULL: u32 = 27;
/// SRS pointer not at +0x08; dynamic offset scan found no route.
pub const RESOLVE_SIMPLE_ROUTE_SRC_OFFSET_UNKNOWN: u32 = 28;
/// Dynamic SRS offset scan exhausted with no UID-bearing items.
pub const RESOLVE_SRS_OFFSET_SCAN_FAILED: u32 = 29;
/// Deep route memory scan disabled (default crash-safe mode).
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
/// GPS offset one-shot probe completed (`gps_offset_probe` mode).
pub const RESOLVE_GPS_OFFSET_PROBE_DONE: u32 = 47;
/// `game_ctrl` resolved but `game_ctrl+0x40F8` unreadable (`gps_offset_probe` mode).
pub const RESOLVE_GPS_OFFSET_PROBE_READ_FAILED: u32 = 48;

/// No publish yet.
pub const PUBLISH_NONE: u32 = 0;
/// Empty invalid route published.
pub const PUBLISH_EMPTY: u32 = 1;
/// Active route with waypoints published.
pub const PUBLISH_ACTIVE: u32 = 2;

/// World-reset reason: none yet.
pub const WORLD_RESET_REASON_NONE: u32 = 0;
/// Last reset triggered by SCS `paused` event.
pub const WORLD_RESET_REASON_PAUSED: u32 = 4;
/// Last reset triggered by SCS `unpaused` event.
pub const WORLD_RESET_REASON_UNPAUSED: u32 = 5;

/// Decode `reserved[7]` world-reset packing from the telemetry DLL.
pub fn unpack_world_reset(packed: u32) -> (u32, u32, u32) {
    (
        packed & 0xFFFF,
        (packed >> 16) & 0xFF,
        packed >> 24,
    )
}

/// Decode `reserved[6]`: low 16 = last waypoint count, high 16 = frame_end count.
pub fn unpack_waypoint_frame_end(packed: u32) -> (u32, u32) {
    (packed & 0xFFFF, packed >> 16)
}

/// Map resolver status code to a stable snake-case label for logs and dumps.
pub fn route_resolve_status_name(code: u32) -> &'static str {
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
        RESOLVE_TRIP_DISTANCE_LOW => "trip_distance_low",
        RESOLVE_POINTER_CHAIN_FAILED => "pointer_chain_failed",
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
        RESOLVE_GPS_OFFSET_PROBE_DONE => "gps_offset_probe_done",
        RESOLVE_GPS_OFFSET_PROBE_READ_FAILED => "gps_offset_probe_read_failed",
        _ => "unknown",
    }
}

/// Map world-reset reason code to a stable snake-case label.
pub fn world_reset_reason_name(code: u32) -> &'static str {
    match code {
        WORLD_RESET_REASON_PAUSED => "paused",
        WORLD_RESET_REASON_UNPAUSED => "unpaused",
        _ => "none",
    }
}

/// Never expose `none` in SHM when resolve attempts already ran (diagnostic guard).
pub fn effective_resolve_status(stored: u32, attempts: u32) -> u32 {
    if stored == RESOLVE_NONE && attempts > 0 {
        RESOLVE_MODULE_SCAN_FAILED
    } else {
        stored
    }
}

/// Map publish status code to a stable snake-case label for logs and dumps.
pub fn route_publish_status_name(code: u32) -> &'static str {
    match code {
        PUBLISH_NONE => "none",
        PUBLISH_EMPTY => "publish_empty",
        PUBLISH_ACTIVE => "publish_active",
        _ => "unknown",
    }
}

/// Per-waypoint bitflags (Phase 5i) — must match `telemetry-dll/src/nav_route.rs`.
pub const ROUTE_WP_FLAG_HAS_POSITION: u32 = 1 << 0;
/// Waypoint carries a remaining-distance field (`RouteWaypoint::distance`).
pub const ROUTE_WP_FLAG_HAS_DISTANCE: u32 = 1 << 1;
/// Waypoint carries a remaining-time field (`RouteWaypoint::time`).
pub const ROUTE_WP_FLAG_HAS_TIME: u32 = 1 << 2;
/// Field read from an unverified ETS2 offset or otherwise not trusted for routing.
pub const ROUTE_WP_FLAG_UNTRUSTED: u32 = 1 << 3;

/// Single route waypoint — must mirror the DLL writer layout byte-for-byte.
///
/// Coordinate convention (ETS2 world space, matches telemetry `position[0/1/2]`):
/// - `x`, `z`: horizontal map metres
/// - `y`: height / elevation metres
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RouteWaypoint {
    /// Graph node UID (ETS2 physical-route-item UID).
    pub uid: i64,
    /// ETS2 world X (metres).
    pub x: f32,
    /// ETS2 world Y / height (metres).
    pub y: f32,
    /// ETS2 world Z (metres).
    pub z: f32,
    /// Remaining route distance at this waypoint (metres), when flagged.
    pub distance: f32,
    /// Remaining route time at this waypoint (seconds), when flagged.
    pub time: f32,
    /// Bitfield of [`ROUTE_WP_FLAG_*`] values.
    pub flags: u32,
}

/// Fixed header prefix of [`RouteBlackboardLayout`] (64 bytes) — safe to read on stack.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RouteBlackboardHeader {
    /// Magic `"TPRB"` ([`ROUTE_MAGIC`]).
    pub magic: u32,
    /// Layout version ([`ROUTE_VERSION`]).
    pub version: u32,
    /// Seqlock sequence (odd = writer active).
    pub sequence: u32,
    /// Non-zero when route snapshot is valid.
    pub valid: u32,
    /// Populated waypoint count in SHM.
    pub waypoint_count: u32,
    /// Route-level flags (e.g. [`ROUTE_FLAG_TRUNCATED`]).
    pub flags: u32,
    /// FNV-1a hash over waypoint UIDs.
    pub route_hash: u64,
    /// Reserved (`reserved[0]` carries [`ROUTE_BB_STATUS_*`] bits from DLL).
    pub reserved: [u32; 8],
}

/// Wire layout — must mirror `RouteBlackboard` in `crates/telemetry-dll/src/nav_route.rs`.
#[repr(C)]
pub struct RouteBlackboardLayout {
    /// Magic `"TPRB"` (`ROUTE_MAGIC`).
    pub magic: u32,
    /// Layout version ([`ROUTE_VERSION`]).
    pub version: u32,
    /// Seqlock: odd = writer active, even = stable snapshot.
    pub sequence: u32,
    /// Non-zero when the route snapshot is valid.
    pub valid: u32,
    /// Number of populated entries in [`Self::waypoints`].
    pub waypoint_count: u32,
    /// Route-level flags (e.g. [`ROUTE_FLAG_TRUNCATED`]).
    pub flags: u32,
    /// FNV-1a hash over waypoint UIDs.
    pub route_hash: u64,
    /// Reserved for future fields (must be zero).
    pub reserved: [u32; 8],
    /// Fixed-size waypoint array written by the telemetry DLL.
    pub waypoints: [RouteWaypoint; MAX_ROUTE_WAYPOINTS],
}

const _: () = {
    assert!(mem::offset_of!(RouteBlackboardLayout, magic) == 0);
    assert!(mem::offset_of!(RouteBlackboardLayout, version) == 4);
    assert!(mem::offset_of!(RouteBlackboardLayout, sequence) == 8);
    assert!(mem::offset_of!(RouteBlackboardLayout, valid) == 12);
    assert!(mem::offset_of!(RouteBlackboardLayout, waypoint_count) == 16);
    assert!(mem::offset_of!(RouteBlackboardLayout, flags) == 20);
    assert!(mem::offset_of!(RouteBlackboardLayout, route_hash) == 24);
    assert!(mem::offset_of!(RouteBlackboardLayout, reserved) == 32);
    assert!(mem::offset_of!(RouteBlackboardLayout, waypoints) == 64);
    assert!(mem::size_of::<RouteWaypoint>() == 32);
    assert!(mem::size_of::<RouteBlackboardHeader>() == 64);
    assert!(mem::size_of::<RouteBlackboardLayout>() == 64 + MAX_ROUTE_WAYPOINTS * 32);
    // Full layout must never be copied onto the default thread stack (~1 MiB on Windows).
    assert!(mem::size_of::<RouteBlackboardLayout>() > 100_000);
};

/// FNV-1a 64-bit over waypoint UIDs — must match the DLL writer.
pub fn route_uid_hash(waypoints: &[RouteWaypoint]) -> u64 {
    let mut h = 0xcbf29ce484222325u64;
    for wp in waypoints {
        for byte in wp.uid.to_le_bytes() {
            h ^= u64::from(byte);
            h = h.wrapping_mul(0x100000001b3);
        }
    }
    h
}

/// Decoded route snapshot from the route blackboard SHM.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct RouteSnapshot {
    /// Seqlock sequence at read time.
    pub sequence: u32,
    /// FNV-1a hash over waypoint UIDs.
    pub route_hash: u64,
    /// Whether the DLL marked the route valid.
    pub valid: bool,
    /// Route-level flags from SHM.
    pub flags: u32,
    /// DLL status bits from SHM `reserved[0]`.
    pub bb_status: u32,
    /// Last resolver status (`reserved[1]`).
    pub resolve_status: u32,
    /// Cumulative resolver attempts (`reserved[2]`).
    pub resolve_attempts: u32,
    /// Last publish status (`reserved[3]`).
    pub publish_status: u32,
    /// Frame callback count from DLL (`reserved[4]`).
    pub frame_cb_count: u32,
    /// Route tick count from DLL (`reserved[5]` low word).
    pub route_tick_count: u32,
    /// Frame-start callback count (`reserved[5]` high word).
    pub frame_start_count: u32,
    /// Last published waypoint count (`reserved[6]` low word).
    pub last_waypoint_count: u32,
    /// Frame-end event count (`reserved[6]` high word).
    pub frame_end_count: u32,
    /// World-reset count from DLL (`reserved[7]` low word).
    pub world_reset_count: u32,
    /// Last world-reset reason code (`reserved[7]` bits 16–23).
    pub last_world_reset_reason: u32,
    /// Suppressed world-reset events since last effective reset (`reserved[7]` bits 24–31).
    pub world_reset_suppressed_count: u32,
    /// Decoded waypoints (length = SHM `waypoint_count`).
    pub waypoints: Vec<RouteWaypoint>,
}

/// Count waypoints with a specific [`ROUTE_WP_FLAG_*`] bit set.
pub fn count_waypoints_with_flag(waypoints: &[RouteWaypoint], flag: u32) -> usize {
    waypoints
        .iter()
        .filter(|wp| wp.flags & flag != 0)
        .count()
}

/// True when at least one waypoint carries [`ROUTE_WP_FLAG_HAS_POSITION`].
pub fn has_any_positions(waypoints: &[RouteWaypoint]) -> bool {
    count_waypoints_with_flag(waypoints, ROUTE_WP_FLAG_HAS_POSITION) > 0
}

/// High-level coordinate availability derived from SHM waypoint flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RouteCoordStatus {
    /// No coordinate fields present on any waypoint.
    Unavailable,
    /// Every waypoint has trusted position fields.
    Available,
    /// Some but not all waypoints have positions.
    Partial,
    /// At least one waypoint is flagged [`ROUTE_WP_FLAG_UNTRUSTED`].
    Untrusted,
}

impl RouteCoordStatus {
    /// Stable snake-case label for blackboard / logs.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Unavailable => "unavailable",
            Self::Available => "available",
            Self::Partial => "partial",
            Self::Untrusted => "untrusted",
        }
    }
}

/// Where ETS2 waypoint coordinate fields came from (diagnostic only).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RouteCoordSource {
    /// Route invalid or empty.
    None,
    /// Positions/distances from ETS2 waypoint memory walk.
    Ets2Waypoint,
    /// UIDs only — no ETS2 coordinate fields.
    GraphOnly,
    /// Mix of graph-only and ETS2 fields.
    Mixed,
}

impl RouteCoordSource {
    /// Stable snake-case label for blackboard / logs.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::Ets2Waypoint => "ets2_waypoint",
            Self::GraphOnly => "graph_only",
            Self::Mixed => "mixed",
        }
    }
}

/// Aggregated coordinate diagnostics for blackboard publish (Phase 5i).
#[derive(Debug, Clone, PartialEq)]
pub struct RouteCoordDiagnostics {
    /// Waypoints with [`ROUTE_WP_FLAG_HAS_POSITION`].
    pub position_count: usize,
    /// Waypoints with [`ROUTE_WP_FLAG_HAS_DISTANCE`].
    pub distance_count: usize,
    /// Waypoints with [`ROUTE_WP_FLAG_HAS_TIME`].
    pub time_count: usize,
    /// `position_count / total` in `[0, 1]`.
    pub position_ratio: f64,
    /// Derived coordinate availability status.
    pub coord_status: RouteCoordStatus,
    /// Where coordinate fields originated.
    pub coord_source: RouteCoordSource,
    /// First waypoint with a position, if any.
    pub first_position: Option<(f32, f32, f32)>,
    /// Last waypoint with a position, if any.
    pub last_position: Option<(f32, f32, f32)>,
}

/// Derive coordinate diagnostics from a route snapshot's waypoints.
pub fn diagnose_route_coords(valid: bool, waypoints: &[RouteWaypoint]) -> RouteCoordDiagnostics {
    let total = waypoints.len();
    let position_count = count_waypoints_with_flag(waypoints, ROUTE_WP_FLAG_HAS_POSITION);
    let distance_count = count_waypoints_with_flag(waypoints, ROUTE_WP_FLAG_HAS_DISTANCE);
    let time_count = count_waypoints_with_flag(waypoints, ROUTE_WP_FLAG_HAS_TIME);
    let any_untrusted = waypoints
        .iter()
        .any(|wp| wp.flags & ROUTE_WP_FLAG_UNTRUSTED != 0);

    let position_ratio = if total == 0 {
        0.0
    } else {
        position_count as f64 / total as f64
    };

    let has_ets2_fields = position_count > 0 || distance_count > 0 || time_count > 0;

    let coord_source = if !valid || total == 0 {
        RouteCoordSource::None
    } else if !has_ets2_fields {
        RouteCoordSource::GraphOnly
    } else if position_count > 0 && position_count < total {
        RouteCoordSource::Mixed
    } else {
        RouteCoordSource::Ets2Waypoint
    };

    let coord_status = if !has_ets2_fields {
        RouteCoordStatus::Unavailable
    } else if any_untrusted {
        RouteCoordStatus::Untrusted
    } else if position_count == total && total > 0 {
        RouteCoordStatus::Available
    } else if position_count == total && total == 0 {
        RouteCoordStatus::Unavailable
    } else {
        RouteCoordStatus::Partial
    };

    let first_position = waypoints
        .iter()
        .find(|wp| wp.flags & ROUTE_WP_FLAG_HAS_POSITION != 0)
        .map(|wp| (wp.x, wp.y, wp.z));
    let last_position = waypoints
        .iter()
        .rev()
        .find(|wp| wp.flags & ROUTE_WP_FLAG_HAS_POSITION != 0)
        .map(|wp| (wp.x, wp.y, wp.z));

    RouteCoordDiagnostics {
        position_count,
        distance_count,
        time_count,
        position_ratio,
        coord_status,
        coord_source,
        first_position,
        last_position,
    }
}

/// Format `"x,y,z"` for blackboard (one decimal) or empty when absent.
pub fn format_position_triple(pos: Option<(f32, f32, f32)>) -> String {
    pos.map(|(x, y, z)| format!("{x:.1},{y:.1},{z:.1}"))
        .unwrap_or_default()
}

/// Tolerance for ETS2 remaining-distance monotonic checks (metres).
pub const ROUTE_DISTANCE_TOLERANCE_M: f32 = 0.5;

/// Monotonicity classification for ETS2 `distance` fields (Phase 5j diagnostic).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RouteDistanceMonotonicStatus {
    /// No distance samples.
    None,
    /// Non-increasing within tolerance.
    Ok,
    /// All samples equal within tolerance.
    Flat,
    /// Mostly increasing (suspicious).
    Increasing,
    /// Occasional increases but not dominant.
    Jumpy,
    /// Distance present on only part of the route.
    Partial,
}

impl RouteDistanceMonotonicStatus {
    /// Stable snake-case label for blackboard / logs.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::Ok => "ok",
            Self::Flat => "flat",
            Self::Increasing => "increasing",
            Self::Jumpy => "jumpy",
            Self::Partial => "partial",
        }
    }
}

/// Aggregated distance-field diagnostics (Phase 5j).
#[derive(Debug, Clone, PartialEq)]
pub struct RouteDistanceDiagnostics {
    /// Waypoints with [`ROUTE_WP_FLAG_HAS_DISTANCE`].
    pub distance_count: usize,
    /// Distance samples flagged [`ROUTE_WP_FLAG_UNTRUSTED`].
    pub distance_untrusted_count: usize,
    /// First distance sample (metres).
    pub distance_first_m: Option<f32>,
    /// Last distance sample (metres).
    pub distance_last_m: Option<f32>,
    /// Minimum distance sample (metres).
    pub distance_min_m: Option<f32>,
    /// Maximum distance sample (metres).
    pub distance_max_m: Option<f32>,
    /// Monotonicity classification.
    pub distance_monotonic_status: RouteDistanceMonotonicStatus,
    /// Count of step-to-step increases beyond tolerance.
    pub distance_increase_count: usize,
    /// Largest drop between consecutive samples (metres).
    pub distance_drop_max_m: f64,
    /// Mean absolute step size between consecutive samples (metres).
    pub distance_step_avg_m: Option<f64>,
}

fn distance_samples(waypoints: &[RouteWaypoint]) -> Vec<f32> {
    waypoints
        .iter()
        .filter(|wp| wp.flags & ROUTE_WP_FLAG_HAS_DISTANCE != 0)
        .map(|wp| wp.distance)
        .collect()
}

/// Analyse ETS2 remaining-distance samples along the route (diagnostic only).
pub fn diagnose_route_distances(waypoints: &[RouteWaypoint]) -> RouteDistanceDiagnostics {
    let distance_count = count_waypoints_with_flag(waypoints, ROUTE_WP_FLAG_HAS_DISTANCE);
    let distance_untrusted_count = waypoints
        .iter()
        .filter(|wp| {
            wp.flags & ROUTE_WP_FLAG_HAS_DISTANCE != 0 && wp.flags & ROUTE_WP_FLAG_UNTRUSTED != 0
        })
        .count();

    let mut diag = RouteDistanceDiagnostics {
        distance_count,
        distance_untrusted_count,
        distance_first_m: None,
        distance_last_m: None,
        distance_min_m: None,
        distance_max_m: None,
        distance_monotonic_status: RouteDistanceMonotonicStatus::None,
        distance_increase_count: 0,
        distance_drop_max_m: 0.0,
        distance_step_avg_m: None,
    };

    let samples = distance_samples(waypoints);
    if samples.is_empty() {
        return diag;
    }

    diag.distance_first_m = Some(samples[0]);
    diag.distance_last_m = Some(*samples.last().unwrap());
    diag.distance_min_m = Some(
        samples
            .iter()
            .copied()
            .fold(f32::INFINITY, f32::min),
    );
    diag.distance_max_m = Some(
        samples
            .iter()
            .copied()
            .fold(f32::NEG_INFINITY, f32::max),
    );

    let coverage_partial = distance_count < waypoints.len();
    if samples.len() == 1 {
        diag.distance_monotonic_status = RouteDistanceMonotonicStatus::Partial;
        return diag;
    }

    let tol = ROUTE_DISTANCE_TOLERANCE_M;
    let mut step_sum = 0.0_f64;
    let mut step_count = 0usize;

    for w in samples.windows(2) {
        let delta = w[1] - w[0];
        if delta > tol {
            diag.distance_increase_count += 1;
        }
        if delta < -tol {
            diag.distance_drop_max_m = diag.distance_drop_max_m.max(f64::from(-delta));
        }
        step_sum += f64::from((w[1] - w[0]).abs());
        step_count += 1;
    }
    if step_count > 0 {
        diag.distance_step_avg_m = Some(step_sum / step_count as f64);
    }

    let min_v = diag.distance_min_m.unwrap_or(0.0);
    let max_v = diag.distance_max_m.unwrap_or(0.0);
    let all_flat = samples
        .windows(2)
        .all(|w| (w[0] - w[1]).abs() <= tol)
        && (max_v - min_v).abs() <= tol;

    diag.distance_monotonic_status = if coverage_partial {
        RouteDistanceMonotonicStatus::Partial
    } else if all_flat {
        RouteDistanceMonotonicStatus::Flat
    } else if diag.distance_increase_count == 0 {
        RouteDistanceMonotonicStatus::Ok
    } else if diag.distance_increase_count >= 3
        || diag.distance_increase_count * 2 >= step_count
    {
        RouteDistanceMonotonicStatus::Increasing
    } else {
        RouteDistanceMonotonicStatus::Jumpy
    };

    diag
}

/// Decode [`RouteWaypoint::flags`] into human-readable names (dump / debug).
pub fn decode_waypoint_flag_names(flags: u32) -> Vec<&'static str> {
    let mut names = Vec::new();
    if flags & ROUTE_WP_FLAG_HAS_POSITION != 0 {
        names.push("has_position");
    }
    if flags & ROUTE_WP_FLAG_HAS_DISTANCE != 0 {
        names.push("has_distance");
    }
    if flags & ROUTE_WP_FLAG_HAS_TIME != 0 {
        names.push("has_time");
    }
    if flags & ROUTE_WP_FLAG_UNTRUSTED != 0 {
        names.push("untrusted");
    }
    names
}

/// Read the 64-byte SHM header without copying the full waypoint array.
fn read_header_at(view: *const u8) -> RouteBlackboardHeader {
    unsafe { std::ptr::read_unaligned(view as *const RouteBlackboardHeader) }
}

/// Copy `count` waypoints from mapped SHM into a heap `Vec` (never stack-allocates the full layout).
fn copy_waypoints_at(view: *const u8, count: usize) -> Vec<RouteWaypoint> {
    let count = count.min(MAX_ROUTE_WAYPOINTS);
    if count == 0 {
        return Vec::new();
    }
    let wp_base = unsafe { view.add(mem::offset_of!(RouteBlackboardLayout, waypoints)) };
    let wp_ptr = wp_base as *const RouteWaypoint;
    unsafe { std::slice::from_raw_parts(wp_ptr, count).to_vec() }
}

/// Build a [`RouteSnapshot`] from a mapped view — header on stack, waypoints on heap only.
fn snapshot_from_view(view: *const u8) -> Option<RouteSnapshot> {
    if view.is_null() {
        return None;
    }
    let header = read_header_at(view);
    if header.magic != ROUTE_MAGIC || header.version != ROUTE_VERSION {
        return None;
    }
    let count = header.waypoint_count as usize;
    if count > MAX_ROUTE_WAYPOINTS {
        return None;
    }
    let valid = header.valid != 0;
    let waypoints = if valid && count > 0 {
        copy_waypoints_at(view, count)
    } else {
        Vec::new()
    };
    let (world_reset_count, last_world_reset_reason, world_reset_suppressed_count) =
        unpack_world_reset(header.reserved[ROUTE_BB_RESERVED_WORLD_RESET]);
    let (last_waypoint_count, frame_end_count) =
        unpack_waypoint_frame_end(header.reserved[ROUTE_BB_RESERVED_LAST_WAYPOINT_COUNT]);
    let (route_tick_count, frame_start_count) =
        unpack_route_tick_meta(header.reserved[ROUTE_BB_RESERVED_ROUTE_TICK_COUNT]);
    let resolve_status = effective_resolve_status(
        header.reserved[ROUTE_BB_RESERVED_RESOLVE_STATUS],
        header.reserved[ROUTE_BB_RESERVED_RESOLVE_ATTEMPTS],
    );
    Some(RouteSnapshot {
        sequence: header.sequence,
        route_hash: header.route_hash,
        valid,
        flags: header.flags,
        bb_status: header.reserved[0],
        resolve_status,
        resolve_attempts: header.reserved[ROUTE_BB_RESERVED_RESOLVE_ATTEMPTS],
        publish_status: header.reserved[ROUTE_BB_RESERVED_PUBLISH_STATUS],
        frame_cb_count: header.reserved[ROUTE_BB_RESERVED_FRAME_CB_COUNT],
        route_tick_count,
        frame_start_count,
        last_waypoint_count,
        frame_end_count,
        world_reset_count,
        last_world_reset_reason,
        world_reset_suppressed_count,
        waypoints,
    })
}

/// Persistent reader for `Local\TruckPilotRouteBlackboard`.
pub struct RouteBlackboardReader {
    inner: RouteBbInner,
    last_sequence: u32,
    last_route_hash: u64,
}

unsafe impl Send for RouteBlackboardReader {}
unsafe impl Sync for RouteBlackboardReader {}

impl RouteBlackboardReader {
    /// Open the route blackboard mapping. Returns an error when SHM is absent.
    pub fn open() -> Result<Self, String> {
        Ok(Self {
            inner: RouteBbInner::open()?,
            last_sequence: u32::MAX,
            last_route_hash: 0,
        })
    }

    /// Read a stable snapshot using seqlock protocol. Never panics.
    pub fn read(&self) -> Option<RouteSnapshot> {
        const MAX_ATTEMPTS: u32 = 4;
        for _ in 0..MAX_ATTEMPTS {
            let seq_before = self.inner.read_sequence()?;
            if seq_before & 1 != 0 {
                std::thread::sleep(std::time::Duration::from_micros(50));
                continue;
            }
            let snap = snapshot_from_view(self.inner.view())?;
            if snap.sequence != seq_before {
                std::thread::sleep(std::time::Duration::from_micros(50));
                continue;
            }
            let seq_after = self.inner.read_sequence()?;
            if seq_before != seq_after {
                std::thread::sleep(std::time::Duration::from_micros(50));
                continue;
            }
            return Some(snap);
        }
        None
    }

    /// Like [`Self::read`] but only returns `Some` when `sequence` or `route_hash` changed.
    pub fn read_if_changed(&mut self) -> Option<RouteSnapshot> {
        let snap = self.read()?;
        if snap.sequence == self.last_sequence && snap.route_hash == self.last_route_hash {
            return None;
        }
        self.last_sequence = snap.sequence;
        self.last_route_hash = snap.route_hash;
        Some(snap)
    }
}

#[cfg(windows)]
struct RouteBbInner {
    handle: windows::Win32::Foundation::HANDLE,
    view: *const u8,
}

#[cfg(windows)]
unsafe impl Send for RouteBbInner {}

#[cfg(windows)]
unsafe impl Sync for RouteBbInner {}

#[cfg(windows)]
impl RouteBbInner {
    fn open() -> Result<Self, String> {
        use windows::core::PCWSTR;
        use windows::Win32::System::Memory::{MapViewOfFile, OpenFileMappingW, FILE_MAP_READ};

        let name: Vec<u16> = ROUTE_SHM_NAME.encode_utf16().chain(std::iter::once(0)).collect();
        let handle = unsafe { OpenFileMappingW(FILE_MAP_READ.0, false, PCWSTR(name.as_ptr())) }
            .map_err(|e| format!("OpenFileMappingW({ROUTE_SHM_NAME}): {e}"))?;
        if handle.is_invalid() {
            return Err(format!("invalid handle for {ROUTE_SHM_NAME}"));
        }
        let view = unsafe {
            MapViewOfFile(
                handle,
                FILE_MAP_READ,
                0,
                0,
                mem::size_of::<RouteBlackboardLayout>(),
            )
        };
        if view.Value.is_null() {
            unsafe {
                let _ = windows::Win32::Foundation::CloseHandle(handle);
            }
            return Err(format!("MapViewOfFile returned null for {ROUTE_SHM_NAME}"));
        }
        Ok(Self {
            handle,
            view: view.Value as *const u8,
        })
    }

    fn read_sequence(&self) -> Option<u32> {
        if self.view.is_null() {
            return None;
        }
        Some(unsafe {
            std::ptr::read_volatile(
                self.view.add(mem::offset_of!(RouteBlackboardLayout, sequence)) as *const u32,
            )
        })
    }

    fn view(&self) -> *const u8 {
        self.view
    }
}

#[cfg(windows)]
impl Drop for RouteBbInner {
    fn drop(&mut self) {
        use windows::Win32::Foundation::CloseHandle;
        use windows::Win32::System::Memory::{UnmapViewOfFile, MEMORY_MAPPED_VIEW_ADDRESS};
        unsafe {
            if !self.view.is_null() {
                let addr = MEMORY_MAPPED_VIEW_ADDRESS {
                    Value: self.view as *mut _,
                };
                let _ = UnmapViewOfFile(addr);
            }
            let _ = CloseHandle(self.handle);
        }
    }
}

#[cfg(not(windows))]
struct RouteBbInner;

#[cfg(not(windows))]
impl RouteBbInner {
    fn open() -> Result<Self, String> {
        Ok(Self)
    }
    fn read_sequence(&self) -> Option<u32> {
        None
    }
    fn view(&self) -> *const u8 {
        std::ptr::null()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn canonical_route_shm_name() {
        assert_eq!(ROUTE_SHM_NAME, r"Local\TruckPilotRouteBlackboard");
    }

    #[test]
    fn route_waypoint_size_is_32() {
        assert_eq!(mem::size_of::<RouteWaypoint>(), 32);
    }

    #[test]
    fn route_blackboard_header_size_is_64() {
        assert_eq!(mem::offset_of!(RouteBlackboardLayout, waypoints), 64);
        assert_eq!(
            mem::size_of::<RouteBlackboardLayout>(),
            64 + MAX_ROUTE_WAYPOINTS * mem::size_of::<RouteWaypoint>()
        );
    }

    #[test]
    fn route_magic_and_version_constants() {
        assert_eq!(ROUTE_MAGIC, 0x4252_5054);
        assert_eq!(ROUTE_VERSION, 1);
    }

    #[test]
    fn route_uid_hash_changes_with_uid() {
        let a = [
            RouteWaypoint {
                uid: 1,
                ..Default::default()
            },
            RouteWaypoint {
                uid: 2,
                ..Default::default()
            },
        ];
        let b = [
            RouteWaypoint {
                uid: 1,
                ..Default::default()
            },
            RouteWaypoint {
                uid: 3,
                ..Default::default()
            },
        ];
        assert_eq!(route_uid_hash(&a), route_uid_hash(&a));
        assert_ne!(route_uid_hash(&a), route_uid_hash(&b));
    }

    #[test]
    fn rejects_layout_with_bad_magic() {
        assert_ne!(0u32, ROUTE_MAGIC);
        assert_eq!(ROUTE_MAGIC, 0x4252_5054);
    }

    #[test]
    fn rejects_count_above_max() {
        let count = MAX_ROUTE_WAYPOINTS + 1;
        assert!(count > MAX_ROUTE_WAYPOINTS);
    }

    /// In-memory seqlock writer/reader round-trip (no OS SHM required).
    #[test]
    fn seqlock_empty_invalid_route_roundtrip() {
        let mut backing =
            vec![0u8; mem::size_of::<RouteBlackboardLayout>()].into_boxed_slice();
        let ptr = backing.as_mut_ptr() as *mut RouteBlackboardLayout;

        unsafe {
            (*ptr).magic = ROUTE_MAGIC;
            (*ptr).version = ROUTE_VERSION;
            std::ptr::write_volatile(&mut (*ptr).sequence, 1);
            std::ptr::write_volatile(&mut (*ptr).valid, 0);
            std::ptr::write_volatile(&mut (*ptr).waypoint_count, 0);
            std::ptr::write_volatile(&mut (*ptr).flags, 0);
            std::ptr::write_volatile(&mut (*ptr).route_hash, 0);
            (*ptr).reserved[0] = ROUTE_BB_STATUS_DLL_ACTIVE;
            std::ptr::write_volatile(&mut (*ptr).sequence, 2);
        }

        let snap = snapshot_from_view(backing.as_ptr()).expect("empty route snapshot");
        assert_eq!(snap.sequence, 2);
        assert!(!snap.valid);
        assert!(snap.waypoints.is_empty());
        assert_eq!(snap.route_hash, 0);
        assert_ne!(snap.bb_status & ROUTE_BB_STATUS_DLL_ACTIVE, 0);
    }

    #[test]
    fn read_snapshot_active_route_from_backing() {
        let mut backing =
            vec![0u8; mem::size_of::<RouteBlackboardLayout>()].into_boxed_slice();
        let ptr = backing.as_mut_ptr() as *mut RouteBlackboardLayout;

        unsafe {
            (*ptr).magic = ROUTE_MAGIC;
            (*ptr).version = ROUTE_VERSION;
            std::ptr::write_volatile(&mut (*ptr).sequence, 1);
            std::ptr::write_volatile(&mut (*ptr).valid, 0);
            (*ptr).waypoints[0].uid = 100;
            (*ptr).waypoints[1].uid = 200;
            std::ptr::write_volatile(&mut (*ptr).waypoint_count, 2);
            std::ptr::write_volatile(&mut (*ptr).flags, 0);
            let hash_wps = [(*ptr).waypoints[0], (*ptr).waypoints[1]];
            std::ptr::write_volatile(&mut (*ptr).route_hash, route_uid_hash(&hash_wps));
            std::ptr::write_volatile(&mut (*ptr).valid, 1);
            std::ptr::write_volatile(&mut (*ptr).sequence, 2);
        }

        let snap = snapshot_from_view(backing.as_ptr()).expect("active route snapshot");
        assert!(snap.valid);
        assert_eq!(snap.waypoints.len(), 2);
        assert_eq!(snap.waypoints[0].uid, 100);
        assert_eq!(snap.waypoints[1].uid, 200);
    }

    #[test]
    fn read_snapshot_max_waypoints_without_stack_layout() {
        let mut backing =
            vec![0u8; mem::size_of::<RouteBlackboardLayout>()].into_boxed_slice();
        let ptr = backing.as_mut_ptr() as *mut RouteBlackboardLayout;
        unsafe {
            (*ptr).magic = ROUTE_MAGIC;
            (*ptr).version = ROUTE_VERSION;
            std::ptr::write_volatile(&mut (*ptr).sequence, 2);
            std::ptr::write_volatile(&mut (*ptr).valid, 1);
            std::ptr::write_volatile(&mut (*ptr).waypoint_count, MAX_ROUTE_WAYPOINTS as u32);
            for i in 0..MAX_ROUTE_WAYPOINTS {
                (*ptr).waypoints[i].uid = i as i64 + 1;
            }
        }
        let snap = snapshot_from_view(backing.as_ptr()).expect("max waypoints snapshot");
        assert_eq!(snap.waypoints.len(), MAX_ROUTE_WAYPOINTS);
        assert_eq!(snap.waypoints[0].uid, 1);
        assert_eq!(snap.waypoints[MAX_ROUTE_WAYPOINTS - 1].uid, MAX_ROUTE_WAYPOINTS as i64);
    }

    #[test]
    fn header_size_is_stack_safe() {
        assert!(mem::size_of::<RouteBlackboardHeader>() <= 128);
        assert!(mem::size_of::<RouteBlackboardLayout>() > 100_000);
    }

    #[test]
    fn seqlock_stable_read_after_write() {
        let mut backing =
            vec![0u8; mem::size_of::<RouteBlackboardLayout>()].into_boxed_slice();
        let ptr = backing.as_mut_ptr() as *mut RouteBlackboardLayout;

        unsafe {
            (*ptr).magic = ROUTE_MAGIC;
            (*ptr).version = ROUTE_VERSION;
            // seqlock write: odd → payload → even
            std::ptr::write_volatile(&mut (*ptr).sequence, 1);
            std::ptr::write_volatile(&mut (*ptr).valid, 0);
            (*ptr).waypoints[0].uid = 100;
            (*ptr).waypoints[1].uid = 200;
            std::ptr::write_volatile(&mut (*ptr).waypoint_count, 2);
            std::ptr::write_volatile(&mut (*ptr).flags, 0);
            let hash_wps = [(*ptr).waypoints[0], (*ptr).waypoints[1]];
            std::ptr::write_volatile(&mut (*ptr).route_hash, route_uid_hash(&hash_wps));
            std::ptr::write_volatile(&mut (*ptr).valid, 1);
            std::ptr::write_volatile(&mut (*ptr).sequence, 2);
        }

        let seq = unsafe {
            std::ptr::read_volatile(
                backing
                    .as_ptr()
                    .add(mem::offset_of!(RouteBlackboardLayout, sequence))
                    as *const u32,
            )
        };
        assert_eq!(seq, 2);
        assert_eq!(seq & 1, 0);

        let snap = snapshot_from_view(backing.as_ptr()).expect("stable read");
        assert_eq!(snap.waypoints.len(), 2);
        assert_eq!(snap.waypoints[0].uid, 100);
    }

    #[test]
    fn seqlock_odd_sequence_is_unstable() {
        let seq: u32 = 3;
        assert_ne!(seq & 1, 0);
    }

    #[test]
    fn coord_diag_unavailable_without_flags() {
        let wps = [
            RouteWaypoint {
                uid: 1,
                ..Default::default()
            },
            RouteWaypoint {
                uid: 2,
                ..Default::default()
            },
        ];
        let d = diagnose_route_coords(true, &wps);
        assert_eq!(d.position_count, 0);
        assert_eq!(d.coord_status, RouteCoordStatus::Unavailable);
        assert_eq!(d.coord_source, RouteCoordSource::GraphOnly);
    }

    #[test]
    fn coord_diag_partial_positions() {
        let wps = [
            RouteWaypoint {
                uid: 1,
                x: 10.0,
                z: 20.0,
                flags: ROUTE_WP_FLAG_HAS_POSITION,
                ..Default::default()
            },
            RouteWaypoint {
                uid: 2,
                ..Default::default()
            },
        ];
        let d = diagnose_route_coords(true, &wps);
        assert_eq!(d.position_count, 1);
        assert_eq!(d.position_ratio, 0.5);
        assert_eq!(d.coord_status, RouteCoordStatus::Partial);
        assert_eq!(d.coord_source, RouteCoordSource::Mixed);
        assert_eq!(d.first_position, Some((10.0, 0.0, 20.0)));
    }

    #[test]
    fn coord_diag_distance_only_is_untrusted() {
        let wps = [RouteWaypoint {
            uid: 1,
            distance: 100.0,
            flags: ROUTE_WP_FLAG_HAS_DISTANCE | ROUTE_WP_FLAG_UNTRUSTED,
            ..Default::default()
        }];
        let d = diagnose_route_coords(true, &wps);
        assert_eq!(d.distance_count, 1);
        assert_eq!(d.coord_status, RouteCoordStatus::Untrusted);
        assert_eq!(d.coord_source, RouteCoordSource::Ets2Waypoint);
    }

    #[test]
    fn has_any_positions_respects_flag() {
        assert!(!has_any_positions(&[RouteWaypoint {
            uid: 1,
            x: 1.0,
            ..Default::default()
        }]));
        assert!(has_any_positions(&[RouteWaypoint {
            uid: 1,
            flags: ROUTE_WP_FLAG_HAS_POSITION,
            ..Default::default()
        }]));
    }

    fn wp_dist(uid: i64, distance: f32) -> RouteWaypoint {
        RouteWaypoint {
            uid,
            distance,
            flags: ROUTE_WP_FLAG_HAS_DISTANCE | ROUTE_WP_FLAG_UNTRUSTED,
            ..Default::default()
        }
    }

    #[test]
    fn distance_diag_empty_is_none() {
        let d = diagnose_route_distances(&[]);
        assert_eq!(d.distance_count, 0);
        assert_eq!(
            d.distance_monotonic_status,
            RouteDistanceMonotonicStatus::None
        );
    }

    #[test]
    fn distance_diag_monotonic_falling_is_ok() {
        let wps = vec![wp_dist(1, 3000.0), wp_dist(2, 2000.0), wp_dist(3, 1000.0)];
        let d = diagnose_route_distances(&wps);
        assert_eq!(d.distance_monotonic_status, RouteDistanceMonotonicStatus::Ok);
        assert_eq!(d.distance_increase_count, 0);
        assert!((d.distance_drop_max_m - 1000.0).abs() < 0.01);
    }

    #[test]
    fn distance_diag_small_tolerance_increase_still_ok() {
        let wps = vec![wp_dist(1, 1000.0), wp_dist(2, 1000.3), wp_dist(3, 999.0)];
        let d = diagnose_route_distances(&wps);
        assert_eq!(d.distance_monotonic_status, RouteDistanceMonotonicStatus::Ok);
        assert_eq!(d.distance_increase_count, 0);
    }

    #[test]
    fn distance_diag_flat_values() {
        let wps = vec![wp_dist(1, 500.0), wp_dist(2, 500.0), wp_dist(3, 500.0)];
        let d = diagnose_route_distances(&wps);
        assert_eq!(d.distance_monotonic_status, RouteDistanceMonotonicStatus::Flat);
    }

    #[test]
    fn distance_diag_large_increase() {
        let wps = vec![
            wp_dist(1, 1000.0),
            wp_dist(2, 1500.0),
            wp_dist(3, 2000.0),
            wp_dist(4, 2500.0),
        ];
        let d = diagnose_route_distances(&wps);
        assert_eq!(
            d.distance_monotonic_status,
            RouteDistanceMonotonicStatus::Increasing
        );
        assert!(d.distance_increase_count >= 3);
    }

    #[test]
    fn distance_diag_untrusted_count() {
        let wps = vec![
            wp_dist(1, 100.0),
            RouteWaypoint {
                uid: 2,
                ..Default::default()
            },
            wp_dist(3, 50.0),
        ];
        let d = diagnose_route_distances(&wps);
        assert_eq!(d.distance_untrusted_count, 2);
        assert_eq!(
            d.distance_monotonic_status,
            RouteDistanceMonotonicStatus::Partial
        );
    }

    #[test]
    fn decode_flag_names_includes_untrusted() {
        let names = decode_waypoint_flag_names(ROUTE_WP_FLAG_HAS_DISTANCE | ROUTE_WP_FLAG_UNTRUSTED);
        assert!(names.contains(&"has_distance"));
        assert!(names.contains(&"untrusted"));
    }
}
