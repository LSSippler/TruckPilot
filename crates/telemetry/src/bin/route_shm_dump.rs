//! Read `Local\TruckPilotRouteBlackboard` and dump route diagnostics (Phase 5j).
//!
//! ```text
//! cargo run -p truckpilot-telemetry --bin route-shm-dump -- --once --perf
//! cargo run -p truckpilot-telemetry --bin route-shm-dump -- --json route.json
//! cargo run -p truckpilot-telemetry --bin route-shm-dump -- --csv route.csv
//! ```

use std::fs;
use std::path::Path;

use truckpilot_telemetry::dll_perf::{format_perf_lines, format_perf_unavailable, DllPerfReader};
use truckpilot_telemetry::nav_route::{
    route_publish_status_name, route_resolve_status_name, world_reset_reason_name,
    RouteBlackboardReader, RouteSnapshot, RESOLVE_NONE,
    ROUTE_BB_STATUS_DLL_ACTIVE, ROUTE_BB_STATUS_FRAME_CB_SEEN, ROUTE_BB_STATUS_FRAME_END_MISSING,
    ROUTE_BB_STATUS_PAUSE_EVENT_SEEN, ROUTE_BB_STATUS_PAUSE_GATE_ACTIVE,
    ROUTE_BB_STATUS_ROUTE_TASK_OK, ROUTE_BB_STATUS_ROUTE_TICK_SEEN,
    ROUTE_BB_STATUS_TICK_SOURCE_FALLBACK,
};
use truckpilot_telemetry::route_distance_log::snapshot_to_waypoint_records;

fn has_flag(flag: &str) -> bool {
    std::env::args().any(|a| a == flag)
}

fn arg_value(flag: &str) -> Option<String> {
    let args: Vec<String> = std::env::args().collect();
    args.windows(2)
        .find(|w| w[0] == flag)
        .map(|w| w[1].clone())
}

#[derive(Debug, Clone, PartialEq)]
pub enum RouteShmProbe {
    Missing,
    ReadFailed,
    EmptyRoute { snap: RouteSnapshot },
    ActiveRoute { snap: RouteSnapshot },
}

/// Classify RouteBlackboard SHM for diagnostics (unit-tested).
pub fn probe_route_shm(reader: &RouteBlackboardReader) -> RouteShmProbe {
    match reader.read() {
        Some(snap) if snap.valid && !snap.waypoints.is_empty() => {
            RouteShmProbe::ActiveRoute { snap }
        }
        Some(snap) => RouteShmProbe::EmptyRoute { snap },
        None => RouteShmProbe::ReadFailed,
    }
}

pub fn format_missing_shm_message(err: &str) -> String {
    format!(
        "RouteBlackboard SHM missing.\n\
         Open error: {err}\n\
         Possible causes:\n\
         1. ETS2 is not running\n\
         2. truckpilot_telemetry.dll is not installed\n\
         3. ETS2 did not load the DLL\n\
         4. DLL loaded but RouteBlackboard was not created\n\
         Check: Documents\\Euro Truck Simulator 2\\game.log.txt\n\
         Also check: <ETS2>\\bin\\win_x64\\plugins\\truckpilot_telemetry.log"
    )
}

pub fn format_route_diag_lines(snap: &RouteSnapshot) -> Vec<String> {
    let dll_active = snap.bb_status & ROUTE_BB_STATUS_DLL_ACTIVE != 0;
    let route_task = snap.bb_status & ROUTE_BB_STATUS_ROUTE_TASK_OK != 0;
    let route_tick_seen = snap.bb_status & ROUTE_BB_STATUS_ROUTE_TICK_SEEN != 0;
    let frame_cb_seen = snap.bb_status & ROUTE_BB_STATUS_FRAME_CB_SEEN != 0;
    let pause_event_seen = snap.bb_status & ROUTE_BB_STATUS_PAUSE_EVENT_SEEN != 0;
    let pause_gate_active = snap.bb_status & ROUTE_BB_STATUS_PAUSE_GATE_ACTIVE != 0;
    let frame_end_missing = snap.bb_status & ROUTE_BB_STATUS_FRAME_END_MISSING != 0;
    let tick_source_fallback = snap.bb_status & ROUTE_BB_STATUS_TICK_SOURCE_FALLBACK != 0;
    let route_tick_source = if tick_source_fallback {
        "frame_start_fallback"
    } else if snap.frame_end_count > 0 {
        "frame_end"
    } else {
        "none"
    };
    let status_name = route_resolve_status_name(snap.resolve_status);
    let mut lines = vec![
        format!("dll_active={dll_active}"),
        format!("route_task_ok={route_task}"),
        format!("route_tick_seen={route_tick_seen}"),
        format!("frame_callback_seen={frame_cb_seen}"),
        format!("pause_event_seen={pause_event_seen}"),
        format!("pause_gate_active={pause_gate_active}"),
        format!("frame_start_count={}", snap.frame_start_count),
        format!("frame_end_count={}", snap.frame_end_count),
        format!("frame_end_missing={frame_end_missing}"),
        format!("route_tick_source={route_tick_source}"),
        format!("route_resolve_attempts={}", snap.resolve_attempts),
        format!("route_resolve_status={status_name}"),
        format!(
            "last_publish_status={}",
            route_publish_status_name(snap.publish_status)
        ),
        format!("frame_cb_count={}", snap.frame_cb_count),
        format!("route_tick_count={}", snap.route_tick_count),
        format!("world_reset_count={}", snap.world_reset_count),
        format!(
            "last_world_reset_reason={}",
            world_reset_reason_name(snap.last_world_reset_reason)
        ),
        format!(
            "world_reset_suppressed_count={}",
            snap.world_reset_suppressed_count
        ),
    ];
    if is_module_scan_failure(snap.resolve_status) {
        lines.push(
            "hint=nav resolver module/pattern scan failed; check sidecar module scan diagnostics"
                .into(),
        );
    }
    if is_chain_failure(snap.resolve_status) {
        lines.push(chain_failure_hint(snap.resolve_status));
    }
    if snap.resolve_status
        == truckpilot_telemetry::nav_route::RESOLVE_GPS_RESOLVED_ROUTE_SCAN_DISABLED
        || snap.resolve_status
            == truckpilot_telemetry::nav_route::RESOLVE_UNSAFE_ROUTE_SCAN_DISABLED
    {
        lines.push(
            "hint=deep route memory scan disabled for ETS2 stability".into(),
        );
    }
    if snap.resolve_status == truckpilot_telemetry::nav_route::RESOLVE_ROUTE_SCAN_WARMUP {
        lines.push("hint=route scan warmup — waiting after session start".into());
    }
    if snap.resolve_status
        == truckpilot_telemetry::nav_route::RESOLVE_ROUTE_SCAN_WAITING_FOR_STABLE_WORLD
    {
        lines.push("hint=route scan waiting for stable world / started event".into());
    }
    if snap.resolve_status == truckpilot_telemetry::nav_route::RESOLVE_GPS_TABLE_ONLY_DONE {
        lines.push(
            "hint=gps pointer table logged; no route chain scan performed".into(),
        );
    }
    if snap.resolve_status
        == truckpilot_telemetry::nav_route::RESOLVE_GAME_CTRL_TABLE_ONLY_DONE
    {
        lines.push(
            "hint=game_ctrl pointer table logged; no route chain scan performed".into(),
        );
    }
    if snap.resolve_status
        == truckpilot_telemetry::nav_route::RESOLVE_ROUTE_CANDIDATE_TABLE_DONE
    {
        lines.push(
            "hint=route candidate tables logged; no route chain scan performed".into(),
        );
    }
    if snap.resolve_status == truckpilot_telemetry::nav_route::RESOLVE_GPS_TABLE_READ_FAILED {
        lines.push(
            "hint=gps manager resolved but safe_read failed for gps table".into(),
        );
    }
    if snap.resolve_status
        == truckpilot_telemetry::nav_route::RESOLVE_GAME_CTRL_TABLE_READ_FAILED
    {
        lines.push(
            "hint=game_ctrl resolved but safe_read failed for game_ctrl table".into(),
        );
    }
    if snap.resolve_status
        == truckpilot_telemetry::nav_route::RESOLVE_ROUTE_CANDIDATE_TABLE_READ_FAILED
    {
        lines.push(
            "hint=game_ctrl resolved but safe_read failed for route candidate tables".into(),
        );
    }
    if snap.resolve_status == truckpilot_telemetry::nav_route::RESOLVE_GPS_OFFSET_PROBE_DONE {
        lines.push(
            "hint=gps offset probe logged one pointer-sized value; no chain scan performed"
                .into(),
        );
    }
    if snap.resolve_status
        == truckpilot_telemetry::nav_route::RESOLVE_GPS_OFFSET_PROBE_READ_FAILED
    {
        lines.push(
            "hint=gps offset probe read failed; no chain scan performed".into(),
        );
    }
    if snap.resolve_status
        == truckpilot_telemetry::nav_route::RESOLVE_GPS_MANAGER_NOT_RESOLVED
    {
        lines.push("hint=gps manager could not be resolved in gps_table mode".into());
    }
    if snap.resolve_status
        == truckpilot_telemetry::nav_route::RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE
    {
        lines.push("hint=route resolver disabled in crash-safe mode".into());
    }
    if snap.resolve_status
        == truckpilot_telemetry::nav_route::RESOLVE_ROUTE_RESOLVER_PARKED
    {
        lines.push(
            "hint=route resolver parked; waiting for session/event/manual trigger".into(),
        );
    }
    if snap.resolve_status
        == truckpilot_telemetry::nav_route::RESOLVE_ROUTE_RESOLVER_BACKOFF
    {
        lines.push("hint=route resolver in exponential backoff".into());
    }
    if snap.resolve_status
        == truckpilot_telemetry::nav_route::RESOLVE_ROUTE_RESOLVER_CACHE_HIT
    {
        lines.push("hint=game_ctrl cache hit; no pattern scan performed".into());
    }
    if snap.resolve_status
        == truckpilot_telemetry::nav_route::RESOLVE_ROUTE_RESOLVER_SCAN_LIMITED
    {
        lines.push("hint=pattern scan retry limit reached".into());
    }
    if snap.resolve_status
        == truckpilot_telemetry::nav_route::RESOLVE_PAUSED_NO_ROUTE_WALK
    {
        lines.push("hint=ETS2 appears paused/menu; route walk skipped".into());
    } else if pause_event_seen && snap.frame_end_count > 0 {
        lines.push(
            "hint=paused event observed, but frames are active; route walk is still attempted"
                .into(),
        );
    } else if frame_end_missing && snap.route_tick_count > 0 {
        lines.push(
            "hint=frame_end missing; route resolver uses frame_start fallback tick".into(),
        );
    }
    if snap.last_world_reset_reason
        == truckpilot_telemetry::nav_route::WORLD_RESET_REASON_PAUSED
        && snap.resolve_status != truckpilot_telemetry::nav_route::RESOLVE_PAUSED_NO_ROUTE_WALK
        && snap.resolve_status
            != truckpilot_telemetry::nav_route::RESOLVE_WAYPOINTS_COLLECTED
        && !pause_event_seen
    {
        lines.push(
            "hint=last world reset was paused; ensure game is unpaused before judging chain failures"
                .into(),
        );
    }
    lines
}

fn is_chain_failure(status: u32) -> bool {
    use truckpilot_telemetry::nav_route::{
        RESOLVE_POINTER_CHAIN_FAILED, RESOLVE_ROUTE_CHAIN_ALL_FAILED,
        RESOLVE_ROUTE_CHAIN_CANDIDATE_FAILED, RESOLVE_ROUTE_ITEMS_CANDIDATE_NULL,
        RESOLVE_ROUTE_ITEMS_EMPTY, RESOLVE_ROUTE_TASK_CANDIDATE_NULL,
        RESOLVE_SIMPLE_ROUTE_SRC_NULL, RESOLVE_SIMPLE_ROUTE_SRC_OFFSET_UNKNOWN,
        RESOLVE_SRS_OFFSET_SCAN_FAILED,
    };
    matches!(
        status,
        RESOLVE_POINTER_CHAIN_FAILED
            | RESOLVE_ROUTE_TASK_CANDIDATE_NULL
            | RESOLVE_ROUTE_ITEMS_CANDIDATE_NULL
            | RESOLVE_ROUTE_ITEMS_EMPTY
            | RESOLVE_ROUTE_CHAIN_CANDIDATE_FAILED
            | RESOLVE_ROUTE_CHAIN_ALL_FAILED
            | RESOLVE_SIMPLE_ROUTE_SRC_NULL
            | RESOLVE_SIMPLE_ROUTE_SRC_OFFSET_UNKNOWN
            | RESOLVE_SRS_OFFSET_SCAN_FAILED
    )
}

fn chain_failure_hint(status: u32) -> String {
    use truckpilot_telemetry::nav_route::{
        RESOLVE_SIMPLE_ROUTE_SRC_OFFSET_UNKNOWN, RESOLVE_SRS_OFFSET_SCAN_FAILED,
    };
    match status {
        RESOLVE_SIMPLE_ROUTE_SRC_OFFSET_UNKNOWN => {
            "hint=gps manager resolved, but gps+0x08 is null; check gps pointer table / srs offset scan"
                .into()
        }
        RESOLVE_SRS_OFFSET_SCAN_FAILED => {
            "hint=gps+0x08 null and srs offset scan found no UIDs; check direct route_task/items scan in sidecar"
                .into()
        }
        _ => {
            "hint=gps manager resolved, but route_task/items offsets are invalid or game is paused; check sidecar route chain candidates"
                .into()
        }
    }
}

fn is_module_scan_failure(status: u32) -> bool {
    use truckpilot_telemetry::nav_route::{
        RESOLVE_GAME_CTRL_NULL, RESOLVE_MODULE_NOT_FOUND, RESOLVE_MODULE_PE_PARSE_FAILED,
        RESOLVE_MODULE_SCAN_FAILED, RESOLVE_PATTERN_MATCH_INVALID, RESOLVE_PATTERN_MULTIPLE_MATCHES,
        RESOLVE_PATTERN_NOT_FOUND,
    };
    matches!(
        status,
        RESOLVE_MODULE_SCAN_FAILED
            | RESOLVE_MODULE_NOT_FOUND
            | RESOLVE_MODULE_PE_PARSE_FAILED
            | RESOLVE_PATTERN_NOT_FOUND
            | RESOLVE_PATTERN_MULTIPLE_MATCHES
            | RESOLVE_PATTERN_MATCH_INVALID
            | RESOLVE_GAME_CTRL_NULL
    )
}

pub fn format_diag_warnings(snap: &RouteSnapshot) -> Vec<String> {
    let mut warnings = Vec::new();
    if snap.resolve_attempts > 0 && snap.resolve_status == RESOLVE_NONE {
        warnings.push(
            "WARN: resolver attempts > 0 but status is none; diagnostics inconsistent".into(),
        );
    }
    warnings
}

pub fn format_empty_route_message(snap: &RouteSnapshot) -> String {
    let mut lines = vec![
        "RouteBlackboard available but no route:".into(),
        format!("valid={}", snap.valid),
        format!("waypoint_count={}", snap.waypoints.len()),
        format!("route_hash={:#x}", snap.route_hash),
        format!("sequence={}", snap.sequence),
    ];
    lines.extend(format_route_diag_lines(snap));
    lines.extend(format_diag_warnings(snap));
    lines.join("\n")
}

pub fn format_active_route_summary(snap: &RouteSnapshot) -> String {
    let first_uid = snap.waypoints.first().map(|wp| wp.uid);
    let last_uid = snap.waypoints.last().map(|wp| wp.uid);
    let mut lines = vec![
        "RouteBlackboard active route:".into(),
        format!("valid={}", snap.valid),
        format!("waypoint_count={}", snap.waypoints.len()),
        format!("route_hash={:#x}", snap.route_hash),
        format!("sequence={}", snap.sequence),
        format!(
            "first_uid={}",
            first_uid
                .map(|u| u.to_string())
                .unwrap_or_else(|| "-".into())
        ),
        format!(
            "last_uid={}",
            last_uid.map(|u| u.to_string()).unwrap_or_else(|| "-".into())
        ),
    ];
    lines.extend(format_route_diag_lines(snap));
    lines.extend(format_diag_warnings(snap));
    lines.join("\n")
}

#[derive(Debug, Clone, serde::Serialize, PartialEq)]
pub struct DumpWaypoint {
    pub index: usize,
    pub uid: i64,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub distance: f32,
    pub time: f32,
    pub flags: u32,
    pub flag_names: Vec<String>,
}

#[derive(Debug, Clone, serde::Serialize, PartialEq)]
pub struct RouteDump {
    pub sequence: u32,
    pub route_hash: u64,
    pub valid: bool,
    pub bb_status: u32,
    pub resolve_status: u32,
    pub resolve_attempts: u32,
    pub publish_status: u32,
    pub frame_cb_count: u32,
    pub route_tick_count: u32,
    pub last_waypoint_count: u32,
    pub waypoint_count: usize,
    pub waypoints: Vec<DumpWaypoint>,
}

/// Build a serializable dump struct from a route snapshot.
pub fn snapshot_to_dump(snap: &RouteSnapshot) -> RouteDump {
    let waypoints = snapshot_to_waypoint_records(0, snap)
        .into_iter()
        .map(|rec| DumpWaypoint {
            index: rec.index,
            uid: rec.uid,
            x: rec.x,
            y: rec.y,
            z: rec.z,
            distance: rec.distance,
            time: rec.time,
            flags: rec.flags,
            flag_names: rec.flag_names,
        })
        .collect();
    RouteDump {
        sequence: snap.sequence,
        route_hash: snap.route_hash,
        valid: snap.valid,
        bb_status: snap.bb_status,
        resolve_status: snap.resolve_status,
        resolve_attempts: snap.resolve_attempts,
        publish_status: snap.publish_status,
        frame_cb_count: snap.frame_cb_count,
        route_tick_count: snap.route_tick_count,
        last_waypoint_count: snap.last_waypoint_count,
        waypoint_count: snap.waypoints.len(),
        waypoints,
    }
}

/// Pretty JSON representation of a route snapshot.
pub fn format_dump_json(snap: &RouteSnapshot) -> Result<String, serde_json::Error> {
    serde_json::to_string_pretty(&snapshot_to_dump(snap))
}

/// CSV export (header + one row per waypoint).
pub fn format_dump_csv(snap: &RouteSnapshot) -> String {
    let mut out = String::from("index,uid,x,y,z,distance,time,flags,flag_names\n");
    for wp in snapshot_to_dump(snap).waypoints {
        let names = wp.flag_names.join("|");
        out.push_str(&format!(
            "{},{},{},{},{},{},{},{},\"{names}\"\n",
            wp.index, wp.uid, wp.x, wp.y, wp.z, wp.distance, wp.time, wp.flags
        ));
    }
    out
}

fn main() {
    let once = has_flag("--once");
    let perf = has_flag("--perf");
    let json_path = arg_value("--json");
    let csv_path = arg_value("--csv");

    let reader = match RouteBlackboardReader::open() {
        Ok(r) => r,
        Err(e) => {
            eprintln!("{}", format_missing_shm_message(&e));
            std::process::exit(1);
        }
    };

    let probe = probe_route_shm(&reader);
    let snap = match &probe {
        RouteShmProbe::EmptyRoute { snap } | RouteShmProbe::ActiveRoute { snap } => snap.clone(),
        RouteShmProbe::ReadFailed => {
            eprintln!(
                "RouteBlackboard SHM mapped but snapshot read failed (torn seqlock or bad magic)."
            );
            std::process::exit(1);
        }
        RouteShmProbe::Missing => unreachable!(),
    };

    match &probe {
        RouteShmProbe::EmptyRoute { .. } => println!("{}", format_empty_route_message(&snap)),
        RouteShmProbe::ActiveRoute { .. } => {
            println!("{}", format_active_route_summary(&snap));
        }
        _ => {}
    }

    if perf {
        match DllPerfReader::open() {
            Ok(reader) => match reader.read() {
                Some(perf_snap) => {
                    for line in format_perf_lines(&perf_snap) {
                        println!("{line}");
                    }
                }
                None => println!("{}", format_perf_unavailable("DllPerf SHM mapped but snapshot invalid")),
            },
            Err(e) => println!("{}", format_perf_unavailable(&e)),
        }
    }

    if let Some(path) = json_path {
        let json = format_dump_json(&snap).expect("serialize route dump");
        fs::write(&path, &json).unwrap_or_else(|e| {
            eprintln!("ERROR: write {}: {e}", Path::new(&path).display());
            std::process::exit(1);
        });
        println!("wrote JSON → {path} ({} waypoints)", snap.waypoints.len());
    } else if let Some(path) = csv_path {
        let csv = format_dump_csv(&snap);
        fs::write(&path, &csv).unwrap_or_else(|e| {
            eprintln!("ERROR: write {}: {e}", Path::new(&path).display());
            std::process::exit(1);
        });
        println!("wrote CSV → {path} ({} waypoints)", snap.waypoints.len());
    } else if !matches!(probe, RouteShmProbe::EmptyRoute { .. }) {
        let json = format_dump_json(&snap).expect("serialize route dump");
        println!("{json}");
    }

    if once {
        return;
    }

    eprintln!("Note: use --once for one-shot read (default prints once and exits).");
}

#[cfg(test)]
mod tests {
    use super::*;
    use truckpilot_telemetry::dll_perf::{DllPerfSnapshot, DLL_PERF_MAGIC, DLL_PERF_VERSION};
    use truckpilot_telemetry::nav_route::{
        RouteWaypoint, ROUTE_BB_RESERVED_PUBLISH_STATUS, ROUTE_BB_RESERVED_RESOLVE_ATTEMPTS,
        ROUTE_BB_RESERVED_RESOLVE_STATUS, ROUTE_BB_RESERVED_ROUTE_TICK_COUNT,
        ROUTE_WP_FLAG_HAS_DISTANCE, ROUTE_WP_FLAG_HAS_POSITION, ROUTE_WP_FLAG_UNTRUSTED,
        PUBLISH_EMPTY,
    };

    fn sample_snap() -> RouteSnapshot {
        RouteSnapshot {
            sequence: 4,
            route_hash: 0xABCD,
            valid: true,
            flags: 0,
            bb_status: ROUTE_BB_STATUS_DLL_ACTIVE
                | ROUTE_BB_STATUS_ROUTE_TASK_OK
                | ROUTE_BB_STATUS_ROUTE_TICK_SEEN
                | ROUTE_BB_STATUS_FRAME_CB_SEEN,
            resolve_status: truckpilot_telemetry::nav_route::RESOLVE_MODULE_SCAN_FAILED,
            resolve_attempts: 42,
            publish_status: PUBLISH_EMPTY,
            frame_cb_count: 100,
            route_tick_count: 50,
            frame_start_count: 50,
            last_waypoint_count: 0,
            frame_end_count: 48,
            world_reset_count: 0,
            last_world_reset_reason: 0,
            world_reset_suppressed_count: 0,
            waypoints: vec![
                RouteWaypoint {
                    uid: 100,
                    distance: 3000.0,
                    flags: ROUTE_WP_FLAG_HAS_DISTANCE | ROUTE_WP_FLAG_UNTRUSTED,
                    ..Default::default()
                },
                RouteWaypoint {
                    uid: 200,
                    x: 1.0,
                    z: 2.0,
                    flags: ROUTE_WP_FLAG_HAS_POSITION,
                    ..Default::default()
                },
            ],
        }
    }

    #[test]
    fn missing_shm_message_lists_causes() {
        let msg = format_missing_shm_message("OpenFileMappingW failed");
        assert!(msg.contains("RouteBlackboard SHM missing"));
        assert!(msg.contains("game.log.txt"));
    }

    #[test]
    fn empty_route_message_shows_world_reset_diag() {
        let snap = RouteSnapshot {
            sequence: 2,
            route_hash: 0,
            valid: false,
            flags: 0,
            bb_status: ROUTE_BB_STATUS_DLL_ACTIVE | ROUTE_BB_STATUS_ROUTE_TICK_SEEN,
            resolve_status: truckpilot_telemetry::nav_route::RESOLVE_MODULE_SCAN_FAILED,
            resolve_attempts: 17,
            publish_status: PUBLISH_EMPTY,
            frame_cb_count: 200,
            route_tick_count: 180,
            frame_start_count: 180,
            last_waypoint_count: 0,
            frame_end_count: 175,
            world_reset_count: 3,
            last_world_reset_reason: 5,
            world_reset_suppressed_count: 0,
            waypoints: vec![],
        };
        let msg = format_empty_route_message(&snap);
        assert!(msg.contains("world_reset_count=3"));
        assert!(msg.contains("last_world_reset_reason=unpaused"));
        assert!(format_diag_warnings(&snap).is_empty());
    }

    #[test]
    fn diag_warning_when_status_none_with_attempts() {
        let snap = RouteSnapshot {
            resolve_attempts: 2,
            resolve_status: RESOLVE_NONE,
            ..Default::default()
        };
        let warnings = format_diag_warnings(&snap);
        assert_eq!(warnings.len(), 1);
        assert!(warnings[0].contains("inconsistent"));
    }

    #[test]
    fn empty_route_message_shows_resolver_diag() {
        let snap = RouteSnapshot {
            sequence: 2,
            route_hash: 0,
            valid: false,
            flags: 0,
            bb_status: ROUTE_BB_STATUS_DLL_ACTIVE | ROUTE_BB_STATUS_ROUTE_TICK_SEEN,
            resolve_status: truckpilot_telemetry::nav_route::RESOLVE_MODULE_SCAN_FAILED,
            resolve_attempts: 17,
            publish_status: PUBLISH_EMPTY,
            frame_cb_count: 200,
            route_tick_count: 180,
            frame_start_count: 175,
            last_waypoint_count: 0,
            frame_end_count: 175,
            world_reset_count: 0,
            last_world_reset_reason: 0,
            world_reset_suppressed_count: 0,
            waypoints: vec![],
        };
        let msg = format_empty_route_message(&snap);
        assert!(msg.contains("valid=false"));
        assert!(msg.contains("waypoint_count=0"));
        assert!(msg.contains("route_hash=0x0"));
        assert!(msg.contains("dll_active=true"));
        assert!(msg.contains("route_tick_seen=true"));
        assert!(msg.contains("route_resolve_attempts=17"));
        assert!(msg.contains("route_resolve_status=module_scan_failed"));
        assert!(msg.contains("last_publish_status=publish_empty"));
        assert!(msg.contains("frame_end_count=175"));
        assert!(msg.contains("hint=nav resolver module/pattern scan failed"));
    }

    #[test]
    fn pattern_not_found_status_shows_hint() {
        let snap = RouteSnapshot {
            resolve_status: truckpilot_telemetry::nav_route::RESOLVE_PATTERN_NOT_FOUND,
            resolve_attempts: 1,
            ..Default::default()
        };
        let lines = format_route_diag_lines(&snap);
        assert!(lines.iter().any(|l| l.contains("pattern_not_found")));
        assert!(lines.iter().any(|l| l.contains("hint=nav resolver")));
    }

    #[test]
    fn chain_all_failed_status_shows_hint() {
        let snap = RouteSnapshot {
            resolve_status: truckpilot_telemetry::nav_route::RESOLVE_ROUTE_CHAIN_ALL_FAILED,
            resolve_attempts: 3,
            ..Default::default()
        };
        let lines = format_route_diag_lines(&snap);
        assert!(lines.iter().any(|l| l.contains("route_chain_all_failed")));
        assert!(lines.iter().any(|l| l.contains("route chain candidates")));
    }

    #[test]
    fn unsafe_scan_disabled_status_shows_hint() {
        let snap = RouteSnapshot {
            resolve_status:
                truckpilot_telemetry::nav_route::RESOLVE_GPS_RESOLVED_ROUTE_SCAN_DISABLED,
            resolve_attempts: 2,
            ..Default::default()
        };
        let lines = format_route_diag_lines(&snap);
        assert!(lines.iter().any(|l| l.contains("gps_resolved_route_scan_disabled")));
        assert!(lines.iter().any(|l| l.contains("disabled for ETS2 stability")));
    }

    #[test]
    fn disabled_safe_mode_status_shows_hint() {
        let snap = RouteSnapshot {
            resolve_status:
                truckpilot_telemetry::nav_route::RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE,
            ..Default::default()
        };
        let lines = format_route_diag_lines(&snap);
        assert!(lines
            .iter()
            .any(|l| l.contains("route_resolver_disabled_safe_mode")));
        assert!(lines
            .iter()
            .any(|l| l.contains("hint=route resolver disabled in crash-safe mode")));
    }

    #[test]
    fn gps_table_only_done_status_shows_hint() {
        let snap = RouteSnapshot {
            resolve_status: truckpilot_telemetry::nav_route::RESOLVE_GPS_TABLE_ONLY_DONE,
            resolve_attempts: 2,
            ..Default::default()
        };
        let lines = format_route_diag_lines(&snap);
        assert!(lines.iter().any(|l| l.contains("gps_table_only_done")));
        assert!(lines
            .iter()
            .any(|l| l.contains("gps pointer table logged; no route chain scan performed")));
    }

    #[test]
    fn route_candidate_table_done_status_shows_hint() {
        let snap = RouteSnapshot {
            resolve_status:
                truckpilot_telemetry::nav_route::RESOLVE_ROUTE_CANDIDATE_TABLE_DONE,
            resolve_attempts: 2,
            ..Default::default()
        };
        let lines = format_route_diag_lines(&snap);
        assert!(lines.iter().any(|l| l.contains("route_candidate_table_done")));
        assert!(lines.iter().any(|l| {
            l.contains("route candidate tables logged; no route chain scan performed")
        }));
    }

    #[test]
    fn game_ctrl_table_only_done_status_shows_hint() {
        let snap = RouteSnapshot {
            resolve_status:
                truckpilot_telemetry::nav_route::RESOLVE_GAME_CTRL_TABLE_ONLY_DONE,
            resolve_attempts: 2,
            ..Default::default()
        };
        let lines = format_route_diag_lines(&snap);
        assert!(lines.iter().any(|l| l.contains("game_ctrl_table_only_done")));
        assert!(lines.iter().any(|l| {
            l.contains("game_ctrl pointer table logged; no route chain scan performed")
        }));
    }

    #[test]
    fn gps_offset_probe_done_status_shows_hint() {
        let snap = RouteSnapshot {
            resolve_status: truckpilot_telemetry::nav_route::RESOLVE_GPS_OFFSET_PROBE_DONE,
            resolve_attempts: 2,
            ..Default::default()
        };
        let lines = format_route_diag_lines(&snap);
        assert!(lines.iter().any(|l| l.contains("gps_offset_probe_done")));
        assert!(lines.iter().any(|l| {
            l.contains("gps offset probe logged one pointer-sized value; no chain scan performed")
        }));
    }

    #[test]
    fn gps_offset_probe_read_failed_status_shows_hint() {
        let snap = RouteSnapshot {
            resolve_status:
                truckpilot_telemetry::nav_route::RESOLVE_GPS_OFFSET_PROBE_READ_FAILED,
            resolve_attempts: 2,
            ..Default::default()
        };
        let lines = format_route_diag_lines(&snap);
        assert!(lines.iter().any(|l| l.contains("gps_offset_probe_read_failed")));
        assert!(lines
            .iter()
            .any(|l| l.contains("gps offset probe read failed; no chain scan performed")));
    }

    #[test]
    fn srs_offset_unknown_status_shows_hint() {
        let snap = RouteSnapshot {
            resolve_status:
                truckpilot_telemetry::nav_route::RESOLVE_SIMPLE_ROUTE_SRC_OFFSET_UNKNOWN,
            resolve_attempts: 2,
            ..Default::default()
        };
        let lines = format_route_diag_lines(&snap);
        assert!(lines.iter().any(|l| l.contains("simple_route_src_offset_unknown")));
        assert!(lines.iter().any(|l| l.contains("gps+0x08 is null")));
    }

    #[test]
    fn paused_status_shows_unpause_hint() {
        let snap = RouteSnapshot {
            resolve_status: truckpilot_telemetry::nav_route::RESOLVE_PAUSED_NO_ROUTE_WALK,
            bb_status: ROUTE_BB_STATUS_PAUSE_EVENT_SEEN | ROUTE_BB_STATUS_PAUSE_GATE_ACTIVE,
            resolve_attempts: 1,
            ..Default::default()
        };
        let lines = format_route_diag_lines(&snap);
        assert!(lines.iter().any(|l| l.contains("paused_no_route_walk")));
        assert!(lines.iter().any(|l| l.contains("route walk skipped")));
    }

    #[test]
    fn pause_event_with_active_frames_shows_proceed_hint() {
        let snap = RouteSnapshot {
            resolve_status: truckpilot_telemetry::nav_route::RESOLVE_ROUTE_CHAIN_ALL_FAILED,
            bb_status: ROUTE_BB_STATUS_PAUSE_EVENT_SEEN,
            frame_end_count: 5,
            resolve_attempts: 2,
            ..Default::default()
        };
        let lines = format_route_diag_lines(&snap);
        assert!(lines.iter().any(|l| l.contains("pause_event_seen=true")));
        assert!(lines.iter().any(|l| l.contains("route walk is still attempted")));
        assert!(!lines.iter().any(|l| l.contains("route walk skipped")));
    }

    #[test]
    fn frame_end_missing_shows_fallback_hint() {
        let snap = RouteSnapshot {
            bb_status: ROUTE_BB_STATUS_FRAME_END_MISSING | ROUTE_BB_STATUS_TICK_SOURCE_FALLBACK,
            frame_end_count: 0,
            frame_start_count: 10,
            route_tick_count: 5,
            resolve_attempts: 2,
            resolve_status: truckpilot_telemetry::nav_route::RESOLVE_ROUTE_CHAIN_ALL_FAILED,
            ..Default::default()
        };
        let lines = format_route_diag_lines(&snap);
        assert!(lines.iter().any(|l| l.contains("frame_end_missing=true")));
        assert!(lines.iter().any(|l| l.contains("route_tick_source=frame_start_fallback")));
        assert!(lines.iter().any(|l| l.contains("frame_start fallback")));
    }

    #[test]
    fn dump_json_contains_metadata_and_flags() {
        let json = format_dump_json(&sample_snap()).unwrap();
        assert!(json.contains("\"sequence\": 4"));
        assert!(json.contains("\"has_distance\""));
        assert!(json.contains("\"bb_status\""));
        assert!(json.contains("\"resolve_attempts\": 42"));
    }

    #[test]
    fn active_route_summary_includes_first_and_last_uid() {
        let snap = sample_snap();
        let msg = format_active_route_summary(&snap);
        assert!(msg.contains("first_uid=100"));
        assert!(msg.contains("last_uid=200"));
        assert!(msg.contains("waypoint_count=2"));
        assert!(msg.contains("route_tick_seen=true"));
    }

    #[test]
    fn dump_csv_has_header_and_rows() {
        let csv = format_dump_csv(&sample_snap());
        assert!(csv.starts_with("index,uid,x,y,z,distance,time,flags,flag_names\n"));
        assert!(csv.contains("100"));
        assert!(csv.contains("has_distance|untrusted"));
    }

    #[test]
    fn reserved_field_indices_match_dll_layout() {
        assert_eq!(ROUTE_BB_RESERVED_RESOLVE_STATUS, 1);
        assert_eq!(ROUTE_BB_RESERVED_RESOLVE_ATTEMPTS, 2);
        assert_eq!(ROUTE_BB_RESERVED_PUBLISH_STATUS, 3);
        assert_eq!(ROUTE_BB_RESERVED_ROUTE_TICK_COUNT, 5);
    }

    #[test]
    fn route_shm_dump_renders_perf_snapshot_lines() {
        let snap = DllPerfSnapshot {
            magic: DLL_PERF_MAGIC,
            version: DLL_PERF_VERSION,
            worker_wake_set_event_count: 7,
            ..Default::default()
        };
        let lines = format_perf_lines(&snap);
        assert!(lines.iter().any(|l| l.starts_with("perf:")));
        assert!(lines.iter().any(|l| l.contains("worker_wake_set_event_count=7")));
    }

    fn count_substring(haystack: &str, needle: &str) -> usize {
        haystack.match_indices(needle).count()
    }

    #[test]
    fn empty_route_message_prints_route_status_once() {
        let snap = RouteSnapshot {
            sequence: 1,
            route_hash: 0,
            valid: false,
            flags: 0,
            bb_status: ROUTE_BB_STATUS_DLL_ACTIVE,
            resolve_status: RESOLVE_NONE,
            resolve_attempts: 0,
            publish_status: PUBLISH_EMPTY,
            frame_cb_count: 0,
            route_tick_count: 0,
            frame_start_count: 0,
            last_waypoint_count: 0,
            frame_end_count: 0,
            world_reset_count: 0,
            last_world_reset_reason: 0,
            world_reset_suppressed_count: 0,
            waypoints: vec![],
        };
        let msg = format_empty_route_message(&snap);
        assert_eq!(count_substring(&msg, "dll_active="), 1);
        assert_eq!(count_substring(&msg, "route_resolve_status="), 1);
    }

    #[test]
    fn active_route_summary_prints_route_status_once() {
        let msg = format_active_route_summary(&sample_snap());
        assert_eq!(count_substring(&msg, "dll_active="), 1);
        assert_eq!(count_substring(&msg, "route_resolve_status="), 1);
    }

    #[test]
    fn perf_unavailable_line_matches_expected_format() {
        use truckpilot_telemetry::dll_perf::format_perf_unavailable;
        let line = format_perf_unavailable("OpenFileMappingW failed for Local\\TruckPilotDllPerf");
        assert_eq!(
            line,
            "perf: unavailable (OpenFileMappingW failed for Local\\TruckPilotDllPerf)"
        );
    }
}
