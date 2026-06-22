//! Offline baseline safety invariants — no ETS2, no live memory.

use std::fs::File;
use std::sync::atomic::Ordering;
use std::sync::{Mutex, MutexGuard};

use crate::nav_resolve;
use crate::resolver_guard::{self, WalkDecision};
use crate::resolver_metrics::{
    self, DIAGNOSTIC_TABLE_RUNS, FRAME_CALLBACK_SYNC_RESOLVER_CALLS,
    OFF_MODE_BLOCKED_CALLS, PATTERN_SCANS_FROM_FRAME_CALLBACK, RESOLVER_PARKED,
    RESOLVER_WALK_PROCEEDED, RESOLVER_WORKER_PATTERN_SCAN_COUNT,
};
use crate::resolver_sched::{ResolverSchedule, BACKOFF_STEPS_US, MAX_PATTERN_SCANS_PER_SESSION};
use crate::resolver_worker;
use crate::route_chain;
use crate::route_status::{
    RouteTickSource, RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE,
};
use crate::safe_mem::{self, RouteResolverMode};

static METRICS_TEST_LOCK: Mutex<()> = Mutex::new(());

fn isolated_metrics_test() -> MutexGuard<'static, ()> {
    let guard = METRICS_TEST_LOCK.lock().unwrap();
    resolver_metrics::reset_test_metrics();
    resolver_worker::reset_test_counters();
    guard
}

fn temp_enable_dir(prefix: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("tp-resolver-{prefix}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("temp enable dir");
    dir
}

#[test]
fn no_enable_file_selects_off_mode() {
    let dir = temp_enable_dir("none");
    safe_mem::set_test_enable_dir(Some(dir.clone()));
    let sel = safe_mem::detect_resolver_mode_selection_from_dir(&dir);
    assert!(sel.mode.is_off());
    assert_eq!(sel.mode.sidecar_label(), "off");
    assert_eq!(sel.source_file, "none");
    safe_mem::set_test_enable_dir(None);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn off_mode_status_constant_is_disabled_safe_mode() {
    assert_eq!(
        RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE,
        46,
        "must match route_shm_dump baseline"
    );
}

#[test]
fn off_mode_blocks_resolve_game_ctrl_cached_without_scan() {
    let _lock = isolated_metrics_test();
    safe_mem::set_test_enable_dir(Some(temp_enable_dir("off-cache")));
    let err = unsafe { nav_resolve::resolve_game_ctrl_cached(false, true) };
    assert!(err.is_err());
    assert_eq!(
        RESOLVER_WORKER_PATTERN_SCAN_COUNT.load(Ordering::Relaxed),
        0
    );
    assert!(OFF_MODE_BLOCKED_CALLS.load(Ordering::Relaxed) >= 1);
    safe_mem::set_test_enable_dir(None);
}

#[test]
fn off_mode_blocks_resolve_game_ctrl_manager_without_scan() {
    let _lock = isolated_metrics_test();
    safe_mem::set_test_enable_dir(Some(temp_enable_dir("off-mgr")));
    let err = unsafe { nav_resolve::resolve_game_ctrl_manager() };
    assert!(err.is_err());
    assert_eq!(
        RESOLVER_WORKER_PATTERN_SCAN_COUNT.load(Ordering::Relaxed),
        0
    );
    safe_mem::set_test_enable_dir(None);
}

#[test]
fn off_mode_blocks_route_chain_diagnostics_without_table_reads() {
    let _lock = isolated_metrics_test();
    safe_mem::set_test_enable_dir(Some(temp_enable_dir("off-chain")));
    let st = route_chain::run_route_candidate_table_only_diagnostic(0x10_0000);
    assert_eq!(st, RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE);
    assert_eq!(DIAGNOSTIC_TABLE_RUNS.load(Ordering::Relaxed), 0);
    assert_eq!(
        route_chain::run_gps_table_only_diagnostic(0x20_0000),
        RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE
    );
    assert_eq!(
        route_chain::run_game_ctrl_table_only_diagnostic(0x30_0000),
        RESOLVE_ROUTE_RESOLVER_DISABLED_SAFE_MODE
    );
    safe_mem::set_test_enable_dir(None);
}

#[test]
fn only_gps_table_enable_file_selects_gps_mode() {
    let dir = temp_enable_dir("gps");
    File::create(dir.join("truckpilot_route_resolver.gps_table")).unwrap();
    let sel = safe_mem::detect_resolver_mode_selection_from_dir(&dir);
    assert_eq!(sel.mode, RouteResolverMode::GpsTableOnly);
    assert_eq!(sel.source_file, "truckpilot_route_resolver.gps_table");
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn only_game_ctrl_table_enable_file_selects_game_ctrl_mode() {
    let dir = temp_enable_dir("gct");
    File::create(dir.join("truckpilot_route_resolver.game_ctrl_table")).unwrap();
    let sel = safe_mem::detect_resolver_mode_selection_from_dir(&dir);
    assert_eq!(sel.mode, RouteResolverMode::GameCtrlTableOnly);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn only_route_candidate_table_enable_file_selects_candidate_mode() {
    let dir = temp_enable_dir("rct");
    File::create(dir.join("truckpilot_route_resolver.route_candidate_table")).unwrap();
    let sel = safe_mem::detect_resolver_mode_selection_from_dir(&dir);
    assert_eq!(sel.mode, RouteResolverMode::RouteCandidateTableOnly);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn multiple_enable_files_select_highest_priority() {
    let dir = temp_enable_dir("multi");
    File::create(dir.join("truckpilot_route_resolver.gps_table")).unwrap();
    File::create(dir.join("truckpilot_route_resolver.route_candidate_table")).unwrap();
    File::create(dir.join("truckpilot_route_resolver.full")).unwrap();
    let sel = safe_mem::detect_resolver_mode_selection_from_dir(&dir);
    assert_eq!(sel.mode, RouteResolverMode::FullDeep);
    assert_eq!(sel.source_file, "truckpilot_route_resolver.full");
    assert!(sel
        .ignored_lower_priority
        .iter()
        .any(|f| f.contains("route_candidate_table")));
    assert!(sel.ignored_lower_priority.iter().any(|f| f.contains("gps_table")));
    let lines = resolver_guard::format_mode_init_log_lines(&sel);
    assert!(lines.iter().any(|l| l.contains("mode selected=full source=")));
    assert!(lines
        .iter()
        .any(|l| l.contains("ignored lower-priority resolver mode files:")));
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn legacy_route_scan_enable_is_ignored_for_mode_selection() {
    let dir = temp_enable_dir("legacy");
    File::create(dir.join("truckpilot_route_scan.enable")).unwrap();
    assert!(safe_mem::legacy_route_scan_enable_present(&dir));
    let sel = safe_mem::detect_resolver_mode_selection_from_dir(&dir);
    assert!(sel.mode.is_off());
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn static_beats_table_modes_in_priority() {
    let dir = temp_enable_dir("static-pri");
    File::create(dir.join("truckpilot_route_resolver.static")).unwrap();
    File::create(dir.join("truckpilot_route_resolver.gps_table")).unwrap();
    let sel = safe_mem::detect_resolver_mode_selection_from_dir(&dir);
    assert_eq!(sel.mode, RouteResolverMode::StaticChain);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn off_mode_init_log_matches_live_baseline() {
    let sel = safe_mem::ResolverModeSelection {
        mode: RouteResolverMode::SafeDefault,
        source_file: "none",
        ignored_lower_priority: Vec::new(),
    };
    let lines = resolver_guard::format_mode_init_log_lines(&sel);
    assert_eq!(lines[0], "route resolver mode=off");
    assert!(lines.contains(&"route resolver disabled safe mode".to_string()));
    assert!(lines.contains(&"route resolver worker parked".to_string()));
}

#[test]
fn off_mode_walk_decision_parks_without_proceed() {
    let sched = ResolverSchedule::new();
    assert_eq!(
        resolver_guard::decide_walk(RouteResolverMode::SafeDefault, &sched, 0),
        WalkDecision::OffModePark
    );
}

#[test]
fn worker_may_start_but_off_mode_parks_via_decision() {
    let _lock = isolated_metrics_test();
    let mut sched = ResolverSchedule::new();
    sched.park_resolver_off();
    assert!(RESOLVER_PARKED.load(Ordering::Relaxed));
    assert_eq!(
        resolver_guard::decide_walk(RouteResolverMode::SafeDefault, &sched, 0),
        WalkDecision::OffModePark
    );
}

#[test]
fn frame_callback_regression_metrics_stay_zero_on_notify() {
    let _lock = isolated_metrics_test();
    resolver_worker::notify_frame_tick(1, RouteTickSource::FrameEnd);
    assert_eq!(FRAME_CALLBACK_SYNC_RESOLVER_CALLS.load(Ordering::Relaxed), 0);
    assert_eq!(PATTERN_SCANS_FROM_FRAME_CALLBACK.load(Ordering::Relaxed), 0);
}

#[test]
fn off_mode_resolver_walk_proceeded_stays_zero() {
    let _lock = isolated_metrics_test();
    safe_mem::set_test_enable_dir(Some(temp_enable_dir("off-proceed")));
    let sched = ResolverSchedule::new();
    assert_eq!(
        resolver_guard::decide_walk(RouteResolverMode::SafeDefault, &sched, 1),
        WalkDecision::OffModePark
    );
    assert_eq!(RESOLVER_WALK_PROCEEDED.load(Ordering::Relaxed), 0);
    safe_mem::set_test_enable_dir(None);
}

#[test]
fn backoff_sequence_reaches_sixty_second_max() {
    let mut s = ResolverSchedule::new();
    assert_eq!(s.backoff_us, BACKOFF_STEPS_US[0]);
    for (i, expected) in BACKOFF_STEPS_US.iter().enumerate().skip(1) {
        s.note_expensive_failure();
        assert_eq!(s.backoff_us, *expected, "after failure {i}");
    }
    s.note_expensive_failure();
    assert_eq!(s.backoff_us, 60_000_000);
}

#[test]
fn scan_limit_parks_and_blocks_walks_despite_ticks() {
    let mut s = ResolverSchedule::new();
    for _ in 0..MAX_PATTERN_SCANS_PER_SESSION {
        s.note_pattern_scan();
    }
    assert!(s.parked);
    assert!(!s.should_run_walk(u64::MAX));
}

#[test]
fn enable_generation_change_unparks_scheduler() {
    let mut s = ResolverSchedule::new();
    s.park_after_limit();
    assert!(s.parked);
    s.note_enable_generation(2);
    assert!(!s.parked);
    assert!(s.should_run_walk(1_000_000));
}

#[test]
fn route_candidate_diagnostic_parks_after_one_shot() {
    let mut sched = ResolverSchedule::new();
    assert!(sched.should_run_walk(1));
    sched.note_walk_started(1);
    sched.park_diagnostic_done();
    assert!(!sched.should_run_walk(u64::MAX));
    assert_eq!(
        resolver_guard::decide_walk(RouteResolverMode::RouteCandidateTableOnly, &sched, u64::MAX),
        WalkDecision::DiagnosticParked
    );
}
