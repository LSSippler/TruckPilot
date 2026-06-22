//! Production safety / stutter-regression guards (unit tests).

use crate::resolver_metrics::{
    self, DIAGNOSTIC_TABLE_RUNS, FRAME_CALLBACK_SYNC_RESOLVER_CALLS,
    PATTERN_SCANS_FROM_FRAME_CALLBACK, RESOLVER_PARKED, RESOLVER_WORKER_PATTERN_SCAN_COUNT,
};
use crate::resolver_sched::ResolverSchedule;
use crate::resolver_worker;
use crate::route_status::RouteTickSource;
use crate::safe_mem::{self, RouteResolverMode};

#[test]
fn default_off_mode_performs_no_resolver_work() {
    assert!(RouteResolverMode::SafeDefault.is_off());
    assert!(!RouteResolverMode::SafeDefault.enables_resolver_work());
    assert!(!RouteResolverMode::SafeDefault.is_table_diagnostic());
}

#[test]
fn enable_file_priority_full_beats_tables() {
    assert_eq!(
        safe_mem::select_route_resolver_mode(true, true, true, true, true),
        RouteResolverMode::FullDeep
    );
    assert_eq!(
        safe_mem::select_route_resolver_mode(true, true, true, true, false),
        RouteResolverMode::StaticChain
    );
    assert_eq!(
        safe_mem::select_route_resolver_mode(false, false, false, false, false),
        RouteResolverMode::SafeDefault
    );
}

#[test]
fn frame_callback_never_increments_sync_resolver_metrics() {
    resolver_metrics::reset_test_metrics();
    resolver_worker::reset_test_counters();
    resolver_worker::notify_frame_tick(1_000_000, RouteTickSource::FrameEnd);
    assert_eq!(
        FRAME_CALLBACK_SYNC_RESOLVER_CALLS.load(std::sync::atomic::Ordering::Relaxed),
        0
    );
    assert_eq!(
        PATTERN_SCANS_FROM_FRAME_CALLBACK.load(std::sync::atomic::Ordering::Relaxed),
        0
    );
}

#[test]
fn diagnostic_parked_blocks_repeated_table_runs() {
    resolver_metrics::reset_test_metrics();
    let mut sched = ResolverSchedule::new();
    sched.park_diagnostic_done();
    assert!(!sched.should_run_walk(u64::MAX));
    assert!(RESOLVER_PARKED.load(std::sync::atomic::Ordering::Relaxed));
}

#[test]
fn route_candidate_table_mode_is_one_shot_by_schedule() {
    let mut sched = ResolverSchedule::new();
    assert!(sched.should_run_walk(1));
    sched.note_walk_started(1);
    sched.park_diagnostic_done();
    assert!(!sched.should_run_walk(10_000_000));
}

#[test]
fn off_mode_sidecar_label_is_off() {
    assert_eq!(RouteResolverMode::SafeDefault.sidecar_label(), "off");
}

#[test]
fn worker_pattern_scan_counter_starts_at_zero() {
    resolver_metrics::reset_test_metrics();
    assert_eq!(
        RESOLVER_WORKER_PATTERN_SCAN_COUNT.load(std::sync::atomic::Ordering::Relaxed),
        0
    );
    assert_eq!(DIAGNOSTIC_TABLE_RUNS.load(std::sync::atomic::Ordering::Relaxed), 0);
}

#[test]
fn enable_files_constant_matches_spec() {
    assert_eq!(safe_mem::RESOLVER_ENABLE_FILES.len(), 5);
    assert_eq!(
        safe_mem::RESOLVER_ENABLE_FILES[0].0,
        "truckpilot_route_resolver.full"
    );
    assert_eq!(
        safe_mem::RESOLVER_ENABLE_FILES[4].0,
        "truckpilot_route_resolver.gps_table"
    );
}
