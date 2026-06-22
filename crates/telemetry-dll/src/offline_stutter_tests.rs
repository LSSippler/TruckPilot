//! Offline frame-storm / stutter regression simulation — no ETS2, no SCS DLL.

use std::sync::atomic::Ordering;

use crate::resolver_guard::{self, WalkDecision};
use crate::resolver_metrics::{
    self, DIAGNOSTIC_TABLE_RUNS, FRAME_CALLBACK_SYNC_RESOLVER_CALLS,
    PATTERN_SCANS_FROM_FRAME_CALLBACK, RESOLVER_WALK_PROCEEDED,
    RESOLVER_WORKER_PATTERN_SCAN_COUNT,
};
use crate::resolver_sched::ResolverSchedule;
use crate::resolver_worker;
use crate::route_status::RouteTickSource;
use crate::safe_mem::{self, RouteResolverMode};
use crate::test_isolation::TestResolverStateGuard;

const FRAME_STORM_COUNT: u32 = 100_000;
const FRAME_END_STORM_COUNT: u32 = 1_000;

#[test]
fn frame_storm_notify_frame_tick_stays_o1_without_scans() {
    let _guard = TestResolverStateGuard::acquire();
    let storm_dir = std::env::temp_dir().join(format!("tp-storm-off-{}", std::process::id()));
    let _ = std::fs::create_dir_all(&storm_dir);
    safe_mem::set_test_enable_dir(Some(storm_dir.clone()));

    let scans_before = RESOLVER_WORKER_PATTERN_SCAN_COUNT.load(Ordering::Relaxed);
    let ticks_before = resolver_worker::ROUTE_TICK_COUNT.load(Ordering::Relaxed);
    let t0 = std::time::Instant::now();
    for i in 0..FRAME_STORM_COUNT {
        resolver_worker::notify_frame_tick(i as u64, RouteTickSource::FrameStartFallback);
    }
    for i in 0..FRAME_END_STORM_COUNT {
        resolver_worker::notify_frame_tick(i as u64, RouteTickSource::FrameEnd);
    }
    let elapsed = t0.elapsed();
    let ticks_after = resolver_worker::ROUTE_TICK_COUNT.load(Ordering::Relaxed);
    let scans_after = RESOLVER_WORKER_PATTERN_SCAN_COUNT.load(Ordering::Relaxed);

    assert!(
        ticks_after.saturating_sub(ticks_before) >= FRAME_STORM_COUNT + FRAME_END_STORM_COUNT,
        "notify_frame_tick did not advance route tick counter"
    );
    assert_eq!(FRAME_CALLBACK_SYNC_RESOLVER_CALLS.load(Ordering::Relaxed), 0);
    assert_eq!(PATTERN_SCANS_FROM_FRAME_CALLBACK.load(Ordering::Relaxed), 0);
    assert_eq!(scans_after.saturating_sub(scans_before), 0);
    assert_eq!(DIAGNOSTIC_TABLE_RUNS.load(Ordering::Relaxed), 0);
    assert_eq!(RESOLVER_WALK_PROCEEDED.load(Ordering::Relaxed), 0);
    assert!(
        elapsed.as_secs() < 30,
        "100k notify_frame_tick took {:?} — possible sync regression",
        elapsed
    );

    safe_mem::set_test_enable_dir(None);
    let _ = std::fs::remove_dir_all(&storm_dir);
}

#[test]
fn off_mode_decision_under_storm_never_proceeds() {
    let sched = ResolverSchedule::new();
    for ts in (0..10_000).step_by(7) {
        assert_eq!(
            resolver_guard::decide_walk(RouteResolverMode::SafeDefault, &sched, ts),
            WalkDecision::OffModePark
        );
    }
}

#[test]
fn diagnostic_done_blocks_further_candidate_table_decisions() {
    let mut sched = ResolverSchedule::new();
    sched.park_diagnostic_done();
    for ts in 0..5_000 {
        assert_eq!(
            resolver_guard::decide_walk(RouteResolverMode::RouteCandidateTableOnly, &sched, ts),
            WalkDecision::DiagnosticParked
        );
    }
    assert_eq!(RESOLVER_WORKER_PATTERN_SCAN_COUNT.load(Ordering::Relaxed), 0);
}

#[test]
#[ignore = "run with: cargo test -p truckpilot-telemetry-dll offline_microbench --release -- --ignored"]
fn offline_microbench_notify_frame_tick_100k() {
    resolver_metrics::reset_test_metrics();
    resolver_worker::reset_test_counters();
    let t0 = std::time::Instant::now();
    for i in 0..FRAME_STORM_COUNT {
        resolver_worker::notify_frame_tick(i as u64, RouteTickSource::FrameEnd);
    }
    eprintln!(
        "100k notify_frame_tick: {:?}, scans={}",
        t0.elapsed(),
        RESOLVER_WORKER_PATTERN_SCAN_COUNT.load(Ordering::Relaxed)
    );
    assert_eq!(PATTERN_SCANS_FROM_FRAME_CALLBACK.load(Ordering::Relaxed), 0);
}
