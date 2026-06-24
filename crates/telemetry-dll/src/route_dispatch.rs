//! Route tick dispatch from SCS frame callbacks — hard off gate before worker notify.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Once;

use crate::frame_perf::{self, PerfBucketId, PerfGuard};
use crate::resolver_worker::RESOLVER_RESET_REQUESTED;
use crate::route_status::{RouteTickSource, SCS_EVENT_FRAME_END, SCS_EVENT_FRAME_START};

static OFF_DISPATCH_LOGGED: Once = Once::new();

static DISPATCH_GEN_BASELINE: AtomicU64 = AtomicU64::new(0);
static DISPATCH_GEN_BASELINE_INIT: AtomicBool = AtomicBool::new(false);

/// Signals evaluated once per dispatch — separate from legacy combined `wake_exception`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DispatchWakeEval {
    pub mode_off: bool,
    pub reset_requested: bool,
    pub enable_generation_changed: bool,
    /// Legacy composite: reset OR enable generation changed (instrumentation only).
    pub wake_exception: bool,
    /// Exactly one worker wake allowed this dispatch.
    pub force_wake: bool,
    /// Hard off gate: suppress notify/worker entirely.
    pub take_off_gate: bool,
}

/// Establish enable-generation baseline at DLL init (or test reset).
pub fn init_dispatch_wake_baseline() {
    let gen = crate::safe_mem::enable_file_generation();
    DISPATCH_GEN_BASELINE.store(gen, Ordering::Release);
    DISPATCH_GEN_BASELINE_INIT.store(true, Ordering::Release);
}

#[cfg(test)]
pub fn reset_dispatch_wake_test_state() {
    DISPATCH_GEN_BASELINE.store(0, Ordering::Release);
    DISPATCH_GEN_BASELINE_INIT.store(false, Ordering::Release);
    init_dispatch_wake_baseline();
}

fn commit_dispatch_generation_baseline() {
    DISPATCH_GEN_BASELINE.store(
        crate::safe_mem::enable_file_generation(),
        Ordering::Release,
    );
    DISPATCH_GEN_BASELINE_INIT.store(true, Ordering::Release);
}

/// Evaluate wake/reset/generation signals for one dispatch tick.
pub fn evaluate_dispatch_wake() -> DispatchWakeEval {
    let mode = crate::safe_mem::route_resolver_mode();
    let mode_off = mode.is_off();
    let reset_requested = RESOLVER_RESET_REQUESTED.load(Ordering::Acquire);
    let gen = crate::safe_mem::enable_file_generation();

    let enable_generation_changed = if DISPATCH_GEN_BASELINE_INIT.load(Ordering::Acquire) {
        gen != DISPATCH_GEN_BASELINE.load(Ordering::Acquire)
    } else {
        DISPATCH_GEN_BASELINE.store(gen, Ordering::Release);
        DISPATCH_GEN_BASELINE_INIT.store(true, Ordering::Release);
        false
    };

    let wake_exception = reset_requested || enable_generation_changed;

    // Off mode: reset+gen bump (world reset) must not wake; enable-only change may wake once.
    let force_wake = if mode_off {
        enable_generation_changed && !reset_requested
    } else {
        reset_requested || enable_generation_changed
    };

    // Task C safety fallback — do not trust wake_exception alone while off.
    let take_off_gate = mode_off && !force_wake;

    DispatchWakeEval {
        mode_off,
        reset_requested,
        enable_generation_changed,
        wake_exception,
        force_wake,
        take_off_gate,
    }
}

fn consume_off_gate_pending(eval: &DispatchWakeEval) {
    if eval.reset_requested {
        let _ = RESOLVER_RESET_REQUESTED.swap(false, Ordering::AcqRel);
        frame_perf::note_dispatch_reset_exception();
    }
    if eval.enable_generation_changed {
        commit_dispatch_generation_baseline();
        frame_perf::note_dispatch_enable_generation_changed();
    }
}

fn consume_notify_pending(eval: &DispatchWakeEval) {
    if eval.reset_requested {
        let _ = RESOLVER_RESET_REQUESTED.swap(false, Ordering::AcqRel);
        frame_perf::note_dispatch_reset_exception();
    }
    if eval.enable_generation_changed {
        commit_dispatch_generation_baseline();
        frame_perf::note_dispatch_enable_generation_changed();
    }
}

fn record_dispatch_instrumentation(eval: &DispatchWakeEval) {
    if eval.mode_off {
        frame_perf::note_dispatch_mode_off();
    }
    if eval.wake_exception {
        frame_perf::note_dispatch_wake_exception();
    }
}

fn apply_off_gate_dispatch(ts_qpc: u64, source: RouteTickSource) {
    frame_perf::note_off_mode_notify_suppressed();
    frame_perf::note_dispatch_off_gate_taken();
    crate::resolver_worker::apply_route_tick_dispatch(ts_qpc, source);
    crate::nav_route::write_bb_frame_from_atomics();
    OFF_DISPATCH_LOGGED.call_once(|| {
        crate::diag_log::event_force("dispatch route tick suppressed because resolver mode=off");
    });
}

/// Dispatch a route tick from a frame callback event (frame_end or frame_start fallback).
pub fn dispatch_route_tick(event: u32, ts_qpc: u64) {
    if crate::safe_mem::minimal_telemetry_enabled() {
        return;
    }
    let source = if event == SCS_EVENT_FRAME_END {
        RouteTickSource::FrameEnd
    } else if event == SCS_EVENT_FRAME_START && crate::nav_route::should_tick_on_frame_start(ts_qpc)
    {
        RouteTickSource::FrameStartFallback
    } else {
        return;
    };
    let _guard = PerfGuard::begin(PerfBucketId::DispatchNotifyFrameTick);
    frame_perf::note_route_tick_dispatch();

    let eval = evaluate_dispatch_wake();
    record_dispatch_instrumentation(&eval);

    if eval.take_off_gate {
        consume_off_gate_pending(&eval);
        apply_off_gate_dispatch(ts_qpc, source);
        return;
    }

    if eval.force_wake {
        frame_perf::note_dispatch_force_wake();
    }
    consume_notify_pending(&eval);
    frame_perf::note_dispatch_notify_called();
    crate::resolver_worker::notify_frame_tick(ts_qpc, source, eval.force_wake);
    crate::nav_route::write_bb_frame_from_atomics();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::resolver_worker;
    use crate::test_isolation::TestResolverStateGuard;
    use std::sync::atomic::Ordering;

    fn temp_off_dir() -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("tp-dispatch-off-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");
        dir
    }

    fn temp_probe_dir() -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("tp-dispatch-probe-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");
        std::fs::write(
            dir.join("truckpilot_route_resolver.gps_offset_probe"),
            b"",
        )
        .expect("probe enable");
        dir
    }

    fn assert_off_baseline_counters(expected_dispatches: u32, expected_wake_exceptions: u32) {
        assert_eq!(
            frame_perf::DISPATCH_MODE_OFF_COUNT.load(Ordering::Relaxed),
            expected_dispatches
        );
        assert_eq!(
            frame_perf::DISPATCH_WAKE_EXCEPTION_COUNT.load(Ordering::Relaxed),
            expected_wake_exceptions
        );
        assert_eq!(
            frame_perf::DISPATCH_OFF_GATE_TAKEN_COUNT.load(Ordering::Relaxed),
            expected_dispatches
        );
        assert_eq!(
            frame_perf::DISPATCH_NOTIFY_CALLED_COUNT.load(Ordering::Relaxed),
            0
        );
        assert_eq!(
            frame_perf::NOTIFY_FRAME_TICK_COUNT.load(Ordering::Relaxed),
            0
        );
        assert_eq!(
            frame_perf::WORKER_WAKE_SET_EVENT_COUNT.load(Ordering::Relaxed),
            0
        );
        assert_eq!(
            frame_perf::OFF_MODE_NOTIFY_SUPPRESSED_COUNT.load(Ordering::Relaxed),
            expected_dispatches
        );
    }

    #[test]
    fn off_mode_initial_generation_baseline_suppresses_all_dispatches() {
        let _guard = TestResolverStateGuard::acquire();
        let dir = temp_off_dir();
        crate::safe_mem::set_test_enable_dir(Some(dir.clone()));
        assert!(crate::safe_mem::route_resolver_mode().is_off());
        assert_eq!(crate::safe_mem::enable_file_generation(), 1);
        for i in 0..10_000 {
            dispatch_route_tick(SCS_EVENT_FRAME_END, i);
        }
        assert_off_baseline_counters(10_000, 0);
        assert_eq!(
            frame_perf::DISPATCH_FORCE_WAKE_COUNT.load(Ordering::Relaxed),
            0
        );
        assert_eq!(resolver_worker::WORKER_WALK_COUNT.load(Ordering::Relaxed), 0);
        assert_eq!(
            frame_perf::ROUTE_TICK_DISPATCH_COUNT.load(Ordering::Relaxed),
            10_000
        );
        crate::safe_mem::set_test_enable_dir(None);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn off_mode_persistent_generation_never_wakes() {
        let _guard = TestResolverStateGuard::acquire();
        let dir = temp_off_dir();
        crate::safe_mem::set_test_enable_dir(Some(dir.clone()));
        init_dispatch_wake_baseline();
        for i in 0..5_000 {
            dispatch_route_tick(SCS_EVENT_FRAME_END, i);
        }
        assert_off_baseline_counters(5_000, 0);
        crate::safe_mem::set_test_enable_dir(None);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn off_mode_stuck_reset_uses_safety_fallback_not_notify() {
        let _guard = TestResolverStateGuard::acquire();
        let dir = temp_off_dir();
        crate::safe_mem::set_test_enable_dir(Some(dir.clone()));
        init_dispatch_wake_baseline();
        RESOLVER_RESET_REQUESTED.store(true, Ordering::Release);
        crate::safe_mem::bump_enable_file_generation();
        for i in 0..10_000 {
            dispatch_route_tick(SCS_EVENT_FRAME_END, i);
        }
        assert_off_baseline_counters(10_000, 1);
        assert_eq!(
            frame_perf::DISPATCH_RESET_EXCEPTION_COUNT.load(Ordering::Relaxed),
            1,
            "reset should be consumed once on first off-gate"
        );
        assert!(!RESOLVER_RESET_REQUESTED.load(Ordering::Relaxed));
        crate::safe_mem::set_test_enable_dir(None);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn enable_generation_change_allows_single_notify_while_off() {
        let _guard = TestResolverStateGuard::acquire();
        let dir = temp_off_dir();
        crate::safe_mem::set_test_enable_dir(Some(dir.clone()));
        crate::resolver_metrics::set_resolver_parked(true);
        dispatch_route_tick(SCS_EVENT_FRAME_END, 1);
        assert_eq!(
            frame_perf::OFF_MODE_NOTIFY_SUPPRESSED_COUNT.load(Ordering::Relaxed),
            1
        );
        crate::safe_mem::bump_enable_file_generation();
        dispatch_route_tick(SCS_EVENT_FRAME_END, 2);
        assert_eq!(
            frame_perf::NOTIFY_FRAME_TICK_COUNT.load(Ordering::Relaxed),
            1
        );
        assert_eq!(
            frame_perf::WORKER_WAKE_SET_EVENT_COUNT.load(Ordering::Relaxed),
            1
        );
        assert_eq!(
            frame_perf::DISPATCH_ENABLE_GENERATION_CHANGED_COUNT.load(Ordering::Relaxed),
            1
        );
        dispatch_route_tick(SCS_EVENT_FRAME_END, 3);
        assert_eq!(
            frame_perf::NOTIFY_FRAME_TICK_COUNT.load(Ordering::Relaxed),
            1,
            "second tick after gen change must not notify again"
        );
        crate::safe_mem::set_test_enable_dir(None);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn reset_requested_active_mode_allows_single_wake() {
        let _guard = TestResolverStateGuard::acquire();
        let dir = temp_probe_dir();
        crate::safe_mem::set_test_enable_dir(Some(dir.clone()));
        init_dispatch_wake_baseline();
        dispatch_route_tick(SCS_EVENT_FRAME_END, 1);
        assert_eq!(
            frame_perf::WORKER_WAKE_SET_EVENT_COUNT.load(Ordering::Relaxed),
            1
        );
        resolver_worker::clear_pending_resolve_for_test();
        RESOLVER_RESET_REQUESTED.store(true, Ordering::Release);
        dispatch_route_tick(SCS_EVENT_FRAME_END, 2);
        assert_eq!(
            frame_perf::WORKER_WAKE_SET_EVENT_COUNT.load(Ordering::Relaxed),
            2
        );
        assert_eq!(
            frame_perf::DISPATCH_RESET_EXCEPTION_COUNT.load(Ordering::Relaxed),
            1
        );
        crate::resolver_metrics::set_resolver_parked(true);
        for i in 0..100 {
            dispatch_route_tick(SCS_EVENT_FRAME_END, i + 3);
        }
        assert_eq!(
            frame_perf::WORKER_WAKE_SET_EVENT_COUNT.load(Ordering::Relaxed),
            2
        );
        crate::safe_mem::set_test_enable_dir(None);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn gps_offset_probe_dispatch_allows_worker_wake_before_park() {
        let _guard = TestResolverStateGuard::acquire();
        let dir = temp_probe_dir();
        crate::safe_mem::set_test_enable_dir(Some(dir.clone()));
        assert!(!crate::safe_mem::route_resolver_mode().is_off());
        dispatch_route_tick(SCS_EVENT_FRAME_END, 1);
        assert_eq!(
            frame_perf::NOTIFY_FRAME_TICK_COUNT.load(Ordering::Relaxed),
            1
        );
        assert_eq!(
            frame_perf::WORKER_WAKE_SET_EVENT_COUNT.load(Ordering::Relaxed),
            1
        );
        crate::resolver_metrics::set_resolver_parked(true);
        for i in 0..100 {
            dispatch_route_tick(SCS_EVENT_FRAME_END, i + 2);
        }
        assert_eq!(
            frame_perf::WORKER_WAKE_SET_EVENT_COUNT.load(Ordering::Relaxed),
            1
        );
        assert_eq!(
            frame_perf::OFF_MODE_NOTIFY_SUPPRESSED_COUNT.load(Ordering::Relaxed),
            0
        );
        crate::safe_mem::set_test_enable_dir(None);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
