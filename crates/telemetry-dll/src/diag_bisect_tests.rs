//! Offline bisect-level tests: prove each diag level activates only its own
//! component and that all later hot-path components stay cold (counters at 0).
//!
//! These run with [`TestResolverStateGuard`], which serializes access to the
//! process-global perf atomics and resets them (including the v5 diag counters)
//! before each case.

use std::sync::atomic::Ordering;

use crate::diag_level::DiagLevel;
use crate::frame_perf::{
    CALLBACK_COUNTER_COUNT, CALLBACK_NOOP_COUNT, CALLBACK_QPC_COUNT, INPUT_EVENT_CB_COUNT,
    NOTIFY_FRAME_TICK_COUNT, PERF_SNAPSHOT_COUNT, READY_EVENT_COMPONENT_COUNT,
    READY_EVENT_SET_COUNT, ROUTE_BB_COMPONENT_COUNT, ROUTE_BB_FRAME_WRITE_COUNT,
    ROUTE_TICK_DISPATCH_COUNT, SHM_WRITE_COUNT, TELEMETRY_SHM_COMPONENT_COUNT,
    WORKER_WAKE_SET_EVENT_COUNT,
};
use crate::route_status::SCS_EVENT_FRAME_END;
use crate::test_isolation::TestResolverStateGuard;

const FRAMES: u32 = 10_000;

fn load(c: &std::sync::atomic::AtomicU32) -> u32 {
    c.load(Ordering::Relaxed)
}

/// Drive `level` through `FRAMES` diag frame callbacks.
fn run_frames(level: DiagLevel) {
    for _ in 0..FRAMES {
        unsafe { crate::diag_handle_frame(level, SCS_EVENT_FRAME_END, std::ptr::null()) };
    }
}

/// Assert the whole "later than X" tail of the hot path is cold.
fn assert_dispatch_and_worker_cold() {
    assert_eq!(load(&ROUTE_TICK_DISPATCH_COUNT), 0, "dispatch must be cold");
    assert_eq!(load(&NOTIFY_FRAME_TICK_COUNT), 0, "notify must be cold");
    assert_eq!(load(&WORKER_WAKE_SET_EVENT_COUNT), 0, "worker wake must be cold");
    assert_eq!(load(&INPUT_EVENT_CB_COUNT), 0, "input must be cold");
}

fn temp_dir(tag: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("tp-diag-bisect-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("temp dir");
    dir
}

#[test]
fn callback_noop_increments_only_noop_counter() {
    let _g = TestResolverStateGuard::acquire();
    run_frames(DiagLevel::CallbackNoop);
    assert_eq!(load(&CALLBACK_NOOP_COUNT), FRAMES);
    // Every later component counter stays 0.
    assert_eq!(load(&CALLBACK_COUNTER_COUNT), 0);
    assert_eq!(load(&CALLBACK_QPC_COUNT), 0);
    assert_eq!(load(&PERF_SNAPSHOT_COUNT), 0);
    assert_eq!(load(&TELEMETRY_SHM_COMPONENT_COUNT), 0);
    assert_eq!(load(&READY_EVENT_COMPONENT_COUNT), 0);
    assert_eq!(load(&ROUTE_BB_COMPONENT_COUNT), 0);
    assert_eq!(load(&SHM_WRITE_COUNT), 0);
    assert_eq!(load(&READY_EVENT_SET_COUNT), 0);
    assert_eq!(load(&ROUTE_BB_FRAME_WRITE_COUNT), 0);
    assert_dispatch_and_worker_cold();
}

#[test]
fn callback_counter_increments_only_counter() {
    let _g = TestResolverStateGuard::acquire();
    run_frames(DiagLevel::CallbackCounter);
    assert_eq!(load(&CALLBACK_COUNTER_COUNT), FRAMES);
    assert_eq!(load(&CALLBACK_NOOP_COUNT), 0);
    assert_eq!(load(&CALLBACK_QPC_COUNT), 0);
    assert_eq!(load(&SHM_WRITE_COUNT), 0);
    assert_eq!(load(&READY_EVENT_SET_COUNT), 0);
    assert_eq!(load(&ROUTE_BB_FRAME_WRITE_COUNT), 0);
    assert_dispatch_and_worker_cold();
}

#[test]
fn callback_qpc_increments_only_qpc_counter() {
    let _g = TestResolverStateGuard::acquire();
    run_frames(DiagLevel::CallbackQpc);
    assert_eq!(load(&CALLBACK_QPC_COUNT), FRAMES);
    assert_eq!(load(&PERF_SNAPSHOT_COUNT), 0);
    assert_eq!(load(&SHM_WRITE_COUNT), 0);
    assert_eq!(load(&READY_EVENT_SET_COUNT), 0);
    assert_eq!(load(&ROUTE_BB_FRAME_WRITE_COUNT), 0);
    assert_dispatch_and_worker_cold();
}

#[test]
fn perf_snapshot_increments_only_perf_counter() {
    let _g = TestResolverStateGuard::acquire();
    run_frames(DiagLevel::PerfSnapshot);
    assert_eq!(load(&PERF_SNAPSHOT_COUNT), FRAMES);
    assert_eq!(load(&TELEMETRY_SHM_COMPONENT_COUNT), 0);
    assert_eq!(load(&SHM_WRITE_COUNT), 0, "shm_write_count must be 0");
    assert_eq!(load(&READY_EVENT_SET_COUNT), 0);
    assert_eq!(load(&ROUTE_BB_FRAME_WRITE_COUNT), 0);
    assert_dispatch_and_worker_cold();
}

#[test]
fn telemetry_shm_writes_shm_but_no_ready_or_bb() {
    let _g = TestResolverStateGuard::acquire();
    run_frames(DiagLevel::TelemetryShm);
    assert_eq!(load(&TELEMETRY_SHM_COMPONENT_COUNT), FRAMES);
    assert_eq!(load(&SHM_WRITE_COUNT), FRAMES, "shm_write_count must rise");
    assert_eq!(load(&READY_EVENT_SET_COUNT), 0, "ready must stay 0");
    assert_eq!(load(&ROUTE_BB_FRAME_WRITE_COUNT), 0, "bb must stay 0");
    assert_dispatch_and_worker_cold();
}

#[test]
fn ready_event_sets_event_and_shm_but_no_bb() {
    let _g = TestResolverStateGuard::acquire();
    run_frames(DiagLevel::ReadyEvent);
    assert_eq!(load(&READY_EVENT_COMPONENT_COUNT), FRAMES);
    assert_eq!(load(&SHM_WRITE_COUNT), FRAMES, "shm_write_count must rise");
    assert_eq!(load(&READY_EVENT_SET_COUNT), FRAMES, "ready must rise");
    assert_eq!(load(&ROUTE_BB_FRAME_WRITE_COUNT), 0, "bb must stay 0");
    assert_dispatch_and_worker_cold();
}

#[test]
fn route_bb_writes_bb_but_no_dispatch_or_worker() {
    let _g = TestResolverStateGuard::acquire();
    run_frames(DiagLevel::RouteBb);
    assert_eq!(load(&ROUTE_BB_COMPONENT_COUNT), FRAMES);
    assert_eq!(load(&ROUTE_BB_FRAME_WRITE_COUNT), FRAMES, "bb write must rise");
    // Cumulative lower components also ran.
    assert_eq!(load(&SHM_WRITE_COUNT), FRAMES);
    assert_eq!(load(&READY_EVENT_SET_COUNT), FRAMES);
    // But dispatch / worker / resolver remain cold.
    assert_dispatch_and_worker_cold();
    assert_eq!(
        crate::frame_perf::RESOLVER_ATTEMPTS.load(Ordering::Relaxed),
        0,
        "resolver must never attempt in route_bb diag level"
    );
}

#[test]
fn load_only_and_init_only_frame_path_is_noop() {
    let _g = TestResolverStateGuard::acquire();
    // These levels never register callbacks; if a stray frame reaches the diag
    // dispatcher it must do nothing.
    for level in [DiagLevel::LoadOnly, DiagLevel::InitOnly] {
        unsafe { crate::diag_handle_frame(level, SCS_EVENT_FRAME_END, std::ptr::null()) };
    }
    assert_eq!(load(&CALLBACK_NOOP_COUNT), 0);
    assert_eq!(load(&SHM_WRITE_COUNT), 0);
    assert_eq!(load(&ROUTE_BB_FRAME_WRITE_COUNT), 0);
    assert_dispatch_and_worker_cold();
}

#[test]
fn diag_levels_force_resolver_off_even_with_enable_file() {
    let _g = TestResolverStateGuard::acquire();
    // A resolver enable file present alongside a diag level must be ignored.
    let dir = temp_dir("resolver-block");
    std::fs::write(dir.join("truckpilot_diag.callback_noop"), b"").expect("diag");
    std::fs::write(dir.join("truckpilot_route_resolver.full"), b"").expect("resolver");
    crate::safe_mem::set_test_enable_dir(Some(dir.clone()));
    assert_eq!(crate::diag_level::active(), DiagLevel::CallbackNoop);
    assert!(crate::diag_level::resolver_forced_off());
    assert!(crate::safe_mem::route_resolver_mode().is_off());
    assert!(crate::resolver_guard::block_if_resolver_off().is_some());
    crate::safe_mem::set_test_enable_dir(None);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn normal_default_off_with_input_disable_reports_input_not_registered() {
    let _g = TestResolverStateGuard::acquire();
    // The bug: sidecar logged input_registered=true while the perf dump showed
    // diag_input_registered=false. Both must now agree on the effective state.
    let dir = temp_dir("normal-input-disable");
    std::fs::write(dir.join("truckpilot_diag.normal_default_off"), b"").expect("normal");
    std::fs::write(dir.join("truckpilot_input.disable"), b"").expect("input off");
    crate::safe_mem::set_test_enable_dir(Some(dir.clone()));
    assert_eq!(crate::diag_level::active(), DiagLevel::Normal);
    assert!(crate::safe_mem::input_plugin_disabled());
    assert!(
        !crate::diag_level::effective_input_registered(),
        "input.disable must yield input_registered=false even in normal"
    );
    let sel = crate::diag_level::detect_level_selection();
    let lines = crate::diag_level::format_level_init_log_lines(
        &sel,
        crate::diag_level::effective_input_registered(),
        false,
        false,
    );
    assert!(
        lines.iter().any(|l| l == "diagnostic input_registered=false"),
        "sidecar must log input_registered=false: {lines:?}"
    );
    crate::safe_mem::set_test_enable_dir(None);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn normal_default_off_without_input_disable_registers_input() {
    let _g = TestResolverStateGuard::acquire();
    let dir = temp_dir("normal-input-on");
    std::fs::write(dir.join("truckpilot_diag.normal_default_off"), b"").expect("normal");
    crate::safe_mem::set_test_enable_dir(Some(dir.clone()));
    assert_eq!(crate::diag_level::active(), DiagLevel::Normal);
    assert!(!crate::safe_mem::input_plugin_disabled());
    assert!(crate::diag_level::effective_input_registered());
    let sel = crate::diag_level::detect_level_selection();
    let lines = crate::diag_level::format_level_init_log_lines(
        &sel,
        crate::diag_level::effective_input_registered(),
        false,
        false,
    );
    assert!(lines.iter().any(|l| l == "diagnostic input_registered=true"));
    crate::safe_mem::set_test_enable_dir(None);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn diag_level_with_input_disable_still_reports_input_not_registered() {
    let _g = TestResolverStateGuard::acquire();
    // A diag level never registers input regardless of the disable file.
    let dir = temp_dir("diag-input");
    std::fs::write(dir.join("truckpilot_diag.callback_noop"), b"").expect("diag");
    crate::safe_mem::set_test_enable_dir(Some(dir.clone()));
    assert_eq!(crate::diag_level::active(), DiagLevel::CallbackNoop);
    assert!(!crate::diag_level::effective_input_registered());
    crate::safe_mem::set_test_enable_dir(None);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn normal_default_off_keeps_known_good_baseline() {
    let _g = TestResolverStateGuard::acquire();
    // Explicit normal_default_off marker behaves like the established default/off:
    // resolver off, dispatch hard-off gate, no worker wake.
    let dir = temp_dir("normal");
    std::fs::write(dir.join("truckpilot_diag.normal_default_off"), b"").expect("normal");
    crate::safe_mem::set_test_enable_dir(Some(dir.clone()));
    assert_eq!(crate::diag_level::active(), DiagLevel::Normal);
    assert!(!crate::diag_level::resolver_forced_off());
    assert!(crate::safe_mem::route_resolver_mode().is_off());
    for i in 0..FRAMES {
        crate::route_dispatch::dispatch_route_tick(SCS_EVENT_FRAME_END, i as u64);
    }
    assert_eq!(load(&NOTIFY_FRAME_TICK_COUNT), 0);
    assert_eq!(load(&WORKER_WAKE_SET_EVENT_COUNT), 0);
    assert_eq!(
        crate::resolver_worker::WORKER_WALK_COUNT.load(Ordering::Relaxed),
        0
    );
    assert_eq!(
        crate::resolver_metrics::RESOLVER_WORKER_PATTERN_SCAN_COUNT.load(Ordering::Relaxed),
        0
    );
    // dispatch ran but was fully suppressed by the off gate.
    assert_eq!(
        crate::frame_perf::OFF_MODE_NOTIFY_SUPPRESSED_COUNT.load(Ordering::Relaxed),
        FRAMES
    );
    crate::safe_mem::set_test_enable_dir(None);
    let _ = std::fs::remove_dir_all(&dir);
}
