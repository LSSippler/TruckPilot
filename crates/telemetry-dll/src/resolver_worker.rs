//! Background resolver worker — all expensive route work runs here, not in SCS callbacks.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use crate::route_status::RouteTickSource;

/// Frame-visible counters (updated from SCS callbacks without resolver work).
pub static FRAME_CB_COUNT: AtomicU32 = AtomicU32::new(0);
pub static FRAME_START_COUNT: AtomicU32 = AtomicU32::new(0);
pub static FRAME_END_COUNT: AtomicU32 = AtomicU32::new(0);
pub static ROUTE_TICK_COUNT: AtomicU32 = AtomicU32::new(0);
pub static LAST_FRAME_END_US: AtomicU64 = AtomicU64::new(0);
pub static SESSION_STARTED_US: AtomicU64 = AtomicU64::new(0);
pub static RESOLVER_RESET_REQUESTED: AtomicBool = AtomicBool::new(false);
pub static WORKER_WAKE_COUNT: AtomicU32 = AtomicU32::new(0);
pub static WORKER_WALK_COUNT: AtomicU32 = AtomicU32::new(0);

/// Set when frame path schedules worker; tests assert tick is not called synchronously.
pub static FRAME_SCHEDULED_WORKER: AtomicBool = AtomicBool::new(false);

const WORKER_SLEEP_MS: u64 = 150;

static WORKER_STARTED: AtomicBool = AtomicBool::new(false);
static WORKER_STOP: AtomicBool = AtomicBool::new(false);
static PENDING_RESOLVE: AtomicBool = AtomicBool::new(false);
static LAST_NOTIFY_US: AtomicU64 = AtomicU64::new(0);
static LAST_TICK_SOURCE: AtomicU32 = AtomicU32::new(0);
static mut WORKER_HANDLE: Option<JoinHandle<()>> = None;

pub fn last_tick_source() -> RouteTickSource {
    match LAST_TICK_SOURCE.load(Ordering::Relaxed) {
        1 => RouteTickSource::FrameStartFallback,
        _ => RouteTickSource::FrameEnd,
    }
}

#[cfg(windows)]
mod win {
    use super::*;
    use std::ptr;

    type HANDLE = isize;
    type DWORD = u32;

    const WAIT_OBJECT_0: DWORD = 0;

    extern "system" {
        fn CreateEventW(
            lpEventAttributes: *const core::ffi::c_void,
            bManualReset: i32,
            bInitialState: i32,
            lpName: *const u16,
        ) -> HANDLE;
        fn SetEvent(hEvent: HANDLE) -> i32;
        fn WaitForSingleObject(hHandle: HANDLE, dwMilliseconds: DWORD) -> DWORD;
        fn CloseHandle(hObject: HANDLE) -> i32;
    }

    static mut WAKE_EVENT: HANDLE = 0;

    #[allow(static_mut_refs)]
    pub fn create_wake_event() -> bool {
        unsafe {
            if WAKE_EVENT != 0 {
                return true;
            }
            WAKE_EVENT = CreateEventW(ptr::null(), 0, 0, ptr::null());
            WAKE_EVENT != 0
        }
    }

    #[allow(static_mut_refs)]
    pub fn signal_wake() {
        unsafe {
            if WAKE_EVENT != 0 {
                SetEvent(WAKE_EVENT);
            }
        }
    }

    #[allow(static_mut_refs)]
    pub fn wait_wake_or_timeout(ms: u64) -> bool {
        unsafe {
            if WAKE_EVENT == 0 {
                thread::sleep(Duration::from_millis(ms));
                return PENDING_RESOLVE.load(Ordering::Acquire);
            }
            let r = WaitForSingleObject(WAKE_EVENT, ms.min(u32::MAX as u64) as DWORD);
            r == WAIT_OBJECT_0 || PENDING_RESOLVE.load(Ordering::Acquire)
        }
    }

    #[allow(static_mut_refs)]
    pub fn close_wake_event() {
        unsafe {
            if WAKE_EVENT != 0 {
                CloseHandle(WAKE_EVENT);
                WAKE_EVENT = 0;
            }
        }
    }
}

#[cfg(windows)]
use win as platform;

#[cfg(not(windows))]
mod platform {
    use super::*;

    pub fn create_wake_event() -> bool {
        true
    }
    pub fn signal_wake() {}
    pub fn wait_wake_or_timeout(ms: u64) {
        thread::sleep(Duration::from_millis(ms));
    }
    pub fn close_wake_event() {}
}

#[allow(static_mut_refs)]
pub fn start_worker() {
    if WORKER_STARTED.swap(true, Ordering::SeqCst) {
        return;
    }
    WORKER_STOP.store(false, Ordering::Release);
    platform::create_wake_event();
    let handle = thread::Builder::new()
        .name("truckpilot-route-resolver".into())
        .spawn(worker_main)
        .expect("route resolver worker thread");
    unsafe {
        WORKER_HANDLE = Some(handle);
    }
    crate::diag_log::event_force("route resolver worker started");
}

#[allow(static_mut_refs)]
pub fn stop_worker() {
    WORKER_STOP.store(true, Ordering::Release);
    platform::signal_wake();
    unsafe {
        if let Some(h) = WORKER_HANDLE.take() {
            let _ = h.join();
        }
    }
    platform::close_wake_event();
    WORKER_STARTED.store(false, Ordering::Release);
    crate::diag_log::event_force("route resolver worker stopped");
}

/// Apply route tick atomics without worker notify (off-mode dispatch suppress path).
pub fn apply_route_tick_dispatch(timestamp_us: u64, source: RouteTickSource) {
    ROUTE_TICK_COUNT.fetch_add(1, Ordering::Relaxed);
    LAST_NOTIFY_US.store(timestamp_us, Ordering::Release);
    LAST_TICK_SOURCE.store(
        match source {
            RouteTickSource::FrameStartFallback => 1,
            RouteTickSource::FrameEnd => 0,
        },
        Ordering::Relaxed,
    );
    FRAME_SCHEDULED_WORKER.store(true, Ordering::Release);
}

/// O(1) frame-path notification — no resolver work, no RouteBlackboard write (dispatch owns BB).
pub fn notify_frame_tick(timestamp_us: u64, source: RouteTickSource, force_wake: bool) {
    apply_route_tick_dispatch(timestamp_us, source);
    crate::frame_perf::note_notify_frame_tick();
    if !force_wake && crate::resolver_guard::resolver_is_parked_for_frame_notify() {
        crate::frame_perf::note_worker_parked_no_wake();
        return;
    }
    schedule_worker_wake();
}

fn schedule_worker_wake() {
    if PENDING_RESOLVE.swap(true, Ordering::AcqRel) {
        crate::frame_perf::note_worker_pending_already_set();
    } else {
        WORKER_WAKE_COUNT.fetch_add(1, Ordering::Relaxed);
        crate::frame_perf::note_worker_wake_set_event();
        platform::signal_wake();
    }
}

fn worker_main() {
    while !WORKER_STOP.load(Ordering::Acquire) {
        let _ = platform::wait_wake_or_timeout(WORKER_SLEEP_MS);
        if WORKER_STOP.load(Ordering::Acquire) {
            break;
        }
        let _ = process_one_pending_resolve();
    }
}

/// One worker iteration after wake — coalesces pending ticks, skips when resolver parked.
fn process_one_pending_resolve() -> bool {
    if !PENDING_RESOLVE.swap(false, Ordering::AcqRel) {
        return false;
    }
    if crate::resolver_guard::resolver_is_parked_for_frame_notify() {
        crate::frame_perf::note_worker_parked_skip();
        return false;
    }
    let ts = LAST_NOTIFY_US.load(Ordering::Acquire);
    WORKER_WALK_COUNT.fetch_add(1, Ordering::Relaxed);
    crate::nav_route::resolver_walk(ts);
    true
}

#[cfg(test)]
pub fn clear_pending_resolve_for_test() {
    PENDING_RESOLVE.store(false, Ordering::Release);
}

#[cfg(test)]
pub fn reset_test_counters() {
    FRAME_SCHEDULED_WORKER.store(false, Ordering::Release);
    WORKER_WAKE_COUNT.store(0, Ordering::Release);
    WORKER_WALK_COUNT.store(0, Ordering::Release);
    PENDING_RESOLVE.store(false, Ordering::Release);
    crate::frame_perf::reset_test_counters();
    ROUTE_TICK_COUNT.store(0, Ordering::Release);
    FRAME_CB_COUNT.store(0, Ordering::Release);
    FRAME_START_COUNT.store(0, Ordering::Release);
    FRAME_END_COUNT.store(0, Ordering::Release);
    LAST_FRAME_END_US.store(0, Ordering::Release);
    LAST_NOTIFY_US.store(0, Ordering::Release);
    LAST_TICK_SOURCE.store(0, Ordering::Release);
    RESOLVER_RESET_REQUESTED.store(false, Ordering::Release);
    crate::route_dispatch::reset_dispatch_wake_test_state();
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_probe_enable_dir() -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("tp-active-wake-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp enable dir");
        std::fs::write(
            dir.join("truckpilot_route_resolver.gps_offset_probe"),
            b"",
        )
        .expect("probe enable file");
        dir
    }

    #[test]
    fn notify_frame_tick_schedules_worker_not_sync_resolver() {
        let _guard = crate::test_isolation::TestResolverStateGuard::acquire();
        let dir = temp_probe_enable_dir();
        crate::safe_mem::set_test_enable_dir(Some(dir.clone()));
        assert!(!crate::safe_mem::route_resolver_mode().is_off());
        notify_frame_tick(1_000_000, RouteTickSource::FrameEnd, false);
        assert!(FRAME_SCHEDULED_WORKER.load(Ordering::Acquire));
        assert_eq!(ROUTE_TICK_COUNT.load(Ordering::Relaxed), 1);
        assert_eq!(WORKER_WAKE_COUNT.load(Ordering::Relaxed), 1);
        assert_eq!(
            crate::frame_perf::WORKER_WAKE_SET_EVENT_COUNT.load(Ordering::Relaxed),
            1
        );
        assert_eq!(
            crate::resolver_metrics::FRAME_CALLBACK_SYNC_RESOLVER_CALLS.load(Ordering::Relaxed),
            0
        );
        assert_eq!(
            crate::resolver_metrics::PATTERN_SCANS_FROM_FRAME_CALLBACK.load(Ordering::Relaxed),
            0
        );
        crate::safe_mem::set_test_enable_dir(None);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn wake_coalescing_skips_setevent_when_pending() {
        let _guard = crate::test_isolation::TestResolverStateGuard::acquire();
        let dir = temp_probe_enable_dir();
        crate::safe_mem::set_test_enable_dir(Some(dir.clone()));
        assert!(!crate::safe_mem::route_resolver_mode().is_off());
        notify_frame_tick(1, RouteTickSource::FrameEnd, false);
        assert_eq!(
            crate::frame_perf::WORKER_WAKE_SET_EVENT_COUNT.load(Ordering::Relaxed),
            1
        );
        for _ in 0..999 {
            notify_frame_tick(2, RouteTickSource::FrameEnd, false);
        }
        assert_eq!(
            crate::frame_perf::WORKER_WAKE_SET_EVENT_COUNT.load(Ordering::Relaxed),
            1
        );
        assert_eq!(
            crate::frame_perf::WORKER_PENDING_ALREADY_SET_COUNT.load(Ordering::Relaxed),
            999
        );
        crate::safe_mem::set_test_enable_dir(None);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn parked_resolver_consumes_pending_without_walk() {
        let _guard = crate::test_isolation::TestResolverStateGuard::acquire();
        crate::resolver_metrics::set_resolver_parked(true);
        PENDING_RESOLVE.store(true, Ordering::Release);
        assert!(!process_one_pending_resolve());
        assert_eq!(WORKER_WALK_COUNT.load(Ordering::Relaxed), 0);
        assert_eq!(
            crate::frame_perf::WORKER_PARKED_SKIP_COUNT.load(Ordering::Relaxed),
            1
        );
    }

    #[test]
    fn parked_resolver_skips_worker_wake_on_frame_tick() {
        let _guard = crate::test_isolation::TestResolverStateGuard::acquire();
        crate::resolver_metrics::set_resolver_parked(true);
        for i in 0..1000 {
            notify_frame_tick(i, RouteTickSource::FrameEnd, false);
        }
        assert_eq!(
            crate::frame_perf::WORKER_WAKE_SET_EVENT_COUNT.load(Ordering::Relaxed),
            0
        );
        assert_eq!(
            crate::frame_perf::WORKER_PARKED_NO_WAKE_COUNT.load(Ordering::Relaxed),
            1000
        );
        assert_eq!(WORKER_WAKE_COUNT.load(Ordering::Relaxed), 0);
        assert_eq!(ROUTE_TICK_COUNT.load(Ordering::Relaxed), 1000);
    }

    #[test]
    fn off_mode_live_path_skips_worker_wake_without_parked_atomic() {
        let _guard = crate::test_isolation::TestResolverStateGuard::acquire();
        let dir = std::env::temp_dir().join(format!("tp-off-wake-{}", std::process::id()));
        let _ = std::fs::create_dir_all(&dir);
        crate::safe_mem::set_test_enable_dir(Some(dir.clone()));
        crate::resolver_metrics::set_resolver_parked(false);
        assert!(crate::safe_mem::route_resolver_mode().is_off());
        for i in 0..10_000 {
            crate::route_dispatch::dispatch_route_tick(
                crate::route_status::SCS_EVENT_FRAME_END,
                i,
            );
        }
        assert_eq!(
            crate::frame_perf::NOTIFY_FRAME_TICK_COUNT.load(Ordering::Relaxed),
            0
        );
        assert_eq!(
            crate::frame_perf::WORKER_WAKE_SET_EVENT_COUNT.load(Ordering::Relaxed),
            0
        );
        assert_eq!(
            crate::frame_perf::WORKER_PARKED_SKIP_COUNT.load(Ordering::Relaxed),
            0
        );
        assert_eq!(
            crate::frame_perf::OFF_MODE_NOTIFY_SUPPRESSED_COUNT.load(Ordering::Relaxed),
            10_000
        );
        assert_eq!(WORKER_WAKE_COUNT.load(Ordering::Relaxed), 0);
        assert_eq!(
            crate::resolver_metrics::RESOLVER_WORKER_PATTERN_SCAN_COUNT.load(Ordering::Relaxed),
            0
        );
        crate::safe_mem::set_test_enable_dir(None);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn gps_offset_probe_allows_wake_before_park_not_after() {
        let _guard = crate::test_isolation::TestResolverStateGuard::acquire();
        let dir = std::env::temp_dir().join(format!("tp-probe-wake-{}", std::process::id()));
        let _ = std::fs::create_dir_all(&dir);
        let probe = dir.join("truckpilot_route_resolver.gps_offset_probe");
        std::fs::write(&probe, b"").unwrap();
        crate::safe_mem::set_test_enable_dir(Some(dir.clone()));
        assert!(!crate::safe_mem::route_resolver_mode().is_off());
        crate::route_dispatch::dispatch_route_tick(crate::route_status::SCS_EVENT_FRAME_END, 1);
        assert_eq!(
            crate::frame_perf::NOTIFY_FRAME_TICK_COUNT.load(Ordering::Relaxed),
            1
        );
        assert_eq!(
            crate::frame_perf::WORKER_WAKE_SET_EVENT_COUNT.load(Ordering::Relaxed),
            1
        );
        crate::resolver_metrics::set_resolver_parked(true);
        for i in 0..100 {
            crate::route_dispatch::dispatch_route_tick(
                crate::route_status::SCS_EVENT_FRAME_END,
                i + 2,
            );
        }
        assert_eq!(
            crate::frame_perf::WORKER_WAKE_SET_EVENT_COUNT.load(Ordering::Relaxed),
            1
        );
        assert_eq!(
            crate::frame_perf::WORKER_PARKED_NO_WAKE_COUNT.load(Ordering::Relaxed),
            100
        );
        crate::safe_mem::set_test_enable_dir(None);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn enable_generation_change_allows_single_wake_while_parked() {
        let _guard = crate::test_isolation::TestResolverStateGuard::acquire();
        let dir = temp_probe_enable_dir();
        crate::safe_mem::set_test_enable_dir(Some(dir.clone()));
        crate::resolver_metrics::set_resolver_parked(true);
        crate::route_dispatch::dispatch_route_tick(crate::route_status::SCS_EVENT_FRAME_END, 1);
        assert_eq!(
            crate::frame_perf::WORKER_PARKED_NO_WAKE_COUNT.load(Ordering::Relaxed),
            1
        );
        crate::safe_mem::bump_enable_file_generation();
        crate::route_dispatch::dispatch_route_tick(crate::route_status::SCS_EVENT_FRAME_END, 2);
        assert_eq!(
            crate::frame_perf::WORKER_WAKE_SET_EVENT_COUNT.load(Ordering::Relaxed),
            1
        );
        crate::route_dispatch::dispatch_route_tick(crate::route_status::SCS_EVENT_FRAME_END, 3);
        assert_eq!(
            crate::frame_perf::WORKER_WAKE_SET_EVENT_COUNT.load(Ordering::Relaxed),
            1
        );
        crate::safe_mem::set_test_enable_dir(None);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn parked_no_wake_allows_wake_on_reset_request() {
        let _guard = crate::test_isolation::TestResolverStateGuard::acquire();
        let dir = temp_probe_enable_dir();
        crate::safe_mem::set_test_enable_dir(Some(dir.clone()));
        crate::resolver_metrics::set_resolver_parked(true);
        RESOLVER_RESET_REQUESTED.store(true, Ordering::Release);
        notify_frame_tick(1, RouteTickSource::FrameEnd, true);
        assert_eq!(
            crate::frame_perf::WORKER_WAKE_SET_EVENT_COUNT.load(Ordering::Relaxed),
            1
        );
        assert_eq!(
            crate::frame_perf::WORKER_PARKED_NO_WAKE_COUNT.load(Ordering::Relaxed),
            0
        );
        crate::safe_mem::set_test_enable_dir(None);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[cfg(windows)]
    #[test]
    fn bb_write_dedupe_suppresses_duplicate_same_frame_tick() {
        let _guard = crate::test_isolation::TestResolverStateGuard::acquire();
        if !crate::nav_route::init_shm() {
            return;
        }
        FRAME_CB_COUNT.store(42, Ordering::Release);
        ROUTE_TICK_COUNT.store(7, Ordering::Release);
        crate::nav_route::write_bb_frame_from_atomics();
        crate::nav_route::write_bb_frame_from_atomics();
        assert_eq!(
            crate::frame_perf::ROUTE_BB_FRAME_WRITE_COUNT.load(Ordering::Relaxed),
            1
        );
        assert_eq!(
            crate::frame_perf::ROUTE_BB_FRAME_WRITE_SUPPRESSED_DUPLICATE_COUNT.load(Ordering::Relaxed),
            1
        );
        crate::nav_route::cleanup_shm();
    }

    #[test]
    fn worker_loop_uses_sleep_not_busy_spin() {
        assert!(WORKER_SLEEP_MS >= 100);
        assert!(WORKER_SLEEP_MS <= 200);
    }
}
