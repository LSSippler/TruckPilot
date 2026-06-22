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

    const INFINITE: DWORD = 0xFFFF_FFFF;
    const WAIT_OBJECT_0: DWORD = 0;
    const WAIT_TIMEOUT: DWORD = 0x102;

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

/// O(1) frame-path notification — no resolver work.
pub fn notify_frame_tick(timestamp_us: u64, source: RouteTickSource) {
    ROUTE_TICK_COUNT.fetch_add(1, Ordering::Relaxed);
    LAST_NOTIFY_US.store(timestamp_us, Ordering::Release);
    LAST_TICK_SOURCE.store(
        match source {
            RouteTickSource::FrameStartFallback => 1,
            RouteTickSource::FrameEnd => 0,
        },
        Ordering::Relaxed,
    );
    PENDING_RESOLVE.store(true, Ordering::Release);
    FRAME_SCHEDULED_WORKER.store(true, Ordering::Release);
    WORKER_WAKE_COUNT.fetch_add(1, Ordering::Relaxed);
    platform::signal_wake();
    crate::nav_route::write_bb_frame_from_atomics();
}

fn worker_main() {
    while !WORKER_STOP.load(Ordering::Acquire) {
        let _ = platform::wait_wake_or_timeout(WORKER_SLEEP_MS);
        if WORKER_STOP.load(Ordering::Acquire) {
            break;
        }
        if !PENDING_RESOLVE.swap(false, Ordering::AcqRel) {
            continue;
        }
        let ts = LAST_NOTIFY_US.load(Ordering::Acquire);
        WORKER_WALK_COUNT.fetch_add(1, Ordering::Relaxed);
        crate::nav_route::resolver_walk(ts);
    }
}

#[cfg(test)]
pub fn reset_test_counters() {
    FRAME_SCHEDULED_WORKER.store(false, Ordering::Release);
    WORKER_WAKE_COUNT.store(0, Ordering::Release);
    WORKER_WALK_COUNT.store(0, Ordering::Release);
    PENDING_RESOLVE.store(false, Ordering::Release);
    ROUTE_TICK_COUNT.store(0, Ordering::Release);
    FRAME_CB_COUNT.store(0, Ordering::Release);
    FRAME_START_COUNT.store(0, Ordering::Release);
    FRAME_END_COUNT.store(0, Ordering::Release);
    LAST_FRAME_END_US.store(0, Ordering::Release);
    LAST_NOTIFY_US.store(0, Ordering::Release);
    LAST_TICK_SOURCE.store(0, Ordering::Release);
    RESOLVER_RESET_REQUESTED.store(false, Ordering::Release);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn notify_frame_tick_schedules_worker_not_sync_resolver() {
        let _guard = crate::test_isolation::TestResolverStateGuard::acquire();
        notify_frame_tick(1_000_000, RouteTickSource::FrameEnd);
        assert!(FRAME_SCHEDULED_WORKER.load(Ordering::Acquire));
        assert_eq!(ROUTE_TICK_COUNT.load(Ordering::Relaxed), 1);
        assert_eq!(WORKER_WAKE_COUNT.load(Ordering::Relaxed), 1);
        assert_eq!(
            crate::resolver_metrics::FRAME_CALLBACK_SYNC_RESOLVER_CALLS.load(Ordering::Relaxed),
            0
        );
        assert_eq!(
            crate::resolver_metrics::PATTERN_SCANS_FROM_FRAME_CALLBACK.load(Ordering::Relaxed),
            0
        );
    }

    #[test]
    fn worker_loop_uses_sleep_not_busy_spin() {
        assert!(WORKER_SLEEP_MS >= 100);
        assert!(WORKER_SLEEP_MS <= 200);
    }
}
