//! Testable counters for resolver performance / safety regression guards.

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

/// Synchronous resolver calls from the SCS frame callback (must stay zero).
pub static FRAME_CALLBACK_SYNC_RESOLVER_CALLS: AtomicU32 = AtomicU32::new(0);
/// Pattern scans triggered from the frame callback (must stay zero).
pub static PATTERN_SCANS_FROM_FRAME_CALLBACK: AtomicU32 = AtomicU32::new(0);
/// Full `.text` pattern scans performed on the worker thread.
pub static RESOLVER_WORKER_PATTERN_SCAN_COUNT: AtomicU32 = AtomicU32::new(0);
/// Diagnostic pointer-table runs (gps/game_ctrl/candidate tables).
pub static DIAGNOSTIC_TABLE_RUNS: AtomicU32 = AtomicU32::new(0);
/// Resolver is parked (off mode, retry limit, or diagnostic done).
pub static RESOLVER_PARKED: AtomicBool = AtomicBool::new(false);
/// Blocked calls into resolver/diagnostic while mode is off (must stay zero in production default).
pub static OFF_MODE_BLOCKED_CALLS: AtomicU32 = AtomicU32::new(0);
/// Worker walks that passed the off-mode gate and may perform resolver work.
pub static RESOLVER_WALK_PROCEEDED: AtomicU32 = AtomicU32::new(0);

pub fn note_worker_pattern_scan() {
    RESOLVER_WORKER_PATTERN_SCAN_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_diagnostic_table_run() {
    DIAGNOSTIC_TABLE_RUNS.fetch_add(1, Ordering::Relaxed);
}

pub fn note_off_mode_blocked_call() {
    OFF_MODE_BLOCKED_CALLS.fetch_add(1, Ordering::Relaxed);
}

pub fn note_resolver_walk_proceeded() {
    RESOLVER_WALK_PROCEEDED.fetch_add(1, Ordering::Relaxed);
}

pub fn set_resolver_parked(parked: bool) {
    RESOLVER_PARKED.store(parked, Ordering::Relaxed);
}

#[cfg(test)]
pub fn reset_test_metrics() {
    FRAME_CALLBACK_SYNC_RESOLVER_CALLS.store(0, Ordering::Release);
    PATTERN_SCANS_FROM_FRAME_CALLBACK.store(0, Ordering::Release);
    RESOLVER_WORKER_PATTERN_SCAN_COUNT.store(0, Ordering::Release);
    DIAGNOSTIC_TABLE_RUNS.store(0, Ordering::Release);
    RESOLVER_PARKED.store(false, Ordering::Release);
    OFF_MODE_BLOCKED_CALLS.store(0, Ordering::Release);
    RESOLVER_WALK_PROCEEDED.store(0, Ordering::Release);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_isolation::TestResolverStateGuard;

    #[test]
    fn frame_callback_metrics_start_at_zero() {
        let _guard = TestResolverStateGuard::acquire();
        assert_eq!(FRAME_CALLBACK_SYNC_RESOLVER_CALLS.load(Ordering::Relaxed), 0);
        assert_eq!(PATTERN_SCANS_FROM_FRAME_CALLBACK.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn worker_pattern_scan_counter_increments() {
        let _guard = TestResolverStateGuard::acquire();
        note_worker_pattern_scan();
        assert_eq!(RESOLVER_WORKER_PATTERN_SCAN_COUNT.load(Ordering::Relaxed), 1);
    }
}
