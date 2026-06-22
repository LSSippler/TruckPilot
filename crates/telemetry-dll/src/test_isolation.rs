//! Serialize unit tests that touch process-wide resolver worker atomics and metrics.

use std::sync::{Mutex, MutexGuard};

static RESOLVER_TEST_STATE_LOCK: Mutex<()> = Mutex::new(());

/// RAII guard: holds the global resolver test lock and resets shared state on acquire.
pub struct TestResolverStateGuard {
    _lock: MutexGuard<'static, ()>,
}

impl TestResolverStateGuard {
    /// Acquire the global lock and reset all shared resolver test state.
    pub fn acquire() -> Self {
        let lock = lock_or_recover(&RESOLVER_TEST_STATE_LOCK);
        reset_all_resolver_test_state();
        Self { _lock: lock }
    }
}

fn lock_or_recover<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    match mutex.lock() {
        Ok(guard) => guard,
        Err(poisoned) => poisoned.into_inner(),
    }
}

/// Reset counters, metrics, enable-dir override, and session cache before/after isolated tests.
pub fn reset_all_resolver_test_state() {
    crate::resolver_metrics::reset_test_metrics();
    crate::resolver_worker::reset_test_counters();
    crate::safe_mem::set_test_enable_dir(None);
    crate::nav_resolve::invalidate_session_cache();
}
