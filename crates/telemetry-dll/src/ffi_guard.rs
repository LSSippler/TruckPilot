//! Panic containment at the C ABI boundary (SCS SDK exports and callbacks).
//!
//! Unwinding across `extern "system"` is undefined behaviour; all exported entry
//! points and SDK callbacks must route through these helpers.

use std::panic::{catch_unwind, AssertUnwindSafe};

type ScsResult = i32;

pub const SCS_RESULT_OK: ScsResult = 0;
/// Generic resource / internal failure (matches telemetry init conventions).
pub const SCS_RESULT_GENERIC_ERROR: ScsResult = -7;

/// Whether telemetry init should succeed when optional RouteBlackboard setup fails.
pub fn telemetry_init_should_succeed(telemetry_shm_ok: bool, _route_bb_ok: bool) -> ScsResult {
    if telemetry_shm_ok {
        SCS_RESULT_OK
    } else {
        SCS_RESULT_GENERIC_ERROR
    }
}

fn panic_message(payload: Box<dyn std::any::Any + Send>) -> String {
    if let Some(s) = payload.downcast_ref::<&str>() {
        (*s).to_string()
    } else if let Some(s) = payload.downcast_ref::<String>() {
        s.clone()
    } else {
        "unknown panic payload".into()
    }
}

fn log_panic(label: &str, payload: Box<dyn std::any::Any + Send>) {
    let msg = panic_message(payload);
    crate::diag_log::event_force(&format!("PANIC in {label}: {msg}"));
}

/// Run an init/export function; on panic log and return a safe error code.
pub fn guard_result<F>(label: &str, f: F) -> ScsResult
where
    F: FnOnce() -> ScsResult + std::panic::UnwindSafe,
{
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(code) => code,
        Err(payload) => {
            log_panic(label, payload);
            SCS_RESULT_GENERIC_ERROR
        }
    }
}

/// Run a shutdown/export with no return value.
pub fn guard_void<F>(label: &str, f: F)
where
    F: FnOnce() + std::panic::UnwindSafe,
{
    if let Err(payload) = catch_unwind(AssertUnwindSafe(f)) {
        log_panic(label, payload);
    }
}

/// Run a telemetry/input callback body; swallow panics after logging.
pub fn catch_callback<F>(label: &str, f: F)
where
    F: FnOnce() + std::panic::UnwindSafe,
{
    if let Err(payload) = catch_unwind(AssertUnwindSafe(f)) {
        log_panic(label, payload);
    }
}

/// Like [`catch_callback`] but returns a fallback result when the body panics.
pub fn catch_callback_result<F>(label: &str, fallback: ScsResult, f: F) -> ScsResult
where
    F: FnOnce() -> ScsResult + std::panic::UnwindSafe,
{
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(r) => r,
        Err(payload) => {
            log_panic(label, payload);
            fallback
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn telemetry_init_ok_when_route_bb_fails() {
        assert_eq!(telemetry_init_should_succeed(true, false), SCS_RESULT_OK);
        assert_eq!(telemetry_init_should_succeed(true, true), SCS_RESULT_OK);
        assert_eq!(
            telemetry_init_should_succeed(false, true),
            SCS_RESULT_GENERIC_ERROR
        );
    }

    #[test]
    fn guard_result_catches_panic() {
        let code = guard_result("test_panic", || {
            panic!("boom");
        });
        assert_eq!(code, SCS_RESULT_GENERIC_ERROR);
    }

    #[test]
    fn guard_result_passes_through_ok() {
        let code = guard_result("test_ok", || SCS_RESULT_OK);
        assert_eq!(code, SCS_RESULT_OK);
    }

    #[test]
    fn catch_callback_result_uses_fallback_on_panic() {
        let code = catch_callback_result("cb", -99, || panic!("cb boom"));
        assert_eq!(code, -99);
    }
}
