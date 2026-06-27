//! One-shot daemon startup timing (stderr, no --verbose required).

use std::sync::{Mutex, OnceLock};
use std::time::Instant;

static T0: OnceLock<Instant> = OnceLock::new();
static LAST: Mutex<Option<Instant>> = Mutex::new(None);

/// Reset the startup clock at the beginning of [`crate::run_daemon`].
pub fn begin() {
    let now = Instant::now();
    let _ = T0.set(now);
    *LAST.lock().expect("startup_trace LAST poisoned") = Some(now);
}

/// Log `startup.phase=<name> elapsed_ms=<since begin> delta_ms=<since last phase>`.
pub fn phase(name: &str) {
    let Some(t0) = T0.get() else {
        return;
    };
    let now = Instant::now();
    let elapsed_ms = now.duration_since(*t0).as_millis();
    let mut last = LAST.lock().expect("startup_trace LAST poisoned");
    let delta_ms = last
        .map(|l| now.duration_since(l).as_millis())
        .unwrap_or(0);
    eprintln!("startup.phase={name} elapsed_ms={elapsed_ms} delta_ms={delta_ms}");
    *last = Some(now);
}
