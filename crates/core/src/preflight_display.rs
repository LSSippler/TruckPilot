//! Display-only SHM mirrors for overlay preflight.
//!
//! Reads existing route/perf SHM via `truckpilot-telemetry` — does not activate
//! the resolver or touch ETS2 process memory.

use truckpilot_plugin_api::SharedBlackboard;
use truckpilot_telemetry::resolver_safe::PREFLIGHT_RESOLVER_SAFE_KEY;
use truckpilot_telemetry::status_report::read_live_status;

/// Refresh display-only preflight keys from SHM (each daemon tick).
pub fn refresh_shm_display_keys(bb: &SharedBlackboard) {
    let status = read_live_status();
    match status.preflight.resolver_safe {
        Some(true) => bb.set(PREFLIGHT_RESOLVER_SAFE_KEY, "true"),
        Some(false) => bb.set(PREFLIGHT_RESOLVER_SAFE_KEY, "false"),
        None => bb.remove(PREFLIGHT_RESOLVER_SAFE_KEY),
    }
}
