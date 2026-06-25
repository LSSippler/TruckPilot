//! Rate-limited slow-tick warnings for the daemon loop.

use std::collections::HashMap;
use std::sync::{Mutex, OnceLock};
use std::time::{Duration, Instant};

use tracing::{debug, info, warn};

const PLUGIN_WARN_MS: u128 = 250;
const FULL_TICK_WARN_MS: u128 = 250;
const REPEAT_COOLDOWN: Duration = Duration::from_secs(60);

fn last_warn_map() -> &'static Mutex<HashMap<String, Instant>> {
    static MAP: OnceLock<Mutex<HashMap<String, Instant>>> = OnceLock::new();
    MAP.get_or_init(|| Mutex::new(HashMap::new()))
}

fn should_emit_warn(key: &str) -> bool {
    let now = Instant::now();
    let mut map = last_warn_map().lock().expect("tick_profile map");
    match map.get(key) {
        Some(prev) if now.duration_since(*prev) < REPEAT_COOLDOWN => false,
        _ => {
            map.insert(key.to_string(), now);
            true
        }
    }
}

static LANE_FOLLOWER_LUT_NOTE: OnceLock<Mutex<bool>> = OnceLock::new();

/// Log a slow plugin tick. WARN only above [`PLUGIN_WARN_MS`], rate-limited per plugin.
pub fn maybe_warn_plugin_slow(name: &str, ms: u128, tick_count: u64) {
    if name == "lane-follower" && ms >= 1_000 {
        let note = LANE_FOLLOWER_LUT_NOTE.get_or_init(|| Mutex::new(false));
        let mut seen = note.lock().expect("lane-follower lut note");
        if !*seen {
            *seen = true;
            info!(
                "[tick-profile] plugin 'lane-follower' first LUT/index build took {ms} ms \
                 (tick={tick_count}; expected once per daemon start)"
            );
            return;
        }
    }

    if ms <= PLUGIN_WARN_MS {
        debug!(
            target: "truckpilot_core::tick_profile",
            "[tick-profile] plugin '{name}' took {ms} ms (tick={tick_count})"
        );
        return;
    }

    let key = format!("plugin:{name}");
    if should_emit_warn(&key) {
        warn!(
            "[tick-profile] plugin '{name}' took {ms} ms (tick={tick_count})"
        );
    } else {
        debug!(
            target: "truckpilot_core::tick_profile",
            "[tick-profile] plugin '{name}' took {ms} ms (tick={tick_count})"
        );
    }
}

/// Log a slow full daemon tick. WARN only above [`FULL_TICK_WARN_MS`], rate-limited.
pub fn maybe_warn_full_tick(ms: u128, state: &str, plugins_count: usize) {
    if ms <= FULL_TICK_WARN_MS {
        debug!(
            target: "truckpilot_core::tick_profile",
            "[tick-profile] FULL TICK took {ms} ms (state={state}, plugins_count={plugins_count})"
        );
        return;
    }

    let key = format!("full:{state}");
    if should_emit_warn(&key) {
        warn!(
            "[tick-profile] FULL TICK took {ms} ms (state={state}, plugins_count={plugins_count})"
        );
    } else {
        debug!(
            target: "truckpilot_core::tick_profile",
            "[tick-profile] FULL TICK took {ms} ms (state={state}, plugins_count={plugins_count})"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thresholds_are_sane() {
        assert!(PLUGIN_WARN_MS >= 100);
        assert!(FULL_TICK_WARN_MS >= 100);
    }
}
