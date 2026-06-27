//! Sign-Reader plugin — map-data-based speed limit source.
//!
//! On load, reads all `SpeedLimit` signs from the `MapGraph` that was
//! previously built by the map-parser pipeline and stored in `graph.json`.
//!
//! Every tick the plugin:
//! 1. Reads the truck's current position from telemetry.
//! 2. Finds all `SpeedLimit` signs within `LOOKAHEAD_M` meters ahead.
//! 3. Writes the lowest applicable limit to `sign.speed_limit_kmh`.
//! 4. Clears the key when no sign is in range.
//!
//! ## Blackboard contract
//!
//! | Key                    | Written by  | Read by          |
//! |------------------------|-------------|------------------|
//! | `sign.speed_limit_kmh` | sign-reader | speed-controller |
//! | `sign.source`          | sign-reader | UI / dashboard   |

use std::path::{Path, PathBuf};
use std::thread::JoinHandle;
use std::time::Instant;

use truckpilot_map_parser::signs::{SignKind, TrafficSign};
use truckpilot_plugin_api::{ControlOutput, Plugin, PluginContext, Telemetry, TickPhase};

/// How far ahead (meters) to scan for speed-limit signs.
const LOOKAHEAD_M: f64 = 200.0;

/// Default path to the pre-built graph JSON.
const DEFAULT_GRAPH_PATH: &str = "graph.json";

pub struct SignReaderPlugin {
    signs: Vec<TrafficSign>,
    graph_path: PathBuf,
    loaded: bool,
    /// Set when the background warmup could not be started or did not finish
    /// cleanly (spawn error or thread panic). The plugin then becomes a
    /// permanent no-op — it never falls back to a synchronous load on the
    /// daemon's hot path. Safe-Off semantics: no signs are published, so
    /// speed-controller uses its own fallback; no steering is ever touched.
    unavailable: bool,
    /// Handle to the background sign-loading thread spawned in `on_load`.
    /// `Some` while the warmup is in flight; polled each tick via the
    /// non-blocking [`JoinHandle::is_finished`] and joined (cheap, already
    /// finished) once ready. Yields the loaded signs plus the thread's
    /// measured work time in ms. `JoinHandle` is `Send + Sync`, so the
    /// plugin stays object-safe for the host's `Plugin: Send + Sync` bound.
    warmup_handle: Option<JoinHandle<(Vec<TrafficSign>, u128)>>,
}

impl Default for SignReaderPlugin {
    fn default() -> Self {
        Self {
            signs: Vec::new(),
            graph_path: PathBuf::from(DEFAULT_GRAPH_PATH),
            loaded: false,
            unavailable: false,
            warmup_handle: None,
        }
    }
}

/// Collapse whitespace in an error string to underscores so it stays a single
/// `key=value` token on the space-separated `startup.phase=...` log line.
fn sanitize_error(msg: &str) -> String {
    msg.split_whitespace().collect::<Vec<_>>().join("_")
}

/// Read + parse `graph.json` and extract the SpeedLimit signs attached to a
/// node. Pure function so it can run on a background thread (owns its path)
/// and be unit-tested without a plugin instance. Returns an empty vec on any
/// I/O or parse error (logged), never panics.
fn load_signs_from_path(graph_path: &Path) -> Vec<TrafficSign> {
    let data = match std::fs::read_to_string(graph_path) {
        Ok(d) => d,
        Err(e) => {
            tracing::warn!("[sign-reader] cannot read {:?}: {e}", graph_path);
            return Vec::new();
        }
    };

    let graph: truckpilot_map_parser::MapGraph = match serde_json::from_str(&data) {
        Ok(g) => g,
        Err(e) => {
            tracing::warn!("[sign-reader] cannot parse graph JSON: {e}");
            return Vec::new();
        }
    };

    // Keep only SpeedLimit signs that are attached to a node.
    let signs: Vec<TrafficSign> = graph
        .signs
        .into_iter()
        .filter(|s| s.kind == SignKind::SpeedLimit && s.nearest_node_uid.is_some())
        .collect();

    tracing::info!(
        "[sign-reader] loaded {} speed-limit signs from {:?}",
        signs.len(),
        graph_path
    );
    signs
}

impl SignReaderPlugin {
    /// Find the lowest speed limit within `LOOKAHEAD_M` meters of `(tx, tz)`.
    fn lowest_limit(&self, tx: f64, tz: f64) -> Option<f32> {
        self.signs
            .iter()
            .filter_map(|s| {
                let dx = s.x - tx;
                let dz = s.z - tz;
                let dist = (dx * dx + dz * dz).sqrt();
                if dist <= LOOKAHEAD_M && s.value > 0.0 {
                    Some((dist, s.value))
                } else {
                    None
                }
            })
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .map(|(_, v)| v)
    }
}

impl Plugin for SignReaderPlugin {
    fn name(&self) -> &str {
        "sign-reader"
    }
    fn version(&self) -> &str {
        "0.1.0"
    }
    fn settings_schema(&self) -> &str {
        r#"{
  "type": "object",
  "properties": {
    "graph_path": {
      "type": "string",
      "description": "Path to graph.json produced by the map-parser pipeline."
    },
    "lookahead_m": {
      "type": "number",
      "minimum": 10,
      "maximum": 1000,
      "description": "How far ahead (m) to scan for speed-limit signs."
    }
  }
}"#
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        // Allow overriding the graph path via blackboard (set by core/config).
        if let Some(p) = ctx.blackboard.get("sign_reader.graph_path") {
            self.graph_path = PathBuf::from(p);
        }

        // Reading + parsing the full graph.json (the same multi-hundred-MB file
        // the daemon already loaded) takes ~13 s of pure I/O + serde. Doing it
        // here blocked daemon startup and stuttered ETS2. Move it to a detached
        // background thread; the plugin returns instantly and `tick` picks up
        // the signs once the warmup thread finishes. Until then the plugin is a
        // no-op (it only publishes speed-limit hints, never steering — Safe-Off
        // stays safe).
        let path = self.graph_path.clone();
        match std::thread::Builder::new()
            .name("sign-reader-warmup".into())
            .spawn(move || {
                let t0 = Instant::now();
                let signs = load_signs_from_path(&path);
                (signs, t0.elapsed().as_millis())
            }) {
            Ok(handle) => {
                self.warmup_handle = Some(handle);
                eprintln!("startup.phase=plugin_deferred name=sign-reader");
            }
            Err(e) => {
                // Thread spawn failed. Do NOT load synchronously on the daemon's
                // start path — mark the plugin unavailable and stay a no-op.
                self.unavailable = true;
                tracing::error!(
                    "[sign-reader] warmup thread spawn failed: {e}; plugin unavailable (no-op)"
                );
                eprintln!(
                    "startup.phase=plugin_warmup_failed name=sign-reader error=spawn_failed:{}",
                    sanitize_error(&e.to_string())
                );
            }
        }
        tracing::info!("[sign-reader] loaded (signs warming up in background)");
    }

    fn on_unload(&mut self) {
        // Drop the handle without joining: joining would block on the ~13 s
        // load. The detached thread finishes its work and exits on its own;
        // its result is simply discarded.
        self.warmup_handle = None;
        tracing::info!("[sign-reader] unloaded");
    }

    fn default_phase(&self) -> TickPhase {
        TickPhase::PhaseB
    }

    fn tick(
        &mut self,
        telemetry: Option<&Telemetry>,
        _output: &mut ControlOutput,
        ctx: &PluginContext,
    ) {
        // Permanent no-op once the sign source is known unavailable (warmup
        // could not start or panicked). Never loads synchronously here.
        if self.unavailable {
            ctx.blackboard.remove("sign.speed_limit_kmh");
            ctx.blackboard.remove("sign.source");
            return;
        }

        // Pick up the background warmup result when it lands. Until then the
        // plugin stays a no-op so the hot path is never blocked by the load.
        // No code path here ever loads signs synchronously.
        if !self.loaded {
            // Scope the immutable borrow so the later `take()` is conflict-free.
            // `is_finished` is non-blocking — we only join once it is done, so
            // the join returns immediately.
            let ready = match self.warmup_handle.as_ref() {
                Some(h) => h.is_finished(),
                None => {
                    // No warmup in flight and nothing loaded (e.g. a bare
                    // instance) — publish nothing, never block.
                    ctx.blackboard.remove("sign.speed_limit_kmh");
                    ctx.blackboard.remove("sign.source");
                    return;
                }
            };
            if !ready {
                // Still warming up — publish nothing this tick.
                ctx.blackboard.remove("sign.speed_limit_kmh");
                ctx.blackboard.remove("sign.source");
                return;
            }
            let handle = self.warmup_handle.take().unwrap();
            match handle.join() {
                Ok((signs, elapsed_ms)) => {
                    self.signs = signs;
                    self.loaded = true;
                    eprintln!(
                        "startup.phase=plugin_warmup_done name=sign-reader elapsed_ms={elapsed_ms}"
                    );
                }
                Err(_) => {
                    // Warmup thread panicked: mark unavailable and stay a no-op.
                    // Do NOT load synchronously.
                    self.unavailable = true;
                    tracing::error!(
                        "[sign-reader] warmup thread panicked; plugin unavailable (no-op)"
                    );
                    eprintln!(
                        "startup.phase=plugin_warmup_failed name=sign-reader error=thread_panicked"
                    );
                    ctx.blackboard.remove("sign.speed_limit_kmh");
                    ctx.blackboard.remove("sign.source");
                    return;
                }
            }
        }

        let Some(t) = telemetry else {
            ctx.blackboard.remove("sign.speed_limit_kmh");
            ctx.blackboard.remove("sign.source");
            return;
        };

        let tx = t.position[0];
        let tz = t.position[2];

        match self.lowest_limit(tx, tz) {
            Some(limit_kmh) => {
                ctx.blackboard
                    .set("sign.speed_limit_kmh", limit_kmh.to_string());
                ctx.blackboard.set("sign.source", "map");
                tracing::debug!("[sign-reader] limit={limit_kmh:.0} km/h");
            }
            None => {
                ctx.blackboard.remove("sign.speed_limit_kmh");
                ctx.blackboard.remove("sign.source");
            }
        }
    }
}

truckpilot_plugin_api::export_plugin!(SignReaderPlugin);

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use truckpilot_map_parser::signs::{SignKind, TrafficSign};

    fn make_plugin_with_signs(signs: Vec<TrafficSign>) -> SignReaderPlugin {
        SignReaderPlugin {
            signs,
            graph_path: PathBuf::from("nonexistent.json"),
            loaded: true,
            unavailable: false,
            warmup_handle: None,
        }
    }

    fn speed_sign(x: f64, z: f64, value: f32) -> TrafficSign {
        TrafficSign {
            uid: 1,
            kind: SignKind::SpeedLimit,
            x,
            y: 0.0,
            z,
            value,
            nearest_node_uid: Some(1),
        }
    }

    #[test]
    fn no_signs_returns_none() {
        let p = make_plugin_with_signs(vec![]);
        assert!(p.lowest_limit(0.0, 0.0).is_none());
    }

    #[test]
    fn sign_within_range_returned() {
        let p = make_plugin_with_signs(vec![speed_sign(50.0, 0.0, 80.0)]);
        let limit = p.lowest_limit(0.0, 0.0);
        assert_eq!(limit, Some(80.0));
    }

    #[test]
    fn sign_out_of_range_ignored() {
        let p = make_plugin_with_signs(vec![speed_sign(500.0, 0.0, 80.0)]);
        assert!(p.lowest_limit(0.0, 0.0).is_none());
    }

    #[test]
    fn lowest_limit_returns_min_speed_not_nearest() {
        // 120@20m (nearest) + 80@50m (further but lower) → must return 80.
        let p = make_plugin_with_signs(vec![
            speed_sign(20.0, 0.0, 120.0), // nearer, higher
            speed_sign(50.0, 0.0, 80.0),  // farther, lower
        ]);
        assert_eq!(p.lowest_limit(0.0, 0.0), Some(80.0));
    }

    #[test]
    fn zero_value_sign_ignored() {
        let p = make_plugin_with_signs(vec![speed_sign(10.0, 0.0, 0.0)]);
        assert!(p.lowest_limit(0.0, 0.0).is_none());
    }

    #[test]
    fn missing_graph_file_does_not_panic() {
        // The loader is a pure function now; a missing file yields no signs
        // and never panics. No synchronous load path remains on the plugin.
        let signs = load_signs_from_path(Path::new("nonexistent.json"));
        assert!(signs.is_empty());
    }

    #[test]
    fn sanitize_error_collapses_whitespace() {
        assert_eq!(sanitize_error("a b\tc\nd"), "a_b_c_d");
    }
}
