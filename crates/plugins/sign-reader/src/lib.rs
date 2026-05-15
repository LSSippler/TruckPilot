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

use std::path::PathBuf;

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
}

impl Default for SignReaderPlugin {
    fn default() -> Self {
        Self {
            signs: Vec::new(),
            graph_path: PathBuf::from(DEFAULT_GRAPH_PATH),
            loaded: false,
        }
    }
}

impl SignReaderPlugin {
    /// Load signs from the graph JSON file.
    fn load_signs(&mut self) {
        let data = match std::fs::read_to_string(&self.graph_path) {
            Ok(d) => d,
            Err(e) => {
                tracing::warn!("[sign-reader] cannot read {:?}: {e}", self.graph_path);
                return;
            }
        };

        let graph: truckpilot_map_parser::MapGraph = match serde_json::from_str(&data) {
            Ok(g) => g,
            Err(e) => {
                tracing::warn!("[sign-reader] cannot parse graph JSON: {e}");
                return;
            }
        };

        // Keep only SpeedLimit signs that are attached to a node.
        self.signs = graph
            .signs
            .into_iter()
            .filter(|s| s.kind == SignKind::SpeedLimit && s.nearest_node_uid.is_some())
            .collect();

        tracing::info!(
            "[sign-reader] loaded {} speed-limit signs from {:?}",
            self.signs.len(),
            self.graph_path
        );
        self.loaded = true;
    }

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
        self.load_signs();
        tracing::info!("[sign-reader] loaded");
    }

    fn on_unload(&mut self) {
        tracing::info!("[sign-reader] unloaded");
    }

    fn default_phase(&self) -> TickPhase { TickPhase::PhaseB }

    fn tick(
        &mut self,
        telemetry: Option<&Telemetry>,
        _output: &mut ControlOutput,
        ctx: &PluginContext,
    ) {
        // Lazy-load on first tick if on_load didn't find the file yet.
        if !self.loaded {
            self.load_signs();
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
        let mut p = SignReaderPlugin::default();
        p.load_signs(); // file doesn't exist → warn, no panic
        assert!(p.signs.is_empty());
    }
}
