//! Fuel-Stops plugin — monitors fuel level and plans refuelling stops.
//!
//! When fuel drops below `low_fuel_threshold_liters`, the plugin:
//! 1. Searches for the nearest fuel station in the `MapGraph`.
//! 2. Writes the station's node UID to `fuel_stop.target_node_uid`.
//! 3. The router plugin reads this and re-routes via the station.
//!
//! ## Blackboard contract
//!
//! | Key                          | Written by  | Read by |
//! |------------------------------|-------------|---------|
//! | `fuel_stop.needed`           | fuel-stops  | UI      |
//! | `fuel_stop.target_node_uid`  | fuel-stops  | router  |
//! | `fuel_stop.station_name`     | fuel-stops  | UI      |
//! | `fuel_stop.distance_m`       | fuel-stops  | UI      |

use std::path::PathBuf;

use truckpilot_plugin_api::{ControlOutput, Plugin, PluginContext, Telemetry, TickPhase};

/// Default fuel threshold in liters.
const DEFAULT_LOW_FUEL_L: f64 = 50.0;
/// Default graph path.
const DEFAULT_GRAPH_PATH: &str = "graph.json";

/// A fuel station extracted from the map graph.
#[derive(Debug, Clone)]
pub struct FuelStation {
    pub node_uid: u64,
    pub x: f64,
    pub z: f64,
    pub name: String,
}

pub struct FuelStopsPlugin {
    low_fuel_threshold_l: f64,
    stations: Vec<FuelStation>,
    graph_path: PathBuf,
    loaded: bool,
    /// UID of the currently targeted station (if any).
    active_target: Option<u64>,
}

impl Default for FuelStopsPlugin {
    fn default() -> Self {
        Self {
            low_fuel_threshold_l: DEFAULT_LOW_FUEL_L,
            stations: Vec::new(),
            graph_path: PathBuf::from(DEFAULT_GRAPH_PATH),
            loaded: false,
            active_target: None,
        }
    }
}

impl FuelStopsPlugin {
    fn load_stations(&mut self) {
        // In a full implementation this reads POI data from the MapGraph.
        // For Phase 9 we parse the graph JSON and look for nodes tagged as
        // fuel stations (token "gas_station" in the prefab data).
        // Here we provide a stub that logs and returns empty — the plugin
        // degrades gracefully without crashing.
        let path = &self.graph_path;
        if !path.exists() {
            tracing::debug!(
                "[fuel-stops] graph not found at {:?} — no stations loaded",
                path
            );
            self.loaded = true;
            return;
        }

        // TODO: parse graph.json POI nodes with token "gas_station".
        tracing::info!("[fuel-stops] loaded {} fuel stations", self.stations.len());
        self.loaded = true;
    }

    fn nearest_station(&self, tx: f64, tz: f64) -> Option<&FuelStation> {
        self.stations.iter().min_by_key(|s| {
            let dx = s.x - tx;
            let dz = s.z - tz;
            ((dx * dx + dz * dz).sqrt() * 1000.0) as u64
        })
    }
}

impl Plugin for FuelStopsPlugin {
    fn name(&self) -> &str {
        "fuel-stops"
    }
    fn version(&self) -> &str {
        "0.1.0"
    }
    fn settings_schema(&self) -> &str {
        r#"{
  "type": "object",
  "properties": {
    "low_fuel_threshold_l": {
      "type": "number", "minimum": 10,
      "description": "Fuel level (liters) below which a stop is planned."
    },
    "graph_path": {
      "type": "string",
      "description": "Path to graph.json for station lookup."
    }
  }
}"#
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        if let Some(v) = ctx.blackboard.get_f64("fuel_stops.low_fuel_threshold_l") {
            self.low_fuel_threshold_l = v.max(10.0);
        }
        if let Some(p) = ctx.blackboard.get("fuel_stops.graph_path") {
            self.graph_path = PathBuf::from(p);
        }
        self.load_stations();
        tracing::info!(
            "[fuel-stops] loaded — threshold={:.0}L",
            self.low_fuel_threshold_l
        );
    }

    fn on_unload(&mut self) {
        tracing::info!("[fuel-stops] unloaded");
    }

    fn default_phase(&self) -> TickPhase {
        TickPhase::PhaseA
    }

    fn tick(
        &mut self,
        telemetry: Option<&Telemetry>,
        _output: &mut ControlOutput,
        ctx: &PluginContext,
    ) {
        if !self.loaded {
            self.load_stations();
        }

        let Some(t) = telemetry else {
            ctx.blackboard.set("fuel_stop.needed", "false");
            return;
        };

        // fuel is stored in liters in the Telemetry struct (via SHM).
        // HTTP telemetry doesn't expose fuel — we use a proxy via odometer.
        // For now we read from the blackboard if the SHM plugin wrote it.
        let fuel_l = ctx
            .blackboard
            .get_f64("telemetry.fuel_liters")
            .unwrap_or(f64::MAX);

        if fuel_l > self.low_fuel_threshold_l {
            ctx.blackboard.set("fuel_stop.needed", "false");
            self.active_target = None;
            ctx.blackboard.remove("fuel_stop.target_node_uid");
            ctx.blackboard.remove("fuel_stop.station_name");
            ctx.blackboard.remove("fuel_stop.distance_m");
            return;
        }

        ctx.blackboard.set("fuel_stop.needed", "true");
        tracing::warn!("[fuel-stops] low fuel ({fuel_l:.0}L) — searching for station");

        let tx = t.position[0];
        let tz = t.position[2];

        // Clone to avoid borrow conflict with self.active_target.
        if let Some(station) = self.nearest_station(tx, tz).cloned() {
            let dx = station.x - tx;
            let dz = station.z - tz;
            let dist = (dx * dx + dz * dz).sqrt();

            if self.active_target != Some(station.node_uid) {
                tracing::info!(
                    "[fuel-stops] routing to '{}' ({:.0}m away)",
                    station.name,
                    dist
                );
                self.active_target = Some(station.node_uid);
            }

            ctx.blackboard
                .set("fuel_stop.target_node_uid", station.node_uid.to_string());
            ctx.blackboard
                .set("fuel_stop.station_name", station.name.clone());
            ctx.blackboard.set("fuel_stop.distance_m", dist.to_string());
        } else {
            tracing::warn!("[fuel-stops] no fuel stations in graph — cannot plan stop");
        }
    }
}

truckpilot_plugin_api::export_plugin!(FuelStopsPlugin);

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nearest_station_empty_returns_none() {
        let p = FuelStopsPlugin::default();
        assert!(p.nearest_station(0.0, 0.0).is_none());
    }

    #[test]
    fn nearest_station_picks_closest() {
        let p = FuelStopsPlugin {
            stations: vec![
                FuelStation {
                    node_uid: 1,
                    x: 100.0,
                    z: 0.0,
                    name: "Far".into(),
                },
                FuelStation {
                    node_uid: 2,
                    x: 20.0,
                    z: 0.0,
                    name: "Near".into(),
                },
            ],
            ..Default::default()
        };
        let s = p.nearest_station(0.0, 0.0).unwrap();
        assert_eq!(s.node_uid, 2);
    }

    #[test]
    fn missing_graph_does_not_panic() {
        let mut p = FuelStopsPlugin::default();
        p.load_stations();
        assert!(p.loaded);
        assert!(p.stations.is_empty());
    }

    #[test]
    fn default_threshold_is_reasonable() {
        let p = FuelStopsPlugin::default();
        assert!(p.low_fuel_threshold_l >= 10.0);
        assert!(p.low_fuel_threshold_l <= 500.0);
    }
}
