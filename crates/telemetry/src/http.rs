//! HTTP fallback — polls the Funbit ETS2 Telemetry Server.
//!
//! Persistent `ureq::Agent` is reused across calls for connection pooling.

use std::time::Duration;

use serde::Deserialize;
use truckpilot_plugin_api::Telemetry;

/// Persistent HTTP telemetry source.
pub struct HttpReader {
    url: String,
    agent: ureq::Agent,
}

impl HttpReader {
    /// Construct a reader pointing at the given URL (typically
    /// `http://localhost:25555/api/ets2/telemetry`).
    pub fn new(url: String) -> Self {
        let agent = ureq::AgentBuilder::new()
            .timeout_connect(Duration::from_millis(500))
            .timeout_read(Duration::from_millis(800))
            .build();
        Self { url, agent }
    }

    /// Fetch one telemetry frame. Returns `None` on any error
    /// (timeout, connection refused, parse error).
    pub fn read(&mut self) -> Option<Telemetry> {
        let body = self.agent.get(&self.url).call().ok()?.into_string().ok()?;
        let raw: FunbitTelemetry = serde_json::from_str(&body).ok()?;
        Some(raw.into())
    }
}

#[derive(Debug, Deserialize)]
struct FunbitTelemetry {
    #[serde(rename = "truckPlacement")]
    truck_placement: TruckPlacement,
    #[serde(rename = "truckFloatValues")]
    truck_float_values: TruckFloatValues,
    #[serde(rename = "navigationSpeedLimit")]
    #[serde(default)]
    navigation_speed_limit: Option<f64>,
}

#[derive(Debug, Deserialize)]
struct TruckPlacement {
    x: f64,
    y: f64,
    z: f64,
    heading: f64,
    #[serde(default)]
    pitch: f64,
    #[serde(default)]
    roll: f64,
}

#[derive(Debug, Deserialize)]
struct TruckFloatValues {
    #[serde(default)]
    speed: f64,
    #[serde(default, rename = "engineRpm")]
    engine_rpm: f64,
    #[serde(default, rename = "cruiseControlSpeed")]
    cruise_control_speed: f64,
}

impl From<FunbitTelemetry> for Telemetry {
    fn from(f: FunbitTelemetry) -> Self {
        Telemetry {
            position: [
                f.truck_placement.x,
                f.truck_placement.y,
                f.truck_placement.z,
            ],
            heading: f.truck_placement.heading,
            pitch: f.truck_placement.pitch,
            roll: f.truck_placement.roll,
            speed_ms: f.truck_float_values.speed,
            engine_rpm: f.truck_float_values.engine_rpm,
            cruise_control_kmh: f.truck_float_values.cruise_control_speed,
            nav_speed_limit_kmh: f.navigation_speed_limit.unwrap_or(-1.0),
            lead_vehicle_distance_m: -1.0,
            accel_longitudinal: -1.0,
            // Funbit JSON does not expose fuel/odometer; surface as
            // sentinel so plugins fall back to their own defaults.
            fuel_liters: -1.0,
            odometer_km: -1.0,
            nav_distance_m: -1.0,
            nav_time_s: -1.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_minimal_funbit_json() {
        let json = r#"{
            "truckPlacement": { "x": 1.0, "y": 2.0, "z": 3.0, "heading": 1.5 },
            "truckFloatValues": { "speed": 22.0 },
            "navigationSpeedLimit": 80.0
        }"#;
        let raw: FunbitTelemetry = serde_json::from_str(json).unwrap();
        let t: Telemetry = raw.into();
        assert_eq!(t.position, [1.0, 2.0, 3.0]);
        assert_eq!(t.heading, 1.5);
        assert_eq!(t.speed_ms, 22.0);
        assert_eq!(t.nav_speed_limit_kmh, 80.0);
    }

    #[test]
    fn missing_nav_limit_becomes_sentinel() {
        let json = r#"{
            "truckPlacement": { "x": 0.0, "y": 0.0, "z": 0.0, "heading": 0.0 },
            "truckFloatValues": { "speed": 0.0 }
        }"#;
        let raw: FunbitTelemetry = serde_json::from_str(json).unwrap();
        let t: Telemetry = raw.into();
        assert_eq!(t.nav_speed_limit_kmh, -1.0);
    }

    #[test]
    fn fuel_and_odometer_become_sentinel() {
        let json = r#"{
            "truckPlacement": { "x": 0.0, "y": 0.0, "z": 0.0, "heading": 0.0 },
            "truckFloatValues": { "speed": 0.0 }
        }"#;
        let raw: FunbitTelemetry = serde_json::from_str(json).unwrap();
        let t: Telemetry = raw.into();
        assert_eq!(t.fuel_liters, -1.0);
        assert_eq!(t.odometer_km, -1.0);
    }
}
