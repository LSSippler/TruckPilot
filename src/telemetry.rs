//! Telemetry data fetching from the ETS2 Funbit telemetry server.
//!
//! Parses the REST API JSON provided by the ETS2 Telemetry Server
//! (default: `http://localhost:25555/api/ets2/telemetry`).

use serde::Deserialize;

/// Telemetry snapshot as returned by the ETS2 telemetry REST API.
#[derive(Debug, Clone, Deserialize)]
pub struct TelemetryData {
    /// Truck position and orientation.
    #[serde(rename = "truckPlacement")]
    pub truck_placement: TruckPlacement,
    /// Floating-point telemetry values.
    #[serde(rename = "truckFloatValues")]
    pub truck_float_values: TruckFloatValues,
    /// Navigation speed limit in km/h.
    #[serde(rename = "navigationSpeedLimit")]
    #[serde(default)]
    pub navigation_speed_limit: Option<f64>,
    /// Distance to lead vehicle in meters, if available.
    #[serde(rename = "distanceToLeadVehicle")]
    #[serde(default)]
    pub lead_vehicle_distance_m: Option<f32>,
    /// Local longitudinal acceleration proxy (m/s²), if available.
    #[serde(rename = "localAccelerationLongitudinal")]
    #[serde(default)]
    pub local_acceleration_longitudinal: Option<f32>,
}

/// Position and heading of the truck in world coordinates.
#[derive(Debug, Clone, Deserialize)]
pub struct TruckPlacement {
    /// X coordinate in ETS2 world space.
    pub x: f64,
    /// Y coordinate in ETS2 world space (height).
    pub y: f64,
    /// Z coordinate in ETS2 world space.
    pub z: f64,
    /// Heading angle in radians.
    pub heading: f64,
    /// Pitch angle in radians.
    #[serde(default)]
    pub pitch: f64,
    /// Roll angle in radians.
    #[serde(default)]
    pub roll: f64,
}

/// Floating-point truck values (speed, fuel, etc.).
#[derive(Debug, Clone, Deserialize)]
pub struct TruckFloatValues {
    /// Current speed in m/s (converted from km/h by the server).
    #[serde(default)]
    pub speed: f64,
    /// Engine RPM.
    #[serde(default)]
    pub engine_rpm: f64,
    /// Fuel level in liters.
    #[serde(default)]
    pub fuel: f64,
    /// Odometer reading in km.
    #[serde(default)]
    pub odometer: f64,
    /// Cruise control speed in km/h.
    #[serde(default)]
    pub cruise_control_speed: f64,
}

/// Fetch the current telemetry snapshot from the ETS2 telemetry server.
///
/// Returns `Err` if the server is unreachable or the response cannot be parsed.
pub fn fetch_telemetry(server_url: &str) -> Result<TelemetryData, Box<dyn std::error::Error>> {
    let body = ureq::get(server_url)
        .call()
        .map_err(|e| format!("HTTP request failed: {e}"))?
        .into_body()
        .read_to_string()?;
    let data: TelemetryData = serde_json::from_str(&body)?;
    Ok(data)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_telemetry_json_deserialization() {
        let json = r#"
        {
            "truckPlacement": {
                "x": 12345.6,
                "y": 78.9,
                "z": -1234.5,
                "heading": 1.57,
                "pitch": 0.1,
                "roll": -0.05
            },
            "truckFloatValues": {
                "speed": 25.0,
                "engine_rpm": 1500.0,
                "fuel": 350.5,
                "odometer": 123456.7,
                "cruise_control_speed": 80.0
            },
            "navigationSpeedLimit": 90.0
        }
        "#;
        let data: TelemetryData = serde_json::from_str(json).unwrap();
        assert_eq!(data.truck_placement.x, 12345.6);
        assert_eq!(data.truck_placement.y, 78.9);
        assert_eq!(data.truck_placement.z, -1234.5);
        assert_eq!(data.truck_placement.heading, 1.57);
        assert_eq!(data.truck_placement.pitch, 0.1);
        assert_eq!(data.truck_placement.roll, -0.05);
        assert_eq!(data.truck_float_values.speed, 25.0);
        assert_eq!(data.truck_float_values.engine_rpm, 1500.0);
        assert_eq!(data.truck_float_values.fuel, 350.5);
        assert_eq!(data.truck_float_values.odometer, 123456.7);
        assert_eq!(data.truck_float_values.cruise_control_speed, 80.0);
        assert_eq!(data.navigation_speed_limit, Some(90.0));
        assert_eq!(data.lead_vehicle_distance_m, None);
        assert_eq!(data.local_acceleration_longitudinal, None);
    }
}
