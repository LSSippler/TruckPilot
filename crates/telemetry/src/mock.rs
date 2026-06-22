//! Test helpers — mock telemetry sources for downstream crates.
//!
//! These are gated behind `#[cfg(test)]` in the parent module but re-exported
//! to make them available in integration tests of plugin crates.

use std::sync::{Arc, Mutex};

use truckpilot_plugin_api::Telemetry;

/// In-memory mock for plugin tests.
#[derive(Clone, Default)]
pub struct MockTelemetry {
    inner: Arc<Mutex<Option<Telemetry>>>,
}

impl MockTelemetry {
    /// Create an empty mock — `read` returns `None` until `set` is called.
    pub fn new() -> Self {
        Self::default()
    }

    /// Push a frame to the mock; subsequent `read()` calls return it.
    pub fn set(&self, telemetry: Telemetry) {
        *self.inner.lock().unwrap() = Some(telemetry);
    }

    /// Clear the current frame so `read` returns `None`.
    pub fn clear(&self) {
        *self.inner.lock().unwrap() = None;
    }

    /// Read the current frame.
    pub fn read(&self) -> Option<Telemetry> {
        self.inner.lock().unwrap().clone()
    }
}

/// Build a fully-populated `Telemetry` for tests.
pub fn fixture_telemetry() -> Telemetry {
    Telemetry {
        position: [100.0, 5.0, -50.0],
        heading: 0.5,
        pitch: 0.0,
        roll: 0.0,
        speed_ms: 22.222,
        engine_gear: 1,
        engine_rpm: 1500.0,
        cruise_control_kmh: 80.0,
        nav_speed_limit_kmh: 80.0,
        lead_vehicle_distance_m: -1.0,
        accel_longitudinal: -1.0,
        fuel_liters: 320.0,
        odometer_km: 12_345.0,
        nav_distance_m: -1.0,
        nav_time_s: -1.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mock_starts_empty() {
        let m = MockTelemetry::new();
        assert!(m.read().is_none());
    }

    #[test]
    fn mock_remembers_last_set() {
        let m = MockTelemetry::new();
        m.set(fixture_telemetry());
        let t = m.read().unwrap();
        assert_eq!(t.speed_ms, 22.222);
        m.clear();
        assert!(m.read().is_none());
    }
}
