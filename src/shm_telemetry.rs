//! Shared memory telemetry reader for the TruckPilot native plugin.
//!
//! The native C++ plugin writes a fixed-layout struct into a named shared
//! memory region (Windows: Local\TruckPilotTelemetry, Linux: /truckpilot_telemetry).
//! This module reads that region and converts it into `TelemetryData`.
//!
//! Falls back gracefully if the shared memory is not available (plugin not running).

use crate::telemetry::{TelemetryData, TruckFloatValues, TruckPlacement};

/// Magic value to verify shared memory integrity.
pub const SHM_MAGIC: u32 = 0x54504C54; // "TPLT"

/// Current layout version. Increment when the struct changes.
pub const SHM_VERSION: u32 = 1;

/// Name of the shared memory region (Windows variant).
#[cfg(windows)]
pub const SHM_NAME: &str = "Local\\TruckPilotTelemetry";
/// Name of the shared memory region (Linux/POSIX variant).
#[cfg(not(windows))]
pub const SHM_NAME: &str = "/truckpilot_telemetry";

/// Layout written by the native telemetry plugin.
///
/// This struct must match the C++ `TruckPilotTelemetry` struct exactly,
/// including alignment and padding. It is read directly from shared memory.
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct ShmTelemetryLayout {
    /// Magic: must equal `SHM_MAGIC` for valid data.
    pub magic: u32,
    /// Layout version: must equal `SHM_VERSION`.
    pub version: u32,
    /// Sequence number, incremented each write by the plugin (detect updates).
    pub sequence: u32,
    /// Reserved for alignment.
    pub _pad: u32,

    /// Truck X position in world space (m).
    pub x: f64,
    /// Truck Y position (height) in world space (m).
    pub y: f64,
    /// Truck Z position in world space (m).
    pub z: f64,
    /// Truck heading angle (radians, 0 = world +Z).
    pub heading: f64,
    /// Truck pitch angle (radians).
    pub pitch: f64,
    /// Truck roll angle (radians).
    pub roll: f64,

    /// Forward speed in m/s.
    pub speed_ms: f64,
    /// Engine RPM.
    pub engine_rpm: f64,

    /// Navigation speed limit in km/h (only valid when `nav_speed_limit_valid != 0`).
    pub nav_speed_limit_kmh: f64,
    /// Non-zero if `nav_speed_limit_kmh` should be honoured.
    pub nav_speed_limit_valid: u32,

    /// Remaining fuel in liters.
    pub fuel_liters: f64,
    /// Total odometer reading in km.
    pub odometer_km: f64,

    /// Driver-set cruise control speed in km/h.
    pub cruise_control_speed_kmh: f64,

    /// Local velocity vector (m/s) in truck frame [forward, up, right].
    pub local_velocity: [f32; 3],
    /// Local acceleration vector (m/s^2) in truck frame.
    pub local_acceleration: [f32; 3],
    /// Effective throttle input applied by the engine ECU (0..1).
    pub effective_throttle: f32,
    /// Distance to the nearest lead vehicle (m), if available.
    pub distance_to_lead_m: f32,
}

/// Read telemetry from shared memory.
///
/// Returns `None` if the shared memory region cannot be opened or the
/// magic/version do not match.
pub fn read_shm_telemetry() -> Option<TelemetryData> {
    #[cfg(windows)]
    {
        read_shm_windows()
    }
    #[cfg(not(windows))]
    {
        read_shm_linux()
    }
}

#[cfg(windows)]
fn read_shm_windows() -> Option<TelemetryData> {
    use std::ptr;

    extern "system" {
        fn OpenFileMappingW(dwDesiredAccess: u32, bInheritHandle: i32, lpName: *const u16)
            -> isize;
        fn MapViewOfFile(
            hFileMappingObject: isize,
            dwDesiredAccess: u32,
            dwFileOffsetHigh: u32,
            dwFileOffsetLow: u32,
            dwNumberOfBytesToMap: usize,
        ) -> *mut u8;
        fn UnmapViewOfFile(lpBaseAddress: *const u8) -> i32;
        fn CloseHandle(hObject: isize) -> i32;
    }

    const FILE_MAP_READ: u32 = 4;

    let name_wide: Vec<u16> = SHM_NAME.encode_utf16().chain(std::iter::once(0)).collect();
    let handle = unsafe { OpenFileMappingW(FILE_MAP_READ, 0, name_wide.as_ptr()) };
    if handle == 0 {
        return None;
    }

    let layout_size = std::mem::size_of::<ShmTelemetryLayout>();
    let ptr = unsafe { MapViewOfFile(handle, FILE_MAP_READ, 0, 0, layout_size) };
    if ptr.is_null() {
        unsafe { CloseHandle(handle) };
        return None;
    }

    let layout = unsafe { ptr::read_unaligned(ptr as *const ShmTelemetryLayout) };
    unsafe {
        UnmapViewOfFile(ptr as *const u8);
        CloseHandle(handle);
    }

    shm_to_telemetry(layout)
}

#[cfg(not(windows))]
fn read_shm_linux() -> Option<TelemetryData> {
    // On Linux, shared memory is typically a file in /dev/shm.
    let path = format!("/dev/shm{}", SHM_NAME);
    let data = std::fs::read(&path).ok()?;
    if data.len() < std::mem::size_of::<ShmTelemetryLayout>() {
        return None;
    }
    let layout: ShmTelemetryLayout =
        unsafe { std::ptr::read_unaligned(data.as_ptr() as *const ShmTelemetryLayout) };
    shm_to_telemetry(layout)
}

fn shm_to_telemetry(layout: ShmTelemetryLayout) -> Option<TelemetryData> {
    if layout.magic != SHM_MAGIC || layout.version != SHM_VERSION {
        return None;
    }

    Some(TelemetryData {
        truck_placement: TruckPlacement {
            x: layout.x,
            y: layout.y,
            z: layout.z,
            heading: layout.heading,
            pitch: layout.pitch,
            roll: layout.roll,
        },
        truck_float_values: TruckFloatValues {
            speed: layout.speed_ms,
            engine_rpm: layout.engine_rpm,
            fuel: layout.fuel_liters,
            odometer: layout.odometer_km,
            cruise_control_speed: layout.cruise_control_speed_kmh,
        },
        navigation_speed_limit: if layout.nav_speed_limit_valid != 0 {
            Some(layout.nav_speed_limit_kmh)
        } else {
            None
        },
        lead_vehicle_distance_m: if layout.distance_to_lead_m.is_finite()
            && layout.distance_to_lead_m >= 0.0
        {
            Some(layout.distance_to_lead_m)
        } else {
            None
        },
        local_acceleration_longitudinal: Some(layout.local_acceleration[0]),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_layout_size() {
        // Verify the layout is a fixed, reasonable size.
        let size = std::mem::size_of::<ShmTelemetryLayout>();
        assert!(size >= 64, "SHM layout too small: {size}");
        assert!(size <= 256, "SHM layout too large: {size}");
    }

    #[test]
    fn test_magic_validates() {
        let mut layout = ShmTelemetryLayout {
            magic: 0,
            version: SHM_VERSION,
            sequence: 0,
            _pad: 0,
            x: 1.0,
            y: 0.0,
            z: 2.0,
            heading: 0.5,
            pitch: 0.0,
            roll: 0.0,
            speed_ms: 25.0,
            engine_rpm: 1200.0,
            nav_speed_limit_kmh: 80.0,
            nav_speed_limit_valid: 1,
            fuel_liters: 200.0,
            odometer_km: 15000.0,
            cruise_control_speed_kmh: 85.0,
            local_velocity: [0.0, 0.0, 0.0],
            local_acceleration: [0.0, 0.0, 0.0],
            effective_throttle: 0.0,
            distance_to_lead_m: -1.0,
        };

        assert!(shm_to_telemetry(layout).is_none()); // bad magic

        layout.magic = SHM_MAGIC;
        layout.version = 99; // wrong version
        assert!(shm_to_telemetry(layout).is_none());

        layout.version = SHM_VERSION;
        let t = shm_to_telemetry(layout).unwrap();
        assert_eq!(t.truck_placement.x, 1.0);
        assert_eq!(t.truck_placement.heading, 0.5);
        assert_eq!(t.truck_float_values.speed, 25.0);
        assert_eq!(t.navigation_speed_limit, Some(80.0));
    }

    #[test]
    fn test_nav_speed_limit_invalid() {
        let layout = ShmTelemetryLayout {
            magic: SHM_MAGIC,
            version: SHM_VERSION,
            sequence: 0,
            _pad: 0,
            x: 0.0,
            y: 0.0,
            z: 0.0,
            heading: 0.0,
            pitch: 0.0,
            roll: 0.0,
            speed_ms: 0.0,
            engine_rpm: 0.0,
            nav_speed_limit_kmh: 0.0,
            nav_speed_limit_valid: 0,
            fuel_liters: 0.0,
            odometer_km: 0.0,
            cruise_control_speed_kmh: 0.0,
            local_velocity: [0.0, 0.0, 0.0],
            local_acceleration: [0.0, 0.0, 0.0],
            effective_throttle: 0.0,
            distance_to_lead_m: -1.0,
        };
        let t = shm_to_telemetry(layout).unwrap();
        assert_eq!(t.navigation_speed_limit, None);
    }
}
