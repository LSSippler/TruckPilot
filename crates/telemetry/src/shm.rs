//! Shared-memory telemetry source — opens the mapping once at startup
//! and re-reads the layout struct on every `read()`.
//!
//! - Windows: `Local\TruckPilotTelemetry`
//! - Linux:   `/dev/shm/truckpilot_telemetry`

use std::mem;

use truckpilot_plugin_api::Telemetry;

/// Magic bytes the native plugin writes at the start of the layout.
pub const SHM_MAGIC: u32 = 0x54504C54; // "TPLT"
/// Layout version supported by this reader.
/// Must match the version written by `truckpilot-telemetry-dll`.
pub const SHM_VERSION: u32 = 2;

#[cfg(windows)]
const SHM_NAME: &str = "Local\\TruckPilotTelemetry";
#[cfg(not(windows))]
const SHM_NAME: &str = "/dev/shm/truckpilot_telemetry";

/// Wire-format struct written by the native ETS2 plugin. Must stay in sync
/// with `crates/telemetry_dll/src/lib.rs` of the native side.
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct ShmTelemetryLayout {
    /// Magic, must equal [`SHM_MAGIC`].
    pub magic: u32,
    /// Layout version, must equal [`SHM_VERSION`].
    pub version: u32,
    /// Incremented by the writer on each frame.
    pub sequence: u32,
    /// Padding for 8-byte alignment of the f64 block.
    pub _pad: u32,

    /// World position (m).
    pub x: f64,
    /// World height (m).
    pub y: f64,
    /// World position (m).
    pub z: f64,
    /// Heading (rad).
    pub heading: f64,
    /// Pitch (rad).
    pub pitch: f64,
    /// Roll (rad).
    pub roll: f64,

    /// Forward speed (m/s).
    pub speed_ms: f64,
    /// Engine RPM.
    pub engine_rpm: f64,
    /// Navigation speed limit (km/h), only valid when `nav_speed_limit_valid != 0`.
    pub nav_speed_limit_kmh: f64,
    /// Non-zero ⇒ `nav_speed_limit_kmh` is meaningful.
    pub nav_speed_limit_valid: u32,

    /// Fuel level (l).
    pub fuel_liters: f64,
    /// Total odometer (km).
    pub odometer_km: f64,
    /// Driver-set cruise control speed (km/h, 0 = off).
    pub cruise_control_speed_kmh: f64,

    /// Local velocity vector (m/s) [forward, up, right].
    pub local_velocity: [f32; 3],
    /// Local acceleration vector (m/s²).
    pub local_acceleration: [f32; 3],
    /// Effective throttle commanded by ECU (0..1).
    pub effective_throttle: f32,
    /// Distance to lead vehicle (m). Negative = unknown.
    pub distance_to_lead_m: f32,

    // --- v2 fields ---
    /// Effective brake (0..1).
    pub effective_brake: f32,
    /// Effective clutch (0..1).
    pub effective_clutch: f32,
    /// Driver steering input (−1..1).
    pub input_steering: f32,
    /// Driver throttle input (0..1).
    pub input_throttle: f32,
    /// Driver brake input (0..1).
    pub input_brake: f32,
    /// Driver clutch input (0..1).
    pub input_clutch: f32,
    /// Transmission gear (negative = reverse).
    pub engine_gear: i32,
    /// Displayed gear on dashboard.
    pub displayed_gear: i32,
    /// Hazard lights active (1 = on).
    pub hazard_warning: u8,
    /// Left blinker active (1 = on).
    pub blinker_left: u8,
    /// Right blinker active (1 = on).
    pub blinker_right: u8,
    /// Parking brake engaged (1 = on).
    pub parking_brake: u8,
    /// Game paused (1 = paused).
    pub paused: u8,
    /// Alignment padding — do not use.
    pub _reserved0: [u8; 3],
    /// Microsecond timestamp from the game clock.
    pub timestamp_us: u64,
}

/// Persistent shared-memory reader.
pub struct ShmReader {
    inner: ShmInner,
    last_sequence: u32,
}

impl ShmReader {
    /// Open the shared-memory region. Returns an error if the region does
    /// not exist (e.g. native plugin not running).
    pub fn open() -> Result<Self, String> {
        Ok(Self {
            inner: ShmInner::open()?,
            last_sequence: 0,
        })
    }

    /// Read the current frame. Returns `None` if the magic/version do not
    /// match (writer might be initialising) or if every attempt at a
    /// torn-read-free read fails.
    ///
    /// **Torn-read protection.** The DLL writes the layout struct via a
    /// non-atomic ~196-byte memcpy. A reader running at 50 Hz against a
    /// writer running at frame-rate occasionally catches a half-old /
    /// half-new frame, producing wildly out-of-range f64s. To guard
    /// against this, we read the sequence field, then the body, then
    /// re-read sequence. If the sequence drifted *or* differs from the
    /// sequence embedded in the body, the frame is torn — retry a few
    /// times and give up if the writer is faster than us.
    pub fn read(&mut self) -> Option<Telemetry> {
        const MAX_ATTEMPTS: u32 = 4;

        for _ in 0..MAX_ATTEMPTS {
            let seq_before = self.inner.read_sequence()?;
            let layout = self.inner.read_layout()?;
            let seq_after = self.inner.read_sequence()?;

            if layout.magic != SHM_MAGIC || layout.version != SHM_VERSION {
                return None;
            }

            // Embedded seq must match both bracketing reads — otherwise
            // the writer touched the buffer mid-copy and we got a torn
            // frame. Field reads through `read_sequence` are u32 and
            // therefore atomic on x86_64.
            if seq_before == seq_after && layout.sequence == seq_before {
                self.last_sequence = layout.sequence;
                return Some(layout_to_telemetry(layout));
            }
            // Tiny back-off so we don't spin synchronously with the
            // writer's frame cadence.
            std::thread::sleep(std::time::Duration::from_micros(50));
        }
        None
    }
}

fn layout_to_telemetry(l: ShmTelemetryLayout) -> Telemetry {
    Telemetry {
        position: [l.x, l.y, l.z],
        heading: l.heading,
        pitch: l.pitch,
        roll: l.roll,
        speed_ms: l.speed_ms,
        engine_rpm: l.engine_rpm,
        cruise_control_kmh: l.cruise_control_speed_kmh,
        nav_speed_limit_kmh: if l.nav_speed_limit_valid != 0 {
            l.nav_speed_limit_kmh
        } else {
            -1.0
        },
        lead_vehicle_distance_m: if l.distance_to_lead_m.is_finite() && l.distance_to_lead_m >= 0.0
        {
            l.distance_to_lead_m
        } else {
            -1.0
        },
        accel_longitudinal: l.local_acceleration[0],
        // SHM is the authoritative source for fuel/odometer; both are
        // always present in the layout. Plain copy.
        fuel_liters: l.fuel_liters,
        odometer_km: l.odometer_km,
    }
}

// ---------------------------------------------------------------------------
// Platform-specific backends
// ---------------------------------------------------------------------------

#[cfg(windows)]
struct ShmInner {
    handle: windows::Win32::Foundation::HANDLE,
    view: *const u8,
}

#[cfg(windows)]
unsafe impl Send for ShmInner {}

#[cfg(windows)]
impl ShmInner {
    fn open() -> Result<Self, String> {
        use windows::core::PCWSTR;
        use windows::Win32::System::Memory::{MapViewOfFile, OpenFileMappingW, FILE_MAP_READ};

        let name: Vec<u16> = SHM_NAME.encode_utf16().chain(std::iter::once(0)).collect();
        let handle = unsafe { OpenFileMappingW(FILE_MAP_READ.0, false, PCWSTR(name.as_ptr())) }
            .map_err(|e| format!("OpenFileMappingW failed: {e}"))?;

        if handle.is_invalid() {
            return Err("OpenFileMappingW returned invalid handle".into());
        }

        let view = unsafe {
            MapViewOfFile(
                handle,
                FILE_MAP_READ,
                0,
                0,
                mem::size_of::<ShmTelemetryLayout>(),
            )
        };
        if view.Value.is_null() {
            unsafe {
                let _ = windows::Win32::Foundation::CloseHandle(handle);
            };
            return Err("MapViewOfFile returned null".into());
        }

        Ok(Self {
            handle,
            view: view.Value as *const u8,
        })
    }

    fn read_layout(&self) -> Option<ShmTelemetryLayout> {
        if self.view.is_null() {
            return None;
        }
        Some(unsafe { std::ptr::read_unaligned(self.view as *const ShmTelemetryLayout) })
    }

    /// Read just the `sequence` field (offset 8). Used for torn-read
    /// detection — a u32 read is atomic on x86_64.
    fn read_sequence(&self) -> Option<u32> {
        if self.view.is_null() {
            return None;
        }
        Some(unsafe {
            std::ptr::read_unaligned(self.view.add(mem::offset_of!(ShmTelemetryLayout, sequence))
                as *const u32)
        })
    }
}

#[cfg(windows)]
impl Drop for ShmInner {
    fn drop(&mut self) {
        use windows::Win32::Foundation::CloseHandle;
        use windows::Win32::System::Memory::{UnmapViewOfFile, MEMORY_MAPPED_VIEW_ADDRESS};

        unsafe {
            if !self.view.is_null() {
                let addr = MEMORY_MAPPED_VIEW_ADDRESS {
                    Value: self.view as *mut _,
                };
                let _ = UnmapViewOfFile(addr);
            }
            let _ = CloseHandle(self.handle);
        }
    }
}

#[cfg(not(windows))]
struct ShmInner {
    path: std::path::PathBuf,
}

#[cfg(not(windows))]
impl ShmInner {
    fn open() -> Result<Self, String> {
        let path = std::path::PathBuf::from(SHM_NAME);
        // Don't fail at open time — the file may appear later. We do require
        // the parent dir to exist to keep error messages useful.
        if !path.parent().map(|p| p.exists()).unwrap_or(true) {
            return Err(format!("SHM dir does not exist: {:?}", path.parent()));
        }
        Ok(Self { path })
    }

    fn read_layout(&self) -> Option<ShmTelemetryLayout> {
        let data = std::fs::read(&self.path).ok()?;
        if data.len() < mem::size_of::<ShmTelemetryLayout>() {
            return None;
        }
        Some(unsafe { std::ptr::read_unaligned(data.as_ptr() as *const ShmTelemetryLayout) })
    }

    fn read_sequence(&self) -> Option<u32> {
        let data = std::fs::read(&self.path).ok()?;
        let off = mem::offset_of!(ShmTelemetryLayout, sequence);
        if data.len() < off + 4 {
            return None;
        }
        Some(unsafe { std::ptr::read_unaligned(data.as_ptr().add(off) as *const u32) })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_layout(magic: u32, version: u32) -> ShmTelemetryLayout {
        ShmTelemetryLayout {
            magic,
            version,
            sequence: 1,
            _pad: 0,
            x: 1.0,
            y: 2.0,
            z: 3.0,
            heading: 0.5,
            pitch: 0.0,
            roll: 0.0,
            speed_ms: 25.0,
            engine_rpm: 1200.0,
            nav_speed_limit_kmh: 80.0,
            nav_speed_limit_valid: 1,
            fuel_liters: 200.0,
            odometer_km: 100.0,
            cruise_control_speed_kmh: 85.0,
            local_velocity: [0.0; 3],
            local_acceleration: [-0.5, 0.0, 0.0],
            effective_throttle: 0.5,
            distance_to_lead_m: -1.0,
            effective_brake: 0.0,
            effective_clutch: 0.0,
            input_steering: 0.0,
            input_throttle: 0.0,
            input_brake: 0.0,
            input_clutch: 0.0,
            engine_gear: 3,
            displayed_gear: 3,
            hazard_warning: 0,
            blinker_left: 0,
            blinker_right: 0,
            parking_brake: 0,
            paused: 0,
            _reserved0: [0; 3],
            timestamp_us: 0,
        }
    }

    #[test]
    fn layout_size_is_reasonable() {
        let size = mem::size_of::<ShmTelemetryLayout>();
        assert!((64..=256).contains(&size), "unexpected size: {size}");
    }

    #[test]
    fn converts_valid_layout() {
        let l = make_layout(SHM_MAGIC, SHM_VERSION);
        let t = layout_to_telemetry(l);
        assert_eq!(t.position, [1.0, 2.0, 3.0]);
        assert_eq!(t.speed_ms, 25.0);
        assert_eq!(t.nav_speed_limit_kmh, 80.0);
        assert_eq!(t.lead_vehicle_distance_m, -1.0);
        assert!((t.accel_longitudinal - -0.5).abs() < 1e-6);
        assert_eq!(t.fuel_liters, 200.0);
        assert_eq!(t.odometer_km, 100.0);
    }

    #[test]
    fn invalid_nav_limit_becomes_sentinel() {
        let mut l = make_layout(SHM_MAGIC, SHM_VERSION);
        l.nav_speed_limit_valid = 0;
        let t = layout_to_telemetry(l);
        assert_eq!(t.nav_speed_limit_kmh, -1.0);
    }
}
