use std::path::Path;
use std::thread;
use std::time::Duration;

use clap::Parser;
use truckpilot::shm_telemetry::{ShmTelemetryLayout, SHM_MAGIC, SHM_NAME, SHM_VERSION};

const SHM_TOTAL_SIZE: usize = 512;
const OFF_ENGINE_GEAR: usize = 172;
const OFF_DISPLAYED_GEAR: usize = 176;
const OFF_HAZARD_WARNING: usize = 180;
const OFF_BLINKER_LEFT: usize = 181;
const OFF_BLINKER_RIGHT: usize = 182;
const OFF_PARKING_BRAKE: usize = 183;

#[derive(Debug, Clone, Copy)]
struct ShmTelemetryTail {
    gear: i32,
    gear_displayed: i32,
    blinker_left: u8,
    blinker_right: u8,
    hazard_lights: u8,
    parking_brake: u8,
}

#[derive(Debug, Clone)]
struct TelemetrySnapshot {
    base: ShmTelemetryLayout,
    tail: Option<ShmTelemetryTail>,
}

#[derive(Parser, Debug)]
#[command(
    name = "telemetry_diag",
    about = "Inspect TruckPilot shared-memory telemetry"
)]
struct Cli {
    /// Read one snapshot and exit.
    #[arg(long, default_value_t = false)]
    once: bool,

    /// Refresh interval in milliseconds (default 500).
    #[arg(long, default_value_t = 500)]
    interval_ms: u64,
}

fn heading_deg(rad: f64) -> f64 {
    rad.to_degrees()
}

fn speed_kmh(speed_ms: f64) -> f64 {
    speed_ms * 3.6
}

fn read_i32_le(bytes: &[u8], offset: usize) -> Option<i32> {
    let slice = bytes.get(offset..offset + std::mem::size_of::<i32>())?;
    let arr: [u8; 4] = slice.try_into().ok()?;
    Some(i32::from_le_bytes(arr))
}

fn read_u8(bytes: &[u8], offset: usize) -> Option<u8> {
    bytes.get(offset).copied()
}

fn read_snapshot_from_bytes(bytes: &[u8]) -> Result<TelemetrySnapshot, String> {
    let base_size = std::mem::size_of::<ShmTelemetryLayout>();
    if bytes.len() < base_size {
        return Err(format!(
            "shared memory too small: {} bytes (expected at least {})",
            bytes.len(),
            base_size
        ));
    }

    let base = unsafe { std::ptr::read_unaligned(bytes.as_ptr() as *const ShmTelemetryLayout) };

    let tail = if bytes.len() > OFF_PARKING_BRAKE {
        Some(ShmTelemetryTail {
            gear: read_i32_le(bytes, OFF_ENGINE_GEAR).ok_or("invalid engine_gear bytes")?,
            gear_displayed: read_i32_le(bytes, OFF_DISPLAYED_GEAR)
                .ok_or("invalid displayed_gear bytes")?,
            hazard_lights: read_u8(bytes, OFF_HAZARD_WARNING).ok_or("invalid hazard bytes")?,
            blinker_left: read_u8(bytes, OFF_BLINKER_LEFT).ok_or("invalid blinker_left bytes")?,
            blinker_right: read_u8(bytes, OFF_BLINKER_RIGHT)
                .ok_or("invalid blinker_right bytes")?,
            parking_brake: read_u8(bytes, OFF_PARKING_BRAKE)
                .ok_or("invalid parking_brake bytes")?,
        })
    } else {
        None
    };

    Ok(TelemetrySnapshot { base, tail })
}

fn read_snapshot_from_path(path: &Path) -> Result<TelemetrySnapshot, String> {
    let meta =
        std::fs::metadata(path).map_err(|e| format!("failed to stat {}: {e}", path.display()))?;
    if meta.len() > 4096 {
        return Err(format!(
            "refusing to read oversized SHM dump: {} bytes",
            meta.len()
        ));
    }
    let bytes =
        std::fs::read(path).map_err(|e| format!("failed to read {}: {e}", path.display()))?;
    read_snapshot_from_bytes(&bytes)
}

#[cfg(windows)]
fn read_snapshot_default() -> Result<TelemetrySnapshot, String> {
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
        return Err(format!(
            "shared memory '{SHM_NAME}' not found. Ensure telemetry DLL is loaded in ETS2."
        ));
    }

    let layout_size = SHM_TOTAL_SIZE;
    let ptr = unsafe { MapViewOfFile(handle, FILE_MAP_READ, 0, 0, layout_size) };
    if ptr.is_null() {
        unsafe { CloseHandle(handle) };
        return Err("failed to map shared memory view".to_string());
    }

    let mut bytes = vec![0_u8; layout_size];
    unsafe { ptr::copy_nonoverlapping(ptr, bytes.as_mut_ptr(), layout_size) };
    unsafe {
        UnmapViewOfFile(ptr as *const u8);
        CloseHandle(handle);
    }

    read_snapshot_from_bytes(&bytes)
}

#[cfg(not(windows))]
fn read_snapshot_default() -> Result<TelemetrySnapshot, String> {
    use std::io::Read;

    let path = format!("/dev/shm{SHM_NAME}");
    let mut file = std::fs::File::open(&path).map_err(|e| format!("failed to read {path}: {e}"))?;
    let mut bytes = vec![0_u8; SHM_TOTAL_SIZE];
    let count = file
        .read(&mut bytes)
        .map_err(|e| format!("failed to read {path}: {e}"))?;
    if count < std::mem::size_of::<ShmTelemetryLayout>() {
        return Err(format!(
            "shared memory too small: {count} bytes (expected at least {})",
            std::mem::size_of::<ShmTelemetryLayout>()
        ));
    }
    bytes.truncate(count);
    read_snapshot_from_bytes(&bytes)
}

fn format_extra(tail: Option<ShmTelemetryTail>) -> String {
    if let Some(t) = tail {
        let gear = t.gear;
        let gear_displayed = t.gear_displayed;
        let blinker_left = t.blinker_left != 0;
        let blinker_right = t.blinker_right != 0;
        let hazard_lights = t.hazard_lights != 0;
        let parking_brake = t.parking_brake != 0;
        format!(
            "Gear={} | GearDisplayed={} | Blinkers(L/R/H)={}/{}/{} | ParkingBrake={}",
            gear, gear_displayed, blinker_left, blinker_right, hazard_lights, parking_brake
        )
    } else {
        "Gear=n/a | GearDisplayed=n/a | Blinkers(L/R/H)=n/a/n/a/n/a | ParkingBrake=n/a".to_string()
    }
}

fn print_snapshot(snapshot: &TelemetrySnapshot) {
    let b = snapshot.base;
    let magic = b.magic;
    let version = b.version;
    let x = b.x;
    let y = b.y;
    let z = b.z;
    let heading = b.heading;
    let speed_ms = b.speed_ms;
    let engine_rpm = b.engine_rpm;
    let nav_speed_limit_valid = b.nav_speed_limit_valid;
    let nav_speed_limit_kmh = b.nav_speed_limit_kmh;
    println!("------------------------------------------------------------");
    println!("Magic=0x{:08X} | LayoutVersion={}", magic, version);
    println!(
        "WorldPos=({:.3}, {:.3}, {:.3}) | Heading={:.2}°",
        x,
        y,
        z,
        heading_deg(heading)
    );
    println!(
        "Speed={:.2} km/h | RPM={:.0}",
        speed_kmh(speed_ms),
        engine_rpm
    );
    if nav_speed_limit_valid != 0 {
        println!("NavigationSpeedLimit={:.1} km/h", nav_speed_limit_kmh);
    } else {
        println!("NavigationSpeedLimit=n/a");
    }
    println!("{}", format_extra(snapshot.tail));
}

fn main() {
    let cli = Cli::parse();
    let interval_ms = cli.interval_ms.max(1);

    loop {
        let snapshot = if let Ok(path) = std::env::var("TRUCKPILOT_SHM_PATH") {
            read_snapshot_from_path(Path::new(&path))
        } else {
            read_snapshot_default()
        };

        match snapshot {
            Ok(data) => {
                let magic = data.base.magic;
                let version = data.base.version;
                if magic != SHM_MAGIC || version != SHM_VERSION {
                    if cli.once {
                        eprintln!(
                            "shared memory opened but magic/version mismatch: magic=0x{:08X} version={} expected_magic=0x{:08X} expected_version={}",
                            magic,
                            version,
                            SHM_MAGIC,
                            SHM_VERSION
                        );
                        std::process::exit(1);
                    }

                    eprintln!(
                        "shared memory magic/version mismatch: magic=0x{:08X} version={} expected_magic=0x{:08X} expected_version={}",
                        magic,
                        version,
                        SHM_MAGIC,
                        SHM_VERSION
                    );
                    thread::sleep(Duration::from_millis(interval_ms));
                    continue;
                }
                print_snapshot(&data);
            }
            Err(err) => {
                eprintln!(
                    "shared memory not available: {err}. Ensure ETS2 runs with the telemetry DLL/plugin loaded."
                );
                std::process::exit(1);
            }
        }

        if cli.once {
            break;
        }

        thread::sleep(Duration::from_millis(interval_ms));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use truckpilot::shm_telemetry::{SHM_MAGIC, SHM_VERSION};

    #[test]
    fn test_telemetry_diag_opens_shm() {
        let base = ShmTelemetryLayout {
            magic: SHM_MAGIC,
            version: SHM_VERSION,
            sequence: 1,
            _pad: 0,
            x: 100.0,
            y: 10.0,
            z: -50.0,
            heading: std::f64::consts::FRAC_PI_2,
            pitch: 0.0,
            roll: 0.0,
            speed_ms: 20.0,
            engine_rpm: 1300.0,
            nav_speed_limit_kmh: 80.0,
            nav_speed_limit_valid: 1,
            fuel_liters: 200.0,
            odometer_km: 1000.0,
            cruise_control_speed_kmh: 85.0,
            local_velocity: [0.0, 0.0, 0.0],
            local_acceleration: [0.0, 0.0, 0.0],
            effective_throttle: 0.0,
            distance_to_lead_m: -1.0,
        };

        let tail = ShmTelemetryTail {
            gear: 7,
            gear_displayed: 7,
            blinker_left: 1,
            blinker_right: 0,
            hazard_lights: 0,
            parking_brake: 0,
        };

        let mut bytes = vec![0_u8; SHM_TOTAL_SIZE];
        unsafe {
            std::ptr::copy_nonoverlapping(
                &base as *const ShmTelemetryLayout as *const u8,
                bytes.as_mut_ptr(),
                std::mem::size_of::<ShmTelemetryLayout>(),
            );
        }
        bytes[OFF_ENGINE_GEAR..OFF_ENGINE_GEAR + 4].copy_from_slice(&tail.gear.to_le_bytes());
        bytes[OFF_DISPLAYED_GEAR..OFF_DISPLAYED_GEAR + 4]
            .copy_from_slice(&tail.gear_displayed.to_le_bytes());
        bytes[OFF_HAZARD_WARNING] = tail.hazard_lights;
        bytes[OFF_BLINKER_LEFT] = tail.blinker_left;
        bytes[OFF_BLINKER_RIGHT] = tail.blinker_right;
        bytes[OFF_PARKING_BRAKE] = tail.parking_brake;

        let snapshot = read_snapshot_from_bytes(&bytes).expect("snapshot should parse");
        let magic = snapshot.base.magic;
        let version = snapshot.base.version;
        let speed_ms = snapshot.base.speed_ms;
        assert_eq!(magic, SHM_MAGIC);
        assert_eq!(version, SHM_VERSION);
        assert!((speed_kmh(speed_ms) - 72.0).abs() < 1e-9);

        let extra = format_extra(snapshot.tail);
        assert!(extra.contains("Gear=7"));
        assert!(extra.contains("Blinkers(L/R/H)=true/false/false"));
    }
}
