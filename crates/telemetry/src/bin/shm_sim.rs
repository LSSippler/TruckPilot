//! SHM Telemetry Simulator
//!
//! Simulates the `truckpilot_telemetry.dll` on Linux (or Windows without ETS2).
//! Writes realistic fake truck data to the shared-memory region so the full
//! TruckPilot stack can be tested without a running game.
//!
//! Usage:
//!   shm-sim [--scenario <name>] [--hz <rate>] [--duration <secs>]
//!
//! Scenarios:
//!   highway   — Truck cruising at 80 km/h on a straight road (default)
//!   city      — Stop-and-go traffic, speed limit changes
//!   fuel      — Low fuel, approaching fuel stop
//!   brake     — Emergency braking from 90 km/h
//!   parked    — Engine off, parking brake on
//!
//! Examples:
//!   shm-sim                          # highway, 20 Hz, runs until Ctrl+C
//!   shm-sim --scenario city --hz 50  # city traffic at 50 Hz
//!   shm-sim --duration 10            # run for 10 seconds then exit

use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

// Re-use the layout struct and constants from the telemetry crate.
use truckpilot_telemetry::shm::{SHM_MAGIC, SHM_VERSION};

// ---------------------------------------------------------------------------
// SHM writer — platform-specific
// ---------------------------------------------------------------------------

#[cfg(unix)]
mod writer {
    use std::fs::OpenOptions;
    use std::io::Write;
    use std::mem;

    use truckpilot_telemetry::shm::ShmTelemetryLayout;

    const SHM_PATH: &str = "/dev/shm/truckpilot_telemetry";

    pub struct ShmWriter {
        path: &'static str,
    }

    impl ShmWriter {
        pub fn open() -> Result<Self, String> {
            // Touch the file so it exists; we'll overwrite on every write.
            OpenOptions::new()
                .write(true)
                .create(true)
                .truncate(true)
                .open(SHM_PATH)
                .map_err(|e| format!("cannot create {SHM_PATH}: {e}"))?;
            Ok(Self { path: SHM_PATH })
        }

        pub fn write(&self, layout: &ShmTelemetryLayout) -> Result<(), String> {
            let bytes: &[u8] = unsafe {
                std::slice::from_raw_parts(
                    layout as *const ShmTelemetryLayout as *const u8,
                    mem::size_of::<ShmTelemetryLayout>(),
                )
            };
            let mut f = OpenOptions::new()
                .write(true)
                .create(true)
                .truncate(true)
                .open(self.path)
                .map_err(|e| format!("write open: {e}"))?;
            f.write_all(bytes).map_err(|e| format!("write: {e}"))
        }
    }

    impl Drop for ShmWriter {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(self.path);
        }
    }
}

#[cfg(windows)]
// `HANDLE` / `DWORD` mirror the Win32 typedef names so the FFI block
// reads the same as the SDK headers. The acronym style is intentional.
#[allow(clippy::upper_case_acronyms)]
mod writer {
    use std::ffi::c_void;
    use std::mem;
    use std::ptr;

    use truckpilot_telemetry::shm::ShmTelemetryLayout;

    type HANDLE = isize;
    type DWORD = u32;
    const NULL: HANDLE = 0;
    const INVALID_HANDLE_VALUE: HANDLE = -1isize;
    const PAGE_READWRITE: DWORD = 4;
    const FILE_MAP_WRITE: DWORD = 2;

    extern "system" {
        fn CreateFileMappingW(
            hFile: HANDLE,
            lpAttr: *const c_void,
            flProtect: DWORD,
            dwHigh: DWORD,
            dwLow: DWORD,
            lpName: *const u16,
        ) -> HANDLE;
        fn MapViewOfFile(h: HANDLE, acc: DWORD, hi: DWORD, lo: DWORD, n: usize) -> *mut c_void;
        fn UnmapViewOfFile(p: *const c_void) -> i32;
        fn CloseHandle(h: HANDLE) -> i32;
    }

    fn wide(s: &str) -> Vec<u16> {
        s.encode_utf16().chain(std::iter::once(0)).collect()
    }

    pub struct ShmWriter {
        handle: HANDLE,
        ptr: *mut ShmTelemetryLayout,
    }

    unsafe impl Send for ShmWriter {}
    unsafe impl Sync for ShmWriter {}

    impl ShmWriter {
        pub fn open() -> Result<Self, String> {
            let name = wide("Local\\TruckPilotTelemetry");
            let size = mem::size_of::<ShmTelemetryLayout>();
            let handle = unsafe {
                CreateFileMappingW(
                    INVALID_HANDLE_VALUE,
                    ptr::null(),
                    PAGE_READWRITE,
                    0,
                    size as DWORD,
                    name.as_ptr(),
                )
            };
            if handle == NULL {
                return Err("CreateFileMappingW failed".into());
            }
            let ptr = unsafe {
                MapViewOfFile(handle, FILE_MAP_WRITE, 0, 0, size) as *mut ShmTelemetryLayout
            };
            if ptr.is_null() {
                unsafe { CloseHandle(handle) };
                return Err("MapViewOfFile failed".into());
            }
            Ok(Self { handle, ptr })
        }

        pub fn write(&self, layout: &ShmTelemetryLayout) -> Result<(), String> {
            unsafe { std::ptr::copy_nonoverlapping(layout, self.ptr, 1) };
            Ok(())
        }
    }

    impl Drop for ShmWriter {
        fn drop(&mut self) {
            unsafe {
                UnmapViewOfFile(self.ptr as *const c_void);
                CloseHandle(self.handle);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Scenario definitions
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq)]
enum Scenario {
    Highway,
    City,
    Fuel,
    Brake,
    Parked,
}

impl Scenario {
    fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "highway" => Some(Self::Highway),
            "city" => Some(Self::City),
            "fuel" => Some(Self::Fuel),
            "brake" => Some(Self::Brake),
            "parked" => Some(Self::Parked),
            _ => None,
        }
    }

    fn description(self) -> &'static str {
        match self {
            Self::Highway => "Cruising at 80 km/h, straight road, nav limit 90 km/h",
            Self::City => "Stop-and-go, speed limit changes 50→30→50 km/h",
            Self::Fuel => "Low fuel (12 L), approaching fuel stop at 40 km/h",
            Self::Brake => "Emergency braking from 90 km/h to 0",
            Self::Parked => "Engine off, parking brake on, speed 0",
        }
    }
}

// ---------------------------------------------------------------------------
// Simulation state
// ---------------------------------------------------------------------------

use truckpilot_telemetry::shm::ShmTelemetryLayout;

struct SimState {
    scenario: Scenario,
    t: f64,        // simulation time (seconds)
    x: f64,        // world X position (m)
    z: f64,        // world Z position (m)
    speed_ms: f64, // current speed (m/s)
    heading: f64,  // heading (radians)
    fuel_l: f64,   // fuel level (litres)
    odometer_km: f64,
    sequence: u32,
}

impl SimState {
    fn new(scenario: Scenario) -> Self {
        let fuel_l = match scenario {
            Scenario::Fuel => 12.0,
            _ => 400.0,
        };
        Self {
            scenario,
            t: 0.0,
            x: 10000.0,
            z: -5000.0,
            speed_ms: match scenario {
                Scenario::Parked => 0.0,
                Scenario::Brake => 25.0, // 90 km/h
                Scenario::Fuel => 11.1,  // 40 km/h
                Scenario::City => 0.0,
                Scenario::Highway => 22.2, // 80 km/h
            },
            heading: 0.3, // roughly north-east
            fuel_l,
            odometer_km: 12_450.0,
            sequence: 0,
        }
    }

    /// Advance simulation by `dt` seconds and return the new SHM layout.
    fn tick(&mut self, dt: f64) -> ShmTelemetryLayout {
        self.t += dt;
        self.sequence = self.sequence.wrapping_add(1);

        let (speed_ms, throttle, brake, nav_limit_kmh, gear, cruise_kmh) =
            self.compute_controls(dt);

        self.speed_ms = speed_ms;

        // Move truck along heading
        let dx = speed_ms * self.heading.sin() * dt;
        let dz = speed_ms * self.heading.cos() * dt;
        self.x += dx;
        self.z += dz;

        // Gentle heading oscillation (road curves)
        self.heading += match self.scenario {
            Scenario::Highway => 0.002 * (self.t * 0.1).sin(),
            Scenario::City => 0.01 * (self.t * 0.3).sin(),
            _ => 0.0,
        };

        // Fuel consumption (~30 L/100km at highway speed)
        let fuel_per_sec = speed_ms * 0.30 / 100_000.0 * 3600.0;
        self.fuel_l = (self.fuel_l - fuel_per_sec * dt).max(0.0);
        self.odometer_km += speed_ms * dt / 1000.0;

        // Longitudinal acceleration (finite difference approximation)
        let accel_x = (speed_ms - self.speed_ms) / dt.max(0.001);

        // RPM: idle ~800, proportional to speed and gear
        let rpm = if speed_ms < 0.1 {
            800.0
        } else {
            800.0 + speed_ms * 120.0 * (1.0 + 0.1 * (self.t * 2.0).sin())
        };

        ShmTelemetryLayout {
            magic: SHM_MAGIC,
            version: SHM_VERSION,
            sequence: self.sequence,
            _pad: 0,
            x: self.x,
            y: 0.0,
            z: self.z,
            heading: self.heading,
            pitch: 0.0,
            roll: 0.0,
            speed_ms,
            engine_rpm: rpm,
            nav_speed_limit_kmh: nav_limit_kmh,
            nav_speed_limit_valid: if nav_limit_kmh > 0.0 { 1 } else { 0 },
            fuel_liters: self.fuel_l,
            odometer_km: self.odometer_km,
            cruise_control_speed_kmh: cruise_kmh,
            local_velocity: [speed_ms as f32, 0.0, 0.0],
            local_acceleration: [accel_x as f32, 0.0, 0.0],
            effective_throttle: throttle,
            distance_to_lead_m: -1.0,
            effective_brake: brake,
            effective_clutch: 0.0,
            input_steering: (self.heading * 0.1).sin() as f32,
            input_throttle: throttle,
            input_brake: brake,
            input_clutch: 0.0,
            engine_gear: gear,
            displayed_gear: gear,
            hazard_warning: 0,
            blinker_left: 0,
            blinker_right: 0,
            parking_brake: if self.scenario == Scenario::Parked {
                1
            } else {
                0
            },
            paused: 0,
            _reserved0: [0; 3],
            timestamp_us: (self.t * 1_000_000.0) as u64,
        }
    }

    /// Returns (speed_ms, throttle, brake, nav_limit_kmh, gear, cruise_kmh)
    fn compute_controls(&self, dt: f64) -> (f64, f32, f32, f64, i32, f64) {
        match self.scenario {
            Scenario::Highway => {
                // Steady 80 km/h with minor speed variation
                let target = 22.2 + 0.5 * (self.t * 0.05).sin();
                let speed = approach(self.speed_ms, target, 1.0 * dt);
                let throttle = if speed < target { 0.35 } else { 0.0 };
                let gear = speed_to_gear(speed);
                (speed, throttle, 0.0, 90.0, gear, 80.0)
            }

            Scenario::City => {
                // 30-second cycle: accelerate → cruise → brake → stop → go
                let phase = self.t % 30.0;
                let (target, nav_limit) = if phase < 8.0 {
                    (13.9, 50.0) // accelerate to 50 km/h
                } else if phase < 15.0 {
                    (8.3, 30.0) // slow to 30 km/h zone
                } else if phase < 20.0 {
                    (0.0, 30.0) // brake to stop
                } else if phase < 23.0 {
                    (0.0, 50.0) // stopped at light
                } else {
                    (13.9, 50.0) // accelerate again
                };
                let speed = approach(self.speed_ms, target, 3.0 * dt);
                let throttle = if speed < target - 0.5 { 0.6 } else { 0.0 };
                let brake = if speed > target + 0.5 { 0.7 } else { 0.0 };
                let gear = speed_to_gear(speed);
                (speed, throttle, brake, nav_limit, gear, 0.0)
            }

            Scenario::Fuel => {
                // Slow approach, fuel dropping
                let target = 11.1; // 40 km/h
                let speed = approach(self.speed_ms, target, 0.5 * dt);
                let gear = speed_to_gear(speed);
                (speed, 0.25, 0.0, 50.0, gear, 0.0)
            }

            Scenario::Brake => {
                // Hard braking from 90 km/h
                let speed = (self.speed_ms - 8.0 * dt).max(0.0);
                let brake = if speed > 0.1 { 0.95 } else { 0.0 };
                let gear = speed_to_gear(speed);
                (speed, 0.0, brake, 90.0, gear, 0.0)
            }

            Scenario::Parked => (0.0, 0.0, 0.0, 0.0, 0, 0.0),
        }
    }
}

/// Smoothly approach a target value at a given rate per second.
fn approach(current: f64, target: f64, rate: f64) -> f64 {
    let diff = target - current;
    if diff.abs() < rate {
        target
    } else {
        current + diff.signum() * rate
    }
}

/// Approximate gear from speed (simplified 12-speed gearbox).
fn speed_to_gear(speed_ms: f64) -> i32 {
    let kmh = speed_ms * 3.6;
    match kmh as u32 {
        0 => 0,
        1..=10 => 1,
        11..=20 => 2,
        21..=30 => 3,
        31..=45 => 4,
        46..=60 => 5,
        61..=75 => 6,
        76..=90 => 7,
        91..=105 => 8,
        _ => 9,
    }
}

// ---------------------------------------------------------------------------
// CLI args
// ---------------------------------------------------------------------------

struct Config {
    scenario: Scenario,
    hz: f64,
    duration: Option<f64>,
}

impl Config {
    fn from_args() -> Self {
        let args: Vec<String> = std::env::args().collect();
        let mut scenario = Scenario::Highway;
        let mut hz = 20.0_f64;
        let mut duration = None;

        let mut i = 1;
        while i < args.len() {
            match args[i].as_str() {
                "--scenario" if i + 1 < args.len() => {
                    scenario = Scenario::from_str(&args[i + 1]).unwrap_or_else(|| {
                        eprintln!("Unknown scenario '{}'. Using highway.", args[i + 1]);
                        Scenario::Highway
                    });
                    i += 2;
                }
                "--hz" if i + 1 < args.len() => {
                    hz = args[i + 1].parse::<f64>().unwrap_or(20.0).clamp(1.0, 200.0);
                    i += 2;
                }
                "--duration" if i + 1 < args.len() => {
                    duration = args[i + 1].parse().ok();
                    i += 2;
                }
                "--help" | "-h" => {
                    print_help();
                    std::process::exit(0);
                }
                other => {
                    eprintln!("Unknown argument: {other}");
                    std::process::exit(2);
                }
            }
        }

        Self {
            scenario,
            hz,
            duration,
        }
    }
}

fn print_help() {
    println!("shm-sim — TruckPilot SHM telemetry simulator");
    println!();
    println!("Simulates truckpilot_telemetry.dll without ETS2 running.");
    println!("Writes fake truck data to the shared-memory region.");
    println!();
    println!("Usage: shm-sim [OPTIONS]");
    println!();
    println!("Options:");
    println!("  --scenario <name>   highway|city|fuel|brake|parked  (default: highway)");
    println!("  --hz <rate>         Update rate in Hz, 1-200  (default: 20)");
    println!("  --duration <secs>   Run for N seconds then exit  (default: run forever)");
    println!("  --help              Show this help");
    println!();
    println!("Scenarios:");
    println!("  highway  {}", Scenario::Highway.description());
    println!("  city     {}", Scenario::City.description());
    println!("  fuel     {}", Scenario::Fuel.description());
    println!("  brake    {}", Scenario::Brake.description());
    println!("  parked   {}", Scenario::Parked.description());
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() {
    let cfg = Config::from_args();
    let dt = 1.0 / cfg.hz;
    let interval = Duration::from_secs_f64(dt);

    // Ctrl+C handler
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc_handler(r);

    // Open SHM
    let writer = match writer::ShmWriter::open() {
        Ok(w) => w,
        Err(e) => {
            eprintln!("Failed to open SHM: {e}");
            std::process::exit(1);
        }
    };

    let mut state = SimState::new(cfg.scenario);
    let start = Instant::now();

    println!(
        "shm-sim: scenario={:?}  hz={:.0}  SHM ready",
        cfg.scenario, cfg.hz
    );
    println!("         {}", cfg.scenario.description());
    println!("         Press Ctrl+C to stop.");
    println!();

    let mut frame = 0u64;
    let mut last_print = Instant::now();

    while running.load(Ordering::Relaxed) {
        // Check duration limit
        if let Some(max_secs) = cfg.duration {
            if start.elapsed().as_secs_f64() >= max_secs {
                break;
            }
        }

        let tick_start = Instant::now();
        let layout = state.tick(dt);

        if let Err(e) = writer.write(&layout) {
            eprintln!("SHM write error: {e}");
            break;
        }

        frame += 1;

        // Print status line every second.
        // Copy fields out of packed struct before formatting (avoids unaligned ref).
        if last_print.elapsed() >= Duration::from_secs(1) {
            let speed_ms = layout.speed_ms;
            let x = layout.x;
            let z = layout.z;
            let fuel = layout.fuel_liters;
            let gear = layout.engine_gear;
            let seq = layout.sequence;
            print!(
                "\r  t={:.1}s  speed={:.1}km/h  pos=({:.0},{:.0})  fuel={:.1}L  gear={}  seq={}   ",
                state.t,
                speed_ms * 3.6,
                x,
                z,
                fuel,
                gear,
                seq,
            );
            std::io::stdout().flush().ok();
            last_print = Instant::now();
        }

        // Sleep for the remainder of the tick interval
        let elapsed = tick_start.elapsed();
        if elapsed < interval {
            std::thread::sleep(interval - elapsed);
        }
    }

    println!();
    println!("shm-sim: stopped after {frame} frames ({:.1}s)", state.t);
}

/// Register SIGINT/SIGTERM handlers that set `flag` to true.
/// Separated into its own function so `#[allow]` applies precisely.
#[cfg(unix)]
#[allow(clippy::fn_to_numeric_cast)]
fn register_unix_signals(flag: &'static AtomicBool) {
    unsafe extern "C" fn on_signal(_: libc::c_int) {
        // Safety: AtomicBool::store is async-signal-safe.
        // We use a static reference passed in — no allocation in signal handler.
        // The flag pointer is stored in a thread-local to avoid capturing.
        SIGNAL_FLAG.store(true, Ordering::Relaxed);
    }
    // Store the flag pointer so the signal handler can reach it.
    // We use a global because signal handlers can't capture.
    SIGNAL_FLAG_PTR.store(flag as *const AtomicBool as usize, Ordering::Relaxed);

    unsafe {
        libc::signal(libc::SIGINT, on_signal as *const () as libc::sighandler_t);
        libc::signal(libc::SIGTERM, on_signal as *const () as libc::sighandler_t);
    }

    // Proxy thread: copies SIGNAL_FLAG → the actual flag passed in.
    std::thread::spawn(move || loop {
        if SIGNAL_FLAG.load(Ordering::Relaxed) {
            flag.store(true, Ordering::Relaxed);
            break;
        }
        std::thread::sleep(Duration::from_millis(20));
    });
}

#[cfg(unix)]
static SIGNAL_FLAG: AtomicBool = AtomicBool::new(false);
#[cfg(unix)]
static SIGNAL_FLAG_PTR: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

/// Register a Ctrl+C / SIGTERM handler that sets `running` to false.
fn ctrlc_handler(running: Arc<AtomicBool>) {
    #[cfg(unix)]
    {
        // register_unix_signals sets SIGNAL_FLAG on signal and copies it to
        // the provided static. We bridge that to the Arc via a watcher thread.
        static STOP_FLAG: AtomicBool = AtomicBool::new(false);
        register_unix_signals(&STOP_FLAG);
        std::thread::spawn(move || loop {
            if STOP_FLAG.load(Ordering::Relaxed) {
                running.store(false, Ordering::Relaxed);
                break;
            }
            std::thread::sleep(Duration::from_millis(50));
        });
    }
    #[cfg(windows)]
    {
        // On Windows the default Ctrl+C handler raises SIGINT which terminates
        // the process. For a graceful shutdown we would need SetConsoleCtrlHandler,
        // but for a dev tool the default behaviour is acceptable.
        let _ = running;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scenario_from_str_all_variants() {
        assert_eq!(Scenario::from_str("highway"), Some(Scenario::Highway));
        assert_eq!(Scenario::from_str("CITY"), Some(Scenario::City));
        assert_eq!(Scenario::from_str("fuel"), Some(Scenario::Fuel));
        assert_eq!(Scenario::from_str("brake"), Some(Scenario::Brake));
        assert_eq!(Scenario::from_str("parked"), Some(Scenario::Parked));
        assert_eq!(Scenario::from_str("unknown"), None);
    }

    #[test]
    fn approach_reaches_target() {
        let result = approach(0.0, 10.0, 20.0);
        assert_eq!(result, 10.0); // rate > diff → snap to target
    }

    #[test]
    fn approach_moves_toward_target() {
        let result = approach(0.0, 10.0, 2.0);
        assert!((result - 2.0).abs() < 1e-9);
    }

    #[test]
    fn approach_works_negative_direction() {
        let result = approach(10.0, 0.0, 3.0);
        assert!((result - 7.0).abs() < 1e-9);
    }

    #[test]
    fn speed_to_gear_zero_speed() {
        assert_eq!(speed_to_gear(0.0), 0);
    }

    #[test]
    fn speed_to_gear_highway() {
        // 80 km/h = 22.2 m/s → gear 7
        assert_eq!(speed_to_gear(22.2), 7);
    }

    #[test]
    fn speed_to_gear_city() {
        // 30 km/h = 8.3 m/s → gear 3
        assert_eq!(speed_to_gear(8.3), 3);
    }

    /// Helper: copy packed fields to avoid unaligned reference UB in tests.
    fn unpack(l: &ShmTelemetryLayout) -> (u32, u32, u32, f64, f64, f64, f64, f64, u32, u8, i32) {
        (
            l.magic,
            l.version,
            l.sequence,
            l.speed_ms,
            l.fuel_liters,
            l.engine_rpm,
            l.nav_speed_limit_kmh,
            l.nav_speed_limit_valid as f64,
            l.nav_speed_limit_valid,
            l.parking_brake,
            l.engine_gear,
        )
    }

    #[test]
    fn sim_state_highway_tick_produces_valid_layout() {
        let mut state = SimState::new(Scenario::Highway);
        let layout = state.tick(0.05);
        let (magic, version, seq, speed, fuel, rpm, _, _, _, parking, _) = unpack(&layout);

        assert_eq!(magic, SHM_MAGIC);
        assert_eq!(version, SHM_VERSION);
        assert_eq!(seq, 1);
        assert!(speed > 0.0);
        assert!(fuel > 0.0);
        assert!(rpm > 0.0);
        assert_eq!(parking, 0);
    }

    #[test]
    fn sim_state_parked_has_zero_speed() {
        let mut state = SimState::new(Scenario::Parked);
        let layout = state.tick(0.05);
        let (_, _, _, speed, _, _, _, _, _, parking, gear) = unpack(&layout);

        assert_eq!(speed, 0.0);
        assert_eq!(parking, 1);
        assert_eq!(gear, 0);
    }

    #[test]
    fn sim_state_brake_decelerates() {
        let mut state = SimState::new(Scenario::Brake);
        let s1 = state.tick(0.05).speed_ms;
        let s2 = state.tick(0.05).speed_ms;
        let s3 = state.tick(0.05).speed_ms;

        assert!(s2 <= s1);
        assert!(s3 <= s2);
    }

    #[test]
    fn sim_state_fuel_decreases_while_moving() {
        let mut state = SimState::new(Scenario::Highway);
        let initial_fuel = state.fuel_l;
        for _ in 0..200 {
            state.tick(0.05);
        }
        assert!(state.fuel_l < initial_fuel);
    }

    #[test]
    fn sim_state_odometer_increases() {
        let mut state = SimState::new(Scenario::Highway);
        let initial_odo = state.odometer_km;
        for _ in 0..200 {
            state.tick(0.05);
        }
        assert!(state.odometer_km > initial_odo);
    }

    #[test]
    fn sim_state_sequence_increments() {
        let mut state = SimState::new(Scenario::Highway);
        let s1 = state.tick(0.05).sequence;
        let s2 = state.tick(0.05).sequence;
        let s3 = state.tick(0.05).sequence;
        assert_eq!(s1, 1);
        assert_eq!(s2, 2);
        assert_eq!(s3, 3);
    }

    #[test]
    fn sim_state_fuel_scenario_starts_low() {
        let state = SimState::new(Scenario::Fuel);
        assert!(state.fuel_l < 20.0);
    }

    #[test]
    fn layout_nav_limit_valid_flag() {
        let mut state = SimState::new(Scenario::Highway);
        let layout = state.tick(0.05);
        let nav_valid = layout.nav_speed_limit_valid;
        let nav_kmh = layout.nav_speed_limit_kmh;
        assert_eq!(nav_valid, 1);
        assert!((nav_kmh - 90.0).abs() < 1.0);
    }

    #[test]
    fn layout_parked_nav_limit_zero() {
        let mut state = SimState::new(Scenario::Parked);
        let layout = state.tick(0.05);
        let nav_valid = layout.nav_speed_limit_valid;
        assert_eq!(nav_valid, 0);
    }
}
