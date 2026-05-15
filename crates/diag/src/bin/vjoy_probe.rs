//! Phase 6.2a GATE-0 — vJoy Probe.
//!
//! Validates that the vJoy driver is installed, Device N has X/Y/Z axes
//! configured, and that TruckPilot can write axes that ETS2 will see.
//! See `outputs/active/phase_6.2a_vjoy_probe_spec.md` for the full spec.

use std::process::ExitCode;

use clap::Parser;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// vJoy default axis range. vJoyConf can change this but most setups stick
/// with the 0..=32767 default; the v0.4 crate does not expose Min/Max query.
const AXIS_MIN: i32 = 0;
const AXIS_MAX: i32 = 32_767;

/// HID Usage Codes as expected by `SetAxis(value, rID, axis)` in vJoyInterface.dll.
/// 0x30 = X, 0x31 = Y, 0x32 = Z. The old vjoy-0.4 crate took 1/2/3 (its own
/// position-based index into AXES_HID_USAGE) — the direct FFI API needs the
/// actual HID code.
const STEER_AXIS: u32 = 0x30;
const THROTTLE_AXIS: u32 = 0x31;
const BRAKE_AXIS: u32 = 0x32;

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

#[derive(Parser, Debug, Clone)]
#[command(
    name = "truckpilot-vjoy-probe",
    about = "Phase 6.2a GATE-0: validate vJoy driver + Device N for ETS2 autopilot."
)]
struct Args {
    /// vJoy device id (1..=16).
    #[arg(long, default_value_t = 1)]
    device: u32,

    /// Test duration in seconds.
    #[arg(long, default_value_t = 10)]
    duration: u32,

    /// Test pattern: sine | ramp | center-hold.
    #[arg(long, default_value = "sine")]
    pattern: String,

    /// Update rate in Hz.
    #[arg(long = "rate-hz", default_value_t = 50)]
    rate_hz: u32,

    /// Steering amplitude, 0.0..=1.0.
    #[arg(long = "steer-amp", default_value_t = 0.5)]
    steer_amp: f64,

    /// Sine frequency in Hz.
    #[arg(long = "steer-freq", default_value_t = 0.5)]
    steer_freq: f64,

    /// Constant throttle, 0.0..=1.0. Default 0.0 — truck stays still (Q1=A).
    #[arg(long, default_value_t = 0.0)]
    throttle: f64,

    /// Constant brake, 0.0..=1.0.
    #[arg(long, default_value_t = 0.0)]
    brake: f64,

    /// Acquire device + axis checks only, no send loop.
    #[arg(long, default_value_t = false)]
    dry_run: bool,

    /// Also write JSONL to `outputs/active/vjoy_probe_<ts>.jsonl` (Q3=B).
    #[arg(long, default_value_t = false)]
    json: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Pattern {
    Sine,
    Ramp,
    CenterHold,
}

impl Pattern {
    fn parse(s: &str) -> Result<Self, String> {
        match s {
            "sine" => Ok(Self::Sine),
            "ramp" => Ok(Self::Ramp),
            "center-hold" => Ok(Self::CenterHold),
            other => Err(format!(
                "unknown pattern '{other}'. Valid: sine | ramp | center-hold"
            )),
        }
    }
}

/// Validates CLI arg ranges. Returns Ok(parsed pattern) or error string.
fn validate(args: &Args) -> Result<Pattern, String> {
    if !(1..=16).contains(&args.device) {
        return Err("--device must be in 1..=16".to_string());
    }
    if args.duration == 0 {
        return Err("--duration must be > 0".to_string());
    }
    if args.rate_hz == 0 {
        return Err("--rate-hz must be > 0".to_string());
    }
    if !(0.0..=1.0).contains(&args.steer_amp) {
        return Err("--steer-amp must be in 0.0..=1.0".to_string());
    }
    if !(0.0..=1.0).contains(&args.throttle) {
        return Err("--throttle must be in 0.0..=1.0".to_string());
    }
    if !(0.0..=1.0).contains(&args.brake) {
        return Err("--brake must be in 0.0..=1.0".to_string());
    }
    Pattern::parse(&args.pattern)
}

// ---------------------------------------------------------------------------
// Pattern generator + axis mapping (platform-independent, unit-tested)
// ---------------------------------------------------------------------------

/// (steer, throttle, brake) all normalized to -1.0..=1.0 for steer
/// and 0.0..=1.0 for throttle/brake.
fn compute_sample(
    pattern: Pattern,
    t: f64,
    duration: f64,
    steer_amp: f64,
    steer_freq: f64,
    throttle: f64,
    brake: f64,
) -> (f64, f64, f64) {
    let steer = match pattern {
        Pattern::Sine => steer_amp * (2.0 * std::f64::consts::PI * steer_freq * t).sin(),
        Pattern::Ramp => {
            // Triangle wave: 0 → +1 → -1 → 0 over `duration`.
            let phase = (t / duration).clamp(0.0, 1.0);
            let raw = if phase < 0.25 {
                phase * 4.0
            } else if phase < 0.75 {
                2.0 - phase * 4.0
            } else {
                phase * 4.0 - 4.0
            };
            steer_amp * raw
        }
        Pattern::CenterHold => 0.0,
    };
    (steer.clamp(-1.0, 1.0), throttle, brake)
}

/// Maps `value` from `-1.0..=1.0` to `AXIS_MIN..=AXIS_MAX` with center at the
/// midpoint. Out-of-range inputs are clamped.
fn map_signed_to_raw(value: f64) -> i32 {
    let clamped = value.clamp(-1.0, 1.0);
    let center = (AXIS_MIN + AXIS_MAX) as f64 / 2.0;
    let half_range = (AXIS_MAX - AXIS_MIN) as f64 / 2.0;
    (center + clamped * half_range).round() as i32
}

/// Maps `value` from `0.0..=1.0` to `AXIS_MIN..=AXIS_MAX`. Clamps.
fn map_unsigned_to_raw(value: f64) -> i32 {
    let clamped = value.clamp(0.0, 1.0);
    let span = (AXIS_MAX - AXIS_MIN) as f64;
    (AXIS_MIN as f64 + clamped * span).round() as i32
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

fn main() -> ExitCode {
    let args = Args::parse();

    let pattern = match validate(&args) {
        Ok(p) => p,
        Err(msg) => {
            eprintln!("{msg}");
            return ExitCode::from(3);
        }
    };

    #[cfg(windows)]
    {
        windows_impl::run(args, pattern)
    }

    #[cfg(not(windows))]
    {
        let _ = pattern;
        eprintln!(
            "truckpilot-vjoy-probe: vJoy is Windows-only. \
             This binary cannot exercise the driver on this platform."
        );
        ExitCode::from(1)
    }
}

// ---------------------------------------------------------------------------
// Windows implementation
// ---------------------------------------------------------------------------

#[cfg(windows)]
mod windows_impl {
    use super::*;

    use std::fs::File;
    use std::io::{BufWriter, Write};
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;
    use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

    use windows::core::w;
    use windows::Win32::Foundation::{FreeLibrary, HMODULE};
    use windows::Win32::System::LibraryLoader::LoadLibraryW;

    // ------- Direct FFI signatures -------

    type FnVJoyEnabled = unsafe extern "system" fn() -> i32;
    type FnAcquireVJD = unsafe extern "system" fn(u32) -> i32;
    type FnRelinquishVJD = unsafe extern "system" fn(u32);
    type FnGetVJDStatus = unsafe extern "system" fn(u32) -> i32;
    type FnSetAxis = unsafe extern "system" fn(i32, u32, u32) -> i32;

    const VJD_STAT_OWN: i32 = 0;
    const VJD_STAT_FREE: i32 = 1;
    const VJD_STAT_BUSY: i32 = 2;
    const VJD_STAT_MISS: i32 = 3;

    struct VJoyFfi {
        dll: HMODULE,
        vjoy_enabled: FnVJoyEnabled,
        acquire_vjd: FnAcquireVJD,
        relinquish_vjd: FnRelinquishVJD,
        get_vjd_status: FnGetVJDStatus,
        set_axis: FnSetAxis,
    }

    impl Drop for VJoyFfi {
        fn drop(&mut self) {
            unsafe {
                let _ = FreeLibrary(self.dll);
            }
        }
    }

    /// Load `vJoyInterface.dll` and resolve the symbols the probe needs.
    /// Returns a typed FFI wrapper that frees the library on drop.
    fn load_dll() -> Result<VJoyFfi, String> {
        let dll = unsafe {
            LoadLibraryW(w!("C:\\Program Files\\vJoy\\x64\\vJoyInterface.dll"))
                .or_else(|_| LoadLibraryW(w!("vJoyInterface.dll")))
                .map_err(|e| format!("LoadLibraryW failed: {e}"))?
        };

        unsafe fn resolve<F>(dll: HMODULE, name: &str) -> Result<F, String> {
            use windows::core::PCSTR;
            use windows::Win32::System::LibraryLoader::GetProcAddress;
            let mut cstr = String::with_capacity(name.len() + 1);
            cstr.push_str(name);
            cstr.push('\0');
            let proc = GetProcAddress(dll, PCSTR(cstr.as_ptr()));
            proc.map(|addr| {
                std::mem::transmute_copy::<unsafe extern "system" fn() -> isize, F>(&addr)
            })
            .ok_or_else(|| format!("missing symbol: {name}"))
        }

        unsafe {
            let vjoy_enabled: FnVJoyEnabled = resolve(dll, "vJoyEnabled")?;
            let acquire_vjd: FnAcquireVJD = resolve(dll, "AcquireVJD")?;
            let relinquish_vjd: FnRelinquishVJD = resolve(dll, "RelinquishVJD")?;
            let get_vjd_status: FnGetVJDStatus = resolve(dll, "GetVJDStatus")?;
            let set_axis: FnSetAxis = resolve(dll, "SetAxis")?;
            Ok(VJoyFfi {
                dll,
                vjoy_enabled,
                acquire_vjd,
                relinquish_vjd,
                get_vjd_status,
                set_axis,
            })
        }
    }

    pub fn run(args: Args, pattern: Pattern) -> ExitCode {
        // -- Pre-flight ----------------------------------------------------
        let ffi = match load_dll() {
            Ok(f) => f,
            Err(e) => {
                eprintln!(
                    "vJoy driver not found ({e}). Install from \
                     https://github.com/njz3/vJoy/releases and reboot."
                );
                return ExitCode::from(1);
            }
        };

        if unsafe { (ffi.vjoy_enabled)() } == 0 {
            eprintln!("vJoy driver disabled (vJoyEnabled returned 0). Enable in vJoyConf.");
            return ExitCode::from(1);
        }

        let status = unsafe { (ffi.get_vjd_status)(args.device) };
        match status {
            VJD_STAT_FREE | VJD_STAT_OWN => {}
            VJD_STAT_BUSY => {
                eprintln!(
                    "Device {id} is busy (owned by another process). Close ETS2-LA / \
                     joy.cpl Test page / x360ce / TruckPilot daemon.",
                    id = args.device
                );
                return ExitCode::from(2);
            }
            VJD_STAT_MISS => {
                eprintln!(
                    "Device {id} is not configured. Open vJoyConf and enable \
                     Device {id} with X/Y/Z axes.",
                    id = args.device
                );
                return ExitCode::from(2);
            }
            other => {
                eprintln!("Device {} has unknown status {other}.", args.device);
                return ExitCode::from(1);
            }
        }

        if unsafe { (ffi.acquire_vjd)(args.device) } == 0 {
            let st = unsafe { (ffi.get_vjd_status)(args.device) };
            eprintln!("AcquireVJD({}) failed (post-status={st}).", args.device);
            return ExitCode::from(2);
        }

        let post = unsafe { (ffi.get_vjd_status)(args.device) };
        if post != VJD_STAT_OWN {
            unsafe { (ffi.relinquish_vjd)(args.device) };
            eprintln!("Acquire reported success but status is {post} (expected OWN=0).");
            return ExitCode::from(2);
        }

        // Verify X/Y/Z by probing SetAxis to center. SetAxis returns FALSE
        // when the axis is not configured.
        let center = (AXIS_MIN + AXIS_MAX) / 2;
        for (axis, name) in [
            (STEER_AXIS, "X (steering)"),
            (THROTTLE_AXIS, "Y (throttle)"),
            (BRAKE_AXIS, "Z (brake)"),
        ] {
            if unsafe { (ffi.set_axis)(center, args.device, axis) } == 0 {
                unsafe { (ffi.relinquish_vjd)(args.device) };
                eprintln!(
                    "Device {} missing axis {name}. Reconfigure in vJoyConf.",
                    args.device
                );
                return ExitCode::from(1);
            }
        }

        println!(
            "vJoy ok — device {} acquired (direct FFI), X/Y/Z axes present, range {}..={}",
            args.device, AXIS_MIN, AXIS_MAX
        );

        if args.dry_run {
            println!("--dry-run: skipping send loop");
            unsafe { (ffi.relinquish_vjd)(args.device) };
            return ExitCode::from(0);
        }

        // -- Ctrl-C handler -----------------------------------------------
        let aborted = Arc::new(AtomicBool::new(false));
        {
            let flag = aborted.clone();
            let _ = ctrlc::set_handler(move || flag.store(true, Ordering::SeqCst));
        }

        // -- JSON sink ----------------------------------------------------
        let mut json_writer = if args.json {
            match open_json_sink() {
                Ok((path, w)) => {
                    println!("--json: writing JSONL to {}", path.display());
                    Some(w)
                }
                Err(e) => {
                    eprintln!("could not open JSONL sink: {e}");
                    unsafe { (ffi.relinquish_vjd)(args.device) };
                    return ExitCode::from(1);
                }
            }
        } else {
            None
        };

        // -- Send loop ----------------------------------------------------
        let total_ticks = (args.duration as u64) * (args.rate_hz as u64);
        let tick_period = Duration::from_nanos(1_000_000_000u64 / args.rate_hz as u64);
        let duration_f = args.duration as f64;
        let rate_f = args.rate_hz as f64;
        let t0 = Instant::now();
        let mut last_progress_sec: i64 = -1;

        for tick in 0..total_ticks {
            if aborted.load(Ordering::SeqCst) {
                println!("[aborted] Releasing device...");
                cleanup_center(&ffi, args.device);
                return ExitCode::from(130);
            }

            let t = tick as f64 / rate_f;
            let (steer_n, throttle_n, brake_n) = compute_sample(
                pattern,
                t,
                duration_f,
                args.steer_amp,
                args.steer_freq,
                args.throttle,
                args.brake,
            );
            let steer_raw = map_signed_to_raw(steer_n);
            let throttle_raw = map_unsigned_to_raw(throttle_n);
            let brake_raw = map_unsigned_to_raw(brake_n);

            // SetAxis writes directly to the device — no UpdateVJD buffering.
            unsafe {
                if (ffi.set_axis)(steer_raw, args.device, STEER_AXIS) == 0
                    || (ffi.set_axis)(throttle_raw, args.device, THROTTLE_AXIS) == 0
                    || (ffi.set_axis)(brake_raw, args.device, BRAKE_AXIS) == 0
                {
                    eprintln!("SetAxis failed mid-loop (device released or axis missing?)");
                    cleanup_center(&ffi, args.device);
                    return ExitCode::from(1);
                }
            }

            let current_sec = t.floor() as i64;
            if current_sec > last_progress_sec {
                last_progress_sec = current_sec;
                println!(
                    "progress: t={:.1}s steer={:+.2} throttle={:.2} brake={:.2} \
                     raw=({},{},{})",
                    t, steer_n, throttle_n, brake_n, steer_raw, throttle_raw, brake_raw
                );
            }

            if let Some(w) = json_writer.as_mut() {
                let line = serde_json::json!({
                    "ts_ms": now_ms(),
                    "tick": tick,
                    "t": t,
                    "steer_norm": steer_n,
                    "throttle_norm": throttle_n,
                    "brake_norm": brake_n,
                    "steer_raw": steer_raw,
                    "throttle_raw": throttle_raw,
                    "brake_raw": brake_raw,
                });
                let _ = writeln!(w, "{line}");
            }

            let target = t0 + tick_period * (tick as u32 + 1);
            let now = Instant::now();
            if target > now {
                std::thread::sleep(target - now);
            }
        }

        cleanup_center(&ffi, args.device);
        if let Some(mut w) = json_writer {
            let _ = w.flush();
        }
        println!(
            "done. duration={}s ticks={} elapsed={:.2}s",
            args.duration,
            total_ticks,
            t0.elapsed().as_secs_f64()
        );
        ExitCode::from(0)
    }

    fn cleanup_center(ffi: &VJoyFfi, device_id: u32) {
        let center = (AXIS_MIN + AXIS_MAX) / 2;
        unsafe {
            let _ = (ffi.set_axis)(center, device_id, STEER_AXIS);
            let _ = (ffi.set_axis)(AXIS_MIN, device_id, THROTTLE_AXIS);
            let _ = (ffi.set_axis)(AXIS_MIN, device_id, BRAKE_AXIS);
            (ffi.relinquish_vjd)(device_id);
        }
    }

    fn open_json_sink() -> std::io::Result<(PathBuf, BufWriter<File>)> {
        let ts = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);
        // Best-effort YYYYMMDD-HHMMSS via chrono-less arithmetic would be
        // fragile; the unix seconds suffix is unambiguous and collision-free.
        let dir = PathBuf::from("outputs/active");
        std::fs::create_dir_all(&dir)?;
        let path = dir.join(format!("vjoy_probe_{ts}.jsonl"));
        let f = File::create(&path)?;
        Ok((path, BufWriter::new(f)))
    }

    fn now_ms() -> u128 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_millis())
            .unwrap_or(0)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn args_with(device: u32) -> Args {
        Args {
            device,
            duration: 1,
            pattern: "sine".into(),
            rate_hz: 50,
            steer_amp: 0.5,
            steer_freq: 0.5,
            throttle: 0.0,
            brake: 0.0,
            dry_run: true,
            json: false,
        }
    }

    #[test]
    fn validate_rejects_device_zero() {
        assert!(validate(&args_with(0)).is_err());
    }

    #[test]
    fn validate_rejects_device_seventeen() {
        assert!(validate(&args_with(17)).is_err());
    }

    #[test]
    fn validate_accepts_device_one_and_sixteen() {
        assert!(validate(&args_with(1)).is_ok());
        assert!(validate(&args_with(16)).is_ok());
    }

    #[test]
    fn validate_rejects_unknown_pattern() {
        let mut a = args_with(1);
        a.pattern = "spiral".into();
        assert!(validate(&a).is_err());
    }

    #[test]
    fn map_signed_endpoints_and_center() {
        assert_eq!(map_signed_to_raw(-1.0), AXIS_MIN);
        assert_eq!(map_signed_to_raw(1.0), AXIS_MAX);
        // Range 0..=32767 has fractional midpoint 16383.5; rounds to 16384.
        let center = ((AXIS_MIN + AXIS_MAX) as f64 / 2.0).round() as i32;
        assert_eq!(map_signed_to_raw(0.0), center);
    }

    #[test]
    fn map_signed_clamps_overrange() {
        assert_eq!(map_signed_to_raw(-2.5), AXIS_MIN);
        assert_eq!(map_signed_to_raw(2.5), AXIS_MAX);
    }

    #[test]
    fn map_unsigned_endpoints() {
        assert_eq!(map_unsigned_to_raw(0.0), AXIS_MIN);
        assert_eq!(map_unsigned_to_raw(1.0), AXIS_MAX);
    }

    #[test]
    fn sine_pattern_starts_at_zero() {
        let (s, _, _) = compute_sample(Pattern::Sine, 0.0, 10.0, 0.5, 0.5, 0.0, 0.0);
        assert!(s.abs() < 1e-9, "sin(0) should be 0, got {s}");
    }

    #[test]
    fn sine_pattern_peaks_at_quarter_period() {
        // freq=0.5 Hz, quarter period = 0.5s
        let (s, _, _) = compute_sample(Pattern::Sine, 0.5, 10.0, 0.5, 0.5, 0.0, 0.0);
        assert!((s - 0.5).abs() < 1e-6, "sin peak should be +amp, got {s}");
    }

    #[test]
    fn sine_pattern_trough_at_three_quarters() {
        let (s, _, _) = compute_sample(Pattern::Sine, 1.5, 10.0, 0.5, 0.5, 0.0, 0.0);
        assert!((s + 0.5).abs() < 1e-6, "sin trough should be -amp, got {s}");
    }

    #[test]
    fn ramp_pattern_peaks_at_quarter() {
        let (s, _, _) = compute_sample(Pattern::Ramp, 2.5, 10.0, 1.0, 0.5, 0.0, 0.0);
        assert!(
            (s - 1.0).abs() < 1e-6,
            "ramp peak at t/dur=0.25 should be +1, got {s}"
        );
    }

    #[test]
    fn ramp_pattern_zero_at_start() {
        let (s, _, _) = compute_sample(Pattern::Ramp, 0.0, 10.0, 1.0, 0.5, 0.0, 0.0);
        assert!(s.abs() < 1e-9);
    }

    #[test]
    fn ramp_pattern_trough_at_three_quarter() {
        let (s, _, _) = compute_sample(Pattern::Ramp, 7.5, 10.0, 1.0, 0.5, 0.0, 0.0);
        assert!(
            (s + 1.0).abs() < 1e-6,
            "ramp trough at t/dur=0.75 should be -1, got {s}"
        );
    }

    #[test]
    fn center_hold_always_zero() {
        for t in [0.0, 1.0, 3.7, 9.9] {
            let (s, _, _) = compute_sample(Pattern::CenterHold, t, 10.0, 0.5, 0.5, 0.0, 0.0);
            assert_eq!(s, 0.0);
        }
    }

    #[test]
    fn throttle_brake_pass_through() {
        let (_, th, br) = compute_sample(Pattern::Sine, 0.0, 10.0, 0.5, 0.5, 0.3, 0.7);
        assert_eq!(th, 0.3);
        assert_eq!(br, 0.7);
    }
}
