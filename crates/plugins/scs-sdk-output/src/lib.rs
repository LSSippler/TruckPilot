//! SCS SDK Output plugin — writes arbitrated ControlOutput to the game's
//! native input system via the SCS Input Plugin API.
//!
//! ## How it works
//!
//! The companion `truckpilot_telemetry.dll` (loaded by ETS2 in-process) exports
//! both `scs_telemetry_init` and `scs_input_init`. In `scs_input_init` it:
//! 1. Creates a named Windows shared memory region `Local\TruckPilotControls` (32 B).
//! 2. Registers a semantical input device with ETS2.
//! 3. On each game frame, reads steering/throttle/brake/clutch from the SHM and
//!    feeds them to the game as native controller events.
//!
//! This plugin (running in the TruckPilot daemon, PostPhase) writes
//! `steering/throttle/brake` into that same SHM region.  When `active == 0`
//! the DLL emits no events and the game's own input (keyboard/wheel) is used.
//!
//! ## Blackboard keys written
//!
//! | Key | Value |
//! |---|---|
//! | `scs_sdk_output.connected` | "true"/"false" — SHM opened |
//! | `scs_sdk_output.active` | "true"/"false" — currently sending |
//! | `scs_sdk_output.last_error` | last error message (absent = none) |
//! | `scs_sdk_output.last_write_tick` | tick of most recent write |

use std::ffi::c_void;
use std::mem;

use truckpilot_plugin_api::{ControlOutput, Plugin, PluginContext, Telemetry, TickPhase};

// ---------------------------------------------------------------------------
// Control SHM constants (must match telemetry-dll/src/lib.rs exactly)
// ---------------------------------------------------------------------------

pub const CTRL_SHM_MAGIC: u32 = 0x54504354; // "TPCT"
pub const CTRL_SHM_VERSION: u32 = 1;
const CTRL_SHM_NAME: &str = "Local\\TruckPilotControls";

/// Control SHM layout written by this plugin, read by the DLL's per-frame callback.
/// Must stay byte-for-byte identical to `ShmControlLayout` in
/// `crates/telemetry-dll/src/lib.rs`.
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub struct ShmControlLayout {
    pub magic: u32,
    pub version: u32,
    pub sequence: u32,
    pub active: u32,
    pub steering: f32,
    pub throttle: f32,
    pub brake: f32,
    pub clutch: f32,
}

const _: () = {
    assert!(mem::offset_of!(ShmControlLayout, magic) == 0);
    assert!(mem::offset_of!(ShmControlLayout, version) == 4);
    assert!(mem::offset_of!(ShmControlLayout, sequence) == 8);
    assert!(mem::offset_of!(ShmControlLayout, active) == 12);
    assert!(mem::offset_of!(ShmControlLayout, steering) == 16);
    assert!(mem::offset_of!(ShmControlLayout, throttle) == 20);
    assert!(mem::offset_of!(ShmControlLayout, brake) == 24);
    assert!(mem::offset_of!(ShmControlLayout, clutch) == 28);
    assert!(mem::size_of::<ShmControlLayout>() == 32);
};

// ---------------------------------------------------------------------------
// Windows SHM writer (platform-specific)
// ---------------------------------------------------------------------------

// Windows API type aliases intentionally use the conventional all-caps names.
#[allow(clippy::upper_case_acronyms)]
#[cfg(windows)]
type HANDLE = isize;
#[allow(clippy::upper_case_acronyms)]
#[cfg(windows)]
type LPVOID = *mut c_void;
#[allow(clippy::upper_case_acronyms)]
#[cfg(windows)]
type LPCWSTR = *const u16;
#[allow(clippy::upper_case_acronyms)]
#[cfg(windows)]
type DWORD = u32;
#[allow(clippy::upper_case_acronyms)]
#[cfg(windows)]
type BOOL = i32;

#[cfg(windows)]
const NULL_HANDLE: HANDLE = 0;
#[cfg(windows)]
const FILE_MAP_WRITE: DWORD = 2;

#[cfg(windows)]
extern "system" {
    fn OpenFileMappingW(dwDesiredAccess: DWORD, bInheritHandle: BOOL, lpName: LPCWSTR) -> HANDLE;
    fn MapViewOfFile(
        hFileMappingObject: HANDLE,
        dwDesiredAccess: DWORD,
        dwFileOffsetHigh: DWORD,
        dwFileOffsetLow: DWORD,
        dwNumberOfBytesToMap: usize,
    ) -> LPVOID;
    fn UnmapViewOfFile(lpBaseAddress: LPVOID) -> BOOL;
    fn CloseHandle(hObject: HANDLE) -> BOOL;
}

#[cfg(windows)]
fn wide_str(s: &str) -> Vec<u16> {
    s.encode_utf16().chain(std::iter::once(0)).collect()
}

#[cfg(windows)]
pub struct ShmCtrlWriter {
    handle: HANDLE,
    ptr: *mut ShmControlLayout,
    seq: u32,
}

#[cfg(windows)]
unsafe impl Send for ShmCtrlWriter {}
#[cfg(windows)]
unsafe impl Sync for ShmCtrlWriter {}

#[cfg(windows)]
impl ShmCtrlWriter {
    /// Open the control SHM region created by the DLL's `scs_input_init`.
    /// Returns `Err` if ETS2 is not running (DLL not yet loaded).
    pub fn open() -> Result<Self, String> {
        let name = wide_str(CTRL_SHM_NAME);
        let (shm_handle, shm_view) = unsafe {
            let h = OpenFileMappingW(FILE_MAP_WRITE, 0, name.as_ptr());
            if h == NULL_HANDLE {
                return Err("OpenFileMappingW failed — ETS2 not running or DLL not loaded".into());
            }
            let v = MapViewOfFile(h, FILE_MAP_WRITE, 0, 0, mem::size_of::<ShmControlLayout>());
            if v.is_null() {
                CloseHandle(h);
                return Err("MapViewOfFile failed".into());
            }
            // Read magic/version via read_unaligned (packed struct — direct field refs are UB).
            let raw = v as *const ShmControlLayout;
            let magic = std::ptr::read_unaligned(std::ptr::addr_of!((*raw).magic));
            let version = std::ptr::read_unaligned(std::ptr::addr_of!((*raw).version));
            if magic != CTRL_SHM_MAGIC || version != CTRL_SHM_VERSION {
                UnmapViewOfFile(v);
                CloseHandle(h);
                return Err(format!(
                    "SHM magic/version mismatch: got {:#010x} v={} (expected {:#010x} v={})",
                    magic, version, CTRL_SHM_MAGIC, CTRL_SHM_VERSION,
                ));
            }
            (h, v as *mut ShmControlLayout)
        };
        Ok(Self {
            handle: shm_handle,
            ptr: shm_view,
            seq: 0,
        })
    }

    /// Write a complete control frame. Increments the sequence counter.
    pub fn write(&mut self, active: bool, steering: f32, throttle: f32, brake: f32) {
        self.seq = self.seq.wrapping_add(1);
        unsafe {
            (*self.ptr).steering = steering;
            (*self.ptr).throttle = throttle;
            (*self.ptr).brake = brake;
            (*self.ptr).clutch = 0.0;
            (*self.ptr).sequence = self.seq;
            (*self.ptr).active = if active { 1 } else { 0 };
        }
    }

    /// Set active=0 (passthrough — game's own input takes over).
    pub fn write_idle(&mut self) {
        self.write(false, 0.0, 0.0, 0.0);
    }
}

#[cfg(windows)]
impl Drop for ShmCtrlWriter {
    fn drop(&mut self) {
        unsafe {
            if !self.ptr.is_null() {
                (*self.ptr).active = 0;
                UnmapViewOfFile(self.ptr as LPVOID);
            }
            if self.handle != NULL_HANDLE {
                CloseHandle(self.handle);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Plugin struct
// ---------------------------------------------------------------------------

pub struct ScsSdkOutputPlugin {
    #[cfg(windows)]
    shm: Option<ShmCtrlWriter>,
    failsafe_timeout_ms: u64,
    last_tick_count: u64,
    inactive: bool,
    reconnect_cooldown: u32,
    last_write_tick: u64,
}

impl Default for ScsSdkOutputPlugin {
    fn default() -> Self {
        Self {
            #[cfg(windows)]
            shm: None,
            failsafe_timeout_ms: 500,
            last_tick_count: 0,
            inactive: false,
            reconnect_cooldown: 0,
            last_write_tick: 0,
        }
    }
}

// ---------------------------------------------------------------------------
// Pure helper functions (unit-testable, no hardware)
// ---------------------------------------------------------------------------

pub fn should_emergency_brake(ctx: &PluginContext) -> bool {
    ctx.blackboard.get("safety.emergency_brake").as_deref() == Some("true")
}

pub fn is_autopilot_off(ctx: &PluginContext) -> bool {
    !matches!(
        ctx.blackboard.get("autopilot.state").as_deref(),
        Some("Engaging" | "Active" | "Paused" | "Fault")
    )
}

pub fn is_watchdog_expired(last_tick: u64, current_tick: u64, timeout_ms: u64) -> bool {
    let max_allowed = (timeout_ms / 20).max(1);
    current_tick.wrapping_sub(last_tick) > max_allowed
}

// ---------------------------------------------------------------------------
// Plugin impl
// ---------------------------------------------------------------------------

impl Plugin for ScsSdkOutputPlugin {
    fn name(&self) -> &str {
        "scs-sdk-output"
    }

    fn version(&self) -> &str {
        "0.1.0"
    }

    fn settings_schema(&self) -> &str {
        r#"{
  "type": "object",
  "properties": {
    "enabled": {
      "type": "boolean",
      "default": false,
      "description": "Enable SCS SDK controller output (requires DLL with scs_input_init)"
    },
    "failsafe_timeout_ms": {
      "type": "integer",
      "minimum": 100,
      "maximum": 5000,
      "default": 500,
      "description": "Watchdog timeout in milliseconds"
    }
  }
}"#
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        self.failsafe_timeout_ms = ctx
            .blackboard
            .get_f64("truckpilot.config.scs_sdk_output.failsafe_timeout_ms")
            .map(|v| (v as u64).clamp(100, 5_000))
            .unwrap_or(500);

        ctx.blackboard.set("scs_sdk_output.connected", "false");
        ctx.blackboard.set("scs_sdk_output.active", "false");
        ctx.blackboard.remove("scs_sdk_output.last_error");
        ctx.blackboard.set("scs_sdk_output.last_write_tick", "0");

        #[cfg(windows)]
        {
            match ShmCtrlWriter::open() {
                Ok(writer) => {
                    self.shm = Some(writer);
                    ctx.blackboard.set("scs_sdk_output.connected", "true");
                    tracing::info!("[scs-sdk-output] control SHM opened — ready");
                }
                Err(e) => {
                    ctx.blackboard.set("scs_sdk_output.last_error", &e);
                    tracing::warn!(
                        "[scs-sdk-output] SHM open failed (ETS2 not running?): {e} — will retry"
                    );
                }
            }
        }

        #[cfg(not(windows))]
        {
            self.inactive = true;
            tracing::info!("[scs-sdk-output] platform not supported — inactive");
        }
    }

    fn on_unload(&mut self) {
        #[cfg(windows)]
        if let Some(ref mut w) = self.shm {
            w.write_idle();
        }
        tracing::info!("[scs-sdk-output] unloaded — controller idled");
    }

    fn default_phase(&self) -> TickPhase {
        TickPhase::PostPhase
    }

    fn tick(
        &mut self,
        _telemetry: Option<&Telemetry>,
        output: &mut ControlOutput,
        ctx: &PluginContext,
    ) {
        if self.inactive {
            return;
        }

        // Emergency brake or autopilot off → send idle (passthrough to game)
        if should_emergency_brake(ctx) || is_autopilot_off(ctx) {
            self.write_idle(ctx);
            return;
        }

        // Watchdog
        if is_watchdog_expired(
            self.last_tick_count,
            ctx.tick_count,
            self.failsafe_timeout_ms,
        ) {
            tracing::warn!("[scs-sdk-output] watchdog expired — idling");
            self.write_idle(ctx);
            self.last_tick_count = ctx.tick_count;
            return;
        }
        self.last_tick_count = ctx.tick_count;

        #[cfg(windows)]
        self.tick_windows(output, ctx);
    }
}

// ---------------------------------------------------------------------------
// Windows-only tick (separated for cfg readability)
// ---------------------------------------------------------------------------

#[cfg(windows)]
impl ScsSdkOutputPlugin {
    fn tick_windows(&mut self, output: &mut ControlOutput, ctx: &PluginContext) {
        // Lazy reconnect: try every 50 ticks (~1 s at 50 Hz) while disconnected.
        if self.shm.is_none() {
            self.reconnect_cooldown = self.reconnect_cooldown.saturating_sub(1);
            if self.reconnect_cooldown == 0 {
                self.reconnect_cooldown = 50;
                match ShmCtrlWriter::open() {
                    Ok(writer) => {
                        self.shm = Some(writer);
                        ctx.blackboard.set("scs_sdk_output.connected", "true");
                        ctx.blackboard.remove("scs_sdk_output.last_error");
                        tracing::info!("[scs-sdk-output] control SHM reconnected");
                    }
                    Err(e) => {
                        ctx.blackboard.set("scs_sdk_output.last_error", &e);
                    }
                }
            }
            return;
        }

        let w = self.shm.as_mut().unwrap();
        let steer = output.steering.clamp(-1.0, 1.0) as f32;
        let throttle = output.throttle.clamp(0.0, 1.0) as f32;
        let brake = output.brake.clamp(0.0, 1.0) as f32;

        w.write(true, steer, throttle, brake);
        self.last_write_tick = ctx.tick_count;

        ctx.blackboard.set("scs_sdk_output.active", "true");
        ctx.blackboard
            .set("scs_sdk_output.last_write_tick", ctx.tick_count.to_string());
    }

    fn write_idle(&mut self, ctx: &PluginContext) {
        if let Some(ref mut w) = self.shm {
            w.write_idle();
        }
        ctx.blackboard.set("scs_sdk_output.active", "false");
    }
}

#[cfg(not(windows))]
impl ScsSdkOutputPlugin {
    fn write_idle(&mut self, ctx: &PluginContext) {
        ctx.blackboard.set("scs_sdk_output.active", "false");
    }
}

truckpilot_plugin_api::export_plugin!(ScsSdkOutputPlugin);

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ctrl_shm_magic_matches_dll() {
        assert_eq!(CTRL_SHM_MAGIC, 0x54504354);
    }

    #[test]
    fn ctrl_shm_version_is_1() {
        assert_eq!(CTRL_SHM_VERSION, 1);
    }

    #[test]
    fn ctrl_layout_size_is_32() {
        assert_eq!(mem::size_of::<ShmControlLayout>(), 32);
    }

    #[test]
    fn ctrl_layout_offsets() {
        assert_eq!(mem::offset_of!(ShmControlLayout, magic), 0);
        assert_eq!(mem::offset_of!(ShmControlLayout, active), 12);
        assert_eq!(mem::offset_of!(ShmControlLayout, steering), 16);
        assert_eq!(mem::offset_of!(ShmControlLayout, clutch), 28);
    }

    #[test]
    fn emergency_brake_detected() {
        let ctx = PluginContext::test();
        ctx.blackboard.set("safety.emergency_brake", "true");
        assert!(should_emergency_brake(&ctx));
    }

    #[test]
    fn no_emergency_brake_absent() {
        let ctx = PluginContext::test();
        assert!(!should_emergency_brake(&ctx));
    }

    #[test]
    fn autopilot_off_when_state_missing() {
        let ctx = PluginContext::test();
        assert!(is_autopilot_off(&ctx));
    }

    #[test]
    fn autopilot_not_off_when_active() {
        for state in ["Engaging", "Active", "Paused", "Fault"] {
            let ctx = PluginContext::test();
            ctx.blackboard.set("autopilot.state", state);
            assert!(
                !is_autopilot_off(&ctx),
                "expected not-off for state={state}"
            );
        }
    }

    #[test]
    fn watchdog_fires_after_timeout() {
        assert!(is_watchdog_expired(0, 26, 500));
    }

    #[test]
    fn watchdog_ok_within_timeout() {
        assert!(!is_watchdog_expired(0, 25, 500));
    }

    #[test]
    fn watchdog_handles_wraparound() {
        assert!(is_watchdog_expired(u64::MAX - 30, 1, 500));
    }
}
