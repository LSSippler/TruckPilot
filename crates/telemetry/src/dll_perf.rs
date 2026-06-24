//! Reader for `Local\TruckPilotDllPerf` — layout must match `telemetry-dll/src/frame_perf.rs`.

use std::ffi::OsStr;
use std::os::windows::ffi::OsStrExt;

/// Magic value (`TPPF`) identifying a valid DLL perf snapshot.
pub const DLL_PERF_MAGIC: u32 = 0x4650_5054;
/// Snapshot layout version written by the telemetry DLL.
/// v5 adds the diag-level bisect fields (`diag_*` + `*_component_count`).
pub const DLL_PERF_VERSION: u32 = 5;
/// Windows file-mapping name for the perf snapshot region.
pub const DLL_PERF_SHM_NAME: &str = "Local\\TruckPilotDllPerf";
/// Number of timing buckets in [`DllPerfSnapshot::buckets`].
pub const PERF_BUCKET_COUNT: usize = 7;

/// Human-readable names for each perf bucket, in snapshot order.
pub const PERF_BUCKET_NAMES: [&str; PERF_BUCKET_COUNT] = [
    "frame_cb_total",
    "on_frame_event",
    "telemetry_shm_write",
    "ready_event_set",
    "dispatch_notify_frame_tick",
    "route_bb_frame_write",
    "input_event_cb",
];

/// Aggregated QPC timing stats for one instrumented code path.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PerfBucketSnapshot {
    /// Number of recorded samples.
    pub count: u64,
    /// Sum of sample durations in microseconds.
    pub total_us: u64,
    /// Maximum sample duration in microseconds.
    pub max_us: u64,
    /// Most recent sample duration in microseconds.
    pub last_us: u64,
    /// Samples at or above 100 µs.
    pub over_100us: u64,
    /// Samples at or above 500 µs.
    pub over_500us: u64,
    /// Samples at or above 1000 µs.
    pub over_1000us: u64,
    /// Samples at or above 2000 µs.
    pub over_2000us: u64,
    /// Samples at or above 5000 µs.
    pub over_5000us: u64,
}

/// Live perf counters and bucket snapshots published by the telemetry DLL.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DllPerfSnapshot {
    /// Must equal [`DLL_PERF_MAGIC`].
    pub magic: u32,
    /// Must equal [`DLL_PERF_VERSION`].
    pub version: u32,
    /// Monotonic publish sequence incremented on each snapshot write.
    pub sequence: u32,
    /// Per-path timing aggregates, indexed by [`PERF_BUCKET_NAMES`].
    pub buckets: [PerfBucketSnapshot; PERF_BUCKET_COUNT],
    /// Total telemetry frame callbacks observed.
    pub frame_cb_count: u32,
    /// Telemetry SHM writes performed from the frame callback.
    pub shm_write_count: u32,
    /// Ready-event signals sent after SHM writes.
    pub ready_event_set_count: u32,
    /// Route-blackboard writes from the frame path.
    pub route_bb_frame_write_count: u32,
    /// Worker notify calls (`notify_frame_tick`).
    pub notify_frame_tick_count: u32,
    /// Worker wake events that actually called `SetEvent`.
    pub worker_wake_set_event_count: u32,
    /// Worker wake requests coalesced because resolve was already pending.
    pub worker_pending_already_set_count: u32,
    /// Resolver worker walk iterations executed.
    pub worker_walk_count: u32,
    /// Worker iterations skipped because the resolver was parked.
    pub worker_parked_skip_count: u32,
    /// Route resolver attempts recorded by the worker.
    pub resolver_attempts: u32,
    /// Pattern scans performed by the resolver worker.
    pub pattern_scan_count: u32,
    /// Frame ticks skipped without worker wake because resolver was parked.
    pub worker_parked_no_wake_count: u32,
    /// Route tick dispatches from the frame callback (including off suppress).
    pub route_tick_dispatch_count: u32,
    /// Off-mode dispatches that stopped before `notify_frame_tick`.
    pub off_mode_notify_suppressed_count: u32,
    /// Skipped duplicate RouteBlackboard frame writes within the same frame/tick.
    pub route_bb_frame_write_suppressed_duplicate_count: u32,
    /// `1` when input plugin registered, `0` when disabled via opt-out file.
    pub input_enabled: u32,
    /// Total SCS input event callbacks observed.
    pub input_event_cb_count: u32,
    /// Dispatch ticks while resolver mode is off.
    pub dispatch_mode_off_count: u32,
    /// Dispatch ticks where reset or enable-generation change was pending.
    pub dispatch_wake_exception_count: u32,
    /// Off-mode dispatches that took the hard suppress gate.
    pub dispatch_off_gate_taken_count: u32,
    /// Dispatches that reached `notify_frame_tick`.
    pub dispatch_notify_called_count: u32,
    /// Dispatches that forced a worker wake via notify.
    pub dispatch_force_wake_count: u32,
    /// Enable-generation changes consumed on dispatch.
    pub dispatch_enable_generation_changed_count: u32,
    /// Resolver reset flags consumed on dispatch.
    pub dispatch_reset_exception_count: u32,
    /// `1` when `truckpilot_minimal_telemetry.enable` is active.
    pub minimal_telemetry_enabled: u32,
    /// Frame callbacks handled by the minimal telemetry hotpath.
    pub minimal_frame_cb_count: u32,

    // --- v5: diag-level bisect fields (mirror telemetry-dll/frame_perf.rs) ---
    /// Active diag level code (0=load_only .. 9=normal_default_off).
    pub diag_level_code: u32,
    /// `1` when SCS callbacks are registered for the active level.
    pub diag_callbacks_registered: u32,
    /// `1` when the input plugin device is/will be registered.
    pub diag_input_registered: u32,
    /// `1` when the per-frame telemetry SHM full-copy runs.
    pub diag_telemetry_shm_enabled: u32,
    /// `1` when the per-frame READY_EVENT SetEvent runs.
    pub diag_ready_event_enabled: u32,
    /// `1` when the per-frame RouteBlackboard frame write runs.
    pub diag_route_bb_frame_write_enabled: u32,
    /// `1` when the frame callback dispatches route ticks.
    pub diag_dispatch_enabled: u32,
    /// `1` when the resolver worker thread may run.
    pub diag_worker_enabled: u32,
    /// Frames handled by the `callback_noop` diag hotpath.
    pub callback_noop_count: u32,
    /// Frames handled by the `callback_counter` diag hotpath.
    pub callback_counter_count: u32,
    /// Frames handled by the `callback_qpc` diag hotpath.
    pub callback_qpc_count: u32,
    /// Frames handled by the `perf_snapshot` diag hotpath.
    pub perf_snapshot_count: u32,
    /// Frames that ran the `telemetry_shm` diag component.
    pub telemetry_shm_component_count: u32,
    /// Frames that ran the `ready_event` diag component.
    pub ready_event_component_count: u32,
    /// Frames that ran the `route_bb` diag component.
    pub route_bb_component_count: u32,
}

/// Map a `diag_level_code` to its stable label (mirror of
/// `telemetry-dll/src/diag_level.rs::DiagLevel::label`).
pub fn diag_level_name(code: u32) -> &'static str {
    match code {
        0 => "load_only",
        1 => "init_only",
        2 => "callback_noop",
        3 => "callback_counter",
        4 => "callback_qpc",
        5 => "perf_snapshot",
        6 => "telemetry_shm",
        7 => "ready_event",
        8 => "route_bb",
        9 => "normal_default_off",
        _ => "unknown",
    }
}

/// Computes the integer average microseconds per sample.
pub fn avg_us(total: u64, count: u64) -> u64 {
    if count == 0 {
        0
    } else {
        total / count
    }
}

/// Formats a single-line message when perf SHM is unavailable or invalid.
pub fn format_perf_unavailable(reason: &str) -> String {
    format!("perf: unavailable ({reason})")
}

#[cfg(windows)]
extern "system" {
    fn CloseHandle(hObject: isize) -> i32;
    fn UnmapViewOfFile(lpBaseAddress: *const core::ffi::c_void) -> i32;
}

#[cfg(windows)]
/// Mapped reader for [`DLL_PERF_SHM_NAME`]. Unmaps the view and closes the
/// mapping handle on drop.
pub struct DllPerfReader {
    view: *const DllPerfSnapshot,
    handle: isize,
}

#[cfg(windows)]
impl DllPerfReader {
    /// Opens the named perf mapping for read-only access.
    pub fn open() -> Result<Self, String> {
        type HANDLE = isize;
        const FILE_MAP_READ: u32 = 0x0004;

        extern "system" {
            fn OpenFileMappingW(
                dwDesiredAccess: u32,
                bInheritHandle: i32,
                lpName: *const u16,
            ) -> HANDLE;
            fn MapViewOfFile(
                hFileMappingObject: HANDLE,
                dwDesiredAccess: u32,
                dwFileOffsetHigh: u32,
                dwFileOffsetLow: u32,
                dwNumberOfBytesToMap: usize,
            ) -> *mut core::ffi::c_void;
        }

        fn wide(s: &str) -> Vec<u16> {
            OsStr::new(s).encode_wide().chain([0]).collect()
        }

        unsafe {
            let name = wide(DLL_PERF_SHM_NAME);
            let handle = OpenFileMappingW(FILE_MAP_READ, 0, name.as_ptr());
            if handle == 0 {
                return Err(format!("OpenFileMappingW failed for {DLL_PERF_SHM_NAME}"));
            }
            let size = std::mem::size_of::<DllPerfSnapshot>();
            let view = MapViewOfFile(handle, FILE_MAP_READ, 0, 0, size);
            if view.is_null() {
                // Don't leak the mapping handle when the view fails to map.
                CloseHandle(handle);
                return Err("MapViewOfFile failed for DllPerf".into());
            }
            Ok(Self {
                view: view as *const DllPerfSnapshot,
                handle,
            })
        }
    }

    /// Reads the current snapshot if magic/version are valid.
    pub fn read(&self) -> Option<DllPerfSnapshot> {
        unsafe {
            let snap = std::ptr::read(self.view);
            if snap.magic != DLL_PERF_MAGIC || snap.version != DLL_PERF_VERSION {
                return None;
            }
            Some(snap)
        }
    }
}

#[cfg(windows)]
impl Drop for DllPerfReader {
    fn drop(&mut self) {
        unsafe {
            if !self.view.is_null() {
                let _ = UnmapViewOfFile(self.view as *const core::ffi::c_void);
            }
            if self.handle != 0 {
                let _ = CloseHandle(self.handle);
            }
        }
    }
}

#[cfg(not(windows))]
/// Stub reader used on non-Windows targets.
pub struct DllPerfReader;

#[cfg(not(windows))]
impl DllPerfReader {
    /// Always fails because perf SHM is Windows-only.
    pub fn open() -> Result<Self, String> {
        Err("DllPerfReader is Windows-only".into())
    }

    /// Always returns `None` on non-Windows targets.
    pub fn read(&self) -> Option<DllPerfSnapshot> {
        None
    }
}

/// Renders human-readable perf diagnostic lines from a DLL snapshot.
pub fn format_perf_lines(snap: &DllPerfSnapshot) -> Vec<String> {
    let frame = &snap.buckets[0];
    let input_enabled = snap.input_enabled != 0;
    let mut lines = vec![
        "perf:".into(),
        format!("diag_level={}", diag_level_name(snap.diag_level_code)),
        format!("diag_level_code={}", snap.diag_level_code),
        format!(
            "diag_callbacks_registered={}",
            snap.diag_callbacks_registered != 0
        ),
        format!("diag_input_registered={}", snap.diag_input_registered != 0),
        format!(
            "diag_telemetry_shm_enabled={}",
            snap.diag_telemetry_shm_enabled != 0
        ),
        format!(
            "diag_ready_event_enabled={}",
            snap.diag_ready_event_enabled != 0
        ),
        format!(
            "diag_route_bb_frame_write_enabled={}",
            snap.diag_route_bb_frame_write_enabled != 0
        ),
        format!("diag_dispatch_enabled={}", snap.diag_dispatch_enabled != 0),
        format!("diag_worker_enabled={}", snap.diag_worker_enabled != 0),
        format!("callback_noop_count={}", snap.callback_noop_count),
        format!("callback_counter_count={}", snap.callback_counter_count),
        format!("callback_qpc_count={}", snap.callback_qpc_count),
        format!("perf_snapshot_count={}", snap.perf_snapshot_count),
        format!(
            "telemetry_shm_component_count={}",
            snap.telemetry_shm_component_count
        ),
        format!(
            "ready_event_component_count={}",
            snap.ready_event_component_count
        ),
        format!("route_bb_component_count={}", snap.route_bb_component_count),
        format!("frame_cb_us_last={}", frame.last_us),
        format!("frame_cb_us_max={}", frame.max_us),
        format!("frame_cb_us_avg={}", avg_us(frame.total_us, frame.count)),
        format!("frame_cb_over_1000us={}", frame.over_1000us),
        format!("telemetry_shm_write_us_max={}", snap.buckets[2].max_us),
        format!("route_bb_frame_write_us_max={}", snap.buckets[5].max_us),
        format!("ready_event_set_us_max={}", snap.buckets[3].max_us),
        format!("notify_frame_tick_us_max={}", snap.buckets[4].max_us),
        format!("on_frame_event_us_max={}", snap.buckets[1].max_us),
        format!("input_event_cb_us_max={}", snap.buckets[6].max_us),
        format!("frame_cb_count={}", snap.frame_cb_count),
        format!(
            "minimal_telemetry_enabled={}",
            snap.minimal_telemetry_enabled != 0
        ),
        format!("minimal_frame_cb_count={}", snap.minimal_frame_cb_count),
        format!("route_tick_dispatch_count={}", snap.route_tick_dispatch_count),
        format!("notify_frame_tick_count={}", snap.notify_frame_tick_count),
        format!(
            "off_mode_notify_suppressed_count={}",
            snap.off_mode_notify_suppressed_count
        ),
        format!("shm_write_count={}", snap.shm_write_count),
        format!("ready_event_set_count={}", snap.ready_event_set_count),
        format!(
            "route_bb_frame_write_count={}",
            snap.route_bb_frame_write_count
        ),
        format!(
            "route_bb_frame_write_suppressed_duplicate_count={}",
            snap.route_bb_frame_write_suppressed_duplicate_count
        ),
        format!(
            "worker_wake_set_event_count={}",
            snap.worker_wake_set_event_count
        ),
        format!(
            "worker_pending_already_set_count={}",
            snap.worker_pending_already_set_count
        ),
        format!("worker_walk_count={}", snap.worker_walk_count),
        format!("worker_parked_skip_count={}", snap.worker_parked_skip_count),
        format!(
            "worker_parked_no_wake_count={}",
            snap.worker_parked_no_wake_count
        ),
        format!("resolver_attempts={}", snap.resolver_attempts),
        format!("pattern_scan_count={}", snap.pattern_scan_count),
        format!("input_enabled={input_enabled}"),
        format!("input_event_cb_count={}", snap.input_event_cb_count),
        format!("dispatch_mode_off_count={}", snap.dispatch_mode_off_count),
        format!(
            "dispatch_wake_exception_count={}",
            snap.dispatch_wake_exception_count
        ),
        format!(
            "dispatch_off_gate_taken_count={}",
            snap.dispatch_off_gate_taken_count
        ),
        format!(
            "dispatch_notify_called_count={}",
            snap.dispatch_notify_called_count
        ),
        format!("dispatch_force_wake_count={}", snap.dispatch_force_wake_count),
        format!(
            "dispatch_enable_generation_changed_count={}",
            snap.dispatch_enable_generation_changed_count
        ),
        format!(
            "dispatch_reset_exception_count={}",
            snap.dispatch_reset_exception_count
        ),
    ];
    for (name, bucket) in PERF_BUCKET_NAMES.iter().zip(snap.buckets.iter()) {
        lines.push(format!(
            "perf_bucket_{name}_over_5000us={}",
            bucket.over_5000us
        ));
    }
    lines
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Reader/writer byte-layout tripwire. Must equal the identical assertion in
    /// `telemetry-dll/src/frame_perf.rs`. If you change `DllPerfSnapshot`, change
    /// BOTH structs, bump `DLL_PERF_VERSION` on BOTH sides, and update both numbers.
    #[test]
    fn snapshot_layout_size_and_align_are_stable() {
        assert_eq!(std::mem::size_of::<DllPerfSnapshot>(), 688);
        assert_eq!(std::mem::align_of::<DllPerfSnapshot>(), 8);
    }

    #[test]
    fn diag_level_name_maps_all_codes() {
        assert_eq!(diag_level_name(0), "load_only");
        assert_eq!(diag_level_name(2), "callback_noop");
        assert_eq!(diag_level_name(6), "telemetry_shm");
        assert_eq!(diag_level_name(9), "normal_default_off");
        assert_eq!(diag_level_name(42), "unknown");
    }

    #[test]
    fn format_perf_lines_renders_v5_diag_fields() {
        let snap = DllPerfSnapshot {
            magic: DLL_PERF_MAGIC,
            version: DLL_PERF_VERSION,
            diag_level_code: 4, // callback_qpc
            diag_callbacks_registered: 1,
            diag_input_registered: 0,
            diag_telemetry_shm_enabled: 0,
            diag_ready_event_enabled: 0,
            diag_route_bb_frame_write_enabled: 0,
            diag_dispatch_enabled: 0,
            diag_worker_enabled: 0,
            callback_qpc_count: 1234,
            ..Default::default()
        };
        let joined = format_perf_lines(&snap).join("\n");
        assert!(joined.contains("diag_level=callback_qpc"));
        assert!(joined.contains("diag_level_code=4"));
        assert!(joined.contains("diag_callbacks_registered=true"));
        assert!(joined.contains("diag_input_registered=false"));
        assert!(joined.contains("diag_dispatch_enabled=false"));
        assert!(joined.contains("diag_worker_enabled=false"));
        assert!(joined.contains("callback_qpc_count=1234"));
        // No duplicate RouteBlackboard frame-write line.
        assert_eq!(
            joined.matches("route_bb_frame_write_count=").count(),
            1
        );
    }

    #[test]
    fn format_perf_unavailable_wraps_reason() {
        let line = format_perf_unavailable("OpenFileMappingW failed for Local\\TruckPilotDllPerf");
        assert_eq!(
            line,
            "perf: unavailable (OpenFileMappingW failed for Local\\TruckPilotDllPerf)"
        );
    }

    #[test]
    fn format_perf_lines_renders_all_buckets_and_counters() {
        let mut snap = DllPerfSnapshot {
            magic: DLL_PERF_MAGIC,
            version: DLL_PERF_VERSION,
            route_tick_dispatch_count: 11,
            off_mode_notify_suppressed_count: 10,
            route_bb_frame_write_suppressed_duplicate_count: 2,
            input_enabled: 1,
            input_event_cb_count: 5,
            dispatch_mode_off_count: 100,
            dispatch_wake_exception_count: 0,
            dispatch_off_gate_taken_count: 100,
            dispatch_notify_called_count: 0,
            dispatch_force_wake_count: 0,
            dispatch_enable_generation_changed_count: 0,
            dispatch_reset_exception_count: 0,
            minimal_telemetry_enabled: 1,
            minimal_frame_cb_count: 42,
            ..Default::default()
        };
        snap.buckets[0].last_us = 42;
        snap.buckets[0].max_us = 100;
        snap.buckets[0].count = 2;
        snap.buckets[0].total_us = 142;
        snap.worker_wake_set_event_count = 3;
        snap.pattern_scan_count = 9;
        let lines = format_perf_lines(&snap);
        let joined = lines.join("\n");

        assert!(lines.iter().any(|l| l.starts_with("perf:")));
        assert!(joined.contains("frame_cb_us_last=42"));
        assert!(joined.contains("route_tick_dispatch_count=11"));
        assert!(joined.contains("minimal_telemetry_enabled=true"));
        assert!(joined.contains("minimal_frame_cb_count=42"));
        assert!(joined.contains("off_mode_notify_suppressed_count=10"));
        assert!(joined.contains("route_bb_frame_write_suppressed_duplicate_count=2"));
        assert!(joined.contains("input_enabled=true"));
        assert!(joined.contains("input_event_cb_count=5"));
        assert!(joined.contains("dispatch_mode_off_count=100"));
        assert!(joined.contains("dispatch_off_gate_taken_count=100"));
        assert!(joined.contains("dispatch_notify_called_count=0"));

        for name in PERF_BUCKET_NAMES {
            assert!(
                joined.contains(&format!("perf_bucket_{name}_over_5000us=")),
                "missing bucket line for {name}"
            );
        }

        // v5 diag fields render with default (normal-ish) values.
        assert!(joined.contains("diag_level="));
        assert!(joined.contains("diag_level_code="));

        for key in [
            "frame_cb_count=",
            "notify_frame_tick_count=",
            "worker_wake_set_event_count=",
            "worker_pending_already_set_count=",
            "worker_walk_count=",
            "worker_parked_skip_count=",
            "worker_parked_no_wake_count=",
            "resolver_attempts=",
            "pattern_scan_count=",
            "route_bb_frame_write_count=",
            "dispatch_mode_off_count=",
            "dispatch_wake_exception_count=",
            "dispatch_off_gate_taken_count=",
            "dispatch_notify_called_count=",
            "dispatch_force_wake_count=",
            "dispatch_enable_generation_changed_count=",
            "dispatch_reset_exception_count=",
            "minimal_telemetry_enabled=",
            "minimal_frame_cb_count=",
        ] {
            assert!(joined.contains(key), "missing counter line for {key}");
        }
    }
}
