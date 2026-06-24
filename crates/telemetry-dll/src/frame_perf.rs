//! Low-overhead QPC frame profiler + wake/write counters (aggregated only, no per-frame logs).

use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

/// Magic `TPPF` — TruckPilot perf snapshot.
pub const DLL_PERF_MAGIC: u32 = 0x4650_5054;
/// Bumped to 5 for the diag-level bisect fields (`diag_*` + `*_component_count`).
pub const DLL_PERF_VERSION: u32 = 5;
pub const DLL_PERF_SHM_NAME: &str = "Local\\TruckPilotDllPerf";

/// Number of [`PerfBucketId`] entries in the snapshot.
pub const PERF_BUCKET_COUNT: usize = 7;

#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PerfBucketId {
    FrameCbTotal = 0,
    OnFrameEvent = 1,
    TelemetryShmWrite = 2,
    ReadyEventSet = 3,
    DispatchNotifyFrameTick = 4,
    RouteBbFrameWrite = 5,
    InputEventCb = 6,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PerfBucketSnapshot {
    pub count: u64,
    pub total_us: u64,
    pub max_us: u64,
    pub last_us: u64,
    pub over_100us: u64,
    pub over_500us: u64,
    pub over_1000us: u64,
    pub over_2000us: u64,
    pub over_5000us: u64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DllPerfSnapshot {
    pub magic: u32,
    pub version: u32,
    pub sequence: u32,
    pub buckets: [PerfBucketSnapshot; PERF_BUCKET_COUNT],
    pub frame_cb_count: u32,
    pub shm_write_count: u32,
    pub ready_event_set_count: u32,
    pub route_bb_frame_write_count: u32,
    pub notify_frame_tick_count: u32,
    pub worker_wake_set_event_count: u32,
    pub worker_pending_already_set_count: u32,
    pub worker_walk_count: u32,
    pub worker_parked_skip_count: u32,
    pub resolver_attempts: u32,
    pub pattern_scan_count: u32,
    pub worker_parked_no_wake_count: u32,
    pub route_tick_dispatch_count: u32,
    pub off_mode_notify_suppressed_count: u32,
    pub route_bb_frame_write_suppressed_duplicate_count: u32,
    pub input_enabled: u32,
    pub input_event_cb_count: u32,
    pub dispatch_mode_off_count: u32,
    pub dispatch_wake_exception_count: u32,
    pub dispatch_off_gate_taken_count: u32,
    pub dispatch_notify_called_count: u32,
    pub dispatch_force_wake_count: u32,
    pub dispatch_enable_generation_changed_count: u32,
    pub dispatch_reset_exception_count: u32,
    pub minimal_telemetry_enabled: u32,
    pub minimal_frame_cb_count: u32,

    // --- v5: diag-level bisect fields -------------------------------------
    /// Active [`crate::diag_level::DiagLevel`] code (0=load_only .. 9=normal).
    pub diag_level_code: u32,
    pub diag_callbacks_registered: u32,
    pub diag_input_registered: u32,
    pub diag_telemetry_shm_enabled: u32,
    pub diag_ready_event_enabled: u32,
    pub diag_route_bb_frame_write_enabled: u32,
    pub diag_dispatch_enabled: u32,
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

pub struct PerfBucket {
    count: AtomicU64,
    total_us: AtomicU64,
    max_us: AtomicU64,
    last_us: AtomicU64,
    over_100us: AtomicU64,
    over_500us: AtomicU64,
    over_1000us: AtomicU64,
    over_2000us: AtomicU64,
    over_5000us: AtomicU64,
}

impl PerfBucket {
    pub const fn new() -> Self {
        Self {
            count: AtomicU64::new(0),
            total_us: AtomicU64::new(0),
            max_us: AtomicU64::new(0),
            last_us: AtomicU64::new(0),
            over_100us: AtomicU64::new(0),
            over_500us: AtomicU64::new(0),
            over_1000us: AtomicU64::new(0),
            over_2000us: AtomicU64::new(0),
            over_5000us: AtomicU64::new(0),
        }
    }

    pub fn record(&self, us: u64) {
        self.count.fetch_add(1, Ordering::Relaxed);
        self.total_us.fetch_add(us, Ordering::Relaxed);
        update_max(&self.max_us, us);
        self.last_us.store(us, Ordering::Relaxed);
        if us >= 100 {
            self.over_100us.fetch_add(1, Ordering::Relaxed);
        }
        if us >= 500 {
            self.over_500us.fetch_add(1, Ordering::Relaxed);
        }
        if us >= 1000 {
            self.over_1000us.fetch_add(1, Ordering::Relaxed);
        }
        if us >= 2000 {
            self.over_2000us.fetch_add(1, Ordering::Relaxed);
        }
        if us >= 5000 {
            self.over_5000us.fetch_add(1, Ordering::Relaxed);
        }
    }

    pub fn snapshot(&self) -> PerfBucketSnapshot {
        PerfBucketSnapshot {
            count: self.count.load(Ordering::Relaxed),
            total_us: self.total_us.load(Ordering::Relaxed),
            max_us: self.max_us.load(Ordering::Relaxed),
            last_us: self.last_us.load(Ordering::Relaxed),
            over_100us: self.over_100us.load(Ordering::Relaxed),
            over_500us: self.over_500us.load(Ordering::Relaxed),
            over_1000us: self.over_1000us.load(Ordering::Relaxed),
            over_2000us: self.over_2000us.load(Ordering::Relaxed),
            over_5000us: self.over_5000us.load(Ordering::Relaxed),
        }
    }
}

fn update_max(max: &AtomicU64, val: u64) {
    let mut cur = max.load(Ordering::Relaxed);
    while val > cur {
        match max.compare_exchange_weak(cur, val, Ordering::Relaxed, Ordering::Relaxed) {
            Ok(_) => break,
            Err(v) => cur = v,
        }
    }
}

pub static BUCKET_FRAME_CB_TOTAL: PerfBucket = PerfBucket::new();
pub static BUCKET_ON_FRAME_EVENT: PerfBucket = PerfBucket::new();
pub static BUCKET_TELEMETRY_SHM_WRITE: PerfBucket = PerfBucket::new();
pub static BUCKET_READY_EVENT_SET: PerfBucket = PerfBucket::new();
pub static BUCKET_DISPATCH_NOTIFY: PerfBucket = PerfBucket::new();
pub static BUCKET_ROUTE_BB_FRAME_WRITE: PerfBucket = PerfBucket::new();
pub static BUCKET_INPUT_EVENT_CB: PerfBucket = PerfBucket::new();

pub static SHM_WRITE_COUNT: AtomicU32 = AtomicU32::new(0);
pub static READY_EVENT_SET_COUNT: AtomicU32 = AtomicU32::new(0);
pub static ROUTE_BB_FRAME_WRITE_COUNT: AtomicU32 = AtomicU32::new(0);
pub static NOTIFY_FRAME_TICK_COUNT: AtomicU32 = AtomicU32::new(0);
pub static WORKER_WAKE_SET_EVENT_COUNT: AtomicU32 = AtomicU32::new(0);
pub static WORKER_PENDING_ALREADY_SET_COUNT: AtomicU32 = AtomicU32::new(0);
pub static WORKER_PARKED_SKIP_COUNT: AtomicU32 = AtomicU32::new(0);
pub static WORKER_PARKED_NO_WAKE_COUNT: AtomicU32 = AtomicU32::new(0);
pub static ROUTE_TICK_DISPATCH_COUNT: AtomicU32 = AtomicU32::new(0);
pub static OFF_MODE_NOTIFY_SUPPRESSED_COUNT: AtomicU32 = AtomicU32::new(0);
pub static ROUTE_BB_FRAME_WRITE_SUPPRESSED_DUPLICATE_COUNT: AtomicU32 = AtomicU32::new(0);
pub static INPUT_ENABLED: AtomicU32 = AtomicU32::new(1);
pub static INPUT_EVENT_CB_COUNT: AtomicU32 = AtomicU32::new(0);
pub static DISPATCH_MODE_OFF_COUNT: AtomicU32 = AtomicU32::new(0);
pub static DISPATCH_WAKE_EXCEPTION_COUNT: AtomicU32 = AtomicU32::new(0);
pub static DISPATCH_OFF_GATE_TAKEN_COUNT: AtomicU32 = AtomicU32::new(0);
pub static DISPATCH_NOTIFY_CALLED_COUNT: AtomicU32 = AtomicU32::new(0);
pub static DISPATCH_FORCE_WAKE_COUNT: AtomicU32 = AtomicU32::new(0);
pub static DISPATCH_ENABLE_GENERATION_CHANGED_COUNT: AtomicU32 = AtomicU32::new(0);
pub static DISPATCH_RESET_EXCEPTION_COUNT: AtomicU32 = AtomicU32::new(0);
pub static MINIMAL_TELEMETRY_ENABLED: AtomicU32 = AtomicU32::new(0);
pub static MINIMAL_FRAME_CB_COUNT: AtomicU32 = AtomicU32::new(0);
pub static RESOLVER_ATTEMPTS: AtomicU32 = AtomicU32::new(0);
pub static PERF_SEQUENCE: AtomicU32 = AtomicU32::new(0);

// --- v5: diag-level bisect counters / config ------------------------------
pub static DIAG_LEVEL_CODE: AtomicU32 = AtomicU32::new(crate::diag_level::DiagLevel::Normal as u32);
pub static DIAG_CALLBACKS_REGISTERED: AtomicU32 = AtomicU32::new(0);
pub static DIAG_INPUT_REGISTERED: AtomicU32 = AtomicU32::new(0);
pub static DIAG_TELEMETRY_SHM_ENABLED: AtomicU32 = AtomicU32::new(0);
pub static DIAG_READY_EVENT_ENABLED: AtomicU32 = AtomicU32::new(0);
pub static DIAG_ROUTE_BB_ENABLED: AtomicU32 = AtomicU32::new(0);
pub static DIAG_DISPATCH_ENABLED: AtomicU32 = AtomicU32::new(0);
pub static DIAG_WORKER_ENABLED: AtomicU32 = AtomicU32::new(0);
pub static CALLBACK_NOOP_COUNT: AtomicU32 = AtomicU32::new(0);
pub static CALLBACK_COUNTER_COUNT: AtomicU32 = AtomicU32::new(0);
pub static CALLBACK_QPC_COUNT: AtomicU32 = AtomicU32::new(0);
pub static PERF_SNAPSHOT_COUNT: AtomicU32 = AtomicU32::new(0);
pub static TELEMETRY_SHM_COMPONENT_COUNT: AtomicU32 = AtomicU32::new(0);
pub static READY_EVENT_COMPONENT_COUNT: AtomicU32 = AtomicU32::new(0);
pub static ROUTE_BB_COMPONENT_COUNT: AtomicU32 = AtomicU32::new(0);

/// Diag-level configuration mirrored into the perf snapshot once at init.
#[derive(Debug, Clone, Copy)]
pub struct DiagConfig {
    pub level_code: u32,
    pub callbacks_registered: bool,
    pub input_registered: bool,
    pub telemetry_shm_enabled: bool,
    pub ready_event_enabled: bool,
    pub route_bb_frame_write_enabled: bool,
    pub dispatch_enabled: bool,
    pub worker_enabled: bool,
}

/// Store the diag-level config (called once at DLL init).
pub fn set_diag_config(cfg: DiagConfig) {
    DIAG_LEVEL_CODE.store(cfg.level_code, Ordering::Release);
    DIAG_CALLBACKS_REGISTERED.store(u32::from(cfg.callbacks_registered), Ordering::Release);
    DIAG_INPUT_REGISTERED.store(u32::from(cfg.input_registered), Ordering::Release);
    DIAG_TELEMETRY_SHM_ENABLED.store(u32::from(cfg.telemetry_shm_enabled), Ordering::Release);
    DIAG_READY_EVENT_ENABLED.store(u32::from(cfg.ready_event_enabled), Ordering::Release);
    DIAG_ROUTE_BB_ENABLED.store(u32::from(cfg.route_bb_frame_write_enabled), Ordering::Release);
    DIAG_DISPATCH_ENABLED.store(u32::from(cfg.dispatch_enabled), Ordering::Release);
    DIAG_WORKER_ENABLED.store(u32::from(cfg.worker_enabled), Ordering::Release);
}

pub fn note_callback_noop() {
    CALLBACK_NOOP_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_callback_counter() {
    CALLBACK_COUNTER_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_callback_qpc() {
    CALLBACK_QPC_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_perf_snapshot_component() {
    PERF_SNAPSHOT_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_telemetry_shm_component() {
    TELEMETRY_SHM_COMPONENT_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_ready_event_component() {
    READY_EVENT_COMPONENT_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_route_bb_component() {
    ROUTE_BB_COMPONENT_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn bucket(id: PerfBucketId) -> &'static PerfBucket {
    match id {
        PerfBucketId::FrameCbTotal => &BUCKET_FRAME_CB_TOTAL,
        PerfBucketId::OnFrameEvent => &BUCKET_ON_FRAME_EVENT,
        PerfBucketId::TelemetryShmWrite => &BUCKET_TELEMETRY_SHM_WRITE,
        PerfBucketId::ReadyEventSet => &BUCKET_READY_EVENT_SET,
        PerfBucketId::DispatchNotifyFrameTick => &BUCKET_DISPATCH_NOTIFY,
        PerfBucketId::RouteBbFrameWrite => &BUCKET_ROUTE_BB_FRAME_WRITE,
        PerfBucketId::InputEventCb => &BUCKET_INPUT_EVENT_CB,
    }
}

pub fn qpc_now_us() -> u64 {
    #[cfg(windows)]
    {
        type LARGE_INTEGER = i64;
        extern "system" {
            fn QueryPerformanceCounter(lp: *mut LARGE_INTEGER) -> i32;
            fn QueryPerformanceFrequency(lp: *mut LARGE_INTEGER) -> i32;
        }
        unsafe {
            static mut FREQ: u64 = 0;
            if FREQ == 0 {
                let mut f: LARGE_INTEGER = 0;
                if QueryPerformanceFrequency(&mut f) == 0 || f <= 0 {
                    return 0;
                }
                FREQ = f as u64;
            }
            let mut c: LARGE_INTEGER = 0;
            if QueryPerformanceCounter(&mut c) == 0 {
                return 0;
            }
            return ((c as u64).saturating_mul(1_000_000) / FREQ) as u64;
        }
    }
    #[cfg(not(windows))]
    {
        0
    }
}

pub fn record_since(id: PerfBucketId, start_us: u64) {
    let now = qpc_now_us();
    bucket(id).record(now.saturating_sub(start_us));
}

pub struct PerfGuard {
    bucket: PerfBucketId,
    start_us: u64,
}

impl PerfGuard {
    pub fn begin(bucket: PerfBucketId) -> Self {
        Self {
            bucket,
            start_us: qpc_now_us(),
        }
    }
}

impl Drop for PerfGuard {
    fn drop(&mut self) {
        record_since(self.bucket, self.start_us);
    }
}

pub fn note_shm_write() {
    SHM_WRITE_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_ready_event_set() {
    READY_EVENT_SET_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_route_bb_frame_write() {
    ROUTE_BB_FRAME_WRITE_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_route_bb_frame_write_suppressed_duplicate() {
    ROUTE_BB_FRAME_WRITE_SUPPRESSED_DUPLICATE_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_route_tick_dispatch() {
    ROUTE_TICK_DISPATCH_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_off_mode_notify_suppressed() {
    OFF_MODE_NOTIFY_SUPPRESSED_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_input_event_cb() {
    INPUT_EVENT_CB_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn set_input_enabled(enabled: bool) {
    INPUT_ENABLED.store(u32::from(enabled), Ordering::Release);
}

pub fn set_minimal_telemetry_enabled(enabled: bool) {
    MINIMAL_TELEMETRY_ENABLED.store(u32::from(enabled), Ordering::Release);
}

pub fn note_minimal_frame_cb() {
    MINIMAL_FRAME_CB_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_notify_frame_tick() {
    NOTIFY_FRAME_TICK_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_dispatch_mode_off() {
    DISPATCH_MODE_OFF_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_dispatch_wake_exception() {
    DISPATCH_WAKE_EXCEPTION_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_dispatch_off_gate_taken() {
    DISPATCH_OFF_GATE_TAKEN_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_dispatch_notify_called() {
    DISPATCH_NOTIFY_CALLED_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_dispatch_force_wake() {
    DISPATCH_FORCE_WAKE_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_dispatch_enable_generation_changed() {
    DISPATCH_ENABLE_GENERATION_CHANGED_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_dispatch_reset_exception() {
    DISPATCH_RESET_EXCEPTION_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_worker_wake_set_event() {
    WORKER_WAKE_SET_EVENT_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_worker_pending_already_set() {
    WORKER_PENDING_ALREADY_SET_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_worker_parked_skip() {
    WORKER_PARKED_SKIP_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_worker_parked_no_wake() {
    WORKER_PARKED_NO_WAKE_COUNT.fetch_add(1, Ordering::Relaxed);
}

pub fn note_resolver_attempt() {
    RESOLVER_ATTEMPTS.fetch_add(1, Ordering::Relaxed);
}

pub fn build_snapshot(
    frame_cb_count: u32,
    worker_walk_count: u32,
    pattern_scan_count: u32,
) -> DllPerfSnapshot {
    DllPerfSnapshot {
        magic: DLL_PERF_MAGIC,
        version: DLL_PERF_VERSION,
        sequence: PERF_SEQUENCE.fetch_add(1, Ordering::Relaxed),
        buckets: [
            BUCKET_FRAME_CB_TOTAL.snapshot(),
            BUCKET_ON_FRAME_EVENT.snapshot(),
            BUCKET_TELEMETRY_SHM_WRITE.snapshot(),
            BUCKET_READY_EVENT_SET.snapshot(),
            BUCKET_DISPATCH_NOTIFY.snapshot(),
            BUCKET_ROUTE_BB_FRAME_WRITE.snapshot(),
            BUCKET_INPUT_EVENT_CB.snapshot(),
        ],
        frame_cb_count,
        shm_write_count: SHM_WRITE_COUNT.load(Ordering::Relaxed),
        ready_event_set_count: READY_EVENT_SET_COUNT.load(Ordering::Relaxed),
        route_bb_frame_write_count: ROUTE_BB_FRAME_WRITE_COUNT.load(Ordering::Relaxed),
        notify_frame_tick_count: NOTIFY_FRAME_TICK_COUNT.load(Ordering::Relaxed),
        worker_wake_set_event_count: WORKER_WAKE_SET_EVENT_COUNT.load(Ordering::Relaxed),
        worker_pending_already_set_count: WORKER_PENDING_ALREADY_SET_COUNT.load(Ordering::Relaxed),
        worker_walk_count,
        worker_parked_skip_count: WORKER_PARKED_SKIP_COUNT.load(Ordering::Relaxed),
        resolver_attempts: RESOLVER_ATTEMPTS.load(Ordering::Relaxed),
        pattern_scan_count,
        worker_parked_no_wake_count: WORKER_PARKED_NO_WAKE_COUNT.load(Ordering::Relaxed),
        route_tick_dispatch_count: ROUTE_TICK_DISPATCH_COUNT.load(Ordering::Relaxed),
        off_mode_notify_suppressed_count: OFF_MODE_NOTIFY_SUPPRESSED_COUNT.load(Ordering::Relaxed),
        route_bb_frame_write_suppressed_duplicate_count: ROUTE_BB_FRAME_WRITE_SUPPRESSED_DUPLICATE_COUNT
            .load(Ordering::Relaxed),
        input_enabled: INPUT_ENABLED.load(Ordering::Relaxed),
        input_event_cb_count: INPUT_EVENT_CB_COUNT.load(Ordering::Relaxed),
        dispatch_mode_off_count: DISPATCH_MODE_OFF_COUNT.load(Ordering::Relaxed),
        dispatch_wake_exception_count: DISPATCH_WAKE_EXCEPTION_COUNT.load(Ordering::Relaxed),
        dispatch_off_gate_taken_count: DISPATCH_OFF_GATE_TAKEN_COUNT.load(Ordering::Relaxed),
        dispatch_notify_called_count: DISPATCH_NOTIFY_CALLED_COUNT.load(Ordering::Relaxed),
        dispatch_force_wake_count: DISPATCH_FORCE_WAKE_COUNT.load(Ordering::Relaxed),
        dispatch_enable_generation_changed_count: DISPATCH_ENABLE_GENERATION_CHANGED_COUNT
            .load(Ordering::Relaxed),
        dispatch_reset_exception_count: DISPATCH_RESET_EXCEPTION_COUNT.load(Ordering::Relaxed),
        minimal_telemetry_enabled: MINIMAL_TELEMETRY_ENABLED.load(Ordering::Relaxed),
        minimal_frame_cb_count: MINIMAL_FRAME_CB_COUNT.load(Ordering::Relaxed),
        diag_level_code: DIAG_LEVEL_CODE.load(Ordering::Relaxed),
        diag_callbacks_registered: DIAG_CALLBACKS_REGISTERED.load(Ordering::Relaxed),
        diag_input_registered: DIAG_INPUT_REGISTERED.load(Ordering::Relaxed),
        diag_telemetry_shm_enabled: DIAG_TELEMETRY_SHM_ENABLED.load(Ordering::Relaxed),
        diag_ready_event_enabled: DIAG_READY_EVENT_ENABLED.load(Ordering::Relaxed),
        diag_route_bb_frame_write_enabled: DIAG_ROUTE_BB_ENABLED.load(Ordering::Relaxed),
        diag_dispatch_enabled: DIAG_DISPATCH_ENABLED.load(Ordering::Relaxed),
        diag_worker_enabled: DIAG_WORKER_ENABLED.load(Ordering::Relaxed),
        callback_noop_count: CALLBACK_NOOP_COUNT.load(Ordering::Relaxed),
        callback_counter_count: CALLBACK_COUNTER_COUNT.load(Ordering::Relaxed),
        callback_qpc_count: CALLBACK_QPC_COUNT.load(Ordering::Relaxed),
        perf_snapshot_count: PERF_SNAPSHOT_COUNT.load(Ordering::Relaxed),
        telemetry_shm_component_count: TELEMETRY_SHM_COMPONENT_COUNT.load(Ordering::Relaxed),
        ready_event_component_count: READY_EVENT_COMPONENT_COUNT.load(Ordering::Relaxed),
        route_bb_component_count: ROUTE_BB_COMPONENT_COUNT.load(Ordering::Relaxed),
    }
}

#[cfg(test)]
pub fn reset_test_counters() {
    for id in [
        PerfBucketId::FrameCbTotal,
        PerfBucketId::OnFrameEvent,
        PerfBucketId::TelemetryShmWrite,
        PerfBucketId::ReadyEventSet,
        PerfBucketId::DispatchNotifyFrameTick,
        PerfBucketId::RouteBbFrameWrite,
        PerfBucketId::InputEventCb,
    ] {
        let b = bucket(id);
        b.count.store(0, Ordering::Release);
        b.total_us.store(0, Ordering::Release);
        b.max_us.store(0, Ordering::Release);
        b.last_us.store(0, Ordering::Release);
        b.over_100us.store(0, Ordering::Release);
        b.over_500us.store(0, Ordering::Release);
        b.over_1000us.store(0, Ordering::Release);
        b.over_2000us.store(0, Ordering::Release);
        b.over_5000us.store(0, Ordering::Release);
    }
    SHM_WRITE_COUNT.store(0, Ordering::Release);
    READY_EVENT_SET_COUNT.store(0, Ordering::Release);
    ROUTE_BB_FRAME_WRITE_COUNT.store(0, Ordering::Release);
    NOTIFY_FRAME_TICK_COUNT.store(0, Ordering::Release);
    WORKER_WAKE_SET_EVENT_COUNT.store(0, Ordering::Release);
    WORKER_PENDING_ALREADY_SET_COUNT.store(0, Ordering::Release);
    WORKER_PARKED_SKIP_COUNT.store(0, Ordering::Release);
    WORKER_PARKED_NO_WAKE_COUNT.store(0, Ordering::Release);
    ROUTE_TICK_DISPATCH_COUNT.store(0, Ordering::Release);
    OFF_MODE_NOTIFY_SUPPRESSED_COUNT.store(0, Ordering::Release);
    ROUTE_BB_FRAME_WRITE_SUPPRESSED_DUPLICATE_COUNT.store(0, Ordering::Release);
    INPUT_EVENT_CB_COUNT.store(0, Ordering::Release);
    INPUT_ENABLED.store(1, Ordering::Release);
    DISPATCH_MODE_OFF_COUNT.store(0, Ordering::Release);
    DISPATCH_WAKE_EXCEPTION_COUNT.store(0, Ordering::Release);
    DISPATCH_OFF_GATE_TAKEN_COUNT.store(0, Ordering::Release);
    DISPATCH_NOTIFY_CALLED_COUNT.store(0, Ordering::Release);
    DISPATCH_FORCE_WAKE_COUNT.store(0, Ordering::Release);
    DISPATCH_ENABLE_GENERATION_CHANGED_COUNT.store(0, Ordering::Release);
    DISPATCH_RESET_EXCEPTION_COUNT.store(0, Ordering::Release);
    MINIMAL_TELEMETRY_ENABLED.store(0, Ordering::Release);
    MINIMAL_FRAME_CB_COUNT.store(0, Ordering::Release);
    RESOLVER_ATTEMPTS.store(0, Ordering::Release);
    PERF_SEQUENCE.store(0, Ordering::Release);
    DIAG_LEVEL_CODE.store(crate::diag_level::DiagLevel::Normal as u32, Ordering::Release);
    DIAG_CALLBACKS_REGISTERED.store(0, Ordering::Release);
    DIAG_INPUT_REGISTERED.store(0, Ordering::Release);
    DIAG_TELEMETRY_SHM_ENABLED.store(0, Ordering::Release);
    DIAG_READY_EVENT_ENABLED.store(0, Ordering::Release);
    DIAG_ROUTE_BB_ENABLED.store(0, Ordering::Release);
    DIAG_DISPATCH_ENABLED.store(0, Ordering::Release);
    DIAG_WORKER_ENABLED.store(0, Ordering::Release);
    CALLBACK_NOOP_COUNT.store(0, Ordering::Release);
    CALLBACK_COUNTER_COUNT.store(0, Ordering::Release);
    CALLBACK_QPC_COUNT.store(0, Ordering::Release);
    PERF_SNAPSHOT_COUNT.store(0, Ordering::Release);
    TELEMETRY_SHM_COMPONENT_COUNT.store(0, Ordering::Release);
    READY_EVENT_COMPONENT_COUNT.store(0, Ordering::Release);
    ROUTE_BB_COMPONENT_COUNT.store(0, Ordering::Release);
}

#[cfg(windows)]
mod shm {
    use super::*;
    use std::ffi::OsStr;
    use std::os::windows::ffi::OsStrExt;
    use std::ptr;

    type HANDLE = isize;
    type LPVOID = *mut core::ffi::c_void;
    const PAGE_READWRITE: u32 = 0x04;
    const FILE_MAP_ALL_ACCESS: u32 = 0xF001F;
    const INVALID_HANDLE_VALUE: HANDLE = -1;

    extern "system" {
        fn CreateFileMappingW(
            hFile: HANDLE,
            lpAttributes: *const core::ffi::c_void,
            flProtect: u32,
            dwMaximumSizeHigh: u32,
            dwMaximumSizeLow: u32,
            lpName: *const u16,
        ) -> HANDLE;
        fn MapViewOfFile(
            hFileMappingObject: HANDLE,
            dwDesiredAccess: u32,
            dwFileOffsetHigh: u32,
            dwFileOffsetLow: u32,
            dwNumberOfBytesToMap: usize,
        ) -> *mut core::ffi::c_void;
        fn CloseHandle(hObject: HANDLE) -> i32;
        fn UnmapViewOfFile(lpBaseAddress: LPVOID) -> i32;
    }

    static mut PERF_MAP_HANDLE: HANDLE = 0;
    static mut PERF_PTR: *mut DllPerfSnapshot = ptr::null_mut();

    fn wide(s: &str) -> Vec<u16> {
        OsStr::new(s).encode_wide().chain([0]).collect()
    }

    pub fn init_perf_shm() -> Result<(), String> {
        unsafe {
            if !PERF_PTR.is_null() {
                return Ok(());
            }
            let name = wide(DLL_PERF_SHM_NAME);
            let size = std::mem::size_of::<DllPerfSnapshot>() as u32;
            PERF_MAP_HANDLE = CreateFileMappingW(
                INVALID_HANDLE_VALUE,
                ptr::null(),
                PAGE_READWRITE,
                0,
                size,
                name.as_ptr(),
            );
            if PERF_MAP_HANDLE == 0 {
                return Err(format!(
                    "CreateFileMappingW failed for {DLL_PERF_SHM_NAME}"
                ));
            }
            let view = MapViewOfFile(PERF_MAP_HANDLE, FILE_MAP_ALL_ACCESS, 0, 0, size as usize);
            if view.is_null() {
                CloseHandle(PERF_MAP_HANDLE);
                PERF_MAP_HANDLE = 0;
                return Err(format!("MapViewOfFile failed for {DLL_PERF_SHM_NAME}"));
            }
            PERF_PTR = view as *mut DllPerfSnapshot;
            ptr::write(
                PERF_PTR,
                DllPerfSnapshot {
                    magic: DLL_PERF_MAGIC,
                    version: DLL_PERF_VERSION,
                    ..Default::default()
                },
            );
            Ok(())
        }
    }

    pub fn publish_snapshot(snap: DllPerfSnapshot) {
        unsafe {
            if PERF_PTR.is_null() {
                return;
            }
            ptr::write(PERF_PTR, snap);
        }
    }

    pub fn cleanup_perf_shm() {
        unsafe {
            if !PERF_PTR.is_null() {
                let _ = UnmapViewOfFile(PERF_PTR as LPVOID);
                PERF_PTR = ptr::null_mut();
            }
            if PERF_MAP_HANDLE != 0 {
                let _ = CloseHandle(PERF_MAP_HANDLE);
                PERF_MAP_HANDLE = 0;
            }
        }
    }
}

#[cfg(windows)]
pub use shm::{cleanup_perf_shm, init_perf_shm, publish_snapshot};

#[cfg(not(windows))]
pub fn init_perf_shm() -> Result<(), String> {
    Ok(())
}

#[cfg(not(windows))]
pub fn publish_snapshot(_snap: DllPerfSnapshot) {}

#[cfg(not(windows))]
pub fn cleanup_perf_shm() {}

pub fn publish_live_snapshot(frame_cb_count: u32, worker_walk_count: u32, pattern_scan_count: u32) {
    publish_snapshot(build_snapshot(
        frame_cb_count,
        worker_walk_count,
        pattern_scan_count,
    ));
}

#[cfg(test)]
pub fn avg_us(total: u64, count: u64) -> u64 {
    if count == 0 {
        0
    } else {
        total / count
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Writer/reader byte-layout tripwire. Must equal the identical assertion in
    /// `telemetry/src/dll_perf.rs`. If you change `DllPerfSnapshot`, change BOTH
    /// structs, bump `DLL_PERF_VERSION` on BOTH sides, and update both numbers.
    #[test]
    fn snapshot_layout_size_and_align_are_stable() {
        assert_eq!(std::mem::size_of::<DllPerfSnapshot>(), 688);
        assert_eq!(std::mem::align_of::<DllPerfSnapshot>(), 8);
    }

    #[test]
    fn perf_bucket_aggregates_without_panic() {
        // Serialize with every other test that mutates the process-global perf
        // atomics/buckets (diag bisect, minimal, dispatch).
        let _g = crate::test_isolation::TestResolverStateGuard::acquire();
        reset_test_counters();
        let b = bucket(PerfBucketId::FrameCbTotal);
        b.record(50);
        b.record(150);
        b.record(1200);
        let s = b.snapshot();
        assert_eq!(s.count, 3);
        assert_eq!(s.total_us, 1400);
        assert_eq!(s.max_us, 1200);
        assert_eq!(s.last_us, 1200);
        assert_eq!(s.over_100us, 2);
        assert_eq!(s.over_1000us, 1);
        assert_eq!(avg_us(s.total_us, s.count), 466);
    }

    #[test]
    fn build_snapshot_includes_counters() {
        let _g = crate::test_isolation::TestResolverStateGuard::acquire();
        reset_test_counters();
        note_shm_write();
        note_worker_wake_set_event();
        let snap = build_snapshot(42, 3, 1);
        assert_eq!(snap.magic, DLL_PERF_MAGIC);
        assert_eq!(snap.frame_cb_count, 42);
        assert_eq!(snap.shm_write_count, 1);
        assert_eq!(snap.worker_wake_set_event_count, 1);
        assert_eq!(snap.worker_walk_count, 3);
        assert_eq!(snap.pattern_scan_count, 1);
    }

    #[test]
    fn publish_snapshot_noop_without_init() {
        publish_snapshot(build_snapshot(0, 0, 0));
    }

    #[test]
    fn set_diag_config_and_note_counters_land_in_snapshot() {
        // Direct round-trip: set_diag_config -> atomics -> build_snapshot, plus
        // each diag component note_* lands in its own snapshot field.
        let _g = crate::test_isolation::TestResolverStateGuard::acquire();
        reset_test_counters();
        set_diag_config(DiagConfig {
            level_code: crate::diag_level::DiagLevel::TelemetryShm as u32,
            callbacks_registered: true,
            input_registered: false,
            telemetry_shm_enabled: true,
            ready_event_enabled: false,
            route_bb_frame_write_enabled: false,
            dispatch_enabled: false,
            worker_enabled: false,
        });
        note_callback_noop();
        note_callback_counter();
        note_callback_qpc();
        note_perf_snapshot_component();
        note_telemetry_shm_component();
        note_telemetry_shm_component();
        note_ready_event_component();
        note_route_bb_component();
        let snap = build_snapshot(0, 0, 0);
        // Config mirrored 1:1.
        assert_eq!(
            snap.diag_level_code,
            crate::diag_level::DiagLevel::TelemetryShm as u32
        );
        assert_eq!(snap.diag_callbacks_registered, 1);
        assert_eq!(snap.diag_input_registered, 0);
        assert_eq!(snap.diag_telemetry_shm_enabled, 1);
        assert_eq!(snap.diag_ready_event_enabled, 0);
        assert_eq!(snap.diag_route_bb_frame_write_enabled, 0);
        assert_eq!(snap.diag_dispatch_enabled, 0);
        assert_eq!(snap.diag_worker_enabled, 0);
        // Each component counter independent.
        assert_eq!(snap.callback_noop_count, 1);
        assert_eq!(snap.callback_counter_count, 1);
        assert_eq!(snap.callback_qpc_count, 1);
        assert_eq!(snap.perf_snapshot_count, 1);
        assert_eq!(snap.telemetry_shm_component_count, 2);
        assert_eq!(snap.ready_event_component_count, 1);
        assert_eq!(snap.route_bb_component_count, 1);
    }

    #[cfg(windows)]
    #[test]
    fn init_perf_shm_creates_named_mapping() {
        // Guard so a concurrent diag publish never writes to PERF_PTR mid-unmap.
        let _g = crate::test_isolation::TestResolverStateGuard::acquire();
        cleanup_perf_shm();
        let result = init_perf_shm();
        assert!(result.is_ok(), "init failed: {result:?}");
        publish_live_snapshot(1, 2, 3);
        cleanup_perf_shm();
    }
}
