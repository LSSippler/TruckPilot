//! TruckPilot Telemetry Layer
//!
//! Unified reader with priority cascade:
//!
//!   1. Memory Reading (most accurate, Windows-only, requires offsets)
//!   2. Shared Memory   (native plugin in ETS2 process)
//!   3. HTTP            (Funbit ETS2 Telemetry Server fallback)
//!
//! On startup the selected sources are initialised once and re-used. The
//! `TelemetryReader` is the public entry point — call `read()` once per
//! control cycle.

#![warn(missing_docs)]

pub mod http;
pub mod memory;
pub mod shm;

#[cfg(test)]
pub mod mock;

use std::time::{Duration, Instant};

use tracing::{debug, info, warn};
use truckpilot_plugin_api::Telemetry;

// ---------------------------------------------------------------------------
// Sanity-check tuning
// ---------------------------------------------------------------------------

/// Number of consecutive insane frames a single source may emit before
/// the reader marks it suspended. The offending frames are discarded —
/// once the threshold trips, the cooldown lasts [`SANITY_COOLDOWN`]
/// before that source is tried again. Set high enough that an SHM
/// torn-read burst (a few frames per second under load) does not
/// permanently knock out the source.
const SANITY_CONSECUTIVE_BAD: u32 = 30;

/// How long a source stays suspended after exceeding
/// [`SANITY_CONSECUTIVE_BAD`] consecutive insane frames.
const SANITY_COOLDOWN: Duration = Duration::from_secs(2);

/// Per-source slot indexes into [`TelemetryReader::bad_reads`] and
/// [`TelemetryReader::cooldown_until`]. Kept as `const` instead of
/// `enum as usize` so the inline arrays in the cascade stay readable.
const SLOT_MEMORY: usize = 0;
const SLOT_SHM: usize = 1;
const SLOT_HTTP: usize = 2;
const SLOT_COUNT: usize = 3;

/// Identifies which source produced the most recent telemetry frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TelemetrySource {
    /// Read directly from the ETS2 process memory.
    Memory,
    /// Read from the native plugin's shared-memory region.
    SharedMemory,
    /// Polled from the Funbit telemetry HTTP server.
    Http,
    /// No source is currently producing data.
    None,
}

/// Errors that can occur during telemetry initialisation.
#[derive(Debug, thiserror::Error)]
pub enum TelemetryError {
    /// I/O error while opening a source.
    #[error("io: {0}")]
    Io(#[from] std::io::Error),

    /// Failed to attach to the ETS2 process for memory reading.
    #[error("memory: {0}")]
    Memory(String),

    /// Shared memory region could not be opened.
    #[error("shm: {0}")]
    Shm(String),
}

/// Configuration for the telemetry layer.
#[derive(Debug, Clone)]
pub struct TelemetryConfig {
    /// HTTP fallback URL.
    pub http_url: String,
    /// Path to the game-versions TOML for memory offsets.
    pub game_versions_path: Option<std::path::PathBuf>,
    /// Whether to attempt memory-reading at all.
    pub enable_memory: bool,
    /// Whether to attempt shared-memory at all.
    pub enable_shm: bool,
    /// Whether to attempt HTTP at all.
    pub enable_http: bool,
}

impl Default for TelemetryConfig {
    fn default() -> Self {
        Self {
            http_url: "http://localhost:25555/api/ets2/telemetry".to_string(),
            game_versions_path: None,
            enable_memory: true,
            enable_shm: true,
            // HTTP polls the Funbit server which is rarely running.
            // Each failing tick costs up to ~1.3s (connect+read timeouts),
            // which spams logs and stalls the heartbeat. Opt in explicitly
            // when you actually run the Funbit server.
            enable_http: false,
        }
    }
}

/// Persistent telemetry reader. Holds long-lived handles (mmap, HTTP client,
/// process handle) so that each `read()` call is cheap.
pub struct TelemetryReader {
    /// Configuration the reader was constructed with. Kept for diagnostics.
    pub config: TelemetryConfig,
    shm: Option<shm::ShmReader>,
    memory: Option<memory::MemoryReader>,
    http: Option<http::HttpReader>,
    last_source: TelemetrySource,
    /// Count of consecutive insane frames each source has emitted. Reset
    /// on every sane read; trips the cooldown when it reaches
    /// [`SANITY_CONSECUTIVE_BAD`]. Indexed by `SLOT_*`.
    bad_reads: [u32; SLOT_COUNT],
    /// `Instant` after which each source becomes eligible to read
    /// again. `None` means "no cooldown active". Indexed by `SLOT_*`.
    cooldown_until: [Option<Instant>; SLOT_COUNT],
}

impl TelemetryReader {
    /// Construct a new reader and try to initialise each enabled source.
    /// Failed sources are silently skipped; `read()` will fall through to
    /// the next available source.
    pub fn new(config: TelemetryConfig) -> Self {
        let memory = if config.enable_memory {
            match memory::MemoryReader::try_attach(config.game_versions_path.as_deref()) {
                Ok(m) => {
                    info!("Memory-reading source attached");
                    Some(m)
                }
                Err(e) => {
                    debug!("Memory-reading unavailable: {e}");
                    None
                }
            }
        } else {
            None
        };

        let shm = if config.enable_shm {
            match shm::ShmReader::open() {
                Ok(s) => {
                    info!("SHM telemetry source opened");
                    Some(s)
                }
                Err(e) => {
                    debug!("SHM unavailable: {e}");
                    None
                }
            }
        } else {
            None
        };

        let http = if config.enable_http {
            Some(http::HttpReader::new(config.http_url.clone()))
        } else {
            None
        };

        Self {
            config,
            memory,
            shm,
            http,
            last_source: TelemetrySource::None,
            bad_reads: [0; SLOT_COUNT],
            cooldown_until: [None; SLOT_COUNT],
        }
    }

    /// Returns the source that produced the most recent successful read.
    pub fn last_source(&self) -> TelemetrySource {
        self.last_source
    }

    /// Read a telemetry frame from the highest-priority source available.
    /// Returns `None` if every source fails or is suspended this cycle.
    ///
    /// **Sanity & cooldown behaviour.** Each frame is checked against
    /// [`is_sane`] before it is returned. A source that emits
    /// [`SANITY_CONSECUTIVE_BAD`] insane frames in a row is suspended
    /// for [`SANITY_COOLDOWN`] — during the cooldown the cascade
    /// silently falls through to the next source so the autopilot is
    /// not fed garbage data.
    pub fn read(&mut self) -> Option<Telemetry> {
        // Read into locals first so the `&mut self.<reader>` borrow
        // ends before we call `validate_and_account` (which also needs
        // `&mut self`).
        if !self.cooldown_active(SLOT_MEMORY, "Memory") {
            let frame = self.memory.as_mut().and_then(|m| m.read());
            if let Some(t) = frame {
                if let Some(valid) = self.validate_and_account(SLOT_MEMORY, t, "Memory") {
                    self.update_source(TelemetrySource::Memory);
                    return Some(valid);
                }
            }
        }

        if !self.cooldown_active(SLOT_SHM, "SharedMemory") {
            let frame = self.shm.as_mut().and_then(|s| s.read());
            if let Some(t) = frame {
                if let Some(valid) = self.validate_and_account(SLOT_SHM, t, "SharedMemory") {
                    self.update_source(TelemetrySource::SharedMemory);
                    return Some(valid);
                }
            }
        }

        if !self.cooldown_active(SLOT_HTTP, "Http") {
            let frame = self.http.as_mut().and_then(|h| h.read());
            if let Some(t) = frame {
                if let Some(valid) = self.validate_and_account(SLOT_HTTP, t, "Http") {
                    self.update_source(TelemetrySource::Http);
                    return Some(valid);
                }
            }
        }

        self.update_source(TelemetrySource::None);
        None
    }

    fn update_source(&mut self, src: TelemetrySource) {
        if self.last_source != src {
            info!(
                "Telemetry source switched: {:?} -> {:?}",
                self.last_source, src
            );
            self.last_source = src;
        }
    }

    /// Return `true` if `slot` is currently in cooldown. Side effect:
    /// when the cooldown has expired, clears the slot and logs the
    /// retry so operators can see the recovery in the trace.
    fn cooldown_active(&mut self, slot: usize, src_name: &str) -> bool {
        match self.cooldown_until[slot] {
            None => false,
            Some(until) => {
                if Instant::now() < until {
                    true
                } else {
                    self.cooldown_until[slot] = None;
                    info!("Telemetry source {src_name} cooldown expired — retrying");
                    false
                }
            }
        }
    }

    /// Validate `t` and update bookkeeping. Returns `Some(t)` for a
    /// sane frame, `None` for an insane one (in which case the caller
    /// should fall through to the next cascade level). Trips a cooldown
    /// after [`SANITY_CONSECUTIVE_BAD`] consecutive bad frames.
    fn validate_and_account(
        &mut self,
        slot: usize,
        t: Telemetry,
        src_name: &str,
    ) -> Option<Telemetry> {
        if is_sane(&t) {
            self.bad_reads[slot] = 0;
            return Some(t);
        }
        self.bad_reads[slot] = self.bad_reads[slot].saturating_add(1);
        warn!(
            "Telemetry source {src_name} returned insane frame ({}/{}): \
             speed_ms={} engine_rpm={} heading={} pos_y={} pitch={} roll={}",
            self.bad_reads[slot],
            SANITY_CONSECUTIVE_BAD,
            t.speed_ms,
            t.engine_rpm,
            t.heading,
            t.position[1],
            t.pitch,
            t.roll,
        );
        if self.bad_reads[slot] >= SANITY_CONSECUTIVE_BAD {
            self.cooldown_until[slot] = Some(Instant::now() + SANITY_COOLDOWN);
            self.bad_reads[slot] = 0;
            warn!(
                "Telemetry source {src_name} suspended for {}s after {} \
                 consecutive insane frames",
                SANITY_COOLDOWN.as_secs(),
                SANITY_CONSECUTIVE_BAD,
            );
        }
        None
    }
}

/// Validate that a telemetry frame holds plausible values. Used by
/// [`TelemetryReader::read`] to guard the autopilot against garbage
/// data — most often produced by stale memory offsets after an ETS2
/// patch, but also catches NaN/Infinity from buggy sources.
///
/// Bounds:
/// - `speed_ms` ∈ `[-50.0, 100.0]` (250 km/h forward, slow reverse).
/// - `engine_rpm` ∈ `[0.0, 5000.0]`.
/// - `heading` ∈ `[-π - ε, 2π + ε]` (accept either signed or unsigned
///   convention; ε allows for float drift).
/// - `position[1]` (height): `|y| < 100 km` from the world origin.
/// - `pitch`, `roll` ∈ `[-π/2, π/2]`.
/// - All `f64`/`f32` fields finite (no NaN, no infinity).
///
/// Sentinels (`-1.0` for `nav_speed_limit_kmh`, `lead_vehicle_distance_m`,
/// `accel_longitudinal`, `fuel_liters`, `odometer_km`) are accepted
/// unconditionally — they represent "not available", not "broken".
fn is_sane(t: &Telemetry) -> bool {
    use std::f64::consts::PI;

    // 1) NaN / Infinity rejection — catches every f64/f32 field.
    let f64_finite_ok = t.position.iter().all(|c| c.is_finite())
        && t.heading.is_finite()
        && t.pitch.is_finite()
        && t.roll.is_finite()
        && t.speed_ms.is_finite()
        && t.engine_rpm.is_finite()
        && t.cruise_control_kmh.is_finite()
        && t.nav_speed_limit_kmh.is_finite()
        && t.fuel_liters.is_finite()
        && t.odometer_km.is_finite();
    let f32_finite_ok = t.lead_vehicle_distance_m.is_finite() && t.accel_longitudinal.is_finite();
    if !f64_finite_ok || !f32_finite_ok {
        return false;
    }

    // 2) Range checks for autopilot-critical fields.
    if !(-50.0..=100.0).contains(&t.speed_ms) {
        return false;
    }
    if !(0.0..=5000.0).contains(&t.engine_rpm) {
        return false;
    }
    // Accept both [-π, π] and [0, 2π] heading conventions, with a
    // small epsilon to absorb float-drift at the boundaries.
    let eps = 1e-3;
    if !(-PI - eps..=2.0 * PI + eps).contains(&t.heading) {
        return false;
    }
    if t.position[1].abs() >= 100_000.0 {
        return false;
    }
    let half_pi = PI / 2.0;
    if !(-half_pi..=half_pi).contains(&t.pitch) {
        return false;
    }
    if !(-half_pi..=half_pi).contains(&t.roll) {
        return false;
    }

    true
}

/// Convenience one-shot reader. Creates a process-wide reader on first
/// call, reads once per call thereafter, never drops the reader.
///
/// **Blocking.** Reads memory, shared memory, and (as fallback) HTTP
/// synchronously. Do not call from inside an async task on a tokio
/// runtime — use [`read_telemetry_async`] instead, which dispatches the
/// work to `spawn_blocking`. This sync entry point is kept for
/// non-async callers (the `autopilot` subcommand, CLI probes, tests).
pub fn read_telemetry() -> Option<Telemetry> {
    static INIT: std::sync::OnceLock<std::sync::Mutex<TelemetryReader>> =
        std::sync::OnceLock::new();

    let reader = INIT
        .get_or_init(|| std::sync::Mutex::new(TelemetryReader::new(TelemetryConfig::default())));
    reader.lock().ok()?.read()
}

/// Async wrapper around [`read_telemetry`] that runs the blocking read
/// on tokio's blocking thread pool.
///
/// Why: the underlying `TelemetryReader::read` performs synchronous
/// `ReadProcessMemory` calls, memory-mapped reads, and (in the HTTP
/// fallback) a blocking `ureq` HTTP request. Calling that directly
/// from the 50 Hz core control loop would stall the tokio executor
/// for several milliseconds per tick, starving every other task on
/// the same worker (IPC server, plugin reload watcher, watchdog,
/// etc.). `spawn_blocking` moves the work onto a dedicated blocking
/// thread so the async runtime stays responsive.
///
/// Returns `None` if every source failed this cycle, or if the blocking
/// task panicked. Errors from the join are logged but not surfaced —
/// the next tick will retry from a fresh blocking task.
pub async fn read_telemetry_async() -> Option<Telemetry> {
    match tokio::task::spawn_blocking(read_telemetry).await {
        Ok(t) => t,
        Err(join_err) => {
            tracing::warn!("telemetry read task panicked or was cancelled: {join_err}");
            None
        }
    }
}

#[cfg(test)]
mod sanity_tests {
    use super::*;
    use crate::mock::fixture_telemetry;

    #[test]
    fn fixture_is_sane() {
        assert!(is_sane(&fixture_telemetry()));
    }

    #[test]
    fn rejects_speed_above_max() {
        let mut t = fixture_telemetry();
        t.speed_ms = 150.0;
        assert!(!is_sane(&t));
    }

    #[test]
    fn rejects_speed_below_min() {
        let mut t = fixture_telemetry();
        t.speed_ms = -75.0;
        assert!(!is_sane(&t));
    }

    #[test]
    fn rejects_negative_engine_rpm() {
        let mut t = fixture_telemetry();
        t.engine_rpm = -1.0;
        assert!(!is_sane(&t));
    }

    #[test]
    fn rejects_engine_rpm_above_max() {
        let mut t = fixture_telemetry();
        t.engine_rpm = 9000.0;
        assert!(!is_sane(&t));
    }

    #[test]
    fn rejects_heading_outside_unsigned_range() {
        let mut t = fixture_telemetry();
        t.heading = 10.0; // > 2π + eps
        assert!(!is_sane(&t));
    }

    #[test]
    fn accepts_signed_and_unsigned_heading() {
        let mut t = fixture_telemetry();
        t.heading = -2.5; // signed convention
        assert!(is_sane(&t));
        t.heading = 4.5; // unsigned convention (≈ -1.78 rad)
        assert!(is_sane(&t));
    }

    #[test]
    fn rejects_extreme_height() {
        let mut t = fixture_telemetry();
        t.position[1] = 200_000.0; // 200 km — beyond any plausible map
        assert!(!is_sane(&t));
    }

    #[test]
    fn rejects_pitch_overflow() {
        let mut t = fixture_telemetry();
        t.pitch = 2.0; // > π/2
        assert!(!is_sane(&t));
    }

    #[test]
    fn rejects_nan() {
        let mut t = fixture_telemetry();
        t.speed_ms = f64::NAN;
        assert!(!is_sane(&t));
    }

    #[test]
    fn rejects_infinity() {
        let mut t = fixture_telemetry();
        t.engine_rpm = f64::INFINITY;
        assert!(!is_sane(&t));
    }

    #[test]
    fn accepts_sentinel_values() {
        let mut t = fixture_telemetry();
        t.nav_speed_limit_kmh = -1.0;
        t.lead_vehicle_distance_m = -1.0;
        t.accel_longitudinal = -1.0;
        t.fuel_liters = -1.0;
        t.odometer_km = -1.0;
        assert!(is_sane(&t));
    }
}

#[cfg(test)]
mod cooldown_tests {
    use super::*;

    fn empty_reader() -> TelemetryReader {
        // Disable every concrete source so the reader holds None for
        // each. The cooldown bookkeeping does not need a live source —
        // we drive it through `validate_and_account` directly.
        let cfg = TelemetryConfig {
            http_url: String::new(),
            game_versions_path: None,
            enable_memory: false,
            enable_shm: false,
            enable_http: false,
        };
        TelemetryReader::new(cfg)
    }

    fn insane_frame() -> Telemetry {
        let mut t = crate::mock::fixture_telemetry();
        t.speed_ms = 999.0; // out of range → insane
        t
    }

    fn sane_frame() -> Telemetry {
        crate::mock::fixture_telemetry()
    }

    #[test]
    fn single_insane_does_not_trip_cooldown() {
        let mut r = empty_reader();
        let result = r.validate_and_account(SLOT_SHM, insane_frame(), "SharedMemory");
        assert!(result.is_none());
        assert_eq!(r.bad_reads[SLOT_SHM], 1);
        assert!(r.cooldown_until[SLOT_SHM].is_none());
    }

    #[test]
    fn three_consecutive_insane_trips_cooldown() {
        let mut r = empty_reader();
        for _ in 0..SANITY_CONSECUTIVE_BAD {
            assert!(r
                .validate_and_account(SLOT_HTTP, insane_frame(), "Http")
                .is_none());
        }
        assert!(r.cooldown_until[SLOT_HTTP].is_some());
        // Counter reset once cooldown trips.
        assert_eq!(r.bad_reads[SLOT_HTTP], 0);
    }

    #[test]
    fn sane_frame_resets_bad_counter() {
        let mut r = empty_reader();
        // Two bad frames ...
        r.validate_and_account(SLOT_MEMORY, insane_frame(), "Memory");
        r.validate_and_account(SLOT_MEMORY, insane_frame(), "Memory");
        assert_eq!(r.bad_reads[SLOT_MEMORY], 2);
        // ... then one good one resets the counter.
        let ok = r.validate_and_account(SLOT_MEMORY, sane_frame(), "Memory");
        assert!(ok.is_some());
        assert_eq!(r.bad_reads[SLOT_MEMORY], 0);
        assert!(r.cooldown_until[SLOT_MEMORY].is_none());
    }

    #[test]
    fn cooldown_active_returns_true_during_cooldown() {
        let mut r = empty_reader();
        r.cooldown_until[SLOT_SHM] = Some(Instant::now() + Duration::from_secs(60));
        assert!(r.cooldown_active(SLOT_SHM, "SharedMemory"));
        // And the cooldown is preserved (not cleared while active).
        assert!(r.cooldown_until[SLOT_SHM].is_some());
    }

    #[test]
    fn cooldown_active_clears_expired_cooldown() {
        let mut r = empty_reader();
        // Cooldown that expired 1 second ago.
        r.cooldown_until[SLOT_SHM] = Some(Instant::now() - Duration::from_secs(1));
        assert!(!r.cooldown_active(SLOT_SHM, "SharedMemory"));
        assert!(r.cooldown_until[SLOT_SHM].is_none());
    }
}
