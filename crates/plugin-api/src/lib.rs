//! TruckPilot Plugin API
//!
//! Defines the stable contract all plugins must implement.
//!
//! ## ABI Stability & Hot-Reload Warning
//!
//! **Important**: This crate does **not** use `abi_stable` (no `#[sabi_trait]`, no `RString`/`RVec`).
//! The `Plugin` trait is a normal Rust trait.
//!
//! As a consequence, **Hot-Reload is currently disabled** for safety reasons.
//! Recompiling a plugin .dll and loading it at runtime can cause memory corruption
//! because Rust's ABI is not stable across compiler versions or even minor code changes.
//!
//! The file watcher in `truckpilot-core` is kept for development convenience but
//! will not perform actual reloads until ABI stability is restored (either by
//! re-introducing `abi_stable` or by using a stable C ABI boundary).

pub mod pid;

use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock, RwLock};

// ---------------------------------------------------------------------------
// LogLevel + LogSinkWrapper
// ---------------------------------------------------------------------------

/// Log severity level, mirroring tracing's five levels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogLevel {
    Trace,
    Debug,
    Info,
    Warn,
    Error,
}

/// Type alias for the log-sink closure stored inside [`LogSinkWrapper`].
pub type LogSinkFn = dyn Fn(LogLevel, &str, &str) + Send + Sync;

/// Newtype wrapping a log-sink callback so [`PluginContext`] can derive
/// `Debug` and `Clone`.
///
/// The host constructs one instance per plugin-load and closes over a
/// `tracing::event!` dispatch. Plugin code calls [`PluginContext::log`]
/// or the `ctx_info!/ctx_warn!/…` macros, which invoke this closure
/// inside the *host* subscriber context — bridging the cross-DLL tracing
/// gap where each cdylib would otherwise have its own unregistered global.
#[derive(Clone)]
pub struct LogSinkWrapper(pub Arc<LogSinkFn>);

impl std::fmt::Debug for LogSinkWrapper {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("LogSinkWrapper(<fn>)")
    }
}

use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Telemetry
// ---------------------------------------------------------------------------

/// Telemetry snapshot passed to plugins every tick.
/// Sentinel `-1.0` means "not available" for optional float fields.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Telemetry {
    /// Truck position (x, y, z) in meters.
    pub position: [f64; 3],
    /// Heading in radians (0 = North, positive = clockwise).
    pub heading: f64,
    /// Pitch in radians.
    pub pitch: f64,
    /// Roll in radians.
    pub roll: f64,
    /// Forward speed in m/s.
    pub speed_ms: f64,
    /// Engine RPM.
    pub engine_rpm: f64,
    /// Driver-set cruise control target speed in km/h (0 = off).
    pub cruise_control_kmh: f64,
    /// Navigation speed limit in km/h. `-1.0` = not available.
    pub nav_speed_limit_kmh: f64,
    /// Distance to lead vehicle in meters. `-1.0` = not available.
    pub lead_vehicle_distance_m: f32,
    /// Longitudinal acceleration in m/s². `-1.0` = not available.
    pub accel_longitudinal: f32,
    /// Current fuel level in litres. `-1.0` = not available
    /// (e.g. HTTP fallback source — Funbit JSON does not expose fuel).
    pub fuel_liters: f64,
    /// Total odometer reading in km. `-1.0` = not available
    /// (e.g. HTTP fallback source).
    pub odometer_km: f64,
}

// ---------------------------------------------------------------------------
// ControlOutput
// ---------------------------------------------------------------------------

/// Control output written by plugins each tick.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ControlOutput {
    /// Steering angle in [-1.0, 1.0]. Positive = right.
    pub steering: f64,
    /// Throttle in [0.0, 1.0].
    pub throttle: f64,
    /// Brake in [0.0, 1.0].
    pub brake: f64,
}

impl Default for ControlOutput {
    fn default() -> Self {
        Self {
            steering: 0.0,
            throttle: 0.0,
            brake: 0.0,
        }
    }
}

// ---------------------------------------------------------------------------
// ControlRequest
// ---------------------------------------------------------------------------

/// A plugin's *opinion* on the next control output. Returned by
/// [`Plugin::tick_request`].
///
/// Each axis is `Option<f64>` — `None` means "I have no opinion on this
/// axis, leave it to the others." The host arbitrator combines the
/// requests from every plugin (see `crates/core/src/plugin_manager.rs`)
/// using safety-first rules:
///
/// - **brake**: the maximum of all opinions wins. Any plugin can demand
///   more braking; nobody can override another plugin's brake.
/// - **throttle**: if *any* plugin requests `brake > 0`, throttle is
///   minimised across all opinions (so a brake request always cuts
///   throttle). Otherwise the highest-`priority` opinion wins.
/// - **steering**: the highest-`priority` opinion wins. Ties resolve in
///   plugin-load order (later plugin wins on equality).
///
/// `priority` is a free-form `i32`. Convention:
///   - `0`   ordinary autopilot (lane-keeper, speed-controller, …)
///   - `100` user/operator override
///   - `200` safety / emergency
#[derive(Debug, Clone, Copy, Default)]
pub struct ControlRequest {
    /// Requested steering in `[-1.0, 1.0]`. `None` = no opinion.
    pub steering: Option<f64>,
    /// Requested throttle in `[0.0, 1.0]`. `None` = no opinion.
    pub throttle: Option<f64>,
    /// Requested brake in `[0.0, 1.0]`. `None` = no opinion.
    pub brake: Option<f64>,
    /// Priority used by the arbitrator. Higher wins. Default `0`.
    pub priority: i32,
}

// ---------------------------------------------------------------------------
// SharedBlackboard
// ---------------------------------------------------------------------------

/// Thread-safe key-value store for plugin-to-plugin communication.
///
/// ## Standard keys
///
/// | Key                              | Writer          | Reader(s)                | Format        |
/// |----------------------------------|-----------------|--------------------------|---------------|
/// | `acc.speed_cap_kmh`              | acc             | speed-controller         | f64 as string |
/// | `router.waypoints`               | router          | lane-keeper              | JSON `[[x,z]]`|
/// | `router.active`                  | router          | lane-keeper              | "true"/"false"|
/// | `sign.speed_limit_kmh`           | sign-reader     | speed-controller         | f64 as string |
/// | `vjoy.device_id`                 | core/config     | vjoy-output              | u32 as string |
/// | `telemetry.available`            | core            | any                      | "true"/"false"|
/// | `telemetry.position_x/y/z`       | core            | any                      | f64 as string |
/// | `telemetry.heading`              | core            | any                      | f64 as string |
/// | `telemetry.pitch`                | core            | any                      | f64 as string |
/// | `telemetry.roll`                 | core            | any                      | f64 as string |
/// | `telemetry.speed_ms`             | core            | any                      | f64 as string |
/// | `telemetry.engine_rpm`           | core            | any                      | f64 as string |
/// | `telemetry.cruise_control_kmh`   | core            | any                      | f64 as string |
/// | `telemetry.nav_speed_limit_kmh`  | core            | any                      | f64 as string |
/// | `telemetry.lead_vehicle_distance_m` | core         | acc                      | f64 as string |
/// | `telemetry.accel_longitudinal`   | core            | acc                      | f64 as string |
/// | `telemetry.fuel_liters`          | core            | fuel-stops, stats-logger | f64 as string |
/// | `telemetry.odometer_km`          | core            | stats-logger             | f64 as string |
///
/// `telemetry.*` keys with sentinel `-1.0` are written by removing the
/// key (so `get_f64` returns `None`); plugins must treat absence as
/// "not available" rather than zero.
#[derive(Debug, Clone, Default)]
pub struct SharedBlackboard(Arc<Mutex<HashMap<String, String>>>);

impl SharedBlackboard {
    /// Create a new empty blackboard.
    pub fn new() -> Self {
        Self::default()
    }

    /// Write a value. Overwrites any existing entry.
    pub fn set(&self, key: impl Into<String>, value: impl Into<String>) {
        if let Ok(mut map) = self.0.lock() {
            map.insert(key.into(), value.into());
        }
    }

    /// Read a value. Returns `None` if the key is absent.
    pub fn get(&self, key: &str) -> Option<String> {
        self.0.lock().ok()?.get(key).cloned()
    }

    /// Read and parse a value as `f64`.
    pub fn get_f64(&self, key: &str) -> Option<f64> {
        self.get(key)?.parse().ok()
    }

    /// Remove a key.
    pub fn remove(&self, key: &str) {
        if let Ok(mut map) = self.0.lock() {
            map.remove(key);
        }
    }

    /// Snapshot of all key-value pairs. Takes a single lock; result is
    /// consistent but immediately stale.
    pub fn snapshot(&self) -> HashMap<String, String> {
        self.0.lock().map(|m| m.clone()).unwrap_or_default()
    }

    /// All currently set keys, optionally filtered by prefix.
    pub fn keys(&self, prefix: Option<&str>) -> Vec<String> {
        let map = self.0.lock();
        let Ok(m) = map else {
            return Vec::new();
        };
        match prefix {
            Some(p) => m.keys().filter(|k| k.starts_with(p)).cloned().collect(),
            None => m.keys().cloned().collect(),
        }
    }
}

// ---------------------------------------------------------------------------
// SharedFrame / SharedFrameStore (Phase 6.5c.2)
// ---------------------------------------------------------------------------

/// A single camera frame shared between plugins.
///
/// `SharedFrame` carries the raw JPEG bytes a producer plugin (typically
/// `vision-frame-source`) wrote to shared memory, plus enough metadata
/// for consumers (`sign-vision`, future detectors) to identify and time
/// the frame. The JPEG payload lives behind an [`Arc`] so any number of
/// readers can share it without copying.
///
/// The optional `decoded_rgb8` slot lets the first consumer that needs
/// raw pixels pay the decode cost once and memoise the result via
/// [`OnceLock`]; subsequent readers get the same `Arc<Vec<u8>>` for free.
/// See [`SharedFrame::get_or_init_rgb8`].
///
/// ## Example
/// ```no_run
/// use std::sync::Arc;
/// use truckpilot_plugin_api::SharedFrame;
///
/// let jpeg: Arc<Vec<u8>> = Arc::new(vec![0xFF, 0xD8, /* ... */]);
/// let frame = SharedFrame::new(42, 1_700_000_000_000_000, 1920, 1080, jpeg);
/// assert_eq!(frame.id, 42);
/// assert!(frame.decoded_rgb8().is_none());
/// ```
#[derive(Debug)]
pub struct SharedFrame {
    /// Logical frame id (monotonically increasing per camera).
    pub id: u64,
    /// Producer-side monotonic timestamp in microseconds.
    pub timestamp_us: u64,
    /// Frame width in pixels.
    pub width: u32,
    /// Frame height in pixels.
    pub height: u32,
    /// JPEG-encoded payload. Shared by `Arc`; never mutated after publish.
    pub jpeg: Arc<Vec<u8>>,
    /// Lazily-populated decoded RGB8 buffer (3 bytes/pixel, row-major,
    /// no padding). Filled on first consumer demand via
    /// [`SharedFrame::get_or_init_rgb8`].
    decoded_rgb8: OnceLock<Arc<Vec<u8>>>,
}

impl SharedFrame {
    /// Construct a new frame. `decoded_rgb8` starts empty.
    pub fn new(id: u64, timestamp_us: u64, width: u32, height: u32, jpeg: Arc<Vec<u8>>) -> Self {
        Self {
            id,
            timestamp_us,
            width,
            height,
            jpeg,
            decoded_rgb8: OnceLock::new(),
        }
    }

    /// Return the decoded RGB8 buffer if it has been initialised, else `None`.
    pub fn decoded_rgb8(&self) -> Option<&Arc<Vec<u8>>> {
        self.decoded_rgb8.get()
    }

    /// Get the decoded RGB8 buffer, computing it on first call.
    ///
    /// The closure runs **at most once per `SharedFrame`** (subsequent
    /// callers, even from other threads, observe the value stored by the
    /// winner of the initialisation race). Returns the same `Arc<Vec<u8>>`
    /// every time, so consumers can clone the `Arc` cheaply.
    ///
    /// ## Example
    /// ```no_run
    /// # use std::sync::Arc;
    /// # use truckpilot_plugin_api::SharedFrame;
    /// # let frame = SharedFrame::new(0, 0, 4, 4, Arc::new(vec![]));
    /// let rgb = frame.get_or_init_rgb8(|| Arc::new(vec![0u8; 4 * 4 * 3]));
    /// assert_eq!(rgb.len(), 48);
    /// ```
    pub fn get_or_init_rgb8<F>(&self, init: F) -> &Arc<Vec<u8>>
    where
        F: FnOnce() -> Arc<Vec<u8>>,
    {
        self.decoded_rgb8.get_or_init(init)
    }

    /// Try to populate the decoded buffer from a fallible decoder. The
    /// closure runs at most once per frame and only on the first call;
    /// on success the buffer is cached, on failure nothing is stored
    /// and a later caller may retry.
    pub fn get_or_try_init_rgb8<E, F>(&self, init: F) -> Result<&Arc<Vec<u8>>, E>
    where
        F: FnOnce() -> Result<Arc<Vec<u8>>, E>,
    {
        if let Some(rgb) = self.decoded_rgb8.get() {
            return Ok(rgb);
        }
        let rgb = init()?;
        // If two threads race, `set` returns Err on the loser — that is
        // fine; we read back whichever value won.
        let _ = self.decoded_rgb8.set(rgb);
        Ok(self.decoded_rgb8.get().expect("OnceLock populated above"))
    }
}

/// Thread-safe keyed store of [`SharedFrame`] values.
///
/// Mirrors the shape of [`SharedBlackboard`] for binary frame payloads
/// that don't fit in the string-only blackboard. One writer per key
/// (typically `vision-frame-source`); many readers.
///
/// The store is cheap to clone (`Arc` internally) and threaded through
/// every [`PluginContext`] when the daemon enables the feature. Plugins
/// that don't need frames can ignore it.
///
/// ## Standard keys
///
/// | Key             | Writer              | Readers                   |
/// |-----------------|---------------------|---------------------------|
/// | `camera.front`  | vision-frame-source | sign-vision, detectors    |
///
/// ## Example
/// ```
/// use std::sync::Arc;
/// use truckpilot_plugin_api::{SharedFrame, SharedFrameStore};
///
/// let store = SharedFrameStore::new();
/// let frame = Arc::new(SharedFrame::new(1, 0, 8, 8, Arc::new(vec![0; 0])));
/// store.set("camera.front", frame);
/// assert!(store.get("camera.front").is_some());
/// ```
#[derive(Debug, Clone, Default)]
pub struct SharedFrameStore {
    inner: Arc<RwLock<HashMap<String, Arc<SharedFrame>>>>,
}

impl SharedFrameStore {
    /// Create an empty store.
    pub fn new() -> Self {
        Self::default()
    }

    /// Publish a frame under `key`. Replaces any previous entry.
    pub fn set(&self, key: impl Into<String>, frame: Arc<SharedFrame>) {
        if let Ok(mut map) = self.inner.write() {
            map.insert(key.into(), frame);
        }
    }

    /// Look up the current frame for `key`. Returns a cheap `Arc` clone.
    pub fn get(&self, key: &str) -> Option<Arc<SharedFrame>> {
        self.inner.read().ok()?.get(key).cloned()
    }

    /// Drop the entry for `key`, returning the removed value if any.
    pub fn remove(&self, key: &str) -> Option<Arc<SharedFrame>> {
        self.inner.write().ok()?.remove(key)
    }

    /// Snapshot of all current keys. Order unspecified.
    pub fn keys(&self) -> Vec<String> {
        self.inner
            .read()
            .map(|m| m.keys().cloned().collect())
            .unwrap_or_default()
    }

    /// Number of keys currently published.
    pub fn len(&self) -> usize {
        self.inner.read().map(|m| m.len()).unwrap_or(0)
    }

    /// `true` iff no keys are published.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

// ---------------------------------------------------------------------------
// TickPhase
// ---------------------------------------------------------------------------

/// Scheduler bucket a plugin runs in. Used by the (Phase 6.2b) daemon
/// scheduler; for now only carried in [`PluginContext`] so plugins can
/// branch on it during early adoption.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TickPhase {
    /// 1 Hz — router, fuel-stops, break-planner.
    PhaseA,
    /// 10 Hz — sign-reader, sign-vision, lane-changer-decision.
    PhaseB,
    /// 50 Hz — lane-keeper, speed-controller, ACC, stats-logger.
    PhaseC,
    /// Post-arbitration — vjoy-output.
    PostPhase,
}

// ---------------------------------------------------------------------------
// PluginContext
// ---------------------------------------------------------------------------

/// Context passed to plugins on load and every tick.
#[derive(Debug, Clone)]
pub struct PluginContext {
    /// Plugin name (for logging).
    pub plugin_name: String,
    /// Shared key-value store for inter-plugin communication.
    pub blackboard: SharedBlackboard,
    /// Seconds since the previous tick. Defaults to `0.02` (50 Hz);
    /// the daemon sets the real value via [`PluginContext::with_dt`].
    /// Plugins should prefer `ctx.dt_s` over hardcoded periods.
    pub dt_s: f64,
    /// Scheduler bucket this tick belongs to. Defaults to
    /// [`TickPhase::PhaseC`] (50 Hz, where most plugins run).
    pub tick_phase: TickPhase,
    /// Monotonic tick counter. Increments once per scheduler step.
    /// Defaults to `0`.
    pub tick_count: u64,
    /// Optional shared frame store for binary camera payloads
    /// (Phase 6.5c.2). `None` for legacy contexts; the daemon attaches
    /// a real store when the vision pipeline is wired in. Plugins access
    /// it via [`PluginContext::frame_store`].
    pub(crate) frame_store: Option<Arc<SharedFrameStore>>,
    /// Optional log sink. When set the host routes plugin log calls through
    /// this closure into the host's tracing subscriber, bridging the
    /// cross-DLL dispatcher gap. `None` in unit tests and legacy contexts.
    pub(crate) log_sink: Option<LogSinkWrapper>,
}

impl PluginContext {
    /// Create a new context. `dt_s` defaults to `0.02` for backward-compat.
    pub fn new(plugin_name: impl Into<String>, blackboard: SharedBlackboard) -> Self {
        Self {
            plugin_name: plugin_name.into(),
            blackboard,
            dt_s: 0.02,
            tick_phase: TickPhase::PhaseC,
            tick_count: 0,
            frame_store: None,
            log_sink: None,
        }
    }

    /// Attach a shared frame store. Builder-style. Used by the daemon
    /// when constructing plugin contexts for the vision pipeline; tests
    /// can use this to inject a controlled store.
    pub fn with_frame_store(mut self, store: Arc<SharedFrameStore>) -> Self {
        self.frame_store = Some(store);
        self
    }

    /// Borrow the shared frame store, if the host wired one in.
    ///
    /// Returns `None` for legacy / minimal contexts (most unit tests,
    /// any daemon configuration without the vision pipeline). Plugins
    /// that need camera frames should treat `None` as "no source
    /// available" and degrade gracefully.
    pub fn frame_store(&self) -> Option<Arc<SharedFrameStore>> {
        self.frame_store.clone()
    }

    /// Attach a log sink. Builder-style. Used by the daemon to bridge
    /// cross-DLL tracing; tests leave this `None`.
    pub fn with_log_sink(mut self, sink: LogSinkWrapper) -> Self {
        self.log_sink = Some(sink);
        self
    }

    /// Emit a log record via the host's tracing subscriber. Falls back to
    /// a no-op when the context has no sink (unit tests, legacy contexts).
    pub fn log(&self, level: LogLevel, target: &str, message: &str) {
        if let Some(sink) = &self.log_sink {
            (sink.0)(level, target, message);
        }
    }

    /// Set the per-tick delta time in seconds. Builder-style. Clamped to >= 0.001.
    pub fn with_dt(mut self, dt_s: f64) -> Self {
        self.dt_s = dt_s.max(0.001);
        self
    }

    /// Set the tick phase. Builder-style.
    pub fn with_phase(mut self, phase: TickPhase) -> Self {
        self.tick_phase = phase;
        self
    }

    /// Set the tick counter. Builder-style.
    pub fn with_tick_count(mut self, count: u64) -> Self {
        self.tick_count = count;
        self
    }

    // --- Convenience: autopilot.state reads ---------------------------------

    /// Current autopilot state from the blackboard, if any.
    pub fn state(&self) -> Option<String> {
        self.blackboard.get("autopilot.state")
    }

    /// `true` iff the autopilot state is `"Active"`.
    pub fn is_active(&self) -> bool {
        self.state().as_deref() == Some("Active")
    }

    /// `true` iff the autopilot is engaged in any way (`Engaging`,
    /// `Active`, or `Paused`).
    pub fn is_engaged(&self) -> bool {
        matches!(
            self.state().as_deref(),
            Some("Engaging") | Some("Active") | Some("Paused")
        )
    }

    /// `true` iff the autopilot state is `"Fault"`.
    pub fn is_fault(&self) -> bool {
        self.state().as_deref() == Some("Fault")
    }

    /// `true` for ticks where the router should re-plan: every 50th
    /// `PhaseA` tick (≈ once per minute at 1 Hz × 50).
    pub fn is_replan_tick(&self) -> bool {
        self.tick_phase == TickPhase::PhaseA && self.tick_count.is_multiple_of(50)
    }

    /// `tracing` target string for this plugin, e.g.
    /// `"truckpilot_plugin_lane_keeper"` for plugin name `"lane-keeper"`.
    pub fn log_target(&self) -> String {
        format!("truckpilot_plugin_{}", self.plugin_name.replace('-', "_"))
    }

    // --- Test helpers -------------------------------------------------------

    /// Default test context with a fresh blackboard.
    pub fn test() -> Self {
        Self::new("test", SharedBlackboard::new())
    }

    /// Test context with `autopilot.state = "Off"`.
    pub fn test_off() -> Self {
        let ctx = Self::test();
        ctx.blackboard.set("autopilot.state", "Off");
        ctx
    }

    /// Test context with `autopilot.state = "Engaging"`.
    pub fn test_engaging() -> Self {
        let ctx = Self::test();
        ctx.blackboard.set("autopilot.state", "Engaging");
        ctx
    }

    /// Test context with `autopilot.state = "Active"`.
    pub fn test_active() -> Self {
        let ctx = Self::test();
        ctx.blackboard.set("autopilot.state", "Active");
        ctx
    }

    /// Test context with `autopilot.state = "Paused"`.
    pub fn test_paused() -> Self {
        let ctx = Self::test();
        ctx.blackboard.set("autopilot.state", "Paused");
        ctx
    }

    /// Test context with `autopilot.state = "Fault"`.
    pub fn test_fault() -> Self {
        let ctx = Self::test();
        ctx.blackboard.set("autopilot.state", "Fault");
        ctx
    }

    /// Test context with a custom `dt_s`.
    pub fn test_with_dt(dt: f64) -> Self {
        Self::test().with_dt(dt)
    }

    /// Test context with a custom [`TickPhase`].
    pub fn test_with_phase(phase: TickPhase) -> Self {
        Self::test().with_phase(phase)
    }

    /// Test context with a custom `tick_count`.
    pub fn test_with_tick_count(count: u64) -> Self {
        Self::test().with_tick_count(count)
    }

    /// Test context positioned on a router-replan tick
    /// (`PhaseA`, `tick_count = 50`).
    pub fn test_replan() -> Self {
        Self::test()
            .with_phase(TickPhase::PhaseA)
            .with_tick_count(50)
    }
}

// ---------------------------------------------------------------------------
// Plugin trait
// ---------------------------------------------------------------------------

/// The plugin trait. All plugins must implement this.
///
/// ## Plugin author rules
/// - No global mutable state without explicit synchronisation.
/// - State that must survive hot-reload must be persisted in `on_unload`
///   and restored in `on_load`.
/// - Do not hold references to `PluginContext` beyond the call that provided it.
pub trait Plugin: Send + Sync {
    /// Human-readable plugin name.
    fn name(&self) -> &str;

    /// Semantic version string (e.g. "1.2.3").
    fn version(&self) -> &str;

    /// JSON Schema for plugin settings (as string).
    fn settings_schema(&self) -> &str;

    /// Called once when the plugin is loaded.
    fn on_load(&mut self, ctx: &PluginContext);

    /// Called when the plugin is unloaded or before reload.
    fn on_unload(&mut self);

    /// Called every control cycle (~20 ms).
    ///
    /// **Use this for side effects only** (blackboard writes, telemetry
    /// recording, internal-state updates). Direct writes to `output` are
    /// the legacy "last-writer-wins" path: they survive arbitration
    /// only if no plugin returned a `ControlRequest` for the same axis.
    ///
    /// New plugins should override [`Plugin::tick_request`] instead.
    /// `telemetry` is `None` if no telemetry source is available.
    fn tick(
        &mut self,
        telemetry: Option<&Telemetry>,
        output: &mut ControlOutput,
        ctx: &PluginContext,
    );

    /// Return this plugin's opinion on the next control output.
    ///
    /// Called every tick alongside [`Plugin::tick`]. The host collects
    /// the `Some(_)` results from all plugins and arbitrates a single
    /// final [`ControlOutput`] (see [`ControlRequest`] for the rules).
    ///
    /// Default: returns `None` — the plugin participates only via the
    /// legacy `tick` path. Override this when you have a strong
    /// preference on `steering`, `throttle`, or `brake` that should be
    /// merged safely with other plugins instead of stomping their
    /// values.
    fn tick_request(
        &mut self,
        _telemetry: Option<&Telemetry>,
        _ctx: &PluginContext,
    ) -> Option<ControlRequest> {
        None
    }

    /// Called when the plugin is hot-reloaded. Default: `on_unload` + `on_load`.
    fn on_reload(&mut self, ctx: &PluginContext) {
        self.on_unload();
        self.on_load(ctx);
    }

    /// Which scheduler bucket this plugin runs in. The daemon uses the
    /// returned phase to decide whether to invoke `tick`/`tick_request`
    /// on a given control cycle (Phase 6.2b scheduler).
    ///
    /// Default: [`TickPhase::PhaseC`] (50 Hz). Override for slower
    /// cadences — see [`TickPhase`] for the rate map.
    fn default_phase(&self) -> TickPhase {
        TickPhase::PhaseC
    }
}

// ---------------------------------------------------------------------------
// ctx_* logging macros
// ---------------------------------------------------------------------------

/// Log at INFO level via the plugin context's log sink.
///
/// Falls back to a no-op when the context has no sink (unit tests).
///
/// # Usage
/// ```ignore
/// ctx_info!(ctx, "message {}", value);
/// ctx_info!(ctx, target: "my_target", "message {}", value);
/// ```
#[macro_export]
macro_rules! ctx_info {
    ($ctx:expr, target: $target:expr, $($arg:tt)+) => {
        $ctx.log($crate::LogLevel::Info, $target, &::std::format!($($arg)+))
    };
    ($ctx:expr, $($arg:tt)+) => {
        $ctx.log($crate::LogLevel::Info, &$ctx.log_target(), &::std::format!($($arg)+))
    };
}

/// Log at WARN level via the plugin context's log sink.
#[macro_export]
macro_rules! ctx_warn {
    ($ctx:expr, target: $target:expr, $($arg:tt)+) => {
        $ctx.log($crate::LogLevel::Warn, $target, &::std::format!($($arg)+))
    };
    ($ctx:expr, $($arg:tt)+) => {
        $ctx.log($crate::LogLevel::Warn, &$ctx.log_target(), &::std::format!($($arg)+))
    };
}

/// Log at ERROR level via the plugin context's log sink.
#[macro_export]
macro_rules! ctx_error {
    ($ctx:expr, target: $target:expr, $($arg:tt)+) => {
        $ctx.log($crate::LogLevel::Error, $target, &::std::format!($($arg)+))
    };
    ($ctx:expr, $($arg:tt)+) => {
        $ctx.log($crate::LogLevel::Error, &$ctx.log_target(), &::std::format!($($arg)+))
    };
}

/// Log at DEBUG level via the plugin context's log sink.
#[macro_export]
macro_rules! ctx_debug {
    ($ctx:expr, target: $target:expr, $($arg:tt)+) => {
        $ctx.log($crate::LogLevel::Debug, $target, &::std::format!($($arg)+))
    };
    ($ctx:expr, $($arg:tt)+) => {
        $ctx.log($crate::LogLevel::Debug, &$ctx.log_target(), &::std::format!($($arg)+))
    };
}

/// Log at TRACE level via the plugin context's log sink.
#[macro_export]
macro_rules! ctx_trace {
    ($ctx:expr, target: $target:expr, $($arg:tt)+) => {
        $ctx.log($crate::LogLevel::Trace, $target, &::std::format!($($arg)+))
    };
    ($ctx:expr, $($arg:tt)+) => {
        $ctx.log($crate::LogLevel::Trace, &$ctx.log_target(), &::std::format!($($arg)+))
    };
}

// ---------------------------------------------------------------------------
// FFI export macro
// ---------------------------------------------------------------------------

/// Export a plugin type across the cdylib boundary as a pair of FFI
/// symbols.
///
/// Generates:
/// - `create_plugin() -> *mut dyn Plugin` — allocates a fresh instance
///   of `$plugin_ty` (using its `Default` impl) inside the plugin's own
///   allocator and returns the leaked raw pointer.
/// - `destroy_plugin(ptr: *mut dyn Plugin)` — reconstructs the `Box`
///   from the raw pointer **inside the plugin's own allocator** and
///   drops it.
///
/// ## Why both symbols?
///
/// The plugin's `cdylib` and the host `truckpilot-core` are independent
/// compilation units. Even when both use Rust's default `System`
/// allocator, mixing `Box::into_raw` from one and `Box::from_raw` in
/// the other crosses an allocator boundary that is **not** guaranteed
/// stable: a future change to either side's `#[global_allocator]`
/// silently produces undefined behaviour. Pairing the two symbols
/// guarantees the destructor runs in the same allocator world as the
/// constructor.
///
/// ## Requirements on `$plugin_ty`
/// - Implements [`Plugin`] (the trait above).
/// - Implements `Default` (used by the generated `create_plugin`).
///
/// ## Example
/// ```ignore
/// use truckpilot_plugin_api::{export_plugin, Plugin};
///
/// #[derive(Default)]
/// struct MyPlugin;
/// impl Plugin for MyPlugin { /* ... */ }
///
/// export_plugin!(MyPlugin);
/// ```
#[macro_export]
macro_rules! export_plugin {
    ($plugin_ty:ty) => {
        /// Construct the plugin instance owned by this DLL.
        ///
        /// # Safety
        ///
        /// Returned pointer must be passed back to `destroy_plugin` from
        /// **this same DLL**, never freed by the caller directly.
        #[allow(improper_ctypes_definitions)]
        #[no_mangle]
        pub extern "C" fn create_plugin() -> *mut dyn $crate::Plugin {
            ::std::boxed::Box::into_raw(::std::boxed::Box::new(
                <$plugin_ty as ::std::default::Default>::default(),
            )) as *mut dyn $crate::Plugin
        }

        /// Drop a plugin instance previously returned by `create_plugin`.
        ///
        /// # Safety
        ///
        /// `ptr` must be a non-aliased pointer that was returned by
        /// `create_plugin` from **this DLL** and has not yet been
        /// passed to `destroy_plugin`. Passing a null pointer is a
        /// no-op.
        #[allow(improper_ctypes_definitions)]
        #[no_mangle]
        pub unsafe extern "C" fn destroy_plugin(ptr: *mut dyn $crate::Plugin) {
            if !ptr.is_null() {
                drop(::std::boxed::Box::from_raw(ptr));
            }
        }
    };
}

#[cfg(test)]
mod ctx_tests {
    use super::*;

    #[test]
    fn dt_s_defaults_to_0_02() {
        let ctx = PluginContext::new("test", SharedBlackboard::new());
        assert!((ctx.dt_s - 0.02).abs() < 1e-9);
    }

    #[test]
    fn with_dt_sets_value_and_clamps_min() {
        let bb = SharedBlackboard::new();
        let ctx = PluginContext::new("test", bb.clone()).with_dt(0.005);
        assert!((ctx.dt_s - 0.005).abs() < 1e-9);
        let ctx2 = PluginContext::new("test", bb).with_dt(0.0);
        assert!((ctx2.dt_s - 0.001).abs() < 1e-9, "dt_s={}", ctx2.dt_s);
    }

    #[test]
    fn test_default_tick_phase_is_phase_c() {
        let ctx = PluginContext::new("p", SharedBlackboard::new());
        assert_eq!(ctx.tick_phase, TickPhase::PhaseC);
    }

    #[test]
    fn test_default_tick_count_is_zero() {
        let ctx = PluginContext::new("p", SharedBlackboard::new());
        assert_eq!(ctx.tick_count, 0);
    }

    #[test]
    fn test_with_phase_chainable() {
        let ctx = PluginContext::new("p", SharedBlackboard::new())
            .with_dt(0.1)
            .with_phase(TickPhase::PhaseA);
        assert_eq!(ctx.tick_phase, TickPhase::PhaseA);
        assert!((ctx.dt_s - 0.1).abs() < 1e-9);
    }

    #[test]
    fn test_with_tick_count_chainable() {
        let ctx = PluginContext::new("p", SharedBlackboard::new())
            .with_phase(TickPhase::PhaseB)
            .with_tick_count(123);
        assert_eq!(ctx.tick_phase, TickPhase::PhaseB);
        assert_eq!(ctx.tick_count, 123);
    }

    #[test]
    fn test_is_active_reads_blackboard() {
        let ctx = PluginContext::test();
        assert!(!ctx.is_active());
        ctx.blackboard.set("autopilot.state", "Active");
        assert!(ctx.is_active());
        ctx.blackboard.set("autopilot.state", "Paused");
        assert!(!ctx.is_active());
    }

    #[test]
    fn test_is_engaged_for_engaging_active_paused() {
        for state in ["Engaging", "Active", "Paused"] {
            let ctx = PluginContext::test();
            ctx.blackboard.set("autopilot.state", state);
            assert!(ctx.is_engaged(), "state={state} should be engaged");
        }
        for state in ["Off", "Fault", "SomethingElse"] {
            let ctx = PluginContext::test();
            ctx.blackboard.set("autopilot.state", state);
            assert!(!ctx.is_engaged(), "state={state} should not be engaged");
        }
        // Missing key: not engaged.
        assert!(!PluginContext::test().is_engaged());
    }

    #[test]
    fn test_is_fault_only_for_fault() {
        assert!(PluginContext::test_fault().is_fault());
        assert!(!PluginContext::test_active().is_fault());
        assert!(!PluginContext::test_off().is_fault());
        assert!(!PluginContext::test().is_fault());
    }

    #[test]
    fn test_is_replan_tick_at_phase_a_multiple_of_50() {
        // Phase A + multiple of 50 → true (incl. 0).
        assert!(PluginContext::test()
            .with_phase(TickPhase::PhaseA)
            .with_tick_count(0)
            .is_replan_tick());
        assert!(PluginContext::test()
            .with_phase(TickPhase::PhaseA)
            .with_tick_count(50)
            .is_replan_tick());
        assert!(PluginContext::test()
            .with_phase(TickPhase::PhaseA)
            .with_tick_count(150)
            .is_replan_tick());
        // Phase A + non-multiple → false.
        assert!(!PluginContext::test()
            .with_phase(TickPhase::PhaseA)
            .with_tick_count(49)
            .is_replan_tick());
        // Other phase + multiple → false.
        for phase in [TickPhase::PhaseB, TickPhase::PhaseC, TickPhase::PostPhase] {
            assert!(!PluginContext::test()
                .with_phase(phase)
                .with_tick_count(50)
                .is_replan_tick());
        }
    }

    #[test]
    fn test_log_target_replaces_hyphens() {
        let ctx = PluginContext::new("lane-keeper", SharedBlackboard::new());
        assert_eq!(ctx.log_target(), "truckpilot_plugin_lane_keeper");
        let ctx2 = PluginContext::new("acc", SharedBlackboard::new());
        assert_eq!(ctx2.log_target(), "truckpilot_plugin_acc");
        let ctx3 = PluginContext::new("a-b-c", SharedBlackboard::new());
        assert_eq!(ctx3.log_target(), "truckpilot_plugin_a_b_c");
    }

    #[test]
    fn test_test_helpers_set_correct_state() {
        assert_eq!(PluginContext::test_off().state().as_deref(), Some("Off"));
        assert_eq!(
            PluginContext::test_engaging().state().as_deref(),
            Some("Engaging")
        );
        assert_eq!(
            PluginContext::test_active().state().as_deref(),
            Some("Active")
        );
        assert_eq!(
            PluginContext::test_paused().state().as_deref(),
            Some("Paused")
        );
        assert_eq!(
            PluginContext::test_fault().state().as_deref(),
            Some("Fault")
        );
        assert_eq!(PluginContext::test().state(), None);

        let ctx = PluginContext::test_with_dt(0.05);
        assert!((ctx.dt_s - 0.05).abs() < 1e-9);

        let ctx = PluginContext::test_with_phase(TickPhase::PhaseB);
        assert_eq!(ctx.tick_phase, TickPhase::PhaseB);

        let ctx = PluginContext::test_with_tick_count(7);
        assert_eq!(ctx.tick_count, 7);

        let ctx = PluginContext::test_replan();
        assert_eq!(ctx.tick_phase, TickPhase::PhaseA);
        assert_eq!(ctx.tick_count, 50);
        assert!(ctx.is_replan_tick());
    }
}
