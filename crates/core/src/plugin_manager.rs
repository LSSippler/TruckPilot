//! Plugin manager — handles loading, hot-reloading, and ticking plugins.
//!
//! **Hot-Reload is currently DISABLED** (see plugin-api docs for ABI reasons).
//! The notify watcher is present but reloads are no-ops until ABI stability is restored.
//!
//! Race-condition-safe: reloads are queued and applied between tick() calls.

use std::collections::{HashMap, HashSet};
use std::mem::ManuallyDrop;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Duration, Instant};

use libloading::{Library, Symbol};
use notify::{RecommendedWatcher, RecursiveMode, Watcher};
use tracing::{error, info, warn};
use truckpilot_plugin_api::graph::RouterGraph;
use truckpilot_plugin_api::{
    ControlOutput, ControlRequest, LogLevel, LogSinkWrapper, Plugin, PluginContext,
    SharedBlackboard, SharedFrameStore, Telemetry, TickPhase,
};

/// Per-plugin configuration loaded from `truckpilot.toml`.
pub struct PluginTomlConfig {
    /// Whether to call `on_load` and tick this plugin.
    pub enabled: bool,
    /// Extra TOML fields pre-serialized as `(blackboard_key, value)` pairs.
    /// Keys are already namespaced: `{plugin_name_underscored}.{field}`.
    pub extra: Vec<(String, String)>,
}

/// Build a log-sink that routes plugin log calls into the host's tracing
/// subscriber. The plugin's `target` (e.g. `truckpilot_plugin_sign_vision`)
/// is embedded as a prefix in the message because `tracing` macro targets
/// must be compile-time constants and cannot be runtime `&str` values.
fn make_log_sink() -> LogSinkWrapper {
    LogSinkWrapper(Arc::new(|level, target, message| {
        // Include target as a prefix so it's visible in structured output.
        match level {
            LogLevel::Trace => tracing::trace!(plugin_target = target, "{}", message),
            LogLevel::Debug => tracing::debug!(plugin_target = target, "{}", message),
            LogLevel::Info => tracing::info!(plugin_target = target, "{}", message),
            LogLevel::Warn => tracing::warn!(plugin_target = target, "{}", message),
            LogLevel::Error => tracing::error!(plugin_target = target, "{}", message),
        }
    }))
}

/// Phase-6.2b scheduler gate. Returns `true` when a plugin in `phase`
/// should be ticked on the cycle identified by `tick_count`.
fn should_tick(phase: TickPhase, tick_count: u64) -> bool {
    match phase {
        TickPhase::PhaseA => tick_count.is_multiple_of(50),
        TickPhase::PhaseB => tick_count.is_multiple_of(5),
        TickPhase::PhaseC | TickPhase::PostPhase => true,
    }
}

// Trait-object pointers (`*mut dyn Plugin`) are fat (data + vtable),
// so they aren't strict-C FFI-safe. The plugin DLL and host are both
// Rust though, and they agree on the layout — same `extern "C"`
// boundary the `export_plugin!` macro uses on the plugin side.
#[allow(improper_ctypes_definitions)]
mod ffi {
    use truckpilot_plugin_api::Plugin;

    /// FFI signature of the `create_plugin` symbol every plugin DLL must export.
    pub type CreateFn = unsafe extern "C" fn() -> *mut dyn Plugin;

    /// FFI signature of the `destroy_plugin` symbol every plugin DLL must export.
    /// Called once on unload to free the instance inside the DLL's own allocator.
    pub type DestroyFn = unsafe extern "C" fn(*mut dyn Plugin);
}
use ffi::{CreateFn, DestroyFn};

/// A loaded plugin with its backing library.
///
/// ## Drop ordering
/// `LoadedPlugin` has a custom [`Drop`] impl that releases the plugin
/// instance via the DLL-provided `destroy_plugin` *before* the library
/// is unloaded. Field declaration order matters: `plugin` before
/// `_lib` so the implicit field drop after our manual cleanup keeps
/// the library alive long enough.
pub struct LoadedPlugin {
    pub name: String,
    pub version: String,
    pub path: PathBuf,
    pub enabled: bool,
    /// `true` after a successful [`Plugin::on_load`]. Used to skip `on_load` for
    /// config-disabled plugins and to call `on_load` when enabled at runtime.
    pub initialized: bool,
    /// The plugin instance.
    ///
    /// Wrapped in `ManuallyDrop` because the allocation came from the
    /// plugin DLL's `Box::new` and **must** be freed by the DLL's
    /// matching `destroy_plugin`. Letting Rust's automatic `Box` drop
    /// run on the host side would deallocate via the host allocator —
    /// undefined behaviour if host and plugin disagree on the global
    /// allocator. See [`Drop`] below.
    plugin: ManuallyDrop<Box<dyn Plugin>>,
    /// `destroy_plugin` function pointer extracted from the DLL.
    /// Valid as long as `_lib` is loaded.
    destroy_fn: DestroyFn,
    /// Library must outlive the plugin instance.
    _lib: Library,
}

impl Drop for LoadedPlugin {
    fn drop(&mut self) {
        // SAFETY:
        // - `plugin` is initialised in `load_plugin_from_path` and never
        //   touched by `ManuallyDrop::take` elsewhere, so the move is
        //   exactly once.
        // - `destroy_fn` was extracted from the same `Library` that
        //   produced the pointer, so the raw pointer returns to the
        //   allocator that created it.
        // - `_lib` is dropped *after* this method returns (field drop
        //   order: declaration order), so the function pointer is
        //   still callable here.
        unsafe {
            let boxed = ManuallyDrop::take(&mut self.plugin);
            let ptr = Box::into_raw(boxed);
            (self.destroy_fn)(ptr);
        }
    }
}

pub struct PluginManager {
    plugins: Vec<LoadedPlugin>,
    plugin_dir: PathBuf,
    reload_queue: Arc<Mutex<Vec<PathBuf>>>,
    /// Shared blackboard — same instance across all plugins.
    pub blackboard: SharedBlackboard,
    /// Shared frame store for binary camera payloads (Phase 6.5c.2).
    /// Constructed once at daemon start, cloned into every
    /// [`PluginContext`] via [`PluginContext::with_frame_store`].
    pub frame_store: Arc<SharedFrameStore>,
    /// Monotonic tick counter. Incremented once per [`Self::tick_all`].
    /// Mirrored into [`PluginContext::tick_count`] so plugins can branch
    /// on cadence without keeping their own counters.
    tick_count: u64,
    _watcher: Option<RecommendedWatcher>,
    /// Shared routing graph (Phase 6.5q.1). Set by the daemon before
    /// `load_all()`; injected into every [`PluginContext`].
    pub graph: Option<Arc<RouterGraph>>,
    /// Shared route node IDs (Phase 6.5q.1). The router plugin updates
    /// this each tick; the state machine reads it for engage-time checks.
    pub route_node_ids: Arc<RwLock<HashSet<u64>>>,
    /// Per-plugin configuration loaded from `truckpilot.toml`.
    /// Key = plugin name. Missing key → default enabled=true, no extra keys.
    plugin_configs: HashMap<String, PluginTomlConfig>,
}

impl PluginManager {
    pub fn new(plugin_dir: PathBuf, plugin_configs: HashMap<String, PluginTomlConfig>) -> Self {
        let reload_queue = Arc::new(Mutex::new(Vec::new()));
        let queue_clone = reload_queue.clone();

        let watcher =
            match notify::recommended_watcher(move |res: notify::Result<notify::Event>| {
                if let Ok(event) = res {
                    if matches!(
                        event.kind,
                        notify::EventKind::Modify(_) | notify::EventKind::Create(_)
                    ) {
                        for path in event.paths {
                            if is_plugin_file(&path) {
                                if let Ok(mut q) = queue_clone.lock() {
                                    if !q.contains(&path) {
                                        q.push(path);
                                    }
                                }
                            }
                        }
                    }
                }
            }) {
                Ok(mut w) => {
                    if w.watch(&plugin_dir, RecursiveMode::NonRecursive).is_err() {
                        warn!("Failed to watch plugin dir {:?}", plugin_dir);
                    }
                    Some(w)
                }
                Err(e) => {
                    warn!("Failed to create file watcher: {}", e);
                    None
                }
            };

        Self {
            plugins: Vec::new(),
            plugin_dir,
            reload_queue,
            blackboard: SharedBlackboard::new(),
            frame_store: Arc::new(SharedFrameStore::new()),
            tick_count: 0,
            _watcher: watcher,
            graph: None,
            route_node_ids: Arc::new(RwLock::new(HashSet::new())),
            plugin_configs,
        }
    }

    /// Discover and load all plugins in the directory.
    pub fn load_all(&mut self) {
        let entries = match std::fs::read_dir(&self.plugin_dir) {
            Ok(e) => e,
            Err(e) => {
                warn!("Cannot read plugin dir {:?}: {}", self.plugin_dir, e);
                return;
            }
        };

        for entry in entries.flatten() {
            let path = entry.path();
            if is_plugin_file(&path) {
                self.load_plugin(&path);
            }
        }
    }

    fn load_plugin(&mut self, path: &Path) {
        match unsafe { load_plugin_from_path(path) } {
            Ok(mut loaded) => {
                // Apply enabled flag from config; default to true for backwards compat.
                if let Some(cfg) = self.plugin_configs.get(&loaded.name) {
                    loaded.enabled = cfg.enabled;
                }
                if loaded.enabled {
                    // Seed TOML config values to blackboard BEFORE on_load reads them.
                    if let Some(cfg) = self.plugin_configs.get(&loaded.name) {
                        for (key, value) in &cfg.extra {
                            self.blackboard.set(key.clone(), value.clone());
                        }
                    }
                    self.run_plugin_on_load(&mut loaded);
                    info!("Loaded plugin: {} v{}", loaded.name, loaded.version);
                } else {
                    info!(
                        "plugin '{}' disabled by config, skipping on_load",
                        loaded.name
                    );
                    info!(
                        "Loaded plugin: {} v{} (disabled by config)",
                        loaded.name, loaded.version
                    );
                }
                self.plugins.push(loaded);
                self.publish_loaded_names();
            }
            Err(e) => {
                error!("Failed to load plugin {:?}: {}", path, e);
            }
        }
    }

    /// Returns whether the named plugin is currently enabled.
    /// Returns `false` if the plugin is not loaded.
    pub fn is_plugin_enabled(&self, name: &str) -> bool {
        self.plugins.iter().any(|p| p.name == name && p.enabled)
    }

    /// Publish the comma-joined list of currently enabled plugin names to
    /// `plugins.loaded`. Read by the state machine's
    /// `check_critical_plugins` precondition check.
    fn publish_loaded_names(&self) {
        let joined: String = self
            .plugins
            .iter()
            .filter(|p| p.enabled)
            .map(|p| p.name.as_str())
            .collect::<Vec<_>>()
            .join(",");
        self.blackboard.set("plugins.loaded", joined);
    }

    /// Process pending reload events. Call between tick() invocations only.
    pub fn process_reloads(&mut self) {
        let paths: Vec<PathBuf> = {
            let mut q = match self.reload_queue.lock() {
                Ok(q) => q,
                Err(e) => e.into_inner(), // Poisoned: recover the value anyway
            };
            std::mem::take(&mut *q)
        };

        for path in paths {
            self.reload_plugin(&path);
        }
    }

    fn reload_plugin(&mut self, path: &Path) {
        let canonical = path.canonicalize().unwrap_or_else(|_| path.to_path_buf());

        let existing_idx = self.plugins.iter().position(|p| {
            p.path
                .canonicalize()
                .map(|c| c == canonical)
                .unwrap_or(false)
        });

        if let Some(idx) = existing_idx {
            let mut old = self.plugins.remove(idx);
            info!("Reloading plugin: {}", old.name);
            if old.initialized {
                old.plugin.on_unload();
            }
            drop(old);
            // Give Windows time to release the DLL handle before loading the new version
            std::thread::sleep(Duration::from_millis(200));
        }

        self.load_plugin(path);
    }

    /// Tick all enabled plugins in order, then arbitrate a single
    /// safe [`ControlOutput`] from their opinions.
    ///
    /// Each plugin sees two callbacks per tick:
    ///   1. [`Plugin::tick`] — for side effects (blackboard writes,
    ///      logging, internal state). Direct writes to a *legacy*
    ///      `ControlOutput` are honoured only on axes where no
    ///      `ControlRequest` arrived.
    ///   2. [`Plugin::tick_request`] — for opinions on
    ///      steering/throttle/brake that participate in arbitration.
    ///
    /// Both calls are wrapped in [`catch_unwind`]: a plugin that
    /// panics is disabled (no further ticks until reload) and the
    /// daemon keeps running. See [`arbitrate`] for the merge rules.
    pub fn tick_all(
        &mut self,
        telemetry: Option<&Telemetry>,
        output: &mut ControlOutput,
        dt_s: f64,
    ) {
        self.tick_count = self.tick_count.wrapping_add(1);
        let tick_count = self.tick_count;

        // Legacy bucket: plugins still on the old `tick(&mut output)`
        // API write here. Reset every tick so stale values don't stick.
        let mut legacy = ControlOutput::default();
        let mut requests: Vec<ControlRequest> = Vec::new();

        // Output plugins (PostPhase) must observe the FINAL arbitrated output,
        // not intermediate legacy values. Collect their indices here, skip them
        // in the main loop, then tick each one after arbitration with `*output`.
        let post_phase_indices: Vec<usize> = self
            .plugins
            .iter()
            .enumerate()
            .filter(|(_, p)| p.enabled && p.plugin.default_phase() == TickPhase::PostPhase)
            .map(|(i, _)| i)
            .collect();

        // Set to true whenever a plugin is disabled due to a panic so we
        // refresh `plugins.loaded` in the blackboard after the loop.
        let mut any_disabled = false;

        for (i, p) in self
            .plugins
            .iter_mut()
            .enumerate()
            .filter(|(_, p)| p.enabled)
        {
            if post_phase_indices.contains(&i) {
                continue;
            }
            let phase = p.plugin.default_phase();
            if !should_tick(phase, tick_count) {
                continue;
            }
            let plugin_start = Instant::now();
            let mut ctx = PluginContext::new(p.name.clone(), self.blackboard.clone())
                .with_dt(dt_s)
                .with_phase(phase)
                .with_tick_count(tick_count)
                .with_frame_store(Arc::clone(&self.frame_store))
                .with_log_sink(make_log_sink());
            ctx.graph = self.graph.clone();
            ctx.route_node_ids = Some(Arc::clone(&self.route_node_ids));

            // Side-effect path: blackboard writes, internal state, etc.
            // AssertUnwindSafe: we accept that a panicking plugin may
            // leave its own state inconsistent — that's why we then
            // disable it. The host's invariants (legacy/requests) are
            // local to this method and will be discarded on panic.
            let tick_result = catch_unwind(AssertUnwindSafe(|| {
                p.plugin.tick(telemetry, &mut legacy, &ctx);
            }));
            if let Err(panic) = tick_result {
                log_plugin_panic(&p.name, "tick", panic);
                p.enabled = false;
                any_disabled = true;
                continue;
            }

            // Arbitration path.
            let req_result =
                catch_unwind(AssertUnwindSafe(|| p.plugin.tick_request(telemetry, &ctx)));
            match req_result {
                Ok(Some(req)) => requests.push(req),
                Ok(None) => {}
                Err(panic) => {
                    log_plugin_panic(&p.name, "tick_request", panic);
                    p.enabled = false;
                    any_disabled = true;
                }
            }
            let plugin_elapsed = plugin_start.elapsed();
            if plugin_elapsed.as_millis() > 30 {
                warn!(
                    "[tick-profile] plugin '{}' took {} ms (tick={})",
                    p.name,
                    plugin_elapsed.as_millis(),
                    tick_count
                );
            }
        }

        // Refresh the blackboard list if any plugin was panic-disabled above.
        if any_disabled {
            self.publish_loaded_names();
        }

        *output = arbitrate(legacy, &requests);

        // Tick all PostPhase (output) plugins with the final arbitrated value.
        // Output plugins must not submit ControlRequests, so tick_request is
        // intentionally not called here.
        let mut any_post_disabled = false;
        for &idx in &post_phase_indices {
            // Block so the mutable borrow of `p` is released before
            // `publish_loaded_names` needs `&self`.
            let panicked = {
                let p = &mut self.plugins[idx];
                if !p.enabled {
                    false
                } else {
                    let plugin_start = Instant::now();
                    let phase = p.plugin.default_phase();
                    let mut ctx = PluginContext::new(p.name.clone(), self.blackboard.clone())
                        .with_dt(dt_s)
                        .with_phase(phase)
                        .with_tick_count(tick_count)
                        .with_frame_store(Arc::clone(&self.frame_store))
                        .with_log_sink(make_log_sink());
                    ctx.graph = self.graph.clone();
                    ctx.route_node_ids = Some(Arc::clone(&self.route_node_ids));
                    let tick_result = catch_unwind(AssertUnwindSafe(|| {
                        p.plugin.tick(telemetry, output, &ctx);
                    }));
                    let plugin_elapsed = plugin_start.elapsed();
                    if plugin_elapsed.as_millis() > 30 {
                        warn!(
                            "[tick-profile] plugin '{}' took {} ms (tick={})",
                            p.name,
                            plugin_elapsed.as_millis(),
                            tick_count
                        );
                    }
                    if let Err(panic) = tick_result {
                        log_plugin_panic(&p.name, "tick", panic);
                        p.enabled = false;
                        true
                    } else {
                        false
                    }
                }
            };
            if panicked {
                any_post_disabled = true;
            }
        }
        // Update the blackboard if any output plugin was panic-disabled so the
        // watchdog sees the change and can react.
        if any_post_disabled {
            self.publish_loaded_names();
        }
    }

    pub fn list(&self) -> Vec<truckpilot_ipc_protocol::PluginInfo> {
        self.plugins
            .iter()
            .map(|p| truckpilot_ipc_protocol::PluginInfo {
                name: p.name.clone(),
                version: p.version.clone(),
                enabled: p.enabled,
            })
            .collect()
    }

    /// Borrow a plugin's settings JSON-Schema by name.
    pub fn schema_string(&self, name: &str) -> Option<&str> {
        self.plugins
            .iter()
            .find(|p| p.name == name)
            .map(|p| p.plugin.settings_schema())
    }

    pub fn set_enabled(&mut self, name: &str, enabled: bool) -> bool {
        let Some(idx) = self.plugins.iter().position(|p| p.name == name) else {
            return false;
        };
        let was_enabled = self.plugins[idx].enabled;
        let needs_on_load = enabled && !was_enabled && !self.plugins[idx].initialized;
        self.plugins[idx].enabled = enabled;
        if needs_on_load {
            if let Some(cfg) = self.plugin_configs.get(name) {
                for (key, value) in &cfg.extra {
                    self.blackboard.set(key.clone(), value.clone());
                }
            }
            let blackboard = self.blackboard.clone();
            let frame_store = Arc::clone(&self.frame_store);
            let graph = self.graph.clone();
            let route_node_ids = Arc::clone(&self.route_node_ids);
            Self::run_on_load_for(
                &mut self.plugins[idx],
                &blackboard,
                &frame_store,
                &graph,
                &route_node_ids,
            );
        }
        info!(
            "Plugin {} {}",
            name,
            if enabled { "enabled" } else { "disabled" }
        );
        self.publish_loaded_names();
        true
    }

    /// Run [`Plugin::on_load`] for `loaded` using the manager's shared context.
    fn run_plugin_on_load(&self, loaded: &mut LoadedPlugin) {
        Self::run_on_load_for(
            loaded,
            &self.blackboard,
            &self.frame_store,
            &self.graph,
            &self.route_node_ids,
        );
    }

    fn run_on_load_for(
        loaded: &mut LoadedPlugin,
        blackboard: &SharedBlackboard,
        frame_store: &Arc<SharedFrameStore>,
        graph: &Option<Arc<RouterGraph>>,
        route_node_ids: &Arc<RwLock<HashSet<u64>>>,
    ) {
        debug_assert!(!loaded.initialized, "on_load must not run twice");
        let mut ctx = PluginContext::new(loaded.name.clone(), blackboard.clone())
            .with_frame_store(Arc::clone(frame_store))
            .with_log_sink(make_log_sink());
        ctx.graph = graph.clone();
        ctx.route_node_ids = Some(Arc::clone(route_node_ids));
        loaded.plugin.on_load(&ctx);
        loaded.initialized = true;
    }

    #[allow(dead_code)]
    pub fn unload_all(&mut self) {
        for p in self.plugins.iter_mut() {
            if p.initialized {
                p.plugin.on_unload();
            }
        }
        self.plugins.clear();
    }
}

/// Decode a [`catch_unwind`] panic payload into a printable string and
/// emit a structured `error!` log. Used by [`PluginManager::tick_all`]
/// to surface a misbehaving plugin without taking the daemon down.
fn log_plugin_panic(plugin: &str, method: &str, payload: Box<dyn std::any::Any + Send>) {
    let msg = if let Some(s) = payload.downcast_ref::<&'static str>() {
        (*s).to_string()
    } else if let Some(s) = payload.downcast_ref::<String>() {
        s.clone()
    } else {
        "<non-string panic payload>".to_string()
    };
    error!(
        "plugin '{plugin}' panicked in {method}(): {msg} \
         — plugin disabled until reload"
    );
}

/// Combine plugin [`ControlRequest`]s and the legacy [`ControlOutput`]
/// into a single safe output.
///
/// Rules (safety-first):
/// - **brake**: maximum across `legacy.brake` and every `Some(brake)`
///   in `requests`. Anybody can ask for *more* braking, nobody can
///   override another plugin's brake demand.
/// - **throttle**: if `final_brake > 0`, throttle is the *minimum*
///   across `legacy.throttle` and every `Some(throttle)` in `requests`
///   — a brake request always cuts throttle. Otherwise the highest-
///   `priority` request wins; ties go to the *later* request (load
///   order).
/// - **steering**: highest-`priority` opinion wins. Ties: later
///   request wins. If no plugin returns a steering opinion,
///   `legacy.steering` is used.
pub fn arbitrate(legacy: ControlOutput, requests: &[ControlRequest]) -> ControlOutput {
    // Brake: max(legacy, every Some(brake))
    let brake = requests
        .iter()
        .filter_map(|r| r.brake)
        .fold(legacy.brake, f64::max);
    let any_brake = brake > 0.0;

    // Throttle: min when braking, priority-wins otherwise.
    let throttle = if any_brake {
        requests
            .iter()
            .filter_map(|r| r.throttle)
            .fold(legacy.throttle, f64::min)
    } else {
        let mut best = legacy.throttle;
        let mut best_pri = i32::MIN;
        for r in requests {
            if let Some(t) = r.throttle {
                if r.priority >= best_pri {
                    best = t;
                    best_pri = r.priority;
                }
            }
        }
        best
    };

    // Steering: priority-wins; ties → later request.
    let mut steering = legacy.steering;
    let mut steering_pri = i32::MIN;
    for r in requests {
        if let Some(s) = r.steering {
            if r.priority >= steering_pri {
                steering = s;
                steering_pri = r.priority;
            }
        }
    }

    ControlOutput {
        steering,
        throttle,
        brake,
    }
}

fn is_plugin_file(path: &Path) -> bool {
    matches!(
        path.extension().and_then(|s| s.to_str()),
        Some("dll" | "so" | "dylib")
    )
}

unsafe fn load_plugin_from_path(path: &Path) -> Result<LoadedPlugin, String> {
    let lib = Library::new(path).map_err(|e| format!("library load: {e}"))?;

    let create: Symbol<CreateFn> = lib
        .get(b"create_plugin")
        .map_err(|e| format!("missing create_plugin symbol: {e}"))?;

    let destroy: Symbol<DestroyFn> = lib.get(b"destroy_plugin").map_err(|e| {
        format!(
            "missing destroy_plugin symbol: {e} \
             (plugin must use truckpilot_plugin_api::export_plugin!)"
        )
    })?;
    let destroy_fn: DestroyFn = *destroy;

    let plugin_ptr = create();
    if plugin_ptr.is_null() {
        return Err("create_plugin returned null".into());
    }

    let plugin = Box::from_raw(plugin_ptr);

    let name = plugin.name().to_string();
    let version = plugin.version().to_string();

    Ok(LoadedPlugin {
        name,
        version,
        path: path.to_path_buf(),
        enabled: true,
        initialized: false,
        plugin: ManuallyDrop::new(plugin),
        destroy_fn,
        _lib: lib,
    })
}

#[cfg(test)]
mod should_tick_tests {
    use super::should_tick;
    use truckpilot_plugin_api::TickPhase;

    #[test]
    fn phase_c_ticks_every_cycle() {
        for n in [1u64, 2, 7, 49, 50, 51, 99, 100] {
            assert!(should_tick(TickPhase::PhaseC, n), "phase C tick_count={n}");
        }
    }

    #[test]
    fn phase_a_ticks_every_50th() {
        assert!(should_tick(TickPhase::PhaseA, 50));
        assert!(should_tick(TickPhase::PhaseA, 100));
        assert!(should_tick(TickPhase::PhaseA, 250));
        assert!(!should_tick(TickPhase::PhaseA, 49));
        assert!(!should_tick(TickPhase::PhaseA, 51));
    }

    #[test]
    fn phase_b_ticks_every_5th() {
        assert!(should_tick(TickPhase::PhaseB, 5));
        assert!(should_tick(TickPhase::PhaseB, 50));
        assert!(!should_tick(TickPhase::PhaseB, 1));
        assert!(!should_tick(TickPhase::PhaseB, 4));
        assert!(!should_tick(TickPhase::PhaseB, 6));
    }

    #[test]
    fn post_phase_ticks_every_cycle() {
        for n in [1u64, 2, 50, 99, 100] {
            assert!(should_tick(TickPhase::PostPhase, n));
        }
    }
}

#[cfg(test)]
mod arbitrate_tests {
    use super::*;

    fn legacy(steering: f64, throttle: f64, brake: f64) -> ControlOutput {
        ControlOutput {
            steering,
            throttle,
            brake,
        }
    }

    #[test]
    fn no_requests_returns_legacy_unchanged() {
        let out = arbitrate(legacy(0.3, 0.5, 0.0), &[]);
        assert!((out.steering - 0.3).abs() < 1e-9);
        assert!((out.throttle - 0.5).abs() < 1e-9);
        assert!((out.brake - 0.0).abs() < 1e-9);
    }

    #[test]
    fn brake_max_wins() {
        let reqs = [
            ControlRequest {
                brake: Some(0.2),
                ..Default::default()
            },
            ControlRequest {
                brake: Some(0.7),
                ..Default::default()
            },
            ControlRequest {
                brake: Some(0.4),
                ..Default::default()
            },
        ];
        let out = arbitrate(legacy(0.0, 0.0, 0.1), &reqs);
        assert!((out.brake - 0.7).abs() < 1e-9);
    }

    #[test]
    fn brake_cuts_throttle_to_min() {
        let reqs = [
            ControlRequest {
                throttle: Some(0.8),
                priority: 100,
                ..Default::default()
            },
            ControlRequest {
                brake: Some(0.5),
                throttle: Some(0.0),
                priority: 50,
                ..Default::default()
            },
        ];
        let out = arbitrate(legacy(0.0, 0.6, 0.0), &reqs);
        assert!((out.brake - 0.5).abs() < 1e-9);
        // Despite high-priority throttle=0.8, brake is active so min(0.6,0.8,0.0)=0.0
        assert!((out.throttle - 0.0).abs() < 1e-9);
    }

    #[test]
    fn no_brake_throttle_priority_wins() {
        let reqs = [
            ControlRequest {
                throttle: Some(0.3),
                priority: 10,
                ..Default::default()
            },
            ControlRequest {
                throttle: Some(0.7),
                priority: 50,
                ..Default::default()
            },
            ControlRequest {
                throttle: Some(0.5),
                priority: 20,
                ..Default::default()
            },
        ];
        let out = arbitrate(legacy(0.0, 0.0, 0.0), &reqs);
        assert!((out.throttle - 0.7).abs() < 1e-9);
        assert!((out.brake - 0.0).abs() < 1e-9);
    }

    #[test]
    fn steering_priority_wins() {
        let reqs = [
            ControlRequest {
                steering: Some(0.1),
                priority: 0,
                ..Default::default()
            },
            ControlRequest {
                steering: Some(-0.4),
                priority: 100,
                ..Default::default()
            },
            ControlRequest {
                steering: Some(0.9),
                priority: 50,
                ..Default::default()
            },
        ];
        let out = arbitrate(legacy(0.0, 0.0, 0.0), &reqs);
        assert!((out.steering - -0.4).abs() < 1e-9);
    }

    #[test]
    fn equal_priority_later_wins() {
        let reqs = [
            ControlRequest {
                steering: Some(0.2),
                priority: 50,
                ..Default::default()
            },
            ControlRequest {
                steering: Some(-0.2),
                priority: 50,
                ..Default::default()
            },
        ];
        let out = arbitrate(legacy(0.0, 0.0, 0.0), &reqs);
        assert!((out.steering - -0.2).abs() < 1e-9);
    }

    #[test]
    fn axis_with_no_opinion_keeps_legacy() {
        let reqs = [ControlRequest {
            steering: Some(0.5),
            ..Default::default()
        }];
        let out = arbitrate(legacy(0.0, 0.4, 0.0), &reqs);
        // throttle/brake had no requests at all → legacy passes through
        assert!((out.throttle - 0.4).abs() < 1e-9);
        assert!((out.brake - 0.0).abs() < 1e-9);
        assert!((out.steering - 0.5).abs() < 1e-9);
    }
}

#[cfg(test)]
mod frame_store_tests {
    //! Phase 6.5c.2 Step 2 — daemon wiring.
    //!
    //! These tests bypass the cdylib boundary and inject `LoadedPlugin`
    //! records directly. The whole point of Step 2 is the `PluginContext`
    //! plumbing, so it's enough to drive `tick_all` with in-process
    //! plugins and check what they observe.
    //!
    //! `LoadedPlugin` normally owns a `Library` and a `destroy_fn`; for
    //! tests we substitute a dummy library handle and a no-op destroyer
    //! (the inner `Box<dyn Plugin>` then leaks at the end of the test —
    //! acceptable, the test process exits immediately).
    use super::*;
    use std::sync::Mutex as StdMutex;
    use truckpilot_plugin_api::{ControlOutput, ControlRequest, SharedFrame};

    /// A Plugin that records what it saw in `on_load` and `tick`.
    struct ProbePlugin {
        name: String,
        saw_store_on_load: Arc<StdMutex<Option<bool>>>,
        last_seen_frame_id: Arc<StdMutex<Option<u64>>>,
        /// If set, this plugin will publish a frame with this id on tick.
        publish_frame_id: Option<u64>,
        /// If set, this plugin reads a frame from this key on tick.
        read_key: Option<String>,
        last_read_frame: Arc<StdMutex<Option<Arc<SharedFrame>>>>,
    }

    impl ProbePlugin {
        fn new(name: &str) -> Self {
            Self {
                name: name.into(),
                saw_store_on_load: Arc::new(StdMutex::new(None)),
                last_seen_frame_id: Arc::new(StdMutex::new(None)),
                publish_frame_id: None,
                read_key: None,
                last_read_frame: Arc::new(StdMutex::new(None)),
            }
        }
    }

    impl Plugin for ProbePlugin {
        fn name(&self) -> &str {
            &self.name
        }
        fn version(&self) -> &str {
            "0.0.0"
        }
        fn settings_schema(&self) -> &str {
            "{}"
        }
        fn on_load(&mut self, ctx: &PluginContext) {
            *self.saw_store_on_load.lock().unwrap() = Some(ctx.frame_store().is_some());
        }
        fn on_unload(&mut self) {}
        fn tick(
            &mut self,
            _telemetry: Option<&Telemetry>,
            _output: &mut ControlOutput,
            ctx: &PluginContext,
        ) {
            let store = ctx.frame_store().expect("daemon must wire frame_store");
            if let Some(id) = self.publish_frame_id {
                let frame = Arc::new(SharedFrame::new(
                    id,
                    1_000 + id,
                    320,
                    240,
                    Arc::new(vec![0xAA; 16]),
                ));
                store.set("camera.front", frame);
            }
            if let Some(key) = &self.read_key {
                if let Some(frame) = store.get(key) {
                    *self.last_seen_frame_id.lock().unwrap() = Some(frame.id);
                    *self.last_read_frame.lock().unwrap() = Some(frame);
                }
            }
        }
        fn tick_request(
            &mut self,
            _t: Option<&Telemetry>,
            _ctx: &PluginContext,
        ) -> Option<ControlRequest> {
            None
        }
        fn default_phase(&self) -> TickPhase {
            TickPhase::PhaseC
        }
    }

    /// Manually inject a `Plugin` into `PluginManager` without going
    /// through the cdylib loader. Runs `on_load` against the manager's
    /// real `frame_store`. Leaks the plugin box on drop (no destroy_fn).
    fn inject(mgr: &mut PluginManager, mut plugin: Box<dyn Plugin>) {
        let name = plugin.name().to_string();
        let version = plugin.version().to_string();
        let ctx = PluginContext::new(name.clone(), mgr.blackboard.clone())
            .with_frame_store(Arc::clone(&mgr.frame_store));
        plugin.on_load(&ctx);

        // No-op destroy: tests leak the box. Acceptable for #[cfg(test)].
        #[allow(improper_ctypes_definitions)]
        unsafe extern "C" fn noop_destroy(_p: *mut dyn Plugin) {}

        // A dummy `Library`: we need *some* `Library` value to satisfy
        // `LoadedPlugin._lib`. Load ourselves (the test binary) — that
        // is guaranteed to exist and the handle is benign.
        let lib = unsafe { Library::new(std::env::current_exe().unwrap()) }
            .expect("self-load for dummy Library handle");

        mgr.plugins.push(LoadedPlugin {
            name,
            version,
            path: PathBuf::new(),
            enabled: true,
            initialized: true,
            plugin: ManuallyDrop::new(plugin),
            destroy_fn: noop_destroy,
            _lib: lib,
        });
    }

    pub(super) fn inject_disabled(mgr: &mut PluginManager, plugin: Box<dyn Plugin>) {
        let name = plugin.name().to_string();
        let version = plugin.version().to_string();

        #[allow(improper_ctypes_definitions)]
        unsafe extern "C" fn noop_destroy(_p: *mut dyn Plugin) {}

        let lib = unsafe { Library::new(std::env::current_exe().unwrap()) }
            .expect("self-load for dummy Library handle");

        mgr.plugins.push(LoadedPlugin {
            name,
            version,
            path: PathBuf::new(),
            enabled: false,
            initialized: false,
            plugin: ManuallyDrop::new(plugin),
            destroy_fn: noop_destroy,
            _lib: lib,
        });
    }

    fn new_test_manager() -> PluginManager {
        // Plugin dir doesn't have to exist; we never call load_all.
        PluginManager::new(PathBuf::from("./does-not-exist-test-dir"), HashMap::new())
    }

    #[test]
    fn manager_constructs_a_frame_store() {
        let mgr = new_test_manager();
        assert!(mgr.frame_store.is_empty());
    }

    #[test]
    fn plugin_sees_frame_store_in_on_load() {
        let mut mgr = new_test_manager();
        let plugin = ProbePlugin::new("probe-load");
        let observed = Arc::clone(&plugin.saw_store_on_load);
        inject(&mut mgr, Box::new(plugin));
        assert_eq!(*observed.lock().unwrap(), Some(true));
    }

    #[test]
    fn plugin_sees_frame_store_in_tick() {
        let mut mgr = new_test_manager();
        let mut writer = ProbePlugin::new("writer");
        writer.publish_frame_id = Some(42);
        inject(&mut mgr, Box::new(writer));

        let mut out = ControlOutput::default();
        mgr.tick_all(None, &mut out, 0.02);

        let frame = mgr
            .frame_store
            .get("camera.front")
            .expect("writer should have published");
        assert_eq!(frame.id, 42);
    }

    #[test]
    fn two_plugins_share_one_frame_store() {
        let mut mgr = new_test_manager();

        // Pre-publish a frame from outside, so order of plugins in
        // tick_all doesn't matter for the assertion.
        let published = Arc::new(SharedFrame::new(7, 7_000, 320, 240, Arc::new(vec![1; 8])));
        mgr.frame_store.set("camera.front", Arc::clone(&published));

        let mut reader = ProbePlugin::new("reader");
        reader.read_key = Some("camera.front".to_string());
        let last_read_frame = Arc::clone(&reader.last_read_frame);
        let last_seen = Arc::clone(&reader.last_seen_frame_id);

        inject(&mut mgr, Box::new(reader));

        let mut out = ControlOutput::default();
        mgr.tick_all(None, &mut out, 0.02);

        assert_eq!(*last_seen.lock().unwrap(), Some(7));
        let read_back = last_read_frame
            .lock()
            .unwrap()
            .clone()
            .expect("reader saw frame");
        assert!(Arc::ptr_eq(&read_back, &published));
    }

    #[test]
    fn writer_then_reader_round_trip() {
        let mut mgr = new_test_manager();
        let mut writer = ProbePlugin::new("writer");
        writer.publish_frame_id = Some(99);
        let mut reader = ProbePlugin::new("reader");
        reader.read_key = Some("camera.front".to_string());
        let last_seen = Arc::clone(&reader.last_seen_frame_id);

        // Order matters: writer first, then reader, so within one
        // `tick_all` the reader observes the freshly published frame.
        inject(&mut mgr, Box::new(writer));
        inject(&mut mgr, Box::new(reader));

        let mut out = ControlOutput::default();
        mgr.tick_all(None, &mut out, 0.02);

        assert_eq!(*last_seen.lock().unwrap(), Some(99));
    }

    // --- PostPhase generic dispatch tests ---

    /// Writes a fixed steering value into the legacy output bucket (PhaseC).
    struct SteeringWriterPlugin {
        val: f64,
    }
    impl Plugin for SteeringWriterPlugin {
        fn name(&self) -> &str {
            "steering-writer"
        }
        fn version(&self) -> &str {
            "0.0.0"
        }
        fn settings_schema(&self) -> &str {
            "{}"
        }
        fn on_load(&mut self, _ctx: &PluginContext) {}
        fn on_unload(&mut self) {}
        fn tick(&mut self, _t: Option<&Telemetry>, out: &mut ControlOutput, _ctx: &PluginContext) {
            out.steering = self.val;
        }
        fn tick_request(
            &mut self,
            _t: Option<&Telemetry>,
            _ctx: &PluginContext,
        ) -> Option<ControlRequest> {
            None
        }
        fn default_phase(&self) -> TickPhase {
            TickPhase::PhaseC
        }
    }

    /// Captures the steering value passed to its `tick()` call (PostPhase).
    struct SteeringCapturePlugin {
        plugin_name: String,
        captured: Arc<StdMutex<Option<f64>>>,
    }
    impl SteeringCapturePlugin {
        fn new(name: &str) -> (Self, Arc<StdMutex<Option<f64>>>) {
            let cell = Arc::new(StdMutex::new(None));
            (
                Self {
                    plugin_name: name.into(),
                    captured: Arc::clone(&cell),
                },
                cell,
            )
        }
    }
    impl Plugin for SteeringCapturePlugin {
        fn name(&self) -> &str {
            &self.plugin_name
        }
        fn version(&self) -> &str {
            "0.0.0"
        }
        fn settings_schema(&self) -> &str {
            "{}"
        }
        fn on_load(&mut self, _ctx: &PluginContext) {}
        fn on_unload(&mut self) {}
        fn tick(&mut self, _t: Option<&Telemetry>, out: &mut ControlOutput, _ctx: &PluginContext) {
            *self.captured.lock().unwrap() = Some(out.steering);
        }
        fn tick_request(
            &mut self,
            _t: Option<&Telemetry>,
            _ctx: &PluginContext,
        ) -> Option<ControlRequest> {
            None
        }
        fn default_phase(&self) -> TickPhase {
            TickPhase::PostPhase
        }
    }

    #[test]
    fn post_phase_plugin_sees_arbitrated_steering() {
        let mut mgr = new_test_manager();
        inject(&mut mgr, Box::new(SteeringWriterPlugin { val: 0.5 }));
        let (capture, cell) = SteeringCapturePlugin::new("capture");
        inject(&mut mgr, Box::new(capture));

        let mut out = ControlOutput::default();
        mgr.tick_all(None, &mut out, 0.02);

        let seen = cell
            .lock()
            .unwrap()
            .expect("PostPhase plugin must be called");
        assert!(
            (seen - 0.5).abs() < 1e-9,
            "PostPhase plugin should see arbitrated steering 0.5, got {seen}"
        );
    }

    #[test]
    fn two_post_phase_plugins_both_see_arbitrated_steering() {
        let mut mgr = new_test_manager();
        inject(&mut mgr, Box::new(SteeringWriterPlugin { val: 0.75 }));
        let (cap_a, cell_a) = SteeringCapturePlugin::new("capture-a");
        let (cap_b, cell_b) = SteeringCapturePlugin::new("capture-b");
        inject(&mut mgr, Box::new(cap_a));
        inject(&mut mgr, Box::new(cap_b));

        let mut out = ControlOutput::default();
        mgr.tick_all(None, &mut out, 0.02);

        let a = cell_a.lock().unwrap().expect("capture-a must be called");
        let b = cell_b.lock().unwrap().expect("capture-b must be called");
        assert!((a - 0.75).abs() < 1e-9, "capture-a saw {a}");
        assert!((b - 0.75).abs() < 1e-9, "capture-b saw {b}");
    }

    #[test]
    fn no_post_phase_plugins_arbitrate_still_runs() {
        let mut mgr = new_test_manager();
        inject(&mut mgr, Box::new(SteeringWriterPlugin { val: 0.3 }));

        let mut out = ControlOutput::default();
        mgr.tick_all(None, &mut out, 0.02);

        assert!(
            (out.steering - 0.3).abs() < 1e-9,
            "arbitrated output should be 0.3, got {}",
            out.steering
        );
    }
}

#[cfg(test)]
mod on_load_skip_tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    struct OnLoadCounterPlugin {
        name: String,
        on_load_count: Arc<AtomicUsize>,
    }

    impl OnLoadCounterPlugin {
        fn new(name: &str) -> Self {
            Self {
                name: name.into(),
                on_load_count: Arc::new(AtomicUsize::new(0)),
            }
        }
    }

    impl Plugin for OnLoadCounterPlugin {
        fn name(&self) -> &str {
            &self.name
        }
        fn version(&self) -> &str {
            "0.0.0"
        }
        fn settings_schema(&self) -> &str {
            "{}"
        }
        fn on_load(&mut self, _ctx: &PluginContext) {
            self.on_load_count.fetch_add(1, Ordering::SeqCst);
        }
        fn on_unload(&mut self) {}
        fn tick(
            &mut self,
            _telemetry: Option<&Telemetry>,
            _output: &mut ControlOutput,
            _ctx: &PluginContext,
        ) {
        }
        fn tick_request(
            &mut self,
            _t: Option<&Telemetry>,
            _ctx: &PluginContext,
        ) -> Option<ControlRequest> {
            None
        }
        fn default_phase(&self) -> TickPhase {
            TickPhase::PhaseC
        }
    }

    #[test]
    fn disabled_plugin_skips_on_load_at_inject() {
        let mut mgr =
            PluginManager::new(PathBuf::from("./does-not-exist-test-dir"), HashMap::new());
        let plugin = OnLoadCounterPlugin::new("probe-disabled");
        let count = Arc::clone(&plugin.on_load_count);
        super::frame_store_tests::inject_disabled(&mut mgr, Box::new(plugin));
        assert_eq!(count.load(Ordering::SeqCst), 0);
        assert!(!mgr.plugins[0].initialized);
    }

    #[test]
    fn disabled_plugin_does_not_tick() {
        let mut mgr =
            PluginManager::new(PathBuf::from("./does-not-exist-test-dir"), HashMap::new());
        let plugin = OnLoadCounterPlugin::new("probe-disabled");
        let count = Arc::clone(&plugin.on_load_count);
        super::frame_store_tests::inject_disabled(&mut mgr, Box::new(plugin));
        let mut out = ControlOutput::default();
        mgr.tick_all(None, &mut out, 0.02);
        assert_eq!(count.load(Ordering::SeqCst), 0);
    }

    #[test]
    fn set_enabled_triggers_on_load() {
        let mut mgr =
            PluginManager::new(PathBuf::from("./does-not-exist-test-dir"), HashMap::new());
        let plugin = OnLoadCounterPlugin::new("probe-toggle");
        let count = Arc::clone(&plugin.on_load_count);
        super::frame_store_tests::inject_disabled(&mut mgr, Box::new(plugin));
        assert!(mgr.set_enabled("probe-toggle", true));
        assert_eq!(count.load(Ordering::SeqCst), 1);
        assert!(mgr.plugins[0].initialized);
        assert!(mgr.is_plugin_enabled("probe-toggle"));
    }
}
