//! Plugin manager — handles loading, hot-reloading, and ticking plugins.
//!
//! **Hot-Reload is currently DISABLED** (see plugin-api docs for ABI reasons).
//! The notify watcher is present but reloads are no-ops until ABI stability is restored.
//!
//! Race-condition-safe: reloads are queued and applied between tick() calls.

use std::mem::ManuallyDrop;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use libloading::{Library, Symbol};
use notify::{RecommendedWatcher, RecursiveMode, Watcher};
use tracing::{error, info, warn};
use truckpilot_plugin_api::{
    ControlOutput, ControlRequest, Plugin, PluginContext, SharedBlackboard, Telemetry,
};

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
    _watcher: Option<RecommendedWatcher>,
}

impl PluginManager {
    pub fn new(plugin_dir: PathBuf) -> Self {
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
            _watcher: watcher,
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
        match unsafe { load_plugin_from_path(path, &self.blackboard) } {
            Ok(loaded) => {
                info!("Loaded plugin: {} v{}", loaded.name, loaded.version);
                self.plugins.push(loaded);
            }
            Err(e) => {
                error!("Failed to load plugin {:?}: {}", path, e);
            }
        }
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
            old.plugin.on_unload();
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
        // Legacy bucket: plugins still on the old `tick(&mut output)`
        // API write here. Reset every tick so stale values don't stick.
        let mut legacy = ControlOutput::default();
        let mut requests: Vec<ControlRequest> = Vec::new();

        // vjoy-output must observe the FINAL arbitrated output, not
        // intermediate legacy values. Skip it in the main loop and tick
        // it once after arbitration with `*output`.
        let vjoy_idx: Option<usize> = self
            .plugins
            .iter()
            .position(|p| p.enabled && p.name == "vjoy-output");

        for (i, p) in self.plugins.iter_mut().enumerate().filter(|(_, p)| p.enabled) {
            if Some(i) == vjoy_idx {
                continue;
            }
            let ctx = PluginContext::new(p.name.clone(), self.blackboard.clone()).with_dt(dt_s);

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
                }
            }
        }

        *output = arbitrate(legacy, &requests);

        if let Some(idx) = vjoy_idx {
            let p = &mut self.plugins[idx];
            if p.enabled {
                let ctx = PluginContext::new(p.name.clone(), self.blackboard.clone()).with_dt(dt_s);
                let tick_result = catch_unwind(AssertUnwindSafe(|| {
                    p.plugin.tick(telemetry, output, &ctx);
                }));
                if let Err(panic) = tick_result {
                    log_plugin_panic(&p.name, "tick", panic);
                    p.enabled = false;
                }
            }
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
        if let Some(p) = self.plugins.iter_mut().find(|p| p.name == name) {
            p.enabled = enabled;
            info!(
                "Plugin {} {}",
                name,
                if enabled { "enabled" } else { "disabled" }
            );
            true
        } else {
            false
        }
    }

    #[allow(dead_code)]
    pub fn unload_all(&mut self) {
        for p in self.plugins.iter_mut() {
            p.plugin.on_unload();
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

unsafe fn load_plugin_from_path(
    path: &Path,
    blackboard: &SharedBlackboard,
) -> Result<LoadedPlugin, String> {
    let lib = Library::new(path).map_err(|e| format!("library load: {e}"))?;

    let create: Symbol<CreateFn> = lib
        .get(b"create_plugin")
        .map_err(|e| format!("missing create_plugin symbol: {e}"))?;

    // Look up `destroy_plugin` *before* calling `create_plugin` so a
    // mis-built plugin (one without the matching destructor) fails
    // cleanly without leaking the partially-constructed instance.
    let destroy: Symbol<DestroyFn> = lib.get(b"destroy_plugin").map_err(|e| {
        format!(
            "missing destroy_plugin symbol: {e} \
             (plugin must use truckpilot_plugin_api::export_plugin!)"
        )
    })?;
    // Detach the function pointer from the Symbol guard. Validity is
    // bound to `lib` lifetime, which `LoadedPlugin` holds in `_lib`.
    let destroy_fn: DestroyFn = *destroy;

    let plugin_ptr = create();
    if plugin_ptr.is_null() {
        return Err("create_plugin returned null".into());
    }

    // The Box here is only used as a typed handle for trait dispatch
    // — its destructor must NOT run on the host side. `LoadedPlugin`
    // wraps it in `ManuallyDrop` and frees via `destroy_fn`.
    let mut plugin = Box::from_raw(plugin_ptr);

    let name = plugin.name().to_string();
    let version = plugin.version().to_string();

    let ctx = PluginContext::new(name.clone(), blackboard.clone());
    plugin.on_load(&ctx);

    Ok(LoadedPlugin {
        name,
        version,
        path: path.to_path_buf(),
        enabled: true,
        plugin: ManuallyDrop::new(plugin),
        destroy_fn,
        _lib: lib,
    })
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
