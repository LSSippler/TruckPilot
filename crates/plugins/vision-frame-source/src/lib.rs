//! TruckPilot Phase 6.5c.2 — vision-frame-source plugin.
//!
//! Reads camera frames from a Windows named shared-memory region
//! (`Local\TruckPilotFrame` by default) populated by an external
//! producer (Phase 6.5c.1 Python capture process), wraps each new
//! committed frame in a [`SharedFrame`], and publishes it to the
//! [`SharedFrameStore`] under the key `camera.front`. Lightweight
//! metadata (frame id, timestamp, dimensions, JPEG size, health flags)
//! is mirrored to the [`SharedBlackboard`] so other plugins, the UI,
//! and `stats-logger` can poll cheaply.
//!
//! The plugin never decodes JPEGs itself — decode is lazy on the
//! consumer side via [`SharedFrame::get_or_init_rgb8`]. This keeps
//! the producer-side tick cheap (~10 µs to memcpy + Arc-wrap) and
//! lets multiple consumers share one decode.

use std::sync::Arc;

use truckpilot_plugin_api::{
    ctx_error, ctx_info, ctx_warn, ControlOutput, Plugin, PluginContext, SharedFrame, Telemetry,
};

pub mod shm_reader;

use shm_reader::{map_shm, read_frame, ReadOutcome, DEFAULT_BUFFER_BYTES, DEFAULT_SHM_NAME};

const DEFAULT_STALE_AFTER_MS: u64 = 2000;
const DEFAULT_FRAME_KEY: &str = "camera.front";
/// Re-map the SHM every N ticks while the producer is absent.
const REMAP_INTERVAL_TICKS: u64 = 10;
/// Consecutive `read_frame` misses before the plugin flips to unhealthy.
const MISS_THRESHOLD: u32 = 50;
/// Apparent age above which we assume a clock-source mismatch (producer
/// writing monotonic instead of UNIX-epoch micros). Frames in this case
/// are trusted, not flagged stale — otherwise every frame would look
/// ~1.78e15 µs old.
const CLOCK_MISMATCH_THRESHOLD_US: u64 = 60 * 60 * 1_000_000; // 1 hour

/// Plugin settings, populated from the daemon's `settings_schema` JSON.
/// Defaults match `Default::default()`.
#[derive(Debug, Clone)]
pub struct Settings {
    pub shm_name: String,
    pub buffer_bytes: usize,
    pub stale_after_ms: u64,
    /// SharedFrameStore key under which frames are published. Allows
    /// future multi-camera support without colliding on `camera.front`.
    pub frame_key: String,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            shm_name: DEFAULT_SHM_NAME.to_string(),
            buffer_bytes: DEFAULT_BUFFER_BYTES,
            stale_after_ms: DEFAULT_STALE_AFTER_MS,
            frame_key: DEFAULT_FRAME_KEY.to_string(),
        }
    }
}

#[derive(Default)]
pub struct VisionFrameSource {
    settings: Settings,
    shm_buf: Option<&'static [u8]>,
    /// `header.frame_id` of the last frame we published. Used to
    /// detect "same frame still in SHM" → `vision.frame.ready = false`.
    last_published_id: u64,
    /// Running count of consecutive ticks that produced no fresh frame.
    consecutive_misses: u32,
    /// Tick counter local to the plugin (rolls over at u64::MAX, fine).
    /// Used to re-attempt `map_shm` every `REMAP_INTERVAL_TICKS` while
    /// the producer is absent.
    tick_counter: u64,
    /// `true` once a frame has been successfully published in this
    /// session. Drives the `vision.source.healthy` blackboard key
    /// in combination with `consecutive_misses`.
    has_published: bool,
    /// Set the first time we detect a clock-source mismatch with the
    /// producer (apparent age > 1 h). Suppresses repeated warnings.
    clock_mismatch_warned: bool,
}

impl VisionFrameSource {
    /// Construct with explicit settings (used by tests).
    pub fn with_settings(settings: Settings) -> Self {
        Self {
            settings,
            ..Self::default()
        }
    }

    /// Test-only: inject an already-mapped buffer slice. Lets unit
    /// tests drive `tick` against a Vec-backed mock without going
    /// through Windows SHM.
    #[doc(hidden)]
    pub fn inject_buffer(&mut self, buf: &'static [u8]) {
        self.shm_buf = Some(buf);
    }

    fn try_map(&mut self, ctx: &PluginContext) {
        match map_shm(&self.settings.shm_name, self.settings.buffer_bytes) {
            Ok(buf) => {
                ctx_info!(
                    ctx,
                    target: "truckpilot_plugin_vision_frame_source",
                    "mapped SHM region '{}' ({} bytes)",
                    self.settings.shm_name,
                    self.settings.buffer_bytes
                );
                self.shm_buf = Some(buf);
                ctx.blackboard.remove("vision.source.last_error");
            }
            Err(e) => {
                ctx_warn!(
                    ctx,
                    target: "truckpilot_plugin_vision_frame_source",
                    "SHM not available: {e}"
                );
                ctx.blackboard.set("vision.source.last_error", e);
            }
        }
    }

    fn publish_unhealthy(&self, ctx: &PluginContext) {
        ctx.blackboard.set("vision.source.healthy", "false");
        ctx.blackboard.set("vision.frame.ready", "false");
    }

    fn now_us() -> u64 {
        use std::time::{SystemTime, UNIX_EPOCH};
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_micros() as u64)
            .unwrap_or(0)
    }
}

impl Plugin for VisionFrameSource {
    fn name(&self) -> &str {
        "vision-frame-source"
    }

    fn version(&self) -> &str {
        "0.1.0"
    }

    fn settings_schema(&self) -> &str {
        r#"{
  "type": "object",
  "properties": {
    "shm_name":       { "type": "string",  "default": "TruckPilotFrame" },
    "buffer_bytes":   { "type": "integer", "default": 2097216, "minimum": 65 },
    "stale_after_ms": { "type": "integer", "default": 500, "minimum": 1 },
    "frame_key":      { "type": "string",  "default": "camera.front" }
  }
}"#
    }

    fn default_phase(&self) -> truckpilot_plugin_api::TickPhase {
        truckpilot_plugin_api::TickPhase::PhaseB
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        ctx_info!(
            ctx,
            target: "truckpilot_plugin_vision_frame_source",
            "loaded — shm='{}' buffer_bytes={} stale_after_ms={} frame_key='{}'",
            self.settings.shm_name,
            self.settings.buffer_bytes,
            self.settings.stale_after_ms,
            self.settings.frame_key
        );

        // Initialise blackboard keys so consumers see deterministic
        // state even before the first tick lands.
        ctx.blackboard.set("vision.source.healthy", "false");
        ctx.blackboard.set("vision.frame.ready", "false");
        ctx.blackboard.set("vision.frame.id", "0");
        ctx.blackboard.set("vision.frame.timestamp_us", "0");
        ctx.blackboard.set("vision.frame.jpeg_size", "0");
        ctx.blackboard.set("vision.frame.width", "0");
        ctx.blackboard.set("vision.frame.height", "0");
        ctx.blackboard.set("vision.frame.stale", "false");

        if ctx.frame_store().is_none() {
            ctx_warn!(
                ctx,
                target: "truckpilot_plugin_vision_frame_source",
                "PluginContext has no SharedFrameStore — frames will be dropped. \
                 Daemon must attach one via PluginContext::with_frame_store()."
            );
        }

        self.try_map(ctx);
    }

    fn on_unload(&mut self) {
        // SHM mapping is intentionally leaked (see `shm_reader::map_shm`
        // SAFETY note). Nothing to release here; the daemon's process
        // exit unmaps. Just clear the local handle so a future on_load
        // re-maps cleanly.
        self.shm_buf = None;
        self.last_published_id = 0;
        self.consecutive_misses = 0;
        self.tick_counter = 0;
        self.has_published = false;
        tracing::info!(
            target: "truckpilot_plugin_vision_frame_source",
            "unloaded"
        );
    }

    fn tick(
        &mut self,
        _telemetry: Option<&Telemetry>,
        _output: &mut ControlOutput,
        ctx: &PluginContext,
    ) {
        self.tick_counter = self.tick_counter.wrapping_add(1);

        // Producer-down recovery: retry the mapping periodically.
        if self.shm_buf.is_none() {
            if self.tick_counter.is_multiple_of(REMAP_INTERVAL_TICKS) {
                self.try_map(ctx);
            }
            if self.shm_buf.is_none() {
                self.publish_unhealthy(ctx);
                return;
            }
        }

        let buf = match self.shm_buf {
            Some(b) => b,
            None => {
                self.publish_unhealthy(ctx);
                return;
            }
        };

        match read_frame(buf) {
            ReadOutcome::Frame { header, payload } => {
                self.consecutive_misses = 0;

                if header.frame_id == self.last_published_id {
                    // Producer hasn't advanced since our last publish.
                    ctx.blackboard.set("vision.frame.ready", "false");
                    // healthy still true — we have a recent frame, just no new one.
                    if self.has_published {
                        ctx.blackboard.set("vision.source.healthy", "true");
                    }
                    return;
                }

                let logical_id = header.frame_id / 2;
                let now = Self::now_us();
                let apparent_age_us = now.saturating_sub(header.timestamp_us);
                // Defensive: a producer using a monotonic clock instead
                // of UNIX-epoch microseconds yields apparent_age of order
                // 10^15 µs every frame. Detect that and trust the frame
                // rather than flagging everything stale.
                let stale = if apparent_age_us > CLOCK_MISMATCH_THRESHOLD_US
                    || header.timestamp_us > now
                {
                    if !self.clock_mismatch_warned {
                        ctx_warn!(
                            ctx,
                            target: "truckpilot_plugin_vision_frame_source",
                            "producer/consumer clock mismatch (apparent age {} µs, ts={} now={}); \
                             trusting frame timestamps. Update producer to UNIX-epoch micros.",
                            apparent_age_us,
                            header.timestamp_us,
                            now,
                        );
                        self.clock_mismatch_warned = true;
                    }
                    false
                } else {
                    apparent_age_us > self.settings.stale_after_ms.saturating_mul(1_000)
                };

                // Build SharedFrame and publish, if the host wired in a store.
                if let Some(store) = ctx.frame_store() {
                    let jpeg = Arc::new(payload.to_vec());
                    let frame = SharedFrame::new(
                        logical_id,
                        header.timestamp_us,
                        header.width,
                        header.height,
                        jpeg,
                    );
                    store.set(self.settings.frame_key.clone(), Arc::new(frame));
                }

                ctx.blackboard.set("vision.source.healthy", "true");
                ctx.blackboard.remove("vision.source.last_error");
                ctx.blackboard.set("vision.frame.ready", "true");
                ctx.blackboard
                    .set("vision.frame.id", logical_id.to_string());
                ctx.blackboard
                    .set("vision.frame.timestamp_us", header.timestamp_us.to_string());
                ctx.blackboard
                    .set("vision.frame.jpeg_size", header.jpeg_size.to_string());
                ctx.blackboard
                    .set("vision.frame.width", header.width.to_string());
                ctx.blackboard
                    .set("vision.frame.height", header.height.to_string());
                ctx.blackboard
                    .set("vision.frame.stale", if stale { "true" } else { "false" });

                self.last_published_id = header.frame_id;
                self.has_published = true;
            }
            ReadOutcome::SequenceLockExhausted => {
                self.consecutive_misses = self.consecutive_misses.saturating_add(1);
                ctx.blackboard.set("vision.frame.ready", "false");
                if self.consecutive_misses >= MISS_THRESHOLD {
                    ctx.blackboard.set("vision.source.healthy", "false");
                    ctx.blackboard
                        .set("vision.source.last_error", "sequence-lock exhausted");
                }
            }
            ReadOutcome::InvalidHeader => {
                ctx_error!(
                    ctx,
                    target: "truckpilot_plugin_vision_frame_source",
                    "invalid SHM header (magic/version mismatch or buffer too short)"
                );
                ctx.blackboard.set("vision.source.healthy", "false");
                ctx.blackboard.set("vision.frame.ready", "false");
                ctx.blackboard
                    .set("vision.source.last_error", "invalid header");
            }
            ReadOutcome::PayloadOverflow => {
                ctx_error!(
                    ctx,
                    target: "truckpilot_plugin_vision_frame_source",
                    "SHM payload overflow (jpeg_size exceeds buffer)"
                );
                ctx.blackboard.set("vision.source.healthy", "false");
                ctx.blackboard.set("vision.frame.ready", "false");
                ctx.blackboard
                    .set("vision.source.last_error", "payload overflow");
            }
        }
    }
}

truckpilot_plugin_api::export_plugin!(VisionFrameSource);
