//! Sign-Vision plugin — visual traffic-sign recognition.
//!
//! This plugin is the **fallback** for `sign-reader`. It only activates when:
//! - `sign.source` is NOT "map" (i.e. sign-reader found nothing) **and**
//! - The host wired a `SharedFrameStore` into [`PluginContext`] **and**
//! - A model file exists at `models/tsr_yolo.onnx` (when ONNX inference is wired up).
//!
//! ## Architecture (Phase 6.5c.2 Step 4)
//!
//! ```text
//! vision-frame-source (producer)
//!     │
//!     ▼   ctx.frame_store().set("camera.front", Arc<SharedFrame>)
//! SharedFrameStore
//!     │
//!     ▼   ctx.frame_store().get("camera.front")
//! sign-vision (this plugin)
//!     │  lazy JPEG → RGB8 via SharedFrame::get_or_init_rgb8
//!     ▼
//! ONNX inference (Phase 6.5e, stub today)
//!     │
//!     ▼
//! sign.speed_limit_kmh + sign.source = "vision"
//! ```
//!
//! ## Conflict resolution
//!
//! - `sign-reader` (map) has priority: when `sign.source == "map"`, this
//!   plugin skips inference.
//! - Vision overrides only when its detection confidence exceeds
//!   [`MIN_CONFIDENCE`] (0.7).
//! - When `frame_store` is absent the plugin runs in **map-only mode**:
//!   it is a no-op and never sets `sign.source = "vision"`.
//!
//! ## Modes
//!
//! - **map-only**: `ctx.frame_store()` is `None` *or* the daemon never
//!   publishes `camera.front`. Plugin is dormant; `sign.detection_active`
//!   is `"false"`.
//! - **vision**: `camera.front` is published and not stale. Plugin
//!   processes each new frame id once; `sign.detection_active = "true"`.

use std::path::PathBuf;
use std::sync::Arc;
use std::time::Instant;

use truckpilot_plugin_api::{
    ControlOutput, Plugin, PluginContext, SharedFrame, Telemetry, TickPhase,
};

/// Default path to the ONNX model.
const DEFAULT_MODEL_PATH: &str = "models/tsr_yolo.onnx";

/// Default inference interval in ticks (PhaseB 10 Hz → every tick = 10 Hz).
/// Producer publishes at 10 Hz so we don't need to throttle further; an
/// explicit knob lets operators dial it down on slow hardware.
const DEFAULT_INFERENCE_INTERVAL: u32 = 1;

/// Minimum confidence threshold for a vision detection to override map data.
/// Phase 6.5c.2 Step 4 raises this from 0.6 → 0.7.
pub const MIN_CONFIDENCE: f32 = 0.7;

/// SharedFrameStore key the plugin subscribes to.
const FRAME_KEY: &str = "camera.front";

// ---------------------------------------------------------------------------
// Detection result
// ---------------------------------------------------------------------------

/// A single detected traffic sign.
#[derive(Debug, Clone)]
pub struct Detection {
    /// Detected class label (e.g. "speed_limit_80").
    pub label: String,
    /// Confidence score in [0, 1].
    pub confidence: f32,
    /// Parsed numeric value (e.g. 80.0 for a speed-limit-80 sign).
    pub value: Option<f32>,
}

impl Detection {
    /// Parse a speed limit value from a label like "speed_limit_80".
    fn parse_speed_limit(label: &str) -> Option<f32> {
        label
            .strip_prefix("speed_limit_")
            .and_then(|s| s.parse::<f32>().ok())
    }
}

// ---------------------------------------------------------------------------
// JPEG decode helper
// ---------------------------------------------------------------------------

/// Decode JPEG bytes into a tightly-packed RGB8 buffer (3 bytes/pixel,
/// row-major, no row padding). Returns the buffer plus `(width, height)`.
pub fn decode_jpeg(bytes: &[u8]) -> Result<(Vec<u8>, u32, u32), image::ImageError> {
    let img = image::load_from_memory_with_format(bytes, image::ImageFormat::Jpeg)?;
    let rgb = img.to_rgb8();
    let (w, h) = (rgb.width(), rgb.height());
    Ok((rgb.into_raw(), w, h))
}

// ---------------------------------------------------------------------------
// Inference (Phase 6.5e stub — returns empty)
// ---------------------------------------------------------------------------

/// Run inference on a decoded RGB8 frame. **Stub** until Phase 6.5e wires
/// up the real ONNX session — returns an empty detection list so the
/// rest of the pipeline can be exercised end-to-end.
fn run_inference(_rgb: &[u8], _width: u32, _height: u32) -> Vec<Detection> {
    Vec::new()
}

// ---------------------------------------------------------------------------
// Plugin
// ---------------------------------------------------------------------------

/// Mode the plugin chose at `on_load` based on whether a
/// [`SharedFrameStore`] is wired in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    /// `frame_store` is `None` — plugin is dormant.
    MapOnly,
    /// `frame_store` is `Some(_)` — process frames as they arrive.
    Vision,
}

pub struct SignVisionPlugin {
    model_path: PathBuf,
    inference_interval: u32,
    mode: Mode,
    /// `id` of the last [`SharedFrame`] we ran inference on. Used to
    /// avoid re-processing the same frame across consecutive ticks
    /// when the producer is slower than the scheduler.
    last_processed_frame_id: Option<u64>,
    /// Reserved for future ONNX session result caching.
    #[allow(dead_code)]
    last_detection: Option<Detection>,
    #[allow(dead_code)]
    last_inference: Instant,
    /// Whether an ONNX session was successfully loaded. Today this is
    /// only flipped when `cfg(feature = "onnx")` is on; the stub path
    /// keeps it `false`.
    model_available: bool,
}

impl Default for SignVisionPlugin {
    fn default() -> Self {
        Self {
            model_path: PathBuf::from(DEFAULT_MODEL_PATH),
            inference_interval: DEFAULT_INFERENCE_INTERVAL,
            mode: Mode::MapOnly,
            last_processed_frame_id: None,
            last_detection: None,
            last_inference: Instant::now(),
            model_available: false,
        }
    }
}

impl SignVisionPlugin {
    /// Pull a fresh frame from `ctx.frame_store` if one is available and
    /// has an id we haven't processed yet. Returns `None` when:
    /// - the host did not wire a frame store
    /// - no frame has been published under `camera.front`
    /// - the published frame has the same id we last processed
    /// - the published frame is flagged stale by vision-frame-source
    fn next_frame(&self, ctx: &PluginContext) -> Option<Arc<SharedFrame>> {
        if ctx.blackboard.get("vision.frame.stale").as_deref() == Some("true") {
            return None;
        }
        let store = ctx.frame_store()?;
        let frame = store.get(FRAME_KEY)?;
        if Some(frame.id) == self.last_processed_frame_id {
            return None;
        }
        Some(frame)
    }
}

impl Plugin for SignVisionPlugin {
    fn name(&self) -> &str {
        "sign-vision"
    }
    fn version(&self) -> &str {
        "0.2.0"
    }
    fn settings_schema(&self) -> &str {
        r#"{
  "type": "object",
  "properties": {
    "model_path": {
      "type": "string",
      "description": "Path to the ONNX model file (tsr_yolo.onnx)."
    },
    "inference_interval": {
      "type": "integer",
      "minimum": 1,
      "maximum": 250,
      "description": "Run inference every N ticks. Default 1 (every PhaseB tick = 10 Hz)."
    },
    "min_confidence": {
      "type": "number",
      "minimum": 0.1,
      "maximum": 1.0,
      "description": "Minimum detection confidence threshold (default 0.7)."
    }
  }
}"#
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        // Allow overriding model path and interval via blackboard.
        if let Some(p) = ctx.blackboard.get("sign_vision.model_path") {
            self.model_path = PathBuf::from(p);
        }
        if let Some(iv) = ctx.blackboard.get_f64("sign_vision.inference_interval") {
            self.inference_interval = (iv as u32).clamp(1, 250);
        }

        self.mode = if ctx.frame_store().is_some() {
            Mode::Vision
        } else {
            Mode::MapOnly
        };

        // Surface the chosen mode so UI / stats-logger can show it.
        ctx.blackboard.set(
            "sign.detection_active",
            if self.mode == Mode::Vision {
                "true"
            } else {
                "false"
            },
        );

        if self.model_path.exists() {
            #[cfg(feature = "onnx")]
            {
                // TODO (Phase 6.5e): load ONNX session here.
                self.model_available = true;
                tracing::info!(
                    target: "truckpilot_plugin_sign_vision",
                    "ONNX model loaded from {:?}",
                    self.model_path
                );
            }
            #[cfg(not(feature = "onnx"))]
            {
                tracing::warn!(
                    target: "truckpilot_plugin_sign_vision",
                    "model found at {:?} but onnx feature not compiled in",
                    self.model_path
                );
            }
        } else {
            tracing::info!(
                target: "truckpilot_plugin_sign_vision",
                "model not found at {:?} — visual recognition disabled (stub inference)",
                self.model_path
            );
        }

        tracing::info!(
            target: "truckpilot_plugin_sign_vision",
            "loaded — mode={:?} model_available={} inference_interval={}",
            self.mode, self.model_available, self.inference_interval
        );
    }

    fn on_unload(&mut self) {
        tracing::info!(
            target: "truckpilot_plugin_sign_vision",
            "unloaded"
        );
    }

    fn default_phase(&self) -> TickPhase {
        TickPhase::PhaseB
    }

    fn tick(
        &mut self,
        _telemetry: Option<&Telemetry>,
        _output: &mut ControlOutput,
        ctx: &PluginContext,
    ) {
        // Map data takes priority — skip vision if sign-reader already
        // claimed this tick.
        if ctx.blackboard.get("sign.source").as_deref() == Some("map") {
            return;
        }

        if self.mode == Mode::MapOnly {
            return;
        }

        // Sub-rate throttle (the PhaseB scheduler may itself be slower
        // than the producer; this gives operators another knob).
        if !ctx
            .tick_count
            .is_multiple_of(self.inference_interval as u64)
        {
            return;
        }

        let Some(frame) = self.next_frame(ctx) else {
            return;
        };

        tracing::debug!(
            target: "truckpilot_plugin_sign_vision",
            "frame received id={} size={}x{} jpeg={}B",
            frame.id, frame.width, frame.height, frame.jpeg.len()
        );

        // Lazy decode: closure runs at most once per SharedFrame even
        // if multiple consumers ask. We pass through the Result so the
        // first attempt can recover next tick.
        let decode_attempt: Result<&Arc<Vec<u8>>, image::ImageError> =
            frame.get_or_try_init_rgb8(|| {
                let (rgb, w, h) = decode_jpeg(&frame.jpeg)?;
                debug_assert_eq!(
                    rgb.len() as u32,
                    w * h * 3,
                    "RGB8 buffer size must match decoded dimensions"
                );
                Ok(Arc::new(rgb))
            });

        let rgb = match decode_attempt {
            Ok(rgb) => rgb,
            Err(e) => {
                tracing::warn!(
                    target: "truckpilot_plugin_sign_vision",
                    "JPEG decode failed for frame id={}: {e}",
                    frame.id
                );
                // Mark the frame as processed so a permanently-corrupt
                // payload doesn't burn CPU on every tick.
                self.last_processed_frame_id = Some(frame.id);
                return;
            }
        };

        // Phase 6.5e will replace this with a real ONNX call.
        let detections = if self.model_available {
            run_inference(rgb, frame.width, frame.height)
        } else {
            Vec::new()
        };

        // Find the highest-confidence speed-limit detection above threshold.
        let best = detections
            .into_iter()
            .filter(|d| d.confidence >= MIN_CONFIDENCE)
            .filter_map(|d| Detection::parse_speed_limit(&d.label).map(|v| (d.confidence, v)))
            .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        match best {
            Some((conf, limit_kmh)) => {
                ctx.blackboard
                    .set("sign.speed_limit_kmh", limit_kmh.to_string());
                ctx.blackboard.set("sign.source", "vision");
                ctx.blackboard
                    .set("sign.vision.confidence", format!("{conf:.3}"));
                tracing::debug!(
                    target: "truckpilot_plugin_sign_vision",
                    "detected {limit_kmh:.0} km/h (conf={conf:.2})"
                );
            }
            None => {
                // No high-confidence detection this frame. If we were
                // the previous owner of `sign.source`, fall back to
                // map (i.e. clear our claim).
                if ctx.blackboard.get("sign.source").as_deref() == Some("vision") {
                    ctx.blackboard.remove("sign.speed_limit_kmh");
                    ctx.blackboard.remove("sign.source");
                    ctx.blackboard.remove("sign.vision.confidence");
                }
            }
        }

        self.last_processed_frame_id = Some(frame.id);
    }
}

truckpilot_plugin_api::export_plugin!(SignVisionPlugin);

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use truckpilot_plugin_api::{SharedBlackboard, SharedFrameStore};

    /// 1×1 white JPEG produced with `image` and embedded as bytes.
    /// Decodes to `[0xFF, 0xFF, 0xFF]` RGB8.
    fn one_px_white_jpeg() -> Vec<u8> {
        let img = image::RgbImage::from_pixel(1, 1, image::Rgb([255, 255, 255]));
        let mut buf = std::io::Cursor::new(Vec::new());
        img.write_to(&mut buf, image::ImageFormat::Jpeg)
            .expect("encode jpeg");
        buf.into_inner()
    }

    fn frame_with(id: u64, jpeg: Vec<u8>) -> Arc<SharedFrame> {
        Arc::new(SharedFrame::new(id, 0, 1, 1, Arc::new(jpeg)))
    }

    fn ctx_map_only() -> PluginContext {
        PluginContext::new("sign-vision", SharedBlackboard::new())
    }

    fn ctx_vision() -> (PluginContext, Arc<SharedFrameStore>) {
        let store = Arc::new(SharedFrameStore::new());
        let ctx = PluginContext::new("sign-vision", SharedBlackboard::new())
            .with_frame_store(store.clone());
        (ctx, store)
    }

    // ---- regression: legacy behaviour ----------------------------------

    #[test]
    fn parse_speed_limit_label() {
        assert_eq!(Detection::parse_speed_limit("speed_limit_80"), Some(80.0));
        assert_eq!(Detection::parse_speed_limit("speed_limit_120"), Some(120.0));
        assert_eq!(Detection::parse_speed_limit("stop"), None);
        assert_eq!(Detection::parse_speed_limit("speed_limit_abc"), None);
    }

    #[test]
    fn default_plugin_model_not_available() {
        let p = SignVisionPlugin::default();
        assert!(!p.model_available);
    }

    #[test]
    fn inference_interval_respected() {
        let p = SignVisionPlugin {
            inference_interval: 5,
            ..Default::default()
        };
        for i in 1..=4u64 {
            assert!(
                !i.is_multiple_of(p.inference_interval as u64),
                "tick {i} should not trigger"
            );
        }
        assert!(5u64.is_multiple_of(p.inference_interval as u64));
    }

    #[test]
    fn missing_model_does_not_panic() {
        let mut p = SignVisionPlugin::default();
        let ctx = ctx_map_only();
        p.on_load(&ctx); // model file absent → warn, no panic
        assert!(!p.model_available);
    }

    // ---- Step 4: mode selection ---------------------------------------

    #[test]
    fn on_load_chooses_map_only_without_frame_store() {
        let mut p = SignVisionPlugin::default();
        let ctx = ctx_map_only();
        p.on_load(&ctx);
        assert_eq!(p.mode, Mode::MapOnly);
        assert_eq!(
            ctx.blackboard.get("sign.detection_active").as_deref(),
            Some("false")
        );
    }

    #[test]
    fn on_load_chooses_vision_with_frame_store() {
        let mut p = SignVisionPlugin::default();
        let (ctx, _store) = ctx_vision();
        p.on_load(&ctx);
        assert_eq!(p.mode, Mode::Vision);
        assert_eq!(
            ctx.blackboard.get("sign.detection_active").as_deref(),
            Some("true")
        );
    }

    // ---- Step 4: tick behaviour ---------------------------------------

    #[test]
    fn tick_is_noop_in_map_only_mode() {
        let mut p = SignVisionPlugin::default();
        let ctx = ctx_map_only();
        p.on_load(&ctx);
        let mut out = ControlOutput::default();
        for _ in 0..5 {
            p.tick(None, &mut out, &ctx);
        }
        assert_eq!(p.last_processed_frame_id, None);
        assert!(ctx.blackboard.get("sign.source").is_none());
    }

    #[test]
    fn tick_skips_when_sign_source_is_map() {
        let mut p = SignVisionPlugin::default();
        let (ctx, store) = ctx_vision();
        p.on_load(&ctx);
        store.set(FRAME_KEY, frame_with(7, one_px_white_jpeg()));
        ctx.blackboard.set("sign.source", "map");

        let mut out = ControlOutput::default();
        p.tick(None, &mut out, &ctx);

        assert_eq!(p.last_processed_frame_id, None);
        assert_eq!(ctx.blackboard.get("sign.source").as_deref(), Some("map"));
    }

    #[test]
    fn tick_processes_new_frame_and_decodes_lazily() {
        let mut p = SignVisionPlugin::default();
        let (ctx, store) = ctx_vision();
        p.on_load(&ctx);
        let frame = frame_with(42, one_px_white_jpeg());
        store.set(FRAME_KEY, Arc::clone(&frame));
        assert!(frame.decoded_rgb8().is_none(), "not decoded yet");

        let mut out = ControlOutput::default();
        p.tick(None, &mut out, &ctx);

        assert_eq!(p.last_processed_frame_id, Some(42));
        let rgb = frame.decoded_rgb8().expect("decode populated OnceLock");
        assert_eq!(rgb.as_slice(), &[255, 255, 255]);
    }

    #[test]
    fn tick_does_not_reprocess_same_frame_id() {
        let mut p = SignVisionPlugin::default();
        let (ctx, store) = ctx_vision();
        p.on_load(&ctx);
        let frame = frame_with(7, one_px_white_jpeg());
        store.set(FRAME_KEY, Arc::clone(&frame));

        let mut out = ControlOutput::default();
        p.tick(None, &mut out, &ctx);
        // Inject a second SharedFrame with the same id; OnceLock would be
        // a fresh slot so we'd see decode happen again if the plugin
        // didn't gate on `last_processed_frame_id`.
        let dup = frame_with(7, one_px_white_jpeg());
        store.set(FRAME_KEY, Arc::clone(&dup));
        p.tick(None, &mut out, &ctx);

        assert!(dup.decoded_rgb8().is_none(), "second frame must not decode");
    }

    #[test]
    fn tick_processes_each_new_id_once() {
        let mut p = SignVisionPlugin::default();
        let (ctx, store) = ctx_vision();
        p.on_load(&ctx);
        let mut out = ControlOutput::default();
        for id in 1..=4u64 {
            store.set(FRAME_KEY, frame_with(id, one_px_white_jpeg()));
            p.tick(None, &mut out, &ctx);
            assert_eq!(p.last_processed_frame_id, Some(id));
        }
    }

    #[test]
    fn tick_skips_stale_frame() {
        let mut p = SignVisionPlugin::default();
        let (ctx, store) = ctx_vision();
        p.on_load(&ctx);
        store.set(FRAME_KEY, frame_with(99, one_px_white_jpeg()));
        ctx.blackboard.set("vision.frame.stale", "true");

        let mut out = ControlOutput::default();
        p.tick(None, &mut out, &ctx);

        assert_eq!(p.last_processed_frame_id, None);
    }

    #[test]
    fn tick_handles_corrupt_jpeg_without_panic() {
        let mut p = SignVisionPlugin::default();
        let (ctx, store) = ctx_vision();
        p.on_load(&ctx);
        // 16 random bytes that are not a valid JPEG.
        store.set(FRAME_KEY, frame_with(11, vec![0u8; 16]));

        let mut out = ControlOutput::default();
        p.tick(None, &mut out, &ctx);

        // Plugin must mark the frame processed so it doesn't decode-loop.
        assert_eq!(p.last_processed_frame_id, Some(11));
        // No vision claim on the blackboard.
        assert!(ctx.blackboard.get("sign.source").is_none());
    }

    // ---- decode_jpeg ---------------------------------------------------

    #[test]
    fn decode_jpeg_returns_rgb8_of_expected_size() {
        let (rgb, w, h) = decode_jpeg(&one_px_white_jpeg()).expect("decode");
        assert_eq!((w, h), (1, 1));
        assert_eq!(rgb, vec![255, 255, 255]);
    }

    #[test]
    fn decode_jpeg_rejects_non_jpeg_bytes() {
        let err = decode_jpeg(b"not a jpeg").unwrap_err();
        let _ = format!("{err}"); // make sure it Displays
    }
}
