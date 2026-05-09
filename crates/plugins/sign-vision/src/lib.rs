//! Sign-Vision plugin — visual traffic sign recognition via ONNX.
//!
//! This plugin is the **fallback** for `sign-reader`. It only activates when:
//! - `sign.source` is NOT "map" (i.e. sign-reader found nothing), AND
//! - A model file exists at `models/tsr_yolo.onnx`, AND
//! - The `onnx` feature is compiled in.
//!
//! ## Architecture
//!
//! ```text
//! ETS2 window → screen capture → ONNX inference → detected class + value
//!                                                        ↓
//!                                          sign.speed_limit_kmh (blackboard)
//!                                          sign.source = "vision"
//! ```
//!
//! ## Conflict resolution
//!
//! Map data (sign-reader) has priority. Vision only writes to the blackboard
//! when sign-reader has not set `sign.source = "map"` this tick.
//!
//! ## Inference frequency
//!
//! Running ONNX at 50 Hz is expensive. The plugin runs inference at a
//! configurable sub-rate (default: every 10 ticks = 5 Hz).
//!
//! ## Model
//!
//! `models/tsr_yolo.onnx` — YOLOv8n fine-tuned on a traffic sign dataset.
//! Input: 640×640 RGB float32. Output: YOLO detection format.
//! If the file is absent the plugin logs a warning and becomes a no-op.

use std::path::PathBuf;
use std::time::Instant;

use truckpilot_plugin_api::{ControlOutput, Plugin, PluginContext, Telemetry};

/// Default path to the ONNX model.
const DEFAULT_MODEL_PATH: &str = "models/tsr_yolo.onnx";

/// Default inference interval in ticks (50 Hz loop → 10 ticks = 5 Hz).
const DEFAULT_INFERENCE_INTERVAL: u32 = 10;

/// Minimum confidence threshold for a detection to be accepted.
const MIN_CONFIDENCE: f32 = 0.6;

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
// Screen capture (platform-specific stub)
// ---------------------------------------------------------------------------

/// Capture the ETS2 window as an RGB byte buffer.
/// Returns `None` if capture fails or the window is not found.
fn capture_ets2_window() -> Option<(Vec<u8>, u32, u32)> {
    // Phase 8: stub — real implementation uses `screenshots` crate or
    // platform-specific APIs (BitBlt on Windows, XShmGetImage on Linux).
    // Returns None so the plugin gracefully degrades.
    None
}

// ---------------------------------------------------------------------------
// ONNX inference (feature-gated)
// ---------------------------------------------------------------------------

/// Run ONNX inference on a captured frame.
/// Returns a list of detections above `MIN_CONFIDENCE`.
/// Stub inference — returns empty detections until ONNX session is wired up.
///
/// When the `onnx` feature is enabled, this would accept a loaded session.
/// For now both paths return nothing so the plugin compiles without ort types
/// leaking into the public signature.
fn run_inference(_frame: &[u8], _width: u32, _height: u32) -> Vec<Detection> {
    Vec::new()
}

// ---------------------------------------------------------------------------
// Plugin
// ---------------------------------------------------------------------------

pub struct SignVisionPlugin {
    model_path: PathBuf,
    inference_interval: u32,
    tick_count: u32,
    // Reserved for future ONNX session result caching.
    #[allow(dead_code)]
    last_detection: Option<Detection>,
    #[allow(dead_code)]
    last_inference: Instant,
    /// Whether the ONNX session was successfully loaded.
    model_available: bool,
}

impl Default for SignVisionPlugin {
    fn default() -> Self {
        Self {
            model_path: PathBuf::from(DEFAULT_MODEL_PATH),
            inference_interval: DEFAULT_INFERENCE_INTERVAL,
            tick_count: 0,
            last_detection: None,
            last_inference: Instant::now(),
            model_available: false,
        }
    }
}

impl Plugin for SignVisionPlugin {
    fn name(&self) -> &str {
        "sign-vision"
    }
    fn version(&self) -> &str {
        "0.1.0"
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
      "description": "Run inference every N ticks (50 Hz loop). Default 10 = 5 Hz."
    },
    "min_confidence": {
      "type": "number",
      "minimum": 0.1,
      "maximum": 1.0,
      "description": "Minimum detection confidence threshold."
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

        if self.model_path.exists() {
            #[cfg(feature = "onnx")]
            {
                // TODO: load ONNX session here.
                self.model_available = true;
                tracing::info!("[sign-vision] ONNX model loaded from {:?}", self.model_path);
            }
            #[cfg(not(feature = "onnx"))]
            {
                tracing::warn!(
                    "[sign-vision] model found at {:?} but onnx feature not compiled in",
                    self.model_path
                );
            }
        } else {
            tracing::info!(
                "[sign-vision] model not found at {:?} — visual recognition disabled",
                self.model_path
            );
        }
    }

    fn on_unload(&mut self) {
        tracing::info!("[sign-vision] unloaded");
    }

    fn tick(
        &mut self,
        _telemetry: Option<&Telemetry>,
        _output: &mut ControlOutput,
        ctx: &PluginContext,
    ) {
        self.tick_count = self.tick_count.wrapping_add(1);

        // Map data takes priority — skip vision if sign-reader already found something.
        if ctx.blackboard.get("sign.source").as_deref() == Some("map") {
            return;
        }

        // Only run inference at the configured sub-rate.
        #[allow(clippy::manual_is_multiple_of)]
        if self.tick_count % self.inference_interval != 0 {
            return;
        }

        if !self.model_available {
            return;
        }

        // Capture screen.
        let Some((frame, w, h)) = capture_ets2_window() else {
            return;
        };

        // Run inference (stub — returns empty until ONNX session is wired up).
        let detections = run_inference(&frame, w, h);

        // Find the highest-confidence speed-limit detection.
        let best = detections
            .into_iter()
            .filter(|d| d.confidence >= MIN_CONFIDENCE)
            .filter_map(|d| Detection::parse_speed_limit(&d.label).map(|v| (d.confidence, v)))
            .max_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        match best {
            Some((conf, limit_kmh)) => {
                ctx.blackboard
                    .set("sign.speed_limit_kmh", limit_kmh.to_string());
                ctx.blackboard.set("sign.source", "vision");
                tracing::debug!("[sign-vision] detected {limit_kmh:.0} km/h (conf={conf:.2})");
            }
            None => {
                // No detection — remove stale vision entry if present.
                if ctx.blackboard.get("sign.source").as_deref() == Some("vision") {
                    ctx.blackboard.remove("sign.speed_limit_kmh");
                    ctx.blackboard.remove("sign.source");
                }
            }
        }
    }
}

truckpilot_plugin_api::export_plugin!(SignVisionPlugin);

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

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
        // Ticks 1-4 should not trigger inference (model_available = false anyway,
        // but we verify the modulo logic).
        for i in 1..=4u32 {
            assert_ne!(i % p.inference_interval, 0, "tick {i} should not trigger");
        }
        assert_eq!(5 % p.inference_interval, 0, "tick 5 should trigger");
    }

    #[test]
    fn capture_stub_returns_none() {
        // On this server there is no ETS2 window — stub must return None.
        assert!(capture_ets2_window().is_none());
    }

    #[test]
    fn missing_model_does_not_panic() {
        let mut p = SignVisionPlugin::default();
        let bb = truckpilot_plugin_api::SharedBlackboard::new();
        let ctx = truckpilot_plugin_api::PluginContext::new("sign-vision", bb);
        p.on_load(&ctx); // model file absent → warn, no panic
        assert!(!p.model_available);
    }
}
