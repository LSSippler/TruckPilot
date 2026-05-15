//! Sign-Vision plugin — visual traffic-sign recognition.
//!
//! Reads JPEG frames from `SharedFrameStore` (published by `vision-frame-source`),
//! runs YOLOv8s-v2 inference via ONNX Runtime (DirectML on AMD RX 7800 XT,
//! CPU fallback), and writes speed-limit detections to the Blackboard.
//!
//! ## Architecture (Phase 6.5e)
//!
//! ```text
//! vision-frame-source (producer)
//!     │
//!     ▼   ctx.frame_store().set("camera.front", Arc<SharedFrame>)
//! SharedFrameStore
//!     │
//!     ▼   ctx.frame_store().get("camera.front")
//! sign-vision (this plugin)
//!     │  lazy JPEG → RGB8 via SharedFrame::get_or_try_init_rgb8
//!     │  letterbox 640×640 → NCHW f32 tensor
//!     │  ort::Session::run() [DirectML / CPU]
//!     │  YOLOv8s decode + NMS
//!     │  SpeedLimitSign → template-match → km/h
//!     ▼
//! sign.speed_limit_kmh + sign.source = "vision"
//! ```
//!
//! ## Model
//!
//! `models/truckpilot-yolov8s-v2/best.onnx` — 15 classes, mAP@0.5 = 0.798.
//! `SpeedLimitSign` is class index 11 (generic; km/h extracted via template matching).
//!
//! ## Conflict resolution
//!
//! `sign-reader` (map) has priority: when `sign.source == "map"`, this plugin
//! skips inference. Vision overrides only when confidence ≥ `MIN_CONFIDENCE`.
//!
//! ## Build features
//!
//! | Feature         | Effect                                              |
//! |-----------------|-----------------------------------------------------|
//! | `onnx-directml` | DirectML GPU (default, AMD RX 7800 XT)              |
//! | `onnx-cpu`      | CPU-only (slow, for CI/headless builds)             |
//! | (neither)       | Stub mode — pipeline active, inference returns `[]` |

mod postprocess;
mod preprocess;
mod speed_mapper;

use std::path::PathBuf;
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use truckpilot_plugin_api::{
    ctx_debug, ctx_info, ctx_trace, ctx_warn, ControlOutput, Plugin, PluginContext, SharedFrame,
    Telemetry, TickPhase,
};

use speed_mapper::SpeedMapper;

fn now_us() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const LOG_TARGET: &str = "truckpilot_plugin_sign_vision";

/// Default path to the ONNX model (Phase 6.5e v2 model). The exported
/// model is already FP16 inside, so the separate `_fp16.onnx` variant
/// is unnecessary; the async-inference worker (added in this phase)
/// removes the latency pressure that motivated quantization.
const DEFAULT_MODEL_PATH: &str = "models/truckpilot-yolov8s-v2/best.onnx";

/// Default inference interval in ticks (PhaseB 10 Hz → every tick).
const DEFAULT_INFERENCE_INTERVAL: u32 = 1;

/// Minimum confidence for a vision detection to override map data.
pub const MIN_CONFIDENCE: f32 = 0.7;

/// Default YOLO confidence threshold (below this, anchors are ignored).
const DEFAULT_CONF_THRESHOLD: f32 = 0.5;

/// Default NMS IoU threshold.
const DEFAULT_NMS_IOU: f32 = 0.45;

/// SharedFrameStore key subscribed by this plugin.
const FRAME_KEY: &str = "camera.front";

/// Number of classes in the v2 model (15 header classes, 6 trained).
const NUM_CLASSES: usize = 15;

/// Number of YOLO anchors in YOLOv8s output (80²+40²+20² = 8400).
const NUM_ANCHORS: usize = 8400;

/// Class index of `SpeedLimitSign` in dataset.yaml.
const SPEED_LIMIT_CLASS_ID: usize = 11;

/// Class names in dataset.yaml order (15 entries).
const CLASS_NAMES: [&str; 15] = [
    "Car",
    "Truck",
    "TruckTrailer",
    "Bus",
    "BrakeLightOn",
    "TurnSignalLeft",
    "TurnSignalRight",
    "TrafficLightRed",
    "TrafficLightYellow",
    "TrafficLightGreen",
    "StopSign",
    "SpeedLimitSign",
    "LaneSolid",
    "LaneDashed",
    "RoadEdge",
];

// ---------------------------------------------------------------------------
// Detection result
// ---------------------------------------------------------------------------

/// A single detected traffic sign.
#[derive(Debug, Clone)]
pub struct Detection {
    /// Detected class label (e.g. "speed_limit_80" or "Car").
    pub label: String,
    /// Confidence score in [0, 1].
    pub confidence: f32,
    /// Parsed numeric value — set for speed-limit signs (km/h).
    pub value: Option<f32>,
}

impl Detection {
    /// Parse a speed limit value from a label like `"speed_limit_80"`.
    fn parse_speed_limit(label: &str) -> Option<f32> {
        label
            .strip_prefix("speed_limit_")
            .and_then(|s| s.parse::<f32>().ok())
    }
}

// ---------------------------------------------------------------------------
// Async inference worker (only when an onnx feature is compiled in)
// ---------------------------------------------------------------------------

/// Stale-frame threshold inside the worker, in microseconds. Frames
/// submitted longer ago than this are dropped without running inference
/// — at 10+ Hz frame rate the data is already obsolete by the time we
/// got around to processing it.
const WORKER_STALE_FRAME_US: u64 = 200_000;

#[cfg(any(feature = "onnx-directml", feature = "onnx-cpu"))]
struct InferenceJob {
    frame_id: u64,
    rgb: Arc<Vec<u8>>,
    width: u32,
    height: u32,
    submit_us: u64,
}

#[cfg(any(feature = "onnx-directml", feature = "onnx-cpu"))]
struct InferenceResult {
    frame_id: u64,
    detections: Vec<Detection>,
    inference_ms: f32,
    skipped_stale: bool,
}

/// Channel-backed handle to the inference worker thread. Owned by
/// `SignVisionPlugin` while inference is active; dropping it closes
/// the job channel which lets the worker thread exit cleanly.
#[cfg(any(feature = "onnx-directml", feature = "onnx-cpu"))]
struct InferenceWorker {
    /// `Option` so `Drop` can `.take()` and drop it, closing the job
    /// channel and letting the worker exit cleanly before we join it.
    job_tx: Option<crossbeam_channel::Sender<InferenceJob>>,
    /// Receiver clone used by the *producer* side to drain a
    /// queued-but-not-yet-processed frame and replace it with a fresher
    /// one (drop-oldest backpressure). This is safe because `tick()` is
    /// the only producer; the worker thread holds its own clone.
    job_drain_rx: crossbeam_channel::Receiver<InferenceJob>,
    result_rx: crossbeam_channel::Receiver<InferenceResult>,
    handle: Option<JoinHandle<()>>,
}

#[cfg(any(feature = "onnx-directml", feature = "onnx-cpu"))]
impl InferenceWorker {
    /// Spawn a dedicated thread that owns `session` and processes
    /// frames from `job_rx`. Bounded(1) on both channels so a slow
    /// worker doesn't queue stale frames; the tick path uses
    /// drain-then-send to keep "newest frame wins".
    fn spawn(
        session: ort::session::Session,
        speed_mapper: SpeedMapper,
        conf_threshold: f32,
        nms_iou: f32,
    ) -> Self {
        let (job_tx, job_rx) = crossbeam_channel::bounded::<InferenceJob>(1);
        let job_drain_rx = job_rx.clone();
        let (result_tx, result_rx) = crossbeam_channel::bounded::<InferenceResult>(1);

        let handle = std::thread::Builder::new()
            .name("sign-vision-inference".into())
            .spawn(move || worker_loop(session, speed_mapper, conf_threshold, nms_iou, job_rx, result_tx))
            .expect("spawn sign-vision-inference thread");

        Self {
            job_tx: Some(job_tx),
            job_drain_rx,
            result_rx,
            handle: Some(handle),
        }
    }
}

#[cfg(any(feature = "onnx-directml", feature = "onnx-cpu"))]
impl Drop for InferenceWorker {
    fn drop(&mut self) {
        // Dropping the only Sender disconnects the channel; the worker
        // thread's recv() returns Err and the loop exits.
        drop(self.job_tx.take());
        if let Some(h) = self.handle.take() {
            // Best-effort join. Worker exits within one inference
            // cycle (<200 ms) after seeing the disconnected channel.
            let _ = h.join();
        }
    }
}

#[cfg(any(feature = "onnx-directml", feature = "onnx-cpu"))]
fn worker_loop(
    mut session: ort::session::Session,
    speed_mapper: SpeedMapper,
    conf_threshold: f32,
    nms_iou: f32,
    job_rx: crossbeam_channel::Receiver<InferenceJob>,
    result_tx: crossbeam_channel::Sender<InferenceResult>,
) {
    while let Ok(job) = job_rx.recv() {
        // Stale-skip: frame older than threshold is dropped silently
        // (with a result so the host can count it).
        if now_us().saturating_sub(job.submit_us) > WORKER_STALE_FRAME_US {
            let _ = result_tx.send(InferenceResult {
                frame_id: job.frame_id,
                detections: Vec::new(),
                inference_ms: 0.0,
                skipped_stale: true,
            });
            continue;
        }

        let t0 = Instant::now();
        let detections = do_inference(
            &mut session,
            &speed_mapper,
            &job.rgb,
            job.width,
            job.height,
            conf_threshold,
            nms_iou,
        )
        .unwrap_or_default();
        let inference_ms = t0.elapsed().as_secs_f32() * 1_000.0;

        // If the host has already dropped the receiver (plugin
        // shutdown), send fails — exit the loop.
        if result_tx
            .send(InferenceResult {
                frame_id: job.frame_id,
                detections,
                inference_ms,
                skipped_stale: false,
            })
            .is_err()
        {
            break;
        }
    }
}

// ---------------------------------------------------------------------------
// JPEG decode helper
// ---------------------------------------------------------------------------

/// Decode JPEG bytes into a tightly-packed RGB8 buffer (3 bytes/pixel,
/// row-major, no padding). Returns `(rgb, width, height)`.
pub fn decode_jpeg(bytes: &[u8]) -> Result<(Vec<u8>, u32, u32), image::ImageError> {
    let img = image::load_from_memory_with_format(bytes, image::ImageFormat::Jpeg)?;
    let rgb = img.to_rgb8();
    let (w, h) = (rgb.width(), rgb.height());
    Ok((rgb.into_raw(), w, h))
}

// ---------------------------------------------------------------------------
// Plugin mode
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    /// `frame_store` absent — plugin is dormant.
    MapOnly,
    /// `frame_store` present — process frames as they arrive.
    Vision,
}

// ---------------------------------------------------------------------------
// Plugin struct
// ---------------------------------------------------------------------------

pub struct SignVisionPlugin {
    model_path: PathBuf,
    inference_interval: u32,
    conf_threshold: f32,
    nms_iou: f32,
    mode: Mode,
    last_processed_frame_id: Option<u64>,
    #[allow(dead_code)]
    last_detection: Option<Detection>,
    #[allow(dead_code)]
    last_inference: Instant,
    model_available: bool,
    /// Latency of the most recent inference pass in milliseconds.
    last_inference_ms: f32,
    speed_mapper: SpeedMapper,
    /// Owns the ONNX session; lives only on the worker thread once
    /// `on_load` has spawned it. `None` when no model is available.
    #[cfg(any(feature = "onnx-directml", feature = "onnx-cpu"))]
    worker: Option<InferenceWorker>,

    /// Wall-time (ms) the previous tick spent inside this plugin —
    /// excludes inference, includes JPEG decode + channel ops.
    /// Should stay <2 ms once the worker is in steady state.
    last_tick_blocking_ms: f32,
    /// Frames dropped because the worker channel was full (= worker
    /// busy) and we rotated the slot to keep the newest frame.
    frames_dropped_full: u64,

    // ---- DIAG counters (Phase 6.x sign-vision diagnosis) ----
    diag_tick_count: u64,
    diag_skip_map_source: u64,
    diag_skip_map_only: u64,
    diag_skip_interval: u64,
    diag_frame_fetch_attempts: u64,
    diag_frame_no_store: u64,
    diag_frame_stale_skips: u64,
    diag_frame_fetch_misses: u64,
    diag_frame_already_processed: u64,
    diag_skip_budget: u64,
    diag_decode_failures: u64,
    diag_inference_calls: u64,
    diag_inference_returns_empty: u64,
    diag_detections_published: u64,
}

impl Default for SignVisionPlugin {
    fn default() -> Self {
        Self {
            model_path: PathBuf::from(DEFAULT_MODEL_PATH),
            inference_interval: DEFAULT_INFERENCE_INTERVAL,
            conf_threshold: DEFAULT_CONF_THRESHOLD,
            nms_iou: DEFAULT_NMS_IOU,
            mode: Mode::MapOnly,
            last_processed_frame_id: None,
            last_detection: None,
            last_inference: Instant::now(),
            model_available: false,
            last_inference_ms: 0.0,
            speed_mapper: SpeedMapper::default(),
            #[cfg(any(feature = "onnx-directml", feature = "onnx-cpu"))]
            worker: None,
            last_tick_blocking_ms: 0.0,
            frames_dropped_full: 0,
            diag_tick_count: 0,
            diag_skip_map_source: 0,
            diag_skip_map_only: 0,
            diag_skip_interval: 0,
            diag_frame_fetch_attempts: 0,
            diag_frame_no_store: 0,
            diag_frame_stale_skips: 0,
            diag_frame_fetch_misses: 0,
            diag_frame_already_processed: 0,
            diag_skip_budget: 0,
            diag_decode_failures: 0,
            diag_inference_calls: 0,
            diag_inference_returns_empty: 0,
            diag_detections_published: 0,
        }
    }
}

/// Helper: write all DIAG counters to the blackboard. Called on every
/// tick, regardless of which early-return path was taken — so an
/// operator running `blackboard-query --prefix sign` can see *why* the
/// plugin is silent.
fn publish_diag(p: &SignVisionPlugin, ctx: &PluginContext, last_skip_reason: &str) {
    let bb = &ctx.blackboard;
    bb.set("sign.diag.tick_count", p.diag_tick_count.to_string());
    bb.set("sign.diag.skip_map_source", p.diag_skip_map_source.to_string());
    bb.set("sign.diag.skip_map_only", p.diag_skip_map_only.to_string());
    bb.set("sign.diag.skip_interval", p.diag_skip_interval.to_string());
    bb.set("sign.diag.frame_fetch_attempts", p.diag_frame_fetch_attempts.to_string());
    bb.set("sign.diag.frame_no_store", p.diag_frame_no_store.to_string());
    bb.set("sign.diag.frame_stale_skips", p.diag_frame_stale_skips.to_string());
    bb.set("sign.diag.frame_fetch_misses", p.diag_frame_fetch_misses.to_string());
    bb.set("sign.diag.frame_already_processed", p.diag_frame_already_processed.to_string());
    bb.set("sign.diag.skip_budget", p.diag_skip_budget.to_string());
    bb.set("sign.diag.decode_failures", p.diag_decode_failures.to_string());
    bb.set("sign.diag.inference_calls", p.diag_inference_calls.to_string());
    bb.set("sign.diag.inference_returns_empty", p.diag_inference_returns_empty.to_string());
    bb.set("sign.diag.detections_published", p.diag_detections_published.to_string());
    bb.set("sign.tick_blocking_ms", format!("{:.2}", p.last_tick_blocking_ms));
    bb.set("sign.frames_dropped_full", p.frames_dropped_full.to_string());
    bb.set("sign.diag.last_skip_reason", last_skip_reason);
    bb.set("sign.diag.mode", format!("{:?}", p.mode));
    bb.set("sign.diag.model_available", p.model_available.to_string());
    if let Some(id) = p.last_processed_frame_id {
        bb.set("sign.diag.last_processed_frame_id", id.to_string());
    }
}

// ---------------------------------------------------------------------------
// Inference helpers
// ---------------------------------------------------------------------------

/// Load an ONNX session from `model_path`, trying DirectML first (if compiled)
/// then falling back to CPU. Returns the session and a static label for
/// the EP that actually loaded ("DirectML" or "CPU"), or `None` on any
/// hard failure. The label is propagated to the host log via `ctx_warn!`
/// in `on_load` — raw `tracing::warn!` from a plugin DLL goes to the
/// plugin's own (unsubscribed) tracing global and is silently dropped.
#[cfg(any(feature = "onnx-directml", feature = "onnx-cpu"))]
fn load_onnx_session(
    model_path: &std::path::Path,
) -> Option<(ort::session::Session, &'static str)> {
    use ort::session::Session;

    #[cfg(feature = "onnx-directml")]
    {
        use ort::ep::DirectML;
        if let Ok(builder) = Session::builder() {
            if let Ok(mut builder) = builder.with_execution_providers([DirectML::default().build()])
            {
                match builder.commit_from_file(model_path) {
                    Ok(sess) => return Some((sess, "DirectML")),
                    Err(_) => { /* fall through to CPU */ }
                }
            }
        }
    }

    // CPU fallback (also the only path when only `onnx-cpu` is enabled).
    if let Ok(mut builder) = Session::builder() {
        if let Ok(sess) = builder.commit_from_file(model_path) {
            return Some((sess, "CPU"));
        }
    }
    None
}

/// Run the full YOLOv8s inference pipeline on one RGB8 frame.
/// Returns detected objects with labels and, for speed-limit signs, km/h values.
#[cfg(any(feature = "onnx-directml", feature = "onnx-cpu"))]
fn do_inference(
    session: &mut ort::session::Session,
    speed_mapper: &SpeedMapper,
    rgb: &[u8],
    width: u32,
    height: u32,
    conf_threshold: f32,
    nms_iou: f32,
) -> ort::Result<Vec<Detection>> {
    use ort::value::Tensor;
    use postprocess::{decode_yolov8, nms, unletterbox};
    use preprocess::{crop_rgb, letterbox, rgb_to_gray, to_nchw};

    // 1. Letterbox to 640×640
    let lb = letterbox(rgb, width, height, 640);

    // 2. NCHW f32 tensor [1, 3, 640, 640] — no ndarray needed, (shape, Vec) works directly
    let nchw = to_nchw(&lb.pixels, 640, 640);
    let tensor = Tensor::<f32>::from_array(([1usize, 3, 640, 640], nchw))?;

    // 3. Run inference
    let outputs = session.run(ort::inputs!["images" => tensor])?;

    // 4. Extract output tensor [1, 19, 8400] — try_extract_tensor returns (&Shape, &[f32])
    let (_shape, data) = outputs["output0"].try_extract_tensor::<f32>()?;
    let data: Vec<f32> = data.to_vec();

    // 5. Decode anchors + NMS
    let mut raw = decode_yolov8(&data, NUM_CLASSES, NUM_ANCHORS, conf_threshold);
    let kept = nms(&mut raw, nms_iou);

    // 6. Map to Detection with original-image coords + speed-limit extraction
    let mut detections = Vec::with_capacity(kept.len());
    for det in &kept {
        let orig = unletterbox(det, lb.scale, lb.pad_x, lb.pad_y, width, height);
        let (x1f, y1f, x2f, y2f) = orig.xyxy();
        let x1 = x1f.max(0.0) as u32;
        let y1 = y1f.max(0.0) as u32;
        let x2 = (x2f as u32).min(width);
        let y2 = (y2f as u32).min(height);

        let value = if det.class_id == SPEED_LIMIT_CLASS_ID {
            let (crop, cw, ch) = crop_rgb(rgb, width, height, x1, y1, x2, y2);
            if !crop.is_empty() {
                let gray = rgb_to_gray(&crop);
                speed_mapper
                    .match_speed(&gray, cw, ch)
                    .map(|kmh| kmh as f32)
            } else {
                None
            }
        } else {
            None
        };

        let label = if let Some(kmh) = value {
            format!("speed_limit_{}", kmh as u32)
        } else {
            CLASS_NAMES
                .get(det.class_id)
                .copied()
                .unwrap_or("unknown")
                .to_string()
        };

        detections.push(Detection {
            label,
            confidence: det.confidence,
            value,
        });
    }

    Ok(detections)
}

// ---------------------------------------------------------------------------
// Plugin impl
// ---------------------------------------------------------------------------

enum NextFrame {
    Got(Arc<SharedFrame>),
    Stale,
    NoStore,
    Miss,
    AlreadyProcessed(u64),
}

impl SignVisionPlugin {
    /// Outcome of the per-tick frame-fetch attempt — used by the
    /// instrumented tick path to update DIAG counters and the
    /// `sign.diag.last_skip_reason` key.
    fn next_frame_diag(&mut self, ctx: &PluginContext) -> NextFrame {
        self.diag_frame_fetch_attempts += 1;
        if ctx.blackboard.get("vision.frame.stale").as_deref() == Some("true") {
            self.diag_frame_stale_skips += 1;
            return NextFrame::Stale;
        }
        let Some(store) = ctx.frame_store() else {
            self.diag_frame_no_store += 1;
            return NextFrame::NoStore;
        };
        let Some(frame) = store.get(FRAME_KEY) else {
            self.diag_frame_fetch_misses += 1;
            return NextFrame::Miss;
        };
        if Some(frame.id) == self.last_processed_frame_id {
            self.diag_frame_already_processed += 1;
            return NextFrame::AlreadyProcessed(frame.id);
        }
        NextFrame::Got(frame)
    }

    /// Drain all completed inference results from the worker. Returns
    /// the most recent (frame_id, detections) pair plus a flag for
    /// whether any newer one superseded it (so we can publish stats).
    /// All older results are discarded — only the latest matters.
    #[cfg(any(feature = "onnx-directml", feature = "onnx-cpu"))]
    fn drain_results(&mut self) -> Option<InferenceResult> {
        let worker = self.worker.as_ref()?;
        let mut latest: Option<InferenceResult> = None;
        while let Ok(r) = worker.result_rx.try_recv() {
            latest = Some(r);
        }
        latest
    }

    /// Submit a frame to the worker with drop-oldest backpressure.
    /// Returns true if accepted, false if the channel was full and
    /// we kept the in-flight frame (the new one is dropped).
    #[cfg(any(feature = "onnx-directml", feature = "onnx-cpu"))]
    fn submit_frame(&mut self, job: InferenceJob) -> bool {
        let Some(worker) = self.worker.as_ref() else {
            return false;
        };
        let Some(job_tx) = worker.job_tx.as_ref() else {
            return false;
        };
        match job_tx.try_send(job) {
            Ok(()) => true,
            Err(crossbeam_channel::TrySendError::Full(job)) => {
                // Drain the queued (older) frame and try once more.
                // tick() is the only producer, so after this try_recv
                // the channel is empty and the next try_send is
                // guaranteed to succeed unless the worker exited.
                let _ = worker.job_drain_rx.try_recv();
                self.frames_dropped_full += 1;
                job_tx.try_send(job).is_ok()
            }
            Err(crossbeam_channel::TrySendError::Disconnected(_)) => {
                // Worker died — bail out, don't poison further state.
                false
            }
        }
    }

    /// Drain results from the worker and publish the most recent one
    /// to the blackboard. Runs every tick (cheap when empty).
    #[cfg(any(feature = "onnx-directml", feature = "onnx-cpu"))]
    fn publish_latest_result(&mut self, ctx: &PluginContext) {
        let Some(result) = self.drain_results() else {
            return;
        };

        if result.skipped_stale {
            // Stale frames count as worked-but-empty; don't move the
            // last-inference-ms metric, but do tick the diag counter.
            self.diag_inference_returns_empty += 1;
            return;
        }

        self.diag_inference_calls += 1;
        self.last_inference_ms = result.inference_ms;
        ctx.blackboard.set(
            "sign.last_inference_ms",
            format!("{:.1}", self.last_inference_ms),
        );
        ctx.blackboard
            .set("sign.detected_count", result.detections.len().to_string());

        if result.detections.is_empty() {
            self.diag_inference_returns_empty += 1;
        } else {
            self.diag_detections_published += result.detections.len() as u64;
        }

        // Pick the highest-confidence speed-limit detection above threshold.
        let best = result
            .detections
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
                    .set("sign.speed_limit_confidence", format!("{conf:.3}"));
                ctx.blackboard
                    .set("sign.vision.confidence", format!("{conf:.3}"));
                ctx_debug!(
                    ctx,
                    target: LOG_TARGET,
                    "speed limit {limit_kmh:.0} km/h detected (conf={conf:.2}, frame_id={})",
                    result.frame_id
                );
            }
            None => {
                if ctx.blackboard.get("sign.source").as_deref() == Some("vision") {
                    ctx.blackboard.remove("sign.speed_limit_kmh");
                    ctx.blackboard.remove("sign.source");
                    ctx.blackboard.remove("sign.vision.confidence");
                    ctx.blackboard.remove("sign.speed_limit_confidence");
                }
            }
        }
    }

    fn record_tick_blocking(&mut self, tick_start: Instant) {
        self.last_tick_blocking_ms = tick_start.elapsed().as_secs_f32() * 1_000.0;
    }
}

impl Plugin for SignVisionPlugin {
    fn name(&self) -> &str {
        "sign-vision"
    }
    fn version(&self) -> &str {
        "0.3.0"
    }
    fn settings_schema(&self) -> &str {
        r#"{
  "type": "object",
  "properties": {
    "model_path": {
      "type": "string",
      "description": "Path to the YOLOv8s ONNX model (default: models/truckpilot-yolov8s-v2/best.onnx)."
    },
    "inference_interval": {
      "type": "integer",
      "minimum": 1,
      "maximum": 250,
      "description": "Run inference every N PhaseB ticks. Default 1 (10 Hz)."
    },
    "conf_threshold": {
      "type": "number",
      "minimum": 0.1,
      "maximum": 1.0,
      "description": "YOLO confidence threshold. Default 0.5."
    },
    "nms_iou": {
      "type": "number",
      "minimum": 0.1,
      "maximum": 1.0,
      "description": "NMS IoU threshold. Default 0.45."
    },
    "min_confidence": {
      "type": "number",
      "minimum": 0.1,
      "maximum": 1.0,
      "description": "Min confidence to override map source. Default 0.7."
    }
  }
}"#
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        if let Some(p) = ctx.blackboard.get("sign_vision.model_path") {
            self.model_path = PathBuf::from(p);
        }
        if let Some(iv) = ctx.blackboard.get_f64("sign_vision.inference_interval") {
            self.inference_interval = (iv as u32).clamp(1, 250);
        }
        if let Some(conf) = ctx.blackboard.get_f64("sign_vision.conf_threshold") {
            self.conf_threshold = conf as f32;
        }
        if let Some(iou) = ctx.blackboard.get_f64("sign_vision.nms_iou") {
            self.nms_iou = iou as f32;
        }

        self.mode = if ctx.frame_store().is_some() {
            Mode::Vision
        } else {
            Mode::MapOnly
        };

        ctx.blackboard.set(
            "sign.detection_active",
            if self.mode == Mode::Vision {
                "true"
            } else {
                "false"
            },
        );

        if self.model_path.exists() {
            #[cfg(any(feature = "onnx-directml", feature = "onnx-cpu"))]
            {
                match load_onnx_session(&self.model_path) {
                    Some((sess, ep_label)) => {
                        // Spawn the inference worker thread and move
                        // the session into it; the tick path never
                        // touches the session directly.
                        self.worker = Some(InferenceWorker::spawn(
                            sess,
                            self.speed_mapper.clone(),
                            self.conf_threshold,
                            self.nms_iou,
                        ));
                        self.model_available = true;
                        ctx_warn!(
                            ctx,
                            target: LOG_TARGET,
                            "ONNX EP active: {ep_label} (async worker thread)"
                        );
                        ctx.blackboard.set("sign.onnx.provider", ep_label);
                    }
                    None => {
                        ctx_warn!(
                            ctx,
                            target: LOG_TARGET,
                            "ONNX session init failed for {:?} — stub inference active",
                            self.model_path
                        );
                    }
                }
            }
            #[cfg(not(any(feature = "onnx-directml", feature = "onnx-cpu")))]
            ctx_warn!(
                ctx,
                target: LOG_TARGET,
                "model found at {:?} but no onnx feature compiled — build with --features onnx-directml",
                self.model_path
            );
        } else {
            ctx_info!(
                ctx,
                target: LOG_TARGET,
                "model not found at {:?} — visual recognition disabled",
                self.model_path
            );
        }

        ctx_info!(
            ctx,
            target: LOG_TARGET,
            "loaded — mode={:?} model_available={} conf={} nms_iou={} interval={}",
            self.mode,
            self.model_available,
            self.conf_threshold,
            self.nms_iou,
            self.inference_interval,
        );
    }

    fn on_unload(&mut self) {
        // Drops the worker handle → closes the job channel → worker
        // recv() returns Err → worker thread exits and is joined.
        #[cfg(any(feature = "onnx-directml", feature = "onnx-cpu"))]
        {
            self.worker = None;
        }
        tracing::info!(target: LOG_TARGET, "unloaded");
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
        let tick_start = Instant::now();
        self.diag_tick_count += 1;

        // --- 1) Always drain completed inference results first. This
        //         runs even on early-return paths so the worker's
        //         backlog never blocks publication, and the latest
        //         result is always visible on the blackboard.
        #[cfg(any(feature = "onnx-directml", feature = "onnx-cpu"))]
        self.publish_latest_result(ctx);

        if ctx.blackboard.get("sign.source").as_deref() == Some("map") {
            self.diag_skip_map_source += 1;
            self.record_tick_blocking(tick_start);
            publish_diag(self, ctx, "map_source");
            return;
        }
        if self.mode == Mode::MapOnly {
            self.diag_skip_map_only += 1;
            self.record_tick_blocking(tick_start);
            publish_diag(self, ctx, "map_only_mode");
            return;
        }
        if !ctx
            .tick_count
            .is_multiple_of(self.inference_interval as u64)
        {
            self.diag_skip_interval += 1;
            self.record_tick_blocking(tick_start);
            publish_diag(self, ctx, "interval");
            return;
        }

        let frame = match self.next_frame_diag(ctx) {
            NextFrame::Got(f) => f,
            NextFrame::Stale => {
                self.record_tick_blocking(tick_start);
                publish_diag(self, ctx, "frame_stale");
                return;
            }
            NextFrame::NoStore => {
                self.record_tick_blocking(tick_start);
                publish_diag(self, ctx, "no_frame_store");
                return;
            }
            NextFrame::Miss => {
                self.record_tick_blocking(tick_start);
                publish_diag(self, ctx, "frame_miss");
                return;
            }
            NextFrame::AlreadyProcessed(_id) => {
                self.record_tick_blocking(tick_start);
                publish_diag(self, ctx, "already_processed");
                return;
            }
        };

        ctx_trace!(
            ctx,
            target: LOG_TARGET,
            "frame id={} {}x{} jpeg={}B",
            frame.id, frame.width, frame.height, frame.jpeg.len()
        );

        // Lazy JPEG → RGB8 decode (result cached in SharedFrame OnceLock).
        let decode_attempt: Result<&Arc<Vec<u8>>, image::ImageError> =
            frame.get_or_try_init_rgb8(|| {
                let (rgb, w, h) = decode_jpeg(&frame.jpeg)?;
                debug_assert_eq!(rgb.len() as u32, w * h * 3);
                Ok(Arc::new(rgb))
            });

        let rgb = match decode_attempt {
            Ok(rgb) => rgb,
            Err(e) => {
                ctx_warn!(
                    ctx,
                    target: LOG_TARGET,
                    "JPEG decode failed for frame id={}: {e}",
                    frame.id
                );
                self.diag_decode_failures += 1;
                self.last_processed_frame_id = Some(frame.id);
                self.record_tick_blocking(tick_start);
                publish_diag(self, ctx, "decode_failed");
                return;
            }
        };

        // --- 2) Submit frame to worker (drop-oldest on full).
        //         tick() never waits for inference; the result will be
        //         picked up on a future tick via drain_results().
        #[cfg(any(feature = "onnx-directml", feature = "onnx-cpu"))]
        let submitted = {
            let job = InferenceJob {
                frame_id: frame.id,
                rgb: Arc::clone(rgb),
                width: frame.width,
                height: frame.height,
                submit_us: now_us(),
            };
            self.submit_frame(job)
        };
        #[cfg(not(any(feature = "onnx-directml", feature = "onnx-cpu")))]
        let submitted = false;

        // Mark the frame as "seen" regardless of whether we submitted
        // it — otherwise next_frame_diag would keep re-fetching the
        // same SharedFrame every tick.
        self.last_processed_frame_id = Some(frame.id);

        let reason = if submitted { "submitted" } else { "drop_full" };
        self.record_tick_blocking(tick_start);
        publish_diag(self, ctx, reason);
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

    // ---- regression: label parsing ----------------------------------

    #[test]
    fn parse_speed_limit_label() {
        assert_eq!(Detection::parse_speed_limit("speed_limit_80"), Some(80.0));
        assert_eq!(Detection::parse_speed_limit("speed_limit_120"), Some(120.0));
        assert_eq!(Detection::parse_speed_limit("stop"), None);
        assert_eq!(Detection::parse_speed_limit("speed_limit_abc"), None);
    }

    // ---- regression: defaults ---------------------------------------

    #[test]
    fn default_plugin_model_not_available() {
        let p = SignVisionPlugin::default();
        assert!(!p.model_available);
    }

    #[test]
    fn default_conf_threshold() {
        let p = SignVisionPlugin::default();
        assert!((p.conf_threshold - DEFAULT_CONF_THRESHOLD).abs() < 1e-6);
    }

    #[test]
    fn default_nms_iou() {
        let p = SignVisionPlugin::default();
        assert!((p.nms_iou - DEFAULT_NMS_IOU).abs() < 1e-6);
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
                "tick {i} must not trigger"
            );
        }
        assert!(5u64.is_multiple_of(p.inference_interval as u64));
    }

    #[test]
    fn missing_model_does_not_panic() {
        let mut p = SignVisionPlugin::default();
        let ctx = ctx_map_only();
        p.on_load(&ctx);
        assert!(!p.model_available);
    }

    // ---- mode selection -------------------------------------------

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

    // ---- tick behaviour -------------------------------------------

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
        assert!(frame.decoded_rgb8().is_none(), "should not be decoded yet");

        let mut out = ControlOutput::default();
        p.tick(None, &mut out, &ctx);

        assert_eq!(p.last_processed_frame_id, Some(42));
        let rgb = frame.decoded_rgb8().expect("OnceLock must be populated");
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
        // Replace with a new SharedFrame that has the same id; must not be decoded.
        let dup = frame_with(7, one_px_white_jpeg());
        store.set(FRAME_KEY, Arc::clone(&dup));
        p.tick(None, &mut out, &ctx);

        assert!(
            dup.decoded_rgb8().is_none(),
            "duplicate frame must not be decoded"
        );
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
        store.set(FRAME_KEY, frame_with(11, vec![0u8; 16]));

        let mut out = ControlOutput::default();
        p.tick(None, &mut out, &ctx);

        assert_eq!(p.last_processed_frame_id, Some(11));
        assert!(ctx.blackboard.get("sign.source").is_none());
    }

    #[test]
    fn tick_records_blocking_metric() {
        let mut p = SignVisionPlugin::default();
        let (ctx, store) = ctx_vision();
        p.on_load(&ctx);
        store.set(FRAME_KEY, frame_with(1, one_px_white_jpeg()));

        let mut out = ControlOutput::default();
        p.tick(None, &mut out, &ctx);

        // Async worker may not have published a result yet, so
        // sign.last_inference_ms is not guaranteed on a single tick.
        // But the tick-blocking metric is always written.
        assert!(ctx.blackboard.get("sign.tick_blocking_ms").is_some());
        // Frame was marked processed regardless of submission outcome.
        assert_eq!(p.last_processed_frame_id, Some(1));
    }

    // ---- decode_jpeg -------------------------------------------

    #[test]
    fn decode_jpeg_returns_rgb8_of_expected_size() {
        let (rgb, w, h) = decode_jpeg(&one_px_white_jpeg()).expect("decode");
        assert_eq!((w, h), (1, 1));
        assert_eq!(rgb, vec![255, 255, 255]);
    }

    #[test]
    fn decode_jpeg_rejects_non_jpeg_bytes() {
        let err = decode_jpeg(b"not a jpeg").unwrap_err();
        let _ = format!("{err}");
    }
}
