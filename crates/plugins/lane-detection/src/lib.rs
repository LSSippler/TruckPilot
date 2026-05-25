//! Lane-Detection plugin — Phase 5.29-C.
//!
//! Reads JPEG frames from `SharedFrameStore` ("camera.front"), runs
//! UFLD v2 TuSimple/CULane row-anchor inference on a dedicated thread, and
//! publishes lane data to the Blackboard.
//!
//! ## Blackboard outputs (SCHEMA-LOCK)
//!
//! | Key                      | Type  | Range      | Notes                                        |
//! |--------------------------|-------|------------|----------------------------------------------|
//! | `lane.center_offset`     | f64   | [-1, +1]   | **Bias-corrected** offset; +1 = truck far right |
//! | `lane.center_offset_raw` | f64   | [-1, +1]   | Raw offset before bias subtraction           |
//! | `lane.center_offset_bias`| f64   | [-1, +1]   | Current median bias estimate                 |
//! | `lane.confidence`        | f64   | [0, 1]     | <0.3 unreliable                              |
//! | `lane.detection_count`   | u32   | 0–4        |                                              |
//! | `lane.left_visible`      | bool  |            |                                              |
//! | `lane.right_visible`     | bool  |            |                                              |
//! | `lane.left_x`            | f64   | [-1, +1]   | Ego-left lane x in frame coords; NaN=absent  |
//! | `lane.right_x`           | f64   | [-1, +1]   | Ego-right lane x in frame coords; NaN=absent |
//!
//! ## Default state
//!
//! The plugin is enabled or disabled via `[plugins.lane-detection]` in
//! `truckpilot.toml`, managed by the PluginManager.  The plugin itself has no
//! internal `enabled` flag — it always attempts to load the model in `on_load`
//! and publishes `lane.diag.load_ok` to indicate success or failure.
//! `lane.diag.enabled` is an alias of `lane.diag.load_ok` for backwards
//! compatibility.
//!
//! ## Build features
//!
//! | Feature         | Default | Effect                             |
//! |-----------------|---------|-----------------------------------|
//! | `onnx-cpu`      | yes     | CPU EP (~20 ms TuSimple / 37 ms CULane) |
//! | `onnx-directml` | no      | GPU via DirectML (86 ms, too slow) |
//! | (neither)       | —       | Stub mode — inference returns nothing |

mod bias;
mod clustering;
mod postprocess;
mod preprocess;

use std::path::PathBuf;
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Instant;

use truckpilot_plugin_api::{
    ctx_debug, ctx_error, ctx_info, ctx_trace, ctx_warn, ControlOutput, Plugin, PluginContext,
    Telemetry, TickPhase,
};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const LOG_TARGET: &str = "truckpilot_plugin_lane_detection";
const DEFAULT_MODEL_PATH: &str = "tools/ufld-test/models/ufldv2_tusimple_res18_320x800.onnx";
const FRAME_KEY: &str = "camera.front";
const WORKER_STALE_FRAME_US: u64 = 200_000;
const DEFAULT_EXIST_THRESH: f32 = 0.5;
/// Number of frames in the moving-median window for bias estimation.
const DEFAULT_BIAS_WINDOW_SIZE: usize = 100;
/// Frames to collect before bias correction activates.
const DEFAULT_BIAS_WARMUP_FRAMES: usize = 50;

// ---------------------------------------------------------------------------
// Async inference types (only when an onnx feature is active)
// ---------------------------------------------------------------------------

#[cfg(any(feature = "onnx-cpu", feature = "onnx-directml"))]
struct InferenceJob {
    rgb: Arc<Vec<u8>>,
    orig_w: u32,
    orig_h: u32,
    submitted_at: Instant,
}

#[cfg(any(feature = "onnx-cpu", feature = "onnx-directml"))]
struct InferenceResult {
    /// Flat loc_row data with batch dim stripped: [C * R * L].
    loc_row: Vec<f32>,
    /// Flat exist_row data with batch dim stripped: [2 * R * L].
    exist_row: Vec<f32>,
    col_grids: usize,
    row_anchors: usize,
    num_lanes: usize,
    letterbox_meta: preprocess::LetterboxMeta,
    latency_us: u64,
    /// Non-empty string if inference produced an error.
    error: Option<String>,
}

/// Static model configuration derived from the on_load probe inference.
#[cfg(any(feature = "onnx-cpu", feature = "onnx-directml"))]
#[derive(Clone)]
struct ModelConfig {
    col_grids: usize,
    row_anchors: usize,
    num_lanes: usize,
    input_w: usize,
    input_h: usize,
    input_name: String,
    loc_row_name: String,
    exist_row_name: String,
}

/// Inference worker handle.
#[cfg(any(feature = "onnx-cpu", feature = "onnx-directml"))]
struct InferenceWorker {
    job_tx: Option<crossbeam_channel::Sender<InferenceJob>>,
    job_drain_rx: crossbeam_channel::Receiver<InferenceJob>,
    result_rx: crossbeam_channel::Receiver<InferenceResult>,
    handle: Option<JoinHandle<()>>,
}

#[cfg(any(feature = "onnx-cpu", feature = "onnx-directml"))]
impl InferenceWorker {
    fn spawn(session: ort::session::Session, cfg: ModelConfig, exist_thresh: f32) -> Self {
        let (job_tx, job_rx) = crossbeam_channel::bounded::<InferenceJob>(1);
        let job_drain_rx = job_rx.clone();
        let (result_tx, result_rx) = crossbeam_channel::bounded::<InferenceResult>(1);

        let handle = std::thread::Builder::new()
            .name("lane-detection-inference".into())
            .spawn(move || worker_loop(session, cfg, exist_thresh, job_rx, result_tx))
            .expect("spawn lane-detection-inference thread");

        Self {
            job_tx: Some(job_tx),
            job_drain_rx,
            result_rx,
            handle: Some(handle),
        }
    }
}

#[cfg(any(feature = "onnx-cpu", feature = "onnx-directml"))]
impl Drop for InferenceWorker {
    fn drop(&mut self) {
        drop(self.job_tx.take()); // close channel → worker recv() returns Err → exits
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

#[cfg(any(feature = "onnx-cpu", feature = "onnx-directml"))]
fn worker_loop(
    mut session: ort::session::Session,
    cfg: ModelConfig,
    exist_thresh: f32,
    job_rx: crossbeam_channel::Receiver<InferenceJob>,
    result_tx: crossbeam_channel::Sender<InferenceResult>,
) {
    while let Ok(job) = job_rx.recv() {
        if job.submitted_at.elapsed().as_micros() as u64 > WORKER_STALE_FRAME_US {
            continue;
        }

        let t0 = Instant::now();

        let lb = match preprocess::letterbox_and_normalize(
            &job.rgb,
            job.orig_w,
            job.orig_h,
            cfg.input_w,
            cfg.input_h,
        ) {
            Ok(r) => r,
            Err(e) => {
                let _ = result_tx.send(InferenceResult {
                    loc_row: vec![],
                    exist_row: vec![],
                    col_grids: cfg.col_grids,
                    row_anchors: cfg.row_anchors,
                    num_lanes: cfg.num_lanes,
                    letterbox_meta: preprocess::LetterboxMeta {
                        scale: 1.0,
                        pad_x: 0.0,
                        pad_y: 0.0,
                        orig_w: job.orig_w,
                        orig_h: job.orig_h,
                        canvas_w: cfg.input_w as u32,
                        canvas_h: cfg.input_h as u32,
                    },
                    latency_us: 0,
                    error: Some(format!("preprocess: {e}")),
                });
                continue;
            }
        };

        let tensor = match ort::value::Tensor::<f32>::from_array((
            [1usize, 3, cfg.input_h, cfg.input_w],
            lb.nchw.clone(),
        )) {
            Ok(t) => t,
            Err(e) => {
                let _ = result_tx.send(make_error_result(&cfg, &lb.meta, format!("tensor: {e}")));
                continue;
            }
        };

        let outputs = match session.run(ort::inputs![cfg.input_name.as_str() => tensor]) {
            Ok(o) => o,
            Err(e) => {
                let _ =
                    result_tx.send(make_error_result(&cfg, &lb.meta, format!("inference: {e}")));
                continue;
            }
        };

        let loc_row = match extract_flat_f32(&outputs, &cfg.loc_row_name) {
            Ok(v) => v,
            Err(e) => {
                let _ = result_tx.send(make_error_result(&cfg, &lb.meta, format!("loc_row: {e}")));
                continue;
            }
        };

        let exist_row = match extract_flat_f32(&outputs, &cfg.exist_row_name) {
            Ok(v) => v,
            Err(e) => {
                let _ =
                    result_tx.send(make_error_result(&cfg, &lb.meta, format!("exist_row: {e}")));
                continue;
            }
        };

        let latency_us = t0.elapsed().as_micros() as u64;
        let _ = exist_thresh; // used inside decode_ufld_v2 via postprocess

        if result_tx
            .send(InferenceResult {
                loc_row,
                exist_row,
                col_grids: cfg.col_grids,
                row_anchors: cfg.row_anchors,
                num_lanes: cfg.num_lanes,
                letterbox_meta: lb.meta,
                latency_us,
                error: None,
            })
            .is_err()
        {
            break;
        }
    }
}

#[cfg(any(feature = "onnx-cpu", feature = "onnx-directml"))]
fn make_error_result(
    cfg: &ModelConfig,
    meta: &preprocess::LetterboxMeta,
    msg: String,
) -> InferenceResult {
    InferenceResult {
        loc_row: vec![],
        exist_row: vec![],
        col_grids: cfg.col_grids,
        row_anchors: cfg.row_anchors,
        num_lanes: cfg.num_lanes,
        letterbox_meta: meta.clone(),
        latency_us: 0,
        error: Some(msg),
    }
}

/// Extract a flat `Vec<f32>` from a named output, stripping the batch dim.
#[cfg(any(feature = "onnx-cpu", feature = "onnx-directml"))]
fn extract_flat_f32(
    outputs: &ort::session::SessionOutputs<'_>,
    name: &str,
) -> Result<Vec<f32>, String> {
    let (_shape, data) = outputs[name]
        .try_extract_tensor::<f32>()
        .map_err(|e| e.to_string())?;
    // data is CowArray / ArrayView — iterate to collect flat f32 buffer.
    Ok(data.to_vec())
}

// ---------------------------------------------------------------------------
// Session loading helper (mirrors sign-vision)
// ---------------------------------------------------------------------------

#[cfg(any(feature = "onnx-cpu", feature = "onnx-directml"))]
fn load_onnx_session(
    model_path: &std::path::Path,
) -> Option<(ort::session::Session, &'static str, Option<String>)> {
    use ort::session::Session;

    let directml_warn: Option<String> = {
        #[cfg(feature = "onnx-directml")]
        {
            use ort::ep::DirectML;
            let ep = DirectML::default().build().error_on_failure();
            let result = Session::builder().and_then(|b| Ok(b.with_execution_providers([ep])?));
            match result.and_then(|mut b| b.commit_from_file(model_path)) {
                Ok(sess) => return Some((sess, "DirectML", None)),
                Err(e) => Some(format!("DirectML init failed ({e}), falling back to CPU")),
            }
        }
        #[cfg(not(feature = "onnx-directml"))]
        None
    };

    match Session::builder().and_then(|mut b| b.commit_from_file(model_path)) {
        Ok(sess) => Some((sess, "CPU", directml_warn)),
        Err(_) => None,
    }
}

/// Run a probe inference to determine output tensor shapes and names.
///
/// Tries known input shapes in order: TuSimple (320×800) first, then CULane (320×1600).
/// The first shape that the model accepts defines the ModelConfig.
#[cfg(any(feature = "onnx-cpu", feature = "onnx-directml"))]
fn probe_model_shape(session: &mut ort::session::Session) -> Result<ModelConfig, String> {
    use ort::value::Tensor;

    if session.outputs().len() < 3 {
        return Err(format!(
            "UFLD v2: expected >=3 outputs, model has {}",
            session.outputs().len()
        ));
    }

    let input_name = session
        .inputs()
        .first()
        .ok_or("no model inputs")?
        .name()
        .to_string();
    let loc_row_name = session.outputs()[0].name().to_string();
    let exist_row_name = session.outputs()[2].name().to_string();

    for (input_h, input_w) in [(320usize, 800usize), (320usize, 1600usize)] {
        let n = 3 * input_h * input_w;
        let probe_tensor =
            match Tensor::<f32>::from_array(([1usize, 3, input_h, input_w], vec![0.0f32; n])) {
                Ok(t) => t,
                Err(e) => return Err(format!("probe tensor: {e}")),
            };

        let outputs = match session.run(ort::inputs![input_name.as_str() => probe_tensor]) {
            Ok(o) => o,
            Err(_) => continue, // wrong shape for this model, try next
        };

        // loc_row shape: [1, C, R, L]
        let (loc_shape, _) = outputs[loc_row_name.as_str()]
            .try_extract_tensor::<f32>()
            .map_err(|e| format!("loc_row extract: {e}"))?;

        if loc_shape.len() != 4 {
            return Err(format!(
                "UFLD v2: loc_row expected 4D, got {}D: {:?}",
                loc_shape.len(),
                loc_shape
            ));
        }

        let col_grids = loc_shape[1] as usize;
        let row_anchors = loc_shape[2] as usize;
        let num_lanes = loc_shape[3] as usize;

        if col_grids == 0 || row_anchors == 0 || num_lanes == 0 {
            return Err(format!(
                "UFLD v2: invalid loc_row shape: C={col_grids} R={row_anchors} L={num_lanes}"
            ));
        }

        // exist_row shape: [1, 2, R, L] — validate R and L match
        let (exist_shape, _) = outputs[exist_row_name.as_str()]
            .try_extract_tensor::<f32>()
            .map_err(|e| format!("exist_row extract: {e}"))?;

        if exist_shape.len() < 4
            || (exist_shape[2] as usize) != row_anchors
            || (exist_shape[3] as usize) != num_lanes
        {
            return Err(format!(
                "UFLD v2: exist_row shape mismatch (expected [1,2,{row_anchors},{num_lanes}], got {:?})",
                exist_shape
            ));
        }

        return Ok(ModelConfig {
            col_grids,
            row_anchors,
            num_lanes,
            input_h,
            input_w,
            input_name,
            loc_row_name,
            exist_row_name,
        });
    }

    Err("UFLD v2: unsupported input shape (tried 320×800 and 320×1600)".to_string())
}

// ---------------------------------------------------------------------------
// Plugin struct
// ---------------------------------------------------------------------------

pub struct LaneDetectionPlugin {
    model_path: PathBuf,
    exist_thresh: f32,

    /// Set in on_load; false if model validation failed.
    load_ok: bool,
    last_processed_frame_id: Option<u64>,

    // Derived from probe in on_load (non-zero only when load_ok)
    col_grids: usize,
    row_anchors: usize,
    num_lanes: usize,

    // DIAG counters
    diag_tick_count: u64,
    diag_frames_submitted: u64,
    diag_frames_decoded: u64,
    diag_inference_errors: u64,
    diag_last_latency_ms: f64,
    diag_frames_dropped_full: u64,

    /// Moving-median bias estimator for cockpit-view offset correction.
    bias: bias::BiasEstimator,

    #[cfg(any(feature = "onnx-cpu", feature = "onnx-directml"))]
    worker: Option<InferenceWorker>,
}

impl Default for LaneDetectionPlugin {
    fn default() -> Self {
        Self {
            model_path: PathBuf::from(DEFAULT_MODEL_PATH),
            exist_thresh: DEFAULT_EXIST_THRESH,
            load_ok: false,
            last_processed_frame_id: None,
            col_grids: 0,
            row_anchors: 0,
            num_lanes: 0,
            diag_tick_count: 0,
            diag_frames_submitted: 0,
            diag_frames_decoded: 0,
            diag_inference_errors: 0,
            diag_last_latency_ms: 0.0,
            diag_frames_dropped_full: 0,
            bias: bias::BiasEstimator::new(DEFAULT_BIAS_WINDOW_SIZE, DEFAULT_BIAS_WARMUP_FRAMES),
            #[cfg(any(feature = "onnx-cpu", feature = "onnx-directml"))]
            worker: None,
        }
    }
}

impl LaneDetectionPlugin {
    /// Try to load the model, probe shapes, and spawn the worker.
    /// Returns Err with a human-readable message on any failure.
    #[cfg(any(feature = "onnx-cpu", feature = "onnx-directml"))]
    fn try_init_worker(&mut self, ctx: &PluginContext) -> Result<(), String> {
        if !self.model_path.exists() {
            return Err(format!("model not found: {:?}", self.model_path));
        }

        let (mut session, ep_label, warn) = load_onnx_session(&self.model_path)
            .ok_or_else(|| format!("ONNX session init failed for {:?}", self.model_path))?;

        if let Some(ref w) = warn {
            ctx_warn!(ctx, target: LOG_TARGET, "{w}");
        }

        let cfg = probe_model_shape(&mut session)?;

        ctx_info!(
            ctx,
            target: LOG_TARGET,
            "ONNX EP={ep_label} input={}×{} col_grids={} row_anchors={} num_lanes={}",
            cfg.input_w, cfg.input_h, cfg.col_grids, cfg.row_anchors, cfg.num_lanes
        );
        ctx.blackboard.set("lane.onnx.provider", ep_label);

        self.col_grids = cfg.col_grids;
        self.row_anchors = cfg.row_anchors;
        self.num_lanes = cfg.num_lanes;

        self.worker = Some(InferenceWorker::spawn(session, cfg, self.exist_thresh));
        Ok(())
    }

    /// Submit an inference job with drop-oldest backpressure.
    #[cfg(any(feature = "onnx-cpu", feature = "onnx-directml"))]
    fn submit_frame(&mut self, job: InferenceJob) {
        let Some(worker) = self.worker.as_ref() else {
            return;
        };
        let Some(tx) = worker.job_tx.as_ref() else {
            return;
        };
        match tx.try_send(job) {
            Ok(()) => {
                self.diag_frames_submitted += 1;
            }
            Err(crossbeam_channel::TrySendError::Full(job)) => {
                let _ = worker.job_drain_rx.try_recv();
                self.diag_frames_dropped_full += 1;
                if tx.try_send(job).is_ok() {
                    self.diag_frames_submitted += 1;
                }
            }
            Err(crossbeam_channel::TrySendError::Disconnected(_)) => {}
        }
    }

    /// Drain completed results, decode the most recent one, publish to blackboard.
    #[cfg(any(feature = "onnx-cpu", feature = "onnx-directml"))]
    fn publish_latest_result(&mut self, ctx: &PluginContext) {
        let worker = match self.worker.as_ref() {
            Some(w) => w,
            None => return,
        };

        let mut latest: Option<InferenceResult> = None;
        while let Ok(r) = worker.result_rx.try_recv() {
            latest = Some(r);
        }

        let result = match latest {
            Some(r) => r,
            None => return,
        };

        if let Some(ref e) = result.error {
            self.diag_inference_errors += 1;
            ctx_warn!(ctx, target: LOG_TARGET, "inference error: {e}");
            return;
        }

        self.diag_frames_decoded += 1;
        self.diag_last_latency_ms = result.latency_us as f64 / 1_000.0;

        let lanes = postprocess::decode_ufld_v2(
            &result.loc_row,
            &result.exist_row,
            result.col_grids,
            result.row_anchors,
            result.num_lanes,
            &result.letterbox_meta,
            self.exist_thresh,
        );

        let lane_result = clustering::compute_lane_result(&lanes, result.letterbox_meta.orig_w);

        // Bias compensation: push raw offset, compute corrected value.
        let raw_offset = lane_result.center_offset_norm;
        self.bias.push(raw_offset);
        let bias_val = self.bias.bias();
        let corrected_offset = self.bias.corrected(raw_offset);

        let bb = &ctx.blackboard;
        bb.set("lane.center_offset", format!("{:.6}", corrected_offset));
        bb.set("lane.center_offset_raw", format!("{:.6}", raw_offset));
        bb.set("lane.center_offset_bias", format!("{:.6}", bias_val));
        bb.set("lane.confidence", format!("{:.6}", lane_result.confidence));
        bb.set(
            "lane.detection_count",
            lane_result.detection_count.to_string(),
        );
        bb.set("lane.left_visible", lane_result.left_visible.to_string());
        bb.set("lane.right_visible", lane_result.right_visible.to_string());
        bb.set("lane.left_x", format!("{:.6}", lane_result.left_x_norm));
        bb.set("lane.right_x", format!("{:.6}", lane_result.right_x_norm));

        ctx_debug!(
            ctx, target: LOG_TARGET,
            "lanes={} offset_raw={:.3} bias={:.3} offset={:.3} conf={:.2} latency={:.1}ms",
            lane_result.detection_count,
            raw_offset,
            bias_val,
            corrected_offset,
            lane_result.confidence,
            self.diag_last_latency_ms,
        );
    }

    fn publish_diag(&self, ctx: &PluginContext) {
        let bb = &ctx.blackboard;
        bb.set("lane.diag.tick_count", self.diag_tick_count.to_string());
        bb.set(
            "lane.diag.frames_submitted",
            self.diag_frames_submitted.to_string(),
        );
        bb.set(
            "lane.diag.frames_decoded",
            self.diag_frames_decoded.to_string(),
        );
        bb.set(
            "lane.diag.inference_errors",
            self.diag_inference_errors.to_string(),
        );
        bb.set(
            "lane.diag.last_latency_ms",
            format!("{:.1}", self.diag_last_latency_ms),
        );
        bb.set(
            "lane.diag.frames_dropped_full",
            self.diag_frames_dropped_full.to_string(),
        );
        bb.set("lane.diag.load_ok", self.load_ok.to_string());
        bb.set("lane.diag.enabled", self.load_ok.to_string()); // backwards-compat alias
    }
}

// ---------------------------------------------------------------------------
// Plugin trait
// ---------------------------------------------------------------------------

impl Plugin for LaneDetectionPlugin {
    fn name(&self) -> &str {
        "lane-detection"
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
      "description": "Path to UFLD v2 ONNX model."
    },
    "exist_thresh": {
      "type": "number",
      "minimum": 0.1,
      "maximum": 1.0,
      "description": "Existence softmax threshold (default 0.5)."
    },
    "bias_window_size": {
      "type": "integer",
      "minimum": 1,
      "description": "Frames in the moving-median bias window (default 100 = ~10 s at 10 Hz)."
    },
    "bias_warmup_frames": {
      "type": "integer",
      "minimum": 0,
      "description": "Frames to collect before bias correction activates (default 50)."
    }
  }
}"#
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        // Read optional config overrides from blackboard (set by TOML loader).
        if let Some(p) = ctx.blackboard.get("lane_detection.model_path") {
            self.model_path = PathBuf::from(p);
        }
        if let Some(t) = ctx.blackboard.get_f64("lane_detection.exist_thresh") {
            self.exist_thresh = (t as f32).clamp(0.1, 1.0);
        }

        // Bias estimator config — rebuild with new params if overridden.
        let mut window = self.bias.window_size;
        let mut warmup = self.bias.warmup_frames;
        if let Some(v) = ctx.blackboard.get_f64("lane_detection.bias_window_size") {
            window = (v as usize).max(1);
        }
        if let Some(v) = ctx.blackboard.get_f64("lane_detection.bias_warmup_frames") {
            warmup = v as usize;
        }
        if window != self.bias.window_size || warmup != self.bias.warmup_frames {
            self.bias = bias::BiasEstimator::new(window, warmup);
        }

        #[cfg(any(feature = "onnx-cpu", feature = "onnx-directml"))]
        match self.try_init_worker(ctx) {
            Ok(()) => {
                self.load_ok = true;
                ctx_info!(
                    ctx, target: LOG_TARGET,
                    "lane-detection loaded: col_grids={} row_anchors={} num_lanes={}",
                    self.col_grids, self.row_anchors, self.num_lanes
                );
            }
            Err(e) => {
                ctx_error!(ctx, target: LOG_TARGET, "lane-detection init failed: {e}");
                self.load_ok = false;
            }
        }

        #[cfg(not(any(feature = "onnx-cpu", feature = "onnx-directml")))]
        {
            ctx_warn!(ctx, target: LOG_TARGET,
                "lane-detection: no onnx feature compiled — build with --features onnx-cpu");
        }

        ctx.blackboard
            .set("lane.diag.load_ok", self.load_ok.to_string());
        ctx.blackboard
            .set("lane.diag.enabled", self.load_ok.to_string()); // backwards-compat alias
    }

    fn on_unload(&mut self) {
        #[cfg(any(feature = "onnx-cpu", feature = "onnx-directml"))]
        {
            self.worker = None; // Drop → closes channel → worker exits
        }
        tracing::info!(target: LOG_TARGET, "lane-detection unloaded");
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
        self.diag_tick_count += 1;

        // 1. Always drain results first (cheap when empty).
        #[cfg(any(feature = "onnx-cpu", feature = "onnx-directml"))]
        self.publish_latest_result(ctx);

        if !self.load_ok {
            self.publish_diag(ctx);
            return;
        }

        // 2. Fetch frame.
        let store = match ctx.frame_store() {
            Some(s) => s,
            None => {
                self.publish_diag(ctx);
                return;
            }
        };
        let frame = match store.get(FRAME_KEY) {
            Some(f) => f,
            None => {
                self.publish_diag(ctx);
                return;
            }
        };
        if Some(frame.id) == self.last_processed_frame_id {
            self.publish_diag(ctx);
            return;
        }

        ctx_trace!(ctx, target: LOG_TARGET,
            "frame id={} {}×{} jpeg={}B",
            frame.id, frame.width, frame.height, frame.jpeg.len());

        // 3. Lazy JPEG → RGB8 decode (cached in SharedFrame OnceLock).
        let rgb: &Arc<Vec<u8>> = match frame.get_or_try_init_rgb8(|| {
            let img = image::load_from_memory_with_format(&frame.jpeg, image::ImageFormat::Jpeg)
                .map_err(|e| format!("JPEG decode: {e}"))?;
            let rgb = img.to_rgb8();
            Ok::<_, String>(Arc::new(rgb.into_raw()))
        }) {
            Ok(r) => r,
            Err(e) => {
                ctx_warn!(ctx, target: LOG_TARGET, "JPEG decode failed frame={}: {e}", frame.id);
                self.last_processed_frame_id = Some(frame.id);
                self.publish_diag(ctx);
                return;
            }
        };

        // 4. Submit to worker (non-blocking, drop-oldest on full).
        #[cfg(any(feature = "onnx-cpu", feature = "onnx-directml"))]
        self.submit_frame(InferenceJob {
            rgb: Arc::clone(rgb),
            orig_w: frame.width,
            orig_h: frame.height,
            submitted_at: Instant::now(),
        });

        self.last_processed_frame_id = Some(frame.id);
        self.publish_diag(ctx);
    }
}

truckpilot_plugin_api::export_plugin!(LaneDetectionPlugin);

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use truckpilot_plugin_api::{SharedBlackboard, SharedFrame, SharedFrameStore};

    fn one_px_jpeg() -> Vec<u8> {
        let img = image::RgbImage::from_pixel(1, 1, image::Rgb([200u8, 150, 100]));
        let mut buf = std::io::Cursor::new(Vec::new());
        img.write_to(&mut buf, image::ImageFormat::Jpeg)
            .expect("encode");
        buf.into_inner()
    }

    fn make_frame(id: u64, jpeg: Vec<u8>) -> Arc<SharedFrame> {
        Arc::new(SharedFrame::new(id, 0, 1, 1, Arc::new(jpeg)))
    }

    fn ctx_with_store() -> (PluginContext, Arc<SharedFrameStore>) {
        let store = Arc::new(SharedFrameStore::new());
        let ctx = PluginContext::new("lane-detection", SharedBlackboard::new())
            .with_frame_store(store.clone());
        (ctx, store)
    }

    fn ctx_no_store() -> PluginContext {
        PluginContext::new("lane-detection", SharedBlackboard::new())
    }

    // ---- default state ---------------------------------------------------

    #[test]
    fn default_load_ok_false() {
        let p = LaneDetectionPlugin::default();
        assert!(!p.load_ok);
    }

    // ---- on_load without model sets diag keys to false -------------------

    #[test]
    fn on_load_no_model_sets_diag_false() {
        let mut p = LaneDetectionPlugin::default();
        let ctx = ctx_no_store();
        p.on_load(&ctx);
        // Without a valid model file load_ok stays false; diag reflects that.
        assert_eq!(
            ctx.blackboard.get("lane.diag.load_ok").as_deref(),
            Some("false")
        );
        // backwards-compat alias matches load_ok
        assert_eq!(
            ctx.blackboard.get("lane.diag.enabled").as_deref(),
            Some("false")
        );
        assert!(!p.load_ok);
    }

    // ---- tick no-ops when load_ok=false ----------------------------------

    #[test]
    fn tick_noop_when_load_failed() {
        let mut p = LaneDetectionPlugin::default();
        let ctx = ctx_no_store();
        p.on_load(&ctx); // load_ok stays false — no model present
        let mut out = ControlOutput::default();
        p.tick(None, &mut out, &ctx);
        assert_eq!(p.last_processed_frame_id, None);
    }

    #[test]
    fn tick_noop_when_no_frame_store() {
        let mut p = LaneDetectionPlugin::default();
        let ctx = ctx_no_store();
        // Force load_ok=true to bypass the load gate
        p.load_ok = true;
        let mut out = ControlOutput::default();
        p.tick(None, &mut out, &ctx);
        assert_eq!(p.last_processed_frame_id, None);
    }

    #[test]
    fn tick_marks_frame_processed() {
        let mut p = LaneDetectionPlugin::default();
        let (ctx, store) = ctx_with_store();
        p.load_ok = true;
        store.set(FRAME_KEY, make_frame(42, one_px_jpeg()));
        let mut out = ControlOutput::default();
        p.tick(None, &mut out, &ctx);
        assert_eq!(p.last_processed_frame_id, Some(42));
    }

    #[test]
    fn tick_skips_duplicate_frame_id() {
        let mut p = LaneDetectionPlugin::default();
        let (ctx, store) = ctx_with_store();
        p.load_ok = true;

        let frame = make_frame(7, one_px_jpeg());
        store.set(FRAME_KEY, Arc::clone(&frame));
        let mut out = ControlOutput::default();
        p.tick(None, &mut out, &ctx);

        let dup = make_frame(7, one_px_jpeg());
        store.set(FRAME_KEY, Arc::clone(&dup));
        p.tick(None, &mut out, &ctx);
        assert!(
            dup.decoded_rgb8().is_none(),
            "duplicate frame must not be decoded"
        );
    }

    #[test]
    fn tick_handles_corrupt_jpeg_gracefully() {
        let mut p = LaneDetectionPlugin::default();
        let (ctx, store) = ctx_with_store();
        p.load_ok = true;
        store.set(FRAME_KEY, make_frame(99, vec![0u8; 16]));
        let mut out = ControlOutput::default();
        p.tick(None, &mut out, &ctx);
        assert_eq!(p.last_processed_frame_id, Some(99));
    }

    #[test]
    fn diag_tick_count_increments() {
        let mut p = LaneDetectionPlugin::default();
        let ctx = ctx_no_store();
        let mut out = ControlOutput::default();
        for i in 1..=3u64 {
            p.tick(None, &mut out, &ctx);
            assert_eq!(p.diag_tick_count, i);
        }
    }
}
