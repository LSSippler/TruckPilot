//! TruckPilot VMM-3/4/5 — minimap-vision plugin.
//!
//! Reads the `TruckPilotMinimapLine` SHM region written by the Python
//! minimap sidecar, converts pixel-space route-line points to ETS2
//! world-space, generates Catmull-Rom HermiteSegments, applies a
//! 3-frame temporal confidence filter, and publishes everything to the
//! SharedBlackboard for the lane-follower (VMM-6) to consume as a
//! third spline source.
//!
//! ## Blackboard keys written
//!
//! | Key | Format | Description |
//! |-----|--------|-------------|
//! | `minimap.healthy` | `"true"/"false"` | SHM producer reachable |
//! | `minimap.detected` | `"true"/"false"` | route line detected this tick |
//! | `minimap.confidence` | f32 `[0,1]` | temporal confidence |
//! | `minimap.spline_points_count` | u32 | segments in spline JSON |
//! | `minimap.last_capture_ms` | u64 | producer timestamp (UNIX ms) |
//! | `minimap.spline_json` | JSON | `[HermiteSegment, …]` |
//! | `minimap.source.last_error` | string | set on error, removed on ok |
//!
//! ## Pixel → world transform
//!
//! The minimap is assumed north-up with the truck at the ROI centre:
//! ```text
//! world_x = truck_x + (px - roi_cx) * meters_per_pixel
//! world_z = truck_z + (py - roi_cy) * meters_per_pixel
//! world_y = truck_y   (flat approximation)
//! ```
//! `meters_per_pixel` must be calibrated per monitor / zoom level
//! (default 10.0 m/px; typical ETS2 minimap ≈ 5–15 m/px).

pub mod shm_reader;

use std::collections::VecDeque;

use truckpilot_plugin_api::{
    ctx_info, ctx_warn, ControlOutput, Plugin, PluginContext, Telemetry, TickPhase,
};

use shm_reader::{map_shm, read_frame, ReadOutcome, BUFFER_BYTES, DEFAULT_SHM_NAME};

// ── constants ─────────────────────────────────────────────────────────────────

const REMAP_INTERVAL_TICKS: u64 = 10;
const MISS_THRESHOLD: u32 = 30;
/// Temporal confidence window (VMM-5): keep N recent confidence values.
const TEMPORAL_WINDOW: usize = 3;
/// Stale frame threshold: drop frames older than this (ms).
const DEFAULT_STALE_AFTER_MS: u64 = 500;

// ── settings ──────────────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct Settings {
    pub shm_name: String,
    pub stale_after_ms: u64,
    /// ETS2 minimap scale: world meters per minimap pixel.
    pub meters_per_pixel: f32,
    /// ROI dimensions (pixels) — used to compute roi centre.
    pub roi_w: u32,
    pub roi_h: u32,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            shm_name: DEFAULT_SHM_NAME.to_string(),
            stale_after_ms: DEFAULT_STALE_AFTER_MS,
            meters_per_pixel: 10.0,
            roi_w: 200,
            roi_h: 200,
        }
    }
}

// ── Vec3 inline (avoid map-parser dep in this plugin) ────────────────────────

#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
struct Vec3 {
    x: f32,
    y: f32,
    z: f32,
}

impl Vec3 {
    fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
    fn scale(self, s: f32) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
    fn length(self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }
}

// ── HermiteSegment inline (matches map-parser layout for JSON compat) ─────────

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
struct HermiteSegment {
    p0: Vec3,
    p1: Vec3,
    m0: Vec3,
    m1: Vec3,
    length_m: f32,
    from_uid: u64,
    to_uid: u64,
    edge_uid: u64,
}

// ── spline generation (VMM-4 Catmull-Rom) ────────────────────────────────────

fn catmull_tangent(prev: Vec3, next: Vec3) -> Vec3 {
    next.sub(prev).scale(0.5)
}

fn build_hermite_segments(world_pts: &[Vec3]) -> Vec<HermiteSegment> {
    let n = world_pts.len();
    if n < 2 {
        return vec![];
    }

    // Compute tangents using Catmull-Rom.
    let mut tangents = Vec::with_capacity(n);
    for i in 0..n {
        let t = if i == 0 {
            world_pts[1].sub(world_pts[0])
        } else if i == n - 1 {
            world_pts[n - 1].sub(world_pts[n - 2])
        } else {
            catmull_tangent(world_pts[i - 1], world_pts[i + 1])
        };
        tangents.push(t);
    }

    let mut segs = Vec::with_capacity(n - 1);
    for i in 0..n - 1 {
        let p0 = world_pts[i];
        let p1 = world_pts[i + 1];
        let len = p1.sub(p0).length();
        segs.push(HermiteSegment {
            p0,
            p1,
            m0: tangents[i],
            m1: tangents[i + 1],
            length_m: len,
            from_uid: 0,
            to_uid: 0,
            edge_uid: 0,
        });
    }
    segs
}

// ── pixel → world transform ───────────────────────────────────────────────────

fn pixel_to_world(
    px: f32,
    py: f32,
    roi_cx: f32,
    roi_cy: f32,
    truck_x: f64,
    truck_y: f64,
    truck_z: f64,
    meters_per_pixel: f32,
) -> Vec3 {
    let dx = (px - roi_cx) * meters_per_pixel;
    let dz = (py - roi_cy) * meters_per_pixel; // south = positive Z
    Vec3::new(
        truck_x as f32 + dx,
        truck_y as f32,
        truck_z as f32 + dz,
    )
}

// ── temporal confidence (VMM-5) ───────────────────────────────────────────────

struct TemporalConfidence {
    window: VecDeque<f32>,
}

impl TemporalConfidence {
    fn new() -> Self {
        Self { window: VecDeque::with_capacity(TEMPORAL_WINDOW) }
    }

    fn push(&mut self, raw: f32) -> f32 {
        if self.window.len() >= TEMPORAL_WINDOW {
            self.window.pop_front();
        }
        self.window.push_back(raw);
        // Conservative: minimum over the window.
        self.window.iter().cloned().fold(f32::INFINITY, f32::min)
    }

    fn reset(&mut self) {
        self.window.clear();
    }
}

// ── plugin struct ─────────────────────────────────────────────────────────────

#[derive(Default)]
pub struct MinimapVisionPlugin {
    settings: Settings,
    shm_buf: Option<&'static [u8]>,
    last_published_seq: u64,
    consecutive_misses: u32,
    tick_counter: u64,
    has_published: bool,
    temporal: TemporalConfidence,
}

impl MinimapVisionPlugin {
    pub fn with_settings(settings: Settings) -> Self {
        Self { settings, temporal: TemporalConfidence::new(), ..Self::default() }
    }

    #[doc(hidden)]
    pub fn inject_buffer(&mut self, buf: &'static [u8]) {
        self.shm_buf = Some(buf);
    }

    fn try_map(&mut self, ctx: &PluginContext) {
        match map_shm(&self.settings.shm_name, BUFFER_BYTES) {
            Ok(buf) => {
                ctx_info!(
                    ctx,
                    target: "truckpilot_plugin_minimap_vision",
                    "mapped SHM '{}' ({} bytes)",
                    self.settings.shm_name,
                    BUFFER_BYTES,
                );
                self.shm_buf = Some(buf);
                ctx.blackboard.remove("minimap.source.last_error");
            }
            Err(e) => {
                ctx_warn!(
                    ctx,
                    target: "truckpilot_plugin_minimap_vision",
                    "SHM not available: {e}"
                );
                ctx.blackboard.set("minimap.source.last_error", e);
            }
        }
    }

    fn publish_unhealthy(&self, ctx: &PluginContext) {
        ctx.blackboard.set("minimap.healthy", "false");
        ctx.blackboard.set("minimap.detected", "false");
    }

    fn now_ms() -> u64 {
        use std::time::{SystemTime, UNIX_EPOCH};
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .unwrap_or(0)
    }
}

impl Default for TemporalConfidence {
    fn default() -> Self {
        Self::new()
    }
}

impl Plugin for MinimapVisionPlugin {
    fn name(&self) -> &str {
        "minimap-vision"
    }

    fn version(&self) -> &str {
        "0.1.0"
    }

    fn settings_schema(&self) -> &str {
        r#"{
  "type": "object",
  "properties": {
    "shm_name":          { "type": "string",  "default": "TruckPilotMinimapLine" },
    "stale_after_ms":    { "type": "integer", "default": 500,  "minimum": 1 },
    "meters_per_pixel":  { "type": "number",  "default": 10.0, "minimum": 0.1 },
    "roi_w":             { "type": "integer", "default": 200 },
    "roi_h":             { "type": "integer", "default": 200 }
  }
}"#
    }

    fn default_phase(&self) -> TickPhase {
        TickPhase::PhaseB // 10 Hz, same as sidecar
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        ctx_info!(
            ctx,
            target: "truckpilot_plugin_minimap_vision",
            "loaded — shm='{}' stale_after_ms={} meters_per_pixel={}",
            self.settings.shm_name,
            self.settings.stale_after_ms,
            self.settings.meters_per_pixel,
        );

        ctx.blackboard.set("minimap.healthy", "false");
        ctx.blackboard.set("minimap.detected", "false");
        ctx.blackboard.set("minimap.confidence", "0");
        ctx.blackboard.set("minimap.spline_points_count", "0");
        ctx.blackboard.set("minimap.last_capture_ms", "0");
        ctx.blackboard.set("minimap.spline_json", "[]");

        self.try_map(ctx);
    }

    fn on_unload(&mut self) {
        self.shm_buf = None;
        self.last_published_seq = 0;
        self.consecutive_misses = 0;
        self.tick_counter = 0;
        self.has_published = false;
        self.temporal.reset();
        tracing::info!(target: "truckpilot_plugin_minimap_vision", "unloaded");
    }

    fn tick(
        &mut self,
        telemetry: Option<&Telemetry>,
        _output: &mut ControlOutput,
        ctx: &PluginContext,
    ) {
        self.tick_counter = self.tick_counter.wrapping_add(1);

        // Re-attempt SHM mapping while sidecar is absent.
        if self.shm_buf.is_none() {
            if self.tick_counter % REMAP_INTERVAL_TICKS == 0 {
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
            ReadOutcome::Frame(frame) => {
                self.consecutive_misses = 0;
                ctx.blackboard.set("minimap.healthy", "true");
                ctx.blackboard.remove("minimap.source.last_error");

                // Duplicate frame — sidecar hasn't advanced yet.
                if frame.header.seq == self.last_published_seq {
                    ctx.blackboard.set("minimap.detected", "false");
                    return;
                }

                // Stale check.
                let now_ms = Self::now_ms();
                let apparent_age_ms = now_ms.saturating_sub(frame.header.timestamp_ms);
                if apparent_age_ms > self.settings.stale_after_ms {
                    ctx.blackboard.set("minimap.detected", "false");
                    ctx.blackboard.set("minimap.last_capture_ms", frame.header.timestamp_ms.to_string());
                    self.last_published_seq = frame.header.seq;
                    return;
                }

                // Temporal confidence (VMM-5).
                let temporal_conf = self.temporal.push(frame.header.confidence);

                let n_points = frame.points.len();
                let detected = n_points >= 2 && temporal_conf > 0.1;

                ctx.blackboard.set("minimap.detected", if detected { "true" } else { "false" });
                ctx.blackboard.set("minimap.confidence", format!("{temporal_conf:.4}"));
                ctx.blackboard.set("minimap.last_capture_ms", frame.header.timestamp_ms.to_string());

                if detected {
                    // Pixel → world transform (VMM-3).
                    let roi_cx = self.settings.roi_w as f32 / 2.0;
                    let roi_cy = self.settings.roi_h as f32 / 2.0;
                    let (tx, ty, tz) = telemetry
                        .map(|t| (t.position[0], t.position[1], t.position[2]))
                        .unwrap_or((0.0, 0.0, 0.0));

                    let world_pts: Vec<Vec3> = frame
                        .points
                        .iter()
                        .map(|(px, py)| {
                            pixel_to_world(
                                *px, *py,
                                roi_cx, roi_cy,
                                tx, ty, tz,
                                self.settings.meters_per_pixel,
                            )
                        })
                        .collect();

                    // Catmull-Rom HermiteSegments (VMM-4).
                    let segs = build_hermite_segments(&world_pts);
                    let seg_count = segs.len();

                    let json = serde_json::to_string(&segs).unwrap_or_else(|_| "[]".into());
                    ctx.blackboard.set("minimap.spline_json", json);
                    ctx.blackboard.set("minimap.spline_points_count", seg_count.to_string());
                } else {
                    ctx.blackboard.set("minimap.spline_json", "[]");
                    ctx.blackboard.set("minimap.spline_points_count", "0");
                }

                self.last_published_seq = frame.header.seq;
                self.has_published = true;
            }

            ReadOutcome::NoFrame => {
                // Sidecar hasn't published yet — not an error.
                ctx.blackboard.set("minimap.healthy", "true");
                ctx.blackboard.set("minimap.detected", "false");
            }

            ReadOutcome::WriterBusy => {
                self.consecutive_misses = self.consecutive_misses.saturating_add(1);
                ctx.blackboard.set("minimap.detected", "false");
                if self.consecutive_misses >= MISS_THRESHOLD {
                    ctx.blackboard.set("minimap.healthy", "false");
                    ctx.blackboard.set("minimap.source.last_error", "sequence-lock exhausted");
                }
            }

            ReadOutcome::InvalidHeader => {
                ctx_warn!(
                    ctx,
                    target: "truckpilot_plugin_minimap_vision",
                    "invalid SHM header — magic/version mismatch"
                );
                ctx.blackboard.set("minimap.healthy", "false");
                ctx.blackboard.set("minimap.detected", "false");
                ctx.blackboard.set("minimap.source.last_error", "invalid header");
            }

            ReadOutcome::PayloadOverflow => {
                ctx_warn!(
                    ctx,
                    target: "truckpilot_plugin_minimap_vision",
                    "SHM payload overflow — point_count exceeds MAX_POINTS"
                );
                ctx.blackboard.set("minimap.healthy", "false");
                ctx.blackboard.set("minimap.detected", "false");
                ctx.blackboard.set("minimap.source.last_error", "payload overflow");
            }
        }
    }
}

truckpilot_plugin_api::export_plugin!(MinimapVisionPlugin);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn catmull_rom_interior_tangent() {
        let p0 = Vec3::new(0.0, 0.0, 0.0);
        let p2 = Vec3::new(2.0, 0.0, 0.0);
        let t = catmull_tangent(p0, p2);
        assert!((t.x - 1.0).abs() < 1e-6, "should be midpoint of chord");
    }

    #[test]
    fn build_segments_from_three_points() {
        let pts = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(10.0, 0.0, 0.0),
            Vec3::new(20.0, 0.0, 5.0),
        ];
        let segs = build_hermite_segments(&pts);
        assert_eq!(segs.len(), 2);
        assert!((segs[0].length_m - 10.0).abs() < 0.01);
    }

    #[test]
    fn pixel_to_world_center_is_truck_pos() {
        let w = pixel_to_world(100.0, 100.0, 100.0, 100.0, 500.0, 0.0, 1000.0, 10.0);
        assert!((w.x - 500.0).abs() < 0.01);
        assert!((w.z - 1000.0).abs() < 0.01);
    }

    #[test]
    fn temporal_confidence_min_of_window() {
        let mut tc = TemporalConfidence::new();
        tc.push(0.9);
        tc.push(0.8);
        let conf = tc.push(0.5);
        assert!((conf - 0.5).abs() < 0.001, "should be min of 0.9, 0.8, 0.5");
    }

    #[test]
    fn temporal_confidence_resets_old_values() {
        let mut tc = TemporalConfidence::new();
        tc.push(0.1);
        tc.push(0.9);
        tc.push(0.9);
        // Window is now [0.9, 0.9, 0.9] — old 0.1 dropped
        let conf = tc.push(0.9);
        assert!(conf > 0.8, "should be ~0.9 after old 0.1 is gone: {conf}");
    }
}
