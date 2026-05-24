//! Lane-Follower plugin — Phase 1c Pure-Pursuit Steering.
//!
//! Loads a SplineIndex from `graph.json` at startup and queries it every
//! 50 Hz tick to publish diagnostic Blackboard keys.  In `Active` mode the
//! plugin additionally emits a `ControlRequest` with a Pure-Pursuit steering
//! command when `lane_follower.status == "ok"`.
//!
//! ## Mode
//! Set `plugin.lane-follower.mode` on the Blackboard to `"active"` to
//! switch from `Observer` (diagnostic-only) to `Active`.  Default: `Observer`.
//!
//! ## Graph path
//! Override the default `"graph.json"` path by setting
//! `plugin.lane-follower.graph_path` **before** `on_load` is called.
//!
//! ## Blackboard keys written per tick
//! | Key | Format | Description |
//! |-----|--------|-------------|
//! | `lane_follower.active` | `"true"/"false"` | mode == Active |
//! | `lane_follower.mode` | `"observer"/"active"` | current mode |
//! | `lane_follower.truck_x` | f64 metres | truck ETS2 X |
//! | `lane_follower.truck_y` | f64 metres | truck ETS2 Y (up) |
//! | `lane_follower.truck_z` | f64 metres | truck ETS2 Z |
//! | `lane_follower.truck_heading_deg` | f32 degrees | truck heading (0=North, CW) |
//! | `lane_follower.nearest_seg_idx` | usize | global segment index |
//! | `lane_follower.nearest_seg_dist_m` | f32 metres | Euclidean distance to road |
//! | `lane_follower.nearest_seg_x` | f32 metres | nearest spline point X |
//! | `lane_follower.nearest_seg_z` | f32 metres | nearest spline point Z |
//! | `lane_follower.nearest_seg_t` | f32 [0,1] | Hermite parameter |
//! | `lane_follower.heading_deg` | f32 degrees | road heading at nearest point |
//! | `lane_follower.heading_diff_deg` | f32 degrees | |truck − road heading| in [0,180] |
//! | `lane_follower.tick_count` | u64 | per-tick counter (stall detection) |
//! | `lane_follower.status` | string | `ok` / `dist_warn` / `heading_warn` / … |
//! | `lane_follower.steering_cmd` | f64 [-1,1] | raw Pure-Pursuit command (status==ok only) |
//! | `lane_follower.steering_curvature` | f64 | geometric curvature κ = 2·y / d² |
//! | `lane_follower.steering_filtered` | f64 [-1,1] | EMA-filtered steering (α=0.3) |

mod pure_pursuit;

use std::collections::HashMap;
use std::collections::VecDeque;

use truckpilot_map_parser::{
    arc_length::{build_all_luts, build_forward_adjacency, lookahead, ArcLengthLUT, LOOKAHEAD_MAX_HOPS},
    graph::MapGraph,
    spline::build_splines,
    spline::Vec3,
    spline_index::{build_index, SplineIndex},
};
use truckpilot_plugin_api::{
    ctx_info, ctx_warn, ControlOutput, ControlRequest, Plugin, PluginContext, Telemetry, TickPhase,
};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const DEFAULT_GRAPH_PATH: &str = "graph.json";

/// R-tree candidate count for nearest_with_projection.
const CANDIDATES: usize = 8;

/// Distance threshold above which status is set to `dist_warn`.
const DIST_WARN_M: f32 = 50.0;

/// Heading difference threshold above which status is set to `heading_warn`.
const HEADING_WARN_DEG: f32 = 90.0;

/// Lookahead distance for arc-length-based forward point computation.
const LOOKAHEAD_DIST_M: f32 = 15.0;

/// Ring-buffer size for lookahead stability metric (ticks = 1 second at 50 Hz).
const STABILITY_WINDOW: usize = 50;

/// EMA smoothing factor for the steering output (higher = faster response).
const STEERING_EMA_ALPHA: f64 = 0.3;

// ---------------------------------------------------------------------------
// Mode
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq, Eq, Default)]
enum LaneFollowerMode {
    #[default]
    Observer,
    Active,
}

impl LaneFollowerMode {
    fn from_bb(ctx: &PluginContext) -> Self {
        match ctx.blackboard.get("plugin.lane-follower.mode").as_deref() {
            Some("active") => Self::Active,
            _ => Self::Observer,
        }
    }

    fn as_str(&self) -> &'static str {
        match self {
            Self::Observer => "observer",
            Self::Active => "active",
        }
    }
}

// ---------------------------------------------------------------------------
// Plugin struct
// ---------------------------------------------------------------------------

#[derive(Default)]
pub struct LaneFollowerPlugin {
    index: Option<SplineIndex>,
    luts: Vec<ArcLengthLUT>,
    forward_adj: HashMap<u64, Vec<usize>>,
    lookahead_seg_history: VecDeque<usize>,
    mode: LaneFollowerMode,
    tick_count: u64,
    /// Last raw Pure-Pursuit steering command; `None` when status ≠ ok.
    last_steering_cmd: Option<f64>,
    /// EMA-filtered steering state.
    steering_ema: f64,
}

impl LaneFollowerPlugin {
    fn load_index(&mut self, path: &str, ctx: &PluginContext) {
        let data = match std::fs::read(path) {
            Ok(d) => d,
            Err(e) => {
                ctx_warn!(ctx, "lane-follower: cannot read '{}': {}", path, e);
                return;
            }
        };
        let graph: MapGraph = match serde_json::from_slice(&data) {
            Ok(g) => g,
            Err(e) => {
                ctx_warn!(ctx, "lane-follower: cannot parse graph '{}': {}", path, e);
                return;
            }
        };
        let (segments, stats) = build_splines(&graph);
        ctx_info!(
            ctx,
            "lane-follower: {} segments built, {} skipped (missing node)",
            stats.total_segments,
            stats.skipped_missing_node
        );
        let t0 = std::time::Instant::now();
        let luts = build_all_luts(&segments);
        let forward_adj = build_forward_adjacency(&segments);
        let lut_ms = t0.elapsed().as_millis();
        let lut_kb = (luts.len() * std::mem::size_of::<ArcLengthLUT>()) as f32 / 1024.0;
        ctx_info!(
            ctx,
            "lane-follower: LUT built in {}ms, {:.1}KB ({} entries)",
            lut_ms,
            lut_kb,
            luts.len()
        );
        self.luts = luts;
        self.forward_adj = forward_adj;
        self.index = Some(build_index(segments));
    }
}

// ---------------------------------------------------------------------------
// Plugin trait
// ---------------------------------------------------------------------------

impl Plugin for LaneFollowerPlugin {
    fn name(&self) -> &str {
        "lane-follower"
    }

    fn version(&self) -> &str {
        "0.1.0"
    }

    fn settings_schema(&self) -> &str {
        "{}"
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        let path = ctx
            .blackboard
            .get("plugin.lane-follower.graph_path")
            .unwrap_or_else(|| DEFAULT_GRAPH_PATH.to_string());
        self.load_index(&path, ctx);
        self.mode = LaneFollowerMode::from_bb(ctx);
        ctx_info!(
            ctx,
            "lane-follower: loaded (mode={}, index={})",
            self.mode.as_str(),
            if self.index.is_some() { "ok" } else { "none" }
        );
    }

    fn on_unload(&mut self) {
        self.index = None;
        self.luts = Vec::new();
        self.forward_adj = HashMap::new();
        self.lookahead_seg_history = VecDeque::new();
        self.tick_count = 0;
        self.last_steering_cmd = None;
        self.steering_ema = 0.0;
    }

    fn tick(&mut self, telemetry: Option<&Telemetry>, _output: &mut ControlOutput, ctx: &PluginContext) {
        // Reset per-tick steering so tick_request() sees None on any early return.
        self.last_steering_cmd = None;

        self.mode = LaneFollowerMode::from_bb(ctx);

        ctx.blackboard.set("lane_follower.mode", self.mode.as_str());
        ctx.blackboard.set(
            "lane_follower.active",
            if self.mode == LaneFollowerMode::Active { "true" } else { "false" },
        );

        let Some(tel) = telemetry else {
            ctx.blackboard.set("lane_follower.status", "no_telemetry");
            return;
        };

        let truck_x = tel.position[0];
        let truck_y = tel.position[1];
        let truck_z = tel.position[2];
        ctx.blackboard.set("lane_follower.truck_x", format!("{truck_x:.3}"));
        ctx.blackboard.set("lane_follower.truck_y", format!("{truck_y:.3}"));
        ctx.blackboard.set("lane_follower.truck_z", format!("{truck_z:.3}"));

        let Some(index) = &self.index else {
            ctx.blackboard.set("lane_follower.status", "no_index");
            return;
        };

        // ETS2 SDK heading is 0..1 CCW from North; convert to CW degrees (0=N, 90=E).
        let truck_heading_deg = ((-tel.heading) * 360.0).rem_euclid(360.0) as f32;
        let query = Vec3::new(truck_x as f32, tel.position[1] as f32, truck_z as f32);
        let Some(hit) = index.nearest_with_heading_filter(query, truck_heading_deg, CANDIDATES) else {
            ctx.blackboard.set("lane_follower.status", "no_hit");
            return;
        };

        ctx.blackboard
            .set("lane_follower.nearest_seg_idx", hit.segment_idx.to_string());
        ctx.blackboard
            .set("lane_follower.nearest_seg_dist_m", format!("{:.2}", hit.dist_m));
        ctx.blackboard
            .set("lane_follower.nearest_seg_t", format!("{:.4}", hit.t));
        ctx.blackboard
            .set("lane_follower.nearest_seg_x", format!("{:.3}", hit.point_on_curve.x));
        ctx.blackboard
            .set("lane_follower.nearest_seg_z", format!("{:.3}", hit.point_on_curve.z));
        ctx.blackboard
            .set("lane_follower.heading_deg", format!("{:.2}", hit.heading_deg));

        ctx.blackboard
            .set("lane_follower.truck_heading_deg", format!("{truck_heading_deg:.2}"));
        ctx.blackboard
            .set("lane_follower.heading_filter_applied", hit.heading_filter_applied.to_string());
        let heading_diff = angular_diff_deg(truck_heading_deg, hit.heading_deg);
        ctx.blackboard
            .set("lane_follower.heading_diff_deg", format!("{heading_diff:.2}"));

        // Per-tick counter — never resets, monotonic. Catches telemetry stalls.
        self.tick_count += 1;
        ctx.blackboard
            .set("lane_follower.tick_count", self.tick_count.to_string());

        // Lookahead — computed before warn-checks so keys are always present after a hit
        if !self.luts.is_empty() {
            if let Some(la) = lookahead(
                hit.segment_idx,
                hit.t,
                LOOKAHEAD_DIST_M,
                &self.forward_adj,
                index.segments.as_slice(),
                &self.luts,
            ) {
                ctx.blackboard
                    .set("lane_follower.lookahead_x", format!("{:.3}", la.point.x));
                ctx.blackboard
                    .set("lane_follower.lookahead_z", format!("{:.3}", la.point.z));
                ctx.blackboard
                    .set("lane_follower.lookahead_seg_idx", la.seg_idx.to_string());
                ctx.blackboard
                    .set("lane_follower.lookahead_remaining_m", format!("{:.3}", la.remaining_dist_m));
                let la_status = if la.remaining_dist_m == 0.0 {
                    "ok"
                } else if la.iteration_count >= LOOKAHEAD_MAX_HOPS {
                    "iteration_limit"
                } else {
                    "dead_end"
                };
                ctx.blackboard.set("lane_follower.lookahead_status", la_status);

                let dx = la.point.x - query.x;
                let dz = la.point.z - query.z;
                let heading_to_la = dx.atan2(-dz).to_degrees().rem_euclid(360.0);
                ctx.blackboard
                    .set("lane_follower.heading_to_lookahead_deg", format!("{:.2}", heading_to_la));

                self.lookahead_seg_history.push_back(la.seg_idx);
                if self.lookahead_seg_history.len() > STABILITY_WINDOW {
                    self.lookahead_seg_history.pop_front();
                }
                let jump_count = self
                    .lookahead_seg_history
                    .iter()
                    .zip(self.lookahead_seg_history.iter().skip(1))
                    .filter(|(a, b)| a != b)
                    .count();
                ctx.blackboard
                    .set("lane_follower.lookahead_seg_jump_count", jump_count.to_string());
                ctx.blackboard.set(
                    "lane_follower.lookahead_stable",
                    if jump_count < 3 { "true" } else { "false" },
                );
            }
        }

        if hit.dist_m > DIST_WARN_M {
            ctx_warn!(
                ctx,
                "lane-follower: dist_warn {:.1}m > {}m threshold",
                hit.dist_m,
                DIST_WARN_M
            );
            ctx.blackboard.set("lane_follower.status", "dist_warn");
            self.steering_ema *= 1.0 - STEERING_EMA_ALPHA;
            ctx.blackboard.set("lane_follower.steering_cmd", "0.0000");
            ctx.blackboard.set("lane_follower.steering_curvature", "0.000000");
            ctx.blackboard.set("lane_follower.steering_filtered", format!("{:.4}", self.steering_ema));
            return;
        }

        if heading_diff > HEADING_WARN_DEG {
            ctx_warn!(
                ctx,
                "lane-follower: heading_warn diff={:.1}° > {}°",
                heading_diff,
                HEADING_WARN_DEG
            );
            ctx.blackboard.set("lane_follower.status", "heading_warn");
            self.steering_ema *= 1.0 - STEERING_EMA_ALPHA;
            ctx.blackboard.set("lane_follower.steering_cmd", "0.0000");
            ctx.blackboard.set("lane_follower.steering_curvature", "0.000000");
            ctx.blackboard.set("lane_follower.steering_filtered", format!("{:.4}", self.steering_ema));
            return;
        }

        ctx.blackboard.set("lane_follower.status", "ok");

        // Pure-Pursuit steering — only computed when a valid lookahead point exists.
        if let (Some(lx), Some(lz)) = (
            ctx.blackboard.get_f64("lane_follower.lookahead_x"),
            ctx.blackboard.get_f64("lane_follower.lookahead_z"),
        ) {
            let cmd = pure_pursuit::compute_steering(
                (truck_x, truck_z),
                truck_heading_deg as f64,
                (lx, lz),
                pure_pursuit::WHEELBASE_M,
            );

            // Geometric curvature κ = 2·y_local / d² (informational BB key).
            let vx = lx - truck_x;
            let vz = lz - truck_z;
            let h = (truck_heading_deg as f64).to_radians();
            let y_local = vx * h.cos() + vz * h.sin();
            let d2 = vx * vx + vz * vz;
            let curvature = if d2 > 1e-6 { 2.0 * y_local / d2 } else { 0.0 };

            self.steering_ema =
                STEERING_EMA_ALPHA * cmd + (1.0 - STEERING_EMA_ALPHA) * self.steering_ema;

            ctx.blackboard.set("lane_follower.steering_cmd", format!("{cmd:.4}"));
            ctx.blackboard.set("lane_follower.steering_curvature", format!("{curvature:.6}"));
            ctx.blackboard
                .set("lane_follower.steering_filtered", format!("{:.4}", self.steering_ema));

            self.last_steering_cmd = Some(cmd);
        }
    }

    fn tick_request(
        &mut self,
        _telemetry: Option<&Telemetry>,
        _ctx: &PluginContext,
    ) -> Option<ControlRequest> {
        if self.mode != LaneFollowerMode::Active {
            return None;
        }
        let cmd = self.last_steering_cmd?;
        Some(ControlRequest {
            steering: Some(cmd),
            ..Default::default()
        })
    }

    fn default_phase(&self) -> TickPhase {
        TickPhase::PhaseC
    }
}

/// Minimum angular difference between two headings in degrees, result in [0, 180].
fn angular_diff_deg(a: f32, b: f32) -> f32 {
    let diff = (a - b).abs() % 360.0;
    if diff > 180.0 { 360.0 - diff } else { diff }
}

truckpilot_plugin_api::export_plugin!(LaneFollowerPlugin);

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use truckpilot_map_parser::{
        arc_length::{build_all_luts, build_forward_adjacency},
        spline::HermiteSegment,
        spline_index::build_index,
    };
    use truckpilot_plugin_api::{PluginContext, Telemetry};

    // ── Helpers ──────────────────────────────────────────────────────────────

    fn make_telemetry(x: f64, y: f64, z: f64, heading: f64) -> Telemetry {
        Telemetry {
            position: [x, y, z],
            heading,
            pitch: 0.0,
            roll: 0.0,
            speed_ms: 0.0,
            engine_rpm: 0.0,
            cruise_control_kmh: 0.0,
            nav_speed_limit_kmh: -1.0,
            lead_vehicle_distance_m: -1.0,
            accel_longitudinal: -1.0,
            fuel_liters: -1.0,
            odometer_km: -1.0,
            nav_distance_m: -1.0,
            nav_time_s: -1.0,
        }
    }

    /// Build a straight Hermite segment from p0 to p1 using chord tangents.
    fn make_seg(p0: Vec3, p1: Vec3, uid: u64) -> HermiteSegment {
        let chord = Vec3::new(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
        let len = (chord.x * chord.x + chord.y * chord.y + chord.z * chord.z).sqrt();
        HermiteSegment {
            p0,
            p1,
            m0: chord,
            m1: chord,
            length_m: len,
            from_uid: uid,
            to_uid: uid + 1,
            edge_uid: uid,
        }
    }

    // ── angular_diff_deg ─────────────────────────────────────────────────────

    #[test]
    fn angular_diff_wraps_correctly() {
        assert!((angular_diff_deg(359.0, 1.0) - 2.0).abs() < 0.01, "wrap case");
        assert!((angular_diff_deg(1.0, 359.0) - 2.0).abs() < 0.01, "reverse wrap");
        assert!((angular_diff_deg(0.0, 180.0) - 180.0).abs() < 0.01, "opposite");
        assert!((angular_diff_deg(90.0, 90.0) - 0.0).abs() < 0.01, "same");
    }

    // ── Mode switch ──────────────────────────────────────────────────────────

    #[test]
    fn mode_defaults_to_observer_without_bb_key() {
        let ctx = PluginContext::test();
        assert_eq!(LaneFollowerMode::from_bb(&ctx), LaneFollowerMode::Observer);
    }

    #[test]
    fn mode_switches_to_active_via_bb() {
        let ctx = PluginContext::test();
        ctx.blackboard.set("plugin.lane-follower.mode", "active");
        assert_eq!(LaneFollowerMode::from_bb(&ctx), LaneFollowerMode::Active);
    }

    #[test]
    fn mode_stays_observer_for_unknown_value() {
        let ctx = PluginContext::test();
        ctx.blackboard.set("plugin.lane-follower.mode", "garbage");
        assert_eq!(LaneFollowerMode::from_bb(&ctx), LaneFollowerMode::Observer);
    }

    // ── No telemetry ─────────────────────────────────────────────────────────

    #[test]
    fn tick_writes_no_telemetry_status_when_none() {
        let mut plugin = LaneFollowerPlugin::default();
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        plugin.tick(None, &mut out, &ctx);
        assert_eq!(ctx.blackboard.get("lane_follower.status").as_deref(), Some("no_telemetry"));
    }

    // ── No index ─────────────────────────────────────────────────────────────

    #[test]
    fn tick_writes_no_index_status_when_graph_absent() {
        let mut plugin = LaneFollowerPlugin::default();
        // index is None (no on_load called)
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, 0.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(ctx.blackboard.get("lane_follower.status").as_deref(), Some("no_index"));
    }

    // ── SplineIndex with 3 mock segments ─────────────────────────────────────
    //
    // Layout (XZ plane):
    //   Seg 0: horizontal road, z=0, x: -100 → +100  (East, heading 90°)
    //   Seg 1: vertical road, x=-50, z: 0 → -100     (North, heading 0°)
    //   Seg 2: vertical road, x=+50, z: 0 → -100     (North, heading 0°)
    //
    // Query at (-45, 0, -50):
    //   - closest on Seg 0: (-45, 0, 0), dist ≈ 50 m
    //   - closest on Seg 1: (-50, 0, -50), dist = 5 m  ← expected winner
    //   - closest on Seg 2: (+50, 0, -50), dist = 95 m

    fn three_segment_index() -> SplineIndex {
        let seg0 = make_seg(
            Vec3::new(-100.0, 0.0, 0.0),
            Vec3::new(100.0, 0.0, 0.0),
            10,
        );
        let seg1 = make_seg(
            Vec3::new(-50.0, 0.0, 0.0),
            Vec3::new(-50.0, 0.0, -100.0),
            20,
        );
        let seg2 = make_seg(
            Vec3::new(50.0, 0.0, 0.0),
            Vec3::new(50.0, 0.0, -100.0),
            30,
        );
        build_index(vec![seg0, seg1, seg2])
    }

    #[test]
    fn spline_index_nearest_returns_expected_segment() {
        let index = three_segment_index();
        let query = Vec3::new(-45.0, 0.0, -50.0);
        let hit = index.nearest_with_projection(query, 8).expect("must find a hit");
        // Segment 1 (index 1) should win with dist ≈ 5 m
        assert_eq!(hit.segment_idx, 1, "expected segment 1 (North road at x=-50)");
        assert!(
            (hit.dist_m - 5.0).abs() < 0.5,
            "expected dist ≈ 5 m, got {:.3}",
            hit.dist_m
        );
    }

    #[test]
    fn spline_index_heading_is_north_for_north_segment() {
        let index = three_segment_index();
        let query = Vec3::new(-45.0, 0.0, -50.0);
        let hit = index.nearest_with_projection(query, 8).unwrap();
        // Segment 1 goes in -z direction: heading_deg = atan2(0, -(-chord.z)) = atan2(0,100) = 0°
        assert!(
            hit.heading_deg < 5.0 || hit.heading_deg > 355.0,
            "expected heading ≈ 0° (North), got {:.1}°",
            hit.heading_deg
        );
    }

    // ── dist_warn STOP condition ──────────────────────────────────────────────

    #[test]
    fn tick_sets_dist_warn_when_far_from_road() {
        let index = three_segment_index();
        let mut plugin = LaneFollowerPlugin { index: Some(index), mode: LaneFollowerMode::Observer, ..Default::default() };
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        // Query far from all segments (e.g. x=0, z=-500 — all segments end at z=-100)
        let tel = make_telemetry(0.0, 0.0, -500.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_follower.status").as_deref(),
            Some("dist_warn"),
            "expected dist_warn for position 400m past segment ends"
        );
    }

    // ── heading_warn STOP condition ───────────────────────────────────────────

    #[test]
    fn tick_sets_heading_warn_when_truck_facing_wrong_way() {
        let index = three_segment_index();
        let mut plugin = LaneFollowerPlugin { index: Some(index), mode: LaneFollowerMode::Observer, ..Default::default() };
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        // Truck at (-45, 0, -50) near Seg 1 (heading 0°=North)
        // ETS2 heading 0.5 = South → truck_heading_deg=180° → diff=180° > 90°
        let tel = make_telemetry(-45.0, 0.0, -50.0, 0.5);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_follower.status").as_deref(),
            Some("heading_warn"),
            "expected heading_warn when truck faces South on North road"
        );
    }

    // ── nominal ok path ──────────────────────────────────────────────────────

    #[test]
    fn tick_writes_ok_and_all_keys_on_nominal_path() {
        let index = three_segment_index();
        let mut plugin = LaneFollowerPlugin { index: Some(index), mode: LaneFollowerMode::Observer, ..Default::default() };
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        // Truck at (-45, 0, -50), facing North (heading=0)
        let tel = make_telemetry(-45.0, 0.0, -50.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(ctx.blackboard.get("lane_follower.status").as_deref(), Some("ok"));
        assert!(ctx.blackboard.get("lane_follower.nearest_seg_idx").is_some());
        assert!(ctx.blackboard.get("lane_follower.nearest_seg_dist_m").is_some());
        assert!(ctx.blackboard.get("lane_follower.nearest_seg_t").is_some());
        assert!(ctx.blackboard.get("lane_follower.heading_deg").is_some());
        assert!(ctx.blackboard.get("lane_follower.truck_x").is_some());
        assert!(ctx.blackboard.get("lane_follower.truck_z").is_some());
        assert_eq!(ctx.blackboard.get("lane_follower.active").as_deref(), Some("false"));
        assert_eq!(ctx.blackboard.get("lane_follower.mode").as_deref(), Some("observer"));
    }

    // ── tick_request always returns None in Observer mode ────────────────────

    #[test]
    fn tick_request_returns_none_in_observer_mode() {
        let mut plugin = LaneFollowerPlugin::default();
        let ctx = PluginContext::test();
        let tel = make_telemetry(0.0, 0.0, 0.0, 0.0);
        assert!(plugin.tick_request(Some(&tel), &ctx).is_none());
        assert!(plugin.tick_request(None, &ctx).is_none());
    }

    // ── Phase 1c: tick_request in Active mode ────────────────────────────────

    /// Build a chain plugin with a lateral road offset: road at x=offset, truck at x=0.
    /// Useful for testing steering sign and magnitude.
    fn make_chain_plugin_at_x(road_x: f32) -> LaneFollowerPlugin {
        let p0 = Vec3::new(road_x, 0.0, 0.0);
        let p1 = Vec3::new(road_x, 0.0, -10.0);
        let p2 = Vec3::new(road_x, 0.0, -20.0);
        let p3 = Vec3::new(road_x, 0.0, -30.0);
        let seg0 = make_seg(p0, p1, 1);
        let seg1 = make_seg(p1, p2, 2);
        let seg2 = make_seg(p2, p3, 3);
        let segs = vec![seg0, seg1, seg2];
        let luts = build_all_luts(&segs);
        let forward_adj = build_forward_adjacency(&segs);
        let index = build_index(segs);
        LaneFollowerPlugin { index: Some(index), luts, forward_adj, ..Default::default() }
    }

    #[test]
    fn tick_request_returns_some_in_active_mode_on_ok() {
        let mut plugin = make_chain_plugin();
        let ctx = PluginContext::test();
        ctx.blackboard.set("plugin.lane-follower.mode", "active");
        let mut out = ControlOutput::default();
        // Truck 2m into chain, facing North — status ok, lookahead exists.
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(ctx.blackboard.get("lane_follower.status").as_deref(), Some("ok"));
        let req = plugin.tick_request(Some(&tel), &ctx);
        assert!(req.is_some(), "Active + ok should produce a ControlRequest");
        let req = req.unwrap();
        assert!(req.steering.is_some(), "steering axis must be Some");
    }

    #[test]
    fn tick_request_returns_none_in_observer_even_when_ok() {
        let mut plugin = make_chain_plugin();
        let ctx = PluginContext::test();
        // mode is Observer by default
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(ctx.blackboard.get("lane_follower.status").as_deref(), Some("ok"));
        let req = plugin.tick_request(Some(&tel), &ctx);
        assert!(req.is_none(), "Observer mode must never produce ControlRequest");
    }

    #[test]
    fn tick_request_returns_none_when_dist_warn() {
        let mut plugin = make_chain_plugin();
        let ctx = PluginContext::test();
        ctx.blackboard.set("plugin.lane-follower.mode", "active");
        let mut out = ControlOutput::default();
        // Far from road → dist_warn
        let tel = make_telemetry(0.0, 0.0, -500.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(ctx.blackboard.get("lane_follower.status").as_deref(), Some("dist_warn"));
        assert!(
            plugin.tick_request(Some(&tel), &ctx).is_none(),
            "dist_warn must suppress ControlRequest"
        );
    }

    #[test]
    fn tick_request_returns_none_when_heading_warn() {
        let mut plugin = make_chain_plugin();
        let ctx = PluginContext::test();
        ctx.blackboard.set("plugin.lane-follower.mode", "active");
        let mut out = ControlOutput::default();
        // ETS2 heading 0.5 = South; chain goes North → heading_warn
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.5);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(ctx.blackboard.get("lane_follower.status").as_deref(), Some("heading_warn"));
        assert!(
            plugin.tick_request(Some(&tel), &ctx).is_none(),
            "heading_warn must suppress ControlRequest"
        );
    }

    #[test]
    fn steering_cmd_bb_key_written_zero_on_heading_warn() {
        // Hotfix: steering_cmd must be visible in blackboard even when status!=ok
        let mut plugin = make_chain_plugin();
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.5); // heading_warn
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(ctx.blackboard.get("lane_follower.status").as_deref(), Some("heading_warn"));
        let cmd = ctx.blackboard.get("lane_follower.steering_cmd");
        assert!(cmd.is_some(), "steering_cmd must be present on heading_warn");
        let val: f64 = cmd.unwrap().parse().unwrap();
        assert_eq!(val, 0.0, "steering_cmd must be 0.0 on heading_warn");
        assert!(ctx.blackboard.get("lane_follower.steering_filtered").is_some(), "filtered must be present");
        assert!(ctx.blackboard.get("lane_follower.steering_curvature").is_some(), "curvature must be present");
    }

    #[test]
    fn tick_request_steering_positive_for_road_to_right() {
        // Road at x=+5, truck at x=0, both facing North → lookahead is right → positive steering
        let mut plugin = make_chain_plugin_at_x(5.0);
        let ctx = PluginContext::test();
        ctx.blackboard.set("plugin.lane-follower.mode", "active");
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        let req = plugin.tick_request(Some(&tel), &ctx).expect("should produce request");
        let s = req.steering.unwrap();
        assert!(s > 0.0, "road to right → positive steering, got {s}");
    }

    #[test]
    fn tick_request_steering_negative_for_road_to_left() {
        // Road at x=-5, truck at x=0, both facing North → lookahead is left → negative steering
        let mut plugin = make_chain_plugin_at_x(-5.0);
        let ctx = PluginContext::test();
        ctx.blackboard.set("plugin.lane-follower.mode", "active");
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        let req = plugin.tick_request(Some(&tel), &ctx).expect("should produce request");
        let s = req.steering.unwrap();
        assert!(s < 0.0, "road to left → negative steering, got {s}");
    }

    #[test]
    fn tick_request_steering_near_zero_when_on_road() {
        // Road directly at truck x, truck aligned → steering ≈ 0
        let mut plugin = make_chain_plugin(); // road at x=0, truck at x=0
        let ctx = PluginContext::test();
        ctx.blackboard.set("plugin.lane-follower.mode", "active");
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        let req = plugin.tick_request(Some(&tel), &ctx).expect("should produce request");
        let s = req.steering.unwrap();
        assert!(s.abs() < 0.1, "on-road aligned → steering near 0, got {s}");
    }

    #[test]
    fn steering_cmd_bb_key_written_on_ok() {
        let mut plugin = make_chain_plugin();
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(ctx.blackboard.get("lane_follower.status").as_deref(), Some("ok"));
        assert!(
            ctx.blackboard.get("lane_follower.steering_cmd").is_some(),
            "steering_cmd key must be written when status=ok"
        );
    }

    #[test]
    fn steering_filtered_bb_key_written_on_ok() {
        let mut plugin = make_chain_plugin();
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert!(
            ctx.blackboard.get("lane_follower.steering_filtered").is_some(),
            "steering_filtered key must be written when status=ok"
        );
    }

    #[test]
    fn steering_curvature_bb_key_written_on_ok() {
        let mut plugin = make_chain_plugin();
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert!(
            ctx.blackboard.get("lane_follower.steering_curvature").is_some(),
            "steering_curvature key must be written when status=ok"
        );
    }

    #[test]
    fn steering_ema_converges_after_repeated_ticks() {
        // After enough identical ticks, filtered should converge toward raw cmd.
        let mut plugin = make_chain_plugin_at_x(5.0);
        let ctx = PluginContext::test();
        ctx.blackboard.set("plugin.lane-follower.mode", "active");
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        // 30 ticks is plenty for α=0.3 EMA to converge to >95% of final value.
        for _ in 0..30 {
            plugin.tick(Some(&tel), &mut out, &ctx);
        }
        let raw: f64 = ctx
            .blackboard
            .get("lane_follower.steering_cmd")
            .unwrap()
            .parse()
            .unwrap();
        let filtered: f64 = ctx
            .blackboard
            .get("lane_follower.steering_filtered")
            .unwrap()
            .parse()
            .unwrap();
        assert!(
            (raw - filtered).abs() < 0.01,
            "EMA must converge: raw={raw:.4}, filtered={filtered:.4}"
        );
    }

    // ── Phase 1b — Lookahead helpers ─────────────────────────────────────────

    /// 3 connected 10m North-going segments (UIDs 1→2→3→4), with LUTs + adjacency.
    fn make_chain_plugin() -> LaneFollowerPlugin {
        let seg0 = make_seg(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, -10.0), 1);
        let seg1 = make_seg(Vec3::new(0.0, 0.0, -10.0), Vec3::new(0.0, 0.0, -20.0), 2);
        let seg2 = make_seg(Vec3::new(0.0, 0.0, -20.0), Vec3::new(0.0, 0.0, -30.0), 3);
        let segs = vec![seg0, seg1, seg2];
        let luts = build_all_luts(&segs);
        let forward_adj = build_forward_adjacency(&segs);
        let index = build_index(segs);
        LaneFollowerPlugin { index: Some(index), luts, forward_adj, ..Default::default() }
    }

    // ── Lookahead status: ok (within segment) ────────────────────────────────

    #[test]
    fn lookahead_within_seg_writes_ok_status() {
        let mut plugin = make_chain_plugin();
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        // truck 2m into seg 0 — 15m lookahead crosses into seg 1
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_follower.lookahead_status").as_deref(),
            Some("ok"),
            "expected ok when successors exist"
        );
        assert!(ctx.blackboard.get("lane_follower.lookahead_x").is_some());
        assert!(ctx.blackboard.get("lane_follower.lookahead_z").is_some());
        assert!(ctx.blackboard.get("lane_follower.lookahead_seg_idx").is_some());
        assert!(ctx.blackboard.get("lane_follower.lookahead_remaining_m").is_some());
    }

    // ── Lookahead status: ok (crosses boundaries) ────────────────────────────

    #[test]
    fn lookahead_crosses_boundaries_writes_ok_status() {
        let mut plugin = make_chain_plugin();
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        // truck 1m into seg 0 — 15m lookahead: 9m to end of seg 0, then 6m into seg 1 → z≈-16
        let tel = make_telemetry(0.0, 0.0, -1.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(ctx.blackboard.get("lane_follower.lookahead_status").as_deref(), Some("ok"));
        let laz: f32 = ctx
            .blackboard
            .get("lane_follower.lookahead_z")
            .unwrap()
            .parse()
            .unwrap();
        assert!((laz - -16.0).abs() < 1.5, "expected z≈-16m, got {laz:.3}");
    }

    // ── Lookahead status: dead_end ────────────────────────────────────────────

    #[test]
    fn lookahead_dead_end_writes_dead_end_status() {
        // Single isolated segment — no successor → dead_end
        let seg = make_seg(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, -10.0), 99);
        let segs = vec![seg];
        let luts = build_all_luts(&segs);
        let forward_adj = build_forward_adjacency(&segs);
        let index = build_index(segs);
        let mut plugin = LaneFollowerPlugin { index: Some(index), luts, forward_adj, ..Default::default() };
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -1.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_follower.lookahead_status").as_deref(),
            Some("dead_end"),
            "single segment with no successor must yield dead_end"
        );
        let remaining: f32 = ctx
            .blackboard
            .get("lane_follower.lookahead_remaining_m")
            .unwrap()
            .parse()
            .unwrap();
        assert!(remaining > 0.0, "dead_end must have remaining_m > 0, got {remaining}");
    }

    // ── Lookahead: heading filter at junction ────────────────────────────────

    #[test]
    fn lookahead_heading_filter_at_junction() {
        // seg 0 and seg 1 go North; seg 2 branches East from the same junction node (uid=2)
        // 15m lookahead from seg 0 should follow the straight North path (seg 1), not the East branch
        let seg0 = make_seg(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, -10.0), 1);
        let seg1 = make_seg(Vec3::new(0.0, 0.0, -10.0), Vec3::new(0.0, 0.0, -20.0), 2);
        let p0 = Vec3::new(0.0, 0.0, -10.0);
        let p1 = Vec3::new(10.0, 0.0, -10.0);
        let chord = Vec3::new(10.0, 0.0, 0.0);
        let seg2 = HermiteSegment {
            p0,
            p1,
            m0: chord,
            m1: chord,
            length_m: 10.0,
            from_uid: 2,
            to_uid: 99,
            edge_uid: 299,
        };
        let segs = vec![seg0, seg1, seg2];
        let luts = build_all_luts(&segs);
        let forward_adj = build_forward_adjacency(&segs);
        let index = build_index(segs);
        let mut plugin = LaneFollowerPlugin { index: Some(index), luts, forward_adj, ..Default::default() };
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(ctx.blackboard.get("lane_follower.lookahead_status").as_deref(), Some("ok"));
        let seg_idx: usize = ctx
            .blackboard
            .get("lane_follower.lookahead_seg_idx")
            .unwrap()
            .parse()
            .unwrap();
        assert_eq!(seg_idx, 1, "heading filter must pick North seg (1), not East branch (2)");
    }

    // ── Stability: stable after repeated same-position ticks ────────────────

    #[test]
    fn stability_stable_after_repeated_ticks_on_chain() {
        let mut plugin = make_chain_plugin();
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        for _ in 0..5 {
            plugin.tick(Some(&tel), &mut out, &ctx);
        }
        assert_eq!(
            ctx.blackboard.get("lane_follower.lookahead_stable").as_deref(),
            Some("true"),
            "stable expected after 5 identical ticks"
        );
        let jumps: usize = ctx
            .blackboard
            .get("lane_follower.lookahead_seg_jump_count")
            .unwrap()
            .parse()
            .unwrap();
        assert_eq!(jumps, 0, "zero jumps expected for constant position");
    }
}
