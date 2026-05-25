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
//! | `lane_follower.steering_filtered` | f64 [-1,1] | rate-limited EMA steering (α=0.15, rate=0.05/tick) |
//! | `lane_follower.lookahead_distance_m` | f32 metres | effective lookahead distance this tick |
//! | `lane_follower.wheelbase_m` | f64 metres | truck wheelbase (diagnostic constant) |
//! | `lane_follower.rate_limited` | `"true"/"false"` | rate-limiter was active this tick |
//! | `lane_follower.lane_offset_m` | f32 metres | effective lane offset this tick (right = positive) |
//! | `lane_follower.lane_normal_x` | f32 | right-normal X at nearest spline point |
//! | `lane_follower.lane_normal_z` | f32 | right-normal Z at nearest spline point |
//! | `lane_follower.lateral_dist_signed` | f32 metres | signed lateral dist (+ = truck right of centreline) |
//! | `lane_follower.lookahead_offset_x` | f32 metres | lane-offset lookahead X (input to Pure-Pursuit) |
//! | `lane_follower.lookahead_offset_z` | f32 metres | lane-offset lookahead Z (input to Pure-Pursuit) |

mod junction;
mod pure_pursuit;

use std::collections::HashMap;
use std::collections::VecDeque;

use junction::{detect_junction, JunctionDetector};
use truckpilot_map_parser::{
    arc_length::{build_all_luts, build_forward_adjacency, lookahead, ArcLengthLUT, LOOKAHEAD_MAX_HOPS},
    graph::MapGraph,
    spline::{build_splines, evaluate_tangent, HermiteSegment, Vec3},
    spline_index::{build_index, SplineIndex},
};
use truckpilot_plugin_api::{
    ctx_info, ctx_warn, graph::RouterGraph, ControlOutput, ControlRequest, Plugin, PluginContext,
    Telemetry, TickPhase,
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

/// Minimum lookahead distance. Floor at low/zero speed.
const LOOKAHEAD_DIST_M_MIN: f32 = 15.0;
/// Maximum lookahead distance. Caps at high speed.
const LOOKAHEAD_DIST_M_MAX: f32 = 50.0;
/// Speed-adaptive lookahead factor: lookahead_m = speed_kmh * factor (≈1 sec ahead).
const LOOKAHEAD_SPEED_FACTOR: f32 = 0.3;

/// Ring-buffer size for lookahead stability metric (ticks = 1 second at 50 Hz).
const STABILITY_WINDOW: usize = 50;

/// VMM-6: minimum minimap temporal confidence to use as fallback spline source.
const MINIMAP_CONF_THRESHOLD: f32 = 0.4;

/// EMA smoothing factor for the steering output (higher = faster response).
const STEERING_EMA_ALPHA: f64 = 0.15;

/// Rate limiter: max steering change per tick (50 Hz → 2.5/s max slew rate).
const STEERING_RATE_LIMIT: f64 = 0.05;

/// Standard ETS2 lane width in metres (2-lane road).
const LANE_WIDTH_M: f32 = 3.75;
/// Default right-lane offset from road centreline: half a lane width.
const LANE_OFFSET_RIGHT_M: f32 = LANE_WIDTH_M / 2.0;

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
    /// Previous rate-limited steering output (rate-limiter state).
    steering_rate_limited_prev: f64,
    /// RouterGraph built from the same graph.json — used for junction detection.
    router_graph: Option<RouterGraph>,
    junction_detector: JunctionDetector,
    // VMM-6: minimap fallback spline source.
    /// SplineIndex rebuilt from minimap.spline_json whenever the capture timestamp changes.
    minimap_index: Option<SplineIndex>,
    /// minimap.last_capture_ms seen on last refresh — change triggers rebuild.
    last_minimap_ts: u64,
    /// minimap.confidence last read from blackboard.
    minimap_confidence: f32,
}

/// VMM-6: simple forward-walk lookahead over minimap segments (no LUT required).
/// Starts at `(start_idx, start_t)` and walks forward until `dist_m` is consumed.
/// Returns the world-space lookahead point, or None if segments run out.
fn minimap_lookahead(
    segs: &[HermiteSegment],
    start_idx: usize,
    start_t: f32,
    dist_m: f32,
) -> Option<Vec3> {
    use truckpilot_map_parser::spline::evaluate;
    let mut remaining = dist_m;
    let mut idx = start_idx;
    let mut t = start_t;
    loop {
        if idx >= segs.len() {
            break;
        }
        let seg = &segs[idx];
        let seg_remaining = seg.length_m * (1.0 - t);
        if remaining <= seg_remaining {
            let t_advance = t + (remaining / seg.length_m.max(1e-6));
            return Some(evaluate(seg, t_advance.min(1.0)));
        }
        remaining -= seg_remaining;
        idx += 1;
        t = 0.0;
    }
    // Ran out of segments — return end of last segment.
    if !segs.is_empty() {
        let last = &segs[segs.len() - 1];
        Some(evaluate(last, 1.0))
    } else {
        None
    }
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

        let rg_nodes: Vec<(u64, f64, f64)> = graph.nodes.iter().map(|n| (n.uid, n.x, n.z)).collect();
        let rg_edges: Vec<(u64, u64, f64)> = graph.edges.iter().map(|e| (e.from, e.to, e.distance_m)).collect();
        let n_nodes = rg_nodes.len();
        let n_edges = rg_edges.len();
        self.router_graph = Some(RouterGraph::new(rg_nodes, rg_edges));
        ctx_info!(ctx, "lane-follower: RouterGraph built ({} nodes, {} edges)", n_nodes, n_edges);
    }

    /// VMM-6: refresh minimap_index from BB if new data is available.
    fn try_refresh_minimap(&mut self, ctx: &PluginContext) {
        let conf: f32 = ctx
            .blackboard
            .get("minimap.confidence")
            .and_then(|v| v.parse().ok())
            .unwrap_or(0.0);
        self.minimap_confidence = conf;

        if ctx.blackboard.get("minimap.detected").as_deref() != Some("true") {
            return;
        }
        if conf < MINIMAP_CONF_THRESHOLD {
            return;
        }

        let ts: u64 = ctx
            .blackboard
            .get("minimap.last_capture_ms")
            .and_then(|v| v.parse().ok())
            .unwrap_or(0);

        if ts == self.last_minimap_ts {
            return; // nothing new
        }

        let json = match ctx.blackboard.get("minimap.spline_json") {
            Some(j) => j,
            None => return,
        };

        match serde_json::from_str::<Vec<HermiteSegment>>(&json) {
            Ok(segs) if segs.len() >= 2 => {
                self.minimap_index = Some(build_index(segs));
                self.last_minimap_ts = ts;
            }
            _ => {
                self.minimap_index = None;
            }
        }
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
        self.steering_rate_limited_prev = 0.0;
        self.router_graph = None;
        self.junction_detector.reset();
        self.minimap_index = None;
        self.last_minimap_ts = 0;
        self.minimap_confidence = 0.0;
    }

    fn tick(&mut self, telemetry: Option<&Telemetry>, _output: &mut ControlOutput, ctx: &PluginContext) {
        // Reset per-tick steering so tick_request() sees None on any early return.
        self.last_steering_cmd = None;

        // VMM-6: refresh minimap spline index from blackboard (cheap no-op if unchanged).
        self.try_refresh_minimap(ctx);

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

        // Junction detection — runs regardless of index availability.
        {
            let detection = if let Some(graph) = &self.router_graph {
                detect_junction(graph, truck_x, truck_z)
            } else {
                junction::JunctionDetection { is_junction: false, max_degree: 0, distance_m: None }
            };
            let (active, phase) = self.junction_detector.tick(&detection);
            ctx.blackboard.set("lane_follower.junction_detected", if active { "true" } else { "false" });
            ctx.blackboard.set("lane_follower.junction_phase", phase.as_str());
            ctx.blackboard.set(
                "lane_follower.junction_distance_m",
                detection.distance_m.map_or_else(String::new, |d| format!("{d:.1}")),
            );
            ctx.blackboard.set("lane_follower.junction_max_degree", detection.max_degree.to_string());
        }

        // VMM-6: select primary or minimap SplineIndex.
        let primary = &self.index;
        let minimap = &self.minimap_index;
        let index: &SplineIndex = match (primary, minimap) {
            (Some(idx), _) => idx,
            (None, Some(mm)) if self.minimap_confidence >= MINIMAP_CONF_THRESHOLD => mm,
            _ => {
                ctx.blackboard.set("lane_follower.status", "no_index");
                ctx.blackboard.set("lane_follower.spline_source", "none");
                return;
            }
        };
        let using_minimap = primary.is_none();
        ctx.blackboard.set(
            "lane_follower.spline_source",
            if using_minimap { "minimap" } else { "map" },
        );

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

        // Lane offset: BB-key override → code constant.
        let lane_offset_m = ctx
            .blackboard
            .get("plugin.lane-follower.lane_offset_m")
            .and_then(|v| v.parse::<f32>().ok())
            .unwrap_or(LANE_OFFSET_RIGHT_M);

        // Task 4 — Diagnostic: right-normal and signed lateral distance at nearest spline point.
        // Right-normal at heading h (CW degrees): n = (cos h, sin h) in XZ.
        // Forward at heading h: f = (sin h, -cos h) in XZ.
        // Signed lateral = f × (truck − spline) = fx*(tz_diff) − fz*(tx_diff). Positive = truck right.
        let road_h_rad = (hit.heading_deg as f32).to_radians();
        let near_n_x = road_h_rad.cos();
        let near_n_z = road_h_rad.sin();
        let fwd_x = road_h_rad.sin();
        let fwd_z = -road_h_rad.cos();
        let lateral_dist_signed = fwd_x * (truck_z as f32 - hit.point_on_curve.z)
            - fwd_z * (truck_x as f32 - hit.point_on_curve.x);
        ctx.blackboard.set("lane_follower.lane_offset_m", format!("{lane_offset_m:.3}"));
        ctx.blackboard.set("lane_follower.lane_normal_x", format!("{near_n_x:.4}"));
        ctx.blackboard.set("lane_follower.lane_normal_z", format!("{near_n_z:.4}"));
        ctx.blackboard.set("lane_follower.lateral_dist_signed", format!("{lateral_dist_signed:.3}"));

        // Per-tick counter — never resets, monotonic. Catches telemetry stalls.
        self.tick_count += 1;
        ctx.blackboard
            .set("lane_follower.tick_count", self.tick_count.to_string());

        // Speed-adaptive lookahead: 1 sec ahead, clamped to [15m, 50m].
        let speed_kmh = tel.speed_ms as f32 * 3.6;
        let lookahead_dist_m = (speed_kmh * LOOKAHEAD_SPEED_FACTOR)
            .max(LOOKAHEAD_DIST_M_MIN)
            .min(LOOKAHEAD_DIST_M_MAX);
        ctx.blackboard
            .set("lane_follower.lookahead_distance_m", format!("{lookahead_dist_m:.1}"));
        ctx.blackboard
            .set("lane_follower.wheelbase_m", format!("{:.1}", pure_pursuit::WHEELBASE_M));

        // Lookahead — computed before warn-checks so keys are always present after a hit.
        // VMM-6: minimap uses simple forward-walk (no LUT); primary uses arc-length LUT.
        let la_point_opt: Option<Vec3> = if using_minimap {
            minimap_lookahead(
                index.segments.as_slice(),
                hit.segment_idx,
                hit.t,
                lookahead_dist_m,
            )
        } else if !self.luts.is_empty() {
            lookahead(
                hit.segment_idx,
                hit.t,
                lookahead_dist_m,
                &self.forward_adj,
                index.segments.as_slice(),
                &self.luts,
            )
            .map(|la| {
                // When using primary LUT lookahead, publish extra LUT-specific keys.
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

                la.point
            })
        } else {
            None
        };

        if let Some(la_pt) = la_point_opt {
            let la = la_pt;
            {
                ctx.blackboard
                    .set("lane_follower.lookahead_x", format!("{:.3}", la.x));
                ctx.blackboard
                    .set("lane_follower.lookahead_z", format!("{:.3}", la.z));

                // Offset lookahead right by lane_offset_m.
                // Tangent at hit point → right-normal in XZ: n = (-tz, tx) (normalised).
                let la_tan = evaluate_tangent(&index.segments[hit.segment_idx], hit.t);
                let la_len_xz = (la_tan.x * la_tan.x + la_tan.z * la_tan.z).sqrt();
                let (la_n_x, la_n_z) = if la_len_xz > 1e-6 {
                    (-la_tan.z / la_len_xz, la_tan.x / la_len_xz)
                } else {
                    (0.0_f32, 0.0_f32)
                };
                let offset_lx = la.x + la_n_x * lane_offset_m;
                let offset_lz = la.z + la_n_z * lane_offset_m;
                ctx.blackboard
                    .set("lane_follower.lookahead_offset_x", format!("{offset_lx:.3}"));
                ctx.blackboard
                    .set("lane_follower.lookahead_offset_z", format!("{offset_lz:.3}"));

                let dx = la.x - query.x;
                let dz = la.z - query.z;
                let heading_to_la = dx.atan2(-dz).to_degrees().rem_euclid(360.0);
                ctx.blackboard
                    .set("lane_follower.heading_to_lookahead_deg", format!("{:.2}", heading_to_la));
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
            let rl = self.steering_ema.clamp(
                self.steering_rate_limited_prev - STEERING_RATE_LIMIT,
                self.steering_rate_limited_prev + STEERING_RATE_LIMIT,
            );
            self.steering_rate_limited_prev = rl;
            ctx.blackboard.set("lane_follower.steering_cmd", "0.0000");
            ctx.blackboard.set("lane_follower.steering_curvature", "0.000000");
            ctx.blackboard.set("lane_follower.steering_filtered", format!("{rl:.4}"));
            ctx.blackboard.set("lane_follower.rate_limited", "false");
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
            let rl = self.steering_ema.clamp(
                self.steering_rate_limited_prev - STEERING_RATE_LIMIT,
                self.steering_rate_limited_prev + STEERING_RATE_LIMIT,
            );
            self.steering_rate_limited_prev = rl;
            ctx.blackboard.set("lane_follower.steering_cmd", "0.0000");
            ctx.blackboard.set("lane_follower.steering_curvature", "0.000000");
            ctx.blackboard.set("lane_follower.steering_filtered", format!("{rl:.4}"));
            ctx.blackboard.set("lane_follower.rate_limited", "false");
            return;
        }

        ctx.blackboard.set("lane_follower.status", "ok");

        // Pure-Pursuit steering — uses the lane-offset lookahead point (not raw centreline).
        if let (Some(lx), Some(lz)) = (
            ctx.blackboard.get_f64("lane_follower.lookahead_offset_x"),
            ctx.blackboard.get_f64("lane_follower.lookahead_offset_z"),
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
            let rl = self.steering_ema.clamp(
                self.steering_rate_limited_prev - STEERING_RATE_LIMIT,
                self.steering_rate_limited_prev + STEERING_RATE_LIMIT,
            );
            let is_rate_limited = (self.steering_ema - rl).abs() > 1e-9;
            self.steering_rate_limited_prev = rl;

            ctx.blackboard.set("lane_follower.steering_cmd", format!("{cmd:.4}"));
            ctx.blackboard.set("lane_follower.steering_curvature", format!("{curvature:.6}"));
            ctx.blackboard.set("lane_follower.steering_filtered", format!("{rl:.4}"));
            ctx.blackboard.set("lane_follower.rate_limited", is_rate_limited.to_string());

            self.last_steering_cmd = Some(cmd);
        }
    }

    fn tick_request(
        &mut self,
        _telemetry: Option<&Telemetry>,
        ctx: &PluginContext,
    ) -> Option<ControlRequest> {
        if self.mode != LaneFollowerMode::Active {
            ctx.blackboard.set("lane_follower.engage_source", "none");
            return None;
        }
        let engage_mode = ctx.blackboard.get("autopilot.engage_mode");
        let engage_source = match engage_mode.as_deref() {
            Some("route") => "route",
            Some("lane") => "lane",
            Some("degraded") => "degraded",
            _ => "none",
        };
        ctx.blackboard.set("lane_follower.engage_source", engage_source);
        // DEGRADED is advisory only — no ControlRequest emitted
        if !matches!(engage_source, "route" | "lane") {
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
        ctx.blackboard.set("autopilot.engage_mode", "route");
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
        ctx.blackboard.set("autopilot.engage_mode", "route");
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
        ctx.blackboard.set("autopilot.engage_mode", "route");
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        let req = plugin.tick_request(Some(&tel), &ctx).expect("should produce request");
        let s = req.steering.unwrap();
        assert!(s < 0.0, "road to left → negative steering, got {s}");
    }

    #[test]
    fn tick_request_steering_near_zero_when_on_road() {
        // Road directly at truck x, truck aligned → steering ≈ 0 (with offset=0 to test raw steering).
        let mut plugin = make_chain_plugin(); // road at x=0, truck at x=0
        let ctx = PluginContext::test();
        ctx.blackboard.set("plugin.lane-follower.mode", "active");
        ctx.blackboard.set("autopilot.engage_mode", "route");
        ctx.blackboard.set("plugin.lane-follower.lane_offset_m", "0.0");
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
        // With α=0.15, rate-limiter disengages at ~tick 11; by tick 30 both EMA and
        // rate-limited output are >99% converged to raw cmd (0.85^30 ≈ 0.008 residual).
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

    // ── P0.3: engage_mode gate + engage_source BB key ────────────────────────

    fn make_active_plugin_with_request() -> (LaneFollowerPlugin, PluginContext) {
        let mut plugin = make_chain_plugin();
        let ctx = PluginContext::test();
        ctx.blackboard.set("plugin.lane-follower.mode", "active");
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(ctx.blackboard.get("lane_follower.status").as_deref(), Some("ok"));
        (plugin, ctx)
    }

    #[test]
    fn p03_engage_mode_route_emits_control_request() {
        let (mut plugin, ctx) = make_active_plugin_with_request();
        ctx.blackboard.set("autopilot.engage_mode", "route");
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        let req = plugin.tick_request(Some(&tel), &ctx);
        assert!(req.is_some(), "engage_mode=route + status=ok → Some(ControlRequest)");
        assert_eq!(ctx.blackboard.get("lane_follower.engage_source").as_deref(), Some("route"));
    }

    #[test]
    fn p03_engage_mode_lane_emits_control_request() {
        let (mut plugin, ctx) = make_active_plugin_with_request();
        ctx.blackboard.set("autopilot.engage_mode", "lane");
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        let req = plugin.tick_request(Some(&tel), &ctx);
        assert!(req.is_some(), "engage_mode=lane + status=ok → Some(ControlRequest)");
        assert_eq!(ctx.blackboard.get("lane_follower.engage_source").as_deref(), Some("lane"));
    }

    #[test]
    fn p03_engage_mode_degraded_suppresses_control_request() {
        let (mut plugin, ctx) = make_active_plugin_with_request();
        ctx.blackboard.set("autopilot.engage_mode", "degraded");
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        let req = plugin.tick_request(Some(&tel), &ctx);
        assert!(req.is_none(), "engage_mode=degraded must NOT emit ControlRequest (advisory only)");
        assert_eq!(ctx.blackboard.get("lane_follower.engage_source").as_deref(), Some("degraded"));
    }

    #[test]
    fn p03_engage_mode_missing_suppresses_control_request() {
        let (mut plugin, ctx) = make_active_plugin_with_request();
        // No autopilot.engage_mode key set
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        let req = plugin.tick_request(Some(&tel), &ctx);
        assert!(req.is_none(), "missing engage_mode must NOT emit ControlRequest");
        assert_eq!(ctx.blackboard.get("lane_follower.engage_source").as_deref(), Some("none"));
    }

    #[test]
    fn p03_observer_mode_suppresses_regardless_of_engage_mode() {
        let mut plugin = make_chain_plugin();
        let ctx = PluginContext::test();
        // mode stays Observer (default), set engage_mode to route
        ctx.blackboard.set("autopilot.engage_mode", "route");
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        let req = plugin.tick_request(Some(&tel), &ctx);
        assert!(req.is_none(), "Observer mode must never emit ControlRequest regardless of engage_mode");
        assert_eq!(ctx.blackboard.get("lane_follower.engage_source").as_deref(), Some("none"));
    }

    // ── Task-5 tuning tests ───────────────────────────────────────────────────

    #[test]
    fn test_steering_at_small_offset() {
        // Road at x=0.5m, truck at x=0 heading North: dist≈0.5m, heading_diff≈0°.
        // With L=4.0m, cmd = 2*y/L² = 2*0.5/16 ≈ 0.0625 — well under 0.15.
        // Use lane_offset_m=0 to test raw wheelbase-correctness independent of QW1 offset.
        let mut plugin = make_chain_plugin_at_x(0.5);
        let ctx = PluginContext::test();
        ctx.blackboard.set("plugin.lane-follower.lane_offset_m", "0.0");
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(ctx.blackboard.get("lane_follower.status").as_deref(), Some("ok"));
        let cmd: f64 = ctx.blackboard.get("lane_follower.steering_cmd").unwrap().parse().unwrap();
        assert!(cmd > 0.0, "road to right → cmd positive, got {cmd:.4}");
        assert!(cmd < 0.15, "0.5m offset must produce cmd < 0.15 (was ~0.39 pre-fix), got {cmd:.4}");
    }

    #[test]
    fn test_lookahead_scales_with_speed() {
        let mut plugin = make_chain_plugin();
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();

        // 50 km/h = 13.89 m/s → max(15, 50 * 0.3) = 15.0 m (floor)
        let mut tel = make_telemetry(0.0, 0.0, -1.0, 0.0);
        tel.speed_ms = 13.89;
        plugin.tick(Some(&tel), &mut out, &ctx);
        let la: f32 = ctx
            .blackboard
            .get("lane_follower.lookahead_distance_m")
            .unwrap()
            .parse()
            .unwrap();
        assert!((la - 15.0).abs() < 0.5, "50 km/h should give 15 m lookahead, got {la:.1}");

        // 80 km/h = 22.22 m/s → max(15, 80 * 0.3) = 24.0 m
        tel.speed_ms = 22.22;
        plugin.tick(Some(&tel), &mut out, &ctx);
        let la: f32 = ctx
            .blackboard
            .get("lane_follower.lookahead_distance_m")
            .unwrap()
            .parse()
            .unwrap();
        assert!((la - 24.0).abs() < 0.5, "80 km/h should give 24 m lookahead, got {la:.1}");
    }

    #[test]
    fn test_rate_limiter_clamps_jumps() {
        // Road far right (x=8m) → cmd saturates to ~1.0. From cold start, rate-limiter
        // must cap the first filtered output at 0.05 (max delta per tick).
        let mut plugin = make_chain_plugin_at_x(8.0);
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(ctx.blackboard.get("lane_follower.status").as_deref(), Some("ok"));
        let cmd: f64 = ctx.blackboard.get("lane_follower.steering_cmd").unwrap().parse().unwrap();
        assert!(cmd > 0.9, "large offset must saturate cmd near 1.0, got {cmd:.4}");
        let filtered: f64 = ctx
            .blackboard
            .get("lane_follower.steering_filtered")
            .unwrap()
            .parse()
            .unwrap();
        assert!(
            filtered <= 0.05 + 1e-9,
            "rate-limiter must cap first-tick output at 0.05, got {filtered:.4}"
        );
        assert_eq!(
            ctx.blackboard.get("lane_follower.rate_limited").as_deref(),
            Some("true"),
            "rate_limited BB key must be true when limiter fires"
        );
    }

    #[test]
    fn test_ema_smoothing_at_alpha_015() {
        // Road at x=0.8m → cmd ≈ 0.10; small enough that rate-limiter doesn't fire on tick 1.
        // After 1 tick: filtered = α * cmd ≈ 0.015 — less than cmd/2, showing α=0.15 not 0.3.
        // After 60 ticks: filtered converges to cmd within 0.005.
        let mut plugin = make_chain_plugin_at_x(0.8);
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);

        plugin.tick(Some(&tel), &mut out, &ctx);
        let raw: f64 = ctx.blackboard.get("lane_follower.steering_cmd").unwrap().parse().unwrap();
        let filtered: f64 =
            ctx.blackboard.get("lane_follower.steering_filtered").unwrap().parse().unwrap();
        assert!(raw > 0.0, "road at x=0.8 must produce positive cmd");
        assert!(
            filtered < raw / 2.0,
            "after 1 tick, EMA (α=0.15) produces filtered < raw/2: raw={raw:.4}, filtered={filtered:.4}"
        );

        for _ in 0..59 {
            plugin.tick(Some(&tel), &mut out, &ctx);
        }
        let raw_final: f64 =
            ctx.blackboard.get("lane_follower.steering_cmd").unwrap().parse().unwrap();
        let filtered_final: f64 =
            ctx.blackboard.get("lane_follower.steering_filtered").unwrap().parse().unwrap();
        assert!(
            (raw_final - filtered_final).abs() < 0.005,
            "after 60 ticks EMA must converge: raw={raw_final:.4}, filtered={filtered_final:.4}"
        );
    }

    // ── QW1: Lane-Offset tests ────────────────────────────────────────────────

    #[test]
    fn test_lane_offset_perpendicular_on_north_road() {
        // North-going chain (heading 0°). Right-normal = East = (+x, 0).
        // With default offset 1.875 m: offset_x ≈ la_x + 1.875, offset_z ≈ la_z.
        let mut plugin = make_chain_plugin();
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(ctx.blackboard.get("lane_follower.status").as_deref(), Some("ok"));
        let la_x: f32 = ctx.blackboard.get("lane_follower.lookahead_x").unwrap().parse().unwrap();
        let la_z: f32 = ctx.blackboard.get("lane_follower.lookahead_z").unwrap().parse().unwrap();
        let off_x: f32 = ctx.blackboard.get("lane_follower.lookahead_offset_x").unwrap().parse().unwrap();
        let off_z: f32 = ctx.blackboard.get("lane_follower.lookahead_offset_z").unwrap().parse().unwrap();
        assert!(
            (off_x - la_x - LANE_OFFSET_RIGHT_M).abs() < 0.05,
            "North road: offset_x must be la_x + {}, got delta {:.4}", LANE_OFFSET_RIGHT_M, off_x - la_x
        );
        assert!(
            (off_z - la_z).abs() < 0.05,
            "North road: offset_z must be unchanged, got delta {:.4}", off_z - la_z
        );
    }

    #[test]
    fn test_lane_offset_perpendicular_on_east_road() {
        // East-going segment (heading 90°). Right-normal = South = (0, +z).
        // With default offset 1.875 m: offset_z ≈ la_z + 1.875, offset_x ≈ la_x.
        let p0 = Vec3::new(-20.0, 0.0, 0.0);
        let p1 = Vec3::new(20.0, 0.0, 0.0);
        let seg = make_seg(p0, p1, 10);
        let segs = vec![seg];
        let luts = build_all_luts(&segs);
        let forward_adj = build_forward_adjacency(&segs);
        let index = build_index(segs);
        let mut plugin = LaneFollowerPlugin { index: Some(index), luts, forward_adj, ..Default::default() };
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        // ETS2 heading 0.75 → truck_heading_deg = (-0.75 * 360).rem_euclid(360) = 90° = East.
        let tel = make_telemetry(0.0, 0.0, 0.0, 0.75);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(ctx.blackboard.get("lane_follower.status").as_deref(), Some("ok"));
        let la_x: f32 = ctx.blackboard.get("lane_follower.lookahead_x").unwrap().parse().unwrap();
        let la_z: f32 = ctx.blackboard.get("lane_follower.lookahead_z").unwrap().parse().unwrap();
        let off_x: f32 = ctx.blackboard.get("lane_follower.lookahead_offset_x").unwrap().parse().unwrap();
        let off_z: f32 = ctx.blackboard.get("lane_follower.lookahead_offset_z").unwrap().parse().unwrap();
        assert!(
            (off_z - la_z - LANE_OFFSET_RIGHT_M).abs() < 0.05,
            "East road: offset_z must be la_z + {}, got delta {:.4}", LANE_OFFSET_RIGHT_M, off_z - la_z
        );
        assert!(
            (off_x - la_x).abs() < 0.05,
            "East road: offset_x must be unchanged, got delta {:.4}", off_x - la_x
        );
    }

    #[test]
    fn test_lane_offset_zero_bb_key_matches_centreline() {
        // STOP condition: lane_offset_m=0 → lookahead_offset == lookahead_raw (no lateral shift).
        let mut plugin = make_chain_plugin();
        let ctx = PluginContext::test();
        ctx.blackboard.set("plugin.lane-follower.lane_offset_m", "0.0");
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(ctx.blackboard.get("lane_follower.status").as_deref(), Some("ok"));
        let la_x: f32 = ctx.blackboard.get("lane_follower.lookahead_x").unwrap().parse().unwrap();
        let la_z: f32 = ctx.blackboard.get("lane_follower.lookahead_z").unwrap().parse().unwrap();
        let off_x: f32 = ctx.blackboard.get("lane_follower.lookahead_offset_x").unwrap().parse().unwrap();
        let off_z: f32 = ctx.blackboard.get("lane_follower.lookahead_offset_z").unwrap().parse().unwrap();
        assert!(
            (off_x - la_x).abs() < 1e-3,
            "offset=0: lookahead_offset_x must equal lookahead_x, delta={:.6}", off_x - la_x
        );
        assert!(
            (off_z - la_z).abs() < 1e-3,
            "offset=0: lookahead_offset_z must equal lookahead_z, delta={:.6}", off_z - la_z
        );
        let cmd: f64 = ctx.blackboard.get("lane_follower.steering_cmd").unwrap().parse().unwrap();
        assert!(cmd.abs() < 0.05, "offset=0 on centreline: steering near 0, got {cmd:.4}");
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
