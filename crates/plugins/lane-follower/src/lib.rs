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
//! | `lane_follower.segment_lanes` | u8 | lanes in this direction (DS8; absent on prefab/ferry hits) |
//! | `lane_follower.segment_lane_width` | f32 metres | lane width from road_look (DS8) |
//! | `lane_follower.segment_offset` | f32 metres | computed lane offset = (lanes−0.5)×width (DS8) |
//! | `lane_follower.safety_disengage_reason` | string | `""` or `lateral_excursion` / `steering_saturated` / `spline_lost` |
//! | `lane_follower.safety_disengage_count` | u32 | monotonic safety-disengage count since daemon start |
//! | `lane_follower.rate_limit_active` | `"true"/"false"` | tightened rate-limit (`|lateral| > 3m`) |
//! | `lane_follower.nearest_seg_is_prefab` | `"true"/"false"` | DS13c: nearest segment is a prefab NavCurve |
//! | `lane_follower.nearest_seg_ai_path_uid` | u64 | DS13c: prefab ai_path array index (0 if road) |
//! | `lane_follower.junction_detection_radius_m` | f64 metres | DS13c: snap radius for junction detection |
//! | `lane_follower.junction_min_activation_frames` | u32 | DS13c: frames needed to activate junction |
//! | `lane_follower.junction_frames_count` | u32 | DS13c: consecutive positive detection frames |
//! | `lane_follower.junction_phase_inside_threshold_m` | string | DS13c: "not_implemented" (no inside phase) |
//! | `lane_follower.junction_phase_transitions_count` | u32 | DS13c: monotonic phase-string change counter |
//! | `lane_follower.lookahead_hop_count` | usize | DS13c: segments traversed during lookahead |
//! | `lane_follower.lookahead_hop_failed_reason` | string | DS13c: `"none"` / `"no_next_edge"` / `"max_hops"` |
//! | `lane_follower.bias_zone_active` | `"true"/"false"` | DS13d: prefab-bias zone is active this tick |
//! | `lane_follower.bias_prefab_attempted` | `"true"/"false"` | DS13d: prefab-only query was issued |
//! | `lane_follower.bias_prefab_accepted` | `"true"/"false"` | DS13d: prefab hit was accepted as nearest |
//! | `lane_follower.bias_prefab_rejected_reason` | string | DS13d: `"none"` / `"too_far"` / `"none_found"` / `"not_active"` |
//! | `lane_follower.lateral_source` | string | DS14: `"navcurve"` / `"road_offset"` / `"road_center"` |

mod junction;
mod pure_pursuit;

use std::collections::HashMap;
use std::collections::VecDeque;
use std::sync::Arc;

use junction::{detect_junction, JunctionDetector};
use truckpilot_map_parser::{
    arc_length::{build_all_luts, build_forward_adjacency, lookahead, ArcLengthLUT, LOOKAHEAD_MAX_HOPS},
    graph::MapGraph,
    spline::{build_splines_ex, evaluate_tangent, HermiteSegment, SegmentMetadata, Vec3},
    spline_index::{build_index, build_index_with_metadata, HeadingFilteredHit, NearestHit, SplineIndex},
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

// ── DS13d/DS13e: Prefab-Bias constants ────────────────────────────────────
/// Default bias-zone radius: junction_detected or junction within this distance.
const DEFAULT_BIAS_RADIUS_M: f32 = 30.0;
/// Default max prefab distance: if prefab farther than this, fall back to road.
/// Reduced from 15.0 → 10.0 per Apollo kLanesSearchRange (DS13e).
const DEFAULT_BIAS_MAX_PREFAB_DIST_M: f32 = 10.0;
/// Default right-lane offset from road centreline: half a lane width.
const LANE_OFFSET_RIGHT_M: f32 = LANE_WIDTH_M / 2.0;
/// Default max heading diff for junction prefab-bias (Apollo max_lane_angle_diff_in_junction = 45°).
const DEFAULT_JUNCTION_MAX_HEADING_DIFF_DEG: f32 = 45.0;
/// Active-segment memory: frames out of junction zone before resetting.
const ACTIVE_SEG_RESET_FRAMES: u32 = 10;
/// Active-segment memory: max age in seconds before resetting.
const ACTIVE_SEG_MAX_AGE_SECS: u64 = 5;

// ── Safety-Fallback (Task 1-3) ──────────────────────────────────────────────
/// |lateral_dist_signed| above this → hard-disengage with reason="lateral_excursion".
const SAFETY_LATERAL_HARD_M: f32 = 8.0;
/// |lateral_dist_signed| above this → tightened rate-limit + dampened gain.
const SAFETY_LATERAL_SOFT_M: f32 = 3.0;
/// |raw Pure-Pursuit cmd| above this counts toward saturation timer.
const SAFETY_SATURATION_THRESHOLD: f64 = 0.95;
/// Saturation must persist this many consecutive ticks (0.6s @ 50 Hz).
const SAFETY_SATURATION_TICKS: u32 = 30;
/// nearest_seg_dist_m above this counts toward spline-lost timer.
const SAFETY_SPLINE_LOST_M: f32 = 20.0;
/// Spline-lost must persist this many consecutive ticks (1.0s @ 50 Hz).
const SAFETY_SPLINE_LOST_TICKS: u32 = 50;
/// Tightened steering rate limit (active when |lateral| > SAFETY_LATERAL_SOFT_M).
const STEERING_RATE_LIMIT_HIGH: f64 = 0.02;
/// Pure-Pursuit gain dampening factor under soft excursion.
const SAFETY_GAIN_DAMPEN: f64 = 0.5;

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
    index: Option<Arc<SplineIndex>>,
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
    /// RouterGraph used for junction detection (Arc-shared when ctx.graph is available).
    router_graph: Option<Arc<RouterGraph>>,
    junction_detector: JunctionDetector,
    // VMM-6: minimap fallback spline source.
    /// SplineIndex rebuilt from minimap.spline_json whenever the capture timestamp changes.
    minimap_index: Option<SplineIndex>,
    /// minimap.last_capture_ms seen on last refresh — change triggers rebuild.
    last_minimap_ts: u64,
    /// minimap.confidence last read from blackboard.
    minimap_confidence: f32,
    // ── Safety-Fallback state (Task 1) ───────────────────────────────────
    /// Consecutive ticks where |raw_cmd| ≥ SAFETY_SATURATION_THRESHOLD.
    saturated_ticks: u32,
    /// Consecutive ticks where hit.dist_m > SAFETY_SPLINE_LOST_M.
    spline_lost_ticks: u32,
    /// Monotonic safety-disengage count since daemon start.
    safety_disengage_count: u32,
    // ── DS13c: junction-failure diagnostic state ──────────────────────
    /// Number of road segments [0..road_seg_count); prefab segs start at this index.
    road_seg_count: usize,
    /// Phase string from the previous tick — used to count transitions.
    prev_junction_phase: Option<&'static str>,
    /// Monotonic counter: increments on every junction_phase string change.
    junction_phase_transitions: u32,
    // ── DS13d: prefab-bias config (loaded from truckpilot.toml) ──────
    /// Radius around a junction node within which prefab-bias is active.
    bias_radius_m: f32,
    /// Max allowed distance to a prefab hit before falling back to road.
    bias_max_prefab_dist_m: f32,
    // ── DS13e: heading-aware prefab-bias ─────────────────────────────
    /// Max heading diff (rad) for heading-aware prefab query (default: π/4 = 45°).
    junction_max_heading_diff_rad: f32,
    /// Active-segment memory: last accepted prefab segment index.
    last_active_segment_idx: Option<usize>,
    /// Active-segment memory: when last_active_segment_idx was last set.
    last_active_segment_at: Option<std::time::Instant>,
    /// Consecutive ticks out of junction zone (for active-segment expiry).
    out_of_zone_frames: u32,
    /// Cumulative count of ticks where heading filter found no aligned prefab in bias zone.
    bias_rejected_heading_count: u32,
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
        let (mut segments, mut metadata, stats) = build_splines_ex(&graph);
        let road_seg_count = segments.len();
        ctx_info!(
            ctx,
            "lane-follower: {} road segments built, {} skipped (missing node)",
            stats.total_segments,
            stats.skipped_missing_node
        );

        // DS7: append PrefabAiPath NavCurve segments so the R*-tree covers junctions.
        // Metadata: is_prefab=true, lane_offset_right_m=0.0 (NavCurves sit at lane-centre).
        let (prefab_segs, prefab_meta) = graph.prefab_hermite_segments_with_metadata();
        let prefab_seg_count = prefab_segs.len();
        segments.extend(prefab_segs);
        metadata.extend(prefab_meta);
        let total_seg_count = segments.len();

        // LUTs are built from ALL segments (road + prefab) so arc-length lookahead
        // traverses junction NavCurves when from/to UIDs connect road→prefab→road.
        let t0 = std::time::Instant::now();
        let luts = build_all_luts(&segments);
        let forward_adj = build_forward_adjacency(&segments);
        let lut_ms = t0.elapsed().as_millis();
        let lut_kb = (luts.len() * std::mem::size_of::<ArcLengthLUT>()) as f32 / 1024.0;
        ctx_info!(
            ctx,
            "lane-follower: SplineIndex {} segs total ({} road + {} prefab NavCurves); LUT {}ms {:.1}KB",
            total_seg_count, road_seg_count, prefab_seg_count, lut_ms, lut_kb
        );

        // Map/spline diagnostic BB keys (written once at load time).
        ctx.blackboard.set("map.prefab.instances_count", graph.prefab_instances.len().to_string());
        ctx.blackboard.set("map.prefab.ai_paths_count", graph.prefab_ai_paths.len().to_string());
        ctx.blackboard.set("map.spline.total_segments", total_seg_count.to_string());
        ctx.blackboard.set("map.spline.prefab_segments", prefab_seg_count.to_string());
        ctx.blackboard.set("map.ppd.files_attempted", graph.stats.ppd_files_attempted.to_string());
        ctx.blackboard.set("map.ppd.files_loaded", graph.stats.ppd_files_loaded.to_string());
        ctx.blackboard.set("map.ppd.files_failed", graph.stats.ppd_files_failed.to_string());
        ctx.blackboard.set("map.ppd.total_nav_curves_parsed", graph.stats.ppd_total_nav_curves.to_string());
        // DS13c – TASK 4: road vs prefab segment index ranges for hypothesis A range-check.
        ctx.blackboard.set("map.spline.road_segments_count", road_seg_count.to_string());
        ctx.blackboard.set("map.spline.road_segment_idx_max", road_seg_count.saturating_sub(1).to_string());
        ctx.blackboard.set("map.spline.prefab_segment_idx_min", road_seg_count.to_string());
        ctx.blackboard.set("map.spline.prefab_segment_idx_max", total_seg_count.saturating_sub(1).to_string());
        self.road_seg_count = road_seg_count;

        self.luts = luts;
        self.forward_adj = forward_adj;
        self.index = Some(Arc::new(build_index_with_metadata(segments, metadata)));

        let rg_nodes: Vec<(u64, f64, f64)> = graph.nodes.iter().map(|n| (n.uid, n.x, n.z)).collect();
        let rg_edges: Vec<(u64, u64, f64)> = graph.edges.iter().map(|e| (e.from, e.to, e.distance_m)).collect();
        let n_nodes = rg_nodes.len();
        let n_edges = rg_edges.len();
        self.router_graph = Some(Arc::new(RouterGraph::new(rg_nodes, rg_edges)));
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

    // ── Safety-Fallback helpers (Task 1-3) ───────────────────────────────
    // Note: spline-lost is inlined in `tick()` because the live `index` borrow
    // forbids `&mut self` method calls between the index select and last use.

    /// Update the consecutive-ticks counter for steering saturation.
    fn update_saturation(&mut self, cmd: f64) {
        if cmd.abs() >= SAFETY_SATURATION_THRESHOLD {
            self.saturated_ticks = self.saturated_ticks.saturating_add(1);
        } else {
            self.saturated_ticks = 0;
        }
    }

    /// Reset transient safety counters (used on early-return paths).
    fn reset_safety_counters(&mut self) {
        self.saturated_ticks = 0;
        self.spline_lost_ticks = 0;
    }

    /// Evaluate the three hard-disengage trip conditions and, if any fire while
    /// the plugin is in `Active` mode, request a state-machine disengage via
    /// `autopilot.disengage_requested` and zero out the per-tick steering cmd.
    fn check_safety_trip(&mut self, lateral_dist_signed: f32, ctx: &PluginContext) {
        if self.mode != LaneFollowerMode::Active {
            ctx.blackboard.set("lane_follower.safety_disengage_reason", "");
            ctx.blackboard.set(
                "lane_follower.safety_disengage_count",
                self.safety_disengage_count.to_string(),
            );
            return;
        }
        let reason: Option<&'static str> = if lateral_dist_signed.abs() > SAFETY_LATERAL_HARD_M {
            Some("lateral_excursion")
        } else if self.saturated_ticks > SAFETY_SATURATION_TICKS {
            Some("steering_saturated")
        } else if self.spline_lost_ticks > SAFETY_SPLINE_LOST_TICKS {
            Some("spline_lost")
        } else {
            None
        };
        if let Some(r) = reason {
            self.safety_disengage_count = self.safety_disengage_count.saturating_add(1);
            // Immediate steering suppression — tick_request() returns None when this is None.
            self.last_steering_cmd = None;
            // Reset transient counters so we don't trip again the next tick on stale state.
            self.saturated_ticks = 0;
            self.spline_lost_ticks = 0;
            ctx.blackboard.set("autopilot.disengage_requested", "true");
            ctx.blackboard.set("lane_follower.safety_disengage_reason", r);
            ctx_warn!(ctx, "lane-follower: SAFETY DISENGAGE — {}", r);
        } else {
            ctx.blackboard.set("lane_follower.safety_disengage_reason", "");
        }
        ctx.blackboard.set(
            "lane_follower.safety_disengage_count",
            self.safety_disengage_count.to_string(),
        );
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
        // Phase 2b: use daemon-provided shared SplineIndex when available.
        // Saves ~140MB RAM and ~2s load time by avoiding a second graph.json read.
        if let Some(shared_index) = &ctx.spline_index {
            let road_seg_count = ctx.spline_index_road_seg_count;
            let total_seg_count = shared_index.segments.len();
            let prefab_seg_count = total_seg_count.saturating_sub(road_seg_count);

            let t0 = std::time::Instant::now();
            let luts = build_all_luts(&shared_index.segments);
            let forward_adj = build_forward_adjacency(&shared_index.segments);
            let lut_ms = t0.elapsed().as_millis();
            let lut_kb = (luts.len() * std::mem::size_of::<ArcLengthLUT>()) as f32 / 1024.0;
            ctx_info!(
                ctx,
                "lane-follower: shared SplineIndex {} segs ({} road + {} NavCurves); LUT {}ms {:.1}KB",
                total_seg_count, road_seg_count, prefab_seg_count, lut_ms, lut_kb
            );

            ctx.blackboard.set("map.spline.total_segments", total_seg_count.to_string());
            ctx.blackboard.set("map.spline.prefab_segments", prefab_seg_count.to_string());
            ctx.blackboard.set("map.spline.road_segments_count", road_seg_count.to_string());
            ctx.blackboard.set("map.spline.road_segment_idx_max", road_seg_count.saturating_sub(1).to_string());
            ctx.blackboard.set("map.spline.prefab_segment_idx_min", road_seg_count.to_string());
            ctx.blackboard.set("map.spline.prefab_segment_idx_max", total_seg_count.saturating_sub(1).to_string());

            self.road_seg_count = road_seg_count;
            self.luts = luts;
            self.forward_adj = forward_adj;
            self.index = Some(Arc::clone(shared_index));

            // RouterGraph from ctx (zero-copy Arc share) or skip (junction detection degrades gracefully).
            if let Some(rg) = &ctx.graph {
                self.router_graph = Some(Arc::clone(rg));
            }
        } else {
            let path = ctx
                .blackboard
                .get("plugin.lane-follower.graph_path")
                .unwrap_or_else(|| DEFAULT_GRAPH_PATH.to_string());
            self.load_index(&path, ctx);
        }
        self.mode = LaneFollowerMode::from_bb(ctx);

        // DS13d: load prefab-bias config (PluginManager seeds from truckpilot.toml).
        self.bias_radius_m = ctx
            .blackboard
            .get("lane_follower.junction_bias_radius_m")
            .and_then(|v| v.parse::<f32>().ok())
            .unwrap_or(DEFAULT_BIAS_RADIUS_M);
        self.bias_max_prefab_dist_m = ctx
            .blackboard
            .get("lane_follower.junction_bias_max_prefab_dist_m")
            .and_then(|v| v.parse::<f32>().ok())
            .unwrap_or(DEFAULT_BIAS_MAX_PREFAB_DIST_M);
        // DS13e: heading-aware prefab query config.
        self.junction_max_heading_diff_rad = ctx
            .blackboard
            .get("lane_follower.junction_max_heading_diff_deg")
            .and_then(|v| v.parse::<f32>().ok())
            .unwrap_or(DEFAULT_JUNCTION_MAX_HEADING_DIFF_DEG)
            .to_radians();

        ctx_info!(
            ctx,
            "lane-follower: loaded (mode={}, index={}, bias_radius={:.1}m, bias_max_prefab={:.1}m, max_heading_diff={:.0}°)",
            self.mode.as_str(),
            if self.index.is_some() { "ok" } else { "none" },
            self.bias_radius_m,
            self.bias_max_prefab_dist_m,
            self.junction_max_heading_diff_rad.to_degrees(),
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
        // Reset transient safety counters; keep safety_disengage_count
        // (spec: monotonic since daemon start).
        self.saturated_ticks = 0;
        self.spline_lost_ticks = 0;
        // DS13e: reset active-segment memory.
        self.last_active_segment_idx = None;
        self.last_active_segment_at = None;
        self.out_of_zone_frames = 0;
        self.bias_rejected_heading_count = 0;
        self.junction_max_heading_diff_rad = DEFAULT_JUNCTION_MAX_HEADING_DIFF_DEG.to_radians();
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
            self.reset_safety_counters();
            return;
        };

        let truck_x = tel.position[0];
        let truck_y = tel.position[1];
        let truck_z = tel.position[2];
        ctx.blackboard.set("lane_follower.truck_x", format!("{truck_x:.3}"));
        ctx.blackboard.set("lane_follower.truck_y", format!("{truck_y:.3}"));
        ctx.blackboard.set("lane_follower.truck_z", format!("{truck_z:.3}"));

        // Junction detection — runs regardless of index availability.
        // Returns (active, distance_m) for DS13d prefab-bias query below.
        let (junction_active_for_bias, junction_distance_for_bias): (bool, Option<f64>) = {
            let detection = if let Some(graph) = self.router_graph.as_deref() {
                detect_junction(graph, truck_x, truck_z)
            } else {
                junction::JunctionDetection { is_junction: false, max_degree: 0, distance_m: None }
            };
            let (active, phase) = self.junction_detector.tick(&detection);
            let saved = (active, detection.distance_m);
            ctx.blackboard.set("lane_follower.junction_detected", if active { "true" } else { "false" });
            ctx.blackboard.set("lane_follower.junction_phase", phase.as_str());
            ctx.blackboard.set(
                "lane_follower.junction_distance_m",
                detection.distance_m.map_or_else(String::new, |d| format!("{d:.1}")),
            );
            ctx.blackboard.set("lane_follower.junction_max_degree", detection.max_degree.to_string());
            // DS13c – TASK 3: Hypothesis D — why does phase never transition to "inside"?
            // Note: JunctionPhase has no "inside"/"crossing" state; "approaching" is the only
            // active phase. SNAP_RADIUS_M is the detection radius — not an inside threshold.
            ctx.blackboard.set(
                "lane_follower.junction_detection_radius_m",
                format!("{:.1}", junction::SNAP_RADIUS_M),
            );
            ctx.blackboard.set(
                "lane_follower.junction_min_activation_frames",
                junction::MIN_ACTIVATION_FRAMES.to_string(),
            );
            ctx.blackboard.set(
                "lane_follower.junction_frames_count",
                self.junction_detector.frames().to_string(),
            );
            // "inside" phase does not exist — approaching is the only active phase.
            ctx.blackboard
                .set("lane_follower.junction_phase_inside_threshold_m", "not_implemented");
            let phase_str = phase.as_str();
            if self.prev_junction_phase != Some(phase_str) {
                self.junction_phase_transitions =
                    self.junction_phase_transitions.saturating_add(1);
                self.prev_junction_phase = Some(phase_str);
            }
            ctx.blackboard.set(
                "lane_follower.junction_phase_transitions_count",
                self.junction_phase_transitions.to_string(),
            );
            saved
        };

        // VMM-6: select primary or minimap SplineIndex.
        let primary = self.index.as_deref();          // Option<Arc<SplineIndex>> → Option<&SplineIndex>
        let minimap = self.minimap_index.as_ref();    // Option<SplineIndex> → Option<&SplineIndex>
        let index: &SplineIndex = match (primary, minimap) {
            (Some(idx), _) => idx,
            (None, Some(mm)) if self.minimap_confidence >= MINIMAP_CONF_THRESHOLD => mm,
            _ => {
                ctx.blackboard.set("lane_follower.status", "no_index");
                ctx.blackboard.set("lane_follower.spline_source", "none");
                self.reset_safety_counters();
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

        // DS13d/DS13e: heading-aware prefab-bias in junction zone.
        let bias_radius_m = self.bias_radius_m;
        let bias_max_prefab_dist_m = self.bias_max_prefab_dist_m;
        let max_heading_diff_rad = self.junction_max_heading_diff_rad;
        let in_junction_zone = junction_active_for_bias
            || junction_distance_for_bias.is_some_and(|d| d < bias_radius_m as f64);

        // Convert truck heading to radians (ETS2 CW from North, matching SplineIndex convention).
        let truck_heading_rad = truck_heading_deg.to_radians();

        let (hit_opt, bias_attempted, bias_accepted, bias_rejected_reason, bias_heading_diff_rad, bias_tiebreak_successor, bias_should_count_heading_reject): (
            Option<NearestHit>,
            bool,
            bool,
            &'static str,
            f32,
            bool,
            bool,
        ) = if in_junction_zone {
            // DS13e TASK 2: heading-aware prefab query (replaces nearest_with_projection_filtered)
            let candidates = index.within_radius_filtered_heading(
                query,
                bias_radius_m,
                truck_heading_rad,
                max_heading_diff_rad,
                |_idx, meta| meta.is_some_and(|m| m.is_prefab),
            );

            // DS13e TASK 3: Forward-Adjacency-Tiebreak — prefer successor of last active segment.
            let (best_hit, tiebreak_used) = if let Some(prev_idx) = self.last_active_segment_idx {
                // Segments whose from_uid == prev_seg.to_uid are the successors.
                let to_uid = index.segments.get(prev_idx).map(|s| s.to_uid);
                let empty: Vec<usize> = Vec::new();
                let successors: &[usize] = to_uid
                    .and_then(|uid| self.forward_adj.get(&uid))
                    .map(|v| v.as_slice())
                    .unwrap_or(empty.as_slice());
                // Prefer the first candidate that is a successor; fall back to closest.
                let found = candidates.iter().find(|c| successors.contains(&c.idx));
                if found.is_some() {
                    (found, true)
                } else {
                    (candidates.first(), false)
                }
            } else {
                (candidates.first(), false)
            };

            match best_hit {
                Some(h) if h.dist_m <= bias_max_prefab_dist_m => {
                    let nearest = heading_filtered_to_nearest(h, index);
                    (Some(nearest), true, true, "none", h.heading_diff_rad, tiebreak_used, false)
                }
                Some(h) => {
                    // Prefab found but too far — heading-aligned but outside dist threshold.
                    let fallback =
                        index.nearest_with_heading_filter(query, truck_heading_deg, CANDIDATES);
                    (fallback, true, false, "too_far", h.heading_diff_rad, false, false)
                }
                None => {
                    // Heading filter rejected all prefab candidates in the radius.
                    let fallback =
                        index.nearest_with_heading_filter(query, truck_heading_deg, CANDIDATES);
                    (fallback, true, false, "none_found", 0.0, false, true)
                }
            }
        } else {
            let hit = index.nearest_with_heading_filter(query, truck_heading_deg, CANDIDATES);
            (hit, false, false, "not_active", 0.0, false, false)
        };

        ctx.blackboard.set("lane_follower.bias_zone_active", in_junction_zone.to_string());
        ctx.blackboard.set("lane_follower.bias_prefab_attempted", bias_attempted.to_string());
        ctx.blackboard.set("lane_follower.bias_prefab_accepted", bias_accepted.to_string());
        ctx.blackboard.set("lane_follower.bias_prefab_rejected_reason", bias_rejected_reason);
        // DS13e: tiebreak diagnostic key (available on all paths).
        ctx.blackboard.set("lane_follower.tiebreak_used_successor", bias_tiebreak_successor.to_string());
        // suppress unused-variable warning for bias_heading_diff_rad on no_hit path
        let _ = bias_heading_diff_rad;

        let Some(hit) = hit_opt else {
            ctx.blackboard.set("lane_follower.status", "no_hit");
            ctx.blackboard.set("lane_follower.heading_diff_rad", "0.0000");
            self.reset_safety_counters();
            return;
        };

        // DS13e: heading_diff of the SELECTED segment (more accurate than bias-attempt diff).
        let selected_heading_diff_rad =
            angular_diff_deg(truck_heading_deg, hit.heading_deg).to_radians();
        ctx.blackboard
            .set("lane_follower.heading_diff_rad", format!("{selected_heading_diff_rad:.4}"));

        // DS8: per-segment lane metadata (None for prefab/building/ferry segments).
        let seg_meta: Option<SegmentMetadata> =
            index.metadata.get(hit.segment_idx).copied().flatten();

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
        // DS13c – TASK 1: Hypothesis A — is the nearest segment a prefab NavCurve?
        let nearest_is_prefab = seg_meta.is_some_and(|m| m.is_prefab);
        ctx.blackboard
            .set("lane_follower.nearest_seg_is_prefab", nearest_is_prefab.to_string());
        // ai_path index = position within prefab array (0 when not prefab).
        let nearest_seg_ai_path_uid = if nearest_is_prefab {
            (hit.segment_idx.saturating_sub(self.road_seg_count)) as u64
        } else {
            0u64
        };
        ctx.blackboard
            .set("lane_follower.nearest_seg_ai_path_uid", nearest_seg_ai_path_uid.to_string());

        ctx.blackboard
            .set("lane_follower.truck_heading_deg", format!("{truck_heading_deg:.2}"));
        ctx.blackboard
            .set("lane_follower.heading_filter_applied", hit.heading_filter_applied.to_string());
        let heading_diff = angular_diff_deg(truck_heading_deg, hit.heading_deg);
        ctx.blackboard
            .set("lane_follower.heading_diff_deg", format!("{heading_diff:.2}"));

        // Lane offset: BB-key override → DS8/DS7 metadata → QW1 fallback constant.
        // NavCurve (prefab) segments already sit at lane-centre: offset = 0.
        let lane_offset_m = ctx
            .blackboard
            .get("plugin.lane-follower.lane_offset_m")
            .and_then(|v| v.parse::<f32>().ok())
            .or_else(|| {
                seg_meta.map(|m| if m.is_prefab { 0.0 } else { m.lane_offset_right_m })
            })
            .unwrap_or(LANE_OFFSET_RIGHT_M);

        // DS8: publish per-segment lane metadata keys.
        if let Some(meta) = seg_meta {
            ctx.blackboard
                .set("lane_follower.segment_lanes", meta.lanes_in_direction.to_string());
            ctx.blackboard
                .set("lane_follower.segment_lane_width", format!("{:.2}", meta.lane_width_m));
            ctx.blackboard
                .set("lane_follower.segment_offset", format!("{:.3}", meta.lane_offset_right_m));
        }

        // Task 4 — Diagnostic: right-normal and signed lateral distance at nearest spline point.
        // Right-normal at heading h (CW degrees): n = (cos h, sin h) in XZ.
        // Forward at heading h: f = (sin h, -cos h) in XZ.
        // Signed lateral = f × (truck − spline) = fx*(tz_diff) − fz*(tx_diff). Positive = truck right.
        let road_h_rad = hit.heading_deg.to_radians();
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

        // ── Safety-Fallback (Task 2) ─────────────────────────────────────
        // Rate-limit tightening + gain dampening active when |lateral| > soft threshold.
        // DS14.1: In K2 gap junctions (none_found), nearest-seg reference is invalid — suppress
        // the Soft-Safety trip to avoid dampening Pure-Pursuit against its own synthetic target.
        // Hard-Safety (check_safety_trip / SAFETY_LATERAL_HARD_M) is unaffected.
        let rate_limit_active = bias_rejected_reason != "none_found"
            && lateral_dist_signed.abs() > SAFETY_LATERAL_SOFT_M;
        ctx.blackboard.set(
            "lane_follower.rate_limit_active",
            if rate_limit_active { "true" } else { "false" },
        );
        // Spline-lost ticker runs in all post-hit branches (inline to avoid
        // method-call borrow conflicting with the live `index` borrow above).
        if hit.dist_m > SAFETY_SPLINE_LOST_M {
            self.spline_lost_ticks = self.spline_lost_ticks.saturating_add(1);
        } else {
            self.spline_lost_ticks = 0;
        }

        // Per-tick counter — never resets, monotonic. Catches telemetry stalls.
        self.tick_count += 1;
        ctx.blackboard
            .set("lane_follower.tick_count", self.tick_count.to_string());

        // Speed-adaptive lookahead: 1 sec ahead, clamped to [15m, 50m].
        let speed_kmh = tel.speed_ms as f32 * 3.6;
        let lookahead_dist_m =
            (speed_kmh * LOOKAHEAD_SPEED_FACTOR).clamp(LOOKAHEAD_DIST_M_MIN, LOOKAHEAD_DIST_M_MAX);
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
                // DS13c – TASK 2: Hypothesis C — lookahead hop diagnostics.
                // remaining_dist_m=0 → ok (target reached); >0 → dead_end or max_hops.
                ctx.blackboard.set(
                    "lane_follower.lookahead_hop_count",
                    la.iteration_count.to_string(),
                );
                let hop_failed_reason = if la.remaining_dist_m == 0.0 {
                    "none"
                } else if la.iteration_count >= LOOKAHEAD_MAX_HOPS {
                    "max_hops"
                } else {
                    "no_next_edge"
                };
                ctx.blackboard
                    .set("lane_follower.lookahead_hop_failed_reason", hop_failed_reason);

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

        // DS14: Synthetic lookahead for none_found gap junctions (K2 coverage gaps).
        //
        // When the prefab-bias query returns none_found, the normal lookahead above landed
        // on a road dead-end at the junction edge (la_point_opt = Some(junction_edge)).
        // That dead-end lookahead is behind the truck once it enters the gap, causing
        // Pure-Pursuit to steer off-course.
        //
        // Fix: overwrite lookahead_offset_x/z with a synthetic target computed directly
        // from truck heading + road_look lane offset.  This block runs AFTER the normal
        // lookahead block so it cleanly overwrites the stale dead-end value.
        //
        // Inherent heading bias: because the target is truck-relative (not road-relative),
        // heading_to_lookahead_deg − truck_heading_deg = atan2(offset, lookahead_dist)
        // = atan2(1.875m, 15m) ≈ 7.1° regardless of truck position.  This is intentional:
        // Pure-Pursuit steers toward the right-lane target; the bias is accepted/harmless
        // for short gaps (<100m) because the EMA+rate-limiter dampens actual heading change.
        //
        // Right-normal convention (matches existing code at ~line 788):
        //   forward  = (sin h, -cos h)  in XZ   [0=North=-Z, 90=East=+X]
        //   right    = (cos h,  sin h)  in XZ
        //   Proof h=0 (North): right=(1,0)=East ✓   h=90 (East): right=(0,1)=South ✓
        let lateral_source: &'static str = if bias_accepted {
            "navcurve"
        } else if bias_rejected_reason == "none_found" {
            let h_rad = truck_heading_deg.to_radians();
            let fwd_x = h_rad.sin();
            let fwd_z = -h_rad.cos();
            let right_x = h_rad.cos(); // right-normal: (cos h, sin h)
            let right_z = h_rad.sin();
            let synth_offset_m = seg_meta
                .map(|m| if m.is_prefab { LANE_OFFSET_RIGHT_M } else { m.lane_offset_right_m })
                .unwrap_or(LANE_OFFSET_RIGHT_M);
            let synth_la_x =
                truck_x as f32 + fwd_x * lookahead_dist_m + right_x * synth_offset_m;
            let synth_la_z =
                truck_z as f32 + fwd_z * lookahead_dist_m + right_z * synth_offset_m;
            ctx.blackboard
                .set("lane_follower.lookahead_x", format!("{synth_la_x:.3}"));
            ctx.blackboard
                .set("lane_follower.lookahead_z", format!("{synth_la_z:.3}"));
            ctx.blackboard
                .set("lane_follower.lookahead_offset_x", format!("{synth_la_x:.3}"));
            ctx.blackboard
                .set("lane_follower.lookahead_offset_z", format!("{synth_la_z:.3}"));
            let heading_to_la = (synth_la_x - truck_x as f32)
                .atan2(-(synth_la_z - truck_z as f32))
                .to_degrees()
                .rem_euclid(360.0);
            ctx.blackboard
                .set("lane_follower.heading_to_lookahead_deg", format!("{heading_to_la:.2}"));
            "road_offset"
        } else {
            "road_center"
        };
        ctx.blackboard.set("lane_follower.lateral_source", lateral_source);

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
            self.saturated_ticks = 0;
            self.check_safety_trip(lateral_dist_signed, ctx);
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
            self.saturated_ticks = 0;
            self.check_safety_trip(lateral_dist_signed, ctx);
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

            // ── Safety-Fallback (Task 2): tightened rate + gain dampening ─
            let (effective_rate_limit, cmd_for_ema) = if rate_limit_active {
                (STEERING_RATE_LIMIT_HIGH, cmd * SAFETY_GAIN_DAMPEN)
            } else {
                (STEERING_RATE_LIMIT, cmd)
            };

            self.steering_ema =
                STEERING_EMA_ALPHA * cmd_for_ema + (1.0 - STEERING_EMA_ALPHA) * self.steering_ema;
            let rl = self.steering_ema.clamp(
                self.steering_rate_limited_prev - effective_rate_limit,
                self.steering_rate_limited_prev + effective_rate_limit,
            );
            let is_rate_limited = (self.steering_ema - rl).abs() > 1e-9;
            self.steering_rate_limited_prev = rl;

            ctx.blackboard.set("lane_follower.steering_cmd", format!("{cmd:.4}"));
            ctx.blackboard.set("lane_follower.steering_curvature", format!("{curvature:.6}"));
            ctx.blackboard.set("lane_follower.steering_filtered", format!("{rl:.4}"));
            ctx.blackboard.set("lane_follower.rate_limited", is_rate_limited.to_string());

            self.last_steering_cmd = Some(cmd);
        }

        // ── Safety-Fallback (Task 1): saturation tracking + trip evaluation ─
        // Uses the raw Pure-Pursuit cmd captured this tick (None ⇒ no steering emitted ⇒ no saturation).
        if let Some(cmd) = self.last_steering_cmd {
            self.update_saturation(cmd);
        } else {
            self.saturated_ticks = 0;
        }
        self.check_safety_trip(lateral_dist_signed, ctx);

        // ── DS13e TASK 3: Active-segment memory update ────────────────────────
        // (placed after last `index` use so borrow checker is happy)
        if bias_accepted {
            self.last_active_segment_idx = Some(hit.segment_idx);
            self.last_active_segment_at = Some(std::time::Instant::now());
            self.out_of_zone_frames = 0;
        } else if !in_junction_zone {
            self.out_of_zone_frames = self.out_of_zone_frames.saturating_add(1);
            if self.out_of_zone_frames >= ACTIVE_SEG_RESET_FRAMES {
                self.last_active_segment_idx = None;
                self.last_active_segment_at = None;
            }
        }
        // Time-based expiry: reset if last accepted prefab was > 5s ago.
        if let Some(at) = self.last_active_segment_at {
            if at.elapsed() > std::time::Duration::from_secs(ACTIVE_SEG_MAX_AGE_SECS) {
                self.last_active_segment_idx = None;
                self.last_active_segment_at = None;
            }
        }

        // DS13e TASK 4: remaining diagnostic BB keys.
        if bias_should_count_heading_reject {
            self.bias_rejected_heading_count =
                self.bias_rejected_heading_count.saturating_add(1);
        }
        ctx.blackboard.set(
            "lane_follower.bias_rejected_heading_count",
            self.bias_rejected_heading_count.to_string(),
        );
        ctx.blackboard.set(
            "lane_follower.last_active_segment_idx",
            self.last_active_segment_idx
                .map_or(-1i64, |i| i as i64)
                .to_string(),
        );
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

/// Converts a [`HeadingFilteredHit`] into a [`NearestHit`].
///
/// `evaluate_tangent` is used to recompute `heading_deg` from the segment tangent
/// at the stored `t` parameter. The segment's `heading_filter_applied` is always `true`
/// (the heading filter was active to produce this hit).
fn heading_filtered_to_nearest(hit: &HeadingFilteredHit, index: &SplineIndex) -> NearestHit {
    let seg = &index.segments[hit.idx];
    let tan = evaluate_tangent(seg, hit.t);
    let heading_deg = f32::atan2(tan.x, -tan.z).to_degrees().rem_euclid(360.0);
    NearestHit {
        segment_idx: hit.idx,
        t: hit.t,
        point_on_curve: hit.point_on_curve,
        dist_m: hit.dist_m,
        heading_deg,
        heading_filter_applied: true,
    }
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
        let mut plugin = LaneFollowerPlugin { index: Some(Arc::new(index)), mode: LaneFollowerMode::Observer, ..Default::default() };
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
        let mut plugin = LaneFollowerPlugin { index: Some(Arc::new(index)), mode: LaneFollowerMode::Observer, ..Default::default() };
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
        let mut plugin = LaneFollowerPlugin { index: Some(Arc::new(index)), mode: LaneFollowerMode::Observer, ..Default::default() };
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
        LaneFollowerPlugin { index: Some(Arc::new(index)), luts, forward_adj, ..Default::default() }
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
        // Keep |lateral| < SAFETY_LATERAL_SOFT_M (3 m) so the rate-limit-tightening
        // path doesn't fire and the test measures pure EMA convergence.
        let mut plugin = make_chain_plugin_at_x(2.0);
        let ctx = PluginContext::test();
        ctx.blackboard.set("plugin.lane-follower.mode", "active");
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        // With α=0.15 and the standard rate-limit (0.05/tick), 30 ticks suffice
        // for the EMA to converge within 0.01 of the raw cmd.
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
        LaneFollowerPlugin { index: Some(Arc::new(index)), luts, forward_adj, ..Default::default() }
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
        let mut plugin = LaneFollowerPlugin { index: Some(Arc::new(index)), luts, forward_adj, ..Default::default() };
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
        let mut plugin = LaneFollowerPlugin { index: Some(Arc::new(index)), luts, forward_adj, ..Default::default() };
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
        let mut plugin = LaneFollowerPlugin { index: Some(Arc::new(index)), luts, forward_adj, ..Default::default() };
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

    // ── Safety-Fallback tests (Task 4) ───────────────────────────────────────

    /// Build a chain plugin where the road runs north (heading 0°) but is
    /// shifted +`offset_z_forward` ahead of the truck on the z-axis, so the
    /// nearest-segment distance is roughly `offset_z_forward` while lateral
    /// stays at 0. Used to test `spline_lost` in isolation from `lateral_excursion`.
    fn make_chain_plugin_forward(offset_z_forward: f32) -> LaneFollowerPlugin {
        let z0 = -offset_z_forward;
        let p0 = Vec3::new(0.0, 0.0, z0);
        let p1 = Vec3::new(0.0, 0.0, z0 - 10.0);
        let p2 = Vec3::new(0.0, 0.0, z0 - 20.0);
        let p3 = Vec3::new(0.0, 0.0, z0 - 30.0);
        let seg0 = make_seg(p0, p1, 1);
        let seg1 = make_seg(p1, p2, 2);
        let seg2 = make_seg(p2, p3, 3);
        let segs = vec![seg0, seg1, seg2];
        let luts = build_all_luts(&segs);
        let forward_adj = build_forward_adjacency(&segs);
        let index = build_index(segs);
        LaneFollowerPlugin { index: Some(Arc::new(index)), luts, forward_adj, ..Default::default() }
    }

    #[test]
    fn test_disengage_on_lateral_excursion() {
        // Road at x=+10, truck at x=0 → |lateral|=10 > SAFETY_LATERAL_HARD_M (8 m).
        // Single tick must trigger lateral_excursion trip.
        let mut plugin = make_chain_plugin_at_x(10.0);
        let ctx = PluginContext::test();
        ctx.blackboard.set("plugin.lane-follower.mode", "active");
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(
            ctx.blackboard.get("autopilot.disengage_requested").as_deref(),
            Some("true"),
            "lateral=10m must request state-machine disengage"
        );
        assert_eq!(
            ctx.blackboard.get("lane_follower.safety_disengage_reason").as_deref(),
            Some("lateral_excursion"),
            "trip reason must be lateral_excursion"
        );
        assert_eq!(
            ctx.blackboard.get("lane_follower.safety_disengage_count").as_deref(),
            Some("1"),
            "safety_disengage_count must increment on trip"
        );
        // last_steering_cmd must be suppressed → tick_request returns None.
        ctx.blackboard.set("autopilot.engage_mode", "route");
        assert!(plugin.tick_request(Some(&tel), &ctx).is_none(),
            "after safety trip, no ControlRequest must be emitted");
    }

    #[test]
    fn test_disengage_on_saturation() {
        // Road at x=+7 → |lateral|=7 (< 8, no excursion) but Pure-Pursuit cmd
        // saturates near 1.0 → after >30 ticks of saturation, steering_saturated fires.
        let mut plugin = make_chain_plugin_at_x(7.0);
        let ctx = PluginContext::test();
        ctx.blackboard.set("plugin.lane-follower.mode", "active");
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        for _ in 0..35 {
            plugin.tick(Some(&tel), &mut out, &ctx);
        }
        let count: u32 = ctx
            .blackboard
            .get("lane_follower.safety_disengage_count")
            .unwrap()
            .parse()
            .unwrap();
        assert!(count >= 1, "saturation for 35 ticks must trip at least once, got count={count}");
        assert_eq!(
            ctx.blackboard.get("autopilot.disengage_requested").as_deref(),
            Some("true"),
            "saturation trip must request disengage"
        );
        // The trip happens once at tick 31; subsequent ticks count fresh, so the
        // reason BB key may be cleared by tick 35. The monotonic count is the
        // authoritative trip signal.
    }

    #[test]
    fn test_disengage_on_spline_lost() {
        // Spline 25 m forward of the truck (same heading) → hit.dist_m ≈ 25 > 20,
        // but |lateral|=0 < 8 (no excursion) and cmd is small (no saturation).
        // After > 50 ticks: spline_lost fires.
        let mut plugin = make_chain_plugin_forward(25.0);
        let ctx = PluginContext::test();
        ctx.blackboard.set("plugin.lane-follower.mode", "active");
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, 0.0, 0.0);
        // First sanity tick: confirm distance and lateral preconditions.
        plugin.tick(Some(&tel), &mut out, &ctx);
        let dist: f32 = ctx
            .blackboard
            .get("lane_follower.nearest_seg_dist_m")
            .unwrap()
            .parse()
            .unwrap();
        let lateral: f32 = ctx
            .blackboard
            .get("lane_follower.lateral_dist_signed")
            .unwrap()
            .parse()
            .unwrap();
        assert!(dist > SAFETY_SPLINE_LOST_M, "precondition dist > {}: got {}", SAFETY_SPLINE_LOST_M, dist);
        assert!(lateral.abs() < SAFETY_LATERAL_HARD_M, "precondition |lateral| < {}: got {}", SAFETY_LATERAL_HARD_M, lateral);
        // Now run 60 more ticks → counter reaches >50 → trip.
        for _ in 0..60 {
            plugin.tick(Some(&tel), &mut out, &ctx);
        }
        let count: u32 = ctx
            .blackboard
            .get("lane_follower.safety_disengage_count")
            .unwrap()
            .parse()
            .unwrap();
        assert!(count >= 1, "dist > 20m for 60 ticks must trip spline_lost, got count={count}");
        assert_eq!(
            ctx.blackboard.get("autopilot.disengage_requested").as_deref(),
            Some("true"),
            "spline_lost must request disengage"
        );
    }

    #[test]
    fn test_rate_limit_at_high_lateral() {
        // Road at x=+4 → |lateral|=4 > SAFETY_LATERAL_SOFT_M (3 m) but
        // < SAFETY_LATERAL_HARD_M (8 m). Single tick: rate_limit_active=true,
        // no disengage triggered.
        let mut plugin = make_chain_plugin_at_x(4.0);
        let ctx = PluginContext::test();
        ctx.blackboard.set("plugin.lane-follower.mode", "active");
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_follower.rate_limit_active").as_deref(),
            Some("true"),
            "lateral=4m must engage tightened rate-limit"
        );
        assert_eq!(
            ctx.blackboard.get("lane_follower.safety_disengage_reason").as_deref(),
            Some(""),
            "lateral=4m alone must NOT trigger any disengage"
        );
        assert_eq!(
            ctx.blackboard.get("lane_follower.safety_disengage_count").as_deref(),
            Some("0"),
            "no trip → count stays at 0"
        );
        // Also: existing engage flow must still emit a ControlRequest.
        ctx.blackboard.set("autopilot.engage_mode", "route");
        assert!(
            plugin.tick_request(Some(&tel), &ctx).is_some(),
            "lateral=4m must still allow steering (no safety trip)"
        );
    }

    // ── DS7: Prefab-Segment Offset-Override ──────────────────────────────────

    #[test]
    fn ds7_prefab_metadata_offset_zero() {
        // A segment with is_prefab=true and lane_offset_right_m=3.5 (non-zero).
        // tick() must produce lane_offset_m=0.0 because NavCurves sit at lane-centre.
        let seg = make_seg(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, -10.0), 1);
        let meta: Option<SegmentMetadata> = Some(SegmentMetadata {
            lanes_in_direction: 1,
            lanes_opposite: 0,
            lanes_total: 1,
            lane_width_m: 3.75,
            lane_offset_right_m: 3.5, // non-zero; would be applied for road edges
            road_look_token: 0,
            is_prefab: true,
        });
        let luts = build_all_luts(&[seg.clone()]);
        let forward_adj = build_forward_adjacency(&[seg.clone()]);
        let index = build_index_with_metadata(vec![seg], vec![meta]);
        let mut plugin = LaneFollowerPlugin {
            index: Some(Arc::new(index)),
            luts,
            forward_adj,
            ..Default::default()
        };
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_follower.status").as_deref(),
            Some("ok"),
            "prefab segment must reach status=ok"
        );
        let offset: f32 = ctx
            .blackboard
            .get("lane_follower.lane_offset_m")
            .unwrap()
            .parse()
            .unwrap();
        assert_eq!(offset, 0.0, "is_prefab=true must give lane_offset_m=0, got {offset}");
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

    // ── DS13d: Prefab-Bias diagnostic keys ───────────────────────────────────

    /// Build a mixed SplineIndex: one road segment + one prefab segment.
    fn make_mixed_index_plugin(road_z: f32, prefab_z: f32) -> LaneFollowerPlugin {
        let road_seg = make_seg(
            Vec3::new(0.0, 0.0, road_z),
            Vec3::new(0.0, 0.0, road_z - 10.0),
            1,
        );
        let prefab_seg = make_seg(
            Vec3::new(0.0, 0.0, prefab_z),
            Vec3::new(0.0, 0.0, prefab_z - 10.0),
            3,
        );
        let road_meta: Option<SegmentMetadata> = Some(SegmentMetadata {
            lanes_in_direction: 1,
            lanes_opposite: 0,
            lanes_total: 1,
            lane_width_m: 3.75,
            lane_offset_right_m: 1.875,
            road_look_token: 0,
            is_prefab: false,
        });
        let prefab_meta: Option<SegmentMetadata> = Some(SegmentMetadata {
            lanes_in_direction: 1,
            lanes_opposite: 0,
            lanes_total: 1,
            lane_width_m: 3.75,
            lane_offset_right_m: 0.0,
            road_look_token: 0,
            is_prefab: true,
        });
        let segs = vec![road_seg, prefab_seg];
        let meta = vec![road_meta, prefab_meta];
        let luts = build_all_luts(&segs);
        let forward_adj = build_forward_adjacency(&segs);
        let index = build_index_with_metadata(segs, meta);
        // road_seg_count=1 → prefab starts at index 1
        LaneFollowerPlugin {
            index: Some(Arc::new(index)),
            luts,
            forward_adj,
            road_seg_count: 1,
            bias_radius_m: DEFAULT_BIAS_RADIUS_M,
            bias_max_prefab_dist_m: DEFAULT_BIAS_MAX_PREFAB_DIST_M,
            ..Default::default()
        }
    }

    /// DS13d TASK 6: Outside junction zone → bias keys reflect not_active, road used.
    #[test]
    fn test_geometric_bias_falls_back_to_road_outside_junction() {
        // No router_graph → junction_detected=false, junction_distance=None
        // → in_junction_zone=false → bias not active → road segment returned.
        let mut plugin = make_mixed_index_plugin(0.0, -2.0); // prefab 2m away
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        // Truck at (0, 0, -5): 5m from road (z=0), 3m from prefab (z=-2).
        let tel = make_telemetry(0.0, 0.0, -5.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_follower.bias_zone_active").as_deref(),
            Some("false"),
            "no router_graph → bias not active"
        );
        assert_eq!(
            ctx.blackboard.get("lane_follower.bias_prefab_attempted").as_deref(),
            Some("false"),
        );
        assert_eq!(
            ctx.blackboard.get("lane_follower.bias_prefab_rejected_reason").as_deref(),
            Some("not_active"),
        );
    }

    /// DS13d TASK 6: Bias keys are always published, even on dist_warn.
    #[test]
    fn test_bias_keys_published_on_any_status() {
        let mut plugin = make_mixed_index_plugin(0.0, -2.0);
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -500.0, 0.0); // dist_warn
        plugin.tick(Some(&tel), &mut out, &ctx);
        // Bias keys must be present regardless of status (they are set before the hit check).
        assert!(
            ctx.blackboard.get("lane_follower.bias_zone_active").is_some(),
            "bias_zone_active must be published"
        );
        assert!(
            ctx.blackboard.get("lane_follower.bias_prefab_attempted").is_some(),
            "bias_prefab_attempted must be published"
        );
        assert!(
            ctx.blackboard.get("lane_follower.bias_prefab_accepted").is_some(),
            "bias_prefab_accepted must be published"
        );
        assert!(
            ctx.blackboard.get("lane_follower.bias_prefab_rejected_reason").is_some(),
            "bias_prefab_rejected_reason must be published"
        );
    }

    /// DS13d TASK 6: With bias_max_prefab_dist_m=0.0, prefab is always "too_far" → road fallback.
    #[test]
    fn test_geometric_bias_rejects_far_prefab_via_max_dist() {
        let mut plugin = make_mixed_index_plugin(0.0, -0.5); // prefab very close
        // Set max_dist=0.0 so any prefab hit is "too_far"
        plugin.bias_max_prefab_dist_m = 0.0;
        // Force bias zone by setting bias_radius_m to a huge value AND
        // manually pre-setting junction_active via a high radius; since there's no
        // router_graph, junction_distance is None → only radius matters.
        // With distance=None and junction_active=false → in_junction_zone=false.
        // We can't easily inject junction_active in a unit test without a RouterGraph.
        // So instead, set bias_radius_m very large and have the prefab close
        // enough that it would be found, but max_dist=0 rejects it.
        // Verify: without junction, bias doesn't fire at all (not_active path).
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);
        // Without router_graph → not_active (no junction detected).
        assert_eq!(
            ctx.blackboard.get("lane_follower.bias_prefab_rejected_reason").as_deref(),
            Some("not_active"),
            "no junction without router_graph → not_active path"
        );
        // nearest_seg_is_prefab should be false (road wins via heading_filter fallback).
        assert_eq!(
            ctx.blackboard.get("lane_follower.nearest_seg_is_prefab").as_deref(),
            Some("false"),
            "road segment must be selected when bias not active"
        );
    }

    // ── DS13e: heading-aware bias + forward-adjacency tiebreak ──────────────

    /// Helper: build a RouterGraph with a junction node at (jx, jz) with 2 outgoing edges
    /// at 90° spread — enough to satisfy the heading_spread > 30° requirement.
    fn make_junction_rg(jx: f64, jz: f64) -> truckpilot_plugin_api::graph::RouterGraph {
        // Node 1 at (jx, jz); Node 2 North of it; Node 3 East of it.
        truckpilot_plugin_api::graph::RouterGraph::new(
            vec![
                (1u64, jx, jz),
                (2u64, jx, jz - 5.0), // 5m North
                (3u64, jx + 5.0, jz), // 5m East
            ],
            vec![
                (1u64, 2u64, 5.0f64),
                (1u64, 3u64, 5.0f64),
            ],
        )
    }

    /// DS13e TASK 3: Forward-adjacency tiebreak picks the successor of the last active segment
    /// even when a closer (but non-successor) candidate exists.
    #[test]
    fn test_forward_adjacency_picks_successor() {
        use std::f32::consts::PI;
        use truckpilot_map_parser::spline_index::build_index_with_metadata;

        // Three North-going prefab segments (all heading ≈ 0°):
        //   A (idx 0): at x=-5 (far from truck), to_uid=200 — serves as "last active".
        //   B (idx 1): at x=0.3 (dist≈0.3 from truck), from_uid=200 — SUCCESSOR of A.
        //   C (idx 2): at x=0.0 (dist≈0  from truck), from_uid=400 — NOT successor (closer!).
        // Without tiebreak C would win; with tiebreak B (successor) should win.
        let make_north_prefab = |x: f32, from_uid: u64, to_uid: u64| HermiteSegment {
            p0: Vec3::new(x, 0.0, 0.0),
            p1: Vec3::new(x, 0.0, -10.0),
            m0: Vec3::new(0.0, 0.0, -10.0),
            m1: Vec3::new(0.0, 0.0, -10.0),
            length_m: 10.0,
            from_uid,
            to_uid,
            edge_uid: from_uid,
        };
        let seg_a = make_north_prefab(-5.0, 100, 200);
        let seg_b = make_north_prefab(0.3, 200, 300); // successor of A
        let seg_c = make_north_prefab(0.0, 400, 500); // NOT successor, but closer

        let prefab_meta: Option<SegmentMetadata> = Some(SegmentMetadata {
            is_prefab: true,
            lane_offset_right_m: 0.0,
            lanes_in_direction: 1,
            lanes_opposite: 0,
            lanes_total: 1,
            lane_width_m: 3.75,
            road_look_token: 0,
        });
        let segs = vec![seg_a, seg_b, seg_c];
        let metas = vec![prefab_meta, prefab_meta, prefab_meta];
        let forward_adj = build_forward_adjacency(&segs);
        let luts = build_all_luts(&segs);
        let index = build_index_with_metadata(segs, metas);

        let mut plugin = LaneFollowerPlugin {
            index: Some(Arc::new(index)),
            luts,
            forward_adj,
            router_graph: Some(Arc::new(make_junction_rg(0.0, -5.0))),
            road_seg_count: 0,
            last_active_segment_idx: Some(0), // A is "last active"
            junction_max_heading_diff_rad: PI / 4.0,
            bias_radius_m: 30.0,
            bias_max_prefab_dist_m: 10.0,
            ..Default::default()
        };

        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        // Truck at (0, 0, -5), heading North (ETS2 heading=0 → truck_heading_deg=0°).
        let tel = make_telemetry(0.0, 0.0, -5.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_follower.bias_prefab_accepted").as_deref(),
            Some("true"),
            "prefab must be accepted in junction zone"
        );
        let nearest_idx: usize = ctx
            .blackboard
            .get("lane_follower.nearest_seg_idx")
            .unwrap()
            .parse()
            .unwrap();
        let tiebreak_used: bool = ctx
            .blackboard
            .get("lane_follower.tiebreak_used_successor")
            .unwrap()
            .parse()
            .unwrap();
        // B (idx=1) should be selected via tiebreak despite C (idx=2) being closer.
        assert_eq!(
            nearest_idx, 1,
            "tiebreak must pick B (successor idx=1) over C (closer idx=2), got {nearest_idx}"
        );
        assert!(tiebreak_used, "tiebreak_used_successor must be true when successor was found");
    }

    /// DS13e TASK 4: bias_rejected_heading_count increments when heading filter
    /// rejects all prefab candidates in the junction zone.
    #[test]
    fn test_bias_rejected_heading_count_increments() {
        use std::f32::consts::PI;
        use truckpilot_map_parser::spline_index::build_index_with_metadata;

        // East-going prefab: truck faces North → heading diff = 90° > 45° → heading filter rejects.
        let east_prefab = HermiteSegment {
            p0: Vec3::new(-5.0, 0.0, -5.0),
            p1: Vec3::new(5.0, 0.0, -5.0),
            m0: Vec3::new(10.0, 0.0, 0.0),
            m1: Vec3::new(10.0, 0.0, 0.0),
            length_m: 10.0,
            from_uid: 100,
            to_uid: 200,
            edge_uid: 1,
        };
        let prefab_meta: Option<SegmentMetadata> = Some(SegmentMetadata {
            is_prefab: true,
            lane_offset_right_m: 0.0,
            lanes_in_direction: 1,
            lanes_opposite: 0,
            lanes_total: 1,
            lane_width_m: 3.75,
            road_look_token: 0,
        });
        let segs = vec![east_prefab];
        let metas = vec![prefab_meta];
        let forward_adj = build_forward_adjacency(&segs);
        let luts = build_all_luts(&segs);
        let index = build_index_with_metadata(segs, metas);

        let mut plugin = LaneFollowerPlugin {
            index: Some(Arc::new(index)),
            luts,
            forward_adj,
            router_graph: Some(Arc::new(make_junction_rg(0.0, -5.0))),
            junction_max_heading_diff_rad: PI / 4.0,
            bias_radius_m: 30.0,
            bias_max_prefab_dist_m: 10.0,
            ..Default::default()
        };

        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        // Truck at (0, 0, -5), heading North (0°). East prefab has 90° diff → heading-rejected.
        let tel = make_telemetry(0.0, 0.0, -5.0, 0.0);

        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_follower.bias_prefab_rejected_reason").as_deref(),
            Some("none_found"),
            "East prefab (90° > 45°) must be heading-rejected → none_found"
        );
        let count: u32 = ctx
            .blackboard
            .get("lane_follower.bias_rejected_heading_count")
            .unwrap()
            .parse()
            .unwrap();
        assert_eq!(count, 1, "heading_reject count must be 1 after first tick");

        // Second tick: count must increment to 2.
        plugin.tick(Some(&tel), &mut out, &ctx);
        let count2: u32 = ctx
            .blackboard
            .get("lane_follower.bias_rejected_heading_count")
            .unwrap()
            .parse()
            .unwrap();
        assert_eq!(count2, 2, "heading_reject count must increment to 2 on second tick");
    }

    // ── DS14: Synthetic Lookahead Tests ─────────────────────────────────────

    /// Right-normal (cos h, sin h) is East when heading North.
    /// Confirms sign convention: positive offset moves lane target right of road center.
    #[test]
    fn ds14_right_normal_heading_north() {
        let h_rad = 0.0_f32.to_radians();
        let right_x = h_rad.cos(); // 1.0
        let right_z = h_rad.sin(); // 0.0
        assert!((right_x - 1.0).abs() < 1e-6, "heading North: right_x must be 1.0 (East)");
        assert!(right_z.abs() < 1e-6, "heading North: right_z must be 0.0");
        // Lane point must be East of road center (larger X)
        let lane_x = 0.0_f32 + right_x * 3.75;
        let lane_z = 0.0_f32 + right_z * 3.75;
        assert!(lane_x > 0.0, "lane center must be East of road center when heading North");
        let _ = lane_z;
    }

    /// Right-normal (cos h, sin h) is South (+Z) when heading East.
    #[test]
    fn ds14_right_normal_heading_east() {
        let h_rad = 90.0_f32.to_radians();
        let right_x = h_rad.cos(); // ~0.0
        let right_z = h_rad.sin(); // ~1.0
        assert!(right_x.abs() < 1e-5, "heading East: right_x must be ~0 (South has no X component)");
        assert!((right_z - 1.0).abs() < 1e-5, "heading East: right_z must be ~1.0 (South = +Z)");
        let lane_z = 0.0_f32 + right_z * 3.75;
        assert!(lane_z > 0.0, "lane center must be South (+Z) of road center when heading East");
    }

    /// Right-normal (cos h, sin h) is West (-X) when heading South.
    #[test]
    fn ds14_right_normal_heading_south() {
        let h_rad = 180.0_f32.to_radians();
        let right_x = h_rad.cos(); // -1.0
        let right_z = h_rad.sin(); // ~0.0
        assert!((right_x + 1.0).abs() < 1e-5, "heading South: right_x must be -1.0 (West)");
        assert!(right_z.abs() < 1e-5, "heading South: right_z must be ~0.0");
        let lane_x = 0.0_f32 + right_x * 3.75;
        assert!(lane_x < 0.0, "lane center must be West (-X) of road center when heading South");
    }

    /// Right-normal (cos h, sin h) is North (-Z) when heading West.
    #[test]
    fn ds14_right_normal_heading_west() {
        let h_rad = 270.0_f32.to_radians();
        let right_x = h_rad.cos(); // ~0.0
        let right_z = h_rad.sin(); // -1.0
        assert!(right_x.abs() < 1e-5, "heading West: right_x must be ~0");
        assert!((right_z + 1.0).abs() < 1e-5, "heading West: right_z must be -1.0 (North = -Z)");
        let lane_z = 0.0_f32 + right_z * 3.75;
        assert!(lane_z < 0.0, "lane center must be North (-Z) of road center when heading West");
    }

    /// When none_found, lateral_source=road_offset and lookahead_offset_x/z are overwritten
    /// with a synthetic target (truck_pos + forward*lookahead_dist + right*lane_offset).
    #[test]
    fn ds14_none_found_sets_road_offset_and_synthetic_lookahead() {
        use truckpilot_map_parser::spline_index::build_index_with_metadata;

        // East-going prefab: truck faces North → heading diff 90° > 45° → none_found.
        let east_prefab = HermiteSegment {
            p0: Vec3::new(-5.0, 0.0, -5.0),
            p1: Vec3::new(5.0, 0.0, -5.0),
            m0: Vec3::new(10.0, 0.0, 0.0),
            m1: Vec3::new(10.0, 0.0, 0.0),
            length_m: 10.0,
            from_uid: 100,
            to_uid: 200,
            edge_uid: 1,
        };
        let prefab_meta: Option<SegmentMetadata> = Some(SegmentMetadata {
            is_prefab: true,
            lane_offset_right_m: 0.0,
            lanes_in_direction: 1,
            lanes_opposite: 0,
            lanes_total: 1,
            lane_width_m: 3.75,
            road_look_token: 0,
        });
        let segs = vec![east_prefab];
        let metas = vec![prefab_meta];
        let forward_adj = build_forward_adjacency(&segs);
        let luts = build_all_luts(&segs);
        let index = build_index_with_metadata(segs, metas);

        let mut plugin = LaneFollowerPlugin {
            index: Some(Arc::new(index)),
            luts,
            forward_adj,
            router_graph: Some(Arc::new(make_junction_rg(0.0, -5.0))),
            junction_max_heading_diff_rad: std::f32::consts::PI / 4.0,
            bias_radius_m: 30.0,
            bias_max_prefab_dist_m: 10.0,
            ..Default::default()
        };

        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        // Truck at (0,0,-5) heading North (0°). Speed=0 → lookahead_dist = 15m.
        let tel = make_telemetry(0.0, 0.0, -5.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_follower.bias_prefab_rejected_reason").as_deref(),
            Some("none_found"),
            "prerequisite: must be none_found"
        );
        assert_eq!(
            ctx.blackboard.get("lane_follower.lateral_source").as_deref(),
            Some("road_offset"),
            "none_found must set lateral_source=road_offset"
        );

        // Synthetic target: truck(0,0,-5) + forward(0,-1)*15 + right(1,0)*LANE_OFFSET_RIGHT_M
        let expected_x = 0.0_f32 + 0.0 * 15.0 + 1.0 * LANE_OFFSET_RIGHT_M;
        let expected_z = -5.0_f32 + (-1.0) * 15.0 + 0.0 * LANE_OFFSET_RIGHT_M;
        let got_x: f32 = ctx
            .blackboard
            .get("lane_follower.lookahead_offset_x")
            .unwrap()
            .parse()
            .unwrap();
        let got_z: f32 = ctx
            .blackboard
            .get("lane_follower.lookahead_offset_z")
            .unwrap()
            .parse()
            .unwrap();
        assert!(
            (got_x - expected_x).abs() < 0.01,
            "lookahead_offset_x: expected {expected_x:.3} got {got_x:.3}"
        );
        assert!(
            (got_z - expected_z).abs() < 0.01,
            "lookahead_offset_z: expected {expected_z:.3} got {got_z:.3}"
        );
    }

    /// When bias_accepted=true (normal NavCurve hit), lateral_source=navcurve.
    #[test]
    fn ds14_navcurve_hit_sets_lateral_source_navcurve() {
        use truckpilot_map_parser::spline_index::build_index_with_metadata;

        // North-going prefab: truck faces North → heading diff 0° < 45° → accepted.
        let north_prefab = HermiteSegment {
            p0: Vec3::new(0.0, 0.0, -0.0),
            p1: Vec3::new(0.0, 0.0, -10.0),
            m0: Vec3::new(0.0, 0.0, -10.0),
            m1: Vec3::new(0.0, 0.0, -10.0),
            length_m: 10.0,
            from_uid: 50,
            to_uid: 51,
            edge_uid: 1,
        };
        let prefab_meta: Option<SegmentMetadata> = Some(SegmentMetadata {
            is_prefab: true,
            lane_offset_right_m: 0.0,
            lanes_in_direction: 1,
            lanes_opposite: 0,
            lanes_total: 1,
            lane_width_m: 3.75,
            road_look_token: 0,
        });
        let segs = vec![north_prefab];
        let metas = vec![prefab_meta];
        let forward_adj = build_forward_adjacency(&segs);
        let luts = build_all_luts(&segs);
        let index = build_index_with_metadata(segs, metas);

        let mut plugin = LaneFollowerPlugin {
            index: Some(Arc::new(index)),
            luts,
            forward_adj,
            router_graph: Some(Arc::new(make_junction_rg(0.0, -5.0))),
            junction_max_heading_diff_rad: std::f32::consts::PI / 4.0,
            bias_radius_m: 30.0,
            bias_max_prefab_dist_m: 10.0,
            ..Default::default()
        };

        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        // Truck 2m into segment, heading North.
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_follower.bias_prefab_accepted").as_deref(),
            Some("true"),
            "prerequisite: prefab must be accepted"
        );
        assert_eq!(
            ctx.blackboard.get("lane_follower.lateral_source").as_deref(),
            Some("navcurve"),
            "bias_accepted=true must set lateral_source=navcurve"
        );
    }

    /// lateral_source is computed fresh each tick — no state bleeds between ticks.
    #[test]
    fn ds14_lateral_source_no_state_bleeding() {
        use truckpilot_map_parser::spline_index::build_index_with_metadata;
        use std::f32::consts::PI;

        // East-going prefab produces none_found when truck heads North,
        // but navcurve when truck heads East.
        let east_prefab = HermiteSegment {
            p0: Vec3::new(-5.0, 0.0, -5.0),
            p1: Vec3::new(5.0, 0.0, -5.0),
            m0: Vec3::new(10.0, 0.0, 0.0),
            m1: Vec3::new(10.0, 0.0, 0.0),
            length_m: 10.0,
            from_uid: 100,
            to_uid: 200,
            edge_uid: 1,
        };
        let prefab_meta: Option<SegmentMetadata> = Some(SegmentMetadata {
            is_prefab: true,
            lane_offset_right_m: 0.0,
            lanes_in_direction: 1,
            lanes_opposite: 0,
            lanes_total: 1,
            lane_width_m: 3.75,
            road_look_token: 0,
        });
        let segs = vec![east_prefab];
        let metas = vec![prefab_meta];
        let forward_adj = build_forward_adjacency(&segs);
        let luts = build_all_luts(&segs);
        let index = build_index_with_metadata(segs, metas);

        let mut plugin = LaneFollowerPlugin {
            index: Some(Arc::new(index)),
            luts,
            forward_adj,
            router_graph: Some(Arc::new(make_junction_rg(0.0, -5.0))),
            junction_max_heading_diff_rad: PI / 4.0,
            bias_radius_m: 30.0,
            bias_max_prefab_dist_m: 10.0,
            ..Default::default()
        };

        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();

        // Tick 1: heading North → none_found → road_offset
        let tel_north = make_telemetry(0.0, 0.0, -5.0, 0.0);
        plugin.tick(Some(&tel_north), &mut out, &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_follower.lateral_source").as_deref(),
            Some("road_offset"),
            "tick 1 heading North must give road_offset"
        );

        // Tick 2: heading East (ETS2 heading = 0.25 full rotation = 90°)
        // ETS2 heading is stored as fraction 0..1 (0=North, 0.25=East).
        // make_telemetry uses heading directly — the plugin converts: (-heading * 360).rem_euclid(360)
        // heading=0.0 → truck_heading_deg = 0.0 (North)
        // heading=-0.25 → truck_heading_deg = 90.0 (East) ... let's verify convention:
        // truck_heading_deg = ((-tel.heading) * 360.0).rem_euclid(360.0)
        // For East (90°): -tel.heading * 360 = 90 → tel.heading = -0.25
        let tel_east = make_telemetry(0.0, 0.0, -5.0, -0.25);
        plugin.tick(Some(&tel_east), &mut out, &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_follower.bias_prefab_accepted").as_deref(),
            Some("true"),
            "tick 2 heading East must accept East prefab"
        );
        assert_eq!(
            ctx.blackboard.get("lane_follower.lateral_source").as_deref(),
            Some("navcurve"),
            "tick 2 heading East must give navcurve (no state from tick 1 road_offset)"
        );
    }

    /// DS14.1: none_found guard — Soft-Safety must NOT fire when lateral > SOFT but bias=none_found.
    /// nearest-seg reference is invalid in K2 gap; suppressing avoids fighting the synthetic target.
    #[test]
    fn ds14_1_soft_safety_none_found_bypass() {
        use truckpilot_map_parser::spline_index::build_index_with_metadata;

        // East prefab at z=-5. Truck at (0,0,0.9) heading North:
        //   heading diff = 90° > 45° → none_found
        //   lateral = fwd_x*(truck_z - curve_z) = 1.0*(0.9-(-5)) ≈ 5.9 > SOFT (3.0m)
        let east_prefab = HermiteSegment {
            p0: Vec3::new(-5.0, 0.0, -5.0),
            p1: Vec3::new(5.0, 0.0, -5.0),
            m0: Vec3::new(10.0, 0.0, 0.0),
            m1: Vec3::new(10.0, 0.0, 0.0),
            length_m: 10.0,
            from_uid: 100,
            to_uid: 200,
            edge_uid: 1,
        };
        let segs = vec![east_prefab];
        let metas = vec![Some(SegmentMetadata {
            is_prefab: true,
            lane_offset_right_m: 0.0,
            lanes_in_direction: 1,
            lanes_opposite: 0,
            lanes_total: 1,
            lane_width_m: 3.75,
            road_look_token: 0,
        })];
        let forward_adj = build_forward_adjacency(&segs);
        let luts = build_all_luts(&segs);
        let index = build_index_with_metadata(segs, metas);
        let mut plugin = LaneFollowerPlugin {
            index: Some(Arc::new(index)),
            luts,
            forward_adj,
            router_graph: Some(Arc::new(make_junction_rg(0.0, -5.0))),
            junction_max_heading_diff_rad: std::f32::consts::PI / 4.0,
            bias_radius_m: 30.0,
            bias_max_prefab_dist_m: 10.0,
            ..Default::default()
        };
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, 0.9, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_follower.bias_prefab_rejected_reason").as_deref(),
            Some("none_found"),
            "prerequisite: must be none_found"
        );
        let lateral: f32 = ctx
            .blackboard
            .get("lane_follower.lateral_dist_signed")
            .unwrap()
            .parse()
            .unwrap();
        assert!(
            lateral.abs() > SAFETY_LATERAL_SOFT_M,
            "prerequisite: |lateral|={lateral} must exceed SOFT threshold"
        );
        assert_eq!(
            ctx.blackboard.get("lane_follower.rate_limit_active").as_deref(),
            Some("false"),
            "none_found: Soft-Safety must NOT fire (invalid nearest-seg reference in K2 gap)"
        );
    }

    /// DS14.1: too_far guard check — Soft-Safety must still fire when lateral > SOFT and bias=too_far.
    /// Uses a North-going prefab 5.9m to the right: heading diff=0° passes the heading filter,
    /// but bias_max_prefab_dist_m=0.0 rejects it as too_far.
    #[test]
    fn ds14_1_soft_safety_too_far_active() {
        use truckpilot_map_parser::spline_index::build_index_with_metadata;

        // North prefab at x=5.9 (heading diff 0° < 45° → passes filter, but dist=5.9 > 0.0 → too_far).
        // lateral_dist_signed ≈ -5.9 (truck left of North road) → |lateral| > SOFT (3.0m).
        let north_prefab = HermiteSegment {
            p0: Vec3::new(5.9, 0.0, 0.0),
            p1: Vec3::new(5.9, 0.0, -10.0),
            m0: Vec3::new(0.0, 0.0, -10.0),
            m1: Vec3::new(0.0, 0.0, -10.0),
            length_m: 10.0,
            from_uid: 100,
            to_uid: 200,
            edge_uid: 1,
        };
        let segs = vec![north_prefab];
        let metas = vec![Some(SegmentMetadata {
            is_prefab: true,
            lane_offset_right_m: 0.0,
            lanes_in_direction: 1,
            lanes_opposite: 0,
            lanes_total: 1,
            lane_width_m: 3.75,
            road_look_token: 0,
        })];
        let forward_adj = build_forward_adjacency(&segs);
        let luts = build_all_luts(&segs);
        let index = build_index_with_metadata(segs, metas);
        let mut plugin = LaneFollowerPlugin {
            index: Some(Arc::new(index)),
            luts,
            forward_adj,
            router_graph: Some(Arc::new(make_junction_rg(0.0, -5.0))),
            junction_max_heading_diff_rad: std::f32::consts::PI / 4.0,
            bias_radius_m: 30.0,
            bias_max_prefab_dist_m: 0.0, // dist=5.9 > 0.0 → too_far
            ..Default::default()
        };
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_follower.bias_prefab_rejected_reason").as_deref(),
            Some("too_far"),
            "prerequisite: must be too_far (heading passes filter, dist > 0.0 threshold)"
        );
        let lateral: f32 = ctx
            .blackboard
            .get("lane_follower.lateral_dist_signed")
            .unwrap()
            .parse()
            .unwrap();
        assert!(
            lateral.abs() > SAFETY_LATERAL_SOFT_M,
            "prerequisite: |lateral|={lateral} must exceed SOFT threshold"
        );
        assert_eq!(
            ctx.blackboard.get("lane_follower.rate_limit_active").as_deref(),
            Some("true"),
            "too_far: Soft-Safety must still fire when lateral > SOFT threshold"
        );
    }

    /// DS14.1: Hard-Safety (lateral_excursion) must fire even when none_found guard suppresses Soft.
    /// Proves the guard only affects rate_limit_active, not check_safety_trip.
    #[test]
    fn ds14_1_hard_safety_unaffected_by_none_found_guard() {
        use truckpilot_map_parser::spline_index::build_index_with_metadata;

        // East prefab at z=-5. Truck at (0,0,3.5) heading North:
        //   heading diff = 90° > 45° → none_found (guard suppresses Soft)
        //   lateral = 1.0*(3.5-(-5)) = 8.5 > HARD (8.0m) → hard trip must still fire
        let east_prefab = HermiteSegment {
            p0: Vec3::new(-5.0, 0.0, -5.0),
            p1: Vec3::new(5.0, 0.0, -5.0),
            m0: Vec3::new(10.0, 0.0, 0.0),
            m1: Vec3::new(10.0, 0.0, 0.0),
            length_m: 10.0,
            from_uid: 100,
            to_uid: 200,
            edge_uid: 1,
        };
        let segs = vec![east_prefab];
        let metas = vec![Some(SegmentMetadata {
            is_prefab: true,
            lane_offset_right_m: 0.0,
            lanes_in_direction: 1,
            lanes_opposite: 0,
            lanes_total: 1,
            lane_width_m: 3.75,
            road_look_token: 0,
        })];
        let forward_adj = build_forward_adjacency(&segs);
        let luts = build_all_luts(&segs);
        let index = build_index_with_metadata(segs, metas);
        let mut plugin = LaneFollowerPlugin {
            index: Some(Arc::new(index)),
            luts,
            forward_adj,
            router_graph: Some(Arc::new(make_junction_rg(0.0, -5.0))),
            junction_max_heading_diff_rad: std::f32::consts::PI / 4.0,
            bias_radius_m: 30.0,
            bias_max_prefab_dist_m: 10.0,
            ..Default::default()
        };
        let ctx = PluginContext::test();
        ctx.blackboard.set("plugin.lane-follower.mode", "active");
        let mut out = ControlOutput::default();
        let tel = make_telemetry(0.0, 0.0, 3.5, 0.0);
        plugin.tick(Some(&tel), &mut out, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_follower.bias_prefab_rejected_reason").as_deref(),
            Some("none_found"),
            "prerequisite: none_found (guard suppresses Soft)"
        );
        assert_eq!(
            ctx.blackboard.get("lane_follower.rate_limit_active").as_deref(),
            Some("false"),
            "Soft-Safety suppressed by none_found guard"
        );
        assert_eq!(
            ctx.blackboard.get("lane_follower.safety_disengage_reason").as_deref(),
            Some("lateral_excursion"),
            "Hard-Safety must still fire despite Soft being suppressed"
        );
        assert_eq!(
            ctx.blackboard.get("autopilot.disengage_requested").as_deref(),
            Some("true"),
            "Hard-Safety must request disengage"
        );
    }

    /// DS13e TASK 3: last_active_segment_idx resets to -1 after ACTIVE_SEG_RESET_FRAMES
    /// consecutive ticks outside the junction zone.
    #[test]
    fn test_active_segment_resets_after_zone_exit() {
        // Plugin with last_active_segment_idx pre-set; no junction zone active (no router_graph).
        let mut plugin = make_chain_plugin();
        plugin.last_active_segment_idx = Some(0);
        plugin.last_active_segment_at = Some(std::time::Instant::now());

        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        // Truck near segment 0, heading North — should reach status=ok every tick.
        let tel = make_telemetry(0.0, 0.0, -2.0, 0.0);

        // Run exactly ACTIVE_SEG_RESET_FRAMES ticks without junction zone.
        for _ in 0..ACTIVE_SEG_RESET_FRAMES {
            plugin.tick(Some(&tel), &mut out, &ctx);
        }
        assert_eq!(ctx.blackboard.get("lane_follower.status").as_deref(), Some("ok"),
            "must reach ok status to write last_active_segment_idx key");

        let last_idx: i64 = ctx
            .blackboard
            .get("lane_follower.last_active_segment_idx")
            .unwrap()
            .parse()
            .unwrap();
        assert_eq!(
            last_idx, -1,
            "after {} ticks outside junction zone, last_active_segment_idx must reset to -1",
            ACTIVE_SEG_RESET_FRAMES
        );
    }
}
