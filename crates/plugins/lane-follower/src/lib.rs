//! Lane-Follower plugin — Phase 1a skeleton (Observer mode).
//!
//! Loads a SplineIndex from `graph.json` at startup and queries it every
//! 50 Hz tick to publish diagnostic Blackboard keys.  No steering output
//! is produced in this phase (`tick_request` always returns `None`).
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
//! | `lane_follower.truck_z` | f64 metres | truck ETS2 Z |
//! | `lane_follower.truck_heading_deg` | f32 degrees | truck heading (0=North, CW) |
//! | `lane_follower.nearest_seg_idx` | usize | global segment index |
//! | `lane_follower.nearest_seg_dist_m` | f32 metres | Euclidean distance to road |
//! | `lane_follower.nearest_seg_t` | f32 [0,1] | Hermite parameter |
//! | `lane_follower.heading_deg` | f32 degrees | road heading at nearest point |
//! | `lane_follower.heading_diff_deg` | f32 degrees | |truck − road heading| in [0,180] |
//! | `lane_follower.status` | string | `ok` / `dist_warn` / `heading_warn` / … |

use truckpilot_map_parser::{
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
    mode: LaneFollowerMode,
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
    }

    fn tick(&mut self, telemetry: Option<&Telemetry>, _output: &mut ControlOutput, ctx: &PluginContext) {
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
        let truck_z = tel.position[2];
        ctx.blackboard.set("lane_follower.truck_x", format!("{truck_x:.3}"));
        ctx.blackboard.set("lane_follower.truck_z", format!("{truck_z:.3}"));

        let Some(index) = &self.index else {
            ctx.blackboard.set("lane_follower.status", "no_index");
            return;
        };

        let query = Vec3::new(truck_x as f32, tel.position[1] as f32, truck_z as f32);
        let Some(hit) = index.nearest_with_projection(query, CANDIDATES) else {
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
            .set("lane_follower.heading_deg", format!("{:.2}", hit.heading_deg));

        let truck_heading_deg = tel.heading.to_degrees().rem_euclid(360.0) as f32;
        ctx.blackboard
            .set("lane_follower.truck_heading_deg", format!("{truck_heading_deg:.2}"));
        let heading_diff = angular_diff_deg(truck_heading_deg, hit.heading_deg);
        ctx.blackboard
            .set("lane_follower.heading_diff_deg", format!("{heading_diff:.2}"));

        if hit.dist_m > DIST_WARN_M {
            ctx_warn!(
                ctx,
                "lane-follower: dist_warn {:.1}m > {}m threshold",
                hit.dist_m,
                DIST_WARN_M
            );
            ctx.blackboard.set("lane_follower.status", "dist_warn");
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
            return;
        }

        ctx.blackboard.set("lane_follower.status", "ok");
    }

    fn tick_request(
        &mut self,
        _telemetry: Option<&Telemetry>,
        _ctx: &PluginContext,
    ) -> Option<ControlRequest> {
        None
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
        spline::HermiteSegment,
        spline_index::build_index,
    };
    use truckpilot_plugin_api::{PluginContext, Telemetry};

    // ── Helpers ──────────────────────────────────────────────────────────────

    fn make_telemetry(x: f64, y: f64, z: f64, heading_rad: f64) -> Telemetry {
        Telemetry {
            position: [x, y, z],
            heading: heading_rad,
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
        let mut plugin = LaneFollowerPlugin { index: Some(index), mode: LaneFollowerMode::Observer };
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
        let mut plugin = LaneFollowerPlugin { index: Some(index), mode: LaneFollowerMode::Observer };
        let ctx = PluginContext::test();
        let mut out = ControlOutput::default();
        // Truck at (-45, 0, -50) near Seg 1 (heading 0°=North)
        // Truck heading = π rad = 180° (South) → diff = 180° > 90°
        let tel = make_telemetry(-45.0, 0.0, -50.0, std::f64::consts::PI);
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
        let mut plugin = LaneFollowerPlugin { index: Some(index), mode: LaneFollowerMode::Observer };
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

    // ── tick_request always returns None ─────────────────────────────────────

    #[test]
    fn tick_request_returns_none() {
        let mut plugin = LaneFollowerPlugin::default();
        let ctx = PluginContext::test();
        let tel = make_telemetry(0.0, 0.0, 0.0, 0.0);
        assert!(plugin.tick_request(Some(&tel), &ctx).is_none());
        assert!(plugin.tick_request(None, &ctx).is_none());
    }
}
