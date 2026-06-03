//! Lane-Keeper plugin — dual-mode: route-following (Catmull-Rom) or vision-based.
//!
//! ## Mode selection
//! Set `plugin.lane_keeper.mode` on the Blackboard to `"vision"` or `"route_following"`.
//! Default (if key absent): `RouteFollowing` — preserves all existing behaviour.
//!
//! ## Vision mode — 5-Level Fallback Cascade (DS1 spec)
//! Level 0: Normal vision PID.  Level 1: Single-lane extrapolation.
//! Level 2: Confidence-drop (EMA).  Level 3: Heading-hold.  Level 4: Disengage.
//!
//! ## Route-following mode
//! Reads waypoints from `router.waypoints`.  Speed-adaptive look-ahead.
//! `look_ahead_m = BASE_LOOK_AHEAD + speed_kmh * SPEED_FACTOR`

mod extrapolation;
mod fallback;
mod heading_hold;

use extrapolation::{extrapolate_center, LaneWidthState};
use fallback::FallbackState;
use heading_hold::{wrap_angle, HeadingHoldState};
use std::collections::HashMap;
use std::sync::Arc;
use truckpilot_map_parser::{
    arc_length::{arc_length, build_lut, t_at_arc_length},
    spline::{evaluate, evaluate_tangent, Vec3},
    SplineIndex,
};
use truckpilot_plugin_api::graph::RouterGraph;
use truckpilot_plugin_api::{
    pid::Pid, ControlOutput, ControlRequest, Plugin, PluginContext, Telemetry,
};

// ── Priorities ────────────────────────────────────────────────────────────────
const PRIORITY_NORMAL: i32 = 50;
const PRIORITY_LEVEL4: i32 = 200;

// ── Route-following constants ─────────────────────────────────────────────────
const BASE_LOOK_AHEAD: f64 = 5.0;
const SPEED_FACTOR: f64 = 0.5;
const WAYPOINT_REACH_M: f64 = 5.0;
/// Right-lane offset (Rechtsfahrgebot). Mirrors lane-follower LANE_OFFSET_RIGHT_M.
const LANE_OFFSET_RIGHT_M: f64 = 1.875;

/// Phase 2d: Max Hops die der Spline-Lookahead entlang der Route walkt, bevor er
/// am Segmentende clampt (Zyklus-/Nicht-Vorrück-Sicherung, analog Lane-Follower).
const SPLINE_LOOKAHEAD_MAX_HOPS: usize = 64;
/// Phase 2d: Max laterale Distanz Truck↔Hop-Segment (Centerline), bevor der
/// Spline-Pfad auf Catmull zurückfällt (Schutz gegen verirrte Projektionen,
/// härtet Finding A zusätzlich ab). Großzügig: korrekt in der rechten Spur
/// liegt der Truck bis ~17m (5-spurig) von der Centerline; >40m = nicht auf
/// diesem Hop.
const MAX_HOP_PROJECTION_DIST_M: f32 = 40.0;

// ── PID defaults ──────────────────────────────────────────────────────────────
const DEFAULT_KP: f64 = 0.8;
const DEFAULT_KI: f64 = 0.1;
const DEFAULT_KD: f64 = 0.3;

/// Block-2: max heading error (radians) before lane-keeper suspends steering.
/// ~80°: covers normal curves/lane-changes (≤45°) but blocks clear mismatch cases.
pub(crate) const HEADING_MISMATCH_THRESHOLD_RAD: f64 = 1.4;

/// Block-2: max steering change per tick.
const STEERING_MAX_DELTA_PER_TICK: f64 = 0.1;

/// Ticks before Level-4 brake is lifted.
const L4_BRAKE_TICKS: u64 = 50;

// ── Mode enum ─────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum LaneKeeperMode {
    #[default]
    RouteFollowing,
    Vision,
    Off,
}

// ── Plugin struct ─────────────────────────────────────────────────────────────

pub struct LaneKeeperPlugin {
    pid: Pid,

    // ── Route-following fields ──────────────────────────────────────────────
    waypoints: Vec<[f64; 2]>,
    progress_idx: usize,
    subdivisions: usize,
    last_gains: (f64, f64, f64),
    last_waypoints_hash: u64,
    previous_steering_out: f64,
    heading_stage: Option<String>,
    previous_heading_stage: Option<String>,

    // ── Vision-mode fields ─────────────────────────────────────────────────
    mode: LaneKeeperMode,
    fallback: FallbackState,
    extrapolator: LaneWidthState,
    heading_hold: HeadingHoldState,
    /// Heading captured at Active-session start (for Block-2 guard in vision mode).
    engagement_heading: Option<f64>,
    /// Tick at which Level-4 was entered (None = not in L4).
    level_4_entered_at_tick: Option<u64>,
    /// Monotonic counter across all tick_request calls.
    tick_count: u64,
    /// Whether the last vision tick was in the Active state (for transition detection).
    was_active: bool,

    // ── Phase 2c/2d: SplineIndex Route-Geometrie ────────────────────────────
    index: Option<Arc<SplineIndex>>,
    router_graph: Option<Arc<RouterGraph>>,
    seg_by_from_to: HashMap<(u64, u64), usize>,
    cached_route_node_ids: Vec<u64>,
    cached_route_hash: u64,
    node_progress_idx: usize,
    /// true solange der Spline-Pfad im letzten Tick aktiv war (für sauberen
    /// Re-Sync des Catmull-progress_idx beim Übergang Spline→Catmull).
    was_spline_active: bool,
    /// Phase 2c/2d-Diagnose (read-only): einmaliges Flag, damit der
    /// route_miss-Sample-Log (erste 3 Hops + in_map) nur EINMAL feuert.
    route_miss_sample_logged: bool,
    /// Phase 2f-B-Diagnose (read-only): zählt JEDEN Eintritt in
    /// `try_spline_heading_error` (vor jeder Bedingung). Wächst er NICHT mit den
    /// Ticks → Funktion wird gar nicht betreten (H1).
    reanchor_called_count: u64,
    /// Phase 2f-B-Diagnose (read-only): zählt, wie oft der Code bis zum
    /// tatsächlichen `node_progress_idx`-Schreiben (re-anchor-Scan-Write) kommt.
    /// called wächst aber reached NICHT → eine Bedingung VOR dem Scan bricht ab.
    reanchor_reached_write: u64,
}

impl Default for LaneKeeperPlugin {
    fn default() -> Self {
        Self {
            pid: Pid::new(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, 2.0, 1.0),
            waypoints: Vec::new(),
            progress_idx: 0,
            subdivisions: 4,
            last_gains: (DEFAULT_KP, DEFAULT_KI, DEFAULT_KD),
            last_waypoints_hash: 0,
            previous_steering_out: 0.0,
            heading_stage: None,
            previous_heading_stage: None,
            mode: LaneKeeperMode::default(),
            fallback: FallbackState::new(),
            extrapolator: LaneWidthState::new(),
            heading_hold: HeadingHoldState::new(),
            engagement_heading: None,
            level_4_entered_at_tick: None,
            tick_count: 0,
            was_active: false,
            index: None,
            router_graph: None,
            seg_by_from_to: HashMap::new(),
            cached_route_node_ids: Vec::new(),
            cached_route_hash: 0,
            node_progress_idx: 0,
            was_spline_active: false,
            route_miss_sample_logged: false,
            reanchor_called_count: 0,
            reanchor_reached_write: 0,
        }
    }
}

// ── Shared helpers ────────────────────────────────────────────────────────────

impl LaneKeeperPlugin {
    fn apply_gain_overrides(&mut self, ctx: &PluginContext) {
        let kp = ctx
            .blackboard
            .get_f64("plugin.lane_keeper.kp")
            .unwrap_or(DEFAULT_KP);
        let ki = ctx
            .blackboard
            .get_f64("plugin.lane_keeper.ki")
            .unwrap_or(DEFAULT_KI);
        let kd = ctx
            .blackboard
            .get_f64("plugin.lane_keeper.kd")
            .unwrap_or(DEFAULT_KD);
        let next = (kp, ki, kd);
        if next != self.last_gains {
            self.pid.set_kp(kp);
            self.pid.set_ki(ki);
            self.pid.set_kd(kd);
            self.last_gains = next;
            tracing::info!("[lane-keeper] gains updated kp={kp} ki={ki} kd={kd}");
            ctx.blackboard
                .set("pid_tuning.lane_keeper.kp", kp.to_string());
            ctx.blackboard
                .set("pid_tuning.lane_keeper.ki", ki.to_string());
            ctx.blackboard
                .set("pid_tuning.lane_keeper.kd", kd.to_string());
        }
    }

    fn update_mode_from_blackboard(&mut self, ctx: &PluginContext) {
        let raw = ctx.blackboard.get("plugin.lane_keeper.mode");
        let next = match raw.as_deref() {
            Some("vision") => LaneKeeperMode::Vision,
            Some("off") => LaneKeeperMode::Off,
            _ => LaneKeeperMode::RouteFollowing,
        };
        if next != self.mode {
            tracing::info!("[lane-keeper] mode switch {:?} → {:?}", self.mode, next);
            self.mode = next;
            self.pid.reset();
            self.fallback.reset();
            self.heading_hold.exit();
            self.engagement_heading = None;
            self.level_4_entered_at_tick = None;
            self.previous_steering_out = 0.0;
        }
    }

    /// Apply the rate-limiter and return the clamped steering value.
    fn rate_limit(&mut self, target: f64, ctx: &PluginContext) -> f64 {
        let delta = target - self.previous_steering_out;
        let clamped = delta.clamp(-STEERING_MAX_DELTA_PER_TICK, STEERING_MAX_DELTA_PER_TICK);
        let output = self.previous_steering_out + clamped;
        let was_limited = (clamped - delta).abs() > 1e-9;
        ctx.blackboard
            .set("lane_keeper.steering_rate_limited", was_limited.to_string());
        ctx.blackboard.set(
            "lane_keeper.steering_delta_clamped",
            format!("{:.4}", delta - clamped),
        );
        self.previous_steering_out = output;
        output
    }
}

// ── Route-following implementation ────────────────────────────────────────────

impl LaneKeeperPlugin {
    fn load_waypoints_from_blackboard(&mut self, ctx: &PluginContext) {
        if let Some(json) = ctx.blackboard.get("router.waypoints") {
            if let Ok(pts) = serde_json::from_str::<Vec<[f64; 2]>>(&json) {
                let new_hash = hash_str(&json);
                self.waypoints = smooth_catmull_rom(&pts, self.subdivisions);
                self.progress_idx = 0;
                self.last_waypoints_hash = new_hash;
                if !self.waypoints.is_empty() {
                    tracing::info!(
                        "[lane-keeper] first 3 spline pts: [{:.1},{:.1}] [{:.1},{:.1}] [{:.1},{:.1}]",
                        self.waypoints[0][0], self.waypoints[0][1],
                        self.waypoints.get(1).map(|p| p[0]).unwrap_or(0.0),
                        self.waypoints.get(1).map(|p| p[1]).unwrap_or(0.0),
                        self.waypoints.get(2).map(|p| p[0]).unwrap_or(0.0),
                        self.waypoints.get(2).map(|p| p[1]).unwrap_or(0.0),
                    );
                }
                tracing::info!(
                    "[lane-keeper] loaded {} waypoints (smoothed, hash={:x})",
                    self.waypoints.len(),
                    new_hash
                );
                self.previous_steering_out = 0.0;
                self.pid.reset();
            }
        }
    }

    fn try_spline_heading_error(
        &mut self,
        tx: f64,
        tz: f64,
        heading: f64,
        speed_ms: f64,
        ctx: &PluginContext,
    ) -> Option<f64> {
        // Phase 2f-B-Diagnose (read-only): Funktions-Eintritt zählen, GANZ AM ANFANG,
        // vor JEDER Bedingung/early-return. Wächst dieser Zähler nicht mit den Ticks,
        // wird try_spline_heading_error (und damit der re-anchor-Scan) gar nicht
        // betreten (H1: hinter dem dist_gate / im Spline-Zweig übersprungen).
        self.reanchor_called_count += 1;
        ctx.blackboard.set(
            "lane_keeper.reanchor_called_count",
            self.reanchor_called_count.to_string(),
        );

        // Phase 2c/2d-Diagnose (read-only): jeder Dispatch-Pfad schreibt GENAU EINEN
        // `lane_keeper.fallback_reason` (6-Wert-Vertrag) plus eine feinere
        // `lane_keeper.fallback_detail`. Reihenfolge = Dispatch-Flow → die ERSTE
        // greifende Bedingung gewinnt. KEINE Verhaltensänderung: nur Keys + Logs.
        let Some(index) = self.index.as_ref() else {
            ctx.blackboard
                .set("lane_keeper.fallback_reason", "index_none");
            ctx.blackboard
                .set("lane_keeper.fallback_detail", "index_none");
            return None;
        };
        let Some(rg) = self.router_graph.as_ref() else {
            ctx.blackboard
                .set("lane_keeper.fallback_reason", "index_none");
            ctx.blackboard
                .set("lane_keeper.fallback_detail", "graph_none");
            return None;
        };

        // Route cachen (hash-reload); progress reset bei Routenwechsel.
        let Some(route_json) = ctx.blackboard.get("router.route_node_ids") else {
            ctx.blackboard
                .set("lane_keeper.fallback_reason", "route_miss");
            ctx.blackboard
                .set("lane_keeper.fallback_detail", "no_route_node_ids");
            return None;
        };
        let route_hash = hash_str(&route_json);
        let route_changed = route_hash != self.cached_route_hash;
        if route_changed {
            let Ok(nodes) = serde_json::from_str::<Vec<u64>>(&route_json) else {
                ctx.blackboard
                    .set("lane_keeper.fallback_reason", "route_miss");
                ctx.blackboard
                    .set("lane_keeper.fallback_detail", "route_parse_err");
                return None;
            };
            self.cached_route_node_ids = nodes;
            self.cached_route_hash = route_hash;
            self.node_progress_idx = 0;
            self.route_miss_sample_logged = false; // neue Route → Sample erneut erlauben
        }
        let route = &self.cached_route_node_ids;
        if route.len() < 2 {
            ctx.blackboard
                .set("lane_keeper.fallback_reason", "route_miss");
            ctx.blackboard
                .set("lane_keeper.fallback_detail", "route_too_short");
            return None;
        }

        // Phase 2g (Variante B): Globale R-tree-nearest-Query (wie Lane-Follower) statt
        // route-only Vorwärts-Fenster-Scan. Der route-only-Scan misst die Distanz zur
        // Vorgänger-/Snap-Kante falsch (Truck sitzt auf to_uid == route[0], KEIN
        // Vorwärts-Hop → 69m statt 3.4m → dist_gate feuert permanent). Die globale
        // nearest-Query findet die geometrisch nächste, heading-kompatible Kante (~3.4m,
        // inkl. Gegenfahrbahn-Schutz). Anschließend Route-Relevanz-Prüfung gegen die
        // Route (kein Abspringen auf Parallelstraßen).
        let truck_heading_deg = ((-heading) * 360.0).rem_euclid(360.0) as f32; // heading = t.heading [0..1]
        let query = Vec3::new(tx as f32, 0.0, tz as f32); // Y=0 ok: R-tree ist XZ-only
        let Some(hit) = index.nearest_with_heading_filter(query, truck_heading_deg, 8) else {
            ctx.blackboard
                .set("lane_keeper.fallback_reason", "route_miss");
            ctx.blackboard
                .set("lane_keeper.fallback_detail", "no_nearest");
            return None;
        };
        let cur_seg = hit.segment_idx;
        let t_cur = hit.t;
        let seg_f = index.segments[cur_seg].from_uid;
        let seg_t = index.segments[cur_seg].to_uid;

        // Route-Relevanz-Prüfung (Route-Constraint). Linearer Scan über die Route (~8 Knoten):
        //   on-route Hop:        ∃ j: route[j]==F && route[j+1]==T → progress=j,  end=j+1
        //   Vorgänger-Kante:     sonst ∃ k: route[k]==T            → progress=k,  end=k
        //   off-route:           sonst → Catmull-Fallback (Schutz gegen Parallelstraße)
        // on-route wird gegenüber feeds-into bevorzugt; jeweils kleinster passender Index.
        let mut on_route_idx: Option<usize> = None;
        let mut feeds_into_idx: Option<usize> = None;
        for k in 0..route.len() {
            if on_route_idx.is_none()
                && k + 1 < route.len()
                && route[k] == seg_f
                && route[k + 1] == seg_t
            {
                on_route_idx = Some(k);
            }
            if feeds_into_idx.is_none() && route[k] == seg_t {
                feeds_into_idx = Some(k);
            }
        }
        let (node_progress_idx, end_route_idx) = if let Some(j) = on_route_idx {
            // on-route: exakter direktionaler (from,to)-Match → immer sicher,
            // unabhängig vom Heading-Flag (W1) und ohne Forward-Hop-Prüfung (W2).
            (j, j + 1)
        } else if let Some(k) = feeds_into_idx {
            // feeds-into (Vorgänger-Kante): nur akzeptieren wenn
            //   W1: der nearest-Treffer den Heading-Filter bestanden hat
            //       (sonst evtl. flach einmündende Querstraße), UND
            //   W2: der erste Vorwärts-Walk-Hop route[k]→route[k+1] existiert
            //       (sonst Rückwärts-/U-turn-Route ohne Forward-Segment).
            let accept_feeds_into = hit.heading_filter_applied
                && k + 1 < route.len()
                && self.seg_by_from_to.contains_key(&(route[k], route[k + 1]));
            if !accept_feeds_into {
                let detail = if !hit.heading_filter_applied {
                    "feeds_into_no_heading"
                } else {
                    "feeds_into_no_forward_hop"
                };
                ctx.blackboard.set("lane_keeper.fallback_reason", "off_route");
                ctx.blackboard.set("lane_keeper.fallback_detail", detail);
                return None;
            }
            (k, k)
        } else {
            ctx.blackboard.set("lane_keeper.fallback_reason", "off_route");
            ctx.blackboard
                .set("lane_keeper.fallback_detail", "nearest_not_on_route");
            return None;
        };
        self.node_progress_idx = node_progress_idx;

        // Phase 2f-B-Diagnose (read-only): Re-anchor-Instrumentierung weiter befüllen.
        // Gate-Distanz ist 2D (XZ) zum projizierten Punkt auf der nearest-Kante.
        let pc = evaluate(&index.segments[cur_seg], t_cur);
        let dist = (((tx - pc.x as f64).powi(2) + (tz - pc.z as f64).powi(2)).sqrt()) as f32;
        // Phase 2g-Diag2 (read-only): IST-Lateralversatz des Trucks gegen die Spur-MITTE,
        // gemessen am Truck-Projektionspunkt `pc` auf cur_seg. +rechts / -links (Right-Normal
        // n=(-tan.z,tan.x)). Entscheidende Metrik: fährt der Truck mittig (truck_lat≈0) trotz
        // gelogtem Offset 5.625 → die Kette Offset→Position greift nicht.
        let tan_cur = evaluate_tangent(&index.segments[cur_seg], t_cur);
        let tcl = (tan_cur.x * tan_cur.x + tan_cur.z * tan_cur.z).sqrt();
        let truck_lat_vs_centerline = if tcl > 1e-6 {
            let rn_cx = (-tan_cur.z / tcl) as f64;
            let rn_cz = (tan_cur.x / tcl) as f64;
            (tx - pc.x as f64) * rn_cx + (tz - pc.z as f64) * rn_cz
        } else {
            0.0
        };
        ctx.blackboard
            .set("lane_keeper.reanchor_scan_window", "global");
        ctx.blackboard
            .set("lane_keeper.reanchor_best_idx", node_progress_idx.to_string());
        ctx.blackboard
            .set("lane_keeper.reanchor_best_dist_m", format!("{dist:.2}"));
        self.reanchor_reached_write += 1;
        ctx.blackboard.set(
            "lane_keeper.reanchor_reached_write",
            self.reanchor_reached_write.to_string(),
        );
        ctx.blackboard
            .set("lane_keeper.reanchor_idx_written", node_progress_idx.to_string());

        // seg_idx0 = cur_seg; i0/a0/b0/t_truck auf die nearest-Kante umgestellt.
        let i0 = node_progress_idx;
        ctx.blackboard
            .set("lane_keeper.seg_idx0_source_value", i0.to_string());
        let seg_idx0 = cur_seg;
        let (a0, b0) = (seg_f, seg_t);
        let t_truck = t_cur;
        ctx.blackboard
            .set("lane_keeper.hop_projection_dist_m", format!("{dist:.2}"));

        // ── Phase 2f-Diagnose (read-only): warum sitzt dist strukturell >40m? ──
        // WICHTIG: `dist` (= hop_projection_dist_m) ist die Distanz Truck→NÄCHSTER
        // PUNKT auf seg_idx0 (project_on_segment), NICHT ein Vorausschau-Abstand.
        // `look_ahead` (Soll-Voraus) wird erst NACH dem Gate berechnet und geht NICHT
        // ins Gate ein. truck_to_segment_dist_m == projected_point_dist_m == dist.
        //   → H1 (Gate zu eng auf legitimem Lookahead) ist damit strukturell NICHT
        //     der Mechanismus; das Gate misst Truck↔Segment.
        // Diskriminator H2-Varianten:
        //   projection_t ≈ 1.0 (oder 0.0) + dist groß  → Truck am Segment-ENDE
        //     vorbei (longitudinaler Overshoot): node_progress hängt → seg_idx0 ist
        //     ein bereits passierter Hop. Fix = node-advance, nicht Gate-Anheben.
        //   projection_t mittig (0.3..0.7) + dist groß  → echter LATERALER Miss
        //     (falsches/zu weit entferntes Segment). Fix = Segment-Auswahl/Projektion.
        //   dist_to_next_node_m strukturell > WAYPOINT_REACH_M (5m)  → der
        //     node-advance feuert nie (Truck fährt rechte Spur ~5.6m neben den
        //     Median-Nodes) → progress hängt. Das ist die wahrscheinlichste Wurzel.
        let look_ahead_target = (BASE_LOOK_AHEAD + speed_ms * 3.6 * SPEED_FACTOR) as f32;
        let seg_len = index.segments[seg_idx0].length_m;
        let seg_is_prefab = index
            .metadata
            .get(seg_idx0)
            .and_then(|m| m.as_ref())
            .map(|m| m.is_prefab)
            .unwrap_or(false);
        let dist_to_next_node = match rg.positions.get(&b0) {
            Some(&(nx, nz)) => ((tx - nx).powi(2) + (tz - nz).powi(2)).sqrt(),
            None => -1.0,
        };
        ctx.blackboard
            .set("lane_keeper.lookahead_target_dist_m", format!("{look_ahead_target:.2}"));
        ctx.blackboard
            .set("lane_keeper.truck_to_segment_dist_m", format!("{dist:.2}"));
        ctx.blackboard
            .set("lane_keeper.projected_point_dist_m", format!("{dist:.2}"));
        ctx.blackboard
            .set("lane_keeper.projection_t", format!("{t_truck:.3}"));
        ctx.blackboard
            .set("lane_keeper.current_seg_length_m", format!("{seg_len:.2}"));
        ctx.blackboard
            .set("lane_keeper.current_seg_is_prefab", seg_is_prefab.to_string());
        ctx.blackboard
            .set("lane_keeper.dist_to_next_node_m", format!("{dist_to_next_node:.2}"));
        ctx.blackboard
            .set("lane_keeper.current_hop", format!("{a0}->{b0}"));
        ctx.blackboard
            .set("lane_keeper.node_progress_idx", self.node_progress_idx.to_string());
        ctx.blackboard.set(
            "lane_keeper.dist_gate_threshold_m",
            format!("{MAX_HOP_PROJECTION_DIST_M:.2}"),
        );

        // ── Phase 2g-Diagnose (read-only): WO kommen die ~58m her? ──
        // Vergleicht für den AKTUELLEN Hop (a0->b0) drei Koordinaten-Quellen am selben Tick:
        //   1. Truck-Weltposition (tx,tz)
        //   2. Router-Graph-Node-Positionen von a0 (= route[i0]) und b0
        //   3. SplineIndex-Segment-Endpunkte p0/p1 des Segments seg_by_from_to[(a0,b0)]
        // Beweisrichtung:
        //   node(a0) ≈ seg.p0 UND node(b0) ≈ seg.p1  → identische Geometrie → NICHT H-A/H-B
        //     (beide stammen aus map_graph.nodes[uid]; build_router_graph & build_splines_ex teilen die Quelle).
        //   dist(truck, node(a0)) ≈ truck_to_segment_dist_m (~60m) bei projection_t≈0
        //     → route[0] (= direction-aligned Snap-Endpunkt) ist selbst weit weg, nicht die Geometrie → H-C.
        let seg0 = &index.segments[seg_idx0];
        let (sp0x, sp0z) = (seg0.p0.x as f64, seg0.p0.z as f64);
        let (sp1x, sp1z) = (seg0.p1.x as f64, seg0.p1.z as f64);
        let (na0x, na0z, na0_present) = match rg.positions.get(&a0) {
            Some(&(x, z)) => (x, z, true),
            None => (0.0, 0.0, false),
        };
        let (nb0x, nb0z, nb0_present) = match rg.positions.get(&b0) {
            Some(&(x, z)) => (x, z, true),
            None => (0.0, 0.0, false),
        };
        let dxz = |ax: f64, az: f64, bx: f64, bz: f64| ((ax - bx).powi(2) + (az - bz).powi(2)).sqrt();
        let node_a0_vs_segp0 = if na0_present { dxz(na0x, na0z, sp0x, sp0z) } else { -1.0 };
        let node_b0_vs_segp1 = if nb0_present { dxz(nb0x, nb0z, sp1x, sp1z) } else { -1.0 };
        let truck_to_node_a0 = if na0_present { dxz(tx, tz, na0x, na0z) } else { -1.0 };
        let truck_to_segp0 = dxz(tx, tz, sp0x, sp0z);
        ctx.blackboard
            .set("lane_keeper.diag_truck_xz", format!("{tx:.2},{tz:.2}"));
        ctx.blackboard
            .set("lane_keeper.diag_node_a0_xz", format!("{na0x:.2},{na0z:.2}"));
        ctx.blackboard
            .set("lane_keeper.diag_node_b0_xz", format!("{nb0x:.2},{nb0z:.2}"));
        ctx.blackboard
            .set("lane_keeper.diag_seg_p0_xz", format!("{sp0x:.2},{sp0z:.2}"));
        ctx.blackboard
            .set("lane_keeper.diag_seg_p1_xz", format!("{sp1x:.2},{sp1z:.2}"));
        ctx.blackboard
            .set("lane_keeper.diag_node_a0_vs_segp0_m", format!("{node_a0_vs_segp0:.2}"));
        ctx.blackboard
            .set("lane_keeper.diag_node_b0_vs_segp1_m", format!("{node_b0_vs_segp1:.2}"));
        ctx.blackboard
            .set("lane_keeper.diag_truck_to_node_a0_m", format!("{truck_to_node_a0:.2}"));
        ctx.blackboard
            .set("lane_keeper.diag_truck_to_segp0_m", format!("{truck_to_segp0:.2}"));

        // Task 2: Offset über mehrere Hops — globaler Frame-Versatz (H-B) oder ein einzelnes
        // falsches Mapping (H-A)? Pro Hop: dist(node(route[j]), seg.p0). Konstant ~58m → H-B;
        // nur dieser Hop → H-A; alle ~0 → Geometrie stimmt überall → NICHT H-A/H-B.
        let mut max_diff = 0.0f64;
        let mut sum_diff = 0.0f64;
        let mut n_diff = 0usize;
        let mut per_hop = String::new();
        let hop_end = route.len().saturating_sub(1).min(10);
        for j in 0..hop_end {
            if let Some(&seg) = self.seg_by_from_to.get(&(route[j], route[j + 1])) {
                if let (Some(&(nx, nz)), Some(s)) =
                    (rg.positions.get(&route[j]), index.segments.get(seg))
                {
                    let d = dxz(nx, nz, s.p0.x as f64, s.p0.z as f64);
                    max_diff = max_diff.max(d);
                    sum_diff += d;
                    n_diff += 1;
                    if per_hop.len() < 200 {
                        per_hop.push_str(&format!("h{j}={d:.1} "));
                    }
                }
            }
        }
        let mean_diff = if n_diff > 0 { sum_diff / n_diff as f64 } else { -1.0 };
        ctx.blackboard
            .set("lane_keeper.diag_node_vs_seg_maxdiff_m", format!("{max_diff:.2}"));
        ctx.blackboard
            .set("lane_keeper.diag_node_vs_seg_meandiff_m", format!("{mean_diff:.2}"));
        ctx.blackboard
            .set("lane_keeper.diag_node_vs_seg_per_hop", per_hop.trim().to_string());

        if dist > MAX_HOP_PROJECTION_DIST_M {
            ctx.blackboard
                .set("lane_keeper.fallback_reason", "dist_gate");
            ctx.blackboard.set("lane_keeper.fallback_detail", "dist_gate");
            return None; // Truck nicht wirklich auf diesem Hop → Catmull-Fallback
        }

        let look_ahead = (BASE_LOOK_AHEAD + speed_ms * 3.6 * SPEED_FACTOR) as f32;

        // Multi-Hop Arc-Length-Walk entlang forward Road-Hops. Startet bei cur_seg/t_cur
        // und hängt ab end_route_idx Vorwärts-Route-Hops an. Vereinheitlicht on-route &
        // Vorgänger-Kante (feeds-into):
        //   on-route Fall (cur_seg = route[j]→route[j+1], end_route_idx=j+1):
        //     erster Advance = route[j+1]→route[j+2].
        //   Vorgänger Fall (cur_seg = Snap-Kante endet an route[k], end_route_idx=k):
        //     erster Advance = route[k]→route[k+1].
        let mut cur = cur_seg;
        let mut cur_lut = build_lut(&index.segments[cur]);
        let mut arc_at = arc_length(&cur_lut, t_cur);
        let mut remaining = look_ahead;
        let mut next_route_idx = end_route_idx; // route-Index des END-Knotens von cur
        let mut hop_count = 0usize;

        let (final_seg, final_t) = loop {
            let arc_remaining = (cur_lut.total_length_m - arc_at).max(0.0);
            if remaining <= arc_remaining {
                let target_arc = arc_at + remaining;
                break (cur, t_at_arc_length(&cur_lut, &index.segments[cur], target_arc));
            }
            remaining -= arc_remaining;
            hop_count += 1;
            if hop_count >= SPLINE_LOOKAHEAD_MAX_HOPS {
                break (cur, 1.0);
            }
            if next_route_idx + 1 >= route.len() {
                break (cur, 1.0);
            }
            let (na, nb) = (route[next_route_idx], route[next_route_idx + 1]);
            match self.seg_by_from_to.get(&(na, nb)) {
                Some(&ni) => {
                    cur = ni;
                    cur_lut = build_lut(&index.segments[cur]);
                    arc_at = 0.0;
                    next_route_idx += 1;
                }
                None => break (cur, 1.0), // nächster Hop reversed/prefab/miss → konservativ clampen (2e-Grenze)
            }
        };

        // Lane-Offset aus Metadaten des Lande-Segments.
        let seg = &index.segments[final_seg];
        let look_point = evaluate(seg, final_t);
        let (lane_offset, source): (f32, &str) = match index.metadata[final_seg] {
            Some(m) if m.is_prefab => (0.0, "spline_prefab"),
            Some(m) => (m.lane_offset_right_m, "spline_road"),
            None => (LANE_OFFSET_RIGHT_M as f32, "spline_road"),
        };

        // Right-Normal an der lokalen Tangente (Fahrtrichtung = p0→p1, nur forward-Hops): n=(-tz,tx)/|t|.
        let tan = evaluate_tangent(seg, final_t);
        let len_xz = (tan.x * tan.x + tan.z * tan.z).sqrt();
        if len_xz < 1e-6 {
            ctx.blackboard
                .set("lane_keeper.fallback_reason", "route_miss");
            ctx.blackboard
                .set("lane_keeper.fallback_detail", "degenerate_tangent");
            return None;
        }
        let n_x = (-tan.z / len_xz) as f64;
        let n_z = (tan.x / len_xz) as f64;
        let look_x = look_point.x as f64 + n_x * lane_offset as f64;
        let look_z = look_point.z as f64 + n_z * lane_offset as f64;

        // ── Phase 2g-Diag2 (read-only): bricht die Kette Offset → Lenk-Zielpunkt? ──
        // Task 1: Soll-Linie sichtbar machen. steer_target = Lookahead-Punkt MIT Offset (geht in
        // den heading_error/Lenkung), centerline = derselbe Lookahead OHNE Offset.
        //   |steer_target - centerline| ≈ 0   → Offset NICHT im Zielpunkt (H1)
        //   |steer_target - centerline| ≈ 5.6 → Offset IST im Zielpunkt (weiter zu H2 / Control-Law)
        let steer_dx = look_x - look_point.x as f64;
        let steer_dz = look_z - look_point.z as f64;
        let steer_target_minus_centerline = (steer_dx * steer_dx + steer_dz * steer_dz).sqrt();
        ctx.blackboard
            .set("lane_keeper.steer_target_xz", format!("{look_x:.2},{look_z:.2}"));
        ctx.blackboard.set(
            "lane_keeper.spline_centerline_xz",
            format!("{:.2},{:.2}", look_point.x, look_point.z),
        );
        ctx.blackboard.set(
            "lane_keeper.steer_target_minus_centerline_m",
            format!("{steer_target_minus_centerline:.3}"),
        );
        // Task 2: Right-Normal + Richtung relativ zur TRUCK-Fahrtrichtung (nicht nur zur Segment-
        // Tangente). dir_dot>0 → Offset nach Truck-RECHTS (+1), <0 → links (-1, H2-Vorzeichenfehler).
        let h_cw = (-heading * std::f64::consts::TAU).rem_euclid(std::f64::consts::TAU);
        let truck_fx = h_cw.sin();
        let truck_fz = -h_cw.cos();
        let truck_right_x = -truck_fz; // right of travel = (-fz, fx)
        let truck_right_z = truck_fx;
        let dir_dot = n_x * truck_right_x + n_z * truck_right_z;
        let offset_direction_check = if (lane_offset as f64).abs() < 1e-6 {
            0.0
        } else if dir_dot >= 0.0 {
            1.0
        } else {
            -1.0
        };
        ctx.blackboard
            .set("lane_keeper.right_normal_xz", format!("{n_x:.3},{n_z:.3}"));
        ctx.blackboard
            .set("lane_keeper.offset_direction_check", format!("{offset_direction_check:.0}"));
        // Task 3: Truck-IST-Versatz gegen Centerline und gegen die Soll-Offset-Linie.
        //   truck_lat_vs_centerline ≈ 0    → Truck fährt MITTIG (Kette greift nicht)
        //   truck_lat_vs_centerline ≈ +5.6 → Truck auf der Soll-Spur (visuell evtl. fehlinterpretiert)
        //   truck_lat_vs_offsetline ≈ 0    → Truck IST auf der Offset-Linie
        ctx.blackboard.set(
            "lane_keeper.truck_lat_vs_centerline_m",
            format!("{truck_lat_vs_centerline:.3}"),
        );
        ctx.blackboard.set(
            "lane_keeper.truck_lat_vs_offsetline_m",
            format!("{:.3}", truck_lat_vs_centerline - lane_offset as f64),
        );
        // Task 4: Es gibt KEINEN Cross-Track-Term. steering_out = PID(heading_error), wobei
        // heading_error den Offset-Lookahead-Zielpunkt (look_x/look_z) nutzt. Referenz dokumentieren.
        ctx.blackboard.set(
            "lane_keeper.lat_error_reference",
            "heading_to_offset_lookahead_no_crosstrack",
        );

        // Diagnostik
        ctx.blackboard.set("lane_keeper.lateral_source", source);
        ctx.blackboard
            .set("lane_keeper.lane_offset_applied_m", format!("{lane_offset:.3}"));
        ctx.blackboard
            .set("lane_keeper.lookahead_hop_count", hop_count.to_string());
        ctx.blackboard
            .set("lane_keeper.current_hop", format!("{a0}->{b0}"));
        ctx.blackboard
            .set("lane_keeper.node_progress_idx", self.node_progress_idx.to_string());
        ctx.blackboard
            .set("lane_keeper.lookahead_m", format!("{look_ahead:.2}"));
        ctx.blackboard
            .set("lane_keeper.look_x", format!("{look_x:.2}"));
        ctx.blackboard
            .set("lane_keeper.look_z", format!("{look_z:.2}"));

        // Spline-Pfad hat gegriffen → Fallback-Grund = "none" (kein Fallback).
        ctx.blackboard.set("lane_keeper.fallback_reason", "none");
        ctx.blackboard.set("lane_keeper.fallback_detail", "none");

        // Shared Tail — IDENTISCH zum Catmull-Pfad; 2b-Heading-Konvert UNVERÄNDERT.
        let dx = look_x - tx;
        let dz = look_z - tz;
        if dx * dx + dz * dz < 1e-12 {
            return Some(0.0);
        }
        let target = dx.atan2(-dz);
        let heading_rad = (-heading * std::f64::consts::TAU).rem_euclid(std::f64::consts::TAU);
        let mut err = target - heading_rad;
        while err > std::f64::consts::PI {
            err -= 2.0 * std::f64::consts::PI;
        }
        while err < -std::f64::consts::PI {
            err += 2.0 * std::f64::consts::PI;
        }
        ctx.blackboard
            .set("lane_keeper.target_heading", format!("{target:.6}"));
        Some(err)
    }

    fn resync_progress_idx(&mut self, tx: f64, tz: f64) {
        if self.waypoints.len() < 2 {
            return;
        }
        let mut best_i = 0usize;
        let mut best_d = f64::MAX;
        for (i, &[wx, wz]) in self.waypoints.iter().enumerate() {
            let d = (tx - wx).powi(2) + (tz - wz).powi(2);
            if d < best_d {
                best_d = d;
                best_i = i;
            }
        }
        self.progress_idx = best_i.min(self.waypoints.len().saturating_sub(2));
    }

    fn compute_heading_error(
        &mut self,
        tx: f64,
        tz: f64,
        heading: f64,
        speed_ms: f64,
        ctx: &PluginContext,
    ) -> f64 {
        if let Some(err) = self.try_spline_heading_error(tx, tz, heading, speed_ms, ctx) {
            self.was_spline_active = true;
            return err;
        }
        if self.was_spline_active {
            self.was_spline_active = false;
            self.resync_progress_idx(tx, tz); // progress_idx auf nächsten Smoothed-Waypoint setzen
        }
        ctx.blackboard
            .set("lane_keeper.lateral_source", "catmullrom_fallback");

        if self.waypoints.len() < 2 {
            return 0.0;
        }

        while self.progress_idx + 1 < self.waypoints.len() {
            let [wx, wz] = self.waypoints[self.progress_idx + 1];
            let dist = ((tx - wx).powi(2) + (tz - wz).powi(2)).sqrt();
            if dist < WAYPOINT_REACH_M {
                self.progress_idx += 1;
            } else {
                break;
            }
        }

        if self.progress_idx + 1 >= self.waypoints.len() {
            ctx.blackboard.set("lane_keeper.skip_reason", "route_end");
            ctx.blackboard.set("lane_keeper.error_rad", "0.0");
            ctx.blackboard.set(
                "lane_keeper.progress_idx_after_advance",
                self.progress_idx.to_string(),
            );
            ctx.blackboard.set("lane_keeper.waypoints_remaining", "0");
            ctx.blackboard.set("lane_keeper.advance_check_dist", "0.00");
            ctx.blackboard.set("lane_keeper.walk_iterations", "0");
            ctx.blackboard.set("lane_keeper.walk_accumulated_m", "0.00");
            return 0.0;
        }

        let [nx, nz] = self.waypoints[self.progress_idx + 1];
        let advance_check_dist = ((tx - nx).powi(2) + (tz - nz).powi(2)).sqrt();

        let look_ahead = BASE_LOOK_AHEAD + speed_ms * 3.6 * SPEED_FACTOR;

        let mut look_x = tx;
        let mut look_z = tz;
        let mut accumulated = 0.0;
        let mut walk_iterations: usize = 0;

        for &[px, pz] in &self.waypoints[(self.progress_idx + 1)..] {
            let seg = ((px - look_x).powi(2) + (pz - look_z).powi(2)).sqrt();
            accumulated += seg;
            walk_iterations += 1;
            look_x = px;
            look_z = pz;
            if accumulated >= look_ahead {
                break;
            }
        }

        ctx.blackboard
            .set("lane_keeper.lookahead_m", format!("{look_ahead:.2}"));
        ctx.blackboard
            .set("lane_keeper.look_x", format!("{look_x:.2}"));
        ctx.blackboard
            .set("lane_keeper.look_z", format!("{look_z:.2}"));
        ctx.blackboard
            .set("lane_keeper.dx", format!("{:.2}", look_x - tx));
        ctx.blackboard
            .set("lane_keeper.dz", format!("{:.2}", look_z - tz));
        ctx.blackboard
            .set("lane_keeper.walk_iterations", walk_iterations.to_string());
        ctx.blackboard.set(
            "lane_keeper.walk_accumulated_m",
            format!("{accumulated:.2}"),
        );
        ctx.blackboard.set(
            "lane_keeper.advance_check_dist",
            format!("{advance_check_dist:.2}"),
        );
        ctx.blackboard.set(
            "lane_keeper.progress_idx_after_advance",
            self.progress_idx.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.waypoints_remaining",
            self.waypoints
                .len()
                .saturating_sub(self.progress_idx + 1)
                .to_string(),
        );

        let dx = look_x - tx;
        let dz = look_z - tz;
        if dx * dx + dz * dz < 1e-12 {
            return 0.0;
        }

        // Shift lookahead right by LANE_OFFSET_RIGHT_M (Rechtsfahrgebot).
        // Right-normal in ETS2 XZ (x=East, z=South): (-dz, dx) / |d|.
        // Mirrors lane-follower/src/lib.rs:913-918. Division safe: 1e-12 guard above.
        let len_xz = (dx * dx + dz * dz).sqrt();
        let look_x = look_x + (-dz / len_xz) * LANE_OFFSET_RIGHT_M;
        let look_z = look_z + (dx / len_xz) * LANE_OFFSET_RIGHT_M;
        let dx = look_x - tx;
        let dz = look_z - tz;

        let target = dx.atan2(-dz);
        // t.heading (Telemetry) is ETS2 SDK format: [0..1] CCW from North.
        // Convert to CW radians (0=N, π/2=E) to match target's convention.
        // Formula mirrors lane-follower: (-raw * 2π).rem_euclid(2π).
        let heading_rad =
            (-heading * std::f64::consts::TAU).rem_euclid(std::f64::consts::TAU);
        let mut err = target - heading_rad;
        while err > std::f64::consts::PI {
            err -= 2.0 * std::f64::consts::PI;
        }
        while err < -std::f64::consts::PI {
            err += 2.0 * std::f64::consts::PI;
        }

        ctx.blackboard
            .set("lane_keeper.target_heading", format!("{target:.6}"));
        err
    }

    fn tick_request_route_following(
        &mut self,
        telemetry: Option<&Telemetry>,
        ctx: &PluginContext,
    ) -> Option<ControlRequest> {
        // heading_stage is set by tick() before tick_request(). In tests that call
        // tick_request() directly, the field is pre-set via struct literal.
        // Do NOT re-read from blackboard here — that would overwrite the pre-set value.

        if !ctx.is_active() {
            self.pid.reset();
            self.previous_steering_out = 0.0;
            if !self.waypoints.is_empty() {
                self.waypoints.clear();
                self.last_waypoints_hash = 0;
                self.progress_idx = 0;
                tracing::info!("[lane-keeper] state=Off, cleared waypoint cache");
            }
            // Phase 2c/2d: Spline-Route-Zustand beim Disengage zurücksetzen —
            // unconditional, NICHT im waypoints-Sub-Block (der wird übersprungen
            // wenn waypoints schon leer). Sonst startet Re-Engage auf identischer
            // Route (gleicher route_node_ids-Hash → kein Reset in try_spline) mit
            // stale node_progress_idx mitten in der alten Route (Reviewer-Finding A).
            self.node_progress_idx = 0;
            self.was_spline_active = false;
            self.cached_route_hash = 0;
            self.cached_route_node_ids.clear();
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard
                .set("lane_keeper.skip_reason", "state_not_active");
            return None;
        }

        self.apply_gain_overrides(ctx);

        let t = telemetry?;

        if t.engine_rpm < 100.0 {
            self.pid.reset();
            self.previous_steering_out = 0.0;
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard.set("lane_keeper.skip_reason", "engine_off");
            return None;
        }

        if self.waypoints.is_empty() {
            ctx.blackboard.set("lane_keeper.skip_reason", "no_waypoints");
            ctx.blackboard.set("lane_keeper.active", "false");
            return None;
        }

        let dt = ctx.dt_s.min(0.1);

        let err =
            self.compute_heading_error(t.position[0], t.position[2], t.heading, t.speed_ms, ctx);

        if err.abs() > HEADING_MISMATCH_THRESHOLD_RAD {
            ctx.blackboard
                .set("lane_keeper.skip_reason", "heading_mismatch");
            ctx.blackboard.set("lane_keeper.heading_mismatch", "true");
            ctx.blackboard
                .set("lane_keeper.error_rad", format!("{err:.6}"));
            ctx.blackboard.set("lane_keeper.steering_out", "0.000000");
            ctx.blackboard.set("lane_keeper.active", "true");
            ctx.blackboard
                .set("lane_keeper.steering_rate_limited", "false");
            ctx.blackboard
                .set("lane_keeper.steering_delta_clamped", "0.0000");
            self.previous_steering_out = 0.0;
            self.pid.reset();
            return None;
        }
        ctx.blackboard.set("lane_keeper.heading_mismatch", "false");

        let stage = self.heading_stage.as_deref().unwrap_or("Normal");

        if matches!(stage, "AutoReplan" | "Disengaging") {
            ctx.blackboard
                .set("lane_keeper.skip_reason", "heading_stage");
            ctx.blackboard.set("lane_keeper.heading_mismatch", "true");
            ctx.blackboard
                .set("lane_keeper.error_rad", format!("{err:.6}"));
            ctx.blackboard.set("lane_keeper.steering_out", "0.000000");
            ctx.blackboard.set("lane_keeper.active", "true");
            ctx.blackboard
                .set("lane_keeper.steering_rate_limited", "false");
            ctx.blackboard
                .set("lane_keeper.steering_delta_clamped", "0.0000");
            self.previous_steering_out = 0.0;
            self.pid.reset();
            return None;
        }

        let stage_changed = self.previous_heading_stage != self.heading_stage;
        let transitional = stage_changed
            && (self.heading_stage.as_deref() == Some("SoftLaneKeep")
                || self.previous_heading_stage.as_deref() == Some("SoftLaneKeep"));
        if transitional {
            self.pid.reset();
        }
        self.previous_heading_stage = self.heading_stage.clone();

        let effective_err = if self.heading_stage.as_deref() == Some("SoftLaneKeep") {
            err * 0.3
        } else {
            err
        };

        let raw = self.pid.update(effective_err, dt).clamp(-1.0, 1.0);

        let delta_raw = raw - self.previous_steering_out;
        let delta_clamped =
            delta_raw.clamp(-STEERING_MAX_DELTA_PER_TICK, STEERING_MAX_DELTA_PER_TICK);
        let steering = self.previous_steering_out + delta_clamped;
        self.previous_steering_out = steering;

        let was_rate_limited = (delta_clamped - delta_raw).abs() > 1e-9;
        ctx.blackboard.set(
            "lane_keeper.steering_rate_limited",
            was_rate_limited.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.steering_delta_clamped",
            format!("{:.4}", delta_raw - delta_clamped),
        );

        ctx.blackboard.set("lane_keeper.active", "true");
        ctx.blackboard
            .set("lane_keeper.error_rad", format!("{err:.6}"));
        ctx.blackboard
            .set("lane_keeper.steering_out", format!("{steering:.6}"));
        ctx.blackboard.set(
            "lane_keeper.waypoints_loaded",
            self.waypoints.len().to_string(),
        );
        ctx.blackboard
            .set("lane_keeper.progress_idx", self.progress_idx.to_string());
        ctx.blackboard.set("lane_keeper.dt_s", format!("{dt:.6}"));
        ctx.blackboard
            .set("lane_keeper.truck_x", format!("{:.2}", t.position[0]));
        ctx.blackboard
            .set("lane_keeper.truck_z", format!("{:.2}", t.position[2]));
        ctx.blackboard
            .set("lane_keeper.truck_heading", format!("{:.6}", t.heading));
        ctx.blackboard
            .set("lane_keeper.truck_speed_ms", format!("{:.2}", t.speed_ms));

        Some(ControlRequest {
            steering: Some(steering),
            priority: PRIORITY_NORMAL,
            ..Default::default()
        })
    }
}

// ── Vision-mode implementation ────────────────────────────────────────────────

impl LaneKeeperPlugin {
    fn tick_request_vision(
        &mut self,
        telemetry: Option<&Telemetry>,
        ctx: &PluginContext,
    ) -> Option<ControlRequest> {
        let is_active = ctx.is_active();
        let was_active = self.was_active;
        self.was_active = is_active;

        // Active→Off: reset steering state but keep fallback accumulating so
        // engage_allowed can be re-evaluated while disengaged.
        if !is_active && was_active {
            self.pid.reset();
            self.previous_steering_out = 0.0;
            self.heading_hold.exit();
            self.engagement_heading = None;
            self.level_4_entered_at_tick = None;
            ctx.blackboard.set("lane_keeper.active", "false");
        }

        // Off→Active: fresh PID start; engagement_heading captured below on first tick.
        if is_active && !was_active {
            self.pid.reset();
            self.previous_steering_out = 0.0;
        }

        self.apply_gain_overrides(ctx);

        // Engine gate — applies in both Active and Off states.
        let t = telemetry?;
        if t.engine_rpm < 100.0 {
            self.pid.reset();
            self.previous_steering_out = 0.0;
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard.set("lane_keeper.engage_allowed", "false");
            ctx.blackboard.set("lane_keeper.skip_reason", "engine_off");
            return None;
        }

        let dt = ctx.dt_s.min(0.1);

        // Read lane perception (no telemetry dependency — blackboard only).
        let center_offset = ctx.blackboard.get_f64("lane.center_offset").unwrap_or(0.0);
        let confidence = ctx.blackboard.get_f64("lane.confidence").unwrap_or(0.0);
        let left_vis = ctx.blackboard.get("lane.left_visible").as_deref() == Some("true");
        let right_vis = ctx.blackboard.get("lane.right_visible").as_deref() == Some("true");
        // NaN → None (absent lane)
        let left_x = ctx
            .blackboard
            .get_f64("lane.left_x")
            .filter(|x| x.is_finite());
        let right_x = ctx
            .blackboard
            .get_f64("lane.right_x")
            .filter(|x| x.is_finite());

        // Update fallback cascade — runs always so engage_allowed reflects real
        // lane quality even while the autopilot is still in Off state.
        self.fallback.push_confidence(confidence);
        self.fallback.push_offset(center_offset);
        self.extrapolator.advance_tick();
        let avg_conf = self.fallback.rolling_avg_confidence();
        let level = self.fallback.update(avg_conf, left_vis, right_vis);

        // Publish engage_allowed always — breaks the Off-state chicken-and-egg.
        let engage_allowed = level <= 1;
        ctx.blackboard.set(
            "lane_keeper.engage_allowed",
            if engage_allowed { "true" } else { "false" },
        );
        ctx.blackboard
            .set("lane_keeper.fallback_level", level.to_string());

        if !is_active {
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard
                .set("lane_keeper.skip_reason", "computing_engage_allowed");
            return None;
        }

        // ── Active-only path ────────────────────────────────────────────────────

        // Capture engagement heading on the first tick of each Active session.
        if self.engagement_heading.is_none() {
            self.engagement_heading = Some(t.heading);
            tracing::info!("[lane-keeper] vision engage heading={:.4}", t.heading);
        }

        // Block-2 guard: heading drift since engagement.
        let eng_heading = self.engagement_heading.unwrap_or(t.heading);
        let heading_drift = wrap_angle(t.heading - eng_heading).abs();
        if heading_drift > HEADING_MISMATCH_THRESHOLD_RAD {
            ctx.blackboard
                .set("lane_keeper.skip_reason", "heading_mismatch");
            ctx.blackboard.set("lane_keeper.heading_mismatch", "true");
            ctx.blackboard.set("lane_keeper.active", "true");
            self.previous_steering_out = 0.0;
            self.pid.reset();
            return None;
        }
        ctx.blackboard.set("lane_keeper.heading_mismatch", "false");

        // Heading stage gate.
        self.heading_stage = ctx.blackboard.get("state.heading_stage");
        let stage = self.heading_stage.as_deref().unwrap_or("Normal");
        if matches!(stage, "AutoReplan" | "Disengaging") {
            ctx.blackboard
                .set("lane_keeper.skip_reason", "heading_stage");
            ctx.blackboard.set("lane_keeper.active", "true");
            self.previous_steering_out = 0.0;
            self.pid.reset();
            return None;
        }

        // Heading-hold transitions.
        if level == 3 && !self.heading_hold.active {
            self.heading_hold
                .enter(t.heading, self.tick_count, self.previous_steering_out);
            ctx.blackboard.set(
                "lane_keeper.level_3_entered_at_tick",
                self.fallback.level_entered_at_tick.to_string(),
            );
            tracing::warn!("[lane-keeper] entering L3 heading-hold h={:.4}", t.heading);
        } else if level != 3 && self.heading_hold.active {
            self.heading_hold.exit();
        }

        // Level-4 disengage.
        if level == 4 {
            let l4_start = *self.level_4_entered_at_tick.get_or_insert(self.tick_count);
            let ticks_since = self.tick_count.saturating_sub(l4_start);

            if ticks_since == 0 {
                let event = format!(
                    r#"{{"tick":{},"reason":"{}","confidence":{:.4},"blind_ticks":{}}}"#,
                    self.tick_count,
                    self.fallback.fallback_reason,
                    confidence,
                    self.fallback.blind_tick_count,
                );
                ctx.blackboard.set("lane_keeper.level_4_event_json", event);
                tracing::error!("[lane-keeper] LEVEL-4 DISENGAGE tick={}", self.tick_count);
            }

            let brake = if ticks_since < L4_BRAKE_TICKS {
                Some(0.30)
            } else {
                None
            };
            ctx.blackboard.set("lane_keeper.fallback_level", "4");
            ctx.blackboard.set("lane_keeper.active", "false");
            ctx.blackboard.set("lane_keeper.steering_source", "none_l4");
            self.publish_vision_diagnostics(ctx, level, avg_conf, 0.0);

            return Some(ControlRequest {
                steering: None,
                brake,
                priority: PRIORITY_LEVEL4,
                ..Default::default()
            });
        }
        self.level_4_entered_at_tick = None;

        // Speed-adaptive gain.
        let speed_kmh = t.speed_ms * 3.6;
        let speed_gain = if speed_kmh < 60.0 {
            1.2
        } else if speed_kmh <= 100.0 {
            1.0
        } else {
            0.8
        };

        // Per-level steering computation.
        let (vision_error, gain_factor, steering_source) = match level {
            0 => {
                // Normal vision: error = -center_offset
                (-center_offset, 1.0 * speed_gain, "vision_l0")
            }
            1 => {
                // Single-lane extrapolation
                let (err, _) =
                    extrapolate_center(left_x, right_x, avg_conf, &mut self.extrapolator);
                (err, 0.6 * speed_gain, "extrapolation_l1")
            }
            2 => {
                // Confidence-drop: EMA-weighted, same error source as L0
                (-center_offset, 0.35 * speed_gain, "vision_l2")
            }
            3 => {
                // Heading-hold: separate PID path, return early
                let pid_ref = &mut self.pid;
                let steering_l3 = self.heading_hold.compute_steering(
                    t.heading,
                    self.tick_count,
                    &mut |err, dt_val| pid_ref.update(err, dt_val),
                    dt,
                );

                // SoftLaneKeep scaling
                let scaled = if self.heading_stage.as_deref() == Some("SoftLaneKeep") {
                    steering_l3 * 0.3
                } else {
                    steering_l3
                };

                let output = self.rate_limit(scaled, ctx);
                ctx.blackboard.set("lane_keeper.active", "true");
                ctx.blackboard.set("lane_keeper.fallback_level", "3");
                ctx.blackboard
                    .set("lane_keeper.heading_hold_active", "true");
                ctx.blackboard.set(
                    "lane_keeper.hold_heading_rad",
                    format!("{:.6}", self.heading_hold.hold_heading),
                );
                ctx.blackboard.set(
                    "lane_keeper.heading_drift_rad",
                    format!("{:.6}", self.heading_hold.heading_drift(t.heading)),
                );
                ctx.blackboard.set(
                    "lane_keeper.heading_hold_ticks",
                    self.heading_hold.ticks_active(self.tick_count).to_string(),
                );
                ctx.blackboard
                    .set("lane_keeper.steering_source", "heading_hold_l3");
                self.publish_vision_diagnostics(ctx, level, avg_conf, output);

                return Some(ControlRequest {
                    steering: Some(output),
                    priority: PRIORITY_NORMAL,
                    ..Default::default()
                });
            }
            _ => (-center_offset, 0.0, "unknown"),
        };

        // PID update (levels 0, 1, 2).
        let raw_pid = self.pid.update(vision_error, dt).clamp(-1.0, 1.0);
        let scaled = (raw_pid * gain_factor).clamp(-1.0, 1.0);

        // SoftLaneKeep scaling (levels 0, 1, 2).
        let effective = if self.heading_stage.as_deref() == Some("SoftLaneKeep") {
            scaled * 0.3
        } else {
            scaled
        };

        // Rate limiter (Block-2).
        let output = self.rate_limit(effective, ctx);

        ctx.blackboard.set("lane_keeper.active", "true");
        ctx.blackboard
            .set("lane_keeper.fallback_level", level.to_string());
        ctx.blackboard
            .set("lane_keeper.heading_hold_active", "false");
        ctx.blackboard
            .set("lane_keeper.steering_source", steering_source);
        self.publish_vision_diagnostics(ctx, level, avg_conf, output);

        Some(ControlRequest {
            steering: Some(output),
            priority: PRIORITY_NORMAL,
            ..Default::default()
        })
    }

    fn publish_vision_diagnostics(
        &self,
        ctx: &PluginContext,
        level: u8,
        avg_conf: f64,
        steering: f64,
    ) {
        ctx.blackboard.set(
            "lane_keeper.fallback_reason",
            self.fallback.fallback_reason.clone(),
        );
        ctx.blackboard.set(
            "lane_keeper.confidence_trend",
            format!("{:.4}", self.fallback.compute_confidence_trend()),
        );
        ctx.blackboard.set(
            "lane_keeper.detection_stability",
            format!("{:.4}", self.fallback.compute_detection_stability()),
        );
        ctx.blackboard.set(
            "lane_keeper.blind_ticks",
            self.fallback.blind_tick_count.to_string(),
        );
        ctx.blackboard.set(
            "lane_keeper.blind_duration_ms",
            format!("{:.0}", self.fallback.blind_tick_count as f64 * 20.0),
        );
        ctx.blackboard.set("lane_keeper.single_lane_side", {
            let lv = ctx.blackboard.get("lane.left_visible").as_deref() == Some("true");
            let rv = ctx.blackboard.get("lane.right_visible").as_deref() == Some("true");
            match (lv, rv) {
                (true, false) => "left_only",
                (false, true) => "right_only",
                (true, true) => "both",
                (false, false) => "none",
            }
        });
        ctx.blackboard.set(
            "lane_keeper.lane_width_estimate_px",
            self.extrapolator
                .lane_width_estimate()
                .map(|w| format!("{w:.4}"))
                .unwrap_or_else(|| "NaN".to_string()),
        );
        ctx.blackboard
            .set("lane_keeper.steering_out", format!("{steering:.6}"));
        let _ = (level, avg_conf); // included in other keys already
    }
}

// ── Plugin trait impl ─────────────────────────────────────────────────────────

impl Plugin for LaneKeeperPlugin {
    fn name(&self) -> &str {
        "lane-keeper"
    }
    fn version(&self) -> &str {
        "0.1.0"
    }
    fn settings_schema(&self) -> &str {
        r#"{"type":"object","properties":{"kp":{"type":"number"},"ki":{"type":"number"},"kd":{"type":"number"},"subdivisions":{"type":"integer","minimum":1,"maximum":20}}}"#
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        self.load_waypoints_from_blackboard(ctx);
        // Pre-populate engage_allowed=false so the state machine never sees an absent key.
        ctx.blackboard.set("lane_keeper.engage_allowed", "false");
        tracing::info!("[lane-keeper] loaded, engage_allowed=false (pre-populated)");

        if let Some(shared) = &ctx.spline_index {
            self.index = Some(Arc::clone(shared));
            let road_n = ctx.spline_index_road_seg_count.min(shared.segments.len());
            let mut map = HashMap::with_capacity(road_n);
            for i in 0..road_n {
                let s = &shared.segments[i];
                map.insert((s.from_uid, s.to_uid), i);
            }
            self.seg_by_from_to = map;
            if let Some(rg) = &ctx.graph {
                self.router_graph = Some(Arc::clone(rg));
            }
            tracing::info!(
                "[lane-keeper] shared SplineIndex: {} road segs mapped (graph={})",
                self.seg_by_from_to.len(),
                self.router_graph.is_some()
            );
            // Phase 2c/2d-Diagnose (read-only): Index-Status auch ohne stdout per
            // blackboard-query lesbar machen.
            ctx.blackboard
                .set("lane_keeper.spline_index_present", "true");
            ctx.blackboard.set(
                "lane_keeper.seg_by_from_to_count",
                self.seg_by_from_to.len().to_string(),
            );
            ctx.blackboard.set(
                "lane_keeper.router_graph_present",
                self.router_graph.is_some().to_string(),
            );
        } else {
            tracing::warn!(
                "[lane-keeper] no shared SplineIndex; route-following uses Catmull-Rom fallback"
            );
            // Phase 2c/2d-Diagnose (read-only): expliziter Negativ-Status.
            ctx.blackboard
                .set("lane_keeper.spline_index_present", "false");
            ctx.blackboard
                .set("lane_keeper.seg_by_from_to_count", "0");
            ctx.blackboard
                .set("lane_keeper.router_graph_present", "false");
        }
    }

    fn on_unload(&mut self) {
        tracing::info!("[lane-keeper] unloaded");
    }

    fn tick(
        &mut self,
        _telemetry: Option<&Telemetry>,
        _output: &mut ControlOutput,
        ctx: &PluginContext,
    ) {
        self.heading_stage = ctx.blackboard.get("state.heading_stage");

        if self.mode == LaneKeeperMode::RouteFollowing
            && ctx.blackboard.get("router.active").as_deref() == Some("true")
        {
            let current_hash = ctx
                .blackboard
                .get("router.waypoints")
                .map(|j| hash_str(&j))
                .unwrap_or(0);
            if self.waypoints.is_empty() || current_hash != self.last_waypoints_hash {
                self.load_waypoints_from_blackboard(ctx);
            }
        }
    }

    fn tick_request(
        &mut self,
        telemetry: Option<&Telemetry>,
        ctx: &PluginContext,
    ) -> Option<ControlRequest> {
        self.tick_count += 1;

        // Re-check mode every 50 ticks (and on first tick).
        if self.tick_count == 1 || self.tick_count.is_multiple_of(50) {
            self.update_mode_from_blackboard(ctx);
        }

        match self.mode {
            LaneKeeperMode::RouteFollowing => self.tick_request_route_following(telemetry, ctx),
            LaneKeeperMode::Vision => self.tick_request_vision(telemetry, ctx),
            LaneKeeperMode::Off => None,
        }
    }
}

// ── Route-following utilities ─────────────────────────────────────────────────

fn smooth_catmull_rom(pts: &[[f64; 2]], subdivisions: usize) -> Vec<[f64; 2]> {
    if pts.len() < 2 {
        return pts.to_vec();
    }
    let n = pts.len();
    let mut result = Vec::with_capacity((n - 1) * (subdivisions + 1));

    for i in 1..n {
        let p0 = if i >= 2 { pts[i - 2] } else { pts[0] };
        let p1 = pts[i - 1];
        let p2 = pts[i];
        let p3 = if i + 1 < n { pts[i + 1] } else { pts[n - 1] };

        for j in 0..=subdivisions {
            if j == 0 && i > 1 {
                continue;
            }
            let t = j as f64 / subdivisions as f64;
            let t2 = t * t;
            let t3 = t2 * t;
            let x = 0.5
                * ((2.0 * p1[0])
                    + (-p0[0] + p2[0]) * t
                    + (2.0 * p0[0] - 5.0 * p1[0] + 4.0 * p2[0] - p3[0]) * t2
                    + (-p0[0] + 3.0 * p1[0] - 3.0 * p2[0] + p3[0]) * t3);
            let z = 0.5
                * ((2.0 * p1[1])
                    + (-p0[1] + p2[1]) * t
                    + (2.0 * p0[1] - 5.0 * p1[1] + 4.0 * p2[1] - p3[1]) * t2
                    + (-p0[1] + 3.0 * p1[1] - 3.0 * p2[1] + p3[1]) * t3);
            result.push([x, z]);
        }
    }
    result
}

fn hash_str(s: &str) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    s.hash(&mut h);
    h.finish()
}

truckpilot_plugin_api::export_plugin!(LaneKeeperPlugin);

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use truckpilot_plugin_api::SharedBlackboard;

    fn make_telemetry(speed_ms: f64, heading: f64) -> Telemetry {
        Telemetry {
            position: [0.0; 3],
            heading,
            pitch: 0.0,
            roll: 0.0,
            speed_ms,
            engine_rpm: 1200.0,
            cruise_control_kmh: 80.0,
            nav_speed_limit_kmh: -1.0,
            lead_vehicle_distance_m: -1.0,
            accel_longitudinal: -1.0,
            fuel_liters: -1.0,
            odometer_km: -1.0,
            nav_distance_m: -1.0,
            nav_time_s: -1.0,
        }
    }

    fn ctx_with_state(state: &str) -> PluginContext {
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", state);
        PluginContext::new("lane-keeper", bb)
    }

    fn fresh_ctx() -> PluginContext {
        let bb = SharedBlackboard::new();
        PluginContext::new("lane-keeper", bb)
    }

    fn active_plugin_with_straight_path() -> LaneKeeperPlugin {
        LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, -100.0], [0.0, -200.0]],
            ..Default::default()
        }
    }

    // ── Vision-mode helpers ───────────────────────────────────────────────────

    fn vision_bb(
        state: &str,
        center_offset: f64,
        confidence: f64,
        left: bool,
        right: bool,
    ) -> PluginContext {
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", state);
        bb.set("plugin.lane_keeper.mode", "vision");
        bb.set("lane.center_offset", center_offset.to_string());
        bb.set("lane.confidence", confidence.to_string());
        bb.set("lane.left_visible", left.to_string());
        bb.set("lane.right_visible", right.to_string());
        bb.set("lane.left_x", "NaN");
        bb.set("lane.right_x", "NaN");
        PluginContext::new("lane-keeper", bb)
    }

    fn make_vision_plugin() -> LaneKeeperPlugin {
        LaneKeeperPlugin {
            mode: LaneKeeperMode::Vision,
            ..Default::default()
        }
    }

    // ── Pre-existing route-following geometry tests ───────────────────────────

    #[test]
    fn look_ahead_increases_with_speed() {
        let slow = BASE_LOOK_AHEAD + 0.0 * SPEED_FACTOR;
        let fast = BASE_LOOK_AHEAD + 80.0 * SPEED_FACTOR;
        assert!(fast > slow);
        assert!((slow - 5.0).abs() < 0.01);
        assert!((fast - 45.0).abs() < 0.01);
    }

    #[test]
    fn straight_north_zero_error() {
        let mut lk = active_plugin_with_straight_path();
        let ctx = fresh_ctx();
        let err = lk.compute_heading_error(0.0, 0.0, 0.0, 10.0, &ctx);
        // After offset: lookahead shifts East → target slightly right of North → err > 0.
        assert!(err > 0.0, "truck on centerline, target right → positive error, got {err}");
    }

    #[test]
    fn turn_right_positive_error() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [100.0, 0.0]],
            ..Default::default()
        };
        let ctx = fresh_ctx();
        let err = lk.compute_heading_error(0.0, 0.0, 0.0, 10.0, &ctx);
        assert!(err > 0.0, "expected positive (right turn), got {err}");
    }

    #[test]
    fn catmull_rom_more_points_than_input() {
        let pts = vec![[0.0, 0.0], [50.0, 10.0], [100.0, 0.0]];
        let smoothed = smooth_catmull_rom(&pts, 4);
        assert!(smoothed.len() > pts.len());
    }

    #[test]
    fn test_state_gate_returns_none_when_off() {
        let mut lk = active_plugin_with_straight_path();
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Off");
        assert!(lk.tick_request(Some(&t), &ctx).is_none());
    }

    #[test]
    fn test_state_gate_returns_none_when_engaging() {
        let mut lk = active_plugin_with_straight_path();
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Engaging");
        assert!(lk.tick_request(Some(&t), &ctx).is_none());
    }

    #[test]
    fn test_active_steering_with_waypoints() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let req = lk
            .tick_request(Some(&t), &ctx)
            .expect("active must request");
        let s = req.steering.expect("active must request steering");
        assert!(s > 0.0, "expected positive steering, got {s}");
        assert_eq!(req.priority, PRIORITY_NORMAL);
    }

    #[test]
    fn test_straight_line_near_zero_steering() {
        let mut lk = active_plugin_with_straight_path();
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let req = lk.tick_request(Some(&t), &ctx).unwrap();
        let s = req.steering.unwrap();
        // After offset: truck on centerline → small positive steering toward right lane.
        assert!(s > 0.0 && s < 0.2, "positive steering toward right lane expected, got {s}");
    }

    #[test]
    fn heading_convention_north_is_zero() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, -100.0]],
            ..Default::default()
        };
        let ctx = fresh_ctx();
        let err = plugin.compute_heading_error(0.0, 0.0, 0.0, 13.88, &ctx);
        // After offset: lookahead shifts East → err > 0 (turn right toward right lane).
        assert!(err > 0.0, "truck on centerline, target right → positive error, got {err}");
    }

    #[test]
    fn heading_convention_east_is_half_pi() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [100.0, 0.0]],
            ..Default::default()
        };
        let ctx = fresh_ctx();
        // ETS2 East = 0.75 (0.75 CCW turns from North = 270° CCW = 90° CW = East)
        let err = plugin.compute_heading_error(0.0, 0.0, 0.75, 13.88, &ctx);
        // After offset: truck heads East, right lane is South (+z) → target shifts South
        // → target angle > π/2, heading_rad = π/2 → err > 0.
        assert!(err > 0.0, "truck on centerline heading East, target shifted South → err > 0, got {err}");
    }

    #[test]
    fn heading_convention_punkt_vor_rechts_kleiner_positiver_error() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [144.89, -156.22]],
            ..Default::default()
        };
        let ctx = fresh_ctx();
        // Truck heading ~20° CW from North in ETS2 [0..1] CCW format:
        // ets2 = 1 - 20°/360° = 0.9444; converts to 0.349 rad ≈ 20°.
        let err = plugin.compute_heading_error(0.0, 0.0, 0.9444, 13.88, &ctx);
        assert!(err > 0.0 && err < 0.6, "expected ~0.4 positive, got {err}");
    }

    #[test]
    fn ets2_raw_heading_gives_small_error_on_aligned_road() {
        // Regression guard: before fix, heading=0.9796 (ETS2 [0..1] for ~7.33° CW) was
        // subtracted directly from a radian target, producing a phantom −49° error
        // that immediately triggered AutoReplan (threshold 60°) and suppressed all steering.
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, -100.0]], // North road
            ..Default::default()
        };
        let ctx = fresh_ctx();
        // ETS2 heading 0.9796 ≈ 7.33° CW from North (truck nearly aligned with road).
        let err = plugin.compute_heading_error(0.0, 0.0, 0.9796, 0.0, &ctx);
        // Must be near-zero (≤15° = 0.26 rad), NOT −49° (−0.852 rad).
        assert!(
            err.abs() < 0.26,
            "ETS2 heading 0.9796 should give ~7° error, got {:.4} rad ({:.1}°)",
            err,
            err.to_degrees(),
        );
    }

    #[test]
    fn test_heading_wraparound() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [-0.1, 100.0]],
            ..Default::default()
        };
        // ETS2 [0..1] for ~179.4° CW (nearly South) ≈ 0.5016.
        // Converts to π-0.01 rad after rem_euclid, matching original test intent.
        let heading = 0.5016_f64;
        let ctx = fresh_ctx();
        let err = lk.compute_heading_error(0.0, 0.0, heading, 10.0, &ctx);
        assert!(err.abs() < 0.5, "wraparound produced {err}");
    }

    #[test]
    fn test_pid_reset_on_state_exit() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let active = ctx_with_state("Active");
        for _ in 0..10 {
            let _ = lk.tick_request(Some(&t), &active);
        }
        let off = ctx_with_state("Off");
        assert!(lk.tick_request(Some(&t), &off).is_none());

        lk.waypoints = vec![[0.0, 0.0], [20.0, -100.0]];

        let mut fresh = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let active2 = ctx_with_state("Active");
        let fresh_req = fresh.tick_request(Some(&t), &active2).unwrap();
        let resumed_req = lk.tick_request(Some(&t), &active2).unwrap();
        assert!(
            (fresh_req.steering.unwrap() - resumed_req.steering.unwrap()).abs() < 1e-6,
            "post-reset response must match a fresh PID"
        );
    }

    #[test]
    fn waypoints_reload_on_route_change() {
        let mut plugin = LaneKeeperPlugin::default();

        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", "Active");
        bb.set("router.active", "true");
        bb.set("router.waypoints", "[[0.0,0.0],[10.0,0.0],[20.0,0.0]]");
        let ctx = PluginContext::new("lane-keeper", bb.clone());

        plugin.tick(None, &mut ControlOutput::default(), &ctx);
        let first_len = plugin.waypoints.len();
        let first_hash = plugin.last_waypoints_hash;
        assert!(first_len > 0);
        assert!(first_hash != 0);

        bb.set(
            "router.waypoints",
            "[[100.0,100.0],[110.0,100.0],[120.0,100.0]]",
        );
        plugin.tick(None, &mut ControlOutput::default(), &ctx);

        assert_ne!(plugin.last_waypoints_hash, first_hash);
        assert!((plugin.waypoints[0][0] - 100.0).abs() < 1e-6);
        assert_eq!(plugin.progress_idx, 0);
    }

    #[test]
    fn test_dt_clamp_at_0_1() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", "Active");
        let ctx = PluginContext::new("lane-keeper", bb).with_dt(10.0);
        let req = lk.tick_request(Some(&t), &ctx).unwrap();
        let s = req.steering.unwrap();
        assert!((-1.0..=1.0).contains(&s), "output out of range: {s}");
    }

    #[test]
    fn route_end_returns_zero_error() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [10.0, 0.0]],
            progress_idx: 1,
            ..Default::default()
        };
        let ctx = fresh_ctx();
        let err = plugin.compute_heading_error(9.0, 0.0, 0.0, 0.0, &ctx);
        assert_eq!(err, 0.0, "Route-End-Guard must return 0.0, got {err}");
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("route_end"),
        );
    }

    #[test]
    fn progress_advances_when_truck_near_next_waypoint() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [10.0, 0.0], [20.0, 0.0]],
            progress_idx: 0,
            ..Default::default()
        };
        let ctx = fresh_ctx();
        plugin.compute_heading_error(6.0, 0.0, 0.0, 0.0, &ctx);
        assert_eq!(plugin.progress_idx, 1);
    }

    #[test]
    fn lookahead_starts_from_truck_position() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, 0.0], [40.0, 0.0]],
            progress_idx: 0,
            ..Default::default()
        };
        let ctx = fresh_ctx();
        plugin.compute_heading_error(0.0, 0.0, 0.0, 0.0, &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.look_x").as_deref(),
            Some("20.00")
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.look_z").as_deref(),
            Some("0.00")
        );
    }

    #[test]
    fn lookahead_walks_through_multiple_waypoints() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [3.0, 0.0], [6.0, 0.0], [9.0, 0.0], [12.0, 0.0]],
            progress_idx: 0,
            ..Default::default()
        };
        let ctx = fresh_ctx();
        plugin.compute_heading_error(0.0, 0.0, 0.0, 0.0, &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.walk_iterations").as_deref(),
            Some("1")
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.look_x").as_deref(),
            Some("6.00")
        );
    }

    #[test]
    fn heading_mismatch_above_threshold_returns_none() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, 100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        assert!(lk.tick_request(Some(&t), &ctx).is_none());
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("heading_mismatch")
        );
    }

    #[test]
    fn heading_mismatch_at_threshold_is_allowed() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        assert!(lk.tick_request(Some(&t), &ctx).is_some());
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.heading_mismatch")
                .as_deref(),
            Some("false")
        );
    }

    #[test]
    fn steering_rate_limit_clamps_large_delta() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let req = lk.tick_request(Some(&t), &ctx).unwrap();
        let s = req.steering.unwrap();
        assert!(s.abs() <= STEERING_MAX_DELTA_PER_TICK + 1e-9);
    }

    #[test]
    fn steering_rate_limit_passes_small_delta() {
        let mut lk = active_plugin_with_straight_path();
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let req = lk.tick_request(Some(&t), &ctx).unwrap();
        let s = req.steering.unwrap();
        assert!(s.abs() < STEERING_MAX_DELTA_PER_TICK);
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.steering_rate_limited")
                .as_deref(),
            Some("false")
        );
    }

    #[test]
    fn previous_steering_resets_on_disengage() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let active = ctx_with_state("Active");
        for _ in 0..5 {
            let _ = lk.tick_request(Some(&t), &active);
        }
        assert!(lk.previous_steering_out.abs() > 0.0);
        let off = ctx_with_state("Off");
        lk.tick_request(Some(&t), &off);
        assert_eq!(lk.previous_steering_out, 0.0);
    }

    #[test]
    fn previous_steering_resets_on_heading_mismatch() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let active = ctx_with_state("Active");
        for _ in 0..5 {
            let _ = lk.tick_request(Some(&t), &active);
        }
        assert!(lk.previous_steering_out.abs() > 0.0);
        lk.waypoints = vec![[0.0, 0.0], [0.0, 100.0]];
        let _ = lk.tick_request(Some(&t), &active);
        assert_eq!(lk.previous_steering_out, 0.0);
    }

    #[test]
    fn heading_convention_north_regression_unaffected_by_mismatch() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, -100.0]],
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let req = lk.tick_request(Some(&t), &ctx);
        assert!(req.is_some());
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.heading_mismatch")
                .as_deref(),
            Some("false")
        );
    }

    #[test]
    fn auto_replan_stage_returns_none() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            heading_stage: Some("AutoReplan".to_string()),
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        assert!(lk.tick_request(Some(&t), &ctx).is_none());
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("heading_stage")
        );
    }

    #[test]
    fn disengaging_stage_returns_none() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            heading_stage: Some("Disengaging".to_string()),
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        assert!(lk.tick_request(Some(&t), &ctx).is_none());
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("heading_stage")
        );
    }

    #[test]
    fn stage_change_soft_to_normal_resets_pid() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            heading_stage: Some("SoftLaneKeep".to_string()),
            previous_heading_stage: Some("Normal".to_string()),
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let _ = lk.tick_request(Some(&t), &ctx);
        assert_eq!(lk.previous_heading_stage, Some("SoftLaneKeep".to_string()));
    }

    #[test]
    fn normal_stage_produces_steering() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [20.0, -100.0]],
            heading_stage: Some("Normal".to_string()),
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let result = lk.tick_request(Some(&t), &ctx);
        assert!(result.is_some());
        assert!(result.unwrap().steering.is_some());
    }

    #[test]
    fn phase_6_5p_guard_takes_priority_over_heading_stage() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, 100.0]],
            heading_stage: Some("Normal".to_string()),
            ..Default::default()
        };
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        let result = lk.tick_request(Some(&t), &ctx);
        assert!(result.is_none());
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("heading_mismatch")
        );
    }

    #[test]
    fn route_following_no_waypoints_yields() {
        let mut lk = LaneKeeperPlugin::default();
        let t = make_telemetry(20.0, 0.0);
        let ctx = ctx_with_state("Active");
        assert!(lk.tick_request(Some(&t), &ctx).is_none());
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("no_waypoints")
        );
    }

    // ── Vision-mode tests ─────────────────────────────────────────────────────

    #[test]
    fn vision_level0_produces_steering() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        let ctx = vision_bb("Active", 0.1, 0.85, true, true);
        let req = plugin
            .tick_request(Some(&t), &ctx)
            .expect("L0 must produce steering");
        assert!(req.steering.is_some());
        assert_eq!(req.priority, PRIORITY_NORMAL);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_level").as_deref(),
            Some("0")
        );
    }

    #[test]
    fn vision_state_gate_returns_none_when_off() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        let ctx = vision_bb("Off", 0.0, 0.9, true, true);
        assert!(plugin.tick_request(Some(&t), &ctx).is_none());
        assert_eq!(
            ctx.blackboard.get("lane_keeper.active").as_deref(),
            Some("false")
        );
    }

    #[test]
    fn vision_engine_off_returns_none() {
        let mut plugin = make_vision_plugin();
        let mut t = make_telemetry(20.0, 0.0);
        t.engine_rpm = 0.0;
        let ctx = vision_bb("Active", 0.0, 0.9, true, true);
        assert!(plugin.tick_request(Some(&t), &ctx).is_none());
        assert_eq!(
            ctx.blackboard.get("lane_keeper.skip_reason").as_deref(),
            Some("engine_off")
        );
    }

    #[test]
    fn vision_engage_allowed_true_at_level0() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // Pump 5 ticks to build rolling avg > 0.70
        for _ in 0..5 {
            let ctx = vision_bb("Active", 0.0, 0.85, true, true);
            let _ = plugin.tick_request(Some(&t), &ctx);
        }
        let ctx = vision_bb("Active", 0.0, 0.85, true, true);
        let _ = plugin.tick_request(Some(&t), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.engage_allowed").as_deref(),
            Some("true")
        );
    }

    #[test]
    fn vision_engage_allowed_false_at_level2() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // Low confidence → level 2
        for _ in 0..5 {
            let ctx = vision_bb("Active", 0.0, 0.20, true, true);
            let _ = plugin.tick_request(Some(&t), &ctx);
        }
        let ctx = vision_bb("Active", 0.0, 0.20, true, true);
        let _ = plugin.tick_request(Some(&t), &ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.engage_allowed").as_deref(),
            Some("false")
        );
    }

    #[test]
    fn vision_level4_produces_high_priority_request() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // 105 blind ticks → L4
        for _ in 0..105 {
            let ctx = vision_bb("Active", 0.0, 0.0, false, false);
            let _ = plugin.tick_request(Some(&t), &ctx);
        }
        let ctx = vision_bb("Active", 0.0, 0.0, false, false);
        let req = plugin
            .tick_request(Some(&t), &ctx)
            .expect("L4 must produce ControlRequest");
        assert_eq!(req.priority, PRIORITY_LEVEL4);
        assert!(req.steering.is_none(), "L4 must not steer");
        assert_eq!(
            ctx.blackboard.get("lane_keeper.active").as_deref(),
            Some("false")
        );
    }

    #[test]
    fn vision_level4_brake_first_50_ticks() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // Get to L4
        for _ in 0..105 {
            let ctx = vision_bb("Active", 0.0, 0.0, false, false);
            let _ = plugin.tick_request(Some(&t), &ctx);
        }
        // First L4 tick → brake = 0.30
        let ctx = vision_bb("Active", 0.0, 0.0, false, false);
        let req = plugin.tick_request(Some(&t), &ctx).unwrap();
        assert_eq!(req.brake, Some(0.30), "first L4 tick must brake at 0.30");
    }

    #[test]
    fn vision_center_offset_steers_toward_lane_center() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // Positive center_offset = truck to the right → steer left (negative)
        // vision_error = -center_offset = -0.3 → negative PID → negative steering
        // Pump a few ticks so rolling avg stabilises before the final assertion.
        for _ in 0..5 {
            let c = vision_bb("Active", 0.3, 0.85, true, true);
            let _ = plugin.tick_request(Some(&t), &c);
        }
        let ctx = vision_bb("Active", 0.3, 0.85, true, true);
        let req = plugin.tick_request(Some(&t), &ctx).unwrap();
        let s = req.steering.unwrap();
        assert!(s < 0.0, "positive offset → steer left (negative), got {s}");
    }

    #[test]
    fn vision_rate_limiter_clamps_first_tick() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // Large offset → PID would want large output, rate-limiter clamps it
        let ctx = vision_bb("Active", 1.0, 0.95, true, true);
        let req = plugin.tick_request(Some(&t), &ctx).unwrap();
        if let Some(s) = req.steering {
            assert!(
                s.abs() <= STEERING_MAX_DELTA_PER_TICK + 1e-9,
                "rate limiter must clamp first-tick output, got {s}"
            );
        }
    }

    #[test]
    fn vision_reset_on_off_clears_engagement_heading() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        let ctx = vision_bb("Active", 0.0, 0.85, true, true);
        let _ = plugin.tick_request(Some(&t), &ctx);
        assert!(plugin.engagement_heading.is_some());

        let ctx_off = vision_bb("Off", 0.0, 0.0, false, false);
        let _ = plugin.tick_request(Some(&t), &ctx_off);
        assert!(plugin.engagement_heading.is_none());
    }

    #[test]
    fn vision_on_load_publishes_engage_allowed_false() {
        let mut plugin = make_vision_plugin();
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", "Off");
        let ctx = PluginContext::new("lane-keeper", bb);
        plugin.on_load(&ctx);
        assert_eq!(
            ctx.blackboard.get("lane_keeper.engage_allowed").as_deref(),
            Some("false")
        );
    }

    // ── Off-state engage_allowed tests (chicken-and-egg fix) ──────────────────

    #[test]
    fn vision_off_state_publishes_engage_allowed_with_good_lane_detection() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // Pump ticks in Off state with valid lane data — engage_allowed must become true.
        for _ in 0..5 {
            let ctx = vision_bb("Off", 0.0, 0.85, true, true);
            let result = plugin.tick_request(Some(&t), &ctx);
            assert!(result.is_none(), "Off state must produce no steering");
        }
        let ctx = vision_bb("Off", 0.0, 0.85, true, true);
        let result = plugin.tick_request(Some(&t), &ctx);
        assert!(result.is_none(), "Off state must produce no steering");
        assert_eq!(
            ctx.blackboard.get("lane_keeper.engage_allowed").as_deref(),
            Some("true")
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_level").as_deref(),
            Some("0")
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.active").as_deref(),
            Some("false")
        );
    }

    #[test]
    fn vision_off_state_produces_no_steering_output() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        let ctx = vision_bb("Off", 0.0, 0.85, true, true);
        assert!(plugin.tick_request(Some(&t), &ctx).is_none());
    }

    #[test]
    fn vision_active_state_produces_steering_output() {
        let mut plugin = make_vision_plugin();
        let t = make_telemetry(20.0, 0.0);
        // Warm up fallback in Active state so rolling avg is stable.
        for _ in 0..5 {
            let ctx = vision_bb("Active", 0.0, 0.85, true, true);
            let _ = plugin.tick_request(Some(&t), &ctx);
        }
        let ctx = vision_bb("Active", 0.0, 0.85, true, true);
        let req = plugin
            .tick_request(Some(&t), &ctx)
            .expect("Active must produce ControlRequest");
        assert!(req.steering.is_some(), "Active must produce steering");
    }

    // ── Lane-offset (Rechtsfahrgebot) tests ──────────────────────────────────

    #[test]
    fn lane_offset_north_road_shifts_target_right() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, -100.0]], // North road
            ..Default::default()
        };
        let ctx = fresh_ctx();
        // Truck on centerline heading North → offset shifts target East (+x).
        // err > 0: must steer right toward right lane.
        let err = plugin.compute_heading_error(0.0, 0.0, 0.0, 10.0, &ctx);
        assert!(
            err > 0.0 && err < 0.1,
            "north road: expected small positive error (target right of center), got {err:.4}",
        );
    }

    #[test]
    fn lane_offset_east_road_shifts_target_south() {
        let mut plugin = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [100.0, 0.0]], // East road
            ..Default::default()
        };
        let ctx = fresh_ctx();
        // Truck on centerline heading East (ETS2=0.75) → offset shifts target South (+z).
        // target angle > π/2, heading_rad = π/2 → err > 0.
        let err = plugin.compute_heading_error(0.0, 0.0, 0.75, 10.0, &ctx);
        assert!(
            err > 0.0 && err < 0.1,
            "east road: expected small positive error (target shifted South), got {err:.4}",
        );
    }

    // ── Phase 2c/2d: SplineIndex route-geometry tests ─────────────────────────
    //
    // These exercise `try_spline_heading_error` (the spline path) and its
    // Catmull-Rom fallback dispatch in `compute_heading_error`.

    use truckpilot_map_parser::build_index_with_metadata;
    use truckpilot_map_parser::spline::{HermiteSegment, SegmentMetadata, Vec3};

    /// Straight Hermite segment with chord tangents (m0=m1=p1-p0).
    fn seg(p0: (f32, f32), p1: (f32, f32), from: u64, to: u64) -> HermiteSegment {
        let a = Vec3::new(p0.0, 0.0, p0.1);
        let b = Vec3::new(p1.0, 0.0, p1.1);
        let m = b - a;
        HermiteSegment {
            p0: a,
            p1: b,
            m0: m,
            m1: m,
            length_m: m.length(),
            from_uid: from,
            to_uid: to,
            edge_uid: from * 100 + to,
        }
    }

    /// Road metadata: lane_offset_right_m = (lanes - 0.5) * width for non-prefab.
    fn road_meta(lanes: u8, w: f32, prefab: bool) -> SegmentMetadata {
        SegmentMetadata {
            lanes_in_direction: lanes,
            lanes_opposite: lanes,
            lanes_total: lanes * 2,
            lane_width_m: w,
            lane_offset_right_m: if prefab { 0.0 } else { (lanes as f32 - 0.5) * w },
            road_look_token: 0,
            is_prefab: prefab,
        }
    }

    /// Wire a plugin via on_load with the given segments/metadata, router graph and
    /// route_node_ids JSON. `road_seg_count` is how many segments are road-derived.
    fn wired_plugin(
        segs: Vec<HermiteSegment>,
        metas: Vec<Option<SegmentMetadata>>,
        nodes: Vec<(u64, f64, f64)>,
        edges: Vec<(u64, u64, f64)>,
        route_json: &str,
        road_seg_count: usize,
    ) -> (LaneKeeperPlugin, PluginContext) {
        let idx = Arc::new(build_index_with_metadata(segs, metas));
        let rg = Arc::new(RouterGraph::new(nodes, edges));
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", "Active");
        bb.set("router.route_node_ids", route_json);
        let mut ctx =
            PluginContext::new("lane-keeper", bb).with_spline_index(Arc::clone(&idx), road_seg_count);
        ctx.graph = Some(Arc::clone(&rg));
        let mut lk = LaneKeeperPlugin::default();
        lk.on_load(&ctx);
        (lk, ctx)
    }

    /// Test 1: 3-lane north road → lane_offset_right_m = (3-0.5)*3.75 = 9.375,
    /// lateral_source = "spline_road".
    #[test]
    fn spline_road_3lane_offset_9375() {
        let segs = vec![seg((0.0, 0.0), (0.0, -200.0), 10, 20)];
        let metas = vec![Some(road_meta(3, 3.75, false))];
        let nodes = vec![(10u64, 0.0, 0.0), (20u64, 0.0, -200.0)];
        let edges = vec![(10u64, 20u64, 200.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20]", 1);

        // Truck on centerline at origin, heading North (0.0), speed 20 m/s.
        let _err = lk.compute_heading_error(0.0, 0.0, 0.0, 20.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_road"),
            "must take the spline-road path"
        );
        let applied = ctx
            .blackboard
            .get("lane_keeper.lane_offset_applied_m")
            .and_then(|s| s.parse::<f64>().ok())
            .expect("lane_offset_applied_m must parse");
        assert!(
            (applied - 9.375).abs() < 0.01,
            "expected offset ≈ 9.375, got {applied}"
        );
        assert!(
            applied > 1.875,
            "spline offset must exceed the catmull constant 1.875, got {applied}"
        );
    }

    /// Test 2: sign check — north travel, offset shifts lookahead East (+x) →
    /// look_x > 0 and err > 0 (steer right).
    #[test]
    fn spline_offset_right_of_travel_direction() {
        let segs = vec![seg((0.0, 0.0), (0.0, -200.0), 10, 20)];
        let metas = vec![Some(road_meta(3, 3.75, false))];
        let nodes = vec![(10u64, 0.0, 0.0), (20u64, 0.0, -200.0)];
        let edges = vec![(10u64, 20u64, 200.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20]", 1);

        let err = lk.compute_heading_error(0.0, 0.0, 0.0, 20.0, &ctx);

        let look_x = ctx
            .blackboard
            .get("lane_keeper.look_x")
            .and_then(|s| s.parse::<f64>().ok())
            .expect("look_x must parse");
        assert!(look_x > 0.0, "right of North-travel = East (+x), got {look_x}");
        assert!(err > 0.0, "target right of heading → positive error, got {err}");
    }

    /// Test 3: route hop is reversed vs. the indexed segment direction →
    /// get((20,10)) misses → catmullrom_fallback.
    #[test]
    fn reversed_hop_falls_back_to_catmull() {
        // Map segment direction is 10→20, but the route walks 20→10.
        let segs = vec![seg((0.0, 0.0), (0.0, -200.0), 10, 20)];
        let metas = vec![Some(road_meta(3, 3.75, false))];
        let nodes = vec![(10u64, 0.0, 0.0), (20u64, 0.0, -200.0)];
        let edges = vec![(20u64, 10u64, 200.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[20,10]", 1);
        // Provide Catmull waypoints so the fallback path has geometry to chew on.
        lk.waypoints = vec![[0.0, 0.0], [0.0, -100.0], [0.0, -200.0]];

        lk.compute_heading_error(0.0, 0.0, 0.0, 20.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("catmullrom_fallback"),
            "reversed hop has no forward segment → must fall back"
        );
    }

    /// Test 4: route nodes not present in the segment map → catmullrom_fallback.
    #[test]
    fn route_miss_falls_back_to_catmull() {
        let segs = vec![seg((0.0, 0.0), (0.0, -200.0), 10, 20)];
        let metas = vec![Some(road_meta(3, 3.75, false))];
        // Graph has positions for 99/98 so node-advance can run, but no segment maps them.
        let nodes = vec![(99u64, 0.0, 0.0), (98u64, 0.0, -200.0)];
        let edges = vec![(99u64, 98u64, 200.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[99,98]", 1);
        lk.waypoints = vec![[0.0, 0.0], [0.0, -100.0], [0.0, -200.0]];

        lk.compute_heading_error(0.0, 0.0, 0.0, 20.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("catmullrom_fallback"),
            "unknown route nodes → must fall back"
        );
    }

    /// Test 5: no SplineIndex (default plugin, no on_load wiring) → existing
    /// Catmull-Rom behaviour, still produces a sensible error.
    #[test]
    fn no_index_falls_back_to_catmull() {
        let mut lk = LaneKeeperPlugin {
            waypoints: vec![[0.0, 0.0], [0.0, -100.0], [0.0, -200.0]],
            ..Default::default()
        };
        // ctx carries a route but no spline index → try_spline returns None early.
        let bb = SharedBlackboard::new();
        bb.set("autopilot.state", "Active");
        bb.set("router.route_node_ids", "[10,20]");
        let ctx = PluginContext::new("lane-keeper", bb);

        let err = lk.compute_heading_error(0.0, 0.0, 0.0, 20.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("catmullrom_fallback"),
            "index=None → must use catmull fallback"
        );
        // North road, truck on centerline → small positive error toward the right lane.
        assert!(
            err > 0.0 && err < 0.2,
            "fallback must still yield a sensible small positive error, got {err}"
        );
    }

    /// Test 6: prefab hop → lateral_source = "spline_prefab", offset ≈ 0.0.
    #[test]
    fn prefab_hop_zero_offset_spline_prefab() {
        let segs = vec![seg((0.0, 0.0), (0.0, -200.0), 10, 20)];
        let metas = vec![Some(road_meta(3, 3.75, true))]; // is_prefab=true
        let nodes = vec![(10u64, 0.0, 0.0), (20u64, 0.0, -200.0)];
        let edges = vec![(10u64, 20u64, 200.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20]", 1);

        lk.compute_heading_error(0.0, 0.0, 0.0, 20.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_prefab"),
            "prefab segment → spline_prefab source"
        );
        let applied = ctx
            .blackboard
            .get("lane_keeper.lane_offset_applied_m")
            .and_then(|s| s.parse::<f64>().ok())
            .expect("lane_offset_applied_m must parse");
        assert!(applied.abs() < 0.001, "prefab offset must be ≈ 0, got {applied}");
    }

    /// Test 7: 2 forward segments, lookahead crosses the segment boundary →
    /// lookahead_hop_count = 1, lateral_source = "spline_road".
    #[test]
    fn multi_hop_walk_crosses_boundary() {
        // Seg A: (0,0)→(0,-30) [10→20], Seg B: (0,-30)→(0,-200) [20→30].
        let segs = vec![
            seg((0.0, 0.0), (0.0, -30.0), 10, 20),
            seg((0.0, -30.0), (0.0, -200.0), 20, 30),
        ];
        let metas = vec![
            Some(road_meta(3, 3.75, false)),
            Some(road_meta(3, 3.75, false)),
        ];
        // Node 20 sits at (0,-30): far enough (>5m) from the truck at origin that
        // node_progress_idx does NOT advance, so the current hop stays 10→20.
        let nodes = vec![
            (10u64, 0.0, 0.0),
            (20u64, 0.0, -30.0),
            (30u64, 0.0, -200.0),
        ];
        let edges = vec![(10u64, 20u64, 30.0), (20u64, 30u64, 170.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20,30]", 2);

        // speed 20 m/s → look_ahead = 5 + 20*3.6*0.5 = 41m > 30m seg-A length → lands on seg B.
        lk.compute_heading_error(0.0, 0.0, 0.0, 20.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_road"),
            "multi-hop walk stays on the spline path"
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_keeper.lookahead_hop_count")
                .as_deref(),
            Some("1"),
            "41m lookahead must walk exactly one hop past the 30m boundary"
        );
    }

    /// Test 8: truck 100m laterally off the segment (> MAX_HOP_PROJECTION_DIST_M=40)
    /// → dist gate trips → catmullrom_fallback.
    #[test]
    fn dist_gate_far_truck_falls_back() {
        let segs = vec![seg((0.0, 0.0), (0.0, -200.0), 10, 20)];
        let metas = vec![Some(road_meta(3, 3.75, false))];
        // Graph node positions far from the truck so node-advance does not fire,
        // but the route does resolve to a forward hop; the per-segment projection
        // distance is what must trip the gate.
        let nodes = vec![(10u64, 100.0, 0.0), (20u64, 100.0, -200.0)];
        let edges = vec![(10u64, 20u64, 200.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20]", 1);
        lk.waypoints = vec![[100.0, 0.0], [100.0, -100.0], [100.0, -200.0]];

        // Truck at x=100, segment centerline at x=0 → projection distance ≈ 100m > 40m.
        lk.compute_heading_error(100.0, 0.0, 0.0, 20.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("catmullrom_fallback"),
            "projection distance > 40m must trip the dist gate → fallback"
        );
    }

    /// Test 9 (Finding-A regression): disengage resets node_progress_idx,
    /// was_spline_active and the cached route, so re-engage on the identical
    /// route does not resume mid-route with a stale index.
    #[test]
    fn disengage_resets_node_progress() {
        // 4-node route so node_progress can advance a couple of steps.
        let segs = vec![
            seg((0.0, 0.0), (0.0, -30.0), 10, 20),
            seg((0.0, -30.0), (0.0, -60.0), 20, 30),
            seg((0.0, -60.0), (0.0, -200.0), 30, 40),
        ];
        let metas = vec![
            Some(road_meta(3, 3.75, false)),
            Some(road_meta(3, 3.75, false)),
            Some(road_meta(3, 3.75, false)),
        ];
        let nodes = vec![
            (10u64, 0.0, 0.0),
            (20u64, 0.0, -30.0),
            (30u64, 0.0, -60.0),
            (40u64, 0.0, -200.0),
        ];
        let edges = vec![
            (10u64, 20u64, 30.0),
            (20u64, 30u64, 30.0),
            (30u64, 40u64, 140.0),
        ];
        let route = "[10,20,30,40]";
        let (mut lk, ctx) =
            wired_plugin(segs, metas, nodes.clone(), edges, route, 3);
        // Waypoints must be present, otherwise tick_request short-circuits at
        // "no_waypoints" before compute_heading_error advances node_progress_idx.
        // (In production the router writes both router.waypoints and
        // router.route_node_ids each tick.)
        lk.waypoints = vec![[0.0, 0.0], [0.0, -60.0], [0.0, -200.0]];

        // Drive a few Active ticks near node 20 (0,-30) then node 30 (0,-60) so
        // node_progress_idx advances past 0.
        let mut t = make_telemetry(20.0, 0.0);
        // Position the truck within reach (<5m) of node 20 → advance to idx 1.
        t.position = [0.0, 0.0, -28.0];
        let _ = lk.tick_request(Some(&t), &ctx);
        // Now within reach of node 30 → advance to idx 2.
        t.position = [0.0, 0.0, -58.0];
        let _ = lk.tick_request(Some(&t), &ctx);
        assert!(
            lk.node_progress_idx > 0,
            "precondition: node_progress_idx must have advanced (got {})",
            lk.node_progress_idx
        );

        // Disengage: state=Off tick.
        ctx.blackboard.set("autopilot.state", "Off");
        let off = lk.tick_request(Some(&t), &ctx);
        assert!(off.is_none(), "Off state yields no control request");

        // All spline route state must be reset.
        assert_eq!(
            lk.node_progress_idx, 0,
            "node_progress_idx must reset to 0 on disengage"
        );
        assert!(
            !lk.was_spline_active,
            "was_spline_active must reset to false on disengage"
        );
        assert_eq!(
            lk.cached_route_hash, 0,
            "cached_route_hash must reset to 0 on disengage"
        );
        assert!(
            lk.cached_route_node_ids.is_empty(),
            "cached_route_node_ids must be cleared on disengage"
        );

        // Re-engage on the IDENTICAL route from the start position → fresh start.
        ctx.blackboard.set("autopilot.state", "Active");
        let mut t_start = make_telemetry(20.0, 0.0);
        t_start.position = [0.0, 0.0, 0.0];
        let _ = lk.tick_request(Some(&t_start), &ctx);
        // At the route start, node 20 (0,-30) is 30m away (>5m) → no advance →
        // node_progress_idx stays 0 (no stale resume into the middle of the route).
        assert_eq!(
            lk.node_progress_idx, 0,
            "re-engage at start must keep node_progress_idx at 0, not resume stale"
        );
    }

    // ── Phase 2f-B: nearest-hop re-anchor tests ───────────────────────────────
    //
    // Geometry convention: straight Hermite segments in ETS2 XZ (x=East, z=South-negative
    // for northward travel). All segments are 100 m, so the second segment spans z=-100..-200.
    //
    //  Node 10 at (0,    0)
    //  Seg 0 : 10→20  (0,0) → (0,-100)   length 100 m
    //  Node 20 at (0, -100)
    //  Seg 1 : 20→30  (0,-100) → (0,-200) length 100 m
    //  Node 30 at (0, -200)

    /// Helper: 3-node north-road route `[10,20,30]` with two 100 m straight segments.
    /// Returns `(plugin, ctx)` wired via `on_load` (same pattern as existing spline tests).
    fn three_node_route() -> (LaneKeeperPlugin, PluginContext) {
        let segs = vec![
            seg((0.0, 0.0), (0.0, -100.0), 10, 20),
            seg((0.0, -100.0), (0.0, -200.0), 20, 30),
        ];
        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
        ];
        let nodes = vec![
            (10u64, 0.0, 0.0),
            (20u64, 0.0, -100.0),
            (30u64, 0.0, -200.0),
        ];
        let edges = vec![(10u64, 20u64, 100.0), (20u64, 30u64, 100.0)];
        wired_plugin(segs, metas, nodes, edges, "[10,20,30]", 2)
    }

    /// Test R1: Re-anchor advances `node_progress_idx` when truck is longitudinally
    /// deep on the second segment (near node 30).
    ///
    /// The old 5m-euclidean advance would never fire here because the truck never gets
    /// within 5 m of the median-node positions. The re-anchor scans all hops on the
    /// first (route_changed) tick and picks the hop whose segment is closest: that is
    /// hop 1 (20→30) because the truck is at z=-160, far from seg 0 (z=0..-100).
    #[test]
    fn reanchor_advances_on_longitudinal_progress() {
        let (mut lk, ctx) = three_node_route();
        // Truck sits 60 m into the second segment (z = -160, well past node 20 at z=-100).
        // On the first tick route_changed=true → full scan → seg 1 (20→30) wins.
        lk.compute_heading_error(0.0, -160.0, 0.0, 0.0, &ctx);

        assert_eq!(
            lk.node_progress_idx, 1,
            "re-anchor must select hop 1 (20→30) when truck is 60 m into the second segment; got {}",
            lk.node_progress_idx
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.current_hop").as_deref(),
            Some("20->30"),
            "current_hop must reflect the advanced hop"
        );
    }

    /// Test R2: Re-anchor is not blocked by lateral offset (the core fix).
    ///
    /// Truck is on the second segment (hop 1, 20→30) but displaced 5.6 m laterally —
    /// the typical right-lane offset in ETS2. The old euclidean node-advance checked
    /// distance to the MEDIAN node (0, -100) and found >5 m → never advanced. The
    /// re-anchor uses `project_on_segment`, which measures perpendicular distance, so
    /// the lateral offset costs only ~5.6 m (< MAX_HOP_PROJECTION_DIST_M = 40 m) and
    /// the second segment still wins against the first (which is ~100 m away along z).
    #[test]
    fn reanchor_lateral_offset_does_not_block_advance() {
        let (mut lk, ctx) = three_node_route();
        // Truck is longitudinally mid-second-segment (z=-150) and 5.6 m east (x=5.6).
        // project_on_segment for seg 0 (z=0..-100): truck is ~50 m past the end → clamped at t=1,
        // distance ≈ sqrt(5.6²+50²) ≈ 50 m.
        // project_on_segment for seg 1 (z=-100..-200): t ≈ 0.5, distance ≈ 5.6 m → wins.
        lk.compute_heading_error(5.6, -150.0, 0.0, 0.0, &ctx);

        assert_eq!(
            lk.node_progress_idx, 1,
            "lateral offset of 5.6 m must not prevent re-anchor to hop 1 (20→30); got {}",
            lk.node_progress_idx
        );
        // Confirm the spline path actually engaged (dist < 40 m gate).
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_road"),
            "spline path must engage despite lateral offset"
        );
    }

    /// Test R3: Re-anchor does not jump backward on a normal (non-replan) tick.
    ///
    /// `node_progress_idx` is pre-set to 1 (hop 20→30). On a non-replan tick the
    /// scan window is `[1, 1+16)` — forward-only. The truck is placed on hop 1,
    /// so hop 1 wins the scan. The index must not drop back to 0.
    #[test]
    fn reanchor_does_not_jump_backward_on_normal_tick() {
        let (mut lk, ctx) = three_node_route();

        // Prime the route cache so the SECOND call is a non-replan tick.
        // First call: route_changed=true, truck at beginning (hop 0 wins → idx stays 0).
        lk.compute_heading_error(0.0, -50.0, 0.0, 0.0, &ctx);

        // Manually advance idx to 1, then call again WITHOUT changing the route.
        // The second call sees route_changed=false → window [1, 17) → forward-only.
        lk.node_progress_idx = 1;
        // Truck still on the second segment.
        lk.compute_heading_error(0.0, -150.0, 0.0, 0.0, &ctx);

        assert!(
            lk.node_progress_idx >= 1,
            "non-replan re-anchor must not jump backward; idx dropped to {}",
            lk.node_progress_idx
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.current_hop").as_deref(),
            Some("20->30"),
            "current_hop must still be 20->30 after forward-only scan"
        );
    }

    /// Test R4: Re-anchor re-establishes position after a route change (replan).
    ///
    /// Simulates a replan: same geometry, but the route JSON string changes so
    /// `cached_route_hash` differs and `route_changed=true`. The truck is positioned
    /// deep in the route (hop 1). The full-route scan (lo=0, hi=route.len()-1) must
    /// pick hop 1, not remain stuck at 0.
    #[test]
    fn reanchor_picks_truck_segment_after_route_change() {
        let (mut lk, ctx) = three_node_route();

        // First tick: establish cache with truck on hop 0.
        lk.compute_heading_error(0.0, -50.0, 0.0, 0.0, &ctx);
        assert_eq!(lk.node_progress_idx, 0, "precondition: truck on hop 0");

        // Simulate a replan by writing a new route_node_ids JSON (same nodes, but
        // whitespace changes the hash so cached_route_hash ≠ new hash).
        ctx.blackboard
            .set("router.route_node_ids", "[ 10 , 20 , 30 ]");

        // Second tick: route_changed=true → full scan → truck now deep on hop 1.
        lk.compute_heading_error(0.0, -160.0, 0.0, 0.0, &ctx);

        assert_eq!(
            lk.node_progress_idx, 1,
            "after replan, full-route scan must re-anchor to hop 1 (truck at z=-160); got {}",
            lk.node_progress_idx
        );
    }

    // ── Phase 2g: 2D-nearest segment selection + Y-ignoring gate ───────────────
    //
    // These guard the Phase-2g fix in `try_spline_heading_error`:
    //   (a) the forward scan picks the geometrically nearest route segment by 2D
    //       (XZ) distance, NOT route[node_progress_idx]'s far segment;
    //   (b) the gate / reported distance is 2D — node-height (Y) is ignored.

    /// Straight Hermite segment with a fixed Y on both endpoints (height baked in).
    /// Tangents are chord = p1-p0 in XZ only (Y delta 0), matching `seg`'s flat tangent.
    fn seg_y(p0: (f32, f32), p1: (f32, f32), y: f32, from: u64, to: u64) -> HermiteSegment {
        let a = Vec3::new(p0.0, y, p0.1);
        let b = Vec3::new(p1.0, y, p1.1);
        let m = Vec3::new(p1.0 - p0.0, 0.0, p1.1 - p0.1);
        HermiteSegment {
            p0: a,
            p1: b,
            m0: m,
            m1: m,
            length_m: (b - a).length(),
            from_uid: from,
            to_uid: to,
            edge_uid: from * 100 + to,
        }
    }

    /// Test 2g-1 (H-C core case): the lookahead must pick the route segment the
    /// truck actually sits on, NOT route[0]'s far segment.
    ///
    /// Route `[10,20,30,40]`:
    ///   hop 0 (10→20) is a segment FAR from the truck (centerline at x=200, ≈200m off)
    ///   hop 2 (30→40) is the segment the truck sits on (≈2m off in XZ)
    /// The old "blind route[node_progress_idx]" lookup would anchor on hop 0 and trip
    /// the dist gate (>40m). The 2g scan must pick hop 2 → `node_progress_idx=2`,
    /// `truck_to_segment_dist_m` < 10m, `fallback_reason="none"`, `lateral_source="spline_road"`.
    #[test]
    fn reanchor_picks_truck_segment_not_route0_far_segment() {
        // hop 0/1 live way out east (x=200); hop 2 (30→40) runs north under the truck.
        let segs = vec![
            seg((200.0, 0.0), (200.0, -100.0), 10, 20),   // far
            seg((200.0, -100.0), (0.0, -100.0), 20, 30),  // connector
            seg((0.0, -100.0), (0.0, -300.0), 30, 40),    // under the truck
        ];
        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
        ];
        let nodes = vec![
            (10u64, 200.0, 0.0),
            (20u64, 200.0, -100.0),
            (30u64, 0.0, -100.0),
            (40u64, 0.0, -300.0),
        ];
        let edges = vec![
            (10u64, 20u64, 100.0),
            (20u64, 30u64, 200.0),
            (30u64, 40u64, 200.0),
        ];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20,30,40]", 3);

        // Truck on hop 2's centerline, ~2m west of it (x=2), mid-segment (z=-200), heading North.
        lk.compute_heading_error(2.0, -200.0, 0.0, 0.0, &ctx);

        assert_eq!(
            lk.node_progress_idx, 2,
            "scan must anchor on hop 2 (30→40, under the truck), not route[0]'s far hop; got {}",
            lk.node_progress_idx
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.current_hop").as_deref(),
            Some("30->40"),
            "current_hop must be the nearby segment"
        );
        let dist = ctx
            .blackboard
            .get("lane_keeper.truck_to_segment_dist_m")
            .and_then(|s| s.parse::<f64>().ok())
            .expect("truck_to_segment_dist_m must parse");
        assert!(
            dist < 10.0,
            "truck-to-segment distance must be the NEAR segment's (~2m), got {dist}"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("none"),
            "near segment is well within the gate → no fallback"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_road"),
            "spline path must engage on the near segment"
        );
    }

    /// Test 2g-2 (Y-2D proof): node height must be ignored when gating.
    ///
    /// A single segment sits ~2m (XZ) from the truck but is baked at y=37 (typical
    /// Berlin node height). The truck is queried at y=0. The OLD 3D
    /// `project_on_segment` distance would be ≈ sqrt(2² + 37²) ≈ 37m and — with a
    /// taller height — would exceed the 40m gate. The 2g fix measures XZ only, so the
    /// reported `truck_to_segment_dist_m` must be ~2m and the gate must NOT trip.
    #[test]
    fn gate_uses_2d_distance_ignoring_node_height() {
        let segs = vec![seg_y((0.0, 0.0), (0.0, -200.0), 37.0, 10, 20)];
        let metas = vec![Some(road_meta(2, 3.75, false))];
        let nodes = vec![(10u64, 0.0, 0.0), (20u64, 0.0, -200.0)];
        let edges = vec![(10u64, 20u64, 200.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20]", 1);

        // Truck 2m east of the centerline at y=0 (implicit), mid-segment.
        lk.compute_heading_error(2.0, -100.0, 0.0, 0.0, &ctx);

        let dist = ctx
            .blackboard
            .get("lane_keeper.truck_to_segment_dist_m")
            .and_then(|s| s.parse::<f64>().ok())
            .expect("truck_to_segment_dist_m must parse");
        assert!(
            (dist - 2.0).abs() < 0.5,
            "reported distance must be 2D (~2m), not 3D (~37m incl. node height), got {dist}"
        );
        // Direct proof the Y-fix matters: the 3D distance would already exceed the gate
        // for a taller node, and even at 37m it is far from the 2m we expect.
        assert!(
            dist < MAX_HOP_PROJECTION_DIST_M as f64,
            "2D distance is below the 40m gate; height must not inflate it (got {dist})"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("none"),
            "Y height ignored → gate does not trip → spline path engages"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_road"),
            "spline path must engage despite the 37m node height"
        );
    }

    /// Test 2g-3 (heading filter, U-turn): two route segments equally near the truck,
    /// one in the truck's heading direction and one doubled back (Δ≈180°). The
    /// heading-compatible candidate (same direction) must be chosen.
    ///
    /// Route `[10,20,30]`:
    ///   hop 0 (10→20): runs NORTH (z: 0 → -100). Truck heads North → compatible.
    ///   hop 1 (20→30): runs back SOUTH (z: -100 → 0), i.e. it folds back over hop 0.
    /// The truck sits at z=-50 — equidistant (in 2D) from both overlapping segments —
    /// but heading North. The heading filter (dot ≥ 0.5) must keep hop 0 and reject the
    /// reversed hop 1, so `node_progress_idx=0` and `current_hop="10->20"`.
    #[test]
    fn heading_filter_prefers_aligned_segment_over_doubled_back() {
        // Both segments occupy the SAME XZ corridor (x=0, z in [0,-100]) but opposite
        // direction, so 2D distance alone cannot disambiguate — only heading can.
        let segs = vec![
            seg((0.0, 0.0), (0.0, -100.0), 10, 20),   // North
            seg((0.0, -100.0), (0.0, 0.0), 20, 30),   // South (doubled back)
        ];
        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
        ];
        let nodes = vec![
            (10u64, 0.0, 0.0),
            (20u64, 0.0, -100.0),
            (30u64, 0.0, 0.0),
        ];
        let edges = vec![(10u64, 20u64, 100.0), (20u64, 30u64, 100.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20,30]", 2);

        // Truck mid-corridor (z=-50), heading North (0.0) → only hop 0 is heading-compatible.
        lk.compute_heading_error(0.0, -50.0, 0.0, 0.0, &ctx);

        assert_eq!(
            lk.node_progress_idx, 0,
            "heading filter must pick the North-aligned hop 0, not the doubled-back hop 1; got {}",
            lk.node_progress_idx
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.current_hop").as_deref(),
            Some("10->20"),
            "current_hop must be the heading-compatible segment"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_road"),
            "spline path must engage on the aligned segment"
        );
    }

    // ── Phase 2g (Variante B): global-nearest + route-relevance + W1/W2 ─────────
    //
    // These guard the Variante-B reroute of `try_spline_heading_error`:
    //   - the GLOBAL R-tree nearest-query (`nearest_with_heading_filter`) finds the
    //     geometrically closest, heading-compatible segment (not a route-only scan),
    //   - the route-relevance gate accepts it as ON-ROUTE (∃j: route[j]==F, route[j+1]==T)
    //     or FEEDS-INTO (∃k: route[k]==T = predecessor/snap edge), else falls back,
    //   - W1: feeds-into only if `hit.heading_filter_applied`,
    //   - W2: feeds-into only if `(route[k],route[k+1])` is a forward hop in seg_by_from_to.

    /// Test VB-1 (the real H-C case Variante b could NOT do): the truck SITS on the
    /// predecessor/snap edge P→A whose head is route[0]=A. The first on-route hop
    /// A→B is the far snap-endpoint hop (>40m east). The global-nearest query finds
    /// P→A (~2m), feeds-into A=route[0], heading-compatible (W1) and A→B is a forward
    /// hop (W2) → accepted. The route-only forward scan would have measured A→B at >40m
    /// and tripped the dist gate.
    ///
    /// Route `[20,30,40]` (A=20, B=30, C=40):
    ///   predecessor P→A = 10→20  : (0,0)→(0,-100), runs NORTH under the truck (~2m)
    ///   hop A→B        = 20→30  : (0,-100)→(200,-100), runs EAST, FAR from the truck
    ///   hop B→C        = 30→40  : (200,-100)→(200,-300)
    #[test]
    fn feeds_into_predecessor_edge_picks_snap_segment() {
        let segs = vec![
            seg((0.0, 0.0), (0.0, -100.0), 10, 20),       // predecessor P→A, under truck
            seg((0.0, -100.0), (200.0, -100.0), 20, 30),  // hop A→B, far (east)
            seg((200.0, -100.0), (200.0, -300.0), 30, 40),// hop B→C
        ];
        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
        ];
        let nodes = vec![
            (10u64, 0.0, 0.0),
            (20u64, 0.0, -100.0),
            (30u64, 200.0, -100.0),
            (40u64, 200.0, -300.0),
        ];
        let edges = vec![
            (10u64, 20u64, 100.0),
            (20u64, 30u64, 200.0),
            (30u64, 40u64, 200.0),
        ];
        // Route does NOT contain P (=10); it starts at A (=20).
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[20,30,40]", 3);

        // Truck on the predecessor edge P→A, 2m east of its centerline, mid-segment,
        // heading North → heading-compatible with the North-running P→A.
        lk.compute_heading_error(2.0, -50.0, 0.0, 0.0, &ctx);

        assert_eq!(
            lk.node_progress_idx, 0,
            "feeds-into must anchor on route[0]=A (the snap endpoint); got {}",
            lk.node_progress_idx
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.current_hop").as_deref(),
            Some("10->20"),
            "current_hop must be the predecessor/snap edge P->A"
        );
        let dist = ctx
            .blackboard
            .get("lane_keeper.truck_to_segment_dist_m")
            .and_then(|s| s.parse::<f64>().ok())
            .expect("truck_to_segment_dist_m must parse");
        assert!(
            dist < 10.0,
            "distance must be to the NEAR predecessor edge (~2m), not the far A->B hop, got {dist}"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("none"),
            "feeds-into accepted (W1+W2) → no fallback"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("spline_road"),
            "spline path must engage on the predecessor edge"
        );
    }

    /// Test VB-2 (off-route protection): the global-nearest segment is a PARALLEL road
    /// X→Y that is neither on-route nor feeds into a route node. The route hops are far
    /// away. The route-relevance gate must reject the nearest hit → off-route → Catmull.
    ///
    /// Route `[10,20,30]` (A→B→C), all hops far east (x=200).
    /// Parallel road X→Y = 90→91 : (0,0)→(0,-200) directly under the truck; neither 90
    /// nor 91 is a route node, and 91 is not the `to` of any route hop → off-route.
    #[test]
    fn off_route_nearest_segment_falls_back_to_catmull() {
        let segs = vec![
            seg((200.0, 0.0), (200.0, -100.0), 10, 20),    // route hop A->B (far east)
            seg((200.0, -100.0), (200.0, -200.0), 20, 30), // route hop B->C (far east)
            seg((0.0, 0.0), (0.0, -200.0), 90, 91),        // parallel road X->Y, under truck
        ];
        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
        ];
        let nodes = vec![
            (10u64, 200.0, 0.0),
            (20u64, 200.0, -100.0),
            (30u64, 200.0, -200.0),
            (90u64, 0.0, 0.0),
            (91u64, 0.0, -200.0),
        ];
        let edges = vec![
            (10u64, 20u64, 100.0),
            (20u64, 30u64, 100.0),
            (90u64, 91u64, 200.0),
        ];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[10,20,30]", 3);
        // Catmull fallback needs geometry to chew on.
        lk.waypoints = vec![[0.0, 0.0], [0.0, -100.0], [0.0, -200.0]];

        // Truck on the parallel road (x=2 east of X->Y centerline), heading North.
        // Nearest segment globally = X->Y (~2m), but it is off-route.
        lk.compute_heading_error(2.0, -100.0, 0.0, 0.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("off_route"),
            "nearest segment is a parallel road (not on-route, not feeds-into) → off_route"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("catmullrom_fallback"),
            "off-route nearest must NOT hijack the spline path → Catmull fallback"
        );
    }

    /// Test VB-3 (W2 explicit): feeds-into is rejected when the first forward walk-hop
    /// route[0]→route[1] does NOT exist as a forward segment.
    ///
    /// Route `[20,30]` (A=20, B=30). The truck sits on predecessor edge P→A = 10→20
    /// (head = A = route[0] → feeds-into candidate, heading-compatible → W1 passes).
    /// BUT the only indexed segment for the (A,B) corridor runs B→A (30→20), so
    /// `seg_by_from_to[(20,30)]` is MISSING → W2 fails → off_route → Catmull.
    ///
    /// This differs from `reversed_hop_falls_back_to_catmull` (no feeds-into edge there;
    /// the truck sits ON the reversed route corridor). Here the feeds-into branch is
    /// entered and only W2 stops it, so the W2 guard itself is exercised.
    #[test]
    fn feeds_into_rejected_when_no_forward_hop() {
        let segs = vec![
            seg((0.0, 0.0), (0.0, -100.0), 10, 20),       // predecessor P->A, under truck
            seg((0.0, -200.0), (0.0, -100.0), 30, 20),    // B->A only (no forward A->B)
        ];
        let metas = vec![
            Some(road_meta(2, 3.75, false)),
            Some(road_meta(2, 3.75, false)),
        ];
        let nodes = vec![
            (10u64, 0.0, 0.0),
            (20u64, 0.0, -100.0),
            (30u64, 0.0, -200.0),
        ];
        // Edge 30->20 indexed (so seg_by_from_to has (30,20) but NOT (20,30)).
        let edges = vec![(10u64, 20u64, 100.0), (30u64, 20u64, 100.0)];
        let (mut lk, ctx) = wired_plugin(segs, metas, nodes, edges, "[20,30]", 2);
        lk.waypoints = vec![[0.0, 0.0], [0.0, -100.0], [0.0, -200.0]];

        // Truck on P->A (10->20), 2m east, heading North → feeds-into A=route[0],
        // W1 passes (heading-compatible), but W2 fails: (20,30) is not a forward hop.
        lk.compute_heading_error(2.0, -50.0, 0.0, 0.0, &ctx);

        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_reason").as_deref(),
            Some("off_route"),
            "feeds-into without a forward route[0]->route[1] hop must be rejected (W2)"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.fallback_detail").as_deref(),
            Some("feeds_into_no_forward_hop"),
            "W2 rejection detail must indicate the missing forward hop"
        );
        assert_eq!(
            ctx.blackboard.get("lane_keeper.lateral_source").as_deref(),
            Some("catmullrom_fallback"),
            "W2 rejection → Catmull fallback"
        );
    }
}
