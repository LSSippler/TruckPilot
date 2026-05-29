//! Shared HUD state, written by the WebSocket client and read by the render loop.
//!
//! All fields default to "no data". The render loop must tolerate stale or missing
//! values — when the daemon disconnects, the last-known values stay until reconnect.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Instant;

#[derive(Debug, Clone, Default)]
pub struct TruckPose {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub heading_deg: f32,
}

#[derive(Debug, Clone, Default)]
pub struct LaneFollower {
    pub nearest_seg_idx: Option<u64>,
    pub nearest_seg_is_prefab: bool,
    pub nearest_seg_dist_m: f32,
    pub nearest_seg_ai_path_uid: u64,
    pub lateral_dist_signed: f32,
    pub steering_filtered: f32,
}

#[derive(Debug, Clone, Default)]
pub struct Junction {
    pub detected: bool,
    pub distance_m: Option<f32>,
    pub phase: String,
}

#[derive(Debug, Clone, Default)]
pub struct BiasStatus {
    pub zone_active: bool,
    pub prefab_attempted: bool,
    pub prefab_accepted: bool,
    pub rejected_reason: String,
}

#[derive(Debug, Clone, Default)]
pub struct SegmentSnapshot {
    pub idx: u64,
    pub is_prefab: bool,
    pub start_x: f32,
    pub start_z: f32,
    pub end_x: f32,
    pub end_z: f32,
}

#[derive(Debug, Clone, Default)]
pub struct HudData {
    pub pose: TruckPose,
    pub lane: LaneFollower,
    pub junction: Junction,
    pub bias: BiasStatus,
    pub nearby_segments: Vec<SegmentSnapshot>,
    pub ds14_target: Option<(f32, f32)>,  // world (x, z) des DS14-Zielpunkts
    pub lateral_source: String,           // "navcurve" | "road_offset" | "road_center" | ""
}

impl HudData {
    /// Liefert den DS14-Zielpunkt nur, wenn er gezeichnet werden soll:
    /// lateral_source == "road_offset" UND ein Zielpunkt vorhanden ist.
    /// In allen anderen Fällen (navcurve / road_center / "" / kein Punkt) None.
    pub fn ds14_target_to_draw(&self) -> Option<(f32, f32)> {
        if self.lateral_source == "road_offset" {
            self.ds14_target
        } else {
            None
        }
    }
}

pub struct HudState {
    inner: Mutex<HudInner>,
}

struct HudInner {
    data: HudData,
    connected: bool,
    last_update: Option<Instant>,
}

impl HudState {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            inner: Mutex::new(HudInner {
                data: HudData::default(),
                connected: false,
                last_update: None,
            }),
        })
    }

    pub fn snapshot(&self) -> HudData {
        self.inner.lock().unwrap().data.clone()
    }

    pub fn is_connected(&self) -> bool {
        self.inner.lock().unwrap().connected
    }

    #[allow(dead_code)]
    pub fn age_ms(&self) -> Option<u128> {
        self.inner
            .lock()
            .unwrap()
            .last_update
            .map(|t| t.elapsed().as_millis())
    }

    pub fn set_connected(&self, connected: bool) {
        self.inner.lock().unwrap().connected = connected;
    }

    pub fn apply_blackboard(&self, values: &HashMap<String, String>) {
        let mut inner = self.inner.lock().unwrap();
        let d = &mut inner.data;

        if let Some(v) = values.get("lane_follower.truck_x").and_then(|s| s.parse().ok()) {
            d.pose.x = v;
        }
        if let Some(v) = values.get("lane_follower.truck_y").and_then(|s| s.parse().ok()) {
            d.pose.y = v;
        }
        if let Some(v) = values.get("lane_follower.truck_z").and_then(|s| s.parse().ok()) {
            d.pose.z = v;
        }
        if let Some(v) = values
            .get("lane_follower.truck_heading_deg")
            .and_then(|s| s.parse().ok())
        {
            d.pose.heading_deg = v;
        }

        if let Some(v) = values
            .get("lane_follower.nearest_seg_idx")
            .and_then(|s| s.parse().ok())
        {
            d.lane.nearest_seg_idx = Some(v);
        }
        d.lane.nearest_seg_is_prefab = values
            .get("lane_follower.nearest_seg_is_prefab")
            .map(|s| s == "true")
            .unwrap_or(false);
        if let Some(v) = values
            .get("lane_follower.nearest_seg_dist_m")
            .and_then(|s| s.parse().ok())
        {
            d.lane.nearest_seg_dist_m = v;
        }
        if let Some(v) = values
            .get("lane_follower.nearest_seg_ai_path_uid")
            .and_then(|s| s.parse().ok())
        {
            d.lane.nearest_seg_ai_path_uid = v;
        }
        if let Some(v) = values
            .get("lane_follower.lateral_dist_signed")
            .and_then(|s| s.parse().ok())
        {
            d.lane.lateral_dist_signed = v;
        }
        if let Some(v) = values
            .get("lane_follower.steering_filtered")
            .and_then(|s| s.parse().ok())
        {
            d.lane.steering_filtered = v;
        }

        d.junction.detected = values
            .get("lane_follower.junction_detected")
            .map(|s| s == "true")
            .unwrap_or(false);
        d.junction.distance_m = values
            .get("lane_follower.junction_distance_m")
            .and_then(|s| if s.is_empty() { None } else { s.parse().ok() });
        if let Some(v) = values.get("lane_follower.junction_phase") {
            d.junction.phase = v.clone();
        }

        d.bias.zone_active = values
            .get("lane_follower.bias_zone_active")
            .map(|s| s == "true")
            .unwrap_or(false);
        d.bias.prefab_attempted = values
            .get("lane_follower.bias_prefab_attempted")
            .map(|s| s == "true")
            .unwrap_or(false);
        d.bias.prefab_accepted = values
            .get("lane_follower.bias_prefab_accepted")
            .map(|s| s == "true")
            .unwrap_or(false);
        if let Some(v) = values.get("lane_follower.bias_prefab_rejected_reason") {
            d.bias.rejected_reason = v.clone();
        }

        d.lateral_source = values
            .get("lane_follower.lateral_source")
            .cloned()
            .unwrap_or_default();
        let off_x = values.get("lane_follower.lookahead_offset_x").and_then(|s| s.parse::<f32>().ok());
        let off_z = values.get("lane_follower.lookahead_offset_z").and_then(|s| s.parse::<f32>().ok());
        d.ds14_target = match (off_x, off_z) {
            (Some(x), Some(z)) => Some((x, z)),
            _ => None,
        };

        inner.last_update = Some(Instant::now());
    }

    pub fn set_nearby_segments(&self, segments: Vec<SegmentSnapshot>) {
        let mut inner = self.inner.lock().unwrap();
        inner.data.nearby_segments = segments;
        inner.last_update = Some(Instant::now());
    }
}

#[cfg(test)]
mod tests {
    use super::{HudData, HudState};
    use std::collections::HashMap;

    #[test]
    fn ds14_target_to_draw_road_offset_with_point() {
        let d = HudData {
            lateral_source: "road_offset".to_string(),
            ds14_target: Some((10.0, 20.0)),
            ..Default::default()
        };
        assert_eq!(d.ds14_target_to_draw(), Some((10.0, 20.0)));
    }

    #[test]
    fn ds14_target_to_draw_navcurve_returns_none() {
        let d = HudData {
            lateral_source: "navcurve".to_string(),
            ds14_target: Some((10.0, 20.0)),
            ..Default::default()
        };
        assert_eq!(d.ds14_target_to_draw(), None);
    }

    #[test]
    fn ds14_target_to_draw_road_center_returns_none() {
        let d = HudData {
            lateral_source: "road_center".to_string(),
            ds14_target: Some((5.0, 15.0)),
            ..Default::default()
        };
        assert_eq!(d.ds14_target_to_draw(), None);
    }

    #[test]
    fn ds14_target_to_draw_road_offset_no_point() {
        let d = HudData {
            lateral_source: "road_offset".to_string(),
            ds14_target: None,
            ..Default::default()
        };
        assert_eq!(d.ds14_target_to_draw(), None);
    }

    #[test]
    fn ds14_target_to_draw_empty_source_returns_none() {
        let d = HudData {
            lateral_source: String::new(),
            ds14_target: Some((1.0, 2.0)),
            ..Default::default()
        };
        assert_eq!(d.ds14_target_to_draw(), None);
    }

    // ------------------------------------------------------------------ //
    // apply_blackboard end-to-end parsing tests                           //
    // ------------------------------------------------------------------ //

    fn bb(pairs: &[(&str, &str)]) -> HashMap<String, String> {
        pairs.iter().map(|(k, v)| (k.to_string(), v.to_string())).collect()
    }

    /// road_offset + beide Offsets -> ds14_target Some, gate liefert Some.
    #[test]
    fn apply_bb_road_offset_both_offsets_parses_correctly() {
        let state = HudState::new();
        let map = bb(&[
            ("lane_follower.lateral_source", "road_offset"),
            ("lane_follower.lookahead_offset_x", "10152.345"),
            ("lane_follower.lookahead_offset_z", "-3.75"),
        ]);
        state.apply_blackboard(&map);
        let snap = state.snapshot();

        assert_eq!(snap.lateral_source, "road_offset");
        assert!(snap.ds14_target.is_some(), "ds14_target sollte Some sein");
        let (x, z) = snap.ds14_target.unwrap();
        assert!((x - 10152.345_f32).abs() < 0.01, "x={x} erwartet ~10152.345");
        assert!((z - (-3.75_f32)).abs() < 0.001, "z={z} erwartet ~-3.75");
        assert_eq!(snap.ds14_target_to_draw(), snap.ds14_target);
    }

    /// road_offset + nur lookahead_offset_x (z fehlt) -> ds14_target None, gate None.
    #[test]
    fn apply_bb_road_offset_only_x_gives_none() {
        let state = HudState::new();
        let map = bb(&[
            ("lane_follower.lateral_source", "road_offset"),
            ("lane_follower.lookahead_offset_x", "42.0"),
            // lookahead_offset_z fehlt absichtlich
        ]);
        state.apply_blackboard(&map);
        let snap = state.snapshot();

        assert_eq!(snap.lateral_source, "road_offset");
        assert_eq!(snap.ds14_target, None, "ohne z-Offset muss ds14_target None sein");
        assert_eq!(snap.ds14_target_to_draw(), None);
    }

    /// road_offset + nur lookahead_offset_z (x fehlt) -> ds14_target None.
    #[test]
    fn apply_bb_road_offset_only_z_gives_none() {
        let state = HudState::new();
        let map = bb(&[
            ("lane_follower.lateral_source", "road_offset"),
            // lookahead_offset_x fehlt absichtlich
            ("lane_follower.lookahead_offset_z", "99.9"),
        ]);
        state.apply_blackboard(&map);
        let snap = state.snapshot();

        assert_eq!(snap.ds14_target, None);
        assert_eq!(snap.ds14_target_to_draw(), None);
    }

    /// navcurve + beide Offsets -> ds14_target Some, aber gate sperrt (None).
    #[test]
    fn apply_bb_navcurve_gate_blocks_draw() {
        let state = HudState::new();
        let map = bb(&[
            ("lane_follower.lateral_source", "navcurve"),
            ("lane_follower.lookahead_offset_x", "1.0"),
            ("lane_follower.lookahead_offset_z", "2.0"),
        ]);
        state.apply_blackboard(&map);
        let snap = state.snapshot();

        assert_eq!(snap.lateral_source, "navcurve");
        // ds14_target wird gesetzt (Parsing ok), aber Gate sperrt
        assert_eq!(snap.ds14_target, Some((1.0, 2.0)));
        assert_eq!(snap.ds14_target_to_draw(), None, "navcurve: gate muss None liefern");
    }

    /// lateral_source-Key fehlt komplett -> leerer String, gate None.
    #[test]
    fn apply_bb_missing_lateral_source_defaults_to_empty() {
        let state = HudState::new();
        let map = bb(&[
            ("lane_follower.lookahead_offset_x", "5.0"),
            ("lane_follower.lookahead_offset_z", "6.0"),
            // lateral_source fehlt
        ]);
        state.apply_blackboard(&map);
        let snap = state.snapshot();

        assert_eq!(snap.lateral_source, "", "fehlender Key muss leeren String ergeben");
        assert_eq!(snap.ds14_target_to_draw(), None);
    }

    /// Leere Map (kein DS14-Key) darf nicht paniken, alle DS14-Felder bleiben Default.
    #[test]
    fn apply_bb_empty_map_does_not_panic() {
        let state = HudState::new();
        state.apply_blackboard(&HashMap::new());
        let snap = state.snapshot();

        assert_eq!(snap.lateral_source, "");
        assert_eq!(snap.ds14_target, None);
        assert_eq!(snap.ds14_target_to_draw(), None);
    }

    /// Ungültige (nicht-numerische) Offset-Werte -> Parsing schlägt fehl -> ds14_target None.
    #[test]
    fn apply_bb_unparseable_offsets_give_none() {
        let state = HudState::new();
        let map = bb(&[
            ("lane_follower.lateral_source", "road_offset"),
            ("lane_follower.lookahead_offset_x", "NOT_A_FLOAT"),
            ("lane_follower.lookahead_offset_z", "2.5"),
        ]);
        state.apply_blackboard(&map);
        let snap = state.snapshot();

        assert_eq!(snap.ds14_target, None, "ungültiger x-Wert muss ds14_target None lassen");
        assert_eq!(snap.ds14_target_to_draw(), None);
    }

    /// Mehrfache apply_blackboard-Aufrufe: zweiter Aufruf überschreibt den ersten.
    #[test]
    fn apply_bb_second_call_overwrites_first() {
        let state = HudState::new();

        // Erster Aufruf: road_offset mit Punkt
        state.apply_blackboard(&bb(&[
            ("lane_follower.lateral_source", "road_offset"),
            ("lane_follower.lookahead_offset_x", "1.0"),
            ("lane_follower.lookahead_offset_z", "2.0"),
        ]));
        assert_eq!(state.snapshot().lateral_source, "road_offset");

        // Zweiter Aufruf: navcurve, Offsets fehlen
        state.apply_blackboard(&bb(&[
            ("lane_follower.lateral_source", "navcurve"),
        ]));
        let snap = state.snapshot();
        assert_eq!(snap.lateral_source, "navcurve");
        assert_eq!(snap.ds14_target, None, "nach zweitem Aufruf ohne Offsets muss ds14_target None sein");
        assert_eq!(snap.ds14_target_to_draw(), None);
    }
}
