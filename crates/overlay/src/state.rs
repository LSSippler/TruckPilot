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

        inner.last_update = Some(Instant::now());
    }

    pub fn set_nearby_segments(&self, segments: Vec<SegmentSnapshot>) {
        let mut inner = self.inner.lock().unwrap();
        inner.data.nearby_segments = segments;
        inner.last_update = Some(Instant::now());
    }
}
