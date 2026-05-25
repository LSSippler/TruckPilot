//! Lane Data Collector — captures ETS2 frames + telemetry snapshots to disk.
//!
//! Runs at PhaseB (10 Hz). Every `capture_interval_ticks` ticks (default 50 = 5 s)
//! it reads the current camera frame from the SharedFrameStore (`camera.front`) and
//! writes a JPEG + sidecar JSON pair to `<output_dir>/<session_ts>/frame_NNNN.{jpg,json}`.
//!
//! ## Blackboard contract
//!
//! | Key                                      | Direction | Format        |
//! |------------------------------------------|-----------|---------------|
//! | `lane_data_collector.active`             | written   | "true"/"false"|
//! | `lane_data_collector.frames_saved`       | written   | u32 as string |
//! | `lane_data_collector.session_dir`        | written   | path string   |
//! | `lane_data_collector.stop`               | read      | "true" to halt|
//! | `lane_data_collector.output_dir`         | read      | path string   |
//! | `lane_data_collector.capture_interval_ticks` | read  | u32 as string |
//! | `lane_data_collector.max_frames`         | read      | u32 as string |

use std::fs;
use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

use serde::{Deserialize, Serialize};
use truckpilot_plugin_api::{ControlOutput, Plugin, PluginContext, Telemetry, TickPhase};

const DEFAULT_OUTPUT_DIR: &str = "data/captures";
const DEFAULT_CAPTURE_INTERVAL_TICKS: u32 = 50;
const DEFAULT_MAX_FRAMES: u32 = 500;
const FRAME_KEY: &str = "camera.front";

#[derive(Serialize, Deserialize)]
struct FrameSidecar {
    frame_idx: u32,
    timestamp: String,
    position: Position,
    heading_rad: f64,
    speed_ms: f64,
    /// Remaining navigation distance in metres. -1.0 if not available.
    nav_distance: f64,
    /// Remaining navigation time in seconds. -1.0 if not available.
    nav_time: f64,
}

#[derive(Serialize, Deserialize)]
struct Position {
    x: f64,
    z: f64,
}

pub struct LaneDataCollectorPlugin {
    output_dir: PathBuf,
    capture_interval_ticks: u32,
    max_frames: u32,
    tick_count: u64,
    frame_count: u32,
    session_dir: Option<PathBuf>,
    stopped: bool,
    no_store_warned: bool,
    no_frame_warned: bool,
}

impl Default for LaneDataCollectorPlugin {
    fn default() -> Self {
        Self {
            output_dir: PathBuf::from(DEFAULT_OUTPUT_DIR),
            capture_interval_ticks: DEFAULT_CAPTURE_INTERVAL_TICKS,
            max_frames: DEFAULT_MAX_FRAMES,
            tick_count: 0,
            frame_count: 0,
            session_dir: None,
            stopped: false,
            no_store_warned: false,
            no_frame_warned: false,
        }
    }
}

impl LaneDataCollectorPlugin {
    /// Create the per-session subdirectory on first capture. Returns `true` on success.
    fn ensure_session_dir(&mut self) -> bool {
        if self.session_dir.is_some() {
            return true;
        }
        let ts = Self::timestamp_for_dir();
        let dir = self.output_dir.join(&ts);
        match fs::create_dir_all(&dir) {
            Ok(()) => {
                tracing::info!("[lane-data-collector] session dir: {:?}", dir);
                self.session_dir = Some(dir);
                true
            }
            Err(e) => {
                tracing::warn!(
                    "[lane-data-collector] cannot create session dir {:?}: {e}",
                    dir
                );
                false
            }
        }
    }

    /// `YYYY-MM-DD_HHMMSS` from UNIX epoch (no chrono).
    pub fn timestamp_for_dir() -> String {
        let secs = Self::unix_secs();
        format_datetime(secs, '_', false)
    }

    /// ISO 8601 UTC timestamp: `YYYY-MM-DDTHH:MM:SSZ`.
    pub fn iso8601() -> String {
        let secs = Self::unix_secs();
        format_datetime(secs, 'T', true)
    }

    fn unix_secs() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs()
    }

    fn try_capture(&mut self, telemetry: Option<&Telemetry>, ctx: &PluginContext) {
        let store = match ctx.frame_store() {
            Some(s) => s,
            None => {
                if !self.no_store_warned {
                    tracing::warn!(
                        "[lane-data-collector] no SharedFrameStore — vision-frame-source not wired in"
                    );
                    self.no_store_warned = true;
                }
                return;
            }
        };

        let frame = match store.get(FRAME_KEY) {
            Some(f) => f,
            None => {
                if !self.no_frame_warned {
                    tracing::warn!(
                        "[lane-data-collector] no frame at '{}' — vision-frame-source not active",
                        FRAME_KEY
                    );
                    self.no_frame_warned = true;
                }
                return;
            }
        };

        if !self.ensure_session_dir() {
            return;
        }
        let session_dir = self.session_dir.as_ref().unwrap().clone();
        let idx = self.frame_count;
        let jpeg_path = session_dir.join(format!("frame_{idx:04}.jpg"));
        let json_path = session_dir.join(format!("frame_{idx:04}.json"));

        if let Err(e) = fs::write(&jpeg_path, frame.jpeg.as_ref()) {
            tracing::warn!("[lane-data-collector] write JPEG {:?}: {e}", jpeg_path);
            return;
        }

        let (pos_x, pos_z) = match telemetry {
            Some(t) => (t.position[0], t.position[2]),
            None => (
                ctx.blackboard
                    .get_f64("telemetry.position_x")
                    .unwrap_or(-1.0),
                ctx.blackboard
                    .get_f64("telemetry.position_z")
                    .unwrap_or(-1.0),
            ),
        };
        let heading = match telemetry {
            Some(t) => t.heading,
            None => ctx.blackboard.get_f64("telemetry.heading").unwrap_or(-1.0),
        };
        let speed = match telemetry {
            Some(t) => t.speed_ms,
            None => ctx.blackboard.get_f64("telemetry.speed_ms").unwrap_or(-1.0),
        };

        let (nav_distance, nav_time) = match telemetry {
            Some(t) => (t.nav_distance_m as f64, t.nav_time_s as f64),
            None => (
                ctx.blackboard.get_f64("nav.distance_m").unwrap_or(-1.0),
                ctx.blackboard.get_f64("nav.time_s").unwrap_or(-1.0),
            ),
        };

        let sidecar = FrameSidecar {
            frame_idx: idx,
            timestamp: Self::iso8601(),
            position: Position { x: pos_x, z: pos_z },
            heading_rad: heading,
            speed_ms: speed,
            nav_distance,
            nav_time,
        };

        match serde_json::to_string_pretty(&sidecar) {
            Ok(json) => {
                if let Err(e) = fs::write(&json_path, json.as_bytes()) {
                    tracing::warn!("[lane-data-collector] write JSON {:?}: {e}", json_path);
                    // JPEG already written — continue counting so naming stays consistent.
                }
            }
            Err(e) => {
                tracing::warn!("[lane-data-collector] serialize sidecar: {e}");
            }
        }

        self.frame_count += 1;
        self.no_frame_warned = false;

        ctx.blackboard.set(
            "lane_data_collector.frames_saved",
            self.frame_count.to_string(),
        );

        tracing::info!(
            "[lane-data-collector] saved frame_{idx:04} (total {}, pos=({pos_x:.1},{pos_z:.1}))",
            self.frame_count,
        );

        if self.frame_count >= self.max_frames {
            tracing::info!(
                "[lane-data-collector] reached max_frames={} — capture complete",
                self.max_frames
            );
            self.stopped = true;
            ctx.blackboard.set("lane_data_collector.active", "false");
        }
    }
}

impl Plugin for LaneDataCollectorPlugin {
    fn name(&self) -> &str {
        "lane-data-collector"
    }
    fn version(&self) -> &str {
        "0.1.0"
    }
    fn settings_schema(&self) -> &str {
        r#"{
  "type": "object",
  "properties": {
    "output_dir":              { "type": "string",  "default": "data/captures" },
    "capture_interval_ticks":  { "type": "integer", "default": 50 },
    "max_frames":              { "type": "integer", "default": 500 }
  }
}"#
    }

    fn default_phase(&self) -> TickPhase {
        TickPhase::PhaseB
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        if let Some(dir) = ctx.blackboard.get("lane_data_collector.output_dir") {
            self.output_dir = PathBuf::from(dir);
        }
        if let Some(n) = ctx
            .blackboard
            .get_f64("lane_data_collector.capture_interval_ticks")
        {
            self.capture_interval_ticks = n as u32;
        }
        if let Some(n) = ctx.blackboard.get_f64("lane_data_collector.max_frames") {
            self.max_frames = n as u32;
        }

        ctx.blackboard.set("lane_data_collector.active", "true");
        ctx.blackboard.set("lane_data_collector.frames_saved", "0");

        tracing::info!(
            "[lane-data-collector] loaded — output_dir={:?} interval={} max_frames={}",
            self.output_dir,
            self.capture_interval_ticks,
            self.max_frames,
        );
    }

    fn on_unload(&mut self) {
        tracing::info!(
            "[lane-data-collector] unloaded ({} frames saved in {:?})",
            self.frame_count,
            self.session_dir,
        );
    }

    fn tick(
        &mut self,
        telemetry: Option<&Telemetry>,
        _output: &mut ControlOutput,
        ctx: &PluginContext,
    ) {
        if self.stopped {
            return;
        }

        if ctx.blackboard.get("lane_data_collector.stop").as_deref() == Some("true") {
            tracing::info!("[lane-data-collector] stop flag set — halting capture");
            self.stopped = true;
            ctx.blackboard.set("lane_data_collector.active", "false");
            return;
        }

        self.tick_count += 1;

        if self
            .tick_count
            .is_multiple_of(u64::from(self.capture_interval_ticks))
        {
            self.try_capture(telemetry, ctx);
        }
    }
}

/// Format a UNIX epoch seconds value as a date/time string.
///
/// `sep` is the date/time separator (e.g. `'T'` for ISO 8601, `'_'` for dir names).
/// `colons` controls whether the time part uses `HH:MM:SS` (true) or `HHMMSS` (false).
pub fn format_datetime(secs: u64, sep: char, colons: bool) -> String {
    let ss = secs % 60;
    let mm = (secs / 60) % 60;
    let hh = (secs / 3600) % 24;
    let days = secs / 86400;
    // Gregorian approximation — good enough for timestamping over decades.
    let year = 1970 + days / 365;
    let doy = days % 365;
    let month = doy / 30 + 1;
    let day = doy % 30 + 1;
    if colons {
        format!("{year:04}-{month:02}-{day:02}{sep}{hh:02}:{mm:02}:{ss:02}Z")
    } else {
        format!("{year:04}-{month:02}-{day:02}{sep}{hh:02}{mm:02}{ss:02}")
    }
}

truckpilot_plugin_api::export_plugin!(LaneDataCollectorPlugin);

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use truckpilot_plugin_api::{
        ControlOutput, PluginContext, SharedFrame, SharedFrameStore, Telemetry, TickPhase,
    };

    fn mock_telemetry() -> Telemetry {
        Telemetry {
            position: [11283.5, 0.0, -7367.5],
            heading: 0.5711,
            pitch: 0.0,
            roll: 0.0,
            speed_ms: 13.88,
            engine_rpm: 1200.0,
            cruise_control_kmh: 0.0,
            nav_speed_limit_kmh: -1.0,
            lead_vehicle_distance_m: -1.0,
            accel_longitudinal: 0.0,
            fuel_liters: -1.0,
            odometer_km: -1.0,
            nav_distance_m: 1234.5,
            nav_time_s: 60.2,
        }
    }

    fn minimal_jpeg() -> Vec<u8> {
        // SOI + EOI — smallest valid JPEG marker sequence, sufficient for disk I/O tests.
        vec![0xFF, 0xD8, 0xFF, 0xD9]
    }

    fn ctx_with_store(store: Arc<SharedFrameStore>) -> PluginContext {
        PluginContext::test().with_frame_store(store)
    }

    // -----------------------------------------------------------------------

    #[test]
    fn timestamp_for_dir_format() {
        let ts = LaneDataCollectorPlugin::timestamp_for_dir();
        // YYYY-MM-DD_HHMMSS — 17 chars, contains underscore, no colons
        assert_eq!(ts.len(), 17, "expected 17 chars, got: {ts}");
        assert!(ts.contains('_'), "expected '_' separator: {ts}");
        assert!(!ts.contains(':'), "should have no colons: {ts}");
    }

    #[test]
    fn iso8601_format() {
        let ts = LaneDataCollectorPlugin::iso8601();
        assert_eq!(ts.len(), 20, "expected 20 chars, got: {ts}");
        assert!(ts.contains('T'), "expected 'T' separator: {ts}");
        assert!(ts.ends_with('Z'), "expected UTC 'Z' suffix: {ts}");
    }

    #[test]
    fn format_datetime_colons_flag() {
        let s = format_datetime(0, 'T', true);
        assert_eq!(s, "1970-01-01T00:00:00Z");
        let s = format_datetime(0, '_', false);
        assert_eq!(s, "1970-01-01_000000");
    }

    #[test]
    fn on_load_sets_blackboard_keys() {
        let mut plugin = LaneDataCollectorPlugin::default();
        let ctx = PluginContext::test();
        plugin.on_load(&ctx);
        assert_eq!(
            ctx.blackboard.get("lane_data_collector.active").as_deref(),
            Some("true")
        );
        assert_eq!(
            ctx.blackboard
                .get("lane_data_collector.frames_saved")
                .as_deref(),
            Some("0")
        );
    }

    #[test]
    fn tick_captures_jpeg_and_sidecar() {
        let tmp = std::env::temp_dir().join("truckpilot_ldc_test_capture");
        let _ = fs::remove_dir_all(&tmp);

        let store = Arc::new(SharedFrameStore::new());
        let frame = Arc::new(SharedFrame::new(1, 0, 640, 480, Arc::new(minimal_jpeg())));
        store.set("camera.front", frame);

        let mut plugin = LaneDataCollectorPlugin {
            output_dir: tmp.clone(),
            capture_interval_ticks: 1, // capture on every tick for testing
            max_frames: 500,
            ..Default::default()
        };

        let ctx = ctx_with_store(store);
        plugin.on_load(&ctx);

        let tel = mock_telemetry();
        let mut out = ControlOutput::default();

        // First tick → tick_count=1, 1%1=0 → capture
        plugin.tick(Some(&tel), &mut out, &ctx);

        // Find the session dir (named by timestamp)
        let entries: Vec<_> = fs::read_dir(&tmp)
            .expect("output_dir should exist")
            .flatten()
            .collect();
        assert_eq!(entries.len(), 1, "expected exactly one session dir");

        let session = &entries[0].path();
        let jpeg = session.join("frame_0000.jpg");
        let json = session.join("frame_0000.json");

        assert!(jpeg.exists(), "JPEG not found: {:?}", jpeg);
        assert!(json.exists(), "sidecar JSON not found: {:?}", json);

        // Verify JPEG bytes match what we put in
        let bytes = fs::read(&jpeg).unwrap();
        assert_eq!(bytes, minimal_jpeg());

        // Verify sidecar fields
        let raw = fs::read_to_string(&json).unwrap();
        let sidecar: FrameSidecar = serde_json::from_str(&raw).unwrap();
        assert_eq!(sidecar.frame_idx, 0);
        assert!((sidecar.position.x - 11283.5).abs() < 0.1);
        assert!((sidecar.position.z - (-7367.5)).abs() < 0.1);
        assert!((sidecar.heading_rad - 0.5711).abs() < 0.001);
        assert!((sidecar.speed_ms - 13.88).abs() < 0.01);

        assert_eq!(
            ctx.blackboard
                .get("lane_data_collector.frames_saved")
                .as_deref(),
            Some("1")
        );

        let _ = fs::remove_dir_all(&tmp);
    }

    #[test]
    fn no_capture_without_frame_store() {
        let mut plugin = LaneDataCollectorPlugin {
            capture_interval_ticks: 1,
            ..Default::default()
        };
        let ctx = PluginContext::test(); // no frame store
        plugin.on_load(&ctx);

        let tel = mock_telemetry();
        let mut out = ControlOutput::default();
        plugin.tick(Some(&tel), &mut out, &ctx);

        assert_eq!(plugin.frame_count, 0);
        assert!(plugin.no_store_warned);
        // Second tick should not log again (warned flag set)
        plugin.tick(Some(&tel), &mut out, &ctx);
        assert_eq!(plugin.frame_count, 0);
    }

    #[test]
    fn stop_flag_halts_capture() {
        let store = Arc::new(SharedFrameStore::new());
        let frame = Arc::new(SharedFrame::new(1, 0, 1, 1, Arc::new(minimal_jpeg())));
        store.set("camera.front", frame);

        let mut plugin = LaneDataCollectorPlugin {
            capture_interval_ticks: 1,
            ..Default::default()
        };
        let ctx = ctx_with_store(store);
        ctx.blackboard.set("lane_data_collector.stop", "true");
        plugin.on_load(&ctx);

        let tel = mock_telemetry();
        let mut out = ControlOutput::default();
        plugin.tick(Some(&tel), &mut out, &ctx);

        assert!(plugin.stopped);
        assert_eq!(plugin.frame_count, 0);
        assert_eq!(
            ctx.blackboard.get("lane_data_collector.active").as_deref(),
            Some("false")
        );
    }

    #[test]
    fn auto_stop_at_max_frames() {
        let tmp = std::env::temp_dir().join("truckpilot_ldc_test_maxframes");
        let _ = fs::remove_dir_all(&tmp);

        let store = Arc::new(SharedFrameStore::new());
        let frame = Arc::new(SharedFrame::new(1, 0, 1, 1, Arc::new(minimal_jpeg())));
        store.set("camera.front", frame);

        let mut plugin = LaneDataCollectorPlugin {
            output_dir: tmp.clone(),
            capture_interval_ticks: 1,
            max_frames: 3,
            ..Default::default()
        };
        let ctx = ctx_with_store(store);
        plugin.on_load(&ctx);

        let tel = mock_telemetry();
        let mut out = ControlOutput::default();

        for _ in 0..5 {
            plugin.tick(Some(&tel), &mut out, &ctx);
        }

        assert_eq!(plugin.frame_count, 3, "expected 3 frames then stop");
        assert!(plugin.stopped, "plugin should be stopped after max_frames");
        assert_eq!(
            ctx.blackboard.get("lane_data_collector.active").as_deref(),
            Some("false")
        );

        let _ = fs::remove_dir_all(&tmp);
    }

    #[test]
    fn interval_controls_capture_cadence() {
        let store = Arc::new(SharedFrameStore::new());
        let frame = Arc::new(SharedFrame::new(1, 0, 1, 1, Arc::new(minimal_jpeg())));
        store.set("camera.front", frame);

        // interval = 3 → capture at tick 3, 6, 9 (but temp dir → skip disk writes)
        // We just count how many capture attempts happen in 9 ticks.
        let tmp = std::env::temp_dir().join("truckpilot_ldc_test_interval");
        let _ = fs::remove_dir_all(&tmp);

        let mut plugin = LaneDataCollectorPlugin {
            output_dir: tmp.clone(),
            capture_interval_ticks: 3,
            max_frames: 500,
            ..Default::default()
        };
        let ctx = ctx_with_store(store);
        plugin.on_load(&ctx);

        let tel = mock_telemetry();
        let mut out = ControlOutput::default();

        for _ in 0..9 {
            plugin.tick(Some(&tel), &mut out, &ctx);
        }

        assert_eq!(
            plugin.frame_count, 3,
            "3 captures expected at ticks 3, 6, 9"
        );

        let _ = fs::remove_dir_all(&tmp);
    }

    #[test]
    fn default_phase_is_phase_b() {
        use truckpilot_plugin_api::Plugin;
        let p = LaneDataCollectorPlugin::default();
        assert_eq!(p.default_phase(), TickPhase::PhaseB);
    }
}
