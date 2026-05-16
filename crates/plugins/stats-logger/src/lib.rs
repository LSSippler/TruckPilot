//! Stats-Logger plugin — records driving sessions to a SQLite database.
//!
//! ## Schema
//!
//! ```sql
//! CREATE TABLE sessions (
//!     id          INTEGER PRIMARY KEY AUTOINCREMENT,
//!     started_at  TEXT NOT NULL,          -- ISO 8601
//!     ended_at    TEXT,
//!     distance_km REAL NOT NULL DEFAULT 0,
//!     duration_s  REAL NOT NULL DEFAULT 0,
//!     avg_speed_kmh REAL,
//!     fuel_used_l REAL,
//!     pauses      INTEGER NOT NULL DEFAULT 0
//! );
//! CREATE TABLE tick_log (
//!     tick_id         INTEGER PRIMARY KEY AUTOINCREMENT,
//!     timestamp_ms    INTEGER NOT NULL,
//!     autopilot_state TEXT    NOT NULL,
//!     lateral_error_m REAL,
//!     heading_error_rad REAL,
//!     speed_kmh       REAL    NOT NULL,
//!     target_speed_kmh REAL   NOT NULL,
//!     throttle        REAL,
//!     brake           REAL,
//!     steering        REAL,
//!     lead_distance_m REAL,
//!     sign_limit_kmh  REAL,
//!     intervention    INTEGER NOT NULL DEFAULT 0
//! );
//! CREATE TABLE fault_log (
//!     fault_id        INTEGER PRIMARY KEY AUTOINCREMENT,
//!     timestamp_ms    INTEGER NOT NULL,
//!     reason          TEXT    NOT NULL,
//!     context         TEXT
//! );
//! CREATE TABLE pid_tuning_log (
//!     tuning_id       INTEGER PRIMARY KEY AUTOINCREMENT,
//!     timestamp_ms    INTEGER NOT NULL,
//!     plugin_name     TEXT    NOT NULL,
//!     parameter       TEXT    NOT NULL,
//!     old_value       REAL    NOT NULL,
//!     new_value       REAL    NOT NULL,
//!     set_by          TEXT    NOT NULL DEFAULT 'manual'
//! );
//! ```
//!
//! ## Blackboard contract
//!
//! | Key                      | Written by   | Read by |
//! |--------------------------|--------------|---------|
//! | `stats.session_id`                     | stats-logger | UI           |
//! | `stats.distance_km`                    | stats-logger | UI           |
//! | `stats.duration_s`                     | stats-logger | UI           |
//! | `stats_logger.tick_log_hz`             | UI/config    | stats-logger |
//! | `autopilot.intervention_steering`      | external     | stats-logger |
//! | `autopilot.intervention_brake`         | external     | stats-logger |
//! | `autopilot.fault_reason`               | core         | stats-logger |
//! | `pid_tuning.speed_controller.{kp,ki,kd}` | speed-controller | stats-logger |
//! | `pid_tuning.lane_keeper.{kp,ki,kd}`    | lane-keeper  | stats-logger |

use std::path::PathBuf;
use std::sync::Mutex;
use std::time::Instant;

use rusqlite::{params, Connection};
use truckpilot_plugin_api::{ControlOutput, Plugin, PluginContext, Telemetry};

const DEFAULT_DB_PATH: &str = "stats.db";
const DEFAULT_TICK_LOG_HZ: f64 = 10.0;

/// Monotonic timestamp in ms for tick log entries.
fn monotonic_ms() -> u64 {
    use std::sync::atomic::{AtomicU64, Ordering};
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    COUNTER.fetch_add(20, Ordering::Relaxed) // ~20ms per tick at 50Hz
}

/// Snapshot of a driving session.
struct Session {
    id: i64,
    #[allow(dead_code)]
    started_at: String,
    distance_km: f64,
    duration_s: f64,
    fuel_start_l: f64,
    pauses: u32,
}

pub struct StatsLoggerPlugin {
    db_path: PathBuf,
    conn: Option<Mutex<Connection>>,
    session: Option<Session>,
    last_odometer_km: f64,
    last_tick: Option<Instant>,
    was_in_break: bool,
    /// Track last state for intervention detection.
    last_state: Option<String>,
    /// Tick counter for per-tick logging frequency.
    tick_idx: u64,
    /// Subscribed to pid_tuning changes.
    last_observed_gains: [(String, (f64, f64, f64)); 2],
}

impl Default for StatsLoggerPlugin {
    fn default() -> Self {
        Self {
            db_path: PathBuf::from(DEFAULT_DB_PATH),
            conn: None,
            session: None,
            last_odometer_km: 0.0,
            last_tick: None,
            was_in_break: false,
            last_state: None,
            tick_idx: 0,
            last_observed_gains: [
                ("speed_controller".into(), (0.25, 0.08, 0.06)),
                ("lane_keeper".into(), (0.8, 0.1, 0.3)),
            ],
        }
    }
}

impl StatsLoggerPlugin {
    fn open_db(&mut self) -> Result<(), String> {
        let conn =
            Connection::open(&self.db_path).map_err(|e| format!("open {:?}: {e}", self.db_path))?;

        conn.execute_batch(
            "CREATE TABLE IF NOT EXISTS sessions (
                id            INTEGER PRIMARY KEY AUTOINCREMENT,
                started_at    TEXT    NOT NULL,
                ended_at      TEXT,
                distance_km   REAL    NOT NULL DEFAULT 0,
                duration_s    REAL    NOT NULL DEFAULT 0,
                avg_speed_kmh REAL,
                fuel_used_l   REAL,
                pauses        INTEGER NOT NULL DEFAULT 0
            );

            CREATE TABLE IF NOT EXISTS tick_log (
                tick_id          INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp_ms     INTEGER NOT NULL,
                autopilot_state  TEXT    NOT NULL,
                lateral_error_m  REAL,
                heading_error_rad REAL,
                speed_kmh        REAL    NOT NULL,
                target_speed_kmh  REAL   NOT NULL,
                throttle         REAL,
                brake            REAL,
                steering         REAL,
                lead_distance_m  REAL,
                sign_limit_kmh   REAL,
                intervention     INTEGER NOT NULL DEFAULT 0
            );

            CREATE INDEX IF NOT EXISTS idx_tick_log_time ON tick_log(timestamp_ms);
            CREATE INDEX IF NOT EXISTS idx_tick_log_state ON tick_log(autopilot_state);

            CREATE TABLE IF NOT EXISTS fault_log (
                fault_id        INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp_ms    INTEGER NOT NULL,
                reason          TEXT    NOT NULL,
                context         TEXT
            );

            CREATE TABLE IF NOT EXISTS pid_tuning_log (
                tuning_id       INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp_ms    INTEGER NOT NULL,
                plugin_name     TEXT    NOT NULL,
                parameter       TEXT    NOT NULL,
                old_value       REAL    NOT NULL,
                new_value       REAL    NOT NULL,
                set_by          TEXT    NOT NULL DEFAULT 'manual'
            );",
        )
        .map_err(|e| format!("create table: {e}"))?;

        self.conn = Some(Mutex::new(conn));
        Ok(())
    }

    fn start_session(&mut self, odometer_km: f64, fuel_l: f64) {
        let conn_mutex = match &self.conn {
            Some(c) => c,
            None => return,
        };
        let conn = match conn_mutex.lock() {
            Ok(c) => c,
            Err(_) => return,
        };

        let now = now_iso8601();
        match conn.execute(
            "INSERT INTO sessions (started_at, distance_km, duration_s, pauses) VALUES (?1, 0, 0, 0)",
            params![now],
        ) {
            Ok(_) => {
                let id = conn.last_insert_rowid();
                tracing::info!("[stats-logger] session #{id} started");
                self.session = Some(Session {
                    id,
                    started_at: now,
                    distance_km: 0.0,
                    duration_s: 0.0,
                    fuel_start_l: fuel_l,
                    pauses: 0,
                });
                self.last_odometer_km = odometer_km;
            }
            Err(e) => tracing::warn!("[stats-logger] insert session: {e}"),
        }
    }

    fn update_session(&mut self, odometer_km: f64, dt: f64, in_break: bool) {
        let session = match &mut self.session {
            Some(s) => s,
            None => return,
        };

        let delta_km = (odometer_km - self.last_odometer_km).max(0.0);
        self.last_odometer_km = odometer_km;
        session.distance_km += delta_km;
        session.duration_s += dt;

        // Count each break start as one pause.
        if in_break && !self.was_in_break {
            session.pauses += 1;
        }
        self.was_in_break = in_break;
    }

    fn flush_session(&mut self, fuel_end_l: f64) {
        let session = match &self.session {
            Some(s) => s,
            None => return,
        };
        let conn_mutex = match &self.conn {
            Some(c) => c,
            None => return,
        };
        let conn = match conn_mutex.lock() {
            Ok(c) => c,
            Err(_) => return,
        };

        let avg_speed = if session.duration_s > 0.0 {
            session.distance_km / (session.duration_s / 3600.0)
        } else {
            0.0
        };
        let fuel_used = (session.fuel_start_l - fuel_end_l).max(0.0);
        let now = now_iso8601();

        if let Err(e) = conn.execute(
            "UPDATE sessions SET ended_at=?1, distance_km=?2, duration_s=?3,
             avg_speed_kmh=?4, fuel_used_l=?5, pauses=?6 WHERE id=?7",
            params![
                now,
                session.distance_km,
                session.duration_s,
                avg_speed,
                fuel_used,
                session.pauses,
                session.id,
            ],
        ) {
            tracing::warn!("[stats-logger] update session: {e}");
        } else {
            tracing::info!(
                "[stats-logger] session #{} ended — {:.1}km in {:.0}min ({:.1}km/h avg)",
                session.id,
                session.distance_km,
                session.duration_s / 60.0,
                avg_speed,
            );
        }
    }

    fn write_tick_log(&mut self, t: &Telemetry, ctx: &PluginContext) {
        let ts = monotonic_ms();
        let state = ctx.state().unwrap_or_else(|| "Unknown".into());
        let speed_kmh = t.speed_ms * 3.6;

        let target_kmh = ctx
            .blackboard
            .get_f64("speed_controller.target_speed_kmh")
            .unwrap_or(speed_kmh);

        let sign_limit = ctx.blackboard.get_f64("sign.speed_limit_kmh");
        let lead_dist = if t.lead_vehicle_distance_m >= 0.0 {
            Some(t.lead_vehicle_distance_m as f64)
        } else {
            None
        };

        let intervention = if self.detect_intervention(ctx) { 1 } else { 0 };

        // Lock DB only after all blackboard reads + mutable self calls.
        let conn_mutex = match &self.conn {
            Some(c) => c,
            None => return,
        };
        let conn = match conn_mutex.lock() {
            Ok(c) => c,
            Err(_) => return,
        };

        if let Err(e) = conn.execute(
            "INSERT INTO tick_log (timestamp_ms, autopilot_state, lateral_error_m, heading_error_rad,
             speed_kmh, target_speed_kmh, throttle, brake, steering, lead_distance_m,
             sign_limit_kmh, intervention)
             VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, ?12)",
            params![
                ts,
                state,
                None::<f64>, // lateral_error_m — not yet available
                None::<f64>, // heading_error_rad — not yet available
                speed_kmh,
                target_kmh,
                None::<f64>, // throttle — from arbitrator, not visible in tick()
                None::<f64>, // brake — same
                None::<f64>, // steering — same
                lead_dist,
                sign_limit,
                intervention,
            ],
        ) {
            tracing::warn!("[stats-logger] tick_log insert: {e}");
        }

        self.tick_idx += 1;
    }

    fn detect_intervention(&mut self, ctx: &PluginContext) -> bool {
        let current = ctx.state();
        let was_active = self.last_state.as_deref() == Some("Active");
        let now_off = current.as_deref() == Some("Off");
        let now_fault = current.as_deref() == Some("Fault");

        // State change: Active -> Off (manual disengage) or Active -> Fault.
        let state_driven = was_active && (now_off || now_fault);

        // Explicit intervention flags from other plugins.
        let steer = ctx
            .blackboard
            .get("autopilot.intervention_steering")
            .as_deref()
            == Some("true");
        let brake = ctx
            .blackboard
            .get("autopilot.intervention_brake")
            .as_deref()
            == Some("true");

        self.last_state = current;
        state_driven || steer || brake
    }

    fn write_fault_log(&mut self, ctx: &PluginContext) {
        let current = ctx.state();
        let was_active = self.last_state.as_deref() == Some("Active");
        let now_fault = current.as_deref() == Some("Fault");

        if !was_active || !now_fault {
            return;
        }

        let conn_mutex = match &self.conn {
            Some(c) => c,
            None => return,
        };
        let conn = match conn_mutex.lock() {
            Ok(c) => c,
            Err(_) => return,
        };

        let ts = monotonic_ms();
        let reason = ctx
            .blackboard
            .get("autopilot.fault_reason")
            .unwrap_or_else(|| "unknown".into());

        if let Err(e) = conn.execute(
            "INSERT INTO fault_log (timestamp_ms, reason, context) VALUES (?1, ?2, ?3)",
            params![ts, reason, None::<String>],
        ) {
            tracing::warn!("[stats-logger] fault_log insert: {e}");
        }
    }

    fn write_pid_tuning_changes(&mut self, ctx: &PluginContext) {
        let conn_mutex = match &self.conn {
            Some(c) => c,
            None => return,
        };
        let conn = match conn_mutex.lock() {
            Ok(c) => c,
            Err(_) => return,
        };

        let ts = monotonic_ms();

        for (name, param, _blackboard_key, old_val, new_val) in [
            (
                "speed_controller",
                "kp",
                "pid_tuning.speed_controller.kp",
                self.last_observed_gains[0].1 .0,
                ctx.blackboard.get_f64("pid_tuning.speed_controller.kp"),
            ),
            (
                "speed_controller",
                "ki",
                "pid_tuning.speed_controller.ki",
                self.last_observed_gains[0].1 .1,
                ctx.blackboard.get_f64("pid_tuning.speed_controller.ki"),
            ),
            (
                "speed_controller",
                "kd",
                "pid_tuning.speed_controller.kd",
                self.last_observed_gains[0].1 .2,
                ctx.blackboard.get_f64("pid_tuning.speed_controller.kd"),
            ),
            (
                "lane_keeper",
                "kp",
                "pid_tuning.lane_keeper.kp",
                self.last_observed_gains[1].1 .0,
                ctx.blackboard.get_f64("pid_tuning.lane_keeper.kp"),
            ),
            (
                "lane_keeper",
                "ki",
                "pid_tuning.lane_keeper.ki",
                self.last_observed_gains[1].1 .1,
                ctx.blackboard.get_f64("pid_tuning.lane_keeper.ki"),
            ),
            (
                "lane_keeper",
                "kd",
                "pid_tuning.lane_keeper.kd",
                self.last_observed_gains[1].1 .2,
                ctx.blackboard.get_f64("pid_tuning.lane_keeper.kd"),
            ),
        ] {
            if let Some(new_v) = new_val {
                if (new_v - old_val).abs() > 1e-9 {
                    let _ = conn.execute(
                        "INSERT INTO pid_tuning_log (timestamp_ms, plugin_name, parameter, old_value, new_value, set_by)
                         VALUES (?1, ?2, ?3, ?4, ?5, ?6)",
                        params![ts, name, param, old_val, new_v, "blackboard"],
                    );
                }
            }
        }

        // Sync observed values for next tick.
        self.last_observed_gains[0].1 = (
            ctx.blackboard
                .get_f64("pid_tuning.speed_controller.kp")
                .unwrap_or(self.last_observed_gains[0].1 .0),
            ctx.blackboard
                .get_f64("pid_tuning.speed_controller.ki")
                .unwrap_or(self.last_observed_gains[0].1 .1),
            ctx.blackboard
                .get_f64("pid_tuning.speed_controller.kd")
                .unwrap_or(self.last_observed_gains[0].1 .2),
        );
        self.last_observed_gains[1].1 = (
            ctx.blackboard
                .get_f64("pid_tuning.lane_keeper.kp")
                .unwrap_or(self.last_observed_gains[1].1 .0),
            ctx.blackboard
                .get_f64("pid_tuning.lane_keeper.ki")
                .unwrap_or(self.last_observed_gains[1].1 .1),
            ctx.blackboard
                .get_f64("pid_tuning.lane_keeper.kd")
                .unwrap_or(self.last_observed_gains[1].1 .2),
        );
    }
}

fn now_iso8601() -> String {
    // Simple wall-clock timestamp without chrono dependency.
    use std::time::{SystemTime, UNIX_EPOCH};
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();
    // Format as YYYY-MM-DDTHH:MM:SSZ (approximate, no timezone conversion).
    let s = secs % 60;
    let m = (secs / 60) % 60;
    let h = (secs / 3600) % 24;
    let days = secs / 86400;
    // Days since 1970-01-01 → approximate date (good enough for logging).
    let year = 1970 + days / 365;
    let doy = days % 365;
    let month = doy / 30 + 1;
    let day = doy % 30 + 1;
    format!("{year:04}-{month:02}-{day:02}T{h:02}:{m:02}:{s:02}Z")
}

impl Plugin for StatsLoggerPlugin {
    fn name(&self) -> &str {
        "stats-logger"
    }
    fn version(&self) -> &str {
        "0.1.0"
    }
    fn settings_schema(&self) -> &str {
        r#"{
  "type": "object",
  "properties": {
    "db_path": {
      "type": "string",
      "description": "Path to the SQLite database file."
    }
  }
}"#
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        if let Some(p) = ctx.blackboard.get("stats_logger.db_path") {
            self.db_path = PathBuf::from(p);
        }
        if let Err(e) = self.open_db() {
            tracing::warn!("[stats-logger] cannot open DB: {e}");
        } else {
            tracing::info!("[stats-logger] DB opened at {:?}", self.db_path);
        }
    }

    fn on_unload(&mut self) {
        // Flush current session on unload.
        self.flush_session(0.0);
        self.session = None;
        self.conn = None;
        tracing::info!("[stats-logger] unloaded");
    }

    fn tick(
        &mut self,
        telemetry: Option<&Telemetry>,
        _output: &mut ControlOutput,
        ctx: &PluginContext,
    ) {
        let now = Instant::now();
        let dt = self
            .last_tick
            .map(|t| now.duration_since(t).as_secs_f64())
            .unwrap_or(0.02);
        self.last_tick = Some(now);

        let Some(t) = telemetry else {
            return;
        };

        // Start session on first tick with engine running.
        if self.session.is_none() && t.engine_rpm > 100.0 {
            let fuel_l = ctx
                .blackboard
                .get_f64("telemetry.fuel_liters")
                .unwrap_or(0.0);
            self.start_session(0.0, fuel_l);
        }

        if self.session.is_some() {
            let in_break = ctx.blackboard.get("break.in_break").as_deref() == Some("true");
            // Use odometer from blackboard (SHM plugin writes it).
            let odometer = ctx
                .blackboard
                .get_f64("telemetry.odometer_km")
                .unwrap_or(0.0);
            self.update_session(odometer, dt, in_break);

            let session = self.session.as_ref().unwrap();
            ctx.blackboard
                .set("stats.session_id", session.id.to_string());
            ctx.blackboard
                .set("stats.distance_km", session.distance_km.to_string());
            ctx.blackboard
                .set("stats.duration_s", session.duration_s.to_string());
        }

        // Fault detection must run before write_tick_log because detect_intervention
        // (called inside write_tick_log) updates self.last_state. write_fault_log needs
        // self.last_state = previous tick's state to detect Active→Fault transitions.
        self.write_fault_log(ctx);

        // Tick-log sampling with configurable frequency.
        let hz = ctx
            .blackboard
            .get_f64("stats_logger.tick_log_hz")
            .unwrap_or(DEFAULT_TICK_LOG_HZ);
        let skip_ticks = (50.0 / hz.max(0.1)) as u64;
        if skip_ticks == 0 || self.tick_idx.is_multiple_of(skip_ticks) {
            self.write_tick_log(t, ctx);
        }

        // PID tuning change tracking.
        self.write_pid_tuning_changes(ctx);
    }
}

truckpilot_plugin_api::export_plugin!(StatsLoggerPlugin);

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn now_iso8601_format() {
        let ts = now_iso8601();
        assert!(ts.contains('T'), "expected ISO 8601 format, got: {ts}");
        assert!(ts.ends_with('Z'), "expected UTC suffix, got: {ts}");
        assert_eq!(ts.len(), 20, "expected 20 chars, got: {ts}");
    }

    #[test]
    fn open_in_memory_db() {
        let mut p = StatsLoggerPlugin {
            db_path: PathBuf::from(":memory:"),
            ..Default::default()
        };
        p.open_db().expect("in-memory DB should open");
        assert!(p.conn.is_some());
    }

    #[test]
    fn start_and_flush_session() {
        let mut p = StatsLoggerPlugin {
            db_path: PathBuf::from(":memory:"),
            ..Default::default()
        };
        p.open_db().unwrap();
        p.start_session(0.0, 200.0);
        assert!(p.session.is_some());

        p.update_session(10.0, 3600.0, false); // 10 km in 1 hour
        p.flush_session(180.0); // 20L used

        // Verify DB row.
        let conn_guard = p.conn.as_ref().unwrap().lock().unwrap();
        let (dist, dur, avg, fuel): (f64, f64, f64, f64) = conn_guard
            .query_row(
                "SELECT distance_km, duration_s, avg_speed_kmh, fuel_used_l FROM sessions WHERE id=1",
                [],
                |r| Ok((r.get(0)?, r.get(1)?, r.get(2)?, r.get(3)?)),
            )
            .unwrap();

        assert!((dist - 10.0).abs() < 0.01);
        assert!((dur - 3600.0).abs() < 0.01);
        assert!((avg - 10.0).abs() < 0.01); // 10 km/h
        assert!((fuel - 20.0).abs() < 0.01);
    }

    #[test]
    fn pause_counter_increments_on_break_start() {
        let mut p = StatsLoggerPlugin {
            db_path: PathBuf::from(":memory:"),
            ..Default::default()
        };
        p.open_db().unwrap();
        p.start_session(0.0, 0.0);

        p.update_session(0.0, 1.0, false); // not in break
        p.update_session(0.0, 1.0, true); // break starts → +1
        p.update_session(0.0, 1.0, true); // still in break → no increment
        p.update_session(0.0, 1.0, false); // break ends
        p.update_session(0.0, 1.0, true); // second break → +1

        assert_eq!(p.session.as_ref().unwrap().pauses, 2);
    }

    #[test]
    fn delta_km_never_negative() {
        let mut p = StatsLoggerPlugin {
            db_path: PathBuf::from(":memory:"),
            ..Default::default()
        };
        p.open_db().unwrap();
        p.start_session(100.0, 0.0);
        p.last_odometer_km = 100.0;

        // Odometer goes backward (e.g. teleport) — delta must be clamped to 0.
        p.update_session(50.0, 1.0, false);
        assert_eq!(p.session.as_ref().unwrap().distance_km, 0.0);
    }
}
