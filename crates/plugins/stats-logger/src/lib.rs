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
//! ```
//!
//! ## Blackboard contract
//!
//! | Key                      | Written by   | Read by |
//! |--------------------------|--------------|---------|
//! | `stats.session_id`       | stats-logger | UI      |
//! | `stats.distance_km`      | stats-logger | UI      |
//! | `stats.duration_s`       | stats-logger | UI      |

use std::path::PathBuf;
use std::sync::Mutex;
use std::time::Instant;

use rusqlite::{params, Connection};
use truckpilot_plugin_api::{ControlOutput, Plugin, PluginContext, Telemetry};

const DEFAULT_DB_PATH: &str = "stats.db";

/// Snapshot of a driving session.
struct Session {
    id: i64,
    #[allow(dead_code)] // stored for future DB queries / UI display
    started_at: String,
    distance_km: f64,
    duration_s: f64,
    fuel_start_l: f64,
    pauses: u32,
}

pub struct StatsLoggerPlugin {
    db_path: PathBuf,
    /// Wrapped in Mutex so the struct is Send+Sync (required by Plugin trait).
    conn: Option<Mutex<Connection>>,
    session: Option<Session>,
    last_odometer_km: f64,
    last_tick: Option<Instant>,
    /// Track break state to count pauses.
    was_in_break: bool,
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
