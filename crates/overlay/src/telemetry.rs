//! TASK 1 — Direct SHM telemetry reader at ~60 Hz.
//!
//! NOTE: The task spec referenced `scs-sdk-telemetry = "1.2"` from crates.io.
//! TruckPilot already ships its own SHM protocol (`Local\TruckPilotTelemetry`,
//! magic 0x54504C54, version 3) implemented in `crates/telemetry/src/shm.rs`.
//! We use that directly — no external crate needed.
//! STOP condition resolved: own SHM available; `scs-sdk-telemetry` not used.
//!
//! ## head_position_world
//! Derived from truck position + cabin offset (1.5 m up, 0.5 m forward).
//! FOV calibration (Task 7) compensates for per-truck model deviations.

use std::{
    sync::{Arc, RwLock},
    time::{Duration, Instant},
};

use tracing::{debug, warn};
use truckpilot_telemetry::shm::ShmReader;

// ─── public pose snapshot ────────────────────────────────────────────────────

/// Live truck pose, updated by the background telemetry thread.
#[derive(Debug, Clone, Default)]
pub struct TruckPose {
    /// Truck reference-point position in ETS2 world space (metres).
    pub truck_x: f32,
    pub truck_y: f32,
    pub truck_z: f32,
    /// Heading in 0..1 SCS SDK units (0 = North = −Z, 0.25 = West, CCW).
    pub heading: f32,
    /// Pitch in 0..1 SCS SDK units.
    pub pitch: f32,
    /// Roll in 0..1 SCS SDK units.
    pub roll: f32,
    /// Derived cabin/head camera position (above + forward of truck ref).
    pub head_x: f32,
    pub head_y: f32,
    pub head_z: f32,
    /// Wall-clock instant of the last successful SHM read.
    pub last_update: Option<Instant>,
}

impl TruckPose {
    /// Returns true when telemetry is fresh (updated within the last second).
    pub fn is_fresh(&self) -> bool {
        self.last_update
            .map(|t| t.elapsed() < Duration::from_secs(1))
            .unwrap_or(false)
    }
}

// ─── background thread ───────────────────────────────────────────────────────

/// Spawn the 60 Hz SHM reader thread. Returns a shared handle to the latest pose.
pub fn spawn_telemetry_thread() -> Arc<RwLock<TruckPose>> {
    let pose = Arc::new(RwLock::new(TruckPose::default()));
    let pose_clone = Arc::clone(&pose);
    std::thread::Builder::new()
        .name("ar-telemetry".into())
        .spawn(move || telemetry_loop(pose_clone))
        .expect("failed to spawn ar-telemetry thread");
    pose
}

const POLL_HZ: u64 = 60;
const POLL_PERIOD: Duration = Duration::from_millis(1000 / POLL_HZ);
/// Cabin offset above truck reference point (metres).
const CABIN_Y_OFFSET: f32 = 1.5;
/// Cabin offset forward along heading direction (metres).
const CABIN_FORWARD_OFFSET: f32 = 0.5;

fn telemetry_loop(pose: Arc<RwLock<TruckPose>>) {
    loop {
        match ShmReader::open() {
            Ok(mut reader) => {
                debug!("AR telemetry: SHM opened");
                shm_read_loop(&mut reader, &pose);
                warn!("AR telemetry: SHM read loop ended, reconnecting in 2 s");
            }
            Err(e) => {
                warn!("AR telemetry: SHM unavailable ({e}), retrying in 2 s");
            }
        }
        std::thread::sleep(Duration::from_secs(2));
    }
}

fn shm_read_loop(reader: &mut ShmReader, pose: &Arc<RwLock<TruckPose>>) {
    loop {
        let t0 = Instant::now();

        if let Some(tel) = reader.read() {
            let tx = tel.position[0] as f32;
            let ty = tel.position[1] as f32;
            let tz = tel.position[2] as f32;
            let heading = tel.heading as f32;

            let (hx, hy, hz) = compute_head_pos(tx, ty, tz, heading);

            if let Ok(mut p) = pose.write() {
                p.truck_x = tx;
                p.truck_y = ty;
                p.truck_z = tz;
                p.heading = heading;
                p.pitch = tel.pitch as f32;
                p.roll = tel.roll as f32;
                p.head_x = hx;
                p.head_y = hy;
                p.head_z = hz;
                p.last_update = Some(Instant::now());
            }
        } else {
            // SHM closed or magic mismatch — exit inner loop to reopen.
            return;
        }

        let elapsed = t0.elapsed();
        if elapsed < POLL_PERIOD {
            std::thread::sleep(POLL_PERIOD - elapsed);
        }
    }
}

/// Compute cabin camera position from truck reference + heading direction.
///
/// ETS2 heading 0..1 (CCW from North, confirmed by lane-follower conversion):
///   heading=0   → North = −Z  →  forward = (−sin 0,   0, −cos 0  ) = (0,  0, −1)
///   heading=0.25 → West  = −X  →  forward = (−sin π/2, 0, −cos π/2) = (−1, 0, 0)
///   heading=0.75 → East  = +X  →  forward = (−sin 3π/2,0, −cos 3π/2)= (+1, 0, 0)
///
/// Note: forward_x = −sin(h_rad), NOT +sin(h_rad).
fn compute_head_pos(tx: f32, ty: f32, tz: f32, heading: f32) -> (f32, f32, f32) {
    let h_rad = heading * 2.0 * std::f32::consts::PI;
    let hx = tx - h_rad.sin() * CABIN_FORWARD_OFFSET; // forward_x = −sin
    let hy = ty + CABIN_Y_OFFSET;
    let hz = tz - h_rad.cos() * CABIN_FORWARD_OFFSET; // forward_z = −cos
    (hx, hy, hz)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn head_pos_north_adds_forward_negative_z() {
        // Heading=0 (North = −Z): forward offset should decrease Z
        let (hx, hy, hz) = compute_head_pos(0.0, 0.0, 0.0, 0.0);
        assert!((hx - 0.0).abs() < 1e-4, "x unchanged at heading=0");
        assert!((hy - CABIN_Y_OFFSET).abs() < 1e-4, "y = CABIN_Y_OFFSET");
        assert!(
            hz < 0.0,
            "heading=0 (North=−Z): hz should be negative, got {hz}"
        );
    }

    #[test]
    fn head_pos_east_adds_forward_positive_x() {
        // Heading=0.75 (East = +X): forward offset should increase X
        let (hx, _hy, hz) = compute_head_pos(0.0, 0.0, 0.0, 0.75);
        assert!(hx > 0.0, "heading=0.75 (East): hx should be positive, got {hx}");
        assert!(hz.abs() < 0.1, "heading=0.75 (East): hz near zero, got {hz}");
    }

    #[test]
    fn freshness_default_is_not_fresh() {
        let p = TruckPose::default();
        assert!(!p.is_fresh());
    }

    #[test]
    fn freshness_after_update_is_fresh() {
        let mut p = TruckPose::default();
        p.last_update = Some(Instant::now());
        assert!(p.is_fresh());
    }
}
