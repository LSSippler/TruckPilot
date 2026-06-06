//! READ-ONLY projection diagnostics for the AR overlay.
//!
//! Emits (once per second) the live camera pose, the camera basis vectors, a
//! handedness/mirror check and the projected screen coordinates of synthetic
//! reference points — so the on-road "diagonal down-right" symptom can be
//! pinned to a specific transform step.
//!
//! This module performs NO rendering and changes NO projection logic. It only
//! RE-computes the existing pipeline (`projection::ets2_heading_to_quat` /
//! `world_to_screen` / `CameraIntrinsics`) from already-available pose/segment
//! data and logs via `tracing` under target `"overlay::diag"`.
//!
//! ## What the lines mean (diagnosed root cause = left/right camera-X mirror)
//! * `BASIS right=…` should point to the viewer's PHYSICAL right; if it points
//!   the opposite way the lateral axis is mirrored.
//! * `MIRROR-CHECK` projects a point 5 m to the truck's physical RIGHT: a
//!   correct camera renders it with `px > cx`. `px < cx` ⇒ the X-mirror.
//! * `FWD-POINT` (15 m straight ahead) must land near `px ≈ cx`.
//! * `fwd_err` (overlay-forward minus lane-keeper-forward) must be ~0.
#![cfg(windows)]

use std::sync::Mutex;
use std::time::{Duration, Instant};

use glam::Vec3;
use tracing::info;

use crate::projection::{ets2_heading_to_quat, world_to_screen, CameraIntrinsics};
use crate::state::HudData;
use crate::telemetry::TruckPose;

/// Last emit time. The render loop calls `log_ar_frame` every frame (~60 Hz);
/// we only emit ~1 Hz to keep the console readable. `Mutex::new` is const.
static LAST_LOG: Mutex<Option<Instant>> = Mutex::new(None);
const LOG_PERIOD: Duration = Duration::from_secs(1);
/// Near plane used by the live renderer (`ar_renderer::NEAR`). Mirrored here
/// read-only so projected coords match what is actually drawn.
const NEAR: f32 = 0.1;

fn fmt(p: Vec3) -> String {
    format!("({:.3},{:.3},{:.3})", p.x, p.y, p.z)
}

fn fmt_screen(s: Option<(f32, f32)>) -> String {
    match s {
        Some((px, py)) => format!("({px:.1},{py:.1})"),
        None => "behind-near".into(),
    }
}

/// Recompute and log the projection diagnostics for the current AR frame.
///
/// Purely observational: identical inputs to `ar_renderer::render_ar`, no
/// shared mutable state, no effect on the rendered output or the autopilot.
pub fn log_ar_frame(pose: &TruckPose, data: &HudData, fov_h: f32, screen_w: f32, screen_h: f32) {
    // ── 1 Hz throttle ─────────────────────────────────────────────────────────
    {
        let mut last = match LAST_LOG.lock() {
            Ok(l) => l,
            Err(_) => return,
        };
        match *last {
            Some(t) if t.elapsed() < LOG_PERIOD => return,
            _ => *last = Some(Instant::now()),
        }
    }

    // ── Recompose the EXACT camera ar_renderer uses (read-only) ─────────────────
    let cam_pos = Vec3::new(pose.head_x, pose.head_y, pose.head_z);
    let cam_rot = ets2_heading_to_quat(pose.heading, pose.pitch, pose.roll);
    let k = CameraIntrinsics::from_hfov(fov_h, screen_w, screen_h);

    // World-space camera basis. forward = look-at, right = lateral, up = vertical.
    let fwd = cam_rot.mul_vec3(Vec3::Z);
    let right = cam_rot.mul_vec3(Vec3::X);
    let up = cam_rot.mul_vec3(Vec3::Y);
    // Chirality of the basis (+1 = right-handed). Expected +1 even WITH the bug,
    // because the quaternion itself is a proper rotation; the mirror lives in the
    // left-handed-world ↔ right-handed-camera assignment, not in the quat.
    let chirality = right.cross(up).dot(fwd);

    // Lane-keeper-convention forward (proven correct). Must match `fwd`.
    let h = pose.heading * std::f32::consts::TAU;
    let lk_fwd = Vec3::new(-h.sin(), 0.0, -h.cos());
    let fwd_err = (fwd - lk_fwd).length();

    let ground_y = pose.truck_y + 0.05;
    let truck_ground = Vec3::new(pose.truck_x, ground_y, pose.truck_z);

    // Horizontal forward / right unit vectors (ignore tiny pitch/roll tilt).
    let fwd_h = Vec3::new(fwd.x, 0.0, fwd.z).normalize_or_zero();
    // Physical-right of the viewer in ETS2 world: East when facing North.
    // right_world = (-fwd.z, 0, fwd.x)  ⇒ heading=0 → (+1,0,0)=East. ✓
    let right_world = Vec3::new(-fwd.z, 0.0, fwd.x).normalize_or_zero();

    // SMOKING GUN: a point 5 m to the truck's physical RIGHT. Correct ⇒ px > cx.
    let p_right = truck_ground + right_world * 5.0;
    let right_screen = world_to_screen(p_right, cam_pos, cam_rot, &k, NEAR);

    // Synthetic 15 m-forward ground point. Correct ⇒ px ≈ cx, py just below cy.
    let p_fwd = truck_ground + fwd_h * 15.0;
    let p_fwd_cam = cam_rot.inverse().mul_vec3(p_fwd - cam_pos);
    let fwd_screen = world_to_screen(p_fwd, cam_pos, cam_rot, &k, NEAR);

    // Nearest segment end — the value that reveals the wrong-side mirror live.
    let nearest = data
        .lane
        .nearest_seg_idx
        .and_then(|idx| data.nearby_segments.iter().find(|s| s.idx == idx))
        .or_else(|| data.nearby_segments.first());

    info!(
        target: "overlay::diag",
        "POSE heading={:.4} pitch={:.4} roll={:.4} truck=({:.1},{:.1},{:.1}) head=({:.1},{:.1},{:.1}) fov={:.1} f={:.1} cx={:.1} cy={:.1}",
        pose.heading, pose.pitch, pose.roll,
        pose.truck_x, pose.truck_y, pose.truck_z,
        pose.head_x, pose.head_y, pose.head_z,
        fov_h, k.f_x, k.cx, k.cy,
    );
    info!(
        target: "overlay::diag",
        "BASIS fwd={} right={} up={} chirality={:+.3} | lk_fwd={} fwd_err={:.4} (expect ~0)",
        fmt(fwd), fmt(right), fmt(up), chirality, fmt(lk_fwd), fwd_err,
    );
    info!(
        target: "overlay::diag",
        "MIRROR-CHECK 5m-physical-RIGHT world={} -> screen={} (EXPECT px>{:.0}; px<{:.0} => CAMERA-X MIRROR)",
        fmt(p_right), fmt_screen(right_screen), k.cx, k.cx,
    );
    info!(
        target: "overlay::diag",
        "FWD-POINT 15m-ahead world={} cam=({:.3},{:.3},{:.3}) -> screen={} (EXPECT px~{:.0}, py>{:.0})",
        fmt(p_fwd), p_fwd_cam.x, p_fwd_cam.y, p_fwd_cam.z, fmt_screen(fwd_screen), k.cx, k.cy,
    );
    match nearest {
        Some(seg) => {
            let end_world = Vec3::new(seg.end_x, ground_y, seg.end_z);
            let dir = (Vec3::new(seg.end_x, 0.0, seg.end_z)
                - Vec3::new(pose.truck_x, 0.0, pose.truck_z))
            .normalize_or_zero();
            let angle_deg = dir.angle_between(fwd_h).to_degrees();
            let end_screen = world_to_screen(end_world, cam_pos, cam_rot, &k, NEAR);
            info!(
                target: "overlay::diag",
                "NEAREST-SEG idx={} start=({:.1},{:.1}) end=({:.1},{:.1}) dir={} angle(cam_fwd,truck->end)={:.1}deg end->screen={} | lateral_signed={:.2}m",
                seg.idx, seg.start_x, seg.start_z, seg.end_x, seg.end_z,
                fmt(dir), angle_deg, fmt_screen(end_screen), data.lane.lateral_dist_signed,
            );
        }
        None => info!(target: "overlay::diag", "NEAREST-SEG none (nearby_segments empty)"),
    }
}
