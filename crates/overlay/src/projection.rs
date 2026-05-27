//! TASK 3 — Pinhole-camera projection: ETS2 world-space → screen pixels.
//!
//! ## Coordinate conventions
//!
//! **ETS2 world space** (left-handed, as described in coords.rs):
//!   X = East (+), Y = Up (+), Z = South (+) / North (−)
//!
//! **Camera space** (right-handed, Z = depth / forward into screen):
//!   The heading quaternion from `ets2_heading_to_quat` maps ETS2 world
//!   vectors into a camera frame where +Z is the look-at direction.
//!
//! ## Heading note
//! The spec states "+Z forward at heading=0". TruckPilot's own docs state
//! "heading=0 = North = −Z". If live testing shows AR lines are 180° off,
//! flip heading with `(heading + 0.5) % 1.0` in `ar_renderer.rs`.
//! The FOV calibration wizard (Task 7) cannot fix a heading flip.

use glam::{EulerRot, Quat, Vec3};

// ─── camera intrinsics ───────────────────────────────────────────────────────

/// Pinhole camera intrinsic matrix (simplified: fx == fy, principal point centered).
#[derive(Debug, Clone, Copy)]
pub struct CameraIntrinsics {
    /// Focal length in pixels (same for X and Y — square pixels assumed).
    pub f_x: f32,
    /// Principal point X (screen centre, pixels).
    pub cx: f32,
    /// Principal point Y (screen centre, pixels).
    pub cy: f32,
}

impl CameraIntrinsics {
    /// Build from horizontal FOV angle and screen dimensions.
    pub fn from_hfov(fov_h_deg: f32, w: f32, h: f32) -> Self {
        let f = (w / 2.0) / (fov_h_deg.to_radians() / 2.0).tan();
        Self { f_x: f, cx: w / 2.0, cy: h / 2.0 }
    }
}

// ─── rotation helper ─────────────────────────────────────────────────────────

/// Convert SCS SDK euler orientation to a glam quaternion.
///
/// All three values are in the 0..1 range (SCS SDK unit where 1.0 = full turn).
///
/// The returned quaternion satisfies: `q * [0,0,1]` = look-at direction in world space.
///
/// ETS2 heading convention (from lane-follower: `(-heading * 360).rem_euclid(360)` = CW°):
///   heading=0   → North (−Z)   heading=0.25 → West (−X)
///   heading=0.5 → South (+Z)   heading=0.75 → East (+X)
///
/// Required Y-rotation for look-at [−sin(h), 0, −cos(h)] = R_y(π + h) * [0,0,1].
/// Derivation: sin(π+h)=−sin(h) ✓  cos(π+h)=−cos(h) ✓
///
/// Note: the original spec used `−h` which assumed heading=0 = South (+Z).
/// Corrected to `π + h` to match TruckPilot's heading=0 = North = −Z convention.
pub fn ets2_heading_to_quat(heading: f32, pitch: f32, roll: f32) -> Quat {
    use std::f32::consts::PI;
    let h = heading * 2.0 * PI;
    let p = pitch * 2.0 * PI;
    let r = roll * 2.0 * PI;
    // YXZ: apply yaw (Y) first. (π + h) maps heading=0/North to look-at −Z.
    Quat::from_euler(EulerRot::YXZ, PI + h, p, r)
}

// ─── projection ──────────────────────────────────────────────────────────────

/// Project a single world-space point onto the screen.
///
/// Returns `None` when the point is at or behind the near plane.
pub fn world_to_screen(
    world_pt: Vec3,
    cam_pos: Vec3,
    cam_rot: Quat,
    k: &CameraIntrinsics,
    near: f32,
) -> Option<(f32, f32)> {
    let p_cam = cam_rot.inverse().mul_vec3(world_pt - cam_pos);
    if p_cam.z < near {
        return None;
    }
    let px = k.f_x * (p_cam.x / p_cam.z) + k.cx;
    let py = k.cy - k.f_x * (p_cam.y / p_cam.z); // screen Y inverted vs camera Y
    Some((px, py))
}

// ─── near-plane clipping ─────────────────────────────────────────────────────

/// Clip a line segment so neither endpoint is behind the near plane.
///
/// Both input points are **already in camera space** (result of `cam_rot.inverse() * delta`).
/// Returns `None` when the whole segment is behind `near`.
/// When one endpoint is behind, linearly interpolates to the near plane.
pub fn clip_segment_to_near(
    p0: Vec3,
    p1: Vec3,
    near: f32,
) -> Option<(Vec3, Vec3)> {
    let p0_behind = p0.z < near;
    let p1_behind = p1.z < near;

    match (p0_behind, p1_behind) {
        (true, true) => None,
        (false, false) => Some((p0, p1)),
        (true, false) => {
            // p0 behind → clip p0 to near plane
            let t = (near - p0.z) / (p1.z - p0.z);
            Some((p0.lerp(p1, t), p1))
        }
        (false, true) => {
            // p1 behind → clip p1 to near plane
            let t = (near - p0.z) / (p1.z - p0.z);
            Some((p0, p0.lerp(p1, t)))
        }
    }
}

// ─── tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const W: f32 = 1920.0;
    const H: f32 = 1080.0;
    const FOV: f32 = 75.0;
    const NEAR: f32 = 0.1;

    fn k() -> CameraIntrinsics {
        CameraIntrinsics::from_hfov(FOV, W, H)
    }

    /// Identity cam_rot (looking along +Z) — used for spec-matching tests.
    fn identity_cam() -> (Vec3, Quat) {
        (Vec3::ZERO, Quat::IDENTITY)
    }

    #[test]
    fn test_world_pt_directly_ahead_projects_to_center() {
        // When cam_rot = identity, "ahead" = +Z direction.
        let (cam_pos, cam_rot) = identity_cam();
        let pt = Vec3::new(0.0, 0.0, 10.0); // 10 m along +Z
        let (px, py) = world_to_screen(pt, cam_pos, cam_rot, &k(), NEAR).unwrap();
        assert!((px - W / 2.0).abs() < 1.0, "should project to cx, got {px}");
        assert!((py - H / 2.0).abs() < 1.0, "should project to cy, got {py}");
    }

    #[test]
    fn test_world_pt_above_camera_projects_above_cy() {
        let (cam_pos, cam_rot) = identity_cam();
        // Point above camera centre (+Y) at depth 10 m → pixel Y < cy (above centre)
        let pt = Vec3::new(0.0, 1.0, 10.0);
        let (_, py) = world_to_screen(pt, cam_pos, cam_rot, &k(), NEAR).unwrap();
        assert!(py < H / 2.0, "point above cam should project above cy, got py={py}");
    }

    #[test]
    fn test_world_pt_behind_camera_returns_none() {
        let (cam_pos, cam_rot) = identity_cam();
        let pt = Vec3::new(0.0, 0.0, -1.0); // behind camera (z < near)
        assert!(
            world_to_screen(pt, cam_pos, cam_rot, &k(), NEAR).is_none(),
            "point behind camera should return None"
        );
    }

    #[test]
    fn test_clip_segment_one_endpoint_behind() {
        // p0 is behind near (z=0 < 0.1), p1 is in front (z=10)
        let p0 = Vec3::new(0.0, 0.0, 0.0);
        let p1 = Vec3::new(1.0, 0.0, 10.0);
        let (a, b) = clip_segment_to_near(p0, p1, NEAR).unwrap();
        // Clipped a.z should equal near
        assert!((a.z - NEAR).abs() < 1e-5, "clipped z should be near={NEAR}, got {}", a.z);
        // b unchanged
        assert!((b - p1).length() < 1e-5, "far endpoint unchanged");
    }

    #[test]
    fn test_clip_segment_both_behind_returns_none() {
        let p0 = Vec3::new(0.0, 0.0, -5.0);
        let p1 = Vec3::new(1.0, 0.0, -1.0);
        assert!(clip_segment_to_near(p0, p1, NEAR).is_none());
    }

    #[test]
    fn test_ets2_heading_to_quat_at_0_returns_defined_rotation() {
        // Heading=0 (North = −Z): quaternion must be normalised
        let q = ets2_heading_to_quat(0.0, 0.0, 0.0);
        assert!(
            (q.length() - 1.0).abs() < 1e-5,
            "quaternion must be unit, length={}", q.length()
        );
    }

    #[test]
    fn test_heading_0_looks_north() {
        // heading=0 = North = (0, 0, −1) in ETS2 world space.
        // cam_rot * [0,0,1] should give the look-at direction = [0,0,−1].
        let q = ets2_heading_to_quat(0.0, 0.0, 0.0);
        let look_at = q.mul_vec3(glam::Vec3::Z);
        assert!(look_at.z < -0.99, "heading=0 should look North (−Z), got {look_at:?}");
        assert!(look_at.x.abs() < 0.01, "heading=0 should have no X component, got {look_at:?}");
    }

    #[test]
    fn test_heading_75_looks_east() {
        // heading=0.75 = East = (+1, 0, 0).
        let q = ets2_heading_to_quat(0.75, 0.0, 0.0);
        let look_at = q.mul_vec3(glam::Vec3::Z);
        assert!(look_at.x > 0.99, "heading=0.75 should look East (+X), got {look_at:?}");
        assert!(look_at.z.abs() < 0.01, "heading=0.75 should have no Z component");
    }

    #[test]
    fn intrinsics_principal_point_at_center() {
        let ki = CameraIntrinsics::from_hfov(90.0, 1920.0, 1080.0);
        assert!((ki.cx - 960.0).abs() < 1e-3);
        assert!((ki.cy - 540.0).abs() < 1e-3);
    }

    #[test]
    fn intrinsics_fov90_focal_length() {
        // At FOV=90°: f = (w/2) / tan(45°) = (w/2) / 1.0 = w/2
        let ki = CameraIntrinsics::from_hfov(90.0, 1920.0, 1080.0);
        assert!((ki.f_x - 960.0).abs() < 1.0, "f_x={}", ki.f_x);
    }
}
