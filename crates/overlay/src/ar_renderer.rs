//! TASK 4 — AR world-anchored renderer.
//!
//! Projects ETS2 road segments directly onto screen space using a pinhole-camera
//! model. Lines stay "glued" to the road surface — the truck drives through them.
//!
//! Rendered alongside (or instead of) the minimap via `RenderMode`.

#![cfg(windows)]

use glam::Vec3;
use procmod_overlay::{Color, Overlay};

use crate::{
    colors::{COLOR_BIAS_ACCEPTED, COLOR_DS14_TARGET, COLOR_NEAREST, COLOR_PREFAB, COLOR_ROAD},
    projection::{clip_segment_to_near, ets2_heading_to_quat, world_to_screen, CameraIntrinsics},
    state::HudData,
    telemetry::TruckPose,
};

/// Segments are raised by this many metres above the truck's Y (road level).
///
/// **Do not use a world-space constant here.** ETS2 terrain Y varies widely
/// (sea-level roads at Y≈0, mountain roads at Y≈200+). Hardcoding 0.05 places
/// all segments near world Y=0, making them project off-screen when the truck
/// is at any significant elevation. Instead we use `pose.truck_y + GROUND_OFFSET`
/// so segments are always rendered at the truck's current ground level.
const GROUND_OFFSET: f32 = 0.05;

/// Near-plane distance (metres). Segments closer than this are clipped.
const NEAR: f32 = 0.1;

/// Render all nearby road segments as AR world-anchored lines.
pub fn render_ar(
    overlay: &mut Overlay,
    pose: &TruckPose,
    data: &HudData,
    fov_h: f32,
    screen_w: f32,
    screen_h: f32,
) {
    // ── 1. Compose camera pose ───────────────────────────────────────────────
    let cam_pos = Vec3::new(pose.head_x, pose.head_y, pose.head_z);
    let cam_rot = ets2_heading_to_quat(pose.heading, pose.pitch, pose.roll);
    let k = CameraIntrinsics::from_hfov(fov_h, screen_w, screen_h);

    let nearest_idx = data.lane.nearest_seg_idx;

    // ── 2. Project all segments ──────────────────────────────────────────────
    // Use truck Y as ground reference so segments project correctly at any
    // terrain elevation. See GROUND_OFFSET doc comment above.
    let ground_y = pose.truck_y;

    for seg in &data.nearby_segments {
        // Raise segment to truck's ground level + small offset.
        let seg_y = ground_y + GROUND_OFFSET;
        let p0_world = Vec3::new(seg.start_x, seg_y, seg.start_z);
        let p1_world = Vec3::new(seg.end_x, seg_y, seg.end_z);

        // World → camera space
        let p0_cam = cam_rot.inverse().mul_vec3(p0_world - cam_pos);
        let p1_cam = cam_rot.inverse().mul_vec3(p1_world - cam_pos);

        // Clip to near plane
        let Some((p0c, p1c)) = clip_segment_to_near(p0_cam, p1_cam, NEAR) else {
            continue;
        };

        // Perspective projection
        let px0 = k.f_x * (p0c.x / p0c.z) + k.cx;
        let py0 = k.cy - k.f_x * (p0c.y / p0c.z);
        let px1 = k.f_x * (p1c.x / p1c.z) + k.cx;
        let py1 = k.cy - k.f_x * (p1c.y / p1c.z);

        // Coarse screen-bounds culling (generous margin to not clip nearby segs).
        let margin = 200.0_f32;
        let in_x = |v: f32| v > -margin && v < screen_w + margin;
        let in_y = |v: f32| v > -margin && v < screen_h + margin;
        if !in_x(px0) && !in_x(px1) {
            continue;
        }
        if !in_y(py0) && !in_y(py1) {
            continue;
        }

        // ── Choose colour ────────────────────────────────────────────────────
        let is_nearest = nearest_idx == Some(seg.idx);
        let is_accepted = data.bias.prefab_accepted && seg.is_prefab && is_nearest;

        let color: Color = if is_accepted {
            COLOR_BIAS_ACCEPTED
        } else if is_nearest {
            COLOR_NEAREST
        } else if seg.is_prefab {
            COLOR_PREFAB
        } else {
            COLOR_ROAD
        };

        // ── Distance-based line width ────────────────────────────────────────
        let avg_z = (p0c.z + p1c.z) / 2.0;
        let width = (60.0 / avg_z.max(1.0)).clamp(1.0, 6.0);

        overlay.line(px0, py0, px1, py1, width, color);

        // Extra highlight stroke for bias-accepted segments.
        if is_accepted {
            overlay.line(
                px0,
                py0,
                px1,
                py1,
                width + 2.0,
                Color::rgba(50, 230, 50, 180),
            );
        }
    }

    // ── DS14 synthetic target (only in K2-gap road_offset mode) ───────────────
    // NOTE (follow-up ticket): target Y is approximated as truck_y + GROUND_OFFSET.
    // lookahead_offset is 2D; exact ground-Y at the target is unavailable. Fine for
    // roughly flat gaps; a gap on a slope will show the marker slightly off-surface.
    if let Some((tx, tz)) = data.ds14_target_to_draw() {
        let target_y = ground_y + GROUND_OFFSET;
        let truck_ground = Vec3::new(pose.truck_x, target_y, pose.truck_z);
        let target_world = Vec3::new(tx, target_y, tz);

        // Linie Truck → Ziel: als Segment per clip_segment_to_near projizieren
        // (dieselbe robuste Behind-Near-Logik wie die Strassen-Segmente).
        let tc = cam_rot.inverse().mul_vec3(truck_ground - cam_pos);
        let gc = cam_rot.inverse().mul_vec3(target_world - cam_pos);
        if let Some((a, b)) = clip_segment_to_near(tc, gc, NEAR) {
            let ax = k.f_x * (a.x / a.z) + k.cx;
            let ay = k.cy - k.f_x * (a.y / a.z);
            let bx = k.f_x * (b.x / b.z) + k.cx;
            let by = k.cy - k.f_x * (b.y / b.z);
            overlay.line(ax, ay, bx, by, 3.0, COLOR_DS14_TARGET);
        }

        // Marker am Zielpunkt: world_to_screen gibt None zurück, wenn hinter Near.
        if let Some((px, py)) = world_to_screen(target_world, cam_pos, cam_rot, &k, NEAR) {
            overlay.circle(px, py, 8.0, COLOR_DS14_TARGET);
            overlay.circle_filled(px, py, 3.0, COLOR_DS14_TARGET);
        }
    }

    // ── 3. Status overlay text ────────────────────────────────────────────────
    let seg_count = data.nearby_segments.len();
    if !pose.is_fresh() {
        let msg = "AR-Mode: NO TELEMETRY (SHM unavailable)";
        overlay.text(10.0, 10.0, msg, 14.0, Color::rgba(255, 80, 80, 255));
    } else {
        let mode_msg = format!(
            "AR-Mode [cabin-cam] | segs: {} | Y: {:.1}m",
            seg_count, ground_y
        );
        overlay.text(10.0, 10.0, &mode_msg, 13.0, Color::rgba(200, 200, 200, 200));
    }
    if seg_count == 0 && pose.is_fresh() {
        overlay.text(
            10.0,
            30.0,
            "No segments — daemon offline or WS not connected",
            13.0,
            Color::rgba(255, 160, 40, 220),
        );
    }
}

/// Draw the F3 calibration wizard overlay.
///
/// Shows a cross at screen centre and a reference marker 15 m ahead.
/// User adjusts FOV with Up/Down arrows; text panel shows current value.
pub fn render_calibration(
    overlay: &mut Overlay,
    pose: &TruckPose,
    fov_h: f32,
    screen_w: f32,
    screen_h: f32,
) {
    let cx = screen_w / 2.0;
    let cy = screen_h / 2.0;

    // ── Cross-hair at screen centre ──────────────────────────────────────────
    const LEN: f32 = 24.0;
    overlay.line(
        cx - LEN,
        cy,
        cx + LEN,
        cy,
        1.5,
        Color::rgba(255, 255, 0, 220),
    );
    overlay.line(
        cx,
        cy - LEN,
        cx,
        cy + LEN,
        1.5,
        Color::rgba(255, 255, 0, 220),
    );
    overlay.circle(cx, cy, 6.0, Color::rgba(255, 255, 0, 200));

    // ── Reference marker: 15 m ahead at ground level ─────────────────────────
    let cam_pos = Vec3::new(pose.head_x, pose.head_y, pose.head_z);
    let cam_rot = ets2_heading_to_quat(pose.heading, pose.pitch, pose.roll);
    let k = CameraIntrinsics::from_hfov(fov_h, screen_w, screen_h);

    // "15 m ahead" in world space: truck_pos + 15 m along heading direction.
    // forward_x = −sin(h_rad), forward_z = −cos(h_rad) (heading=0=North=−Z).
    let h_rad = pose.heading * 2.0 * std::f32::consts::PI;
    let ref_world = Vec3::new(
        pose.truck_x - h_rad.sin() * 15.0,
        0.0, // ground level
        pose.truck_z - h_rad.cos() * 15.0,
    );

    if let Some((rx, ry)) = world_to_screen(ref_world, cam_pos, cam_rot, &k, 0.5) {
        overlay.circle(rx, ry, 12.0, Color::rgba(255, 120, 0, 255));
        overlay.circle(rx, ry, 4.0, Color::rgba(255, 255, 0, 255));
        overlay.line(cx, cy, rx, ry, 1.0, Color::rgba(255, 255, 0, 100));
    }

    // ── FOV info panel ───────────────────────────────────────────────────────
    let panel_x = cx - 180.0;
    let panel_y = 60.0;
    overlay.rect_filled(
        panel_x - 4.0,
        panel_y - 4.0,
        380.0,
        80.0,
        Color::rgba(0, 0, 0, 200),
    );
    overlay.rect(
        panel_x - 4.0,
        panel_y - 4.0,
        380.0,
        80.0,
        Color::rgba(255, 200, 0, 180),
    );

    overlay.text(
        panel_x,
        panel_y,
        &format!("CALIBRATION  FOV = {fov_h:.1}°"),
        16.0,
        Color::rgba(255, 230, 0, 255),
    );
    overlay.text(
        panel_x,
        panel_y + 22.0,
        "UP / DOWN: adjust FOV  |  F3: save & exit",
        13.0,
        Color::rgba(200, 200, 200, 230),
    );
    overlay.text(
        panel_x,
        panel_y + 42.0,
        "Align orange circle with road mark 15 m ahead",
        13.0,
        Color::rgba(180, 180, 255, 220),
    );
}
