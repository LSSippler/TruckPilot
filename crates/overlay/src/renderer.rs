//! TASK 4 — HUD render loop using procmod-overlay (DX11 backend).
//!
//! Runs on the main thread at ~30 fps. Reads HudState every frame.
//! Draws a 480×480 HUD panel in the top-right corner of the ETS2 window.
//!
//! ## Panel layout (480×480 px, origin = panel top-left)
//!
//!   ┌──────────────────────────────────────┐
//!   │ [text panel, top-left, ~200×100px]   │
//!   │                                      │
//!   │        [map area, center]            │
//!   │           ↑ truck arrow              │
//!   │                                      │
//!   └──────────────────────────────────────┘
//!
//! ## Coordinate systems
//!
//!   - HUD-local: origin = panel top-left, x right, y down
//!   - Screen: offset by `panel_origin` (top-right of game window)

#![cfg(windows)]

use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::Result;
use procmod_overlay::{Color, Overlay, OverlayTarget};
use tracing::{error, info, warn};

use crate::coords::{self, HUD_H, HUD_W};
use crate::state::{HudData, HudState};

/// Target game window title substring.
const GAME_TITLE: &str = "Euro Truck Simulator 2";

/// Render frame rate (fps).
const FPS: u64 = 30;
const FRAME_DURATION: Duration = Duration::from_millis(1000 / FPS);

/// Screen width used to position HUD panel at top-right.
/// Override via OVERLAY_SCREEN_W environment variable.
fn screen_width() -> f32 {
    std::env::var("OVERLAY_SCREEN_W")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(1920.0_f32)
}

/// Panel origin in screen coordinates.
fn panel_origin() -> (f32, f32) {
    let sw = screen_width();
    (sw - HUD_W - 10.0, 10.0)
}

pub fn run(state: Arc<HudState>) -> Result<()> {
    info!("Connecting overlay to window: '{GAME_TITLE}'");

    let mut overlay = loop {
        match Overlay::new(OverlayTarget::Title(GAME_TITLE.into())) {
            Ok(o) => break o,
            Err(e) => {
                warn!("Overlay init failed ({e}), retrying in 2s…");
                std::thread::sleep(Duration::from_secs(2));
            }
        }
    };

    info!("Overlay attached. Starting render loop at {FPS} fps.");

    let origin = panel_origin();
    info!(
        "HUD panel at screen ({:.0}, {:.0}), size {}×{}",
        origin.0, origin.1, HUD_W, HUD_H
    );

    loop {
        let frame_start = Instant::now();

        if !overlay.is_visible() {
            // ETS2 not in foreground — still keep running but yield
            std::thread::sleep(Duration::from_millis(50));
            continue;
        }

        let data = state.snapshot();
        let connected = state.is_connected();

        match overlay.begin_frame() {
            Ok(_) => {}
            Err(e) => {
                error!("begin_frame failed: {e}");
                break;
            }
        }

        draw_frame(&mut overlay, &data, connected, origin);

        match overlay.end_frame() {
            Ok(_) => {}
            Err(e) => {
                error!("end_frame failed: {e}");
                break;
            }
        }

        // Sleep remainder of frame budget
        let elapsed = frame_start.elapsed();
        if elapsed < FRAME_DURATION {
            std::thread::sleep(FRAME_DURATION - elapsed);
        }
    }

    Ok(())
}

fn draw_frame(
    overlay: &mut Overlay,
    data: &HudData,
    connected: bool,
    origin: (f32, f32),
) {
    let ox = origin.0;
    let oy = origin.1;
    let scale = coords::DEFAULT_SCALE;
    let center = coords::CENTER; // (240, 240) panel-local

    // ─── Background panel (dark, semi-transparent) ─────────────────────────
    overlay.rect_filled(
        ox,
        oy,
        HUD_W,
        HUD_H,
        Color::rgba(0, 0, 0, 150),
    );

    // ─── Panel border ───────────────────────────────────────────────────────
    overlay.rect(
        ox,
        oy,
        HUD_W,
        HUD_H,
        Color::rgba(80, 80, 80, 200),
    );

    // ─── Junction zone circle (30m radius) ─────────────────────────────────
    let junction_r_px = 30.0 * scale;
    let zone_color = if data.bias.zone_active {
        Color::rgba(255, 140, 0, 180) // orange
    } else {
        Color::rgba(100, 100, 100, 80) // dim grey
    };
    // Dashed circle approximation: draw 16 small arcs (short lines on radius)
    draw_dashed_circle(
        overlay,
        ox + center.0,
        oy + center.1,
        junction_r_px,
        zone_color,
        16,
    );

    // ─── Road & Prefab segments ─────────────────────────────────────────────
    for seg in &data.nearby_segments {
        let (sx, sy) = coords::world_to_pixel(
            seg.start_x, seg.start_z,
            data.pose.x, data.pose.z,
            scale, center,
        );
        let (ex, ey) = coords::world_to_pixel(
            seg.end_x, seg.end_z,
            data.pose.x, data.pose.z,
            scale, center,
        );

        // Skip if both endpoints are far off-panel
        let range = -50.0..=(HUD_W + 50.0);
        if !range.contains(&sx) && !range.contains(&ex) {
            continue;
        }

        let is_nearest = data.lane.nearest_seg_idx == Some(seg.idx);
        let is_accepted_bias = data.bias.prefab_accepted && seg.is_prefab && is_nearest;

        let (color, thickness) = if is_nearest {
            (Color::rgba(220, 50, 50, 255), 3.0_f32) // red — nearest
        } else if seg.is_prefab {
            (Color::rgba(60, 130, 220, 200), 1.5_f32) // blue — prefab
        } else {
            (Color::rgba(150, 150, 150, 160), 1.0_f32) // grey — road
        };

        overlay.line(
            ox + sx, oy + sy,
            ox + ex, oy + ey,
            thickness,
            color,
        );

        // Green outline for bias-accepted prefab
        if is_accepted_bias {
            overlay.line(
                ox + sx, oy + sy,
                ox + ex, oy + ey,
                thickness + 2.0,
                Color::rgba(50, 200, 50, 200),
            );
        }
    }

    // ─── Truck arrow ────────────────────────────────────────────────────────
    let heading_rad = data.pose.heading_deg.to_radians();
    let arrow_len = 12.0_f32;
    // heading_deg: 0 = North (−Z), 90 = East (+X)
    let arrow_dx = heading_rad.sin() * arrow_len;
    let arrow_dz = -heading_rad.cos() * arrow_len; // north = up = negative panel_y
    let tx = ox + center.0;
    let ty = oy + center.1;
    overlay.line(
        tx, ty,
        tx + arrow_dx, ty + arrow_dz,
        2.5,
        Color::rgba(255, 255, 255, 255),
    );
    // Arrow head: small circle at tip
    overlay.circle_filled(
        tx + arrow_dx, ty + arrow_dz,
        2.5,
        Color::rgba(255, 255, 255, 255),
    );
    // Truck center dot
    overlay.circle_filled(tx, ty, 4.0, Color::rgba(0, 0, 0, 255));
    overlay.circle(tx, ty, 4.0, Color::rgba(255, 255, 255, 255));

    // ─── Text panel (top-left corner of HUD) ────────────────────────────────
    let tp_x = ox + 6.0;
    let tp_y = oy + 6.0;
    let tp_w = 220.0_f32;
    let tp_h = 100.0_f32;
    let line_h = 16.0_f32;
    let font_size = 13.0_f32;

    // Background
    overlay.rect_filled(tp_x - 2.0, tp_y - 2.0, tp_w, tp_h, Color::rgba(0, 0, 0, 180));
    overlay.rect(tp_x - 2.0, tp_y - 2.0, tp_w, tp_h, Color::rgba(60, 60, 60, 200));

    // Line 1: lateral + steering
    let line1 = format!(
        "lateral={:.2}m  steer={:.3}",
        data.lane.lateral_dist_signed,
        data.lane.steering_filtered
    );
    overlay.text(tp_x, tp_y, &line1, font_size, Color::rgba(200, 220, 200, 255));

    // Line 2: bias status
    let line2 = format!(
        "bias: zone={} att={} acc={} {}",
        bool_char(data.bias.zone_active),
        bool_char(data.bias.prefab_attempted),
        bool_char(data.bias.prefab_accepted),
        if data.bias.rejected_reason.is_empty() {
            String::new()
        } else {
            format!("({})", &data.bias.rejected_reason)
        }
    );
    overlay.text(tp_x, tp_y + line_h, &line2, font_size, Color::rgba(200, 200, 220, 255));

    // Line 3: junction
    let dist_str = data
        .junction
        .distance_m
        .map(|d| format!("{:.1}m", d))
        .unwrap_or_else(|| "—".into());
    let line3 = format!(
        "junc: dist={} phase={}",
        dist_str,
        if data.junction.phase.is_empty() { "none" } else { &data.junction.phase }
    );
    let junc_color = if data.junction.detected {
        Color::rgba(255, 180, 60, 255)
    } else {
        Color::rgba(180, 180, 180, 255)
    };
    overlay.text(tp_x, tp_y + line_h * 2.0, &line3, font_size, junc_color);

    // Line 4: nearest segment
    let nearest_idx_str = data
        .lane
        .nearest_seg_idx
        .map(|i| i.to_string())
        .unwrap_or_else(|| "—".into());
    let line4 = format!(
        "near: idx={} pfb={} uid={}",
        nearest_idx_str,
        bool_char(data.lane.nearest_seg_is_prefab),
        data.lane.nearest_seg_ai_path_uid
    );
    overlay.text(tp_x, tp_y + line_h * 3.0, &line4, font_size, Color::rgba(180, 180, 255, 255));

    // Line 5: pose
    let line5 = format!(
        "pos=({:.0},{:.0}) hdg={:.1}°",
        data.pose.x, data.pose.z, data.pose.heading_deg
    );
    overlay.text(tp_x, tp_y + line_h * 4.0, &line5, font_size, Color::rgba(160, 160, 160, 255));

    // ─── DISCONNECTED banner ─────────────────────────────────────────────────
    if !connected {
        let msg = "DAEMON DISCONNECTED";
        let bx = ox + 10.0;
        let by = oy + HUD_H / 2.0 - 12.0;
        overlay.rect_filled(bx - 4.0, by - 4.0, HUD_W - 20.0, 28.0, Color::rgba(180, 0, 0, 200));
        overlay.text(bx, by, msg, 16.0, Color::rgba(255, 255, 255, 255));
    }
}

/// Draw a dashed circle with `n_dashes` line segments.
fn draw_dashed_circle(
    overlay: &mut Overlay,
    cx: f32,
    cy: f32,
    radius: f32,
    color: Color,
    n_dashes: usize,
) {
    use std::f32::consts::TAU;
    let step = TAU / (n_dashes as f32 * 2.0); // gap = half segment
    for i in 0..n_dashes {
        let t0 = i as f32 * TAU / n_dashes as f32;
        let t1 = t0 + step;
        let x0 = cx + radius * t0.cos();
        let y0 = cy + radius * t0.sin();
        let x1 = cx + radius * t1.cos();
        let y1 = cy + radius * t1.sin();
        overlay.line(x0, y0, x1, y1, 1.0, color);
    }
}

#[inline]
fn bool_char(b: bool) -> char {
    if b { 'T' } else { 'F' }
}
