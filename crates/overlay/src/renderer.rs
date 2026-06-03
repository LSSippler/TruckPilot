//! TASK 4/5/6/7/8 — Main render loop: Minimap, AR, and Both modes.
//!
//! ## Hotkeys
//! * F1  — cycle RenderMode (Minimap → AR → Both → Minimap …)
//! * F2  — toggle overlay visibility on/off
//! * F3  — enter/exit FOV calibration wizard
//! * ↑↓  — (during calibration) increase/decrease FOV by 1°
//!
//! ## Fullscreen
//! The overlay is drawn at screen coordinates [0..screen_w] × [0..screen_h].
//! Resolution is detected via Win32 GetSystemMetrics, with env var override
//! and a 1920×1080 fallback.

#![cfg(windows)]

use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};

use anyhow::Result;
use procmod_overlay::{Color, Overlay, OverlayTarget};
use tracing::{error, info, warn};

use crate::ar_renderer::{render_ar, render_calibration};
use crate::config_reader::{effective_fov, save_toml_fov};
use crate::coords::{self, HUD_H, HUD_W};
use crate::state::{HudData, HudState};
use crate::telemetry::TruckPose;

// ─── Win32 helpers (raw FFI, avoids version-pinning the `windows` crate) ─────

extern "system" {
    fn GetAsyncKeyState(vKey: i32) -> i16;
    fn GetSystemMetrics(nIndex: i32) -> i32;
}
#[link(name = "user32")]
extern "C" {}

const VK_F1: i32 = 0x70;
const VK_F2: i32 = 0x71;
const VK_F3: i32 = 0x72;
const VK_UP: i32 = 0x26;
const VK_DOWN: i32 = 0x28;
/// GetSystemMetrics index for primary monitor width.
const SM_CXSCREEN: i32 = 0;
/// GetSystemMetrics index for primary monitor height.
const SM_CYSCREEN: i32 = 1;

/// Edge-detect a key press using the high bit of GetAsyncKeyState.
///
/// The low bit ("pressed since last call") only updates for the foreground
/// input thread. Since ETS2 owns the keyboard while the overlay runs in the
/// background, `& 0x0001` never fires. The high bit (0x8000) reflects the
/// actual hardware state regardless of focus and works reliably from any
/// process. We detect a rising edge by comparing to `prev`.
///
/// Call once per frame for each key. `prev` must be initialised to `false`.
fn key_just_pressed_hb(vk: i32, prev: &mut bool) -> bool {
    let now = unsafe { GetAsyncKeyState(vk) as u16 & 0x8000 != 0 };
    let fired = now && !*prev;
    *prev = now;
    fired
}

/// Returns true if the key is currently held down (high bit set).
fn key_held(vk: i32) -> bool {
    unsafe { GetAsyncKeyState(vk) as u16 & 0x8000 != 0 }
}

// ─── RenderMode ──────────────────────────────────────────────────────────────

/// Which visualisation(s) to draw each frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RenderMode {
    Minimap,
    AR,
    Both,
}

impl RenderMode {
    pub fn cycle(self) -> Self {
        match self {
            Self::Minimap => Self::AR,
            Self::AR => Self::Both,
            Self::Both => Self::Minimap,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Minimap => "Minimap",
            Self::AR => "AR",
            Self::Both => "Both",
        }
    }
}

// ─── Constants ───────────────────────────────────────────────────────────────

const GAME_TITLE: &str = "Euro Truck Simulator 2";
const FPS: u64 = 60;
const FRAME_DURATION: Duration = Duration::from_millis(1000 / FPS);
/// How fast FOV steps while arrow keys are held (degrees per second).
const FOV_STEP_PER_SEC: f32 = 10.0;

// ─── Screen size detection ────────────────────────────────────────────────────

fn detect_screen_size() -> (f32, f32) {
    // 1) Environment variable overrides (useful for multi-monitor setups).
    let w = std::env::var("OVERLAY_SCREEN_W")
        .ok()
        .and_then(|s| s.parse().ok());
    let h = std::env::var("OVERLAY_SCREEN_H")
        .ok()
        .and_then(|s| s.parse().ok());
    if let (Some(w), Some(h)) = (w, h) {
        return (w, h);
    }

    // 2) Win32 primary monitor resolution.
    let w = unsafe { GetSystemMetrics(SM_CXSCREEN) };
    let h = unsafe { GetSystemMetrics(SM_CYSCREEN) };
    if w > 0 && h > 0 {
        return (w as f32, h as f32);
    }

    // 3) Fallback.
    (1920.0, 1080.0)
}

// ─── Entry point ─────────────────────────────────────────────────────────────

pub fn run(state: Arc<HudState>, pose: Arc<RwLock<TruckPose>>) -> Result<()> {
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

    let (screen_w, screen_h) = detect_screen_size();
    info!(
        "Screen size: {screen_w}×{screen_h}. Overlay attached. Starting render loop at {FPS} fps."
    );

    // ── Per-frame mutable state ───────────────────────────────────────────────
    let mut mode = RenderMode::AR; // default: AR for live testing
    let mut visible = true;
    let mut calibrating = false;
    let mut fov_h = effective_fov();
    let mut last_fov_key_time = Instant::now();

    // Previous-key-state for high-bit edge detection (see key_just_pressed_hb).
    let mut f1_prev = false;
    let mut f2_prev = false;
    let mut f3_prev = false;

    info!("Initial mode: {}, FOV: {fov_h:.1}°", mode.label());

    loop {
        let frame_start = Instant::now();

        // ── Hotkey polling (high-bit edge detection) ──────────────────────────
        if key_just_pressed_hb(VK_F1, &mut f1_prev) && !calibrating {
            mode = mode.cycle();
            info!("Mode toggled → {}", mode.label());
        }
        if key_just_pressed_hb(VK_F2, &mut f2_prev) {
            visible = !visible;
            info!("Visibility → {visible}");
        }
        if key_just_pressed_hb(VK_F3, &mut f3_prev) {
            calibrating = !calibrating;
            if !calibrating {
                if let Err(e) = save_toml_fov(fov_h) {
                    error!("Failed to save FOV to truckpilot.toml: {e}");
                } else {
                    info!("Saved calibrated FOV {fov_h:.1}° to truckpilot.toml");
                }
            } else {
                info!("Entering FOV calibration wizard (current FOV={fov_h:.1}°)");
            }
        }

        // FOV adjustment during calibration (continuous while held).
        if calibrating {
            let dt = last_fov_key_time.elapsed().as_secs_f32();
            if key_held(VK_UP) {
                fov_h = (fov_h + FOV_STEP_PER_SEC * dt).clamp(30.0, 150.0);
                last_fov_key_time = Instant::now();
            } else if key_held(VK_DOWN) {
                fov_h = (fov_h - FOV_STEP_PER_SEC * dt).clamp(30.0, 150.0);
                last_fov_key_time = Instant::now();
            }
        }

        // ── Visibility gate ───────────────────────────────────────────────────
        if !visible || !overlay.is_visible() {
            std::thread::sleep(Duration::from_millis(50));
            continue;
        }

        // ── Snapshot shared state ─────────────────────────────────────────────
        let data = state.snapshot();
        let connected = state.is_connected();
        let pose_snap = pose.read().map(|p| p.clone()).unwrap_or_default();

        // ── READ-ONLY projection diagnostics (1 Hz, AR/Both, fresh telemetry) ──
        // Pure observation alongside the render; no effect on drawing or autopilot.
        if !calibrating
            && matches!(mode, RenderMode::AR | RenderMode::Both)
            && pose_snap.is_fresh()
        {
            crate::diag::log_ar_frame(&pose_snap, &data, fov_h, screen_w, screen_h);
        }

        // ── Frame draw ────────────────────────────────────────────────────────
        match overlay.begin_frame() {
            Ok(_) => {}
            Err(e) => {
                error!("begin_frame failed: {e}");
                break;
            }
        }

        if calibrating {
            // Calibration wizard always draws full-screen.
            render_calibration(&mut overlay, &pose_snap, fov_h, screen_w, screen_h);
        } else {
            match mode {
                RenderMode::Minimap => {
                    let origin = minimap_origin(screen_w);
                    draw_minimap_frame(&mut overlay, &data, connected, origin);
                }
                RenderMode::AR => {
                    render_ar(
                        &mut overlay, &pose_snap, &data, fov_h, screen_w, screen_h,
                    );
                }
                RenderMode::Both => {
                    render_ar(
                        &mut overlay, &pose_snap, &data, fov_h, screen_w, screen_h,
                    );
                    let origin = minimap_origin(screen_w);
                    draw_minimap_frame(&mut overlay, &data, connected, origin);
                }
            }

            // Mode indicator badge (always visible).
            draw_mode_badge(&mut overlay, mode, screen_w);
        }

        match overlay.end_frame() {
            Ok(_) => {}
            Err(e) => {
                error!("end_frame failed: {e}");
                break;
            }
        }

        // ── Frame-rate limiter ────────────────────────────────────────────────
        let elapsed = frame_start.elapsed();
        if elapsed < FRAME_DURATION {
            std::thread::sleep(FRAME_DURATION - elapsed);
        }
    }

    Ok(())
}

// ─── Minimap helpers ──────────────────────────────────────────────────────────

fn minimap_origin(screen_w: f32) -> (f32, f32) {
    (screen_w - HUD_W - 10.0, 10.0)
}

fn draw_mode_badge(overlay: &mut Overlay, mode: RenderMode, screen_w: f32) {
    let label = format!("Mode: {} (F1)", mode.label());
    let bx = screen_w - 160.0;
    let by = HUD_H + 20.0;
    overlay.rect_filled(bx - 4.0, by - 2.0, 155.0, 22.0, Color::rgba(0, 0, 0, 180));
    overlay.text(bx, by, &label, 13.0, Color::rgba(200, 230, 200, 255));
}

// ── The minimap draw function (verbatim from old renderer.rs) ─────────────────

fn draw_minimap_frame(
    overlay: &mut Overlay,
    data: &HudData,
    connected: bool,
    origin: (f32, f32),
) {
    let ox = origin.0;
    let oy = origin.1;
    let scale = coords::DEFAULT_SCALE;
    let center = coords::CENTER;

    overlay.rect_filled(ox, oy, HUD_W, HUD_H, Color::rgba(0, 0, 0, 150));
    overlay.rect(ox, oy, HUD_W, HUD_H, Color::rgba(80, 80, 80, 200));

    let junction_r_px = 30.0 * scale;
    let zone_color = if data.bias.zone_active {
        Color::rgba(255, 140, 0, 180)
    } else {
        Color::rgba(100, 100, 100, 80)
    };
    draw_dashed_circle(overlay, ox + center.0, oy + center.1, junction_r_px, zone_color, 16);

    for seg in &data.nearby_segments {
        let (sx, sy) = coords::world_to_pixel(
            seg.start_x, seg.start_z, data.pose.x, data.pose.z, scale, center,
        );
        let (ex, ey) = coords::world_to_pixel(
            seg.end_x, seg.end_z, data.pose.x, data.pose.z, scale, center,
        );
        let range = -50.0..=(HUD_W + 50.0);
        if !range.contains(&sx) && !range.contains(&ex) {
            continue;
        }
        let is_nearest = data.lane.nearest_seg_idx == Some(seg.idx);
        let is_accepted = data.bias.prefab_accepted && seg.is_prefab && is_nearest;
        let (color, thickness) = if is_nearest {
            (Color::rgba(220, 50, 50, 255), 3.0_f32)
        } else if seg.is_prefab {
            (Color::rgba(60, 130, 220, 200), 1.5_f32)
        } else {
            (Color::rgba(150, 150, 150, 160), 1.0_f32)
        };
        overlay.line(ox + sx, oy + sy, ox + ex, oy + ey, thickness, color);
        if is_accepted {
            overlay.line(
                ox + sx, oy + sy, ox + ex, oy + ey,
                thickness + 2.0, Color::rgba(50, 200, 50, 200),
            );
        }
    }

    let heading_rad = data.pose.heading_deg.to_radians();
    let arrow_len = 12.0_f32;
    let arrow_dx = heading_rad.sin() * arrow_len;
    let arrow_dz = -heading_rad.cos() * arrow_len;
    let tx = ox + center.0;
    let ty = oy + center.1;
    overlay.line(tx, ty, tx + arrow_dx, ty + arrow_dz, 2.5, Color::rgba(255, 255, 255, 255));
    overlay.circle_filled(tx + arrow_dx, ty + arrow_dz, 2.5, Color::rgba(255, 255, 255, 255));
    overlay.circle_filled(tx, ty, 4.0, Color::rgba(0, 0, 0, 255));
    overlay.circle(tx, ty, 4.0, Color::rgba(255, 255, 255, 255));

    let tp_x = ox + 6.0;
    let tp_y = oy + 6.0;
    let tp_w = 220.0_f32;
    let tp_h = 100.0_f32;
    let line_h = 16.0_f32;
    let font_size = 13.0_f32;
    overlay.rect_filled(tp_x - 2.0, tp_y - 2.0, tp_w, tp_h, Color::rgba(0, 0, 0, 180));
    overlay.rect(tp_x - 2.0, tp_y - 2.0, tp_w, tp_h, Color::rgba(60, 60, 60, 200));

    let line1 = format!(
        "lateral={:.2}m  steer={:.3}",
        data.lane.lateral_dist_signed, data.lane.steering_filtered
    );
    overlay.text(tp_x, tp_y, &line1, font_size, Color::rgba(200, 220, 200, 255));

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

    let line5 = format!(
        "pos=({:.0},{:.0}) hdg={:.1}°",
        data.pose.x, data.pose.z, data.pose.heading_deg
    );
    overlay.text(tp_x, tp_y + line_h * 4.0, &line5, font_size, Color::rgba(160, 160, 160, 255));

    if !connected {
        let msg = "DAEMON DISCONNECTED";
        let bx = ox + 10.0;
        let by = oy + HUD_H / 2.0 - 12.0;
        overlay.rect_filled(bx - 4.0, by - 4.0, HUD_W - 20.0, 28.0, Color::rgba(180, 0, 0, 200));
        overlay.text(bx, by, msg, 16.0, Color::rgba(255, 255, 255, 255));
    }
}

fn draw_dashed_circle(
    overlay: &mut Overlay,
    cx: f32, cy: f32, radius: f32, color: Color, n_dashes: usize,
) {
    use std::f32::consts::TAU;
    let step = TAU / (n_dashes as f32 * 2.0);
    for i in 0..n_dashes {
        let t0 = i as f32 * TAU / n_dashes as f32;
        let t1 = t0 + step;
        overlay.line(
            cx + radius * t0.cos(), cy + radius * t0.sin(),
            cx + radius * t1.cos(), cy + radius * t1.sin(),
            1.0, color,
        );
    }
}

#[inline]
fn bool_char(b: bool) -> char { if b { 'T' } else { 'F' } }

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mode_toggle_cycles() {
        let m = RenderMode::Minimap;
        let m = m.cycle();
        assert_eq!(m, RenderMode::AR);
        let m = m.cycle();
        assert_eq!(m, RenderMode::Both);
        let m = m.cycle();
        assert_eq!(m, RenderMode::Minimap);
    }

    #[test]
    fn detect_screen_size_returns_positive() {
        let (w, h) = detect_screen_size();
        assert!(w > 0.0, "width must be positive, got {w}");
        assert!(h > 0.0, "height must be positive, got {h}");
    }
}
