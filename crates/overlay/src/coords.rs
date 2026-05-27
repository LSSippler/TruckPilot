//! TASK 5 — Coordinate transformation: ETS2 world coordinates → HUD pixmap pixels.
//!
//! HUD is a 480×480 panel. Truck is always at center (240, 240).
//! Orientation: **north-up** (ETS2 −Z axis points north).
//!
//! ## ETS2 axis convention
//! - X: east (+) / west (−)
//! - Y: up (+) / down (−)  [ignored for map projection]
//! - Z: south (+) / north (−)
//!
//! So: pixel_x = center_x + (world_x − truck_x) * scale
//!     pixel_y = center_y + (world_z − truck_z) * scale   (south = down = positive pixel_y)

/// Default HUD panel dimensions.
pub const HUD_W: f32 = 480.0;
pub const HUD_H: f32 = 480.0;

/// Default pixel scale: 50m radius → 240px  ⟹  1m = 4.8 px
pub const DEFAULT_SCALE: f32 = 4.8;

/// Default center of the HUD panel (pixels).
pub const CENTER: (f32, f32) = (HUD_W / 2.0, HUD_H / 2.0);

/// Convert world coordinates to HUD-panel pixel coordinates.
///
/// * `world_x`, `world_z` — point to project (ETS2 world space)
/// * `truck_x`, `truck_z` — truck position (always maps to `center`)
/// * `scale` — pixels per meter
/// * `center` — pixel position of the truck (center of panel)
///
/// Returns `(pixel_x, pixel_y)` in panel-local coordinates.
/// Values outside `[0, HUD_W] × [0, HUD_H]` are off-screen.
#[inline]
pub fn world_to_pixel(
    world_x: f32,
    world_z: f32,
    truck_x: f32,
    truck_z: f32,
    scale: f32,
    center: (f32, f32),
) -> (f32, f32) {
    let dx = (world_x - truck_x) * scale;
    let dz = (world_z - truck_z) * scale;
    // ETS2: +Z = south = downward on map → positive pixel_y
    (center.0 + dx, center.1 + dz)
}

/// Convert panel-local pixel to absolute screen coordinates.
///
/// `panel_origin` is the top-left corner of the HUD panel in screen space.
#[inline]
#[allow(dead_code)]
pub fn panel_to_screen(pixel: (f32, f32), panel_origin: (f32, f32)) -> (f32, f32) {
    (panel_origin.0 + pixel.0, panel_origin.1 + pixel.1)
}

#[cfg(test)]
mod tests {
    use super::*;

    const S: f32 = DEFAULT_SCALE;
    const C: (f32, f32) = CENTER;

    #[test]
    fn test_world_to_pixel_centered() {
        // Truck at (100, 200), querying the truck's own position → center
        let (px, py) = world_to_pixel(100.0, 200.0, 100.0, 200.0, S, C);
        assert!((px - C.0).abs() < 1e-4, "px should be center.x, got {px}");
        assert!((py - C.1).abs() < 1e-4, "py should be center.y, got {py}");
    }

    #[test]
    fn test_world_to_pixel_offset_east() {
        // Point 10m east of truck → pixel 48px to the right of center
        let (px, py) = world_to_pixel(110.0, 200.0, 100.0, 200.0, S, C);
        let expected_x = C.0 + 10.0 * S;
        assert!(
            (px - expected_x).abs() < 1e-3,
            "10m east → +{} px, got {px}",
            10.0 * S
        );
        assert!(
            (py - C.1).abs() < 1e-3,
            "no z offset → py unchanged, got {py}"
        );
    }

    #[test]
    fn test_world_to_pixel_offset_north() {
        // Point 10m north of truck (−Z) → pixel 48px UP (negative pixel_y)
        let (px, py) = world_to_pixel(100.0, 190.0, 100.0, 200.0, S, C);
        let expected_y = C.1 + (-10.0) * S; // dz = 190 − 200 = −10 → py goes up
        assert!(
            (py - expected_y).abs() < 1e-3,
            "10m north → -{} px, got {py}",
            10.0 * S
        );
        assert!((px - C.0).abs() < 1e-3, "no x offset → px unchanged, got {px}");
    }

    #[test]
    fn test_world_to_pixel_offset_south() {
        // Point 10m south (+Z) → pixel 48px DOWN (positive pixel_y)
        let (px, py) = world_to_pixel(100.0, 210.0, 100.0, 200.0, S, C);
        let expected_y = C.1 + 10.0 * S;
        assert!(
            (py - expected_y).abs() < 1e-3,
            "10m south → +{} px, got {py}",
            10.0 * S
        );
        assert!((px - C.0).abs() < 1e-3, "no x offset → px unchanged, got {px}");
    }

    #[test]
    fn test_panel_to_screen() {
        let panel_origin = (1400.0, 10.0);
        let (sx, sy) = panel_to_screen((50.0, 75.0), panel_origin);
        assert!((sx - 1450.0).abs() < 1e-4);
        assert!((sy - 85.0).abs() < 1e-4);
    }
}
