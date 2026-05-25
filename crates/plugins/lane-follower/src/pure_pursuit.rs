/// Default ETS2 truck wheelbase in metres.
pub const WHEELBASE_M: f64 = 4.0;

/// Pure-Pursuit lateral steering from truck pose to a lookahead point.
///
/// Returns a steering command in `[-1.0, +1.0]` (positive = right, ETS2 convention).
///
/// * `truck_pos`     — `(x, z)` truck position in ETS2 world coordinates.
/// * `truck_hdg_deg` — CW heading in degrees (0 = North, 90 = East).
/// * `lookahead_pos` — `(x, z)` lookahead target in world coordinates.
/// * `wheelbase_m`   — Effective wheelbase in metres (typically [`WHEELBASE_M`]).
pub fn compute_steering(
    truck_pos: (f64, f64),
    truck_hdg_deg: f64,
    lookahead_pos: (f64, f64),
    wheelbase_m: f64,
) -> f64 {
    let vx = lookahead_pos.0 - truck_pos.0;
    let vz = lookahead_pos.1 - truck_pos.1;
    let h = truck_hdg_deg.to_radians();
    // Right direction in (x,z): (cos h, sin h). Dot with V gives lateral offset.
    // y_local > 0 means lookahead is to the right.
    let y_local = vx * h.cos() + vz * h.sin();
    (2.0 * y_local / (wheelbase_m * wheelbase_m)).clamp(-1.0, 1.0)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_ahead_north_returns_zero() {
        let cmd = compute_steering((0.0, 0.0), 0.0, (0.0, -15.0), WHEELBASE_M);
        assert!(cmd.abs() < 1e-9, "straight North ahead: expected 0, got {cmd}");
    }

    #[test]
    fn straight_ahead_east_returns_zero() {
        // Truck facing East (h=90°), lookahead directly East
        let cmd = compute_steering((0.0, 0.0), 90.0, (15.0, 0.0), WHEELBASE_M);
        assert!(cmd.abs() < 1e-9, "straight East ahead: expected 0, got {cmd}");
    }

    #[test]
    fn lookahead_right_of_north_facing_truck_returns_positive() {
        // Truck North (h=0), lookahead at positive x = right
        let cmd = compute_steering((0.0, 0.0), 0.0, (5.0, -10.0), WHEELBASE_M);
        assert!(cmd > 0.0, "right lookahead must be positive, got {cmd}");
    }

    #[test]
    fn lookahead_left_of_north_facing_truck_returns_negative() {
        let cmd = compute_steering((0.0, 0.0), 0.0, (-5.0, -10.0), WHEELBASE_M);
        assert!(cmd < 0.0, "left lookahead must be negative, got {cmd}");
    }

    #[test]
    fn east_facing_truck_south_lookahead_is_right() {
        // East-facing (h=90°), South = positive z = right of truck
        let cmd = compute_steering((0.0, 0.0), 90.0, (10.0, 5.0), WHEELBASE_M);
        assert!(cmd > 0.0, "South of East-truck should be positive, got {cmd}");
    }

    #[test]
    fn east_facing_truck_north_lookahead_is_left() {
        let cmd = compute_steering((0.0, 0.0), 90.0, (10.0, -5.0), WHEELBASE_M);
        assert!(cmd < 0.0, "North of East-truck should be negative, got {cmd}");
    }

    #[test]
    fn symmetry_left_right() {
        let right = compute_steering((0.0, 0.0), 0.0, (3.0, -10.0), WHEELBASE_M);
        let left = compute_steering((0.0, 0.0), 0.0, (-3.0, -10.0), WHEELBASE_M);
        assert!((right + left).abs() < 1e-9, "symmetry: right={right}, left={left}");
        assert!(right > 0.0);
        assert!(left < 0.0);
    }

    #[test]
    fn clamps_to_plus_one() {
        let cmd = compute_steering((0.0, 0.0), 0.0, (100.0, 0.0), WHEELBASE_M);
        assert!((cmd - 1.0).abs() < 1e-9, "expected +1.0, got {cmd}");
    }

    #[test]
    fn clamps_to_minus_one() {
        let cmd = compute_steering((0.0, 0.0), 0.0, (-100.0, 0.0), WHEELBASE_M);
        assert!((cmd + 1.0).abs() < 1e-9, "expected -1.0, got {cmd}");
    }

    #[test]
    fn exact_half_wheelbase_squared_offset_returns_one() {
        // y_local = L²/2 → 2*y/L² = 1.0
        let l2_half = WHEELBASE_M * WHEELBASE_M / 2.0;
        let cmd = compute_steering((0.0, 0.0), 0.0, (l2_half, 0.0), WHEELBASE_M);
        assert!((cmd - 1.0).abs() < 1e-9, "expected 1.0, got {cmd}");
    }

    #[test]
    fn quarter_wheelbase_squared_returns_half() {
        // y_local = L²/4 → 2*y/L² = 0.5
        let l2_quarter = WHEELBASE_M * WHEELBASE_M / 4.0;
        let cmd = compute_steering((0.0, 0.0), 0.0, (l2_quarter, 0.0), WHEELBASE_M);
        assert!((cmd - 0.5).abs() < 1e-9, "expected 0.5, got {cmd}");
    }

    #[test]
    fn zero_lookahead_distance_returns_zero() {
        // Lookahead at truck position → y_local = 0
        let cmd = compute_steering((5.0, -3.0), 45.0, (5.0, -3.0), WHEELBASE_M);
        assert!(cmd.abs() < 1e-9, "zero distance: expected 0, got {cmd}");
    }

    #[test]
    fn non_origin_truck_position_right_offset() {
        // Truck at (100, -200), North, lookahead 5m right and 15m ahead
        let cmd = compute_steering((100.0, -200.0), 0.0, (105.0, -215.0), WHEELBASE_M);
        assert!(cmd > 0.0, "right offset from non-origin truck must be positive");
    }
}
