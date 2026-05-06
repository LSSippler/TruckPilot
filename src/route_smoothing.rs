//! Catmull-Rom spline interpolation for smooth route following.
//!
//! Raw graph waypoints can be jagged (straight lines between nodes).
//! A Catmull-Rom spline creates a smooth, continuous curve through the
//! waypoints, producing natural steering angles.

/// Build a dense list of (x, z) sample points along a Catmull-Rom spline
/// passing through the given waypoints.
///
/// Each segment between two successive waypoints is subdivided into
/// `subdivisions` equally-spaced samples (so `subdivisions + 1` points
/// per segment including the start point).
///
/// Returns a flat Vec of (x, z) world coordinates.
pub fn smooth_route(waypoints: &[(f64, f64)], subdivisions: usize) -> Vec<(f64, f64)> {
    if waypoints.len() < 2 {
        return waypoints.to_vec();
    }

    let n = waypoints.len();
    let mut result = Vec::with_capacity((n - 1) * (subdivisions + 1));

    for i in 1..n {
        // Catmull-Rom uses four control points: P_{i-2}, P_{i-1}, P_i, P_{i+1}.
        // For the first segment, duplicate P0; for the last, duplicate P_{n-1}.
        let p0 = if i >= 2 {
            waypoints[i - 2]
        } else {
            waypoints[0]
        };
        let p1 = waypoints[i - 1];
        let p2 = waypoints[i];
        let p3 = if i + 1 < n {
            waypoints[i + 1]
        } else {
            waypoints[n - 1]
        };

        for j in 0..=subdivisions {
            let t = j as f64 / subdivisions as f64;
            // Skip the first point of each segment after the first (already emitted).
            if j == 0 && i > 1 {
                continue;
            }
            let pt = catmull_rom_point(p0, p1, p2, p3, t);
            result.push(pt);
        }
    }

    result
}

/// Evaluate a single point on a Catmull-Rom spline.
///
/// `p0..p3` are the four control points; `t` ∈ [0, 1] is the interpolation
/// parameter along the segment from `p1` to `p2`.
fn catmull_rom_point(
    p0: (f64, f64),
    p1: (f64, f64),
    p2: (f64, f64),
    p3: (f64, f64),
    t: f64,
) -> (f64, f64) {
    let t2 = t * t;
    let t3 = t2 * t;

    // Catmull-Rom basis matrix coefficients (uniform).
    let cx = 0.5
        * ((2.0 * p1.0)
            + (-p0.0 + p2.0) * t
            + (2.0 * p0.0 - 5.0 * p1.0 + 4.0 * p2.0 - p3.0) * t2
            + (-p0.0 + 3.0 * p1.0 - 3.0 * p2.0 + p3.0) * t3);
    let cz = 0.5
        * ((2.0 * p1.1)
            + (-p0.1 + p2.1) * t
            + (2.0 * p0.1 - 5.0 * p1.1 + 4.0 * p2.1 - p3.1) * t2
            + (-p0.1 + 3.0 * p1.1 - 3.0 * p2.1 + p3.1) * t3);

    (cx, cz)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_smooth_straight_line() {
        let wp = vec![(0.0, 0.0), (100.0, 0.0)];
        let pts = smooth_route(&wp, 2);
        // 2 waypoints, 1 segment, 2 subdivisions → 3 points.
        assert_eq!(pts.len(), 3);
        assert_eq!(pts[0], (0.0, 0.0));
        assert_eq!(pts[2], (100.0, 0.0));
    }

    #[test]
    fn test_smooth_single_point() {
        let wp = vec![(5.0, 3.0)];
        let pts = smooth_route(&wp, 4);
        assert_eq!(pts.len(), 1);
        assert_eq!(pts[0], (5.0, 3.0));
    }

    #[test]
    fn test_smooth_empty() {
        let pts = smooth_route(&[], 2);
        assert!(pts.is_empty());
    }

    #[test]
    fn test_catmull_rom_straight() {
        // Four collinear points → output should be on the line.
        let pt = catmull_rom_point((0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0), 0.5);
        assert!((pt.0 - 1.5).abs() < 0.01);
        assert!((pt.1 - 0.0).abs() < 0.01);
    }

    #[test]
    fn test_catmull_rom_endpoints() {
        // t=0 should match p1, t=1 should match p2.
        let p0 = (-1.0, -1.0);
        let p1 = (0.0, 0.0);
        let p2 = (2.0, 2.0);
        let p3 = (3.0, 3.0);

        let pt0 = catmull_rom_point(p0, p1, p2, p3, 0.0);
        assert!((pt0.0 - p1.0).abs() < 0.0001, "t=0 should be p1");
        assert!((pt0.1 - p1.1).abs() < 0.0001, "t=0 should be p1");

        let pt1 = catmull_rom_point(p0, p1, p2, p3, 1.0);
        assert!((pt1.0 - p2.0).abs() < 0.0001, "t=1 should be p2");
        assert!((pt1.1 - p2.1).abs() < 0.0001, "t=1 should be p2");
    }
}
