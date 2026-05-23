//! Lane clustering: identify ego-left / ego-right lanes and compute
//! center offset normalised to [-1, +1].

use crate::postprocess::DecodedLane;

/// Minimum valid points per lane (matches Python `len(pts) >= 2`).
const MIN_POINTS: usize = 2;

/// Estimated half-lane width as a fraction of frame width, used when
/// only one bounding lane is visible.
const HALF_LANE_FRAC: f32 = 0.15;

/// Per-tick lane result published to the Blackboard.
#[derive(Debug, Clone)]
pub struct LaneResult {
    /// Lane-center offset normalised to [-1, +1].
    /// +1 = truck is at the far right of its lane (should steer left).
    /// -1 = truck is at the far left  (should steer right).
    pub center_offset_norm: f64,
    /// Mean detection confidence in [0, 1]. <0.3 = unreliable.
    pub confidence: f64,
    /// Number of valid lanes detected (0–4).
    pub detection_count: u32,
    pub left_visible: bool,
    pub right_visible: bool,
    /// Ego-left lane x in frame coordinates [-1, +1]; 0 = frame centre.
    /// NaN when no left lane is detected.
    pub left_x_norm: f64,
    /// Ego-right lane x in frame coordinates [-1, +1]; 0 = frame centre.
    /// NaN when no right lane is detected.
    pub right_x_norm: f64,
}

/// Compute a LaneResult from decoded lanes.
pub fn compute_lane_result(lanes: &[DecodedLane], orig_w: u32) -> LaneResult {
    let valid: Vec<&DecodedLane> = lanes
        .iter()
        .filter(|l| l.points.len() >= MIN_POINTS)
        .collect();

    let detection_count = valid.len() as u32;

    if detection_count == 0 {
        return LaneResult {
            center_offset_norm: 0.0,
            confidence: 0.0,
            detection_count: 0,
            left_visible: false,
            right_visible: false,
            left_x_norm: f64::NAN,
            right_x_norm: f64::NAN,
        };
    }

    let frame_cx = orig_w as f32 / 2.0;

    // Representative x: median x of the lower-third points (highest y = closest to truck).
    // Using y explicitly to select only near-truck points where offset matters most.
    let lane_xs: Vec<f32> = valid
        .iter()
        .map(|lane| {
            let max_y = lane.points.iter().map(|p| p.y).fold(0.0_f32, f32::max);
            let threshold_y = max_y * 0.67;
            let mut xs: Vec<f32> = lane
                .points
                .iter()
                .filter(|p| p.y >= threshold_y)
                .map(|p| p.x)
                .collect();
            if xs.is_empty() {
                xs = lane.points.iter().map(|p| p.x).collect();
            }
            xs.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            xs[xs.len() / 2]
        })
        .collect();

    // Ego-left: rightmost lane strictly left of frame centre.
    let ego_left_x = lane_xs
        .iter()
        .copied()
        .filter(|&x| x < frame_cx)
        .reduce(f32::max);

    // Ego-right: leftmost lane at or right of frame centre.
    let ego_right_x = lane_xs
        .iter()
        .copied()
        .filter(|&x| x >= frame_cx)
        .reduce(f32::min);

    let half_frame = orig_w as f64 / 2.0;

    let center_offset_norm = match (ego_left_x, ego_right_x) {
        (Some(lx), Some(rx)) => {
            let lane_cx = (lx + rx) / 2.0;
            // Positive offset → frame centre is to the right of lane centre
            // → truck is to the right of its ideal position.
            ((frame_cx - lane_cx) as f64 / half_frame).clamp(-1.0, 1.0)
        }
        (Some(lx), None) => {
            let est_cx = lx + orig_w as f32 * HALF_LANE_FRAC;
            ((frame_cx - est_cx) as f64 / half_frame).clamp(-1.0, 1.0)
        }
        (None, Some(rx)) => {
            let est_cx = rx - orig_w as f32 * HALF_LANE_FRAC;
            ((frame_cx - est_cx) as f64 / half_frame).clamp(-1.0, 1.0)
        }
        (None, None) => 0.0,
    };

    // Confidence: mean of per-point col_prob across all valid lanes.
    let (conf_sum, conf_n) = valid.iter().fold((0.0f64, 0usize), |(sum, n), lane| {
        let s: f32 = lane.points.iter().map(|p| p.col_prob).sum();
        (sum + s as f64, n + lane.points.len())
    });
    let confidence = if conf_n > 0 {
        (conf_sum / conf_n as f64).clamp(0.0, 1.0)
    } else {
        0.0
    };

    let half_frame_f64 = half_frame;
    LaneResult {
        center_offset_norm,
        confidence,
        detection_count,
        left_visible: ego_left_x.is_some(),
        right_visible: ego_right_x.is_some(),
        left_x_norm: ego_left_x
            .map(|x| ((x as f64 - half_frame_f64) / half_frame_f64).clamp(-1.0, 1.0))
            .unwrap_or(f64::NAN),
        right_x_norm: ego_right_x
            .map(|x| ((x as f64 - half_frame_f64) / half_frame_f64).clamp(-1.0, 1.0))
            .unwrap_or(f64::NAN),
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::postprocess::{DecodedLane, LanePoint};

    fn lane(xs: &[f32]) -> DecodedLane {
        DecodedLane {
            points: xs.iter().map(|&x| LanePoint { x, y: 100.0, col_prob: 0.8 }).collect(),
        }
    }

    #[test]
    fn no_lanes_yields_zero_offset() {
        let result = compute_lane_result(&[], 1920);
        assert_eq!(result.detection_count, 0);
        assert!((result.center_offset_norm).abs() < 1e-6);
    }

    #[test]
    fn symmetric_lanes_offset_near_zero() {
        // Left lane at 760, right lane at 1160, frame centre 960 → lane cx = 960.
        let lanes = [lane(&[760.0, 760.0, 760.0]), lane(&[1160.0, 1160.0, 1160.0])];
        let r = compute_lane_result(&lanes, 1920);
        assert!(r.left_visible);
        assert!(r.right_visible);
        assert!(r.center_offset_norm.abs() < 0.05, "offset={}", r.center_offset_norm);
    }

    #[test]
    fn truck_right_of_centre_positive_offset() {
        // Left lane at 200, right lane at 500 → lane cx = 350.
        // Frame centre = 500. frame_cx - lane_cx = 500 - 350 = 150 → positive.
        let lanes = [lane(&[200.0, 200.0, 200.0]), lane(&[500.0, 500.0, 500.0])];
        let r = compute_lane_result(&lanes, 1000);
        assert!(r.center_offset_norm > 0.0, "expected positive, got {}", r.center_offset_norm);
    }

    #[test]
    fn truck_left_of_centre_negative_offset() {
        // ego_left=400 (<500), ego_right=900 (>=500) → lane_cx=(400+900)/2=650
        // offset = (500-650)/500 = -0.3 → negative (truck left of lane centre)
        let lanes = [lane(&[400.0, 400.0, 400.0]), lane(&[900.0, 900.0, 900.0])];
        let r = compute_lane_result(&lanes, 1000);
        assert!(r.center_offset_norm < 0.0, "expected negative, got {}", r.center_offset_norm);
    }

    #[test]
    fn single_point_lane_filtered_out() {
        // Only one point → below MIN_POINTS (2).
        let short = DecodedLane {
            points: vec![LanePoint { x: 400.0, y: 100.0, col_prob: 0.9 }],
        };
        let r = compute_lane_result(&[short], 1920);
        assert_eq!(r.detection_count, 0);
    }
}
