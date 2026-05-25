//! UFLD v2 row-anchor decoder.
//!
//! Uses two of the four model outputs:
//!  - `loc_row`   shape [1, C, R, L]: column probability logits
//!  - `exist_row` shape [1, 2, R, L]: 2-class existence logits
//!
//! Decoding mirrors the Python reference (`test_ufld.py::decode_v2`).

use crate::preprocess::LetterboxMeta;

/// One valid lane point in original-image coordinates.
#[derive(Debug, Clone)]
pub struct LanePoint {
    pub x: f32,
    pub y: f32, // retained for lane-keeper steering geometry
    /// Softmax probability of the winning column class.
    pub col_prob: f32,
}

/// Decoded lane — a sequence of LanePoints from top to bottom.
#[derive(Debug, Clone)]
pub struct DecodedLane {
    pub points: Vec<LanePoint>,
}

/// Decode UFLD v2 row-anchor output into lane point lists.
///
/// - `loc_row`  : flat C×R×L buffer (batch dim stripped), C-order.
/// - `exist_row`: flat 2×R×L buffer (batch dim stripped), C-order.
/// - `col_grids`, `row_anchors`, `num_lanes` from the probe at on_load.
/// - `exist_thresh`: minimum softmax probability of class-1 ("lane exists").
pub fn decode_ufld_v2(
    loc_row: &[f32],
    exist_row: &[f32],
    col_grids: usize,
    row_anchors: usize,
    num_lanes: usize,
    meta: &LetterboxMeta,
    exist_thresh: f32,
) -> Vec<DecodedLane> {
    // Safety: silent return on unexpected buffer sizes.
    let expected_loc = col_grids * row_anchors * num_lanes;
    let expected_exist = 2 * row_anchors * num_lanes;
    if loc_row.len() < expected_loc || exist_row.len() < expected_exist {
        return vec![];
    }

    let mut lanes: Vec<DecodedLane> = (0..num_lanes)
        .map(|_| DecodedLane { points: Vec::new() })
        .collect();

    for l in 0..num_lanes {
        for r in 0..row_anchors {
            // Existence: logits [e0, e1] for classes "no-lane", "lane"
            let e0 = exist_row[r * num_lanes + l];
            let e1 = exist_row[row_anchors * num_lanes + r * num_lanes + l];
            let max_e = e0.max(e1);
            let s0 = (e0 - max_e).exp();
            let s1 = (e1 - max_e).exp();
            let prob_exist = s1 / (s0 + s1);
            if prob_exist < exist_thresh {
                continue;
            }

            // Column: argmax over C logits at (r, l)
            let (col_idx, col_max) = (0..col_grids)
                .map(|c| (c, loc_row[c * row_anchors * num_lanes + r * num_lanes + l]))
                .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
                .unwrap_or((0, 0.0));

            // Map (col_idx, r) → model-space pixel, then unletterbox.
            let x_model = col_idx as f32 / col_grids as f32 * meta.canvas_w as f32;
            let y_model = r as f32 / row_anchors as f32 * meta.canvas_h as f32;

            let x_orig = (x_model - meta.pad_x) / meta.scale;
            let y_orig = (y_model - meta.pad_y) / meta.scale;

            lanes[l].points.push(LanePoint {
                x: x_orig.clamp(0.0, meta.orig_w as f32),
                y: y_orig.clamp(0.0, meta.orig_h as f32),
                col_prob: col_max,
            });
        }
    }

    lanes
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::preprocess::LetterboxMeta;

    fn identity_meta() -> LetterboxMeta {
        LetterboxMeta {
            scale: 1.0,
            pad_x: 0.0,
            pad_y: 0.0,
            orig_w: 1600,
            orig_h: 320,
            canvas_w: 1600,
            canvas_h: 320,
        }
    }

    #[test]
    fn all_zeros_returns_no_points() {
        let loc = vec![0.0f32; 200 * 72 * 4];
        // e0 (no-lane class, indices 0..R*L) = 10.0, e1 (lane class) = 0.0
        // → prob_exist ≈ 0 < 0.5 threshold → all rows skipped
        let mut exist = vec![0.0f32; 2 * 72 * 4];
        for v in &mut exist[0..(72 * 4)] {
            *v = 10.0;
        }
        let lanes = decode_ufld_v2(&loc, &exist, 200, 72, 4, &identity_meta(), 0.5);
        let total_points: usize = lanes.iter().map(|l| l.points.len()).sum();
        assert_eq!(total_points, 0);
    }

    #[test]
    fn exist_class1_high_yields_points() {
        // One row, one lane, C=4, all existence high for class-1
        let col_grids = 4;
        let row_anchors = 1;
        let num_lanes = 1;
        let mut loc = vec![0.0f32; col_grids * row_anchors * num_lanes];
        // argmax at col 2
        loc[2 * row_anchors * num_lanes + 0 * num_lanes + 0] = 10.0;
        let mut exist = vec![0.0f32; 2 * row_anchors * num_lanes];
        // e1 >> e0 → class-1 prob ≈ 1.0
        exist[row_anchors * num_lanes + 0 * num_lanes + 0] = 10.0;

        let lanes = decode_ufld_v2(
            &loc,
            &exist,
            col_grids,
            row_anchors,
            num_lanes,
            &identity_meta(),
            0.5,
        );
        assert_eq!(lanes.len(), 1);
        assert_eq!(lanes[0].points.len(), 1);
        // x = col_idx/C * INPUT_W = 2/4 * 1600 = 800
        let pt = &lanes[0].points[0];
        assert!((pt.x - 800.0).abs() < 1.0, "x={}", pt.x);
    }

    #[test]
    fn short_loc_row_buffer_returns_empty() {
        let meta = identity_meta();
        let lanes = decode_ufld_v2(&[], &[], 200, 72, 4, &meta, 0.5);
        assert!(lanes.is_empty());
    }

    #[test]
    fn tusimple_shape_decodes_correctly() {
        // col_grids=100, row_anchors=56, num_lanes=4 — TuSimple output shape
        let col_grids = 100;
        let row_anchors = 56;
        let num_lanes = 4;
        let mut loc = vec![0.0f32; col_grids * row_anchors * num_lanes];
        // argmax at col 50 for lane 0, row 0
        loc[50 * row_anchors * num_lanes + 0 * num_lanes + 0] = 10.0;
        let mut exist = vec![0.0f32; 2 * row_anchors * num_lanes];
        // class-1 >> class-0 for lane 0, row 0 → detected
        exist[row_anchors * num_lanes + 0 * num_lanes + 0] = 10.0;
        // class-0 high for all other (r, l) → no-lane
        for r in 0..row_anchors {
            for l in 0..num_lanes {
                if r == 0 && l == 0 {
                    continue;
                }
                exist[r * num_lanes + l] = 10.0;
            }
        }
        let meta = LetterboxMeta {
            scale: 1.0,
            pad_x: 0.0,
            pad_y: 0.0,
            orig_w: 800,
            orig_h: 320,
            canvas_w: 800,
            canvas_h: 320,
        };
        let lanes = decode_ufld_v2(&loc, &exist, col_grids, row_anchors, num_lanes, &meta, 0.5);
        // Lane 0 should have one point at col 50/100 * 800 = 400
        assert_eq!(lanes[0].points.len(), 1);
        assert!(
            (lanes[0].points[0].x - 400.0).abs() < 1.0,
            "x={}",
            lanes[0].points[0].x
        );
    }
}
