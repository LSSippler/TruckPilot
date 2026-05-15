//! YOLOv8 output decoding, NMS, and coordinate transforms.

/// A raw detection in letterboxed-image (640×640) coordinates.
#[derive(Debug, Clone)]
pub struct Det {
    pub class_id: usize,
    pub confidence: f32,
    /// Bounding-box center X in letterboxed space (pixels).
    pub cx: f32,
    /// Bounding-box center Y in letterboxed space (pixels).
    pub cy: f32,
    pub w: f32,
    pub h: f32,
}

impl Det {
    /// Axis-aligned bounding box as (x1, y1, x2, y2) in the same coord space.
    pub fn xyxy(&self) -> (f32, f32, f32, f32) {
        let hw = self.w * 0.5;
        let hh = self.h * 0.5;
        (self.cx - hw, self.cy - hh, self.cx + hw, self.cy + hh)
    }
}

/// Decode YOLOv8 output tensor `[1, 4+nc, num_anchors]` (flat, row-major).
///
/// Memory layout: `data[feature * num_anchors + anchor]`.
/// Features 0-3: cx, cy, w, h (in letterboxed pixel coords).
/// Features 4..(4+nc): per-class confidence scores (no separate objectness).
pub fn decode_yolov8(data: &[f32], nc: usize, num_anchors: usize, conf_thresh: f32) -> Vec<Det> {
    let mut dets = Vec::new();
    for j in 0..num_anchors {
        let cx = data[j];
        let cy = data[num_anchors + j];
        let w = data[2 * num_anchors + j];
        let h = data[3 * num_anchors + j];

        let mut best_cls = 0usize;
        let mut best_conf = 0f32;
        for c in 0..nc {
            let s = data[(4 + c) * num_anchors + j];
            if s > best_conf {
                best_conf = s;
                best_cls = c;
            }
        }

        if best_conf >= conf_thresh {
            dets.push(Det {
                class_id: best_cls,
                confidence: best_conf,
                cx,
                cy,
                w,
                h,
            });
        }
    }
    dets
}

fn iou_of(a: &Det, b: &Det) -> f32 {
    let (ax1, ay1, ax2, ay2) = a.xyxy();
    let (bx1, by1, bx2, by2) = b.xyxy();
    let ix = (ax2.min(bx2) - ax1.max(bx1)).max(0.0);
    let iy = (ay2.min(by2) - ay1.max(by1)).max(0.0);
    let inter = ix * iy;
    let union = a.w * a.h + b.w * b.h - inter;
    if union <= 0.0 {
        0.0
    } else {
        inter / union
    }
}

/// Greedy class-agnostic NMS. Sorts by confidence descending, then suppresses
/// overlapping boxes with IoU > `iou_thresh`.
pub fn nms(dets: &mut [Det], iou_thresh: f32) -> Vec<Det> {
    dets.sort_unstable_by(|a, b| {
        b.confidence
            .partial_cmp(&a.confidence)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let n = dets.len();
    let mut suppressed = vec![false; n];
    let mut keep = Vec::new();
    for i in 0..n {
        if suppressed[i] {
            continue;
        }
        keep.push(dets[i].clone());
        for j in (i + 1)..n {
            if !suppressed[j] && iou_of(&dets[i], &dets[j]) > iou_thresh {
                suppressed[j] = true;
            }
        }
    }
    keep
}

/// Transform a detection from letterbox-640×640 coords back to original image coords.
pub fn unletterbox(det: &Det, scale: f32, pad_x: u32, pad_y: u32, orig_w: u32, orig_h: u32) -> Det {
    let cx = ((det.cx - pad_x as f32) / scale).clamp(0.0, orig_w as f32);
    let cy = ((det.cy - pad_y as f32) / scale).clamp(0.0, orig_h as f32);
    let w = (det.w / scale).min(orig_w as f32);
    let h = (det.h / scale).min(orig_h as f32);
    Det {
        class_id: det.class_id,
        confidence: det.confidence,
        cx,
        cy,
        w,
        h,
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_det(class_id: usize, conf: f32, cx: f32, cy: f32, w: f32, h: f32) -> Det {
        Det {
            class_id,
            confidence: conf,
            cx,
            cy,
            w,
            h,
        }
    }

    #[test]
    fn decode_empty_returns_no_dets() {
        // Minimal synthetic tensor: 1 anchor, 5 features (4 box + 1 class), all zeros.
        let data = vec![0f32; 5];
        let dets = decode_yolov8(&data, 1, 1, 0.5);
        assert!(dets.is_empty());
    }

    #[test]
    fn decode_single_anchor_above_threshold() {
        // 1 anchor, nc=2, conf_thresh=0.3
        // features: cx=10, cy=20, w=50, h=60, class0=0.1, class1=0.8
        let data = vec![10.0f32, 20.0, 50.0, 60.0, 0.1, 0.8]; // 6 = (4+2)*1
        let dets = decode_yolov8(&data, 2, 1, 0.3);
        assert_eq!(dets.len(), 1);
        let d = &dets[0];
        assert_eq!(d.class_id, 1);
        assert!((d.confidence - 0.8).abs() < 1e-5);
        assert!((d.cx - 10.0).abs() < 1e-5);
    }

    #[test]
    fn decode_below_threshold_filtered() {
        let data = vec![10.0f32, 20.0, 50.0, 60.0, 0.1, 0.2];
        let dets = decode_yolov8(&data, 2, 1, 0.5);
        assert!(dets.is_empty());
    }

    #[test]
    fn decode_multi_anchor_row_major() {
        // 2 anchors, nc=1, anchors interleaved: data[feature*2 + anchor]
        // anchor0: cx=10, cy=20, w=30, h=40, class0=0.9
        // anchor1: cx=50, cy=60, w=70, h=80, class0=0.1
        let data = vec![
            10.0f32, 50.0, // cx: anchor0, anchor1
            20.0, 60.0, // cy
            30.0, 70.0, // w
            40.0, 80.0, // h
            0.9, 0.1, // class0
        ];
        let dets = decode_yolov8(&data, 1, 2, 0.5);
        assert_eq!(dets.len(), 1);
        assert!((dets[0].cx - 10.0).abs() < 1e-5);
    }

    #[test]
    fn nms_removes_overlapping_lower_confidence() {
        // Two boxes that overlap heavily.
        let mut dets = vec![
            make_det(0, 0.9, 50.0, 50.0, 80.0, 80.0),
            make_det(0, 0.5, 55.0, 55.0, 80.0, 80.0),
        ];
        let kept = nms(&mut dets, 0.45);
        assert_eq!(kept.len(), 1);
        assert!((kept[0].confidence - 0.9).abs() < 1e-5);
    }

    #[test]
    fn nms_keeps_non_overlapping() {
        let mut dets = vec![
            make_det(0, 0.9, 50.0, 50.0, 20.0, 20.0),
            make_det(0, 0.8, 500.0, 500.0, 20.0, 20.0),
        ];
        let kept = nms(&mut dets, 0.45);
        assert_eq!(kept.len(), 2);
    }

    #[test]
    fn nms_empty_input() {
        let mut dets: Vec<Det> = Vec::new();
        let kept = nms(&mut dets, 0.45);
        assert!(kept.is_empty());
    }

    #[test]
    fn iou_identical_boxes() {
        let a = make_det(0, 1.0, 50.0, 50.0, 100.0, 100.0);
        let b = make_det(0, 0.9, 50.0, 50.0, 100.0, 100.0);
        let mut dets = vec![a, b];
        // identical → IoU=1.0 > 0.45 → suppress b
        let kept = nms(&mut dets, 0.45);
        assert_eq!(kept.len(), 1);
    }

    #[test]
    fn unletterbox_restores_center() {
        // A detection at letterbox center (320,180) with scale=0.5, pad=(0,90).
        // original coords: cx=(320-0)/0.5=640, cy=(180-90)/0.5=180
        let det = make_det(0, 0.9, 320.0, 180.0, 100.0, 100.0);
        let out = unletterbox(&det, 0.5, 0, 90, 1280, 720);
        assert!((out.cx - 640.0).abs() < 1.0, "cx={}", out.cx);
        assert!((out.cy - 180.0).abs() < 1.0, "cy={}", out.cy);
    }

    #[test]
    fn unletterbox_clamps_to_image_bounds() {
        let det = make_det(0, 0.9, 0.0, 0.0, 5000.0, 5000.0);
        let out = unletterbox(&det, 1.0, 0, 0, 640, 480);
        assert!(out.cx >= 0.0 && out.cx <= 640.0);
        assert!(out.cy >= 0.0 && out.cy <= 480.0);
        assert!(out.w <= 640.0);
        assert!(out.h <= 480.0);
    }
}
