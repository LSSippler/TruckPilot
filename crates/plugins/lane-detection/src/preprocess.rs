//! UFLD v2 model input normalisation.

const IMAGENET_MEAN: [f32; 3] = [0.485, 0.456, 0.406];
const IMAGENET_STD:  [f32; 3] = [0.229, 0.224, 0.225];

/// Metadata needed to map model-space coordinates back to original-image space.
#[derive(Clone)]
pub struct LetterboxMeta {
    pub scale: f32,
    pub pad_x: f32,    // pixels from the left edge of the canvas
    pub pad_y: f32,    // pixels from the top  edge of the canvas
    pub orig_w: u32,
    pub orig_h: u32,
    pub canvas_w: u32, // letterbox canvas width  (= model input W)
    pub canvas_h: u32, // letterbox canvas height (= model input H)
}

/// Preprocessed tensor + its letterbox metadata.
pub struct LetterboxResult {
    /// Flat NCHW f32 buffer, shape [1, 3, target_h, target_w].
    pub nchw: Vec<f32>,
    pub meta: LetterboxMeta,
}

/// Letterbox `rgb` (packed RGB8, row-major) to `target_w × target_h`, then
/// build the normalised NCHW tensor the model expects.
///
/// Matches the Python reference implementation exactly:
/// - black (0) padding
/// - integer-division pad offsets (floor)
/// - per-channel ImageNet normalisation
pub fn letterbox_and_normalize(
    rgb: &[u8],
    src_w: u32,
    src_h: u32,
    target_w: usize,
    target_h: usize,
) -> Result<LetterboxResult, String> {
    if rgb.len() != (src_w as usize) * (src_h as usize) * 3 {
        return Err(format!(
            "rgb buffer len mismatch: {} != {}*{}*3",
            rgb.len(), src_w, src_h
        ));
    }

    let scale_x = target_w as f32 / src_w as f32;
    let scale_y = target_h as f32 / src_h as f32;
    let scale = scale_x.min(scale_y);

    let scaled_w = (src_w as f32 * scale) as u32;
    let scaled_h = (src_h as f32 * scale) as u32;

    // Integer-division padding (matches Python `// 2`)
    let pad_x = ((target_w as u32 - scaled_w) / 2) as f32;
    let pad_y = ((target_h as u32 - scaled_h) / 2) as f32;

    let img = image::RgbImage::from_raw(src_w, src_h, rgb.to_vec())
        .ok_or("failed to wrap RGB buffer as RgbImage")?;

    let scaled = image::DynamicImage::ImageRgb8(img)
        .resize_exact(scaled_w, scaled_h, image::imageops::FilterType::Triangle)
        .to_rgb8();
    let scaled_raw = scaled.as_raw();

    // Canvas starts as all-zero (black padding)
    let mut canvas = vec![0u8; target_h * target_w * 3];
    let px = pad_x as usize;
    let py = pad_y as usize;

    for row in 0..scaled_h as usize {
        let dst_row = py + row;
        if dst_row >= target_h { break; }
        let src_off = row * scaled_w as usize * 3;
        let dst_off = (dst_row * target_w + px) * 3;
        let copy_pixels = (scaled_w as usize).min(target_w - px);
        canvas[dst_off..dst_off + copy_pixels * 3]
            .copy_from_slice(&scaled_raw[src_off..src_off + copy_pixels * 3]);
    }

    // HWC → NCHW with ImageNet normalisation
    let pixel_count = target_h * target_w;
    let mut nchw = vec![0.0f32; 3 * pixel_count];
    for i in 0..pixel_count {
        for c in 0..3usize {
            let v = canvas[i * 3 + c] as f32 / 255.0;
            nchw[c * pixel_count + i] = (v - IMAGENET_MEAN[c]) / IMAGENET_STD[c];
        }
    }

    Ok(LetterboxResult {
        nchw,
        meta: LetterboxMeta {
            scale,
            pad_x,
            pad_y,
            orig_w: src_w,
            orig_h: src_h,
            canvas_w: target_w as u32,
            canvas_h: target_h as u32,
        },
    })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn white_rgb(w: u32, h: u32) -> Vec<u8> {
        vec![255u8; (w * h * 3) as usize]
    }

    #[test]
    fn output_shape_matches_model_input() {
        let result = letterbox_and_normalize(&white_rgb(1920, 1080), 1920, 1080, 800, 320)
            .expect("letterbox ok");
        assert_eq!(result.nchw.len(), 3 * 320 * 800);
    }

    #[test]
    fn square_input_has_symmetric_padding() {
        let meta = letterbox_and_normalize(&white_rgb(320, 320), 320, 320, 800, 320)
            .expect("ok")
            .meta;
        // Scale = min(800/320, 320/320) = min(2.5, 1.0) = 1.0 (height-limited)
        assert!((meta.scale - 1.0).abs() < 0.01, "scale={}", meta.scale);
        assert_eq!(meta.pad_y as u32, 0);
        // Horizontal padding = (800 - 320*1.0) / 2 = 240
        assert_eq!(meta.pad_x as u32, 240);
    }

    #[test]
    fn wrong_buffer_length_returns_error() {
        let result = letterbox_and_normalize(&[0u8; 10], 100, 100, 800, 320);
        assert!(result.is_err());
    }
}
