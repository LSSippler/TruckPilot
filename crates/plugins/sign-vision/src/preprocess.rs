//! Image preprocessing: letterbox resize, NCHW normalization, crop helpers.

/// Result of letterbox-resizing an image to a square target size.
pub struct LetterboxResult {
    /// Resized + padded RGB8 pixels (target × target × 3 bytes).
    pub pixels: Vec<u8>,
    /// Scale applied to the original (original_dim * scale = resized_dim).
    pub scale: f32,
    /// Left/right padding in the letterboxed image (pixels).
    pub pad_x: u32,
    /// Top/bottom padding in the letterboxed image (pixels).
    pub pad_y: u32,
}

/// Resize `rgb` (src_w × src_h, RGB8) to a `target`×`target` square using
/// nearest-neighbour interpolation. Pads with mid-gray (114, 114, 114).
pub fn letterbox(rgb: &[u8], src_w: u32, src_h: u32, target: u32) -> LetterboxResult {
    let scale = (target as f32 / src_w as f32).min(target as f32 / src_h as f32);
    let new_w = (src_w as f32 * scale).round() as u32;
    let new_h = (src_h as f32 * scale).round() as u32;
    let pad_x = target.saturating_sub(new_w) / 2;
    let pad_y = target.saturating_sub(new_h) / 2;

    let mut out = vec![114u8; (target * target * 3) as usize];

    for py in 0..new_h {
        let sy = ((py as f32 / scale) as u32).min(src_h.saturating_sub(1));
        for px in 0..new_w {
            let sx = ((px as f32 / scale) as u32).min(src_w.saturating_sub(1));
            let src_off = ((sy * src_w + sx) * 3) as usize;
            let dst_off = (((py + pad_y) * target + (px + pad_x)) * 3) as usize;
            out[dst_off] = rgb[src_off];
            out[dst_off + 1] = rgb[src_off + 1];
            out[dst_off + 2] = rgb[src_off + 2];
        }
    }

    LetterboxResult {
        pixels: out,
        scale,
        pad_x,
        pad_y,
    }
}

/// Convert RGB8 buffer (w × h) to normalized f32 NCHW layout [1, 3, h, w] in [0, 1].
pub fn to_nchw(rgb: &[u8], w: u32, h: u32) -> Vec<f32> {
    let n = (w * h) as usize;
    let mut out = vec![0f32; 3 * n];
    for i in 0..n {
        out[i] = rgb[3 * i] as f32 / 255.0;
        out[n + i] = rgb[3 * i + 1] as f32 / 255.0;
        out[2 * n + i] = rgb[3 * i + 2] as f32 / 255.0;
    }
    out
}

/// Crop a rectangle from an RGB8 image. Returns (pixels, crop_w, crop_h).
/// Coordinates are clamped to the image bounds; returns empty if degenerate.
pub fn crop_rgb(
    src: &[u8],
    src_w: u32,
    src_h: u32,
    x1: u32,
    y1: u32,
    x2: u32,
    y2: u32,
) -> (Vec<u8>, u32, u32) {
    let x1 = x1.min(src_w);
    let y1 = y1.min(src_h);
    let x2 = x2.min(src_w);
    let y2 = y2.min(src_h);
    let cw = x2.saturating_sub(x1);
    let ch = y2.saturating_sub(y1);
    if cw == 0 || ch == 0 {
        return (Vec::new(), 0, 0);
    }
    let mut out = vec![0u8; (cw * ch * 3) as usize];
    for y in 0..ch {
        let src_row = ((y1 + y) * src_w + x1) as usize * 3;
        let dst_row = (y * cw) as usize * 3;
        out[dst_row..dst_row + (cw * 3) as usize]
            .copy_from_slice(&src[src_row..src_row + (cw * 3) as usize]);
    }
    (out, cw, ch)
}

/// Convert RGB8 pixels to grayscale using the standard luminosity weights.
pub fn rgb_to_gray(rgb: &[u8]) -> Vec<u8> {
    rgb.chunks_exact(3)
        .map(|p| (0.299 * p[0] as f32 + 0.587 * p[1] as f32 + 0.114 * p[2] as f32) as u8)
        .collect()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn letterbox_square_input_is_noop_on_scale() {
        let rgb = vec![255u8; 640 * 640 * 3];
        let lb = letterbox(&rgb, 640, 640, 640);
        assert_eq!(lb.scale, 1.0);
        assert_eq!(lb.pad_x, 0);
        assert_eq!(lb.pad_y, 0);
        assert_eq!(lb.pixels.len(), 640 * 640 * 3);
    }

    #[test]
    fn letterbox_wide_image_adds_vertical_padding() {
        // 1280×360 → 640×640: scale=0.5, new=(640,180), pad_y=(640-180)/2=230
        let rgb = vec![0u8; 1280 * 360 * 3];
        let lb = letterbox(&rgb, 1280, 360, 640);
        assert!((lb.scale - 0.5).abs() < 1e-3, "scale={}", lb.scale);
        assert!(lb.pad_y > 0, "expected top/bottom padding");
        assert_eq!(lb.pixels.len(), 640 * 640 * 3);
    }

    #[test]
    fn letterbox_tall_image_adds_horizontal_padding() {
        let rgb = vec![0u8; 320 * 640 * 3];
        let lb = letterbox(&rgb, 320, 640, 640);
        assert!((lb.scale - 1.0).abs() < 1e-3);
        assert!(lb.pad_x > 0);
    }

    #[test]
    fn letterbox_pixel_passthrough_at_scale_1() {
        // 1×1 red pixel, letterbox to 4×4 — the red pixel should appear in the center.
        let rgb = vec![200u8, 0, 0];
        let lb = letterbox(&rgb, 1, 1, 4);
        // Padded canvas is 4×4; the single resized pixel lands at (pad_x, pad_y).
        let px = lb.pad_x as usize;
        let py = lb.pad_y as usize;
        let off = (py * 4 + px) * 3;
        assert_eq!(&lb.pixels[off..off + 3], &[200, 0, 0]);
    }

    #[test]
    fn to_nchw_channel_layout() {
        // 1×1 RGB pixel [R=51, G=102, B=153]
        let rgb = vec![51u8, 102, 153];
        let out = to_nchw(&rgb, 1, 1);
        assert_eq!(out.len(), 3);
        assert!((out[0] - 51.0 / 255.0).abs() < 1e-5); // R channel
        assert!((out[1] - 102.0 / 255.0).abs() < 1e-5); // G
        assert!((out[2] - 153.0 / 255.0).abs() < 1e-5); // B
    }

    #[test]
    fn to_nchw_2x1_interleaving() {
        // 2×1 image: pixel0=[10,20,30], pixel1=[40,50,60]
        let rgb = vec![10u8, 20, 30, 40, 50, 60];
        let out = to_nchw(&rgb, 2, 1);
        // R: [10/255, 40/255], G: [20/255, 50/255], B: [30/255, 60/255]
        assert!((out[0] - 10.0 / 255.0).abs() < 1e-5);
        assert!((out[1] - 40.0 / 255.0).abs() < 1e-5);
        assert!((out[2] - 20.0 / 255.0).abs() < 1e-5);
        assert!((out[3] - 50.0 / 255.0).abs() < 1e-5);
    }

    #[test]
    fn crop_rgb_full_image() {
        let rgb = vec![1u8, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]; // 2×2
        let (crop, cw, ch) = crop_rgb(&rgb, 2, 2, 0, 0, 2, 2);
        assert_eq!((cw, ch), (2, 2));
        assert_eq!(crop, rgb);
    }

    #[test]
    fn crop_rgb_top_left_pixel() {
        let rgb = vec![1u8, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]; // 2×2
        let (crop, cw, ch) = crop_rgb(&rgb, 2, 2, 0, 0, 1, 1);
        assert_eq!((cw, ch), (1, 1));
        assert_eq!(crop, vec![1, 2, 3]);
    }

    #[test]
    fn crop_rgb_out_of_bounds_clamped() {
        let rgb = vec![1u8, 2, 3];
        let (crop, cw, ch) = crop_rgb(&rgb, 1, 1, 0, 0, 100, 100);
        assert_eq!((cw, ch), (1, 1));
        assert_eq!(crop, rgb);
    }

    #[test]
    fn rgb_to_gray_white() {
        let gray = rgb_to_gray(&[255, 255, 255]);
        assert_eq!(gray, vec![255]);
    }

    #[test]
    fn rgb_to_gray_black() {
        let gray = rgb_to_gray(&[0, 0, 0]);
        assert_eq!(gray, vec![0]);
    }

    #[test]
    fn rgb_to_gray_pure_red() {
        let gray = rgb_to_gray(&[255, 0, 0]);
        assert_eq!(gray[0], (0.299 * 255.0) as u8);
    }
}
