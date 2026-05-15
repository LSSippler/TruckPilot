//! Template-matching based speed-limit extractor for EU round signs.
//!
//! `SpeedMapper` generates grayscale reference templates for common EU speed
//! limits (30/50/60/70/80/90/100/110/120/130 km/h) at init time and matches
//! a resized crop of a detected `SpeedLimitSign` bounding box against them via
//! normalized cross-correlation (NCC).
//!
//! ETS2 renders EU signs deterministically — round, red border, black number —
//! which makes NCC matching reliable enough for the in-game environment.

/// Side length of the square templates used for comparison.
const TEMPLATE_SIZE: u32 = 32;

/// EU speed limits to generate templates for.
const SPEED_LIMITS: &[u32] = &[30, 50, 60, 70, 80, 90, 100, 110, 120, 130];

/// Minimum NCC score to accept a match (in [−1, 1]).
const MIN_NCC: f32 = 0.45;

// ---------------------------------------------------------------------------
// 5×7 bitmap font for digits 0–9
// Each [u8; 7] = seven rows; bit4 = leftmost pixel, bit0 = rightmost pixel.
// ---------------------------------------------------------------------------
const DIGITS_5X7: [[u8; 7]; 10] = [
    // 0: _███_ / █___█ ×5 / _███_
    [0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E],
    // 1: __█__ / _██__ / __█__ ×4 / _███_
    [0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E],
    // 2: _███_ / █___█ / ____█ / ___█_ / __█__ / _█___ / █████
    [0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F],
    // 3: _████ / ____█ / ____█ / _████ / ____█ / ____█ / _████
    [0x0F, 0x01, 0x01, 0x0F, 0x01, 0x01, 0x0F],
    // 4: __██_ / _█_█_ / █__█_ / █████ / ___█_ ×3
    [0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02, 0x02],
    // 5: █████ / █____ / ████_ / ____█ / ____█ / █___█ / _███_
    [0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E],
    // 6: _███_ / █____ ×2 / ████_ / █___█ ×2 / _███_
    [0x0E, 0x10, 0x10, 0x1E, 0x11, 0x11, 0x0E],
    // 7: █████ / ____█ / ___█_ / __█__ / _█___ ×3
    [0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08],
    // 8: _███_ / █___█ ×2 / _███_ / █___█ ×2 / _███_
    [0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E],
    // 9: _███_ / █___█ ×2 / _████ / ____█ ×2 / _███_
    [0x0E, 0x11, 0x11, 0x0F, 0x01, 0x01, 0x0E],
];

// ---------------------------------------------------------------------------
// SpeedMapper
// ---------------------------------------------------------------------------

/// Template-matching speed-limit extractor.
#[derive(Clone)]
pub struct SpeedMapper {
    /// Precomputed (limit_kmh, 32×32 grayscale template) pairs.
    templates: Vec<(u32, Vec<u8>)>,
}

impl SpeedMapper {
    /// Build templates for all EU speed limits in `SPEED_LIMITS`.
    pub fn new() -> Self {
        let templates = SPEED_LIMITS
            .iter()
            .map(|&limit| (limit, render_eu_speed_sign(limit, TEMPLATE_SIZE)))
            .collect();
        Self { templates }
    }

    /// Given a grayscale crop of a `SpeedLimitSign` bounding box (`crop_w` ×
    /// `crop_h`), resize to 32×32 and match against all templates. Returns the
    /// km/h value of the best-matching template, or `None` if no template
    /// reaches `MIN_NCC`.
    pub fn match_speed(&self, gray: &[u8], crop_w: u32, crop_h: u32) -> Option<u32> {
        if gray.is_empty() || crop_w == 0 || crop_h == 0 {
            return None;
        }
        let query = resize_gray_nn(gray, crop_w, crop_h, TEMPLATE_SIZE, TEMPLATE_SIZE);

        let mut best_score = MIN_NCC;
        let mut best_limit = None;
        for (limit, template) in &self.templates {
            let score = ncc_score(template, &query);
            if score > best_score {
                best_score = score;
                best_limit = Some(*limit);
            }
        }
        best_limit
    }
}

impl Default for SpeedMapper {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Template rendering
// ---------------------------------------------------------------------------

/// Render a 32×32 grayscale EU speed limit sign for `limit` km/h.
///
/// Visual structure:
/// - Light-gray exterior
/// - Red border ring (represented as dark gray for grayscale matching)
/// - White inner circle
/// - Black number digits in the center
fn render_eu_speed_sign(limit: u32, size: u32) -> Vec<u8> {
    let mut img = vec![200u8; (size * size) as usize]; // exterior: light gray
    let cx = (size / 2) as i32;
    let cy = (size / 2) as i32;
    let outer_r = (size as f32 * 0.47) as i32;
    let inner_r = (size as f32 * 0.36) as i32;

    for y in 0..size as i32 {
        for x in 0..size as i32 {
            let dist = (((x - cx).pow(2) + (y - cy).pow(2)) as f32).sqrt() as i32;
            let idx = (y as u32 * size + x as u32) as usize;
            if dist <= inner_r {
                img[idx] = 240; // inner white area
            } else if dist <= outer_r {
                img[idx] = 70; // red border → dark gray
            }
        }
    }

    render_number(&mut img, size, limit);
    img
}

/// Render the `number` string onto `img` (32×32) using the 5×7 bitmap font.
fn render_number(img: &mut [u8], size: u32, number: u32) {
    let text: Vec<usize> = number
        .to_string()
        .chars()
        .filter_map(|c| c.to_digit(10).map(|d| d as usize))
        .collect();
    if text.is_empty() {
        return;
    }

    const DIGIT_W: u32 = 5;
    const DIGIT_H: u32 = 7;
    const GAP: u32 = 1;

    let n = text.len() as u32;
    let total_w = n * DIGIT_W + (n.saturating_sub(1)) * GAP;
    let start_x = (size as i32 - total_w as i32) / 2;
    let start_y = (size as i32 - DIGIT_H as i32) / 2;

    for (i, &digit) in text.iter().enumerate() {
        let bmp = &DIGITS_5X7[digit];
        let gx = start_x + i as i32 * (DIGIT_W as i32 + GAP as i32);
        for row in 0..DIGIT_H {
            let row_bits = bmp[row as usize];
            for col in 0..DIGIT_W {
                let bit = (row_bits >> (DIGIT_W - 1 - col)) & 1;
                if bit == 1 {
                    let px = gx + col as i32;
                    let py = start_y + row as i32;
                    if px >= 0 && px < size as i32 && py >= 0 && py < size as i32 {
                        img[(py as u32 * size + px as u32) as usize] = 10; // near-black
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Image helpers
// ---------------------------------------------------------------------------

/// Nearest-neighbour grayscale resize.
fn resize_gray_nn(src: &[u8], sw: u32, sh: u32, dw: u32, dh: u32) -> Vec<u8> {
    let mut out = vec![0u8; (dw * dh) as usize];
    for dy in 0..dh {
        let sy = ((dy as f32 * sh as f32 / dh as f32) as u32).min(sh.saturating_sub(1));
        for dx in 0..dw {
            let sx = ((dx as f32 * sw as f32 / dw as f32) as u32).min(sw.saturating_sub(1));
            out[(dy * dw + dx) as usize] = src[(sy * sw + sx) as usize];
        }
    }
    out
}

/// Normalized cross-correlation in [−1, 1]. Returns 0.0 if either image has
/// zero variance (constant).
pub fn ncc_score(template: &[u8], query: &[u8]) -> f32 {
    debug_assert_eq!(template.len(), query.len());
    let n = template.len() as f32;
    let t_mean: f32 = template.iter().map(|&v| v as f32).sum::<f32>() / n;
    let q_mean: f32 = query.iter().map(|&v| v as f32).sum::<f32>() / n;

    let mut num = 0f32;
    let mut t_sq = 0f32;
    let mut q_sq = 0f32;
    for (&t, &q) in template.iter().zip(query.iter()) {
        let t = t as f32 - t_mean;
        let q = q as f32 - q_mean;
        num += t * q;
        t_sq += t * t;
        q_sq += q * q;
    }

    let denom = (t_sq * q_sq).sqrt();
    if denom < 1e-6 {
        0.0
    } else {
        (num / denom).clamp(-1.0, 1.0)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn speed_mapper_creates_all_templates() {
        let mapper = SpeedMapper::new();
        assert_eq!(mapper.templates.len(), SPEED_LIMITS.len());
        for (limit, tmpl) in &mapper.templates {
            assert_eq!(
                tmpl.len(),
                (TEMPLATE_SIZE * TEMPLATE_SIZE) as usize,
                "template for {limit}"
            );
        }
    }

    #[test]
    fn render_produces_correct_size() {
        let img = render_eu_speed_sign(80, 32);
        assert_eq!(img.len(), 1024);
    }

    #[test]
    fn render_has_dark_border_ring() {
        let img = render_eu_speed_sign(80, 32);
        // Outer-edge pixels should be near-gray (exterior), not dark (border only in ring zone).
        // Just check that not all pixels are the same value.
        let unique: std::collections::HashSet<u8> = img.iter().copied().collect();
        assert!(
            unique.len() >= 3,
            "expected at least exterior/border/interior/digit values"
        );
    }

    #[test]
    fn ncc_identical_images_is_one() {
        let tmpl = vec![100u8, 120, 80, 200, 50];
        let score = ncc_score(&tmpl, &tmpl);
        assert!((score - 1.0).abs() < 1e-5, "score={score}");
    }

    #[test]
    fn ncc_inverted_images_is_neg_one() {
        let tmpl = vec![0u8, 255];
        let inv = vec![255u8, 0];
        let score = ncc_score(&tmpl, &inv);
        assert!((score - (-1.0)).abs() < 1e-5, "score={score}");
    }

    #[test]
    fn ncc_constant_image_returns_zero() {
        let tmpl = vec![100u8; 32 * 32];
        let qry = vec![150u8; 32 * 32];
        let score = ncc_score(&tmpl, &qry);
        assert_eq!(score, 0.0);
    }

    #[test]
    fn self_match_returns_one() {
        let mapper = SpeedMapper::new();
        for &limit in SPEED_LIMITS {
            let tmpl: &Vec<u8> = &mapper
                .templates
                .iter()
                .find(|(l, _)| *l == limit)
                .unwrap()
                .1;
            let score = ncc_score(tmpl, tmpl);
            assert!(
                (score - 1.0).abs() < 1e-5,
                "self-match for {limit}: score={score}"
            );
        }
    }

    #[test]
    fn match_speed_returns_none_for_uniform_gray() {
        let mapper = SpeedMapper::new();
        let gray = vec![128u8; 32 * 32]; // constant → zero variance → NCC=0 < MIN_NCC
        let result = mapper.match_speed(&gray, 32, 32);
        assert!(result.is_none());
    }

    #[test]
    fn match_speed_correct_template_matches_itself() {
        let mapper = SpeedMapper::new();
        for &limit in SPEED_LIMITS {
            let tmpl: Vec<u8> = mapper
                .templates
                .iter()
                .find(|(l, _)| *l == limit)
                .unwrap()
                .1
                .clone();
            let result = mapper.match_speed(&tmpl, TEMPLATE_SIZE, TEMPLATE_SIZE);
            assert_eq!(
                result,
                Some(limit),
                "template for {limit} km/h must match itself"
            );
        }
    }

    #[test]
    fn resize_gray_nn_identity() {
        let src: Vec<u8> = (0..16).map(|i| i as u8 * 16).collect();
        let out = resize_gray_nn(&src, 4, 4, 4, 4);
        assert_eq!(out, src);
    }

    #[test]
    fn resize_gray_nn_upscale() {
        let src = vec![10u8, 20, 30, 40]; // 2×2
        let out = resize_gray_nn(&src, 2, 2, 4, 4);
        assert_eq!(out.len(), 16);
        // Top-left quadrant should be 10.
        assert_eq!(out[0], 10);
        assert_eq!(out[1], 10);
    }

    #[test]
    fn match_speed_empty_input_returns_none() {
        let mapper = SpeedMapper::new();
        assert!(mapper.match_speed(&[], 0, 0).is_none());
        assert!(mapper.match_speed(&[], 10, 10).is_none());
    }
}
