//! Phase 0c.2 — Arc-Length Parametrisierung für Hermite-Splines
//!
//! Ermöglicht gleichmäßige Bogen-Längen-Queries auf Hermite-Kurven:
//! - [`ArcLengthLUT`] — kompakte Lookup-Table (36 Bytes/Segment)
//! - [`build_lut`] — Gauss-Legendre 8-Punkt Quadratur
//! - [`arc_length`] — Bogen-Länge bei t ∈ [0,1]
//! - [`t_at_arc_length`] — Invers-Query via Binary-Search + Newton
//! - [`point_at_arc_length`] — Punkt auf Kurve bei Bogen-Abstand s
//! - [`lookahead`] — Cross-Segment-Lookahead für Pure-Pursuit
//!
//! Spec: DS-Arc-Length-Spec §1–4 (Phase 0c.2).
//! `HermiteSegment.length_m` bleibt Chord-Länge. Arc-Length nur in LUT.

use std::collections::HashMap;

use crate::spline::{evaluate, evaluate_tangent, HermiteSegment, Vec3};

// ---------------------------------------------------------------------------
// Gauss-Legendre 8-Punkt Quadratur
// ---------------------------------------------------------------------------
// Knoten und Gewichte auf [-1, 1] (Standard-Literaturwerte, 15+ signifikante Stellen)

/// GL8-Knoten auf [-1, 1] (positiv + negativ paarweise).
// Literaturwerte mit 15+ Stellen — f32 rundet auf 7 Stellen, allow nötig.
#[allow(clippy::excessive_precision)]
const GL8_NODES: [f32; 8] = [
    -0.960_289_856_497_536_3,
    -0.796_666_477_413_626_7,
    -0.525_532_409_916_329_0,
    -0.183_434_642_495_649_8,
    0.183_434_642_495_649_8,
    0.525_532_409_916_329_0,
    0.796_666_477_413_626_7,
    0.960_289_856_497_536_3,
];

/// GL8-Gewichte, korrespondierend zu [`GL8_NODES`].
#[allow(clippy::excessive_precision)]
const GL8_WEIGHTS: [f32; 8] = [
    0.101_228_536_290_376_3,
    0.222_381_034_453_374_5,
    0.313_706_645_877_887_3,
    0.362_683_783_378_362_0,
    0.362_683_783_378_362_0,
    0.313_706_645_877_887_3,
    0.222_381_034_453_374_5,
    0.101_228_536_290_376_3,
];

/// Geschwindigkeit |p'(t)| — Betrag der Hermite-Tangente (= Integrand der Bogen-Länge).
#[inline]
fn speed(seg: &HermiteSegment, t: f32) -> f32 {
    evaluate_tangent(seg, t).length()
}

/// Gauss-Legendre 8-Punkt Integration von |p'(t)| über [t0, t1].
///
/// Transformiert [-1, 1] → [t0, t1] via:
///   t = mid + half * node,  dt = half * weight
/// Fehlerordnung: O(h^16) — für kubische Hermite mehr als ausreichend.
pub fn integrate_speed(seg: &HermiteSegment, t0: f32, t1: f32) -> f32 {
    let mid = 0.5 * (t0 + t1);
    let half = 0.5 * (t1 - t0);
    let mut sum = 0.0f32;
    for i in 0..8 {
        let t = mid + half * GL8_NODES[i];
        sum += GL8_WEIGHTS[i] * speed(seg, t);
    }
    sum * half
}

// ---------------------------------------------------------------------------
// ArcLengthLUT
// ---------------------------------------------------------------------------

/// Anzahl LUT-Samples pro Segment (Breakpoints bei t = 1/N, 2/N, ..., N/N).
pub const LUT_N: usize = 16;

/// Kompakte Bogen-Längen-Tabelle für ein Hermite-Segment.
///
/// `samples[i]` = kumulierte Bogen-Länge bei t=(i+1)/N,
///   codiert als Anteil von `total_length_m` × 65535 (u16-Quantisierung).
///
/// sizeof = 16×2 + 4 = 36 Bytes.
#[derive(Debug, Clone, Copy)]
pub struct ArcLengthLUT {
    /// Kumulierte Bogen-Länge bei t=(i+1)/LUT_N, normiert auf [0, 65535].
    /// samples[LUT_N-1] = 65535 immer (t=1 = total_length).
    pub samples: [u16; LUT_N],
    /// Gesamte Bogen-Länge in Metern.
    pub total_length_m: f32,
}

/// Baut eine [`ArcLengthLUT`] für ein Hermite-Segment via GL8-Quadratur.
///
/// Für Segmente mit extrem kleiner Länge (<0.001m) wird eine degenerierte
/// LUT zurückgegeben (alle Samples linear, total=chord-Länge).
pub fn build_lut(seg: &HermiteSegment) -> ArcLengthLUT {
    // Degenerate case
    if seg.length_m < 1e-3 {
        return ArcLengthLUT {
            samples: {
                let mut s = [0u16; LUT_N];
                for (i, v) in s.iter_mut().enumerate() {
                    *v = (((i + 1) as f32 / LUT_N as f32) * 65535.0) as u16;
                }
                s
            },
            total_length_m: seg.length_m,
        };
    }

    // Berechne kumulierte Bogen-Länge an den N Breakpoints
    let step = 1.0 / LUT_N as f32;
    let mut cumulative = [0.0f32; LUT_N];
    let mut total = 0.0f32;
    for (i, slot) in cumulative.iter_mut().enumerate().take(LUT_N) {
        let t0 = i as f32 * step;
        let t1 = (i + 1) as f32 * step;
        let arc = integrate_speed(seg, t0, t1);
        total += arc;
        *slot = total;
    }

    // u16-Quantisierung: normiert auf total
    let mut samples = [0u16; LUT_N];
    for i in 0..LUT_N {
        let frac = (cumulative[i] / total).clamp(0.0, 1.0);
        samples[i] = (frac * 65535.0) as u16;
    }
    // Letzten Sample immer exakt auf 65535 setzen (Floating-Point-Robustheit)
    samples[LUT_N - 1] = 65535;

    ArcLengthLUT {
        samples,
        total_length_m: total,
    }
}

// ---------------------------------------------------------------------------
// Query-API
// ---------------------------------------------------------------------------

/// Gibt die kumulierte Bogen-Länge in Metern bei Parameter t ∈ [0,1] zurück.
///
/// Lineare Interpolation zwischen LUT-Breakpoints.
pub fn arc_length(lut: &ArcLengthLUT, t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    if t == 0.0 {
        return 0.0;
    }
    if t >= 1.0 {
        return lut.total_length_m;
    }

    let i_float = t * LUT_N as f32;
    let i_lo = (i_float as usize).min(LUT_N - 1);
    let frac = i_float - i_lo as f32;

    let lo_raw = if i_lo == 0 {
        0u16
    } else {
        lut.samples[i_lo - 1]
    };
    let hi_raw = lut.samples[i_lo];

    let lo = lo_raw as f32 / 65535.0 * lut.total_length_m;
    let hi = hi_raw as f32 / 65535.0 * lut.total_length_m;
    lo + frac * (hi - lo)
}

/// Invertiert `arc_length`: gibt t zurück sodass `arc_length(t) ≈ s`.
///
/// Algorithmus: Binary-Search im LUT → 2 Newton-Schritte.
/// Für s ≤ 0 → t=0, für s ≥ total → t=1.
pub fn t_at_arc_length(lut: &ArcLengthLUT, seg: &HermiteSegment, s: f32) -> f32 {
    if s <= 0.0 {
        return 0.0;
    }
    if s >= lut.total_length_m {
        return 1.0;
    }
    if lut.total_length_m < 1e-6 {
        return 0.0;
    }

    // u16-Suchwert
    let s_norm = (s / lut.total_length_m * 65535.0) as u16;

    // Binary-Search in samples
    let mut lo = 0usize;
    let mut hi = LUT_N;
    while lo < hi {
        let mid = (lo + hi) / 2;
        if lut.samples[mid] < s_norm {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    // lo ist der erste Index mit samples[lo] >= s_norm
    // Bracket: t ∈ [lo/LUT_N, (lo+1)/LUT_N]
    let step = 1.0 / LUT_N as f32;
    let t_lo_raw = lo as f32 * step;
    let t_hi_raw = (lo + 1).min(LUT_N) as f32 * step;

    // Lineare Interpolation als Startwert
    let s_lo = if lo == 0 {
        0.0f32
    } else {
        lut.samples[lo - 1] as f32 / 65535.0 * lut.total_length_m
    };
    let s_hi = lut.samples[lo.min(LUT_N - 1)] as f32 / 65535.0 * lut.total_length_m;
    let frac = if (s_hi - s_lo).abs() > 1e-9 {
        ((s - s_lo) / (s_hi - s_lo)).clamp(0.0, 1.0)
    } else {
        0.5
    };
    let mut t = (t_lo_raw + frac * (t_hi_raw - t_lo_raw)).clamp(0.0, 1.0);

    // Newton-Raphson: 2 Iterationen (nach Binary-Search typisch < 0.01m Restfehler)
    for _ in 0..2 {
        let arc_t = arc_length(lut, t);
        let v = speed(seg, t);
        if v < 1e-6 {
            break;
        }
        let dt = (arc_t - s) / v;
        t = (t - dt).clamp(0.0, 1.0);
    }

    t.clamp(0.0, 1.0)
}

/// Gibt den Punkt auf der Hermite-Kurve bei Bogen-Abstand s (in Metern vom Anfang) zurück.
///
/// Kombination aus [`t_at_arc_length`] + [`evaluate`].
pub fn point_at_arc_length(seg: &HermiteSegment, lut: &ArcLengthLUT, s: f32) -> Vec3 {
    let t = t_at_arc_length(lut, seg, s);
    evaluate(seg, t)
}

// ---------------------------------------------------------------------------
// Builder Pipeline
// ---------------------------------------------------------------------------

/// Baut LUTs für alle Segmente. Single-Thread (GL8 ist schnell genug).
pub fn build_all_luts(segs: &[HermiteSegment]) -> Vec<ArcLengthLUT> {
    segs.iter().map(build_lut).collect()
}

/// Baut Adjacency-Map: `from_uid → Vec<segment_idx>`.
///
/// Für Cross-Segment-Lookahead: Nachfolge-Segmente eines Segments `seg`
/// sind alle Segmente mit `from_uid == seg.to_uid`.
pub fn build_forward_adjacency(segs: &[HermiteSegment]) -> HashMap<u64, Vec<usize>> {
    let mut adj: HashMap<u64, Vec<usize>> = HashMap::new();
    for (i, seg) in segs.iter().enumerate() {
        adj.entry(seg.from_uid).or_default().push(i);
    }
    adj
}

// ---------------------------------------------------------------------------
// Cross-Segment-Lookahead
// ---------------------------------------------------------------------------

/// Ergebnis eines [`lookahead`]-Calls.
#[derive(Debug, Clone)]
pub struct LookaheadResult {
    /// Index des Segments wo der Lookahead-Punkt liegt.
    pub seg_idx: usize,
    /// Hermite-Parameter t ∈ [0,1] des Punktes.
    pub t: f32,
    /// Koordinate des Lookahead-Punktes.
    pub point: Vec3,
    /// Verbleibende Distanz die nicht mehr konsumiert wurde (nur bei Dead-End > 0).
    pub remaining_dist_m: f32,
    /// Anzahl traversierter Segment-Hops. >= [`LOOKAHEAD_MAX_HOPS`] → Iteration-Limit getriggert.
    pub iteration_count: usize,
}

/// Maximale Segmente die lookahead traversiert (schützt vor Endlos-Loops in Kreisstraßen).
pub const LOOKAHEAD_MAX_HOPS: usize = 100;

/// Dot-Produkt zweier 3D-Vektoren (für Heading-Vergleich).
#[inline]
fn dot3(a: Vec3, b: Vec3) -> f32 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

/// Fahrt-Richtungs-Konsistenz-Schwelle (cos 60° = 0.5).
/// Unter diesem Wert wird ein Kandidat-Segment abgelehnt.
const HEADING_DOT_THRESHOLD: f32 = 0.5;

/// Berechnet den Lookahead-Punkt `dist_m` Meter voraus.
///
/// Startet bei `(seg_idx, t)` und traversiert Segmente bis der Abstand
/// konsumiert ist. An Junctions: wählt den Nachfolger mit dem höchsten
/// Heading-Dot-Produkt (≥ [`HEADING_DOT_THRESHOLD`]).
///
/// Gibt immer `Some` zurück. Status-Diskriminierung über `remaining_dist_m` und
/// `iteration_count`:
/// - `remaining_dist_m == 0` → Ziel erreicht (ok)
/// - `iteration_count >= LOOKAHEAD_MAX_HOPS` → Iteration-Limit (Bug-Indikator)
/// - sonst → Dead-End (kein gültiger Nachfolger)
pub fn lookahead(
    seg_idx: usize,
    t: f32,
    dist_m: f32,
    forward_adj: &HashMap<u64, Vec<usize>>,
    segs: &[HermiteSegment],
    luts: &[ArcLengthLUT],
) -> Option<LookaheadResult> {
    let mut current_seg = seg_idx;
    let mut current_t = t.clamp(0.0, 1.0);
    let mut remaining = dist_m;
    let mut hop = 0usize;

    loop {
        let seg = &segs[current_seg];
        let lut = &luts[current_seg];

        // Arc-Länge vom aktuellen t bis zum Segment-Ende
        let arc_at_current = arc_length(lut, current_t);
        let arc_remaining_in_seg = lut.total_length_m - arc_at_current;

        if remaining <= arc_remaining_in_seg {
            // Ziel liegt in diesem Segment
            let target_arc = arc_at_current + remaining;
            let t_target = t_at_arc_length(lut, seg, target_arc);
            let point = evaluate(seg, t_target);
            return Some(LookaheadResult {
                seg_idx: current_seg,
                t: t_target,
                point,
                remaining_dist_m: 0.0,
                iteration_count: hop,
            });
        }

        // Restdistanz konsumieren und nächstes Segment suchen
        remaining -= arc_remaining_in_seg;

        // Aktuelles Heading am Ende des Segments
        let exit_tan = evaluate_tangent(seg, 1.0).normalize();

        // Nachfolge-Segmente über Adjacency
        let candidates = match forward_adj.get(&seg.to_uid) {
            Some(c) if !c.is_empty() => c,
            _ => {
                // Dead-End: kein Nachfolger
                let point = evaluate(seg, 1.0);
                return Some(LookaheadResult {
                    seg_idx: current_seg,
                    t: 1.0,
                    point,
                    remaining_dist_m: remaining,
                    iteration_count: hop,
                });
            }
        };

        // Bestes Kandidat-Segment via Heading-Dot-Produkt
        let mut best_idx = None;
        let mut best_dot = HEADING_DOT_THRESHOLD - 1e-6;

        for &cand_idx in candidates {
            if cand_idx == current_seg {
                continue; // Selbst-Referenz ignorieren
            }
            let cand_seg = &segs[cand_idx];
            // b-ii-Schutz: exaktes Reverse-Geschwister (from/to vertauscht) hart
            // überspringen — analog zum Chain-Advance (lane-keeper `advance_forward_adj`).
            // Rückwärts-Kanten tragen kopierte statt negierte Hermite-Tangenten: bei t=0
            // zeigen sie vorwärts und täuschen den Heading-Dot-Filter unten, im Segment-
            // Inneren drehen sie nach hinten. Ohne diesen Skip läuft der Lookahead auf das
            // Reverse-Geschwister → `head_c` springt ~π → Lenk-Anschlag/Crash.
            if cand_seg.from_uid == seg.to_uid && cand_seg.to_uid == seg.from_uid {
                continue;
            }
            let entry_tan = evaluate_tangent(cand_seg, 0.0).normalize();
            let d = dot3(exit_tan, entry_tan);
            if d >= HEADING_DOT_THRESHOLD && d > best_dot {
                best_dot = d;
                best_idx = Some(cand_idx);
            }
        }

        match best_idx {
            Some(next_idx) => {
                current_seg = next_idx;
                current_t = 0.0;
                hop += 1;
                if hop >= LOOKAHEAD_MAX_HOPS {
                    // Iteration-Limit: Kreis-Straße oder Adjacency-Bug
                    let point = evaluate(&segs[current_seg], 0.0);
                    return Some(LookaheadResult {
                        seg_idx: current_seg,
                        t: 0.0,
                        point,
                        remaining_dist_m: remaining,
                        iteration_count: hop,
                    });
                }
            }
            None => {
                // Kein kompatibler Nachfolger (Heading-Filter)
                let point = evaluate(seg, 1.0);
                return Some(LookaheadResult {
                    seg_idx: current_seg,
                    t: 1.0,
                    point,
                    remaining_dist_m: remaining,
                    iteration_count: hop,
                });
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Unit-Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spline::HermiteSegment;

    /// Gerades Segment von (0,0,0) nach (10,0,0) mit Catmull-Rom-Tangenten.
    /// Arc-Länge = Chord-Länge = 10m (da gerade Linie).
    fn straight_seg_10m() -> HermiteSegment {
        HermiteSegment {
            p0: Vec3::new(0.0, 0.0, 0.0),
            p1: Vec3::new(10.0, 0.0, 0.0),
            m0: Vec3::new(10.0, 0.0, 0.0),
            m1: Vec3::new(10.0, 0.0, 0.0),
            length_m: 10.0,
            from_uid: 1,
            to_uid: 2,
            edge_uid: 1,
        }
    }

    /// Hilfsfunktion: Segment mit parametrisch bekannter Arc-Länge = Chord.
    fn make_seg(x0: f32, z0: f32, x1: f32, z1: f32, from: u64, to: u64) -> HermiteSegment {
        let p0 = Vec3::new(x0, 0.0, z0);
        let p1 = Vec3::new(x1, 0.0, z1);
        let chord = p1 - p0;
        let len = chord.length();
        let m = chord.normalize() * len;
        HermiteSegment {
            p0,
            p1,
            m0: m,
            m1: m,
            length_m: len,
            from_uid: from,
            to_uid: to,
            edge_uid: from * 100 + to,
        }
    }

    // --- GL8 Quadratur ---

    #[test]
    fn gl8_straight_segment_arc_equals_chord() {
        // Gerades Segment: Arc-Länge muss exakt Chord-Länge sein.
        // HermiteSegment mit m0=m1=(10,0,0): die Kurve ist eine Gerade.
        let seg = straight_seg_10m();
        let arc = integrate_speed(&seg, 0.0, 1.0);
        assert!(
            (arc - 10.0).abs() < 0.001,
            "Gerades Segment: arc={arc:.4} erwartet 10.0"
        );
    }

    #[test]
    fn gl8_partial_interval() {
        // [0.0, 0.5] muss ≈ 5m sein für gerades Segment.
        let seg = straight_seg_10m();
        let arc_half = integrate_speed(&seg, 0.0, 0.5);
        assert!(
            (arc_half - 5.0).abs() < 0.01,
            "Halbes Segment: arc_half={arc_half:.4} erwartet ~5.0"
        );
    }

    #[test]
    fn gl8_additive() {
        // Integral [0,0.5] + [0.5,1] muss gleich [0,1] sein.
        let seg = straight_seg_10m();
        let left = integrate_speed(&seg, 0.0, 0.5);
        let right = integrate_speed(&seg, 0.5, 1.0);
        let full = integrate_speed(&seg, 0.0, 1.0);
        assert!(
            (left + right - full).abs() < 1e-5,
            "Additivität verletzt: {left:.6} + {right:.6} != {full:.6}"
        );
    }

    #[test]
    fn gl8_curved_seg_arc_longer_than_chord() {
        // Kurvenreiches Segment: Arc > Chord (Kurve ist länger als Sehne).
        let seg = HermiteSegment {
            p0: Vec3::new(0.0, 0.0, 0.0),
            p1: Vec3::new(10.0, 0.0, 10.0),
            m0: Vec3::new(0.0, 0.0, 20.0), // starke Kurve
            m1: Vec3::new(20.0, 0.0, 0.0),
            length_m: (200.0f32).sqrt(),
            from_uid: 1,
            to_uid: 2,
            edge_uid: 1,
        };
        let chord = (200.0f32).sqrt(); // ≈ 14.14m
        let arc = integrate_speed(&seg, 0.0, 1.0);
        assert!(
            arc > chord,
            "Arc {arc:.2} muss > Chord {chord:.2} bei Kurve sein"
        );
    }

    // --- LUT Build ---

    #[test]
    fn lut_total_length_matches_arc() {
        let seg = straight_seg_10m();
        let lut = build_lut(&seg);
        assert!(
            (lut.total_length_m - 10.0).abs() < 0.01,
            "LUT total_length={:.4} erwartet ~10.0",
            lut.total_length_m
        );
    }

    #[test]
    fn lut_last_sample_is_max() {
        let seg = straight_seg_10m();
        let lut = build_lut(&seg);
        assert_eq!(
            lut.samples[LUT_N - 1],
            65535,
            "Letzter Sample muss 65535 sein"
        );
    }

    #[test]
    fn lut_samples_monotone() {
        let seg = straight_seg_10m();
        let lut = build_lut(&seg);
        for i in 1..LUT_N {
            assert!(
                lut.samples[i] >= lut.samples[i - 1],
                "Samples nicht monoton an i={i}: {} < {}",
                lut.samples[i],
                lut.samples[i - 1]
            );
        }
    }

    #[test]
    fn lut_sizeof() {
        assert_eq!(
            std::mem::size_of::<ArcLengthLUT>(),
            LUT_N * 2 + 4,
            "sizeof(ArcLengthLUT) sollte {} sein",
            LUT_N * 2 + 4
        );
    }

    // --- arc_length Query ---

    #[test]
    fn arc_length_at_t0_is_zero() {
        let seg = straight_seg_10m();
        let lut = build_lut(&seg);
        assert!((arc_length(&lut, 0.0) - 0.0).abs() < 1e-6);
    }

    #[test]
    fn arc_length_at_t1_equals_total() {
        let seg = straight_seg_10m();
        let lut = build_lut(&seg);
        let al = arc_length(&lut, 1.0);
        assert!(
            (al - lut.total_length_m).abs() < 0.01,
            "arc_length(t=1) = {al:.4} erwartet {:.4}",
            lut.total_length_m
        );
    }

    #[test]
    fn arc_length_midpoint_straight() {
        // Bei t=0.5 sollte arc ≈ 5m sein (gerade Linie).
        let seg = straight_seg_10m();
        let lut = build_lut(&seg);
        let al = arc_length(&lut, 0.5);
        assert!(
            (al - 5.0).abs() < 0.05,
            "arc_length(t=0.5) = {al:.4} erwartet ~5.0"
        );
    }

    // --- t_at_arc_length (Invers-Query) ---

    #[test]
    fn t_at_arc_length_roundtrip() {
        // t → arc_length → t_at_arc_length muss t wiedergeben.
        let seg = straight_seg_10m();
        let lut = build_lut(&seg);
        for i in 1..10 {
            let t_orig = i as f32 / 10.0;
            let s = arc_length(&lut, t_orig);
            let t_back = t_at_arc_length(&lut, &seg, s);
            assert!(
                (t_back - t_orig).abs() < 0.01,
                "Roundtrip: t={t_orig:.2} → s={s:.4} → t_back={t_back:.4}"
            );
        }
    }

    #[test]
    fn t_at_arc_length_zero_returns_zero() {
        let seg = straight_seg_10m();
        let lut = build_lut(&seg);
        assert!((t_at_arc_length(&lut, &seg, 0.0) - 0.0).abs() < 1e-6);
    }

    #[test]
    fn t_at_arc_length_total_returns_one() {
        let seg = straight_seg_10m();
        let lut = build_lut(&seg);
        let t = t_at_arc_length(&lut, &seg, lut.total_length_m);
        assert!((t - 1.0).abs() < 1e-6, "t_at_arc_length(total)={t}");
    }

    #[test]
    fn u16_quantization_error_analysis() {
        // Prüft dass der u16-Quantisierungsfehler für ein 2km-Segment < 0.1m bleibt.
        let long_seg = HermiteSegment {
            p0: Vec3::new(0.0, 0.0, 0.0),
            p1: Vec3::new(2000.0, 0.0, 0.0),
            m0: Vec3::new(2000.0, 0.0, 0.0),
            m1: Vec3::new(2000.0, 0.0, 0.0),
            length_m: 2000.0,
            from_uid: 1,
            to_uid: 2,
            edge_uid: 1,
        };
        let lut = build_lut(&long_seg);
        let total = lut.total_length_m;
        let mut max_err = 0.0f32;
        for i in 0..LUT_N {
            let t = (i + 1) as f32 / LUT_N as f32;
            let true_arc = integrate_speed(&long_seg, 0.0, t);
            let lut_arc = arc_length(&lut, t);
            // Quantisierungsfehler: u16 hat Auflösung von 1/65535 * total
            let quant_err = (lut_arc - true_arc).abs();
            max_err = max_err.max(quant_err);
            let _ = total;
        }
        assert!(
            max_err < 0.1,
            "u16-Quantisierungsfehler {max_err:.4}m > 0.1m bei 2km-Segment — STOP!"
        );
    }

    // --- point_at_arc_length ---

    #[test]
    fn point_at_arc_length_midpoint() {
        // Bei s=5m auf gerader Linie (0,0,0)→(10,0,0): Punkt ≈ (5,0,0).
        let seg = straight_seg_10m();
        let lut = build_lut(&seg);
        let p = point_at_arc_length(&seg, &lut, 5.0);
        assert!((p.x - 5.0).abs() < 0.1, "x={:.4} erwartet ~5.0", p.x);
        assert!(p.y.abs() < 0.01);
        assert!(p.z.abs() < 0.01);
    }

    // --- Adjacency + Lookahead ---

    #[test]
    fn forward_adjacency_correct() {
        // A→B→C: B hat nur A als to_uid=2, C hat from_uid=2
        let segs = vec![
            make_seg(0.0, 0.0, 10.0, 0.0, 1, 2),
            make_seg(10.0, 0.0, 20.0, 0.0, 2, 3),
        ];
        let adj = build_forward_adjacency(&segs);
        // Segment 0 startet bei node 1, Segment 1 startet bei node 2
        assert!(adj.get(&1).is_some_and(|v| v.contains(&0)));
        assert!(adj.get(&2).is_some_and(|v| v.contains(&1)));
    }

    #[test]
    fn lookahead_within_segment() {
        // 10m-Segment, lookahead 3m vom Anfang → t sollte ≈ 0.3 sein
        let segs = vec![straight_seg_10m()];
        let luts = build_all_luts(&segs);
        let adj = build_forward_adjacency(&segs);
        let result = lookahead(0, 0.0, 3.0, &adj, &segs, &luts).unwrap();
        assert_eq!(result.seg_idx, 0);
        assert!(
            (result.point.x - 3.0).abs() < 0.2,
            "x={:.4}",
            result.point.x
        );
        assert_eq!(result.remaining_dist_m, 0.0);
    }

    #[test]
    fn lookahead_crosses_segment_boundary() {
        // 3 Segmente je 10m: lookahead 25m von t=0 → landet in Segment 2, bei x≈5m
        let segs = vec![
            make_seg(0.0, 0.0, 10.0, 0.0, 1, 2),
            make_seg(10.0, 0.0, 20.0, 0.0, 2, 3),
            make_seg(20.0, 0.0, 30.0, 0.0, 3, 4),
        ];
        let luts = build_all_luts(&segs);
        let adj = build_forward_adjacency(&segs);
        let result = lookahead(0, 0.0, 25.0, &adj, &segs, &luts).unwrap();
        assert_eq!(result.seg_idx, 2, "Sollte in Segment 2 landen");
        assert!(
            (result.point.x - 25.0).abs() < 0.5,
            "x={:.4} erwartet ~25.0",
            result.point.x
        );
    }

    #[test]
    fn lookahead_dead_end_returns_remaining() {
        // Einzelnes 10m-Segment, lookahead 15m → Dead-End, remaining=5m
        let segs = vec![straight_seg_10m()];
        let luts = build_all_luts(&segs);
        let adj = build_forward_adjacency(&segs);
        let result = lookahead(0, 0.0, 15.0, &adj, &segs, &luts).unwrap();
        assert!(
            (result.remaining_dist_m - 5.0).abs() < 0.1,
            "remaining={:.4} erwartet ~5.0",
            result.remaining_dist_m
        );
    }

    #[test]
    fn lookahead_heading_filters_wrong_direction() {
        // T-Junction: Segment 0→1, dann zwei Kandidaten:
        //   - Segment 1→2: geradeaus (dot≈1.0) ✓
        //   - Segment 1→3: 90° Kurve (dot≈0.0) ✗
        // lookahead sollte geradeaus gehen.
        let segs = vec![
            make_seg(0.0, 0.0, 10.0, 0.0, 1, 2),   // 0: Ost
            make_seg(10.0, 0.0, 20.0, 0.0, 2, 3),  // 1: Ost (geradeaus) ✓
            make_seg(10.0, 0.0, 10.0, 10.0, 2, 4), // 2: Nord (90°) ✗
        ];
        let luts = build_all_luts(&segs);
        let adj = build_forward_adjacency(&segs);
        let result = lookahead(0, 0.0, 15.0, &adj, &segs, &luts).unwrap();
        assert_eq!(
            result.seg_idx, 1,
            "Sollte geradeaus weiterfahren (Seg 1), nicht abbiegen"
        );
    }

    /// b-ii-Reverse-Geschwister: from/to vertauscht, Geometrie rückwärts (West), aber
    /// kopierte Vorwärts-Tangente bei t=0 (täuscht den Heading-Dot-Filter). Hier sogar
    /// mit HÖHEREM t=0-Dot (1.0) als der echte Nachfolger (leichte Kurve, ~0.98) und VOR
    /// ihm in der Adjacency → ohne Skip würde es gewinnen und der Lookahead liefe rückwärts.
    #[test]
    fn lookahead_skips_reverse_sibling() {
        let a = make_seg(0.0, 0.0, 10.0, 0.0, 1, 2); // seg 0: Ost
        let r = HermiteSegment {
            p0: Vec3::new(10.0, 0.0, 0.0),
            p1: Vec3::new(0.0, 0.0, 0.0),  // Geometrie West (rückwärts)
            m0: Vec3::new(10.0, 0.0, 0.0), // kopiert (Ost) statt negiert → t=0 zeigt vorwärts
            m1: Vec3::new(10.0, 0.0, 0.0),
            length_m: 10.0,
            from_uid: 2, // exaktes Reverse-Geschwister von a (from/to vertauscht)
            to_uid: 1,
            edge_uid: 201,
        };
        let c = make_seg(10.0, 0.0, 20.0, 2.0, 2, 3); // seg 2: echte Fortsetzung (leichte Kurve)
        let segs = vec![a, r, c];
        let luts = build_all_luts(&segs);
        let adj = build_forward_adjacency(&segs);
        // adj[2] = [1(R), 2(C)] — R steht vorne und hätte den höheren t=0-Dot.
        let result = lookahead(0, 0.0, 15.0, &adj, &segs, &luts).unwrap();
        assert_eq!(
            result.seg_idx, 2,
            "Reverse-Geschwister (Seg 1) muss übersprungen, echte Fortsetzung (Seg 2) gewählt werden"
        );
        assert!(
            result.point.x > 10.0,
            "Lookahead muss vorwärts (x>10) liegen, nicht rückwärts; x={:.2}",
            result.point.x
        );
    }

    /// Regressions-Guard: ohne Reverse-Geschwister bleibt die echte Fortsetzung wählbar
    /// (der Skip darf NUR das exakte from/to-vertauschte Geschwister treffen).
    #[test]
    fn lookahead_keeps_real_successor() {
        let segs = vec![
            make_seg(0.0, 0.0, 10.0, 0.0, 1, 2),  // 0: Ost
            make_seg(10.0, 0.0, 20.0, 0.0, 2, 3), // 1: echte Fortsetzung Ost (to=3 ≠ from=1)
        ];
        let luts = build_all_luts(&segs);
        let adj = build_forward_adjacency(&segs);
        let result = lookahead(0, 0.0, 15.0, &adj, &segs, &luts).unwrap();
        assert_eq!(result.seg_idx, 1, "Echte Fortsetzung muss gewählt werden");
        assert_eq!(
            result.remaining_dist_m, 0.0,
            "Vorwärts-Walk muss gelingen (kein Dead-End durch versehentlichen Skip)"
        );
        assert!(
            result.point.x > 10.0,
            "x={:.2} muss vorwärts sein",
            result.point.x
        );
    }

    /// Tie-Break: Reverse-Geschwister und echter Nachfolger haben IDENTISCHEN t=0-Dot
    /// (beide Ost, 1.0), das Reverse steht zuerst in der Adjacency. Nach dem Skip wird
    /// deterministisch der echte Nachfolger gewählt.
    #[test]
    fn lookahead_tie_break_picks_forward() {
        let a = make_seg(0.0, 0.0, 10.0, 0.0, 1, 2); // seg 0
        let r = HermiteSegment {
            p0: Vec3::new(10.0, 0.0, 0.0),
            p1: Vec3::new(0.0, 0.0, 0.0),
            m0: Vec3::new(10.0, 0.0, 0.0), // identische Vorwärts-Tangente wie der echte Nachfolger
            m1: Vec3::new(10.0, 0.0, 0.0),
            length_m: 10.0,
            from_uid: 2,
            to_uid: 1,
            edge_uid: 201,
        };
        let c = make_seg(10.0, 0.0, 20.0, 0.0, 2, 3); // seg 2: gerade Ost, gleicher Dot
        let segs = vec![a, r, c];
        let luts = build_all_luts(&segs);
        let adj = build_forward_adjacency(&segs);
        let result = lookahead(0, 0.0, 15.0, &adj, &segs, &luts).unwrap();
        assert_eq!(
            result.seg_idx, 2,
            "Bei gleichem Dot muss nach dem Reverse-Skip der echte Nachfolger (Seg 2) gewinnen"
        );
    }
}
