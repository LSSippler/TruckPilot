//! Phase 0c — R-tree Spatial Index über Hermite-Spline-Segmente
//!
//! Ermöglicht schnelle Spatial Queries:
//! - [`SplineIndex::nearest`] — k nächste Segmente zu einem Punkt
//! - [`SplineIndex::within_radius`] — alle Segmente im Umkreis
//! - [`SplineIndex::nearest_with_projection`] — nächstes Segment + exakte Projektion
//!
//! Index-Aufbau: rstar R*-Tree über Segment-BBoxes (AABB in XZ-Ebene).
//! Projektion: Newton-Raphson auf kubischer Hermite-Kurve.

use rstar::{PointDistance, RTree, RTreeObject, AABB};

use crate::spline::{evaluate, evaluate_tangent, HermiteSegment, Vec3};

// ---------------------------------------------------------------------------
// Segment-AABB (intern + export)
// ---------------------------------------------------------------------------

/// 2D-AABB (XZ-Ebene) über ein Hermite-Segment.
/// Y (Höhe) wird ignoriert — Queries sind 2D-Nähe auf der Karte.
#[derive(Debug, Clone, Copy)]
pub struct SegmentAabb {
    pub x_min: f32,
    pub z_min: f32,
    pub x_max: f32,
    pub z_max: f32,
}

impl SegmentAabb {
    /// Naive AABB: nur über p0 und p1 (Chord). Schnell, leicht zu eng bei Kurven.
    pub fn from_chord(seg: &HermiteSegment) -> Self {
        let x_min = seg.p0.x.min(seg.p1.x);
        let x_max = seg.p0.x.max(seg.p1.x);
        let z_min = seg.p0.z.min(seg.p1.z);
        let z_max = seg.p0.z.max(seg.p1.z);
        // Padding: mindestens 1m in jede Richtung (verhindert Null-Volumen bei geraden Edges)
        Self {
            x_min: x_min - 1.0,
            z_min: z_min - 1.0,
            x_max: x_max + 1.0,
            z_max: z_max + 1.0,
        }
    }

    /// Sampled AABB: sampelt N Punkte der Hermite-Kurve, nimmt deren Hülle.
    /// Korrekt auch bei starken Kurven (Bulge-Kompensation).
    pub fn from_sampled(seg: &HermiteSegment, n: usize) -> Self {
        let n = n.max(2);
        let mut x_min = f32::INFINITY;
        let mut x_max = f32::NEG_INFINITY;
        let mut z_min = f32::INFINITY;
        let mut z_max = f32::NEG_INFINITY;
        for i in 0..=n {
            let t = i as f32 / n as f32;
            let p = evaluate(seg, t);
            x_min = x_min.min(p.x);
            x_max = x_max.max(p.x);
            z_min = z_min.min(p.z);
            z_max = z_max.max(p.z);
        }
        // Kleines Padding für Floating-Point-Toleranz
        Self {
            x_min: x_min - 0.5,
            z_min: z_min - 0.5,
            x_max: x_max + 0.5,
            z_max: z_max + 0.5,
        }
    }

    #[inline]
    pub fn area(&self) -> f32 {
        ((self.x_max - self.x_min) * (self.z_max - self.z_min)).max(0.0)
    }
}

// ---------------------------------------------------------------------------
// rstar-Wrapper
// ---------------------------------------------------------------------------

/// rstar-Objekt: hält den Index des Segments in SplineIndex::segments.
/// Leichtgewichtig — keine Kopie der Segment-Daten.
#[derive(Clone)]
struct SegmentEntry {
    /// Index in `SplineIndex::segments`
    idx: u32,
    aabb: SegmentAabb,
}

impl RTreeObject for SegmentEntry {
    type Envelope = AABB<[f32; 2]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(
            [self.aabb.x_min, self.aabb.z_min],
            [self.aabb.x_max, self.aabb.z_max],
        )
    }
}

impl PointDistance for SegmentEntry {
    fn distance_2(&self, point: &[f32; 2]) -> f32 {
        // Distanz² zum nächsten Punkt der AABB
        let dx = (self.aabb.x_min.max(point[0]).min(self.aabb.x_max) - point[0]).powi(2);
        let dz = (self.aabb.z_min.max(point[1]).min(self.aabb.z_max) - point[1]).powi(2);
        dx + dz
    }
}

// ---------------------------------------------------------------------------
// Newton-Projektion
// ---------------------------------------------------------------------------

/// Zweite Ableitung der Hermite-Basisfunktionen.
///
///   h00''(t) = 12t − 6
///   h10''(t) =  6t − 4
///   h01''(t) = −12t + 6
///   h11''(t) =  6t − 2
#[inline]
fn hermite_basis_second_derivative(t: f32) -> (f32, f32, f32, f32) {
    (
        12.0 * t - 6.0,
        6.0 * t - 4.0,
        -12.0 * t + 6.0,
        6.0 * t - 2.0,
    )
}

#[inline]
fn evaluate_second_derivative(seg: &HermiteSegment, t: f32) -> Vec3 {
    let (d2h00, d2h10, d2h01, d2h11) = hermite_basis_second_derivative(t);
    seg.p0 * d2h00 + seg.m0 * d2h10 + seg.p1 * d2h01 + seg.m1 * d2h11
}

/// Nächster Punkt auf einem Hermite-Segment via Newton-Raphson.
///
/// Minimiert f(t) = |p(t) − Q|² über t ∈ [0,1].
/// Bedingung 1. Ordnung: g(t) = (p(t) − Q) · p'(t) = 0
/// Newton-Schritt: t ← t − g(t) / g'(t)
/// g'(t) = |p'(t)|² + (p(t) − Q) · p''(t)
///
/// Liefert (t_best, dist²_best). Konvergiert bei normaler Straßengeometrie in 3–5 Iterationen.
fn newton_closest(seg: &HermiteSegment, query: Vec3, t_init: f32) -> (f32, f32) {
    const MAX_ITER: usize = 10;
    const EPS: f32 = 1e-6;

    let mut t = t_init.clamp(0.0, 1.0);

    for _ in 0..MAX_ITER {
        let p = evaluate(seg, t);
        let dp = evaluate_tangent(seg, t);
        let d2p = evaluate_second_derivative(seg, t);

        let diff = Vec3::new(p.x - query.x, p.y - query.y, p.z - query.z);

        // g(t) = diff · dp (in XZ für 2D, aber wir nutzen 3D für Korrektheit)
        let g = diff.x * dp.x + diff.y * dp.y + diff.z * dp.z;
        // g'(t) = |dp|² + diff · d2p
        let gp = dp.x * dp.x
            + dp.y * dp.y
            + dp.z * dp.z
            + diff.x * d2p.x
            + diff.y * d2p.y
            + diff.z * d2p.z;

        if gp.abs() < 1e-10 {
            break; // Flacher Gradient — kein Fortschritt möglich
        }

        let t_new = (t - g / gp).clamp(0.0, 1.0);
        if (t_new - t).abs() < EPS {
            t = t_new;
            break;
        }
        t = t_new;
    }

    let p = evaluate(seg, t);
    let dist2 = (p.x - query.x).powi(2) + (p.y - query.y).powi(2) + (p.z - query.z).powi(2);
    (t, dist2)
}

/// Ergebnis von [`SplineIndex::nearest_with_projection`].
#[derive(Debug, Clone)]
pub struct NearestHit {
    /// Index des Segments in `SplineIndex::segments`
    pub segment_idx: usize,
    /// Hermite-Parameter t ∈ [0,1] des nächsten Punkts
    pub t: f32,
    /// Der nächste Punkt auf der Kurve
    pub point_on_curve: Vec3,
    /// Euklidische Distanz vom Query-Punkt zur Kurve (in Metern)
    pub dist_m: f32,
    /// Heading an diesem Punkt (0=Nord, 90=Ost, Grad)
    pub heading_deg: f32,
}

// ---------------------------------------------------------------------------
// SplineIndex
// ---------------------------------------------------------------------------

/// R*-Tree-Index über alle Hermite-Segmente.
///
/// Aufgebaut via [`build_index`], nutzt rstar Bulk-Load für optimale Baumstruktur.
pub struct SplineIndex {
    /// Alle Segmente — Ownership liegt hier.
    pub segments: Vec<HermiteSegment>,
    /// rstar R*-Tree mit SegmentEntry-Referenzen (Index in `segments`).
    tree: RTree<SegmentEntry>,
}

/// Baut einen SplineIndex aus einem Segment-Vec.
///
/// Nutzt rstar-Bulk-Load (O(n log n), optimal für statische Daten).
pub fn build_index(segments: Vec<HermiteSegment>) -> SplineIndex {
    let entries: Vec<SegmentEntry> = segments
        .iter()
        .enumerate()
        .map(|(i, seg)| SegmentEntry {
            idx: i as u32,
            aabb: SegmentAabb::from_sampled(seg, 10),
        })
        .collect();

    let tree = RTree::bulk_load(entries);
    SplineIndex { segments, tree }
}

/// AABB-Vergleichsstatistik (für Report).
pub struct AabbComparisonStats {
    pub mean_chord_area: f64,
    pub mean_sampled_area: f64,
    /// Anteil Segmente wo sampled_area > chord_area * 1.01 (Bulge vorhanden)
    pub pct_bulge: f64,
}

/// Vergleicht chord_aabb vs sampled_aabb auf einer Stichprobe.
pub fn compare_aabb_strategies(
    segments: &[HermiteSegment],
    sample_n: usize,
) -> AabbComparisonStats {
    let step = (segments.len() / sample_n.max(1)).max(1);
    let mut chord_areas = Vec::new();
    let mut sampled_areas = Vec::new();
    let mut bulge_count = 0usize;
    let mut total = 0usize;

    for seg in segments.iter().step_by(step) {
        let chord = SegmentAabb::from_chord(seg);
        let sampled = SegmentAabb::from_sampled(seg, 10);
        chord_areas.push(chord.area() as f64);
        sampled_areas.push(sampled.area() as f64);
        if sampled.area() > chord.area() * 1.01 {
            bulge_count += 1;
        }
        total += 1;
    }

    let mean_chord = chord_areas.iter().sum::<f64>() / total as f64;
    let mean_sampled = sampled_areas.iter().sum::<f64>() / total as f64;
    AabbComparisonStats {
        mean_chord_area: mean_chord,
        mean_sampled_area: mean_sampled,
        pct_bulge: bulge_count as f64 / total as f64 * 100.0,
    }
}

impl SplineIndex {
    /// Gibt die k nächsten Segmente zu einem Query-Punkt zurück (nach AABB-Distanz).
    /// Rückgabe: `(segment_ref, dist_to_aabb_m)`, aufsteigend sortiert.
    pub fn nearest(&self, point: Vec3, k: usize) -> Vec<(&HermiteSegment, f32)> {
        let p2 = [point.x, point.z];
        self.tree
            .nearest_neighbor_iter_with_distance_2(&p2)
            .take(k)
            .map(|(entry, dist2)| (&self.segments[entry.idx as usize], dist2.sqrt()))
            .collect()
    }

    /// Gibt alle Segmente zurück, deren AABB innerhalb von `radius_m` liegt.
    pub fn within_radius(&self, point: Vec3, radius_m: f32) -> Vec<&HermiteSegment> {
        let p2 = [point.x, point.z];
        let r2 = radius_m * radius_m;
        self.tree
            .locate_within_distance(p2, r2)
            .map(|entry| &self.segments[entry.idx as usize])
            .collect()
    }

    /// Findet das nächste Segment mit exakter Projektion via Newton-Raphson.
    ///
    /// Strategie:
    /// 1. Hole `candidates` nächste Segmente via R-tree (AABB-Distanz)
    /// 2. Für jedes Kandidat-Segment: Newton von 3 Seeds (t=0, 0.5, 1)
    /// 3. Bestes (t, dist²) gewinnt
    ///
    /// Gibt `None` bei leerem Index.
    pub fn nearest_with_projection(&self, point: Vec3, candidates: usize) -> Option<NearestHit> {
        let p2 = [point.x, point.z];

        let mut best: Option<(usize, f32, f32)> = None; // (seg_idx, t, dist2)

        for (entry, _aabb_dist2) in self
            .tree
            .nearest_neighbor_iter_with_distance_2(&p2)
            .take(candidates)
        {
            let seg_idx = entry.idx as usize;
            let seg = &self.segments[seg_idx];

            // Newton von 3 Seeds
            let seeds = [0.0f32, 0.5, 1.0];
            let (t_best, d2_best) = seeds
                .iter()
                .map(|&t0| newton_closest(seg, point, t0))
                .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                .unwrap();

            match best {
                None => best = Some((seg_idx, t_best, d2_best)),
                Some((_, _, best_d2)) if d2_best < best_d2 => {
                    best = Some((seg_idx, t_best, d2_best));
                }
                _ => {}
            }
        }

        let (seg_idx, t, dist2) = best?;
        let seg = &self.segments[seg_idx];
        let point_on_curve = evaluate(seg, t);
        let tan = evaluate_tangent(seg, t);
        let heading_deg = f32::atan2(tan.x, -tan.z).to_degrees().rem_euclid(360.0);

        Some(NearestHit {
            segment_idx: seg_idx,
            t,
            point_on_curve,
            dist_m: dist2.sqrt(),
            heading_deg,
        })
    }

    /// Gibt Speicher-Statistiken zurück.
    pub fn memory_stats(&self) -> IndexMemoryStats {
        let seg_bytes = self.segments.len() * std::mem::size_of::<HermiteSegment>();
        // rstar-Entry: AABB (4×f32=16B) + idx (4B) + overhead ≈ 32B
        let entry_bytes = self.tree.size() * 32;
        // rstar-Interne-Knoten: typisch ~10–20% Overhead über Entries
        let tree_overhead = entry_bytes + entry_bytes / 10;
        IndexMemoryStats {
            segment_count: self.segments.len(),
            seg_bytes,
            tree_entry_count: self.tree.size(),
            tree_bytes_approx: tree_overhead,
            total_bytes_approx: seg_bytes + tree_overhead,
        }
    }
}

/// Speicher-Statistiken für SplineIndex.
#[derive(Debug, Clone)]
pub struct IndexMemoryStats {
    pub segment_count: usize,
    pub seg_bytes: usize,
    pub tree_entry_count: usize,
    pub tree_bytes_approx: usize,
    pub total_bytes_approx: usize,
}

impl IndexMemoryStats {
    pub fn print_summary(&self) {
        println!("=== SplineIndex Memory ===");
        println!(
            "  Segmente:          {:>8} ({:.1} MB)",
            self.segment_count,
            self.seg_bytes as f64 / 1_048_576.0
        );
        println!(
            "  R-tree Entries:    {:>8} (~{:.1} MB)",
            self.tree_entry_count,
            self.tree_bytes_approx as f64 / 1_048_576.0
        );
        println!(
            "  Total:             {:>8} ({:.1} MB)",
            self.total_bytes_approx,
            self.total_bytes_approx as f64 / 1_048_576.0
        );
    }
}

// ---------------------------------------------------------------------------
// Unit-Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spline::HermiteSegment;

    fn make_seg(x0: f32, z0: f32, x1: f32, z1: f32) -> HermiteSegment {
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
            from_uid: 1,
            to_uid: 2,
            edge_uid: 99,
        }
    }

    // --- AABB Tests ---

    #[test]
    fn chord_aabb_contains_endpoints() {
        let seg = make_seg(0.0, 0.0, 10.0, 5.0);
        let aabb = SegmentAabb::from_chord(&seg);
        assert!(aabb.x_min <= 0.0 && aabb.x_max >= 10.0);
        assert!(aabb.z_min <= 0.0 && aabb.z_max >= 5.0);
    }

    #[test]
    fn sampled_aabb_contains_all_curve_points() {
        let seg = make_seg(0.0, 0.0, 10.0, 0.0);
        let aabb = SegmentAabb::from_sampled(&seg, 20);
        for i in 0..=20 {
            let t = i as f32 / 20.0;
            let p = evaluate(&seg, t);
            assert!(
                p.x >= aabb.x_min && p.x <= aabb.x_max,
                "x={} not in [{}, {}]",
                p.x,
                aabb.x_min,
                aabb.x_max
            );
            assert!(
                p.z >= aabb.z_min && p.z <= aabb.z_max,
                "z={} not in [{}, {}]",
                p.z,
                aabb.z_min,
                aabb.z_max
            );
        }
    }

    #[test]
    fn sampled_aabb_bigger_than_chord_for_curved_seg() {
        // Starke S-Kurve: Tangenten senkrecht zur Chord → signifikanter Bulge
        let seg = HermiteSegment {
            p0: Vec3::new(0.0, 0.0, 0.0),
            p1: Vec3::new(100.0, 0.0, 0.0),
            m0: Vec3::new(0.0, 0.0, 100.0), // 90° zur Chord
            m1: Vec3::new(0.0, 0.0, -100.0),
            length_m: 100.0,
            from_uid: 1,
            to_uid: 2,
            edge_uid: 1,
        };
        let chord_a = SegmentAabb::from_chord(&seg);
        let sampled_a = SegmentAabb::from_sampled(&seg, 20);
        // Sampled AABB sollte in z größer sein (Bulge in z-Richtung)
        assert!(
            sampled_a.z_max > chord_a.z_max || sampled_a.z_min < chord_a.z_min,
            "Expected sampled AABB to be larger in z; chord_z=[{:.1},{:.1}] sampled_z=[{:.1},{:.1}]",
            chord_a.z_min, chord_a.z_max, sampled_a.z_min, sampled_a.z_max
        );
    }

    // --- Newton-Projektion Tests ---

    #[test]
    fn newton_projects_to_endpoint_at_t0() {
        let seg = make_seg(0.0, 0.0, 10.0, 0.0);
        let query = Vec3::new(-5.0, 0.0, 0.0); // links von p0
        let (t, _) = newton_closest(&seg, query, 0.0);
        assert!(t < 0.1, "Punkt links von p0 → t≈0, got {t}");
    }

    #[test]
    fn newton_projects_to_endpoint_at_t1() {
        let seg = make_seg(0.0, 0.0, 10.0, 0.0);
        let query = Vec3::new(15.0, 0.0, 0.0); // rechts von p1
        let (t, _) = newton_closest(&seg, query, 1.0);
        assert!(t > 0.9, "Punkt rechts von p1 → t≈1, got {t}");
    }

    #[test]
    fn newton_projects_to_midpoint_on_straight() {
        // Gerade Linie (0,0)→(10,0): Punkt bei (5, 0, 3) → nächster Punkt = (5, 0, 0)
        let seg = make_seg(0.0, 0.0, 10.0, 0.0);
        let query = Vec3::new(5.0, 0.0, 3.0);
        let (t, dist2) = newton_closest(&seg, query, 0.5);
        assert!((t - 0.5).abs() < 0.05, "Mittelpunkt → t≈0.5, got {t}");
        assert!(
            (dist2.sqrt() - 3.0).abs() < 0.1,
            "Distanz≈3m, got {}",
            dist2.sqrt()
        );
    }

    // --- Index-Tests ---

    fn toy_index() -> SplineIndex {
        // 4 Segmente in 2D-Grid
        let segs = vec![
            make_seg(0.0, 0.0, 10.0, 0.0),    // idx 0: (0,0)→(10,0)
            make_seg(20.0, 0.0, 30.0, 0.0),   // idx 1: (20,0)→(30,0)
            make_seg(0.0, 20.0, 10.0, 20.0),  // idx 2: (0,20)→(10,20)
            make_seg(20.0, 20.0, 30.0, 20.0), // idx 3: (20,20)→(30,20)
        ];
        build_index(segs)
    }

    #[test]
    fn nearest_returns_k_results() {
        let idx = toy_index();
        let results = idx.nearest(Vec3::new(5.0, 0.0, 0.0), 2);
        assert_eq!(results.len(), 2, "nearest(k=2) returns 2 results");
        // Nächstes sollte Segment 0 sein (liegt direkt bei (0,0)→(10,0))
        assert_eq!(results[0].0.from_uid, 1, "Segment 0 is nearest");
    }

    #[test]
    fn nearest_empty_index() {
        let idx = build_index(vec![]);
        let results = idx.nearest(Vec3::new(0.0, 0.0, 0.0), 5);
        assert!(results.is_empty());
    }

    #[test]
    fn within_radius_finds_nearby() {
        let idx = toy_index();
        // Query bei (5, 0, 0) mit Radius 2m → sollte Segment 0 finden
        let results = idx.within_radius(Vec3::new(5.0, 0.0, 0.0), 2.0);
        assert!(!results.is_empty(), "Should find segment 0 within 2m");
    }

    #[test]
    fn within_radius_excludes_far() {
        let idx = toy_index();
        // Query bei (5, 0, 0) mit Radius 5m — Segment 1 ist bei x=20, also 15m weg
        let results = idx.within_radius(Vec3::new(5.0, 0.0, 0.0), 5.0);
        // Segment 1 sollte nicht dabei sein
        let has_seg1 = results.iter().any(|s| s.p0.x > 15.0);
        assert!(!has_seg1, "Segment at x=20 should not be within 5m of x=5");
    }

    #[test]
    fn nearest_with_projection_finds_correct_segment() {
        let idx = toy_index();
        let query = Vec3::new(5.0, 0.0, 1.0); // knapp über Segment 0
        let hit = idx.nearest_with_projection(query, 4).unwrap();
        // Nächster Punkt sollte auf Segment 0 sein, t≈0.5
        assert!(
            hit.dist_m < 2.0,
            "Distanz sollte < 2m sein, got {}",
            hit.dist_m
        );
        assert!((hit.t - 0.5).abs() < 0.15, "t≈0.5, got {}", hit.t);
    }

    #[test]
    fn nearest_with_projection_single_segment() {
        let idx = build_index(vec![make_seg(0.0, 0.0, 10.0, 0.0)]);
        let hit = idx
            .nearest_with_projection(Vec3::new(5.0, 0.0, 0.0), 1)
            .unwrap();
        assert!(
            hit.dist_m < 0.01,
            "Query on segment → dist≈0, got {}",
            hit.dist_m
        );
    }

    #[test]
    fn projection_heading_on_eastward_seg() {
        // Segment von (0,0) nach (10,0) → Ost-Richtung = 90°
        let seg = make_seg(0.0, 0.0, 10.0, 0.0);
        let idx = build_index(vec![seg]);
        let hit = idx
            .nearest_with_projection(Vec3::new(5.0, 0.0, 0.0), 1)
            .unwrap();
        assert!(
            (hit.heading_deg - 90.0).abs() < 5.0,
            "Ost-Segment → heading≈90°, got {}",
            hit.heading_deg
        );
    }
}
