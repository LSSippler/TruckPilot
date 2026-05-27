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

use crate::spline::{evaluate, evaluate_tangent, HermiteSegment, SegmentMetadata, Vec3};

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

/// Ergebnis von [`SplineIndex::nearest_with_projection`] und
/// [`SplineIndex::nearest_with_heading_filter`].
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
    /// `true` wenn der Heading-Filter mind. einen kompatiblen Kandidaten gefunden hat.
    /// Immer `false` bei `nearest_with_projection`.
    pub heading_filter_applied: bool,
}

// ---------------------------------------------------------------------------
// SplineIndex
// ---------------------------------------------------------------------------

/// R*-Tree-Index über alle Hermite-Segmente.
///
/// Aufgebaut via [`build_index`] oder [`build_index_with_metadata`].
pub struct SplineIndex {
    /// Alle Segmente — Ownership liegt hier.
    pub segments: Vec<HermiteSegment>,
    /// Per-Segment DS8-Metadaten; `metadata[i]` gehört zu `segments[i]`.
    /// `None` für Nicht-Road-Edges (prefab, building, ferry, …).
    pub metadata: Vec<Option<SegmentMetadata>>,
    /// rstar R*-Tree mit SegmentEntry-Referenzen (Index in `segments`).
    tree: RTree<SegmentEntry>,
}

/// Baut einen SplineIndex mit Segment-Metadaten (DS8).
///
/// `metadata.len()` muss `segments.len()` entsprechen.
pub fn build_index_with_metadata(
    segments: Vec<HermiteSegment>,
    metadata: Vec<Option<SegmentMetadata>>,
) -> SplineIndex {
    let entries: Vec<SegmentEntry> = segments
        .iter()
        .enumerate()
        .map(|(i, seg)| SegmentEntry {
            idx: i as u32,
            aabb: SegmentAabb::from_sampled(seg, 10),
        })
        .collect();
    let tree = RTree::bulk_load(entries);
    SplineIndex { segments, metadata, tree }
}

/// Baut einen SplineIndex aus einem Segment-Vec (alle Metadaten `None`).
///
/// Nutzt rstar-Bulk-Load (O(n log n), optimal für statische Daten).
pub fn build_index(segments: Vec<HermiteSegment>) -> SplineIndex {
    let n = segments.len();
    build_index_with_metadata(segments, vec![None; n])
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

    /// Wie [`within_radius`], liefert aber zusätzlich den Segment-Index und die Metadaten.
    ///
    /// Rückgabe: `Vec<(segment_idx, &HermiteSegment, Option<SegmentMetadata>)>`
    ///
    /// Wird vom Core-Daemon für `SpatialSegmentsInRadius`-IPC-Queries genutzt
    /// (Overlay HUD Phase 6.9).
    pub fn within_radius_with_idx(
        &self,
        point: Vec3,
        radius_m: f32,
    ) -> Vec<(usize, &HermiteSegment, Option<SegmentMetadata>)> {
        let p2 = [point.x, point.z];
        let r2 = radius_m * radius_m;
        self.tree
            .locate_within_distance(p2, r2)
            .map(|entry| {
                let idx = entry.idx as usize;
                (idx, &self.segments[idx], self.metadata[idx])
            })
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
            heading_filter_applied: false,
        })
    }

    /// Findet das geometrisch nächste Segment, das ein Prädikat erfüllt.
    ///
    /// Wie [`nearest_with_projection`], aber überspringt Segmente, für die
    /// `pred(seg_idx, metadata)` `false` zurückgibt.
    ///
    /// * `candidates` — maximale R-tree-Einträge, die untersucht werden.
    ///   Bei dünner Prädikat-Übereinstimmung (z.B. prefab-only in einem
    ///   Road-dominierten Index) sollte dieser Wert deutlich höher als der
    ///   Standard-`CANDIDATES` gewählt werden (z.B. `CANDIDATES * 16`).
    ///
    /// Kein Heading-Filter — für Bias-Queries in Junctions sind beliebige
    /// Einfahrtswinkel zu tolerieren.
    pub fn nearest_with_projection_filtered<F>(
        &self,
        point: Vec3,
        candidates: usize,
        pred: F,
    ) -> Option<NearestHit>
    where
        F: Fn(usize, Option<SegmentMetadata>) -> bool,
    {
        let p2 = [point.x, point.z];
        let mut best: Option<(usize, f32, f32)> = None;

        for (entry, _) in self
            .tree
            .nearest_neighbor_iter_with_distance_2(&p2)
            .take(candidates)
        {
            let seg_idx = entry.idx as usize;
            let meta = self.metadata[seg_idx];
            if !pred(seg_idx, meta) {
                continue;
            }
            let seg = &self.segments[seg_idx];
            let seeds = [0.0f32, 0.5, 1.0];
            let (t_best, d2_best) = seeds
                .iter()
                .map(|&t0| newton_closest(seg, point, t0))
                .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                .unwrap();
            match best {
                None => best = Some((seg_idx, t_best, d2_best)),
                Some((_, _, bd2)) if d2_best < bd2 => {
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
            heading_filter_applied: false,
        })
    }

    /// Findet das nächste Segment mit Heading-Filter.
    ///
    /// Wie [`nearest_with_projection`], aber bevorzugt Segmente deren Tangente in
    /// die gleiche Richtung wie `truck_heading_deg` zeigt (dot ≥ 0.5, ≤60° Abweichung).
    /// Fällt auf geometrisch nächstes zurück wenn alle Kandidaten gefiltert werden
    /// (`heading_filter_applied` = `false` im Rückgabewert).
    ///
    /// * `truck_heading_deg` — Fahrtrichtung in CW-Grad von Nord (0=N, 90=E, 180=S, 270=W)
    pub fn nearest_with_heading_filter(
        &self,
        point: Vec3,
        truck_heading_deg: f32,
        candidates: usize,
    ) -> Option<NearestHit> {
        let h_rad = truck_heading_deg.to_radians();
        let fw_x = h_rad.sin();
        let fw_z = -h_rad.cos();

        let p2 = [point.x, point.z];

        let mut best_unfiltered: Option<(usize, f32, f32)> = None; // (seg_idx, t, dist2)
        let mut best_filtered: Option<(usize, f32, f32)> = None;

        for (entry, _) in self
            .tree
            .nearest_neighbor_iter_with_distance_2(&p2)
            .take(candidates)
        {
            let seg_idx = entry.idx as usize;
            let seg = &self.segments[seg_idx];

            let seeds = [0.0f32, 0.5, 1.0];
            let (t_best, d2_best) = seeds
                .iter()
                .map(|&t0| newton_closest(seg, point, t0))
                .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
                .unwrap();

            match best_unfiltered {
                None => best_unfiltered = Some((seg_idx, t_best, d2_best)),
                Some((_, _, bd2)) if d2_best < bd2 => {
                    best_unfiltered = Some((seg_idx, t_best, d2_best));
                }
                _ => {}
            }

            let tan = evaluate_tangent(seg, t_best);
            let tan_len = (tan.x * tan.x + tan.z * tan.z).sqrt();
            if tan_len > 1e-6 {
                let dot = (tan.x / tan_len) * fw_x + (tan.z / tan_len) * fw_z;
                if dot >= 0.5 {
                    match best_filtered {
                        None => best_filtered = Some((seg_idx, t_best, d2_best)),
                        Some((_, _, bd2)) if d2_best < bd2 => {
                            best_filtered = Some((seg_idx, t_best, d2_best));
                        }
                        _ => {}
                    }
                }
            }
        }

        let (seg_idx, t, dist2, filter_applied) = if let Some((si, t, d2)) = best_filtered {
            (si, t, d2, true)
        } else if let Some((si, t, d2)) = best_unfiltered {
            (si, t, d2, false)
        } else {
            return None;
        };

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
            heading_filter_applied: filter_applied,
        })
    }

    /// Gibt Speicher-Statistiken zurück.
    pub fn memory_stats(&self) -> IndexMemoryStats {
        let seg_bytes = self.segments.len() * std::mem::size_of::<HermiteSegment>();
        let meta_bytes = self.metadata.len() * std::mem::size_of::<Option<SegmentMetadata>>();
        // rstar-Entry: AABB (4×f32=16B) + idx (4B) + overhead ≈ 32B
        let entry_bytes = self.tree.size() * 32;
        // rstar-Interne-Knoten: typisch ~10–20% Overhead über Entries
        let tree_overhead = entry_bytes + entry_bytes / 10;
        IndexMemoryStats {
            segment_count: self.segments.len(),
            seg_bytes: seg_bytes + meta_bytes,
            tree_entry_count: self.tree.size(),
            tree_bytes_approx: tree_overhead,
            total_bytes_approx: seg_bytes + meta_bytes + tree_overhead,
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

    // --- nearest_with_heading_filter Tests ---

    #[test]
    fn heading_filter_selects_north_on_bidirectional_road() {
        // Bidirektionale Straße: Nord-Segment (0,0)→(0,-100) und Süd-Segment (0,-100)→(0,0).
        // Truck bei (0,0,-50) mit Heading 0° (Nord) → Soll Nord-Segment wählen.
        let north_seg = make_seg(0.0, 0.0, 0.0, -100.0);
        let south_seg = make_seg(0.0, -100.0, 0.0, 0.0);
        let idx = build_index(vec![north_seg, south_seg]);
        let hit = idx
            .nearest_with_heading_filter(Vec3::new(0.0, 0.0, -50.0), 0.0, 4)
            .expect("must find a hit");
        assert!(
            hit.heading_filter_applied,
            "filter must apply when heading-aligned segment exists"
        );
        assert!(
            hit.heading_deg < 10.0 || hit.heading_deg > 350.0,
            "expected heading ≈ 0° (North), got {:.1}°",
            hit.heading_deg
        );
    }

    #[test]
    fn heading_filter_falls_back_when_all_candidates_opposite() {
        // Nur ein Süd-Segment; Truck fährt Nord → Filter lehnt alle ab → Fallback.
        let south_seg = make_seg(0.0, -100.0, 0.0, 0.0);
        let idx = build_index(vec![south_seg]);
        let hit = idx
            .nearest_with_heading_filter(Vec3::new(0.0, 0.0, -50.0), 0.0, 4)
            .expect("must find a hit via fallback");
        assert!(
            !hit.heading_filter_applied,
            "filter_applied must be false when all candidates rejected"
        );
    }

    // --- nearest_with_projection_filtered Tests (DS13d) ---

    fn prefab_meta() -> Option<SegmentMetadata> {
        Some(SegmentMetadata {
            lanes_in_direction: 1,
            lanes_opposite: 0,
            lanes_total: 1,
            lane_width_m: 3.75,
            lane_offset_right_m: 0.0,
            road_look_token: 0,
            is_prefab: true,
        })
    }

    fn road_meta() -> Option<SegmentMetadata> {
        Some(SegmentMetadata {
            lanes_in_direction: 1,
            lanes_opposite: 0,
            lanes_total: 1,
            lane_width_m: 3.75,
            lane_offset_right_m: 1.875,
            road_look_token: 0,
            is_prefab: false,
        })
    }

    /// DS13d TASK 6a: Bias-Query picks prefab segment when it is the closest qualifying entry.
    #[test]
    fn test_geometric_bias_picks_prefab_in_junction() {
        // Layout: road at z=0 (10m from query), prefab at z=-3 (3m from query).
        // Unfiltered nearest would be prefab (closer), but let's also test that
        // the filter correctly returns only the prefab-tagged segment.
        let road_seg = make_seg(0.0, 0.0, 20.0, 0.0);
        let prefab_seg = make_seg(0.0, -3.0, 20.0, -3.0);
        let segs = vec![road_seg, prefab_seg];
        let meta = vec![road_meta(), prefab_meta()];
        let idx = build_index_with_metadata(segs, meta);

        let query = Vec3::new(10.0, 0.0, -3.0); // on the prefab segment
        let hit = idx
            .nearest_with_projection_filtered(query, 32, |_, m| m.is_some_and(|m| m.is_prefab))
            .expect("must find prefab hit");
        assert_eq!(hit.segment_idx, 1, "prefab segment is at index 1");
        assert!(hit.dist_m < 0.5, "query is on prefab → dist ≈ 0, got {}", hit.dist_m);
    }

    /// DS13d TASK 6b: Filter returns None when no prefab in candidate range.
    #[test]
    fn test_geometric_bias_rejects_far_prefab() {
        // Road at z=0, prefab far away at z=-200 (beyond a tight candidate window).
        // With candidates=4, the tree returns the 4 closest entries by AABB;
        // if the prefab AABB is far, it won't appear → filter returns None.
        let road_seg = make_seg(0.0, 0.0, 20.0, 0.0);
        let far_prefab = make_seg(0.0, -200.0, 20.0, -200.0);
        let segs = vec![road_seg, far_prefab];
        let meta = vec![road_meta(), prefab_meta()];
        let idx = build_index_with_metadata(segs, meta);

        let query = Vec3::new(10.0, 0.0, 0.0); // on the road segment
        // With a tight candidates=1, the tree returns only the closest (road), prefab is skipped.
        let hit = idx.nearest_with_projection_filtered(query, 1, |_, m| m.is_some_and(|m| m.is_prefab));
        assert!(hit.is_none(), "no prefab within candidate window → None");
    }

    /// DS13d TASK 6c: Without predicate filtering, both road and prefab are candidates
    /// and the geometrically closest wins normally.
    #[test]
    fn test_unfiltered_nearest_picks_road_when_road_is_closer() {
        // Road at z=0 (2m from query), prefab at z=-10 (8m from query).
        let road_seg = make_seg(0.0, 0.0, 20.0, 0.0);
        let prefab_seg = make_seg(0.0, -10.0, 20.0, -10.0);
        let segs = vec![road_seg, prefab_seg];
        let meta = vec![road_meta(), prefab_meta()];
        let idx = build_index_with_metadata(segs, meta);

        let query = Vec3::new(10.0, 0.0, -2.0); // 2m from road
        let hit = idx.nearest_with_projection(query, 8).expect("must find hit");
        assert_eq!(hit.segment_idx, 0, "road is closer → unfiltered query picks road");
        assert!(hit.dist_m < 3.0, "dist should be ~2m, got {}", hit.dist_m);
    }

    // --- within_radius_with_idx Tests (Phase 6.9 HUD Overlay) ---

    /// within_radius_with_idx returns segment indices alongside geometry.
    #[test]
    fn within_radius_with_idx_returns_indices() {
        let idx = toy_index();
        // Query near segment 0 ((0,0)→(10,0)) with 2m radius
        let results = idx.within_radius_with_idx(Vec3::new(5.0, 0.0, 0.0), 2.0);
        assert!(
            !results.is_empty(),
            "Should find at least one segment within 2m of (5,0)"
        );
        // All returned indices must be valid
        for (seg_idx, seg, _meta) in &results {
            assert!(
                *seg_idx < idx.segments.len(),
                "idx {seg_idx} out of bounds"
            );
            // The returned reference must match the indexed segment
            assert!(
                std::ptr::eq(*seg, &idx.segments[*seg_idx]),
                "returned segment ref must match segments[idx]"
            );
        }
    }

    /// within_radius_with_idx returns metadata when available.
    #[test]
    fn within_radius_with_idx_returns_metadata() {
        let road_seg = make_seg(0.0, 0.0, 20.0, 0.0);
        let prefab_seg = make_seg(0.0, -5.0, 20.0, -5.0);
        let segs = vec![road_seg, prefab_seg];
        let meta = vec![road_meta(), prefab_meta()];
        let idx = build_index_with_metadata(segs, meta);

        // Query near both segments (y=0 and y=-5, radius 10m covers both)
        let results = idx.within_radius_with_idx(Vec3::new(10.0, 0.0, -2.5), 10.0);
        assert_eq!(results.len(), 2, "both segments within 10m");

        // Each segment should have the correct is_prefab flag
        for (seg_idx, _seg, meta) in &results {
            let expected_prefab = *seg_idx == 1;
            let got_prefab = meta.map(|m| m.is_prefab).unwrap_or(false);
            assert_eq!(
                got_prefab, expected_prefab,
                "segment {seg_idx}: expected is_prefab={expected_prefab}, got {got_prefab}"
            );
        }
    }

    /// within_radius_with_idx respects the radius bound.
    #[test]
    fn within_radius_with_idx_excludes_far_segments() {
        let idx = toy_index();
        // Only segment 0 ((0,0)→(10,0)) is within 2m of (5,0)
        let results = idx.within_radius_with_idx(Vec3::new(5.0, 0.0, 0.0), 2.0);
        let far_count = results.iter().filter(|(_, s, _)| s.p0.x > 15.0).count();
        assert_eq!(
            far_count, 0,
            "no segment starting at x>15 should be within 2m of x=5"
        );
    }
}
