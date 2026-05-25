//! Phase 0b — Hermite Spline Generator
//!
//! Erzeugt Hermite-Segmente aus Edge-Geometrie des Routing-Graphen.
//! Tangenten-Quelle: `GraphNode.rotation` Quaternion (DS12); Fallback auf
//! gewichteten Edge-Richtungs-Durchschnitt wenn Quaternion null ist (sized-format nodes).
//!
//! # Übersicht
//! 1. [`build_splines`] — Haupteinstieg: MapGraph → Vec<HermiteSegment>
//! 2. [`HermiteSegment`] — ein Segment (p0, p1, m0, m1)
//! 3. [`evaluate`] / [`evaluate_tangent`] — Hermite-Auswertung bei t ∈ [0,1]

use std::collections::HashMap;

use serde::{Deserialize, Serialize};

use crate::graph::{GraphEdge, MapGraph};

// ---------------------------------------------------------------------------
// Konstanten
// ---------------------------------------------------------------------------

/// Skalierungsfaktor für Tangenten-Magnitude (Catmull-Rom-artig).
/// 1.0 = |p1 − p0| (Standardwert).
/// < 1.0 → weichere Kurven; > 1.0 → aggressivere Überschwinger.
pub const TANGENT_SCALE: f32 = 1.0;

// ---------------------------------------------------------------------------
// Datenmodell
// ---------------------------------------------------------------------------

/// Ein Hermite-Spline-Segment zwischen zwei Knoten.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HermiteSegment {
    /// Startpunkt (from_node.position)
    pub p0: Vec3,
    /// Endpunkt (to_node.position)
    pub p1: Vec3,
    /// Tangente am Startpunkt (Richtung + Magnitude)
    pub m0: Vec3,
    /// Tangente am Endpunkt
    pub m1: Vec3,
    /// Euklidische Länge der direkten Verbindung p0→p1 in Metern
    pub length_m: f32,
    /// UID des from-Node
    pub from_uid: u64,
    /// UID des to-Node
    pub to_uid: u64,
    /// UID der originalen Edge
    pub edge_uid: u64,
}

/// Einfacher 3D-Vektor (f32, ETS2-Koordinatensystem: x=Ost, y=Höhe, z=Süd).
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    #[inline]
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    #[inline]
    pub fn length(self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Normiert den Vektor. Gibt (0,0,0) zurück wenn near-zero.
    #[inline]
    pub fn normalize(self) -> Self {
        let len = self.length();
        if len < 1e-9 {
            Self::default()
        } else {
            Self::new(self.x / len, self.y / len, self.z / len)
        }
    }

    #[inline]
    pub fn scale(self, s: f32) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }

    /// Gibt `atan2(dx, -dz)` in Radiant zurück (Spec §1.3: Heading aus Edge-Geometrie).
    /// 0 = Nord, π/2 = Ost, positiv im Uhrzeigersinn.
    #[inline]
    pub fn edge_heading_rad(from: Self, to: Self) -> f32 {
        let dx = to.x - from.x;
        let dz = to.z - from.z;
        f32::atan2(dx, -dz)
    }
}

impl std::ops::Add for Vec3 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl std::ops::Sub for Vec3 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl std::ops::Mul<f32> for Vec3 {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self {
        Vec3::scale(self, rhs)
    }
}

impl std::ops::Mul<Vec3> for f32 {
    type Output = Vec3;
    fn mul(self, rhs: Vec3) -> Vec3 {
        Vec3::scale(rhs, self)
    }
}

// ---------------------------------------------------------------------------
// Quaternion-Hilfsfunktionen
// ---------------------------------------------------------------------------

/// Returns true if the quaternion has a non-zero magnitude (i.e., a valid rotation).
/// `[0.0; 4]` is the sentinel for "not set" (sized-format nodes).
#[inline]
fn quat_is_set(q: [f32; 4]) -> bool {
    q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] > 1e-6
}

/// Rotates vector `v` by unit quaternion `q = [qw, qx, qy, qz]` (w-first, ETS2 convention).
///
/// Uses the Rodrigues formula: `t = 2·(q_xyz × v); v' = v + qw·t + q_xyz × t`
#[inline]
pub fn quat_rotate_vec(q: [f32; 4], v: Vec3) -> Vec3 {
    let (qw, qx, qy, qz) = (q[0], q[1], q[2], q[3]);
    let tx = 2.0 * (qy * v.z - qz * v.y);
    let ty = 2.0 * (qz * v.x - qx * v.z);
    let tz = 2.0 * (qx * v.y - qy * v.x);
    Vec3::new(
        v.x + qw * tx + qy * tz - qz * ty,
        v.y + qw * ty + qz * tx - qx * tz,
        v.z + qw * tz + qx * ty - qy * tx,
    )
}

// ---------------------------------------------------------------------------
// Tangenten-Ableitung
// ---------------------------------------------------------------------------

/// Interne Hilfsstruktur: alle Edge-Richtungen pro Node.
struct NodeAdjacency {
    /// (direction_vec, length) — unormiert, schon auf Einheitslänge normiert wird später.
    edges: Vec<(Vec3, f32)>,
}

/// Baut einen Adjacency-Index: node_uid → alle angrenzenden Edge-Richtungen.
///
/// Pro Edge (from→to) tragen wir am from-Node die Richtung to−from ein,
/// am to-Node die Richtung from−to (Eingangsrichtung).
/// Damit bekommen wir am jeder Node alle tangentialen Richtungen.
fn build_adjacency(nodes: &HashMap<u64, Vec3>, edges: &[GraphEdge]) -> HashMap<u64, NodeAdjacency> {
    let mut adj: HashMap<u64, NodeAdjacency> = HashMap::with_capacity(nodes.len());

    for edge in edges {
        let (Some(&p0), Some(&p1)) = (nodes.get(&edge.from), nodes.get(&edge.to)) else {
            continue;
        };

        let dir_fwd = p1 - p0; // Richtung from→to
        let len = dir_fwd.length();
        if len < 1e-4 {
            continue; // Degenerate edge (Nulllänge)
        }

        // Am from-Node: ausgehende Richtung (fwd)
        adj.entry(edge.from)
            .or_insert_with(|| NodeAdjacency { edges: Vec::new() })
            .edges
            .push((dir_fwd, len));

        // Am to-Node: einkommende Richtung (wir tragen den Gegenvektor ein,
        // so dass auch am Endknoten die Fahrtrichtung ankommt)
        let dir_bwd = p0 - p1; // Richtung to→from (eintretender Vektor gespiegelt)
        adj.entry(edge.to)
            .or_insert_with(|| NodeAdjacency { edges: Vec::new() })
            .edges
            .push((dir_bwd, len));
    }

    adj
}

/// Berechnet die gemittelte Tangente an einem Node.
///
/// Strategie: Gewichteter Durchschnitt aller angrenzenden Edge-Richtungen,
/// Gewicht = Länge der Edge (längere Edges prägen den Tangenten-Charakter stärker).
/// Resultat wird normiert, damit TANGENT_SCALE die Magnitude kontrolliert.
///
/// Edge-Cases:
/// - 0 Edges (isolierter Node): gibt (0,0,0) zurück.
/// - 1 Edge: gibt genau diese Edge-Richtung zurück (normiert).
/// - n Edges: gewichtetes Mittel.
fn node_tangent(adj: &NodeAdjacency, magnitude: f32) -> Vec3 {
    if adj.edges.is_empty() {
        return Vec3::default();
    }

    let mut sum = Vec3::default();
    let mut total_weight = 0.0f32;

    for &(dir, len) in &adj.edges {
        let weight = len.max(1e-6);
        // Normiere zuerst, dann gewichte — verhindert, dass eine einzelne
        // sehr kurze Edge das Ergebnis dominiert.
        let unit = dir.normalize();
        sum = sum + unit * weight;
        total_weight += weight;
    }

    if total_weight < 1e-9 {
        return Vec3::default();
    }

    // Normierter Durchschnittsvektor × gewünschte Magnitude
    (sum * (1.0 / total_weight)).normalize() * magnitude
}

// ---------------------------------------------------------------------------
// Hauptfunktion
// ---------------------------------------------------------------------------

/// Per-Segment metadata derived from the originating GraphEdge (DS8).
///
/// Present only for road edges (`"forward"`, `"backward"`, `"bidirectional_unknown"`).
/// `None` for prefab, building, ferry, and cross-sector edges.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SegmentMetadata {
    /// Number of lanes travelling in this direction.
    pub lanes_in_direction: u8,
    /// Number of lanes in the opposite direction on the same road.
    pub lanes_opposite: u8,
    /// Total lanes on the road cross-section.
    pub lanes_total: u8,
    /// Lane width in metres from the road-look definition.
    pub lane_width_m: f32,
    /// Right-of-centreline offset: `(lanes_in_direction − 0.5) × lane_width_m`.
    pub lane_offset_right_m: f32,
    /// Road-look token64 for this direction (0 = unknown).
    pub road_look_token: u64,
}

/// Baut alle Hermite-Segmente aus dem MapGraph und liefert pro Segment optionale Metadaten.
///
/// Gibt `(segmente, metadaten, stats)` zurück.  `metadaten[i]` entspricht `segmente[i]`;
/// road-Edges (`"forward"` / `"backward"` / `"bidirectional_unknown"`) liefern `Some(SegmentMetadata)`,
/// alle anderen `None`.
pub fn build_splines_ex(
    graph: &MapGraph,
) -> (Vec<HermiteSegment>, Vec<Option<SegmentMetadata>>, SplineStats) {
    let node_map: HashMap<u64, Vec3> = graph
        .nodes
        .iter()
        .map(|n| (n.uid, Vec3::new(n.x as f32, n.y as f32, n.z as f32)))
        .collect();

    let rotation_map: HashMap<u64, [f32; 4]> = graph
        .nodes
        .iter()
        .map(|n| (n.uid, n.rotation))
        .collect();

    let adj = build_adjacency(&node_map, &graph.edges);
    let neighbor_degree = build_neighbor_degree(&graph.edges);

    let mut segments = Vec::with_capacity(graph.edges.len());
    let mut metadata: Vec<Option<SegmentMetadata>> = Vec::with_capacity(graph.edges.len());
    let mut stats = SplineStats::default();

    for edge in &graph.edges {
        let (Some(&p0), Some(&p1)) = (node_map.get(&edge.from), node_map.get(&edge.to)) else {
            stats.skipped_missing_node += 1;
            continue;
        };

        let chord_len = (p1 - p0).length();
        if chord_len < 1e-4 {
            stats.skipped_degenerate += 1;
            continue;
        }

        let mag = chord_len * TANGENT_SCALE;

        let m0 = {
            let q = rotation_map.get(&edge.from).copied().unwrap_or([0.0; 4]);
            if quat_is_set(q) {
                stats.quat_tangents += 1;
                quat_rotate_vec(q, Vec3::new(0.0, 0.0, -mag))
            } else {
                stats.fallback_tangents += 1;
                match adj.get(&edge.from) {
                    Some(a) => node_tangent(a, mag),
                    None => (p1 - p0).normalize() * mag,
                }
            }
        };

        let m1 = {
            let q = rotation_map.get(&edge.to).copied().unwrap_or([0.0; 4]);
            if quat_is_set(q) {
                stats.quat_tangents += 1;
                quat_rotate_vec(q, Vec3::new(0.0, 0.0, -mag))
            } else {
                stats.fallback_tangents += 1;
                match adj.get(&edge.to) {
                    Some(a) => node_tangent(a, mag),
                    None => (p1 - p0).normalize() * mag,
                }
            }
        };

        let deg_from = neighbor_degree.get(&edge.from).copied().unwrap_or(0);
        let deg_to = neighbor_degree.get(&edge.to).copied().unwrap_or(0);
        if deg_from == 1 || deg_to == 1 {
            stats.end_nodes += 1;
        }
        if deg_from > 2 || deg_to > 2 {
            stats.high_degree_junctions += 1;
        }

        let m0_ratio = m0.length() / chord_len.max(1.0);
        let m1_ratio = m1.length() / chord_len.max(1.0);
        if m0_ratio > 3.0 || m1_ratio > 3.0 {
            stats.pathological_tangents += 1;
        }

        segments.push(HermiteSegment {
            p0,
            p1,
            m0,
            m1,
            length_m: chord_len,
            from_uid: edge.from,
            to_uid: edge.to,
            edge_uid: edge.uid,
        });

        let seg_meta = match edge.direction.as_str() {
            "forward" | "backward" | "bidirectional_unknown" => {
                let lanes = edge.lanes.max(1);
                Some(SegmentMetadata {
                    lanes_in_direction: lanes,
                    lanes_opposite: edge.lanes_opposite,
                    lanes_total: lanes.saturating_add(edge.lanes_opposite),
                    lane_width_m: edge.lane_width_m,
                    lane_offset_right_m: (lanes as f32 - 0.5) * edge.lane_width_m,
                    road_look_token: edge.road_look_token,
                })
            }
            _ => None,
        };
        metadata.push(seg_meta);
    }

    stats.total_segments = segments.len();
    (segments, metadata, stats)
}

/// Baut alle Hermite-Segmente aus dem MapGraph.
///
/// Gibt `(segmente, stats)` zurück, wobei `stats` Debug-Infos enthält.
/// Wrapper um [`build_splines_ex`] — drop die Metadaten.
pub fn build_splines(graph: &MapGraph) -> (Vec<HermiteSegment>, SplineStats) {
    let (segs, _, stats) = build_splines_ex(graph);
    (segs, stats)
}

/// Baut Splines für Nodes in einem Bounding-Box-Filter.
/// Nur Edges wo BEIDE Nodes im BBox liegen werden berücksichtigt.
pub fn build_splines_bbox(graph: &MapGraph, bbox: BBox) -> (Vec<HermiteSegment>, SplineStats) {
    // Filtere Node-Map auf BBox
    let node_map: HashMap<u64, Vec3> = graph
        .nodes
        .iter()
        .filter(|n| bbox.contains(n.x as f32, n.z as f32))
        .map(|n| (n.uid, Vec3::new(n.x as f32, n.y as f32, n.z as f32)))
        .collect();

    // Rotation-Map: uid → [qw, qx, qy, qz] (alle Nodes, Lookup miss = zero sentinel)
    let rotation_map: HashMap<u64, [f32; 4]> = graph
        .nodes
        .iter()
        .map(|n| (n.uid, n.rotation))
        .collect();

    // Filtere Edges: beide Nodes in BBox
    let bbox_edges: Vec<_> = graph
        .edges
        .iter()
        .filter(|e| node_map.contains_key(&e.from) && node_map.contains_key(&e.to))
        .collect();

    // Adjacency nur auf gefilterten Edges
    let adj = build_adjacency_filtered(&node_map, &bbox_edges);
    let bbox_edge_owned: Vec<GraphEdge> = bbox_edges.iter().map(|e| (*e).clone()).collect();
    let neighbor_degree = build_neighbor_degree(&bbox_edge_owned);

    let mut segments = Vec::new();
    let mut stats = SplineStats::default();

    for edge in &bbox_edges {
        let (&p0, &p1) = (
            node_map.get(&edge.from).unwrap(),
            node_map.get(&edge.to).unwrap(),
        );
        let chord_len = (p1 - p0).length();
        if chord_len < 1e-4 {
            stats.skipped_degenerate += 1;
            continue;
        }

        let mag = chord_len * TANGENT_SCALE;

        // m0: quaternion-derived if set, else weighted-average fallback
        let m0 = {
            let q = rotation_map.get(&edge.from).copied().unwrap_or([0.0; 4]);
            if quat_is_set(q) {
                stats.quat_tangents += 1;
                quat_rotate_vec(q, Vec3::new(0.0, 0.0, -mag))
            } else {
                stats.fallback_tangents += 1;
                match adj.get(&edge.from) {
                    Some(a) => node_tangent(a, mag),
                    None => (p1 - p0).normalize() * mag,
                }
            }
        };

        // m1: quaternion-derived if set, else weighted-average fallback
        let m1 = {
            let q = rotation_map.get(&edge.to).copied().unwrap_or([0.0; 4]);
            if quat_is_set(q) {
                stats.quat_tangents += 1;
                quat_rotate_vec(q, Vec3::new(0.0, 0.0, -mag))
            } else {
                stats.fallback_tangents += 1;
                match adj.get(&edge.to) {
                    Some(a) => node_tangent(a, mag),
                    None => (p1 - p0).normalize() * mag,
                }
            }
        };

        let deg_from = neighbor_degree.get(&edge.from).copied().unwrap_or(0);
        let deg_to = neighbor_degree.get(&edge.to).copied().unwrap_or(0);
        if deg_from == 1 || deg_to == 1 {
            stats.end_nodes += 1;
        }
        if deg_from > 2 || deg_to > 2 {
            stats.high_degree_junctions += 1;
        }
        let m0_ratio = m0.length() / chord_len.max(1.0);
        let m1_ratio = m1.length() / chord_len.max(1.0);
        if m0_ratio > 3.0 || m1_ratio > 3.0 {
            stats.pathological_tangents += 1;
        }

        segments.push(HermiteSegment {
            p0,
            p1,
            m0,
            m1,
            length_m: chord_len,
            from_uid: edge.from,
            to_uid: edge.to,
            edge_uid: edge.uid,
        });
    }

    stats.total_segments = segments.len();
    (segments, stats)
}

/// Adjacency-Builder für gefilterte Edge-Slice (BBox-Variante).
fn build_adjacency_filtered(
    nodes: &HashMap<u64, Vec3>,
    edges: &[&GraphEdge],
) -> HashMap<u64, NodeAdjacency> {
    let mut adj: HashMap<u64, NodeAdjacency> = HashMap::new();
    for edge in edges {
        let (Some(&p0), Some(&p1)) = (nodes.get(&edge.from), nodes.get(&edge.to)) else {
            continue;
        };
        let dir_fwd = p1 - p0;
        let len = dir_fwd.length();
        if len < 1e-4 {
            continue;
        }
        adj.entry(edge.from)
            .or_insert_with(|| NodeAdjacency { edges: Vec::new() })
            .edges
            .push((dir_fwd, len));
        adj.entry(edge.to)
            .or_insert_with(|| NodeAdjacency { edges: Vec::new() })
            .edges
            .push((p0 - p1, len));
    }
    adj
}
/// Baut einen Nachbar-Degree-Index für Stats-Reporting.
///
/// Zählt pro Node die Anzahl *eindeutiger* Nachbar-Nodes (ungerichtet).
/// Getrennt vom Adjacency-Index für Tangenten, um korrekte Junction-Grade zu messen.
///
/// Bidirektionale Edges (A→B + B→A) zählen als ein Nachbar, nicht zwei.
fn build_neighbor_degree(edges: &[GraphEdge]) -> HashMap<u64, usize> {
    // uid → Set<Nachbar-uid>
    let mut neighbors: HashMap<u64, std::collections::HashSet<u64>> = HashMap::new();
    for edge in edges {
        if edge.from == edge.to {
            continue; // Selbst-Loop ignorieren
        }
        neighbors.entry(edge.from).or_default().insert(edge.to);
        neighbors.entry(edge.to).or_default().insert(edge.from);
    }
    neighbors
        .into_iter()
        .map(|(uid, set)| (uid, set.len()))
        .collect()
}

// ---------------------------------------------------------------------------
// Hermite-Evaluation (Task 3)
// ---------------------------------------------------------------------------

/// Hermite-Basisfunktionen — Partition of Unity: h00 + h10 + h01 + h11 ≠ 1
/// (h10 und h11 sind Tangenten-Blending, keine echten Positions-Basis-fkt).
/// Für Positions-Basis gilt: h00 + h01 = 1 für alle t.
///
/// Cubic Hermite:
///   p(t) = h00·p0 + h10·m0 + h01·p1 + h11·m1
///
/// Basisfunktionen:
///   h00(t) = 2t³ − 3t² + 1
///   h10(t) = t³  − 2t² + t
///   h01(t) = −2t³ + 3t²
///   h11(t) = t³  − t²
#[inline]
pub fn hermite_basis(t: f32) -> (f32, f32, f32, f32) {
    let t2 = t * t;
    let t3 = t2 * t;
    let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    let h10 = t3 - 2.0 * t2 + t;
    let h01 = -2.0 * t3 + 3.0 * t2;
    let h11 = t3 - t2;
    (h00, h10, h01, h11)
}

/// Hermite-Ableitungs-Basisfunktionen (für Tangenten-Evaluation).
///
///   h00'(t) = 6t² − 6t
///   h10'(t) = 3t² − 4t + 1
///   h01'(t) = −6t² + 6t
///   h11'(t) = 3t² − 2t
#[inline]
pub fn hermite_basis_derivative(t: f32) -> (f32, f32, f32, f32) {
    let t2 = t * t;
    let dh00 = 6.0 * t2 - 6.0 * t;
    let dh10 = 3.0 * t2 - 4.0 * t + 1.0;
    let dh01 = -6.0 * t2 + 6.0 * t;
    let dh11 = 3.0 * t2 - 2.0 * t;
    (dh00, dh10, dh01, dh11)
}

/// Wertet die Hermite-Kurve bei Parameter `t` ∈ [0, 1] aus.
///
/// - t = 0 → p0 (exakt)
/// - t = 1 → p1 (exakt)
pub fn evaluate(seg: &HermiteSegment, t: f32) -> Vec3 {
    let (h00, h10, h01, h11) = hermite_basis(t);
    seg.p0 * h00 + seg.m0 * h10 + seg.p1 * h01 + seg.m1 * h11
}

/// Wertet die Tangente (1. Ableitung) der Hermite-Kurve bei `t` aus.
/// Nicht normiert — gibt die Geschwindigkeitsvektorrichtung.
pub fn evaluate_tangent(seg: &HermiteSegment, t: f32) -> Vec3 {
    let (dh00, dh10, dh01, dh11) = hermite_basis_derivative(t);
    seg.p0 * dh00 + seg.m0 * dh10 + seg.p1 * dh01 + seg.m1 * dh11
}

/// Gibt den Heading-Winkel in Grad (0=Nord, CW) der Tangente bei t zurück.
pub fn evaluate_heading_deg(seg: &HermiteSegment, t: f32) -> f32 {
    let tan = evaluate_tangent(seg, t);
    f32::atan2(tan.x, -tan.z).to_degrees().rem_euclid(360.0)
}

// ---------------------------------------------------------------------------
// Hilfs-Typen
// ---------------------------------------------------------------------------

/// Axis-Aligned Bounding Box im (x, z)-Koordinatensystem.
#[derive(Debug, Clone, Copy)]
pub struct BBox {
    pub x_min: f32,
    pub z_min: f32,
    pub x_max: f32,
    pub z_max: f32,
}

impl BBox {
    pub fn new(x_min: f32, z_min: f32, x_max: f32, z_max: f32) -> Self {
        Self {
            x_min,
            z_min,
            x_max,
            z_max,
        }
    }

    #[inline]
    pub fn contains(&self, x: f32, z: f32) -> bool {
        x >= self.x_min && x <= self.x_max && z >= self.z_min && z <= self.z_max
    }
}

/// Statistik-Output von build_splines.
#[derive(Debug, Clone, Default)]
pub struct SplineStats {
    pub total_segments: usize,
    pub skipped_missing_node: usize,
    pub skipped_degenerate: usize,
    pub end_nodes: usize,
    pub high_degree_junctions: usize,
    pub pathological_tangents: usize,
    /// Tangenten die aus Node-Quaternion berechnet wurden (DS12).
    pub quat_tangents: usize,
    /// Tangenten die auf Edge-Richtungs-Fallback zurückgefallen sind (sized-format nodes).
    pub fallback_tangents: usize,
}

impl SplineStats {
    pub fn print_summary(&self) {
        println!("=== SplineStats ===");
        println!("  Segmente gesamt:        {}", self.total_segments);
        println!("  Übersprungen (Node):    {}", self.skipped_missing_node);
        println!("  Übersprungen (Degener): {}", self.skipped_degenerate);
        println!("  End-Nodes (deg=1):      {}", self.end_nodes);
        println!("  High-Degree Junctions:  {}", self.high_degree_junctions);
        println!("  Pathologische Tangenten:{}", self.pathological_tangents);
        println!("  Quat-Tangenten:         {}", self.quat_tangents);
        println!("  Fallback-Tangenten:     {}", self.fallback_tangents);
        let bytes = self.total_segments * std::mem::size_of::<HermiteSegment>();
        println!(
            "  Memory (approx):        {:.1} MB ({} bytes/segment)",
            bytes as f64 / 1_048_576.0,
            std::mem::size_of::<HermiteSegment>()
        );
    }
}

// ---------------------------------------------------------------------------
// Unit-Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: Segment von (0,0,0) nach (1,0,0) mit Catmull-Rom-Tangenten.
    fn straight_seg() -> HermiteSegment {
        HermiteSegment {
            p0: Vec3::new(0.0, 0.0, 0.0),
            p1: Vec3::new(1.0, 0.0, 0.0),
            m0: Vec3::new(1.0, 0.0, 0.0),
            m1: Vec3::new(1.0, 0.0, 0.0),
            length_m: 1.0,
            from_uid: 1,
            to_uid: 2,
            edge_uid: 10,
        }
    }

    // --- Basisfunktionen ---

    #[test]
    fn basis_at_t0() {
        let (h00, h10, h01, h11) = hermite_basis(0.0);
        assert!((h00 - 1.0).abs() < 1e-6, "h00(0) = 1");
        assert!((h10 - 0.0).abs() < 1e-6, "h10(0) = 0");
        assert!((h01 - 0.0).abs() < 1e-6, "h01(0) = 0");
        assert!((h11 - 0.0).abs() < 1e-6, "h11(0) = 0");
    }

    #[test]
    fn basis_at_t1() {
        let (h00, h10, h01, h11) = hermite_basis(1.0);
        assert!((h00 - 0.0).abs() < 1e-6, "h00(1) = 0");
        assert!((h10 - 0.0).abs() < 1e-6, "h10(1) = 0");
        assert!((h01 - 1.0).abs() < 1e-6, "h01(1) = 1");
        assert!((h11 - 0.0).abs() < 1e-6, "h11(1) = 0");
    }

    #[test]
    fn basis_at_t05() {
        let (h00, h10, h01, h11) = hermite_basis(0.5);
        // h00(0.5) = 2*0.125 - 3*0.25 + 1 = 0.25 - 0.75 + 1 = 0.5
        assert!((h00 - 0.5).abs() < 1e-6, "h00(0.5) = 0.5, got {h00}");
        // h01(0.5) = -2*0.125 + 3*0.25 = -0.25 + 0.75 = 0.5
        assert!((h01 - 0.5).abs() < 1e-6, "h01(0.5) = 0.5, got {h01}");
        // Positions-Partition: h00 + h01 = 1 für alle t
        assert!((h00 + h01 - 1.0).abs() < 1e-6, "h00+h01=1 at 0.5");
        let _ = (h10, h11); // tangent basis, nur zur Vollständigkeit
    }

    #[test]
    fn position_partition_of_unity() {
        // h00(t) + h01(t) = 1 für alle t — das ist die echte Partition of Unity
        for i in 0..=100 {
            let t = i as f32 / 100.0;
            let (h00, _h10, h01, _h11) = hermite_basis(t);
            assert!(
                (h00 + h01 - 1.0).abs() < 1e-5,
                "h00+h01 ≠ 1 at t={t}: {h00} + {h01}"
            );
        }
    }

    #[test]
    fn evaluate_endpoints() {
        let seg = straight_seg();
        let p_at_0 = evaluate(&seg, 0.0);
        let p_at_1 = evaluate(&seg, 1.0);
        assert!((p_at_0.x - 0.0).abs() < 1e-6, "p(0)=p0.x");
        assert!((p_at_0.z - 0.0).abs() < 1e-6, "p(0)=p0.z");
        assert!((p_at_1.x - 1.0).abs() < 1e-6, "p(1)=p1.x");
        assert!((p_at_1.z - 0.0).abs() < 1e-6, "p(1)=p1.z");
    }

    #[test]
    fn evaluate_midpoint_straight_line() {
        // Für gleiche Tangenten und gerade Linie muss der Mittelpunkt exakt auf 0.5 liegen.
        let seg = straight_seg();
        let mid = evaluate(&seg, 0.5);
        assert!((mid.x - 0.5).abs() < 1e-5, "midpoint x=0.5, got {}", mid.x);
        assert!((mid.y - 0.0).abs() < 1e-5, "midpoint y=0");
        assert!((mid.z - 0.0).abs() < 1e-5, "midpoint z=0");
    }

    #[test]
    fn tangent_at_t0_equals_m0() {
        let seg = straight_seg();
        let tan = evaluate_tangent(&seg, 0.0);
        // Ableitung: dh00(0)=0, dh10(0)=1, dh01(0)=0, dh11(0)=0 → tan = m0
        assert!((tan.x - seg.m0.x).abs() < 1e-6, "tan(0)=m0.x");
        assert!((tan.y - seg.m0.y).abs() < 1e-6, "tan(0)=m0.y");
        assert!((tan.z - seg.m0.z).abs() < 1e-6, "tan(0)=m0.z");
    }

    #[test]
    fn tangent_at_t1_equals_m1() {
        let seg = straight_seg();
        let tan = evaluate_tangent(&seg, 1.0);
        // dh00(1)=0, dh10(1)=0, dh01(1)=0, dh11(1)=1 → tan = m1
        assert!((tan.x - seg.m1.x).abs() < 1e-6, "tan(1)=m1.x");
        assert!((tan.y - seg.m1.y).abs() < 1e-6, "tan(1)=m1.y");
        assert!((tan.z - seg.m1.z).abs() < 1e-6, "tan(1)=m1.z");
    }

    #[test]
    fn derivative_basis_at_t0() {
        let (dh00, dh10, dh01, dh11) = hermite_basis_derivative(0.0);
        assert!((dh00 - 0.0).abs() < 1e-6, "dh00(0)=0");
        assert!((dh10 - 1.0).abs() < 1e-6, "dh10(0)=1");
        assert!((dh01 - 0.0).abs() < 1e-6, "dh01(0)=0");
        assert!((dh11 - 0.0).abs() < 1e-6, "dh11(0)=0");
    }

    #[test]
    fn derivative_basis_at_t1() {
        let (dh00, dh10, dh01, dh11) = hermite_basis_derivative(1.0);
        assert!((dh00 - 0.0).abs() < 1e-6, "dh00(1)=0");
        assert!((dh10 - 0.0).abs() < 1e-6, "dh10(1)=0");
        assert!((dh01 - 0.0).abs() < 1e-6, "dh01(1)=0");
        assert!((dh11 - 1.0).abs() < 1e-6, "dh11(1)=1");
    }

    #[test]
    fn curved_seg_stays_between_endpoints() {
        // Segment mit Kurve (Tangenten senkrecht zur Chord)
        let seg = HermiteSegment {
            p0: Vec3::new(0.0, 0.0, 0.0),
            p1: Vec3::new(1.0, 0.0, 1.0),
            m0: Vec3::new(0.0, 0.0, 1.0), // 90° zur Chord
            m1: Vec3::new(1.0, 0.0, 0.0), // 90° zur Chord
            length_m: std::f32::consts::SQRT_2,
            from_uid: 1,
            to_uid: 2,
            edge_uid: 11,
        };
        // Alle Sample-Punkte sollten in einem sinnvollen Bereich liegen
        for i in 0..=20 {
            let t = i as f32 / 20.0;
            let p = evaluate(&seg, t);
            // x und z sollten im Bereich [-0.5, 1.5] liegen (kein Overshooting > 50%)
            assert!(p.x > -0.5 && p.x < 1.5, "x out of range at t={t}: {}", p.x);
            assert!(p.z > -0.5 && p.z < 1.5, "z out of range at t={t}: {}", p.z);
        }
    }

    #[test]
    fn vec3_edge_heading() {
        // Nord: from=(0,0,0) to=(0,0,-1) → atan2(0, 1) = 0
        let from = Vec3::new(0.0, 0.0, 0.0);
        let to_north = Vec3::new(0.0, 0.0, -1.0);
        let hdg = Vec3::edge_heading_rad(from, to_north).to_degrees();
        assert!((hdg - 0.0).abs() < 0.01, "Nord=0°, got {hdg}");

        // Ost: to=(1,0,0) → atan2(1, 0) = 90°
        let to_east = Vec3::new(1.0, 0.0, 0.0);
        let hdg = Vec3::edge_heading_rad(from, to_east).to_degrees();
        assert!((hdg - 90.0).abs() < 0.01, "Ost=90°, got {hdg}");

        // Süd: to=(0,0,1) → atan2(0, -1) = 180°
        let to_south = Vec3::new(0.0, 0.0, 1.0);
        let hdg = Vec3::edge_heading_rad(from, to_south).to_degrees();
        assert!((hdg.abs() - 180.0).abs() < 0.01, "Süd=180°, got {hdg}");
    }

    #[test]
    fn node_tangent_single_edge() {
        // Nur eine Edge → Tangente = Edge-Richtung (normiert × magnitude)
        let adj = NodeAdjacency {
            edges: vec![(Vec3::new(3.0, 0.0, 4.0), 5.0)], // len=5
        };
        let t = node_tangent(&adj, 5.0);
        // Normiert: (0.6, 0, 0.8), skaliert × 5 → (3.0, 0, 4.0)
        assert!((t.x - 3.0).abs() < 1e-5, "x={}", t.x);
        assert!((t.z - 4.0).abs() < 1e-5, "z={}", t.z);
    }

    #[test]
    fn node_tangent_two_opposing_edges_cancel() {
        // Zwei entgegengesetzte Edges gleichem Gewicht → Summe ≈ 0
        let adj = NodeAdjacency {
            edges: vec![
                (Vec3::new(1.0, 0.0, 0.0), 1.0),
                (Vec3::new(-1.0, 0.0, 0.0), 1.0),
            ],
        };
        let t = node_tangent(&adj, 1.0);
        assert!(t.length() < 1e-5, "opposing edges cancel, got {:?}", t);
    }

    #[test]
    fn node_tangent_empty_adj() {
        let adj = NodeAdjacency { edges: vec![] };
        let t = node_tangent(&adj, 1.0);
        assert!(t.length() < 1e-9);
    }

    #[test]
    fn memory_size_reported() {
        let size = std::mem::size_of::<HermiteSegment>();
        // Wir erwarten etwa 17 f32 + 3 u64 = 68 + 24 = 92 Bytes, aligned auf 96
        // Der Test prüft nur, dass der Wert vernünftig ist.
        assert!(size > 0 && size <= 200, "unexpected size: {size}");
        let extrapolated_gb = (size * 1_303_542) as f64 / 1_073_741_824.0;
        // Must be under 1 GB (STOP-Bedingung)
        assert!(
            extrapolated_gb < 1.0,
            "Extrapolierter Memory > 1 GB: {extrapolated_gb:.2} GB — STOP!"
        );
    }
    #[test]
    fn neighbor_degree_t_junction() {
        // T-Junction: Node B verbunden mit A, C, D → degree = 3
        // Edges: A→B, B→C, B→D (gerichtet, aber Degree zählt Nachbarn ungerichtet)
        use crate::graph::GraphEdge;
        let edges = vec![
            GraphEdge {
                uid: 1,
                from: 1,
                to: 2,
                distance_m: 10.0,
                speed_limit_kmh: None,
                lanes: 1,
                direction: "forward".into(),
                dlc_guard: 0,
                is_hidden: false,
                gps_avoid: false,
                road_look_token: 0,
                lanes_opposite: 0,
                lane_width_m: 3.75,
            },
            GraphEdge {
                uid: 2,
                from: 2,
                to: 3,
                distance_m: 10.0,
                speed_limit_kmh: None,
                lanes: 1,
                direction: "forward".into(),
                dlc_guard: 0,
                is_hidden: false,
                gps_avoid: false,
                road_look_token: 0,
                lanes_opposite: 0,
                lane_width_m: 3.75,
            },
            GraphEdge {
                uid: 3,
                from: 2,
                to: 4,
                distance_m: 10.0,
                speed_limit_kmh: None,
                lanes: 1,
                direction: "forward".into(),
                dlc_guard: 0,
                is_hidden: false,
                gps_avoid: false,
                road_look_token: 0,
                lanes_opposite: 0,
                lane_width_m: 3.75,
            },
        ];
        let deg = build_neighbor_degree(&edges);
        assert_eq!(
            deg.get(&2).copied().unwrap_or(0),
            3,
            "T-Junction Node B = deg 3"
        );
        assert_eq!(deg.get(&1).copied().unwrap_or(0), 1, "End-Node A = deg 1");
        assert_eq!(deg.get(&3).copied().unwrap_or(0), 1, "End-Node C = deg 1");
    }

    #[test]
    fn neighbor_degree_4way_junction() {
        // 4-Way: Node B verbunden mit A, C, D, E → degree = 4
        use crate::graph::GraphEdge;
        let edges = vec![
            GraphEdge {
                uid: 1,
                from: 1,
                to: 2,
                distance_m: 10.0,
                speed_limit_kmh: None,
                lanes: 1,
                direction: "forward".into(),
                dlc_guard: 0,
                is_hidden: false,
                gps_avoid: false,
                road_look_token: 0,
                lanes_opposite: 0,
                lane_width_m: 3.75,
            },
            GraphEdge {
                uid: 2,
                from: 2,
                to: 3,
                distance_m: 10.0,
                speed_limit_kmh: None,
                lanes: 1,
                direction: "forward".into(),
                dlc_guard: 0,
                is_hidden: false,
                gps_avoid: false,
                road_look_token: 0,
                lanes_opposite: 0,
                lane_width_m: 3.75,
            },
            GraphEdge {
                uid: 3,
                from: 2,
                to: 4,
                distance_m: 10.0,
                speed_limit_kmh: None,
                lanes: 1,
                direction: "forward".into(),
                dlc_guard: 0,
                is_hidden: false,
                gps_avoid: false,
                road_look_token: 0,
                lanes_opposite: 0,
                lane_width_m: 3.75,
            },
            GraphEdge {
                uid: 4,
                from: 2,
                to: 5,
                distance_m: 10.0,
                speed_limit_kmh: None,
                lanes: 1,
                direction: "forward".into(),
                dlc_guard: 0,
                is_hidden: false,
                gps_avoid: false,
                road_look_token: 0,
                lanes_opposite: 0,
                lane_width_m: 3.75,
            },
        ];
        let deg = build_neighbor_degree(&edges);
        assert_eq!(
            deg.get(&2).copied().unwrap_or(0),
            4,
            "4-Way Junction Node B = deg 4"
        );
    }

    #[test]
    fn neighbor_degree_bidirectional_counted_once() {
        // A→B und B→A (bidirektional) → B hat nur 1 Nachbar A, nicht 2
        use crate::graph::GraphEdge;
        let edges = vec![
            GraphEdge {
                uid: 1,
                from: 1,
                to: 2,
                distance_m: 10.0,
                speed_limit_kmh: None,
                lanes: 1,
                direction: "forward".into(),
                dlc_guard: 0,
                is_hidden: false,
                gps_avoid: false,
                road_look_token: 0,
                lanes_opposite: 0,
                lane_width_m: 3.75,
            },
            GraphEdge {
                uid: 2,
                from: 2,
                to: 1,
                distance_m: 10.0,
                speed_limit_kmh: None,
                lanes: 1,
                direction: "backward".into(),
                dlc_guard: 0,
                is_hidden: false,
                gps_avoid: false,
                road_look_token: 0,
                lanes_opposite: 0,
                lane_width_m: 3.75,
            },
        ];
        let deg = build_neighbor_degree(&edges);
        assert_eq!(
            deg.get(&1).copied().unwrap_or(0),
            1,
            "Bidir: A has 1 unique neighbor B"
        );
        assert_eq!(
            deg.get(&2).copied().unwrap_or(0),
            1,
            "Bidir: B has 1 unique neighbor A"
        );
    }

    // --- DS12 Quaternion-Tangent Tests ---

    #[test]
    fn test_quaternion_to_tangent_north() {
        // Identity quaternion [1,0,0,0] → North direction (0,0,-len) unchanged
        let q = [1.0f32, 0.0, 0.0, 0.0];
        let result = quat_rotate_vec(q, Vec3::new(0.0, 0.0, -10.0));
        assert!((result.x).abs() < 1e-4, "x≈0, got {}", result.x);
        assert!((result.y).abs() < 1e-4, "y≈0, got {}", result.y);
        assert!((result.z + 10.0).abs() < 1e-4, "z≈-10 (North), got {}", result.z);
    }

    #[test]
    fn test_quaternion_to_tangent_east() {
        // East quaternion [√2/2, 0, -√2/2, 0] → rotates North to East
        let s = (2.0f32).sqrt() / 2.0;
        let q = [s, 0.0, -s, 0.0];
        let result = quat_rotate_vec(q, Vec3::new(0.0, 0.0, -10.0));
        assert!((result.x - 10.0).abs() < 1e-4, "x≈10 (East), got {}", result.x);
        assert!((result.y).abs() < 1e-4, "y≈0, got {}", result.y);
        assert!((result.z).abs() < 1e-4, "z≈0, got {}", result.z);
    }

    #[test]
    fn test_build_splines_uses_quaternion_when_available() {
        // Node A at origin with East quaternion, edge goes North.
        // m0 must point East (from quat), not North (from edge geometry).
        use crate::graph::GraphNode;
        let s = (2.0f32).sqrt() / 2.0;
        let graph = MapGraph {
            nodes: vec![
                GraphNode { uid: 1, x: 0.0, y: 0.0, z: 0.0, rotation: [s, 0.0, -s, 0.0] },
                GraphNode { uid: 2, x: 0.0, y: 0.0, z: -10.0, rotation: [1.0, 0.0, 0.0, 0.0] },
            ],
            edges: vec![GraphEdge {
                uid: 10, from: 1, to: 2,
                distance_m: 10.0, speed_limit_kmh: None, lanes: 1,
                direction: "forward".to_string(),
                dlc_guard: 0, is_hidden: false, gps_avoid: false,
                road_look_token: 0, lanes_opposite: 0, lane_width_m: 3.75,
            }],
            ..MapGraph::default()
        };
        let (segs, stats) = build_splines(&graph);
        assert_eq!(segs.len(), 1);
        let seg = &segs[0];
        // m0 from East quaternion → positive x, near-zero z
        assert!(seg.m0.x > 0.1, "m0.x should be positive (East), got {}", seg.m0.x);
        assert!(
            seg.m0.z.abs() < seg.m0.x.abs() * 0.1,
            "m0.z should be near zero, got {}",
            seg.m0.z
        );
        assert_eq!(stats.quat_tangents, 2, "both endpoints should use quaternion");
        assert_eq!(stats.fallback_tangents, 0);
    }

    // --- DS8 SegmentMetadata Tests ---

    fn make_road_graph(lanes_fwd: u8, lanes_bwd: u8, lane_width: f32, direction: &str) -> MapGraph {
        use crate::graph::GraphNode;
        MapGraph {
            nodes: vec![
                GraphNode { uid: 1, x: 0.0, y: 0.0, z: 0.0, rotation: [0.0; 4] },
                GraphNode { uid: 2, x: 100.0, y: 0.0, z: 0.0, rotation: [0.0; 4] },
            ],
            edges: vec![GraphEdge {
                uid: 1, from: 1, to: 2,
                distance_m: 100.0, speed_limit_kmh: None,
                lanes: lanes_fwd,
                direction: direction.to_string(),
                dlc_guard: 0, is_hidden: false, gps_avoid: false,
                road_look_token: 42, lanes_opposite: lanes_bwd, lane_width_m: lane_width,
            }],
            ..MapGraph::default()
        }
    }

    #[test]
    fn ds8_motorway_3lane_offset() {
        // 3-lane motorway, 3.75m: offset = (3 − 0.5) × 3.75 = 9.375m
        let graph = make_road_graph(3, 3, 3.75, "forward");
        let (_, meta, _) = build_splines_ex(&graph);
        assert_eq!(meta.len(), 1);
        let m = meta[0].expect("forward edge must have metadata");
        assert_eq!(m.lanes_in_direction, 3);
        assert_eq!(m.lane_width_m, 3.75);
        assert!((m.lane_offset_right_m - 9.375).abs() < 1e-4, "expected 9.375, got {}", m.lane_offset_right_m);
    }

    #[test]
    fn ds8_city_1lane_offset() {
        // 1-lane city road, 3.0m: offset = (1 − 0.5) × 3.0 = 1.5m
        let graph = make_road_graph(1, 1, 3.0, "forward");
        let (_, meta, _) = build_splines_ex(&graph);
        let m = meta[0].expect("forward edge must have metadata");
        assert_eq!(m.lanes_in_direction, 1);
        assert_eq!(m.lane_width_m, 3.0);
        assert!((m.lane_offset_right_m - 1.5).abs() < 1e-4, "expected 1.5, got {}", m.lane_offset_right_m);
    }

    #[test]
    fn ds8_prefab_metadata_is_none() {
        use crate::graph::GraphNode;
        let graph = MapGraph {
            nodes: vec![
                GraphNode { uid: 1, x: 0.0, y: 0.0, z: 0.0, rotation: [0.0; 4] },
                GraphNode { uid: 2, x: 10.0, y: 0.0, z: 0.0, rotation: [0.0; 4] },
            ],
            edges: vec![GraphEdge {
                uid: 1, from: 1, to: 2,
                distance_m: 10.0, speed_limit_kmh: None, lanes: 1,
                direction: "prefab".to_string(),
                dlc_guard: 0, is_hidden: false, gps_avoid: false,
                road_look_token: 0, lanes_opposite: 0, lane_width_m: 3.75,
            }],
            ..MapGraph::default()
        };
        let (_, meta, _) = build_splines_ex(&graph);
        assert_eq!(meta.len(), 1);
        assert!(meta[0].is_none(), "prefab edge must have no metadata");
    }

    #[test]
    fn ds8_fallback_default_width() {
        // Edge with lane_width_m = 3.75 (default) for unknown road type.
        let graph = make_road_graph(1, 0, 3.75, "forward");
        let (_, meta, _) = build_splines_ex(&graph);
        let m = meta[0].expect("forward edge must have metadata");
        assert!((m.lane_width_m - 3.75).abs() < 1e-4, "default width = 3.75, got {}", m.lane_width_m);
        // (1 − 0.5) × 3.75 = 1.875m — QW1 baseline
        assert!((m.lane_offset_right_m - 1.875).abs() < 1e-4, "offset = 1.875m, got {}", m.lane_offset_right_m);
    }

    #[test]
    fn test_build_splines_fallback_when_quaternion_zero() {
        // Both nodes have zero quaternion → fallback to edge geometry (North).
        use crate::graph::GraphNode;
        let graph = MapGraph {
            nodes: vec![
                GraphNode { uid: 1, x: 0.0, y: 0.0, z: 0.0, rotation: [0.0; 4] },
                GraphNode { uid: 2, x: 0.0, y: 0.0, z: -10.0, rotation: [0.0; 4] },
            ],
            edges: vec![GraphEdge {
                uid: 10, from: 1, to: 2,
                distance_m: 10.0, speed_limit_kmh: None, lanes: 1,
                direction: "forward".to_string(),
                dlc_guard: 0, is_hidden: false, gps_avoid: false,
                road_look_token: 0, lanes_opposite: 0, lane_width_m: 3.75,
            }],
            ..MapGraph::default()
        };
        let (segs, stats) = build_splines(&graph);
        assert_eq!(segs.len(), 1);
        let seg = &segs[0];
        // Fallback: edge direction is (0,0,-10) = North → m0.z should be negative
        assert!(seg.m0.z < -0.1, "m0.z should be negative (North), got {}", seg.m0.z);
        assert!(seg.m0.x.abs() < 0.01, "m0.x should be ~0, got {}", seg.m0.x);
        assert_eq!(stats.quat_tangents, 0);
        assert_eq!(stats.fallback_tangents, 2, "both endpoints should use fallback");
    }
}
