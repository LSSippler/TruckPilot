//! `tangent-consistency-audit` — Hermite-Tangenten-Konsistenz-Audit
//!
//! Misst den Winkel zwischen der tatsächlichen Segment-Verlaufsrichtung (P0→P1, XZ-Ebene)
//! und der Start-Tangente M0 (sowie End-Tangente M1) pro Segment.
//!
//! Erwartung bei korrekter Implementierung:
//!   forward-Segmente:  angle_deg ≈ 0° (Tangente zeigt in Fahrtrichtung)
//!   backward-Segmente: angle_deg ≈ 180° (Tangente NICHT gespiegelt — Bug-Hypothese)
//!
//! Usage:
//!   tangent-consistency-audit [--graph PATH] [--sample N]

use std::collections::HashMap;
use std::path::PathBuf;

use truckpilot_map_parser::graph::MapGraph;
use truckpilot_map_parser::spline::{build_splines_ex, Vec3};

// ---------------------------------------------------------------------------
// Args
// ---------------------------------------------------------------------------

struct Args {
    graph: PathBuf,
    sample_n: usize,
}

fn parse_args() -> Args {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut graph = PathBuf::from("graph.json");
    let mut sample_n = 5usize;
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--graph" if i + 1 < argv.len() => {
                graph = PathBuf::from(&argv[i + 1]);
                i += 2;
            }
            "--sample" if i + 1 < argv.len() => {
                sample_n = argv[i + 1].parse().unwrap_or(5);
                i += 2;
            }
            "--help" | "-h" => {
                eprintln!("usage: tangent-consistency-audit [--graph PATH] [--sample N]");
                std::process::exit(0);
            }
            other => {
                eprintln!("unknown arg: {other}");
                std::process::exit(2);
            }
        }
    }
    Args { graph, sample_n }
}

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

/// Winkel in Grad zwischen zwei Vektoren in der XZ-Ebene (0..180).
/// Gibt 0.0 zurück wenn einer der Vektoren nahezu Null-Länge hat.
fn angle_xz_deg(a: Vec3, b: Vec3) -> f32 {
    let ax = a.x;
    let az = a.z;
    let bx = b.x;
    let bz = b.z;
    let len_a = (ax * ax + az * az).sqrt();
    let len_b = (bx * bx + bz * bz).sqrt();
    if len_a < 1e-9 || len_b < 1e-9 {
        return 0.0;
    }
    let dot = (ax * bx + az * bz) / (len_a * len_b);
    dot.clamp(-1.0, 1.0).acos().to_degrees()
}

// ---------------------------------------------------------------------------
// Aggregation
// ---------------------------------------------------------------------------

struct GroupStats {
    total: u64,
    over_90: u64,
    over_150: u64,
    under_30: u64,
    /// Histogram bins: [0-30, 30-90, 90-150, 150-180]
    hist: [u64; 4],
    min: f32,
    max: f32,
    sum: f64,
    /// Samples: (seg_idx, angle_deg, end_angle_deg, from_uid, to_uid, p0x, p0z, p1x, p1z, m0x, m0z)
    #[allow(clippy::type_complexity)]
    samples: Vec<(usize, f32, f32, u64, u64, f32, f32, f32, f32, f32, f32)>,
}

impl Default for GroupStats {
    fn default() -> Self {
        Self::new()
    }
}

impl GroupStats {
    fn new() -> Self {
        Self {
            total: 0,
            over_90: 0,
            over_150: 0,
            under_30: 0,
            hist: [0; 4],
            min: f32::MAX,
            max: f32::MIN,
            sum: 0.0,
            samples: Vec::new(),
        }
    }

    fn add(&mut self, seg_idx: usize, angle: f32, end_angle: f32, seg: &truckpilot_map_parser::spline::HermiteSegment, sample_n: usize) {
        self.total += 1;
        if angle > 90.0 {
            self.over_90 += 1;
        }
        if angle > 150.0 {
            self.over_150 += 1;
        }
        if angle < 30.0 {
            self.under_30 += 1;
        }
        let bin = if angle < 30.0 {
            0
        } else if angle < 90.0 {
            1
        } else if angle < 150.0 {
            2
        } else {
            3
        };
        self.hist[bin] += 1;
        if angle < self.min {
            self.min = angle;
        }
        if angle > self.max {
            self.max = angle;
        }
        self.sum += angle as f64;

        // Collect samples weighted toward the interesting end (angle > 90)
        if self.samples.len() < sample_n || angle > 90.0 && self.samples.len() < sample_n * 3 {
            self.samples.push((
                seg_idx, angle, end_angle,
                seg.from_uid, seg.to_uid,
                seg.p0.x, seg.p0.z,
                seg.p1.x, seg.p1.z,
                seg.m0.x, seg.m0.z,
            ));
        }
    }

    fn mean(&self) -> f64 {
        if self.total == 0 {
            0.0
        } else {
            self.sum / self.total as f64
        }
    }

    fn pct(&self, n: u64) -> f64 {
        if self.total == 0 {
            0.0
        } else {
            n as f64 / self.total as f64 * 100.0
        }
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() {
    let args = parse_args();

    eprintln!("Loading {}…", args.graph.display());
    let bytes = match std::fs::read(&args.graph) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("ERROR: cannot read {}: {e}", args.graph.display());
            std::process::exit(2);
        }
    };
    let graph: MapGraph = match serde_json::from_slice(&bytes) {
        Ok(g) => g,
        Err(e) => {
            eprintln!("ERROR: cannot parse graph.json: {e}");
            std::process::exit(2);
        }
    };
    eprintln!(
        "Graph: {} nodes, {} edges",
        graph.nodes.len(),
        graph.edges.len()
    );

    // Build edge_uid → direction lookup
    let dir_map: HashMap<u64, String> = graph
        .edges
        .iter()
        .map(|e| (e.uid, e.direction.clone()))
        .collect();

    eprintln!("Building splines…");
    let (segments, _meta, stats) = build_splines_ex(&graph);
    eprintln!(
        "Splines: {} segments ({} skipped missing, {} degenerate)",
        stats.total_segments, stats.skipped_missing_node, stats.skipped_degenerate
    );

    // Calibration segment indices to watch
    const CALIB: &[(usize, &str, &str)] = &[
        (333252, "forward", "<30 erwartet"),
        (333070, "forward", "<30 erwartet"),
        (333361, "backward", ">150 erwartet"),
    ];
    let calib_indices: HashMap<usize, (&str, &str)> = CALIB
        .iter()
        .map(|&(idx, dir, exp)| (idx, (dir, exp)))
        .collect();

    // Aggregate by direction
    let mut groups: HashMap<String, GroupStats> = HashMap::new();
    let mut calib_results: Vec<(usize, String, f32, f32)> = Vec::new();

    for (seg_idx, seg) in segments.iter().enumerate() {
        let dir = dir_map
            .get(&seg.edge_uid)
            .cloned()
            .unwrap_or_else(|| "unknown".to_string());

        let chord = Vec3::new(seg.p1.x - seg.p0.x, 0.0, seg.p1.z - seg.p0.z);
        let m0_xz = Vec3::new(seg.m0.x, 0.0, seg.m0.z);
        let m1_xz = Vec3::new(seg.m1.x, 0.0, seg.m1.z);

        let angle = angle_xz_deg(chord, m0_xz);
        let end_angle = angle_xz_deg(chord, m1_xz);

        groups
            .entry(dir.clone())
            .or_default()
            .add(seg_idx, angle, end_angle, seg, args.sample_n);

        if calib_indices.contains_key(&seg_idx) {
            calib_results.push((seg_idx, dir, angle, end_angle));
        }
    }

    // ---------------------------------------------------------------------------
    // Output
    // ---------------------------------------------------------------------------

    println!("# Tangenten-Konsistenz-Audit");
    println!();
    println!("Graph: `{}`", args.graph.display());
    println!(
        "Segmente gesamt: {} (übersprungen: {} missing-node, {} degenerate)",
        stats.total_segments, stats.skipped_missing_node, stats.skipped_degenerate
    );
    println!();

    // Calibration check first
    println!("## Kalibrierungs-Check (bekannte Segmente)");
    println!();
    println!("| seg_idx | direction | angle_deg (M0) | end_angle_deg (M1) | Erwartung | OK? |");
    println!("|---------|-----------|---------------|---------------------|-----------|-----|");
    calib_results.sort_by_key(|r| r.0);
    for (idx, dir, angle, end_angle) in &calib_results {
        if let Some(&(exp_dir, exp_desc)) = calib_indices.get(idx) {
            let ok = if exp_dir == "forward" {
                *angle < 30.0
            } else {
                *angle > 150.0
            };
            println!(
                "| {} | {} | {:.1}° | {:.1}° | {} | {} |",
                idx,
                dir,
                angle,
                end_angle,
                exp_desc,
                if ok { "✓" } else { "✗ ABWEICHUNG" }
            );
        }
    }
    if calib_results.is_empty() {
        println!("| (keine der 3 Kalibrierungssegmente gefunden — seg_idx außerhalb Reichweite?) | | | | | |");
    }
    println!();

    // Per-group stats
    let mut dir_order: Vec<String> = groups.keys().cloned().collect();
    dir_order.sort();
    // Put forward/backward first
    dir_order.sort_by_key(|d| match d.as_str() {
        "forward" => 0,
        "backward" => 1,
        "bidirectional_unknown" => 2,
        _ => 3,
    });

    println!("## Ergebnisse nach Edge-Direction");
    println!();

    for dir in &dir_order {
        let g = &groups[dir];
        let label = match dir.as_str() {
            "forward" => "forward (Vorwärts-Edges)",
            "backward" => "backward (Rückwärts-Edges)",
            "bidirectional_unknown" => "bidirectional_unknown",
            _ => dir.as_str(),
        };
        println!("### {label}");
        println!();
        println!("| Metrik | Wert |");
        println!("|--------|------|");
        println!("| Segmente | {} |", g.total);
        println!(
            "| angle_deg < 30° (konsistent) | {} ({:.1}%) |",
            g.under_30,
            g.pct(g.under_30)
        );
        println!(
            "| angle_deg > 90° (klar invertiert) | {} ({:.1}%) |",
            g.over_90,
            g.pct(g.over_90)
        );
        println!(
            "| angle_deg > 150° (~180°-Flip) | {} ({:.1}%) |",
            g.over_150,
            g.pct(g.over_150)
        );
        println!();
        println!("**Histogramm:**");
        println!();
        println!("| Bin | Anzahl | % |");
        println!("|-----|--------|---|");
        let bins = ["0–30°", "30–90°", "90–150°", "150–180°"];
        for (i, &label) in bins.iter().enumerate() {
            println!(
                "| {} | {} | {:.1}% |",
                label,
                g.hist[i],
                g.pct(g.hist[i])
            );
        }
        println!();
        println!(
            "**Min:** {:.1}° | **Mean:** {:.1}° | **Max:** {:.1}°",
            if g.min == f32::MAX { 0.0 } else { g.min },
            g.mean(),
            if g.max == f32::MIN { 0.0 } else { g.max }
        );
        println!();

        // Samples
        if !g.samples.is_empty() {
            println!("**Beispiel-Segmente (–sample {}):**", args.sample_n);
            println!();
            println!("| seg_idx | angle_deg | end_angle_deg | from_uid | to_uid | P0(x,z) | P1(x,z) | M0(x,z) |");
            println!("|---------|-----------|---------------|----------|--------|---------|---------|---------|");
            for &(si, ang, end_ang, from_uid, to_uid, p0x, p0z, p1x, p1z, m0x, m0z) in
                g.samples.iter().take(args.sample_n.max(10))
            {
                println!(
                    "| {} | {:.1}° | {:.1}° | {} | {} | ({:.0},{:.0}) | ({:.0},{:.0}) | ({:.1},{:.1}) |",
                    si, ang, end_ang, from_uid, to_uid, p0x, p0z, p1x, p1z, m0x, m0z
                );
            }
            println!();
        }
    }

    // Summary / hypothesis verdict
    println!("## Hypothesen-Bewertung");
    println!();
    let fwd = groups.get("forward");
    let bwd = groups.get("backward");

    match (fwd, bwd) {
        (Some(f), Some(b)) => {
            let fwd_clean_pct = f.pct(f.under_30);
            let bwd_flip_pct = b.pct(b.over_150);
            println!(
                "- forward-Gruppe: **{:.1}%** der Segmente angle < 30° (konsistent)",
                fwd_clean_pct
            );
            println!(
                "- backward-Gruppe: **{:.1}%** der Segmente angle > 150° (~180°-Flip)",
                bwd_flip_pct
            );
            println!();
            if fwd_clean_pct > 80.0 && bwd_flip_pct > 50.0 {
                println!(
                    "**HYPOTHESE BESTÄTIGT** — backward-Tangenten sind systematisch ~180° verkehrt, \
                    forward-Tangenten korrekt."
                );
            } else if fwd_clean_pct <= 80.0 && bwd_flip_pct > 50.0 {
                println!(
                    "**HYPOTHESE TEILWEISE** — backward eindeutig geflippt ({:.1}%), \
                    aber auch forward unclean ({:.1}% < 30°). Tangenten-Fallback vermutlich bei beiden betroffen.",
                    bwd_flip_pct, fwd_clean_pct
                );
            } else {
                println!(
                    "**HYPOTHESE UNKLAR** — backward-Flip {:.1}%, forward-Clean {:.1}%. \
                    Bitte Kalibrierungs-Check oben prüfen und Samples begutachten.",
                    bwd_flip_pct, fwd_clean_pct
                );
            }
        }
        _ => {
            println!("Zu wenige Daten für forward/backward-Vergleich.");
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn angle_same_direction() {
        let a = Vec3::new(1.0, 0.0, 0.0);
        let b = Vec3::new(2.0, 0.0, 0.0); // same XZ direction, different magnitude
        let deg = angle_xz_deg(a, b);
        assert!(deg < 0.01, "same direction should be ~0°, got {deg}");
    }

    #[test]
    fn angle_opposite() {
        let a = Vec3::new(1.0, 0.0, 0.0);
        let b = Vec3::new(-1.0, 0.0, 0.0);
        let deg = angle_xz_deg(a, b);
        assert!((deg - 180.0).abs() < 0.01, "opposite should be ~180°, got {deg}");
    }

    #[test]
    fn angle_perpendicular() {
        let a = Vec3::new(1.0, 0.0, 0.0);
        let b = Vec3::new(0.0, 0.0, 1.0);
        let deg = angle_xz_deg(a, b);
        assert!((deg - 90.0).abs() < 0.01, "perpendicular should be ~90°, got {deg}");
    }

    #[test]
    fn angle_ignores_y() {
        // Y component should be ignored — only XZ matters
        let a = Vec3::new(1.0, 999.0, 0.0);
        let b = Vec3::new(1.0, -999.0, 0.0);
        let deg = angle_xz_deg(a, b);
        assert!(deg < 0.01, "Y ignored: same XZ should be ~0°, got {deg}");
    }
}
