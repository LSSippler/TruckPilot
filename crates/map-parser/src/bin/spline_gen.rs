//! spline-gen — Phase 0b Hermite Spline Generator CLI
//!
//! Subcommands:
//!   gen   --graph <graph.json> --output <splines.bin>
//!   plot  --graph <graph.json> --output <out.svg> --bbox X1,Z1,X2,Z2
//!   stats --graph <graph.json> [--bbox X1,Z1,X2,Z2]
//!
//! Heading-Quelle: atan2(dx, -dz) aus Edge-Positionen (Spec §1.3).
//! Node-Quaternions werden nicht verwendet.

use std::collections::HashMap;
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use truckpilot_map_parser::arc_length::{
    build_all_luts, build_forward_adjacency, lookahead, point_at_arc_length, t_at_arc_length,
    ArcLengthLUT,
};
use truckpilot_map_parser::graph::MapGraph;
use truckpilot_map_parser::spline::{
    build_splines, build_splines_bbox, evaluate, BBox, HermiteSegment, Vec3, TANGENT_SCALE,
};
use truckpilot_map_parser::spline_index::{build_index, compare_aabb_strategies};

// ---------------------------------------------------------------------------
// CLI-Parsing (kein clap, nur std::env::args für minimale Deps)
// ---------------------------------------------------------------------------

#[derive(Debug)]
enum Command {
    Gen {
        graph_path: PathBuf,
        output_path: PathBuf,
    },
    Plot {
        graph_path: PathBuf,
        output_path: PathBuf,
        bbox: BBox,
        samples: usize,
    },
    Stats {
        graph_path: PathBuf,
        bbox: Option<BBox>,
    },
    Bench {
        graph_path: PathBuf,
        queries: usize,
        seed: u64,
        candidates: usize,
    },
    ArcLength {
        graph_path: PathBuf,
        queries: usize,
        seed: u64,
    },
}

fn parse_bbox(s: &str) -> Result<BBox, String> {
    let parts: Vec<f32> = s
        .split(',')
        .map(|p| p.trim().parse::<f32>().map_err(|e| e.to_string()))
        .collect::<Result<_, _>>()?;
    if parts.len() != 4 {
        return Err(format!(
            "bbox braucht 4 Werte X1,Z1,X2,Z2, got {}",
            parts.len()
        ));
    }
    Ok(BBox::new(parts[0], parts[1], parts[2], parts[3]))
}

fn parse_args() -> Result<Command, String> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        return Err(usage());
    }
    match args[1].as_str() {
        "gen" => {
            let graph_path = get_arg(&args, "--graph")?;
            let output_path = get_arg(&args, "--output")?;
            Ok(Command::Gen {
                graph_path: PathBuf::from(graph_path),
                output_path: PathBuf::from(output_path),
            })
        }
        "plot" => {
            let graph_path = get_arg(&args, "--graph")?;
            let output_path = get_arg(&args, "--output")?;
            let bbox_str = get_arg(&args, "--bbox")?;
            let bbox = parse_bbox(&bbox_str)?;
            let samples = get_arg(&args, "--samples")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(20usize);
            Ok(Command::Plot {
                graph_path: PathBuf::from(graph_path),
                output_path: PathBuf::from(output_path),
                bbox,
                samples,
            })
        }
        "stats" => {
            let graph_path = get_arg(&args, "--graph")?;
            let bbox = get_arg(&args, "--bbox")
                .ok()
                .map(|s| parse_bbox(&s))
                .transpose()?;
            Ok(Command::Stats {
                graph_path: PathBuf::from(graph_path),
                bbox,
            })
        }
        "bench" => {
            let graph_path = get_arg(&args, "--graph")?;
            let queries = get_arg(&args, "--queries")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(10_000usize);
            let seed = get_arg(&args, "--seed")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(42u64);
            let candidates = get_arg(&args, "--candidates")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(8usize);
            Ok(Command::Bench {
                graph_path: PathBuf::from(graph_path),
                queries,
                seed,
                candidates,
            })
        }
        "arclength" => {
            let graph_path = get_arg(&args, "--graph")?;
            let queries = get_arg(&args, "--queries")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(10_000usize);
            let seed = get_arg(&args, "--seed")
                .ok()
                .and_then(|s| s.parse().ok())
                .unwrap_or(42u64);
            Ok(Command::ArcLength {
                graph_path: PathBuf::from(graph_path),
                queries,
                seed,
            })
        }
        _ => Err(usage()),
    }
}

fn get_arg(args: &[String], flag: &str) -> Result<String, String> {
    args.windows(2)
        .find(|w| w[0] == flag)
        .map(|w| w[1].clone())
        .ok_or_else(|| format!("Fehlendes Argument: {flag}"))
}

fn usage() -> String {
    concat!(
        "spline-gen -- Phase 0b/0c/0c.2 Hermite Spline Generator\n\n",
        "Verwendung:\n",
        "  spline-gen gen       --graph <graph.json> --output <splines.json>\n",
        "  spline-gen plot      --graph <graph.json> --output <out.svg> --bbox X1,Z1,X2,Z2 [--samples N]\n",
        "  spline-gen stats     --graph <graph.json> [--bbox X1,Z1,X2,Z2]\n",
        "  spline-gen bench     --graph <graph.json> [--queries N] [--seed N] [--candidates N]\n",
        "  spline-gen arclength --graph <graph.json> [--queries N] [--seed N]\n\n",
        "BBox-Format:  X1,Z1,X2,Z2 in Weltkoordinaten (Meter)\n",
        "Samples:      Sample-Punkte pro Hermite-Segment im SVG (default: 20)\n",
        "Queries:      Anzahl Zufalls-Queries fuer Benchmark (default: 10000)\n",
        "Seed:         RNG-Seed fuer reproduzierbare Queries (default: 42)\n",
        "Candidates:   R-tree-Kandidaten fuer Projektion (default: 8)\n\n",
        "Eyeball-Test-BBoxes (Berlin-Anker UID 6919855103841468416 als Referenz):\n",
        "  Highway-Auffahrt:  -10500,-6200,-10000,-5700\n",
        "  Stadt-Junction:    -10200,-5900,-9800,-5500\n",
        "  ProMods-Bereich:   30000,-20000,31000,-19000\n",
    ).to_string()
}

// ---------------------------------------------------------------------------
// graph.json laden
// ---------------------------------------------------------------------------

fn load_graph(path: &PathBuf) -> MapGraph {
    eprintln!("Lade graph.json: {}", path.display());
    let start = std::time::Instant::now();
    let data = std::fs::read(path).unwrap_or_else(|e| {
        eprintln!("FEHLER: Kann {}: {e}", path.display());
        std::process::exit(1);
    });
    let graph: MapGraph = serde_json::from_slice(&data).unwrap_or_else(|e| {
        eprintln!("FEHLER: JSON-Parse fehlgeschlagen: {e}");
        std::process::exit(1);
    });
    eprintln!(
        "Graph geladen: {} Nodes, {} Edges in {:.1}s",
        graph.nodes.len(),
        graph.edges.len(),
        start.elapsed().as_secs_f32()
    );
    graph
}

// ---------------------------------------------------------------------------
// Subcommand: gen
// ---------------------------------------------------------------------------

fn cmd_gen(graph_path: &PathBuf, output_path: &PathBuf) {
    let graph = load_graph(graph_path);

    eprintln!("Baue Hermite-Segmente (TANGENT_SCALE={TANGENT_SCALE})...");
    let start = std::time::Instant::now();
    let (segments, stats) = build_splines(&graph);
    eprintln!("Fertig in {:.1}s", start.elapsed().as_secs_f32());
    stats.print_summary();

    // Memory-Footprint STOP-Check
    let bytes_per_seg = std::mem::size_of::<HermiteSegment>();
    let total_bytes = bytes_per_seg * segments.len();
    let total_gb = total_bytes as f64 / 1_073_741_824.0;
    if total_gb >= 1.0 {
        eprintln!(
            "STOP: Extrapolierter Memory-Footprint {total_gb:.2} GB >= 1 GB! \
             Trade-offs mit User diskutieren."
        );
        std::process::exit(2);
    }

    // Serialisierung als JSON (kompakt, gut für Debugging)
    eprintln!("Schreibe {output_path:?}...");
    let file = std::fs::File::create(output_path).unwrap_or_else(|e| {
        eprintln!("FEHLER: {e}");
        std::process::exit(1);
    });
    let writer = BufWriter::new(file);
    serde_json::to_writer(writer, &segments).unwrap_or_else(|e| {
        eprintln!("FEHLER beim Schreiben: {e}");
        std::process::exit(1);
    });

    let file_size = std::fs::metadata(output_path).map(|m| m.len()).unwrap_or(0);
    eprintln!(
        "Output: {} Segmente → {} ({:.1} MB)",
        segments.len(),
        output_path.display(),
        file_size as f64 / 1_048_576.0
    );
}

// ---------------------------------------------------------------------------
// Subcommand: stats
// ---------------------------------------------------------------------------

fn cmd_stats(graph_path: &PathBuf, bbox: Option<BBox>) {
    let graph = load_graph(graph_path);

    let (segments, stats) = if let Some(bb) = bbox {
        eprintln!(
            "BBox-Filter: x=[{:.0}..{:.0}] z=[{:.0}..{:.0}]",
            bb.x_min, bb.x_max, bb.z_min, bb.z_max
        );
        build_splines_bbox(&graph, bb)
    } else {
        build_splines(&graph)
    };

    stats.print_summary();

    if segments.is_empty() {
        eprintln!("Keine Segmente — BBox prüfen?");
        return;
    }

    // Längen-Statistik
    let lengths: Vec<f32> = segments.iter().map(|s| s.length_m).collect();
    let mean_len = lengths.iter().sum::<f32>() / lengths.len() as f32;
    let max_len = lengths.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
    let min_len = lengths.iter().cloned().fold(f32::INFINITY, f32::min);

    println!("\n=== Längen-Statistik ===");
    println!("  Min:    {min_len:.1} m");
    println!("  Max:    {max_len:.1} m");
    println!("  Mean:   {mean_len:.1} m");

    // Tangenten-Magnitude-Histogramm (relativ zur Chord-Length)
    let ratios: Vec<f32> = segments
        .iter()
        .map(|s| {
            let m0r = s.m0.length() / s.length_m.max(1.0);
            let m1r = s.m1.length() / s.length_m.max(1.0);
            m0r.max(m1r)
        })
        .collect();

    println!("\n=== Tangenten-Magnitude / Chord-Length (Histogramm) ===");
    let buckets = [
        (0.0, 0.5),
        (0.5, 1.0),
        (1.0, 1.5),
        (1.5, 2.0),
        (2.0, 3.0),
        (3.0, f32::INFINITY),
    ];
    for (lo, hi) in buckets {
        let count = ratios.iter().filter(|&&r| r >= lo && r < hi).count();
        let pct = count as f32 / ratios.len() as f32 * 100.0;
        let label = if hi == f32::INFINITY {
            format!("{lo:.1}+")
        } else {
            format!("{lo:.1}–{hi:.1}")
        };
        let bar: String = "#".repeat((pct / 2.0) as usize);
        println!("  [{label:>8}]: {count:>7} ({pct:5.1}%) {bar}");
    }

    // Memory-Footprint
    let bytes_per_seg = std::mem::size_of::<HermiteSegment>();
    let total_bytes_actual = bytes_per_seg * segments.len();
    let extrapolated_all = bytes_per_seg * graph.edges.len();
    println!("\n=== Memory-Footprint ===");
    println!("  sizeof(HermiteSegment): {} Bytes", bytes_per_seg);
    println!(
        "  Aktuell ({} Seg):       {:.1} MB",
        segments.len(),
        total_bytes_actual as f64 / 1_048_576.0
    );
    println!(
        "  Extrapoliert ({}k Edges): {:.1} MB ({:.2} GB)",
        graph.edges.len() / 1000,
        extrapolated_all as f64 / 1_048_576.0,
        extrapolated_all as f64 / 1_073_741_824.0
    );
    if extrapolated_all as f64 / 1_073_741_824.0 >= 1.0 {
        println!("  !! STOP-Bedingung: > 1 GB !!");
    }

    // Pathologische Fälle detail
    if stats.pathological_tangents > 0 {
        println!("\n=== Pathologische Fälle (Tangenten-Ratio > 3.0) ===");
        println!("  Anzahl: {}", stats.pathological_tangents);
        let examples: Vec<_> = segments
            .iter()
            .filter(|s| {
                let r0 = s.m0.length() / s.length_m.max(1.0);
                let r1 = s.m1.length() / s.length_m.max(1.0);
                r0 > 3.0 || r1 > 3.0
            })
            .take(5)
            .collect();
        for seg in examples {
            let r0 = seg.m0.length() / seg.length_m.max(1.0);
            let r1 = seg.m1.length() / seg.length_m.max(1.0);
            println!(
                "    edge_uid={} from={} to={} len={:.1}m m0_ratio={:.2} m1_ratio={:.2}",
                seg.edge_uid, seg.from_uid, seg.to_uid, seg.length_m, r0, r1
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Subcommand: plot (SVG)
// ---------------------------------------------------------------------------

/// RGB-Farbe
struct Color(u8, u8, u8);

impl Color {
    fn to_css(&self) -> String {
        format!("#{:02X}{:02X}{:02X}", self.0, self.1, self.2)
    }
}

fn cmd_plot(graph_path: &PathBuf, output_path: &PathBuf, bbox: BBox, samples: usize) {
    let graph = load_graph(graph_path);

    eprintln!(
        "BBox: x=[{:.0}..{:.0}] z=[{:.0}..{:.0}]",
        bbox.x_min, bbox.x_max, bbox.z_min, bbox.z_max
    );

    let (segments, stats) = build_splines_bbox(&graph, bbox);
    eprintln!("{} Segmente in BBox", segments.len());
    stats.print_summary();

    if segments.is_empty() {
        eprintln!("Keine Segmente — BBox prüfen oder vergrößern.");
        return;
    }

    // Node-Map für Junction-Degree-Ermittlung
    let node_pos: HashMap<u64, Vec3> = graph
        .nodes
        .iter()
        .filter(|n| bbox.contains(n.x as f32, n.z as f32))
        .map(|n| (n.uid, Vec3::new(n.x as f32, n.y as f32, n.z as f32)))
        .collect();

    // Degree-Map
    let mut degree: HashMap<u64, usize> = HashMap::new();
    for seg in &segments {
        *degree.entry(seg.from_uid).or_insert(0) += 1;
        *degree.entry(seg.to_uid).or_insert(0) += 1;
    }

    // SVG-Koordinaten: x → SVG-x, z → SVG-y (z-Achse invertiert)
    let svg_w = 1200.0f64;
    let svg_h = 900.0f64;
    let margin = 40.0f64;

    let world_w = (bbox.x_max - bbox.x_min) as f64;
    let world_h = (bbox.z_max - bbox.z_min) as f64;

    let scale_x = (svg_w - 2.0 * margin) / world_w;
    let scale_z = (svg_h - 2.0 * margin) / world_h;
    let scale = scale_x.min(scale_z); // uniform scale

    let to_svg = |x: f32, z: f32| -> (f64, f64) {
        let sx = margin + (x as f64 - bbox.x_min as f64) * scale;
        // z-Achse: kleinere z-Werte sind weiter oben (Nord)
        let sy = svg_h - margin - (z as f64 - bbox.z_min as f64) * scale;
        (sx, sy)
    };

    // Farbskala für Splines: nach Heading
    // Heading 0° (Nord) = Blau, 90° (Ost) = Rot, 180° (Süd) = Grün, 270° (West) = Lila
    let heading_color = |heading_deg: f32| -> Color {
        let h = heading_deg.rem_euclid(360.0) / 360.0;
        // HSV → RGB (vereinfacht: hue-only, s=0.8, v=0.9)
        let i = (h * 6.0) as u32;
        let f = h * 6.0 - i as f32;
        let p = (0.9 * (1.0 - 0.8)) * 255.0;
        let q = (0.9 * (1.0 - 0.8 * f)) * 255.0;
        let t_val = (0.9 * (1.0 - 0.8 * (1.0 - f))) * 255.0;
        let v = (0.9 * 255.0) as u8;
        match i % 6 {
            0 => Color(v, t_val as u8, p as u8),
            1 => Color(q as u8, v, p as u8),
            2 => Color(p as u8, v, t_val as u8),
            3 => Color(p as u8, q as u8, v),
            4 => Color(t_val as u8, p as u8, v),
            _ => Color(v, p as u8, q as u8),
        }
    };

    // SVG aufbauen
    let file = std::fs::File::create(output_path).unwrap_or_else(|e| {
        eprintln!("FEHLER: {e}");
        std::process::exit(1);
    });
    let mut w = BufWriter::new(file);

    {
        let bg = "#111111";
        writeln!(
            w,
            r#"<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{svg_w}" height="{svg_h}" viewBox="0 0 {svg_w} {svg_h}">
  <rect width="{svg_w}" height="{svg_h}" fill="{bg}"/>
  <g id="edges">"#
        )
        .unwrap();
    }

    // Edges als dünne graue Linien (Chord)
    for seg in &segments {
        let (x0, y0) = to_svg(seg.p0.x, seg.p0.z);
        let (x1, y1) = to_svg(seg.p1.x, seg.p1.z);
        {
            let stroke = "#444444";
            writeln!(
                w,
                r#"    <line x1="{x0:.1}" y1="{y0:.1}" x2="{x1:.1}" y2="{y1:.1}" stroke="{stroke}" stroke-width="0.8" opacity="0.6"/>"#
            )
            .unwrap();
        }
    }

    writeln!(w, r#"  </g>"#).unwrap();
    writeln!(w, r#"  <g id="splines">"#).unwrap();

    // Hermite-Splines als farbige Polylines
    let clamp_svg = |v: f64, max: f64| v.clamp(0.0, max);

    for seg in &segments {
        // Heading am Mittelpunkt für Farbwahl
        let mid_tan = {
            use truckpilot_map_parser::spline::evaluate_tangent;
            evaluate_tangent(seg, 0.5)
        };
        let heading = f32::atan2(mid_tan.x, -mid_tan.z)
            .to_degrees()
            .rem_euclid(360.0);
        let color = heading_color(heading);

        let pts: Vec<(f64, f64)> = (0..=samples)
            .map(|i| {
                let t = i as f32 / samples as f32;
                let p = evaluate(seg, t);
                let (sx, sy) = to_svg(p.x, p.z);
                (clamp_svg(sx, svg_w), clamp_svg(sy, svg_h))
            })
            .collect();

        let points_str: Vec<String> = pts.iter().map(|(x, y)| format!("{x:.1},{y:.1}")).collect();
        writeln!(
            w,
            r#"    <polyline points="{}" fill="none" stroke="{}" stroke-width="1.5" opacity="0.85"/>"#,
            points_str.join(" "),
            color.to_css()
        )
        .unwrap();
    }

    writeln!(w, r#"  </g>"#).unwrap();
    writeln!(w, r#"  <g id="junctions">"#).unwrap();

    // Junctions (deg > 2) als gelbe Kreise
    for (&uid, &deg) in &degree {
        if deg <= 2 {
            continue;
        }
        if let Some(&pos) = node_pos.get(&uid) {
            let (sx, sy) = to_svg(pos.x, pos.z);
            let radius = if deg > 4 { 4.0 } else { 3.0 };
            let color = if deg > 4 { "#FF4444" } else { "#FFDD44" };
            writeln!(
                w,
                r#"    <circle cx="{sx:.1}" cy="{sy:.1}" r="{radius}" fill="{color}" opacity="0.9"/>"#
            )
            .unwrap();
        }
    }

    writeln!(w, r#"  </g>"#).unwrap();

    // Legende
    {
        let fg = "#CCCCCC";
        let c_edge = "#444444";
        let c_spline = "#44AAFF";
        let c_junc = "#FFDD44";
        let c_high = "#FF4444";
        writeln!(
            w,
            r#"  <g id="legend" font-family="monospace" font-size="11" fill="{fg}">
    <text x="10" y="20">Phase 0b — Hermite Splines | BBox: x=[{bx1:.0}..{bx2:.0}] z=[{bz1:.0}..{bz2:.0}]</text>
    <text x="10" y="36">Segmente: {nseg} | TANGENT_SCALE: {TANGENT_SCALE} | Samples/Seg: {samples}</text>
    <rect x="10" y="44" width="12" height="4" fill="{c_edge}"/>
    <text x="26" y="50">Edge (chord)</text>
    <rect x="90" y="44" width="12" height="4" fill="{c_spline}"/>
    <text x="106" y="50">Hermite (heading-colored)</text>
    <circle cx="16" cy="62" r="3" fill="{c_junc}"/>
    <text x="26" y="66">Junction (deg 3-4)</text>
    <circle cx="16" cy="76" r="4" fill="{c_high}"/>
    <text x="26" y="80">High-degree Junction (deg 5+)</text>
  </g>"#,
            bx1 = bbox.x_min, bx2 = bbox.x_max, bz1 = bbox.z_min, bz2 = bbox.z_max,
            nseg = segments.len()
        )
        .unwrap();
    }

    writeln!(w, r#"</svg>"#).unwrap();
    w.flush().unwrap();

    let file_size = std::fs::metadata(output_path).map(|m| m.len()).unwrap_or(0);
    eprintln!(
        "SVG geschrieben: {} ({:.2} MB)",
        output_path.display(),
        file_size as f64 / 1_048_576.0
    );
    if file_size > 5 * 1_048_576 {
        eprintln!(
            "WARNUNG: SVG > 5 MB ({:.2} MB). Erwäge --samples zu reduzieren oder BBox zu verkleinern.",
            file_size as f64 / 1_048_576.0
        );
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

fn main() {
    match parse_args() {
        Err(e) => {
            eprintln!("{e}");
            std::process::exit(1);
        }
        Ok(Command::Gen {
            graph_path,
            output_path,
        }) => {
            cmd_gen(&graph_path, &output_path);
        }
        Ok(Command::Plot {
            graph_path,
            output_path,
            bbox,
            samples,
        }) => {
            cmd_plot(&graph_path, &output_path, bbox, samples);
        }
        Ok(Command::Stats { graph_path, bbox }) => {
            cmd_stats(&graph_path, bbox);
        }
        Ok(Command::Bench {
            graph_path,
            queries,
            seed,
            candidates,
        }) => {
            cmd_bench(&graph_path, queries, seed, candidates);
        }
        Ok(Command::ArcLength {
            graph_path,
            queries,
            seed,
        }) => {
            cmd_arclength(&graph_path, queries, seed);
        }
    }
}

// ---------------------------------------------------------------------------
// Subcommand: bench
// ---------------------------------------------------------------------------

/// Einfacher LCG-RNG (keine extra dep). Seed-basiert, reproduzierbar.
struct Lcg(u64);
impl Lcg {
    fn next_f32(&mut self) -> f32 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        ((self.0 >> 33) as f32) / (u32::MAX as f32)
    }
}

/// Misst Laufzeit eines Closures in Mikrosekunden.
fn time_us<F: FnMut()>(mut f: F) -> f64 {
    let start = std::time::Instant::now();
    f();
    start.elapsed().as_secs_f64() * 1_000_000.0
}

fn cmd_bench(graph_path: &PathBuf, n_queries: usize, seed: u64, candidates: usize) {
    let graph = load_graph(graph_path);

    // --- Spline-Build ---
    eprintln!("Baue Hermite-Segmente...");
    let t0 = std::time::Instant::now();
    let (segments, stats) = build_splines(&graph);
    let spline_build_ms = t0.elapsed().as_secs_f64() * 1000.0;
    eprintln!(
        "Spline-Build: {spline_build_ms:.0} ms, {} Segmente",
        segments.len()
    );
    stats.print_summary();

    // --- AABB-Vergleich ---
    eprintln!("\nVergleiche AABB-Strategien (Stichprobe 10k)...");
    let aabb_stats = compare_aabb_strategies(&segments, 10_000);
    println!("\n=== AABB-Vergleich (chord vs sampled) ===");
    println!("  Mean chord_area:   {:.1} m²", aabb_stats.mean_chord_area);
    println!(
        "  Mean sampled_area: {:.1} m²",
        aabb_stats.mean_sampled_area
    );
    println!(
        "  Overhead:          {:.1}x",
        aabb_stats.mean_sampled_area / aabb_stats.mean_chord_area.max(1.0)
    );
    println!(
        "  Bulge-Anteil:      {:.1}% (sampled > chord * 1.01)",
        aabb_stats.pct_bulge
    );

    // --- R-tree-Build ---
    eprintln!("\nBaue R-tree Index...");
    let t1 = std::time::Instant::now();
    let index = build_index(segments);
    let rtree_build_ms = t1.elapsed().as_secs_f64() * 1000.0;
    eprintln!("R-tree-Build: {rtree_build_ms:.0} ms");

    let mem = index.memory_stats();
    mem.print_summary();

    if mem.total_bytes_approx as f64 / 1_073_741_824.0 > 0.5 {
        eprintln!("STOP: R-tree-Footprint > 500 MB! Sektor-Partitionierung noetig.");
        std::process::exit(2);
    }

    // --- Query-Positionen generieren ---
    // Bounding-Box des Graphen aus Segment-Endpunkten
    let (mut xmin, mut xmax, mut zmin, mut zmax) = (
        f32::INFINITY,
        f32::NEG_INFINITY,
        f32::INFINITY,
        f32::NEG_INFINITY,
    );
    for seg in &index.segments {
        xmin = xmin.min(seg.p0.x).min(seg.p1.x);
        xmax = xmax.max(seg.p0.x).max(seg.p1.x);
        zmin = zmin.min(seg.p0.z).min(seg.p1.z);
        zmax = zmax.max(seg.p0.z).max(seg.p1.z);
    }
    eprintln!("\nGraph-BBox: x=[{xmin:.0}..{xmax:.0}] z=[{zmin:.0}..{zmax:.0}]");

    let mut rng = Lcg(seed);
    let query_points: Vec<Vec3> = (0..n_queries)
        .map(|_| {
            Vec3::new(
                xmin + rng.next_f32() * (xmax - xmin),
                0.0,
                zmin + rng.next_f32() * (zmax - zmin),
            )
        })
        .collect();

    println!("\n=== Benchmark: {n_queries} Queries, seed={seed} ===");

    // --- nearest(k=5) ---
    let mut nearest_times: Vec<f64> = Vec::with_capacity(n_queries);
    for &q in &query_points {
        let us = time_us(|| {
            let _ = index.nearest(q, 5);
        });
        nearest_times.push(us);
    }
    nearest_times.sort_by(|a, b| a.partial_cmp(b).unwrap());
    println!("\n--- nearest(k=5) ---");
    println!("  p50:  {:>8.2} µs", percentile(&nearest_times, 50));
    println!("  p95:  {:>8.2} µs", percentile(&nearest_times, 95));
    println!("  p99:  {:>8.2} µs", percentile(&nearest_times, 99));
    println!(
        "  mean: {:>8.2} µs",
        nearest_times.iter().sum::<f64>() / n_queries as f64
    );

    if percentile(&nearest_times, 99) > 5000.0 {
        eprintln!("STOP: nearest() p99 > 5ms! R-tree ineffizient — neu evaluieren.");
    }

    // --- within_radius(50m) ---
    let mut radius_times: Vec<f64> = Vec::with_capacity(n_queries);
    for &q in &query_points {
        let us = time_us(|| {
            let _ = index.within_radius(q, 50.0);
        });
        radius_times.push(us);
    }
    radius_times.sort_by(|a, b| a.partial_cmp(b).unwrap());
    println!("\n--- within_radius(50m) ---");
    println!("  p50:  {:>8.2} µs", percentile(&radius_times, 50));
    println!("  p95:  {:>8.2} µs", percentile(&radius_times, 95));
    println!("  p99:  {:>8.2} µs", percentile(&radius_times, 99));
    println!(
        "  mean: {:>8.2} µs",
        radius_times.iter().sum::<f64>() / n_queries as f64
    );

    // --- nearest_with_projection ---
    let mut proj_times: Vec<f64> = Vec::with_capacity(n_queries);
    for &q in &query_points {
        let us = time_us(|| {
            let _ = index.nearest_with_projection(q, candidates);
        });
        proj_times.push(us);
    }
    proj_times.sort_by(|a, b| a.partial_cmp(b).unwrap());
    println!("\n--- nearest_with_projection(candidates={candidates}) ---");
    println!("  p50:  {:>8.2} µs", percentile(&proj_times, 50));
    println!("  p95:  {:>8.2} µs", percentile(&proj_times, 95));
    println!("  p99:  {:>8.2} µs", percentile(&proj_times, 99));
    println!(
        "  mean: {:>8.2} µs",
        proj_times.iter().sum::<f64>() / n_queries as f64
    );

    if percentile(&proj_times, 99) > 5000.0 {
        eprintln!("STOP: nearest_with_projection() p99 > 5ms!");
    }

    // --- Full-Scan-Baseline (nur auf 100 Queries, wäre sonst zu langsam) ---
    let baseline_n = n_queries.min(100);
    let segs = &index.segments;
    let mut scan_times: Vec<f64> = Vec::with_capacity(baseline_n);
    for &q in query_points.iter().take(baseline_n) {
        let us = time_us(|| {
            let _ = segs
                .iter()
                .map(|s| {
                    let dx = s.p0.x - q.x;
                    let dz = s.p0.z - q.z;
                    dx * dx + dz * dz
                })
                .fold(f32::INFINITY, f32::min);
        });
        scan_times.push(us);
    }
    scan_times.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let scan_p50 = percentile(&scan_times, 50);
    let rtree_p50 = percentile(&nearest_times, 50);
    println!("\n--- Full-Scan-Baseline ({baseline_n} Queries) ---");
    println!("  p50:    {:>8.2} µs", scan_p50);
    println!(
        "  Speedup nearest vs scan: {:.1}x",
        scan_p50 / rtree_p50.max(0.001)
    );

    // --- Zusammenfassung ---
    println!("\n=== Build-Zeiten ===");
    println!("  Spline-Build:  {spline_build_ms:.0} ms");
    println!("  R-tree-Build:  {rtree_build_ms:.0} ms");
    println!(
        "  Gesamt:        {:.0} ms",
        spline_build_ms + rtree_build_ms
    );
}

fn percentile(sorted: &[f64], p: usize) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    let idx = ((p * sorted.len()) / 100).min(sorted.len() - 1);
    sorted[idx]
}

// ---------------------------------------------------------------------------
// Subcommand: arclength (Phase 0c.2)
// ---------------------------------------------------------------------------

fn cmd_arclength(graph_path: &PathBuf, n_queries: usize, seed: u64) {
    let graph = load_graph(graph_path);

    // --- Spline-Build ---
    eprintln!("Baue Hermite-Segmente...");
    let t0 = std::time::Instant::now();
    let (segments, stats) = build_splines(&graph);
    let spline_build_ms = t0.elapsed().as_secs_f64() * 1000.0;
    eprintln!(
        "Spline-Build: {spline_build_ms:.0} ms, {} Segmente",
        segments.len()
    );
    stats.print_summary();

    let n_segs = segments.len();

    // --- LUT-Build ---
    eprintln!("\nBaue Arc-Length LUTs ({n_segs} Segmente)...");
    let t1 = std::time::Instant::now();
    let luts = build_all_luts(&segments);
    let lut_build_ms = t1.elapsed().as_secs_f64() * 1000.0;
    eprintln!("LUT-Build: {lut_build_ms:.0} ms");

    // LUT-Footprint
    let lut_bytes_per_entry = std::mem::size_of::<ArcLengthLUT>();
    let lut_total_bytes = lut_bytes_per_entry * n_segs;

    println!("\n=== LUT-Footprint ===");
    println!("  sizeof(ArcLengthLUT): {} Bytes", lut_bytes_per_entry);
    println!(
        "  {} Segmente × {} Bytes = {:.1} MB",
        n_segs,
        lut_bytes_per_entry,
        lut_total_bytes as f64 / 1_048_576.0
    );
    println!(
        "  HermiteSegment {:.1} MB + LUT {:.1} MB = {:.1} MB gesamt",
        (std::mem::size_of::<HermiteSegment>() * n_segs) as f64 / 1_048_576.0,
        lut_total_bytes as f64 / 1_048_576.0,
        (std::mem::size_of::<HermiteSegment>() * n_segs + lut_total_bytes) as f64 / 1_048_576.0,
    );

    // --- Forward-Adjacency ---
    let adj = build_forward_adjacency(&segments);

    // --- Arc/Chord-Ratio-Histogramm ---
    println!("\n=== Arc-Length / Chord-Length Ratio (gesamt) ===");
    let ratios: Vec<f32> = segments
        .iter()
        .zip(luts.iter())
        .map(|(seg, lut)| lut.total_length_m / seg.length_m.max(0.001))
        .collect();
    let buckets = [
        (0.0f32, 1.001f32, "1.000 (exakt)"),
        (1.001, 1.01, "1.001-1.010"),
        (1.01, 1.05, "1.010-1.050"),
        (1.05, 1.10, "1.050-1.100"),
        (1.10, 1.25, "1.100-1.250"),
        (1.25, f32::INFINITY, "1.250+   "),
    ];
    for (lo, hi, label) in buckets {
        let count = ratios.iter().filter(|&&r| r >= lo && r < hi).count();
        let pct = count as f64 / ratios.len() as f64 * 100.0;
        let bar: String = "#".repeat((pct / 2.0) as usize);
        println!("  [{label}]: {:>7} ({:5.1}%) {bar}", count, pct);
    }
    let mean_ratio = ratios.iter().sum::<f32>() / ratios.len() as f32;
    let max_ratio = ratios.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
    println!("  Mean ratio: {mean_ratio:.4}  Max ratio: {max_ratio:.4}");

    // Generiere Abfragen als (seg_idx, s)-Paare: zufälliges Segment, zufälliger Bogen-Abstand
    let mut rng = Lcg(seed);
    // Segment-Indices (aus tatsächlichen Segment-LUTs)
    let query_pairs: Vec<(usize, f32)> = (0..n_queries)
        .map(|_| {
            let seg_idx = (rng.next_f32() * n_segs as f32) as usize % n_segs;
            let s_frac = rng.next_f32();
            let s = s_frac * luts[seg_idx].total_length_m;
            (seg_idx, s)
        })
        .collect();

    println!("\n=== Benchmark: {n_queries} Queries, seed={seed} ===");

    // --- point_at_arc_length Latenz ---
    let mut pal_times: Vec<f64> = Vec::with_capacity(n_queries);
    for &(seg_idx, s) in &query_pairs {
        let seg = &segments[seg_idx];
        let lut = &luts[seg_idx];
        let us = time_us(|| {
            let _ = point_at_arc_length(seg, lut, s);
        });
        pal_times.push(us);
    }
    pal_times.sort_by(|a, b| a.partial_cmp(b).unwrap());
    println!("\n--- point_at_arc_length ---");
    println!("  p50:  {:>8.2} µs", percentile(&pal_times, 50));
    println!("  p95:  {:>8.2} µs", percentile(&pal_times, 95));
    println!("  p99:  {:>8.2} µs", percentile(&pal_times, 99));
    println!(
        "  mean: {:>8.2} µs",
        pal_times.iter().sum::<f64>() / n_queries as f64
    );

    // --- lookahead(15m) Latenz ---
    let mut la_times: Vec<f64> = Vec::with_capacity(n_queries);
    for &(seg_idx, s) in &query_pairs {
        let seg = &segments[seg_idx];
        let lut = &luts[seg_idx];
        let t_start = t_at_arc_length(lut, seg, s);
        let us = time_us(|| {
            let _ = lookahead(seg_idx, t_start, 15.0, &adj, &segments, &luts);
        });
        la_times.push(us);
    }
    la_times.sort_by(|a, b| a.partial_cmp(b).unwrap());
    println!("\n--- lookahead(15m) ---");
    println!("  p50:  {:>8.2} µs", percentile(&la_times, 50));
    println!("  p95:  {:>8.2} µs", percentile(&la_times, 95));
    println!("  p99:  {:>8.2} µs", percentile(&la_times, 99));
    println!(
        "  mean: {:>8.2} µs",
        la_times.iter().sum::<f64>() / n_queries as f64
    );

    // Lookahead success rate
    let mut n_success = 0usize;
    let mut n_dead = 0usize;
    for &(seg_idx, s) in &query_pairs {
        let seg = &segments[seg_idx];
        let lut = &luts[seg_idx];
        let t_start = t_at_arc_length(lut, seg, s);
        if let Some(r) = lookahead(seg_idx, t_start, 15.0, &adj, &segments, &luts) {
            if r.remaining_dist_m == 0.0 {
                n_success += 1;
            } else {
                n_dead += 1;
            }
        }
    }
    println!(
        "  Success:  {} ({:.1}%)",
        n_success,
        n_success as f64 / n_queries as f64 * 100.0
    );
    println!(
        "  Dead-end: {} ({:.1}%)",
        n_dead,
        n_dead as f64 / n_queries as f64 * 100.0
    );

    // --- Parametrisierungs-Bias: point_at_arc_length(s=L/2) vs evaluate(t=0.5) ---
    // Auf 1000 zufälligen Segmenten
    let bias_n = n_queries.min(1000);
    let mut biases: Vec<f32> = Vec::with_capacity(bias_n);
    let mut rng2 = Lcg(seed ^ 0xDEAD_BEEF);
    for _ in 0..bias_n {
        let seg_idx = (rng2.next_f32() * n_segs as f32) as usize % n_segs;
        let seg = &segments[seg_idx];
        let lut = &luts[seg_idx];
        let s_mid = lut.total_length_m * 0.5;
        let p_arc = point_at_arc_length(seg, lut, s_mid);
        let p_param = evaluate(seg, 0.5);
        let dx = p_arc.x - p_param.x;
        let dy = p_arc.y - p_param.y;
        let dz = p_arc.z - p_param.z;
        let dist = (dx * dx + dy * dy + dz * dz).sqrt();
        biases.push(dist);
    }
    biases.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let bias_mean = biases.iter().sum::<f32>() / biases.len() as f32;
    let bias_p50 = biases[biases.len() / 2];
    let bias_p95 = biases[(biases.len() * 95) / 100];
    let bias_max = *biases.last().unwrap_or(&0.0);

    println!("\n--- Parametrisierungs-Bias: point_at_arc_length(s=L/2) vs evaluate(t=0.5) ---");
    println!("  (Differenz = Kurven-Verzerrung durch nicht-gleichmässige Bogenlängenverteilung)");
    println!("  N={bias_n} Segmente");
    println!("  Mean:  {bias_mean:.3} m");
    println!("  p50:   {bias_p50:.3} m");
    println!("  p95:   {bias_p95:.3} m");
    println!("  max:   {bias_max:.3} m");

    // Bias-Histogramm
    let bias_buckets: &[(f32, f32, &str)] = &[
        (0.0, 0.01, "0-0.01m (negligible)"),
        (0.01, 0.1, "0.01-0.1m"),
        (0.1, 0.5, "0.1-0.5m"),
        (0.5, 2.0, "0.5-2.0m"),
        (2.0, f32::INFINITY, "2.0m+   "),
    ];
    for &(lo, hi, label) in bias_buckets {
        let count = biases.iter().filter(|&&b| b >= lo && b < hi).count();
        let pct = count as f64 / biases.len() as f64 * 100.0;
        println!("  [{label}]: {:>6} ({:5.1}%)", count, pct);
    }

    // --- Build-Zeiten ---
    println!("\n=== Build-Zeiten ===");
    println!("  Spline-Build: {spline_build_ms:.0} ms");
    println!("  LUT-Build:    {lut_build_ms:.0} ms");
    println!("  Gesamt:       {:.0} ms", spline_build_ms + lut_build_ms);

    // Arc-Length STOP checks
    if lut_build_ms > 10_000.0 {
        eprintln!("STOP: LUT-Build > 10s ({lut_build_ms:.0} ms)! Parallelisierung nötig.");
        std::process::exit(2);
    }
    if percentile(&pal_times, 99) > 5000.0 {
        eprintln!("STOP: point_at_arc_length p99 > 5ms!");
        std::process::exit(2);
    }
    // u16-Quantisierungsfehler ist bereits durch Unit-Test gesichert (<0.1m bei 2km).
    eprintln!("\nAlle STOP-Bedingungen OK.");
}
