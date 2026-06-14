//! `quaternion-validate` — Spike 0a
//!
//! Validates the Quaternion→Heading convention from the clean-room spec
//! (Section 1.3) against real ETS2 sector data and live telemetry.
//!
//! ## Spec formula (Section 1.3, Y-up left-handed, `[qw, qx, qy, qz]`)
//! ```text
//! yaw_rad = atan2(−qy, qw) × 2 − π/2
//! ```
//! Convention: 0 = North (−Z), +π/2 = East (+X), +π/−π = South (+Z), −π/2 = West (−X).
//! CW positive.
//!
//! ## Acceptance criterion (Task 5)
//! For each of 6 cardinal anchors: `|diff_deg| < 0.5°` at a standstill on a straight edge.
//!
//! ## Usage
//! ```text
//! quaternion-validate --graph graph.json --ets2-dir "C:/..." compute-heading 12345678
//! quaternion-validate --graph graph.json list-cardinal-anchors --auto-discover
//! quaternion-validate --graph graph.json --ets2-dir "C:/..." live-compare
//! ```

use std::collections::HashMap;
use std::f32::consts::PI as PI_F32;
use std::path::PathBuf;

use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand};
use tracing::{info, warn};
use truckpilot_map_parser::archive::Archive;
use truckpilot_map_parser::hashfs::HashFsArchive;
use truckpilot_map_parser::sector::parse_sector;
use truckpilot_map_parser::{ModLoadOrder, ParseError, ZipArchive};

// ─────────────────────────────────────────────────────────────────────────────
// CLI
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Parser)]
#[command(
    name = "quaternion-validate",
    about = "Spike 0a — Validate Quaternion→Heading convention before Hermite build",
    long_about = "\
Reads node quaternions from ETS2 .scs archives and validates the spec formula \
(Section 1.3) against node geometry and live telemetry heading.\n\n\
STOP conditions:\n\
  - Node UID not in graph.json        → check --graph path\n\
  - Sector not readable               → check --ets2-dir path\n\
  - Quaternion is zero vector          → sized-format sector (no full quat)\n\
  - Telemetry unavailable              → ETS2 not running or DLL not loaded\n\
  - |diff| > 5° for any anchor        → STOP, spec drift — do NOT proceed to Hermite"
)]
struct Args {
    /// Path to graph.json (provides node positions for sector lookup)
    #[arg(long, default_value = "graph.json")]
    graph: PathBuf,

    /// Path to ETS2 installation directory (required for compute-heading + live-compare)
    #[arg(long)]
    ets2_dir: Option<PathBuf>,

    /// Path to ETS2 mods directory (optional; defaults to Documents/Euro Truck Simulator 2/mod)
    #[arg(long)]
    mods_dir: Option<PathBuf>,

    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Read a node's quaternion from the .scs sector, compute its heading from the spec formula.
    /// Requires --ets2-dir.
    ComputeHeading {
        /// Node UID (decimal or 0x-prefixed hex)
        #[arg(value_parser = parse_uid)]
        node_uid: u64,
    },

    /// Find candidate nodes for 6 cardinal directions (N/E/S/W/NE/SW) using
    /// edge direction vectors from graph.json.
    /// With --auto-discover: prints top-3 candidates per direction.
    /// Without: prints the hardcoded validated anchors (empty until committed).
    ListCardinalAnchors {
        /// Compute candidates from graph.json edge directions.
        #[arg(long)]
        auto_discover: bool,
    },

    /// Read live telemetry heading and compare it with quaternion-derived headings
    /// of the 5 nearest map nodes. Requires --ets2-dir and a running ETS2.
    LiveCompare,
}

fn parse_uid(s: &str) -> std::result::Result<u64, String> {
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u64::from_str_radix(hex, 16).map_err(|e| e.to_string())
    } else {
        s.parse::<u64>().map_err(|e| e.to_string())
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Heading math  (spec Section 1.3)
// ─────────────────────────────────────────────────────────────────────────────

/// Convert a quaternion `[qw, qx, qy, qz]` (Y-up, left-handed) to a heading in
/// radians using the spec formula from Section 1.3.
///
/// Sanity anchor: identity quaternion `(1, 0, 0, 0)` → **−π/2 rad = 270°** (West).
///
/// ```
/// # use std::f32::consts::FRAC_PI_2;
/// fn quat_to_heading_rad(qw: f32, _qx: f32, qy: f32, _qz: f32) -> f32 {
///     let pi = std::f32::consts::PI;
///     let mut yaw = f32::atan2(-qy, qw) * 2.0 - pi / 2.0;
///     while yaw > pi  { yaw -= 2.0 * pi; }
///     while yaw < -pi { yaw += 2.0 * pi; }
///     yaw
/// }
/// let h = quat_to_heading_rad(1.0, 0.0, 0.0, 0.0);
/// assert!((h - (-FRAC_PI_2)).abs() < 1e-5, "identity quat must give -π/2, got {h}");
/// ```
pub fn quat_to_heading_rad(qw: f32, _qx: f32, qy: f32, _qz: f32) -> f32 {
    let mut yaw = f32::atan2(-qy, qw) * 2.0 - PI_F32 / 2.0;
    while yaw > PI_F32 {
        yaw -= 2.0 * PI_F32;
    }
    while yaw < -PI_F32 {
        yaw += 2.0 * PI_F32;
    }
    yaw
}

/// Heading in radians → degrees in `[0, 360)`.
pub fn rad_to_deg360(rad: f32) -> f32 {
    (rad.to_degrees() + 360.0) % 360.0
}

/// Signed angular difference `a − b`, normalised to `(−180, 180]`.
pub fn heading_diff_deg(a_deg: f32, b_deg: f32) -> f32 {
    let mut d = a_deg - b_deg;
    while d > 180.0 {
        d -= 360.0;
    }
    while d <= -180.0 {
        d += 360.0;
    }
    d
}

/// Edge direction → heading in degrees `[0, 360)`.
/// ETS2 coordinate convention: X = East, Z = South (positive Z points south).
pub fn edge_heading_deg(from_x: f64, from_z: f64, to_x: f64, to_z: f64) -> f64 {
    let dx = to_x - from_x;
    let dz = to_z - from_z;
    let deg = f64::atan2(dx, -dz).to_degrees();
    (deg + 360.0) % 360.0
}

/// Four quaternion heading formula variants, all normalised to `[0°, 360°)`.
///
/// | Variant | Formula                       | Notes                       |
/// |---------|-------------------------------|-----------------------------|
/// | V1      | `atan2(−qy, qw)×2 − π/2`     | Spec Section 1.3 (current)  |
/// | V2      | `atan2(+qy, qw)×2`            | Positive qy, no offset      |
/// | V3      | `atan2(−qz, qw)×2 − π/2`     | Z-component instead of Y    |
/// | V4      | `atan2(−qx, qw)×2 − π/2`     | X-component instead of Y    |
///
/// Used in diagnostic `live-compare` to identify which axis/sign combo (if any)
/// matches telemetry when the spec formula does not.
pub fn quat_variants_deg(qw: f32, qx: f32, qy: f32, qz: f32) -> [f32; 4] {
    let normalise = |mut r: f32| -> f32 {
        while r > PI_F32 {
            r -= 2.0 * PI_F32;
        }
        while r < -PI_F32 {
            r += 2.0 * PI_F32;
        }
        rad_to_deg360(r)
    };
    [
        normalise(f32::atan2(-qy, qw) * 2.0 - PI_F32 / 2.0), // V1
        normalise(f32::atan2(qy, qw) * 2.0),                 // V2
        normalise(f32::atan2(-qz, qw) * 2.0 - PI_F32 / 2.0), // V3
        normalise(f32::atan2(-qx, qw) * 2.0 - PI_F32 / 2.0), // V4
    ]
}

/// Single-character quality flag for a heading diff.
fn flag(diff_abs: f32) -> &'static str {
    if diff_abs < 0.5 {
        "✓"
    } else if diff_abs < 5.0 {
        "!"
    } else {
        "✗"
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Graph JSON helpers
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Debug, serde::Deserialize, Clone)]
struct GNode {
    uid: u64,
    x: f64,
    y: f64,
    z: f64,
}

#[derive(Debug, serde::Deserialize)]
struct GEdge {
    from: u64,
    to: u64,
}

#[derive(Debug, serde::Deserialize)]
struct GraphJson {
    nodes: Vec<GNode>,
    edges: Vec<GEdge>,
}

fn load_graph(path: &PathBuf) -> Result<GraphJson> {
    if !path.exists() {
        bail!(
            "STOP: graph.json not found at '{}'. Pass --graph <path>.",
            path.display()
        );
    }
    let bytes = std::fs::read(path).with_context(|| format!("read {}", path.display()))?;
    serde_json::from_slice(&bytes).context("parse graph.json")
}

fn build_node_map(g: &GraphJson) -> HashMap<u64, &GNode> {
    g.nodes.iter().map(|n| (n.uid, n)).collect()
}

/// `(to_uid, geometric_heading_deg, edge_length_m)`
type EdgeEntry = (u64, f64, f64);

/// Build map: `source_node_uid → Vec<(to_uid, heading_deg, length_m)>` for
/// every directed edge in `graph.json`.  Used by diagnostic `live-compare`.
fn build_edge_adjacency(
    graph: &GraphJson,
    node_map: &HashMap<u64, &GNode>,
) -> HashMap<u64, Vec<EdgeEntry>> {
    let mut adj: HashMap<u64, Vec<EdgeEntry>> = HashMap::new();
    for edge in &graph.edges {
        let (Some(from_n), Some(to_n)) = (node_map.get(&edge.from), node_map.get(&edge.to)) else {
            continue;
        };
        let dx = to_n.x - from_n.x;
        let dy = to_n.y - from_n.y;
        let dz = to_n.z - from_n.z;
        let len = (dx * dx + dy * dy + dz * dz).sqrt();
        let hdg = edge_heading_deg(from_n.x, from_n.z, to_n.x, to_n.z);
        adj.entry(edge.from).or_default().push((edge.to, hdg, len));
    }
    adj
}

// ─────────────────────────────────────────────────────────────────────────────
// Sector / archive helpers
// ─────────────────────────────────────────────────────────────────────────────

fn format_sector_path(sx: i32, sz: i32) -> String {
    let fmt = |c: i32| -> String {
        if c >= 0 {
            format!("+{c:04}")
        } else {
            format!("-{:04}", c.unsigned_abs())
        }
    };
    format!("map/europe/sec{}{}.base", fmt(sx), fmt(sz))
}

/// Derive the primary sector path for a world position (in meters).
/// Each ETS2 sector covers 4096 × 4096 meters.
fn sector_path_from_position(x: f64, z: f64) -> String {
    const SECTOR_SIZE: f64 = 4096.0;
    let sx = (x / SECTOR_SIZE).floor() as i32;
    let sz = (z / SECTOR_SIZE).floor() as i32;
    format_sector_path(sx, sz)
}

fn open_archives(
    ets2_dir: &std::path::Path,
    mods_dir: Option<&PathBuf>,
) -> Result<Vec<Box<dyn Archive>>> {
    let default_mods = dirs::document_dir()
        .unwrap_or_default()
        .join("Euro Truck Simulator 2/mod");
    let mods: &std::path::Path = mods_dir
        .map(|p| p.as_path())
        .unwrap_or(default_mods.as_path());

    let order = ModLoadOrder::from_directories(ets2_dir, mods)
        .with_context(|| format!("discovering archives in {}", ets2_dir.display()))?;

    let mut archives: Vec<Box<dyn Archive>> = Vec::new();
    for entry in &order.entries {
        match HashFsArchive::open(&entry.path) {
            Ok(a) => archives.push(Box::new(a)),
            Err(ParseError::InvalidMagic(_)) => match ZipArchive::open(&entry.path) {
                Ok(a) => archives.push(Box::new(a)),
                Err(e) => warn!("skip {}: {e}", entry.path.display()),
            },
            Err(e) => warn!("skip {}: {e}", entry.path.display()),
        }
    }

    if archives.is_empty() {
        bail!(
            "STOP: No readable archives found in '{}'. Check --ets2-dir.",
            ets2_dir.display()
        );
    }
    info!("Opened {} archives", archives.len());
    Ok(archives)
}

/// Try to find a node by UID in one specific sector (across all archives).
/// Returns `Some([qw, qx, qy, qz])` if found; `None` if the sector doesn't
/// contain that node or the sector file doesn't exist.
fn try_find_in_sector(
    archives: &mut [Box<dyn Archive>],
    sector_path: &str,
    target_uid: u64,
) -> Result<Option<[f32; 4]>> {
    // Try archives in reverse priority (last loaded = highest priority, like mod override).
    let data = archives
        .iter_mut()
        .rev()
        .find_map(|arc| arc.read_path(sector_path).ok());

    let Some(data) = data else {
        return Ok(None);
    };

    let sector = parse_sector(&data).with_context(|| format!("parse {sector_path}"))?;

    for node in &sector.nodes {
        if node.uid == target_uid {
            return Ok(Some(node.rotation));
        }
    }
    Ok(None)
}

/// Find the quaternion of a node by scanning the position-derived sector plus
/// its 8 immediate neighbours (handles nodes near sector boundaries).
fn find_node_quaternion(
    archives: &mut [Box<dyn Archive>],
    uid: u64,
    pos_x: f64,
    pos_z: f64,
) -> Result<[f32; 4]> {
    const SECTOR_SIZE: f64 = 4096.0;
    let sx = (pos_x / SECTOR_SIZE).floor() as i32;
    let sz = (pos_z / SECTOR_SIZE).floor() as i32;

    let offsets: &[(i32, i32)] = &[
        (0, 0),
        (-1, 0),
        (1, 0),
        (0, -1),
        (0, 1),
        (-1, -1),
        (-1, 1),
        (1, -1),
        (1, 1),
    ];

    for &(dx, dz) in offsets {
        let path = format_sector_path(sx + dx, sz + dz);
        match try_find_in_sector(archives, &path, uid) {
            Ok(Some(q)) => {
                if dx != 0 || dz != 0 {
                    info!("Node {uid} found in adjacent sector {path}");
                }
                return Ok(q);
            }
            Ok(None) => {}
            Err(e) => warn!("Error reading {path}: {e}"),
        }
    }

    bail!(
        "STOP: Node {uid} not found in sector {} or any of its 8 neighbours. \
         Verify --ets2-dir is correct and the map is fully installed.",
        format_sector_path(sx, sz)
    )
}

// ─────────────────────────────────────────────────────────────────────────────
// Subcommand: compute-heading
// ─────────────────────────────────────────────────────────────────────────────

fn cmd_compute_heading(args: &Args, node_uid: u64) -> Result<()> {
    let ets2_dir = args
        .ets2_dir
        .as_ref()
        .context("--ets2-dir is required for compute-heading")?;

    eprintln!("[compute-heading] loading graph.json…");
    let graph = load_graph(&args.graph)?;
    let node_map = build_node_map(&graph);

    let node = node_map
        .get(&node_uid)
        .with_context(|| format!("STOP: Node {node_uid} not found in graph.json"))?;

    let primary_sector = sector_path_from_position(node.x, node.z);
    eprintln!(
        "[compute-heading] node {node_uid} at ({:.1}, {:.1}, {:.1}) → primary sector: {primary_sector}",
        node.x, node.y, node.z
    );

    let mut archives = open_archives(ets2_dir, args.mods_dir.as_ref())?;
    let quat = find_node_quaternion(&mut archives, node_uid, node.x, node.z)?;
    let [qw, qx, qy, qz] = quat;

    let is_zero = qw.abs() < 1e-6 && qx.abs() < 1e-6 && qy.abs() < 1e-6 && qz.abs() < 1e-6;
    let heading_rad = quat_to_heading_rad(qw, qx, qy, qz);
    let heading_deg = rad_to_deg360(heading_rad);

    println!();
    println!("Node UID      : {node_uid}");
    println!(
        "Position      : x={:.3}  y={:.3}  z={:.3}",
        node.x, node.y, node.z
    );
    println!("Quaternion    : qw={qw:.6}  qx={qx:.6}  qy={qy:.6}  qz={qz:.6}");

    if is_zero {
        println!("WARNING       : Quaternion is zero vector — sized-format sector has no full quaternion.");
        println!("               Heading is INVALID for this node.");
    } else {
        println!("Heading (rad) : {heading_rad:.6}");
        println!("Heading (deg) : {heading_deg:.2}°");
    }
    println!();

    Ok(())
}

// ─────────────────────────────────────────────────────────────────────────────
// Subcommand: list-cardinal-anchors
// ─────────────────────────────────────────────────────────────────────────────

/// Hardcoded validated anchors committed after live testing.
/// Format: (direction, node_uid, expected_heading_deg).
/// Empty until the first successful live-compare run.
const VALIDATED_ANCHORS: &[(&str, u64, f64)] = &[
    // ("N",  0xDEADBEEF00000001, 0.0),
    // ("E",  0xDEADBEEF00000002, 90.0),
    // etc.
];

/// (direction_label, target_heading_deg, tolerance_deg)
const CARDINALS: &[(&str, f64, f64)] = &[
    ("N", 0.0, 5.0),
    ("E", 90.0, 5.0),
    ("S", 180.0, 5.0),
    ("W", 270.0, 5.0),
    ("NE", 45.0, 5.0),
    ("SW", 225.0, 5.0),
];

fn cmd_list_cardinal_anchors(args: &Args, auto_discover: bool) -> Result<()> {
    if !auto_discover {
        if VALIDATED_ANCHORS.is_empty() {
            println!("No hardcoded anchors defined yet.");
            println!("Run with --auto-discover to find candidates, then commit the UIDs to VALIDATED_ANCHORS.");
        } else {
            println!("{:<4}  {:>18}  {:>10}", "Dir", "Node UID", "Expected°");
            println!("{}", "-".repeat(40));
            for (dir, uid, hdg) in VALIDATED_ANCHORS {
                println!("{dir:<4}  {uid:>18}  {hdg:>10.1}");
            }
        }
        return Ok(());
    }

    eprintln!("[list-cardinal-anchors] loading graph.json…");
    let graph = load_graph(&args.graph)?;
    eprintln!(
        "[list-cardinal-anchors] {} nodes, {} edges",
        graph.nodes.len(),
        graph.edges.len()
    );

    let node_map: HashMap<u64, &GNode> = graph.nodes.iter().map(|n| (n.uid, n)).collect();

    // Build (from_uid, edge_heading_deg, edge_length_m) for all long edges.
    let mut edge_data: Vec<(u64, f64, f64)> = Vec::with_capacity(graph.edges.len());
    for edge in &graph.edges {
        let (Some(from), Some(to)) = (node_map.get(&edge.from), node_map.get(&edge.to)) else {
            continue;
        };
        let dx = to.x - from.x;
        let dy = to.y - from.y;
        let dz = to.z - from.z;
        let len = (dx * dx + dy * dy + dz * dz).sqrt();
        if len < 100.0 {
            continue; // skip short or point edges
        }
        let hdg = edge_heading_deg(from.x, from.z, to.x, to.z);
        edge_data.push((edge.from, hdg, len));
    }
    eprintln!(
        "[list-cardinal-anchors] {} edges with length ≥ 100 m",
        edge_data.len()
    );

    println!();
    println!(
        "{:<4}  {:>18}  {:>10.1}  {:>10.1}  {:>10.1}  {:>12.1}  {:>12.1}",
        "Dir", "Node UID", "Target°", "Actual°", "Diff°", "X (m)", "Z (m)"
    );
    println!("{}", "─".repeat(95));

    for &(dir_name, target_deg, tolerance) in CARDINALS {
        // Collect edges within tolerance; score = angular distance (lower=better),
        // tie-break by longer edge (likely straighter road).
        let mut candidates: Vec<(f64, f64, u64)> = edge_data
            .iter()
            .filter_map(|&(uid, hdg, len)| {
                let mut diff = (hdg - target_deg).abs();
                if diff > 180.0 {
                    diff = 360.0 - diff;
                }
                if diff <= tolerance {
                    // score: prioritise small angular error and long edges
                    Some((diff - len * 0.001, hdg, uid))
                } else {
                    None
                }
            })
            .collect();

        if candidates.is_empty() {
            println!(
                "{dir_name:<4}  {:>18}  {:>10.1}  {:>10}  {:>10}  {:>12}  {:>12}",
                "—", target_deg, "no match", "—", "—", "—"
            );
            println!();
            continue;
        }

        candidates.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        candidates.truncate(3);

        for (i, &(_, actual_hdg, uid)) in candidates.iter().enumerate() {
            let node = node_map[&uid];
            let diff = {
                let mut d = actual_hdg - target_deg;
                if d > 180.0 {
                    d -= 360.0;
                }
                if d < -180.0 {
                    d += 360.0;
                }
                d
            };
            let len_m = edge_data.iter().find(|e| e.0 == uid).map_or(0.0, |e| e.2);

            if i == 0 {
                println!(
                    "{dir_name:<4}  {uid:>18}  {target_deg:>10.1}  {actual_hdg:>10.2}  {diff:>+10.2}  {x:>12.1}  {z:>12.1}  (len={len_m:.0}m)",
                    x = node.x,
                    z = node.z,
                );
            } else {
                println!(
                    "     {uid:>18}  {:>10}  {actual_hdg:>10.2}  {diff:>+10.2}  {x:>12.1}  {z:>12.1}  (len={len_m:.0}m)",
                    "",
                    x = node.x,
                    z = node.z,
                );
            }
        }
        println!();
    }

    println!("Next steps:");
    println!("  1. Pick one UID per direction from the candidates above.");
    println!("  2. Drive the truck to that position in ETS2, align with the road, stop.");
    println!("  3. Run:  quaternion-validate --graph graph.json --ets2-dir <...> live-compare");
    println!("  4. Record results in outputs/2026-05-24/spike_0a_quaternion_results.md");
    println!("  5. If |diff| < 0.5° for all 6: commit UIDs to VALIDATED_ANCHORS in this file.");

    Ok(())
}

// ─────────────────────────────────────────────────────────────────────────────
// Subcommand: live-compare  (diagnostic mode — all heading sources, no STOP)
// ─────────────────────────────────────────────────────────────────────────────

/// Print one row of the diagnostic heading table.
#[allow(clippy::too_many_arguments)]
fn print_diag_row(
    uid: u64,
    dist: f64,
    qw_str: &str,
    node_yaw: Option<f32>,
    edge_geo: Option<f32>,
    telem_deg: f32,
    diff_node: Option<f32>,
    diff_edge: Option<f32>,
) {
    let ny = node_yaw.map_or_else(|| "—".to_string(), |v| format!("{v:.2}"));
    let eg = edge_geo.map_or_else(|| "—".to_string(), |v| format!("{v:.2}"));
    let dn = diff_node.map_or_else(|| "—".to_string(), |v| format!("{v:+.2}"));
    let de = diff_edge.map_or_else(|| "—".to_string(), |v| format!("{v:+.2}"));

    let best_abs: Option<f32> = match (diff_node.map(f32::abs), diff_edge.map(f32::abs)) {
        (Some(a), Some(b)) => Some(a.min(b)),
        (Some(a), None) | (None, Some(a)) => Some(a),
        (None, None) => None,
    };
    let status = best_abs.map_or("—", flag);

    println!(
        "{:>18}  {:>7.1}  {:>8}  {:>10}  {:>10}  {:>8.2}  {:>9}  {:>9}  {}",
        uid, dist, qw_str, ny, eg, telem_deg, dn, de, status
    );
}

fn cmd_live_compare(args: &Args) -> Result<()> {
    let ets2_dir = args
        .ets2_dir
        .as_ref()
        .context("--ets2-dir is required for live-compare")?;

    eprintln!("[live-compare] reading telemetry…");
    let telem = truckpilot_telemetry::read_telemetry().ok_or_else(|| {
        anyhow::anyhow!(
            "STOP: Telemetry not available.\n\
             Ensure ETS2 is running and the TruckPilot DLL (truckpilot_telemetry.dll) is installed."
        )
    })?;
    let [tx, ty, tz] = telem.position;
    let telem_deg = rad_to_deg360(telem.heading as f32);

    println!();
    println!("Telemetry snapshot:");
    println!("  position : ({tx:.2}, {ty:.2}, {tz:.2}) m");
    println!("  heading  : {:.4} rad  =  {telem_deg:.2}°", telem.heading);
    println!("  speed    : {:.1} km/h", telem.speed_ms * 3.6);
    println!();

    eprintln!("[live-compare] loading graph.json…");
    let graph = load_graph(&args.graph)?;
    eprintln!(
        "[live-compare] {} nodes, {} edges — finding nearest 5…",
        graph.nodes.len(),
        graph.edges.len()
    );

    let node_map = build_node_map(&graph);
    let edge_adj = build_edge_adjacency(&graph, &node_map);

    // O(N) nearest scan — acceptable for a one-shot diagnostic tool.
    let mut scored: Vec<(f64, &GNode)> = graph
        .nodes
        .iter()
        .map(|n| {
            let dx = n.x - tx;
            let dy = n.y - ty;
            let dz = n.z - tz;
            (dx * dx + dy * dy + dz * dz, n)
        })
        .collect();
    scored.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    scored.truncate(5);

    eprintln!("[live-compare] opening archives…");
    let mut archives = open_archives(ets2_dir, args.mods_dir.as_ref())?;

    // ── Header ────────────────────────────────────────────────────────────────
    println!(
        "{:>18}  {:>7}  {:>8}  {:>10}  {:>10}  {:>8}  {:>9}  {:>9}  Status",
        "Node UID", "Dist(m)", "qw", "NodeYaw°", "EdgeGeo°", "Telem°", "DiffNode°", "DiffEdge°"
    );
    println!("{}", "─".repeat(104));

    for (dist_sq, node) in &scored {
        let dist = dist_sq.sqrt();
        let edges = edge_adj.get(&node.uid).cloned().unwrap_or_default();

        // Best outgoing edge = longest (most stable heading signal).
        let best_edge = edges
            .iter()
            .max_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal))
            .copied();
        let edge_geo_deg = best_edge.map(|(_, hdg, _)| hdg as f32);
        let diff_edge = edge_geo_deg.map(|eg| heading_diff_deg(telem_deg, eg));

        match find_node_quaternion(&mut archives, node.uid, node.x, node.z) {
            Ok([qw, qx, qy, qz]) => {
                let is_zero =
                    qw.abs() < 1e-6 && qx.abs() < 1e-6 && qy.abs() < 1e-6 && qz.abs() < 1e-6;

                if is_zero {
                    print_diag_row(
                        node.uid,
                        dist,
                        "sized-fmt",
                        None,
                        edge_geo_deg,
                        telem_deg,
                        None,
                        diff_edge,
                    );
                    println!("  [sized-format sector — no full quaternion stored]");
                } else {
                    let variants = quat_variants_deg(qw, qx, qy, qz);
                    let diff_node = heading_diff_deg(telem_deg, variants[0]);
                    print_diag_row(
                        node.uid,
                        dist,
                        &format!("{qw:.4}"),
                        Some(variants[0]),
                        edge_geo_deg,
                        telem_deg,
                        Some(diff_node),
                        diff_edge,
                    );

                    // ── Quaternion variants (diagnostic, no STOP) ─────────
                    println!(
                        "  └─ quat [qw={qw:.4} qx={qx:.4} qy={qy:.4} qz={qz:.4}]  \
                         [diag-mode: all variants shown, STOP logic suspended]"
                    );
                    let labels = [
                        "V1 atan2(−qy,qw)×2−π/2  (spec §1.3)",
                        "V2 atan2(+qy,qw)×2       (no minus, no offset)",
                        "V3 atan2(−qz,qw)×2−π/2  (Z-axis variant)",
                        "V4 atan2(−qx,qw)×2−π/2  (X-axis variant)",
                    ];
                    for (label, &vdeg) in labels.iter().zip(variants.iter()) {
                        let d = heading_diff_deg(telem_deg, vdeg);
                        println!(
                            "     {}  → {:>7.2}°  diff={:>+7.2}°  {}",
                            label,
                            vdeg,
                            d,
                            flag(d.abs())
                        );
                    }

                    // ── Edge geometry ─────────────────────────────────────
                    if edges.is_empty() {
                        println!("  └─ no outgoing edges in graph.json for this node");
                    } else {
                        println!("  └─ {} outgoing edge(s):", edges.len());
                        for &(to_uid, hdg, len) in &edges {
                            let d = heading_diff_deg(telem_deg, hdg as f32);
                            println!(
                                "     → {:>18}  hdg={:>7.2}°  len={:>7.1}m  diff={:>+7.2}°  {}",
                                to_uid,
                                hdg,
                                len,
                                d,
                                flag(d.abs())
                            );
                        }
                    }
                }
            }
            Err(e) => {
                print_diag_row(
                    node.uid,
                    dist,
                    "ERR",
                    None,
                    edge_geo_deg,
                    telem_deg,
                    None,
                    diff_edge,
                );
                eprintln!("  node {} quaternion lookup failed: {e}", node.uid);
            }
        }
        println!();
    }

    println!("Legend: ✓ = |diff| < 0.5°   ! = |diff| < 5°   ✗ = |diff| ≥ 5°");
    println!("Status = min(|DiffNode°|, |DiffEdge°|) — best available source vs. telemetry.");
    println!();
    println!(
        "Diagnosis key: if EdgeGeo° ✓ and NodeYaw° ✗ → road direction = edge geometry,\n\
         NOT node rotation quaternion.  Spec §1.3 formula is valid math on the wrong source."
    );

    Ok(())
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::INFO)
        .with_writer(std::io::stderr)
        .init();

    let args = Args::parse();

    match &args.cmd {
        Cmd::ComputeHeading { node_uid } => cmd_compute_heading(&args, *node_uid),
        Cmd::ListCardinalAnchors { auto_discover } => {
            cmd_list_cardinal_anchors(&args, *auto_discover)
        }
        Cmd::LiveCompare => cmd_live_compare(&args),
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Identity quaternion (1, 0, 0, 0):
    /// yaw = atan2(−0, 1) × 2 − π/2 = −π/2 rad = 270° (West).
    /// This is the primary sanity anchor from the spec.
    #[test]
    fn identity_quaternion_is_west() {
        let h_rad = quat_to_heading_rad(1.0, 0.0, 0.0, 0.0);
        assert!(
            (h_rad - (-PI_F32 / 2.0)).abs() < 1e-5,
            "identity quat: expected −π/2 rad, got {h_rad}"
        );
        let h_deg = rad_to_deg360(h_rad);
        assert!(
            (h_deg - 270.0).abs() < 0.01,
            "identity quat: expected 270°, got {h_deg}°"
        );
    }

    /// Pure Y-rotation by π/2 (qw = cos(π/4), qy = sin(π/4)):
    /// yaw = atan2(−sin(π/4), cos(π/4)) × 2 − π/2
    ///     = atan2(−1, 1) × 2 − π/2
    ///     = (−π/4) × 2 − π/2
    ///     = −π/2 − π/2 = −π rad = 180° (South).
    #[test]
    fn pi_over_2_y_rotation_is_south() {
        let qw = (PI_F32 / 4.0).cos();
        let qy = (PI_F32 / 4.0).sin();
        let h_deg = rad_to_deg360(quat_to_heading_rad(qw, 0.0, qy, 0.0));
        assert!(
            (h_deg - 180.0).abs() < 0.5,
            "π/2 Y-rotation: expected 180° (South), got {h_deg}°"
        );
    }

    /// Output is always in [−π, π].
    #[test]
    fn heading_range_bounded() {
        let cases = [
            (1.0f32, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.707, 0.0, 0.707, 0.0),
            (-0.707, 0.0, 0.707, 0.0),
        ];
        for (qw, qx, qy, qz) in cases {
            let h = quat_to_heading_rad(qw, qx, qy, qz);
            assert!(
                (-PI_F32..=PI_F32).contains(&h),
                "({qw},{qx},{qy},{qz}) → {h} out of [−π,π]"
            );
        }
    }

    #[test]
    fn edge_heading_north() {
        let h = edge_heading_deg(0.0, 0.0, 0.0, -100.0);
        assert!((h - 0.0).abs() < 0.01, "expected 0° (N), got {h}°");
    }

    #[test]
    fn edge_heading_east() {
        let h = edge_heading_deg(0.0, 0.0, 100.0, 0.0);
        assert!((h - 90.0).abs() < 0.01, "expected 90° (E), got {h}°");
    }

    #[test]
    fn edge_heading_south() {
        let h = edge_heading_deg(0.0, 0.0, 0.0, 100.0);
        assert!((h - 180.0).abs() < 0.01, "expected 180° (S), got {h}°");
    }

    #[test]
    fn edge_heading_west() {
        let h = edge_heading_deg(0.0, 0.0, -100.0, 0.0);
        assert!((h - 270.0).abs() < 0.01, "expected 270° (W), got {h}°");
    }

    #[test]
    fn heading_diff_wraps_correctly() {
        assert!((heading_diff_deg(5.0, 355.0) - 10.0).abs() < 0.01);
        assert!((heading_diff_deg(355.0, 5.0) - (-10.0)).abs() < 0.01);
        assert!((heading_diff_deg(180.0, 0.0) - 180.0).abs() < 0.01);
    }

    /// Sector path formula: floor(x / 4096).
    #[test]
    fn sector_path_origin() {
        assert_eq!(
            sector_path_from_position(0.0, 0.0),
            "map/europe/sec+0000+0000.base"
        );
    }

    #[test]
    fn sector_path_positive() {
        // floor(4097/4096)=1, floor(8192/4096)=2
        assert_eq!(
            sector_path_from_position(4097.0, 8192.0),
            "map/europe/sec+0001+0002.base"
        );
    }

    #[test]
    fn sector_path_berlin_approx() {
        // floor(−45117/4096)=−12, floor(−8353/4096)=−3
        assert_eq!(
            sector_path_from_position(-45117.0, -8353.0),
            "map/europe/sec-0012-0003.base"
        );
    }

    /// V1 of quat_variants_deg must produce the same value as quat_to_heading_rad.
    #[test]
    fn variants_v1_matches_existing_formula() {
        for &(qw, qx, qy, qz) in &[
            (1.0f32, 0.0, 0.0, 0.0),
            (0.707, 0.0, 0.707, 0.0),
            (0.0, 0.0, 1.0, 0.0),
        ] {
            let v1 = quat_variants_deg(qw, qx, qy, qz)[0];
            let expected = rad_to_deg360(quat_to_heading_rad(qw, qx, qy, qz));
            assert!(
                (v1 - expected).abs() < 0.1,
                "V1={v1}° expected={expected}° for quat ({qw},{qx},{qy},{qz})"
            );
        }
    }

    /// All four variants must stay in [0°, 360°) for arbitrary inputs.
    #[test]
    fn variants_all_in_range() {
        let cases = [
            (1.0f32, 0.0, 0.0, 0.0),
            (0.707, 0.0, 0.707, 0.0),
            (-0.707, 0.0, 0.707, 0.0),
            (0.0, 0.707, 0.0, 0.707),
        ];
        for (qw, qx, qy, qz) in cases {
            for (i, v) in quat_variants_deg(qw, qx, qy, qz).iter().enumerate() {
                assert!(
                    *v >= 0.0 && *v < 360.0,
                    "V{} = {v}° out of [0°,360°) for quat ({qw},{qx},{qy},{qz})",
                    i + 1
                );
            }
        }
    }
}
