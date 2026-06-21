//! ets2la-route-dump — liest Local\ETS2LARoute (ETS2LA Shared Memory)
//! und matched die Node-UIDs gegen TruckPilot's graph.json.
//!
//! Schritt 1+2 des ETS2LA-Ansatzes:
//!   1. Rust liest Local\ETS2LARoute  → RouteItems (uid, distance, time)
//!   2. UIDs gegen graph.json matchen → Match-Rate prüfen
//!
//! Usage:
//!   cargo run --release -p truckpilot-diag --bin ets2la-route-dump
//!   cargo run --release -p truckpilot-diag --bin ets2la-route-dump -- --graph graph.json --csv outputs/2026-06-21/route.csv

use std::collections::HashMap;
use std::path::PathBuf;

use anyhow::{bail, Context, Result};
use clap::Parser;

// ETS2LARoute SHM layout:
//   6000 × RouteItem, each 16 bytes:
//     i64 uid       (8 bytes, little-endian)
//     f32 distance  (4 bytes, little-endian)
//     f32 time      (4 bytes, little-endian)
//   First uid == 0 marks end of route.
const SHM_NAME: &str = "Local\\ETS2LARoute";
const ROUTE_BUFFER_BYTES: usize = 96_000;
const ROUTE_ITEM_BYTES: usize = 16;
const MAX_ROUTE_ITEMS: usize = 6000;

#[derive(Debug, Clone)]
struct RouteItem {
    uid: u64,
    distance: f32,
    time: f32,
}

#[derive(Parser)]
#[command(name = "ets2la-route-dump", about = "Reads Local\\ETS2LARoute and matches UIDs against graph.json")]
struct Args {
    /// Path to graph.json (TruckPilot's built routing graph)
    #[arg(long, default_value = "graph.json")]
    graph: PathBuf,

    /// Write matched route as CSV (optional)
    #[arg(long)]
    csv: Option<PathBuf>,

    /// How many items to print in detail (0 = none, default 20)
    #[arg(long, default_value = "20")]
    show: usize,

    /// Poll continuously every N ms instead of one-shot (0 = one-shot)
    #[arg(long, default_value = "0")]
    poll_ms: u64,
}

fn main() -> Result<()> {
    let args = Args::parse();

    // Load graph nodes into uid → (x, y, z) map
    let nodes = load_graph_nodes(&args.graph)
        .with_context(|| format!("loading {}", args.graph.display()))?;
    println!("Graph nodes loaded: {}", nodes.len());
    println!();

    if args.poll_ms == 0 {
        run_once(&args, &nodes)?;
    } else {
        loop {
            if let Err(e) = run_once(&args, &nodes) {
                println!("ERROR: {e}");
            }
            std::thread::sleep(std::time::Duration::from_millis(args.poll_ms));
            println!("---");
        }
    }

    Ok(())
}

fn run_once(args: &Args, nodes: &HashMap<u64, [f32; 3]>) -> Result<()> {
    let items = read_shm()?;

    let total = items.len();
    let matched: Vec<_> = items
        .iter()
        .filter(|it| nodes.contains_key(&it.uid))
        .collect();
    let match_count = matched.len();
    let match_pct = if total > 0 {
        match_count as f64 / total as f64 * 100.0
    } else {
        0.0
    };

    println!("Route items:  {total}");
    println!("Matched UIDs: {match_count} / {total} ({match_pct:.1}%)");
    println!();

    // Print first N items
    if args.show > 0 {
        println!("{:<6} {:<20} {:>10} {:>8}  {:<14}  coords", "idx", "uid", "dist_m", "time_s", "status");
        println!("{}", "-".repeat(80));
        for (i, item) in items.iter().enumerate().take(args.show) {
            let status = if nodes.contains_key(&item.uid) { "OK" } else { "MISSING" };
            let coords = nodes
                .get(&item.uid)
                .map(|p| format!("({:.0},{:.0},{:.0})", p[0], p[1], p[2]))
                .unwrap_or_else(|| "-".to_string());
            println!(
                "{:<6} {:<20} {:>10.1} {:>8.1}  {:<14}  {}",
                i, item.uid, item.distance, item.time, status, coords
            );
        }
        if total > args.show {
            println!("... ({} more)", total - args.show);
        }
        println!();
    }

    // CSV export
    if let Some(csv_path) = &args.csv {
        if let Some(parent) = csv_path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let mut out = String::new();
        out.push_str("idx,uid,distance_m,time_s,matched,x,y,z\n");
        for (i, item) in items.iter().enumerate() {
            let (matched_flag, x, y, z) = if let Some(p) = nodes.get(&item.uid) {
                (1, p[0], p[1], p[2])
            } else {
                (0, 0.0, 0.0, 0.0)
            };
            out.push_str(&format!(
                "{},{},{:.2},{:.2},{},{:.3},{:.3},{:.3}\n",
                i, item.uid, item.distance, item.time, matched_flag, x, y, z
            ));
        }
        std::fs::write(csv_path, &out)?;
        println!("CSV written: {}", csv_path.display());
    }

    Ok(())
}

/// Load graph.json and return uid → [x, y, z] for all nodes.
fn load_graph_nodes(path: &PathBuf) -> Result<HashMap<u64, [f32; 3]>> {
    let data = std::fs::read(path)?;
    let v: serde_json::Value = serde_json::from_slice(&data)?;
    let mut map = HashMap::new();

    let nodes = v
        .get("nodes")
        .and_then(|n| n.as_array())
        .with_context(|| "graph.json has no 'nodes' array")?;

    for node in nodes {
        let uid = node.get("uid").and_then(|u| u.as_u64());
        let x = node.get("x").and_then(|v| v.as_f64()).map(|f| f as f32);
        let y = node.get("y").and_then(|v| v.as_f64()).map(|f| f as f32);
        let z = node.get("z").and_then(|v| v.as_f64()).map(|f| f as f32);
        if let (Some(uid), Some(x), Some(y), Some(z)) = (uid, x, y, z) {
            map.insert(uid, [x, y, z]);
        }
    }

    if map.is_empty() {
        bail!("No nodes found in graph.json — check format");
    }
    Ok(map)
}

/// Read Local\ETS2LARoute shared memory and return parsed RouteItems.
#[cfg(windows)]
fn read_shm() -> Result<Vec<RouteItem>> {
    use std::ffi::c_void;

    extern "system" {
        fn OpenFileMappingA(
            dwDesiredAccess: u32,
            bInheritHandle: i32,
            lpName: *const u8,
        ) -> *mut c_void;
        fn MapViewOfFile(
            hFileMappingObject: *mut c_void,
            dwDesiredAccess: u32,
            dwFileOffsetHigh: u32,
            dwFileOffsetLow: u32,
            dwNumberOfBytesToMap: usize,
        ) -> *mut u8;
        fn UnmapViewOfFile(lpBaseAddress: *const u8) -> i32;
        fn CloseHandle(hObject: *mut c_void) -> i32;
    }

    const FILE_MAP_READ: u32 = 0x0004;

    let name_cstr = std::ffi::CString::new(SHM_NAME).unwrap();

    unsafe {
        let handle = OpenFileMappingA(FILE_MAP_READ, 0, name_cstr.as_ptr() as *const u8);
        if handle.is_null() {
            bail!(
                "Could not open '{}'\n  → Is ETS2LA (or a compatible route writer) running?\n  → Is a route set in-game?",
                SHM_NAME
            );
        }

        let view = MapViewOfFile(handle, FILE_MAP_READ, 0, 0, ROUTE_BUFFER_BYTES);
        if view.is_null() {
            CloseHandle(handle);
            bail!("MapViewOfFile failed for '{}'", SHM_NAME);
        }

        let bytes = std::slice::from_raw_parts(view, ROUTE_BUFFER_BYTES);
        let mut items = Vec::new();

        for chunk in bytes.chunks_exact(ROUTE_ITEM_BYTES).take(MAX_ROUTE_ITEMS) {
            let uid_raw = i64::from_le_bytes(chunk[0..8].try_into().unwrap());
            if uid_raw == 0 {
                break; // end-of-route sentinel
            }
            let uid = uid_raw as u64;
            let distance = f32::from_le_bytes(chunk[8..12].try_into().unwrap());
            let time = f32::from_le_bytes(chunk[12..16].try_into().unwrap());
            items.push(RouteItem { uid, distance, time });
        }

        UnmapViewOfFile(view);
        CloseHandle(handle);

        Ok(items)
    }
}

#[cfg(not(windows))]
fn read_shm() -> Result<Vec<RouteItem>> {
    bail!("Local\\ETS2LARoute is Windows-only (requires ETS2 + ETS2LA running)");
}
