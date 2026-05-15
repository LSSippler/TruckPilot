//! `truckpilot-vis-uids` — Phase 5.13 cross-sector hypothesis test.
//!
//! Every legacy ETS2 sector ends with the trailing layout
//!
//! ```text
//!     ... | node_count u32 | nodes(56·N) | vis_count u32 | vis_uids(8·M)
//! ```
//!
//! `recover_nodes_from_tail` (sector.rs) already locates this layout via the
//! sweep `count_pos + 4 + N*56 + 4 + M*8 == data.len()`, but only consumes
//! the nodes — the `vis_uids` block is **discarded**.
//!
//! Phase 5.11 showed cross-sector graph edges sit at ~1.4 %, far too low
//! for a connected map. Hypothesis: `vis_uids` are exactly the node UIDs
//! that the sector "sees" from its neighbours — i.e. the cross-sector
//! references. If that holds, generating edges from them in the graph
//! builder fixes the 0/90 routing failure in one shot.
//!
//! This binary tests the hypothesis purely diagnostically:
//!
//! 1. For every `.base` sector in `base_map.scs`:
//!    a. Parse normally (so the local `node_to_sector` map is populated).
//!    b. Re-sweep the same `(M, N)` layout on the raw bytes and extract
//!    the M trailing u64s — these are the `vis_uids`.
//! 2. Classify every vis_uid against `node_to_sector`:
//!    * **same** — uid resolves to a node in the *same* sector
//!    * **cross** — uid resolves to a node in a *different* sector
//!    * **unresolved** — uid not seen anywhere
//! 3. Dump per-sector population stats + a 10-sample inspection list +
//!    an overall verdict line.
//!
//! High **cross** share confirms the hypothesis; high **unresolved** means
//! either (a) the vis_uids are something else entirely, or (b) the cross
//! sectors are the ones that fail to parse so their node UIDs never make
//! it into `node_to_sector`. Either way the numbers tell us where to go.
//!
//! Usage:
//!
//! ```powershell
//! cargo run --release --bin truckpilot-vis-uids -- `
//!   --ets2-dir "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2"
//! ```

use std::collections::HashMap;
use std::fmt::Write as _;
use std::path::PathBuf;

use truckpilot_map_parser::sector::parse_sector;
use truckpilot_map_parser::{Archive, HashFsArchive};

#[derive(Debug)]
struct Args {
    ets2_dir: PathBuf,
    output: PathBuf,
    sample_count: usize,
}

fn parse_args() -> Args {
    let mut ets2_dir: Option<PathBuf> = None;
    let mut output = PathBuf::from("outputs/vis_uids_diagnostic.txt");
    let mut sample_count = 10usize;

    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--ets2-dir" => {
                ets2_dir = Some(PathBuf::from(
                    argv.get(i + 1).expect("--ets2-dir needs value"),
                ));
                i += 2;
            }
            "--output" => {
                output = PathBuf::from(argv.get(i + 1).expect("--output needs value"));
                i += 2;
            }
            "--samples" => {
                sample_count = argv
                    .get(i + 1)
                    .and_then(|s| s.parse().ok())
                    .expect("--samples needs positive integer");
                i += 2;
            }
            "-h" | "--help" => {
                eprintln!(
                    "usage: truckpilot-vis-uids --ets2-dir <PATH> [--output <FILE>] [--samples N]"
                );
                std::process::exit(0);
            }
            other => {
                eprintln!("unknown argument: {other}");
                std::process::exit(2);
            }
        }
    }

    Args {
        ets2_dir: ets2_dir.unwrap_or_else(|| {
            eprintln!("ERROR: --ets2-dir is required");
            std::process::exit(2);
        }),
        output,
        sample_count,
    }
}

/// `SectorId` indexes into the parallel `sector_paths` Vec.
type SectorId = u32;

/// Result of the trailing-layout sweep on a sector's raw bytes.
struct VisLayout {
    /// Number of vis_uids extracted (M).
    vis_uids: Vec<u64>,
    /// Number of nodes the layout claims (N) — used for sanity reporting.
    node_count: u32,
}

/// Re-implement the sweep from `recover_nodes_from_tail` but extract the
/// vis_uids instead of nodes.
fn sweep_vis_layout(data: &[u8]) -> Option<VisLayout> {
    const NODE_BYTES: usize = 56;
    const MAX_N: usize = 4096;
    const MAX_M: usize = 4096;

    let total = data.len();
    for m in 0..=MAX_M {
        let vis_block = 4 + m * 8;
        if vis_block > total {
            break;
        }
        let vis_count_pos = total - vis_block;
        if vis_count_pos < 4 + 16 {
            continue;
        }
        let vis_count = u32::from_le_bytes([
            data[vis_count_pos],
            data[vis_count_pos + 1],
            data[vis_count_pos + 2],
            data[vis_count_pos + 3],
        ]);
        if vis_count as usize != m {
            continue;
        }

        let nodes_end = vis_count_pos;
        for n in 0..=MAX_N {
            let block = 4 + n * NODE_BYTES;
            if block > nodes_end {
                break;
            }
            let count_pos = nodes_end - block;
            if count_pos < 16 {
                break;
            }
            let count_at = u32::from_le_bytes([
                data[count_pos],
                data[count_pos + 1],
                data[count_pos + 2],
                data[count_pos + 3],
            ]);
            if count_at as usize != n {
                continue;
            }

            // Plausibility — same idea as recovery. Spot-check the first
            // node's UID is non-zero when N > 0.
            if n > 0 {
                let first_uid_pos = count_pos + 4;
                let first_uid = u64::from_le_bytes(
                    data[first_uid_pos..first_uid_pos + 8].try_into().unwrap(),
                );
                if first_uid == 0 {
                    continue;
                }
            }

            // Extract the M vis_uids (right after vis_count u32).
            let mut vis_uids = Vec::with_capacity(m);
            for k in 0..m {
                let pos = vis_count_pos + 4 + k * 8;
                let uid = u64::from_le_bytes(data[pos..pos + 8].try_into().unwrap());
                vis_uids.push(uid);
            }

            return Some(VisLayout {
                vis_uids,
                node_count: n as u32,
            });
        }
    }

    None
}

fn pct(num: u64, denom: u64) -> f64 {
    if denom == 0 {
        0.0
    } else {
        100.0 * num as f64 / denom as f64
    }
}

fn main() {
    let args = parse_args();
    let base_map = args.ets2_dir.join("base_map.scs");
    if !base_map.exists() {
        eprintln!(
            "ERROR: {} not found — wrong --ets2-dir?",
            base_map.display()
        );
        std::process::exit(1);
    }

    eprintln!("opening {} …", base_map.display());
    let mut archive = HashFsArchive::open(&base_map).expect("open base_map.scs");
    let mut sector_paths: Vec<String> = archive
        .probe_sector_paths()
        .into_iter()
        .filter(|p| p.ends_with(".base"))
        .collect();
    sector_paths.sort();
    eprintln!("probed {} `.base` sector paths", sector_paths.len());

    // ----- Pass 1: parse every sector + extract vis_uids ----------------

    let mut node_to_sector: HashMap<u64, SectorId> =
        HashMap::with_capacity(1_000_000);
    // Per-sector: list of vis_uids (parallel to sector_paths)
    let mut sector_vis: Vec<Vec<u64>> = Vec::with_capacity(sector_paths.len());
    let mut sector_node_counts: Vec<u32> = Vec::with_capacity(sector_paths.len());
    let mut sectors_parsed = 0usize;
    let mut sweep_failed = 0usize;
    let mut parse_failed = 0usize;

    for (sid, path) in sector_paths.iter().enumerate() {
        if sid > 0 && sid % 500 == 0 {
            eprintln!("  …{}/{} sectors", sid, sector_paths.len());
        }
        let Ok(data) = archive.read_path(path) else {
            sector_vis.push(Vec::new());
            sector_node_counts.push(0);
            continue;
        };

        // Extract vis_uids (independent of parser success).
        match sweep_vis_layout(&data) {
            Some(l) => {
                sector_vis.push(l.vis_uids);
                sector_node_counts.push(l.node_count);
            }
            None => {
                sector_vis.push(Vec::new());
                sector_node_counts.push(0);
                sweep_failed += 1;
            }
        }

        // Parse normally to populate node_to_sector.
        let sector_id = sid as SectorId;
        match parse_sector(&data) {
            Ok(parsed) => {
                sectors_parsed += 1;
                for n in parsed.nodes {
                    node_to_sector.insert(n.uid, sector_id);
                }
            }
            Err(_) => {
                parse_failed += 1;
            }
        }
    }

    eprintln!(
        "parsed {} sectors ({} parse-failed, {} sweep-failed), {} unique nodes",
        sectors_parsed,
        parse_failed,
        sweep_failed,
        node_to_sector.len()
    );

    // ----- Pass 2: classify ---------------------------------------------

    let mut total_vis: u64 = 0;
    let mut zero_uids: u64 = 0;
    let mut same: u64 = 0;
    let mut cross: u64 = 0;
    let mut unresolved: u64 = 0;
    let mut max_m = 0usize;
    let mut sectors_with_zero_m = 0usize;

    for (sid, uids) in sector_vis.iter().enumerate() {
        if uids.is_empty() {
            sectors_with_zero_m += 1;
            continue;
        }
        if uids.len() > max_m {
            max_m = uids.len();
        }
        for &uid in uids {
            total_vis += 1;
            if uid == 0 {
                zero_uids += 1;
                unresolved += 1;
                continue;
            }
            match node_to_sector.get(&uid) {
                Some(&owner) if owner == sid as SectorId => same += 1,
                Some(_) => cross += 1,
                None => unresolved += 1,
            }
        }
    }

    let nonzero_total = total_vis - zero_uids;
    let cross_share_nonzero = pct(cross, nonzero_total);

    // ----- Render -------------------------------------------------------

    let mut out = String::with_capacity(16 * 1024);
    let _ = writeln!(out, "=== VIS_UIDS DIAGNOSE — Phase 5.13 ===");
    let _ = writeln!(out, "archive            : {}", base_map.display());
    let _ = writeln!(
        out,
        "sectors            : {} probed, {} parse-failed, {} sweep-failed",
        sector_paths.len(),
        parse_failed,
        sweep_failed
    );
    let _ = writeln!(out);

    // Population
    let _ = writeln!(out, "── VIS_UIDS POPULATION ──");
    let _ = writeln!(out, "total vis_uids        : {:>10}", total_vis);
    let _ = writeln!(
        out,
        "  of which zero (=0)  : {:>10}  ({:>5.1} %)",
        zero_uids,
        pct(zero_uids, total_vis)
    );
    let _ = writeln!(
        out,
        "sectors with M = 0    : {:>10}  ({:>5.1} %)",
        sectors_with_zero_m,
        pct(sectors_with_zero_m as u64, sector_paths.len() as u64)
    );
    let nonzero_sectors = sector_paths.len() - sectors_with_zero_m;
    let avg_per_sector = if nonzero_sectors > 0 {
        total_vis as f64 / nonzero_sectors as f64
    } else {
        0.0
    };
    let _ = writeln!(out, "avg M per nonzero sec : {:>10.1}", avg_per_sector);
    let _ = writeln!(out, "max M (largest sector): {:>10}", max_m);
    let _ = writeln!(out);

    // Classification
    let _ = writeln!(out, "── CLASSIFICATION (against node→sector lookup) ──");
    let _ = writeln!(out, "total vis_uid refs    : {:>10}", total_vis);
    let _ = writeln!(
        out,
        "  same sector         : {:>10}  ({:>5.1} %)",
        same,
        pct(same, total_vis)
    );
    let _ = writeln!(
        out,
        "  CROSS sector        : {:>10}  ({:>5.1} %)",
        cross,
        pct(cross, total_vis)
    );
    let _ = writeln!(
        out,
        "  unresolved          : {:>10}  ({:>5.1} %)",
        unresolved,
        pct(unresolved, total_vis)
    );
    let _ = writeln!(out);
    let _ = writeln!(out, "non-zero only:");
    let _ = writeln!(out, "  total non-zero      : {:>10}", nonzero_total);
    let _ = writeln!(
        out,
        "  CROSS share         : {:>10}  ({:>5.1} %)",
        cross,
        cross_share_nonzero
    );
    let _ = writeln!(out);

    // Sample inspection
    let _ = writeln!(
        out,
        "── SAMPLE: first {} sectors with M > 0 (visual inspection) ──",
        args.sample_count
    );
    let mut shown = 0usize;
    for (sid, uids) in sector_vis.iter().enumerate() {
        if uids.is_empty() {
            continue;
        }
        if shown >= args.sample_count {
            break;
        }
        shown += 1;
        let _ = writeln!(
            out,
            "{} (M={}, N={})",
            sector_paths[sid],
            uids.len(),
            sector_node_counts[sid]
        );
        for (shown_uids, &uid) in uids.iter().enumerate() {
            if shown_uids >= 12 {
                let _ = writeln!(out, "  ... ({} more)", uids.len() - shown_uids);
                break;
            }
            if uid == 0 {
                let _ = writeln!(out, "  0x0000000000000000   ZERO");
                continue;
            }
            match node_to_sector.get(&uid) {
                Some(&owner) if owner == sid as SectorId => {
                    let _ = writeln!(
                        out,
                        "  0x{:016X}   SAME    sector {}",
                        uid, sid
                    );
                }
                Some(&owner) => {
                    let _ = writeln!(
                        out,
                        "  0x{:016X}   CROSS   {} → {}",
                        uid, sid, sector_paths[owner as usize]
                    );
                }
                None => {
                    let _ = writeln!(out, "  0x{:016X}   UNRES", uid);
                }
            }
        }
        let _ = writeln!(out);
    }

    // Verdict
    let _ = writeln!(out, "── VERDICT ──");
    if cross_share_nonzero >= 50.0 {
        let _ = writeln!(
            out,
            "B (vis_uids = cross-sector refs) STRONGLY SUPPORTED ({:.1} % of non-zero refs cross sectors).",
            cross_share_nonzero
        );
        let _ = writeln!(
            out,
            "→ Generate cross-sector edges from vis_uids in graph.rs."
        );
    } else if cross_share_nonzero >= 20.0 {
        let _ = writeln!(
            out,
            "B partially supported ({:.1} % cross). Mixed signal — vis_uids contain *some* cross-sector links but also other content.",
            cross_share_nonzero
        );
        let _ = writeln!(
            out,
            "→ Inspect samples to see what the non-cross UIDs are; consider filtering."
        );
    } else {
        let _ = writeln!(
            out,
            "B REJECTED ({:.1} % cross). vis_uids are not the cross-sector connectivity source.",
            cross_share_nonzero
        );
        let _ = writeln!(
            out,
            "→ Look elsewhere — likely .aux companion files or item-stream items we still drop."
        );
    }

    if let Some(parent) = args.output.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    std::fs::write(&args.output, &out)
        .unwrap_or_else(|e| panic!("write {}: {e}", args.output.display()));
    eprintln!("wrote {}", args.output.display());
    print!("{}", out);
}
