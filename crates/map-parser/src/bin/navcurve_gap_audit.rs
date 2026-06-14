//! `truckpilot-navcurve-gap-audit` — DS14 Plan-A Audit-Gate (read-only).
//!
//! Verifies the core assumption of DS14 Plan A at a known NavCurve-gap junction:
//! that the Road segments through the gap already live in the daemon's SplineIndex
//! and are reachable by the lane-keeper's route-aware nearest query.
//!
//! It mirrors the daemon's index assembly EXACTLY:
//!   1. `build_splines_ex(graph)`                          → road segments `[0 .. road_n)`
//!   2. `graph.prefab_hermite_segments_with_metadata()`    → NavCurve segments `[road_n ..)`
//!   3. `build_index_with_metadata(...)`                   → one combined `SplineIndex`
//!   4. `seg_by_from_to` built from `[0 .. road_n)` only   (lane-keeper `on_load`)
//!   5. RouterGraph edges = ALL graph edges                (daemon `build_router_graph`)
//!
//! Read-only. Touches no production code. Output goes to stdout.
//!
//! Usage:
//!   truckpilot-navcurve-gap-audit --graph graph.json --jx 9106 --jz -10001 \
//!       [--radius 60] [--west X,Z] [--east X,Z] [--auto-dist 300]

use std::collections::{BinaryHeap, HashMap};
use std::path::PathBuf;

use truckpilot_map_parser::spline::{evaluate, evaluate_tangent, HermiteSegment, Vec3};
use truckpilot_map_parser::{build_index_with_metadata, build_splines_ex, MapGraph};

// ── Lane-keeper constants (copied verbatim for faithful dispatch simulation) ──
const MAX_HOP_PROJECTION_DIST_M: f32 = 50.0;
const ROUTE_NEAREST_CANDIDATES: usize = 24;
const PREFAB_CURVE_FALLBACK_DEG: f32 = 40.0;
const INTK_PLAUSIBLE_MAX_DEG: f32 = 100.0;

// ---------------------------------------------------------------------------
// Args
// ---------------------------------------------------------------------------

struct Args {
    graph: PathBuf,
    jx: f64,
    jz: f64,
    radius: f64,
    west: Option<(f64, f64)>,
    east: Option<(f64, f64)>,
    auto_dist: f64,
    turn_edge: Option<(u64, u64)>,
    ring_min: f64,
    ring_max: f64,
    cruise: f32,
    scan: bool,
}

fn parse_uid_pair(s: &str) -> Option<(u64, u64)> {
    let (a, b) = s.split_once(',')?;
    Some((a.trim().parse().ok()?, b.trim().parse().ok()?))
}

fn parse_xz(s: &str) -> Option<(f64, f64)> {
    let (a, b) = s.split_once(',')?;
    Some((a.trim().parse().ok()?, b.trim().parse().ok()?))
}

fn parse_args() -> Args {
    let mut graph = PathBuf::from("graph.json");
    let mut jx = 9106.0;
    let mut jz = -10001.0;
    let mut radius = 60.0;
    let mut west = None;
    let mut east = None;
    let mut auto_dist = 300.0;
    let mut turn_edge = None;
    let mut ring_min = 120.0;
    let mut ring_max = 500.0;
    let mut cruise = 30.0;
    let mut scan = false;
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0;
    while i < argv.len() {
        match argv[i].as_str() {
            "--scan" => {
                scan = true;
                i += 1;
            }
            "--turn-edge" => {
                turn_edge = parse_uid_pair(&argv[i + 1]);
                i += 2;
            }
            "--ring-min" => {
                ring_min = argv[i + 1].parse().unwrap_or(ring_min);
                i += 2;
            }
            "--ring-max" => {
                ring_max = argv[i + 1].parse().unwrap_or(ring_max);
                i += 2;
            }
            "--cruise" => {
                cruise = argv[i + 1].parse().unwrap_or(cruise);
                i += 2;
            }
            "--graph" => {
                graph = PathBuf::from(&argv[i + 1]);
                i += 2;
            }
            "--jx" => {
                jx = argv[i + 1].parse().unwrap_or(jx);
                i += 2;
            }
            "--jz" => {
                jz = argv[i + 1].parse().unwrap_or(jz);
                i += 2;
            }
            "--radius" => {
                radius = argv[i + 1].parse().unwrap_or(radius);
                i += 2;
            }
            "--west" => {
                west = parse_xz(&argv[i + 1]);
                i += 2;
            }
            "--east" => {
                east = parse_xz(&argv[i + 1]);
                i += 2;
            }
            "--auto-dist" => {
                auto_dist = argv[i + 1].parse().unwrap_or(auto_dist);
                i += 2;
            }
            other => {
                eprintln!("unknown arg: {other}");
                std::process::exit(2);
            }
        }
    }
    Args {
        graph,
        jx,
        jz,
        radius,
        west,
        east,
        auto_dist,
        turn_edge,
        ring_min,
        ring_max,
        cruise,
        scan,
    }
}

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

fn dist_xz(ax: f64, az: f64, bx: f64, bz: f64) -> f64 {
    let dx = ax - bx;
    let dz = az - bz;
    (dx * dx + dz * dz).sqrt()
}

/// Minimum XZ distance from `(qx,qz)` to a sampled Hermite segment, plus the t of the min.
fn min_dist_to_seg(seg: &HermiteSegment, qx: f64, qz: f64, n: usize) -> (f64, f64) {
    let mut best = f64::MAX;
    let mut best_t = 0.0;
    for i in 0..=n {
        let t = i as f32 / n as f32;
        let p = evaluate(seg, t);
        let d = dist_xz(qx, qz, p.x as f64, p.z as f64);
        if d < best {
            best = d;
            best_t = t as f64;
        }
    }
    (best, best_t)
}

/// Full internal kink of a segment: angle (deg) between tangent at t=0 and t=1.
/// This is the m0-anomaly headline; the lane-keeper latches `prefab_curve` when the
/// kink the truck actually drives exceeds `PREFAB_CURVE_FALLBACK_DEG` (40°).
fn internal_kink_deg(seg: &HermiteSegment) -> f64 {
    kink_between(seg, 0.0, 1.0)
}

/// Point-geometry internal kink (Ebene-2 metric): sample `n` Hermite points and measure
/// the max single-step heading change and the total turning along the sampled polyline.
/// A geometrically straight chord gives ~0 for both, regardless of its endpoint tangents.
fn point_geom_intk(seg: &HermiteSegment, n: usize) -> (f64, f64) {
    let n = n.max(3);
    let mut dirs: Vec<f64> = Vec::with_capacity(n - 1);
    let mut prev = evaluate(seg, 0.0);
    for i in 1..n {
        let p = evaluate(seg, i as f32 / (n - 1) as f32);
        let dx = (p.x - prev.x) as f64;
        let dz = (p.z - prev.z) as f64;
        if dx * dx + dz * dz > 1e-12 {
            dirs.push(dx.atan2(-dz));
        }
        prev = p;
    }
    let mut max_step = 0.0_f64;
    let mut total = 0.0_f64;
    for w in dirs.windows(2) {
        let mut d = w[1] - w[0];
        while d > std::f64::consts::PI {
            d -= std::f64::consts::TAU;
        }
        while d < -std::f64::consts::PI {
            d += std::f64::consts::TAU;
        }
        let a = d.abs();
        if a > max_step {
            max_step = a;
        }
        total += a;
    }
    (max_step.to_degrees(), total.to_degrees())
}

/// Percentile from a pre-sorted slice (p in [0,1]).
fn percentile(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return f64::NAN;
    }
    let idx = (((sorted.len() - 1) as f64) * p).round() as usize;
    sorted[idx.min(sorted.len() - 1)]
}

/// Kink (deg) between tangents at parameters `ta` and `tb`.
fn kink_between(seg: &HermiteSegment, ta: f32, tb: f32) -> f64 {
    let a = evaluate_tangent(seg, ta);
    let b = evaluate_tangent(seg, tb);
    let la = (a.x * a.x + a.z * a.z).sqrt();
    let lb = (b.x * b.x + b.z * b.z).sqrt();
    if la < 1e-6 || lb < 1e-6 {
        return -1.0;
    }
    let ha = a.x.atan2(-a.z);
    let hb = b.x.atan2(-b.z);
    let mut d = hb - ha;
    while d > std::f32::consts::PI {
        d -= std::f32::consts::TAU;
    }
    while d < -std::f32::consts::PI {
        d += std::f32::consts::TAU;
    }
    (d.abs() as f64).to_degrees()
}

// ---------------------------------------------------------------------------
// Minimal A* (mirrors RouterGraph::plan)
// ---------------------------------------------------------------------------

#[derive(Clone)]
struct HeapEntry {
    uid: u64,
    f: f64,
}
impl PartialEq for HeapEntry {
    fn eq(&self, o: &Self) -> bool {
        self.f.total_cmp(&o.f).is_eq() && self.uid == o.uid
    }
}
impl Eq for HeapEntry {}
impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for HeapEntry {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        o.f.total_cmp(&self.f).then_with(|| self.uid.cmp(&o.uid)) // min-heap via reverse
    }
}

fn astar(
    adj: &HashMap<u64, Vec<(u64, f64)>>,
    pos: &HashMap<u64, (f64, f64)>,
    start: u64,
    goal: u64,
) -> Option<(Vec<u64>, f64)> {
    let goal_pos = *pos.get(&goal)?;
    let h = |u: u64| -> f64 {
        let p = pos.get(&u).copied().unwrap_or(goal_pos);
        dist_xz(p.0, p.1, goal_pos.0, goal_pos.1)
    };
    let mut open: BinaryHeap<HeapEntry> = BinaryHeap::new();
    let mut g: HashMap<u64, f64> = HashMap::new();
    let mut came: HashMap<u64, u64> = HashMap::new();
    g.insert(start, 0.0);
    open.push(HeapEntry {
        uid: start,
        f: h(start),
    });
    let mut closed: std::collections::HashSet<u64> = std::collections::HashSet::new();
    while let Some(e) = open.pop() {
        if e.uid == goal {
            let mut path = vec![goal];
            let mut cur = goal;
            while cur != start {
                let p = *came.get(&cur)?;
                path.push(p);
                cur = p;
            }
            path.reverse();
            return Some((path, g[&goal]));
        }
        if !closed.insert(e.uid) {
            continue;
        }
        for &(nb, cost) in adj.get(&e.uid).into_iter().flatten() {
            if closed.contains(&nb) {
                continue;
            }
            let tg = g[&e.uid] + cost;
            if tg < *g.get(&nb).unwrap_or(&f64::MAX) {
                came.insert(nb, e.uid);
                g.insert(nb, tg);
                open.push(HeapEntry {
                    uid: nb,
                    f: tg + h(nb),
                });
            }
        }
    }
    None
}

/// Bounded single-source Dijkstra over `adj`; returns shortest-path cost to every node
/// reachable within `max_dist`. Used to find start/goal candidates around a turn edge.
fn dijkstra_dist(
    adj: &HashMap<u64, Vec<(u64, f64)>>,
    src: u64,
    max_dist: f64,
) -> HashMap<u64, f64> {
    let mut dist: HashMap<u64, f64> = HashMap::new();
    let mut open: BinaryHeap<HeapEntry> = BinaryHeap::new();
    dist.insert(src, 0.0);
    open.push(HeapEntry { uid: src, f: 0.0 });
    while let Some(e) = open.pop() {
        if e.f > max_dist {
            break; // Dijkstra pops in increasing order → all remaining are farther
        }
        let d = *dist.get(&e.uid).unwrap_or(&f64::MAX);
        if e.f > d {
            continue;
        }
        for &(nb, cost) in adj.get(&e.uid).into_iter().flatten() {
            let nd = d + cost;
            if nd <= max_dist && nd < *dist.get(&nb).unwrap_or(&f64::MAX) {
                dist.insert(nb, nd);
                open.push(HeapEntry { uid: nb, f: nd });
            }
        }
    }
    dist
}

/// Nearest node (that has at least one incident edge) to a point.
fn snap_node(
    pos: &HashMap<u64, (f64, f64)>,
    has_edge: &std::collections::HashSet<u64>,
    x: f64,
    z: f64,
) -> Option<(u64, f64)> {
    pos.iter()
        .filter(|(uid, _)| has_edge.contains(*uid))
        .map(|(&uid, &(nx, nz))| (uid, dist_xz(x, z, nx, nz)))
        .min_by(|a, b| a.1.total_cmp(&b.1))
}

// ---------------------------------------------------------------------------
// Classification
// ---------------------------------------------------------------------------

fn kind_of(idx: usize, road_n: usize) -> &'static str {
    if idx < road_n {
        "ROAD"
    } else {
        "NAVCURVE"
    }
}

/// 8-point compass label for a heading in degrees (CW from North).
fn bearing_cardinal(deg: f64) -> &'static str {
    const DIRS: [&str; 8] = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];
    DIRS[(((deg.rem_euclid(360.0) + 22.5) / 45.0).floor() as usize) % 8]
}

/// Task 1 of the m0-intK-Fix: global audit of the `metadata=None` separator over the
/// whole road block. Verifies prefab chords are short artifacts (tangent-intK clusters
/// at 0/90/180) while real curves carry `metadata=Some`.
fn run_global_scan(
    segments: &[HermiteSegment],
    metadata: &[Option<truckpilot_map_parser::SegmentMetadata>],
    road_n: usize,
    dir_by_edge_uid: &HashMap<u64, String>,
) {
    let mut dir_counts: HashMap<String, usize> = HashMap::new();
    let mut some_count = 0usize;
    let mut none_count = 0usize;
    let mut some_sharp = 0usize; // metadata=Some, tangent-intK > 40 (real curve latch catches)
    let mut some_cap = 0usize; // metadata=Some, tangent-intK > 100
    let mut some_ptot_sharp = 0usize; // metadata=Some, point-geom total > 40 (real geometric curve)

    let mut prefab_tan: Vec<f64> = Vec::new();
    let mut prefab_ptot: Vec<f64> = Vec::new();
    let mut prefab_pmax: Vec<f64> = Vec::new();
    let mut prefab_len: Vec<f64> = Vec::new();
    // metadata=None but direction != "prefab": dir -> (count, lens, ptots)
    let mut none_other: HashMap<String, (usize, Vec<f64>, Vec<f64>)> = HashMap::new();
    // gate evidence samples
    let mut crooked_prefab_ex: Vec<(usize, u64, u64, f64, f64, f64)> = Vec::new(); // idx,from,to,len,tan,pmax
    let mut long_none_ex: Vec<(usize, String, u64, u64, f64, f64, f64)> = Vec::new(); // idx,dir,from,to,len,tan,ptot

    for (i, seg) in segments.iter().enumerate().take(road_n) {
        let dir = dir_by_edge_uid
            .get(&seg.edge_uid)
            .cloned()
            .unwrap_or_else(|| "?".into());
        *dir_counts.entry(dir.clone()).or_default() += 1;
        let tan = internal_kink_deg(seg);
        let len = seg.length_m as f64;

        if metadata[i].is_some() {
            some_count += 1;
            if tan > 40.0 {
                some_sharp += 1;
            }
            if tan > 100.0 {
                some_cap += 1;
            }
            if point_geom_intk(seg, 7).1 > 40.0 {
                some_ptot_sharp += 1;
            }
            continue;
        }

        none_count += 1;
        let (pmax, ptot) = point_geom_intk(seg, 7);
        if dir == "prefab" {
            prefab_tan.push(tan);
            prefab_ptot.push(ptot);
            prefab_pmax.push(pmax);
            prefab_len.push(len);
        } else {
            let e = none_other
                .entry(dir.clone())
                .or_insert((0, Vec::new(), Vec::new()));
            e.0 += 1;
            e.1.push(len);
            e.2.push(ptot);
        }
        let crooked = (tan > 12.0 && tan < 78.0) || (tan > 102.0 && tan < 168.0);
        let long = len > 55.0;
        if dir == "prefab" && crooked && len < 45.0 && crooked_prefab_ex.len() < 18 {
            crooked_prefab_ex.push((i, seg.from_uid, seg.to_uid, len, tan, pmax));
        }
        if long && long_none_ex.len() < 18 {
            long_none_ex.push((i, dir, seg.from_uid, seg.to_uid, len, tan, ptot));
        }
    }

    // histogram helper for a [0,180]-degree value set
    let hist5 = |v: &[f64]| -> [usize; 5] {
        let mut h = [0usize; 5];
        for &x in v {
            let b = if x < 10.0 {
                0
            } else if x < 80.0 {
                1
            } else if x <= 100.0 {
                2
            } else if x <= 170.0 {
                3
            } else {
                4
            };
            h[b] += 1;
        }
        h
    };
    let hist_ptot = |v: &[f64]| -> [usize; 4] {
        let mut h = [0usize; 4];
        for &x in v {
            let b = if x < 5.0 {
                0
            } else if x < 20.0 {
                1
            } else if x < 60.0 {
                2
            } else {
                3
            };
            h[b] += 1;
        }
        h
    };

    println!("================================================================");
    println!("m0-intK-FIX  AUDIT-GATE  —  GLOBAL metadata=None SEPARATOR SCAN");
    println!("================================================================");
    println!();
    println!("Road block scanned: {} segments", road_n);
    println!("  metadata=Some (real road edges) : {}", some_count);
    println!("  metadata=None (artificial/other): {}", none_count);
    println!();
    println!("Direction histogram (road block):");
    let mut dirs: Vec<(&String, &usize)> = dir_counts.iter().collect();
    dirs.sort_by(|a, b| b.1.cmp(a.1));
    for (d, c) in dirs {
        println!("  {:<24} {}", d, c);
    }
    println!();

    // 1a — prefab chords
    let th = hist5(&prefab_tan);
    let pe = hist_ptot(&prefab_ptot);
    let crooked_prefab = th[1] + th[3];
    println!(
        "1a — direction==\"prefab\" (metadata=None): {} segments",
        prefab_tan.len()
    );
    println!(
        "  tangent-intK:  <10°:{}  10–80°:{}  ~90°(80–100):{}  100–170°:{}  >170°:{}",
        th[0], th[1], th[2], th[3], th[4]
    );
    println!(
        "     => CROOKED (10–80 ∪ 100–170, possible real curve): {}",
        crooked_prefab
    );
    // Ebene-2 metric is MAX per-step heading change (point_geom_intk.0):
    let pmax_b = {
        let mut h = [0usize; 4];
        for &x in &prefab_pmax {
            let b = if x < 6.0 {
                0
            } else if x < 40.0 {
                1
            } else if x <= 95.0 {
                2
            } else {
                3
            };
            h[b] += 1;
        }
        h
    };
    println!(
        "  point-geom MAX-step (Ebene-2 metric):  <6°:{}  6–40°:{}  40–95°:{}  >95°:{}",
        pmax_b[0], pmax_b[1], pmax_b[2], pmax_b[3]
    );
    println!(
        "     => Ebene-2 would NOT latch (max-step<40°): {} / {}",
        pmax_b[0] + pmax_b[1],
        prefab_pmax.len()
    );
    println!(
        "     => Ebene-2 would STILL latch (max-step>40°): {}",
        pmax_b[2] + pmax_b[3]
    );
    println!(
        "  point-geom total turning:  <5°:{}  5–20°:{}  20–60°:{}  >60°:{}",
        pe[0], pe[1], pe[2], pe[3]
    );
    println!(
        "     => geometrically straight (total<5°): {} / {}",
        pe[0],
        prefab_ptot.len()
    );
    // Short-chord subset (<45m = junction-internal): is the TARGET population clean?
    let mut short_tan: Vec<f64> = Vec::new();
    let mut short_pmax: Vec<f64> = Vec::new();
    for ((&t, &pm), &l) in prefab_tan
        .iter()
        .zip(prefab_pmax.iter())
        .zip(prefab_len.iter())
    {
        if l < 45.0 {
            short_tan.push(t);
            short_pmax.push(pm);
        }
    }
    let sth = hist5(&short_tan);
    let mut spm = [0usize; 4];
    for &x in &short_pmax {
        let b = if x < 6.0 {
            0
        } else if x < 40.0 {
            1
        } else if x <= 95.0 {
            2
        } else {
            3
        };
        spm[b] += 1;
    }
    println!(
        "  SHORT chords (<45m, junction-internal target): {} of {}",
        short_tan.len(),
        prefab_tan.len()
    );
    println!(
        "    tangent-intK  <10°:{}  10–80°:{}  ~90°:{}  100–170°:{}  >170°:{}  (crooked {})",
        sth[0],
        sth[1],
        sth[2],
        sth[3],
        sth[4],
        sth[1] + sth[3]
    );
    println!("    point-geom max-step  <6°:{}  6–40°:{}  40–95°:{}  >95°:{}  => Ebene-2 still-latch(>40°): {}", spm[0], spm[1], spm[2], spm[3], spm[2] + spm[3]);
    println!();

    // 1c — prefab lengths
    let mut plen = prefab_len.clone();
    plen.sort_by(f64::total_cmp);
    let long_prefab = plen.iter().filter(|&&x| x > 55.0).count();
    println!("1c — direction==\"prefab\" length distribution:");
    if !plen.is_empty() {
        println!(
            "  min {:.1}m  p50 {:.1}m  p95 {:.1}m  max {:.1}m   count>55m: {}",
            plen[0],
            percentile(&plen, 0.5),
            percentile(&plen, 0.95),
            plen[plen.len() - 1],
            long_prefab
        );
    }
    println!();

    // metadata=None & direction != prefab (broader Ebene-2 set)
    println!("metadata=None & direction!=\"prefab\" (broader Ebene-2 set):");
    if none_other.is_empty() {
        println!("  (none — every metadata=None segment is direction==prefab)");
    } else {
        let mut keys: Vec<_> = none_other.iter().collect();
        keys.sort_by_key(|(_, v)| std::cmp::Reverse(v.0));
        for (d, (cnt, lens, ptots)) in keys {
            let mut l = lens.clone();
            l.sort_by(f64::total_cmp);
            let long = l.iter().filter(|&&x| x > 55.0).count();
            let curved = ptots.iter().filter(|&&x| x > 20.0).count();
            println!(
                "  {:<22} count {}  len[p50 {:.0}m p95 {:.0}m max {:.0}m >55m:{}]  ptot>20°:{}",
                d,
                cnt,
                percentile(&l, 0.5),
                percentile(&l, 0.95),
                l[l.len() - 1],
                long,
                curved
            );
        }
    }
    println!();

    // 1b — real curves carry metadata=Some
    println!("1b — metadata=Some with REAL sharp kink (latch rightly catches):");
    println!(
        "  tangent-intK>40°: {}   >100°(over cap): {}   point-geom-total>40°: {}",
        some_sharp, some_cap, some_ptot_sharp
    );
    println!();

    println!("GATE-EVIDENCE A — SHORT prefab chords with CROOKED tangent-intK (NOT the 0/90/180 artifact signature):");
    println!(
        "  {:>9} {:>22} {:>22} {:>7} {:>7} {:>7}",
        "idx", "from", "to", "len_m", "tanK°", "maxStep°"
    );
    for (idx, from, to, len, tan, pmax) in &crooked_prefab_ex {
        println!(
            "  {:>9} {:>22} {:>22} {:>7.1} {:>7.1} {:>7.1}",
            idx, from, to, len, tan, pmax
        );
    }
    println!();
    println!("GATE-EVIDENCE C — LONG metadata=None segments (straightening would distort a real stretch):");
    println!(
        "  {:>9} {:>10} {:>22} {:>22} {:>8} {:>7} {:>7}",
        "idx", "dir", "from", "to", "len_m", "tanK°", "ptot°"
    );
    for (idx, dir, from, to, len, tan, ptot) in &long_none_ex {
        println!(
            "  {:>9} {:>10} {:>22} {:>22} {:>8.1} {:>7.1} {:>7.1}",
            idx, dir, from, to, len, tan, ptot
        );
    }
    println!();
    println!("================================================================");
}

fn main() {
    let args = parse_args();
    let t0 = std::time::Instant::now();

    eprintln!("[load] reading {} ...", args.graph.display());
    let bytes = std::fs::read(&args.graph).expect("read graph.json");
    eprintln!(
        "[load] {} bytes read in {:.1}s, parsing JSON ...",
        bytes.len(),
        t0.elapsed().as_secs_f64()
    );
    let graph: MapGraph = serde_json::from_slice(&bytes).expect("parse graph.json");
    drop(bytes);
    eprintln!(
        "[load] parsed: {} nodes, {} edges, {} prefab_ai_paths in {:.1}s",
        graph.nodes.len(),
        graph.edges.len(),
        graph.prefab_ai_paths.len(),
        t0.elapsed().as_secs_f64()
    );

    // ── Mirror daemon index assembly (main.rs::build_spline_index_for_hud) ──
    let (mut segments, mut metadata, _stats) = build_splines_ex(&graph);
    let road_n = segments.len();
    let (nav_segs, nav_meta) = graph.prefab_hermite_segments_with_metadata();
    let nav_n = nav_segs.len();
    segments.extend(nav_segs);
    metadata.extend(nav_meta);
    let total_n = segments.len();

    // seg_by_from_to mirrors lane-keeper on_load: ROAD block [0..road_n) only.
    let mut seg_by_from_to: HashMap<(u64, u64), usize> = HashMap::with_capacity(road_n);
    for (i, s) in segments.iter().enumerate().take(road_n) {
        seg_by_from_to.insert((s.from_uid, s.to_uid), i);
    }

    // direction lookup by edge_uid (road segs carry the graph edge uid; nav segs carry 0)
    let dir_by_edge_uid: HashMap<u64, String> = graph
        .edges
        .iter()
        .map(|e| (e.uid, e.direction.clone()))
        .collect();

    // ── GLOBAL SCAN MODE (Task 1 of m0-intK-Fix): metadata=None separator audit ──
    if args.scan {
        run_global_scan(&segments, &metadata, road_n, &dir_by_edge_uid);
        eprintln!("[scan] done in {:.1}s", t0.elapsed().as_secs_f64());
        return;
    }

    // Router graph data (ALL graph edges — daemon build_router_graph)
    let node_pos: HashMap<u64, (f64, f64)> =
        graph.nodes.iter().map(|n| (n.uid, (n.x, n.z))).collect();
    let mut adj: HashMap<u64, Vec<(u64, f64)>> = HashMap::new();
    let mut has_edge: std::collections::HashSet<u64> = std::collections::HashSet::new();
    for e in &graph.edges {
        adj.entry(e.from).or_default().push((e.to, e.distance_m));
        has_edge.insert(e.from);
        has_edge.insert(e.to);
    }

    let index = build_index_with_metadata(segments.clone(), metadata.clone());

    println!("================================================================");
    println!(
        "DS14 PLAN-A AUDIT-GATE  —  junction ({:.0}, {:.0})",
        args.jx, args.jz
    );
    println!("================================================================");
    println!();
    println!("SECTION 0 — INDEX ASSEMBLY (mirrors daemon)");
    println!(
        "  graph nodes / edges / prefab_ai_paths : {} / {} / {}",
        graph.nodes.len(),
        graph.edges.len(),
        graph.prefab_ai_paths.len()
    );
    println!(
        "  ROAD segments      [0 .. {})          : {}",
        road_n, road_n
    );
    println!(
        "  NAVCURVE segments  [{} .. {})         : {}",
        road_n, total_n, nav_n
    );
    println!(
        "  seg_by_from_to entries (ROAD only)    : {}",
        seg_by_from_to.len()
    );
    println!(
        "  search radius                         : {:.0} m",
        args.radius
    );
    println!(
        "  gate consts: dist<= {:.0}m, candidates {}, prefab_curve> {:.0}deg, intk_cap {:.0}deg",
        MAX_HOP_PROJECTION_DIST_M,
        ROUTE_NEAREST_CANDIDATES,
        PREFAB_CURVE_FALLBACK_DEG,
        INTK_PLAUSIBLE_MAX_DEG
    );
    println!();

    // ── Local node topology at the junction ──
    let mut near_nodes: Vec<(f64, u64, f64, f64)> = graph
        .nodes
        .iter()
        .map(|n| (dist_xz(args.jx, args.jz, n.x, n.z), n.uid, n.x, n.z))
        .filter(|t| t.0 <= args.radius)
        .collect();
    near_nodes.sort_by(|a, b| a.0.total_cmp(&b.0));
    println!(
        "SECTION T — GRAPH NODES within {:.0}m (topology)",
        args.radius
    );
    println!(
        "  {:>8}  {:>22}  {:>9}  {:>9}  out in",
        "dist", "uid", "x", "z"
    );
    for &(d, uid, x, z) in near_nodes.iter().take(20) {
        let out = adj.get(&uid).map_or(0, |v| v.len());
        let inc = graph.edges.iter().filter(|e| e.to == uid).count();
        println!("  {d:8.2}  {uid:>22}  {x:9.1}  {z:9.1}  {out:>3} {inc:>2}");
    }
    if near_nodes.len() > 20 {
        println!("  ... {} more nodes", near_nodes.len() - 20);
    }
    println!();

    // ── SECTION 1a + 1c: all segments within radius, classified ──
    let within = index.within_radius_with_idx(
        Vec3::new(args.jx as f32, 0.0, args.jz as f32),
        (args.radius + 30.0) as f32,
    );
    struct Row {
        idx: usize,
        kind: &'static str,
        from: u64,
        to: u64,
        dist: f64,
        t_at_min: f64,
        len: f32,
        intk: f64,
        lane_off: f32,
        on_seg_map: bool,
        dir: String,
        mid_x: f64,
    }
    let mut rows: Vec<Row> = Vec::new();
    for &(idx, seg, meta) in &within {
        let (dist, t_at) = min_dist_to_seg(seg, args.jx, args.jz, 24);
        if dist > args.radius {
            continue;
        }
        let mid = evaluate(seg, 0.5);
        rows.push(Row {
            idx,
            kind: kind_of(idx, road_n),
            from: seg.from_uid,
            to: seg.to_uid,
            dist,
            t_at_min: t_at,
            len: seg.length_m,
            intk: internal_kink_deg(seg),
            lane_off: meta.map(|m| m.lane_offset_right_m).unwrap_or(f32::NAN),
            on_seg_map: idx < road_n
                && seg_by_from_to.get(&(seg.from_uid, seg.to_uid)) == Some(&idx),
            dir: if idx < road_n {
                dir_by_edge_uid
                    .get(&seg.edge_uid)
                    .cloned()
                    .unwrap_or_else(|| "?".into())
            } else {
                "navcurve".into()
            },
            mid_x: mid.x as f64,
        });
    }
    rows.sort_by(|a, b| a.dist.total_cmp(&b.dist));

    let n_road = rows.iter().filter(|r| r.kind == "ROAD").count();
    let n_nav = rows.iter().filter(|r| r.kind == "NAVCURVE").count();
    println!(
        "SECTION 1a — SEGMENTS within {:.0}m  ({} ROAD, {} NAVCURVE)",
        args.radius, n_road, n_nav
    );
    println!(
        "  {:>5} {:>8} {:>9} {:>7} {:>7} {:>7} {:>22} {:>22} {:>7} onMap dir",
        "kind", "idx", "dist_m", "len_m", "intk°", "laneoff", "from", "to", "t@min"
    );
    for r in &rows {
        println!(
            "  {:>5} {:>8} {:>9.2} {:>7.1} {:>7.1} {:>7.2} {:>22} {:>22} {:>7.2} {:>5} {}",
            r.kind,
            r.idx,
            r.dist,
            r.len,
            r.intk,
            r.lane_off,
            r.from,
            r.to,
            r.t_at_min,
            r.on_seg_map,
            r.dir
        );
    }
    println!();

    // ── 1c focus: NavCurve availability on the WEST vs EAST approach ──
    let nav_rows: Vec<&Row> = rows.iter().filter(|r| r.kind == "NAVCURVE").collect();
    let nav_west = nav_rows.iter().filter(|r| r.mid_x < args.jx).count();
    let nav_east = nav_rows.iter().filter(|r| r.mid_x >= args.jx).count();
    println!("SECTION 1c — NAVCURVE adjacency (tangent-copy source)");
    println!(
        "  NavCurve segments within radius        : {}",
        nav_rows.len()
    );
    println!("  ... with midpoint WEST of junction (x<jx): {}", nav_west);
    println!("  ... with midpoint EAST of junction (x>=jx): {}", nav_east);
    println!(
        "  (Plan-A C1 trick needs a NavCurve on BOTH sides of the gap to copy seam tangents.)"
    );
    println!();

    // road segments spanning the junction with high internal kink (prefab_curve trigger)
    let kinky_road: Vec<&Row> = rows
        .iter()
        .filter(|r| r.kind == "ROAD" && r.intk > PREFAB_CURVE_FALLBACK_DEG as f64)
        .collect();
    println!("SECTION 1a' — ROAD segments with internal kink > {:.0}° (would latch prefab_curve→Catmull):", PREFAB_CURVE_FALLBACK_DEG);
    if kinky_road.is_empty() {
        println!("  (none)");
    } else {
        for r in &kinky_road {
            let capped = r.intk > INTK_PLAUSIBLE_MAX_DEG as f64;
            println!(
                "  idx {:>8}  {}->{}  intk={:.1}°  dist={:.2}m  onMap={}  {}  [{}]",
                r.idx,
                r.from,
                r.to,
                r.intk,
                r.dist,
                r.on_seg_map,
                r.dir,
                if capped {
                    "ABOVE intk_cap(100°): NO latch (artifact regime)"
                } else {
                    "in latch band → Catmull"
                }
            );
        }
    }
    println!();

    // ── SECTION 1b: route-aware reachability ──
    println!("SECTION 1b — ROUTE-AWARE REACHABILITY");
    let west_pt = args.west.unwrap_or((args.jx - args.auto_dist, args.jz));
    let east_pt = args.east.unwrap_or((args.jx + args.auto_dist, args.jz));
    println!(
        "  west endpoint target  : ({:.1}, {:.1}){}",
        west_pt.0,
        west_pt.1,
        if args.west.is_some() {
            " [explicit]"
        } else {
            " [auto]"
        }
    );
    println!(
        "  east endpoint target  : ({:.1}, {:.1}){}",
        east_pt.0,
        east_pt.1,
        if args.east.is_some() {
            " [explicit]"
        } else {
            " [auto]"
        }
    );

    let snap_w = snap_node(&node_pos, &has_edge, west_pt.0, west_pt.1);
    let snap_e = snap_node(&node_pos, &has_edge, east_pt.0, east_pt.1);
    let mut best_route: Option<(Vec<u64>, f64, &str)> = None;
    if let (Some((wn, wd)), Some((en, ed))) = (snap_w, snap_e) {
        println!(
            "  west node {} ({:.1}m away),  east node {} ({:.1}m away)",
            wn, wd, en, ed
        );
        // try both directions; keep the one whose path passes closest to the junction
        for (s, g, label) in [(wn, en, "W->E"), (en, wn, "E->W")] {
            if let Some((path, total)) = astar(&adj, &node_pos, s, g) {
                let closest = path
                    .iter()
                    .filter_map(|u| node_pos.get(u))
                    .map(|&(x, z)| dist_xz(args.jx, args.jz, x, z))
                    .fold(f64::MAX, f64::min);
                println!(
                    "  A* {label}: {} hops, {:.0}m total, closest approach to junction {:.1}m",
                    path.len(),
                    total,
                    closest
                );
                let take = match &best_route {
                    None => true,
                    Some((bp, _, _)) => {
                        let bc = bp
                            .iter()
                            .filter_map(|u| node_pos.get(u))
                            .map(|&(x, z)| dist_xz(args.jx, args.jz, x, z))
                            .fold(f64::MAX, f64::min);
                        closest < bc
                    }
                };
                if take {
                    best_route = Some((path, total, label));
                }
            } else {
                println!("  A* {label}: NO PATH");
            }
        }
    } else {
        println!("  ERROR: could not snap west/east endpoints to graph nodes");
    }

    if let Some((path, _total, label)) = &best_route {
        println!();
        println!(
            "  Chosen route: {label}, {} nodes. Junction-neighborhood hops (within {:.0}m):",
            path.len(),
            args.radius * 1.5
        );
        // build cached_route_seg_set
        let mut route_seg_set: std::collections::HashSet<usize> = std::collections::HashSet::new();
        for w in path.windows(2) {
            if let Some(&s) = seg_by_from_to.get(&(w[0], w[1])) {
                route_seg_set.insert(s);
            }
        }
        println!(
            "  cached_route_seg_set size (ROAD hops resolved): {}",
            route_seg_set.len()
        );
        println!(
            "  {:>8} {:>22} {:>22} {:>9} {:>8} {:>7} onMap segIdx intk° spanGap",
            "hop#", "from", "to", "fromDist", "toDist", "len"
        );
        let mut gap_hop_segs: Vec<usize> = Vec::new();
        for (hi, w) in path.windows(2).enumerate() {
            let fp = node_pos.get(&w[0]).copied();
            let tp = node_pos.get(&w[1]).copied();
            let fd = fp
                .map(|(x, z)| dist_xz(args.jx, args.jz, x, z))
                .unwrap_or(f64::MAX);
            let td = tp
                .map(|(x, z)| dist_xz(args.jx, args.jz, x, z))
                .unwrap_or(f64::MAX);
            if fd.min(td) > args.radius * 1.5 {
                continue;
            }
            let seg = seg_by_from_to.get(&(w[0], w[1])).copied();
            let intk = seg.map(|s| internal_kink_deg(&segments[s])).unwrap_or(-1.0);
            let dist_to_j = seg
                .map(|s| min_dist_to_seg(&segments[s], args.jx, args.jz, 24).0)
                .unwrap_or(f64::MAX);
            let span = dist_to_j <= args.radius;
            if span {
                if let Some(s) = seg {
                    gap_hop_segs.push(s);
                }
            }
            let flen = seg.map(|s| segments[s].length_m).unwrap_or(0.0);
            println!(
                "  {:>8} {:>22} {:>22} {:>9.1} {:>8.1} {:>7.1} {:>5} {:>6} {:>5.1} {}",
                hi,
                w[0],
                w[1],
                fd,
                td,
                flen,
                seg.is_some(),
                seg.map(|s| s as i64).unwrap_or(-1),
                intk,
                span
            );
        }
        println!();

        // ── Route-walk dispatch sampler ──
        // Simulate the lane-keeper at truck positions interpolated along the route
        // polyline through the junction neighborhood. This is the faithful test: if NO
        // sample yields a Catmull verdict, the corridor is already handled by spline_road.
        let rss = route_seg_set.clone();
        let mut verdict: HashMap<&str, usize> = HashMap::new();
        let mut catmull_pts: Vec<(f64, f64, &'static str, f64, f64)> = Vec::new();
        let mut samples = 0usize;
        let step_m = 4.0;
        for w in path.windows(2) {
            let (Some(&fp), Some(&tp)) = (node_pos.get(&w[0]), node_pos.get(&w[1])) else {
                continue;
            };
            let hop_len = dist_xz(fp.0, fp.1, tp.0, tp.1);
            let near =
                dist_xz(args.jx, args.jz, fp.0, fp.1).min(dist_xz(args.jx, args.jz, tp.0, tp.1));
            if near > args.radius * 2.0 {
                continue;
            }
            let hdg = ((tp.0 - fp.0)
                .atan2(-(tp.1 - fp.1))
                .to_degrees()
                .rem_euclid(360.0)) as f32;
            let nsteps = ((hop_len / step_m).ceil() as i32).max(1);
            for k in 0..=nsteps {
                let a = k as f64 / nsteps as f64;
                let qx = fp.0 + a * (tp.0 - fp.0);
                let qz = fp.1 + a * (tp.1 - fp.1);
                if dist_xz(args.jx, args.jz, qx, qz) > args.radius * 1.5 {
                    continue;
                }
                samples += 1;
                let q = Vec3::new(qx as f32, 0.0, qz as f32);
                let ra =
                    index.nearest_with_projection_filtered(q, ROUTE_NEAREST_CANDIDATES, |i, _| {
                        rss.contains(&i)
                    });
                let gl = index.nearest_with_heading_filter(q, hdg, 8);
                let gated = ra.as_ref().filter(|h| {
                    if h.dist_m > MAX_HOP_PROJECTION_DIST_M {
                        return false;
                    }
                    let mut d = (h.heading_deg - hdg).rem_euclid(360.0);
                    if d > 180.0 {
                        d -= 360.0;
                    }
                    d.abs() <= 60.0
                });
                let (label, reason, dist, ik): (&str, &'static str, f64, f64) = match (&gated, &gl)
                {
                    (Some(h), _) => {
                        // intK the truck actually drives: projection-t → segment end (Alternative A).
                        let ik = kink_between(&segments[h.segment_idx], h.t, 1.0);
                        if ik > PREFAB_CURVE_FALLBACK_DEG as f64
                            && ik <= INTK_PLAUSIBLE_MAX_DEG as f64
                        {
                            ("CATMULL:prefab_curve", "prefab_curve", h.dist_m as f64, ik)
                        } else if metadata[h.segment_idx]
                            .map(|m| m.is_prefab)
                            .unwrap_or(false)
                        {
                            ("spline_prefab", "", h.dist_m as f64, ik)
                        } else {
                            ("spline_road", "", h.dist_m as f64, ik)
                        }
                    }
                    (None, Some(g)) => {
                        let gseg = &segments[g.segment_idx];
                        let on_pair = path
                            .windows(2)
                            .any(|p| p[0] == gseg.from_uid && p[1] == gseg.to_uid);
                        let feeds = path.contains(&gseg.to_uid);
                        if on_pair || feeds {
                            ("spline_global_onroute", "", g.dist_m as f64, -1.0)
                        } else {
                            ("CATMULL:off_route", "off_route", g.dist_m as f64, -1.0)
                        }
                    }
                    (None, None) => ("CATMULL:route_miss", "route_miss", -1.0, -1.0),
                };
                *verdict.entry(label).or_default() += 1;
                if label.starts_with("CATMULL") {
                    catmull_pts.push((qx, qz, reason, dist, ik));
                }
            }
        }
        println!(
            "  ── Route-walk dispatch sampler ({} samples, {:.0}m step, within {:.0}m of junction) ──",
            samples,
            step_m,
            args.radius * 1.5
        );
        let mut vk: Vec<(&&str, &usize)> = verdict.iter().collect();
        vk.sort_by(|a, b| b.1.cmp(a.1));
        for (k, c) in vk {
            println!("    {:<26} {}", k, c);
        }
        let catmull_total: usize = verdict
            .iter()
            .filter(|(k, _)| k.starts_with("CATMULL"))
            .map(|(_, c)| *c)
            .sum();
        println!(
            "    => CATMULL samples on this corridor: {} / {}",
            catmull_total, samples
        );
        for (x, z, reason, dist, ik) in catmull_pts.iter().take(12) {
            println!(
                "       CATMULL @ ({:.1},{:.1}) reason={} dist={:.1}m intk={:.1}°",
                x, z, reason, dist, ik
            );
        }
        println!();

        // ── Simulate the route-aware nearest query at the junction center (reference) ──
        // Query point = junction center; heading taken from the route tangent near the junction.
        let q = Vec3::new(args.jx as f32, 0.0, args.jz as f32);
        // heading: direction of the nearest junction hop
        let heading_deg = path
            .windows(2)
            .filter_map(|w| {
                let f = node_pos.get(&w[0])?;
                let t = node_pos.get(&w[1])?;
                let md = dist_xz(args.jx, args.jz, (f.0 + t.0) / 2.0, (f.1 + t.1) / 2.0);
                Some((md, (t.0 - f.0, t.1 - f.1)))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, (dx, dz))| dx.atan2(-dz).to_degrees().rem_euclid(360.0))
            .unwrap_or(90.0) as f32;
        println!(
            "  Simulated query at junction center, heading {:.1}° (route tangent):",
            heading_deg
        );

        let global = index.nearest_with_heading_filter(q, heading_deg, 8);
        let route_aware =
            index.nearest_with_projection_filtered(q, ROUTE_NEAREST_CANDIDATES, |i, _| {
                rss.contains(&i)
            });

        let describe =
            |label: &str, hit: &Option<truckpilot_map_parser::spline_index::NearestHit>| match hit {
                None => println!("    {label}: NONE"),
                Some(h) => {
                    let seg = &segments[h.segment_idx];
                    let intk = internal_kink_deg(seg);
                    let m = metadata[h.segment_idx];
                    println!(
                        "    {label}: seg {} [{}] {}->{} dist={:.2}m t={:.2} hdg={:.1}° intk={:.1}° is_prefab={} laneoff={:.2} hfilt={}",
                        h.segment_idx,
                        kind_of(h.segment_idx, road_n),
                        seg.from_uid,
                        seg.to_uid,
                        h.dist_m,
                        h.t,
                        h.heading_deg,
                        intk,
                        m.map(|x| x.is_prefab).unwrap_or(false),
                        m.map(|x| x.lane_offset_right_m).unwrap_or(f32::NAN),
                        h.heading_filter_applied
                    );
                }
            };
        describe("global  (nearest_with_heading_filter)", &global);
        describe("route-aware (filtered to route_seg_set)", &route_aware);

        // apply lane-keeper gate to route_aware
        println!();
        println!("  ── Lane-keeper dispatch verdict at this query ──");
        let gated = route_aware.as_ref().filter(|h| {
            if h.dist_m > MAX_HOP_PROJECTION_DIST_M {
                return false;
            }
            let mut d = (h.heading_deg - heading_deg).rem_euclid(360.0);
            if d > 180.0 {
                d -= 360.0;
            }
            d.abs() <= 60.0
        });
        match (&gated, &global) {
            (Some(h), _) => {
                let seg = &segments[h.segment_idx];
                let intk = internal_kink_deg(seg);
                if intk > PREFAB_CURVE_FALLBACK_DEG as f64 && intk <= INTK_PLAUSIBLE_MAX_DEG as f64
                {
                    println!("    route-aware ROAD hit passes gate BUT intk={:.1}° in latch band → prefab_curve → CATMULL today.", intk);
                    println!("    => Plan A target case: swap seam tangents to drop intk below {:.0}° so spline_road tracks the gap.", PREFAB_CURVE_FALLBACK_DEG);
                } else if intk > INTK_PLAUSIBLE_MAX_DEG as f64 {
                    println!("    route-aware ROAD hit passes gate, intk={:.1}° ABOVE cap → no latch → spline_road already tracks (artifact regime).", intk);
                } else {
                    println!("    route-aware ROAD hit passes gate, intk={:.1}° below threshold → spline_road tracks today (NO Catmull).", intk);
                }
            }
            (None, Some(_g)) => {
                println!("    route-aware produced no gated hit → lane-keeper uses GLOBAL hit (may be off-route → Catmull).");
            }
            (None, None) => {
                println!("    neither route-aware nor global hit → route_miss → CATMULL.");
            }
        }
    } else {
        println!("  no usable route — falling back to structural reasoning only (see SECTION 1a).");
    }

    // ── SECTION 2 — TURN-ROUTE FINDER: a drivable start/goal that forces the turn ──
    if let Some((tf, tt)) = args.turn_edge {
        println!();
        println!("================================================================");
        println!(
            "SECTION 2 — TURN-ROUTE FINDER  (force A* through {} -> {})",
            tf, tt
        );
        println!("================================================================");

        let edge_ok = adj
            .get(&tf)
            .map(|v| v.iter().any(|&(n, _)| n == tt))
            .unwrap_or(false);
        let turn_seg = seg_by_from_to.get(&(tf, tt)).copied();
        let turn_intk = turn_seg
            .map(|s| internal_kink_deg(&segments[s]))
            .unwrap_or(-1.0);
        let tfp = node_pos.get(&tf).copied();
        let ttp = node_pos.get(&tt).copied();
        println!(
            "  turn edge in graph: {}   segIdx={:?}   intk={:.1}°   entry={:?} exit={:?}",
            edge_ok, turn_seg, turn_intk, tfp, ttp
        );

        if !edge_ok {
            println!("  ERROR: turn edge not present in router graph — cannot force it.");
        } else {
            // reverse adjacency for backward reachability to the entry node
            let mut radj: HashMap<u64, Vec<(u64, f64)>> = HashMap::new();
            for (&f, outs) in &adj {
                for &(t, c) in outs {
                    radj.entry(t).or_default().push((f, c));
                }
            }
            let max_d = args.ring_max * 2.0;
            let drev = dijkstra_dist(&radj, tf, max_d); // cost node -> tf (entry)
            let dfwd = dijkstra_dist(&adj, tt, max_d); // cost tt (exit) -> node

            let indeg = |u: u64| radj.get(&u).map_or(0, |v| v.len());
            let outdeg = |u: u64| adj.get(&u).map_or(0, |v| v.len());

            // Candidates are sorted CLOSEST-FIRST by graph distance: a start/goal pair
            // close to the junction is unlikely to have a parallel route that bypasses
            // the turn, so its shortest A* path is forced through tf->tt. We still require
            // a minimum run-up (ring_min) so the truck has road before the junction.
            let mut start_c: Vec<(u64, f64)> = drev
                .iter()
                .filter(|(_, &d)| d >= args.ring_min && d <= args.ring_max)
                .map(|(&u, &d)| (u, d))
                .filter(|&(u, _)| u != tf && u != tt && outdeg(u) >= 1 && indeg(u) >= 1)
                .collect();
            start_c.sort_by(|a, b| a.1.total_cmp(&b.1));
            start_c.truncate(40);

            let mut goal_c: Vec<(u64, f64)> = dfwd
                .iter()
                .filter(|(_, &d)| d >= args.ring_min && d <= args.ring_max)
                .map(|(&u, &d)| (u, d))
                .filter(|&(u, _)| u != tf && u != tt && outdeg(u) >= 1 && indeg(u) >= 1)
                .collect();
            goal_c.sort_by(|a, b| a.1.total_cmp(&b.1));
            goal_c.truncate(40);

            // pick the (start, goal) whose SHORTEST A* path uses tf->tt consecutively
            let mut chosen: Option<(u64, u64, Vec<u64>, f64)> = None;
            'outer: for &(s, _) in &start_c {
                for &(g, _) in &goal_c {
                    if let Some((path, total)) = astar(&adj, &node_pos, s, g) {
                        if path.windows(2).any(|w| w[0] == tf && w[1] == tt) {
                            chosen = Some((s, g, path, total));
                            break 'outer;
                        }
                    }
                }
            }

            match chosen {
                None => {
                    println!(
                        "  No start/goal pair (of {}×{} candidates) whose shortest path uses the turn.",
                        start_c.len(),
                        goal_c.len()
                    );
                    println!("  Try a wider window: --ring-min/--ring-max.");
                }
                Some((s, g, path, total)) => {
                    let sp = node_pos.get(&s).copied().unwrap_or((0.0, 0.0));
                    let gp = node_pos.get(&g).copied().unwrap_or((0.0, 0.0));
                    let next = path
                        .get(1)
                        .and_then(|u| node_pos.get(u))
                        .copied()
                        .unwrap_or(sp);
                    let bearing = (next.0 - sp.0)
                        .atan2(-(next.1 - sp.1))
                        .to_degrees()
                        .rem_euclid(360.0);
                    let cardinal = bearing_cardinal(bearing);

                    println!();
                    println!(
                        "  START node {}  pos=({:.1}, {:.1})   face bearing {:.0}° ({})",
                        s, sp.0, sp.1, bearing, cardinal
                    );
                    println!("  GOAL  node {}  pos=({:.1}, {:.1})", g, gp.0, gp.1);
                    println!(
                        "  Route: {} hops, {:.0}m total — turn {}→{} (segIdx {:?}, intk {:.1}°) INCLUDED.",
                        path.len() - 1,
                        total,
                        tf,
                        tt,
                        turn_seg,
                        turn_intk
                    );
                    println!();
                    println!("  Hop list (cum = cumulative metres from start):");
                    println!(
                        "    {:>4} {:>22} -> {:>22} {:>8} {:>9}  mark",
                        "i", "from", "to", "len_m", "cum_m"
                    );
                    let mut cum = 0.0;
                    for (hi, w) in path.windows(2).enumerate() {
                        let fp = node_pos.get(&w[0]).copied().unwrap_or((0.0, 0.0));
                        let tp = node_pos.get(&w[1]).copied().unwrap_or((0.0, 0.0));
                        let len = dist_xz(fp.0, fp.1, tp.0, tp.1);
                        cum += len;
                        let mark = if w[0] == tf && w[1] == tt {
                            "<== TURN (Catmull trigger)"
                        } else if dist_xz(args.jx, args.jz, fp.0, fp.1)
                            .min(dist_xz(args.jx, args.jz, tp.0, tp.1))
                            <= args.radius
                        {
                            "junction"
                        } else {
                            ""
                        };
                        println!(
                            "    {:>4} {:>22} -> {:>22} {:>8.1} {:>9.1}  {}",
                            hi, w[0], w[1], len, cum, mark
                        );
                    }
                    println!();

                    // Verify on THIS drivable route that the junction passage Catmulls.
                    let rss: std::collections::HashSet<usize> = path
                        .windows(2)
                        .filter_map(|w| seg_by_from_to.get(&(w[0], w[1])).copied())
                        .collect();
                    let mut catmull = 0usize;
                    let mut samples = 0usize;
                    let mut first_catmull: Option<(f64, f64, f64)> = None;
                    for w in path.windows(2) {
                        let (Some(&fp), Some(&tp)) = (node_pos.get(&w[0]), node_pos.get(&w[1]))
                        else {
                            continue;
                        };
                        let near = dist_xz(args.jx, args.jz, fp.0, fp.1)
                            .min(dist_xz(args.jx, args.jz, tp.0, tp.1));
                        if near > args.radius * 2.0 {
                            continue;
                        }
                        let hop_len = dist_xz(fp.0, fp.1, tp.0, tp.1);
                        let hdg = ((tp.0 - fp.0)
                            .atan2(-(tp.1 - fp.1))
                            .to_degrees()
                            .rem_euclid(360.0)) as f32;
                        let nsteps = ((hop_len / 4.0).ceil() as i32).max(1);
                        for k in 0..=nsteps {
                            let a = k as f64 / nsteps as f64;
                            let qx = fp.0 + a * (tp.0 - fp.0);
                            let qz = fp.1 + a * (tp.1 - fp.1);
                            if dist_xz(args.jx, args.jz, qx, qz) > args.radius * 1.5 {
                                continue;
                            }
                            samples += 1;
                            let q = Vec3::new(qx as f32, 0.0, qz as f32);
                            let ra = index.nearest_with_projection_filtered(
                                q,
                                ROUTE_NEAREST_CANDIDATES,
                                |i, _| rss.contains(&i),
                            );
                            let gated = ra.as_ref().filter(|h| {
                                if h.dist_m > MAX_HOP_PROJECTION_DIST_M {
                                    return false;
                                }
                                let mut d = (h.heading_deg - hdg).rem_euclid(360.0);
                                if d > 180.0 {
                                    d -= 360.0;
                                }
                                d.abs() <= 60.0
                            });
                            let is_catmull = match &gated {
                                Some(h) => {
                                    let ik = kink_between(&segments[h.segment_idx], h.t, 1.0);
                                    ik > PREFAB_CURVE_FALLBACK_DEG as f64
                                        && ik <= INTK_PLAUSIBLE_MAX_DEG as f64
                                }
                                None => match index.nearest_with_heading_filter(q, hdg, 8) {
                                    Some(gl) => {
                                        let gseg = &segments[gl.segment_idx];
                                        let on = path
                                            .windows(2)
                                            .any(|p| p[0] == gseg.from_uid && p[1] == gseg.to_uid)
                                            || path.contains(&gseg.to_uid);
                                        !on
                                    }
                                    None => true,
                                },
                            };
                            if is_catmull {
                                catmull += 1;
                                if first_catmull.is_none() {
                                    let ik = gated
                                        .as_ref()
                                        .map(|h| kink_between(&segments[h.segment_idx], h.t, 1.0))
                                        .unwrap_or(-1.0);
                                    first_catmull = Some((qx, qz, ik));
                                }
                            }
                        }
                    }
                    println!(
                        "  Verify on THIS route: {} / {} junction samples would Catmull (prefab_curve).",
                        catmull, samples
                    );
                    if let Some((cx, cz, cik)) = first_catmull {
                        println!("    first Catmull @ ({:.1},{:.1}) intk≈{:.1}°", cx, cz, cik);
                    }
                    println!();
                    println!("  ── COPY-PASTE ENGAGE SEQUENCE ──");
                    println!(
                        "  1) Truck in ETS2 platzieren bei  X={:.1}  Z={:.1}   (Blick Richtung {:.0}° = {})",
                        sp.0, sp.1, bearing, cardinal
                    );
                    println!("        muss <50 m vom Startknoten {} sein (Daemon snappt auf nearest node).", s);
                    println!(
                        "  2) .\\target\\release\\engage-cli.exe set-goal-pos --x={:.1} --z={:.1}",
                        gp.0, gp.1
                    );
                    println!(
                        "        robuster (expliziter Knoten): .\\target\\release\\engage-cli.exe set-goal {}",
                        g
                    );
                    println!(
                        "  3) .\\target\\release\\engage-cli.exe set-cruise {:.0}",
                        args.cruise
                    );
                    println!("  4) .\\target\\release\\engage-cli.exe engage");
                    println!("  5) Live beobachten:");
                    println!("        .\\target\\release\\blackboard-query.exe --keys lane_keeper.fallback_reason,lane_keeper.prefab_curve_fallback,lane_keeper.final_internal_kink_deg,lane_keeper.internal_kink_over_threshold");
                }
            }
        }
    }

    println!();
    println!("================================================================");
    println!("Done in {:.1}s", t0.elapsed().as_secs_f64());
}
