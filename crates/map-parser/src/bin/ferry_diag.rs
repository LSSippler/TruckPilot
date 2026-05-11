//! `truckpilot-ferry-diag` — Phase 5.22 ferry-port-token diagnostic.

use std::collections::HashMap;

use truckpilot_map_parser::sector::audit_sector;
use truckpilot_map_parser::{Archive, HashFsArchive};

const FERRY_BODY_SIZE: usize = 89;

fn extract_ferry_bodies(data: &[u8]) -> Vec<Vec<u8>> {
    let report = audit_sector(data);
    let mut out = Vec::new();
    for item in &report.items {
        if item.kind_name == "ferry" {
            let body_start = item.start_offset + 4;
            let body_end = item.end_offset;
            if body_end > body_start && body_end <= data.len() {
                let bytes = data[body_start..body_end].to_vec();
                if bytes.len() == FERRY_BODY_SIZE {
                    out.push(bytes);
                }
            }
        }
    }
    out
}

fn main() {
    let mut a = HashFsArchive::open(
        r"C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2\base_map.scs",
    )
    .expect("open archive");
    let paths: Vec<String> = a
        .probe_sector_paths()
        .into_iter()
        .filter(|p| p.ends_with(".base"))
        .collect();

    let mut bodies: Vec<(String, Vec<u8>)> = Vec::new();
    for p in &paths {
        let Ok(d) = a.read_path(p) else { continue };
        for body in extract_ferry_bodies(&d) {
            bodies.push((p.clone(), body));
        }
    }
    println!("Total ferry bodies captured: {}", bodies.len());

    println!("\n=== u32 shared-value scan ===");
    for off in 0..=(FERRY_BODY_SIZE - 4) {
        let mut counts: HashMap<u32, usize> = HashMap::new();
        let mut zero_n = 0usize;
        for (_, body) in &bodies {
            let v = u32::from_le_bytes(body[off..off + 4].try_into().unwrap());
            if v == 0 {
                zero_n += 1;
            } else {
                *counts.entry(v).or_default() += 1;
            }
        }
        let shared = counts.values().filter(|c| **c >= 2).count();
        let max_share = counts.values().max().copied().unwrap_or(0);
        if max_share >= 2 {
            println!(
                "  off=+{off:2}  unique={:3}  zero={zero_n:2}  shared_groups={shared}  max_group={max_share}",
                counts.len()
            );
        }
    }
    println!("\n=== u64 shared-value scan ===");
    for off in 0..=(FERRY_BODY_SIZE - 8) {
        let mut counts: HashMap<u64, usize> = HashMap::new();
        let mut zero_n = 0usize;
        for (_, body) in &bodies {
            let v = u64::from_le_bytes(body[off..off + 8].try_into().unwrap());
            if v == 0 {
                zero_n += 1;
            } else {
                *counts.entry(v).or_default() += 1;
            }
        }
        let shared = counts.values().filter(|c| **c >= 2).count();
        let max_share = counts.values().max().copied().unwrap_or(0);
        if max_share >= 2 {
            println!(
                "  off=+{off:2}  unique={:3}  zero={zero_n:2}  shared_groups={shared}  max_group={max_share}",
                counts.len()
            );
        }
    }

    println!("\n=== Per-ferry decoded fields (TruckLib spec) ===");
    for (i, (path, body)) in bodies.iter().enumerate() {
        let uid = u64::from_le_bytes(body[0..8].try_into().unwrap());
        let flags = u32::from_le_bytes(body[48..52].try_into().unwrap());
        let view = body[52];
        let port_token = u64::from_le_bytes(body[53..61].try_into().unwrap());
        let prefab_uid = u64::from_le_bytes(body[61..69].try_into().unwrap());
        let node_uid = u64::from_le_bytes(body[69..77].try_into().unwrap());
        println!(
            "{i:3}  uid={uid:016x}  flags={flags:08x}  view={view:3}  pt={port_token:016x}  pf={prefab_uid:016x}  n={node_uid:016x}  [{}]",
            path
        );
    }
}
