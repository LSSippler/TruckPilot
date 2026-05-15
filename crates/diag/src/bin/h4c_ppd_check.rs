//! H4c A1 quick-check — count `.ppd` entries across the production load order.
use std::collections::HashSet;
use std::fs::{self, File};
use std::io::Write as _;
use std::path::PathBuf;
use truckpilot_map_parser::hashfs::{parse_directory_listing, scs_path_hash, HashFsArchive};
use truckpilot_map_parser::mod_loader::ModLoadOrder;

fn walk(arc: &HashFsArchive) -> Vec<String> {
    let mut hits = Vec::new();
    let mut stack = vec![String::new()];
    let mut seen = HashSet::new();
    while let Some(dir) = stack.pop() {
        if !seen.insert(dir.clone()) { continue; }
        let bytes = match arc.read_hash(scs_path_hash(0, &dir.to_lowercase())) {
            Ok(b) => b, Err(_) => continue };
        let items = match parse_directory_listing(&bytes) { Ok(v) => v, Err(_) => continue };
        for it in items {
            let child = if dir.is_empty() { it.name.clone() } else { format!("{dir}/{}", it.name) };
            if it.is_dir { stack.push(child); }
            else if it.name.to_ascii_lowercase().ends_with(".ppd") { hits.push(child); }
        }
    }
    hits
}

fn main() {
    let ets2 = std::env::args().skip_while(|a| a != "--ets2-dir").nth(1)
        .unwrap_or_else(|| r"C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2".into());
    let mods = std::env::args().skip_while(|a| a != "--mods-dir").nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| dirs::document_dir().unwrap().join("Euro Truck Simulator 2/mod"));
    let order = ModLoadOrder::from_directories(&PathBuf::from(&ets2), &mods).expect("loader");
    let mut per: Vec<(String, Vec<String>)> = order.entries.iter().filter_map(|e| {
        HashFsArchive::open(&e.path).ok().map(|a| (e.name.clone(), walk(&a)))
    }).collect();
    per.sort_by_key(|(_, v)| std::cmp::Reverse(v.len()));
    let total: usize = per.iter().map(|(_, v)| v.len()).sum();
    let mut buf = String::new();
    buf.push_str(&format!("H4c A1 QUICK-CHECK — PPD Availability\nArchives: {}\n\n", order.entries.len()));
    buf.push_str(&format!("{:<35} | {:>9} | sample\n", "Archive", "PPDs"));
    for (n, h) in &per {
        let s = h.first().map(String::as_str).unwrap_or("(none)");
        buf.push_str(&format!("{:<35} | {:>9} | {}\n", n, h.len(), s));
    }
    let verdict = if total > 100 { "GO" } else if total < 10 { "SKIP" } else { "REVIEW" };
    buf.push_str(&format!("\nGLOBAL TOTAL: {total} .ppd files\nRECOMMENDATION: {verdict}\n"));
    fs::create_dir_all("outputs/claude").ok();
    File::create("outputs/h4c_a1_quick_check.txt").unwrap().write_all(buf.as_bytes()).unwrap();
    File::create("outputs/claude/h4c_a1_quick_check.txt").unwrap().write_all(buf.as_bytes()).unwrap();
    print!("{buf}");
}
