// Quick test: parse real base_map.scs
//
// Path is taken from the `TRUCKPILOT_BASE_SCS` env var; on a fresh checkout
// the test will skip silently. Example (PowerShell):
//   $env:TRUCKPILOT_BASE_SCS = "C:\...\base_map.scs"
use std::path::PathBuf;
use std::time::Instant;

#[test]
fn test_real_base_map_scs() {
    let path = match std::env::var("TRUCKPILOT_BASE_SCS") {
        Ok(s) => PathBuf::from(s),
        Err(_) => {
            eprintln!("TRUCKPILOT_BASE_SCS not set, skipping");
            return;
        }
    };
    if !path.is_file() {
        eprintln!("base_map.scs not found at {:?}, skipping", path);
        return;
    }
    let path = path.as_path();

    // Open archive
    let mut archive = match truckpilot::ets2_parser::scs_reader::ScsArchive::open(path) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("SCS open error: {}", e);
            return;
        }
    };
    let hashes = archive.entry_hashes();
    println!(
        "base_map.scs has {} entries (from header)",
        archive.num_entries()
    );
    println!("base_map.scs has {} entries in hash map", hashes.len());

    // Collect ALL map sectors (no take(10))
    let all_possible: Vec<String> = truckpilot::ets2_parser::scs_reader::list_map_sector_paths();
    let sectors: Vec<String> = all_possible
        .into_iter()
        .filter(|p| p.starts_with("map/europe/sec") && p.ends_with(".base"))
        .collect();

    println!("Probing {} possible sector paths...", sectors.len());

    let mut found = 0;
    let mut total_nodes = 0usize;
    let mut total_roads = 0usize;
    let mut total_prefabs = 0usize;
    let mut sector_counts = Vec::new();
    let overall_start = Instant::now();

    for s in &sectors {
        match archive.read_file(s) {
            Ok(data) => {
                found += 1;
                let parse_start = Instant::now();
                match truckpilot::ets2_parser::binary_parser::parse_binary_sector(&data) {
                    Ok(sector) => {
                        let parse_time = parse_start.elapsed();
                        total_nodes += sector.nodes.len();
                        total_roads += sector.roads.len();
                        total_prefabs += sector.prefabs.len();
                        sector_counts.push((
                            s.clone(),
                            sector.nodes.len(),
                            sector.roads.len(),
                            sector.prefabs.len(),
                        ));
                        if found <= 5 || found % 50 == 0 {
                            println!(
                                "  FOUND {} : {} bytes -> {}n/{}r/{}p ({:?})",
                                s,
                                data.len(),
                                sector.nodes.len(),
                                sector.roads.len(),
                                sector.prefabs.len(),
                                parse_time
                            );
                        }
                    }
                    Err(e) => {
                        if found <= 5 {
                            println!("    Binary parse failed for {}: {}", s, e);
                        }
                    }
                }
            }
            Err(_e) => {}
        }
    }

    let overall_elapsed = overall_start.elapsed();
    let throughput = if overall_elapsed.as_secs_f64() > 0.0 {
        found as f64 / overall_elapsed.as_secs_f64()
    } else {
        0.0
    };

    println!("Found {}/{} sectors", found, sectors.len());
    println!(
        "TOTAL: {} nodes, {} roads, {} prefabs",
        total_nodes, total_roads, total_prefabs
    );
    println!(
        "Total time: {:?} -> {:.1} Sektoren/s",
        overall_elapsed, throughput
    );

    // Write sector_counts.txt
    use std::io::Write;
    let mut log = std::fs::File::create("sector_counts.txt").unwrap();
    writeln!(log, "sector,nodes,roads,prefabs").unwrap();
    for (path, n, r, p) in &sector_counts {
        writeln!(log, "{},{},{},{}", path, n, r, p).unwrap();
    }
    println!(
        "Wrote sector_counts.txt with {} entries",
        sector_counts.len()
    );

    // Read entry by hash (first 5 non-zero-sized entries)
    let mut count = 0;
    for &hash in &hashes {
        if count >= 5 {
            break;
        }
        match archive.read_entry(hash) {
            Ok(data) => {
                count += 1;
                // Try text parse
                if let Ok(text) = std::str::from_utf8(&data) {
                    if text.len() < 200 {
                        println!(
                            "  hash 0x{:016X}: text ({} bytes): {}",
                            hash,
                            data.len(),
                            &text[..text.len().min(100)]
                        );
                    } else {
                        println!(
                            "  hash 0x{:016X}: text ({} bytes): {}...",
                            hash,
                            data.len(),
                            &text[..100]
                        );
                    }
                } else {
                    println!("  hash 0x{:016X}: binary ({} bytes)", hash, data.len());
                    // Try binary parse
                    let start = Instant::now();
                    match truckpilot::ets2_parser::binary_parser::parse_binary_sector(&data) {
                        Ok(sector) => {
                            let elapsed = start.elapsed();
                            println!(
                                "    -> {} nodes, {} roads, {} prefabs",
                                sector.nodes.len(),
                                sector.roads.len(),
                                sector.prefabs.len()
                            );
                            println!("    -> parse time: {:?}", elapsed);
                        }
                        Err(e) => println!("    -> binary parse failed: {}", e),
                    }
                }
            }
            Err(e) => println!("  hash 0x{:016X}: read error: {}", hash, e),
        }
    }
}
