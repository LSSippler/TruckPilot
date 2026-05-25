//! `compare-with-ets2la` — compare TruckPilot graph stats against ETS2LA reference
//!
//! Usage:
//!   cargo run --release -p truckpilot-diag --bin compare-with-ets2la -- \
//!     --stats-file outputs/2026-05-22/diag/graph_stats.json \
//!     --audit-csv  outputs/2026-05-22/diag/route_audit.csv \
//!     --out-dir    outputs/2026-05-22/diag

use std::collections::HashMap;
use std::io::Write as IoWrite;
use std::path::PathBuf;

use anyhow::{Context, Result};
use clap::Parser;

#[derive(Parser)]
#[command(
    name = "compare-with-ets2la",
    about = "Compare TruckPilot graph vs ETS2LA reference"
)]
struct Args {
    #[arg(long, default_value = "outputs/2026-05-22/diag/graph_stats.json")]
    stats_file: PathBuf,
    #[arg(long, default_value = "outputs/2026-05-22/diag/route_audit.csv")]
    audit_csv: PathBuf,
    #[arg(long, default_value = "outputs/2026-05-22/diag")]
    out_dir: PathBuf,
}

// ETS2LA reference numbers (hardcoded — kept for documentation even if not all used directly)
#[allow(dead_code)]
const ETS2LA_NODES: u64 = 1_145_092;
#[allow(dead_code)]
const ETS2LA_NAVIGATION_ENTRIES: u64 = 194_199;
const ETS2LA_ROADS: u64 = 251_295;
#[allow(dead_code)]
const ETS2LA_ROAD_LOOKS: u64 = 371;
#[allow(dead_code)]
const ETS2LA_FERRIES: u64 = 87;
const ETS2LA_PREFABS: u64 = 73_132;
#[allow(dead_code)]
const ETS2LA_PREFAB_DESCRIPTIONS: u64 = 3_182;
#[allow(dead_code)]
const ETS2LA_MODELS: u64 = 174_139;
const ETS2LA_SIGNS: u64 = 505_808;
#[allow(dead_code)]
const ETS2LA_CITIES: u64 = 374;
#[allow(dead_code)]
const ETS2LA_ITEM_MISSING: u64 = 10_411;
#[allow(dead_code)]
const ETS2LA_ITEM_SUCCESSFUL: u64 = 377_831;
#[allow(dead_code)]
const ETS2LA_ITEM_TOTAL: u64 = 388_242;
const ETS2LA_ITEM_RESOLUTION_RATE: f64 = 0.9732;
// Estimated edges: roads * 2 for bidirectional
const ETS2LA_ESTIMATED_EDGES: u64 = ETS2LA_ROADS * 2; // 502_590

#[allow(dead_code)]
struct StatsData {
    total_nodes: Option<u64>,
    total_edges: Option<u64>,
    total_signs: Option<u64>,
    total_prefabs: Option<u64>,
    edge_type_distribution: HashMap<String, u64>,
    raw_direction_distribution: HashMap<String, u64>,
    isolated_nodes: Option<u64>,
    source_only_nodes: Option<u64>,
    sink_only_nodes: Option<u64>,
    avg_out_degree: Option<f64>,
    prefab_nodes_total: Option<u64>,
    prefab_nodes_also_in_edges: Option<u64>,
}

fn load_stats(path: &PathBuf) -> Option<StatsData> {
    if !path.exists() {
        eprintln!(
            "[compare-with-ets2la] stats-file not found: {} — skipping",
            path.display()
        );
        return None;
    }
    let text = std::fs::read_to_string(path).ok()?;
    let v: serde_json::Value = serde_json::from_str(&text).ok()?;

    let edge_type_distribution = if let Some(obj) = v["edge_type_distribution"].as_object() {
        obj.iter()
            .filter_map(|(k, val)| val.as_u64().map(|n| (k.clone(), n)))
            .collect()
    } else {
        HashMap::new()
    };

    let raw_direction_distribution = if let Some(obj) = v["raw_direction_distribution"].as_object()
    {
        obj.iter()
            .filter_map(|(k, val)| val.as_u64().map(|n| (k.clone(), n)))
            .collect()
    } else {
        HashMap::new()
    };

    Some(StatsData {
        total_nodes: v["total_nodes"].as_u64(),
        total_edges: v["total_edges"].as_u64(),
        total_signs: v["total_signs"].as_u64(),
        total_prefabs: v["total_prefabs"].as_u64(),
        edge_type_distribution,
        raw_direction_distribution,
        isolated_nodes: v["isolated_nodes"].as_u64(),
        source_only_nodes: v["source_only_nodes"].as_u64(),
        sink_only_nodes: v["sink_only_nodes"].as_u64(),
        avg_out_degree: v["avg_out_degree"].as_f64(),
        prefab_nodes_total: v["prefab_nodes_total"].as_u64(),
        prefab_nodes_also_in_edges: v["prefab_nodes_also_in_edges"].as_u64(),
    })
}

struct AuditData {
    total_pairs: usize,
    cat4_success: usize,
    cat3_no_path: usize,
    cat1_snap: usize,
}

fn load_audit(path: &PathBuf) -> Option<AuditData> {
    if !path.exists() {
        eprintln!(
            "[compare-with-ets2la] audit-csv not found: {} — skipping",
            path.display()
        );
        return None;
    }
    let text = std::fs::read_to_string(path).ok()?;
    let mut total_pairs = 0usize;
    let mut cat4_success = 0usize;
    let mut cat3_no_path = 0usize;
    let mut cat1_snap = 0usize;

    for (i, line) in text.lines().enumerate() {
        if i == 0 {
            continue; // skip header
        }
        let cols: Vec<&str> = line.split(',').collect();
        if cols.len() < 3 {
            continue;
        }
        total_pairs += 1;
        match cols[2] {
            "CAT4_SUCCESS" => cat4_success += 1,
            "CAT3_NO_PATH" => cat3_no_path += 1,
            c if c.starts_with("CAT1_") => cat1_snap += 1,
            _ => {}
        }
    }

    Some(AuditData {
        total_pairs,
        cat4_success,
        cat3_no_path,
        cat1_snap,
    })
}

fn ratio_str(tp: Option<u64>, ets: u64) -> String {
    match tp {
        None => "N/A".to_string(),
        Some(v) => format!("{:.3}", v as f64 / ets as f64),
    }
}

fn fmt_opt_u64(v: Option<u64>) -> String {
    v.map(|n| n.to_string())
        .unwrap_or_else(|| "N/A".to_string())
}

fn hypothesis_status(confirmed: Option<bool>) -> &'static str {
    match confirmed {
        Some(true) => "BESTAETIGT",
        Some(false) => "WIDERLEGT",
        None => "UNKLAR",
    }
}

fn main() -> Result<()> {
    let args = Args::parse();

    let stats = load_stats(&args.stats_file);
    let audit = load_audit(&args.audit_csv);

    std::fs::create_dir_all(&args.out_dir)
        .with_context(|| format!("create out-dir: {}", args.out_dir.display()))?;

    let md_path = args.out_dir.join("ets2la_comparison.md");
    let mut md =
        std::fs::File::create(&md_path).with_context(|| format!("create {}", md_path.display()))?;

    writeln!(md, "# ETS2LA vs TruckPilot Comparison")?;
    writeln!(md)?;
    writeln!(md, "Generated: {}", chrono_lite())?;
    writeln!(md)?;

    // --- Comparison table ---
    writeln!(md, "## Metric Comparison")?;
    writeln!(md)?;
    writeln!(md, "| Metric | ETS2LA | TruckPilot | Ratio | Note |")?;
    writeln!(md, "|--------|--------|------------|-------|------|")?;

    let tp_nodes = stats.as_ref().and_then(|s| s.total_nodes);
    writeln!(
        md,
        "| nodes | {} | {} | {} | |",
        ETS2LA_NODES,
        fmt_opt_u64(tp_nodes),
        ratio_str(tp_nodes, ETS2LA_NODES),
    )?;

    let tp_edges = stats.as_ref().and_then(|s| s.total_edges);
    writeln!(
        md,
        "| edges | {} (est.) | {} | {} | ETS2LA estimated as roads×2={} |",
        ETS2LA_ESTIMATED_EDGES,
        fmt_opt_u64(tp_edges),
        ratio_str(tp_edges, ETS2LA_ESTIMATED_EDGES),
        ETS2LA_ESTIMATED_EDGES,
    )?;

    let tp_prefabs = stats.as_ref().and_then(|s| s.total_prefabs);
    writeln!(
        md,
        "| prefabs | {} | {} | {} | |",
        ETS2LA_PREFABS,
        fmt_opt_u64(tp_prefabs),
        ratio_str(tp_prefabs, ETS2LA_PREFABS),
    )?;

    let tp_signs = stats.as_ref().and_then(|s| s.total_signs);
    writeln!(
        md,
        "| signs | {} | {} | {} | |",
        ETS2LA_SIGNS,
        fmt_opt_u64(tp_signs),
        ratio_str(tp_signs, ETS2LA_SIGNS),
    )?;

    // item resolution rate vs route success rate
    let tp_success_rate = audit.as_ref().map(|a| {
        if a.total_pairs > 0 {
            a.cat4_success as f64 / a.total_pairs as f64
        } else {
            0.0
        }
    });
    let tp_rate_str = tp_success_rate
        .map(|r| format!("{:.4} ({:.2}%)", r, r * 100.0))
        .unwrap_or_else(|| "N/A".to_string());
    let ratio_rate = tp_success_rate
        .map(|r| format!("{:.3}", r / ETS2LA_ITEM_RESOLUTION_RATE))
        .unwrap_or_else(|| "N/A".to_string());
    writeln!(
        md,
        "| item_resolution_rate / route_success_rate | {:.4} ({:.2}%) | {} | {} | ETS2LA=item resolution; TP=city-pair A* success |",
        ETS2LA_ITEM_RESOLUTION_RATE,
        ETS2LA_ITEM_RESOLUTION_RATE * 100.0,
        tp_rate_str,
        ratio_rate,
    )?;
    writeln!(md)?;

    // --- Route audit summary ---
    writeln!(md, "## Routing Success Comparison")?;
    writeln!(md)?;
    if let Some(a) = &audit {
        let success_pct = if a.total_pairs > 0 {
            a.cat4_success as f64 / a.total_pairs as f64 * 100.0
        } else {
            0.0
        };
        writeln!(md, "| Metric | Value |")?;
        writeln!(md, "|--------|-------|")?;
        writeln!(md, "| Total city pairs | {} |", a.total_pairs)?;
        writeln!(
            md,
            "| CAT4_SUCCESS | {} ({success_pct:.2}%) |",
            a.cat4_success
        )?;
        writeln!(md, "| CAT3_NO_PATH | {} |", a.cat3_no_path)?;
        writeln!(md, "| CAT1_SNAP_* | {} |", a.cat1_snap)?;
        writeln!(md)?;
        writeln!(
            md,
            "ETS2LA item resolution rate: **{:.2}%**  \nTruckPilot route success rate: **{success_pct:.2}%**",
            ETS2LA_ITEM_RESOLUTION_RATE * 100.0
        )?;
    } else {
        writeln!(
            md,
            "_audit-csv not available — no routing comparison possible._"
        )?;
    }
    writeln!(md)?;

    // --- Hypothesis evaluation ---
    writeln!(md, "## Hypothesen-Bewertung")?;
    writeln!(md)?;

    // H1: Fehlende Edges
    {
        let (status, reason) = if let Some(edges) = tp_edges {
            let ratio = edges as f64 / ETS2LA_ESTIMATED_EDGES as f64;
            if ratio < 0.8 {
                (
                    Some(true),
                    format!(
                        "TruckPilot hat {} Edges vs. ETS2LA-Schätzung {}. Ratio {:.3} < 0.80 — signifikant weniger Edges.",
                        edges, ETS2LA_ESTIMATED_EDGES, ratio
                    ),
                )
            } else if ratio > 1.2 {
                (
                    Some(false),
                    format!(
                        "TruckPilot hat {} Edges vs. ETS2LA-Schätzung {}. Ratio {:.3} > 1.20 — mehr als erwartet, kein Mangel.",
                        edges, ETS2LA_ESTIMATED_EDGES, ratio
                    ),
                )
            } else {
                (
                    Some(false),
                    format!(
                        "TruckPilot hat {} Edges vs. ETS2LA-Schätzung {}. Ratio {:.3} im Bereich 0.80–1.20 — Edge-Menge plausibel.",
                        edges, ETS2LA_ESTIMATED_EDGES, ratio
                    ),
                )
            }
        } else {
            (
                None,
                "graph_stats.json nicht verfügbar — kein Vergleich möglich.".to_string(),
            )
        };
        writeln!(md, "### H1: Fehlende Edges")?;
        writeln!(md)?;
        writeln!(md, "**Status: {}**", hypothesis_status(status))?;
        writeln!(md)?;
        writeln!(md, "{reason}")?;
        writeln!(md)?;
    }

    // H2: Edge-Direction falsch (expect ~50:50 forward/backward)
    {
        let (status, reason) = if let Some(s) = &stats {
            let fwd = s
                .edge_type_distribution
                .get("forward")
                .copied()
                .unwrap_or(0);
            let bwd = s
                .edge_type_distribution
                .get("backward")
                .copied()
                .unwrap_or(0);
            let total_dir = fwd + bwd;
            if total_dir == 0 {
                (None, "Keine forward/backward Edges gefunden.".to_string())
            } else {
                let fwd_ratio = fwd as f64 / total_dir as f64;
                let deviation = (fwd_ratio - 0.5).abs();
                if deviation > 0.05 {
                    (
                        Some(true),
                        format!(
                            "Forward: {fwd} ({:.1}%), Backward: {bwd} ({:.1}%). Abweichung von 50:50 = {:.1}pp > 5pp — Richtungs-Imbalance vorhanden.",
                            fwd_ratio * 100.0,
                            (1.0 - fwd_ratio) * 100.0,
                            deviation * 100.0,
                        ),
                    )
                } else {
                    (
                        Some(false),
                        format!(
                            "Forward: {fwd} ({:.1}%), Backward: {bwd} ({:.1}%). Ratio {fwd_ratio:.3} — innerhalb 5pp von 50:50, kein systematischer Richtungsfehler.",
                            fwd_ratio * 100.0,
                            (1.0 - fwd_ratio) * 100.0,
                        ),
                    )
                }
            }
        } else {
            (None, "graph_stats.json nicht verfügbar.".to_string())
        };
        writeln!(md, "### H2: Edge-Direction falsch")?;
        writeln!(md)?;
        writeln!(md, "**Status: {}**", hypothesis_status(status))?;
        writeln!(md)?;
        writeln!(md, "{reason}")?;
        writeln!(md)?;
    }

    // H3: Graph in viele Inseln
    {
        let (status, reason) = if let Some(a) = &audit {
            let cat3_rate = if a.total_pairs > 0 {
                a.cat3_no_path as f64 / a.total_pairs as f64
            } else {
                0.0
            };
            if cat3_rate > 0.10 {
                (
                    Some(true),
                    format!(
                        "CAT3_NO_PATH: {} Paare ({:.1}%) > 10% Schwelle — Graph hat erhebliche Konnektivitätslücken (viele Inseln wahrscheinlich).",
                        a.cat3_no_path,
                        cat3_rate * 100.0,
                    ),
                )
            } else {
                (
                    Some(false),
                    format!(
                        "CAT3_NO_PATH: {} Paare ({:.1}%) — unter 10%-Schwelle, keine massiven Inseln erkennbar.",
                        a.cat3_no_path,
                        cat3_rate * 100.0,
                    ),
                )
            }
        } else {
            (
                None,
                "Kein route_audit.csv vorhanden. routing_islands.json prüfen (falls vorhanden) für SCC-Analyse.".to_string(),
            )
        };
        writeln!(md, "### H3: Graph in viele Inseln")?;
        writeln!(md)?;
        writeln!(md, "**Status: {}**", hypothesis_status(status))?;
        writeln!(md)?;
        writeln!(md, "{reason}")?;
        writeln!(md)?;
    }

    // H4: Prefab-Edges fehlen
    {
        let (status, reason) = if let Some(s) = &stats {
            let prefab_edges = s.edge_type_distribution.get("prefab").copied().unwrap_or(0);
            let total_edges = s.total_edges.unwrap_or(0);
            let prefab_pct = if total_edges > 0 {
                prefab_edges as f64 / total_edges as f64 * 100.0
            } else {
                0.0
            };
            let prefab_coverage = if let (Some(pt), Some(pe)) =
                (s.prefab_nodes_total, s.prefab_nodes_also_in_edges)
            {
                if pt > 0 {
                    Some(pe as f64 / pt as f64)
                } else {
                    None
                }
            } else {
                None
            };
            let cov_str = prefab_coverage
                .map(|c| format!("{:.1}%", c * 100.0))
                .unwrap_or_else(|| "N/A".to_string());

            if prefab_pct < 1.0 {
                (
                    Some(true),
                    format!(
                        "Prefab-Edges: {prefab_edges} ({prefab_pct:.2}% aller Edges) — sehr gering. Prefab-Node-Coverage: {cov_str}. Prefab-Edges sind unterrepräsentiert.",
                    ),
                )
            } else {
                (
                    Some(false),
                    format!(
                        "Prefab-Edges: {prefab_edges} ({prefab_pct:.2}% aller Edges). Prefab-Node-Coverage: {cov_str}. Kein signifikanter Mangel erkennbar.",
                    ),
                )
            }
        } else {
            (None, "graph_stats.json nicht verfügbar.".to_string())
        };
        writeln!(md, "### H4: Prefab-Edges fehlen")?;
        writeln!(md)?;
        writeln!(md, "**Status: {}**", hypothesis_status(status))?;
        writeln!(md)?;
        writeln!(md, "{reason}")?;
        writeln!(md)?;
    }

    // H5: Sektor-Grenzen (cross_sector edges)
    {
        let (status, reason) = if let Some(s) = &stats {
            let cross_sector_total: u64 = s
                .raw_direction_distribution
                .iter()
                .filter(|(k, _)| k.contains("cross_sector") || k.contains("crosssector"))
                .map(|(_, v)| v)
                .sum();
            let total_edges = s.total_edges.unwrap_or(0);

            // Also check for "other" bucket which might hide cross-sector entries
            let other_count = s.edge_type_distribution.get("other").copied().unwrap_or(0);

            if cross_sector_total > 0 {
                let pct = cross_sector_total as f64 / total_edges as f64 * 100.0;
                if pct < 0.5 {
                    (
                        Some(true),
                        format!(
                            "Cross-Sector-Edges in raw_direction_distribution: {cross_sector_total} ({pct:.3}% aller Edges) — sehr gering. Sektor-Grenzen könnten Connectivity-Probleme verursachen.",
                        ),
                    )
                } else {
                    (
                        Some(false),
                        format!(
                            "Cross-Sector-Edges: {cross_sector_total} ({pct:.3}%). Ausreichend vorhanden — kein signifikantes Sektor-Grenz-Problem.",
                        ),
                    )
                }
            } else {
                let direction_values: Vec<String> = s
                    .raw_direction_distribution
                    .keys()
                    .take(10)
                    .map(|k| format!("`{k}`"))
                    .collect();
                (
                    None,
                    format!(
                        "Kein `cross_sector`-Eintrag in raw_direction_distribution gefunden. \"other\"-Bucket: {other_count} Edges. Bekannte Richtungsstrings (erste 10): {}. Sektor-Grenzen-Analyse nicht möglich ohne Rohdaten.",
                        direction_values.join(", "),
                    ),
                )
            }
        } else {
            (None, "graph_stats.json nicht verfügbar.".to_string())
        };
        writeln!(md, "### H5: Sektor-Grenzen")?;
        writeln!(md)?;
        writeln!(md, "**Status: {}**", hypothesis_status(status))?;
        writeln!(md)?;
        writeln!(md, "{reason}")?;
        writeln!(md)?;
    }

    // --- Phase 5.26 recommendation ---
    writeln!(md, "## Empfehlung Phase 5.26")?;
    writeln!(md)?;

    let rec = generate_recommendation(&stats, &audit);
    writeln!(md, "{rec}")?;

    eprintln!("[compare-with-ets2la] wrote {}", md_path.display());

    Ok(())
}

fn generate_recommendation(stats: &Option<StatsData>, audit: &Option<AuditData>) -> String {
    let cat3_rate = audit.as_ref().map(|a| {
        if a.total_pairs > 0 {
            a.cat3_no_path as f64 / a.total_pairs as f64
        } else {
            0.0
        }
    });
    let cat1_rate = audit.as_ref().map(|a| {
        if a.total_pairs > 0 {
            a.cat1_snap as f64 / a.total_pairs as f64
        } else {
            0.0
        }
    });
    let prefab_edge_pct = stats.as_ref().map(|s| {
        let pe = s.edge_type_distribution.get("prefab").copied().unwrap_or(0);
        let te = s.total_edges.unwrap_or(1);
        pe as f64 / te as f64
    });
    let edge_ratio = stats
        .as_ref()
        .and_then(|s| s.total_edges)
        .map(|e| e as f64 / ETS2LA_ESTIMATED_EDGES as f64);

    let mut reasons: Vec<&str> = Vec::new();

    if cat1_rate.map(|r| r > 0.15).unwrap_or(false) {
        reasons.push("H4");
    }
    if cat3_rate.map(|r| r > 0.10).unwrap_or(false) {
        reasons.push("H3");
    }
    if edge_ratio.map(|r| r < 0.8).unwrap_or(false) {
        reasons.push("H1");
    }
    if prefab_edge_pct.map(|p| p < 0.01).unwrap_or(false) {
        reasons.push("H4/Prefab");
    }

    if reasons.is_empty() {
        "Kein dominantes Problem aus den Metriken erkennbar. Empfehlung: \
         ProMods-Support (Phase 6.4) und Routing-Feintuning (A*-Kostenfunktion, \
         Speed-Limit-Gewichtung). Rohdaten liefern keine Anomalie-Schwerpunkte."
            .to_string()
    } else {
        format!(
            "Stärkste Problemhypothesen nach Metriken: **{}**.\n\n\
             Phase-5.26-Fokus:\n\
             - Falls H4/H3 dominant: Prefab-Connectivity-Audit + SCC-Analyse (routing-islands) \
             für genaue Island-Karte.\n\
             - Falls H1 dominant: Graph-Builder-Audit — fehlende Road-Segments tracen \
             via road-drop-audit.\n\
             - Falls H4/Prefab dominant: PPD-Check und Prefab-Edge-Generierung debuggen \
             (h4c_ppd_check, h4c_prefab_roi).",
            reasons.join(", ")
        )
    }
}

fn chrono_lite() -> String {
    use std::time::{SystemTime, UNIX_EPOCH};
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();
    let s = secs % 60;
    let m = (secs / 60) % 60;
    let h = (secs / 3600) % 24;
    let days = secs / 86400;
    let year = 1970 + days / 365;
    let day_of_year = days % 365;
    let month = day_of_year / 30 + 1;
    let day = day_of_year % 30 + 1;
    format!("{year}-{month:02}-{day:02} {h:02}:{m:02}:{s:02} UTC")
}
