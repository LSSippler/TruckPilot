//! TruckPilot CLI — ETS2 Map Parser & Autopilot
//!
//! Parses ETS2 map data, builds a road network graph, exports compat formats,
//! generates quality reports, and plans routes.

use std::collections::HashMap;
use std::fs::File;
use std::io::BufReader;
use std::path::{Path, PathBuf};
use std::time::Instant;

use clap::Parser;
use truckpilot::autopilot_loop;
use truckpilot::config::TruckPilotConfig;
use truckpilot::ets2_parser;
use truckpilot::graph_schema::GraphData;
use truckpilot::pipeline::{self, CliOptions};

/// Build a [`ModLoadOrder`] from optional CLI flags.
///
/// Priority of inputs:
/// 1. If `mod_order_file` is set, load that JSON.
/// 2. Else if `mod_dir` is set, auto-discover all `.scs` files there.
/// 3. Else return an empty order (caller falls back to vanilla).
fn build_mod_load_order(
    mod_dir: Option<&Path>,
    mod_order_file: Option<&Path>,
    game_path: &Path,
    verbose: bool,
) -> Result<truckpilot::ets2_parser::ModLoadOrder, String> {
    use truckpilot::ets2_parser::ModLoadOrder;

    if let Some(json_path) = mod_order_file {
        if verbose {
            eprintln!("Loading mod order from {}", json_path.display());
        }
        return ModLoadOrder::from_json_file(json_path, mod_dir, Some(game_path))
            .map_err(|e| format!("{e}"));
    }

    if let Some(dir) = mod_dir {
        if verbose {
            eprintln!("Auto-discovering mods in {}", dir.display());
        }
        return ModLoadOrder::from_directory(dir).map_err(|e| format!("{e}"));
    }

    Ok(ModLoadOrder::default())
}

/// Parse a u64 from decimal or hexadecimal (0x-prefixed) string.
fn parse_u64_hex_or_dec(s: &str) -> Result<u64, String> {
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u64::from_str_radix(hex, 16).map_err(|e| format!("invalid hex u64 '{s}': {e}"))
    } else {
        s.parse::<u64>()
            .map_err(|e| format!("invalid u64 '{s}': {e}"))
    }
}

#[derive(Parser, Debug)]
#[command(name = "truckpilot", version, about)]
struct Cli {
    /// Path to ETS2 installation directory (e.g. C:/Program Files/.../Euro Truck Simulator 2).
    #[arg(long)]
    ets2_dir: Option<String>,

    /// Directory with extracted HashFS sector files (`*.base`).
    #[arg(long)]
    hashfs_sectors: Option<String>,

    /// Path to a prebuilt graph.json (takes priority over all map sources).
    #[arg(long, value_name = "PATH")]
    graph_json: Option<PathBuf>,

    /// Path to an exported text-format map sector file (from edit_save_text).
    #[arg(long)]
    text_map_file: Option<String>,

    /// Path to SCS packer executable.
    #[arg(long)]
    scs_packer: Option<String>,

    #[arg(long, default_value_t = true)]
    write_graph: bool,

    #[arg(long, default_value_t = false)]
    compat_export: bool,

    #[arg(long, default_value_t = false)]
    quality_report: bool,

    #[arg(long, default_value_t = false)]
    performance_compare: bool,

    #[arg(long, default_value = "self_route")]
    routing_mode: String,

    #[arg(long, default_value_t = false)]
    prefer_speed: bool,

    #[arg(long)]
    cost_mode: Option<String>,

    #[arg(short, long, default_value_t = false)]
    verbose: bool,

    #[arg(long, value_parser = parse_u64_hex_or_dec)]
    start: Option<u64>,

    #[arg(long, value_parser = parse_u64_hex_or_dec)]
    goal: Option<u64>,

    #[arg(long)]
    telemetry_server: Option<String>,

    /// Optional path to truckpilot config TOML.
    #[arg(long, value_name = "PATH")]
    config: Option<PathBuf>,

    #[arg(long, default_value_t = false)]
    telemetry_disable: bool,

    /// vJoy device ID (1-16). When set, outputs through vJoy instead of console.
    #[arg(long)]
    vjoy_device: Option<u32>,

    /// Directory containing user map mods (`*.scs`). When `--enable-mods`
    /// is set the directory is scanned and every archive is layered on top
    /// of the base game in alphabetical order (or the order given by
    /// `--mod-order`).
    #[arg(long, value_name = "PATH")]
    mod_dir: Option<PathBuf>,

    /// Path to a JSON file with a custom mod load order
    /// (see `mod_order.json`).
    #[arg(long, value_name = "FILE")]
    mod_order: Option<PathBuf>,

    /// Enable mod support. Without this flag `--mod-dir` and `--mod-order`
    /// are ignored, preserving legacy single-archive behaviour.
    #[arg(long, default_value_t = false)]
    enable_mods: bool,
}

fn main() {
    let cli = Cli::parse();
    let config = if let Some(ref p) = cli.config {
        match TruckPilotConfig::try_load_from_file(&p.to_string_lossy()) {
            Ok(cfg) => cfg,
            Err(e) => {
                eprintln!("Failed to load config '{}': {e}", p.display());
                std::process::exit(1);
            }
        }
    } else {
        TruckPilotConfig::load()
    };
    let telemetry_server = cli
        .telemetry_server
        .clone()
        .unwrap_or_else(|| config.telemetry.url.clone());
    let telemetry_disable = cli.telemetry_disable || config.telemetry.disabled;

    if let Some(ref graph_path) = cli.graph_json {
        let load_start = Instant::now();
        let file = match File::open(graph_path) {
            Ok(f) => f,
            Err(e) => {
                eprintln!("Failed to open graph JSON {}: {}", graph_path.display(), e);
                std::process::exit(1);
            }
        };
        let reader = BufReader::new(file);
        let graph: GraphData = match serde_json::from_reader(reader) {
            Ok(g) => g,
            Err(e) => {
                eprintln!("Failed to parse graph JSON {}: {}", graph_path.display(), e);
                std::process::exit(1);
            }
        };

        if graph.nodes.is_empty() || graph.edges.is_empty() {
            eprintln!(
                "Graph JSON {} is invalid: nodes={} edges={}",
                graph_path.display(),
                graph.nodes.len(),
                graph.edges.len()
            );
            std::process::exit(1);
        }

        if cli.verbose {
            let ms = load_start.elapsed().as_secs_f64() * 1000.0;
            println!("TruckPilot v{}", env!("CARGO_PKG_VERSION"));
            println!(
                "Loaded graph JSON: {} nodes, {} edges in {:.2} ms",
                graph.nodes.len(),
                graph.edges.len(),
                ms
            );
        }

        if let (Some(start), Some(goal)) = (cli.start, cli.goal) {
            let active_cost_mode = cli
                .cost_mode
                .clone()
                .unwrap_or_else(|| config.routing.cost_mode.clone());
            let cost_mode = match active_cost_mode.as_str() {
                "eta" => truckpilot::autopilot::CostMode::Eta,
                _ => truckpilot::autopilot::CostMode::Distance,
            };
            let route_config = truckpilot::autopilot::RouteConfig {
                prefer_speed: cli.prefer_speed || config.routing.prefer_speed,
                cost_mode,
            };

            let route =
                truckpilot::autopilot::plan_route_on_graph(&graph, start, goal, &route_config);

            match route {
                Some(result) => {
                    println!(
                        "Route found: {} nodes, cost={:.2}, validated={}, time={:.2}ms",
                        result.path.len(),
                        result.total_cost,
                        result.validated,
                        result.planning_time_ms
                    );
                    if cli.verbose {
                        println!("  Path: {:?}", result.path);
                        println!("  Edges examined: {}", result.edges_examined);
                        println!("  Nodes expanded: {}", result.nodes_expanded);
                    }

                    if !telemetry_disable {
                        println!();
                        println!("Starting autopilot loop. Telemetry: {}", telemetry_server);

                        let positions: HashMap<u64, (f64, f64)> =
                            graph.nodes.iter().map(|n| (n.uid, (n.x, n.z))).collect();

                        #[cfg(windows)]
                        let output: Box<
                            dyn truckpilot::vjoy::ControlOutput,
                        > = {
                            let selected_device = cli.vjoy_device.or_else(|| {
                                truckpilot::vjoy::enumerate_vjoy_devices()
                                    .into_iter()
                                    .next()
                            });
                            match selected_device
                                .and_then(truckpilot::vjoy::VJoyOutput::try_acquire)
                            {
                                Some(vjoy) => Box::new(vjoy),
                                None => {
                                    eprintln!("No free vJoy device found, falling back to console output.");
                                    Box::new(truckpilot::vjoy::ConsoleOutput)
                                }
                            }
                        };
                        #[cfg(not(windows))]
                        let output: Box<
                            dyn truckpilot::vjoy::ControlOutput,
                        > = Box::new(truckpilot::vjoy::ConsoleOutput);

                        autopilot_loop::run_autopilot_loop(
                            result.path,
                            positions,
                            &telemetry_server,
                            output,
                            &config,
                        );
                    } else if cli.verbose {
                        println!("Telemetry disabled — autopilot loop skipped.");
                    }
                }
                None => {
                    eprintln!("No route found from {start} to {goal}");
                }
            }
        } else if cli.verbose {
            println!("No --start/--goal specified — route planning skipped.");
        }

        return;
    }

    // Determine map data source.
    let mut map = None;

    if let Some(ref dir) = cli.hashfs_sectors {
        let path = Path::new(dir);
        if cli.verbose {
            eprintln!("Parsing HashFS sectors from: {}", path.display());
        }
        match ets2_parser::parse_hashfs_sectors_dir(path) {
            Ok(m) => {
                if cli.verbose {
                    eprintln!(
                        "Parsed: {} nodes, {} roads, {} prefabs",
                        m.nodes.len(),
                        m.roads.len(),
                        m.prefabs.len()
                    );
                }
                map = Some(m);
            }
            Err(e) => {
                eprintln!("Failed to parse HashFS sectors: {e}");
                std::process::exit(1);
            }
        }
    }

    if map.is_none() && cli.enable_mods {
        if let Some(ref dir) = cli.ets2_dir {
            let game_path = Path::new(dir);
            let order = match build_mod_load_order(
                cli.mod_dir.as_deref(),
                cli.mod_order.as_deref(),
                game_path,
                cli.verbose,
            ) {
                Ok(o) => o,
                Err(e) => {
                    eprintln!("Failed to build mod load order: {e}");
                    std::process::exit(1);
                }
            };
            if cli.verbose {
                eprintln!(
                    "Mod support enabled: {} mod(s), {} base-game archive(s)",
                    order.descriptors.iter().filter(|d| d.is_enabled).count(),
                    order.base_game_paths.len()
                );
            }
            match ets2_parser::parse_ets2_map_with_mods(game_path, &order) {
                Ok(m) => {
                    if cli.verbose {
                        eprintln!(
                            "Parsed (with mods): {} nodes, {} roads, {} prefabs",
                            m.nodes.len(),
                            m.roads.len(),
                            m.prefabs.len()
                        );
                    }
                    map = Some(m);
                }
                Err(e) => {
                    eprintln!("Failed to parse ETS2 map with mods: {e}");
                    std::process::exit(1);
                }
            }
        }
    }

    if map.is_none() {
        if let Some(ref dir) = cli.ets2_dir {
            let path = Path::new(dir);
            if cli.verbose {
                eprintln!("Parsing ETS2 map from: {}", path.display());
            }
            match ets2_parser::parse_ets2_map(path) {
                Ok(m) => {
                    if cli.verbose {
                        eprintln!(
                            "Parsed: {} nodes, {} roads, {} prefabs",
                            m.nodes.len(),
                            m.roads.len(),
                            m.prefabs.len()
                        );
                    }
                    map = Some(m);
                }
                Err(e) => {
                    eprintln!("Failed to parse ETS2 map: {e}");
                    std::process::exit(1);
                }
            }
        }
    }

    if map.is_none() {
        if let Some(ref file) = cli.text_map_file {
            let path = Path::new(file);
            if cli.verbose {
                eprintln!("Loading text map from: {}", path.display());
            }
            match ets2_parser::parse_text_map_file(path) {
                Ok(m) => {
                    if cli.verbose {
                        eprintln!(
                            "Loaded: {} nodes, {} roads, {} prefabs",
                            m.nodes.len(),
                            m.roads.len(),
                            m.prefabs.len()
                        );
                    }
                    map = Some(m);
                }
                Err(e) => {
                    eprintln!("Failed to parse text map: {e}");
                    std::process::exit(1);
                }
            }
        }
    }

    let map = map.unwrap_or_else(|| {
        if cli.verbose {
            eprintln!("No map source specified — using built-in test fixture.");
        } else {
            eprintln!("Falling back to test fixture.");
        }
        pipeline::build_test_map()
    });

    let skip_pipeline_route = !cli.telemetry_disable && cli.start.is_some() && cli.goal.is_some();

    let opts = CliOptions {
        ets2_dir: cli.ets2_dir,
        hashfs_sectors: cli.hashfs_sectors,
        scs_packer: cli.scs_packer,
        write_graph: cli.write_graph,
        compat_export: cli.compat_export,
        quality_report: cli.quality_report,
        performance_compare: cli.performance_compare,
        routing_mode: cli.routing_mode,
        prefer_speed: cli.prefer_speed,
        cost_mode: cli
            .cost_mode
            .clone()
            .unwrap_or_else(|| config.routing.cost_mode.clone()),
        verbose: cli.verbose,
        start_uid: if skip_pipeline_route { None } else { cli.start },
        goal_uid: if skip_pipeline_route { None } else { cli.goal },
    };

    #[cfg_attr(not(windows), allow(unused_variables))]
    let vjoy_device = cli.vjoy_device;
    let cost_mode_cli = cli
        .cost_mode
        .clone()
        .unwrap_or_else(|| config.routing.cost_mode.clone());
    let prefer_speed_cli = cli.prefer_speed || config.routing.prefer_speed;
    let start_cli = cli.start;
    let goal_cli = cli.goal;

    if opts.verbose {
        println!("TruckPilot v{}", env!("CARGO_PKG_VERSION"));
        println!(
            "Nodes: {}, Roads: {}, Prefabs: {}",
            map.nodes.len(),
            map.roads.len(),
            map.prefabs.len()
        );
    }

    // Pipeline.
    match pipeline::run_pipeline(&map, &opts) {
        Ok(()) => {}
        Err(e) => {
            eprintln!("Pipeline error: {e}");
            std::process::exit(1);
        }
    }

    // Telemetry loop.
    if !telemetry_disable {
        if let (Some(start), Some(goal)) = (start_cli, goal_cli) {
            let graph = match truckpilot::graph_export::build_graph(&map) {
                Ok(g) => g,
                Err(e) => {
                    eprintln!("Graph build error: {e}");
                    std::process::exit(1);
                }
            };

            let cost_mode = match cost_mode_cli.as_str() {
                "eta" => truckpilot::autopilot::CostMode::Eta,
                _ => truckpilot::autopilot::CostMode::Distance,
            };
            let route_config = truckpilot::autopilot::RouteConfig {
                prefer_speed: prefer_speed_cli,
                cost_mode,
            };

            let route =
                truckpilot::autopilot::plan_route(&map, Some(&graph), start, goal, &route_config);

            match route {
                Some(result) => {
                    println!();
                    println!("Starting autopilot loop. Telemetry: {}", telemetry_server);
                    let positions: HashMap<u64, (f64, f64)> =
                        graph.nodes.iter().map(|n| (n.uid, (n.x, n.z))).collect();

                    #[cfg(windows)]
                    let output: Box<dyn truckpilot::vjoy::ControlOutput> = {
                        let selected_device = vjoy_device.or_else(|| {
                            truckpilot::vjoy::enumerate_vjoy_devices()
                                .into_iter()
                                .next()
                        });
                        match selected_device.and_then(truckpilot::vjoy::VJoyOutput::try_acquire) {
                            Some(vjoy) => Box::new(vjoy),
                            None => {
                                eprintln!(
                                    "No free vJoy device found, falling back to console output."
                                );
                                Box::new(truckpilot::vjoy::ConsoleOutput)
                            }
                        }
                    };
                    #[cfg(not(windows))]
                    let output: Box<dyn truckpilot::vjoy::ControlOutput> =
                        Box::new(truckpilot::vjoy::ConsoleOutput);

                    autopilot_loop::run_autopilot_loop(
                        result.path,
                        positions,
                        &telemetry_server,
                        output,
                        &config,
                    );
                }
                None => {
                    eprintln!("No route found from {start} to {goal} — telemetry loop skipped.");
                }
            }
        } else if opts.verbose {
            println!("No --start/--goal specified — telemetry loop skipped.");
        }
    }
}
