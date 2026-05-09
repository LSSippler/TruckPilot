//! TruckPilot Core — plugin host, map parser, route planner, autopilot.

use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use tokio::sync::Mutex;

use tokio::sync::broadcast;

use tracing::{info, warn};
use truckpilot_ipc_protocol::{CoreMessage, TelemetrySnapshot};
use truckpilot_plugin_api::{ControlOutput, SharedBlackboard, Telemetry};

mod ipc;
mod plugin_manager;

use plugin_manager::PluginManager;

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

enum Command {
    Daemon,
    ParseMap {
        ets2_dir: PathBuf,
        mods_dir: Option<PathBuf>,
    },
    Route {
        from: u64,
        to: u64,
    },
    Autopilot {
        from: u64,
        to: u64,
        vjoy_device: u32,
    },
}

fn parse_args() -> Command {
    let args: Vec<String> = std::env::args().collect();

    if args.len() < 2 {
        return Command::Daemon;
    }

    match args[1].as_str() {
        "daemon" => Command::Daemon,

        "parse-map" => {
            let ets2_dir = flag_value(&args, "--ets2-dir")
                .map(PathBuf::from)
                .unwrap_or_else(default_ets2_dir);
            let mods_dir = flag_value(&args, "--mods-dir").map(PathBuf::from);
            Command::ParseMap { ets2_dir, mods_dir }
        }

        "route" => {
            let from = flag_value(&args, "--from")
                .and_then(|s| parse_uid(&s))
                .unwrap_or_else(|| {
                    eprintln!("--from <uid> required");
                    std::process::exit(2);
                });
            let to = flag_value(&args, "--to")
                .and_then(|s| parse_uid(&s))
                .unwrap_or_else(|| {
                    eprintln!("--to <uid> required");
                    std::process::exit(2);
                });
            Command::Route { from, to }
        }

        "autopilot" => {
            let from = flag_value(&args, "--from")
                .and_then(|s| parse_uid(&s))
                .unwrap_or_else(|| {
                    eprintln!("--from <uid> required");
                    std::process::exit(2);
                });
            let to = flag_value(&args, "--to")
                .and_then(|s| parse_uid(&s))
                .unwrap_or_else(|| {
                    eprintln!("--to <uid> required");
                    std::process::exit(2);
                });
            let vjoy_device = flag_value(&args, "--vjoy-device")
                .and_then(|s| s.parse().ok())
                .unwrap_or(1);
            Command::Autopilot {
                from,
                to,
                vjoy_device,
            }
        }

        "--help" | "-h" | "help" => {
            print_help();
            std::process::exit(0);
        }

        other => {
            eprintln!("Unknown subcommand: {other}");
            eprintln!("Run with --help for usage.");
            std::process::exit(2);
        }
    }
}

fn flag_value(args: &[String], flag: &str) -> Option<String> {
    args.windows(2).find(|w| w[0] == flag).map(|w| w[1].clone())
}

fn parse_uid(s: &str) -> Option<u64> {
    if let Some(hex) = s.strip_prefix("0x").or_else(|| s.strip_prefix("0X")) {
        u64::from_str_radix(hex, 16).ok()
    } else {
        s.parse().ok()
    }
}

fn default_ets2_dir() -> PathBuf {
    #[cfg(windows)]
    {
        PathBuf::from(r"C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2")
    }
    #[cfg(not(windows))]
    {
        PathBuf::from("/opt/ets2")
    }
}

fn default_mods_dir() -> PathBuf {
    let base = dirs::document_dir()
        .or_else(dirs::home_dir)
        .unwrap_or_default();
    base.join("Euro Truck Simulator 2/mod")
}

fn print_help() {
    println!("truckpilot-core — TruckPilot daemon and tools");
    println!();
    println!("USAGE:");
    println!("  truckpilot-core [SUBCOMMAND] [OPTIONS]");
    println!();
    println!("SUBCOMMANDS:");
    println!("  daemon                    Run plugin loop (default)");
    println!("  parse-map                 Parse ETS2 map → graph.json");
    println!("  route                     Plan route between two nodes");
    println!("  autopilot                 Run full autopilot loop");
    println!();
    println!("OPTIONS (parse-map):");
    println!("  --ets2-dir <path>         ETS2 installation directory");
    println!("  --mods-dir <path>         ETS2 mods directory (optional)");
    println!();
    println!("OPTIONS (route / autopilot):");
    println!("  --from <uid>              Start node UID (hex or decimal)");
    println!("  --to   <uid>              Goal node UID (hex or decimal)");
    println!("  --vjoy-device <n>         vJoy device number (default: 1)");
}

// ---------------------------------------------------------------------------
// Subcommand implementations
// ---------------------------------------------------------------------------

fn cmd_parse_map(ets2_dir: &std::path::Path, mods_dir: Option<PathBuf>) {
    println!("Parsing ETS2 map from: {}", ets2_dir.display());
    let final_mods_dir = mods_dir.unwrap_or_else(default_mods_dir);

    if !ets2_dir.exists() {
        eprintln!("ERROR: ETS2 directory not found: {}", ets2_dir.display());
        eprintln!("Use --ets2-dir to specify the correct path.");
        std::process::exit(1);
    }

    let start = Instant::now();

    let order =
        match truckpilot_map_parser::ModLoadOrder::from_directories(ets2_dir, &final_mods_dir) {
            Ok(o) => o,
            Err(e) => {
                eprintln!("Failed to read game directories: {e}");
                std::process::exit(1);
            }
        };

    match truckpilot_map_parser::load_and_build(&order, None) {
        Ok(graph) => {
            let elapsed = start.elapsed();
            println!();
            println!("=== Map Parse Result ===");
            println!("  Nodes  : {}", graph.stats.node_count);
            println!("  Edges  : {}", graph.stats.edge_count);
            println!("  Prefabs: {}", graph.stats.prefab_count);
            println!("  Signs  : {}", graph.stats.sign_count);
            println!("  Time   : {:.1}s", elapsed.as_secs_f64());

            // Save graph.json
            match serde_json::to_string_pretty(&graph) {
                Ok(json) => {
                    if let Err(e) = std::fs::write("graph.json", &json) {
                        eprintln!("Failed to write graph.json: {e}");
                    } else {
                        println!("  Saved  : graph.json ({} bytes)", json.len());
                    }
                }
                Err(e) => eprintln!("Failed to serialize graph: {e}"),
            }

            // Print a few sample node UIDs so the user can use them for routing
            println!();
            println!("=== Sample Node UIDs (use for --from / --to) ===");
            for node in graph.nodes.iter().take(5) {
                println!("  0x{:016X}  ({:.0}, {:.0})", node.uid, node.x, node.z);
            }
        }
        Err(e) => {
            eprintln!("Map parse failed: {e}");
            std::process::exit(1);
        }
    }
}

fn cmd_route(from: u64, to: u64) {
    println!("Planning route: 0x{from:016X} → 0x{to:016X}");

    let graph = load_graph_or_exit();

    let start = Instant::now();
    match plan_route_on_graph(&graph, from, to) {
        Some(path) => {
            let elapsed = start.elapsed();
            println!();
            println!("=== Route Found ===");
            println!("  Nodes  : {}", path.len());
            println!("  Time   : {:.2}ms", elapsed.as_secs_f64() * 1000.0);
            println!("  First 5 waypoints:");
            for uid in path.iter().take(5) {
                if let Some(node) = graph.nodes.iter().find(|n| n.uid == *uid) {
                    println!("    0x{:016X}  ({:.0}, {:.0})", uid, node.x, node.z);
                }
            }
        }
        None => {
            eprintln!("No route found between 0x{from:016X} and 0x{to:016X}");
            eprintln!("Check that both UIDs exist in graph.json");
            std::process::exit(1);
        }
    }
}

fn cmd_autopilot(from: u64, to: u64, _vjoy_device: u32) {
    println!("Starting autopilot: 0x{from:016X} → 0x{to:016X}");

    let graph = load_graph_or_exit();

    let path = match plan_route_on_graph(&graph, from, to) {
        Some(p) => p,
        None => {
            eprintln!("No route found. Run parse-map first and check node UIDs.");
            std::process::exit(1);
        }
    };

    println!("Route: {} waypoints", path.len());
    println!("Autopilot running — press Ctrl+C to stop");
    println!();

    // Build position lookup
    let positions: std::collections::HashMap<u64, (f64, f64)> =
        graph.nodes.iter().map(|n| (n.uid, (n.x, n.z))).collect();

    let mut waypoint_idx = 0usize;
    let mut last_log = Instant::now();

    loop {
        let telemetry = truckpilot_telemetry::read_telemetry();

        let Some(t) = telemetry else {
            warn!("No telemetry — waiting for ETS2 + DLL...");
            std::thread::sleep(Duration::from_millis(500));
            continue;
        };

        // Advance waypoint when close enough
        if waypoint_idx < path.len() {
            if let Some(&(wx, wz)) = positions.get(&path[waypoint_idx]) {
                let dx = wx - t.position[0];
                let dz = wz - t.position[2];
                let dist = (dx * dx + dz * dz).sqrt();
                if dist < 15.0 {
                    waypoint_idx += 1;
                }
            }
        }

        if waypoint_idx >= path.len() {
            println!("Destination reached!");
            break;
        }

        // Compute heading error to next waypoint
        let target_uid = path[waypoint_idx];
        let steering = if let Some(&(tx, tz)) = positions.get(&target_uid) {
            let dx = tx - t.position[0];
            let dz = tz - t.position[2];
            let target_heading = dz.atan2(dx);
            let error = angle_diff(target_heading, t.heading);
            (error * 0.5).clamp(-1.0, 1.0)
        } else {
            0.0
        };

        // Simple speed control: 60 km/h target
        let target_ms = 16.7_f64; // 60 km/h
        let throttle = if t.speed_ms < target_ms - 1.0 {
            0.4
        } else {
            0.0
        };
        let brake = if t.speed_ms > target_ms + 2.0 {
            0.3
        } else {
            0.0
        };

        // Log every second
        if last_log.elapsed() >= Duration::from_secs(1) {
            println!(
                "  wp={}/{} speed={:.1}km/h steer={:.2} thr={:.2} brk={:.2}",
                waypoint_idx,
                path.len(),
                t.speed_ms * 3.6,
                steering,
                throttle,
                brake,
            );
            last_log = Instant::now();
        }

        // TODO: send to vJoy (vjoy-output plugin handles this in full daemon mode)
        // For now: print controls so you can verify the logic is correct
        let _ = (steering, throttle, brake);

        std::thread::sleep(Duration::from_millis(50)); // 20 Hz
    }
}

/// Minimal A* on the graph — reuses the same logic as the router plugin.
fn plan_route_on_graph(
    graph: &truckpilot_map_parser::graph::MapGraph,
    start: u64,
    goal: u64,
) -> Option<Vec<u64>> {
    use std::cmp::Reverse;
    use std::collections::{BinaryHeap, HashMap};

    let positions: HashMap<u64, (f64, f64)> =
        graph.nodes.iter().map(|n| (n.uid, (n.x, n.z))).collect();

    if !positions.contains_key(&start) {
        eprintln!("Start node 0x{start:016X} not found in graph");
        return None;
    }
    if !positions.contains_key(&goal) {
        eprintln!("Goal node 0x{goal:016X} not found in graph");
        return None;
    }

    let &(gx, gz) = positions.get(&goal).unwrap();

    let mut dist: HashMap<u64, f64> = HashMap::new();
    let mut came_from: HashMap<u64, u64> = HashMap::new();
    let mut heap: BinaryHeap<Reverse<(u64, u64)>> = BinaryHeap::new(); // (f*1000, uid)

    dist.insert(start, 0.0);
    heap.push(Reverse((0, start)));

    // Build adjacency from edges
    let mut adj: HashMap<u64, Vec<(u64, f64)>> = HashMap::new();
    for edge in &graph.edges {
        adj.entry(edge.from)
            .or_default()
            .push((edge.to, edge.distance_m));
    }

    while let Some(Reverse((_, uid))) = heap.pop() {
        if uid == goal {
            // Reconstruct path
            let mut path = vec![goal];
            let mut cur = goal;
            while let Some(&prev) = came_from.get(&cur) {
                path.push(prev);
                cur = prev;
                if cur == start {
                    break;
                }
            }
            path.reverse();
            return Some(path);
        }

        let cur_dist = *dist.get(&uid).unwrap_or(&f64::MAX);

        for &(next, edge_dist) in adj.get(&uid).unwrap_or(&vec![]) {
            let new_dist = cur_dist + edge_dist;
            if new_dist < *dist.get(&next).unwrap_or(&f64::MAX) {
                dist.insert(next, new_dist);
                came_from.insert(next, uid);
                let &(nx, nz) = positions.get(&next).unwrap_or(&(0.0, 0.0));
                let h = ((nx - gx).powi(2) + (nz - gz).powi(2)).sqrt();
                let f = ((new_dist + h) * 1000.0) as u64;
                heap.push(Reverse((f, next)));
            }
        }
    }

    None
}

fn load_graph_or_exit() -> truckpilot_map_parser::graph::MapGraph {
    let path = PathBuf::from("graph.json");
    if !path.exists() {
        eprintln!("graph.json not found.");
        eprintln!("Run first: truckpilot-core parse-map --ets2-dir <path>");
        std::process::exit(1);
    }
    let json = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        eprintln!("Cannot read graph.json: {e}");
        std::process::exit(1);
    });
    serde_json::from_str(&json).unwrap_or_else(|e| {
        eprintln!("Cannot parse graph.json: {e}");
        std::process::exit(1);
    })
}

fn angle_diff(a: f64, b: f64) -> f64 {
    let mut d = a - b;
    while d > std::f64::consts::PI {
        d -= 2.0 * std::f64::consts::PI;
    }
    while d < -std::f64::consts::PI {
        d += 2.0 * std::f64::consts::PI;
    }
    d
}

// ---------------------------------------------------------------------------
// Daemon mode
// ---------------------------------------------------------------------------

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt::init();

    let cmd = parse_args();

    match cmd {
        Command::ParseMap { ets2_dir, mods_dir } => {
            cmd_parse_map(&ets2_dir, mods_dir); // PathBuf deref to Path
        }
        Command::Route { from, to } => {
            cmd_route(from, to);
        }
        Command::Autopilot {
            from,
            to,
            vjoy_device,
        } => {
            cmd_autopilot(from, to, vjoy_device);
        }
        Command::Daemon => {
            run_daemon().await;
        }
    }
}

async fn run_daemon() {
    info!("TruckPilot Core — daemon mode");

    let plugin_dir = PathBuf::from("./plugins");
    if !plugin_dir.exists() {
        std::fs::create_dir_all(&plugin_dir).expect("create plugins dir");
    }

    let mut manager = PluginManager::new(plugin_dir);
    manager.load_all();
    info!("Loaded {} plugin(s)", manager.list().len());

    // Pull a clone of the shared blackboard *before* the manager moves
    // into the Arc<Mutex>. `SharedBlackboard` wraps an `Arc<Mutex<…>>`,
    // so this clone keeps pointing at the same inner map the plugins
    // see. Used by `publish_telemetry_to_blackboard` each tick.
    let blackboard = manager.blackboard.clone();
    let manager = Arc::new(Mutex::new(manager));

    // Broadcast channel that carries `CoreMessage`s out to every
    // connected UI client. Producers: this loop (real telemetry frames)
    // and, when the `mock_telemetry` feature is on, a sine-wave task
    // inside `ipc::start_ipc_server`. Capacity 256 absorbs short UI
    // stalls without dropping frames.
    let (ipc_tx, _ipc_rx) = broadcast::channel::<CoreMessage>(256);
    tokio::spawn(ipc::start_ipc_server(manager.clone(), ipc_tx.clone()));

    #[cfg(feature = "mock_telemetry")]
    warn!(
        "mock_telemetry feature is ENABLED — real telemetry IPC push is also \
         disabled to avoid two producers racing on the same channel. \
         Disable this feature for production."
    );

    // Watchdog: a u64 heartbeat (microseconds since `daemon_start`)
    // that the control loop bumps every tick. If it goes stale beyond
    // `WATCHDOG_STALL_MS`, the watchdog task forces vJoy into the
    // failsafe state so the truck doesn't keep cruising blind on the
    // last good outputs.
    let daemon_start = Instant::now();
    let heartbeat = Arc::new(AtomicU64::new(0));
    heartbeat.store(0, Ordering::Relaxed);
    tokio::spawn(watchdog_loop(heartbeat.clone(), daemon_start));

    let mut last_log = Instant::now();
    // 50 ms IPC throttle: matches the 20 Hz cadence the UI subscribes
    // at; never sends more than one telemetry frame per UI render
    // cycle even though the control loop ticks at 50 Hz. Only used on
    // the real-telemetry path; gated to silence `unused` warnings
    // when `mock_telemetry` is on.
    #[cfg(not(feature = "mock_telemetry"))]
    let mut last_ipc_push = Instant::now();
    let mut output = ControlOutput::default();

    info!("Running — press Ctrl+C to stop");

    loop {
        // Read telemetry off the async executor — see `read_telemetry_async`
        // doc comment in `crates/telemetry/src/lib.rs` for the rationale.
        // Reading before locking the manager keeps the lock window small.
        let telemetry = truckpilot_telemetry::read_telemetry_async().await;

        // Mirror the frame onto the blackboard *before* taking the
        // manager lock so plugins like `fuel-stops` and `stats-logger`
        // see fresh `telemetry.*` values when their `tick` runs. The
        // blackboard has its own Mutex; no contention with the manager.
        publish_telemetry_to_blackboard(telemetry.as_ref(), &blackboard);

        // IPC broadcast: real telemetry → UI clients, gated to one
        // frame per 50 ms. Compile-time off when `mock_telemetry` is
        // enabled so the synthetic stream from `ipc::spawn_mock_telemetry`
        // is the sole producer.
        #[cfg(not(feature = "mock_telemetry"))]
        if let Some(t) = telemetry.as_ref() {
            if last_ipc_push.elapsed() >= Duration::from_millis(50) {
                // `broadcast::Sender::send` returns `Err` when there are
                // no receivers — which is the normal state until a UI
                // connects. Discard.
                let _ = ipc_tx.send(CoreMessage::Telemetry {
                    v: CoreMessage::VERSION,
                    data: snapshot_from(t),
                });
                last_ipc_push = Instant::now();
            }
        }

        let mut mgr = manager.lock().await;
        mgr.process_reloads();
        mgr.tick_all(telemetry.as_ref(), &mut output);

        if last_log.elapsed() >= Duration::from_secs(1) {
            info!(
                "Tick — steering={:.2} throttle={:.2} brake={:.2} (plugins={})",
                output.steering,
                output.throttle,
                output.brake,
                mgr.list().len()
            );
            last_log = Instant::now();
        }
        drop(mgr);

        // Bump heartbeat *after* a full tick completed so the watchdog
        // measures end-to-end progress, not just async-task entry.
        heartbeat.store(daemon_start.elapsed().as_micros() as u64, Ordering::Relaxed);

        tokio::time::sleep(Duration::from_millis(20)).await;
    }
}

// ---------------------------------------------------------------------------
// Watchdog
// ---------------------------------------------------------------------------

/// Time without a heartbeat update before the watchdog engages
/// failsafe.
const WATCHDOG_STALL_MS: u64 = 100;
/// How often the watchdog wakes up to inspect the heartbeat. Must be
/// notably below `WATCHDOG_STALL_MS` to react inside the same stall.
const WATCHDOG_POLL_MS: u64 = 25;
/// Throttle how often we *log* a "still stalled" warning. The
/// failsafe output is still applied every poll while stalled.
const WATCHDOG_LOG_INTERVAL_S: u64 = 1;

/// Watchdog task. Wakes every [`WATCHDOG_POLL_MS`] and checks how long
/// it has been since the control loop last bumped `heartbeat`. If the
/// gap exceeds [`WATCHDOG_STALL_MS`], applies the vJoy failsafe.
async fn watchdog_loop(heartbeat: Arc<AtomicU64>, daemon_start: Instant) {
    let mut last_warn: Option<Instant> = None;
    let stall_us = WATCHDOG_STALL_MS * 1_000;
    loop {
        tokio::time::sleep(Duration::from_millis(WATCHDOG_POLL_MS)).await;

        let beat_us = heartbeat.load(Ordering::Relaxed);
        // Special-case: very first ticks before the loop has had a
        // chance to bump the heartbeat. Treat 0 as "not started yet".
        if beat_us == 0 {
            continue;
        }

        let now_us = daemon_start.elapsed().as_micros() as u64;
        let age_us = now_us.saturating_sub(beat_us);
        if age_us > stall_us {
            apply_vjoy_failsafe();
            let now = Instant::now();
            let should_warn = match last_warn {
                None => true,
                Some(t) => now.duration_since(t) >= Duration::from_secs(WATCHDOG_LOG_INTERVAL_S),
            };
            if should_warn {
                warn!(
                    "WATCHDOG: control loop stalled for {} ms — vJoy held at failsafe",
                    age_us / 1_000
                );
                last_warn = Some(now);
            }
        }
    }
}

/// Force vJoy into a safe state when the control loop stalls.
///
/// Failsafe values: `steering=0.0`, `throttle=0.0`, `brake=0.3`.
/// Brake is *non-zero* to actively slow the truck down rather than
/// merely freeing the pedals — a stalled daemon almost always means
/// something is very wrong and continuing to coast is more dangerous
/// than a moderate brake.
fn apply_vjoy_failsafe() {
    // TODO(Phase 6): wire this to the real vJoyInterface.dll path in
    // `crates/plugins/vjoy-output`. Today both that plugin's
    // `send_to_vjoy` and this helper are stubs that only log.
    let failsafe = ControlOutput {
        steering: 0.0,
        throttle: 0.0,
        brake: 0.3,
    };
    tracing::debug!(
        "[failsafe] steer={:.2} thr={:.2} brk={:.2}",
        failsafe.steering,
        failsafe.throttle,
        failsafe.brake,
    );
}

// ---------------------------------------------------------------------------
// Telemetry → Blackboard bridge
// ---------------------------------------------------------------------------

/// All `telemetry.*` keys the daemon may write to the blackboard.
/// Used by [`publish_telemetry_to_blackboard`] to wipe stale values
/// when telemetry becomes unavailable. Order matches `Telemetry`'s
/// field declaration so it's easy to keep in sync if the struct grows.
const TELEMETRY_BLACKBOARD_KEYS: &[&str] = &[
    "telemetry.position_x",
    "telemetry.position_y",
    "telemetry.position_z",
    "telemetry.heading",
    "telemetry.pitch",
    "telemetry.roll",
    "telemetry.speed_ms",
    "telemetry.engine_rpm",
    "telemetry.cruise_control_kmh",
    "telemetry.nav_speed_limit_kmh",
    "telemetry.lead_vehicle_distance_m",
    "telemetry.accel_longitudinal",
    "telemetry.fuel_liters",
    "telemetry.odometer_km",
];

/// Mirror the current telemetry frame onto the shared blackboard so
/// plugins can consume it via `bb.get_f64("telemetry.fuel_liters")`
/// etc. — without holding a `&Telemetry` reference.
///
/// **Sentinel handling.** Fields that use the `-1.0 = not available`
/// convention (`nav_speed_limit_kmh`, `lead_vehicle_distance_m`,
/// `accel_longitudinal`, `fuel_liters`, `odometer_km`) are **removed**
/// from the blackboard rather than written as the literal string
/// `"-1"`. Plugins must therefore treat a missing key as "not
/// available", not as zero. See the Standard-keys doc table in
/// `truckpilot_plugin_api`.
///
/// When `telemetry` is `None`, `telemetry.available` is set to
/// `"false"` and every other `telemetry.*` key is removed so a stalled
/// source can't leave plugins reading stale frames.
fn publish_telemetry_to_blackboard(t: Option<&Telemetry>, bb: &SharedBlackboard) {
    let Some(t) = t else {
        bb.set("telemetry.available", "false");
        for key in TELEMETRY_BLACKBOARD_KEYS {
            bb.remove(key);
        }
        return;
    };

    bb.set("telemetry.available", "true");
    bb.set("telemetry.position_x", t.position[0].to_string());
    bb.set("telemetry.position_y", t.position[1].to_string());
    bb.set("telemetry.position_z", t.position[2].to_string());
    bb.set("telemetry.heading", t.heading.to_string());
    bb.set("telemetry.pitch", t.pitch.to_string());
    bb.set("telemetry.roll", t.roll.to_string());
    bb.set("telemetry.speed_ms", t.speed_ms.to_string());
    bb.set("telemetry.engine_rpm", t.engine_rpm.to_string());
    bb.set(
        "telemetry.cruise_control_kmh",
        t.cruise_control_kmh.to_string(),
    );

    set_or_remove(bb, "telemetry.nav_speed_limit_kmh", t.nav_speed_limit_kmh);
    set_or_remove(
        bb,
        "telemetry.lead_vehicle_distance_m",
        f64::from(t.lead_vehicle_distance_m),
    );
    set_or_remove(
        bb,
        "telemetry.accel_longitudinal",
        f64::from(t.accel_longitudinal),
    );
    set_or_remove(bb, "telemetry.fuel_liters", t.fuel_liters);
    set_or_remove(bb, "telemetry.odometer_km", t.odometer_km);
}

/// Write `value` as an `f64` string to `bb[key]`, or remove the key
/// entirely if the value is the `-1.0` sentinel ("not available").
fn set_or_remove(bb: &SharedBlackboard, key: &str, value: f64) {
    if value < 0.0 {
        bb.remove(key);
    } else {
        bb.set(key, value.to_string());
    }
}

/// Project a `Telemetry` frame into the `TelemetrySnapshot` carried by
/// `CoreMessage::Telemetry`.
///
/// `TelemetrySnapshot` is intentionally narrower than `Telemetry`: it
/// carries only the six fields the UI's telemetry tab consumes today.
/// Extending the snapshot (pitch/roll/fuel/odometer/etc.) is a Phase 7
/// concern — it requires bumping `PROTOCOL_VERSION` in the IPC crate
/// and updating the TS-side mirror in `crates/ui/src/lib/types.ts`.
///
/// Only the real-IPC-push path calls this; the synthetic mock stream
/// in `ipc.rs` constructs its own snapshot inline. Hence the
/// `dead_code` allowance under `mock_telemetry`.
#[cfg_attr(feature = "mock_telemetry", allow(dead_code))]
fn snapshot_from(t: &Telemetry) -> TelemetrySnapshot {
    TelemetrySnapshot {
        position: t.position,
        heading: t.heading,
        speed_ms: t.speed_ms,
        engine_rpm: t.engine_rpm,
        cruise_control_kmh: t.cruise_control_kmh,
        nav_speed_limit_kmh: t.nav_speed_limit_kmh,
    }
}

#[cfg(test)]
mod telemetry_blackboard_tests {
    use super::*;

    fn fake_telemetry() -> Telemetry {
        Telemetry {
            position: [100.0, 5.0, -50.0],
            heading: 0.5,
            pitch: 0.0,
            roll: 0.0,
            speed_ms: 22.222,
            engine_rpm: 1500.0,
            cruise_control_kmh: 80.0,
            nav_speed_limit_kmh: 80.0,
            lead_vehicle_distance_m: -1.0,
            accel_longitudinal: -1.0,
            fuel_liters: 320.0,
            odometer_km: 12_345.0,
        }
    }

    #[test]
    fn writes_available_true_with_frame() {
        let bb = SharedBlackboard::new();
        publish_telemetry_to_blackboard(Some(&fake_telemetry()), &bb);
        assert_eq!(bb.get("telemetry.available").as_deref(), Some("true"));
    }

    #[test]
    fn writes_position_and_speed() {
        let bb = SharedBlackboard::new();
        publish_telemetry_to_blackboard(Some(&fake_telemetry()), &bb);
        assert_eq!(bb.get_f64("telemetry.position_x"), Some(100.0));
        assert_eq!(bb.get_f64("telemetry.position_y"), Some(5.0));
        assert_eq!(bb.get_f64("telemetry.position_z"), Some(-50.0));
        assert_eq!(bb.get_f64("telemetry.speed_ms"), Some(22.222));
        assert_eq!(bb.get_f64("telemetry.engine_rpm"), Some(1500.0));
    }

    #[test]
    fn writes_fuel_and_odometer_when_present() {
        let bb = SharedBlackboard::new();
        publish_telemetry_to_blackboard(Some(&fake_telemetry()), &bb);
        assert_eq!(bb.get_f64("telemetry.fuel_liters"), Some(320.0));
        assert_eq!(bb.get_f64("telemetry.odometer_km"), Some(12_345.0));
    }

    #[test]
    fn skips_sentinels_for_optional_fields() {
        let bb = SharedBlackboard::new();
        publish_telemetry_to_blackboard(Some(&fake_telemetry()), &bb);
        // `lead_vehicle_distance_m` and `accel_longitudinal` are -1.0
        // in the fixture → key must NOT be present.
        assert_eq!(bb.get("telemetry.lead_vehicle_distance_m"), None);
        assert_eq!(bb.get("telemetry.accel_longitudinal"), None);
    }

    #[test]
    fn none_marks_unavailable_and_clears_keys() {
        let bb = SharedBlackboard::new();
        // Pre-seed the blackboard from a good frame.
        publish_telemetry_to_blackboard(Some(&fake_telemetry()), &bb);
        assert!(bb.get("telemetry.speed_ms").is_some());

        // Now the source goes dead — every field must be wiped, and
        // `available` flipped to false.
        publish_telemetry_to_blackboard(None, &bb);
        assert_eq!(bb.get("telemetry.available").as_deref(), Some("false"));
        for key in TELEMETRY_BLACKBOARD_KEYS {
            assert!(
                bb.get(key).is_none(),
                "key {key} should have been removed when telemetry went None"
            );
        }
    }

    #[test]
    fn re_publish_after_none_repopulates() {
        let bb = SharedBlackboard::new();
        publish_telemetry_to_blackboard(None, &bb);
        publish_telemetry_to_blackboard(Some(&fake_telemetry()), &bb);
        assert_eq!(bb.get("telemetry.available").as_deref(), Some("true"));
        assert_eq!(bb.get_f64("telemetry.fuel_liters"), Some(320.0));
    }
}
