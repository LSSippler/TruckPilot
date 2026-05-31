//! TruckPilot Engage CLI (Phase 6.2b A1)
//!
//! Minimal command-line driver for the autopilot engage workflow. Talks to the
//! running daemon over WebSocket (`ws://127.0.0.1:8765` by default).
//!
//! Subcommands:
//!   set-goal <uid>           Set router.goal_uid (requires a valid road-node UID)
//!   set-goal-pos --x --z     Set router goal by ETS2 world coords; server snaps to nearest node
//!   set-start <uid>          Set router.start_uid (or pass --clear to use current position)
//!   set-cruise <kmh>         Set cruise.target_kmh
//!   engage                   Request AutopilotEngage
//!   disengage                Request AutopilotDisengage
//!   reset                    Request AutopilotReset
//!   status                   Print autopilot.state, fault_reason, preconditions

use std::net::TcpStream;

use clap::{Parser, Subcommand};
use truckpilot_ipc_protocol::{CoreMessage, UiCommand};
use tungstenite::{connect, stream::MaybeTlsStream, WebSocket};

#[derive(Parser, Debug)]
#[command(name = "engage-cli", about = "TruckPilot engage workflow CLI")]
struct Cli {
    /// Daemon WebSocket URL
    #[arg(long, default_value = "ws://127.0.0.1:8765")]
    url: String,

    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand, Debug)]
enum Cmd {
    /// Set router.goal_uid (requires a valid road-node UID)
    SetGoal { uid: u64 },
    /// Set router goal by ETS2 world-space position; server snaps to nearest road node
    SetGoalPos {
        /// World X coordinate in metres (ETS2 space)
        #[arg(long)]
        x: f64,
        /// World Z coordinate in metres (ETS2 space)
        #[arg(long)]
        z: f64,
    },
    /// Set router.start_uid; pass --clear to use the current truck position instead
    SetStart {
        #[arg(conflicts_with = "clear")]
        uid: Option<u64>,
        #[arg(long, default_value_t = false)]
        clear: bool,
    },
    /// Set cruise.target_kmh
    SetCruise { kmh: f32 },
    /// Request autopilot engage
    Engage {
        /// Engage in lane-only mode (no route required; steering only)
        #[arg(long, default_value_t = false)]
        lane_only: bool,
    },
    /// Request autopilot disengage
    Disengage,
    /// Request autopilot reset (from Fault back to Off)
    Reset,
    /// Print autopilot state + fault reason + preconditions
    Status,
}

type Ws = WebSocket<MaybeTlsStream<TcpStream>>;

fn open_ws(url: &str) -> Result<Ws, String> {
    connect(url).map(|(ws, _)| ws).map_err(|e| e.to_string())
}

fn send_cmd(ws: &mut Ws, cmd: &UiCommand) -> Result<(), String> {
    let json = serde_json::to_string(cmd).map_err(|e| e.to_string())?;
    ws.send(tungstenite::Message::Text(json))
        .map_err(|e| e.to_string())
}

/// Read messages until we get one matching `pred`; skip Hello/PluginList chatter
/// and any unrelated push frames.
fn recv_until<F>(ws: &mut Ws, pred: F) -> Result<CoreMessage, String>
where
    F: Fn(&CoreMessage) -> bool,
{
    for _ in 0..40 {
        let msg = ws.read().map_err(|e| e.to_string())?;
        let text = match msg {
            tungstenite::Message::Text(t) => t,
            tungstenite::Message::Close(_) => return Err("connection closed".into()),
            _ => continue,
        };
        let core: CoreMessage = serde_json::from_str(&text).map_err(|e| e.to_string())?;
        if pred(&core) {
            return Ok(core);
        }
    }
    Err("no matching reply received".into())
}

fn fire_and_forget(ws: &mut Ws, cmd: &UiCommand, label: &str) -> Result<(), String> {
    send_cmd(ws, cmd)?;
    // Brief pause so the daemon has a chance to process and (for Engage) the
    // next AutopilotStatus push reflects the new request.
    std::thread::sleep(std::time::Duration::from_millis(150));
    println!("OK: {label}");
    Ok(())
}

fn print_status(ws: &mut Ws) -> Result<(), String> {
    // Daemon pushes AutopilotStatus every 100 ms (10 Hz), so the next frame
    // should arrive promptly. recv_until caps at 40 reads.
    let reply = recv_until(ws, |m| matches!(m, CoreMessage::AutopilotStatus { .. }));
    match reply {
        Ok(CoreMessage::AutopilotStatus {
            state,
            fault_reason,
            preconditions,
            tick_count,
            ..
        }) => {
            println!("autopilot.state       = {state}");
            println!(
                "autopilot.fault       = {}",
                fault_reason.as_deref().unwrap_or("(none)")
            );
            println!("autopilot.tick_count  = {tick_count}");
            println!("preconditions:");
            println!("  telemetry_ok        = {}", preconditions.telemetry_ok);
            println!("  engine_running      = {}", preconditions.engine_running);
            println!(
                "  critical_plugins    = {}",
                preconditions.critical_plugins_loaded
            );
            println!("  router_active       = {}", preconditions.router_active);
            Ok(())
        }
        Ok(other) => Err(format!("unexpected reply: {other:?}")),
        Err(e) => Err(e),
    }
}

fn main() {
    let cli = Cli::parse();

    let mut ws = match open_ws(&cli.url) {
        Ok(ws) => ws,
        Err(e) => {
            eprintln!("Error: cannot connect to {} -- {}", cli.url, e);
            eprintln!("Is the TruckPilot daemon running?");
            std::process::exit(1);
        }
    };

    let result = match cli.cmd {
        Cmd::SetGoal { uid } => fire_and_forget(
            &mut ws,
            &UiCommand::SetRouterGoal { uid },
            &format!("router.goal_uid = {uid}"),
        ),
        Cmd::SetGoalPos { x, z } => fire_and_forget(
            &mut ws,
            &UiCommand::SetRouterGoalByPosition { x, z },
            &format!("router goal by position ({x:.1}, {z:.1})"),
        ),
        Cmd::SetStart { uid, clear } => {
            let value = if clear { None } else { uid };
            let label = match value {
                Some(u) => format!("router.start_uid = {u}"),
                None => "router.start_uid cleared (use current position)".to_string(),
            };
            fire_and_forget(&mut ws, &UiCommand::SetRouterStart { uid: value }, &label)
        }
        Cmd::SetCruise { kmh } => fire_and_forget(
            &mut ws,
            &UiCommand::SetCruiseTarget { kmh },
            &format!("cruise.target_kmh = {kmh}"),
        ),
        Cmd::Engage { lane_only } => {
            let pre = if lane_only {
                fire_and_forget(
                    &mut ws,
                    &UiCommand::SetBlackboardKey {
                        key: "autopilot.requested_mode".to_string(),
                        value: "lane_only".to_string(),
                    },
                    "autopilot.requested_mode = lane_only",
                )
            } else {
                Ok(())
            };
            pre.and_then(|_| {
                fire_and_forget(&mut ws, &UiCommand::AutopilotEngage, "engage requested")
            })
        }
        Cmd::Disengage => fire_and_forget(
            &mut ws,
            &UiCommand::AutopilotDisengage,
            "disengage requested",
        ),
        Cmd::Reset => fire_and_forget(&mut ws, &UiCommand::AutopilotReset, "reset requested"),
        Cmd::Status => print_status(&mut ws),
    };

    if let Err(e) = result {
        eprintln!("Error: {e}");
        std::process::exit(1);
    }

    let _ = ws.close(None);
}
