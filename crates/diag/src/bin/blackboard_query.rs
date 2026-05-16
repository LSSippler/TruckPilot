//! TruckPilot Blackboard Query Tool
//!
//! Connects to the running TruckPilot daemon via WebSocket and queries the
//! shared blackboard.
//!
//! Usage:
//!   blackboard-query [OPTIONS]
//!
//! Options:
//!   --url <ws-url>        Daemon WebSocket URL (default: ws://127.0.0.1:8765)
//!   --prefix <prefix>     List all keys with this prefix
//!   --keys <k1,k2,...>    Fetch specific keys by name
//!   --help                Show this help

use std::net::TcpStream;

use truckpilot_ipc_protocol::{CoreMessage, UiCommand};
use tungstenite::{connect, stream::MaybeTlsStream, WebSocket};

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

struct Config {
    url: String,
    mode: Mode,
}

enum Mode {
    List { prefix: Option<String> },
    Get { keys: Vec<String> },
}

impl Config {
    fn from_args() -> Self {
        let args: Vec<String> = std::env::args().collect();
        let mut url = "ws://127.0.0.1:8765".to_string();
        let mut prefix: Option<String> = None;
        let mut keys: Option<Vec<String>> = None;

        let mut i = 1;
        while i < args.len() {
            match args[i].as_str() {
                "--url" if i + 1 < args.len() => {
                    url = args[i + 1].clone();
                    i += 2;
                }
                "--prefix" if i + 1 < args.len() => {
                    prefix = Some(args[i + 1].clone());
                    i += 2;
                }
                "--keys" if i + 1 < args.len() => {
                    keys = Some(args[i + 1].split(',').map(str::to_string).collect());
                    i += 2;
                }
                "--help" | "-h" => {
                    print_help();
                    std::process::exit(0);
                }
                other => {
                    eprintln!("Unknown argument: {other}");
                    std::process::exit(2);
                }
            }
        }

        let mode = match keys {
            Some(k) => Mode::Get { keys: k },
            None => Mode::List { prefix },
        };

        Self { url, mode }
    }
}

fn print_help() {
    println!("blackboard-query — TruckPilot blackboard inspector");
    println!();
    println!("Usage: blackboard-query [OPTIONS]");
    println!();
    println!("Options:");
    println!("  --url <ws-url>       Daemon URL (default: ws://127.0.0.1:8765)");
    println!("  --prefix <prefix>    List keys with given prefix (default: all)");
    println!("  --keys <k1,k2,...>   Fetch specific keys and their values");
    println!("  --help               Show this help");
    println!();
    println!("Examples:");
    println!("  blackboard-query                      # list all keys");
    println!("  blackboard-query --prefix vjoy        # list vjoy.* keys");
    println!("  blackboard-query --keys vjoy.connected,autopilot.state");
}

// ---------------------------------------------------------------------------
// WebSocket helpers
// ---------------------------------------------------------------------------

fn open_ws(url: &str) -> Result<WebSocket<MaybeTlsStream<TcpStream>>, String> {
    connect(url).map(|(ws, _)| ws).map_err(|e| e.to_string())
}

fn send_cmd(ws: &mut WebSocket<MaybeTlsStream<TcpStream>>, cmd: &UiCommand) -> Result<(), String> {
    let json = serde_json::to_string(cmd).map_err(|e| e.to_string())?;
    ws.send(tungstenite::Message::Text(json))
        .map_err(|e| e.to_string())
}

/// Read messages until we get the expected reply type, skipping Hello/PluginList push messages.
fn recv_reply(ws: &mut WebSocket<MaybeTlsStream<TcpStream>>) -> Result<CoreMessage, String> {
    for _ in 0..20 {
        let msg = ws.read().map_err(|e| e.to_string())?;
        let text = match msg {
            tungstenite::Message::Text(t) => t,
            tungstenite::Message::Close(_) => return Err("connection closed".into()),
            _ => continue,
        };
        let core: CoreMessage = serde_json::from_str(&text).map_err(|e| e.to_string())?;
        match &core {
            CoreMessage::Hello { .. } | CoreMessage::PluginList { .. } => continue,
            _ => return Ok(core),
        }
    }
    Err("no relevant reply received".into())
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

fn main() {
    let cfg = Config::from_args();

    let mut ws = match open_ws(&cfg.url) {
        Ok(ws) => ws,
        Err(e) => {
            eprintln!("Error: cannot connect to {} — {}", cfg.url, e);
            eprintln!("Is the TruckPilot daemon running?");
            std::process::exit(1);
        }
    };

    let cmd = match &cfg.mode {
        Mode::List { prefix } => UiCommand::BlackboardList {
            prefix: prefix.clone(),
        },
        Mode::Get { keys } => UiCommand::BlackboardGet { keys: keys.clone() },
    };

    if let Err(e) = send_cmd(&mut ws, &cmd) {
        eprintln!("Error sending command: {e}");
        std::process::exit(1);
    }

    match recv_reply(&mut ws) {
        Ok(CoreMessage::BlackboardKeys { keys, .. }) => {
            println!("{} key(s):", keys.len());
            for k in &keys {
                println!("  {k}");
            }
        }
        Ok(CoreMessage::BlackboardSnapshot { values, .. }) => {
            let mut pairs: Vec<_> = values.iter().collect();
            pairs.sort_by_key(|(k, _)| k.as_str());
            println!("{} key(s):", pairs.len());
            for (k, v) in pairs {
                println!("  {k} = {v}");
            }
        }
        Ok(other) => {
            eprintln!("Unexpected reply: {other:?}");
            std::process::exit(1);
        }
        Err(e) => {
            eprintln!("Error: {e}");
            std::process::exit(1);
        }
    }

    let _ = ws.close(None);
}
