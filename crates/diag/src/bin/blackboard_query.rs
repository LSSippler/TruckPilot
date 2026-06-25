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
//!   --prefix <prefix>     List keys with this prefix
//!   --keys <k1,k2,...>    Fetch specific keys (outputs key=value)
//!   --values              With --prefix: fetch and print key=value lines
//!   --names-only          With --prefix: print key names only (default)
//!   --help                Show this help

use std::collections::HashMap;
use std::net::TcpStream;

use truckpilot_ipc_protocol::{CoreMessage, UiCommand};
use tungstenite::{connect, stream::MaybeTlsStream, WebSocket};

pub(crate) const MISSING: &str = "<missing>";

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

struct Config {
    url: String,
    mode: Mode,
}

enum Mode {
    List {
        prefix: Option<String>,
        with_values: bool,
    },
    Get { keys: Vec<String> },
    Set { key: String, value: String },
}

impl Config {
    fn from_args() -> Self {
        let args: Vec<String> = std::env::args().collect();
        let mut url = "ws://127.0.0.1:8765".to_string();
        let mut prefix: Option<String> = None;
        let mut keys: Option<Vec<String>> = None;
        let mut with_values = false;
        let mut names_only = false;

        let mut set_kv: Option<(String, String)> = None;
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
                    keys = Some(args[i + 1].split(',').map(str::trim).map(str::to_string).collect());
                    i += 2;
                }
                "--values" => {
                    with_values = true;
                    i += 1;
                }
                "--names-only" => {
                    names_only = true;
                    i += 1;
                }
                "--set" if i + 1 < args.len() => {
                    let pair = &args[i + 1];
                    match pair.find('=') {
                        Some(pos) => {
                            set_kv = Some((pair[..pos].to_string(), pair[pos + 1..].to_string()));
                        }
                        None => {
                            eprintln!("Error: --set requires <key>=<value> format");
                            std::process::exit(2);
                        }
                    }
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

        let mode = if let Some((key, value)) = set_kv {
            Mode::Set { key, value }
        } else {
            match keys {
                Some(k) => Mode::Get { keys: k },
                None => Mode::List {
                    prefix,
                    with_values: with_values && !names_only,
                },
            }
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
    println!("  --keys <k1,k2,...>   Fetch specific keys as key=value lines");
    println!("  --values             With --prefix: fetch values (key=value output)");
    println!("  --names-only         With --prefix: key names only (default)");
    println!("  --set <key>=<value>  Set a blackboard key in the running daemon");
    println!("  --help               Show this help");
    println!();
    println!("Examples:");
    println!("  blackboard-query --prefix navigation.ets2_route --values");
    println!("  blackboard-query --keys navigation.ets2_route.imported,router.active");
    println!("  blackboard-query --prefix vjoy --names-only");
    println!("  blackboard-query --set lane_keeper.mode=vision");
}

// ---------------------------------------------------------------------------
// Output formatting (unit-tested)
// ---------------------------------------------------------------------------

pub(crate) fn format_key_value(key: &str, value: Option<&str>) -> String {
    match value {
        Some(v) => format!("{key}={v}"),
        None => format!("{key}={MISSING}"),
    }
}

pub(crate) fn format_requested_keys(
    requested: &[String],
    values: &HashMap<String, String>,
) -> Vec<String> {
    requested
        .iter()
        .map(|k| format_key_value(k, values.get(k).map(String::as_str)))
        .collect()
}

pub(crate) fn format_prefix_values(
    keys: &[String],
    values: &HashMap<String, String>,
) -> Vec<String> {
    let mut sorted = keys.to_vec();
    sorted.sort();
    sorted
        .iter()
        .map(|k| format_key_value(k, values.get(k).map(String::as_str)))
        .collect()
}

// ---------------------------------------------------------------------------
// WebSocket helpers
// ---------------------------------------------------------------------------

fn open_ws(url: &str) -> Result<WebSocket<MaybeTlsStream<TcpStream>>, String> {
    connect(url).map_err(|e| e.to_string()).map(|(ws, _)| ws)
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

fn fetch_values(
    ws: &mut WebSocket<MaybeTlsStream<TcpStream>>,
    keys: &[String],
) -> Result<HashMap<String, String>, String> {
    if keys.is_empty() {
        return Ok(HashMap::new());
    }
    send_cmd(
        ws,
        &UiCommand::BlackboardGet {
            keys: keys.to_vec(),
        },
    )?;
    match recv_reply(ws)? {
        CoreMessage::BlackboardSnapshot { values, .. } => Ok(values),
        other => Err(format!("Unexpected reply: {other:?}")),
    }
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

    if let Mode::Set { key, value } = &cfg.mode {
        let cmd = UiCommand::SetBlackboardKey {
            key: key.clone(),
            value: value.clone(),
        };
        if let Err(e) = send_cmd(&mut ws, &cmd) {
            eprintln!("Error sending command: {e}");
            std::process::exit(1);
        }
        println!("Set {key} = {value}");
        let _ = ws.close(None);
        return;
    }

    match &cfg.mode {
        Mode::List {
            prefix,
            with_values,
        } => {
            send_cmd(
                &mut ws,
                &UiCommand::BlackboardList {
                    prefix: prefix.clone(),
                },
            )
            .unwrap_or_else(|e| {
                eprintln!("Error sending command: {e}");
                std::process::exit(1);
            });

            match recv_reply(&mut ws) {
                Ok(CoreMessage::BlackboardKeys { keys, .. }) => {
                    if *with_values {
                        let values = fetch_values(&mut ws, &keys).unwrap_or_else(|e| {
                            eprintln!("Error: {e}");
                            std::process::exit(1);
                        });
                        for line in format_prefix_values(&keys, &values) {
                            println!("{line}");
                        }
                    } else {
                        for k in &keys {
                            println!("{k}");
                        }
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
        }
        Mode::Get { keys } => {
            let values = fetch_values(&mut ws, keys).unwrap_or_else(|e| {
                eprintln!("Error: {e}");
                std::process::exit(1);
            });
            for line in format_requested_keys(keys, &values) {
                println!("{line}");
            }
        }
        Mode::Set { .. } => unreachable!(),
    }

    let _ = ws.close(None);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn format_key_value_present_and_missing() {
        assert_eq!(format_key_value("router.active", Some("false")), "router.active=false");
        assert_eq!(
            format_key_value("router.missing", None),
            "router.missing=<missing>"
        );
    }

    #[test]
    fn format_requested_keys_preserves_order_and_missing() {
        let requested = vec![
            "a".to_string(),
            "b".to_string(),
            "c".to_string(),
        ];
        let mut values = HashMap::new();
        values.insert("a".to_string(), "1".to_string());
        values.insert("c".to_string(), "3".to_string());
        let lines = format_requested_keys(&requested, &values);
        assert_eq!(lines, vec!["a=1", "b=<missing>", "c=3"]);
    }

    #[test]
    fn format_prefix_values_sorted() {
        let keys = vec!["z".to_string(), "a".to_string()];
        let mut values = HashMap::new();
        values.insert("a".to_string(), "1".to_string());
        values.insert("z".to_string(), "9".to_string());
        assert_eq!(
            format_prefix_values(&keys, &values),
            vec!["a=1", "z=9"]
        );
    }
}
