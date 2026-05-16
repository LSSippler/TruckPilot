//! WebSocket IPC server for UI communication.

use std::sync::Arc;

use futures_util::{SinkExt, StreamExt};
use serde_json::Value;
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::{broadcast, Mutex};
use tokio_tungstenite::{accept_async, tungstenite::Message};
use tracing::{info, warn};

use truckpilot_ipc_protocol::{
    CoreMessage, ModInfo, PidProfile, PluginEventKind, UiCommand, PROTOCOL_VERSION,
};

use crate::plugin_manager::PluginManager;

pub type SharedManager = Arc<Mutex<PluginManager>>;

const V: u32 = CoreMessage::VERSION;

/// Start the IPC WebSocket server on `127.0.0.1:8765`.
///
/// `tx` is the broadcast channel that `main::run_daemon` produces real
/// `CoreMessage::Telemetry` frames into. The IPC server simply
/// forwards every message to all connected UI clients via
/// [`handle_connection`].
///
/// When the `mock_telemetry` cargo feature is enabled, a synthetic
/// sine-wave producer is *also* spawned into the same channel so the
/// UI can be developed without ETS2 running. The feature is **off**
/// by default — see `crates/core/Cargo.toml`.
pub async fn start_ipc_server(manager: SharedManager, tx: broadcast::Sender<CoreMessage>) {
    let addr = "127.0.0.1:8765";
    let listener = TcpListener::bind(addr).await.expect("bind websocket");
    info!("IPC WebSocket server listening on {}", addr);

    spawn_mock_telemetry(tx.clone());

    while let Ok((stream, _)) = listener.accept().await {
        let tx = tx.clone();
        let mgr = manager.clone();
        tokio::spawn(handle_connection(stream, tx, mgr));
    }
}

#[cfg(feature = "mock_telemetry")]
fn spawn_mock_telemetry(tx: broadcast::Sender<CoreMessage>) {
    tracing::warn!(
        "mock_telemetry feature is ENABLED — synthetic sine-wave telemetry will be broadcast. \
         Disable this feature for production builds."
    );
    tokio::spawn(async move {
        let mut last_sent = std::time::Instant::now();
        let mut t: f64 = 0.0;
        loop {
            let now = std::time::Instant::now();
            if now.duration_since(last_sent) < std::time::Duration::from_millis(50) {
                tokio::time::sleep(std::time::Duration::from_millis(5)).await;
                continue;
            }
            last_sent = now;
            t += 0.05;

            let msg = CoreMessage::Telemetry {
                v: V,
                data: truckpilot_ipc_protocol::TelemetrySnapshot {
                    position: [12345.6 + 5.0 * t.sin(), 78.9, -1234.5 + 5.0 * t.cos()],
                    heading: (t * 0.05).sin(),
                    speed_ms: 22.2 + (t * 0.3).sin() * 1.5,
                    engine_rpm: 1350.0,
                    cruise_control_kmh: 80.0,
                    nav_speed_limit_kmh: 80.0,
                },
            };
            let _ = tx.send(msg);
        }
    });
}

#[cfg(not(feature = "mock_telemetry"))]
fn spawn_mock_telemetry(_tx: broadcast::Sender<CoreMessage>) {
    // No-op: the real telemetry source publishes into `tx` directly.
}

async fn handle_connection(
    stream: TcpStream,
    tx: broadcast::Sender<CoreMessage>,
    manager: SharedManager,
) {
    let ws_stream = match accept_async(stream).await {
        Ok(s) => s,
        Err(e) => {
            warn!("WebSocket handshake failed: {}", e);
            return;
        }
    };

    let (mut write, mut read) = ws_stream.split();

    let hello = CoreMessage::Hello {
        v: V,
        version: PROTOCOL_VERSION.to_string(),
    };
    if let Ok(json) = serde_json::to_string(&hello) {
        let _ = write.send(Message::Text(json)).await;
    }

    let plugins = manager.lock().await.list();
    let list_msg = CoreMessage::PluginList { v: V, plugins };
    if let Ok(json) = serde_json::to_string(&list_msg) {
        let _ = write.send(Message::Text(json)).await;
    }

    let mut rx = tx.subscribe();

    loop {
        tokio::select! {
            msg = read.next() => {
                match msg {
                    Some(Ok(Message::Text(text))) => {
                        match serde_json::from_str::<UiCommand>(&text) {
                            Ok(cmd) => handle_command(cmd, &manager, &mut write).await,
                            Err(err) => {
                                warn!("invalid UI command: {} ({err})", text);
                                let msg = CoreMessage::error(
                                    "bad_request",
                                    None,
                                    format!("malformed command: {err}"),
                                );
                                if let Ok(json) = serde_json::to_string(&msg) {
                                    let _ = write.send(Message::Text(json)).await;
                                }
                            }
                        }
                    }
                    Some(Ok(Message::Close(_))) | None => break,
                    _ => {}
                }
            }
            core_msg = rx.recv() => {
                if let Ok(msg) = core_msg {
                    if let Ok(json) = serde_json::to_string(&msg) {
                        if write.send(Message::Text(json)).await.is_err() {
                            break;
                        }
                    }
                }
            }
        }
    }
}

type WriteHalf =
    futures_util::stream::SplitSink<tokio_tungstenite::WebSocketStream<TcpStream>, Message>;

async fn handle_command(cmd: UiCommand, manager: &SharedManager, write: &mut WriteHalf) {
    let response = build_response(cmd, manager).await;
    for msg in response {
        if let Ok(json) = serde_json::to_string(&msg) {
            if write.send(Message::Text(json)).await.is_err() {
                break;
            }
        }
    }
}

async fn build_response(cmd: UiCommand, manager: &SharedManager) -> Vec<CoreMessage> {
    match cmd {
        UiCommand::Ping => Vec::new(),
        UiCommand::RequestPluginList => {
            let plugins = manager.lock().await.list();
            vec![CoreMessage::PluginList { v: V, plugins }]
        }
        UiCommand::PluginToggle { name, enabled } => {
            let ok = manager.lock().await.set_enabled(&name, enabled);
            if ok {
                vec![CoreMessage::PluginEvent {
                    v: V,
                    plugin: name,
                    event: PluginEventKind::SettingsChanged,
                }]
            } else {
                vec![CoreMessage::error(
                    "not_found",
                    Some("plugin_toggle".into()),
                    format!("plugin '{name}' not found"),
                )]
            }
        }
        UiCommand::PluginReload { name } => {
            info!("Plugin reload requested for {}", name);
            vec![CoreMessage::PluginEvent {
                v: V,
                plugin: name,
                event: PluginEventKind::Loaded,
            }]
        }
        UiCommand::PluginSettingsUpdate { plugin, settings } => {
            info!("settings update for {plugin}: {settings}");
            vec![CoreMessage::PluginEvent {
                v: V,
                plugin,
                event: PluginEventKind::SettingsChanged,
            }]
        }
        UiCommand::RequestPluginSchema { plugin } => {
            let schema = manager.lock().await.schema_for(&plugin);
            match schema {
                Some(schema) => vec![CoreMessage::PluginSchema {
                    v: V,
                    plugin,
                    schema,
                }],
                None => vec![CoreMessage::error(
                    "not_found",
                    Some("request_plugin_schema".into()),
                    format!("plugin '{plugin}' not found or has no schema"),
                )],
            }
        }
        UiCommand::RequestModList => {
            // TODO(Phase F): scan <ets2-dir>/mod for actual mods.
            vec![CoreMessage::ModList {
                v: V,
                mods: empty_mod_list(),
            }]
        }
        UiCommand::ModApply { active_mods } => {
            info!("mod apply requested with {} mods", active_mods.len());
            vec![CoreMessage::error(
                "not_implemented",
                Some("mod_apply".into()),
                "mod manager service not wired up yet".to_string(),
            )]
        }
        UiCommand::CacheClear => vec![CoreMessage::error(
            "not_implemented",
            Some("cache_clear".into()),
            "cache clear not wired up yet".to_string(),
        )],
        UiCommand::RequestPidProfiles => {
            let profiles = manager.lock().await.pid_profiles();
            vec![CoreMessage::PidProfileList { v: V, profiles }]
        }
        UiCommand::PidProfileUpdate {
            profile,
            kp,
            ki,
            kd,
            output_limit: _,
        } => {
            info!("PID profile update {profile}: kp={kp} ki={ki} kd={kd}");
            let mgr = manager.lock().await;
            match profile.as_str() {
                "lane_keeper" => {
                    mgr.blackboard.set("plugin.lane_keeper.kp", kp.to_string());
                    mgr.blackboard.set("plugin.lane_keeper.ki", ki.to_string());
                    mgr.blackboard.set("plugin.lane_keeper.kd", kd.to_string());
                }
                "speed_controller" | "speed-controller" => {
                    mgr.blackboard
                        .set("plugin.speed_controller.kp", kp.to_string());
                    mgr.blackboard
                        .set("plugin.speed_controller.ki", ki.to_string());
                    mgr.blackboard
                        .set("plugin.speed_controller.kd", kd.to_string());
                }
                _ => {
                    return vec![CoreMessage::error(
                        "not_found",
                        Some("pid_profile_update".into()),
                        format!("unknown PID profile '{profile}'"),
                    )];
                }
            }
            let profiles = mgr.pid_profiles();
            vec![CoreMessage::PidProfileList { v: V, profiles }]
        }
        UiCommand::PidProfileReset { profile } => vec![CoreMessage::error(
            "not_implemented",
            Some("pid_profile_reset".into()),
            format!("reset for '{profile}' not implemented"),
        )],
        UiCommand::PidStreamSubscribe { profile, enabled } => {
            info!("pid stream subscribe profile={profile} enabled={enabled}");
            Vec::new()
        }
        UiCommand::SetLogSubscription { levels, plugin } => {
            info!(
                "log subscription updated levels={:?} plugin={:?}",
                levels, plugin
            );
            Vec::new()
        }
        UiCommand::AutopilotEngage => {
            manager
                .lock()
                .await
                .blackboard
                .set("autopilot.engage_requested", "true");
            info!("autopilot engage requested via IPC");
            Vec::new()
        }
        UiCommand::AutopilotDisengage => {
            manager
                .lock()
                .await
                .blackboard
                .set("autopilot.disengage_requested", "true");
            info!("autopilot disengage requested via IPC");
            Vec::new()
        }
        UiCommand::AutopilotReset => {
            manager
                .lock()
                .await
                .blackboard
                .set("autopilot.reset_requested", "true");
            info!("autopilot reset requested via IPC");
            Vec::new()
        }
        UiCommand::BlackboardGet { keys } => {
            let bb = manager.lock().await.blackboard.clone();
            let values: std::collections::HashMap<String, String> = keys
                .into_iter()
                .filter_map(|k| bb.get(&k).map(|v| (k, v)))
                .collect();
            vec![truckpilot_ipc_protocol::CoreMessage::BlackboardSnapshot { v: V, values }]
        }
        UiCommand::BlackboardList { prefix } => {
            let bb = manager.lock().await.blackboard.clone();
            let mut keys = bb.keys(prefix.as_deref());
            keys.sort();
            vec![truckpilot_ipc_protocol::CoreMessage::BlackboardKeys { v: V, keys }]
        }
    }
}

fn empty_mod_list() -> Vec<ModInfo> {
    Vec::new()
}

impl PluginManager {
    /// Look up a plugin's settings schema and parse it as JSON.
    pub fn schema_for(&self, name: &str) -> Option<Value> {
        let schema_str = self.schema_string(name)?;
        serde_json::from_str(schema_str).ok()
    }

    /// Derive a default PID profile list from known plugin names.
    /// Real implementation will pull live values from each plugin's settings; for now
    /// this returns hardcoded defaults so the UI tab has something to render.
    pub fn pid_profiles(&self) -> Vec<PidProfile> {
        const KNOWN: &[(&str, f64, f64, f64, f64)] = &[
            ("steering", 1.0, 0.0, 0.05, 1.0),
            ("acc", 0.6, 0.05, 0.0, 1.0),
            ("lane_keeper", 0.8, 0.0, 0.1, 1.0),
        ];
        KNOWN
            .iter()
            .map(|(name, kp, ki, kd, lim)| PidProfile {
                name: (*name).to_string(),
                kp: *kp,
                ki: *ki,
                kd: *kd,
                output_limit: *lim,
            })
            .collect()
    }
}

#[cfg(test)]
mod ipc_command_tests {
    use super::*;

    fn manager_for_test() -> SharedManager {
        let dir = std::env::temp_dir().join("truckpilot-ipc-test-plugins");
        let _ = std::fs::create_dir_all(&dir);
        Arc::new(Mutex::new(PluginManager::new(dir)))
    }

    #[tokio::test]
    async fn engage_command_sets_blackboard() {
        let mgr = manager_for_test();
        let response = build_response(UiCommand::AutopilotEngage, &mgr).await;
        assert!(response.is_empty(), "engage produces no immediate reply");
        let value = mgr
            .lock()
            .await
            .blackboard
            .get("autopilot.engage_requested");
        assert_eq!(value.as_deref(), Some("true"));
    }

    #[tokio::test]
    async fn disengage_command_sets_blackboard() {
        let mgr = manager_for_test();
        let _ = build_response(UiCommand::AutopilotDisengage, &mgr).await;
        let value = mgr
            .lock()
            .await
            .blackboard
            .get("autopilot.disengage_requested");
        assert_eq!(value.as_deref(), Some("true"));
    }

    #[tokio::test]
    async fn reset_command_sets_blackboard() {
        let mgr = manager_for_test();
        let _ = build_response(UiCommand::AutopilotReset, &mgr).await;
        let value = mgr.lock().await.blackboard.get("autopilot.reset_requested");
        assert_eq!(value.as_deref(), Some("true"));
    }
}
