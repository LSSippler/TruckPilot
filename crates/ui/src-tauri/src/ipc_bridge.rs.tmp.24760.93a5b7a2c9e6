//! WebSocket client that bridges the truckpilot-core daemon to the Tauri front-end.
//!
//! Lifecycle:
//!   * `IpcBridge::spawn` starts a long-running tokio task with reconnect-loop.
//!   * Inbound `CoreMessage` JSON is forwarded to the frontend as `core-event`.
//!   * Outbound `UiCommand` is fed via an `mpsc` channel from Tauri commands.
//!   * Connection state changes are emitted as `connection-status`.
//!
//! Reconnect uses exponential backoff (500ms → 30s, ±20% jitter). A manual
//! `IpcBridge::reconnect` shortcut wakes the loop immediately.

use std::sync::Arc;
use std::time::Duration;

use futures_util::{SinkExt, StreamExt};
use rand::Rng;
use serde::Serialize;
use tauri::{AppHandle, Emitter};
use tokio::net::TcpStream;
use tokio::sync::{mpsc, Mutex, Notify};
use tokio_tungstenite::{connect_async, tungstenite::Message, MaybeTlsStream, WebSocketStream};
use tracing::{debug, info, warn};
use truckpilot_ipc_protocol::{CoreMessage, UiCommand};
use url::Url;

const DEFAULT_URL: &str = "ws://127.0.0.1:8765";
const BACKOFF_MIN_MS: u64 = 500;
const BACKOFF_MAX_MS: u64 = 30_000;

#[derive(Debug, Clone, Copy, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum ConnectionStatus {
    Connected,
    Reconnecting,
    Disconnected,
}

#[derive(Debug, Clone, Serialize)]
pub struct ConnectionStatusEvent {
    pub status: ConnectionStatus,
    pub protocol_version: Option<String>,
    pub last_error: Option<String>,
}

#[derive(Default)]
struct StateInner {
    status: Option<ConnectionStatusEvent>,
}

#[derive(Clone)]
pub struct IpcBridge {
    cmd_tx: mpsc::Sender<UiCommand>,
    reconnect: Arc<Notify>,
    state: Arc<Mutex<StateInner>>,
}

impl IpcBridge {
    pub fn spawn(app: AppHandle) -> Self {
        let (cmd_tx, cmd_rx) = mpsc::channel::<UiCommand>(64);
        let reconnect = Arc::new(Notify::new());
        let state = Arc::new(Mutex::new(StateInner::default()));

        let bridge = Self {
            cmd_tx,
            reconnect: reconnect.clone(),
            state: state.clone(),
        };

        let app_clone = app.clone();
        tauri::async_runtime::spawn(run_loop(app_clone, cmd_rx, reconnect, state));

        bridge
    }

    pub async fn send(&self, cmd: UiCommand) -> Result<(), String> {
        self.cmd_tx
            .send(cmd)
            .await
            .map_err(|e| format!("ipc command channel closed: {e}"))
    }

    pub async fn current_status(&self) -> ConnectionStatusEvent {
        self.state
            .lock()
            .await
            .status
            .clone()
            .unwrap_or(ConnectionStatusEvent {
                status: ConnectionStatus::Disconnected,
                protocol_version: None,
                last_error: None,
            })
    }

    pub fn reconnect_now(&self) {
        self.reconnect.notify_one();
    }
}

async fn run_loop(
    app: AppHandle,
    mut cmd_rx: mpsc::Receiver<UiCommand>,
    reconnect: Arc<Notify>,
    state: Arc<Mutex<StateInner>>,
) {
    let url = Url::parse(DEFAULT_URL).expect("static ws url parses");
    let mut backoff_ms = BACKOFF_MIN_MS;

    loop {
        emit_status(&app, &state, ConnectionStatus::Reconnecting, None, None).await;

        match connect_async(url.as_str()).await {
            Ok((stream, _)) => {
                info!("connected to core at {}", url);
                backoff_ms = BACKOFF_MIN_MS;
                handle_session(&app, &state, stream, &mut cmd_rx, &reconnect).await;
            }
            Err(err) => {
                let msg = format!("{err}");
                warn!("ws connect failed: {msg}");
                emit_status(
                    &app,
                    &state,
                    ConnectionStatus::Disconnected,
                    None,
                    Some(msg),
                )
                .await;
            }
        }

        let delay = jittered(backoff_ms);
        debug!("reconnect in {delay:?}");
        tokio::select! {
            _ = tokio::time::sleep(delay) => {}
            _ = reconnect.notified() => {
                info!("manual reconnect requested");
            }
        }
        backoff_ms = next_backoff(backoff_ms);
    }
}

async fn handle_session(
    app: &AppHandle,
    state: &Arc<Mutex<StateInner>>,
    stream: WebSocketStream<MaybeTlsStream<TcpStream>>,
    cmd_rx: &mut mpsc::Receiver<UiCommand>,
    reconnect: &Arc<Notify>,
) {
    let (mut write, mut read) = stream.split();

    loop {
        tokio::select! {
            inbound = read.next() => {
                match inbound {
                    Some(Ok(Message::Text(text))) => match serde_json::from_str::<CoreMessage>(&text) {
                        Ok(msg) => {
                            if let CoreMessage::Hello { ref version, .. } = msg {
                                emit_status(app, state, ConnectionStatus::Connected, Some(version.clone()), None).await;
                            }
                            if let Err(err) = app.emit("core-event", &msg) {
                                warn!("emit core-event failed: {err}");
                            }
                        }
                        Err(err) => warn!("invalid core message: {err} :: {text}"),
                    },
                    Some(Ok(Message::Binary(_))) => {}
                    Some(Ok(Message::Ping(p))) => {
                        let _ = write.send(Message::Pong(p)).await;
                    }
                    Some(Ok(Message::Pong(_))) | Some(Ok(Message::Frame(_))) => {}
                    Some(Ok(Message::Close(_))) | None => {
                        warn!("core closed the connection");
                        emit_status(app, state, ConnectionStatus::Disconnected, None, Some("core closed connection".into())).await;
                        return;
                    }
                    Some(Err(err)) => {
                        warn!("read error: {err}");
                        emit_status(app, state, ConnectionStatus::Disconnected, None, Some(err.to_string())).await;
                        return;
                    }
                }
            }
            outbound = cmd_rx.recv() => {
                let Some(cmd) = outbound else { return };
                match serde_json::to_string(&cmd) {
                    Ok(json) => {
                        if let Err(err) = write.send(Message::Text(json)).await {
                            warn!("send failed: {err}");
                            emit_status(app, state, ConnectionStatus::Disconnected, None, Some(err.to_string())).await;
                            return;
                        }
                    }
                    Err(err) => warn!("serialize command failed: {err}"),
                }
            }
            _ = reconnect.notified() => {
                info!("manual reconnect — closing session");
                let _ = write.send(Message::Close(None)).await;
                return;
            }
        }
    }
}

async fn emit_status(
    app: &AppHandle,
    state: &Arc<Mutex<StateInner>>,
    status: ConnectionStatus,
    protocol_version: Option<String>,
    last_error: Option<String>,
) {
    let event = ConnectionStatusEvent {
        status,
        protocol_version,
        last_error,
    };
    {
        let mut guard = state.lock().await;
        guard.status = Some(event.clone());
    }
    if let Err(err) = app.emit("connection-status", &event) {
        warn!("emit connection-status failed: {err}");
    }
}

fn jittered(base_ms: u64) -> Duration {
    let mut rng = rand::thread_rng();
    let jitter: f64 = rng.gen_range(0.8..1.2);
    Duration::from_millis(((base_ms as f64) * jitter) as u64)
}

/// Compute the next backoff value using the same `*2 capped at max` rule as the
/// reconnect loop. Pulled out for unit-testing.
pub(crate) fn next_backoff(current_ms: u64) -> u64 {
    current_ms.saturating_mul(2).min(BACKOFF_MAX_MS)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn jitter_stays_within_band() {
        for _ in 0..256 {
            let d = jittered(1_000);
            let ms = d.as_millis() as u64;
            assert!(ms >= 800, "jitter went too low: {ms}");
            assert!(ms <= 1_200, "jitter went too high: {ms}");
        }
    }

    #[test]
    fn backoff_doubles_until_cap() {
        assert_eq!(next_backoff(BACKOFF_MIN_MS), BACKOFF_MIN_MS * 2);
        assert_eq!(next_backoff(2_000), 4_000);
        assert_eq!(next_backoff(20_000), BACKOFF_MAX_MS);
        assert_eq!(next_backoff(BACKOFF_MAX_MS), BACKOFF_MAX_MS);
        assert_eq!(next_backoff(u64::MAX), BACKOFF_MAX_MS);
    }
}
