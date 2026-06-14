use std::sync::Arc;

use tauri::{AppHandle, Manager, State};
use tracing::info;
use truckpilot_ipc_protocol::UiCommand;

use crate::daemon::{DaemonManager, DaemonStatus};
use crate::daemon_config::{self, DaemonConfig};
use crate::hotkey_config::{self, HotkeyConfig};
use crate::hotkey_manager;
use crate::ipc_bridge::{ConnectionStatusEvent, IpcBridge};
use crate::steam_detect::detect_ets2_install;
use crate::window_manager;

#[tauri::command]
pub async fn send_command(bridge: State<'_, IpcBridge>, cmd: UiCommand) -> Result<(), String> {
    // Log SetRouterGoal so we can verify the UID string is not corrupted
    // between the UI and the WebSocket send (Task 5, Phase 6.5c).
    if let UiCommand::SetRouterGoal { uid } = &cmd {
        info!("[tauri] SetRouterGoal received from UI: uid='{uid}'");
    }
    let result = bridge.send(cmd).await;
    if let Ok(()) = &result {
        // Could also log here, but the daemon-side ipc.rs already logs "router goal set via IPC".
    }
    result
}

#[tauri::command]
pub async fn get_connection_status(
    bridge: State<'_, IpcBridge>,
) -> Result<ConnectionStatusEvent, String> {
    Ok(bridge.current_status().await)
}

#[tauri::command]
pub async fn reconnect(bridge: State<'_, IpcBridge>) -> Result<(), String> {
    bridge.reconnect_now();
    Ok(())
}

#[tauri::command]
pub async fn detect_ets2_path() -> Result<Option<String>, String> {
    Ok(detect_ets2_install())
}

#[tauri::command]
pub async fn open_external_dashboard(app: AppHandle) -> Result<(), String> {
    window_manager::open_external_dashboard(&app).map_err(|e| e.to_string())
}

#[tauri::command]
pub async fn close_external_dashboard(app: AppHandle) -> Result<(), String> {
    if let Some(window) = app.get_webview_window(window_manager::EXTERNAL_LABEL) {
        window.close().map_err(|e| e.to_string())?;
    }
    Ok(())
}

/// Toggle the transparent HUD overlay window (Phase 6.5a).
#[tauri::command]
pub async fn toggle_overlay(app: AppHandle) -> Result<(), String> {
    window_manager::toggle_overlay(&app).map_err(|e| e.to_string())
}

#[tauri::command]
pub async fn daemon_status(daemon: State<'_, Arc<DaemonManager>>) -> Result<DaemonStatus, String> {
    Ok(daemon.status())
}

#[tauri::command]
pub async fn daemon_start(daemon: State<'_, Arc<DaemonManager>>) -> Result<DaemonStatus, String> {
    daemon.start()
}

#[tauri::command]
pub async fn daemon_stop(daemon: State<'_, Arc<DaemonManager>>) -> Result<DaemonStatus, String> {
    daemon.stop()
}

#[tauri::command]
pub async fn daemon_restart(daemon: State<'_, Arc<DaemonManager>>) -> Result<DaemonStatus, String> {
    daemon.restart()
}

#[tauri::command]
pub async fn daemon_get_auto_start(app: AppHandle) -> Result<bool, String> {
    Ok(daemon_config::load(&app).auto_start)
}

#[tauri::command]
pub async fn daemon_set_auto_start(app: AppHandle, enabled: bool) -> Result<(), String> {
    daemon_config::save(
        &app,
        &DaemonConfig {
            auto_start: enabled,
        },
    )
}

#[tauri::command]
pub async fn hotkey_get_config(app: AppHandle) -> Result<HotkeyConfig, String> {
    Ok(hotkey_config::load(&app))
}

#[tauri::command]
pub async fn hotkey_set_config(
    app: AppHandle,
    engage: String,
    disengage: String,
) -> Result<(), String> {
    let cfg = HotkeyConfig {
        engage: engage.clone(),
        disengage: disengage.clone(),
    };
    hotkey_config::save(&app, &cfg)?;
    hotkey_manager::apply(&app, &engage, &disengage)
}
