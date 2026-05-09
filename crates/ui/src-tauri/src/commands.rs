use tauri::{AppHandle, Manager, State};
use truckpilot_ipc_protocol::UiCommand;

use crate::ipc_bridge::{ConnectionStatusEvent, IpcBridge};
use crate::steam_detect::detect_ets2_install;
use crate::window_manager;

#[tauri::command]
pub async fn send_command(bridge: State<'_, IpcBridge>, cmd: UiCommand) -> Result<(), String> {
    bridge.send(cmd).await
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
