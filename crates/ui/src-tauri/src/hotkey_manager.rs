use tauri::{AppHandle, Manager};
use tauri_plugin_global_shortcut::{GlobalShortcutExt, ShortcutState};
use truckpilot_ipc_protocol::UiCommand;

use crate::ipc_bridge::IpcBridge;

/// Unregister all current global shortcuts and register engage/disengage with new keys.
/// Called once during setup and again whenever the user changes the config.
pub fn apply(app: &AppHandle, engage: &str, disengage: &str) -> Result<(), String> {
    if engage == disengage {
        return Err("Engage and disengage hotkeys must be different".into());
    }

    let gs = app.global_shortcut();
    gs.unregister_all()
        .map_err(|e| format!("unregister_all: {e}"))?;

    {
        let bridge = app.state::<IpcBridge>().inner().clone();
        gs.on_shortcut(engage, move |_app, _shortcut, event| {
            if event.state == ShortcutState::Pressed {
                let b = bridge.clone();
                tauri::async_runtime::spawn(async move {
                    let _ = b.send(UiCommand::AutopilotEngage).await;
                });
            }
        })
        .map_err(|e| format!("register engage '{engage}': {e}"))?;
    }

    {
        let bridge = app.state::<IpcBridge>().inner().clone();
        gs.on_shortcut(disengage, move |_app, _shortcut, event| {
            if event.state == ShortcutState::Pressed {
                let b = bridge.clone();
                tauri::async_runtime::spawn(async move {
                    let _ = b.send(UiCommand::AutopilotDisengage).await;
                });
            }
        })
        .map_err(|e| format!("register disengage '{disengage}': {e}"))?;
    }

    Ok(())
}
