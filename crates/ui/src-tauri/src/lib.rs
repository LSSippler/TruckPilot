mod commands;
mod daemon;
mod daemon_config;
mod hotkey_config;
mod hotkey_manager;
mod ipc_bridge;
mod overlay_snap;
mod steam_detect;
mod window_manager;

use std::sync::Arc;

use ipc_bridge::IpcBridge;
use tauri::{Manager, RunEvent};
use tracing::warn;

use crate::daemon::DaemonManager;

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| {
                tracing_subscriber::EnvFilter::new("info,tokio_tungstenite=warn")
            }),
        )
        .try_init()
        .ok();

    let daemon = Arc::new(DaemonManager::new());

    tauri::Builder::default()
        .plugin(tauri_plugin_dialog::init())
        .plugin(tauri_plugin_fs::init())
        .plugin(tauri_plugin_global_shortcut::Builder::new().build())
        .plugin(tauri_plugin_shell::init())
        .manage(daemon.clone())
        .setup(move |app| {
            let bridge = IpcBridge::spawn(app.handle().clone());
            app.manage(bridge);

            // Daemon auto-start is handled by the main UI window (App.tsx) with a
            // delay when enabled in Settings — not here. Spawning during setup caused
            // graph.json + plugin on_load spikes while ETS2 was in the foreground.

            let hk = hotkey_config::load(app.handle());
            if let Err(e) = hotkey_manager::apply(app.handle(), &hk.engage, &hk.disengage) {
                warn!("global hotkey registration failed: {e}");
            }
            Ok(())
        })
        .invoke_handler(tauri::generate_handler![
            commands::send_command,
            commands::get_connection_status,
            commands::reconnect,
            commands::detect_ets2_path,
            commands::open_external_dashboard,
            commands::close_external_dashboard,
            commands::toggle_overlay,
            commands::overlay_set_layout_editor,
            commands::daemon_status,
            commands::daemon_start,
            commands::daemon_stop,
            commands::daemon_restart,
            commands::daemon_get_auto_start,
            commands::daemon_set_auto_start,
            commands::hotkey_get_config,
            commands::hotkey_set_config,
        ])
        .build(tauri::generate_context!())
        .expect("error while building tauri application")
        .run(move |_app, event| {
            if let RunEvent::Exit = event {
                daemon.shutdown_for_exit();
            }
        });
}
