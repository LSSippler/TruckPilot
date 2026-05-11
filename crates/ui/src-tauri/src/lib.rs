mod commands;
mod ipc_bridge;
mod steam_detect;
mod window_manager;

use ipc_bridge::IpcBridge;
use tauri::Manager;

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

    tauri::Builder::default()
        .plugin(tauri_plugin_dialog::init())
        .plugin(tauri_plugin_fs::init())
        .plugin(tauri_plugin_shell::init())
        .setup(|app| {
            let bridge = IpcBridge::spawn(app.handle().clone());
            app.manage(bridge);
            Ok(())
        })
        .invoke_handler(tauri::generate_handler![
            commands::send_command,
            commands::get_connection_status,
            commands::reconnect,
            commands::detect_ets2_path,
            commands::open_external_dashboard,
            commands::close_external_dashboard,
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
