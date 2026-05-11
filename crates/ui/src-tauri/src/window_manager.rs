use tauri::{AppHandle, Manager, WebviewUrl, WebviewWindowBuilder};

pub const EXTERNAL_LABEL: &str = "dashboard-window";

pub fn open_external_dashboard(app: &AppHandle) -> tauri::Result<()> {
    if let Some(existing) = app.get_webview_window(EXTERNAL_LABEL) {
        existing.set_focus()?;
        return Ok(());
    }

    let window = WebviewWindowBuilder::new(
        app,
        EXTERNAL_LABEL,
        WebviewUrl::App("/external-dashboard".into()),
    )
    .title("TruckPilot — Dashboard")
    .inner_size(1280.0, 720.0)
    .resizable(true)
    .build()?;

    let _ = window.set_focus();
    Ok(())
}
