use tauri::{AppHandle, Manager, WebviewUrl, WebviewWindowBuilder};

use crate::overlay_snap;

pub const EXTERNAL_LABEL: &str = "dashboard-window";
pub const OVERLAY_LABEL: &str = "overlay";

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

/// Open the transparent, click-through HUD overlay window (Phase 6.5a).
///
/// Frameless, always-on-top, skips the taskbar and never takes focus. Cursor
/// events are ignored so clicks pass through to ETS2 underneath. A background
/// snap loop keeps it aligned with the ETS2 window (Windows only; no-op
/// elsewhere). Idempotent: focuses the existing overlay if already open.
pub fn open_overlay(app: &AppHandle) -> tauri::Result<()> {
    if let Some(existing) = app.get_webview_window(OVERLAY_LABEL) {
        existing.set_focus()?;
        return Ok(());
    }

    let window = WebviewWindowBuilder::new(app, OVERLAY_LABEL, WebviewUrl::App("/overlay".into()))
        .title("TruckPilot Overlay")
        .transparent(true)
        .decorations(false)
        .always_on_top(true)
        .skip_taskbar(true)
        .focused(false)
        .resizable(false)
        .shadow(false)
        // Start hidden: the snap loop only shows it while actively driving in
        // ETS2 (foreground + not paused), so it never flashes elsewhere.
        .visible(false)
        .inner_size(1280.0, 800.0)
        .build()?;

    // Click-through: cursor events fall through to the game window underneath.
    window.set_ignore_cursor_events(true)?;

    // Keep the overlay glued to the ETS2 window (Windows only; no-op otherwise).
    overlay_snap::start(app.clone(), OVERLAY_LABEL);
    Ok(())
}

pub fn close_overlay(app: &AppHandle) -> tauri::Result<()> {
    if let Some(window) = app.get_webview_window(OVERLAY_LABEL) {
        window.close()?;
    }
    Ok(())
}

/// Toggle the overlay: open it if closed, close it if open.
pub fn toggle_overlay(app: &AppHandle) -> tauri::Result<()> {
    if app.get_webview_window(OVERLAY_LABEL).is_some() {
        close_overlay(app)
    } else {
        open_overlay(app)
    }
}

/// Layout editor needs mouse input on the overlay webview (drag panels, viz zoom).
pub fn set_overlay_layout_editor(app: &AppHandle, enabled: bool) -> tauri::Result<()> {
    if let Some(window) = app.get_webview_window(OVERLAY_LABEL) {
        window.set_ignore_cursor_events(!enabled)?;
    }
    Ok(())
}
