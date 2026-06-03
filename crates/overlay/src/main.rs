//! TruckPilot AR + Debug HUD — transparent overlay over ETS2
//!
//! Architecture:
//!   - WebSocket client   → polls daemon at 5 Hz (bias/segments/lane data)
//!   - AR telemetry thread → reads SHM directly at 60 Hz (truck pose)
//!   - Render loop        → procmod-overlay DX11, 60 fps
//!
//! ## Modes (F1 to cycle)
//!   Minimap  — classic 480×480 HUD panel top-right
//!   AR       — world-anchored road lines, full-screen projection
//!   Both     — AR lines + minimap panel simultaneously
//!
//! ## Hotkeys
//!   F1  cycle mode  |  F2  toggle visibility  |  F3  FOV calibration wizard

mod ar_renderer;
mod colors;
mod config_reader;
mod coords;
mod diag;
mod projection;
mod renderer;
mod state;
mod telemetry;
mod ws_client;

use std::sync::Arc;

use anyhow::Result;
use tracing::info;

use crate::state::HudState;

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "overlay=info,warn".parse().unwrap()),
        )
        .init();

    info!("TruckPilot AR HUD starting");

    // ── Shared state ──────────────────────────────────────────────────────────
    let hud_state = HudState::new();
    let ar_pose = telemetry::spawn_telemetry_thread();

    // ── WebSocket client (bias / segments / lane data) ────────────────────────
    let state_for_ws = Arc::clone(&hud_state);
    let _ws_handle = std::thread::spawn(move || {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("tokio rt");
        rt.block_on(ws_client::run(state_for_ws));
    });

    // ── Render loop (Win32 message pump must be on main thread) ───────────────
    #[cfg(windows)]
    {
        renderer::run(hud_state, ar_pose)?;
    }

    #[cfg(not(windows))]
    {
        tracing::error!("TruckPilot overlay only supports Windows");
        let _ = _ws_handle.join();
    }

    Ok(())
}
