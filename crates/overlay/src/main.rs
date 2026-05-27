//! TruckPilot Debug HUD — transparent overlay over ETS2
//!
//! Standalone binary. Lifecycle separate from daemon and UI.
//!
//! Architecture:
//!   - WebSocket client polls daemon at 5 Hz (ws://127.0.0.1:8765)
//!   - Shared HudState (Arc) updated by WS thread, read by render thread
//!   - procmod-overlay renders over ETS2 window at ~30 fps via DX11
//!   - Runs without daemon: shows "DAEMON DISCONNECTED" banner

mod coords;
mod renderer;
mod state;
mod ws_client;

use std::sync::Arc;

use anyhow::Result;
use tracing::info;

use crate::state::HudState;

fn main() -> Result<()> {
    // Tracing init — level controlled by RUST_LOG env var, default INFO
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "overlay=info,warn".parse().unwrap()),
        )
        .init();

    info!("TruckPilot Debug HUD starting");

    let state = HudState::new();

    // Spawn WS client in background tokio runtime
    let state_for_ws = Arc::clone(&state);
    let ws_handle = std::thread::spawn(move || {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .expect("tokio rt");
        rt.block_on(ws_client::run(state_for_ws));
    });

    // Render loop on main thread (Win32 message pump requires it)
    #[cfg(windows)]
    {
        renderer::run(state)?;
    }

    #[cfg(not(windows))]
    {
        tracing::error!("TruckPilot overlay only supports Windows");
        // Keep WS thread alive for smoke-test
        let _ = ws_handle.join();
    }

    let _ = ws_handle;
    Ok(())
}
