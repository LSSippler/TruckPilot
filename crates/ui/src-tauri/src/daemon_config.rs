//! Persisted daemon settings (auto-start toggle).
//!
//! Stored alongside other Tauri app data at
//! `%LOCALAPPDATA%/com.truckpilot.app/daemon.json` on Windows. Read by the
//! Rust setup hook before frontend boot; written by `daemon_set_auto_start`.

use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};
use tauri::{AppHandle, Manager};
use tracing::warn;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DaemonConfig {
    pub auto_start: bool,
}

impl Default for DaemonConfig {
    fn default() -> Self {
        Self { auto_start: true }
    }
}

fn config_path(app: &AppHandle) -> Option<PathBuf> {
    app.path()
        .app_local_data_dir()
        .ok()
        .map(|d| d.join("daemon.json"))
}

pub fn load(app: &AppHandle) -> DaemonConfig {
    let Some(path) = config_path(app) else {
        return DaemonConfig::default();
    };
    load_from(&path).unwrap_or_default()
}

fn load_from(path: &Path) -> Option<DaemonConfig> {
    let raw = std::fs::read_to_string(path).ok()?;
    match serde_json::from_str::<DaemonConfig>(&raw) {
        Ok(cfg) => Some(cfg),
        Err(err) => {
            warn!("daemon.json parse failed: {err}");
            None
        }
    }
}

pub fn save(app: &AppHandle, cfg: &DaemonConfig) -> Result<(), String> {
    let Some(path) = config_path(app) else {
        return Err("app_local_data_dir unavailable".into());
    };
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).map_err(|e| format!("create config dir: {e}"))?;
    }
    let json = serde_json::to_string_pretty(cfg).map_err(|e| e.to_string())?;
    std::fs::write(&path, json).map_err(|e| format!("write daemon.json: {e}"))
}
