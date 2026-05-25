use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};
use tauri::{AppHandle, Manager};
use tracing::warn;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HotkeyConfig {
    pub engage: String,
    pub disengage: String,
}

impl Default for HotkeyConfig {
    fn default() -> Self {
        Self {
            engage: "NumpadEnter".to_string(),
            disengage: "NumpadDecimal".to_string(),
        }
    }
}

fn config_path(app: &AppHandle) -> Option<PathBuf> {
    app.path()
        .app_local_data_dir()
        .ok()
        .map(|d| d.join("hotkeys.json"))
}

pub fn load(app: &AppHandle) -> HotkeyConfig {
    let Some(path) = config_path(app) else {
        return HotkeyConfig::default();
    };
    load_from(&path).unwrap_or_default()
}

fn load_from(path: &Path) -> Option<HotkeyConfig> {
    let raw = std::fs::read_to_string(path).ok()?;
    match serde_json::from_str::<HotkeyConfig>(&raw) {
        Ok(cfg) => Some(cfg),
        Err(err) => {
            warn!("hotkeys.json parse failed: {err}");
            None
        }
    }
}

pub fn save(app: &AppHandle, cfg: &HotkeyConfig) -> Result<(), String> {
    let Some(path) = config_path(app) else {
        return Err("app_local_data_dir unavailable".into());
    };
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).map_err(|e| format!("create config dir: {e}"))?;
    }
    let json = serde_json::to_string_pretty(cfg).map_err(|e| e.to_string())?;
    std::fs::write(&path, json).map_err(|e| format!("write hotkeys.json: {e}"))
}
