//! Read-only overlay snapshot JSON from the telemetry CLI loop file.
//!
//! Matches `truckpilot-status --overlay-loop` default output path. No writes,
//! no process spawn, no SHM, no resolver.

use std::io;
use std::path::{Path, PathBuf};

/// Default path for `truckpilot-status --overlay-loop` (Windows: `%LOCALAPPDATA%`).
pub fn default_overlay_snapshot_path() -> PathBuf {
    #[cfg(windows)]
    if let Ok(local) = std::env::var("LOCALAPPDATA") {
        return PathBuf::from(local)
            .join("TruckPilot")
            .join("overlay_snapshot.json");
    }
    #[cfg(unix)]
    if let Ok(home) = std::env::var("HOME") {
        return PathBuf::from(home)
            .join(".local/share/TruckPilot/overlay_snapshot.json");
    }
    PathBuf::from("overlay_snapshot.json")
}

/// Read snapshot JSON from `path`. Missing or empty file → `Ok(None)`.
pub fn read_overlay_snapshot_file_at(path: &Path) -> Result<Option<String>, String> {
    match std::fs::read_to_string(path) {
        Ok(content) if content.trim().is_empty() => Ok(None),
        Ok(content) => Ok(Some(content)),
        Err(e) if e.kind() == io::ErrorKind::NotFound => Ok(None),
        Err(e) => Err(format!("read overlay snapshot file: {e}")),
    }
}

/// Read snapshot JSON from optional explicit path or [`default_overlay_snapshot_path`].
pub fn read_overlay_snapshot_file(path: Option<String>) -> Result<Option<String>, String> {
    let path = path.map(PathBuf::from).unwrap_or_else(default_overlay_snapshot_path);
    read_overlay_snapshot_file_at(&path)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_path_non_empty() {
        assert!(!default_overlay_snapshot_path().as_os_str().is_empty());
    }

    #[test]
    fn missing_file_returns_none() {
        let path = std::env::temp_dir().join("truckpilot_overlay_missing_bridge_test.json");
        let _ = std::fs::remove_file(&path);
        assert!(read_overlay_snapshot_file_at(&path).unwrap().is_none());
    }

    #[test]
    fn roundtrip_reads_written_json() {
        let dir = std::env::temp_dir().join("truckpilot_overlay_bridge_cmd_test");
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("create temp dir");
        let path = dir.join("overlay_snapshot.json");
        std::fs::write(&path, r#"{"verdict":"unavailable"}"#).expect("write temp json");
        let raw = read_overlay_snapshot_file_at(&path)
            .expect("read")
            .expect("non-empty");
        assert!(raw.contains("unavailable"));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
