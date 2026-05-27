//! TASK 2 — Read ETS2 config.cfg for horizontal FOV.
//!
//! Also reads `truckpilot.toml` [overlay] section for persistent overrides.

use std::{env, fs};

use tracing::debug;

/// Default horizontal FOV in degrees when nothing else provides a value.
pub const DEFAULT_FOV_H: f32 = 75.0;

/// Read `r_multimon_fov_horizontal` from the ETS2 user config.
/// Returns `None` when the file doesn't exist or the key isn't present.
pub fn read_ets2_fov() -> Option<f32> {
    let user = env::var("USERPROFILE").ok()?;
    let path = format!(
        "{}/Documents/Euro Truck Simulator 2/config.cfg",
        user
    );
    let content = fs::read_to_string(&path)
        .map_err(|e| debug!("config.cfg read failed: {e}"))
        .ok()?;

    for line in content.lines() {
        if line.contains("r_multimon_fov_horizontal") {
            return parse_quoted_float(line);
        }
    }
    None
}

/// Read `[overlay] fov_h_deg` from `truckpilot.toml` in the current directory.
pub fn read_toml_fov() -> Option<f32> {
    let content = fs::read_to_string("truckpilot.toml")
        .map_err(|_| ())
        .ok()?;
    let doc: toml::Value = content.parse().ok()?;
    doc.get("overlay")?.get("fov_h_deg")?.as_float().map(|v| v as f32)
}

/// Write back the calibrated FOV to `truckpilot.toml`.
pub fn save_toml_fov(fov: f32) -> anyhow::Result<()> {
    // Read existing TOML or start fresh.
    let mut doc: toml::Table = if let Ok(content) = fs::read_to_string("truckpilot.toml") {
        content.parse().unwrap_or_default()
    } else {
        toml::Table::new()
    };

    let overlay = doc
        .entry("overlay")
        .or_insert_with(|| toml::Value::Table(toml::Table::new()));

    if let toml::Value::Table(t) = overlay {
        t.insert("fov_h_deg".into(), toml::Value::Float(fov as f64));
    }

    fs::write("truckpilot.toml", toml::to_string(&doc)?)?;
    Ok(())
}

/// Resolve effective FOV: TOML override → ETS2 config.cfg → default 75°.
pub fn effective_fov() -> f32 {
    if let Some(f) = read_toml_fov() {
        debug!("FOV from truckpilot.toml: {f}°");
        return f;
    }
    if let Some(f) = read_ets2_fov() {
        debug!("FOV from ETS2 config.cfg: {f}°");
        return f;
    }
    debug!("FOV fallback: {DEFAULT_FOV_H}°");
    DEFAULT_FOV_H
}

/// Parse the quoted float in lines like: `uset r_multimon_fov_horizontal "70"`
fn parse_quoted_float(line: &str) -> Option<f32> {
    let start = line.rfind('"')
        .and_then(|end| line[..end].rfind('"').map(|s| (s + 1, end)))?;
    line[start.0..start.1].parse().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fov_reader_from_synthetic_config() {
        let line = r#"uset r_multimon_fov_horizontal "70""#;
        let fov = parse_quoted_float(line);
        assert_eq!(fov, Some(70.0_f32));
    }

    #[test]
    fn test_fov_reader_other_value() {
        let line = r#"uset r_multimon_fov_horizontal "90""#;
        assert_eq!(parse_quoted_float(line), Some(90.0_f32));
    }

    #[test]
    fn test_parse_no_quotes_returns_none() {
        assert_eq!(parse_quoted_float("no quotes here"), None);
    }
}
