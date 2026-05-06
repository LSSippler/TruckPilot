//! Runtime configuration loaded from `truckpilot.toml`.
//!
//! Parser is intentionally minimal (section + key/value) to avoid extra
//! dependencies. Unknown keys are ignored.

/// Top-level configuration loaded from `truckpilot.toml`.
#[derive(Debug, Clone, Default)]
pub struct TruckPilotConfig {
    /// Steering PID configuration (`[steering]` section).
    pub steering: SteeringConfig,
    /// Speed PID configuration (`[speed]` section).
    pub speed: SpeedConfig,
    /// Routing/cost-mode configuration (`[routing]` section).
    pub routing: RoutingConfig,
    /// Telemetry transport configuration (`[telemetry]` section).
    pub telemetry: TelemetryConfig,
    /// Adaptive cruise control configuration (`[acc]` section).
    pub acc: AccConfig,
}

impl TruckPilotConfig {
    /// Load `truckpilot.toml` from the current working directory.
    ///
    /// Returns the default configuration with a warning if the file cannot
    /// be read or parsed — the autopilot is expected to start with sensible
    /// fallbacks even without a config file.
    pub fn load() -> Self {
        match Self::try_load_from_file("truckpilot.toml") {
            Ok(cfg) => cfg,
            Err(e) => {
                eprintln!("Warning: could not load config 'truckpilot.toml': {e}. Using defaults.");
                Self::default()
            }
        }
    }

    /// Like [`Self::load`], but reads from a caller-supplied path.
    pub fn load_from_file(path: &str) -> Self {
        match Self::try_load_from_file(path) {
            Ok(cfg) => cfg,
            Err(e) => {
                eprintln!("Warning: could not load config '{path}': {e}. Using defaults.");
                Self::default()
            }
        }
    }

    /// Try to read and parse a TOML config file at `path`.
    ///
    /// Returns an error string on I/O failure. Unknown keys/sections are
    /// silently ignored.
    pub fn try_load_from_file(path: &str) -> Result<Self, String> {
        let contents = std::fs::read_to_string(path).map_err(|e| format!("{e}"))?;

        let mut cfg = Self::default();
        let mut section = String::new();

        for raw_line in contents.lines() {
            let line = strip_unquoted_comment(raw_line).trim();
            if line.is_empty() {
                continue;
            }

            if line.starts_with('[') && line.ends_with(']') {
                section = line[1..line.len() - 1].trim().to_lowercase();
                continue;
            }

            let Some((key, value)) = line.split_once('=') else {
                continue;
            };

            let key = key.trim().to_lowercase();
            let value = value.trim().trim_matches('"');

            match section.as_str() {
                "steering" => apply_steering(&mut cfg.steering, &key, value),
                "speed" => apply_speed(&mut cfg.speed, &key, value),
                "routing" => apply_routing(&mut cfg.routing, &key, value),
                "telemetry" => apply_telemetry(&mut cfg.telemetry, &key, value),
                "acc" => apply_acc(&mut cfg.acc, &key, value),
                _ => {}
            }
        }

        Ok(cfg)
    }
}

fn strip_unquoted_comment(line: &str) -> &str {
    let mut in_quotes = false;
    let mut escaped = false;

    for (idx, ch) in line.char_indices() {
        if escaped {
            escaped = false;
            continue;
        }

        match ch {
            '\\' if in_quotes => escaped = true,
            '"' => in_quotes = !in_quotes,
            '#' if !in_quotes => return &line[..idx],
            _ => {}
        }
    }

    line
}

/// Steering PID controller settings.
#[derive(Debug, Clone)]
pub struct SteeringConfig {
    /// Proportional gain.
    pub kp: f64,
    /// Integral gain.
    pub ki: f64,
    /// Derivative gain.
    pub kd: f64,
    /// Anti-windup limit on the integral term.
    pub integral_limit: f64,
    /// Pure-pursuit look-ahead distance in meters.
    pub look_ahead_distance: f64,
}

impl Default for SteeringConfig {
    fn default() -> Self {
        Self {
            // Keep identical to previously hardcoded loop defaults.
            kp: 0.8,
            ki: 0.1,
            kd: 0.3,
            integral_limit: 2.0,
            look_ahead_distance: 50.0,
        }
    }
}

/// Speed PID controller settings.
#[derive(Debug, Clone)]
pub struct SpeedConfig {
    /// Proportional gain.
    pub kp: f64,
    /// Integral gain.
    pub ki: f64,
    /// Derivative gain.
    pub kd: f64,
    /// Anti-windup limit on the integral term.
    pub integral_limit: f64,
    /// Speed (km/h) to use when no edge speed-limit is known.
    pub fallback_speed_kmh: f64,
}

impl Default for SpeedConfig {
    fn default() -> Self {
        Self {
            // Keep identical to SpeedController::default_tuning + loop fallback.
            kp: 0.5,
            ki: 0.1,
            kd: 0.05,
            integral_limit: 10.0,
            fallback_speed_kmh: 80.0,
        }
    }
}

/// Routing / planner configuration.
#[derive(Debug, Clone)]
pub struct RoutingConfig {
    /// Cost-mode selector — `"distance"` (default) or `"eta"`.
    pub cost_mode: String,
    /// Bias the planner towards higher speed limits.
    pub prefer_speed: bool,
    /// Apply route smoothing after A* search.
    pub smooth_route: bool,
    /// Number of intermediate points per smoothed segment.
    pub subdivisions: usize,
}

impl Default for RoutingConfig {
    fn default() -> Self {
        Self {
            cost_mode: "distance".to_string(),
            prefer_speed: false,
            smooth_route: true,
            subdivisions: 4,
        }
    }
}

/// Telemetry transport configuration.
#[derive(Debug, Clone)]
pub struct TelemetryConfig {
    /// HTTP endpoint of the telemetry-server-style provider (legacy).
    pub url: String,
    /// Disable HTTP telemetry entirely (use SHM only).
    pub disabled: bool,
}

/// Adaptive cruise control configuration.
#[derive(Debug, Clone)]
pub struct AccConfig {
    /// Enable ACC.
    pub enabled: bool,
    /// Target following distance in meters.
    pub target_distance_m: f32,
    /// Proportional gain.
    pub kp: f64,
    /// Integral gain.
    pub ki: f64,
    /// Derivative gain.
    pub kd: f64,
}

impl Default for AccConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            target_distance_m: 50.0,
            kp: 0.8,
            ki: 0.02,
            kd: 0.2,
        }
    }
}

impl Default for TelemetryConfig {
    fn default() -> Self {
        Self {
            url: "http://localhost:25555/api/ets2/telemetry".to_string(),
            disabled: false,
        }
    }
}

fn parse_bool(v: &str) -> Option<bool> {
    match v.trim().to_lowercase().as_str() {
        "true" | "1" | "yes" | "on" => Some(true),
        "false" | "0" | "no" | "off" => Some(false),
        _ => None,
    }
}

fn apply_steering(cfg: &mut SteeringConfig, key: &str, value: &str) {
    match key {
        "kp" => cfg.kp = value.parse().unwrap_or(cfg.kp),
        "ki" => cfg.ki = value.parse().unwrap_or(cfg.ki),
        "kd" => cfg.kd = value.parse().unwrap_or(cfg.kd),
        "integral_limit" => cfg.integral_limit = value.parse().unwrap_or(cfg.integral_limit),
        "look_ahead_distance" => {
            cfg.look_ahead_distance = value.parse().unwrap_or(cfg.look_ahead_distance)
        }
        _ => {}
    }
}

fn apply_speed(cfg: &mut SpeedConfig, key: &str, value: &str) {
    match key {
        "kp" => cfg.kp = value.parse().unwrap_or(cfg.kp),
        "ki" => cfg.ki = value.parse().unwrap_or(cfg.ki),
        "kd" => cfg.kd = value.parse().unwrap_or(cfg.kd),
        "integral_limit" => cfg.integral_limit = value.parse().unwrap_or(cfg.integral_limit),
        "fallback_speed_kmh" => {
            cfg.fallback_speed_kmh = value.parse().unwrap_or(cfg.fallback_speed_kmh)
        }
        _ => {}
    }
}

fn apply_routing(cfg: &mut RoutingConfig, key: &str, value: &str) {
    match key {
        "cost_mode" => cfg.cost_mode = value.to_string(),
        "prefer_speed" => cfg.prefer_speed = parse_bool(value).unwrap_or(cfg.prefer_speed),
        "smooth_route" => cfg.smooth_route = parse_bool(value).unwrap_or(cfg.smooth_route),
        "subdivisions" => {
            let parsed = value.parse::<usize>().unwrap_or(cfg.subdivisions);
            cfg.subdivisions = parsed.clamp(1, 1000);
        }
        _ => {}
    }
}

fn apply_telemetry(cfg: &mut TelemetryConfig, key: &str, value: &str) {
    match key {
        "url" | "server_url" => cfg.url = value.to_string(),
        "disabled" => cfg.disabled = parse_bool(value).unwrap_or(cfg.disabled),
        _ => {}
    }
}

fn apply_acc(cfg: &mut AccConfig, key: &str, value: &str) {
    match key {
        "enabled" => cfg.enabled = parse_bool(value).unwrap_or(cfg.enabled),
        "target_distance_m" => {
            let parsed = value.parse::<f32>().unwrap_or(cfg.target_distance_m);
            cfg.target_distance_m = parsed.max(1.0);
        }
        "kp" => cfg.kp = value.parse().unwrap_or(cfg.kp),
        "ki" => cfg.ki = value.parse().unwrap_or(cfg.ki),
        "kd" => cfg.kd = value.parse().unwrap_or(cfg.kd),
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_config_integration() {
        let dir = std::env::temp_dir().join("truckpilot_tests");
        std::fs::create_dir_all(&dir).expect("create temp dir");
        let path = dir.join("truckpilot_test_config.toml");
        let data = r#"
[steering]
kp = 0.9
ki = 0.2
kd = 0.4
look_ahead_distance = 65.0

[routing]
smooth_route = true
subdivisions = 6

[speed]
fallback_speed_kmh = 70.0

[telemetry]
url = "http://127.0.0.1:25555/api/ets2/telemetry"

[acc]
enabled = true
target_distance_m = 60.0
kp = 1.0
ki = 0.05
kd = 0.3
"#;
        std::fs::write(&path, data).expect("write temp config");

        let cfg = TruckPilotConfig::load_from_file(path.to_string_lossy().as_ref());
        assert!((cfg.steering.kp - 0.9).abs() < 1e-9);
        assert!((cfg.steering.ki - 0.2).abs() < 1e-9);
        assert!((cfg.steering.kd - 0.4).abs() < 1e-9);
        assert!((cfg.steering.look_ahead_distance - 65.0).abs() < 1e-9);
        assert!(cfg.routing.smooth_route);
        assert_eq!(cfg.routing.subdivisions, 6);
        assert!((cfg.speed.fallback_speed_kmh - 70.0).abs() < 1e-9);
        assert_eq!(
            cfg.telemetry.url,
            "http://127.0.0.1:25555/api/ets2/telemetry"
        );
        assert!(cfg.acc.enabled);
        assert!((cfg.acc.target_distance_m - 60.0).abs() < f32::EPSILON);
        assert!((cfg.acc.kp - 1.0).abs() < 1e-9);
        assert!((cfg.acc.ki - 0.05).abs() < 1e-9);
        assert!((cfg.acc.kd - 0.3).abs() < 1e-9);

        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn test_routing_cost_mode_and_prefer_speed_parsing() {
        let dir = std::env::temp_dir().join("truckpilot_tests");
        std::fs::create_dir_all(&dir).expect("create temp dir");
        let path = dir.join("truckpilot_routing_test.toml");
        let data = r#"
[routing]
cost_mode = "eta"
prefer_speed = true
smooth_route = false
subdivisions = 8
"#;
        std::fs::write(&path, data).expect("write temp config");

        let cfg = TruckPilotConfig::load_from_file(path.to_string_lossy().as_ref());
        assert_eq!(cfg.routing.cost_mode, "eta");
        assert!(cfg.routing.prefer_speed);
        assert!(!cfg.routing.smooth_route);
        assert_eq!(cfg.routing.subdivisions, 8);

        let _ = std::fs::remove_file(path);
    }

    #[test]
    fn test_routing_config_defaults() {
        let cfg = RoutingConfig::default();
        assert_eq!(cfg.cost_mode, "distance");
        assert!(!cfg.prefer_speed);
        assert!(cfg.smooth_route);
        assert_eq!(cfg.subdivisions, 4);
    }

    #[test]
    fn test_hash_in_quoted_value_is_preserved() {
        let dir = std::env::temp_dir().join("truckpilot_tests");
        std::fs::create_dir_all(&dir).expect("create temp dir");
        let path = dir.join("truckpilot_hash_in_url.toml");
        let data = r#"
[telemetry]
url = "http://localhost/api#fragment" # trailing comment
"#;
        std::fs::write(&path, data).expect("write temp config");

        let cfg = TruckPilotConfig::load_from_file(path.to_string_lossy().as_ref());
        assert_eq!(cfg.telemetry.url, "http://localhost/api#fragment");

        let _ = std::fs::remove_file(path);
    }
}
