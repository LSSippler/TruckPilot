//! Integration tests validating the repo-root truckpilot.toml and config fallbacks.

use std::path::Path;
use truckpilot::config::TruckPilotConfig;

#[test]
fn test_truckpilot_toml_exists_at_repo_root() {
    let path = Path::new("truckpilot.toml");
    assert!(path.exists(), "truckpilot.toml must exist at repo root");
}

#[test]
fn test_truckpilot_toml_parses_with_expected_routing_defaults() {
    let cfg = TruckPilotConfig::try_load_from_file("truckpilot.toml")
        .expect("truckpilot.toml must parse without errors");

    assert_eq!(
        cfg.routing.cost_mode, "distance",
        "repo-root truckpilot.toml should default routing.cost_mode to 'distance'"
    );
    assert!(
        !cfg.routing.prefer_speed,
        "repo-root truckpilot.toml should default routing.prefer_speed to false"
    );
    assert!(
        cfg.routing.smooth_route,
        "repo-root truckpilot.toml should default routing.smooth_route to true"
    );
    assert_eq!(
        cfg.routing.subdivisions, 4,
        "repo-root truckpilot.toml should default routing.subdivisions to 4"
    );
}

#[test]
fn test_truckpilot_toml_override_routing_values() {
    let dir = std::env::temp_dir().join("truckpilot_tests");
    std::fs::create_dir_all(&dir).expect("create temp dir");
    let path = dir.join("truckpilot_override.toml");
    let data = r#"
[routing]
cost_mode = "eta"
prefer_speed = true
"#;
    std::fs::write(&path, data).expect("write temp config");

    let cfg = TruckPilotConfig::load_from_file(path.to_string_lossy().as_ref());
    assert_eq!(cfg.routing.cost_mode, "eta");
    assert!(cfg.routing.prefer_speed);

    let _ = std::fs::remove_file(path);
}
