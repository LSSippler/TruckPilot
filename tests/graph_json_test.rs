use std::fs;
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

struct TempGraphFile {
    path: std::path::PathBuf,
}

impl TempGraphFile {
    fn new(name: &str) -> Self {
        Self {
            path: unique_path(name),
        }
    }

    fn path(&self) -> &std::path::Path {
        &self.path
    }
}

impl Drop for TempGraphFile {
    fn drop(&mut self) {
        let _ = fs::remove_file(&self.path);
    }
}

fn unique_path(name: &str) -> std::path::PathBuf {
    let nonce = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    std::env::temp_dir().join(format!(
        "truckpilot_{}_{}_{}.json",
        name,
        std::process::id(),
        nonce
    ))
}

fn write_minimal_graph(path: &std::path::Path) {
    let graph_json = serde_json::json!({
        "meta": {
            "schema_version": "1.0.0",
            "map_name": "test",
            "generated_at": ""
        },
        "nodes": [
            { "uid": 1, "x": 0.0, "y": 0.0, "z": 0.0 },
            { "uid": 2, "x": 100.0, "y": 0.0, "z": 0.0 }
        ],
        "edges": [
            {
                "edge_uid": 1,
                "from_node_uid": 1,
                "to_node_uid": 2,
                "distance_m": 100.0,
                "direction": "forward",
                "lane_count": 1,
                "speed_limit_kmh": 50.0,
                "flags": []
            }
        ]
    });

    fs::write(path, serde_json::to_vec(&graph_json).unwrap()).unwrap();
}

#[test]
fn cli_graph_json_route_found() {
    let graph_file = TempGraphFile::new("graph_json_route_found");
    write_minimal_graph(graph_file.path());

    let output = Command::new(env!("CARGO_BIN_EXE_truckpilot"))
        .arg("--graph-json")
        .arg(graph_file.path())
        .arg("--start")
        .arg("1")
        .arg("--goal")
        .arg("2")
        .arg("--telemetry-disable")
        .output()
        .unwrap();

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        output.status.success(),
        "CLI failed:\nstdout:\n{}\nstderr:\n{}",
        stdout,
        stderr
    );
    assert!(
        stdout.contains("Route found"),
        "expected route output, got:\n{}",
        stdout
    );
}

#[test]
fn cli_graph_json_missing_file_fails() {
    let graph_path = unique_path("graph_json_missing");
    assert!(!graph_path.exists());

    let output = Command::new(env!("CARGO_BIN_EXE_truckpilot"))
        .arg("--graph-json")
        .arg(&graph_path)
        .arg("--start")
        .arg("1")
        .arg("--goal")
        .arg("2")
        .arg("--telemetry-disable")
        .output()
        .unwrap();

    assert!(!output.status.success());
}

#[test]
fn cli_graph_json_invalid_json_fails() {
    let graph_file = TempGraphFile::new("graph_json_invalid");
    fs::write(graph_file.path(), b"this is not json").unwrap();

    let output = Command::new(env!("CARGO_BIN_EXE_truckpilot"))
        .arg("--graph-json")
        .arg(graph_file.path())
        .arg("--start")
        .arg("1")
        .arg("--goal")
        .arg("2")
        .arg("--telemetry-disable")
        .output()
        .unwrap();
    assert!(!output.status.success());
}

#[test]
fn cli_graph_json_empty_graph_fails() {
    let graph_file = TempGraphFile::new("graph_json_empty");
    let graph_json = serde_json::json!({
        "meta": {
            "schema_version": "1.0.0",
            "map_name": "test",
            "generated_at": ""
        },
        "nodes": [],
        "edges": []
    });
    fs::write(graph_file.path(), serde_json::to_vec(&graph_json).unwrap()).unwrap();

    let output = Command::new(env!("CARGO_BIN_EXE_truckpilot"))
        .arg("--graph-json")
        .arg(graph_file.path())
        .arg("--start")
        .arg("1")
        .arg("--goal")
        .arg("2")
        .arg("--telemetry-disable")
        .output()
        .unwrap();
    assert!(!output.status.success());
}

#[test]
fn cli_graph_json_empty_nodes_fails() {
    let graph_file = TempGraphFile::new("graph_json_empty_nodes");
    let graph_json = serde_json::json!({
        "meta": {
            "schema_version": "1.0.0",
            "map_name": "test",
            "generated_at": ""
        },
        "nodes": [],
        "edges": [
            {
                "edge_uid": 1,
                "from_node_uid": 1,
                "to_node_uid": 2,
                "distance_m": 100.0,
                "direction": "forward",
                "lane_count": 1,
                "speed_limit_kmh": 50.0,
                "flags": []
            }
        ]
    });
    fs::write(graph_file.path(), serde_json::to_vec(&graph_json).unwrap()).unwrap();

    let output = Command::new(env!("CARGO_BIN_EXE_truckpilot"))
        .arg("--graph-json")
        .arg(graph_file.path())
        .arg("--start")
        .arg("1")
        .arg("--goal")
        .arg("2")
        .arg("--telemetry-disable")
        .output()
        .unwrap();

    assert!(!output.status.success());
}

#[test]
fn cli_graph_json_has_priority_over_hashfs() {
    let graph_file = TempGraphFile::new("graph_json_priority");
    write_minimal_graph(graph_file.path());

    let output = Command::new(env!("CARGO_BIN_EXE_truckpilot"))
        .arg("--graph-json")
        .arg(graph_file.path())
        .arg("--hashfs-sectors")
        .arg("/definitely/not/existing/path")
        .arg("--start")
        .arg("1")
        .arg("--goal")
        .arg("2")
        .arg("--telemetry-disable")
        .output()
        .unwrap();

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(output.status.success());
    assert!(stdout.contains("Route found"));
}
