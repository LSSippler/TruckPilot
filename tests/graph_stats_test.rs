//! Integration test for the `graph_stats` binary.
//!
//! Verifies that the binary runs successfully against `output/graph.json`
//! and prints the expected section headers.

use std::path::PathBuf;
use std::process::Command;

fn cargo_bin(name: &str) -> PathBuf {
    let mut p = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    p.push("target");
    p.push(if cfg!(debug_assertions) {
        "debug"
    } else {
        "release"
    });
    p.push(name);
    p
}

fn ensure_built(bin: &str) {
    let status = Command::new(env!("CARGO"))
        .args(["build", "--quiet", "--bin", bin])
        .current_dir(env!("CARGO_MANIFEST_DIR"))
        .status()
        .expect("cargo build failed");
    assert!(status.success(), "cargo build for `{bin}` exited non-zero");
}

#[test]
fn test_graph_stats_runs() {
    ensure_built("graph_stats");

    let bin = cargo_bin("graph_stats");
    assert!(
        bin.exists(),
        "graph_stats binary not built at {}",
        bin.display()
    );

    let mut graph_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    graph_path.push("output");
    graph_path.push("graph.json");

    if !graph_path.exists() {
        eprintln!(
            "skipping test_graph_stats_runs: {} not present",
            graph_path.display()
        );
        return;
    }

    let output = Command::new(&bin)
        .arg(&graph_path)
        .output()
        .expect("failed to launch graph_stats");

    assert!(
        output.status.success(),
        "graph_stats exited non-zero: {}\nstderr: {}",
        output.status,
        String::from_utf8_lossy(&output.stderr)
    );

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        stdout.contains("Graph statistics"),
        "missing header: {stdout}"
    );
    assert!(
        stdout.contains("Nodes total"),
        "missing nodes line: {stdout}"
    );
    assert!(
        stdout.contains("Edges total"),
        "missing edges line: {stdout}"
    );
    assert!(
        stdout.contains("Edge length"),
        "missing edge length section: {stdout}"
    );
    assert!(
        stdout.contains("Edge direction distribution"),
        "missing direction section: {stdout}"
    );
    assert!(
        stdout.contains("Speed limit distribution"),
        "missing speed section: {stdout}"
    );
}
