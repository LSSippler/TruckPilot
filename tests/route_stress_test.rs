//! Integration test for the `route_stress` binary.

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
fn test_route_stress_runs() {
    ensure_built("route_stress");

    let bin = cargo_bin("route_stress");
    assert!(
        bin.exists(),
        "route_stress binary not built at {}",
        bin.display()
    );

    let mut graph_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    graph_path.push("output");
    graph_path.push("graph.json");

    if !graph_path.exists() {
        eprintln!(
            "skipping test_route_stress_runs: {} not present",
            graph_path.display()
        );
        return;
    }

    let output = Command::new(&bin)
        .arg(&graph_path)
        .output()
        .expect("failed to launch route_stress");

    assert!(
        output.status.success(),
        "route_stress exited non-zero: {}\nstderr: {}",
        output.status,
        String::from_utf8_lossy(&output.stderr)
    );

    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        stdout.contains("Route stress test"),
        "missing header: {stdout}"
    );
    assert!(
        stdout.contains("Pairs evaluated"),
        "missing pairs line: {stdout}"
    );
    assert!(
        stdout.contains("Planning time"),
        "missing planning section: {stdout}"
    );
    assert!(
        stdout.contains("Nodes expanded"),
        "missing nodes section: {stdout}"
    );
    assert!(
        stdout.contains("Top 10 hardest routes"),
        "missing top-10 section: {stdout}"
    );
}
