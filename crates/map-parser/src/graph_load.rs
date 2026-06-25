//! Safe loading of `graph.json` into [`MapGraph`] with staged logging and validation.
//!
//! Uses streaming JSON parse (`serde_json::from_reader`) — no full-file `String` buffer.

use std::fs::File;
use std::io::{BufReader, Read};
use std::path::{Path, PathBuf};

use crate::graph::MapGraph;

/// Which phase of graph loading failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GraphLoadPhase {
    OpenFile,
    FileSize,
    ParseJson,
    ValidateReferences,
}

impl GraphLoadPhase {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::OpenFile => "open file",
            Self::FileSize => "file size",
            Self::ParseJson => "parse json",
            Self::ValidateReferences => "validate references",
        }
    }
}

/// Error while loading a routing graph from disk.
#[derive(Debug)]
pub struct GraphLoadError {
    pub path: PathBuf,
    pub phase: GraphLoadPhase,
    pub file_size_bytes: Option<u64>,
    pub message: String,
}

impl std::fmt::Display for GraphLoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let size = self
            .file_size_bytes
            .map(|b| format!(" (file size = {:.1} MB)", b as f64 / (1024.0 * 1024.0)))
            .unwrap_or_default();
        write!(
            f,
            "failed to load graph {} during {}: {}{}",
            self.path.display(),
            self.phase.as_str(),
            self.message,
            size
        )
    }
}

impl std::error::Error for GraphLoadError {}

/// Emit a staged progress line: `[{prefix}] graph load: {message}`.
pub fn log_graph_load_stage(prefix: &str, message: &str) {
    eprintln!("[{prefix}] graph load: {message}");
}

fn err(
    path: &Path,
    phase: GraphLoadPhase,
    file_size_bytes: Option<u64>,
    message: impl Into<String>,
) -> GraphLoadError {
    GraphLoadError {
        path: path.to_path_buf(),
        phase,
        file_size_bytes,
        message: message.into(),
    }
}

/// Open `path`, log file size, parse JSON via buffered reader, validate edge endpoints.
pub fn load_map_graph_from_path(path: &Path, log_prefix: &str) -> Result<MapGraph, GraphLoadError> {
    log_graph_load_stage(log_prefix, "open file");
    let file = File::open(path).map_err(|e| {
        err(path, GraphLoadPhase::OpenFile, None, format!("{e}"))
    })?;

    let file_size = file.metadata().map_err(|e| {
        err(path, GraphLoadPhase::FileSize, None, format!("metadata: {e}"))
    })?.len();

    log_graph_load_stage(
        log_prefix,
        &format!("file size = {:.1} MB", file_size as f64 / (1024.0 * 1024.0)),
    );

    log_graph_load_stage(log_prefix, "read/parse json start");
    let reader = BufReader::with_capacity(8 * 1024 * 1024, file);
    let graph: MapGraph = serde_json::from_reader(reader).map_err(|e| {
        err(
            path,
            GraphLoadPhase::ParseJson,
            Some(file_size),
            format!("{e}"),
        )
    })?;
    log_graph_load_stage(log_prefix, "parse json done");

    log_graph_load_stage(
        log_prefix,
        &format!("nodes = {}, edges = {}", graph.nodes.len(), graph.edges.len()),
    );

    warn_dangling_edges(&graph, log_prefix);
    log_graph_load_stage(log_prefix, "validate references done");

    Ok(graph)
}

/// Load graph from bytes (tests / in-memory fixtures).
pub fn load_map_graph_from_bytes(
    path_label: &Path,
    bytes: &[u8],
    log_prefix: &str,
) -> Result<MapGraph, GraphLoadError> {
    log_graph_load_stage(log_prefix, "parse json start (bytes)");
    let graph: MapGraph = serde_json::from_slice(bytes).map_err(|e| {
        err(
            path_label,
            GraphLoadPhase::ParseJson,
            Some(bytes.len() as u64),
            format!("{e}"),
        )
    })?;
    log_graph_load_stage(log_prefix, "parse json done");
    validate_edge_node_references(&graph, path_label, Some(bytes.len() as u64))?;
    Ok(graph)
}

/// Count edges whose endpoints are absent from `nodes`. Real ETS2 graphs may
/// contain a small number of unresolved cross-sector edges; these are skipped
/// when building [`RouterGraph`] / splines rather than aborting daemon startup.
pub fn count_dangling_edges(graph: &MapGraph) -> usize {
    let node_uids: std::collections::HashSet<u64> = graph.nodes.iter().map(|n| n.uid).collect();
    graph
        .edges
        .iter()
        .filter(|e| !node_uids.contains(&e.from) || !node_uids.contains(&e.to))
        .count()
}

/// Reject graphs whose edges reference unknown node UIDs (strict — for tests).
pub fn validate_edge_node_references(
    graph: &MapGraph,
    path: &Path,
    file_size_bytes: Option<u64>,
) -> Result<(), GraphLoadError> {
    if graph.nodes.is_empty() && graph.edges.is_empty() {
        return Ok(());
    }

    let node_uids: std::collections::HashSet<u64> = graph.nodes.iter().map(|n| n.uid).collect();

    for edge in &graph.edges {
        if !node_uids.contains(&edge.from) {
            return Err(err(
                path,
                GraphLoadPhase::ValidateReferences,
                file_size_bytes,
                format!("missing node position for uid {} (edge.from)", edge.from),
            ));
        }
        if !node_uids.contains(&edge.to) {
            return Err(err(
                path,
                GraphLoadPhase::ValidateReferences,
                file_size_bytes,
                format!("missing node position for uid {} (edge.to)", edge.to),
            ));
        }
    }

    if node_uids.len() != graph.nodes.len() {
        let mut seen = std::collections::HashSet::new();
        for n in &graph.nodes {
            if !seen.insert(n.uid) {
                return Err(err(
                    path,
                    GraphLoadPhase::ValidateReferences,
                    file_size_bytes,
                    format!("duplicate node uid {}", n.uid),
                ));
            }
        }
    }

    Ok(())
}

/// Warn about dangling edges after a successful parse (non-fatal).
fn warn_dangling_edges(graph: &MapGraph, log_prefix: &str) {
    let dangling = count_dangling_edges(graph);
    if dangling > 0 {
        log_graph_load_stage(
            log_prefix,
            &format!(
                "warning: {dangling} edges reference missing nodes (will be skipped for routing/splines)"
            ),
        );
    }
}

/// Read entire file into a `Vec<u8>` (for tools that need raw bytes). Prefer
/// [`load_map_graph_from_path`] for production loads.
pub fn read_graph_file_bytes(path: &Path) -> Result<Vec<u8>, GraphLoadError> {
    log_graph_load_stage("graph-load", "open file");
    let mut file = File::open(path).map_err(|e| {
        err(path, GraphLoadPhase::OpenFile, None, format!("{e}"))
    })?;
    let file_size = file.metadata().map_err(|e| {
        err(path, GraphLoadPhase::FileSize, None, format!("metadata: {e}"))
    })?.len();
    log_graph_load_stage(
        "graph-load",
        &format!("file size = {:.1} MB", file_size as f64 / (1024.0 * 1024.0)),
    );
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes).map_err(|e| {
        err(path, GraphLoadPhase::OpenFile, Some(file_size), format!("read: {e}"))
    })?;
    Ok(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn fixture(name: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures")
            .join(name)
    }

    #[test]
    fn tiny_fixture_loads() {
        let path = fixture("tiny_graph.json");
        let graph = load_map_graph_from_path(&path, "test").expect("tiny graph");
        assert_eq!(graph.nodes.len(), 2);
        assert_eq!(graph.edges.len(), 1);
    }

    #[test]
    fn invalid_json_errors_cleanly() {
        let path = fixture("invalid_graph.json");
        let err = load_map_graph_from_path(&path, "test").unwrap_err();
        assert_eq!(err.phase, GraphLoadPhase::ParseJson);
    }

    #[test]
    fn missing_file_errors_cleanly() {
        let path = fixture("does_not_exist.json");
        let err = load_map_graph_from_path(&path, "test").unwrap_err();
        assert_eq!(err.phase, GraphLoadPhase::OpenFile);
    }

    #[test]
    fn edge_to_missing_node_rejected() {
        let path = fixture("bad_edge_graph.json");
        let graph = load_map_graph_from_path(&path, "test").expect("parse bad edge fixture");
        let err = validate_edge_node_references(&graph, &path, None).unwrap_err();
        assert_eq!(err.phase, GraphLoadPhase::ValidateReferences);
        assert!(err.message.contains("missing node"));
    }
}
