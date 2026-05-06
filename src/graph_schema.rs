//! Schema definitions for the TruckPilot road network graph and its export formats.
//!
//! This module defines all data structures used throughout the project:
//! - Core graph types (`GraphData`, `GraphNode`, `GraphEdge`)
//! - Quality assurance types (`QualityMeta`, `QualityReport`, `GraphMetrics`)
//! - Compatibility export types (`CompatNode`, `CompatRoad`, `CompatRoadLook`, `CompatGraph`)

use serde::{Deserialize, Serialize};

/// The complete road network graph.
///
/// Contains metadata and the topological representation as nodes and edges.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GraphData {
    /// Version and provenance metadata.
    pub meta: QualityMeta,
    /// All graph nodes (intersections, endpoints, prefab connectors).
    pub nodes: Vec<GraphNode>,
    /// All directed edges connecting nodes.
    pub edges: Vec<GraphEdge>,
}

/// A single node in the road network graph.
///
/// Represents a point in world space (ETS2 coordinate system) with a unique identifier.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct GraphNode {
    /// Unique node identifier.
    pub uid: u64,
    /// Optional lane-subnode identifier (e.g. `0xABC_lane_1`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lane_uid: Option<String>,
    /// Optional base node UID for lane sub-nodes.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub base_uid: Option<u64>,
    /// Optional lane index for lane sub-nodes.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub lane_index: Option<u32>,
    /// X coordinate in ETS2 world space.
    pub x: f64,
    /// Y coordinate in ETS2 world space (height).
    pub y: f64,
    /// Z coordinate in ETS2 world space.
    pub z: f64,
}

/// A directed edge in the road network graph.
///
/// Connects two nodes and carries road-specific attributes.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct GraphEdge {
    /// Deterministic edge identifier (SipHash over `from|to|road_uid|direction`).
    pub edge_uid: u64,
    /// Source node UID.
    pub from_node_uid: u64,
    /// Target node UID.
    pub to_node_uid: u64,
    /// Optional source lane-subnode identifier.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub from_lane_uid: Option<String>,
    /// Optional target lane-subnode identifier.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub to_lane_uid: Option<String>,
    /// Associated road UID, if this edge belongs to a road item.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub road_uid: Option<String>,
    /// Length of this edge segment in meters.
    pub distance_m: f64,
    /// Direction class (e.g. "forward", "backward", "bidirectional_unknown", "prefab_interconnect").
    pub direction: String,
    /// Number of lanes on this edge.
    pub lane_count: u32,
    /// Speed limit in km/h, if known.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub speed_limit_kmh: Option<f64>,
    /// Arbitrary flags describing road properties (e.g. "no_lanes_unknown", "toll", "oneway").
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub flags: Vec<String>,
}

/// Metadata describing the origin and version of a graph export.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QualityMeta {
    /// Schema version identifier (e.g. "1.0.0").
    pub schema_version: String,
    /// Human-readable name of the source map (e.g. "ets2_base_1.53").
    pub map_name: String,
    /// ISO 8601 timestamp of graph generation (may be empty for deterministic exports).
    #[serde(default)]
    pub generated_at: String,
}

/// A quality report bundling metadata with computed metrics.
///
/// Written to `quality_report.json` when `--quality-report` is active.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QualityReport {
    /// Origin metadata.
    pub meta: QualityMeta,
    /// Computed graph metrics.
    pub metrics: GraphMetrics,
}

/// Computed statistics about a generated graph.
///
/// Populated by `compute_graph_metrics` in `graph_export.rs`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GraphMetrics {
    /// Total number of nodes.
    pub nodes_total: usize,
    /// Total number of edges.
    pub edges_total: usize,
    /// Edge-to-node density ratio.
    pub density: f64,
    /// Fraction of nodes belonging to the largest connected component.
    pub largest_component_ratio: f64,
    /// Percentage of edges with a known direction (forward/backward).
    pub pct_directed: f64,
    /// Percentage of edges with unknown direction (bidirectional_unknown).
    pub pct_unknown: f64,
    /// Percentage of edges that carry a speed limit.
    pub pct_with_speed_limit: f64,
    /// Time taken to build the graph, in milliseconds.
    pub build_time_ms: f64,
}

// ---------------------------------------------------------------------------
// Compatibility export types
//
// These represent a simplified format intended for external tooling.
// All fields use camelCase via serde rename — except where noted.
// ---------------------------------------------------------------------------

/// A node in the compatibility export format.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct CompatNode {
    /// Unique node identifier.
    pub node_uid: u64,
    /// X coordinate in world space.
    pub x: f64,
    /// Y coordinate in world space.
    pub y: f64,
    /// Z coordinate in world space.
    pub z: f64,
}

/// A road in the compatibility export format.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct CompatRoad {
    /// Unique road identifier.
    pub road_uid: String,
    /// Road name (may be empty).
    #[serde(default)]
    pub name: String,
    /// Look token referencing a `CompatRoadLook`.
    pub look_token: String,
    /// Source node UID.
    pub from_node_uid: u64,
    /// Target node UID.
    pub to_node_uid: u64,
    /// Total road length in meters.
    pub length: f64,
    /// Speed limit in km/h, if any.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub speed_limit: Option<f64>,
    /// Number of lanes.
    pub lane_count: u32,
}

/// A road-look definition in the compatibility export format.
///
/// Describes the visual appearance / surface type of a road.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "camelCase")]
pub struct CompatRoadLook {
    /// Unique look token.
    pub token: String,
    /// Human-readable look name.
    pub name: String,
}

/// Compatibility graph (nodes + edges in camelCase form).
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct CompatGraph {
    /// Compatibility nodes.
    pub nodes: Vec<CompatNode>,
    /// Compatibility edges (graph-edge-like structure, also camelCase).
    pub edges: Vec<CompatGraphEdge>,
}

/// An edge in the compatibility graph export.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "camelCase")]
pub struct CompatGraphEdge {
    /// Deterministic edge identifier.
    pub edge_uid: u64,
    /// Source node UID.
    pub from_node_uid: u64,
    /// Target node UID.
    pub to_node_uid: u64,
    /// Associated road UID, if any.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub road_uid: Option<String>,
    /// Edge length in meters.
    pub distance_m: f64,
    /// Direction class.
    pub direction: String,
    /// Number of lanes.
    pub lane_count: u32,
    /// Speed limit in km/h, if any.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub speed_limit_kmh: Option<f64>,
    /// Road / edge flags.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub flags: Vec<String>,
}
