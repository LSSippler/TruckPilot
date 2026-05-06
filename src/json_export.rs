//! Placeholder definitions for ETS2 map data structures.
//!
//! These types simulate the expected output of an ETS2 map parser.
//! In production, they would be populated by reading and deserializing
//! the binary SCS map format.

use serde::{Deserialize, Serialize};

/// Top-level container for parsed ETS2 map data.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MapData {
    /// All map nodes (road vertices, prefab points, etc.).
    pub nodes: Vec<MapNode>,
    /// All road items defining the road network.
    pub roads: Vec<MapRoad>,
    /// All prefab items (intersections, company yards, rest areas).
    pub prefabs: Vec<MapPrefab>,
}

/// A single point in the road network (road vertex).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct MapNode {
    /// Unique node identifier (ETS2 item UID).
    pub uid: u64,
    /// X coordinate in ETS2 world space.
    pub x: f64,
    /// Y coordinate in ETS2 world space (height).
    pub y: f64,
    /// Z coordinate in ETS2 world space.
    pub z: f64,
}

/// A road item connecting a sequence of nodes.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct MapRoad {
    /// Unique road identifier.
    pub uid: String,
    /// Display name (may be empty).
    #[serde(default)]
    pub name: String,
    /// Look token referencing a road appearance.
    #[serde(default)]
    pub look_token: String,
    /// Ordered list of node UIDs forming the road polyline.
    pub nodes: Vec<u64>,
    /// Speed limit in km/h, if any.
    #[serde(default)]
    pub speed_limit: Option<f64>,
    /// Number of lanes in forward direction.
    #[serde(default)]
    pub lane_count_forward: u32,
    /// Number of lanes in backward direction.
    #[serde(default)]
    pub lane_count_backward: u32,
}

/// A prefab item (intersection, depot, etc.) with multiple connection nodes.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct MapPrefab {
    /// Unique prefab identifier.
    pub uid: String,
    /// Node UIDs belonging to this prefab.
    pub nodes: Vec<u64>,
}
