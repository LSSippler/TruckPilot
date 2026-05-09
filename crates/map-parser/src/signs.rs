//! Traffic sign extraction and node attachment.
//!
//! Signs are extracted from sector data and attached to the nearest graph
//! node within a configurable radius. The speed-controller plugin reads
//! `SpeedLimit` signs from the SharedBlackboard.

use serde::{Deserialize, Serialize};

use crate::graph::GraphNode;
use crate::sector::RawSign;

/// Maximum distance (m) to attach a sign to a node.
const ATTACH_RADIUS_M: f64 = 50.0;

/// Parsed traffic sign with world position and semantic value.
#[derive(Debug, Clone, Serialize, Deserialize, bincode::Encode, bincode::Decode)]
pub struct TrafficSign {
    pub uid: u64,
    pub kind: SignKind,
    pub x: f64,
    pub y: f64,
    pub z: f64,
    /// Numeric value (e.g. speed in km/h for SpeedLimit signs).
    pub value: f32,
    /// UID of the nearest graph node, if within `ATTACH_RADIUS_M`.
    pub nearest_node_uid: Option<u64>,
}

/// Semantic sign type.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq, bincode::Encode, bincode::Decode)]
pub enum SignKind {
    SpeedLimit,
    Stop,
    Yield,
    NoEntry,
    Other(u32),
}

impl From<u32> for SignKind {
    fn from(token: u32) -> Self {
        match token {
            0x0001 => SignKind::SpeedLimit,
            0x0002 => SignKind::Stop,
            0x0003 => SignKind::Yield,
            0x0004 => SignKind::NoEntry,
            other => SignKind::Other(other),
        }
    }
}

/// Attach raw signs to the nearest graph node within `ATTACH_RADIUS_M`.
pub fn attach_signs_to_nodes(raw: &[RawSign], nodes: &[GraphNode]) -> Vec<TrafficSign> {
    raw.iter()
        .map(|s| {
            let nearest = nearest_node(s.x as f64, s.y as f64, s.z as f64, nodes);
            TrafficSign {
                uid: s.uid,
                kind: SignKind::from(s.sign_type),
                x: s.x as f64,
                y: s.y as f64,
                z: s.z as f64,
                value: s.value,
                nearest_node_uid: nearest,
            }
        })
        .collect()
}

fn nearest_node(x: f64, y: f64, z: f64, nodes: &[GraphNode]) -> Option<u64> {
    nodes
        .iter()
        .filter_map(|n| {
            let dx = n.x - x;
            let dy = n.y - y;
            let dz = n.z - z;
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            if dist <= ATTACH_RADIUS_M {
                Some((dist, n.uid))
            } else {
                None
            }
        })
        .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap())
        .map(|(_, uid)| uid)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn node(uid: u64, x: f64, z: f64) -> GraphNode {
        GraphNode { uid, x, y: 0.0, z }
    }

    fn sign(uid: u64, x: f32, z: f32, kind: u32, value: f32) -> RawSign {
        RawSign {
            uid,
            sign_type: kind,
            x,
            y: 0.0,
            z,
            value,
        }
    }

    #[test]
    fn sign_attaches_to_nearest_node() {
        let nodes = vec![node(1, 0.0, 0.0), node(2, 100.0, 0.0)];
        let signs = vec![sign(10, 5.0, 0.0, 0x0001, 80.0)];
        let result = attach_signs_to_nodes(&signs, &nodes);
        assert_eq!(result[0].nearest_node_uid, Some(1));
    }

    #[test]
    fn sign_too_far_has_no_node() {
        let nodes = vec![node(1, 0.0, 0.0)];
        let signs = vec![sign(10, 200.0, 0.0, 0x0001, 80.0)];
        let result = attach_signs_to_nodes(&signs, &nodes);
        assert_eq!(result[0].nearest_node_uid, None);
    }

    #[test]
    fn sign_kind_parsed_correctly() {
        assert_eq!(SignKind::from(0x0001), SignKind::SpeedLimit);
        assert_eq!(SignKind::from(0x0002), SignKind::Stop);
        assert_eq!(SignKind::from(0xFFFF), SignKind::Other(0xFFFF));
    }

    #[test]
    fn empty_signs_returns_empty() {
        let nodes = vec![node(1, 0.0, 0.0)];
        let result = attach_signs_to_nodes(&[], &nodes);
        assert!(result.is_empty());
    }
}
