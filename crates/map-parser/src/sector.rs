//! ETS2 map sector parser.
//!
//! Parses `.base` binary sector files extracted from `.scs` archives.
//!
//! ## Format overview (from ts-map / TruckLib reference implementations)
//!
//! Header (16 bytes):
//! - CoreMapVersion (u32)
//! - GameId token (u64)
//! - GameMapVersion (u32)
//!
//! Item section: item_count (u32) followed by type-dispatched items.
//! - Type 3 (Road, v895+): 0x111 = 273 bytes — uid at 0, StartNodeUid at 0xF5, EndNodeUid at 0xFD
//! - Type 4 (Prefab): variable — KdopItem(53) + model(8) + variant(8) + counted lists
//! - Other types: unsupported — returns error so the caller can skip this sector
//!
//! Node section: node_count (u32), then 56-byte records:
//! - uid(u64) + xyz(3×i32, /256 = meters) + rotation(4×f32) + backward_uid(u64) + forward_uid(u64) + flags(u32)
//!
//! All values are little-endian.

use std::io::{Cursor, Read};
use tracing::{debug, instrument};

use crate::error::ParseError;

// ---------------------------------------------------------------------------
// Public item types (field layout kept stable so graph.rs / signs.rs compile)
// ---------------------------------------------------------------------------

/// A road node — a point in world space.
#[derive(Debug, Clone)]
pub struct RawNode {
    pub uid: u64,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// A road segment connecting two nodes.
#[derive(Debug, Clone)]
pub struct RawRoad {
    pub uid: u64,
    /// Start node UID.
    pub node_a: u64,
    /// End node UID.
    pub node_b: u64,
    /// Speed limit in km/h (0 = unknown, treated as bidirectional by graph builder).
    pub speed_limit_kmh: u16,
    pub lanes_forward: u8,
    pub lanes_backward: u8,
    pub look_token: u32,
}

/// A prefab (intersection / junction template).
#[derive(Debug, Clone)]
pub struct RawPrefab {
    pub uid: u64,
    pub template_token: u32,
    pub node_count: u8,
    pub nodes: Vec<u64>,
}

/// A traffic sign attached to the road network.
#[derive(Debug, Clone)]
pub struct RawSign {
    pub uid: u64,
    pub sign_type: u32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub value: f32,
}

// ---------------------------------------------------------------------------
// Parsed sector
// ---------------------------------------------------------------------------

/// All items extracted from one sector file.
#[derive(Debug, Clone, Default)]
pub struct ParsedSector {
    pub nodes: Vec<RawNode>,
    pub roads: Vec<RawRoad>,
    pub prefabs: Vec<RawPrefab>,
    pub signs: Vec<RawSign>,
}

// ---------------------------------------------------------------------------
// Parser
// ---------------------------------------------------------------------------

const ITEM_TYPE_ROAD: u32 = 3;
const ITEM_TYPE_PREFAB: u32 = 4;

/// Total bytes consumed by a Road item after the item_type field (version 895+).
const ROAD_BLOCK_SIZE: usize = 0x111;
/// Byte offset within road data where StartNodeUid begins.
const ROAD_START_NODE_OFFSET: usize = 0xF5;
/// Byte offset within road data where EndNodeUid begins.
const ROAD_END_NODE_OFFSET: usize = 0xFD;

/// Parse a sector from raw bytes.
#[instrument(skip(data), fields(bytes = data.len()))]
pub fn parse_sector(data: &[u8]) -> Result<ParsedSector, ParseError> {
    let mut cur = Cursor::new(data);
    let mut sector = ParsedSector::default();

    // 16-byte header: CoreMapVersion(u32) + GameId(u64) + GameMapVersion(u32)
    let core_version = read_u32(&mut cur)?;
    let _game_id = read_u64(&mut cur)?;
    let _game_map_version = read_u32(&mut cur)?;

    debug!("Sector CoreMapVersion={core_version}");

    // Items
    let item_count = read_u32(&mut cur)?;
    debug!("{item_count} items");

    for _ in 0..item_count {
        let item_type = read_u32(&mut cur)?;
        match item_type {
            ITEM_TYPE_ROAD => parse_road(&mut cur, &mut sector)?,
            ITEM_TYPE_PREFAB => parse_prefab(&mut cur, &mut sector)?,
            other => {
                return Err(ParseError::Binary(format!(
                    "unsupported item type {other}"
                )));
            }
        }
    }

    // Nodes
    let node_count = read_u32(&mut cur)?;
    debug!("{node_count} nodes");

    for _ in 0..node_count {
        sector.nodes.push(parse_node(&mut cur)?);
    }

    Ok(sector)
}

// ---------------------------------------------------------------------------
// Item parsers
// ---------------------------------------------------------------------------

fn parse_road(cur: &mut Cursor<&[u8]>, sector: &mut ParsedSector) -> Result<(), ParseError> {
    let mut buf = [0u8; ROAD_BLOCK_SIZE];
    cur.read_exact(&mut buf)
        .map_err(|e| ParseError::Binary(format!("road item: {e}")))?;

    let uid = u64::from_le_bytes(buf[0..8].try_into().unwrap());
    let node_a = u64::from_le_bytes(
        buf[ROAD_START_NODE_OFFSET..ROAD_START_NODE_OFFSET + 8]
            .try_into()
            .unwrap(),
    );
    let node_b = u64::from_le_bytes(
        buf[ROAD_END_NODE_OFFSET..ROAD_END_NODE_OFFSET + 8]
            .try_into()
            .unwrap(),
    );

    sector.roads.push(RawRoad {
        uid,
        node_a,
        node_b,
        speed_limit_kmh: 0,
        lanes_forward: 0,
        lanes_backward: 0,
        look_token: 0,
    });
    Ok(())
}

fn parse_prefab(cur: &mut Cursor<&[u8]>, sector: &mut ParsedSector) -> Result<(), ParseError> {
    // KdopItem: uid(8) + bounds(40) + flags(4) + view_dist(1) = 53 bytes
    let uid = read_u64(cur)?;
    skip(cur, 45)?; // bounds(40) + flags(4) + view_dist(1)

    let model_token = read_u64(cur)?;
    skip(cur, 8)?; // variant token (u64)

    // additionalParts: count(u32) + n×uid(u64)
    let n = read_u32(cur)? as usize;
    skip(cur, n * 8)?;

    // nodes: count(u32) + n×uid(u64) — connection points of this junction
    let node_count = read_u32(cur)? as usize;
    let mut nodes = Vec::with_capacity(node_count);
    for _ in 0..node_count {
        nodes.push(read_u64(cur)?);
    }

    // slaveItems: count(u32) + n×uid(u64)
    let m = read_u32(cur)? as usize;
    skip(cur, m * 8)?;

    // ferryLinkUid(u64=8) + origin(u16=2)
    skip(cur, 10)?;

    // corner terrain: node_count × (quadrant_random u64 + sphere_radius f32) = 12 bytes each
    skip(cur, node_count * 12)?;

    // semaphoreProfile(u64)
    skip(cur, 8)?;

    sector.prefabs.push(RawPrefab {
        uid,
        template_token: (model_token & 0xFFFF_FFFF) as u32,
        node_count: node_count.min(255) as u8,
        nodes,
    });
    Ok(())
}

fn parse_node(cur: &mut Cursor<&[u8]>) -> Result<RawNode, ParseError> {
    let uid = read_u64(cur)?;
    let x_raw = read_i32(cur)?;
    let y_raw = read_i32(cur)?;
    let z_raw = read_i32(cur)?;
    skip(cur, 16)?; // rotation quaternion (4×f32)
    skip(cur, 16)?; // backward_uid(u64) + forward_uid(u64)
    skip(cur, 4)?;  // flags(u32)
    // total: 8+4+4+4+16+16+4 = 56 bytes

    Ok(RawNode {
        uid,
        x: x_raw as f32 / 256.0,
        y: y_raw as f32 / 256.0,
        z: z_raw as f32 / 256.0,
    })
}

// ---------------------------------------------------------------------------
// Low-level readers
// ---------------------------------------------------------------------------

fn read_u32(cur: &mut Cursor<&[u8]>) -> Result<u32, ParseError> {
    let mut buf = [0u8; 4];
    cur.read_exact(&mut buf)
        .map_err(|e| ParseError::Binary(format!("read u32: {e}")))?;
    Ok(u32::from_le_bytes(buf))
}

fn read_u64(cur: &mut Cursor<&[u8]>) -> Result<u64, ParseError> {
    let mut buf = [0u8; 8];
    cur.read_exact(&mut buf)
        .map_err(|e| ParseError::Binary(format!("read u64: {e}")))?;
    Ok(u64::from_le_bytes(buf))
}

fn read_i32(cur: &mut Cursor<&[u8]>) -> Result<i32, ParseError> {
    let mut buf = [0u8; 4];
    cur.read_exact(&mut buf)
        .map_err(|e| ParseError::Binary(format!("read i32: {e}")))?;
    Ok(i32::from_le_bytes(buf))
}

fn skip(cur: &mut Cursor<&[u8]>, n: usize) -> Result<(), ParseError> {
    let pos = cur.position() as usize;
    let len = cur.get_ref().len();
    if pos + n > len {
        return Err(ParseError::Binary(format!(
            "skip({n} bytes) at pos {pos}, data len {len}"
        )));
    }
    cur.set_position((pos + n) as u64);
    Ok(())
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn write_u32(buf: &mut Vec<u8>, v: u32) {
        buf.extend_from_slice(&v.to_le_bytes());
    }
    fn write_u64(buf: &mut Vec<u8>, v: u64) {
        buf.extend_from_slice(&v.to_le_bytes());
    }
    fn write_i32(buf: &mut Vec<u8>, v: i32) {
        buf.extend_from_slice(&v.to_le_bytes());
    }

    /// Build a minimal 16-byte sector header.
    fn header(core_version: u32) -> Vec<u8> {
        let mut buf = Vec::new();
        write_u32(&mut buf, core_version); // CoreMapVersion
        write_u64(&mut buf, 0);            // GameId token
        write_u32(&mut buf, 1);            // GameMapVersion
        buf
    }

    /// Append a node record (56 bytes) to buf.
    fn append_node(buf: &mut Vec<u8>, uid: u64, x_raw: i32, y_raw: i32, z_raw: i32) {
        write_u64(buf, uid);
        write_i32(buf, x_raw);
        write_i32(buf, y_raw);
        write_i32(buf, z_raw);
        buf.extend_from_slice(&[0u8; 16]); // quaternion
        write_u64(buf, 0);                 // backward_uid
        write_u64(buf, 0);                 // forward_uid
        write_u32(buf, 0);                 // flags
    }

    /// Append a road item (type tag + 273 bytes) to buf.
    fn append_road(buf: &mut Vec<u8>, uid: u64, node_a: u64, node_b: u64) {
        write_u32(buf, ITEM_TYPE_ROAD); // type
        let start = buf.len();
        buf.extend_from_slice(&[0u8; ROAD_BLOCK_SIZE]);
        // uid at offset 0
        buf[start..start + 8].copy_from_slice(&uid.to_le_bytes());
        // node_a at ROAD_START_NODE_OFFSET
        buf[start + ROAD_START_NODE_OFFSET..start + ROAD_START_NODE_OFFSET + 8]
            .copy_from_slice(&node_a.to_le_bytes());
        // node_b at ROAD_END_NODE_OFFSET
        buf[start + ROAD_END_NODE_OFFSET..start + ROAD_END_NODE_OFFSET + 8]
            .copy_from_slice(&node_b.to_le_bytes());
    }

    #[test]
    fn parse_empty_sector() {
        let mut data = header(895);
        write_u32(&mut data, 0); // item_count
        write_u32(&mut data, 0); // node_count
        let s = parse_sector(&data).unwrap();
        assert!(s.nodes.is_empty());
        assert!(s.roads.is_empty());
        assert!(s.prefabs.is_empty());
    }

    #[test]
    fn parse_two_nodes() {
        let mut data = header(895);
        write_u32(&mut data, 0); // item_count
        write_u32(&mut data, 2); // node_count

        // x_raw=256 → x=256/256=1.0 m
        append_node(&mut data, 1, 256, 0, 0);
        append_node(&mut data, 2, 0, 512, 0);

        let s = parse_sector(&data).unwrap();
        assert_eq!(s.nodes.len(), 2);
        assert_eq!(s.nodes[0].uid, 1);
        assert!((s.nodes[0].x - 1.0).abs() < 1e-5, "x={}", s.nodes[0].x);
        assert_eq!(s.nodes[1].uid, 2);
        assert!((s.nodes[1].y - 2.0).abs() < 1e-5, "y={}", s.nodes[1].y);
    }

    #[test]
    fn parse_road_item() {
        let mut data = header(895);
        write_u32(&mut data, 1); // item_count
        append_road(&mut data, 42, 100, 200);
        write_u32(&mut data, 0); // node_count

        let s = parse_sector(&data).unwrap();
        assert_eq!(s.roads.len(), 1);
        assert_eq!(s.roads[0].uid, 42);
        assert_eq!(s.roads[0].node_a, 100);
        assert_eq!(s.roads[0].node_b, 200);
    }

    #[test]
    fn road_and_nodes_together() {
        let mut data = header(895);
        write_u32(&mut data, 1); // item_count
        append_road(&mut data, 10, 1, 2);
        write_u32(&mut data, 2); // node_count
        append_node(&mut data, 1, 0, 0, 0);
        append_node(&mut data, 2, 25600, 0, 0); // 25600/256 = 100.0 m

        let s = parse_sector(&data).unwrap();
        assert_eq!(s.roads.len(), 1);
        assert_eq!(s.nodes.len(), 2);
        assert!((s.nodes[1].x - 100.0).abs() < 1e-5);
    }

    #[test]
    fn unknown_item_type_returns_error() {
        let mut data = header(895);
        write_u32(&mut data, 1);  // item_count
        write_u32(&mut data, 12); // type 12 = City, not supported

        let result = parse_sector(&data);
        assert!(result.is_err());
    }

    #[test]
    fn truncated_data_returns_error() {
        // Header only, no item_count
        let data = header(895);
        assert!(parse_sector(&data).is_err());
    }
}
