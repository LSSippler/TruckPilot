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
//! - Type 3 (Road, v895+): 0x111 = 273 bytes — uid at 0, StartNodeUid at 0xFD, EndNodeUid at 0x105
//! - Type 4 (Prefab): variable — KdopItem(53) + model(8) + variant(8) + counted lists
//! - All other 22 known ETS2 types: skip-only (no extraction yet) — ported from
//!   the reference parser at `src/ets2_parser/binary_parser.rs`.
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

const ITEM_TYPE_TERRAIN: u32 = 1;
const ITEM_TYPE_BUILDINGS: u32 = 2;
const ITEM_TYPE_ROAD: u32 = 3;
const ITEM_TYPE_PREFAB: u32 = 4;
const ITEM_TYPE_MODEL: u32 = 5;
const ITEM_TYPE_COMPANY: u32 = 6;
const ITEM_TYPE_SERVICE: u32 = 7;
/// CutPlane (visibility / culling helper) — ported from
/// `src/ets2_parser/binary_parser.rs::skip_cut_plane`.
const ITEM_TYPE_CUT_PLANE: u32 = 8;
const ITEM_TYPE_CITY: u32 = 12;
const ITEM_TYPE_MAP_OVERLAY: u32 = 18;
const ITEM_TYPE_FERRY: u32 = 19;
const ITEM_TYPE_GARAGE: u32 = 22;
const ITEM_TYPE_TRIGGER: u32 = 34;
const ITEM_TYPE_FUEL_PUMP: u32 = 35;
const ITEM_TYPE_SIGN: u32 = 36;
const ITEM_TYPE_BUS_STOP: u32 = 37;
const ITEM_TYPE_TRAFFIC_AREA: u32 = 38;
const ITEM_TYPE_BEZIER_PATCH: u32 = 39;
const ITEM_TYPE_TRAJECTORY: u32 = 41;
const ITEM_TYPE_MAP_AREA: u32 = 42;
const ITEM_TYPE_FAR_MODEL: u32 = 43;
const ITEM_TYPE_CURVE: u32 = 44;
const ITEM_TYPE_CUTSCENE: u32 = 46;
const ITEM_TYPE_VISIBILITY_AREA: u32 = 48;

/// Total bytes consumed by a Road item after the item_type field (version 895+).
///
/// Layout of the 273-byte block (offsets within `buf`):
/// ```text
///   0x000 8   uid (kdop_uid)
///   0x008 40  kdop_bounds
///   0x030 4   kdop_flag1..4 (u8 × 4)
///   0x034 1   view_distance
///   0x035 4   road_flag1..4 (u8 × 4)
///   0x039 88  11 × token (u64)
///   0x091 4   right_terrain_coef (f32)
///   0x095 8   token (u64)
///   0x09D 4   left_terrain_coef (f32)
///   0x0A1 24  3 × token (u64)
///   0x0B9 60  3 × (token u64 + i16 + token u64 + i16)  // 20 B each
///   0x0F5 4   right_height_offset (i32)   ← previously misread as start_node_uid
///   0x0F9 4   left_height_offset (i32)
///   0x0FD 8   backward_node_uid (u64)     ← actual start node UID
///   0x105 8   forward_node_uid (u64)      ← actual end node UID
///   0x10D 4   length (f32)
/// ```
const ROAD_BLOCK_SIZE: usize = 0x111;
/// Byte offset of the **backward** (start) node UID — the kdop+terrain+offset
/// header is 0xFD bytes long.
const ROAD_START_NODE_OFFSET: usize = 0xFD;
/// Byte offset of the **forward** (end) node UID — directly after the
/// backward node.
const ROAD_END_NODE_OFFSET: usize = 0x105;

/// Hard cap on `item_count` / `node_count` to reject corrupt headers up front.
const MAX_LIST_COUNT: u32 = 2_000_000;

/// Hard cap on a Pascal-style string length read from the on-disk format.
const MAX_PASCAL_STRING_LEN: u64 = 1_048_576;

/// Parse a sector from raw bytes.
///
/// Two on-disk formats are accepted:
/// 1. **Sized format** (modern ETS2 1.50+): 20-byte header followed by
///    `item_count × (item_type u32 + item_size u32 + payload[size])`. Unknown
///    item kinds can be skipped trivially via `seek(item_end)`. Roads, prefabs
///    and the trailing node list are parsed.
/// 2. **Legacy/TruckLib format** (older sectors / fallback): 16-byte header,
///    then type-specific bodies without size prefix. Roads (273 B fixed) and
///    Prefabs (counted-list walk) are parsed; all 22 other known ETS2 item
///    types are skipped via dedicated handlers ported from the reference
///    parser. Unknown types (e.g. ETS2 1.50+ types > 48) abort the sector.
///
/// The dispatcher tries sized first via `try_parse_sized_sector`; on failure it
/// falls back to `parse_sector_legacy` so existing test fixtures keep passing.
#[instrument(skip(data), fields(bytes = data.len()))]
pub fn parse_sector(data: &[u8]) -> Result<ParsedSector, ParseError> {
    if let Some(sector) = try_parse_sized_sector(data) {
        return Ok(sector);
    }
    parse_sector_legacy(data)
}

fn parse_sector_legacy(data: &[u8]) -> Result<ParsedSector, ParseError> {
    let mut cur = Cursor::new(data);
    let mut sector = ParsedSector::default();

    // 16-byte header: CoreMapVersion(u32) + GameId(u64) + GameMapVersion(u32)
    let core_version = read_u32(&mut cur)?;
    let _game_id = read_u64(&mut cur)?;
    let _game_map_version = read_u32(&mut cur)?;

    debug!("Sector CoreMapVersion={core_version}");

    // Items
    let item_count = read_u32(&mut cur)?;
    if item_count > MAX_LIST_COUNT {
        return Err(ParseError::Binary(format!(
            "implausible item_count {item_count}"
        )));
    }
    debug!("{item_count} items");

    for _ in 0..item_count {
        let item_type = read_u32(&mut cur)?;
        match item_type {
            ITEM_TYPE_ROAD => parse_road(&mut cur, &mut sector)?,
            ITEM_TYPE_PREFAB => parse_prefab(&mut cur, &mut sector)?,
            ITEM_TYPE_TERRAIN => skip_terrain(&mut cur)?,
            ITEM_TYPE_BUILDINGS => skip_buildings(&mut cur)?,
            ITEM_TYPE_MODEL => skip_model(&mut cur)?,
            ITEM_TYPE_COMPANY => skip_company(&mut cur)?,
            ITEM_TYPE_SERVICE => skip_service(&mut cur)?,
            ITEM_TYPE_CUT_PLANE => skip_cut_plane(&mut cur)?,
            ITEM_TYPE_CITY => skip_city(&mut cur)?,
            ITEM_TYPE_MAP_OVERLAY => skip_map_overlay(&mut cur)?,
            ITEM_TYPE_FERRY => skip_ferry(&mut cur)?,
            ITEM_TYPE_GARAGE => skip_garage(&mut cur)?,
            ITEM_TYPE_TRIGGER => skip_trigger(&mut cur)?,
            ITEM_TYPE_FUEL_PUMP => skip_fuel_pump(&mut cur)?,
            ITEM_TYPE_SIGN => skip_sign(&mut cur)?,
            ITEM_TYPE_BUS_STOP => skip_bus_stop(&mut cur)?,
            ITEM_TYPE_TRAFFIC_AREA => skip_traffic_area(&mut cur)?,
            ITEM_TYPE_BEZIER_PATCH => skip_bezier_patch(&mut cur)?,
            ITEM_TYPE_TRAJECTORY => skip_trajectory(&mut cur)?,
            ITEM_TYPE_MAP_AREA => skip_map_area(&mut cur)?,
            ITEM_TYPE_FAR_MODEL => skip_far_model(&mut cur)?,
            ITEM_TYPE_CURVE => skip_curve(&mut cur)?,
            ITEM_TYPE_CUTSCENE => skip_cutscene(&mut cur)?,
            ITEM_TYPE_VISIBILITY_AREA => skip_visibility_area(&mut cur)?,
            other => {
                return Err(ParseError::Binary(format!(
                    "unsupported item type {other}"
                )));
            }
        }
    }

    // Nodes
    let node_count = read_u32(&mut cur)?;
    if node_count > MAX_LIST_COUNT {
        return Err(ParseError::Binary(format!(
            "implausible node_count {node_count}"
        )));
    }
    debug!("{node_count} nodes");

    for _ in 0..node_count {
        sector.nodes.push(parse_node(&mut cur)?);
    }

    Ok(sector)
}

/// Attempt the sized-format parse. Returns `None` if the header doesn't look
/// sized so the caller can fall back to the legacy parser.
///
/// Detection: we treat the data as sized format when the 20-byte header parses
/// cleanly **and** the first `min(4, item_count)` items walk through cleanly
/// using the `type+size+payload` pattern without overrunning the buffer.
fn try_parse_sized_sector(data: &[u8]) -> Option<ParsedSector> {
    const HEADER_LEN: usize = 4 + 8 + 8; // u32 + u64 + u64

    if data.len() < HEADER_LEN + 4 {
        return None;
    }

    let mut cur = Cursor::new(data);
    let _version = read_u32(&mut cur).ok()?;
    let _game_id = read_u64(&mut cur).ok()?;
    let _map_version = read_u64(&mut cur).ok()?;
    let item_count = read_u32(&mut cur).ok()?;
    if item_count > MAX_LIST_COUNT {
        return None;
    }

    // Probe the *full* item list to confirm the sized layout. The probe must
    // (a) walk every item without overrunning, (b) reject implausible item
    // sizes, and (c) leave the trailing node tail in a plausible layout —
    // otherwise we treat the data as legacy.
    //
    // Minimum item payload size in real ETS2 sectors is at least 8 bytes
    // (a single u64 uid); 0-size items mean we accidentally interpreted
    // legacy bytes as a size field.
    const MIN_ITEM_PAYLOAD: u32 = 8;
    const MAX_ITEM_PAYLOAD: u32 = 16 * 1024 * 1024; // 16 MB — generous

    let probe_pos = cur.position();
    for _ in 0..item_count {
        let _item_type = read_u32(&mut cur).ok()?;
        let item_size = read_u32(&mut cur).ok()?;
        if !(MIN_ITEM_PAYLOAD..=MAX_ITEM_PAYLOAD).contains(&item_size) {
            return None;
        }
        let new_pos = cur.position().checked_add(item_size as u64)?;
        if (new_pos as usize) > data.len() {
            return None;
        }
        cur.set_position(new_pos);
    }

    // Validate the trailing node tail. Accept any of:
    //   • 0 bytes left
    //   • exactly 4 bytes left (`node_count u32` = 0)
    //   • a `count u32` followed by exactly `count × 36` bytes
    //   • a stream of 36-byte records to EOF
    let after_items_pos = cur.position() as usize;
    let remaining = data.len().saturating_sub(after_items_pos);
    let node_record_size = 36usize;
    let plausible_tail = remaining == 0
        || remaining == 4
        || (remaining >= 4 && {
            let count_bytes: [u8; 4] = data
                .get(after_items_pos..after_items_pos + 4)
                .and_then(|b| b.try_into().ok())
                .unwrap_or([0; 4]);
            let count = u32::from_le_bytes(count_bytes);
            count <= MAX_LIST_COUNT
                && (count as usize).saturating_mul(node_record_size)
                    == remaining.saturating_sub(4)
        })
        || remaining.is_multiple_of(node_record_size);
    if !plausible_tail {
        return None;
    }

    cur.set_position(probe_pos);

    // Real walk — partial failures inside an item are tolerated by always
    // seeking to the declared `item_end`.
    let mut sector = ParsedSector::default();
    for _ in 0..item_count {
        let item_type = read_u32(&mut cur).ok()?;
        let item_size = read_u32(&mut cur).ok()?;
        let item_start = cur.position();
        let item_end = item_start.checked_add(item_size as u64)?;
        if (item_end as usize) > data.len() {
            return Some(sector);
        }

        match item_type {
            ITEM_TYPE_ROAD => {
                if let Ok(road) = parse_sized_road(&mut cur) {
                    sector.roads.push(road);
                }
            }
            ITEM_TYPE_PREFAB => {
                if let Ok(prefab) = parse_sized_prefab(&mut cur) {
                    sector.prefabs.push(prefab);
                }
            }
            _ => { /* unknown — skip via item_end */ }
        }

        cur.set_position(item_end);
    }

    // Trailing node list. Two layouts seen empirically:
    //   (a) `node_count u32` followed by `node_count × 36 B` records.
    //   (b) Stream of 36-B records to EOF (no count prefix).
    //
    // We try (a) first; if the count would overrun the remaining bytes we
    // rewind and try (b).
    let node_record_size = 8 + 8 + 8 + 8 + 4; // uid + 3×f64 + rot f32 = 36 B
    let pos_before_nodes = cur.position() as usize;
    let remaining = data.len().saturating_sub(pos_before_nodes);
    if remaining >= 4 {
        if let Ok(node_count) = read_u32(&mut cur) {
            let needed = (node_count as usize).saturating_mul(node_record_size);
            if node_count <= MAX_LIST_COUNT && needed <= remaining.saturating_sub(4) {
                for _ in 0..node_count {
                    if let Ok(node) = parse_node_f64(&mut cur) {
                        sector.nodes.push(node);
                    }
                }
                return Some(sector);
            }
            cur.set_position(pos_before_nodes as u64);
        }
    }
    if remaining > 0 && remaining.is_multiple_of(node_record_size) {
        let count = remaining / node_record_size;
        for _ in 0..count {
            if let Ok(node) = parse_node_f64(&mut cur) {
                sector.nodes.push(node);
            }
        }
    }

    Some(sector)
}

/// Sized-format Road payload (32 B).
/// Layout: start_node u64 | end_node u64 | length f32 | lanes_fwd u32 |
///         lanes_back u32 | speed_limit f32
fn parse_sized_road(cur: &mut Cursor<&[u8]>) -> Result<RawRoad, ParseError> {
    let start = read_u64(cur)?;
    let end = read_u64(cur)?;
    let _length = read_f32(cur)?;
    let lanes_forward = read_u32(cur)?.min(255) as u8;
    let lanes_backward = read_u32(cur)?.min(255) as u8;
    let speed_raw = read_f32(cur)?;
    let speed_kmh = if speed_raw.is_finite() && speed_raw > 0.0 {
        speed_raw.round().clamp(0.0, u16::MAX as f32) as u16
    } else {
        0
    };

    // Sized payload doesn't carry the road's own UID — synthesize zero;
    // GraphBuilder doesn't dedup by road.uid, only edges.
    Ok(RawRoad {
        uid: 0,
        node_a: start,
        node_b: end,
        speed_limit_kmh: speed_kmh,
        lanes_forward,
        lanes_backward,
        look_token: 0,
    })
}

/// Sized-format Prefab payload.
/// Layout: uid u64 | descriptor_token u64 | node_count u32 | nodes (u64 × count)
fn parse_sized_prefab(cur: &mut Cursor<&[u8]>) -> Result<RawPrefab, ParseError> {
    let uid = read_u64(cur)?;
    let token = read_u64(cur)?;
    let node_count = read_u32(cur)?;
    if node_count > MAX_LIST_COUNT {
        return Err(ParseError::Binary(format!(
            "implausible prefab node_count {node_count}"
        )));
    }
    let mut nodes = Vec::with_capacity(node_count as usize);
    for _ in 0..node_count {
        nodes.push(read_u64(cur)?);
    }
    Ok(RawPrefab {
        uid,
        template_token: (token & 0xFFFF_FFFF) as u32,
        node_count: (node_count.min(255)) as u8,
        nodes,
    })
}

/// Sized-format node record (36 B): uid u64 | x f64 | y f64 | z f64 | rot f32
fn parse_node_f64(cur: &mut Cursor<&[u8]>) -> Result<RawNode, ParseError> {
    let uid = read_u64(cur)?;
    let x = read_f64(cur)?;
    let y = read_f64(cur)?;
    let z = read_f64(cur)?;
    let _rot = read_f32(cur)?;
    Ok(RawNode {
        uid,
        x: x as f32,
        y: y as f32,
        z: z as f32,
    })
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
// Skip handlers — ported from `src/ets2_parser/binary_parser.rs`
//
// These walk past items whose semantics we don't (yet) extract. Each handler
// consumes exactly the bytes the legacy reference parser does — see the
// reference line numbers in the doc comments. The shape of every handler is
// `fn(&mut Cursor<&[u8]>) -> Result<(), ParseError>`.
// ---------------------------------------------------------------------------

/// Type 1 — Terrain. Ref: `binary_parser.rs::skip_terrain` lines 662-675.
fn skip_terrain(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    skip_token(cur)?;
    skip_token(cur)?;
    skip_token(cur)?;
    let _ = read_u16(cur)?;
    skip_token(cur)?;
    let _ = read_u16(cur)?;
    let _ = read_u32(cur)?;
    let _ = read_u32(cur)?;
    skip_float_list(cur)?;
    skip_float_list(cur)?;
    skip_float_list(cur)
}

/// Type 2 — Buildings. Ref: `skip_buildings` lines 677-686.
fn skip_buildings(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    skip_token(cur)?;
    let _ = read_u64(cur)?;
    skip_token(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_u32(cur)?;
    let _ = read_u32(cur)?;
    Ok(())
}

/// Type 5 — Model. Ref: `skip_model` lines 688-696.
fn skip_model(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_u64(cur)?;
    skip_token(cur)?;
    let _ = read_u8(cur)?;
    Ok(())
}

/// Type 6 — Company. Ref: `skip_company` lines 752-769.
fn skip_company(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_u64(cur)?;
    let spawn_count = read_u32(cur)?;
    ensure_count(spawn_count, "company spawn count")?;
    ensure_capacity(cur, spawn_count, 8, "company spawn nodes")?;
    for _ in 0..spawn_count {
        let _ = read_u64(cur)?;
    }
    ensure_capacity(cur, spawn_count, 4, "company spawn counters")?;
    for _ in 0..spawn_count {
        let _ = read_u32(cur)?;
    }
    Ok(())
}

/// Type 7 — Service. Ref: `skip_service` lines 862-868.
fn skip_service(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_u64(cur)?;
    let _ = skip_node_ref_list(cur)?;
    Ok(())
}

/// Type 8 — CutPlane. Ref: `skip_cut_plane` lines 825-829.
fn skip_cut_plane(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    let _ = skip_node_ref_list(cur)?;
    Ok(())
}

/// Type 12 — City. Ref: `skip_city` lines 771-778.
fn skip_city(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_f32(cur)?;
    let _ = read_f32(cur)?;
    let _ = read_u64(cur)?;
    Ok(())
}

/// Type 18 — MapOverlay. Ref: `skip_map_overlay` lines 848-853.
fn skip_map_overlay(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    skip_token(cur)?;
    let _ = read_u64(cur)?;
    Ok(())
}

/// Type 19 — Ferry. Ref: `skip_ferry` lines 780-789.
fn skip_ferry(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_f32(cur)?;
    let _ = read_f32(cur)?;
    let _ = read_f32(cur)?;
    Ok(())
}

/// Type 22 — Garage. Ref: `skip_garage` lines 838-846.
fn skip_garage(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    skip_token(cur)?;
    let _ = read_u32(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_u64(cur)?;
    let _ = skip_node_ref_list(cur)?;
    Ok(())
}

/// Type 34 — Trigger. Ref: `skip_trigger` lines 893-902.
fn skip_trigger(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    skip_token_list(cur)?;
    let node_count = skip_node_ref_list(cur)?;
    skip_action_list(cur, true)?;
    if node_count == 1 {
        let _ = read_f32(cur)?;
    }
    Ok(())
}

/// Type 35 — FuelPump. Ref: `skip_fuel_pump` lines 791-800.
fn skip_fuel_pump(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_u64(cur)?;
    let node_count = skip_node_ref_list(cur)?;
    for _ in 0..node_count {
        let _ = read_u64(cur)?;
    }
    Ok(())
}

/// Type 36 — Sign. Ref: `skip_sign` lines 802-815.
fn skip_sign(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_u64(cur)?;
    let board_count = read_u8(cur)? as u32;
    for _ in 0..board_count {
        skip_token(cur)?;
        skip_token(cur)?;
    }
    skip_sign_board_override_list(cur)?;
    skip_sign_override_list(cur)
}

/// Type 37 — BusStop. Ref: `skip_bus_stop` lines 817-823.
fn skip_bus_stop(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_u64(cur)?;
    Ok(())
}

/// Type 38 — TrafficArea. Ref: `skip_traffic_area` lines 870-877.
fn skip_traffic_area(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    skip_token_list(cur)?;
    let _ = skip_node_ref_list(cur)?;
    skip_token(cur)?;
    let _ = read_f32(cur)?;
    Ok(())
}

/// Type 39 — BezierPatch. Ref: `skip_bezier_patch` lines 698-706.
fn skip_bezier_patch(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_u64(cur)?;
    for _ in 0..4 {
        skip_vector3(cur)?;
    }
    Ok(())
}

/// Type 41 — Trajectory. Ref: `skip_trajectory` lines 879-891.
fn skip_trajectory(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    let _ = skip_node_ref_list(cur)?;
    skip_token(cur)?;
    skip_trajectory_rule_list(cur)?;
    let checkpoint_count = read_u32(cur)?;
    ensure_count(checkpoint_count, "trajectory checkpoints")?;
    for _ in 0..checkpoint_count {
        skip_token(cur)?;
        skip_token(cur)?;
    }
    skip_token_list(cur)
}

/// Type 42 — MapArea. Ref: `skip_map_area` lines 855-860.
fn skip_map_area(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    let _ = skip_node_ref_list(cur)?;
    let _ = read_u32(cur)?;
    Ok(())
}

/// Type 43 — FarModel. Ref: `skip_far_model` lines 743-750.
fn skip_far_model(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    skip_token(cur)?;
    skip_token(cur)?;
    skip_token(cur)?;
    let _ = read_u64(cur)?;
    Ok(())
}

/// Type 44 — Curve. Ref: `skip_curve` lines 708-741.
fn skip_curve(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_u64(cur)?;
    skip_vector3(cur)?;
    skip_vector3(cur)?;
    let _ = read_f32(cur)?;
    let _ = read_f32(cur)?;
    let _ = read_u32(cur)?;
    for _ in 0..3 {
        skip_token(cur)?;
        let _ = read_i16(cur)?;
    }
    for _ in 0..2 {
        let _ = read_u16(cur)?;
        skip_token(cur)?;
        let _ = read_f32(cur)?;
        skip_token(cur)?;
        let _ = read_f32(cur)?;
        for _ in 0..3 {
            skip_road_vegetation(cur)?;
        }
        let _ = read_u16(cur)?;
        let _ = read_u16(cur)?;
    }
    skip_vegetation_sphere_list(cur)?;
    skip_terrain_quad_data(cur)?;
    skip_terrain_quad_data(cur)?;
    skip_token(cur)?;
    skip_token(cur)?;
    skip_token(cur)?;
    skip_token(cur)?;
    Ok(())
}

/// Type 46 — Cutscene. Ref: `skip_cutscene` lines 831-836.
fn skip_cutscene(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    skip_token_list(cur)?;
    let _ = read_u64(cur)?;
    skip_action_list(cur, false)
}

/// Type 48 — VisibilityArea. Ref: `skip_visibility_area` lines 904-910.
fn skip_visibility_area(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_f32(cur)?;
    let _ = read_f32(cur)?;
    skip_item_ref_list(cur)
}

// ---------------------------------------------------------------------------
// KdopItem + small helpers
// ---------------------------------------------------------------------------

/// Read the common `KdopItem` header that prefixes most ETS2 sector items:
/// `uid u64 + bounds(10×f32) + flags u32 + view_distance u8` = 53 bytes.
/// Returns the UID. Ref: `read_kdop_item` lines 912-918.
fn read_kdop_item(cur: &mut Cursor<&[u8]>) -> Result<u64, ParseError> {
    let uid = read_u64(cur)?;
    skip_kdop_bounds(cur)?;
    let _flags = read_u32(cur)?;
    let _view_distance = read_u8(cur)?;
    Ok(uid)
}

/// Skip the 10×f32 KDOP bounds. Ref: `skip_kdop_bounds` lines 920-925.
fn skip_kdop_bounds(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    skip(cur, 10 * 4)
}

/// Skip a 4×f32 quaternion. Ref: `skip_quaternion` lines 927-932.
#[allow(dead_code)]
fn skip_quaternion(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    skip(cur, 16)
}

/// Skip an 8-byte token (u64). Ref: `skip_token` lines 934-937.
fn skip_token(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_u64(cur)?;
    Ok(())
}

/// Skip a Pascal-style string: `len u64` followed by `len` raw bytes.
/// Ref: `skip_pascal_string` lines 939-948.
fn skip_pascal_string(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let len = read_u64(cur)?;
    if len > MAX_PASCAL_STRING_LEN {
        return Err(ParseError::Binary(format!(
            "pascal string length {len} exceeds safety limit"
        )));
    }
    if len > usize::MAX as u64 {
        return Err(ParseError::Binary(format!(
            "string length {len} exceeds addressable memory"
        )));
    }
    skip(cur, len as usize)
}

/// Skip a token list: `count u32` + `count × u64`. Ref: `skip_token_list` lines 950-958.
fn skip_token_list(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let count = read_u32(cur)?;
    ensure_count(count, "token list")?;
    ensure_capacity(cur, count, 8, "token list")?;
    for _ in 0..count {
        skip_token(cur)?;
    }
    Ok(())
}

/// Skip a node-ref list: `count u32` + `count × u64`. Returns the count.
/// Ref: `skip_node_ref_list` lines 960-968.
fn skip_node_ref_list(cur: &mut Cursor<&[u8]>) -> Result<u32, ParseError> {
    let count = read_u32(cur)?;
    ensure_count(count, "node refs")?;
    ensure_capacity(cur, count, 8, "node refs")?;
    for _ in 0..count {
        let _ = read_u64(cur)?;
    }
    Ok(count)
}

/// Skip an item-ref list: `count u32` + `count × u64`. Ref: `skip_item_ref_list` lines 970-978.
fn skip_item_ref_list(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let count = read_u32(cur)?;
    ensure_count(count, "item refs")?;
    ensure_capacity(cur, count, 8, "item refs")?;
    for _ in 0..count {
        let _ = read_u64(cur)?;
    }
    Ok(())
}

/// Skip a list of trajectory rules. Ref: `skip_trajectory_rule_list` lines 980-993.
fn skip_trajectory_rule_list(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let count = read_u32(cur)?;
    ensure_count(count, "trajectory rules")?;
    for _ in 0..count {
        let _ = read_u32(cur)?;
        skip_token(cur)?;
        let param_count = read_u32(cur)?;
        ensure_count(param_count, "trajectory rule params")?;
        for _ in 0..param_count {
            let _ = read_f32(cur)?;
        }
    }
    Ok(())
}

/// Skip a list of actions. `include_name` adds a leading token per action.
/// Ref: `skip_action_list` lines 995-1005.
fn skip_action_list(cur: &mut Cursor<&[u8]>, include_name: bool) -> Result<(), ParseError> {
    let count = read_u32(cur)?;
    ensure_count(count, "action list")?;
    for _ in 0..count {
        if include_name {
            skip_token(cur)?;
        }
        skip_action_base(cur)?;
    }
    Ok(())
}

/// Skip an action base. Ref: `skip_action_base` lines 1007-1030.
fn skip_action_base(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let num_param_count = read_u32(cur)?;
    ensure_count(num_param_count, "action numeric params")?;
    ensure_capacity(cur, num_param_count, 4, "action numeric params")?;
    for _ in 0..num_param_count {
        let _ = read_f32(cur)?;
    }

    let string_param_count = read_u32(cur)?;
    ensure_count(string_param_count, "action string params")?;
    for _ in 0..string_param_count {
        skip_pascal_string(cur)?;
    }

    let target_tag_count = read_u32(cur)?;
    ensure_count(target_tag_count, "action target tags")?;
    for _ in 0..target_tag_count {
        skip_token(cur)?;
    }

    let _ = read_f32(cur)?;
    let _ = read_u32(cur)?;
    Ok(())
}

/// Ref: `skip_sign_board_override_list` lines 1032-1047.
fn skip_sign_board_override_list(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let count = read_u32(cur)?;
    ensure_count(count, "sign board overrides")?;
    for _ in 0..count {
        skip_token(cur)?;
        let flags = read_u8(cur)?;
        if flags & 0x01 != 0 {
            let _ = read_u8(cur)?;
            let _ = read_u8(cur)?;
        }
        if flags & 0x02 != 0 {
            skip_token(cur)?;
        }
    }
    Ok(())
}

/// Ref: `skip_sign_override_list` lines 1049-1086.
fn skip_sign_override_list(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let count = read_u32(cur)?;
    ensure_count(count, "sign overrides")?;
    for _ in 0..count {
        let _ = read_u32(cur)?;
        skip_token(cur)?;
        let attr_count = read_u32(cur)?;
        ensure_count(attr_count, "sign override attrs")?;
        for _ in 0..attr_count {
            let attr_type = read_u16(cur)?;
            let _ = read_u32(cur)?;
            match attr_type {
                1 => {
                    let _ = read_u8(cur)?;
                }
                2 => {
                    let _ = read_i32(cur)?;
                }
                3 => {
                    let _ = read_u32(cur)?;
                }
                4 => {
                    let _ = read_f32(cur)?;
                }
                5 => {
                    skip_pascal_string(cur)?;
                }
                6 => {
                    let _ = read_u64(cur)?;
                }
                _ => {
                    return Err(ParseError::Binary(format!(
                        "unknown sign override attribute type {attr_type}"
                    )));
                }
            }
        }
    }
    Ok(())
}

/// Ref: `skip_road_vegetation` lines 1088-1096.
fn skip_road_vegetation(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    skip_token(cur)?;
    let _ = read_u16(cur)?;
    let _ = read_u8(cur)?;
    let _ = read_u8(cur)?;
    let _ = read_u16(cur)?;
    let _ = read_u16(cur)?;
    Ok(())
}

/// Ref: `skip_vegetation_sphere_list` lines 1098-1108.
fn skip_vegetation_sphere_list(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let count = read_u32(cur)?;
    ensure_count(count, "vegetation spheres")?;
    ensure_capacity(cur, count, 16, "vegetation spheres")?;
    for _ in 0..count {
        skip_vector3(cur)?;
        let _ = read_f32(cur)?;
        let _ = read_u32(cur)?;
    }
    Ok(())
}

/// Ref: `skip_terrain_quad_data` lines 1110-1148.
fn skip_terrain_quad_data(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let brush_mat_count = read_u16(cur)?;
    for _ in 0..brush_mat_count {
        skip_token(cur)?;
        let _ = read_u16(cur)?;
    }

    let color_count = read_u16(cur)?;
    for _ in 0..color_count {
        skip_color(cur)?;
    }

    let _rows = read_u16(cur)?;
    let _cols = read_u16(cur)?;

    let quad_count = read_u32(cur)?;
    ensure_count(quad_count, "terrain quad colors")?;
    for _ in 0..quad_count {
        skip_color(cur)?;
    }

    let offset_count = read_u32(cur)?;
    ensure_count(offset_count, "terrain offsets")?;
    for _ in 0..offset_count {
        let _ = read_u16(cur)?;
        let _ = read_u16(cur)?;
        skip_vector3(cur)?;
    }

    let normal_count = read_u32(cur)?;
    ensure_count(normal_count, "terrain normals")?;
    for _ in 0..normal_count {
        let _ = read_u16(cur)?;
        let _ = read_u16(cur)?;
        skip_vector3(cur)?;
    }

    Ok(())
}

/// Ref: `skip_float_list` lines 1150-1158.
fn skip_float_list(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let count = read_u32(cur)?;
    ensure_count(count, "float list")?;
    ensure_capacity(cur, count, 4, "float list")?;
    for _ in 0..count {
        let _ = read_f32(cur)?;
    }
    Ok(())
}

/// Ref: `skip_vector3` lines 1160-1165.
fn skip_vector3(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    skip(cur, 12)
}

/// Ref: `skip_color` lines 1167-1173.
fn skip_color(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    skip(cur, 4)
}

// ---------------------------------------------------------------------------
// Capacity / count guards (mirroring `ensure_count` / `ensure_capacity` in
// the reference parser at lines 1243-1266).
// ---------------------------------------------------------------------------

fn ensure_count(count: u32, label: &str) -> Result<(), ParseError> {
    if count > MAX_LIST_COUNT {
        return Err(ParseError::Binary(format!(
            "{label}: count {count} exceeds safety limit"
        )));
    }
    Ok(())
}

fn ensure_capacity(
    cur: &Cursor<&[u8]>,
    count: u32,
    elem_size: usize,
    label: &str,
) -> Result<(), ParseError> {
    let need = (count as usize)
        .checked_mul(elem_size)
        .ok_or_else(|| ParseError::Binary(format!("{label}: byte size overflow")))?;
    let remaining = cursor_remaining(cur);
    if need > remaining {
        return Err(ParseError::Binary(format!(
            "{label}: need {need} bytes, have {remaining}"
        )));
    }
    Ok(())
}

fn cursor_remaining(cur: &Cursor<&[u8]>) -> usize {
    let len = cur.get_ref().len();
    let pos = cur.position() as usize;
    len.saturating_sub(pos)
}

// ---------------------------------------------------------------------------
// Low-level readers
// ---------------------------------------------------------------------------

fn read_u8(cur: &mut Cursor<&[u8]>) -> Result<u8, ParseError> {
    let mut buf = [0u8; 1];
    cur.read_exact(&mut buf)
        .map_err(|e| ParseError::Binary(format!("read u8: {e}")))?;
    Ok(buf[0])
}

fn read_u16(cur: &mut Cursor<&[u8]>) -> Result<u16, ParseError> {
    let mut buf = [0u8; 2];
    cur.read_exact(&mut buf)
        .map_err(|e| ParseError::Binary(format!("read u16: {e}")))?;
    Ok(u16::from_le_bytes(buf))
}

fn read_i16(cur: &mut Cursor<&[u8]>) -> Result<i16, ParseError> {
    let mut buf = [0u8; 2];
    cur.read_exact(&mut buf)
        .map_err(|e| ParseError::Binary(format!("read i16: {e}")))?;
    Ok(i16::from_le_bytes(buf))
}

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

fn read_f32(cur: &mut Cursor<&[u8]>) -> Result<f32, ParseError> {
    let mut buf = [0u8; 4];
    cur.read_exact(&mut buf)
        .map_err(|e| ParseError::Binary(format!("read f32: {e}")))?;
    Ok(f32::from_le_bytes(buf))
}

fn read_f64(cur: &mut Cursor<&[u8]>) -> Result<f64, ParseError> {
    let mut buf = [0u8; 8];
    cur.read_exact(&mut buf)
        .map_err(|e| ParseError::Binary(format!("read f64: {e}")))?;
    Ok(f64::from_le_bytes(buf))
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
        write_u32(&mut data, 99); // type 99 = unknown — must error
        // Pad enough bytes so we don't fail on a different read first.
        data.extend_from_slice(&[0u8; 64]);

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
