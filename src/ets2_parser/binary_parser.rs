//! Binary ETS2 map sector parser.
//!
//! Supports two sector layouts found in ETS2 data:
//! - legacy TruckLib-style items (`u32 item_type` + item-specific body)
//! - sized items (`u32 item_type`, `u32 item_size`, payload bytes)
//! - new ETS2 1.50+ items (item_type > 48, treated as sized with graceful skip)
//!
//! We parse roads/prefabs/nodes and skip all other item kinds.

use std::collections::HashMap;

use crate::json_export::{MapNode, MapPrefab, MapRoad};

const ITEM_TERRAIN: u32 = 1;
const ITEM_BUILDINGS: u32 = 2;
const ITEM_ROAD: u32 = 3;
const ITEM_PREFAB: u32 = 4;
const ITEM_MODEL: u32 = 5;
const ITEM_COMPANY: u32 = 6;
const ITEM_SERVICE: u32 = 7;
const ITEM_CUT_PLANE: u32 = 8;
const ITEM_CITY: u32 = 12;
const ITEM_MAP_OVERLAY: u32 = 18;
const ITEM_FERRY: u32 = 19;
const ITEM_GARAGE: u32 = 22;
const ITEM_TRIGGER: u32 = 34;
const ITEM_FUEL_PUMP: u32 = 35;
const ITEM_SIGN: u32 = 36;
const ITEM_BUS_STOP: u32 = 37;
const ITEM_TRAFFIC_AREA: u32 = 38;
const ITEM_BEZIER_PATCH: u32 = 39;
const ITEM_TRAJECTORY: u32 = 41;
const ITEM_MAP_AREA: u32 = 42;
const ITEM_FAR_MODEL: u32 = 43;
const ITEM_CURVE: u32 = 44;
const ITEM_CUTSCENE: u32 = 46;
const ITEM_VISIBILITY_AREA: u32 = 48;

const MAX_LIST_COUNT: u32 = 2_000_000;
const MAX_PASCAL_STRING_LEN: u64 = 1_048_576;

/// Parsed data for a single binary map sector.
#[derive(Debug, Clone, Default)]
pub struct SectorData {
    /// Nodes contained in this sector.
    pub nodes: Vec<MapNode>,
    /// Roads contained in this sector.
    pub roads: Vec<MapRoad>,
    /// Prefabs (intersections, junctions, …) contained in this sector.
    pub prefabs: Vec<MapPrefab>,
}

/// Parse a binary ETS2 sector into structured data.
///
/// Dispatch order:
/// 1. `parse_sized_sector`  — when the header looks like `type(4)+size(4)+payload`.
/// 2. `parse_new_format_sector` — ETS2 1.50+ format: item_count nodes at the
///    start (type+uid+x+y+z+rot, 28 bytes each), followed by road/prefab data.
/// 3. `parse_legacy_sector` — TruckLib-style items; unknown types (> 48) are
///    treated as sized blocks and skipped gracefully.
pub fn parse_binary_sector(data: &[u8]) -> Result<SectorData, String> {
    if data.len() < 24 {
        return Err("sector data too short for header".to_string());
    }

    if looks_like_sized_items(data) {
        return parse_sized_sector(data);
    }

    // Detect ETS2 1.50+ new format: item_type > 48 and declared size exceeds
    // the data length (no actual size field — the u32 after the type is the
    // first 4 bytes of the UID, not a size).
    if looks_like_new_format(data) {
        return parse_new_format_sector(data);
    }

    parse_legacy_sector(data)
}

/// Returns true when the sector uses the ETS2 1.50+ node-first layout.
///
/// Heuristic: item_count is small (≤ 1000), the first item_type is > 48,
/// and the u32 that would be `item_size` in a sized layout is larger than
/// the remaining data (proving there is no size field).
fn looks_like_new_format(data: &[u8]) -> bool {
    if data.len() < 32 {
        return false;
    }
    let mut reader = Reader::new(data);
    // Skip version(4) + game_id(8) + map_version(8)
    if reader.read_u32().is_err() || reader.read_u64().is_err() || reader.read_u64().is_err() {
        return false;
    }
    let item_count = match reader.read_u32() {
        Ok(v) if v > 0 && v <= 1000 => v,
        _ => return false,
    };
    let item_type = match reader.read_u32() {
        Ok(v) => v,
        Err(_) => return false,
    };
    if item_type <= 48 {
        return false;
    }
    // The next u32 would be item_size in a sized layout.
    let pseudo_size = match reader.read_u32() {
        Ok(v) => v,
        Err(_) => return false,
    };
    // If pseudo_size > remaining data, there is no size field — new format.
    let remaining = data.len().saturating_sub(reader.pos);
    pseudo_size as usize > remaining && item_count <= 1000
}

/// Parse the ETS2 1.50+ sector format.
///
/// Layout:
/// - Header: version(4) + game_id(8) + map_version(8) + node_count(4) = 24 bytes
/// - node_count × Node: type(4) + uid(8) + x(4) + y(4) + z(4) + rot(4) = 28 bytes
/// - Remaining data (roads, prefabs, node-uid list) — partially parsed via
///   the legacy road/prefab reader on the remaining bytes.
///
/// We extract nodes from the header section and attempt to parse roads and
/// prefabs from the remaining data using the legacy sized-item reader.
fn parse_new_format_sector(data: &[u8]) -> Result<SectorData, String> {
    let mut reader = Reader::new(data);
    let _version = reader.read_u32()?;
    let _game_id = reader.read_u64()?;
    let _map_version = reader.read_u64()?;
    let node_count = reader.read_u32()?;

    if node_count > 1000 {
        return Ok(SectorData::default());
    }

    let mut nodes = Vec::with_capacity(node_count as usize);
    for _ in 0..node_count {
        if reader.remaining() < 28 {
            break;
        }
        let _item_type = reader.read_u32()?;
        let uid = reader.read_u64()?;
        // Coordinates are stored as f32 in the new format.
        let x = reader.read_f32()? as f64;
        let y = reader.read_f32()? as f64;
        let z = reader.read_f32()? as f64;
        let _rot = reader.read_f32()?;

        if is_valid_uid(uid) && is_plausible_coord(x, y, z) {
            nodes.push(MapNode { uid, x, y, z });
        }
    }

    // After the node block, scan the remaining data for road connections.
    // In ETS2 1.50+, roads are stored as consecutive 16-byte entries
    // (start_uid u64 + end_uid u64). We extract all plausible pairs;
    // the caller filters by known UIDs and deduplicates.
    let residual = &data[reader.pos..];
    let roads = extract_road_pairs_from_residual(residual);

    Ok(SectorData {
        nodes,
        roads,
        prefabs: Vec::new(),
    })
}

/// Extract road candidates from the residual data after the node block.
///
/// Scans 8-byte-aligned offsets for consecutive u64 pairs where both values
/// are plausible UIDs (≥ 0x1000000000000000). The caller is responsible for
/// filtering against a known-UID set and deduplication.
///
/// Capped at 100 000 roads per sector to prevent blow-up from false positives.
fn extract_road_pairs_from_residual(data: &[u8]) -> Vec<MapRoad> {
    const MAX_ROADS: usize = 100_000;
    let mut roads = Vec::new();
    let mut off = 0usize;
    while off + 16 <= data.len() && roads.len() < MAX_ROADS {
        let bytes1: [u8; 8] = match data[off..off + 8].try_into() {
            Ok(b) => b,
            Err(_) => break,
        };
        let bytes2: [u8; 8] = match data[off + 8..off + 16].try_into() {
            Ok(b) => b,
            Err(_) => break,
        };
        let u1 = u64::from_le_bytes(bytes1);
        let u2 = u64::from_le_bytes(bytes2);

        // Both must be plausible ETS2 UIDs (real UIDs are above 0x1000000000000000).
        if u1 >= 0x1000000000000000 && u2 >= 0x1000000000000000 && u1 != u2 {
            roads.push(MapRoad {
                uid: format!("0x{u1:016X}-0x{u2:016X}"),
                name: String::new(),
                look_token: String::new(),
                nodes: vec![u1, u2],
                speed_limit: Some(80.0),
                lane_count_forward: 1,
                lane_count_backward: 1,
            });
            off += 16;
        } else {
            off += 8;
        }
    }
    roads
}

fn looks_like_sized_items(data: &[u8]) -> bool {
    let mut reader = Reader::new(data);
    if reader.read_u32().is_err() || reader.read_u64().is_err() || reader.read_u64().is_err() {
        return false;
    }

    let item_count = match reader.read_u32() {
        Ok(v) if v <= MAX_LIST_COUNT => v,
        _ => return false,
    };

    let checks = item_count.min(8);
    for _ in 0..checks {
        let _item_type = match reader.read_u32() {
            Ok(v) => v,
            Err(_) => return false,
        };
        let item_size = match reader.read_u32() {
            Ok(v) => v,
            Err(_) => return false,
        };
        let item_start = reader.pos;
        let item_end = match resolve_item_end(reader.data.len(), item_start, item_size) {
            Ok(v) => v,
            Err(_) => return false,
        };
        if reader.seek_to(item_end).is_err() {
            return false;
        }
    }

    true
}

fn parse_sized_sector(data: &[u8]) -> Result<SectorData, String> {
    let mut reader = Reader::new(data);
    let _version = reader.read_u32()?;
    let _game_id = reader.read_u64()?;
    let _map_version = reader.read_u64()?;

    let item_count = reader.read_u32()?;
    ensure_count(item_count)?;

    let mut roads = Vec::new();
    let mut prefabs = Vec::new();

    for _ in 0..item_count {
        let item_type = reader.read_u32()?;
        let item_size = reader.read_u32()?;
        let item_start = reader.pos;
        // item_size is treated as payload length (excluding the type/size header).
        let item_end = resolve_item_end(reader.data.len(), item_start, item_size)?;

        match item_type {
            ITEM_ROAD => roads.push(parse_road_item(&mut reader)?),
            ITEM_PREFAB => prefabs.push(parse_prefab_item(&mut reader)?),
            ITEM_COMPANY | ITEM_CITY | ITEM_FERRY | ITEM_FUEL_PUMP | ITEM_SIGN => {
                // Optional: skip for now
            }
            _ => {}
        }

        if reader.pos > item_end {
            return Err(format!(
                "item type {item_type} overran payload at offset {item_start}"
            ));
        }
        reader.seek_to(item_end)?;
    }

    let nodes = parse_nodes_after_items(&mut reader)?;

    Ok(SectorData {
        nodes,
        roads,
        prefabs,
    })
}

fn parse_legacy_sector(data: &[u8]) -> Result<SectorData, String> {
    let mut reader = Reader::new(data);
    let _version = reader.read_u32()?;
    let _game_id = reader.read_u64()?;
    let _map_version = reader.read_u32()?;

    let item_count = reader.read_u32()?;
    ensure_count(item_count)?;

    let mut roads = Vec::new();
    let mut prefabs = Vec::new();
    let mut node_flags = HashMap::new();

    for _ in 0..item_count {
        let item_type = reader.read_u32()?;
        match item_type {
            ITEM_ROAD => roads.push(parse_road_from_reader(&mut reader)?),
            ITEM_PREFAB => prefabs.push(parse_prefab_from_reader(&mut reader)?),
            ITEM_TERRAIN => skip_terrain(&mut reader)?,
            ITEM_BUILDINGS => skip_buildings(&mut reader)?,
            ITEM_MODEL => skip_model(&mut reader)?,
            ITEM_BEZIER_PATCH => skip_bezier_patch(&mut reader)?,
            ITEM_CURVE => skip_curve(&mut reader)?,
            ITEM_FAR_MODEL => skip_far_model(&mut reader)?,
            ITEM_COMPANY => skip_company(&mut reader)?,
            ITEM_CITY => skip_city(&mut reader)?,
            ITEM_FERRY => skip_ferry(&mut reader)?,
            ITEM_FUEL_PUMP => skip_fuel_pump(&mut reader)?,
            ITEM_SIGN => skip_sign(&mut reader)?,
            ITEM_BUS_STOP => skip_bus_stop(&mut reader)?,
            ITEM_CUT_PLANE => skip_cut_plane(&mut reader)?,
            ITEM_CUTSCENE => skip_cutscene(&mut reader)?,
            ITEM_GARAGE => skip_garage(&mut reader)?,
            ITEM_MAP_OVERLAY => skip_map_overlay(&mut reader)?,
            ITEM_MAP_AREA => skip_map_area(&mut reader)?,
            ITEM_SERVICE => skip_service(&mut reader)?,
            ITEM_TRAFFIC_AREA => skip_traffic_area(&mut reader)?,
            ITEM_TRAJECTORY => skip_trajectory(&mut reader)?,
            ITEM_TRIGGER => skip_trigger(&mut reader)?,
            ITEM_VISIBILITY_AREA => skip_visibility_area(&mut reader)?,
            _ => {
                // ETS2 1.50+ introduces new item types (> 48) that are stored
                // as sized blocks: item_type(4) + item_size(4) + payload.
                // We read the size and skip the payload so the rest of the
                // sector (nodes tail) can still be parsed.
                let item_size = match reader.read_u32() {
                    Ok(s) => s,
                    Err(_) => {
                        // Cannot read size — sector is malformed; return what
                        // we have so far rather than propagating an error.
                        return Ok(SectorData {
                            nodes: Vec::new(),
                            roads,
                            prefabs,
                        });
                    }
                };
                let item_start = reader.pos;
                let item_end = match resolve_item_end(reader.data.len(), item_start, item_size) {
                    Ok(e) => e,
                    Err(_) => {
                        // Size exceeds remaining data — sector uses a format
                        // we cannot skip safely; return partial results.
                        return Ok(SectorData {
                            nodes: Vec::new(),
                            roads,
                            prefabs,
                        });
                    }
                };
                reader.seek_to(item_end)?;
            }
        }
    }

    let nodes = parse_nodes_tail(&mut reader, &mut node_flags)?;
    let roads = apply_lane_counts(roads, &node_flags);

    Ok(SectorData {
        nodes,
        roads,
        prefabs,
    })
}

fn parse_nodes_tail(
    reader: &mut Reader<'_>,
    node_flags: &mut HashMap<u64, u32>,
) -> Result<Vec<MapNode>, String> {
    let node_count = reader.read_u32()?;
    ensure_count(node_count)?;
    ensure_capacity(reader, node_count, 56, "nodes")?;

    let mut nodes = Vec::with_capacity(node_count as usize);
    for _ in 0..node_count {
        if let Some(node) = parse_node(reader, node_flags)? {
            nodes.push(node);
        }
    }

    let vis_area_child_count = reader.read_u32()?;
    ensure_count(vis_area_child_count)?;
    ensure_capacity(reader, vis_area_child_count, 8, "vis area children")?;
    for _ in 0..vis_area_child_count {
        let _ = reader.read_u64()?;
    }

    Ok(nodes)
}

fn parse_nodes_after_items(reader: &mut Reader<'_>) -> Result<Vec<MapNode>, String> {
    // Sized format uses f64 positions (per spec: uid u64, x/y/z f64, rot f32)
    const NODE_SIZE: usize = 8 + 8 + 8 + 8 + 4; // uid + 3*f64 + f32

    if reader.remaining() == 0 {
        return Ok(Vec::new());
    }

    let start_pos = reader.pos;
    let mut count_opt: Option<u32> = None;

    if reader.remaining() >= 4 {
        let pos = reader.pos;
        let count = reader.read_u32()?;
        if count <= MAX_LIST_COUNT {
            let needed = (count as usize).saturating_mul(NODE_SIZE);
            if needed <= reader.remaining() {
                count_opt = Some(count);
            }
        }
        if count_opt.is_none() {
            reader.pos = pos;
        }
    }

    let count = if let Some(count) = count_opt {
        count as usize
    } else {
        let remaining = reader.remaining();
        if remaining.is_multiple_of(NODE_SIZE) {
            remaining / NODE_SIZE
        } else {
            0
        }
    };

    if count == 0 {
        reader.pos = start_pos;
        return Ok(Vec::new());
    }

    let mut nodes = Vec::with_capacity(count);
    for _ in 0..count {
        if let Some(node) = parse_simple_node_f64(reader)? {
            nodes.push(node);
        }
    }

    Ok(nodes)
}

fn parse_simple_node_f64(reader: &mut Reader<'_>) -> Result<Option<MapNode>, String> {
    let uid = reader.read_u64()?;
    let x = reader.read_f64()?;
    let y = reader.read_f64()?;
    let z = reader.read_f64()?;
    let _rotation = reader.read_f32()?;

    if !is_valid_uid(uid) || !is_plausible_coord(x, y, z) {
        return Ok(None);
    }

    Ok(Some(MapNode { uid, x, y, z }))
}

fn parse_node(
    reader: &mut Reader<'_>,
    node_flags: &mut HashMap<u64, u32>,
) -> Result<Option<MapNode>, String> {
    let uid = reader.read_u64()?;
    let fx = reader.read_i32()?;
    let fy = reader.read_i32()?;
    let fz = reader.read_i32()?;

    let x = fx as f64 / 256.0;
    let y = fy as f64 / 256.0;
    let z = fz as f64 / 256.0;

    skip_quaternion(reader)?;

    let _backward = reader.read_u64()?;
    let _forward = reader.read_u64()?;
    let flags = reader.read_u32()?;

    if !is_valid_uid(uid) || !is_plausible_coord(x, y, z) {
        return Ok(None);
    }

    node_flags.entry(uid).or_insert(flags);

    Ok(Some(MapNode { uid, x, y, z }))
}

// Road item payload offsets:
// 0x00 start_node_uid (u64)
// 0x08 end_node_uid (u64)
// 0x10 length_m (f32)
// 0x14 lane_count_forward (u32)
// 0x18 lane_count_backward (u32)
// 0x1C speed_limit (f32)
fn parse_road_item(reader: &mut Reader<'_>) -> Result<MapRoad, String> {
    let start_node_uid = reader.read_u64()?;
    let end_node_uid = reader.read_u64()?;
    let _length = reader.read_f32()? as f64;
    let lane_count_forward = reader.read_u32()?;
    let lane_count_backward = reader.read_u32()?;
    let speed_limit_raw = reader.read_f32()? as f64;
    let speed_limit = if speed_limit_raw.is_finite() && speed_limit_raw > 0.0 {
        speed_limit_raw
    } else {
        80.0
    };

    let mut nodes = Vec::with_capacity(2);
    if is_valid_uid(start_node_uid) {
        nodes.push(start_node_uid);
    }
    if is_valid_uid(end_node_uid) {
        nodes.push(end_node_uid);
    }

    Ok(MapRoad {
        uid: format!("0x{start_node_uid:016X}-0x{end_node_uid:016X}"),
        name: String::new(),
        look_token: String::new(),
        nodes,
        speed_limit: Some(speed_limit),
        lane_count_forward,
        lane_count_backward,
    })
}

// Prefab item payload offsets:
// 0x00 uid (u64)
// 0x08 descriptor_token (u64)
// 0x10 node_count (u32)
// 0x14 node_uids (u64 * node_count)
fn parse_prefab_item(reader: &mut Reader<'_>) -> Result<MapPrefab, String> {
    let uid = reader.read_u64()?;
    let _descriptor_token = reader.read_u64()?;
    let node_count = reader.read_u32()?;
    ensure_count(node_count)?;
    ensure_capacity(reader, node_count, 8, "prefab nodes")?;

    let mut nodes = Vec::with_capacity(node_count as usize);
    for _ in 0..node_count {
        let node_uid = reader.read_u64()?;
        if is_valid_uid(node_uid) {
            nodes.push(node_uid);
        }
    }

    Ok(MapPrefab {
        uid: format!("0x{uid:016X}"),
        nodes,
    })
}

fn parse_road_from_reader(reader: &mut Reader<'_>) -> Result<MapRoad, String> {
    let kdop_uid = reader.read_u64()?;
    skip_kdop_bounds(reader)?;

    let _kdop_flag1 = reader.read_u8()?;
    let _kdop_flag2 = reader.read_u8()?;
    let _kdop_flag3 = reader.read_u8()?;
    let _kdop_flag4 = reader.read_u8()?;
    let _view_distance = reader.read_u8()?;

    let _road_flag1 = reader.read_u8()?;
    let _dlc_guard = reader.read_u8()?;
    let _road_flag3 = reader.read_u8()?;
    let _road_flag4 = reader.read_u8()?;

    for _ in 0..11 {
        skip_token(reader)?;
    }
    let _right_terrain_coef = reader.read_f32()?;
    skip_token(reader)?;
    let _left_terrain_coef = reader.read_f32()?;
    skip_token(reader)?;
    skip_token(reader)?;
    skip_token(reader)?;

    for _ in 0..3 {
        skip_token(reader)?;
        let _ = reader.read_i16()?;
        skip_token(reader)?;
        let _ = reader.read_i16()?;
    }

    let _right_height_offset = reader.read_i32()?;
    let _left_height_offset = reader.read_i32()?;

    let backward_node = reader.read_u64()?;
    let forward_node = reader.read_u64()?;
    let length = reader.read_f32()? as f64;

    let mut nodes = Vec::with_capacity(2);
    if is_valid_uid(backward_node) {
        nodes.push(backward_node);
    }
    if is_valid_uid(forward_node) {
        nodes.push(forward_node);
    }

    Ok(MapRoad {
        uid: format!("0x{kdop_uid:016X}"),
        name: String::new(),
        look_token: String::new(),
        nodes,
        speed_limit: Some(if length.is_finite() { 80.0 } else { 0.0 }),
        lane_count_forward: 0,
        lane_count_backward: 0,
    })
}

fn parse_prefab_from_reader(reader: &mut Reader<'_>) -> Result<MapPrefab, String> {
    let kdop_uid = read_kdop_item(reader)?;

    let _model_token = reader.read_u64()?;
    skip_token(reader)?;

    let add_part_count = reader.read_u32()?;
    ensure_count(add_part_count)?;
    ensure_capacity(reader, add_part_count, 8, "prefab add parts")?;
    for _ in 0..add_part_count {
        skip_token(reader)?;
    }

    let node_count = reader.read_u32()?;
    ensure_count(node_count)?;
    ensure_capacity(reader, node_count, 8, "prefab nodes")?;
    let mut nodes = Vec::with_capacity(node_count as usize);
    for _ in 0..node_count {
        let node_uid = reader.read_u64()?;
        if is_valid_uid(node_uid) {
            nodes.push(node_uid);
        }
    }

    let slave_count = reader.read_u32()?;
    ensure_count(slave_count)?;
    ensure_capacity(reader, slave_count, 8, "prefab slaves")?;
    for _ in 0..slave_count {
        let _ = reader.read_u64()?;
    }

    let _ferry_link = reader.read_u64()?;
    let _origin_idx = reader.read_u16()?;

    for _ in 0..node_count {
        skip_token(reader)?;
        let _ = reader.read_f32()?;
    }

    skip_token(reader)?;

    Ok(MapPrefab {
        uid: format!("0x{kdop_uid:016X}"),
        nodes,
    })
}

fn skip_terrain(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    skip_token(reader)?;
    skip_token(reader)?;
    skip_token(reader)?;
    let _ = reader.read_u16()?;
    skip_token(reader)?;
    let _ = reader.read_u16()?;
    let _ = reader.read_u32()?;
    let _ = reader.read_u32()?;
    skip_float_list(reader)?;
    skip_float_list(reader)?;
    skip_float_list(reader)
}

fn skip_buildings(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    skip_token(reader)?;
    let _ = reader.read_u64()?;
    skip_token(reader)?;
    let _ = reader.read_u64()?;
    let _ = reader.read_u32()?;
    let _ = reader.read_u32()?;
    Ok(())
}

fn skip_model(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    let _ = reader.read_u64()?;
    let _ = reader.read_u64()?;
    let _ = reader.read_u64()?;
    skip_token(reader)?;
    let _ = reader.read_u8()?;
    Ok(())
}

fn skip_bezier_patch(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    let _ = reader.read_u64()?;
    let _ = reader.read_u64()?;
    for _ in 0..4 {
        skip_vector3(reader)?;
    }
    Ok(())
}

fn skip_curve(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    let _ = reader.read_u64()?;
    let _ = reader.read_u64()?;
    skip_vector3(reader)?;
    skip_vector3(reader)?;
    let _ = reader.read_f32()?;
    let _ = reader.read_f32()?;
    let _ = reader.read_u32()?;
    for _ in 0..3 {
        skip_token(reader)?;
        let _ = reader.read_i16()?;
    }
    for _ in 0..2 {
        let _ = reader.read_u16()?;
        skip_token(reader)?;
        let _ = reader.read_f32()?;
        skip_token(reader)?;
        let _ = reader.read_f32()?;
        for _ in 0..3 {
            skip_road_vegetation(reader)?;
        }
        let _ = reader.read_u16()?;
        let _ = reader.read_u16()?;
    }
    skip_vegetation_sphere_list(reader)?;
    skip_terrain_quad_data(reader)?;
    skip_terrain_quad_data(reader)?;
    skip_token(reader)?;
    skip_token(reader)?;
    skip_token(reader)?;
    skip_token(reader)?;
    Ok(())
}

fn skip_far_model(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    skip_token(reader)?;
    skip_token(reader)?;
    skip_token(reader)?;
    let _ = reader.read_u64()?;
    Ok(())
}

fn skip_company(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    let _ = reader.read_u64()?;
    let _ = reader.read_u64()?;
    let _ = reader.read_u64()?;
    let _ = reader.read_u64()?;
    let spawn_count = reader.read_u32()?;
    ensure_count(spawn_count)?;
    ensure_capacity(reader, spawn_count, 8, "company spawn nodes")?;
    for _ in 0..spawn_count {
        let _ = reader.read_u64()?;
    }
    ensure_capacity(reader, spawn_count, 4, "company spawn counters")?;
    for _ in 0..spawn_count {
        let _ = reader.read_u32()?;
    }
    Ok(())
}

fn skip_city(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    let _ = reader.read_u64()?;
    let _ = reader.read_f32()?;
    let _ = reader.read_f32()?;
    let _ = reader.read_u64()?;
    Ok(())
}

fn skip_ferry(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    let _ = reader.read_u64()?;
    let _ = reader.read_u64()?;
    let _ = reader.read_u64()?;
    let _ = reader.read_f32()?;
    let _ = reader.read_f32()?;
    let _ = reader.read_f32()?;
    Ok(())
}

fn skip_fuel_pump(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    let _ = reader.read_u64()?;
    let _ = reader.read_u64()?;
    let node_count = skip_node_ref_list(reader)?;
    for _ in 0..node_count {
        let _ = reader.read_u64()?;
    }
    Ok(())
}

fn skip_sign(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    let _ = reader.read_u64()?;
    let _ = reader.read_u64()?;
    let _ = reader.read_u64()?;
    let _ = reader.read_u64()?;
    let board_count = reader.read_u8()? as u32;
    for _ in 0..board_count {
        skip_token(reader)?;
        skip_token(reader)?;
    }
    skip_sign_board_override_list(reader)?;
    skip_sign_override_list(reader)
}

fn skip_bus_stop(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    let _ = reader.read_u64()?;
    let _ = reader.read_u64()?;
    let _ = reader.read_u64()?;
    Ok(())
}

fn skip_cut_plane(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    let _ = skip_node_ref_list(reader)?;
    Ok(())
}

fn skip_cutscene(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    skip_token_list(reader)?;
    let _ = reader.read_u64()?;
    skip_action_list(reader, false)
}

fn skip_garage(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    skip_token(reader)?;
    let _ = reader.read_u32()?;
    let _ = reader.read_u64()?;
    let _ = reader.read_u64()?;
    let _ = skip_node_ref_list(reader)?;
    Ok(())
}

fn skip_map_overlay(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    skip_token(reader)?;
    let _ = reader.read_u64()?;
    Ok(())
}

fn skip_map_area(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    let _ = skip_node_ref_list(reader)?;
    let _ = reader.read_u32()?;
    Ok(())
}

fn skip_service(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    let _ = reader.read_u64()?;
    let _ = reader.read_u64()?;
    let _ = skip_node_ref_list(reader)?;
    Ok(())
}

fn skip_traffic_area(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    skip_token_list(reader)?;
    let _ = skip_node_ref_list(reader)?;
    skip_token(reader)?;
    let _ = reader.read_f32()?;
    Ok(())
}

fn skip_trajectory(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    let _ = skip_node_ref_list(reader)?;
    skip_token(reader)?;
    skip_trajectory_rule_list(reader)?;
    let checkpoint_count = reader.read_u32()?;
    ensure_count(checkpoint_count)?;
    for _ in 0..checkpoint_count {
        skip_token(reader)?;
        skip_token(reader)?;
    }
    skip_token_list(reader)
}

fn skip_trigger(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    skip_token_list(reader)?;
    let node_count = skip_node_ref_list(reader)?;
    skip_action_list(reader, true)?;
    if node_count == 1 {
        let _ = reader.read_f32()?;
    }
    Ok(())
}

fn skip_visibility_area(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = read_kdop_item(reader)?;
    let _ = reader.read_u64()?;
    let _ = reader.read_f32()?;
    let _ = reader.read_f32()?;
    skip_item_ref_list(reader)
}

fn read_kdop_item(reader: &mut Reader<'_>) -> Result<u64, String> {
    let uid = reader.read_u64()?;
    skip_kdop_bounds(reader)?;
    let _flags = reader.read_u32()?;
    let _view_distance = reader.read_u8()?;
    Ok(uid)
}

fn skip_kdop_bounds(reader: &mut Reader<'_>) -> Result<(), String> {
    for _ in 0..10 {
        let _ = reader.read_f32()?;
    }
    Ok(())
}

fn skip_quaternion(reader: &mut Reader<'_>) -> Result<(), String> {
    for _ in 0..4 {
        let _ = reader.read_f32()?;
    }
    Ok(())
}

fn skip_token(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = reader.read_u64()?;
    Ok(())
}

fn skip_pascal_string(reader: &mut Reader<'_>) -> Result<(), String> {
    let len = reader.read_u64()?;
    if len > MAX_PASCAL_STRING_LEN {
        return Err(format!("pascal string length {len} exceeds safety limit"));
    }
    if len > usize::MAX as u64 {
        return Err(format!("string length {len} exceeds addressable memory"));
    }
    reader.skip(len as usize)
}

fn skip_token_list(reader: &mut Reader<'_>) -> Result<(), String> {
    let count = reader.read_u32()?;
    ensure_count(count)?;
    ensure_capacity(reader, count, 8, "token list")?;
    for _ in 0..count {
        skip_token(reader)?;
    }
    Ok(())
}

fn skip_node_ref_list(reader: &mut Reader<'_>) -> Result<u32, String> {
    let count = reader.read_u32()?;
    ensure_count(count)?;
    ensure_capacity(reader, count, 8, "node refs")?;
    for _ in 0..count {
        let _ = reader.read_u64()?;
    }
    Ok(count)
}

fn skip_item_ref_list(reader: &mut Reader<'_>) -> Result<(), String> {
    let count = reader.read_u32()?;
    ensure_count(count)?;
    ensure_capacity(reader, count, 8, "item refs")?;
    for _ in 0..count {
        let _ = reader.read_u64()?;
    }
    Ok(())
}

fn skip_trajectory_rule_list(reader: &mut Reader<'_>) -> Result<(), String> {
    let count = reader.read_u32()?;
    ensure_count(count)?;
    for _ in 0..count {
        let _ = reader.read_u32()?;
        skip_token(reader)?;
        let param_count = reader.read_u32()?;
        ensure_count(param_count)?;
        for _ in 0..param_count {
            let _ = reader.read_f32()?;
        }
    }
    Ok(())
}

fn skip_action_list(reader: &mut Reader<'_>, include_name: bool) -> Result<(), String> {
    let count = reader.read_u32()?;
    ensure_count(count)?;
    for _ in 0..count {
        if include_name {
            skip_token(reader)?;
        }
        skip_action_base(reader)?;
    }
    Ok(())
}

fn skip_action_base(reader: &mut Reader<'_>) -> Result<(), String> {
    let num_param_count = reader.read_u32()?;
    ensure_count(num_param_count)?;
    ensure_capacity(reader, num_param_count, 4, "action numeric params")?;
    for _ in 0..num_param_count {
        let _ = reader.read_f32()?;
    }

    let string_param_count = reader.read_u32()?;
    ensure_count(string_param_count)?;
    for _ in 0..string_param_count {
        skip_pascal_string(reader)?;
    }

    let target_tag_count = reader.read_u32()?;
    ensure_count(target_tag_count)?;
    for _ in 0..target_tag_count {
        skip_token(reader)?;
    }

    let _ = reader.read_f32()?;
    let _ = reader.read_u32()?;
    Ok(())
}

fn skip_sign_board_override_list(reader: &mut Reader<'_>) -> Result<(), String> {
    let count = reader.read_u32()?;
    ensure_count(count)?;
    for _ in 0..count {
        skip_token(reader)?;
        let flags = reader.read_u8()?;
        if flags & 0x01 != 0 {
            let _ = reader.read_u8()?;
            let _ = reader.read_u8()?;
        }
        if flags & 0x02 != 0 {
            skip_token(reader)?;
        }
    }
    Ok(())
}

fn skip_sign_override_list(reader: &mut Reader<'_>) -> Result<(), String> {
    let count = reader.read_u32()?;
    ensure_count(count)?;
    for _ in 0..count {
        let _ = reader.read_u32()?;
        skip_token(reader)?;
        let attr_count = reader.read_u32()?;
        ensure_count(attr_count)?;
        for _ in 0..attr_count {
            let attr_type = reader.read_u16()?;
            let _ = reader.read_u32()?;
            match attr_type {
                1 => {
                    let _ = reader.read_u8()?;
                }
                2 => {
                    let _ = reader.read_i32()?;
                }
                3 => {
                    let _ = reader.read_u32()?;
                }
                4 => {
                    let _ = reader.read_f32()?;
                }
                5 => {
                    skip_pascal_string(reader)?;
                }
                6 => {
                    let _ = reader.read_u64()?;
                }
                _ => {
                    return Err(format!("unknown sign override attribute type {attr_type}"));
                }
            }
        }
    }
    Ok(())
}

fn skip_road_vegetation(reader: &mut Reader<'_>) -> Result<(), String> {
    skip_token(reader)?;
    let _ = reader.read_u16()?;
    let _ = reader.read_u8()?;
    let _ = reader.read_u8()?;
    let _ = reader.read_u16()?;
    let _ = reader.read_u16()?;
    Ok(())
}

fn skip_vegetation_sphere_list(reader: &mut Reader<'_>) -> Result<(), String> {
    let count = reader.read_u32()?;
    ensure_count(count)?;
    ensure_capacity(reader, count, 16, "vegetation spheres")?;
    for _ in 0..count {
        skip_vector3(reader)?;
        let _ = reader.read_f32()?;
        let _ = reader.read_u32()?;
    }
    Ok(())
}

fn skip_terrain_quad_data(reader: &mut Reader<'_>) -> Result<(), String> {
    let brush_mat_count = reader.read_u16()?;
    for _ in 0..brush_mat_count {
        skip_token(reader)?;
        let _ = reader.read_u16()?;
    }

    let color_count = reader.read_u16()?;
    for _ in 0..color_count {
        skip_color(reader)?;
    }

    let _rows = reader.read_u16()?;
    let _cols = reader.read_u16()?;

    let quad_count = reader.read_u32()?;
    ensure_count(quad_count)?;
    for _ in 0..quad_count {
        skip_color(reader)?;
    }

    let offset_count = reader.read_u32()?;
    ensure_count(offset_count)?;
    for _ in 0..offset_count {
        let _ = reader.read_u16()?;
        let _ = reader.read_u16()?;
        skip_vector3(reader)?;
    }

    let normal_count = reader.read_u32()?;
    ensure_count(normal_count)?;
    for _ in 0..normal_count {
        let _ = reader.read_u16()?;
        let _ = reader.read_u16()?;
        skip_vector3(reader)?;
    }

    Ok(())
}

fn skip_float_list(reader: &mut Reader<'_>) -> Result<(), String> {
    let count = reader.read_u32()?;
    ensure_count(count)?;
    ensure_capacity(reader, count, 4, "float list")?;
    for _ in 0..count {
        let _ = reader.read_f32()?;
    }
    Ok(())
}

fn skip_vector3(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = reader.read_f32()?;
    let _ = reader.read_f32()?;
    let _ = reader.read_f32()?;
    Ok(())
}

fn skip_color(reader: &mut Reader<'_>) -> Result<(), String> {
    let _ = reader.read_u8()?;
    let _ = reader.read_u8()?;
    let _ = reader.read_u8()?;
    let _ = reader.read_u8()?;
    Ok(())
}

fn apply_lane_counts(roads: Vec<MapRoad>, node_flags: &HashMap<u64, u32>) -> Vec<MapRoad> {
    roads
        .into_iter()
        .map(|mut road| {
            if road.nodes.len() == 2 {
                let mut forward = 0;
                let mut backward = 0;
                let mut has_flags = false;

                if let Some(flags) = node_flags.get(&road.nodes[0]) {
                    has_flags = true;
                    forward = forward.max(forward_lane_count(*flags));
                    backward = backward.max(backward_lane_count(*flags));
                }

                if let Some(flags) = node_flags.get(&road.nodes[1]) {
                    has_flags = true;
                    forward = forward.max(forward_lane_count(*flags));
                    backward = backward.max(backward_lane_count(*flags));
                }

                if !has_flags || (forward == 0 && backward == 0) {
                    forward = 1;
                    backward = 1;
                }

                road.lane_count_forward = forward;
                road.lane_count_backward = backward;
            }
            road
        })
        .collect()
}

fn has_forward_traffic(flags: u32) -> bool {
    ((flags >> 5) & 1) != 0 || ((flags >> 6) & 1) != 0 || ((flags >> 7) & 1) != 0
}

fn has_backward_traffic(flags: u32) -> bool {
    ((flags >> 28) & 1) != 0 || ((flags >> 29) & 1) != 0 || ((flags >> 30) & 1) != 0
}

fn forward_lane_count(flags: u32) -> u32 {
    let lanes = (flags >> 5) & 0x7;
    if lanes == 0 && has_forward_traffic(flags) {
        1
    } else {
        lanes
    }
}

fn backward_lane_count(flags: u32) -> u32 {
    let lanes = (flags >> 28) & 0x7;
    if lanes == 0 && has_backward_traffic(flags) {
        1
    } else {
        lanes
    }
}

fn is_valid_uid(uid: u64) -> bool {
    uid >= 0x1000
}

fn is_plausible_coord(x: f64, y: f64, z: f64) -> bool {
    x.abs() <= 300_000.0 && y.abs() <= 20_000.0 && z.abs() <= 300_000.0
}

fn ensure_count(count: u32) -> Result<(), String> {
    if count > MAX_LIST_COUNT {
        return Err(format!("count {count} exceeds safety limit"));
    }
    Ok(())
}

fn ensure_capacity(
    reader: &Reader<'_>,
    count: u32,
    elem_size: usize,
    label: &str,
) -> Result<(), String> {
    let need = (count as usize)
        .checked_mul(elem_size)
        .ok_or_else(|| format!("{label} byte size overflow"))?;
    if need > reader.remaining() {
        return Err(format!(
            "not enough bytes for {label}: need {need}, have {}",
            reader.remaining()
        ));
    }
    Ok(())
}

fn resolve_item_end(data_len: usize, item_start: usize, item_size: u32) -> Result<usize, String> {
    let size = item_size as usize;
    let end = item_start
        .checked_add(size)
        .ok_or_else(|| "offset overflow while resolving item end".to_string())?;
    if end > data_len {
        return Err(format!(
            "item size {item_size} out of bounds at offset {item_start} (data_len={data_len})"
        ));
    }
    Ok(end)
}

struct Reader<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0 }
    }

    fn skip(&mut self, len: usize) -> Result<(), String> {
        let end = self
            .pos
            .checked_add(len)
            .ok_or_else(|| "offset overflow while skipping".to_string())?;
        if end > self.data.len() {
            return Err(format!("unexpected end of data at offset {end}"));
        }
        self.pos = end;
        Ok(())
    }

    fn seek_to(&mut self, pos: usize) -> Result<(), String> {
        if pos > self.data.len() {
            return Err(format!("unexpected end of data at offset {pos}"));
        }
        self.pos = pos;
        Ok(())
    }

    fn remaining(&self) -> usize {
        self.data.len().saturating_sub(self.pos)
    }

    fn read_slice(&mut self, len: usize) -> Result<&'a [u8], String> {
        let start = self.pos;
        self.skip(len)?;
        Ok(&self.data[start..self.pos])
    }

    fn read_u8(&mut self) -> Result<u8, String> {
        Ok(self.read_slice(1)?[0])
    }

    fn read_u16(&mut self) -> Result<u16, String> {
        let bytes = self.read_slice(2)?;
        Ok(u16::from_le_bytes([bytes[0], bytes[1]]))
    }

    fn read_i16(&mut self) -> Result<i16, String> {
        let bytes = self.read_slice(2)?;
        Ok(i16::from_le_bytes([bytes[0], bytes[1]]))
    }

    fn read_u32(&mut self) -> Result<u32, String> {
        let bytes = self.read_slice(4)?;
        Ok(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
    }

    fn read_i32(&mut self) -> Result<i32, String> {
        let bytes = self.read_slice(4)?;
        Ok(i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
    }

    fn read_u64(&mut self) -> Result<u64, String> {
        let bytes = self.read_slice(8)?;
        Ok(u64::from_le_bytes([
            bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
        ]))
    }

    fn read_f32(&mut self) -> Result<f32, String> {
        let bytes = self.read_slice(4)?;
        Ok(f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
    }

    fn read_f64(&mut self) -> Result<f64, String> {
        let bytes = self.read_slice(8)?;
        Ok(f64::from_le_bytes([
            bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
        ]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn append_header(buf: &mut Vec<u8>, item_count: u32) {
        buf.extend_from_slice(&906u32.to_le_bytes());
        buf.extend_from_slice(&0u64.to_le_bytes());
        buf.extend_from_slice(&0u64.to_le_bytes());
        buf.extend_from_slice(&item_count.to_le_bytes());
    }

    fn append_nodes_tail(buf: &mut Vec<u8>, nodes: &[(u64, f64, f64, f64, f32)]) {
        buf.extend_from_slice(&(nodes.len() as u32).to_le_bytes());
        for (uid, x, y, z, rot) in nodes {
            buf.extend_from_slice(&uid.to_le_bytes());
            buf.extend_from_slice(&x.to_le_bytes());
            buf.extend_from_slice(&y.to_le_bytes());
            buf.extend_from_slice(&z.to_le_bytes());
            buf.extend_from_slice(&rot.to_le_bytes());
        }
    }

    fn build_synthetic_sector(include_unknown: bool) -> Vec<u8> {
        let mut data = Vec::new();
        let item_count = if include_unknown { 3 } else { 2 };
        append_header(&mut data, item_count);

        data.extend_from_slice(&ITEM_ROAD.to_le_bytes());
        data.extend_from_slice(&44u32.to_le_bytes());
        data.extend_from_slice(&0x1000u64.to_le_bytes());
        data.extend_from_slice(&0x1001u64.to_le_bytes());
        data.extend_from_slice(&100.0f32.to_le_bytes());
        data.extend_from_slice(&2u32.to_le_bytes());
        data.extend_from_slice(&1u32.to_le_bytes());
        data.extend_from_slice(&80.0f32.to_le_bytes());
        data.extend_from_slice(&0u32.to_le_bytes());
        data.extend_from_slice(&0u64.to_le_bytes());

        if include_unknown {
            data.extend_from_slice(&99u32.to_le_bytes());
            data.extend_from_slice(&24u32.to_le_bytes());
            data.extend_from_slice(&[1u8; 16]);
            data.extend_from_slice(&[0u8; 8]);
        }

        data.extend_from_slice(&ITEM_PREFAB.to_le_bytes());
        data.extend_from_slice(&40u32.to_le_bytes());
        data.extend_from_slice(&100u64.to_le_bytes());
        data.extend_from_slice(&200u64.to_le_bytes());
        data.extend_from_slice(&2u32.to_le_bytes());
        data.extend_from_slice(&0x1000u64.to_le_bytes());
        data.extend_from_slice(&0x1001u64.to_le_bytes());
        data.extend_from_slice(&0u32.to_le_bytes());

        append_nodes_tail(
            &mut data,
            &[(0x1000, 0.0, 0.0, 0.0, 0.0), (0x1001, 100.0, 0.0, 0.0, 0.0)],
        );
        data
    }

    #[test]
    fn test_parse_binary_sector_road_prefab() {
        let data = build_synthetic_sector(false);
        let sector = parse_binary_sector(&data).unwrap();

        assert_eq!(sector.nodes.len(), 2);
        assert_eq!(sector.roads.len(), 1);
        assert_eq!(sector.prefabs.len(), 1);
        assert_eq!(sector.roads[0].nodes, vec![0x1000, 0x1001]);
        assert_eq!(sector.roads[0].lane_count_forward, 2);
        assert_eq!(sector.roads[0].lane_count_backward, 1);
        assert_eq!(sector.roads[0].speed_limit, Some(80.0));
        assert_eq!(sector.prefabs[0].nodes, vec![0x1000, 0x1001]);
    }

    #[test]
    fn test_parse_binary_sector_skip_unknown() {
        let data = build_synthetic_sector(true);
        let sector = parse_binary_sector(&data).unwrap();
        assert_eq!(sector.roads.len(), 1);
        assert_eq!(sector.prefabs.len(), 1);
    }

    #[test]
    fn test_parse_binary_empty() {
        let err = parse_binary_sector(&[]).unwrap_err();
        assert!(err.contains("too short"));
    }

    #[test]
    fn test_parse_binary_garbage() {
        let garbage = vec![0xFFu8; 100];
        let err = parse_binary_sector(&garbage).unwrap_err();
        assert!(
            err.contains("count") || err.contains("unsupported") || err.contains("unexpected end")
        );
    }

    #[test]
    fn test_parse_binary_sector_integration() {
        let sector = parse_binary_sector(&build_synthetic_sector(true)).unwrap();
        let map = crate::json_export::MapData {
            nodes: sector.nodes,
            roads: sector.roads,
            prefabs: sector.prefabs,
        };
        let graph = crate::graph_export::build_graph(&map).unwrap();
        assert!(graph.nodes.len() >= 2);
        assert!(
            !graph.edges.is_empty(),
            "expected >=1 edge, got {}",
            graph.edges.len()
        );
    }

    #[test]
    fn test_lane_counts_default_to_one_without_flags() {
        let road = MapRoad {
            uid: "0xTEST".to_string(),
            name: String::new(),
            look_token: String::new(),
            nodes: vec![0x1001, 0x1002],
            speed_limit: None,
            lane_count_forward: 0,
            lane_count_backward: 0,
        };
        let result = apply_lane_counts(vec![road], &HashMap::new());
        assert_eq!(result[0].lane_count_forward, 1);
        assert_eq!(result[0].lane_count_backward, 1);
    }

    #[test]
    fn test_lane_counts_extract_forward_and_backward() {
        let road = MapRoad {
            uid: "0xTEST".to_string(),
            name: String::new(),
            look_token: String::new(),
            nodes: vec![0x1001, 0x1002],
            speed_limit: None,
            lane_count_forward: 0,
            lane_count_backward: 0,
        };
        let mut flags = HashMap::new();
        // node_a: 2 forward lanes, node_b: 3 backward lanes
        flags.insert(0x1001, 2 << 5);
        flags.insert(0x1002, 3 << 28);
        let result = apply_lane_counts(vec![road], &flags);
        assert_eq!(result[0].lane_count_forward, 2);
        assert_eq!(result[0].lane_count_backward, 3);
    }

    #[test]
    fn test_lane_counts_defaults_when_flags_are_zero() {
        let road = MapRoad {
            uid: "0xTEST".to_string(),
            name: String::new(),
            look_token: String::new(),
            nodes: vec![0x1001, 0x1002],
            speed_limit: None,
            lane_count_forward: 0,
            lane_count_backward: 0,
        };
        let mut flags = HashMap::new();
        // flags entry exists but lane bits are all zero -> still defaults to 1/1
        flags.insert(0x1001, 0);
        let result = apply_lane_counts(vec![road], &flags);
        assert_eq!(result[0].lane_count_forward, 1);
        assert_eq!(result[0].lane_count_backward, 1);
    }

    #[test]
    fn test_lane_counts_from_various_flag_bits() {
        let road = MapRoad {
            uid: "0xTEST".to_string(),
            name: String::new(),
            look_token: String::new(),
            nodes: vec![0x1001, 0x1002],
            speed_limit: None,
            lane_count_forward: 0,
            lane_count_backward: 0,
        };
        let mut flags = HashMap::new();
        // bit 6 is the second bit in the 3-bit forward lane field -> lanes = 2
        flags.insert(0x1001, 1 << 6);
        // bit 29 is the second bit in the 3-bit backward lane field -> lanes = 2
        flags.insert(0x1002, 1 << 29);
        let result = apply_lane_counts(vec![road], &flags);
        assert_eq!(result[0].lane_count_forward, 2);
        assert_eq!(result[0].lane_count_backward, 2);
    }

    #[test]
    fn test_lane_counts_max_across_two_nodes() {
        let road = MapRoad {
            uid: "0xTEST".to_string(),
            name: String::new(),
            look_token: String::new(),
            nodes: vec![0x1001, 0x1002],
            speed_limit: None,
            lane_count_forward: 0,
            lane_count_backward: 0,
        };
        let mut flags = HashMap::new();
        flags.insert(0x1001, 1 << 5); // 1 forward lane
        flags.insert(0x1002, 4 << 5); // 4 forward lanes
        let result = apply_lane_counts(vec![road], &flags);
        assert_eq!(result[0].lane_count_forward, 4);
        assert_eq!(result[0].lane_count_backward, 0);
    }

    /// Synthetic sector with item_type=99, size=24, 16 bytes payload + 8 bytes padding.
    /// Verifies that the parser skips the unknown item and returns Ok (no error).
    #[test]
    fn test_unknown_item_type_skip() {
        let mut data = Vec::new();
        // Header: version + game_id + map_version + item_count=1
        data.extend_from_slice(&906u32.to_le_bytes());
        data.extend_from_slice(&0u64.to_le_bytes());
        data.extend_from_slice(&0u64.to_le_bytes());
        data.extend_from_slice(&1u32.to_le_bytes());

        // Unknown item: type=99, size=24, 16 bytes arbitrary payload + 8 bytes padding
        data.extend_from_slice(&99u32.to_le_bytes());
        data.extend_from_slice(&24u32.to_le_bytes());
        data.extend_from_slice(&[0xABu8; 16]);
        data.extend_from_slice(&[0u8; 8]);

        // Node tail: 0 nodes
        data.extend_from_slice(&0u32.to_le_bytes());
        // vis_area_child_count: 0
        data.extend_from_slice(&0u32.to_le_bytes());

        let result = parse_binary_sector(&data);
        assert!(result.is_ok(), "parser must not error on unknown item type");
        let sector = result.unwrap();
        assert!(sector.roads.is_empty());
        assert!(sector.prefabs.is_empty());
        assert!(sector.nodes.is_empty());
    }

    /// Sector where the unknown item's declared size exceeds the remaining data.
    /// Simulates the ETS2 1.50+ format where items have no explicit size field.
    /// The parser must return Ok with empty data instead of an error.
    ///
    /// Uses the legacy header layout (map_version as u32) so that
    /// `parse_legacy_sector` is invoked (not `parse_sized_sector`).
    #[test]
    fn test_unknown_item_type_oversized_returns_empty() {
        let mut data = Vec::new();
        // Legacy header: version(4) + game_id(8) + map_version(4) + item_count(4) = 20 bytes
        data.extend_from_slice(&906u32.to_le_bytes()); // version
        data.extend_from_slice(&0u64.to_le_bytes()); // game_id
        data.extend_from_slice(&0u32.to_le_bytes()); // map_version (u32 in legacy)
        data.extend_from_slice(&1u32.to_le_bytes()); // item_count = 1

        // Unknown item: type=866335 (ProMods-style), size=999_999_999 (way too large)
        data.extend_from_slice(&866335u32.to_le_bytes());
        data.extend_from_slice(&999_999_999u32.to_le_bytes());
        // Only 8 bytes of actual data follow (much less than declared size)
        data.extend_from_slice(&[0u8; 8]);

        let result = parse_binary_sector(&data);
        assert!(
            result.is_ok(),
            "parser must not error when item size exceeds data: {:?}",
            result.err()
        );
        let sector = result.unwrap();
        assert!(sector.roads.is_empty());
        assert!(sector.prefabs.is_empty());
        assert!(sector.nodes.is_empty());
    }

    /// Sector with multiple items: one known Road, one unknown (type=2700766),
    /// one known Prefab. Verifies that known items before and after the unknown
    /// item are still parsed correctly.
    #[test]
    fn test_unknown_item_between_known_items() {
        let mut data = Vec::new();
        // Header: item_count=3
        data.extend_from_slice(&906u32.to_le_bytes());
        data.extend_from_slice(&0u64.to_le_bytes());
        data.extend_from_slice(&0u64.to_le_bytes());
        data.extend_from_slice(&3u32.to_le_bytes());

        // Item 0: Road (type=3, size=44)
        data.extend_from_slice(&ITEM_ROAD.to_le_bytes());
        data.extend_from_slice(&44u32.to_le_bytes());
        data.extend_from_slice(&0x1000u64.to_le_bytes()); // start_node
        data.extend_from_slice(&0x1001u64.to_le_bytes()); // end_node
        data.extend_from_slice(&100.0f32.to_le_bytes()); // length
        data.extend_from_slice(&1u32.to_le_bytes()); // lane_fwd
        data.extend_from_slice(&1u32.to_le_bytes()); // lane_bwd
        data.extend_from_slice(&80.0f32.to_le_bytes()); // speed
        data.extend_from_slice(&0u32.to_le_bytes()); // flags
        data.extend_from_slice(&0u64.to_le_bytes()); // padding

        // Item 1: Unknown (type=2700766, size=16)
        data.extend_from_slice(&2700766u32.to_le_bytes());
        data.extend_from_slice(&16u32.to_le_bytes());
        data.extend_from_slice(&[0xCCu8; 16]);

        // Item 2: Prefab (type=4, size=40)
        data.extend_from_slice(&ITEM_PREFAB.to_le_bytes());
        data.extend_from_slice(&40u32.to_le_bytes());
        data.extend_from_slice(&0x2000u64.to_le_bytes()); // uid
        data.extend_from_slice(&0u64.to_le_bytes()); // descriptor
        data.extend_from_slice(&2u32.to_le_bytes()); // node_count
        data.extend_from_slice(&0x1000u64.to_le_bytes());
        data.extend_from_slice(&0x1001u64.to_le_bytes());
        data.extend_from_slice(&0u32.to_le_bytes()); // padding

        // Node tail: 0 nodes + 0 vis_area_children
        data.extend_from_slice(&0u32.to_le_bytes());
        data.extend_from_slice(&0u32.to_le_bytes());

        let result = parse_binary_sector(&data);
        assert!(result.is_ok(), "parser must not error: {:?}", result.err());
        let sector = result.unwrap();
        assert_eq!(
            sector.roads.len(),
            1,
            "road before unknown item must be parsed"
        );
        assert_eq!(
            sector.prefabs.len(),
            1,
            "prefab after unknown item must be parsed"
        );
    }

    #[test]
    fn test_single_node_road_skips_lane_counts() {
        let road = MapRoad {
            uid: "0xTEST".to_string(),
            name: String::new(),
            look_token: String::new(),
            nodes: vec![0x1001],
            speed_limit: None,
            lane_count_forward: 0,
            lane_count_backward: 0,
        };
        let mut flags = HashMap::new();
        flags.insert(0x1001, 2 << 5);
        let result = apply_lane_counts(vec![road], &flags);
        // lane counts remain zero for roads that don't connect exactly two nodes
        assert_eq!(result[0].lane_count_forward, 0);
        assert_eq!(result[0].lane_count_backward, 0);
    }
}
