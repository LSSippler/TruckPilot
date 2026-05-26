//! PPD (Prism3D Prefab Descriptor) parser — Task DS7.
//!
//! Parses ETS2 `.ppd` binary files extracted from `.scs` archives.
//! These contain pre-baked AI navigation graphs (NavCurves, NavNodes)
//! that describe how the in-game AI traffic traverses junctions,
//! roundabouts, and service areas.
//!
//! ## Format overview
//!
//! PPD files have a 96-byte header followed by up to 12 sections:
//!
//! ```text
//! HEADER (96 bytes, little-endian, packed):
//!   0x00  u32      version          0x15..0x19 (21..25)
//!   0x04  u32      node_count       ControlNodes
//!   0x08  u32      nav_curve_count
//!   0x0C  u32      sign_count
//!   0x10  u32      semaphore_count
//!   0x14  u32      spawn_point_count
//!   0x18  u32      terrain_point_count
//!   0x1C  u32      terrain_point_variant_count
//!   0x20  u32      map_point_count
//!   0x24  u32      trigger_point_count
//!   0x28  u32      intersection_count
//!   0x2C  u32      nav_node_count    (0 in v15)
//!   0x30  u32[12]  section_offsets   byte offsets to each section
//! ```
//!
//! Sections (in disk order):
//!   1. ControlNodes      104 B each
//!   2. NavCurves         132 B each (v16+)
//!   3. Signs              52 B each
//!   4. Semaphores         84 B (v19) / 68 B (v15-18)
//!   5. SpawnPoints        36 B (v18+) / 32 B (v15-17)
//!   6. TerrainPoint positions  12 B each
//!   7. TerrainPoint normals    12 B each
//!   8. TerrainPoint variants    8 B each
//!   9. MapPoints          48 B each
//!  10. TriggerPoints      48 B each
//!  11. Intersections      16 B each
//!  12. NavNodes          188 B each (v16+)
//!
//! ## Version history
//! | Version | Game           | Change                                    |
//! |---------|----------------|-------------------------------------------|
//! | 0x15    | Pre-ETS2 1.30  | No NavNodes, no MapPoints                 |
//! | 0x16    | ETS2 1.30      | NavNodes added; NavCurve.nav_node_index   |
//! | 0x17    | ETS2 1.3x      | MapPoints restored                        |
//! | 0x18    | ETS2 1.4x-1.5x | SpawnPoint.flags added                    |
//! | 0x19    | ETS2 1.51+     | Semaphore.unknown2[4] added               |

use std::io::{Cursor, Read};

use serde::{Deserialize, Serialize};

use crate::error::ParseError;

// ---------------------------------------------------------------------------
// Read helpers
// ---------------------------------------------------------------------------

fn read_u8(cur: &mut Cursor<&[u8]>) -> Result<u8, ParseError> {
    let mut buf = [0u8; 1];
    cur.read_exact(&mut buf)
        .map_err(|e| ParseError::Binary(format!("read_u8: {e}")))?;
    Ok(buf[0])
}

fn read_u16(cur: &mut Cursor<&[u8]>) -> Result<u16, ParseError> {
    let mut buf = [0u8; 2];
    cur.read_exact(&mut buf)
        .map_err(|e| ParseError::Binary(format!("read_u16: {e}")))?;
    Ok(u16::from_le_bytes(buf))
}

fn read_u32(cur: &mut Cursor<&[u8]>) -> Result<u32, ParseError> {
    let mut buf = [0u8; 4];
    cur.read_exact(&mut buf)
        .map_err(|e| ParseError::Binary(format!("read_u32: {e}")))?;
    Ok(u32::from_le_bytes(buf))
}

fn read_u64(cur: &mut Cursor<&[u8]>) -> Result<u64, ParseError> {
    let mut buf = [0u8; 8];
    cur.read_exact(&mut buf)
        .map_err(|e| ParseError::Binary(format!("read_u64: {e}")))?;
    Ok(u64::from_le_bytes(buf))
}

fn read_i32(cur: &mut Cursor<&[u8]>) -> Result<i32, ParseError> {
    let mut buf = [0u8; 4];
    cur.read_exact(&mut buf)
        .map_err(|e| ParseError::Binary(format!("read_i32: {e}")))?;
    Ok(i32::from_le_bytes(buf))
}

fn read_f32(cur: &mut Cursor<&[u8]>) -> Result<f32, ParseError> {
    let mut buf = [0u8; 4];
    cur.read_exact(&mut buf)
        .map_err(|e| ParseError::Binary(format!("read_f32: {e}")))?;
    let v = f32::from_le_bytes(buf);
    // serde_json serialises NaN/Inf as null; reject them at parse time.
    Ok(if v.is_finite() { v } else { 0.0 })
}

fn read_f32x3(cur: &mut Cursor<&[u8]>) -> Result<[f32; 3], ParseError> {
    Ok([read_f32(cur)?, read_f32(cur)?, read_f32(cur)?])
}

fn read_f32x4(cur: &mut Cursor<&[u8]>) -> Result<[f32; 4], ParseError> {
    Ok([read_f32(cur)?, read_f32(cur)?, read_f32(cur)?, read_f32(cur)?])
}

fn read_i32x4(cur: &mut Cursor<&[u8]>) -> Result<[i32; 4], ParseError> {
    Ok([
        read_i32(cur)?,
        read_i32(cur)?,
        read_i32(cur)?,
        read_i32(cur)?,
    ])
}

fn read_i32x8(cur: &mut Cursor<&[u8]>) -> Result<[i32; 8], ParseError> {
    Ok([
        read_i32(cur)?,
        read_i32(cur)?,
        read_i32(cur)?,
        read_i32(cur)?,
        read_i32(cur)?,
        read_i32(cur)?,
        read_i32(cur)?,
        read_i32(cur)?,
    ])
}

#[allow(dead_code)]
fn skip(cur: &mut Cursor<&[u8]>, n: u64) -> Result<(), ParseError> {
    let pos = cur.position();
    let new_pos = pos + n;
    if new_pos > cur.get_ref().len() as u64 {
        return Err(ParseError::Binary(format!(
            "skip({n}): cursor at {pos}, file size {}",
            cur.get_ref().len()
        )));
    }
    cur.set_position(new_pos);
    Ok(())
}

fn seek_to(cur: &mut Cursor<&[u8]>, offset: u64) -> Result<(), ParseError> {
    if offset > cur.get_ref().len() as u64 {
        return Err(ParseError::Binary(format!(
            "seek_to({offset}): file size {}",
            cur.get_ref().len()
        )));
    }
    cur.set_position(offset);
    Ok(())
}

// ---------------------------------------------------------------------------
// PPD Header
// ---------------------------------------------------------------------------

pub const PPD_MAGIC: u32 = 0x53505044u32; // "SPPD" in LE

pub const PPD_HEADER_SIZE: usize = 96;

/// Version range known to this parser.
pub const PPD_VERSION_MIN: u32 = 0x15;
pub const PPD_VERSION_MAX: u32 = 0x19;

/// V15 maps ControlNode at 0x30 (only 11 sections), not 0x60.
pub const PPD_HEADER_V15_SIZE: usize = 92;

#[derive(Debug, Clone)]
pub struct PpdCounts {
    pub node_count: u32,
    pub nav_curve_count: u32,
    pub sign_count: u32,
    pub semaphore_count: u32,
    pub spawn_point_count: u32,
    pub terrain_point_count: u32,
    pub terrain_point_variant_count: u32,
    pub map_point_count: u32,
    pub trigger_point_count: u32,
    pub intersection_count: u32,
    pub nav_node_count: u32,
}

#[derive(Debug, Clone)]
pub struct PpdHeader {
    pub version: u32,
    pub counts: PpdCounts,
    pub section_offsets: [u32; 12],
}

fn read_ppd_header(data: &[u8]) -> Result<PpdHeader, ParseError> {
    let len = data.len();
    let mut cur = Cursor::new(data);

    // Try to detect if there's a magic prefix.
    // Newer PPD files may start with "SPPD" magic, older ones start with version.
    let first = read_u32(&mut cur)?;
    let version = if first == PPD_MAGIC {
        read_u32(&mut cur)?
    } else {
        first
    };

    if !(PPD_VERSION_MIN..=PPD_VERSION_MAX).contains(&version) {
        return Err(ParseError::Binary(format!(
            "unsupported PPD version 0x{version:02X} ({version}); expected 0x{PPD_VERSION_MIN:02X}..0x{PPD_VERSION_MAX:02X}"
        )));
    }

    let counts = PpdCounts {
        node_count: read_u32(&mut cur)?,
        nav_curve_count: read_u32(&mut cur)?,
        sign_count: read_u32(&mut cur)?,
        semaphore_count: read_u32(&mut cur)?,
        spawn_point_count: read_u32(&mut cur)?,
        terrain_point_count: read_u32(&mut cur)?,
        terrain_point_variant_count: read_u32(&mut cur)?,
        map_point_count: read_u32(&mut cur)?,
        trigger_point_count: read_u32(&mut cur)?,
        intersection_count: read_u32(&mut cur)?,
        nav_node_count: read_u32(&mut cur)?,
    };

    let offsets_start = if first == PPD_MAGIC { 0x34 } else { 0x30 };
    let _pos = cur.position();

    let mut section_offsets = [0u32; 12];
    let off_start = if first == PPD_MAGIC { 52 } else { 48 };
    let off_data = &data[off_start..];
    if off_data.len() < 48 {
        return Err(ParseError::Binary(format!(
            "PPD too short for offsets: {} bytes total", len
        )));
    }
    for (i, slot) in section_offsets.iter_mut().enumerate() {
        let base = i * 4;
        *slot = u32::from_le_bytes(off_data[base..base + 4].try_into().unwrap());
    }

    let _ = offsets_start; // Keep for reference

    Ok(PpdHeader {
        version,
        counts,
        section_offsets,
    })
}

// ---------------------------------------------------------------------------
// Section structs
// ---------------------------------------------------------------------------

/// ControlNode — the physical connection point where external roads snap in.
/// Max 6 per prefab. 104 bytes.
#[derive(Debug, Clone, Serialize, Deserialize, bincode::Encode, bincode::Decode)]
pub struct ControlNode {
    pub terrain_point_index: u32,
    pub terrain_point_count: u32,
    pub terrain_point_variant_index: u32,
    pub terrain_point_variant_count: u32,
    pub position: [f32; 3],
    pub direction: [f32; 3],
    pub input_lines: [i32; 8],
    pub output_lines: [i32; 8],
}

fn read_control_node(cur: &mut Cursor<&[u8]>) -> Result<ControlNode, ParseError> {
    Ok(ControlNode {
        terrain_point_index: read_u32(cur)?,
        terrain_point_count: read_u32(cur)?,
        terrain_point_variant_index: read_u32(cur)?,
        terrain_point_variant_count: read_u32(cur)?,
        position: read_f32x3(cur)?,
        direction: read_f32x3(cur)?,
        input_lines: read_i32x8(cur)?,
        output_lines: read_i32x8(cur)?,
    })
}

/// LeadsTo — packed 4-byte lane/node indices inside a NavCurve.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, bincode::Encode, bincode::Decode)]
pub struct LeadsTo {
    pub end_node: u8,
    pub end_lane: u8,
    pub start_node: u8,
    pub start_lane: u8,
}

/// NavCurve flags bitmask.
/// Bit 2-3: blinker (0=none, 1=NoBlinkerForced, 2=Right, 4=Left)
/// Bit 5-6: allowed_vehicles (0=PlayerOnly, 1=Small, 2=Large, 3=All)
/// Bit 13: low_probability
/// Bit 14: limit_displacement
/// Bit 15: additive_priority
/// Bit 16-19: priority_modifier
pub const NAVCURVE_FLAG_BLINKER_MASK: u32 = 0x0000_000C;
pub const NAVCURVE_FLAG_BLINKER_SHIFT: u32 = 2;
pub const NAVCURVE_BLINKER_NONE: u32 = 0;
pub const NAVCURVE_BLINKER_NO_FORCED: u32 = 1;
pub const NAVCURVE_BLINKER_RIGHT: u32 = 2;
pub const NAVCURVE_BLINKER_LEFT: u32 = 3;

/// NavCurve — directed Hermite spline segment. 132 bytes (v16+).
#[derive(Debug, Clone, Serialize, Deserialize, bincode::Encode, bincode::Decode)]
pub struct NavCurve {
    pub name: u64,
    pub flags: u32,
    pub leads_to: LeadsTo,
    pub start_position: [f32; 3],
    pub end_position: [f32; 3],
    pub start_rotation: [f32; 4],
    pub end_rotation: [f32; 4],
    pub length: f32,
    pub next_lines: [i32; 4],
    pub prev_lines: [i32; 4],
    pub next_used: u32,
    pub prev_used: u32,
    pub semaphore_id: i32,
    pub traffic_rule: u64,
    pub nav_node_index: u32,
}

fn read_nav_curve(cur: &mut Cursor<&[u8]>) -> Result<NavCurve, ParseError> {
    let name = read_u64(cur)?;
    let flags = read_u32(cur)?;
    let end_node = read_u8(cur)?;
    let end_lane = read_u8(cur)?;
    let start_node = read_u8(cur)?;
    let start_lane = read_u8(cur)?;
    let start_position = read_f32x3(cur)?;
    let end_position = read_f32x3(cur)?;
    let start_rotation = read_f32x4(cur)?;
    let end_rotation = read_f32x4(cur)?;
    let length = { let v = read_f32(cur)?; if v.is_finite() { v.max(0.0) } else { 0.0 } };
    let next_lines = read_i32x4(cur)?;
    let prev_lines = read_i32x4(cur)?;
    let next_used = read_u32(cur)?;
    let prev_used = read_u32(cur)?;
    let semaphore_id = read_i32(cur)?;
    let traffic_rule = read_u64(cur)?;
    let nav_node_index = read_u32(cur)?;

    Ok(NavCurve {
        name,
        flags,
        leads_to: LeadsTo {
            end_node,
            end_lane,
            start_node,
            start_lane,
        },
        start_position,
        end_position,
        start_rotation,
        end_rotation,
        length,
        next_lines,
        prev_lines,
        next_used,
        prev_used,
        semaphore_id,
        traffic_rule,
        nav_node_index,
    })
}

/// NavNode — coarser graph layer used by in-game GPS. 188 bytes (v16+).
#[derive(Debug, Clone, Serialize, Deserialize, bincode::Encode, bincode::Decode)]
pub struct NavNode {
    pub node_type: u8,
    pub index: u16,
    pub connections: Vec<NavNodeConnection>,
}

/// NavNodeConnectionInfo — 23 bytes per connection, max 8.
#[derive(Debug, Clone, Serialize, Deserialize, bincode::Encode, bincode::Decode)]
pub struct NavNodeConnection {
    pub target_node_index: u16,
    pub length: f32,
    pub curve_indices: Vec<u16>,
}

fn read_nav_node(cur: &mut Cursor<&[u8]>) -> Result<NavNode, ParseError> {
    let node_type = read_u8(cur)?;
    let index = read_u16(cur)?;
    let connection_count = read_u8(cur)?;
    let actual_count = connection_count.min(8) as usize;

    let mut connections = Vec::with_capacity(actual_count);
    for _ in 0..actual_count {
        let target_node_index = read_u16(cur)?;
        let length = { let v = read_f32(cur)?; if v.is_finite() { v.max(0.0) } else { 0.0 } };
        let curve_count = read_u8(cur)? as usize;
        let actual_cc = curve_count.min(8);
        let mut curve_indices = Vec::with_capacity(actual_cc);
        for _ in 0..8 {
            let idx = read_u16(cur)?;
            if curve_indices.len() < actual_cc && idx != 0xFFFF {
                curve_indices.push(idx);
            }
        }
        connections.push(NavNodeConnection {
            target_node_index,
            length,
            curve_indices,
        });
    }

    Ok(NavNode {
        node_type,
        index,
        connections,
    })
}

/// Semaphore — traffic light / barrier. 84 B (v19) / 68 B (v15-18).
#[derive(Debug, Clone, Serialize, Deserialize, bincode::Encode, bincode::Decode)]
pub struct Semaphore {
    pub position: [f32; 3],
    pub rotation: [f32; 4],
    pub sem_type: u32,
    pub semaphore_id: u32,
    pub intervals: [f32; 4],
    pub cycle_delay: f32,
    pub profile: u64,
    pub unknown1: u32,
    pub unknown2: [u32; 4],
}

fn read_semaphore(cur: &mut Cursor<&[u8]>, version: u32) -> Result<Semaphore, ParseError> {
    let sem = Semaphore {
        position: read_f32x3(cur)?,
        rotation: read_f32x4(cur)?,
        sem_type: read_u32(cur)?,
        semaphore_id: read_u32(cur)?,
        intervals: read_f32x4(cur)?,
        cycle_delay: read_f32(cur)?,
        profile: read_u64(cur)?,
        unknown1: read_u32(cur)?,
        unknown2: if version >= 0x19 {
            let mut a = [0u32; 4];
            for slot in &mut a {
                *slot = read_u32(cur)?;
            }
            a
        } else {
            [0u32; 4]
        },
    };
    Ok(sem)
}

/// SpawnPoint — AI vehicle spawn. 36 B (v18+) / 32 B (v15-17).
#[derive(Debug, Clone, Serialize, Deserialize, bincode::Encode, bincode::Decode)]
pub struct SpawnPoint {
    pub position: [f32; 3],
    pub rotation: [f32; 4],
    pub spawn_type: u32,
    pub flags: u32,
}

fn read_spawn_point(cur: &mut Cursor<&[u8]>, version: u32) -> Result<SpawnPoint, ParseError> {
    Ok(SpawnPoint {
        position: read_f32x3(cur)?,
        rotation: read_f32x4(cur)?,
        spawn_type: read_u32(cur)?,
        flags: if version >= 0x18 {
            read_u32(cur)?
        } else {
            0
        },
    })
}

/// Sign — prefab-embedded traffic sign. 52 bytes.
#[derive(Debug, Clone, Serialize, Deserialize, bincode::Encode, bincode::Decode)]
pub struct PpdSign {
    pub position: [f32; 3],
    pub rotation: [f32; 4],
    pub model_token: u64,
    pub part_token: u64,
}

fn read_ppd_sign(cur: &mut Cursor<&[u8]>) -> Result<PpdSign, ParseError> {
    Ok(PpdSign {
        position: read_f32x3(cur)?,
        rotation: read_f32x4(cur)?,
        model_token: read_u64(cur)?,
        part_token: read_u64(cur)?,
    })
}

/// MapPoint — mini-map drawing node. 48 bytes.
#[derive(Debug, Clone, Serialize, Deserialize, bincode::Encode, bincode::Decode)]
pub struct MapPoint {
    pub vis_flags: u32,
    pub nav_flags: u32,
    pub position: [f32; 3],
    pub neighbors: [i32; 6],
    pub neighbors_used: u32,
}

fn read_map_point(cur: &mut Cursor<&[u8]>) -> Result<MapPoint, ParseError> {
    Ok(MapPoint {
        vis_flags: read_u32(cur)?,
        nav_flags: read_u32(cur)?,
        position: read_f32x3(cur)?,
        neighbors: [
            read_i32(cur)?,
            read_i32(cur)?,
            read_i32(cur)?,
            read_i32(cur)?,
            read_i32(cur)?,
            read_i32(cur)?,
        ],
        neighbors_used: read_u32(cur)?,
    })
}

/// TriggerPoint — action trigger. 48 bytes.
#[derive(Debug, Clone, Serialize, Deserialize, bincode::Encode, bincode::Decode)]
pub struct TriggerPoint {
    pub action_token: u64,
    pub range: f32,
    pub position: [f32; 3],
    pub neighbors: [i32; 6],
    pub neighbors_used: u32,
}

fn read_trigger_point(cur: &mut Cursor<&[u8]>) -> Result<TriggerPoint, ParseError> {
    Ok(TriggerPoint {
        action_token: read_u64(cur)?,
        range: read_f32(cur)?,
        position: read_f32x3(cur)?,
        neighbors: [
            read_i32(cur)?,
            read_i32(cur)?,
            read_i32(cur)?,
            read_i32(cur)?,
            read_i32(cur)?,
            read_i32(cur)?,
        ],
        neighbors_used: read_u32(cur)?,
    })
}

/// Intersection — AI priority crossing pair. 16 bytes.
#[derive(Debug, Clone, Serialize, Deserialize, bincode::Encode, bincode::Decode)]
pub struct Intersection {
    pub curve_id: u32,
    pub param_t: f32,
    pub radius: f32,
    pub flags: u32,
}

fn read_intersection(cur: &mut Cursor<&[u8]>) -> Result<Intersection, ParseError> {
    Ok(Intersection {
        curve_id: read_u32(cur)?,
        param_t: read_f32(cur)?,
        radius: read_f32(cur)?,
        flags: read_u32(cur)?,
    })
}

// ---------------------------------------------------------------------------
// PrefabDescriptor — the parsed PPD payload
// ---------------------------------------------------------------------------

/// A fully-deserialized PPD file, ready for world-space instantiation.
#[derive(Debug, Clone, Serialize, Deserialize, bincode::Encode, bincode::Decode)]
pub struct PrefabDescriptor {
    pub control_nodes: Vec<ControlNode>,
    pub nav_curves: Vec<NavCurve>,
    pub nav_nodes: Vec<NavNode>,
    pub semaphores: Vec<Semaphore>,
    pub spawn_points: Vec<SpawnPoint>,
    pub signs: Vec<PpdSign>,
    pub map_points: Vec<MapPoint>,
    pub trigger_points: Vec<TriggerPoint>,
    pub intersections: Vec<Intersection>,
    /// Version field from the PPD header.
    pub ppd_version: u32,
}

// ---------------------------------------------------------------------------
// Main parse entry-point
// ---------------------------------------------------------------------------

/// Parse a PPD file from raw bytes.
///
/// Handles versions 0x15 through 0x19, with appropriate field-size
/// adjustments for each version (e.g. v15 has no NavNodes, v19 has
/// larger Semaphores).
pub fn parse_ppd(data: &[u8]) -> Result<PrefabDescriptor, ParseError> {
    let header = read_ppd_header(data)?;
    let version = header.version;

    let mut cur = Cursor::new(data);

    let offsets = &header.section_offsets;
    let counts = &header.counts;

    // 1. ControlNodes — always present
    let control_nodes = read_section_array(
        &mut cur,
        offsets[0],
        counts.node_count,
        104,
        read_control_node,
    )?;

    // 2. NavCurves — 132 B each (v16+)
    let nav_curves = read_section_array(
        &mut cur,
        offsets[1],
        counts.nav_curve_count,
        132,
        read_nav_curve,
    )?;

    // 3. Signs
    let signs = read_section_array(
        &mut cur,
        offsets[2],
        counts.sign_count,
        52,
        read_ppd_sign,
    )?;

    // 4. Semaphores — 84 B (v19) / 68 B (v15-18)
    let sem_size: u32 = if version >= 0x19 { 84 } else { 68 };
    let semaphores = read_section_with(
        &mut cur,
        offsets[3],
        counts.semaphore_count,
        sem_size,
        |c| read_semaphore(c, version),
    )?;

    // 5. SpawnPoints — 36 B (v18+) / 32 B (v15-17)
    let sp_size: u32 = if version >= 0x18 { 36 } else { 32 };
    let spawn_points = read_section_with(
        &mut cur,
        offsets[4],
        counts.spawn_point_count,
        sp_size,
        |c| read_spawn_point(c, version),
    )?;

    // 6. TerrainPoint positions (12 B each) — parsed but not stored
    // 7. TerrainPoint normals (12 B each)
    // 8. TerrainPoint variants (8 B each)
    // We skip these for now; future phases may need them.

    // 9. MapPoints — 48 B each
    let map_points = read_section_array(
        &mut cur,
        offsets[8],
        counts.map_point_count,
        48,
        read_map_point,
    )?;

    // 10. TriggerPoints — 48 B each
    let trigger_points = read_section_array(
        &mut cur,
        offsets[9],
        counts.trigger_point_count,
        48,
        read_trigger_point,
    )?;

    // 11. Intersections — 16 B each
    let intersections = read_section_array(
        &mut cur,
        offsets[10],
        counts.intersection_count,
        16,
        read_intersection,
    )?;

    // 12. NavNodes — 188 B each (v16+). For v15, offset[11] is 0/unused.
    let nav_node_size: u32 = 188;
    let nav_nodes: Vec<NavNode> = if version >= 0x16 && offsets[11] != 0 {
        read_section_with(
            &mut cur,
            offsets[11],
            counts.nav_node_count,
            nav_node_size,
            read_nav_node,
        )?
    } else {
        Vec::new()
    };

    Ok(PrefabDescriptor {
        control_nodes,
        nav_curves,
        nav_nodes,
        semaphores,
        spawn_points,
        signs,
        map_points,
        trigger_points,
        intersections,
        ppd_version: version,
    })
}

// ---------------------------------------------------------------------------
// Section reader helpers
// ---------------------------------------------------------------------------

/// Read a fixed-size section as an array of items.
fn read_section_array<T, F>(
    cur: &mut Cursor<&[u8]>,
    offset: u32,
    count: u32,
    item_size: u32,
    reader: F,
) -> Result<Vec<T>, ParseError>
where
    F: Fn(&mut Cursor<&[u8]>) -> Result<T, ParseError>,
{
    read_section_with(cur, offset, count, item_size, reader)
}

/// Read a fixed-size section.
fn read_section_with<T, F>(
    cur: &mut Cursor<&[u8]>,
    offset: u32,
    count: u32,
    item_size: u32,
    reader: F,
) -> Result<Vec<T>, ParseError>
where
    F: Fn(&mut Cursor<&[u8]>) -> Result<T, ParseError>,
{
    if count == 0 {
        return Ok(Vec::new());
    }

    let start = offset as u64;
    let total_bytes = count as u64 * item_size as u64;
    let file_size = cur.get_ref().len() as u64;

    if start + total_bytes > file_size {
        return Err(ParseError::Binary(format!(
            "section at 0x{offset:08X}: {count} items × {item_size} B = {total_bytes} B exceeds file size {file_size}"
        )));
    }

    seek_to(cur, start)?;

    let mut items = Vec::with_capacity(count as usize);
    for _ in 0..count {
        items.push(reader(cur)?);
    }

    Ok(items)
}

// ---------------------------------------------------------------------------
// NavCurve helpers
// ---------------------------------------------------------------------------

impl NavCurve {
    /// Blinker (turn signal) extracted from flags bits [3:2].
    pub fn blinker(&self) -> u32 {
        (self.flags & NAVCURVE_FLAG_BLINKER_MASK) >> NAVCURVE_FLAG_BLINKER_SHIFT
    }

    /// True if this curve has an associated NavNode (v16+).
    pub fn has_nav_node(&self) -> bool {
        self.nav_node_index != 0xFFFF_FFFF
    }
}

// ---------------------------------------------------------------------------
// Blinker enum
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq, bincode::Encode, bincode::Decode)]
pub enum Blinker {
    None,
    Left,
    Right,
}

impl Blinker {
    pub fn from_nav_curve_flags(flags: u32) -> Self {
        let val = (flags & NAVCURVE_FLAG_BLINKER_MASK) >> NAVCURVE_FLAG_BLINKER_SHIFT;
        match val {
            NAVCURVE_BLINKER_LEFT => Blinker::Left,
            NAVCURVE_BLINKER_RIGHT => Blinker::Right,
            _ => Blinker::None,
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a minimal valid PPD v19 binary for testing.
    fn make_v19_ppd(node_count: u32, curve_count: u32, nav_node_count: u32) -> Vec<u8> {
        let version: u32 = 0x19;
        let sem_count: u32 = 0;
        let sign_count: u32 = 0;
        let sp_count: u32 = 0;
        let tp_count: u32 = 0;
        let tpv_count: u32 = 0;
        let mp_count: u32 = 0;
        let trig_count: u32 = 0;
        let int_count: u32 = 0;

        let node_size: u32 = 104;
        let curve_size: u32 = 132;
        let sem_size: u32 = 84;
        let sp_size: u32 = 36;
        let tp_pos_size: u32 = 12;
        let tp_nor_size: u32 = 12;
        let tpv_size: u32 = 8;
        let mp_size: u32 = 48;
        let trig_size: u32 = 48;
        let int_size: u32 = 16;
        let nav_node_size: u32 = 188;

        let mut off: u32 = 96;
        let off_nodes = off;
        off += node_count * node_size;
        let off_curves = off;
        off += curve_count * curve_size;
        let off_signs = off;
        off += sign_count * 52;
        let off_sems = off;
        off += sem_count * sem_size;
        let off_sp = off;
        off += sp_count * sp_size;
        let off_tp_pos = off;
        off += tp_count * tp_pos_size;
        let off_tp_nor = off;
        off += tp_count * tp_nor_size;
        let off_tp_var = off;
        off += tpv_count * tpv_size;
        let off_mp = off;
        off += mp_count * mp_size;
        let off_trig = off;
        off += trig_count * trig_size;
        let off_int = off;
        off += int_count * int_size;
        let off_nav = off;

        let mut buf = Vec::with_capacity((off + nav_node_count * nav_node_size) as usize);

        // Header
        buf.extend_from_slice(&version.to_le_bytes());
        buf.extend_from_slice(&node_count.to_le_bytes());
        buf.extend_from_slice(&curve_count.to_le_bytes());
        buf.extend_from_slice(&sign_count.to_le_bytes());
        buf.extend_from_slice(&sem_count.to_le_bytes());
        buf.extend_from_slice(&sp_count.to_le_bytes());
        buf.extend_from_slice(&tp_count.to_le_bytes());
        buf.extend_from_slice(&tpv_count.to_le_bytes());
        buf.extend_from_slice(&mp_count.to_le_bytes());
        buf.extend_from_slice(&trig_count.to_le_bytes());
        buf.extend_from_slice(&int_count.to_le_bytes());
        buf.extend_from_slice(&nav_node_count.to_le_bytes());

        // 12 offsets
        let offsets: [u32; 12] = [
            off_nodes, off_curves, off_signs, off_sems, off_sp, off_tp_pos, off_tp_nor,
            off_tp_var, off_mp, off_trig, off_int, off_nav,
        ];
        for o in &offsets {
            buf.extend_from_slice(&o.to_le_bytes());
        }

        // Zero-pad to off_nodes
        while buf.len() < off_nodes as usize {
            buf.push(0);
        }

        // ControlNodes
        for i in 0..node_count {
            let mut cn = Vec::new();
            cn.extend_from_slice(&0u32.to_le_bytes()); // terrain_point_index
            cn.extend_from_slice(&0u32.to_le_bytes()); // terrain_point_count
            cn.extend_from_slice(&0u32.to_le_bytes()); // variant_index
            cn.extend_from_slice(&0u32.to_le_bytes()); // variant_count
            cn.extend_from_slice(&((i as f32) * 10.0).to_le_bytes()); // x
            cn.extend_from_slice(&0.0f32.to_le_bytes()); // y
            cn.extend_from_slice(&0.0f32.to_le_bytes()); // z
            cn.extend_from_slice(&1.0f32.to_le_bytes()); // dir_x
            cn.extend_from_slice(&0.0f32.to_le_bytes()); // dir_y
            cn.extend_from_slice(&0.0f32.to_le_bytes()); // dir_z
            for _ in 0..8 {
                cn.extend_from_slice(&(-1i32).to_le_bytes()); // input_lines
            }
            for _ in 0..8 {
                cn.extend_from_slice(&(-1i32).to_le_bytes()); // output_lines
            }
            buf.extend_from_slice(&cn);
        }

        // NavCurves
        for i in 0..curve_count {
            let mut nc = Vec::new();
            nc.extend_from_slice(&0u64.to_le_bytes()); // name
            nc.extend_from_slice(&0u32.to_le_bytes()); // flags
            nc.push(0); // end_node
            nc.push(0); // end_lane
            nc.push(0); // start_node
            nc.push(0); // start_lane
            let pos = (i as f32) * 10.0;
            nc.extend_from_slice(&pos.to_le_bytes()); // start_x
            nc.extend_from_slice(&0.0f32.to_le_bytes());
            nc.extend_from_slice(&0.0f32.to_le_bytes());
            nc.extend_from_slice(&(pos + 10.0).to_le_bytes()); // end_x
            nc.extend_from_slice(&0.0f32.to_le_bytes());
            nc.extend_from_slice(&0.0f32.to_le_bytes());
            for _ in 0..4 {
                nc.extend_from_slice(&0.0f32.to_le_bytes()); // start_rot
            }
            nc.extend_from_slice(&0.0f32.to_le_bytes()); // end_rot w
            nc.extend_from_slice(&0.0f32.to_le_bytes()); // end_rot x
            nc.extend_from_slice(&0.0f32.to_le_bytes()); // end_rot y
            nc.extend_from_slice(&1.0f32.to_le_bytes()); // end_rot z
            nc.extend_from_slice(&10.0f32.to_le_bytes()); // length
            for _ in 0..4 {
                nc.extend_from_slice(&(-1i32).to_le_bytes()); // next_lines
            }
            for _ in 0..4 {
                nc.extend_from_slice(&(-1i32).to_le_bytes()); // prev_lines
            }
            nc.extend_from_slice(&0u32.to_le_bytes()); // next_used
            nc.extend_from_slice(&0u32.to_le_bytes()); // prev_used
            nc.extend_from_slice(&(-1i32).to_le_bytes()); // semaphore_id
            nc.extend_from_slice(&0u64.to_le_bytes()); // traffic_rule
            nc.extend_from_slice(&0xFFFF_FFFFu32.to_le_bytes()); // nav_node_index
            buf.extend_from_slice(&nc);
        }

        // Zero-pad remaining to end of sections
        let target = off_nav as usize + nav_node_count as usize * nav_node_size as usize;
        while buf.len() < target {
            buf.push(0);
        }

        buf
    }

    #[test]
    fn test_ppd_header_v19_empty() {
        let data = make_v19_ppd(0, 0, 0);
        let header = read_ppd_header(&data).unwrap();
        assert_eq!(header.version, 0x19);
        assert_eq!(header.counts.node_count, 0);
        assert_eq!(header.counts.nav_curve_count, 0);
        assert_eq!(header.counts.nav_node_count, 0);
    }

    #[test]
    fn test_ppd_header_v19_three_nodes() {
        let data = make_v19_ppd(3, 20, 6);
        let header = read_ppd_header(&data).unwrap();
        assert_eq!(header.version, 0x19);
        assert_eq!(header.counts.node_count, 3);
        assert_eq!(header.counts.nav_curve_count, 20);
        assert_eq!(header.counts.nav_node_count, 6);
        assert_eq!(header.section_offsets[0], 96); // nodes at 0x60
    }

    #[test]
    fn test_parse_ppd_v19_simple() {
        let data = make_v19_ppd(3, 20, 6);
        let ppd = parse_ppd(&data).unwrap();
        assert_eq!(ppd.control_nodes.len(), 3);
        assert_eq!(ppd.nav_curves.len(), 20);
        assert_eq!(ppd.nav_nodes.len(), 6);
        assert_eq!(ppd.ppd_version, 0x19);

        // Check first node position
        assert!((ppd.control_nodes[0].position[0] - 0.0).abs() < 0.01);
        assert!((ppd.control_nodes[1].position[0] - 10.0).abs() < 0.01);

        // Check first curve
        assert!((ppd.nav_curves[0].length - 10.0).abs() < 0.01);
        assert_eq!(ppd.nav_curves[0].nav_node_index, 0xFFFF_FFFF);
        assert!(ppd.nav_curves[0].next_lines.iter().all(|&v| v == -1));
        assert!(ppd.nav_curves[0].prev_lines.iter().all(|&v| v == -1));
    }

    #[test]
    fn test_blinker_from_flags() {
        assert_eq!(Blinker::from_nav_curve_flags(0x0000_0000), Blinker::None);
        assert_eq!(
            Blinker::from_nav_curve_flags(0x0000_0008),
            Blinker::Right
        );
        assert_eq!(
            Blinker::from_nav_curve_flags(0x0000_000C),
            Blinker::Left
        );
    }

    #[test]
    fn test_nav_curve_has_nav_node() {
        let mut nc = NavCurve {
            name: 0,
            flags: 0,
            leads_to: LeadsTo {
                end_node: 0,
                end_lane: 0,
                start_node: 0,
                start_lane: 0,
            },
            start_position: [0.0; 3],
            end_position: [0.0; 3],
            start_rotation: [0.0; 4],
            end_rotation: [0.0; 4],
            length: 0.0,
            next_lines: [-1; 4],
            prev_lines: [-1; 4],
            next_used: 0,
            prev_used: 0,
            semaphore_id: -1,
            traffic_rule: 0,
            nav_node_index: 0xFFFF_FFFF,
        };
        assert!(!nc.has_nav_node());
        nc.nav_node_index = 5;
        assert!(nc.has_nav_node());
    }

    #[test]
    fn test_reject_unsupported_version() {
        let mut data = make_v19_ppd(0, 0, 0);
        data[0] = 0x10; // Set version to unsupported
        let result = parse_ppd(&data);
        assert!(result.is_err());
    }
}
