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

use binrw::BinRead;
use tracing::{debug, instrument, warn};

use crate::drop_tracer::{DropCategory, DropEvent, DropTracer};
use crate::error::ParseError;
use crate::road_full::RoadFixedHeader;

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
    /// Rotation quaternion `[qw, qx, qy, qz]` from the 56-byte legacy node record.
    /// Set to `[0.0; 4]` for sized-format nodes (only 1×f32 is stored there).
    pub rotation: [f32; 4],
    /// UID of the item attached in the "forward" direction (0 = none / sized-format).
    pub forward_item_uid: u64,
    /// UID of the item attached in the "backward" direction (0 = none / sized-format).
    pub backward_item_uid: u64,
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
    pub look_token: u64,
    /// DLC-guard byte from the fixed header — 0 = no DLC required.
    pub dlc_guard: u8,
    /// `true` when the road is hidden from the in-game UI map.
    pub is_hidden: bool,
    /// `true` when the road is flagged "GPS-avoid" (routing should penalise it).
    pub gps_avoid: bool,
    /// Token identifying the road type (look/category).  0 for legacy/sized roads.
    pub road_type_token: u64,
}

/// A prefab (intersection / junction template).
#[derive(Debug, Clone)]
pub struct RawPrefab {
    pub uid: u64,
    pub template_token: u64,
    pub node_count: u8,
    pub nodes: Vec<u64>,
}

/// A ferry/train item (Type 19). Phase 5.22 — items sharing a `port_token`
/// belong to the same ferry route; the graph builder turns each port-token
/// group into a fully-connected clique of bidirectional `direction="ferry"`
/// edges, which is the only natural cross-sector connectivity in v907 maps.
#[derive(Debug, Clone)]
pub struct RawFerry {
    pub uid: u64,
    /// Hash/ID of the port unit name from `/def/ferry.sii`. Same token across
    /// all ferry items of the same route.
    pub port_token: u64,
    /// UID of the linked harbour prefab (0 = none).
    pub prefab_uid: u64,
    /// UID of the graph node at the ferry port location (0 = unset).
    pub node_uid: u64,
}

/// A buildings item (Type 2). Phase 5.25a — buildings carry a primary
/// `Node` and a `ForwardNode` delineating the strip along a road. The
/// graph builder turns each pair into a bidirectional
/// `direction="building"` edge, recovering connectivity through nodes
/// that road items reference but ignore (Phase 5.24 audit: ~9% of 574k
/// singletons are referenced by ignored item types in base_map).
#[derive(Debug, Clone)]
pub struct RawBuilding {
    pub uid: u64,
    pub node_uid: u64,
    pub forward_node_uid: u64,
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
    /// Ferry/train items (Type 19). Phase 5.22 cross-sector connectivity.
    pub ferries: Vec<RawFerry>,
    /// Buildings items (Type 2). Phase 5.25a — node + forward_node pairs
    /// turned into bidirectional `direction="building"` edges by the
    /// graph builder, recovering connectivity hidden in 9% of singletons.
    pub buildings: Vec<RawBuilding>,
    /// Diagnostic counter — how many of `nodes` were rebuilt by
    /// `recover_nodes_from_tail` after a partial-item failure (Phase 5.8
    /// recovery path). `0` when the standard parse-trailing-nodes path ran.
    /// Used by `truckpilot-uid-resolution` to assess UID-space mismatches.
    pub recovered_nodes_count: usize,
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
const ITEM_TYPE_MOVER: u32 = 9;
const ITEM_TYPE_NO_WEATHER: u32 = 11;
const ITEM_TYPE_HINGE: u32 = 13;
const ITEM_TYPE_CAMERA_POINT: u32 = 23;
const ITEM_TYPE_COMPOUND: u32 = 40;
const ITEM_TYPE_CAMERA_PATH: u32 = 45;
const ITEM_TYPE_HOOKUP: u32 = 47;
const ITEM_TYPE_GATE: u32 = 49;

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
    if let Some(sector) = try_parse_sized_sector(data, None) {
        debug!(
            roads = sector.roads.len(),
            nodes = sector.nodes.len(),
            "sized-format sector parsed"
        );
        return Ok(sector);
    }
    debug!("falling back to legacy sector parser");
    parse_sector_legacy(data)
}

fn parse_sector_legacy(data: &[u8]) -> Result<ParsedSector, ParseError> {
    parse_sector_legacy_inner(data, None)
}

/// Same as [`parse_sector`] but instruments every drop point with a
/// [`DropEvent`] recorded into `tracer`. Both the sized-format path
/// (`SizedRoadParseFailed`) and the legacy path are instrumented.
pub fn parse_sector_with_tracer(
    data: &[u8],
    sector_path: &str,
    tracer: &DropTracer,
) -> Result<ParsedSector, ParseError> {
    if let Some(sector) = try_parse_sized_sector(data, Some((tracer, sector_path))) {
        return Ok(sector);
    }
    parse_sector_legacy_inner(data, Some((tracer, sector_path)))
}

fn parse_sector_legacy_inner(
    data: &[u8],
    tracer_ctx: Option<(&DropTracer, &str)>,
) -> Result<ParsedSector, ParseError> {
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

    let mut all_items_parsed = true;
    for idx in 0..item_count as usize {
        // Phase 5.8: any read error here means the previous handler desynced
        // the cursor — accept whatever we already collected as a partial
        // sector instead of dropping every parsed Road/Prefab.
        let item_type = match read_u32(&mut cur) {
            Ok(t) => t,
            Err(e) => {
                warn!("sector item #{idx} type read failed: {e} — partial sector accepted");
                if let Some((tracer, sector_path)) = tracer_ctx {
                    tracer.record(DropEvent {
                        category: DropCategory::SectorHandlerError,
                        sector_path: sector_path.to_string(),
                        item_type: 0,
                        item_uid: None,
                        node_a: None,
                        node_b: None,
                        node_a_resolved: None,
                        node_b_resolved: None,
                        x: None,
                        z: None,
                        raw_hex: Vec::new(),
                    });
                }
                all_items_parsed = false;
                break;
            }
        };
        // Capture body-start offset AFTER reading item_type, so raw_hex covers
        // the item body bytes (not the 4-byte type prefix).
        let item_body_start = cur.position() as usize;
        let dispatch_result: Result<(), ParseError> = match item_type {
            ITEM_TYPE_ROAD => parse_road(&mut cur, &mut sector),
            ITEM_TYPE_PREFAB => parse_prefab(&mut cur, &mut sector),
            ITEM_TYPE_TERRAIN => skip_terrain(&mut cur),
            ITEM_TYPE_BUILDINGS => parse_buildings(&mut cur, &mut sector),
            ITEM_TYPE_MODEL => skip_model(&mut cur),
            ITEM_TYPE_COMPANY => skip_company(&mut cur),
            ITEM_TYPE_SERVICE => skip_service(&mut cur),
            ITEM_TYPE_CUT_PLANE => skip_cut_plane(&mut cur),
            ITEM_TYPE_CITY => skip_city(&mut cur),
            ITEM_TYPE_MAP_OVERLAY => skip_map_overlay(&mut cur),
            ITEM_TYPE_FERRY => parse_ferry(&mut cur, &mut sector),
            ITEM_TYPE_GARAGE => skip_garage(&mut cur),
            ITEM_TYPE_TRIGGER => skip_trigger(&mut cur),
            ITEM_TYPE_FUEL_PUMP => skip_fuel_pump(&mut cur),
            ITEM_TYPE_SIGN => skip_sign(&mut cur),
            ITEM_TYPE_BUS_STOP => skip_bus_stop(&mut cur),
            ITEM_TYPE_TRAFFIC_AREA => skip_traffic_area(&mut cur),
            ITEM_TYPE_BEZIER_PATCH => skip_bezier_patch(&mut cur),
            ITEM_TYPE_TRAJECTORY => skip_trajectory(&mut cur),
            ITEM_TYPE_MAP_AREA => skip_map_area(&mut cur),
            ITEM_TYPE_FAR_MODEL => skip_far_model(&mut cur),
            ITEM_TYPE_CURVE => skip_curve(&mut cur),
            ITEM_TYPE_CUTSCENE => skip_cutscene(&mut cur),
            ITEM_TYPE_VISIBILITY_AREA => skip_visibility_area(&mut cur),
            ITEM_TYPE_MOVER => skip_mover(&mut cur),
            ITEM_TYPE_NO_WEATHER => skip_no_weather(&mut cur),
            ITEM_TYPE_HINGE => skip_hinge(&mut cur),
            ITEM_TYPE_CAMERA_POINT => skip_camera_point(&mut cur),
            ITEM_TYPE_COMPOUND => parse_compound(&mut cur, &mut sector),
            ITEM_TYPE_CAMERA_PATH => skip_camera_path(&mut cur),
            ITEM_TYPE_HOOKUP => skip_hookup(&mut cur),
            ITEM_TYPE_GATE => skip_gate(&mut cur),
            other => Err(ParseError::Binary(format!("unsupported item type {other}"))),
        };
        if let Err(e) = dispatch_result {
            warn!("sector item #{idx} (type={item_type}) failed: {e} — partial sector accepted");
            if let Some((tracer, sector_path)) = tracer_ctx {
                let category = if item_type == ITEM_TYPE_ROAD {
                    DropCategory::RoadParseFailed
                } else if is_known_item_type(item_type) {
                    DropCategory::SectorHandlerError
                } else {
                    DropCategory::UnknownItemType
                };
                let body = &data[item_body_start..];
                let hex_cap = tracer.hex_limit.min(body.len());
                let raw_hex = body[..hex_cap].to_vec();
                let (item_uid, node_a, node_b) = if item_type == ITEM_TYPE_ROAD {
                    try_extract_road_uids(body)
                } else {
                    (None, None, None)
                };
                tracer.record(DropEvent {
                    category,
                    sector_path: sector_path.to_string(),
                    item_type,
                    item_uid,
                    node_a,
                    node_b,
                    node_a_resolved: None,
                    node_b_resolved: None,
                    x: None,
                    z: None,
                    raw_hex,
                });
            }
            all_items_parsed = false;
            break;
        }
    }

    // Trailing node section.
    if all_items_parsed {
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
    } else {
        // Phase 5.8 recovery: rebuild the node section from the sector tail.
        // Layout (TruckLib + ts-map): items… | node_count u32 | nodes(56·N)
        //                                    | vis_count u32  | vis_uids(8·M)
        // We sweep plausible (M, N) and accept the first arrangement whose
        // counts add up to the remaining bytes after the items section.
        recover_nodes_from_tail(data, &mut sector);
    }

    Ok(sector)
}

/// Returns `true` for every item_type that has a dispatch branch in
/// `parse_sector_legacy_inner`. Used to distinguish `SectorHandlerError`
/// (known type, handler failed) from `UnknownItemType` (no branch exists).
fn is_known_item_type(t: u32) -> bool {
    matches!(
        t,
        ITEM_TYPE_TERRAIN
            | ITEM_TYPE_BUILDINGS
            | ITEM_TYPE_ROAD
            | ITEM_TYPE_PREFAB
            | ITEM_TYPE_MODEL
            | ITEM_TYPE_COMPANY
            | ITEM_TYPE_SERVICE
            | ITEM_TYPE_CUT_PLANE
            | ITEM_TYPE_CITY
            | ITEM_TYPE_MAP_OVERLAY
            | ITEM_TYPE_FERRY
            | ITEM_TYPE_GARAGE
            | ITEM_TYPE_TRIGGER
            | ITEM_TYPE_FUEL_PUMP
            | ITEM_TYPE_SIGN
            | ITEM_TYPE_BUS_STOP
            | ITEM_TYPE_TRAFFIC_AREA
            | ITEM_TYPE_BEZIER_PATCH
            | ITEM_TYPE_TRAJECTORY
            | ITEM_TYPE_MAP_AREA
            | ITEM_TYPE_FAR_MODEL
            | ITEM_TYPE_CURVE
            | ITEM_TYPE_CUTSCENE
            | ITEM_TYPE_VISIBILITY_AREA
            | ITEM_TYPE_MOVER
            | ITEM_TYPE_NO_WEATHER
            | ITEM_TYPE_HINGE
            | ITEM_TYPE_CAMERA_POINT
            | ITEM_TYPE_COMPOUND
            | ITEM_TYPE_CAMERA_PATH
            | ITEM_TYPE_HOOKUP
            | ITEM_TYPE_GATE
    )
}

/// Try to extract uid / start_node_uid / end_node_uid from raw road body bytes.
/// Offsets are fixed by the `RoadFixedHeader` layout (265 B):
///   uid at 0, start_node_uid at 0xF5 (245), end_node_uid at 0xFD (253).
fn try_extract_road_uids(body: &[u8]) -> (Option<u64>, Option<u64>, Option<u64>) {
    let uid = read_u64_at(body, 0);
    let node_a = read_u64_at(body, 245);
    let node_b = read_u64_at(body, 253);
    (uid, node_a, node_b)
}

fn read_u64_at(data: &[u8], offset: usize) -> Option<u64> {
    let end = offset + 8;
    if end <= data.len() {
        Some(u64::from_le_bytes(data[offset..end].try_into().unwrap()))
    } else {
        None
    }
}

/// Tail-rebuild used after partial-item parsing — see `parse_sector_legacy`.
/// Tries to identify the trailing node block by matching the equation
/// `count_pos + 4 + N*56 + 4 + M*8 == data.len()` for plausible `N`/`M`.
fn recover_nodes_from_tail(data: &[u8], sector: &mut ParsedSector) {
    /// Per-record size for a 56-byte legacy node.
    const NODE_BYTES: usize = 56;
    const MAX_N: usize = 4096;
    const MAX_M: usize = 4096;

    let total = data.len();
    // The fixed sector header is 16 bytes; the smallest plausible tail starts
    // a few bytes into the file. Scan from the end for an `(M, N)` pair that
    // perfectly accounts for the remaining bytes — favour the largest plausible
    // anchor (longest tail) so we don't accept a tiny (0,0) layout when a
    // bigger one fits.
    for m in 0..=MAX_M {
        let vis_block = 4 + m * 8;
        if vis_block > total {
            break;
        }
        let vis_count_pos = total - vis_block;
        if vis_count_pos < 4 + 16 {
            continue;
        }
        let vis_count = u32::from_le_bytes([
            data[vis_count_pos],
            data[vis_count_pos + 1],
            data[vis_count_pos + 2],
            data[vis_count_pos + 3],
        ]);
        if vis_count as usize != m {
            continue;
        }

        let nodes_end = vis_count_pos;
        for n in 0..=MAX_N {
            let block = 4 + n * NODE_BYTES;
            if block > nodes_end {
                break;
            }
            let count_pos = nodes_end - block;
            if count_pos < 16 {
                break;
            }
            let count_at = u32::from_le_bytes([
                data[count_pos],
                data[count_pos + 1],
                data[count_pos + 2],
                data[count_pos + 3],
            ]);
            if count_at as usize != n {
                continue;
            }

            // Plausibility: every parsed UID should look like an ETS2 v907
            // node uid (high u16 typically `0x0029` or similar). We require
            // at least 50 % of the parsed UIDs to be non-zero, which rejects
            // accidental all-zero matches.
            let mut cur = Cursor::new(&data[count_pos + 4..nodes_end]);
            let mut tmp = Vec::with_capacity(n);
            let mut nonzero = 0usize;
            let mut ok = true;
            for _ in 0..n {
                match parse_node(&mut cur) {
                    Ok(node) => {
                        if node.uid != 0 {
                            nonzero += 1;
                        }
                        tmp.push(node);
                    }
                    Err(_) => {
                        ok = false;
                        break;
                    }
                }
            }
            if ok && (n == 0 || nonzero * 2 >= n) {
                sector.recovered_nodes_count = tmp.len();
                sector.nodes.extend(tmp);
                return;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Phase 5.8 audit walker — instrumented version of the legacy dispatch loop.
// ---------------------------------------------------------------------------

/// One item that was successfully walked during an audit.
#[derive(Debug, Clone)]
pub struct AuditedItem {
    /// Zero-based index inside the sector's item list.
    pub index: usize,
    /// Raw `item_type` u32 read from the sector.
    pub item_type: u32,
    /// Static name of the matched handler (`"road"`, `"prefab"`, …).
    pub kind_name: &'static str,
    /// Absolute offset (from the sector start) at which the item's `item_type`
    /// field was read.
    pub start_offset: usize,
    /// Absolute offset at which the item's body ended (== where the next
    /// item's `item_type` should be read from).
    pub end_offset: usize,
}

/// Failure record produced when the audit walker can't make progress.
#[derive(Debug, Clone)]
pub struct AuditFailure {
    /// Index of the item that failed (i.e. the (N+1)-th item if N items were
    /// already consumed successfully).
    pub item_index: usize,
    /// The raw u32 that was read where an `item_type` was expected. Often 0
    /// or garbage when the previous handler desynced the cursor.
    pub raw_type: u32,
    /// Absolute offset where the bad u32 was read from.
    pub error_offset: usize,
    /// Human-readable error description.
    pub error_msg: String,
}

/// Per-sector audit report — every item handled, and the failure (if any).
#[derive(Debug, Clone)]
pub struct AuditReport {
    /// `item_count` field from the sector header.
    pub item_count: u32,
    /// Items the walker consumed successfully, in order.
    pub items: Vec<AuditedItem>,
    /// The first failure, if any. `None` means the entire item list parsed
    /// cleanly.
    pub failure: Option<AuditFailure>,
    /// Total sector data length (bytes).
    pub data_len: usize,
}

/// Walk a sector's item dispatch loop, recording every successful item and
/// the first failure. Reuses the same skip/parse handlers as the production
/// parser so the audit reflects exactly what `parse_sector_legacy` would do.
///
/// Does NOT walk the trailing node section — Phase 5.8 only investigates
/// item-handler alignment.
pub fn audit_sector(data: &[u8]) -> AuditReport {
    let mut cur = Cursor::new(data);
    let mut report = AuditReport {
        item_count: 0,
        items: Vec::new(),
        failure: None,
        data_len: data.len(),
    };

    // 16-byte header: version u32 + game_id u64 + map_version u32
    if read_u32(&mut cur).is_err() || read_u64(&mut cur).is_err() || read_u32(&mut cur).is_err() {
        report.failure = Some(AuditFailure {
            item_index: 0,
            raw_type: 0,
            error_offset: 0,
            error_msg: "sector header truncated".into(),
        });
        return report;
    }

    let item_count = match read_u32(&mut cur) {
        Ok(c) => c,
        Err(e) => {
            report.failure = Some(AuditFailure {
                item_index: 0,
                raw_type: 0,
                error_offset: cur.position() as usize,
                error_msg: format!("item_count read failed: {e}"),
            });
            return report;
        }
    };
    if item_count > MAX_LIST_COUNT {
        report.failure = Some(AuditFailure {
            item_index: 0,
            raw_type: 0,
            error_offset: cur.position().saturating_sub(4) as usize,
            error_msg: format!("implausible item_count {item_count}"),
        });
        return report;
    }
    report.item_count = item_count;

    let mut throwaway = ParsedSector::default();
    for idx in 0..item_count as usize {
        let pos_before_type = cur.position() as usize;
        let item_type = match read_u32(&mut cur) {
            Ok(t) => t,
            Err(e) => {
                report.failure = Some(AuditFailure {
                    item_index: idx,
                    raw_type: 0,
                    error_offset: pos_before_type,
                    error_msg: format!("item_type u32 read failed: {e}"),
                });
                return report;
            }
        };

        let result: Result<&'static str, ParseError> = match item_type {
            ITEM_TYPE_ROAD => parse_road(&mut cur, &mut throwaway).map(|_| "road"),
            ITEM_TYPE_PREFAB => parse_prefab(&mut cur, &mut throwaway).map(|_| "prefab"),
            ITEM_TYPE_TERRAIN => skip_terrain(&mut cur).map(|_| "terrain"),
            ITEM_TYPE_BUILDINGS => skip_buildings(&mut cur).map(|_| "buildings"),
            ITEM_TYPE_MODEL => skip_model(&mut cur).map(|_| "model"),
            ITEM_TYPE_COMPANY => skip_company(&mut cur).map(|_| "company"),
            ITEM_TYPE_SERVICE => skip_service(&mut cur).map(|_| "service"),
            ITEM_TYPE_CUT_PLANE => skip_cut_plane(&mut cur).map(|_| "cut_plane"),
            ITEM_TYPE_CITY => skip_city(&mut cur).map(|_| "city"),
            ITEM_TYPE_MAP_OVERLAY => skip_map_overlay(&mut cur).map(|_| "map_overlay"),
            ITEM_TYPE_FERRY => skip_ferry(&mut cur).map(|_| "ferry"),
            ITEM_TYPE_GARAGE => skip_garage(&mut cur).map(|_| "garage"),
            ITEM_TYPE_TRIGGER => skip_trigger(&mut cur).map(|_| "trigger"),
            ITEM_TYPE_FUEL_PUMP => skip_fuel_pump(&mut cur).map(|_| "fuel_pump"),
            ITEM_TYPE_SIGN => skip_sign(&mut cur).map(|_| "sign"),
            ITEM_TYPE_BUS_STOP => skip_bus_stop(&mut cur).map(|_| "bus_stop"),
            ITEM_TYPE_TRAFFIC_AREA => skip_traffic_area(&mut cur).map(|_| "traffic_area"),
            ITEM_TYPE_BEZIER_PATCH => skip_bezier_patch(&mut cur).map(|_| "bezier_patch"),
            ITEM_TYPE_TRAJECTORY => skip_trajectory(&mut cur).map(|_| "trajectory"),
            ITEM_TYPE_MAP_AREA => skip_map_area(&mut cur).map(|_| "map_area"),
            ITEM_TYPE_FAR_MODEL => skip_far_model(&mut cur).map(|_| "far_model"),
            ITEM_TYPE_CURVE => skip_curve(&mut cur).map(|_| "curve"),
            ITEM_TYPE_CUTSCENE => skip_cutscene(&mut cur).map(|_| "cutscene"),
            ITEM_TYPE_VISIBILITY_AREA => skip_visibility_area(&mut cur).map(|_| "visibility_area"),
            ITEM_TYPE_MOVER => skip_mover(&mut cur).map(|_| "mover"),
            ITEM_TYPE_NO_WEATHER => skip_no_weather(&mut cur).map(|_| "no_weather"),
            ITEM_TYPE_HINGE => skip_hinge(&mut cur).map(|_| "hinge"),
            ITEM_TYPE_CAMERA_POINT => skip_camera_point(&mut cur).map(|_| "camera_point"),
            ITEM_TYPE_COMPOUND => parse_compound(&mut cur, &mut throwaway).map(|_| "compound"),
            ITEM_TYPE_CAMERA_PATH => skip_camera_path(&mut cur).map(|_| "camera_path"),
            ITEM_TYPE_HOOKUP => skip_hookup(&mut cur).map(|_| "hookup"),
            ITEM_TYPE_GATE => skip_gate(&mut cur).map(|_| "gate"),
            other => Err(ParseError::Binary(format!("unsupported item type {other}"))),
        };

        match result {
            Ok(name) => {
                let end_offset = cur.position() as usize;
                report.items.push(AuditedItem {
                    index: idx,
                    item_type,
                    kind_name: name,
                    start_offset: pos_before_type,
                    end_offset,
                });
            }
            Err(e) => {
                report.failure = Some(AuditFailure {
                    item_index: idx,
                    raw_type: item_type,
                    error_offset: pos_before_type,
                    error_msg: format!("{e}"),
                });
                return report;
            }
        }
    }

    report
}

/// Attempt the sized-format parse. Returns `None` if the header doesn't look
/// sized so the caller can fall back to the legacy parser.
///
/// Detection: we treat the data as sized format when the 20-byte header parses
/// cleanly **and** the first `min(4, item_count)` items walk through cleanly
/// using the `type+size+payload` pattern without overrunning the buffer.
fn try_parse_sized_sector(
    data: &[u8],
    tracer_ctx: Option<(&DropTracer, &str)>,
) -> Option<ParsedSector> {
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
                && (count as usize).saturating_mul(node_record_size) == remaining.saturating_sub(4)
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
            ITEM_TYPE_ROAD => match parse_sized_road(&mut cur) {
                Ok(road) => sector.roads.push(road),
                Err(_) => {
                    if let Some((tracer, sector_path)) = tracer_ctx {
                        let hex_cap = tracer.hex_limit.min(item_size as usize);
                        let body = data
                            .get(item_start as usize..item_start as usize + hex_cap)
                            .unwrap_or(&[]);
                        tracer.record(DropEvent {
                            category: DropCategory::SizedRoadParseFailed,
                            sector_path: sector_path.to_string(),
                            item_type: ITEM_TYPE_ROAD,
                            item_uid: None,
                            node_a: None,
                            node_b: None,
                            node_a_resolved: None,
                            node_b_resolved: None,
                            x: None,
                            z: None,
                            raw_hex: body.to_vec(),
                        });
                    }
                }
            },
            ITEM_TYPE_PREFAB => {
                if let Ok(prefab) = parse_sized_prefab(&mut cur) {
                    sector.prefabs.push(prefab);
                }
            }
            ITEM_TYPE_FERRY => {
                let _ = parse_ferry(&mut cur, &mut sector);
            }
            ITEM_TYPE_BUILDINGS => {
                let _ = parse_buildings(&mut cur, &mut sector);
            }
            ITEM_TYPE_COMPOUND => {
                // Compound child nodes use the legacy 56-byte layout even
                // inside sized-format sectors; any parse error is tolerated
                // and the cursor is corrected by the item_end seek below.
                let _ = parse_compound(&mut cur, &mut sector);
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
                // ── Plausibility Gate (Phase 6.2b-Fix-5a: Root Cause D) ───────
                // Reject false-positive sized-sector accepts that would
                // otherwise prevent the legacy parser from running on a
                // perfectly good legacy sector.
                if !sized_sector_plausible(item_count, &sector, data) {
                    return None;
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

    if !sized_sector_plausible(item_count, &sector, data) {
        return None;
    }
    Some(sector)
}

/// Plausibility gate for sized-sector acceptance (Phase 6.2b-Fix-5a).
///
/// Returns `false` when the parsed sector is likely a false-positive
/// (a legacy sector whose header bytes happened to decode as a valid
/// sized header). Two-stage check:
///   1. Item count must be plausible for the file size.
///   2. If nodes are present, at least one must have valid ETS2 world
///      coordinates (wrong parse stride produces garbage floats).
fn sized_sector_plausible(item_count: u32, sector: &ParsedSector, data: &[u8]) -> bool {
    const MIN_SIZED_ITEMS_FOR_LARGE_SECTOR: u32 = 10;
    const LARGE_SECTOR_THRESHOLD_BYTES: usize = 50_000;
    const MAX_WORLD_X_Z: f32 = 250_000.0;
    const MAX_WORLD_Y: f32 = 10_000.0;

    // Stage 1 — Item-count heuristic
    if item_count < MIN_SIZED_ITEMS_FOR_LARGE_SECTOR && data.len() > LARGE_SECTOR_THRESHOLD_BYTES {
        tracing::debug!(
            target: "map_parser::sector",
            "try_parse_sized_sector rejected: item_count={} too small for {}-byte sector",
            item_count, data.len()
        );
        return false;
    }

    // Stage 2 — Node coordinate plausibility
    if !sector.nodes.is_empty()
        && !sector.nodes.iter().any(|n| {
            n.x.abs() < MAX_WORLD_X_Z && n.y.abs() < MAX_WORLD_Y && n.z.abs() < MAX_WORLD_X_Z
        })
    {
        tracing::debug!(
            target: "map_parser::sector",
            "try_parse_sized_sector rejected: no node with valid world coords in {}-byte sector ({} nodes)",
            data.len(), sector.nodes.len()
        );
        return false;
    }

    true
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
        dlc_guard: 0,
        is_hidden: false,
        gps_avoid: false,
        road_type_token: 0,
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
        template_token: token,
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
        rotation: [0.0; 4],
        forward_item_uid: 0,
        backward_item_uid: 0,
    })
}

// ---------------------------------------------------------------------------
// Item parsers
// ---------------------------------------------------------------------------

/// Parse one Type-3 (Road) item via the full binrw struct pipeline:
/// 265-byte fixed header followed by the variable-length data payload.
///
/// On binrw failure we emit a `warn!` carrying the cursor position
/// (relative to the road start) and return `Err`, which aborts the
/// surrounding sector.  Aborting the sector is safer than guessing — a
/// desynced cursor would corrupt every following item.
fn parse_road(cur: &mut Cursor<&[u8]>, sector: &mut ParsedSector) -> Result<(), ParseError> {
    let road_start = cur.position();

    let header = RoadFixedHeader::read(cur).map_err(|e| {
        let consumed = cur.position().saturating_sub(road_start);
        warn!(
            road_start,
            consumed, "Road fixed header parse failed at byte +{consumed} of road body: {e}"
        );
        ParseError::Binary(format!("road fixed header at +{consumed} bytes: {e}"))
    })?;

    // Phase 5.7: ETS2 v907 `base_map.scs` does NOT carry a variable
    // RoadDataPayload after the 265-byte fixed header (verified empirically
    // via `truckpilot-road-dump`). The cursor is already correctly positioned
    // at the next item / node section. The `RoadDataPayload` struct is kept
    // in `road_full.rs` as documented format reference for editor-saved
    // sectors that may use it.
    let _ = road_start; // silence unused-warning, kept for diagnostic context

    sector.roads.push(RawRoad {
        uid: header.uid,
        node_a: header.start_node_uid,
        node_b: header.end_node_uid,
        speed_limit_kmh: 0,
        lanes_forward: 0,
        lanes_backward: 0,
        look_token: header.left_look,
        dlc_guard: header.dlc_guard,
        is_hidden: header.is_hidden(),
        gps_avoid: header.gps_avoid(),
        road_type_token: header.road_type,
    });
    Ok(())
}

fn parse_prefab(cur: &mut Cursor<&[u8]>, sector: &mut ParsedSector) -> Result<(), ParseError> {
    // KdopItem: uid(8) + bounds(40) + flags(4) + view_dist(1) = 53 bytes
    let uid = read_u64(cur)?;
    skip(cur, 45)?; // bounds(40) + flags(4) + view_dist(1)

    let model_token = read_u64(cur)?; // field A: offset 53
    let _variant_token = read_u64(cur)?; // field B: offset 61

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
        template_token: model_token,
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
    let qw = read_f32(cur)?;
    let qx = read_f32(cur)?;
    let qy = read_f32(cur)?;
    let qz = read_f32(cur)?;
    let backward_item_uid = read_u64(cur)?;
    let forward_item_uid = read_u64(cur)?;
    skip(cur, 4)?; // flags(u32)
                   // total: 8+4+4+4+16+8+8+4 = 56 bytes

    Ok(RawNode {
        uid,
        x: x_raw as f32 / 256.0,
        y: y_raw as f32 / 256.0,
        z: z_raw as f32 / 256.0,
        rotation: [qw, qx, qy, qz],
        forward_item_uid,
        backward_item_uid,
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
/// Type 1 — Terrain.
///
/// Phase 5.16 rewrite: the legacy 3-token+u16+token+u16+u32+u32+3×float-list
/// layout was a placeholder that desynced on every v907 terrain item, leading
/// to "float list: count … exceeds safety limit" failures in 25/154 sectors
/// pre-trigger-fix and 39/103 (37.9 %) of remaining failures after that fix.
///
/// The actual v907 layout matches TruckLib's `TerrainSerializer.Deserialize`
/// (read 2026-05-10, no code copied — field order in own words):
///
/// 1. `read_kdop_item` already consumes uid + kdop bounds + 4 flag bytes +
///    view_distance.
/// 2. Then in order: u64 Node + u64 ForwardNode + vec3 NodeOffset + vec3
///    ForwardNodeOffset + f32 Length + f32 prev_length + u32 RandomSeed.
/// 3. Four railings, each `(u64 model_token, i16 offset)`.
/// 4. Two sides (right, left) with the same layout as in [`skip_curve`]:
///    u16 size + token profile + f32 coef + token prev_profile + f32
///    prev_coef + 3 × 16-byte vegetation + 2 × u16 detail-veg distances.
/// 5. Vegetation sphere list (count u32 + entries).
/// 6. Two `terrain_quad_data` blocks (right, then left).
/// 7. Four trailing edge tokens (right_edge, right_edge_look, left_edge,
///    left_edge_look).
///
/// The only difference vs. `skip_curve` is the railing count (4 vs. 3).
fn skip_terrain(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
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

/// Type 2 — Buildings.
///
/// Phase 5.19 rewrite. The legacy port consumed only 93 bytes per item
/// (kdop + Name + Look + Node + ForwardNode + Length + RandomSeed),
/// under-reading by 8 + 4 N bytes. In the Phase 5.18 audit `buildings`
/// was the top failing predecessor at 26/36 (72.2 %); 26/26 samples
/// landed on `0x3F800000` (= IEEE float 1.0, the default Stretch
/// coefficient) at `rel_end = +0`, a deterministic constant marker.
///
/// The actual v907 layout matches TruckLib's `BuildingsSerializer`
/// (read 2026-05-10, no code copied — see `outputs/buildings_format_notes.md`):
/// kdop_item, then `Name` token, `Look` token, `Node` u64, `ForwardNode`
/// u64, `Length` f32, `RandomSeed` u32, `Stretch` f32, `HeightOffsets`
/// (u32 count + N × f32). Total = 101 + 4 × N bytes.
///
/// Empirically verified: at handler-claimed end the next 4 bytes are
/// always Stretch=1.0, the next 4 a count, and `8 + 4 × count` bytes
/// later sits the next item_type — a perfect alignment match.
fn parse_buildings(cur: &mut Cursor<&[u8]>, sector: &mut ParsedSector) -> Result<(), ParseError> {
    let uid = read_kdop_item(cur)?;
    skip_token(cur)?; // Name
    skip_token(cur)?; // Look
    let node_uid = read_u64(cur)?;
    let forward_node_uid = read_u64(cur)?;
    let _ = read_f32(cur)?; // Length
    let _ = read_u32(cur)?; // RandomSeed
    let _ = read_f32(cur)?; // Stretch
    let height_offsets_count = read_u32(cur)?;
    ensure_count(height_offsets_count, "buildings height offsets")?;
    ensure_capacity(cur, height_offsets_count, 4, "buildings height offsets")?;
    for _ in 0..height_offsets_count {
        let _ = read_f32(cur)?;
    }
    sector.buildings.push(RawBuilding {
        uid,
        node_uid,
        forward_node_uid,
    });
    Ok(())
}

/// Audit-only buildings skip (no capture). Identical byte advance to
/// [`parse_buildings`] — used by `audit_sector` which doesn't materialise
/// items.
fn skip_buildings(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    skip_token(cur)?;
    skip_token(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_u64(cur)?;
    let _ = read_f32(cur)?;
    let _ = read_u32(cur)?;
    let _ = read_f32(cur)?;
    let height_offsets_count = read_u32(cur)?;
    ensure_count(height_offsets_count, "buildings height offsets")?;
    ensure_capacity(cur, height_offsets_count, 4, "buildings height offsets")?;
    for _ in 0..height_offsets_count {
        let _ = read_f32(cur)?;
    }
    Ok(())
}

/// Type 5 — Model.
///
/// Phase 5.18 rewrite. The legacy port consumed only 86 bytes per item
/// (kdop + 3 × u64 + token + u8), under-reading by at least 31 bytes.
/// In the Phase 5.17 audit `model` was the top failing predecessor at
/// 53/77 (68.8 %); the Δ-scan window of [-16, +32] found no valid
/// item-type at any of 20 samples, i.e. the missing tail was wider than
/// 32 bytes — consistent with the AdditionalParts list pushing the true
/// end far past the claimed end_offset.
///
/// The actual v907 layout matches TruckLib's `ModelSerializer.Deserialize`
/// (read 2026-05-10, no code copied — see `outputs/model_format_notes.md`):
/// kdop_item, then `Name` token, `Look` token, `Variant` token,
/// `AdditionalParts` (u32 count + count × u64 token), `Node` u64,
/// `Scale` vec3, `TerrainMaterial` token, `TerrainColor` (4 × u8 RGBA),
/// `TerrainRotation` f32. Total = 117 + 8 × N bytes where N is the
/// AdditionalParts count.
///
/// Empirically verified at sec+0000+0001.base body offset 0x27f, N=0:
/// Scale = (1.0, 1.0, 1.0), TerrainColor = white, and the next u32 at
/// body+117 reads as `5` (= ITEM_TYPE_MODEL), the strongest possible
/// alignment signal.
fn skip_model(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    skip_token(cur)?; // Name
    skip_token(cur)?; // Look
    skip_token(cur)?; // Variant
    let additional_parts = read_u32(cur)?;
    ensure_count(additional_parts, "model additional parts")?;
    ensure_capacity(cur, additional_parts, 8, "model additional parts")?;
    for _ in 0..additional_parts {
        skip_token(cur)?;
    }
    let _ = read_u64(cur)?; // Node
    skip_vector3(cur)?; // Scale
    skip_token(cur)?; // TerrainMaterial
    skip_color(cur)?; // TerrainColor (4 bytes)
    let _ = read_f32(cur)?; // TerrainRotation
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

/// Type 19 — Ferry. Phase 5.22 captures port_token + node_uid for
/// cross-sector clique-edge generation in [`crate::graph::GraphBuilder`].
/// Body layout (89 bytes after item type tag): KdopItem (53) + port_token
/// u64 + prefab_uid u64 + node_uid u64 + unload_offset 3×f32. Source:
/// `outputs/ferry_format_notes.md` (TruckLib FerrySerializer + ts-map
/// cross-check, no code copied).
fn parse_ferry(cur: &mut Cursor<&[u8]>, sector: &mut ParsedSector) -> Result<(), ParseError> {
    let uid = read_kdop_item(cur)?;
    let port_token = read_u64(cur)?;
    let prefab_uid = read_u64(cur)?;
    let node_uid = read_u64(cur)?;
    let _ = read_f32(cur)?; // unload_offset.x
    let _ = read_f32(cur)?; // unload_offset.y
    let _ = read_f32(cur)?; // unload_offset.z
    sector.ferries.push(RawFerry {
        uid,
        port_token,
        prefab_uid,
        node_uid,
    });
    Ok(())
}

/// Audit-only ferry skip (no capture). Identical byte advance to
/// [`parse_ferry`] — used by `audit_sector` which doesn't materialise items.
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

/// Type 36 — Sign.
///
/// Phase 5.12 rewrite — the original port from `binary_parser.rs:802-815`
/// matched the byte count for empty-template signs but desynced for any
/// non-empty template (those have variable bytes between the boards
/// section and the override lists). The audit walker showed `sign` as
/// the source of 56.8 % of all sector parse failures.
///
/// Layout per TruckLib `SignSerializer.cs` (sk-zk/TruckLib, MIT-licensed
/// reference — used here for format facts only, no code copied):
///   1. kdop_item                         (53 B)
///   2. Model token                       (8 B)
///   3. Node UID u64                      (8 B)
///   4. Look token                        (8 B)
///   5. Variant token                     (8 B)
///   6. board_count u8                    (1 B)
///   7. for each board: Road / City1 / City2 tokens (3 tokens = 24 B)
///   8. SignTemplate PascalString         (8 B header + N B payload)
///   9. **Only if SignTemplate is non-empty:**
///      - SignBoardOverride list
///      - SignOverride list
fn skip_sign(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    skip_token(cur)?; // Model
    let _ = read_u64(cur)?; // Node UID
    skip_token(cur)?; // Look
    skip_token(cur)?; // Variant
    let board_count = read_u8(cur)? as u32;
    for _ in 0..board_count {
        skip_token(cur)?; // Road
        skip_token(cur)?; // City1
        skip_token(cur)?; // City2
    }
    let template_len = read_pascal_string_len(cur)?;
    if template_len > 0 {
        skip_sign_board_override_list(cur)?;
        skip_sign_override_list(cur)?;
    }
    Ok(())
}

/// Read a Pascal-style string (u64 length + raw bytes) and return the
/// length without copying the bytes.  Used by `skip_sign` to decide
/// whether the override lists follow.
fn read_pascal_string_len(cur: &mut Cursor<&[u8]>) -> Result<u64, ParseError> {
    let len = read_u64(cur)?;
    if len > MAX_PASCAL_STRING_LEN {
        return Err(ParseError::Binary(format!(
            "pascal string length {len} exceeds safety limit"
        )));
    }
    if len > usize::MAX as u64 {
        return Err(ParseError::Binary(format!(
            "pascal string length {len} exceeds usize::MAX"
        )));
    }
    skip(cur, len as usize)?;
    Ok(len)
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

/// Type 39 — BezierPatch. Full layout (Phase 6.2b-Fix-5c: vegetation=3 entries).
/// Layout: kdop(53) + 16×vec3(192) + tess(4) + node(8) + seed(4)
///        + vegetation 3×(u64+u16+u8)=33 + sphere_count×20
///        + TerrainQuadData: mat_count×10, col_count×4, rows, cols,
///          quad_count×4, off_count×16, norm_count×16
/// Empty-patch fixed overhead: 318 bytes (all list counts = 0).
fn skip_bezier_patch(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?; // 53
    for _ in 0..16 {
        skip_vector3(cur)?; // 16×12 = 192
    }
    let _ = read_u16(cur)?; // tess_x u16
    let _ = read_u16(cur)?; // tess_z u16
    let _ = read_u64(cur)?; // node uid
    let _ = read_u32(cur)?; // random seed
                            // Vegetation[0..3]: each entry = u64 token + u16 density + u8 type = 11 bytes
    skip(cur, 3 * 11)?; // 33 bytes
                        // VegetationSpheres: u32 count + count × 20 bytes (vec3 + f32 radius + u32 type)
    let sphere_count = read_u32(cur)? as usize;
    ensure_count(sphere_count as u32, "bezier_patch vegetation spheres")?;
    skip(cur, sphere_count * 20)?;
    // TerrainQuadData
    // Materials: u16 count + count × 10 bytes (u64 token + u16 rotation)
    let mat_count = read_u16(cur)? as usize;
    ensure_count(mat_count as u32, "bezier_patch materials")?;
    skip(cur, mat_count * 10)?;
    // Colors: u16 count + count × 4 bytes (RGBA)
    let col_count = read_u16(cur)? as usize;
    ensure_count(col_count as u32, "bezier_patch colors")?;
    skip(cur, col_count * 4)?;
    let _ = read_u16(cur)?; // rows
    let _ = read_u16(cur)?; // cols
                            // Quads: u32 count + count × 4 bytes (u32 index)
    let quad_count = read_u32(cur)? as usize;
    ensure_count(quad_count as u32, "bezier_patch quads")?;
    skip(cur, quad_count * 4)?;
    // Offsets: u32 count + count × 16 bytes (u16 X + u16 Y + vec3)
    let off_count = read_u32(cur)? as usize;
    ensure_count(off_count as u32, "bezier_patch offsets")?;
    skip(cur, off_count * 16)?;
    // Normals: u32 count + count × 16 bytes
    let norm_count = read_u32(cur)? as usize;
    ensure_count(norm_count as u32, "bezier_patch normals")?;
    skip(cur, norm_count * 16)?;
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

/// Type 44 — Curve.
///
/// Phase 5.17 rewrite. The legacy port (kdop, two u64s, two vec3s, two f32s,
/// a u32, three railings, two side blocks, a sphere list, two quad-data
/// blocks, four trailing tokens) was a placeholder that desynced on every
/// v907 curve item. Phase 5.16 made it byte-identical to the rewritten
/// skip_terrain on the theory that "curve and terrain carry identical body
/// layout in the engine"; the audit refuted that — terrain failures dropped
/// to 0 while curve failures stayed at 36/96 (37.5 %), all crashing inside
/// `skip_vegetation_sphere_list` or `skip_terrain_quad_data` with absurd
/// counts (cursor already misaligned by then).
///
/// The actual v907 layout matches TruckLib's `CurveSerializer.Deserialize`
/// (read 2026-05-10, no code copied — see `outputs/curve_format_notes.md`):
/// kdop_item (uid, 10×f32 bounds, flags, view-distance), then `Node` u64,
/// `ForwardNode` u64, two locator u64s (always on disk; the engine filters
/// zero entries in-memory but the bytes are always present), `Length` f32,
/// `SubcurveUseMask` u32 bitmask, then `popcount(mask)` × `Subcurve`. A
/// `Subcurve` is 100 fixed bytes plus 4 × `heightOffsetCount` extra bytes
/// (the only count-driven block in the curve body).
///
/// Curve shares only the first 69 bytes with terrain (kdop + Node +
/// ForwardNode); after that they are unrelated layouts. No shared
/// helper — see `outputs/curve_format_notes.md` for the diff.
fn skip_curve(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    let _ = read_u64(cur)?; // Node
    let _ = read_u64(cur)?; // ForwardNode
    let _ = read_u64(cur)?; // Locator[0] — always on disk, filtered in-memory
    let _ = read_u64(cur)?; // Locator[1]
    let _ = read_f32(cur)?; // Length
    let mask = read_u32(cur)?; // SubcurveUseMask
    let subcurve_count = mask.count_ones();
    for _ in 0..subcurve_count {
        skip_subcurve(cur)?;
    }
    Ok(())
}

/// Subcurve body (100 fixed bytes + 4·H for the height-offset list).
///
/// Field order per TruckLib `Subcurve.Deserialize`:
/// Model token, flags u32, Seed u32, Stretch f32, Scale f32, FixedStep f32,
/// TerrainMaterial token, TerrainColor (4×u8), TerrainRotation f32,
/// FirstPart token, LastPart token, CenterPartVariation token, Look token,
/// HeightOffsets (u32 count + count × f32), InitialHeightOffset f32,
/// OffsetFromBaseCurveStartX/Y f32, OffsetFromBaseCurveEndX/Y f32.
fn skip_subcurve(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    skip_token(cur)?; // Model
    let _ = read_u32(cur)?; // flags
    let _ = read_u32(cur)?; // Seed
    let _ = read_f32(cur)?; // Stretch
    let _ = read_f32(cur)?; // Scale
    let _ = read_f32(cur)?; // FixedStep
    skip_token(cur)?; // TerrainMaterial
    skip_color(cur)?; // TerrainColor (4 bytes)
    let _ = read_f32(cur)?; // TerrainRotation
    skip_token(cur)?; // FirstPart
    skip_token(cur)?; // LastPart
    skip_token(cur)?; // CenterPartVariation
    skip_token(cur)?; // Look
    let height_offset_count = read_u32(cur)?;
    ensure_count(height_offset_count, "subcurve height offsets")?;
    ensure_capacity(cur, height_offset_count, 4, "subcurve height offsets")?;
    for _ in 0..height_offset_count {
        let _ = read_f32(cur)?;
    }
    let _ = read_f32(cur)?; // InitialHeightOffset
    let _ = read_f32(cur)?; // OffsetFromBaseCurveStartX
    let _ = read_f32(cur)?; // OffsetFromBaseCurveStartY
    let _ = read_f32(cur)?; // OffsetFromBaseCurveEndX
    let _ = read_f32(cur)?; // OffsetFromBaseCurveEndY
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

/// Type 9 — Mover. Binary layout (after KdopItem): tags(token list) +
/// model/look/variant(3×u64) + speed/endDelay/width(3×f32) + count(u32) +
/// lengths(f32 list) + nodeUids(u64 list).
fn skip_mover(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    skip_token_list(cur)?;
    skip_token(cur)?; // model
    skip_token(cur)?; // look
    skip_token(cur)?; // variant
    let _ = read_f32(cur)?; // speed
    let _ = read_f32(cur)?; // endDelay
    let _ = read_f32(cur)?; // width
    let _ = read_u32(cur)?; // count
    let len_count = read_u32(cur)?;
    ensure_count(len_count, "mover lengths")?;
    ensure_capacity(cur, len_count, 4, "mover lengths")?;
    for _ in 0..len_count {
        let _ = read_f32(cur)?;
    }
    let node_count = read_u32(cur)?;
    ensure_count(node_count, "mover node uids")?;
    ensure_capacity(cur, node_count, 8, "mover node uids")?;
    for _ in 0..node_count {
        let _ = read_u64(cur)?;
    }
    Ok(())
}

/// Type 11 — NoWeather. Binary layout (after KdopItem): width(f32) +
/// height(f32) + fogMaskPresetId(i32) + reserved(16 bytes) + nodeUid(u64).
fn skip_no_weather(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    let _ = read_f32(cur)?; // width
    let _ = read_f32(cur)?; // height
    let _ = read_i32(cur)?; // fogMaskPresetId
    skip(cur, 16)?; // reserved (new in v901)
    let _ = read_u64(cur)?; // nodeUid
    Ok(())
}

/// Type 13 — Hinge. Binary layout (after KdopItem): token(u64) + look(u64) +
/// nodeUid(u64) + minRot(f32) + maxRot(f32).
fn skip_hinge(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    skip_token(cur)?; // token
    skip_token(cur)?; // look
    let _ = read_u64(cur)?; // nodeUid
    let _ = read_f32(cur)?; // minRot
    let _ = read_f32(cur)?; // maxRot
    Ok(())
}

/// Type 23 — CameraPoint. Binary layout (after KdopItem): tags(token list) +
/// nodeUid(u64).
fn skip_camera_point(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    skip_token_list(cur)?;
    let _ = read_u64(cur)?; // nodeUid
    Ok(())
}

/// Type 40 — Compound. Binary layout (after KdopItem): nodeUid(u64) +
/// childItems(count u32 + count × SimpleItem) + childNodes(count u32 +
/// count × 56-byte SectorNode).
///
/// Phase 5.28-B: child nodes are extracted into `sector.nodes` so that
/// Roads and Prefabs referencing these UIDs can find their endpoints.
/// Child items are still skipped (their geometry is decorative).
fn parse_compound(cur: &mut Cursor<&[u8]>, sector: &mut ParsedSector) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    let _ = read_u64(cur)?; // nodeUid (the compound's own attachment node)
    let child_item_count = read_u32(cur)?;
    ensure_count(child_item_count, "compound child items")?;
    for i in 0..child_item_count {
        skip_child_simple_item(cur)
            .map_err(|e| ParseError::Binary(format!("compound child item #{i}: {e}")))?;
    }
    let child_node_count = read_u32(cur)?;
    ensure_count(child_node_count, "compound child nodes")?;
    ensure_capacity(cur, child_node_count, 56, "compound child nodes")?;
    for _ in 0..child_node_count {
        sector.nodes.push(parse_node(cur)?);
    }
    Ok(())
}

/// Skip a single SimpleItem (type u32 + KdopItem + type-specific body).
/// Used inside [`parse_compound`] for child items.
fn skip_child_simple_item(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let item_type = read_u32(cur)?;
    match item_type {
        ITEM_TYPE_TERRAIN => skip_terrain(cur),
        ITEM_TYPE_BUILDINGS => skip_buildings(cur),
        ITEM_TYPE_MODEL => skip_model(cur),
        ITEM_TYPE_COMPANY => skip_company(cur),
        ITEM_TYPE_SERVICE => skip_service(cur),
        ITEM_TYPE_CUT_PLANE => skip_cut_plane(cur),
        ITEM_TYPE_CITY => skip_city(cur),
        ITEM_TYPE_MAP_OVERLAY => skip_map_overlay(cur),
        ITEM_TYPE_FERRY => skip_ferry(cur),
        ITEM_TYPE_GARAGE => skip_garage(cur),
        ITEM_TYPE_TRIGGER => skip_trigger(cur),
        ITEM_TYPE_FUEL_PUMP => skip_fuel_pump(cur),
        ITEM_TYPE_SIGN => skip_sign(cur),
        ITEM_TYPE_BUS_STOP => skip_bus_stop(cur),
        ITEM_TYPE_TRAFFIC_AREA => skip_traffic_area(cur),
        ITEM_TYPE_BEZIER_PATCH => skip_bezier_patch(cur),
        ITEM_TYPE_TRAJECTORY => skip_trajectory(cur),
        ITEM_TYPE_MAP_AREA => skip_map_area(cur),
        ITEM_TYPE_FAR_MODEL => skip_far_model(cur),
        ITEM_TYPE_CURVE => skip_curve(cur),
        ITEM_TYPE_CUTSCENE => skip_cutscene(cur),
        ITEM_TYPE_VISIBILITY_AREA => skip_visibility_area(cur),
        ITEM_TYPE_MOVER => skip_mover(cur),
        ITEM_TYPE_NO_WEATHER => skip_no_weather(cur),
        ITEM_TYPE_HINGE => skip_hinge(cur),
        ITEM_TYPE_CAMERA_POINT => skip_camera_point(cur),
        ITEM_TYPE_CAMERA_PATH => skip_camera_path(cur),
        ITEM_TYPE_HOOKUP => skip_hookup(cur),
        ITEM_TYPE_GATE => skip_gate(cur),
        other => Err(ParseError::Binary(format!(
            "unsupported compound child item type {other}"
        ))),
    }
}

/// Type 45 — CameraPath. Binary layout (after KdopItem): tags(token list) +
/// nodeUids(u64 list) + trackPointNodeUids(u64 list) +
/// curveControlNodeUids(u64 list) + keyFrames(count u32 + count × 40 bytes) +
/// speed(f32).
///
/// Each keyframe is 40 bytes: speedChange(i32) + rotationChange(i32) +
/// speedCoef(f32) + fov(f32) + backwardTangentPos(vec3=12) +
/// forwardTangentPos(vec3=12).
fn skip_camera_path(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    skip_token_list(cur)?;
    let _ = skip_node_ref_list(cur)?; // nodeUids
    let _ = skip_node_ref_list(cur)?; // trackPointNodeUids
    let _ = skip_node_ref_list(cur)?; // curveControlNodeUids
    let keyframe_count = read_u32(cur)?;
    ensure_count(keyframe_count, "camera path keyframes")?;
    ensure_capacity(cur, keyframe_count, 40, "camera path keyframes")?;
    skip(cur, keyframe_count as usize * 40)?;
    let _ = read_f32(cur)?; // speed
    Ok(())
}

/// Type 47 — Hookup. Binary layout (after KdopItem): name(Pascal string) +
/// nodeUid(u64).
fn skip_hookup(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    skip_pascal_string(cur)?; // name
    let _ = read_u64(cur)?; // nodeUid
    Ok(())
}

/// Type 49 — Gate. Binary layout (after KdopItem): model(u64) +
/// nodeUids(u64 list) + 2 activation point units each (Pascal string +
/// i32). The activation count is always exactly 2 (fixed per binary spec).
fn skip_gate(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let _ = read_kdop_item(cur)?;
    skip_token(cur)?; // model
    let _ = skip_node_ref_list(cur)?; // nodeUids
    for _ in 0..2 {
        skip_pascal_string(cur)?; // triggerUnitName
        let _ = read_i32(cur)?; // triggerNodeIndex
    }
    Ok(())
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

/// Skip an action base.
///
/// Phase 5.15 fix: TruckLib's `ActionBase.Deserialize` documents that a
/// `numeric_params_count` value of `0xFFFFFFFF` is a sentinel meaning
/// "no params, deserialization ends here" — the remaining fields
/// (string params, target tags, range, flags) are NOT present on disk.
/// The legacy reference parser missed this and bailed with
/// `count 4294967295 exceeds safety limit`, which produced 89/154
/// (57.8 %) of the v907 sector audit failures.
fn skip_action_base(cur: &mut Cursor<&[u8]>) -> Result<(), ParseError> {
    let num_param_count = read_u32(cur)?;
    if num_param_count == 0xFFFF_FFFF {
        // Sentinel: action body is empty.
        return Ok(());
    }
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
        write_u64(&mut buf, 0); // GameId token
        write_u32(&mut buf, 1); // GameMapVersion
        buf
    }

    /// Append a node record (56 bytes) to buf.
    fn append_node(buf: &mut Vec<u8>, uid: u64, x_raw: i32, y_raw: i32, z_raw: i32) {
        write_u64(buf, uid);
        write_i32(buf, x_raw);
        write_i32(buf, y_raw);
        write_i32(buf, z_raw);
        buf.extend_from_slice(&[0u8; 16]); // quaternion
        write_u64(buf, 0); // backward_uid
        write_u64(buf, 0); // forward_uid
        write_u32(buf, 0); // flags
    }

    /// Append a road item (type tag + 265-byte fixed header) to buf.
    ///
    /// Phase 5.7 verified empirically that ETS2 v907 `base_map.scs` carries
    /// no variable payload after the fixed header — the next bytes are
    /// either the next item's `item_type` or the trailing `node_count`.
    /// `parse_road` therefore consumes exactly 4 (type tag) + 265 (fixed
    /// header) bytes, and the helper mirrors that.
    ///
    /// Layout written:
    ///   • 4 B item_type = ITEM_TYPE_ROAD
    ///   • 265 B fixed header (zeros except uid at +0, start at +0xF5, end at +0xFD)
    fn append_road(buf: &mut Vec<u8>, uid: u64, node_a: u64, node_b: u64) {
        write_u32(buf, ITEM_TYPE_ROAD); // type
        let header_start = buf.len();
        buf.extend_from_slice(&[0u8; 0x109]);
        buf[header_start..header_start + 8].copy_from_slice(&uid.to_le_bytes());
        buf[header_start + 0xF5..header_start + 0xF5 + 8].copy_from_slice(&node_a.to_le_bytes());
        buf[header_start + 0xFD..header_start + 0xFD + 8].copy_from_slice(&node_b.to_le_bytes());
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
    fn unknown_item_type_returns_partial_sector() {
        // Phase 5.8 changed the dispatcher: unknown item types no longer
        // abort the whole sector — `parse_sector_legacy` logs a warning,
        // sets `all_items_parsed = false`, and returns the items it did
        // dispatch successfully (here: none).  The trailing node section
        // is then either skipped or recovered via `recover_nodes_from_tail`.
        let mut data = header(895);
        write_u32(&mut data, 1); // item_count
        write_u32(&mut data, 99); // type 99 = unknown
        data.extend_from_slice(&[0u8; 64]);

        let s = parse_sector(&data).expect("partial recovery returns Ok");
        assert!(s.roads.is_empty());
        assert!(s.prefabs.is_empty());
    }

    #[test]
    fn truncated_data_returns_error() {
        // Header only, no item_count
        let data = header(895);
        assert!(parse_sector(&data).is_err());
    }

    // -------------------------------------------------------------------------
    // Phase 5.28-B — Compound child node extraction
    // -------------------------------------------------------------------------

    fn write_u8(buf: &mut Vec<u8>, v: u8) {
        buf.push(v);
    }
    fn write_f32(buf: &mut Vec<u8>, v: f32) {
        buf.extend_from_slice(&v.to_le_bytes());
    }

    /// Append the 53-byte KdopItem header.
    fn append_kdop_item(buf: &mut Vec<u8>, uid: u64) {
        write_u64(buf, uid); // uid
        for _ in 0..10 {
            write_f32(buf, 0.0); // 10 × f32 bounds
        }
        write_u32(buf, 0); // flags
        write_u8(buf, 0); // view_distance
    }

    /// Append a compound item (type tag + body) with zero child items
    /// and `nodes` child nodes.
    fn append_compound(
        buf: &mut Vec<u8>,
        uid: u64,
        node_uid: u64,
        child_nodes: &[(u64, i32, i32, i32)],
    ) {
        write_u32(buf, ITEM_TYPE_COMPOUND);
        append_kdop_item(buf, uid);
        write_u64(buf, node_uid); // nodeUid
        write_u32(buf, 0); // child item count = 0
        write_u32(buf, child_nodes.len() as u32);
        for &(cuid, cx, cy, cz) in child_nodes {
            append_node(buf, cuid, cx, cy, cz);
        }
    }

    #[test]
    fn compound_zero_children() {
        // Compound with no child items and no child nodes — cursor must advance
        // cleanly without error, and sector.nodes must stay empty.
        let mut data = header(895);
        write_u32(&mut data, 1); // item_count
        append_compound(&mut data, 0xAB, 0xCD, &[]);
        write_u32(&mut data, 0); // trailing node_count

        let s = parse_sector(&data).unwrap();
        assert!(s.nodes.is_empty());
    }

    #[test]
    fn compound_extracts_two_child_nodes() {
        // Compound with 2 child nodes — both must appear in sector.nodes.
        let mut data = header(895);
        write_u32(&mut data, 1); // item_count
        append_compound(
            &mut data,
            0x01,
            0x00,
            &[
                (10, 256, 0, 0), // uid=10, x=1.0 m
                (20, 0, 512, 0), // uid=20, y=2.0 m
            ],
        );
        write_u32(&mut data, 0); // trailing node_count

        let s = parse_sector(&data).unwrap();
        assert_eq!(s.nodes.len(), 2);
        let n10 = s.nodes.iter().find(|n| n.uid == 10).expect("uid 10");
        let n20 = s.nodes.iter().find(|n| n.uid == 20).expect("uid 20");
        assert!((n10.x - 1.0).abs() < 1e-4, "x={}", n10.x);
        assert!((n20.y - 2.0).abs() < 1e-4, "y={}", n20.y);
    }

    #[test]
    fn compound_child_nodes_merged_with_trailing_nodes() {
        // Compound child nodes plus trailing section nodes must all appear.
        let mut data = header(895);
        write_u32(&mut data, 1); // item_count
        append_compound(
            &mut data,
            0x01,
            0x00,
            &[
                (100, 25600, 0, 0), // uid=100, x=100.0 m
            ],
        );
        write_u32(&mut data, 1); // trailing node_count
        append_node(&mut data, 200, 0, 25600, 0); // uid=200, y=100.0 m

        let s = parse_sector(&data).unwrap();
        assert_eq!(s.nodes.len(), 2);
        assert!(s.nodes.iter().any(|n| n.uid == 100));
        assert!(s.nodes.iter().any(|n| n.uid == 200));
    }

    #[test]
    fn compound_child_nodes_available_for_road_routing() {
        // A Road whose endpoint UIDs match compound child nodes should be
        // resolvable in the same sector (both items in same sector payload).
        let mut data = header(895);
        write_u32(&mut data, 2); // item_count: 1 compound + 1 road
        append_compound(
            &mut data,
            0x10,
            0x00,
            &[(1001, 0, 0, 0), (1002, 25600, 0, 0)],
        );
        append_road(&mut data, 0x42, 1001, 1002);
        write_u32(&mut data, 0); // no trailing nodes

        let s = parse_sector(&data).unwrap();
        // Both compound child nodes must be present
        assert_eq!(s.nodes.len(), 2);
        assert!(s.nodes.iter().any(|n| n.uid == 1001));
        assert!(s.nodes.iter().any(|n| n.uid == 1002));
        // Road must reference those node UIDs
        assert_eq!(s.roads.len(), 1);
        assert_eq!(s.roads[0].node_a, 1001);
        assert_eq!(s.roads[0].node_b, 1002);
    }

    #[test]
    fn compound_with_five_child_nodes() {
        let mut data = header(895);
        let child_nodes: Vec<(u64, i32, i32, i32)> =
            (1..=5).map(|i| (i as u64, i * 256, 0, 0)).collect();
        write_u32(&mut data, 1); // item_count
        append_compound(&mut data, 0xFF, 0x00, &child_nodes);
        write_u32(&mut data, 0);

        let s = parse_sector(&data).unwrap();
        assert_eq!(s.nodes.len(), 5);
        for i in 1u64..=5 {
            let node = s.nodes.iter().find(|n| n.uid == i).expect("uid");
            assert!((node.x - i as f32).abs() < 1e-4);
        }
    }
}
