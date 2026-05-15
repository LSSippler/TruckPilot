//! Road item parser — binrw structures and parse pipeline.
//!
//! Binary layout reverse-engineered from documented ETS2 sector-format
//! sources (TruckLib, ts-map). This implementation is original Rust;
//! no GPL code was copied.
//!
//! ## Overview
//!
//! Every Road item in an ETS2 sector consists of two back-to-back parts:
//!
//! 1. [`RoadFixedHeader`] — exactly 265 bytes (`0x109`). Carries kdop bounds,
//!    flags, the eleven primary token references, three railing pairs, two
//!    height offsets, the start/end node UIDs and the road length.
//! 2. [`RoadDataPayload`] — variable length. Holds the per-side model and
//!    vegetation lists, terrain quad mesh data, override lists and a few
//!    stray scalars (overlay token, center material, vegetation spheres,
//!    additional parts, edge looks).
//!
//! The two structs together describe a complete v907 (ETS2 1.50+) Road item
//! body — the leading `item_type u32` is consumed by the dispatcher in
//! [`crate::sector::parse_sector_legacy`] before this parser is invoked.
//!
//! Quote from Phase 5.6 brief: "Implement Phase 5.6 of the TruckPilot 2.0
//! ETS2 sector parser: a complete Road-item parser using binrw structs.
//! This replaces the current 273-byte fixed-size Road parser with a
//! struct-driven implementation that reads both the FixedHeader (265 bytes)
//! and the variable-length DataPayload that follows it."

use binrw::{binread, BinRead};

// ---------------------------------------------------------------------------
// Fixed header (265 bytes)
// ---------------------------------------------------------------------------

/// 10-component kdop bounding volume: five floor (`mins`) and five ceiling
/// (`maxs`) coefficients, each an `f32`.  Total 40 bytes.
#[derive(BinRead, Debug, Clone, Default)]
#[br(little)]
pub struct KdopBounds {
    /// Lower bounds along each of the five kdop axes.
    pub mins: [f32; 5],
    /// Upper bounds along each of the five kdop axes.
    pub maxs: [f32; 5],
}

/// Exactly one railing entry — both sides packed together (right_model,
/// right_offset, left_model, left_offset).  20 bytes.  Three of these
/// appear in every Road's fixed header.
#[derive(BinRead, Debug, Clone, Default)]
#[br(little)]
pub struct RoadRailingPair {
    /// Token identifying the right-side railing model.
    pub right_model: u64,
    /// Right-side offset, in 1/100 m units (engine-internal).
    pub right_offset: i16,
    /// Token identifying the left-side railing model.
    pub left_model: u64,
    /// Left-side offset, in 1/100 m units (engine-internal).
    pub left_offset: i16,
}

/// The 265-byte fixed header of every ETS2 Road item.
///
/// Field offsets are taken from the documented ETS2 sector layout (see the
/// TruckLib and ts-map specifications).  This struct mirrors the layout in
/// declaration order — binrw walks the reader byte-for-byte without
/// alignment padding.
#[derive(BinRead, Debug, Clone)]
#[br(little)]
pub struct RoadFixedHeader {
    /// Item UID — also the kdop UID.
    pub uid: u64,
    /// 10-component kdop bounds.
    pub kdop: KdopBounds,
    /// First kdop flag byte.
    pub kflag1: u8,
    /// Second kdop flag byte.
    pub kflag2: u8,
    /// Third kdop flag byte (carries the SECRET bit on sector versions ≥ 854).
    pub kflag3: u8,
    /// Fourth kdop flag byte (HIDDEN, HIGH_POLY, etc.).
    pub kflag4: u8,
    /// View distance in 10-metre units — multiply by 10 for the meter value.
    pub view_distance_div10: u8,
    /// First road flag byte (carries SUPERFINE).
    pub rflag1: u8,
    /// DLC guard index — 0 = no DLC required.
    pub dlc_guard: u8,
    /// Third road flag byte.
    pub rflag3: u8,
    /// Fourth road flag byte (carries GPS_AVOID).
    pub rflag4: u8,
    /// Token identifying the road type.
    pub road_type: u64,
    /// Right-lane traffic rule token.
    pub right_traffic_rule: u64,
    /// Left-lane traffic rule token.
    pub left_traffic_rule: u64,
    /// Right-lane geometry variant.
    pub right_variant: u64,
    /// Left-lane geometry variant.
    pub left_variant: u64,
    /// Right-side, right-edge token.
    pub right_right_edge: u64,
    /// Right-side, left-edge token.
    pub right_left_edge: u64,
    /// Left-side, right-edge token.
    pub left_right_edge: u64,
    /// Left-side, left-edge token.
    pub left_left_edge: u64,
    /// Right-side terrain profile token.
    pub right_terrain_profile: u64,
    /// Right-side terrain profile coefficient.
    pub right_terrain_coefficient: f32,
    /// Left-side terrain profile token.
    pub left_terrain_profile: u64,
    /// Left-side terrain profile coefficient.
    pub left_terrain_coefficient: f32,
    /// Right-look token (visual variant).
    pub right_look: u64,
    /// Left-look token (visual variant).
    pub left_look: u64,
    /// Material token applied to the road surface.
    pub material: u64,
    /// Three railing pairs (right + left for each of three slots).
    pub railings: [RoadRailingPair; 3],
    /// Right-side height offset (engine-internal units).
    pub right_height_offset: i32,
    /// Left-side height offset (engine-internal units).
    pub left_height_offset: i32,
    /// UID of the start (backward) node — at fixed offset `0xF5`.
    pub start_node_uid: u64,
    /// UID of the end (forward) node — at fixed offset `0xFD`.
    pub end_node_uid: u64,
    /// Road length in meters.
    pub length: f32,
}

impl RoadFixedHeader {
    /// `true` when the kflag4 bit-1 (HIDDEN) is set.
    ///
    /// TruckLib stores the inverted form (`ShowInUiMap`) — we expose the raw
    /// engine flag here.
    pub fn is_hidden(&self) -> bool {
        self.kflag4 & 0b0000_0010 != 0
    }

    /// `true` when the kflag3 bit-0 (SECRET) is set.  Only meaningful on
    /// sector versions ≥ 854; older sectors leave it clear.
    pub fn is_secret(&self) -> bool {
        self.kflag3 & 0b0000_0001 != 0
    }

    /// `true` when the rflag4 bit-4 (GPS_AVOID) is set — the routing layer
    /// uses this to penalise/avoid the segment.
    pub fn gps_avoid(&self) -> bool {
        self.rflag4 & 0b0001_0000 != 0
    }

    /// `true` when the kflag4 bit-0 (HIGH_POLY) is set.
    pub fn is_high_poly(&self) -> bool {
        self.kflag4 & 0b0000_0001 != 0
    }

    /// `true` when the rflag1 bit-1 (SUPERFINE) is set.
    pub fn is_superfine(&self) -> bool {
        self.rflag1 & 0b0000_0010 != 0
    }

    /// `true` when the kflag2 bit-7 (LEFT_HAND_TRAFFIC) is set.
    pub fn is_left_hand_traffic(&self) -> bool {
        self.kflag2 & 0b1000_0000 != 0
    }

    /// View distance in meters — the on-disk value is divided by ten.
    pub fn view_distance_meters(&self) -> u32 {
        (self.view_distance_div10 as u32).saturating_mul(10)
    }

    /// DLC-guard byte — 0 means no DLC ownership is required to use this
    /// segment.
    pub fn dlc_guard(&self) -> u8 {
        self.dlc_guard
    }
}

// ---------------------------------------------------------------------------
// Variable payload
// ---------------------------------------------------------------------------

/// One of the two road models in each side's `models` slot — name token, an
/// integer offset and a draw distance.  12 bytes.
#[derive(BinRead, Debug, Clone, Default)]
#[br(little)]
pub struct RoadModel {
    /// Token identifying the model name.
    pub name: u64,
    /// Lateral offset, in engine units.
    pub offset: i16,
    /// Draw distance — where the model fades out, in engine units.
    pub distance: u16,
}

/// One vegetation entry.  16 bytes.
#[derive(BinRead, Debug, Clone, Default)]
#[br(little)]
pub struct RoadVegetation {
    /// Token identifying the vegetation set.
    pub name: u64,
    /// Density in arbitrary engine units.
    pub density: u16,
    /// Distance threshold beyond which only low-poly variants are drawn.
    pub high_poly_distance: u8,
    /// Vegetation scale.
    pub scale: u8,
    /// Range start, measured along the road in engine units.
    pub from: u16,
    /// Range end, measured along the road in engine units.
    pub to: u16,
}

/// Brush-material entry inside [`TerrainQuadData`].  10 bytes.
#[derive(BinRead, Debug, Clone, Default)]
#[br(little)]
pub struct BrushMaterial {
    /// Material name token.
    pub name: u64,
    /// In-engine rotation index.
    pub rotation: u16,
}

/// Per-vertex terrain attribute (offset or normal).  16 bytes.
///
/// The Phase 5.6 spec wrote "10 bytes" but the actual byte budget is
/// `2 + 2 + 12 = 16` — we trust the math.
#[derive(BinRead, Debug, Clone, Default)]
#[br(little)]
pub struct VertexData {
    /// Quad column index.
    pub x: u16,
    /// Quad row index.
    pub y: u16,
    /// 3-component vector (offset or normal).
    pub vec: [f32; 3],
}

/// Per-side terrain quad mesh — counts followed by raw arrays.
#[binread]
#[derive(Debug, Clone, Default)]
#[br(little)]
pub struct TerrainQuadData {
    #[br(temp)]
    brush_material_count: u16,
    /// Brush materials.
    #[br(count = brush_material_count)]
    pub brush_materials: Vec<BrushMaterial>,

    #[br(temp)]
    color_count: u16,
    /// Per-vertex color samples (RGBA 32-bit).
    #[br(count = color_count)]
    pub colors: Vec<[u8; 4]>,

    /// Mesh row count.
    pub rows: u16,
    /// Mesh column count.
    pub cols: u16,

    #[br(temp)]
    terrain_quad_count: u32,
    /// Per-quad RGBA color (stored as 4-byte tuples).
    #[br(count = terrain_quad_count)]
    pub quads: Vec<[u8; 4]>,

    #[br(temp)]
    offset_count: u32,
    /// Per-vertex offset displacements.
    #[br(count = offset_count)]
    pub offsets: Vec<VertexData>,

    #[br(temp)]
    normal_count: u32,
    /// Per-vertex normals.
    #[br(count = normal_count)]
    pub normals: Vec<VertexData>,
}

/// One side (left or right) of the variable Road payload.
#[derive(BinRead, Debug, Clone, Default)]
#[br(little)]
pub struct RoadPayloadSide {
    /// Up to two road-side models.
    pub models: [RoadModel; 2],
    /// Footprint of the terrain mesh on this side.
    pub terrain_size: u16,
    /// Three vegetation slots.
    pub vegetation: [RoadVegetation; 3],
    /// Sidewalk material token.
    pub sidewalk_material: u64,
    /// Per-side terrain mesh data.
    pub terrain_quad_data: TerrainQuadData,
}

/// Center-strip vegetation block — 12 bytes.
#[derive(BinRead, Debug, Clone, Default)]
#[br(little)]
pub struct CenterVegetation {
    /// Token identifying the vegetation set.
    pub name: u64,
    /// Density in arbitrary engine units.
    pub density: u16,
    /// Vegetation scale.
    pub scale: u8,
    /// Lateral offset.
    pub offset: u8,
}

/// One sphere of artistic vegetation override.  20 bytes.
#[derive(BinRead, Debug, Clone, Default)]
#[br(little)]
pub struct VegetationSphere {
    /// Sphere center.
    pub position: [f32; 3],
    /// Sphere radius.
    pub radius: f32,
    /// Sphere kind (engine-internal enum).
    pub kind: u32,
}

/// One per-side edge override entry.  20 bytes.
#[derive(BinRead, Debug, Clone, Default)]
#[br(little)]
pub struct EdgeOverride {
    /// Override start, measured along the road in engine units.
    pub offset: u16,
    /// Override length, in engine units.
    pub length: u16,
    /// Edge token applied within the range.
    pub edge: u64,
    /// Edge-look token applied within the range.
    pub look: u64,
}

/// One per-side variant override entry.  12 bytes.
#[derive(BinRead, Debug, Clone, Default)]
#[br(little)]
pub struct VariantOverride {
    /// Override start, measured along the road in engine units.
    pub offset: u16,
    /// Override length, in engine units.
    pub length: u16,
    /// Variant token applied within the range.
    pub variant: u64,
}

/// Variable-length payload that follows every [`RoadFixedHeader`].
///
/// Field order matches the on-disk layout: overlay token, then the right
/// side, then the left side, then the trailing center-material / center-veg
/// / random-seed / preceding-length scalars, then no-detail vegetation
/// ranges, vegetation spheres, additional parts (right + left), edge
/// overrides (left + right) and variant overrides (left + right), then four
/// trailing edge-look tokens.
#[binread]
#[derive(Debug, Clone, Default)]
#[br(little)]
pub struct RoadDataPayload {
    /// Overlay token applied across the segment.
    pub overlay_token: u64,

    /// Right side of the road.
    pub right: RoadPayloadSide,
    /// Left side of the road.
    pub left: RoadPayloadSide,

    /// Center-strip material token.
    pub center_material: u64,
    /// Center-strip material color (RGBA).
    pub center_material_color: [u8; 4],
    /// Center-strip material rotation index.
    pub center_material_rotation: u16,
    /// Random seed used to vary repeating textures.
    pub random_seed: u32,
    /// Length of the previous segment (used for visual continuity).
    pub previous_length: f32,

    /// Center-strip vegetation.
    pub center_vegetation: CenterVegetation,

    /// Left side: lower bound of the no-detail-vegetation band.
    pub left_no_detail_vegetation_from: u16,
    /// Right side: lower bound of the no-detail-vegetation band.
    pub right_no_detail_vegetation_from: u16,
    /// Left side: upper bound of the no-detail-vegetation band.
    pub left_no_detail_vegetation_to: u16,
    /// Right side: upper bound of the no-detail-vegetation band.
    pub right_no_detail_vegetation_to: u16,

    #[br(temp)]
    vegetation_sphere_count: u32,
    /// Vegetation spheres painted onto the segment.
    #[br(count = vegetation_sphere_count)]
    pub vegetation_spheres: Vec<VegetationSphere>,

    #[br(temp)]
    left_additional_parts_count: u32,
    /// Left-side additional parts (model UIDs).
    #[br(count = left_additional_parts_count)]
    pub left_additional_parts: Vec<u64>,

    #[br(temp)]
    right_additional_parts_count: u32,
    /// Right-side additional parts (model UIDs).
    #[br(count = right_additional_parts_count)]
    pub right_additional_parts: Vec<u64>,

    #[br(temp)]
    left_edge_override_count: u32,
    /// Left-side edge overrides.
    #[br(count = left_edge_override_count)]
    pub left_edge_overrides: Vec<EdgeOverride>,

    #[br(temp)]
    right_edge_override_count: u32,
    /// Right-side edge overrides.
    #[br(count = right_edge_override_count)]
    pub right_edge_overrides: Vec<EdgeOverride>,

    #[br(temp)]
    left_variant_override_count: u32,
    /// Left-side variant overrides.
    #[br(count = left_variant_override_count)]
    pub left_variant_overrides: Vec<VariantOverride>,

    #[br(temp)]
    right_variant_override_count: u32,
    /// Right-side variant overrides.
    #[br(count = right_variant_override_count)]
    pub right_variant_overrides: Vec<VariantOverride>,

    /// Right-side, right-edge look token (trailing).
    pub right_right_edge_look: u64,
    /// Right-side, left-edge look token (trailing).
    pub right_left_edge_look: u64,
    /// Left-side, right-edge look token (trailing).
    pub left_right_edge_look: u64,
    /// Left-side, left-edge look token (trailing).
    pub left_left_edge_look: u64,
}

/// Convenience aggregate — header followed by payload.
///
/// `parse_sector_legacy` reads them sequentially via binrw and copies the
/// fields it cares about into [`crate::sector::RawRoad`].  This struct
/// exists for tests and downstream code that wants the full layout.
#[derive(BinRead, Debug, Clone)]
#[br(little)]
pub struct ParsedRoad {
    /// 265-byte fixed header.
    pub header: RoadFixedHeader,
    /// Variable-length payload.
    pub payload: RoadDataPayload,
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use binrw::io::Cursor;

    /// Build a 265-byte buffer with the headline fields set; everything else
    /// stays zero.
    #[allow(clippy::too_many_arguments)]
    fn fixed_header_bytes(
        uid: u64,
        kflag2: u8,
        kflag3: u8,
        kflag4: u8,
        rflag1: u8,
        rflag4: u8,
        view_distance_div10: u8,
        dlc_guard: u8,
        start_node: u64,
        end_node: u64,
        length: f32,
    ) -> Vec<u8> {
        let mut buf = vec![0u8; 0x109];
        buf[0..8].copy_from_slice(&uid.to_le_bytes());
        // KdopBounds spans 0x08..0x30 (40 bytes), already zero.
        // Flag bytes:
        // 0x30 kflag1, 0x31 kflag2, 0x32 kflag3, 0x33 kflag4
        buf[0x31] = kflag2;
        buf[0x32] = kflag3;
        buf[0x33] = kflag4;
        // 0x34 view_distance_div10
        buf[0x34] = view_distance_div10;
        // 0x35 rflag1, 0x36 dlc_guard, 0x37 rflag3, 0x38 rflag4
        buf[0x35] = rflag1;
        buf[0x36] = dlc_guard;
        buf[0x38] = rflag4;
        // start_node_uid at 0xF5, end_node_uid at 0xFD
        buf[0xF5..0xF5 + 8].copy_from_slice(&start_node.to_le_bytes());
        buf[0xFD..0xFD + 8].copy_from_slice(&end_node.to_le_bytes());
        // length at 0x105
        buf[0x105..0x105 + 4].copy_from_slice(&length.to_le_bytes());
        buf
    }

    #[test]
    fn parses_minimal_fixed_header() {
        let bytes = fixed_header_bytes(
            0xCAFE_BABE_DEAD_BEEF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0xAA,
            0xBB,
            0.0,
        );
        assert_eq!(bytes.len(), 0x109, "fixed header must be exactly 265 bytes");

        let mut cur = Cursor::new(&bytes);
        let h = RoadFixedHeader::read(&mut cur).expect("parse header");
        assert_eq!(h.uid, 0xCAFE_BABE_DEAD_BEEF);
        assert_eq!(h.start_node_uid, 0xAA);
        assert_eq!(h.end_node_uid, 0xBB);
        assert_eq!(h.length, 0.0);
        assert_eq!(h.dlc_guard, 0);
        assert!(!h.is_hidden());
        assert!(!h.is_secret());
        assert!(!h.gps_avoid());
        assert!(!h.is_high_poly());
        assert!(!h.is_superfine());
        assert!(!h.is_left_hand_traffic());
        assert_eq!(h.view_distance_meters(), 0);
        // Cursor must have advanced exactly 265 bytes.
        assert_eq!(cur.position(), 0x109);
    }

    #[test]
    fn flag_helpers_decode_correctly() {
        // is_hidden — kflag4 bit 1
        let bytes = fixed_header_bytes(1, 0, 0, 0b0000_0010, 0, 0, 0, 0, 0, 0, 0.0);
        let h = RoadFixedHeader::read(&mut Cursor::new(&bytes)).unwrap();
        assert!(h.is_hidden());
        assert!(!h.is_high_poly());

        // is_secret — kflag3 bit 0
        let bytes = fixed_header_bytes(2, 0, 0b0000_0001, 0, 0, 0, 0, 0, 0, 0, 0.0);
        let h = RoadFixedHeader::read(&mut Cursor::new(&bytes)).unwrap();
        assert!(h.is_secret());

        // gps_avoid — rflag4 bit 4
        let bytes = fixed_header_bytes(3, 0, 0, 0, 0, 0b0001_0000, 0, 0, 0, 0, 0.0);
        let h = RoadFixedHeader::read(&mut Cursor::new(&bytes)).unwrap();
        assert!(h.gps_avoid());

        // is_high_poly — kflag4 bit 0
        let bytes = fixed_header_bytes(4, 0, 0, 0b0000_0001, 0, 0, 0, 0, 0, 0, 0.0);
        let h = RoadFixedHeader::read(&mut Cursor::new(&bytes)).unwrap();
        assert!(h.is_high_poly());

        // is_superfine — rflag1 bit 1
        let bytes = fixed_header_bytes(5, 0, 0, 0, 0b0000_0010, 0, 0, 0, 0, 0, 0.0);
        let h = RoadFixedHeader::read(&mut Cursor::new(&bytes)).unwrap();
        assert!(h.is_superfine());

        // is_left_hand_traffic — kflag2 bit 7
        let bytes = fixed_header_bytes(6, 0b1000_0000, 0, 0, 0, 0, 0, 0, 0, 0, 0.0);
        let h = RoadFixedHeader::read(&mut Cursor::new(&bytes)).unwrap();
        assert!(h.is_left_hand_traffic());

        // view_distance_meters — div10 = 25 → 250 m
        let bytes = fixed_header_bytes(7, 0, 0, 0, 0, 0, 25, 0, 0, 0, 0.0);
        let h = RoadFixedHeader::read(&mut Cursor::new(&bytes)).unwrap();
        assert_eq!(h.view_distance_meters(), 250);

        // dlc_guard
        let bytes = fixed_header_bytes(8, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0.0);
        let h = RoadFixedHeader::read(&mut Cursor::new(&bytes)).unwrap();
        assert_eq!(h.dlc_guard(), 7);
    }

    /// Build a minimal payload buffer: every list zero-length, every scalar
    /// zero.  Total size is the sum of all fixed-size fields plus eight
    /// `u32` count prefixes (each = 0).
    fn minimal_payload_bytes() -> Vec<u8> {
        let mut buf = Vec::new();

        // overlay_token
        buf.extend_from_slice(&0u64.to_le_bytes());

        // Right side, then left side. Each side:
        //   models: [RoadModel; 2]               2 * 12 = 24 B
        //   terrain_size: u16                    2 B
        //   vegetation: [RoadVegetation; 3]      3 * 16 = 48 B
        //   sidewalk_material: u64               8 B
        //   terrain_quad_data:
        //     brush_material_count u16 = 0       2 B
        //     color_count u16 = 0                2 B
        //     rows u16, cols u16                 4 B
        //     terrain_quad_count u32 = 0         4 B
        //     offset_count u32 = 0               4 B
        //     normal_count u32 = 0               4 B
        // = 24 + 2 + 48 + 8 + 2 + 2 + 4 + 4 + 4 + 4 = 102 B per side
        let side_zeros = vec![0u8; 102];
        buf.extend_from_slice(&side_zeros);
        buf.extend_from_slice(&side_zeros);

        // center_material u64 + color [u8;4] + rotation u16 + random_seed u32 + previous_length f32
        buf.extend_from_slice(&0u64.to_le_bytes());
        buf.extend_from_slice(&[0u8; 4]);
        buf.extend_from_slice(&0u16.to_le_bytes());
        buf.extend_from_slice(&0u32.to_le_bytes());
        buf.extend_from_slice(&0f32.to_le_bytes());

        // CenterVegetation: u64 + u16 + u8 + u8 = 12 B
        buf.extend_from_slice(&[0u8; 12]);

        // four no_detail_vegetation u16
        buf.extend_from_slice(&[0u8; 8]);

        // vegetation_sphere_count u32 = 0
        buf.extend_from_slice(&0u32.to_le_bytes());
        // 6 list-count u32 = 0
        for _ in 0..6 {
            buf.extend_from_slice(&0u32.to_le_bytes());
        }

        // four trailing edge_look u64
        for _ in 0..4 {
            buf.extend_from_slice(&0u64.to_le_bytes());
        }

        buf
    }

    #[test]
    fn parses_minimal_payload() {
        let bytes = minimal_payload_bytes();
        let total = bytes.len();
        let mut cur = Cursor::new(&bytes);
        let p = RoadDataPayload::read(&mut cur).expect("parse minimal payload");

        assert_eq!(p.overlay_token, 0);
        assert!(p.right.terrain_quad_data.brush_materials.is_empty());
        assert!(p.right.terrain_quad_data.colors.is_empty());
        assert!(p.right.terrain_quad_data.quads.is_empty());
        assert!(p.right.terrain_quad_data.offsets.is_empty());
        assert!(p.right.terrain_quad_data.normals.is_empty());
        assert!(p.left.terrain_quad_data.brush_materials.is_empty());
        assert_eq!(p.center_material, 0);
        assert!(p.vegetation_spheres.is_empty());
        assert!(p.left_additional_parts.is_empty());
        assert!(p.right_additional_parts.is_empty());
        assert!(p.left_edge_overrides.is_empty());
        assert!(p.right_edge_overrides.is_empty());
        assert!(p.left_variant_overrides.is_empty());
        assert!(p.right_variant_overrides.is_empty());
        assert_eq!(p.right_right_edge_look, 0);
        assert_eq!(p.left_left_edge_look, 0);

        // Cursor must have consumed every byte of the buffer — that's the
        // strongest guarantee we walked the layout correctly.
        assert_eq!(cur.position() as usize, total);
    }

    #[test]
    fn parses_terrain_quad_data_with_one_of_each() {
        let mut buf = Vec::new();

        // brush_material_count = 1, then 1 BrushMaterial (10 B)
        buf.extend_from_slice(&1u16.to_le_bytes());
        buf.extend_from_slice(&0xDEAD_BEEFu64.to_le_bytes()); // name token
        buf.extend_from_slice(&7u16.to_le_bytes()); // rotation

        // color_count = 1, then 1 RGBA color (4 B)
        buf.extend_from_slice(&1u16.to_le_bytes());
        buf.extend_from_slice(&[0x11, 0x22, 0x33, 0x44]);

        // rows = 1, cols = 1
        buf.extend_from_slice(&1u16.to_le_bytes());
        buf.extend_from_slice(&1u16.to_le_bytes());

        // terrain_quad_count = 1, then 1 RGBA color
        buf.extend_from_slice(&1u32.to_le_bytes());
        buf.extend_from_slice(&[0x55, 0x66, 0x77, 0x88]);

        // offset_count = 1, then 1 VertexData (16 B)
        buf.extend_from_slice(&1u32.to_le_bytes());
        buf.extend_from_slice(&3u16.to_le_bytes()); // x
        buf.extend_from_slice(&5u16.to_le_bytes()); // y
        buf.extend_from_slice(&1.0f32.to_le_bytes());
        buf.extend_from_slice(&2.0f32.to_le_bytes());
        buf.extend_from_slice(&3.0f32.to_le_bytes());

        // normal_count = 1, then 1 VertexData
        buf.extend_from_slice(&1u32.to_le_bytes());
        buf.extend_from_slice(&7u16.to_le_bytes()); // x
        buf.extend_from_slice(&11u16.to_le_bytes()); // y
        buf.extend_from_slice(&0.0f32.to_le_bytes());
        buf.extend_from_slice(&1.0f32.to_le_bytes());
        buf.extend_from_slice(&0.0f32.to_le_bytes());

        let total = buf.len();
        let mut cur = Cursor::new(&buf);
        let q = TerrainQuadData::read(&mut cur).expect("parse terrain quad data");

        assert_eq!(q.brush_materials.len(), 1);
        assert_eq!(q.brush_materials[0].name, 0xDEAD_BEEF);
        assert_eq!(q.brush_materials[0].rotation, 7);
        assert_eq!(q.colors, vec![[0x11, 0x22, 0x33, 0x44]]);
        assert_eq!(q.rows, 1);
        assert_eq!(q.cols, 1);
        assert_eq!(q.quads, vec![[0x55, 0x66, 0x77, 0x88]]);
        assert_eq!(q.offsets.len(), 1);
        assert_eq!(q.offsets[0].x, 3);
        assert_eq!(q.offsets[0].y, 5);
        assert_eq!(q.offsets[0].vec, [1.0, 2.0, 3.0]);
        assert_eq!(q.normals.len(), 1);
        assert_eq!(q.normals[0].x, 7);
        assert_eq!(q.normals[0].y, 11);

        // Cursor must reach exactly the end of the input.
        assert_eq!(cur.position() as usize, total);
    }

    #[test]
    fn truncated_data_returns_err_not_panic() {
        // 100 bytes is less than the 265-byte fixed header — must Err, not panic.
        let bytes = vec![0u8; 100];
        let mut cur = Cursor::new(&bytes);
        let result = RoadFixedHeader::read(&mut cur);
        assert!(result.is_err(), "expected Err on truncated header");
    }
}
