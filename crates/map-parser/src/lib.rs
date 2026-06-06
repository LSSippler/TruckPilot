//! TruckPilot Map Parser — Phase 5
//!
//! Parses ETS2 `.scs` archives into a routing graph with traffic signs.

#![allow(missing_docs)]

pub mod arc_length;
pub mod archive;
pub mod cache;
pub mod city_sii;
pub mod cityhash;
pub mod drop_tracer;
pub mod error;
pub mod graph;
pub mod hashfs;
pub mod mod_loader;
pub mod ppd;
pub mod prefab_sii;
pub mod road_full;
pub mod road_look;
pub mod sector;
pub mod signs;
pub mod spatial_match;
pub mod spline;
pub mod spline_index;
pub mod zip_archive;

pub use archive::Archive;
pub use city_sii::{
    build_display_name_index, load_city_sii, normalize_name, parse_city_sii, CityEntry,
};
pub use drop_tracer::{DropCategory, DropEvent, DropTracer};
pub use error::ParseError;
pub use graph::RoadAuditResult;
pub use graph::{GraphBuilder, GraphEdge, GraphNode, MapGraph};
pub use hashfs::{parse_directory_listing, DirItem, HashFsArchive};
pub use mod_loader::{load_and_build, parse_sectors_with_drop_tracer, ArchiveFile, ModLoadOrder};
pub use spline::{build_splines_ex, SegmentMetadata};
pub use spline_index::{build_index_with_metadata, SplineIndex};
pub use zip_archive::ZipArchive;
