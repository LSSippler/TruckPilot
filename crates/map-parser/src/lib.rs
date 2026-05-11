//! TruckPilot Map Parser — Phase 5
//!
//! Parses ETS2 `.scs` archives into a routing graph with traffic signs.

#![allow(missing_docs)]

pub mod archive;
pub mod cache;
pub mod cityhash;
pub mod error;
pub mod graph;
pub mod hashfs;
pub mod mod_loader;
pub mod road_full;
pub mod sector;
pub mod signs;
pub mod spatial_match;
pub mod zip_archive;

pub use archive::Archive;
pub use error::ParseError;
pub use graph::{GraphBuilder, GraphEdge, GraphNode, MapGraph};
pub use hashfs::{parse_directory_listing, DirItem, HashFsArchive};
pub use mod_loader::{load_and_build, ArchiveFile, ModLoadOrder};
pub use zip_archive::ZipArchive;
