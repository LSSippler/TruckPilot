// src/archive.rs

//! Defines a common interface for reading files from different archive types.

use crate::error::ParseError;
use sha2::{Digest, Sha256};
use std::any::Any;
use std::path::Path;
use std::time::SystemTime;

/// A unified interface for `.scs` archives, which can be either HashFS
/// or standard ZIP files.
pub trait Archive {
    /// Read a file by its archive path.
    fn read_path(&mut self, path: &str) -> Result<Vec<u8>, ParseError>;

    /// Check if a file exists in the archive.
    fn contains(&self, path: &str) -> bool;

    /// Return a list of all file paths in the archive.
    fn list_files(&self) -> Vec<String>;

    /// Return a stable identity hash of the archive file. Used **only**
    /// as input to `cache::compute_cache_key`. See [`archive_identity_hash`]
    /// for the derivation.
    fn file_hash(&self) -> [u8; 32];

    /// Return the on-disk path of the archive file.
    fn path(&self) -> &Path;

    /// Returns this archive as `Any` so that it can be downcast.
    fn as_any(&self) -> &dyn Any;
}

/// Compute a stable 32-byte identity hash of an archive file from its
/// **path + modified-time + size**, _not_ its content.
///
/// Why not SHA-256 over the file content?
///   On a 9.4 GB `base.scs` the content SHA-256 took ~15 s in release
///   mode. Profiling showed it dominated `HashFsArchive::open` at
///   96.7 % of total time. The hash is only used to derive a Disk-cache
///   key (`cache::compute_cache_key`), and `(path, mtime, size)` is a
///   strictly weaker collision domain — but for cache invalidation it
///   is sufficient: any meaningful edit to the archive bumps mtime, and
///   touching mtime alone (e.g. `touch base.scs` after a content swap
///   that preserves length) is rare enough that the worst case is one
///   stale cache entry that gets rebuilt next time the user explicitly
///   clears the cache.
///
/// The hash is namespaced (`"TruckPilot/archive-id/v1"` prefix) so that
/// future schema changes can rotate keys without colliding.
pub fn archive_identity_hash(path: &Path) -> Result<[u8; 32], ParseError> {
    let metadata = std::fs::metadata(path).map_err(|e| {
        ParseError::Io(format!("metadata {:?}: {e}", path))
    })?;
    let size = metadata.len();
    let mtime_nanos = metadata
        .modified()
        .ok()
        .and_then(|t| t.duration_since(SystemTime::UNIX_EPOCH).ok())
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0);

    let path_str = path.to_string_lossy();
    let mut hasher = Sha256::new();
    hasher.update(b"TruckPilot/archive-id/v1\0");
    hasher.update(path_str.as_bytes());
    hasher.update(b"\0");
    hasher.update(mtime_nanos.to_le_bytes());
    hasher.update(size.to_le_bytes());
    Ok(hasher.finalize().into())
}
