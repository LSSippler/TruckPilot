//! Error types for the map-parser crate.

/// All errors that can occur during map parsing.
#[derive(Debug, thiserror::Error)]
pub enum ParseError {
    /// File I/O error.
    #[error("I/O error: {0}")]
    Io(String),

    /// Archive parsing / format error.
    #[error("Archive error: {0}")]
    Archive(String),

    /// Invalid SCS magic bytes.
    #[error("Invalid magic: expected SCS#, got {0:02X?}")]
    InvalidMagic([u8; 4]),

    /// Unsupported HashFS version.
    #[error("Unsupported version: {0} (expected 2)")]
    UnsupportedVersion(u16),

    /// Zlib decompression failed.
    #[error("Decompression error: {0}")]
    DecompressionError(String),

    /// Requested file not found in archive.
    #[error("Entry not found: {0}")]
    EntryNotFound(String),

    /// Binary sector parse error.
    #[error("Binary parse error: {0}")]
    Binary(String),

    /// Cache error.
    #[error("Cache error: {0}")]
    Cache(String),
}

impl From<std::io::Error> for ParseError {
    fn from(e: std::io::Error) -> Self {
        ParseError::Io(e.to_string())
    }
}
