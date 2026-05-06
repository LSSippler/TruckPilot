//! Central error types for the ETS2 parser.
//!
//! Uses `thiserror` for ergonomic error definitions that implement
//! `Display`, `std::error::Error`, and `From` conversions.

use thiserror::Error;

/// Unified error type for all parser operations.
#[derive(Error, Debug)]
pub enum Error {
    /// Generic I/O error.
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// Invalid or unsupported archive format.
    #[error("archive format error: {0}")]
    ArchiveFormat(String),

    /// File not found in archive.
    #[error("file not found: {0}")]
    FileNotFound(String),

    /// Decompression failure (zlib / deflate).
    #[error("decompression error: {0}")]
    Decompression(String),

    /// UTF-8 decoding error.
    #[error("UTF-8 error: {0}")]
    Utf8(#[from] std::str::Utf8Error),

    /// SII parse error.
    #[error("SII parse error: {0}")]
    SiiParse(String),

    /// Map parse error (text or binary).
    #[error("map parse error: {0}")]
    MapParse(String),

    /// ZIP archive error.
    #[error("ZIP error: {0}")]
    Zip(#[from] zip::result::ZipError),

    /// JSON serialization error.
    #[error("JSON error: {0}")]
    Json(#[from] serde_json::Error),

    /// Generic string error (catch-all for legacy code).
    #[error("{0}")]
    Generic(String),
}

impl From<String> for Error {
    fn from(s: String) -> Self {
        Error::Generic(s)
    }
}

impl From<&str> for Error {
    fn from(s: &str) -> Self {
        Error::Generic(s.to_string())
    }
}

/// Convenience alias.
pub type Result<T> = std::result::Result<T, Error>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_file_not_found_display() {
        let err = Error::FileNotFound("foo.sii".into());
        assert_eq!(format!("{}", err), "file not found: foo.sii");
    }

    #[test]
    fn test_error_propagation_from_io() {
        fn inner() -> Result<()> {
            Err(std::io::Error::new(std::io::ErrorKind::NotFound, "nope").into())
        }
        fn outer() -> Result<()> {
            inner()?;
            Ok(())
        }
        let err = outer().unwrap_err();
        assert!(format!("{}", err).contains("nope"));
    }

    #[test]
    fn test_error_propagation_from_str() {
        fn inner() -> Result<()> {
            Err("something went wrong".into())
        }
        fn outer() -> Result<()> {
            inner()?;
            Ok(())
        }
        let err = outer().unwrap_err();
        assert_eq!(format!("{}", err), "something went wrong");
    }
}
