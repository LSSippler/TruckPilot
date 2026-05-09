// src/zip_archive.rs

//! Implements the `Archive` trait for standard ZIP files.

use std::collections::HashSet;
use std::fs::File;
use std::io::Read;
use std::path::{Path, PathBuf};

use zip::ZipArchive as ZipReader;

use crate::archive::{archive_identity_hash, Archive};
use crate::error::ParseError;

/// An opened `.scs` archive that is a standard ZIP file.
pub struct ZipArchive {
    path: PathBuf,
    file_hash: [u8; 32],
    reader: ZipReader<File>,
    /// Cache of file paths for faster lookups.
    files: HashSet<String>,
}

impl ZipArchive {
    pub fn open(path: impl AsRef<Path>) -> Result<Self, ParseError> {
        let path = path.as_ref().to_path_buf();
        let file = File::open(&path)?;

        let file_hash = archive_identity_hash(&path)?;

        let reader = ZipReader::new(file)?;
        let files = reader.file_names().map(String::from).collect();

        Ok(Self {
            path,
            file_hash,
            reader,
            files,
        })
    }
}

impl Archive for ZipArchive {
    fn read_path(&mut self, path: &str) -> Result<Vec<u8>, ParseError> {
        let mut file = self.reader.by_name(path)?;
        let mut data = Vec::with_capacity(file.size() as usize);
        file.read_to_end(&mut data)?;
        Ok(data)
    }

    fn contains(&self, path: &str) -> bool {
        self.files.contains(path)
    }

    fn list_files(&self) -> Vec<String> {
        self.files.iter().cloned().collect()
    }

    fn file_hash(&self) -> [u8; 32] {
        self.file_hash
    }

    fn path(&self) -> &Path {
        &self.path
    }

    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
}

impl From<zip::result::ZipError> for ParseError {
    fn from(e: zip::result::ZipError) -> Self {
        ParseError::Archive(format!("ZIP error: {e}"))
    }
}
