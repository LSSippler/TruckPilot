//! ZIP archive reader for ETS2 mod archives.
//!
//! Many ETS2 mods are distributed as `.zip` files (even with a `.scs`
//! extension). This module provides an `ArchiveReader` implementation that
//! wraps the `zip` crate.
//!
//! # Memory strategy
//!
//! Large mod archives (1 GB+) cannot be loaded entirely into RAM. We
//! therefore keep the file open and read entries on demand. The entry
//! *index* (name → position in the central directory) is built once at
//! open time and kept in memory; actual file data is decompressed lazily.

use std::collections::HashMap;
use std::fs::File;
use std::io::Read;
use std::path::Path;

use crate::ets2_parser::archive::ArchiveReader;
use crate::ets2_parser::error::{Error, Result};

/// A ZIP archive opened for lazy reading.
pub struct ZipArchive {
    path: std::path::PathBuf,
    /// Maps logical path → index in the ZIP central directory.
    index: HashMap<String, usize>,
}

impl ZipArchive {
    /// Open a ZIP file and build the name→index map.
    ///
    /// Only the central directory is read here; file data is decompressed
    /// on demand in [`read_file`](Self::read_file).
    pub fn open(path: &Path) -> Result<Self> {
        let file = File::open(path).map_err(Error::Io)?;
        let mut archive = zip::ZipArchive::new(file).map_err(Error::Zip)?;

        let mut index = HashMap::with_capacity(archive.len());
        for i in 0..archive.len() {
            // `by_index_raw` gives us the name without decompressing.
            if let Ok(entry) = archive.by_index_raw(i) {
                index.insert(entry.name().to_string(), i);
            }
        }

        Ok(ZipArchive {
            path: path.to_path_buf(),
            index,
        })
    }
}

impl ArchiveReader for ZipArchive {
    fn read_file(&mut self, logical_path: &str) -> Result<Vec<u8>> {
        let idx = *self
            .index
            .get(logical_path)
            .ok_or_else(|| Error::FileNotFound(logical_path.into()))?;

        // Re-open the file for each read. This is slower than keeping a
        // persistent handle but avoids the borrow-checker complexity of
        // holding a mutable `ZipArchive<File>` alongside the index.
        let file = File::open(&self.path).map_err(Error::Io)?;
        let mut archive = zip::ZipArchive::new(file).map_err(Error::Zip)?;
        let mut entry = archive.by_index(idx).map_err(Error::Zip)?;

        let mut data = Vec::with_capacity(entry.size() as usize);
        entry.read_to_end(&mut data).map_err(Error::Io)?;
        Ok(data)
    }

    fn find_files_starting_with(&self, prefix: &str) -> Vec<String> {
        let mut result: Vec<String> = self
            .index
            .keys()
            .filter(|p| p.starts_with(prefix))
            .cloned()
            .collect();
        result.sort();
        result
    }

    fn entry_count(&self) -> usize {
        self.index.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    fn make_test_zip(path: &std::path::Path, files: &[(&str, &[u8])]) {
        let file = std::fs::File::create(path).unwrap();
        let mut zip = zip::write::ZipWriter::new(file);
        let options = zip::write::SimpleFileOptions::default()
            .compression_method(zip::CompressionMethod::Stored);
        for (name, content) in files {
            zip.start_file(*name, options).unwrap();
            zip.write_all(content).unwrap();
        }
        zip.finish().unwrap();
    }

    #[test]
    fn test_read_known_file() {
        let path = std::env::temp_dir().join("truckpilot_zipreader_test.zip");
        make_test_zip(&path, &[("test/file.txt", b"zip content")]);

        let mut archive = ZipArchive::open(&path).unwrap();
        assert_eq!(archive.entry_count(), 1);

        let data = archive.read_file("test/file.txt").unwrap();
        assert_eq!(data, b"zip content");

        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn test_read_missing_file() {
        let path = std::env::temp_dir().join("truckpilot_zipreader_test2.zip");
        make_test_zip(&path, &[("other.txt", b"")]);

        let mut archive = ZipArchive::open(&path).unwrap();
        let err = archive.read_file("missing.txt").unwrap_err();
        assert!(format!("{}", err).contains("file not found"));

        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn test_find_files_starting_with() {
        let path = std::env::temp_dir().join("truckpilot_zipreader_test3.zip");
        make_test_zip(
            &path,
            &[
                ("pref/a.txt", b"1"),
                ("pref/b.txt", b"2"),
                ("other.txt", b"3"),
            ],
        );

        let archive = ZipArchive::open(&path).unwrap();
        let found = archive.find_files_starting_with("pref/");
        assert_eq!(found, vec!["pref/a.txt", "pref/b.txt"]);

        std::fs::remove_file(&path).unwrap();
    }
}
