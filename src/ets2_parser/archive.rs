//! Archive reader abstraction.
//!
//! Defines the `ArchiveReader` trait implemented by all archive backends
//! (HashFS v1/v2, ZIP). `Archive` is the unified entry point that
//! auto-detects the format.

use std::path::Path;

use crate::ets2_parser::error::Result;

/// Trait implemented by all archive format backends.
pub trait ArchiveReader {
    /// Read a file by its logical path within the archive.
    /// The path is hashed internally for HashFS archives.
    fn read_file(&mut self, logical_path: &str) -> Result<Vec<u8>>;

    /// Find all file paths starting with the given prefix.
    fn find_files_starting_with(&self, prefix: &str) -> Vec<String>;

    /// Return the number of entries in the archive.
    fn entry_count(&self) -> usize;

    /// Read a file and, if it is a combined texture object (v2 .tobj),
    /// split the raw data into (metadata, texture_data).
    ///
    /// Detects the 0x10 flag in the HashFS directory entry that marks
    /// combined tobj entries.
    fn read_file_with_texture(&mut self, logical_path: &str) -> Result<(Vec<u8>, Option<Vec<u8>>)> {
        let data = self.read_file(logical_path)?;
        Ok((data, None))
    }
}

/// Unified archive type — auto-detects format on open.
pub enum Archive {
    /// Native SCS archive (CityHash64 directory).
    Scs(Box<dyn ArchiveReader>),
    /// Plain ZIP archive (mods, side-loaded data).
    Zip(Box<dyn ArchiveReader>),
}

impl Archive {
    /// Open an archive file, auto-detecting its format by magic bytes.
    ///
    /// ETS2 mod files use `.scs` as extension regardless of whether they
    /// are HashFS archives (`SCS#`) or plain ZIP files (`PK\x03\x04`).
    /// We therefore inspect the first four bytes rather than the extension.
    pub fn open(path: &Path) -> Result<Self> {
        // Read magic bytes to determine format.
        let magic = {
            use std::io::Read;
            let mut f = std::fs::File::open(path).map_err(crate::ets2_parser::error::Error::Io)?;
            let mut buf = [0u8; 4];
            f.read_exact(&mut buf)
                .map_err(crate::ets2_parser::error::Error::Io)?;
            buf
        };

        // ZIP magic: PK\x03\x04
        if magic == [0x50, 0x4B, 0x03, 0x04] {
            let reader = crate::ets2_parser::zip_reader::ZipArchive::open(path)?;
            return Ok(Archive::Zip(Box::new(reader)));
        }

        // SCS# magic — HashFS v1/v2.
        if magic == [0x53, 0x43, 0x53, 0x23] {
            let reader = crate::ets2_parser::scs_reader::ScsArchive::open(path)?;
            return Ok(Archive::Scs(Box::new(reader)));
        }

        // Fall back to extension-based detection for edge cases.
        let path_str = path.to_string_lossy().to_lowercase();
        if path_str.ends_with(".zip") {
            let reader = crate::ets2_parser::zip_reader::ZipArchive::open(path)?;
            return Ok(Archive::Zip(Box::new(reader)));
        }

        Err(crate::ets2_parser::error::Error::ArchiveFormat(format!(
            "unknown archive format (magic {:02X?}) for {}",
            magic,
            path.display()
        )))
    }
}

impl ArchiveReader for Archive {
    fn read_file(&mut self, logical_path: &str) -> Result<Vec<u8>> {
        match self {
            Archive::Scs(a) => a.read_file(logical_path),
            Archive::Zip(a) => a.read_file(logical_path),
        }
    }

    fn find_files_starting_with(&self, prefix: &str) -> Vec<String> {
        match self {
            Archive::Scs(a) => a.find_files_starting_with(prefix),
            Archive::Zip(a) => a.find_files_starting_with(prefix),
        }
    }

    fn entry_count(&self) -> usize {
        match self {
            Archive::Scs(a) => a.entry_count(),
            Archive::Zip(a) => a.entry_count(),
        }
    }

    fn read_file_with_texture(&mut self, logical_path: &str) -> Result<(Vec<u8>, Option<Vec<u8>>)> {
        match self {
            Archive::Scs(a) => a.read_file_with_texture(logical_path),
            Archive::Zip(a) => a.read_file_with_texture(logical_path),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use std::path::PathBuf;

    fn temp_path(name: &str) -> PathBuf {
        let mut p = std::env::temp_dir();
        p.push(name);
        p
    }

    fn make_test_zip(path: &std::path::Path) {
        let file = std::fs::File::create(path).unwrap();
        let mut zip = zip::write::ZipWriter::new(file);
        let options = zip::write::SimpleFileOptions::default()
            .compression_method(zip::CompressionMethod::Stored);
        zip.start_file("hello.txt", options).unwrap();
        zip.write_all(b"world").unwrap();
        zip.finish().unwrap();
    }

    fn make_test_scs(path: &std::path::Path) {
        // Valid v2 HashFS header (40 bytes) with zero-length tables
        let mut file = std::fs::File::create(path).unwrap();
        file.write_all(b"SCS#").unwrap(); // magic
        file.write_all(&2u16.to_le_bytes()).unwrap(); // version
        file.write_all(&0u16.to_le_bytes()).unwrap(); // salt
        file.write_all(b"CITY").unwrap(); // hash_method
        file.write_all(&0u32.to_le_bytes()).unwrap(); // entry_count = 0
        file.write_all(&0u32.to_le_bytes()).unwrap(); // entry_table_length = 0
        file.write_all(&0u32.to_le_bytes()).unwrap(); // metadata_table_length = 0
        file.write_all(&40u64.to_le_bytes()).unwrap(); // entry_table_start = 40 (after header)
        file.write_all(&40u64.to_le_bytes()).unwrap(); // metadata_table_start = 40
    }

    #[test]
    fn test_open_detects_zip() {
        let path = temp_path("truckpilot_test.zip");
        make_test_zip(&path);
        let archive = Archive::open(&path);
        assert!(archive.is_ok());
        match archive.unwrap() {
            Archive::Zip(_) => {}
            Archive::Scs(_) => panic!("expected Zip variant"),
        }
        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn test_open_detects_scs() {
        let path = temp_path("truckpilot_test.scs");
        make_test_scs(&path);
        let archive = Archive::open(&path);
        assert!(archive.is_ok());
        match archive.unwrap() {
            Archive::Scs(_) => {}
            Archive::Zip(_) => panic!("expected Scs variant"),
        }
        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn test_open_nonexistent_is_err() {
        let path = temp_path("truckpilot_nonexistent.zip");
        assert!(Archive::open(&path).is_err());
    }
}
