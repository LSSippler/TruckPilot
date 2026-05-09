//! HashFS layer — reads `.scs` archives (SCS HashFS v2 format).
//!
//! Format reference: `docs/hashfs_format.md`. Summary:
//!
//! ```text
//! HEADER (52 bytes, little-endian)
//!   0x00  4  Magic "SCS#"
//!   0x04  2  Version (must be 2)
//!   0x06  2  Salt (must be 0)
//!   0x08  4  Hash method "CITY"
//!   0x0C  4  entry_count
//!   0x10  4  entry_table_compressed_size
//!   0x14  4  metadata_word_count       (count of u32 words in inflated index2)
//!   0x18  4  metadata_table_compressed_size
//!   0x1C  8  entry_table_start         (u64, absolute file offset)
//!   0x24  8  metadata_table_start      (u64, absolute file offset)
//!   0x2C  8  security_descriptor_start (u64, always 0 in practice)
//!
//! ENTRY TABLE  (index1, zlib-compressed; 16 B/entry once inflated):
//!   0x00  8  hash             u64  CityHash64 of full path
//!   0x08  4  metadata_index   u32  offset into index2 in 4-byte words
//!   0x0C  2  metadata_count   u16  number of metadata records
//!   0x0E  2  flags            u16  bit 0 = is_directory
//!
//! METADATA TABLE (index2, zlib-compressed, variable-length record forest).
//! Each record's first 4 bytes carry an 8-bit `kind` at byte 0x03 that
//! determines its total length:
//!   0x80 → 16 B  (Data part — see below)
//!   0x05 → 32 B
//!   0x01 → 8 B
//!   0x06 → 8 B
//!   0x02 → 4 B
//!
//! DATA PART (kind 0x80, 16 B):
//!   0x00..0x04  u32  bits 0..27 = compressed_size; bit 28 = is_compressed;
//!                                                  bit 31 = data-part marker
//!   0x04..0x08  u32  bits 0..27 = size (uncompressed); bits 28..31 unused
//!   0x08..0x0C  u32  unknown
//!   0x0C..0x10  u32  offset_block — absolute data offset = offset_block * 16
//! ```

use std::collections::HashMap;
use std::io::Read;
use std::path::{Path, PathBuf};

use memmap2::Mmap;
use miniz_oxide::inflate::decompress_to_vec_zlib;
use sha2::{Digest, Sha256};
use tracing::{debug, info, trace, warn};

use crate::error::ParseError;

// ── helpers ─────────────────────────────────────────────────────────

/// Format a zlib error compactly without dumping its partial-output buffer.
fn short_zlib_err<E: std::fmt::Debug>(e: &E) -> String {
    // Take the type name + the first ~120 chars of Debug output, but stop
    // before the partial-output Vec gets dumped.
    let s = format!("{e:?}");
    let cut = s.find(", output:").unwrap_or(s.len().min(160));
    s[..cut].to_string()
}

// ── constants ───────────────────────────────────────────────────────

const SCS_MAGIC: [u8; 4] = *b"SCS#";
const SCS_CITY: [u8; 4] = *b"CITY";
/// Synthetic flag — set in `DirEntry.flags` based on the MainMetadata
/// compression bit (bit 28 of the packed `compressed_size` word). The
/// reader's read path branches on this to decide whether to inflate.
const FLAG_COMPRESSED: u32 = 1 << 1;
const LARGE_FILE_THRESHOLD: u64 = 100 * 1024 * 1024; // 100 MB

/// HashFS v2 header is exactly 52 bytes.
const HEADER_SIZE: usize = 0x34;

/// Index2 record kinds (kind byte sits at offset 0x03 of every record).
///
/// Aux kinds use the full byte value. The data-part record overlays the
/// kind byte with `flags1` from the MainMetadata layout — only bit 7 of
/// the byte is the actual "data part" marker; the lower bits encode the
/// compression method (0x10 = zlib). So we test data-part membership via
/// `byte & DATA_PART_MARKER`, never by exact byte equality.
const KIND_AUX_4: u8 = 0x02;
const KIND_AUX_8A: u8 = 0x01;
const KIND_AUX_8B: u8 = 0x06;
const KIND_AUX_32: u8 = 0x05;
const DATA_PART_MARKER: u8 = 0x80;

// ── storage ─────────────────────────────────────────────────────────

/// Backing storage for an archive.
enum Storage {
    /// Memory-mapped for large files (>100 MB).
    Mapped(Mmap),
    /// In-memory buffer for small files.
    Buffer(Vec<u8>),
}

impl Storage {
    fn len(&self) -> usize {
        match self {
            Storage::Mapped(m) => m.len(),
            Storage::Buffer(b) => b.len(),
        }
    }

    fn read(&self, offset: usize, len: usize) -> Result<&[u8], ParseError> {
        let buf = match self {
            Storage::Mapped(m) => &m[..],
            Storage::Buffer(b) => &b[..],
        };
        buf.get(offset..offset + len)
            .ok_or_else(|| ParseError::Archive(format!(
                "read out of bounds: offset={offset}, len={len}, file_len={}",
                buf.len()
            )))
    }
}

// ── DirEntry ────────────────────────────────────────────────────────

/// Metadata for one file in the archive.
#[derive(Debug, Clone)]
pub(crate) struct DirEntry {
    pub(crate) offset: u64,
    pub(crate) size: u32,
    pub(crate) compressed_size: u32,
    pub(crate) flags: u32,
}

// ── HashFsArchive ───────────────────────────────────────────────────

/// An opened `.scs` archive (SCS HashFS v2 format).
pub struct HashFsArchive {
    /// Path to the `.scs` file on disk.
    pub path: PathBuf,
    /// SHA-256 of the archive file (for cache keying).
    pub file_hash: [u8; 32],
    salt: u16,
    /// Hash → entry metadata.
    index: HashMap<u64, DirEntry>,
    storage: Storage,
}

// ── path hashing ────────────────────────────────────────────────────

/// Compute the SCS CityHash64 of a path string.
///
/// Paths must be lowercase and use `/` as separator, no leading `/`.
/// SCS HashFS v2 hashes the raw path bytes directly — the salt field in
/// the header is not mixed into individual path hashes.
pub fn scs_path_hash(_salt: u16, path: &str) -> u64 {
    cityhash64(path.as_bytes())
}

/// Pure CityHash64 — re-exported for external use.
pub use crate::archive::Archive;
use crate::cityhash::cityhash64;

// ── HashFsArchive impl ──────────────────────────────────────────────

impl HashFsArchive {
    /// Open a `.scs` file and build the hash index.
    ///
    /// For large archives (>100 MB) the file is memory-mapped via
    /// `memmap2`. Smaller files are read entirely into memory.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, ParseError> {
        let path = path.as_ref().to_path_buf();
        let file = std::fs::File::open(&path)?;

        let file_len = file
            .metadata()
            .map(|m| m.len())
            .map_err(|e| ParseError::Io(format!("metadata {:?}: {e}", &path)))?;

        // SHA-256 for cache keying
        let file_hash = {
            let mut f = std::fs::File::open(&path)?;
            let mut hasher = Sha256::new();
            let mut buf = vec![0u8; 65536];
            loop {
                let n = f.read(&mut buf)
                    .map_err(|e| ParseError::Io(format!("hash read: {e}")))?;
                if n == 0 {
                    break;
                }
                hasher.update(&buf[..n]);
            }
            hasher.finalize().into()
        };

        // Memory-map large files, read small ones
        let storage = if file_len > LARGE_FILE_THRESHOLD {
            let mmap = unsafe {
                Mmap::map(&file).map_err(|e| ParseError::Io(format!("mmap: {e}")))?
            };
            Storage::Mapped(mmap)
        } else {
            let mut buf = Vec::with_capacity(file_len as usize);
            (&mut &file).read_to_end(&mut buf)?;
            Storage::Buffer(buf)
        };

        // ── parse header ──
        let header = parse_header(&storage, &path)?;
        let salt = header.salt;

        info!(
            "HashFS {:?}: {} entries, entry_table @{} len={}, metadata_table @{} len={}, salt=0x{salt:04X}",
            path.file_name(),
            header.entry_count,
            header.entry_table_start,
            header.entry_table_compressed_size,
            header.metadata_table_start,
            header.metadata_table_compressed_size,
        );

        // ── read + zlib-inflate index1 (entry table) and index2 (metadata) ──
        let entry_data = inflate_table(
            &storage,
            header.entry_table_start,
            header.entry_table_compressed_size,
            "entry_table",
        )?;
        let metadata_data = inflate_table(
            &storage,
            header.metadata_table_start,
            header.metadata_table_compressed_size,
            "metadata_table",
        )?;

        let index = build_index(&entry_data, &metadata_data);

        debug!(
            "Opened HashFS {:?}: {} entries",
            path.file_name(),
            index.len()
        );

        Ok(Self {
            path,
            file_hash,
            salt,
            index,
            storage,
        })
    }

    /// Read a file by its CityHash64.
    pub fn read_hash(&self, hash: u64) -> Result<Vec<u8>, ParseError> {
        let entry = self
            .index
            .get(&hash)
            .ok_or_else(|| ParseError::EntryNotFound(format!("0x{hash:016X}")))?;

        let is_compressed = (entry.flags & FLAG_COMPRESSED) != 0;
        let read_len = if is_compressed {
            entry.compressed_size as usize
        } else {
            entry.size as usize
        };

        if read_len == 0 {
            return Ok(Vec::new());
        }

        let raw = self
            .storage
            .read(entry.offset as usize, read_len)
            .map_err(|e| {
                warn!("read_hash 0x{hash:016X}: {e}");
                e
            })?;

        if is_compressed {
            decompress_to_vec_zlib(raw).map_err(|e| {
                ParseError::DecompressionError(format!(
                    "file 0x{hash:016X}: {}",
                    short_zlib_err(&e)
                ))
            })
        } else {
            Ok(raw.to_vec())
        }
    }

    /// Number of entries in the archive.
    pub fn entry_count(&self) -> usize {
        self.index.len()
    }

    /// Return all hashes in this archive.
    pub fn list_hashes(&self) -> Vec<u64> {
        self.index.keys().copied().collect()
    }

    /// Return the DirEntry for a hash (used by higher layers).
    #[allow(dead_code)] // May be useful for debugging
    pub(crate) fn get_entry(&self, hash: u64) -> Option<&DirEntry> {
        self.index.get(&hash)
    }

    /// Return the salt used for path hashing in this archive.
    pub fn salt(&self) -> u16 {
        self.salt
    }

    /// Probe for known ETS2 sector paths and return those that exist.
    ///
    /// HashFS does not store path strings — we must guess them.
    /// This tries multiple common prefixes used by different ETS2 versions/mods.
    pub fn probe_sector_paths(&self) -> Vec<String> {
        let mut found = Vec::new();
        let prefixes = ["map/europe/", "map/", "europe/"];
        for prefix in &prefixes {
            for x in -25i32..=25 {
                for z in -25i32..=25 {
                    let sx = if x >= 0 {
                        format!("+{x:04}")
                    } else {
                        format!("-{:04}", x.unsigned_abs())
                    };
                    let sz = if z >= 0 {
                        format!("+{z:04}")
                    } else {
                        format!("-{:04}", z.unsigned_abs())
                    };
                    for ext in &[".base", ".aux", ".data", ".desc"] {
                        let path = format!("{prefix}sec{sx}{sz}{ext}");
                        if self.contains(&path) {
                            found.push(path);
                        }
                    }
                }
            }
        }
        found
    }
}

impl Archive for HashFsArchive {
    fn read_path(&mut self, path: &str) -> Result<Vec<u8>, ParseError> {
        let hash = scs_path_hash(self.salt, &path.to_lowercase());
        self.read_hash(hash)
    }

    fn contains(&self, path: &str) -> bool {
        self.index
            .contains_key(&scs_path_hash(self.salt, &path.to_lowercase()))
    }

    fn list_files(&self) -> Vec<String> {
        // HashFS does not store paths, so we can't list them.
        // Brute-force would require a massive dictionary.
        warn!("list_files() called on a HashFS archive; paths are not stored, returning empty list.");
        vec![]
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

// ── internal helpers ────────────────────────────────────────────────

/// Parsed v2 header.
struct HeaderV2 {
    entry_count: u32,
    entry_table_compressed_size: u32,
    metadata_table_compressed_size: u32,
    entry_table_start: u64,
    metadata_table_start: u64,
    salt: u16,
}

fn parse_header(storage: &Storage, path: &Path) -> Result<HeaderV2, ParseError> {
    let buf = storage
        .read(0, HEADER_SIZE)
        .map_err(|e| ParseError::Archive(format!("read header: {e}")))?;

    if buf[0x00..0x04] != SCS_MAGIC {
        return Err(ParseError::InvalidMagic(
            buf[0x00..0x04].try_into().unwrap(),
        ));
    }

    let version = u16::from_le_bytes(buf[0x04..0x06].try_into().unwrap());
    if version != 2 {
        return Err(ParseError::UnsupportedVersion(version));
    }

    let salt = u16::from_le_bytes(buf[0x06..0x08].try_into().unwrap());
    if salt != 0 {
        return Err(ParseError::Archive(format!(
            "unsupported non-zero salt in {:?}: {salt}",
            path.file_name()
        )));
    }

    if buf[0x08..0x0C] != SCS_CITY {
        return Err(ParseError::Archive(format!(
            "unsupported hash method {:?}: {:02X?}",
            path.file_name(),
            &buf[0x08..0x0C],
        )));
    }

    let entry_count = u32::from_le_bytes(buf[0x0C..0x10].try_into().unwrap());
    let entry_table_compressed_size =
        u32::from_le_bytes(buf[0x10..0x14].try_into().unwrap());
    // 0x14..0x18 is metadata_word_count (u32) — sanity-check value, not retained.
    let _metadata_word_count =
        u32::from_le_bytes(buf[0x14..0x18].try_into().unwrap());
    let metadata_table_compressed_size =
        u32::from_le_bytes(buf[0x18..0x1C].try_into().unwrap());
    let entry_table_start = u64::from_le_bytes(buf[0x1C..0x24].try_into().unwrap());
    let metadata_table_start =
        u64::from_le_bytes(buf[0x24..0x2C].try_into().unwrap());
    // 0x2C..0x34 is security_descriptor_start (u64, always 0).

    Ok(HeaderV2 {
        entry_count,
        entry_table_compressed_size,
        metadata_table_compressed_size,
        entry_table_start,
        metadata_table_start,
        salt,
    })
}

/// Read a zlib-compressed table from the archive and return the inflated bytes.
fn inflate_table(
    storage: &Storage,
    start: u64,
    compressed_size: u32,
    label: &str,
) -> Result<Vec<u8>, ParseError> {
    if compressed_size == 0 {
        return Ok(Vec::new());
    }
    let start_usize = start as usize;
    if start_usize >= storage.len() {
        return Err(ParseError::Archive(format!(
            "{label} start {start_usize} past end of file ({} bytes)",
            storage.len()
        )));
    }
    let compressed = storage.read(start_usize, compressed_size as usize)?;
    let inflated = decompress_to_vec_zlib(compressed).map_err(|e| {
        ParseError::DecompressionError(format!("{label}: {}", short_zlib_err(&e)))
    })?;
    debug!(
        "{label}: {} compressed → {} decompressed",
        compressed_size,
        inflated.len()
    );
    Ok(inflated)
}

/// Build the hash → `DirEntry` index by walking each entry's metadata
/// records in `index2` until a data part (kind 0x80) is found.
fn build_index(entry_data: &[u8], metadata_data: &[u8]) -> HashMap<u64, DirEntry> {
    const ENTRY_SIZE: usize = 16;
    let count = entry_data.len() / ENTRY_SIZE;
    let mut index = HashMap::with_capacity(count);

    let mut resolve_failed = 0usize;
    let mut size_zero_filtered = 0usize;

    for i in 0..count {
        let base = i * ENTRY_SIZE;
        let hash =
            u64::from_le_bytes(entry_data[base..base + 8].try_into().unwrap());
        let metadata_index =
            u32::from_le_bytes(entry_data[base + 8..base + 12].try_into().unwrap());
        let metadata_count =
            u16::from_le_bytes(entry_data[base + 12..base + 14].try_into().unwrap());
        let flags_raw =
            u16::from_le_bytes(entry_data[base + 14..base + 16].try_into().unwrap());

        match resolve_data_part(
            metadata_data,
            metadata_index,
            metadata_count,
            flags_raw,
        ) {
            Some(entry) if entry.size > 0 => {
                trace!(
                    "0x{hash:016X} → offset={} size={} cs={}",
                    entry.offset, entry.size, entry.compressed_size
                );
                index.insert(hash, entry);
            }
            Some(_) => size_zero_filtered += 1,
            None => resolve_failed += 1,
        }
    }

    debug!(
        "build_index: {} raw entries → {} indexed (resolve failed: {}, size==0: {})",
        count,
        index.len(),
        resolve_failed,
        size_zero_filtered
    );

    index
}

/// Walk this entry's part list and return the data-part decoded into a
/// `DirEntry`.
///
/// Index2 is structured in two layers:
///   1. The entry references a contiguous run of **4-byte mini-headers** at
///      `metadata_index * 4` (one per part, `metadata_count` of them).
///      Each mini-header carries a 24-bit offset (in 4-byte words) and an
///      8-bit kind: `(u16 offset_lo, u8 offset_hi, u8 kind)`.
///   2. The mini-header's offset points at the part's **actual body**
///      elsewhere in index2. For a data part (kind bit 7 set) the body is
///      16 bytes; aux kinds have their own sizes.
///
/// We walk the mini-headers, dereference each offset to read the body,
/// and stop on the first data part.
///
/// TODO: texture entries can carry multiple data parts (MIP_0, MIP_TAIL).
/// For sector / SII files there is exactly one data part so this is fine
/// for `base.scs`, but a future texture path will need to expose all of
/// them.
fn resolve_data_part(
    metadata_data: &[u8],
    metadata_index: u32,
    metadata_count: u16,
    entry_flags: u16,
) -> Option<DirEntry> {
    if metadata_count == 0 {
        return None;
    }

    let headers_start = (metadata_index as usize).checked_mul(4)?;
    for k in 0..metadata_count as usize {
        let header_pos = headers_start.checked_add(k.checked_mul(4)?)?;
        if header_pos + 4 > metadata_data.len() {
            return None;
        }

        let offset_lo =
            u16::from_le_bytes(metadata_data[header_pos..header_pos + 2].try_into().unwrap());
        let offset_hi = metadata_data[header_pos + 2];
        let kind_byte = metadata_data[header_pos + 3];

        // 24-bit body offset, expressed in units of 4-byte words.
        let body_offset_words = (offset_lo as u32) | ((offset_hi as u32) << 16);
        let body_pos = (body_offset_words as usize).checked_mul(4)?;

        let body_len = if kind_byte & DATA_PART_MARKER != 0 {
            16
        } else {
            match kind_byte {
                KIND_AUX_8A | KIND_AUX_8B => 8,
                KIND_AUX_4 => 4,
                KIND_AUX_32 => 32,
                _ => return None,
            }
        };
        if body_pos + body_len > metadata_data.len() {
            return None;
        }

        if kind_byte & DATA_PART_MARKER != 0 {
            return parse_main_metadata(
                &metadata_data[body_pos..body_pos + 16],
                entry_flags,
            );
        }
    }
    None
}

/// Decode the 16-byte data-part body into a `DirEntry`.
///
/// Layout (per Archive-SCS HashFS2.pm `unpack 'SCC SCC LL'`):
///   0x00..0x02  zsize_lo  u16
///   0x02        zsize_hi  u8
///   0x03        flags2    u8   bits 4..7 = compression method (0x10 = zlib)
///   0x04..0x06  usize_lo  u16
///   0x06        usize_hi  u8
///   0x07        flags3    u8   (unused)
///   0x08..0x0C  unknown7  u32
///   0x0C..0x10  data_off  u32  absolute offset = data_off * 16
fn parse_main_metadata(body: &[u8], entry_flags: u16) -> Option<DirEntry> {
    debug_assert_eq!(body.len(), 16);

    let zsize = (u16::from_le_bytes(body[0..2].try_into().unwrap()) as u32)
        | ((body[2] as u32) << 16);
    let flags2 = body[3];
    let usize_ = (u16::from_le_bytes(body[4..6].try_into().unwrap()) as u32)
        | ((body[6] as u32) << 16);
    // body[7] = flags3, currently unused.
    // body[8..12] = unknown7, currently unused.
    let data_offset_blocks = u32::from_le_bytes(body[12..16].try_into().unwrap());

    let is_compressed = (flags2 & 0xF0) != 0;
    let abs_offset = (data_offset_blocks as u64).checked_mul(16)?;
    if abs_offset == 0 {
        return None;
    }

    let mut flags = entry_flags as u32;
    if is_compressed {
        flags |= FLAG_COMPRESSED;
    } else {
        flags &= !FLAG_COMPRESSED;
    }

    Some(DirEntry {
        offset: abs_offset,
        size: usize_,
        compressed_size: if is_compressed { zsize } else { 0 },
        flags,
    })
}

// ── tests ───────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn build_v2_header() -> [u8; HEADER_SIZE] {
        let mut buf = [0u8; HEADER_SIZE];
        buf[0x00..0x04].copy_from_slice(b"SCS#");
        buf[0x04..0x06].copy_from_slice(&2u16.to_le_bytes());
        buf[0x06..0x08].copy_from_slice(&0u16.to_le_bytes());
        buf[0x08..0x0C].copy_from_slice(b"CITY");
        buf
    }

    #[test]
    fn test_parse_header_v2_layout() {
        let mut buf = build_v2_header();
        buf[0x0C..0x10].copy_from_slice(&0x1111_1111u32.to_le_bytes());
        buf[0x10..0x14].copy_from_slice(&0x2222_2222u32.to_le_bytes());
        buf[0x14..0x18].copy_from_slice(&0x3333_3333u32.to_le_bytes());
        buf[0x18..0x1C].copy_from_slice(&0x4444_4444u32.to_le_bytes());
        buf[0x1C..0x24].copy_from_slice(&0x5555_5555_5555_5555u64.to_le_bytes());
        buf[0x24..0x2C].copy_from_slice(&0x6666_6666_6666_6666u64.to_le_bytes());

        let storage = Storage::Buffer(buf.to_vec());
        let h = parse_header(&storage, Path::new("synthetic.scs"))
            .expect("header should parse");

        assert_eq!(h.entry_count, 0x1111_1111);
        assert_eq!(h.entry_table_compressed_size, 0x2222_2222);
        assert_eq!(h.metadata_table_compressed_size, 0x4444_4444);
        assert_eq!(h.entry_table_start, 0x5555_5555_5555_5555);
        assert_eq!(h.metadata_table_start, 0x6666_6666_6666_6666);
        assert_eq!(h.salt, 0);
    }

    #[test]
    fn test_parse_header_rejects_v1() {
        let mut buf = build_v2_header();
        buf[0x04..0x06].copy_from_slice(&1u16.to_le_bytes());
        let storage = Storage::Buffer(buf.to_vec());
        assert!(parse_header(&storage, Path::new("x")).is_err());
    }

    #[test]
    fn test_parse_header_rejects_nonzero_salt() {
        let mut buf = build_v2_header();
        buf[0x06..0x08].copy_from_slice(&7u16.to_le_bytes());
        let storage = Storage::Buffer(buf.to_vec());
        assert!(parse_header(&storage, Path::new("x")).is_err());
    }

    /// Build a 4-byte mini-header (offset_lo u16, offset_hi u8, kind u8).
    /// `body_off_words` is the offset to the body in 4-byte word units.
    fn build_mini_header(body_off_words: u32, kind: u8) -> [u8; 4] {
        let mut h = [0u8; 4];
        h[0..2].copy_from_slice(&((body_off_words & 0xFFFF) as u16).to_le_bytes());
        h[2] = ((body_off_words >> 16) & 0xFF) as u8;
        h[3] = kind;
        h
    }

    /// Build a 16-byte data-part body (zsize/usize 24-bit + 8-bit flags + u32s).
    fn build_data_part_body(
        zsize: u32,
        usize_: u32,
        data_off_blocks: u32,
        is_compressed: bool,
    ) -> [u8; 16] {
        let mut body = [0u8; 16];
        body[0..2]
            .copy_from_slice(&((zsize & 0xFFFF) as u16).to_le_bytes());
        body[2] = ((zsize >> 16) & 0xFF) as u8;
        body[3] = if is_compressed { 0x10 } else { 0x00 };
        body[4..6]
            .copy_from_slice(&((usize_ & 0xFFFF) as u16).to_le_bytes());
        body[6] = ((usize_ >> 16) & 0xFF) as u8;
        // body[7] = 0; body[8..12] = 0
        body[12..16].copy_from_slice(&data_off_blocks.to_le_bytes());
        body
    }

    #[test]
    fn test_resolve_data_part_via_mini_header() {
        // Layout (byte offsets):
        //   0..4   data-part body (16 B) starts here. Mini-headers point here.
        //   ...
        //   16..32 data-part body
        //   32..36 mini-header for aux record (kind 0x01) — points to body @ word 9
        //   36..40 mini-header for data part (kind 0x80) — points to body @ word 4
        //   40..48 aux body (8 bytes)
        //
        // metadata_index = 8 (= word 8) → the two mini-headers
        // metadata_count = 2 (one aux, one data part)

        let mut idx2 = vec![0u8; 64];

        // Data-part body at word 4 (= byte 16).
        idx2[16..32]
            .copy_from_slice(&build_data_part_body(100, 250, 0x40, true));

        // Mini-headers at word 8 (= byte 32).
        idx2[32..36].copy_from_slice(&build_mini_header(9, KIND_AUX_8A));
        idx2[36..40].copy_from_slice(&build_mini_header(4, DATA_PART_MARKER));

        // Aux body at word 9 (= byte 36..40 — actually overlaps with the
        // second mini-header in this synthetic layout, but we never read
        // its contents since we stop at the data part. Just give it 8
        // valid bytes somewhere.)
        idx2[40..48].copy_from_slice(&[0u8; 8]);
        // Re-point the aux mini-header to byte 40 = word 10 to avoid overlap.
        idx2[32..36].copy_from_slice(&build_mini_header(10, KIND_AUX_8A));

        let entry = resolve_data_part(&idx2, 8, 2, 0)
            .expect("should resolve the data-part mini-header");
        assert_eq!(entry.offset, 0x400);
        assert_eq!(entry.size, 250);
        assert_eq!(entry.compressed_size, 100);
        assert!(entry.flags & FLAG_COMPRESSED != 0);
    }

    #[test]
    fn test_resolve_data_part_uncompressed() {
        // Mini-header at word 0 (byte 0), data-part body at word 1 (byte 4).
        let mut idx2 = vec![0u8; 4 + 16];
        idx2[0..4].copy_from_slice(&build_mini_header(1, DATA_PART_MARKER));
        idx2[4..20]
            .copy_from_slice(&build_data_part_body(0, 500, 0x10, false));

        let entry = resolve_data_part(&idx2, 0, 1, 0)
            .expect("uncompressed data part should resolve");
        assert_eq!(entry.offset, 0x100);
        assert_eq!(entry.size, 500);
        assert_eq!(entry.compressed_size, 0);
        assert!(entry.flags & FLAG_COMPRESSED == 0);
    }
}
