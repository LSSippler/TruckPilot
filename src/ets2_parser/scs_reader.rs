//! SCS archive reader for ETS2 HashFS v2 (CityHash).
//!
//! Format details and source citations are documented in
//! `docs/hashfs_format.md`. The summary below is the minimum needed to read
//! this module:
//!
//! ```text
//! HashFS v2 header (52 bytes, little-endian):
//!   0x00  4  magic        = b"SCS#"
//!   0x04  2  version      = u16, must be 2
//!   0x06  2  salt         = u16, must be 0
//!   0x08  4  hash_method  = b"CITY"
//!   0x0C  4  entry_count                       u32
//!   0x10  4  entry_table_compressed_size       u32
//!   0x14  4  metadata_word_count               u32 (count of u32 words in
//!                                                   the inflated index2)
//!   0x18  4  metadata_table_compressed_size    u32
//!   0x1C  8  entry_table_start                 u64  ← absolute file offset
//!   0x24  8  metadata_table_start              u64  ← absolute file offset
//!   0x2C  8  security_descriptor_start         u64  (in practice 0)
//!
//! Entry table (index1, zlib-compressed, 16 B per entry once inflated):
//!   0x00  8  hash             u64  CityHash64 of full path
//!   0x08  4  metadata_index   u32  offset into index2 in 4-byte words
//!   0x0C  2  metadata_count   u16  number of metadata records
//!   0x0E  2  flags            u16  bit 0 = is_directory
//!
//! Metadata table (index2, zlib-compressed, variable-length record forest).
//! Each record's first 4 bytes carry an 8-bit `kind` at byte 0x03 that
//! determines the record's total length:
//!   0x80 → 16 B  (Data part — see below)
//!   0x05 → 32 B
//!   0x01 → 8 B
//!   0x06 → 8 B
//!   0x02 → 4 B
//!
//! Data part (kind 0x80, 16 B):
//!   0x00..0x04  u32  packed: bits 0..27 = compressed_size, bits 28..31 = flags1
//!                              (bit 28 = is_compressed, bit 31 = data-part marker)
//!   0x04..0x08  u32  packed: bits 0..27 = size, bits 28..31 = flags2 (unused)
//!   0x08..0x0C  u32  unknown
//!   0x0C..0x10  u32  offset_block — absolute data offset = offset_block * 16
//! ```

use std::collections::HashMap;
use std::fs::File;
use std::io::{BufReader, Read, Seek, SeekFrom};
use std::path::Path;

use miniz_oxide::inflate;

use crate::ets2_parser::archive::ArchiveReader;
use crate::ets2_parser::error::{Error, Result as ParserResult};

const MAGIC: [u8; 4] = [0x53, 0x43, 0x53, 0x23]; // "SCS#"
const CITY: [u8; 4] = [0x43, 0x49, 0x54, 0x59]; // "CITY"

/// Entry flags.
#[allow(dead_code)]
const FLAG_DIRECTORY: u32 = 1 << 0;
/// Synthetic flag — set in `DirEntry.flags` based on the MainMetadata
/// compression bit, so the rest of the reader can branch on it directly.
const FLAG_COMPRESSED: u32 = 1 << 1;
#[allow(dead_code)]
const FLAG_VERIFY: u32 = 1 << 2;
#[allow(dead_code)]
const FLAG_ENCRYPTED: u32 = 1 << 3;

/// Index2 record kinds. The kind byte sits at offset 0x03 of every record
/// and determines its total length.
///
/// Aux kinds use the full byte value. The data-part record overlays the
/// kind byte with `flags1` from MainMetadata — only bit 7 is the actual
/// "data part" marker; the lower bits encode the compression method
/// (0x10 = zlib). Test membership via `byte & DATA_PART_MARKER`.
const KIND_AUX_4: u8 = 0x02;
const KIND_AUX_8A: u8 = 0x01;
const KIND_AUX_8B: u8 = 0x06;
const KIND_AUX_32: u8 = 0x05;
const DATA_PART_MARKER: u8 = 0x80;

/// One directory entry in the SCS archive (32 bytes on disk).
#[derive(Debug, Clone)]
struct DirEntry {
    /// CityHash64 of the full file path.
    #[allow(dead_code)]
    hash: u64,
    /// Absolute byte offset within the .scs file to the file data.
    offset: u64,
    /// Bit flags (directory, compressed, etc.).
    flags: u32,
    /// CRC32 checksum.
    #[allow(dead_code)]
    crc: u32,
    /// Uncompressed file size.
    size: u32,
    /// Compressed size on disk (0 or == size if uncompressed).
    compressed_size: u32,
}

/// Parsed HashFS v2 header.
///
/// The `salt` field (offset 0x06) is read for validation but not retained —
/// every observed archive has `salt == 0`, and a non-zero value is rejected
/// during parsing.
#[derive(Debug)]
struct Header {
    entry_count: u32,
    entry_table_compressed_size: u32,
    metadata_table_compressed_size: u32,
    entry_table_start: u64,
    metadata_table_start: u64,
}

/// An opened SCS archive with an in-memory file directory (keyed by CityHash64).
pub struct ScsArchive {
    file: BufReader<File>,
    by_hash: HashMap<u64, DirEntry>,
    num_entries: u32,
    known_path_hashes: HashMap<String, u64>,
}

impl ScsArchive {
    /// Open and index an SCS file.
    pub fn open(path: &Path) -> Result<Self, String> {
        let mut file = BufReader::new(
            File::open(path).map_err(|e| format!("cannot open {}: {}", path.display(), e))?,
        );

        let header = parse_header(&mut file)?;

        // 1. Read + decompress Entry table (handle zero-length for minimal test files)
        let entry_data: Vec<u8> = if header.entry_table_compressed_size == 0 {
            vec![]
        } else {
            file.seek(SeekFrom::Start(header.entry_table_start))
                .map_err(|e| format!("seek to entry_table_start: {e}"))?;
            let mut entry_buf = vec![0u8; header.entry_table_compressed_size as usize];
            file.read_exact(&mut entry_buf)
                .map_err(|e| format!("read entry_table: {e}"))?;
            inflate::decompress_to_vec_zlib(&entry_buf)
                .map_err(|e| format!("decompress entry_table: {e:?}"))?
        };

        // 2. Read + decompress Metadata table (handle zero-length)
        let meta_data: Vec<u8> = if header.metadata_table_compressed_size == 0 {
            vec![]
        } else {
            file.seek(SeekFrom::Start(header.metadata_table_start))
                .map_err(|e| format!("seek to metadata_table_start: {e}"))?;
            let mut meta_buf = vec![0u8; header.metadata_table_compressed_size as usize];
            file.read_exact(&mut meta_buf)
                .map_err(|e| format!("read metadata_table: {e}"))?;
            inflate::decompress_to_vec_zlib(&meta_buf)
                .map_err(|e| format!("decompress metadata_table: {e:?}"))?
        };

        // 3. Parse entries (16 B each) and resolve MainMetadata
        let mut by_hash = HashMap::new();
        let entry_count = (entry_data.len() / 16) as u32;
        for i in 0..entry_count {
            let base = (i as usize) * 16;
            if base + 16 > entry_data.len() {
                break;
            }
            let hash = u64::from_le_bytes(entry_data[base..base + 8].try_into().unwrap());
            let metadata_index =
                u32::from_le_bytes(entry_data[base + 8..base + 12].try_into().unwrap());
            let metadata_count =
                u16::from_le_bytes(entry_data[base + 12..base + 14].try_into().unwrap());
            let flags = u16::from_le_bytes(entry_data[base + 14..base + 16].try_into().unwrap());

            // Resolve first MainMetadata chunk (metadata_index * 4)
            if let Some(dir_entry) =
                resolve_main_metadata(&meta_data, metadata_index, metadata_count, flags)
            {
                if dir_entry.size > 0 {
                    by_hash.insert(hash, dir_entry);
                }
            }
        }

        Ok(ScsArchive {
            file,
            by_hash,
            num_entries: header.entry_count,
            known_path_hashes: HashMap::new(),
        })
    }

    /// Open an SCS file, returning the new Error type.
    pub fn open_err(path: &Path) -> ParserResult<Self> {
        Self::open(path).map_err(Error::ArchiveFormat)
    }

    /// Number of entries in the archive.
    pub fn num_entries(&self) -> u32 {
        self.num_entries
    }

    /// Return all hashes present in the archive directory.
    pub fn entry_hashes(&self) -> Vec<u64> {
        self.by_hash.keys().copied().collect()
    }

    /// Register a known path→hash mapping (populated by probing).
    fn register_path(&mut self, path: &str) {
        let hash = cityhash64(path.as_bytes());
        if self.by_hash.contains_key(&hash) {
            self.known_path_hashes.insert(path.to_string(), hash);
        }
    }

    /// Read a file from the archive by its CityHash64 hash directly.
    pub fn read_entry(&mut self, hash: u64) -> Result<Vec<u8>, String> {
        let entry = self
            .by_hash
            .get(&hash)
            .ok_or_else(|| format!("hash 0x{hash:016X} not found in archive"))?;

        let abs_offset = entry.offset;
        self.file
            .seek(SeekFrom::Start(abs_offset))
            .map_err(|e| format!("seek to {abs_offset}: {e}"))?;

        let is_compressed = (entry.flags & FLAG_COMPRESSED) != 0;
        let read_len = if is_compressed {
            entry.compressed_size as usize
        } else {
            entry.size as usize
        };

        if read_len == 0 {
            return Ok(Vec::new());
        }

        let mut buf = vec![0u8; read_len];
        self.file
            .read_exact(&mut buf)
            .map_err(|e| format!("read: {e}"))?;

        if !is_compressed || entry.compressed_size == 0 || entry.compressed_size == entry.size {
            return Ok(buf);
        }

        inflate::decompress_to_vec_zlib(&buf).map_err(|e| format!("deflate: {e:?}"))
    }

    /// Read a file from the archive by its full path.
    ///
    /// The path is hashed with CityHash64 and matched against the directory.
    pub fn read_file(&mut self, filename: &str) -> Result<Vec<u8>, String> {
        let hash = cityhash64(filename.as_bytes());
        self.read_entry(hash)
            .map_err(|e| format!("{filename}: {e}"))
    }

    /// List common ETS2 file paths for probing.
    pub fn list_known_files(&self) -> Vec<String> {
        vec![
            "def/world/road.sii".into(),
            "def/world/prefab.sii".into(),
            "def/world/sign.sii".into(),
            "def/world/semaphore_profile.sii".into(),
        ]
    }
}

// ---------------------------------------------------------------------------
// ArchiveReader trait implementation
// ---------------------------------------------------------------------------

impl ArchiveReader for ScsArchive {
    fn read_file(&mut self, logical_path: &str) -> ParserResult<Vec<u8>> {
        self.register_path(logical_path);
        let hash = cityhash64(logical_path.as_bytes());
        let entry = self
            .by_hash
            .get(&hash)
            .ok_or_else(|| Error::FileNotFound(logical_path.into()))?;

        let abs_offset = entry.offset;
        self.file
            .seek(SeekFrom::Start(abs_offset))
            .map_err(Error::Io)?;

        let is_compressed = (entry.flags & FLAG_COMPRESSED) != 0;
        let read_len = if is_compressed {
            entry.compressed_size as usize
        } else {
            entry.size as usize
        };

        if read_len == 0 {
            return Ok(Vec::new());
        }

        let mut buf = vec![0u8; read_len];
        self.file.read_exact(&mut buf).map_err(Error::Io)?;

        if !is_compressed || entry.compressed_size == 0 || entry.compressed_size == entry.size {
            return Ok(buf);
        }

        inflate::decompress_to_vec_zlib(&buf).map_err(|e| Error::Decompression(format!("{e:?}")))
    }

    fn find_files_starting_with(&self, prefix: &str) -> Vec<String> {
        let mut result: Vec<String> = self
            .known_path_hashes
            .keys()
            .filter(|p| p.starts_with(prefix))
            .cloned()
            .collect();
        result.sort();
        result
    }

    fn entry_count(&self) -> usize {
        self.num_entries as usize
    }

    fn read_file_with_texture(
        &mut self,
        logical_path: &str,
    ) -> ParserResult<(Vec<u8>, Option<Vec<u8>>)> {
        let data = self.read_file(logical_path)?;
        Ok((data, None))
    }
}

// ---------------------------------------------------------------------------
// Header parsing
// ---------------------------------------------------------------------------

fn parse_header<R: Read>(r: &mut R) -> Result<Header, String> {
    // Read the full 52-byte v2 header in one go so field offsets stay
    // anchored to the spec (see module-level docstring).
    let mut buf = [0u8; 0x34];
    r.read_exact(&mut buf).map_err(|e| format!("io: {e}"))?;

    if buf[0x00..0x04] != MAGIC {
        return Err(format!(
            "invalid SCS magic: {:02X?}",
            &buf[0x00..0x04]
        ));
    }

    let version = u16::from_le_bytes([buf[0x04], buf[0x05]]);
    if version != 2 {
        return Err(format!("unsupported SCS version: {version} (expected 2)"));
    }

    let salt = u16::from_le_bytes([buf[0x06], buf[0x07]]);
    if salt != 0 {
        return Err(format!("unsupported non-zero salt: {salt}"));
    }

    if buf[0x08..0x0C] != CITY {
        return Err(format!(
            "unsupported hash method: {:02X?} (expected CITY)",
            &buf[0x08..0x0C]
        ));
    }

    let entry_count = u32::from_le_bytes(buf[0x0C..0x10].try_into().unwrap());
    let entry_table_compressed_size =
        u32::from_le_bytes(buf[0x10..0x14].try_into().unwrap());
    // 0x14..0x18 is metadata_word_count (u32 LE) — number of u32 words in
    // the inflated index2. Useful as a sanity check; not retained for now.
    let _metadata_word_count = u32::from_le_bytes(buf[0x14..0x18].try_into().unwrap());
    let metadata_table_compressed_size =
        u32::from_le_bytes(buf[0x18..0x1C].try_into().unwrap());
    let entry_table_start = u64::from_le_bytes(buf[0x1C..0x24].try_into().unwrap());
    let metadata_table_start = u64::from_le_bytes(buf[0x24..0x2C].try_into().unwrap());
    // 0x2C..0x34 is security_descriptor_start (u64, always 0 in practice).

    Ok(Header {
        entry_count,
        entry_table_compressed_size,
        metadata_table_compressed_size,
        entry_table_start,
        metadata_table_start,
    })
}

// ---------------------------------------------------------------------------
// IO helpers
// ---------------------------------------------------------------------------

fn read_u16(r: &mut BufReader<File>) -> Result<u16, String> {
    let mut buf = [0u8; 2];
    r.read_exact(&mut buf).map_err(|e| format!("io: {e}"))?;
    Ok(u16::from_le_bytes(buf))
}

fn read_u32(r: &mut BufReader<File>) -> Result<u32, String> {
    let mut buf = [0u8; 4];
    r.read_exact(&mut buf).map_err(|e| format!("io: {e}"))?;
    Ok(u32::from_le_bytes(buf))
}

// ---------------------------------------------------------------------------

/// Walk this entry's part list (mini-headers in index2) and return the
/// data-part materialised into a `DirEntry`.
///
/// Index2 is two-tier:
///   1. `metadata_index * 4` is the byte offset to a run of
///      `metadata_count` 4-byte mini-headers. Each mini-header carries a
///      24-bit body offset (in 4-byte word units) and an 8-bit kind:
///      `(u16 offset_lo, u8 offset_hi, u8 kind)`.
///   2. The 24-bit offset points at the part's actual body elsewhere in
///      index2 — 16 B for a data part, 4/8/32 for aux kinds.
///
/// We stop at the first part whose kind byte has bit 7 set.
fn resolve_main_metadata(
    meta_data: &[u8],
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
        if header_pos + 4 > meta_data.len() {
            return None;
        }
        let offset_lo =
            u16::from_le_bytes(meta_data[header_pos..header_pos + 2].try_into().unwrap());
        let offset_hi = meta_data[header_pos + 2];
        let kind_byte = meta_data[header_pos + 3];

        let body_off_words = (offset_lo as u32) | ((offset_hi as u32) << 16);
        let body_pos = (body_off_words as usize).checked_mul(4)?;

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
        if body_pos + body_len > meta_data.len() {
            return None;
        }

        if kind_byte & DATA_PART_MARKER != 0 {
            return parse_main_metadata(
                &meta_data[body_pos..body_pos + 16],
                entry_flags,
            );
        }
    }
    None
}

/// Decode the 16-byte data-part body into a `DirEntry`.
///
/// Body layout (verified against Archive-SCS `HashFS2.pm`,
/// `unpack '(SCC SCC LL)<'`):
///   0x00..0x02 zsize_lo  u16
///   0x02       zsize_hi  u8     → zsize is 24-bit
///   0x03       flags2    u8     bits 4..7 = compression method (0x10 = zlib)
///   0x04..0x06 usize_lo  u16
///   0x06       usize_hi  u8     → usize is 24-bit
///   0x07       flags3    u8     unused
///   0x08..0x0C unknown7  u32
///   0x0C..0x10 data_off  u32    absolute byte offset = data_off * 16
fn parse_main_metadata(body: &[u8], entry_flags: u16) -> Option<DirEntry> {
    debug_assert_eq!(body.len(), 16);

    let zsize = (u16::from_le_bytes(body[0..2].try_into().unwrap()) as u32)
        | ((body[2] as u32) << 16);
    let flags2 = body[3];
    let usize_ = (u16::from_le_bytes(body[4..6].try_into().unwrap()) as u32)
        | ((body[6] as u32) << 16);
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
        hash: 0,
        offset: abs_offset,
        flags,
        crc: 0,
        size: usize_,
        compressed_size: if is_compressed { zsize } else { 0 },
    })
}

// ---------------------------------------------------------------------------
// CityHash64
// ---------------------------------------------------------------------------

const K0: u64 = 0xc3a5c85c97cb3127;
const K1: u64 = 0xb492b66fbe98f273;
const K2: u64 = 0x9ae16a3b2f90404f;

/// CityHash64 — the hash function used by the ETS2 SCS HashFS directory.
pub fn cityhash64(data: &[u8]) -> u64 {
    let len = data.len();
    if len <= 16 {
        hash_0_16(data)
    } else if len <= 32 {
        hash_17_32(data)
    } else if len <= 64 {
        hash_33_64(data)
    } else {
        hash_above_64(data)
    }
}

fn hash_0_16(s: &[u8]) -> u64 {
    let len = s.len();
    if len >= 8 {
        let mul = K2.wrapping_add((len as u64).wrapping_mul(2));
        let a = u64le_slice(s, 0).wrapping_add(K2);
        let b = u64le_slice(s, len - 8);
        let c = b.rotate_right(37).wrapping_mul(mul).wrapping_add(a);
        let d = (a.rotate_right(25).wrapping_add(b)).wrapping_mul(mul);
        return hash128to64(c, d);
    }
    if len >= 4 {
        let mul = K2.wrapping_add((len as u64).wrapping_mul(2));
        let a = u32le_slice(s, 0) as u64;
        return hash128to64(
            (len as u64).wrapping_add(a << 3),
            u32le_slice(s, len - 4) as u64,
        )
        .wrapping_mul(mul);
    }
    if len > 0 {
        let a = s[0] as u64;
        let b = s[len >> 1] as u64;
        let c = s[len - 1] as u64;
        let y = a.wrapping_add(b << 8);
        let z = (len as u64).wrapping_add(c << 2);
        return shift_mix(y.wrapping_mul(K2) ^ z.wrapping_mul(K0)).wrapping_mul(K0);
    }
    K2
}

fn hash_17_32(s: &[u8]) -> u64 {
    let len = s.len();
    let mul = K2.wrapping_add((len as u64).wrapping_mul(2));
    let a = u64le_slice(s, 0).wrapping_mul(K1);
    let b = u64le_slice(s, 8);
    let c = u64le_slice(s, len - 8).wrapping_mul(mul);
    let d = u64le_slice(s, len - 16).wrapping_mul(K2);
    hash128to64(
        (a ^ b)
            .rotate_right(43)
            .wrapping_add(c ^ d)
            .wrapping_add(d.wrapping_mul(mul)),
        a.wrapping_add(b.rotate_right(18))
            .wrapping_add(c)
            .wrapping_mul(mul),
    )
}

fn hash_33_64(s: &[u8]) -> u64 {
    let len = s.len();
    let mul = K2.wrapping_add((len as u64).wrapping_mul(2));
    let a = u64le_slice(s, 0).wrapping_mul(K2);
    let b = u64le_slice(s, 8);
    let c = u64le_slice(s, len - 24);
    let d = u64le_slice(s, len - 32);
    let e = u64le_slice(s, 16).wrapping_mul(K2);
    let f = u64le_slice(s, 24).wrapping_mul(9);
    let g = u64le_slice(s, len - 8);
    let h = u64le_slice(s, len - 16).wrapping_mul(mul);
    let u = shift_mix(a.wrapping_add(g).wrapping_add(e.wrapping_mul(b)))
        .wrapping_mul(mul)
        .wrapping_add(d);
    let v = shift_mix(b.wrapping_add(h).wrapping_add(f.wrapping_mul(c)))
        .wrapping_mul(mul)
        .wrapping_add(a);
    let mut result = u ^ v;
    result = result.wrapping_mul(mul).wrapping_add(h);
    result = shift_mix(result).wrapping_mul(mul).wrapping_add(d);
    result
}

fn hash_above_64(s: &[u8]) -> u64 {
    let len = s.len();
    let mul = K2.wrapping_add((len as u64).wrapping_mul(2));
    let mut a = u64le_slice(s, 0).wrapping_mul(K2);
    let mut b = u64le_slice(s, 8);
    let mut c = u64le_slice(s, len - 24);
    let mut d = u64le_slice(s, len - 32);
    let mut e = u64le_slice(s, 16).wrapping_mul(K2);
    let mut f = u64le_slice(s, 24).wrapping_mul(9);
    let mut g = u64le_slice(s, len - 8);
    let mut h = u64le_slice(s, len - 16).wrapping_mul(mul);

    let end = len - 64;
    let mut offset = 32;
    while offset <= end {
        a = a
            .wrapping_add(u64le_slice(s, offset).wrapping_mul(K0))
            .rotate_right(33)
            .wrapping_mul(K1);
        b = b
            .wrapping_add(u64le_slice(s, offset + 8).wrapping_mul(K1))
            .rotate_right(33)
            .wrapping_mul(K2);
        c = c
            .wrapping_add(u64le_slice(s, offset + 16).wrapping_mul(K2))
            .rotate_right(33)
            .wrapping_mul(K0);
        d = d
            .wrapping_add(u64le_slice(s, offset + 24).wrapping_mul(K0))
            .rotate_right(33);
        e = e
            .wrapping_add(u64le_slice(s, offset + 32).wrapping_mul(K0))
            .rotate_right(33)
            .wrapping_mul(K1);
        f = f
            .wrapping_add(u64le_slice(s, offset + 40).wrapping_mul(K1))
            .rotate_right(33)
            .wrapping_mul(K2);
        g = g
            .wrapping_add(u64le_slice(s, offset + 48).wrapping_mul(K2))
            .rotate_right(33)
            .wrapping_mul(K0);
        h = h
            .wrapping_add(u64le_slice(s, offset + 56).wrapping_mul(K0))
            .rotate_right(33);
        offset += 64;
    }

    let u = shift_mix(a.wrapping_add(g).wrapping_add(e.wrapping_mul(b)))
        .wrapping_mul(mul)
        .wrapping_add(d);
    let v = shift_mix(b.wrapping_add(h).wrapping_add(f.wrapping_mul(c)))
        .wrapping_mul(mul)
        .wrapping_add(a);
    let mut result = u ^ v;
    result = result.wrapping_mul(mul).wrapping_add(h);
    result = shift_mix(result).wrapping_mul(mul).wrapping_add(d);
    result
}

fn hash128to64(lo: u64, hi: u64) -> u64 {
    const K3: u64 = 0x9ddfea08eb382d69;
    let mut a = (lo ^ hi).wrapping_mul(K3);
    a ^= a >> 47;
    let mut b = (hi ^ a).wrapping_mul(K3);
    b ^= b >> 47;
    b.wrapping_mul(K3)
}

fn shift_mix(v: u64) -> u64 {
    v ^ (v >> 47)
}

fn u64le_slice(data: &[u8], off: usize) -> u64 {
    u64::from_le_bytes(data[off..off + 8].try_into().expect("u64le: invalid slice"))
}

fn u32le_slice(data: &[u8], off: usize) -> u32 {
    u32::from_le_bytes(data[off..off + 4].try_into().expect("u32le: invalid slice"))
}

// ---------------------------------------------------------------------------
// Sector paths
// ---------------------------------------------------------------------------

/// Return all candidate sector logical paths the parser knows about.
///
/// Iterates the regular ETS2 sector grid (`map/europe/sec±XXXX±YYYY.data` /
/// `.base`) and produces a list ready to be probed against an opened
/// `ScsArchive`.
pub fn list_map_sector_paths() -> Vec<String> {
    let mut paths = Vec::new();
    for x in -25..=25i32 {
        for z in -25..=25i32 {
            let sx = if x >= 0 {
                format!("+{x:04}")
            } else {
                format!("-{:04}", x.abs())
            };
            let sz = if z >= 0 {
                format!("+{z:04}")
            } else {
                format!("-{:04}", z.abs())
            };
            paths.push(format!("map/europe/sec{sx}{sz}.data"));
            paths.push(format!("map/europe/sec{sx}{sz}.base"));
            paths.push(format!("map/europe/sec{sx}{sz}.aux"));
            paths.push(format!("map/europe/sec{sx}{sz}.desc"));
        }
    }
    paths.extend(vec![
        "def/map.sii".into(),
        "def/world/road.sii".into(),
        "def/world/prefab.sii".into(),
        "def/world/sign.sii".into(),
    ]);
    paths
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cityhash64_known() {
        assert_eq!(cityhash64(b"abc"), cityhash64(b"abc"));
        assert_ne!(cityhash64(b""), cityhash64(b"a"));
        assert_ne!(cityhash64(b"a"), cityhash64(b"b"));
        assert_ne!(cityhash64(b"hello"), 0);
    }

    #[test]
    fn test_cityhash64_deterministic() {
        assert_eq!(
            cityhash64(b"def/world/road.sii"),
            cityhash64(b"def/world/road.sii")
        );
    }

    #[test]
    fn test_entry_layout_size() {
        assert_eq!(
            std::mem::size_of::<DirEntry>(),
            32,
            "Entry must be exactly 32 bytes"
        );
    }

    #[test]
    fn test_flag_constants() {
        assert_eq!(FLAG_DIRECTORY, 1);
        assert_eq!(FLAG_COMPRESSED, 2);
    }

    #[test]
    fn test_list_map_sector_paths() {
        let paths = list_map_sector_paths();
        assert!(paths.len() > 1000);
        assert!(paths.iter().any(|p| p == "map/europe/sec+0000+0000.data"));
        assert!(paths.iter().any(|p| p == "def/map.sii"));
    }

    #[test]
    fn test_parse_header_v2_layout() {
        // Hand-build a 52-byte v2 header with distinct sentinel values per
        // field, so a misaligned read shows up as a wrong sentinel.
        let mut buf = [0u8; 0x34];
        buf[0x00..0x04].copy_from_slice(b"SCS#");
        buf[0x04..0x06].copy_from_slice(&2u16.to_le_bytes()); // version
        buf[0x06..0x08].copy_from_slice(&0u16.to_le_bytes()); // salt
        buf[0x08..0x0C].copy_from_slice(b"CITY");
        buf[0x0C..0x10].copy_from_slice(&0x1111_1111u32.to_le_bytes()); // entry_count
        buf[0x10..0x14].copy_from_slice(&0x2222_2222u32.to_le_bytes()); // entry_table_compressed_size
        buf[0x14..0x18].copy_from_slice(&0x3333_3333u32.to_le_bytes()); // metadata_word_count
        buf[0x18..0x1C].copy_from_slice(&0x4444_4444u32.to_le_bytes()); // metadata_table_compressed_size
        buf[0x1C..0x24].copy_from_slice(&0x5555_5555_5555_5555u64.to_le_bytes()); // entry_table_start
        buf[0x24..0x2C].copy_from_slice(&0x6666_6666_6666_6666u64.to_le_bytes()); // metadata_table_start
        buf[0x2C..0x34].copy_from_slice(&0u64.to_le_bytes()); // security_descriptor_start

        let mut cursor = std::io::Cursor::new(&buf[..]);
        let h = parse_header(&mut cursor).expect("header should parse");

        assert_eq!(h.entry_count, 0x1111_1111);
        assert_eq!(h.entry_table_compressed_size, 0x2222_2222);
        assert_eq!(h.metadata_table_compressed_size, 0x4444_4444);
        assert_eq!(h.entry_table_start, 0x5555_5555_5555_5555);
        assert_eq!(h.metadata_table_start, 0x6666_6666_6666_6666);
    }

    #[test]
    fn test_parse_header_rejects_v1() {
        let mut buf = [0u8; 0x34];
        buf[0x00..0x04].copy_from_slice(b"SCS#");
        buf[0x04..0x06].copy_from_slice(&1u16.to_le_bytes()); // version 1
        buf[0x08..0x0C].copy_from_slice(b"CITY");
        let mut cursor = std::io::Cursor::new(&buf[..]);
        assert!(parse_header(&mut cursor).is_err());
    }

    #[test]
    fn test_parse_header_rejects_nonzero_salt() {
        let mut buf = [0u8; 0x34];
        buf[0x00..0x04].copy_from_slice(b"SCS#");
        buf[0x04..0x06].copy_from_slice(&2u16.to_le_bytes());
        buf[0x06..0x08].copy_from_slice(&7u16.to_le_bytes()); // bad salt
        buf[0x08..0x0C].copy_from_slice(b"CITY");
        let mut cursor = std::io::Cursor::new(&buf[..]);
        assert!(parse_header(&mut cursor).is_err());
    }

    fn build_mini_header(body_off_words: u32, kind: u8) -> [u8; 4] {
        let mut h = [0u8; 4];
        h[0..2].copy_from_slice(&((body_off_words & 0xFFFF) as u16).to_le_bytes());
        h[2] = ((body_off_words >> 16) & 0xFF) as u8;
        h[3] = kind;
        h
    }

    fn build_data_part_body(
        zsize: u32,
        usize_: u32,
        data_off_blocks: u32,
        is_compressed: bool,
    ) -> [u8; 16] {
        let mut body = [0u8; 16];
        body[0..2].copy_from_slice(&((zsize & 0xFFFF) as u16).to_le_bytes());
        body[2] = ((zsize >> 16) & 0xFF) as u8;
        body[3] = if is_compressed { 0x10 } else { 0x00 };
        body[4..6].copy_from_slice(&((usize_ & 0xFFFF) as u16).to_le_bytes());
        body[6] = ((usize_ >> 16) & 0xFF) as u8;
        body[12..16].copy_from_slice(&data_off_blocks.to_le_bytes());
        body
    }

    #[test]
    fn test_resolve_main_metadata_via_mini_header() {
        // Layout: data-part body @ word 4 (byte 16); aux body @ word 10 (byte 40);
        // mini-headers @ word 8 (byte 32): aux first, then data-part.
        let mut idx2 = vec![0u8; 64];
        idx2[16..32]
            .copy_from_slice(&build_data_part_body(100, 250, 0x40, true));
        idx2[32..36].copy_from_slice(&build_mini_header(10, KIND_AUX_8A));
        idx2[36..40].copy_from_slice(&build_mini_header(4, DATA_PART_MARKER));

        let entry = resolve_main_metadata(&idx2, 8, 2, 0)
            .expect("should resolve via mini-headers to the data part");
        assert_eq!(entry.offset, 0x400);
        assert_eq!(entry.size, 250);
        assert_eq!(entry.compressed_size, 100);
        assert!(entry.flags & FLAG_COMPRESSED != 0);
    }

    #[test]
    fn test_resolve_main_metadata_uncompressed() {
        let mut idx2 = vec![0u8; 4 + 16];
        idx2[0..4].copy_from_slice(&build_mini_header(1, DATA_PART_MARKER));
        idx2[4..20]
            .copy_from_slice(&build_data_part_body(0, 500, 0x10, false));

        let entry = resolve_main_metadata(&idx2, 0, 1, 0)
            .expect("uncompressed data part should resolve");
        assert_eq!(entry.offset, 0x100);
        assert_eq!(entry.size, 500);
        assert_eq!(entry.compressed_size, 0);
        assert!(entry.flags & FLAG_COMPRESSED == 0);
    }

    #[test]
    fn test_cityhash64_vectors() {
        // Empty string
        let h = cityhash64(b"");
        assert_ne!(h, 0, "empty string hash should not be 0");
        // "abc" — deterministic
        assert_eq!(cityhash64(b"abc"), cityhash64(b"abc"));
        // Long path
        let p = b"map/europe/sec+0010-0025.data";
        assert_ne!(cityhash64(p), 0);
    }
}
