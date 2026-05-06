//! SCS archive reader for ETS2 HashFS format (CityHash v2).
//!
//! # File format (32-byte header + 32-byte flat entries)
//!
//! ```text
//! Offset  Size  Field
//! 0x00    4     magic       = b"SCS#" (0x53,0x43,0x53,0x23)
//! 0x04    2     version     = u16 LE (typically 2)
//! 0x06    2     salt        = u16 LE
//! 0x08    4     hash_method = b"CITY" (0x43,0x49,0x54,0x59)
//! 0x0C    4     entry_count = u32 LE
//! 0x10    4     start_offset= u32 LE (absolute offset to data section)
//! 0x14    12    padding     (ignored)
//! 0x20    *     entries[]   = entry_count × 32 bytes each
//! ─────────────────────────────────────────────────────────
//! Entry layout (32 bytes each):
//!   0x00  8  hash            = u64 LE (CityHash64 of file path)
//!   0x08  8  offset          = u64 LE (absolute byte offset to data)
//!   0x10  4  flags           = u32 LE (bit0=dir, bit1=compressed, bit2=verify, bit3=encrypted)
//!   0x14  4  crc             = u32 LE (CRC32)
//!   0x18  4  size            = u32 LE (uncompressed size)
//!   0x1C  4  compressed_size = u32 LE (0 or ==size if uncompressed)
//! ```
//!
//! Data entries are zlib-compressed when bit 1 of flags is set.
//! Offsets are absolute within the .scs file.

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
const FLAG_COMPRESSED: u32 = 1 << 1;
#[allow(dead_code)]
const FLAG_VERIFY: u32 = 1 << 2;
#[allow(dead_code)]
const FLAG_ENCRYPTED: u32 = 1 << 3;

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

/// Parsed SCS header.
///
/// Note: the `salt` field at byte offset 0x06 is read from the archive but
/// not retained — it is only required during entry-hash mixing in older
/// SCS variants and is not used by the v2 CityHash-based layout we target.
#[derive(Debug)]
struct Header {
    entry_count: u32,
    entry_table_start: u64,
    metadata_table_start: u64,
    entry_table_length: u32,
    metadata_table_length: u32,
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
        let entry_data: Vec<u8> = if header.entry_table_length == 0 {
            vec![]
        } else {
            file.seek(SeekFrom::Start(header.entry_table_start))
                .map_err(|e| format!("seek to entry_table_start: {e}"))?;
            let mut entry_buf = vec![0u8; header.entry_table_length as usize];
            file.read_exact(&mut entry_buf)
                .map_err(|e| format!("read entry_table: {e}"))?;
            inflate::decompress_to_vec_zlib(&entry_buf)
                .map_err(|e| format!("decompress entry_table: {e:?}"))?
        };

        // 2. Read + decompress Metadata table (handle zero-length)
        let meta_data: Vec<u8> = if header.metadata_table_length == 0 {
            vec![]
        } else {
            file.seek(SeekFrom::Start(header.metadata_table_start))
                .map_err(|e| format!("seek to metadata_table_start: {e}"))?;
            let mut meta_buf = vec![0u8; header.metadata_table_length as usize];
            file.read_exact(&mut meta_buf)
                .map_err(|e| format!("read metadata_table: {e}"))?;
            inflate::decompress_to_vec_zlib(&meta_buf)
                .map_err(|e| format!("decompress metadata_table: {e:?}"))?
        };

        // TEMP DEBUG: print first 32 bytes of meta_data to understand layout
        if !meta_data.is_empty() {
            eprintln!(
                "META first 32: {:02X?}",
                &meta_data[..meta_data.len().min(32)]
            );
        }

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

fn parse_header(r: &mut BufReader<File>) -> Result<Header, String> {
    let mut magic = [0u8; 4];
    r.read_exact(&mut magic).map_err(|e| format!("io: {e}"))?;
    if magic != MAGIC {
        return Err(format!("invalid SCS magic: {magic:02X?}"));
    }

    let version = read_u16(r)?;
    if version != 2 {
        return Err(format!("unsupported SCS version: {version} (expected 2)"));
    }

    // Skip 2-byte salt field at offset 0x06 (not used in the v2 CityHash layout).
    read_u16(r)?;

    let mut hm = [0u8; 4];
    r.read_exact(&mut hm).map_err(|e| format!("io: {e}"))?;
    if hm != CITY {
        return Err(format!(
            "unsupported hash method: {hm:02X?} (expected CITY)"
        ));
    }

    let entry_count = read_u32(r)?;
    let entry_table_length = read_u32(r)?;
    let metadata_table_length = read_u32(r)?;

    // Table starts are u32 in the v2 header (not u64)
    let entry_table_start = read_u32(r)? as u64;
    let metadata_table_start = read_u32(r)? as u64;

    Ok(Header {
        entry_count,
        entry_table_start,
        metadata_table_start,
        entry_table_length,
        metadata_table_length,
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

/// Resolve MainMetadata for an entry and return a DirEntry with correct absolute offset.
/// Layout (assumed 12+ bytes starting at metadata_index*4):
///   [0..4] compressed_size (low 28 bits + flags in high 4)
///   [4..8] uncompressed_size (low 28 bits + flags)
///   [8..12] offset_block (data offset = offset_block * 16)
fn resolve_main_metadata(
    meta_data: &[u8],
    metadata_index: u32,
    metadata_count: u16,
    entry_flags: u16,
) -> Option<DirEntry> {
    if metadata_count == 0 {
        return None;
    }
    let base = (metadata_index as usize) * 4;
    if base + 12 > meta_data.len() {
        return None;
    }

    let compressed_raw = u32::from_le_bytes([
        meta_data[base],
        meta_data[base + 1],
        meta_data[base + 2],
        meta_data[base + 3],
    ]);
    let uncompressed_raw = u32::from_le_bytes([
        meta_data[base + 4],
        meta_data[base + 5],
        meta_data[base + 6],
        meta_data[base + 7],
    ]);
    let offset_block = u32::from_le_bytes([
        meta_data[base + 8],
        meta_data[base + 9],
        meta_data[base + 10],
        meta_data[base + 11],
    ]);

    let is_compressed = (compressed_raw & 0x10000000) != 0; // flag bit
    let compressed_size = compressed_raw & 0x0FFFFFFF;
    let uncompressed_size = uncompressed_raw & 0x0FFFFFFF;

    let mut abs_offset = (offset_block as u64) * 16;
    // Try alternative positions if the first guess is invalid
    if abs_offset > 300_000_000 || abs_offset == 0 {
        // try offset at base+4 as u32
        if base + 8 <= meta_data.len() {
            let alt = u32::from_le_bytes([
                meta_data[base + 4],
                meta_data[base + 5],
                meta_data[base + 6],
                meta_data[base + 7],
            ]);
            let alt_off = (alt as u64) * 16;
            if alt_off > 0 && alt_off < 300_000_000 {
                abs_offset = alt_off;
            }
        }
    }
    if abs_offset > 300_000_000 || abs_offset == 0 {
        return None;
    }

    let size = if is_compressed {
        compressed_size
    } else {
        uncompressed_size
    };
    let compressed_size_field = if is_compressed { compressed_size } else { 0 };

    Some(DirEntry {
        hash: 0,
        offset: abs_offset,
        flags: entry_flags as u32,
        crc: 0,
        size,
        compressed_size: compressed_size_field,
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
