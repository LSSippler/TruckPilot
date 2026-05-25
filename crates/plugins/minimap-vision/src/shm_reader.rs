//! SHM reader for the TruckPilotMinimapLine region.
//!
//! Header layout (64 bytes, little-endian):
//! ```text
//!     magic        char[4]   "TPM1"
//!     version      u32       1
//!     seq          u64       sequence counter (odd=writing, even=committed)
//!     timestamp_ms u64       UNIX epoch milliseconds
//!     point_count  u32       number of (f32, f32) in payload (max 256)
//!     confidence   f32       detection confidence [0.0, 1.0]
//!     reserved     u8[32]    zero-padded
//! ```
//! Payload: `point_count × (f32 x, f32 y)` — pixel coords within ROI.

pub const HEADER_BYTES: usize = 64;
pub const HEADER_MAGIC: [u8; 4] = *b"TPM1";
pub const HEADER_VERSION: u32 = 1;
pub const MAX_POINTS: usize = 256;
pub const POINT_BYTES: usize = 8; // two f32
pub const PAYLOAD_BYTES: usize = MAX_POINTS * POINT_BYTES;
pub const BUFFER_BYTES: usize = HEADER_BYTES + PAYLOAD_BYTES;
pub const DEFAULT_SHM_NAME: &str = "TruckPilotMinimapLine";

/// Sequence-lock retry budget before giving up on one tick.
pub const SEQUENCE_LOCK_RETRIES: u32 = 64;

#[derive(Debug, Clone)]
pub struct MinimapHeader {
    pub seq: u64,
    pub timestamp_ms: u64,
    pub point_count: u32,
    pub confidence: f32,
}

impl MinimapHeader {
    pub fn parse(buf: &[u8]) -> Option<Self> {
        if buf.len() < HEADER_BYTES {
            return None;
        }
        let magic: [u8; 4] = buf[0..4].try_into().ok()?;
        if magic != HEADER_MAGIC {
            return None;
        }
        let version = u32::from_le_bytes(buf[4..8].try_into().ok()?);
        if version != HEADER_VERSION {
            return None;
        }
        let seq = u64::from_le_bytes(buf[8..16].try_into().ok()?);
        let timestamp_ms = u64::from_le_bytes(buf[16..24].try_into().ok()?);
        let point_count = u32::from_le_bytes(buf[24..28].try_into().ok()?);
        let confidence = f32::from_le_bytes(buf[28..32].try_into().ok()?);
        Some(Self { seq, timestamp_ms, point_count, confidence })
    }
}

#[derive(Debug)]
pub struct MinimapFrame {
    pub header: MinimapHeader,
    pub points: Vec<(f32, f32)>,
}

#[derive(Debug)]
pub enum ReadOutcome {
    Frame(MinimapFrame),
    NoFrame,           // seq==0 (nothing published yet)
    WriterBusy,        // sequence-lock budget exhausted
    InvalidHeader,     // bad magic/version or buffer too short
    PayloadOverflow,   // point_count exceeds MAX_POINTS
}

pub fn read_frame(buf: &[u8]) -> ReadOutcome {
    if buf.len() < HEADER_BYTES {
        return ReadOutcome::InvalidHeader;
    }

    for _ in 0..SEQUENCE_LOCK_RETRIES {
        let pre = match MinimapHeader::parse(&buf[..HEADER_BYTES]) {
            Some(h) => h,
            None => return ReadOutcome::InvalidHeader,
        };

        if pre.seq == 0 {
            return ReadOutcome::NoFrame;
        }
        if pre.seq & 1 == 1 {
            // write in progress
            std::hint::spin_loop();
            continue;
        }

        let count = pre.point_count as usize;
        if count > MAX_POINTS {
            return ReadOutcome::PayloadOverflow;
        }

        let payload_end = HEADER_BYTES + count * POINT_BYTES;
        if payload_end > buf.len() {
            return ReadOutcome::PayloadOverflow;
        }

        let mut points = Vec::with_capacity(count);
        for i in 0..count {
            let off = HEADER_BYTES + i * POINT_BYTES;
            let x = f32::from_le_bytes(buf[off..off + 4].try_into().unwrap());
            let y = f32::from_le_bytes(buf[off + 4..off + 8].try_into().unwrap());
            points.push((x, y));
        }

        // Verify seq didn't change while we read the payload.
        let post = match MinimapHeader::parse(&buf[..HEADER_BYTES]) {
            Some(h) => h,
            None => return ReadOutcome::InvalidHeader,
        };
        if post.seq == pre.seq {
            return ReadOutcome::Frame(MinimapFrame { header: pre, points });
        }
        std::hint::spin_loop();
    }

    ReadOutcome::WriterBusy
}

/// Open the named SHM region read-only and return a `'static` slice.
#[cfg(windows)]
pub fn map_shm(name: &str, size: usize) -> Result<&'static [u8], String> {
    use windows::core::PCWSTR;
    use windows::Win32::Foundation::CloseHandle;
    use windows::Win32::System::Memory::{MapViewOfFile, OpenFileMappingW, FILE_MAP_READ};

    let wide: Vec<u16> = name.encode_utf16().chain(std::iter::once(0)).collect();
    unsafe {
        let handle = OpenFileMappingW(FILE_MAP_READ.0, false, PCWSTR(wide.as_ptr()))
            .map_err(|e| format!("OpenFileMappingW({name}): {e}"))?;
        if handle.is_invalid() {
            return Err(format!("OpenFileMappingW returned invalid handle for {name}"));
        }
        let view = MapViewOfFile(handle, FILE_MAP_READ, 0, 0, size);
        let _ = CloseHandle(handle);
        if view.Value.is_null() {
            return Err(format!("MapViewOfFile failed for {name}"));
        }
        // SAFETY: producer never shrinks mapping; process exit unmaps.
        let slice = std::slice::from_raw_parts(view.Value as *const u8, size);
        Ok(slice)
    }
}

#[cfg(not(windows))]
pub fn map_shm(_name: &str, _size: usize) -> Result<&'static [u8], String> {
    Err("minimap-vision SHM backend is Windows-only".to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write_header(buf: &mut [u8], seq: u64, ts_ms: u64, count: u32, conf: f32) {
        buf[0..4].copy_from_slice(&HEADER_MAGIC);
        buf[4..8].copy_from_slice(&HEADER_VERSION.to_le_bytes());
        buf[8..16].copy_from_slice(&seq.to_le_bytes());
        buf[16..24].copy_from_slice(&ts_ms.to_le_bytes());
        buf[24..28].copy_from_slice(&count.to_le_bytes());
        buf[28..32].copy_from_slice(&conf.to_le_bytes());
    }

    #[test]
    fn no_frame_when_seq_zero() {
        let mut buf = vec![0u8; BUFFER_BYTES];
        write_header(&mut buf, 0, 0, 0, 0.0);
        assert!(matches!(read_frame(&buf), ReadOutcome::NoFrame));
    }

    #[test]
    fn invalid_header_on_bad_magic() {
        let mut buf = vec![0u8; BUFFER_BYTES];
        write_header(&mut buf, 2, 0, 0, 0.0);
        buf[0] = b'X';
        assert!(matches!(read_frame(&buf), ReadOutcome::InvalidHeader));
    }

    #[test]
    fn writer_busy_on_odd_seq() {
        let mut buf = vec![0u8; BUFFER_BYTES];
        write_header(&mut buf, 1, 0, 0, 0.0);
        assert!(matches!(read_frame(&buf), ReadOutcome::WriterBusy));
    }

    #[test]
    fn reads_committed_frame_with_points() {
        let mut buf = vec![0u8; BUFFER_BYTES];
        let count = 3u32;
        write_header(&mut buf, 2, 1_700_000_000_000, count, 0.9);
        // Write 3 (f32, f32) points.
        let points = [(1.0f32, 2.0f32), (3.0, 4.0), (5.0, 6.0)];
        for (i, (x, y)) in points.iter().enumerate() {
            let off = HEADER_BYTES + i * POINT_BYTES;
            buf[off..off + 4].copy_from_slice(&x.to_le_bytes());
            buf[off + 4..off + 8].copy_from_slice(&y.to_le_bytes());
        }
        match read_frame(&buf) {
            ReadOutcome::Frame(f) => {
                assert_eq!(f.header.seq, 2);
                assert_eq!(f.points.len(), 3);
                assert!((f.header.confidence - 0.9).abs() < 0.001);
            }
            other => panic!("expected Frame, got {other:?}"),
        }
    }

    #[test]
    fn payload_overflow_on_excessive_count() {
        let mut buf = vec![0u8; BUFFER_BYTES];
        write_header(&mut buf, 2, 0, (MAX_POINTS as u32) + 1, 0.0);
        assert!(matches!(read_frame(&buf), ReadOutcome::PayloadOverflow));
    }
}
