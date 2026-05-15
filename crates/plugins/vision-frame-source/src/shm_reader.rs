//! Sequence-lock SHM frame reader extracted from `crates/diag/src/bin/shm_frame_reader.rs`.
//!
//! Header layout (64 bytes, little-endian, packed):
//! ```text
//!     magic        char[4]    "TPF1"
//!     version      u32        1
//!     frame_id     u64        sequence counter (odd = writing, even = committed)
//!     timestamp_us u64        producer monotonic micros at commit
//!     width        u32
//!     height       u32
//!     jpeg_size    u32
//!     reserved     u8[28]
//! ```
//! Payload immediately follows the header.

pub const HEADER_BYTES: usize = 64;
pub const HEADER_MAGIC: [u8; 4] = *b"TPF1";
pub const HEADER_VERSION: u32 = 1;
pub const DEFAULT_BUFFER_BYTES: usize = 64 + 2 * 1024 * 1024;
pub const DEFAULT_SHM_NAME: &str = "TruckPilotFrame";

/// Retry attempts for the sequence-lock loop inside `read_frame`.
/// Bumped from 64 (diag-binary PoC) to 128 for plugin use; producer may
/// be mid-write when scheduler invokes the tick.
pub const SEQUENCE_LOCK_RETRIES: u32 = 128;

#[derive(Debug, Clone, Copy)]
pub struct Header {
    pub magic: [u8; 4],
    pub version: u32,
    pub frame_id: u64,
    pub timestamp_us: u64,
    pub width: u32,
    pub height: u32,
    pub jpeg_size: u32,
}

impl Header {
    pub fn parse(buf: &[u8]) -> Option<Header> {
        if buf.len() < HEADER_BYTES {
            return None;
        }
        let mut magic = [0u8; 4];
        magic.copy_from_slice(&buf[0..4]);
        Some(Header {
            magic,
            version: u32::from_le_bytes(buf[4..8].try_into().ok()?),
            frame_id: u64::from_le_bytes(buf[8..16].try_into().ok()?),
            timestamp_us: u64::from_le_bytes(buf[16..24].try_into().ok()?),
            width: u32::from_le_bytes(buf[24..28].try_into().ok()?),
            height: u32::from_le_bytes(buf[28..32].try_into().ok()?),
            jpeg_size: u32::from_le_bytes(buf[32..36].try_into().ok()?),
        })
    }

    pub fn is_valid_magic(&self) -> bool {
        self.magic == HEADER_MAGIC && self.version == HEADER_VERSION
    }
}

/// Outcome of a single `read_frame` attempt.
#[derive(Debug)]
pub enum ReadOutcome<'a> {
    /// Sequence-locked, magic/version checked, payload sliced.
    Frame { header: Header, payload: &'a [u8] },
    /// Header missing or malformed (magic / version / truncated buffer).
    InvalidHeader,
    /// Buffer too small for declared `jpeg_size`. Producer config drift.
    PayloadOverflow,
    /// Sequence-lock retry budget exhausted (writer is stuck mid-write
    /// or producer is silent). Caller should count toward producer
    /// health budget.
    SequenceLockExhausted,
}

/// Read one committed frame from the shared buffer.
///
/// Uses the sequence-lock protocol: `frame_id` is incremented to an odd
/// value before a write and to an even value after. A reader observes a
/// stable even `frame_id` before *and* after slicing the payload.
pub fn read_frame(buf: &[u8]) -> ReadOutcome<'_> {
    if buf.len() < HEADER_BYTES {
        return ReadOutcome::InvalidHeader;
    }
    for _ in 0..SEQUENCE_LOCK_RETRIES {
        let pre = match Header::parse(&buf[..HEADER_BYTES]) {
            Some(h) => h,
            None => return ReadOutcome::InvalidHeader,
        };
        if !pre.is_valid_magic() {
            return ReadOutcome::InvalidHeader;
        }
        if pre.frame_id == 0 || pre.frame_id & 1 == 1 {
            // not yet published or write in progress
            std::hint::spin_loop();
            continue;
        }
        let size = pre.jpeg_size as usize;
        if size == 0 {
            std::hint::spin_loop();
            continue;
        }
        if size > buf.len().saturating_sub(HEADER_BYTES) {
            return ReadOutcome::PayloadOverflow;
        }
        let payload = &buf[HEADER_BYTES..HEADER_BYTES + size];
        let post = match Header::parse(&buf[..HEADER_BYTES]) {
            Some(h) => h,
            None => return ReadOutcome::InvalidHeader,
        };
        if post.frame_id == pre.frame_id {
            return ReadOutcome::Frame {
                header: pre,
                payload,
            };
        }
        // writer overlapped; spin
        std::hint::spin_loop();
    }
    ReadOutcome::SequenceLockExhausted
}

/// Open the named SHM region read-only and return a `'static` slice.
///
/// On non-Windows platforms this always returns an error (SHM is
/// Windows-only for now).
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
            return Err(format!(
                "OpenFileMappingW returned invalid handle for {name}"
            ));
        }
        let view = MapViewOfFile(handle, FILE_MAP_READ, 0, 0, size);
        let _ = CloseHandle(handle);
        if view.Value.is_null() {
            return Err(format!("MapViewOfFile failed for {name}"));
        }
        // SAFETY: producer never shrinks the mapping; reader treats it as read-only.
        // The mapping is intentionally leaked for the lifetime of the daemon;
        // process exit unmaps it. UnmapViewOfFile is intentionally not called
        // because the slice's `'static` lifetime would be unsound otherwise.
        let slice = std::slice::from_raw_parts(view.Value as *const u8, size);
        Ok(slice)
    }
}

#[cfg(not(windows))]
pub fn map_shm(_name: &str, _size: usize) -> Result<&'static [u8], String> {
    Err("vision-frame-source SHM backend is Windows-only".to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a valid header into the front of `buf`. Caller must ensure
    /// `buf.len() >= HEADER_BYTES`.
    fn write_header(
        buf: &mut [u8],
        frame_id: u64,
        timestamp_us: u64,
        width: u32,
        height: u32,
        jpeg_size: u32,
    ) {
        buf[0..4].copy_from_slice(&HEADER_MAGIC);
        buf[4..8].copy_from_slice(&HEADER_VERSION.to_le_bytes());
        buf[8..16].copy_from_slice(&frame_id.to_le_bytes());
        buf[16..24].copy_from_slice(&timestamp_us.to_le_bytes());
        buf[24..28].copy_from_slice(&width.to_le_bytes());
        buf[28..32].copy_from_slice(&height.to_le_bytes());
        buf[32..36].copy_from_slice(&jpeg_size.to_le_bytes());
        // 28 reserved bytes left as-is.
    }

    #[test]
    fn parse_returns_none_for_too_short_buffer() {
        assert!(Header::parse(&[0u8; 32]).is_none());
    }

    #[test]
    fn parse_round_trips_known_values() {
        let mut buf = [0u8; 128];
        write_header(&mut buf, 4, 1_700_000_000_000_000, 1920, 1080, 42);
        let h = Header::parse(&buf).expect("parse");
        assert_eq!(h.magic, HEADER_MAGIC);
        assert_eq!(h.version, HEADER_VERSION);
        assert_eq!(h.frame_id, 4);
        assert_eq!(h.timestamp_us, 1_700_000_000_000_000);
        assert_eq!(h.width, 1920);
        assert_eq!(h.height, 1080);
        assert_eq!(h.jpeg_size, 42);
        assert!(h.is_valid_magic());
    }

    #[test]
    fn read_frame_returns_invalid_header_for_bad_magic() {
        let mut buf = [0u8; 256];
        write_header(&mut buf, 2, 0, 4, 4, 16);
        buf[0] = b'X'; // corrupt magic
        assert!(matches!(read_frame(&buf), ReadOutcome::InvalidHeader));
    }

    #[test]
    fn read_frame_retries_when_frame_id_odd() {
        // frame_id=1 → odd, never converges → SequenceLockExhausted
        let mut buf = [0u8; 256];
        write_header(&mut buf, 1, 0, 4, 4, 16);
        assert!(matches!(
            read_frame(&buf),
            ReadOutcome::SequenceLockExhausted
        ));
    }

    #[test]
    fn read_frame_returns_frame_for_committed_buffer() {
        let mut buf = vec![0u8; 256];
        write_header(&mut buf, 2, 0, 4, 4, 16);
        for (i, b) in buf[HEADER_BYTES..HEADER_BYTES + 16].iter_mut().enumerate() {
            *b = i as u8;
        }
        match read_frame(&buf) {
            ReadOutcome::Frame { header, payload } => {
                assert_eq!(header.frame_id, 2);
                assert_eq!(payload.len(), 16);
                assert_eq!(payload[0], 0);
                assert_eq!(payload[15], 15);
            }
            other => panic!("expected Frame, got {other:?}"),
        }
    }

    #[test]
    fn read_frame_rejects_payload_overflow() {
        let mut buf = vec![0u8; 128];
        let overflow_size = (buf.len() - HEADER_BYTES + 1) as u32;
        // jpeg_size larger than buffer minus header → overflow
        write_header(&mut buf, 2, 0, 1, 1, overflow_size);
        assert!(matches!(read_frame(&buf), ReadOutcome::PayloadOverflow));
    }

    #[test]
    fn read_frame_skips_zero_jpeg_size() {
        let mut buf = vec![0u8; 256];
        write_header(&mut buf, 2, 0, 4, 4, 0);
        assert!(matches!(
            read_frame(&buf),
            ReadOutcome::SequenceLockExhausted
        ));
    }
}
