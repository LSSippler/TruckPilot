//! TruckPilot Phase 6.5c.1 — SHM frame reader.
//!
//! Opens the named shared-memory region produced by `vision-pipeline-capture`
//! (`Local\TruckPilotFrame` on Windows), reads each committed frame using the
//! sequence-lock protocol, decodes its JPEG payload with the `image` crate,
//! and prints per-frame diagnostics plus a 30 s rollup.
//!
//! Header layout (64 bytes, little-endian, packed):
//!     magic        char[4]    "TPF1"
//!     version      u32        1
//!     frame_id     u64        sequence counter (odd = writing, even = committed)
//!     timestamp_us u64        producer monotonic micros at commit
//!     width        u32
//!     height       u32
//!     jpeg_size    u32
//!     reserved     u8[28]
//!
//! Payload immediately follows the header.

use std::time::{Duration, Instant};

const HEADER_BYTES: usize = 64;
const HEADER_MAGIC: [u8; 4] = *b"TPF1";
const HEADER_VERSION: u32 = 1;
const DEFAULT_BUFFER_BYTES: usize = 64 + 2 * 1024 * 1024;
const DEFAULT_SHM_NAME: &str = "TruckPilotFrame";
const DEFAULT_DURATION_SECS: u64 = 30;

#[derive(Debug, Clone, Copy)]
struct Header {
    magic: [u8; 4],
    version: u32,
    frame_id: u64,
    timestamp_us: u64,
    width: u32,
    height: u32,
    jpeg_size: u32,
}

impl Header {
    fn parse(buf: &[u8]) -> Option<Header> {
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
}

struct Cli {
    shm_name: String,
    duration_secs: u64,
    buffer_bytes: usize,
    decode: bool,
}

fn parse_cli() -> Cli {
    let mut shm_name = DEFAULT_SHM_NAME.to_string();
    let mut duration_secs = DEFAULT_DURATION_SECS;
    let mut buffer_bytes = DEFAULT_BUFFER_BYTES;
    let mut decode = true;

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--shm-name" => shm_name = args.next().expect("--shm-name needs value"),
            "--duration" => {
                duration_secs = args
                    .next()
                    .expect("--duration needs value")
                    .parse()
                    .expect("--duration must be u64");
            }
            "--buffer-bytes" => {
                buffer_bytes = args
                    .next()
                    .expect("--buffer-bytes needs value")
                    .parse()
                    .expect("--buffer-bytes must be usize");
            }
            "--no-decode" => decode = false,
            "--help" | "-h" => {
                println!(
                    "shm-frame-reader [--shm-name NAME] [--duration SECS] [--buffer-bytes N] [--no-decode]"
                );
                std::process::exit(0);
            }
            other => panic!("unknown arg: {other}"),
        }
    }

    Cli {
        shm_name,
        duration_secs,
        buffer_bytes,
        decode,
    }
}

#[cfg(windows)]
fn map_shm(name: &str, size: usize) -> Result<&'static [u8], String> {
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
        // close the handle - mapping survives until UnmapViewOfFile
        let _ = CloseHandle(handle);
        if view.Value.is_null() {
            return Err(format!("MapViewOfFile failed for {name}"));
        }
        // SAFETY: producer never shrinks the mapping; reader treats it as read-only.
        // We intentionally leak the mapping for the lifetime of the program.
        let slice = std::slice::from_raw_parts(view.Value as *const u8, size);
        Ok(slice)
    }
}

#[cfg(not(windows))]
fn map_shm(_name: &str, _size: usize) -> Result<&'static [u8], String> {
    Err("shm-frame-reader is Windows-only".to_string())
}

fn read_frame(buf: &[u8]) -> Option<(Header, &[u8])> {
    // sequence-lock loop: try a few times before giving up to caller.
    for _ in 0..64 {
        let pre = Header::parse(&buf[..HEADER_BYTES])?;
        if pre.magic != HEADER_MAGIC || pre.version != HEADER_VERSION {
            return None;
        }
        if pre.frame_id == 0 || pre.frame_id & 1 == 1 {
            // not yet published or write in progress
            std::hint::spin_loop();
            continue;
        }
        let size = pre.jpeg_size as usize;
        if size == 0 || size > buf.len() - HEADER_BYTES {
            std::hint::spin_loop();
            continue;
        }
        let payload = &buf[HEADER_BYTES..HEADER_BYTES + size];
        let post = Header::parse(&buf[..HEADER_BYTES])?;
        if post.frame_id == pre.frame_id {
            return Some((pre, payload));
        }
        // writer overlapped; spin
        std::hint::spin_loop();
    }
    None
}

struct Stats {
    frames_observed: u64,
    bytes_observed: u64,
    decode_ns_total: u128,
    decode_ns_max: u128,
    decode_ok: u64,
    decode_err: u64,
    gaps: u64,
    last_frame_id: u64,
    first_frame_id: u64,
    first_ts_us: u64,
    last_ts_us: u64,
}

impl Stats {
    fn new() -> Self {
        Stats {
            frames_observed: 0,
            bytes_observed: 0,
            decode_ns_total: 0,
            decode_ns_max: 0,
            decode_ok: 0,
            decode_err: 0,
            gaps: 0,
            last_frame_id: 0,
            first_frame_id: 0,
            first_ts_us: 0,
            last_ts_us: 0,
        }
    }
}

fn main() {
    let cli = parse_cli();
    eprintln!(
        "shm-frame-reader: name={} duration={}s buffer={}B decode={}",
        cli.shm_name, cli.duration_secs, cli.buffer_bytes, cli.decode
    );

    let buf = match map_shm(&cli.shm_name, cli.buffer_bytes) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("FATAL: {e}");
            std::process::exit(2);
        }
    };

    let deadline = Instant::now() + Duration::from_secs(cli.duration_secs);
    let mut stats = Stats::new();

    while Instant::now() < deadline {
        let Some((hdr, payload)) = read_frame(buf) else {
            std::thread::sleep(Duration::from_millis(1));
            continue;
        };

        if hdr.frame_id == stats.last_frame_id {
            std::thread::sleep(Duration::from_micros(500));
            continue;
        }

        // gap detection: producer increments by 2 per committed frame
        if stats.last_frame_id != 0 {
            let expected = stats.last_frame_id + 2;
            if hdr.frame_id > expected {
                stats.gaps += (hdr.frame_id - expected) / 2;
            }
        } else {
            stats.first_frame_id = hdr.frame_id;
            stats.first_ts_us = hdr.timestamp_us;
        }
        stats.last_frame_id = hdr.frame_id;
        stats.last_ts_us = hdr.timestamp_us;
        stats.frames_observed += 1;
        stats.bytes_observed += payload.len() as u64;

        let decode_ns = if cli.decode {
            let t = Instant::now();
            match image::load_from_memory_with_format(payload, image::ImageFormat::Jpeg) {
                Ok(img) => {
                    let dt = t.elapsed().as_nanos();
                    let _ = img.width(); // touch
                    stats.decode_ok += 1;
                    dt
                }
                Err(e) => {
                    eprintln!("decode error at frame_id={}: {e}", hdr.frame_id);
                    stats.decode_err += 1;
                    0
                }
            }
        } else {
            0
        };
        stats.decode_ns_total += decode_ns;
        if decode_ns > stats.decode_ns_max {
            stats.decode_ns_max = decode_ns;
        }

        if stats.frames_observed.is_multiple_of(10) || stats.frames_observed == 1 {
            println!(
                "frame_id={:>6} real_id={:>5} ts_us={:>14} {}x{} jpeg={:>6}B decode={:>6.2}ms",
                hdr.frame_id,
                hdr.frame_id / 2,
                hdr.timestamp_us,
                hdr.width,
                hdr.height,
                hdr.jpeg_size,
                decode_ns as f64 / 1_000_000.0,
            );
        }
    }

    print_summary(&stats, cli.duration_secs);
}

fn print_summary(stats: &Stats, duration_secs: u64) {
    let avg_decode_ms = if stats.decode_ok > 0 {
        (stats.decode_ns_total as f64 / stats.decode_ok as f64) / 1_000_000.0
    } else {
        0.0
    };
    let observed_fps = stats.frames_observed as f64 / duration_secs as f64;
    let producer_span_us = stats.last_ts_us.saturating_sub(stats.first_ts_us);
    let producer_fps = if producer_span_us > 0 && stats.frames_observed > 1 {
        (stats.frames_observed - 1) as f64 / (producer_span_us as f64 / 1_000_000.0)
    } else {
        0.0
    };

    println!();
    println!("==== shm-frame-reader summary ====");
    println!("duration:               {duration_secs} s");
    println!("frames observed:        {}", stats.frames_observed);
    println!("observed FPS (reader):  {observed_fps:.2}");
    println!("producer FPS (header):  {producer_fps:.2}");
    println!("gaps (frames missed):   {}", stats.gaps);
    println!(
        "bytes observed:         {} ({:.2} MB)",
        stats.bytes_observed,
        stats.bytes_observed as f64 / (1024.0 * 1024.0)
    );
    println!(
        "avg JPEG size:          {:.1} KB",
        if stats.frames_observed > 0 {
            stats.bytes_observed as f64 / stats.frames_observed as f64 / 1024.0
        } else {
            0.0
        }
    );
    println!(
        "decode ok / err:        {} / {}",
        stats.decode_ok, stats.decode_err
    );
    println!("avg decode time:        {avg_decode_ms:.2} ms");
    println!(
        "max decode time:        {:.2} ms",
        stats.decode_ns_max as f64 / 1_000_000.0
    );
    println!(
        "first/last frame_id:    {} / {}",
        stats.first_frame_id, stats.last_frame_id
    );

    // verdict
    let mut ok = true;
    let mut notes: Vec<String> = Vec::new();
    if stats.gaps > 0 {
        ok = false;
        notes.push(format!("{} gaps detected", stats.gaps));
    }
    if avg_decode_ms > 5.0 {
        ok = false;
        notes.push(format!("avg decode {avg_decode_ms:.2} ms > 5 ms"));
    }
    if (observed_fps - 10.0).abs() > 0.5 && stats.frames_observed > 30 {
        notes.push(format!(
            "observed FPS {observed_fps:.2} outside 10 ± 0.5 window"
        ));
    }
    println!(
        "verdict:                {} {}",
        if ok { "PASS" } else { "REVIEW" },
        notes.join(" / ")
    );
}
