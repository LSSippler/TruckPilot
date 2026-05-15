//! Integration test: drive the vision-frame-source plugin against a
//! Vec-backed mock SHM buffer, no platform deps.

use std::sync::Arc;

use truckpilot_plugin_api::{
    ControlOutput, Plugin, PluginContext, SharedBlackboard, SharedFrameStore,
};
use truckpilot_plugin_vision_frame_source::{
    shm_reader::{HEADER_BYTES, HEADER_MAGIC, HEADER_VERSION},
    Settings, VisionFrameSource,
};

/// Owns a `Box<[u8]>` leaked into `'static` and exposes both a writer
/// (`buf_mut`) and a reader-view (`view`). Sequence-lock semantics are
/// driven by `publish` — tests are single-threaded so we can alias
/// mutably while the plugin holds the immutable view; the reader only
/// observes committed frames between `publish` calls.
struct MockShm {
    ptr: *mut u8,
    len: usize,
}

impl MockShm {
    fn new(size: usize) -> (Self, &'static [u8]) {
        let boxed: Box<[u8]> = vec![0u8; size].into_boxed_slice();
        let leaked: &'static mut [u8] = Box::leak(boxed);
        let ptr = leaked.as_mut_ptr();
        let len = leaked.len();
        // Reader view aliases the same memory. Producer never writes
        // while the reader reads (single-threaded test); the
        // sequence-lock protocol still applies as a correctness check
        // against `read_frame`.
        let view: &'static [u8] = unsafe { std::slice::from_raw_parts(ptr as *const u8, len) };
        (MockShm { ptr, len }, view)
    }

    fn buf_mut(&mut self) -> &mut [u8] {
        // SAFETY: we own the leaked allocation; tests are single-threaded
        // so no concurrent reader/writer aliasing occurs during this call.
        unsafe { std::slice::from_raw_parts_mut(self.ptr, self.len) }
    }

    /// Atomically (single-thread, sequence-lock semantics) publish a
    /// frame: bump frame_id to odd, write header+payload, bump frame_id
    /// to the next even value.
    fn publish(
        &mut self,
        frame_id: u64,
        timestamp_us: u64,
        width: u32,
        height: u32,
        payload: &[u8],
    ) {
        assert!(frame_id.is_multiple_of(2), "frame_id must be even");
        assert!(payload.len() + HEADER_BYTES <= self.len);
        let buf = self.buf_mut();
        // Step 1: write magic/version once.
        buf[0..4].copy_from_slice(&HEADER_MAGIC);
        buf[4..8].copy_from_slice(&HEADER_VERSION.to_le_bytes());
        // Step 2: bump to odd (write-in-progress).
        let writing = frame_id | 1;
        buf[8..16].copy_from_slice(&writing.to_le_bytes());
        // Step 3: write metadata + payload.
        buf[16..24].copy_from_slice(&timestamp_us.to_le_bytes());
        buf[24..28].copy_from_slice(&width.to_le_bytes());
        buf[28..32].copy_from_slice(&height.to_le_bytes());
        buf[32..36].copy_from_slice(&(payload.len() as u32).to_le_bytes());
        buf[HEADER_BYTES..HEADER_BYTES + payload.len()].copy_from_slice(payload);
        // Step 4: bump to even (committed).
        buf[8..16].copy_from_slice(&frame_id.to_le_bytes());
    }
}

fn make_ctx() -> (PluginContext, Arc<SharedFrameStore>) {
    let store = Arc::new(SharedFrameStore::new());
    let ctx = PluginContext::new("vision-frame-source", SharedBlackboard::new())
        .with_frame_store(store.clone());
    (ctx, store)
}

#[test]
fn on_load_initialises_blackboard_keys() {
    let mut plugin = VisionFrameSource::with_settings(Settings::default());
    let (ctx, _store) = make_ctx();
    plugin.on_load(&ctx);
    assert_eq!(
        ctx.blackboard.get("vision.source.healthy").as_deref(),
        Some("false")
    );
    assert_eq!(
        ctx.blackboard.get("vision.frame.ready").as_deref(),
        Some("false")
    );
    assert_eq!(ctx.blackboard.get("vision.frame.id").as_deref(), Some("0"));
}

#[test]
fn tick_without_shm_keeps_unhealthy() {
    let mut plugin = VisionFrameSource::with_settings(Settings::default());
    let (ctx, _store) = make_ctx();
    plugin.on_load(&ctx);
    let mut out = ControlOutput::default();
    for _ in 0..5 {
        plugin.tick(None, &mut out, &ctx);
    }
    assert_eq!(
        ctx.blackboard.get("vision.source.healthy").as_deref(),
        Some("false")
    );
}

#[test]
fn tick_publishes_frame_then_marks_not_ready_on_repeat() {
    let (mut shm, view) = MockShm::new(1024);
    let mut plugin = VisionFrameSource::with_settings(Settings::default());
    let (ctx, store) = make_ctx();
    plugin.on_load(&ctx);
    plugin.inject_buffer(view);

    let now_us = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_micros() as u64;
    let payload = vec![0xAA, 0xBB, 0xCC, 0xDD];
    shm.publish(2, now_us, 320, 240, &payload);

    let mut out = ControlOutput::default();
    plugin.tick(None, &mut out, &ctx);

    assert_eq!(
        ctx.blackboard.get("vision.source.healthy").as_deref(),
        Some("true")
    );
    assert_eq!(
        ctx.blackboard.get("vision.frame.ready").as_deref(),
        Some("true")
    );
    assert_eq!(ctx.blackboard.get("vision.frame.id").as_deref(), Some("1"));
    assert_eq!(
        ctx.blackboard.get("vision.frame.width").as_deref(),
        Some("320")
    );
    assert_eq!(
        ctx.blackboard.get("vision.frame.height").as_deref(),
        Some("240")
    );
    assert_eq!(
        ctx.blackboard.get("vision.frame.jpeg_size").as_deref(),
        Some("4")
    );
    let frame = store.get("camera.front").expect("frame published");
    assert_eq!(frame.id, 1);
    assert_eq!(frame.width, 320);
    assert_eq!(frame.height, 240);
    assert_eq!(frame.jpeg.as_slice(), payload.as_slice());

    // Same frame on the next tick → ready=false, healthy still true.
    plugin.tick(None, &mut out, &ctx);
    assert_eq!(
        ctx.blackboard.get("vision.frame.ready").as_deref(),
        Some("false")
    );
    assert_eq!(
        ctx.blackboard.get("vision.source.healthy").as_deref(),
        Some("true")
    );
}

#[test]
fn tick_advances_frame_id_across_publishes() {
    let (mut shm, view) = MockShm::new(1024);
    let mut plugin = VisionFrameSource::with_settings(Settings::default());
    let (ctx, store) = make_ctx();
    plugin.on_load(&ctx);
    plugin.inject_buffer(view);

    let mut out = ControlOutput::default();
    let now_us = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_micros() as u64;

    let mut payload = vec![0u8; 8];
    for logical_id in 1..=5u64 {
        payload[0] = logical_id as u8;
        shm.publish(logical_id * 2, now_us, 4, 4, &payload);
        plugin.tick(None, &mut out, &ctx);
        let frame = store.get("camera.front").expect("frame");
        assert_eq!(frame.id, logical_id, "logical id should advance");
        assert_eq!(
            ctx.blackboard.get("vision.frame.id").as_deref(),
            Some(logical_id.to_string().as_str())
        );
    }
}

#[test]
fn tick_flags_stale_when_timestamp_old() {
    let (mut shm, view) = MockShm::new(1024);
    let mut plugin = VisionFrameSource::with_settings(Settings {
        stale_after_ms: 100,
        ..Settings::default()
    });
    let (ctx, _store) = make_ctx();
    plugin.on_load(&ctx);
    plugin.inject_buffer(view);

    // Timestamp 1 second in the past → stale.
    let old_ts = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_micros() as u64
        - 1_000_000;
    let payload = vec![0u8; 4];
    shm.publish(2, old_ts, 4, 4, &payload);

    let mut out = ControlOutput::default();
    plugin.tick(None, &mut out, &ctx);
    assert_eq!(
        ctx.blackboard.get("vision.frame.stale").as_deref(),
        Some("true")
    );
}

#[test]
fn tick_handles_invalid_header() {
    let (mut shm, view) = MockShm::new(1024);
    {
        // Corrupt the magic bytes; no valid frame ever appears.
        let buf = shm.buf_mut();
        buf[0..4].copy_from_slice(b"XXXX");
    }
    let mut plugin = VisionFrameSource::with_settings(Settings::default());
    let (ctx, _store) = make_ctx();
    plugin.on_load(&ctx);
    plugin.inject_buffer(view);

    let mut out = ControlOutput::default();
    plugin.tick(None, &mut out, &ctx);
    assert_eq!(
        ctx.blackboard.get("vision.source.healthy").as_deref(),
        Some("false")
    );
    assert!(ctx
        .blackboard
        .get("vision.source.last_error")
        .as_deref()
        .is_some());
}
