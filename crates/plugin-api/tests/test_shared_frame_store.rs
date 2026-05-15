//! Tests for the Phase 6.5c.2 additions: `SharedFrame`,
//! `SharedFrameStore`, and `PluginContext::frame_store`.

use std::sync::{Arc, Barrier};
use std::thread;

use truckpilot_plugin_api::{PluginContext, SharedBlackboard, SharedFrame, SharedFrameStore};

fn make_frame(id: u64, payload_len: usize) -> Arc<SharedFrame> {
    Arc::new(SharedFrame::new(
        id,
        1_700_000_000_000_000 + id,
        1920,
        1080,
        Arc::new(vec![0xAB; payload_len]),
    ))
}

#[test]
fn shared_frame_fields_are_preserved() {
    let jpeg = Arc::new(vec![0xFF, 0xD8, 0xFF, 0xD9]);
    let frame = SharedFrame::new(7, 12_345, 640, 480, Arc::clone(&jpeg));
    assert_eq!(frame.id, 7);
    assert_eq!(frame.timestamp_us, 12_345);
    assert_eq!(frame.width, 640);
    assert_eq!(frame.height, 480);
    assert_eq!(frame.jpeg.as_slice(), &[0xFF, 0xD8, 0xFF, 0xD9]);
    assert!(frame.decoded_rgb8().is_none());
}

#[test]
fn store_set_and_get_round_trip() {
    let store = SharedFrameStore::new();
    assert!(store.is_empty());
    let frame = make_frame(1, 256);
    store.set("camera.front", Arc::clone(&frame));
    let fetched = store.get("camera.front").expect("frame should be present");
    assert_eq!(fetched.id, 1);
    assert!(Arc::ptr_eq(&fetched, &frame));
    assert_eq!(store.len(), 1);
}

#[test]
fn store_get_missing_returns_none() {
    let store = SharedFrameStore::new();
    assert!(store.get("camera.front").is_none());
}

#[test]
fn store_remove_drops_entry() {
    let store = SharedFrameStore::new();
    store.set("camera.front", make_frame(1, 16));
    let removed = store.remove("camera.front");
    assert!(removed.is_some());
    assert!(store.get("camera.front").is_none());
    assert!(store.remove("camera.front").is_none());
}

#[test]
fn store_overwrites_previous_value() {
    let store = SharedFrameStore::new();
    store.set("camera.front", make_frame(1, 16));
    store.set("camera.front", make_frame(2, 16));
    let cur = store.get("camera.front").unwrap();
    assert_eq!(cur.id, 2);
    assert_eq!(store.len(), 1);
}

#[test]
fn store_clone_shares_state() {
    let store = SharedFrameStore::new();
    let clone = store.clone();
    store.set("camera.front", make_frame(42, 32));
    let from_clone = clone.get("camera.front").expect("clone sees write");
    assert_eq!(from_clone.id, 42);
}

#[test]
fn store_keys_lists_published() {
    let store = SharedFrameStore::new();
    store.set("camera.front", make_frame(1, 0));
    store.set("camera.rear", make_frame(2, 0));
    let mut keys = store.keys();
    keys.sort();
    assert_eq!(
        keys,
        vec!["camera.front".to_string(), "camera.rear".to_string()]
    );
}

#[test]
fn decoded_rgb8_oncelock_lazy_init() {
    let frame = make_frame(1, 4);
    assert!(frame.decoded_rgb8().is_none());

    let pixels = Arc::new(vec![1u8, 2, 3, 4, 5, 6]);
    let got = frame.get_or_init_rgb8(|| Arc::clone(&pixels));
    assert!(Arc::ptr_eq(got, &pixels));

    // Second call must not re-run the closure and must return the same Arc.
    let again = frame.get_or_init_rgb8(|| panic!("closure must not run twice"));
    assert!(Arc::ptr_eq(again, &pixels));
    assert!(frame.decoded_rgb8().is_some());
}

#[test]
fn decoded_rgb8_try_init_caches_on_success() {
    let frame = make_frame(1, 0);
    let pixels = Arc::new(vec![9u8; 12]);
    let res: Result<_, &'static str> = frame.get_or_try_init_rgb8(|| Ok(Arc::clone(&pixels)));
    assert!(res.is_ok());
    assert!(frame.decoded_rgb8().is_some());

    // Subsequent failing closure must not be called and must not clear the cache.
    let res2: Result<_, &'static str> =
        frame.get_or_try_init_rgb8(|| panic!("must not run after success"));
    assert!(res2.is_ok());
}

#[test]
fn decoded_rgb8_try_init_does_not_cache_on_error() {
    let frame = make_frame(1, 0);
    let res: Result<_, &'static str> = frame.get_or_try_init_rgb8(|| Err("decoder fail"));
    assert_eq!(res.err(), Some("decoder fail"));
    assert!(frame.decoded_rgb8().is_none());

    // A later successful attempt populates the slot.
    let pixels = Arc::new(vec![1u8, 2, 3]);
    let res2: Result<_, &'static str> = frame.get_or_try_init_rgb8(|| Ok(Arc::clone(&pixels)));
    assert!(res2.is_ok());
    assert!(frame.decoded_rgb8().is_some());
}

#[test]
fn store_concurrent_writers_and_readers() {
    let store = Arc::new(SharedFrameStore::new());
    let n_writers = 4;
    let n_readers = 4;
    let writes_per_writer = 200;
    let barrier = Arc::new(Barrier::new(n_writers + n_readers));

    let mut handles = Vec::new();

    for w in 0..n_writers {
        let store = Arc::clone(&store);
        let barrier = Arc::clone(&barrier);
        handles.push(thread::spawn(move || {
            barrier.wait();
            for i in 0..writes_per_writer {
                let id = (w as u64) * 1000 + i as u64;
                store.set("camera.front", make_frame(id, 64));
            }
        }));
    }

    for _ in 0..n_readers {
        let store = Arc::clone(&store);
        let barrier = Arc::clone(&barrier);
        handles.push(thread::spawn(move || {
            barrier.wait();
            for _ in 0..writes_per_writer {
                // Just verify get never panics / never returns a torn value.
                if let Some(f) = store.get("camera.front") {
                    assert_eq!(f.width, 1920);
                    assert_eq!(f.height, 1080);
                    assert_eq!(f.jpeg.len(), 64);
                }
            }
        }));
    }

    for h in handles {
        h.join().expect("thread panicked");
    }

    let final_frame = store.get("camera.front").expect("must still be set");
    assert_eq!(final_frame.width, 1920);
}

#[test]
fn decoded_rgb8_concurrent_init_single_winner() {
    let frame = make_frame(1, 0);
    let frame = Arc::new((*frame).clone_for_test());
    let barrier = Arc::new(Barrier::new(8));
    let mut handles = Vec::new();
    for t in 0..8 {
        let frame = Arc::clone(&frame);
        let barrier = Arc::clone(&barrier);
        handles.push(thread::spawn(move || {
            barrier.wait();
            let buf = Arc::new(vec![t as u8; 16]);
            let got = frame.get_or_init_rgb8(|| buf);
            got.clone()
        }));
    }
    let results: Vec<Arc<Vec<u8>>> = handles.into_iter().map(|h| h.join().unwrap()).collect();
    // All threads must observe the same Arc instance (the one that won).
    let first = &results[0];
    for other in &results[1..] {
        assert!(
            Arc::ptr_eq(first, other),
            "OnceLock returned different Arcs"
        );
    }
}

#[test]
fn plugin_context_frame_store_defaults_none() {
    let ctx = PluginContext::new("p", SharedBlackboard::new());
    assert!(ctx.frame_store().is_none());
}

#[test]
fn plugin_context_with_frame_store_round_trip() {
    let store = Arc::new(SharedFrameStore::new());
    store.set("camera.front", make_frame(99, 8));
    let ctx = PluginContext::new("p", SharedBlackboard::new()).with_frame_store(Arc::clone(&store));
    let got = ctx.frame_store().expect("store should be attached");
    assert!(Arc::ptr_eq(&got, &store));
    assert_eq!(got.get("camera.front").unwrap().id, 99);
}

#[test]
fn plugin_context_clone_preserves_frame_store() {
    let store = Arc::new(SharedFrameStore::new());
    let ctx = PluginContext::new("p", SharedBlackboard::new()).with_frame_store(Arc::clone(&store));
    let clone = ctx.clone();
    let got = clone.frame_store().expect("clone should retain store");
    assert!(Arc::ptr_eq(&got, &store));
}

// Test-only helper: SharedFrame is not Clone (OnceLock isn't). Some
// concurrency tests want multiple Arc<SharedFrame> backed by a fresh
// (uninitialised) OnceLock. We rebuild from the public fields rather
// than expose Clone on the public type.
trait CloneForTest {
    fn clone_for_test(&self) -> Self;
}

impl CloneForTest for SharedFrame {
    fn clone_for_test(&self) -> Self {
        SharedFrame::new(
            self.id,
            self.timestamp_us,
            self.width,
            self.height,
            Arc::clone(&self.jpeg),
        )
    }
}
