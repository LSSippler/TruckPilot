//! Extreme minimal telemetry frame path — isolates SCS callback overhead from SHM/route work.

use std::sync::Once;

static MINIMAL_HOTPATH_LOGGED: Once = Once::new();

/// Sidecar lines emitted once at init when minimal mode is active.
pub fn format_minimal_init_log_lines(
    resolver_enable_active: bool,
    resolver_source: &str,
) -> Vec<String> {
    let mut lines = vec!["minimal telemetry mode enabled".into()];
    if resolver_enable_active {
        lines.push(format!(
            "minimal telemetry mode ignores route resolver enable file source={resolver_source}"
        ));
    }
    lines
}

/// Minimal frame hotpath — no telemetry SHM write, route tick, or RouteBlackboard updates.
pub fn handle_frame(frame_cb_count: u32) {
    crate::frame_perf::note_minimal_frame_cb();
    MINIMAL_HOTPATH_LOGGED.call_once(|| {
        crate::diag_log::event_force("minimal telemetry frame hotpath active");
    });
    crate::frame_perf::publish_live_snapshot(frame_cb_count, 0, 0);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_isolation::TestResolverStateGuard;
    use std::sync::atomic::Ordering;

    fn temp_dir(tag: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("tp-minimal-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");
        dir
    }

    #[test]
    fn without_enable_file_minimal_telemetry_disabled() {
        let _guard = TestResolverStateGuard::acquire();
        let dir = temp_dir("off");
        crate::safe_mem::set_test_enable_dir(Some(dir.clone()));
        assert!(!crate::safe_mem::minimal_telemetry_enabled());
        crate::safe_mem::set_test_enable_dir(None);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn with_enable_file_minimal_telemetry_enabled() {
        let _guard = TestResolverStateGuard::acquire();
        let dir = temp_dir("on");
        std::fs::write(dir.join("truckpilot_minimal_telemetry.enable"), b"").expect("enable");
        crate::safe_mem::set_test_enable_dir(Some(dir.clone()));
        assert!(crate::safe_mem::minimal_telemetry_enabled());
        crate::safe_mem::set_test_enable_dir(None);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn minimal_frame_storm_suppresses_hotpath_work() {
        let _guard = TestResolverStateGuard::acquire();
        let dir = temp_dir("storm");
        std::fs::write(dir.join("truckpilot_minimal_telemetry.enable"), b"").expect("enable");
        crate::safe_mem::set_test_enable_dir(Some(dir.clone()));
        crate::frame_perf::set_minimal_telemetry_enabled(true);
        let shm_before = crate::frame_perf::SHM_WRITE_COUNT.load(Ordering::Relaxed);
        let ready_before = crate::frame_perf::READY_EVENT_SET_COUNT.load(Ordering::Relaxed);
        let bb_before = crate::frame_perf::ROUTE_BB_FRAME_WRITE_COUNT.load(Ordering::Relaxed);
        let dispatch_before =
            crate::frame_perf::ROUTE_TICK_DISPATCH_COUNT.load(Ordering::Relaxed);
        let notify_before = crate::frame_perf::NOTIFY_FRAME_TICK_COUNT.load(Ordering::Relaxed);
        let wake_before = crate::frame_perf::WORKER_WAKE_SET_EVENT_COUNT.load(Ordering::Relaxed);
        for i in 1..=10_000 {
            handle_frame(i);
        }
        assert_eq!(
            crate::frame_perf::MINIMAL_FRAME_CB_COUNT.load(Ordering::Relaxed),
            10_000
        );
        assert_eq!(
            crate::frame_perf::SHM_WRITE_COUNT.load(Ordering::Relaxed),
            shm_before
        );
        assert_eq!(
            crate::frame_perf::READY_EVENT_SET_COUNT.load(Ordering::Relaxed),
            ready_before
        );
        assert_eq!(
            crate::frame_perf::ROUTE_BB_FRAME_WRITE_COUNT.load(Ordering::Relaxed),
            bb_before
        );
        assert_eq!(
            crate::frame_perf::ROUTE_TICK_DISPATCH_COUNT.load(Ordering::Relaxed),
            dispatch_before
        );
        assert_eq!(
            crate::frame_perf::NOTIFY_FRAME_TICK_COUNT.load(Ordering::Relaxed),
            notify_before
        );
        assert_eq!(
            crate::frame_perf::WORKER_WAKE_SET_EVENT_COUNT.load(Ordering::Relaxed),
            wake_before
        );
        assert_eq!(
            crate::resolver_metrics::RESOLVER_WORKER_PATTERN_SCAN_COUNT.load(Ordering::Relaxed),
            0
        );
        crate::safe_mem::set_test_enable_dir(None);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn minimal_with_route_resolver_enable_files_stays_safe() {
        let _guard = TestResolverStateGuard::acquire();
        let dir = temp_dir("resolver-block");
        std::fs::write(dir.join("truckpilot_minimal_telemetry.enable"), b"").expect("minimal");
        std::fs::write(
            dir.join("truckpilot_route_resolver.gps_offset_probe"),
            b"",
        )
        .expect("probe");
        crate::safe_mem::set_test_enable_dir(Some(dir.clone()));
        assert!(crate::safe_mem::minimal_telemetry_enabled());
        assert!(crate::safe_mem::route_resolver_mode().is_off());
        assert!(crate::resolver_guard::block_if_resolver_off().is_some());
        let lines = format_minimal_init_log_lines(true, "truckpilot_route_resolver.gps_offset_probe");
        assert!(lines.iter().any(|l| l.contains("minimal telemetry mode enabled")));
        assert!(lines.iter().any(|l| l.contains("ignores route resolver enable file")));
        crate::safe_mem::set_test_enable_dir(None);
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn format_minimal_init_log_without_resolver_files() {
        let lines = format_minimal_init_log_lines(false, "none");
        assert_eq!(lines.len(), 1);
        assert_eq!(lines[0], "minimal telemetry mode enabled");
    }
}
