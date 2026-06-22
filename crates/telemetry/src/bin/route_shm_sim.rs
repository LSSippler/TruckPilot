//! Fake ETS2 route writer for `Local\TruckPilotRouteBlackboard`.
//!
//! Lets Core integration be tested without a running game.
//!
//! Usage:
//!   route-shm-sim                              # 5 synthetic waypoints, every 2 s
//!   route-shm-sim --once --uids 101,202,303    # explicit graph UIDs (recommended)
//!   route-shm-sim --once --uids-file uids.txt  # one UID per line or comma-separated
//!   route-shm-sim --once --with-positions --uids 101,202,303
//!   route-shm-sim --once --uid-pos-file coords.txt   # uid,x,y,z per line

use std::collections::HashMap;
use std::fs;
use std::mem;
use std::path::Path;
use std::time::Duration;

use truckpilot_telemetry::nav_route::{
    count_waypoints_with_flag, route_uid_hash, RouteBlackboardLayout, RouteWaypoint,
    MAX_ROUTE_WAYPOINTS, ROUTE_MAGIC, ROUTE_SHM_NAME, ROUTE_VERSION, ROUTE_WP_FLAG_HAS_DISTANCE,
    ROUTE_WP_FLAG_HAS_POSITION, ROUTE_WP_FLAG_HAS_TIME,
};

fn parse_usize_arg(flag: &str, default: usize) -> usize {
    let args: Vec<String> = std::env::args().collect();
    args.windows(2)
        .find(|w| w[0] == flag)
        .and_then(|w| w[1].parse().ok())
        .unwrap_or(default)
}

fn parse_u64_arg(flag: &str, default: u64) -> u64 {
    let args: Vec<String> = std::env::args().collect();
    args.windows(2)
        .find(|w| w[0] == flag)
        .and_then(|w| w[1].parse().ok())
        .unwrap_or(default)
}

fn arg_value(flag: &str) -> Option<String> {
    let args: Vec<String> = std::env::args().collect();
    args.windows(2)
        .find(|w| w[0] == flag)
        .map(|w| w[1].clone())
}

fn has_flag(flag: &str) -> bool {
    std::env::args().any(|a| a == flag)
}

/// Parse comma/whitespace-separated decimal or `0x` hex UIDs.
pub fn parse_uid_list(input: &str) -> Result<Vec<u64>, String> {
    let trimmed = input.trim();
    if trimmed.is_empty() {
        return Err("UID list is empty".into());
    }
    let mut uids = Vec::new();
    for part in trimmed.split(|c: char| c == ',' || c.is_whitespace()) {
        let token = part.trim();
        if token.is_empty() {
            continue;
        }
        let uid = if let Some(hex) = token.strip_prefix("0x").or_else(|| token.strip_prefix("0X"))
        {
            u64::from_str_radix(hex, 16)
                .map_err(|e| format!("invalid hex UID '{token}': {e}"))?
        } else {
            token
                .parse::<u64>()
                .map_err(|e| format!("invalid UID '{token}': {e}"))?
        };
        uids.push(uid);
    }
    if uids.is_empty() {
        return Err("UID list is empty".into());
    }
    if uids.len() > MAX_ROUTE_WAYPOINTS {
        return Err(format!(
            "UID list too long: {} (max {MAX_ROUTE_WAYPOINTS})",
            uids.len()
        ));
    }
    Ok(uids)
}

/// Read UIDs from a text file (comma or whitespace separated, `#` comments).
pub fn parse_uid_file(path: &Path) -> Result<Vec<u64>, String> {
    let raw = fs::read_to_string(path)
        .map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    let mut chunks = Vec::new();
    for line in raw.lines() {
        let line = line.split('#').next().unwrap_or(line).trim();
        if !line.is_empty() {
            chunks.push(line);
        }
    }
    parse_uid_list(&chunks.join(","))
}

/// Parse `uid,x,y,z` per line (commas, `#` comments allowed).
pub fn parse_uid_pos_file(path: &Path) -> Result<Vec<(u64, f32, f32, f32)>, String> {
    let raw = fs::read_to_string(path)
        .map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    let mut rows = Vec::new();
    for (line_no, line) in raw.lines().enumerate() {
        let line = line.split('#').next().unwrap_or(line).trim();
        if line.is_empty() {
            continue;
        }
        let parts: Vec<&str> = line.split(',').map(str::trim).collect();
        if parts.len() != 4 {
            return Err(format!(
                "{}:{}: expected uid,x,y,z (4 fields), got {}",
                path.display(),
                line_no + 1,
                parts.len()
            ));
        }
        let uid = parts[0]
            .parse::<u64>()
            .map_err(|e| format!("{}:{}: bad uid: {e}", path.display(), line_no + 1))?;
        let x: f32 = parts[1]
            .parse()
            .map_err(|e| format!("{}:{}: bad x: {e}", path.display(), line_no + 1))?;
        let y: f32 = parts[2]
            .parse()
            .map_err(|e| format!("{}:{}: bad y: {e}", path.display(), line_no + 1))?;
        let z: f32 = parts[3]
            .parse()
            .map_err(|e| format!("{}:{}: bad z: {e}", path.display(), line_no + 1))?;
        rows.push((uid, x, y, z));
    }
    if rows.is_empty() {
        return Err(format!("{}: no uid,x,y,z rows", path.display()));
    }
    Ok(rows)
}

/// Options for synthetic ETS2 waypoint fields in the simulator.
#[derive(Debug, Default)]
pub struct SimWaypointOptions {
    pub with_positions: bool,
    pub with_distance: bool,
    pub with_time: bool,
    pub uid_positions: Vec<(u64, f32, f32, f32)>,
}

fn resolve_sim_options() -> Result<SimWaypointOptions, String> {
    let mut opts = SimWaypointOptions {
        with_positions: has_flag("--with-positions"),
        with_distance: has_flag("--with-distance"),
        with_time: has_flag("--with-time"),
        ..Default::default()
    };
    if let Some(path) = arg_value("--uid-pos-file") {
        opts.uid_positions = parse_uid_pos_file(Path::new(&path))?;
        opts.with_positions = true;
    }
    Ok(opts)
}

/// Build SHM waypoints for simulation (Phase 5i).
pub fn build_sim_waypoints(uids: &[u64], opts: &SimWaypointOptions) -> Vec<RouteWaypoint> {
    let pos_map: HashMap<u64, (f32, f32, f32)> = opts
        .uid_positions
        .iter()
        .map(|&(uid, x, y, z)| (uid, (x, y, z)))
        .collect();
    let n = uids.len();
    uids
        .iter()
        .enumerate()
        .map(|(i, &uid)| {
            let mut wp = RouteWaypoint {
                uid: uid as i64,
                ..Default::default()
            };
            if let Some((x, y, z)) = pos_map.get(&uid) {
                wp.x = *x;
                wp.y = *y;
                wp.z = *z;
                wp.flags |= ROUTE_WP_FLAG_HAS_POSITION;
            } else if opts.with_positions {
                wp.x = i as f32 * 100.0;
                wp.y = 0.0;
                wp.z = i as f32 * 50.0;
                wp.flags |= ROUTE_WP_FLAG_HAS_POSITION;
            }
            if opts.with_distance {
                wp.distance = (n.saturating_sub(i)) as f32 * 1000.0;
                wp.flags |= ROUTE_WP_FLAG_HAS_DISTANCE;
            }
            if opts.with_time {
                wp.time = (n.saturating_sub(i)) as f32 * 60.0;
                wp.flags |= ROUTE_WP_FLAG_HAS_TIME;
            }
            wp
        })
        .collect()
}

fn resolve_uids() -> Result<Vec<u64>, String> {
    if let Some(path) = arg_value("--uids-file") {
        return parse_uid_file(Path::new(&path));
    }
    if let Some(list) = arg_value("--uids") {
        return parse_uid_list(&list);
    }
    let count = parse_usize_arg("--count", 5).clamp(1, MAX_ROUTE_WAYPOINTS);
    let base = parse_u64_arg("--base", 6_000_000_000_000_000_000);
    Ok(fake_uids(count, base))
}

/// Returns a warning message when explicit UID lists are too short for stable import.
pub(crate) fn short_uid_route_message(uid_count: usize) -> Option<&'static str> {
    if uid_count <= 2 {
        Some("2 UID route may be rejected after trim/too_short; use at least 4 connected graph UIDs")
    } else if uid_count < 4 {
        Some("short UID list; 4+ connected graph UIDs recommended for T1 smoke")
    } else {
        None
    }
}

/// Warn when explicit UID lists are too short for stable ETS2 route import.
pub(crate) fn warn_short_uid_route(uid_count: usize, explicit: bool) {
    if !explicit {
        return;
    }
    if let Some(msg) = short_uid_route_message(uid_count) {
        eprintln!("WARN route-shm-sim: {uid_count} UIDs — {msg}");
    }
}

#[cfg(windows)]
mod writer {
    use super::*;
    use std::ffi::c_void;
    use std::ptr;

    type HANDLE = isize;
    type DWORD = u32;
    const NULL: HANDLE = 0;
    const INVALID_HANDLE_VALUE: HANDLE = -1isize;
    const PAGE_READWRITE: DWORD = 0x04;
    const FILE_MAP_WRITE: DWORD = 0x02;

    extern "system" {
        fn CreateFileMappingW(
            hFile: HANDLE,
            lpAttr: *const c_void,
            flProtect: DWORD,
            dwHigh: DWORD,
            dwLow: DWORD,
            lpName: *const u16,
        ) -> HANDLE;
        fn MapViewOfFile(h: HANDLE, acc: DWORD, hi: DWORD, lo: DWORD, n: usize) -> *mut c_void;
        fn UnmapViewOfFile(p: *const c_void) -> i32;
        fn CloseHandle(h: HANDLE) -> i32;
    }

    fn wide(s: &str) -> Vec<u16> {
        s.encode_utf16().chain(std::iter::once(0)).collect()
    }

    pub struct RouteWriter {
        handle: HANDLE,
        ptr: *mut RouteBlackboardLayout,
        sequence: u32,
    }

    impl RouteWriter {
        pub fn open() -> Result<Self, String> {
            let name = wide(ROUTE_SHM_NAME);
            let size = mem::size_of::<RouteBlackboardLayout>();
            let handle = unsafe {
                CreateFileMappingW(
                    INVALID_HANDLE_VALUE,
                    ptr::null(),
                    PAGE_READWRITE,
                    0,
                    size as DWORD,
                    name.as_ptr(),
                )
            };
            if handle == NULL {
                return Err("CreateFileMappingW failed".into());
            }
            let ptr = unsafe {
                MapViewOfFile(handle, FILE_MAP_WRITE, 0, 0, size) as *mut RouteBlackboardLayout
            };
            if ptr.is_null() {
                unsafe { CloseHandle(handle) };
                return Err("MapViewOfFile failed".into());
            }
            unsafe {
                (*ptr).magic = ROUTE_MAGIC;
                (*ptr).version = ROUTE_VERSION;
            }
            Ok(Self {
                handle,
                ptr,
                sequence: 0,
            })
        }

        pub fn publish_waypoints(&mut self, waypoints: &[RouteWaypoint]) {
            unsafe {
                let shm = &mut *self.ptr;
                let odd = self.sequence.wrapping_add(1) | 1;
                ptr::write_volatile(&mut shm.sequence, odd);
                ptr::write_volatile(&mut shm.valid, 0);

                let count = waypoints.len().min(MAX_ROUTE_WAYPOINTS);
                for (i, wp) in waypoints[..count].iter().enumerate() {
                    shm.waypoints[i] = *wp;
                }
                let hash = route_uid_hash(&waypoints[..count]);
                ptr::write_volatile(&mut shm.waypoint_count, count as u32);
                ptr::write_volatile(&mut shm.flags, 0);
                ptr::write_volatile(&mut shm.route_hash, hash);
                ptr::write_volatile(&mut shm.valid, if count > 0 { 1 } else { 0 });
                self.sequence = odd.wrapping_add(1);
                ptr::write_volatile(&mut shm.sequence, self.sequence);
            }
        }

        #[allow(dead_code)]
        pub fn publish_uids(&mut self, uids: &[u64]) {
            let waypoints: Vec<RouteWaypoint> = uids
                .iter()
                .map(|&uid| RouteWaypoint {
                    uid: uid as i64,
                    ..Default::default()
                })
                .collect();
            self.publish_waypoints(&waypoints);
        }

        pub fn publish_invalid(&mut self) {
            unsafe {
                let shm = &mut *self.ptr;
                let odd = self.sequence.wrapping_add(1) | 1;
                ptr::write_volatile(&mut shm.sequence, odd);
                ptr::write_volatile(&mut shm.valid, 0);
                ptr::write_volatile(&mut shm.waypoint_count, 0);
                ptr::write_volatile(&mut shm.route_hash, 0);
                self.sequence = odd.wrapping_add(1);
                ptr::write_volatile(&mut shm.sequence, self.sequence);
            }
        }
    }

    impl Drop for RouteWriter {
        fn drop(&mut self) {
            unsafe {
                UnmapViewOfFile(self.ptr as *const c_void);
                CloseHandle(self.handle);
            }
        }
    }
}

#[cfg(not(windows))]
mod writer {
    pub struct RouteWriter;

    impl RouteWriter {
        pub fn open() -> Result<Self, String> {
            Err("route-shm-sim requires Windows (Local\\TruckPilotRouteBlackboard)".into())
        }
        pub fn publish_uids(&mut self, _uids: &[u64]) {}
        pub fn publish_waypoints(&mut self, _waypoints: &[RouteWaypoint]) {}
        pub fn publish_invalid(&mut self) {}
    }
}

fn fake_uids(count: usize, base: u64) -> Vec<u64> {
    (0..count)
        .map(|i| base + u64::try_from(i).unwrap_or(0) * 1_000_000_000_000_000)
        .collect()
}

fn format_uid_summary(uids: &[u64]) -> String {
    if uids.is_empty() {
        return "0 UIDs".into();
    }
    if uids.len() == 1 {
        return format!("1 UID [{}]", uids[0]);
    }
    format!(
        "{} UIDs [first={}, last={}]",
        uids.len(),
        uids[0],
        uids[uids.len() - 1]
    )
}

fn main() {
    let once = has_flag("--once");
    let invalidate = has_flag("--invalidate");

    let sim_opts = match resolve_sim_options() {
        Ok(o) => o,
        Err(e) => {
            eprintln!("ERROR: {e}");
            std::process::exit(1);
        }
    };

    let uids = match resolve_uids() {
        Ok(u) => u,
        Err(e) => {
            eprintln!("ERROR: {e}");
            std::process::exit(1);
        }
    };

    let explicit_uids =
        arg_value("--uids").is_some() || arg_value("--uids-file").is_some();
    warn_short_uid_route(uids.len(), explicit_uids);

    println!("route-shm-sim → {ROUTE_SHM_NAME}");
    println!("  mode={}", if invalidate { "invalidate" } else { "publish" });
    println!("  once={once}");
    if arg_value("--uids").is_some() || arg_value("--uids-file").is_some() {
        println!("  uids={}", format_uid_summary(&uids));
        println!(
            "  note: explicit UIDs are not snapped to current truck position; router trims on import (Phase 5d) and advances live progress with telemetry (Phase 5e)"
        );
    } else {
        let count = uids.len();
        let base = uids.first().copied().unwrap_or(0);
        println!("  synthetic count={count} base={base}");
    }
    if sim_opts.with_positions || !sim_opts.uid_positions.is_empty() {
        println!("  sim positions=on");
    }
    if sim_opts.with_distance {
        println!("  sim distance=on");
    }
    if sim_opts.with_time {
        println!("  sim time=on");
    }

    let base_waypoints = build_sim_waypoints(&uids, &sim_opts);
    let pos_count = count_waypoints_with_flag(&base_waypoints, ROUTE_WP_FLAG_HAS_POSITION);
    println!("  position_count={pos_count}/{}", base_waypoints.len());

    let mut writer = match writer::RouteWriter::open() {
        Ok(w) => w,
        Err(e) => {
            eprintln!("ERROR: {e}");
            std::process::exit(1);
        }
    };

    let mut seq: u64 = 0;
    loop {
        if invalidate {
            writer.publish_invalid();
            println!("published invalid/empty route snapshot");
        } else {
            let publish_uids = if arg_value("--uids").is_some() || arg_value("--uids-file").is_some() {
                uids.clone()
            } else {
                fake_uids(uids.len(), uids[0].wrapping_add(seq))
            };
            let publish = if sim_opts.with_positions
                || sim_opts.with_distance
                || sim_opts.with_time
                || !sim_opts.uid_positions.is_empty()
            {
                build_sim_waypoints(&publish_uids, &sim_opts)
            } else {
                publish_uids
                    .iter()
                    .map(|&uid| RouteWaypoint {
                        uid: uid as i64,
                        ..Default::default()
                    })
                    .collect()
            };
            writer.publish_waypoints(&publish);
            let hash = route_uid_hash(&publish);
            let pos_n = count_waypoints_with_flag(&publish, ROUTE_WP_FLAG_HAS_POSITION);
            println!(
                "published {} hash={:#x} position_count={pos_n}/{}",
                format_uid_summary(&publish_uids),
                hash,
                publish.len()
            );
        }
        if once {
            break;
        }
        seq = seq.wrapping_add(1);
        std::thread::sleep(Duration::from_secs(2));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_valid_uid_list() {
        let uids = parse_uid_list("101,202,303").unwrap();
        assert_eq!(uids, vec![101, 202, 303]);
    }

    #[test]
    fn parse_uid_list_with_spaces() {
        let uids = parse_uid_list("101, 202 , 303").unwrap();
        assert_eq!(uids, vec![101, 202, 303]);
    }

    #[test]
    fn parse_empty_uid_list_errors() {
        assert!(parse_uid_list("").is_err());
        assert!(parse_uid_list("  ,  ").is_err());
    }

    #[test]
    fn parse_invalid_uid_entry_errors() {
        assert!(parse_uid_list("101,abc,303").is_err());
    }

    #[test]
    fn parse_hex_uid() {
        let uids = parse_uid_list("0x10,0xFF").unwrap();
        assert_eq!(uids, vec![16, 255]);
    }

    #[test]
    fn build_sim_waypoints_sets_position_flags() {
        let uids = vec![101_u64, 202];
        let opts = SimWaypointOptions {
            with_positions: true,
            ..Default::default()
        };
        let wps = build_sim_waypoints(&uids, &opts);
        assert_eq!(wps.len(), 2);
        assert_ne!(wps[0].flags & ROUTE_WP_FLAG_HAS_POSITION, 0);
        assert!((wps[0].x - 0.0).abs() < 0.01);
        assert!((wps[1].z - 50.0).abs() < 0.01);
    }

    #[test]
    fn build_sim_waypoints_uid_pos_file_rows() {
        let uids = vec![1_u64, 2];
        let opts = SimWaypointOptions {
            uid_positions: vec![(2, 10.0, 20.0, 30.0)],
            ..Default::default()
        };
        let wps = build_sim_waypoints(&uids, &opts);
        assert_eq!(wps[0].flags & ROUTE_WP_FLAG_HAS_POSITION, 0);
        assert_eq!(wps[1].flags & ROUTE_WP_FLAG_HAS_POSITION, ROUTE_WP_FLAG_HAS_POSITION);
        assert!((wps[1].y - 20.0).abs() < 0.01);
    }

    #[test]
    fn build_sim_waypoints_distance_and_time_flags() {
        let uids = vec![1_u64, 2, 3];
        let opts = SimWaypointOptions {
            with_distance: true,
            with_time: true,
            ..Default::default()
        };
        let wps = build_sim_waypoints(&uids, &opts);
        assert_eq!(wps[0].flags & ROUTE_WP_FLAG_HAS_DISTANCE, ROUTE_WP_FLAG_HAS_DISTANCE);
        assert_eq!(wps[0].flags & ROUTE_WP_FLAG_HAS_TIME, ROUTE_WP_FLAG_HAS_TIME);
        assert!((wps[0].distance - 3000.0).abs() < 0.01);
    }

    #[test]
    fn short_uid_route_message_warns_at_two_and_three() {
        assert!(short_uid_route_message(2).is_some());
        assert!(short_uid_route_message(3).is_some());
        assert!(short_uid_route_message(4).is_none());
    }

    #[test]
    fn warn_short_uid_route_skips_synthetic_routes() {
        assert!(short_uid_route_message(2).is_some());
        // explicit=false → no panic; function is side-effect only on stderr
        warn_short_uid_route(2, false);
    }
}
