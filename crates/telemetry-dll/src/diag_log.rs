//! Append-only diagnostic log for the ETS2 telemetry DLL.
//!
//! Log file: `<ETS2>/bin/win_x64/plugins/truckpilot_telemetry.log` (next to the DLL).
//! State changes are always logged; repeated identical messages are suppressed.

use std::io::Write;
use std::sync::Mutex;

static LOG_PATH: Mutex<Option<String>> = Mutex::new(None);
static LAST_LINE: Mutex<Option<String>> = Mutex::new(None);

#[cfg(windows)]
fn output_debug(msg: &str) {
    let s = format!("[TruckPilot] {msg}\0");
    unsafe {
        extern "system" {
            fn OutputDebugStringA(lpOutputString: *const u8);
        }
        OutputDebugStringA(s.as_ptr());
    }
}

#[cfg(not(windows))]
fn output_debug(_msg: &str) {}

#[cfg(windows)]
fn resolve_log_path() -> Option<String> {
    use std::ffi::c_void;
    use std::os::windows::ffi::OsStringExt;
    use std::path::PathBuf;

    type HMODULE = isize;
    type DWORD = u32;

    const GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS: DWORD = 0x0000_0004;
    const GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT: DWORD = 0x0000_0002;

    extern "system" {
        fn GetModuleHandleExW(
            dwFlags: DWORD,
            lpModuleName: *const c_void,
            phModule: *mut HMODULE,
        ) -> i32;
        fn GetModuleFileNameW(hModule: HMODULE, lpFilename: *mut u16, nSize: DWORD) -> DWORD;
    }

    unsafe {
        let anchor = resolve_log_path as *const c_void;
        let mut module: HMODULE = 0;
        if GetModuleHandleExW(
            GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
            anchor,
            &mut module,
        ) == 0
        {
            return None;
        }
        let mut buf = [0u16; 512];
        let n = GetModuleFileNameW(module, buf.as_mut_ptr(), buf.len() as DWORD);
        if n == 0 || n as usize >= buf.len() {
            return None;
        }
        let dll_path = PathBuf::from(std::ffi::OsString::from_wide(&buf[..n as usize]));
        let log = dll_path.with_file_name("truckpilot_telemetry.log");
        Some(log.to_string_lossy().into_owned())
    }
}

#[cfg(not(windows))]
fn resolve_log_path() -> Option<String> {
    None
}

fn lock_path() -> Option<std::sync::MutexGuard<'static, Option<String>>> {
    LOG_PATH.lock().ok()
}

fn lock_last() -> Option<std::sync::MutexGuard<'static, Option<String>>> {
    LAST_LINE.lock().ok()
}

/// Resolve log path once (no-op when already initialized). Never panics.
pub fn init() {
    let mut guard = match lock_path() {
        Some(g) => g,
        None => return,
    };
    if guard.is_some() {
        return;
    }
    *guard = resolve_log_path();
}

/// Earliest possible trace — DebugView + sidecar when path is known.
pub fn boot(msg: &str) {
    output_debug(msg);
    append_line(msg, true);
}

fn append_line(line: &str, force: bool) {
    init();
    let path = match lock_path().and_then(|g| g.clone()) {
        Some(p) => p,
        None => return,
    };

    if !force {
        if let Some(mut last) = lock_last() {
            if last.as_deref() == Some(line) {
                return;
            }
            *last = Some(line.to_string());
        }
    }

    let ts = {
        use std::time::{SystemTime, UNIX_EPOCH};
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_millis())
            .unwrap_or(0)
    };
    let full = format!("[{ts}] {line}\n");
    if let Ok(mut f) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&path)
    {
        let _ = f.write_all(full.as_bytes());
    }
}

/// Log a one-shot or rate-limited event (skips consecutive duplicates).
#[allow(dead_code)]
pub fn event(msg: &str) {
    append_line(msg, false);
}

/// Log without deduplication — use for init phases and errors.
pub fn event_force(msg: &str) {
    append_line(msg, true);
}

/// Log a state transition (never deduplicated against previous different keys).
pub fn state(key: &str, value: &str) {
    append_line(&format!("state {key}={value}"), true);
}

/// Directory containing `truckpilot_telemetry.dll` (plugin folder).
#[cfg(windows)]
pub fn plugin_dir() -> Option<std::path::PathBuf> {
    resolve_log_path().and_then(|p| {
        std::path::PathBuf::from(p)
            .parent()
            .map(std::path::Path::to_path_buf)
    })
}

#[cfg(not(windows))]
pub fn plugin_dir() -> Option<std::path::PathBuf> {
    None
}

#[cfg(test)]
pub fn set_log_path_for_test(path: Option<String>) {
    if let Some(mut guard) = lock_path() {
        *guard = path;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn init_is_idempotent() {
        init();
        init();
    }

    #[test]
    fn sidecar_writes_early_without_panic() {
        let dir = std::env::temp_dir().join(format!("tp-diag-log-{}", std::process::id()));
        let _ = std::fs::create_dir_all(&dir);
        let log_path = dir.join("truckpilot_telemetry.log");
        set_log_path_for_test(Some(log_path.to_string_lossy().into_owned()));
        boot("test boot line");
        event_force("test force line");
        let text = std::fs::read_to_string(&log_path).expect("log file");
        assert!(text.contains("test boot line"));
        assert!(text.contains("test force line"));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
