//! Snap-to-game + visibility loop for the HUD overlay window (Phase 6.5).
//!
//! Every 200 ms the overlay is shown while ETS2 is the FOREGROUND window and not
//! minimized, then moved/sized to exactly cover the ETS2 window. On minimize or
//! focus loss it is hidden. A short hysteresis (see `HYST_POLLS`) debounces
//! show/hide so brief focus changes don't flicker the overlay. The window is
//! created hidden, so it only ever appears under those conditions.
//!
//! NOTE (Phase 6.5b): hiding in the pause/main menu is intentionally NOT done
//! here. Both candidate signals were empirically disproven — the telemetry
//! `sequence` counter does NOT freeze in the pause menu (ETS2 keeps delivering
//! `frame_end`), and the SHM `paused` byte is never written by the DLL. A real
//! SCS `paused` event in `truckpilot-telemetry.dll` is the follow-up (Phase 6.5c).
//!
//! ALL Win32 calls live behind `#[cfg(target_os = "windows")]`; on every other
//! target (e.g. the Geekom Ubuntu build) `start` is a no-op so the UI crate still
//! compiles and links. The loop terminates once the overlay window closes.

#[cfg(target_os = "windows")]
pub fn start(app: tauri::AppHandle, label: &'static str) {
    use std::thread;
    use std::time::Duration;

    use tauri::{Manager, PhysicalPosition, PhysicalSize};
    use windows_sys::Win32::Foundation::{HWND, RECT};
    use windows_sys::Win32::UI::WindowsAndMessaging::{
        FindWindowW, GetForegroundWindow, GetWindowRect, IsIconic,
    };

    // ETS2's top-level window title (null class name → match by title only).
    let title: Vec<u16> = "Euro Truck Simulator 2"
        .encode_utf16()
        .chain(std::iter::once(0))
        .collect();

    const POLL: Duration = Duration::from_millis(200);
    // Hysteresis: require this many consecutive polls agreeing on the desired
    // visibility before toggling, to avoid flicker on brief focus changes.
    const HYST_POLLS: u32 = 2;

    thread::spawn(move || {
        // `shown` = the visibility we last commanded; `pending` counts polls in
        // which the desired state disagrees with `shown`.
        let mut shown = false;
        let mut pending: u32 = 0;

        loop {
            // Stop the loop once the overlay window no longer exists.
            let Some(window) = app.get_webview_window(label) else {
                break;
            };

            let mut rect_opt: Option<(i32, i32, u32, u32)> = None;
            // SAFETY: FindWindowW/GetForegroundWindow/GetWindowRect/IsIconic are
            // read-only Win32 lookups. A null HWND (ETS2 not running) is handled
            // explicitly. Pointer comparison is valid for opaque HWNDs.
            unsafe {
                let hwnd: HWND = FindWindowW(std::ptr::null(), title.as_ptr());
                if !hwnd.is_null() && IsIconic(hwnd) == 0 && GetForegroundWindow() == hwnd {
                    let mut rect = RECT {
                        left: 0,
                        top: 0,
                        right: 0,
                        bottom: 0,
                    };
                    if GetWindowRect(hwnd, &mut rect) != 0 {
                        let w = rect.right - rect.left;
                        let h = rect.bottom - rect.top;
                        if w > 0 && h > 0 {
                            rect_opt = Some((rect.left, rect.top, w as u32, h as u32));
                        }
                    }
                }
            }

            // Keep the overlay aligned to the game window whenever we have
            // geometry (cheap; keeps a soon-to-show window in place).
            if let Some((x, y, w, h)) = rect_opt {
                let _ = window.set_position(PhysicalPosition::new(x, y));
                let _ = window.set_size(PhysicalSize::new(w, h));
            }

            // Hysteresis: only flip show/hide after HYST_POLLS agree.
            let want = rect_opt.is_some();
            if want == shown {
                pending = 0;
            } else {
                pending += 1;
                if pending >= HYST_POLLS {
                    if want {
                        let _ = window.show();
                    } else {
                        let _ = window.hide();
                    }
                    shown = want;
                    pending = 0;
                }
            }

            thread::sleep(POLL);
        }
    });
}

/// Non-Windows stub: window-snapping/visibility gating is Windows-only. Keeps the
/// overlay where it was placed so the rest of the UI crate compiles/links.
#[cfg(not(target_os = "windows"))]
pub fn start(_app: tauri::AppHandle, _label: &'static str) {}
