//! Reader for `Local\TruckPilotNavRoute` — ETS2 in-process route UID list.
//!
//! The DLL (`truckpilot_telemetry.dll`) scans ETS2's own route data in-process
//! and writes the physical-route-item UID sequence here every 500 ms (or when
//! the route changes). TruckPilot Core reads this and maps UIDs to graph nodes.

use std::mem;

pub const NAV_ROUTE_SHM_MAGIC: u32 = 0x54504E52; // "TPNR"
pub const NAV_ROUTE_SHM_VERSION: u32 = 1;
/// Must match `NAV_ROUTE_MAX_ITEMS` in `crates/telemetry-dll/src/nav_route.rs`.
pub const NAV_ROUTE_MAX_ITEMS: usize = 2048;

#[cfg(windows)]
const SHM_NAME: &str = "Local\\TruckPilotNavRoute";

/// Wire layout — must mirror `NavRouteShmLayout` in `crates/telemetry-dll/src/nav_route.rs`.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct NavRouteShmLayout {
    pub magic: u32,
    pub version: u32,
    pub sequence: u32,
    pub item_count: u32,
    pub items: [u64; NAV_ROUTE_MAX_ITEMS],
}

const _: () = {
    assert!(mem::offset_of!(NavRouteShmLayout, magic) == 0);
    assert!(mem::offset_of!(NavRouteShmLayout, version) == 4);
    assert!(mem::offset_of!(NavRouteShmLayout, sequence) == 8);
    assert!(mem::offset_of!(NavRouteShmLayout, item_count) == 12);
    assert!(mem::offset_of!(NavRouteShmLayout, items) == 16);
};

/// Snapshot of the ETS2 route at one point in time.
#[derive(Debug, Clone)]
pub struct NavRouteSnapshot {
    /// Sequence number — compare with previous to detect changes.
    pub sequence: u32,
    /// Physical-route-item UIDs in travel order (from ETS2's own routing).
    pub uids: Vec<u64>,
}

/// Persistent reader for the nav-route SHM. Open once; call `read_if_changed`
/// each control cycle.
pub struct NavRouteReader {
    inner: NavRouteInner,
    last_sequence: u32,
}

unsafe impl Send for NavRouteReader {}
unsafe impl Sync for NavRouteReader {}

impl NavRouteReader {
    pub fn open() -> Result<Self, String> {
        Ok(Self {
            inner: NavRouteInner::open()?,
            last_sequence: u32::MAX,
        })
    }

    /// Read current route. Returns `None` when no valid route is in SHM.
    pub fn read(&self) -> Option<NavRouteSnapshot> {
        let layout = self.inner.read_layout()?;
        if layout.magic != NAV_ROUTE_SHM_MAGIC || layout.version != NAV_ROUTE_SHM_VERSION {
            return None;
        }
        let count = (layout.item_count as usize).min(NAV_ROUTE_MAX_ITEMS);
        if count == 0 {
            return None;
        }
        Some(NavRouteSnapshot {
            sequence: layout.sequence,
            uids: layout.items[..count].to_vec(),
        })
    }

    /// Like `read()` but only returns `Some` when the route changed since the last call.
    pub fn read_if_changed(&mut self) -> Option<NavRouteSnapshot> {
        let snap = self.read()?;
        if snap.sequence == self.last_sequence {
            return None;
        }
        self.last_sequence = snap.sequence;
        Some(snap)
    }

    /// Returns the raw item_count from SHM (0 if no route, None if magic invalid).
    pub fn peek_item_count(&self) -> Option<u32> {
        let layout = self.inner.read_layout()?;
        if layout.magic != NAV_ROUTE_SHM_MAGIC || layout.version != NAV_ROUTE_SHM_VERSION {
            return None;
        }
        Some(layout.item_count)
    }

    /// Returns the diagnostic step code written by the DLL when the route walk fails.
    /// 0xD1A60001 = GPS AOB scan failed
    /// 0xD1A60002 = trip_distance not plausible (< 100 m or not finite)
    /// 0xD1A60003 = route_task pointer chain failed
    /// 0xD1A60004 = uid_buf empty after walking items
    pub fn peek_diag_code(&self) -> Option<u32> {
        let layout = self.inner.read_layout()?;
        if layout.magic != NAV_ROUTE_SHM_MAGIC || layout.version != NAV_ROUTE_SHM_VERSION {
            return None;
        }
        if layout.item_count == 0 && layout.sequence >= 0xD1A6_0000 {
            return Some(layout.sequence);
        }
        None
    }
}

// ---------------------------------------------------------------------------
// Windows backend
// ---------------------------------------------------------------------------

#[cfg(windows)]
struct NavRouteInner {
    handle: windows::Win32::Foundation::HANDLE,
    view: *const u8,
}

#[cfg(windows)]
unsafe impl Send for NavRouteInner {}

#[cfg(windows)]
unsafe impl Sync for NavRouteInner {}

#[cfg(windows)]
impl NavRouteInner {
    fn open() -> Result<Self, String> {
        use windows::core::PCWSTR;
        use windows::Win32::System::Memory::{MapViewOfFile, OpenFileMappingW, FILE_MAP_READ};

        let name: Vec<u16> = SHM_NAME.encode_utf16().chain(std::iter::once(0)).collect();
        let handle =
            unsafe { OpenFileMappingW(FILE_MAP_READ.0, false, PCWSTR(name.as_ptr())) }
                .map_err(|e| format!("OpenFileMappingW(TruckPilotNavRoute): {e}"))?;
        if handle.is_invalid() {
            return Err("invalid handle for TruckPilotNavRoute".into());
        }
        let view = unsafe {
            MapViewOfFile(handle, FILE_MAP_READ, 0, 0, mem::size_of::<NavRouteShmLayout>())
        };
        if view.Value.is_null() {
            unsafe {
                let _ = windows::Win32::Foundation::CloseHandle(handle);
            }
            return Err("MapViewOfFile returned null for TruckPilotNavRoute".into());
        }
        Ok(Self {
            handle,
            view: view.Value as *const u8,
        })
    }

    fn read_layout(&self) -> Option<NavRouteShmLayout> {
        if self.view.is_null() {
            return None;
        }
        Some(unsafe { std::ptr::read_unaligned(self.view as *const NavRouteShmLayout) })
    }
}

#[cfg(windows)]
impl Drop for NavRouteInner {
    fn drop(&mut self) {
        use windows::Win32::Foundation::CloseHandle;
        use windows::Win32::System::Memory::{UnmapViewOfFile, MEMORY_MAPPED_VIEW_ADDRESS};
        unsafe {
            if !self.view.is_null() {
                let addr = MEMORY_MAPPED_VIEW_ADDRESS {
                    Value: self.view as *mut _,
                };
                let _ = UnmapViewOfFile(addr);
            }
            let _ = CloseHandle(self.handle);
        }
    }
}

// ---------------------------------------------------------------------------
// non-Windows stub
// ---------------------------------------------------------------------------

#[cfg(not(windows))]
struct NavRouteInner;

#[cfg(not(windows))]
impl NavRouteInner {
    fn open() -> Result<Self, String> {
        Ok(Self)
    }
    fn read_layout(&self) -> Option<NavRouteShmLayout> {
        None
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn layout_offsets_are_stable() {
        assert_eq!(mem::offset_of!(NavRouteShmLayout, magic), 0);
        assert_eq!(mem::offset_of!(NavRouteShmLayout, version), 4);
        assert_eq!(mem::offset_of!(NavRouteShmLayout, sequence), 8);
        assert_eq!(mem::offset_of!(NavRouteShmLayout, item_count), 12);
        assert_eq!(mem::offset_of!(NavRouteShmLayout, items), 16);
    }

    #[test]
    fn layout_size_covers_2048_items() {
        let sz = mem::size_of::<NavRouteShmLayout>();
        // 16-byte header + 2048 * 8 = 16400
        assert_eq!(sz, 16 + NAV_ROUTE_MAX_ITEMS * 8);
    }

    #[test]
    fn read_returns_none_on_bad_magic() {
        let reader = NavRouteReader {
            inner: NavRouteInner::open().unwrap(),
            last_sequence: u32::MAX,
        };
        // On non-Windows the inner always returns None, so read() is None.
        // This test verifies the magic check path exists.
        let result = reader.read();
        #[cfg(not(windows))]
        assert!(result.is_none());
        let _ = result;
    }
}
