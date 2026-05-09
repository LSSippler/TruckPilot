//! ETS2 process memory reader.
//!
//! ## Status
//!
//! Skeleton implementation:
//!
//! - Static offsets per game version (loaded from `game_versions.toml`)
//! - Pointer-chain resolver helper
//! - `OffsetSource::Pattern` enum variant prepared but resolution not yet
//!   implemented (returns `None`).
//!
//! On Linux this module is a no-op stub — memory reading only ever runs
//! against a real ETS2 process on Windows.

use std::path::Path;

use serde::Deserialize;
use truckpilot_plugin_api::Telemetry;

/// How an offset is located in the target process.
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "kind", rename_all = "lowercase")]
pub enum OffsetSource {
    /// Static RVA from the module base. Brittle across game versions.
    Static {
        /// Offset value (added to the module base).
        offset: u64,
    },
    /// Pattern-scan signature. Pattern resolution will land in a later
    /// phase; this variant currently degrades to "not available".
    Pattern {
        /// Hex-byte pattern with `??` wildcards, e.g. "48 8B 05 ?? ?? ?? ??".
        signature: String,
    },
}

/// Game-version-specific addresses needed to extract telemetry.
#[derive(Debug, Clone, Deserialize, Default)]
pub struct GameOffsets {
    /// Game version this set of offsets applies to (e.g. "1.50.2").
    pub version: String,
    /// Truck position (3 × f64).
    pub position: Option<OffsetSource>,
    /// Heading (f64).
    pub heading: Option<OffsetSource>,
    /// Forward speed in m/s (f64).
    pub speed: Option<OffsetSource>,
    /// Engine RPM (f64).
    pub rpm: Option<OffsetSource>,
    /// Cruise-control set speed in km/h (f64).
    pub cruise_control: Option<OffsetSource>,
    /// Navigation speed limit in km/h (f64).
    pub nav_speed_limit: Option<OffsetSource>,
    /// Pointer chain leading to the navigation waypoint list (optional).
    pub nav_waypoints: Option<Vec<u64>>,
}

/// Wrapper for the contents of `game_versions.toml`.
#[derive(Debug, Clone, Deserialize, Default)]
pub struct GameVersionsFile {
    /// One entry per supported version.
    #[serde(default)]
    pub versions: Vec<GameOffsets>,
}

impl GameVersionsFile {
    /// Load the file from disk. Missing files yield an empty version list.
    pub fn load(path: &Path) -> Result<Self, String> {
        if !path.exists() {
            return Ok(Self::default());
        }
        let text = std::fs::read_to_string(path).map_err(|e| format!("read {path:?}: {e}"))?;
        toml::from_str(&text).map_err(|e| format!("parse {path:?}: {e}"))
    }
}

/// Persistent memory-reading source.
pub struct MemoryReader {
    inner: PlatformReader,
}

impl MemoryReader {
    /// Try to attach to the ETS2 process. Returns an error if the process is
    /// not running, the offsets file cannot be loaded, or the platform does
    /// not support memory reading.
    pub fn try_attach(game_versions: Option<&Path>) -> Result<Self, String> {
        let offsets = match game_versions {
            Some(p) => GameVersionsFile::load(p)?,
            None => GameVersionsFile::default(),
        };
        Ok(Self {
            inner: PlatformReader::attach(offsets)?,
        })
    }

    /// Read one telemetry frame. Returns `None` if reading fails this cycle
    /// (process gone, page fault, missing offsets).
    pub fn read(&mut self) -> Option<Telemetry> {
        self.inner.read()
    }
}

/// Resolve a pointer chain: read `*base`, then add `chain[1]`, read again,
/// etc. The last entry in `chain` is the final dereference offset.
///
/// `read_u64_at` is supplied by the platform layer.
pub fn resolve_pointer_chain<F>(base: u64, chain: &[u64], mut read_u64_at: F) -> Option<u64>
where
    F: FnMut(u64) -> Option<u64>,
{
    if chain.is_empty() {
        return Some(base);
    }
    let mut addr = base;
    for (i, off) in chain.iter().enumerate() {
        addr = addr.wrapping_add(*off);
        // Last entry is the leaf address — don't dereference.
        if i + 1 == chain.len() {
            return Some(addr);
        }
        addr = read_u64_at(addr)?;
    }
    Some(addr)
}

// ---------------------------------------------------------------------------
// Platform backends
// ---------------------------------------------------------------------------

#[cfg(windows)]
mod platform {
    use super::*;
    use windows::Win32::Foundation::{CloseHandle, HANDLE};
    use windows::Win32::System::Diagnostics::ToolHelp::{
        CreateToolhelp32Snapshot, Process32FirstW, Process32NextW, PROCESSENTRY32W,
        TH32CS_SNAPPROCESS,
    };
    use windows::Win32::System::Threading::{
        OpenProcess, PROCESS_QUERY_INFORMATION, PROCESS_VM_READ,
    };

    pub(super) struct PlatformReader {
        offsets: GameVersionsFile,
        process: HANDLE,
        module_base: u64,
    }

    unsafe impl Send for PlatformReader {}

    impl PlatformReader {
        pub fn attach(offsets: GameVersionsFile) -> Result<Self, String> {
            let pid = find_ets2_pid().ok_or_else(|| "eurotrucks2.exe not running".to_string())?;
            let process =
                unsafe { OpenProcess(PROCESS_VM_READ | PROCESS_QUERY_INFORMATION, false, pid) }
                    .map_err(|e| format!("OpenProcess({pid}) failed: {e}"))?;

            // Module base lookup is left as a stub — proper PEB-walking will
            // be added in a future phase. For now we use 0, which means
            // `Static` offsets can't yet resolve.
            Ok(Self {
                offsets,
                process,
                module_base: 0,
            })
        }

        pub fn read(&mut self) -> Option<Telemetry> {
            // No offsets, no module base ⇒ nothing to read yet.
            if self.module_base == 0 || self.offsets.versions.is_empty() {
                return None;
            }
            // TODO: pick the right version, resolve each offset, read fields.
            None
        }

        #[allow(dead_code)] // reserved for pointer-chain reads in future telemetry offsets
        fn read_u64(&self, addr: u64) -> Option<u64> {
            use windows::Win32::System::Diagnostics::Debug::ReadProcessMemory;
            let mut buf = [0u8; 8];
            let mut got = 0usize;
            let ok = unsafe {
                ReadProcessMemory(
                    self.process,
                    addr as *const _,
                    buf.as_mut_ptr() as *mut _,
                    buf.len(),
                    Some(&mut got),
                )
            };
            if ok.is_ok() && got == buf.len() {
                Some(u64::from_le_bytes(buf))
            } else {
                None
            }
        }
    }

    impl Drop for PlatformReader {
        fn drop(&mut self) {
            unsafe {
                let _ = CloseHandle(self.process);
            }
        }
    }

    fn find_ets2_pid() -> Option<u32> {
        unsafe {
            let snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0).ok()?;
            let mut entry = PROCESSENTRY32W {
                dwSize: std::mem::size_of::<PROCESSENTRY32W>() as u32,
                ..Default::default()
            };
            if Process32FirstW(snap, &mut entry).is_err() {
                let _ = CloseHandle(snap);
                return None;
            }
            loop {
                let name_end = entry
                    .szExeFile
                    .iter()
                    .position(|&c| c == 0)
                    .unwrap_or(entry.szExeFile.len());
                let name = String::from_utf16_lossy(&entry.szExeFile[..name_end]);
                if name.eq_ignore_ascii_case("eurotrucks2.exe") {
                    let pid = entry.th32ProcessID;
                    let _ = CloseHandle(snap);
                    return Some(pid);
                }
                if Process32NextW(snap, &mut entry).is_err() {
                    break;
                }
            }
            let _ = CloseHandle(snap);
            None
        }
    }
}

#[cfg(not(windows))]
mod platform {
    use super::*;

    pub(super) struct PlatformReader {
        _offsets: GameVersionsFile,
    }

    impl PlatformReader {
        pub fn attach(_offsets: GameVersionsFile) -> Result<Self, String> {
            Err("memory-reading is Windows-only".to_string())
        }

        pub fn read(&mut self) -> Option<Telemetry> {
            None
        }
    }
}

use platform::PlatformReader;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pointer_chain_empty_returns_base() {
        let result = resolve_pointer_chain(0xDEADBEEF, &[], |_| None);
        assert_eq!(result, Some(0xDEADBEEF));
    }

    #[test]
    fn pointer_chain_single_offset_no_deref() {
        // One entry = leaf offset, no dereference.
        let result = resolve_pointer_chain(0x1000, &[0x40], |_| panic!("must not deref"));
        assert_eq!(result, Some(0x1040));
    }

    #[test]
    fn pointer_chain_multi_step_dereferences() {
        // [0x10, 0x20, 0x08]:
        //   addr = base + 0x10 -> deref -> X
        //   addr = X + 0x20    -> deref -> Y
        //   addr = Y + 0x08    -> leaf
        let mut step = 0;
        let result = resolve_pointer_chain(0x1000, &[0x10, 0x20, 0x08], |addr| {
            step += 1;
            match step {
                1 => {
                    assert_eq!(addr, 0x1010);
                    Some(0x2000)
                }
                2 => {
                    assert_eq!(addr, 0x2020);
                    Some(0x3000)
                }
                _ => panic!("unexpected deref"),
            }
        });
        assert_eq!(result, Some(0x3008));
    }

    #[test]
    fn missing_offsets_file_yields_empty() {
        let path = std::path::PathBuf::from("/nonexistent/game_versions.toml");
        let f = GameVersionsFile::load(&path).unwrap();
        assert!(f.versions.is_empty());
    }

    #[test]
    fn parses_offsets_file() {
        let toml = r#"
[[versions]]
version = "1.50.2"
position = { kind = "static", offset = 0x12345 }
speed = { kind = "pattern", signature = "48 8B 05 ?? ?? ?? ??" }
nav_waypoints = [0x10, 0x20, 0x08]
"#;
        let f: GameVersionsFile = toml::from_str(toml).unwrap();
        assert_eq!(f.versions.len(), 1);
        assert_eq!(f.versions[0].version, "1.50.2");
        match f.versions[0].position.as_ref().unwrap() {
            OffsetSource::Static { offset } => assert_eq!(*offset, 0x12345),
            _ => panic!("expected Static"),
        }
        match f.versions[0].speed.as_ref().unwrap() {
            OffsetSource::Pattern { signature } => assert_eq!(signature, "48 8B 05 ?? ?? ?? ??"),
            _ => panic!("expected Pattern"),
        }
        assert_eq!(
            f.versions[0].nav_waypoints.as_ref().unwrap(),
            &[0x10, 0x20, 0x08]
        );
    }

    #[cfg(not(windows))]
    #[test]
    fn attach_fails_on_non_windows() {
        let err = MemoryReader::try_attach(None).err().unwrap();
        assert!(err.contains("Windows-only"));
    }
}
