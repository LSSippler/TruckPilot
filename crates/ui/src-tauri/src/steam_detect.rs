//! Cross-platform detection of an Euro Truck Simulator 2 install via Steam.
//!
//! Steam stores its main install path in `HKCU\Software\Valve\Steam\SteamPath`
//! (Windows) or `~/.steam/steam` / `~/.local/share/Steam` (Linux/macOS).
//! Each library is then declared in `<SteamPath>/steamapps/libraryfolders.vdf`.
//! ETS2 has Steam app id 227300 — when present in a library, an
//! `appmanifest_227300.acf` lives inside `<library>/steamapps/`.

use std::path::{Path, PathBuf};

const ETS2_APPID: &str = "227300";
const ETS2_FOLDER: &str = "Euro Truck Simulator 2";

pub fn detect_ets2_install() -> Option<String> {
    let steam_path = locate_steam_path()?;
    let libraries = parse_library_folders(&steam_path).unwrap_or_else(|| vec![steam_path.clone()]);
    for lib in libraries {
        let manifest = lib
            .join("steamapps")
            .join(format!("appmanifest_{ETS2_APPID}.acf"));
        if !manifest.exists() {
            continue;
        }
        let install = lib.join("steamapps").join("common").join(ETS2_FOLDER);
        if install.exists() {
            return install.to_str().map(|s| s.to_string());
        }
    }
    None
}

#[cfg(windows)]
fn locate_steam_path() -> Option<PathBuf> {
    use winreg::enums::HKEY_CURRENT_USER;
    use winreg::RegKey;

    let hkcu = RegKey::predef(HKEY_CURRENT_USER);
    let key = hkcu.open_subkey("Software\\Valve\\Steam").ok()?;
    let path: String = key.get_value("SteamPath").ok()?;
    let pb = PathBuf::from(path.replace('/', "\\"));
    if pb.exists() {
        Some(pb)
    } else {
        None
    }
}

#[cfg(target_os = "linux")]
fn locate_steam_path() -> Option<PathBuf> {
    let home = std::env::var_os("HOME")?;
    for candidate in [".steam/steam", ".local/share/Steam"] {
        let p = PathBuf::from(&home).join(candidate);
        if p.exists() {
            return Some(p);
        }
    }
    None
}

#[cfg(target_os = "macos")]
fn locate_steam_path() -> Option<PathBuf> {
    let home = std::env::var_os("HOME")?;
    let p = PathBuf::from(home).join("Library/Application Support/Steam");
    p.exists().then_some(p)
}

#[cfg(not(any(windows, target_os = "linux", target_os = "macos")))]
fn locate_steam_path() -> Option<PathBuf> {
    None
}

/// Parse `libraryfolders.vdf` and return all library root paths.
/// The VDF format is curly-brace nested key/value pairs; we only need string
/// values for the `path` keys, so a minimal regex-style scan is enough.
fn parse_library_folders(steam_path: &Path) -> Option<Vec<PathBuf>> {
    let path = steam_path.join("steamapps").join("libraryfolders.vdf");
    let content = std::fs::read_to_string(&path).ok()?;

    let mut paths = Vec::new();
    for line in content.lines() {
        let trimmed = line.trim();
        if let Some(rest) = trimmed.strip_prefix("\"path\"") {
            // expected:  "path"\t\t"C:\\SteamLibrary"
            if let Some(start) = rest.find('"') {
                let after = &rest[start + 1..];
                if let Some(end) = after.find('"') {
                    let raw = &after[..end];
                    let cleaned = raw.replace("\\\\", "\\");
                    paths.push(PathBuf::from(cleaned));
                }
            }
        }
    }

    if paths.is_empty() {
        None
    } else {
        Some(paths)
    }
}
