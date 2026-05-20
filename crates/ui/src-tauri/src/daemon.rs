//! Daemon lifecycle: spawn `truckpilot-core daemon` as a child process of the UI.
//!
//! Tradeoffs (Phase 6.5a Task 1):
//!  * Uses raw `std::process::Command` instead of Tauri's externalBin sidecar.
//!    Rationale: avoids the target-triple bundling dance during dev; binary
//!    resolution probes a small list of candidate paths.
//!  * Graceful shutdown is `Child::kill()` (Windows has no SIGTERM). The daemon
//!    has no IPC shutdown command yet (grep ipc-protocol — no Quit variant).
//!  * Zombie risk: if the UI process is hard-killed (not RunEvent::Exit), the
//!    daemon child is NOT reaped. A Win32 Job Object would fix this; deferred.

use std::io;
use std::net::{SocketAddr, TcpStream};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::Mutex;
use std::time::Duration;

use serde::{Deserialize, Serialize};
use tracing::{info, warn};

const DAEMON_PORT: u16 = 8765;
const PORT_PROBE_TIMEOUT: Duration = Duration::from_millis(500);
const POST_SPAWN_RETRIES: u32 = 3;
const POST_SPAWN_INTERVAL: Duration = Duration::from_millis(200);

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum DaemonState {
    /// We spawned the child and it's still alive.
    RunningManaged,
    /// Port 8765 answers but we didn't spawn it (pre-existing daemon).
    RunningExternal,
    /// Port closed and no managed child.
    Stopped,
    /// We had a managed child but it exited unexpectedly.
    Crashed,
}

#[derive(Debug, Clone, Serialize)]
pub struct DaemonStatus {
    pub state: DaemonState,
    pub pid: Option<u32>,
    pub binary_path: Option<String>,
    pub last_error: Option<String>,
}

pub struct DaemonManager {
    child: Mutex<Option<Child>>,
    binary: Mutex<Option<PathBuf>>,
    last_error: Mutex<Option<String>>,
}

impl DaemonManager {
    pub fn new() -> Self {
        Self {
            child: Mutex::new(None),
            binary: Mutex::new(None),
            last_error: Mutex::new(None),
        }
    }

    /// Probe the daemon's port. Cheap connect-then-drop; matches the bridge's
    /// own connect path so we avoid TOCTOU on `TcpListener::bind`.
    pub fn port_listening() -> bool {
        let addr: SocketAddr = ([127, 0, 0, 1], DAEMON_PORT).into();
        match TcpStream::connect_timeout(&addr, PORT_PROBE_TIMEOUT) {
            Ok(_) => {
                tracing::debug!("port :{DAEMON_PORT} probe → listening");
                true
            }
            Err(err) => {
                tracing::debug!("port :{DAEMON_PORT} probe → closed ({err})");
                false
            }
        }
    }

    pub fn status(&self) -> DaemonStatus {
        // Reap or detect crash of managed child.
        let mut managed_alive = false;
        let mut pid = None;
        {
            let mut guard = self.child.lock().unwrap();
            if let Some(child) = guard.as_mut() {
                match child.try_wait() {
                    Ok(None) => {
                        managed_alive = true;
                        pid = Some(child.id());
                    }
                    Ok(Some(status)) => {
                        warn!("daemon child exited: {status}");
                        *self.last_error.lock().unwrap() =
                            Some(format!("daemon exited with {status}"));
                        *guard = None;
                    }
                    Err(err) => {
                        warn!("try_wait on daemon child failed: {err}");
                        *guard = None;
                    }
                }
            }
        }

        let listening = Self::port_listening();
        let state = match (managed_alive, listening) {
            (true, _) => DaemonState::RunningManaged,
            (false, true) => DaemonState::RunningExternal,
            (false, false) => {
                if self.last_error.lock().unwrap().is_some() {
                    DaemonState::Crashed
                } else {
                    DaemonState::Stopped
                }
            }
        };

        DaemonStatus {
            state,
            pid,
            binary_path: self
                .binary
                .lock()
                .unwrap()
                .as_ref()
                .map(|p| p.display().to_string()),
            last_error: self.last_error.lock().unwrap().clone(),
        }
    }

    /// Start the daemon if no instance is already listening on the port.
    /// Returns the resulting status. Idempotent.
    pub fn start(&self) -> Result<DaemonStatus, String> {
        // Pre-spawn: explicit port-probe. If a daemon is already bound (from
        // another terminal or a previous run), do NOT spawn — that would
        // panic with AddrInUse.
        if Self::port_listening() {
            info!("daemon already listening on :{DAEMON_PORT} — external, skipping spawn");
            return Ok(self.status());
        }
        {
            let guard = self.child.lock().unwrap();
            if guard.is_some() {
                info!("managed daemon already tracked, skipping spawn");
                return Ok(self.status());
            }
        }

        let binary = locate_daemon_binary().map_err(|e| {
            let msg = format!("locate daemon binary: {e}");
            *self.last_error.lock().unwrap() = Some(msg.clone());
            msg
        })?;

        info!("spawning daemon: {}", binary.display());
        let mut child = Command::new(&binary)
            .arg("daemon")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .stdin(Stdio::null())
            .spawn()
            .map_err(|e| {
                let msg = format!("spawn daemon: {e}");
                *self.last_error.lock().unwrap() = Some(msg.clone());
                msg
            })?;

        // Post-spawn verification: poll the port up to N times. If it never
        // comes up, the daemon likely failed (e.g. AddrInUse race) — reap it
        // and surface the error instead of leaving a zombie marked managed.
        let mut listening = false;
        for attempt in 0..POST_SPAWN_RETRIES {
            std::thread::sleep(POST_SPAWN_INTERVAL);
            if let Ok(Some(exit)) = child.try_wait() {
                let msg = format!("daemon exited during startup with {exit}");
                warn!("{msg}");
                *self.last_error.lock().unwrap() = Some(msg.clone());
                *self.binary.lock().unwrap() = Some(binary);
                return Err(msg);
            }
            if Self::port_listening() {
                info!("daemon listening after probe {}", attempt + 1);
                listening = true;
                break;
            }
        }

        *self.binary.lock().unwrap() = Some(binary);
        if listening {
            *self.child.lock().unwrap() = Some(child);
            *self.last_error.lock().unwrap() = None;
        } else {
            // Port never came up — kill the orphan and report.
            let _ = child.kill();
            let _ = child.wait();
            let msg = format!(
                "daemon spawned but port :{DAEMON_PORT} never opened ({} probes)",
                POST_SPAWN_RETRIES
            );
            warn!("{msg}");
            *self.last_error.lock().unwrap() = Some(msg.clone());
            return Err(msg);
        }
        Ok(self.status())
    }

    /// Kill a managed child if present. No-op for external daemons.
    pub fn stop(&self) -> Result<DaemonStatus, String> {
        let mut guard = self.child.lock().unwrap();
        if let Some(mut child) = guard.take() {
            if let Err(err) = child.kill() {
                let msg = format!("kill daemon: {err}");
                *self.last_error.lock().unwrap() = Some(msg.clone());
                return Err(msg);
            }
            let _ = child.wait();
            info!("daemon child killed");
        }
        drop(guard);
        Ok(self.status())
    }

    pub fn restart(&self) -> Result<DaemonStatus, String> {
        self.stop()?;
        std::thread::sleep(Duration::from_millis(200));
        self.start()
    }

    /// Kill the child without re-acquiring the lock for status. Called from
    /// Tauri's exit hook where we want best-effort cleanup.
    pub fn shutdown_for_exit(&self) {
        let mut guard = self.child.lock().unwrap();
        if let Some(mut child) = guard.take() {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

impl Default for DaemonManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Probe candidate paths for `truckpilot-core[.exe]`.
///   1. `TRUCKPILOT_DAEMON_BIN` env override (absolute path).
///   2. Sibling of `current_exe()` (production bundle layout).
///   3. `<workspace>/target/release/truckpilot-core[.exe]` (dev).
///   4. `<workspace>/target/debug/truckpilot-core[.exe]` (dev).
fn locate_daemon_binary() -> io::Result<PathBuf> {
    let exe_name = if cfg!(windows) {
        "truckpilot-core.exe"
    } else {
        "truckpilot-core"
    };

    if let Ok(p) = std::env::var("TRUCKPILOT_DAEMON_BIN") {
        let path = PathBuf::from(p);
        if path.is_file() {
            return Ok(path);
        }
    }

    let current_exe = std::env::current_exe()?;
    if let Some(dir) = current_exe.parent() {
        let candidate = dir.join(exe_name);
        if candidate.is_file() {
            return Ok(candidate);
        }
        // Dev layout: current_exe is target/<profile>/truckpilot-ui[.exe].
        // truckpilot-core lives in the same directory.
        let workspace_target = climb_to_workspace_target(dir);
        for profile in ["release", "debug"] {
            if let Some(t) = &workspace_target {
                let c = t.join(profile).join(exe_name);
                if c.is_file() {
                    return Ok(c);
                }
            }
        }
    }

    Err(io::Error::new(
        io::ErrorKind::NotFound,
        format!("daemon binary '{exe_name}' not found near UI exe or in target/"),
    ))
}

/// From a path inside `target/...`, climb up to the `target/` directory itself.
fn climb_to_workspace_target(start: &Path) -> Option<PathBuf> {
    let mut cur = Some(start);
    while let Some(p) = cur {
        if p.file_name().and_then(|s| s.to_str()) == Some("target") {
            return Some(p.to_path_buf());
        }
        cur = p.parent();
    }
    None
}
