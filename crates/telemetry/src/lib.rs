//! TruckPilot Telemetry Layer
//!
//! Unified reader with priority cascade:
//!
//!   1. Memory Reading (most accurate, Windows-only, requires offsets)
//!   2. Shared Memory   (native plugin in ETS2 process)
//!   3. HTTP            (Funbit ETS2 Telemetry Server fallback)
//!
//! On startup the selected sources are initialised once and re-used. The
//! `TelemetryReader` is the public entry point — call `read()` once per
//! control cycle.

#![warn(missing_docs)]

pub mod http;
pub mod memory;
pub mod shm;

#[cfg(test)]
pub mod mock;

use tracing::{debug, info};
use truckpilot_plugin_api::Telemetry;

/// Identifies which source produced the most recent telemetry frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TelemetrySource {
    /// Read directly from the ETS2 process memory.
    Memory,
    /// Read from the native plugin's shared-memory region.
    SharedMemory,
    /// Polled from the Funbit telemetry HTTP server.
    Http,
    /// No source is currently producing data.
    None,
}

/// Errors that can occur during telemetry initialisation.
#[derive(Debug, thiserror::Error)]
pub enum TelemetryError {
    /// I/O error while opening a source.
    #[error("io: {0}")]
    Io(#[from] std::io::Error),

    /// Failed to attach to the ETS2 process for memory reading.
    #[error("memory: {0}")]
    Memory(String),

    /// Shared memory region could not be opened.
    #[error("shm: {0}")]
    Shm(String),
}

/// Configuration for the telemetry layer.
#[derive(Debug, Clone)]
pub struct TelemetryConfig {
    /// HTTP fallback URL.
    pub http_url: String,
    /// Path to the game-versions TOML for memory offsets.
    pub game_versions_path: Option<std::path::PathBuf>,
    /// Whether to attempt memory-reading at all.
    pub enable_memory: bool,
    /// Whether to attempt shared-memory at all.
    pub enable_shm: bool,
    /// Whether to attempt HTTP at all.
    pub enable_http: bool,
}

impl Default for TelemetryConfig {
    fn default() -> Self {
        Self {
            http_url: "http://localhost:25555/api/ets2/telemetry".to_string(),
            game_versions_path: None,
            enable_memory: true,
            enable_shm: true,
            enable_http: true,
        }
    }
}

/// Persistent telemetry reader. Holds long-lived handles (mmap, HTTP client,
/// process handle) so that each `read()` call is cheap.
pub struct TelemetryReader {
    /// Configuration the reader was constructed with. Kept for diagnostics.
    pub config: TelemetryConfig,
    shm: Option<shm::ShmReader>,
    memory: Option<memory::MemoryReader>,
    http: Option<http::HttpReader>,
    last_source: TelemetrySource,
}

impl TelemetryReader {
    /// Construct a new reader and try to initialise each enabled source.
    /// Failed sources are silently skipped; `read()` will fall through to
    /// the next available source.
    pub fn new(config: TelemetryConfig) -> Self {
        let memory = if config.enable_memory {
            match memory::MemoryReader::try_attach(config.game_versions_path.as_deref()) {
                Ok(m) => {
                    info!("Memory-reading source attached");
                    Some(m)
                }
                Err(e) => {
                    debug!("Memory-reading unavailable: {e}");
                    None
                }
            }
        } else {
            None
        };

        let shm = if config.enable_shm {
            match shm::ShmReader::open() {
                Ok(s) => {
                    info!("SHM telemetry source opened");
                    Some(s)
                }
                Err(e) => {
                    debug!("SHM unavailable: {e}");
                    None
                }
            }
        } else {
            None
        };

        let http = if config.enable_http {
            Some(http::HttpReader::new(config.http_url.clone()))
        } else {
            None
        };

        Self {
            config,
            memory,
            shm,
            http,
            last_source: TelemetrySource::None,
        }
    }

    /// Returns the source that produced the most recent successful read.
    pub fn last_source(&self) -> TelemetrySource {
        self.last_source
    }

    /// Read a telemetry frame from the highest-priority source available.
    /// Returns `None` if every source fails this cycle.
    pub fn read(&mut self) -> Option<Telemetry> {
        if let Some(m) = self.memory.as_mut() {
            if let Some(t) = m.read() {
                self.update_source(TelemetrySource::Memory);
                return Some(t);
            }
        }

        if let Some(s) = self.shm.as_mut() {
            if let Some(t) = s.read() {
                self.update_source(TelemetrySource::SharedMemory);
                return Some(t);
            }
        }

        if let Some(h) = self.http.as_mut() {
            if let Some(t) = h.read() {
                self.update_source(TelemetrySource::Http);
                return Some(t);
            }
        }

        self.update_source(TelemetrySource::None);
        None
    }

    fn update_source(&mut self, src: TelemetrySource) {
        if self.last_source != src {
            info!(
                "Telemetry source switched: {:?} -> {:?}",
                self.last_source, src
            );
            self.last_source = src;
        }
    }
}

/// Convenience one-shot reader. Creates a process-wide reader on first
/// call, reads once per call thereafter, never drops the reader.
///
/// **Blocking.** Reads memory, shared memory, and (as fallback) HTTP
/// synchronously. Do not call from inside an async task on a tokio
/// runtime — use [`read_telemetry_async`] instead, which dispatches the
/// work to `spawn_blocking`. This sync entry point is kept for
/// non-async callers (the `autopilot` subcommand, CLI probes, tests).
pub fn read_telemetry() -> Option<Telemetry> {
    static INIT: std::sync::OnceLock<std::sync::Mutex<TelemetryReader>> =
        std::sync::OnceLock::new();

    let reader = INIT
        .get_or_init(|| std::sync::Mutex::new(TelemetryReader::new(TelemetryConfig::default())));
    reader.lock().ok()?.read()
}

/// Async wrapper around [`read_telemetry`] that runs the blocking read
/// on tokio's blocking thread pool.
///
/// Why: the underlying `TelemetryReader::read` performs synchronous
/// `ReadProcessMemory` calls, memory-mapped reads, and (in the HTTP
/// fallback) a blocking `ureq` HTTP request. Calling that directly
/// from the 50 Hz core control loop would stall the tokio executor
/// for several milliseconds per tick, starving every other task on
/// the same worker (IPC server, plugin reload watcher, watchdog,
/// etc.). `spawn_blocking` moves the work onto a dedicated blocking
/// thread so the async runtime stays responsive.
///
/// Returns `None` if every source failed this cycle, or if the blocking
/// task panicked. Errors from the join are logged but not surfaced —
/// the next tick will retry from a fresh blocking task.
pub async fn read_telemetry_async() -> Option<Telemetry> {
    match tokio::task::spawn_blocking(read_telemetry).await {
        Ok(t) => t,
        Err(join_err) => {
            tracing::warn!("telemetry read task panicked or was cancelled: {join_err}");
            None
        }
    }
}
