//! TruckPilot IPC Protocol (versioned)
//!
//! Shared between Core (Rust) and UI (TypeScript).
//!
//! All messages serialize with `#[serde(tag = "type", rename_all = "snake_case")]`
//! so the wire form is `{"type": "<variant>", ...}`. Keep the TypeScript mirror at
//! `crates/ui/src/lib/types.ts` in sync — the `npm run sync-types` script verifies
//! this and fails CI on drift.

use serde::{Deserialize, Serialize};

pub const PROTOCOL_VERSION: &str = "1.0";

/// Messages sent from Core to UI (push)
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum CoreMessage {
    Hello {
        v: u32,
        version: String,
    },
    Telemetry {
        v: u32,
        data: TelemetrySnapshot,
    },
    PluginList {
        v: u32,
        plugins: Vec<PluginInfo>,
    },
    PluginEvent {
        v: u32,
        plugin: String,
        event: PluginEventKind,
    },
    PluginSchema {
        v: u32,
        plugin: String,
        schema: serde_json::Value,
    },
    Log {
        v: u32,
        level: String,
        message: String,
        plugin: Option<String>,
    },
    ModList {
        v: u32,
        mods: Vec<ModInfo>,
    },
    ModBuildProgress {
        v: u32,
        phase: String,
        percent: f32,
        eta_seconds: Option<u32>,
    },
    ModBuildResult {
        v: u32,
        ok: bool,
        from_cache: bool,
        message: String,
    },
    PidProfileList {
        v: u32,
        profiles: Vec<PidProfile>,
    },
    PidSample {
        v: u32,
        profile: String,
        setpoint: f64,
        actual: f64,
        t_ms: u64,
    },
    Error {
        v: u32,
        code: String,
        command: Option<String>,
        message: String,
    },
}

/// Messages sent from UI to Core (commands)
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum UiCommand {
    Ping,
    RequestPluginList,
    PluginToggle {
        name: String,
        enabled: bool,
    },
    PluginReload {
        name: String,
    },
    PluginSettingsUpdate {
        plugin: String,
        settings: serde_json::Value,
    },
    RequestPluginSchema {
        plugin: String,
    },
    RequestModList,
    ModApply {
        active_mods: Vec<String>,
    },
    CacheClear,
    RequestPidProfiles,
    PidProfileUpdate {
        profile: String,
        kp: f64,
        ki: f64,
        kd: f64,
        output_limit: f64,
    },
    PidProfileReset {
        profile: String,
    },
    PidStreamSubscribe {
        profile: String,
        enabled: bool,
    },
    SetLogSubscription {
        levels: Vec<String>,
        plugin: Option<String>,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TelemetrySnapshot {
    pub position: [f64; 3],
    pub heading: f64,
    pub speed_ms: f64,
    pub engine_rpm: f64,
    pub cruise_control_kmh: f64,
    pub nav_speed_limit_kmh: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PluginInfo {
    pub name: String,
    pub version: String,
    pub enabled: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum PluginEventKind {
    Loaded,
    Unloaded,
    Crashed { reason: String },
    SettingsChanged,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ModInfo {
    pub name: String,
    pub path: String,
    pub enabled: bool,
    pub hash: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PidProfile {
    pub name: String,
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub output_limit: f64,
}

impl CoreMessage {
    pub const VERSION: u32 = 1;

    pub fn hello() -> Self {
        Self::Hello {
            v: Self::VERSION,
            version: PROTOCOL_VERSION.to_string(),
        }
    }

    pub fn error(
        code: impl Into<String>,
        command: Option<String>,
        message: impl Into<String>,
    ) -> Self {
        Self::Error {
            v: Self::VERSION,
            code: code.into(),
            command,
            message: message.into(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hello_round_trip() {
        let msg = CoreMessage::hello();
        let s = serde_json::to_string(&msg).unwrap();
        assert!(s.contains(r#""type":"hello""#));
        let back: CoreMessage = serde_json::from_str(&s).unwrap();
        match back {
            CoreMessage::Hello { v, version } => {
                assert_eq!(v, 1);
                assert_eq!(version, PROTOCOL_VERSION);
            }
            _ => panic!("wrong variant"),
        }
    }

    #[test]
    fn ui_command_pid_update_round_trip() {
        let cmd = UiCommand::PidProfileUpdate {
            profile: "steering".into(),
            kp: 1.0,
            ki: 0.0,
            kd: 0.05,
            output_limit: 1.0,
        };
        let s = serde_json::to_string(&cmd).unwrap();
        assert!(s.contains(r#""type":"pid_profile_update""#));
        let back: UiCommand = serde_json::from_str(&s).unwrap();
        match back {
            UiCommand::PidProfileUpdate { profile, kp, .. } => {
                assert_eq!(profile, "steering");
                assert!((kp - 1.0).abs() < 1e-9);
            }
            _ => panic!("wrong variant"),
        }
    }
}
