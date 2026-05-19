//! TruckPilot IPC Protocol (versioned)
//!
//! Shared between Core (Rust) and UI (TypeScript).
//!
//! All messages serialize with `#[serde(tag = "type", rename_all = "snake_case")]`
//! so the wire form is `{"type": "<variant>", ...}`. Keep the TypeScript mirror at
//! `crates/ui/src/lib/types.ts` in sync — the `npm run sync-types` script verifies
//! this and fails CI on drift.

use std::collections::HashMap;

use serde::{Deserialize, Deserializer, Serialize, Serializer};

/// Serde helper: serialize `u64` as a JSON string, deserialize from either a
/// JSON string (decimal, or `0x…` hex) or a JSON number.
///
/// Background: JSON numbers in JavaScript are IEEE-754 f64, so values above
/// 2^53 lose precision. ETS2 node UIDs are full u64 (e.g. Berlin =
/// 282_353_445_640_339_601). Sending them through `serde_json` as numbers
/// from a JS client silently truncates. Sending as strings is exact.
///
/// Number-input is still accepted on deserialize so older tests and CLI tools
/// that pass numeric literals keep working.
pub mod u64_string {
    use super::*;

    pub fn serialize<S: Serializer>(value: &u64, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.collect_str(value)
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(deserializer: D) -> Result<u64, D::Error> {
        parse_u64(deserializer)
    }

    pub(super) fn parse_u64<'de, D: Deserializer<'de>>(deserializer: D) -> Result<u64, D::Error> {
        use serde::de::{Error, Visitor};
        use std::fmt;

        struct V;
        impl<'de> Visitor<'de> for V {
            type Value = u64;
            fn expecting(&self, f: &mut fmt::Formatter) -> fmt::Result {
                f.write_str("u64 as string (decimal or 0x-hex) or JSON number")
            }
            fn visit_u64<E: Error>(self, v: u64) -> Result<u64, E> {
                Ok(v)
            }
            fn visit_i64<E: Error>(self, v: i64) -> Result<u64, E> {
                u64::try_from(v).map_err(|_| E::custom("negative u64"))
            }
            fn visit_f64<E: Error>(self, v: f64) -> Result<u64, E> {
                if v.fract() != 0.0 || v < 0.0 || v > u64::MAX as f64 {
                    return Err(E::custom("non-integer JSON number for u64"));
                }
                Ok(v as u64)
            }
            fn visit_str<E: Error>(self, v: &str) -> Result<u64, E> {
                let trimmed = v.trim();
                if let Some(hex) = trimmed.strip_prefix("0x").or_else(|| trimmed.strip_prefix("0X")) {
                    u64::from_str_radix(hex, 16).map_err(E::custom)
                } else {
                    trimmed.parse::<u64>().map_err(E::custom)
                }
            }
            fn visit_string<E: Error>(self, v: String) -> Result<u64, E> {
                self.visit_str(&v)
            }
        }
        deserializer.deserialize_any(V)
    }
}

/// Companion of [`u64_string`] for `Option<u64>` fields.
pub mod option_u64_string {
    use super::*;

    pub fn serialize<S: Serializer>(value: &Option<u64>, serializer: S) -> Result<S::Ok, S::Error> {
        match value {
            Some(v) => serializer.collect_str(v),
            None => serializer.serialize_none(),
        }
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Option<u64>, D::Error> {
        use serde::de::{Error, Visitor};
        use std::fmt;

        struct V;
        impl<'de> Visitor<'de> for V {
            type Value = Option<u64>;
            fn expecting(&self, f: &mut fmt::Formatter) -> fmt::Result {
                f.write_str("optional u64 as string, number, or null")
            }
            fn visit_none<E: Error>(self) -> Result<Option<u64>, E> {
                Ok(None)
            }
            fn visit_unit<E: Error>(self) -> Result<Option<u64>, E> {
                Ok(None)
            }
            fn visit_some<D2: Deserializer<'de>>(self, d: D2) -> Result<Option<u64>, D2::Error> {
                super::u64_string::parse_u64(d).map(Some)
            }
        }
        deserializer.deserialize_option(V)
    }
}

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
    AutopilotStatus {
        v: u32,
        state: String,
        fault_reason: Option<String>,
        preconditions: PreconditionSnapshot,
        tick_count: u64,
    },
    BlackboardSnapshot {
        v: u32,
        values: HashMap<String, String>,
    },
    BlackboardKeys {
        v: u32,
        keys: Vec<String>,
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
    AutopilotEngage,
    AutopilotDisengage,
    AutopilotReset,
    SetRouterGoal {
        #[serde(with = "u64_string")]
        uid: u64,
    },
    SetRouterStart {
        #[serde(with = "option_u64_string", default)]
        uid: Option<u64>,
    },
    SetCruiseTarget {
        kmh: f32,
    },
    BlackboardGet {
        keys: Vec<String>,
    },
    BlackboardList {
        prefix: Option<String>,
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

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct PreconditionSnapshot {
    pub telemetry_ok: bool,
    pub engine_running: bool,
    pub cruise_active: bool,
    pub critical_plugins_loaded: bool,
    pub router_active: bool,
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
    fn autopilot_status_round_trip() {
        let msg = CoreMessage::AutopilotStatus {
            v: CoreMessage::VERSION,
            state: "Active".into(),
            fault_reason: None,
            preconditions: PreconditionSnapshot {
                telemetry_ok: true,
                engine_running: true,
                cruise_active: true,
                critical_plugins_loaded: true,
                router_active: false,
            },
            tick_count: 12_345,
        };
        let s = serde_json::to_string(&msg).unwrap();
        assert!(s.contains(r#""type":"autopilot_status""#));
        assert!(s.contains(r#""state":"Active""#));
        assert!(s.contains(r#""router_active":false"#));
        let back: CoreMessage = serde_json::from_str(&s).unwrap();
        match back {
            CoreMessage::AutopilotStatus {
                state,
                fault_reason,
                preconditions,
                tick_count,
                ..
            } => {
                assert_eq!(state, "Active");
                assert!(fault_reason.is_none());
                assert!(preconditions.telemetry_ok);
                assert!(!preconditions.router_active);
                assert_eq!(tick_count, 12_345);
            }
            _ => panic!("wrong variant"),
        }
    }

    #[test]
    fn autopilot_status_with_fault_reason() {
        let msg = CoreMessage::AutopilotStatus {
            v: CoreMessage::VERSION,
            state: "Fault".into(),
            fault_reason: Some("WatchdogStall".into()),
            preconditions: PreconditionSnapshot {
                telemetry_ok: false,
                engine_running: false,
                cruise_active: false,
                critical_plugins_loaded: true,
                router_active: false,
            },
            tick_count: 99,
        };
        let s = serde_json::to_string(&msg).unwrap();
        let back: CoreMessage = serde_json::from_str(&s).unwrap();
        if let CoreMessage::AutopilotStatus { fault_reason, .. } = back {
            assert_eq!(fault_reason.as_deref(), Some("WatchdogStall"));
        } else {
            panic!("wrong variant");
        }
    }

    #[test]
    fn ui_command_autopilot_engage_round_trip() {
        let cmd = UiCommand::AutopilotEngage;
        let s = serde_json::to_string(&cmd).unwrap();
        assert_eq!(s, r#"{"type":"autopilot_engage"}"#);
        let back: UiCommand = serde_json::from_str(&s).unwrap();
        assert!(matches!(back, UiCommand::AutopilotEngage));
    }

    #[test]
    fn ui_command_autopilot_disengage_reset_round_trip() {
        for (cmd, expected) in [
            (UiCommand::AutopilotDisengage, "autopilot_disengage"),
            (UiCommand::AutopilotReset, "autopilot_reset"),
        ] {
            let s = serde_json::to_string(&cmd).unwrap();
            assert!(s.contains(expected), "wire form for {cmd:?}: {s}");
            let _back: UiCommand = serde_json::from_str(&s).unwrap();
        }
    }

    #[test]
    fn ui_command_set_router_goal_round_trip() {
        let cmd = UiCommand::SetRouterGoal { uid: 12345 };
        let s = serde_json::to_string(&cmd).unwrap();
        assert!(s.contains(r#""type":"set_router_goal""#));
        // uid now serializes as a JSON string (not number) to preserve full u64.
        assert!(s.contains(r#""uid":"12345""#), "wire form: {s}");
        let back: UiCommand = serde_json::from_str(&s).unwrap();
        assert!(matches!(back, UiCommand::SetRouterGoal { uid: 12345 }));
    }

    /// Berlin's snap-node UID exceeds 2^53, which is the safe-integer ceiling
    /// for JS / serde_json::Number(f64). Verify that string transport keeps
    /// every bit and that the daemon decodes the exact same u64.
    #[test]
    fn ui_command_set_router_goal_preserves_high_u64() {
        const BERLIN_UID: u64 = 282_353_445_640_339_601;
        const HAMBURG_UID: u64 = 6_526_933_291_294_064_640;
        for uid in [BERLIN_UID, HAMBURG_UID, u64::MAX] {
            let cmd = UiCommand::SetRouterGoal { uid };
            let s = serde_json::to_string(&cmd).unwrap();
            assert!(s.contains(&format!(r#""uid":"{uid}""#)), "wire form: {s}");
            let back: UiCommand = serde_json::from_str(&s).unwrap();
            match back {
                UiCommand::SetRouterGoal { uid: parsed } => assert_eq!(parsed, uid),
                _ => panic!("wrong variant"),
            }
        }
    }

    /// Hex form must round-trip exactly through the deserializer.
    #[test]
    fn ui_command_set_router_goal_accepts_hex_string() {
        let raw = r#"{"type":"set_router_goal","uid":"0x3EB7A55B0000111"}"#;
        let cmd: UiCommand = serde_json::from_str(raw).unwrap();
        match cmd {
            UiCommand::SetRouterGoal { uid } => {
                assert_eq!(uid, 0x3EB7A55B0000111u64);
            }
            _ => panic!("wrong variant"),
        }
    }

    /// Numeric input is still accepted for compatibility with older callers
    /// (CLI tools, daemon-side tests). Loss only happens client-side in JS.
    #[test]
    fn ui_command_set_router_goal_accepts_numeric_for_compat() {
        let raw = r#"{"type":"set_router_goal","uid":42}"#;
        let cmd: UiCommand = serde_json::from_str(raw).unwrap();
        assert!(matches!(cmd, UiCommand::SetRouterGoal { uid: 42 }));
    }

    #[test]
    fn ui_command_set_router_start_round_trip() {
        let cmd = UiCommand::SetRouterStart { uid: Some(99) };
        let s = serde_json::to_string(&cmd).unwrap();
        assert!(s.contains(r#""uid":"99""#), "wire form: {s}");
        let back: UiCommand = serde_json::from_str(&s).unwrap();
        assert!(matches!(back, UiCommand::SetRouterStart { uid: Some(99) }));

        let cmd_none = UiCommand::SetRouterStart { uid: None };
        let s2 = serde_json::to_string(&cmd_none).unwrap();
        let back2: UiCommand = serde_json::from_str(&s2).unwrap();
        assert!(matches!(back2, UiCommand::SetRouterStart { uid: None }));
    }

    #[test]
    fn ui_command_set_router_start_preserves_high_u64() {
        const HAMBURG_UID: u64 = 6_526_933_291_294_064_640;
        let cmd = UiCommand::SetRouterStart { uid: Some(HAMBURG_UID) };
        let s = serde_json::to_string(&cmd).unwrap();
        let back: UiCommand = serde_json::from_str(&s).unwrap();
        match back {
            UiCommand::SetRouterStart { uid: Some(u) } => assert_eq!(u, HAMBURG_UID),
            other => panic!("wrong variant: {other:?}"),
        }
    }

    #[test]
    fn ui_command_set_cruise_target_round_trip() {
        let cmd = UiCommand::SetCruiseTarget { kmh: 85.0 };
        let s = serde_json::to_string(&cmd).unwrap();
        assert!(s.contains(r#""type":"set_cruise_target""#));
        let back: UiCommand = serde_json::from_str(&s).unwrap();
        match back {
            UiCommand::SetCruiseTarget { kmh } => assert!((kmh - 85.0).abs() < 1e-3),
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
