// Auto-generated mirror of crates/ipc-protocol/src/lib.rs
// Run `npm run sync-types` to regenerate after editing the Rust source.
// Manual edits will be overwritten.

export const PROTOCOL_VERSION = "1.0";

export interface TelemetrySnapshot {
  position: [number, number, number];
  heading: number;
  speed_ms: number;
  engine_rpm: number;
  cruise_control_kmh: number;
  nav_speed_limit_kmh: number;
}

export interface PluginInfo {
  name: string;
  version: string;
  enabled: boolean;
}

export interface ModInfo {
  name: string;
  path: string;
  enabled: boolean;
  hash: string;
}

export interface PidProfile {
  name: string;
  kp: number;
  ki: number;
  kd: number;
  output_limit: number;
}

export type PluginEventKind =
  | { kind: "loaded" }
  | { kind: "unloaded" }
  | { kind: "crashed"; reason: string }
  | { kind: "settings_changed" };

export type CoreMessage =
  | { type: "hello"; v: number; version: string }
  | { type: "telemetry"; v: number; data: TelemetrySnapshot }
  | { type: "plugin_list"; v: number; plugins: PluginInfo[] }
  | { type: "plugin_event"; v: number; plugin: string; event: PluginEventKind }
  | { type: "plugin_schema"; v: number; plugin: string; schema: unknown }
  | { type: "log"; v: number; level: LogLevel; message: string; plugin: string | null }
  | { type: "mod_list"; v: number; mods: ModInfo[] }
  | {
      type: "mod_build_progress";
      v: number;
      phase: string;
      percent: number;
      eta_seconds: number | null;
    }
  | { type: "mod_build_result"; v: number; ok: boolean; from_cache: boolean; message: string }
  | { type: "pid_profile_list"; v: number; profiles: PidProfile[] }
  | {
      type: "pid_sample";
      v: number;
      profile: string;
      setpoint: number;
      actual: number;
      t_ms: number;
    }
  | { type: "error"; v: number; code: string; command: string | null; message: string };

export type CoreMessageType = CoreMessage["type"];
export type CoreMessageOf<T extends CoreMessageType> = Extract<CoreMessage, { type: T }>;

export type UiCommand =
  | { type: "ping" }
  | { type: "request_plugin_list" }
  | { type: "plugin_toggle"; name: string; enabled: boolean }
  | { type: "plugin_reload"; name: string }
  | { type: "plugin_settings_update"; plugin: string; settings: unknown }
  | { type: "request_plugin_schema"; plugin: string }
  | { type: "request_mod_list" }
  | { type: "mod_apply"; active_mods: string[] }
  | { type: "cache_clear" }
  | { type: "request_pid_profiles" }
  | {
      type: "pid_profile_update";
      profile: string;
      kp: number;
      ki: number;
      kd: number;
      output_limit: number;
    }
  | { type: "pid_profile_reset"; profile: string }
  | { type: "pid_stream_subscribe"; profile: string; enabled: boolean }
  | { type: "set_log_subscription"; levels: LogLevel[]; plugin: string | null };

export type LogLevel = "trace" | "debug" | "info" | "warn" | "error";

export type ConnectionStatus = "connected" | "reconnecting" | "disconnected";

export interface ConnectionStatusEvent {
  status: ConnectionStatus;
  protocol_version: string | null;
  last_error: string | null;
}
