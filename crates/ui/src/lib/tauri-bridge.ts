import { invoke } from "@tauri-apps/api/core";
import { listen, type UnlistenFn } from "@tauri-apps/api/event";
import type { ConnectionStatusEvent, CoreMessage, UiCommand } from "@/lib/types";

export async function invokeCommand<T = void>(cmd: string, args?: Record<string, unknown>): Promise<T> {
  return invoke<T>(cmd, args);
}

export async function sendCommand(cmd: UiCommand): Promise<void> {
  return invokeCommand("send_command", { cmd });
}

export async function getConnectionStatus(): Promise<ConnectionStatusEvent> {
  return invokeCommand<ConnectionStatusEvent>("get_connection_status");
}

export async function reconnect(): Promise<void> {
  return invokeCommand("reconnect");
}

export async function detectEts2Path(): Promise<string | null> {
  return invokeCommand<string | null>("detect_ets2_path");
}

export async function openExternalDashboard(): Promise<void> {
  return invokeCommand("open_external_dashboard");
}

export async function toggleOverlay(): Promise<void> {
  return invokeCommand("toggle_overlay");
}

/** Enable overlay layout editor: window accepts mouse input (Tauri overlay only). */
export async function overlaySetLayoutEditor(enabled: boolean): Promise<void> {
  return invokeCommand("overlay_set_layout_editor", { enabled });
}

export type DaemonState = "runningmanaged" | "runningexternal" | "stopped" | "crashed";

export interface DaemonStatus {
  state: DaemonState;
  pid: number | null;
  binary_path: string | null;
  last_error: string | null;
}

export async function daemonStatus(): Promise<DaemonStatus> {
  return invokeCommand<DaemonStatus>("daemon_status");
}

export async function daemonStart(): Promise<DaemonStatus> {
  return invokeCommand<DaemonStatus>("daemon_start");
}

export async function daemonStop(): Promise<DaemonStatus> {
  return invokeCommand<DaemonStatus>("daemon_stop");
}

export async function daemonRestart(): Promise<DaemonStatus> {
  return invokeCommand<DaemonStatus>("daemon_restart");
}

export async function daemonGetAutoStart(): Promise<boolean> {
  return invokeCommand<boolean>("daemon_get_auto_start");
}

export async function daemonSetAutoStart(enabled: boolean): Promise<void> {
  return invokeCommand("daemon_set_auto_start", { enabled });
}

export interface HotkeyConfig {
  engage: string;
  disengage: string;
}

export async function hotkeyGetConfig(): Promise<HotkeyConfig> {
  return invokeCommand<HotkeyConfig>("hotkey_get_config");
}

export async function hotkeySetConfig(engage: string, disengage: string): Promise<void> {
  return invokeCommand("hotkey_set_config", { engage, disengage });
}

export async function listenCoreEvent(handler: (msg: CoreMessage) => void): Promise<UnlistenFn> {
  return listen<CoreMessage>("core-event", (event) => handler(event.payload));
}

export async function listenConnectionStatus(
  handler: (status: ConnectionStatusEvent) => void
): Promise<UnlistenFn> {
  return listen<ConnectionStatusEvent>("connection-status", (event) => handler(event.payload));
}
