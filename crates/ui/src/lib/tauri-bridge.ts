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

export async function listenCoreEvent(handler: (msg: CoreMessage) => void): Promise<UnlistenFn> {
  return listen<CoreMessage>("core-event", (event) => handler(event.payload));
}

export async function listenConnectionStatus(
  handler: (status: ConnectionStatusEvent) => void
): Promise<UnlistenFn> {
  return listen<ConnectionStatusEvent>("connection-status", (event) => handler(event.payload));
}
