import {
  listenConnectionStatus,
  listenCoreEvent,
  sendCommand as invokeSendCommand,
} from "@/lib/tauri-bridge";
import type { CoreMessage, CoreMessageOf, CoreMessageType, UiCommand } from "@/lib/types";
import { useConnectionStore } from "@/stores/connection";
import { useTelemetryStore } from "@/stores/telemetry";
import { usePluginsStore } from "@/stores/plugins";
import { useLogsStore, type LogEntry } from "@/stores/logs";
import { usePidStore } from "@/stores/pid";
import { useModsStore } from "@/stores/mods";
import { useAutopilotStore } from "@/stores/autopilot";
import { useBlackboardStore } from "@/stores/blackboard";

type AnyHandler = (msg: CoreMessage) => void;

// Log ingestion is coalesced via rAF so bursts (e.g. backlog flushes) trigger
// at most one setState per frame, not one per log line.
let pendingLogs: LogEntry[] = [];
let flushScheduled = false;

function enqueueLog(entry: LogEntry) {
  pendingLogs.push(entry);
  if (flushScheduled) return;
  flushScheduled = true;
  const schedule =
    typeof requestAnimationFrame !== "undefined"
      ? requestAnimationFrame
      : (cb: () => void) => setTimeout(cb, 16);
  schedule(() => {
    flushScheduled = false;
    if (pendingLogs.length === 0) return;
    const batch = pendingLogs;
    pendingLogs = [];
    useLogsStore.getState().pushMany(batch);
  });
}

// Exposed for tests / dev tools.
export function __flushPendingLogsForTest() {
  if (pendingLogs.length === 0) return;
  const batch = pendingLogs;
  pendingLogs = [];
  useLogsStore.getState().pushMany(batch);
}

const handlers = new Map<CoreMessageType, Set<AnyHandler>>();

export function subscribeToCoreEvents<T extends CoreMessageType>(
  type: T,
  handler: (msg: CoreMessageOf<T>) => void
): () => void {
  const set = handlers.get(type) ?? new Set<AnyHandler>();
  const wrapped = handler as AnyHandler;
  set.add(wrapped);
  handlers.set(type, set);
  return () => {
    set.delete(wrapped);
  };
}

function dispatch(msg: CoreMessage) {
  const set = handlers.get(msg.type);
  if (!set) return;
  for (const handler of set) {
    try {
      handler(msg);
    } catch (err) {
      console.error("[ipc] handler threw for", msg.type, err);
    }
  }
}

export async function sendCommand(cmd: UiCommand): Promise<void> {
  return invokeSendCommand(cmd);
}

function invokeRequestPluginList(): Promise<void> {
  return invokeSendCommand({ type: "request_plugin_list" });
}

export async function initIpcSubscriptions(): Promise<() => void> {
  const unlistenCore = await listenCoreEvent((msg) => {
    routeCoreMessage(msg);
    dispatch(msg);
  });

  const unlistenStatus = await listenConnectionStatus((status) => {
    useConnectionStore.getState().setStatus(status);
    if (status.status === "disconnected") {
      useAutopilotStore.getState().clear();
    }
  });

  const stopPoll = startBlackboardPoller();

  return () => {
    unlistenCore();
    unlistenStatus();
    stopPoll();
  };
}

// ── Blackboard polling ─────────────────────────────────────────────────────
//
// The daemon's IPC has no push-stream for blackboard values. We poll instead:
//
//  * `BlackboardList(null)` every 5 s refreshes the key inventory.
//  * `BlackboardGet([...subscribed])` every 500 ms refreshes interesting keys.
//
// Components register interest via `subscribeBlackboardKeys(keys)` so the
// poller only asks for what's actually rendered.

const subscribedKeys = new Map<string, number>();

export function subscribeBlackboardKeys(keys: readonly string[]): () => void {
  for (const k of keys) subscribedKeys.set(k, (subscribedKeys.get(k) ?? 0) + 1);
  return () => {
    for (const k of keys) {
      const n = (subscribedKeys.get(k) ?? 0) - 1;
      if (n <= 0) subscribedKeys.delete(k);
      else subscribedKeys.set(k, n);
    }
  };
}

function startBlackboardPoller(): () => void {
  const valueTick = setInterval(() => {
    if (useConnectionStore.getState().status !== "connected") return;
    if (subscribedKeys.size === 0) return;
    void invokeSendCommand({
      type: "blackboard_get",
      keys: Array.from(subscribedKeys.keys()),
    });
  }, 500);
  const keysTick = setInterval(() => {
    if (useConnectionStore.getState().status !== "connected") return;
    void invokeSendCommand({ type: "blackboard_list", prefix: null });
  }, 5_000);
  // Kick once on startup so the first paint isn't empty.
  setTimeout(() => {
    if (useConnectionStore.getState().status === "connected") {
      void invokeSendCommand({ type: "blackboard_list", prefix: null });
    }
  }, 250);
  return () => {
    clearInterval(valueTick);
    clearInterval(keysTick);
  };
}

function routeCoreMessage(msg: CoreMessage) {
  switch (msg.type) {
    case "hello":
      useConnectionStore.getState().setHello(msg.version, msg.v);
      break;
    case "telemetry":
      useTelemetryStore.getState().push(msg.data);
      break;
    case "plugin_list":
      usePluginsStore.getState().setList(msg.plugins);
      break;
    case "plugin_event":
      usePluginsStore.getState().applyEvent(msg.plugin, msg.event);
      if (msg.event.kind === "loaded") {
        void invokeRequestPluginList();
      }
      break;
    case "plugin_schema":
      usePluginsStore.getState().setSchema(msg.plugin, msg.schema);
      break;
    case "log":
      enqueueLog({
        level: msg.level,
        message: msg.message,
        plugin: msg.plugin,
        ts: Date.now(),
      });
      break;
    case "mod_list":
      useModsStore.getState().setList(msg.mods);
      break;
    case "mod_build_progress":
      useModsStore.getState().setProgress({
        phase: msg.phase,
        percent: msg.percent,
        etaSeconds: msg.eta_seconds,
      });
      break;
    case "mod_build_result":
      useModsStore.getState().setResult({
        ok: msg.ok,
        fromCache: msg.from_cache,
        message: msg.message,
      });
      break;
    case "pid_profile_list":
      usePidStore.getState().setProfiles(msg.profiles);
      break;
    case "pid_sample":
      usePidStore.getState().pushSample(msg.profile, {
        setpoint: msg.setpoint,
        actual: msg.actual,
        tMs: msg.t_ms,
      });
      break;
    case "autopilot_status":
      useAutopilotStore.getState().setStatus({
        state: msg.state,
        faultReason: msg.fault_reason,
        preconditions: msg.preconditions,
        tickCount: msg.tick_count,
      });
      break;
    case "error":
      console.warn("[ipc] core error:", msg);
      if (msg.command === "request_plugin_schema") {
        // Avoid hot-loop re-requests when a plugin has no schema.
        const lastSelected = usePluginsStore.getState().selected;
        if (lastSelected) usePluginsStore.getState().setSchema(lastSelected, null);
      }
      break;
    case "blackboard_snapshot":
      useBlackboardStore.getState().ingest(msg.values);
      break;
    case "blackboard_keys":
      useBlackboardStore.getState().setKeys(msg.keys);
      break;
    default: {
      const _exhaustive: never = msg;
      void _exhaustive;
    }
  }
}
