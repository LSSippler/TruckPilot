import { useBlackboardStore } from "@/stores/blackboard";

/// Blackboard keys the overlay panels render. Polled via the EXISTING
/// `subscribeBlackboardKeys` mechanism (lib/ipc.ts) — we don't invent a new
/// poll loop. Note: each Tauri webview has its own JS context, so this window
/// runs its own poller instance (a second blackboard_get stream); that's
/// inherent to multi-window and acceptable. Crucially there is still only ONE
/// daemon connection (the shared Rust IpcBridge). Every key is verified to be
/// written by the daemon/plugins (see outputs/overlay_telemetry_mapping.md).
export const OVERLAY_BB_KEYS = [
  "speed_controller.target_speed_kmh",
  "cruise.target_kmh",
  "lane_keeper.steering_out",
  "lane.center_offset",
  "lane_keeper.fallback_level",
  "lane_keeper.steering_source",
  "speed_controller.throttle_cmd",
  "speed_controller.brake_cmd",
  "router.active",
  "router.waypoint_count",
  "telemetry.engine_gear",
  "telemetry.reverse_gear",
  "telemetry.fuel_liters",
  "graph_ready",
  "spline_index_ready",
  "plugins_ready",
  "lane_detection_ready",
  "truckpilot_system_ready",
  "telemetry.available",
  "state.engage_precondition_telemetry_fresh",
  "state.engage_precondition_route_planned",
  "lane.confidence",
  "lane.left_visible",
  "lane.right_visible",
  "output.sink.configured",
  "vjoy.connected",
  "scs_sdk_output.connected",
  "preflight.resolver_safe",
] as const;

/// Subscribe to a single blackboard value (raw stringified form).
export function useBB(key: string): string | undefined {
  return useBlackboardStore((s) => s.values[key]);
}

/// Parse a blackboard string to a finite number, or null when absent/empty/NaN.
export function bbNum(v: string | undefined): number | null {
  if (v == null || v === "") return null;
  const n = Number(v);
  return Number.isFinite(n) ? n : null;
}

export function fmt0(n: number | null): string {
  return n == null ? "—" : n.toFixed(0);
}

export function fmt2(n: number | null): string {
  return n == null ? "—" : n.toFixed(2);
}

/** Tri-state display for daemon readiness blackboard keys (`true` / `false` / unknown). */
export function bbReadyTri(v: string | undefined): string {
  if (v === "true") return "yes";
  if (v === "false") return "no";
  return "unknown";
}
