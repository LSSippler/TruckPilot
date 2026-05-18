import { useEffect } from "react";
import { subscribeBlackboardKeys } from "@/lib/ipc";
import { useBlackboardStore } from "@/stores/blackboard";
import { VJoyBar } from "@/components/VJoyBar";
import { FailsafeBanner } from "@/components/FailsafeBanner";

const VJOY_KEYS = [
  "vjoy.connected",
  "vjoy.idle_centered",
  "vjoy.last_raw_x",
  "vjoy.last_raw_y",
  "vjoy.last_raw_z",
  "vjoy.last_raw_source",
  "vjoy.last_error",
  "vjoy.last_write_tick",
] as const;

const VJOY_MAX = 32767;
const VJOY_CENTER = 16384;

function steerNorm(raw: number): number {
  return (raw - VJOY_CENTER) / VJOY_CENTER;
}
function unitNorm(raw: number): number {
  return Math.max(0, Math.min(1, raw / VJOY_MAX));
}

export function VjoyMonitor() {
  const values = useBlackboardStore((s) => s.values);

  useEffect(() => subscribeBlackboardKeys(VJOY_KEYS), []);

  const rx = Number(values["vjoy.last_raw_x"] ?? "0");
  const ry = Number(values["vjoy.last_raw_y"] ?? "0");
  const rz = Number(values["vjoy.last_raw_z"] ?? "0");
  const connected = values["vjoy.connected"] === "true";
  const idle = values["vjoy.idle_centered"] === "true";
  const source = values["vjoy.last_raw_source"] ?? "—";
  const lastError = values["vjoy.last_error"];

  const steer = Number.isFinite(rx) ? steerNorm(rx) : 0;
  const throttle = Number.isFinite(ry) ? unitNorm(ry) : 0;
  const brake = Number.isFinite(rz) ? unitNorm(rz) : 0;
  const failsafeActive = idle && source === "watchdog";

  return (
    <div className="bg-surface-card border border-subtle rounded-md p-4 flex flex-col gap-3">
      <div className="flex items-center justify-between">
        <span className="text-fg-muted text-xs uppercase tracking-wider font-sans">vJoy output</span>
        <span
          className={
            connected
              ? "text-[10px] font-sans text-success"
              : "text-[10px] font-sans text-fg-muted"
          }
        >
          {connected ? "connected" : "disconnected"}
        </span>
      </div>

      {failsafeActive && (
        <FailsafeBanner
          reason="watchdog re-centered axes"
          hint="Autopilot output stopped. Check plugin state."
        />
      )}

      <div className="flex flex-col gap-2">
        <VJoyBar variant="bipolar"  label="Steering"  value={steer} />
        <VJoyBar variant="unipolar" label="Throttle"  value={throttle} accent="warning" />
        <VJoyBar variant="unipolar" label="Brake"     value={brake}    accent="danger" />
      </div>

      <div className="flex justify-between text-[10px] text-fg-muted font-mono">
        <span>source: {source}</span>
        {lastError && (
          <span className="text-danger truncate max-w-[60%]">err: {lastError}</span>
        )}
      </div>
    </div>
  );
}
