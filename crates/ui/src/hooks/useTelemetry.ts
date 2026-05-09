import { useTelemetryStore } from "@/stores/telemetry";

export function useTelemetry() {
  return useTelemetryStore((s) => s.latest);
}
