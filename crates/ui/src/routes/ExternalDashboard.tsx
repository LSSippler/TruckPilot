import { useTelemetryStore } from "@/stores/telemetry";
import { useConnectionStore } from "@/stores/connection";
import { SpeedGauge } from "@/components/telemetry/SpeedGauge";
import { HeadingCompass } from "@/components/telemetry/HeadingCompass";
import { MiniMap } from "@/components/telemetry/MiniMap";

export function ExternalDashboard() {
  const latest = useTelemetryStore((s) => s.latest);
  const status = useConnectionStore((s) => s.status);

  if (status !== "connected" || !latest) {
    return (
      <div className="flex h-screen w-screen items-center justify-center bg-background text-muted-foreground">
        <span className="text-2xl">{status === "connected" ? "Waiting for telemetry…" : "Disconnected"}</span>
      </div>
    );
  }

  return (
    <div className="grid h-screen w-screen grid-cols-3 bg-background p-8 gap-8">
      <div className="flex items-center justify-center">
        <div className="scale-150">
          <SpeedGauge speedMs={latest.speed_ms} navLimitKmh={latest.nav_speed_limit_kmh} />
        </div>
      </div>
      <div className="flex items-center justify-center">
        <div className="scale-150">
          <HeadingCompass headingRad={latest.heading} />
        </div>
      </div>
      <div className="flex flex-col justify-center gap-4">
        <MiniMap position={latest.position} />
        <dl className="grid grid-cols-3 gap-2 text-base text-muted-foreground">
          <div>
            <dt>X</dt>
            <dd className="font-mono text-foreground">{latest.position[0].toFixed(0)}</dd>
          </div>
          <div>
            <dt>Y</dt>
            <dd className="font-mono text-foreground">{latest.position[1].toFixed(0)}</dd>
          </div>
          <div>
            <dt>Z</dt>
            <dd className="font-mono text-foreground">{latest.position[2].toFixed(0)}</dd>
          </div>
        </dl>
      </div>
    </div>
  );
}
