import { useEffect, useState } from "react";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { useTelemetryStore } from "@/stores/telemetry";
import { useConnectionStore } from "@/stores/connection";
import { usePluginsStore } from "@/stores/plugins";
import { sendCommand } from "@/lib/ipc";
import { SpeedGauge } from "@/components/telemetry/SpeedGauge";
import { HeadingCompass } from "@/components/telemetry/HeadingCompass";
import { MiniMap } from "@/components/telemetry/MiniMap";
import { Badge } from "@/components/ui/badge";

export function Dashboard() {
  const latest = useTelemetryStore((s) => s.latest);
  const lastUpdateMs = useTelemetryStore((s) => s.lastUpdateMs);
  const status = useConnectionStore((s) => s.status);
  const list = usePluginsStore((s) => s.list);
  const order = usePluginsStore((s) => s.order);

  useEffect(() => {
    if (status === "connected") {
      void sendCommand({ type: "request_plugin_list" });
    }
  }, [status]);

  const [now, setNow] = useState(() => Date.now());
  useEffect(() => {
    const id = window.setInterval(() => setNow(Date.now()), 1000);
    return () => window.clearInterval(id);
  }, []);
  const stale = lastUpdateMs > 0 && now - lastUpdateMs > 2_000;
  const activePlugins = order.map((name) => list.find((p) => p.name === name)).filter(
    (p): p is (typeof list)[number] => Boolean(p?.enabled)
  );

  if (status !== "connected") {
    return (
      <EmptyState title="Disconnected" body="Waiting for the TruckPilot core daemon on ws://localhost:8765." />
    );
  }
  if (!latest) {
    return <EmptyState title="No telemetry yet" body="Start ETS2 or wait for the first frame…" />;
  }

  return (
    <div className="grid gap-4 md:grid-cols-2 xl:grid-cols-3">
      <Card>
        <CardHeader>
          <CardTitle className="text-sm font-medium text-muted-foreground">Speed</CardTitle>
        </CardHeader>
        <CardContent className="flex items-center justify-center pb-6">
          <SpeedGauge speedMs={latest.speed_ms} navLimitKmh={latest.nav_speed_limit_kmh} />
        </CardContent>
      </Card>

      <Card>
        <CardHeader>
          <CardTitle className="text-sm font-medium text-muted-foreground">Heading</CardTitle>
        </CardHeader>
        <CardContent className="flex items-center justify-center pb-6">
          <HeadingCompass headingRad={latest.heading} />
        </CardContent>
      </Card>

      <Card>
        <CardHeader>
          <CardTitle className="text-sm font-medium text-muted-foreground">Position</CardTitle>
        </CardHeader>
        <CardContent>
          <MiniMap position={latest.position} />
          <dl className="mt-3 grid grid-cols-3 gap-2 text-xs text-muted-foreground">
            <div>
              <dt>X</dt>
              <dd className="font-mono text-foreground">{latest.position[0].toFixed(1)}</dd>
            </div>
            <div>
              <dt>Y</dt>
              <dd className="font-mono text-foreground">{latest.position[1].toFixed(1)}</dd>
            </div>
            <div>
              <dt>Z</dt>
              <dd className="font-mono text-foreground">{latest.position[2].toFixed(1)}</dd>
            </div>
          </dl>
        </CardContent>
      </Card>

      <Card className="md:col-span-2 xl:col-span-3">
        <CardHeader>
          <CardTitle className="text-sm font-medium text-muted-foreground">Active plugins</CardTitle>
        </CardHeader>
        <CardContent>
          {activePlugins.length === 0 ? (
            <p className="text-sm text-muted-foreground">No plugins enabled.</p>
          ) : (
            <ul className="flex flex-wrap gap-2">
              {activePlugins.map((p) => (
                <li key={p.name}>
                  <Badge variant="secondary">
                    {p.name} <span className="ml-2 text-muted-foreground">v{p.version}</span>
                  </Badge>
                </li>
              ))}
            </ul>
          )}
        </CardContent>
      </Card>

      {stale ? (
        <div className="md:col-span-2 xl:col-span-3 text-xs text-amber-500">
          No telemetry frame received in over 2 seconds.
        </div>
      ) : null}
    </div>
  );
}

function EmptyState({ title, body }: { title: string; body: string }) {
  return (
    <div className="flex h-full items-center justify-center">
      <Card className="max-w-md">
        <CardHeader>
          <CardTitle>{title}</CardTitle>
        </CardHeader>
        <CardContent>
          <p className="text-sm text-muted-foreground">{body}</p>
        </CardContent>
      </Card>
    </div>
  );
}
