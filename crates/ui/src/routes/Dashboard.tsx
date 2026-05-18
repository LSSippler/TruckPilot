import { useEffect, useMemo, useState } from "react";
import { Badge } from "@/components/ui/badge";
import { useTelemetryStore } from "@/stores/telemetry";
import { useConnectionStore } from "@/stores/connection";
import { useBlackboardStore } from "@/stores/blackboard";
import { subscribeBlackboardKeys } from "@/lib/ipc";
import { AutopilotStatusCard } from "@/components/AutopilotStatusCard";
import { RouteCard } from "@/components/RouteCard";
import { CruiseCard } from "@/components/CruiseCard";
import { VjoyMonitor } from "@/components/VjoyMonitor";
import { BigNumberDisplay } from "@/components/BigNumberDisplay";
import { TelemetrySparkline } from "@/components/TelemetrySparkline";
import { MiniMap } from "@/components/telemetry/MiniMap";
import { HeadingCompass } from "@/components/telemetry/HeadingCompass";

const TELE_KEYS = ["telemetry.available"] as const;

export function Dashboard() {
  const latest = useTelemetryStore((s) => s.latest);
  const lastUpdateMs = useTelemetryStore((s) => s.lastUpdateMs);
  const history = useTelemetryStore((s) => s.history);
  const status = useConnectionStore((s) => s.status);
  const bbValues = useBlackboardStore((s) => s.values);

  useEffect(() => subscribeBlackboardKeys(TELE_KEYS), []);

  const [now, setNow] = useState(() => Date.now());
  useEffect(() => {
    const id = window.setInterval(() => setNow(Date.now()), 1000);
    return () => window.clearInterval(id);
  }, []);
  const stale = lastUpdateMs > 0 && now - lastUpdateMs > 2_000;
  const teleAvailable = bbValues["telemetry.available"] === "true";

  const speedSeries = useMemo(
    () => history.map((h) => ({ t: h.tMs, v: h.speed_ms * 3.6 })),
    [history],
  );
  const rpmSeries = useMemo(
    () => history.map((h) => ({ t: h.tMs, v: h.engine_rpm })),
    [history],
  );

  return (
    <div className="h-full overflow-y-auto">
      {/* HERO — autopilot decision + route goal */}
      <section className="border-b border-subtle p-4">
        <div className="grid grid-cols-12 gap-3">
          <div className="col-span-12 lg:col-span-5">
            <AutopilotStatusCard />
          </div>
          <div className="col-span-12 lg:col-span-7">
            <RouteCard />
          </div>
        </div>
      </section>

      {/* LIVE — telemetry values, vjoy output */}
      {status !== "connected" ? (
        <section className="p-4">
          <EmptyState
            title="Disconnected"
            body="Waiting for the TruckPilot core daemon on ws://localhost:8765."
          />
        </section>
      ) : !latest ? (
        <section className="p-4 flex flex-col gap-3">
          <TelemetryEmpty teleAvailable={teleAvailable} />
          <VjoyMonitor />
        </section>
      ) : (
        <>
          <section className="p-4">
            <div className="grid grid-cols-12 gap-3">
              {/* Speed */}
              <div className="col-span-12 sm:col-span-6 lg:col-span-3 bg-surface-card border border-subtle rounded-md p-4 flex flex-col gap-2">
                <BigNumberDisplay
                  label="Speed"
                  value={latest.speed_ms * 3.6}
                  unit="km/h"
                  target={latest.nav_speed_limit_kmh > 0 ? latest.nav_speed_limit_kmh : null}
                />
                {speedSeries.length > 1 && (
                  <TelemetrySparkline data={speedSeries} accent="brand" unit="km/h" />
                )}
              </div>

              {/* Engine */}
              <div className="col-span-12 sm:col-span-6 lg:col-span-3 bg-surface-card border border-subtle rounded-md p-4 flex flex-col gap-2">
                <BigNumberDisplay
                  label="RPM"
                  value={latest.engine_rpm}
                  unit="rpm"
                />
                {rpmSeries.length > 1 && (
                  <TelemetrySparkline data={rpmSeries} accent="chart-2" unit="rpm" />
                )}
                <div className="flex items-center justify-between">
                  <span className="text-xs font-sans text-fg-muted">engine</span>
                  <Badge
                    variant="secondary"
                    className={
                      latest.engine_rpm > 100
                        ? "bg-success-soft text-success border-0"
                        : "bg-surface-elevated text-fg-muted border-0"
                    }
                  >
                    {latest.engine_rpm > 100 ? "running" : "off"}
                  </Badge>
                </div>
                <div className="grid grid-cols-2 gap-1 text-[10px] font-mono text-fg-muted">
                  <span>cruise</span>
                  <span className="text-right text-fg">
                    {latest.cruise_control_kmh > 0 ? `${latest.cruise_control_kmh.toFixed(0)} km/h` : "off"}
                  </span>
                  <span>nav limit</span>
                  <span className="text-right text-fg">
                    {latest.nav_speed_limit_kmh > 0 ? `${latest.nav_speed_limit_kmh.toFixed(0)}` : "—"}
                  </span>
                </div>
              </div>

              {/* vJoy */}
              <div className="col-span-12 lg:col-span-6">
                <VjoyMonitor />
              </div>
            </div>
          </section>

          {/* DETAIL — cruise + position/heading */}
          <section className="px-4 pb-4">
            <div className="grid grid-cols-12 gap-3">
              <div className="col-span-12 lg:col-span-4">
                <CruiseCard />
              </div>
              <div className="col-span-12 lg:col-span-8 bg-surface-card border border-subtle rounded-md p-4">
                <span className="text-fg-muted text-xs uppercase tracking-wider font-sans block mb-3">
                  Position
                </span>
                <div className="flex flex-col gap-3 sm:flex-row sm:gap-4">
                  <MiniMap position={latest.position} />
                  <div className="flex flex-col gap-3">
                    <HeadingCompass headingRad={latest.heading} />
                    <dl className="grid grid-cols-3 gap-2 text-xs text-fg-muted font-mono">
                      <Coord label="X" value={latest.position[0]} />
                      <Coord label="Y" value={latest.position[1]} />
                      <Coord label="Z" value={latest.position[2]} />
                    </dl>
                  </div>
                </div>
              </div>
            </div>
          </section>

          {stale && (
            <div className="px-4 pb-4 text-xs text-warning">
              No telemetry frame received in over 2 seconds.
            </div>
          )}
        </>
      )}
    </div>
  );
}

function Coord({ label, value }: { label: string; value: number }) {
  return (
    <div>
      <dt className="text-fg-muted">{label}</dt>
      <dd className="text-fg">{value.toFixed(1)}</dd>
    </div>
  );
}

function TelemetryEmpty({ teleAvailable }: { teleAvailable: boolean }) {
  if (teleAvailable) {
    return (
      <div className="bg-surface-card border border-subtle rounded-md p-4">
        <p className="text-sm font-sans text-fg font-medium mb-1">Awaiting first telemetry frame…</p>
        <p className="text-xs text-fg-muted font-sans">
          Daemon reports telemetry is available. Waiting for the first frame to arrive.
        </p>
      </div>
    );
  }
  return (
    <div className="bg-surface-card border border-subtle rounded-md p-4">
      <p className="text-sm font-sans text-fg font-medium mb-2">ETS2 not running</p>
      <p className="text-xs text-fg-muted font-sans mb-2">
        The daemon is connected but no telemetry source is active.
      </p>
      <ol className="ml-4 list-decimal text-xs text-fg-muted font-sans space-y-1">
        <li>Start Euro Truck Simulator 2.</li>
        <li>
          Ensure <code className="font-mono">truckpilot_telemetry.dll</code> is installed in{" "}
          <code className="font-mono">bin/win_x64/plugins/</code>.
        </li>
        <li>Load any save and the daemon should detect the SHM segment within ~1 s.</li>
      </ol>
    </div>
  );
}

function EmptyState({ title, body }: { title: string; body: string }) {
  return (
    <div className="bg-surface-card border border-subtle rounded-md p-4">
      <p className="text-sm font-sans text-fg font-medium mb-1">{title}</p>
      <p className="text-xs text-fg-muted font-sans">{body}</p>
    </div>
  );
}
