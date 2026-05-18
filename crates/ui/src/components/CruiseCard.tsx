import { useEffect, useState } from "react";
import { toast } from "sonner";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Button } from "@/components/ui/button";
import { Label } from "@/components/ui/label";
import { Slider } from "@/components/ui/slider";
import { sendCommand, subscribeBlackboardKeys } from "@/lib/ipc";
import { useBlackboardStore } from "@/stores/blackboard";
import { useTelemetryStore } from "@/stores/telemetry";

const CRUISE_KEYS = ["cruise.target_kmh", "telemetry.cruise_control_kmh"] as const;

export function CruiseCard() {
  const [target, setTarget] = useState(80);
  const values = useBlackboardStore((s) => s.values);
  const latest = useTelemetryStore((s) => s.latest);

  useEffect(() => subscribeBlackboardKeys(CRUISE_KEYS), []);

  const istKmh = latest ? latest.speed_ms * 3.6 : null;
  const ets2CruiseKmh = latest?.cruise_control_kmh ?? null;
  const blackboardTarget = Number(values["cruise.target_kmh"]);
  const effectiveTarget = Number.isFinite(blackboardTarget) ? blackboardTarget : null;

  const apply = () => {
    void sendCommand({ type: "set_cruise_target", kmh: target }).then(
      () => toast.success(`Cruise target ${target} km/h`),
      (err) => toast.error("Cruise failed", { description: String(err) }),
    );
  };

  return (
    <Card>
      <CardHeader>
        <CardTitle className="text-sm font-medium text-muted-foreground">Cruise</CardTitle>
      </CardHeader>
      <CardContent className="space-y-3">
        <div className="space-y-1.5">
          <div className="flex items-center justify-between">
            <Label className="text-xs">Target speed</Label>
            <span className="font-mono text-sm">{target} km/h</span>
          </div>
          <Slider
            value={[target]}
            min={30}
            max={130}
            step={5}
            onValueChange={(v) => setTarget(v[0] ?? 80)}
          />
          <Button className="w-full" size="sm" onClick={apply}>
            Apply
          </Button>
        </div>
        <div className="grid grid-cols-3 gap-2 rounded border bg-muted/40 p-2 text-xs">
          <Metric label="Ist" value={istKmh != null ? `${istKmh.toFixed(0)}` : "—"} unit="km/h" />
          <Metric
            label="Soll (UI)"
            value={effectiveTarget != null ? `${effectiveTarget.toFixed(0)}` : "—"}
            unit="km/h"
          />
          <Metric
            label="ETS2 CC"
            value={ets2CruiseKmh && ets2CruiseKmh > 0 ? ets2CruiseKmh.toFixed(0) : "off"}
            unit="km/h"
          />
        </div>
      </CardContent>
    </Card>
  );
}

function Metric({ label, value, unit }: { label: string; value: string; unit: string }) {
  return (
    <div className="flex flex-col">
      <span className="text-muted-foreground">{label}</span>
      <span className="font-mono">
        {value}
        {value !== "—" && value !== "off" && <span className="ml-1 text-muted-foreground">{unit}</span>}
      </span>
    </div>
  );
}
