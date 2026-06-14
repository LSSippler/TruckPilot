import { useTelemetryStore } from "@/stores/telemetry";
import { msToKmh } from "@/lib/utils";
import { Panel, Row } from "./Panel";
import { useBB, bbNum, fmt0 } from "./overlay-lib";

function gearLabel(gear: number | null, reverse: boolean): string {
  if (reverse || (gear != null && gear < 0)) return "R";
  if (gear == null) return "—";
  if (gear === 0) return "N";
  return String(gear);
}

/// Rechts oben: Vehicle — Speed, Gang, Fuel, Cruise Control.
/// Gang/Fuel kommen NICHT im 20 Hz-Push, sondern nur als Blackboard-Pull-Keys
/// (telemetry.engine_gear / telemetry.fuel_liters). `fuel_liters` < 0 oder
/// abwesend bedeutet „nicht verfügbar" → als „--" zeigen.
export function VehiclePanel() {
  const latest = useTelemetryStore((s) => s.latest);
  const gear = bbNum(useBB("telemetry.engine_gear"));
  const reverse = useBB("telemetry.reverse_gear") === "true";
  const fuel = bbNum(useBB("telemetry.fuel_liters"));
  const cruise = latest?.cruise_control_kmh ?? 0;

  return (
    <Panel title="Vehicle" className="min-w-[13rem]">
      <Row label="Speed" value={latest ? `${fmt0(msToKmh(latest.speed_ms))} km/h` : "—"} />
      <Row
        label="Gang"
        value={gearLabel(gear, reverse)}
        hint="Blackboard-Pull telemetry.engine_gear (nicht im 20 Hz-Push)"
      />
      <Row
        label="Fuel"
        value={fuel == null || fuel < 0 ? "--" : `${fmt0(fuel)} L`}
        hint="Blackboard-Pull telemetry.fuel_liters (nicht im Push)"
      />
      <Row label="Cruise" value={cruise > 0 ? `${fmt0(cruise)} km/h` : "off"} />
    </Panel>
  );
}
