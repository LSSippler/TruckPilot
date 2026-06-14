import { useTelemetryStore } from "@/stores/telemetry";
import { msToKmh } from "@/lib/utils";
import { Panel, Row } from "./Panel";
import { useBB, bbNum, fmt0, fmt2 } from "./overlay-lib";

/// Links oben: ACC — Ist/Soll-Speed, Steering-Command, normierter Lane-Offset.
/// Es gibt KEINEN physischen Lenkwinkel und KEINEN Meter-Querfehler in der
/// Telemetrie; gezeigt werden die real verfügbaren normierten Ersatzwerte,
/// explizit als „cmd"/„norm" gelabelt.
export function AccPanel() {
  const speedMs = useTelemetryStore((s) => s.latest?.speed_ms ?? null);
  // Call both hooks unconditionally (no `??` short-circuit between hook calls),
  // then prefer the resolved speed-controller target over the player cruise set.
  const targetResolved = bbNum(useBB("speed_controller.target_speed_kmh"));
  const targetCruise = bbNum(useBB("cruise.target_kmh"));
  const target = targetResolved ?? targetCruise;
  const steer = bbNum(useBB("lane_keeper.steering_out"));
  const offset = bbNum(useBB("lane.center_offset"));

  return (
    <Panel title="ACC" className="min-w-[13rem]">
      <Row label="Ist" value={speedMs == null ? "—" : `${fmt0(msToKmh(speedMs))} km/h`} />
      <Row label="Soll" value={target == null ? "—" : `${fmt0(target)} km/h`} />
      <Row
        label="Steer (cmd −1..1)"
        value={fmt2(steer)}
        hint="lane_keeper.steering_out — normiertes Command, kein physischer Lenkwinkel"
      />
      <Row
        label="Offset (norm)"
        value={fmt2(offset)}
        hint="lane.center_offset [-1..1] — kein Meter-Querfehler in Telemetrie"
      />
    </Panel>
  );
}
