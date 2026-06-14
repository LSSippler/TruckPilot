import { useAutopilotStore } from "@/stores/autopilot";
import { Panel, Row } from "./Panel";
import { useBB, bbNum, fmt2 } from "./overlay-lib";

/// Links unten: State — Autopilot-State-Machine + Steering-Level.
/// Longitudinal-Level existiert NICHT als diskreter Zustand in der Telemetrie
/// (Slot bleibt leer mit Hint); Throttle/Brake werden als reale [0..1]-Werte
/// gezeigt, klar gelabelt (KEIN Surrogat für einen „Level").
export function StatePanel() {
  const state = useAutopilotStore((s) => s.state);
  const fault = useAutopilotStore((s) => s.faultReason);
  const level = useBB("lane_keeper.fallback_level");
  const source = useBB("lane_keeper.steering_source");
  const throttle = bbNum(useBB("speed_controller.throttle_cmd"));
  const brake = bbNum(useBB("speed_controller.brake_cmd"));

  return (
    <Panel title="State" className="min-w-[13rem]">
      <Row
        label="Autopilot"
        value={state ?? "—"}
        warn={state === "Fault"}
        hint={fault ?? undefined}
      />
      <Row
        label="Steer-Level"
        value={level ? `L${level}${source ? ` · ${source}` : ""}` : "—"}
      />
      <Row
        label="Long-Level"
        value="—"
        hint="Kein diskreter Longitudinal-Level in der Telemetrie"
      />
      <Row label="Throttle / Brake" value={`${fmt2(throttle)} / ${fmt2(brake)}`} />
    </Panel>
  );
}
