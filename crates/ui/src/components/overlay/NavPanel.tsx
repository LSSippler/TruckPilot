import { Panel, Row } from "./Panel";
import { useBB, bbNum, fmt2 } from "./overlay-lib";

/// Mitte: Nav — Navigations-Status + normierter Lane-Offset.
/// Ist-Querfehler in Metern existiert NICHT in der Telemetrie (Slot leer mit
/// Hint); der normierte Offset wird klar als „norm" gelabelt gezeigt.
export function NavPanel() {
  const active = useBB("router.active") === "true";
  const waypoints = useBB("router.waypoint_count");
  const offset = bbNum(useBB("lane.center_offset"));

  return (
    <Panel title="Nav" className="min-w-[12rem]">
      <Row label="Status" value={active ? "Navigating" : "Idle"} />
      <Row label="Waypoints" value={waypoints ?? "—"} />
      <Row
        label="Offset (norm)"
        value={fmt2(offset)}
        hint="lane.center_offset [-1..1] — kein Meter-Querfehler in Telemetrie"
      />
      <Row
        label="Cross-track (m)"
        value="—"
        hint="Ist-Querfehler in Metern nicht in Telemetrie verfügbar"
      />
    </Panel>
  );
}
