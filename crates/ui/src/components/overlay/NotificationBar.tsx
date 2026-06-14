import { useTelemetryStore } from "@/stores/telemetry";
import { msToKmh } from "@/lib/utils";
import { Panel, Row } from "./Panel";
import { fmt0 } from "./overlay-lib";

/// Mitte oben: Notification — Speedlimit-Warnung.
/// `nav_speed_limit_kmh == -1` bedeutet „kein Limit", nicht 0 → als „--" zeigen.
/// Es gibt KEINEN dedizierten Warn-Flag-Key; die Warnung wird UI-seitig aus
/// `msToKmh(speed_ms) > limit` abgeleitet (Konvertierung an EINER Stelle).
export function NotificationBar() {
  const latest = useTelemetryStore((s) => s.latest);
  const limit = latest?.nav_speed_limit_kmh ?? -1;
  const hasLimit = limit > 0; // -1 = nicht verfügbar
  const speedKmh = latest ? msToKmh(latest.speed_ms) : 0;
  const over = hasLimit && speedKmh > limit + 1;

  return (
    <Panel title="Notification" className="min-w-[15rem] text-center">
      <Row
        label="Speedlimit"
        value={hasLimit ? `${fmt0(limit)} km/h` : "--"}
        warn={over}
        hint={hasLimit ? undefined : "nav_speed_limit_kmh = -1 → kein Limit verfügbar"}
      />
      {over ? <Row label="" value={`⚠ ÜBER LIMIT (${fmt0(speedKmh)} km/h)`} warn /> : null}
    </Panel>
  );
}
