import { Panel, Row } from "./Panel";
import type { OverlaySnapshot } from "./overlay-snapshot";

function verdictLabel(v: OverlaySnapshot["verdict"]): string {
  switch (v) {
    case "safe_cold":
      return "safe_cold";
    case "hot":
      return "hot";
    case "unavailable":
      return "unavailable";
  }
}

/// Read-only DLL/lane debug panel from `truckpilot-status --overlay` JSON.
/// Display only — `lane_keeper_allowed` is never used to enable steering.
export function SnapshotDebugPanel({ snapshot }: { snapshot: OverlaySnapshot }) {
  const { status, lane } = snapshot;
  const isMock = lane.source === "mock";
  const resolverLine = status.resolver_off
    ? `off · ${status.resolve_status}`
    : status.resolve_status;

  return (
    <Panel
      title={isMock ? "DLL Debug · MOCK" : "DLL Debug"}
      className="min-w-[13rem] ring-amber-400/30"
    >
      {isMock && (
        <div className="mb-1 rounded bg-amber-500/20 px-1.5 py-0.5 text-[10px] font-bold tracking-wider text-amber-300">
          MOCK DATA — read-only
        </div>
      )}
      <Row label="Verdict" value={verdictLabel(snapshot.verdict)} />
      <Row label="Diag-Level" value={status.diag_level} />
      <Row label="Resolver" value={resolverLine} />
      <Row label="Route valid" value={status.route_valid ? "yes" : "no"} />
      <Row label="Frame cb max" value={`${status.frame_cb_us_max} µs`} />
      <Row label="Lane source" value={lane.source} />
      <Row
        label="Lane model"
        value={lane.lane_model_valid ? "valid" : "invalid (debug)"}
        warn={!lane.lane_model_valid}
      />
      <Row
        label="LK allowed"
        value={snapshot.lane_keeper_allowed ? "yes (display)" : "no"}
        hint="Anzeige only — steuert nichts"
      />
    </Panel>
  );
}
