import { useRef, useState } from "react";
import { Panel, Row } from "./Panel";
import {
  saveOverlaySnapshotToStorage,
  coreReadyLabel,
  type OverlaySnapshot,
} from "./overlay-snapshot";

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
export function SnapshotDebugPanel({
  snapshot,
  onSnapshotImported,
}: {
  snapshot: OverlaySnapshot | null;
  onSnapshotImported: (snapshot: OverlaySnapshot) => void;
}) {
  const fileRef = useRef<HTMLInputElement | null>(null);
  const [importError, setImportError] = useState<string | null>(null);

  const handleImportClick = () => {
    setImportError(null);
    fileRef.current?.click();
  };

  const handleFileChange = async (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    e.target.value = "";
    if (!file) return;

    try {
      const text = await file.text();
      const imported = saveOverlaySnapshotToStorage(text);
      if (!imported) {
        setImportError("Ungültiges Overlay-Snapshot JSON");
        return;
      }
      setImportError(null);
      onSnapshotImported(imported);
    } catch {
      setImportError("Datei konnte nicht gelesen werden");
    }
  };

  const status = snapshot?.status;
  const lane = snapshot?.lane;
  const readiness = status?.core_readiness;
  const isMock = lane?.source === "mock";
  const resolverLine = status
    ? status.resolver_off
      ? `off · ${status.resolve_status}`
      : status.resolve_status
    : "—";

  return (
    <Panel
      title={isMock ? "DLL Debug · MOCK" : "DLL Debug"}
      className="min-w-[13rem] ring-amber-400/30 pointer-events-auto"
    >
      <div className="mb-1.5 flex flex-col gap-1">
        <button
          type="button"
          className="rounded border border-white/20 bg-white/10 px-2 py-1 text-[10px] font-semibold uppercase tracking-wide text-white hover:bg-white/15"
          onClick={handleImportClick}
        >
          Import JSON
        </button>
        <input
          ref={fileRef}
          type="file"
          accept="application/json,.json"
          className="hidden"
          onChange={(e) => void handleFileChange(e)}
        />
        {importError ? (
          <p className="text-[10px] text-red-400" role="alert">
            {importError}
          </p>
        ) : null}
      </div>

      {!snapshot ? (
        <p className="text-[10px] text-white/55">
          Kein Snapshot geladen — JSON importieren oder localStorage befüllen.
        </p>
      ) : (
        <>
          {isMock && (
            <div className="mb-1 rounded bg-amber-500/20 px-1.5 py-0.5 text-[10px] font-bold tracking-wider text-amber-300">
              MOCK DATA — read-only
            </div>
          )}
          <Row label="Verdict" value={verdictLabel(snapshot.verdict)} />
          <Row label="Diag-Level" value={status?.diag_level ?? "—"} />
          <Row label="Resolver" value={resolverLine} />
          <Row label="Route valid" value={status?.route_valid ? "yes" : "no"} />
          {readiness ? (
            <>
              <Row label="Graph" value={coreReadyLabel(readiness.graph_ready)} />
              <Row label="Spline" value={coreReadyLabel(readiness.spline_index_ready)} />
              <Row label="Plugins" value={coreReadyLabel(readiness.plugins_ready)} />
              <Row
                label="Lane detection"
                value={coreReadyLabel(readiness.lane_detection_ready)}
              />
              <Row
                label="System ready"
                value={coreReadyLabel(readiness.truckpilot_system_ready)}
                hint="not engage authorization"
              />
            </>
          ) : null}
          <Row label="Frame cb max" value={`${status?.frame_cb_us_max ?? 0} µs`} />
          <Row label="Lane source" value={lane?.source ?? "—"} />
          <Row
            label="Lane model"
            value={lane?.lane_model_valid ? "valid" : "invalid (debug)"}
            warn={!lane?.lane_model_valid}
          />
          <Row
            label="LK allowed"
            value={snapshot.lane_keeper_allowed ? "yes (display)" : "no"}
            hint="Anzeige only — steuert nichts"
          />
        </>
      )}
    </Panel>
  );
}
