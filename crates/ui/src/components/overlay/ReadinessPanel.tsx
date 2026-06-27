import { Panel, Row } from "./Panel";
import { useBB, bbReadyTri } from "./overlay-lib";

/// Read-only core daemon readiness (blackboard contract). Display only — never gates Engage.
export function ReadinessPanel() {
  const graph = bbReadyTri(useBB("graph_ready"));
  const spline = bbReadyTri(useBB("spline_index_ready"));
  const plugins = bbReadyTri(useBB("plugins_ready"));
  const laneDetection = bbReadyTri(useBB("lane_detection_ready"));
  const system = bbReadyTri(useBB("truckpilot_system_ready"));

  return (
    <Panel title="Core readiness" className="min-w-[13rem]">
      <Row label="Graph" value={graph} />
      <Row label="Spline" value={spline} />
      <Row label="Plugins" value={plugins} />
      <Row label="Lane detection" value={laneDetection} />
      <Row
        label="System ready"
        value={system}
        hint="Subsystem startup only — not engage authorization"
      />
    </Panel>
  );
}
