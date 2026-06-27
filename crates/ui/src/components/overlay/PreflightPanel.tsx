import { useMemo } from "react";
import { useAutopilotStore } from "@/stores/autopilot";
import { Panel, Row } from "./Panel";
import { useBB } from "./overlay-lib";
import {
  bbTri,
  computePreflightView,
  inputAllowedFromBb,
  laneModelValidFromBb,
  preflightLabel,
  resolverSafeFromBb,
  telemetryFreshFromBb,
} from "./preflight";

/// Read-only preflight — explains why TruckPilot must not steer. Display only.
export function PreflightPanel() {
  const autopilotState = useAutopilotStore((s) => s.state);
  const faultReason = useAutopilotStore((s) => s.faultReason);

  const systemReadyRaw = useBB("truckpilot_system_ready");
  const telemetryFreshRaw = useBB("state.engage_precondition_telemetry_fresh");
  const telemetryAvailable = useBB("telemetry.available");
  const routerActive = useBB("router.active");
  const routePlanned = useBB("state.engage_precondition_route_planned");
  const laneConfidence = useBB("lane.confidence");
  const laneLeft = useBB("lane.left_visible");
  const laneRight = useBB("lane.right_visible");
  const outputSink = useBB("output.sink.configured");
  const vjoyConnected = useBB("vjoy.connected");
  const scsConnected = useBB("scs_sdk_output.connected");
  const resolverSafeRaw = useBB("preflight.resolver_safe");

  const view = useMemo(() => {
    const systemReady =
      systemReadyRaw === "true"
        ? true
        : systemReadyRaw === "false"
          ? false
          : null;

    let routeValid = bbTri(routerActive);
    if (routeValid === null) {
      routeValid = bbTri(routePlanned);
    }

    return computePreflightView({
      systemReady,
      telemetryFresh: telemetryFreshFromBb(telemetryFreshRaw, telemetryAvailable),
      routeValid,
      laneModelValid: laneModelValidFromBb(laneConfidence, laneLeft, laneRight),
      resolverSafe: resolverSafeFromBb(resolverSafeRaw),
      inputAllowed: inputAllowedFromBb(outputSink, vjoyConnected, scsConnected),
      autopilotState: autopilotState ?? null,
    });
  }, [
    autopilotState,
    laneConfidence,
    laneLeft,
    laneRight,
    outputSink,
    routePlanned,
    routerActive,
    resolverSafeRaw,
    scsConnected,
    systemReadyRaw,
    telemetryAvailable,
    telemetryFreshRaw,
    vjoyConnected,
  ]);

  const reasonLine =
    view.reasons.length === 0 ? "none" : view.reasons.join(", ");

  return (
    <Panel title="TruckPilot Preflight" className="min-w-[13rem]">
      <Row label="System ready" value={preflightLabel(view.systemReady)} />
      <Row label="Telemetry fresh" value={preflightLabel(view.telemetryFresh)} />
      <Row label="Route valid" value={preflightLabel(view.routeValid)} />
      <Row label="Lane model valid" value={preflightLabel(view.laneModelValid)} />
      <Row label="Resolver safe" value={preflightLabel(view.resolverSafe)} />
      <Row label="Input allowed" value={preflightLabel(view.inputAllowed)} />
      <Row
        label="Autopilot state"
        value={view.autopilotState ?? "—"}
        warn={view.autopilotState === "Fault"}
        hint={faultReason ?? undefined}
      />
      <Row
        label="Drive allowed"
        value={view.driveAllowedDisplay ? "yes (display)" : "no"}
        warn={!view.driveAllowedDisplay}
        hint="Display only — not engage authorization"
      />
      <Row label="Reason" value={reasonLine} />
    </Panel>
  );
}
