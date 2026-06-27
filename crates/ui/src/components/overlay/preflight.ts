/** Tri-state for read-only preflight display (`true` / `false` / unknown). */
export type PreflightTri = boolean | null;

export interface PreflightInputs {
  systemReady: PreflightTri;
  telemetryFresh: PreflightTri;
  routeValid: PreflightTri;
  laneModelValid: PreflightTri;
  resolverSafe: PreflightTri;
  inputAllowed: PreflightTri;
  autopilotState: string | null;
}

export interface PreflightView {
  systemReady: PreflightTri;
  telemetryFresh: PreflightTri;
  routeValid: PreflightTri;
  laneModelValid: PreflightTri;
  resolverSafe: PreflightTri;
  inputAllowed: PreflightTri;
  autopilotState: string | null;
  /** Display-only — not engage authorization. */
  driveAllowedDisplay: boolean;
  reasons: string[];
}

export function parseBbTri(v: string | undefined): PreflightTri {
  if (v === "true") return true;
  if (v === "false") return false;
  return null;
}

/** Map blackboard `true`/`false` strings to tri-state bool. */
export function bbTri(v: string | undefined): PreflightTri {
  return parseBbTri(v);
}

function pushTriReason(
  reasons: string[],
  label: string,
  v: PreflightTri,
  falseMsg: string,
): void {
  if (v === false) reasons.push(falseMsg);
  else if (v === null) reasons.push(`${label} unknown`);
}

/** Pure display-only preflight evaluation — must not gate control. */
export function computePreflightView(inputs: PreflightInputs): PreflightView {
  const reasons: string[] = [];

  pushTriReason(reasons, "system ready", inputs.systemReady, "system not ready");
  pushTriReason(
    reasons,
    "telemetry fresh",
    inputs.telemetryFresh,
    "telemetry not fresh",
  );
  pushTriReason(reasons, "route valid", inputs.routeValid, "route invalid");
  pushTriReason(
    reasons,
    "lane model valid",
    inputs.laneModelValid,
    "lane model invalid",
  );
  pushTriReason(reasons, "resolver safe", inputs.resolverSafe, "resolver not safe");
  pushTriReason(reasons, "input allowed", inputs.inputAllowed, "input disabled");
  if (inputs.autopilotState === "Fault") {
    reasons.push("autopilot fault");
  }

  return {
    ...inputs,
    driveAllowedDisplay: reasons.length === 0,
    reasons,
  };
}

/** Overlay label: yes / no / unknown */
export function preflightLabel(v: PreflightTri): string {
  if (v === true) return "yes";
  if (v === false) return "no";
  return "unknown";
}

/** Lane model valid from live lane-detection keys (not plugin load). */
export function laneModelValidFromBb(
  confidenceRaw: string | undefined,
  leftVisible: string | undefined,
  rightVisible: string | undefined,
): PreflightTri {
  if (
    confidenceRaw == null ||
    confidenceRaw === "" ||
    leftVisible == null ||
    rightVisible == null
  ) {
    return null;
  }
  const confidence = Number(confidenceRaw);
  if (!Number.isFinite(confidence)) return null;
  const left = leftVisible === "true";
  const right = rightVisible === "true";
  return confidence >= 0.5 && left && right;
}

/** Output sink permitted to act (vjoy or scs-sdk connected). */
export function inputAllowedFromBb(
  sink: string | undefined,
  vjoyConnected: string | undefined,
  scsConnected: string | undefined,
): PreflightTri {
  if (sink == null || sink === "") return null;
  if (sink === "none") return false;
  if (vjoyConnected === "true" || scsConnected === "true") return true;
  if (vjoyConnected === "false" && scsConnected === "false") return false;
  return null;
}

/** Tri-state from optional blackboard mirror (`true` / `false` / unknown). */
export function resolverSafeFromBb(v: string | undefined): PreflightTri {
  return parseBbTri(v);
}

/** Telemetry fresh: prefer state-machine precondition, fall back to telemetry.available. */
export function telemetryFreshFromBb(
  engagePrecondition: string | undefined,
  telemetryAvailable: string | undefined,
): PreflightTri {
  const fromEngage = parseBbTri(engagePrecondition);
  if (fromEngage !== null) return fromEngage;
  return parseBbTri(telemetryAvailable);
}
