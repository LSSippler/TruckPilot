import { describe, expect, it } from "vitest";
import {
  computePreflightView,
  inputAllowedFromBb,
  laneModelValidFromBb,
  preflightLabel,
  resolverSafeFromBb,
  telemetryFreshFromBb,
} from "@/components/overlay/preflight";

describe("computePreflightView", () => {
  it("drive allowed only when all checks true and not fault", () => {
    const ok = computePreflightView({
      systemReady: true,
      telemetryFresh: true,
      routeValid: true,
      laneModelValid: true,
      resolverSafe: true,
      inputAllowed: true,
      autopilotState: "Off",
    });
    expect(ok.driveAllowedDisplay).toBe(true);
    expect(ok.reasons).toHaveLength(0);
  });

  it("lists blockers for route, lane, input, and fault", () => {
    const blocked = computePreflightView({
      systemReady: true,
      telemetryFresh: true,
      routeValid: false,
      laneModelValid: false,
      resolverSafe: true,
      inputAllowed: false,
      autopilotState: "Fault",
    });
    expect(blocked.driveAllowedDisplay).toBe(false);
    expect(blocked.reasons).toContain("route invalid");
    expect(blocked.reasons).toContain("lane model invalid");
    expect(blocked.reasons).toContain("input disabled");
    expect(blocked.reasons).toContain("autopilot fault");
  });

  it("unknown fields add unknown reasons", () => {
    const view = computePreflightView({
      systemReady: null,
      telemetryFresh: null,
      routeValid: null,
      laneModelValid: null,
      resolverSafe: null,
      inputAllowed: null,
      autopilotState: null,
    });
    expect(view.driveAllowedDisplay).toBe(false);
    expect(view.reasons.some((r) => r.includes("unknown"))).toBe(true);
  });
});

describe("preflight helpers", () => {
  it("resolverSafeFromBb maps blackboard mirror", () => {
    expect(resolverSafeFromBb("true")).toBe(true);
    expect(resolverSafeFromBb("false")).toBe(false);
    expect(resolverSafeFromBb(undefined)).toBe(null);
  });

  it("telemetryFreshFromBb prefers engage precondition", () => {
    expect(telemetryFreshFromBb("false", "true")).toBe(false);
    expect(telemetryFreshFromBb(undefined, "true")).toBe(true);
  });

  it("laneModelValidFromBb requires confidence and both lanes", () => {
    expect(laneModelValidFromBb("0.9", "true", "true")).toBe(true);
    expect(laneModelValidFromBb("0.2", "true", "true")).toBe(false);
    expect(laneModelValidFromBb(undefined, "true", "true")).toBe(null);
  });

  it("inputAllowedFromBb checks output sink", () => {
    expect(inputAllowedFromBb("vjoy", "true", "false")).toBe(true);
    expect(inputAllowedFromBb("none", "false", "false")).toBe(false);
  });

  it("preflightLabel maps tri-state", () => {
    expect(preflightLabel(true)).toBe("yes");
    expect(preflightLabel(false)).toBe("no");
    expect(preflightLabel(null)).toBe("unknown");
  });
});
