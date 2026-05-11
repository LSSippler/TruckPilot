import { describe, expect, it, beforeEach } from "vitest";
import { useAutopilotStore } from "@/stores/autopilot";

const fakeStatus = {
  state: "Active" as const,
  faultReason: null,
  preconditions: {
    telemetry_ok: true,
    engine_running: true,
    cruise_active: true,
    critical_plugins_loaded: true,
    router_active: true,
  },
  tickCount: 250,
};

describe("autopilot store", () => {
  beforeEach(() => {
    useAutopilotStore.getState().clear();
  });

  it("starts with state=null", () => {
    expect(useAutopilotStore.getState().state).toBeNull();
  });

  it("updates on status payload", () => {
    useAutopilotStore.getState().setStatus(fakeStatus);
    const s = useAutopilotStore.getState();
    expect(s.state).toBe("Active");
    expect(s.tickCount).toBe(250);
    expect(s.preconditions.engine_running).toBe(true);
    expect(s.lastUpdateMs).toBeGreaterThan(0);
  });

  it("clear() resets to defaults", () => {
    useAutopilotStore.getState().setStatus(fakeStatus);
    useAutopilotStore.getState().clear();
    const s = useAutopilotStore.getState();
    expect(s.state).toBeNull();
    expect(s.tickCount).toBe(0);
    expect(s.preconditions.engine_running).toBe(false);
  });

  it("records fault reason when present", () => {
    useAutopilotStore.getState().setStatus({
      ...fakeStatus,
      state: "Fault",
      faultReason: "WatchdogStall",
    });
    expect(useAutopilotStore.getState().faultReason).toBe("WatchdogStall");
  });
});
