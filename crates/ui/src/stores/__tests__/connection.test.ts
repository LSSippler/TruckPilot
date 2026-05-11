import { describe, expect, it, beforeEach } from "vitest";
import { useConnectionStore } from "@/stores/connection";

describe("connection store", () => {
  beforeEach(() => {
    useConnectionStore.setState({
      status: "disconnected",
      protocolVersion: null,
      lastError: null,
      helloReceived: false,
    });
  });

  it("starts disconnected", () => {
    expect(useConnectionStore.getState().status).toBe("disconnected");
  });

  it("flips to connected once a hello arrives", () => {
    useConnectionStore.getState().setHello("1.0", 1);
    const s = useConnectionStore.getState();
    expect(s.status).toBe("connected");
    expect(s.protocolVersion).toBe("1.0");
    expect(s.helloReceived).toBe(true);
  });

  it("records an error from a status event", () => {
    useConnectionStore.getState().setStatus({
      status: "disconnected",
      protocol_version: null,
      last_error: "boom",
    });
    expect(useConnectionStore.getState().lastError).toBe("boom");
  });
});
