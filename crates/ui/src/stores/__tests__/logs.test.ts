import { describe, expect, it, beforeEach } from "vitest";
import { selectFilteredLogs, useLogsStore } from "@/stores/logs";

describe("logs store filtering", () => {
  beforeEach(() => {
    useLogsStore.setState({
      entries: [],
      maxEntries: 1000,
      filterLevels: ["trace", "debug", "info", "warn", "error"],
      filterPlugin: null,
      search: "",
    });
  });

  it("respects level filter", () => {
    const push = useLogsStore.getState().push;
    push({ ts: 1, level: "info", message: "hello", plugin: "core" });
    push({ ts: 2, level: "error", message: "kaboom", plugin: "core" });
    useLogsStore.getState().setFilter({ filterLevels: ["error"] });
    const filtered = selectFilteredLogs(useLogsStore.getState());
    expect(filtered).toHaveLength(1);
    expect(filtered[0]?.message).toBe("kaboom");
  });

  it("filters by plugin", () => {
    useLogsStore.getState().push({ ts: 1, level: "info", message: "a", plugin: "x" });
    useLogsStore.getState().push({ ts: 2, level: "info", message: "b", plugin: "y" });
    useLogsStore.getState().setFilter({ filterPlugin: "y" });
    const filtered = selectFilteredLogs(useLogsStore.getState());
    expect(filtered.map((e) => e.message)).toEqual(["b"]);
  });

  it("substring searches the message", () => {
    useLogsStore.getState().push({ ts: 1, level: "info", message: "Connecting to core", plugin: null });
    useLogsStore.getState().push({ ts: 2, level: "info", message: "Telemetry frame", plugin: null });
    useLogsStore.getState().setFilter({ search: "telemetry" });
    const filtered = selectFilteredLogs(useLogsStore.getState());
    expect(filtered).toHaveLength(1);
  });
});
