import { describe, expect, it, beforeEach } from "vitest";
import { usePluginsStore } from "@/stores/plugins";

describe("plugins store", () => {
  beforeEach(() => {
    usePluginsStore.setState({ list: [], order: [], selected: null, schemas: {} });
  });

  it("setList replaces list and order", () => {
    usePluginsStore.getState().setList([
      { name: "a", version: "1.0", enabled: true },
      { name: "b", version: "2.0", enabled: false },
    ]);
    const s = usePluginsStore.getState();
    expect(s.list.map((p) => p.name)).toEqual(["a", "b"]);
    expect(s.order).toEqual(["a", "b"]);
  });

  it("reorder swaps two plugins", () => {
    usePluginsStore.getState().setList([
      { name: "a", version: "1.0", enabled: true },
      { name: "b", version: "1.0", enabled: true },
      { name: "c", version: "1.0", enabled: true },
    ]);
    usePluginsStore.getState().reorder(0, 2);
    expect(usePluginsStore.getState().order).toEqual(["b", "c", "a"]);
  });

  it("dropping an unloaded plugin removes it from list and order", () => {
    usePluginsStore.getState().setList([
      { name: "a", version: "1.0", enabled: true },
      { name: "b", version: "1.0", enabled: true },
    ]);
    usePluginsStore.getState().applyEvent("a", { kind: "unloaded" });
    const s = usePluginsStore.getState();
    expect(s.list.map((p) => p.name)).toEqual(["b"]);
    expect(s.order).toEqual(["b"]);
  });
});
