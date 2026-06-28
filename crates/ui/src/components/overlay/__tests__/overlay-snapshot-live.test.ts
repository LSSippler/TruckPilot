import { describe, expect, it, vi } from "vitest";
import fixtureJson from "@/components/overlay/overlay-snapshot.fixture.json";
import {
  pollLiveOverlaySnapshot,
  pollLiveOverlaySnapshotAsync,
} from "@/components/overlay/overlay-snapshot-live";
import {
  createLiveOverlaySnapshotShell,
  loadOverlaySnapshotFixture,
  parseOverlaySnapshot,
  resolveOverlaySnapshotFeed,
} from "@/components/overlay/overlay-snapshot";

describe("pollLiveOverlaySnapshot", () => {
  const fixtureJsonString = JSON.stringify(fixtureJson);

  it("prefers Tauri file JSON over localStorage when enabled", () => {
    const snap = pollLiveOverlaySnapshot({
      tauriFileEnabled: true,
      tauriFileJson: fixtureJsonString,
      internalVisualization: true,
    });
    expect(snap?.planned_path?.source).toBe("offline_graph");
  });

  it("falls back to live shell when Tauri file missing and no storage", () => {
    const snap = pollLiveOverlaySnapshot({
      tauriFileEnabled: true,
      tauriFileJson: null,
      internalVisualization: true,
    });
    expect(snap?.planned_path).toBeUndefined();
    expect(snap?.verdict).toBe("unavailable");
  });

  it("skips Tauri file when not in Tauri runtime", () => {
    const snap = pollLiveOverlaySnapshot({
      tauriFileEnabled: false,
      tauriFileJson: fixtureJsonString,
      internalVisualization: true,
    });
    expect(snap?.planned_path).toBeUndefined();
  });

  it("async poll uses Tauri reader when runtime reports Tauri", async () => {
    const readOverlaySnapshotFile = vi.fn().mockResolvedValue(fixtureJsonString);
    const snap = await pollLiveOverlaySnapshotAsync({
      internalVisualization: true,
      isTauriRuntime: () => true,
      readOverlaySnapshotFile,
    });
    expect(readOverlaySnapshotFile).toHaveBeenCalledOnce();
    expect(snap?.planned_path?.source).toBe("offline_graph");
  });

  it("async poll falls back when Tauri reader unavailable", async () => {
    const snap = await pollLiveOverlaySnapshotAsync({
      internalVisualization: true,
      isTauriRuntime: () => false,
    });
    expect(snap).toEqual(createLiveOverlaySnapshotShell());
  });

  it("async poll falls back when Tauri reader throws", async () => {
    const snap = await pollLiveOverlaySnapshotAsync({
      internalVisualization: true,
      isTauriRuntime: () => true,
      readOverlaySnapshotFile: vi.fn().mockRejectedValue(new Error("denied")),
    });
    expect(snap).toEqual(createLiveOverlaySnapshotShell());
  });

  it("parses planned_path_producer from file JSON without failing", () => {
    const withProducer = {
      ...fixtureJson,
      planned_path_producer: { status: "attached", source: "offline_graph" },
    };
    const snap = pollLiveOverlaySnapshot({
      tauriFileEnabled: true,
      tauriFileJson: JSON.stringify(withProducer),
      internalVisualization: true,
    });
    expect(snap?.planned_path?.items.length).toBeGreaterThan(0);
  });

  it("fixture feed mode unchanged", () => {
    expect(resolveOverlaySnapshotFeed(new URLSearchParams("overlay_snapshot=fixture"))).toBe(
      "fixture",
    );
    expect(loadOverlaySnapshotFixture().planned_path?.source).toBe("offline_graph");
  });

  it("storage feed mode unchanged", () => {
    expect(resolveOverlaySnapshotFeed(new URLSearchParams("overlay_snapshot=storage"))).toBe(
      "storage",
    );
  });

  it("invalid Tauri file JSON falls back to shell", () => {
    const snap = pollLiveOverlaySnapshot({
      tauriFileEnabled: true,
      tauriFileJson: "{not-json",
      internalVisualization: true,
    });
    expect(snap).toEqual(createLiveOverlaySnapshotShell());
  });

  it("live file snapshot parses offline_graph for OFFLINE badge path", () => {
    const snap = parseOverlaySnapshot(fixtureJsonString);
    expect(snap?.planned_path?.source).toBe("offline_graph");
  });
});
