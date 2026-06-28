import { useEffect, useMemo, useState } from "react";
import { isTauriRuntime, readOverlaySnapshotFile } from "@/lib/tauri-bridge";
import {
  createLiveOverlaySnapshotShell,
  resolveOverlaySnapshotFeed,
  resolveOverlaySnapshotForFeed,
  type OverlaySnapshot,
  type OverlaySnapshotFeed,
} from "./overlay-snapshot";
import { pollLiveOverlaySnapshotAsync } from "./overlay-snapshot-live";

const LIVE_POLL_MS = 2000;

function resolveInitialSnapshot(
  search: URLSearchParams,
  feed: OverlaySnapshotFeed,
  internalVisualization: boolean,
): OverlaySnapshot | null {
  const snap = resolveOverlaySnapshotForFeed(search, feed);
  if (snap) return snap;
  if (feed === "live" && internalVisualization) {
    return createLiveOverlaySnapshotShell();
  }
  return null;
}

/** Snapshot + feed for overlay route (fixture, storage paste, or live polling). */
export function useOverlaySnapshotFeed(
  search: URLSearchParams,
  internalVisualization: boolean,
): { snapshot: OverlaySnapshot | null; feed: OverlaySnapshotFeed; setSnapshot: (snap: OverlaySnapshot | null) => void } {
  const feed = useMemo(() => resolveOverlaySnapshotFeed(search), [search]);
  const [snapshot, setSnapshot] = useState<OverlaySnapshot | null>(() =>
    resolveInitialSnapshot(search, feed, internalVisualization),
  );

  useEffect(() => {
    if (feed === "fixture" || feed === "storage" || feed === "off") {
      setSnapshot(resolveOverlaySnapshotForFeed(search, feed));
      return;
    }

    let cancelled = false;

    const refresh = async () => {
      const snap = await pollLiveOverlaySnapshotAsync({
        internalVisualization,
        readOverlaySnapshotFile,
        isTauriRuntime,
      });
      if (!cancelled) setSnapshot(snap);
    };

    void refresh();
    const id = window.setInterval(() => {
      void refresh();
    }, LIVE_POLL_MS);
    return () => {
      cancelled = true;
      window.clearInterval(id);
    };
  }, [feed, internalVisualization, search]);

  return { snapshot, feed, setSnapshot };
}
