import { useEffect, useMemo, useState } from "react";
import {
  createLiveOverlaySnapshotShell,
  readOverlaySnapshotFromStorage,
  resolveOverlaySnapshotFeed,
  resolveOverlaySnapshotForFeed,
  type OverlaySnapshot,
  type OverlaySnapshotFeed,
} from "./overlay-snapshot";

const LIVE_STORAGE_POLL_MS = 2000;

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

    const refresh = () => {
      const fromStorage = readOverlaySnapshotFromStorage();
      if (fromStorage) {
        setSnapshot(fromStorage);
        return;
      }
      if (internalVisualization) {
        setSnapshot(createLiveOverlaySnapshotShell());
      } else {
        setSnapshot(null);
      }
    };

    refresh();
    const id = window.setInterval(refresh, LIVE_STORAGE_POLL_MS);
    return () => window.clearInterval(id);
  }, [feed, internalVisualization, search]);

  return { snapshot, feed, setSnapshot };
}
