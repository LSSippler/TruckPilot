import { create } from "zustand";
import type { TelemetrySnapshot } from "@/lib/types";

interface TelemetryState {
  latest: TelemetrySnapshot | null;
  lastUpdateMs: number;
  history: { tMs: number; speed_ms: number }[];
  push: (snap: TelemetrySnapshot) => void;
}

const HISTORY_WINDOW_MS = 60_000;
const MIN_INTERVAL_MS = 1000 / 30;

let lastWriteAt = 0;
let pendingFrame = 0;
let pendingSnap: TelemetrySnapshot | null = null;

export const useTelemetryStore = create<TelemetryState>((set, get) => ({
  latest: null,
  lastUpdateMs: 0,
  history: [],
  push: (snap) => {
    pendingSnap = snap;
    const now = performance.now();
    if (now - lastWriteAt < MIN_INTERVAL_MS) {
      if (pendingFrame !== 0) return;
      pendingFrame = requestAnimationFrame(() => {
        pendingFrame = 0;
        if (!pendingSnap) return;
        commit(pendingSnap, set, get);
      });
      return;
    }
    commit(snap, set, get);
  },
}));

function commit(
  snap: TelemetrySnapshot,
  set: (partial: Partial<TelemetryState>) => void,
  get: () => TelemetryState
) {
  lastWriteAt = performance.now();
  const tMs = Date.now();
  const history = [...get().history, { tMs, speed_ms: snap.speed_ms }].filter(
    (entry) => tMs - entry.tMs <= HISTORY_WINDOW_MS
  );
  set({ latest: snap, lastUpdateMs: tMs, history });
}
