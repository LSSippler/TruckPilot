import { create } from "zustand";

const HISTORY_WINDOW_MS = 60_000;
const HISTORY_MAX_POINTS = 600;

export interface HistoryPoint {
  tMs: number;
  v: number;
}

interface BlackboardState {
  /// Full list of keys advertised by the daemon (refreshed via blackboard_list).
  keys: string[];
  /// Latest value per key (raw stringified form from the daemon).
  values: Record<string, string>;
  /// Timestamp (ms epoch) of the last value update per key.
  updatedAt: Record<string, number>;
  /// Rolling numeric history per key, in insertion order. Pruned to 60 s / 600 pts.
  history: Record<string, HistoryPoint[]>;
  setKeys: (keys: string[]) => void;
  ingest: (values: Record<string, string>) => void;
  clear: () => void;
}

export const useBlackboardStore = create<BlackboardState>((set) => ({
  keys: [],
  values: {},
  updatedAt: {},
  history: {},
  setKeys: (keys) => set({ keys: keys.slice().sort() }),
  ingest: (values) =>
    set((state) => {
      const tMs = Date.now();
      const nextValues = { ...state.values };
      const nextUpdated = { ...state.updatedAt };
      const nextHistory = { ...state.history };
      for (const [k, v] of Object.entries(values)) {
        nextValues[k] = v;
        nextUpdated[k] = tMs;
        const num = Number(v);
        if (Number.isFinite(num)) {
          const series = (nextHistory[k] ?? []).filter((p) => tMs - p.tMs <= HISTORY_WINDOW_MS);
          series.push({ tMs, v: num });
          if (series.length > HISTORY_MAX_POINTS) {
            series.splice(0, series.length - HISTORY_MAX_POINTS);
          }
          nextHistory[k] = series;
        }
      }
      return { values: nextValues, updatedAt: nextUpdated, history: nextHistory };
    }),
  clear: () => set({ keys: [], values: {}, updatedAt: {}, history: {} }),
}));

export function classifyKeyGroup(key: string): string {
  const idx = key.indexOf(".");
  return idx > 0 ? key.slice(0, idx) : "_other";
}
