import { create } from "zustand";
import type { LogLevel } from "@/lib/types";

export interface LogEntry {
  ts: number;
  level: LogLevel;
  message: string;
  plugin: string | null;
}

interface LogsState {
  entries: LogEntry[];
  maxEntries: number;
  filterLevels: LogLevel[];
  filterPlugin: string | null;
  search: string;
  push: (entry: LogEntry) => void;
  setFilter: (filter: Partial<Pick<LogsState, "filterLevels" | "filterPlugin" | "search">>) => void;
  clear: () => void;
}

const DEFAULT_MAX = 5_000;

export const useLogsStore = create<LogsState>((set) => ({
  entries: [],
  maxEntries: DEFAULT_MAX,
  filterLevels: ["trace", "debug", "info", "warn", "error"],
  filterPlugin: null,
  search: "",
  push: (entry) =>
    set((state) => {
      const next = [...state.entries, entry];
      if (next.length > state.maxEntries) next.splice(0, next.length - state.maxEntries);
      return { entries: next };
    }),
  setFilter: (filter) => set((state) => ({ ...state, ...filter })),
  clear: () => set({ entries: [] }),
}));

export function selectFilteredLogs(state: LogsState): LogEntry[] {
  return state.entries.filter((entry) => {
    if (!state.filterLevels.includes(entry.level)) return false;
    if (state.filterPlugin && entry.plugin !== state.filterPlugin) return false;
    if (state.search && !entry.message.toLowerCase().includes(state.search.toLowerCase())) return false;
    return true;
  });
}
