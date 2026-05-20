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
  pushMany: (entries: LogEntry[]) => void;
  setFilter: (filter: Partial<Pick<LogsState, "filterLevels" | "filterPlugin" | "search">>) => void;
  clear: () => void;
}

const DEFAULT_MAX = 5_000;

function appendBounded(prev: LogEntry[], add: LogEntry[], max: number): LogEntry[] {
  if (add.length === 0) return prev;
  const total = prev.length + add.length;
  if (total <= max) return prev.concat(add);
  if (add.length >= max) return add.slice(add.length - max);
  // prev keeps its tail of size (max - add.length).
  const keep = max - add.length;
  const next = new Array<LogEntry>(max);
  for (let i = 0; i < keep; i++) next[i] = prev[prev.length - keep + i]!;
  for (let i = 0; i < add.length; i++) next[keep + i] = add[i]!;
  return next;
}

export const useLogsStore = create<LogsState>((set) => ({
  entries: [],
  maxEntries: DEFAULT_MAX,
  filterLevels: ["trace", "debug", "info", "warn", "error"],
  filterPlugin: null,
  search: "",
  push: (entry) =>
    set((state) => ({ entries: appendBounded(state.entries, [entry], state.maxEntries) })),
  pushMany: (entries) =>
    set((state) => ({ entries: appendBounded(state.entries, entries, state.maxEntries) })),
  setFilter: (filter) => set((state) => ({ ...state, ...filter })),
  clear: () => set({ entries: [] }),
}));

export function selectFilteredLogs(state: LogsState): LogEntry[] {
  return filterEntries(state.entries, state.filterLevels, state.filterPlugin, state.search);
}

export function filterEntries(
  entries: LogEntry[],
  levels: LogLevel[],
  plugin: string | null,
  search: string,
): LogEntry[] {
  const needle = search ? search.toLowerCase() : "";
  const levelSet = new Set(levels);
  return entries.filter((entry) => {
    if (!levelSet.has(entry.level)) return false;
    if (plugin && entry.plugin !== plugin) return false;
    if (needle && !entry.message.toLowerCase().includes(needle)) return false;
    return true;
  });
}
