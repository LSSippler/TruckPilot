import { create } from "zustand";
import type { PluginEventKind, PluginInfo } from "@/lib/types";

interface PluginsState {
  list: PluginInfo[];
  order: string[];
  selected: string | null;
  schemas: Record<string, unknown>;
  setList: (plugins: PluginInfo[]) => void;
  applyEvent: (plugin: string, event: PluginEventKind) => void;
  setSchema: (plugin: string, schema: unknown) => void;
  select: (name: string | null) => void;
  reorder: (from: number, to: number) => void;
}

export const usePluginsStore = create<PluginsState>((set, get) => ({
  list: [],
  order: [],
  selected: null,
  schemas: {},
  setList: (plugins) =>
    set((state) => ({
      list: plugins,
      order: plugins.map((p) => p.name),
      selected: state.selected && plugins.some((p) => p.name === state.selected) ? state.selected : null,
    })),
  applyEvent: (plugin, event) =>
    set((state) => {
      switch (event.kind) {
        case "loaded":
        case "settings_changed":
          return state;
        case "unloaded":
          return {
            list: state.list.filter((p) => p.name !== plugin),
            order: state.order.filter((n) => n !== plugin),
          };
        case "crashed":
          return {
            list: state.list.map((p) => (p.name === plugin ? { ...p, enabled: false } : p)),
          };
        default:
          return state;
      }
    }),
  setSchema: (plugin, schema) =>
    set((state) => ({ schemas: { ...state.schemas, [plugin]: schema } })),
  select: (name) => set({ selected: name }),
  reorder: (from, to) => {
    const order = [...get().order];
    if (from < 0 || from >= order.length || to < 0 || to >= order.length) return;
    const [moved] = order.splice(from, 1);
    if (moved !== undefined) {
      order.splice(to, 0, moved);
      set({ order });
    }
  },
}));
