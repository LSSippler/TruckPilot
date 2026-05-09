import { create } from "zustand";
import type { ModInfo } from "@/lib/types";

export interface BuildProgress {
  phase: string;
  percent: number;
  etaSeconds: number | null;
}

export interface BuildResult {
  ok: boolean;
  fromCache: boolean;
  message: string;
}

interface ModsState {
  list: ModInfo[];
  active: string[];
  progress: BuildProgress | null;
  result: BuildResult | null;
  setList: (mods: ModInfo[]) => void;
  setActive: (active: string[]) => void;
  setProgress: (progress: BuildProgress) => void;
  setResult: (result: BuildResult) => void;
  clearProgress: () => void;
}

export const useModsStore = create<ModsState>((set) => ({
  list: [],
  active: [],
  progress: null,
  result: null,
  setList: (list) =>
    set((state) => ({
      list,
      active: state.active.length === 0 ? list.filter((m) => m.enabled).map((m) => m.name) : state.active,
    })),
  setActive: (active) => set({ active }),
  setProgress: (progress) => set({ progress }),
  setResult: (result) => set({ result, progress: null }),
  clearProgress: () => set({ progress: null }),
}));
