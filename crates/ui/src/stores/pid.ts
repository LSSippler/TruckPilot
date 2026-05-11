import { create } from "zustand";
import type { PidProfile } from "@/lib/types";

export interface PidSamplePoint {
  setpoint: number;
  actual: number;
  tMs: number;
}

interface PidState {
  profiles: PidProfile[];
  samples: Record<string, PidSamplePoint[]>;
  setProfiles: (profiles: PidProfile[]) => void;
  pushSample: (profile: string, sample: PidSamplePoint) => void;
  updateProfile: (profile: PidProfile) => void;
  clearSamples: (profile: string) => void;
}

const SAMPLE_WINDOW_MS = 10_000;

export const usePidStore = create<PidState>((set) => ({
  profiles: [],
  samples: {},
  setProfiles: (profiles) => set({ profiles }),
  pushSample: (profile, sample) =>
    set((state) => {
      const cur = state.samples[profile] ?? [];
      const next = [...cur, sample].filter((s) => sample.tMs - s.tMs <= SAMPLE_WINDOW_MS);
      return { samples: { ...state.samples, [profile]: next } };
    }),
  updateProfile: (profile) =>
    set((state) => ({
      profiles: state.profiles.map((p) => (p.name === profile.name ? profile : p)),
    })),
  clearSamples: (profile) =>
    set((state) => ({ samples: { ...state.samples, [profile]: [] } })),
}));
