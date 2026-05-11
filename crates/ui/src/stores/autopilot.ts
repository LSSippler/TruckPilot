import { create } from "zustand";
import type { AutopilotState, PreconditionSnapshot } from "@/lib/types";

const initialPreconditions: PreconditionSnapshot = {
  telemetry_ok: false,
  engine_running: false,
  cruise_active: false,
  critical_plugins_loaded: false,
  router_active: false,
};

export interface AutopilotStatusPayload {
  state: AutopilotState;
  faultReason: string | null;
  preconditions: PreconditionSnapshot;
  tickCount: number;
}

interface AutopilotStore {
  state: AutopilotState | null;
  faultReason: string | null;
  preconditions: PreconditionSnapshot;
  tickCount: number;
  lastUpdateMs: number;
  setStatus: (payload: AutopilotStatusPayload) => void;
  clear: () => void;
}

export const useAutopilotStore = create<AutopilotStore>((set) => ({
  state: null,
  faultReason: null,
  preconditions: initialPreconditions,
  tickCount: 0,
  lastUpdateMs: 0,
  setStatus: (payload) =>
    set({
      state: payload.state,
      faultReason: payload.faultReason,
      preconditions: payload.preconditions,
      tickCount: payload.tickCount,
      lastUpdateMs: Date.now(),
    }),
  clear: () =>
    set({
      state: null,
      faultReason: null,
      preconditions: initialPreconditions,
      tickCount: 0,
      lastUpdateMs: 0,
    }),
}));
