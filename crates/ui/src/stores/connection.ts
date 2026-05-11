import { create } from "zustand";
import type { ConnectionStatus, ConnectionStatusEvent } from "@/lib/types";

interface ConnectionState {
  status: ConnectionStatus;
  protocolVersion: string | null;
  lastError: string | null;
  helloReceived: boolean;
  setStatus: (event: ConnectionStatusEvent) => void;
  setHello: (version: string, v: number) => void;
}

export const useConnectionStore = create<ConnectionState>((set) => ({
  status: "disconnected",
  protocolVersion: null,
  lastError: null,
  helloReceived: false,
  setStatus: (event) =>
    set({
      status: event.status,
      protocolVersion: event.protocol_version,
      lastError: event.last_error,
      helloReceived: event.status === "connected" ? true : false,
    }),
  setHello: (version, _v) =>
    set({
      protocolVersion: version,
      helloReceived: true,
      status: "connected",
    }),
}));
