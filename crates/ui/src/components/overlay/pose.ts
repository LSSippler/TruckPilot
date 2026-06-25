// Phase 6.6a-1 — live truck pose for the overlay projection.
//
// Position + heading ride the typed `core-event` telemetry push (~20 Hz, no
// parsing). pitch/roll are NOT in that snapshot — they only exist as blackboard
// keys, written every daemon tick (~50 Hz) in core/main.rs. The shared poller
// fetches blackboard at 500 ms (2 Hz); for the pose we add a dedicated fast
// `blackboard_get` for just those two keys. The daemon handler is a cheap
// mutex+hashmap lookup, so this needs no daemon change.

import { sendCommand } from "@/lib/ipc";
import { useBlackboardStore } from "@/stores/blackboard";
import { useConnectionStore } from "@/stores/connection";
import { useTelemetryStore } from "@/stores/telemetry";
import type { Vec3 } from "@/lib/projection";

/** Blackboard keys carrying truck pitch/roll (SCS 0..1 turns). */
export const POSE_BB_KEYS = ["telemetry.pitch", "telemetry.roll"] as const;

/** Dedicated fast poll cadence for pitch/roll (ms). 100 ms ≈ 10 Hz — faster
 *  than the shared 500 ms blackboard poll, without touching the daemon. */
export const POSE_POLL_MS = 100;

/** Telemetry older than this (ms) → pose treated as unavailable.
 *  Matches TruckPose::is_fresh (1 s) in crates/overlay/src/telemetry.rs. */
export const POSE_STALE_MS = 1000;

export interface LivePose {
  /** Truck reference-point position, world metres. */
  pos: Vec3;
  /** Heading, SCS 0..1 turns. */
  heading: number;
  /** Pitch, SCS 0..1 turns. */
  pitch: number;
  /** Roll, SCS 0..1 turns. */
  roll: number;
  /** Age of the position/heading sample (ms). */
  ageMs: number;
}

function num(v: string | undefined): number {
  if (v == null || v === "") return 0;
  const n = Number(v);
  return Number.isFinite(n) ? n : 0;
}

/** Read the freshest pose by combining the typed telemetry store (position +
 *  heading) with blackboard pitch/roll. Returns null when telemetry is absent
 *  or stale. Reads via getState() so the rAF caller never subscribes/re-renders. */
export function readLivePose(): LivePose | null {
  const tel = useTelemetryStore.getState();
  const snap = tel.latest;
  if (!snap || tel.lastUpdateMs === 0) return null;
  const ageMs = Date.now() - tel.lastUpdateMs;
  if (ageMs > POSE_STALE_MS) return null;

  const bb = useBlackboardStore.getState().values;
  return {
    pos: [snap.position[0], snap.position[1], snap.position[2]],
    heading: snap.heading,
    pitch: num(bb["telemetry.pitch"]),
    roll: num(bb["telemetry.roll"]),
    ageMs,
  };
}

/** Start the dedicated fast pitch/roll poll. Returns a stop function. Polls
 *  only while the shared daemon connection is "connected". */
export function startPosePoll(): () => void {
  const id = setInterval(() => {
    if (useConnectionStore.getState().status !== "connected") return;
    void sendCommand({ type: "blackboard_get", keys: [...POSE_BB_KEYS] });
  }, POSE_POLL_MS);
  return () => clearInterval(id);
}
