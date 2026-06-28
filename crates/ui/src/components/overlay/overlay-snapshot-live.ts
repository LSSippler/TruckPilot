import {
  createLiveOverlaySnapshotShell,
  parseOverlaySnapshot,
  readOverlaySnapshotFromStorage,
  type OverlaySnapshot,
} from "./overlay-snapshot";

export type PollLiveOverlaySnapshotOptions = {
  /** Raw JSON from Tauri `read_overlay_snapshot_file` when available. */
  tauriFileJson?: string | null;
  /** When false, skip Tauri file (browser/Vite). */
  tauriFileEnabled?: boolean;
  internalVisualization: boolean;
};

/** Resolve live overlay snapshot: Tauri loop file → localStorage → empty shell. */
export function pollLiveOverlaySnapshot(options: PollLiveOverlaySnapshotOptions): OverlaySnapshot | null {
  if (options.tauriFileEnabled !== false && options.tauriFileJson) {
    const fromFile = parseOverlaySnapshot(options.tauriFileJson);
    if (fromFile) return fromFile;
  }
  const fromStorage = readOverlaySnapshotFromStorage();
  if (fromStorage) return fromStorage;
  if (options.internalVisualization) return createLiveOverlaySnapshotShell();
  return null;
}

export type PollLiveOverlaySnapshotAsyncOptions = {
  internalVisualization: boolean;
  readOverlaySnapshotFile?: () => Promise<string | null>;
  isTauriRuntime?: () => boolean;
};

/** Async live poll wrapper for the overlay hook (Tauri file read + fallbacks). */
export async function pollLiveOverlaySnapshotAsync(
  options: PollLiveOverlaySnapshotAsyncOptions,
): Promise<OverlaySnapshot | null> {
  const inTauri = options.isTauriRuntime?.() ?? false;
  let tauriFileJson: string | null = null;
  if (inTauri && options.readOverlaySnapshotFile) {
    try {
      tauriFileJson = await options.readOverlaySnapshotFile();
    } catch {
      tauriFileJson = null;
    }
  }
  return pollLiveOverlaySnapshot({
    tauriFileJson,
    tauriFileEnabled: inTauri,
    internalVisualization: options.internalVisualization,
  });
}
