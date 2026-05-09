import { create } from "zustand";

export type Theme = "light" | "dark" | "system";

interface SettingsState {
  theme: Theme;
  autoConnect: boolean;
  ets2Path: string | null;
  cacheDir: string | null;
  hotkeyEngage: string;
  hotkeyDisengage: string;
  setTheme: (theme: Theme) => void;
  setAutoConnect: (auto: boolean) => void;
  setEts2Path: (path: string | null) => void;
  setCacheDir: (path: string | null) => void;
  setHotkeyEngage: (key: string) => void;
  setHotkeyDisengage: (key: string) => void;
}

const STORAGE_KEY = "truckpilot.settings.v1";

interface PersistedSettings {
  theme: Theme;
  autoConnect: boolean;
  ets2Path: string | null;
  cacheDir: string | null;
  hotkeyEngage: string;
  hotkeyDisengage: string;
}

function loadPersisted(): PersistedSettings {
  if (typeof window === "undefined") return defaults();
  try {
    const raw = window.localStorage.getItem(STORAGE_KEY);
    if (!raw) return defaults();
    return { ...defaults(), ...(JSON.parse(raw) as Partial<PersistedSettings>) };
  } catch {
    return defaults();
  }
}

function defaults(): PersistedSettings {
  return {
    theme: "system",
    autoConnect: true,
    ets2Path: null,
    cacheDir: null,
    hotkeyEngage: "F5",
    hotkeyDisengage: "F6",
  };
}

function persist(state: PersistedSettings) {
  if (typeof window === "undefined") return;
  window.localStorage.setItem(STORAGE_KEY, JSON.stringify(state));
}

const initial = loadPersisted();

export const useSettingsStore = create<SettingsState>((set, get) => {
  const update = (patch: Partial<PersistedSettings>) => {
    set(patch);
    const { theme, autoConnect, ets2Path, cacheDir, hotkeyEngage, hotkeyDisengage } = get();
    persist({ theme, autoConnect, ets2Path, cacheDir, hotkeyEngage, hotkeyDisengage });
  };
  return {
    ...initial,
    setTheme: (theme) => update({ theme }),
    setAutoConnect: (autoConnect) => update({ autoConnect }),
    setEts2Path: (ets2Path) => update({ ets2Path }),
    setCacheDir: (cacheDir) => update({ cacheDir }),
    setHotkeyEngage: (hotkeyEngage) => update({ hotkeyEngage }),
    setHotkeyDisengage: (hotkeyDisengage) => update({ hotkeyDisengage }),
  };
});
