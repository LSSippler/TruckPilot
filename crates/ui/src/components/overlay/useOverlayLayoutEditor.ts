import { useCallback, useEffect, useState } from "react";
import { overlaySetLayoutEditor } from "@/lib/tauri-bridge";
import {
  isOverlayLayoutEditorForced,
  loadOverlayLayout,
  OVERLAY_LAYOUT_EDITOR_KEY,
  resetOverlayLayout,
  saveOverlayLayout,
  type OverlayLayoutState,
  type OverlayPanelId,
  type OverlayPanelLayout,
  setPanelLayout,
} from "./overlay-layout";

export function useOverlayLayoutEditor(search: URLSearchParams) {
  const forced = isOverlayLayoutEditorForced(search);
  const [editorMode, setEditorMode] = useState(forced);
  const [layoutState, setLayoutState] = useState<OverlayLayoutState>(() => loadOverlayLayout());
  const [viewport, setViewport] = useState(() => ({
    w: window.innerWidth,
    h: window.innerHeight,
  }));

  useEffect(() => {
    const onResize = () => {
      setViewport({ w: window.innerWidth, h: window.innerHeight });
    };
    window.addEventListener("resize", onResize);
    return () => window.removeEventListener("resize", onResize);
  }, []);

  useEffect(() => {
    void overlaySetLayoutEditor(editorMode).catch(() => {
      // Browser dev (Vite) has no Tauri window — CSS pointer-events still work.
    });
    document.documentElement.classList.toggle("overlay-layout-editor", editorMode);
    return () => {
      document.documentElement.classList.remove("overlay-layout-editor");
      void overlaySetLayoutEditor(false).catch(() => {});
    };
  }, [editorMode]);

  useEffect(() => {
    if (forced) return;
    const onKey = (e: KeyboardEvent) => {
      if (e.key !== OVERLAY_LAYOUT_EDITOR_KEY) return;
      if (e.ctrlKey || e.altKey || e.metaKey) return;
      const tag = (e.target as HTMLElement | null)?.tagName;
      if (tag === "INPUT" || tag === "TEXTAREA" || tag === "SELECT") return;
      e.preventDefault();
      setEditorMode((v) => !v);
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [forced]);

  const persistPanel = useCallback((id: OverlayPanelId, layout: OverlayPanelLayout) => {
    setLayoutState((prev) => {
      const next = setPanelLayout(prev, id, layout);
      saveOverlayLayout(next);
      return next;
    });
  }, []);

  const resetLayout = useCallback(() => {
    const next = resetOverlayLayout();
    setLayoutState(next);
  }, []);

  return {
    editorMode,
    layoutState,
    viewport,
    persistPanel,
    resetLayout,
  };
}
