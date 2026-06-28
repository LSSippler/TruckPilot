import { OVERLAY_LAYOUT_EDITOR_KEY } from "./overlay-layout";

export function OverlayLayoutEditorBanner({
  editorMode,
  onReset,
}: {
  editorMode: boolean;
  onReset: () => void;
}) {
  if (!editorMode) {
    return (
      <div className="pointer-events-none absolute bottom-2 left-1/2 z-[60] -translate-x-1/2 rounded bg-black/45 px-2 py-0.5 text-[10px] text-white/45">
        {OVERLAY_LAYOUT_EDITOR_KEY}: Layout-Editor
      </div>
    );
  }

  return (
    <div className="pointer-events-auto absolute bottom-2 left-1/2 z-[60] flex -translate-x-1/2 items-center gap-2 rounded-md bg-amber-500/20 px-3 py-1.5 text-[10px] text-amber-50 ring-1 ring-amber-400/40">
      <span>
        <strong>Layout-Editor</strong> · Panels ziehen · Internal Viz: Mausrad / + − ·{" "}
        {OVERLAY_LAYOUT_EDITOR_KEY} beenden
      </span>
      <button
        type="button"
        className="rounded bg-black/40 px-1.5 py-0.5 text-[9px] uppercase tracking-wide text-white/80 hover:bg-black/60"
        onClick={onReset}
      >
        Reset
      </button>
    </div>
  );
}
