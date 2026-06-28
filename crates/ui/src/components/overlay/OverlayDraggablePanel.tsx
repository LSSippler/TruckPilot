import { useCallback, useRef, type ReactNode } from "react";
import { cn } from "@/lib/utils";
import type { OverlayPanelId, OverlayPanelLayout } from "./overlay-layout";

export function OverlayDraggablePanel({
  id,
  label,
  editorMode,
  layout,
  onLayoutChange,
  interactive = false,
  className,
  children,
}: {
  id: OverlayPanelId;
  label: string;
  editorMode: boolean;
  layout: OverlayPanelLayout;
  onLayoutChange: (next: OverlayPanelLayout) => void;
  /** Keep pointer-events when not editing (e.g. snapshot import buttons). */
  interactive?: boolean;
  className?: string;
  children: ReactNode;
}) {
  const dragRef = useRef<{ startX: number; startY: number; originX: number; originY: number } | null>(
    null,
  );

  const onPointerDown = useCallback(
    (e: React.PointerEvent<HTMLDivElement>) => {
      if (!editorMode) return;
      e.preventDefault();
      e.stopPropagation();
      dragRef.current = {
        startX: e.clientX,
        startY: e.clientY,
        originX: layout.x,
        originY: layout.y,
      };
      e.currentTarget.setPointerCapture(e.pointerId);
    },
    [editorMode, layout.x, layout.y],
  );

  const onPointerMove = useCallback(
    (e: React.PointerEvent<HTMLDivElement>) => {
      if (!editorMode || !dragRef.current) return;
      const dx = e.clientX - dragRef.current.startX;
      const dy = e.clientY - dragRef.current.startY;
      onLayoutChange({
        ...layout,
        x: Math.max(0, dragRef.current.originX + dx),
        y: Math.max(0, dragRef.current.originY + dy),
      });
    },
    [editorMode, layout, onLayoutChange],
  );

  const onPointerUp = useCallback((e: React.PointerEvent<HTMLDivElement>) => {
    dragRef.current = null;
    if (e.currentTarget.hasPointerCapture(e.pointerId)) {
      e.currentTarget.releasePointerCapture(e.pointerId);
    }
  }, []);

  return (
    <div
      className={cn(
        "absolute max-w-[min(100vw-1.5rem,20rem)]",
        editorMode
          ? "pointer-events-auto z-50 cursor-grab active:cursor-grabbing ring-2 ring-amber-400/70 ring-offset-1 ring-offset-transparent"
          : interactive
            ? "pointer-events-auto z-10"
            : "pointer-events-none z-10",
        className,
      )}
      style={{ left: layout.x, top: layout.y }}
      data-overlay-panel={id}
      onPointerDown={onPointerDown}
      onPointerMove={onPointerMove}
      onPointerUp={onPointerUp}
      onPointerCancel={onPointerUp}
    >
      {editorMode ? (
        <div className="mb-0.5 flex items-center gap-1 rounded bg-amber-500/25 px-1.5 py-0.5 text-[9px] font-semibold uppercase tracking-wide text-amber-100">
          <span aria-hidden>⋮⋮</span>
          {label}
        </div>
      ) : null}
      {children}
    </div>
  );
}
