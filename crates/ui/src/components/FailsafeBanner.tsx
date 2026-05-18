import { AlertOctagon, X } from "lucide-react";
import { cn } from "@/lib/utils";

export interface FailsafeBannerProps {
  reason: string;
  hint?: string;
  onClear?: () => void;
  className?: string;
}

export function FailsafeBanner({ reason, hint, onClear, className }: FailsafeBannerProps) {
  return (
    <div
      role="alert"
      className={cn(
        "relative flex items-start gap-2.5 pl-3 pr-2 py-2 rounded-sm",
        "bg-danger-soft border border-subtle",
        className,
      )}
    >
      <span
        aria-hidden
        className="absolute left-0 top-0 bottom-0 w-[2px] bg-danger animate-tp-pulse-soft rounded-l-sm"
      />
      <AlertOctagon size={14} className="text-danger mt-0.5 shrink-0" />
      <div className="flex-1 min-w-0">
        <p className="text-sm font-sans text-fg">
          <span className="text-danger font-medium">Failsafe — </span>
          {reason}
        </p>
        {hint && <p className="text-xs text-fg-muted font-sans mt-0.5">{hint}</p>}
      </div>
      {onClear && (
        <button
          type="button"
          onClick={onClear}
          aria-label="Clear failsafe"
          className="shrink-0 inline-flex items-center justify-center w-6 h-6 rounded-sm text-fg-muted hover:text-fg hover:bg-surface-elevated transition-colors"
        >
          <X size={14} />
        </button>
      )}
    </div>
  );
}
