import * as React from "react";
import { NavLink } from "react-router-dom";
import { cn } from "@/lib/utils";

export interface NavItemProps {
  to: string;
  icon?: React.ReactNode;
  children: React.ReactNode;
  end?: boolean;
}

export function NavItem({ to, icon, children, end }: NavItemProps) {
  return (
    <NavLink
      to={to}
      end={end}
      className={({ isActive }) =>
        cn(
          "relative flex items-center gap-2.5 h-8 px-2 rounded-sm text-sm font-sans",
          "transition-colors duration-[var(--dur-1)]",
          "outline-none focus-visible:ring-1 focus-visible:ring-[var(--ring)]",
          isActive
            ? "bg-brand-soft text-fg"
            : "text-fg-secondary hover:bg-surface-card hover:text-fg",
        )
      }
    >
      {({ isActive }) => (
        <>
          {isActive && (
            <span
              aria-hidden
              className="absolute left-0 top-[4px] bottom-[4px] w-[2px] rounded-full bg-brand"
            />
          )}
          {icon && (
            <span className="shrink-0 text-current" aria-hidden>
              {icon}
            </span>
          )}
          <span className="truncate">{children}</span>
        </>
      )}
    </NavLink>
  );
}
