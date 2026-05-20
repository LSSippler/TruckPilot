# TruckPilot — Design System Delivery

> Restrained, modern, dark-first. Linear/Vercel/Stripe direction. Brand gradient as accent, not a default.

## What's in this folder

```
design-system/
├── tokens.md                  # A) CSS custom properties + Tailwind 4 @theme inline
├── implementation-guide.md    # B) 8 phases, file paths, diffs, acceptance criteria
├── route-map-integration.md   # RouteMap-specific wiring + daemon-side blackboard contract
├── visual-spec.html           # D) Static visual reference of the dashboard
├── types/
│   └── map.ts                 # MapNode / MapEdge / WorldPoint contracts
└── components/                # C) 17 React + TypeScript components
    ├── EngageButton.tsx
    ├── PreconditionPill.tsx        (+ PreconditionRow named export)
    ├── TelemetrySparkline.tsx
    ├── BigNumberDisplay.tsx
    ├── VJoyBar.tsx                 (variant: bipolar | unipolar)
    ├── FailsafeBanner.tsx
    ├── CruiseSlider.tsx
    ├── NaviStatusCard.tsx
    ├── PluginToggleItem.tsx
    ├── LogLine.tsx
    ├── BlackboardTreeItem.tsx
    ├── BlackboardDetail.tsx
    ├── CustomToast.tsx             (notify.* helpers wrapping sonner)
    ├── RouteMap.tsx                (ETS2LA-style: pan/zoom/follow/debug, empty states)
    ├── SpeedLimitDisplay.tsx
    ├── DiagnosticPanel.tsx         (skeleton)
    └── ETS2StatusIndicator.tsx
```

## Where to start

1. **Read `tokens.md`** — paste both code blocks into `crates/ui/src/styles/globals.css`. That's Phase 1.
2. **Open `visual-spec.html` in a browser** — it's the static reference for the dashboard look. Theme toggle in the header (top-right) flips light/dark; everything reads from the same tokens you're about to ship.
3. **Follow `implementation-guide.md`** phase-by-phase. Each phase is a small PR.
4. **Drop `components/*.tsx`** into `crates/ui/src/components/` as you need them, adjusting imports (`@/lib/utils`, `@/components/ui/*`) to your project's path aliases.

## Component import assumptions

The components assume your project already has:

- `@/lib/utils` exporting `cn(...)` (the standard shadcn helper combining `clsx` + `tailwind-merge`).
- `@/components/ui/*` for shadcn primitives (`Switch`, `Tooltip`, ...).
- Path alias `@/` pointing at `crates/ui/src/` (or equivalent).

If your aliases differ, the only change per file is the import line at the top.

## Design principles (the self-check)

Before shipping any new component, ask:

1. Do I need this accent, or is regular text clear enough?
2. Do I need this animation, or does it distract?
3. Do I need this gradient, or does a solid suffice?
4. Would Linear / Vercel / Stripe ship this, or would they cut it back?

If three of four answers are "cut it back", cut it back.

## Where the brand gradient actually appears

Exactly three places:

1. The logo glyph in the sidebar header.
2. The 1px ring around the **engaged** EngageButton (soft-pulse).
3. Optional 14%-opacity fill under a single live sparkline.

Anywhere else: solid `--brand` or no color at all.
