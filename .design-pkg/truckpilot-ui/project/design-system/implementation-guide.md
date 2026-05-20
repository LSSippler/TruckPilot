# TruckPilot — Implementation Guide

> Eight phases, executable end-to-end in 1–2 sprints. Each phase has: scope, files, what to add/replace, test, risk, acceptance. Phases are isolated — you can ship Phase 1–3 alone and the app already looks meaningfully better.

**Estimated total effort:** ~3.5 dev days for one engineer who already knows the codebase.

---

## Phase 1 — Design tokens (foundation)

**Goal:** every following phase only references tokens. Nothing inline.

**Effort:** 2–3 h
**Risk:** Low. Pure additive CSS; if you keep your old vars side-by-side under different names you can revert instantly.

**Files**
- `crates/ui/src/styles/globals.css` — paste both code blocks from `tokens.md`. Order matters: `:root` block first, `@import "tailwindcss";` next, then `@theme inline {}`.
- `tailwind.config.ts` — **delete it** if it still exists. Tailwind v4 reads the theme from CSS. If you have a `content:` array there for legacy reasons, move it to a `@source` directive in `globals.css`:
  ```css
  @source "../**/*.{ts,tsx}";
  ```
- `crates/ui/src/main.tsx` (or wherever React boots) — make sure `globals.css` is imported **first**, before any component CSS.
- `crates/ui/src/components/ThemeProvider.tsx` (or your equivalent) — flip the theme by setting `document.documentElement.dataset.theme = "light" | "dark"`. Do not toggle a class — the tokens use the `[data-theme]` selector.

**Replace existing**
- Any `bg-zinc-900`, `bg-neutral-950`, `text-gray-400`, `border-zinc-800` → `bg-surface-base`, `bg-surface-card`, `text-fg-muted`, `border-subtle`. Do this with a global find-and-replace as you visit each page in later phases — **do not** try to do all of it now.

**How to test**
1. `pnpm tauri dev` (or your script).
2. Open Settings → toggle theme. Switch should be instant, no flash, every page reads the new tokens.
3. DevTools → `getComputedStyle(document.body).getPropertyValue('--brand')` returns the teal in dark and the cooler teal in light.

**Acceptance**
- Theme toggle works.
- Sidebar, header, footer use only token-based classes.
- No raw hex outside `globals.css`.

---

## Phase 2 — Sidebar + Navigation

**Goal:** restful navigation, two semantic groups, a quiet active state.

**Effort:** 2 h
**Risk:** Low. Pure markup + class swap.

**Files**
- `crates/ui/src/components/Sidebar.tsx` (or `Layout.tsx` if combined).
- New: `crates/ui/src/components/sidebar/NavGroup.tsx`, `crates/ui/src/components/sidebar/NavItem.tsx`.

**Structure**
```tsx
<aside className="w-[var(--sidebar-w)] h-full bg-surface-base border-r border-subtle flex flex-col">
  <div className="h-[var(--header-h)] px-3 flex items-center gap-2 border-b border-subtle">
    <LogoMark />
    <span className="font-sans font-medium tracking-tight">TruckPilot</span>
    <span className="ml-auto text-fg-muted font-mono text-xs">v0.8.4</span>
  </div>

  <nav className="flex-1 overflow-y-auto py-3 px-2 flex flex-col gap-4">
    <NavGroup label="Operation">
      <NavItem to="/"            icon={<Gauge size={14} />}     >Dashboard</NavItem>
      <NavItem to="/blackboard"  icon={<Database size={14} />}  >Blackboard</NavItem>
      <NavItem to="/logs"        icon={<ScrollText size={14} />}>Logs</NavItem>
    </NavGroup>

    <NavGroup label="Configuration">
      <NavItem to="/plugins"  icon={<Boxes size={14} />}    >Plugins</NavItem>
      <NavItem to="/pid"      icon={<SlidersHorizontal size={14} />}>PID Tuning</NavItem>
      <NavItem to="/mods"     icon={<Puzzle size={14} />}   >Mods</NavItem>
      <NavItem to="/settings" icon={<Settings size={14} />} >Settings</NavItem>
    </NavGroup>
  </nav>

  <ETS2StatusIndicator className="border-t border-subtle" />
</aside>
```

**`NavItem` rules**
- Idle: `text-fg-secondary`, no background.
- Hover: `bg-surface-card text-fg`. **No** scale, **no** shadow.
- Active: `bg-brand-soft text-fg`, plus a **2px left bar in `bg-brand`** absolutely-positioned at `left-0`. That bar is the only brand color on the sidebar.

**Group label**
- `text-xs uppercase tracking-[0.08em] text-fg-muted font-sans font-medium px-2 mb-1`.

**Acceptance**
- Two groups visually distinct.
- Active item is unmistakable but quiet.
- No accent color appears in idle/hover.

---

## Phase 3 — Dashboard Layout

**Goal:** three zones; the engage decision visible in 1 second.

**Effort:** 3 h
**Risk:** Medium. Existing Dashboard probably has its own grid — replace top-to-bottom, don't patch.

**File:** `crates/ui/src/routes/Dashboard.tsx`

**Zones**
```tsx
<div className="h-full overflow-y-auto">
  {/* HERO — engage decision + preconditions + navi-status */}
  <section className="grid grid-cols-12 gap-3 p-4 border-b border-subtle">
    <div className="col-span-5">
      <EngageButton state={engageState} hotkey="F5" />
      <PreconditionRow preconditions={preconditions} className="mt-3" />
    </div>
    <div className="col-span-7">
      <NaviStatusCard navi={navi} />
    </div>
  </section>

  {/* LIVE — telemetry, vJoy, cruise */}
  <section className="grid grid-cols-12 gap-3 p-4">
    <Card className="col-span-3">
      <BigNumberDisplay label="Speed" unit="km/h" value={speed} target={cruiseTarget} />
      <TelemetrySparkline data={speedHistory} accent="brand" className="mt-2" />
    </Card>
    <Card className="col-span-3">
      <BigNumberDisplay label="RPM" value={rpm} mono />
      <TelemetrySparkline data={rpmHistory} accent="chart-2" className="mt-2" />
    </Card>
    <Card className="col-span-6">
      <h3 className="text-fg-muted text-xs uppercase tracking-wider mb-3">vJoy Output</h3>
      <VJoyBar variant="bipolar"  label="Steering" value={vjoy.steering} />
      <VJoyBar variant="unipolar" label="Throttle" value={vjoy.throttle} className="mt-2" />
      <VJoyBar variant="unipolar" label="Brake"    value={vjoy.brake}    className="mt-2" />
      {vjoy.failsafe && <FailsafeBanner className="mt-3" reason={vjoy.failsafeReason} />}
    </Card>
  </section>

  {/* DETAIL — cruise, route preview, diagnostic */}
  <section className="grid grid-cols-12 gap-3 px-4 pb-4">
    <Card className="col-span-4">
      <h3 className="text-fg-muted text-xs uppercase tracking-wider mb-3">Cruise</h3>
      <CruiseSlider value={cruiseTarget} onChange={setCruiseTarget} />
    </Card>
    <Card className="col-span-8">
      <h3 className="text-fg-muted text-xs uppercase tracking-wider mb-3">Route preview</h3>
      <RouteMap nodes={routeNodes} route={naviRoute} />
    </Card>
  </section>
</div>
```

**`Card` primitive** — re-use shadcn `Card`, but force these classes via a project-local wrapper:
```tsx
className="bg-surface-card border border-subtle rounded-md p-4"
```
No shadow in dark.

**What goes away**
- Any "Goal UID" input on Dashboard → moves to Settings → Diagnostics (hidden by default behind a "Show advanced" toggle).
- Any oversized "Engage" hero that takes a third of the screen — the new EngageButton is a normal-height (40px) button. Importance comes from position + the live state ring, not size.

**Acceptance**
- A new user opens the app and finds Engage in <1 s.
- No card has both a shadow and a border.
- Brand color appears at most twice on screen (Engage when engaged + sidebar active item).

---

## Phase 4 — Hero components

**Goal:** the three components above the fold.

**Effort:** 4 h
**Risk:** Medium (state machine in EngageButton).

**New files**
- `crates/ui/src/components/EngageButton.tsx`
- `crates/ui/src/components/PreconditionPill.tsx`
- `crates/ui/src/components/NaviStatusCard.tsx`

**Zustand wiring** — assume you already have:
```ts
// crates/ui/src/state/blackboard.ts
export const useBlackboard = create<BlackboardState>(...);
// selectors used here:
useEngageState();         // 'off-ready' | 'off-disabled' | 'engaging' | 'engaged' | 'disengaging' | 'fault'
usePreconditions();       // { engine, cruise, naviActive, telemetryHealthy, vjoy }
useNavi();                // { currentRoad, nextManeuver, distanceToManeuver, totalDistance, eta } | null
```

Code is in `design-system/components/`. Drop in, then in `Dashboard.tsx`:
```tsx
const engageState = useEngageState();
const preconds    = usePreconditions();
const navi        = useNavi();

const onEngage = () => invoke("engage_toggle");  // existing Tauri command
```

**Acceptance**
- All 6 EngageButton states visually distinct *without* color being the only differentiator (icon and label change too — colorblind-safe).
- Precondition pills inline beside the button, never wrap to 2 rows on a >=1280px viewport.
- NaviStatusCard `no-navi` state is calm: muted text, no warning color. The hint reads "Set a destination in ETS2 to begin." That's it.

---

## Phase 5 — Live-display components

**Goal:** values are legible at a glance, charts don't shout.

**Effort:** 4 h
**Risk:** Low–medium (Recharts theming).

**Files**
- `crates/ui/src/components/BigNumberDisplay.tsx` (new)
- `crates/ui/src/components/TelemetrySparkline.tsx` (replace existing — strip gradients off, single 1.5px stroke, optional 14%-opacity fill)
- `crates/ui/src/components/VJoyBar.tsx` (replace)
- `crates/ui/src/components/FailsafeBanner.tsx` (new)

**Recharts adjustments**
- `<Line strokeWidth={1.5} dot={false} isAnimationActive={false} />`
- Axis lines off, grid off (sparklines), or grid `stroke="var(--border-subtle)" strokeDasharray="2 4"` for full charts.
- Tooltip: replace default with a tiny div using `bg-surface-overlay border border-subtle font-mono text-xs px-2 py-1 rounded-sm`.

**Animation budget**
- BigNumberDisplay: value-change → 320 ms `value-flash` color animation, **only** when the rounded integer changes (debounce).
- VJoyBar: width transitions at `--dur-1` (120 ms). No bar pulse.
- Sparkline: no pulse on the trailing dot — just a static 3 px dot.

**Acceptance**
- Open Dashboard, drive in ETS2 — eyes can rest on a single value without flicker.
- Switching to light theme: bars still readable, no white-on-white.

---

## Phase 6 — Page-by-page polish

**Goal:** every page reads from the token system, every component uses the right primitive.

**Effort:** 8–10 h total — split per page.

### 6a. Plugins (`crates/ui/src/routes/Plugins.tsx`)
- Replace each row with `<PluginToggleItem>`.
- Detail panel on the right: re-use shadcn `Tabs`. Tab list: `border-b border-subtle`, active trigger `text-fg border-b-2 border-brand -mb-px`.
- Effort: 1.5 h.

### 6b. Blackboard (`crates/ui/src/routes/Blackboard.tsx`)
- Tree column: virtualized list (you already have `react-window`) of `<BlackboardTreeItem>`.
- Detail: `<BlackboardDetail>` — value in mono, type badge muted, copy button right-aligned, optional sparkline for numeric series.
- Effort: 2 h.

### 6c. Logs (`crates/ui/src/routes/Logs.tsx`)
- Filter bar: a single row of shadcn `ToggleGroup` (DEBUG/INFO/WARN/ERROR) + a search input + a "Tail" toggle. No background fill; just bordered.
- Body: virtualized list of `<LogLine>`. Levels carry only a 2 px left border accent.
- Effort: 1.5 h.

### 6d. PID Tuning (`crates/ui/src/routes/PidTuning.tsx`)
- Tabs (per controller) as in Plugins.
- Sliders: `<CruiseSlider>` pattern reused (but with input boxes).
- Live response chart: full `<Line>` chart, dotted grid, single-color stroke per metric (`chart-1` setpoint, `chart-2` measurement). No fills.
- Effort: 2 h.

### 6e. Mods (`crates/ui/src/routes/Mods.tsx`)
- List rows nearly identical to plugin rows. Use `<PluginToggleItem>` (rename prop API to `<ListRow>`-style if you prefer — see component file).
- "Apply" CTA: solid brand button, top-right.
- Effort: 0.5 h.

### 6f. Settings (`crates/ui/src/routes/Settings.tsx`)
- Section headers: `text-fg-muted text-xs uppercase tracking-wider mt-6 mb-2`.
- Cards: same `Card` primitive as Dashboard.
- Hotkeys table: 2-column grid, mono kbd badges.
- Effort: 1.5 h.

**Acceptance for Phase 6 overall**
- No page has its own one-off accent color.
- Every text size resolves to one of `--text-xs..3xl`.
- Every spacing resolves to one of `--space-1..12`.

---

## Phase 7 — Toasts + Notifications

**Goal:** Sonner that matches the system.

**Effort:** 1 h
**Risk:** Low.

**Files**
- `crates/ui/src/components/CustomToast.tsx` (new — wraps Sonner's `toast.custom`).
- `crates/ui/src/main.tsx` — mount `<Toaster position="bottom-right" theme="dark" toastOptions={{...}} />` after the router.

**Use cases (be sparse)**
- **Engage Engaged** — info toast, 1.6 s. "Engaged at 0 km/h."
- **Engage Fault** — danger toast, sticky until dismiss. "Failsafe: <reason>."
- **Navi acquired** — info, 1.6 s. "Following ETS2 route to Hannover."
- **Plugin error** — warning toast, 4 s, action: "Open logs".

No toasts for "connected", "settings saved", "throttle changed". Trust the user to see the in-place state change.

**Acceptance**
- Toast width ≤ 360 px. No card shadow in dark. One-line title + optional one-line body.

---

## Phase 8 — Animations + reduced motion

**Goal:** every animation justifies its existence.

**Effort:** 1 h to audit + remove anything that doesn't pass the self-check.

**The four animations we keep**
1. `tp-fade-in` (180 ms) — route changes, modal mounts.
2. `tp-slide-up` (180 ms) — toasts, popovers.
3. `tp-pulse-soft` (2.4 s) — **only** EngageButton `engaged` ring + FailsafeBanner border.
4. `value-flash` (320 ms) — BigNumberDisplay integer change.

Everything else is a CSS `transition: color/background-color/border-color/transform var(--dur-1) var(--ease-out)`.

**Reduced motion** — the `@media (prefers-reduced-motion: reduce)` block in `globals.css` already collapses all of the above. Verify with macOS System Settings → Accessibility → Display → Reduce motion.

**Acceptance**
- The pulse appears in exactly two places.
- With reduce-motion on, nothing moves except instant state changes.

---

## Migration order — what to merge first

1. Phase 1 alone → ship.
2. Phase 2 + 3 → ship. App looks like a different product.
3. Phase 4 → ship. Hero is now the strongest part of the app.
4. Phase 5 → ship. Live data feels solid.
5. Phase 6 → roll out page by page; each page is its own PR.
6. Phase 7 + 8 → final polish, one PR each.

**Do not** combine Phase 1 with Phase 3 in one PR. Token migration in isolation is reviewable; mixed with layout it isn't.
