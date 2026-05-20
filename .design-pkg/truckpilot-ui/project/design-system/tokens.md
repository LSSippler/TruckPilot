# TruckPilot Design Tokens

> Drop the two code blocks below into `crates/ui/src/styles/globals.css` (or wherever your global stylesheet lives).
> They are ordered so the `@theme inline` block references the `:root` custom properties.
> Light theme overrides `[data-theme="light"]` on `<html>` or `<body>`.

The brand accent is a single **teal/mint** solid. The full **teal → cyan → blue → violet** gradient from the logo exists as a separate token (`--brand-gradient`) and is used in **exactly three places**: the logo itself, the Engage button in the `engaged` state, and a single active sparkline. Nowhere else.

---

## 1. CSS Custom Properties

```css
/* crates/ui/src/styles/globals.css */

/* ------------------------------------------------------------------ */
/*  TruckPilot — design tokens                                         */
/*  Dark is default. Light is reached via [data-theme="light"].        */
/* ------------------------------------------------------------------ */

:root,
[data-theme="dark"] {
  /* ---- Surfaces (4 layers only) ---- */
  --surface-base:      #0B0C0E;   /* app background                */
  --surface-card:      #111317;   /* cards, panels                 */
  --surface-elevated:  #161A20;   /* hovered card, popovers        */
  --surface-overlay:   #1C2128;   /* dialogs, tooltips             */

  /* ---- Borders (2 strengths) ---- */
  --border-subtle:     #1E2228;
  --border-strong:     #2A2F37;

  /* ---- Text ---- */
  --text-primary:      #E6E8EC;
  --text-secondary:    #9CA3AF;
  --text-muted:        #6B7280;
  --text-disabled:     #44485042;

  /* ---- Brand: ONE solid accent + ONE gradient (rarely used) ---- */
  --brand:             #2DD4BF;   /* teal/mint — default accent    */
  --brand-hover:       #34E0CB;
  --brand-pressed:     #1FB8A4;
  --brand-soft:        #2DD4BF14; /*  8% — tint for selected rows  */
  --brand-soft-strong: #2DD4BF24; /* 14% — focus rings, hovers     */

  --brand-gradient:    linear-gradient(
                         100deg,
                         #2DD4BF 0%,    /* teal  */
                         #22D3EE 35%,   /* cyan  */
                         #3B82F6 65%,   /* blue  */
                         #A78BFA 100%   /* violet */
                       );

  /* ---- Semantic (muted, not poster colors) ---- */
  --success:           #2DD4A0;
  --success-soft:      #2DD4A018;
  --warning:           #E0A52C;
  --warning-soft:      #E0A52C18;
  --danger:            #E5484D;
  --danger-soft:       #E5484D18;
  --info:              #3B82F6;
  --info-soft:         #3B82F618;

  /* ---- Chart palette (categorical, muted) ---- */
  --chart-1:           #2DD4BF;
  --chart-2:           #6BA8FF;
  --chart-3:           #A78BFA;
  --chart-4:           #E0A52C;
  --chart-5:           #F472B6;
  --chart-6:           #9CA3AF;

  /* ---- Typography ---- */
  --font-sans:         "Geist", "Inter", ui-sans-serif, system-ui,
                       -apple-system, "Segoe UI", Roboto, sans-serif;
  --font-mono:         "Geist Mono", "JetBrains Mono", ui-monospace,
                       SFMono-Regular, Menlo, Consolas, monospace;

  --text-xs:           11px;
  --text-sm:           12px;
  --text-base:         13px;   /* dense desktop UI baseline */
  --text-md:           15px;
  --text-lg:           18px;
  --text-xl:           24px;
  --text-2xl:          32px;
  --text-3xl:          48px;   /* big-number readout        */

  --leading-tight:     1.15;
  --leading-snug:      1.35;
  --leading-normal:    1.5;

  /* ---- Spacing (4 / 8 / 12 / 16 / 24 / 32 / 48) ---- */
  --space-1:           4px;
  --space-2:           8px;
  --space-3:           12px;
  --space-4:           16px;
  --space-6:           24px;
  --space-8:           32px;
  --space-12:          48px;

  /* ---- Radii ---- */
  --radius-sm:         4px;    /* default — buttons, inputs, pills */
  --radius-md:         6px;    /* cards, panels                    */
  --radius-full:       999px;  /* avatars, status dots             */

  /* ---- Shadows (use sparingly, mostly Light) ---- */
  --shadow-1:          0 1px 0 0 rgba(0,0,0,0.20);
  --shadow-2:          0 4px 12px -2px rgba(0,0,0,0.35);
  --shadow-pop:        0 12px 32px -8px rgba(0,0,0,0.55);

  /* ---- Focus ring ---- */
  --ring:              #2DD4BF;
  --ring-offset:       var(--surface-base);

  /* ---- Motion ---- */
  --dur-1:             120ms;
  --dur-2:             180ms;
  --dur-3:             240ms;
  --ease-out:          cubic-bezier(0.16, 1, 0.3, 1);
  --ease-in-out:       cubic-bezier(0.4, 0, 0.2, 1);

  /* ---- Layout ---- */
  --sidebar-w:         220px;
  --header-h:          44px;
  --footer-h:          28px;
}

[data-theme="light"] {
  /* Light is deliberately cool-neutral, not warm. */
  --surface-base:      #FBFBFC;
  --surface-card:      #FFFFFF;
  --surface-elevated:  #FFFFFF;
  --surface-overlay:   #FFFFFF;

  --border-subtle:     #ECEEF1;
  --border-strong:     #DADDE2;

  --text-primary:      #0B0C0E;
  --text-secondary:    #4A5160;
  --text-muted:        #6B7280;
  --text-disabled:     #B7BCC4;

  --brand:             #0EA890;
  --brand-hover:       #14B89E;
  --brand-pressed:     #0B8C77;
  --brand-soft:        #0EA89014;
  --brand-soft-strong: #0EA89024;

  --brand-gradient:    linear-gradient(
                         100deg,
                         #0EA890 0%,
                         #0891B2 35%,
                         #2563EB 65%,
                         #7C3AED 100%
                       );

  --success:           #0E9A6E;
  --success-soft:      #0E9A6E14;
  --warning:           #A6741B;
  --warning-soft:      #A6741B14;
  --danger:            #C0383D;
  --danger-soft:       #C0383D14;
  --info:              #2563EB;
  --info-soft:         #2563EB14;

  --chart-1:           #0EA890;
  --chart-2:           #2563EB;
  --chart-3:           #7C3AED;
  --chart-4:           #A6741B;
  --chart-5:           #BE3470;
  --chart-6:           #6B7280;

  --shadow-1:          0 1px 0 0 rgba(15,20,30,0.04);
  --shadow-2:          0 4px 12px -2px rgba(15,20,30,0.08);
  --shadow-pop:        0 12px 32px -8px rgba(15,20,30,0.14);

  --ring:              #0EA890;
  --ring-offset:       var(--surface-base);
}

/* ------------------------------------------------------------------ */
/*  Reduced motion                                                     */
/* ------------------------------------------------------------------ */
@media (prefers-reduced-motion: reduce) {
  *,
  *::before,
  *::after {
    animation-duration: 0.001ms !important;
    animation-iteration-count: 1 !important;
    transition-duration: 0.001ms !important;
  }
}

/* ------------------------------------------------------------------ */
/*  Body baseline                                                      */
/* ------------------------------------------------------------------ */
html, body, #root {
  height: 100%;
  background: var(--surface-base);
  color: var(--text-primary);
  font-family: var(--font-sans);
  font-size: var(--text-base);
  line-height: var(--leading-normal);
  font-feature-settings: "cv11", "ss01", "ss03";
  -webkit-font-smoothing: antialiased;
  text-rendering: optimizeLegibility;
}
```

---

## 2. Tailwind 4 `@theme inline` block

Place **directly after** the `:root` block above. Tailwind v4 picks up custom property aliases through `@theme inline`, so utilities like `bg-surface-card`, `text-muted`, `border-subtle` resolve to your tokens at build time.

```css
/* crates/ui/src/styles/globals.css — continued */

@import "tailwindcss";

@theme inline {
  /* ---- Colors ---- */
  --color-surface-base:     var(--surface-base);
  --color-surface-card:     var(--surface-card);
  --color-surface-elevated: var(--surface-elevated);
  --color-surface-overlay:  var(--surface-overlay);

  --color-border-subtle:    var(--border-subtle);
  --color-border-strong:    var(--border-strong);

  --color-fg:               var(--text-primary);
  --color-fg-secondary:     var(--text-secondary);
  --color-fg-muted:         var(--text-muted);
  --color-fg-disabled:      var(--text-disabled);

  --color-brand:            var(--brand);
  --color-brand-hover:      var(--brand-hover);
  --color-brand-pressed:    var(--brand-pressed);
  --color-brand-soft:       var(--brand-soft);

  --color-success:          var(--success);
  --color-success-soft:     var(--success-soft);
  --color-warning:          var(--warning);
  --color-warning-soft:     var(--warning-soft);
  --color-danger:           var(--danger);
  --color-danger-soft:      var(--danger-soft);
  --color-info:             var(--info);
  --color-info-soft:        var(--info-soft);

  --color-chart-1:          var(--chart-1);
  --color-chart-2:          var(--chart-2);
  --color-chart-3:          var(--chart-3);
  --color-chart-4:          var(--chart-4);
  --color-chart-5:          var(--chart-5);
  --color-chart-6:          var(--chart-6);

  /* ---- Typography ---- */
  --font-sans:              var(--font-sans);
  --font-mono:              var(--font-mono);

  --text-xs:                var(--text-xs);
  --text-sm:                var(--text-sm);
  --text-base:              var(--text-base);
  --text-md:                var(--text-md);
  --text-lg:                var(--text-lg);
  --text-xl:                var(--text-xl);
  --text-2xl:               var(--text-2xl);
  --text-3xl:               var(--text-3xl);

  /* ---- Radii ---- */
  --radius-sm:              var(--radius-sm);
  --radius-md:              var(--radius-md);
  --radius-full:            var(--radius-full);

  /* ---- Spacing aliases (Tailwind already exposes 1..96; we add semantic names) ---- */
  --spacing-1:              var(--space-1);
  --spacing-2:              var(--space-2);
  --spacing-3:              var(--space-3);
  --spacing-4:              var(--space-4);
  --spacing-6:              var(--space-6);
  --spacing-8:              var(--space-8);
  --spacing-12:             var(--space-12);

  /* ---- Shadows ---- */
  --shadow-1:               var(--shadow-1);
  --shadow-2:               var(--shadow-2);
  --shadow-pop:             var(--shadow-pop);

  /* ---- Motion ---- */
  --animate-tp-fade-in:     fade-in var(--dur-2) var(--ease-out);
  --animate-tp-slide-up:    slide-up var(--dur-2) var(--ease-out);
  --animate-tp-pulse-soft:  pulse-soft 2.4s var(--ease-in-out) infinite;
  --animate-tp-flash:       value-flash 320ms var(--ease-out);
}

/* ------------------------------------------------------------------ */
/*  Keyframes used by --animate-*                                       */
/* ------------------------------------------------------------------ */
@keyframes fade-in {
  from { opacity: 0; }
  to   { opacity: 1; }
}
@keyframes slide-up {
  from { opacity: 0; transform: translateY(4px); }
  to   { opacity: 1; transform: translateY(0); }
}
/* Slow, low-amplitude opacity pulse for engaged + failsafe ONLY */
@keyframes pulse-soft {
  0%, 100% { opacity: 1; }
  50%      { opacity: 0.55; }
}
/* Brief color flash on big-number value change (Phase 5) */
@keyframes value-flash {
  0%   { color: var(--brand); }
  100% { color: var(--text-primary); }
}
```

### Resulting utilities (sample)

| Class                       | Resolves to                              |
| --------------------------- | ---------------------------------------- |
| `bg-surface-card`           | `var(--surface-card)`                    |
| `bg-surface-elevated`       | `var(--surface-elevated)`                |
| `text-fg` / `text-fg-muted` | primary / muted text                     |
| `border-subtle`             | `border-color: var(--border-subtle)`     |
| `text-brand`, `bg-brand`    | solid teal                               |
| `bg-brand-soft`             | 8% tint — selected rows                  |
| `text-success`, `bg-warning-soft`, `border-danger` | semantic       |
| `font-mono` / `font-sans`   | Geist Mono / Geist                       |
| `rounded-sm` (4) / `rounded-md` (6) / `rounded-full` (pill) | radii    |
| `animate-tp-fade-in` etc.   | tokenized keyframes                      |

### Brand gradient — controlled usage only

```tsx
// Use `var(--brand-gradient)` inline. Three places, total.
<div style={{ backgroundImage: "var(--brand-gradient)" }} />
```

Do **not** add a `bg-brand-gradient` utility — keeping it inline forces a conscious choice every time.

---

## 3. Quick reference — when to use what

| Need                                    | Token                          |
| --------------------------------------- | ------------------------------ |
| Page background                         | `surface-base`                 |
| Card on the page                        | `surface-card`                 |
| Hovered card / popover                  | `surface-elevated`             |
| Dialog, tooltip, command palette        | `surface-overlay`              |
| Normal text                             | `fg`                           |
| Label, caption, secondary line          | `fg-secondary`                 |
| Helper text, units, "/h"                | `fg-muted`                     |
| Borders between cards                   | `border-subtle`                |
| Borders that need to register (input)   | `border-strong`                |
| Primary CTA                             | `bg-brand` + `text-surface-base` |
| Active row / selected nav item          | `bg-brand-soft`                |
| Success state (telemetry healthy)       | `text-success`                 |
| Danger (failsafe, fault)                | `text-danger` + `border-danger`|
| Numeric values, IDs, timestamps         | `font-mono`                    |
| Big readouts (speed, RPM)               | `font-mono text-3xl`           |

---

## 4. Self-check applied

- 4 surface layers, not 6. ✓
- 2 border strengths. ✓
- One solid brand accent. ✓
- Brand gradient explicitly *not* a utility — avoids accidental scatter. ✓
- 3 radii. ✓
- 3 durations. ✓
- Shadows: minimal in dark, present-but-quiet in light. ✓
- Mono only for values. ✓
