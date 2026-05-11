# truckpilot-ui

Tauri 2 + React 19 + shadcn/ui desktop app for the TruckPilot self-driving daemon.

## Architecture

- **`src-tauri/`** — Rust Tauri shell. Owns the WebSocket connection to
  `truckpilot-core` (`ws://127.0.0.1:8765`), forwards `core-event` to the front-end,
  exposes `invoke` commands for outgoing `UiCommand` payloads.
- **`src/`** — React 19 SPA, react-router-dom for the six tabs, Zustand for state,
  shadcn/ui (manually pinned to project) for primitives, recharts for plots,
  react-window for log virtualization.
- **`scripts/sync-ipc-types.mjs`** — checks that every `CoreMessage`/`UiCommand`
  variant defined in `crates/ipc-protocol/src/lib.rs` has a matching TypeScript
  discriminated-union arm in `src/lib/types.ts`. Run after editing the Rust file.

The IPC layer is **never** hit directly from React. Instead:

1. The frontend calls `sendCommand(...)` (`src/lib/ipc.ts`) which invokes the Rust
   command `send_command`.
2. The Rust command pushes the typed payload onto an `mpsc` channel into the
   `IpcBridge` task (`src-tauri/src/ipc_bridge.rs`).
3. `IpcBridge` has a single WebSocket connection with reconnect-loop
   (exponential backoff 500 ms → 30 s, ±20 % jitter). All inbound `CoreMessage`
   payloads are emitted as `core-event`. Connection state changes are emitted
   as `connection-status`.

This guarantees one connection regardless of how many windows are open and
keeps the reconnect/backoff logic in a single place.

## Dev setup

```bash
# from repo root
cd crates/ui
npm install
npm run tauri dev      # starts Vite + Tauri (auto-launches the Rust shell)
```

In a second terminal, run the core daemon:

```bash
cargo run -p truckpilot-core -- daemon
```

The status bar should turn green and read `Connected • core v1.0`.

## Available scripts

| Script               | What it does                                                  |
|----------------------|---------------------------------------------------------------|
| `npm run dev`        | Vite dev server (without Tauri)                               |
| `npm run tauri dev`  | Vite + Tauri shell                                            |
| `npm run build`      | Production Vite bundle (`dist/`)                              |
| `npm run tauri build`| Full Tauri release binary (MSI on Windows, DEB on Linux, …)   |
| `npm run typecheck`  | `tsc --noEmit`                                                |
| `npm run lint`       | ESLint (`--max-warnings 0`)                                   |
| `npm run test`       | Vitest run                                                    |
| `npm run sync-types` | IPC protocol diff (Rust ↔ TypeScript)                         |
| `npm run check-ui`   | Typecheck + lint + tests                                      |

The `/check-ui` slash command runs the same chain plus `cargo clippy` and
`cargo fmt --check` on `src-tauri/`.

## Routes

| Path                  | Tab            | Status |
|-----------------------|----------------|--------|
| `/`                   | Dashboard      | Live telemetry + active plugins. |
| `/plugins`            | Plugins        | Toggle, reload, drag-reorder; clicking a row opens the JSON-Schema settings form. |
| `/mods`               | Mods           | Multi-select, build-progress stepper, cache control (mod manager service is stubbed in core). |
| `/pid`                | PID Tuning     | Three profiles, sliders, recharts live plot of `pid_sample` stream. |
| `/settings`           | Settings       | Theme, autoconnect, hotkeys, ETS2 path with auto-detect via Steam `libraryfolders.vdf`. |
| `/logs`               | Logs           | Virtualized list, level/plugin filters, search, export. |
| `/external-dashboard` | second window  | Read-only fullscreen dashboard for a 2nd monitor. Open via topbar button or **F2**. |

## Adding shadcn components

```bash
npx shadcn@latest add <component-name>
```

The project ships with a hand-written subset of primitives in
`src/components/ui/` (button, card, badge, input, label, switch, select, slider,
tabs, separator, scroll-area, tooltip, dialog, alert-dialog, progress) so the
initial bootstrap works offline. Anything new should go through the CLI to keep
the upgrade story sane.

## Troubleshooting

- **Status bar stuck on `Reconnecting…`** — the core daemon isn't listening.
  Check `cargo run -p truckpilot-core -- daemon` is up; the bridge polls every
  500 ms initially, escalating to 30 s.
- **Auto-detect fails to find ETS2** — the parser only knows about Steam
  installs (`appmanifest_227300.acf`). Non-Steam copies need the path picked
  manually.
- **`npm run sync-types` reports a mismatch** — edit `src/lib/types.ts` to mirror
  the Rust enum. The script is intentionally read-only because protocol drift
  is a review-worthy event, not something to silently auto-write.
- **Second window opens blank** — the `dashboard-window` capability needs
  permissions in `src-tauri/capabilities/default.json` (already configured for
  this label).
