# TruckPilot — Claude-Code-Konfiguration

## Architektur
- Workspace mit Crates für Core, Plugins, Map-Parser, IPC, Telemetrie, UI
- UI: Tauri 2 + React + shadcn/ui in crates/ui/
- IPC: WebSocket localhost:8765, JSON-Protokoll versioniert in crates/ipc-protocol

## Verbindliche Regeln
- Niemals IPC-Typen nur frontend- oder nur backend-seitig ändern. Immer beide.
- Niemals raw fetch oder raw WebSocket im Frontend. Immer über src/lib/ipc.ts.
- Niemals shadcn-Komponenten von Hand kopieren. Immer `npx shadcn@latest add`.
- Bei jedem Phasen-Abschluss: `/check-ui` ausführen, erst bei Grün committen.

## Slash-Commands
- /add-shadcn <name>     — shadcn-Komponente sicher hinzufügen
- /sync-ipc-types        — IPC-Typen Rust↔TS synchronisieren
- /check-ui              — Lint, Typecheck, Test, Clippy, Fmt
- /new-tab <Name>        — neue Tab-Route anlegen

## Sub-Agents
- ipc-protocol-guard     — IPC-Synchronität prüfen
- shadcn-styler          — UI-Polish-Reviews
