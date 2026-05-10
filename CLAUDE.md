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


## Obsidian Vault — Dokumentations-Workflow

Lokaler Vault unter `docs/vault/` (nicht in Git, komplett privat). Claude Code liest und schreibt dort wie auf jeden anderen Ordner.

### Beim Session-Start automatisch lesen
Vor jeder neuen Aufgabe:
1. `docs/vault/00-Index.md` — aktueller Projekt-Stand
2. Die letzte Datei in `docs/vault/01-Phases/` (chronologisch per Filename) — was zuletzt passiert ist
3. Bei Bezug zu einem Modul: passende Datei aus `docs/vault/02-Architecture/`

### Beim Phasen-Abschluss automatisch schreiben
Nach jeder erfolgreich abgeschlossenen Phase ohne Nachfrage:
1. Neue Datei `docs/vault/01-Phases/Phase-X.Y-<Kurztitel>.md` mit Frontmatter + Ziel + Umsetzung + Vorher/Nachher-Tabelle + Lessons Learned
2. `docs/vault/00-Index.md` updaten — neuen Phase-Eintrag in chronologische Liste
3. Bei Architektur-Änderung: passende `docs/vault/02-Architecture/<Modul>.md` aktualisieren
4. Bei wichtiger Entscheidung: neuer ADR in `docs/vault/05-Decisions/ADR-NNN-<Kurztitel>.md`

### Konventionen
- Wikilinks `[[Phase-5.12-Sign-Handler]]` zu verwandten Phasen
- Frontmatter-Tags: `[phase, truckpilot]` für Phasen, `[architecture, <modul>]` für Module
- Tabellen für Vorher/Nachher-Metriken
- Phase-Filename: `Phase-X.Y-<Kurztitel-mit-Bindestrichen>.md`