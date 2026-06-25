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
- engage-diagnostician   — Engage/Lane-Keeper/Blackboard-Diagnose (kein Code, nur Befund + Fix-Plan)

## Build & Deploy

| Befehl | Wann |
|---|---|
| `cargo build-release` | **Standard** — baut Workspace + deployed alle Plugin-DLLs nach `plugins/` automatisch |
| `cargo xtask copy-plugins` | Manuell nachträglich deployen (Release-DLLs aus `target/release/`) |
| `cargo xtask copy-plugins --debug` | Debug-DLLs deployen — **Pflicht vor `cargo run -p truckpilot-core -- daemon`** |
| `cargo deploy-ets2 [DIR]` | `truckpilot_telemetry.dll` nach ETS2 deployen (manuell, benötigt `ETS2_PLUGINS_DIR` env oder DIR-Arg) |
| `.\scripts\ship.ps1` | Fallback-PowerShell-Wrapper, identisch zu `cargo build-release` |

Hinweis bei laufendem Daemon: DLL-Copy schlaegt hart fehl (locked). `cargo build-release` gibt dann ERROR mit Exit-Code 2. Daemon stoppen und `cargo xtask copy-plugins` nachfahren.

## Phase-Status (Map-Parser)

| Phase | Status | Ergebnis |
|---|---|---|
| 6.2b | CLOSED | Routing functional. BothUnresolved 1 878 → 9 (-99.5%). Roads 366 012, Nodes 1 153 648. Berlin-Sample 227 Waypoints / 151 ms. |
| 6.4 | offen | ProMods Support. Deferred Known-Issues: promods-bezier-desync, sign-handler-promods-east, remaining-sector-aborts. |

Smoke-Validation: `.\target\release\truckpilot-route-test.exe --all-pairs` — muss Berlin und andere cities.toml-Paare routen.

Deferred Known-Issues: `docs/known-issues/` — drei Dateien mit Kontext für Phase 6.4.


## Outputs-Ordner — Dateibasierte Organisation

Alle generierten Dateien (Specs, Logs, Reports, Dumps) landen in `outputs/YYYY-MM-DD/` — flach nach Datum sortiert, kein Unterordner-Chaos mehr.

- JEDE Ausgabedatei direkt in den Ordner des ERSTELLDATUMS legen
- Niemals `outputs/claude/`, `outputs/deepseek/` o.ä. verwenden
- Niemals Dateien direkt in `outputs/` root ablegen
- Immer den Pfad als `outputs/YYYY-MM-DD/filename.ext` angeben
- Datum aus `date +%Y-%m-%d` (Linux) oder `Get-Date -Format 'yyyy-MM-dd'` (PowerShell)

Prompt-Snippet zum Anhängen:
```
OUTPUT: Alle generierten Dateien nach outputs/YYYY-MM-DD/ schreiben (flach nach Datum, keine Unterordner wie claude/deepseek). Datum = heute (YYYY-MM-DD).
```

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

## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

Rules:
- For codebase questions, first run `graphify query "<question>"` when graphify-out/graph.json exists. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).
