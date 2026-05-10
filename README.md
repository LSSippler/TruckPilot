# TruckPilot

TruckPilot is an ETS2 autopilot/tooling project with a Rust runtime, map/graph export pipeline, telemetry SHM integration, and vJoy control output.

> **Status (2026-05-10)** — Phase 5 (Map-Parser) **CLOSED**. Phase 6.2
> (Autopilot State-Machine) in Vorbereitung. Plugin-Architektur 6.2-ready
> (5 Refactor-Fixes gemerged).
> Details: [`docs/vault/01-Phases/Phase-5-Closeout.md`].

## Map Coverage (Stand 2026-05-10)

- **Sector-Parse:** 270/282 Sektoren clean (95.7%). 12 BezierPatch-Failures
  akzeptiert (kein Routing-Hebel).
- **Routing-Erfolg:** 22% (20/90 base_map + 1/27 DLC). Ziel war 60/90 —
  der Rest ist strukturell durch Cross-Sector-Topologie blockiert.
- **Big-8-Cluster** (~205k Nodes, ~2000 km drivable): zusammenhängende
  Komponente Munich–Prag–Warschau–Amsterdam–Mailand–Sevilla–Sofia–Istanbul.
  Production-Bereich für Autopilot-Tests.
- **Singleton-Floor:** 53.9% aller Nodes haben degree=0. Architektonische
  Eigenschaft des trailing-node-Blocks, uniform über 124 Archive
  (incl. ProMods + alle DLCs). Kein Item-Type-Fix bricht diesen Floor.
- **Empfohlene Testrouten:** innerhalb Big-8 (Berlin→München,
  Hamburg→Hannover). Cross-Border Berlin→Madrid funktioniert nicht.

## Quickstart für zu Hause (Windows-PC mit ETS2)

Voraussetzungen:
- ETS2 läuft, ein Lkw ist im Cockpit.
- `truckpilot_telemetry.dll` ist im ETS2-Plugin-Ordner installiert
  (`scripts/install_dll.ps1` erledigt das).
- vJoy-Treiber ist installiert, Device 1 ist frei und in ETS2 als
  Eingabegerät aktiviert.
- `output/graph.json` liegt vor (Map-Export aus dem .NET- oder Rust-
  Pipeline-Lauf).

Einzeiler:

```powershell
.\scripts\start_live_loop.ps1
```

Das Skript prüft `output/graph.json`, extrahiert Start-/Ziel-UIDs via
`scripts/pick_route_uids.ps1`, prüft die SHM-Telemetrie und startet
`cargo run --release` mit den passenden Argumenten. Eine vollständige
Live-Test-Anleitung steht in [`docs/RUNBOOK_LIVETEST.md`](docs/RUNBOOK_LIVETEST.md).

## Schnellstart (Windows)

1. **Telemetry DLL build and install**
   - Open PowerShell in `TruckPilot.TelemetryDLL`
   - Run:
     ```powershell
     ./build.ps1 -Config Release
     ```
   - Copy `truckpilot_telemetry.dll` to:
     `C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2\bin\win_x64\plugins`

2. **Build Rust binaries**
   ```bash
   cargo build --release
   ```

3. **Optional: adjust config**
   - Edit `truckpilot.toml` in repository root.

4. **Prepare graph data**
   - Use existing `.NET` graph export or export a graph with your parser pipeline.
   - Expected file: `output/graph.json`

5. **Start autopilot**
   ```bash
   cargo run --release -- --graph-json output/graph.json --start <UID1> --goal <UID2> --vjoy-device 1 -v
   ```
   If your Cargo setup requires explicit binary selection, use:
   `cargo run --bin truckpilot --release -- ...`

## Troubleshooting

- **`Magic mismatch`**
  - DLL and Rust client use different SHM constants/layout.
  - Rebuild and redeploy `truckpilot_telemetry.dll` together with current repo version.

- **`Shared memory not found`**
  - ETS2 telemetry plugin was not loaded, or SHM name differs.
  - Verify plugin path and test with `cargo run --bin telemetry_diag -- --once`.

- **`vJoy device not available`**
  - vJoy driver/device not installed or currently acquired by another process.
  - Open Configure vJoy, ensure a free device exists.

- **`No route found`**
  - Start/goal UIDs missing from graph or not connected.
  - Validate graph consistency with `scripts/validate_graph.ps1`.

- **`clippy warnings`**
  - Run:
    ```bash
    cargo clippy --all-targets --all-features
    cargo clippy --fix --allow-dirty --allow-staged
    ```

## Architekturübersicht

```text
              +------------------------------+
              | ETS2 + telemetry DLL         |
              | (Local\TruckPilotTelemetry) |
              +--------------+---------------+
                             |
                             v
                   +-------------------+
                   | telemetry_diag    |
                   | shm_telemetry.rs  |
                   +---------+---------+
                             |
                             v
+-------------------+   +-------------------+   +-------------------+
| graph.json input  +-->| route planning    +-->| control outputs   |
| (Rust/.NET export)|   | A* + smoothing    |   | vJoy / console    |
+-------------------+   +-------------------+   +-------------------+
         |
         v
+-------------------+
| benchmark         |
| validate_graph    |
| vjoy_test         |
+-------------------+
```

## Verzeichnisstruktur

- `src/main.rs` — Haupt-CLI (map parsing, graph loading, route + loop start)
- `src/autopilot.rs` — A* routing logic
- `src/autopilot_loop.rs` — live control loop (telemetry + controllers)
- `src/controller.rs` — PID steering and speed controllers
- `src/shm_telemetry.rs` — shared-memory reader
- `src/vjoy.rs` — dynamic vJoy interface + fallback output
- `src/bin/` — offline helper tools (`telemetry_diag`, `vjoy_test`, `benchmark`)
- `scripts/` — external utility scripts (`validate_graph.ps1`)
- `TruckPilot.TelemetryDLL/` — C++ telemetry plugin project
- `docs/` — design and architecture documentation
- `output/` — generated runtime/export artifacts (`graph.json`, reports)

## Tools

- **`telemetry_diag`**
  - Reads SHM directly and prints key telemetry fields in 500 ms intervals.
  - Usage: `cargo run --bin telemetry_diag`

- **`vjoy_test`**
  - Runs steering/throttle/brake axis movement independently from the autopilot loop.
  - Usage: `cargo run --bin vjoy_test`

- **`benchmark`**
  - Loads a graph and calculates 100 sampled A* routes (distance mode) to report planning times.
  - Usage: `cargo run --bin benchmark -- output/graph.json`

- **`validate_graph.ps1`**
  - Validates graph consistency (references, duplicates, self-loops, etc.).
  - Usage (Windows PowerShell 7+):
    ```powershell
    .\scripts\validate_graph.ps1 output\graph.json
    ```

## Tests ausführen

Rust-Tests:
```bash
cargo test
```

PowerShell-Validator-Tests:
```bash
python3 -m pytest tests/test_validate_graph_ps1.py -v
```

## Graph parsing notes

- ETS2 base map is commonly located at:
  `C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2\base_map.scs`
- HashFS helper can extract sectors before feeding them into parser/export tools.

## Mod-Support

TruckPilot kann beliebige ETS2-Karten-Mods (`*.scs`) zusammen mit dem
Basisspiel laden. Sektor-Kollisionen werden über einen `load_order`-Wert
aufgelöst — höher = höhere Priorität. Connector-Patches kommen damit
automatisch über alle Karten-Mods.

Aktiviert wird der Mod-Support per CLI-Flag `--enable-mods`.

### ProMods

```bash
cargo run --release -- \
  --ets2-dir "/pfad/zu/Euro Truck Simulator 2" \
  --mod-dir  "$HOME/Documents/Euro Truck Simulator 2/mod" \
  --enable-mods --verbose
```

Der Mod-Ordner wird alphabetisch eingelesen; ProMods-Pakete (z. B.
`promods-map-v269.scs`, `promods-assets-v269.scs`) werden in dieser
Reihenfolge auf das Basisspiel gelegt.

### RusMap, Africa, RoExtended

Identisches Vorgehen — alle `.scs`-Dateien in `--mod-dir` werden geladen.
Wer eine eigene Reihenfolge braucht, übergibt zusätzlich
`--mod-order mod_order.json`.

### Eigene Kombinationen

Beispiel `mod_order.json`:

```json
{
  "base_game_files": ["base.scs", "base_map.scs", "def.scs"],
  "mod_descriptors": [
    {"name": "ProMods Map",     "file": "promods-map-v269.scs",     "order": 10},
    {"name": "ProMods Assets",  "file": "promods-assets-v269.scs",  "order": 11},
    {"name": "RusMap",          "file": "rusmap.scs",               "order": 20},
    {"name": "PM-RM Connector", "file": "promods_rusmap_connector.scs", "order": 999}
  ]
}
```

Höhere `order`-Werte gewinnen bei Sektor-Kollisionen.

### Connector-Patches

Connector-Patches (z. B. ProMods ↔ RusMap, ProMods ↔ Africa) brauchen die
*höchste* `order`. Sie überschreiben damit sowohl die einzelnen
Karten-Mods als auch das Basisspiel.

Details und API-Beispiele: [`docs/MOD_SUPPORT.md`](docs/MOD_SUPPORT.md).

## Weiterführende Dokumentation

- [`PROJECT_FINAL.md`](PROJECT_FINAL.md) — Feature-Übersicht, finale
  Metriken, Architektur, Limitierungen, Roadmap.
- [`HANDOFF.md`](HANDOFF.md) — Übergabe an neue Entwickler:
  Setup, Build, Tests, Modul-Index, Fallstricke, Erweiterungs-Anleitungen.
- [`offline_validation_results.txt`](offline_validation_results.txt) —
  Ergebnis der jüngsten Offline-Validierung (Tests, Clippy, Benchmark,
  Diagnose-Tools, Graph-Validator).
- [`docs/`](docs/) — Design-Dokumente (ACC, Lane-Graph, HashFS-Limit,
  Live-Test-Runbook, Feature-Roadmap, Mod-Support …).
