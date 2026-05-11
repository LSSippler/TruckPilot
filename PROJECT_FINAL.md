# PROJECT_FINAL.md — TruckPilot

Stand: 2026-05-06 — Reifegrad: **100 % (Offline-Validierung grün)**

TruckPilot ist ein Autopilot- und Werkzeug-Stack für Euro Truck Simulator 2
(ETS2). Er liest die ETS2-Karte, baut einen Routing-Graphen, plant Routen
mit A*, nimmt Live-Telemetrie aus dem Spiel via Shared-Memory entgegen und
steuert Lkw-Achsen über vJoy.

---

## 1. Feature-Überblick

### Map-Pipeline
- **HashFS-Reader** (`ets2_hashfs/`, `src/ets2_parser/`): liest ETS2-`.scs`-Archive
  (CityHash64-basiertes virtuelles Dateisystem) und extrahiert Sektoren.
- **.NET-Map-Parser** (`TruckPilot.NET/TruckPilot.Core`): parst Binär-Sektoren
  (Items, Nodes, Roads, Prefabs, Companies, Cities, Signs, Ferries, Fuel Pumps).
- **Rust-Map-Parser** (`src/ets2_parser/`): paritätischer Parser; deckungsgleiche
  Ergebnisse mit der .NET-Implementierung (0 % Abweichung).
- **Graph-Export** (`src/graph_export.rs`, `src/graph_schema.rs`,
  `src/compat_export.rs`): schreibt deterministischen `output/graph.json` mit
  Schema-Version, sortierten Nodes/Edges und stabilen UIDs.

### Routing
- **A\*-Router** (`src/autopilot.rs`): Distanz- und Zeitmodus, Heuristik mit
  euklidischem Lower Bound, deterministische Tiebreaks.
- **Routen-Smoothing** (`src/route_smoothing.rs`): Glättung der A\*-Pfade
  vor der Übergabe an den Steuer-Loop.
- **Lane-Graph-Bausteine** (Edges mit `lane_count`, `direction`, Lane-Change-
  UIDs) — Grundlage für die geplante Spur-Auswahl.

### Telemetrie & Bridge
- **C++-Telemetry-DLL** (`TruckPilot.TelemetryDLL/`): ETS2-Plugin (SCS-SDK),
  schreibt Telemetrie-Frames in eine Shared-Memory-Region.
- **Rust-SHM-Reader** (`src/shm_telemetry.rs`): liest dieselbe Region,
  validiert Magic (`0x54505054`) und reicht Frames an den Loop weiter.
- **Diagnose-Tool** (`src/bin/telemetry_diag.rs`): Einmal- oder Endlosmodus,
  signalisiert fehlende SHM ohne Crash.

### Steuerung & Aktorik
- **PID-Controller** (`src/controller.rs`): Lenk- und Speed-Regler.
- **Adaptive Cruise Control** (`src/acc_controller.rs`): Abstandsregelung.
- **vJoy-Wrapper** (`src/vjoy.rs`): dynamisches Laden von `vJoyInterface.dll`
  zur Laufzeit, automatischer Mock-Fallback auf Nicht-Windows-Plattformen.
- **Autopilot-Loop** (`src/autopilot_loop.rs`): bindet Telemetrie, Controller
  und vJoy-Ausgabe zusammen.

### Werkzeuge & Skripte
- `cargo run --bin benchmark` — 100 A\*-Routen, Min/Max/Avg-Planungszeit.
- `cargo run --bin telemetry_diag` — Live-Telemetrie-Anzeige.
- `cargo run --bin vjoy_test` — Achs-Selbsttest (real auf Windows, Mock sonst).
- `scripts/validate_graph.ps1` — strukturelle Graph-Prüfungen
  (Refs, Duplikate, Self-Loops, Hex-UIDs, isolierte Nodes).
- `scripts/start_live_loop.ps1` — Quickstart für den Live-Test (siehe §6).
- `scripts/install_dll.ps1` — kopiert die Telemetrie-DLL ins ETS2-Plugin-
  Verzeichnis.

---

## 2. Finale Metriken

| Metrik | Wert |
|---|---|
| Map-Parser-Parität (Rust ↔ .NET) | **0 % Abweichung** |
| Map-Export-Durchsatz | **74.8 Sektoren/s** |
| Nodes / Roads / Prefabs (Realmap) | **222 891 / 65 638 / 18 189** |
| Companies / Signs / Sektoren | 460 / 50 993 / 276 |
| Routenplanung (Mini-Graph, Avg) | 0.0052 ms |
| Rust-Tests | **158 passed, 0 failed, 1 ignored** |
| Clippy | 0 Fehler, 4 Hygiene-Warnungen |
| Reifegrad | **100 %** |

> **Hinweis**: Die historische Kennzahl „139 Tests grün“ stammt aus einem
> früheren Stand. Der aktuelle Lauf liefert 158 grüne Tests + 1 ignorierten
> Test (Detail-Aufschlüsselung in `offline_validation_results.txt`).

---

## 3. Architektur-Übersicht

```text
                +------------------------------+
                | ETS2 + telemetry DLL         |
                | (Local\TruckPilotTelemetry)  |
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

Datenfluss kompakt:

```
HashFS Reader -> .NET / Rust Map Parser -> Graph Builder -> graph.json
                                                                 |
                                                                 v
ETS2 -> Telemetry DLL -> SHM -> shm_telemetry.rs -> autopilot_loop -> vJoy -> ETS2
                                                          ^
                                                          |
                                                    A* Router (autopilot.rs)
```

---

## 4. Verzeichnisstruktur

```text
TruckPilot/
├── src/
│   ├── main.rs                 # CLI-Entry, Pipeline-Orchestrierung
│   ├── lib.rs                  # Modul-Wurzel
│   ├── autopilot.rs            # A*-Routing
│   ├── autopilot_loop.rs       # Live-Loop (Telemetrie + Controller + vJoy)
│   ├── controller.rs           # PID Lenkung / Speed
│   ├── acc_controller.rs       # Adaptive Cruise Control
│   ├── route_smoothing.rs      # Pfadglättung
│   ├── shm_telemetry.rs        # Shared-Memory-Reader
│   ├── telemetry.rs            # Telemetrie-Datentypen
│   ├── vjoy.rs                 # vJoy-Bridge (dynamic load + Mock)
│   ├── pipeline.rs             # End-to-End-Pipeline (Parse -> Graph -> Export)
│   ├── graph_export.rs         # graph.json schreiben
│   ├── graph_schema.rs         # Graph-Schema (Node/Edge)
│   ├── compat_export.rs        # Kompatibilitäts-/Legacy-Export
│   ├── json_export.rs          # JSON-Hilfen
│   ├── config.rs               # truckpilot.toml laden
│   ├── ets2_parser/            # Rust-Parser für SCS-Archive + Sektoren
│   └── bin/
│       ├── benchmark.rs
│       ├── telemetry_diag.rs
│       └── vjoy_test.rs
├── tests/                      # Integrationstests (config, determinism,
│                               # graph_json, pipeline, real_map)
├── crates/telemetry_dll/       # interner Rust-Crate (Telemetry-Helfer)
├── TruckPilot.NET/             # .NET-Referenz-Implementierung (CLI/Core/Tests)
├── TruckPilot.TelemetryDLL/    # C++-ETS2-Plugin (SCS-SDK)
├── ets2_hashfs/                # Python-Hashfs-Tooling
├── scripts/                    # PowerShell + Bash Hilfsskripte
├── docs/                       # Design-Dokumente (ACC, Lane-Graph, Runbook ...)
├── output/                     # graph.json + Reports
├── truckpilot.toml             # Laufzeit-Konfiguration
├── Cargo.toml / Cargo.lock
├── README.md
├── PROJECT_FINAL.md            # (diese Datei)
├── HANDOFF.md                  # Übergabe-Dokument
└── offline_validation_results.txt
```

---

## 5. Build- und Test-Anleitung

### Rust (Linux & Windows)
```bash
cargo build --release
cargo test
cargo clippy --all-targets --all-features
```

### .NET (Windows / Linux mit .NET 8 SDK)
```bash
dotnet build TruckPilot.NET/TruckPilot.sln
dotnet test  TruckPilot.NET/TruckPilot.Tests/TruckPilot.Tests.csproj
```

### C++ Telemetry-DLL (Windows, MSVC 2022 x64, /MT)
```powershell
cd TruckPilot.TelemetryDLL
./build.ps1 -Config Release
```
oder direkt mit CMake:
```bash
cmake -S TruckPilot.TelemetryDLL -B TruckPilot.TelemetryDLL/build -A x64
cmake --build TruckPilot.TelemetryDLL/build --config Release
```

### Python-Tools
```bash
python3 -m pip install -e .
python3 -m pytest tests/
```

---

## 6. Live-Test-Anleitung

Voraussetzungen am Windows-PC:
1. ETS2 installiert und gestartet, Lkw im Cockpit.
2. `truckpilot_telemetry.dll` im ETS2-Plugin-Verzeichnis
   (`scripts/install_dll.ps1` erledigt das).
3. vJoy-Treiber installiert, Device 1 frei und in ETS2 als Eingabegerät
   ausgewählt.
4. `output/graph.json` vorhanden (Rust- oder .NET-Map-Export).

**Quickstart (Einzeiler):**

```powershell
.\scripts\start_live_loop.ps1
```

Das Skript prüft `graph.json`, extrahiert Start-/Ziel-UIDs per
`scripts/pick_route_uids.ps1`, prüft die SHM-Telemetrie und startet
`cargo run --release` mit `--graph-json`, `--start`, `--goal`,
`--vjoy-device 1`, `-v`.

Manuell ohne Skript:
```powershell
cargo run --release -- --graph-json output/graph.json `
    --start <UID1> --goal <UID2> --vjoy-device 1 -v
```

---

## 7. Bekannte Limitierungen

- **Layout-Größen-Differenz** zwischen C++-Header (512 Bytes) und
  Rust/C#-Sicht (116 Bytes) im SHM. Magic-Konstante (`0x54505054`) ist
  abgeglichen, das gemeinsame Prefix wird konsistent gelesen — neue Felder
  müssen in allen drei Sprachen synchron ergänzt werden.
- **Lane-Counts** werden derzeit aus Traffic-Flags hergeleitet, nicht aus
  echten Spurdefinitionen.
- **Speed-Limits** fallen auf 80 km/h zurück, wenn keine Limit-Information
  am Edge vorliegt.
- **Ampeln und Vorfahrt** sind im Routing-Graph noch nicht modelliert
  (Roadmap: Sign- + TrafficRules-Integration).
- **Live-Test nur auf Windows** validierbar — die Linux-Builds arbeiten
  für vJoy und SHM im Mock-Modus.

---

## 8. Roadmap (Post-1.0)

1. Fahrspurerkennung (Lane-Graph, Lane-Change-Logik).
2. Ampeln & Vorfahrt (Signs + Traffic-Rules → Routen-Heuristik).
3. ACC-Feintuning mit echten Verkehrsfahrzeugen.
4. Dynamische Stauumfahrung.
5. Cross-Plattform-Telemetrie (Linux-ETS2-Bridge via Wine-Plugin).

Siehe auch `docs/FEATURE_ROADMAP.md`.
