# HANDOFF.md — TruckPilot

Stand: 2026-05-06

Dieses Dokument richtet sich an Entwickler, die TruckPilot übernehmen,
warten oder weiterentwickeln. Ziel ist ein Setup-zu-Build-zu-Run-Pfad in
unter einer Stunde, plus eine Karte aller wichtigen Module und Fallstricke.

---

## 1. Für wen ist das Projekt gedacht?

- **Primärzielgruppe**: Rust-/C#-Entwickler, die einen Autopiloten /
  ein Fahrerassistenzsystem für ETS2 weiterbauen möchten — etwa um
  Lane-Graphen, Ampelhandling oder Verkehrsfluss-Heuristiken zu ergänzen.
- **Sekundärzielgruppe**: Reverse-Engineering-Interessierte, die die
  ETS2-Map-Pipeline (HashFS, SCS-Sektoren, Item-Layout) studieren wollen.
- **Nicht-Zielgruppe**: Endanwender, die ohne Code-Kontakt einen fertigen
  Autopilot installieren wollen — TruckPilot ist ein Entwickler-Stack.

---

## 2. Was wird benötigt?

### Pflicht
| Komponente | Version | Zweck |
|---|---|---|
| Rust-Toolchain | 1.95+ (cargo, clippy, rustfmt) | Hauptcode + Bins |
| .NET SDK | 8.0 | Referenz-Map-Exporter |
| CMake | ≥ 3.20 | Telemetrie-DLL |
| MSVC 2022 (x64) | aktuell | Telemetrie-DLL kompilieren (Windows) |
| Python | 3.11+ | HashFS-Tooling, Tests |
| Git | aktuell | Repo-Pflege |

### Pflicht für Live-Test
| Komponente | Hinweis |
|---|---|
| Windows 10/11 (x64) | ETS2 läuft nativ nur auf Windows |
| ETS2 (Steam) | Aktuelle Version, base_map.scs muss zum Parser passen |
| vJoy-Treiber | Device 1 frei, in ETS2 als Eingabe ausgewählt |
| Telemetrie-DLL im Plugin-Ordner | siehe `scripts/install_dll.ps1` |

### Optional
- PowerShell 7 (auch auf Linux verfügbar) — für Validator- und Loop-Skripte.
- `cargo install cargo-watch` — für Auto-Rebuilds.
- `dotnet tool install -g dotnet-format` — Code-Formatierung im .NET-Teil.

---

## 3. Build

```bash
# 1) Rust (alle Plattformen)
cargo build --release
cargo build --release --bin benchmark
cargo build --release --bin telemetry_diag
cargo build --release --bin vjoy_test

# 2) .NET (Linux/Windows)
dotnet build TruckPilot.NET/TruckPilot.sln -c Release

# 3) Telemetrie-DLL (nur Windows)
cd TruckPilot.TelemetryDLL
./build.ps1 -Config Release
# erzeugt build/Release/truckpilot_telemetry.dll
# danach:
./scripts/install_dll.ps1
```

---

## 4. Tests

```bash
# Rust-Unit + Integrationstests
cargo test                       # 158 passed, 0 failed, 1 ignored

# Lints
cargo clippy --all-targets --all-features

# .NET-Tests
dotnet test TruckPilot.NET/TruckPilot.Tests/TruckPilot.Tests.csproj

# Python-Tests (PowerShell-Validator etc.)
python3 -m pytest tests/

# Graph-Validator
pwsh ./scripts/validate_graph.ps1 output/graph.json
```

Erwartete Ergebnisse: siehe `offline_validation_results.txt` im Repo-Root.

---

## 5. Architektur — Zusammenspiel der Komponenten

```
┌────────────────────────────┐
│  ETS2 (Spielprozess, Win)  │
│   ├── Telemetrie-DLL (C++) │ schreibt SHM-Frames
│   └── vJoy-Eingabe         │ liest virtuelle Achsen
└──────────────┬─────────────┘
               │
        Shared Memory (Local\TruckPilotTelemetry, Magic 0x54505054)
               │
┌──────────────▼─────────────┐
│  TruckPilot Rust Runtime   │
│                            │
│  shm_telemetry.rs          │ ── liest SHM
│      │                     │
│      ▼                     │
│  autopilot_loop.rs ◄── controller.rs (PID) + acc_controller.rs
│      │                     │
│      ▼                     │
│  vjoy.rs ─────────────────►│ schreibt vJoy-Achsen
│      ▲                     │
│      │                     │
│  autopilot.rs (A*) ◄── route_smoothing.rs
│      ▲                     │
│      │ Graph                │
│  graph_export.rs / graph_schema.rs
│      ▲                     │
│  pipeline.rs ◄── ets2_parser/ ◄── HashFS Reader
└────────────────────────────┘
```

Lese-Reihenfolge für Neueinsteiger:
1. `src/main.rs` — wie wird gestartet, welche CLI-Argumente gibt es?
2. `src/pipeline.rs` — Parse → Graph → Export.
3. `src/autopilot.rs` — A*-Routing.
4. `src/autopilot_loop.rs` — Live-Schleife.
5. `src/shm_telemetry.rs` und `src/vjoy.rs` — die zwei Brücken zur Außenwelt.

---

## 6. Wichtigste Dateien (Kurz-Index)

| Pfad | Inhalt |
|---|---|
| `src/main.rs` | CLI, Dispatch in Pipeline / Loop |
| `src/lib.rs` | Re-Exports und Modul-Hierarchie |
| `src/autopilot.rs` | A*-Algorithmus, Heuristik, Tiebreaks |
| `src/autopilot_loop.rs` | Telemetrie → Controller → vJoy-Loop |
| `src/controller.rs` | Lenk- und Speed-PID |
| `src/acc_controller.rs` | Adaptive Cruise Control |
| `src/route_smoothing.rs` | Pfad-Glättung |
| `src/shm_telemetry.rs` | SHM-Reader + Magic-Validierung |
| `src/telemetry.rs` | Telemetrie-Datentypen |
| `src/vjoy.rs` | Dynamic-Load von vJoyInterface.dll, Mock-Pfad |
| `src/pipeline.rs` | End-to-End Parser → Graph |
| `src/graph_export.rs` + `src/graph_schema.rs` | graph.json Schema & Writer |
| `src/compat_export.rs` | Kompatibilitäts-Export für Legacy-Tools |
| `src/json_export.rs` | gemeinsame JSON-Hilfen |
| `src/config.rs` | Laden von `truckpilot.toml` |
| `src/ets2_parser/scs_reader.rs` | SCS-Archiv-Reader |
| `src/bin/benchmark.rs` | A*-Benchmark |
| `src/bin/telemetry_diag.rs` | SHM-Diagnose |
| `src/bin/vjoy_test.rs` | vJoy-Selbsttest |
| `tests/integration_tests.rs` | End-to-End Pipeline-Tests |
| `tests/determinism_test.rs` | Reproduzierbarkeit von Graph + Routing |
| `tests/graph_json_test.rs` | CLI-Verhalten mit `--graph-json` |
| `tests/real_map_test.rs` | Smoke-Test gegen echte base_map.scs |
| `TruckPilot.NET/TruckPilot.Core/` | .NET-Referenz-Map-Parser |
| `TruckPilot.TelemetryDLL/src/` | C++-ETS2-Plugin |
| `truckpilot.toml` | Laufzeit-Konfiguration |
| `scripts/start_live_loop.ps1` | Quickstart-Skript |
| `scripts/validate_graph.ps1` | Graph-Konsistenzprüfung |

---

## 7. Bekannte Fallstricke

### 7.1 SCS-Reader Magic / Header-Salt
- `src/ets2_parser/scs_reader.rs` liest den SCS-Archiv-Header. Das `salt`-
  Feld wird nicht aktiv genutzt, aber **nicht entfernen** — es ist Teil des
  binären Layouts und dient der späteren Verifikation. Clippy meldet das
  als `dead_code`-Warnung; ignorieren.
- Bei einem ETS2-Patch kann sich die Sektorversion ändern. Der Parser
  prüft die Version und wirft einen verständlichen Fehler — das ist
  Absicht, kein Bug.

### 7.2 vJoy-DLL-Suche
- `src/vjoy.rs` lädt `vJoyInterface.dll` zur Laufzeit (LoadLibrary).
  Reihenfolge:
  1. Pfad aus `truckpilot.toml` (`[vjoy] dll_path`) — wenn gesetzt.
  2. `PATH`-Umgebung.
  3. Standard-vJoy-Installation (`%ProgramFiles%\vJoy\x64`).
- Wenn keine DLL gefunden wird **und** das Target nicht Windows ist, wird
  automatisch der Mock-Pfad aktiviert (siehe `vjoy_test`-Output „Kein
  Windows/vJoy erkannt. Test läuft im Mock-Modus.“). Das ist erwünscht
  für Linux-CI, **aber leicht zu verwechseln**: ein Live-Test, der keine
  Achsen bewegt, weist meist auf einen versehentlichen Mock-Modus auf
  Windows hin (z. B. weil die vJoy-DLL nicht im Suchpfad steht).
- Device 1 muss in der vJoy-Konfiguration aktiviert und nicht von einem
  anderen Prozess akquiriert sein.

### 7.3 SHM-Name & Layout
- Name: `Local\TruckPilotTelemetry` (Windows) bzw. `/dev/shm/truckpilot_telemetry`
  (Linux-Diagnose-Mock).
- Magic: `0x54505054` ("TPPT") als erste 4 Bytes.
- **Layout-Falle**: Die C++-Struct ist mit Padding 512 Bytes groß, der
  Rust-/C#-View liest die ersten 116 Bytes (gemeinsames Prefix). Wenn neue
  Felder ergänzt werden, **immer in beiden Sprachen synchron** und
  ausschließlich am Ende des Prefix anhängen. Magic-Wert + Versions-Byte
  am Anfang sind dabei zwingend zu erhalten.
- Bei `Magic mismatch` immer DLL **und** Rust-Binary gemeinsam neu deployen.

### 7.4 graph.json
- UIDs müssen Hex-Strings im Format `"0x...."` sein. Der Validator
  `scripts/validate_graph.ps1` warnt bei numerischen UIDs (Check 4).
- Synthetische Test-Dateien (z. B. die im Repo vorliegende
  `output/graph.json` mit 2 Nodes) verwenden Integer-UIDs — das ist
  bewusst, da diese Dateien nur für Build-Smoke-Tests gedacht sind.
- Die Reihenfolge der Edges/Nodes muss deterministisch bleiben — siehe
  `tests/determinism_test.rs`. Beim Anpassen des Exports nicht die
  Sort-Keys ändern.

### 7.5 Pfade auf Linux vs. Windows
- Im Rust-Code immer `Path` / `PathBuf` verwenden, nie String-Konkatenation
  mit `\`.
- PowerShell-Skripte mit Backslash-Pfaden (`output\graph.json`) sind
  Windows-zentriert. PowerShell 7 auf Linux toleriert beides — aber der
  ETS2-Plugin-Pfad existiert nur auf Windows.

### 7.6 Cargo Build im Linux-Snap
- Der Linux-Server in dieser Repo-Umgebung hat `cargo` unter
  `~/.cargo/bin/cargo`. Wenn Skripte fehlschlagen mit „cargo not found“,
  `PATH` ergänzen: `export PATH="$HOME/.cargo/bin:$PATH"`.

---

## 8. Wie erweitert man das Projekt?

### Neuen Item-Typ im Map-Parser ergänzen
1. .NET-Referenz: `TruckPilot.NET/TruckPilot.Core/Items/<NeuesItem>.cs`
   anlegen (Layout 1:1 aus SCS-Reverse-Engineering oder
   ETS2-SDK-Sektor-Spec).
2. Rust-Parität: `src/ets2_parser/items/<neues_item>.rs` analog.
3. Im Item-Dispatcher (`src/ets2_parser/sector_parser.rs`) die Type-ID
   registrieren.
4. Test in `tests/integration_tests.rs` ergänzen, der das neue Item in
   einer Mini-Sektor-Datei erwartet.
5. `cargo test` und `dotnet test` müssen grün bleiben — die
   `determinism_test`-Suite stellt sicher, dass das Edge-Sortier-Verhalten
   stabil bleibt.

### Neuen Telemetrie-Kanal registrieren
1. Feld im C++-Plugin (`TruckPilot.TelemetryDLL/src/plugin.cpp`) am Ende
   des SHM-Prefix-Layouts hinzufügen, in `register_for_channel(...)`
   abonnieren und in den Frame schreiben.
2. Rust-View (`src/telemetry.rs` + `src/shm_telemetry.rs`) um dasselbe
   Feld in derselben Reihenfolge erweitern. Layout-Größe in
   `shm_telemetry.rs` aktualisieren.
3. Optional Konsumenten anpassen (`src/autopilot_loop.rs`,
   `src/bin/telemetry_diag.rs`).
4. Magic-Wert behalten, **Versions-Byte erhöhen**, und prüfen dass alte
   DLL gegen neuen Reader sauber „Magic mismatch / Version mismatch“
   meldet.

### Neue Controller-Strategie
1. Neuen Modul in `src/` anlegen (z. B. `src/lane_keep.rs`).
2. Schnittstelle: `pub fn step(&mut self, telem: &Telemetry, plan: &RoutePlan) -> ControlOutput`.
3. In `autopilot_loop.rs` zwischen Routenfolge und vJoy-Output einklinken.
4. Konfiguration über `truckpilot.toml` (siehe `src/config.rs`).
5. Tests in `tests/` ergänzen — möglichst deterministisch, ohne SHM-/vJoy-Hardware.

### Neuen Routing-Modus
1. Erweitere die Cost-Funktion in `src/autopilot.rs`
   (z. B. `RoutingMode::FuelOptimal`).
2. Tests in `tests/determinism_test.rs` und ein Mini-Benchmark in
   `src/bin/benchmark.rs` (Modus als CLI-Flag).

---

## 9. Pflege-Checkliste vor jedem Release

- [ ] `cargo test` grün
- [ ] `cargo clippy --all-targets --all-features` ohne neue Warnungen
- [ ] `cargo run --bin benchmark -- output/graph.json` läuft
- [ ] `dotnet test` grün
- [ ] `pwsh ./scripts/validate_graph.ps1 output/graph.json` 0 Verletzungen
      (mit echter graph.json — synthetische Datei nur für Build-Smoke)
- [ ] `offline_validation_results.txt` aktualisieren
- [ ] `PROJECT_FINAL.md` Metriken-Tabelle aktualisieren
- [ ] Telemetrie-DLL und Rust-Binary gemeinsam neu releasen, falls SHM-
      Layout geändert wurde

---

## 10. Kontakt / weiterführende Dokumente

- `PROJECT_FINAL.md` — Feature- und Metrik-Übersicht.
- `README.md` — Quickstart.
- `docs/RUNBOOK_LIVETEST.md` — detaillierter Live-Test-Ablauf.
- `docs/ACC_DESIGN.md` — Adaptive Cruise Control.
- `docs/LANE_GRAPH_DESIGN.md` — geplante Lane-Graph-Architektur.
- `docs/HASHFS_HASH_LIMITATION.md` — bekannte Grenzen des HashFS-Readers.
- `docs/FEATURE_ROADMAP.md` — geplante Features.
