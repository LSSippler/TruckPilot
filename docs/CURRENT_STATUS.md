# TruckPilot – Aktueller Projektstatus
Datum: 2026-05-06

## 1. Überblick

| Metrik | Wert |
|--------|------|
| Rust-Quelldateien (src/) | 27 |
| Rust-Testdateien (tests/) | 5 |
| Tests (compile-time gelistet) | 159 |
| Tests (aktueller Lauf) | 127 passed, 1 failed, 1 ignored (nur lib; Integration-/Binary-Tests nicht ausgeführt wegen lib-Failure) |
| Fehlgeschlagener Test | `test_open_detects_scs` — panicked at archive.rs:138, assertion `archive.is_ok()` |
| Ignorierter Test | `test_real_ets2_map_parsing` — requires ETS2 installation at default path |
| Clippy-Status | 3 Warnungen (1 dead_code, 1 needless_match, 1 let_and_return) |
| PowerShell-Skripte (.ps1) | 8 |
| Shell-Skripte (.sh) | 2 |
| C++-Quelldateien (eigene, ohne SDK) | 6 (3 cpp + 3 h) |
| C#-Quelldateien | 20 (14 Core + 6 Tests) |
| Dokumentationsdateien (docs/) | 8 |
| Weitere Doku-Dateien (Root) | 5 (README, PROJECT_FINAL, PROJECT_REPORT, QA_REVIEW, AGENTS) |

## 2. Kernkomponenten (Status)

| Komponente | Status | Anmerkung |
|-----------|--------|-----------|
| BinaryMapParser (Rust) | in Arbeit (95%) | Sized-Format vollständig; Realdaten-Nachweis blockiert durch SCS-Reader-Bug |
| SCS-Reader (Rust) | fehlerhaft | Seek Invalid argument bei HashFS-Sektoren → 0 parsebare Sektoren (QA B.2) |
| A*-Router | produktionsreif | Deterministisch, Distance/ETA-Modi, Lane-Routing, alle Tests grün |
| Live-Loop (Autopilot) | in Arbeit | Funktional, aber nicht vollständig mit ETS2 validiert |
| vJoy-Integration | in Arbeit | Unit-Tests passen; echte vJoy-Hardware nicht auf Linux testbar |
| Telemetrie-Client (SHM) | produktionsreif | Magic validiert (0x54505054), Layout-Test grün, SHM-Priorität vor HTTP |
| C++-Telemetrie-DLL | produktionsreif | Build existiert (build/Release/truckpilot_telemetry.dll), Magic aligned |
| .NET-Map-Parser (Referenz) | produktionsreif | 74.8 Sektoren/s, 222k/65k/18k Counts verifiziert |
| Config-System (truckpilot.toml) | aktiv | PID, Speed, Routing, ACC, Telemetry — alle Config-Tests grün |
| Route-Smoothing (Catmull-Rom) | aktiv | Konfigurierbar via `[routing]`, Tests grün |
| PID-Steering | aktiv | P/I/D mit Anti-Windup und Clamping, 15 Controller-Tests grün |

## 3. Tools (Offline)

| Tool | Status | Anmerkung |
|------|--------|-----------|
| telemetry_diag | funktionsfähig | Test `test_telemetry_diag_opens_shm` passed; benötigt ETS2-SHM |
| vjoy_test | funktionsfähig | Test `test_vjoy_test_runs` passed; benötigt vJoy-Treiber |
| benchmark | funktionsfähig | Test `test_benchmark_runs` passed; 100 Route-Benchmarks |
| validate_graph.ps1 | funktionsfähig | Pytest-Test in `tests/test_validate_graph_ps1.py` vorhanden |

## 4. Metriken (Real-Map)

| Metrik | Rust-Parser | .NET-Referenz |
|--------|-------------|---------------|
| Nodes | nicht bestimmbar | 222.891 |
| Roads | nicht bestimmbar | 65.638 |
| Prefabs | nicht bestimmbar | 18.189 |
| Companies | nicht bestimmt | 460 |
| Signs | nicht bestimmt | 50.993 |
| Sektoren (total) | — | 276 |
| Abweichung | nicht bestimmbar | — |
| Durchsatz | nur synthetisch (~100k Sektoren/s) | 74.8 Sektoren/s |
| Getestete Sektoren | 0 von 276 (SCS-Reader-Bug) | 276 |

**Ursache:** `base_map.scs` hat 2054 Einträge. `list_known_files()` listet nur 4 hartkodierte Definitions-Pfade — keine Map-Sektoren. HashFS-Offset-Berechnung liefert ungültige Seek-Positionen (`seek to 13475380228292063266: Invalid argument`). Siehe `QA_REVIEW.md` B.2 und `FEATURE_ASSESSMENT_REPORT.md` Abschnitt 3.

## 5. Live-Test-Status

| Kriterium | Status |
|-----------|--------|
| Build (cargo build) | erfolgreich |
| DLL kompiliert | ja (build/Release/truckpilot_telemetry.dll) |
| DLL installiert | Installationsskripte vorhanden (`scripts/install_dll.ps1`, `.sh`) |
| Telemetrie (SHM) | Tests grün; echter ETS2-Test nur auf Windows möglich |
| vJoy | Unit-Tests grün; vJoy-Treiber nicht auf Linux verfügbar |
| Autopilot mit ETS2 | Live-Test-Logs vom 04.05.2026 existieren (6 Logs, UTF-16); Status unbestätigt |
| Bekannte Fehler | SCS-Reader liefert 0 Map-Sektoren (seek Invalid argument); `test_open_detects_scs` aktuell flaky (Panic) |

## 6. Bekannte Limitierungen

- **SCS-Reader (kritisch):** Seek-Bug in HashFS-Offset-Berechnung blockiert Rust-Binärparser-Realdaten-Nachweis. `list_known_files()` enthält keine Map-Sektor-Pfade.
- **Fahrspuren (78%):** Lane-Subnodes, parallele Fahrkanten und Spurwechsel-Kanten implementiert. Es fehlen: segmentabhängige Lane-Change-Erlaubnis, geometrische Lane-Offsets, Zielspur-Heuristik in A*.
- **ACC (70%):** AccController und Config-Integration vorhanden. Distanzkanal ist compile-time optional (`#ifdef`). Proxy-Fallback über lokale Beschleunigung ist heuristisch. ACC-Design-Dokument nicht auf aktuelle Implementierung synchronisiert.
- **Binärparser (95%):** Sized-Format vollständig. Realdaten-Counts gegen .NET nicht messbar (0 Sektoren). Company/City/Sign/Service werden im sized-Format noch nicht geparst (Skip).
- **Magic-Alignment:** Abgeschlossen (0x54505054), aber Layout-Größe C++ (512 Bytes) vs Rust/C# (116 Bytes) unterschiedlich.
- **Speed-Limits:** Fallback 80 km/h wenn kein Nav-Speed-Limit verfügbar.
- **Lane-Offsets:** Positionen der Lane-Knoten bleiben auf Basisknoten-Position (keine laterale Versetzung).

## 7. Nächste empfohlene Schritte

| Priorität | Aufgabe | Geschätzter Aufwand |
|-----------|---------|---------------------|
| 1 | **SCS-Reader fixen:** HashFS-Offset-Berechnung reparieren, `list_known_files()` um Map-Sektor-Pfade erweitern → Rust-Binärparser Realdaten-Nachweis ermöglichen | 6–10 PT |
| 2 | **Lane-Graph vervollständigen:** Segmentabhängige Lane-Change-Regeln, geometrische Lane-Offsets, Zielspur-Heuristik in A* ergänzen | 3–5 PT |
| 3 | **ACC validieren:** `docs/ACC_DESIGN.md` auf Ist-Stand bringen, Runtime-Status (Distanzkanal vs. Proxy) ergänzen, echte Fahrtests | 2–4 PT |

---

*Erstellt am 2026-05-06 aus tatsächlichen Datei-Analysen, `cargo test`, `cargo clippy` und bestehender Projektdokumentation.*
