# ETS2 Route Distance Verification (Phase 5k / 5l)

Tooling zur objektiven Verifikation von `RouteWaypoint.distance` (`item+0x14` in der Telemetry-DLL) über echte ETS2-Fahrten. **Distance bleibt `ROUTE_WP_FLAG_UNTRUSTED`** — diese Tools sammeln und werten nur Daten; sie ändern kein Routing, keinen Lane-Keeper und keine Fahrlogik.

## Voraussetzungen

- ETS2 läuft mit `truckpilot_telemetry.dll` (RouteBlackboard wird befüllt), **oder**
- `route-shm-sim` für synthetische Tests
- TruckPilot Daemon optional (Recorder liest SHM direkt)

## 1. Aufnahme (Recorder)

```bash
cargo run -p truckpilot-telemetry --bin route-distance-recorder -- \
  --out logs/route_distance.csv \
  --hz 2 \
  --duration-sec 300
```

### CLI

| Option | Beschreibung |
|--------|--------------|
| `--out <path>` | **Pflicht.** CSV-Ausgabedatei |
| `--hz <float>` | Samplingrate (Default: 2 Hz) |
| `--duration-sec <n>` | Stoppt nach n Sekunden (sonst bis Ctrl+C) |
| `--append` | An bestehende CSV anhängen (Header nur wenn Datei neu) |
| `--wait` | Wartet bis SHM verfügbar (Default) |
| `--no-wait` | Beendet sofort wenn SHM fehlt |
| `--include-waypoints` | Zusätzliche JSONL-Datei mit Waypoint-Details |
| `--jsonl <path>` | Expliziter Pfad für Waypoint-JSONL |

Ohne `--include-waypoints` / `--jsonl` bleibt die CSV kompakt (eine Zeile pro Sample).

### Echte Testfahrt

1. Route in ETS2 planen und Job annehmen (oder bestehende Route laden).
2. Recorder starten **bevor** oder **während** du fährst (`--wait` wartet auf DLL/SHM).
3. Mindestens **2–5 Minuten** fahren, idealerweise mit sichtbarer Restdistanz-Änderung.
4. Nicht pausieren/laden — Route-Hash-Wechsel ohne Neustart verfälschen die Auswertung.
5. Recorder mit Ctrl+C oder `--duration-sec` beenden.

## 2. Auswertung (Report)

```bash
cargo run -p truckpilot-telemetry --bin route-distance-report -- logs/route_distance.csv
cargo run -p truckpilot-telemetry --bin route-distance-report -- logs/route_distance.csv --json report.json
```

Exit-Code **2** bei Verdict `suspicious` (für Skripte/CI).

## 3. Mehrere Testfahrten (Meta-Report, Phase 5l)

Nimm **pro Szenario eine eigene CSV** auf (eindeutiger Dateiname, z. B. `logs/autobahn_gerade.csv`). Empfohlene Testmatrix:

| # | Szenario | Ziel |
|---|----------|------|
| 1 | Autobahn gerade | Monotone Restdistanz, hohe Geschwindigkeit |
| 2 | Stadt mit Kreuzungen | Stop-and-go, kurze Segmente |
| 3 | Autobahnabfahrt / -auffahrt | Geschwindigkeitswechsel, Kurven |
| 4 | Route mit Pause / Stillstand | `mostly_flat`-Erkennung |
| 5 | Re-Route während Fahrt | `route_hash_changes` (bewusst oder vermeiden) |
| 6 | Kurzer Firmen- / Depot-Bereich | Kurze Distanzen, viele Waypoints |

Pro Fahrt: **2–5 Minuten** fahren, Recorder mit `--duration-sec 300` oder Ctrl+C beenden.

### Meta-Report ausführen

```bash
cargo run -p truckpilot-telemetry --bin route-distance-meta-report -- logs/*.csv
cargo run -p truckpilot-telemetry --bin route-distance-meta-report -- --json meta_report.json --min-runs 3 logs/*.csv
```

| Option | Beschreibung |
|--------|--------------|
| `--min-runs <n>` | Mindestanzahl Runs für `plausible` (Default: 3) |
| `--json <path>` | Maschinenlesbarer Meta-Report |

Exit-Codes: **0** bei `plausible` / `inconclusive`, **2** bei `suspicious`, **1** bei Parse-/Inputfehlern.

Einzel-Reports pro Datei weiterhin:

```bash
cargo run -p truckpilot-telemetry --bin route-distance-report -- logs/autobahn_gerade.csv
```

### Wann darf `ROUTE_WP_FLAG_UNTRUSTED` entfernt werden?

Erst wenn der **Meta-Report** `plausible` liefert:

- Mindestens **3 Runs** (`--min-runs`, Default)
- Besser **5+ Runs** über verschiedene Straßenarten (siehe Testmatrix)
- **Keine** `suspicious` Runs
- Mindestens **2** einzelne Runs mit Verdict `plausible`
- Gewichtete Distance-Coverage **≥ 80 %**
- Globale bad-monotonic-Ratio **≤ 25 %**

Ein einzelner guter Lauf oder nur inconclusive Meta-Reports reichen **nicht**. Für die formale Promotion gilt das strengere **Verification Gate** (Phase 5m).

## 4. Promotion Gate (Phase 5m)

`route-distance-verify` ist **strenger** als `route-distance-meta-report` und dient als formelles Gate vor einer manuellen Code-Review zur Entfernung von `ROUTE_WP_FLAG_UNTRUSTED`. **Das Tool ändert keinen Code und entfernt das Flag nicht automatisch.**

```bash
cargo run -p truckpilot-telemetry --bin route-distance-verify -- logs/*.csv

cargo run -p truckpilot-telemetry --bin route-distance-verify -- \
  --min-runs 5 \
  --require-scenarios autobahn,stadt,auffahrt,depot \
  --json verification.json \
  --markdown verification.md \
  logs/*.csv
```

| Option | Default | Beschreibung |
|--------|---------|--------------|
| `--min-runs <n>` | 5 | Mindestanzahl CSV-Runs |
| `--require-scenarios <list>` | (leer) | Komma-getrennte Pflicht-Szenarien |
| `--json <path>` | — | Maschinenlesbarer Verification-Report |
| `--markdown <path>` | — | Promotion-Dokument für manuelle Review |

**Exit-Codes:** `0` = passed, `2` = failed/inconclusive, `1` = Input-/Parsefehler

### Empfohlene Dateinamen (Szenario-Tagging)

Benenne CSVs nach Szenario, damit `--require-scenarios` greift:

| Dateiname | Szenario |
|-----------|----------|
| `autobahn_01.csv` | Autobahn gerade |
| `stadt_01.csv` | Stadt / Kreuzungen |
| `auffahrt_01.csv` | Ab-/Auffahrt |
| `depot_01.csv` | Firmen- / Depot-Bereich |
| `pause_01.csv` | Pause / Stillstand |
| `reroute_01.csv` | Re-Route (Hash-Wechsel erlaubt) |

Alternativ: Sidecar `autobahn_01.meta.json` neben der CSV:

```json
{
  "scenario": "autobahn",
  "ets2_version": "1.59",
  "map": "vanilla",
  "notes": "straight highway route"
}
```

Sidecar überschreibt Dateiname-Erkennung.

### Verification-Kriterien (Default)

| Kriterium | Schwelle |
|-----------|----------|
| Runs | ≥ 5 |
| Meta-Verdict | `plausible` |
| Suspicious Einzel-Runs | 0 |
| Gewichtete Distance-Coverage | ≥ 90 % |
| Bad-monotonic-Ratio global | ≤ 10 % |
| Runs ohne Distance | 0 |
| First-distance-jumps ohne Hash-Change | 0 |
| Route-Hash-Changes | nur in `reroute_*`-Runs |
| `percent_untrusted` | darf 100 % sein (Flag wird gerade verifiziert) |

### Verdicts

| Verdict | Bedeutung |
|---------|-----------|
| **passed** | Alle Kriterien erfüllt → eligible nach Code-Review |
| **failed** | Qualitäts- oder Szenario-Probleme |
| **inconclusive** | Zu wenige Runs, sonst kein hartes Qualitätsproblem |

**Reasons:** `insufficient_runs`, `meta_not_plausible`, `suspicious_runs_present`, `low_distance_coverage`, `bad_monotonic_ratio_too_high`, `no_distance_run`, `unexplained_distance_jumps`, `route_hash_instability`, `missing_required_scenarios`

### Wann darf `ROUTE_WP_FLAG_UNTRUSTED` entfernt werden?

1. `route-distance-verify` liefert **passed** (Exit-Code 0)
2. Empfehlung im Report: *Eligible to promote distance @+0x14 to trusted after code review*
3. **Separater manueller Code-Review-Schritt** in DLL/Core — nicht automatisch durch dieses Tool

## 5. CSV-Spalten (pro Sample)

| Spalte | Bedeutung |
|--------|-----------|
| `wall_time_ms` | Unix-Zeitstempel (ms) |
| `sequence` | SHM-Sequenz |
| `route_hash` | Route-Identifikator |
| `valid` | Snapshot gültig |
| `waypoint_count` | Anzahl Waypoints |
| `distance_count` | Waypoints mit `HAS_DISTANCE` |
| `distance_untrusted_count` | Waypoints mit `UNTRUSTED` |
| `distance_first_m` / `distance_last_m` | Erste/letzte Distance (m) |
| `distance_min_m` / `distance_max_m` | Min/Max über Waypoints mit Distance |
| `distance_increase_count` | Schritte mit steigender Distance |
| `distance_drop_max_m` | Größter Abfall zwischen aufeinanderfolgenden Distances |
| `distance_step_avg_m` | Durchschnittlicher Schritt |
| `distance_monotonic_status` | `ok`, `flat`, `increasing`, `jumpy`, `none`, … |
| `position_count` | Waypoints mit Position |
| `coord_status` | Koordinaten-Diagnose (separat von Distance) |
| `first_uid` / `last_uid` | UID des ersten/letzten Waypoints |

Waypoint-Details (optional JSONL): `wall_time_ms`, `index`, `uid`, `distance`, `flags`, `flag_names`, …

## 6. Report interpretieren

### Meta-Verdicts (Phase 5l)

| Verdict | Bedeutung |
|---------|-----------|
| **plausible** | ≥`min-runs` Runs, ≥2 plausible Einzel-Runs, 0 suspicious, gewichtete Coverage ≥80 %, bad-monotonic ≤25 % |
| **suspicious** | ≥1 suspicious Run (bei ≥2 Runs), Runs ohne Distance, viele first-distance-jumps, bad-monotonic >40 % |
| **inconclusive** | Zu wenige Runs oder gemischtes Bild |

Typische Meta-Gründe: `insufficient_runs`, `too_few_plausible_runs`, `suspicious_runs_present`, `low_weighted_distance_coverage`, `high_bad_monotonic_ratio`, `runs_without_distance`, `multiple_first_distance_jump_runs`

### Einzel-Report-Verdicts

| Verdict | Bedeutung |
|---------|-----------|
| **plausible** | Viele Samples, ≥80 % mit Distance, überwiegend monoton ok/flat, wenige Sprünge, stabiler `route_hash` |
| **inconclusive** | Zu wenig Daten oder gemischtes Bild |
| **suspicious** | Viele increasing/jumpy Samples, große Sprünge ohne Hash-Wechsel, instabile `distance_count`, gar keine Distance |

### Typische Gründe (`reasons`)

- `no_distance_samples` — DLL liefert keine Distance-Flags
- `insufficient_duration` — zu wenige Samples (<20)
- `too_many_increases` — Restdistanz steigt oft (unerwartet beim Annähern ans Ziel)
- `too_many_jumps` — große Sprünge zwischen Samples
- `mostly_flat` — Distance ändert sich kaum trotz Fahrt
- `route_hash_changes` — Route wurde neu geladen/geplant

### Wann `UNTRUSTED` entfernen?

Siehe Abschnitt **Promotion Gate** (Phase 5m). Kurz: `route-distance-verify` → **passed** + manueller Code-Review.

## 7. Verwandte Tools

```bash
# Einmaliger SHM-Snapshot (JSON/CSV pro Waypoint)
cargo run -p truckpilot-telemetry --bin route-shm-dump -- --json route.json

# Synthetischer SHM für lokale Tests
cargo run -p truckpilot-telemetry --bin route-shm-sim -- --with-positions
```

Diagnose-Logik: `crates/telemetry/src/nav_route.rs` (`diagnose_route_distances`, `decode_waypoint_flag_names`).

## Siehe auch

- [blackboard_keys.md](blackboard_keys.md) — Blackboard-Keys Phase 5i–5m
- Phase 5n — Manuelle Entfernung von `ROUTE_WP_FLAG_UNTRUSTED` nach Verification-`passed` + Code-Review
