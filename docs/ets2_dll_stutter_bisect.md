# ETS2 DLL Stutter Bisect

## Ziel

Isolieren, **ab welcher minimalen Stufe** die geladene `truckpilot_telemetry.dll`
ETS2 messbar schlechter laufen lässt als komplett ohne TruckPilot-DLL. Dazu wird
der Hot-Path der DLL in klar abgegrenzte, einzeln aktivierbare und crash-sichere
Stufen zerlegt, jede mit eigenen Perf-Countern.

Subjektiv: ohne TruckPilot-DLL läuft ETS2 besser. Mit DLL stottert es etwas
mehr — obwohl intern fast alles kalt ist. Dieses System beantwortet die Frage
**welche** Komponente den Unterschied macht.

## Aktueller Befund (Ausgangslage)

| Befund | Status |
|---|---|
| `route_resolve_status=route_resolver_disabled_safe_mode`, `resolver_attempts=0` | Default/off ist kalt |
| `notify_frame_tick_count=0`, `worker_wake_set_event_count=0`, `worker_walk_count=0`, `pattern_scan_count=0` | Worker-Wake-Storm behoben |
| `truckpilot_input.disable` → `input_enabled=false`, `input_event_cb_count=0` | Input-disable funktioniert |
| `truckpilot_minimal_telemetry.enable` → `minimal_telemetry_enabled=true`, `shm_write_count=0`, `frame_cb_over_1000us=0` | Minimal-Telemetry ist kalt |
| Trotzdem: **ohne DLL besser als mit DLL** | offen → dieses Bisect-System |

Offene Verdächtige (von „billig" nach „teuer"): bloßes DLL-Load, SCS-Init,
Callback-Registrierung, der Frame-Callback selbst, QPC/Profiling, Perf-SHM-Publish,
Telemetry-SHM-Full-Copy, READY_EVENT SetEvent, RouteBlackboard-Write, Dispatch,
Input-SDK.

## Diagnose-Level

Aktiviert durch eine **leere Markerdatei** im ETS2-Plugin-Ordner (neben
`truckpilot_telemetry.dll`). Der Level wird **einmal beim DLL-Init** gelesen und
für den Frame-Hot-Path gecached — kein Dateisystem-Zugriff pro Frame.

| # | Enable-Datei | Level | Aktive Hot-Path-Komponenten (zusätzlich zur vorherigen Stufe) |
|---|---|---|---|
| — | *(keine Datei)* | `normal_default_off` | = aktueller sicherer Default/off |
| 0 | `truckpilot_diag.load_only` | load_only | nur DLL-Load + `scs_telemetry_init` OK. **Kein** SHM, Callback, Worker |
| 1 | `truckpilot_diag.init_only` | init_only | + Telemetry-SHM, Perf-SHM, RouteBlackboard (empty publish). **Keine** Frame-Callbacks |
| 2 | `truckpilot_diag.callback_noop` | callback_noop | + Frame-Callback registriert; pro Frame nur 1 Atomic-Counter, dann return |
| 3 | `truckpilot_diag.callback_counter` | callback_counter | + Perf-Snapshot-Publish pro Frame (Counter wird live sichtbar) |
| 4 | `truckpilot_diag.callback_qpc` | callback_qpc | + QPC-Frame-Timing (`frame_cb_us_*`) |
| 5 | `truckpilot_diag.perf_snapshot` | perf_snapshot | + dedizierte Perf-Snapshot-Komponente |
| 6 | `truckpilot_diag.telemetry_shm` | telemetry_shm | + Telemetry-SHM Full-Copy pro Frame |
| 7 | `truckpilot_diag.ready_event` | ready_event | + READY_EVENT `SetEvent` pro Frame |
| 8 | `truckpilot_diag.route_bb` | route_bb | + RouteBlackboard-Frame-Write pro Frame |
| 9 | `truckpilot_diag.normal_default_off` | normal | expliziter aktueller Default/off (Resolver off, Dispatch hard-off-gate, kein Worker-Wake, Input je nach `truckpilot_input.disable`) |

### Priorität bei mehreren Dateien

Die **niedrigste / sicherste** Stufe gewinnt. Existieren z. B. `callback_noop`
und `route_bb` gleichzeitig, gewinnt `callback_noop`. Der Gewinner und alle
ignorierten höherprioren Dateien werden einmal ins Sidecar geloggt:

```text
diagnostic level=callback_noop
diagnostic level selected=callback_noop source=truckpilot_diag.callback_noop
diagnostic ignored higher-priority diag level files: truckpilot_diag.route_bb
```

### Kumulatives Verhalten — Counter vs. Arbeit

- **Component-Counter** (`callback_noop_count`, `callback_qpc_count`,
  `telemetry_shm_component_count`, …): jede Stufe erhöht **nur ihren eigenen**.
  So beweist ein Test pro Stufe die saubere Isolation.
- **Work-Counter** (`shm_write_count`, `ready_event_set_count`,
  `route_bb_frame_write_count`): **kumulativ** ab der Stufe, die die Arbeit
  einführt. `route_bb` macht also auch SHM-Write + Ready-Event.

### Wichtige Designentscheidungen

- **`callback_noop` publiziert NICHT pro Frame.** Das hält die Roh-Callback-
  Baseline pur (keine Publish-Memcpy-Kosten in der Messung). Der Level ist live
  am Sidecar (`diagnostic frame path active level=callback_noop`) und am
  Init-Publish (`diag_level=callback_noop` im Dump) erkennbar; `callback_noop_count`
  steigt im Live-Dump **nicht**. Genau dafür existiert `callback_counter`: es
  fügt den Publish hinzu, sodass der Counter live hochzählt. Die Differenz in
  der subjektiven Glätte zwischen `callback_noop` und `callback_counter` =
  Kosten des Perf-SHM-Publish.
- **Diag-Level registrieren nur `frame_start` + `frame_end`** — NICHT die ~25
  Kanal-Callbacks. Der Bisect misst den Frame-Callback isoliert. Die ~25
  Kanal-Callbacks sind eine separate Achse; ist `load_only` sauber aber
  `callback_noop` stottert, kann auch die Kanal-Callback-Frequenz Thema sein
  (hier bewusst ausgeklammert).
- **Der Diag-Frame-Pfad läuft pro SCS-Callback — also auf `frame_start` UND
  `frame_end`**, genau wie der Normalpfad (`telemetry_frame_cb` ist für beide
  Events registriert und schreibt Telemetry-SHM/SetEvent bei jedem Callback).
  Deshalb sind die Diag-Component-Counter (und `shm_write_count`/
  `ready_event_set_count`) ≈ **2× der ETS2-FPS** — das ist erwartbar und treu zu
  den echten Per-Callback-Kosten von Normal. Einzige Asymmetrie: `route_bb` macht
  pro Callback einen BB-Write (kein Dedup), während Normal über den Dispatch
  dedupliziert (~1× pro Frame) — für den Bisect akzeptabel, der BB-Write-Kostenpunkt
  bleibt eindeutig zuordenbar.
- **`callback_noop_count` ist im Live-Dump immer `0`** (Atomic steigt intern, aber
  `callback_noop` publisht bewusst nicht → SHM bleibt auf dem Init-Snapshot). Live
  bestätigt die Sidecar-Zeile `diagnostic frame path active level=callback_noop`,
  dass Callbacks feuern; ab `callback_counter` zählt der Counter live hoch.
- **`telemetry_shm` kopiert genullte Truck-Daten** (Kanäle nicht registriert) —
  die Memcpy-Kosten sind identisch zur Normalstufe (gleiche Struktgröße).
- **`truckpilot_minimal_telemetry.enable` bleibt Legacy** und wird NICHT auf
  einen Diag-Level gemappt. Liegt eine `truckpilot_diag.*`-Datei vor, gewinnt der
  Diag-Level und Minimal-Telemetry wird ignoriert (einmal geloggt). Grob
  entspricht Legacy-Minimal etwa `callback_counter`..`perf_snapshot`
  (Counter + Perf-Publish, ohne SHM/Ready/BB).
- **Resolver ist in allen Diag-Leveln (außer normal) zwangsweise off.** Route-
  Enable-Dateien werden ignoriert und einmal geloggt:
  `route resolver enable files ignored because diagnostic level=<level>`.

## Perf-Counter pro Level (Erwartung im Dump)

`cargo run -p truckpilot-telemetry --bin route-shm-dump -- --once --perf`

| Level | `diag_level` | callbacks_reg | telemetry_shm | ready_event | route_bb | dispatch | worker | Live-Counter der steigt |
|---|---|---|---|---|---|---|---|---|
| keine DLL | *(Dump: unavailable)* | — | — | — | — | — | — | — |
| load_only | *(Perf-SHM unavailable — kein Init)* | false | false | false | false | false | false | — (Sidecar ist Beweis) |
| init_only | init_only | false | false | false | false | false | false | — (keine Frames) |
| callback_noop | callback_noop | true | false | false | false | false | false | `callback_noop_count` (nur intern, kein Live-Publish) |
| callback_counter | callback_counter | true | false | false | false | false | false | `callback_counter_count` |
| callback_qpc | callback_qpc | true | false | false | false | false | false | `callback_qpc_count`, `frame_cb_us_*` |
| perf_snapshot | perf_snapshot | true | false | false | false | false | false | `perf_snapshot_count` |
| telemetry_shm | telemetry_shm | true | true | false | false | false | false | `telemetry_shm_component_count`, `shm_write_count` |
| ready_event | ready_event | true | true | true | false | false | false | + `ready_event_set_count` |
| route_bb | route_bb | true | true | true | true | false | false | + `route_bb_frame_write_count` |
| normal + input.disable | normal_default_off | true | true | true | true | true | true | Frame-Path voll; `input_enabled=false` |
| normal | normal_default_off | true | true | true | true | true | true | wie oben; `input_enabled=true` |

In allen Diag-Leveln gilt: `resolver_attempts=0`, `worker_walk_count=0`,
`pattern_scan_count=0`, `notify_frame_tick_count=0`, `route_tick_dispatch_count=0`,
`input_event_cb_count=0`.

## Sidecar-Logs (`truckpilot_telemetry.log`)

Einmal beim Init:

```text
diagnostic level=<level>
diagnostic callbacks_registered=<true/false>
diagnostic input_registered=<true/false>
diagnostic telemetry_shm=<true/false>
diagnostic ready_event=<true/false>
diagnostic route_bb_frame_write=<true/false>
diagnostic dispatch=<true/false>
diagnostic worker=<true/false>
```

`diagnostic input_registered` zeigt den **effektiven** Zustand und stimmt immer
mit `diag_input_registered` im Perf-Dump überein: `true` nur bei
`normal_default_off` **ohne** `truckpilot_input.disable`; in jedem Diag-Level und
bei `normal_default_off` **mit** `truckpilot_input.disable` ist es `false`.

Einmal beim ersten Frame: `diagnostic frame path active level=<level>` (kein
Per-Frame-Log).

## Interpretation

| Symptom | Verdächtige Komponente |
|---|---|
| Schon `load_only` stottert | bloßes Plugin-/DLL-Load oder der ETS2-Plugin-Loader |
| Erst `callback_noop` stottert | der SCS-Frame-Callback / Callback-Frequenz selbst |
| Erst `callback_counter` stottert | Perf-SHM-Publish (Snapshot-Memcpy pro Frame) |
| Erst `callback_qpc` stottert | QPC / Profiling-Overhead |
| Erst `perf_snapshot` stottert | dedizierte Perf-Snapshot-Komponente |
| Erst `telemetry_shm` stottert | Telemetry-SHM Full-Copy |
| Erst `ready_event` stottert | `SetEvent` / Consumer-Wake |
| Erst `route_bb` stottert | RouteBlackboard-Write |
| Nur `normal_default_off` stottert | Dispatch/Status-Gate / per-Frame Enable-File-Stat |
| Nur mit Input (ohne `input.disable`) | SCS Input-SDK |

> Hinweis: Der Normalpfad statet pro Frame das Enable-File (Minimal-Check). Wenn
> `route_bb` sauber ist aber `normal_default_off` stottert, ist der per-Frame
> Enable-File-Stat im Dispatch-/Minimal-Gate ein primärer Verdächtiger.

## Live-Testreihenfolge

Daemon nötig: **nein**.

```text
1.  keine TruckPilot-DLL
2.  truckpilot_diag.load_only
3.  truckpilot_diag.init_only
4.  truckpilot_diag.callback_noop
5.  truckpilot_diag.callback_counter
6.  truckpilot_diag.callback_qpc
7.  truckpilot_diag.perf_snapshot
8.  truckpilot_diag.telemetry_shm
9.  truckpilot_diag.ready_event
10. truckpilot_diag.route_bb
11. normal_default_off + truckpilot_input.disable
12. normal_default_off ohne input-disable
```

Jeweils ~60 Sekunden subjektiv fahren und Glätte vergleichen. Der erste Level,
der spürbar schlechter ist als der vorherige, isoliert die schuldige Komponente.

### PowerShell-Helper

Vorbereitung (einmal):

```powershell
$plugins = "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2\bin\win_x64\plugins"

Stop-Process -Name eurotrucks2 -Force -ErrorAction SilentlyContinue
Start-Sleep -Seconds 2

Remove-Item "$plugins\truckpilot_telemetry.log" -ErrorAction SilentlyContinue
Remove-Item "$plugins\truckpilot_input.disable" -ErrorAction SilentlyContinue
Remove-Item "$plugins\truckpilot_minimal_telemetry.enable" -ErrorAction SilentlyContinue
Remove-Item "$plugins\truckpilot_diag.*" -ErrorAction SilentlyContinue
Remove-Item "$plugins\truckpilot_route_resolver.*" -ErrorAction SilentlyContinue

cargo copy-ets2-dll --release
```

Einen Level setzen (Beispiel `callback_noop`):

```powershell
Stop-Process -Name eurotrucks2 -Force -ErrorAction SilentlyContinue
Start-Sleep -Seconds 2
Remove-Item "$plugins\truckpilot_telemetry.log" -ErrorAction SilentlyContinue
Remove-Item "$plugins\truckpilot_diag.*" -ErrorAction SilentlyContinue
New-Item -ItemType File "$plugins\truckpilot_diag.callback_noop" -Force
cargo copy-ets2-dll --release
```

ETS2 starten, Profil laden, ~60 s fahren, dann auslesen:

```powershell
cargo run -p truckpilot-telemetry --bin route-shm-dump -- --once --perf
Get-Content "$plugins\truckpilot_telemetry.log" -TotalCount 200
```

> Für Stufe 1 (keine DLL): `truckpilot_telemetry.dll` aus dem Plugin-Ordner
> entfernen (oder umbenennen). Für `load_only` zeigt `--perf` „unavailable" —
> das ist erwartet; das Sidecar-Log ist der Beweis.
