# Phase 6 — Integration-Test-Anleitung

End-to-End-Verifikation der Phase-6-Telemetrie-Pipeline:

> **Quelle (SHM oder HTTP) → `TelemetryReader` mit Sanity-Cooldown → Daemon-Loop → Blackboard + IPC-Broadcast → Plugins / UI**

Memory-Reader bleibt Phase 6.4 — `game_versions.toml` enthält heute keine echten ETS2-Offsets.

---

## Voraussetzungen

| Tool | Wofür | Pflicht? |
|------|------|----------|
| `cargo` (Rust 1.78+) | Build | ja |
| `wscat` oder vergleichbarer WebSocket-Client | IPC-Frame-Verifikation | optional |
| ETS2 ≥ 1.50 + `truckpilot_telemetry.dll` in `<ETS2>/bin/win_x64/plugins/` | Setup B | optional |

`wscat` installieren: `npm install -g wscat`.

---

## Setup A — SHM via `shm-sim` (kein ETS2 nötig)

`shm-sim` schreibt in dieselbe SHM-Region wie die echte `truckpilot_telemetry.dll`. Damit testen Sie den ganzen Stack ohne laufendes Spiel.

### Schritte

1. **Terminal 1 — Simulator starten:**
   ```powershell
   cargo run -p truckpilot-telemetry --bin shm-sim
   ```
   Default-Szenario: `highway` mit 80 km/h, 20 Hz Update-Rate. Andere Szenarien:
   - `--scenario city` — Stop-and-go mit wechselnden Tempolimits
   - `--scenario fuel` — Niedriger Tankfüllstand (für Plugin-Smoke unten)
   - `--scenario brake` — Notbremsung von 90 km/h
   - `--scenario parked` — Motor aus

2. **Terminal 2 — Daemon starten:**
   ```powershell
   cargo run -p truckpilot-core -- daemon
   ```

3. **Erwartung im Daemon-Log:** Innerhalb von 1–2 Sekunden:
   ```
   INFO Telemetry source switched: None -> SharedMemory
   INFO Tick — steering=0.00 throttle=0.00 brake=0.00 (plugins=N)
   ```

4. **Stoppt man `shm-sim`** (Ctrl-C in Terminal 1), schaltet die Cascade nach kurzer Zeit weiter:
   ```
   INFO Telemetry source switched: SharedMemory -> Http
   ```
   bzw. `-> None` wenn auch HTTP nicht antwortet.

---

## Setup B — Echtes ETS2 (Windows)

### DLL bauen und installieren

```powershell
# Cross-compile für Windows-x64 (auf Linux: x86_64-pc-windows-gnu).
cargo build --release -p truckpilot-telemetry-dll

# Resultat liegt in target/release/truckpilot_telemetry.dll
copy target\release\truckpilot_telemetry.dll "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2\bin\win_x64\plugins\"
```

ETS2 lädt SDK-Plugins automatisch beim Start. Beim ersten Mal das Game-Menü `Optionen → Aktivierte Plugins` prüfen.

### Verifikation

1. ETS2 starten, Save laden, in den Truck steigen, losfahren (ohne Fahrt schreibt der SDK keine Werte).
2. **Terminal:**
   ```powershell
   cargo run -p truckpilot-core -- daemon
   ```
3. Im Daemon-Log: `Telemetry source switched: None -> SharedMemory`. Beim Lenken / Gas-Geben aktualisieren sich die Werte im 1-Sekunden-Tick-Log.

---

## Verifikations-Szenarien

### V1 — SHM-Source aktiv

Erfolgskriterium aus den vorigen Setups:

- Daemon-Log zeigt `Telemetry source switched: None -> SharedMemory`.
- 1-Sekunden-Tick-Log läuft kontinuierlich.
- Kein `Telemetry source X suspended` (das wäre der Sanity-Trip).

### V2 — IPC-Broadcast erreicht UI-Clients

Während Daemon + `shm-sim` (oder ETS2) laufen:

```powershell
wscat -c ws://127.0.0.1:8765
```

Erwartung: Zwei Initial-Frames (`hello`, `plugin_list`), dann ~20 `telemetry`-Frames pro Sekunde:

```json
{"type":"telemetry","v":1,"data":{"position":[12345.6,78.9,-1234.5],"heading":0.5,"speed_ms":22.2,"engine_rpm":1350.0,"cruise_control_kmh":80.0,"nav_speed_limit_kmh":80.0}}
```

Hinweis: `pitch`, `roll`, `fuel_liters`, `odometer_km` werden vom Daemon ans Blackboard, **nicht** an die UI gesendet. Die `TelemetrySnapshot`-Erweiterung um diese Felder ist Phase-7 (Protokoll-Bump).

### V3 — Sanity-Cooldown bei garbage Werten

`shm-sim` hat heute keinen "insane"-Modus. Manueller Test über kurzes Patchen:

1. In `crates/telemetry/src/bin/shm_sim.rs` temporär das Highway-Layout auf `speed_ms = 999.0` setzen (außerhalb `[-50, 100]`).
2. Daemon starten → erwartet:
   ```
   WARN Telemetry source SharedMemory returned insane frame (1/3): speed_ms=999 ...
   WARN Telemetry source SharedMemory returned insane frame (2/3): ...
   WARN Telemetry source SharedMemory returned insane frame (3/3): ...
   WARN Telemetry source SharedMemory suspended for 10s after 3 consecutive insane frames
   INFO Telemetry source switched: SharedMemory -> Http
   ```
3. Nach 10 s:
   ```
   INFO Telemetry source SharedMemory cooldown expired — retrying
   ```
   Wenn `shm-sim` weiter Garbage liefert, trippt der Cooldown sofort wieder.

Patch danach zurücknehmen.

> TODO Phase 6.5+: Einen `--insane-burst N` Flag in `shm-sim` einbauen, der für N Frames bewusst out-of-range Werte schreibt. Vermeidet das manuelle Patchen.

### V4 — Plugin-Smoke: fuel-stops sieht echte Werte

`fuel-stops` setzt `fuel_stop.needed = "true"` sobald `telemetry.fuel_liters` unter dem Threshold (Default 50 L) liegt — und kann das jetzt wirklich beobachten, weil Task 3 die Werte ans Blackboard mirrort.

1. **Terminal 1:**
   ```powershell
   cargo run -p truckpilot-telemetry --bin shm-sim -- --scenario fuel
   ```
   Das `fuel`-Szenario startet bei niedrigem Tankfüllstand.
2. **Terminal 2:** Daemon mit Plugins starten:
   ```powershell
   cargo run -p truckpilot-core -- daemon
   ```
3. **Erwartung im Daemon-Log:** `[fuel-stops]`-Tracing-Einträge erscheinen, sobald die Telemetrie unter den Threshold rutscht. Beobachten Sie das Blackboard via dem 1-Sekunden-Tick-Log oder einem zusätzlichen `tracing::info!` in der Daemon-Loop:
   ```rust
   tracing::info!("fuel_stop.needed={:?}", blackboard.get("fuel_stop.needed"));
   ```

---

## Troubleshooting

| Symptom | Wahrscheinliche Ursache | Lösung |
|---------|------------------------|--------|
| `Telemetry source switched: None -> None` (sticky) | Weder SHM noch HTTP antwortet. | `shm-sim` läuft? Funbit-Server (Port 25555) läuft? |
| `WARN Telemetry source X suspended …` direkt beim Boot | Source liefert Werte außerhalb der Sanity-Bereiche. | `WARN`-Zeile zeigt `speed_ms=… engine_rpm=… heading=…` — gegen Bounds in `crates/telemetry/src/lib.rs::is_sane` abgleichen. |
| WebSocket-Verbindung wird sofort geschlossen | Daemon noch nicht ready oder bereits gestoppt. | Daemon-Log auf `IPC WebSocket server listening on 127.0.0.1:8765` prüfen. |
| Telemetrie-Frames kommen, aber Plugins reagieren nicht | Mock-Feature versehentlich aktiv. | `cargo run -p truckpilot-core -- daemon` ohne `--features mock_telemetry`. Real-Push ist `#[cfg(not(feature = "mock_telemetry"))]`. |
| `mock_telemetry`-Builds mit doppelten Frames in der UI | Erwartetes Verhalten — Mock-Sine + Real-Push sind getrennte Features, sollten nicht zusammen aktiv sein. | Build ohne Mock-Feature, oder Mock dauerhaft an mit Doku-Warning beim Daemon-Start. |

---

## Was Phase 6 NICHT abdeckt

| Feature | Status | Wo dokumentiert |
|---------|--------|----------------|
| Memory-Reading (PEB-Walking, Pattern-Scanning) | Phase 6.4 — `crates/telemetry/src/memory.rs` ist Stub, `game_versions.toml` ohne echte Offsets | Plan in `~/.claude/plans/giggly-tumbling-sundae.md` |
| `telemetry.fatigue` für break-planner | Phase 7 — keine Datenquelle in SHM oder HTTP, break-planner-Fallback `0.0` ist sicher unter Threshold `0.85` | Plan-Entscheidung |
| TelemetrySnapshot-Erweiterung (pitch, roll, fuel, odometer in UI) | Phase 7 — erfordert `PROTOCOL_VERSION`-Bump und TS-Mirror in `crates/ui/src/lib/types.ts` | Snapshot-Doc-Kommentar in `crates/core/src/main.rs::snapshot_from` |
| Echter vJoy-Output (statt tracing-Stub) | Phase 6+ Hardware-Test — `crates/plugins/vjoy-output/src/lib.rs::send_to_vjoy` und `crates/core/src/main.rs::apply_vjoy_failsafe` sind Stubs | TODO-Kommentare im Code |

---

## End-to-End-Erfolgs-Definition

Phase 6 gilt als erfolgreich verifiziert, wenn:

1. ✅ V1: Daemon erkennt SHM-Source und schaltet auf sie um.
2. ✅ V2: WebSocket-Client empfängt 20 Hz Telemetrie-Frames.
3. ✅ V3: Garbage-Werte trippen den 3-Frame-Cooldown, Source-Switch erfolgt, nach 10 s wird die Source automatisch reaktiviert.
4. ✅ V4: `fuel-stops` reagiert auf echte SHM-Werte (sichtbar im Blackboard).

Stand der Code-Tasks (alle grün im Workspace-Test):

| Task | Status |
|------|--------|
| Task 1 — Telemetry-Struct erweitert (fuel_liters, odometer_km) | ✅ |
| Task 2 — Sanity-Checks im TelemetryReader | ✅ |
| Task 3 — Blackboard-Helper im Daemon | ✅ |
| Task 5 — IPC-Broadcast mit echter Telemetrie | ✅ |
| Phase 6.5 — Diese Doku | ✅ |
| Task 4 — Memory-Reader | 📦 Phase 6.4 |
