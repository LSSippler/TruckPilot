# TruckPilot — Live Test Guide

Diese Anleitung beschreibt, wie der erste Live-Test des TruckPilot-Autopiloten mit
eigener Telemetrie-DLL und vJoy-Steuerung durchgeführt wird.

## Voraussetzungen

- **ETS2 installiert** (Steam-Version, Windows x64)
- **vJoy installiert und konfiguriert**
  - vJoy Device 1 aktiviert
  - Achsen X (Lenkung), Y (Gas), Z (Bremse) freigeschaltet
  - In ETS2: Steuerung → vJoy als Eingabegerät auswählen
  - Lenkachse auf X, Gas auf Y, Bremse auf Z mappen
- **Rust-Toolchain** (stable, msvc target für Windows-DLL-Build)
- **DLL gebaut:** `cargo build --release -p truckpilot_telemetry_dll`
- **DLL installiert:** `./scripts/install_dll.sh` (Linux) oder `.\scripts\install_dll.ps1` (Windows)

## Schritt-für-Schritt

### 1. DLL installieren

```bash
# Bash (Linux / WSL)
./scripts/install_dll.sh
```
```powershell
# PowerShell (Windows)
.\scripts\install_dll.ps1
```

Das Skript kopiert `truckpilot_telemetry_dll.dll` nach:
```
<ETS2>/bin/win_x64/plugins/
```

### 2. ETS2 starten

Starte ETS2 und lade einen Spielstand. Die DLL wird automatisch geladen.
Überprüfe, ob die DLL aktiv ist:

1. Öffne `Dokumente/Euro Truck Simulator 2/game.log.txt`
2. Suche nach `[TruckPilot]` — es sollten Einträge wie `scs_telemetry_init called` erscheinen
3. Wenn nichts erscheint: siehe Troubleshooting

### 3. Route identifizieren (optional)

Falls eine `output/nodes.json` existiert:
```bash
python3 -c "import json; d=json.load(open('output/nodes.json')); print(d[0]['node_uid'], d[1]['node_uid'])"
```

Notiere zwei gültige Node-UIDs (z. B. `0x2935DE00004D04` und `0x2935DE66504D03`).

### 4. Live-Test starten

```bash
# Bash (Linux / WSL)
./scripts/run_live_test.sh 0x2935DE00004D04 0x2935DE66504D03
```
```powershell
# PowerShell (Windows)
.\scripts\run_live_test.ps1 -StartUid 0x2935DE00004D04 -GoalUid 0x2935DE66504D03
```

Das Skript führt zuerst einen **Route-Only-Test** aus (`--telemetry-disable`), der die
Route plant und ausgibt — **ohne** auf Telemetrie zu warten.

### 5. Live-Telemetrie-Loop starten

Wenn der Route-Only-Test erfolgreich war, starte den echten Loop **ohne** `--telemetry-disable`:

```bash
cargo run --release -- \
    --ets2-dir "C:/Program Files (x86)/Steam/steamapps/common/Euro Truck Simulator 2" \
    --start 0x2935DE00004D04 \
    --goal 0x2935DE66504D03 \
    --vjoy-device 1 \
    -v
```

## Erfolgskriterien

| Kriterium | Erwartetes Verhalten |
|-----------|---------------------|
| **DLL geladen** | `game.log.txt` enthält `[TruckPilot] scs_telemetry_init` |
| **Shared Memory** | Autopilot-Loop startet ohne "telemetry error" |
| **Route gefunden** | Ausgabe: `Route found: N nodes, cost=X, validated=true` |
| **Lenkung (vJoy)** | Lenkrad bewegt sich im Spiel in Richtung des Ziel-Waypoints |
| **Geschwindigkeit** | LKW beschleunigt/bremslt gemäß PID-Regler |
| **Waypoint-Wechsel** | Log zeigt `wp=1/4, wp=2/4` — Waypoints werden nacheinander erreicht |

## Troubleshooting

| Problem | Lösung |
|---------|--------|
| **DLL wird nicht geladen** | Prüfe `game.log.txt` auf Fehler. DLL muss in `bin/win_x64/plugins/` liegen. ETS2 neu starten. |
| **`[TruckPilot]` erscheint nie im Log** | Prüfe ob `truckpilot_telemetry_dll.dll` existiert. Falsche Architektur? (muss x64 sein) |
| **`telemetry error:` im Loop** | Shared Memory wird nicht gefüllt. Prüfe ob DLL im Log aktiv ist und `CreateFileMappingW` erfolgreich war. |
| **Route nicht gefunden** | UIDs existieren nicht in der Map. Verwende `--text-map-file` mit einem gültigen Sektor oder extrahiere UIDs aus `graph.json`. |
| **Lenkrad bewegt sich nicht** | vJoy Device 1 konfiguriert? In ETS2 unter Steuerung vJoy ausgewählt? Achsen korrekt gemappt? |
| **Lenkung schlägt wild aus** | PID-Gain zu hoch. Parameter im Code anpassen (`kp_steer` in `autopilot_loop.rs`). |
| **LKW fährt nicht los** | Navigation-Speed-Limit fehlt (DLL liefert 0). Fallback auf 80 km/h sollte greifen. Prüfe `target_speed_ms` im Log. |
| **Build schlägt fehl (DLL)** | DLL muss auf Windows mit msvc-Target gebaut werden: `rustup target add x86_64-pc-windows-msvc` |

## Verwendete Shared-Memory-Namen

| Ressource | Name |
|-----------|------|
| Shared Memory | `Local\TruckPilotTelemetry` |
| Ready Event | `Local\TruckPilotTelemetryReady` |

Diese Namen sind fest im Code definiert und können nicht per CLI geändert werden.
