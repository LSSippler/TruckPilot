# RUNBOOK_LIVETEST

Kurzablauf fuer den Live-Test auf Windows.

## 1) Vorbereitung (ETS2 aus)

```powershell
cd C:\Users\Sippler\Documents\TruckPilot\TruckPilot.TelemetryDLL
Remove-Item -Recurse -Force build -ErrorAction SilentlyContinue
.\build.ps1 -Config Release
Copy-Item build\Release\truckpilot_telemetry.dll "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2\bin\win_x64\plugins\" -Force
```

## 2) Graph erzeugen

```powershell
cd C:\Users\Sippler\Documents\TruckPilot
python -m ets2_hashfs extract "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2\base_map.scs" --sectors --out "C:\temp\ets2_sectors"
dotnet run --project TruckPilot.NET\TruckPilot.CLI -- --hashfs-sectors "C:\temp\ets2_sectors" -v
```

## 3) UIDs automatisch auswaehlen

```powershell
.\scripts\pick_route_uids.ps1
```

## 4) ETS2 starten und MMF pruefen

```powershell
dotnet run --project TruckPilot.NET\TruckPilot.CLI -- --check-telemetry-dll
```

Erwartet: `OK`

## 5) Route testen (.NET)

```powershell
$json = Get-Content graph.json | ConvertFrom-Json
$edge = $json.Edges | Select-Object -First 1
dotnet run --project TruckPilot.NET\TruckPilot.CLI -- --hashfs-sectors "C:\temp\ets2_sectors" --start $edge.FromNodeUid --goal $edge.ToNodeUid -v
```

## 6) Live-Loop (Rust + vJoy)

```powershell
$json = Get-Content graph.json | ConvertFrom-Json
$edge = $json.edges | Select-Object -First 1
cargo run --release -- --hashfs-sectors "C:\temp\ets2_sectors" --start $edge.from_node_uid --goal $edge.to_node_uid --vjoy-device 1 -v
```

## Troubleshooting

- `MMF not found`:
  - ETS2 nicht gestartet oder Plugin nicht geladen.
- `magic mismatch`:
  - Alte DLL geladen. ETS2 schliessen, DLL neu kopieren, ETS2 neu starten.
- `No route found`:
  - Andere UIDs waehlen (verbundene Kante aus `Edges`).
- `vJoy: not enabled`:
  - vJoy installieren, Device 1 aktivieren, in ETS2 als Eingabegeraet waehlen.
