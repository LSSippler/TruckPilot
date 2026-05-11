# TruckPilot Live-Loop Startskript
# Dieses Skript startet den Autopiloten mit echter Karte, Telemetrie und vJoy.
# Voraussetzungen:
#   - ETS2 läuft und ein Lkw ist im Cockpit
#   - truckpilot_telemetry.dll ist im ETS2-Plugin-Ordner (scripts/install_dll.ps1)
#   - vJoy Device 1 ist konfiguriert
#   - output/graph.json existiert

$ErrorActionPreference = "Stop"
Set-Location "$PSScriptRoot\.."

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  TruckPilot Live-Loop Startskript" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# 1. Prüfen ob output/graph.json existiert
if (-not (Test-Path "output/graph.json")) {
    Write-Host "[FEHLER] output/graph.json nicht gefunden." -ForegroundColor Red
    Write-Host "Führe zuerst den .NET-Map-Export aus oder kopiere die graph.json in das output/-Verzeichnis."
    Read-Host "Drücke eine Taste zum Beenden"
    exit 1
}

# 2. UIDs extrahieren
Write-Host "[INFO] Extrahiere UIDs aus graph.json..." -ForegroundColor Yellow
$routeOutput = .\scripts\pick_route_uids.ps1 -GraphPath "output\graph.json" 2>&1
$startUid = ($routeOutput | Where-Object { $_ -match "^START_UID=" }) -replace "^START_UID=", ""
$goalUid  = ($routeOutput | Where-Object { $_ -match "^GOAL_UID=" })  -replace "^GOAL_UID=", ""

if (-not $startUid -or -not $goalUid) {
    Write-Host "[FEHLER] Konnte UIDs nicht extrahieren." -ForegroundColor Red
    Write-Host "Ausgabe von pick_route_uids.ps1:"
    Write-Host $routeOutput
    Read-Host "Drücke eine Taste zum Beenden"
    exit 1
}

Write-Host "[INFO] Start UID: $startUid" -ForegroundColor Green
Write-Host "[INFO] Goal UID:  $goalUid" -ForegroundColor Green
Write-Host ""

# 3. Telemetrie prüfen
Write-Host "[INFO] Prüfe Telemetrie..." -ForegroundColor Yellow
cargo run --bin telemetry_diag -- --once 2>&1 | Out-Null
if ($LASTEXITCODE -ne 0) {
    Write-Host "[WARNUNG] Telemetrie-Diagnose fehlgeschlagen. Die DLL könnte nicht geladen sein." -ForegroundColor Yellow
    Write-Host "[INFO] Starte trotzdem in 5 Sekunden... (STRG+C zum Abbrechen)"
    Start-Sleep -Seconds 5
}

# 4. Autopilot starten
Write-Host "[INFO] Starte Autopilot..." -ForegroundColor Yellow
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  Befehl: cargo run --release -- --graph-json output/graph.json --start $startUid --goal $goalUid --vjoy-device 1 -v" -ForegroundColor White
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

cargo run --release -- --graph-json output/graph.json --start $startUid --goal $goalUid --vjoy-device 1 -v

# 5. Ende
Write-Host ""
Write-Host "[INFO] Autopilot wurde beendet." -ForegroundColor Yellow
Read-Host "Drücke eine Taste zum Schließen"
