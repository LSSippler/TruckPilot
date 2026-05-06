# ============================================================
# TruckPilot - Erweiterter Live-Test Starter (V2)
# ============================================================

$ErrorActionPreference = "Stop"
$GraphPath = "output\graph.json"

function Write-Header {
    Clear-Host
    Write-Host "==============================================" -ForegroundColor Cyan
    Write-Host "     TruckPilot - Erweiterter Live-Test" -ForegroundColor Cyan
    Write-Host "==============================================" -ForegroundColor Cyan
    Write-Host ""
}

function Write-Step {
    param([string]$Text)
    Write-Host "[Schritt] $Text" -ForegroundColor Yellow
}

function Write-OK {
    param([string]$Text)
    Write-Host "[OK]    $Text" -ForegroundColor Green
}

function Write-Warn {
    param([string]$Text)
    Write-Host "[WARN]  $Text" -ForegroundColor Yellow
}

function Write-Err {
    param([string]$Text)
    Write-Host "[FEHLER] $Text" -ForegroundColor Red
}

Write-Header

# 1. Voraussetzungen prüfen
Write-Step "Prüfe Voraussetzungen..."

# vJoy
if (-not (Test-Path "C:\Program Files\vJoy\x64\vJoyInterface.dll")) {
    Write-Err "vJoy ist nicht installiert!"
    exit 1
}
Write-OK "vJoy gefunden"

# Telemetrie-DLL
$dllPath = "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2\bin\win_x64\plugins\truckpilot_telemetry.dll"
if (-not (Test-Path $dllPath)) {
    Write-Warn "Telemetrie-DLL nicht gefunden. Bitte install_dll.ps1 ausführen."
} else {
    Write-OK "Telemetrie-DLL gefunden"
}

# 2. graph.json sicherstellen
if (-not (Test-Path $GraphPath)) {
    Write-Step "graph.json nicht gefunden → wird neu generiert..."
    dotnet run --project TruckPilot.NET\TruckPilot.CLI -- --hashfs-sectors "C:\temp\ets2_sectors" | Out-Null

    if (-not (Test-Path $GraphPath)) {
        Write-Err "Konnte graph.json nicht erzeugen!"
        exit 1
    }
    Write-OK "graph.json wurde erstellt"
} else {
    Write-OK "graph.json gefunden"
}

# 3. Route ermitteln
Write-Step "Ermittle Start- und Ziel-UID..."
$routeOutput = .\scripts\pick_route_uids.ps1 -GraphPath $GraphPath 2>&1

if ($LASTEXITCODE -ne 0) {
    Write-Err "Konnte keine gültige Route ermitteln."
    exit 1
}

$startUid = ($routeOutput | Select-String "Start UID:").Line -replace "Start UID:\s*", ""
$goalUid  = ($routeOutput | Select-String "Goal UID:").Line  -replace "Goal UID:\s*", ""

Write-OK "Route ermittelt"
Write-Host "        Start: $startUid" -ForegroundColor Green
Write-Host "        Goal:  $goalUid"  -ForegroundColor Green
Write-Host ""

# 4. ETS2 Status
$ets2 = Get-Process -Name "eurotrucks2" -ErrorAction SilentlyContinue
if (-not $ets2) {
    Write-Warn "ETS2 läuft nicht!"
    Write-Host "   Bitte starte ETS2 und setze dich in einen Lkw." -ForegroundColor Yellow
    Write-Host "   Drücke [Enter], wenn du bereit bist..." -ForegroundColor Yellow
    Read-Host
} else {
    Write-OK "ETS2 läuft"
}

# 5. Autopilot starten
Write-Step "Starte Autopilot mit vJoy..."
Write-Host ""

$cmdArgs = @(
    "run", "--release", "--bin", "truckpilot", "--",
    "--graph-json", $GraphPath,
    "--start", $startUid,
    "--goal", $goalUid,
    "--vjoy-device", "1",
    "-v"
)

Start-Process -FilePath "cargo" -ArgumentList $cmdArgs -NoNewWindow

Write-Host ""
Write-OK "Autopilot wurde gestartet!"
Write-Host "Beobachte die Konsole und das Spiel." -ForegroundColor Cyan
Write-Host ""
Write-Host "Tipp: Mit Strg+C in der Autopilot-Konsole kannst du abbrechen." -ForegroundColor DarkGray