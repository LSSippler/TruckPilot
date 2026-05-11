# start_pro_mods_live_loop.ps1 — ProMods Live-Test für TruckPilot
#
# Startet den Autopiloten mit ProMods-Karte auf einer weit entfernten Route
# (Middle East → Deep South, ~456 km).
#
# Voraussetzungen:
#   - output/graph.json liegt vor (Map-Export aus cargo run --enable-mods --write-graph)
#   - ETS2 läuft mit geladenem TruckPilot-Telemetry-Plugin
#   - vJoy Device 1 ist frei und in ETS2 als Eingabegerät konfiguriert
#   - ProMods-Mods liegen im ETS2-mod-Ordner
#
# Verwendung:
#   .\scripts\start_pro_mods_live_loop.ps1 [-Ets2Dir "C:\ETS2"] [-ModDir "C:\ETS2\mod"]

param(
    [string]$Ets2Dir,
    [string]$ModDir
)

$ErrorActionPreference = "Stop"
$projectDir = Split-Path -Parent (Split-Path -Parent $PSCommandPath)

function Write-Step { Write-Host "`n[PRO-MODS] $args" -ForegroundColor Cyan }
function Write-OK   { Write-Host "  ✓ $args" -ForegroundColor Green }
function Write-Warn { Write-Host "  ⚠ $args" -ForegroundColor Yellow }
function Write-Err  { Write-Host "  ✗ $args" -ForegroundColor Red ; exit 1 }

# ── 1. Locate ETS2 directories ──────────────────────────────────────────
if (-not $Ets2Dir) {
    $candidates = @(
        "$env:USERPROFILE\Documents\Euro Truck Simulator 2"
        "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2"
        "D:\SteamLibrary\steamapps\common\Euro Truck Simulator 2"
    )
    foreach ($d in $candidates) {
        if (Test-Path $d) { $Ets2Dir = $d; break }
    }
}
if (-not $Ets2Dir -or -not (Test-Path $Ets2Dir)) {
    Write-Err "ETS2 directory not found. Provide it with -Ets2Dir."
}

if (-not $ModDir) {
    $ModDir = Join-Path $Ets2Dir "mod"
}
if (-not (Test-Path $ModDir)) {
    Write-Warn "Mod directory not found: $ModDir"
    Write-Host "  Continuing without mods — route may fail."
}

Write-Step "ETS2 dir : $Ets2Dir"
Write-Step "Mod dir  : $ModDir"

# ── 2. Check graph.json ─────────────────────────────────────────────────
$graphPath = Join-Path $projectDir "output\graph.json"
if (-not (Test-Path $graphPath)) {
    Write-Err "graph.json not found at $graphPath. Run: cargo run --release -- --enable-mods --write-graph --ets2-dir `"$Ets2Dir`" --mod-dir `"$ModDir`" --verbose"
}
Write-OK "graph.json found: $graphPath"

# ── 3. Extract route UIDs ───────────────────────────────────────────────
# ProMods farthest-apart node pair (auto-identified 2026-05-06):
#   Start: Middle East  (x=0,     z=-193759)  → uid 0x9747D7C4BD47CE01
#   Goal:  Deep South   (x=38,    z=262796)   → uid 0x5FF506710180F56F
#   Distance: 456.6 km air-line
$startUid = "0x9747D7C4BD47CE01"
$goalUid  = "0x5FF506710180F56F"

# Alternative (224 km): $startUid = "0x17AF173FBD7E9752"; $goalUid = "0xC77AA162419A6800"

Write-Step "Route UIDs: $startUid → $goalUid (456 km)"

# ── 4. Telemetry diagnostics ────────────────────────────────────────────
Write-Step "Running telemetry diagnostics..."
$diagResult = cargo run --bin telemetry_diag -- --once 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Warn "Telemetry diagnostic returned exit code $LASTEXITCODE"
    Write-Host $diagResult
    Write-Host "  ETS2 must be running with the truck in the cockpit."
    Write-Host "  Proceeding anyway — autopilot will wait for telemetry."
} else {
    Write-OK "Telemetry OK"
}

# ── 5. Start autopilot ──────────────────────────────────────────────────
Write-Step "Starting TruckPilot autopilot with ProMods..."
Write-Host "  Route: $startUid -> $goalUid"
Write-Host "  vJoy:  Device 1"
Write-Host ""

cargo run --release --bin truckpilot -- `
  --graph-json $graphPath `
  --start $startUid `
  --goal $goalUid `
  --vjoy-device 1 `
  --prefer-speed `
  --verbose

if ($LASTEXITCODE -ne 0) {
    Write-Err "Autopilot exited with code $LASTEXITCODE"
}

Write-OK "Autopilot finished."
