# install_dll.ps1 — Copies truckpilot_telemetry.dll to the ETS2 plugin directory.
#
# Usage:
#   .\scripts\install_dll.ps1 [-Ets2Dir "C:\Path\To\ETS2"]
#
# If -Ets2Dir is not provided, auto-detects from standard Steam paths.

param(
    [string]$Ets2Dir
)

$ErrorActionPreference = "Stop"

function Write-Info  { Write-Host "[INSTALL] $args" -ForegroundColor Green }
function Write-Warn  { Write-Host "[WARN] $args" -ForegroundColor Yellow }
function Write-ErrorMsg { Write-Host "[ERROR] $args" -ForegroundColor Red }

# --- Locate ETS2 directory ---
if (-not $Ets2Dir) {
    $candidates = @(
        "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2"
        "D:\SteamLibrary\steamapps\common\Euro Truck Simulator 2"
        "$env:ProgramFiles(x86)\Steam\steamapps\common\Euro Truck Simulator 2"
    )
    foreach ($d in $candidates) {
        if (Test-Path $d) {
            $Ets2Dir = $d
            break
        }
    }
}

if (-not $Ets2Dir -or -not (Test-Path $Ets2Dir)) {
    Write-ErrorMsg "ETS2 directory not found. Provide it with -Ets2Dir."
    exit 1
}

$pluginDir = Join-Path $Ets2Dir "bin\win_x64\plugins"
if (-not (Test-Path $pluginDir)) {
    Write-ErrorMsg "Plugin directory not found: $pluginDir"
    exit 1
}

# --- Locate DLL ---
$projectDir = Split-Path -Parent (Split-Path -Parent $PSCommandPath)
$dllSrc = Join-Path $projectDir "target\release\truckpilot_telemetry_dll.dll"

if (-not (Test-Path $dllSrc)) {
    Write-Warn "DLL not found at $dllSrc"
    Write-Host "  Build it first: cargo build --release -p truckpilot_telemetry_dll"
    exit 1
}

# --- Copy ---
Copy-Item -Path $dllSrc -Destination $pluginDir -Force -Verbose
Write-Info "DLL installed to: $pluginDir"
Write-Info "Done. Restart ETS2 to load the plugin."
