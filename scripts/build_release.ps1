# TruckPilot release packaging script (Windows).
#
# Builds the Rust workspace in release mode and assembles a self-contained
# release\ directory plus a truckpilot-v1.0.zip archive.

[CmdletBinding()]
param()

$ErrorActionPreference = "Stop"
$RootDir   = (Resolve-Path "$PSScriptRoot\..").Path
Set-Location $RootDir

$ReleaseDir = Join-Path $RootDir "release"
$ZipName    = "truckpilot-v1.0.zip"
$ZipPath    = Join-Path $RootDir $ZipName

Write-Host "[1/6] cargo build --release" -ForegroundColor Cyan
cargo build --release
if ($LASTEXITCODE -ne 0) { throw "cargo build failed" }

Write-Host "[2/6] preparing release\ directory" -ForegroundColor Cyan
if (Test-Path $ReleaseDir)  { Remove-Item -Recurse -Force $ReleaseDir }
New-Item -ItemType Directory -Path $ReleaseDir          | Out-Null
New-Item -ItemType Directory -Path "$ReleaseDir\scripts" | Out-Null
New-Item -ItemType Directory -Path "$ReleaseDir\docs"    | Out-Null

Write-Host "[3/6] copying binary" -ForegroundColor Cyan
$mainBin = "target\release\truckpilot.exe"
if (-not (Test-Path $mainBin)) { $mainBin = "target\release\truckpilot" }
if (-not (Test-Path $mainBin)) { throw "truckpilot binary not found in target\release" }
Copy-Item $mainBin $ReleaseDir

foreach ($extra in @("benchmark", "telemetry_diag", "vjoy_test", "graph_stats", "route_stress")) {
    foreach ($suffix in @(".exe", "")) {
        $cand = "target\release\$extra$suffix"
        if (Test-Path $cand) { Copy-Item $cand $ReleaseDir }
    }
}

Write-Host "[4/6] copying telemetry DLL (if built)" -ForegroundColor Cyan
$dll = "TruckPilot.TelemetryDLL\build\Release\truckpilot_telemetry.dll"
if (Test-Path $dll) {
    Copy-Item $dll $ReleaseDir
} else {
    Write-Host "  - $dll not found (skipping; build it first)" -ForegroundColor Yellow
}

Write-Host "[5/6] copying config, scripts and docs" -ForegroundColor Cyan
if (Test-Path "truckpilot.toml") { Copy-Item "truckpilot.toml" $ReleaseDir }
Get-ChildItem "scripts\*.ps1" -ErrorAction SilentlyContinue | Copy-Item -Destination "$ReleaseDir\scripts\"
Get-ChildItem "scripts\*.sh"  -ErrorAction SilentlyContinue | Copy-Item -Destination "$ReleaseDir\scripts\"
foreach ($doc in @("README.md", "PROJECT_FINAL.md", "HANDOFF.md", "CHANGELOG.md", "offline_validation_results.txt")) {
    if (Test-Path $doc) { Copy-Item $doc "$ReleaseDir\docs\" }
}

Write-Host "[6/6] creating $ZipName" -ForegroundColor Cyan
if (Test-Path $ZipPath) { Remove-Item -Force $ZipPath }
Compress-Archive -Path $ReleaseDir -DestinationPath $ZipPath -Force

$sizeBytes = (Get-Item $ZipPath).Length
$sizeMb    = [math]::Round($sizeBytes / 1MB, 2)
Write-Host "Release archive: $ZipPath ($sizeMb MB / $sizeBytes bytes)" -ForegroundColor Green
