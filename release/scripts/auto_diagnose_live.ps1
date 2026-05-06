$ErrorActionPreference = "Continue"

Set-Location "$PSScriptRoot\.."

function Step($msg) { Write-Host "`n=== $msg ===" -ForegroundColor Cyan }
function Ok($msg)   { Write-Host "[OK] $msg" -ForegroundColor Green }
function Warn($msg) { Write-Host "[WARN] $msg" -ForegroundColor Yellow }
function Err($msg)  { Write-Host "[ERR] $msg" -ForegroundColor Red }

# 1) ETS2 Prozess
Step "ETS2 Prozess"
$ets2 = Get-Process -Name eurotrucks2 -ErrorAction SilentlyContinue
if ($ets2) { Ok "ETS2 laeuft." } else { Warn "ETS2 laeuft nicht." }

# 2) Plugin DLL
Step "Plugin DLL"
$plugin = "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2\bin\win_x64\plugins\truckpilot_telemetry.dll"
if (Test-Path $plugin) {
    Ok "DLL gefunden: $plugin"
    Get-Item $plugin | Select-Object Name,Length,LastWriteTime | Format-Table -AutoSize
} else {
    Err "DLL fehlt: $plugin"
}

# 3) MMF Sanity Check
Step "MMF Sanity-Check"
$sanity = dotnet run --project TruckPilot.NET\TruckPilot.CLI -- --check-telemetry-dll 2>&1
$sanityText = ($sanity | Out-String).Trim()
Write-Host $sanityText

if ($sanityText -match "^OK\s*$") {
    Ok "MMF ist korrekt."
} elseif ($sanityText -match "MMF not found") {
    Err "MMF nicht gefunden."
    Warn "=> ETS2 starten, Profil laden, im Truck sitzen (nicht pausiert), dann erneut."
} elseif ($sanityText -match "magic mismatch") {
    Err "Magic mismatch erkannt."
    Warn "=> Alte DLL aktiv. ETS2 komplett beenden, DLL neu kopieren, ETS2 neu starten."
} else {
    Warn "Unbekannter Sanity-Status."
}

# 4) graph.json + Route Test
Step "Route-Test"
if (-not (Test-Path ".\graph.json")) {
    Warn "graph.json fehlt -> wird erzeugt..."
    dotnet run --project TruckPilot.NET\TruckPilot.CLI -- --hashfs-sectors "C:\temp\ets2_sectors" -v
}

if (Test-Path ".\graph.json") {
    try {
        $json = Get-Content .\graph.json | ConvertFrom-Json
        $edge = if ($json.edges) { $json.edges[0] } else { $json.Edges[0] }
        $start = if ($edge.from_node_uid) { $edge.from_node_uid } else { $edge.FromNodeUid }
        $goal  = if ($edge.to_node_uid)   { $edge.to_node_uid }   else { $edge.ToNodeUid }

        Write-Host "Start=$start Goal=$goal"

        $route = dotnet run --project TruckPilot.NET\TruckPilot.CLI -- --hashfs-sectors "C:\temp\ets2_sectors" --start $start --goal $goal -v 2>&1
        $routeText = ($route | Out-String).Trim()
        Write-Host $routeText

        if ($routeText -match "Route found") {
            Ok "Route erfolgreich."
        } elseif ($routeText -match "No route found") {
            Err "Keine Route gefunden."
            Warn "=> Andere Edge in graph.json waehlen."
        } else {
            Warn "Route-Ausgabe nicht eindeutig."
        }
    } catch {
        Err "Route-Test fehlgeschlagen: $($_.Exception.Message)"
    }
} else {
    Err "graph.json konnte nicht erzeugt werden."
}

# 5) vJoy Quick Check
Step "vJoy Quick-Check"
$vjoyDllCandidates = @(
    "C:\Program Files\vJoy\x64\vJoyInterface.dll",
    "C:\Program Files (x86)\vJoy\x64\vJoyInterface.dll",
    "C:\Windows\System32\vJoyInterface.dll"
)
$vjoyFound = $false
foreach ($p in $vjoyDllCandidates) {
    if (Test-Path $p) { Ok "vJoy DLL gefunden: $p"; $vjoyFound = $true; break }
}
if (-not $vjoyFound) {
    Warn "vJoy DLL nicht gefunden."
    Warn "=> vJoy installieren/repair, Device 1 aktivieren."
}

# 6) Cargo Check
Step "Cargo Check"
$cargo = Get-Command cargo -ErrorAction SilentlyContinue
if ($cargo) { Ok "cargo vorhanden." } else { Warn "cargo nicht im PATH." }

Step "Diagnose fertig"
Write-Host "Wenn MMF=OK und Route found, dann Rust-Live-Loop starten:" -ForegroundColor Green
Write-Host "cargo run --release -- --hashfs-sectors `"C:\temp\ets2_sectors`" --start <START> --goal <GOAL> --vjoy-device 1 -v"
