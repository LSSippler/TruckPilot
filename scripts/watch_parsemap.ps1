# watch_parsemap.ps1 - beobachtet den parse-map-Lauf live und misst am Ende die LCC.
# Aufruf aus dem Workspace-Root:
#   .\scripts\watch_parsemap.ps1
# Optional:  -Interval 5   (Poll-Sekunden, Default 5)

param(
    [int]$Interval = 5,
    [string]$Graph = "graph.json",
    [string]$Stats = ".\target\release\truckpilot-graph-stats.exe"
)

$ErrorActionPreference = "SilentlyContinue"

# Ausgangs-Zeitstempel von graph.json merken (um eine NEUE Schreibung zu erkennen)
$baseStamp = (Get-Item $Graph).LastWriteTime
$start     = Get-Date

Write-Host "Beobachte parse-map ... (Strg+C zum Abbrechen)" -ForegroundColor Cyan
Write-Host "Baseline graph.json: $baseStamp" -ForegroundColor DarkGray

while ($true) {
    $p = Get-Process truckpilot-core -ErrorAction SilentlyContinue
    $g = Get-Item $Graph -ErrorAction SilentlyContinue
    $now = Get-Date

    Clear-Host
    Write-Host "=== parse-map Watcher ===  $($now.ToString('HH:mm:ss'))" -ForegroundColor Cyan

    if ($p) {
        $rt  = ($now - $p.StartTime).ToString('hh\:mm\:ss')
        $cpu = [math]::Round($p.CPU, 1)
        $ram = [math]::Round($p.WorkingSet64 / 1MB)
        Write-Host ("Prozess  : PID {0}  Runtime {1}  CPU {2}s  RAM {3} MB" -f $p.Id, $rt, $cpu, $ram) -ForegroundColor Green
    } else {
        Write-Host "Prozess  : NICHT mehr aktiv (beendet oder noch nicht gestartet)" -ForegroundColor Yellow
    }

    if ($g) {
        $mb = [math]::Round($g.Length / 1MB)
        $fresh = $g.LastWriteTime -gt $baseStamp
        if ($fresh) { $tag = "NEU GESCHRIEBEN" } else { $tag = "noch alt (Baseline)" }
        if ($fresh) { $col = "Green" }           else { $col = "DarkGray" }
        Write-Host ("graph.json: {0} MB  geaendert {1}  [{2}]" -f $mb, $g.LastWriteTime.ToString('HH:mm:ss'), $tag) -ForegroundColor $col
    }

    # Fertig-Bedingung: graph.json wurde neu geschrieben UND Prozess ist weg
    if ($g -and ($g.LastWriteTime -gt $baseStamp) -and (-not $p)) {
        Write-Host ""
        Write-Host "FERTIG - graph.json wurde neu geschrieben." -ForegroundColor Cyan
        Write-Host "Gesamtdauer Watcher: $(((Get-Date) - $start).ToString('hh\:mm\:ss'))" -ForegroundColor DarkGray
        Write-Host ""
        if (Test-Path $Stats) {
            Write-Host "=== LCC / Connectivity (truckpilot-graph-stats) ===" -ForegroundColor Cyan
            & $Stats --graph $Graph
        } else {
            Write-Host "graph-stats nicht gefunden unter $Stats - bitte manuell messen." -ForegroundColor Yellow
        }
        break
    }

    Start-Sleep -Seconds $Interval
}
