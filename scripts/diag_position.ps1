# Position Diagnostic Snapshot -- Phase 1a Bug 1
# Takes N snapshots every 500ms and dumps all position-related BB keys
# to distinguish SplineIndex coverage vs Telemetry-stall vs Coord-system.
# Compatible with Windows PowerShell 5.1+.
#
# Usage:
#   .\scripts\diag_position.ps1
#   .\scripts\diag_position.ps1 -Snapshots 10 -OutFile outputs\2026-05-25\position_diag.txt

param(
    [int]$Snapshots    = 5,
    [string]$OutFile   = "outputs\2026-05-25\position_diag.txt",
    [string]$DaemonUrl = "ws://127.0.0.1:8765"
)

function Get-OrDefault {
    param($Hash, $Key, $Default = "?")
    if ($Hash.ContainsKey($Key) -and $null -ne $Hash[$Key] -and $Hash[$Key] -ne '') {
        return $Hash[$Key]
    }
    return $Default
}

$bbQuery = Join-Path $PSScriptRoot "..\target\release\blackboard-query.exe"
if (-not (Test-Path $bbQuery)) {
    Write-Error "blackboard-query.exe not found. Run 'cargo build-release' first."
    exit 1
}

$dir = Split-Path -Parent $OutFile
if ($dir -and -not (Test-Path $dir)) {
    New-Item -ItemType Directory -Force $dir | Out-Null
}

# Keys: all position-related
$keys = (
    "lane_follower.status",
    "lane_follower.truck_x",
    "lane_follower.truck_y",
    "lane_follower.truck_z",
    "lane_follower.nearest_seg_idx",
    "lane_follower.nearest_seg_dist_m",
    "lane_follower.nearest_seg_x",
    "lane_follower.nearest_seg_z",
    "lane_follower.nearest_seg_t",
    "lane_follower.heading_deg",
    "lane_follower.truck_heading_deg",
    "lane_follower.heading_diff_deg",
    "lane_follower.tick_count"
) -join ","

$lines = @()
$lines += "# Position Diagnostic Snapshot -- Phase 1a Bug 1"
$lines += "# Time: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
$lines += "# Truck should be STATIONARY (parked) for this test."
$lines += "# Expectation: truck_x/y/z and tick_count should change every snapshot."
$lines += "# If truck_x/y/z identical across all snapshots: telemetry stall."
$lines += "# If truck_x/y/z change but nearest_seg_x/z are 50m+ away: SplineIndex coverage gap."
$lines += ""
$lines += ("| {0,-3} | {1,-8} | {2,-10} | {3,-8} | {4,-10} | {5,-13} | {6,-6} | {7,-12} | {8,-12} | {9,-8} | {10,-6} | {11,-8} |" -f `
    "N", "status", "truck_x", "truck_y", "truck_z", "seg_idx", "dist_m", "seg_x", "seg_z", "t", "t_hdg", "tick")
$lines += ("| {0,-3} | {1,-8} | {2,-10} | {3,-8} | {4,-10} | {5,-13} | {6,-6} | {7,-12} | {8,-12} | {9,-8} | {10,-6} | {11,-8} |" -f `
    "---", "--------", "----------", "--------", "----------", "-------------", "------", "------------", "------------", "--------", "------", "--------")

Write-Host "Taking $Snapshots snapshots (0.5s interval) -- truck should be STATIONARY..."
Write-Host ""

$prevTick = -1
for ($i = 1; $i -le $Snapshots; $i++) {
    $raw = & $bbQuery --url $DaemonUrl --keys $keys 2>&1
    $kv = @{}
    foreach ($line in ($raw -split "`n")) {
        $line = $line.Trim()
        if ($line -match "^([\w_\.]+)\s*=\s*(.*)$") {
            $kv[$Matches[1]] = $Matches[2].Trim()
        }
    }

    $currentTick = Get-OrDefault $kv "lane_follower.tick_count" "?"
    $stallNote = ""
    if ($i -gt 1 -and $currentTick -eq $prevTick -and $currentTick -ne "?") {
        $stallNote = " <<< STALL"
    }

    $row = ("| {0,-3} | {1,-8} | {2,-10} | {3,-8} | {4,-10} | {5,-13} | {6,-6} | {7,-12} | {8,-12} | {9,-8} | {10,-6} | {11,-8} |" -f `
        $i,
        (Get-OrDefault $kv "lane_follower.status"),
        (Get-OrDefault $kv "lane_follower.truck_x"),
        (Get-OrDefault $kv "lane_follower.truck_y"),
        (Get-OrDefault $kv "lane_follower.truck_z"),
        (Get-OrDefault $kv "lane_follower.nearest_seg_idx"),
        (Get-OrDefault $kv "lane_follower.nearest_seg_dist_m"),
        (Get-OrDefault $kv "lane_follower.nearest_seg_x"),
        (Get-OrDefault $kv "lane_follower.nearest_seg_z"),
        (Get-OrDefault $kv "lane_follower.nearest_seg_t"),
        (Get-OrDefault $kv "lane_follower.truck_heading_deg"),
        (Get-OrDefault $kv "lane_follower.tick_count"))

    $display = $row + $stallNote
    Write-Host $display
    $lines += $row
    if ($stallNote) {
        $lines[-1] = $row + $stallNote
    }

    $prevTick = $currentTick
    Start-Sleep -Milliseconds 500
}

# Post-diagnosis
$lastDist = Get-OrDefault $kv "lane_follower.nearest_seg_dist_m" "?"
$lastTx = Get-OrDefault $kv "lane_follower.truck_x" "?"
$lastSx = Get-OrDefault $kv "lane_follower.nearest_seg_x" "?"
$lastSz = Get-OrDefault $kv "lane_follower.nearest_seg_z" "?"

$lines += ""
$lines += ("# Last dist_m=" + $lastDist + " truck_x=" + $lastTx + " seg_x=" + $lastSx + " seg_z=" + $lastSz)
$lines += ""

# Diagnosis hint
$lines += "## Diagnosis hint"
if ($lastDist -ne "?" -and [double]::TryParse($lastDist, [ref]$null)) {
    $dval = [double]$lastDist
    if ($dval -gt 50.0) {
        $lines += ("# -> SplineIndex coverage gap: nearest segment is " + $lastDist + "m away.")
        $lines += "#    Check if truck is on a road that exists in graph.json."
        $lines += "#    Compare truck_x/z vs seg_x/z to see direction of offset."
        Write-Host ""
        Write-Host ("DIAGNOSIS: Coverage Gap -- dist_m=" + $lastDist + "m (above 50m)")
    } elseif ($dval -gt 5.0) {
        $lines += ("# -> Marginal match: dist_m=" + $lastDist + "m. Could be adjacent lane or service road.")
        Write-Host ""
        Write-Host ("DIAGNOSIS: Marginal -- dist_m=" + $lastDist + "m")
    } else {
        $lines += ("# -> Good match: dist_m=" + $lastDist + "m. Heading diff tells the real story.")
        Write-Host ""
        Write-Host ("DIAGNOSIS: Good match -- dist_m=" + $lastDist + "m")
    }
} else {
    $lines += "# -> No dist_m value. Daemon not running or plugin not loaded."
    Write-Host "DIAGNOSIS: No data -- check daemon is running and lane-follower plugin is active."
}

$lines | Out-File -FilePath $OutFile
Write-Host ""
Write-Host "Output: $OutFile"
