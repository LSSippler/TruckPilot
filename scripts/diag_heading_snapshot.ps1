# Heading Diagnostic Snapshot — Phase 1a Blocker
# Takes N snapshots every 500ms and dumps all lane_follower BB keys
# to help distinguish Hypothesis A (position/dist problem) vs B/C (heading convention).
# Compatible with Windows PowerShell 5.1+.
#
# Usage:
#   .\scripts\diag_heading_snapshot.ps1
#   .\scripts\diag_heading_snapshot.ps1 -Snapshots 20 -OutFile outputs\2026-05-25\heading_diag_snapshot.txt

param(
    [int]$Snapshots    = 10,
    [string]$OutFile   = "outputs\2026-05-25\heading_diag_snapshot.txt",
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

# Keys: all heading-relevant + position + lookahead
$keys = (
    "lane_follower.status",
    "lane_follower.nearest_seg_idx",
    "lane_follower.nearest_seg_dist_m",
    "lane_follower.nearest_seg_t",
    "lane_follower.heading_deg",
    "lane_follower.truck_heading_deg",
    "lane_follower.heading_diff_deg",
    "lane_follower.truck_x",
    "lane_follower.truck_z",
    "lane_follower.lookahead_x",
    "lane_follower.lookahead_z",
    "lane_follower.lookahead_status"
) -join ","

$lines = @()
$lines += "# Heading Diagnostic Snapshot — Phase 1a"
$lines += "# Time: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
$lines += "# Expected: telemetry 0.8091681599617004 rad = 46.36 deg (if bug active)"
$lines += "# STOP-check: dist_m > 50 -> Hypothesis A (position), < 5 -> Hypothesis B/C (heading)"
$lines += ""
$lines += ("| {0,-3} | {1,-8} | {2,-6} | {3,-8} | {4,-6} | {5,-8} | {6,-8} | {7,-8} | {8,-10} | {9,-10} | {10,-6} |" -f `
    "N", "status", "idx", "dist_m", "t", "road_hdg", "truck_hdg", "diff_deg", "truck_x", "truck_z", "la_stat")
$lines += ("| {0,-3} | {1,-8} | {2,-6} | {3,-8} | {4,-6} | {5,-8} | {6,-8} | {7,-8} | {8,-10} | {9,-10} | {10,-6} |" -f `
    "---", "--------", "------", "--------", "------", "--------", "--------", "--------", "----------", "----------", "------")

Write-Host "Taking $Snapshots snapshots (0.5s interval)..."
Write-Host ""

for ($i = 1; $i -le $Snapshots; $i++) {
    $raw = & $bbQuery --url $DaemonUrl --keys $keys 2>&1
    $kv = @{}
    foreach ($line in ($raw -split "`n")) {
        $line = $line.Trim()
        if ($line -match "^([\w_\.]+)\s*=\s*(.*)$") {
            $kv[$Matches[1]] = $Matches[2].Trim()
        }
    }

    $row = ("| {0,-3} | {1,-8} | {2,-6} | {3,-8} | {4,-6} | {5,-8} | {6,-8} | {7,-8} | {8,-10} | {9,-10} | {10,-6} |" -f `
        $i,
        (Get-OrDefault $kv "lane_follower.status"),
        (Get-OrDefault $kv "lane_follower.nearest_seg_idx"),
        (Get-OrDefault $kv "lane_follower.nearest_seg_dist_m"),
        (Get-OrDefault $kv "lane_follower.nearest_seg_t"),
        (Get-OrDefault $kv "lane_follower.heading_deg"),
        (Get-OrDefault $kv "lane_follower.truck_heading_deg"),
        (Get-OrDefault $kv "lane_follower.heading_diff_deg"),
        (Get-OrDefault $kv "lane_follower.truck_x"),
        (Get-OrDefault $kv "lane_follower.truck_z"),
        (Get-OrDefault $kv "lane_follower.lookahead_status"))

    Write-Host $row
    $lines += $row
    Start-Sleep -Milliseconds 500
}

$lines += ""
$dist = Get-OrDefault $kv "lane_follower.nearest_seg_dist_m" "?"
$status = Get-OrDefault $kv "lane_follower.status" "?"
$lines += "# Last dist_m=$dist  status=$status"
$lines += ""

# Diagnosis
$lines += "## Diagnosis hint"
if ($dist -ne "?" -and [double]::TryParse($dist, [ref]$null)) {
    $dval = [double]$dist
    if ($dval -gt 50.0) {
        $lines += "# -> HYPOTHESIS A likely: dist_m=$dist > 50m. SplineIndex miss or position error."
        $lines += "#    Check truck_x/truck_z vs known road coordinates."
        Write-Host ""
        Write-Host "DIAGNOSIS: Hypothesis A — dist_m=$dist > 50m (position problem)"
    } elseif ($dval -lt 5.0) {
        $lines += "# -> HYPOTHESIS B/C likely: dist_m=$dist < 5m. Heading convention mismatch or tangent error."
        $lines += "#    Check road_hdg vs truck_hdg convention (SCS euler vs North-0-CW)."
        Write-Host ""
        Write-Host "DIAGNOSIS: Hypothesis B/C — dist_m=$dist < 5m (heading convention problem)"
    } else {
        $lines += "# -> AMBIGUOUS: dist_m=$dist in (5,50) range. Run live-compare for full diagnosis."
        Write-Host ""
        Write-Host "DIAGNOSIS: AMBIGUOUS — dist_m=$dist in (5,50) range"
    }
} else {
    $lines += "# -> Could not parse dist_m. Daemon not running or plugin not loaded."
    Write-Host "DIAGNOSIS: No data — check daemon is running and lane-follower plugin is active."
}

$lines | Out-File -Encoding utf8 -FilePath $OutFile
Write-Host ""
Write-Host "Output: $OutFile"
