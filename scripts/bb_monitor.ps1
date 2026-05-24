# BB Monitor — Phase 1a Live-Test
# Polls lane_follower.* Blackboard keys at 1 Hz and writes a timestamped log.
#
# Usage:
#   .\scripts\bb_monitor.ps1 -OutFile outputs\2026-05-24\phase_1a_livetest_highway.txt -Duration 30
#   .\scripts\bb_monitor.ps1 -OutFile outputs\2026-05-24\phase_1a_livetest_junctions.txt -Duration 120

param(
    [Parameter(Mandatory=$true)]
    [string]$OutFile,

    [Parameter(Mandatory=$false)]
    [int]$Duration = 30,          # seconds; 0 = run until Ctrl+C

    [Parameter(Mandatory=$false)]
    [string]$DaemonUrl = "ws://127.0.0.1:8765"
)

$bbQuery = Join-Path $PSScriptRoot "..\target\release\blackboard-query.exe"
if (-not (Test-Path $bbQuery)) {
    Write-Error "blackboard-query.exe not found at $bbQuery — run 'cargo build-release' first."
    exit 1
}

$keys = @(
    "lane_follower.nearest_seg_idx",
    "lane_follower.nearest_seg_dist_m",
    "lane_follower.nearest_seg_t",
    "lane_follower.heading_deg",
    "lane_follower.truck_heading_deg",
    "lane_follower.heading_diff_deg",
    "lane_follower.status"
) -join ","

$dir = Split-Path -Parent $OutFile
if ($dir -and -not (Test-Path $dir)) {
    New-Item -ItemType Directory -Force $dir | Out-Null
}

$header = "# BB Monitor — Phase 1a lane_follower — $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
$header | Out-File -Encoding utf8 -FilePath $OutFile
"# Duration: $($Duration)s | Keys: $keys" | Out-File -Encoding utf8 -Append -FilePath $OutFile
"" | Out-File -Encoding utf8 -Append -FilePath $OutFile

Write-Host "BB Monitor started. Output: $OutFile"
Write-Host "Press Ctrl+C to stop early."
Write-Host ""

$tick = 0
$start = [datetime]::Now
$infinite = ($Duration -eq 0)

while ($infinite -or (([datetime]::Now - $start).TotalSeconds -lt $Duration)) {
    $ts = "{0:HH:mm:ss.fff}" -f [datetime]::Now
    $elapsed = [int](([datetime]::Now - $start).TotalSeconds)

    try {
        $raw = & $bbQuery --url $DaemonUrl --keys $keys 2>&1
        # raw output:  "N key(s):\n  key = value\n  ..."
        $kvPairs = @{}
        foreach ($line in ($raw -split "`n")) {
            $line = $line.Trim()
            if ($line -match "^([\w_\.]+)\s*=\s*(.*)$") {
                $kvPairs[$Matches[1]] = $Matches[2].Trim()
            }
        }
        $snap = "T+{0:D3}s {1} idx={2} dist={3}m t={4} road_hdg={5}° truck_hdg={6}° diff={7}° status={8}" -f `
            $elapsed, $ts,
            ($kvPairs["lane_follower.nearest_seg_idx"] ?? "?"),
            ($kvPairs["lane_follower.nearest_seg_dist_m"] ?? "?"),
            ($kvPairs["lane_follower.nearest_seg_t"] ?? "?"),
            ($kvPairs["lane_follower.heading_deg"] ?? "?"),
            ($kvPairs["lane_follower.truck_heading_deg"] ?? "?"),
            ($kvPairs["lane_follower.heading_diff_deg"] ?? "?"),
            ($kvPairs["lane_follower.status"] ?? "no_reply")
    } catch {
        $snap = "T+{0:D3}s {1} ERROR: {2}" -f $elapsed, $ts, $_.Exception.Message
    }

    $snap | Out-File -Encoding utf8 -Append -FilePath $OutFile
    Write-Host $snap

    $tick++
    Start-Sleep -Seconds 1
}

"" | Out-File -Encoding utf8 -Append -FilePath $OutFile
"# Done — $tick snapshots captured." | Out-File -Encoding utf8 -Append -FilePath $OutFile
Write-Host ""
Write-Host "Done. $tick snapshot(s) written to $OutFile"
