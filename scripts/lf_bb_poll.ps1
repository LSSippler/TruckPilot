# Lane-Follower Blackboard Poll (200 ms default)
# Ctrl+C zum Beenden.
#
# Usage:
#   .\scripts\lf_bb_poll.ps1
#   .\scripts\lf_bb_poll.ps1 -IntervalMs 200

param(
    [int]$IntervalMs = 200,
    [string]$DaemonUrl = "ws://127.0.0.1:8765"
)

$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
$bbQuery = Join-Path $root "target\release\blackboard-query.exe"
if (-not (Test-Path $bbQuery)) {
    Write-Error "$bbQuery not found - run 'cargo build-release' first."
    exit 1
}

$keys = @(
    "lane_follower.nearest_seg_idx",
    "lane_follower.nearest_seg_t",
    "lane_follower.lateral_source",
    "lane_follower.lateral_dist_signed",
    "lane_follower.heading_diff_deg",
    "lane_follower.heading_to_lookahead_deg",
    "lane_follower.steering_cmd",
    "lane_follower.steering_curvature",
    "lane_follower.lookahead_distance_m",
    "lane_follower.lookahead_remaining_m",
    "lane_follower.lookahead_seg_idx",
    "lane_follower.lookahead_hop_count",
    "lane_follower.lookahead_status",
    "lane_follower.junction_detected",
    "lane_follower.junction_phase",
    # Junction-Snap-Diag (lane-keeper route-guard):
    "lane_keeper.nearest_seg_idx",
    "lane_keeper.sel_path",
    "lane_keeper.route_set_size",
    "lane_keeper.route_set_has_nearest",
    "lane_keeper.route_set_has_lookahead",
    "lane_keeper.chain_broken"
) -join ","

$bbLinePattern = '^([\w_\.]+)\s*=\s*(.*)$'

Write-Host "lf_bb_poll: every ${IntervalMs}ms, Ctrl+C to stop"
Write-Host ""

while ($true) {
    $ts = Get-Date -Format "HH:mm:ss.fff"
    try {
        $raw = & $bbQuery --url $DaemonUrl --keys $keys 2>&1
        $parts = @($ts)
        foreach ($line in ($raw -split "`n")) {
            $line = $line.Trim()
            if ($line -match $bbLinePattern) {
                $k = $Matches[1]
                $v = $Matches[2].Trim()
                if ($k -like "lane_follower.*") {
                    $short = $k.Substring("lane_follower.".Length)
                    $parts += ("{0}={1}" -f $short, $v)
                } elseif ($k -like "lane_keeper.*") {
                    $short = "lk." + $k.Substring("lane_keeper.".Length)
                    $parts += ("{0}={1}" -f $short, $v)
                }
            }
        }
        Write-Host ($parts -join " | ")
    } catch {
        Write-Host "$ts | ERROR: $($_.Exception.Message)"
    }
    Start-Sleep -Milliseconds $IntervalMs
}
