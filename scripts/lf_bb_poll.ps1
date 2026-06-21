# Lane-Follower Blackboard Poll (200 ms default)
# Ctrl+C zum Beenden.
#
# Usage:
#   .\scripts\lf_bb_poll.ps1
#   .\scripts\lf_bb_poll.ps1 -IntervalMs 200

param(
    [int]$IntervalMs = 200,
    [string]$DaemonUrl = "ws://127.0.0.1:8765",
    [string]$LogFile = ""
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
    "lane_keeper.chain_broken",
    # Junction-Guards (neueste Commits):
    "lane_keeper.junction_failsafe_active",
    "lane_keeper.dual_cw_guard_active",
    "lane_keeper.chosen_segment_is_navcurve",
    "lane_keeper.junction_navcurve_forced",
    "lk.force_candidates",
    "lk.force_proj_ok",
    "lk.force_t_filtered",
    "lk.force_best_dist_m",
    "lane_keeper.total_route_navcurve_count",
    "lane_keeper.navcurve_by_from_to_count",
    "lane_keeper.reanchor_scan_window",
    "lane_keeper.engage_max_lateral_m",
    # Disengage-Ursache (warum schaltet er an der Junction ab):
    "lane_keeper.stage",
    "lane_keeper.stage_block_reason",
    "lane_keeper.fallback_reason",
    "lane_keeper.skip_reason",
    "lane_keeper.safety_state",
    "lane_keeper.safety_autoreplan_secs",
    "lane_keeper.mismatch_herr_deg",
    "lane_keeper.heading_error_deg",
    # Lateral-Offset-Diagnose (halb-daneben-Debug):
    "lane_keeper.lateral_source",
    "lane_keeper.xtrack_e_lat_m",
    "lane_keeper.truck_lat_vs_centerline_m",
    "lane_keeper.lane_offset_applied_m",
    "lane_keeper.xtrack_integ",
    "lane_keeper.xtrack_p_rad",
    "lane_keeper.xtrack_i_rad",
    "lane_keeper.xtrack_contribution_rad",
    # Junction NavCurve routing diagnosis:
    "lane_keeper.nearest_route_query_hits",
    "lane_keeper.nearest_route_gate_rejected",
    "lane_keeper.nearest_route_best_dist_m",
    "lane_keeper.nearest_route_seg_set_size",
    "lane_keeper.nearest_route_best_hop",
    "lane_keeper.nearest_route_heading_diff_deg",
    # Lanes-in-direction (5.625m-Bug diagnose):
    "lane_keeper.spline_seg_lanes_in_direction",
    "lane_keeper.catmull_seg_lanes_in_direction",
    "lane_keeper.nearest_seg_lanes_in_direction",
    "autopilot.state"
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
                } elseif ($k -like "autopilot.*") {
                    $short = "ap." + $k.Substring("autopilot.".Length)
                    $parts += ("{0}={1}" -f $short, $v)
                }
            }
        }
        $line = $parts -join " | "
        Write-Host $line
        if ($LogFile) { Add-Content -Path $LogFile -Value $line }
    } catch {
        Write-Host "$ts | ERROR: $($_.Exception.Message)"
    }
    Start-Sleep -Milliseconds $IntervalMs
}
