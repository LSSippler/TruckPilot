# Big-8 Lane-Keeper Testfahrt — Logger (echte BB-Keys)
# Daemon + ETS2 (Truck in Welt, Motor an) muessen laufen. Autopilot engaged.
# Strg+C zum Stoppen.

$ErrorActionPreference = "Stop"
$bb = ".\target\release\blackboard-query.exe"
$keys = @(
    "autopilot.state",
    "autopilot.fault_reason",
    "autopilot.engage_mode",
    "lane_keeper.active",
    "lane_keeper.skip_reason",
    "router.active",
    "router.waypoint_count",
    "router.path_total_distance_m",
    "router.snap_stability",
    "router.last_snap_dist",
    "lane_follower.steering_cmd",
    "lane_follower.steering_filtered",
    "lane_follower.truck_x",
    "lane_follower.truck_z",
    "lane_follower.truck_heading_deg",
    "lane_follower.lateral_dist_signed",
    "lane_follower.heading_to_lookahead_deg"
) -join ","

$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$outDir = "outputs\$(Get-Date -Format 'yyyy-MM-dd')"
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
$csv = Join-Path $outDir "big8_lanekeeper_drive_$stamp.csv"

"t,state,fault,mode,lk_active,lk_skip,r_active,wp_count,r_dist,snap_stab,snap_dist,steer_cmd,steer_filt,x,z,heading,lat,head_to_la" | Out-File -FilePath $csv -Encoding utf8

Write-Host "Logging nach $csv  (Strg+C zum Stoppen)" -ForegroundColor Cyan
Write-Host ("{0,-7} {1,-10} {2,-14} {3,-7} {4,-6} {5,-9} {6,-8} {7}" -f "t","state","fault","lk_act","wp","steer","lat","pos") -ForegroundColor Gray

$t0 = Get-Date
while ($true) {
    $t = "{0:N1}" -f ((Get-Date) - $t0).TotalSeconds
    $raw = & $bb --keys $keys 2>$null

    $h = @{}
    foreach ($line in $raw) {
        if ($line -match '^\s*([\w.]+)\s*=\s*(.+?)\s*$') { $h[$matches[1]] = $matches[2] }
    }

    $vals = @(
        $t,
        $h["autopilot.state"], $h["autopilot.fault_reason"], $h["autopilot.engage_mode"],
        $h["lane_keeper.active"], $h["lane_keeper.skip_reason"],
        $h["router.active"], $h["router.waypoint_count"], $h["router.path_total_distance_m"],
        $h["router.snap_stability"], $h["router.last_snap_dist"],
        $h["lane_follower.steering_cmd"], $h["lane_follower.steering_filtered"],
        $h["lane_follower.truck_x"], $h["lane_follower.truck_z"],
        $h["lane_follower.truck_heading_deg"], $h["lane_follower.lateral_dist_signed"],
        $h["lane_follower.heading_to_lookahead_deg"]
    )
    ($vals -join ",") | Out-File -FilePath $csv -Append -Encoding utf8

    $pos = if ($h["lane_follower.truck_x"] -and $h["lane_follower.truck_z"]) {
        "$([math]::Round([double]$h['lane_follower.truck_x'])),$([math]::Round([double]$h['lane_follower.truck_z']))"
    } else { "-" }
    $row = "{0,-7} {1,-10} {2,-14} {3,-7} {4,-6} {5,-9} {6,-8} {7}" -f `
        $t, $h["autopilot.state"], $h["autopilot.fault_reason"], $h["lane_keeper.active"], `
        $h["router.waypoint_count"], $h["lane_follower.steering_cmd"], $h["lane_follower.lateral_dist_signed"], $pos

    if ($h["autopilot.state"] -eq "Fault") { Write-Host $row -ForegroundColor Red }
    elseif ($h["autopilot.state"] -eq "Active" -or $h["autopilot.state"] -eq "Engaged") { Write-Host $row -ForegroundColor Green }
    else { Write-Host $row }

    Start-Sleep -Milliseconds 200
}