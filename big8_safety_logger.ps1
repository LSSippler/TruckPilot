# Phase 2h-Safety — Live-Test Logger: bremst der Truck bei Lenkautoritaets-Verlust?
# Erwartung: bei herr-Spike geht steerO NICHT mehr still auf 0 mit gehaltener Speed,
# sondern shm_brake > 0, safety_state meldet den Zustand, Ist-Speed faellt. Nach
# Erholung (Stage Normal) safety_state=normal, shm_brake=0, Truck faehrt weiter.
# Sicherer als zuvor (Truck bremst), trotzdem: ~15 km/h, Hand am Lenkrad.
# Daemon + ETS2 (Motor an, engaged) muessen laufen. Strg+C stoppt.

$ErrorActionPreference = "Stop"
$bb = ".\target\release\blackboard-query.exe"
$keys = @(
    "autopilot.state",
    "state.heading_stage",
    "state.heading_diff_deg",
    "lane_keeper.heading_error_rad",
    "lane_keeper.safety_state",
    "lane_keeper.safety_brake",
    "lane_keeper.safety_autoreplan_secs",
    "lane_keeper.steering_suppressed",
    "lane_keeper.steering_out",
    "lane_keeper.lateral_source",
    "lane_keeper.source_changed",
    "arbitration.steering_winner_plugin",
    "scs_sdk_output.steer_written",
    "scs_sdk_output.brake_written",
    "scs_sdk_output.throttle_written",
    "lane_keeper.truck_speed_ms",
    "autopilot.disengage_requested"
) -join ","

$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$outDir = "outputs\$(Get-Date -Format 'yyyy-MM-dd')"
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
$csv = Join-Path $outDir "big8_safety_brake_$stamp.csv"

"t,state,stage,head_deg,herr,safety_state,safety_brake,ar_secs,steer_supp,steer_out,lat_src,src_chg,arb_win,shm_steer,shm_brake,shm_thr,speed_ms,diseng_req" | Out-File -FilePath $csv -Encoding utf8

Write-Host "Logging nach $csv  (Strg+C zum Stoppen)" -ForegroundColor Cyan
Write-Host "Erwartung: bei Spike -> shm_brake>0 + safety_state gesetzt + speed faellt (NICHT still steerO=0)." -ForegroundColor Yellow
Write-Host ("{0,-6} {1,-7} {2,-12} {3,-7} {4,-32} {5,-7} {6,-7} {7,-8} {8}" -f `
    "t","state","stage","herr","safety_state","s_brk","shm_brk","speed","diseng") -ForegroundColor Gray

$t0 = Get-Date
while ($true) {
    $t = "{0:N1}" -f ((Get-Date) - $t0).TotalSeconds
    $raw = & $bb --keys $keys 2>$null

    $h = @{}
    foreach ($line in $raw) {
        if ($line -match '^\s*([\w.]+)\s*=\s*(.+?)\s*$') { $h[$matches[1]] = $matches[2] }
    }

    $vals = @(
        $t, $h["autopilot.state"], $h["state.heading_stage"], $h["state.heading_diff_deg"],
        $h["lane_keeper.heading_error_rad"], $h["lane_keeper.safety_state"],
        $h["lane_keeper.safety_brake"], $h["lane_keeper.safety_autoreplan_secs"],
        $h["lane_keeper.steering_suppressed"], $h["lane_keeper.steering_out"],
        $h["lane_keeper.lateral_source"], $h["lane_keeper.source_changed"],
        $h["arbitration.steering_winner_plugin"],
        $h["scs_sdk_output.steer_written"], $h["scs_sdk_output.brake_written"],
        $h["scs_sdk_output.throttle_written"], $h["lane_keeper.truck_speed_ms"],
        $h["autopilot.disengage_requested"]
    )
    ($vals -join ",") | Out-File -FilePath $csv -Append -Encoding utf8

    $speedKmh = if ($h["lane_keeper.truck_speed_ms"]) { "{0:N0}" -f ([double]$h["lane_keeper.truck_speed_ms"] * 3.6) } else { "-" }
    $row = "{0,-6} {1,-7} {2,-12} {3,-7} {4,-32} {5,-7} {6,-7} {7,-8} {8}" -f `
        $t, $h["autopilot.state"], $h["state.heading_stage"], `
        $h["lane_keeper.heading_error_rad"], $h["lane_keeper.safety_state"], `
        $h["lane_keeper.safety_brake"], $h["scs_sdk_output.brake_written"], `
        "$speedKmh kmh", $h["autopilot.disengage_requested"]

    $brk = $h["scs_sdk_output.brake_written"]
    if ($brk -and [double]$brk -gt 0.0) { Write-Host $row -ForegroundColor Yellow }       # Safety-Bremsung aktiv
    elseif ($h["autopilot.state"] -eq "Active" -or $h["autopilot.state"] -eq "Engaged") { Write-Host $row -ForegroundColor Green }
    elseif ($h["autopilot.state"] -eq "Fault") { Write-Host $row -ForegroundColor Red }
    else { Write-Host $row }

    Start-Sleep -Milliseconds 200
}
