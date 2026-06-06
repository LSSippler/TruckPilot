# Phase 2h-Diag2 — Steer-Herkunft + Vollausschlag-Pendeln Logger
# Loggt den ECHTEN Treiber (lane_keeper @ prio 50) + arbitrierten SHM-Wert
# (scs_sdk_output.steer_written) + lane_follower (prio 0, Vergleich).
# Daemon + ETS2 (Motor an, Autopilot engaged) muessen laufen. Strg+C stoppt.
#
# Frage A (konstanter steer 0.8281): steer_p + steer_i + steer_d vs steer_unclamped
#   vs output_clamp_active. Wenn unclamped<1.0 und p+i+d==steer_raw -> echter Regler,
#   kein Clamp. heading_error konstant waehrend lat sinkt -> heading-only/ferner Lookahead.
# Frage B (Vollausschlag): lateral_source / lane_offset_applied_m / offset_delta_m /
#   source_changed / steer_target_xz an den Spruengen.

$ErrorActionPreference = "Stop"
$bb = ".\target\release\blackboard-query.exe"
$keys = @(
    "autopilot.state",
    "autopilot.fault_reason",
    "lane_keeper.active",
    "lane_keeper.skip_reason",
    # ── Frage A: Steer-Herkunft (PID-Glieder) ──
    "lane_keeper.heading_error_rad",
    "lane_keeper.effective_err_rad",
    "lane_keeper.steer_p_term",
    "lane_keeper.steer_i_term",
    "lane_keeper.steer_d_term",
    "lane_keeper.steer_unclamped",
    "lane_keeper.steer_raw",
    "lane_keeper.steer_output_clamp_active",
    "lane_keeper.steering_out",
    "lane_keeper.steering_rate_limited",
    # ── Frage A: Lookahead-Verhalten ──
    "lane_keeper.lookahead_m",
    "lane_keeper.target_heading",
    "lane_keeper.truck_heading",
    "lane_keeper.truck_lat_vs_centerline_m",
    "lane_keeper.steer_target_xz",
    # ── Frage B: Uebergangs-Spruenge ──
    "lane_keeper.lateral_source",
    "lane_keeper.lane_offset_applied_m",
    "lane_keeper.offset_delta_m",
    "lane_keeper.source_changed",
    # ── echter arbitrierter SHM-Wert + Vergleich Bystander ──
    "scs_sdk_output.steer_written",
    "lane_follower.steering_cmd"
) -join ","

$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$outDir = "outputs\$(Get-Date -Format 'yyyy-MM-dd')"
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
$csv = Join-Path $outDir "big8_diag2_steer_$stamp.csv"

"t,state,fault,lk_act,lk_skip,herr,eff_err,p,i,d,unclamp,raw,oclamp,steer_out,ratelim,la_m,tgt_head,truck_head,lat_vs_cl,steer_tgt_xz,lat_src,offset,offset_d,src_chg,shm_steer,lf_cmd" | Out-File -FilePath $csv -Encoding utf8

Write-Host "Logging nach $csv  (Strg+C zum Stoppen)" -ForegroundColor Cyan
Write-Host ("{0,-6} {1,-8} {2,-6} {3,-9} {4,-9} {5,-7} {6,-7} {7,-7} {8,-8} {9}" -f `
    "t","state","lk","herr","p","i","raw","steerO","shm","src") -ForegroundColor Gray

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
        $h["autopilot.state"], $h["autopilot.fault_reason"],
        $h["lane_keeper.active"], $h["lane_keeper.skip_reason"],
        $h["lane_keeper.heading_error_rad"], $h["lane_keeper.effective_err_rad"],
        $h["lane_keeper.steer_p_term"], $h["lane_keeper.steer_i_term"], $h["lane_keeper.steer_d_term"],
        $h["lane_keeper.steer_unclamped"], $h["lane_keeper.steer_raw"],
        $h["lane_keeper.steer_output_clamp_active"], $h["lane_keeper.steering_out"],
        $h["lane_keeper.steering_rate_limited"],
        $h["lane_keeper.lookahead_m"], $h["lane_keeper.target_heading"], $h["lane_keeper.truck_heading"],
        $h["lane_keeper.truck_lat_vs_centerline_m"], $h["lane_keeper.steer_target_xz"],
        $h["lane_keeper.lateral_source"], $h["lane_keeper.lane_offset_applied_m"],
        $h["lane_keeper.offset_delta_m"], $h["lane_keeper.source_changed"],
        $h["scs_sdk_output.steer_written"], $h["lane_follower.steering_cmd"]
    )
    ($vals -join ",") | Out-File -FilePath $csv -Append -Encoding utf8

    $row = "{0,-6} {1,-8} {2,-6} {3,-9} {4,-9} {5,-7} {6,-7} {7,-7} {8,-8} {9}" -f `
        $t, $h["autopilot.state"], $h["lane_keeper.active"], `
        $h["lane_keeper.heading_error_rad"], $h["lane_keeper.steer_p_term"], `
        $h["lane_keeper.steer_i_term"], $h["lane_keeper.steer_raw"], `
        $h["lane_keeper.steering_out"], $h["scs_sdk_output.steer_written"], `
        $h["lane_keeper.lateral_source"]

    if ($h["autopilot.state"] -eq "Fault") { Write-Host $row -ForegroundColor Red }
    elseif ($h["autopilot.state"] -eq "Active" -or $h["autopilot.state"] -eq "Engaged") { Write-Host $row -ForegroundColor Green }
    else { Write-Host $row }

    Start-Sleep -Milliseconds 200
}
