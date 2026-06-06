# Phase 2h-Diag3 — Null-Pfad (steerO/shm=0) + herr-Sprung Logger
# Sicherheitskritisch: Truck faehrt bei steerO=0 UNGELENKT geradeaus.
# NUR sehr langsam (~10 km/h) fahren, Hand am Lenkrad, SOFORT eingreifen wenn
# steerO/shm auf 0 geht. Daemon + ETS2 (Motor an, engaged) muessen laufen.
#
# Frage A (warum steerO/shm=0): lk_none + null_cause + stage_seen + arb_winner.
#   Erwartung: bei herr>1.047 (60 Grad) -> heading_stage=AutoReplan ->
#   lk_none=true, null_cause=heading_stage_autoreplan, arb_winner=none_legacy_default
#   ODER lane_follower (Prio 0) -> arb_value=0 -> shm=0.
# Frage B (warum herr=1.156 am Source-Wechsel): nearest_seg / final_seg / target
#   vor und nach dem Wechsel catmull->spline (src_chg=1).

$ErrorActionPreference = "Stop"
$bb = ".\target\release\blackboard-query.exe"
$keys = @(
    "autopilot.state",
    # ── Frage A: Null-Pfad ──
    "lane_keeper.heading_error_rad",
    "lane_keeper.error_rad",
    "lane_keeper.steer_raw",
    "lane_keeper.steering_out",
    "lane_keeper.returned_none",
    "lane_keeper.null_steer_cause",
    "lane_keeper.skip_reason",
    "lane_keeper.stage_seen",
    "state.heading_stage",
    "state.heading_diff_deg",
    "arbitration.steering_winner_plugin",
    "arbitration.steering_winner_priority",
    "arbitration.steering_offer_count",
    "arbitration.steering_value",
    "scs_sdk_output.steer_written",
    # ── Frage B: herr-Sprung am Source-Wechsel ──
    "lane_keeper.lateral_source",
    "lane_keeper.source_changed",
    "lane_keeper.nearest_segment_id",
    "lane_keeper.nearest_seg_hop",
    "lane_keeper.nearest_heading_filter_applied",
    "lane_keeper.lookahead_final_seg_id",
    "lane_keeper.lookahead_m",
    "lane_keeper.target_heading",
    "lane_keeper.steer_target_xz",
    "lane_keeper.truck_x",
    "lane_keeper.truck_z"
) -join ","

$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$outDir = "outputs\$(Get-Date -Format 'yyyy-MM-dd')"
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
$csv = Join-Path $outDir "big8_diag3_nullpath_$stamp.csv"

"t,state,herr,err,raw,steer_out,lk_none,null_cause,skip,stage_seen,heading_stage,head_deg,arb_winner,arb_prio,arb_offers,arb_value,shm,lat_src,src_chg,near_seg,near_hop,near_hf,final_seg,la_m,tgt_head,steer_tgt_xz,tx,tz" | Out-File -FilePath $csv -Encoding utf8

Write-Host "Logging nach $csv  (Strg+C zum Stoppen)" -ForegroundColor Cyan
Write-Host "WARNUNG: bei steerO=0 faehrt der Truck UNGELENKT. Hand am Lenkrad!" -ForegroundColor Red
Write-Host ("{0,-6} {1,-7} {2,-9} {3,-9} {4,-7} {5,-6} {6,-22} {7,-13} {8,-8} {9}" -f `
    "t","state","herr","steerO","shm","none","null_cause","stage","arb_win","src") -ForegroundColor Gray

$t0 = Get-Date
while ($true) {
    $t = "{0:N1}" -f ((Get-Date) - $t0).TotalSeconds
    $raw = & $bb --keys $keys 2>$null

    $h = @{}
    foreach ($line in $raw) {
        if ($line -match '^\s*([\w.]+)\s*=\s*(.+?)\s*$') { $h[$matches[1]] = $matches[2] }
    }

    $vals = @(
        $t, $h["autopilot.state"],
        $h["lane_keeper.heading_error_rad"], $h["lane_keeper.error_rad"],
        $h["lane_keeper.steer_raw"], $h["lane_keeper.steering_out"],
        $h["lane_keeper.returned_none"], $h["lane_keeper.null_steer_cause"],
        $h["lane_keeper.skip_reason"], $h["lane_keeper.stage_seen"],
        $h["state.heading_stage"], $h["state.heading_diff_deg"],
        $h["arbitration.steering_winner_plugin"], $h["arbitration.steering_winner_priority"],
        $h["arbitration.steering_offer_count"], $h["arbitration.steering_value"],
        $h["scs_sdk_output.steer_written"],
        $h["lane_keeper.lateral_source"], $h["lane_keeper.source_changed"],
        $h["lane_keeper.nearest_segment_id"], $h["lane_keeper.nearest_seg_hop"],
        $h["lane_keeper.nearest_heading_filter_applied"],
        $h["lane_keeper.lookahead_final_seg_id"], $h["lane_keeper.lookahead_m"],
        $h["lane_keeper.target_heading"], $h["lane_keeper.steer_target_xz"],
        $h["lane_keeper.truck_x"], $h["lane_keeper.truck_z"]
    )
    ($vals -join ",") | Out-File -FilePath $csv -Append -Encoding utf8

    $row = "{0,-6} {1,-7} {2,-9} {3,-9} {4,-7} {5,-6} {6,-22} {7,-13} {8,-8} {9}" -f `
        $t, $h["autopilot.state"], $h["lane_keeper.heading_error_rad"], `
        $h["lane_keeper.steering_out"], $h["scs_sdk_output.steer_written"], `
        $h["lane_keeper.returned_none"], $h["lane_keeper.null_steer_cause"], `
        $h["state.heading_stage"], $h["arbitration.steering_winner_plugin"], `
        $h["lane_keeper.lateral_source"]

    # Rot hervorheben wenn der gefaehrliche Null-Zustand aktiv ist.
    if ($h["scs_sdk_output.steer_written"] -eq "0.000000" -and $h["autopilot.state"] -eq "Active") {
        Write-Host $row -ForegroundColor Red
    } elseif ($h["autopilot.state"] -eq "Active" -or $h["autopilot.state"] -eq "Engaged") {
        Write-Host $row -ForegroundColor Green
    } else { Write-Host $row }

    Start-Sleep -Milliseconds 200
}
