# Phase 2h-Diag4 — Wurzel des herr-Sprungs am catmull->spline-Uebergang
# Loggt die Diag3-Segment-Keys, um H-Seg vs H-Walk zu entscheiden.
# Read-only. Safety-Fix bremst jetzt -> gefahrloser, trotzdem ~15 km/h, Hand am Lenkrad.
# Daemon + ETS2 (Motor an, engaged) muessen laufen. Strg+C stoppt.
#
# VERDIKT-AUSWERTUNG (Zeilen VOR/NACH source_changed=1 vergleichen):
#   H-Seg : nearest_segment_id/nearest_seg_hop springt am Wechsel UND steer_target
#           liegt seitlich/hinter (tx,tz)  -> Segment-Auswahl (H-C-artig).
#           Zusatz-Indiz: nearest_segment_id == final_seg (kurzer Lookahead bleibt
#           auf demselben, bereits falschen Segment).
#   H-Walk: nearest_segment_id STABIL, nur lookahead_final_seg_id springt
#           -> Arc-Walk landet auf Knick-Hop.
# Die Spline-Keys werden NUR auf spline-Ticks frisch geschrieben; auf catmull-Ticks
# stehen stale Werte -> immer an lateral_source orientieren.

$ErrorActionPreference = "Stop"
$bb = ".\target\release\blackboard-query.exe"
$keys = @(
    "autopilot.state",
    "state.heading_stage",
    "lane_keeper.heading_error_rad",
    "lane_keeper.error_rad",
    "lane_keeper.lateral_source",
    "lane_keeper.nearest_route_filtered",
    "lane_keeper.nearest_route_seg_set_size",
    "lane_keeper.nearest_route_query_hits",
    "lane_keeper.nearest_route_gate_rejected",
    "lane_keeper.nearest_route_best_dist_m",
    "lane_keeper.nearest_route_best_hop",
    "lane_keeper.nearest_route_heading_diff_deg",
    "lane_keeper.fallback_reason",
    "lane_keeper.fallback_detail",
    "lane_keeper.nearest_discarded_offroute_seg",
    "lane_keeper.nearest_discarded_offroute_hop",
    "lane_keeper.catmull_effective_look_ahead_m",
    "lane_keeper.catmull_curve_factor",
    "lane_keeper.catmull_target_along_m",
    "lane_keeper.catmull_target_lateral_m",
    "lane_keeper.catmull_current_route_idx",
    "lane_keeper.catmull_max_waypoint_idx",
    "lane_keeper.catmull_walk_end",
    "lane_keeper.catmull_total_waypoints",
    "lane_keeper.catmull_max_hops",
    "lane_keeper.source_changed",
    "lane_keeper.nearest_segment_id",
    "lane_keeper.nearest_seg_hop",
    "lane_keeper.nearest_segment_t",
    "lane_keeper.nearest_heading_filter_applied",
    "lane_keeper.lookahead_final_seg_id",
    "lane_keeper.lookahead_final_t",
    "lane_keeper.walk_stopped_at_kink",
    "lane_keeper.walk_kink_deg",
    "lane_keeper.walk_kink_hop",
    "lane_keeper.kink_threshold_deg",
    "lane_keeper.kink_stuck_secs",
    "lane_keeper.walk_hops_checked",
    "lane_keeper.walk_kink_degenerate",
    "lane_keeper.walk_max_kink_deg",
    "lane_keeper.walk_max_kink_hop",
    "lane_keeper.final_internal_kink_deg",
    "lane_keeper.final_tangent_heading_deg",
    "lane_keeper.final_seg_is_prefab",
    "lane_keeper.final_seg_length_m",
    "lane_keeper.prefab_curve_fallback",
    "lane_keeper.internal_kink_over_threshold",
    "lane_keeper.prefab_curve_threshold_deg",
    "lane_keeper.catmull_steer_target_xz",
    "lane_keeper.catmull_centerline_xz",
    "lane_keeper.catmull_truck_xz",
    "lane_keeper.catmull_offset_applied_m",
    "lane_keeper.lookahead_m",
    "lane_keeper.steer_target_xz",
    "lane_keeper.spline_centerline_xz",
    "lane_keeper.diag_truck_xz",
    "lane_keeper.truck_x",
    "lane_keeper.truck_z",
    "lane_keeper.target_heading",
    "lane_keeper.safety_state",
    "scs_sdk_output.brake_written"
) -join ","

$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$outDir = "outputs\$(Get-Date -Format 'yyyy-MM-dd')"
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
$csv = Join-Path $outDir "big8_diag4_segjump_$stamp.csv"

"t,state,stage,herr,err,lat_src,rt_filt,rq_setsz,rq_hits,rq_gaterej,rq_bestdist,rq_besthop,rq_hdiff,fb_reason,fb_detail,disc_seg,disc_hop,look_m,cf,c_along,c_lat,route_idx,max_wp,walk_end,total_wp,max_hops,src_chg,near_seg,near_hop,near_t,near_hf,final_seg,final_t,kink_stop,kink_deg,kink_hop,kink_thr,kink_stuck_s,hops_chk,kink_degen,max_kink,max_kink_hop,final_int_kink,final_tan_head,final_prefab,final_len,pc_fallback,pc_over,pc_thr,cat_tgt_xz,cat_cl_xz,cat_truck_xz,cat_off,la_m,steer_tgt_xz,centerline_xz,diag_truck_xz,truck_x,truck_z,tgt_head,safety_state,shm_brake" | Out-File -FilePath $csv -Encoding utf8

Write-Host "Logging nach $csv  (Strg+C zum Stoppen)" -ForegroundColor Cyan
Write-Host ">>> markiert = lateral_source wechselt catmull->spline (source_changed)" -ForegroundColor Yellow
Write-Host ">>> magenta = catmull->spline-Wechsel | gelb = prefab_curve-Fallback aktiv (intK>Schwelle)" -ForegroundColor Yellow
Write-Host ("{0,-6} {1,-9} {2,-8} {3,-8} {4,-8} {5,-7} {6,-7} {7,-9} {8,-6} {9}" -f `
    "t","stage","herr","near_seg","fin_seg","nahtK","intK","fin_pfab","pc_fb","lat_src") -ForegroundColor Gray

$prevSrc = ""
$t0 = Get-Date
while ($true) {
    $t = "{0:N1}" -f ((Get-Date) - $t0).TotalSeconds
    $raw = & $bb --keys $keys 2>$null

    $h = @{}
    foreach ($line in $raw) {
        if ($line -match '^\s*([\w.]+)\s*=\s*(.+?)\s*$') { $h[$matches[1]] = $matches[2] }
    }

    $vals = @(
        $t, $h["autopilot.state"], $h["state.heading_stage"],
        $h["lane_keeper.heading_error_rad"], $h["lane_keeper.error_rad"],
        $h["lane_keeper.lateral_source"],
        $h["lane_keeper.nearest_route_filtered"],
        $h["lane_keeper.nearest_route_seg_set_size"], $h["lane_keeper.nearest_route_query_hits"],
        $h["lane_keeper.nearest_route_gate_rejected"], $h["lane_keeper.nearest_route_best_dist_m"],
        $h["lane_keeper.nearest_route_best_hop"], $h["lane_keeper.nearest_route_heading_diff_deg"],
        $h["lane_keeper.fallback_reason"], $h["lane_keeper.fallback_detail"],
        $h["lane_keeper.nearest_discarded_offroute_seg"], $h["lane_keeper.nearest_discarded_offroute_hop"],
        $h["lane_keeper.catmull_effective_look_ahead_m"], $h["lane_keeper.catmull_curve_factor"],
        $h["lane_keeper.catmull_target_along_m"], $h["lane_keeper.catmull_target_lateral_m"],
        $h["lane_keeper.catmull_current_route_idx"], $h["lane_keeper.catmull_max_waypoint_idx"],
        $h["lane_keeper.catmull_walk_end"], $h["lane_keeper.catmull_total_waypoints"],
        $h["lane_keeper.catmull_max_hops"],
        $h["lane_keeper.source_changed"],
        $h["lane_keeper.nearest_segment_id"], $h["lane_keeper.nearest_seg_hop"],
        $h["lane_keeper.nearest_segment_t"], $h["lane_keeper.nearest_heading_filter_applied"],
        $h["lane_keeper.lookahead_final_seg_id"], $h["lane_keeper.lookahead_final_t"],
        $h["lane_keeper.walk_stopped_at_kink"], $h["lane_keeper.walk_kink_deg"],
        $h["lane_keeper.walk_kink_hop"], $h["lane_keeper.kink_threshold_deg"],
        $h["lane_keeper.kink_stuck_secs"],
        $h["lane_keeper.walk_hops_checked"], $h["lane_keeper.walk_kink_degenerate"],
        $h["lane_keeper.walk_max_kink_deg"], $h["lane_keeper.walk_max_kink_hop"],
        $h["lane_keeper.final_internal_kink_deg"], $h["lane_keeper.final_tangent_heading_deg"],
        $h["lane_keeper.final_seg_is_prefab"], $h["lane_keeper.final_seg_length_m"],
        $h["lane_keeper.prefab_curve_fallback"], $h["lane_keeper.internal_kink_over_threshold"],
        $h["lane_keeper.prefab_curve_threshold_deg"],
        $h["lane_keeper.catmull_steer_target_xz"], $h["lane_keeper.catmull_centerline_xz"],
        $h["lane_keeper.catmull_truck_xz"], $h["lane_keeper.catmull_offset_applied_m"],
        $h["lane_keeper.lookahead_m"], $h["lane_keeper.steer_target_xz"],
        $h["lane_keeper.spline_centerline_xz"], $h["lane_keeper.diag_truck_xz"],
        $h["lane_keeper.truck_x"], $h["lane_keeper.truck_z"],
        $h["lane_keeper.target_heading"], $h["lane_keeper.safety_state"],
        $h["scs_sdk_output.brake_written"]
    )
    ($vals -join ",") | Out-File -FilePath $csv -Append -Encoding utf8

    $curSrc = $h["lane_keeper.lateral_source"]
    # Uebergang catmull -> spline_road erkennen (Quellenwechsel auf spline).
    $isSwitch = ($prevSrc -ne "" -and $curSrc -ne $prevSrc -and $curSrc -like "spline*")
    $prevSrc = $curSrc

    $row = "{0,-6} {1,-9} {2,-8} {3,-8} {4,-8} {5,-7} {6,-7} {7,-9} {8,-6} {9}" -f `
        $t, $h["state.heading_stage"], $h["lane_keeper.heading_error_rad"], `
        $h["lane_keeper.nearest_segment_id"], $h["lane_keeper.lookahead_final_seg_id"], `
        $h["lane_keeper.walk_max_kink_deg"], $h["lane_keeper.final_internal_kink_deg"], `
        $h["lane_keeper.final_seg_is_prefab"], $h["lane_keeper.prefab_curve_fallback"], `
        $curSrc

    if ($isSwitch) { Write-Host (">>> " + $row) -ForegroundColor Magenta }                         # Source-Wechsel
    elseif ($h["lane_keeper.prefab_curve_fallback"] -eq "true") { Write-Host $row -ForegroundColor Yellow }  # Fallback aktiv
    elseif ($h["scs_sdk_output.brake_written"] -and [double]$h["scs_sdk_output.brake_written"] -gt 0.0) { Write-Host $row -ForegroundColor Red }  # Safety-Bremse (sollte nun selten)
    elseif ($h["autopilot.state"] -eq "Active" -or $h["autopilot.state"] -eq "Engaged") { Write-Host $row -ForegroundColor Green }
    else { Write-Host $row }

    Start-Sleep -Milliseconds 200
}
