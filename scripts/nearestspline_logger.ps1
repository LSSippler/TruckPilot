# nearestspline_logger.ps1
# Kontinuierlicher Logger fuer den routerlosen NearestSpline-Modus.
# Schreibt Chain- + Cross-Track-Keys in eine CSV unter outputs\<datum>\.
# Strg+C zum Stoppen. Faengt gezielt den Moment ab, in dem der Truck quer faehrt.

$ErrorActionPreference = "SilentlyContinue"

$datum   = Get-Date -Format "yyyy-MM-dd"
$outDir  = Join-Path "outputs" $datum
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
$stamp   = Get-Date -Format "yyyyMMdd_HHmmss"
$csvPath = Join-Path $outDir "nearestspline_$stamp.csv"

# Keys die wir pro Tick lesen
$keys = @(
    "autopilot.state",
    "lane_keeper.mode",
    "lane_keeper.chain_segment_idx",
    "lane_keeper.chain_advance_reason",
    "lane_keeper.chain_successor_count",
    "lane_keeper.chain_reverse_skipped",
    "lane_keeper.chain_broken",
    "lane_keeper.chain_seg_t",
    "lane_keeper.chain_remaining_m",
    "lane_keeper.lane_offset_applied_m",
    "lane_keeper.xtrack_e_lat_m",
    "lane_keeper.heading_contribution_rad",
    "lane_keeper.xtrack_contribution_rad",
    "lane_keeper.steering_out",
    "lane_keeper.null_steer_cause",
    "telemetry.speed_ms",
    "telemetry.position_x",
    "telemetry.position_z",
    "telemetry.heading",
    # Hypothesen-Diag: Snap-Ursache
    "lane_keeper.sel_path",
    "lane_keeper.route_set_size",
    "lane_keeper.route_set_has_nearest",
    "lane_keeper.route_set_has_lookahead",
    "lane_keeper.lookahead_final_seg_id",
    "lane_keeper.nearest_seg_idx"
) -join ","

# CSV-Header (kurze Spaltennamen)
$header = "t,state,mode,seg,advance,succ,rev_skip,broken,seg_t,remain_m,lane_off,e_lat,head_c,xtrack_c,steer,null_cause,speed,pos_x,pos_z,hdg,sel_path,rs_size,rs_has_near,rs_has_look,look_seg,near_seg"
Set-Content -Path $csvPath -Value $header -Encoding UTF8

Write-Host "Logging nach $csvPath  (Strg+C zum Stoppen)" -ForegroundColor Cyan
Write-Host ">>> rot = chain_broken=true | gelb = |e_lat|>3m oder |steer|>0.8 (Querfahrt-Verdacht)" -ForegroundColor Gray
Write-Host ("{0,-6} {1,-8} {2,-13} {3,-9} {4,-12} {5,-5} {6,-8} {7,-8} {8,-9} {9,-9} {10,-9}" -f `
    "t","state","seg","advance","succ/skip","brk","e_lat","head_c","xtrk_c","steer","speed")

$t0 = Get-Date

while ($true) {
    $now = (Get-Date) - $t0
    $t = "{0:N1}" -f $now.TotalSeconds

    $raw = & .\target\release\blackboard-query.exe --keys $keys 2>$null

    # Werte aus der "  key = value"-Ausgabe parsen
    $v = @{}
    foreach ($line in $raw) {
        if ($line -match '^\s*([\w\.]+)\s*=\s*(.*)$') {
            $v[$matches[1]] = $matches[2].Trim()
        }
    }

    $state    = $v["autopilot.state"]
    $seg      = $v["lane_keeper.chain_segment_idx"]
    $advance  = $v["lane_keeper.chain_advance_reason"]
    $succ     = $v["lane_keeper.chain_successor_count"]
    $revskip  = $v["lane_keeper.chain_reverse_skipped"]
    $broken   = $v["lane_keeper.chain_broken"]
    $segt     = $v["lane_keeper.chain_seg_t"]
    $remain   = $v["lane_keeper.chain_remaining_m"]
    $laneoff  = $v["lane_keeper.lane_offset_applied_m"]
    $elat     = $v["lane_keeper.xtrack_e_lat_m"]
    $headc    = $v["lane_keeper.heading_contribution_rad"]
    $xtrkc    = $v["lane_keeper.xtrack_contribution_rad"]
    $steer    = $v["lane_keeper.steering_out"]
    $nullc    = $v["lane_keeper.null_steer_cause"]
    $speed    = $v["telemetry.speed_ms"]
    $posx     = $v["telemetry.position_x"]
    $posz     = $v["telemetry.position_z"]
    $hdg      = $v["telemetry.heading"]
    # Hypothesen-Diag
    $selpath  = $v["lane_keeper.sel_path"]
    $rssize   = $v["lane_keeper.route_set_size"]
    $rsnear   = $v["lane_keeper.route_set_has_nearest"]
    $rslook   = $v["lane_keeper.route_set_has_lookahead"]
    $lookseg  = $v["lane_keeper.lookahead_final_seg_id"]
    $nearseg  = $v["lane_keeper.nearest_seg_idx"]

    # CSV-Zeile
    $row = "$t,$state,$($v['lane_keeper.mode']),$seg,$advance,$succ,$revskip,$broken,$segt,$remain,$laneoff,$elat,$headc,$xtrkc,$steer,$nullc,$speed,$posx,$posz,$hdg,$selpath,$rssize,$rsnear,$rslook,$lookseg,$nearseg"
    Add-Content -Path $csvPath -Value $row -Encoding UTF8

    # Konsolen-Zeile, eingefaerbt bei Verdacht
    $color = "White"
    if ($broken -eq "true") { $color = "Red" }
    elseif (($elat -as [double]) -ne $null -and [math]::Abs([double]$elat) -gt 3.0) { $color = "Yellow" }
    elseif (($steer -as [double]) -ne $null -and [math]::Abs([double]$steer) -gt 0.8) { $color = "Yellow" }

    $sucskip = "$succ/$revskip"
    Write-Host ("{0,-6} {1,-8} {2,-13} {3,-9} {4,-12} {5,-5} {6,-9} {7,-9} {8,-9} {9,-9} {10,-9}" -f `
        $t, $state, $seg, $advance, $sucskip, $broken, $elat, $headc, $xtrkc, $steer, $speed) -ForegroundColor $color
    # Diag-Zeile nur wenn Snap stattgefunden hat (near_seg wechselt)
    if ($rsnear -ne "" -or $rssize -ne "") {
        Write-Host ("       sel_path={0} rs_size={1} rs_has_near={2} rs_has_look={3} near={4} look={5}" -f `
            $selpath, $rssize, $rsnear, $rslook, $nearseg, $lookseg) -ForegroundColor DarkGray
    }

    Start-Sleep -Milliseconds 200
}