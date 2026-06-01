# DS14 Live-Test T1 Durchfahrt — Logger
# Pollt den Blackboard 5x/Sek, schreibt CSV + Live-Konsole, markiert Gap-Frames.
# Fahre T1 (Ost-Arm -> durch die Kreuzung -> raus) bei ~30 km/h mit Autopilot.
# Stoppen mit Strg+C.

$ErrorActionPreference = "Stop"
$bb = ".\target\release\blackboard-query.exe"
$keys = @(
    "lane_follower.lateral_source",
    "lane_follower.bias_prefab_rejected_reason",
    "lane_follower.rate_limit_active",
    "lane_follower.heading_to_lookahead_deg",
    "lane_follower.truck_heading_deg",
    "lane_follower.steering_cmd",
    "lane_follower.lateral_dist_signed",
    "lane_follower.truck_x",
    "lane_follower.truck_z"
) -join ","

$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$outDir = "outputs\$(Get-Date -Format 'yyyy-MM-dd')"
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
$csv = Join-Path $outDir "ds14_t1_drive_$stamp.csv"

"t,lateral_source,reject,rate_limit,head_to_la,truck_head,head_diff,steering,lat_dist,x,z" | Out-File -FilePath $csv -Encoding utf8

Write-Host "Logging nach $csv  (Strg+C zum Stoppen)" -ForegroundColor Cyan
Write-Host ("{0,-8} {1,-12} {2,-10} {3,-9} {4,-9} {5,-8} {6,-8} {7}" -f "t","source","reject","rate_lim","head_dif","steer","lat","pos") -ForegroundColor Gray

$t0 = Get-Date
while ($true) {
    $t = "{0:N1}" -f ((Get-Date) - $t0).TotalSeconds
    $raw = & $bb --keys $keys 2>$null

    $h = @{}
    foreach ($line in $raw) {
        if ($line -match '^\s*lane_follower\.(\S+)\s*=\s*(.+?)\s*$') {
            $h[$matches[1]] = $matches[2]
        }
    }

    $src   = $h["lateral_source"]
    $rej   = $h["bias_prefab_rejected_reason"]
    $rl    = $h["rate_limit_active"]
    $h2la  = $h["heading_to_lookahead_deg"]
    $th    = $h["truck_heading_deg"]
    $steer = $h["steering_cmd"]
    $lat   = $h["lateral_dist_signed"]
    $x     = $h["truck_x"]
    $z     = $h["truck_z"]

    # Heading-Diff (signiert, auf -180..180 normiert)
    $diff = ""
    if ($h2la -and $th) {
        try {
            $d = [double]$h2la - [double]$th
            while ($d -gt 180) { $d -= 360 }
            while ($d -lt -180) { $d += 360 }
            $diff = "{0:N1}" -f $d
        } catch { $diff = "" }
    }

    "$t,$src,$rej,$rl,$h2la,$th,$diff,$steer,$lat,$x,$z" | Out-File -FilePath $csv -Append -Encoding utf8

    # Live-Zeile, Gap-Frames (none_found) farbig
    $pos = if ($x -and $z) { "$([math]::Round([double]$x)),$([math]::Round([double]$z))" } else { "-" }
    $row = "{0,-8} {1,-12} {2,-10} {3,-9} {4,-9} {5,-8} {6,-8} {7}" -f $t,$src,$rej,$rl,$diff,$steer,$lat,$pos

    if ($rej -eq "none_found") {
        $color = if ($rl -eq "true") { "Red" } else { "Green" }   # rate_limit in Gap: rot=schlecht, grün=Fix wirkt
        Write-Host $row -ForegroundColor $color
    } else {
        Write-Host $row
    }

    Start-Sleep -Milliseconds 200
}