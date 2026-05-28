# Auto-Snap v2: triggert bei DURCHGANG durch Thresholds (von darüber zu darunter)
# Verhindert dass alle Trigger gleichzeitig feuern wenn man schon nah dran ist

$diagKeys = @(
    "lane_follower.junction_detected",
    "lane_follower.junction_distance_m",
    "lane_follower.junction_phase",
    "lane_follower.lateral_dist_signed",
    "lane_follower.nearest_seg_dist_m",
    "lane_follower.nearest_seg_idx",
    "lane_follower.nearest_seg_is_prefab",
    "lane_follower.nearest_seg_ai_path_uid",
    "lane_follower.bias_zone_active",
    "lane_follower.bias_prefab_attempted",
    "lane_follower.bias_prefab_accepted",
    "lane_follower.bias_prefab_rejected_reason",
    "lane_follower.heading_diff_rad",
    "lane_follower.tiebreak_used_successor",
    "lane_follower.last_active_segment_idx",
    "lane_follower.steering_cmd",
    "lane_follower.truck_x",
    "lane_follower.truck_z"
) -join ","

$logFile = "outputs\2026-05-27\ds13e_auto_$(Get-Date -Format 'yyyy-MM-dd_HHmmss').log"
$dir = Split-Path $logFile -Parent
if (-not (Test-Path $dir)) { New-Item -ItemType Directory -Path $dir -Force | Out-Null }

# Trigger-Reihenfolge: 30m -> 20m -> 10m -> 5m -> 2m
# Jeder Trigger feuert nur wenn man vorher DARÜBER war
$thresholds = @(30.0, 20.0, 10.0, 5.0, 2.0)
$labels = @("30m", "20m", "10m", "5m", "in")
$nextIdx = 0  # Index in $thresholds: nächster zu feuernder Trigger
$lastDist = $null

Write-Host "Auto-Snap v2 aktiv." -ForegroundColor Cyan
Write-Host ""
Write-Host "WICHTIG: Starte das Script wenn dist > 35m ist!" -ForegroundColor Yellow
Write-Host "Sonst werden Trigger uebersprungen." -ForegroundColor Yellow
Write-Host ""
Write-Host "Warte auf erste valide Distanz..." -ForegroundColor Gray

$startedTracking = $false

while ($nextIdx -lt $thresholds.Count) {
    $distRaw = .\target\release\blackboard-query.exe --keys lane_follower.junction_distance_m 2>$null | 
               Select-String "junction_distance_m\s*=\s*([\d.]+)" | 
               ForEach-Object { $_.Matches[0].Groups[1].Value }
    
    $detRaw = .\target\release\blackboard-query.exe --keys lane_follower.junction_detected 2>$null |
              Select-String "junction_detected\s*=\s*(\w+)" |
              ForEach-Object { $_.Matches[0].Groups[1].Value }
    
    if ($distRaw -and $detRaw -eq "true") {
        $dist = [float]$distRaw
        
        # Initial-Check: wenn erste Distanz schon unter erstem Threshold, warne
        if (-not $startedTracking) {
            $startedTracking = $true
            if ($dist -lt $thresholds[0]) {
                Write-Host ""
                Write-Host "WARNUNG: Startdistanz ist $([math]::Round($dist,1))m, unter 30m Threshold!" -ForegroundColor Red
                Write-Host "Fahre RUECKWAERTS bis dist > 35m, dann Script neu starten." -ForegroundColor Red
                Write-Host "Trotzdem weiter? Trigger werden ggf. uebersprungen." -ForegroundColor Yellow
                $cont = Read-Host "Weiter? (j/n)"
                if ($cont -ne "j") { break }
                # Skip alle Thresholds die schon unterschritten sind
                while ($nextIdx -lt $thresholds.Count -and $dist -lt $thresholds[$nextIdx]) {
                    Write-Host "Skip $($labels[$nextIdx]) (Threshold $($thresholds[$nextIdx])m bereits unterschritten)" -ForegroundColor DarkGray
                    $nextIdx++
                }
            }
        }
        
        Write-Host -NoNewline "`rdist=$([math]::Round($dist,1))m  next=$($labels[$nextIdx])@$($thresholds[$nextIdx])m       "
        
        # Trigger nur wenn dist UNTER Threshold und vorhin DARUEBER war
        if ($nextIdx -lt $thresholds.Count) {
            $threshold = $thresholds[$nextIdx]
            if ($dist -le $threshold -and ($lastDist -eq $null -or $lastDist -gt $threshold)) {
                Write-Host ""
                Write-Host "=== Trigger $($labels[$nextIdx]) at $([math]::Round($dist,2))m ===" -ForegroundColor Green
                Add-Content $logFile "`n=== Snap $($labels[$nextIdx]) @ dist=$([math]::Round($dist,2))m @ $(Get-Date -Format 'HH:mm:ss.fff') ==="
                .\target\release\blackboard-query.exe --keys $diagKeys | 
                  Tee-Object -Append -FilePath $logFile
                $nextIdx++
            }
        }
        
        $lastDist = $dist
    } else {
        Write-Host -NoNewline "`rkeine junction detected   "
        $lastDist = $null  # reset wenn keine junction
    }
    
    Start-Sleep -Milliseconds 200
}

Write-Host ""
Write-Host "Warte 5s fuer Post-Junction-Snap..." -ForegroundColor Yellow
Start-Sleep -Seconds 5
Add-Content $logFile "`n=== Snap POST @ $(Get-Date -Format 'HH:mm:ss.fff') ==="
.\target\release\blackboard-query.exe --keys $diagKeys | 
  Tee-Object -Append -FilePath $logFile

Write-Host ""
Write-Host "Fertig! Log: $logFile" -ForegroundColor Green