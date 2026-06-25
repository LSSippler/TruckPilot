# Segment-Snap A/B Burst - Live-Diagnose (Hypothese A vs B)
#
# Misst lane_follower Segment-Identitaet vs Spurzahl in schnellen Bursts waehrend
# der Fahrt. Auto-Engage optional via engage-cli.
#
# Usage (Daemon + ETS2 laufen):
#   .\scripts\seg_snap_ab_burst.ps1
#   .\scripts\seg_snap_ab_burst.ps1 -LaneOnly
#   .\scripts\seg_snap_ab_burst.ps1 -NoAutoEngage          # schon engaged
#   .\scripts\seg_snap_ab_burst.ps1 -GoalX 10542 -GoalZ -10870 -CruiseKmh 60
#   .\scripts\seg_snap_ab_burst.ps1 -BurstCount 8 -BurstIntervalMs 60
#
# Steuerung waehrend der Fahrt:
#   SPACE  = 6-8 Samples hintereinander (Burst)
#   Q      = Beenden
#
# A = nearest_seg_idx pendelt, segment_lanes folgt dem Index (Auswahl-Instabilitaet)
# B = idx konstant, segment_lanes wechselt (Daten-Inkonsistenz im Graph)
# Mischform = nearest_seg_is_prefab toggelt (Road <-> Prefab)

param(
    [switch]$NoAutoEngage,
    [switch]$LaneOnly,
    [Parameter(Mandatory = $false)]
    [Nullable[double]]$GoalX,
    [Parameter(Mandatory = $false)]
    [Nullable[double]]$GoalZ,
    [float]$CruiseKmh = 50,
    [int]$BurstCount = 7,
    [int]$BurstIntervalMs = 80,
    [string]$DaemonUrl = "ws://127.0.0.1:8765",
    [string]$OutFile = ""
)

$ErrorActionPreference = "Stop"

function Get-OrDefault {
    param($Hash, $Key, $Default = "?")
    if ($Hash.ContainsKey($Key) -and $null -ne $Hash[$Key] -and $Hash[$Key] -ne "") {
        return $Hash[$Key]
    }
    return $Default
}

$root = Split-Path -Parent $PSScriptRoot
$bbQuery = Join-Path $root "target\release\blackboard-query.exe"
$engageCli = Join-Path $root "target\release\engage-cli.exe"

foreach ($bin in @($bbQuery, $engageCli)) {
    if (-not (Test-Path $bin)) {
        Write-Error "$bin not found - run 'cargo build-release' first."
        exit 1
    }
}

# Mess-Satz C: Segment-Identitaet an Offset koppeln
$burstKeys = @(
    "autopilot.state",
    "telemetry.speed_ms",
    "lane_follower.steering_cmd",
    "lane_follower.nearest_seg_idx",
    "lane_follower.last_active_segment_idx",
    "lane_follower.nearest_seg_ai_path_uid",
    "lane_follower.nearest_seg_is_prefab",
    "lane_follower.segment_lanes",
    "lane_follower.segment_offset",
    "lane_follower.lateral_source",
    "lane_follower.lookahead_seg_idx",
    "lane_follower.lookahead_seg_jump_count",
    "lane_follower.tiebreak_used_successor",
    "lane_follower.nearest_seg_t"
) -join ","

$statusKeys = "autopilot.state,autopilot.fault_reason,lane_keeper.engage_allowed"
$bbLinePattern = '^([\w_\.]+)\s*=\s*(.*)$'

function Invoke-BbQuery {
    param([string]$KeyList)
    $raw = & $bbQuery --url $DaemonUrl --keys $KeyList 2>&1
    $kv = @{}
    foreach ($line in ($raw -split "`n")) {
        $line = $line.Trim()
        if ($line -match $bbLinePattern) {
            $kv[$Matches[1]] = $Matches[2].Trim()
        }
    }
    return $kv
}

function Invoke-EngageCli {
    param([string[]]$CliArgs)
    $out = & $engageCli --url $DaemonUrl @CliArgs 2>&1
    if ($LASTEXITCODE -ne 0) {
        throw ("engage-cli failed: " + ($out -join " "))
    }
    return $out
}

function Wait-AutopilotActive {
    param([int]$TimeoutSec = 30)
    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    while ((Get-Date) -lt $deadline) {
        $kv = Invoke-BbQuery $statusKeys
        $state = Get-OrDefault $kv "autopilot.state"
        Write-Host -NoNewline "`r  Warte auf Active ... state=$state          "
        if ($state -eq "Active") {
            Write-Host ""
            return $true
        }
        if ($state -eq "Fault") {
            $fault = Get-OrDefault $kv "autopilot.fault_reason"
            Write-Host ""
            Write-Warning "Autopilot Fault: $fault"
            return $false
        }
        Start-Sleep -Milliseconds 200
    }
    Write-Host ""
    Write-Warning "Timeout - nicht Active nach ${TimeoutSec}s."
    return $false
}

function Read-BurstSample {
    $kv = Invoke-BbQuery $burstKeys
    return [ordered]@{
        ts       = Get-Date -Format "HH:mm:ss.fff"
        state    = Get-OrDefault $kv "autopilot.state"
        speed    = Get-OrDefault $kv "telemetry.speed_ms"
        steer    = Get-OrDefault $kv "lane_follower.steering_cmd"
        idx      = Get-OrDefault $kv "lane_follower.nearest_seg_idx"
        last_idx = Get-OrDefault $kv "lane_follower.last_active_segment_idx"
        uid      = Get-OrDefault $kv "lane_follower.nearest_seg_ai_path_uid"
        prefab   = Get-OrDefault $kv "lane_follower.nearest_seg_is_prefab"
        lanes    = Get-OrDefault $kv "lane_follower.segment_lanes"
        offset   = Get-OrDefault $kv "lane_follower.segment_offset"
        lat_src  = Get-OrDefault $kv "lane_follower.lateral_source"
        la_idx   = Get-OrDefault $kv "lane_follower.lookahead_seg_idx"
        la_jump  = Get-OrDefault $kv "lane_follower.lookahead_seg_jump_count"
        tiebreak = Get-OrDefault $kv "lane_follower.tiebreak_used_successor"
        seg_t    = Get-OrDefault $kv "lane_follower.nearest_seg_t"
    }
}

function Format-BurstTable {
    param($Samples)
    $hdr = "{0,-12} {1,-5} {2,-6} {3,-6} {4,-7} {5,-6} {6,-5} {7,-6} {8,-7} {9,-6} {10,-5} {11,-4} {12,-8}" -f `
        "zeit", "idx", "last", "lanes", "offset", "prefab", "t", "la_idx", "jump", "tie", "steer", "spd", "lat_src"
    Write-Host $hdr -ForegroundColor DarkGray
    foreach ($s in $Samples) {
        $tVal = $s.seg_t
        $tDisp = $tVal
        if ($tVal -ne "?" -and [double]::TryParse($tVal, [ref]$null)) {
            $tv = [double]$tVal
            if ($tv -lt 0.08 -or $tv -gt 0.92) { $tDisp = "*$tVal" }
        }
        $line = "{0,-12} {1,-5} {2,-6} {3,-6} {4,-7} {5,-6} {6,-7} {7,-6} {8,-6} {9,-5} {10,-4} {11,-8} {12,-8}" -f `
            $s.ts, $s.idx, $s.last_idx, $s.lanes, $s.offset, $s.prefab, $tDisp, $s.la_idx, $s.la_jump, $s.tiebreak, $s.steer, $s.speed, $s.lat_src
        $color = "White"
        if ($s.prefab -eq "true") { $color = "Cyan" }
        Write-Host $line -ForegroundColor $color
    }
    Write-Host "  (* = nearest_seg_t nahe Segmentgrenze 0/1)" -ForegroundColor DarkGray
}

function Analyze-AbVerdict {
    param($Samples)

    $idxSet    = ($Samples | ForEach-Object { $_.idx }    | Where-Object { $_ -ne "?" } | Select-Object -Unique)
    $lanesSet  = ($Samples | ForEach-Object { $_.lanes }  | Where-Object { $_ -ne "?" } | Select-Object -Unique)
    $prefabSet = ($Samples | ForEach-Object { $_.prefab } | Where-Object { $_ -ne "?" } | Select-Object -Unique)
    $tieTrue   = @($Samples | Where-Object { $_.tiebreak -eq "true" }).Count
    $maxJump   = 0
    foreach ($s in $Samples) {
        if ($s.la_jump -ne "?" -and [int]::TryParse($s.la_jump, [ref]$null)) {
            $j = [int]$s.la_jump
            if ($j -gt $maxJump) { $maxJump = $j }
        }
    }

    Write-Host ""
    Write-Host "=== Auswertung ===" -ForegroundColor Yellow
    Write-Host ("  idx-Werte:    {0}" -f ($idxSet -join ", "))
    Write-Host ("  lanes-Werte:  {0}" -f ($lanesSet -join ", "))
    Write-Host ("  prefab-Werte: {0}" -f ($prefabSet -join ", "))
    Write-Host ("  tiebreak=true: {0}/{1}  max la_jump: {2}" -f $tieTrue, $Samples.Count, $maxJump)

    $verdicts = @()

    if ($idxSet.Count -ge 2 -and $lanesSet.Count -ge 2) {
        $verdicts += ('A - Segment-Pendeln: idx wechselt zwischen {0}, lanes folgt mit.' -f ($idxSet -join ' <-> '))
    }
    if ($idxSet.Count -eq 1 -and $lanesSet.Count -ge 2) {
        $verdicts += ('B - Graph-Daten: idx={0} konstant, segment_lanes wechselt trotzdem ({1}).' -f $idxSet[0], ($lanesSet -join ' <-> '))
    }
    if ($prefabSet.Count -ge 2) {
        $verdicts += ('Mischform - prefab toggelt ({0}); oft Road/Prefab-Snap + navcurve-offset-0.' -f ($prefabSet -join ' <-> '))
    }
    if ($tieTrue -ge 2 -or $maxJump -ge 3) {
        $verdicts += ('A-Signal - instabile Segment-Auswahl (tiebreak={0}, max_jump={1}).' -f $tieTrue, $maxJump)
    }
    if ($verdicts.Count -eq 0) {
        $verdicts += 'Kein klares A/B in diesem Burst - erneut waehrend Schlaengeln triggern (SPACE).'
    }

    foreach ($v in $verdicts) {
        $fc = if ($v.StartsWith("B")) { "Red" } elseif ($v.StartsWith("A")) { "Green" } else { "Yellow" }
        Write-Host "  -> $v" -ForegroundColor $fc
    }
    Write-Host ""
    return ($verdicts -join ' | ')
}

function Invoke-Burst {
    param([int]$BurstNum)
    Write-Host ""
    $burstTitle = 'Burst #{0} @ {1} ({2} samples, {3} ms)' -f $BurstNum, (Get-Date -Format 'HH:mm:ss'), $BurstCount, $BurstIntervalMs
    Write-Host $burstTitle -ForegroundColor Cyan

    $samples = @()
    for ($i = 1; $i -le $BurstCount; $i++) {
        $samples += Read-BurstSample
        if ($i -lt $BurstCount) {
            Start-Sleep -Milliseconds $BurstIntervalMs
        }
    }

    Format-BurstTable $samples
    $verdict = Analyze-AbVerdict $samples

    if ($OutFile -ne "") {
        $block = @()
        $block += ""
        $block += ('=== Burst #{0} {1} ===' -f $BurstNum, (Get-Date -Format 'yyyy-MM-dd HH:mm:ss'))
        $block += ($samples | ConvertTo-Json -Compress)
        $block += "Verdict: $verdict"
        $block | Out-File -Encoding utf8 -Append -FilePath $OutFile
    }

    return $samples
}

if ($OutFile -eq "") {
    $datum = Get-Date -Format "yyyy-MM-dd"
    $outDir = Join-Path $root "outputs\$datum"
    New-Item -ItemType Directory -Force -Path $outDir | Out-Null
    $OutFile = Join-Path $outDir ("seg_snap_ab_burst_{0}.log" -f (Get-Date -Format "yyyyMMdd_HHmmss"))
}

$logHeader = '# seg_snap_ab_burst ' + (Get-Date -Format 'yyyy-MM-dd HH:mm:ss')
$logHeader | Out-File -Encoding utf8 -FilePath $OutFile
"# keys: $burstKeys" | Out-File -Encoding utf8 -Append -FilePath $OutFile

Write-Host "Segment-Snap A/B Burst" -ForegroundColor Cyan
Write-Host "Log: $OutFile"
Write-Host ""

if (-not $NoAutoEngage) {
    Write-Host "Auto-Engage ..." -ForegroundColor Cyan
    try {
        if ($null -ne $GoalX -and $null -ne $GoalZ) {
            Write-Host ('  set-goal-pos --x={0} --z={1}' -f $GoalX, $GoalZ)
            Invoke-EngageCli @("set-goal-pos", "--x=$GoalX", "--z=$GoalZ") | Out-Null
        }
        Write-Host ('  set-cruise {0}' -f $CruiseKmh)
        Invoke-EngageCli @("set-cruise", "$CruiseKmh") | Out-Null
        if ($LaneOnly) {
            Write-Host "  engage --lane-only"
            Invoke-EngageCli @("engage", "--lane-only") | Out-Null
        } else {
            Write-Host "  engage"
            Invoke-EngageCli @("engage") | Out-Null
        }
        $null = Wait-AutopilotActive
    } catch {
        Write-Warning $_.Exception.Message
        Write-Host "Weiter ohne Engage - SPACE fuer Burst, Q zum Beenden." -ForegroundColor Yellow
    }
} else {
    $kv = Invoke-BbQuery $statusKeys
    Write-Host ('Autopilot: state={0}' -f (Get-OrDefault $kv "autopilot.state"))
}

Write-Host ""
Write-Host "Fahre los. Wenn es schlaengelt:" -ForegroundColor Green
Write-Host ('  SPACE = Burst ({0} x {1} ms)   Q = Ende' -f $BurstCount, $BurstIntervalMs)
Write-Host ""

$burstNum = 0
$consoleOk = $true
try {
    $null = [Console]::KeyAvailable
} catch {
    $consoleOk = $false
    Write-Warning "Keine interaktive Konsole erkannt - Enter startet Burst, leere Zeile beendet."
}

while ($true) {
    if ($consoleOk -and [Console]::KeyAvailable) {
        $key = [Console]::ReadKey($true)
        if ($key.Key -eq "Spacebar") {
            $burstNum++
            $null = Invoke-Burst $burstNum
            Write-Host "SPACE = naechster Burst   Q = Ende" -ForegroundColor DarkGray
        } elseif ($key.Key -eq "Q") {
            Write-Host "Beendet."
            break
        }
    } elseif (-not $consoleOk) {
        $mini = Invoke-BbQuery "lane_follower.nearest_seg_idx,lane_follower.segment_lanes,lane_follower.nearest_seg_t,lane_follower.steering_cmd,autopilot.state"
        $idx   = Get-OrDefault $mini "lane_follower.nearest_seg_idx"
        $lanes = Get-OrDefault $mini "lane_follower.segment_lanes"
        $t     = Get-OrDefault $mini "lane_follower.nearest_seg_t"
        $steer = Get-OrDefault $mini "lane_follower.steering_cmd"
        $st    = Get-OrDefault $mini "autopilot.state"
        Write-Host ('live: state={0} idx={1} lanes={2} t={3} steer={4}' -f $st, $idx, $lanes, $t, $steer)
        $inp = Read-Host "Enter=Burst, leer=Ende"
        if ($inp -eq "") { break }
        $burstNum++
        $null = Invoke-Burst $burstNum
    } else {
        $mini = Invoke-BbQuery "lane_follower.nearest_seg_idx,lane_follower.segment_lanes,lane_follower.nearest_seg_t,lane_follower.steering_cmd,autopilot.state"
        $idx   = Get-OrDefault $mini "lane_follower.nearest_seg_idx"
        $lanes = Get-OrDefault $mini "lane_follower.segment_lanes"
        $t     = Get-OrDefault $mini "lane_follower.nearest_seg_t"
        $steer = Get-OrDefault $mini "lane_follower.steering_cmd"
        $st    = Get-OrDefault $mini "autopilot.state"
        Write-Host -NoNewline ("`r  live: state={0} idx={1} lanes={2} t={3} steer={4}   [SPACE=Burst Q=Ende]   " -f $st, $idx, $lanes, $t, $steer)
        Start-Sleep -Milliseconds 250
    }
}

Write-Host ""
Write-Host "Fertig. Log: $OutFile"
