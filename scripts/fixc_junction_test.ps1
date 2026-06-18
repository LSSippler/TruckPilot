<#
.SYNOPSIS
  Test-Harness fuer Fix-C-Junction-Reality-Check (navcurve_by_from_to + Junction-Failsafe).

.BESCHREIBUNG
  Automatisiert Route-Setup, Route-Stabilitaets-Check, Engage im richtigen Moment (nur
  waehrend der Fahrt) und strukturiertes Junction-Polling. Das Skript lenkt den Truck
  NICHT - der Mensch muss fahren.

.BEDIENUNG
  1. Repo-Root als Arbeitsverzeichnis (cd TruckPilot).
  2. cargo build-release (blackboard-query.exe + engage-cli.exe muessen existieren).
  3. TruckPilot-Daemon starten (z.B. cargo run --release --bin truckpilot ...).
  4. ETS2 laufen, Motor an, Lkw auf einer befahrbaren Strecke mit Junction in Reichweite.
  5. Skript starten (Default: Ziel 3 km voraus in Fahrtrichtung vom Lkw):
       .\scripts\fixc_junction_test.ps1
     Festes Welt-Ziel (nur wenn vom aktuellen Standort erreichbar):
       .\scripts\fixc_junction_test.ps1 -FixedGoal -GoalX 18000 -GoalZ 14000
     Zielweite anpassen:
       .\scripts\fixc_junction_test.ps1 -AutoGoalAheadM 5000
  6. Auf Konsolen-Aufforderung losfahren (~CruiseKmh km/h Richtung naechste Junction).
     Engage erfolgt automatisch sobald Geschwindigkeit + engage_block_reason + Route stimmen.
  7. Waehrend Active durch die Junction fahren; Skript loggt und wertet am Ende aus.
     Strg+C beendet sauber und fuehrt die Auto-Auswertung trotzdem aus.

.NOTES
  Fix C (90e40ea) + Junction-Failsafe (3c6963d). Log: outputs/.../fixc_junction_test_<ts>.log
#>

param(
    [switch]$FixedGoal,
    [double]$GoalX = 18000,
    [double]$GoalZ = 14000,
    [double]$AutoGoalAheadM = 3000,
    [bool]$UseTruckStart = $true,
    [float]$CruiseKmh = 30,
    [double]$MinEngageSpeedMs = 5.0,
    [int]$PollIntervalMs = 200,
    [string]$BbExe = ".\target\release\blackboard-query.exe",
    [string]$EngageExe = ".\target\release\engage-cli.exe",
    [string]$OutDir = "outputs/2026-06-17",
    [string]$DaemonUrl = "ws://127.0.0.1:8765",
    [int]$RoutePlanWaitSec = 20
)

$ErrorActionPreference = "Continue"

# â”€â”€ Repo-Pfade â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
$RepoRoot = Split-Path -Parent $PSScriptRoot

function Resolve-RepoPath {
    param([string]$Path)
    if ([System.IO.Path]::IsPathRooted($Path)) { return $Path }
    return (Join-Path $RepoRoot ($Path -replace '/', '\'))
}

$BbExe = Resolve-RepoPath $BbExe
$EngageExe = Resolve-RepoPath $EngageExe
$OutDir = Resolve-RepoPath $OutDir

# â”€â”€ Skript-Zustand (fuer Auswertung + Ctrl+C) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
$script:LogFile = $null
$script:LogWriter = $null
$script:ShouldExit = $false
$script:RouteUnstableInStand = $false
$script:EngageFailed = $false
$script:EverReachedActive = $false
$script:Phase5Started = $false
$script:ResolvedGoalX = 0.0
$script:ResolvedGoalZ = 0.0
$script:GoalMode = "ahead"

# Planungsergebnisse, bei denen Warten sinnlos ist (sofortiger Abbruch).
$script:TerminalPlanningResults = @(
    "no_path_found",
    "uid_not_in_graph",
    "start_node_unknown",
    "goal_snap_failed",
    "graph_not_loaded",
    "serialise_error",
    "uid_parse_error",
    "error"
)

$bbLinePattern = '^([\w_\.]+)\s*=\s*(.*)$'
$statusPrecondPattern = '^\s*(telemetry_ok|engine_running|critical_plugins|router_active)\s*=\s*(true|false)\s*$'

# â”€â”€ Hilfsfunktionen â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
function Get-OrDefault {
    param($Hash, [string]$Key, [string]$Default = "?")
    if ($null -ne $Hash -and $Hash.ContainsKey($Key) -and $null -ne $Hash[$Key] -and $Hash[$Key] -ne "") {
        return $Hash[$Key]
    }
    return $Default
}

function Format-Multi {
    param(
        [string]$Template,
        [Parameter(ValueFromRemainingArguments = $true)]
        [object[]]$Values
    )
    return $Template -f $Values
}

function Write-LogLine {
    param([string]$Line)
    Write-Host $Line
    if ($null -ne $script:LogWriter) {
        try {
            $script:LogWriter.WriteLine($Line)
            $script:LogWriter.Flush()
        } catch {
            Write-Warning (Format-Multi "Log-Schreibfehler: {0}" $_.Exception.Message)
        }
    }
}

function Write-LogFormatted {
    param(
        [string]$Template,
        [Parameter(ValueFromRemainingArguments = $true)]
        [object[]]$Values
    )
    Write-LogLine (Format-Multi $Template @Values)
}

function Write-Phase {
    param([string]$Title)
    Write-Host ""
    Write-Host (Format-Multi "========== {0} ==========" $Title) -ForegroundColor Cyan
}

function Parse-KvLines {
    param([string[]]$Lines)
    $kv = @{}
    foreach ($line in $Lines) {
        $line = $line.Trim()
        if ($line -match $bbLinePattern) {
            $kv[$Matches[1]] = $Matches[2].Trim()
        }
    }
    return $kv
}

function Invoke-BbQuery {
    param([string]$KeyList)
    try {
        $raw = & $BbExe --url $DaemonUrl --keys $KeyList 2>&1
        if ($LASTEXITCODE -ne 0 -and $null -eq $raw) {
            Write-Warning "blackboard-query Exit-Code $LASTEXITCODE"
        }
        $text = if ($raw -is [array]) { $raw -join "`n" } else { [string]$raw }
        return Parse-KvLines ($text -split "`n")
    } catch {
        Write-Warning ("blackboard-query Fehler: {0}" -f $_.Exception.Message)
        return @{}
    }
}

function Invoke-EngageCli {
    param([string[]]$CliArgs)
    try {
        $out = & $EngageExe --url $DaemonUrl @CliArgs 2>&1
        $text = if ($out -is [array]) { $out -join "`n" } else { [string]$out }
        if ($LASTEXITCODE -ne 0) {
            Write-Warning ("engage-cli fehlgeschlagen (Exit {0}): {1}" -f $LASTEXITCODE, $text.Trim())
            return @{ Ok = $false; Lines = @($text -split "`n"); Text = $text }
        }
        return @{ Ok = $true; Lines = @($text -split "`n"); Text = $text }
    } catch {
        Write-Warning ("engage-cli Fehler: {0}" -f $_.Exception.Message)
        return @{ Ok = $false; Lines = @(); Text = "" }
    }
}

function Parse-EngageStatus {
    param([string[]]$Lines)
    $result = @{
        autopilot_state = "?"
        fault_reason    = "?"
        telemetry_ok    = $false
        engine_running  = $false
        critical_plugins = $false
        router_active   = $false
    }
    foreach ($line in $Lines) {
        $t = $line.Trim()
        if ($t -match '^autopilot\.state\s*=\s*(.+)$') {
            $result.autopilot_state = $Matches[1].Trim()
        } elseif ($t -match '^autopilot\.fault\s*=\s*(.+)$') {
            $result.fault_reason = $Matches[1].Trim()
        } elseif ($t -match $statusPrecondPattern) {
            $val = ($Matches[2] -eq "true")
            switch ($Matches[1]) {
                "telemetry_ok"       { $result.telemetry_ok = $val }
                "engine_running"     { $result.engine_running = $val }
                "critical_plugins"   { $result.critical_plugins = $val }
                "router_active"      { $result.router_active = $val }
            }
        }
    }
    return $result
}

function Test-PlanningTerminal {
    param([string]$PlanResult)
    if ([string]::IsNullOrWhiteSpace($PlanResult) -or $PlanResult -eq "?" -or $PlanResult -eq "pending") {
        return $false
    }
    if ($PlanResult -eq "ok") { return $false }
    return $script:TerminalPlanningResults -contains $PlanResult
}

function Get-GoalCoordinates {
    if ($FixedGoal) {
        $script:GoalMode = "fixed"
        return @{
            X = $GoalX
            Z = $GoalZ
            Mode = "fixed"
        }
    }

    $kv = Invoke-BbQuery "telemetry.position_x,telemetry.position_z,telemetry.heading"
    $tx = 0.0
    $tz = 0.0
    $hdg = 0.0
    $xOk = [double]::TryParse((Get-OrDefault $kv "telemetry.position_x"), [ref]$tx)
    $zOk = [double]::TryParse((Get-OrDefault $kv "telemetry.position_z"), [ref]$tz)
    $hOk = [double]::TryParse((Get-OrDefault $kv "telemetry.heading"), [ref]$hdg)

    if (-not ($xOk -and $zOk -and $hOk)) {
        Write-Host "Abbruch: Telemetrie-Position/Heading nicht lesbar fuer Auto-Ziel." -ForegroundColor Red
        exit 1
    }

    $gx = $tx + ($AutoGoalAheadM * [Math]::Sin($hdg))
    $gz = $tz + ($AutoGoalAheadM * [Math]::Cos($hdg))
    $script:GoalMode = "ahead"

    return @{
        X = $gx
        Z = $gz
        Mode = "ahead"
        TruckX = $tx
        TruckZ = $tz
        HeadingRad = $hdg
        AheadM = $AutoGoalAheadM
    }
}

function Get-RouterActive {
    # Bevorzugt Blackboard-Key router.active; Fallback engage-cli status.
    $kv = Invoke-BbQuery "router.active"
    $ra = Get-OrDefault $kv "router.active" ""
    if ($ra -eq "true" -or $ra -eq "false") {
        return ($ra -eq "true")
    }
    $st = Invoke-EngageCli @("status")
    if ($st.Ok) {
        $parsed = Parse-EngageStatus $st.Lines
        return $parsed.router_active
    }
    return $false
}

function Wait-RouterActive {
    param([int]$TimeoutSec)

    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    $lastPre = $null
    $lastPlan = "?"
    $lastErr = ""

    while ((Get-Date) -lt $deadline -and -not $script:ShouldExit) {
        $rStatus = Invoke-EngageCli @("status")
        if ($rStatus.Ok) {
            $lastPre = Parse-EngageStatus $rStatus.Lines
            if ($lastPre.router_active) {
                return @{ Pre = $lastPre; TerminalFailure = $false; PlanResult = "ok"; PlanError = "" }
            }
        }

        $kv = Invoke-BbQuery "router.last_planning_result,router.waypoint_count,router.last_planning_error_detail,router.last_snap_dist,router.current_goal_uid,router.start_uid"
        $plan = Get-OrDefault $kv "router.last_planning_result" "?"
        $wps  = Get-OrDefault $kv "router.waypoint_count" "?"
        $err  = Get-OrDefault $kv "router.last_planning_error_detail" ""
        $snap = Get-OrDefault $kv "router.last_snap_dist" "?"
        $lastPlan = $plan
        $lastErr = $err

        $diag = "plan=$plan waypoints=$wps snap_dist_m=$snap"
        if ($err -ne "" -and $err -ne "?") {
            $diag += " err=$err"
        }
        Write-Host (Format-Multi "  warte auf Route ... router_active=false ({0}) {1}" (Get-Date -Format "HH:mm:ss.fff") $diag)

        if (Test-PlanningTerminal $plan) {
            Write-Host (Format-Multi "  -> Planung endgueltig fehlgeschlagen ({0}), breche Wartezeit ab." $plan) -ForegroundColor Yellow
            break
        }

        Start-Sleep -Milliseconds $PollIntervalMs
    }

    if ($null -eq $lastPre) {
        $rStatus = Invoke-EngageCli @("status")
        if ($rStatus.Ok) {
            $lastPre = Parse-EngageStatus $rStatus.Lines
        }
    }
    if ($null -eq $lastPre) {
        $lastPre = Parse-EngageStatus @()
    }

    return @{
        Pre = $lastPre
        TerminalFailure = (Test-PlanningTerminal $lastPlan)
        PlanResult = $lastPlan
        PlanError = $lastErr
    }
}

function Test-EngageReady {
    param($Kv, [bool]$RouterActive)
    $speedStr = Get-OrDefault $Kv "telemetry.speed_ms" "0"
    $speed = 0.0
    [void][double]::TryParse($speedStr, [ref]$speed)
    $block = Get-OrDefault $Kv "lane_keeper.engage_block_reason" ""
    return ($speed -gt $MinEngageSpeedMs) -and ($block -eq "ok") -and $RouterActive
}

function Format-WaitLine {
    param($Kv, [bool]$RouterActive, [string]$Timestamp)
    $speed = Get-OrDefault $Kv "telemetry.speed_ms" "?"
    $block = Get-OrDefault $Kv "lane_keeper.engage_block_reason" "?"
    $dist  = Get-OrDefault $Kv "lane_keeper.engage_dist_m" "?"
    $ra    = if ($RouterActive) { "true" } else { "false" }
    return ("{0} | speed_ms={1} | engage_block={2} | engage_dist_m={3} | router_active={4}" -f `
        $Timestamp, $speed, $block, $dist, $ra)
}

function Format-JunctionLine {
    param($Kv, [string]$Timestamp, [string]$Marker = "")
    $parts = @(
        $Timestamp,
        ("state={0}" -f (Get-OrDefault $Kv "autopilot.state")),
        ("skip={0}" -f (Get-OrDefault $Kv "lane_keeper.skip_reason")),
        ("stage={0}" -f (Get-OrDefault $Kv "lane_keeper.stage")),
        ("steer_offers={0}" -f (Get-OrDefault $Kv "arbitration.steering_offer_count")),
        ("steer_winner={0}" -f (Get-OrDefault $Kv "arbitration.steering_winner_plugin")),
        ("route_filt={0}" -f (Get-OrDefault $Kv "lane_keeper.nearest_route_filtered")),
        ("navcurve_cnt={0}" -f (Get-OrDefault $Kv "lane_keeper.total_route_navcurve_count")),
        ("chosen_navcurve={0}" -f (Get-OrDefault $Kv "lane_keeper.chosen_segment_is_navcurve")),
        ("failsafe={0}" -f (Get-OrDefault $Kv "lane_keeper.junction_failsafe_active")),
        ("speed_ms={0}" -f (Get-OrDefault $Kv "telemetry.speed_ms")),
        ("pos=({0},{1})" -f (Get-OrDefault $Kv "telemetry.position_x"), (Get-OrDefault $Kv "telemetry.position_z"))
    )
    $line = $parts -join " | "
    if ($Marker -ne "") {
        $line = "*** $Marker *** $line"
    }
    return $line
}

function Open-LogFile {
    if (-not (Test-Path $OutDir)) {
        New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
    }
    $tsName = Get-Date -Format "yyyyMMdd_HHmmss"
    $script:LogFile = Join-Path $OutDir ("fixc_junction_test_{0}.log" -f $tsName)
    $script:LogWriter = New-Object System.IO.StreamWriter($script:LogFile, $false, [System.Text.Encoding]::UTF8)
    $script:LogWriter.WriteLine((Format-Multi "# fixc_junction_test.ps1 - {0}" (Get-Date -Format "yyyy-MM-dd HH:mm:ss")))
    $goalNote = if ($FixedGoal) {
        Format-Multi "fixed goal=({0},{1})" $GoalX $GoalZ
    } else {
        Format-Multi "auto goal ahead_m={0}" $AutoGoalAheadM
    }
    $script:LogWriter.WriteLine((Format-Multi "# {0} CruiseKmh={1} MinEngageSpeedMs={2}" $goalNote $CruiseKmh $MinEngageSpeedMs))
    $script:LogWriter.Flush()
    Write-Host ("Logdatei: {0}" -f $script:LogFile) -ForegroundColor DarkGray
}

function Close-LogFile {
    if ($null -ne $script:LogWriter) {
        try {
            $script:LogWriter.Flush()
            $script:LogWriter.Close()
        } catch { }
        $script:LogWriter = $null
    }
}

function Invoke-Phase6Evaluation {
    param([string]$LogPath)

    Write-Phase "PHASE 6 - Auto-Auswertung"

    if ([string]::IsNullOrWhiteSpace($LogPath) -or -not (Test-Path $LogPath)) {
        Write-Host "Keine Logdatei fuer Auswertung vorhanden." -ForegroundColor Yellow
        return
    }

    $everActive = $false
    $everNavcurveWhileActive = $false
    $stateDroppedDuringNavcurve = $false
    $failsafeEver = $false

    # heading_stage-Stuck: skip_reason == heading_stage > 1s am Stueck
    $headingStageRunStart = $null
    $headingStageStuck = $false
    $maxHeadingStageRunSec = 0.0

    $lines = Get-Content -Path $LogPath -Encoding UTF8
    foreach ($rawLine in $lines) {
        if ($rawLine.StartsWith("#")) { continue }
        $line = $rawLine.Trim()
        if ($line -eq "") { continue }

        # Timestamp am Zeilenanfang: yyyy-MM-dd HH:mm:ss.fff
        if ($line -notmatch '^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})') { continue }
        $ts = [datetime]::ParseExact($Matches[1], "yyyy-MM-dd HH:mm:ss.fff", $null)

        $state = "?"
        $skip = "?"
        $navcurve = "false"
        $failsafe = "false"
        if ($line -match '\bstate=([^|]+)') { $state = $Matches[1].Trim() }
        if ($line -match '\bskip=([^|]+)') { $skip = $Matches[1].Trim() }
        if ($line -match '\bchosen_navcurve=([^|]+)') { $navcurve = $Matches[1].Trim() }
        if ($line -match '\bfailsafe=([^|]+)') { $failsafe = $Matches[1].Trim() }

        if ($state -eq "Active") { $everActive = $true }

        if ($state -eq "Active" -and $navcurve -eq "true") {
            $everNavcurveWhileActive = $true
        }
        if ($navcurve -eq "true" -and ($state -eq "Off" -or $state -eq "Disengaging")) {
            $stateDroppedDuringNavcurve = $true
        }
        if ($failsafe -eq "true") { $failsafeEver = $true }

        if ($skip -eq "heading_stage") {
            if ($null -eq $headingStageRunStart) {
                $headingStageRunStart = $ts
            } else {
                $runSec = ($ts - $headingStageRunStart).TotalSeconds
                if ($runSec -gt $maxHeadingStageRunSec) { $maxHeadingStageRunSec = $runSec }
                if ($runSec -gt 1.0) { $headingStageStuck = $true }
            }
        } else {
            if ($null -ne $headingStageRunStart) {
                $runSec = ($ts - $headingStageRunStart).TotalSeconds
                if ($runSec -gt $maxHeadingStageRunSec) { $maxHeadingStageRunSec = $runSec }
                if ($runSec -gt 1.0) { $headingStageStuck = $true }
            }
            $headingStageRunStart = $null
        }
    }

    if ($script:EverReachedActive) { $everActive = $true }
    if ($script:EngageFailed) { Write-Host "Hinweis: Engage in Phase 4 als FEHLVERSUCH markiert." -ForegroundColor Yellow }
    if ($script:RouteUnstableInStand) { Write-Host "Hinweis: Route war im Stand instabil (Phase 2)." -ForegroundColor Yellow }

    Write-Host ""
    Write-Host "Metriken:" -ForegroundColor White
    Write-Host ("  state je Active gewesen:           {0}" -f $everActive)
    Write-Host ("  NavCurve-Phase waehrend Active:    {0}" -f $everNavcurveWhileActive)
    Write-Host ("  State-Drop waehrend NavCurve:      {0}" -f $stateDroppedDuringNavcurve)
    Write-Host ("  Junction-Failsafe jemals aktiv:    {0}" -f $failsafeEver)
    Write-Host ("  skip_reason heading_stage > 1s:    {0} (max Run {1:N1}s)" -f $headingStageStuck, $maxHeadingStageRunSec)
    Write-Host ""

    $verdict = ""
    $color = "White"
    if (-not $everActive -or -not $everNavcurveWhileActive) {
        $verdict = "KEIN ENGAGE / KEIN JUNCTION-DURCHLAUF"
        $color = "Red"
    } elseif ($failsafeEver) {
        $verdict = "FAILSAFE GRIFF: C nicht ausreichend, Bremsnetz fing"
        $color = "Yellow"
    } elseif (-not $stateDroppedDuringNavcurve -and -not $headingStageStuck) {
        $verdict = "DURCHBRUCH: Fix C wirkt"
        $color = "Green"
    } else {
        $verdict = "TEILWEISE: NavCurve gesehen, aber State-Drop oder heading_stage-Stuck"
        $color = "Yellow"
    }

    Write-Host ("FAZIT: {0}" -f $verdict) -ForegroundColor $color
    Write-Host ("Log: {0}" -f $LogPath) -ForegroundColor DarkGray

    try {
        $summaryPath = Join-Path $OutDir ("fixc_junction_test_{0}_summary.txt" -f (Get-Date -Format "yyyyMMdd_HHmmss"))
        @(
            "fixc_junction_test Auswertung - $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')",
            "Verdict: $verdict",
            "everActive=$everActive everNavcurveWhileActive=$everNavcurveWhileActive",
            "stateDroppedDuringNavcurve=$stateDroppedDuringNavcurve failsafeEver=$failsafeEver",
            "headingStageStuck=$headingStageStuck maxHeadingStageRunSec=$maxHeadingStageRunSec",
            "engageFailed=$($script:EngageFailed) routeUnstableInStand=$($script:RouteUnstableInStand)",
            "Log: $LogPath"
        ) | Out-File -Encoding utf8 -FilePath $summaryPath
        Write-Host ("Summary: {0}" -f $summaryPath) -ForegroundColor DarkGray
    } catch {
        Write-Warning ("Summary-Datei nicht geschrieben: {0}" -f $_.Exception.Message)
    }
}

# â”€â”€ Ctrl+C sauber abfangen â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
$cancelHandler = [ConsoleCancelEventHandler]{
    param($sender, $e)
    $e.Cancel = $true
    $script:ShouldExit = $true
    Write-Host ""
    Write-Host "*** Strg+C - beende nach aktuellem Poll-Zyklus... ***" -ForegroundColor Yellow
}
[Console]::Add_CancelKeyPress($cancelHandler)

# â”€â”€ Binaries pruefen â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
foreach ($bin in @($BbExe, $EngageExe)) {
    if (-not (Test-Path $bin)) {
        Write-Error ("{0} nicht gefunden - zuerst 'cargo build-release' aus Repo-Root." -f $bin)
        exit 1
    }
}

Open-LogFile

try {
    # â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
    # PHASE 1 - Setup
    # â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
    Write-Phase "PHASE 1 - Setup"

    if ($UseTruckStart) {
        Write-Host "set-start --clear (Route-Start = aktuelle Truck-Position)"
        $rStart = Invoke-EngageCli @("set-start", "--clear")
        if (-not $rStart.Ok) {
            Write-Host "Abbruch: set-start --clear fehlgeschlagen." -ForegroundColor Red
            exit 1
        }
    }

    $goal = Get-GoalCoordinates
    $script:ResolvedGoalX = $goal.X
    $script:ResolvedGoalZ = $goal.Z

    if ($goal.Mode -eq "ahead") {
        Write-Host (Format-Multi "Auto-Ziel: {0} m voraus in Fahrtrichtung" $goal.AheadM)
        Write-Host (Format-Multi "  truck=({0:N1},{1:N1}) heading_rad={2:N3}" $goal.TruckX $goal.TruckZ $goal.HeadingRad)
        Write-Host (Format-Multi "  goal=({0:N1},{1:N1})" $goal.X $goal.Z)
    } else {
        Write-Host (Format-Multi "Festes Ziel: ({0},{1})" $goal.X $goal.Z)
    }

    Write-Host (Format-Multi "set-goal-pos --x={0} --z={1}" $goal.X $goal.Z)
    $rGoal = Invoke-EngageCli @("set-goal-pos", ("--x={0}" -f $goal.X), ("--z={0}" -f $goal.Z))
    if (-not $rGoal.Ok) {
        Write-Host "Abbruch: set-goal-pos fehlgeschlagen." -ForegroundColor Red
        exit 1
    }

    Write-Host ("set-cruise {0}" -f $CruiseKmh)
    $rCruise = Invoke-EngageCli @("set-cruise", "$CruiseKmh")
    if (-not $rCruise.Ok) {
        Write-Host "Abbruch: set-cruise fehlgeschlagen." -ForegroundColor Red
        exit 1
    }

    Start-Sleep -Milliseconds 500

    Write-Host ("Warte bis zu {0}s auf Route-Planung nach set-goal-pos ..." -f $RoutePlanWaitSec)
    $wait = Wait-RouterActive -TimeoutSec $RoutePlanWaitSec
    $pre = $wait.Pre
    if (-not $pre.router_active) {
        $rStatus = Invoke-EngageCli @("status")
        if ($rStatus.Ok) {
            $pre = Parse-EngageStatus $rStatus.Lines
        }
    }

    Write-Host ("  autopilot.state     = {0}" -f $pre.autopilot_state)
    Write-Host ("  telemetry_ok        = {0}" -f $pre.telemetry_ok)
    Write-Host ("  engine_running      = {0}" -f $pre.engine_running)
    Write-Host ("  critical_plugins    = {0}" -f $pre.critical_plugins)
    Write-Host ("  router_active       = {0}" -f $pre.router_active)

    if (-not $pre.telemetry_ok) {
        Write-Host "WARNUNG: telemetry_ok=false - ETS2/Telemetrie pruefen." -ForegroundColor Yellow
    }
    if (-not $pre.engine_running) {
        Write-Host "WARNUNG: engine_running=false - Motor an?" -ForegroundColor Yellow
    }
    if (-not $pre.critical_plugins) {
        Write-Host "WARNUNG: critical_plugins=false - Plugins/Daemon pruefen." -ForegroundColor Yellow
    }

    if (-not $pre.router_active) {
        Write-Host ""
        Write-Host "Route nicht aktiv, Ziel pruefen" -ForegroundColor Red
        $kv = Invoke-BbQuery "router.last_planning_result,router.last_planning_error_detail,router.waypoint_count,telemetry.position_x,telemetry.position_z,router.current_goal_uid,router.start_uid"
        $planRes = Get-OrDefault $kv "router.last_planning_result"
        if ($wait.PlanResult -ne "?" -and $wait.PlanResult -ne "") {
            $planRes = $wait.PlanResult
        }
        Write-Host ("  router.last_planning_result = {0}" -f $planRes)
        $planErr = Get-OrDefault $kv "router.last_planning_error_detail"
        if ($wait.PlanError -ne "") { $planErr = $wait.PlanError }
        Write-Host ("  router.last_planning_error_detail = {0}" -f $planErr)
        Write-Host ("  router.waypoint_count = {0}" -f (Get-OrDefault $kv "router.waypoint_count"))
        Write-Host ("  router.start_uid = {0}" -f (Get-OrDefault $kv "router.start_uid"))
        Write-Host ("  router.current_goal_uid = {0}" -f (Get-OrDefault $kv "router.current_goal_uid"))
        Write-Host ("  truck pos = ({0}, {1})" -f (Get-OrDefault $kv "telemetry.position_x"), (Get-OrDefault $kv "telemetry.position_z"))
        Write-Host ("  goal pos  = ({0:N1}, {1:N1}) mode={2}" -f $script:ResolvedGoalX, $script:ResolvedGoalZ, $script:GoalMode)
        if ($planRes -eq "no_path_found") {
            Write-Host ""
            Write-Host "Hinweis: Start- und Ziel-Knoten liegen vermutlich in getrennten Graph-Komponenten" -ForegroundColor Yellow
            Write-Host "  oder das Ziel ist von hier aus nicht erreichbar." -ForegroundColor Yellow
            if ($FixedGoal) {
                Write-Host "  Versuch ohne festes Ziel (empfohlen):" -ForegroundColor Cyan
                Write-Host "    .\scripts\fixc_junction_test.ps1 -AutoGoalAheadM 3000" -ForegroundColor Cyan
            } else {
                Write-Host "  Naeher am Ziel fahren / anderen Abschnitt waehlen, oder -AutoGoalAheadM erhoehen." -ForegroundColor Cyan
            }
        }
        Write-Host "router_active=false nach set-goal-pos - Abbruch." -ForegroundColor Red
        Write-LogFormatted "# PHASE1 ABORT router_active=false plan={0} err={1} goal_mode={2}" $planRes $planErr $script:GoalMode
        exit 1
    }

    Write-LogFormatted "# PHASE1 OK router_active=true goal=({0},{1}) mode={2} cruise={3}" $script:ResolvedGoalX $script:ResolvedGoalZ $script:GoalMode $CruiseKmh

    if ($script:ShouldExit) { throw [System.OperationCanceledException]::new("User cancel") }

    # â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
    # PHASE 2 - Route-Stabilitaet (3 s)
    # â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
    Write-Phase "PHASE 2 - Route-Stabilitaet (3 s)"

    $stabilityPolls = 15
    $routerDrops = 0
    for ($i = 1; $i -le $stabilityPolls; $i++) {
        $ra = Get-RouterActive
        $ts = Get-Date -Format "yyyy-MM-dd HH:mm:ss.fff"
        Write-Host ("  [{0}/{1}] {2} router_active={3}" -f $i, $stabilityPolls, $ts, $ra)
        if (-not $ra) { $routerDrops++ }
        if ($i -lt $stabilityPolls) { Start-Sleep -Milliseconds $PollIntervalMs }
        if ($script:ShouldExit) { break }
    }

    if ($routerDrops -gt 0) {
        $script:RouteUnstableInStand = $true
        Write-Host ""
        Write-Host "WARNUNG: Route instabil im Stand ($routerDrops/$stabilityPolls Polls false) - trotzdem weiter." -ForegroundColor Yellow
        Write-LogLine ("# PHASE2 WARN route_unstable_in_stand drops=$routerDrops/$stabilityPolls")
    } else {
        Write-Host "Route stabil im Stand (15/15 router_active=true)." -ForegroundColor Green
        Write-LogLine "# PHASE2 OK route_stable_in_stand"
    }

    if ($script:ShouldExit) { throw [System.OperationCanceledException]::new("User cancel") }

    # â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
    # PHASE 3 - Warte auf Fahrt (Mensch faehrt)
    # â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
    Write-Phase "PHASE 3 - Warte auf Fahrt"
    Write-Host ""
    Write-Host "Jetzt LOSFAHREN." -ForegroundColor Green
    Write-Host ("Auf ca. {0} km/h beschleunigen und Richtung naechste Junction/naechstes Kreuz halten." -f $CruiseKmh)
    Write-Host "Das Skript engagt automatisch, sobald du faehrst und engage moeglich ist."
    Write-Host ("Bedingung: speed_ms > {0}, engage_block_reason=ok, router_active=true" -f $MinEngageSpeedMs)
    Write-Host ""

    $waitKeys = "telemetry.speed_ms,lane_keeper.engage_block_reason,lane_keeper.engage_dist_m,router.active,autopilot.state"
    $deadline = (Get-Date).AddSeconds(120)
    $engageReady = $false

    while ((Get-Date) -lt $deadline -and -not $script:ShouldExit) {
        $ts = Get-Date -Format "yyyy-MM-dd HH:mm:ss.fff"
        $kv = Invoke-BbQuery $waitKeys
        $routerActive = (Get-OrDefault $kv "router.active" "false") -eq "true"
        $line = Format-WaitLine $kv $routerActive $ts
        Write-Host $line

        if (Test-EngageReady $kv $routerActive) {
            $engageReady = $true
            Write-Host ""
            Write-Host "Engage-Bedingungen erfuellt - Phase 4." -ForegroundColor Green
            Write-LogFormatted "# PHASE3 engage_ready {0}" $line
            break
        }

        Start-Sleep -Milliseconds $PollIntervalMs
    }

    if (-not $engageReady) {
        if ($script:ShouldExit) {
            throw [System.OperationCanceledException]::new("User cancel")
        }
        Write-Host ""
        Write-Host "Timeout 120s: Engage-Bedingungen nicht erfuellt." -ForegroundColor Red
        Write-Host "Tipps: Motor an, auf Strecke fahren (>5 m/s), Route/Ziel pruefen, nicht im Stand engagen."
        Write-LogLine "# PHASE3 TIMEOUT engage_conditions_not_met"
        exit 1
    }

    if ($script:ShouldExit) { throw [System.OperationCanceledException]::new("User cancel") }

    # â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
    # PHASE 4 - Engage
    # â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
    Write-Phase "PHASE 4 - Engage"

    $rEngage = Invoke-EngageCli @("engage")
    if (-not $rEngage.Ok) {
        $script:EngageFailed = $true
        Write-Host "WARNUNG: engage-cli engage fehlgeschlagen." -ForegroundColor Yellow
    } else {
        Write-Host "engage angefordert - warte auf Active (max 2 s)..."
    }

    $activePolls = 10
    $reachedActive = $false
    for ($i = 1; $i -le $activePolls; $i++) {
        Start-Sleep -Milliseconds $PollIntervalMs
        $kv = Invoke-BbQuery "autopilot.state,autopilot.fault_reason,lane_keeper.engage_block_reason"
        $state = Get-OrDefault $kv "autopilot.state" "?"
        Write-Host ("  [{0}/{1}] autopilot.state={2}" -f $i, $activePolls, $state)
        if ($state -eq "Active") {
            $reachedActive = $true
            $script:EverReachedActive = $true
            break
        }
        if ($script:ShouldExit) { break }
    }

    if (-not $reachedActive) {
        $script:EngageFailed = $true
        $kv = Invoke-BbQuery "autopilot.state,autopilot.fault_reason,lane_keeper.engage_block_reason"
        $state = Get-OrDefault $kv "autopilot.state" "?"
        $fault = Get-OrDefault $kv "autopilot.fault_reason" "?"
        $block = Get-OrDefault $kv "lane_keeper.engage_block_reason" "?"
        Write-Host ("WARNUNG: Engage nicht erfolgreich (state={0})" -f $state) -ForegroundColor Yellow
        Write-Host ("  autopilot.fault_reason={0}" -f $fault)
        Write-Host ("  lane_keeper.engage_block_reason={0}" -f $block)
        Write-Host "Phase 5 trotzdem - FEHLVERSUCH markiert." -ForegroundColor Yellow
        Write-LogFormatted "# PHASE4 FAIL state={0} fault={1} engage_block={2}" $state $fault $block
    } else {
        Write-Host "Autopilot Active." -ForegroundColor Green
        Write-LogLine "# PHASE4 OK autopilot.state=Active"
    }

    if ($script:ShouldExit) { throw [System.OperationCanceledException]::new("User cancel") }

    # â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
    # PHASE 5 - Junction-Poll
    # â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
    Write-Phase "PHASE 5 - Junction-Poll"
    Write-Host "Fahre durch die Junction. Strg+C zum vorzeitigen Beenden."
    Write-Host ""

    $script:Phase5Started = $true
    $junctionKeys = @(
        "autopilot.state",
        "lane_keeper.skip_reason",
        "lane_keeper.stage",
        "arbitration.steering_offer_count",
        "arbitration.steering_winner_plugin",
        "lane_keeper.nearest_route_filtered",
        "lane_keeper.total_route_navcurve_count",
        "lane_keeper.chosen_segment_is_navcurve",
        "lane_keeper.junction_failsafe_active",
        "telemetry.speed_ms",
        "telemetry.position_x",
        "telemetry.position_z"
    ) -join ","

    $prevState = "?"
    $exitAfterStateChange = $null

    while (-not $script:ShouldExit) {
        $ts = Get-Date -Format "yyyy-MM-dd HH:mm:ss.fff"
        $kv = Invoke-BbQuery $junctionKeys
        $state = Get-OrDefault $kv "autopilot.state" "?"

        if ($state -eq "Active") { $script:EverReachedActive = $true }

        $marker = ""
        if ($prevState -eq "Active" -and $state -ne "Active" -and $state -ne "?") {
            $marker = "STATE CHANGE $prevState -> $state"
            $exitAfterStateChange = (Get-Date).AddSeconds(3)
            Write-Host "State-Wechsel erkannt - 3s Nachlauf-Poll..." -ForegroundColor Yellow
        }

        $line = Format-JunctionLine $kv $ts $marker
        Write-LogLine $line

        $prevState = $state
        Start-Sleep -Milliseconds $PollIntervalMs

        if ($null -ne $exitAfterStateChange -and (Get-Date) -ge $exitAfterStateChange) {
            Write-Host "3s Nachlauf abgeschlossen - Ende Phase 5." -ForegroundColor Yellow
            break
        }
    }

    Write-LogLine "# PHASE5 END"

} catch [System.OperationCanceledException] {
    Write-Host "Abbruch durch Benutzer." -ForegroundColor Yellow
    Write-LogLine "# USER_CANCEL"
} catch {
    Write-Host ("Unerwarteter Fehler: {0}" -f $_.Exception.Message) -ForegroundColor Red
    Write-LogFormatted "# ERROR {0}" $_.Exception.Message
} finally {
    Close-LogFile
    if ($null -ne $script:LogFile) {
        Invoke-Phase6Evaluation -LogPath $script:LogFile
    }
    [Console]::Remove_CancelKeyPress($cancelHandler)
}

