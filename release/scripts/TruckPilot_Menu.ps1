# ============================================================
# TruckPilot - Kombiniertes Menü (Pre-Setup + Live-Test) v1.3
# ============================================================

$ErrorActionPreference = "Stop"

function Show-Menu {
    Clear-Host
    Write-Host "==============================================" -ForegroundColor Cyan
    Write-Host "          TruckPilot - Hauptmenü" -ForegroundColor Cyan
    Write-Host "==============================================" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "  1. Pre-ETS2 Setup (vor dem Spielstart)" -ForegroundColor White
    Write-Host "  2. Live-Test starten (nach ETS2-Start)" -ForegroundColor White
    Write-Host "  3. Beides nacheinander ausführen" -ForegroundColor White
    Write-Host "  4. Beenden" -ForegroundColor White
    Write-Host ""
}

function Run-PreSetup {
    Write-Host "`n=== Pre-ETS2 Setup ===" -ForegroundColor Cyan

    if (-not (Test-Path "C:\Program Files\vJoy\x64\vJoyInterface.dll")) {
        Write-Host "[FEHLER] vJoy nicht gefunden!" -ForegroundColor Red
        return
    }
    Write-Host "[OK] vJoy gefunden" -ForegroundColor Green

    # Kill hanging cargo/rustc processes only (keep target folder for speed)
    Write-Host "[INFO] Incremental-Build..." -ForegroundColor Yellow
    $currentPid = $PID
    Get-Process -Name "cargo","rustc" -ErrorAction SilentlyContinue |
        Where-Object { $_.Id -ne $currentPid } |
        Stop-Process -Force -ErrorAction SilentlyContinue

    cargo build --release

    if ($LASTEXITCODE -ne 0) {
        Write-Host "[WARN] Build fehlgeschlagen, versuche mit clean..." -ForegroundColor Yellow
        cargo clean
        cargo build --release
    }

    if ($LASTEXITCODE -ne 0) {
        Write-Host "[FEHLER] Build fehlgeschlagen!" -ForegroundColor Red
        return
    }
    Write-Host "[OK] Rust-Build erfolgreich" -ForegroundColor Green

    # DLL installieren
    Write-Host "[INFO] Installiere Telemetrie-DLL..." -ForegroundColor Yellow
    if (Test-Path ".\TruckPilot.TelemetryDLL\install_dll.ps1") {
        & ".\TruckPilot.TelemetryDLL\install_dll.ps1"
    }

    if (-not (Test-Path "output\graph.json")) {
        Write-Host "[INFO] Erzeuge graph.json..." -ForegroundColor Yellow
        dotnet run --project TruckPilot.NET\TruckPilot.CLI -- --hashfs-sectors "C:\temp\ets2_sectors" | Out-Null
    }
    Write-Host "[OK] Pre-Setup abgeschlossen`n" -ForegroundColor Green
}

function Run-LiveTest {
    Write-Host "`n=== Live-Test wird gestartet ===" -ForegroundColor Cyan

    $ets2 = Get-Process -Name "eurotrucks2" -ErrorAction SilentlyContinue
    if (-not $ets2) {
        Write-Host "[WARN] ETS2 läuft nicht. Bitte starte ETS2 und setze dich in einen Lkw." -ForegroundColor Yellow
        Read-Host "Drücke Enter, wenn du bereit bist"
    }

    try {
        $routeOutput = .\scripts\pick_route_uids.ps1 -GraphPath "output\graph.json" 2>&1

        if ($LASTEXITCODE -ne 0) {
            Write-Host "[FEHLER] pick_route_uids.ps1 ist fehlgeschlagen!" -ForegroundColor Red
            return
        }

        $startLine = $routeOutput | Where-Object { $_ -match "^START_UID=" }
        $goalLine  = $routeOutput | Where-Object { $_ -match "^GOAL_UID=" }

        if (-not $startLine -or -not $goalLine) {
            Write-Host "[FEHLER] Konnte Start- oder Goal-UID nicht finden!" -ForegroundColor Red
            return
        }

        $startUid = ($startLine -replace "^START_UID=", "").Trim()
        $goalUid  = ($goalLine  -replace "^GOAL_UID=", "").Trim()

    } catch {
        Write-Host "[FEHLER] $($_.Exception.Message)" -ForegroundColor Red
        return
    }

    Write-Host "Start: $startUid" -ForegroundColor Green
    Write-Host "Goal:  $goalUid"  -ForegroundColor Green
    Write-Host ""

    # === Korrekt: --telemetry-server shm ===
    $GraphPath = "output\graph.json"
    $cmd = "cargo run --release --bin truckpilot -- --graph-json `"$GraphPath`" --start $startUid --goal $goalUid --vjoy-device 1 --telemetry-server shm -v"

    Write-Host "[INFO] Starte Autopilot mit SHM-Telemetrie..." -ForegroundColor Yellow
    Start-Process powershell -ArgumentList "-NoExit", "-Command", $cmd

    Write-Host "[OK] Autopilot wurde in einem neuen Fenster gestartet." -ForegroundColor Green
}

# ==================== HAUPTMENÜ ====================

while ($true) {
    Show-Menu
    $choice = Read-Host "Bitte Auswahl treffen (1-4)"

    try {
        switch ($choice) {
            "1" { Run-PreSetup }
            "2" { Run-LiveTest }
            "3" { 
                Run-PreSetup
                Write-Host "`n>>> Jetzt bitte ETS2 starten und im Lkw sitzen <<<" -ForegroundColor Yellow
                Read-Host "Drücke Enter, wenn ETS2 bereit ist"
                Run-LiveTest 
            }
            "4" { 
                Write-Host "Auf Wiedersehen!" -ForegroundColor Cyan
                break
            }
            default { 
                Write-Host "Ungültige Eingabe!" -ForegroundColor Red 
            }
        }
    } catch {
        Write-Host "[FEHLER] $($_.Exception.Message)" -ForegroundColor Red
        Write-Host "[INFO] Drücke eine Taste, um zum Menü zurückzukehren..."
        Read-Host
        continue
    }

    Write-Host ""
    Read-Host "Drücke Enter um zurück zum Menü zu kommen"
}

Write-Host "`n[INFO] Skript beendet. Drücke eine Taste, um das Fenster zu schließen..."
Read-Host