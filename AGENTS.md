> ⚠️ WICHTIG: Du läufst auf Linux (Ubuntu 26.04). Verwende NUR bash-Befehle. NIEMALS Windows/PowerShell-Befehle verwenden.

# TruckPilot — Agent Environment

## Du läufst hier
- Auf einem **Ubuntu 26.04 Linux-Server** (Geekom mini-PC)
- NICHT auf Windows
- Alle Befehle die du ausführst laufen in **bash** auf diesem Linux-Server

## ❌ Diese Befehle funktionieren NICHT — niemals verwenden
- `Get-ChildItem`, `Get-Content`, `Get-Item`, `Get-Process` (PowerShell)
- `Where-Object`, `Select-Object`, `Sort-Object`, `ForEach-Object` (PowerShell)
- `Set-Content`, `Out-File`, `Copy-Item`, `Move-Item`, `Remove-Item` (PowerShell)
- `$env:...`, `$PSVersionTable`, `Write-Host`, `Write-Output` (PowerShell)
- `where.exe`, `python.exe`, `cmd.exe` (Windows-Executables)
- Pfade mit `C:\`, `D:\`, Backslashes `\`
- `& "C:\..."` (PowerShell-Aufruf-Operator)
- `python` → heißt hier `python3`

## ✅ Stattdessen bash verwenden
- `ls`, `find`, `cat`, `cp`, `mv`, `rm`, `mkdir`
- `grep`, `awk`, `sed`, `sort`, `wc`
- `python3` statt `python`
- Pfade immer mit `/` (forward slash)

## Pfade
- Projekt-Root: `/home/geekom/TruckPilot`
- Home: `/home/geekom`
- Temp: `/tmp`

## Der Windows-PC
- Ein Windows-PC synchronisiert via Mutagen mit `/home/geekom/TruckPilot`
- ETS2 liegt auf dem PC unter `C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2`
- ETS2 existiert NICHT auf diesem Linux-Server
- Code der ETS2-Dateien lesen muss, muss auf dem PC ausgeführt werden
- Dateien die du hier erstellst, landen automatisch auch auf dem PC (via Mutagen)

## Beim Erstellen von Dateien
- Immer Linux-Pfad verwenden: `/home/geekom/TruckPilot/dateiname`
- Nach dem Schreiben mit `ls -la <pfad>` bestätigen dass die Datei existiert
- Niemals Windows-Pfad als Dateiname verwenden

## Build
- Sprache: Rust
- Build: `cargo build`
- Test: `cargo test`
- Run: `cargo run`
