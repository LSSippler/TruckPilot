param(
    [string]$GraphPath = "output\graph.json"
)

if (-not (Test-Path $GraphPath)) {
    Write-Host "[ERROR] graph.json not found: $GraphPath" -ForegroundColor Red
    exit 1
}

try {
    # Regex-Extraktion – kein JSON-Parsing (vermeidet Speicherprobleme bei 222k Nodes)
    $raw = Get-Content -Raw $GraphPath

    # Zuerst: erstes Edge-Paar
    $edgeMatch = [regex]::Match($raw, '"from_node_uid"\s*:\s*(\d+).*?"to_node_uid"\s*:\s*(\d+)', [System.Text.RegularExpressions.RegexOptions]::Singleline)
    if ($edgeMatch.Success) {
        $startUid = $edgeMatch.Groups[1].Value
        $goalUid  = $edgeMatch.Groups[2].Value
    } else {
        # Fallback: erster und letzter Node
        $nodeMatches = [regex]::Matches($raw, '"uid"\s*:\s*(\d+)')
        if ($nodeMatches.Count -ge 2) {
            $startUid = $nodeMatches[0].Groups[1].Value
            $goalUid  = $nodeMatches[$nodeMatches.Count - 1].Groups[1].Value
        }
    }

    if (-not $startUid -or -not $goalUid) {
        throw "No usable nodes or edges found in graph file."
    }

    Write-Host "Start UID: $startUid"
    Write-Host "Goal UID:  $goalUid"
    Write-Host ""
    Write-Host "PowerShell vars:" -ForegroundColor Cyan
    Write-Host "`$start = $startUid"
    Write-Host "`$goal  = $goalUid"

    # Maschinenlesbare Ausgabe
    Write-Output "START_UID=$startUid"
    Write-Output "GOAL_UID=$goalUid"

} catch {
    $errMsg = $_.Exception.Message
    Write-Host "[ERROR] Failed to parse $GraphPath`: $errMsg" -ForegroundColor Red
    exit 1
}
