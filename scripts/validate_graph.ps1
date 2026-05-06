param(
    [Parameter(Mandatory = $true, Position = 0)]
    [string]$GraphPath
)

$ErrorActionPreference = 'Stop'

if (-not (Test-Path -LiteralPath $GraphPath)) {
    [Console]::Error.WriteLine("Fehler: Datei nicht gefunden: $GraphPath")
    exit 1
}

$raw = Get-Content -LiteralPath $GraphPath -Raw
$lines = @(Get-Content -LiteralPath $GraphPath)

try {
    $graph = $raw | ConvertFrom-Json
} catch {
    [Console]::Error.WriteLine("Fehler: Ungültiges JSON: $($_.Exception.Message)")
    exit 1
}

if ($null -eq $graph.nodes -or $null -eq $graph.edges) {
    [Console]::Error.WriteLine("Fehler: JSON muss 'nodes' und 'edges' enthalten.")
    exit 1
}

$graph.nodes = @($graph.nodes)
$graph.edges = @($graph.edges)

function Find-LineNumber {
    param(
        [string[]]$FileLines,
        [string]$Field,
        [object]$Value,
        [int]$StartLine = 1
    )

    $escapedField = [regex]::Escape($Field)
    $pattern = $null

    if ($null -eq $Value) {
        $pattern = '"' + $escapedField + '"\s*:\s*null'
    } else {
        $valueText = if ($Value -is [System.IFormattable]) {
            $Value.ToString($null, [System.Globalization.CultureInfo]::InvariantCulture)
        } else {
            [string]$Value
        }
        if ([string]::IsNullOrWhiteSpace($valueText)) {
            return -1
        }
        $escapedValue = [regex]::Escape($valueText)
        $pattern = '"' + $escapedField + '"\s*:\s*("?' + $escapedValue + '"?)(?:\s*[,}]|$)'
    }

    $start = [Math]::Max(0, $StartLine - 1)
    for ($i = $start; $i -lt $FileLines.Count; $i++) {
        if ($FileLines[$i] -match $pattern) {
            return ($i + 1)
        }
    }
    return -1
}

function Normalize-Uid {
    param($Value)

    if ($null -eq $Value) { return $null }
    if ($Value -is [string]) { return $Value.Trim().ToLowerInvariant() }

    if ($Value -is [System.IFormattable]) {
        return $Value.ToString($null, [System.Globalization.CultureInfo]::InvariantCulture).ToLowerInvariant()
    }

    return ([string]$Value).ToLowerInvariant()
}

function To-InvariantString {
    param($Value)
    if ($null -eq $Value) { return "" }
    if ($Value -is [System.IFormattable]) {
        return $Value.ToString($null, [System.Globalization.CultureInfo]::InvariantCulture)
    }
    return [string]$Value
}

function Is-HexUidString {
    param($Value)

    if ($Value -isnot [string]) {
        return $false
    }
    return ($Value -match '^0x[0-9a-fA-F]+$')
}

$violations = New-Object System.Collections.Generic.List[string]
$checksPassed = 0

$nodeUidSet = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::OrdinalIgnoreCase)
$dupNodes = 0
foreach ($n in $graph.nodes) {
    $uid = Normalize-Uid $n.uid
    if ($null -ne $uid) {
        if (-not $nodeUidSet.Add($uid)) {
            $dupNodes++
            $lineNo = Find-LineNumber -FileLines $lines -Field 'uid' -Value $n.uid
            $violations.Add("[Check0] Line ${lineNo}: duplicate node uid '$uid'.")
        }
    }
}
if ($dupNodes -eq 0) { $checksPassed++ }

# 1) from/to must exist in nodes
$missingRefs = 0
for ($edgeIndex = 0; $edgeIndex -lt $graph.edges.Count; $edgeIndex++) {
    $e = $graph.edges[$edgeIndex]
    $fromUid = Normalize-Uid $e.from_node_uid
    $toUid = Normalize-Uid $e.to_node_uid
    $fromLineNo = Find-LineNumber -FileLines $lines -Field 'from_node_uid' -Value $fromUid
    $toLineNo = Find-LineNumber -FileLines $lines -Field 'to_node_uid' -Value $toUid

    if (-not $nodeUidSet.Contains($fromUid)) {
        $missingRefs++
        $violations.Add("[Check1] edge[$edgeIndex], line ${fromLineNo}: from_node_uid '$fromUid' fehlt in nodes.")
    }
    if (-not $nodeUidSet.Contains($toUid)) {
        $missingRefs++
        $violations.Add("[Check1] edge[$edgeIndex], line ${toLineNo}: to_node_uid '$toUid' fehlt in nodes.")
    }
}
if ($missingRefs -eq 0) { $checksPassed++ }

# 2) duplicate edges by from+to+road_uid
$seenEdges = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::OrdinalIgnoreCase)
$dupes = 0
for ($edgeIndex = 0; $edgeIndex -lt $graph.edges.Count; $edgeIndex++) {
    $e = $graph.edges[$edgeIndex]
    $fromUid = Normalize-Uid $e.from_node_uid
    $toUid = Normalize-Uid $e.to_node_uid
    $roadUid = if ($null -eq $e.road_uid) { '<null>' } else { To-InvariantString $e.road_uid }
    $key = "$fromUid|$toUid|$roadUid"
    if (-not $seenEdges.Add($key)) {
        $dupes++
        $lineNo = Find-LineNumber -FileLines $lines -Field 'road_uid' -Value $e.road_uid
        $violations.Add("[Check2] edge[$edgeIndex], line ${lineNo}: doppelte Edge '$key'.")
    }
}
if ($dupes -eq 0) { $checksPassed++ }

# 3) no negative distance_m
$negDistances = 0
for ($edgeIndex = 0; $edgeIndex -lt $graph.edges.Count; $edgeIndex++) {
    $e = $graph.edges[$edgeIndex]
    $distance = 0.0
    $distanceText = To-InvariantString $e.distance_m
    $parseOk = [double]::TryParse($distanceText, [System.Globalization.NumberStyles]::Float, [System.Globalization.CultureInfo]::InvariantCulture, [ref]$distance)
    if (-not $parseOk) {
        $negDistances++
        $lineNo = Find-LineNumber -FileLines $lines -Field 'distance_m' -Value $distanceText
        $violations.Add("[Check3] edge[$edgeIndex], line ${lineNo}: distance_m '$($e.distance_m)' ist keine Zahl.")
        continue
    }

    if ([double]::IsNaN($distance) -or [double]::IsInfinity($distance)) {
        $negDistances++
        $lineNo = Find-LineNumber -FileLines $lines -Field 'distance_m' -Value $distanceText
        $violations.Add("[Check3] edge[$edgeIndex], line ${lineNo}: distance_m '$($e.distance_m)' ist keine endliche Zahl.")
        continue
    }

    if ($distance -lt 0.0) {
        $negDistances++
        $lineNo = Find-LineNumber -FileLines $lines -Field 'distance_m' -Value $distanceText
        $violations.Add("[Check3] edge[$edgeIndex], line ${lineNo}: negative distance_m=$($e.distance_m) (from=$($e.from_node_uid), to=$($e.to_node_uid)).")
    }
}
if ($negDistances -eq 0) { $checksPassed++ }

# 4) all UIDs must be hex strings starting with 0x (strict mode)
$badHex = 0
for ($nodeIndex = 0; $nodeIndex -lt $graph.nodes.Count; $nodeIndex++) {
    $n = $graph.nodes[$nodeIndex]
    if (-not (Is-HexUidString $n.uid)) {
        $badHex++
        $lineNo = Find-LineNumber -FileLines $lines -Field 'uid' -Value ([string]$n.uid)
        $violations.Add("[Check4] node[$nodeIndex], line ${lineNo}: node uid '$($n.uid)' ist kein Hex-String (0x...).")
    }
}
for ($edgeIndex = 0; $edgeIndex -lt $graph.edges.Count; $edgeIndex++) {
    $e = $graph.edges[$edgeIndex]
    if (-not (Is-HexUidString $e.from_node_uid)) {
        $badHex++
        $lineNo = Find-LineNumber -FileLines $lines -Field 'from_node_uid' -Value ([string]$e.from_node_uid)
        $violations.Add("[Check4] edge[$edgeIndex], line ${lineNo}: from_node_uid '$($e.from_node_uid)' ist kein Hex-String (0x...).")
    }
    if (-not (Is-HexUidString $e.to_node_uid)) {
        $badHex++
        $lineNo = Find-LineNumber -FileLines $lines -Field 'to_node_uid' -Value ([string]$e.to_node_uid)
        $violations.Add("[Check4] edge[$edgeIndex], line ${lineNo}: to_node_uid '$($e.to_node_uid)' ist kein Hex-String (0x...).")
    }
    if ($null -ne $e.road_uid -and -not (Is-HexUidString $e.road_uid)) {
        $badHex++
        $lineNo = Find-LineNumber -FileLines $lines -Field 'road_uid' -Value ([string]$e.road_uid)
        $violations.Add("[Check4] edge[$edgeIndex], line ${lineNo}: road_uid '$($e.road_uid)' ist kein Hex-String (0x...).")
    }
}
if ($badHex -eq 0) { $checksPassed++ }

# 5) no self loops
$selfLoops = 0
for ($edgeIndex = 0; $edgeIndex -lt $graph.edges.Count; $edgeIndex++) {
    $e = $graph.edges[$edgeIndex]
    $fromUid = Normalize-Uid $e.from_node_uid
    $toUid = Normalize-Uid $e.to_node_uid
    if ($fromUid -eq $toUid) {
        $selfLoops++
        $lineNo = Find-LineNumber -FileLines $lines -Field 'from_node_uid' -Value $fromUid
        $violations.Add("[Check5] edge[$edgeIndex], line ${lineNo}: Self-loop bei uid '$fromUid'.")
    }
}
if ($selfLoops -eq 0) { $checksPassed++ }

# 6) no isolated nodes
$connectedNodeUids = [System.Collections.Generic.HashSet[string]]::new([System.StringComparer]::OrdinalIgnoreCase)
foreach ($e in $graph.edges) {
    [void]$connectedNodeUids.Add((Normalize-Uid $e.from_node_uid))
    [void]$connectedNodeUids.Add((Normalize-Uid $e.to_node_uid))
}

$isolated = 0
for ($nodeIndex = 0; $nodeIndex -lt $graph.nodes.Count; $nodeIndex++) {
    $n = $graph.nodes[$nodeIndex]
    $uid = Normalize-Uid $n.uid
    if (-not $connectedNodeUids.Contains($uid)) {
        $isolated++
        $lineNo = Find-LineNumber -FileLines $lines -Field 'uid' -Value ([string]$n.uid)
        $violations.Add("[Check6] node[$nodeIndex], line ${lineNo}: isolierter Node '$uid'.")
    }
}
if ($isolated -eq 0) { $checksPassed++ }

if ($violations.Count -gt 0) {
    $violations | ForEach-Object { Write-Host $_ }
}

Write-Host ""
Write-Host "$checksPassed Prüfungen bestanden, $($violations.Count) Verletzungen gefunden."

if ($violations.Count -gt 0) {
    exit 2
}

exit 0
