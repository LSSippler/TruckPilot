using TruckPilot.Core.Export;
using TruckPilot.Core.Models;
using TruckPilot.Core.Parsers;
using TruckPilot.Core.Telemetry;
using System.Diagnostics;

string? ets2Dir = null, textMap = null, hashfsSectors = null, startStr = null, goalStr = null;
bool verbose = false;
bool checkTelemetryDll = false;

for (int i = 0; i < args.Length; i++)
{
    switch (args[i])
    {
        case "--ets2-dir" when i + 1 < args.Length: ets2Dir = args[++i]; break;
        case "--hashfs-sectors" when i + 1 < args.Length: hashfsSectors = args[++i]; break;
        case "--text-map-file" when i + 1 < args.Length: textMap = args[++i]; break;
        case "--start" when i + 1 < args.Length: startStr = args[++i]; break;
        case "--goal" when i + 1 < args.Length: goalStr = args[++i]; break;
        case "-v": case "--verbose": verbose = true; break;
        case "--check-telemetry-dll": checkTelemetryDll = true; break;
        case "--help": PrintHelp(); return;
    }
}

if (checkTelemetryDll)
{
    var ok = DllSanityCheck.PerformCheck();
    Console.WriteLine(ok ? "OK" : "FAIL");
    Environment.Exit(ok ? 0 : 1);
}

ulong? startUid = startStr != null ? ParseUid(startStr) : null;
ulong? goalUid = goalStr != null ? ParseUid(goalStr) : null;

List<MapNode> nodes;
List<MapRoad> roads;
List<MapPrefab> prefabs = new();

if (hashfsSectors != null)
{
    if (verbose) Console.Error.WriteLine($"Parsing HashFS sectors from: {hashfsSectors}");
    var map = MapParser.ParseSectorDirectory(hashfsSectors, verbose);
    nodes = map.Nodes;
    roads = map.Roads;
    prefabs = map.Prefabs;
    if (verbose)
        Console.WriteLine($"Parsed: {nodes.Count} nodes, {roads.Count} roads, {prefabs.Count} prefabs, {map.Companies.Count} companies, {map.Signs.Count} signs");
}
else if (ets2Dir != null)
{
    var baseMap = Path.Combine(ets2Dir, "base_map.scs");
    if (File.Exists(baseMap))
    {
        var tempDir = Path.Combine(Path.GetTempPath(), $"ets2_sectors_{Guid.NewGuid()}");
        Directory.CreateDirectory(tempDir);
        try
        {
            if (verbose) Console.Error.WriteLine($"Extracting sectors via HashFS reader: {baseMap}");
            RunHashFsExtract(baseMap, tempDir, verbose);
            if (verbose) Console.Error.WriteLine($"Parsing extracted sectors: {tempDir}");
            var map = MapParser.ParseSectorDirectory(tempDir, verbose);
            nodes = map.Nodes;
            roads = map.Roads;
            prefabs = map.Prefabs;
            if (verbose)
                Console.WriteLine($"Parsed: {nodes.Count} nodes, {roads.Count} roads, {prefabs.Count} prefabs, {map.Companies.Count} companies, {map.Signs.Count} signs");
        }
        finally
        {
            if (Directory.Exists(tempDir)) Directory.Delete(tempDir, true);
        }
    }
    else
    {
        if (verbose) Console.Error.WriteLine($"Parsing ETS2 map from: {ets2Dir}");
        var map = MapParser.ParseEts2Directory(ets2Dir, verbose);
        nodes = map.Nodes;
        roads = map.Roads;
        prefabs = map.Prefabs;
        if (verbose)
            Console.WriteLine($"Parsed: {nodes.Count} nodes, {roads.Count} roads, {prefabs.Count} prefabs, {map.Companies.Count} companies, {map.Signs.Count} signs");
    }
}
else if (textMap != null)
{
    if (verbose) Console.WriteLine($"Loading text map from: {textMap}");
    (nodes, roads, prefabs) = MapParser.ParseTextMapFile(textMap);
    if (verbose) Console.WriteLine($"Loaded: {nodes.Count} nodes, {roads.Count} roads, {prefabs.Count} prefabs");
}
else
{
    // Built-in test fixture
    nodes = new() { new(){Uid=1,X=0,Y=0,Z=0}, new(){Uid=2,X=100,Y=0,Z=0}, new(){Uid=3,X=200,Y=0,Z=0}, new(){Uid=4,X=200,Y=0,Z=100}, new(){Uid=5,X=100,Y=0,Z=100} };
    roads = new() { new(){Uid=0xA,NodeUids=new(){1,2,3},LaneCountForward=2,LaneCountBackward=2,SpeedLimit=80}, new(){Uid=0xB,NodeUids=new(){3,4},LaneCountForward=1,LaneCountBackward=1,SpeedLimit=50}, new(){Uid=0xC,NodeUids=new(){4,5,2}} };
    if (verbose) Console.WriteLine($"Using built-in test map ({nodes.Count} nodes, {roads.Count} roads)");
}

if (verbose) Console.WriteLine($"TruckPilot.NET v0.1.0 — Nodes: {nodes.Count}, Roads: {roads.Count}");
if (verbose)
{
    Console.WriteLine($"  Has forward lanes: {roads.Any(r => r.LaneCountForward > 0)}");
    Console.WriteLine($"  Prefab nodes > 2: {prefabs.Any(p => p.NodeUids.Count > 2)}");
}
Pipeline.Run(nodes, roads, prefabs, startUid, goalUid, verbose);

static ulong ParseUid(string s)
{
    try
    {
        if (s.StartsWith("0x", StringComparison.OrdinalIgnoreCase))
            return ulong.Parse(s[2..], System.Globalization.NumberStyles.HexNumber);
        return ulong.Parse(s);
    }
    catch (Exception ex)
    {
        Console.Error.WriteLine($"Invalid UID '{s}': {ex.Message}");
        Environment.Exit(1);
        return 0;
    }
}

static void PrintHelp()
{
    Console.WriteLine("TruckPilot.NET — ETS2 Map Parser & Autopilot");
    Console.WriteLine("  --ets2-dir <path>       ETS2 installation directory");
    Console.WriteLine("  --hashfs-sectors <dir>  Directory with extracted map/*/*.base files");
    Console.WriteLine("  --text-map-file <path>  Text-format map sector file");
    Console.WriteLine("  --start <uid>           Start node UID (hex: 0x1, dec: 1)");
    Console.WriteLine("  --goal <uid>            Goal node UID");
    Console.WriteLine("  -v, --verbose           Detailed output");
    Console.WriteLine("  --check-telemetry-dll   Check telemetry shared memory");
    Console.WriteLine("  --help                  This help");
}

static void RunHashFsExtract(string baseMapPath, string outDir, bool verbose)
{
    var args = $"-m ets2_hashfs extract \"{baseMapPath}\" --sectors --out \"{outDir}\"";
    if (TryRunPython("python3", args, verbose, out var exitCode))
    {
        if (exitCode != 0)
            throw new InvalidOperationException($"HashFS reader failed with exit code {exitCode}.");
        return;
    }

    if (TryRunPython("python", args, verbose, out exitCode))
    {
        if (exitCode != 0)
            throw new InvalidOperationException($"HashFS reader failed with exit code {exitCode}.");
        return;
    }

    throw new InvalidOperationException("Failed to start HashFS reader (python3/python not found).");
}

static bool TryRunPython(string exe, string args, bool verbose, out int exitCode)
{
    exitCode = -1;
    try
    {
        var psi = new ProcessStartInfo
        {
            FileName = exe,
            Arguments = args,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            UseShellExecute = false
        };

        using var proc = Process.Start(psi);
        if (proc == null) return false;

        var output = proc.StandardOutput.ReadToEnd();
        var error = proc.StandardError.ReadToEnd();
        if (!proc.WaitForExit(120000))
        {
            try { proc.Kill(); } catch { }
            throw new InvalidOperationException("HashFS reader timed out.");
        }
        exitCode = proc.ExitCode;

        if (verbose)
        {
            if (!string.IsNullOrWhiteSpace(output)) Console.Error.WriteLine(output.TrimEnd());
            if (!string.IsNullOrWhiteSpace(error)) Console.Error.WriteLine(error.TrimEnd());
        }
        return true;
    }
    catch (System.ComponentModel.Win32Exception)
    {
        return false;
    }
}
