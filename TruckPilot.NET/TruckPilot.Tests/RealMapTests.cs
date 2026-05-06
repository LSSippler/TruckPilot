using System.IO.Compression;
using System.Diagnostics;
using TruckPilot.Core.Parsers;

namespace TruckPilot.Tests;

public class RealMapTests
{
    [Fact(Skip = "Requires ETS2 installation. Set --ets2-dir for manual test.")]
    public void ParseRealEts2Map_ShouldFindManyItems()
    {
        var dir = Environment.GetEnvironmentVariable("ETS2_DIR")
                  ?? @"C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2";
        if (!Directory.Exists(dir)) return;

        var (nodes, roads, prefabs) = MapParser.ParseEts2Directory(dir, verbose: true);

        Assert.True(nodes.Count > 100_000, $"Expected >100k nodes, got {nodes.Count}");
        Assert.True(roads.Count > 30_000, $"Expected >30k roads, got {roads.Count}");
        Assert.True(prefabs.Count > 5_000, $"Expected >5k prefabs, got {prefabs.Count}");
        Assert.Contains(nodes, n => !double.IsNaN(n.X) && !double.IsInfinity(n.X));
    }

    [Fact(Skip = "Requires ETS2 base.scs. Set ETS2_BASE_SCS or ETS2_DIR for manual test.")]
    public void RealMapParsing_FromScs()
    {
        var baseScs = Environment.GetEnvironmentVariable("ETS2_BASE_SCS");
        if (string.IsNullOrWhiteSpace(baseScs))
        {
            var dir = Environment.GetEnvironmentVariable("ETS2_DIR")
                      ?? @"C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2";
            baseScs = Path.Combine(dir, "base.scs");
        }
        if (!File.Exists(baseScs)) return;
        if (!IsZipArchive(baseScs)) return;

        var ets2Dir = Path.GetDirectoryName(baseScs)!;
        var (nodes, roads, prefabs) = MapParser.ParseEts2Directory(ets2Dir, verbose: true);

        Assert.True(nodes.Count > 100_000, $"Expected >100k nodes, got {nodes.Count}");
        Assert.True(roads.Count > 30_000, $"Expected >30k roads, got {roads.Count}");
        Assert.True(prefabs.Count > 5_000, $"Expected >5k prefabs, got {prefabs.Count}");
        Assert.Contains(roads, r => r.LaneCountForward > 0);
        Assert.Contains(nodes, n => !double.IsNaN(n.X) && !double.IsInfinity(n.X));
    }

    [Fact(Skip = "Requires HashFS reader and base.scs.")]
    public void RealMapParsing_WithHashFsReader()
    {
        var baseScs = Environment.GetEnvironmentVariable("ETS2_BASE_SCS");
        if (string.IsNullOrWhiteSpace(baseScs) || !File.Exists(baseScs)) return;
        var mapScs = baseScs;
        var candidate = Path.Combine(Path.GetDirectoryName(baseScs)!, "base_map.scs");
        if (File.Exists(candidate)) mapScs = candidate;

        var tempDir = Path.Combine(Path.GetTempPath(), $"ets2_sectors_{Guid.NewGuid()}");
        Directory.CreateDirectory(tempDir);

        try
        {
            var psi = new ProcessStartInfo
            {
                FileName = "python3",
                Arguments = $"-m ets2_hashfs extract \"{mapScs}\" --sectors --out \"{tempDir}\"",
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                UseShellExecute = false
            };
            using var proc = Process.Start(psi);
            if (proc == null) return;
            proc.WaitForExit(120000);
            if (proc.ExitCode != 0) return;

            var (nodes, roads, prefabs) = MapParser.ParseSectorDirectory(tempDir, verbose: true);

            Assert.True(nodes.Count > 100_000, $"Expected >100k nodes, got {nodes.Count}");
            Assert.True(roads.Count > 30_000, $"Expected >30k roads, got {roads.Count}");
            Assert.True(prefabs.Count > 5_000, $"Expected >5k prefabs, got {prefabs.Count}");
        }
        finally
        {
            if (Directory.Exists(tempDir)) Directory.Delete(tempDir, true);
        }
    }

    private static bool IsZipArchive(string path)
    {
        try
        {
            using var _ = ZipFile.OpenRead(path);
            return true;
        }
        catch (InvalidDataException)
        {
            return false;
        }
    }
}
