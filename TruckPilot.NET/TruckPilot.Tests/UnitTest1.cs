using System.IO.MemoryMappedFiles;
using TruckPilot.Core.Telemetry;

namespace TruckPilot.Tests;

public class TestDllSanityCheck
{
    [Fact]
    public void PerformCheck_ReturnsTrue_WhenMagicAndVersionMatch()
    {
        if (!OperatingSystem.IsWindows())
        {
            return;
        }

        using var mmf = MemoryMappedFile.CreateOrOpen("Local\\TruckPilotTelemetry", 8);
        using var accessor = mmf.CreateViewAccessor(0, 8, MemoryMappedFileAccess.ReadWrite);

        accessor.Write(0, 0x54505054u);
        accessor.Write(4, 1u);

        var result = DllSanityCheck.PerformCheck();

        Assert.True(result);
    }
}
