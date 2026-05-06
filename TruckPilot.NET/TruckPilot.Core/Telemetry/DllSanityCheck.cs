using System.IO;
using System.IO.MemoryMappedFiles;

namespace TruckPilot.Core.Telemetry;

public static class DllSanityCheck
{
    private const string ShmName = "Local\\TruckPilotTelemetry";
    private const uint ExpectedMagic = 0x54505054;
    private const uint ExpectedVersion = 2;

    public static bool PerformCheck()
    {
        try
        {
            using var mmf = MemoryMappedFile.OpenExisting(ShmName);
            using var accessor = mmf.CreateViewAccessor(0, 8, MemoryMappedFileAccess.Read);

            accessor.Read(0, out uint magic);
            accessor.Read(4, out uint version);

            if (magic != ExpectedMagic)
            {
                Console.Error.WriteLine($"Telemetry MMF magic mismatch. Expected 0x{ExpectedMagic:X8}, got 0x{magic:X8}.");
                return false;
            }

            if (version != ExpectedVersion)
            {
                Console.Error.WriteLine($"Telemetry MMF layout version mismatch. Expected {ExpectedVersion}, got {version}.");
                return false;
            }

            return true;
        }
        catch (FileNotFoundException)
        {
            Console.Error.WriteLine("Telemetry MMF not found. Is truckpilot_telemetry.dll loaded?");
            return false;
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine($"Telemetry MMF check failed: {ex.Message}");
            return false;
        }
    }
}
