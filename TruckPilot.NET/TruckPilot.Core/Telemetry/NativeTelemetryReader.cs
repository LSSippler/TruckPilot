using System.IO.MemoryMappedFiles;
using System.Runtime.InteropServices;

namespace TruckPilot.Core.Telemetry;

/// <summary>
/// Shared memory layout prefix matching the native telemetry DLL.
/// Keep field order synchronized up to <see cref="DistanceToLeadM"/>.
/// </summary>
[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct TelemetryLayout
{
    public uint Magic;           // 0x54505054 ("TPPT")
    public uint LayoutVersion;   // 2
    public uint Sequence;
    public uint Padding;

    // Position
    public double X, Y, Z;
    public double Heading, Pitch, Roll;
    public double SpeedMs;
    public double EngineRpm;
    public double NavSpeedLimitKmh;
    public uint NavSpeedLimitValid;
    public double FuelLiters;
    public double OdometerKm;
    public double CruiseControlSpeedKmh;

    // Extended channels (offset 116+)
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
    public float[] LocalVelocity;
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
    public float[] LocalAcceleration;
    public float EffectiveThrottle;
    public float DistanceToLeadM;
}

/// <summary>Reads telemetry from the native DLL's shared memory region.</summary>
public sealed class NativeTelemetryReader : IDisposable
{
    private const string ShmName = "Local\\TruckPilotTelemetry";
    private const uint ExpectedMagic = 0x54505054;
    private const uint ExpectedVersion = 2;

    private MemoryMappedFile? _mmf;
    private MemoryMappedViewAccessor? _accessor;

    public bool TryOpen()
    {
        try
        {
            _mmf = MemoryMappedFile.OpenExisting(ShmName);
            _accessor = _mmf.CreateViewAccessor(0, Marshal.SizeOf<TelemetryLayout>());
            return true;
        }
        catch (FileNotFoundException) { return false; }
        catch (Exception) { return false; }
    }

    public TelemetryLayout? Read()
    {
        if (_accessor == null) return null;
        try
        {
            _accessor.Read(0, out TelemetryLayout layout);
            if (layout.Magic != ExpectedMagic || layout.LayoutVersion != ExpectedVersion) return null;
            return layout;
        }
        catch { return null; }
    }

    public void Dispose()
    {
        _accessor?.Dispose();
        _mmf?.Dispose();
    }
}
