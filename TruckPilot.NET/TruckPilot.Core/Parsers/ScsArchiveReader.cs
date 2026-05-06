using System.IO.Compression;
using System.Text;

namespace TruckPilot.Core.Parsers;

/// <summary>
/// Reads ETS2 .scs archives. Supports ZIP format (ETS2 1.50+ and mods).
/// HashFS v2 format is NOT supported — use the Rust/Python parser instead.
/// </summary>
public sealed class ScsArchiveReader : IDisposable
{
    private readonly ZipArchive _archive;
    public string Path { get; }

    public ScsArchiveReader(string path)
    {
        Path = path;
        try { _archive = ZipFile.OpenRead(path); }
        catch (InvalidDataException ex) { throw new InvalidDataException($"'{path}' is not a valid ZIP archive (HashFS format is not supported by this reader). Use --text-map-file instead.", ex); }
    }

    public int EntryCount => _archive.Entries.Count;

    public IEnumerable<string> FindFiles(string prefix)
    {
        return _archive.Entries
            .Where(e => e.FullName.StartsWith(prefix, StringComparison.OrdinalIgnoreCase))
            .Select(e => e.FullName);
    }

    public byte[]? ReadFile(string path)
    {
        var entry = _archive.GetEntry(path);
        if (entry == null) return null;
        using var stream = entry.Open();
        using var ms = new MemoryStream();
        stream.CopyTo(ms);
        return ms.ToArray();
    }

    public string? ReadTextFile(string path)
    {
        var data = ReadFile(path);
        return data != null ? Encoding.UTF8.GetString(data) : null;
    }

    public void Dispose() => _archive.Dispose();
}
