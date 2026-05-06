using TruckPilot.Core.Models;

namespace TruckPilot.Core.Parsers;

public static class MapParser
{
    public static MapData ParseEts2Directory(string dir, bool verbose = false)
    {
        var allNodes = new List<MapNode>();
        var allRoads = new List<MapRoad>();
        var allPrefabs = new List<MapPrefab>();
        var allCompanies = new List<MapCompany>();
        var allCities = new List<MapCity>();
        var allFerries = new List<MapFerry>();
        var allFuelPumps = new List<MapFuelPump>();
        var allSigns = new List<MapSign>();
        int sectorsParsed = 0;

        foreach (var scsFile in new[] { "base.scs", "base_map.scs", "def.scs" })
        {
            var path = System.IO.Path.Combine(dir, scsFile);
            if (!File.Exists(path)) continue;

            try
            {
                using var archive = new ScsArchiveReader(path);
                if (verbose) Console.Error.WriteLine($"  {scsFile}: {archive.EntryCount} entries");

                int mapFilesFound = 0;
                foreach (var entry in archive.FindFiles("map/europe/"))
                {
                    if (!entry.EndsWith(".data", StringComparison.OrdinalIgnoreCase)) continue;
                    mapFilesFound++;
                    var data = archive.ReadFile(entry);
                    if (data == null || data.Length < 100) continue;

                    var sector = BinaryMapParser.ParseSector(data);
                    allNodes.AddRange(sector.Nodes);
                    allRoads.AddRange(sector.Roads);
                    allPrefabs.AddRange(sector.Prefabs);
                    allCompanies.AddRange(sector.Companies);
                    allCities.AddRange(sector.Cities);
                    allFerries.AddRange(sector.Ferries);
                    allFuelPumps.AddRange(sector.FuelPumps);
                    allSigns.AddRange(sector.Signs);
                    sectorsParsed++;
                }
                if (verbose) Console.Error.WriteLine($"    map files: {mapFilesFound}, sectors parsed: {sectorsParsed}");
            }
            catch (InvalidDataException)
            {
                if (verbose) Console.Error.WriteLine($"  {scsFile}: not a ZIP archive (HashFS format — use Rust parser)");
            }
        }

        var result = new MapData
        {
            Nodes = Dedup(allNodes, n => n.Uid),
            Roads = Dedup(allRoads, r => r.Uid),
            Prefabs = Dedup(allPrefabs, p => p.Uid),
            Companies = Dedup(allCompanies, c => c.Uid),
            Cities = Dedup(allCities, c => c.Uid),
            Ferries = Dedup(allFerries, f => f.Uid),
            FuelPumps = Dedup(allFuelPumps, f => f.Uid),
            Signs = Dedup(allSigns, s => s.Uid)
        };

        if (verbose)
            Console.Error.WriteLine($"Total: {result.Nodes.Count} nodes, {result.Roads.Count} roads, {result.Prefabs.Count} prefabs from {sectorsParsed} sectors");

        return result;
    }

    public static MapData ParseSectorDirectory(string dir, bool verbose = false)
    {
        var allNodes = new List<MapNode>();
        var allRoads = new List<MapRoad>();
        var allPrefabs = new List<MapPrefab>();
        var allCompanies = new List<MapCompany>();
        var allCities = new List<MapCity>();
        var allFerries = new List<MapFerry>();
        var allFuelPumps = new List<MapFuelPump>();
        var allSigns = new List<MapSign>();
        int sectorsParsed = 0;

        if (!Directory.Exists(dir))
            return new MapData();

        foreach (var path in Directory.EnumerateFiles(dir, "*.*", SearchOption.AllDirectories))
        {
            var ext = Path.GetExtension(path).ToLowerInvariant();
            if (ext is not ".base") continue;
            var data = File.ReadAllBytes(path);
            data = TryDecompressZlib(data);
            if (data.Length < 100) continue;

            MapData sector;
            try
            {
                sector = BinaryMapParser.ParseSector(data);
            }
            catch (InvalidDataException ex)
            {
                throw new InvalidDataException($"Failed parsing {path}: {ex.Message}", ex);
            }
            allNodes.AddRange(sector.Nodes);
            allRoads.AddRange(sector.Roads);
            allPrefabs.AddRange(sector.Prefabs);
            allCompanies.AddRange(sector.Companies);
            allCities.AddRange(sector.Cities);
            allFerries.AddRange(sector.Ferries);
            allFuelPumps.AddRange(sector.FuelPumps);
            allSigns.AddRange(sector.Signs);
            sectorsParsed++;
        }

        var result = new MapData
        {
            Nodes = Dedup(allNodes, n => n.Uid),
            Roads = Dedup(allRoads, r => r.Uid),
            Prefabs = Dedup(allPrefabs, p => p.Uid),
            Companies = Dedup(allCompanies, c => c.Uid),
            Cities = Dedup(allCities, c => c.Uid),
            Ferries = Dedup(allFerries, f => f.Uid),
            FuelPumps = Dedup(allFuelPumps, f => f.Uid),
            Signs = Dedup(allSigns, s => s.Uid)
        };

        if (verbose)
            Console.Error.WriteLine($"Total: {result.Nodes.Count} nodes, {result.Roads.Count} roads, {result.Prefabs.Count} prefabs from {sectorsParsed} sectors");

        return result;
    }

    private static byte[] TryDecompressZlib(byte[] data)
    {
        if (data.Length < 2) return data;
        if (data[0] != 0x78) return data;

        try
        {
            using var input = new MemoryStream(data);
            using var z = new System.IO.Compression.ZLibStream(input, System.IO.Compression.CompressionMode.Decompress);
            using var output = new MemoryStream();
            z.CopyTo(output);
            return output.ToArray();
        }
        catch
        {
            return data;
        }
    }

    private static List<T> Dedup<T>(List<T> items, Func<T, ulong> keySelector)
    {
        return items.GroupBy(keySelector).Select(g => g.First()).ToList();
    }

    public static MapData ParseTextMapFile(string path)
        => ParseTextMap(File.ReadAllText(path));

    public static MapData ParseTextMap(string content)
    {
        var nodes = new List<MapNode>(); var roads = new List<MapRoad>(); var prefabs = new List<MapPrefab>();
        int i = 0;
        while (i < content.Length)
        {
            i = SkipWS(content, i);
            if (i >= content.Length) break;
            if (Match(content, i, "node")) { var n = ParseTextNode(content, ref i); if (n != null) nodes.Add(n); }
            else if (Match(content, i, "road")) { var r = ParseTextRoad(content, ref i); if (r != null) roads.Add(r); }
            else if (Match(content, i, "prefab")) { var f = ParseTextPrefab(content, ref i); if (f != null) prefabs.Add(f); }
            else i++;
        }
        return new MapData { Nodes = nodes, Roads = roads, Prefabs = prefabs };
    }

    private static MapNode? ParseTextNode(string s, ref int i) { i+=4;i=SkipWS(s,i);if(i>=s.Length||s[i]!='{')return null;i++;ulong uid=0;double x=0,y=0,z=0;while(i<s.Length){i=SkipWS(s,i);if(i>=s.Length||s[i]=='}'){if(i<s.Length)i++;break;}var k=ReadKey(s,ref i);i=SkipWS(s,i);if(i<s.Length&&s[i]==':')i++;i=SkipWS(s,i);switch(k){case"uid":uid=ReadHex(s,ref i);break;case"position":if(i<s.Length&&s[i]=='(')i++;x=ReadFloat(s,ref i);SkipComma(s,ref i);y=ReadFloat(s,ref i);SkipComma(s,ref i);z=ReadFloat(s,ref i);if(i<s.Length&&s[i]==')')i++;break;default:SkipVal(s,ref i);break;}}return uid!=0?new MapNode{Uid=uid,X=x,Y=y,Z=z}:null;}
    private static MapRoad? ParseTextRoad(string s, ref int i) { i+=4;i=SkipWS(s,i);if(i>=s.Length||s[i]!='{')return null;i++;ulong uid=0;var nids=new List<ulong>();uint fwd=0,bwd=0;float?spd=null;while(i<s.Length){i=SkipWS(s,i);if(i>=s.Length||s[i]=='}'){if(i<s.Length)i++;break;}var k=ReadKey(s,ref i);i=SkipWS(s,i);if(i<s.Length&&s[i]==':')i++;i=SkipWS(s,i);switch(k){case"uid":uid=ReadHex(s,ref i);break;case"nodes":nids=ReadUids(s,ref i);break;case"lane_count_forward":fwd=(uint)ReadInt(s,ref i);break;case"lane_count_backward":bwd=(uint)ReadInt(s,ref i);break;case"speed_limit":spd=(float)ReadFloat(s,ref i);break;default:SkipVal(s,ref i);break;}}return uid!=0&&nids.Count>=2?new MapRoad{Uid=uid,NodeUids=nids,LaneCountForward=fwd,LaneCountBackward=bwd,SpeedLimit=spd}:null;}
    private static MapPrefab? ParseTextPrefab(string s, ref int i) { i+=6;i=SkipWS(s,i);if(i>=s.Length||s[i]!='{')return null;i++;ulong uid=0;var nids=new List<ulong>();while(i<s.Length){i=SkipWS(s,i);if(i>=s.Length||s[i]=='}'){if(i<s.Length)i++;break;}var k=ReadKey(s,ref i);i=SkipWS(s,i);if(i<s.Length&&s[i]==':')i++;i=SkipWS(s,i);if(k=="uid")uid=ReadHex(s,ref i);else if(k=="nodes")nids=ReadUids(s,ref i);else SkipVal(s,ref i);}return uid!=0?new MapPrefab{Uid=uid,NodeUids=nids}:null;}

    private static string ReadKey(string s, ref int i) { int st=i;while(i<s.Length&&(char.IsLetterOrDigit(s[i])||s[i]=='_'))i++;return s[st..i]; }
    private static ulong ReadHex(string s, ref int i) { if(i+1<s.Length&&s[i]=='0'&&(s[i+1]=='x'||s[i+1]=='X')){i+=2;int st=i;while(i<s.Length&&char.IsAsciiHexDigit(s[i]))i++;return ulong.Parse(s[st..i],System.Globalization.NumberStyles.HexNumber);}return(ulong)ReadFloat(s,ref i);}
    private static double ReadFloat(string s, ref int i) { int st=i;if(i<s.Length&&s[i]=='-')i++;while(i<s.Length&&(char.IsDigit(s[i])||s[i]=='.'))i++;return double.Parse(s[st..i],System.Globalization.CultureInfo.InvariantCulture);}
    private static long ReadInt(string s, ref int i) { int st=i;while(i<s.Length&&char.IsDigit(s[i]))i++;return long.Parse(s[st..i],System.Globalization.CultureInfo.InvariantCulture);}
    private static List<ulong> ReadUids(string s, ref int i) { var l=new List<ulong>();i=SkipWS(s,i);if(i<s.Length&&s[i]=='(')i++;while(i<s.Length&&s[i]!=')'){i=SkipWS(s,i);if(i>=s.Length||s[i]==')')break;l.Add(ReadHex(s,ref i));i=SkipWS(s,i);if(i<s.Length&&s[i]==',')i++;}if(i<s.Length&&s[i]==')')i++;return l;}
    private static void SkipVal(string s, ref int i) { if(i<s.Length&&s[i]=='"'){int end=s.IndexOf('"',i+1);i=end>=0?end+1:s.Length;return;}if(i<s.Length&&s[i]=='('){int end=s.IndexOf(')',i);i=end>=0?end+1:s.Length;return;}while(i<s.Length&&!char.IsWhiteSpace(s[i])&&s[i]!=','&&s[i]!='}')i++; }
    private static int SkipWS(string s, int i) { while(i<s.Length&&char.IsWhiteSpace(s[i]))i++;return i; }
    private static void SkipComma(string s, ref int i) { i=SkipWS(s,i);if(i<s.Length&&s[i]==',')i++; }
    private static bool Match(string s, int i, string p) => i+p.Length<=s.Length&&s[i..(i+p.Length)]==p;
}
