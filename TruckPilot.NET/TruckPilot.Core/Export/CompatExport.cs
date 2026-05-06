using System.Text.Json;
using System.Text.Json.Serialization;
using TruckPilot.Core.Models;

namespace TruckPilot.Core.Export;

public static class CompatExport
{
    public static void Write(string prefix, List<MapNode> nodes, List<MapRoad> roads, GraphData graph)
    {
        var cNodes = nodes.Select(n => new CompatNode { NodeUid = n.Uid.ToString("X16"), X = n.X, Y = n.Y, Z = n.Z })
            .OrderBy(n => n.NodeUid).ToList();
        var cRoads = roads.Select(r => new CompatRoad
        {
            RoadUid = r.Uid.ToString("X16"), Name = r.Name, LookToken = r.LookToken,
            FromNodeUid = r.NodeUids.FirstOrDefault().ToString("X16"),
            ToNodeUid = r.NodeUids.LastOrDefault().ToString("X16"),
            Length = ComputeLength(r, nodes), SpeedLimit = r.SpeedLimit,
            LaneCount = Math.Max(r.LaneCountForward, Math.Max(r.LaneCountBackward, 1))
        }).OrderBy(r => r.RoadUid).ToList();
        var cLooks = roads.Select(r => r.LookToken).Where(t => !string.IsNullOrEmpty(t)).Distinct().OrderBy(t => t)
            .Select(t => new CompatRoadLook { Token = t!, Name = t! }).ToList();
        var cGraph = new CompatGraph
        {
            Nodes = cNodes,
            Edges = graph.Edges.Select(e => new CompatGraphEdge
            {
                EdgeUid = e.EdgeUid.ToString("X12"), FromNodeUid = e.FromNodeUid.ToString("X16"), ToNodeUid = e.ToNodeUid.ToString("X16"),
                RoadUid = e.RoadUid, DistanceM = e.DistanceM, Direction = e.Direction,
                LaneCount = e.LaneCount, SpeedLimitKmh = e.SpeedLimitKmh, Flags = e.Flags
            }).ToList()
        };

        WriteJson($"{prefix}_nodes.json", cNodes);
        WriteJson($"{prefix}_roads.json", cRoads);
        WriteJson($"{prefix}_road_looks.json", cLooks);
        WriteJson($"{prefix}_graph.json", cGraph);
    }

    private static double ComputeLength(MapRoad r, List<MapNode> nodes)
    {
        var lookup = nodes.ToDictionary(n => n.Uid);
        double total = 0;
        for (int i = 0; i < r.NodeUids.Count - 1; i++)
        {
            if (lookup.TryGetValue(r.NodeUids[i], out var a) && lookup.TryGetValue(r.NodeUids[i + 1], out var b))
            {
                total += Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y) + (a.Z - b.Z) * (a.Z - b.Z));
            }
        }
        return total;
    }

    private static void WriteJson<T>(string path, T obj)
    {
        var opts = new JsonSerializerOptions { WriteIndented = false, DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull };
        File.WriteAllText(path, JsonSerializer.Serialize(obj, opts));
    }
}

// Compat types with camelCase serialization
public sealed class CompatNode
{
    [JsonPropertyName("nodeUid")] public string NodeUid { get; init; } = "";
    public double X { get; init; }
    public double Y { get; init; }
    public double Z { get; init; }
}

public sealed class CompatRoad
{
    [JsonPropertyName("roadUid")] public string RoadUid { get; init; } = "";
    public string Name { get; init; } = "";
    [JsonPropertyName("lookToken")] public string LookToken { get; init; } = "";
    [JsonPropertyName("fromNodeUid")] public string FromNodeUid { get; init; } = "";
    [JsonPropertyName("toNodeUid")] public string ToNodeUid { get; init; } = "";
    public double Length { get; init; }
    public float? SpeedLimit { get; init; }
    [JsonPropertyName("laneCount")] public uint LaneCount { get; init; }
}

public sealed class CompatRoadLook
{
    public string Token { get; init; } = "";
    public string Name { get; init; } = "";
}

public sealed class CompatGraph
{
    public List<CompatNode> Nodes { get; init; } = new();
    public List<CompatGraphEdge> Edges { get; init; } = new();
}

public sealed class CompatGraphEdge
{
    [JsonPropertyName("edgeUid")] public string EdgeUid { get; init; } = "";
    [JsonPropertyName("fromNodeUid")] public string FromNodeUid { get; init; } = "";
    [JsonPropertyName("toNodeUid")] public string ToNodeUid { get; init; } = "";
    [JsonPropertyName("roadUid")] public string? RoadUid { get; init; }
    [JsonPropertyName("distanceM")] public double DistanceM { get; init; }
    public string Direction { get; init; } = "";
    [JsonPropertyName("laneCount")] public uint LaneCount { get; init; }
    [JsonPropertyName("speedLimitKmh")] public double? SpeedLimitKmh { get; init; }
    public List<string> Flags { get; init; } = new();
}
