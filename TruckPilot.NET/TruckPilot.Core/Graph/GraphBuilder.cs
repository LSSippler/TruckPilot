using System.Security.Cryptography;
using System.Text;
using TruckPilot.Core.Models;

namespace TruckPilot.Core.Graph;

/// <summary>Builds a directed road network graph from raw map data.</summary>
public static class GraphBuilder
{
    private const string HashSalt = "TruckPilot";

    public static GraphData Build(IReadOnlyList<MapNode> nodes, IReadOnlyList<MapRoad> roads, IReadOnlyList<MapPrefab> prefabs)
    {
        var nodeSet = nodes.ToDictionary(n => n.Uid);
        var result = new GraphData { Meta = new() };
        var edges = new List<GraphEdge>();

        // Road edges with lane support
        foreach (var road in roads)
        {
            for (int i = 0; i < road.NodeUids.Count - 1; i++)
            {
                var fromUid = road.NodeUids[i];
                var toUid = road.NodeUids[i + 1];
                if (!nodeSet.ContainsKey(fromUid) || !nodeSet.ContainsKey(toUid)) continue;

                var dist = road.Length > 0 ? road.Length : Distance(nodeSet[fromUid], nodeSet[toUid]);

                if (road.LaneCountForward > 0)
                    for (uint lane = 0; lane < road.LaneCountForward; lane++)
                        edges.Add(MakeEdge(fromUid, toUid, road.Uid, dist, "forward", road.LaneCountForward, road.SpeedLimit, lane));

                if (road.LaneCountBackward > 0)
                    for (uint lane = 0; lane < road.LaneCountBackward; lane++)
                        edges.Add(MakeEdge(toUid, fromUid, road.Uid, dist, "backward", road.LaneCountBackward, road.SpeedLimit, lane));

                if (road.LaneCountForward == 0 && road.LaneCountBackward == 0)
                {
                    edges.Add(MakeEdge(fromUid, toUid, road.Uid, dist, "bidirectional_unknown", 1, road.SpeedLimit, 0, "no_lanes_unknown"));
                    edges.Add(MakeEdge(toUid, fromUid, road.Uid, dist, "bidirectional_unknown", 1, road.SpeedLimit, 0, "no_lanes_unknown"));
                }
            }
        }

        // Prefab interconnect — check directed pairs
        var existingPairs = new HashSet<(ulong, ulong)>();
        foreach (var e in edges) existingPairs.Add((e.FromNodeUid, e.ToNodeUid));

        foreach (var prefab in prefabs)
        {
            if (prefab.NodeUids.Count <= 1) continue;
            for (int i = 0; i < prefab.NodeUids.Count; i++)
            {
                for (int j = i + 1; j < prefab.NodeUids.Count; j++)
                {
                    var u = prefab.NodeUids[i];
                    var v = prefab.NodeUids[j];
                    if (!nodeSet.ContainsKey(u) || !nodeSet.ContainsKey(v)) continue;
                    var dist = Distance(nodeSet[u], nodeSet[v]);

                    if (existingPairs.Add((u, v)))
                        edges.Add(MakeEdge(u, v, null, dist, "prefab_interconnect", 1, null, 0));
                    if (existingPairs.Add((v, u)))
                        edges.Add(MakeEdge(v, u, null, dist, "prefab_interconnect", 1, null, 0));
                }
            }
        }

        // Lane-change edges

        // Sort deterministically
        result.Nodes = nodes.Select(n => new GraphNode { Uid = n.Uid, X = n.X, Y = n.Y, Z = n.Z })
            .OrderBy(n => n.Uid).ToList();
        result.Edges = edges
            .OrderBy(e => e.FromNodeUid).ThenBy(e => e.ToNodeUid).ThenBy(e => e.RoadUid)
            .ThenBy(e => e.Direction).ThenBy(e => e.EdgeUid).ToList();

        return result;
    }


    private static GraphEdge MakeEdge(ulong from, ulong to, ulong? roadUid, double dist, string dir, uint lanes, float? speed, uint laneIdx, params string[] extraFlags)
    {
        var flags = extraFlags.ToList();
        if (lanes > 0 && dir is "forward" or "backward") flags.Add($"lane={laneIdx}");
        return new GraphEdge
        {
            EdgeUid = ComputeEdgeUid(from, to, roadUid?.ToString(), dir, laneIdx),
            FromNodeUid = from, ToNodeUid = to,
            RoadUid = roadUid?.ToString(),
            DistanceM = dist, Direction = dir,
            LaneCount = lanes, SpeedLimitKmh = speed,
            Flags = flags
        };
    }

    public static ulong ComputeEdgeUid(ulong from, ulong to, string? roadUid, string dir, uint laneIdx = 0)
    {
        var input = $"{from}|{to}|{roadUid ?? ""}|{dir}|{laneIdx}|{HashSalt}";
        var hash = SHA256.HashData(Encoding.UTF8.GetBytes(input));
        return ulong.Parse(Convert.ToHexString(hash)[..12], System.Globalization.NumberStyles.HexNumber);
    }

    internal static double Distance(MapNode a, MapNode b)
    {
        var dx = a.X - b.X; var dy = a.Y - b.Y; var dz = a.Z - b.Z;
        return Math.Sqrt(dx * dx + dy * dy + dz * dz);
    }
}
