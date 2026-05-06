using System.Text.Json;
using TruckPilot.Core.Models;

namespace TruckPilot.Core.Export;

public sealed class GraphMetrics
{
    public int NodesTotal { get; init; }
    public int EdgesTotal { get; init; }
    public double Density { get; init; }
    public double LargestComponentRatio { get; init; }
    public double PctDirected { get; init; }
    public double PctUnknown { get; init; }
    public double PctWithSpeedLimit { get; init; }
    public double BuildTimeMs { get; init; }
}

public sealed class QualityReport
{
    public GraphMeta Meta { get; init; } = new();
    public GraphMetrics Metrics { get; init; } = new();
}

public static class MetricsCalculator
{
    public static GraphMetrics Compute(GraphData graph, double buildTimeMs)
    {
        int n = graph.Nodes.Count;
        int e = graph.Edges.Count;
        double density = n > 1 ? (double)e / n : 0;
        int directed = graph.Edges.Count(ed => ed.Direction is "forward" or "backward");
        int unknown = graph.Edges.Count(ed => ed.Direction == "bidirectional_unknown");
        int withSpeed = graph.Edges.Count(ed => ed.SpeedLimitKmh.HasValue);
        double lcr = LargestComponentRatio(graph);

        return new GraphMetrics
        {
            NodesTotal = n, EdgesTotal = e, Density = density,
            LargestComponentRatio = lcr,
            PctDirected = e > 0 ? 100.0 * directed / e : 0,
            PctUnknown = e > 0 ? 100.0 * unknown / e : 0,
            PctWithSpeedLimit = e > 0 ? 100.0 * withSpeed / e : 0,
            BuildTimeMs = buildTimeMs
        };
    }

    private static double LargestComponentRatio(GraphData graph)
    {
        if (graph.Nodes.Count == 0) return 0;
        var adj = new Dictionary<ulong, List<ulong>>();
        foreach (var e in graph.Edges)
        {
            if (!adj.ContainsKey(e.FromNodeUid)) adj[e.FromNodeUid] = new();
            if (!adj.ContainsKey(e.ToNodeUid)) adj[e.ToNodeUid] = new();
            adj[e.FromNodeUid].Add(e.ToNodeUid);
            adj[e.ToNodeUid].Add(e.FromNodeUid);
        }
        var visited = new HashSet<ulong>();
        int maxSize = 0;
        foreach (var node in graph.Nodes)
        {
            if (visited.Contains(node.Uid)) continue;
            var stack = new Stack<ulong>(); stack.Push(node.Uid);
            visited.Add(node.Uid);
            int size = 1;
            while (stack.Count > 0)
            {
                var cur = stack.Pop();
                if (adj.TryGetValue(cur, out var neighbors))
                    foreach (var nb in neighbors)
                        if (visited.Add(nb)) { stack.Push(nb); size++; }
            }
            if (size > maxSize) maxSize = size;
        }
        return (double)maxSize / graph.Nodes.Count;
    }

    public static void WriteQualityReport(string path, GraphData graph, double buildTimeMs, string mapName = "unknown")
    {
        var metrics = Compute(graph, buildTimeMs);
        var report = new QualityReport { Meta = new() { MapName = mapName, GeneratedAt = "" }, Metrics = metrics };
        File.WriteAllText(path, JsonSerializer.Serialize(report, new JsonSerializerOptions { WriteIndented = false }));
    }
}
