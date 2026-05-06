using System.Diagnostics;
using TruckPilot.Core.Models;

namespace TruckPilot.Core.Graph;

/// <summary>A* route planner on the road network graph.</summary>
public static class AStarRouter
{
    private const double MaxSpeedMs = 25.0;
    private const double DefaultSpeedMs = 22.222;

    public static RouteResult? FindRoute(GraphData graph, ulong start, ulong goal, RouteConfig config)
    {
        var sw = Stopwatch.StartNew();

        var positions = graph.Nodes.ToDictionary(n => n.Uid, n => (n.X, n.Z));
        if (!positions.ContainsKey(start) || !positions.ContainsKey(goal)) return null;

        var adj = graph.Edges.GroupBy(e => e.FromNodeUid)
            .ToDictionary(g => g.Key, g => g.OrderBy(e => e.ToNodeUid).ThenBy(e => e.EdgeUid).ToList());

        var openSet = new PriorityQueue<(ulong uid, double f), (double f, ulong uid)>(new NodeComparer());
        var gScore = new Dictionary<ulong, double> { [start] = 0 };
        var cameFrom = new Dictionary<ulong, ulong>();
        var closed = new HashSet<ulong>();
        ulong edgesExamined = 0, nodesExpanded = 0;

        double hStart = Heuristic(positions[start], positions[goal], config.Mode);
        openSet.Enqueue((start, hStart), (hStart, start));

        while (openSet.TryDequeue(out var current, out _))
        {
            if (current.uid == goal)
            {
                var path = ReconstructPath(cameFrom, start, goal);
                return new RouteResult
                {
                    Path = path, TotalCost = gScore[goal],
                    EdgesExamined = edgesExamined, NodesExpanded = nodesExpanded,
                    Validated = ValidatePath(path, adj), PlanningTimeMs = sw.Elapsed.TotalMilliseconds
                };
            }

            if (!closed.Add(current.uid)) continue;
            nodesExpanded++;

            if (!adj.TryGetValue(current.uid, out var neighbors)) continue;
            foreach (var edge in neighbors)
            {
                edgesExamined++;
                if (closed.Contains(edge.ToNodeUid)) continue;

                double cost = EdgeCost(edge, config);
                double tentG = gScore.GetValueOrDefault(current.uid, double.MaxValue) + cost;

                if (tentG < gScore.GetValueOrDefault(edge.ToNodeUid, double.MaxValue))
                {
                    cameFrom[edge.ToNodeUid] = current.uid;
                    gScore[edge.ToNodeUid] = tentG;
                    double h = Heuristic(positions[edge.ToNodeUid], positions[goal], config.Mode);
                    double f = tentG + h;
                    openSet.Enqueue((edge.ToNodeUid, f), (f, edge.ToNodeUid));
                }
            }
        }

        return null;
    }

    private static double Heuristic((double X, double Z) a, (double X, double Z) b, CostMode mode)
    {
        double d = Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Z - b.Z) * (a.Z - b.Z));
        return mode == CostMode.Eta ? d / MaxSpeedMs : d;
    }

    private static double EdgeCost(GraphEdge edge, RouteConfig config)
    {
        double speed = config.PreferSpeed ? (edge.SpeedLimitKmh ?? 80.0) / 3.6 : DefaultSpeedMs;
        double cost = config.Mode == CostMode.Eta ? edge.DistanceM / Math.Max(speed, 1.0) : edge.DistanceM;

        if (edge.Flags.Contains("no_lanes_unknown")) cost *= 1.5;
        if (edge.Direction == "lane_change") cost *= 3.0;
        return cost;
    }

    private static List<ulong> ReconstructPath(Dictionary<ulong, ulong> cameFrom, ulong start, ulong goal)
    {
        var path = new List<ulong> { goal };
        var current = goal;
        while (current != start && cameFrom.TryGetValue(current, out var prev))
        {
            path.Add(prev);
            current = prev;
        }
        path.Reverse();
        return path;
    }

    private static bool ValidatePath(List<ulong> path, Dictionary<ulong, List<GraphEdge>> adj)
    {
        for (int i = 0; i < path.Count - 1; i++)
        {
            if (!adj.TryGetValue(path[i], out var edges)) return false;
            if (!edges.Any(e => e.ToNodeUid == path[i + 1])) return false;
        }
        return true;
    }

    private sealed class NodeComparer : IComparer<(double f, ulong uid)>
    {
        public int Compare((double f, ulong uid) x, (double f, ulong uid) y)
        {
            int c = x.f.CompareTo(y.f);
            return c != 0 ? c : x.uid.CompareTo(y.uid);
        }
    }
}
