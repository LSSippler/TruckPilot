using System.Diagnostics;
using System.Text.Json;
using TruckPilot.Core.Graph;
using TruckPilot.Core.Models;

namespace TruckPilot.Core.Export;

public static class Pipeline
{
    public static void Run(List<MapNode> nodes, List<MapRoad> roads, List<MapPrefab> prefabs,
        ulong? startUid, ulong? goalUid, bool verbose)
    {
        var sw = Stopwatch.StartNew();
        var graph = GraphBuilder.Build(nodes, roads, prefabs);
        sw.Stop();

        if (verbose)
        {
            Console.WriteLine($"Nodes: {graph.Nodes.Count}, Edges: {graph.Edges.Count}");
            Console.WriteLine($"Build time: {sw.Elapsed.TotalMilliseconds:F2} ms");
        }

        // Write graph
        var json = JsonSerializer.Serialize(graph, new JsonSerializerOptions { WriteIndented = false, PropertyNamingPolicy = System.Text.Json.JsonNamingPolicy.SnakeCaseLower });
        File.WriteAllText("graph.json", json);
        if (verbose) Console.WriteLine("Wrote graph.json");

        // Route
        if (startUid.HasValue && goalUid.HasValue)
        {
            var config = new RouteConfig();
            var route = AStarRouter.FindRoute(graph, startUid.Value, goalUid.Value, config);
            if (route != null)
            {
                Console.WriteLine($"Route found: {route.Path.Count} nodes, cost={route.TotalCost:F2}, validated={route.Validated}, time={route.PlanningTimeMs:F2}ms");
                if (verbose) Console.WriteLine($"  Path: [{string.Join(", ", route.Path)}]");
            }
            else Console.WriteLine("No route found.");
        }
    }
}
