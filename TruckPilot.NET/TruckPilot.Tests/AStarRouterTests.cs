using TruckPilot.Core.Graph;
using TruckPilot.Core.Models;

namespace TruckPilot.Tests;

public class AStarRouterTests
{
    private static (List<MapNode>, List<MapRoad>) ChainMap()
    {
        var nodes = new List<MapNode>
        {
            new() { Uid = 1, X = 0, Y = 0, Z = 0 },
            new() { Uid = 2, X = 100, Y = 0, Z = 0 },
            new() { Uid = 3, X = 200, Y = 0, Z = 0 },
            new() { Uid = 4, X = 300, Y = 0, Z = 0 },
        };
        var roads = new List<MapRoad>
        {
            new() { Uid = 10, NodeUids = new() { 1, 2, 3, 4 }, LaneCountForward = 1, LaneCountBackward = 1 }
        };
        return (nodes, roads);
    }

    [Fact]
    public void SimpleRoute_ShouldFindPath()
    {
        var (nodes, roads) = ChainMap();
        var graph = GraphBuilder.Build(nodes, roads, new List<MapPrefab>());
        var route = AStarRouter.FindRoute(graph, 1, 4, new RouteConfig());
        Assert.NotNull(route);
        Assert.Equal(new List<ulong> { 1, 2, 3, 4 }, route!.Path);
        Assert.True(route.Validated);
    }

    [Fact]
    public void UnreachableGoal_ShouldReturnNull()
    {
        var nodes = new List<MapNode> { new() { Uid = 1 }, new() { Uid = 2 } };
        var graph = GraphBuilder.Build(nodes, new List<MapRoad>(), new List<MapPrefab>());
        Assert.Null(AStarRouter.FindRoute(graph, 1, 2, new RouteConfig()));
    }

    [Fact]
    public void EtaMode_ShouldBeCheaperThanDistance()
    {
        var (nodes, roads) = ChainMap();
        var graph = GraphBuilder.Build(nodes, roads, new List<MapPrefab>());
        var dist = AStarRouter.FindRoute(graph, 1, 4, new RouteConfig { Mode = CostMode.Distance })!;
        var eta = AStarRouter.FindRoute(graph, 1, 4, new RouteConfig { Mode = CostMode.Eta })!;
        Assert.True(eta.TotalCost < dist.TotalCost);
    }

    [Fact]
    public void Route_ShouldBeDeterministic()
    {
        var (nodes, roads) = ChainMap();
        var graph = GraphBuilder.Build(nodes, roads, new List<MapPrefab>());
        var r1 = AStarRouter.FindRoute(graph, 1, 4, new RouteConfig())!;
        var r2 = AStarRouter.FindRoute(graph, 1, 4, new RouteConfig())!;
        Assert.Equal(r1.Path, r2.Path);
        Assert.Equal(r1.TotalCost, r2.TotalCost, 1e-9);
    }
}
