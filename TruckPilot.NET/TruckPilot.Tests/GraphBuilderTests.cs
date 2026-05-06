using TruckPilot.Core.Graph;
using TruckPilot.Core.Models;

namespace TruckPilot.Tests;

public class GraphBuilderTests
{
    private static List<MapNode> SampleNodes() => new()
    {
        new() { Uid = 1, X = 0, Y = 0, Z = 0 },
        new() { Uid = 2, X = 100, Y = 0, Z = 0 },
        new() { Uid = 3, X = 200, Y = 0, Z = 0 },
        new() { Uid = 4, X = 300, Y = 0, Z = 0 },
    };

    private static List<MapRoad> SampleRoads() => new()
    {
        new() { Uid = 0xA, NodeUids = new() { 1, 2, 3, 4 }, LaneCountForward = 2, LaneCountBackward = 2, SpeedLimit = 80 }
    };

    [Fact]
    public void Build_ShouldCreateNodes()
    {
        var graph = GraphBuilder.Build(SampleNodes(), SampleRoads(), new List<MapPrefab>());
        Assert.Equal(4, graph.Nodes.Count);
    }

    [Fact]
    public void Build_ShouldCreateLaneEdges()
    {
        var graph = GraphBuilder.Build(SampleNodes(), SampleRoads(), new List<MapPrefab>());
        Assert.True(graph.Edges.Count >= 12);
    }

    [Fact]
    public void ComputeEdgeUid_ShouldBeDeterministic()
    {
        var uid1 = GraphBuilder.ComputeEdgeUid(1, 2, "road_a", "forward");
        var uid2 = GraphBuilder.ComputeEdgeUid(1, 2, "road_a", "forward");
        Assert.Equal(uid1, uid2);
        Assert.NotEqual(uid1, GraphBuilder.ComputeEdgeUid(1, 2, "road_a", "backward"));
    }
}
