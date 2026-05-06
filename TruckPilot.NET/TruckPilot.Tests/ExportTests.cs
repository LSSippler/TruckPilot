using System.Text.Json;
using TruckPilot.Core.Export;
using TruckPilot.Core.Models;

namespace TruckPilot.Tests;

public class ExportTests
{
    [Fact]
    public void CompatExport_NodesJson_HasCamelCaseKeys()
    {
        var prefix = Path.Combine(Path.GetTempPath(), $"tp_test_{Guid.NewGuid()}");
        var nodes = new List<MapNode>
        {
            new() { Uid = 1, X = 10.5, Y = 20.0, Z = 30.0 }
        };
        var roads = new List<MapRoad>
        {
            new()
            {
                Uid = 100,
                Name = "Highway",
                LookToken = "hw",
                NodeUids = new() { 1, 2 },
                SpeedLimit = 80,
                LaneCountForward = 2,
                LaneCountBackward = 1
            }
        };
        var graph = new GraphData
        {
            Nodes = new List<GraphNode> { new() { Uid = 1 } },
            Edges = new List<GraphEdge>
            {
                new()
                {
                    EdgeUid = 0xabc123UL,
                    FromNodeUid = 1,
                    ToNodeUid = 2,
                    RoadUid = "100",
                    DistanceM = 100.0,
                    Direction = "forward",
                    LaneCount = 2,
                    SpeedLimitKmh = 80,
                    Flags = new() { "lane=0" }
                }
            }
        };

        try
        {
            CompatExport.Write(prefix, nodes, roads, graph);

            var nodesJson = File.ReadAllText($"{prefix}_nodes.json");
            Assert.Contains("\"nodeUid\"", nodesJson);

            var roadsJson = File.ReadAllText($"{prefix}_roads.json");
            Assert.Contains("\"roadUid\"", roadsJson);
            Assert.Contains("\"lookToken\"", roadsJson);
            Assert.Contains("\"fromNodeUid\"", roadsJson);
            Assert.Contains("\"toNodeUid\"", roadsJson);
            Assert.Contains("\"laneCount\"", roadsJson);

            var graphJson = File.ReadAllText($"{prefix}_graph.json");
            Assert.Contains("\"edgeUid\"", graphJson);
            Assert.Contains("\"distanceM\"", graphJson);
            Assert.Contains("\"speedLimitKmh\"", graphJson);
        }
        finally
        {
            foreach (var suffix in new[] { "_nodes.json", "_roads.json", "_road_looks.json", "_graph.json" })
            {
                var path = $"{prefix}{suffix}";
                if (File.Exists(path)) File.Delete(path);
            }
        }
    }

    [Fact]
    public void GraphMetrics_Compute_PercentagesAreCorrect()
    {
        var graph = new GraphData
        {
            Nodes = new List<GraphNode>
            {
                new() { Uid = 1 },
                new() { Uid = 2 },
                new() { Uid = 3 }
            },
            Edges = new List<GraphEdge>
            {
                new() { FromNodeUid = 1, ToNodeUid = 2, Direction = "forward", SpeedLimitKmh = 80 },
                new() { FromNodeUid = 2, ToNodeUid = 1, Direction = "backward" },
                new() { FromNodeUid = 2, ToNodeUid = 3, Direction = "bidirectional_unknown" },
                new() { FromNodeUid = 1, ToNodeUid = 3, Direction = "prefab_interconnect", SpeedLimitKmh = 60 }
            }
        };

        var metrics = MetricsCalculator.Compute(graph, 42.0);

        Assert.Equal(3, metrics.NodesTotal);
        Assert.Equal(4, metrics.EdgesTotal);
        Assert.Equal(42.0, metrics.BuildTimeMs);

        // directed = forward + backward = 2 / 4 = 50%
        Assert.Equal(50.0, metrics.PctDirected);
        // unknown = bidirectional_unknown = 1 / 4 = 25%
        Assert.Equal(25.0, metrics.PctUnknown);
        // withSpeed = forward + prefab_interconnect = 2 / 4 = 50%
        Assert.Equal(50.0, metrics.PctWithSpeedLimit);
        // density = edges / nodes = 4 / 3
        Assert.Equal(4.0 / 3.0, metrics.Density, 6);
        // largest component = all 3 nodes connected = 3/3 = 1.0
        Assert.Equal(1.0, metrics.LargestComponentRatio, 6);
    }

    [Fact]
    public void GraphMetrics_EmptyGraph_ReturnsZeros()
    {
        var graph = new GraphData { Nodes = new List<GraphNode>(), Edges = new List<GraphEdge>() };
        var metrics = MetricsCalculator.Compute(graph, 0.0);

        Assert.Equal(0, metrics.NodesTotal);
        Assert.Equal(0, metrics.EdgesTotal);
        Assert.Equal(0.0, metrics.PctDirected);
        Assert.Equal(0.0, metrics.PctUnknown);
        Assert.Equal(0.0, metrics.PctWithSpeedLimit);
        Assert.Equal(0.0, metrics.Density);
        Assert.Equal(0.0, metrics.LargestComponentRatio);
    }
}
