namespace TruckPilot.Core.Models;

/// <summary>Position and identity of a point in the road network.</summary>
public sealed class GraphNode
{
    public ulong Uid { get; init; }
    public double X { get; init; }
    public double Y { get; init; }
    public double Z { get; init; }
}

/// <summary>Directed edge between two nodes with road attributes.</summary>
public sealed class GraphEdge
{
    public ulong EdgeUid { get; init; }
    public ulong FromNodeUid { get; init; }
    public ulong ToNodeUid { get; init; }
    public string? RoadUid { get; init; }
    public double DistanceM { get; init; }
    public string Direction { get; init; } = "";
    public uint LaneCount { get; init; }
    public double? SpeedLimitKmh { get; init; }
    public List<string> Flags { get; init; } = new();
}

/// <summary>Top-level graph structure with metadata.</summary>
public sealed class GraphData
{
    public GraphMeta Meta { get; set; } = new();
    public List<GraphNode> Nodes { get; set; } = new();
    public List<GraphEdge> Edges { get; set; } = new();
}

/// <summary>Origin and version info for a graph export.</summary>
public sealed class GraphMeta
{
    public string SchemaVersion { get; init; } = "1.0.0";
    public string MapName { get; init; } = "unknown";
    public string GeneratedAt { get; init; } = "";
}

/// <summary>Aggregated map data from one or more sectors.</summary>
public sealed class MapData
{
    public List<MapNode> Nodes { get; set; } = new();
    public List<MapRoad> Roads { get; set; } = new();
    public List<MapPrefab> Prefabs { get; set; } = new();
    public List<MapCompany> Companies { get; set; } = new();
    public List<MapCity> Cities { get; set; } = new();
    public List<MapFerry> Ferries { get; set; } = new();
    public List<MapFuelPump> FuelPumps { get; set; } = new();
    public List<MapSign> Signs { get; set; } = new();

    public void Deconstruct(out List<MapNode> nodes, out List<MapRoad> roads, out List<MapPrefab> prefabs)
    {
        nodes = Nodes;
        roads = Roads;
        prefabs = Prefabs;
    }
}

/// <summary>Parsed map node from binary/text format.</summary>
public sealed class MapNode
{
    public ulong Uid { get; init; }
    public double X { get; init; }
    public double Y { get; init; }
    public double Z { get; init; }
    public float Rotation { get; init; }
    public ulong? ForwardItemUid { get; init; }
    public ulong? BackwardItemUid { get; init; }
}

/// <summary>Parsed road from binary/text format.</summary>
public sealed class MapRoad
{
    public ulong Uid { get; init; }
    public string Name { get; init; } = "";
    public string LookToken { get; init; } = "";
    public float Length { get; init; }
    public List<ulong> NodeUids { get; init; } = new();
    public float? SpeedLimit { get; init; }
    public uint LaneCountForward { get; init; }
    public uint LaneCountBackward { get; init; }
}

/// <summary>Parsed prefab from binary/text format.</summary>
public sealed class MapPrefab
{
    public ulong Uid { get; init; }
    public string DescriptorToken { get; init; } = "";
    public List<ulong> NodeUids { get; init; } = new();
}

/// <summary>Parsed company item (prefab slave).</summary>
public sealed class MapCompany
{
    public ulong Uid { get; init; }
    public string CompanyToken { get; init; } = "";
    public ulong? NodeUid { get; init; }
    public List<ulong> SpawnNodeUids { get; init; } = new();
}

/// <summary>Parsed city item.</summary>
public sealed class MapCity
{
    public ulong Uid { get; init; }
    public string CityToken { get; init; } = "";
    public float? Width { get; init; }
    public float? Height { get; init; }
    public ulong? NodeUid { get; init; }
}

/// <summary>Parsed ferry item.</summary>
public sealed class MapFerry
{
    public ulong Uid { get; init; }
    public string FerryToken { get; init; } = "";
    public ulong? PrefabUid { get; init; }
    public ulong? NodeUid { get; init; }
}

/// <summary>Parsed fuel pump item.</summary>
public sealed class MapFuelPump
{
    public ulong Uid { get; init; }
    public ulong? NodeUid { get; init; }
    public ulong? PrefabUid { get; init; }
    public List<ulong> NodeUids { get; init; } = new();
}

/// <summary>Parsed sign item.</summary>
public sealed class MapSign
{
    public ulong Uid { get; init; }
    public string ModelToken { get; init; } = "";
    public string LookToken { get; init; } = "";
    public string VariantToken { get; init; } = "";
    public ulong? NodeUid { get; init; }
}

/// <summary>Result of a route planning operation.</summary>
public sealed class RouteResult
{
    public List<ulong> Path { get; init; } = new();
    public double TotalCost { get; init; }
    public ulong EdgesExamined { get; init; }
    public ulong NodesExpanded { get; init; }
    public bool Validated { get; init; }
    public double PlanningTimeMs { get; init; }
}

/// <summary>Configuration for A* routing.</summary>
public sealed class RouteConfig
{
    public bool PreferSpeed { get; init; }
    public CostMode Mode { get; init; } = CostMode.Distance;
}

public enum CostMode { Distance, Eta }
