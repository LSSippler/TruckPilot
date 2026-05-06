using TruckPilot.Core.Parsers;

namespace TruckPilot.Tests;

public class MapParserTests
{
    [Fact]
    public void ParseTextMap_ShouldFindNodes()
    {
        var text = @"node { uid: 0x1 position: (0,0,0) }
node { uid: 0x2 position: (10,0,0) }
road { uid: 0xA name: ""test"" look_token: ""a"" nodes: (0x1, 0x2) speed_limit: 50.0 lane_count_forward: 1 lane_count_backward: 0 }";
        var (nodes, roads, _) = MapParser.ParseTextMap(text);
        Assert.Equal(2, nodes.Count);
        Assert.Single(roads);
        Assert.Equal(2, roads[0].NodeUids.Count);
    }

    [Fact]
    public void ParseTextMap_ShouldHandleWhitespaceBeforeColon()
    {
        var text = @"node { uid : 0x1 position : (0,0,0) }";
        var (nodes, _, _) = MapParser.ParseTextMap(text);
        Assert.Single(nodes);
    }

    [Fact]
    public void ParseBinarySector_ShouldFindAtLeastOneNode()
    {
        using var ms = new MemoryStream();
        using (var writer = new BinaryWriter(ms, System.Text.Encoding.Default, true))
        {
            // Header
            writer.Write((uint)906);
            writer.Write((ulong)0);
            writer.Write((uint)3);

            // Items
            writer.Write((uint)1); // item_count
            writer.Write((uint)3); // road item
            writer.Write((ulong)0xABCDEF0123456789); // road uid
            for (int i = 0; i < 10; i++) writer.Write(0.0f); // kdop bounds
            writer.Write((byte)0); // kdop flag 1
            writer.Write((byte)0); // kdop flag 2
            writer.Write((byte)0); // kdop flag 3
            writer.Write((byte)0); // kdop flag 4
            writer.Write((byte)0); // view distance
            writer.Write((byte)0); // road flags 1
            writer.Write((byte)0); // dlc guard
            writer.Write((byte)0); // road flags 3
            writer.Write((byte)0); // road flags 4
            for (int i = 0; i < 9; i++) writer.Write((ulong)0); // tokens up to left edge left
            writer.Write((ulong)0); // right terrain profile
            writer.Write(0.0f); // right terrain coef
            writer.Write((ulong)0); // left terrain profile
            writer.Write(0.0f); // left terrain coef
            writer.Write((ulong)0); // right look
            writer.Write((ulong)0); // left look
            writer.Write((ulong)0); // material
            for (int i = 0; i < 3; i++)
            {
                writer.Write((ulong)0); // right railing
                writer.Write((short)0); // right railing offset
                writer.Write((ulong)0); // left railing
                writer.Write((short)0); // left railing offset
            }
            writer.Write(0); // right height offset
            writer.Write(0); // left height offset
            writer.Write((ulong)0x123456789ABCDEF0); // backward node
            writer.Write((ulong)0xFEDCBA9876543210); // forward node
            writer.Write(100.0f); // length

            // Nodes
            writer.Write((uint)2); // node_count
            WriteNode(writer, 0x123456789ABCDEF0, 1.0f, 2.0f, 3.0f);
            WriteNode(writer, 0xFEDCBA9876543210, 4.0f, 5.0f, 6.0f);

            // Vis area child list
            writer.Write((uint)0);
        }

        var data = ms.ToArray();
        var (nodes, roads, prefabs) = BinaryMapParser.ParseSector(data);

        Assert.Equal(2, nodes.Count);
        Assert.Equal((ulong)0x123456789ABCDEF0, nodes[0].Uid);
        Assert.Equal(1.0f, nodes[0].X);
        Assert.Equal(2.0f, nodes[0].Y);
        Assert.Equal(3.0f, nodes[0].Z);

        Assert.Single(roads);
        Assert.Equal(2, roads[0].NodeUids.Count);
        Assert.Equal((uint)1, roads[0].LaneCountForward);
        Assert.Equal((uint)1, roads[0].LaneCountBackward);
        Assert.Equal(80.0f, roads[0].SpeedLimit);
    }

    private static void WriteNode(BinaryWriter writer, ulong uid, float x, float y, float z)
    {
        writer.Write(uid);
        writer.Write((int)(x * 256f));
        writer.Write((int)(y * 256f));
        writer.Write((int)(z * 256f));
        writer.Write(0.0f);
        writer.Write(0.0f);
        writer.Write(0.0f);
        writer.Write(1.0f);
        writer.Write((ulong)0);
        writer.Write((ulong)0);
        writer.Write((uint)((1u << 5) | (1u << 28)));
    }
}
