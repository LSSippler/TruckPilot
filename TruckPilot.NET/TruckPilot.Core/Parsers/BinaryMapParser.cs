using TruckPilot.Core.Models;

namespace TruckPilot.Core.Parsers;

/// <summary>Parses binary ETS2 map sector data (HashFS/ZIP extracted files).</summary>
public static class BinaryMapParser
{
    private const uint ItemRoad = 3;
    private const uint ItemPrefab = 4;
    private const uint ItemBuildings = 2;
    private const uint ItemTerrain = 1;
    private const uint ItemModel = 5;
    private const uint ItemBezierPatch = 39;
    private const uint ItemCurve = 44;
    private const uint ItemFarModel = 43;
    private const uint ItemCompany = 6;
    private const uint ItemService = 7;
    private const uint ItemCutPlane = 8;
    private const uint ItemCity = 12;
    private const uint ItemFerry = 19;
    private const uint ItemGarage = 22;
    private const uint ItemMapOverlay = 18;
    private const uint ItemFuelPump = 35;
    private const uint ItemSign = 36;
    private const uint ItemBusStop = 37;
    private const uint ItemTrafficArea = 38;
    private const uint ItemMapArea = 42;
    private const uint ItemTrajectory = 41;
    private const uint ItemTrigger = 34;
    private const uint ItemVisibilityArea = 48;
    private const uint ItemCutscene = 46;

    /// <remarks>
    /// Notes from TruckLib and map-docs research:
    /// - Sector files start with a header: u32 version, token game_id (u64), u32 map_version.
    /// - After the header: u32 item_count, then item_count items. Each item starts with u32 item_type
    ///   followed by item-specific data (no size field in this format).
    /// - After items: u32 node_count, then node_count nodes. Each node stores uid, fixed-point position,
    ///   rotation (quat), backward/forward item uid, and a flags field.
    /// - After nodes: u32 vis_area_child_count, then that many u64 uids.
    /// - Road and prefab attributes include a k-DOP item header (uid, bounds, flags, view distance) and
    ///   long token sequences; the parser advances through these fields to reach the node references.
    /// - Relevant item types for TruckPilot: road(3), prefab(4), company(6), city(12), ferry(19),
    ///   fuel pump(35), sign(36). Other types exist (terrain, building, model, etc.).
    /// - HashFS v2 entries can be compressed; decompression is handled before calling ParseSector.
    /// </remarks>
    public static MapData ParseSector(byte[] data)
    {
        var result = new MapData();
        int pos = 0;
        var nodeFlags = new Dictionary<ulong, uint>();

        uint version = ReadUInt32(data, ref pos);
        ulong gameId = ReadUInt64(data, ref pos);
        uint mapVersion = ReadUInt32(data, ref pos);

        uint itemCount = ReadUInt32(data, ref pos);

        for (uint i = 0; i < itemCount; i++)
        {
            uint itemType = ReadUInt32(data, ref pos);
            switch (itemType)
            {
                case ItemRoad:
                    result.Roads.Add(ParseRoad(data, ref pos));
                    break;
                case ItemPrefab:
                    result.Prefabs.Add(ParsePrefab(data, ref pos));
                    break;
                case ItemTerrain:
                    SkipTerrain(data, ref pos);
                    break;
                case ItemBuildings:
                    SkipBuildings(data, ref pos);
                    break;
                case ItemModel:
                    SkipModel(data, ref pos);
                    break;
                case ItemBezierPatch:
                    SkipBezierPatch(data, ref pos);
                    break;
                case ItemCurve:
                    SkipCurve(data, ref pos);
                    break;
                case ItemFarModel:
                    SkipFarModel(data, ref pos);
                    break;
                case ItemCompany:
                    result.Companies.Add(ParseCompany(data, ref pos));
                    break;
                case ItemCity:
                    result.Cities.Add(ParseCity(data, ref pos));
                    break;
                case ItemFerry:
                    result.Ferries.Add(ParseFerry(data, ref pos));
                    break;
                case ItemFuelPump:
                    result.FuelPumps.Add(ParseFuelPump(data, ref pos));
                    break;
                case ItemSign:
                    result.Signs.Add(ParseSign(data, ref pos));
                    break;
                case ItemBusStop:
                    SkipBusStop(data, ref pos);
                    break;
                case ItemCutPlane:
                    SkipCutPlane(data, ref pos);
                    break;
                case ItemCutscene:
                    SkipCutscene(data, ref pos);
                    break;
                case ItemGarage:
                    SkipGarage(data, ref pos);
                    break;
                case ItemMapOverlay:
                    SkipMapOverlay(data, ref pos);
                    break;
                case ItemMapArea:
                    SkipMapArea(data, ref pos);
                    break;
                case ItemService:
                    SkipService(data, ref pos);
                    break;
                case ItemTrafficArea:
                    SkipTrafficArea(data, ref pos);
                    break;
                case ItemTrajectory:
                    SkipTrajectory(data, ref pos);
                    break;
                case ItemTrigger:
                    SkipTrigger(data, ref pos);
                    break;
                case ItemVisibilityArea:
                    SkipVisibilityArea(data, ref pos);
                    break;
                default:
                    throw new InvalidDataException($"Unsupported item type {itemType} at offset {pos - 4} (sector v{version}, map v{mapVersion}, game_id=0x{gameId:X16}).");
            }
        }

        uint nodeCount = ReadUInt32(data, ref pos);
        for (uint i = 0; i < nodeCount; i++)
        {
            var node = ParseNode(data, ref pos, nodeFlags);
            if (node != null)
                result.Nodes.Add(node);
        }

        uint visAreaChildCount = ReadUInt32(data, ref pos);
        for (uint i = 0; i < visAreaChildCount; i++)
        {
            ReadUInt64(data, ref pos);
        }

        result.Roads = ApplyLaneCounts(result.Roads, nodeFlags);

        return result;
    }

    private static MapNode? ParseNode(byte[] data, ref int pos, Dictionary<ulong, uint> nodeFlags)
    {
        ulong uid = ReadUInt64(data, ref pos);
        int fx = ReadInt32(data, ref pos);
        int fy = ReadInt32(data, ref pos);
        int fz = ReadInt32(data, ref pos);

        float x = fx / 256f;
        float y = fy / 256f;
        float z = fz / 256f;

        SkipQuaternion(data, ref pos);

        ulong backward = ReadUInt64(data, ref pos);
        ulong forward = ReadUInt64(data, ref pos);
        uint flags = ReadUInt32(data, ref pos);

        if (!IsValidUid(uid)) return null;
        if (!IsPlausibleCoord(x, y, z)) return null;

        if (!nodeFlags.ContainsKey(uid))
            nodeFlags.Add(uid, flags);

        return new MapNode
        {
            Uid = uid,
            X = x,
            Y = y,
            Z = z,
            Rotation = 0f,
            BackwardItemUid = backward == 0 ? null : backward,
            ForwardItemUid = forward == 0 ? null : forward
        };
    }

    private static MapRoad ParseRoad(byte[] data, ref int pos)
    {
        var kdopUid = ReadUInt64(data, ref pos);
        SkipKdopBounds(data, ref pos);

        ReadByte(data, ref pos); // kdop flag 1
        ReadByte(data, ref pos); // kdop flag 2
        ReadByte(data, ref pos); // kdop flag 3
        ReadByte(data, ref pos); // kdop flag 4
        ReadByte(data, ref pos); // view distance

        ReadByte(data, ref pos); // road flags byte 1
        ReadByte(data, ref pos); // dlc guard
        ReadByte(data, ref pos); // road flags byte 3
        ReadByte(data, ref pos); // road flags byte 4

        SkipToken(data, ref pos); // road type
        SkipToken(data, ref pos); // right traffic rule
        SkipToken(data, ref pos); // left traffic rule
        SkipToken(data, ref pos); // right variant
        SkipToken(data, ref pos); // left variant
        SkipToken(data, ref pos); // right edge right
        SkipToken(data, ref pos); // right edge left
        SkipToken(data, ref pos); // left edge right
        SkipToken(data, ref pos); // left edge left
        SkipToken(data, ref pos); // right terrain profile
        ReadSingle(data, ref pos); // right terrain coef
        SkipToken(data, ref pos); // left terrain profile
        ReadSingle(data, ref pos); // left terrain coef
        SkipToken(data, ref pos); // right look
        SkipToken(data, ref pos); // left look
        SkipToken(data, ref pos); // material

        for (int i = 0; i < 3; i++)
        {
            SkipToken(data, ref pos); // right railing
            ReadInt16(data, ref pos); // right railing offset
            SkipToken(data, ref pos); // left railing
            ReadInt16(data, ref pos); // left railing offset
        }

        ReadInt32(data, ref pos); // right height offset
        ReadInt32(data, ref pos); // left height offset

        ulong backwardNode = ReadUInt64(data, ref pos);
        ulong forwardNode = ReadUInt64(data, ref pos);
        float length = ReadSingle(data, ref pos);

        var nodes = new List<ulong>(2);
        if (IsValidUid(backwardNode)) nodes.Add(backwardNode);
        if (IsValidUid(forwardNode)) nodes.Add(forwardNode);

        return new MapRoad
        {
            Uid = kdopUid,
            NodeUids = nodes,
            Length = length,
            LaneCountForward = 0u,
            LaneCountBackward = 0u,
            SpeedLimit = 80f
        };
    }

    private static MapPrefab ParsePrefab(byte[] data, ref int pos)
    {
        var kdopUid = ReadKdopItem(data, ref pos);

        ulong modelToken = ReadUInt64(data, ref pos);
        SkipToken(data, ref pos); // variant

        uint addPartCount = ReadUInt32(data, ref pos);
        for (uint i = 0; i < addPartCount; i++) SkipToken(data, ref pos);

        uint nodeCount = ReadUInt32(data, ref pos);
        var nodes = new List<ulong>((int)nodeCount);
        for (uint i = 0; i < nodeCount; i++)
        {
            ulong nuid = ReadUInt64(data, ref pos);
            if (IsValidUid(nuid)) nodes.Add(nuid);
        }

        uint slaveCount = ReadUInt32(data, ref pos);
        for (uint i = 0; i < slaveCount; i++) ReadUInt64(data, ref pos);

        ReadUInt64(data, ref pos); // ferry link
        ReadUInt16(data, ref pos); // origin idx

        for (uint i = 0; i < nodeCount; i++)
        {
            SkipToken(data, ref pos); // terrain profile
            ReadSingle(data, ref pos); // terrain coef
        }

        SkipToken(data, ref pos); // semaphore profile

        return new MapPrefab
        {
            Uid = kdopUid,
            DescriptorToken = FormatToken(modelToken),
            NodeUids = nodes
        };
    }

    private static MapCompany ParseCompany(byte[] data, ref int pos)
    {
        var kdopUid = ReadKdopItem(data, ref pos);

        ulong cityToken = ReadUInt64(data, ref pos);
        ReadUInt64(data, ref pos); // prefab uid
        ulong companyToken = ReadUInt64(data, ref pos);
        ulong nodeUid = ReadUInt64(data, ref pos);

        uint spawnCount = ReadUInt32(data, ref pos);
        var nodes = new List<ulong>((int)spawnCount);
        for (uint i = 0; i < spawnCount; i++)
        {
            ulong nuid = ReadUInt64(data, ref pos);
            if (IsValidUid(nuid)) nodes.Add(nuid);
        }
        for (uint i = 0; i < spawnCount; i++) ReadUInt32(data, ref pos);

        return new MapCompany
        {
            Uid = kdopUid,
            CompanyToken = FormatToken(companyToken),
            NodeUid = IsValidUid(nodeUid) ? nodeUid : null,
            SpawnNodeUids = nodes
        };
    }

    private static MapCity ParseCity(byte[] data, ref int pos)
    {
        var kdopUid = ReadKdopItem(data, ref pos);

        ulong cityToken = ReadUInt64(data, ref pos);
        float width = ReadSingle(data, ref pos);
        float height = ReadSingle(data, ref pos);
        ulong nodeUid = ReadUInt64(data, ref pos);

        return new MapCity
        {
            Uid = kdopUid,
            CityToken = FormatToken(cityToken),
            Width = width,
            Height = height,
            NodeUid = IsValidUid(nodeUid) ? nodeUid : null
        };
    }

    private static MapFerry ParseFerry(byte[] data, ref int pos)
    {
        var kdopUid = ReadKdopItem(data, ref pos);

        ulong ferryToken = ReadUInt64(data, ref pos);
        ulong prefabUid = ReadUInt64(data, ref pos);
        ulong nodeUid = ReadUInt64(data, ref pos);
        ReadSingle(data, ref pos); // unload offset x
        ReadSingle(data, ref pos); // unload offset y
        ReadSingle(data, ref pos); // unload offset z

        return new MapFerry
        {
            Uid = kdopUid,
            FerryToken = FormatToken(ferryToken),
            PrefabUid = IsValidUid(prefabUid) ? prefabUid : null,
            NodeUid = IsValidUid(nodeUid) ? nodeUid : null
        };
    }

    private static MapFuelPump ParseFuelPump(byte[] data, ref int pos)
    {
        var kdopUid = ReadKdopItem(data, ref pos);

        ulong nodeUid = ReadUInt64(data, ref pos);
        ulong prefabUid = ReadUInt64(data, ref pos);

        uint nodeCount = ReadUInt32(data, ref pos);
        var nodes = new List<ulong>((int)nodeCount);
        for (uint i = 0; i < nodeCount; i++)
        {
            ulong nuid = ReadUInt64(data, ref pos);
            if (IsValidUid(nuid)) nodes.Add(nuid);
        }

        return new MapFuelPump
        {
            Uid = kdopUid,
            NodeUid = IsValidUid(nodeUid) ? nodeUid : null,
            PrefabUid = IsValidUid(prefabUid) ? prefabUid : null,
            NodeUids = nodes
        };
    }

    private static MapSign ParseSign(byte[] data, ref int pos)
    {
        var kdopUid = ReadKdopItem(data, ref pos);

        ulong modelToken = ReadUInt64(data, ref pos);
        ulong nodeUid = ReadUInt64(data, ref pos);
        ulong lookToken = ReadUInt64(data, ref pos);
        ulong variantToken = ReadUInt64(data, ref pos);

        byte boardCount = ReadByte(data, ref pos);
        for (int i = 0; i < boardCount; i++)
        {
            SkipToken(data, ref pos);
            SkipToken(data, ref pos);
            SkipToken(data, ref pos);
        }

        var templateLength = SkipPascalString(data, ref pos);
        if (templateLength > 0)
        {
            SkipSignBoardOverrideList(data, ref pos);
            SkipSignOverrideList(data, ref pos);
        }

        return new MapSign
        {
            Uid = kdopUid,
            ModelToken = FormatToken(modelToken),
            LookToken = FormatToken(lookToken),
            VariantToken = FormatToken(variantToken),
            NodeUid = IsValidUid(nodeUid) ? nodeUid : null
        };
    }

    private static void SkipBusStop(byte[] data, ref int pos)
    {
        ReadKdopItem(data, ref pos);
        SkipToken(data, ref pos); // city token
        ReadUInt64(data, ref pos); // prefab
        ReadUInt64(data, ref pos); // node
    }

    private static void SkipBuildings(byte[] data, ref int pos)
    {
        ReadKdopItem(data, ref pos);
        SkipToken(data, ref pos); // name
        SkipToken(data, ref pos); // look
        ReadUInt64(data, ref pos); // node
        ReadUInt64(data, ref pos); // forward node
        ReadSingle(data, ref pos); // length
        ReadUInt32(data, ref pos); // random seed
        ReadSingle(data, ref pos); // stretch
        SkipFloatList(data, ref pos);
    }

    private static void SkipModel(byte[] data, ref int pos)
    {
        ReadKdopItem(data, ref pos);
        SkipToken(data, ref pos); // name
        SkipToken(data, ref pos); // look
        SkipToken(data, ref pos); // variant
        SkipTokenList(data, ref pos); // additional parts
        ReadUInt64(data, ref pos); // node
        SkipVector3(data, ref pos); // scale
        SkipToken(data, ref pos); // terrain material
        SkipColor(data, ref pos); // terrain color
        ReadSingle(data, ref pos); // terrain rotation
    }

    private static void SkipBezierPatch(byte[] data, ref int pos)
    {
        ReadKdopItem(data, ref pos);
        for (int i = 0; i < 16; i++) SkipVector3(data, ref pos); // control points
        ReadUInt16(data, ref pos); // tesselation x
        ReadUInt16(data, ref pos); // tesselation z
        ReadUInt64(data, ref pos); // node
        ReadUInt32(data, ref pos); // random seed
        for (int i = 0; i < 3; i++)
        {
            SkipToken(data, ref pos);
            ReadUInt16(data, ref pos);
            ReadByte(data, ref pos);
        }
        SkipVegetationSphereList(data, ref pos);
        SkipTerrainQuadData(data, ref pos);
    }

    private static void SkipCurve(byte[] data, ref int pos)
    {
        ReadKdopItem(data, ref pos);
        SkipToken(data, ref pos); // model
        ReadUInt64(data, ref pos); // node
        ReadUInt64(data, ref pos); // forward node
        ReadUInt64(data, ref pos); // locator 1
        ReadUInt64(data, ref pos); // locator 2
        ReadSingle(data, ref pos); // length
        ReadUInt32(data, ref pos); // random seed
        ReadSingle(data, ref pos); // stretch
        ReadSingle(data, ref pos); // scale
        ReadSingle(data, ref pos); // fixed step
        SkipToken(data, ref pos); // terrain material
        SkipColor(data, ref pos); // terrain color
        ReadSingle(data, ref pos); // terrain rotation
        SkipToken(data, ref pos); // first part
        SkipToken(data, ref pos); // last part
        SkipToken(data, ref pos); // center part variation
        SkipToken(data, ref pos); // look
        SkipFloatList(data, ref pos); // height offsets
    }

    private static void SkipFarModel(byte[] data, ref int pos)
    {
        ReadKdopItem(data, ref pos);
        ReadSingle(data, ref pos); // width
        ReadSingle(data, ref pos); // height
        uint modelCount = ReadUInt32(data, ref pos);
        for (uint i = 0; i < modelCount; i++)
        {
            SkipToken(data, ref pos);
            SkipVector3(data, ref pos);
        }
        uint childCount = ReadUInt32(data, ref pos);
        for (uint i = 0; i < childCount; i++) ReadUInt64(data, ref pos);
        uint nodeCount = ReadUInt32(data, ref pos);
        for (uint i = 0; i < nodeCount; i++) ReadUInt64(data, ref pos);
    }

    private static void SkipTerrain(byte[] data, ref int pos)
    {
        ReadUInt64(data, ref pos); // uid
        SkipKdopBounds(data, ref pos);
        ReadByte(data, ref pos); // kflag1
        ReadByte(data, ref pos); // kflag2
        ReadByte(data, ref pos); // kflag3
        ReadByte(data, ref pos); // kflag4
        ReadByte(data, ref pos); // view distance
        ReadUInt64(data, ref pos); // node
        ReadUInt64(data, ref pos); // forward node
        SkipVector3(data, ref pos); // node offset
        SkipVector3(data, ref pos); // forward node offset
        ReadSingle(data, ref pos); // length
        ReadSingle(data, ref pos); // previous length
        ReadUInt32(data, ref pos); // random seed
        for (int i = 0; i < 3; i++)
        {
            SkipToken(data, ref pos);
            ReadInt16(data, ref pos);
        }
        for (int side = 0; side < 2; side++)
        {
            ReadUInt16(data, ref pos); // terrain size
            SkipToken(data, ref pos); // profile
            ReadSingle(data, ref pos); // coefficient
            SkipToken(data, ref pos); // prev profile
            ReadSingle(data, ref pos); // prev coef
            for (int veg = 0; veg < 3; veg++) SkipRoadVegetation(data, ref pos);
            ReadUInt16(data, ref pos); // no detail veg from
            ReadUInt16(data, ref pos); // no detail veg to
        }
        SkipVegetationSphereList(data, ref pos);
        SkipTerrainQuadData(data, ref pos); // right
        SkipTerrainQuadData(data, ref pos); // left
        SkipToken(data, ref pos); // right edge
        SkipToken(data, ref pos); // right edge look
        SkipToken(data, ref pos); // left edge
        SkipToken(data, ref pos); // left edge look
    }

    private static void SkipCutPlane(byte[] data, ref int pos)
    {
        ReadKdopItem(data, ref pos);
        SkipNodeRefList(data, ref pos);
    }

    private static void SkipCutscene(byte[] data, ref int pos)
    {
        ReadKdopItem(data, ref pos);
        SkipTokenList(data, ref pos);
        ReadUInt64(data, ref pos); // node
        SkipActionList(data, ref pos, includeName: false);
    }

    private static void SkipGarage(byte[] data, ref int pos)
    {
        ReadKdopItem(data, ref pos);
        SkipToken(data, ref pos); // city token
        ReadUInt32(data, ref pos); // is_buy_point
        ReadUInt64(data, ref pos); // node
        ReadUInt64(data, ref pos); // prefab
        SkipNodeRefList(data, ref pos);
    }

    private static void SkipMapOverlay(byte[] data, ref int pos)
    {
        ReadKdopItem(data, ref pos);
        SkipToken(data, ref pos); // look
        ReadUInt64(data, ref pos); // node
    }

    private static void SkipMapArea(byte[] data, ref int pos)
    {
        ReadKdopItem(data, ref pos);
        SkipNodeRefList(data, ref pos);
        ReadUInt32(data, ref pos); // color
    }

    private static void SkipService(byte[] data, ref int pos)
    {
        ReadKdopItem(data, ref pos);
        ReadUInt64(data, ref pos); // node
        ReadUInt64(data, ref pos); // prefab
        SkipNodeRefList(data, ref pos);
    }

    private static void SkipTrafficArea(byte[] data, ref int pos)
    {
        ReadKdopItem(data, ref pos);
        SkipTokenList(data, ref pos);
        SkipNodeRefList(data, ref pos);
        SkipToken(data, ref pos); // rule
        ReadSingle(data, ref pos); // range
    }

    private static void SkipTrajectory(byte[] data, ref int pos)
    {
        ReadKdopItem(data, ref pos);
        SkipNodeRefList(data, ref pos);
        SkipToken(data, ref pos); // access rule
        SkipTrajectoryRuleList(data, ref pos);
        uint checkpointCount = ReadUInt32(data, ref pos);
        for (uint i = 0; i < checkpointCount; i++)
        {
            SkipToken(data, ref pos);
            SkipToken(data, ref pos);
        }
        SkipTokenList(data, ref pos);
    }

    private static void SkipTrigger(byte[] data, ref int pos)
    {
        ReadKdopItem(data, ref pos);
        SkipTokenList(data, ref pos);
        uint nodeCount = SkipNodeRefList(data, ref pos);
        SkipActionList(data, ref pos, includeName: true);
        if (nodeCount == 1)
            ReadSingle(data, ref pos);
    }

    private static void SkipVisibilityArea(byte[] data, ref int pos)
    {
        ReadKdopItem(data, ref pos);
        ReadUInt64(data, ref pos); // node
        ReadSingle(data, ref pos); // width
        ReadSingle(data, ref pos); // height
        SkipItemRefList(data, ref pos);
    }

    private static ulong ReadKdopItem(byte[] data, ref int pos)
    {
        ulong uid = ReadUInt64(data, ref pos);
        SkipKdopBounds(data, ref pos);
        ReadUInt32(data, ref pos); // flags
        ReadByte(data, ref pos); // view distance
        return uid;
    }

    private static void SkipKdopBounds(byte[] data, ref int pos)
    {
        for (int i = 0; i < 10; i++) ReadSingle(data, ref pos);
    }

    private static void SkipQuaternion(byte[] data, ref int pos)
    {
        ReadSingle(data, ref pos);
        ReadSingle(data, ref pos);
        ReadSingle(data, ref pos);
        ReadSingle(data, ref pos);
    }

    private static void SkipToken(byte[] data, ref int pos) => ReadUInt64(data, ref pos);

    private static ulong SkipPascalString(byte[] data, ref int pos)
    {
        ulong len = ReadUInt64(data, ref pos);
        if (len > int.MaxValue)
            throw new InvalidDataException($"String length {len} exceeds limit at offset {pos}.");
        pos = checked(pos + (int)len);
        EnsureAvailable(data, pos - 1);
        return len;
    }

    private static void SkipTokenList(byte[] data, ref int pos)
    {
        uint count = ReadUInt32(data, ref pos);
        for (uint i = 0; i < count; i++) SkipToken(data, ref pos);
    }

    private static uint SkipNodeRefList(byte[] data, ref int pos)
    {
        uint count = ReadUInt32(data, ref pos);
        for (uint i = 0; i < count; i++) ReadUInt64(data, ref pos);
        return count;
    }

    private static void SkipItemRefList(byte[] data, ref int pos)
    {
        uint count = ReadUInt32(data, ref pos);
        for (uint i = 0; i < count; i++) ReadUInt64(data, ref pos);
    }

    private static void SkipTrajectoryRuleList(byte[] data, ref int pos)
    {
        uint count = ReadUInt32(data, ref pos);
        for (uint i = 0; i < count; i++)
        {
            ReadUInt32(data, ref pos); // node index
            SkipToken(data, ref pos); // rule token
            uint paramCount = ReadUInt32(data, ref pos);
            for (uint p = 0; p < paramCount; p++) ReadSingle(data, ref pos);
        }
    }

    private static void SkipActionList(byte[] data, ref int pos, bool includeName)
    {
        uint count = ReadUInt32(data, ref pos);
        for (uint i = 0; i < count; i++)
        {
            if (includeName) SkipToken(data, ref pos);
            SkipActionBase(data, ref pos);
        }
    }

    private static void SkipActionBase(byte[] data, ref int pos)
    {
        uint numParamCount = ReadUInt32(data, ref pos);
        if (numParamCount == uint.MaxValue) return;
        for (uint i = 0; i < numParamCount; i++) ReadSingle(data, ref pos);

        uint stringParamCount = ReadUInt32(data, ref pos);
        for (uint i = 0; i < stringParamCount; i++) SkipPascalString(data, ref pos);

        uint targetTagCount = ReadUInt32(data, ref pos);
        for (uint i = 0; i < targetTagCount; i++) SkipToken(data, ref pos);

        ReadSingle(data, ref pos); // target range
        ReadUInt32(data, ref pos); // flags
    }

    private static void SkipSignBoardOverrideList(byte[] data, ref int pos)
    {
        uint count = ReadUInt32(data, ref pos);
        for (uint i = 0; i < count; i++)
        {
            SkipToken(data, ref pos); // area name
            byte flags = ReadByte(data, ref pos);
            if ((flags & 0x01) != 0)
            {
                ReadByte(data, ref pos); // x offset (sbyte)
                ReadByte(data, ref pos); // y offset (sbyte)
            }
            if ((flags & 0x02) != 0)
            {
                SkipToken(data, ref pos); // board
            }
        }
    }

    private static void SkipSignOverrideList(byte[] data, ref int pos)
    {
        uint count = ReadUInt32(data, ref pos);
        for (uint i = 0; i < count; i++)
        {
            ReadUInt32(data, ref pos); // id
            SkipToken(data, ref pos); // area name
            uint attrCount = ReadUInt32(data, ref pos);
            for (uint a = 0; a < attrCount; a++)
            {
                ushort type = ReadUInt16(data, ref pos);
                ReadUInt32(data, ref pos); // index
                switch (type)
                {
                    case 1: // sbyte
                        ReadByte(data, ref pos);
                        break;
                    case 2: // int32
                        ReadInt32(data, ref pos);
                        break;
                    case 3: // uint32
                        ReadUInt32(data, ref pos);
                        break;
                    case 4: // float
                        ReadSingle(data, ref pos);
                        break;
                    case 5: // string
                        SkipPascalString(data, ref pos);
                        break;
                    case 6: // uint64
                        ReadUInt64(data, ref pos);
                        break;
                    default:
                        throw new InvalidDataException($"Unknown sign override attribute type {type} at offset {pos}.");
                }
            }
        }
    }

    private static void SkipRoadVegetation(byte[] data, ref int pos)
    {
        SkipToken(data, ref pos);
        ReadUInt16(data, ref pos);
        ReadByte(data, ref pos);
        ReadByte(data, ref pos);
        ReadUInt16(data, ref pos);
        ReadUInt16(data, ref pos);
    }

    private static void SkipVegetationSphereList(byte[] data, ref int pos)
    {
        uint count = ReadUInt32(data, ref pos);
        for (uint i = 0; i < count; i++)
        {
            SkipVector3(data, ref pos);
            ReadSingle(data, ref pos);
            ReadUInt32(data, ref pos);
        }
    }

    private static void SkipTerrainQuadData(byte[] data, ref int pos)
    {
        ushort brushMatCount = ReadUInt16(data, ref pos);
        for (int i = 0; i < brushMatCount; i++)
        {
            SkipToken(data, ref pos);
            ReadUInt16(data, ref pos);
        }
        ushort colorCount = ReadUInt16(data, ref pos);
        for (int i = 0; i < colorCount; i++)
        {
            ReadByte(data, ref pos);
            ReadByte(data, ref pos);
            ReadByte(data, ref pos);
            ReadByte(data, ref pos);
        }
        ReadUInt16(data, ref pos); // rows
        ReadUInt16(data, ref pos); // cols
        uint quadCount = ReadUInt32(data, ref pos);
        for (uint i = 0; i < quadCount; i++)
        {
            ReadByte(data, ref pos);
            ReadByte(data, ref pos);
            ReadByte(data, ref pos);
            ReadByte(data, ref pos);
        }
        uint offsetCount = ReadUInt32(data, ref pos);
        for (uint i = 0; i < offsetCount; i++)
        {
            ReadUInt16(data, ref pos);
            ReadUInt16(data, ref pos);
            SkipVector3(data, ref pos);
        }
        uint normalCount = ReadUInt32(data, ref pos);
        for (uint i = 0; i < normalCount; i++)
        {
            ReadUInt16(data, ref pos);
            ReadUInt16(data, ref pos);
            SkipVector3(data, ref pos);
        }
    }

    private static void SkipFloatList(byte[] data, ref int pos)
    {
        uint count = ReadUInt32(data, ref pos);
        for (uint i = 0; i < count; i++) ReadSingle(data, ref pos);
    }

    private static void SkipVector3(byte[] data, ref int pos)
    {
        ReadSingle(data, ref pos);
        ReadSingle(data, ref pos);
        ReadSingle(data, ref pos);
    }

    private static void SkipColor(byte[] data, ref int pos)
    {
        ReadByte(data, ref pos);
        ReadByte(data, ref pos);
        ReadByte(data, ref pos);
        ReadByte(data, ref pos);
    }

    private static string FormatToken(ulong token) => token == 0 ? "" : $"0x{token:X16}";

    private static List<MapRoad> ApplyLaneCounts(List<MapRoad> roads, Dictionary<ulong, uint> nodeFlags)
    {
        if (roads.Count == 0) return roads;
        var updated = new List<MapRoad>(roads.Count);
        foreach (var road in roads)
        {
            uint forward = 0;
            uint backward = 0;
            if (road.NodeUids.Count == 2)
            {
                var hasFlags = false;
                if (nodeFlags.TryGetValue(road.NodeUids[0], out var bFlags))
                {
                    hasFlags = true;
                    if (HasForwardTraffic(bFlags)) forward = 1;
                    if (HasBackwardTraffic(bFlags)) backward = 1;
                }
                if (nodeFlags.TryGetValue(road.NodeUids[1], out var fFlags))
                {
                    hasFlags = true;
                    if (HasForwardTraffic(fFlags)) forward = 1;
                    if (HasBackwardTraffic(fFlags)) backward = 1;
                }
                if (!hasFlags || (forward == 0 && backward == 0))
                {
                    forward = 1;
                    backward = 1;
                }
            }

            updated.Add(new MapRoad
            {
                Uid = road.Uid,
                Name = road.Name,
                LookToken = road.LookToken,
                Length = road.Length,
                NodeUids = road.NodeUids,
                SpeedLimit = road.SpeedLimit,
                LaneCountForward = forward,
                LaneCountBackward = backward
            });
        }
        return updated;
    }

    private static bool HasForwardTraffic(uint flags)
    {
        return ((flags >> 5) & 0x1) != 0 || ((flags >> 6) & 0x1) != 0 || ((flags >> 7) & 0x1) != 0;
    }

    private static bool HasBackwardTraffic(uint flags)
    {
        return ((flags >> 28) & 0x1) != 0 || ((flags >> 29) & 0x1) != 0 || ((flags >> 30) & 0x1) != 0;
    }

    private static bool IsValidUid(ulong uid) => uid != 0 && uid >= 0x1000;

    private static bool IsPlausibleCoord(float x, float y, float z)
    {
        return Math.Abs(x) <= 300_000 && Math.Abs(z) <= 300_000 && Math.Abs(y) <= 20_000;
    }

    private static byte ReadByte(byte[] data, ref int pos)
    {
        EnsureAvailable(data, pos);
        return data[pos++];
    }

    private static ushort ReadUInt16(byte[] data, ref int pos)
    {
        EnsureAvailable(data, pos + 1);
        ushort value = BitConverter.ToUInt16(data, pos);
        pos += 2;
        return value;
    }

    private static int ReadInt16(byte[] data, ref int pos)
    {
        EnsureAvailable(data, pos + 1);
        short value = BitConverter.ToInt16(data, pos);
        pos += 2;
        return value;
    }

    private static uint ReadUInt32(byte[] data, ref int pos)
    {
        EnsureAvailable(data, pos + 3);
        uint value = BitConverter.ToUInt32(data, pos);
        pos += 4;
        return value;
    }

    private static int ReadInt32(byte[] data, ref int pos)
    {
        EnsureAvailable(data, pos + 3);
        int value = BitConverter.ToInt32(data, pos);
        pos += 4;
        return value;
    }

    private static ulong ReadUInt64(byte[] data, ref int pos)
    {
        EnsureAvailable(data, pos + 7);
        ulong value = BitConverter.ToUInt64(data, pos);
        pos += 8;
        return value;
    }

    private static float ReadSingle(byte[] data, ref int pos)
    {
        EnsureAvailable(data, pos + 3);
        float value = BitConverter.ToSingle(data, pos);
        pos += 4;
        return value;
    }

    private static void EnsureAvailable(byte[] data, int index)
    {
        if (index >= data.Length)
            throw new InvalidDataException($"Unexpected end of sector data at offset {index}.");
    }
}
