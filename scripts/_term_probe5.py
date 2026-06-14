import pymem, struct, sys, ijson
sys.path.insert(0, "scripts")
from resolve_gps import resolve_gps_manager

UID_MIN = 5_000_000_000_000_000_000
STRIDE = 0x40

pm = pymem.Pymem("eurotrucks2.exe")
gps = resolve_gps_manager(pm)
arr = pm.read_ulonglong(
    pm.read_ulonglong(
        pm.read_ulonglong(
            pm.read_ulonglong(pm.read_ulonglong(gps + 0x08) + 0x58) + 0x2C0
        )
        + 0x1A8
    )
    + 0x18
    + 0x50
)

graph = set()
with open("graph.json", "rb") as f:
    for i, n in enumerate(ijson.items(f, "nodes.item")):
        graph.add(int(n["uid"]))


def walk_trim_u0c(max_n=2500):
    uids = []
    flags = []
    for i in range(max_n):
        raw = pm.read_bytes(arr + i * STRIDE, STRIDE)
        uid = struct.unpack_from("<Q", raw, 0x30)[0]
        u0c = struct.unpack_from("<I", raw, 0x0C)[0]
        if uid == 0 or uid < UID_MIN:
            break
        uids.append(uid)
        flags.append(u0c)
    # trim trailing u0c==0
    while uids and flags[-1] == 0:
        uids.pop()
        flags.pop()
    return uids


def walk_5e18_only(max_n=2500):
    uids = []
    for i in range(max_n):
        uid = pm.read_ulonglong(arr + i * STRIDE + 0x30)
        if uid == 0 or uid < UID_MIN:
            break
        uids.append(uid)
    return uids


def graph_last_contiguous(uids):
    last = -1
    for i, u in enumerate(uids):
        if u in graph:
            last = i
        else:
            if last >= 0 and i == last + 1:
                break
    return last + 1


for name, fn in [("5e18", walk_5e18_only), ("5e18+trim_u0c", walk_trim_u0c)]:
    uids = fn()
    g_last = max((i for i, u in enumerate(uids) if u in graph), default=-1)
    g_prefix = graph_last_contiguous(uids)
    print(f"{name}: count={len(uids)} graph_last_idx={g_last} graph_prefix={g_prefix}")
    print(f"  last uid={uids[-1] if uids else None}")
    print(f"  first5={uids[:3]}")

# stability: run trim walk 5 times
counts = [len(walk_trim_u0c()) for _ in range(5)]
print(f"trim stability 5x: {counts}")
