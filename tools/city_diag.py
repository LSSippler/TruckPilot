"""
city_diag.py — Per-city nearest-node diagnostic for TruckPilot graph.
Finds: nearest node (any), nearest connected node, nearest node per top CC.
Usage: python3 tools/city_diag.py [--graph graph.json]
"""
import json
import sys
import math
import argparse
from collections import Counter

CITIES = [
    ("Berlin",    -16400.0,  -3200.0),   # reference — should snap fine
    ("Madrid",    -65000.0,  14000.0),
    ("Valencia",  -58000.0,  14000.0),
    ("Lissabon",  -95000.0,  13000.0),
    ("Porto",     -95000.0,   4000.0),
    ("Oslo",      -19500.0, -33500.0),
    ("Goeteborg", -19500.0, -23000.0),
    ("Bergen",    -26000.0, -36000.0),
]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--graph", default="graph.json")
    args = ap.parse_args()

    print(f"Loading {args.graph} ...", flush=True)
    with open(args.graph, "rb") as f:
        data = json.load(f)

    nodes = data["nodes"]
    edges = data["edges"]
    print(f"  {len(nodes)} nodes, {len(edges)} edges", flush=True)

    # Build set of connected node UIDs (appear in at least one edge)
    print("Building connected-node set ...", flush=True)
    connected = set()
    for e in edges:
        connected.add(e["from"])
        connected.add(e["to"])
    print(f"  {len(connected)} connected nodes", flush=True)

    # Union-Find for CC membership
    print("Building CC (Union-Find) ...", flush=True)
    uid_to_idx = {n["uid"]: i for i, n in enumerate(nodes)}
    parent = list(range(len(nodes)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        a, b = find(a), find(b)
        if a != b:
            parent[a] = b

    for e in edges:
        ia = uid_to_idx.get(e["from"])
        ib = uid_to_idx.get(e["to"])
        if ia is not None and ib is not None:
            union(ia, ib)

    roots = [find(i) for i in range(len(nodes))]
    cc_sizes = Counter(roots)
    top_ccs = cc_sizes.most_common(5)
    print(f"  Top-5 CCs: {[s for _, s in top_ccs]}", flush=True)

    # Map CC-root -> rank (0 = largest)
    cc_rank = {root: rank for rank, (root, _) in enumerate(top_ccs)}
    top_cc_roots = set(r for r, _ in top_ccs)

    # Index nodes by their CC root
    print("Indexing nodes by CC ...", flush=True)
    # For each top-CC, we store all node indices in that CC
    cc_nodes = {root: [] for root, _ in top_ccs}
    for i, r in enumerate(roots):
        if r in cc_nodes:
            cc_nodes[r].append(i)

    def dist2(n, cx, cz):
        dx = n["x"] - cx
        dz = n["z"] - cz
        return dx * dx + dz * dz

    print()
    print("=" * 70)
    print("PER-CITY DIAGNOSTICS")
    print("=" * 70)

    for city_name, cx, cz in CITIES:
        best_any_d2 = math.inf
        best_any = None
        best_conn_d2 = math.inf
        best_conn = None
        # nearest per top-CC
        best_by_cc = {root: (math.inf, None) for root, _ in top_ccs}

        for n in nodes:
            d2 = dist2(n, cx, cz)
            if d2 < best_any_d2:
                best_any_d2 = d2
                best_any = n
            uid = n["uid"]
            if uid in connected:
                if d2 < best_conn_d2:
                    best_conn_d2 = d2
                    best_conn = n
            idx = uid_to_idx.get(uid)
            if idx is not None:
                r = roots[idx]
                if r in best_by_cc:
                    prev_d2, _ = best_by_cc[r]
                    if d2 < prev_d2:
                        best_by_cc[r] = (d2, n)

        def cc_label(n):
            if n is None:
                return "N/A"
            idx = uid_to_idx.get(n["uid"])
            if idx is None:
                return "NOT-IN-INDEX"
            r = roots[idx]
            size = cc_sizes[r]
            rank = cc_rank.get(r)
            tag = f"CC#{rank+1}" if rank is not None else "CC"
            return f"{tag}(size={size})"

        print()
        print(f"City: {city_name}  query=({cx}, {cz})")
        if best_any:
            d = math.sqrt(best_any_d2)
            print(f"  Nearest any:       uid=0x{best_any['uid']:016X}  pos=({best_any['x']:.1f}, {best_any['z']:.1f})  dist={d:.0f}m  {cc_label(best_any)}")
        if best_conn:
            d = math.sqrt(best_conn_d2)
            print(f"  Nearest connected: uid=0x{best_conn['uid']:016X}  pos=({best_conn['x']:.1f}, {best_conn['z']:.1f})  dist={d:.0f}m  {cc_label(best_conn)}")
        else:
            print(f"  Nearest connected: NONE")

        for root, size in top_ccs:
            d2, n = best_by_cc[root]
            rank = cc_rank[root]
            if n is not None:
                d = math.sqrt(d2)
                print(f"  Nearest CC#{rank+1}(sz={size}): uid=0x{n['uid']:016X}  pos=({n['x']:.1f}, {n['z']:.1f})  dist={d:.0f}m")
            else:
                print(f"  Nearest CC#{rank+1}(sz={size}): NONE")

    print()
    print("Done.")

if __name__ == "__main__":
    main()
