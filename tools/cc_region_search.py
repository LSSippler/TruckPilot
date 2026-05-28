"""
cc_region_search.py — Find connected nodes in a bounding box.
Usage: python3 tools/cc_region_search.py --graph graph.json --xmin X --xmax X --zmin Z --zmax Z [--top N]
"""
import json
import sys
import math
import argparse
from collections import Counter

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--graph", default="graph.json")
    ap.add_argument("--xmin", type=float, required=True)
    ap.add_argument("--xmax", type=float, required=True)
    ap.add_argument("--zmin", type=float, required=True)
    ap.add_argument("--zmax", type=float, required=True)
    ap.add_argument("--top", type=int, default=20, help="show top N nodes by CC size")
    args = ap.parse_args()

    print(f"Loading {args.graph} ...", flush=True)
    with open(args.graph, "rb") as f:
        data = json.load(f)

    nodes = data["nodes"]
    edges = data["edges"]
    print(f"  {len(nodes)} nodes, {len(edges)} edges", flush=True)

    connected = set()
    for e in edges:
        connected.add(e["from"])
        connected.add(e["to"])

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
    cc_rank = {root: rank for rank, (root, _) in enumerate(top_ccs)}

    # Find connected nodes in bounding box
    hits = []
    for n in nodes:
        if args.xmin <= n["x"] <= args.xmax and args.zmin <= n["z"] <= args.zmax:
            uid = n["uid"]
            if uid in connected:
                idx = uid_to_idx.get(uid)
                r = roots[idx] if idx is not None else None
                cc_size = cc_sizes[r] if r is not None else 0
                rank = cc_rank.get(r, 99)
                degree = sum(1 for e in edges if e["from"] == uid or e["to"] == uid)
                hits.append((cc_size, rank, n["uid"], n, degree))

    hits.sort(key=lambda t: (-t[0], t[1], t[2]))

    print(f"\nBbox: x=[{args.xmin}, {args.xmax}]  z=[{args.zmin}, {args.zmax}]")
    print(f"Connected nodes in bbox: {len(hits)}")
    print()
    print(f"{'UID':20s}  {'x':>12s}  {'z':>12s}  {'CC-rank':>8s}  {'CC-size':>8s}")
    print("-" * 75)
    for cc_size, rank, _uid, n, deg in hits[:args.top]:
        tag = f"CC#{rank+1}" if rank < 5 else "other"
        print(f"0x{n['uid']:016X}  {n['x']:>12.1f}  {n['z']:>12.1f}  {tag:>8s}  {cc_size:>8d}")

if __name__ == "__main__":
    main()
