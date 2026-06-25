#!/usr/bin/env python3
"""Analyze route audit vs SCC membership."""
import csv
import json
import re
from collections import deque
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
g = json.loads((ROOT / "graph.json").read_text(encoding="utf-8"))
nodes = g["nodes"]
edges = g["edges"]
uid_to_i = {int(n["uid"]): i for i, n in enumerate(nodes)}
coords = [(n["x"], n["y"], n["z"]) for n in nodes]
adj = [[] for _ in nodes]
radj = [[] for _ in nodes]
for e in edges:
    fu = uid_to_i.get(int(e["from"]))
    tu = uid_to_i.get(int(e["to"]))
    if fu is None or tu is None:
        continue
    adj[fu].append(tu)
    radj[tu].append(fu)


def scc_nodes(start: int) -> set[int]:
    fwd = {start}
    stack = [start]
    while stack:
        v = stack.pop()
        for w in adj[v]:
            if w not in fwd:
                fwd.add(w)
                stack.append(w)
    rev = {start}
    stack = [start]
    while stack:
        v = stack.pop()
        for w in radj[v]:
            if w not in rev:
                rev.add(w)
                stack.append(w)
    return fwd & rev


truck_i = uid_to_i[3829402255816130719]
MAIN = scc_nodes(truck_i)

nodes_with_edges = [False] * len(nodes)
for e in edges:
    for uid in (int(e["from"]), int(e["to"])):
        if uid in uid_to_i:
            nodes_with_edges[uid_to_i[uid]] = True


def read_cities():
    cities = []
    cur = {}
    for raw in (ROOT / "crates/map-parser/tests/fixtures/test_cities.toml").read_text(
        encoding="utf-8"
    ).splitlines():
        line = raw.split("#")[0].strip()
        if not line:
            continue
        if line == "[[city]]":
            if cur.get("name"):
                cities.append(cur)
            cur = {}
            continue
        m = re.match(r"^(\w+)\s*=\s*(.+)$", line)
        if m:
            k, v = m.group(1), m.group(2).strip().strip('"')
            if k == "name":
                cur["name"] = v
            elif k in ("x", "z"):
                cur[k] = float(v)
    if cur.get("name"):
        cities.append(cur)
    return cities


def snap(x, z, r=5000.0):
    best = None
    bd = r * r
    for i, (nx, _, nz) in enumerate(coords):
        if not nodes_with_edges[i]:
            continue
        d2 = (nx - x) ** 2 + (nz - z) ** 2
        if d2 < bd:
            bd = d2
            best = i
    return best


cities = read_cities()
city_main = {}
for c in cities:
    i = snap(c["x"], c["z"])
    if i is None:
        city_main[c["name"]] = None
    else:
        city_main[c["name"]] = i in MAIN

rows = list(
    csv.DictReader(
        open(ROOT / "outputs/2026-06-14/routing-audit-tmp/route_audit.csv", encoding="utf-8")
    )
)

main_names = [n for n, m in city_main.items() if m]
print("MAIN SCC size", len(MAIN))
print("Cities in MAIN by snap", len(main_names), "/", len(cities))

both_main_ok = both_main = 0
main_main_fail = []
for r in rows:
    sm = city_main.get(r["source_city"])
    dm = city_main.get(r["target_city"])
    if sm and dm:
        both_main += 1
        if r["result"] == "CAT4_SUCCESS":
            both_main_ok += 1
        else:
            main_main_fail.append(r)

print("Main-main pairs", both_main, "success", both_main_ok, "rate", both_main_ok / both_main * 100)
print("Main-main failures", len(main_main_fail))

first10 = ["Berlin", "Hamburg", "Muenchen", "Wien", "Prag", "Warschau", "Amsterdam", "Bruessel", "Paris", "Mailand"]
sub = [r for r in rows if r["source_city"] in first10 and r["target_city"] in first10]
ok = sum(1 for r in sub if r["result"] == "CAT4_SUCCESS")
print("10-city 90 pairs", ok, "/", len(sub), "=", ok / len(sub) * 100)

out = {
    "main_scc_size": len(MAIN),
    "cities_in_main": len(main_names),
    "main_main_pairs": both_main,
    "main_main_success": both_main_ok,
    "ten_city_rate": f"{ok}/{len(sub)}",
    "main_main_failures": [
        {"source": r["source_city"], "target": r["target_city"], "result": r["result"]}
        for r in main_main_fail
    ],
    "city_in_main": city_main,
}
(ROOT / "outputs/2026-06-14/routing-audit-tmp/contradiction_analysis.json").write_text(
    json.dumps(out, indent=2), encoding="utf-8"
)
