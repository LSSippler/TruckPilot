# Known Issue: ProMods Bezier Patch Desync

**Status:** DEFERRED to Phase 6.4 (ProMods support)
**Last reviewed:** 2026-05-17
**Source phases:** 6.2b-Fix-5b, 6.2b-Fix-5c

---

## Symptom

In 9 of the 11 remaining `BothUnresolved` events after Phase 6.2b-Fix-5c,
the root cause is `sec-0001-0008` and similar ProMods v2.82 sectors where
a `bezier_patch` item is the **last item in the sector**.

The `skip_bezier_patch` handler consumes more bytes than the actual item
contains, overrunning into the trailing-nodes block. This causes:

- `all_items_parsed = false`
- `recover_nodes_from_tail` activates → produces 1 phantom node
- The phantom node holds a road reference → `BothUnresolved` road

## Mechanism

```
sector layout:
  item #1 … item #N-1  ← parsed correctly
  item #N (bezier_patch) ← handler over-reads by ~10 000+ bytes
  [trailing-node block] ← partially consumed as bezier payload
```

Post-Fix-5c skip overhead for an empty bezier_patch = 318 bytes. For
certain ProMods sectors the actual item body is shorter — the item_count
confirms all items are present (410/411 for `sec-0001-0008`) but the last
one overflows.

## Empirical evidence

`sec-0001-0008` multi-sector-audit (post-Fix-5c):
- Items: 410/411 parsed OK
- Item #411 (`bezier_patch`): overflow / cursor past end
- Result: 1 recovered phantom node → 1 BothUnresolved road

9 of 11 total BothUnresolved events trace back to this pattern.
The remaining 2 are in non-bezier sectors and unrelated to this issue.

## Impact

- **9 roads** affected out of 366,012 total (0.002%)
- Geographic area: ProMods Nordost-Extension (eastern Europe, x ≥ 60 000 m)
- **No `cities.toml` route is blocked** — all 9 sectors are far from
  Vanilla ETS2 cities
- Routing metric: unaffected (sample Berlin route 227 waypoints / 151 ms)

## Location

`crates/map-parser/src/sector.rs` — `skip_bezier_patch`

## Fix hypothesis (Phase 6.4)

ProMods v2.82 likely uses a shorter `bezier_patch` variant — possibly
fewer vegetation entries, a different tess-grid size, or a stripped
TerrainQuadData block. Two candidate fixes:

1. **Format-detect via size field:** If the sector binary carries an
   item-size prefix (sized-format sectors), use it to hard-skip the
   exact byte count regardless of layout.
2. **Empirical re-audit:** Run `bezier-size-scan` on ProMods sectors to
   measure actual body sizes, then trace which structural field differs
   from the Vanilla layout.

## Reactivation

Re-open when Phase 6.4 (ProMods support) starts. Run:

```powershell
.\target\release\bezier-size-scan.exe `
  --ets2-dir "C:\Program Files (x86)\Steam\steamapps\common\Euro Truck Simulator 2" `
  --mods-dir "$env:USERPROFILE\Documents\Euro Truck Simulator 2\mod" `
  --sector "map/europe/sec-0001-0008.base" `
  --max-bytes 2000
```

Compare empirical body_size against the current `skip_bezier_patch`
fixed overhead (318 bytes base) to locate the delta.

## References

- `outputs/2026-05-17/phase_6.2b_fix5c_status.md` — Fix-5c analysis
- `outputs/2026-05-17/road_drop_audit_post_fix5c.json` — road-drop events
- `crates/map-parser/src/sector.rs:skip_bezier_patch` — current handler
- `crates/diag/src/bin/bezier-size-scan.rs` — brute-force body-size scanner
