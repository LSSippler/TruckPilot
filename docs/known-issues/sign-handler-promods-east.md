# Known Issue: Sign-Handler crashes on ProMods-East content

**Status:** DEFERRED to Phase 6.4 (ProMods support)
**Last reviewed:** 2026-05-17
**Source phases:** 6.2b-Diag-4, 6.2b-Fix-1 (reverted), 6.2b-Fix-2

---

## Symptom

`SectorHandlerError` is raised in **9 sectors** for `item_type=36` (Sign).
`audit_sector` logs:

```
sector item #N (type=36) failed: Binary parse error:
  sign overrides: count <huge> exceeds safety limit
  — OR — sign override attrs: count <huge> exceeds safety limit
  — OR — unknown sign override attribute type 0
```

The affected sign item drops, all subsequent items in the sector are
dropped too (partial sector accepted), and any roads after the failed item
do not enter the graph.

## Affected sectors (9)

| Sector | Items parsed | Items total | Nearest Vanilla city | Crash count value |
|---|---|---|---|---|
| `sec+0029+0024` | 17 | 158 | Budapest @ 151 km | 1832976384 (attrs) |
| `sec+0029+0025` | 10 | 278 | Budapest @ 153 km | 16777217 (so_count) |
| `sec+0027+0025` | 6 | 109 | Budapest @ 147 km | 536870912 (so_count) |
| `sec+0014-0009` | 14 | 135 | Riga @ 50 km | 33554432 (so_count) |
| `sec+0028+0024` | 124 | 533 | Budapest @ 148 km | unknown attr_type 0 |
| `sec+0028+0015` | 13 | 348 | Vilnius @ 128 km | 33554432 (so_count) |
| `sec+0027+0022` | 5 | 878 | no nodes recovered | 3241426944 (attrs) |
| `sec+0026+0024` | 68 | 71 | Budapest @ 142 km | 1633837396 (attrs) |
| `sec+0027+0016` | 33 | 230 | no nodes recovered | 33554432 (so_count) |

All 9 sectors lie in the ProMods-East coverage area (eastern map region,
raw-x 56000–120000m). **No Vanilla city is isolated by these drops** —
nearest Vanilla city is always ≥ 50 km away.

## Diagnose-Zusammenfassung

Phase 6.2b-Diag-4 captured 256 bytes per crashing sign body. Phase
6.2b-Fix-2 expanded the audit to **1523 sectors, 8769 working signs**.
The combined empirical findings:

1. **Vanilla sign-override format works** (2347 Tier-3 working samples
   confirmed): the current `skip_sign_override_list` correctly parses
   each item as `u32 + u64 token + u32 attr_count + per-attr (u16 type +
   u32 + value)`.

2. **Tier 4 (`bo_count > 0` AND `so_count > 0`) is empty** across all
   8769 working signs. This rules out the simple "padding byte" fix
   tried in Phase 6.2b-Fix-1 (revert documented in
   `outputs/2026-05-17/phase_6.2b_fix1_status_report.md`).

3. **ProMods-East signs use a different per-override-item format.**
   Side-by-side hex of Vanilla `sec+0000+0001` #424 vs crash
   `sec+0027+0022` #5:

   ```
   offset  Vanilla (so=4)                  ProMods-East (so=3, crash)
   +000    00 00 a0 0c a3 0e 00 00         52 00 05 23 30 b2 c8 35
   +008    00 00 00 00 01 00 00 00         36 54 da 47 00 40 34 c1
   +016    04 00 04 00 00 00 00 00         28 07 ac 47 04 46 c3 47
   +024    00 00 00 00 90 50 a3 0e         58 22 38 46 1b b0 da 47
   ```

   Bytes `36 54 da 47` decode as f32 ≈ **112 000 m** (world x);
   `28 07 ac 47` as f32 ≈ **88 078 m** (world z). The ProMods-East
   item carries world-coordinate Vec3 data, not the Vanilla
   `attr_count + attr` sequence.

4. **`board_override_count` is NOT pure padding either** — 4 working
   Tier-2 samples in `sec+0025+0025`, `sec+0026+0025`, `sec+0027+0015`,
   `sec+0027+0023` show `bo_count` ∈ {2, 7} with the current per-item
   layout (`skip_token + u8 flags`) producing a valid `so_count=0`.
   These four are exactly the sectors that the Fix-1 padding-skip
   broke; they cleanly re-parse after the revert.

## Format hypothesis (to verify in Phase 6.4)

**Vanilla** (confirmed):
```
override_start + 0   : u32 board_override_count (often 0)
                       (in Vanilla almost always 0 → cursor moves +4)
override_start + 4   : u32 sign_override_count
override_start + 8   : sign_override_item[count]
                         u32 (board index / target)
                         u64 token (UID)
                         u32 attr_count
                         per attr: u16 type + u32 placeholder + value
```

**ProMods-East** (suspected):
```
override_start + 0   : same u32 leading field (sometimes a real
                       count, sometimes 0)
override_start + 4   : u32 sign_override_count (or possibly a
                       different layout — needs 256B+ capture)
override_start + 8   : sign_override_item with embedded Vec3s
                         8B header
                         Vec3 position (3×f32, ~world coords)
                         Vec3 second position (bbox? rotation?)
                         further data (variable per item)
```

Per-override-item size ≥ 32 B (probably variable). Exact structure
remains open — Phase 6.4 will need 256 B+ captures across all 9 crash
sectors and the 4 Tier-2 working ProMods-East samples to lock the layout.

## Impact

- **9 sectors abort partial-parse.** Items parsed before the failing sign
  (5 to 124 per sector) enter the graph; everything after is dropped.
- **No Vanilla city becomes unreachable.** All 9 sectors are ≥ 50 km
  from the nearest Vanilla city.
- **Phase 6.2b final state (post-Fix-5c):**
  - `SectorHandlerError` (sign-handler): 9 — unchanged by Phase 6.2b
  - `BothUnresolved`: 9 (was 1 878 before Phase 6.2b; -99.5%)
  - Roads in graph: 366 012
  - Nodes: 1 153 648
  - Sample route Berlin: 227 waypoints / 151 ms
  - `truckpilot-route-test`: routing functional for Vanilla city pairs

  The 9 sign-handler drops are now the **only notable sector abort
  category** for ProMods sign items. They do not affect Vanilla routing.

## Reactivation

Re-open this issue when **Phase 6.4 (ProMods support)** starts. Then:

1. Re-run `sign-format-compare` with capture window enlarged to
   256–512 B (currently 32 B beyond `bo_end_rel`).
2. Identify the per-override-item size by matching `so_count` items
   in the captured bytes.
3. Add a ProMods-format branch in `skip_sign_override_list` (or skip
   the override list entirely for ProMods signs if the layout
   resists clean parsing — sign overrides are visual overlays, not
   routing-critical).
4. Validate: `SectorHandlerError` should drop from 9 to 0; the four
   Tier-2 sectors (`sec+0025+0025` etc.) must continue to parse.

## References

- `outputs/2026-05-17/sign_format_compare_v3.md` — full audit
  (1523 sectors, 8769 working samples, 9 crash samples).
- `outputs/2026-05-17/phase_6.2b_fix2_status_report.md` — empirical
  findings.
- `outputs/2026-05-17/phase_6.2b_fix1_status_report.md` — first fix
  attempt and revert.
- `outputs/2026-05-17/sign_precrash_audit.md` — Phase 6.2b-Diag-4
  per-sector body decode.
- `crates/map-parser/src/sector.rs:1337` — `skip_sign` (current
  Vanilla reader, unchanged).
- `crates/diag/src/bin/sign-format-compare.rs` — audit binary
  (kept; used to track this issue in Phase 6.4).
