# Known Issue: Remaining bezier_patch Count Overflow Aborts

**Status:** DEFERRED — reactivate when routing is blocked or Phase 6.4 starts
**Last reviewed:** 2026-05-17
**Source phases:** 6.2b-Fix-5c

---

## Symptom

After Phase 6.2b-Fix-5c (~40 sector aborts remain with patterns like:

```
bezier_patch quads count 3925868544 exceeds safety limit
bezier_patch offsets count 130817536 exceeds safety limit
bezier_patch vegetation spheres count 4289802304 exceeds safety limit
```

These are distinct from the ProMods-Bezier-Desync issue
(`docs/known-issues/promods-bezier-desync.md`): affected sectors fail
**mid-item** (not at the last item), and the garbage counts appear to
be float bytes interpreted as u32 integers.

## Garbage-count fingerprint

Known observed garbage values and their byte interpretation:

| Value | Hex | Likely origin |
|---|---|---|
| 3925868544 | `0xEA000000` | Upper byte = float exponent |
| 4289802304 | `0xFFB80000` | NaN / very large float |
| 117440512  | `0x07000000` | Small float near zero |
| 16798660   | `0x01005884` | Low-value float |

Pattern: the `u32 count` field is being read from Vec3 coordinate bytes.
This indicates a cursor drift of **N bytes earlier than expected** in the
`skip_bezier_patch` handler.

## Hypothesis

A structural field before the count-field is mis-sized for this class of
sectors. Candidates:

1. **Material-refs block:** There may be a per-entry structure between
   `seed` and `vegetation` that is not present in the 6 audit sectors
   used for Fix-5c. If this block is skipped with the wrong size, all
   subsequent reads are shifted.
2. **Tess-block variant:** Sectors with larger `tess_x × tess_z` values
   (e.g. 8×8 or higher) may carry additional data in the tess header.
3. **Version-gated field:** A conditional field (version byte or flags)
   that exists in some bezier variants but not in the Fix-5c sample set.

## Impact

- ~40 sectors abort; each loses the items after the failing bezier_patch
- Routing: not currently blocked (all affected sectors are away from
  tested city pairs)
- Exact road count: not yet measured (deferred)

## Location

`crates/map-parser/src/sector.rs` — `skip_bezier_patch`

## Trigger to reactivate

- A new `cities.toml` pair fails to route AND the blocking sectors appear
  in this abort list
- Phase 6.4 ProMods work begins (likely shares the same root cause)

## Fix approach (when activated)

1. Run `bezier-size-scan` on 3–5 sectors from this abort list to get
   empirical body_size values
2. Trace the cursor byte-by-byte against the Fix-5c layout to find the
   delta
3. Apply minimal targeted fix (single constant or conditional skip)
4. Verify with multi-sector-audit: `items_parsed == item_count` on
   the failing sectors

## References

- `outputs/2026-05-17/phase_6.2b_fix5c_status.md` — Fix-5c derivation
- `crates/diag/src/bin/bezier-size-scan.rs` — brute-force body-size scanner
- `crates/diag/src/bin/multi-sector-audit.rs` — audit tool for verification
- `docs/known-issues/promods-bezier-desync.md` — related but distinct issue
