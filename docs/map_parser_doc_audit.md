# Map-Parser Inline-Doc Audit

Scope: `crates/map-parser/` — all library modules (`src/*.rs`) plus 16 binary
entries (`src/bin/*.rs`). Read-only documentation pass — **no behaviour
changes**. Goal: a Phase 6 developer (or the maintainer after a four-week
break) can read this crate without reaching for vault notes.

## 1. Before-state inventory

Per library module: source-of-truth counts measured at HEAD~1 (commit
`8589639`).

| File              | LOC  | Module header `//!` | `pub fn/struct/enum` | Doc-commented | Coverage |
| ----------------- | ---- | ------------------- | -------------------- | ------------- | -------- |
| `archive.rs`      | 72   | yes                 | 1                    | 1             | 100 %    |
| `cache.rs`        | 201  | yes                 | 4                    | 4             | 100 %    |
| `cityhash.rs`     | 319  | yes                 | 1                    | 1             | 100 %    |
| `error.rs`        | 43   | yes                 | 1                    | 1             | 100 %    |
| `graph.rs`        | 784  | yes                 | 9                    | 9             | 100 %    |
| `hashfs.rs`       | 902  | yes                 | 11                   | 11            | 100 %    |
| `lib.rs`          | 25   | yes                 | 0                    | n/a           | n/a      |
| `mod_loader.rs`   | 324  | yes                 | 4                    | 3             | **75 %** |
| `road_full.rs`    | 743  | yes                 | 23                   | 23            | 100 %    |
| `sector.rs`       | 1945 | yes                 | 12                   | 12            | 100 %    |
| `signs.rs`        | 135  | yes                 | 3                    | 3             | 100 %    |
| `spatial_match.rs`| 465  | yes                 | 14                   | 12            | **86 %** |
| `zip_archive.rs`  | 76   | yes                 | 2                    | 2             | 100 %    |
| **TOTAL**         | 6034 | 12/12               | 85                   | 82            | **96 %** |

Binaries (`src/bin/*.rs`, 16 files): 15/16 had `//!` headers. The single
miss was `map_probe.rs` (2-line stub, no header).

Summary: documentation coverage was **already high** before this pass.
The prompt anticipated significantly more drift; in practice the codebase
had been kept current alongside the Phase 5 sub-phase work.

## 2. After-state

| Metric                                  | Before | After |
| --------------------------------------- | -----: | ----: |
| Library `//!` module headers            |  12/12 | 12/12 |
| Library `pub` items with `///`          |  82/85 | 85/85 |
| `cargo doc --no-deps` warnings/errors   |      1 |     0 |
| `cargo doc -D broken_intra_doc_links`   |    fails | clean |
| Binary `//!` headers                    |  15/16 | 16/16 |

## 3. What was added

### Module headers (Task 2)
- `src/bin/map_probe.rs` — stub header explaining the placeholder
  semantics so future contributors know not to delete the slot.

### `///` doc-comments on public APIs (Task 3)
- `mod_loader::load_and_build` — pipeline summary, error semantics, cache
  hit-path.
- `spatial_match::SpatialIndex::total_nodes` — one-liner.
- `spatial_match::SpatialIndex::cell_count` — one-liner.

### Broken intra-doc link fix (cargo-doc gate)
- `road_full.rs` module header pointed
  `` [`crate::sector::parse_sector_legacy`] `` at a private fn. Rustdoc
  rejects this under `-D rustdoc::broken_intra_doc_links`. Replaced with
  a plain prose reference; no public surface change.

### Inline cursor / offset comments (Task 4)
None added. The codebase parses ETS2 sectors via `binrw` structs rather
than manual `cursor += N` arithmetic, so there were essentially no naked
byte-offset increments to annotate. The non-trivial offset math that
does exist (`hashfs::resolve_data_part`, `sector::recover_nodes_from_tail`)
already has docstring-level explanation including the algebra
(`count_pos + 4 + N*56 + 4 + M*8 == data.len()`) and the rationale
(favour the largest plausible anchor). Adding inline `// skip token`
breadcrumbs on top would have been noise.

## 4. Outdated comments triaged (Task 5)

| Location                | Comment                                          | Decision |
| ----------------------- | ------------------------------------------------ | -------- |
| `hashfs.rs:632`         | `TODO: texture entries can carry multiple data parts (MIP_0, MIP_TAIL)…` | **Kept** — accurately describes a real gap deferred to a future texture path. Not stale. |
| 36 × `Phase 5.X` references across `cache.rs`, `road_full.rs`, `sector.rs`, `bin/curve_audit.rs` | Historical context inside doc-comments (e.g. *"Phase 5.16 rewrite: the legacy 3-token+u16+token+u16+u32+u32+3×float-list…"*) | **Kept** — these explain *why* the code looks the way it does after each rewrite. Removing them would lose the rationale; the phases themselves are closed but the consequences live in the code. |
| `cache.rs:19`           | `const PARSER_VERSION: u32 = 3; // Bumped: GraphEdge gained dlc_guard/is_hidden/gps_avoid (Phase 5.6)` | **Kept** — schema invalidation breadcrumb is still relevant. |

No stale comments needed deletion. No comment was found to describe code
that is actually wrong (no Bug-Fund per the STOP-CONDITION).

## 5. Deliberately NOT documented

- **Private impl details inside `sector.rs` per-item parsers** — many of
  the `fn read_*` helpers are private, single-call, and their bodies are
  short binrw `Vec<u8>` reads. Doc-commenting every private one would be
  noise without adding navigation value.
- **`binrw`-derived field layouts in `road_full.rs`** — the `#[br(...)]`
  attributes and struct field names already document the wire format; an
  extra `///` per field would duplicate without clarifying.
- **Magic constants inside hashfs metadata parsing** — `0x40`, `0x80`,
  the kind-bit-7 flag, etc. are explained in the function-level docstring
  for `resolve_data_part` and `parse_dir_entry`. Per-line repetition was
  declined.

## 6. Gate results

| Check                                                        | Result |
| ------------------------------------------------------------ | ------ |
| `cargo build --workspace --exclude truckpilot-ui`            | PASS   |
| `cargo test --workspace --exclude truckpilot-ui`             | PASS (226) |
| `cargo clippy --all-targets --exclude truckpilot-ui -D warnings` | PASS |
| `cargo doc --no-deps -p truckpilot-map-parser` (with `-D rustdoc::broken_intra_doc_links`) | PASS |

Note: `truckpilot-ui` is excluded because Tauri requires `gdk-3.0` on
Linux and the sandbox has no GTK toolchain. The exclusion is invariant
across the branch; this audit does not regress that.

## 7. What this audit does NOT address

- Vault sync (`docs/vault/`) — the vault tree is not checked into git
  (per CLAUDE.md), so phase-back-references stay in code rather than as
  wikilinks from these docs.
- Architecture-level prose (call graphs, module diagrams). Out of scope
  for an inline-comment pass.
- Diag-crate documentation. The prompt scoped this to `crates/map-parser/`.
