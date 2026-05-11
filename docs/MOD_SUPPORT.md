# TruckPilot Mod-Support

TruckPilot kann beliebige ETS2-Karten-Mods (`*.scs`) zusammen mit dem
Basisspiel laden, Sektor-Overrides auflösen und Connector-Patches
korrekt einordnen. Dieses Dokument beschreibt das System und zeigt
Beispiel-Konfigurationen.

## Konzept

| Begriff           | Bedeutung                                                   |
|-------------------|-------------------------------------------------------------|
| `ModDescriptor`   | Beschreibt eine `.scs`-Datei (Pfad, Name, `load_order`).    |
| `load_order`      | Priorität — **höher gewinnt** bei Sektor-Kollisionen.        |
| `ModLoadOrder`    | Geordnete Liste von Deskriptoren plus Basisspiel-Pfade.     |
| `MultiArchiveReader` | Öffnet alle Archive und löst Sektoren nach Priorität auf. |

Pro Sektorpfad (`map/europe/sec±XXXX±YYYY.base`) gewinnt die Quelle mit
dem höchsten `load_order`. Das Basisspiel hat per Konvention `order = 0`,
Karten-Mods kleinere zweistellige Werte, Connector-Patches drei- oder
vierstellige Werte.

## Aktivierung über die CLI

```bash
cargo run --release -- \
  --ets2-dir "/pfad/zu/Euro Truck Simulator 2" \
  --mod-dir  "$HOME/Documents/Euro Truck Simulator 2/mod" \
  --enable-mods \
  --verbose
```

Ohne `--enable-mods` werden `--mod-dir` und `--mod-order` ignoriert; das
bisherige Single-Archive-Verhalten bleibt erhalten.

### Flags

| Flag             | Bedeutung                                                          |
|------------------|--------------------------------------------------------------------|
| `--enable-mods`  | Schaltet die Mod-Verarbeitung ein.                                  |
| `--mod-dir DIR`  | Verzeichnis mit `.scs`-Dateien. Alphabetische Default-Reihenfolge.  |
| `--mod-order F`  | JSON-Datei mit benutzerdefinierter Reihenfolge (siehe unten).       |

Wenn weder `--mod-dir` noch `--mod-order` gesetzt sind, wird automatisch
auf das Basisspiel zurückgefallen.

## `mod_order.json`

```json
{
  "base_game_files": ["base.scs", "base_map.scs", "def.scs"],
  "mod_descriptors": [
    {"name": "ProMods Map",      "file": "promods-map-v269.scs",      "order": 10},
    {"name": "ProMods Assets",   "file": "promods-assets-v269.scs",   "order": 11},
    {"name": "RusMap",           "file": "rusmap.scs",                "order": 20},
    {"name": "Africa",           "file": "africa.scs",                "order": 30},
    {"name": "PM-RM Connector",  "file": "promods_rusmap_connector.scs", "order": 999}
  ]
}
```

* `base_game_files` werden relativ zu `--ets2-dir` aufgelöst.
* `file` in `mod_descriptors` wird relativ zu `--mod-dir` aufgelöst,
  absolute Pfade sind ebenfalls erlaubt.
* `order` darf beliebige `u32`-Werte annehmen.
* `is_enabled` (default `true`) erlaubt das temporäre Abschalten ohne
  Einträge zu löschen.

## Programmatische API

```rust
use std::path::Path;
use truckpilot::ets2_parser::{ModLoadOrder, parse_ets2_map_with_mods};

let order = ModLoadOrder::from_directory(Path::new("/home/me/mods"))?;
let map   = parse_ets2_map_with_mods(Path::new("/games/ets2"), &order)?;

println!("nodes: {}", map.nodes.len());
```

Wer mehr Kontrolle braucht, kann den `MultiArchiveReader` direkt
benutzen:

```rust
use truckpilot::ets2_parser::{ModLoadOrder, MultiArchiveReader};

let order  = ModLoadOrder::from_json_file(json_path, Some(mod_dir), Some(game_dir))?;
let mut r  = MultiArchiveReader::load_from_order(&order)?;
for path in r.get_all_sectors() {
    if let Some(bytes) = r.get_sector(&path) {
        // ... eigene Logik ...
    }
}
```

## Sektor-Override und Node-Deduplizierung

Beim Mergen mehrerer Sektoren gilt:

* **Nodes**: First-occurrence wins (per UID dedupliziert).
* **Roads / Prefabs**: Werden konkateniert; das anschließende
  `dedup_map` räumt globale UID-Duplikate auf.

Das funktioniert, weil ETS2-UIDs global eindeutig sind — derselbe Node
erscheint nicht mit unterschiedlichen Daten in zwei Sektoren.

## Beispiel-Kombinationen

### ProMods solo

```bash
truckpilot --enable-mods \
  --ets2-dir "/games/ets2" \
  --mod-dir  "/games/ets2-mods/promods/"
```

### ProMods + RusMap + Connector

```bash
truckpilot --enable-mods \
  --ets2-dir  "/games/ets2" \
  --mod-order /games/ets2-mods/order.json
```

mit `order.json` siehe oben.

### Africa + ProMods Asia + Connector

Identisches Schema — nur die Pfade und `order`-Werte ändern sich. Das
System ist *generisch* und kennt keine Mod-spezifische Hard-codierung.

## Tests

`tests/mod_support_test.rs` enthält neun Integration-Tests:

| Test                                   | Verifiziert                                |
|----------------------------------------|---------------------------------------------|
| `test_mod_duplicate_nodes`             | Node-Dedup beim Sector-Merge                |
| `test_mod_sector_override`             | Mod überschreibt Basisspiel-Sektor          |
| `test_multi_mod_loading`               | Drei Mods nebeneinander                     |
| `test_connector_priority`              | Connector schlägt alle Karten-Mods          |
| `test_mod_discovery`                   | `expand_mod_list` sortiert alphabetisch      |
| `test_mod_order_json`                  | JSON-Reihenfolge wird übernommen            |
| `test_combined_map_parsing`            | Combined > base                             |
| `test_idempotent_parsing`              | Determinismus                               |
| `test_resolve_then_get_sector_consistency` | Index ↔ Bytes konsistent                |

Alle grün:

```bash
cargo test --test mod_support_test
```

## Fallstricke

* `--enable-mods` ohne `--mod-dir`/`--mod-order` ist erlaubt — wirkt
  dann wie kein Mod-Support.
* Die `order`-Werte in `mod_order.json` dürfen Lücken enthalten;
  Sortierung ist stabil.
* Sektoren, die in keinem Archiv existieren, werden ignoriert.
* Wenn `parse_ets2_map_with_mods` keine Nodes findet, fällt es
  automatisch auf `parse_ets2_map` zurück.
