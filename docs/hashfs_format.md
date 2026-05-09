# SCS HashFS — Format-Spezifikation (eigene Worte)

> Ziel: ein verständliches Referenzdokument für unseren eigenen Rust-Parser.
> Kein fremder Code wird übernommen — nur die durch unabhängige Quellen
> bestätigten Format-Fakten werden hier zusammengetragen.
>
> **Status:** Erste Version, basierend auf Quellen vom 2026-05-09. Jede
> Aussage ist mit der Quelle markiert, aus der sie stammt — siehe Legende.

## Quellen-Legende

| Tag    | Quelle                                                                                          | Sprache    | Priorität |
| ------ | ----------------------------------------------------------------------------------------------- | ---------- | --------- |
| `[MD]` | sk-zk/map-docs Wiki — <https://github.com/sk-zk/map-docs/wiki>                                  | Doku       | 1         |
| `[TL]` | sk-zk/TruckLib.HashFs — <https://github.com/sk-zk/TruckLib.HashFs>                              | C#         | 1         |
| `[AS]` | nautofon/Archive-SCS — <https://github.com/nautofon/Archive-SCS> (`lib/Archive/SCS/HashFS{,2}.pm`) | Perl       | 2         |
| `[TM]` | truckermudgeon/maps — `packages/clis/parser/game-files/scs-archive.ts`                          | TypeScript | 3         |

> Hinweis zu `[MD]`: Das Wiki dokumentiert ausschließlich das Map-Binary-Format
> (`.mbd`/sektor-Dateien). Eine separate Wiki-Seite zum HashFS-Container existiert
> nicht — der HashFS-Layer wird nur durch `[TL]`, `[AS]` und `[TM]` belegt.
> `[MD]` bleibt aber relevant, sobald wir den **Inhalt** der `map/europe/sec…`-Dateien
> auswerten, die aus dem HashFS gezogen werden.

Endianness: durchgehend **Little-Endian**. — `[TL]` `[AS]` `[TM]`

---

## 1. Magic-Bytes & Versions-Detection

| Feld         | Offset | Größe | Erwarteter Wert                              | Quellen           |
| ------------ | ------ | ----- | -------------------------------------------- | ----------------- |
| `magic`      | 0x00   | 4 B   | ASCII `"SCS#"` (`53 43 53 23`)              | `[TL]` `[AS]` `[TM]` |
| `version`    | 0x04   | 2 B   | `u16` LE — `1` (HashFS v1) oder `2` (HashFS v2) | `[TL]` `[AS]` `[TM]` |
| `salt`       | 0x06   | 2 B   | `u16` LE — in der Praxis immer `0`          | `[TL]` `[AS]` `[TM]` |
| `hash_method`| 0x08   | 4 B   | ASCII `"CITY"` (`43 49 54 59`)              | `[TL]` `[AS]` `[TM]` |

Diese ersten **12 Bytes sind in v1 und v2 identisch** — die Versions-Erkennung
erfolgt ausschließlich über `version` (Offset 0x04). — `[AS]`

`salt` ist als 16-Bit-Feld vorhanden, aber in allen bekannten Archiven `0`. Sowohl
Archive-SCS als auch TruckLib lehnen Werte ≠ 0 mit "unsupported" ab — entsprechend
fließt der Salt **nicht** in die CityHash-Berechnung ein, solange er 0 ist
(siehe §6). — `[AS]`

---

## 2. Header-Layout

### 2.1 HashFS v1 — 20 Byte Header

| Offset | Größe | Typ  | Feld           | Bedeutung                                                                      | Quellen        |
| ------ | ----- | ---- | -------------- | ------------------------------------------------------------------------------ | -------------- |
| 0x00   | 4     | A4   | `magic`        | `"SCS#"`                                                                       | `[TL]` `[AS]` |
| 0x04   | 2     | u16  | `version`      | `1`                                                                            | `[TL]` `[AS]` |
| 0x06   | 2     | u16  | `salt`         | `0`                                                                            | `[AS]`         |
| 0x08   | 4     | A4   | `hash_method`  | `"CITY"`                                                                       | `[TL]` `[AS]` |
| 0x0C   | 4     | u32  | `entry_count`  | Anzahl Einträge in der Entry-Tabelle                                          | `[TL]` `[AS]` |
| 0x10   | 4     | u32  | `start_offset` | Absolute Datei-Position der Entry-Tabelle (typischerweise `0x40`)             | `[TL]` `[AS]` |

Direkt nach Position `start_offset` liegen `entry_count` × **32-Byte-Einträge**
(siehe §3.1) — _flach im Klartext, **nicht** komprimiert_. — `[AS]`

### 2.2 HashFS v2 — 52 Byte Header (53 mit `platform`)

| Offset | Größe | Typ  | Feld                            | Bedeutung                                                                                           | Quellen                  |
| ------ | ----- | ---- | ------------------------------- | --------------------------------------------------------------------------------------------------- | ------------------------ |
| 0x00   | 4     | A4   | `magic`                         | `"SCS#"`                                                                                            | `[TL]` `[AS]` `[TM]`    |
| 0x04   | 2     | u16  | `version`                       | `2`                                                                                                 | `[TL]` `[AS]` `[TM]`    |
| 0x06   | 2     | u16  | `salt`                          | `0`                                                                                                 | `[AS]` `[TL]`            |
| 0x08   | 4     | A4   | `hash_method`                   | `"CITY"`                                                                                            | `[TL]` `[AS]` `[TM]`    |
| 0x0C   | 4     | u32  | `entry_count`                   | Anzahl Einträge in **Index1** (= Entry-Tabelle)                                                    | `[TL]` `[AS]` `[TM]`    |
| 0x10   | 4     | u32  | `entry_table_compressed_size`   | Komprimierte Größe der Entry-Tabelle in Bytes                                                      | `[TL]` `[AS]` `[TM]`    |
| 0x14   | 4     | u32  | `metadata_word_count`           | Anzahl 4-Byte-Wörter in **Index2** (= Metadata-Tabelle), unkomprimiert                              | `[AS]` `[TM]`            |
| 0x18   | 4     | u32  | `metadata_table_compressed_size`| Komprimierte Größe der Metadata-Tabelle in Bytes                                                   | `[TL]` `[AS]` `[TM]`    |
| 0x1C   | **8** | **u64** | `entry_table_start`             | **Absolute** Datei-Position der (komprimierten) Entry-Tabelle                                      | `[TL]` `[AS]` `[TM]`    |
| 0x24   | **8** | **u64** | `metadata_table_start`          | **Absolute** Datei-Position der (komprimierten) Metadata-Tabelle                                   | `[TL]` `[AS]` `[TM]`    |
| 0x2C   | **8** | **u64** | `security_descriptor_start`     | Reserviert / Cert-Offset, in der Praxis `0`                                                        | `[AS]` `[TM]`            |
| 0x34   | 1     | u8   | `platform`                      | Plattform-Marker (optional; nicht alle Archive haben das Feld nach `cert_start`)                   | `[TL]` `[TM]`            |

**Kritischer Unterschied zu v1:** die drei **Tabellen-Start-Offsets sind `u64`**
(je 8 Byte), nicht `u32`. — `[TL]` `[AS]` `[TM]`

Beide Tabellen (Index1 und Index2) sind **zlib-komprimiert** auf Disk und müssen
vor der Auswertung dekomprimiert werden. — `[TL]` `[AS]`

> **Größen-Check:** Index1 nach Inflate sollte exakt `entry_count * 16` Byte
> ergeben. Index2 nach Inflate sollte exakt `metadata_word_count * 4` Byte
> ergeben. — `[AS]`

---

## 3. Entry-Layouts

### 3.1 v1-Entry — 32 Byte

| Offset | Größe | Typ | Feld              | Bedeutung                                                                  | Quellen      |
| ------ | ----- | --- | ----------------- | -------------------------------------------------------------------------- | ------------ |
| 0x00   | 8     | u64 | `hash`            | CityHash64 des vollen Pfades                                              | `[TL]` `[AS]` |
| 0x08   | 8     | u64 | `offset`          | Absolute Datei-Position der Daten                                          | `[TL]` `[AS]` |
| 0x10   | 4     | u32 | `flags`           | Bitfeld — Bit 0 = Verzeichnis, Bit 1 = komprimiert                         | `[TL]` `[AS]` |
| 0x14   | 4     | u32 | `crc`             | CRC32 (vom offiziellen Extractor laut `[AS]` nicht validiert)              | `[TL]` `[AS]` |
| 0x18   | 4     | u32 | `size`            | Unkomprimierte Größe                                                       | `[TL]` `[AS]` |
| 0x1C   | 4     | u32 | `compressed_size` | Komprimierte Größe (gleich `size` falls unkomprimiert)                     | `[TL]` `[AS]` |

### 3.2 v2-Entry (Index1) — 16 Byte

| Offset | Größe | Typ | Feld             | Bedeutung                                                                                       | Quellen           |
| ------ | ----- | --- | ---------------- | ----------------------------------------------------------------------------------------------- | ----------------- |
| 0x00   | 8     | u64 | `hash`           | CityHash64 des vollen Pfades                                                                    | `[TL]` `[AS]` `[TM]` |
| 0x08   | 4     | u32 | `metadata_index` | Offset in Index2 (in **4-Byte-Wörtern**), wo die Metadaten dieses Eintrags beginnen             | `[TL]` `[AS]` `[TM]` |
| 0x0C   | 2     | u16 | `metadata_count` | Anzahl der Metadata-Records für diesen Eintrag                                                  | `[TL]` `[AS]` `[TM]` |
| 0x0E   | 2     | u16 | `flags`          | Bit 0 = `is_directory`, Bit 2 = `preload`; obere Bits unbekannt / unbenutzt                     | `[TL]` `[AS]`      |

`[TM]` interpretiert die letzten 2 Byte als zwei separate `u8`
(`flags:Bitfield(isDirectory)` + `someByte`); das ist semantisch äquivalent
und nur eine andere Sicht auf dieselben 16 Bit.

### 3.3 v2-Index2 — zweistufige Indirektion

> **Wichtig — Korrektur gegenüber dem ersten Wurf dieser Doku.**
> Index2 ist **nicht** einfach „eine Sequenz von Records hintereinander". Beim
> empirischen Test gegen `base.scs` (177.929 Entries) hat sich gezeigt: der
> tatsächliche Aufbau ist zweistufig, und nur die Archive-SCS-Source
> (`HashFS2.pm`) zeigt das eindeutig — die WebFetch-Zusammenfassung hatte den
> Indirektions-Schritt verschluckt.

**Stufe 1 — Mini-Header-Run.** `entry.metadata_index × 4` ist der Byte-Offset
in Index2 zum Beginn einer Sequenz von **`metadata_count` × 4-Byte
Mini-Headern**: — `[AS]` (HashFS2.pm: `unpack '(SCC)<', substr $index2, ($index_offset+$k)*4, 4`)

| Offset | Größe | Feld         | Bedeutung                                              |
| ------ | ----- | ------------ | ------------------------------------------------------ |
| 0x00   | 2     | `offset_lo`  | Untere 16 Bit des Body-Offsets                          |
| 0x02   | 1     | `offset_hi`  | Obere 8 Bit des Body-Offsets                            |
| 0x03   | 1     | `kind`       | Record-Typ (siehe Mapping)                              |

Der Body-Offset ist 24 Bit, in **4-Byte-Wort-Einheiten**, relativ zum Start von
Index2. Body-Position in Bytes = `body_offset × 4`.

**Stufe 2 — Body.** An der Body-Position liegt das eigentliche Record. Die
Body-Länge richtet sich nach `kind`: — `[AS]`

| `kind`-Byte         | Body-Länge | Bezeichnung                                       |
| ------------------- | ---------- | ------------------------------------------------- |
| `0x01`              | 8          | (Hilfs-Record, z. B. TObj/Texture)                |
| `0x02`              | 4          | (Hilfs-Record)                                    |
| `0x05`              | 32         | PackedTobjDdsMetadata (1.55+, `.pma`-Files)       |
| `0x06`              | 8          | (Hilfs-Record, `.pmg`-Files 1.55+)                |
| `& 0x80 != 0`       | 16         | **Data-Part** — siehe §3.4                        |

Der **Data-Part-Marker ist Bit 7** des Kind-Bytes. `flags2` (oberes Nibble von
Byte 0x03 des Body, **nicht** des Mini-Headers) trägt die Kompressionsmethode;
TruckLib kodiert `0x10` für zlib. In `base.scs` 1.55 wurden empirisch nur Werte
mit Bit 7 gesetzt (`0x80`, `0x81`, `0x90`, …) als Data-Parts beobachtet.

**Beispiel** (root-Eintrag von ETS2 1.55 `base.scs`, `metadata_index = 15`):

```text
Mini-Header @ Byte 60 (= word 15):     10 00 00 81
  → offset_lo = 0x0010, offset_hi = 0x00, kind = 0x81 (Data-Part)
  → body @ word 16 = byte 64
Data-Part Body @ Byte 64..80:           98 00 00 10  C7 00 00 00  00 00 00 00  E0 01 00 00
  → zsize  = 0x000098 = 152
  → flags2 = 0x10 (zlib)
  → usize  = 0x0000C7 = 199
  → flags3 = 0x00
  → unknown7 = 0x00000000
  → data_offset = 0x000001E0 = 480 → abs = 480 × 16 = 7680
```

Ein einzelner Entry referenziert in der Regel **mehrere** Mini-Header
(`metadata_count > 1`), aber nur einer zeigt auf einen Data-Part. Texturen
hängen `0x05`/`0x01`/`0x06`-Records an.

### 3.4 v2-MainMetadata (Data-Part Body, 16 Byte)

Das echte Layout, verifiziert gegen `[AS]` (`HashFS2.pm`,
`unpack '(SCC SCC LL)<'`) und `[TL]` (`MainMetadata.cs:Deserialize`):

| Offset | Größe | Typ | Feld           | Bedeutung                                                                |
| ------ | ----- | --- | -------------- | ------------------------------------------------------------------------ |
| 0x00   | 2     | u16 | `zsize_lo`     | untere 16 Bit der komprimierten Größe                                    |
| 0x02   | 1     | u8  | `zsize_hi`     | obere 8 Bit der komprimierten Größe (zsize ist 24 Bit total)             |
| 0x03   | 1     | u8  | `flags2`       | obere 4 Bit = Kompressionsmethode (`0x10` = zlib, `0x00` = none)         |
| 0x04   | 2     | u16 | `usize_lo`     | untere 16 Bit der unkomprimierten Größe                                  |
| 0x06   | 1     | u8  | `usize_hi`     | obere 8 Bit der unkomprimierten Größe (usize ist 24 Bit total)           |
| 0x07   | 1     | u8  | `flags3`       | aktuell unbenutzt                                                        |
| 0x08   | 4     | u32 | `unknown7`     | Bedeutung in keiner Quelle dokumentiert                                  |
| 0x0C   | 4     | u32 | `data_offset`  | Datei-Offset in **16-Byte-Blöcken** — abs Bytes = `data_offset × 16`     |

> **Achtung — die ursprüngliche TL-Beschreibung „CompressedSize 28 bits + 4 bits
> flags1" beschreibt zwar dasselbe Bit-Muster, gibt aber eine andere
> Lese-Reihenfolge vor. `Deserialize` in `MainMetadata.cs` macht es identisch
> zur AS-Source: 3 Byte zsize + 1 Byte (4 Bit MSB von zsize ∪ 4 Bit flags1).
> Der hier abgebildete (24+8)-Layout ist die einfachere äquivalente Sicht und
> ist es, was empirisch in `base.scs` durch den Inflate kommt.**

---

## 4. Kompression

| Ort                                | Verfahren                                                              | Quellen           |
| ---------------------------------- | ---------------------------------------------------------------------- | ----------------- |
| v1 Datei-Daten (wenn `flags & 2`)  | **zlib / deflate**                                                     | `[TL]` `[AS]`     |
| v1 Entry-Tabelle                   | **unkomprimiert** (flach im Archiv)                                    | `[AS]`            |
| v2 Index1 (Entry-Tabelle)          | **zlib**                                                               | `[TL]` `[AS]`     |
| v2 Index2 (Metadata-Tabelle)       | **zlib**                                                               | `[TL]` `[AS]`     |
| v2 Daten-Part `flags2 & 0xF0`      | `0x00` = **none**; `0x10` = **zlib**                                   | `[AS]` `[TM]`     |
| v2 Texture-Daten (DDS / Mip-Tail)  | **GDEFLATE** (NVIDIA tile compression, 64 KiB Tiles) — fakultativ      | `[TM]`            |
| v2 (deklariert, aber unsupported)  | `ZLIB_HEADERLESS`, `ZSTD`                                              | `[TM]`            |

**Wichtig:** zstd taucht nur in `[TM]` als _„declared but unsupported"_ auf —
**keine** Quelle bestätigt, dass v2-Archive zstd tatsächlich verwenden. Für unsere
erste Iteration reicht zlib + GDEFLATE (letzteres nur falls wir Texturen brauchen).

---

## 5. Hash-Verfahren

- **Algorithmus:** Standard-CityHash64 (Google CityHash 1.0.3, MIT). — `[AS]`
  (`inc/city.cc` ist eine Kopie der Original-CityHash-Implementierung)
- **Eingabe:** Roher Byte-Inhalt des vollständigen logischen Pfades, z. B.
  `def/world/road.sii`. Pfade benutzen `/` als Separator. — `[TL]` (`HashFsConsts.Separator = '/'`)
- **Salt:** Header-Feld existiert, ist aber in allen aktuellen Archiven `0`.
  Bei `salt == 0` wird die Hash-Eingabe **nicht** modifiziert — der
  CityHash-Aufruf entspricht direkt dem Pfad. Werte ≠ 0 sind in keiner der
  Quellen unterstützt. — `[AS]` (HashFS.pm: _"Non-zero salt is unsupported"_)
- **Verzeichnisse:** sind reguläre Einträge (Bit 0 von `flags`); ihre „Daten"
  sind eine Auflistung der enthaltenen Items (siehe §7).

> **Implikation für unseren Parser:** wir brauchen weder ein Salt-Mixing noch
> eine Pfad-Normalisierung über `/` hinaus. Pfade müssen aber **ohne führenden
> Slash** gehasht werden — Archive-SCS und TruckLib stimmen darin überein,
> dass Verzeichnis-Listings Sub-Verzeichnisse mit `/` _präfixen_, der gehashte
> Eintrags-Pfad selbst aber relativ ist.

---

## 6. Verzeichnis-Listings

Verzeichnis-Einträge speichern in ihren Daten ein Listing der enthaltenen
Files / Subdirs.

### 6.1 v1-Verzeichnis-Daten

Newline-getrennte UTF-8-Liste. Subdirs sind mit `*` präfixiert, Files stehen
ohne Präfix da. — `[AS]`

### 6.2 v2-Verzeichnis-Daten

Custom-Format: — `[AS]`

| Offset | Größe | Feld         | Bedeutung                              |
| ------ | ----- | ------------ | -------------------------------------- |
| 0x00   | 4     | `item_count` | u32 LE                                 |
| 0x04   | N     | `sizes[]`    | je 1 Byte pro Item (Länge des Strings) |
| 0x04+N | var.  | `items[]`    | konkatenierte Strings                  |

Items mit führendem `/` sind Subdirs, alles andere sind Files.

---

## 7. v1 ↔ v2 — Diff in Stichworten

| Aspekt                              | HashFS v1                                  | HashFS v2                                                                         |
| ----------------------------------- | ------------------------------------------ | --------------------------------------------------------------------------------- |
| Header-Größe                        | 20 Byte                                    | 52 (53) Byte                                                                      |
| Tabellen-Start-Offsets              | `u32`, **eines** (`start_offset`)          | **`u64`**, **drei** (Index1, Index2, Cert)                                        |
| Entry-Größe                         | 32 Byte (alle Infos im Entry)              | 16 Byte (+ Metadata in Index2)                                                    |
| Entry-Tabelle komprimiert?          | nein                                       | **ja, zlib**                                                                      |
| Metadata-Tabelle                    | existiert nicht                            | Index2, **zlib**, variabel lange Records                                          |
| Daten-Offset                        | direkt im Entry (absolute Bytes)           | indirekt: `offset_block × 16`                                                     |
| Kompression Daten                   | zlib (oder unkomprimiert)                  | zlib oder unkomprimiert; Texturen ggf. GDEFLATE                                   |
| Verzeichnis-Listing                 | Newline + `*`-Prefix                       | Count + Length-Array + Strings                                                    |
| CRC32 pro Entry                     | ja (vorhanden, oft ignoriert)              | nein                                                                              |
| Spielversionen                      | bis ETS2 1.49                              | ab ETS2 1.50                                                                      |

ETS2/ATS-Versions-Zuordnung gemäß `[AS]`-README.

---

## 8. Aktuelle Rust-Implementierung vs. Soll (post-fix)

Bug-Symptom war **Decompression-Failure beim Lesen von `base.scs`**. Verifiziert gefixt
(2026-05-09): `cargo test -p truckpilot-map-parser --test real_archive_test` läuft
auf einer realen ETS2-1.55 `base.scs` (9.9 GB) durch — 177.929 Index1-Entries werden
erkannt, 177.922 indexiert, das Root-Verzeichnis (Hash = K2) entpackt sich von
152 → 199 Byte.

Es waren tatsächlich **drei** unabhängige Bugs (nicht der eine, den die erste
Doku-Version vermutete):

| #   | Bereich                | War falsch …                                                                                                            | Korrigiert auf …                                                                                                                                                                                | Erkannt durch         |
| --- | ---------------------- | ----------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------- |
| 1   | Header (v2)            | `entry_table_start` / `metadata_table_start` als `u32` ab Offset 0x18/0x1C gelesen.                                    | `u64` @ 0x1C bzw. @ 0x24; @ 0x14 ist `metadata_word_count` (Wörter, nicht Bytes); @ 0x18 ist die komprimierte Größe Index2.                                                                     | Quellen-Read (`[AS]` `[TL]` `[TM]`) |
| 2   | Data-Part-Marker       | Match auf exakten Byte-Wert `0x80`.                                                                                     | Test auf Bit 7: `kind_byte & 0x80 != 0`. Komprimierte Data-Parts haben `0x90`/`0x91`/etc., **nicht** `0x80`.                                                                                    | `cargo test` Synthetik-Test          |
| 3   | Index2-Indirektion     | `metadata_index*4` zeigt direkt auf einen 16-Byte MainMetadata-Record.                                                  | `metadata_index*4` zeigt auf eine **Sequenz von `metadata_count × 4-Byte Mini-Headern`**. Jeder Mini-Header hat (offset 24 bit, kind 8 bit); der 24-Bit-Offset (in 4-Byte-Wörtern) zeigt auf den eigentlichen Body. | echtes `base.scs` (177 K Entries, 60 % Resolve-Failure-Rate) |

Bug #3 war der eigentliche Show-Stopper. Nach Fix #1 + #2 lief das
Tabellen-Inflate bereits durch (72.292 Entries indexiert), aber 105.628 Entries
fielen lautlos durch das Resolve-Sieb. Erst der Hex-Dump des K2-Records gegen
die Archive-SCS-Source `unpack '(SCC)<', $index2, ($index_offset+$k)*4, 4`
machte die Indirektion sichtbar.

### Status der Code-Änderungen

| Datei                                              | Status                                                                                                |
| -------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| `crates/map-parser/src/hashfs.rs` (lebender Code) | Header v2 + 2-stufiger Index2 + 24+8-Bit MainMetadata. Verifiziert gegen `base.scs`.                 |
| `src/ets2_parser/scs_reader.rs` (Legacy)          | Header v2 + Bit-7-Marker. **Index2-Indirektion noch ausstehend** (Code wird derzeit nicht gebaut).   |
| `crates/map-parser/tests/real_archive_test.rs`    | `TRUCKPILOT_BASE_SCS`-Env-Var, skippt sauber wenn unset.                                              |
| `tests/real_map_test.rs` (Legacy)                 | Pfad portabel via Env-Var, lebt weiter ohne aktiven Build.                                            |

---

## 9. Offene Punkte / noch zu klären

- **CityHash64-Bit-Identität** — empirisch bestätigt: alle 177.929 Index1-Hashes
  aus `base.scs` matchen unsere Implementation, das Root-Verzeichnis (= K2)
  resolvt. Test-Vektoren in `crates/map-parser/src/cityhash.rs` decken zusätzlich
  bekannte Werte ab. **Status: erledigt.**
- **Mehrere Data-Parts pro Entry** — Texturen können MIP_0 + MIP_TAIL haben
  (vgl. `[TM]`). Unser `resolve_data_part` gibt den **ersten** Data-Part zurück.
  Für Sektor- und SII-Files reicht das (jeweils ein Data-Part), für Texturen
  brauchen wir später eine API, die alle Parts ausgibt. TODO im Code markiert.
- **GDEFLATE-Pfad** — fehlt noch komplett für Textur-Loading. Native Binding bei
  `[TM]`, in Rust bisher nicht etabliert. Nicht für `base.scs`-Funktionalität nötig.
- **`platform`-Byte @ 0x34** — präzise Werte über `[TL]` `Platform.cs` müssten
  noch verifiziert werden. Aktuell ignoriert.
- **`flags1` (oberes Nibble Byte 0x03 des MainMetadata-Body)** — über die
  Compression-Method hinaus haben wir bisher keine semantische Bedeutung
  identifiziert.
- **Map-docs Wiki `[MD]`** — relevant erst, wenn wir die _Inhalte_ der
  extrahierten `.base`/`.aux`/`.data`-Dateien parsen.
- **Performance** — `HashFsArchive::open` berechnet derzeit eine SHA-256 über
  die volle Archiv-Datei für Cache-Keying (~15 s in Release auf 9.9 GB,
  ~130 s in Debug). Für interaktive Workflows könnte das lazy oder nur über
  einen Streaming-Sample berechnet werden.
- **Legacy-Code in `src/ets2_parser/scs_reader.rs`** — die Datei ist nicht im
  Cargo-Workspace, wird also nicht gebaut. Header- und Marker-Bit-Fix sind
  drin, der 2-stufige Index2-Walk fehlt aber noch. Wenn sie tatsächlich nirgends
  genutzt wird, sollte sie gelöscht werden, statt zu divergieren.
