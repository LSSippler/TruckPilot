# ETS2 1.50+ Road Format — Analyse und Teilerfolge

## Stand

Das neue ETS2 1.50+ Sektor-Format (von ProMods v2.81 verwendet) speichert
Roads in einem bisher nicht vollständig reverse-engineerten Layout.
Nachfolgend die Erkenntnisse aus der Byte-für-Byte-Analyse eines ProMods-
Sektors.

## Sektor-Layout (ETS2 1.50+)

```
Offset  Größe  Feld
0x00    4      version (u32, typisch 906)
0x04    8      game_id (u64)
0x0C    8      map_version (u64)
0x14    4      node_count (u32)
        28×N   Nodes: type(4) + uid(8) + x(4/f32) + y(4/f32) + z(4/f32) + rot(4/f32)
        ...    Road-Daten (unbekanntes Layout)
        ...    Prefab-Daten
        ...    Tail: node_count(4) + node_count×uid(8) (externe Node-Referenzen)
```

## Node-Format

28 Bytes pro Node:
- `type` (u32): Item-Typ-Code, immer > 48 (z.B. 0x000D381F, 0x95C02ED3)
- `uid` (u64): Eindeutige Node-ID
- `x, y, z` (f32): Koordinaten in ETS2-Weltkoordinaten
- `rot` (f32): Rotation

Das Node-Format wurde vollständig implementiert (2326 Nodes erfolgreich
extrahiert).

## Road-Daten

### Beobachtungen

1. **Kein type/size-Präfix**: Anders als im alten Format (`type(3)+size(44)+payload`)
   haben die Road-Einträge im neuen Format keinen expliziten Typ-Header.

2. **UID-Paare vorhanden**: Im Residual-Datenbereich nach den Header-Nodes
   finden sich 16-Byte-aligned UID-Paare. Diese repräsentieren Roads
   (Start-Node-UID → End-Node-UID).

3. **Keine Attribut-Daten**: Die Road-Attribute (Länge, Geschwindigkeit,
   Spuranzahlen) sind NICHT im selben 16-Byte-Block gespeichert. Sie
   könnten in Parallel-Arrays oder an anderer Stelle liegen.

4. **Referenzen auf externe Nodes**: ~99.999% der Road-UID-Paare referenzieren
   Nodes die in keinem Mod-Sektor-Header definiert sind → diese Nodes
   stammen aus dem Basisspiel (2.3M Nodes, SCS v2 verschlüsselt).

### Implementierter Ansatz

1. **Kandidatenerzeugung**: `extract_road_pairs_from_residual()` scannt die
   Residual-Daten nach 16-Byte-aligned UID-Paaren wo beide UIDs ≥
   0x1000000000000000 sind (ETS2-typischer Wertebereich).

2. **Zwei-Pass-Validierung**: Alle Header-Node-UIDs werden im ersten Durchlauf
   gesammelt. Im zweiten Durchlauf werden nur Road-Kandidaten akzeptiert,
   deren beide UIDs im gesammelten Set enthalten sind.

3. **Ergebnis**: 71 validierte Roads aus 5.969.298 Kandidaten (0.0012%).

### Warum so wenige Roads?

Die Road-Daten referenzieren fast ausschließlich Basisspiel-Nodes (die
verschlüsselt und nicht lesbar sind). Für vollständige Road-Extraktion
müsste das Basisspiel entschlüsselt werden oder ein alternativer Ansatz
gefunden werden.

## Offene Fragen

1. **Road-Attribute**: Liegen Länge/Spuren/Geschwindigkeit in einem
   separaten Array? Nach den UID-Paaren?
2. **Präambel-Daten**: Was bedeuten die 4-Byte-Werte direkt nach dem
   Node-Block (z.B. `5491` in mehreren Sektoren)?
3. **Prefab-Format**: Analog ungeklärt.

## Weiteres Vorgehen

- SCS v2 Entschlüsselung (AES-Key aus ETS2-Binary extrahieren)
- Oder: Nutzung eines externen SCS-Extractors für das Basisspiel
- Alternativ: Road-Attribute aus Node-Positionen berechnen (Luftlinie)

## Relevante Dateien

- `src/ets2_parser/binary_parser.rs` — `parse_new_format_sector`,
  `extract_road_pairs_from_residual`, `looks_like_new_format`
- `src/ets2_parser/mod.rs` — `parse_ets2_map_with_mods` (Zwei-Pass-Logik)
- `promods_parse_results.txt` — Metriken aller Parse-Läufe
