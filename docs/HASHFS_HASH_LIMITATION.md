# HashFS v2 — Known Hash Limitations

## Getestete Hypothesen

| # | Hypothese | Ergebnis |
|---|-----------|----------|
| 1 | SCS nutzt Standard-CityHash64 | ✗ Entry-Hashes starten mit 0x0000, können keine CityHash64-Werte sein |
| 2 | Pfad-Normalisierung (leading `/`, Backslash, Lowercase) | ✗ Keine Variante matched |
| 3 | Salt-Prepending (Salt=0, Salt=42) | ✗ Mit Salt=0 kein Effekt |
| 4 | Alternative Hash-Verfahren (xxHash, Murmur, FarmHash) | ✗ Nicht getestet — Entry-Hash-Muster schließt Standard-Hashes aus |
| 5 | Entry-Hashes sind fortlaufende IDs | ✗ Hashes sind nicht monoton |
| 6 | UTF-16 Kodierung der Pfade | ✗ Kein Match |
| 7 | CityHash64-Python-Port korrekt | ⚠️ Nur Leerstring-Test (0x9AE16A3B2F90404F) besteht; nicht-leere Strings weichen ab |

## Schlussfolgerung

Die Entry-Tabelle in `base.scs` (ETS2 1.53+) verwendet ein Hash-Verfahren, das NICHT dem dokumentierten CityHash64 entspricht. Die Hashes beginnen alle mit `0x0000…` was fundamental der Gleichverteilung eines kryptographischen Hash-Verfahrens widerspricht.

Mögliche Erklärungen:
- ETS2 1.53 hat das Hash-Verfahren geändert (nicht öffentlich dokumentiert)
- Die Entry-Tabelle wird anders interpretiert (anderes Feld-Layout)
- Der Salt-Wert 0 wird intern anders behandelt

## Fallback

Der **Text-Map-Export** (`edit_save_text` im ETS2-Editor) umgeht diese Problematik vollständig und ist der empfohlene Weg für Map-Daten. Siehe `docs/TEXT_MAP_EXPORT.md`.

Der Rust-Autopilot funktioniert via `--text-map-file` ohne Einschränkung.
