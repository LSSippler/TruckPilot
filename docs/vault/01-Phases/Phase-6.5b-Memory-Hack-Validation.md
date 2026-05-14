---
created: 2026-05-14
tags: [phase, truckpilot, reverse-engineering, memory-reader]
---

# Phase 6.5b — Memory-Hack-Validierung mit Cold-Path-Pivot

## Ziel

ETS2-Speicher direkt lesen (Telemetry Source #4), ohne SDK-DLL. Validierung von statischen Ghidra-Offsets durch Live-Debugging in x64dbg + Cheat Engine Speed-Scan als Pivot.

## Tag 1 — Ghidra Static Analysis + x64dbg Live-Validation

### Positive Befunde

| Fund | Detail |
|---|---|
| Pattern 1 Game-Singleton-Setter | Live verifiziert bei `0x7FF6E9539C35` |
| Game-Singleton-Object-Anfang | `0x204F2BEA740` mit valider vtable bestätigt |
| Struct-Layouts | `traffic_ai_vehicle_t` + `vehicle_manager_t` extrahiert |
| Position-Encoding | Entschlüsselt: 17b local | 15b sector |
| Engine-Manager-Singletons | 8 Stück im `.data`-Cluster identifiziert |

### Negative Befunde (ehrlich dokumentiert)

| Hypothese | Ergebnis |
|---|---|
| Vehicle-Manager-Offset `0x150` | **WIDERLEGT** — Pointer `0x00007FFF00000000` (ungültig) |
| Pattern 2 Position-Read | **Cold-Path / Multiplayer-only** — nicht im Single-Player-Hot-Path |
| `[rsi+0x510]`-Adressen (5 Stück) aus Cluster | **Kein Trigger** gefunden |
| x64dbg String-Search auf mangled Symbols | **Kein Treffer** |

## Tag 2 — Cheat Engine Speed-Scan (in Arbeit)

Speed-Scan via Float-Increment/Decrement:

```
384.175 km/h → 100 → 88 → 76 → 8 Kandidaten
```

Nächste Schritte:
- Find-what-writes auf die 8 verbleibenden Kandidaten
- Pointer-Scan für ASLR-stabile Pfade

## Aufwand-Revision

| Schätzung vorher | Schätzung nachher |
|---|---|
| 2–4 Wochen | 1–2 Wochen |

Pivot-Grund: Cheat-Engine-Methode ist schneller als rein-statische Ghidra-Analyse.

## Lessons Learned

- **Ghidra-Offsets sind Hypothesen, nicht Fakten** — Live-Validation in x64dbg ist Pflicht vor jeder Struct-Nutzung
- **Pattern-Adresse ≠ Hot-Path-Garantie** — Pattern 2 war Cold-Path/Multiplayer-only, trotz valider Adresse
- **ASLR-Disziplin** — Pattern 1 jede Session neu evaluieren, keine hardcodierten Adressen
- **Source-Paths in Asserts sind RE-Gold** — `prism::` Engine-Namespace aus Assert-Strings extrahiert
- **Cold-Path-Pivot zahlt sich aus** — Cheat-Engine-Speed-Scan liefert schneller stabile Pointer als statische Analyse

## Verweise

- [[Phase-6-Telemetry]] — Telemetry-Pipeline (bisherige Sources 1–3)
- [[Phase-6.2g-Watchdog]] — Watchdog für Telemetry-Stale (relevant wenn Source 4 ausfällt)
