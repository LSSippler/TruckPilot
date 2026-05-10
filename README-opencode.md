# OpenCode – Multi-Model Setup mit Featherless

## Voraussetzungen

### OpenCode installieren
```bash
curl -fsSL https://opencode.ai/install | bash
```
oder via npm:
```bash
npm install -g opencode-ai
```

### API-Key setzen
```bash
export FEATHERLESS_API_KEY="dein_key_hier"
# Dauerhaft in ~/.bashrc oder ~/.zshrc eintragen
```

### Abhängigkeiten für die Test-Skripte
```bash
sudo apt install jq bc curl
```

---

## Konfiguration

Die Datei `opencode.json` im Projekt-Root definiert alle Modelle.
OpenCode lädt sie automatisch wenn du `opencode` im TruckPilot2-Verzeichnis startest.

---

## Modell-Rollen

### CODER (Default — 80-90% der Arbeit)
| Typ     | Modell                              |
|---------|-------------------------------------|
| Primary | `Qwen/Qwen3-Coder-30B-A3B-Instruct` |
| FB 1    | `Qwen/Qwen3-32B`                    |
| FB 2    | `Qwen/Qwen2.5-Coder-32B-Instruct`   |

### REVIEWER (Code-Reviews, Architektur, Bug-Hunting)
| Typ     | Modell                          |
|---------|---------------------------------|
| Primary | `deepseek-ai/DeepSeek-V3.2`     |
| FB 1    | `deepseek-ai/DeepSeek-V3-0324`  |
| FB 2    | `zai-org/GLM-4.6`               |

### TESTER (Test-Generierung, Edge-Cases)
| Typ     | Modell                              |
|---------|-------------------------------------|
| Primary | `moonshotai/Kimi-K2.6`              |
| FB 1    | `moonshotai/Kimi-K2.5`              |
| FB 2    | `moonshotai/Kimi-K2-Instruct-0905`  |
| FB 3    | `moonshotai/Kimi-K2-Instruct`       |

---

## Modell-Wechsel in OpenCode

OpenCode hat kein natives Slash-Command-System für Rollenwechsel.
Wechsel das aktive Modell über die OpenCode-UI:

| Aktion           | OpenCode-Shortcut       |
|------------------|-------------------------|
| Modell wechseln  | `m` (im Hauptmenü)      |
| Modell-Liste     | `m` → Modelle sind nach Rolle benannt (`[CODER]`, `[REVIEWER]`, `[TESTER]`) |

### Empfohlener Workflow
```
Normales Coding:    [CODER]   Qwen3-Coder-30B  ← Default
Vor Commits:        [REVIEWER] DeepSeek-V3.2
Tests schreiben:    [TESTER]  Kimi-K2.6
```

---

## Fallback-Strategie (manuell)

Featherless kann Modelle in den "Cold"-Zustand versetzen (erster Request dauert 30-60s).
Wenn ein Modell hängt:

1. Prüfe mit `test-fallbacks.sh` welche Modelle warm sind
2. Wechsle in OpenCode auf das nächste Fallback (`[CODER-FB1]`, `[CODER-FB2]`, etc.)
3. Nach 5-10 Minuten: Primary-Modell probieren

**Trigger für manuellen Fallback:**
- Keine Antwort nach 60 Sekunden
- HTTP 503 / 504 / 429
- Antwort enthält "model loading" oder "warming up"

---

## Test-Skripte

### test-fallbacks.sh — Schnell-Check (täglich morgens)
```bash
chmod +x test-fallbacks.sh
./test-fallbacks.sh
```
Gibt eine farbige Tabelle aus: GRÜN = warm, GELB = cold, ROT = Fehler.

### test-models.sh — Vollständiger Report
```bash
chmod +x test-models.sh
./test-models.sh            # einmaliger Lauf
./test-models.sh --watch    # alle 5 Minuten, bis Strg+C
```
Speichert Ergebnis in `test-results.md`.

---

## Fehlerbehebung

```bash
# API-Key prüfen
echo $FEATHERLESS_API_KEY

# Manuell testen ob API erreichbar ist
curl -s https://api.featherless.ai/v1/models \
  -H "Authorization: Bearer $FEATHERLESS_API_KEY" | jq '.data[].id' | head -10

# Exakte Modell-IDs aus der Featherless-Liste holen (Groß/Kleinschreibung!)
curl -s https://api.featherless.ai/v1/models \
  -H "Authorization: Bearer $FEATHERLESS_API_KEY" | jq -r '.data[].id' | grep -i qwen
```

---

## Hinweis für TruckPilot-Entwicklung

- Mutagen-Sync läuft parallel weiter — OpenCode-Sessions nicht vergessen zu beenden
- Bei großem Map-Parser-Code: einzelne Dateien reviewen statt alles auf einmal
- Featherless Premium: 32K Context-Window — bei langen Sessions auf Tokens achten
