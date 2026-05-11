#!/bin/bash
# Auto-sync nach Agent-Aktion
cd ~/TruckPilot || exit 1

# Nichts machen wenn keine Änderungen
if git diff --quiet && git diff --cached --quiet && [ -z "$(git ls-files --others --exclude-standard)" ]; then
    exit 0
fi

# Pull vor Push (falls jemand parallel was gepusht hat)
git pull --rebase origin geekom-work 2>/dev/null || true

# Alles staging
git add -A

# Commit mit Agent-Tag
AGENT="${1:-agent}"
TIMESTAMP=$(date +"%Y-%m-%d %H:%M:%S")
git commit -m "[$AGENT] auto-sync $TIMESTAMP" 2>/dev/null || exit 0

# Push
git push origin geekom-work 2>/dev/null || true

