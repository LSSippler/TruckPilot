Führe folgende Checks im crates/ui/-Verzeichnis aus, in dieser Reihenfolge:
1. `npm run typecheck` (tsc --noEmit)
2. `npm run lint`
3. `npm run test`
4. `cargo clippy --manifest-path src-tauri/Cargo.toml -- -D warnings`
5. `cargo fmt --manifest-path src-tauri/Cargo.toml --check`
Bei Fehlern: stoppe und zeige die ersten 3 Fehler. Bei Erfolg: kurze grüne
Zusammenfassung.
