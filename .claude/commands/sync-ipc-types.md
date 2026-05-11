Lies crates/ipc-protocol/src/lib.rs und synchronisiere die TypeScript-Typen in
crates/ui/src/lib/types.ts. Achte besonders auf Enum-Varianten (Rust-Enum mit
Daten → TypeScript Discriminated Union mit `type` als Diskriminator). Falls neue
Message-Typen existieren, ergänze entsprechende Handler in src/lib/ipc.ts.
