# Fallback-Wrapper: cargo build --workspace --release + copy plugin DLLs to plugins/
# Bevorzugter Workflow: `cargo build-release` (xtask alias)
cargo build --workspace --release
if ($LASTEXITCODE -eq 0) { cargo xtask copy-plugins }
