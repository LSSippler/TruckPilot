//! Phase 5h: legacy `Local\TruckPilotNavRoute` must not reappear in the DLL writer.

#[test]
fn nav_route_source_has_no_legacy_shm_symbols() {
    let src = include_str!("../src/nav_route.rs");
    for forbidden in [
        "NAV_ROUTE_SHM_NAME",
        "NavRouteShmLayout",
        "init_legacy_shm",
        "write_route_to_shm",
        "write_diag_to_shm",
    ] {
        assert!(
            !src.contains(forbidden),
            "legacy NavRoute symbol `{forbidden}` must stay removed (Phase 5h)"
        );
    }
}
