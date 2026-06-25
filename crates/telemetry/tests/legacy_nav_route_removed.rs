//! Phase 5h: legacy `Local\TruckPilotNavRoute` reader must not reappear.

#[test]
fn nav_route_source_has_no_legacy_reader_symbols() {
    let src = include_str!("../src/nav_route.rs");
    for forbidden in [
        "NavRouteReader",
        "NavRouteShmLayout",
        "NavRouteSnapshot",
        "NAV_ROUTE_SHM_MAGIC",
        "peek_diag_code",
    ] {
        assert!(
            !src.contains(forbidden),
            "legacy NavRoute symbol `{forbidden}` must stay removed (Phase 5h)"
        );
    }
}
