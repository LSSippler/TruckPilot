//! Core daemon readiness flags (blackboard contract).
//!
//! Published by `truckpilot-core` during daemon startup — **not** in the telemetry DLL.
//! `truckpilot-status` reads SHM only; without a separate daemon blackboard connection
//! all fields stay `null` / `available: false`.

/// Blackboard key: router graph built.
pub const GRAPH_READY: &str = "graph_ready";
/// Blackboard key: spline index built.
pub const SPLINE_INDEX_READY: &str = "spline_index_ready";
/// Blackboard key: all plugins finished loading.
pub const PLUGINS_READY: &str = "plugins_ready";
/// Blackboard key: lane-detection plugin load readiness.
pub const LANE_DETECTION_READY: &str = "lane_detection_ready";
/// Blackboard key: aggregate subsystem startup gate (not engage authorization).
pub const SYSTEM_READY: &str = "truckpilot_system_ready";

/// All readiness keys in display order.
pub const READINESS_KEYS: [&str; 5] = [
    GRAPH_READY,
    SPLINE_INDEX_READY,
    PLUGINS_READY,
    LANE_DETECTION_READY,
    SYSTEM_READY,
];

/// Parse a blackboard string into a tri-state bool (`true` / `false` / unknown).
pub fn parse_bb_bool(raw: Option<&str>) -> Option<bool> {
    match raw {
        Some("true") => Some(true),
        Some("false") => Some(false),
        _ => None,
    }
}

/// Core subsystem readiness snapshot for status / overlay JSON.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
pub struct CoreReadiness {
    /// Router graph built and loaded.
    pub graph_ready: Option<bool>,
    /// Spline index built.
    pub spline_index_ready: Option<bool>,
    /// All plugins finished `load_all`.
    pub plugins_ready: Option<bool>,
    /// lane-detection plugin load readiness (`lane.diag.load_ok`), not live lane quality.
    pub lane_detection_ready: Option<bool>,
    /// AND of component flags — subsystem startup only, **not** engage authorization.
    pub truckpilot_system_ready: Option<bool>,
    /// `true` when values were read from the core blackboard (daemon connected).
    pub available: bool,
}

impl CoreReadiness {
    /// Default when no daemon blackboard connection exists (CLI SHM-only path).
    pub fn unavailable() -> Self {
        Self {
            graph_ready: None,
            spline_index_ready: None,
            plugins_ready: None,
            lane_detection_ready: None,
            truckpilot_system_ready: None,
            available: false,
        }
    }

    /// Build from an optional key→value map (e.g. future daemon WS query).
    pub fn from_values(values: &std::collections::HashMap<String, String>) -> Self {
        let graph_ready = parse_bb_bool(values.get(GRAPH_READY).map(String::as_str));
        let spline_index_ready =
            parse_bb_bool(values.get(SPLINE_INDEX_READY).map(String::as_str));
        let plugins_ready = parse_bb_bool(values.get(PLUGINS_READY).map(String::as_str));
        let lane_detection_ready =
            parse_bb_bool(values.get(LANE_DETECTION_READY).map(String::as_str));
        let truckpilot_system_ready =
            parse_bb_bool(values.get(SYSTEM_READY).map(String::as_str));

        let any_present = READINESS_KEYS
            .iter()
            .any(|k| values.contains_key(*k));

        Self {
            graph_ready,
            spline_index_ready,
            plugins_ready,
            lane_detection_ready,
            truckpilot_system_ready,
            available: any_present,
        }
    }
}

fn tri_label(v: Option<bool>) -> &'static str {
    match v {
        Some(true) => "true",
        Some(false) => "false",
        None => "unknown",
    }
}

/// Human-readable block appended to `truckpilot-status` output.
pub fn format_core_readiness_human(r: &CoreReadiness) -> String {
    format!(
        "Core readiness:\n  \
         graph_ready              {}\n  \
         spline_index_ready       {}\n  \
         plugins_ready            {}\n  \
         lane_detection_ready     {}\n  \
         truckpilot_system_ready  {}\n  \
         note: system_ready is not engage authorization",
        tri_label(r.graph_ready),
        tri_label(r.spline_index_ready),
        tri_label(r.plugins_ready),
        tri_label(r.lane_detection_ready),
        tri_label(r.truckpilot_system_ready),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    #[test]
    fn unavailable_has_null_fields_and_not_available() {
        let r = CoreReadiness::unavailable();
        assert!(!r.available);
        assert!(r.graph_ready.is_none());
        assert!(r.truckpilot_system_ready.is_none());
    }

    #[test]
    fn parse_bb_bool_accepts_true_false_only() {
        assert_eq!(parse_bb_bool(Some("true")), Some(true));
        assert_eq!(parse_bb_bool(Some("false")), Some(false));
        assert_eq!(parse_bb_bool(Some("")), None);
        assert_eq!(parse_bb_bool(None), None);
    }

    #[test]
    fn from_values_marks_available_when_any_key_present() {
        let mut map = HashMap::new();
        map.insert(GRAPH_READY.to_string(), "true".to_string());
        let r = CoreReadiness::from_values(&map);
        assert!(r.available);
        assert_eq!(r.graph_ready, Some(true));
        assert!(r.spline_index_ready.is_none());
    }

    #[test]
    fn human_format_uses_unknown_for_missing() {
        let out = format_core_readiness_human(&CoreReadiness::unavailable());
        assert!(out.contains("graph_ready              unknown"));
        assert!(out.contains("note: system_ready is not engage authorization"));
    }
}
