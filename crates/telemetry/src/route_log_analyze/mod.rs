//! Offline parser/classifier for TruckPilot telemetry sidecar logs (ETS2 route research prep).

mod classify;
mod parse;
mod report;

pub use classify::{
    CandidateCategory, ClassifiedCandidate, classify_candidate, classify_candidate_values,
};
pub use parse::{ParsedLog, parse_log};
pub use report::{AnalysisReport, analyze_log_text, render_report};

#[cfg(test)]
mod tests {
    use super::*;

    const FIXTURE: &str = r#"
route resolver worker started
route resolver mode=route_candidate_table
module scan success candidate=game_ctrl_load_v159 slot_rva=0x33C0548 game_ctrl=0x1A2B3C4D5E6F gps_slot_addr=0x1A2B3C4E9E6F gps_slot_value=0x0
candidate source game_ctrl+0x3AC0 value=0x0
candidate table source=game_ctrl+0x3AC0
+0x00 = 0x0
+0x08 = 0x0
candidate source game_ctrl+0x4038 value=0x1A2B3C500000
candidate table source=game_ctrl+0x4038
+0x00 = 0x1A2B3C500000
+0x08 = 0x696E76656E746F72
candidate source game_ctrl+0x4228 value=0x1A2B3C510000
candidate table source=game_ctrl+0x4228
+0x00 = 0x1A2B3C510000
+0x08 = 0x1A2B3C510008
candidate table done sources=8 nonzero=12
resolver attempt count=7 status=route_candidate_table_done
"#;

    #[test]
    fn parser_finds_gps_slot_value_zero() {
        let parsed = parse_log(FIXTURE);
        assert_eq!(parsed.gps_slot_value.as_deref(), Some("0x0"));
    }

    #[test]
    fn parser_finds_all_candidate_sources() {
        let parsed = parse_log(FIXTURE);
        assert!(parsed.candidate_sources.contains(&0x3AC0));
        assert!(parsed.candidate_sources.contains(&0x4038));
        assert!(parsed.candidate_sources.contains(&0x4228));
    }

    #[test]
    fn parser_finds_module_scan_and_done_status() {
        let parsed = parse_log(FIXTURE);
        assert_eq!(parsed.module_scan_success_count, 1);
        assert!(parsed
            .status_lines
            .iter()
            .any(|s| s.contains("route_candidate_table_done")));
    }

    #[test]
    fn classifier_marks_null_table_not_route_like() {
        let cat = classify_candidate_values(0x3AC0, &[0, 0, 0, 0]);
        assert_eq!(cat.category, CandidateCategory::NullTable);
        assert!(!cat.route_like);
    }

    #[test]
    fn classifier_marks_ascii_inventory_not_route_like() {
        let inv = u64::from_le_bytes(*b"inventor");
        let cat = classify_candidate_values(0x4038, &[0x1A2B_3C50_0000, inv, 0, 0]);
        assert!(
            cat.category == CandidateCategory::AssetOrInventoryText
                || cat.category == CandidateCategory::AsciiTextBlob
        );
        assert!(!cat.route_like);
    }

    #[test]
    fn classifier_marks_self_ref_container_not_route_like() {
        let base = 0x1A2B_3C51_0000_u64;
        let cat = classify_candidate_values(0x4228, &[base, base + 8, base, 0]);
        assert_eq!(cat.category, CandidateCategory::SelfRefContainer);
        assert!(!cat.route_like);
    }

    #[test]
    fn report_recommends_no_deeper_deref_for_fixture() {
        let report = analyze_log_text(FIXTURE);
        assert_eq!(report.route_like_candidates, 0);
        assert!(report.recommended_next_step.contains("do_not_deeper_deref"));
        let text = render_report(&report);
        assert!(text.contains("gps_slot_value=0x0"));
        assert!(text.contains("route_like=false"));
        assert!(text.contains("route_like_candidates=0"));
    }

    #[test]
    fn ets2_160_excerpt_fixture_integration() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/fixtures/route_candidate_table_ets2_160_excerpt.log"
        );
        let text = std::fs::read_to_string(path).expect("fixture log");
        let report = analyze_log_text(&text);
        let rendered = render_report(&report);

        assert_eq!(report.gps_slot_value.as_deref(), Some("0x0"));
        assert_eq!(report.route_like_candidates, 0);
        assert!(report
            .recommended_next_step
            .contains("do_not_deeper_deref; find new 1.60 signature/root offline"));

        let expectations: &[(usize, &str)] = &[
            (0x3AC0, "null_table"),
            (0x4038, "asset_or_inventory_text"),
            (0x4130, "asset_or_inventory_text"),
            (0x41B0, "asset_or_inventory_text"),
            (0x4228, "self_ref_container"),
            (0x4230, "self_ref_container"),
            (0x4390, "asset_or_inventory_text"),
            (0x4580, "asset_or_inventory_text"),
        ];
        assert_eq!(report.candidates.len(), expectations.len());
        for (offset, category) in expectations {
            let line = format!("candidate +0x{offset:X}: {category},");
            assert!(
                rendered.contains(&line),
                "missing report line for +0x{offset:X}, got:\n{rendered}"
            );
            let candidate = report
                .candidates
                .iter()
                .find(|c| c.offset == *offset)
                .unwrap_or_else(|| panic!("missing candidate +0x{offset:X}"));
            assert!(!candidate.route_like, "+0x{offset:X} must not be route-like");
        }

        assert!(rendered.contains("route_like_candidates=0"));
        assert!(rendered.contains("gps_slot_value=0x0"));
    }
}
