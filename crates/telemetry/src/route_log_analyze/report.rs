//! Offline analysis report for route research prep.

use super::classify::{ClassifiedCandidate, classify_candidate};
use super::parse::{ParsedLog, parse_log};

/// Full offline analysis of a sidecar log.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AnalysisReport {
    pub game_ctrl: Option<String>,
    pub gps_slot_addr: Option<String>,
    pub gps_slot_value: Option<String>,
    pub candidates: Vec<ClassifiedCandidate>,
    pub route_like_candidates: u32,
    pub recommended_next_step: String,
    pub module_scan_success_count: u32,
    pub status_lines: Vec<String>,
}

pub fn analyze_log_text(text: &str) -> AnalysisReport {
    let parsed = parse_log(text);
    build_report(parsed)
}

pub fn build_report(parsed: ParsedLog) -> AnalysisReport {
    let mut candidates = Vec::new();

    for &source in &parsed.candidate_sources {
        let values = parsed
            .candidate_table_values
            .get(&source)
            .cloned()
            .unwrap_or_default();
        let log_nonzero = parsed.candidate_done_nonzero.get(&source).copied();
        let hints = parsed
            .candidate_hints
            .get(&source)
            .map(String::as_str)
            .unwrap_or("");
        candidates.push(classify_candidate(source, &values, log_nonzero, hints));
    }

    let route_like_candidates = candidates.iter().filter(|c| c.route_like).count() as u32;
    let recommended_next_step = if route_like_candidates == 0 {
        "do_not_deeper_deref; find new 1.60 signature/root offline".into()
    } else {
        "review route_like candidates offline before any live derefs".into()
    };

    AnalysisReport {
        game_ctrl: parsed.game_ctrl,
        gps_slot_addr: parsed.gps_slot_addr,
        gps_slot_value: parsed.gps_slot_value,
        candidates,
        route_like_candidates,
        recommended_next_step,
        module_scan_success_count: parsed.module_scan_success_count,
        status_lines: parsed.status_lines,
    }
}

pub fn render_report(report: &AnalysisReport) -> String {
    let mut out = String::new();
    if let Some(ref gc) = report.game_ctrl {
        out.push_str(&format!("game_ctrl={gc}\n"));
    }
    if let Some(ref addr) = report.gps_slot_addr {
        out.push_str(&format!("gps_slot_addr={addr}\n"));
    }
    if let Some(ref val) = report.gps_slot_value {
        out.push_str(&format!("gps_slot_value={val}\n"));
    }
    for c in &report.candidates {
        out.push_str(&format!(
            "candidate +0x{:X}: {}, nonzero={}, route_like={}\n",
            c.offset,
            c.category.as_str(),
            c.nonzero,
            c.route_like
        ));
    }
    out.push_str(&format!(
        "summary:\nroute_like_candidates={}\nrecommended_next_step={}\n",
        report.route_like_candidates, report.recommended_next_step
    ));
    out
}
