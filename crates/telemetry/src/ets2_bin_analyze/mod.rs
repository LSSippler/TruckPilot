//! Offline PE / AOB analysis for ETS2 route signature research (file-only, no live memory).
#![allow(missing_docs)] // Internal offline research modules; CLI usage documented in `mod` exports.

mod code_heuristics;
mod diff;
mod focus;
mod gps_semantic;
mod patterns;
mod pe;
mod report;
mod scan;
mod strings;
mod xref;

pub use diff::{diff_snapshots, AnalysisSnapshot, DiffReport};
pub use focus::{build_gps_focus_report, render_gps_focus_report, GpsFocusReport};
pub use gps_semantic::{
    live_probe_from_semantic, render_gps_semantic_section, render_live_probe_candidate,
    scan_gps_semantic_patterns, GpsSemanticMatch, LiveProbeCandidate, SingletonRankHint,
};
pub use patterns::{OfflinePattern, OFFLINE_PATTERNS};
pub use pe::{PeFile, PeSection};
pub use report::{render_diff, render_focus_only, render_snapshot, render_snapshot_with_focus};
pub use scan::{PatternHit, rip_resolve_rva, scan_aob, scan_patterns};
pub use strings::{scan_strings, StringHit, STRING_KEYWORDS};
pub use xref::{find_heuristic_xrefs, HeuristicXref};

/// Limits to keep CLI output readable on large executables.
pub struct AnalyzeLimits {
    pub max_pattern_hits_per_pattern: usize,
    pub max_strings_per_keyword: usize,
    pub max_xrefs: usize,
    pub scan_whole_file_strings: bool,
}

impl Default for AnalyzeLimits {
    fn default() -> Self {
        Self {
            max_pattern_hits_per_pattern: 32,
            max_strings_per_keyword: 64,
            max_xrefs: 64,
            scan_whole_file_strings: false,
        }
    }
}

/// Full analysis snapshot including heuristic xrefs.
#[derive(Debug, Clone)]
pub struct FullAnalysis {
    pub pe: PeFile,
    pub snapshot: SnapshotWithXrefs,
}

#[derive(Debug, Clone)]
pub struct SnapshotWithXrefs {
    pub label: String,
    pub pattern_hits: Vec<PatternHit>,
    pub string_hits: Vec<StringHit>,
    pub xrefs: Vec<HeuristicXref>,
}

impl SnapshotWithXrefs {
    pub fn as_diff_snapshot(&self) -> AnalysisSnapshot {
        AnalysisSnapshot {
            label: self.label.clone(),
            pattern_hits: self.pattern_hits.clone(),
            string_hits: self.string_hits.clone(),
        }
    }
}

/// Analyze one PE file from disk.
pub fn analyze_file(path: &std::path::Path, limits: &AnalyzeLimits) -> Result<FullAnalysis, String> {
    let pe = PeFile::load(path)?;
    let label = path.display().to_string();
    let pattern_hits = scan_patterns(&pe, limits.max_pattern_hits_per_pattern);
    let string_hits = scan_strings(
        &pe,
        limits.scan_whole_file_strings,
        limits.max_strings_per_keyword,
    );
    let xrefs = find_heuristic_xrefs(&pe, &string_hits, limits.max_xrefs);
    Ok(FullAnalysis {
        pe,
        snapshot: SnapshotWithXrefs {
            label,
            pattern_hits,
            string_hits,
            xrefs,
        },
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ets2_bin_analyze::pe::build_minimal_pe64;
    use std::path::Path;

    #[test]
    fn end_to_end_synthetic_fixture() {
        let mut text = vec![0x90u8; 48];
        text[8..8 + 7].copy_from_slice(&[0x48, 0x8B, 0x0D, 0x02, 0x00, 0x00, 0x00]);
        let rdata = b"job_route_nav\0";
        let bytes = build_minimal_pe64(&text, rdata);
        let pe = PeFile::from_bytes_for_test(Path::new("fixture.exe"), bytes).unwrap();
        let pattern_hits = scan_patterns(&pe, 8);
        let string_hits = scan_strings(&pe, false, 8);
        assert!(!pattern_hits.is_empty());
        assert!(string_hits.iter().any(|s| s.keyword == "route" || s.keyword == "nav"));
        let rendered = report::render_pe_header(&pe);
        assert!(rendered.contains("image_base="));
        assert!(rendered.contains("section .text"));
    }
}
