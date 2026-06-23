//! Text report rendering for offline PE analysis.

use super::diff::DiffReport;
use super::focus::{build_gps_focus_report, render_gps_focus_report};
use super::scan::PatternHit;
use super::strings::StringHit;
use super::xref::HeuristicXref;
use super::SnapshotWithXrefs;
use super::pe::PeFile;
use std::collections::BTreeMap;

pub fn render_pe_header(pe: &PeFile) -> String {
    let mut out = String::new();
    out.push_str(&format!("file={}\n", pe.path.display()));
    out.push_str(&format!("image_base=0x{:X}\n", pe.image_base));
    for name in [".text", ".rdata", ".data"] {
        if let Some(sec) = pe.section(name) {
            out.push_str(&format!(
                "section {} rva=0x{:X} raw=0x{:X} size=0x{:X} vsize=0x{:X}\n",
                sec.name, sec.rva, sec.raw_offset, sec.raw_size, sec.virtual_size
            ));
        }
    }
    for sec in &pe.sections {
        if !matches!(sec.name.as_str(), ".text" | ".rdata" | ".data") {
            out.push_str(&format!(
                "section {} rva=0x{:X} raw=0x{:X} size=0x{:X} vsize=0x{:X}\n",
                sec.name, sec.rva, sec.raw_offset, sec.raw_size, sec.virtual_size
            ));
        }
    }
    out
}

pub fn render_pattern_hits(hits: &[PatternHit], max_listed: usize) -> String {
    let mut out = String::new();
    let mut grouped: BTreeMap<String, Vec<&PatternHit>> = BTreeMap::new();
    for h in hits {
        grouped.entry(h.pattern_name.clone()).or_default().push(h);
    }
    for (name, group) in grouped {
        out.push_str(&format!("pattern={name} hits={}\n", group.len()));
        for h in group.iter().take(max_listed) {
            out.push_str(&format!("hit rva=0x{:X}\n", h.hit_rva));
            out.push_str(&format!("context bytes={}\n", h.context_hex));
            if let Some(rip) = h.rip_target_rva {
                out.push_str(&format!("rip_target_rva=0x{rip:X}\n"));
            }
            if let Some(disp) = h.embedded_disp32 {
                out.push_str(&format!("embedded_disp32=0x{disp:X}\n"));
            }
        }
        if group.len() > max_listed {
            out.push_str(&format!("... truncated {} more hits\n", group.len() - max_listed));
        }
    }
    out
}

pub fn render_string_hits(hits: &[StringHit], max_listed: usize) -> String {
    let mut out = String::new();
    for h in hits.iter().take(max_listed) {
        out.push_str(&format!(
            "string keyword={} rva=0x{:X} enc={:?} text=\"{}\"\n",
            h.keyword, h.rva, h.encoding, h.text
        ));
    }
    if hits.len() > max_listed {
        out.push_str(&format!("... truncated {} more strings\n", hits.len() - max_listed));
    }
    out
}

pub fn render_xrefs(xrefs: &[HeuristicXref], max_listed: usize) -> String {
    let mut out = String::new();
    for x in xrefs.iter().take(max_listed) {
        out.push_str(&format!(
            "xref heuristic string_rva=0x{:X} text_rva=0x{:X} kind={:?} {}\n",
            x.string_rva, x.text_rva, x.kind, x.detail
        ));
    }
    if xrefs.len() > max_listed {
        out.push_str(&format!("... truncated {} more xrefs\n", xrefs.len() - max_listed));
    }
    out
}

pub fn render_snapshot(snapshot: &SnapshotWithXrefs, pe: &PeFile) -> String {
    render_snapshot_with_focus(snapshot, pe, true)
}

pub fn render_snapshot_with_focus(
    snapshot: &SnapshotWithXrefs,
    pe: &PeFile,
    include_standard: bool,
) -> String {
    let mut out = String::new();
    if include_standard {
        out.push_str(&render_pe_header(pe));
        out.push_str("\n[patterns]\n");
        out.push_str(&render_pattern_hits(&snapshot.pattern_hits, 32));
        out.push_str("\n[strings]\n");
        out.push_str(&render_string_hits(&snapshot.string_hits, 64));
        out.push_str("\n[xrefs heuristic]\n");
        out.push_str(&render_xrefs(&snapshot.xrefs, 32));
        out.push('\n');
    }
    let focus = build_gps_focus_report(pe, snapshot);
    out.push_str(&render_gps_focus_report(&focus));
    out
}

pub fn render_focus_only(snapshot: &SnapshotWithXrefs, pe: &PeFile) -> String {
    render_snapshot_with_focus(snapshot, pe, false)
}

pub fn render_diff(diff: &DiffReport) -> String {
    diff.lines.join("\n")
}
