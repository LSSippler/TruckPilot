//! Compare two offline analysis snapshots (1.59 vs 1.60).

use super::scan::PatternHit;
use super::strings::{keyword_totals, StringHit};
use std::collections::{BTreeMap, BTreeSet};

/// Summary of one executable analysis.
#[derive(Debug, Clone)]
pub struct AnalysisSnapshot {
    pub label: String,
    pub pattern_hits: Vec<PatternHit>,
    pub string_hits: Vec<StringHit>,
}

/// Diff between old and new analysis runs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DiffReport {
    pub lines: Vec<String>,
}

pub fn diff_snapshots(old: &AnalysisSnapshot, new: &AnalysisSnapshot) -> DiffReport {
    let mut lines = vec![
        "diff mode".into(),
        format!("old={}", old.label),
        format!("new={}", new.label),
        String::new(),
    ];

    let old_patterns = group_pattern_hits(&old.pattern_hits);
    let new_patterns = group_pattern_hits(&new.pattern_hits);
    let all_names: BTreeSet<_> = old_patterns.keys().chain(new_patterns.keys()).cloned().collect();

    lines.push("pattern diff:".into());
    for name in all_names {
        let o = old_patterns.get(&name);
        let n = new_patterns.get(&name);
        match (o, n) {
            (None, Some(nh)) => {
                lines.push(format!("pattern={name} status=only_new hits={}", nh.len()));
            }
            (Some(oh), None) => {
                lines.push(format!("pattern={name} status=only_old hits={}", oh.len()));
            }
            (Some(oh), Some(nh)) => {
                lines.push(format!(
                    "pattern={name} status=both old_hits={} new_hits={}",
                    oh.len(),
                    nh.len()
                ));
                compare_hit_sets(&name, oh, nh, &mut lines);
            }
            (None, None) => {}
        }
    }

    lines.push(String::new());
    lines.push("string keyword counts:".into());
    let old_kw = keyword_totals(&old.string_hits);
    let new_kw = keyword_totals(&new.string_hits);
    let keys: BTreeSet<_> = old_kw
        .iter()
        .map(|(k, _)| k.clone())
        .chain(new_kw.iter().map(|(k, _)| k.clone()))
        .collect();
    let old_map: BTreeMap<_, _> = old_kw.into_iter().collect();
    let new_map: BTreeMap<_, _> = new_kw.into_iter().collect();
    for k in keys {
        let o = old_map.get(&k).copied().unwrap_or(0);
        let n = new_map.get(&k).copied().unwrap_or(0);
        let status = if o == 0 {
            "only_new"
        } else if n == 0 {
            "only_old"
        } else if o == n {
            "unchanged"
        } else {
            "changed"
        };
        lines.push(format!("keyword={k} old={o} new={n} status={status}"));
    }

    DiffReport { lines }
}

fn group_pattern_hits(hits: &[PatternHit]) -> BTreeMap<String, Vec<PatternHit>> {
    let mut map: BTreeMap<String, Vec<PatternHit>> = BTreeMap::new();
    for h in hits {
        map.entry(h.pattern_name.clone()).or_default().push(h.clone());
    }
    map
}

fn compare_hit_sets(name: &str, old: &[PatternHit], new: &[PatternHit], lines: &mut Vec<String>) {
    let old_rvas: BTreeSet<_> = old.iter().map(|h| h.hit_rva).collect();
    let new_rvas: BTreeSet<_> = new.iter().map(|h| h.hit_rva).collect();
    for rva in old_rvas.difference(&new_rvas) {
        lines.push(format!("pattern={name} hit_rva=0x{rva:X} status=removed_in_new"));
    }
    for rva in new_rvas.difference(&old_rvas) {
        lines.push(format!("pattern={name} hit_rva=0x{rva:X} status=added_in_new"));
    }
    for rva in old_rvas.intersection(&new_rvas) {
        let o = old.iter().find(|h| h.hit_rva == *rva).unwrap();
        let n = new.iter().find(|h| h.hit_rva == *rva).unwrap();
        if o.rip_target_rva != n.rip_target_rva {
            lines.push(format!(
                "pattern={name} hit_rva=0x{rva:X} rip_target old=0x{:X} new=0x{:X} status=rip_target_changed",
                o.rip_target_rva.unwrap_or(0),
                n.rip_target_rva.unwrap_or(0)
            ));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ets2_bin_analyze::scan::PatternHit;

    #[test]
    fn diff_reports_missing_pattern_in_new() {
        let old = AnalysisSnapshot {
            label: "old.exe".into(),
            pattern_hits: vec![PatternHit {
                pattern_name: "game_ctrl_load_lea_wc".into(),
                hit_rva: 0x1000,
                context_hex: "aa".into(),
                rip_target_rva: Some(0x33C0548),
                embedded_disp32: None,
            }],
            string_hits: vec![],
        };
        let new = AnalysisSnapshot {
            label: "new.exe".into(),
            pattern_hits: vec![],
            string_hits: vec![],
        };
        let diff = diff_snapshots(&old, &new);
        let text = diff.lines.join("\n");
        assert!(text.contains("only_old"));
        assert!(text.contains("game_ctrl_load_lea_wc"));
    }

    #[test]
    fn diff_reports_rip_target_change() {
        let mk = |rip: usize| PatternHit {
            pattern_name: "game_ctrl_load_lea_wc".into(),
            hit_rva: 0x1000,
            context_hex: "aa".into(),
            rip_target_rva: Some(rip),
            embedded_disp32: None,
        };
        let old = AnalysisSnapshot {
            label: "old".into(),
            pattern_hits: vec![mk(0x33C0548)],
            string_hits: vec![],
        };
        let new = AnalysisSnapshot {
            label: "new".into(),
            pattern_hits: vec![mk(0x354F398)],
            string_hits: vec![],
        };
        let diff = diff_snapshots(&old, &new);
        assert!(diff.lines.iter().any(|l| l.contains("rip_target_changed")));
    }
}
