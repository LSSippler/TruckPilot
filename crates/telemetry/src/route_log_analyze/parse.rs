//! Sidecar log line parser.

use std::collections::{BTreeMap, BTreeSet};

/// Parsed fields extracted from a TruckPilot telemetry sidecar log.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ParsedLog {
    pub game_ctrl: Option<String>,
    pub gps_slot_addr: Option<String>,
    pub gps_slot_value: Option<String>,
    pub module_scan_success_count: u32,
    pub candidate_sources: BTreeSet<usize>,
    pub candidate_table_values: BTreeMap<usize, Vec<u64>>,
    pub candidate_done_nonzero: BTreeMap<usize, u32>,
    pub candidate_hints: BTreeMap<usize, String>,
    pub candidate_source_values: BTreeMap<usize, u64>,
    pub table_slot_lines: Vec<String>,
    pub status_lines: Vec<String>,
}

pub fn parse_log(text: &str) -> ParsedLog {
    let mut out = ParsedLog::default();
    let mut current_table: Option<usize> = None;
    for line in text.lines() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        if line.contains("module scan success") {
            out.module_scan_success_count = out.module_scan_success_count.saturating_add(1);
            if let Some(v) = extract_field(line, "game_ctrl=") {
                out.game_ctrl = Some(v);
            }
            if let Some(v) = extract_field(line, "gps_slot_addr=") {
                out.gps_slot_addr = Some(v);
            }
            if let Some(v) = extract_field(line, "gps_slot_value=") {
                out.gps_slot_value = Some(v);
            }
        }
        if let Some(rest) = line.strip_prefix("candidate source game_ctrl+0x") {
            if let Some(hex) = rest.split_whitespace().next() {
                if let Ok(off) = usize::from_str_radix(hex, 16) {
                    out.candidate_sources.insert(off);
                    current_table = Some(off);
                    if let Some(v) = extract_field(line, "value=") {
                        if let Ok(val) = parse_hex_u64(&v) {
                            out.candidate_source_values.insert(off, val);
                        }
                    }
                }
            }
        }
        if let Some(off) = parse_table_source_offset(line) {
            current_table = Some(off);
            out.candidate_sources.insert(off);
        }
        let trimmed = line.trim_start();
        if trimmed.starts_with("+0x") {
            out.table_slot_lines.push(trimmed.to_string());
            if let (Some(src), Some((_, val))) = (current_table, parse_slot(trimmed)) {
                out.candidate_table_values.entry(src).or_default().push(val);
            }
        }
        if line.contains("contains") {
            if let Some(off) = current_table {
                out.candidate_hints
                    .entry(off)
                    .or_default()
                    .push(' ');
                out.candidate_hints.entry(off).or_default().push_str(line);
            }
        }
        if line.contains("status=") || line.contains("_done") || line.contains("_failed") {
            out.status_lines.push(line.to_string());
        }
        if line.contains("candidate table done") {
            if let Some(off) = parse_done_source_offset(line) {
                if let Some(nz) = extract_field(line, "nonzero=") {
                    if let Ok(n) = nz.parse::<u32>() {
                        out.candidate_done_nonzero.insert(off, n);
                    }
                }
            }
            current_table = None;
        }
    }
    out
}

fn parse_table_source_offset(line: &str) -> Option<usize> {
    if let Some(rest) = line.strip_prefix("candidate table source=+0x") {
        let hex = rest.split_whitespace().next()?.trim();
        return usize::from_str_radix(hex, 16).ok();
    }
    if let Some(rest) = line.strip_prefix("candidate table source=game_ctrl+0x") {
        let hex = rest.split_whitespace().next()?.trim();
        return usize::from_str_radix(hex, 16).ok();
    }
    None
}

fn parse_done_source_offset(line: &str) -> Option<usize> {
    if let Some(idx) = line.find("source=+0x") {
        let rest = &line[idx + "source=+0x".len()..];
        let hex = rest.split_whitespace().next()?.trim();
        return usize::from_str_radix(hex, 16).ok();
    }
    if let Some(idx) = line.find("source=game_ctrl+0x") {
        let rest = &line[idx + "source=game_ctrl+0x".len()..];
        let hex = rest.split_whitespace().next()?.trim();
        return usize::from_str_radix(hex, 16).ok();
    }
    None
}

fn parse_slot(line: &str) -> Option<(usize, u64)> {
    let rest = line.strip_prefix("+0x")?;
    let (off_hex, val_part) = rest.split_once('=')?;
    let off = usize::from_str_radix(off_hex.trim(), 16).ok()?;
    let val_str = val_part.trim().trim_start_matches("0x");
    u64::from_str_radix(val_str, 16).ok().map(|v| (off, v))
}

fn parse_hex_u64(raw: &str) -> Result<u64, std::num::ParseIntError> {
    u64::from_str_radix(raw.trim().trim_start_matches("0x"), 16)
}

fn extract_field(line: &str, key: &str) -> Option<String> {
    let start = line.find(key)? + key.len();
    let rest = &line[start..];
    let end = rest
        .find(|c: char| c.is_whitespace() || c == ',' || c == ')')
        .unwrap_or(rest.len());
    Some(rest[..end].to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_per_candidate_done_nonzero() {
        let text = "\
candidate source game_ctrl+0x3AC0 value=0x1
candidate table done source=+0x3AC0 slots=33 nonzero=0
candidate source game_ctrl+0x4038 value=0x2
candidate table done source=+0x4038 slots=33 nonzero=30
";
        let parsed = parse_log(text);
        assert_eq!(parsed.candidate_done_nonzero.get(&0x3AC0), Some(&0));
        assert_eq!(parsed.candidate_done_nonzero.get(&0x4038), Some(&30));
    }

    #[test]
    fn parses_contains_hints_for_current_candidate() {
        let text = "\
candidate source game_ctrl+0x4038 value=0x1
... contains vehicle/def, transmission, .sii ...
candidate table done source=+0x4038 slots=33 nonzero=30
";
        let parsed = parse_log(text);
        let hints = parsed.candidate_hints.get(&0x4038).map(String::as_str).unwrap_or("");
        assert!(hints.contains("vehicle/def"));
    }
}
