//! ASCII / UTF-16LE string extraction from PE sections.

use super::pe::PeFile;

/// Keywords matched case-insensitively (substring).
pub const STRING_KEYWORDS: &[&str] = &[
    "gps",
    "route",
    "nav",
    "navigation",
    "simple_route",
    "route_task",
    "trip",
    "map",
    "job",
    "waypoint",
];

const MIN_STRING_LEN: usize = 4;

/// One keyword string hit at an RVA.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StringHit {
    pub keyword: String,
    pub rva: usize,
    pub encoding: StringEncoding,
    pub text: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StringEncoding {
    Ascii,
    Utf16Le,
}

/// Scan `.rdata` and optionally the full file for keyword strings.
pub fn scan_strings(pe: &PeFile, include_whole_file: bool, max_per_keyword: usize) -> Vec<StringHit> {
    let mut out = Vec::new();
    let mut regions: Vec<(&str, usize, &[u8])> = Vec::new();
    if let Some(sec) = pe.section(".rdata") {
        if let Some(data) = pe.section_data(".rdata") {
            regions.push((".rdata", sec.rva, data));
        }
    }
    if include_whole_file {
        regions.push(("file", 0, &pe.data));
    }
    for (region, base_rva, data) in regions {
        out.extend(scan_ascii(data, base_rva, region, max_per_keyword));
        out.extend(scan_utf16le(data, base_rva, region, max_per_keyword));
    }
    out.sort_by_key(|h| (h.keyword.clone(), h.rva));
    out
}

fn scan_ascii(data: &[u8], base_rva: usize, region: &str, max_per_keyword: usize) -> Vec<StringHit> {
    let mut out = Vec::new();
    let mut counts = keyword_counts();
    let mut i = 0usize;
    while i < data.len() {
        if !is_printable_ascii(data[i]) {
            i += 1;
            continue;
        }
        let start = i;
        while i < data.len() && is_printable_ascii(data[i]) {
            i += 1;
        }
        let len = i - start;
        if len >= MIN_STRING_LEN {
            let text = String::from_utf8_lossy(&data[start..i]).into_owned();
            let lower = text.to_ascii_lowercase();
            for kw in STRING_KEYWORDS {
                if lower.contains(kw) {
                    let c = counts.entry(kw).or_insert(0);
                    if *c < max_per_keyword {
                        *c += 1;
                        out.push(StringHit {
                            keyword: kw.to_string(),
                            rva: base_rva + start,
                            encoding: StringEncoding::Ascii,
                            text: truncate_text(&text, 120),
                        });
                    }
                }
            }
        }
    }
    let _ = region;
    out
}

fn scan_utf16le(data: &[u8], base_rva: usize, region: &str, max_per_keyword: usize) -> Vec<StringHit> {
    let mut out = Vec::new();
    let mut counts = keyword_counts();
    let mut i = 0usize;
    while i + 1 < data.len() {
        let u = u16::from_le_bytes([data[i], data[i + 1]]);
        if !is_printable_utf16(u) {
            i += 2;
            continue;
        }
        let start = i;
        let mut chars = Vec::new();
        while i + 1 < data.len() {
            let u = u16::from_le_bytes([data[i], data[i + 1]]);
            if u == 0 {
                break;
            }
            if !is_printable_utf16(u) {
                break;
            }
            chars.push(u);
            i += 2;
        }
        if chars.len() >= MIN_STRING_LEN {
            let text: String = chars.iter().filter_map(|&u| char::from_u32(u as u32)).collect();
            let lower = text.to_ascii_lowercase();
            for kw in STRING_KEYWORDS {
                if lower.contains(kw) {
                    let c = counts.entry(kw).or_insert(0);
                    if *c < max_per_keyword {
                        *c += 1;
                        out.push(StringHit {
                            keyword: kw.to_string(),
                            rva: base_rva + start,
                            encoding: StringEncoding::Utf16Le,
                            text: truncate_text(&text, 120),
                        });
                    }
                }
            }
        }
        i += 2;
    }
    let _ = region;
    out
}

fn keyword_counts() -> std::collections::HashMap<&'static str, usize> {
    STRING_KEYWORDS.iter().copied().map(|k| (k, 0usize)).collect()
}

fn is_printable_ascii(b: u8) -> bool {
    (0x20..=0x7E).contains(&b)
}

fn is_printable_utf16(u: u16) -> bool {
    (0x20..=0x7E).contains(&u) || (u >= 0xA0 && u <= 0xFF)
}

fn truncate_text(s: &str, max: usize) -> String {
    if s.len() <= max {
        s.to_string()
    } else {
        format!("{}…", &s[..max])
    }
}

/// Count hits per keyword (for diff summaries).
pub fn keyword_totals(hits: &[StringHit]) -> Vec<(String, u32)> {
    let mut map = std::collections::BTreeMap::new();
    for h in hits {
        *map.entry(h.keyword.clone()).or_insert(0u32) += 1;
    }
    map.into_iter().collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ets2_bin_analyze::pe::{build_minimal_pe64, PeFile};
    use std::path::Path;

    #[test]
    fn finds_ascii_route_keyword_in_rdata() {
        let rdata = b"prefix simple_route_suffix\0";
        let pe_bytes = build_minimal_pe64(&[0x90; 16], rdata);
        let pe = PeFile::from_bytes_for_test(Path::new("t.exe"), pe_bytes).unwrap();
        let hits = scan_strings(&pe, false, 10);
        assert!(hits.iter().any(|h| h.keyword == "route" || h.keyword == "simple_route"));
    }

    #[test]
    fn finds_utf16_keyword() {
        let mut rdata = Vec::new();
        for c in "nav_ui".encode_utf16() {
            rdata.extend_from_slice(&c.to_le_bytes());
        }
        rdata.extend_from_slice(&[0, 0]);
        let pe_bytes = build_minimal_pe64(&[0x90; 8], &rdata);
        let pe = PeFile::from_bytes_for_test(Path::new("t.exe"), pe_bytes).unwrap();
        let hits = scan_strings(&pe, false, 10);
        assert!(hits.iter().any(|h| h.keyword == "nav"));
    }
}
