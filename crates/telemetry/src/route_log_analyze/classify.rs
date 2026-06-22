//! Offline candidate slot classification heuristics.

/// High-level category for a `game_ctrl` candidate table region.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CandidateCategory {
    NullTable,
    AsciiTextBlob,
    AssetOrInventoryText,
    SelfRefContainer,
    PointerTable,
    NumericStruct,
    Unknown,
}

impl CandidateCategory {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::NullTable => "null_table",
            Self::AsciiTextBlob => "ascii_text_blob",
            Self::AssetOrInventoryText => "asset_or_inventory_text",
            Self::SelfRefContainer => "self_ref_container",
            Self::PointerTable => "pointer_table",
            Self::NumericStruct => "numeric_struct",
            Self::Unknown => "unknown",
        }
    }
}

/// Classification result for one candidate offset.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClassifiedCandidate {
    pub offset: usize,
    pub category: CandidateCategory,
    pub nonzero: u32,
    pub route_like: bool,
}

pub fn classify_candidate_values(offset: usize, values: &[u64]) -> ClassifiedCandidate {
    classify_candidate(offset, values, None, "")
}

/// Classify one candidate using optional log metadata (`nonzero=` line, `contains` hints).
pub fn classify_candidate(
    offset: usize,
    values: &[u64],
    log_nonzero: Option<u32>,
    hints: &str,
) -> ClassifiedCandidate {
    let nonzero = log_nonzero.unwrap_or_else(|| {
        values.iter().filter(|&&v| v != 0).count() as u32
    });
    if nonzero == 0 {
        return ClassifiedCandidate {
            offset,
            category: CandidateCategory::NullTable,
            nonzero,
            route_like: false,
        };
    }

    let ascii_hits = values
        .iter()
        .filter(|&&v| v != 0 && looks_like_ascii_qword(v))
        .count();
    let joined = format!("{}{}", values_ascii_join(values), hints.to_ascii_lowercase());

    if joined.contains("inventory")
        || joined.contains("cabin")
        || joined.contains("vehicle")
        || joined.contains("accessory")
        || joined.contains("inventory_item_metadata")
        || joined.contains(".sii")
        || joined.contains("def")
        || joined.contains("transmission")
    {
        return ClassifiedCandidate {
            offset,
            category: CandidateCategory::AssetOrInventoryText,
            nonzero,
            route_like: false,
        };
    }

    if ascii_hits >= 2 || (ascii_hits >= 1 && joined.len() >= 6) {
        return ClassifiedCandidate {
            offset,
            category: CandidateCategory::AsciiTextBlob,
            nonzero,
            route_like: false,
        };
    }

    if is_self_ref_container(values) {
        return ClassifiedCandidate {
            offset,
            category: CandidateCategory::SelfRefContainer,
            nonzero,
            route_like: false,
        };
    }

    let pointerish = values
        .iter()
        .filter(|&&v| v != 0 && plausible_heap_pointer(v))
        .count();
    if pointerish >= values.len() / 2 && pointerish >= 2 {
        return ClassifiedCandidate {
            offset,
            category: CandidateCategory::PointerTable,
            nonzero,
            route_like: false,
        };
    }

    ClassifiedCandidate {
        offset,
        category: CandidateCategory::NumericStruct,
        nonzero,
        route_like: false,
    }
}

fn looks_like_ascii_qword(v: u64) -> bool {
    let bytes = v.to_le_bytes();
    bytes.iter().filter(|&&b| (0x20..=0x7E).contains(&b)).count() >= 4
}

fn values_ascii_join(values: &[u64]) -> String {
    let mut s = String::new();
    for &v in values {
        for b in v.to_le_bytes() {
            if (0x20..=0x7E).contains(&b) {
                s.push(b as char);
            }
        }
    }
    s.to_ascii_lowercase()
}

fn is_self_ref_container(values: &[u64]) -> bool {
    if values.is_empty() {
        return false;
    }
    let base = values[0];
    if base == 0 {
        return false;
    }
    values
        .iter()
        .filter(|&&v| v != 0 && (v == base || v == base.saturating_add(8)))
        .count() >= 2
}

fn plausible_heap_pointer(v: u64) -> bool {
    (0x1_0000..0x0007_FFFF_FFFF_FFFF).contains(&v)
}
