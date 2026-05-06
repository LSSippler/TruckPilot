//! ETS2 map sector text parser.
//!
//! Parses the text-based map sector format produced by the ETS2 map editor
//! (`edit_save_text`). This format contains `node {}`, `road {}`, and
//! `prefab {}` blocks with UIDs and position data.
//!
//! The parsed data feeds directly into `MapData` for graph building.

use crate::json_export::{MapNode, MapPrefab, MapRoad};

/// Parsed data for a single map sector.
#[derive(Debug, Clone, Default)]
pub struct SectorData {
    /// Nodes parsed from this text sector.
    pub nodes: Vec<MapNode>,
    /// Roads parsed from this text sector.
    pub roads: Vec<MapRoad>,
    /// Prefabs parsed from this text sector.
    pub prefabs: Vec<MapPrefab>,
}

/// Raw node from the text format (before conversion to `MapNode`).
#[derive(Debug, Clone)]
#[allow(dead_code)]
struct RawNode {
    uid: u64,
    x: f64,
    y: f64,
    z: f64,
    forward_uid: Option<u64>,
    backward_uid: Option<u64>,
}

/// Raw road from the text format.
#[derive(Debug, Clone)]
struct RawRoad {
    uid: u64,
    name: String,
    look_token: String,
    node_uids: Vec<u64>,
    speed_limit: Option<f64>,
    lane_count_forward: u32,
    lane_count_backward: u32,
}

/// Raw prefab from the text format.
#[derive(Debug, Clone)]
struct RawPrefab {
    uid: u64,
    node_uids: Vec<u64>,
}

/// Parse a text-format map sector content into structured data.
///
/// This handles the `edit_save_text`-exported format where blocks are
/// delimited by braces and properties by key-value pairs.
pub fn parse_text_sector(content: &str) -> Result<SectorData, String> {
    let mut raw_nodes: Vec<RawNode> = Vec::new();
    let mut raw_roads: Vec<RawRoad> = Vec::new();
    let mut raw_prefabs: Vec<RawPrefab> = Vec::new();

    let chars: Vec<char> = content.chars().collect();
    let mut i = 0;

    while i < chars.len() {
        skip_whitespace_and_comments(&chars, &mut i);
        if i >= chars.len() {
            break;
        }

        if let Some(block_type) = try_read_word(&chars, &i) {
            i += block_type.len();
            match block_type.as_str() {
                "node" => {
                    if let Some(node) = parse_raw_node(&chars, &mut i)? {
                        raw_nodes.push(node);
                    }
                }
                "road" => {
                    if let Some(road) = parse_raw_road(&chars, &mut i)? {
                        raw_roads.push(road);
                    }
                }
                "prefab" => {
                    if let Some(prefab) = parse_raw_prefab(&chars, &mut i)? {
                        raw_prefabs.push(prefab);
                    }
                }
                _ => {
                    // Skip unknown block.
                    skip_block(&chars, &mut i);
                }
            }
        } else {
            i += 1;
        }
    }

    // Convert to MapData types.
    let mut nodes: Vec<MapNode> = Vec::new();
    for rn in &raw_nodes {
        nodes.push(MapNode {
            uid: rn.uid,
            x: rn.x,
            y: rn.y,
            z: rn.z,
        });
    }

    let mut roads: Vec<MapRoad> = Vec::new();
    for rr in &raw_roads {
        let uid_str = format!("0x{:016X}", rr.uid);
        roads.push(MapRoad {
            uid: uid_str,
            name: rr.name.clone(),
            look_token: rr.look_token.clone(),
            nodes: rr.node_uids.clone(),
            speed_limit: rr.speed_limit,
            lane_count_forward: rr.lane_count_forward,
            lane_count_backward: rr.lane_count_backward,
        });
    }

    let mut prefabs: Vec<MapPrefab> = Vec::new();
    for rp in &raw_prefabs {
        let uid_str = format!("0x{:016X}", rp.uid);
        prefabs.push(MapPrefab {
            uid: uid_str,
            nodes: rp.node_uids.clone(),
        });
    }

    // Sort by UID for determinism.
    nodes.sort_by_key(|n| n.uid);
    roads.sort_by(|a, b| a.uid.cmp(&b.uid));
    prefabs.sort_by(|a, b| a.uid.cmp(&b.uid));

    Ok(SectorData {
        nodes,
        roads,
        prefabs,
    })
}

// ---------------------------------------------------------------------------
// Block parsers
// ---------------------------------------------------------------------------

fn parse_raw_node(chars: &[char], i: &mut usize) -> Result<Option<RawNode>, String> {
    skip_whitespace_and_comments(chars, i);
    if *i >= chars.len() || chars[*i] != '{' {
        return Ok(None);
    }
    *i += 1; // skip {

    let mut uid = 0u64;
    let mut x = 0.0f64;
    let mut y = 0.0f64;
    let mut z = 0.0f64;
    let mut forward_uid = None;
    let mut backward_uid = None;

    while *i < chars.len() {
        skip_whitespace_and_comments(chars, i);
        if *i >= chars.len() || chars[*i] == '}' {
            *i += 1;
            break;
        }

        let key = read_key(chars, i);
        skip_whitespace_and_comments(chars, i);
        if *i < chars.len() && chars[*i] == ':' {
            *i += 1; // skip :
        }
        skip_whitespace_and_comments(chars, i);

        match key.as_str() {
            "uid" => uid = read_hex_or_num(chars, i),
            "position" => {
                let (px, py, pz) = read_vec3(chars, i)?;
                x = px;
                y = py;
                z = pz;
            }
            "forward_item_uid" => forward_uid = Some(read_hex_or_num(chars, i)),
            "backward_item_uid" => backward_uid = Some(read_hex_or_num(chars, i)),
            _ => {
                skip_value(chars, i);
            }
        }

        // Skip comma if present.
        skip_whitespace_and_comments(chars, i);
        if *i < chars.len() && chars[*i] == ',' {
            *i += 1;
        }
    }

    if uid == 0 {
        // Node without UID — skip.
        return Ok(None);
    }

    Ok(Some(RawNode {
        uid,
        x,
        y,
        z,
        forward_uid,
        backward_uid,
    }))
}

fn parse_raw_road(chars: &[char], i: &mut usize) -> Result<Option<RawRoad>, String> {
    skip_whitespace_and_comments(chars, i);
    if *i >= chars.len() || chars[*i] != '{' {
        return Ok(None);
    }
    *i += 1;

    let mut uid = 0u64;
    let mut name = String::new();
    let mut look_token = String::new();
    let mut node_uids: Vec<u64> = Vec::new();
    let mut speed_limit = None;
    let mut lane_count_forward = 0u32;
    let mut lane_count_backward = 0u32;

    while *i < chars.len() {
        skip_whitespace_and_comments(chars, i);
        if *i >= chars.len() || chars[*i] == '}' {
            *i += 1;
            break;
        }

        let key = read_key(chars, i);
        skip_whitespace_and_comments(chars, i);
        if *i < chars.len() && chars[*i] == ':' {
            *i += 1;
        }
        skip_whitespace_and_comments(chars, i);

        match key.as_str() {
            "uid" => uid = read_hex_or_num(chars, i),
            "name" => {
                if let Some(s) = read_quoted_string(chars, i) {
                    name = s;
                }
            }
            "look_token" => {
                if let Some(s) = read_quoted_string(chars, i) {
                    look_token = s;
                }
            }
            "nodes" => {
                node_uids = read_uid_list(chars, i);
            }
            "speed_limit" => {
                speed_limit = Some(read_float_or_int(chars, i));
            }
            "lane_count_forward" => {
                lane_count_forward = read_int(chars, i) as u32;
            }
            "lane_count_backward" => {
                lane_count_backward = read_int(chars, i) as u32;
            }
            _ => {
                skip_value(chars, i);
            }
        }

        skip_whitespace_and_comments(chars, i);
        if *i < chars.len() && chars[*i] == ',' {
            *i += 1;
        }
    }

    if uid == 0 || node_uids.is_empty() {
        return Ok(None);
    }

    Ok(Some(RawRoad {
        uid,
        name,
        look_token,
        node_uids,
        speed_limit,
        lane_count_forward,
        lane_count_backward,
    }))
}

fn parse_raw_prefab(chars: &[char], i: &mut usize) -> Result<Option<RawPrefab>, String> {
    skip_whitespace_and_comments(chars, i);
    if *i >= chars.len() || chars[*i] != '{' {
        return Ok(None);
    }
    *i += 1;

    let mut uid = 0u64;
    let mut node_uids: Vec<u64> = Vec::new();

    while *i < chars.len() {
        skip_whitespace_and_comments(chars, i);
        if *i >= chars.len() || chars[*i] == '}' {
            *i += 1;
            break;
        }

        let key = read_key(chars, i);
        skip_whitespace_and_comments(chars, i);
        if *i < chars.len() && chars[*i] == ':' {
            *i += 1;
        }
        skip_whitespace_and_comments(chars, i);

        match key.as_str() {
            "uid" => uid = read_hex_or_num(chars, i),
            "nodes" => {
                node_uids = read_uid_list(chars, i);
            }
            "node_uids" => {
                node_uids = read_uid_list(chars, i);
            }
            _ => {
                skip_value(chars, i);
            }
        }

        skip_whitespace_and_comments(chars, i);
        if *i < chars.len() && chars[*i] == ',' {
            *i += 1;
        }
    }

    if uid == 0 {
        return Ok(None);
    }

    Ok(Some(RawPrefab { uid, node_uids }))
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn skip_whitespace_and_comments(chars: &[char], i: &mut usize) {
    while *i < chars.len() {
        let c = chars[*i];
        if c.is_whitespace() {
            *i += 1;
        } else if c == '/' && *i + 1 < chars.len() && chars[*i + 1] == '/' {
            while *i < chars.len() && chars[*i] != '\n' {
                *i += 1;
            }
        } else if c == '/' && *i + 1 < chars.len() && chars[*i + 1] == '*' {
            *i += 2;
            while *i + 1 < chars.len() && !(chars[*i] == '*' && chars[*i + 1] == '/') {
                *i += 1;
            }
            if *i + 1 < chars.len() {
                *i += 2;
            }
        } else {
            break;
        }
    }
}

fn try_read_word(chars: &[char], i: &usize) -> Option<String> {
    let mut end = *i;
    while end < chars.len() && (chars[end].is_alphanumeric() || chars[end] == '_') {
        end += 1;
    }
    if end > *i {
        Some(chars[*i..end].iter().collect())
    } else {
        None
    }
}

fn read_key(chars: &[char], i: &mut usize) -> String {
    let start = *i;
    while *i < chars.len() && (chars[*i].is_alphanumeric() || chars[*i] == '_') {
        *i += 1;
    }
    chars[start..*i].iter().collect()
}

fn read_hex_or_num(chars: &[char], i: &mut usize) -> u64 {
    skip_whitespace_and_comments(chars, i);
    let start = *i;
    if *i + 1 < chars.len() && chars[*i] == '0' && (chars[*i + 1] == 'x' || chars[*i + 1] == 'X') {
        *i += 2;
        while *i < chars.len() && chars[*i].is_ascii_hexdigit() {
            *i += 1;
        }
        let hex_str: String = chars[(start + 2)..*i].iter().collect();
        u64::from_str_radix(&hex_str, 16).unwrap_or(0)
    } else if *i < chars.len() && chars[*i].is_ascii_digit() {
        while *i < chars.len() && (chars[*i].is_ascii_digit() || chars[*i] == '.') {
            *i += 1;
        }
        let num_str: String = chars[start..*i].iter().collect();
        num_str.parse::<f64>().ok().map(|f| f as u64).unwrap_or(0)
    } else {
        0
    }
}

fn read_float_or_int(chars: &[char], i: &mut usize) -> f64 {
    skip_whitespace_and_comments(chars, i);
    let start = *i;
    if *i < chars.len() && chars[*i] == '-' {
        *i += 1;
    }
    while *i < chars.len() && (chars[*i].is_ascii_digit() || chars[*i] == '.') {
        *i += 1;
    }
    let num_str: String = chars[start..*i].iter().collect();
    num_str.parse().unwrap_or(0.0)
}

fn read_int(chars: &[char], i: &mut usize) -> i64 {
    read_float_or_int(chars, i) as i64
}

fn read_vec3(chars: &[char], i: &mut usize) -> Result<(f64, f64, f64), String> {
    skip_whitespace_and_comments(chars, i);
    if *i >= chars.len() || chars[*i] != '(' {
        return Ok((0.0, 0.0, 0.0));
    }
    *i += 1;
    let x = read_float_or_int(chars, i);
    skip_whitespace_and_comments(chars, i);
    if *i < chars.len() && chars[*i] == ',' {
        *i += 1;
    }
    let y = read_float_or_int(chars, i);
    skip_whitespace_and_comments(chars, i);
    if *i < chars.len() && chars[*i] == ',' {
        *i += 1;
    }
    let z = read_float_or_int(chars, i);
    skip_whitespace_and_comments(chars, i);
    if *i < chars.len() && chars[*i] == ')' {
        *i += 1;
    }
    Ok((x, y, z))
}

fn read_uid_list(chars: &[char], i: &mut usize) -> Vec<u64> {
    skip_whitespace_and_comments(chars, i);
    let mut uids = Vec::new();
    if *i >= chars.len() || chars[*i] != '(' {
        // Single UID.
        let uid = read_hex_or_num(chars, i);
        if uid != 0 {
            uids.push(uid);
        }
        return uids;
    }
    *i += 1;
    while *i < chars.len() && chars[*i] != ')' {
        skip_whitespace_and_comments(chars, i);
        if *i >= chars.len() || chars[*i] == ')' {
            break;
        }
        let uid = read_hex_or_num(chars, i);
        if uid != 0 {
            uids.push(uid);
        }
        skip_whitespace_and_comments(chars, i);
        if *i < chars.len() && chars[*i] == ',' {
            *i += 1;
        }
    }
    if *i < chars.len() && chars[*i] == ')' {
        *i += 1;
    }
    uids
}

fn read_quoted_string(chars: &[char], i: &mut usize) -> Option<String> {
    skip_whitespace_and_comments(chars, i);
    if *i >= chars.len() || chars[*i] != '"' {
        // Unquoted — include alphanumeric, underscore and dot.
        let start = *i;
        while *i < chars.len()
            && (chars[*i].is_alphanumeric() || chars[*i] == '_' || chars[*i] == '.')
        {
            *i += 1;
        }
        if *i > start {
            return Some(chars[start..*i].iter().collect());
        }
        return None;
    }
    *i += 1;
    let start = *i;
    while *i < chars.len() && chars[*i] != '"' {
        *i += 1;
    }
    let s = chars[start..*i].iter().collect();
    if *i < chars.len() {
        *i += 1;
    }
    Some(s)
}

fn skip_value(chars: &[char], i: &mut usize) {
    skip_whitespace_and_comments(chars, i);
    if *i >= chars.len() {
        return;
    }
    match chars[*i] {
        '"' => {
            read_quoted_string(chars, i);
        }
        '(' => {
            *i += 1;
            let mut depth = 1;
            while *i < chars.len() && depth > 0 {
                match chars[*i] {
                    '(' => depth += 1,
                    ')' => depth -= 1,
                    _ => {}
                }
                *i += 1;
            }
        }
        '{' => {
            *i += 1;
            let mut depth = 1;
            while *i < chars.len() && depth > 0 {
                match chars[*i] {
                    '{' => depth += 1,
                    '}' => depth -= 1,
                    _ => {}
                }
                *i += 1;
            }
        }
        _ => {
            while *i < chars.len()
                && !chars[*i].is_whitespace()
                && chars[*i] != ','
                && chars[*i] != '}'
            {
                *i += 1;
            }
        }
    }
}

fn skip_block(chars: &[char], i: &mut usize) {
    skip_whitespace_and_comments(chars, i);
    if *i < chars.len() && chars[*i] == '{' {
        *i += 1;
        let mut depth = 1;
        while *i < chars.len() && depth > 0 {
            match chars[*i] {
                '{' => depth += 1,
                '}' => depth -= 1,
                _ => {}
            }
            *i += 1;
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_node() {
        let input = r#"
node {
    uid: 0x002935DE00004D04
    position: (21271.468750, -21.031250, 3992.742188)
    forward_item_uid: 0x002935DE6CB04D05
    backward_item_uid: 0x002935DE66504D03
}
"#;
        let sector = parse_text_sector(input).unwrap();
        assert_eq!(sector.nodes.len(), 1);
        assert_eq!(sector.nodes[0].uid, 0x002935DE00004D04);
        assert!((sector.nodes[0].x - 21271.46875).abs() < 0.001);
        assert!((sector.nodes[0].y - (-21.03125)).abs() < 0.001);
        assert!((sector.nodes[0].z - 3992.742188).abs() < 0.001);
    }

    #[test]
    fn test_parse_road() {
        let input = r#"
road {
    uid: 0x002935DE66504D03
    name: "Highway A1"
    look_token: "asphalt"
    nodes: (0x01, 0x02, 0x03)
    speed_limit: 80.0
    lane_count_forward: 2
    lane_count_backward: 1
}
"#;
        let sector = parse_text_sector(input).unwrap();
        assert_eq!(sector.roads.len(), 1);
        assert_eq!(sector.roads[0].uid, "0x002935DE66504D03");
        assert_eq!(sector.roads[0].name, "Highway A1");
        assert_eq!(sector.roads[0].nodes, vec![0x01, 0x02, 0x03]);
        assert_eq!(sector.roads[0].speed_limit, Some(80.0));
        assert_eq!(sector.roads[0].lane_count_forward, 2);
        assert_eq!(sector.roads[0].lane_count_backward, 1);
    }

    #[test]
    fn test_parse_prefab() {
        let input = r#"
prefab {
    uid: 0xABC
    nodes: (0x01, 0x02, 0x03, 0x04)
}
"#;
        let sector = parse_text_sector(input).unwrap();
        assert_eq!(sector.prefabs.len(), 1);
        assert_eq!(sector.prefabs[0].uid, "0x0000000000000ABC");
        assert_eq!(sector.prefabs[0].nodes.len(), 4);
    }

    #[test]
    fn test_parse_multiple_blocks() {
        let input = r#"
node { uid: 0x1 position: (0,0,0) }
node { uid: 0x2 position: (10,0,0) }
road { uid: 0xA name: "" look_token: "a" nodes: (0x1, 0x2) speed_limit: 50.0 }
"#;
        let sector = parse_text_sector(input).unwrap();
        assert_eq!(sector.nodes.len(), 2);
        assert_eq!(sector.roads.len(), 1);
        assert_eq!(sector.prefabs.len(), 0);
    }

    #[test]
    fn test_empty_input() {
        let sector = parse_text_sector("").unwrap();
        assert!(sector.nodes.is_empty());
        assert!(sector.roads.is_empty());
        assert!(sector.prefabs.is_empty());
    }

    #[test]
    fn test_unknown_block_skipped() {
        let input = r#"
unknown_block { some: data }
node { uid: 0x1 position: (0,0,0) }
"#;
        let sector = parse_text_sector(input).unwrap();
        assert_eq!(sector.nodes.len(), 1);
    }

    #[test]
    fn test_whitespace_before_colon() {
        let input = r#"
node {
    uid : 0x1
    position : (10, 20, 30)
}
"#;
        let sector = parse_text_sector(input).unwrap();
        assert_eq!(sector.nodes.len(), 1);
        assert_eq!(sector.nodes[0].uid, 1);
        assert!((sector.nodes[0].x - 10.0).abs() < 0.001);
        assert!((sector.nodes[0].y - 20.0).abs() < 0.001);
        assert!((sector.nodes[0].z - 30.0).abs() < 0.001);
    }
}
