//! Route distance verification logging and report analysis (Phase 5k).
//!
//! Shared by `route-distance-recorder`, `route-distance-report`, and tests.
//! Report structs intentionally mirror CSV/JSON column names.
#![allow(missing_docs)]

use std::collections::HashMap;
use std::fmt::Write as _;
use std::io::{self, Write};
use std::path::Path;

use serde::{Deserialize, Serialize};

use crate::nav_route::{
    decode_waypoint_flag_names, diagnose_route_coords, diagnose_route_distances,
    RouteDistanceMonotonicStatus, RouteSnapshot,
};

/// One time-series sample row for CSV export.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RouteDistanceSampleRow {
    pub wall_time_ms: u64,
    pub sequence: u32,
    pub route_hash: u64,
    pub valid: bool,
    pub waypoint_count: usize,
    pub distance_count: usize,
    pub distance_untrusted_count: usize,
    pub distance_first_m: Option<f32>,
    pub distance_last_m: Option<f32>,
    pub distance_min_m: Option<f32>,
    pub distance_max_m: Option<f32>,
    pub distance_increase_count: usize,
    pub distance_drop_max_m: f64,
    pub distance_step_avg_m: Option<f64>,
    pub distance_monotonic_status: String,
    pub position_count: usize,
    pub coord_status: String,
    pub first_uid: Option<i64>,
    pub last_uid: Option<i64>,
}

/// Waypoint detail line for optional JSONL export.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct RouteDistanceWaypointRecord {
    pub wall_time_ms: u64,
    pub sequence: u32,
    pub route_hash: u64,
    pub index: usize,
    pub uid: i64,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub distance: f32,
    pub time: f32,
    pub flags: u32,
    pub flag_names: Vec<String>,
}

/// Build a CSV sample row from a route snapshot at `wall_time_ms`.
pub fn snapshot_to_sample_row(wall_time_ms: u64, snap: &RouteSnapshot) -> RouteDistanceSampleRow {
    let dist = diagnose_route_distances(&snap.waypoints);
    let coord = diagnose_route_coords(snap.valid, &snap.waypoints);
    let first_uid = snap.waypoints.first().map(|wp| wp.uid);
    let last_uid = snap.waypoints.last().map(|wp| wp.uid);

    RouteDistanceSampleRow {
        wall_time_ms,
        sequence: snap.sequence,
        route_hash: snap.route_hash,
        valid: snap.valid,
        waypoint_count: snap.waypoints.len(),
        distance_count: dist.distance_count,
        distance_untrusted_count: dist.distance_untrusted_count,
        distance_first_m: dist.distance_first_m,
        distance_last_m: dist.distance_last_m,
        distance_min_m: dist.distance_min_m,
        distance_max_m: dist.distance_max_m,
        distance_increase_count: dist.distance_increase_count,
        distance_drop_max_m: dist.distance_drop_max_m,
        distance_step_avg_m: dist.distance_step_avg_m,
        distance_monotonic_status: dist.distance_monotonic_status.as_str().to_string(),
        position_count: coord.position_count,
        coord_status: coord.coord_status.as_str().to_string(),
        first_uid,
        last_uid,
    }
}

/// Build JSONL waypoint records for a snapshot.
pub fn snapshot_to_waypoint_records(
    wall_time_ms: u64,
    snap: &RouteSnapshot,
) -> Vec<RouteDistanceWaypointRecord> {
    snap.waypoints
        .iter()
        .enumerate()
        .map(|(index, wp)| RouteDistanceWaypointRecord {
            wall_time_ms,
            sequence: snap.sequence,
            route_hash: snap.route_hash,
            index,
            uid: wp.uid,
            x: wp.x,
            y: wp.y,
            z: wp.z,
            distance: wp.distance,
            time: wp.time,
            flags: wp.flags,
            flag_names: decode_waypoint_flag_names(wp.flags)
                .into_iter()
                .map(str::to_string)
                .collect(),
        })
        .collect()
}

/// CSV header for sample rows.
pub const SAMPLE_CSV_HEADER: &str = "wall_time_ms,sequence,route_hash,valid,waypoint_count,distance_count,distance_untrusted_count,distance_first_m,distance_last_m,distance_min_m,distance_max_m,distance_increase_count,distance_drop_max_m,distance_step_avg_m,distance_monotonic_status,position_count,coord_status,first_uid,last_uid";

fn fmt_opt_f32(v: Option<f32>) -> String {
    v.map(|x| format!("{x:.1}"))
        .unwrap_or_default()
}

fn fmt_opt_f64(v: Option<f64>) -> String {
    v.map(|x| format!("{x:.1}"))
        .unwrap_or_default()
}

fn fmt_opt_i64(v: Option<i64>) -> String {
    v.map(|x| x.to_string())
        .unwrap_or_default()
}

/// Format one CSV data row (no newline).
pub fn format_sample_csv_row(row: &RouteDistanceSampleRow) -> String {
    format!(
        "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
        row.wall_time_ms,
        row.sequence,
        row.route_hash,
        if row.valid { 1 } else { 0 },
        row.waypoint_count,
        row.distance_count,
        row.distance_untrusted_count,
        fmt_opt_f32(row.distance_first_m),
        fmt_opt_f32(row.distance_last_m),
        fmt_opt_f32(row.distance_min_m),
        fmt_opt_f32(row.distance_max_m),
        row.distance_increase_count,
        format!("{:.1}", row.distance_drop_max_m),
        fmt_opt_f64(row.distance_step_avg_m),
        row.distance_monotonic_status,
        row.position_count,
        row.coord_status,
        fmt_opt_i64(row.first_uid),
        fmt_opt_i64(row.last_uid),
    )
}

/// Write CSV header if the file is new or not appending.
pub fn write_sample_csv_header(w: &mut dyn Write, append: bool, file_exists: bool) -> io::Result<()> {
    if !append || !file_exists {
        writeln!(w, "{SAMPLE_CSV_HEADER}")?;
    }
    Ok(())
}

/// Append one sample row to a CSV writer.
pub fn write_sample_csv_row(w: &mut dyn Write, row: &RouteDistanceSampleRow) -> io::Result<()> {
    writeln!(w, "{}", format_sample_csv_row(row))
}

fn parse_opt_f32(s: &str) -> Option<f32> {
    let t = s.trim();
    if t.is_empty() {
        None
    } else {
        t.parse().ok()
    }
}

fn parse_opt_f64(s: &str) -> Option<f64> {
    let t = s.trim();
    if t.is_empty() {
        None
    } else {
        t.parse().ok()
    }
}

fn parse_opt_i64(s: &str) -> Option<i64> {
    let t = s.trim();
    if t.is_empty() {
        None
    } else {
        t.parse().ok()
    }
}

/// Parse CSV text (with header) into sample rows.
pub fn parse_sample_csv(content: &str) -> Result<Vec<RouteDistanceSampleRow>, String> {
    let mut lines = content.lines().filter(|l| !l.trim().is_empty());
    let header = lines
        .next()
        .ok_or_else(|| "CSV is empty".to_string())?;
    if header != SAMPLE_CSV_HEADER {
        return Err(format!("unexpected CSV header (expected canonical Phase 5k header)"));
    }

    let mut rows = Vec::new();
    for (line_no, line) in lines.enumerate() {
        let cols: Vec<&str> = line.split(',').collect();
        if cols.len() != 19 {
            return Err(format!(
                "line {}: expected 19 columns, got {}",
                line_no + 2,
                cols.len()
            ));
        }
        rows.push(RouteDistanceSampleRow {
            wall_time_ms: cols[0]
                .parse()
                .map_err(|e| format!("line {}: wall_time_ms: {e}", line_no + 2))?,
            sequence: cols[1]
                .parse()
                .map_err(|e| format!("line {}: sequence: {e}", line_no + 2))?,
            route_hash: cols[2]
                .parse()
                .map_err(|e| format!("line {}: route_hash: {e}", line_no + 2))?,
            valid: cols[3] == "1",
            waypoint_count: cols[4]
                .parse()
                .map_err(|e| format!("line {}: waypoint_count: {e}", line_no + 2))?,
            distance_count: cols[5]
                .parse()
                .map_err(|e| format!("line {}: distance_count: {e}", line_no + 2))?,
            distance_untrusted_count: cols[6].parse().map_err(|e| {
                format!("line {}: distance_untrusted_count: {e}", line_no + 2)
            })?,
            distance_first_m: parse_opt_f32(cols[7]),
            distance_last_m: parse_opt_f32(cols[8]),
            distance_min_m: parse_opt_f32(cols[9]),
            distance_max_m: parse_opt_f32(cols[10]),
            distance_increase_count: cols[11].parse().map_err(|e| {
                format!("line {}: distance_increase_count: {e}", line_no + 2)
            })?,
            distance_drop_max_m: cols[12]
                .parse()
                .map_err(|e| format!("line {}: distance_drop_max_m: {e}", line_no + 2))?,
            distance_step_avg_m: parse_opt_f64(cols[13]),
            distance_monotonic_status: cols[14].to_string(),
            position_count: cols[15]
                .parse()
                .map_err(|e| format!("line {}: position_count: {e}", line_no + 2))?,
            coord_status: cols[16].to_string(),
            first_uid: parse_opt_i64(cols[17]),
            last_uid: parse_opt_i64(cols[18]),
        });
    }
    Ok(rows)
}

/// Load sample rows from a CSV file path.
pub fn load_sample_csv(path: &Path) -> Result<Vec<RouteDistanceSampleRow>, String> {
    let raw = std::fs::read_to_string(path)
        .map_err(|e| format!("read {}: {e}", path.display()))?;
    parse_sample_csv(&raw)
}

/// Verification verdict for real-world distance @+0x14 logs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DistanceVerificationVerdict {
    Plausible,
    Inconclusive,
    Suspicious,
}

impl DistanceVerificationVerdict {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Plausible => "plausible",
            Self::Inconclusive => "inconclusive",
            Self::Suspicious => "suspicious",
        }
    }
}

/// Aggregated analysis of a recording session.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RouteDistanceReport {
    pub sample_count: usize,
    pub valid_sample_count: usize,
    pub unique_route_hash_count: usize,
    pub route_hash_changes: usize,
    pub total_duration_sec: f64,
    pub percent_samples_with_distance: f64,
    pub percent_untrusted: f64,
    pub monotonic_status_counts: HashMap<String, usize>,
    pub total_increase_events: usize,
    pub max_increase_count_per_sample: usize,
    pub max_drop_seen_m: f64,
    pub first_distance_min_m: Option<f32>,
    pub first_distance_max_m: Option<f32>,
    pub last_distance_min_m: Option<f32>,
    pub last_distance_max_m: Option<f32>,
    pub verdict: DistanceVerificationVerdict,
    pub reasons: Vec<String>,
}

/// Analyse sample rows and produce a verification report.
pub fn analyze_distance_samples(rows: &[RouteDistanceSampleRow]) -> RouteDistanceReport {
    let sample_count = rows.len();
    let valid_sample_count = rows.iter().filter(|r| r.valid).count();

    let mut hash_set = std::collections::HashSet::new();
    let mut route_hash_changes = 0usize;
    let mut prev_hash: Option<u64> = None;
    for row in rows {
        hash_set.insert(row.route_hash);
        if let Some(ph) = prev_hash {
            if ph != row.route_hash {
                route_hash_changes += 1;
            }
        }
        prev_hash = Some(row.route_hash);
    }

    let total_duration_sec = if sample_count >= 2 {
        let first = rows[0].wall_time_ms;
        let last = rows[sample_count - 1].wall_time_ms;
        last.saturating_sub(first) as f64 / 1000.0
    } else {
        0.0
    };

    let with_distance = rows.iter().filter(|r| r.distance_count > 0).count();
    let percent_samples_with_distance = if sample_count > 0 {
        with_distance as f64 / sample_count as f64 * 100.0
    } else {
        0.0
    };

    let untrusted_samples = rows
        .iter()
        .filter(|r| r.distance_untrusted_count > 0)
        .count();
    let percent_untrusted = if sample_count > 0 {
        untrusted_samples as f64 / sample_count as f64 * 100.0
    } else {
        0.0
    };

    let mut monotonic_status_counts: HashMap<String, usize> = HashMap::new();
    let mut total_increase_events = 0usize;
    let mut max_increase_count_per_sample = 0usize;
    let mut max_drop_seen_m = 0.0_f64;
    let mut first_mins = Vec::new();
    let mut first_maxs = Vec::new();
    let mut last_mins = Vec::new();
    let mut last_maxs = Vec::new();

    let mut prev_first: Option<f32> = None;
    let mut prev_hash_for_jump: Option<u64> = None;
    let mut large_first_jump_without_hash_change = 0usize;
    let mut distance_count_swings = 0usize;
    let mut prev_distance_count: Option<usize> = None;

    for row in rows {
        *monotonic_status_counts
            .entry(row.distance_monotonic_status.clone())
            .or_insert(0) += 1;
        total_increase_events += row.distance_increase_count;
        max_increase_count_per_sample = max_increase_count_per_sample.max(row.distance_increase_count);
        max_drop_seen_m = max_drop_seen_m.max(row.distance_drop_max_m);

        if let Some(f) = row.distance_first_m {
            first_mins.push(f);
            first_maxs.push(f);
        }
        if let Some(l) = row.distance_last_m {
            last_mins.push(l);
            last_maxs.push(l);
        }

        if let (Some(pf), Some(cf), Some(ph)) = (prev_first, row.distance_first_m, prev_hash_for_jump)
        {
            if ph == row.route_hash && (cf - pf).abs() > 5000.0 {
                large_first_jump_without_hash_change += 1;
            }
        }
        prev_first = row.distance_first_m;
        prev_hash_for_jump = Some(row.route_hash);

        if let Some(pdc) = prev_distance_count {
            if row.distance_count.abs_diff(pdc) > row.waypoint_count / 2 && row.waypoint_count > 0 {
                distance_count_swings += 1;
            }
        }
        prev_distance_count = Some(row.distance_count);
    }

    let ok_flat_count = rows
        .iter()
        .filter(|r| {
            r.distance_count > 0
                && (r.distance_monotonic_status == RouteDistanceMonotonicStatus::Ok.as_str()
                    || r.distance_monotonic_status == RouteDistanceMonotonicStatus::Flat.as_str())
        })
        .count();
    let bad_mono_count = rows
        .iter()
        .filter(|r| {
            r.distance_monotonic_status == RouteDistanceMonotonicStatus::Increasing.as_str()
                || r.distance_monotonic_status == RouteDistanceMonotonicStatus::Jumpy.as_str()
        })
        .count();

    let mut reasons = Vec::new();
    let mut verdict = DistanceVerificationVerdict::Inconclusive;

    if sample_count == 0 {
        reasons.push("no_distance_samples".into());
    } else {
        if sample_count < 20 {
            reasons.push("insufficient_duration".into());
        }
        if with_distance == 0 {
            reasons.push("no_distance_samples".into());
        }
        if percent_samples_with_distance < 80.0 {
            reasons.push("low_distance_coverage".into());
        }
        if bad_mono_count * 2 > sample_count.max(1) {
            reasons.push("too_many_increases".into());
        }
        if rows
            .iter()
            .filter(|r| r.distance_monotonic_status == RouteDistanceMonotonicStatus::Jumpy.as_str())
            .count()
            * 3
            > sample_count
        {
            reasons.push("too_many_jumps".into());
        }
        if monotonic_status_counts
            .get(RouteDistanceMonotonicStatus::Flat.as_str())
            .copied()
            .unwrap_or(0)
            * 2
            > sample_count
            && sample_count >= 10
        {
            reasons.push("mostly_flat".into());
        }
        if route_hash_changes > sample_count / 4 && sample_count >= 10 {
            reasons.push("route_hash_changes".into());
        }
        if large_first_jump_without_hash_change >= 3 {
            reasons.push("first_distance_jump_without_route_change".into());
        }
        if distance_count_swings > sample_count / 5 && sample_count >= 10 {
            reasons.push("distance_count_unstable".into());
        }

        let plausible = sample_count >= 20
            && percent_samples_with_distance >= 80.0
            && with_distance > 0
            && ok_flat_count * 100 / with_distance.max(1) >= 70
            && bad_mono_count * 100 / sample_count.max(1) <= 25
            && route_hash_changes <= sample_count / 4
            && large_first_jump_without_hash_change < 3
            && distance_count_swings <= sample_count / 5;

        let suspicious = bad_mono_count * 100 / sample_count.max(1) > 40
            || large_first_jump_without_hash_change >= 5
            || distance_count_swings > sample_count / 3
            || (with_distance == 0 && sample_count >= 5);

        if suspicious {
            verdict = DistanceVerificationVerdict::Suspicious;
        } else if plausible {
            verdict = DistanceVerificationVerdict::Plausible;
        } else {
            verdict = DistanceVerificationVerdict::Inconclusive;
        }
    }

    RouteDistanceReport {
        sample_count,
        valid_sample_count,
        unique_route_hash_count: hash_set.len(),
        route_hash_changes,
        total_duration_sec,
        percent_samples_with_distance,
        percent_untrusted,
        monotonic_status_counts,
        total_increase_events,
        max_increase_count_per_sample,
        max_drop_seen_m,
        first_distance_min_m: first_mins.iter().copied().reduce(f32::min),
        first_distance_max_m: first_maxs.iter().copied().reduce(f32::max),
        last_distance_min_m: last_mins.iter().copied().reduce(f32::min),
        last_distance_max_m: last_maxs.iter().copied().reduce(f32::max),
        verdict,
        reasons,
    }
}

/// Human-readable text report.
pub fn format_distance_report_text(report: &RouteDistanceReport) -> String {
    let mut out = String::new();
    writeln!(out, "ETS2 Route Distance Verification Report (Phase 5k)").unwrap();
    writeln!(out, "==================================================").unwrap();
    writeln!(out, "sample_count:              {}", report.sample_count).unwrap();
    writeln!(out, "valid_sample_count:        {}", report.valid_sample_count).unwrap();
    writeln!(out, "unique_route_hash_count:   {}", report.unique_route_hash_count).unwrap();
    writeln!(out, "route_hash_changes:        {}", report.route_hash_changes).unwrap();
    writeln!(
        out,
        "total_duration_sec:        {:.1}",
        report.total_duration_sec
    )
    .unwrap();
    writeln!(
        out,
        "percent_samples_with_distance: {:.1}%",
        report.percent_samples_with_distance
    )
    .unwrap();
    writeln!(
        out,
        "percent_untrusted:         {:.1}%",
        report.percent_untrusted
    )
    .unwrap();
    writeln!(out, "total_increase_events:     {}", report.total_increase_events).unwrap();
    writeln!(
        out,
        "max_increase_count/sample: {}",
        report.max_increase_count_per_sample
    )
    .unwrap();
    writeln!(
        out,
        "max_drop_seen_m:           {:.1}",
        report.max_drop_seen_m
    )
    .unwrap();
    if let (Some(min), Some(max)) = (report.first_distance_min_m, report.first_distance_max_m) {
        writeln!(out, "first_distance_range_m:    {min:.1} .. {max:.1}").unwrap();
    }
    if let (Some(min), Some(max)) = (report.last_distance_min_m, report.last_distance_max_m) {
        writeln!(out, "last_distance_range_m:     {min:.1} .. {max:.1}").unwrap();
    }
    writeln!(out, "monotonic_status_counts:").unwrap();
    let mut keys: Vec<_> = report.monotonic_status_counts.keys().collect();
    keys.sort();
    for k in keys {
        writeln!(out, "  {k}: {}", report.monotonic_status_counts[k]).unwrap();
    }
    writeln!(out, "verdict:                   {}", report.verdict.as_str()).unwrap();
    writeln!(out, "reasons:").unwrap();
    if report.reasons.is_empty() {
        writeln!(out, "  (none)").unwrap();
    } else {
        for r in &report.reasons {
            writeln!(out, "  - {r}").unwrap();
        }
    }
    writeln!(
        out,
        "\nNote: distance @ item+0x14 remains ROUTE_WP_FLAG_UNTRUSTED until multiple real logs are plausible."
    )
    .unwrap();
    out
}

/// Per-run report entry for meta-analysis (Phase 5l).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RouteDistancePerRunReport {
    pub file: String,
    pub report: RouteDistanceReport,
}

/// Aggregated metrics across multiple recording runs.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RouteDistanceMetaAggregateMetrics {
    pub run_count: usize,
    pub valid_run_count: usize,
    pub plausible_count: usize,
    pub inconclusive_count: usize,
    pub suspicious_count: usize,
    pub total_samples: usize,
    pub total_valid_samples: usize,
    pub total_duration_sec: f64,
    pub weighted_percent_samples_with_distance: f64,
    pub weighted_percent_untrusted: f64,
    pub total_route_hash_changes: usize,
    pub total_increase_events: usize,
    pub max_drop_seen_m_global: f64,
    pub bad_monotonic_sample_ratio_global: f64,
    pub files_with_no_distance: usize,
    pub files_with_route_hash_changes: usize,
    pub files_with_first_distance_jumps: usize,
}

/// Meta-report over multiple CSV recordings.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RouteDistanceMetaReport {
    pub generated_at_ms: u64,
    pub input_files: Vec<String>,
    pub min_runs_required: usize,
    pub per_run_reports: Vec<RouteDistancePerRunReport>,
    pub aggregate_metrics: RouteDistanceMetaAggregateMetrics,
    pub verdict: DistanceVerificationVerdict,
    pub reasons: Vec<String>,
}

fn count_bad_monotonic_samples(rows: &[RouteDistanceSampleRow]) -> usize {
    rows.iter()
        .filter(|r| {
            r.distance_count > 0
                && (r.distance_monotonic_status
                    == RouteDistanceMonotonicStatus::Increasing.as_str()
                    || r.distance_monotonic_status == RouteDistanceMonotonicStatus::Jumpy.as_str())
        })
        .count()
}

fn count_samples_with_distance(rows: &[RouteDistanceSampleRow]) -> usize {
    rows.iter().filter(|r| r.distance_count > 0).count()
}

fn run_has_no_distance(report: &RouteDistanceReport) -> bool {
    report.percent_samples_with_distance == 0.0
        || report
            .reasons
            .iter()
            .any(|r| r == "no_distance_samples")
}

fn run_has_first_distance_jumps(report: &RouteDistanceReport) -> bool {
    report
        .reasons
        .iter()
        .any(|r| r == "first_distance_jump_without_route_change")
}

/// Build per-run reports from CSV paths (loads each file once).
pub fn analyze_distance_runs(paths: &[impl AsRef<Path>]) -> Result<Vec<RouteDistancePerRunReport>, String> {
    let mut runs = Vec::new();
    for path in paths {
        let path_ref = path.as_ref();
        let rows = load_sample_csv(path_ref)?;
        let report = analyze_distance_samples(&rows);
        runs.push(RouteDistancePerRunReport {
            file: path_ref.display().to_string(),
            report,
        });
    }
    Ok(runs)
}

/// Aggregate per-run reports into a meta-report.
pub fn analyze_distance_meta(
    per_runs: &[RouteDistancePerRunReport],
    rows_by_run: &[Vec<RouteDistanceSampleRow>],
    min_runs: usize,
    generated_at_ms: u64,
) -> RouteDistanceMetaReport {
    let input_files: Vec<String> = per_runs.iter().map(|r| r.file.clone()).collect();
    let run_count = per_runs.len();

    let plausible_count = per_runs
        .iter()
        .filter(|r| r.report.verdict == DistanceVerificationVerdict::Plausible)
        .count();
    let inconclusive_count = per_runs
        .iter()
        .filter(|r| r.report.verdict == DistanceVerificationVerdict::Inconclusive)
        .count();
    let suspicious_count = per_runs
        .iter()
        .filter(|r| r.report.verdict == DistanceVerificationVerdict::Suspicious)
        .count();

    let mut total_samples = 0usize;
    let mut total_valid_samples = 0usize;
    let mut total_duration_sec = 0.0_f64;
    let mut total_with_distance = 0usize;
    let mut total_untrusted_samples = 0usize;
    let mut total_route_hash_changes = 0usize;
    let mut total_increase_events = 0usize;
    let mut max_drop_seen_m_global = 0.0_f64;
    let mut total_bad_monotonic = 0usize;
    let mut total_with_distance_samples = 0usize;

    let mut files_with_no_distance = 0usize;
    let mut files_with_route_hash_changes = 0usize;
    let mut files_with_first_distance_jumps = 0usize;

    for (run, rows) in per_runs.iter().zip(rows_by_run.iter()) {
        total_samples += run.report.sample_count;
        total_valid_samples += run.report.valid_sample_count;
        total_duration_sec += run.report.total_duration_sec;
        total_route_hash_changes += run.report.route_hash_changes;
        total_increase_events += run.report.total_increase_events;
        max_drop_seen_m_global = max_drop_seen_m_global.max(run.report.max_drop_seen_m);

        let with_dist = count_samples_with_distance(rows);
        total_with_distance += with_dist;
        total_untrusted_samples += rows
            .iter()
            .filter(|r| r.distance_untrusted_count > 0)
            .count();
        total_bad_monotonic += count_bad_monotonic_samples(rows);
        total_with_distance_samples += with_dist;

        if run_has_no_distance(&run.report) {
            files_with_no_distance += 1;
        }
        if run.report.route_hash_changes > 0 {
            files_with_route_hash_changes += 1;
        }
        if run_has_first_distance_jumps(&run.report) {
            files_with_first_distance_jumps += 1;
        }
    }

    let valid_run_count = per_runs
        .iter()
        .filter(|r| r.report.sample_count > 0)
        .count();

    let weighted_percent_samples_with_distance = if total_samples > 0 {
        total_with_distance as f64 / total_samples as f64 * 100.0
    } else {
        0.0
    };

    let weighted_percent_untrusted = if total_samples > 0 {
        total_untrusted_samples as f64 / total_samples as f64 * 100.0
    } else {
        0.0
    };

    let bad_monotonic_sample_ratio_global = if total_with_distance_samples > 0 {
        total_bad_monotonic as f64 / total_with_distance_samples as f64 * 100.0
    } else {
        0.0
    };

    let aggregate_metrics = RouteDistanceMetaAggregateMetrics {
        run_count,
        valid_run_count,
        plausible_count,
        inconclusive_count,
        suspicious_count,
        total_samples,
        total_valid_samples,
        total_duration_sec,
        weighted_percent_samples_with_distance,
        weighted_percent_untrusted,
        total_route_hash_changes,
        total_increase_events,
        max_drop_seen_m_global,
        bad_monotonic_sample_ratio_global,
        files_with_no_distance,
        files_with_route_hash_changes,
        files_with_first_distance_jumps,
    };

    let mut reasons = Vec::new();

    if run_count < min_runs {
        reasons.push("insufficient_runs".into());
    }
    if plausible_count < 2 {
        reasons.push("too_few_plausible_runs".into());
    }
    if suspicious_count > 0 {
        reasons.push("suspicious_runs_present".into());
    }
    if weighted_percent_samples_with_distance < 80.0 && total_samples > 0 {
        reasons.push("low_weighted_distance_coverage".into());
    }
    if bad_monotonic_sample_ratio_global > 25.0 && total_with_distance_samples > 0 {
        reasons.push("high_bad_monotonic_ratio".into());
    }
    if files_with_no_distance > 0 && valid_run_count >= 2 {
        reasons.push("runs_without_distance".into());
    }
    if files_with_first_distance_jumps >= 2 {
        reasons.push("multiple_first_distance_jump_runs".into());
    }

    let plausible = run_count >= min_runs
        && plausible_count >= 2
        && suspicious_count == 0
        && weighted_percent_samples_with_distance >= 80.0
        && bad_monotonic_sample_ratio_global <= 25.0;

    let suspicious = (suspicious_count >= 1 && run_count >= 2)
        || (files_with_no_distance > 0 && valid_run_count >= 2)
        || files_with_first_distance_jumps >= 2
        || bad_monotonic_sample_ratio_global > 40.0;

    let verdict = if suspicious {
        DistanceVerificationVerdict::Suspicious
    } else if plausible {
        DistanceVerificationVerdict::Plausible
    } else {
        DistanceVerificationVerdict::Inconclusive
    };

    RouteDistanceMetaReport {
        generated_at_ms,
        input_files,
        min_runs_required: min_runs,
        per_run_reports: per_runs.to_vec(),
        aggregate_metrics,
        verdict,
        reasons,
    }
}

/// Load CSVs and produce a full meta-report.
pub fn build_distance_meta_report(
    paths: &[impl AsRef<Path>],
    min_runs: usize,
    generated_at_ms: u64,
) -> Result<RouteDistanceMetaReport, String> {
    if paths.is_empty() {
        return Err("no input CSV files".into());
    }
    let mut per_runs = Vec::new();
    let mut rows_by_run = Vec::new();
    for path in paths {
        let path_ref = path.as_ref();
        let rows = load_sample_csv(path_ref)?;
        let report = analyze_distance_samples(&rows);
        per_runs.push(RouteDistancePerRunReport {
            file: path_ref.display().to_string(),
            report,
        });
        rows_by_run.push(rows);
    }
    Ok(analyze_distance_meta(
        &per_runs,
        &rows_by_run,
        min_runs,
        generated_at_ms,
    ))
}

/// Expand glob patterns in path strings (supports `*` and `?` in the final segment).
pub fn expand_csv_path_patterns(patterns: &[String]) -> Result<Vec<std::path::PathBuf>, String> {
    use std::path::PathBuf;

    let mut out = Vec::new();
    for pattern in patterns {
        if !pattern.contains('*') && !pattern.contains('?') {
            out.push(PathBuf::from(pattern));
            continue;
        }

        let path = PathBuf::from(pattern);
        let parent = path
            .parent()
            .filter(|p| !p.as_os_str().is_empty())
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from("."));
        let file_pattern = path
            .file_name()
            .and_then(|s| s.to_str())
            .ok_or_else(|| format!("invalid glob pattern: {pattern}"))?;

        let entries = std::fs::read_dir(&parent)
            .map_err(|e| format!("read dir {}: {e}", parent.display()))?;
        for entry in entries {
            let entry = entry.map_err(|e| format!("read dir entry: {e}"))?;
            let name = entry.file_name();
            let name_str = name.to_string_lossy();
            if simple_glob_match(file_pattern, &name_str) {
                out.push(parent.join(name));
            }
        }
    }
    out.sort();
    out.dedup();
    if out.is_empty() {
        return Err("no files matched input patterns".into());
    }
    Ok(out)
}

fn simple_glob_match(pattern: &str, text: &str) -> bool {
    fn rec(pat: &[u8], txt: &[u8]) -> bool {
        match (pat.first(), txt.first()) {
            (None, None) => true,
            (Some(b'?'), Some(_)) => rec(&pat[1..], &txt[1..]),
            (Some(b'*'), _) => {
                rec(&pat[1..], txt)
                    || (!txt.is_empty() && rec(pat, &txt[1..]))
            }
            (Some(p), Some(t)) if p == t => rec(&pat[1..], &txt[1..]),
            _ => false,
        }
    }
    rec(pattern.as_bytes(), text.as_bytes())
}

/// Collect CSV paths from CLI args (non-flag), expanding globs.
pub fn collect_meta_report_input_paths(args: &[String]) -> Result<Vec<std::path::PathBuf>, String> {
    collect_tool_input_paths(args)
}

/// Human-readable meta-report text.
pub fn format_distance_meta_report_text(meta: &RouteDistanceMetaReport) -> String {
    let mut out = String::new();
    writeln!(out, "ETS2 Route Distance Meta-Report (Phase 5l)").unwrap();
    writeln!(out, "=============================================").unwrap();
    writeln!(out, "input_files:               {}", meta.input_files.len()).unwrap();
    writeln!(
        out,
        "min_runs_required:         {}",
        meta.min_runs_required
    )
    .unwrap();
    writeln!(out, "generated_at_ms:             {}", meta.generated_at_ms).unwrap();
    writeln!(out).unwrap();

    let a = &meta.aggregate_metrics;
    writeln!(out, "Aggregate metrics").unwrap();
    writeln!(out, "-----------------").unwrap();
    writeln!(out, "run_count:                 {}", a.run_count).unwrap();
    writeln!(out, "valid_run_count:           {}", a.valid_run_count).unwrap();
    writeln!(out, "plausible_count:           {}", a.plausible_count).unwrap();
    writeln!(out, "inconclusive_count:        {}", a.inconclusive_count).unwrap();
    writeln!(out, "suspicious_count:          {}", a.suspicious_count).unwrap();
    writeln!(out, "total_samples:             {}", a.total_samples).unwrap();
    writeln!(out, "total_valid_samples:       {}", a.total_valid_samples).unwrap();
    writeln!(
        out,
        "total_duration_sec:        {:.1}",
        a.total_duration_sec
    )
    .unwrap();
    writeln!(
        out,
        "weighted_distance_coverage: {:.1}%",
        a.weighted_percent_samples_with_distance
    )
    .unwrap();
    writeln!(
        out,
        "weighted_untrusted:        {:.1}%",
        a.weighted_percent_untrusted
    )
    .unwrap();
    writeln!(
        out,
        "total_route_hash_changes:  {}",
        a.total_route_hash_changes
    )
    .unwrap();
    writeln!(
        out,
        "total_increase_events:     {}",
        a.total_increase_events
    )
    .unwrap();
    writeln!(
        out,
        "max_drop_seen_m_global:    {:.1}",
        a.max_drop_seen_m_global
    )
    .unwrap();
    writeln!(
        out,
        "bad_monotonic_ratio_global: {:.1}%",
        a.bad_monotonic_sample_ratio_global
    )
    .unwrap();
    writeln!(
        out,
        "files_with_no_distance:    {}",
        a.files_with_no_distance
    )
    .unwrap();
    writeln!(
        out,
        "files_with_route_hash_changes: {}",
        a.files_with_route_hash_changes
    )
    .unwrap();
    writeln!(
        out,
        "files_with_first_distance_jumps: {}",
        a.files_with_first_distance_jumps
    )
    .unwrap();
    writeln!(out).unwrap();

    writeln!(out, "Per-run summaries").unwrap();
    writeln!(out, "-----------------").unwrap();
    for run in &meta.per_run_reports {
        writeln!(
            out,
            "  {} → verdict={} samples={} duration={:.1}s distance={:.1}%",
            run.file,
            run.report.verdict.as_str(),
            run.report.sample_count,
            run.report.total_duration_sec,
            run.report.percent_samples_with_distance
        )
        .unwrap();
    }
    writeln!(out).unwrap();
    writeln!(out, "meta_verdict:              {}", meta.verdict.as_str()).unwrap();
    writeln!(out, "meta_reasons:").unwrap();
    if meta.reasons.is_empty() {
        writeln!(out, "  (none)").unwrap();
    } else {
        for r in &meta.reasons {
            writeln!(out, "  - {r}").unwrap();
        }
    }
    writeln!(
        out,
        "\nNote: distance @ item+0x14 remains ROUTE_WP_FLAG_UNTRUSTED until meta-report is plausible across multiple real runs."
    )
    .unwrap();
    out
}

/// Known scenario tags for filename-based tagging (Phase 5m).
pub const KNOWN_ROUTE_DISTANCE_SCENARIOS: &[&str] = &[
    "autobahn", "stadt", "auffahrt", "depot", "pause", "reroute",
];

/// Optional sidecar metadata (`<stem>.meta.json` next to CSV).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RouteDistanceRunSidecar {
    pub scenario: String,
    #[serde(default)]
    pub ets2_version: Option<String>,
    #[serde(default)]
    pub map: Option<String>,
    #[serde(default)]
    pub notes: Option<String>,
}

/// Scenario assignment for one recording run.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RouteDistanceRunScenarioInfo {
    pub file: String,
    pub scenario: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sidecar: Option<RouteDistanceRunSidecar>,
}

/// Promotion gate verdict (Phase 5m) — stricter than meta-report.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RouteDistancePromotionVerdict {
    Passed,
    Failed,
    Inconclusive,
}

impl RouteDistancePromotionVerdict {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Passed => "passed",
            Self::Failed => "failed",
            Self::Inconclusive => "inconclusive",
        }
    }
}

/// Stricter verification gate configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct RouteDistanceVerificationConfig {
    pub min_runs: usize,
    pub min_weighted_distance_coverage_pct: f64,
    pub max_bad_monotonic_ratio_pct: f64,
    pub required_scenarios: Vec<String>,
}

impl Default for RouteDistanceVerificationConfig {
    fn default() -> Self {
        Self {
            min_runs: 5,
            min_weighted_distance_coverage_pct: 90.0,
            max_bad_monotonic_ratio_pct: 10.0,
            required_scenarios: Vec::new(),
        }
    }
}

/// Full verification gate report (reuses meta-report internally).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RouteDistanceVerificationReport {
    pub generated_at_ms: u64,
    pub input_files: Vec<String>,
    pub run_scenarios: Vec<RouteDistanceRunScenarioInfo>,
    pub detected_scenarios: Vec<String>,
    pub required_scenarios: Vec<String>,
    pub meta_report: RouteDistanceMetaReport,
    pub aggregate_metrics: RouteDistanceMetaAggregateMetrics,
    pub verdict: RouteDistancePromotionVerdict,
    pub reasons: Vec<String>,
    pub recommendation: String,
}

/// Detect scenario tag from CSV filename (`autobahn_01.csv` → `autobahn`).
pub fn detect_scenario_from_filename(path: impl AsRef<Path>) -> Option<String> {
    let path = path.as_ref();
    let stem = path.file_stem()?.to_str()?.to_lowercase();
    for scenario in KNOWN_ROUTE_DISTANCE_SCENARIOS {
        if stem == *scenario
            || stem.starts_with(&format!("{scenario}_"))
            || stem.starts_with(&format!("{scenario}-"))
        {
            return Some((*scenario).to_string());
        }
    }
    None
}

/// Load optional sidecar `<stem>.meta.json` for a CSV path.
pub fn load_run_sidecar(csv_path: &Path) -> Option<RouteDistanceRunSidecar> {
    let stem = csv_path.file_stem()?.to_str()?;
    let parent = csv_path.parent().unwrap_or_else(|| Path::new("."));
    let sidecar_path = parent.join(format!("{stem}.meta.json"));
    let raw = std::fs::read_to_string(&sidecar_path).ok()?;
    serde_json::from_str(&raw).ok()
}

/// Resolve scenario for a run (sidecar overrides filename).
pub fn resolve_run_scenario(csv_path: &Path) -> RouteDistanceRunScenarioInfo {
    let file = csv_path.display().to_string();
    if let Some(sidecar) = load_run_sidecar(csv_path) {
        let scenario = Some(sidecar.scenario.clone());
        return RouteDistanceRunScenarioInfo {
            file,
            scenario,
            sidecar: Some(sidecar),
        };
    }
    RouteDistanceRunScenarioInfo {
        file: file.clone(),
        scenario: detect_scenario_from_filename(csv_path),
        sidecar: None,
    }
}

fn run_has_unexplained_route_hash(run: &RouteDistancePerRunReport, scenario: Option<&str>) -> bool {
    run.report.route_hash_changes > 0 && scenario != Some("reroute")
}

fn is_count_only_verification_reason(reason: &String) -> bool {
    matches!(
        reason.as_str(),
        "insufficient_runs" | "meta_not_plausible" | "missing_required_scenarios"
    ) || reason.starts_with("missing_required_scenario:")
}

/// Apply promotion gate criteria on top of meta-report.
pub fn analyze_distance_verification(
    per_runs: &[RouteDistancePerRunReport],
    rows_by_run: &[Vec<RouteDistanceSampleRow>],
    run_scenarios: &[RouteDistanceRunScenarioInfo],
    config: &RouteDistanceVerificationConfig,
    generated_at_ms: u64,
) -> RouteDistanceVerificationReport {
    let meta = analyze_distance_meta(per_runs, rows_by_run, config.min_runs, generated_at_ms);
    let aggregate_metrics = meta.aggregate_metrics.clone();
    let a = &aggregate_metrics;

    let detected: std::collections::BTreeSet<String> = run_scenarios
        .iter()
        .filter_map(|r| r.scenario.clone())
        .collect();
    let detected_scenarios: Vec<String> = detected.iter().cloned().collect();

    let mut reasons = Vec::new();

    if a.run_count < config.min_runs {
        reasons.push("insufficient_runs".into());
    }
    if meta.verdict != DistanceVerificationVerdict::Plausible {
        reasons.push("meta_not_plausible".into());
    }
    if a.suspicious_count > 0 {
        reasons.push("suspicious_runs_present".into());
    }
    if a.weighted_percent_samples_with_distance < config.min_weighted_distance_coverage_pct {
        reasons.push("low_distance_coverage".into());
    }
    if a.bad_monotonic_sample_ratio_global > config.max_bad_monotonic_ratio_pct {
        reasons.push("bad_monotonic_ratio_too_high".into());
    }
    if a.files_with_no_distance > 0 {
        reasons.push("no_distance_run".into());
    }
    if a.files_with_first_distance_jumps > 0 {
        reasons.push("unexplained_distance_jumps".into());
    }

    let mut route_hash_unstable = false;
    for (run, info) in per_runs.iter().zip(run_scenarios.iter()) {
        if run_has_unexplained_route_hash(run, info.scenario.as_deref()) {
            route_hash_unstable = true;
            break;
        }
    }
    if route_hash_unstable {
        reasons.push("route_hash_instability".into());
    }

    let missing: Vec<_> = config
        .required_scenarios
        .iter()
        .filter(|req| !detected.contains(req.as_str()))
        .cloned()
        .collect();
    if !missing.is_empty() {
        reasons.push("missing_required_scenarios".into());
        for m in missing {
            reasons.push(format!("missing_required_scenario:{m}"));
        }
    }

    let passes = a.run_count >= config.min_runs
        && meta.verdict == DistanceVerificationVerdict::Plausible
        && a.suspicious_count == 0
        && a.weighted_percent_samples_with_distance >= config.min_weighted_distance_coverage_pct
        && a.bad_monotonic_sample_ratio_global <= config.max_bad_monotonic_ratio_pct
        && a.files_with_no_distance == 0
        && a.files_with_first_distance_jumps == 0
        && !route_hash_unstable
        && config
            .required_scenarios
            .iter()
            .all(|req| detected.contains(req.as_str()));

    let verdict = if passes {
        RouteDistancePromotionVerdict::Passed
    } else if a.run_count < config.min_runs
        && reasons.iter().all(is_count_only_verification_reason)
    {
        RouteDistancePromotionVerdict::Inconclusive
    } else {
        RouteDistancePromotionVerdict::Failed
    };

    let recommendation = if passes {
        "Eligible to promote distance @+0x14 to trusted after code review".into()
    } else {
        "Do not remove ROUTE_WP_FLAG_UNTRUSTED yet".into()
    };

    RouteDistanceVerificationReport {
        generated_at_ms,
        input_files: per_runs.iter().map(|r| r.file.clone()).collect(),
        run_scenarios: run_scenarios.to_vec(),
        detected_scenarios,
        required_scenarios: config.required_scenarios.clone(),
        meta_report: meta,
        aggregate_metrics,
        verdict,
        reasons,
        recommendation,
    }
}

/// Load CSVs and produce a verification gate report.
pub fn build_distance_verification_report(
    paths: &[impl AsRef<Path>],
    config: &RouteDistanceVerificationConfig,
    generated_at_ms: u64,
) -> Result<RouteDistanceVerificationReport, String> {
    if paths.is_empty() {
        return Err("no input CSV files".into());
    }
    let mut per_runs = Vec::new();
    let mut rows_by_run = Vec::new();
    let mut run_scenarios = Vec::new();
    for path in paths {
        let path_ref = path.as_ref();
        let rows = load_sample_csv(path_ref)?;
        let report = analyze_distance_samples(&rows);
        per_runs.push(RouteDistancePerRunReport {
            file: path_ref.display().to_string(),
            report,
        });
        rows_by_run.push(rows);
        run_scenarios.push(resolve_run_scenario(path_ref));
    }
    Ok(analyze_distance_verification(
        &per_runs,
        &rows_by_run,
        &run_scenarios,
        config,
        generated_at_ms,
    ))
}

/// Parse comma-separated scenario list (`autobahn,stadt,depot`).
pub fn parse_required_scenarios(raw: &str) -> Vec<String> {
    raw.split(',')
        .map(|s| s.trim().to_lowercase())
        .filter(|s| !s.is_empty())
        .collect()
}

const TOOL_VALUE_FLAGS: &[&str] = &[
    "--json",
    "--min-runs",
    "--require-scenarios",
    "--markdown",
];

fn is_tool_value_flag(flag: &str) -> bool {
    TOOL_VALUE_FLAGS.contains(&flag)
}

/// Collect CSV path patterns from CLI args, skipping known flags and their values.
pub fn collect_tool_input_paths(args: &[String]) -> Result<Vec<std::path::PathBuf>, String> {
    let mut patterns = Vec::new();
    let mut i = 1;
    while i < args.len() {
        let arg = &args[i];
        if arg.starts_with('-') {
            if is_tool_value_flag(arg) && i + 1 < args.len() && !args[i + 1].starts_with('-') {
                i += 2;
            } else {
                i += 1;
            }
            continue;
        }
        patterns.push(arg.clone());
        i += 1;
    }
    if patterns.is_empty() {
        return Err("no input CSV files (positional args or globs)".into());
    }
    expand_csv_path_patterns(&patterns)
}

/// Human-readable verification gate output.
pub fn format_distance_verification_text(report: &RouteDistanceVerificationReport) -> String {
    let mut out = String::new();
    writeln!(out, "ETS2 Route Distance Verification Gate (Phase 5m)").unwrap();
    writeln!(out, "===================================================").unwrap();
    writeln!(out, "verdict:        {}", report.verdict.as_str()).unwrap();
    writeln!(out, "recommendation: {}", report.recommendation).unwrap();
    writeln!(out).unwrap();
    writeln!(out, "detected_scenarios: {}", report.detected_scenarios.join(", ")).unwrap();
    if !report.required_scenarios.is_empty() {
        writeln!(
            out,
            "required_scenarios: {}",
            report.required_scenarios.join(", ")
        )
        .unwrap();
    }
    writeln!(out, "input_files ({}):", report.input_files.len()).unwrap();
    for f in &report.input_files {
        writeln!(out, "  - {f}").unwrap();
    }
    writeln!(out).unwrap();
    let a = &report.aggregate_metrics;
    writeln!(out, "run_count:                 {}", a.run_count).unwrap();
    writeln!(
        out,
        "weighted_distance_coverage: {:.1}%",
        a.weighted_percent_samples_with_distance
    )
    .unwrap();
    writeln!(
        out,
        "bad_monotonic_ratio_global: {:.1}%",
        a.bad_monotonic_sample_ratio_global
    )
    .unwrap();
    writeln!(
        out,
        "meta_verdict:              {}",
        report.meta_report.verdict.as_str()
    )
    .unwrap();
    writeln!(out, "reasons:").unwrap();
    if report.reasons.is_empty() {
        writeln!(out, "  (none)").unwrap();
    } else {
        for r in &report.reasons {
            writeln!(out, "  - {r}").unwrap();
        }
    }
    writeln!(
        out,
        "\nNote: This gate does not modify code or remove ROUTE_WP_FLAG_UNTRUSTED automatically."
    )
    .unwrap();
    out
}

/// Markdown promotion report for manual review.
pub fn format_distance_verification_markdown(report: &RouteDistanceVerificationReport) -> String {
    let mut out = String::new();
    writeln!(out, "# ETS2 Route Distance Verification Report").unwrap();
    writeln!(out).unwrap();
    writeln!(out, "- **Generated (ms):** {}", report.generated_at_ms).unwrap();
    writeln!(out, "- **Verdict:** `{}`", report.verdict.as_str()).unwrap();
    writeln!(out, "- **Recommendation:** {}", report.recommendation).unwrap();
    writeln!(out).unwrap();
    writeln!(out, "## Input files").unwrap();
    for f in &report.input_files {
        writeln!(out, "- `{f}`").unwrap();
    }
    writeln!(out).unwrap();
    writeln!(out, "## Scenarios").unwrap();
    writeln!(
        out,
        "- Detected: {}",
        if report.detected_scenarios.is_empty() {
            "(none)".into()
        } else {
            report.detected_scenarios.join(", ")
        }
    )
    .unwrap();
    if !report.required_scenarios.is_empty() {
        writeln!(
            out,
            "- Required: {}",
            report.required_scenarios.join(", ")
        )
        .unwrap();
    }
    for rs in &report.run_scenarios {
        writeln!(
            out,
            "- `{}` → scenario: {}",
            rs.file,
            rs.scenario.as_deref().unwrap_or("(unknown)")
        )
        .unwrap();
    }
    writeln!(out).unwrap();
    writeln!(out, "## Aggregate metrics").unwrap();
    let a = &report.aggregate_metrics;
    writeln!(out, "| Metric | Value |").unwrap();
    writeln!(out, "|--------|-------|").unwrap();
    writeln!(out, "| run_count | {} |", a.run_count).unwrap();
    writeln!(out, "| plausible_count | {} |", a.plausible_count).unwrap();
    writeln!(out, "| suspicious_count | {} |", a.suspicious_count).unwrap();
    writeln!(
        out,
        "| weighted_distance_coverage | {:.1}% |",
        a.weighted_percent_samples_with_distance
    )
    .unwrap();
    writeln!(
        out,
        "| bad_monotonic_ratio_global | {:.1}% |",
        a.bad_monotonic_sample_ratio_global
    )
    .unwrap();
    writeln!(
        out,
        "| total_route_hash_changes | {} |",
        a.total_route_hash_changes
    )
    .unwrap();
    writeln!(out, "| meta_verdict | {} |", report.meta_report.verdict.as_str()).unwrap();
    writeln!(out).unwrap();
    writeln!(out, "## Reasons").unwrap();
    if report.reasons.is_empty() {
        writeln!(out, "- (none)").unwrap();
    } else {
        for r in &report.reasons {
            writeln!(out, "- `{r}`").unwrap();
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nav_route::{RouteWaypoint, ROUTE_WP_FLAG_HAS_DISTANCE, ROUTE_WP_FLAG_UNTRUSTED};

    fn snap_with_distances(distances: &[f32]) -> RouteSnapshot {
        RouteSnapshot {
            sequence: 1,
            route_hash: 0xABC,
            valid: true,
            flags: 0,
            bb_status: 0,
            waypoints: distances
                .iter()
                .enumerate()
                .map(|(i, &d)| RouteWaypoint {
                    uid: 1000 + i as i64,
                    distance: d,
                    flags: ROUTE_WP_FLAG_HAS_DISTANCE | ROUTE_WP_FLAG_UNTRUSTED,
                    ..Default::default()
                })
                .collect(),
            ..Default::default()
        }
    }

    fn row_from_distances(wall_ms: u64, hash: u64, distances: &[f32]) -> RouteDistanceSampleRow {
        let mut snap = snap_with_distances(distances);
        snap.route_hash = hash;
        snapshot_to_sample_row(wall_ms, &snap)
    }

    #[test]
    fn sample_row_uses_diagnose_values() {
        let row = row_from_distances(1000, 42, &[3000.0, 2000.0, 1000.0]);
        assert_eq!(row.distance_count, 3);
        assert_eq!(row.distance_untrusted_count, 3);
        assert_eq!(row.distance_monotonic_status, "ok");
        assert_eq!(row.distance_first_m, Some(3000.0));
        assert_eq!(row.first_uid, Some(1000));
        assert_eq!(row.last_uid, Some(1002));
    }

    #[test]
    fn csv_round_trip() {
        let rows = vec![
            row_from_distances(1000, 1, &[100.0, 50.0]),
            row_from_distances(1500, 1, &[]),
        ];
        let mut buf = Vec::new();
        write_sample_csv_header(&mut buf, false, false).unwrap();
        for r in &rows {
            write_sample_csv_row(&mut buf, r).unwrap();
        }
        let parsed = parse_sample_csv(&String::from_utf8(buf).unwrap()).unwrap();
        assert_eq!(parsed.len(), 2);
        assert_eq!(parsed[0].distance_count, 2);
        assert_eq!(parsed[1].distance_count, 0);
    }

    #[test]
    fn report_plausible_monotone_session() {
        let rows: Vec<_> = (0..25)
            .map(|i| {
                row_from_distances(
                    i * 500,
                    99,
                    &[50000.0 - i as f32 * 100.0, 40000.0 - i as f32 * 100.0],
                )
            })
            .collect();
        let report = analyze_distance_samples(&rows);
        assert_eq!(report.verdict, DistanceVerificationVerdict::Plausible);
        assert!(report.percent_samples_with_distance >= 80.0);
    }

    #[test]
    fn report_no_distance_is_inconclusive_or_suspicious() {
        let rows: Vec<_> = (0..10)
            .map(|i| {
                let snap = RouteSnapshot {
                    sequence: i as u32,
                    route_hash: 1,
                    valid: true,
                    flags: 0,
                    bb_status: 0,
                    waypoints: vec![RouteWaypoint {
                        uid: 1,
                        ..Default::default()
                    }],
                    ..Default::default()
                };
                snapshot_to_sample_row(i * 1000, &snap)
            })
            .collect();
        let report = analyze_distance_samples(&rows);
        assert_ne!(report.verdict, DistanceVerificationVerdict::Plausible);
        assert!(report.reasons.iter().any(|r| r.contains("no_distance")));
    }

    #[test]
    fn report_many_increasing_is_suspicious() {
        let rows: Vec<_> = (0..30)
            .map(|i| {
                row_from_distances(
                    i * 500,
                    1,
                    &[1000.0 + i as f32 * 100.0, 2000.0 + i as f32 * 100.0, 3000.0 + i as f32 * 100.0],
                )
            })
            .collect();
        let report = analyze_distance_samples(&rows);
        assert_eq!(report.verdict, DistanceVerificationVerdict::Suspicious);
    }

    #[test]
    fn report_detects_route_hash_changes() {
        let rows = vec![
            row_from_distances(0, 1, &[100.0, 50.0]),
            row_from_distances(500, 2, &[200.0, 100.0]),
            row_from_distances(1000, 2, &[190.0, 90.0]),
        ];
        let report = analyze_distance_samples(&rows);
        assert_eq!(report.route_hash_changes, 1);
        assert_eq!(report.unique_route_hash_count, 2);
    }

    #[test]
    fn json_report_serializes() {
        let rows = vec![row_from_distances(0, 1, &[100.0, 50.0])];
        let report = analyze_distance_samples(&rows);
        let json = serde_json::to_string(&report).unwrap();
        assert!(json.contains("\"verdict\""));
    }

    fn plausible_run_rows(base_ms: u64, hash: u64) -> Vec<RouteDistanceSampleRow> {
        (0..25)
            .map(|i| {
                row_from_distances(
                    base_ms + i * 500,
                    hash,
                    &[50000.0 - i as f32 * 100.0, 40000.0 - i as f32 * 100.0],
                )
            })
            .collect()
    }

    fn build_per_run(file: &str, rows: Vec<RouteDistanceSampleRow>) -> (RouteDistancePerRunReport, Vec<RouteDistanceSampleRow>) {
        let report = analyze_distance_samples(&rows);
        (
            RouteDistancePerRunReport {
                file: file.to_string(),
                report,
            },
            rows,
        )
    }

    #[test]
    fn meta_three_plausible_runs_is_plausible() {
        let runs: Vec<_> = (0..3)
            .map(|i| build_per_run(&format!("run{i}.csv"), plausible_run_rows(i * 100_000, 99)))
            .collect();
        let per_runs: Vec<_> = runs.iter().map(|(r, _)| r.clone()).collect();
        let rows: Vec<_> = runs.into_iter().map(|(_, rows)| rows).collect();
        let meta = analyze_distance_meta(&per_runs, &rows, 3, 1_000_000);
        assert_eq!(meta.verdict, DistanceVerificationVerdict::Plausible);
        assert_eq!(meta.aggregate_metrics.plausible_count, 3);
        assert_eq!(meta.aggregate_metrics.run_count, 3);
    }

    #[test]
    fn meta_one_suspicious_run_is_suspicious() {
        let good = build_per_run("good.csv", plausible_run_rows(0, 1));
        let bad_rows: Vec<_> = (0..30)
            .map(|i| {
                row_from_distances(
                    i * 500,
                    1,
                    &[1000.0 + i as f32 * 100.0, 2000.0 + i as f32 * 100.0],
                )
            })
            .collect();
        let bad = build_per_run("bad.csv", bad_rows);
        let per_runs = vec![good.0, bad.0];
        let rows = vec![good.1, bad.1];
        let meta = analyze_distance_meta(&per_runs, &rows, 3, 1_000_000);
        assert_eq!(meta.verdict, DistanceVerificationVerdict::Suspicious);
        assert!(meta.reasons.iter().any(|r| r == "suspicious_runs_present"));
    }

    #[test]
    fn meta_too_few_runs_is_inconclusive() {
        let (run, rows) = build_per_run("only.csv", plausible_run_rows(0, 1));
        let meta = analyze_distance_meta(&[run], &[rows], 3, 1_000_000);
        assert_eq!(meta.verdict, DistanceVerificationVerdict::Inconclusive);
        assert!(meta.reasons.iter().any(|r| r == "insufficient_runs"));
    }

    #[test]
    fn meta_no_distance_run_triggers_suspicious_with_multiple_valid_runs() {
        let good = build_per_run("good.csv", plausible_run_rows(0, 1));
        let no_dist_rows: Vec<_> = (0..25)
            .map(|i| {
                let snap = RouteSnapshot {
                    sequence: i as u32,
                    route_hash: 1,
                    valid: true,
                    flags: 0,
                    bb_status: 0,
                    waypoints: vec![RouteWaypoint {
                        uid: 1,
                        ..Default::default()
                    }],
                    ..Default::default()
                };
                snapshot_to_sample_row(i * 500, &snap)
            })
            .collect();
        let empty = build_per_run("empty.csv", no_dist_rows);
        let per_runs = vec![good.0.clone(), empty.0.clone()];
        let rows = vec![good.1, empty.1];
        let meta = analyze_distance_meta(&per_runs, &rows, 2, 1_000_000);
        assert_eq!(meta.verdict, DistanceVerificationVerdict::Suspicious);
        assert_eq!(meta.aggregate_metrics.files_with_no_distance, 1);
    }

    #[test]
    fn meta_weighted_distance_coverage() {
        let half: Vec<_> = (0..20)
            .map(|i| {
                if i % 2 == 0 {
                    row_from_distances(i * 500, 1, &[1000.0, 500.0])
                } else {
                    row_from_distances(i * 500, 1, &[])
                }
            })
            .collect();
        let (run, rows) = build_per_run("half.csv", half);
        let meta = analyze_distance_meta(&[run], &[rows], 1, 0);
        assert!((meta.aggregate_metrics.weighted_percent_samples_with_distance - 50.0).abs() < 0.01);
    }

    #[test]
    fn meta_json_contains_per_run_and_aggregate() {
        let (run, rows) = build_per_run("r.csv", plausible_run_rows(0, 1));
        let meta = analyze_distance_meta(&[run], &[rows], 1, 42);
        let json = serde_json::to_string(&meta).unwrap();
        assert!(json.contains("\"per_run_reports\""));
        assert!(json.contains("\"aggregate_metrics\""));
        assert!(json.contains("\"generated_at_ms\""));
    }

    #[test]
    fn simple_glob_match_works() {
        assert!(simple_glob_match("*.csv", "a.csv"));
        assert!(!simple_glob_match("*.csv", "a.json"));
        assert!(simple_glob_match("run?.csv", "run1.csv"));
    }

    #[test]
    fn collect_meta_paths_skips_flags() {
        let args = vec![
            "prog".into(),
            "--json".into(),
            "out.json".into(),
            "--min-runs".into(),
            "3".into(),
            "logs/a.csv".into(),
            "logs/b.csv".into(),
        ];
        let paths = collect_meta_report_input_paths(&args).unwrap();
        assert_eq!(paths.len(), 2);
    }

    fn verify_from_named_runs(
        files_and_rows: Vec<(String, Vec<RouteDistanceSampleRow>)>,
        config: &RouteDistanceVerificationConfig,
    ) -> RouteDistanceVerificationReport {
        let mut per_runs = Vec::new();
        let mut rows_by_run = Vec::new();
        let mut run_scenarios = Vec::new();
        for (file, rows) in files_and_rows {
            let report = analyze_distance_samples(&rows);
            per_runs.push(RouteDistancePerRunReport {
                file: file.clone(),
                report,
            });
            run_scenarios.push(RouteDistanceRunScenarioInfo {
                file: file.clone(),
                scenario: detect_scenario_from_filename(&file),
                sidecar: None,
            });
            rows_by_run.push(rows);
        }
        analyze_distance_verification(&per_runs, &rows_by_run, &run_scenarios, config, 42)
    }

    #[test]
    fn detect_scenario_from_filename_tags() {
        assert_eq!(
            detect_scenario_from_filename("logs/autobahn_01.csv"),
            Some("autobahn".into())
        );
        assert_eq!(
            detect_scenario_from_filename("stadt-test.csv"),
            Some("stadt".into())
        );
        assert_eq!(detect_scenario_from_filename("unknown.csv"), None);
    }

    #[test]
    fn verify_five_plausible_with_required_scenarios_passes() {
        let files = [
            "autobahn_01.csv",
            "stadt_01.csv",
            "auffahrt_01.csv",
            "depot_01.csv",
            "pause_01.csv",
        ];
        let runs: Vec<_> = files
            .iter()
            .enumerate()
            .map(|(i, f)| (f.to_string(), plausible_run_rows(i as u64 * 100_000, 99 + i as u64)))
            .collect();
        let config = RouteDistanceVerificationConfig {
            min_runs: 5,
            required_scenarios: parse_required_scenarios("autobahn,stadt,auffahrt,depot"),
            ..Default::default()
        };
        let report = verify_from_named_runs(runs, &config);
        assert_eq!(report.verdict, RouteDistancePromotionVerdict::Passed);
        assert!(report.recommendation.contains("Eligible to promote"));
    }

    #[test]
    fn verify_three_runs_is_inconclusive() {
        let runs: Vec<_> = (0..3)
            .map(|i| {
                (
                    format!("autobahn_{i:02}.csv"),
                    plausible_run_rows(i as u64 * 100_000, 99),
                )
            })
            .collect();
        let report = verify_from_named_runs(runs, &RouteDistanceVerificationConfig::default());
        assert_eq!(report.verdict, RouteDistancePromotionVerdict::Inconclusive);
        assert!(report.reasons.iter().any(|r| r == "insufficient_runs"));
    }

    #[test]
    fn verify_suspicious_run_fails() {
        let bad_rows: Vec<_> = (0..30)
            .map(|i| {
                row_from_distances(
                    i * 500,
                    1,
                    &[1000.0 + i as f32 * 100.0, 2000.0 + i as f32 * 100.0],
                )
            })
            .collect();
        let mut runs: Vec<_> = (0..4)
            .map(|i| {
                (
                    format!("autobahn_{i:02}.csv"),
                    plausible_run_rows(i as u64 * 100_000, 99),
                )
            })
            .collect();
        runs.push(("bad.csv".into(), bad_rows));
        let report = verify_from_named_runs(runs, &RouteDistanceVerificationConfig::default());
        assert_eq!(report.verdict, RouteDistancePromotionVerdict::Failed);
        assert!(report.reasons.iter().any(|r| r == "suspicious_runs_present"));
    }

    #[test]
    fn verify_low_coverage_fails() {
        let sparse: Vec<_> = (0..25)
            .map(|i| {
                if i % 5 == 0 {
                    row_from_distances(i * 500, 1, &[1000.0, 500.0])
                } else {
                    row_from_distances(i * 500, 1, &[])
                }
            })
            .collect();
        let runs: Vec<_> = (0..5)
            .map(|i| {
                if i == 0 {
                    ("autobahn_01.csv".into(), sparse.clone())
                } else {
                    (
                        format!("stadt_{i:02}.csv"),
                        plausible_run_rows(i as u64 * 100_000, 99),
                    )
                }
            })
            .collect();
        let report = verify_from_named_runs(runs, &RouteDistanceVerificationConfig::default());
        assert_eq!(report.verdict, RouteDistancePromotionVerdict::Failed);
        assert!(report.reasons.iter().any(|r| r == "low_distance_coverage"));
    }

    #[test]
    fn verify_bad_monotonic_fails() {
        let bad_mono: Vec<_> = (0..25)
            .map(|i| {
                row_from_distances(
                    i * 500,
                    1,
                    &[1000.0 + i as f32 * 50.0, 2000.0 + i as f32 * 50.0],
                )
            })
            .collect();
        let runs: Vec<_> = (0..5)
            .map(|i| {
                if i == 0 {
                    ("autobahn_01.csv".into(), bad_mono.clone())
                } else {
                    (
                        format!("stadt_{i:02}.csv"),
                        plausible_run_rows(i as u64 * 100_000, 99),
                    )
                }
            })
            .collect();
        let report = verify_from_named_runs(runs, &RouteDistanceVerificationConfig::default());
        assert_eq!(report.verdict, RouteDistancePromotionVerdict::Failed);
        assert!(
            report
                .reasons
                .iter()
                .any(|r| r == "bad_monotonic_ratio_too_high")
        );
    }

    #[test]
    fn verify_missing_required_scenario_fails() {
        let runs: Vec<_> = (0..5)
            .map(|i| {
                (
                    format!("autobahn_{i:02}.csv"),
                    plausible_run_rows(i as u64 * 100_000, 99),
                )
            })
            .collect();
        let config = RouteDistanceVerificationConfig {
            required_scenarios: parse_required_scenarios("autobahn,stadt"),
            ..Default::default()
        };
        let report = verify_from_named_runs(runs, &config);
        assert_eq!(report.verdict, RouteDistancePromotionVerdict::Failed);
        assert!(
            report
                .reasons
                .iter()
                .any(|r| r == "missing_required_scenarios")
        );
    }

    #[test]
    fn verify_json_contains_verdict_scenarios_and_metrics() {
        let runs: Vec<_> = (0..5)
            .map(|i| {
                (
                    format!("autobahn_{i:02}.csv"),
                    plausible_run_rows(i as u64 * 100_000, 99),
                )
            })
            .collect();
        let report = verify_from_named_runs(runs, &RouteDistanceVerificationConfig::default());
        let json = serde_json::to_string(&report).unwrap();
        assert!(json.contains("\"verdict\""));
        assert!(json.contains("\"run_scenarios\""));
        assert!(json.contains("\"aggregate_metrics\""));
        assert!(json.contains("\"detected_scenarios\""));
    }

    #[test]
    fn verify_markdown_contains_recommendation_and_files() {
        let runs: Vec<_> = (0..5)
            .map(|i| {
                (
                    format!("autobahn_{i:02}.csv"),
                    plausible_run_rows(i as u64 * 100_000, 99),
                )
            })
            .collect();
        let report = verify_from_named_runs(runs, &RouteDistanceVerificationConfig::default());
        let md = format_distance_verification_markdown(&report);
        assert!(md.contains("Recommendation"));
        assert!(md.contains("autobahn_00.csv"));
        assert!(md.contains("Do not remove") || md.contains("Eligible to promote"));
    }
}
