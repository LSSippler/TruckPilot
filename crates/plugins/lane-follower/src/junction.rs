use truckpilot_plugin_api::graph::RouterGraph;

const SNAP_RADIUS_M: f64 = 25.0;
const MIN_DEGREE: usize = 2;
const HEADING_SPREAD_THRESHOLD_RAD: f64 = std::f64::consts::PI / 6.0; // 30°
const MIN_ACTIVATION_FRAMES: u32 = 3;
const COAST_FRAMES: u32 = 5;

// ---------------------------------------------------------------------------
// Detection result
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct JunctionDetection {
    pub is_junction: bool,
    pub max_degree: usize,
    pub distance_m: Option<f64>,
}

// ---------------------------------------------------------------------------
// Phase
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum JunctionPhase {
    #[default]
    None,
    Approaching,
    Exiting,
}

impl JunctionPhase {
    pub fn as_str(self) -> &'static str {
        match self {
            JunctionPhase::None => "none",
            JunctionPhase::Approaching => "approaching",
            JunctionPhase::Exiting => "exiting",
        }
    }
}

// ---------------------------------------------------------------------------
// Hysteresis detector
// ---------------------------------------------------------------------------

#[derive(Debug, Default)]
pub struct JunctionDetector {
    junction_frames: u32,
    coast_remaining: u32,
}

impl JunctionDetector {
    /// Returns `(active, phase)`. Activation requires `MIN_ACTIVATION_FRAMES` consecutive
    /// positive detections; deactivation coasts for `COAST_FRAMES` before clearing.
    pub fn tick(&mut self, detection: &JunctionDetection) -> (bool, JunctionPhase) {
        if detection.is_junction {
            self.junction_frames = self.junction_frames.saturating_add(1);
            self.coast_remaining = COAST_FRAMES;
            if self.junction_frames >= MIN_ACTIVATION_FRAMES {
                (true, JunctionPhase::Approaching)
            } else {
                (false, JunctionPhase::None)
            }
        } else if self.coast_remaining > 0 {
            self.coast_remaining -= 1;
            (true, JunctionPhase::Exiting)
        } else {
            self.junction_frames = 0;
            (false, JunctionPhase::None)
        }
    }

    pub fn reset(&mut self) {
        self.junction_frames = 0;
        self.coast_remaining = 0;
    }
}

// ---------------------------------------------------------------------------
// Detection function
// ---------------------------------------------------------------------------

/// Scans graph nodes within `SNAP_RADIUS_M` and returns whether a junction
/// (out-degree ≥ 2 with heading spread > 30°) is nearby.
pub fn detect_junction(graph: &RouterGraph, truck_x: f64, truck_z: f64) -> JunctionDetection {
    let max_dist_sq = SNAP_RADIUS_M * SNAP_RADIUS_M;

    let mut candidates: Vec<(u64, f64)> = graph
        .nodes
        .iter()
        .filter_map(|&(uid, nx, nz)| {
            let dx = nx - truck_x;
            let dz = nz - truck_z;
            let dist_sq = dx * dx + dz * dz;
            if dist_sq <= max_dist_sq {
                Some((uid, dist_sq.sqrt()))
            } else {
                None
            }
        })
        .collect();

    candidates.sort_by(|a, b| a.1.total_cmp(&b.1));

    let mut max_degree = 0usize;
    let mut junction_dist: Option<f64> = None;

    for &(uid, dist) in candidates.iter().take(3) {
        let out_headings: Vec<f64> = graph
            .edges
            .iter()
            .filter(|&&(from, _, _)| from == uid)
            .filter_map(|&(_, to, _)| {
                let &(nx, nz) = graph.positions.get(&uid)?;
                let &(tx, tz) = graph.positions.get(&to)?;
                Some((tz - nz).atan2(tx - nx))
            })
            .collect();

        let degree = out_headings.len();
        if degree < MIN_DEGREE {
            continue;
        }
        max_degree = max_degree.max(degree);

        if heading_spread(&out_headings) > HEADING_SPREAD_THRESHOLD_RAD {
            junction_dist = Some(junction_dist.map_or(dist, |d: f64| d.min(dist)));
        }
    }

    JunctionDetection {
        is_junction: junction_dist.is_some(),
        max_degree,
        distance_m: junction_dist,
    }
}

/// Maximum pairwise angular difference among `headings` (wraparound-correct), in radians.
fn heading_spread(headings: &[f64]) -> f64 {
    let mut max = 0.0f64;
    for i in 0..headings.len() {
        for j in (i + 1)..headings.len() {
            let diff = (headings[i] - headings[j]).abs();
            let diff = if diff > std::f64::consts::PI { std::f64::consts::TAU - diff } else { diff };
            if diff > max {
                max = diff;
            }
        }
    }
    max
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use truckpilot_plugin_api::graph::RouterGraph;

    /// Y-fork: node 1 at (50,0) has 2 exits — one left, one right (~90° spread).
    fn make_y_fork() -> RouterGraph {
        let nodes = vec![(0u64, 0.0, 0.0), (1, 50.0, 0.0), (2, 80.0, 30.0), (3, 80.0, -30.0)];
        let edges = vec![(0, 1, 50.0), (1, 2, 36.0), (1, 3, 36.0)];
        RouterGraph::new(nodes, edges)
    }

    /// Straight road: node 1 at (50,0) has 2 exits only 10° apart — not a real junction.
    fn make_near_parallel() -> RouterGraph {
        // dz = 30 * tan(5°) ≈ 2.625 per side → 10° total spread
        let nodes = vec![(0u64, 0.0, 0.0), (1, 50.0, 0.0), (2, 80.0, 2.63), (3, 80.0, -2.63)];
        let edges = vec![(0, 1, 50.0), (1, 2, 30.1), (1, 3, 30.1)];
        RouterGraph::new(nodes, edges)
    }

    #[test]
    fn p31_y_fork_detected_after_hysteresis() {
        let graph = make_y_fork();
        // Truck 5m west of junction node
        let det = detect_junction(&graph, 45.0, 0.0);
        assert!(det.is_junction, "Y-fork should be detected");
        assert_eq!(det.max_degree, 2);
        assert!(det.distance_m.is_some());

        let mut detector = JunctionDetector::default();
        // Ticks 1–2: not yet stable
        let (a, p) = detector.tick(&det);
        assert!(!a, "tick 1: not yet active");
        assert_eq!(p, JunctionPhase::None);
        let (a, p) = detector.tick(&det);
        assert!(!a, "tick 2: not yet active");
        assert_eq!(p, JunctionPhase::None);
        // Tick 3: stable
        let (a, p) = detector.tick(&det);
        assert!(a, "tick 3: should be active");
        assert_eq!(p, JunctionPhase::Approaching);
    }

    #[test]
    fn p31_no_junction_below_spread_threshold() {
        let graph = make_near_parallel();
        let det = detect_junction(&graph, 45.0, 0.0);
        assert!(!det.is_junction, "near-parallel exits (10° spread) must not be detected as junction");
    }

    #[test]
    fn p31_hysteresis_coast_holds_active() {
        let graph = make_y_fork();
        let det_on = detect_junction(&graph, 45.0, 0.0);
        let det_off = JunctionDetection { is_junction: false, max_degree: 0, distance_m: None };

        let mut detector = JunctionDetector::default();
        // Stabilise (3 active frames)
        for _ in 0..3 {
            detector.tick(&det_on);
        }
        // 5 coast frames must all return (true, Exiting)
        for i in 1..=5 {
            let (a, p) = detector.tick(&det_off);
            assert!(a, "coast tick {i}: should still be active");
            assert_eq!(p, JunctionPhase::Exiting, "coast tick {i}: phase must be Exiting");
        }
        // 6th tick: coast exhausted
        let (a, p) = detector.tick(&det_off);
        assert!(!a, "after coast exhausted: must be inactive");
        assert_eq!(p, JunctionPhase::None);
    }

    #[test]
    fn p31_heading_spread_30deg_boundary() {
        // Exactly 30° spread: NOT detected (threshold is STRICT >)
        // Exits at +15° and -15° from the x-axis:
        // tan(15°)*30 ≈ 8.036
        let nodes_30 = vec![(0u64, 0.0, 0.0), (1, 50.0, 0.0), (2, 80.0, 8.036), (3, 80.0, -8.036)];
        let edges_30 = vec![(0, 1, 50.0), (1, 2, 30.0), (1, 3, 30.0)];
        let graph_30 = RouterGraph::new(nodes_30, edges_30);
        let det = detect_junction(&graph_30, 45.0, 0.0);
        assert!(!det.is_junction, "exactly 30° spread must NOT be detected (strict >)");

        // Just over 30° spread: exits at +16° and -16°:
        // tan(16°)*30 ≈ 8.594
        let nodes_31 = vec![(0u64, 0.0, 0.0), (1, 50.0, 0.0), (2, 80.0, 8.594), (3, 80.0, -8.594)];
        let edges_31 = vec![(0, 1, 50.0), (1, 2, 30.0), (1, 3, 30.0)];
        let graph_31 = RouterGraph::new(nodes_31, edges_31);
        let det = detect_junction(&graph_31, 45.0, 0.0);
        assert!(det.is_junction, "spread just over 30° must be detected");
    }
}
