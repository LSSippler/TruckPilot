use truckpilot_plugin_api::graph::RouterGraph;

const MIN_DEGREE: usize = 2;
/// Default activation frames (FIX 2: raised 3 → 6 to reject transient false
/// positives on straight multi-lane roads). Overridable per-detector.
pub const MIN_ACTIVATION_FRAMES: u32 = 6;
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

#[derive(Debug)]
pub struct JunctionDetector {
    junction_frames: u32,
    coast_remaining: u32,
    /// Consecutive positive detections required to activate (FIX 2: configurable;
    /// default [`MIN_ACTIVATION_FRAMES`]). Plugin sets this from truckpilot.toml.
    min_activation_frames: u32,
}

impl Default for JunctionDetector {
    fn default() -> Self {
        Self {
            junction_frames: 0,
            coast_remaining: 0,
            min_activation_frames: MIN_ACTIVATION_FRAMES,
        }
    }
}

impl JunctionDetector {
    /// Override the activation-frame threshold (loaded from config by the plugin).
    pub fn set_min_activation_frames(&mut self, frames: u32) {
        self.min_activation_frames = frames.max(1);
    }

    /// Current activation-frame threshold (for the diagnostic BB key).
    pub fn min_activation_frames(&self) -> u32 {
        self.min_activation_frames
    }

    /// Returns `(active, phase)`. Activation requires `min_activation_frames` consecutive
    /// positive detections; deactivation coasts for `COAST_FRAMES` before clearing.
    pub fn tick(&mut self, detection: &JunctionDetection) -> (bool, JunctionPhase) {
        if detection.is_junction {
            self.junction_frames = self.junction_frames.saturating_add(1);
            self.coast_remaining = COAST_FRAMES;
            if self.junction_frames >= self.min_activation_frames {
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

    /// DS13c: expose internal frame counter for diagnostic BB key.
    pub fn frames(&self) -> u32 {
        self.junction_frames
    }
}

// ---------------------------------------------------------------------------
// Detection function
// ---------------------------------------------------------------------------

/// Scans graph nodes within `radius_m` that lie in the forward driving cone and
/// returns whether a junction (out-degree ≥ 2 with heading spread > `spread_threshold_rad`)
/// is nearby.
///
/// FIX 2: the forward-cone filter is the single most important guard against
/// false positives on straight multi-lane roads — a junction node behind or to
/// the side of the truck no longer triggers detection.
///
/// * `truck_heading_rad` — truck heading in ETS2 convention (CW from North).
/// * `radius_m`          — scan radius (default 18 m).
/// * `spread_threshold_rad` — minimum edge heading spread to count as junction (default 50°).
/// * `cone_half_angle_rad`  — half-angle of the forward cone (default 60°).
pub fn detect_junction(
    graph: &RouterGraph,
    truck_x: f64,
    truck_z: f64,
    truck_heading_rad: f64,
    radius_m: f64,
    spread_threshold_rad: f64,
    cone_half_angle_rad: f64,
) -> JunctionDetection {
    let max_dist_sq = radius_m * radius_m;

    let mut candidates: Vec<(u64, f64)> = graph
        .nodes
        .iter()
        .filter_map(|&(uid, nx, nz)| {
            let dx = nx - truck_x;
            let dz = nz - truck_z;
            let dist_sq = dx * dx + dz * dz;
            if dist_sq > max_dist_sq {
                return None;
            }
            // Forward-cone gate (ETS2 convention: bearing = atan2(dx, -dz), CW from North).
            // A node sitting essentially on the truck (dist < 1 m) is treated as ahead.
            if dist_sq > 1.0 {
                let bearing = dx.atan2(-dz);
                let diff = wrap_pi(bearing - truck_heading_rad).abs();
                if diff > cone_half_angle_rad {
                    return None;
                }
            }
            Some((uid, dist_sq.sqrt()))
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

        if heading_spread(&out_headings) > spread_threshold_rad {
            junction_dist = Some(junction_dist.map_or(dist, |d: f64| d.min(dist)));
        }
    }

    JunctionDetection {
        is_junction: junction_dist.is_some(),
        max_degree,
        distance_m: junction_dist,
    }
}

/// Wrap an angle (radians) into `[-π, π]`.
fn wrap_pi(a: f64) -> f64 {
    let mut x = a % std::f64::consts::TAU;
    if x > std::f64::consts::PI {
        x -= std::f64::consts::TAU;
    } else if x < -std::f64::consts::PI {
        x += std::f64::consts::TAU;
    }
    x
}

/// Maximum pairwise angular difference among `headings` (wraparound-correct), in radians.
fn heading_spread(headings: &[f64]) -> f64 {
    let mut max = 0.0f64;
    for i in 0..headings.len() {
        for j in (i + 1)..headings.len() {
            let diff = (headings[i] - headings[j]).abs();
            let diff = if diff > std::f64::consts::PI {
                std::f64::consts::TAU - diff
            } else {
                diff
            };
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

    // FIX 2 live defaults mirrored from lib.rs (radius 18 m, spread 50°, cone ±60°).
    const TEST_RADIUS_M: f64 = 18.0;
    const TEST_SPREAD_RAD: f64 = 50.0 * std::f64::consts::PI / 180.0;
    const TEST_CONE_RAD: f64 = 60.0 * std::f64::consts::PI / 180.0;
    // All fixtures place the junction node East (+X) of the truck → truck faces East.
    const TEST_HEADING_EAST_RAD: f64 = std::f64::consts::FRAC_PI_2;

    /// Run `detect_junction` with the FIX 2 default parameters and an East-facing truck.
    fn detect_default(graph: &RouterGraph, truck_x: f64, truck_z: f64) -> JunctionDetection {
        detect_junction(
            graph,
            truck_x,
            truck_z,
            TEST_HEADING_EAST_RAD,
            TEST_RADIUS_M,
            TEST_SPREAD_RAD,
            TEST_CONE_RAD,
        )
    }

    /// Y-fork: node 1 at (50,0) has 2 exits — one left, one right (~90° spread).
    fn make_y_fork() -> RouterGraph {
        let nodes = vec![
            (0u64, 0.0, 0.0),
            (1, 50.0, 0.0),
            (2, 80.0, 30.0),
            (3, 80.0, -30.0),
        ];
        let edges = vec![(0, 1, 50.0), (1, 2, 36.0), (1, 3, 36.0)];
        RouterGraph::new(nodes, edges)
    }

    /// Straight road: node 1 at (50,0) has 2 exits only 10° apart — not a real junction.
    fn make_near_parallel() -> RouterGraph {
        // dz = 30 * tan(5°) ≈ 2.625 per side → 10° total spread
        let nodes = vec![
            (0u64, 0.0, 0.0),
            (1, 50.0, 0.0),
            (2, 80.0, 2.63),
            (3, 80.0, -2.63),
        ];
        let edges = vec![(0, 1, 50.0), (1, 2, 30.1), (1, 3, 30.1)];
        RouterGraph::new(nodes, edges)
    }

    #[test]
    fn p31_y_fork_detected_after_hysteresis() {
        let graph = make_y_fork();
        // Truck 5m west of junction node, facing East toward it.
        let det = detect_default(&graph, 45.0, 0.0);
        assert!(det.is_junction, "Y-fork should be detected");
        assert_eq!(det.max_degree, 2);
        assert!(det.distance_m.is_some());

        // FIX 2: activation now needs MIN_ACTIVATION_FRAMES (6) consecutive frames.
        let mut detector = JunctionDetector::default();
        for tick in 1..MIN_ACTIVATION_FRAMES {
            let (a, p) = detector.tick(&det);
            assert!(!a, "tick {tick}: not yet active");
            assert_eq!(p, JunctionPhase::None);
        }
        // Final activation tick: stable.
        let (a, p) = detector.tick(&det);
        assert!(a, "tick {MIN_ACTIVATION_FRAMES}: should be active");
        assert_eq!(p, JunctionPhase::Approaching);
    }

    #[test]
    fn p31_no_junction_below_spread_threshold() {
        let graph = make_near_parallel();
        let det = detect_default(&graph, 45.0, 0.0);
        assert!(
            !det.is_junction,
            "near-parallel exits (10° spread) must not be detected as junction"
        );
    }

    #[test]
    fn p31_hysteresis_coast_holds_active() {
        let graph = make_y_fork();
        let det_on = detect_default(&graph, 45.0, 0.0);
        let det_off = JunctionDetection {
            is_junction: false,
            max_degree: 0,
            distance_m: None,
        };

        let mut detector = JunctionDetector::default();
        // Stabilise (MIN_ACTIVATION_FRAMES active frames)
        for _ in 0..MIN_ACTIVATION_FRAMES {
            detector.tick(&det_on);
        }
        // 5 coast frames must all return (true, Exiting)
        for i in 1..=5 {
            let (a, p) = detector.tick(&det_off);
            assert!(a, "coast tick {i}: should still be active");
            assert_eq!(
                p,
                JunctionPhase::Exiting,
                "coast tick {i}: phase must be Exiting"
            );
        }
        // 6th tick: coast exhausted
        let (a, p) = detector.tick(&det_off);
        assert!(!a, "after coast exhausted: must be inactive");
        assert_eq!(p, JunctionPhase::None);
    }

    #[test]
    fn p31_heading_spread_50deg_boundary() {
        // FIX 2: spread threshold raised 30° → 50°.
        // Exactly 50° spread: NOT detected (threshold is STRICT >).
        // Exits at +25° and -25° from the x-axis: tan(25°)*30 ≈ 13.989
        let nodes_50 = vec![
            (0u64, 0.0, 0.0),
            (1, 50.0, 0.0),
            (2, 80.0, 13.989),
            (3, 80.0, -13.989),
        ];
        let edges_50 = vec![(0, 1, 50.0), (1, 2, 30.0), (1, 3, 30.0)];
        let graph_50 = RouterGraph::new(nodes_50, edges_50);
        let det = detect_default(&graph_50, 45.0, 0.0);
        assert!(
            !det.is_junction,
            "exactly 50° spread must NOT be detected (strict >)"
        );

        // Just over 50° spread: exits at +26° and -26°: tan(26°)*30 ≈ 14.631
        let nodes_51 = vec![
            (0u64, 0.0, 0.0),
            (1, 50.0, 0.0),
            (2, 80.0, 14.631),
            (3, 80.0, -14.631),
        ];
        let edges_51 = vec![(0, 1, 50.0), (1, 2, 30.0), (1, 3, 30.0)];
        let graph_51 = RouterGraph::new(nodes_51, edges_51);
        let det = detect_default(&graph_51, 45.0, 0.0);
        assert!(det.is_junction, "spread just over 50° must be detected");
    }

    /// FIX 2: a real Y-fork that lies BEHIND the truck (opposite the heading) must
    /// NOT be detected — the forward-cone filter is the key guard against false
    /// positives on straight multi-lane roads where side/rear nodes used to trigger.
    #[test]
    fn fix2_junction_behind_truck_not_detected() {
        let graph = make_y_fork();
        // Junction node is East (+X) of the truck; truck faces West → node is behind.
        let det = detect_junction(
            &graph,
            45.0,
            0.0,
            -std::f64::consts::FRAC_PI_2, // facing West
            TEST_RADIUS_M,
            TEST_SPREAD_RAD,
            TEST_CONE_RAD,
        );
        assert!(
            !det.is_junction,
            "junction behind the truck must be filtered by the forward cone"
        );
    }
}
