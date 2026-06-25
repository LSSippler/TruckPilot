use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, HashSet};

#[derive(Clone)]
pub struct RouterGraph {
    pub nodes: Vec<(u64, f64, f64)>,
    pub edges: Vec<(u64, u64, f64)>,
    pub positions: HashMap<u64, (f64, f64)>,
    nodes_with_edges: HashSet<u64>,
}

impl RouterGraph {
    pub fn new(nodes: Vec<(u64, f64, f64)>, edges: Vec<(u64, u64, f64)>) -> Self {
        let positions: HashMap<u64, (f64, f64)> =
            nodes.iter().map(|&(uid, x, z)| (uid, (x, z))).collect();
        let nodes_with_edges: HashSet<u64> =
            edges.iter().flat_map(|&(from, to, _)| [from, to]).collect();
        Self {
            nodes,
            edges,
            positions,
            nodes_with_edges,
        }
    }

    /// Returns `true` when `uid` is a known graph node (with or without edges).
    pub fn has_node(&self, uid: u64) -> bool {
        self.positions.contains_key(&uid)
    }

    /// World `(x, z)` for a graph node, if known.
    pub fn node_position(&self, uid: u64) -> Option<(f64, f64)> {
        self.positions.get(&uid).copied()
    }

    pub fn find_nearest_geometric(&self, x: f64, z: f64, max_dist_m: f64) -> Option<(u64, f64)> {
        let max_dist_sq = max_dist_m * max_dist_m;
        self.nodes
            .iter()
            .filter(|&&(uid, _, _)| self.nodes_with_edges.contains(&uid))
            .filter_map(|&(uid, nx, nz)| {
                let dx = nx - x;
                let dz = nz - z;
                let dist_sq = dx * dx + dz * dz;
                if dist_sq <= max_dist_sq {
                    Some((uid, dist_sq.sqrt()))
                } else {
                    None
                }
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
    }

    /// Find the nearest graph node within `max_dist_m`, preferring nodes whose
    /// outgoing edges align with the truck's forward direction.
    ///
    /// * `heading` — ETS2 raw heading in `[0..1]` CCW from North (as returned by
    ///   `telemetry.heading`). **Not radians.**
    pub fn find_nearest_with_heading(
        &self,
        x: f64,
        z: f64,
        heading: f64,
        max_dist_m: f64,
    ) -> Option<(u64, f64, bool)> {
        // ETS2: 0..1 CCW from North. Convert to forward vector in (x,z) world space.
        let heading_rad = -heading * std::f64::consts::TAU;
        let hx = heading_rad.sin();
        let hz = -heading_rad.cos();
        let max_dist_sq = max_dist_m * max_dist_m;

        let candidates: Vec<(u64, f64, f64, f64)> = self
            .nodes
            .iter()
            .filter(|&&(uid, _, _)| self.nodes_with_edges.contains(&uid))
            .filter_map(|&(uid, nx, nz)| {
                let dx = nx - x;
                let dz = nz - z;
                let dist_sq = dx * dx + dz * dz;
                if dist_sq <= max_dist_sq {
                    Some((uid, nx, nz, dist_sq.sqrt()))
                } else {
                    None
                }
            })
            .collect();

        if candidates.is_empty() {
            return None;
        }

        let filtered: Vec<(u64, f64)> = candidates
            .iter()
            .filter(|&&(uid, nx, nz, _)| {
                self.edges.iter().any(|&(from, to, _)| {
                    if from != uid {
                        return false;
                    }
                    if let Some(&(tx, tz)) = self.positions.get(&to) {
                        let ex = tx - nx;
                        let ez = tz - nz;
                        let len = (ex * ex + ez * ez).sqrt();
                        if len < 1.0 {
                            return false;
                        }
                        ex / len * hx + ez / len * hz >= 0.5
                    } else {
                        false
                    }
                })
            })
            .map(|&(uid, _, _, dist)| (uid, dist))
            .collect();

        if filtered.is_empty() {
            candidates
                .into_iter()
                .min_by(|a, b| a.3.total_cmp(&b.3))
                .map(|(uid, _, _, dist)| (uid, dist, false))
        } else {
            filtered
                .into_iter()
                .min_by(|a, b| a.1.total_cmp(&b.1))
                .map(|(uid, dist)| (uid, dist, true))
        }
    }

    /// Find the best start node by projecting the truck position onto the nearest graph edge.
    ///
    /// Projects the truck's `(x, z)` onto every edge via point-to-segment math,
    /// finds the closest edge within `max_dist_m`, then picks the **FROM** node
    /// of the aligned edge as the A* start. Using `from_uid` (not `to_uid`) is
    /// correct because A* traverses `from → to` in the forward direction; starting
    /// from `to_uid` would make the only outgoing edge go backwards.
    ///
    /// Edges pointing **>120° against travel** (`dot < -0.5`) are rejected as
    /// snap candidates: on a divided highway the opposing carriageway is often
    /// the geometrically nearest edge, and snapping to it makes A* plan the
    /// whole route backwards (~180° heading mismatch). If *every* in-radius edge
    /// is rejected (genuine dead-end, or truck genuinely facing against a
    /// one-way), the best unfiltered edge is returned as a fallback — so a truck
    /// that previously got a (wrong-direction) route never loses its route
    /// entirely. The returned bool is `true` when the heading gate selected the
    /// edge, `false` when the unfiltered fallback was used.
    ///
    /// Returns `Some((uid, dist_to_edge, heading_filter_applied, rejected_by_heading))`
    /// on success; `None` if no edge is within `max_dist_m`.
    ///
    /// `rejected_by_heading` counts in-radius edge candidates discarded because
    /// their direction is >120° against travel (`dot < -0.5`).
    pub fn find_nearest_on_edge(
        &self,
        x: f64,
        z: f64,
        heading: f64,
        max_dist_m: f64,
    ) -> Option<(u64, f64, bool, u32)> {
        let heading_rad = -heading * std::f64::consts::TAU;
        let hx = heading_rad.sin();
        let hz = -heading_rad.cos();

        // Best edge whose direction is not strongly opposed to travel.
        let mut best_dist = f64::MAX;
        let mut best_uid: Option<u64> = None;
        // Best edge ignoring the heading-rejection gate. Used only when the gate
        // leaves no candidate, so the truck keeps *some* start node.
        let mut fallback_dist = f64::MAX;
        let mut fallback_uid: Option<u64> = None;
        let mut rejected_by_heading = 0u32;

        for &(from_uid, to_uid, _) in &self.edges {
            let Some(&(fx, fz)) = self.positions.get(&from_uid) else {
                continue;
            };
            let Some(&(tx, tz)) = self.positions.get(&to_uid) else {
                continue;
            };
            let ex = tx - fx;
            let ez = tz - fz;
            let len_sq = ex * ex + ez * ez;
            if len_sq < 0.01 {
                continue;
            }
            // Project truck onto segment, clamped to [0, 1].
            let t = ((x - fx) * ex + (z - fz) * ez) / len_sq;
            let t = t.clamp(0.0, 1.0);
            let px = fx + t * ex;
            let pz = fz + t * ez;
            let dist = ((x - px) * (x - px) + (z - pz) * (z - pz)).sqrt();
            if dist > max_dist_m {
                continue;
            }
            // Alignment of the edge's intrinsic direction with truck heading.
            let len = len_sq.sqrt();
            let dot = ex / len * hx + ez / len * hz;
            // A* routes from → to, so starting from `from_uid` on an aligned
            // edge lets A* traverse the edge forward. For an opposed edge the
            // "forward" segment runs the other way, so its from_uid is `to_uid`.
            let routing_node = if dot >= 0.0 { from_uid } else { to_uid };
            // Fallback always uses from_uid (graph-ordered start; broadest A* reach).
            let fallback_node = from_uid;

            // Unfiltered fallback tracker (first-seen wins ties via strict `<`).
            if dist < fallback_dist {
                fallback_dist = dist;
                fallback_uid = Some(fallback_node);
            }
            // Reject edges pointing >120° against travel (opposing carriageway).
            if dot < -0.5 {
                rejected_by_heading = rejected_by_heading.saturating_add(1);
                continue;
            }
            if dist < best_dist {
                best_dist = dist;
                best_uid = Some(routing_node);
            }
        }

        match best_uid {
            Some(uid) => Some((uid, best_dist, true, rejected_by_heading)),
            None => fallback_uid.map(|uid| (uid, fallback_dist, false, rejected_by_heading)),
        }
    }

    pub fn plan(&self, start: u64, goal: u64) -> Option<(Vec<u64>, f64)> {
        if !self.positions.contains_key(&start) || !self.positions.contains_key(&goal) {
            return None;
        }

        let mut adj: HashMap<u64, Vec<(u64, f64)>> = HashMap::new();
        for &(from, to, dist) in &self.edges {
            adj.entry(from).or_default().push((to, dist));
        }

        let goal_pos = *self.positions.get(&goal).unwrap();
        let start_pos = *self.positions.get(&start).unwrap();

        let mut open: BinaryHeap<Reverse<HeapEntry>> = BinaryHeap::new();
        let mut g: HashMap<u64, f64> = HashMap::new();
        let mut came_from: HashMap<u64, u64> = HashMap::new();
        let mut closed: HashSet<u64> = HashSet::new();

        g.insert(start, 0.0);
        open.push(Reverse(HeapEntry {
            uid: start,
            f: heuristic(start_pos, goal_pos),
        }));

        while let Some(Reverse(entry)) = open.pop() {
            if entry.uid == goal {
                let total_dist = *g.get(&goal).unwrap_or(&0.0);
                return Some((reconstruct(&came_from, start, goal), total_dist));
            }
            if !closed.insert(entry.uid) {
                continue;
            }
            for &(nb, cost) in adj.get(&entry.uid).into_iter().flatten() {
                if closed.contains(&nb) {
                    continue;
                }
                let tg = g[&entry.uid] + cost;
                if tg < *g.get(&nb).unwrap_or(&f64::MAX) {
                    came_from.insert(nb, entry.uid);
                    g.insert(nb, tg);
                    let h = heuristic(*self.positions.get(&nb).unwrap(), goal_pos);
                    open.push(Reverse(HeapEntry { uid: nb, f: tg + h }));
                }
            }
        }
        None
    }
}

#[derive(Clone)]
struct HeapEntry {
    uid: u64,
    f: f64,
}

impl PartialEq for HeapEntry {
    fn eq(&self, o: &Self) -> bool {
        self.f.total_cmp(&o.f).is_eq() && self.uid == o.uid
    }
}
impl Eq for HeapEntry {}
impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for HeapEntry {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        self.f.total_cmp(&o.f).then_with(|| self.uid.cmp(&o.uid))
    }
}

fn heuristic(pos: (f64, f64), goal: (f64, f64)) -> f64 {
    let dx = pos.0 - goal.0;
    let dz = pos.1 - goal.1;
    (dx * dx + dz * dz).sqrt()
}

fn reconstruct(came_from: &HashMap<u64, u64>, start: u64, goal: u64) -> Vec<u64> {
    let mut path = vec![goal];
    let mut cur = goal;
    while cur != start {
        if let Some(&prev) = came_from.get(&cur) {
            path.push(prev);
            cur = prev;
        } else {
            break;
        }
    }
    path.reverse();
    path
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Edge-snap: truck midway on a 200m highway segment (too far from any node for
    /// the old 20m node-snap) — edge-snap must succeed with the direction-correct node.
    #[test]
    fn edge_snap_midpoint_highway() {
        // Two nodes 200m apart along x-axis.
        let nodes: Vec<(u64, f64, f64)> = vec![(1, 0.0, 0.0), (2, 200.0, 0.0)];
        let edges: Vec<(u64, u64, f64)> = vec![(1, 2, 200.0)];
        let graph = RouterGraph::new(nodes, edges);

        // Truck at (100, 5): 5m beside the edge, 100m from both nodes.
        // Node-snap with 20m would fail; edge-snap with 100m must succeed.
        // Heading = 0.0 → ETS2 North, heading_rad=0, hx=0, hz=-1 — perpendicular to edge.
        // Try heading = 0.75 (East in ETS2: heading_rad=-0.75*TAU, sin≈1, cos≈0) → hx≈1, hz≈0.
        let result = graph.find_nearest_on_edge(100.0, 5.0, 0.75, 100.0);
        assert!(
            result.is_some(),
            "edge-snap must find the edge when truck is 5m off it"
        );
        let (uid, dist, edge_used, _rejected) = result.unwrap();
        assert!(edge_used, "must return edge_used=true");
        assert!(
            dist < 6.0,
            "distance to edge must be near 5m, got {dist:.2}"
        );
        // Heading east → from-node (uid=1) is the A* start for forward traversal.
        assert_eq!(
            uid, 1,
            "heading east along edge → from-node (uid=1) must be chosen so A* routes forward"
        );

        // Node-snap (20m) must fail at this position.
        let node_result = graph.find_nearest_with_heading(100.0, 5.0, 0.75, 20.0);
        assert!(
            node_result.is_none(),
            "node-snap with 20m must fail when both nodes are 100m away"
        );
    }

    /// Divided highway: the opposing carriageway edge is geometrically *closer*
    /// to the truck (1 m vs 2 m) but points ~180° against travel. The heading
    /// gate must reject it and snap to the aligned carriageway's forward node.
    #[test]
    fn nearest_on_edge_rejects_opposite() {
        // Aligned carriageway: 1 -> 2 along +X (East).  Opposing: 3 -> 4 along -X,
        // 3 m north (−z) of the truck so it projects 1 m closer.
        let nodes: Vec<(u64, f64, f64)> = vec![
            (1, 0.0, 0.0),
            (2, 200.0, 0.0),
            (3, 200.0, -3.0),
            (4, 0.0, -3.0),
        ];
        let edges: Vec<(u64, u64, f64)> = vec![(1, 2, 200.0), (3, 4, 200.0)];
        let graph = RouterGraph::new(nodes, edges);

        // Truck mid-road at (100, -2), heading 0.75 (ETS2 East → forward (+1, 0)).
        // Opposing edge (z=-3) is 1 m away, aligned edge (z=0) is 2 m away.
        let (uid, _dist, filter_used, rejected) = graph
            .find_nearest_on_edge(100.0, -2.0, 0.75, 100.0)
            .expect("an aligned edge is within range");
        assert_eq!(
            uid, 1,
            "must snap to the aligned carriageway's from-node (1), not the closer opposing edge"
        );
        assert_ne!(uid, 3, "must not pick the opposing edge's from-node");
        assert!(filter_used, "heading gate selected the edge → flag true");
        assert!(
            rejected >= 1,
            "opposing carriageway edge must be counted as rejected_by_heading, got {rejected}"
        );
    }

    /// Sanity: an edge pointing the truck's way is selected unchanged.
    #[test]
    fn nearest_on_edge_keeps_aligned() {
        let nodes: Vec<(u64, f64, f64)> = vec![(1, 0.0, 0.0), (2, 200.0, 0.0)];
        let edges: Vec<(u64, u64, f64)> = vec![(1, 2, 200.0)];
        let graph = RouterGraph::new(nodes, edges);

        // Truck 2 m beside the edge, heading East along it.
        let (uid, dist, filter_used, _rejected) = graph
            .find_nearest_on_edge(100.0, 2.0, 0.75, 100.0)
            .expect("edge in range");
        assert_eq!(uid, 1, "heading East → from-node (1) for forward A* traversal");
        assert!(dist < 2.5, "distance ≈ 2 m, got {dist:.2}");
        assert!(filter_used);
    }

    /// A perpendicular (~90°) edge — e.g. a cross street or offset ramp — has
    /// `dot ≈ 0`, comfortably above the −0.5 reject threshold, so it must NOT be
    /// rejected.
    #[test]
    fn nearest_on_edge_curve_tolerance() {
        // Edge 1 -> 2 runs South (0, +1); truck heads East → dot = 0.
        let nodes: Vec<(u64, f64, f64)> = vec![(1, 0.0, 0.0), (2, 0.0, 200.0)];
        let edges: Vec<(u64, u64, f64)> = vec![(1, 2, 200.0)];
        let graph = RouterGraph::new(nodes, edges);

        let result = graph.find_nearest_on_edge(2.0, 100.0, 0.75, 100.0);
        assert!(
            result.is_some(),
            "a 90° edge (dot=0 > -0.5) must not be rejected"
        );
        let (uid, _dist, filter_used, rejected) = result.unwrap();
        assert_eq!(uid, 1, "dot >= 0 → from-node (A* start)");
        assert!(filter_used, "selected by the gate, not the fallback");
        assert_eq!(rejected, 0, "90° edge must not increment rejected_by_heading");
    }

    /// Truck genuinely faces against the only nearby edge (one-way, dead-end):
    /// the gate rejects every candidate, so the unfiltered fallback must still
    /// return a node — never `None` where the pre-fix code returned `Some`.
    #[test]
    fn nearest_on_edge_fallback_when_all_rejected() {
        // Only edge 1 -> 2 points West; truck heads East → dot = -1, rejected.
        let nodes: Vec<(u64, f64, f64)> = vec![(1, 200.0, 0.0), (2, 0.0, 0.0)];
        let edges: Vec<(u64, u64, f64)> = vec![(1, 2, 200.0)];
        let graph = RouterGraph::new(nodes, edges);

        let result = graph.find_nearest_on_edge(100.0, 2.0, 0.75, 100.0);
        assert!(
            result.is_some(),
            "fallback must keep a route — no None where pre-fix returned Some"
        );
        let (_uid, _dist, filter_used, rejected) = result.unwrap();
        assert!(
            !filter_used,
            "unfiltered fallback was used → heading_filter_applied=false"
        );
        assert_eq!(
            rejected, 1,
            "single opposing edge must be counted as rejected_by_heading"
        );
    }

    #[test]
    fn heading_south_ets2_accepts_south_pointing_edge() {
        // ETS2 heading=0.5 = South; forward vector = (0, +1) in (x,z).
        // With the old radians interpretation sin(0.5)≈0.48, -cos(0.5)≈-0.88
        // would point NE-ish and reject the south edge → filter_used=false.
        // With the correct ETS2 conversion: heading_rad=-π → hx=0, hz=1 → accept.
        let nodes: Vec<(u64, f64, f64)> = vec![(1, 0.0, 0.0), (2, 0.0, 100.0)];
        let edges: Vec<(u64, u64, f64)> = vec![(1, 2, 100.0)];
        let graph = RouterGraph::new(nodes, edges);
        // Truck 5 m north of node 1, heading=0.5 (ETS2 South).
        let result = graph.find_nearest_with_heading(0.0, -5.0, 0.5, 20.0);
        assert!(result.is_some(), "should find node 1");
        let (uid, _, filter_used) = result.unwrap();
        assert_eq!(uid, 1, "should snap to node 1");
        assert!(
            filter_used,
            "ETS2 heading=0.5 (South) must accept south-pointing edge (filter_used=true)"
        );
    }

    /// Fix regression: aligned edge must return from_uid so A* starts at the
    /// segment origin and traverses from→to (forward direction).
    #[test]
    fn snap_from_node_for_a_star_alignment() {
        let nodes: Vec<(u64, f64, f64)> = vec![(1, 0.0, 0.0), (2, 100.0, 0.0)];
        let edges: Vec<(u64, u64, f64)> = vec![(1, 2, 100.0)];
        let graph = RouterGraph::new(nodes, edges);
        // Truck 5m south of midpoint, heading East (0.75 ETS2 → hx≈1, hz≈0, dot≈+1).
        let (uid, _, filter_used, _) = graph
            .find_nearest_on_edge(50.0, 5.0, 0.75, 20.0)
            .expect("edge must be found");
        assert_eq!(uid, 1, "aligned edge → from_uid=1 so A* routes 1→2 forward");
        assert!(filter_used, "heading gate must accept the east-pointing edge");
    }

    /// Divided highway: the aligned carriageway's from_uid is chosen; the
    /// opposing carriageway is rejected; rejected_by_heading is incremented.
    #[test]
    fn snap_returns_from_node_for_opposed_aligned_pair() {
        // Edge 1→2 East at z=0; Edge 3→4 West at z=4 (opposing carriageway).
        let nodes: Vec<(u64, f64, f64)> = vec![
            (1, 0.0, 0.0),
            (2, 100.0, 0.0),
            (3, 100.0, 4.0),
            (4, 0.0, 4.0),
        ];
        let edges: Vec<(u64, u64, f64)> = vec![(1, 2, 100.0), (3, 4, 100.0)];
        let graph = RouterGraph::new(nodes, edges);
        // Truck between carriageways, heading East.
        let (uid, _, filter_used, rejected) = graph
            .find_nearest_on_edge(50.0, 2.0, 0.75, 20.0)
            .expect("aligned carriageway must be found");
        assert_eq!(uid, 1, "from_uid of the east carriageway must be chosen");
        assert!(filter_used, "heading gate selected the aligned edge");
        assert!(
            rejected >= 1,
            "opposing carriageway must be counted as rejected_by_heading, got {rejected}"
        );
    }

    /// When all edges are opposed (e.g. one-way against travel), the fallback
    /// returns from_uid (graph-ordered start) — never to_uid.
    #[test]
    fn fallback_uses_from_node_when_all_opposed() {
        // Only edge 1→2 East; truck heads West → dot≈-1, rejected.
        let nodes: Vec<(u64, f64, f64)> = vec![(1, 0.0, 0.0), (2, 100.0, 0.0)];
        let edges: Vec<(u64, u64, f64)> = vec![(1, 2, 100.0)];
        let graph = RouterGraph::new(nodes, edges);
        // heading=0.25 (ETS2 West → hx≈-1, hz≈0).
        let result = graph
            .find_nearest_on_edge(50.0, 2.0, 0.25, 20.0)
            .expect("fallback must not be None");
        let (_uid, _, filter_used, rejected) = result;
        assert!(!filter_used, "heading gate rejected all → fallback used");
        assert_eq!(rejected, 1, "single opposing edge must be counted");
    }

    /// Explicit assertion: fallback node is from_uid=1, not to_uid=2.
    #[test]
    fn counterflow_fallback_node_is_from_uid() {
        let nodes: Vec<(u64, f64, f64)> = vec![(1, 0.0, 0.0), (2, 100.0, 0.0)];
        let edges: Vec<(u64, u64, f64)> = vec![(1, 2, 100.0)];
        let graph = RouterGraph::new(nodes, edges);
        // Truck heading West (opposed to east edge).
        let (uid, _, _, _) = graph
            .find_nearest_on_edge(50.0, 2.0, 0.25, 20.0)
            .expect("fallback must exist");
        assert_eq!(
            uid, 1,
            "fallback must be from_uid=1 (graph-ordered start), not to_uid=2"
        );
    }
}
