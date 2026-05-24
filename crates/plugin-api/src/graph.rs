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
}
