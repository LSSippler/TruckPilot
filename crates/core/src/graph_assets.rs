//! Owned routing assets loaded once at daemon startup.
//!
//! Built on the OS main thread (before the Tokio runtime) so large graph /
//! SplineIndex work does not run on a Tokio worker with a smaller stack.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use truckpilot_map_parser::{build_index_with_metadata, build_splines_ex, MapGraph, SplineIndex};
use truckpilot_plugin_api::graph::RouterGraph;

/// Postcard cache consumed by plugins at `on_load` (avoids passing `Arc` across cdylib).
pub const SPLINE_LOOKUP_CACHE: &str = "plugins/.spline_lookups.bin";

/// Shared graph resources injected into [`PluginManager`](crate::plugin_manager::PluginManager).
pub struct CoreGraphAssets {
    pub router_graph: Arc<RouterGraph>,
    pub spline_index: Arc<SplineIndex>,
    pub spline_index_road_seg_count: usize,
    pub road_seg_by_from_to: Arc<HashMap<(u64, u64), usize>>,
    pub navcurve_seg_by_from_to: Arc<HashMap<(u64, u64), Vec<usize>>>,
}

pub fn log_init_stage(message: &str) {
    eprintln!("[core] init: {message}");
}

/// Load `graph.json`, build routing graph + SplineIndex, drop the heavy [`MapGraph`]
/// before returning so plugin init has maximum headroom.
pub fn load_core_graph_assets_or_exit(graph_path: &Path) -> CoreGraphAssets {
    if !graph_path.exists() {
        eprintln!("ERROR: graph.json not found at {}", graph_path.display());
        eprintln!("Run first: truckpilot-core parse-map --ets2-dir <path>");
        std::process::exit(1);
    }
    let abs = graph_path.canonicalize().unwrap_or_else(|_| graph_path.to_path_buf());
    eprintln!("INFO: Loading graph from {:?}", abs);

    let map_graph = match truckpilot_map_parser::load_map_graph_from_path(graph_path, "core") {
        Ok(g) => g,
        Err(e) => {
            eprintln!("ERROR: {e}");
            std::process::exit(1);
        }
    };
    truckpilot_map_parser::log_graph_load_stage("core", "done");

    log_init_stage("graph assets built (MapGraph parsed)");
    log_init_stage("building RouterGraph");
    let router_graph = Arc::new(build_router_graph(&map_graph));
    log_init_stage("RouterGraph ready");

    log_init_stage("building SplineIndex");
    let (spline_index, road_seg_count) = build_spline_index(&map_graph);
    log_init_stage("SplineIndex ready");

    // Release ~GB of MapGraph (nodes, prefab descriptors, ai paths) before plugins.
    drop(map_graph);
    log_init_stage("MapGraph dropped");

    log_init_stage("storing graph assets");
    let spline_index = Arc::new(spline_index);
    log_init_stage("building spline segment lookup tables");
    let (road_seg_by_from_to, navcurve_seg_by_from_to) =
        build_spline_segment_lookups(&spline_index, road_seg_count);
    if let Err(e) = write_spline_lookup_cache(&road_seg_by_from_to, &navcurve_seg_by_from_to) {
        eprintln!("WARNING: failed to write {SPLINE_LOOKUP_CACHE}: {e}");
    }
    log_init_stage("spline segment lookup tables ready");

    CoreGraphAssets {
        router_graph,
        spline_index,
        spline_index_road_seg_count: road_seg_count,
        road_seg_by_from_to,
        navcurve_seg_by_from_to,
    }
}

fn build_router_graph(map_graph: &MapGraph) -> RouterGraph {
    truckpilot_map_parser::log_graph_load_stage("core", "build RouterGraph start");
    let nodes: Vec<(u64, f64, f64)> = map_graph.nodes.iter().map(|n| (n.uid, n.x, n.z)).collect();
    let edges: Vec<(u64, u64, f64)> = map_graph
        .edges
        .iter()
        .map(|e| (e.from, e.to, e.distance_m))
        .collect();
    truckpilot_map_parser::log_graph_load_stage(
        "core",
        &format!("nodes = {}, edges = {}", nodes.len(), edges.len()),
    );
    let router = RouterGraph::new(nodes, edges);
    truckpilot_map_parser::log_graph_load_stage("core", "build RouterGraph done");
    router
}

fn build_spline_index(map_graph: &MapGraph) -> (SplineIndex, usize) {
    truckpilot_map_parser::log_graph_load_stage("core", "build SplineIndex start");
    let t0 = std::time::Instant::now();
    let (mut segments, mut metadata, _stats) = build_splines_ex(map_graph);
    let road_seg_count = segments.len();

    let (prefab_segs, prefab_meta) = map_graph.prefab_hermite_segments_with_metadata();
    let navcurve_count = prefab_segs.len();
    segments.extend(prefab_segs);
    metadata.extend(prefab_meta);

    let total_count = segments.len();
    truckpilot_map_parser::log_graph_load_stage("core", "indexes start");
    let index = build_index_with_metadata(segments, metadata);
    truckpilot_map_parser::log_graph_load_stage("core", "indexes done");
    eprintln!(
        "[core] init: SplineIndex built: {} segs ({} road + {} NavCurve) in {:.1}s",
        total_count,
        road_seg_count,
        navcurve_count,
        t0.elapsed().as_secs_f64()
    );
    (index, road_seg_count)
}

fn build_spline_segment_lookups(
    index: &SplineIndex,
    road_seg_count: usize,
) -> (
    Arc<HashMap<(u64, u64), usize>>,
    Arc<HashMap<(u64, u64), Vec<usize>>>,
) {
    let road_n = road_seg_count.min(index.segments.len());
    let mut road_map = HashMap::with_capacity(road_n);
    for i in 0..road_n {
        let s = &index.segments[i];
        road_map.insert((s.from_uid, s.to_uid), i);
    }
    let mut nc_map: HashMap<(u64, u64), Vec<usize>> = HashMap::new();
    for i in road_n..index.segments.len() {
        let s = &index.segments[i];
        nc_map.entry((s.from_uid, s.to_uid)).or_default().push(i);
    }
    (Arc::new(road_map), Arc::new(nc_map))
}

fn write_spline_lookup_cache(
    road: &HashMap<(u64, u64), usize>,
    navcurve: &HashMap<(u64, u64), Vec<usize>>,
) -> std::io::Result<()> {
    let path = PathBuf::from(SPLINE_LOOKUP_CACHE);
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let bytes = postcard::to_allocvec(&(road, navcurve))
        .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
    std::fs::write(path, bytes)
}
