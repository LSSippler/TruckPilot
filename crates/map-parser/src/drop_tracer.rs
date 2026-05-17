//! Zero-overhead diagnostic hook for tracing roads dropped during parse.
//!
//! In production the tracer is never constructed — all call sites take
//! `Option<(&DropTracer, &str)>` and the `None` branch compiles to nothing.
//! In diagnostic binaries (e.g. `road-drop-audit`) the caller creates a
//! `DropTracer`, passes it into `parse_sectors_with_drop_tracer`, and drains
//! events with `take_events()` after the parse completes.
//!
//! Injection pattern (sector.rs):
//! ```ignore
//! fn parse_sector_legacy_inner(data, tracer_ctx: Option<(&DropTracer, &str)>) { … }
//! ```
//! The tracer is thread-safe (`Mutex<Vec<…>>`) so it can be shared across
//! concurrent sector parses if needed in the future.

use std::sync::Mutex;

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DropCategory {
    /// parse_road() returned Err inside parse_sector_legacy
    RoadParseFailed,
    /// a non-road item handler failed, causing a sector-level abort (all subsequent items lost)
    SectorHandlerError,
    /// item_type is not handled by any branch, sector aborted
    UnknownItemType,
    /// GraphBuilder: both node_a and node_b are absent from the global node_lookup
    BothUnresolved,
    /// GraphBuilder: one node resolved, one absent AND spatial fallback also failed
    OneUnresolved,
    /// try_parse_sized_sector: sized-format road parse failed (seek to item_end used as fallback)
    SizedRoadParseFailed,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DropEvent {
    pub category: DropCategory,
    pub sector_path: String,
    /// item_type byte from the sector stream (3 for road, etc.)
    pub item_type: u32,
    pub item_uid: Option<u64>,
    pub node_a: Option<u64>,
    pub node_b: Option<u64>,
    pub node_a_resolved: Option<bool>,
    pub node_b_resolved: Option<bool>,
    /// x position of the road start node (if available)
    pub x: Option<f32>,
    /// z position of the road start node (if available)
    pub z: Option<f32>,
    /// raw bytes from road header (capped at DropTracer::hex_limit)
    pub raw_hex: Vec<u8>,
}

pub struct DropTracer {
    events: Mutex<Vec<DropEvent>>,
    /// max bytes captured per drop event into raw_hex
    pub hex_limit: usize,
}

impl DropTracer {
    pub fn new(hex_limit: usize) -> Self {
        Self {
            events: Mutex::new(Vec::new()),
            hex_limit,
        }
    }

    pub fn record(&self, event: DropEvent) {
        self.events.lock().unwrap().push(event);
    }

    /// Drain all recorded events (consuming them from internal storage).
    pub fn take_events(&self) -> Vec<DropEvent> {
        std::mem::take(&mut self.events.lock().unwrap())
    }

    pub fn event_count(&self) -> usize {
        self.events.lock().unwrap().len()
    }
}
