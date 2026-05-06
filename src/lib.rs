//! TruckPilot — autopilot, map-export and tooling library for Euro Truck
//! Simulator 2.
//!
//! The crate is organised into a few coherent layers:
//!
//! - **Map / graph pipeline**: [`ets2_parser`], [`pipeline`], [`graph_schema`],
//!   [`graph_export`], [`compat_export`], [`json_export`].
//! - **Routing**: [`autopilot`] (A* on `GraphData`), [`route_smoothing`].
//! - **Live runtime**: [`autopilot_loop`], [`controller`], [`acc_controller`],
//!   [`telemetry`], [`shm_telemetry`], [`vjoy`].
//! - **Configuration**: [`config`].

pub mod acc_controller;
pub mod autopilot;
pub mod autopilot_loop;
pub mod compat_export;
pub mod config;
pub mod controller;
pub mod ets2_parser;
pub mod graph_export;
pub mod graph_schema;
pub mod json_export;
pub mod pipeline;
pub mod route_smoothing;
pub mod shm_telemetry;
pub mod telemetry;
pub mod vjoy;
