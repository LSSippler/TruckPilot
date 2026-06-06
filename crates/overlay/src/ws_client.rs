//! WebSocket client — connects to daemon at ws://127.0.0.1:8765,
//! polls Blackboard keys at 5 Hz, updates shared HudState.
//!
//! Reconnect strategy: exponential backoff 500ms → 1s → 2s → 5s (max).

use std::sync::Arc;
use std::time::Duration;

use futures_util::{SinkExt, StreamExt};
use tokio::time::sleep;
use tokio_tungstenite::{connect_async, tungstenite::Message};
use tracing::{debug, info, warn};

use truckpilot_ipc_protocol::{CoreMessage, UiCommand};

use crate::state::{HudState, SegmentSnapshot};

const WS_URL: &str = "ws://127.0.0.1:8765";
const POLL_INTERVAL: Duration = Duration::from_millis(200); // 5 Hz

/// Blackboard keys to poll every tick.
const BLACKBOARD_KEYS: &[&str] = &[
    "lane_follower.truck_x",
    "lane_follower.truck_y",
    "lane_follower.truck_z",
    "lane_follower.truck_heading_deg",
    "lane_follower.nearest_seg_idx",
    "lane_follower.nearest_seg_is_prefab",
    "lane_follower.nearest_seg_dist_m",
    "lane_follower.nearest_seg_ai_path_uid",
    "lane_follower.lateral_dist_signed",
    "lane_follower.steering_filtered",
    "lane_follower.bias_zone_active",
    "lane_follower.bias_prefab_attempted",
    "lane_follower.bias_prefab_accepted",
    "lane_follower.bias_prefab_rejected_reason",
    "lane_follower.junction_detected",
    "lane_follower.junction_distance_m",
    "lane_follower.junction_phase",
    "lane_follower.lateral_source",
    "lane_follower.lookahead_offset_x",
    "lane_follower.lookahead_offset_z",
];

/// Runs forever: connect, poll, reconnect on error.
pub async fn run(state: Arc<HudState>) {
    let mut backoff_ms: u64 = 500;

    loop {
        info!("Connecting to daemon at {WS_URL}…");
        match connect_async(WS_URL).await {
            Ok((ws_stream, _)) => {
                info!("Connected to daemon");
                state.set_connected(true);
                backoff_ms = 500; // reset on successful connect

                if let Err(e) = poll_loop(ws_stream, Arc::clone(&state)).await {
                    warn!("WS session ended: {e}");
                }
                state.set_connected(false);
            }
            Err(e) => {
                debug!("Connect failed ({e}), retry in {backoff_ms}ms");
                state.set_connected(false);
            }
        }

        sleep(Duration::from_millis(backoff_ms)).await;
        backoff_ms = (backoff_ms * 2).min(5_000);
    }
}

async fn poll_loop(
    ws_stream: tokio_tungstenite::WebSocketStream<
        tokio_tungstenite::MaybeTlsStream<tokio::net::TcpStream>,
    >,
    state: Arc<HudState>,
) -> anyhow::Result<()> {
    let (mut write, mut read) = ws_stream.split();
    let mut poll_interval = tokio::time::interval(POLL_INTERVAL);

    loop {
        tokio::select! {
            _ = poll_interval.tick() => {
                // Send BlackboardGet request
                let cmd = UiCommand::BlackboardGet {
                    keys: BLACKBOARD_KEYS.iter().map(|s| s.to_string()).collect(),
                };
                let json = serde_json::to_string(&cmd)?;
                write.send(Message::Text(json)).await?;

                // Also request nearby segments
                let seg_cmd = UiCommand::SpatialSegmentsInRadius {
                    x: state.snapshot().pose.x,
                    z: state.snapshot().pose.z,
                    radius_m: 50.0,
                    max_results: 200,
                };
                let json = serde_json::to_string(&seg_cmd)?;
                write.send(Message::Text(json)).await?;
            }

            msg = read.next() => {
                match msg {
                    Some(Ok(Message::Text(text))) => {
                        handle_message(&text, &state);
                    }
                    Some(Ok(Message::Close(_))) | None => {
                        return Err(anyhow::anyhow!("connection closed"));
                    }
                    Some(Err(e)) => {
                        return Err(e.into());
                    }
                    _ => {}
                }
            }
        }
    }
}

fn handle_message(text: &str, state: &HudState) {
    match serde_json::from_str::<CoreMessage>(text) {
        Ok(CoreMessage::BlackboardSnapshot { values, .. }) => {
            state.apply_blackboard(&values);
        }
        Ok(CoreMessage::SpatialSegmentsResponse { segments, .. }) => {
            let snaps: Vec<SegmentSnapshot> = segments
                .into_iter()
                .map(|s| SegmentSnapshot {
                    idx: s.idx,
                    is_prefab: s.is_prefab,
                    start_x: s.start_x,
                    start_z: s.start_z,
                    end_x: s.end_x,
                    end_z: s.end_z,
                })
                .collect();
            state.set_nearby_segments(snaps);
        }
        Ok(_) => {
            // Other daemon messages (telemetry, plugin events, etc.) ignored by overlay
        }
        Err(e) => {
            debug!("Unknown WS message (parse error: {e}): {text:.80}");
        }
    }
}
