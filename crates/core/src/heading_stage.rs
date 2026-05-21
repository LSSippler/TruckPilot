//! Three-stage heading response system — Phase 6.5s.
//!
//! The [`HeadingStageManager`] reads the absolute heading error produced by
//! the lane-keeper plugin (`lane_keeper.error_rad`) and decides which of
//! four stages the autopilot is in:
//!
//! | Stage        | Error range   | Lane-keeper behaviour          | Router behaviour     |
//! |-------------|---------------|-------------------------------|---------------------|
//! | Normal       | < 45°         | Standard PID                  | Periodic replan     |
//! | SoftLaneKeep | 45° – 60°     | 30 % gain PID + PID reset     | Periodic replan     |
//! | AutoReplan   | > 60°         | Steering = 0                  | Trigger replan      |
//! | Disengaging  | latched       | Steering = 0                  | —                   |
//!
//! Transitions are hysteresis-gated (3 consecutive frames, ~60 ms at
//! 50 Hz) except the safety-critical Disengaging transition which is
//! immediate. The Disengaging stage is a latch — once entered it
//! persists until explicitly reset.

use truckpilot_plugin_api::SharedBlackboard;

const SOFT_ENTER_RAD: f64 = 0.785; // 45°
const SOFT_EXIT_RAD: f64 = 0.393; // 22.5°
const REPLAN_ENTER_RAD: f64 = 1.047; // 60°
const REPLAN_EXIT_RAD: f64 = 0.785; // 45°
const HYSTERESIS_FRAMES: u32 = 3; // 60 ms at 50 Hz

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeadingStage {
    Normal,
    SoftLaneKeep,
    AutoReplan,
    Disengaging,
}

impl HeadingStage {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Normal => "Normal",
            Self::SoftLaneKeep => "SoftLaneKeep",
            Self::AutoReplan => "AutoReplan",
            Self::Disengaging => "Disengaging",
        }
    }
}

pub struct HeadingStageManager {
    pub stage: HeadingStage,
    stage_entered_at: Option<std::time::Instant>,
    pub transition_count: u32,
    hysteresis_ticks: u32,
    hysteresis_target: Option<HeadingStage>,
}

impl HeadingStageManager {
    pub fn new() -> Self {
        Self {
            stage: HeadingStage::Normal,
            stage_entered_at: Some(std::time::Instant::now()),
            transition_count: 0,
            hysteresis_ticks: 0,
            hysteresis_target: None,
        }
    }

    pub fn evaluate(&mut self, bb: &SharedBlackboard) {
        let error_rad = bb
            .get("lane_keeper.error_rad")
            .and_then(|s| s.trim().parse::<f64>().ok())
            .map(|e| e.abs())
            .unwrap_or(0.0);

        let desired = self.desired_stage(error_rad, bb);

        if desired != self.stage {
            if self.hysteresis_target == Some(desired) {
                self.hysteresis_ticks += 1;
            } else {
                self.hysteresis_target = Some(desired);
                self.hysteresis_ticks = 1;
            }
            if desired == HeadingStage::Disengaging || self.hysteresis_ticks >= HYSTERESIS_FRAMES {
                self.transition_to(desired);
            }
        } else {
            self.hysteresis_ticks = 0;
            self.hysteresis_target = None;
        }

        bb.set("state.heading_stage", self.stage.as_str());
        bb.set("state.heading_diff_rad", format!("{:.6}", error_rad));
        bb.set("state.heading_diff_deg", format!("{:.1}", error_rad.to_degrees()));
        bb.set("state.heading_stage_transition_count", self.transition_count.to_string());
        if let Some(t) = self.stage_entered_at {
            bb.set("state.heading_stage_entered_at_ms", t.elapsed().as_millis().to_string());
        }
    }

    fn desired_stage(&self, error_rad: f64, bb: &SharedBlackboard) -> HeadingStage {
        if self.stage == HeadingStage::Disengaging {
            return HeadingStage::Disengaging;
        }

        if self.stage == HeadingStage::AutoReplan {
            let exhausted = bb
                .get("router.auto_replan_count")
                .and_then(|s| s.trim().parse::<u32>().ok())
                .map(|c| c >= 3)
                .unwrap_or(false);
            if exhausted && error_rad > SOFT_EXIT_RAD {
                return HeadingStage::Disengaging;
            }
        }

        match self.stage {
            HeadingStage::Normal => {
                if error_rad > SOFT_ENTER_RAD {
                    HeadingStage::SoftLaneKeep
                } else {
                    HeadingStage::Normal
                }
            }
            HeadingStage::SoftLaneKeep => {
                if error_rad < SOFT_EXIT_RAD {
                    HeadingStage::Normal
                } else if error_rad > REPLAN_ENTER_RAD {
                    HeadingStage::AutoReplan
                } else {
                    HeadingStage::SoftLaneKeep
                }
            }
            HeadingStage::AutoReplan => {
                if error_rad < SOFT_EXIT_RAD {
                    HeadingStage::Normal
                } else if error_rad < REPLAN_EXIT_RAD {
                    HeadingStage::SoftLaneKeep
                } else {
                    HeadingStage::AutoReplan
                }
            }
            HeadingStage::Disengaging => HeadingStage::Disengaging,
        }
    }

    fn transition_to(&mut self, new_stage: HeadingStage) {
        if new_stage == self.stage {
            return;
        }
        tracing::info!(
            "[heading_stage] {:?} -> {:?} (transitions={})",
            self.stage,
            new_stage,
            self.transition_count + 1
        );
        self.stage = new_stage;
        self.stage_entered_at = Some(std::time::Instant::now());
        self.transition_count += 1;
        self.hysteresis_ticks = 0;
        self.hysteresis_target = None;
    }

    pub fn reset(&mut self) {
        if self.stage != HeadingStage::Normal {
            tracing::info!("[heading_stage] reset -> Normal");
            self.stage = HeadingStage::Normal;
            self.stage_entered_at = Some(std::time::Instant::now());
        }
        self.hysteresis_ticks = 0;
        self.hysteresis_target = None;
    }
}

impl Default for HeadingStageManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn new_bb() -> SharedBlackboard {
        SharedBlackboard::new()
    }

    fn set_error(bb: &SharedBlackboard, rad: f64) {
        bb.set("lane_keeper.error_rad", format!("{:.6}", rad));
    }

    #[test]
    fn stage_normal_at_zero() {
        let mut hsm = HeadingStageManager::new();
        let bb = new_bb();
        set_error(&bb, 0.0);
        hsm.evaluate(&bb);
        assert_eq!(hsm.stage, HeadingStage::Normal);
    }

    #[test]
    fn stage_normal_below_soft_enter() {
        let mut hsm = HeadingStageManager::new();
        let bb = new_bb();
        set_error(&bb, 0.76);
        hsm.evaluate(&bb);
        assert_eq!(hsm.stage, HeadingStage::Normal);
    }

    #[test]
    fn stage_soft_after_hysteresis() {
        let mut hsm = HeadingStageManager::new();
        let bb = new_bb();
        set_error(&bb, 0.85);
        for _ in 0..3 {
            hsm.evaluate(&bb);
        }
        assert_eq!(hsm.stage, HeadingStage::SoftLaneKeep);
    }

    #[test]
    fn stage_no_transition_at_2_frames() {
        let mut hsm = HeadingStageManager::new();
        let bb = new_bb();
        set_error(&bb, 0.85);
        hsm.evaluate(&bb);
        hsm.evaluate(&bb);
        assert_eq!(hsm.stage, HeadingStage::Normal);
        set_error(&bb, 0.2);
        hsm.evaluate(&bb);
        assert_eq!(hsm.stage, HeadingStage::Normal);
    }

    #[test]
    fn stage_autoreplan_after_hysteresis() {
        let mut hsm = HeadingStageManager::new();
        let bb = new_bb();
        set_error(&bb, 0.85);
        for _ in 0..3 {
            hsm.evaluate(&bb);
        }
        assert_eq!(hsm.stage, HeadingStage::SoftLaneKeep);
        set_error(&bb, 1.10);
        for _ in 0..3 {
            hsm.evaluate(&bb);
        }
        assert_eq!(hsm.stage, HeadingStage::AutoReplan);
    }

    #[test]
    fn stage_autoreplan_falls_back_to_soft() {
        let mut hsm = HeadingStageManager::new();
        let bb = new_bb();
        set_error(&bb, 0.85);
        for _ in 0..3 {
            hsm.evaluate(&bb);
        }
        assert_eq!(hsm.stage, HeadingStage::SoftLaneKeep);
        set_error(&bb, 1.10);
        for _ in 0..3 {
            hsm.evaluate(&bb);
        }
        assert_eq!(hsm.stage, HeadingStage::AutoReplan);
        set_error(&bb, 0.70);
        for _ in 0..3 {
            hsm.evaluate(&bb);
        }
        assert_eq!(hsm.stage, HeadingStage::SoftLaneKeep);
    }

    #[test]
    fn stage_soft_falls_back_to_normal() {
        let mut hsm = HeadingStageManager::new();
        let bb = new_bb();
        set_error(&bb, 0.85);
        for _ in 0..3 {
            hsm.evaluate(&bb);
        }
        assert_eq!(hsm.stage, HeadingStage::SoftLaneKeep);
        set_error(&bb, 0.30);
        for _ in 0..3 {
            hsm.evaluate(&bb);
        }
        assert_eq!(hsm.stage, HeadingStage::Normal);
    }

    #[test]
    fn stage_disengaging_when_replan_exhausted() {
        let mut hsm = HeadingStageManager::new();
        let bb = new_bb();
        set_error(&bb, 0.85);
        for _ in 0..3 {
            hsm.evaluate(&bb);
        }
        set_error(&bb, 1.10);
        for _ in 0..3 {
            hsm.evaluate(&bb);
        }
        assert_eq!(hsm.stage, HeadingStage::AutoReplan);
        bb.set("router.auto_replan_count", "3");
        set_error(&bb, 0.50);
        hsm.evaluate(&bb);
        assert_eq!(hsm.stage, HeadingStage::Disengaging);
    }

    #[test]
    fn stage_reset_to_normal() {
        let mut hsm = HeadingStageManager::new();
        hsm.stage = HeadingStage::AutoReplan;
        hsm.reset();
        assert_eq!(hsm.stage, HeadingStage::Normal);
    }

    #[test]
    fn transition_count_increments() {
        let mut hsm = HeadingStageManager::new();
        let bb = new_bb();
        assert_eq!(hsm.transition_count, 0);
        set_error(&bb, 0.85);
        for _ in 0..3 {
            hsm.evaluate(&bb);
        }
        assert_eq!(hsm.transition_count, 1);
    }

    #[test]
    fn publishes_blackboard_keys() {
        let mut hsm = HeadingStageManager::new();
        let bb = new_bb();
        set_error(&bb, 0.50);
        hsm.evaluate(&bb);
        assert_eq!(bb.get("state.heading_stage").as_deref(), Some("Normal"));
        assert!(bb.get("state.heading_diff_rad").is_some());
        assert!(bb.get("state.heading_diff_deg").is_some());
        assert!(bb.get("state.heading_stage_transition_count").is_some());
    }

    #[test]
    fn disengaging_latched_until_reset() {
        let mut hsm = HeadingStageManager::new();
        hsm.stage = HeadingStage::Disengaging;
        let bb = new_bb();
        set_error(&bb, 0.0);
        for _ in 0..10 {
            hsm.evaluate(&bb);
        }
        assert_eq!(hsm.stage, HeadingStage::Disengaging);
    }

    #[test]
    fn missing_error_defaults_to_normal() {
        let mut hsm = HeadingStageManager::new();
        let bb = new_bb();
        for _ in 0..5 {
            hsm.evaluate(&bb);
        }
        assert_eq!(hsm.stage, HeadingStage::Normal);
    }
}
