//! 5-Level vision-fallback cascade state machine.

use std::collections::{HashMap, VecDeque};

// ── Thresholds ──────────────────────────────────────────────────────────────

/// 5-frame rolling avg threshold for Level 0 → Level 1 boundary.
const CONF_L0: f64 = 0.70;
/// Threshold below which Level 2 is triggered.
const CONF_L2: f64 = 0.30;
/// Threshold below which level is considered "blind" (Level 3 / 4 territory).
const CONF_BLIND: f64 = 0.15;

/// Consecutive blind ticks before heading-hold (Level 3). Normal debounce.
const L3_BLIND_NORMAL: u64 = 5;
/// Accelerated blind ticks when confidence_trend < DEGRADING_TREND.
const L3_BLIND_RAPID: u64 = 2;
/// Consecutive blind ticks before disengage (Level 4). 2.0 s at 50 Hz.
const L4_BLIND_TICKS: u64 = 100;

/// Confidence trend below which debounce is accelerated (rapid degradation).
const DEGRADING_TREND: f64 = -0.05;
/// Confidence trend above which recovery hold is halved (rapid improvement).
const IMPROVING_TREND: f64 = 0.05;

/// De-escalation hold ticks: Level 1 → Level 0 (and Level 2 → Level 1). Normal.
const HOLD_L1_TO_L0: u32 = 8;
/// De-escalation hold ticks: Level 3 → Level 2 (or Level 1). Normal.
const HOLD_L3_RECOVERY: u32 = 10;

const CONF_HISTORY_LEN: usize = 10;
const OFFSET_HISTORY_LEN: usize = 5;
const ROLLING_AVG_WINDOW: usize = 5;

// ── State struct ─────────────────────────────────────────────────────────────

pub struct FallbackState {
    pub level: u8,
    pub previous_level: u8,
    pub fallback_reason: String,

    pub blind_tick_count: u64,
    pub blind_start_tick: Option<u64>,
    pub level_entered_at_tick: u64,

    /// Recovery candidate level (de-escalation target) + how many consecutive
    /// ticks we have been trying to recover to it.
    recovery_candidate: Option<u8>,
    recovery_hold_ticks: u32,

    /// Tick → cooldown-expires-at map. Escalating back into a level is blocked
    /// until `tick_count >= cooldown_map[level]`.
    reentry_cooldown_map: HashMap<u8, u64>,

    pub confidence_history: VecDeque<f64>,
    pub offset_history: VecDeque<f64>,

    /// EMA (α=0.3) used as the smoothed signal for Level 2 decisions.
    pub ema_confidence: f64,

    /// Monotonic counter incremented by `update()` each call.
    pub tick_count: u64,
}

impl FallbackState {
    pub fn new() -> Self {
        Self {
            level: 0,
            previous_level: 0,
            fallback_reason: "normal".to_string(),
            blind_tick_count: 0,
            blind_start_tick: None,
            level_entered_at_tick: 0,
            recovery_candidate: None,
            recovery_hold_ticks: 0,
            reentry_cooldown_map: HashMap::new(),
            confidence_history: VecDeque::with_capacity(CONF_HISTORY_LEN),
            offset_history: VecDeque::with_capacity(OFFSET_HISTORY_LEN),
            ema_confidence: 0.0,
            tick_count: 0,
        }
    }

    pub fn reset(&mut self) {
        self.level = 0;
        self.previous_level = 0;
        self.fallback_reason = "normal".to_string();
        self.blind_tick_count = 0;
        self.blind_start_tick = None;
        self.level_entered_at_tick = 0;
        self.recovery_candidate = None;
        self.recovery_hold_ticks = 0;
        self.reentry_cooldown_map.clear();
        self.confidence_history.clear();
        self.offset_history.clear();
        self.ema_confidence = 0.0;
        // tick_count is NOT reset — it's monotonic
    }

    /// Push the current confidence sample (includes it in this tick's window).
    pub fn push_confidence(&mut self, conf: f64) {
        if self.confidence_history.len() >= CONF_HISTORY_LEN {
            self.confidence_history.pop_front();
        }
        self.confidence_history.push_back(conf);
        self.ema_confidence = 0.3 * conf + 0.7 * self.ema_confidence;
    }

    pub fn push_offset(&mut self, offset: f64) {
        if self.offset_history.len() >= OFFSET_HISTORY_LEN {
            self.offset_history.pop_front();
        }
        self.offset_history.push_back(offset);
    }

    /// Arithmetic mean of the most recent `ROLLING_AVG_WINDOW` confidence samples.
    /// Includes the sample just pushed by `push_confidence`.
    pub fn rolling_avg_confidence(&self) -> f64 {
        let n = self.confidence_history.len().min(ROLLING_AVG_WINDOW);
        if n == 0 {
            return 0.0;
        }
        let skip = self.confidence_history.len() - n;
        self.confidence_history.iter().skip(skip).sum::<f64>() / n as f64
    }

    /// Linear regression slope over the last 10 confidence samples.
    /// Positive = improving, negative = degrading.
    pub fn compute_confidence_trend(&self) -> f64 {
        if self.confidence_history.len() < CONF_HISTORY_LEN {
            return 0.0;
        }
        // Use the 10 most recent samples (all of history)
        let n = 10.0_f64;
        let sum_x: f64 = 45.0; // 0+1+...+9
        let sum_x2: f64 = 285.0; // 0²+1²+...+9²
        let sum_y: f64 = self.confidence_history.iter().sum();
        let sum_xy: f64 = self
            .confidence_history
            .iter()
            .enumerate()
            .map(|(i, &y)| i as f64 * y)
            .sum();
        let denom = n * sum_x2 - sum_x * sum_x; // 825
        if denom.abs() < 1e-9 {
            return 0.0;
        }
        (n * sum_xy - sum_x * sum_y) / denom
    }

    /// Std-dev of the last 5 `lane.center_offset` samples.
    pub fn compute_detection_stability(&self) -> f64 {
        if self.offset_history.len() < 2 {
            return 0.0;
        }
        let n = self.offset_history.len() as f64;
        let mean = self.offset_history.iter().sum::<f64>() / n;
        let var = self
            .offset_history
            .iter()
            .map(|&x| (x - mean).powi(2))
            .sum::<f64>()
            / n;
        var.sqrt()
    }

    // ── Core update ──────────────────────────────────────────────────────────

    /// Advance the fallback state by one tick.
    ///
    /// Call AFTER `push_confidence()`. `avg_conf` is the 5-frame rolling avg
    /// from `rolling_avg_confidence()`.
    pub fn update(&mut self, avg_conf: f64, left_vis: bool, right_vis: bool) -> u8 {
        self.tick_count += 1;

        let trend = self.compute_confidence_trend();

        // ── Blind-tick counter ──────────────────────────────────────────────
        let no_detection = avg_conf < CONF_BLIND;
        if no_detection {
            self.blind_tick_count += 1;
            if self.blind_start_tick.is_none() {
                self.blind_start_tick = Some(self.tick_count);
            }
        } else {
            self.blind_tick_count = 0;
            self.blind_start_tick = None;
        }

        // ── Compute raw candidate level (descending severity) ───────────────
        let l3_required = if trend < DEGRADING_TREND {
            L3_BLIND_RAPID
        } else {
            L3_BLIND_NORMAL
        };
        let both_visible = left_vis && right_vis;

        let raw = if self.blind_tick_count >= L4_BLIND_TICKS {
            4u8
        } else if self.blind_tick_count >= l3_required {
            3
        } else if avg_conf < CONF_L2 {
            2
        } else if !both_visible || avg_conf < CONF_L0 {
            1
        } else {
            0
        };

        // ── Update fallback_reason before any transition ────────────────────
        self.fallback_reason = match raw {
            4 => "no_detection_timeout_2s",
            3 => "no_detection_short",
            2 => "confidence_low",
            1 if !left_vis && !right_vis => "no_lanes_visible",
            1 if !left_vis => "left_lane_lost",
            1 if !right_vis => "right_lane_lost",
            1 => "confidence_marginal",
            _ => "normal",
        }
        .to_string();

        // ── Apply hysteresis + cooldown ─────────────────────────────────────
        if raw > self.level {
            // Escalation: immediate, but check re-entry cooldown.
            // Level 4 is never blocked by cooldown (safety-critical).
            let effective = if raw == 4 {
                raw
            } else if let Some(&until) = self.reentry_cooldown_map.get(&raw) {
                if self.tick_count < until {
                    self.level
                } else {
                    raw
                }
            } else {
                raw
            };
            if effective != self.level {
                self.do_transition(effective);
            }
        } else if raw < self.level {
            // De-escalation: require sustained improvement.
            let hold = self.required_hold(self.level, raw, trend);
            if self.recovery_candidate == Some(raw) {
                self.recovery_hold_ticks += 1;
                if self.recovery_hold_ticks >= hold {
                    self.do_transition(raw);
                    self.recovery_candidate = None;
                    self.recovery_hold_ticks = 0;
                }
            } else {
                self.recovery_candidate = Some(raw);
                self.recovery_hold_ticks = 1;
            }
        } else {
            // Staying at same level — reset recovery tracking.
            self.recovery_candidate = None;
            self.recovery_hold_ticks = 0;
        }

        self.level
    }

    fn required_hold(&self, from: u8, to: u8, trend: f64) -> u32 {
        let rapid_improving = trend > IMPROVING_TREND;
        match (from, to) {
            // Level 3 recovery always uses the longer hold, halved if improving fast.
            (3, _) => {
                if rapid_improving {
                    HOLD_L3_RECOVERY / 2
                } else {
                    HOLD_L3_RECOVERY
                }
            }
            // Level 1→0 or Level 2→1.
            _ => {
                if rapid_improving {
                    HOLD_L1_TO_L0 / 2
                } else {
                    HOLD_L1_TO_L0
                }
            }
        }
    }

    fn do_transition(&mut self, new_level: u8) {
        let cooldown_duration: u64 = if self.level == 3 { 30 } else { 20 };
        self.reentry_cooldown_map
            .insert(self.level, self.tick_count + cooldown_duration);
        self.previous_level = self.level;
        self.level_entered_at_tick = self.tick_count;
        self.level = new_level;
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn state() -> FallbackState {
        FallbackState::new()
    }

    fn tick(s: &mut FallbackState, conf: f64, left: bool, right: bool) -> u8 {
        s.push_confidence(conf);
        let avg = s.rolling_avg_confidence();
        s.update(avg, left, right)
    }

    fn tick_n(s: &mut FallbackState, n: u64, conf: f64, left: bool, right: bool) -> u8 {
        let mut level = 0;
        for _ in 0..n {
            level = tick(s, conf, left, right);
        }
        level
    }

    // ── Level determination ───────────────────────────────────────────────

    #[test]
    fn level_0_with_high_confidence_both_lanes() {
        let mut s = state();
        let level = tick_n(&mut s, 5, 0.85, true, true);
        assert_eq!(level, 0);
    }

    #[test]
    fn level_1_with_single_lane() {
        let mut s = state();
        let level = tick(&mut s, 0.80, true, false);
        assert_eq!(level, 1);
        assert_eq!(s.fallback_reason, "right_lane_lost");
    }

    #[test]
    fn level_1_with_marginal_confidence_0_65() {
        let mut s = state();
        let level = tick(&mut s, 0.65, true, true);
        assert_eq!(level, 1);
        assert_eq!(s.fallback_reason, "confidence_marginal");
    }

    #[test]
    fn level_2_with_low_confidence_0_25() {
        let mut s = state();
        let level = tick(&mut s, 0.25, true, true);
        assert_eq!(level, 2);
    }

    #[test]
    fn level_3_after_5_ticks_no_detection() {
        let mut s = state();
        // First 4 blind ticks → still level 0 (debounce not yet satisfied)
        for _ in 0..4 {
            let l = tick(&mut s, 0.0, false, false);
            assert!(l < 3, "expected <3 before 5 ticks, got {l}");
        }
        // 5th tick → level 3
        let l = tick(&mut s, 0.0, false, false);
        assert_eq!(l, 3);
    }

    #[test]
    fn level_4_after_100_ticks_no_detection() {
        let mut s = state();
        tick_n(&mut s, 100, 0.0, false, false);
        let l = tick(&mut s, 0.0, false, false);
        assert_eq!(l, 4);
    }

    // ── Hysteresis ───────────────────────────────────────────────────────

    #[test]
    fn level_0_to_1_is_immediate() {
        // Single-lane loss triggers L1 immediately (no rolling-avg debounce on visibility).
        let mut s = state();
        tick_n(&mut s, 10, 0.85, true, true); // establish level 0
        let l = tick(&mut s, 0.85, true, false); // right lane lost → L1 immediately
        assert_eq!(l, 1);
        assert_eq!(s.fallback_reason, "right_lane_lost");
    }

    #[test]
    fn level_1_to_0_requires_8_ticks_sustained() {
        let mut s = state();
        // Enter level 1
        tick(&mut s, 0.65, true, true);
        assert_eq!(s.level, 1);

        // 7 ticks at conf >= 0.70 + both lanes → not yet at L0
        for i in 0..7 {
            let l = tick(&mut s, 0.85, true, true);
            assert!(
                l > 0,
                "tick {i}: should not have recovered to L0 yet, got {l}"
            );
        }
        // 8th tick → should transition to L0
        let l = tick(&mut s, 0.85, true, true);
        assert_eq!(l, 0, "8th sustained tick should recover to L0");
    }

    #[test]
    fn level_3_to_2_requires_10_ticks_sustained() {
        // L3→lower recovery requires HOLD_L3_RECOVERY (10) ticks of sustained better raw level.
        // The rolling-avg needs several ticks to climb out of the blind zone (avg < 0.15) before
        // recovery counting begins. Total at conf=0.35: ~13 ticks still L3, transition on 14th.
        let mut s = state();
        tick_n(&mut s, 5, 0.0, false, false);
        assert_eq!(s.level, 3);

        // 13 ticks: rolling avg hasn't fully stabilised and/or hold not yet reached
        for _ in 0..13 {
            let l = tick(&mut s, 0.35, true, true);
            assert_eq!(
                l, 3,
                "should not recover from L3 before 14 ticks of conf=0.35"
            );
        }
        // 14th tick → rolling avg fully above thresholds, hold counter saturated → transition
        let l = tick(&mut s, 0.35, true, true);
        assert!(
            l < 3,
            "should have recovered from L3 after 14 sustained ticks: got {l}"
        );
    }

    // ── Anti-flapping ────────────────────────────────────────────────────

    #[test]
    fn no_oscillation_at_confidence_boundary() {
        let mut s = state();
        tick_n(&mut s, 10, 0.85, true, true); // settle at L0
                                              // Alternate 0.69 and 0.71 for 30 ticks
        let mut transitions = 0u32;
        let mut prev = s.level;
        for i in 0..30 {
            let conf = if i % 2 == 0 { 0.69 } else { 0.71 };
            let l = tick(&mut s, conf, true, true);
            if l != prev {
                transitions += 1;
                prev = l;
            }
        }
        assert!(
            transitions <= 2,
            "too many transitions at boundary: {transitions}"
        );
    }

    #[test]
    fn level_4_not_blocked_by_cooldown() {
        let mut s = state();
        // Reach L4, then reset, then immediately trigger L4 again
        tick_n(&mut s, 101, 0.0, false, false);
        assert_eq!(s.level, 4);
        // Simulate a manual re-engage reset
        s.reset();
        // Now trigger L4 again — cooldown from the previous L4 must not block
        tick_n(&mut s, 101, 0.0, false, false);
        assert_eq!(s.level, 4);
    }

    // ── Confidence trend ──────────────────────────────────────────────────

    #[test]
    fn confidence_trend_negative_falling() {
        // Slope from [1.0..0.1 in steps of 0.1] = -0.10, strictly < DEGRADING_TREND (-0.05).
        let mut s = state();
        for v in [1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1] {
            s.push_confidence(v);
        }
        let trend = s.compute_confidence_trend();
        assert!(
            trend < DEGRADING_TREND,
            "expected falling trend, got {trend:.4}"
        );
    }

    #[test]
    fn confidence_trend_zero_stable() {
        let mut s = state();
        for _ in 0..10 {
            s.push_confidence(0.70);
        }
        let trend = s.compute_confidence_trend();
        assert!(trend.abs() < 0.01, "expected ~0 trend, got {trend:.4}");
    }

    #[test]
    fn rapid_degradation_bypasses_debounce() {
        let mut s = state();
        // Fill history with steeply falling confidence so trend < DEGRADING_TREND (-0.05).
        for v in [1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1] {
            s.push_confidence(v);
        }
        let trend = s.compute_confidence_trend();
        assert!(trend < DEGRADING_TREND);
        // With rapid degradation, only 2 blind ticks are needed
        tick(&mut s, 0.0, false, false);
        tick(&mut s, 0.0, false, false);
        let l = tick(&mut s, 0.0, false, false);
        assert_eq!(l, 3, "should enter L3 in 2 blind ticks when degrading fast");
    }

    // ── Detection stability ───────────────────────────────────────────────

    #[test]
    fn detection_stability_low_for_steady_offset() {
        let mut s = state();
        for _ in 0..5 {
            s.push_offset(0.05);
        }
        let stab = s.compute_detection_stability();
        assert!(
            stab < 0.01,
            "expected low stability for steady offset, got {stab:.4}"
        );
    }

    #[test]
    fn detection_stability_high_for_jittery_offset() {
        let mut s = state();
        for v in [-0.5, 0.5, -0.5, 0.5, -0.5] {
            s.push_offset(v);
        }
        let stab = s.compute_detection_stability();
        assert!(
            stab > 0.20,
            "expected high stability for jittery offset, got {stab:.4}"
        );
    }
}
