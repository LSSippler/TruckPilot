//! Resolver scheduling: exponential backoff, scan limits, diagnose parking.

/// Minimum worker polling interval (5 Hz).
pub const MIN_RESOLVER_POLL_US: u64 = 200_000;

/// Backoff steps after failed / expensive resolver walks (microseconds).
pub const BACKOFF_STEPS_US: [u64; 6] = [
    2_000_000,
    4_000_000,
    8_000_000,
    16_000_000,
    30_000_000,
    60_000_000,
];

/// Max full `.text` pattern scans per session before parking.
pub const MAX_PATTERN_SCANS_PER_SESSION: u32 = 5;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResolverSchedule {
    pub failure_streak: u32,
    pub backoff_us: u64,
    pub pattern_scans: u32,
    pub parked: bool,
    pub diagnostic_parked: bool,
    pub last_walk_us: u64,
    pub last_enable_generation: u64,
}

impl ResolverSchedule {
    pub const fn new() -> Self {
        Self {
            failure_streak: 0,
            backoff_us: BACKOFF_STEPS_US[0],
            pattern_scans: 0,
            parked: false,
            diagnostic_parked: false,
            last_walk_us: 0,
            last_enable_generation: 0,
        }
    }

    pub fn reset_session(&mut self) {
        *self = Self::new();
    }

    pub fn note_enable_generation(&mut self, generation: u64) {
        if generation != self.last_enable_generation {
            self.last_enable_generation = generation;
            self.unpark("enable_file_changed");
        }
    }

    pub fn unpark(&mut self, _reason: &str) {
        self.parked = false;
        self.diagnostic_parked = false;
        self.failure_streak = 0;
        self.backoff_us = BACKOFF_STEPS_US[0];
    }

    pub fn park_after_limit(&mut self) {
        self.parked = true;
        crate::resolver_metrics::set_resolver_parked(true);
    }

    pub fn park_resolver_off(&mut self) {
        self.parked = true;
        crate::resolver_metrics::set_resolver_parked(true);
    }

    pub fn park_diagnostic_done(&mut self) {
        self.diagnostic_parked = true;
        crate::resolver_metrics::set_resolver_parked(true);
    }

    pub fn should_run_walk(&self, now_us: u64) -> bool {
        if self.parked || self.diagnostic_parked {
            return false;
        }
        if self.last_walk_us == 0 {
            return true;
        }
        now_us.saturating_sub(self.last_walk_us) >= self.backoff_us.max(MIN_RESOLVER_POLL_US)
    }

    pub fn note_walk_started(&mut self, now_us: u64) {
        self.last_walk_us = now_us;
    }

    pub fn note_expensive_failure(&mut self) {
        self.failure_streak = self.failure_streak.saturating_add(1);
        let idx = (self.failure_streak as usize).min(BACKOFF_STEPS_US.len() - 1);
        self.backoff_us = BACKOFF_STEPS_US[idx];
        if self.pattern_scans >= MAX_PATTERN_SCANS_PER_SESSION {
            self.park_after_limit();
        }
    }

    pub fn note_pattern_scan(&mut self) {
        self.pattern_scans = self.pattern_scans.saturating_add(1);
        if self.pattern_scans >= MAX_PATTERN_SCANS_PER_SESSION {
            self.park_after_limit();
        }
    }

    pub fn note_success(&mut self) {
        self.failure_streak = 0;
        self.backoff_us = BACKOFF_STEPS_US[0];
    }

    pub fn scan_limited(&self) -> bool {
        self.pattern_scans >= MAX_PATTERN_SCANS_PER_SESSION
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn backoff_increases_after_failure() {
        let mut s = ResolverSchedule::new();
        s.note_expensive_failure();
        assert_eq!(s.backoff_us, 4_000_000);
        s.note_expensive_failure();
        assert_eq!(s.backoff_us, 8_000_000);
    }

    #[test]
    fn retry_limit_parks_resolver() {
        let mut s = ResolverSchedule::new();
        for _ in 0..MAX_PATTERN_SCANS_PER_SESSION {
            s.note_pattern_scan();
        }
        assert!(s.parked);
        assert!(!s.should_run_walk(10_000_000));
    }

    #[test]
    fn diagnostic_parked_blocks_walks() {
        let mut s = ResolverSchedule::new();
        s.park_diagnostic_done();
        assert!(!s.should_run_walk(100_000_000));
    }

    #[test]
    fn min_poll_is_five_hz() {
        let mut s = ResolverSchedule::new();
        s.last_walk_us = 1_000;
        s.backoff_us = MIN_RESOLVER_POLL_US;
        assert!(!s.should_run_walk(1_000 + MIN_RESOLVER_POLL_US - 1));
        assert!(s.should_run_walk(1_000 + MIN_RESOLVER_POLL_US));
    }
}
