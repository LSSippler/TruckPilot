//! Rate-limited logging for optional producer SHM regions (minimap, vision frame, …).

use std::time::{Duration, Instant};

use crate::PluginContext;

const REPEAT_INTERVAL: Duration = Duration::from_secs(30);

/// Tracks optional SHM availability and suppresses log spam when the producer is absent.
#[derive(Debug, Clone, Default)]
pub struct OptionalShmGate {
    mapped: bool,
    first_missing_logged: bool,
    last_log_at: Option<Instant>,
}

impl OptionalShmGate {
    /// Producer SHM mapped successfully.
    pub fn on_mapped(&mut self, ctx: &PluginContext, target: &str, detail: &str) {
        if self.mapped {
            return;
        }
        if self.first_missing_logged {
            crate::ctx_info!(
                ctx,
                target: target,
                "SHM recovered: {detail}"
            );
        } else {
            crate::ctx_info!(
                ctx,
                target: target,
                "mapped SHM {detail}"
            );
        }
        self.mapped = true;
        self.first_missing_logged = false;
    }

    /// Producer SHM is not available (expected when optional sidecar / ETS2 plugin is off).
    pub fn on_missing(&mut self, ctx: &PluginContext, target: &str, err: &str) {
        let now = Instant::now();
        if self.mapped {
            crate::ctx_warn!(
                ctx,
                target: target,
                "SHM lost: {err}"
            );
            self.mapped = false;
            self.last_log_at = Some(now);
            return;
        }

        let due = self
            .last_log_at
            .map(|t| now.duration_since(t) >= REPEAT_INTERVAL)
            .unwrap_or(true);

        if !self.first_missing_logged {
            crate::ctx_warn!(
                ctx,
                target: target,
                "SHM not available: {err} (optional; retries every 30s at DEBUG)"
            );
            self.first_missing_logged = true;
            self.last_log_at = Some(now);
        } else if due {
            crate::ctx_debug!(
                ctx,
                target: target,
                "SHM still unavailable: {err}"
            );
            self.last_log_at = Some(now);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::SharedBlackboard;

    #[test]
    fn first_missing_then_debug_repeat_policy() {
        let mut gate = OptionalShmGate::default();
        let ctx = PluginContext::new("test", SharedBlackboard::new());
        gate.on_missing(&ctx, "test_plugin", "OpenFileMappingW(foo)");
        assert!(!gate.mapped);
        assert!(gate.first_missing_logged);
        gate.on_missing(&ctx, "test_plugin", "OpenFileMappingW(foo)");
        assert!(gate.first_missing_logged);
        gate.on_mapped(&ctx, "test_plugin", "foo (64 B)");
        assert!(gate.mapped);
        assert!(!gate.first_missing_logged);
    }

    #[test]
    fn lost_shm_resets_mapped_flag() {
        let mut gate = OptionalShmGate::default();
        let ctx = PluginContext::new("test", SharedBlackboard::new());
        gate.on_mapped(&ctx, "test_plugin", "foo");
        assert!(gate.mapped);
        gate.on_missing(&ctx, "test_plugin", "lost");
        assert!(!gate.mapped);
    }
}
