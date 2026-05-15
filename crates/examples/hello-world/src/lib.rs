//! Hello World demo plugin — updated for Phase 6 API.

use truckpilot_plugin_api::{ControlOutput, Plugin, PluginContext, Telemetry};

#[derive(Default)]
struct HelloWorldPlugin;

impl Plugin for HelloWorldPlugin {
    fn name(&self) -> &str {
        "hello-world"
    }
    fn version(&self) -> &str {
        "0.1.0"
    }
    fn settings_schema(&self) -> &str {
        r#"{"type":"object","properties":{}}"#
    }

    fn on_load(&mut self, ctx: &PluginContext) {
        tracing::info!("[{}] loaded", ctx.plugin_name);
    }

    fn on_unload(&mut self) {
        tracing::info!("[hello-world] unloaded");
    }

    fn tick(
        &mut self,
        telemetry: Option<&Telemetry>,
        output: &mut ControlOutput,
        ctx: &PluginContext,
    ) {
        // 1 Hz log via the scheduler-supplied tick counter.
        if ctx.tick_count > 0 && ctx.tick_count.is_multiple_of(50) {
            if let Some(t) = telemetry {
                tracing::info!("[hello-world] speed={:.1} km/h", t.speed_ms * 3.6);
            } else {
                tracing::info!("[hello-world] no telemetry");
            }
        }
        if telemetry.map(|t| t.speed_ms > 5.0).unwrap_or(false) {
            output.steering = 0.1;
        }
    }
}

truckpilot_plugin_api::export_plugin!(HelloWorldPlugin);
