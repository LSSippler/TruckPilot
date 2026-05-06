#include "channels.h"
#include "shared_memory.h"
#include "telemetry_layout.h"

#include "scssdk_telemetry.h"

#include <atomic>
#include <cstring>

namespace {
SharedMemory g_shared_memory;
ShmTelemetryLayout* g_layout = nullptr;
std::atomic<std::uint32_t> g_sequence{0};
bool g_initialized = false;
}

extern "C" {

SCSAPI_RESULT scs_telemetry_init(const scs_u32_t version, const scs_telemetry_init_params_t* const params)
{
    if (g_initialized) {
        return SCS_RESULT_ok;
    }
    if (version < SCS_TELEMETRY_VERSION_1_00 || version > SCS_TELEMETRY_VERSION_CURRENT) {
        return SCS_RESULT_unsupported;
    }

    if (!params) {
        return SCS_RESULT_invalid_parameter;
    }

    if (!g_shared_memory.Create()) {
        return SCS_RESULT_generic_error;
    }

    g_layout = static_cast<ShmTelemetryLayout*>(g_shared_memory.Data());
    if (!g_layout) {
        return SCS_RESULT_generic_error;
    }

    std::memset(g_layout, 0, sizeof(ShmTelemetryLayout));
    g_layout->magic = 0x54505054u;
    g_layout->layout_version = 2u;
    g_layout->sequence = 0u;
    g_layout->nav_speed_limit_valid = 0u;

    const auto* params_v100 = static_cast<const scs_telemetry_init_params_v100_t*>(params);
    if (params_v100->common.game_id) {
        std::strncpy(g_layout->game_id, params_v100->common.game_id, sizeof(g_layout->game_id) - 1);
    }
    g_layout->game_version = params_v100->common.game_version;

    g_sequence.store(0u);
    InitChannels(params, g_layout, &g_sequence, g_shared_memory.EventHandle());

    if (g_shared_memory.EventHandle()) {
        SetEvent(g_shared_memory.EventHandle());
    }

    g_initialized = true;
    return SCS_RESULT_ok;
}

SCSAPI_VOID scs_telemetry_shutdown(void)
{
    if (!g_initialized) {
        return;
    }

    ShutdownChannels();
    g_initialized = false;
}

}
