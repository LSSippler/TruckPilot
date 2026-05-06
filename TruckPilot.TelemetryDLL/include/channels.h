#pragma once

#include "telemetry_layout.h"
#include <windows.h>
#include <atomic>

struct scs_telemetry_init_params_t;

void InitChannels(const scs_telemetry_init_params_t* params, ShmTelemetryLayout* layout, std::atomic<std::uint32_t>* sequence, HANDLE ready_event);
void ShutdownChannels();
