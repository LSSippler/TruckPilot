#include "channels.h"
#include "telemetry_layout.h"

#include "scssdk_telemetry.h"
#include "eurotrucks2/scssdk_telemetry_eut2.h"

#include <atomic>

namespace {
ShmTelemetryLayout* g_layout = nullptr;
std::atomic<std::uint32_t>* g_sequence = nullptr;
HANDLE g_ready_event = nullptr;
bool g_ready_signaled = false;

scs_telemetry_register_for_channel_t g_register_channel = nullptr;
scs_telemetry_unregister_from_channel_t g_unregister_channel = nullptr;
scs_telemetry_register_for_event_t g_register_event = nullptr;
scs_telemetry_unregister_from_event_t g_unregister_event = nullptr;
scs_log_t g_log = nullptr;

void LogMessage(const scs_log_type_t type, const char* message)
{
    if (g_log) {
        g_log(type, message);
    }
}

bool RegisterEvent(const scs_event_t event, const scs_telemetry_event_callback_t callback)
{
    if (!g_register_event) {
        return false;
    }

    const auto result = g_register_event(event, callback, nullptr);
    if (result != SCS_RESULT_ok) {
        LogMessage(SCS_LOG_TYPE_warning, "TruckPilot: Failed to register event.");
        return false;
    }

    return true;
}

bool RegisterChannel(const scs_string_t name, const scs_value_type_t type, const scs_u32_t flags,
    const scs_telemetry_channel_callback_t callback)
{
    if (!g_register_channel) {
        return false;
    }

    const auto result = g_register_channel(name, SCS_U32_NIL, type, flags, callback, nullptr);
    if (result != SCS_RESULT_ok) {
        LogMessage(SCS_LOG_TYPE_warning, "TruckPilot: Failed to register channel.");
        return false;
    }

    return true;
}

void SCSAPIFUNC OnFrameStart(const scs_event_t event, const void* const event_info, const scs_context_t)
{
    if (event != SCS_TELEMETRY_EVENT_frame_start || !g_layout || !g_sequence) {
        return;
    }

    const auto* info = static_cast<const scs_telemetry_frame_start_t*>(event_info);
    if (info) {
        g_layout->timestamp_us = static_cast<std::uint64_t>(info->simulation_time);
    }

    // Reset optional channel values each frame; channel callbacks will overwrite.
    g_layout->distance_to_lead_m = -1.0f;

    const auto next = g_sequence->fetch_add(1u) + 1u;
    g_layout->sequence = next;

    if (g_ready_event && !g_ready_signaled) {
        SetEvent(g_ready_event);
        g_ready_signaled = true;
    }
}

void SCSAPIFUNC OnDistanceToLeadVehicle(const scs_string_t, const scs_u32_t, const scs_value_t* const value,
    const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    g_layout->distance_to_lead_m = value->value_float.value;
}

void SCSAPIFUNC OnFrameEnd(const scs_event_t event, const void* const, const scs_context_t)
{
    if (event != SCS_TELEMETRY_EVENT_frame_end || !g_layout || !g_sequence) {
        return;
    }

    const auto next = g_sequence->fetch_add(1u) + 1u;
    g_layout->sequence = next;
}

void SCSAPIFUNC OnPauseEvent(const scs_event_t event, const void* const, const scs_context_t)
{
    if (!g_layout) {
        return;
    }

    if (event == SCS_TELEMETRY_EVENT_paused) {
        g_layout->paused = 1u;
    } else if (event == SCS_TELEMETRY_EVENT_started) {
        g_layout->paused = 0u;
    }
}

void SCSAPIFUNC OnWorldPlacement(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    const auto& placement = value->value_dplacement;
    g_layout->x = placement.position.x;
    g_layout->y = placement.position.y;
    g_layout->z = placement.position.z;
    g_layout->heading = static_cast<double>(placement.orientation.heading);
    g_layout->pitch = static_cast<double>(placement.orientation.pitch);
    g_layout->roll = static_cast<double>(placement.orientation.roll);
}

void SCSAPIFUNC OnSpeed(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    g_layout->speed_ms = static_cast<double>(value->value_float.value);
}

void SCSAPIFUNC OnEngineRpm(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    g_layout->engine_rpm = static_cast<double>(value->value_float.value);
}

void SCSAPIFUNC OnNavigationSpeedLimit(const scs_string_t, const scs_u32_t, const scs_value_t* const value,
    const scs_context_t)
{
    if (!g_layout) {
        return;
    }

    if (!value) {
        g_layout->nav_speed_limit_valid = 0u;
        g_layout->nav_speed_limit_kmh = 0.0;
        return;
    }

    g_layout->nav_speed_limit_valid = 1u;
    g_layout->nav_speed_limit_kmh = static_cast<double>(value->value_float.value) * 3.6;
}

void SCSAPIFUNC OnFuel(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    g_layout->fuel_liters = static_cast<double>(value->value_float.value);
}

void SCSAPIFUNC OnOdometer(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    g_layout->odometer_km = static_cast<double>(value->value_float.value);
}

void SCSAPIFUNC OnCruiseControl(const scs_string_t, const scs_u32_t, const scs_value_t* const value,
    const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    g_layout->cruise_control_speed_kmh = static_cast<double>(value->value_float.value) * 3.6;
}

void SCSAPIFUNC OnLocalVelocity(const scs_string_t, const scs_u32_t, const scs_value_t* const value,
    const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    g_layout->local_velocity[0] = value->value_fvector.x;
    g_layout->local_velocity[1] = value->value_fvector.y;
    g_layout->local_velocity[2] = value->value_fvector.z;
}

void SCSAPIFUNC OnLocalAcceleration(const scs_string_t, const scs_u32_t, const scs_value_t* const value,
    const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    g_layout->local_acceleration[0] = value->value_fvector.x;
    g_layout->local_acceleration[1] = value->value_fvector.y;
    g_layout->local_acceleration[2] = value->value_fvector.z;
}

void SCSAPIFUNC OnEffectiveThrottle(const scs_string_t, const scs_u32_t, const scs_value_t* const value,
    const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    g_layout->effective_throttle = value->value_float.value;
}

void SCSAPIFUNC OnEffectiveBrake(const scs_string_t, const scs_u32_t, const scs_value_t* const value,
    const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    g_layout->effective_brake = value->value_float.value;
}

void SCSAPIFUNC OnEffectiveClutch(const scs_string_t, const scs_u32_t, const scs_value_t* const value,
    const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    g_layout->effective_clutch = value->value_float.value;
}

void SCSAPIFUNC OnInputSteering(const scs_string_t, const scs_u32_t, const scs_value_t* const value,
    const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    g_layout->input_steering = value->value_float.value;
}

void SCSAPIFUNC OnInputThrottle(const scs_string_t, const scs_u32_t, const scs_value_t* const value,
    const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    g_layout->input_throttle = value->value_float.value;
}

void SCSAPIFUNC OnInputBrake(const scs_string_t, const scs_u32_t, const scs_value_t* const value,
    const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    g_layout->input_brake = value->value_float.value;
}

void SCSAPIFUNC OnInputClutch(const scs_string_t, const scs_u32_t, const scs_value_t* const value,
    const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    g_layout->input_clutch = value->value_float.value;
}

void SCSAPIFUNC OnEngineGear(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    g_layout->engine_gear = value->value_s32.value;
}

void SCSAPIFUNC OnDisplayedGear(const scs_string_t, const scs_u32_t, const scs_value_t* const value,
    const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    g_layout->displayed_gear = value->value_s32.value;
}

void SCSAPIFUNC OnHazardWarning(const scs_string_t, const scs_u32_t, const scs_value_t* const value,
    const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    g_layout->hazard_warning = value->value_bool.value ? 1u : 0u;
}

void SCSAPIFUNC OnLeftBlinker(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    g_layout->blinker_left = value->value_bool.value ? 1u : 0u;
}

void SCSAPIFUNC OnRightBlinker(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    g_layout->blinker_right = value->value_bool.value ? 1u : 0u;
}

void SCSAPIFUNC OnParkingBrake(const scs_string_t, const scs_u32_t, const scs_value_t* const value, const scs_context_t)
{
    if (!g_layout || !value) {
        return;
    }

    g_layout->parking_brake = value->value_bool.value ? 1u : 0u;
}

} // namespace

void InitChannels(const scs_telemetry_init_params_t* params, ShmTelemetryLayout* layout,
    std::atomic<std::uint32_t>* sequence, HANDLE ready_event)
{
    const auto* params_v100 = static_cast<const scs_telemetry_init_params_v100_t*>(params);
    g_register_event = params_v100->register_for_event;
    g_unregister_event = params_v100->unregister_from_event;
    g_register_channel = params_v100->register_for_channel;
    g_unregister_channel = params_v100->unregister_from_channel;
    g_log = params_v100->common.log;

    g_layout = layout;
    g_sequence = sequence;
    g_ready_event = ready_event;
    g_ready_signaled = false;

    RegisterEvent(SCS_TELEMETRY_EVENT_frame_start, OnFrameStart);
    RegisterEvent(SCS_TELEMETRY_EVENT_frame_end, OnFrameEnd);
    RegisterEvent(SCS_TELEMETRY_EVENT_paused, OnPauseEvent);
    RegisterEvent(SCS_TELEMETRY_EVENT_started, OnPauseEvent);

    const scs_u32_t each_frame = SCS_TELEMETRY_CHANNEL_FLAG_each_frame;
    const scs_u32_t each_frame_no_value = SCS_TELEMETRY_CHANNEL_FLAG_each_frame | SCS_TELEMETRY_CHANNEL_FLAG_no_value;

    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_world_placement, SCS_VALUE_TYPE_dplacement, each_frame, OnWorldPlacement);
    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_speed, SCS_VALUE_TYPE_float, each_frame, OnSpeed);
    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_engine_rpm, SCS_VALUE_TYPE_float, each_frame, OnEngineRpm);
    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_navigation_speed_limit, SCS_VALUE_TYPE_float, each_frame_no_value,
        OnNavigationSpeedLimit);
    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_fuel, SCS_VALUE_TYPE_float, each_frame, OnFuel);
    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_odometer, SCS_VALUE_TYPE_float, each_frame, OnOdometer);
    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_cruise_control, SCS_VALUE_TYPE_float, each_frame, OnCruiseControl);

    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_local_linear_velocity, SCS_VALUE_TYPE_fvector, each_frame,
        OnLocalVelocity);
    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_local_linear_acceleration, SCS_VALUE_TYPE_fvector, each_frame,
        OnLocalAcceleration);

    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_effective_throttle, SCS_VALUE_TYPE_float, each_frame,
        OnEffectiveThrottle);
#ifdef SCS_TELEMETRY_TRUCK_CHANNEL_distance_to_lead_vehicle
    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_distance_to_lead_vehicle, SCS_VALUE_TYPE_float, each_frame,
        OnDistanceToLeadVehicle);
#endif
    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_effective_brake, SCS_VALUE_TYPE_float, each_frame, OnEffectiveBrake);
    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_effective_clutch, SCS_VALUE_TYPE_float, each_frame, OnEffectiveClutch);

    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_input_steering, SCS_VALUE_TYPE_float, each_frame, OnInputSteering);
    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_input_throttle, SCS_VALUE_TYPE_float, each_frame, OnInputThrottle);
    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_input_brake, SCS_VALUE_TYPE_float, each_frame, OnInputBrake);
    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_input_clutch, SCS_VALUE_TYPE_float, each_frame, OnInputClutch);

    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_engine_gear, SCS_VALUE_TYPE_s32, each_frame, OnEngineGear);
    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_displayed_gear, SCS_VALUE_TYPE_s32, each_frame, OnDisplayedGear);

    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_hazard_warning, SCS_VALUE_TYPE_bool, each_frame, OnHazardWarning);
    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_lblinker, SCS_VALUE_TYPE_bool, each_frame, OnLeftBlinker);
    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_rblinker, SCS_VALUE_TYPE_bool, each_frame, OnRightBlinker);
    RegisterChannel(SCS_TELEMETRY_TRUCK_CHANNEL_parking_brake, SCS_VALUE_TYPE_bool, each_frame, OnParkingBrake);
}

void ShutdownChannels()
{
    if (!g_unregister_event || !g_unregister_channel) {
        return;
    }

    g_unregister_event(SCS_TELEMETRY_EVENT_frame_start);
    g_unregister_event(SCS_TELEMETRY_EVENT_frame_end);
    g_unregister_event(SCS_TELEMETRY_EVENT_paused);
    g_unregister_event(SCS_TELEMETRY_EVENT_started);

    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_world_placement, SCS_U32_NIL, SCS_VALUE_TYPE_dplacement);
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_speed, SCS_U32_NIL, SCS_VALUE_TYPE_float);
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_engine_rpm, SCS_U32_NIL, SCS_VALUE_TYPE_float);
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_navigation_speed_limit, SCS_U32_NIL, SCS_VALUE_TYPE_float);
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_fuel, SCS_U32_NIL, SCS_VALUE_TYPE_float);
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_odometer, SCS_U32_NIL, SCS_VALUE_TYPE_float);
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_cruise_control, SCS_U32_NIL, SCS_VALUE_TYPE_float);
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_local_linear_velocity, SCS_U32_NIL, SCS_VALUE_TYPE_fvector);
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_local_linear_acceleration, SCS_U32_NIL, SCS_VALUE_TYPE_fvector);
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_effective_throttle, SCS_U32_NIL, SCS_VALUE_TYPE_float);
#ifdef SCS_TELEMETRY_TRUCK_CHANNEL_distance_to_lead_vehicle
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_distance_to_lead_vehicle, SCS_U32_NIL, SCS_VALUE_TYPE_float);
#endif
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_effective_brake, SCS_U32_NIL, SCS_VALUE_TYPE_float);
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_effective_clutch, SCS_U32_NIL, SCS_VALUE_TYPE_float);
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_input_steering, SCS_U32_NIL, SCS_VALUE_TYPE_float);
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_input_throttle, SCS_U32_NIL, SCS_VALUE_TYPE_float);
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_input_brake, SCS_U32_NIL, SCS_VALUE_TYPE_float);
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_input_clutch, SCS_U32_NIL, SCS_VALUE_TYPE_float);
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_engine_gear, SCS_U32_NIL, SCS_VALUE_TYPE_s32);
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_displayed_gear, SCS_U32_NIL, SCS_VALUE_TYPE_s32);
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_hazard_warning, SCS_U32_NIL, SCS_VALUE_TYPE_bool);
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_lblinker, SCS_U32_NIL, SCS_VALUE_TYPE_bool);
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_rblinker, SCS_U32_NIL, SCS_VALUE_TYPE_bool);
    g_unregister_channel(SCS_TELEMETRY_TRUCK_CHANNEL_parking_brake, SCS_U32_NIL, SCS_VALUE_TYPE_bool);
}
