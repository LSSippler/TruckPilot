#pragma once

#include <cstdint>

#pragma pack(push, 1)
struct ShmTelemetryLayout
{
    std::uint32_t magic;            // 0x54505054 ("TPPT")
    std::uint32_t layout_version;   // 2
    std::uint32_t sequence;
    std::uint32_t padding;

    double x;
    double y;
    double z;
    double heading;
    double pitch;
    double roll;
    double speed_ms;
    double engine_rpm;
    double nav_speed_limit_kmh;
    std::uint32_t nav_speed_limit_valid;
    double fuel_liters;
    double odometer_km;
    double cruise_control_speed_kmh;

    // Extended fields (not used by current C# reader)
    float local_velocity[3];
    float local_acceleration[3];
    float effective_throttle;
    float distance_to_lead_m; // offset 144 (f32), -1 if unavailable
    float effective_brake;
    float effective_clutch;
    float input_steering;
    float input_throttle;
    float input_brake;
    float input_clutch;
    std::int32_t engine_gear;
    std::int32_t displayed_gear;
    std::uint8_t hazard_warning;
    std::uint8_t blinker_left;
    std::uint8_t blinker_right;
    std::uint8_t parking_brake;
    std::uint8_t paused;
    std::uint8_t reserved0[3];
    std::uint64_t timestamp_us;
    char game_id[16];
    std::uint32_t game_version;
    std::uint32_t reserved1;

    std::uint8_t reserved[512 - 220];
};
#pragma pack(pop)

static_assert(sizeof(ShmTelemetryLayout) == 512, "ShmTelemetryLayout must be 512 bytes");
