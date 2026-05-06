cmake_minimum_required(VERSION 3.20)
project(truckpilot_telemetry LANGUAGES CXX)

add_library(truckpilot_telemetry SHARED
    src/main.cpp
    src/shared_memory.cpp
    src/channels.cpp
)

target_include_directories(truckpilot_telemetry PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/scs_sdk/include
)

target_compile_features(truckpilot_telemetry PRIVATE cxx_std_17)
target_compile_definitions(truckpilot_telemetry PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)

if(MSVC)
    set_property(TARGET truckpilot_telemetry PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded")
    target_compile_options(truckpilot_telemetry PRIVATE /W4 /permissive-)
    target_link_options(truckpilot_telemetry PRIVATE /DEF:${CMAKE_CURRENT_LIST_DIR}/truckpilot_telemetry.def)
endif()

set_target_properties(truckpilot_telemetry PROPERTIES
    OUTPUT_NAME "truckpilot_telemetry"
    PREFIX ""
)

target_link_libraries(truckpilot_telemetry PRIVATE Kernel32 Advapi32)
