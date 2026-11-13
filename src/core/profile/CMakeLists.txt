# Phase 10a: profiling zones and the built-in trace sink (docs/observability.md).
#
# orrery_profile is graphics-free and free of single precision like core; core
# links it publicly, so every module can place zones. The zone macros compile
# to nothing unless ORRERY_PROFILING is on; ORRERY_TRACY additionally turns
# every zone into a Tracy zone (Tracy client from the vcpkg manifest feature
# "profiling", approved for this target only: tools/check_deps.py
# THIRD_PARTY_DEPS). Both options are off by default and in every CI preset.
add_library(orrery_profile STATIC
    profile.cpp
    trace_json.cpp
    include/orrery/core/profile.h
    include/orrery/core/trace_capture.h)
add_library(orrery::profile ALIAS orrery_profile)
target_include_directories(orrery_profile PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(ORRERY_PROFILING)
    target_compile_definitions(orrery_profile PUBLIC ORRERY_PROFILING=1)
endif()
if(ORRERY_TRACY)
    find_package(Tracy CONFIG REQUIRED)
    target_compile_definitions(orrery_profile PUBLIC ORRERY_PROFILING_TRACY=1)
    target_link_libraries(orrery_profile PUBLIC Tracy::TracyClient)
endif()

orrery_configure_target(orrery_profile)
