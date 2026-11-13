find_package(nlohmann_json 3 CONFIG REQUIRED)

add_library(orrery_scene STATIC
    module.cpp
    json_support.cpp
    json_support.h
    scenario.cpp
    experiment.cpp
    include/orrery/scene/module.h
    include/orrery/scene/error.h
    include/orrery/scene/limits.h
    include/orrery/scene/scenario.h
    include/orrery/scene/experiment.h)
add_library(orrery::scene ALIAS orrery_scene)

target_include_directories(orrery_scene PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(orrery_scene PUBLIC orrery_core orrery_geom orrery_aero orrery_atmo)
# JSON is an implementation detail: no public header includes nlohmann/json.
target_link_libraries(orrery_scene PRIVATE nlohmann_json::nlohmann_json)

# Phase 8: typed records of aero-domain experiments (geometry reference,
# free-stream conditions, solver choice, result) on the generic schema.
target_sources(orrery_scene PRIVATE
    aero_experiment.cpp
    include/orrery/scene/aero_experiment.h)

orrery_configure_target(orrery_scene)
