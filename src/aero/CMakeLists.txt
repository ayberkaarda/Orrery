add_library(orrery_aero STATIC
    module.cpp
    include/orrery/aero/module.h)
add_library(orrery::aero ALIAS orrery_aero)

target_include_directories(orrery_aero PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(orrery_aero PUBLIC orrery_core orrery_geom)

# Phase 7a: solver interface, flow conditions, results and regime selection.
target_sources(orrery_aero PRIVATE
    flow_conditions.cpp
    aero_result.cpp
    regime.cpp
    include/orrery/aero/flow_conditions.h
    include/orrery/aero/aero_result.h
    include/orrery/aero/regime.h
    include/orrery/aero/aero_solver.h)

orrery_configure_target(orrery_aero)
