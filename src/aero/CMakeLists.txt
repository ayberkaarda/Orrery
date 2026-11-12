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

# Phase 7a: panel methods (2D Hess-Smith, 3D source + doublet) with Eigen, a
# private header-only dependency (tools/check_deps.py THIRD_PARTY_DEPS), and
# modified Newtonian impact theory.
find_package(Eigen3 CONFIG REQUIRED)
target_sources(orrery_aero PRIVATE
    panel2d.cpp
    panel3d.cpp
    panel_influence.cpp
    newtonian.cpp
    include/orrery/aero/panel_method.h
    include/orrery/aero/newtonian.h)
target_link_libraries(orrery_aero PRIVATE Eigen3::Eigen)

orrery_configure_target(orrery_aero)
