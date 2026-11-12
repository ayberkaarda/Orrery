add_library(orrery_core STATIC
    module.cpp
    body_store.cpp
    kepler.cpp
    newtonian_gravity.cpp
    integrator.cpp
    conservation.cpp
    state_hash.cpp
    include/orrery/core/module.h
    include/orrery/core/dimension.h
    include/orrery/core/quantity.h
    include/orrery/core/units.h
    include/orrery/core/angle.h
    include/orrery/core/vec3.h
    include/orrery/core/result.h
    include/orrery/core/body.h
    include/orrery/core/body_store.h
    include/orrery/core/sim_state.h
    include/orrery/core/kepler.h
    include/orrery/core/force_model.h
    include/orrery/core/integrator.h
    include/orrery/core/conservation.h
    include/orrery/core/state_hash.h)
add_library(orrery::core ALIAS orrery_core)

target_include_directories(orrery_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_definitions(orrery_core PRIVATE
    ORRERY_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    ORRERY_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    ORRERY_VERSION_PATCH=${PROJECT_VERSION_PATCH})

# Phase 3a: Barnes-Hut tree gravity (oneTBB parallel_for, deterministic per
# body) and the IAS15 adaptive integrator. oneTBB stays a private dependency
# of core (tools/check_deps.py THIRD_PARTY_DEPS).
find_package(TBB CONFIG REQUIRED)
target_sources(orrery_core PRIVATE
    octree.cpp
    barnes_hut.cpp
    ias15.cpp
    include/orrery/core/octree.h
    include/orrery/core/barnes_hut.h
    include/orrery/core/ias15.h)
target_link_libraries(orrery_core PRIVATE TBB::tbb)

# Phase 3b: black holes (Paczynski-Wiita), tidal disruption, horizon
# absorption, collisions and the event log.
target_sources(orrery_core PRIVATE
    black_hole.cpp
    paczynski_wiita_gravity.cpp
    collision.cpp
    horizon.cpp
    tidal.cpp
    event_step.cpp
    include/orrery/core/black_hole.h
    include/orrery/core/paczynski_wiita_gravity.h
    include/orrery/core/event.h
    include/orrery/core/collision.h
    include/orrery/core/horizon.h
    include/orrery/core/tidal.h
    include/orrery/core/event_step.h)

# Phase 4b: immutable snapshot of the observable state read by the renderer.
target_sources(orrery_core PRIVATE
    sim_snapshot.cpp
    include/orrery/core/sim_snapshot.h)

# Phase 5: Schwarzschild null geodesics, the double-precision reference of the
# black-hole ray marcher (graphics-free; the shader lives in orrery_render).
target_sources(orrery_core PRIVATE
    schwarzschild_geodesic.cpp
    include/orrery/core/schwarzschild_geodesic.h)

orrery_configure_target(orrery_core)
