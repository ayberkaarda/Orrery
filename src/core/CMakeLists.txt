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

orrery_configure_target(orrery_core)
