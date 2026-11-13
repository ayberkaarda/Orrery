add_library(orrery_atmo STATIC
    module.cpp
    include/orrery/atmo/module.h)
add_library(orrery::atmo ALIAS orrery_atmo)

target_include_directories(orrery_atmo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(orrery_atmo PUBLIC orrery_core)

# Phase 9a: atmosphere models (US Standard Atmosphere 1976, Mars, exponential),
# the Cd(Mach) table sampled from the aerodynamic solvers, atmospheric drag as
# a core force model and the entry experiment that wires them together.
# orrery_aero is public: the table sampler's request carries aero types.
target_sources(orrery_atmo PRIVATE
    layers.h
    convert.h
    entry_internal.h
    atmosphere.cpp
    us76.cpp
    planet.cpp
    aero_table.cpp
    cd_sampling.cpp
    drag.cpp
    entry.cpp
    entry_impl.cpp
    entry_record.cpp
    include/orrery/atmo/error.h
    include/orrery/atmo/atmosphere.h
    include/orrery/atmo/planet.h
    include/orrery/atmo/aero_table.h
    include/orrery/atmo/drag.h
    include/orrery/atmo/entry.h)
target_link_libraries(orrery_atmo PUBLIC orrery_aero)

orrery_configure_target(orrery_atmo)
