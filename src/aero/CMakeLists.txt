add_library(orrery_aero STATIC
    module.cpp
    include/orrery/aero/module.h)
add_library(orrery::aero ALIAS orrery_aero)

target_include_directories(orrery_aero PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(orrery_aero PUBLIC orrery_core orrery_geom)

orrery_configure_target(orrery_aero)
