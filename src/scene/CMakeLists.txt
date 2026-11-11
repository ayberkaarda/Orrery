add_library(orrery_scene STATIC
    module.cpp
    include/orrery/scene/module.h)
add_library(orrery::scene ALIAS orrery_scene)

target_include_directories(orrery_scene PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(orrery_scene PUBLIC orrery_core orrery_geom orrery_aero orrery_atmo)

orrery_configure_target(orrery_scene)
