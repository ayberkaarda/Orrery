add_library(orrery_geom STATIC
    module.cpp
    include/orrery/geom/module.h)
add_library(orrery::geom ALIAS orrery_geom)

target_include_directories(orrery_geom PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(orrery_geom PUBLIC orrery_core)

orrery_configure_target(orrery_geom)
