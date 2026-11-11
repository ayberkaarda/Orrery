add_library(orrery_render STATIC
    module.cpp
    include/orrery/render/module.h)
add_library(orrery::render ALIAS orrery_render)

target_include_directories(orrery_render PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(orrery_render PUBLIC orrery_core orrery_geom orrery_aero)

orrery_configure_target(orrery_render)
