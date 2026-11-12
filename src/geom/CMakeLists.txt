add_library(orrery_geom STATIC
    module.cpp
    include/orrery/geom/module.h)
add_library(orrery::geom ALIAS orrery_geom)

target_include_directories(orrery_geom PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(orrery_geom PUBLIC orrery_core)

# Phase 6a: geometry pipeline (docs/geometry-import.md).
target_sources(orrery_geom PRIVATE
    triangle_mesh.cpp
    mesh_validation.cpp
    mesh_repair.cpp
    primitives.cpp
    naca.cpp
    include/orrery/geom/vector3.h
    include/orrery/geom/triangle_mesh.h
    include/orrery/geom/mesh_validation.h
    include/orrery/geom/mesh_repair.h
    include/orrery/geom/primitives.h
    include/orrery/geom/naca.h)

orrery_configure_target(orrery_geom)
