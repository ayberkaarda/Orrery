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
    winding_number.cpp
    voxel_grid.cpp
    include/orrery/geom/vector3.h
    include/orrery/geom/triangle_mesh.h
    include/orrery/geom/mesh_validation.h
    include/orrery/geom/mesh_repair.h
    include/orrery/geom/primitives.h
    include/orrery/geom/naca.h
    include/orrery/geom/winding_number.h
    include/orrery/geom/voxel_grid.h)

# Phase 6a: model import through assimp, a private dependency of geom
# (tools/check_deps.py THIRD_PARTY_DEPS), behind a validation gate (ADR-018).
find_package(assimp CONFIG REQUIRED)
target_sources(orrery_geom PRIVATE
    mesh_import.cpp
    model_prescan.cpp
    model_prescan.h
    include/orrery/geom/mesh_import.h)
target_link_libraries(orrery_geom PRIVATE assimp::assimp)

orrery_configure_target(orrery_geom)
