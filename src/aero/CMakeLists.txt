add_library(orrery_aero STATIC
    module.cpp
    include/orrery/aero/module.h)
add_library(orrery::aero ALIAS orrery_aero)

target_include_directories(orrery_aero PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(orrery_aero PUBLIC orrery_core orrery_geom)

orrery_configure_target(orrery_aero)

# Phase 7b: the compute backend interface (IComputeBackend, the host-memory
# NullComputeBackend, binary32 packing of the GPU boundary) and the D2Q9
# lattice Boltzmann solver (CPU reference on oneTBB, GPU driver through
# IComputeBackend). oneTBB is private and graphics-free, as in orrery_core.
find_package(TBB CONFIG REQUIRED)
target_sources(orrery_aero PRIVATE
    compute/compute_backend.cpp
    compute/float32_pack.cpp
    compute/null_compute_backend.cpp
    compute/include/orrery/aero/compute/compute_backend.h
    compute/include/orrery/aero/compute/float32_pack.h
    compute/include/orrery/aero/compute/null_compute_backend.h
    lbm/force_history.cpp
    lbm/lattice_units.cpp
    lbm/lbm2d_common.cpp
    lbm/lbm2d_cpu.cpp
    lbm/lbm2d_gpu.cpp
    lbm/lbm2d_internal.h
    lbm/solid_mask.cpp
    lbm/include/orrery/aero/lbm/d2q9.h
    lbm/include/orrery/aero/lbm/force_history.h
    lbm/include/orrery/aero/lbm/lattice_units.h
    lbm/include/orrery/aero/lbm/lbm2d.h
    lbm/include/orrery/aero/lbm/solid_mask.h)
target_include_directories(orrery_aero PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/compute/include
    ${CMAKE_CURRENT_SOURCE_DIR}/lbm/include)
target_link_libraries(orrery_aero PRIVATE TBB::tbb)
