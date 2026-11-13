# Benchmarks and the nightly validation (docs/performance.md, docs/ci.md).
# Built with the tests so every CI build compiles them with the project's
# warnings, but never registered with ctest:
#   - orrery_bench_<suite>: the section 8 measurements, one executable per
#     module (Catch2 test cases run by common/bench_main.cpp, which writes the
#     results as JSON with --json <file>). run_benchmarks.py runs them all,
#     adds the application measurements (cold start, memory, size, frame
#     rate) and merges the JSON; make_report.py turns it into a table.
#   - orrery_nightly_lbm_validation: the large CPU lattice Boltzmann cases of
#     the nightly workflow.
find_package(nlohmann_json CONFIG REQUIRED)

function(orrery_add_bench target suite)
    add_executable(${target} common/bench_main.cpp common/bench_record.h ${ARGN})
    target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/common)
    target_compile_definitions(${target} PRIVATE
        ORRERY_BENCH_SUITE="${suite}"
        ORRERY_BENCH_LTO=$<BOOL:$<TARGET_PROPERTY:INTERPROCEDURAL_OPTIMIZATION>>)
    orrery_configure_target(${target})
endfunction()

orrery_add_bench(orrery_bench_core core core/bench_core.cpp)
target_link_libraries(orrery_bench_core PRIVATE orrery_core
    Catch2::Catch2 nlohmann_json::nlohmann_json)

orrery_add_bench(orrery_bench_geom geom geom/bench_geom.cpp)
target_link_libraries(orrery_bench_geom PRIVATE orrery_geom
    Catch2::Catch2 nlohmann_json::nlohmann_json)
target_compile_definitions(orrery_bench_geom PRIVATE
    ORRERY_GEOM_FIXTURE_DIR="${PROJECT_SOURCE_DIR}/tests/geom/fixtures"
    ORRERY_GEOM_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/bench_output")

orrery_add_bench(orrery_bench_aero aero aero/bench_aero.cpp)
target_link_libraries(orrery_bench_aero PRIVATE orrery_aero orrery_geom
    Catch2::Catch2 nlohmann_json::nlohmann_json)

# The GPU kernels run through the OpenGL compute backend of orrery_render.
orrery_add_bench(orrery_bench_lbm_gpu lbm_gpu aero/bench_lbm_gpu.cpp)
target_link_libraries(orrery_bench_lbm_gpu PRIVATE orrery_render
    Catch2::Catch2 nlohmann_json::nlohmann_json)

add_executable(orrery_nightly_lbm_validation nightly/nightly_lbm_validation.cpp)
target_link_libraries(orrery_nightly_lbm_validation PRIVATE orrery_aero orrery_geom
    Catch2::Catch2WithMain)
orrery_configure_target(orrery_nightly_lbm_validation)
