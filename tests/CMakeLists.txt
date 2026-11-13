find_package(Catch2 3 CONFIG REQUIRED)
include(Catch)

# One test executable per library so each links only what it exercises.
function(orrery_add_module_test module)
    set(target orrery_test_${module})
    set(discover_props)
    # The render tests call glfwInit, which leaks a few libX11/libxcb
    # allocations when no display exists; hide only that (see lsan.supp).
    if(ORRERY_SANITIZE AND NOT MSVC AND module STREQUAL "render")
        set(discover_props PROPERTIES ENVIRONMENT
            "LSAN_OPTIONS=suppressions=${CMAKE_CURRENT_SOURCE_DIR}/lsan.supp:print_suppressions=0")
    endif()
    add_executable(${target} smoke/test_${module}.cpp)
    target_link_libraries(${target} PRIVATE orrery_${module} Catch2::Catch2WithMain)
    target_compile_definitions(${target} PRIVATE
        ORRERY_EXPECTED_VERSION="${PROJECT_VERSION}")
    orrery_configure_target(${target})
    catch_discover_tests(${target} DISCOVERY_MODE PRE_TEST ${discover_props})
endfunction()

foreach(module IN ITEMS geom atmo aero render)
    orrery_add_module_test(${module})
endforeach()

# Phase 6a: geometry pipeline tests. Fixtures live in geom/fixtures;
# model files written by the tests go to the build tree.
target_sources(orrery_test_geom PRIVATE
    geom/geom_test_support.h
    geom/test_primitives.cpp
    geom/test_naca.cpp
    geom/test_import.cpp
    geom/test_repair.cpp
    geom/test_voxel.cpp
    geom/test_panel.cpp
    geom/test_reference.cpp)
target_compile_definitions(orrery_test_geom PRIVATE
    ORRERY_GEOM_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/geom/fixtures"
    ORRERY_GEOM_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/geom_output")

# Phase 7a: aero solver interface, regime selection and analytic solver tests.
target_sources(orrery_test_aero PRIVATE
    aero/test_regime.cpp
    aero/test_panel.cpp
    aero/test_newtonian.cpp)
add_subdirectory(aero_validation)

# Phase 9a: atmosphere models, the Cd(Mach) table (literal data and sampled
# from the aero solvers), atmospheric drag and the entry experiment.
target_sources(orrery_test_atmo PRIVATE
    atmo/test_atmosphere.cpp
    atmo/test_aero_table.cpp
    atmo/test_drag.cpp
    atmo/test_entry.cpp)

# Phase 4a: render foundation. Unit tests (camera, PNG, SSIM, tone map,
# shader embedding) always run. Tests tagged [gl] need an OpenGL 4.6 core
# context and are reported as skipped when none can be created (set
# ORRERY_REQUIRE_GL=1 to make that a failure). The golden reference is
# regenerated only on request: ORRERY_UPDATE_GOLDEN=1 orrery_test_render "[golden]".
target_sources(orrery_test_render PRIVATE
    render/gl_test_support.h
    render/test_camera.cpp
    render/test_png.cpp
    render/test_ssim.cpp
    render/test_tone_map.cpp
    render/test_shaders.cpp
    render/test_gl.cpp
    render/test_golden.cpp)
target_include_directories(orrery_test_render PRIVATE ${PROJECT_BINARY_DIR}/src/render/generated)
target_compile_definitions(orrery_test_render PRIVATE
    ORRERY_RENDER_SHADER_DIR="${PROJECT_SOURCE_DIR}/src/render/shaders"
    ORRERY_RENDER_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/render/golden"
    ORRERY_RENDER_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/render_output")

# Phase 4a review fixes: renderer lifetime across contexts, image size
# overflow, and logger reentrancy.
target_sources(orrery_test_render PRIVATE
    render/test_renderer_lifetime.cpp
    render/test_image.cpp
    render/test_log.cpp)

# Phase 4b: space scene visuals and the UI layer. CPU tests (icosphere, LOD,
# scale policy, trails, starfield, theme, headless UI panels) always run; the
# golden snapshots and the frame-time measurement are tagged [gl]. The J2000
# scenario is loaded through orrery_scene.
target_sources(orrery_test_render PRIVATE
    render/space_test_support.h
    render/test_icosphere.cpp
    render/test_body_visuals.cpp
    render/test_starfield.cpp
    render/test_trails.cpp
    render/test_ui.cpp
    render/test_space_golden.cpp
    render/test_space_render.cpp)

# Phase 5: black-hole lensing. CPU tests (black-body spectrum, lens selection,
# settings) always run; the GPU-against-CPU geodesic comparison, the golden
# snapshots (Einstein ring and shadow, Doppler-asymmetric disk) and the
# 1080p frame time are tagged [gl].
target_sources(orrery_test_render PRIVATE
    render/black_hole_test_support.h
    render/test_black_hole.cpp
    render/test_black_hole_render.cpp)
target_link_libraries(orrery_test_render PRIVATE orrery_scene)
target_compile_definitions(orrery_test_render PRIVATE
    ORRERY_SCENARIO_DIR="${PROJECT_SOURCE_DIR}/assets/scenarios")

# Phase 8a: aero visualisation. CPU tests (colour maps, fields and section
# sampling, surfaces, marching cubes, streamlines, the turntable camera)
# always run; the GPU kernels against their CPU references and the golden
# snapshots (NACA 0012 Cp, cylinder vortex street) are tagged [gl]. Goldens
# live in render/golden/aero and are written only with ORRERY_UPDATE_GOLDEN=1.
target_sources(orrery_test_render PRIVATE
    render/aero/aero_test_support.h
    render/aero/test_aero_colormap.cpp
    render/aero/test_aero_headers.cpp
    render/aero/test_aero_field.cpp
    render/aero/test_marching_cubes.cpp
    render/aero/test_streamlines.cpp
    render/aero/test_aero_view.cpp
    render/aero/test_turntable.cpp)

# Phase 9b: the entry view (geometry without a GPU, drawing tagged [gl]).
target_sources(orrery_test_render PRIVATE render/entry/test_entry_view.cpp)

# Shader embedding of overlong lines: fixture shaders are generated at build
# time and embedded by the production script into a header that the test
# compiles (MSVC limits each string literal to 16380 bytes). The fixture
# translation unit is compiled as UTF-8 so its non-ASCII fixture keeps its bytes.
set(ORRERY_EMBED_FIXTURE_DIR ${CMAKE_CURRENT_BINARY_DIR}/render_embed_fixture)
set(ORRERY_EMBED_FIXTURE_SHADERS
    ${ORRERY_EMBED_FIXTURE_DIR}/long_line.frag.glsl
    ${ORRERY_EMBED_FIXTURE_DIR}/many_lines.frag.glsl
    ${ORRERY_EMBED_FIXTURE_DIR}/utf8_line.frag.glsl)
set(ORRERY_EMBED_FIXTURE_HEADER
    ${ORRERY_EMBED_FIXTURE_DIR}/include/orrery_test/embedded_fixture_shaders.h)
add_custom_command(
    OUTPUT ${ORRERY_EMBED_FIXTURE_HEADER} ${ORRERY_EMBED_FIXTURE_SHADERS}
    COMMAND ${CMAKE_COMMAND} -DOUTPUT_DIR=${ORRERY_EMBED_FIXTURE_DIR}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/render/make_embed_fixtures.cmake
    COMMAND ${CMAKE_COMMAND}
        -DOUTPUT=${ORRERY_EMBED_FIXTURE_HEADER}
        -DNAMESPACE=orrery::render::embed_fixture
        "-DSHADERS=${ORRERY_EMBED_FIXTURE_SHADERS}"
        -P ${PROJECT_SOURCE_DIR}/src/render/cmake/embed_shaders.cmake
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/render/make_embed_fixtures.cmake
        ${PROJECT_SOURCE_DIR}/src/render/cmake/embed_shaders.cmake
    COMMENT "Generating shader embedding fixtures"
    VERBATIM)
target_sources(orrery_test_render PRIVATE
    render/test_shader_embedding.cpp
    ${ORRERY_EMBED_FIXTURE_HEADER})
target_include_directories(orrery_test_render PRIVATE ${ORRERY_EMBED_FIXTURE_DIR}/include)
target_compile_definitions(orrery_test_render PRIVATE
    ORRERY_EMBED_FIXTURE_DIR="${ORRERY_EMBED_FIXTURE_DIR}"
    ORRERY_EMBED_FIXTURE_HEADER="${ORRERY_EMBED_FIXTURE_HEADER}"
    ORRERY_RENDER_SHADER_HEADER="${PROJECT_BINARY_DIR}/src/render/generated/orrery/render/embedded_shaders.h")
set_source_files_properties(render/test_shader_embedding.cpp PROPERTIES
    COMPILE_OPTIONS "$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

# Negative test of the dependency checker: core, geom and atmo must still be
# rejected when they link a graphics library, while render may link GLFW/glad.
find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_test(NAME check_deps.graphics_boundary
        COMMAND Python3::Interpreter ${CMAKE_CURRENT_SOURCE_DIR}/render/test_check_deps_graphics.py)
    # The PNG writer checked by an independent decoder (Python's zlib).
    add_test(NAME render.png_python_zlib
        COMMAND Python3::Interpreter ${CMAKE_CURRENT_SOURCE_DIR}/render/check_png_zlib.py
            $<TARGET_FILE:orrery_test_render> ${CMAKE_CURRENT_BINARY_DIR}/render_output)
else()
    message(WARNING "Python 3 not found: check_deps.graphics_boundary and "
        "render.png_python_zlib tests not registered")
endif()

# Phase 6b: URL model loader (orrery_test_geom_net, tests named "net.*").
add_subdirectory(geom/net)

# Phase 10a: profiling zones and trace sink (orrery_test_profile) and the
# --trace command-line tests ("profile.*").
add_subdirectory(profile)

# Phase 11: documentation links, physics formula index and APPROXIMATION
# coverage (docs.check, tools/check_docs.py).
add_subdirectory(docs)

# Core: unit and property tests (including the former smoke test).
# allocation_failure_injection.cpp replaces the global operator new/delete
# family of each executable that lists it (allocation-failure tests).
add_executable(orrery_test_core
    core/test_module_info.cpp
    core/test_quantity.cpp
    core/test_units.cpp
    core/allocation_failure_injection.h
    core/allocation_failure_injection.cpp
    core/test_body_store.cpp
    core/test_sim_state.cpp
    core/test_kepler.cpp
    core/gravity_test_support.h
    core/test_gravity.cpp
    core/test_integrators.cpp
    core/test_conservation.cpp
    core/test_determinism.cpp)
target_link_libraries(orrery_test_core PRIVATE orrery_core Catch2::Catch2WithMain)
target_compile_definitions(orrery_test_core PRIVATE
    ORRERY_EXPECTED_VERSION="${PROJECT_VERSION}")
orrery_configure_target(orrery_test_core)
catch_discover_tests(orrery_test_core DISCOVERY_MODE PRE_TEST)

# Phase 3b: black holes, collisions and events.
target_sources(orrery_test_core PRIVATE
    core/test_black_hole.cpp
    core/test_collisions.cpp
    core/test_events.cpp)

# Phase 3a: Barnes-Hut and IAS15. The force timing benchmark is tagged
# [.benchmark] (hidden): it is not discovered by ctest and runs with
# `orrery_test_core "[benchmark]"`.
target_sources(orrery_test_core PRIVATE
    core/test_barnes_hut.cpp
    core/test_ias15.cpp)

# Phase 4b: immutable simulation snapshot read by the renderer.
target_sources(orrery_test_core PRIVATE
    core/test_sim_snapshot.cpp)

# Phase 5: Schwarzschild null geodesics (reference of the black-hole shader).
target_sources(orrery_test_core PRIVATE
    core/test_schwarzschild_geodesic.cpp)

# Phase 3a embedded scenarios: loaded with orrery_scene, integrated with core.
add_executable(orrery_test_phase3a_scenarios
    core/gravity_test_support.h
    core/test_scenarios_phase3a.cpp)
target_link_libraries(orrery_test_phase3a_scenarios PRIVATE orrery_scene Catch2::Catch2WithMain)
target_compile_definitions(orrery_test_phase3a_scenarios PRIVATE
    ORRERY_SCENARIO_DIR="${PROJECT_SOURCE_DIR}/assets/scenarios")
orrery_configure_target(orrery_test_phase3a_scenarios)
catch_discover_tests(orrery_test_phase3a_scenarios DISCOVERY_MODE PRE_TEST)

# Scene: scenario and experiment formats (including the former smoke test).
# Fixtures and bundled scenarios are located through compile-time paths.
add_executable(orrery_test_scene
    smoke/test_scene.cpp
    core/allocation_failure_injection.h
    core/allocation_failure_injection.cpp
    scene/scene_test_support.h
    scene/test_scenario_load.cpp
    scene/test_scenario_validation.cpp
    scene/test_scenario_roundtrip.cpp
    scene/test_experiment.cpp
    scene/test_aero_experiment.cpp
    scene/test_entry_experiment.cpp
    scene/test_scene_limits.cpp)
target_link_libraries(orrery_test_scene PRIVATE orrery_scene Catch2::Catch2WithMain)
target_compile_definitions(orrery_test_scene PRIVATE
    ORRERY_SCENE_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/scene/fixtures"
    ORRERY_SCENARIO_DIR="${PROJECT_SOURCE_DIR}/assets/scenarios")
orrery_configure_target(orrery_test_scene)
catch_discover_tests(orrery_test_scene DISCOVERY_MODE PRE_TEST)

# Phase 4c: the application loop. orrery_test_sim links only the graphics-free
# simulation library (triple buffer, command queue, accumulator, physics
# thread, experiments); orrery_test_app covers the command line, the headless
# renderer (golden: ORRERY_UPDATE_GOLDEN=1 orrery_test_app "[golden]") and the
# application bar. Bundled assets are found through ORRERY_SCENARIO_DIR.
add_executable(orrery_test_sim
    app/app_test_support.h
    core/allocation_failure_injection.h
    core/allocation_failure_injection.cpp
    app/test_triple_buffer.cpp
    app/test_command_queue.cpp
    app/test_simulation.cpp
    app/test_physics_thread.cpp
    app/test_space_experiment.cpp
    app/test_warp_governor.cpp
    app/test_aero_workspace.cpp)
# Phase 9b: the Entry workspace's graphics-free side (inputs, solver thread,
# comparison and CSV).
target_sources(orrery_test_sim PRIVATE app/test_entry_model.cpp)
# Downloads of experiment models against the loader's mock server
# (tests/geom/net): cancellation, the recorded digest, permission.
target_sources(orrery_test_sim PRIVATE app/test_aero_download.cpp)
target_link_libraries(orrery_test_sim PRIVATE orrery_sim orrery_geom_net test_net_support
    Catch2::Catch2WithMain)
target_compile_definitions(orrery_test_sim PRIVATE
    ORRERY_SCENARIO_DIR="${PROJECT_SOURCE_DIR}/assets/scenarios")
orrery_configure_target(orrery_test_sim)
# RUN_SERIAL: the physics and solver thread tests observe intermediate frames
# of a running thread, which a fully loaded machine (ctest --parallel) can
# make them miss; they run alone.
catch_discover_tests(orrery_test_sim DISCOVERY_MODE PRE_TEST PROPERTIES RUN_SERIAL TRUE)

# The shortcut and journey tests queue keyboard events on the Dear ImGui
# context (app/key_events.h), as the GLFW backend does in the window.
find_package(imgui CONFIG REQUIRED)
add_executable(orrery_test_app
    app/app_test_support.h
    app/key_events.h
    core/allocation_failure_injection.h
    core/allocation_failure_injection.cpp
    app/test_cli.cpp
    app/test_headless.cpp
    app/test_app_bar.cpp
    app/test_frame_stats.cpp
    app/test_config.cpp
    app/test_shortcuts.cpp
    app/test_workspace_journeys.cpp
    app/test_workspace_render.cpp
    app/test_aero_ui.cpp
    app/test_aero_viewport.cpp
    app/test_run_guard.cpp)
# nlohmann_json parses the traces written by run_traced (test_run_guard.cpp).
find_package(nlohmann_json CONFIG REQUIRED)
target_link_libraries(orrery_test_app PRIVATE orrery_app_core imgui::imgui
    nlohmann_json::nlohmann_json Catch2::Catch2WithMain)
target_compile_definitions(orrery_test_app PRIVATE
    ORRERY_SCENARIO_DIR="${PROJECT_SOURCE_DIR}/assets/scenarios"
    ORRERY_RENDER_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/app_output")
# Phase 9b: the Entry workspace under a headless UiContext, and its golden
# snapshot through the headless path (render/golden/entry, [gl][golden]).
target_sources(orrery_test_app PRIVATE
    app/test_entry_golden.cpp
    app/test_entry_workspace.cpp)
target_compile_definitions(orrery_test_app PRIVATE
    ORRERY_RENDER_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/render/golden")
orrery_configure_target(orrery_test_app)
set(app_discover_props)
if(ORRERY_SANITIZE AND NOT MSVC)
    set(app_discover_props PROPERTIES ENVIRONMENT
        "LSAN_OPTIONS=suppressions=${CMAKE_CURRENT_SOURCE_DIR}/lsan.supp:print_suppressions=0")
endif()
catch_discover_tests(orrery_test_app DISCOVERY_MODE PRE_TEST ${app_discover_props})

# The executable itself: version, help, module graph, and the documented exit
# codes of the command line (2 for usage errors, 1 for missing files).
add_test(NAME app.version COMMAND orrery_app --version)
set_tests_properties(app.version PROPERTIES
    PASS_REGULAR_EXPRESSION "Orrery ${PROJECT_VERSION}"
    FAIL_REGULAR_EXPRESSION "modules:")

add_test(NAME app.help COMMAND orrery_app --help)
set_tests_properties(app.help PROPERTIES
    PASS_REGULAR_EXPRESSION "--headless --experiment <name-or-path> --out <file.png>")

add_test(NAME app.module_graph COMMAND orrery_app --modules)
set_tests_properties(app.module_graph PROPERTIES
    PASS_REGULAR_EXPRESSION "render -> core geom aero")

# Exit codes are checked by a small CMake script (PASS_REGULAR_EXPRESSION
# alone ignores them).
set(ORRERY_EXPECT_EXIT ${CMAKE_CURRENT_SOURCE_DIR}/app/expect_exit.cmake)
add_test(NAME app.cli_unknown_option
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:orrery_app> -DEXPECT_CODE=2
        "-DEXPECT_STDERR=unknown option '--bogus'" "-DARGS=--bogus"
        -P ${ORRERY_EXPECT_EXIT})
add_test(NAME app.cli_headless_without_out
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:orrery_app> -DEXPECT_CODE=2
        "-DEXPECT_STDERR=requires --out" "-DARGS=--headless;--experiment;solar_system"
        -P ${ORRERY_EXPECT_EXIT})
add_test(NAME app.cli_missing_experiment
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:orrery_app> -DEXPECT_CODE=1
        "-DEXPECT_STDERR=experiment 'no_such_experiment' not found"
        "-DARGS=--headless;--experiment;no_such_experiment;--out;${CMAKE_CURRENT_BINARY_DIR}/app_output/never.png"
        -P ${ORRERY_EXPECT_EXIT})
# End to end: renders the bundled solar system experiment. Exit code 3 (no
# OpenGL 4.6 context) is reported as skipped; the Catch2 [gl] tests enforce
# ORRERY_REQUIRE_GL.
add_test(NAME app.headless_png
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:orrery_app> -DEXPECT_CODE=0
        "-DEXPECT_STDOUT=wrote .*app_output/cli_solar_system.png \\(480x270\\)"
        "-DARGS=--headless;--experiment;solar_system;--frame;60;--size;480x270;--out;${CMAKE_CURRENT_BINARY_DIR}/app_output/cli_solar_system.png"
        -DSKIP_CODE=3 -DOUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/app_output
        -P ${ORRERY_EXPECT_EXIT})
set_tests_properties(app.headless_png PROPERTIES SKIP_RETURN_CODE 77)
# Phase 5: the black-hole experiment end to end, with the headless frame-time
# measurement (--benchmark) after the image.
add_test(NAME app.headless_black_hole_benchmark
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:orrery_app> -DEXPECT_CODE=0
        "-DEXPECT_STDOUT=benchmark: 20 frames at 480x270 .*lensing on"
        "-DARGS=--headless;--experiment;black_hole;--size;480x270;--benchmark;20;--out;${CMAKE_CURRENT_BINARY_DIR}/app_output/cli_black_hole.png"
        -DSKIP_CODE=3 -DOUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/app_output
        -P ${ORRERY_EXPECT_EXIT})
set_tests_properties(app.headless_black_hole_benchmark PROPERTIES SKIP_RETURN_CODE 77)
# Phase 8b: an aero experiment end to end (panel method on the bundled
# NACA 0012 section, drawn by the Aero view).
add_test(NAME app.headless_aero_png
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:orrery_app> -DEXPECT_CODE=0
        "-DEXPECT_STDOUT=wrote .*app_output/cli_aero_naca0012.png \\(480x270\\): NACA 0012 section"
        "-DARGS=--headless;--experiment;aero_naca0012_panel;--size;480x270;--out;${CMAKE_CURRENT_BINARY_DIR}/app_output/cli_aero_naca0012.png"
        -DSKIP_CODE=3 -DOUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/app_output
        -P ${ORRERY_EXPECT_EXIT})
set_tests_properties(app.headless_aero_png PROPERTIES SKIP_RETURN_CODE 77)
# Phase 9b: an entry experiment end to end (the Apollo-like entry run to the
# ground, the whole Entry workspace drawn offscreen).
add_test(NAME app.headless_entry_png
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:orrery_app> -DEXPECT_CODE=0
        "-DEXPECT_STDOUT=wrote .*app_output/cli_entry_apollo.png \\(960x540\\): Apollo-like entry, Earth: landed"
        "-DARGS=--headless;--experiment;entry_apollo_like;--size;960x540;--out;${CMAKE_CURRENT_BINARY_DIR}/app_output/cli_entry_apollo.png"
        -DSKIP_CODE=3 -DOUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/app_output
        -P ${ORRERY_EXPECT_EXIT})
set_tests_properties(app.headless_entry_png PROPERTIES SKIP_RETURN_CODE 77)
# Started by its bare name through PATH from another directory, the
# executable still finds the assets next to itself.
add_test(NAME app.bare_name_finds_its_assets
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:orrery_app>
        -DSCENARIO_DIR=${PROJECT_SOURCE_DIR}/assets/scenarios
        -DWORK=${CMAKE_CURRENT_BINARY_DIR}/app_output/bare_name
        -P ${CMAKE_CURRENT_SOURCE_DIR}/app/bare_name_launch.cmake)
set_tests_properties(app.bare_name_finds_its_assets PROPERTIES SKIP_RETURN_CODE 77)
# The terminate handler in a process of its own (it aborts): its report for
# an unhandled exception, and its allocation-free fallback when memory is
# exhausted while the report is formatted (one report, no re-entry).
add_executable(orrery_terminate_probe
    app/terminate_probe.cpp
    core/allocation_failure_injection.h
    core/allocation_failure_injection.cpp)
target_link_libraries(orrery_terminate_probe PRIVATE orrery_app_core)
orrery_configure_target(orrery_terminate_probe)
set(ORRERY_EXPECT_ABORT ${CMAKE_CURRENT_SOURCE_DIR}/app/expect_abort.cmake)
add_test(NAME app.terminate_report
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:orrery_terminate_probe> -DARGS=exception
        "-DEXPECT_STDERR=orrery_app: fatal: unhandled exception: probe failure"
        -P ${ORRERY_EXPECT_ABORT})
# Phase 10: end to end with the crash log - the report reaches standard
# error and exactly one timestamped entry with the version reaches the log,
# whose directory is created.
set(ORRERY_CRASH_DIR ${CMAKE_CURRENT_BINARY_DIR}/app_output/crash)
add_test(NAME app.terminate_writes_crash_log
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:orrery_terminate_probe>
        "-DARGS=exception;${ORRERY_CRASH_DIR}/main/nested/crash.log"
        -DLOG=${ORRERY_CRASH_DIR}/main/nested/crash.log -DLOG_DIR=${ORRERY_CRASH_DIR}/main
        "-DEXPECT_STDERR=orrery_app: fatal: unhandled exception: probe failure"
        "-DEXPECT_LOG=^[0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9]T[0-9:]+Z Orrery probe: unhandled exception: probe failure with a message"
        -P ${ORRERY_EXPECT_ABORT})
add_test(NAME app.terminate_report_out_of_memory
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:orrery_terminate_probe> -DARGS=out_of_memory
        "-DEXPECT_STDERR=orrery_app: fatal: std::terminate \\(the reason could not be formatted\\)"
        -P ${ORRERY_EXPECT_ABORT})
if(ORRERY_SANITIZE AND NOT MSVC)
    # The executable calls glfwInit without a display; see lsan.supp.
    set_tests_properties(app.headless_png app.headless_black_hole_benchmark
        app.headless_aero_png app.headless_entry_png app.bare_name_finds_its_assets PROPERTIES ENVIRONMENT
        "LSAN_OPTIONS=suppressions=${CMAKE_CURRENT_SOURCE_DIR}/lsan.supp:print_suppressions=0")
endif()

# Phase 7b: compute backend and the D2Q9 lattice Boltzmann solver.
# orrery_test_aero_lbm links only orrery_aero (graphics-free): Null backend,
# lattice, unit conversion, solid masks and the analytic flows (Poiseuille,
# Couette, Taylor-Green, mass conservation) on the CPU solver.
# orrery_test_aero_lbm_gl runs the OpenGL compute backend: tests tagged [gl]
# need an OpenGL 4.6 context and skip without one (ORRERY_REQUIRE_GL=1 makes
# that a failure); the 2048x1024 throughput benchmark is hidden
# (orrery_test_aero_lbm_gl "[benchmark]").
add_executable(orrery_test_aero_lbm
    aero/lbm/lbm_test_support.h
    aero/lbm/test_compute_backend.cpp
    aero/lbm/test_d2q9.cpp
    aero/lbm/test_lattice_units.cpp
    aero/lbm/test_solid_mask.cpp
    aero/lbm/test_force_history.cpp
    aero/lbm/test_lbm2d_flows.cpp
    aero/lbm/test_lbm2d_gpu_driver.cpp
    aero/lbm/test_lbm_aero_solver.cpp)
target_link_libraries(orrery_test_aero_lbm PRIVATE orrery_aero Catch2::Catch2WithMain)
orrery_configure_target(orrery_test_aero_lbm)
catch_discover_tests(orrery_test_aero_lbm DISCOVERY_MODE PRE_TEST)

add_executable(orrery_test_aero_lbm_gl
    aero/lbm/lbm_test_support.h
    aero/lbm/lbm_gl_support.h
    aero/lbm/test_lbm_gpu.cpp)
target_link_libraries(orrery_test_aero_lbm_gl PRIVATE orrery_render Catch2::Catch2WithMain)
target_include_directories(orrery_test_aero_lbm_gl PRIVATE ${PROJECT_BINARY_DIR}/src/render/generated)
target_compile_definitions(orrery_test_aero_lbm_gl PRIVATE
    ORRERY_RENDER_LBM_SHADER_DIR="${PROJECT_SOURCE_DIR}/src/render/shaders/lbm")
orrery_configure_target(orrery_test_aero_lbm_gl)
set(lbm_gl_discover_props)
if(ORRERY_SANITIZE AND NOT MSVC)
    set(lbm_gl_discover_props PROPERTIES ENVIRONMENT
        "LSAN_OPTIONS=suppressions=${CMAKE_CURRENT_SOURCE_DIR}/lsan.supp:print_suppressions=0")
endif()
catch_discover_tests(orrery_test_aero_lbm_gl DISCOVERY_MODE PRE_TEST ${lbm_gl_discover_props})

# Phase 7b validation (tests/aero_validation/lbm2d): circular cylinder at
# Re 100, Strouhal number and mean drag against the literature, on the CPU
# solver and, when an OpenGL 4.6 context exists, on the GPU solver. The run
# needs an optimised build; unoptimised (Debug, sanitizer) builds skip it.
add_executable(orrery_test_lbm2d_validation
    aero/lbm/lbm_test_support.h
    aero/lbm/lbm_gl_support.h
    aero_validation/lbm2d/cylinder_case.h
    aero_validation/lbm2d/test_cylinder.cpp)
target_link_libraries(orrery_test_lbm2d_validation PRIVATE orrery_render Catch2::Catch2WithMain)
orrery_configure_target(orrery_test_lbm2d_validation)
catch_discover_tests(orrery_test_lbm2d_validation DISCOVERY_MODE PRE_TEST
    ${lbm_gl_discover_props})

# Phase 7c: the D3Q19 lattice Boltzmann solver. orrery_test_aero_lbm3d links
# only orrery_aero (graphics-free): lattice, masks, analytic flows, the GPU
# driver on the Null backend and the IAeroSolver adapter on the CPU.
# orrery_test_aero_lbm3d_gl runs the OpenGL kernels against the CPU solver
# ([gl], skipped without an OpenGL 4.6 context; the 128^3 throughput
# benchmark is hidden: orrery_test_aero_lbm3d_gl "[benchmark]").
add_executable(orrery_test_aero_lbm3d
    aero/lbm/lbm_test_support.h
    aero/lbm3d/lbm3d_test_support.h
    aero/lbm3d/test_d3q19.cpp
    aero/lbm3d/test_solid_mask3d.cpp
    aero/lbm3d/test_lbm3d_flows.cpp
    aero/lbm3d/test_lbm3d_gpu_driver.cpp
    aero/lbm3d/test_lbm_aero_solver3d.cpp)
target_link_libraries(orrery_test_aero_lbm3d PRIVATE orrery_aero Catch2::Catch2WithMain)
orrery_configure_target(orrery_test_aero_lbm3d)
catch_discover_tests(orrery_test_aero_lbm3d DISCOVERY_MODE PRE_TEST)

add_executable(orrery_test_aero_lbm3d_gl
    aero/lbm/lbm_test_support.h
    aero/lbm/lbm_gl_support.h
    aero/lbm3d/lbm3d_test_support.h
    aero/lbm3d/test_lbm3d_gpu.cpp)
target_link_libraries(orrery_test_aero_lbm3d_gl PRIVATE orrery_render Catch2::Catch2WithMain)
target_include_directories(orrery_test_aero_lbm3d_gl PRIVATE ${PROJECT_BINARY_DIR}/src/render/generated)
orrery_configure_target(orrery_test_aero_lbm3d_gl)
catch_discover_tests(orrery_test_aero_lbm3d_gl DISCOVERY_MODE PRE_TEST ${lbm_gl_discover_props})

# Phase 7c validation (tests/aero_validation/lbm3d): sphere drag against the
# Schiller-Naumann curve and a square plate normal to the flow, through the
# IAeroSolver adapter. The CPU case needs an optimised build (unoptimised
# Debug and sanitizer builds skip it); the GPU cases need an OpenGL 4.6
# context; the larger runs are hidden ("[.study]").
add_executable(orrery_test_lbm3d_validation
    aero/lbm/lbm_gl_support.h
    aero_validation/lbm3d/body_case.h
    aero_validation/lbm3d/test_plate.cpp
    aero_validation/lbm3d/test_sphere.cpp)
target_link_libraries(orrery_test_lbm3d_validation PRIVATE orrery_render Catch2::Catch2WithMain)
orrery_configure_target(orrery_test_lbm3d_validation)
catch_discover_tests(orrery_test_lbm3d_validation DISCOVERY_MODE PRE_TEST
    ${lbm_gl_discover_props})

# Phase 11: benchmarks (spec section 8) and the nightly CPU validation
# (tests/bench). Compiled with the tests, never run by ctest.
option(ORRERY_BUILD_BENCHMARKS "Build the benchmark and nightly executables (tests/bench)" ON)
if(ORRERY_BUILD_BENCHMARKS)
    add_subdirectory(bench)
endif()
