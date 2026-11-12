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
    geom/test_voxel.cpp)
target_compile_definitions(orrery_test_geom PRIVATE
    ORRERY_GEOM_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/geom/fixtures"
    ORRERY_GEOM_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/geom_output")

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
    app/test_triple_buffer.cpp
    app/test_command_queue.cpp
    app/test_simulation.cpp
    app/test_physics_thread.cpp
    app/test_space_experiment.cpp)
target_link_libraries(orrery_test_sim PRIVATE orrery_sim Catch2::Catch2WithMain)
target_compile_definitions(orrery_test_sim PRIVATE
    ORRERY_SCENARIO_DIR="${PROJECT_SOURCE_DIR}/assets/scenarios")
orrery_configure_target(orrery_test_sim)
catch_discover_tests(orrery_test_sim DISCOVERY_MODE PRE_TEST)

add_executable(orrery_test_app
    app/app_test_support.h
    app/test_cli.cpp
    app/test_headless.cpp
    app/test_app_bar.cpp
    app/test_frame_stats.cpp)
target_link_libraries(orrery_test_app PRIVATE orrery_app_core Catch2::Catch2WithMain)
target_compile_definitions(orrery_test_app PRIVATE
    ORRERY_SCENARIO_DIR="${PROJECT_SOURCE_DIR}/assets/scenarios"
    ORRERY_RENDER_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/app_output")
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
# Started by its bare name through PATH from another directory, the
# executable still finds the assets next to itself.
add_test(NAME app.bare_name_finds_its_assets
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:orrery_app>
        -DSCENARIO_DIR=${PROJECT_SOURCE_DIR}/assets/scenarios
        -DWORK=${CMAKE_CURRENT_BINARY_DIR}/app_output/bare_name
        -P ${CMAKE_CURRENT_SOURCE_DIR}/app/bare_name_launch.cmake)
set_tests_properties(app.bare_name_finds_its_assets PROPERTIES SKIP_RETURN_CODE 77)
if(ORRERY_SANITIZE AND NOT MSVC)
    # The executable calls glfwInit without a display; see lsan.supp.
    set_tests_properties(app.headless_png app.headless_black_hole_benchmark
        app.bare_name_finds_its_assets PROPERTIES ENVIRONMENT
        "LSAN_OPTIONS=suppressions=${CMAKE_CURRENT_SOURCE_DIR}/lsan.supp:print_suppressions=0")
endif()
