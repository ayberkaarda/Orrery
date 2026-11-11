find_package(Catch2 3 CONFIG REQUIRED)
include(Catch)

# One test executable per library so each links only what it exercises.
function(orrery_add_module_test module)
    set(target orrery_test_${module})
    add_executable(${target} smoke/test_${module}.cpp)
    target_link_libraries(${target} PRIVATE orrery_${module} Catch2::Catch2WithMain)
    target_compile_definitions(${target} PRIVATE
        ORRERY_EXPECTED_VERSION="${PROJECT_VERSION}")
    orrery_configure_target(${target})
    catch_discover_tests(${target} DISCOVERY_MODE PRE_TEST)
endfunction()

foreach(module IN ITEMS geom atmo aero render)
    orrery_add_module_test(${module})
endforeach()

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
else()
    message(WARNING "Python 3 not found: check_deps.graphics_boundary test not registered")
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

# The application has no library surface; smoke-test the executable itself.
add_test(NAME app.version COMMAND orrery_app --version)
set_tests_properties(app.version PROPERTIES
    PASS_REGULAR_EXPRESSION "Orrery ${PROJECT_VERSION}"
    FAIL_REGULAR_EXPRESSION "modules:")

add_test(NAME app.module_graph COMMAND orrery_app)
set_tests_properties(app.module_graph PROPERTIES
    PASS_REGULAR_EXPRESSION "render -> core geom aero")
