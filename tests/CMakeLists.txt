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
