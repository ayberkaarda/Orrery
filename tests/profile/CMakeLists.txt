# Phase 10a: profiling zones, the trace sink and --trace (docs/observability.md).
#
# orrery_test_profile links only orrery_core (and through it orrery_profile),
# so the final link line of the graphics-free profile library is checked by
# tools/check_deps.py. Its translation units turn the zone macros on
# (test_profile_on.cpp) and off (test_profile_off.cpp) for themselves, so the
# sink and the empty expansion are tested in every build. The determinism
# test compares the state hash of a Barnes-Hut run with and without a capture.
# nlohmann_json parses the written traces.
find_package(nlohmann_json CONFIG REQUIRED)

add_executable(orrery_test_profile
    test_profile_on.cpp
    test_profile_off.cpp
    test_trace_json.cpp
    test_profile_determinism.cpp)
target_link_libraries(orrery_test_profile PRIVATE orrery_core nlohmann_json::nlohmann_json
    Catch2::Catch2WithMain)
orrery_configure_target(orrery_test_profile)
catch_discover_tests(orrery_test_profile DISCOVERY_MODE PRE_TEST)

# orrery_app --trace end to end: every run writes a valid trace (exit code 3,
# no OpenGL context, is accepted since the physics runs first); with
# ORRERY_PROFILING the trace holds the zones of the run.
set(trace_dir ${CMAKE_CURRENT_BINARY_DIR}/trace_output)
set(trace_script ${CMAKE_CURRENT_SOURCE_DIR}/trace_cli.cmake)
add_test(NAME profile.trace_space
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:orrery_app>
        "-DARGS=--headless;--experiment;solar_system;--frame;30;--size;160x90;--out;${trace_dir}/space.png"
        -DTRACE=${trace_dir}/space.json -DZONES=physics_step -DGL_ZONES=frame
        -P ${trace_script})
add_test(NAME profile.trace_black_hole
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:orrery_app>
        "-DARGS=--headless;--experiment;black_hole;--size;160x90;--out;${trace_dir}/black_hole.png"
        -DTRACE=${trace_dir}/black_hole.json "-DGL_ZONES=frame;bh_raymarch"
        -P ${trace_script})
add_test(NAME profile.trace_aero_panel
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:orrery_app>
        "-DARGS=--headless;--experiment;aero_naca0012_panel;--size;160x90;--out;${trace_dir}/aero.png"
        -DTRACE=${trace_dir}/aero.json -DZONES=panel_solve
        -P ${trace_script})
add_test(NAME profile.trace_entry
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:orrery_app>
        "-DARGS=--headless;--experiment;entry_apollo_like;--size;480x270;--out;${trace_dir}/entry.png"
        -DTRACE=${trace_dir}/entry.json -DZONES=entry_step
        -P ${trace_script})

# Command-line errors of --trace (tests/app/expect_exit.cmake).
set(expect_exit ${PROJECT_SOURCE_DIR}/tests/app/expect_exit.cmake)
add_test(NAME profile.trace_requires_value
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:orrery_app> -DEXPECT_CODE=2
        "-DEXPECT_STDERR=option --trace requires a value" "-DARGS=--headless;--trace"
        -P ${expect_exit})
add_test(NAME profile.trace_given_twice
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:orrery_app> -DEXPECT_CODE=2
        "-DEXPECT_STDERR=option --trace given more than once"
        "-DARGS=--modules;--trace;a.json;--trace;b.json" -P ${expect_exit})
# A trace that cannot be written fails an otherwise successful run (exit 1;
# skipped without an OpenGL context, where the run itself ends with exit 3).
add_test(NAME profile.trace_unwritable
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:orrery_app> -DEXPECT_CODE=1
        "-DEXPECT_STDERR=cannot write the trace .*: cannot open the file for writing"
        "-DARGS=--headless;--experiment;solar_system;--size;160x90;--out;${trace_dir}/unwritable.png;--trace;${trace_dir}/no_such_dir/trace.json"
        -DSKIP_CODE=3 -DOUT_DIR=${trace_dir} -P ${expect_exit})
set_tests_properties(profile.trace_unwritable PROPERTIES SKIP_RETURN_CODE 77)

# The dependency checker's entries for orrery_profile (graphics-free, Tracy
# approved for it alone).
find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_test(NAME profile.check_deps
        COMMAND Python3::Interpreter ${CMAKE_CURRENT_SOURCE_DIR}/test_check_deps_profile.py)
endif()
if(ORRERY_SANITIZE AND NOT MSVC)
    # The executable calls glfwInit without a display; see tests/lsan.supp.
    set_tests_properties(profile.trace_space profile.trace_black_hole profile.trace_aero_panel
        profile.trace_entry profile.trace_unwritable PROPERTIES ENVIRONMENT
        "LSAN_OPTIONS=suppressions=${PROJECT_SOURCE_DIR}/tests/lsan.supp:print_suppressions=0")
endif()
