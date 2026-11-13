cmake_minimum_required(VERSION 4.1)

# Profiling (docs/observability.md). Declared before project() because the
# vcpkg toolchain installs the manifest there: ORRERY_TRACY adds the manifest
# feature "profiling" (the Tracy client), so default builds install nothing more.
option(ORRERY_PROFILING "Compile the profiling zones in (off: the zone macros are empty)" OFF)
option(ORRERY_TRACY "Make every profiling zone a Tracy zone (requires ORRERY_PROFILING)" OFF)
if(ORRERY_TRACY AND NOT ORRERY_PROFILING)
    message(FATAL_ERROR "ORRERY_TRACY requires ORRERY_PROFILING=ON")
endif()
if(ORRERY_TRACY)
    list(APPEND VCPKG_MANIFEST_FEATURES "profiling")
endif()

project(Orrery
    VERSION 0.0.1
    DESCRIPTION "Physics laboratory: space dynamics, aerodynamics and atmospheric entry"
    LANGUAGES CXX)

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
    message(FATAL_ERROR "In-source builds are not allowed. Use a preset: cmake --preset release")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# C++20 modules are not used by this project; skip dependency scanning.
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ORRERY_BUILD_TESTS "Build the Catch2 test suite" ON)
option(ORRERY_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
option(ORRERY_SANITIZE "Enable AddressSanitizer (and UBSan on GCC/Clang)" OFF)
option(ORRERY_HEADLESS "Build for headless environments (no window system)" OFF)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(Warnings)
include(Sanitizers)
include(FpDeterminism)

# Applies the project-wide compile policy (warnings, sanitizers, deterministic
# floating-point semantics) to a first-party target.
function(orrery_configure_target target)
    orrery_target_warnings(${target})
    orrery_target_sanitizers(${target})
    orrery_target_fp_determinism(${target})
endfunction()

# Request the CMake File API code model so tools/check_deps.py can inspect the
# real target link graph of this build tree.
cmake_file_api(QUERY API_VERSION 1 CODEMODEL 2)

add_subdirectory(src)

if(ORRERY_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
