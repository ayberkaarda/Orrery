# install.relocated (cmake/Packaging.cmake): installs a build tree to a
# temporary prefix, moves the installed tree elsewhere and runs the moved
# application with a clean library search path. Checks:
#   - the layout: bin/orrery_app, the bundled assets, no debug information;
#   - the third-party notices match what the tree bundles
#     (packaging/check_notices.cmake);
#   - every relative link and image of the installed documentation
#     (share/doc/orrery: the READMEs and docs/) resolves inside that folder
#     (tools/check_docs.py --tree, run with PYTHON);
#   - the size of the installed tree (MAX_BYTES, spec section 8: binary and
#     assets under 80 MB);
#   - --version reports VERSION;
#   - an experiment that exists only in the moved tree is found by name from
#     another working directory and rendered headless (with its scenario,
#     resolved relative to the experiment), and so is a bundled one;
#   - with SOURCE_FALLBACK off, the executable holds no path of the source
#     tree, the build tree or the vcpkg checkout (packaging/embedded_paths.cmake;
#     the only tolerated strings are OpenSSL's two compiled-in directories).
# Without an OpenGL 4.6 context the application exits with 3 after it has
# found and loaded the experiment: that still proves the lookup and passes,
# unless ORRERY_REQUIRE_GL=1 makes it a failure.
#
# Arguments: BUILD_DIR, WORK, VERSION, SOURCE_DIR, SOURCE_FALLBACK, ASSET_DIR,
# MAX_BYTES, PYTHON, and (empty without vcpkg) VCPKG_ROOT, VCPKG_INSTALLED_DIR.
cmake_minimum_required(VERSION 3.25)

set(staged "${WORK}/staged")
set(prefix "${WORK}/moved/Orrery ${VERSION}")
set(run_dir "${WORK}/elsewhere")
file(REMOVE_RECURSE "${WORK}")
file(MAKE_DIRECTORY "${WORK}/moved" "${run_dir}")

execute_process(COMMAND "${CMAKE_COMMAND}" --install "${BUILD_DIR}" --prefix "${staged}" --strip
    RESULT_VARIABLE code OUTPUT_VARIABLE out ERROR_VARIABLE err)
if(NOT code EQUAL 0)
    message(FATAL_ERROR "cmake --install failed (${code}):\n${out}\n${err}")
endif()
# Moved after installation, to a path with a space: nothing may point back
# at the staging prefix or the build tree.
file(RENAME "${staged}" "${prefix}")

if(WIN32)
    set(exe "${prefix}/bin/orrery_app.exe")
else()
    set(exe "${prefix}/bin/orrery_app")
endif()
foreach(required IN ITEMS "${exe}" "${prefix}/${ASSET_DIR}/experiments/solar_system.json"
        "${prefix}/${ASSET_DIR}/scenarios/sun_earth.json" "${prefix}/share/doc/orrery/notice.txt"
        "${prefix}/share/doc/orrery/LICENSE"
        "${prefix}/share/doc/orrery/README.md")
    if(NOT EXISTS "${required}")
        message(FATAL_ERROR "missing from the installed tree: ${required}")
    endif()
endforeach()

include("${CMAKE_CURRENT_LIST_DIR}/check_notices.cmake")
orrery_notice_problems(notice_problems "${prefix}")
if(notice_problems)
    string(REPLACE ";" "\n  " notice_problems "${notice_problems}")
    message(FATAL_ERROR "third-party notices:\n  ${notice_problems}")
endif()
message(STATUS "third-party notices: complete")

execute_process(COMMAND "${PYTHON}" "${SOURCE_DIR}/tools/check_docs.py"
        --tree "${prefix}/share/doc/orrery"
    RESULT_VARIABLE code OUTPUT_VARIABLE out ERROR_VARIABLE err)
if(NOT code EQUAL 0)
    message(FATAL_ERROR "links of the installed documentation (${code}):\n${out}\n${err}")
endif()
message(STATUS "installed documentation: ${out}")

file(GLOB_RECURSE installed LIST_DIRECTORIES false "${prefix}/*")
set(total 0)
set(debug_files "")
foreach(file IN LISTS installed)
    file(SIZE "${file}" size)
    math(EXPR total "${total} + ${size}")
    if(file MATCHES "\\.(pdb|ilk|debug)$")
        list(APPEND debug_files "${file}")
    endif()
endforeach()
file(SIZE "${exe}" exe_size)
math(EXPR total_kib "${total} / 1024")
math(EXPR exe_kib "${exe_size} / 1024")
list(LENGTH installed file_count)
message(STATUS "installed tree: ${file_count} files, ${total_kib} KiB (executable ${exe_kib} KiB, "
    "limit ${MAX_BYTES} bytes)")
if(debug_files)
    message(FATAL_ERROR "debug information installed: ${debug_files}")
endif()
if(total GREATER MAX_BYTES)
    message(FATAL_ERROR "installed tree is ${total} bytes, over the limit of ${MAX_BYTES}")
endif()
if(NOT WIN32)
    execute_process(COMMAND file "${exe}" OUTPUT_VARIABLE kind ERROR_QUIET)
    message(STATUS "executable: ${kind}")
    if(kind MATCHES "not stripped")
        message(FATAL_ERROR "the installed executable is not stripped")
    endif()
endif()

# Only the system's own library directories: every DLL the application needs
# must come from the installed tree.
if(WIN32)
    set(ENV{PATH} "$ENV{SystemRoot}\\System32;$ENV{SystemRoot}")
else()
    unset(ENV{LD_LIBRARY_PATH})
endif()

execute_process(COMMAND "${exe}" --version
    WORKING_DIRECTORY "${run_dir}"
    RESULT_VARIABLE code OUTPUT_VARIABLE out ERROR_VARIABLE err)
if(NOT code EQUAL 0 OR NOT out MATCHES "Orrery ${VERSION}")
    message(FATAL_ERROR "--version: exit code ${code}\nstdout:\n${out}\nstderr:\n${err}")
endif()
message(STATUS "--version: ${out}")

set(gl_missing FALSE)
# Runs the moved application headless on experiment NAME and checks the image.
function(render_installed name)
    set(image "${run_dir}/${name}.png")
    string(TIMESTAMP start "%s%f")
    execute_process(COMMAND "${exe}" --headless --experiment "${name}" --size 160x90
            --out "${image}"
        WORKING_DIRECTORY "${run_dir}"
        RESULT_VARIABLE code OUTPUT_VARIABLE out ERROR_VARIABLE err)
    string(TIMESTAMP stop "%s%f")
    math(EXPR elapsed_ms "(${stop} - ${start}) / 1000")
    message(STATUS "${name}: exit code ${code} after ${elapsed_ms} ms\nstdout:\n${out}\nstderr:\n${err}")
    if(code EQUAL 3)
        set(gl_missing TRUE PARENT_SCOPE)
        return()
    endif()
    if(NOT code EQUAL 0)
        message(FATAL_ERROR "${name}: expected exit code 0, got ${code}")
    endif()
    if(NOT EXISTS "${image}")
        message(FATAL_ERROR "${name}: no image written")
    endif()
endfunction()

# An experiment that exists only in the moved tree.
file(WRITE "${prefix}/${ASSET_DIR}/experiments/only_in_the_installed_tree.json" [=[
{
  "version": 1,
  "domain": "space",
  "inputs": { "scenario": "../scenarios/sun_earth.json" },
  "solver": { "name": "leapfrog_kdk" }
}
]=])
render_installed(only_in_the_installed_tree)
render_installed(solar_system)

if(NOT SOURCE_FALLBACK)
    include("${CMAKE_CURRENT_LIST_DIR}/embedded_paths.cmake")
    orrery_embedded_path_problems(build_paths tolerated_paths "${exe}"
        FORBIDDEN "${SOURCE_DIR}" "${BUILD_DIR}" "${VCPKG_ROOT}"
        VCPKG_INSTALLED_DIR "${VCPKG_INSTALLED_DIR}")
    if(tolerated_paths)
        string(REPLACE ";" "\n  " tolerated_paths "${tolerated_paths}")
        message(STATUS "tolerated in the executable (OpenSSL's compiled-in engine and module "
            "directories, never read; docs/packaging.md):\n  ${tolerated_paths}")
    endif()
    if(build_paths)
        string(REPLACE ";" "\n  " build_paths "${build_paths}")
        message(FATAL_ERROR "the executable holds paths of the source tree, the build tree or "
            "vcpkg:\n  ${build_paths}")
    endif()
    message(STATUS "no source, build or vcpkg path in the executable")
else()
    message(STATUS "ORRERY_SOURCE_ASSET_FALLBACK is on: the executable may also look in the "
        "source tree; the experiment only in the installed tree proves the installed lookup")
endif()

if(gl_missing)
    if("$ENV{ORRERY_REQUIRE_GL}" STREQUAL "1")
        message(FATAL_ERROR "no OpenGL 4.6 context, and ORRERY_REQUIRE_GL=1")
    endif()
    message(STATUS "no OpenGL 4.6 context: the experiments were found and loaded from the "
        "installed tree (exit code 3), the images were not rendered")
endif()
file(REMOVE_RECURSE "${WORK}")
