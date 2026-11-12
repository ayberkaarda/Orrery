# Launches a copy of orrery_app by its bare name through PATH, from another
# working directory, and requires it to find the assets next to the copy
# (an experiment that exists only there). Arguments:
#   EXE           the built executable
#   SCENARIO_DIR  source scenarios (one is copied next to the executable)
#   WORK          scratch directory (recreated)
# Exit code 3 of the application (no OpenGL context) is reported as skipped.
get_filename_component(exe_dir "${EXE}" DIRECTORY)
get_filename_component(exe_name "${EXE}" NAME)
get_filename_component(exe_stem "${EXE}" NAME_WE)
set(bin "${WORK}/bin")
set(run "${WORK}/elsewhere")
file(REMOVE_RECURSE "${WORK}")
file(MAKE_DIRECTORY "${bin}/assets/experiments" "${bin}/assets/scenarios" "${run}")
file(COPY "${EXE}" DESTINATION "${bin}")
# Runtime libraries next to the executable (dynamic vcpkg triplets).
file(GLOB libraries "${exe_dir}/*.dll" "${exe_dir}/*.so*" "${exe_dir}/*.dylib")
if(libraries)
    file(COPY ${libraries} DESTINATION "${bin}")
endif()
file(COPY "${SCENARIO_DIR}/sun_earth.json" DESTINATION "${bin}/assets/scenarios")
file(WRITE "${bin}/assets/experiments/only_beside_the_copy.json" [=[
{
  "version": 1,
  "domain": "space",
  "inputs": { "scenario": "../scenarios/sun_earth.json" },
  "solver": { "name": "leapfrog_kdk" }
}
]=])
if(WIN32)
    set(ENV{PATH} "${bin};$ENV{PATH}")
else()
    set(ENV{PATH} "${bin}:$ENV{PATH}")
endif()
execute_process(COMMAND "${exe_stem}" --headless --experiment only_beside_the_copy
        --size 64x64 --out "${run}/out.png"
    WORKING_DIRECTORY "${run}"
    RESULT_VARIABLE code
    OUTPUT_VARIABLE out
    ERROR_VARIABLE err)
message(STATUS "${exe_name} by bare name: exit code ${code}\nstdout:\n${out}\nstderr:\n${err}")
if(code STREQUAL "3")
    cmake_language(EXIT 77)
endif()
if(NOT code STREQUAL "0")
    message(FATAL_ERROR "expected exit code 0, got ${code}")
endif()
if(NOT EXISTS "${run}/out.png")
    message(FATAL_ERROR "no image written")
endif()
