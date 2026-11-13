# ctest packaging.refuses_fallback, registered in builds with
# ORRERY_SOURCE_ASSET_FALLBACK=ON: cpack must refuse to package such a tree
# (its executable holds the source tree path) before it produces anything.
#   cmake -DBUILD_DIR=<build tree> -DWORK=<scratch dir> -DCPACK=<cpack> -P cpack_refuses_fallback.cmake
cmake_minimum_required(VERSION 3.25)

file(REMOVE_RECURSE "${WORK}")
file(MAKE_DIRECTORY "${WORK}")
execute_process(COMMAND "${CPACK}" --config "${BUILD_DIR}/CPackConfig.cmake" -G ZIP -B "${WORK}"
    RESULT_VARIABLE code OUTPUT_VARIABLE out ERROR_VARIABLE err)
file(GLOB_RECURSE produced LIST_DIRECTORIES false "${WORK}/*.zip")
file(REMOVE_RECURSE "${WORK}")
if(code EQUAL 0 OR produced)
    message(FATAL_ERROR "cpack packaged a tree with ORRERY_SOURCE_ASSET_FALLBACK=ON "
        "(exit code ${code}, files: ${produced})\n${out}\n${err}")
endif()
if(NOT "${out}${err}" MATCHES "ORRERY_SOURCE_ASSET_FALLBACK")
    message(FATAL_ERROR "cpack failed (exit code ${code}) without naming the source asset "
        "fallback:\n${out}\n${err}")
endif()
message(STATUS "cpack refused the tree (exit code ${code}) and wrote no package")
