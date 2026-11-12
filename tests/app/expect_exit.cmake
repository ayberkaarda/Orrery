# Runs EXE with the ;-separated ARGS and checks its exit code and output.
#   EXPECT_CODE    required exit code
#   EXPECT_STDOUT  optional regular expression the standard output must match
#   EXPECT_STDERR  optional regular expression the standard error must match
#   SKIP_CODE      optional exit code reported as skipped (script exit 77)
#   OUT_DIR        optional directory created before the run
if(DEFINED OUT_DIR)
    file(MAKE_DIRECTORY "${OUT_DIR}")
endif()
execute_process(COMMAND "${EXE}" ${ARGS}
    RESULT_VARIABLE code
    OUTPUT_VARIABLE out
    ERROR_VARIABLE err)
message(STATUS "exit code ${code}\nstdout:\n${out}\nstderr:\n${err}")
if(DEFINED SKIP_CODE AND code STREQUAL "${SKIP_CODE}")
    message(STATUS "skipped: exit code ${SKIP_CODE}")
    cmake_language(EXIT 77)
endif()
if(NOT code STREQUAL "${EXPECT_CODE}")
    message(FATAL_ERROR "expected exit code ${EXPECT_CODE}, got ${code}")
endif()
if(DEFINED EXPECT_STDOUT AND NOT out MATCHES "${EXPECT_STDOUT}")
    message(FATAL_ERROR "standard output does not match '${EXPECT_STDOUT}'")
endif()
if(DEFINED EXPECT_STDERR AND NOT err MATCHES "${EXPECT_STDERR}")
    message(FATAL_ERROR "standard error does not match '${EXPECT_STDERR}'")
endif()
