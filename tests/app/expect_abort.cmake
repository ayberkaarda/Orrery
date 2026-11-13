# Runs EXE with ARGS and checks that it ends abnormally (not exit code 0, 1
# or 2) after writing exactly one line matching EXPECT_STDERR, the report of
# the terminate handler (a second report would mean the handler re-entered).
execute_process(COMMAND "${EXE}" ${ARGS}
    RESULT_VARIABLE code
    OUTPUT_VARIABLE out
    ERROR_VARIABLE err)
message(STATUS "exit code ${code}\nstdout:\n${out}\nstderr:\n${err}")
if(code STREQUAL "0" OR code STREQUAL "1" OR code STREQUAL "2")
    message(FATAL_ERROR "expected an abnormal end, got exit code ${code}")
endif()
if(NOT err MATCHES "${EXPECT_STDERR}")
    message(FATAL_ERROR "standard error does not match '${EXPECT_STDERR}'")
endif()
string(REGEX MATCHALL "orrery_app: fatal:" reports "${err}")
list(LENGTH reports count)
if(NOT count EQUAL 1)
    message(FATAL_ERROR "expected one fatal report, got ${count}")
endif()
