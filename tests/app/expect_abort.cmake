# Runs EXE with ARGS and checks that it ends abnormally (not exit code 0, 1
# or 2) after writing exactly one line matching EXPECT_STDERR, the report of
# the terminate handler (a second report would mean the handler re-entered).
# With LOG, the crash log is removed first and must afterwards hold exactly
# one entry matching EXPECT_LOG.
if(DEFINED LOG)
    file(REMOVE_RECURSE "${LOG_DIR}")
endif()
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
if(DEFINED LOG)
    if(NOT EXISTS "${LOG}")
        message(FATAL_ERROR "the crash log ${LOG} was not written")
    endif()
    file(READ "${LOG}" log_text)
    message(STATUS "crash log:\n${log_text}")
    if(NOT log_text MATCHES "${EXPECT_LOG}")
        message(FATAL_ERROR "the crash log does not match '${EXPECT_LOG}'")
    endif()
    string(REGEX MATCHALL "\n" lines "${log_text}")
    list(LENGTH lines line_count)
    if(NOT line_count EQUAL 1)
        message(FATAL_ERROR "expected one crash log entry, got ${line_count} lines")
    endif()
    if(NOT err MATCHES "crash report appended to")
        message(FATAL_ERROR "standard error does not name the crash log")
    endif()
endif()
