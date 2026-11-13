# ctest packaging.checks: the conditions under which a package may be built
# (packaging/package_checks.cmake), for the generators and CMake versions
# that cannot be exercised by the build that runs the test.
#   cmake -P packaging/tests/package_checks_test.cmake
cmake_minimum_required(VERSION 3.25)

include("${CMAKE_CURRENT_LIST_DIR}/../package_checks.cmake")

set(failures 0)
# expect(<generator> <cmake version> <source fallback> <expected: OK or a regex>)
function(expect generator version fallback expected)
    orrery_package_problem(problem "${generator}" "${version}" "${fallback}")
    if(expected STREQUAL "OK")
        set(passed FALSE)
        if(problem STREQUAL "")
            set(passed TRUE)
        endif()
    else()
        set(passed FALSE)
        if(problem MATCHES "${expected}")
            set(passed TRUE)
        endif()
    endif()
    if(passed)
        message(STATUS "ok: ${generator}, CMake ${version}, fallback ${fallback}")
    else()
        message(SEND_ERROR "${generator}, CMake ${version}, fallback ${fallback}: expected "
            "'${expected}', got '${problem}'")
    endif()
endfunction()

# A tree with the source asset fallback compiled in is never packaged.
expect(ZIP 4.4.4 ON "ORRERY_SOURCE_ASSET_FALLBACK")
expect(WIX 4.4.4 ON "ORRERY_SOURCE_ASSET_FALLBACK")
expect(TGZ 4.1.0 ON "ORRERY_SOURCE_ASSET_FALLBACK")
expect(ZIP 4.4.4 1 "ORRERY_SOURCE_ASSET_FALLBACK")
# A package preset turns it off.
expect(WIX 4.4.4 OFF OK)
expect(ZIP 4.1.0 OFF OK)
expect(TGZ 4.1.0 OFF OK)
