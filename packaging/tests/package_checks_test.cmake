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
# The MSI (CPACK_WIX_INSTALL_SCOPE perUser) needs CMake 4.3.
expect(WIX 4.1.0 OFF "CMake 4\\.3")
expect(WIX 4.2.3 OFF "CMake 4\\.3")
expect(WIX 4.3.0 OFF OK)
expect(WIX 4.4.4 OFF OK)
# The zip and the tarball build with the project's minimum, CMake 4.1.
expect(ZIP 4.1.0 OFF OK)
expect(TGZ 4.1.0 OFF OK)

# The package notice (packaging/notice.txt, also the installer's licence
# page) reproduces the project licence verbatim.
file(READ "${CMAKE_CURRENT_LIST_DIR}/../../LICENSE" licence_text)
file(READ "${CMAKE_CURRENT_LIST_DIR}/../notice.txt" notice_text)
string(REPLACE "\r" "" licence_text "${licence_text}")
string(REPLACE "\r" "" notice_text "${notice_text}")
string(FIND "${notice_text}" "${licence_text}" licence_at)
if(licence_at EQUAL -1)
    message(SEND_ERROR "packaging/notice.txt does not contain the text of LICENSE")
else()
    message(STATUS "ok: packaging/notice.txt contains LICENSE")
endif()
