# Checks the third-party notices of an installed tree against what it
# bundles (packaging/third_party/notices.cmake):
#   - Windows: every DLL in bin/ is either the MSVC runtime or maps to a vcpkg
#     port, and that port's copyright file is installed;
#   - every port with an extra notice whose copyright file is installed (that
#     is, every port the build uses) also has the extra notice, with the
#     pinned SHA-256.
# Included by packaging/install_smoke.cmake (install.relocated), or run on
# its own:
#   cmake -DPREFIX=<installed tree> -P packaging/check_notices.cmake
cmake_minimum_required(VERSION 3.25)

include("${CMAKE_CURRENT_LIST_DIR}/third_party/notices.cmake")

# orrery_notice_problems(<out> <prefix>): sets <out> to the list of problems.
function(orrery_notice_problems out prefix)
    set(problems "")
    set(notices "${prefix}/share/doc/orrery/third_party")
    file(GLOB dlls LIST_DIRECTORIES false RELATIVE "${prefix}/bin" "${prefix}/bin/*.dll")
    foreach(dll IN LISTS dlls)
        string(TOLOWER "${dll}" name)
        if(name MATCHES "${ORRERY_SYSTEM_RUNTIME_DLLS}")
            continue()
        endif()
        set(port "")
        foreach(entry IN LISTS ORRERY_BUNDLED_DLLS)
            string(REPLACE "|" ";" fields "${entry}")
            list(GET fields 0 pattern)
            if(name MATCHES "${pattern}")
                list(GET fields 1 port)
                break()
            endif()
        endforeach()
        if(port STREQUAL "")
            list(APPEND problems "bin/${dll} is not in ORRERY_BUNDLED_DLLS of packaging/third_party/notices.cmake: find its licence and the notices it requires")
        elseif(NOT EXISTS "${notices}/${port}/copyright")
            list(APPEND problems "bin/${dll}: the notice of port ${port} is not installed")
        endif()
    endforeach()
    foreach(entry IN LISTS ORRERY_EXTRA_NOTICES)
        string(REPLACE "|" ";" fields "${entry}")
        list(GET fields 0 port)
        list(GET fields 1 file)
        list(GET fields 3 sha256)
        if(NOT EXISTS "${notices}/${port}/copyright")
            continue()
        endif()
        set(notice "${notices}/${port}/${file}")
        if(NOT EXISTS "${notice}")
            list(APPEND problems "${port} is bundled but its notice ${file} is not installed")
            continue()
        endif()
        file(SHA256 "${notice}" actual)
        if(NOT actual STREQUAL sha256)
            list(APPEND problems "${port}/${file}: SHA-256 ${actual}, expected ${sha256}")
        endif()
    endforeach()
    set(${out} "${problems}" PARENT_SCOPE)
endfunction()

if(CMAKE_SCRIPT_MODE_FILE STREQUAL CMAKE_CURRENT_LIST_FILE)
    if(NOT DEFINED PREFIX)
        message(FATAL_ERROR "usage: cmake -DPREFIX=<installed tree> -P ${CMAKE_CURRENT_LIST_FILE}")
    endif()
    orrery_notice_problems(problems "${PREFIX}")
    if(problems)
        string(REPLACE ";" "\n  " problems "${problems}")
        message(FATAL_ERROR "third-party notices:\n  ${problems}")
    endif()
    message(STATUS "third-party notices: complete")
endif()
