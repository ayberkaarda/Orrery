# Build machine paths in a shipped binary (docs/packaging.md, "Paths in the
# executable"). A release executable must not name the source tree, the
# build tree or the vcpkg checkout it was built from: such a string is either
# a lookup that cannot work on a user's machine or a leak of the build
# machine's layout. Included by packaging/install_smoke.cmake
# (install.relocated) and by packaging/tests/embedded_paths_test.cmake.
cmake_minimum_required(VERSION 3.25)

# A regular expression matching <dir> literally, with either separator.
function(z_orrery_path_regex out dir)
    file(TO_CMAKE_PATH "${dir}" dir)
    string(REGEX REPLACE "/+$" "" dir "${dir}")
    string(REGEX REPLACE "([][+.*()^$?|\\\\])" "\\\\\\1" regex "${dir}")
    string(REPLACE "/" "[/\\]" regex "${regex}")
    set(${out} "${regex}" PARENT_SCOPE)
endfunction()

# orrery_embedded_path_problems(<problems> <tolerated> <binary>
#     FORBIDDEN <dir>... [VCPKG_INSTALLED_DIR <dir>])
# Sets <problems> to the printable strings of <binary> that contain one of
# the FORBIDDEN directories, written with / or \ separators, and <tolerated>
# to those of them that are tolerated. The only tolerated strings are
# OpenSSL's ENGINESDIR and MODULESDIR, which its Configure derives from the
# install prefix vcpkg gives it (VCPKG_INSTALLED_DIR/<triplet>): they are
# preprocessor definitions, not __FILE__, so no prefix map reaches them, and
# vcpkg's port offers no option to change them. The Linux build links
# OpenSSL statically without loadable engines or providers (no-module), so
# neither directory is ever read unless a caller asks OpenSSL for a dynamic
# engine or provider by name, which Orrery never does.
function(orrery_embedded_path_problems problems_out tolerated_out binary)
    cmake_parse_arguments(PARSE_ARGV 3 arg "" "VCPKG_INSTALLED_DIR" "FORBIDDEN")
    set(alternatives "")
    foreach(dir IN LISTS arg_FORBIDDEN)
        if(NOT dir STREQUAL "")
            z_orrery_path_regex(regex "${dir}")
            list(APPEND alternatives "${regex}")
        endif()
    endforeach()
    set(problems "")
    set(tolerated "")
    if(alternatives)
        list(JOIN alternatives "|" any_forbidden)
        file(STRINGS "${binary}" found REGEX "(${any_forbidden})")
        list(REMOVE_DUPLICATES found)
        set(openssl_dirs "")
        if(NOT arg_VCPKG_INSTALLED_DIR STREQUAL "")
            z_orrery_path_regex(installed "${arg_VCPKG_INSTALLED_DIR}")
            set(openssl_dirs
                "^((ENGINESDIR|MODULESDIR): \")?${installed}/[^/\"]+/lib/(engines-3|ossl-modules)\"?$")
        endif()
        foreach(string IN LISTS found)
            if(NOT openssl_dirs STREQUAL "" AND string MATCHES "${openssl_dirs}")
                list(APPEND tolerated "${string}")
            else()
                list(APPEND problems "${string}")
            endif()
        endforeach()
    endif()
    set(${problems_out} "${problems}" PARENT_SCOPE)
    set(${tolerated_out} "${tolerated}" PARENT_SCOPE)
endfunction()
