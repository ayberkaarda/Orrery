# ctest packaging.embedded_paths: the check of install.relocated for build
# machine paths in the executable (packaging/embedded_paths.cmake), run on
# fake binaries: a path the project's own code would embed is found, with
# either separator; OpenSSL's two install directories, and nothing else, are
# tolerated.
#   cmake -DWORK=<scratch directory> -P packaging/tests/embedded_paths_test.cmake
cmake_minimum_required(VERSION 3.25)

include("${CMAKE_CURRENT_LIST_DIR}/../embedded_paths.cmake")

set(source "/work/Orrery")
set(build "/work/Orrery/build/linux-package")
set(installed "${build}/vcpkg_installed")
set(vcpkg "/opt/vcpkg")
file(REMOVE_RECURSE "${WORK}")
file(MAKE_DIRECTORY "${WORK}")

# Non-printable bytes around each string, as between the literals of a
# binary's read-only data.
string(ASCII 1 2 127 3 separator)
set(case 0)
# expect(<description> <expected problems> <expected tolerated> <string>...)
# Writes a fake binary holding the strings and compares what the check finds.
function(expect description expected_problems expected_tolerated)
    math(EXPR case "${case} + 1")
    set(case "${case}" PARENT_SCOPE)
    set(binary "${WORK}/fake_${case}.bin")
    set(content "${separator}ELF${separator}")
    foreach(string IN LISTS ARGN)
        string(APPEND content "${string}${separator}")
    endforeach()
    file(WRITE "${binary}" "${content}")
    orrery_embedded_path_problems(problems tolerated "${binary}"
        FORBIDDEN "${source}" "${build}" "${vcpkg}" VCPKG_INSTALLED_DIR "${installed}")
    list(LENGTH problems problem_count)
    list(LENGTH tolerated tolerated_count)
    if(problem_count EQUAL expected_problems AND tolerated_count EQUAL expected_tolerated)
        message(STATUS "ok: ${description}")
    else()
        message(SEND_ERROR "${description}: expected ${expected_problems} problem(s) and "
            "${expected_tolerated} tolerated, got problems '${problems}', tolerated '${tolerated}'")
    endif()
endfunction()

expect("a clean binary" 0 0 "orrery_app: no OpenGL 4.6 core context" "share/orrery/assets"
    "/etc/ssl/certs/ca-certificates.crt" "orrery/src/app/main.cpp")
# The project's own code: __FILE__ of a source file, an assert message.
expect("a source file path" 1 0 "${source}/src/app/main.cpp")
expect("a source path inside a longer string" 1 0
    "assertion failed at ${source}/src/core/result.h:42")
expect("a Windows source path" 1 0 "C:\\work\\Orrery\\src\\app\\main.cpp")
expect("a generated file in the build tree" 1 0 "${build}/src/app/version.h")
# Dependencies: __FILE__ of a vcpkg build tree, a header in vcpkg_installed.
expect("a vcpkg build tree path" 1 0
    "${vcpkg}/buildtrees/curl/src/url-8_23_0-7ec8cad9a3.clean/lib/vtls/openssl.c")
expect("a header of an installed dependency" 1 0
    "${installed}/x64-linux/include/openssl/crypto.h")
# OpenSSL's compiled-in directories, as `openssl version -a` prints them and bare.
expect("OpenSSL's ENGINESDIR and MODULESDIR" 0 4
    "ENGINESDIR: \"${installed}/x64-linux/lib/engines-3\""
    "MODULESDIR: \"${installed}/x64-linux/lib/ossl-modules\""
    "${installed}/x64-linux/lib/engines-3" "${installed}/x64-linux/lib/ossl-modules")
# The tolerance is exact: other files or directories under the same prefix fail.
expect("anything else under OpenSSL's directories" 3 0
    "${installed}/x64-linux/lib/engines-3/capi.so" "${installed}/x64-linux/lib/libcrypto.a"
    "OPENSSLDIR: \"${installed}/x64-linux/etc/ssl\"")
# OpenSSL records its compiler flags: a prefix map there names the old prefix.
expect("compiler flags recorded by a library" 1 0
    "compiler: clang-17 -fPIC -ffile-prefix-map=${source}=orrery -O3")
expect("OpenSSL's directories outside vcpkg_installed" 2 0
    "ENGINESDIR: \"${source}/lib/engines-3\"" "${build}/lib/ossl-modules")
expect("a mixed binary" 2 2 "ENGINESDIR: \"${installed}/x64-linux/lib/engines-3\""
    "${installed}/x64-linux/lib/ossl-modules" "${source}/src/geom/net/https_download.cpp"
    "orrery_app" "${vcpkg}/buildtrees/glfw3/src/x11_init.c")
file(REMOVE_RECURSE "${WORK}")
