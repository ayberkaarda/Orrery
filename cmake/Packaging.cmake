# Phase 11: install layout, runtime dependencies, the relocated install test
# and the CPack packages (docs/packaging.md). Included last by the top-level
# CMakeLists.txt, once every target exists.
#
# Installed layout (relative to the prefix, the same on every platform):
#   bin/orrery_app[.exe]           the application (and its DLLs on Windows)
#   share/orrery/assets/           bundled experiments and scenarios
#   share/doc/orrery/              READMEs, docs/, notice, third_party/<library>/
#   share/applications/            desktop entry (Linux)
#   share/icons/hicolor/           application icon (Linux)
# The application finds share/orrery/assets from its own directory
# (app::asset_directories), so an installed tree works wherever it is moved.

set(ORRERY_PACKAGE_VENDOR "Orrery")
set(ORRERY_PACKAGE_DISPLAY_NAME "Orrery")
set(ORRERY_ICON_ICO "${PROJECT_SOURCE_DIR}/packaging/icon/orrery.ico")
set(ORRERY_INSTALL_ASSET_DIR "share/orrery/assets")
set(ORRERY_INSTALL_DOC_DIR "share/doc/orrery")

# The version has one source, project(VERSION) in the top-level
# CMakeLists.txt. The vcpkg manifest must carry the same version; the release
# workflow reads orrery_version.txt to check a tag against it.
file(READ "${PROJECT_SOURCE_DIR}/vcpkg.json" orrery_manifest)
string(JSON orrery_manifest_version ERROR_VARIABLE orrery_manifest_error
    GET "${orrery_manifest}" version-semver)
if(NOT orrery_manifest_version STREQUAL PROJECT_VERSION)
    message(FATAL_ERROR "vcpkg.json version-semver '${orrery_manifest_version}' differs from "
        "the project version ${PROJECT_VERSION}: change both together")
endif()
file(CONFIGURE OUTPUT "${PROJECT_BINARY_DIR}/orrery_version.txt" CONTENT "${PROJECT_VERSION}\n")

# Windows resources: the icon (as GLFW_ICON, which GLFW also uses for the
# window) and the version information shown in the file properties.
if(WIN32)
    enable_language(RC)
    configure_file("${PROJECT_SOURCE_DIR}/packaging/windows/orrery_app.rc.in"
        "${PROJECT_BINARY_DIR}/packaging/orrery_app.rc" @ONLY)
    target_sources(orrery_app PRIVATE "${PROJECT_BINARY_DIR}/packaging/orrery_app.rc")
endif()

# The application and, on Windows, the DLLs it loads. vcpkg's Windows
# triplet links dynamically, so the DLLs it built are resolved from the
# executable and installed beside it; system DLLs are left out, and the MSVC
# runtime is installed app-local by InstallRequiredSystemLibraries. The Linux
# triplet links the vcpkg libraries statically: the executable needs only
# system libraries (libc, libstdc++, libGL, X11), which the AppImage bundles
# where appropriate (packaging/linux/build-appimage.sh).
if(POLICY CMP0207)
    # Match the exclusion patterns against normalised (forward-slash) paths.
    cmake_policy(SET CMP0207 NEW)
endif()
if(WIN32)
    install(TARGETS orrery_app RUNTIME_DEPENDENCY_SET orrery_runtime RUNTIME DESTINATION bin)
    set(orrery_vcpkg_bin "")
    if(DEFINED VCPKG_INSTALLED_DIR AND DEFINED VCPKG_TARGET_TRIPLET)
        set(orrery_vcpkg_bin "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/bin")
    endif()
    install(RUNTIME_DEPENDENCY_SET orrery_runtime
        PRE_EXCLUDE_REGEXES "^api-ms-" "^ext-ms-"
        POST_EXCLUDE_REGEXES "^[A-Za-z]:[/\\\\][Ww][Ii][Nn][Dd][Oo][Ww][Ss][/\\\\]"
        DIRECTORIES ${orrery_vcpkg_bin}
        DESTINATION bin)
    set(CMAKE_INSTALL_SYSTEM_RUNTIME_DESTINATION bin)
    set(CMAKE_INSTALL_UCRT_LIBRARIES OFF)
    include(InstallRequiredSystemLibraries)
else()
    install(TARGETS orrery_app RUNTIME DESTINATION bin)
endif()

install(DIRECTORY "${PROJECT_SOURCE_DIR}/assets/" DESTINATION "${ORRERY_INSTALL_ASSET_DIR}")
# The READMEs keep their relative links (docs/..., README.tr.md, the
# screenshots), so the documentation they link to is installed beside them;
# install.relocated checks every link of the installed tree.
install(FILES "${PROJECT_SOURCE_DIR}/README.md" "${PROJECT_SOURCE_DIR}/README.tr.md"
    "${PROJECT_SOURCE_DIR}/packaging/notice.txt"
    DESTINATION "${ORRERY_INSTALL_DOC_DIR}")
install(DIRECTORY "${PROJECT_SOURCE_DIR}/docs/" DESTINATION "${ORRERY_INSTALL_DOC_DIR}/docs")

# Licence notices of the vcpkg libraries (vcpkg installs one copyright file
# per port). Build helpers and the test framework are not shipped.
if(DEFINED VCPKG_INSTALLED_DIR AND DEFINED VCPKG_TARGET_TRIPLET)
    file(GLOB orrery_copyrights LIST_DIRECTORIES false
        "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/share/*/copyright")
    foreach(copyright IN LISTS orrery_copyrights)
        get_filename_component(port_dir "${copyright}" DIRECTORY)
        get_filename_component(port "${port_dir}" NAME)
        if(NOT port MATCHES "^(vcpkg-.*|catch2)$")
            install(FILES "${copyright}" DESTINATION "${ORRERY_INSTALL_DOC_DIR}/third_party/${port}")
        endif()
    endforeach()
    # Notices some libraries require beyond their copyright file
    # (packaging/third_party/notices.cmake), checked against the port version
    # they were taken from and their pinned SHA-256.
    include("${PROJECT_SOURCE_DIR}/packaging/third_party/notices.cmake")
    foreach(entry IN LISTS ORRERY_EXTRA_NOTICES)
        string(REPLACE "|" ";" fields "${entry}")
        list(GET fields 0 port)
        list(GET fields 1 notice_file)
        list(GET fields 2 notice_version)
        list(GET fields 3 notice_sha256)
        set(port_share "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/share/${port}")
        if(NOT EXISTS "${port_share}/copyright")
            continue()
        endif()
        file(READ "${port_share}/vcpkg.spdx.json" port_spdx)
        string(JSON port_version ERROR_VARIABLE port_version_error
            GET "${port_spdx}" packages 0 versionInfo)
        if(NOT port_version STREQUAL notice_version)
            message(FATAL_ERROR "vcpkg port ${port} is version '${port_version}', but "
                "packaging/third_party/${port}/${notice_file} is the notice of ${notice_version}: "
                "take the notice of the new version from upstream and update notices.cmake")
        endif()
        set(notice "${PROJECT_SOURCE_DIR}/packaging/third_party/${port}/${notice_file}")
        file(SHA256 "${notice}" actual_sha256)
        if(NOT actual_sha256 STREQUAL notice_sha256)
            message(FATAL_ERROR "${notice}: SHA-256 ${actual_sha256}, pinned ${notice_sha256}")
        endif()
        install(FILES "${notice}" DESTINATION "${ORRERY_INSTALL_DOC_DIR}/third_party/${port}")
    endforeach()
endif()

if(UNIX AND NOT APPLE)
    install(FILES "${PROJECT_SOURCE_DIR}/packaging/linux/orrery.desktop"
        DESTINATION share/applications)
    install(FILES "${PROJECT_SOURCE_DIR}/packaging/icon/orrery.png"
        DESTINATION share/icons/hicolor/256x256/apps)
    install(FILES "${PROJECT_SOURCE_DIR}/packaging/icon/orrery.svg"
        DESTINATION share/icons/hicolor/scalable/apps)
endif()

# install.relocated: installs this build tree to a temporary prefix, moves the
# tree, and runs the moved application headless (packaging/install_smoke.cmake).
# Off by default; the package presets turn it on.
option(ORRERY_INSTALL_TEST "Register the relocated install test (install.relocated)" OFF)
if(ORRERY_INSTALL_TEST)
    # Runs tools/check_docs.py on the installed documentation.
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    # The vcpkg checkout (from its toolchain file) and the manifest's installed
    # tree: the executable must not name the first, and may name the second
    # only in OpenSSL's compiled-in directories (packaging/embedded_paths.cmake).
    set(orrery_vcpkg_root "")
    if(CMAKE_TOOLCHAIN_FILE MATCHES "/scripts/buildsystems/vcpkg\\.cmake$")
        cmake_path(GET CMAKE_TOOLCHAIN_FILE PARENT_PATH orrery_vcpkg_root)
        cmake_path(GET orrery_vcpkg_root PARENT_PATH orrery_vcpkg_root)
        cmake_path(GET orrery_vcpkg_root PARENT_PATH orrery_vcpkg_root)
    endif()
    set(orrery_vcpkg_installed "")
    if(DEFINED VCPKG_INSTALLED_DIR)
        set(orrery_vcpkg_installed "${VCPKG_INSTALLED_DIR}")
    endif()
    enable_testing()
    add_test(NAME install.relocated
        COMMAND ${CMAKE_COMMAND}
            -DBUILD_DIR=${PROJECT_BINARY_DIR}
            -DWORK=${PROJECT_BINARY_DIR}/install_test
            -DVERSION=${PROJECT_VERSION}
            -DSOURCE_DIR=${PROJECT_SOURCE_DIR}
            -DSOURCE_FALLBACK=${ORRERY_SOURCE_ASSET_FALLBACK}
            -DASSET_DIR=${ORRERY_INSTALL_ASSET_DIR}
            -DMAX_BYTES=80000000
            -DPYTHON=${Python3_EXECUTABLE}
            -DVCPKG_ROOT=${orrery_vcpkg_root}
            -DVCPKG_INSTALLED_DIR=${orrery_vcpkg_installed}
            -P ${PROJECT_SOURCE_DIR}/packaging/install_smoke.cmake)
    set_tests_properties(install.relocated PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 900)
endif()

# Packages: an MSI installer (WiX .NET tool) and a portable zip on Windows, a
# tarball on Linux; the AppImage is assembled from the installed tree by
# packaging/linux/build-appimage.sh. Debug information is never installed
# (Release builds, stripped on Linux).
set(CPACK_PACKAGE_NAME "Orrery")
set(CPACK_PACKAGE_VENDOR "${ORRERY_PACKAGE_VENDOR}")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PROJECT_DESCRIPTION}")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "Orrery")
set(CPACK_PACKAGE_EXECUTABLES "orrery_app" "Orrery")
set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/packaging/notice.txt")
set(CPACK_STRIP_FILES TRUE)
set(CPACK_PACKAGE_CHECKSUM SHA256)
set(CPACK_VERBATIM_VARIABLES TRUE)
if(WIN32)
    set(CPACK_SYSTEM_NAME "windows-x64")
    set(CPACK_GENERATOR "WIX;ZIP")
    # WiX .NET tool (v4 schema; the pinned tool version is in docs/packaging.md
    # and .github/workflows/release.yml). Per-user scope: no elevation, the
    # application goes to %LOCALAPPDATA%\Programs\Orrery (apart from the
    # application's own data under %APPDATA%\orrery and %LOCALAPPDATA%\orrery).
    set(CPACK_WIX_VERSION 4)
    set(CPACK_PACKAGE_INSTALL_DIRECTORY "Programs/Orrery")
    set(CPACK_WIX_PATCH_FILE "${PROJECT_SOURCE_DIR}/packaging/windows/wix_patch.xml")
    set(CPACK_WIX_INSTALL_SCOPE perUser)
    # Constant across versions so a newer installer replaces an older one.
    set(CPACK_WIX_UPGRADE_GUID "E71DEC8C-E50A-424D-A282-4DD7644544D0")
    set(CPACK_WIX_PRODUCT_ICON "${ORRERY_ICON_ICO}")
    set(CPACK_WIX_PROGRAM_MENU_FOLDER ".")
    set(CPACK_WIX_PROPERTY_ARPCOMMENTS "${PROJECT_DESCRIPTION}")
else()
    set(CPACK_SYSTEM_NAME "linux-x86_64")
    set(CPACK_GENERATOR "TGZ")
endif()
set(CPACK_PACKAGE_FILE_NAME "orrery-${PROJECT_VERSION}-${CPACK_SYSTEM_NAME}")
# cpack refuses a tree built with the source asset fallback (its executable
# holds the source tree path) and an MSI with a CMake older than 4.3
# (packaging/package_checks.cmake, checked per generator by
# packaging/cpack_project_config.cmake).
include("${PROJECT_SOURCE_DIR}/packaging/package_checks.cmake")
set(CPACK_ORRERY_SOURCE_ASSET_FALLBACK "${ORRERY_SOURCE_ASSET_FALLBACK}")
set(CPACK_PROJECT_CONFIG_FILE "${PROJECT_SOURCE_DIR}/packaging/cpack_project_config.cmake")
if(WIN32 AND NOT ORRERY_SOURCE_ASSET_FALLBACK)
    # A package build: its MSI needs CMake 4.3, so stop now rather than at cpack.
    orrery_package_problem(orrery_problem WIX "${CMAKE_VERSION}" OFF)
    if(orrery_problem)
        message(FATAL_ERROR "Package build: ${orrery_problem}. Install CMake "
            "${ORRERY_WIX_MIN_CMAKE} or newer (docs/packaging.md); the project itself "
            "builds with CMake 4.1 in the other presets.")
    endif()
endif()
include(CPack)

# Tests of the packaging conditions (packaging/tests/).
if(ORRERY_BUILD_TESTS)
    add_test(NAME packaging.checks
        COMMAND ${CMAKE_COMMAND} -P ${PROJECT_SOURCE_DIR}/packaging/tests/package_checks_test.cmake)
    add_test(NAME packaging.embedded_paths
        COMMAND ${CMAKE_COMMAND} -DWORK=${PROJECT_BINARY_DIR}/embedded_paths_test
            -P ${PROJECT_SOURCE_DIR}/packaging/tests/embedded_paths_test.cmake)
    if(ORRERY_SOURCE_ASSET_FALLBACK)
        add_test(NAME packaging.refuses_fallback
            COMMAND ${CMAKE_COMMAND}
                -DBUILD_DIR=${PROJECT_BINARY_DIR}
                -DWORK=${PROJECT_BINARY_DIR}/cpack_refusal_test
                -DCPACK=${CMAKE_CPACK_COMMAND}
                -P ${PROJECT_SOURCE_DIR}/packaging/tests/cpack_refuses_fallback.cmake)
    endif()
endif()
