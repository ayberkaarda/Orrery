# The conditions under which a package may be built (docs/packaging.md):
#   - never from a tree with ORRERY_SOURCE_ASSET_FALLBACK=ON: its executable
#     holds the source tree path and looks there for assets;
#   - the MSI (WIX generator) only with CMake 4.3 or newer, the first whose
#     WIX generator supports CPACK_WIX_INSTALL_SCOPE perUser.
# Used at configure time by cmake/Packaging.cmake, at package time by
# packaging/cpack_project_config.cmake (CPACK_PROJECT_CONFIG_FILE, read by
# cpack for every generator), and by packaging/tests/package_checks_test.cmake.

set(ORRERY_WIX_MIN_CMAKE 4.3)

# orrery_package_problem(<out> <generator> <cmake version> <source fallback>):
# sets <out> to why the package must not be built, or to "" when it may.
function(orrery_package_problem out generator version fallback)
    set(problem "")
    if(fallback)
        set(problem "this build has ORRERY_SOURCE_ASSET_FALLBACK=ON, so its executable holds "
            "the source tree path. Packages are built only from a package preset "
            "(cmake --preset package or linux-package), which turns it off")
    elseif(generator STREQUAL "WIX" AND version VERSION_LESS ORRERY_WIX_MIN_CMAKE)
        set(problem "the MSI needs CMake ${ORRERY_WIX_MIN_CMAKE} or newer "
            "(CPACK_WIX_INSTALL_SCOPE perUser), this is CMake ${version}")
    endif()
    string(CONCAT problem ${problem})
    set(${out} "${problem}" PARENT_SCOPE)
endfunction()
