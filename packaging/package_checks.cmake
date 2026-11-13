# The conditions under which a package may be built (docs/packaging.md):
#   - never from a tree with ORRERY_SOURCE_ASSET_FALLBACK=ON: its executable
#     holds the source tree path and looks there for assets.
# Used at configure time by cmake/Packaging.cmake, at package time by
# packaging/cpack_project_config.cmake (CPACK_PROJECT_CONFIG_FILE, read by
# cpack for every generator), and by packaging/tests/package_checks_test.cmake.

# orrery_package_problem(<out> <generator> <cmake version> <source fallback>):
# sets <out> to why the package must not be built, or to "" when it may.
function(orrery_package_problem out generator version fallback)
    set(problem "")
    if(fallback)
        set(problem "this build has ORRERY_SOURCE_ASSET_FALLBACK=ON, so its executable holds "
            "the source tree path. Packages are built only from a package preset "
            "(cmake --preset package or linux-package), which turns it off")
    endif()
    string(CONCAT problem ${problem})
    set(${out} "${problem}" PARENT_SCOPE)
endfunction()
