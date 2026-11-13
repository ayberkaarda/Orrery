# CPACK_PROJECT_CONFIG_FILE (cmake/Packaging.cmake): cpack reads this file for
# every generator, with CPACK_GENERATOR set to that generator, before it
# installs or packs anything. It refuses the packages that must not be built
# (packaging/package_checks.cmake).
include("${CMAKE_CURRENT_LIST_DIR}/package_checks.cmake")

orrery_package_problem(orrery_problem "${CPACK_GENERATOR}" "${CMAKE_VERSION}"
    "${CPACK_ORRERY_SOURCE_ASSET_FALLBACK}")
if(orrery_problem)
    message(FATAL_ERROR "no ${CPACK_GENERATOR} package: ${orrery_problem}")
endif()
