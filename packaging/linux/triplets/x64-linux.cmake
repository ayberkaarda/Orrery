# Overlay of vcpkg's x64-linux triplet, used only by the linux-package preset
# (VCPKG_OVERLAY_TRIPLETS in CMakePresets.json). Same settings as the builtin
# triplet, plus prefix maps: the libraries are linked statically into the
# shipped executable, and without the maps a __FILE__ expanded in their code
# (curl's lib/vtls/openssl.c through OpenSSL's OPENSSL_free, for example)
# would carry the absolute path of vcpkg's build tree into it.
# install.relocated checks the result (docs/packaging.md, "Paths in the
# executable").
set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE static)
set(VCPKG_CMAKE_SYSTEM_NAME Linux)

# The project root (this file is packaging/linux/triplets/x64-linux.cmake): in
# CI it holds the vcpkg checkout and the manifest's vcpkg_installed/.
get_filename_component(z_orrery_project_root "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
set(z_orrery_prefix_maps "")
# vcpkg defines these when it builds a port; other evaluations of a triplet
# (ABI hashing, compiler detection) see an empty value and add no map. A path
# with a space is skipped: VCPKG_C_FLAGS is split on spaces.
foreach(z_orrery_dir_and_name IN ITEMS
        "${z_orrery_project_root}|orrery"
        "${VCPKG_ROOT_DIR}|vcpkg"
        "${CURRENT_BUILDTREES_DIR}|vcpkg-buildtree"
        "${CURRENT_INSTALLED_DIR}|vcpkg-installed")
    string(REPLACE "|" ";" z_orrery_pair "${z_orrery_dir_and_name}")
    list(GET z_orrery_pair 0 z_orrery_dir)
    list(GET z_orrery_pair 1 z_orrery_name)
    if(IS_ABSOLUTE "${z_orrery_dir}" AND NOT z_orrery_dir MATCHES " ")
        string(APPEND z_orrery_prefix_maps " -ffile-prefix-map=${z_orrery_dir}=${z_orrery_name}")
    endif()
endforeach()
# Not for OpenSSL: it records its compiler flags in the library (the
# "compiler: ..." string of OpenSSL_version(OPENSSL_CFLAGS)), so the maps
# would carry the very paths they remove; its build passes relative source
# paths to the compiler, so its __FILE__ strings need no map.
if(NOT PORT STREQUAL "openssl")
    set(VCPKG_C_FLAGS "${z_orrery_prefix_maps}")
    set(VCPKG_CXX_FLAGS "${z_orrery_prefix_maps}")
endif()
