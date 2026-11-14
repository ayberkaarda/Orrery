# Phase 6b: secure URL model loader (ADR-010, docs/url-import-security.md).
# A separate library so that only code which downloads models links libcurl
# and miniz; it depends on orrery_core alone and never parses meshes.
find_package(CURL CONFIG REQUIRED)
find_package(miniz CONFIG REQUIRED)

add_library(orrery_geom_net STATIC
    fetch_error.cpp
    ca_certificates.cpp
    sha256.cpp
    url_policy.cpp
    model_format.cpp
    exclusive_file.cpp
    exclusive_file.h
    zip_extract.cpp
    https_download.cpp
    https_download.h
    model_cache.cpp
    model_cache.h
    model_fetch.cpp
    include/orrery/geom/net/ca_certificates.h
    include/orrery/geom/net/fetch_error.h
    include/orrery/geom/net/sha256.h
    include/orrery/geom/net/url_policy.h
    include/orrery/geom/net/model_format.h
    include/orrery/geom/net/zip_extract.h
    include/orrery/geom/net/model_fetch.h)
add_library(orrery::geom_net ALIAS orrery_geom_net)

target_include_directories(orrery_geom_net PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(orrery_geom_net PUBLIC orrery_core)
target_link_libraries(orrery_geom_net PRIVATE CURL::libcurl miniz::miniz)
# curl.h includes <windows.h> on Windows: keep its min/max macros out.
target_compile_definitions(orrery_geom_net PRIVATE
    $<$<PLATFORM_ID:Windows>:NOMINMAX>
    $<$<PLATFORM_ID:Windows>:WIN32_LEAN_AND_MEAN>)

orrery_configure_target(orrery_geom_net)
