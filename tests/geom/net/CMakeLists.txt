# Phase 6b: URL model loader. Offline and deterministic: the transport tests
# talk to an in-process mock server on 127.0.0.1 (TLS with a certificate
# generated at test time, passed to the loader as FetchOptions::
# ca_bundle_override). OpenSSL and miniz are linked by the tests only to run
# that server and to compress test archives. Test names start with "net.".
find_package(OpenSSL REQUIRED)
find_package(miniz CONFIG REQUIRED)

# The mock server and archive builder, shared with the application tests
# that download experiment models (tests/app). Not named orrery_*: it is test
# scaffolding, not a module of the dependency graph (tools/check_deps.py).
add_library(test_net_support STATIC
    net_test_support.h
    zip_builder.cpp
    fs_links.cpp
    mock_server.h
    mock_server.cpp)
target_include_directories(test_net_support PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_net_support PRIVATE OpenSSL::SSL OpenSSL::Crypto miniz::miniz)
if(WIN32)
    target_link_libraries(test_net_support PRIVATE ws2_32)
    # OpenSSL and Winsock include <windows.h>: keep its min/max macros out.
    target_compile_definitions(test_net_support PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()
orrery_configure_target(test_net_support)

add_executable(orrery_test_geom_net
    test_sha256.cpp
    test_url_policy.cpp
    test_model_format.cpp
    test_zip_extract.cpp
    test_ca_certificates.cpp
    test_model_fetch.cpp)
target_link_libraries(orrery_test_geom_net PRIVATE
    orrery_geom_net test_net_support Catch2::Catch2WithMain)
orrery_configure_target(orrery_test_geom_net)
catch_discover_tests(orrery_test_geom_net DISCOVERY_MODE PRE_TEST TEST_PREFIX "net.")
