# Phase 6b: URL model loader. Offline and deterministic: the transport tests
# talk to an in-process mock server on 127.0.0.1 (TLS with a certificate
# generated at test time, passed to the loader as FetchOptions::
# ca_bundle_override). OpenSSL and miniz are linked by the tests only to run
# that server and to compress test archives. Test names start with "net.".
find_package(OpenSSL REQUIRED)
find_package(miniz CONFIG REQUIRED)

add_executable(orrery_test_geom_net
    net_test_support.h
    zip_builder.cpp
    fs_links.cpp
    mock_server.h
    mock_server.cpp
    test_ca_certificates.cpp
    test_sha256.cpp
    test_url_policy.cpp
    test_model_format.cpp
    test_zip_extract.cpp
    test_model_fetch.cpp)
target_link_libraries(orrery_test_geom_net PRIVATE
    orrery_geom_net Catch2::Catch2WithMain OpenSSL::SSL OpenSSL::Crypto miniz::miniz)
if(WIN32)
    target_link_libraries(orrery_test_geom_net PRIVATE ws2_32)
    # OpenSSL and Winsock include <windows.h>: keep its min/max macros out.
    target_compile_definitions(orrery_test_geom_net PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
endif()
orrery_configure_target(orrery_test_geom_net)
catch_discover_tests(orrery_test_geom_net DISCOVERY_MODE PRE_TEST TEST_PREFIX "net.")
