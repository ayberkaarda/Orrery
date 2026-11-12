# Phase 7a: aerodynamic validation set (docs/physics.md section 6). Each case
# cites its reference in the test source.
add_executable(orrery_test_aero_validation
    test_naca0012.cpp
    test_wing.cpp
    test_apollo.cpp)
target_link_libraries(orrery_test_aero_validation PRIVATE orrery_aero Catch2::Catch2WithMain)
orrery_configure_target(orrery_test_aero_validation)
catch_discover_tests(orrery_test_aero_validation DISCOVERY_MODE PRE_TEST)
