#include <catch2/catch_test_macros.hpp>

#include <string>

#include "orrery/core/module.h"

TEST_CASE("core reports the configured project version", "[core][smoke]") {
    CHECK(orrery::core::version_string() == std::string(ORRERY_EXPECTED_VERSION));

    const orrery::core::Version v = orrery::core::version();
    CHECK(v.major >= 0);
    CHECK(v.minor >= 0);
    CHECK(v.patch >= 0);
}

TEST_CASE("core is a leaf module", "[core][smoke]") {
    const orrery::core::ModuleInfo info = orrery::core::module_info();
    CHECK(info.name == "core");
    CHECK(info.dependencies.empty());
}
