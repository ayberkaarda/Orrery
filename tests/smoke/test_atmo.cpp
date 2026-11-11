#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <string_view>

#include "orrery/atmo/module.h"
#include "orrery/core/module.h"

TEST_CASE("atmo reports its name", "[atmo][smoke]") {
    CHECK(orrery::atmo::module_info().name == "atmo");
}

TEST_CASE("atmo declares exactly its direct dependencies", "[atmo][smoke]") {
    constexpr std::array<std::string_view, 1> expected{"core"};
    const orrery::core::ModuleInfo info = orrery::atmo::module_info();
    REQUIRE(info.dependencies.size() == expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        CHECK(info.dependencies[i] == expected[i]);
    }
}

TEST_CASE("atmo links against its dependencies", "[atmo][smoke]") {
    CHECK(orrery::core::module_info().name == "core");
}
