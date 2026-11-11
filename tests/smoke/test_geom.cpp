#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <string_view>

#include "orrery/core/module.h"
#include "orrery/geom/module.h"

TEST_CASE("geom reports its name", "[geom][smoke]") {
    CHECK(orrery::geom::module_info().name == "geom");
}

TEST_CASE("geom declares exactly its direct dependencies", "[geom][smoke]") {
    constexpr std::array<std::string_view, 1> expected{"core"};
    const orrery::core::ModuleInfo info = orrery::geom::module_info();
    REQUIRE(info.dependencies.size() == expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        CHECK(info.dependencies[i] == expected[i]);
    }
}

TEST_CASE("geom links against its dependencies", "[geom][smoke]") {
    CHECK(orrery::core::module_info().name == "core");
}
