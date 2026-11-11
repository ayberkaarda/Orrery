#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <string_view>

#include "orrery/aero/module.h"
#include "orrery/core/module.h"
#include "orrery/geom/module.h"

TEST_CASE("aero reports its name", "[aero][smoke]") {
    CHECK(orrery::aero::module_info().name == "aero");
}

TEST_CASE("aero declares exactly its direct dependencies", "[aero][smoke]") {
    constexpr std::array<std::string_view, 2> expected{"core", "geom"};
    const orrery::core::ModuleInfo info = orrery::aero::module_info();
    REQUIRE(info.dependencies.size() == expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        CHECK(info.dependencies[i] == expected[i]);
    }
}

TEST_CASE("aero links against its dependencies", "[aero][smoke]") {
    CHECK(orrery::core::module_info().name == "core");
    CHECK(orrery::geom::module_info().name == "geom");
}
