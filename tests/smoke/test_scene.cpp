#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <string_view>

#include "orrery/aero/module.h"
#include "orrery/atmo/module.h"
#include "orrery/core/module.h"
#include "orrery/geom/module.h"
#include "orrery/scene/module.h"

TEST_CASE("scene reports its name", "[scene][smoke]") {
    CHECK(orrery::scene::module_info().name == "scene");
}

TEST_CASE("scene declares exactly its direct dependencies", "[scene][smoke]") {
    constexpr std::array<std::string_view, 4> expected{"core", "geom", "aero", "atmo"};
    const orrery::core::ModuleInfo info = orrery::scene::module_info();
    REQUIRE(info.dependencies.size() == expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        CHECK(info.dependencies[i] == expected[i]);
    }
}

TEST_CASE("scene links against its dependencies", "[scene][smoke]") {
    CHECK(orrery::core::module_info().name == "core");
    CHECK(orrery::geom::module_info().name == "geom");
    CHECK(orrery::aero::module_info().name == "aero");
    CHECK(orrery::atmo::module_info().name == "atmo");
}
