#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <string_view>

#include "orrery/aero/module.h"
#include "orrery/core/module.h"
#include "orrery/geom/module.h"
#include "orrery/render/module.h"

TEST_CASE("render reports its name", "[render][smoke]") {
    CHECK(orrery::render::module_info().name == "render");
}

TEST_CASE("render declares exactly its direct dependencies", "[render][smoke]") {
    constexpr std::array<std::string_view, 3> expected{"core", "geom", "aero"};
    const orrery::core::ModuleInfo info = orrery::render::module_info();
    REQUIRE(info.dependencies.size() == expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
        CHECK(info.dependencies[i] == expected[i]);
    }
}

TEST_CASE("render links against its dependencies", "[render][smoke]") {
    CHECK(orrery::core::module_info().name == "core");
    CHECK(orrery::geom::module_info().name == "geom");
    CHECK(orrery::aero::module_info().name == "aero");
}
