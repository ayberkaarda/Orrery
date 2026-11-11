#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <string>
#include <utility>
#include <variant>

#include "orrery/scene/experiment.h"
#include "orrery/scene/scenario.h"
#include "scene_test_support.h"

namespace scene = orrery::scene;
using scene::SceneErrorCode;
using scene::test::fixture_path;

namespace {

std::string experiment_with_domain(const std::string& domain) {
    return R"({"version": 1, "domain": ")" + domain +
           R"(", "inputs": {"geometry": "naca0012"}, "conditions": {"mach": 0.2, "aoa_deg": 4},
              "solver": {"name": "panel", "version": "1", "settings": {"panels": 200}},
              "result": {"cl": 0.4382910571234567, "cd": 0.0081}})";
}

} // namespace

TEST_CASE("experiment domains space, aero and entry are accepted", "[scene][experiment]") {
    const auto [label, domain] =
        GENERATE(std::pair{"space", scene::Domain::space}, std::pair{"aero", scene::Domain::aero},
                 std::pair{"entry", scene::Domain::entry});
    const auto parsed = scene::parse_experiment(experiment_with_domain(label));
    REQUIRE(parsed.has_value());
    const scene::Experiment& e = parsed.value();
    CHECK(e.domain == domain);
    CHECK(std::get<std::string>(e.inputs.at("geometry")) == "naca0012");
    CHECK(std::get<double>(e.conditions.at("mach")) == 0.2);
    CHECK(std::get<double>(e.conditions.at("aoa_deg")) == 4.0);
    CHECK(e.solver.name == "panel");
    CHECK(e.solver.version == "1");
    CHECK(std::get<double>(e.solver.settings.at("panels")) == 200.0);
    CHECK(std::get<double>(e.result.at("cl")) == 0.4382910571234567);
}

TEST_CASE("any other experiment domain is rejected", "[scene][experiment]") {
    const std::string label = GENERATE("fluid", "thermal", "Space", "", "aero ");
    CAPTURE(label);
    const auto parsed = scene::parse_experiment(experiment_with_domain(label));
    REQUIRE_FALSE(parsed.has_value());
    CHECK(parsed.error().code == SceneErrorCode::invalid_value);
    CHECK(parsed.error().path == "domain");
    CHECK(parsed.error().message.find("allowed: space, aero, entry") != std::string::npos);
}

TEST_CASE("experiment fixture with an unknown domain is rejected", "[scene][experiment]") {
    const auto text = scene::read_text_file(fixture_path("experiment_unknown_domain.json"));
    REQUIRE(text.has_value());
    const auto parsed = scene::parse_experiment(text.value());
    REQUIRE_FALSE(parsed.has_value());
    CHECK(parsed.error().path == "domain");
    CHECK(parsed.error().message.find("'fluid'") != std::string::npos);
}

TEST_CASE("experiment schema errors carry paths", "[scene][experiment]") {
    SECTION("domain of the wrong type") {
        const auto parsed =
            scene::parse_experiment(R"({"version": 1, "domain": 3, "solver": {"name": "x"}})");
        REQUIRE_FALSE(parsed.has_value());
        CHECK(parsed.error().code == SceneErrorCode::wrong_type);
        CHECK(parsed.error().path == "domain");
    }
    SECTION("parameter of the wrong type") {
        const auto parsed = scene::parse_experiment(
            R"({"version": 1, "domain": "aero", "conditions": {"mach": [0.2]},
                "solver": {"name": "x"}})");
        REQUIRE_FALSE(parsed.has_value());
        CHECK(parsed.error().code == SceneErrorCode::wrong_type);
        CHECK(parsed.error().path == "conditions.mach");
    }
    SECTION("unknown key") {
        const auto parsed = scene::parse_experiment(
            R"({"version": 1, "domain": "aero", "solver": {"name": "x", "flags": 1}})");
        REQUIRE_FALSE(parsed.has_value());
        CHECK(parsed.error().code == SceneErrorCode::unknown_key);
        CHECK(parsed.error().path == "solver.flags");
    }
    SECTION("missing solver") {
        const auto parsed = scene::parse_experiment(R"({"version": 1, "domain": "entry"})");
        REQUIRE_FALSE(parsed.has_value());
        CHECK(parsed.error().code == SceneErrorCode::missing_key);
        CHECK(parsed.error().path == "solver");
    }
    SECTION("unsupported version") {
        const auto parsed = scene::parse_experiment(
            R"({"version": 2, "domain": "fluid", "solver": {"name": "x"}})");
        REQUIRE_FALSE(parsed.has_value());
        CHECK(parsed.error().code == SceneErrorCode::unsupported_version);
        CHECK(parsed.error().path == "version");
    }
}

TEST_CASE("experiments round-trip exactly", "[scene][experiment][roundtrip]") {
    scene::Experiment e;
    e.domain = scene::Domain::entry;
    e.inputs = {{"scenario", std::string("apollo_entry.json")}};
    e.conditions = {{"entry_speed_km_s", 11.0}, {"flight_path_deg", -6.5}};
    e.solver = {"ias15", "1", {{"tolerance", 1e-9}}};
    e.result = {{"peak_g", 7.123456789012345}, {"tiny", 4.9406564584124654e-324}};
    const auto parsed = scene::parse_experiment(scene::save_experiment(e));
    REQUIRE(parsed.has_value());
    CHECK(parsed.value() == e);
}
