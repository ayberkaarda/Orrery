#include <catch2/catch_test_macros.hpp>

#include <array>
#include <filesystem>
#include <string>
#include <vector>

#include "app_test_support.h"
#include "orrery/sim/space_experiment.h"

using namespace orrery;
using namespace orrery::sim;

namespace {

std::filesystem::path fixture(const char* name) {
    return app_test::test_source_dir() / "fixtures" / name;
}

bool contains(const std::string& text, const char* part) {
    return text.find(part) != std::string::npos;
}

} // namespace

TEST_CASE("Every bundled experiment loads", "[app][sim][experiment]") {
    for (const char* name :
         {"solar_system", "sun_earth_moon", "figure_eight", "sgr_a_s2", "black_hole"}) {
        INFO(name);
        const std::array<std::filesystem::path, 1> dirs{app_test::experiment_dir()};
        const auto path = resolve_experiment(name, dirs);
        REQUIRE(path.has_value());
        const auto experiment = load_space_experiment(path.value());
        REQUIRE(experiment.has_value());
        CHECK_FALSE(experiment.value().scene.state.bodies().empty());
        CHECK(std::filesystem::exists(experiment.value().scenario_path));
        CHECK_FALSE(validate(experiment.value().config).has_value());
    }
    const auto solar = load_space_experiment(app_test::experiment_dir() / "solar_system.json");
    REQUIRE(solar.has_value());
    CHECK(solar.value().config.integrator == SpaceIntegrator::leapfrog_kdk);
    CHECK(solar.value().config.dt_days == 0.05);
    CHECK(solar.value().config.time_warp == 1'000'000);
    CHECK(solar.value().scenario_path.filename() == "solar_system_j2000.json");
}

TEST_CASE("Experiment settings are read from conditions and solver", "[app][sim][experiment]") {
    const auto loaded = load_space_experiment(fixture("custom_space.json"));
    REQUIRE(loaded.has_value());
    const SimulationConfig& config = loaded.value().config;
    CHECK(config.integrator == SpaceIntegrator::rk4);
    CHECK(config.force == ForceChoice::newtonian);
    CHECK(config.dt_days == 0.25);
    CHECK(config.softening_au == 0.001);
    CHECK(config.time_warp == 50'000);
    CHECK(config.max_steps_per_advance == 64);
    CHECK(config.paused);
    CHECK(loaded.value().scene.state.bodies().size() == 3);
}

TEST_CASE("Invalid experiments fail with a message naming the problem", "[app][sim][experiment]") {
    const auto expect_error = [](const char* name, const char* part) {
        INFO(name);
        const auto loaded = load_space_experiment(fixture(name));
        REQUIRE_FALSE(loaded.has_value());
        INFO(loaded.error());
        CHECK(contains(loaded.error(), name));
        CHECK(contains(loaded.error(), part));
    };
    expect_error("aero_domain.json", "domain 'aero'");
    expect_error("missing_scenario.json", "no_such_scenario.json");
    expect_error("unknown_integrator.json", "unknown integrator 'euler'");
    expect_error("unknown_setting.json", "solver.settings.dt: unknown key");
    expect_error("bad_warp.json", "conditions.time_warp");
    expect_error("no_scenario.json", "inputs.scenario");
    expect_error("syntax_error.json", "syntax");
    const auto missing = load_space_experiment(fixture("not_there.json"));
    REQUIRE_FALSE(missing.has_value());
    CHECK(contains(missing.error(), "not_there.json"));
}

TEST_CASE("Experiment names resolve through the search directories", "[app][sim][experiment]") {
    const std::vector<std::filesystem::path> dirs{app_test::asset_dir() / "nowhere",
                                                  app_test::experiment_dir()};
    const auto by_name = resolve_experiment("figure_eight", dirs);
    REQUIRE(by_name.has_value());
    CHECK(by_name.value().filename() == "figure_eight.json");
    const auto with_extension = resolve_experiment("figure_eight.json", dirs);
    REQUIRE(with_extension.has_value());
    CHECK(with_extension.value() == by_name.value());
    const auto by_path = resolve_experiment(fixture("custom_space.json").string(), dirs);
    REQUIRE(by_path.has_value());
    CHECK(by_path.value() == fixture("custom_space.json"));

    const auto unknown = resolve_experiment("andromeda", dirs);
    REQUIRE_FALSE(unknown.has_value());
    CHECK(contains(unknown.error(), "andromeda"));
    CHECK(contains(unknown.error(), "nowhere"));
    CHECK_FALSE(resolve_experiment("", dirs).has_value());
}

TEST_CASE("Scenario listing is sorted and limited to JSON files", "[app][sim][experiment]") {
    const std::vector<ScenarioEntry> scenarios = list_scenarios(app_test::scenario_dir());
    REQUIRE(scenarios.size() >= 5);
    for (std::size_t i = 1; i < scenarios.size(); ++i) {
        CHECK(scenarios[i - 1].name < scenarios[i].name);
    }
    CHECK(scenarios.front().path.extension() == ".json");
    CHECK(list_scenarios(app_test::asset_dir() / "nowhere").empty());
}
